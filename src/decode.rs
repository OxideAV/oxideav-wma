//! WMA single-channel decoder-block assembler — the full §8 FIG.6
//! decoder chain for one block of one channel.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §8 draws the decoder
//! pipeline as a fixed-order chain (Thumpudi-180 FIG.6, inverse of the
//! encoder FIG.5). The load-bearing citation, with the per-stage patents
//! the trace attributes to each box:
//!
//! > ```text
//! > bitstream
//! >  → DEMUX (each process extracts its own parameters)           (US7,885,819 FIG.7)
//! >  → entropy decode (run-level → coefficients; matrix deltas)   (US6,223,162; US7,930,171)
//! >  → inverse quantize + inverse weighting                       (US7,383,180; US6,240,380)
//! >  → fill noise-substituted bands (noise generator)             (US7,383,180 mod 240)
//! >  → inverse MLT                                                (US6,029,126/380)
//! >  → overlap-add                                                (US7,383,180 overlapper/adder)
//! >  → [inverse sum-difference / multi-channel post-process]      (US7,502,743)
//! >  → PCM
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (decoder
//! >     pipeline, Thumpudi-180 FIG.6)
//!
//! ## Scope of this module
//!
//! This module is the **assembler** that wires the four decode stages
//! already landed — [`crate::spectral::SpectralDecode`] (Round 19, the
//! *entropy decode* box), [`crate::dequant::DequantStage`] (Round 18,
//! the *inverse quantize + inverse weighting* box),
//! [`crate::noisefill::NoiseFiller`] (the *fill noise-substituted bands*
//! box, decoder module 240), and [`crate::synthesis::Synthesis`]
//! (Round 16, the *inverse MLT → window → overlap-add* boxes) — into the
//! single per-channel decode path the patent's FIG.6 draws between the
//! DEMUX and the multi-channel post-process.
//!
//! Each adjacent pair of these stages is already individually
//! chain-tested (spectral→dequant in [`crate::spectral`], dequant→synthesis
//! in [`crate::dequant`] / [`crate::synthesis`]); what this module adds is
//! the **complete front-to-back single-channel chain with the noise-fill
//! step inserted in its FIG.6-fixed position** — between the inverse
//! quantizer and the inverse MLT, exactly where the §8 diagram places
//! module 240 and where both [`crate::dequant`] and [`crate::synthesis`]
//! explicitly deferred it ("the noise-substituted / truncated band fill …
//! act on the *coefficients* upstream of this stage").
//!
//! For one block of one channel the chain runs, in patent order:
//!
//! 1. **Entropy decode** — the decoded level-mode symbols and run-level
//!    `(R, L)` pairs become the `M`-coefficient integer spectral vector
//!    ([`SpectralDecode::block`]).
//! 2. **Inverse quantize + inverse weight** — `coeff_hat[k] = q[k] *
//!    Q[d(k)] * step` ([`DequantStage::block`]).
//! 3. **Noise / truncation band fill** — the `NoiseSubstituted` bands are
//!    overwritten with their energy-scaled noise patterns and the
//!    `Truncated` bands are zeroed, in place on the dequantized vector
//!    ([`NoiseFiller::fill`]); `Coded` bands stand.
//! 4. **Synthesis** — inverse MLT → synthesis window `hs(n)` →
//!    overlap-add, emitting `M` reconstructed time-domain samples
//!    ([`Synthesis::block`]), carrying the overlap-add tail across calls.
//!
//! The stage is **stateful** only through its owned [`Synthesis`] (the
//! overlap-add carry); the entropy / dequant / noise-fill stages are
//! stateless per block. [`ChannelDecoder::flush`] drains the trailing
//! overlap-add tail and [`ChannelDecoder::reset`] clears the carry at a
//! discontinuity, both delegating to the synthesis stage.
//!
//! ## What is NOT in this module
//!
//! * **Any new transform / quantization / entropy math.** This stage adds
//!   no arithmetic of its own beyond sequencing the four existing stages
//!   in the patent-fixed order; the math lives in their modules and stays
//!   the single source of truth for each step.
//! * **The bitstream reader / DEMUX.** Per §6 the codeword tables and the
//!   per-process parameter demux (US7,885,819 FIG.7) are `[GAP]`; this
//!   assembler consumes the **already-demuxed, already-decoded** per-block
//!   parameters (the partition, the level/run-level symbols, the band
//!   weights, the step size, the band plan, and the noise patterns),
//!   exactly as each underlying stage does.
//! * **The multi-channel post-process.** The `[inverse sum-difference]`
//!   box (US7,502,743) operates *across* two per-channel decode chains and
//!   is already assembled in [`crate::stereo_synthesis`]; this module is
//!   the single-channel chain that feeds it.
//! * **Block-size-transition frames.** The owned [`Synthesis`] carries one
//!   uniform [`crate::block::BlockSize`] `M`; adjacent blocks of different
//!   patent-disclosed sizes (§2) need transition handling whose shape is
//!   `[GAP]`, the same deferral [`crate::synthesis`] records.

use crate::block::BlockSize;
use crate::dequant::{DequantStage, InvalidDequant};
use crate::noisefill::{InvalidNoiseFill, NoiseFiller};
use crate::runlevel::RunLevelPair;
use crate::spectral::{SpectralDecode, SpectralError};
use crate::synthesis::{InvalidCoeffLen, Synthesis};

/// Stateful single-channel decoder-block assembler for one uniform
/// [`BlockSize`] `M`, per §8 of the patent trace (Thumpudi-180 FIG.6:
/// entropy decode → inverse quantize/weight → noise-fill → inverse MLT →
/// window → overlap-add).
///
/// One [`ChannelDecoder::block`] call consumes one block's already-decoded
/// per-block parameters and emits `M` reconstructed time-domain samples,
/// carrying the overlap-add tail across calls.
///
/// The decode stages are held as owned fields so the four boxes are wired
/// in one place; only the [`Synthesis`] field carries state (the
/// overlap-add tail). Construct the parts independently (each validates its
/// own block-size / band-count / length invariants), then assemble them
/// here — the constructor cross-checks that they all agree on the same
/// `M`.
#[derive(Debug, Clone)]
pub struct ChannelDecoder {
    spectral: SpectralDecode,
    dequant: DequantStage,
    noise: NoiseFiller,
    synthesis: Synthesis,
}

impl ChannelDecoder {
    /// Assemble the four decode stages into one single-channel chain.
    ///
    /// All four stages must describe the same block: the spectral
    /// partition's `total_coeffs`, the dequant stage's `M`, the noise
    /// filler's `total_coeffs`, and the synthesis stage's `M` must all be
    /// equal to one another (and to the synthesis stage's `BlockSize`
    /// sample count). The constructor cross-checks every pair so the
    /// per-stage length contracts cannot fail at decode time.
    ///
    /// # Errors
    ///
    /// Returns [`AssemblyError::CoeffCountMismatch`] naming the two stages
    /// whose coefficient counts disagree (checked spectral↔dequant,
    /// dequant↔noise, noise↔synthesis in turn). A single mismatch is
    /// enough to reject; the first disagreeing pair is reported.
    pub fn new(
        spectral: SpectralDecode,
        dequant: DequantStage,
        noise: NoiseFiller,
        synthesis: Synthesis,
    ) -> Result<Self, AssemblyError> {
        let m_spectral = spectral.total_coeffs() as usize;
        let m_dequant = dequant.block_len();
        let m_noise = noise.total_coeffs();
        let m_synthesis = synthesis.block_len();

        if m_spectral != m_dequant {
            return Err(AssemblyError::CoeffCountMismatch {
                stage_a: Stage::Spectral,
                count_a: m_spectral,
                stage_b: Stage::Dequant,
                count_b: m_dequant,
            });
        }
        if m_dequant != m_noise {
            return Err(AssemblyError::CoeffCountMismatch {
                stage_a: Stage::Dequant,
                count_a: m_dequant,
                stage_b: Stage::NoiseFill,
                count_b: m_noise,
            });
        }
        if m_noise != m_synthesis {
            return Err(AssemblyError::CoeffCountMismatch {
                stage_a: Stage::NoiseFill,
                count_a: m_noise,
                stage_b: Stage::Synthesis,
                count_b: m_synthesis,
            });
        }

        Ok(Self {
            spectral,
            dequant,
            noise,
            synthesis,
        })
    }

    /// Block size `M` for this decoder (the per-call output sample count
    /// and the coefficient count every stage shares).
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.synthesis.block_size()
    }

    /// `M`, the per-call reconstructed-sample output length (equal to the
    /// shared per-stage coefficient count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.synthesis.block_len()
    }

    /// The spectral (entropy-decode) stage.
    #[inline]
    pub const fn spectral(&self) -> &SpectralDecode {
        &self.spectral
    }

    /// The dequantization stage.
    #[inline]
    pub const fn dequant(&self) -> &DequantStage {
        &self.dequant
    }

    /// The noise-substitution / truncation band filler.
    #[inline]
    pub const fn noise(&self) -> &NoiseFiller {
        &self.noise
    }

    /// The time-domain synthesis stage (also the carrier of the
    /// overlap-add state).
    #[inline]
    pub const fn synthesis(&self) -> &Synthesis {
        &self.synthesis
    }

    /// Decode one block of one channel into `M` reconstructed time-domain
    /// samples, running the full §8 FIG.6 chain in patent order.
    ///
    /// The arguments are the already-demuxed, already-decoded per-block
    /// parameters of the four stages, in chain order:
    ///
    /// * `levels` — the level-mode head symbols for the spectral stage
    ///   (length `split`, the entropy stage's partition boundary).
    /// * `pairs` — the run-level `(R, L)` pairs for the high-frequency
    ///   tail of the spectral stage.
    /// * `patterns` — one entry per band for the noise filler, indexed in
    ///   lockstep with the band plan; the entries for non-noise bands are
    ///   ignored (an empty slice is fine for those).
    ///
    /// The fixed per-block weights `Q[d]` and overall step size were folded
    /// into the [`DequantStage`] at its construction, so they are not
    /// repeated here; the band plan and layout were fixed into the
    /// [`NoiseFiller`] likewise.
    ///
    /// # Errors
    ///
    /// * [`DecodeError::Spectral`] if the entropy stage rejects the
    ///   level/run-level symbols (see [`SpectralError`]).
    /// * [`DecodeError::Dequant`] if the inverse-quantize stage rejects the
    ///   coefficient block (see [`InvalidDequant`]) — by construction the
    ///   length always matches, so this surfaces only a future invariant
    ///   break, never a length mismatch the constructor already ruled out.
    /// * [`DecodeError::NoiseFill`] if the noise filler rejects the
    ///   patterns (band count or per-band pattern length; see
    ///   [`InvalidNoiseFill`]).
    /// * [`DecodeError::Synthesis`] if the synthesis stage rejects the
    ///   coefficient length (see [`InvalidCoeffLen`]) — also ruled out by
    ///   the constructor's cross-check, surfaced for completeness.
    ///
    /// On any error the overlap-add carry is left unchanged (the synthesis
    /// stage is the last step and is reached only after the earlier stages
    /// succeed; the noise-fill step never mutates the synthesis carry).
    pub fn block(
        &mut self,
        levels: &[i32],
        pairs: &[RunLevelPair],
        patterns: &[&[f64]],
    ) -> Result<Vec<f64>, DecodeError> {
        // 1. Entropy decode: per-mode symbols -> M integer coefficients.
        let q = self
            .spectral
            .block(levels, pairs)
            .map_err(DecodeError::Spectral)?;

        // 2. Inverse quantize + inverse weight: M integers -> M reals.
        let mut coeffs = self.dequant.block(&q).map_err(DecodeError::Dequant)?;

        // 3. Noise / truncation band fill (decoder module 240), in place:
        //    NoiseSubstituted bands overwritten at their energy, Truncated
        //    bands zeroed, Coded bands untouched.
        self.noise
            .fill(&mut coeffs, patterns)
            .map_err(DecodeError::NoiseFill)?;

        // 4. Synthesis: inverse MLT -> hs(n) window -> overlap-add,
        //    emitting M reconstructed time-domain samples and carrying the
        //    overlap-add tail across calls.
        self.synthesis
            .block(&coeffs)
            .map_err(DecodeError::Synthesis)
    }

    /// Drain the trailing-edge overlap-add tail after the last block,
    /// returning the final `M` reconstructed samples and zeroing the
    /// carry. Delegates to [`Synthesis::flush`].
    pub fn flush(&mut self) -> Vec<f64> {
        self.synthesis.flush()
    }

    /// Clear the overlap-add carry at a discontinuity (seek / decoder
    /// flush). Delegates to [`Synthesis::reset`].
    pub fn reset(&mut self) {
        self.synthesis.reset();
    }
}

/// Names the four decode stages this module assembles, for error
/// reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Stage {
    /// The entropy-decode stage ([`SpectralDecode`]).
    Spectral,
    /// The inverse-quantize + inverse-weight stage ([`DequantStage`]).
    Dequant,
    /// The noise-substitution / truncation band-fill stage
    /// ([`NoiseFiller`]).
    NoiseFill,
    /// The time-domain synthesis stage ([`Synthesis`]).
    Synthesis,
}

impl core::fmt::Display for Stage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Stage::Spectral => "spectral (entropy decode)",
            Stage::Dequant => "dequant (inverse quantize/weight)",
            Stage::NoiseFill => "noise-fill (band substitution/truncation)",
            Stage::Synthesis => "synthesis (inverse MLT/overlap-add)",
        })
    }
}

/// Rejection reason for [`ChannelDecoder::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AssemblyError {
    /// Two adjacent stages disagree on the block's coefficient count, so
    /// the chain's per-stage length contracts could not all hold. The two
    /// disagreeing stages and their declared counts are reported.
    CoeffCountMismatch {
        /// The earlier (upstream) stage in the disagreeing pair.
        stage_a: Stage,
        /// Coefficient count `stage_a` declared.
        count_a: usize,
        /// The later (downstream) stage in the disagreeing pair.
        stage_b: Stage,
        /// Coefficient count `stage_b` declared.
        count_b: usize,
    },
}

impl core::fmt::Display for AssemblyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AssemblyError::CoeffCountMismatch {
                stage_a,
                count_a,
                stage_b,
                count_b,
            } => write!(
                f,
                "oxideav-wma::decode: stage {stage_a} declares {count_a} coefficients but stage {stage_b} declares {count_b}",
            ),
        }
    }
}

impl std::error::Error for AssemblyError {}

/// Failure mode for [`ChannelDecoder::block`]; wraps the rejecting stage's
/// own error so the failing step is unambiguous.
#[derive(Debug, Clone, PartialEq)]
pub enum DecodeError {
    /// The entropy-decode stage rejected the level/run-level symbols.
    Spectral(SpectralError),
    /// The inverse-quantize stage rejected the coefficient block.
    Dequant(InvalidDequant),
    /// The noise-fill stage rejected the patterns.
    NoiseFill(InvalidNoiseFill),
    /// The synthesis stage rejected the coefficient length.
    Synthesis(InvalidCoeffLen),
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DecodeError::Spectral(e) => {
                write!(f, "oxideav-wma::decode: entropy decode failed: {e}")
            }
            DecodeError::Dequant(e) => {
                write!(f, "oxideav-wma::decode: inverse quantize failed: {e}")
            }
            DecodeError::NoiseFill(e) => write!(f, "oxideav-wma::decode: noise fill failed: {e}"),
            DecodeError::Synthesis(e) => write!(f, "oxideav-wma::decode: synthesis failed: {e}"),
        }
    }
}

impl std::error::Error for DecodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            DecodeError::Spectral(e) => Some(e),
            DecodeError::Dequant(e) => Some(e),
            DecodeError::NoiseFill(e) => Some(e),
            DecodeError::Synthesis(e) => Some(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bands::{BandPlan, BandPolicy};
    use crate::entropy_mode::Partition;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::step_size::OverallStepSize;
    use crate::window::WindowPair;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    /// Build a single-band layout that tiles a whole block.
    fn single_band_layout(bs: BlockSize) -> QuantBandLayout {
        QuantBandLayout::for_block(vec![QuantBand::new(0, bs.samples(), 0).unwrap()], bs).unwrap()
    }

    /// Build an all-coded single-band decoder for `bs` with whole-block
    /// run-level entropy (split == 0), unit weight, unit step.
    fn coded_decoder(bs: BlockSize) -> ChannelDecoder {
        let m = bs.samples();
        let spectral = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let layout = single_band_layout(bs);
        let dequant =
            DequantStage::new(bs, &layout, &[1.0_f64], OverallStepSize::new(1.0).unwrap()).unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap()
    }

    // ---------- construction / accessors ----------

    #[test]
    fn new_accepts_consistent_stages_and_accessors_agree() {
        let bs = BlockSize::from_samples(256).unwrap();
        let dec = coded_decoder(bs);
        assert_eq!(dec.block_size(), bs);
        assert_eq!(dec.block_len(), 256);
        assert_eq!(dec.spectral().total_coeffs(), 256);
        assert_eq!(dec.dequant().block_len(), 256);
        assert_eq!(dec.noise().total_coeffs(), 256);
        assert_eq!(dec.synthesis().block_len(), 256);
    }

    #[test]
    fn new_rejects_spectral_dequant_mismatch() {
        // spectral over 512 coeffs, every other stage over 256.
        let bs = BlockSize::from_samples(256).unwrap();
        let spectral = SpectralDecode::new(Partition::new(512, 0, false).unwrap());
        let layout = single_band_layout(bs);
        let dequant =
            DequantStage::new(bs, &layout, &[1.0_f64], OverallStepSize::new(1.0).unwrap()).unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let err = ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap_err();
        assert_eq!(
            err,
            AssemblyError::CoeffCountMismatch {
                stage_a: Stage::Spectral,
                count_a: 512,
                stage_b: Stage::Dequant,
                count_b: 256,
            }
        );
    }

    #[test]
    fn new_rejects_noise_synthesis_mismatch() {
        // spectral + dequant + noise over 256, synthesis over 512.
        let small = BlockSize::from_samples(256).unwrap();
        let big = BlockSize::from_samples(512).unwrap();
        let spectral = SpectralDecode::new(Partition::new(256, 0, false).unwrap());
        let layout = single_band_layout(small);
        let dequant = DequantStage::new(
            small,
            &layout,
            &[1.0_f64],
            OverallStepSize::new(1.0).unwrap(),
        )
        .unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(big, WindowPair::orthogonal_sine(big)).unwrap();
        let err = ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap_err();
        assert_eq!(
            err,
            AssemblyError::CoeffCountMismatch {
                stage_a: Stage::NoiseFill,
                count_a: 256,
                stage_b: Stage::Synthesis,
                count_b: 512,
            }
        );
    }

    // ---------- full-chain decode ----------

    #[test]
    fn block_runs_full_chain_matching_hand_wired_stages() {
        // Decode one block through the assembler and through the four
        // stages by hand, and assert byte-for-byte identical output. This
        // pins that the assembler adds no arithmetic of its own and wires
        // the stages in the patent-fixed order.
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;

        let mut dec = coded_decoder(bs);
        let pairs = [pair(255, 3)]; // a single non-zero at the last coeff.
        let empty: &[f64] = &[];
        let got = dec.block(&[], &pairs, &[empty]).unwrap();

        // Hand-wired reference: same stages, same inputs, no noise-fill
        // effect (single Coded band), so the coefficient vector is the
        // dequant output unchanged.
        let spectral = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let layout = single_band_layout(bs);
        let dequant =
            DequantStage::new(bs, &layout, &[1.0_f64], OverallStepSize::new(1.0).unwrap()).unwrap();
        let mut synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let q = spectral.block(&[], &pairs).unwrap();
        let coeffs = dequant.block(&q).unwrap();
        let expect = synthesis.block(&coeffs).unwrap();

        assert_eq!(got, expect);
        assert_eq!(got.len(), m);
    }

    #[test]
    fn noise_fill_step_is_applied_between_dequant_and_synthesis() {
        // Two bands: band 0 Coded, band 1 NoiseSubstituted at a fixed
        // energy. The decoded coefficients in band 1 must be replaced by
        // the energy-scaled noise pattern *before* the inverse MLT, so the
        // assembler output must equal the hand-wired chain that fills the
        // band between dequant and synthesis — and must differ from a
        // chain that skips the fill.
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let half = (m / 2) as u16;

        let spectral = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let layout = QuantBandLayout::for_block(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, half, 1).unwrap(),
            ],
            bs,
        )
        .unwrap();
        let dequant = DequantStage::new(
            bs,
            &layout,
            &[1.0_f64, 1.0_f64],
            OverallStepSize::new(1.0).unwrap(),
        )
        .unwrap();
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 5.0 },
        ]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let mut dec = ChannelDecoder::new(spectral, dequant, noise.clone(), synthesis).unwrap();

        // Put one non-zero in band 0 (coded, kept) and one in band 1
        // (noise, will be overwritten). A run-level pair `(R, L)` consumes
        // `R + 1` coefficients (R zeros + one level-L non-zero), so the
        // two pairs must consume exactly `m`: (1,4) → 2 coeffs (idx 1),
        // (m-3, 7) → m-2 coeffs (idx m-1), total m.
        let pairs = [pair(1, 4), pair((m - 3) as u32, 7)];
        let band1_pattern: Vec<f64> = (0..half).map(|i| i as f64 + 1.0).collect();
        let p1: &[f64] = &band1_pattern;
        let empty: &[f64] = &[];
        let got = dec.block(&[], &pairs, &[empty, p1]).unwrap();

        // Hand-wired chain WITH the fill applied between dequant and
        // synthesis.
        let spectral_h = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let layout_h = QuantBandLayout::for_block(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, half, 1).unwrap(),
            ],
            bs,
        )
        .unwrap();
        let dequant_h = DequantStage::new(
            bs,
            &layout_h,
            &[1.0_f64, 1.0_f64],
            OverallStepSize::new(1.0).unwrap(),
        )
        .unwrap();
        let mut synthesis_h = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let q = spectral_h.block(&[], &pairs).unwrap();
        let mut coeffs = dequant_h.block(&q).unwrap();
        noise.fill(&mut coeffs, &[empty, p1]).unwrap();
        let expect = synthesis_h.block(&coeffs).unwrap();
        assert_eq!(got, expect);

        // And the fill genuinely changed the band: a chain that SKIPS the
        // fill produces a different output (otherwise the test proves
        // nothing about ordering).
        let mut synthesis_skip = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let coeffs_skip = dequant_h.block(&q).unwrap();
        let without_fill = synthesis_skip.block(&coeffs_skip).unwrap();
        assert_ne!(got, without_fill);
    }

    #[test]
    fn truncated_band_is_zeroed_before_synthesis() {
        // band 1 Truncated → its dequantized coefficients must be zeroed
        // before the inverse MLT. Output equals the hand-wired chain that
        // zeroes the band.
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let half = (m / 2) as u16;

        let spectral = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let bands = vec![
            QuantBand::new(0, half, 0).unwrap(),
            QuantBand::new(half, half, 1).unwrap(),
        ];
        let layout = QuantBandLayout::for_block(bands.clone(), bs).unwrap();
        let dequant = DequantStage::new(
            bs,
            &layout,
            &[1.0_f64, 1.0_f64],
            OverallStepSize::new(1.0).unwrap(),
        )
        .unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded, BandPolicy::Truncated]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let mut dec = ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap();

        // A non-zero coefficient lands in band 1; truncation must erase it.
        let pairs = [pair((m - 1) as u32, 9)];
        let empty: &[f64] = &[];
        let got = dec.block(&[], &pairs, &[empty, empty]).unwrap();

        // Hand-wired with band 1 zeroed → equals decoding an all-zero
        // band-1 coefficient set (the truncated band carries no energy).
        let layout_h = QuantBandLayout::for_block(bands, bs).unwrap();
        let dequant_h = DequantStage::new(
            bs,
            &layout_h,
            &[1.0_f64, 1.0_f64],
            OverallStepSize::new(1.0).unwrap(),
        )
        .unwrap();
        let mut synthesis_h = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        // The only non-zero was in band 1, which is truncated → all zeros.
        let zeros = vec![0.0_f64; m];
        let _ = dequant_h; // dequant of all-zero ints is all-zero reals.
        let expect = synthesis_h.block(&zeros).unwrap();
        assert_eq!(got, expect);
    }

    // ---------- state: overlap-add carry across blocks ----------

    #[test]
    fn carry_persists_across_blocks_and_flush_drains_tail() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let mut dec = coded_decoder(bs);
        let empty: &[f64] = &[];

        // First block: a single non-zero. The overlap-add tail it leaves
        // must influence the SECOND block's output, so the second block of
        // a fresh decoder differs from the second block of this one.
        let p = [pair(255, 5)]; // (255,5) consumes exactly 256 coeffs
        let _b0 = dec.block(&[], &p, &[empty]).unwrap();
        let b1 = dec.block(&[], &p, &[empty]).unwrap();

        // Fresh decoder, only the second-block input: no carry from a
        // first block.
        let mut fresh = coded_decoder(bs);
        let b1_fresh = fresh.block(&[], &p, &[empty]).unwrap();
        assert_ne!(b1, b1_fresh, "carry from block 0 must affect block 1");

        // Flush drains a final M-sample tail.
        let tail = dec.flush();
        assert_eq!(tail.len(), m);
    }

    #[test]
    fn reset_clears_carry_to_fresh_state() {
        let bs = BlockSize::from_samples(256).unwrap();
        let mut dec = coded_decoder(bs);
        let empty: &[f64] = &[];
        let p = [pair(255, 5)]; // (255,5) consumes exactly 256 coeffs

        let _ = dec.block(&[], &p, &[empty]).unwrap();
        dec.reset();
        let after_reset = dec.block(&[], &p, &[empty]).unwrap();

        let mut fresh = coded_decoder(bs);
        let fresh_first = fresh.block(&[], &p, &[empty]).unwrap();
        assert_eq!(after_reset, fresh_first);
    }

    // ---------- error propagation ----------

    #[test]
    fn block_propagates_spectral_error() {
        let bs = BlockSize::from_samples(256).unwrap();
        let mut dec = coded_decoder(bs);
        // split == 0, so any non-empty `levels` is a length mismatch.
        let empty: &[f64] = &[];
        let err = dec
            .block(&[1, 2, 3], &[pair(256, 1)], &[empty])
            .unwrap_err();
        assert!(matches!(err, DecodeError::Spectral(_)));
    }

    #[test]
    fn block_propagates_noise_fill_error() {
        // A NoiseSubstituted band needs a pattern of the band's length; a
        // wrong-length pattern surfaces as a NoiseFill error.
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples();
        let spectral = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let layout = single_band_layout(bs);
        let dequant =
            DequantStage::new(bs, &layout, &[1.0_f64], OverallStepSize::new(1.0).unwrap()).unwrap();
        let plan = BandPlan::new(vec![BandPolicy::NoiseSubstituted { energy: 1.0 }]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let mut dec = ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap();

        let short = [1.0_f64, 2.0]; // band needs `m`, supplied 2.
        let p: &[f64] = &short;
        // A whole-block run-level fill that the spectral stage accepts, so
        // the chain reaches the noise-fill step and the pattern-length
        // mismatch is the surfacing error.
        let err = dec.block(&[], &[pair(m as u32 - 1, 1)], &[p]).unwrap_err();
        assert!(matches!(err, DecodeError::NoiseFill(_)));
    }

    // ---------- error Display / source ----------

    #[test]
    fn assembly_error_display_names_both_stages() {
        let e = AssemblyError::CoeffCountMismatch {
            stage_a: Stage::Spectral,
            count_a: 512,
            stage_b: Stage::Dequant,
            count_b: 256,
        };
        let s = format!("{e}");
        assert!(s.contains("entropy decode"));
        assert!(s.contains("inverse quantize"));
        assert!(s.contains("512") && s.contains("256"));
    }

    #[test]
    fn decode_error_display_and_source() {
        let e = DecodeError::Spectral(SpectralError::LevelLenMismatch {
            expected: 3,
            got: 2,
        });
        assert!(format!("{e}").contains("entropy decode failed"));
        assert!(std::error::Error::source(&e).is_some());
    }

    #[test]
    fn stage_display_covers_all_four() {
        for st in [
            Stage::Spectral,
            Stage::Dequant,
            Stage::NoiseFill,
            Stage::Synthesis,
        ] {
            assert!(!format!("{st}").is_empty());
        }
    }

    // ---------- every block size ----------

    #[test]
    fn decodes_every_patent_block_size() {
        for bs in BlockSize::ALL {
            let m = bs.samples() as usize;
            let mut dec = coded_decoder(bs);
            let empty: &[f64] = &[];
            // a single non-zero at the final coefficient.
            let out = dec
                .block(&[], &[pair((m - 1) as u32, 2)], &[empty])
                .unwrap();
            assert_eq!(out.len(), m);
            assert!(out.iter().all(|v| v.is_finite()));
        }
    }
}

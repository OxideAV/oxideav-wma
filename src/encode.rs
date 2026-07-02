//! WMA full single-channel encoder-block chain.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §8 draws the encoder
//! pipeline this module assembles (Thumpudi-180 FIG.5, WMA7-relevant
//! subset):
//!
//! > ```text
//! > PCM
//! >  → partition into variable-size blocks {256,512,1024,2048,4096}
//! >  → MLT (per block)
//! >  → uniform scalar quantize (matrix weight × overall step)
//! >  → entropy code:
//! >       • coefficients: run-level (R,L), mode-switched, escape
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (encoder
//! >     pipeline, Thumpudi-180 FIG.5)
//!
//! ## Scope of this module
//!
//! This module is the single-channel **encoder assembler**, the
//! forward mirror of [`crate::decode::ChannelDecoder`]: it wires the
//! three encode stages already landed —
//! [`crate::analysis::Analysis`] (window + forward MLT),
//! [`crate::quant::QuantStage`] (forward uniform scalar quantizer),
//! and [`crate::spectral::SpectralEncode`] (level head + run-level
//! tail) — into one stateful [`ChannelEncoder`] mapping `M` fresh
//! time-domain samples per block to the per-block entropy symbols the
//! paired decoder chain consumes. The constructor cross-checks all
//! three stages share one coefficient count `M`, exactly as
//! [`crate::decode::ChannelDecoder::new`] does for its four.
//!
//! An encoder/decoder pair built from **the same parameter set**
//! (partition, band layout, weights, step, window pair, block size)
//! round-trips: decode(encode(PCM)) reproduces the PCM after the
//! chain's `M`-sample leading latency, within the §4 uniform
//! quantizer's `divisor / 2` per-coefficient error bound — the
//! cross-module tests pin both.
//!
//! ## What is NOT in this module
//!
//! * **The perceptual model / parameter selection.** The weighting
//!   matrix (§4 excitation / Bark masking), the overall step size
//!   (rate control), and the partition boundary (mode selector
//!   tuning) are all encoder analysis whose tuned rules are `[GAP]` or
//!   encoder-side-only; they arrive here already folded into the three
//!   stages. [`crate::excitation`] and
//!   [`crate::spectral::SpectralEncode::min_split_for`] are the
//!   analysis helpers a caller derives them with.
//! * **Noise substitution / band truncation decisions.** §7's
//!   encoder-side selection ("use noise substitution to convey
//!   information in certain bands") is a rate/quality decision; this
//!   chain literal-codes every band (the [`crate::bands::BandPolicy::Coded`]
//!   default). The paired decoder runs an all-coded
//!   [`crate::bands::BandPlan`].
//! * **The MUX / codeword tables.** Emitted symbols are typed values,
//!   not bits; the codeword tables and the per-process MUX byte layout
//!   are `[GAP]` per §6/§9 of the trace.

use crate::analysis::{Analysis, InvalidSampleLen};
use crate::frame::BlockParams;
use crate::quant::{InvalidQuant, QuantStage};
use crate::runlevel::RunLevelPair;
use crate::spectral::{SpectralEncode, SpectralEncodeError};

/// One encoded block's per-mode entropy symbols — the encoder-side
/// product [`crate::decode::ChannelDecoder::block`] consumes as its
/// `(levels, pairs)` arguments.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncodedBlock {
    /// Level-mode head symbols (`split` entries, already signed).
    pub levels: Vec<i32>,
    /// Run-level tail pairs, implicit `(N, 1)` terminator included
    /// when the tail has trailing zeros.
    pub pairs: Vec<RunLevelPair>,
}

impl EncodedBlock {
    /// Repackage as the owned per-block parameter set the
    /// [`crate::frame`] decoders consume. `band_count` is the paired
    /// decoder's band count; every band is literal-coded by this
    /// chain, so each gets the empty ignored pattern
    /// ([`crate::noisefill::NoiseFiller::fill`] lockstep contract).
    pub fn into_block_params(self, band_count: usize) -> BlockParams {
        BlockParams::new(self.levels, self.pairs, vec![Vec::new(); band_count])
    }
}

/// Stateful single-channel encoder-block chain for one uniform block
/// size `M`, per §8 of the patent trace (Thumpudi-180 FIG.5): window +
/// forward MLT → uniform scalar quantize → run-level entropy code.
///
/// The forward mirror of [`crate::decode::ChannelDecoder`]. Only the
/// [`Analysis`] field carries state (the 50%-overlap frame buffer).
#[derive(Debug, Clone)]
pub struct ChannelEncoder {
    analysis: Analysis,
    quant: QuantStage,
    spectral: SpectralEncode,
}

impl ChannelEncoder {
    /// Assemble the three encode stages into one single-channel chain.
    ///
    /// All three must describe the same block: the analysis stage's
    /// `M`, the quantizer's `M`, and the spectral partition's
    /// `total_coeffs` must be equal — the same cross-check
    /// [`crate::decode::ChannelDecoder::new`] applies to the decode
    /// stages.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeAssemblyError::CoeffCountMismatch`] naming the
    /// first disagreeing stage pair (checked analysis↔quant then
    /// quant↔spectral).
    pub fn new(
        analysis: Analysis,
        quant: QuantStage,
        spectral: SpectralEncode,
    ) -> Result<Self, EncodeAssemblyError> {
        let m_analysis = analysis.block_len();
        let m_quant = quant.block_len();
        let m_spectral = spectral.total_coeffs() as usize;

        if m_analysis != m_quant {
            return Err(EncodeAssemblyError::CoeffCountMismatch {
                stage_a: EncodeStage::Analysis,
                count_a: m_analysis,
                stage_b: EncodeStage::Quant,
                count_b: m_quant,
            });
        }
        if m_quant != m_spectral {
            return Err(EncodeAssemblyError::CoeffCountMismatch {
                stage_a: EncodeStage::Quant,
                count_a: m_quant,
                stage_b: EncodeStage::Spectral,
                count_b: m_spectral,
            });
        }

        Ok(Self {
            analysis,
            quant,
            spectral,
        })
    }

    /// Block size `M` for this encoder (the per-call fresh-sample
    /// input count every stage shares).
    #[inline]
    pub const fn block_size(&self) -> crate::block::BlockSize {
        self.analysis.block_size()
    }

    /// `M`, the per-call fresh-sample input length (equal to the
    /// shared per-stage coefficient count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.analysis.block_len()
    }

    /// The time-domain analysis stage (also the carrier of the
    /// 50%-overlap frame buffer).
    #[inline]
    pub const fn analysis(&self) -> &Analysis {
        &self.analysis
    }

    /// The forward quantization stage.
    #[inline]
    pub const fn quant(&self) -> &QuantStage {
        &self.quant
    }

    /// The spectral (entropy-encode) stage.
    #[inline]
    pub const fn spectral(&self) -> &SpectralEncode {
        &self.spectral
    }

    /// Encode one block of one channel: consume `M` fresh time-domain
    /// samples, emit the block's per-mode entropy symbols, running the
    /// §8 FIG.5 chain in patent order (analysis → quantize → entropy
    /// code).
    ///
    /// # Errors
    ///
    /// * [`EncodeError::Analysis`] if the sample count is not `M`.
    /// * [`EncodeError::Quant`] if the quantizer rejects the
    ///   coefficient block — ruled out by the constructor's
    ///   cross-check, surfaced for completeness.
    /// * [`EncodeError::Spectral`] if the entropy stage rejects the
    ///   quantized block: a negative coefficient landed in the
    ///   run-level tail, or a tail non-zero has no preceding zero.
    ///   Both mean the caller-supplied partition boundary sits below
    ///   this block's structural floor
    ///   ([`SpectralEncode::min_split_for`]); the analysis frame
    ///   buffer has already advanced when this surfaces, matching the
    ///   stage order.
    pub fn block(&mut self, samples: &[f64]) -> Result<EncodedBlock, EncodeError> {
        // 1. Analysis: M fresh samples -> M MLT coefficients.
        let coeffs = self
            .analysis
            .block(samples)
            .map_err(EncodeError::Analysis)?;

        // 2. Forward quantize: M reals -> M integers.
        let q = self.quant.block(&coeffs).map_err(EncodeError::Quant)?;

        // 3. Entropy encode: M integers -> level head + run-level tail.
        let (levels, pairs) = self.spectral.block(&q).map_err(EncodeError::Spectral)?;

        Ok(EncodedBlock { levels, pairs })
    }

    /// Close the stream: encode the final all-zero block that carries
    /// the last real block's trailing frame half
    /// ([`Analysis::flush`]), returning its entropy symbols. Feeding
    /// this block to the paired decoder drains the last `M` real
    /// samples out of its overlap-add carry.
    ///
    /// # Errors
    ///
    /// Same as [`ChannelEncoder::block`] for the quant/spectral stages
    /// (the flush block's samples are all zero, so in practice only a
    /// partition below the structural floor of the *windowed previous
    /// block* can reject).
    pub fn flush(&mut self) -> Result<EncodedBlock, EncodeError> {
        let coeffs = self.analysis.flush();
        let q = self.quant.block(&coeffs).map_err(EncodeError::Quant)?;
        let (levels, pairs) = self.spectral.block(&q).map_err(EncodeError::Spectral)?;
        Ok(EncodedBlock { levels, pairs })
    }

    /// Clear the analysis frame buffer at a discontinuity, so the next
    /// [`ChannelEncoder::block`] behaves as if freshly constructed.
    pub fn reset(&mut self) {
        self.analysis.reset();
    }
}

/// Names the three encode stages this module assembles, for error
/// reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EncodeStage {
    /// The time-domain analysis stage (window + forward MLT).
    Analysis,
    /// The forward quantization stage.
    Quant,
    /// The spectral entropy-encode stage.
    Spectral,
}

impl core::fmt::Display for EncodeStage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            EncodeStage::Analysis => "analysis",
            EncodeStage::Quant => "quant",
            EncodeStage::Spectral => "spectral",
        })
    }
}

/// Constructor rejection for [`ChannelEncoder::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EncodeAssemblyError {
    /// Two stages disagree on the block's coefficient count `M`.
    CoeffCountMismatch {
        /// First stage of the disagreeing pair.
        stage_a: EncodeStage,
        /// Its coefficient count.
        count_a: usize,
        /// Second stage of the disagreeing pair.
        stage_b: EncodeStage,
        /// Its coefficient count.
        count_b: usize,
    },
}

impl core::fmt::Display for EncodeAssemblyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EncodeAssemblyError::CoeffCountMismatch {
                stage_a,
                count_a,
                stage_b,
                count_b,
            } => write!(
                f,
                "oxideav-wma::encode: {stage_a} stage covers {count_a} coefficients but {stage_b} stage covers {count_b}",
            ),
        }
    }
}

impl std::error::Error for EncodeAssemblyError {}

/// Per-stage failure from [`ChannelEncoder::block`] /
/// [`ChannelEncoder::flush`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EncodeError {
    /// The analysis stage rejected the fresh-sample slice.
    Analysis(InvalidSampleLen),
    /// The forward quantizer rejected the coefficient block.
    Quant(InvalidQuant),
    /// The entropy stage rejected the quantized block.
    Spectral(SpectralEncodeError),
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EncodeError::Analysis(e) => write!(f, "oxideav-wma::encode: analysis stage: {e}"),
            EncodeError::Quant(e) => write!(f, "oxideav-wma::encode: quant stage: {e}"),
            EncodeError::Spectral(e) => write!(f, "oxideav-wma::encode: spectral stage: {e}"),
        }
    }
}

impl std::error::Error for EncodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            EncodeError::Analysis(e) => Some(e),
            EncodeError::Quant(e) => Some(e),
            EncodeError::Spectral(e) => Some(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bands::{BandPlan, BandPolicy};
    use crate::block::BlockSize;
    use crate::decode::ChannelDecoder;
    use crate::dequant::DequantStage;
    use crate::entropy_mode::Partition;
    use crate::noisefill::NoiseFiller;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::spectral::SpectralDecode;
    use crate::step_size::OverallStepSize;
    use crate::synthesis::Synthesis;
    use crate::window::WindowPair;

    /// One shared parameter set describing an encoder/decoder pair:
    /// single band, unit weight, small step, all-level-mode partition
    /// (split == M, so arbitrary signed spectra encode).
    struct Params {
        bs: BlockSize,
        layout: QuantBandLayout,
        weights: [f64; 1],
        step: OverallStepSize,
        partition: Partition,
    }

    impl Params {
        fn new(bs: BlockSize, step: f64, split: u32) -> Self {
            let m = bs.samples() as usize;
            let band = QuantBand::new(0, m as u16, 0).unwrap();
            Self {
                bs,
                layout: QuantBandLayout::new(vec![band], m).unwrap(),
                weights: [1.0],
                step: OverallStepSize::new(step).unwrap(),
                partition: Partition::new(m as u32, split, false).unwrap(),
            }
        }

        fn encoder(&self) -> ChannelEncoder {
            let analysis = Analysis::new(self.bs, WindowPair::orthogonal_sine(self.bs)).unwrap();
            let quant = QuantStage::new(self.bs, &self.layout, &self.weights, self.step).unwrap();
            let spectral = SpectralEncode::new(self.partition);
            ChannelEncoder::new(analysis, quant, spectral).unwrap()
        }

        fn decoder(&self) -> ChannelDecoder {
            let spectral = SpectralDecode::new(self.partition);
            let dequant =
                DequantStage::new(self.bs, &self.layout, &self.weights, self.step).unwrap();
            let plan = BandPlan::new(vec![BandPolicy::Coded; self.layout.band_count()]);
            let noise = NoiseFiller::new(plan, self.layout.clone()).unwrap();
            let synthesis = Synthesis::new(self.bs, WindowPair::orthogonal_sine(self.bs)).unwrap();
            ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap()
        }
    }

    fn pseudo_random(len: usize, seed: u64) -> Vec<f64> {
        let mut state = seed.wrapping_mul(0x9E3779B97F4A7C15).wrapping_add(1);
        (0..len)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((state >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
            })
            .collect()
    }

    // ---------- construction ----------

    #[test]
    fn new_accepts_agreeing_stages() {
        let p = Params::new(BlockSize::S256, 0.001, 256);
        let enc = p.encoder();
        assert_eq!(enc.block_len(), 256);
        assert_eq!(enc.analysis().block_len(), 256);
        assert_eq!(enc.quant().block_len(), 256);
        assert_eq!(enc.spectral().total_coeffs(), 256);
    }

    #[test]
    fn new_rejects_analysis_quant_mismatch() {
        let p256 = Params::new(BlockSize::S256, 0.001, 256);
        let p512 = Params::new(BlockSize::S512, 0.001, 512);
        let analysis = Analysis::new(p512.bs, WindowPair::orthogonal_sine(p512.bs)).unwrap();
        let quant = QuantStage::new(p256.bs, &p256.layout, &p256.weights, p256.step).unwrap();
        let spectral = SpectralEncode::new(p256.partition);
        let err = ChannelEncoder::new(analysis, quant, spectral).unwrap_err();
        assert_eq!(
            err,
            EncodeAssemblyError::CoeffCountMismatch {
                stage_a: EncodeStage::Analysis,
                count_a: 512,
                stage_b: EncodeStage::Quant,
                count_b: 256,
            }
        );
    }

    #[test]
    fn new_rejects_quant_spectral_mismatch() {
        let p256 = Params::new(BlockSize::S256, 0.001, 256);
        let p512 = Params::new(BlockSize::S512, 0.001, 512);
        let analysis = Analysis::new(p256.bs, WindowPair::orthogonal_sine(p256.bs)).unwrap();
        let quant = QuantStage::new(p256.bs, &p256.layout, &p256.weights, p256.step).unwrap();
        let spectral = SpectralEncode::new(p512.partition);
        let err = ChannelEncoder::new(analysis, quant, spectral).unwrap_err();
        assert_eq!(
            err,
            EncodeAssemblyError::CoeffCountMismatch {
                stage_a: EncodeStage::Quant,
                count_a: 256,
                stage_b: EncodeStage::Spectral,
                count_b: 512,
            }
        );
    }

    // ---------- per-stage errors ----------

    #[test]
    fn block_rejects_wrong_sample_len() {
        let p = Params::new(BlockSize::S256, 0.001, 256);
        let mut enc = p.encoder();
        let err = enc.block(&[0.0; 255]).unwrap_err();
        assert_eq!(
            err,
            EncodeError::Analysis(InvalidSampleLen {
                expected: 256,
                got: 255,
            })
        );
    }

    #[test]
    fn block_surfaces_spectral_rejection_for_too_low_split() {
        // split == 0 forces the whole (dense, signed) spectrum into
        // the run-level tail, which cannot represent it.
        let p = Params::new(BlockSize::S256, 0.001, 0);
        let mut enc = p.encoder();
        let x = pseudo_random(256, 5);
        let err = enc.block(&x).unwrap_err();
        assert!(matches!(err, EncodeError::Spectral(_)), "got {err:?}");
    }

    // ---------- the chain adds no arithmetic of its own ----------

    #[test]
    fn block_equals_manual_three_stage_chain() {
        let p = Params::new(BlockSize::S256, 0.001, 256);
        let mut enc = p.encoder();

        let mut analysis = Analysis::new(p.bs, WindowPair::orthogonal_sine(p.bs)).unwrap();
        let quant = QuantStage::new(p.bs, &p.layout, &p.weights, p.step).unwrap();
        let spectral = SpectralEncode::new(p.partition);

        let x = pseudo_random(512, 17);
        for t in 0..2 {
            let block = &x[t * 256..(t + 1) * 256];
            let via_enc = enc.block(block).unwrap();
            let coeffs = analysis.block(block).unwrap();
            let q = quant.block(&coeffs).unwrap();
            let (levels, pairs) = spectral.block(&q).unwrap();
            assert_eq!(via_enc, EncodedBlock { levels, pairs }, "t={t}");
        }
    }

    // ---------- encode → decode round trip ----------

    /// decode(encode(PCM)) reproduces the PCM after the chain's
    /// M-sample leading latency, within the uniform quantizer's error
    /// bound. With unit weight and step `s`, each spectral coefficient
    /// is off by at most `s/2`; the synthesis chain is unity-gain, so
    /// a comfortable time-domain tolerance is a small multiple of `s`.
    fn assert_encode_decode_round_trip(bs: BlockSize, blocks: usize, step: f64, seed: u64) {
        let m = bs.samples() as usize;
        let p = Params::new(bs, step, m as u32);
        let mut enc = p.encoder();
        let mut dec = p.decoder();
        let band_count = p.layout.band_count();

        let signal = pseudo_random(blocks * m, seed);
        let mut output = Vec::new();
        for t in 0..blocks {
            let eb = enc.block(&signal[t * m..(t + 1) * m]).unwrap();
            let bp = eb.into_block_params(band_count);
            let patterns: Vec<&[f64]> = bp.patterns.iter().map(|p| p.as_slice()).collect();
            output.extend(dec.block(&bp.levels, &bp.pairs, &patterns).unwrap());
        }
        let eb = enc.flush().unwrap();
        let bp = eb.into_block_params(band_count);
        let patterns: Vec<&[f64]> = bp.patterns.iter().map(|p| p.as_slice()).collect();
        output.extend(dec.block(&bp.levels, &bp.pairs, &patterns).unwrap());

        assert_eq!(output.len(), (blocks + 1) * m);
        let tolerance = 4.0 * step;
        // Leading latency block ~ 0.
        for (i, &y) in output[..m].iter().enumerate() {
            assert!(y.abs() < tolerance, "bs={bs:?} leading i={i}: {y}");
        }
        // Interior reproduces the signal within the quantization bound.
        for i in 0..blocks * m {
            let err = (output[m + i] - signal[i]).abs();
            assert!(
                err < tolerance,
                "bs={bs:?} i={i}: got {} want {} (err {err})",
                output[m + i],
                signal[i],
            );
        }
    }

    #[test]
    fn encode_decode_round_trip_s256() {
        assert_encode_decode_round_trip(BlockSize::S256, 4, 1e-3, 31);
    }

    #[test]
    fn encode_decode_round_trip_s512() {
        assert_encode_decode_round_trip(BlockSize::S512, 3, 1e-3, 32);
    }

    #[test]
    fn round_trip_error_shrinks_with_the_step() {
        // Halving the quantizer step must not enlarge the worst-case
        // reconstruction error — the rate/quality dial the patents
        // describe the step as.
        let bs = BlockSize::S256;
        let m = 256usize;
        let signal = pseudo_random(2 * m, 41);

        let mut worst = Vec::new();
        for step in [1e-2, 1e-3] {
            let p = Params::new(bs, step, m as u32);
            let mut enc = p.encoder();
            let mut dec = p.decoder();
            let mut output = Vec::new();
            for t in 0..2 {
                let eb = enc.block(&signal[t * m..(t + 1) * m]).unwrap();
                output.extend(dec.block(&eb.levels, &eb.pairs, &[&[]]).unwrap());
            }
            let eb = enc.flush().unwrap();
            output.extend(dec.block(&eb.levels, &eb.pairs, &[&[]]).unwrap());
            let max_err = (0..2 * m)
                .map(|i| (output[m + i] - signal[i]).abs())
                .fold(0.0_f64, f64::max);
            worst.push(max_err);
        }
        assert!(
            worst[1] < worst[0],
            "coarse {} should exceed fine {}",
            worst[0],
            worst[1]
        );
    }

    #[test]
    fn run_level_partition_round_trips_for_sparse_spectra() {
        // A quiet low-amplitude signal quantized with a coarse step
        // yields a mostly-zero spectrum, so a low split works: use the
        // per-block structural floor and re-encode by hand through the
        // stage pair to prove the run-level branch is exercised.
        let bs = BlockSize::S256;
        let m = 256usize;
        let p = Params::new(bs, 0.05, m as u32);

        // Get one block's quantized spectrum via the front two stages.
        let mut analysis = Analysis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let quant = QuantStage::new(bs, &p.layout, &p.weights, p.step).unwrap();
        let x: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.02).sin() * 0.02).collect();
        let coeffs = analysis.block(&x).unwrap();
        let q = quant.block(&coeffs).unwrap();

        let split = SpectralEncode::min_split_for(&q);
        assert!(
            (split as usize) < m,
            "expected a non-trivial run-level tail, got split {split}"
        );
        let partition = Partition::new(m as u32, split, false).unwrap();
        let (levels, pairs) = SpectralEncode::new(partition).block(&q).unwrap();
        let back = SpectralDecode::new(partition)
            .block(&levels, &pairs)
            .unwrap();
        assert_eq!(back, q);
    }

    // ---------- flush / reset ----------

    #[test]
    fn flush_matches_zero_block_encode() {
        let p = Params::new(BlockSize::S256, 0.001, 256);
        let mut a = p.encoder();
        let mut b = p.encoder();
        let x = pseudo_random(256, 51);
        let _ = a.block(&x).unwrap();
        let _ = b.block(&x).unwrap();
        let via_flush = a.flush().unwrap();
        let via_zero = b.block(&vec![0.0; 256]).unwrap();
        assert_eq!(via_flush, via_zero);
    }

    #[test]
    fn reset_restores_fresh_behaviour() {
        let p = Params::new(BlockSize::S256, 0.001, 256);
        let mut used = p.encoder();
        let mut fresh = p.encoder();
        let x = pseudo_random(256, 52);
        let _ = used.block(&pseudo_random(256, 53)).unwrap();
        used.reset();
        assert_eq!(used.block(&x).unwrap(), fresh.block(&x).unwrap());
    }

    // ---------- EncodedBlock plumbing ----------

    #[test]
    fn into_block_params_carries_symbols_and_empty_patterns() {
        let eb = EncodedBlock {
            levels: vec![1, -2, 3],
            pairs: vec![RunLevelPair::new(1, 4).unwrap()],
        };
        let bp = eb.clone().into_block_params(3);
        assert_eq!(bp.levels, eb.levels);
        assert_eq!(bp.pairs, eb.pairs);
        assert_eq!(bp.patterns.len(), 3);
        assert!(bp.patterns.iter().all(|p| p.is_empty()));
    }

    // ---------- error Display / source ----------

    #[test]
    fn error_displays_name_stages() {
        let e = EncodeAssemblyError::CoeffCountMismatch {
            stage_a: EncodeStage::Analysis,
            count_a: 512,
            stage_b: EncodeStage::Quant,
            count_b: 256,
        };
        let s = format!("{e}");
        assert!(s.contains("analysis"));
        assert!(s.contains("quant"));
        let dyn_err: &dyn std::error::Error = &e;
        assert!(dyn_err.source().is_none());

        let e = EncodeError::Analysis(InvalidSampleLen {
            expected: 256,
            got: 255,
        });
        assert!(format!("{e}").contains("analysis stage"));
        assert!(std::error::Error::source(&e).is_some());
    }
}

//! WMA entropy-stage spectral-coefficient assembler.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 fixes the structure
//! of the entropy stage that turns the per-mode decoded symbols back into
//! the block's integer spectral-coefficient vector. The load-bearing
//! citations:
//!
//! > **Mode switching.** The encoder switches between a **level mode**
//! > and a **run-length/level mode** depending on the sub-range
//! > (low-frequency mostly-non-zero range vs high-frequency mostly-zero
//! > range), selected by a mode selector across N predefined sub-coders.
//! >   — [PATENT US6,223,162 — mode selector 400, encoders 402–406]
//! >   — [PATENT US7,383,180 — entropy encoder 570 "switches between
//! >      level and run length/level modes"]
//!
//! > **Multi-level run-length coding.** Input symbols are **(R, L)
//! > pairings**: a run `R` of zero-valued coefficients followed by one
//! > non-zero coefficient of level `L`.
//! >   — [PATENT US6,223,162 — FIG.5/FIG.6; Claim 1; Claim 2]
//!
//! > **Partition / flag overhead.** The boundary between sub-ranges may
//! > be predetermined (no overhead) or adaptive…
//! >   — [PATENT US6,223,162 — partition 306]
//!
//! and the same FIG.6 step as drawn in §8 of the trace:
//!
//! > ```text
//! >  → entropy decode (run-level → coefficients; matrix deltas)
//! >  → inverse quantize + inverse weighting
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (decoder
//! >     pipeline, Thumpudi-180 FIG.6)
//!
//! ## Scope of this module
//!
//! This module is the **assembler** that wires the two §6 primitives
//! already landed — [`crate::entropy_mode::Partition`] (Round 4) and the
//! [`crate::runlevel`] run-level walker (Round 3) — into the single
//! entropy-decode step the patent's FIG.6 draws immediately upstream of
//! the inverse quantizer, exactly as [`crate::synthesis`] assembled the
//! §3 reconstruction chain and [`crate::dequant`] assembled the §4
//! inverse-quantize step.
//!
//! For one block the partition splits the `total` coefficients into two
//! patent-named sub-ranges:
//!
//! * **Level mode** — coefficient indices `0..split`, the low-frequency,
//!   mostly-non-zero range. Each coefficient carries its own decoded
//!   level symbol; the assembler copies the `split` level symbols
//!   verbatim into the head of the output vector
//!   ([`crate::entropy_mode::EntropyMode::Level`]).
//! * **Run-level mode** — coefficient indices `split..total`, the
//!   high-frequency, mostly-zero range. The decoded `(R, L)` pairs are
//!   expanded by [`crate::runlevel::expand_into`] over the
//!   `run_level_range_len()`-coefficient tail window, honouring the
//!   patent's implicit `(N, 1)` terminator
//!   ([`crate::entropy_mode::EntropyMode::RunLevel`]).
//!
//! The output `M`-coefficient `i32` vector is exactly the input
//! [`crate::dequant::DequantStage::block`] consumes, so the two
//! assemblers chain into the FIG.6 decoder front-half *entropy decode →
//! inverse quantize/weight*.
//!
//! ## What is NOT in this module
//!
//! * **The codeword tables / the bit reader.** Per §6 of the trace the
//!   joint `(R, L)` Huffman tables and the level-mode Huffman tables are
//!   `[GAP]` — they are not staged anywhere in this crate. This
//!   assembler therefore consumes **already-decoded symbols** (level
//!   values and `(R, L)` pairs), exactly as
//!   [`crate::runlevel::expand_into`] consumes already-decoded pairs: it
//!   places the symbols at their patent-fixed positions, it does not
//!   read codewords from bits.
//! * **Escape decoding.** A pairing excluded from the probability-
//!   thresholded codebook is recovered from its escape literal by
//!   [`crate::escape::EscapeLiteral::as_run_level_pair`] *before* it
//!   reaches this assembler; the caller passes the recovered
//!   [`crate::runlevel::RunLevelPair`] in the run-level slice. The
//!   in-codebook vs. escape disposition is [`crate::codebook`]'s job
//!   (Round 6).
//! * **Per-coefficient sign reconstruction.** The patent describes
//!   levels as magnitudes; sign-bit placement is `[GAP]` per §6
//!   (sign coding). The level-mode head carries already-signed `i32`
//!   level symbols (whatever sign the caller decoded); the run-level
//!   tail carries non-negative magnitudes, matching
//!   [`crate::runlevel`]'s documented sign-gap posture. The downstream
//!   [`crate::dequant::DequantStage::block`] likewise accepts
//!   already-signed `i32`.
//! * **The partition decision.** Whether the `split` boundary is
//!   predetermined or adaptive (and, if adaptive, how its flag is
//!   encoded) is `[GAP]` per §6; the [`crate::entropy_mode::Partition`]
//!   arrives already decoded.

use crate::entropy_mode::Partition;
use crate::runlevel::{self, RunLevelPair, WalkError};

/// Stateless entropy-stage spectral-coefficient assembler for one block,
/// per §6 of the patent trace (US6,223,162 mode selector 400 / FIG.5–6;
/// US7,383,180 entropy encoder 570).
///
/// One [`SpectralDecode::block`] call consumes the per-mode decoded
/// symbols of one block — `split` level-mode level values and the
/// run-level `(R, L)` pairs of the high-frequency tail — and emits the
/// `total`-coefficient `i32` spectral vector the patent's FIG.6 chain
/// feeds into the inverse quantizer
/// ([`crate::dequant::DequantStage::block`]).
///
/// The stage owns only the immutable [`Partition`] that names the
/// boundary between the two sub-ranges; it adds no arithmetic of its own
/// beyond placing each mode's symbols at their patent-fixed positions and
/// delegating the run-level expansion to [`crate::runlevel::expand_into`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SpectralDecode {
    partition: Partition,
}

impl SpectralDecode {
    /// Construct the assembler for a block from its decoded
    /// [`Partition`].
    ///
    /// The partition fixes the block's `total_coeffs` and the `split`
    /// boundary between the level-mode and run-level-mode sub-ranges; it
    /// has already validated `split <= total_coeffs` at its own
    /// construction, so this constructor is infallible.
    #[inline]
    pub const fn new(partition: Partition) -> Self {
        Self { partition }
    }

    /// The partition this assembler was built from.
    #[inline]
    pub const fn partition(&self) -> Partition {
        self.partition
    }

    /// Total coefficient count `M` of the block (the output length).
    #[inline]
    pub const fn total_coeffs(&self) -> u32 {
        self.partition.total_coeffs
    }

    /// Number of coefficients carried by the level-mode head sub-range
    /// (`0..split`).
    #[inline]
    pub const fn level_range_len(&self) -> u32 {
        self.partition.level_range_len()
    }

    /// Number of coefficients carried by the run-level-mode tail
    /// sub-range (`split..total`).
    #[inline]
    pub const fn run_level_range_len(&self) -> u32 {
        self.partition.run_level_range_len()
    }

    /// Assemble one block's `total`-coefficient `i32` spectral vector
    /// from its per-mode decoded symbols.
    ///
    /// * `levels` — the `split` already-decoded level-mode symbols for
    ///   the low-frequency head `0..split`, in coefficient order. These
    ///   are copied verbatim into the head of the output (already signed
    ///   per the caller's sign decode; sign placement is `[GAP]`).
    /// * `pairs` — the already-decoded run-level `(R, L)` pairs for the
    ///   high-frequency tail `split..total`. They are expanded by
    ///   [`crate::runlevel::expand_into`] over the
    ///   `run_level_range_len()`-coefficient tail window, with the
    ///   patent's implicit `(N, 1)` terminator honoured against the
    ///   tail's own remaining-coefficient count.
    ///
    /// The run-level tail magnitudes are non-negative; the level-mode
    /// head carries whatever sign the caller decoded.
    ///
    /// # Errors
    ///
    /// * [`SpectralError::LevelLenMismatch`] if `levels.len()` is not
    ///   exactly `level_range_len()` — the level-mode sub-range codes one
    ///   symbol per coefficient, so the count is fixed by the partition.
    /// * [`SpectralError::RunLevelWalk`] if the run-level walk over the
    ///   tail window fails (a pair overruns the tail, or the pairs are
    ///   exhausted before the tail is filled with no explicit upstream
    ///   end signal) — see [`crate::runlevel::WalkError`].
    pub fn block(&self, levels: &[i32], pairs: &[RunLevelPair]) -> Result<Vec<i32>, SpectralError> {
        let split = self.partition.split as usize;
        let total = self.partition.total_coeffs as usize;
        let tail = self.partition.run_level_range_len() as u64;

        if levels.len() != split {
            return Err(SpectralError::LevelLenMismatch {
                expected: split,
                got: levels.len(),
            });
        }

        let mut out = vec![0i32; total];

        // Level-mode head: one decoded symbol per coefficient, copied
        // verbatim into `0..split` (US6,223,162 level mode).
        out[..split].copy_from_slice(levels);

        // Run-level tail: expand the (R, L) pairs over the
        // `split..total` window. `expand_into` writes non-negative
        // magnitudes into a `u32` scratch, then we widen them into the
        // signed output tail (sign is `[GAP]`; the run-level tail is
        // non-negative per `crate::runlevel`).
        if tail > 0 {
            let mut scratch = vec![0u32; tail as usize];
            runlevel::expand_into(pairs, tail, &mut scratch)
                .map_err(SpectralError::RunLevelWalk)?;
            for (dst, &mag) in out[split..].iter_mut().zip(scratch.iter()) {
                // A run-level magnitude that exceeds `i32::MAX` cannot be
                // represented as a signed coefficient; the patent's level
                // alphabet is bounded by the (gapped) codeword tables, so
                // a magnitude this large signals a corrupt symbol stream.
                *dst = i32::try_from(mag)
                    .map_err(|_| SpectralError::LevelOverflow { magnitude: mag })?;
            }
        }

        Ok(out)
    }
}

/// Rejection reasons for [`SpectralDecode::block`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpectralError {
    /// The supplied level-mode symbol count does not match the
    /// partition's `split` (the level mode codes exactly one symbol per
    /// coefficient in the head sub-range).
    LevelLenMismatch {
        /// Symbols the partition's `split` requires.
        expected: usize,
        /// Symbols the caller supplied.
        got: usize,
    },
    /// The run-level walk over the tail window failed. Wraps the
    /// [`crate::runlevel::WalkError`] (overflow / underrun).
    RunLevelWalk(WalkError),
    /// A run-level magnitude exceeded `i32::MAX` and cannot be
    /// represented as a signed spectral coefficient — a corrupt symbol
    /// stream.
    LevelOverflow {
        /// The offending non-negative magnitude.
        magnitude: u32,
    },
}

impl core::fmt::Display for SpectralError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SpectralError::LevelLenMismatch { expected, got } => write!(
                f,
                "oxideav-wma::spectral: level-mode symbol count {got} does not match partition split {expected}",
            ),
            SpectralError::RunLevelWalk(e) => write!(
                f,
                "oxideav-wma::spectral: run-level tail walk failed: {e}",
            ),
            SpectralError::LevelOverflow { magnitude } => write!(
                f,
                "oxideav-wma::spectral: run-level magnitude {magnitude} exceeds i32::MAX",
            ),
        }
    }
}

impl std::error::Error for SpectralError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SpectralError::RunLevelWalk(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::BlockSize;
    use crate::dequant::DequantStage;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::step_size::OverallStepSize;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    fn part(total: u32, split: u32) -> Partition {
        Partition::new(total, split, false).expect("test partition must be valid")
    }

    // ---------- construction / accessors ----------

    #[test]
    fn accessors_mirror_partition() {
        let sd = SpectralDecode::new(part(64, 16));
        assert_eq!(sd.total_coeffs(), 64);
        assert_eq!(sd.level_range_len(), 16);
        assert_eq!(sd.run_level_range_len(), 48);
        assert_eq!(sd.partition(), part(64, 16));
    }

    // ---------- level-mode head ----------

    #[test]
    fn level_head_copied_verbatim_including_signs() {
        // split == total → the whole block is level mode; every symbol
        // is copied through, signs preserved.
        let sd = SpectralDecode::new(part(4, 4));
        let out = sd.block(&[3, -2, 0, 7], &[]).unwrap();
        assert_eq!(out, vec![3, -2, 0, 7]);
    }

    #[test]
    fn level_len_must_equal_split() {
        let sd = SpectralDecode::new(part(8, 3));
        // 2 levels supplied for a split of 3.
        let err = sd.block(&[1, 2], &[pair(5, 1)]).unwrap_err();
        assert_eq!(
            err,
            SpectralError::LevelLenMismatch {
                expected: 3,
                got: 2,
            }
        );
    }

    // ---------- run-level tail ----------

    #[test]
    fn run_level_tail_expands_over_tail_window_only() {
        // split = 2 (level head [9, -9]); tail = 6 coefficients.
        // (run=2, level=5) → tail [0, 0, 5, 0, 0, 0]; the trailing
        // zeros surface an underrun, which we feed an implicit (N, 1).
        // Instead build a clean fill: (1, 4) (3, 7) over 6 tail coeffs:
        //   c=0 →(1,4): write idx1 → [0,4,...]; c=2
        //   c=2 →(3,7): write idx5 → [0,4,0,0,0,7]; c=6 fills
        let sd = SpectralDecode::new(part(8, 2));
        let out = sd.block(&[9, -9], &[pair(1, 4), pair(3, 7)]).unwrap();
        assert_eq!(out, vec![9, -9, 0, 4, 0, 0, 0, 7]);
    }

    #[test]
    fn whole_block_run_level_when_split_is_zero() {
        // split = 0 → entire block is run-level mode (high-frequency).
        let sd = SpectralDecode::new(part(4, 0));
        // (1, 4) (1, 2) over 4 coeffs → [0, 4, 0, 2]
        let out = sd.block(&[], &[pair(1, 4), pair(1, 2)]).unwrap();
        assert_eq!(out, vec![0, 4, 0, 2]);
    }

    #[test]
    fn implicit_terminator_fires_over_tail_remaining_not_block() {
        // split = 4, tail = 4. The (N, 1) terminator must be measured
        // against the *tail's* remaining count (4), not the block's (8).
        // (1, 5) consumes 2 tail coeffs → tail remaining = 2; then a
        // (2, 1) pair is the implicit terminator for the 2-coeff
        // remainder → tail = [0, 5, 0, 0].
        let sd = SpectralDecode::new(part(8, 4));
        let out = sd.block(&[1, 1, 1, 1], &[pair(1, 5), pair(2, 1)]).unwrap();
        assert_eq!(out, vec![1, 1, 1, 1, 0, 5, 0, 0]);
    }

    #[test]
    fn run_level_walk_overflow_surfaces() {
        // tail = 4; (5, 1) overruns it.
        let sd = SpectralDecode::new(part(8, 4));
        let err = sd.block(&[0, 0, 0, 0], &[pair(5, 1)]).unwrap_err();
        assert_eq!(
            err,
            SpectralError::RunLevelWalk(WalkError::Overflow {
                at: 0,
                remaining: 4,
            })
        );
    }

    #[test]
    fn run_level_walk_underrun_surfaces() {
        // tail = 4; a single (1, 5) leaves 2 coeffs unfilled and no
        // implicit terminator → underrun.
        let sd = SpectralDecode::new(part(8, 4));
        let err = sd.block(&[0, 0, 0, 0], &[pair(1, 5)]).unwrap_err();
        assert_eq!(
            err,
            SpectralError::RunLevelWalk(WalkError::Underrun { remaining: 2 })
        );
    }

    #[test]
    fn empty_tail_ignores_pairs() {
        // split == total → tail = 0; pairs are irrelevant, none consumed.
        let sd = SpectralDecode::new(part(3, 3));
        let out = sd.block(&[1, 2, 3], &[]).unwrap();
        assert_eq!(out, vec![1, 2, 3]);
    }

    #[test]
    fn level_overflow_is_rejected() {
        // A run-level magnitude above i32::MAX cannot be a signed coeff.
        // tail = 2; (1, u32::MAX) writes the magnitude at tail idx 1.
        let sd = SpectralDecode::new(part(2, 0));
        let err = sd.block(&[], &[pair(1, u32::MAX)]).unwrap_err();
        assert_eq!(
            err,
            SpectralError::LevelOverflow {
                magnitude: u32::MAX,
            }
        );
    }

    // ---------- cross-module: chains into DequantStage ----------

    #[test]
    fn output_feeds_dequant_stage_unchanged() {
        // Assemble a block then run it straight into the §4 dequant
        // assembler, proving the entropy front-half and inverse-quantize
        // step chain on the same i32 vector.
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;

        // A whole-block run-level decode that fills the 256-coeff block:
        // a (255, 1) pair writes the last coefficient and an implicit
        // (1, 1) is not needed — (255, 1) lands the non-zero on the
        // final slot exactly.
        let sd = SpectralDecode::new(part(m as u32, 0));
        let q = sd.block(&[], &[pair(255, 3)]).unwrap();
        assert_eq!(q.len(), m);
        assert_eq!(q[m - 1], 3);
        assert!(q[..m - 1].iter().all(|&v| v == 0));

        // Single-band layout, unit weight, unit step → dequant is the
        // identity on the integer coefficients.
        let band = QuantBand::new(0, m as u16, 0).unwrap();
        let layout = QuantBandLayout::new(vec![band], m).unwrap();
        let weights = [1.0_f64];
        let step = OverallStepSize::new(1.0).unwrap();
        let stage = DequantStage::new(bs, &layout, &weights, step).unwrap();
        let coeff_hat = stage.block(&q).unwrap();
        assert_eq!(coeff_hat.len(), m);
        assert_eq!(coeff_hat[m - 1], 3.0);
        assert!(coeff_hat[..m - 1].iter().all(|&v| v == 0.0));
    }

    // ---------- error Display / source ----------

    #[test]
    fn error_display_and_source() {
        let e = SpectralError::RunLevelWalk(WalkError::Underrun { remaining: 1 });
        assert!(format!("{e}").contains("run-level tail walk failed"));
        assert!(std::error::Error::source(&e).is_some());

        let e2 = SpectralError::LevelLenMismatch {
            expected: 4,
            got: 2,
        };
        assert!(format!("{e2}").contains("does not match partition split"));
        assert!(std::error::Error::source(&e2).is_none());
    }
}

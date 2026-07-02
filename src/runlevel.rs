//! WMA spectral-coefficient run-level pairing model.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 lifts the
//! patent-disclosed run-level pairing primitive that the WMA Standard
//! entropy stage operates on. The two load-bearing citations:
//!
//! > **Multi-level run-length coding.** Input symbols are **(R, L)
//! > pairings**: a run `R` of zero-valued coefficients followed by one
//! > non-zero coefficient of level `L`. … Levels are `{1…Ln}` (with
//! > `Ln` meaning "≥ Ln") and runs are `{1…Rm}` (with `Rm` meaning
//! > "≥ Rm").
//! >   — [PATENT US6,223,162 — FIG.5/FIG.6; Claim 1; Claim 2]
//!
//! > **End of block / end of stream.** Termination uses "either a
//! > special ending signal… or a special event such as `(N, 1)`"
//! > because the decoder knows the total coefficient count for the
//! > block.
//! >   — [PATENT US6,223,162 — end-of-stream discussion]
//!
//! ## Scope of this module
//!
//! This module exposes the `(R, L)` pairing as a typed primitive plus
//! a sequence-walker that converts a stream of pairings back into the
//! sparse coefficient sequence they encode. The walker honours both
//! termination rules the patent names: an explicit caller-supplied
//! "end signal" boundary and the implicit `(N, 1)` event detected
//! against the block's known total coefficient count.
//!
//! ## What is NOT in this module
//!
//! * **The codeword tables.** The patent corpus discloses the *method*
//!   (joint 2-D Huffman over `(R, L)` plus escape) and the *structure*
//!   of the code book (probability grid with a threshold) but it does
//!   not reproduce the WMA v1/v2 code-word tables. Those are `[GAP]`
//!   in the trace and are not implemented anywhere in this crate.
//! * **The escape-coding bit layout.** The trace establishes that an
//!   escape symbol exists and is followed by enough literal bits to
//!   reconstruct the rejected pairing, but the bit widths are not
//!   patent-disclosed and are `[GAP]`.
//! * **Sign coding.** The level conveys magnitude only; the sign of
//!   each non-zero coefficient is `[GAP]` per the trace.
//!
//! The primitive here therefore carries `level` as a magnitude
//! (`NonZeroU32`); a sign byte/bit lives in the future bit-stream
//! reader once a trace pins it.

use core::num::NonZeroU32;

/// One run-level pairing as the patent describes it.
///
/// `run` is the count of zero-valued coefficients that precede the
/// non-zero coefficient; `level` is the magnitude of that non-zero
/// coefficient (sign placement is `[GAP]` per the trace and lives
/// outside this type).
///
/// Per [PATENT US6,223,162] the set the patent indexes its code book
/// over starts at `run = 1` and `level = 1`. Constructing a pair with
/// `run == 0` is rejected by [`RunLevelPair::new`] — a downstream
/// trace doc would have to widen the allowed range explicitly before
/// this primitive accepts that case.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RunLevelPair {
    /// Number of preceding zero-valued coefficients. Range per the
    /// trace: `{1, 2, …}`.
    pub run: u32,
    /// Magnitude of the non-zero coefficient that closes the run.
    /// Non-zero by construction (the patent's Claim 2 states "the
    /// first value is zero, and L is non-zero").
    pub level: NonZeroU32,
}

impl RunLevelPair {
    /// Try to build a pair from a `(run, level)` tuple. Returns
    /// `Err(InvalidPair)` when the inputs leave the patent-disclosed
    /// range, i.e. when `run == 0` or `level == 0`.
    pub fn new(run: u32, level: u32) -> core::result::Result<Self, InvalidPair> {
        if run == 0 {
            return Err(InvalidPair::ZeroRun);
        }
        let Some(level) = NonZeroU32::new(level) else {
            return Err(InvalidPair::ZeroLevel);
        };
        Ok(RunLevelPair { run, level })
    }

    /// Number of spectrum samples this pair contributes — i.e.
    /// `run` zeros plus one non-zero.
    #[inline]
    pub const fn coefficient_count(self) -> u64 {
        // Widen to u64 so very long runs cannot overflow during a
        // walk-length accumulation.
        self.run as u64 + 1
    }

    /// `true` when this pair matches the patent's implicit
    /// `(N, 1)` end-of-block event for a block whose remaining
    /// (un-decoded) coefficient count is `remaining_coeffs`.
    ///
    /// Per the patent: "Termination uses … a special event such as
    /// `(N, 1)` because the decoder knows the total coefficient count
    /// for the block."
    #[inline]
    pub fn is_implicit_terminator_for(self, remaining_coeffs: u64) -> bool {
        self.run as u64 == remaining_coeffs && self.level.get() == 1
    }
}

/// Construction-time rejection for [`RunLevelPair::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidPair {
    /// `run == 0`. Outside the patent-disclosed `{1..Rm}` set.
    ZeroRun,
    /// `level == 0`. Outside the patent-disclosed `{1..Ln}` set (and
    /// excluded by Claim 2: "L is non-zero").
    ZeroLevel,
}

impl core::fmt::Display for InvalidPair {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidPair::ZeroRun => f.write_str(
                "oxideav-wma::runlevel: run == 0 is outside the patent-disclosed {1..Rm} set",
            ),
            InvalidPair::ZeroLevel => f.write_str(
                "oxideav-wma::runlevel: level == 0 is excluded by Claim 2 of US6,223,162",
            ),
        }
    }
}

impl std::error::Error for InvalidPair {}

/// Errors a [`expand_into`] walk can surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WalkError {
    /// A pair would extend past the block's total coefficient count.
    /// `at` is the pair index that overflows; `remaining` is the
    /// number of coefficients still expected before the overflowing
    /// pair was consumed.
    Overflow {
        /// Zero-based index of the pair that overflows the block.
        at: usize,
        /// Coefficients still expected before the pair was processed.
        remaining: u64,
    },
    /// The pair stream terminated before the block's coefficient
    /// count was fully consumed. `remaining` reports the deficit.
    /// Per the patent this is legal only when the encoder emits an
    /// explicit end-of-stream signal upstream of this walker; the
    /// walker itself surfaces the underrun so the caller can decide.
    Underrun {
        /// Coefficients still missing after the pair stream emptied.
        remaining: u64,
    },
}

impl core::fmt::Display for WalkError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WalkError::Overflow { at, remaining } => write!(
                f,
                "oxideav-wma::runlevel: pair index {at} would overflow the block ({remaining} coefficients remaining)",
            ),
            WalkError::Underrun { remaining } => write!(
                f,
                "oxideav-wma::runlevel: pair stream exhausted with {remaining} coefficients still unfilled",
            ),
        }
    }
}

impl std::error::Error for WalkError {}

/// Expand a sequence of run-level pairs into the sparse coefficient
/// sequence they encode. Writes signed magnitudes (no sign — the
/// trace marks sign placement as `[GAP]`) into `out`.
///
/// `total_coeffs` is the block's known total coefficient count, used
/// both as the write-window length and as the trigger for the
/// patent's implicit `(N, 1)` terminator. The returned `usize` is the
/// number of pairs actually consumed from the input (including the
/// terminator pair if one fired); the caller's `out` slice is filled
/// to `total_coeffs` entries when the walk succeeds.
///
/// Termination rules (patent §6):
///
/// * The walker stops as soon as one of the input pairs satisfies
///   [`RunLevelPair::is_implicit_terminator_for`]; the implicit
///   terminator does **not** emit its own non-zero coefficient,
///   matching the patent's "(N, 1)" sentinel reading.
/// * The walker also stops when `total_coeffs` coefficients have been
///   emitted by ordinary pairs, even if the input stream still has
///   more pairs to give. This is the "explicit ending signal" branch
///   delegated to the caller — the upstream bit-stream reader will
///   have stopped feeding pairs by that point.
pub fn expand_into(
    pairs: &[RunLevelPair],
    total_coeffs: u64,
    out: &mut [u32],
) -> Result<usize, WalkError> {
    assert!(
        out.len() as u64 >= total_coeffs,
        "oxideav-wma::runlevel::expand_into: output slice must hold at least total_coeffs entries",
    );

    // Pre-zero the window so unfilled positions are deterministic.
    for slot in out.iter_mut().take(total_coeffs as usize) {
        *slot = 0;
    }

    let mut cursor: u64 = 0;
    for (idx, pair) in pairs.iter().copied().enumerate() {
        let remaining = total_coeffs.saturating_sub(cursor);

        // Implicit (N, 1) terminator check: the patent says this is
        // detected against the block's known remaining coefficient
        // count, so we test before we try to emit the pair.
        if pair.is_implicit_terminator_for(remaining) {
            return Ok(idx + 1);
        }

        let needed = pair.coefficient_count();
        if needed > remaining {
            return Err(WalkError::Overflow { at: idx, remaining });
        }

        // Advance the cursor past `run` zeros and write the non-zero
        // magnitude at the run's tail position.
        cursor += pair.run as u64;
        out[cursor as usize] = pair.level.get();
        cursor += 1;

        if cursor == total_coeffs {
            return Ok(idx + 1);
        }
    }

    let remaining = total_coeffs - cursor;
    if remaining == 0 {
        Ok(pairs.len())
    } else {
        Err(WalkError::Underrun { remaining })
    }
}

/// The encoder-side product of [`compress`]: the ordinary `(R, L)`
/// pairs of a sparse coefficient sequence plus the count of zeros left
/// after its last non-zero coefficient.
///
/// The trailing zeros are reported rather than encoded because the
/// patent names **two** ways to close the block — "either a special
/// ending signal… or a special event such as `(N, 1)`"
/// [PATENT US6,223,162 — end-of-stream discussion] — and the choice
/// between them belongs to the caller (see
/// [`crate::terminator::TerminatorMechanism`]).
/// [`Compressed::pairs_with_implicit_terminator`] realises the
/// implicit-`(N, 1)` branch, the one [`expand_into`] recognises.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Compressed {
    /// The ordinary `(R, L)` pairs, in coefficient order.
    pub pairs: Vec<RunLevelPair>,
    /// Zero-valued coefficients after the final non-zero one (the
    /// whole block when no coefficient is non-zero).
    pub trailing_zeros: u64,
}

impl Compressed {
    /// The pair sequence with the patent's implicit `(N, 1)`
    /// terminator appended when trailing zeros remain — exactly the
    /// input [`expand_into`] decodes back to the original sequence.
    ///
    /// When the last non-zero coefficient falls on the block's final
    /// slot (`trailing_zeros == 0`) no terminator is needed and the
    /// ordinary pairs are returned unchanged, matching the walker's
    /// natural-fill return path.
    pub fn pairs_with_implicit_terminator(&self) -> Vec<RunLevelPair> {
        let mut out = self.pairs.clone();
        if self.trailing_zeros > 0 {
            // `trailing_zeros` fits u32 for every patent block size
            // (max 4096); a hand-built larger sequence saturates,
            // which `debug_assert` guards during development.
            debug_assert!(self.trailing_zeros <= u32::MAX as u64);
            let run = u32::try_from(self.trailing_zeros).unwrap_or(u32::MAX);
            out.push(RunLevelPair {
                run,
                level: NonZeroU32::MIN,
            });
        }
        out
    }
}

/// Rejection reason for [`compress`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CompressError {
    /// The coefficient at `index` is non-zero but no zero precedes it
    /// (since the block start or the previous non-zero), so its run
    /// would be `0` — outside the patent-disclosed `{1..Rm}` run set
    /// (US6,223,162 Claim 1). Per the patent's own rationale this is
    /// exactly the dense low-frequency statistic the **level mode**
    /// exists for: the encoder's mode selector must widen the
    /// level-mode head past this coefficient instead of run-level
    /// coding it.
    NoPrecedingZero {
        /// Zero-based index of the unrepresentable non-zero.
        index: usize,
    },
}

impl core::fmt::Display for CompressError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CompressError::NoPrecedingZero { index } => write!(
                f,
                "oxideav-wma::runlevel: non-zero coefficient at index {index} has no preceding zero — run 0 is outside the patent-disclosed {{1..Rm}} set; widen the level-mode head instead",
            ),
        }
    }
}

impl std::error::Error for CompressError {}

/// Compress a sparse coefficient sequence (non-negative magnitudes)
/// into its `(R, L)` pair sequence — the encoder-side inverse of
/// [`expand_into`].
///
/// Walks the sequence once: each non-zero coefficient of magnitude `L`
/// preceded by `R ≥ 1` zeros (counted since the block start or the
/// previous non-zero) emits the pair `(R, L)`
/// [PATENT US6,223,162 — Claim 1: "a run of R first-value symbols and
/// an adjacent symbol of value L"; Claim 2: "the first value is zero,
/// and L is non-zero"]. Zeros after the final non-zero are returned as
/// [`Compressed::trailing_zeros`] for the caller's terminator choice.
///
/// Sign handling is out of scope exactly as in [`expand_into`]: the
/// input carries magnitudes (`u32`), the sign-bit placement being
/// `[GAP]` per §6 of the trace.
///
/// # Errors
///
/// [`CompressError::NoPrecedingZero`] if a non-zero coefficient has no
/// preceding zero — its run would be `0`, outside the patent's
/// `{1..Rm}` set. Such a coefficient belongs in the level-mode head
/// (see [`crate::spectral::SpectralEncode`]).
pub fn compress(coeffs: &[u32]) -> Result<Compressed, CompressError> {
    let mut pairs = Vec::new();
    let mut run: u64 = 0;
    for (index, &c) in coeffs.iter().enumerate() {
        if c == 0 {
            run += 1;
            continue;
        }
        if run == 0 {
            return Err(CompressError::NoPrecedingZero { index });
        }
        // `run` counts positions inside one block, so it fits u32 for
        // every patent block size; saturate defensively for hand-built
        // oversized inputs.
        let run_u32 = u32::try_from(run).unwrap_or(u32::MAX);
        pairs.push(RunLevelPair {
            run: run_u32,
            level: NonZeroU32::new(c).expect("checked non-zero above"),
        });
        run = 0;
    }
    Ok(Compressed {
        pairs,
        trailing_zeros: run,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    // ---------- Constructor validation ----------

    #[test]
    fn new_rejects_zero_run() {
        assert_eq!(RunLevelPair::new(0, 5), Err(InvalidPair::ZeroRun));
    }

    #[test]
    fn new_rejects_zero_level() {
        assert_eq!(RunLevelPair::new(1, 0), Err(InvalidPair::ZeroLevel));
    }

    #[test]
    fn new_accepts_smallest_patent_disclosed_pair() {
        // (run=1, level=1) is the smallest member of the patent's
        // {1..Rm} × {1..Ln} grid.
        let p = pair(1, 1);
        assert_eq!(p.run, 1);
        assert_eq!(p.level.get(), 1);
    }

    #[test]
    fn new_accepts_large_runs_and_levels() {
        let p = pair(4095, u32::MAX);
        assert_eq!(p.run, 4095);
        assert_eq!(p.level.get(), u32::MAX);
    }

    // ---------- coefficient_count ----------

    #[test]
    fn coefficient_count_is_run_plus_one() {
        assert_eq!(pair(1, 1).coefficient_count(), 2);
        assert_eq!(pair(7, 9).coefficient_count(), 8);
        assert_eq!(pair(4095, 1).coefficient_count(), 4096);
    }

    #[test]
    fn coefficient_count_does_not_overflow_at_u32_max() {
        // run = u32::MAX widens cleanly into u64 + 1.
        assert_eq!(pair(u32::MAX, 1).coefficient_count(), u32::MAX as u64 + 1);
    }

    // ---------- Implicit (N, 1) terminator ----------

    #[test]
    fn is_implicit_terminator_when_run_matches_remaining_and_level_is_one() {
        // Block has 8 remaining coefficients; a (8, 1) pair is the
        // patent's special end-of-block event.
        assert!(pair(8, 1).is_implicit_terminator_for(8));
    }

    #[test]
    fn is_not_implicit_terminator_when_level_is_above_one() {
        // (8, 2) is an ordinary pair, not a terminator.
        assert!(!pair(8, 2).is_implicit_terminator_for(8));
    }

    #[test]
    fn is_not_implicit_terminator_when_run_does_not_match_remaining() {
        // (7, 1) with 8 remaining is an ordinary pair.
        assert!(!pair(7, 1).is_implicit_terminator_for(8));
        // (9, 1) with 8 remaining is overflow material, not a
        // terminator.
        assert!(!pair(9, 1).is_implicit_terminator_for(8));
    }

    // ---------- expand_into: happy paths ----------

    #[test]
    fn expand_into_writes_a_single_pair_at_run_tail() {
        // (run=3, level=5) over a 4-coefficient block produces
        // [0, 0, 0, 5].
        let mut out = [0u32; 4];
        let n = expand_into(&[pair(3, 5)], 4, &mut out).unwrap();
        assert_eq!(n, 1);
        assert_eq!(out, [0, 0, 0, 5]);
    }

    #[test]
    fn expand_into_chains_multiple_pairs() {
        // (1, 7) (1, 3) (2, 9) over an 8-coefficient block:
        // [0, 7, 0, 3, 0, 0, 9, ?]
        // The trailing slot is left at zero so the underrun branch
        // surfaces — the patent's explicit-end-of-stream branch
        // would have stopped the pair feed before this point.
        let mut out = [0u32; 8];
        let err = expand_into(&[pair(1, 7), pair(1, 3), pair(2, 9)], 8, &mut out).unwrap_err();
        assert_eq!(err, WalkError::Underrun { remaining: 1 });
        assert_eq!(out, [0, 7, 0, 3, 0, 0, 9, 0]);
    }

    #[test]
    fn expand_into_completes_block_with_implicit_terminator() {
        // (1, 7) (1, 3) (2, 9) (1, 1) over an 8-coefficient block:
        // first three pairs consume 7 coefficients; remaining = 1;
        // (1, 1) is the implicit terminator.
        let mut out = [0u32; 8];
        let n = expand_into(
            &[pair(1, 7), pair(1, 3), pair(2, 9), pair(1, 1)],
            8,
            &mut out,
        )
        .unwrap();
        assert_eq!(n, 4);
        // The terminator does NOT emit its own non-zero — the slot
        // at the block's tail stays at zero.
        assert_eq!(out, [0, 7, 0, 3, 0, 0, 9, 0]);
    }

    #[test]
    fn expand_into_returns_when_block_fills_naturally() {
        // Two pairs whose coefficient counts exactly sum to the
        // block size: (3, 7) writes slot 3, (3, 4) writes slot 7.
        // No (N, 1) terminator needed because the second pair's
        // non-zero lands on the final slot.
        let mut out = [0u32; 8];
        let n = expand_into(&[pair(3, 7), pair(3, 4)], 8, &mut out).unwrap();
        assert_eq!(n, 2);
        assert_eq!(out, [0, 0, 0, 7, 0, 0, 0, 4]);
    }

    // ---------- expand_into: error paths ----------

    #[test]
    fn expand_into_flags_overflow_when_pair_runs_past_block() {
        // (5, 1) over a 4-coefficient block: needs 6 slots, has 4.
        let mut out = [0u32; 4];
        let err = expand_into(&[pair(5, 1)], 4, &mut out).unwrap_err();
        assert_eq!(
            err,
            WalkError::Overflow {
                at: 0,
                remaining: 4
            }
        );
    }

    #[test]
    fn expand_into_flags_overflow_mid_stream() {
        // (1, 1) fills slot 1, then (4, 2) needs 5 more — but only
        // 2 remain (slots 2 and 3).
        let mut out = [0u32; 4];
        let err = expand_into(&[pair(1, 1), pair(4, 2)], 4, &mut out).unwrap_err();
        assert_eq!(
            err,
            WalkError::Overflow {
                at: 1,
                remaining: 2
            }
        );
    }

    #[test]
    fn expand_into_flags_underrun_when_pairs_exhausted_early() {
        let mut out = [0u32; 4];
        let err = expand_into(&[pair(1, 5)], 4, &mut out).unwrap_err();
        assert_eq!(err, WalkError::Underrun { remaining: 2 });
        // Partial work is observable.
        assert_eq!(out, [0, 5, 0, 0]);
    }

    #[test]
    fn expand_into_accepts_empty_block_without_pairs() {
        let mut out: [u32; 0] = [];
        let n = expand_into(&[], 0, &mut out).unwrap();
        assert_eq!(n, 0);
    }

    #[test]
    fn expand_into_implicit_terminator_on_empty_block_requires_run_zero() {
        // The patent's terminator triggers when run == remaining; on
        // an empty block remaining == 0, so the terminator would
        // need run == 0 — which the constructor rejects, so this
        // case is structurally unreachable. Confirm the walker
        // returns Overflow for any (≥1, 1) pair over an empty block.
        let mut out: [u32; 0] = [];
        let err = expand_into(&[pair(1, 1)], 0, &mut out).unwrap_err();
        assert_eq!(
            err,
            WalkError::Overflow {
                at: 0,
                remaining: 0
            }
        );
    }

    #[test]
    #[should_panic(expected = "output slice must hold at least total_coeffs entries")]
    fn expand_into_panics_when_out_is_too_short() {
        let mut out = [0u32; 2];
        let _ = expand_into(&[], 4, &mut out);
    }

    // ---------- compress: encoder-side inverse of expand_into ----------

    #[test]
    fn compress_emits_pair_per_isolated_nonzero() {
        // [0, 4, 0, 0, 2, 0, 0, 0, 0, 9] — the module's round-trip
        // spectrum — compresses to (1,4) (2,2) (4,9) with no trailing
        // zeros.
        let c = compress(&[0, 4, 0, 0, 2, 0, 0, 0, 0, 9]).unwrap();
        assert_eq!(c.pairs, vec![pair(1, 4), pair(2, 2), pair(4, 9)]);
        assert_eq!(c.trailing_zeros, 0);
    }

    #[test]
    fn compress_reports_trailing_zeros() {
        let c = compress(&[0, 7, 0, 3, 0, 0, 9, 0]).unwrap();
        assert_eq!(c.pairs, vec![pair(1, 7), pair(1, 3), pair(2, 9)]);
        assert_eq!(c.trailing_zeros, 1);
    }

    #[test]
    fn compress_all_zero_block_is_all_trailing() {
        let c = compress(&[0, 0, 0, 0]).unwrap();
        assert!(c.pairs.is_empty());
        assert_eq!(c.trailing_zeros, 4);
    }

    #[test]
    fn compress_empty_block() {
        let c = compress(&[]).unwrap();
        assert!(c.pairs.is_empty());
        assert_eq!(c.trailing_zeros, 0);
    }

    #[test]
    fn compress_rejects_leading_nonzero() {
        // A non-zero at index 0 has run 0 — outside {1..Rm}.
        assert_eq!(
            compress(&[5, 0, 0]),
            Err(CompressError::NoPrecedingZero { index: 0 })
        );
    }

    #[test]
    fn compress_rejects_adjacent_nonzeros() {
        // The second of two adjacent non-zeros has run 0.
        assert_eq!(
            compress(&[0, 3, 7, 0]),
            Err(CompressError::NoPrecedingZero { index: 2 })
        );
    }

    #[test]
    fn compress_error_display_cites_the_run_set() {
        let e = CompressError::NoPrecedingZero { index: 2 };
        let s = format!("{e}");
        assert!(s.contains("index 2"));
        assert!(s.contains("{1..Rm}"));
        let dyn_err: &dyn std::error::Error = &e;
        assert!(dyn_err.source().is_none());
    }

    #[test]
    fn pairs_with_implicit_terminator_appends_n1_only_when_needed() {
        // Trailing zeros → an (N, 1) pair is appended.
        let c = compress(&[0, 5, 0, 0, 0]).unwrap();
        assert_eq!(c.trailing_zeros, 3);
        assert_eq!(
            c.pairs_with_implicit_terminator(),
            vec![pair(1, 5), pair(3, 1)]
        );
        // Natural fill → pairs unchanged.
        let c = compress(&[0, 5]).unwrap();
        assert_eq!(c.trailing_zeros, 0);
        assert_eq!(c.pairs_with_implicit_terminator(), vec![pair(1, 5)]);
    }

    #[test]
    fn compress_expand_round_trip() {
        // compress → (implicit terminator) → expand_into recovers the
        // original sequence for a spread of shapes.
        let cases: Vec<Vec<u32>> = vec![
            vec![0, 4, 0, 0, 2, 0, 0, 0, 0, 9],
            vec![0, 7, 0, 3, 0, 0, 9, 0],
            vec![0, 0, 0, 0],
            vec![0, 1],
            vec![],
            vec![0, u32::MAX, 0],
        ];
        for original in cases {
            let c = compress(&original).unwrap();
            let feed = c.pairs_with_implicit_terminator();
            let mut out = vec![0u32; original.len()];
            expand_into(&feed, original.len() as u64, &mut out).unwrap();
            assert_eq!(out, original, "case {original:?}");
        }
    }

    #[test]
    fn compress_expand_round_trip_pseudorandom_sparse() {
        // A deterministic pseudo-random sparse sequence: every third
        // slot at most carries a non-zero, so runs are always >= 1.
        let mut seq = vec![0u32; 256];
        let mut state = 0x2545F491_u32;
        let mut i = 1usize;
        while i < seq.len() {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            seq[i] = (state >> 24) + 1; // non-zero magnitude
            i += 2 + (state as usize % 5);
        }
        let c = compress(&seq).unwrap();
        let feed = c.pairs_with_implicit_terminator();
        let mut out = vec![0u32; seq.len()];
        expand_into(&feed, seq.len() as u64, &mut out).unwrap();
        assert_eq!(out, seq);
    }

    // ---------- Round-trip: encode (manually) → decode (expand_into) ----------

    #[test]
    fn expand_round_trip_against_handcrafted_sparse_spectrum() {
        // A hand-crafted sparse spectrum (zeros + magnitudes):
        // index: 0 1 2 3 4 5 6 7 8 9
        // value: 0 4 0 0 2 0 0 0 0 9
        // Build the corresponding (R, L) sequence by hand. From
        // cursor `c`, a pair (R, L) advances `c → c+R` then writes
        // `L` at the new `c` and steps once more, so the formula
        // is `R = (target_index - c)`:
        //   c=0, target=1 → R=1, L=4
        //   c=2, target=4 → R=2, L=2
        //   c=5, target=9 → R=4, L=9
        let expected = [0u32, 4, 0, 0, 2, 0, 0, 0, 0, 9];
        let mut out = [0u32; 10];
        let n = expand_into(&[pair(1, 4), pair(2, 2), pair(4, 9)], 10, &mut out).unwrap();
        assert_eq!(n, 3);
        assert_eq!(out, expected);
    }
}

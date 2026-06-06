//! WMA escape-symbol literal payload carrier.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 lifts the
//! patent-disclosed escape mechanism that the WMA Standard run-level
//! entropy stage uses for `(R, L)` pairings excluded from the
//! probability-thresholded code book (US6,223,162). The load-bearing
//! citation:
//!
//! > **Escape coding.** A pairing that falls below the threshold (not
//! > in the code book) is emitted with an **escape/special symbol**
//! > followed by enough literal information to identify the zero-run
//! > length and the non-zero sample value.
//! >   — [PATENT US6,223,162 — escape symbol; Claim 4 ("the entropy
//! >      code is an escape code"); Claims 5–6 (decoder detects escape
//! >      and recovers `R` and `L` from the literal trailer)]
//!
//! The patent therefore fixes the **structural shape** of the escape
//! payload: an escape-symbol prefix followed by a literal trailer that
//! carries enough bits to recover the `(R, L)` pair the codebook
//! rejected. The bit widths of the run and level literals themselves
//! are deliberately abstracted: the patent does not pin them, and the
//! WMA v1/v2 wire format leaves their exact widths as `[GAP]` in the
//! trace doc. This module therefore carries the patent-disclosed
//! presence of an escape literal payload, with run / level fields wide
//! enough to host whatever value the upstream entropy reader recovers.
//!
//! ## Scope of this module
//!
//! Three patent-named structural facts are realised here:
//!
//! * The escape literal is a `(run, level)` tuple typed as
//!   [`EscapeLiteral`], with `run: u32` and `level: NonZeroU32` matching
//!   the underlying [`RunLevelPair`] domain. Claim 2 of US6,223,162
//!   fixes "the first value is zero, and `L` is non-zero," so an
//!   escape level of zero is rejected by the same claim that rejects a
//!   zero level on any `(R, L)` pair. Claim 1 fixes `R ≥ 1`, and the
//!   patent's run-length set is `{1..Rm}`, so a run of zero is also
//!   rejected.
//! * The escape branch applies **only** to pairings whose codebook
//!   disposition is [`Disposition::Escape`] — Claim 4 states "the
//!   entropy code *is* an escape code" when the codebook does not
//!   contain the pair. [`EscapeLiteral::for_pair`] enforces this by
//!   re-checking the grid's disposition before producing the literal.
//! * The literal is **invertible**: by Claims 5–6 the decoder
//!   recovers the original `(R, L)` from the literal trailer.
//!   [`EscapeLiteral::as_run_level_pair`] is that inverse — it
//!   reconstructs a [`RunLevelPair`] equal to the one the encoder fed
//!   into the escape branch.
//!
//! ## What is NOT in this module
//!
//! * **The escape literal bit layout.** Claim 4 establishes the
//!   structural presence of the literal trailer; the bit widths and
//!   ordering of the run / level literals on the wire are `[GAP]` per
//!   §6 of the trace doc. This module is the typed carrier, not the
//!   bitstream reader.
//! * **The escape symbol's codeword.** The patent names an
//!   "escape/special symbol" preceding the literal trailer; its
//!   codeword inside the WMA v1/v2 Huffman table is part of the
//!   `[GAP]` table contents and is not represented here.
//! * **Sign placement for the non-zero coefficient.** Per §6 the sign
//!   of each non-zero coefficient is `[GAP]`. The literal here carries
//!   only the level magnitude (`NonZeroU32`); the sign bit lives in a
//!   future bitstream reader once a trace pins it.
//!
//! ## Why a dedicated carrier rather than reusing `RunLevelPair`
//!
//! [`RunLevelPair`] is the codebook-domain primitive — a pairing the
//! entropy stage might code with a Huffman codeword **or** with an
//! escape trailer. [`EscapeLiteral`] is the strictly narrower type
//! that only inhabits the escape branch: by construction every value
//! of [`EscapeLiteral`] corresponds to a `(R, L)` pair whose codebook
//! disposition was [`Disposition::Escape`] at the time of
//! construction. Downstream code that operates on escape-branch
//! literals only can take the typed carrier and skip the disposition
//! re-check.

use core::num::NonZeroU32;

use crate::codebook::{CodebookGrid, Disposition};
use crate::runlevel::{InvalidPair, RunLevelPair};

/// Typed carrier for the literal payload that trails the patent's
/// escape symbol.
///
/// Per US6,223,162 Claim 4 the escape code is followed by "enough
/// literal information to identify the zero-run length and the
/// non-zero sample value". This type holds exactly that information:
/// the run `R` and the level magnitude `L` of the pairing that the
/// codebook excluded.
///
/// Constructors enforce the patent-fixed predicates:
///
/// * `run ≥ 1` (Claim 1: runs `{1..Rm}`).
/// * `level ≥ 1` (Claim 2: "the first value is zero, and L is
///   non-zero").
///
/// The escape literal carries the same `(R, L)` value the `(R, L)`
/// pair carried; the patent does not transform the values on the
/// escape branch, it only widens the representation so pairings
/// outside the codebook can still be transmitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EscapeLiteral {
    /// Number of preceding zero-valued coefficients. Patent range:
    /// `{1..}` (Claim 1).
    run: u32,
    /// Magnitude of the non-zero coefficient that closes the run.
    /// Non-zero by construction (Claim 2).
    level: NonZeroU32,
}

impl EscapeLiteral {
    /// Construct an escape literal from raw `(run, level)` values.
    ///
    /// Returns [`EscapeError::InvalidPair`] when the inputs leave the
    /// patent-disclosed range — `run == 0` or `level == 0`. The
    /// inner [`InvalidPair`] reports which predicate was violated.
    ///
    /// This constructor takes the run and level on their own and does
    /// not consult any codebook. Use [`EscapeLiteral::for_pair`] when
    /// a grid is on hand and the disposition predicate should be
    /// re-checked.
    pub fn new(run: u32, level: u32) -> core::result::Result<Self, EscapeError> {
        let pair = RunLevelPair::new(run, level).map_err(EscapeError::InvalidPair)?;
        Ok(EscapeLiteral {
            run: pair.run,
            level: pair.level,
        })
    }

    /// Construct an escape literal from a [`RunLevelPair`] whose
    /// codebook disposition is [`Disposition::Escape`].
    ///
    /// Per Claim 4 of US6,223,162 the escape branch applies precisely
    /// when the codebook does *not* contain the pair. This constructor
    /// re-checks `grid.disposition(pair)` and returns
    /// [`EscapeError::InCodebook`] when the pair is in the codebook —
    /// emitting an escape literal for an in-codebook pair would be
    /// malformed because the codeword path would have been the right
    /// emission.
    ///
    /// Round-trip: the resulting literal's `(run, level)` match
    /// `pair.run` and `pair.level`, so [`EscapeLiteral::as_run_level_pair`]
    /// rebuilds the input pair exactly.
    pub fn for_pair(
        grid: &CodebookGrid,
        pair: RunLevelPair,
    ) -> core::result::Result<Self, EscapeError> {
        match grid.disposition(pair) {
            Disposition::Escape => Ok(EscapeLiteral {
                run: pair.run,
                level: pair.level,
            }),
            Disposition::InCodebook => Err(EscapeError::InCodebook),
        }
    }

    /// The literal's zero-run length `R`.
    #[inline]
    pub const fn run(self) -> u32 {
        self.run
    }

    /// The literal's level magnitude `L` as a non-zero `u32`.
    #[inline]
    pub const fn level(self) -> NonZeroU32 {
        self.level
    }

    /// The literal's level magnitude `L` as a raw `u32`. Always
    /// non-zero.
    #[inline]
    pub const fn level_raw(self) -> u32 {
        self.level.get()
    }

    /// Reconstruct the [`RunLevelPair`] this literal represents — the
    /// patent's Claim 5/6 decoder side, where the literal trailer is
    /// inverted back into the codebook-domain pair.
    #[inline]
    pub const fn as_run_level_pair(self) -> RunLevelPair {
        RunLevelPair {
            run: self.run,
            level: self.level,
        }
    }
}

/// Construction-time rejection for the [`EscapeLiteral`] constructors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EscapeError {
    /// `run` or `level` were outside the patent-disclosed range. The
    /// inner [`InvalidPair`] names the offending field.
    InvalidPair(InvalidPair),
    /// [`EscapeLiteral::for_pair`] was called with a pair whose
    /// codebook disposition is [`Disposition::InCodebook`] — the
    /// escape branch does not apply per Claim 4.
    InCodebook,
}

impl core::fmt::Display for EscapeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EscapeError::InvalidPair(inner) => write!(
                f,
                "oxideav-wma: invalid escape literal — {inner}",
            ),
            EscapeError::InCodebook => f.write_str(
                "oxideav-wma: pair is in the codebook — the escape branch does not apply (US6,223,162 Claim 4)",
            ),
        }
    }
}

impl std::error::Error for EscapeError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codebook::CodebookGrid;

    // ---- helpers ----

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).unwrap()
    }

    /// Build a 2x2 codebook grid where pair (1,1) is the only
    /// in-codebook pair (probability 0.9) and (1,2), (2,1), (2,2) are
    /// below threshold (probability 0.1). Threshold 0.5 separates
    /// them cleanly.
    fn grid_2x2_only_11_in_codebook() -> CodebookGrid {
        CodebookGrid::from_probabilities(2, 2, 0.5, vec![0.9, 0.1, 0.1, 0.1]).unwrap()
    }

    // ---- new(): accept paths ----

    #[test]
    fn new_accepts_minimum_run_and_level() {
        let lit = EscapeLiteral::new(1, 1).unwrap();
        assert_eq!(lit.run(), 1);
        assert_eq!(lit.level_raw(), 1);
    }

    #[test]
    fn new_accepts_large_run_and_level() {
        let lit = EscapeLiteral::new(4096, 32_768).unwrap();
        assert_eq!(lit.run(), 4096);
        assert_eq!(lit.level_raw(), 32_768);
    }

    #[test]
    fn new_accepts_u32_max_run() {
        // The literal width is `[GAP]` per the patent; the carrier
        // must therefore host any value upstream might decode.
        let lit = EscapeLiteral::new(u32::MAX, 1).unwrap();
        assert_eq!(lit.run(), u32::MAX);
    }

    #[test]
    fn new_accepts_u32_max_level() {
        let lit = EscapeLiteral::new(1, u32::MAX).unwrap();
        assert_eq!(lit.level_raw(), u32::MAX);
    }

    // ---- new(): reject paths ----

    #[test]
    fn new_rejects_zero_run() {
        let err = EscapeLiteral::new(0, 1).unwrap_err();
        assert_eq!(err, EscapeError::InvalidPair(InvalidPair::ZeroRun));
    }

    #[test]
    fn new_rejects_zero_level() {
        let err = EscapeLiteral::new(1, 0).unwrap_err();
        assert_eq!(err, EscapeError::InvalidPair(InvalidPair::ZeroLevel));
    }

    #[test]
    fn new_rejects_zero_run_when_level_also_zero() {
        // Constructor checks run first per RunLevelPair::new contract.
        let err = EscapeLiteral::new(0, 0).unwrap_err();
        assert_eq!(err, EscapeError::InvalidPair(InvalidPair::ZeroRun));
    }

    // ---- for_pair(): grid integration ----

    #[test]
    fn for_pair_accepts_pair_with_escape_disposition() {
        let grid = grid_2x2_only_11_in_codebook();
        let p = pair(1, 2);
        assert!(grid.is_escape(p));
        let lit = EscapeLiteral::for_pair(&grid, p).unwrap();
        assert_eq!(lit.run(), 1);
        assert_eq!(lit.level_raw(), 2);
    }

    #[test]
    fn for_pair_accepts_pair_outside_grid_rectangle() {
        // Pairs outside (rm, ln) are unconditionally escape pairings.
        let grid = grid_2x2_only_11_in_codebook();
        let p = pair(5, 7);
        assert!(grid.is_escape(p));
        let lit = EscapeLiteral::for_pair(&grid, p).unwrap();
        assert_eq!(lit.run(), 5);
        assert_eq!(lit.level_raw(), 7);
    }

    #[test]
    fn for_pair_rejects_in_codebook_pair() {
        let grid = grid_2x2_only_11_in_codebook();
        let p = pair(1, 1);
        assert!(grid.is_in_codebook(p));
        let err = EscapeLiteral::for_pair(&grid, p).unwrap_err();
        assert_eq!(err, EscapeError::InCodebook);
    }

    // ---- accessors ----

    #[test]
    fn level_returns_non_zero_u32() {
        let lit = EscapeLiteral::new(3, 42).unwrap();
        assert_eq!(lit.level(), NonZeroU32::new(42).unwrap());
        assert_eq!(lit.level_raw(), 42);
    }

    #[test]
    fn copy_and_eq_hold_for_two_literals_with_same_fields() {
        let a = EscapeLiteral::new(7, 3).unwrap();
        let b = a;
        assert_eq!(a, b);
        let c = EscapeLiteral::new(7, 3).unwrap();
        assert_eq!(a, c);
    }

    // ---- round-trip: as_run_level_pair ----

    #[test]
    fn as_run_level_pair_round_trips_explicit_constructor() {
        let lit = EscapeLiteral::new(9, 17).unwrap();
        let p = lit.as_run_level_pair();
        assert_eq!(p, pair(9, 17));
    }

    #[test]
    fn as_run_level_pair_round_trips_grid_constructor() {
        let grid = grid_2x2_only_11_in_codebook();
        let p_in = pair(2, 2);
        let lit = EscapeLiteral::for_pair(&grid, p_in).unwrap();
        let p_out = lit.as_run_level_pair();
        assert_eq!(p_in, p_out);
    }

    #[test]
    fn as_run_level_pair_round_trips_u32_max_run_and_level() {
        let lit = EscapeLiteral::new(u32::MAX, u32::MAX).unwrap();
        let p = lit.as_run_level_pair();
        assert_eq!(p.run, u32::MAX);
        assert_eq!(p.level.get(), u32::MAX);
    }

    // ---- error display ----

    #[test]
    fn error_display_includes_inner_invalid_pair_for_zero_run() {
        let err = EscapeError::InvalidPair(InvalidPair::ZeroRun);
        let s = format!("{err}");
        assert!(s.contains("invalid escape literal"));
        assert!(s.contains("run"));
    }

    #[test]
    fn error_display_for_in_codebook_cites_claim_4() {
        let err = EscapeError::InCodebook;
        let s = format!("{err}");
        assert!(s.contains("US6,223,162"));
        assert!(s.contains("Claim 4"));
    }

    // ---- structural invariants ----

    #[test]
    fn every_grid_escape_pair_yields_a_literal_via_for_pair() {
        // Build a richer 3x3 grid where (1,1) and (2,1) are in
        // codebook, the rest are escape.
        let grid = CodebookGrid::from_probabilities(
            3,
            3,
            0.5,
            vec![0.9, 0.1, 0.1, 0.9, 0.1, 0.1, 0.1, 0.1, 0.1],
        )
        .unwrap();
        for r in 1..=3 {
            for l in 1..=3 {
                let p = pair(r, l);
                let disp = grid.disposition(p);
                let result = EscapeLiteral::for_pair(&grid, p);
                match disp {
                    Disposition::Escape => {
                        let lit = result.unwrap();
                        assert_eq!(lit.as_run_level_pair(), p);
                    }
                    Disposition::InCodebook => {
                        assert_eq!(result, Err(EscapeError::InCodebook));
                    }
                }
            }
        }
    }
}

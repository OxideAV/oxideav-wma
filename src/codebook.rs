//! WMA run-level codebook construction model and escape disposition.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 lifts the
//! patent-disclosed structure of the WMA Standard run-level codebook
//! and the escape mechanism that handles pairings the codebook excludes.
//! Two load-bearing citations:
//!
//! > **Code-book construction.** A 2-D probability grid over `(R, L)`
//! > pairings is built; pairings above a **probability threshold** get
//! > Huffman codewords, pairings below it are excluded to bound table
//! > size.
//! >   — [PATENT US6,223,162 — grid 500, threshold 518, FIG.6; Claims 8–10]
//!
//! > **Escape coding.** A pairing that falls below the threshold (not in
//! > the code book) is emitted with an **escape/special symbol**
//! > followed by enough literal information to identify the zero-run
//! > length and the non-zero sample value.
//! >   — [PATENT US6,223,162 — escape symbol; Claim 4 ("the entropy
//! >      code is an escape code"); Claims 5–6 (decoder detects escape)]
//!
//! ## Scope of this module
//!
//! This module exposes the *structural* objects the patent names — the
//! 2-D `(R, L)` probability grid bounded by `(Rm, Ln)`, a threshold
//! that splits the grid into in-codebook vs. escape pairings, and a
//! typed [`Disposition`] that names what an entropy stage should do
//! with a given [`crate::runlevel::RunLevelPair`]. The module
//! deliberately does not produce codewords or write bits: the
//! patent-disclosed mechanism (grid + threshold + escape branch) is
//! the carrier, not the wire format, and the WMA v1/v2 codeword
//! tables are `[GAP]` per the trace.
//!
//! Three concrete operations the patent text names are realised:
//!
//! * Building a grid from caller-supplied per-pairing probabilities
//!   (the encoder analysis stage) — [`CodebookGrid::from_probabilities`].
//! * Asking the grid whether a pairing is in the codebook or below the
//!   threshold — [`CodebookGrid::disposition`].
//! * Counting / iterating the in-codebook population — [`CodebookGrid::
//!   in_codebook_count`], [`CodebookGrid::in_codebook_pairs`].
//!
//! ## What is NOT in this module
//!
//! * **The codeword tables.** Per §6 of the trace, the literal WMA v1/v2
//!   Huffman codewords are `[GAP]`. This module produces no bits.
//! * **The escape literal layout.** Claim 4 establishes the escape
//!   mechanism's structural existence; the bit widths of the run /
//!   level literals that follow the escape symbol are `[GAP]`. This
//!   module reports only the patent-disclosed *disposition* of a pair
//!   (in-codebook vs. escape); the wire-format follow-on bits are a
//!   future-bitstream-reader concern.
//! * **The probability-estimation step.** The patent describes a 2-D
//!   probability grid; *how* the encoder estimates the probabilities
//!   (training corpus, per-mode statistics, etc.) is encoder analysis
//!   and not bit-stream-level. This module takes probabilities as
//!   opaque `f64` inputs.
//! * **Sign-bit placement.** Per §6 the sign of each non-zero
//!   coefficient is `[GAP]`. The disposition reported here is over the
//!   `(R, |L|)` pairing only — sign storage is downstream.

use crate::runlevel::RunLevelPair;

/// A 2-D probability grid over `(R, L)` run-level pairings, with a
/// probability threshold separating in-codebook pairings (those that
/// receive Huffman codewords) from escape pairings (those emitted via
/// the patent's escape-symbol branch).
///
/// The dimensions `(rm, ln)` match the patent's "runs are `{1..Rm}`
/// (with `Rm` meaning ≥ Rm)" and "levels are `{1..Ln}` (with `Ln`
/// meaning ≥ Ln)" bounds (US6,223,162). The grid stores one
/// probability per `(R, L)` pair in row-major (`run` outer, `level`
/// inner) order; positions outside the `(rm, ln)` rectangle are not
/// represented at all and are reported as escape pairings.
///
/// The threshold is a probability cutoff: pairings whose stored
/// probability is `>= threshold` are *in codebook*; pairings whose
/// stored probability is `< threshold` and pairings outside the
/// `(rm, ln)` rectangle are escape pairings.
///
/// The construction model is patent-disclosed; the probabilities
/// themselves are opaque per-application inputs.
#[derive(Debug, Clone, PartialEq)]
pub struct CodebookGrid {
    /// Patent's `Rm`: the largest representable run value (run is
    /// `{1..rm}` with `rm` meaning "≥ rm").
    rm: u32,
    /// Patent's `Ln`: the largest representable level magnitude
    /// (level is `{1..ln}` with `ln` meaning "≥ ln").
    ln: u32,
    /// Probability cutoff. Pairings with stored probability
    /// `>= threshold` are in-codebook; below it they escape.
    threshold: f64,
    /// Per-pair probabilities in row-major order with `run` outer
    /// (index `run-1`) and `level` inner (index `level-1`); length
    /// is exactly `rm * ln`.
    probabilities: Vec<f64>,
}

impl CodebookGrid {
    /// Construct a grid from the bounding `(rm, ln)` rectangle, a
    /// threshold, and a flat `probabilities` slice in row-major order
    /// (run outer, level inner, both 1-indexed).
    ///
    /// `probabilities[(r - 1) * ln + (l - 1)]` is the probability of
    /// pair `(r, l)`.
    pub fn from_probabilities(
        rm: u32,
        ln: u32,
        threshold: f64,
        probabilities: Vec<f64>,
    ) -> core::result::Result<Self, InvalidGrid> {
        if rm == 0 {
            return Err(InvalidGrid::ZeroRm);
        }
        if ln == 0 {
            return Err(InvalidGrid::ZeroLn);
        }
        if !(0.0..=1.0).contains(&threshold) {
            return Err(InvalidGrid::ThresholdOutOfRange { threshold });
        }
        let expected = (rm as usize)
            .checked_mul(ln as usize)
            .ok_or(InvalidGrid::DimensionsOverflow { rm, ln })?;
        if probabilities.len() != expected {
            return Err(InvalidGrid::ProbabilityLengthMismatch {
                expected,
                got: probabilities.len(),
            });
        }
        for &p in &probabilities {
            if !p.is_finite() || !(0.0..=1.0).contains(&p) {
                return Err(InvalidGrid::ProbabilityOutOfRange { probability: p });
            }
        }
        Ok(CodebookGrid {
            rm,
            ln,
            threshold,
            probabilities,
        })
    }

    /// Patent's `Rm` — the maximum representable run.
    #[inline]
    pub const fn rm(&self) -> u32 {
        self.rm
    }

    /// Patent's `Ln` — the maximum representable level magnitude.
    #[inline]
    pub const fn ln(&self) -> u32 {
        self.ln
    }

    /// The probability threshold separating in-codebook from escape
    /// pairings.
    #[inline]
    pub const fn threshold(&self) -> f64 {
        self.threshold
    }

    /// Lookup the stored probability for pair `(r, l)`. Returns `None`
    /// when `(r, l)` falls outside the `(rm, ln)` rectangle (i.e. is
    /// not represented at all).
    pub fn probability_of(&self, run: u32, level: u32) -> Option<f64> {
        if run == 0 || level == 0 || run > self.rm || level > self.ln {
            return None;
        }
        let idx = (run as usize - 1) * (self.ln as usize) + (level as usize - 1);
        Some(self.probabilities[idx])
    }

    /// Return the codebook disposition of a `(R, L)` pair: whether the
    /// pair is in the codebook (above-threshold and inside the
    /// rectangle) or must be emitted via the patent's escape branch.
    ///
    /// Per Claim 4 of US6,223,162 the escape branch applies when the
    /// pair is *not* in the codebook; this includes both
    /// "below-threshold inside the rectangle" and "outside the
    /// rectangle entirely".
    pub fn disposition(&self, pair: RunLevelPair) -> Disposition {
        match self.probability_of(pair.run, pair.level.get()) {
            Some(p) if p >= self.threshold => Disposition::InCodebook,
            _ => Disposition::Escape,
        }
    }

    /// `true` when the pair is in the codebook (above-threshold and
    /// inside the `(rm, ln)` rectangle).
    #[inline]
    pub fn is_in_codebook(&self, pair: RunLevelPair) -> bool {
        matches!(self.disposition(pair), Disposition::InCodebook)
    }

    /// `true` when the pair must use the patent's escape branch.
    #[inline]
    pub fn is_escape(&self, pair: RunLevelPair) -> bool {
        matches!(self.disposition(pair), Disposition::Escape)
    }

    /// Count of `(r, l)` positions inside the rectangle whose stored
    /// probability is `>= threshold`. Positions outside the rectangle
    /// are not counted (they are unconditionally escape pairings).
    pub fn in_codebook_count(&self) -> usize {
        self.probabilities
            .iter()
            .filter(|&&p| p >= self.threshold)
            .count()
    }

    /// Count of `(r, l)` positions inside the rectangle whose stored
    /// probability is `< threshold`. These are the escape-branch
    /// pairings *that the grid represents* — pairings outside the
    /// rectangle are excluded from this count.
    pub fn escape_count_in_rectangle(&self) -> usize {
        self.probabilities.len() - self.in_codebook_count()
    }

    /// Iterate over every in-codebook pair as a [`RunLevelPair`],
    /// in row-major (`run` outer, `level` inner) order.
    ///
    /// The iteration excludes pairings outside the `(rm, ln)`
    /// rectangle (those are unconditionally escape pairings and are
    /// not represented in the grid).
    pub fn in_codebook_pairs(&self) -> impl Iterator<Item = RunLevelPair> + '_ {
        let ln = self.ln;
        let threshold = self.threshold;
        self.probabilities
            .iter()
            .enumerate()
            .filter_map(move |(idx, &p)| {
                if p < threshold {
                    return None;
                }
                let run = (idx / ln as usize) as u32 + 1;
                let level = (idx % ln as usize) as u32 + 1;
                // Constructor guarantees run >= 1 and level >= 1, so
                // RunLevelPair::new cannot fail.
                RunLevelPair::new(run, level).ok()
            })
    }
}

/// Per-pair disposition reported by [`CodebookGrid::disposition`].
///
/// Per US6,223,162 a `(R, L)` pair is either coded by its codeword
/// from the probability-thresholded code book (Claims 8–10) or
/// emitted via the escape branch (Claims 4–6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Disposition {
    /// The pair has a codeword in the codebook. The downstream
    /// bit-stream stage emits the Huffman code for the pair.
    InCodebook,
    /// The pair is below threshold or outside the `(rm, ln)`
    /// rectangle. The downstream bit-stream stage emits the patent's
    /// escape symbol followed by literal run and level information
    /// (bit widths `[GAP]` per §6 of the trace).
    Escape,
}

impl Disposition {
    /// `true` for [`Disposition::InCodebook`].
    #[inline]
    pub const fn is_in_codebook(self) -> bool {
        matches!(self, Disposition::InCodebook)
    }

    /// `true` for [`Disposition::Escape`].
    #[inline]
    pub const fn is_escape(self) -> bool {
        matches!(self, Disposition::Escape)
    }
}

/// Construction-time rejection for [`CodebookGrid::from_probabilities`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InvalidGrid {
    /// `rm == 0`. The patent's run range is `{1..Rm}` with at least
    /// `Rm == 1`.
    ZeroRm,
    /// `ln == 0`. The patent's level range is `{1..Ln}` with at
    /// least `Ln == 1`.
    ZeroLn,
    /// `threshold` is outside the closed `[0.0, 1.0]` range required
    /// of a probability cutoff (or is non-finite).
    ThresholdOutOfRange {
        /// The rejected threshold value.
        threshold: f64,
    },
    /// `rm * ln` overflowed `usize` while sizing the probability
    /// table.
    DimensionsOverflow {
        /// The supplied `rm` (patent's `Rm`).
        rm: u32,
        /// The supplied `ln` (patent's `Ln`).
        ln: u32,
    },
    /// The probability slice length does not match `rm * ln`.
    ProbabilityLengthMismatch {
        /// The expected length (`rm * ln`).
        expected: usize,
        /// The actual length of the slice the caller passed.
        got: usize,
    },
    /// One of the supplied probabilities is non-finite or outside
    /// the `[0.0, 1.0]` range.
    ProbabilityOutOfRange {
        /// The rejected probability.
        probability: f64,
    },
}

impl core::fmt::Display for InvalidGrid {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidGrid::ZeroRm => f.write_str(
                "oxideav-wma::codebook: rm == 0; the patent's run range is {1..Rm} with Rm >= 1",
            ),
            InvalidGrid::ZeroLn => f.write_str(
                "oxideav-wma::codebook: ln == 0; the patent's level range is {1..Ln} with Ln >= 1",
            ),
            InvalidGrid::ThresholdOutOfRange { threshold } => write!(
                f,
                "oxideav-wma::codebook: threshold {threshold} is not a probability in [0.0, 1.0]",
            ),
            InvalidGrid::DimensionsOverflow { rm, ln } => write!(
                f,
                "oxideav-wma::codebook: rm ({rm}) * ln ({ln}) overflows usize",
            ),
            InvalidGrid::ProbabilityLengthMismatch { expected, got } => write!(
                f,
                "oxideav-wma::codebook: probability slice length mismatch (expected {expected} entries, got {got})",
            ),
            InvalidGrid::ProbabilityOutOfRange { probability } => write!(
                f,
                "oxideav-wma::codebook: probability {probability} is not in [0.0, 1.0]",
            ),
        }
    }
}

impl std::error::Error for InvalidGrid {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runlevel::RunLevelPair;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    fn grid_2x2(threshold: f64, probs: [f64; 4]) -> CodebookGrid {
        CodebookGrid::from_probabilities(2, 2, threshold, probs.to_vec())
            .expect("test grid must build")
    }

    // ---------- Constructor: accept paths ----------

    #[test]
    fn from_probabilities_accepts_a_minimal_grid() {
        // A 1x1 grid with a single in-codebook pair (1, 1).
        let g = CodebookGrid::from_probabilities(1, 1, 0.1, vec![0.5]).unwrap();
        assert_eq!(g.rm(), 1);
        assert_eq!(g.ln(), 1);
        assert_eq!(g.threshold(), 0.1);
    }

    #[test]
    fn from_probabilities_accepts_an_all_zero_grid() {
        // Every pair below threshold => every represented pair escapes.
        let g = grid_2x2(0.5, [0.0, 0.0, 0.0, 0.0]);
        assert_eq!(g.in_codebook_count(), 0);
        assert_eq!(g.escape_count_in_rectangle(), 4);
    }

    #[test]
    fn from_probabilities_accepts_an_all_one_grid() {
        // Every pair above threshold => every represented pair is
        // in-codebook.
        let g = grid_2x2(0.5, [1.0, 1.0, 1.0, 1.0]);
        assert_eq!(g.in_codebook_count(), 4);
        assert_eq!(g.escape_count_in_rectangle(), 0);
    }

    // ---------- Constructor: reject paths ----------

    #[test]
    fn from_probabilities_rejects_zero_rm() {
        let err = CodebookGrid::from_probabilities(0, 1, 0.5, vec![]).unwrap_err();
        assert_eq!(err, InvalidGrid::ZeroRm);
    }

    #[test]
    fn from_probabilities_rejects_zero_ln() {
        let err = CodebookGrid::from_probabilities(1, 0, 0.5, vec![]).unwrap_err();
        assert_eq!(err, InvalidGrid::ZeroLn);
    }

    #[test]
    fn from_probabilities_rejects_threshold_below_zero() {
        let err = CodebookGrid::from_probabilities(1, 1, -0.1, vec![0.5]).unwrap_err();
        assert_eq!(err, InvalidGrid::ThresholdOutOfRange { threshold: -0.1 });
    }

    #[test]
    fn from_probabilities_rejects_threshold_above_one() {
        let err = CodebookGrid::from_probabilities(1, 1, 1.5, vec![0.5]).unwrap_err();
        assert_eq!(err, InvalidGrid::ThresholdOutOfRange { threshold: 1.5 });
    }

    #[test]
    fn from_probabilities_rejects_nan_threshold() {
        let err = CodebookGrid::from_probabilities(1, 1, f64::NAN, vec![0.5]).unwrap_err();
        match err {
            InvalidGrid::ThresholdOutOfRange { threshold } => assert!(threshold.is_nan()),
            other => panic!("expected ThresholdOutOfRange, got {other:?}"),
        }
    }

    #[test]
    fn from_probabilities_rejects_length_mismatch() {
        // 2x2 grid expects 4 entries; supply 3.
        let err = CodebookGrid::from_probabilities(2, 2, 0.5, vec![0.1, 0.2, 0.3]).unwrap_err();
        assert_eq!(
            err,
            InvalidGrid::ProbabilityLengthMismatch {
                expected: 4,
                got: 3
            }
        );
    }

    #[test]
    fn from_probabilities_rejects_nan_probability() {
        let err = CodebookGrid::from_probabilities(1, 2, 0.5, vec![0.5, f64::NAN]).unwrap_err();
        match err {
            InvalidGrid::ProbabilityOutOfRange { probability } => assert!(probability.is_nan()),
            other => panic!("expected ProbabilityOutOfRange, got {other:?}"),
        }
    }

    #[test]
    fn from_probabilities_rejects_negative_probability() {
        let err = CodebookGrid::from_probabilities(1, 2, 0.5, vec![0.5, -0.01]).unwrap_err();
        assert_eq!(
            err,
            InvalidGrid::ProbabilityOutOfRange { probability: -0.01 }
        );
    }

    #[test]
    fn from_probabilities_rejects_probability_above_one() {
        let err = CodebookGrid::from_probabilities(1, 2, 0.5, vec![0.5, 1.01]).unwrap_err();
        assert_eq!(
            err,
            InvalidGrid::ProbabilityOutOfRange { probability: 1.01 }
        );
    }

    // ---------- Lookup: probability_of ----------

    #[test]
    fn probability_of_returns_row_major_value_inside_rectangle() {
        // 2x2 grid, run outer / level inner:
        //   (1,1) -> 0.10, (1,2) -> 0.20
        //   (2,1) -> 0.30, (2,2) -> 0.40
        let g = grid_2x2(0.25, [0.10, 0.20, 0.30, 0.40]);
        assert_eq!(g.probability_of(1, 1), Some(0.10));
        assert_eq!(g.probability_of(1, 2), Some(0.20));
        assert_eq!(g.probability_of(2, 1), Some(0.30));
        assert_eq!(g.probability_of(2, 2), Some(0.40));
    }

    #[test]
    fn probability_of_reports_outside_rectangle_as_none() {
        let g = grid_2x2(0.5, [0.1, 0.2, 0.3, 0.4]);
        // run too large
        assert_eq!(g.probability_of(3, 1), None);
        // level too large
        assert_eq!(g.probability_of(1, 3), None);
        // run == 0 (outside the patent's {1..Rm} set)
        assert_eq!(g.probability_of(0, 1), None);
        // level == 0 (outside the patent's {1..Ln} set)
        assert_eq!(g.probability_of(1, 0), None);
    }

    // ---------- Disposition ----------

    #[test]
    fn disposition_above_threshold_is_in_codebook() {
        let g = grid_2x2(0.25, [0.10, 0.20, 0.30, 0.40]);
        // (2, 1) at probability 0.30 >= threshold 0.25
        assert_eq!(g.disposition(pair(2, 1)), Disposition::InCodebook);
        assert!(g.is_in_codebook(pair(2, 1)));
        assert!(!g.is_escape(pair(2, 1)));
    }

    #[test]
    fn disposition_below_threshold_is_escape() {
        let g = grid_2x2(0.25, [0.10, 0.20, 0.30, 0.40]);
        // (1, 1) at probability 0.10 < threshold 0.25
        assert_eq!(g.disposition(pair(1, 1)), Disposition::Escape);
        assert!(g.is_escape(pair(1, 1)));
        assert!(!g.is_in_codebook(pair(1, 1)));
    }

    #[test]
    fn disposition_outside_rectangle_is_escape() {
        // 2x2 grid, all above threshold; a pair outside is still
        // escape because it is not represented in the grid at all
        // (Claim 4 escape branch covers "not in code book").
        let g = grid_2x2(0.1, [0.5, 0.5, 0.5, 0.5]);
        assert_eq!(g.disposition(pair(3, 1)), Disposition::Escape);
        assert_eq!(g.disposition(pair(1, 3)), Disposition::Escape);
        assert_eq!(g.disposition(pair(99, 99)), Disposition::Escape);
    }

    #[test]
    fn disposition_at_exact_threshold_is_in_codebook() {
        // The cutoff is inclusive on the in-codebook side: `>=`.
        let g = grid_2x2(0.30, [0.10, 0.20, 0.30, 0.40]);
        assert_eq!(g.disposition(pair(2, 1)), Disposition::InCodebook);
    }

    // ---------- Counts ----------

    #[test]
    fn in_codebook_count_matches_strictly_above_or_equal() {
        let g = grid_2x2(0.25, [0.10, 0.20, 0.30, 0.40]);
        // Only (2,1) and (2,2) are >= 0.25.
        assert_eq!(g.in_codebook_count(), 2);
        assert_eq!(g.escape_count_in_rectangle(), 2);
    }

    #[test]
    fn counts_partition_the_rectangle() {
        // Total of the two counts always equals rm * ln.
        let g = grid_2x2(0.25, [0.10, 0.30, 0.20, 0.40]);
        assert_eq!(g.in_codebook_count() + g.escape_count_in_rectangle(), 4);
    }

    // ---------- Iteration ----------

    #[test]
    fn in_codebook_pairs_iterates_in_row_major_order() {
        // Probabilities arranged so the in-codebook positions are
        // (1, 2), (2, 1), (2, 2).
        let g = grid_2x2(0.25, [0.10, 0.30, 0.40, 0.50]);
        let collected: Vec<(u32, u32)> = g
            .in_codebook_pairs()
            .map(|p| (p.run, p.level.get()))
            .collect();
        assert_eq!(collected, vec![(1, 2), (2, 1), (2, 2)]);
    }

    #[test]
    fn in_codebook_pairs_is_empty_when_threshold_excludes_everything() {
        // threshold 1.0 with strictly-below-1 probabilities => empty.
        let g = grid_2x2(1.0, [0.10, 0.30, 0.40, 0.50]);
        let collected: Vec<RunLevelPair> = g.in_codebook_pairs().collect();
        assert!(collected.is_empty());
    }

    #[test]
    fn in_codebook_pairs_is_full_when_threshold_zero() {
        // threshold 0.0 includes every represented pair (since every
        // probability is >= 0.0). Patent's "all pairings above
        // threshold" reduces to "all pairings represented".
        let g = grid_2x2(0.0, [0.10, 0.30, 0.40, 0.50]);
        let collected: Vec<(u32, u32)> = g
            .in_codebook_pairs()
            .map(|p| (p.run, p.level.get()))
            .collect();
        assert_eq!(collected, vec![(1, 1), (1, 2), (2, 1), (2, 2)]);
    }

    #[test]
    fn in_codebook_pairs_count_matches_in_codebook_count_method() {
        let g = grid_2x2(0.25, [0.10, 0.30, 0.40, 0.50]);
        assert_eq!(g.in_codebook_pairs().count(), g.in_codebook_count());
    }

    // ---------- Disposition helpers ----------

    #[test]
    fn disposition_predicates_are_exclusive() {
        for disp in [Disposition::InCodebook, Disposition::Escape] {
            assert_ne!(disp.is_in_codebook(), disp.is_escape());
        }
    }

    // ---------- Cross-module: composes with runlevel ----------

    #[test]
    fn disposition_distinguishes_implicit_terminator_via_grid_state() {
        // The patent's (N, 1) implicit terminator (runlevel module) is
        // structurally orthogonal to the codebook disposition: an
        // implicit-terminator pair is dispositioned by the grid the
        // same way as any other (run, 1) pair would be. This test
        // pins that orthogonality so a future code path that mixes
        // the two layers does not accidentally couple them.
        let g = CodebookGrid::from_probabilities(8, 4, 0.5, vec![0.9; 32]).unwrap();
        let terminator = pair(8, 1); // (N, 1) for an N=8 block.
        assert!(terminator.is_implicit_terminator_for(8));
        // ...and the codebook still says "in codebook" because the
        // probability table says so. The terminator semantics live
        // in the runlevel walker, not the codebook.
        assert_eq!(g.disposition(terminator), Disposition::InCodebook);
    }

    // ---------- Error display naming ----------

    #[test]
    fn invalid_grid_display_names_each_variant() {
        let msgs = [
            format!("{}", InvalidGrid::ZeroRm),
            format!("{}", InvalidGrid::ZeroLn),
            format!("{}", InvalidGrid::ThresholdOutOfRange { threshold: 2.0 }),
            format!(
                "{}",
                InvalidGrid::DimensionsOverflow {
                    rm: u32::MAX,
                    ln: u32::MAX
                }
            ),
            format!(
                "{}",
                InvalidGrid::ProbabilityLengthMismatch {
                    expected: 4,
                    got: 3
                }
            ),
            format!(
                "{}",
                InvalidGrid::ProbabilityOutOfRange { probability: 1.5 }
            ),
        ];
        for msg in &msgs {
            assert!(msg.starts_with("oxideav-wma::codebook:"));
        }
    }
}

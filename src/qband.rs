//! WMA quantization-band descriptor.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 names the
//! patent-disclosed structural notion of a *quantization band* and
//! distinguishes it from the per-band coding-policy carrier already
//! exposed by [`crate::bands`]:
//!
//! > A **quantization band** is a contiguous frequency range of
//! > coefficients quantized with the same weighting.
//! >   — [PATENT US7,930,171 / US8,805,696 — quantization-band grouping]
//!
//! > A **quantization matrix** is "a set of weighting factors for series
//! > of values called quantization bands."
//! >   — [PATENT US7,930,171 — definition]
//!
//! Whereas [`crate::bands`] models *what is done* with a band (coded /
//! noise-substituted / truncated), and [`crate::invquant`] models the
//! per-coefficient `q * Q[d] * step` multiplication, this module models
//! the band *layout* itself: the contiguous coefficient ranges that
//! partition a block, each carrying a single weight-table index `d`.
//!
//! ## What this module provides
//!
//! * [`QuantBand`] — one contiguous coefficient range
//!   `start..start+length` plus the weight-table index `d` that all
//!   coefficients in the range share. Constructor enforces
//!   `length >= 1`, the patent's "contiguous range" precondition.
//! * [`QuantBandLayout`] — the ordered list of [`QuantBand`]s that
//!   partition a transform block. Constructor enforces the patent's
//!   contiguous-partition shape: bands form a covering tiling of
//!   `[0, total_coeffs)` with no gaps and no overlap, in ascending
//!   order, with at least one band when the block is non-empty.
//! * [`QuantBandLayout::band_map`] — materialises the per-coefficient
//!   band index `d(k)` that [`crate::invquant::dequantize_in_place`]
//!   consumes, threading the patent's "weight per band" through to the
//!   patent's "weight per coefficient" decoder step.
//!
//! ## What is NOT in this module
//!
//! * **The weight values themselves.** The matrix `Q[d]` is carried by
//!   [`crate::qmatrix`] (differential coding of the bitstream side
//!   information); the masking model that *computes* `Q[d]` at the
//!   encoder is encoder analysis and out of scope for the decoder.
//! * **Per-band coding policy.** Whether a given band is coded,
//!   noise-substituted, or truncated is `[`crate::bands`]`'s job; this
//!   module is purely the geometric partition.
//! * **The per-rate exponent-band partition tables themselves.** The
//!   trace's §9 lists these as `[GAP]` (table contents not staged); a
//!   real bitstream-driven layout will be assembled once those tables
//!   land in `docs/audio/wma/`.

use crate::block::BlockSize;

/// One contiguous quantization band: a coefficient range
/// `start..start+length` that all share the weight-table index `weight_index`.
///
/// Lifted from the patent's "contiguous frequency range of coefficients
/// quantized with the same weighting" definition
/// (US7,930,171 / US8,805,696).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantBand {
    /// First coefficient index covered by the band (inclusive).
    pub start: u16,
    /// Number of coefficients in the band; the patent's "contiguous
    /// range" requires `length >= 1`.
    pub length: u16,
    /// Index `d` into the weight table `Q[d]`. The bitstream carries
    /// the weights themselves via [`crate::qmatrix`].
    pub weight_index: u16,
}

/// Reasons [`QuantBand::new`] rejects its inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvalidQuantBand {
    /// `length == 0`. The patent definition requires a contiguous range,
    /// which is degenerate at zero length.
    ZeroLength,
    /// `start + length` would overflow `u16`. Transform-block sizes
    /// come from the patent set `{256..4096}` so this is safe in
    /// practice; the check is defensive.
    EndOverflow {
        /// The offending start.
        start: u16,
        /// The offending length.
        length: u16,
    },
}

impl core::fmt::Display for InvalidQuantBand {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidQuantBand::ZeroLength => f.write_str(
                "oxideav-wma::qband: zero-length quantization band rejected (patent-disclosed range is contiguous, length >= 1)",
            ),
            InvalidQuantBand::EndOverflow { start, length } => write!(
                f,
                "oxideav-wma::qband: quantization-band end overflows u16 (start={start}, length={length})",
            ),
        }
    }
}

impl std::error::Error for InvalidQuantBand {}

impl QuantBand {
    /// Build a [`QuantBand`] over `start..start+length` with weight
    /// index `weight_index`.
    pub fn new(start: u16, length: u16, weight_index: u16) -> Result<Self, InvalidQuantBand> {
        if length == 0 {
            return Err(InvalidQuantBand::ZeroLength);
        }
        if start.checked_add(length).is_none() {
            return Err(InvalidQuantBand::EndOverflow { start, length });
        }
        Ok(Self {
            start,
            length,
            weight_index,
        })
    }

    /// First coefficient index covered by the band (inclusive).
    #[inline]
    pub fn start(self) -> u16 {
        self.start
    }

    /// One-past-the-last coefficient index covered by the band.
    /// Guaranteed not to overflow `u16` by construction.
    #[inline]
    pub fn end(self) -> u16 {
        self.start + self.length
    }

    /// Number of coefficients in the band; always `>= 1`.
    #[inline]
    pub fn length(self) -> u16 {
        self.length
    }

    /// Weight-table index `d` shared by every coefficient in the band.
    #[inline]
    pub fn weight_index(self) -> u16 {
        self.weight_index
    }

    /// `true` if the coefficient index `k` falls inside this band.
    #[inline]
    pub fn contains(self, k: u16) -> bool {
        k >= self.start && k < self.end()
    }
}

/// An ordered partition of `[0, total_coeffs)` into [`QuantBand`]s.
///
/// Lifted from the patent's per-block "weighting factors for series of
/// values called quantization bands" — one weight per band, one band
/// table per block (US7,930,171 / US8,805,696).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QuantBandLayout {
    bands: Vec<QuantBand>,
    total_coeffs: usize,
}

/// Reasons [`QuantBandLayout::new`] rejects its inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvalidQuantBandLayout {
    /// `total_coeffs > 0` was supplied but the band list is empty,
    /// or `total_coeffs == 0` with a non-empty band list.
    BandCountMismatchEmptiness,
    /// The first band does not start at coefficient 0.
    LeadingGap {
        /// The unwanted start index.
        first_start: u16,
    },
    /// Two adjacent bands do not abut: `bands[i+1].start != bands[i].end()`.
    Gap {
        /// Index of the lower band in the pair.
        at_band: usize,
        /// Expected start (equal to lower band's end).
        expected_start: u16,
        /// Reported start of the next band.
        got_start: u16,
    },
    /// The total coverage does not match `total_coeffs` exactly.
    CoverageMismatch {
        /// Sum of all `length`s in the layout.
        covered: usize,
        /// The declared total.
        total_coeffs: usize,
    },
}

impl core::fmt::Display for InvalidQuantBandLayout {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidQuantBandLayout::BandCountMismatchEmptiness => f.write_str(
                "oxideav-wma::qband: empty band list paired with non-zero total_coeffs (or vice versa)",
            ),
            InvalidQuantBandLayout::LeadingGap { first_start } => write!(
                f,
                "oxideav-wma::qband: quantization-band layout does not start at coefficient 0 (first_start={first_start})",
            ),
            InvalidQuantBandLayout::Gap {
                at_band,
                expected_start,
                got_start,
            } => write!(
                f,
                "oxideav-wma::qband: gap or overlap between adjacent quantization bands at index {at_band} (expected next start {expected_start}, got {got_start})",
            ),
            InvalidQuantBandLayout::CoverageMismatch {
                covered,
                total_coeffs,
            } => write!(
                f,
                "oxideav-wma::qband: quantization-band coverage {covered} does not match total_coeffs {total_coeffs}",
            ),
        }
    }
}

impl std::error::Error for InvalidQuantBandLayout {}

impl QuantBandLayout {
    /// Build a layout from an ordered list of bands plus the declared
    /// total coefficient count. The layout must tile `[0, total_coeffs)`
    /// exactly (no gaps, no overlap, no trailing slack).
    pub fn new(bands: Vec<QuantBand>, total_coeffs: usize) -> Result<Self, InvalidQuantBandLayout> {
        if bands.is_empty() != (total_coeffs == 0) {
            return Err(InvalidQuantBandLayout::BandCountMismatchEmptiness);
        }
        if let Some(first) = bands.first() {
            if first.start != 0 {
                return Err(InvalidQuantBandLayout::LeadingGap {
                    first_start: first.start,
                });
            }
        }
        for i in 0..bands.len().saturating_sub(1) {
            let cur = bands[i];
            let next = bands[i + 1];
            if next.start != cur.end() {
                return Err(InvalidQuantBandLayout::Gap {
                    at_band: i,
                    expected_start: cur.end(),
                    got_start: next.start,
                });
            }
        }
        let covered: usize = bands.iter().map(|b| b.length as usize).sum();
        if covered != total_coeffs {
            return Err(InvalidQuantBandLayout::CoverageMismatch {
                covered,
                total_coeffs,
            });
        }
        Ok(Self {
            bands,
            total_coeffs,
        })
    }

    /// Build a layout that spans an entire [`BlockSize`] with the bands
    /// supplied. Convenience wrapper around [`QuantBandLayout::new`].
    pub fn for_block(
        bands: Vec<QuantBand>,
        block: BlockSize,
    ) -> Result<Self, InvalidQuantBandLayout> {
        Self::new(bands, block.samples() as usize)
    }

    /// Number of bands in the layout.
    pub fn band_count(&self) -> usize {
        self.bands.len()
    }

    /// Total coefficients covered (equal to the value supplied at
    /// construction).
    pub fn total_coeffs(&self) -> usize {
        self.total_coeffs
    }

    /// Whether the layout is empty (`total_coeffs == 0`).
    pub fn is_empty(&self) -> bool {
        self.bands.is_empty()
    }

    /// Iterate over the bands in ascending coefficient order.
    pub fn bands(&self) -> impl Iterator<Item = &QuantBand> {
        self.bands.iter()
    }

    /// Look up the band at slot `i`, if any.
    pub fn band(&self, i: usize) -> Option<&QuantBand> {
        self.bands.get(i)
    }

    /// Find the band containing coefficient `k`, returning its slot
    /// index. `None` if `k >= total_coeffs`.
    ///
    /// Linear scan; suitable for diagnostic use. For dequant-loop use
    /// build a band map via [`QuantBandLayout::band_map`].
    pub fn band_slot_of(&self, k: u16) -> Option<usize> {
        if (k as usize) >= self.total_coeffs {
            return None;
        }
        self.bands.iter().position(|b| b.contains(k))
    }

    /// Look up the weight-table index `d` for coefficient `k`. `None`
    /// if `k >= total_coeffs`.
    pub fn weight_index_of(&self, k: u16) -> Option<u16> {
        self.band_slot_of(k).map(|i| self.bands[i].weight_index)
    }

    /// Materialise the per-coefficient weight-index map `d(k)` for use
    /// with [`crate::invquant::dequantize_in_place`].
    ///
    /// `band_map[k] == self.bands[i].weight_index` where `i` is the
    /// slot containing `k`. The vector has length `total_coeffs`.
    pub fn band_map(&self) -> Vec<u16> {
        let mut out = Vec::with_capacity(self.total_coeffs);
        for band in &self.bands {
            for _ in 0..band.length {
                out.push(band.weight_index);
            }
        }
        out
    }

    /// How many bands resolve to a given weight-table index `d`.
    /// The patent allows different bands to share a weight index when
    /// the encoder so chooses.
    pub fn bands_referencing_weight(&self, weight_index: u16) -> usize {
        self.bands
            .iter()
            .filter(|b| b.weight_index == weight_index)
            .count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::BlockSize;

    // ---------- QuantBand constructor ----------

    #[test]
    fn quant_band_constructor_accepts_minimal_band() {
        let b = QuantBand::new(0, 1, 0).expect("length 1 is the contiguous minimum");
        assert_eq!(b.start(), 0);
        assert_eq!(b.length(), 1);
        assert_eq!(b.end(), 1);
        assert_eq!(b.weight_index(), 0);
    }

    #[test]
    fn quant_band_constructor_rejects_zero_length() {
        let err = QuantBand::new(0, 0, 0).unwrap_err();
        assert_eq!(err, InvalidQuantBand::ZeroLength);
    }

    #[test]
    fn quant_band_constructor_rejects_end_overflow() {
        // u16::MAX + 1 overflows u16.
        let err = QuantBand::new(u16::MAX, 1, 0).unwrap_err();
        assert_eq!(
            err,
            InvalidQuantBand::EndOverflow {
                start: u16::MAX,
                length: 1,
            },
        );
    }

    #[test]
    fn quant_band_constructor_accepts_max_end() {
        // start + length == u16::MAX is allowed (end is one-past-last).
        let b = QuantBand::new(0, u16::MAX, 7).expect("end at u16::MAX is in range");
        assert_eq!(b.end(), u16::MAX);
        assert_eq!(b.length(), u16::MAX);
    }

    // ---------- QuantBand contains ----------

    #[test]
    fn quant_band_contains_inclusive_start_exclusive_end() {
        let b = QuantBand::new(5, 3, 0).unwrap();
        assert!(!b.contains(4));
        assert!(b.contains(5));
        assert!(b.contains(6));
        assert!(b.contains(7));
        assert!(!b.contains(8));
    }

    // ---------- Layout constructor accept ----------

    #[test]
    fn layout_accepts_single_band_covering_block() {
        let bands = vec![QuantBand::new(0, 4, 0).unwrap()];
        let layout = QuantBandLayout::new(bands, 4).unwrap();
        assert_eq!(layout.band_count(), 1);
        assert_eq!(layout.total_coeffs(), 4);
        assert!(!layout.is_empty());
    }

    #[test]
    fn layout_accepts_two_abutting_bands() {
        let bands = vec![
            QuantBand::new(0, 4, 0).unwrap(),
            QuantBand::new(4, 4, 1).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 8).unwrap();
        assert_eq!(layout.band_count(), 2);
        assert_eq!(layout.total_coeffs(), 8);
    }

    #[test]
    fn layout_accepts_empty_block() {
        let layout = QuantBandLayout::new(vec![], 0).unwrap();
        assert!(layout.is_empty());
        assert_eq!(layout.band_count(), 0);
        assert_eq!(layout.total_coeffs(), 0);
        assert_eq!(layout.band_map(), Vec::<u16>::new());
    }

    #[test]
    fn layout_for_block_accepts_full_block_size_partition() {
        // 256-sample block partitioned into four equal quarters.
        let bands = vec![
            QuantBand::new(0, 64, 0).unwrap(),
            QuantBand::new(64, 64, 1).unwrap(),
            QuantBand::new(128, 64, 2).unwrap(),
            QuantBand::new(192, 64, 3).unwrap(),
        ];
        let layout = QuantBandLayout::for_block(bands, BlockSize::S256).unwrap();
        assert_eq!(layout.total_coeffs(), 256);
    }

    // ---------- Layout constructor reject ----------

    #[test]
    fn layout_rejects_empty_bands_with_nonzero_total() {
        let err = QuantBandLayout::new(vec![], 4).unwrap_err();
        assert_eq!(err, InvalidQuantBandLayout::BandCountMismatchEmptiness);
    }

    #[test]
    fn layout_rejects_nonempty_bands_with_zero_total() {
        let bands = vec![QuantBand::new(0, 1, 0).unwrap()];
        let err = QuantBandLayout::new(bands, 0).unwrap_err();
        assert_eq!(err, InvalidQuantBandLayout::BandCountMismatchEmptiness);
    }

    #[test]
    fn layout_rejects_leading_gap() {
        let bands = vec![QuantBand::new(2, 4, 0).unwrap()];
        let err = QuantBandLayout::new(bands, 6).unwrap_err();
        assert_eq!(err, InvalidQuantBandLayout::LeadingGap { first_start: 2 },);
    }

    #[test]
    fn layout_rejects_gap_between_bands() {
        let bands = vec![
            QuantBand::new(0, 4, 0).unwrap(),
            QuantBand::new(5, 4, 1).unwrap(), // gap at coefficient 4
        ];
        let err = QuantBandLayout::new(bands, 9).unwrap_err();
        assert_eq!(
            err,
            InvalidQuantBandLayout::Gap {
                at_band: 0,
                expected_start: 4,
                got_start: 5,
            },
        );
    }

    #[test]
    fn layout_rejects_overlap_between_bands() {
        let bands = vec![
            QuantBand::new(0, 4, 0).unwrap(),
            QuantBand::new(3, 4, 1).unwrap(), // overlaps positions 3
        ];
        let err = QuantBandLayout::new(bands, 7).unwrap_err();
        assert_eq!(
            err,
            InvalidQuantBandLayout::Gap {
                at_band: 0,
                expected_start: 4,
                got_start: 3,
            },
        );
    }

    #[test]
    fn layout_rejects_coverage_mismatch_below() {
        let bands = vec![QuantBand::new(0, 4, 0).unwrap()];
        let err = QuantBandLayout::new(bands, 8).unwrap_err();
        assert_eq!(
            err,
            InvalidQuantBandLayout::CoverageMismatch {
                covered: 4,
                total_coeffs: 8,
            },
        );
    }

    #[test]
    fn layout_rejects_coverage_mismatch_above() {
        let bands = vec![
            QuantBand::new(0, 4, 0).unwrap(),
            QuantBand::new(4, 4, 1).unwrap(),
        ];
        let err = QuantBandLayout::new(bands, 6).unwrap_err();
        assert_eq!(
            err,
            InvalidQuantBandLayout::CoverageMismatch {
                covered: 8,
                total_coeffs: 6,
            },
        );
    }

    // ---------- Layout accessors ----------

    #[test]
    fn layout_band_slot_of_locates_each_coefficient() {
        let bands = vec![
            QuantBand::new(0, 2, 10).unwrap(),
            QuantBand::new(2, 3, 11).unwrap(),
            QuantBand::new(5, 1, 12).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 6).unwrap();
        assert_eq!(layout.band_slot_of(0), Some(0));
        assert_eq!(layout.band_slot_of(1), Some(0));
        assert_eq!(layout.band_slot_of(2), Some(1));
        assert_eq!(layout.band_slot_of(3), Some(1));
        assert_eq!(layout.band_slot_of(4), Some(1));
        assert_eq!(layout.band_slot_of(5), Some(2));
        assert_eq!(layout.band_slot_of(6), None);
    }

    #[test]
    fn layout_weight_index_of_routes_via_band_slot() {
        let bands = vec![
            QuantBand::new(0, 2, 10).unwrap(),
            QuantBand::new(2, 3, 11).unwrap(),
            QuantBand::new(5, 1, 12).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 6).unwrap();
        assert_eq!(layout.weight_index_of(0), Some(10));
        assert_eq!(layout.weight_index_of(2), Some(11));
        assert_eq!(layout.weight_index_of(5), Some(12));
        assert_eq!(layout.weight_index_of(6), None);
    }

    #[test]
    fn layout_band_lookup_returns_the_stored_record() {
        let bands = vec![
            QuantBand::new(0, 2, 10).unwrap(),
            QuantBand::new(2, 4, 20).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 6).unwrap();
        assert_eq!(
            layout.band(0).copied(),
            Some(QuantBand::new(0, 2, 10).unwrap())
        );
        assert_eq!(
            layout.band(1).copied(),
            Some(QuantBand::new(2, 4, 20).unwrap())
        );
        assert_eq!(layout.band(2), None);
    }

    #[test]
    fn layout_bands_iterator_yields_ascending_order() {
        let bands = vec![
            QuantBand::new(0, 1, 0).unwrap(),
            QuantBand::new(1, 2, 1).unwrap(),
            QuantBand::new(3, 3, 2).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands.clone(), 6).unwrap();
        let collected: Vec<QuantBand> = layout.bands().copied().collect();
        assert_eq!(collected, bands);
    }

    // ---------- Layout band_map ----------

    #[test]
    fn layout_band_map_materialises_per_coefficient_weight_index() {
        // Three bands carrying weight indices 0, 1, 2 with sizes 2, 3, 1.
        let bands = vec![
            QuantBand::new(0, 2, 0).unwrap(),
            QuantBand::new(2, 3, 1).unwrap(),
            QuantBand::new(5, 1, 2).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 6).unwrap();
        let band_map = layout.band_map();
        assert_eq!(band_map, vec![0_u16, 0, 1, 1, 1, 2]);
    }

    #[test]
    fn layout_band_map_round_trips_with_weight_index_of() {
        let bands = vec![
            QuantBand::new(0, 4, 5).unwrap(),
            QuantBand::new(4, 4, 7).unwrap(),
            QuantBand::new(8, 8, 9).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 16).unwrap();
        let band_map = layout.band_map();
        for k in 0..16_u16 {
            assert_eq!(
                Some(band_map[k as usize]),
                layout.weight_index_of(k),
                "k={k}",
            );
        }
    }

    // ---------- Cross-module wiring with invquant ----------

    #[test]
    fn layout_band_map_drives_invquant_dequantize_in_place() {
        // The patent's "weight per band" arrangement threads through to
        // the "weight per coefficient" dequant step via this band_map.
        // Build a small layout, hand its map to invquant, and check the
        // per-coefficient product matches the per-band weight.
        let bands = vec![
            QuantBand::new(0, 2, 0).unwrap(), // weight 2.0
            QuantBand::new(2, 2, 1).unwrap(), // weight 3.0
        ];
        let layout = QuantBandLayout::new(bands, 4).unwrap();
        let band_map = layout.band_map();
        let weights = [2.0_f64, 3.0];
        let step = 5.0_f64;
        let q = [1_i32, 2, 3, 4];
        let mut out = [0.0_f64; 4];
        crate::invquant::dequantize_in_place(&q, &band_map, &weights, step, &mut out);
        assert_eq!(out[0], 1.0 * 2.0 * 5.0);
        assert_eq!(out[1], 2.0 * 2.0 * 5.0);
        assert_eq!(out[2], 3.0 * 3.0 * 5.0);
        assert_eq!(out[3], 4.0 * 3.0 * 5.0);
    }

    // ---------- bands_referencing_weight ----------

    #[test]
    fn layout_counts_bands_referencing_a_given_weight() {
        // Two bands share weight index 5; one uses weight index 7.
        let bands = vec![
            QuantBand::new(0, 2, 5).unwrap(),
            QuantBand::new(2, 2, 7).unwrap(),
            QuantBand::new(4, 2, 5).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, 6).unwrap();
        assert_eq!(layout.bands_referencing_weight(5), 2);
        assert_eq!(layout.bands_referencing_weight(7), 1);
        assert_eq!(layout.bands_referencing_weight(99), 0);
    }

    // ---------- Cross-module wiring with BlockSize ----------

    #[test]
    fn layout_for_block_accepts_all_patent_block_sizes() {
        // Every BlockSize from the patent set can host a trivial
        // single-band layout. This ties the qband partition shape to
        // the patent-disclosed transform-block-size set explicitly.
        for size in BlockSize::ALL {
            let samples = size.samples();
            let bands = vec![QuantBand::new(0, samples, 0).unwrap()];
            let layout =
                QuantBandLayout::for_block(bands, size).expect("trivial single-band layout");
            assert_eq!(layout.total_coeffs(), samples as usize);
            assert_eq!(layout.band_count(), 1);
        }
    }

    // ---------- Error display ----------

    #[test]
    fn invalid_quant_band_display_names_each_variant() {
        let s = format!("{}", InvalidQuantBand::ZeroLength);
        assert!(s.contains("zero-length"));
        assert!(s.contains("oxideav-wma::qband"));
        let s = format!(
            "{}",
            InvalidQuantBand::EndOverflow {
                start: 100,
                length: 200,
            },
        );
        assert!(s.contains("overflow"));
        assert!(s.contains("start=100"));
        assert!(s.contains("length=200"));
    }

    #[test]
    fn invalid_quant_band_layout_display_names_each_variant() {
        let s = format!("{}", InvalidQuantBandLayout::BandCountMismatchEmptiness);
        assert!(s.contains("empty"));
        let s = format!("{}", InvalidQuantBandLayout::LeadingGap { first_start: 3 },);
        assert!(s.contains("start at coefficient 0"));
        assert!(s.contains("first_start=3"));
        let s = format!(
            "{}",
            InvalidQuantBandLayout::Gap {
                at_band: 2,
                expected_start: 8,
                got_start: 9,
            },
        );
        assert!(s.contains("gap or overlap"));
        assert!(s.contains("index 2"));
        let s = format!(
            "{}",
            InvalidQuantBandLayout::CoverageMismatch {
                covered: 4,
                total_coeffs: 8,
            },
        );
        assert!(s.contains("coverage 4"));
        assert!(s.contains("total_coeffs 8"));
    }
}

//! WMA excitation-pattern / energy-derived quantization matrix.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 ("Quantization
//! matrix (the \"weighting factors\" / scalefactor analogue)") lifts the
//! patent-disclosed formula that derives the per-band quantization
//! matrix `Q[c][d]` from the block's MLT coefficients:
//!
//! > For WMA7 the matrix is computed per channel as `Q[c][d] = E[d]`,
//! > where `E[d]` is an excitation pattern: coefficient values are
//! > squared to get energies, then energies are summed within each
//! > band; the matrix "spreads distortion between bands in proportion
//! > to the energies of the bands."
//! >   — [PATENT US7,930,171 — WMA7 formula, Background]
//!
//! > Because bands differ in size, WMA7 adjusts the matrix by band size
//! > (divide by the coefficient count `Card{B[d]}` raised to an
//! > experimentally-derived exponent).
//! >   — [PATENT US7,930,171 — formula (3)]
//!
//! ## Scope of this module
//!
//! The patent discloses the *parametric form* of the excitation-pattern
//! computation completely:
//!
//! 1. Square each MLT coefficient → per-coefficient energy.
//! 2. Sum the energies of the coefficients within a quantization band →
//!    the band's raw energy `E_raw[d]`.
//! 3. Adjust by band size: divide by `Card{B[d]}^exponent`, where
//!    `Card{B[d]}` is the number of coefficients in the band.
//!
//! The result `E[d]` is the excitation pattern, and the patent states
//! `Q[c][d] = E[d]` — so the per-band excitation values are the
//! quantization-matrix weights the bitstream later carries as side
//! information (via [`crate::qmatrix`]) and the decoder folds into the
//! per-coefficient dequant step (via [`crate::invquant::BandScale`]).
//!
//! This module exposes the band-energy step ([`band_energies`]) and the
//! full excitation-pattern derivation ([`excitation_pattern`]) over a
//! [`crate::qband::QuantBandLayout`], plus a per-band primitive
//! ([`band_excitation`]).
//!
//! ## The exponent is a `[GAP]` value, supplied by the caller
//!
//! The patent says the band-size adjustment divides by `Card{B[d]}`
//! "raised to an **experimentally-derived** exponent". The *numeric
//! value* of that exponent is not disclosed in the trace — it is an
//! encoder analysis constant, marked `[GAP]` in §9 alongside the other
//! encoder-tuned quantities. This module therefore does **not** invent
//! an exponent: every helper takes the exponent as a caller-supplied
//! `f64` parameter, exactly as the other modules accept the opaque
//! step size ([`crate::step_size`]) and the opaque matrix weights
//! ([`crate::invquant`]). Two boundary exponents have closed-form
//! meaning the patent text frames directly:
//!
//! * `exponent == 0.0` → no band-size adjustment (`Card^0 == 1`); the
//!   excitation pattern is the raw summed energy. This is the
//!   un-normalised baseline the patent then "adjusts by band size".
//! * `exponent == 1.0` → divide by `Card{B[d]}` exactly; the excitation
//!   pattern is the *mean* per-coefficient energy of the band.
//!
//! All other exponents interpolate between those endpoints; the
//! shipping encoder's choice stays `[GAP]`.
//!
//! ## What is NOT in this module
//!
//! * **The encoder's exponent constant.** `[GAP]` per the trace — never
//!   fabricated; always a caller parameter.
//! * **The Bark-scale masking model.** §4 names the masking curve
//!   (`-25 dB/Bark` left, `+10 dB/Bark` right, optional β whitening) as
//!   the analysis that *shapes* the final weighting, but it operates on
//!   top of the excitation pattern and the patent marks its constants
//!   as encoder analysis. This module computes only the energy-derived
//!   excitation pattern the masking model starts from.
//! * **Quantization / differential coding / Huffman of the matrix.**
//!   Once `Q[d]` is derived it is uniformly quantized (step 110),
//!   differentially coded ([`crate::qmatrix`], step 120) and
//!   Huffman-coded (step 130, table contents `[GAP]`) for transmission.
//!   Those stages live in their own modules; this one produces the
//!   pre-quantization excitation values.

use crate::qband::QuantBandLayout;

/// Per-coefficient energy: the square of an MLT coefficient.
///
/// The patent's step 1 — "coefficient values are squared to get
/// energies". Exposed as a named helper so the squaring convention is
/// pinned in one place.
#[inline]
pub fn coefficient_energy(coeff: f64) -> f64 {
    coeff * coeff
}

/// Raw energy of a single band: the sum of the squared coefficients in
/// the slice.
///
/// This is the patent's steps 1–2 applied to one band's coefficients —
/// square each coefficient, sum within the band — *without* the
/// band-size adjustment. An empty slice has zero energy.
pub fn band_raw_energy(coeffs: &[f64]) -> f64 {
    coeffs.iter().map(|&c| coefficient_energy(c)).sum()
}

/// Excitation value `E[d]` for a single band.
///
/// Applies the full patent formula to one band's coefficients: sum the
/// squared coefficients (raw energy), then divide by
/// `Card{B[d]}^exponent` where `Card{B[d]} == coeffs.len()`.
///
/// The `exponent` is the patent's experimentally-derived band-size
/// adjustment power — a caller-supplied `[GAP]` value (see the module
/// docs). `exponent == 0.0` returns the raw summed energy; `1.0`
/// returns the mean per-coefficient energy.
///
/// An empty band has `Card{B[d]} == 0`. For any positive exponent
/// `0^exponent == 0`, so dividing by it is undefined; this function
/// returns `0.0` for an empty band rather than producing a NaN or an
/// infinity. (A real layout never contains an empty band —
/// [`crate::qband::QuantBand`] enforces `length >= 1` at construction —
/// so this is a defensive boundary only.)
pub fn band_excitation(coeffs: &[f64], exponent: f64) -> f64 {
    let card = coeffs.len();
    if card == 0 {
        return 0.0;
    }
    let raw = band_raw_energy(coeffs);
    // `Card^exponent`; for exponent == 0 this is exactly 1.0 so the
    // raw energy passes through unchanged.
    let divisor = (card as f64).powf(exponent);
    raw / divisor
}

/// Compute the per-band **raw** energies of a transform block, without
/// the band-size adjustment.
///
/// `coeffs` are the block's MLT coefficients in coefficient order;
/// `layout` partitions them into quantization bands. The returned vector
/// has one entry per band (`layout.band_count()` entries), each the sum
/// of the squared coefficients in that band.
///
/// # Panics
///
/// Panics if `coeffs.len() != layout.total_coeffs()`. The layout must
/// tile exactly the coefficients supplied.
pub fn band_energies(coeffs: &[f64], layout: &QuantBandLayout) -> Vec<f64> {
    assert_eq!(
        coeffs.len(),
        layout.total_coeffs(),
        "oxideav-wma::excitation::band_energies: coefficient count {} does not match layout total {}",
        coeffs.len(),
        layout.total_coeffs(),
    );
    let mut out = Vec::with_capacity(layout.band_count());
    for band in layout.bands() {
        let start = band.start as usize;
        let end = band.end() as usize;
        out.push(band_raw_energy(&coeffs[start..end]));
    }
    out
}

/// Compute the per-band **excitation pattern** `E[d]` of a transform
/// block — the patent's full formula including the band-size adjustment.
///
/// `coeffs` are the block's MLT coefficients in coefficient order;
/// `layout` partitions them into quantization bands; `exponent` is the
/// patent's experimentally-derived band-size adjustment power (a
/// caller-supplied `[GAP]` value — see the module docs). The returned
/// vector has one entry per band; `out[d] == band_excitation(band d's
/// coefficients, exponent)`.
///
/// Per the patent, `Q[c][d] = E[d]`, so these are the quantization-matrix
/// weights for the block.
///
/// # Panics
///
/// Panics if `coeffs.len() != layout.total_coeffs()`.
pub fn excitation_pattern(coeffs: &[f64], layout: &QuantBandLayout, exponent: f64) -> Vec<f64> {
    assert_eq!(
        coeffs.len(),
        layout.total_coeffs(),
        "oxideav-wma::excitation::excitation_pattern: coefficient count {} does not match layout total {}",
        coeffs.len(),
        layout.total_coeffs(),
    );
    let mut out = Vec::with_capacity(layout.band_count());
    for band in layout.bands() {
        let start = band.start as usize;
        let end = band.end() as usize;
        out.push(band_excitation(&coeffs[start..end], exponent));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::BlockSize;
    use crate::qband::{QuantBand, QuantBandLayout};

    // ---------- coefficient_energy ----------

    #[test]
    fn coefficient_energy_is_the_square() {
        assert_eq!(coefficient_energy(0.0), 0.0);
        assert_eq!(coefficient_energy(1.0), 1.0);
        assert_eq!(coefficient_energy(2.0), 4.0);
        assert_eq!(coefficient_energy(-3.0), 9.0);
        assert_eq!(coefficient_energy(0.5), 0.25);
    }

    #[test]
    fn coefficient_energy_is_sign_independent() {
        // Squaring discards the sign — the patent's energy is a magnitude.
        for c in [0.25_f64, 1.5, 7.0, 100.0] {
            assert_eq!(coefficient_energy(c), coefficient_energy(-c), "c={c}");
        }
    }

    // ---------- band_raw_energy ----------

    #[test]
    fn band_raw_energy_sums_squares() {
        // 1^2 + 2^2 + 3^2 = 1 + 4 + 9 = 14
        assert_eq!(band_raw_energy(&[1.0, 2.0, 3.0]), 14.0);
    }

    #[test]
    fn band_raw_energy_of_empty_is_zero() {
        assert_eq!(band_raw_energy(&[]), 0.0);
    }

    #[test]
    fn band_raw_energy_ignores_signs() {
        let pos = band_raw_energy(&[1.0, 2.0, 3.0, 4.0]);
        let mixed = band_raw_energy(&[-1.0, 2.0, -3.0, 4.0]);
        assert_eq!(pos, mixed);
    }

    #[test]
    fn band_raw_energy_of_all_zeros_is_zero() {
        assert_eq!(band_raw_energy(&[0.0; 8]), 0.0);
    }

    // ---------- band_excitation ----------

    #[test]
    fn band_excitation_exponent_zero_is_raw_energy() {
        // Card^0 == 1 → no band-size adjustment.
        let coeffs = [1.0, 2.0, 3.0];
        assert_eq!(band_excitation(&coeffs, 0.0), band_raw_energy(&coeffs));
        assert_eq!(band_excitation(&coeffs, 0.0), 14.0);
    }

    #[test]
    fn band_excitation_exponent_one_is_mean_energy() {
        // Card^1 == Card → divide by coefficient count → mean energy.
        // (1 + 4 + 9) / 3 = 14 / 3.
        let coeffs = [1.0, 2.0, 3.0];
        let expected = 14.0 / 3.0;
        assert!((band_excitation(&coeffs, 1.0) - expected).abs() < 1e-12);
    }

    #[test]
    fn band_excitation_single_coefficient_is_exponent_independent() {
        // Card == 1 → 1^exponent == 1 for every exponent, so the
        // excitation equals the lone coefficient's energy regardless.
        for e in [0.0_f64, 0.5, 1.0, 1.7, 3.0] {
            assert_eq!(band_excitation(&[5.0], e), 25.0, "exponent={e}");
        }
    }

    #[test]
    fn band_excitation_half_exponent_divides_by_sqrt_card() {
        // Card^0.5 == sqrt(Card). Four coefficients → divide by 2.
        // raw = 1 + 1 + 1 + 1 = 4; 4 / sqrt(4) = 4 / 2 = 2.
        let coeffs = [1.0, 1.0, 1.0, 1.0];
        assert!((band_excitation(&coeffs, 0.5) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn band_excitation_empty_band_is_zero_not_nan() {
        // Defensive boundary: a real QuantBand has length >= 1, but the
        // primitive must not produce NaN/inf for an empty slice.
        for e in [0.0_f64, 0.5, 1.0, 2.0] {
            let v = band_excitation(&[], e);
            assert_eq!(v, 0.0, "exponent={e}");
            assert!(v.is_finite());
        }
    }

    #[test]
    fn band_excitation_larger_band_with_same_per_coeff_energy_scales_with_card_power() {
        // Two bands, each coefficient = 1.0 (energy 1.0 each).
        // Band A: 2 coeffs, raw = 2. Band B: 8 coeffs, raw = 8.
        // With exponent 1.0 both normalise to the same mean (1.0).
        let a = band_excitation(&[1.0, 1.0], 1.0);
        let b = band_excitation(&[1.0; 8], 1.0);
        assert!((a - 1.0).abs() < 1e-12);
        assert!((b - 1.0).abs() < 1e-12);
        assert!((a - b).abs() < 1e-12);
    }

    // ---------- band_energies over a layout ----------

    fn two_band_layout() -> QuantBandLayout {
        // Band 0: coeffs 0..2 (weight 0); band 1: coeffs 2..6 (weight 1).
        let bands = vec![
            QuantBand::new(0, 2, 0).unwrap(),
            QuantBand::new(2, 4, 1).unwrap(),
        ];
        QuantBandLayout::new(bands, 6).unwrap()
    }

    #[test]
    fn band_energies_partitions_the_block() {
        let layout = two_band_layout();
        let coeffs = [1.0, 2.0, 1.0, 1.0, 1.0, 1.0];
        let e = band_energies(&coeffs, &layout);
        // Band 0: 1 + 4 = 5. Band 1: 1 + 1 + 1 + 1 = 4.
        assert_eq!(e, vec![5.0, 4.0]);
    }

    #[test]
    fn band_energies_has_one_entry_per_band() {
        let layout = two_band_layout();
        let coeffs = [0.0; 6];
        let e = band_energies(&coeffs, &layout);
        assert_eq!(e.len(), layout.band_count());
        assert_eq!(e, vec![0.0, 0.0]);
    }

    #[test]
    fn band_energies_empty_layout_is_empty() {
        let layout = QuantBandLayout::new(vec![], 0).unwrap();
        let e = band_energies(&[], &layout);
        assert!(e.is_empty());
    }

    #[test]
    #[should_panic(expected = "does not match layout total")]
    fn band_energies_panics_on_count_mismatch() {
        let layout = two_band_layout();
        let coeffs = [1.0; 5]; // layout total is 6
        let _ = band_energies(&coeffs, &layout);
    }

    // ---------- excitation_pattern over a layout ----------

    #[test]
    fn excitation_pattern_exponent_zero_equals_band_energies() {
        let layout = two_band_layout();
        let coeffs = [1.0, 2.0, 3.0, 0.0, 0.0, 4.0];
        let raw = band_energies(&coeffs, &layout);
        let exc = excitation_pattern(&coeffs, &layout, 0.0);
        assert_eq!(exc, raw);
    }

    #[test]
    fn excitation_pattern_applies_per_band_card_adjustment() {
        let layout = two_band_layout();
        // Band 0: coeffs [2, 2] → raw 8, Card 2.
        // Band 1: coeffs [1, 1, 1, 1] → raw 4, Card 4.
        let coeffs = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0];
        let exc = excitation_pattern(&coeffs, &layout, 1.0);
        // Band 0 mean: 8 / 2 = 4. Band 1 mean: 4 / 4 = 1.
        assert!((exc[0] - 4.0).abs() < 1e-12);
        assert!((exc[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn excitation_pattern_matches_per_band_primitive() {
        // The layout-level helper must equal calling band_excitation on
        // each band's coefficient slice.
        let layout = two_band_layout();
        let coeffs = [3.0, 1.0, 4.0, 1.0, 5.0, 9.0];
        for exponent in [0.0_f64, 0.5, 1.0, 1.5] {
            let exc = excitation_pattern(&coeffs, &layout, exponent);
            let band0 = band_excitation(&coeffs[0..2], exponent);
            let band1 = band_excitation(&coeffs[2..6], exponent);
            assert_eq!(exc, vec![band0, band1], "exponent={exponent}");
        }
    }

    #[test]
    fn excitation_pattern_zero_block_is_all_zeros() {
        let layout = two_band_layout();
        let coeffs = [0.0; 6];
        for exponent in [0.0_f64, 0.5, 1.0, 2.0] {
            let exc = excitation_pattern(&coeffs, &layout, exponent);
            assert_eq!(exc, vec![0.0, 0.0], "exponent={exponent}");
        }
    }

    #[test]
    fn excitation_pattern_spreads_in_proportion_to_band_energy() {
        // The patent: the matrix "spreads distortion between bands in
        // proportion to the energies of the bands." With exponent 0 the
        // excitation IS the band energy, so a band with 4x the energy of
        // another reports 4x the excitation.
        let layout = two_band_layout();
        // Band 0: [2, 0] raw = 4. Band 1: [1, 1, 1, 1] raw = 4 → equal.
        // Make band 0 carry 4x: [4, 0] raw = 16 vs band1 raw = 4.
        let coeffs = [4.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let exc = excitation_pattern(&coeffs, &layout, 0.0);
        assert_eq!(exc[0], 16.0);
        assert_eq!(exc[1], 4.0);
        assert!((exc[0] / exc[1] - 4.0).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "does not match layout total")]
    fn excitation_pattern_panics_on_count_mismatch() {
        let layout = two_band_layout();
        let coeffs = [1.0; 7]; // layout total is 6
        let _ = excitation_pattern(&coeffs, &layout, 1.0);
    }

    // ---------- Cross-module: excitation -> Q[d] -> BandScale ----------

    #[test]
    fn excitation_pattern_feeds_band_scale_as_quantization_matrix() {
        // Patent: Q[c][d] = E[d]. The excitation pattern IS the
        // per-band weight table the decoder folds with the overall step
        // into a BandScale. Thread the values through end-to-end.
        use crate::invquant::BandScale;

        let layout = two_band_layout();
        let coeffs = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0];
        let weights = excitation_pattern(&coeffs, &layout, 1.0); // [4.0, 1.0]
        let step = 0.5;
        let scale = BandScale::from_weights(&weights, step);
        // scale[d] = Q[d] * step.
        assert_eq!(scale.get(0), Some(2.0)); // 4.0 * 0.5
        assert_eq!(scale.get(1), Some(0.5)); // 1.0 * 0.5
    }

    #[test]
    fn excitation_pattern_covers_a_full_block_size() {
        // Use the smallest patent block size end-to-end: one band per
        // 32-coefficient stripe across the 256-sample block's 256 MLT
        // coefficients (the MLT yields M coefficients for a 2M frame; we
        // treat the layout total as the block's sample count here purely
        // to exercise the for_block path's coverage check).
        let block = BlockSize::S256;
        let total = block.samples() as usize; // 256
        let mut bands = Vec::new();
        let mut start = 0u16;
        while (start as usize) < total {
            bands.push(QuantBand::new(start, 32, 0).unwrap());
            start += 32;
        }
        let layout = QuantBandLayout::for_block(bands, block).unwrap();
        let coeffs = vec![1.0_f64; total];
        let exc = excitation_pattern(&coeffs, &layout, 1.0);
        // Every band has 32 unit coefficients → mean energy 1.0.
        assert_eq!(exc.len(), total / 32);
        for v in exc {
            assert!((v - 1.0).abs() < 1e-12);
        }
    }
}

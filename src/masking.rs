//! Encoder-side Bark-scale masking model for the weighting function.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 ("Masking model"):
//!
//! > The weighting function `w(k)` follows an auditory **masking curve
//! > computed on the Bark scale**, with a simplified asymmetric
//! > spreading function (−25 dB/Bark left, +10 dB/Bark right) and an
//! > optional **partial-whitening** exponent β.
//! >   — [PATENT US6,240,380 — FIGS.13–14, box 1318]
//!
//! The trace adds: "This is an encoder analysis detail; it determines
//! the matrix values but is not itself transmitted beyond the matrix."
//! **[DSP]** — so nothing in this module touches the bitstream; its
//! output feeds the §4 weighting matrix that
//! [`crate::matrix_coding`] carries as side information.
//!
//! ## Scope of this module
//!
//! * [`bark_from_hz`] — the Bark-scale frequency mapping, realised via
//!   the standard public psychoacoustic formula (the trace's `[DSP]`
//!   framing tier: the *patent* pins that the curve is computed on the
//!   Bark scale; the scale itself is textbook material).
//! * [`bin_frequency`] — MLT bin-centre frequency
//!   `(k + ½) · sr / 2M` (`[DSP]`: the standard bin spacing of an
//!   M-band cosine-modulated bank at sample rate `sr`).
//! * [`SpreadingSlopes`] + [`spread_masking`] — the patent's
//!   **simplified asymmetric triangular spreading**: each masker's
//!   level falls off linearly in dB per Bark, at the patent-pinned
//!   rates of 25 dB/Bark toward lower frequencies ("left") and
//!   10 dB/Bark toward higher frequencies ("right"), the combined
//!   curve being the per-position maximum. The asymmetry (shallower
//!   rightward slope → masking spreads farther upward) is the
//!   patent-disclosed shape.
//! * [`partial_whitening`] / [`partial_whitening_in_place`] — the
//!   optional exponent β applied to the weighting function,
//!   compressing its dynamic range between the β = 1 (unchanged) and
//!   β = 0 (flat) endpoints. The patent disclosed the exponent's
//!   existence and role; its shipping value is encoder tuning and a
//!   caller input, never fabricated.
//!
//! ## What is NOT in this module
//!
//! * **Absolute threshold / level calibration.** The patent excerpt
//!   pins the spreading slopes; playback-level calibration, threshold
//!   in quiet, and tonality analysis are not in the staged material.
//! * **Any bitstream field.** Encoder analysis only.

/// Bark-scale value for a frequency in Hz.
///
/// The patent fixes that the masking curve is computed **on the Bark
/// scale** (US6,240,380 box 1318); the mapping itself is the standard
/// public psychoacoustic formula (`[DSP]` tier)
/// `13·atan(0.00076·f) + 3.5·atan((f / 7500)²)`.
///
/// Monotone in `f`; `0.0` at DC.
pub fn bark_from_hz(hz: f64) -> f64 {
    13.0 * (0.00076 * hz).atan() + 3.5 * (hz / 7500.0).powi(2).atan()
}

/// Centre frequency in Hz of MLT bin `k` for an `m`-coefficient block
/// at `sample_rate` Hz: `(k + ½) · sample_rate / (2m)`.
///
/// `[DSP]`: the standard bin spacing of an M-band cosine-modulated
/// filter bank — the bank in [`crate::mlt`] — whose M bins tile
/// `0 .. sample_rate / 2` (the wiki's `high frequency = sample rate /
/// 2` ceiling).
///
/// # Panics
///
/// Panics if `m == 0`.
pub fn bin_frequency(k: usize, m: usize, sample_rate: u32) -> f64 {
    assert!(
        m > 0,
        "oxideav-wma::masking::bin_frequency: m must be positive"
    );
    (k as f64 + 0.5) * f64::from(sample_rate) / (2.0 * m as f64)
}

/// The asymmetric spreading-function slopes, in dB per Bark.
///
/// [`SpreadingSlopes::PATENT`] carries the §4 patent-pinned pair —
/// 25 dB/Bark toward lower frequencies ("left"), 10 dB/Bark toward
/// higher frequencies ("right") (US6,240,380 FIGS.13–14). Both values
/// are attenuation rates and must be positive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpreadingSlopes {
    /// Attenuation toward lower frequencies, dB per Bark.
    pub lower_db_per_bark: f64,
    /// Attenuation toward higher frequencies, dB per Bark.
    pub upper_db_per_bark: f64,
}

impl SpreadingSlopes {
    /// The patent-disclosed simplified pair: −25 dB/Bark left,
    /// +10 dB/Bark right (US6,240,380 — trace §4).
    pub const PATENT: SpreadingSlopes = SpreadingSlopes {
        lower_db_per_bark: 25.0,
        upper_db_per_bark: 10.0,
    };

    /// Contribution, in dB, of a masker of `level_db` at `masker_bark`
    /// to the position `at_bark`: the triangular fall-off at this
    /// pair's rates.
    pub fn contribution(&self, level_db: f64, masker_bark: f64, at_bark: f64) -> f64 {
        if at_bark <= masker_bark {
            level_db - self.lower_db_per_bark * (masker_bark - at_bark)
        } else {
            level_db - self.upper_db_per_bark * (at_bark - masker_bark)
        }
    }
}

/// Spread a set of masker levels across the Bark axis: the combined
/// masking curve at each position is the **maximum** of every
/// masker's triangular contribution (the simplified combination the
/// patent's FIGS.13–14 spreading describes).
///
/// `levels_db[j]` is the masker level at `barks[j]`; the output has
/// one combined masking level per input position. Empty input yields
/// an empty curve.
///
/// # Panics
///
/// Panics if `levels_db.len() != barks.len()`.
pub fn spread_masking(levels_db: &[f64], barks: &[f64], slopes: SpreadingSlopes) -> Vec<f64> {
    assert_eq!(
        levels_db.len(),
        barks.len(),
        "oxideav-wma::masking::spread_masking: levels and barks must have equal length",
    );
    (0..barks.len())
        .map(|i| {
            (0..barks.len())
                .map(|j| slopes.contribution(levels_db[j], barks[j], barks[i]))
                .fold(f64::NEG_INFINITY, f64::max)
        })
        .collect()
}

/// Apply the §4 optional **partial-whitening** exponent β to a
/// weighting function: `w[i] → w[i]^β`.
///
/// β = 1 leaves the weights unchanged; β = 0 flattens every positive
/// weight to `1.0` (full whitening — the quantization noise floor is
/// no longer shaped); β between the endpoints compresses the
/// weighting's dynamic range. Zero weights stay zero for β > 0 (a
/// silent band stays silent). The shipping β is encoder tuning —
/// caller-supplied, never fabricated.
///
/// # Panics
///
/// Panics if `beta` is negative or any weight is negative (weights
/// are §4 energies).
pub fn partial_whitening(weights: &[f64], beta: f64) -> Vec<f64> {
    let mut out = weights.to_vec();
    partial_whitening_in_place(&mut out, beta);
    out
}

/// In-place form of [`partial_whitening`].
///
/// # Panics
///
/// Panics if `beta` is negative or any weight is negative.
pub fn partial_whitening_in_place(weights: &mut [f64], beta: f64) {
    assert!(
        beta >= 0.0,
        "oxideav-wma::masking::partial_whitening: beta must be non-negative",
    );
    for w in weights.iter_mut() {
        assert!(
            *w >= 0.0,
            "oxideav-wma::masking::partial_whitening: weights are energies and must be non-negative",
        );
        if *w > 0.0 {
            *w = w.powf(beta);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- bark_from_hz ----------

    #[test]
    fn bark_is_zero_at_dc_and_monotone() {
        assert_eq!(bark_from_hz(0.0), 0.0);
        let mut prev = 0.0;
        for f in [50.0, 100.0, 500.0, 1000.0, 4000.0, 8000.0, 16000.0, 22050.0] {
            let b = bark_from_hz(f);
            assert!(b > prev, "f={f}: {b} <= {prev}");
            prev = b;
        }
    }

    #[test]
    fn bark_lands_in_the_conventional_ranges() {
        // The Bark scale places ~1 kHz near band 8-9 and the full
        // audible range within ~25 Bark — coarse sanity pins on the
        // public formula, not WMA claims.
        let b1k = bark_from_hz(1000.0);
        assert!((8.0..9.0).contains(&b1k), "bark(1k)={b1k}");
        let btop = bark_from_hz(22050.0);
        assert!((24.0..26.0).contains(&btop), "bark(22050)={btop}");
    }

    // ---------- bin_frequency ----------

    #[test]
    fn bin_frequencies_tile_zero_to_nyquist() {
        let m = 256usize;
        let sr = 44_100u32;
        // First bin sits half a spacing (sr / 2M / 2) above DC…
        let f0 = bin_frequency(0, m, sr);
        assert!((f0 - 44_100.0 / 1024.0).abs() < 1e-9);
        // …and the last half a spacing below Nyquist.
        let flast = bin_frequency(m - 1, m, sr);
        assert!(flast < 22_050.0);
        assert!(22_050.0 - flast < 44_100.0 / 512.0);
        // Uniform spacing sr / 2M.
        let spacing = bin_frequency(1, m, sr) - f0;
        assert!((spacing - 44_100.0 / 512.0).abs() < 1e-9);
    }

    #[test]
    #[should_panic(expected = "m must be positive")]
    fn bin_frequency_rejects_zero_m() {
        let _ = bin_frequency(0, 0, 44_100);
    }

    // ---------- spreading ----------

    #[test]
    fn patent_slopes_carry_the_disclosed_values() {
        assert_eq!(SpreadingSlopes::PATENT.lower_db_per_bark, 25.0);
        assert_eq!(SpreadingSlopes::PATENT.upper_db_per_bark, 10.0);
    }

    #[test]
    fn single_masker_forms_the_asymmetric_triangle() {
        // One masker at 10 Bark, 80 dB. One Bark to the left: −25 dB;
        // one Bark to the right: −10 dB (the patent's asymmetry —
        // masking spreads farther upward).
        let s = SpreadingSlopes::PATENT;
        assert_eq!(s.contribution(80.0, 10.0, 10.0), 80.0);
        assert_eq!(s.contribution(80.0, 10.0, 9.0), 55.0);
        assert_eq!(s.contribution(80.0, 10.0, 11.0), 70.0);
        assert!(
            s.contribution(80.0, 10.0, 11.0) > s.contribution(80.0, 10.0, 9.0),
            "upward spread must exceed downward at equal distance"
        );
    }

    #[test]
    fn spread_masking_takes_the_per_position_maximum() {
        // Two maskers; at each position the combined curve equals the
        // stronger contribution.
        let barks = [8.0, 10.0, 12.0];
        let levels = [60.0, 0.0, 66.0];
        let curve = spread_masking(&levels, &barks, SpreadingSlopes::PATENT);
        // Position 1 (10 Bark): from masker 0 (8 Bark, 60 dB, right
        // side): 60 − 10·2 = 40; from masker 2 (12 Bark, 66 dB, left
        // side): 66 − 25·2 = 16; from itself: 0. Max = 40.
        assert_eq!(curve[1], 40.0);
        // Each masker dominates its own position.
        assert_eq!(curve[0], 60.0);
        assert_eq!(curve[2], 66.0);
    }

    #[test]
    fn spread_masking_empty_and_mismatch() {
        assert!(spread_masking(&[], &[], SpreadingSlopes::PATENT).is_empty());
    }

    #[test]
    #[should_panic(expected = "equal length")]
    fn spread_masking_rejects_length_mismatch() {
        let _ = spread_masking(&[1.0], &[1.0, 2.0], SpreadingSlopes::PATENT);
    }

    // ---------- partial whitening ----------

    #[test]
    fn beta_one_is_identity_and_beta_zero_flattens() {
        let w = [4.0, 1.0, 0.25, 0.0];
        assert_eq!(partial_whitening(&w, 1.0), vec![4.0, 1.0, 0.25, 0.0]);
        // β = 0: every positive weight → 1; zero stays zero.
        assert_eq!(partial_whitening(&w, 0.0), vec![1.0, 1.0, 1.0, 0.0]);
    }

    #[test]
    fn intermediate_beta_compresses_dynamic_range() {
        let w = [16.0, 1.0];
        let half = partial_whitening(&w, 0.5);
        assert_eq!(half, vec![4.0, 1.0]);
        // Ratio 16:1 compressed to 4:1 — strictly narrower, still > 1.
        assert!(half[0] / half[1] < w[0] / w[1]);
        assert!(half[0] / half[1] > 1.0);
    }

    #[test]
    fn in_place_matches_fresh_vec() {
        let w = [9.0, 3.0, 0.5];
        let fresh = partial_whitening(&w, 0.7);
        let mut inplace = w;
        partial_whitening_in_place(&mut inplace, 0.7);
        assert_eq!(fresh, inplace.to_vec());
    }

    #[test]
    #[should_panic(expected = "beta must be non-negative")]
    fn whitening_rejects_negative_beta() {
        let _ = partial_whitening(&[1.0], -0.1);
    }

    #[test]
    #[should_panic(expected = "must be non-negative")]
    fn whitening_rejects_negative_weight() {
        let _ = partial_whitening(&[-1.0], 0.5);
    }

    // ---------- toward the §4 matrix ----------

    #[test]
    fn masking_curve_feeds_the_weighting_pipeline() {
        // End-to-end shape check: band-centre barks from bin
        // frequencies → spread a spectrum's band levels → whiten →
        // a strictly positive per-band weight vector of the right
        // length, ready to serve as the §4 Q[d] input.
        let m = 256usize;
        let sr = 44_100u32;
        let n_bands = 8usize;
        let barks: Vec<f64> = (0..n_bands)
            .map(|d| bark_from_hz(bin_frequency(d * (m / n_bands) + m / (2 * n_bands), m, sr)))
            .collect();
        let levels: Vec<f64> = (0..n_bands).map(|d| 60.0 - 3.0 * d as f64).collect();
        let curve = spread_masking(&levels, &barks, SpreadingSlopes::PATENT);
        assert_eq!(curve.len(), n_bands);
        // dB → linear energy weights, then partial whitening.
        let weights: Vec<f64> = curve.iter().map(|db| 10.0_f64.powf(db / 10.0)).collect();
        let whitened = partial_whitening(&weights, 0.5);
        assert_eq!(whitened.len(), n_bands);
        assert!(whitened.iter().all(|&w| w > 0.0));
    }
}

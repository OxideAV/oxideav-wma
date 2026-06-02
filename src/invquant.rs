//! WMA inverse-quantization (dequantization) step.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 lifts the
//! patent-disclosed quantizer arrangement that the WMA Standard
//! decoder reverses on a per-coefficient basis:
//!
//! > Each coefficient is quantized by the **product of its band's
//! > matrix weight `Q[c][d]` and a single overall step size** for the
//! > whole block, the step size being chosen to meet rate/quality
//! > targets.
//! >   — [PATENT US7,930,171 — overall step-size description]
//!
//! > The decoder applies inverse quantization and inverse weighting so
//! > that reconstructed coefficients carry quantization noise shaped
//! > by the weighting function.
//! >   — [PATENT US7,383,180 — inverse quantizer/weighter, FIG.6]
//! >   — [PATENT US6,240,380 — re-weighting at decoder]
//!
//! The patent draws the encoder forward step as
//!
//! ```text
//! q[k] = round( coeff[k] / (Q[d(k)] * step) )
//! ```
//!
//! and the decoder inverse as
//!
//! ```text
//! coeff_hat[k] = q[k] * Q[d(k)] * step
//! ```
//!
//! where `d(k)` is the band index of coefficient `k`, `Q[d]` is the
//! per-band quantization-matrix weight carried as side information
//! (see [`crate::qmatrix`]), and `step` is the per-block overall step
//! size.
//!
//! ## Scope of this module
//!
//! This module implements the patent's **decoder-side** inverse step
//! as integer-quantized-input × `f64`-weighted-output. The encoder
//! forward step is parametric (rate-control loop) and not implemented
//! here, but the multiplicative decomposition `Q[d] * step` is exposed
//! as a precomputable [`BandScale`] so a decoder can fold the two
//! patent factors into one per-band scalar across a whole block:
//!
//! * [`dequantize_sample`] — per-coefficient helper applying
//!   `q * weight * step` once.
//! * [`dequantize_in_place`] — whole-block helper applying the per-band
//!   product across a slice of quantized coefficients given a band map.
//! * [`BandScale`] — precomputed `Q[d] * step` table so the inner
//!   dequant loop multiplies once per coefficient instead of twice.
//!
//! ## What is NOT in this module
//!
//! * **The encoder forward quantization loop.** Step-size selection is
//!   a rate-distortion control problem (Thumpudi-180 / Thumpudi-291
//!   inner/outer loop). The patent describes it as encoder analysis;
//!   no bitstream-level rule pins the encoder's choice.
//! * **The Bark-scale masking model.** The matrix values `Q[d]` arrive
//!   here as opaque weights via [`crate::qmatrix`]; how the encoder
//!   *computed* them (Bark spreading, β whitening) is encoder analysis
//!   and lives outside the decoder pipeline.
//! * **Per-coefficient sign reconstruction.** The patent describes
//!   levels as magnitudes; the sign-bit placement is `[GAP]` per §6 of
//!   the trace. This module accepts already-signed `i32` quantized
//!   coefficients; sign decoding is the bitstream reader's job.
//! * **Clipping / overflow policy.** The patent draws the inverse step
//!   over arbitrary-precision real arithmetic. `f64` is used here for
//!   accuracy headroom; clipping back to a fixed-point sample range is
//!   a downstream concern.

/// Inverse-quantize a single coefficient.
///
/// Computes `q * weight * step` in `f64`, matching the patent's
/// decoder-side reverse of the encoder's
/// `q = round(coeff / (weight * step))` quantizer.
///
/// All three inputs are treated as opaque scalars: the patent text
/// fixes the multiplicative arrangement but does not fix the numeric
/// range of any factor. Callers feed `weight` from the per-band
/// quantization matrix (carried as side information; see
/// [`crate::qmatrix`]) and `step` from the per-block overall step size.
#[inline]
pub fn dequantize_sample(q: i32, weight: f64, step: f64) -> f64 {
    f64::from(q) * weight * step
}

/// Inverse-quantize a contiguous block of coefficients in place.
///
/// `bands[k]` is the band index of coefficient `k`; `weights[d]` is the
/// per-band matrix weight `Q[d]`; `step` is the block-wide overall step
/// size. After the call `out[k] = q[k] * weights[bands[k]] * step` for
/// every position.
///
/// Returns the number of coefficients dequantized.
///
/// # Panics
///
/// Panics if `q.len() != bands.len()` or if any `bands[k]` is out of
/// range for `weights`.
pub fn dequantize_in_place(q: &[i32], bands: &[u16], weights: &[f64], step: f64, out: &mut [f64]) {
    assert_eq!(
        q.len(),
        bands.len(),
        "oxideav-wma::invquant::dequantize_in_place: quantized slice and band map must have equal length",
    );
    assert_eq!(
        q.len(),
        out.len(),
        "oxideav-wma::invquant::dequantize_in_place: input and output slices must have equal length",
    );
    for k in 0..q.len() {
        let d = bands[k] as usize;
        assert!(
            d < weights.len(),
            "oxideav-wma::invquant::dequantize_in_place: band index {d} out of range for weights table of length {}",
            weights.len(),
        );
        out[k] = dequantize_sample(q[k], weights[d], step);
    }
}

/// Precomputed per-band scale factor `Q[d] * step`.
///
/// The patent's two-factor decoder step `q * Q[d] * step` is
/// associative; folding `Q[d] * step` once per band saves one
/// multiplication per coefficient when the block has many coefficients
/// in the same band. The carrier preserves the patent's per-band
/// arrangement (one scale per band index `d`).
#[derive(Debug, Clone, PartialEq)]
pub struct BandScale {
    /// `scale[d] = Q[d] * step` for band `d`. Indexed by the same `d`
    /// the bitstream's band map uses.
    pub scale: Vec<f64>,
}

impl BandScale {
    /// Build a [`BandScale`] from a slice of per-band weights and a
    /// single block-wide step size. Each `scale[d]` is exactly
    /// `weights[d] * step`.
    pub fn from_weights(weights: &[f64], step: f64) -> Self {
        Self {
            scale: weights.iter().map(|w| w * step).collect(),
        }
    }

    /// Number of bands the table covers.
    pub fn len(&self) -> usize {
        self.scale.len()
    }

    /// Whether the table is empty.
    pub fn is_empty(&self) -> bool {
        self.scale.is_empty()
    }

    /// Look up the folded scale for band `d`, or `None` if out of range.
    pub fn get(&self, d: usize) -> Option<f64> {
        self.scale.get(d).copied()
    }

    /// Apply the folded scale across a block of quantized coefficients
    /// in place. `out[k] = q[k] * self.scale[bands[k]]` for every `k`.
    ///
    /// # Panics
    ///
    /// Panics if `q.len() != bands.len() != out.len()` or if any band
    /// index is out of range for the table.
    pub fn apply(&self, q: &[i32], bands: &[u16], out: &mut [f64]) {
        assert_eq!(
            q.len(),
            bands.len(),
            "oxideav-wma::invquant::BandScale::apply: quantized slice and band map must have equal length",
        );
        assert_eq!(
            q.len(),
            out.len(),
            "oxideav-wma::invquant::BandScale::apply: input and output slices must have equal length",
        );
        for k in 0..q.len() {
            let d = bands[k] as usize;
            assert!(
                d < self.scale.len(),
                "oxideav-wma::invquant::BandScale::apply: band index {d} out of range for scale table of length {}",
                self.scale.len(),
            );
            out[k] = f64::from(q[k]) * self.scale[d];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- Per-sample identities ----------

    #[test]
    fn dequantize_sample_is_product_of_q_weight_and_step() {
        // Patent draws inverse as q * Q[d] * step. Sanity-check with
        // concrete values.
        assert_eq!(dequantize_sample(0, 1.0, 1.0), 0.0);
        assert_eq!(dequantize_sample(1, 1.0, 1.0), 1.0);
        assert_eq!(dequantize_sample(3, 2.0, 4.0), 24.0);
        assert_eq!(dequantize_sample(-2, 3.0, 5.0), -30.0);
    }

    #[test]
    fn dequantize_zero_quantized_is_zero_regardless_of_weight_or_step() {
        // q == 0 must reconstruct to 0 for any weight/step. This is
        // the patent's dead-zone behaviour: a quantizer bin centred on
        // zero reconstructs to zero.
        for w in [0.5, 1.0, 2.0, 100.0] {
            for s in [0.5, 1.0, 2.0, 100.0] {
                assert_eq!(dequantize_sample(0, w, s), 0.0, "w={w} s={s}");
            }
        }
    }

    #[test]
    fn dequantize_sample_is_linear_in_q() {
        // Scaling q scales the output by the same factor.
        let (w, s) = (1.5_f64, 2.5_f64);
        let base = dequantize_sample(1, w, s);
        assert_eq!(dequantize_sample(2, w, s), 2.0 * base);
        assert_eq!(dequantize_sample(5, w, s), 5.0 * base);
        assert_eq!(dequantize_sample(-3, w, s), -3.0 * base);
    }

    #[test]
    fn dequantize_sample_factors_commute() {
        // f64 multiplication is commutative; swapping weight and step
        // must give bit-identical results.
        for q in [1, 7, -3, 42] {
            let a = dequantize_sample(q, 2.5, 4.0);
            let b = dequantize_sample(q, 4.0, 2.5);
            assert_eq!(a, b, "q={q}");
        }
    }

    // ---------- Whole-block helper ----------

    #[test]
    fn dequantize_in_place_applies_per_band_weights() {
        // Two bands: 0 (weight=2.0) covers k=0..2, 1 (weight=3.0)
        // covers k=2..4. Step=1.0 to isolate the weight effect.
        let q = [1_i32, 2, 3, 4];
        let bands = [0_u16, 0, 1, 1];
        let weights = [2.0_f64, 3.0];
        let mut out = [0.0_f64; 4];
        dequantize_in_place(&q, &bands, &weights, 1.0, &mut out);
        // Position 0: 1 * 2.0 * 1.0 = 2.0
        // Position 1: 2 * 2.0 * 1.0 = 4.0
        // Position 2: 3 * 3.0 * 1.0 = 9.0
        // Position 3: 4 * 3.0 * 1.0 = 12.0
        assert_eq!(out, [2.0, 4.0, 9.0, 12.0]);
    }

    #[test]
    fn dequantize_in_place_threads_step_across_block() {
        // Same weight for both bands; step varies the whole-block
        // scale. Patent: step size is per-block.
        let q = [1_i32, 1, 1, 1];
        let bands = [0_u16, 0, 0, 0];
        let weights = [1.0_f64];
        let mut out = [0.0_f64; 4];
        dequantize_in_place(&q, &bands, &weights, 7.5, &mut out);
        assert_eq!(out, [7.5, 7.5, 7.5, 7.5]);
    }

    #[test]
    fn dequantize_in_place_empty_block_is_noop() {
        let q: [i32; 0] = [];
        let bands: [u16; 0] = [];
        let weights = [1.0_f64];
        let mut out: [f64; 0] = [];
        dequantize_in_place(&q, &bands, &weights, 1.0, &mut out);
        assert_eq!(out, [] as [f64; 0]);
    }

    #[test]
    #[should_panic(expected = "quantized slice and band map must have equal length")]
    fn dequantize_in_place_panics_on_band_map_mismatch() {
        let q = [1_i32, 2];
        let bands = [0_u16];
        let weights = [1.0_f64];
        let mut out = [0.0_f64; 2];
        dequantize_in_place(&q, &bands, &weights, 1.0, &mut out);
    }

    #[test]
    #[should_panic(expected = "input and output slices must have equal length")]
    fn dequantize_in_place_panics_on_output_mismatch() {
        let q = [1_i32, 2];
        let bands = [0_u16, 0];
        let weights = [1.0_f64];
        let mut out = [0.0_f64; 1];
        dequantize_in_place(&q, &bands, &weights, 1.0, &mut out);
    }

    #[test]
    #[should_panic(expected = "band index 5 out of range")]
    fn dequantize_in_place_panics_on_band_index_overflow() {
        let q = [1_i32];
        let bands = [5_u16];
        let weights = [1.0_f64];
        let mut out = [0.0_f64; 1];
        dequantize_in_place(&q, &bands, &weights, 1.0, &mut out);
    }

    // ---------- BandScale ----------

    #[test]
    fn band_scale_folds_weight_and_step_per_band() {
        let bs = BandScale::from_weights(&[2.0, 3.0, 5.0], 4.0);
        assert_eq!(bs.len(), 3);
        assert!(!bs.is_empty());
        assert_eq!(bs.get(0), Some(8.0));
        assert_eq!(bs.get(1), Some(12.0));
        assert_eq!(bs.get(2), Some(20.0));
        assert_eq!(bs.get(3), None);
    }

    #[test]
    fn band_scale_from_empty_weights_is_empty() {
        let bs = BandScale::from_weights(&[], 1.0);
        assert!(bs.is_empty());
        assert_eq!(bs.len(), 0);
        assert_eq!(bs.get(0), None);
    }

    #[test]
    fn band_scale_apply_matches_dequantize_in_place() {
        // The patent's two-factor product is associative under f64
        // multiplication for these exactly-representable inputs;
        // folding Q[d]*step once must give the same answer as
        // multiplying twice per coefficient.
        let q = [1_i32, -2, 3, -4, 5, -6];
        let bands = [0_u16, 0, 1, 1, 2, 2];
        let weights = [2.0_f64, 3.0, 4.0];
        let step = 0.5;

        let mut via_helper = [0.0_f64; 6];
        dequantize_in_place(&q, &bands, &weights, step, &mut via_helper);

        let bs = BandScale::from_weights(&weights, step);
        let mut via_bs = [0.0_f64; 6];
        bs.apply(&q, &bands, &mut via_bs);

        assert_eq!(via_helper, via_bs);
    }

    #[test]
    fn band_scale_apply_zero_block_is_zero() {
        let bs = BandScale::from_weights(&[2.5_f64, 3.5, 4.5], 10.0);
        let q = [0_i32; 8];
        let bands = [0_u16, 1, 2, 0, 1, 2, 0, 1];
        let mut out = [99.0_f64; 8];
        bs.apply(&q, &bands, &mut out);
        assert_eq!(out, [0.0; 8]);
    }

    #[test]
    #[should_panic(expected = "band index 7 out of range")]
    fn band_scale_apply_panics_on_band_index_overflow() {
        let bs = BandScale::from_weights(&[1.0_f64], 1.0);
        let q = [1_i32];
        let bands = [7_u16];
        let mut out = [0.0_f64; 1];
        bs.apply(&q, &bands, &mut out);
    }

    #[test]
    #[should_panic(expected = "quantized slice and band map must have equal length")]
    fn band_scale_apply_panics_on_band_map_mismatch() {
        let bs = BandScale::from_weights(&[1.0_f64], 1.0);
        let q = [1_i32, 2];
        let bands = [0_u16];
        let mut out = [0.0_f64; 2];
        bs.apply(&q, &bands, &mut out);
    }

    // ---------- Patent-named invariants ----------

    #[test]
    fn dequantize_round_trip_against_a_synthetic_encoder_quantizer() {
        // Build a synthetic forward quantizer that exactly inverts the
        // module's dequantize step. This documents the patent's stated
        // multiplicative arrangement: forward = round(c / (Q*step)),
        // inverse = q * Q * step. For inputs that already lie on the
        // quantization grid, the round trip is exact.
        let weights = [2.0_f64, 4.0];
        let step = 1.5;
        // Pick coefficients that fall exactly on the grid: c = k * Q[d] * step.
        let coeffs = [
            (0_i32, 0), // 0 in band 0
            (1, 0),     // 1*2*1.5 = 3.0
            (-3, 0),    // -3*2*1.5 = -9.0
            (5, 1),     // 5*4*1.5 = 30.0
            (-7, 1),    // -7*4*1.5 = -42.0
        ];
        for (q, d) in coeffs {
            let reconstructed = dequantize_sample(q, weights[d], step);
            // The forward step is q * weight * step / (weight * step) = q.
            // Compute (reconstructed / (weight * step)) and check we
            // recover q exactly.
            let recovered = reconstructed / (weights[d] * step);
            assert_eq!(recovered, f64::from(q), "q={q} d={d}");
        }
    }

    #[test]
    fn dequantize_in_place_handles_non_contiguous_band_layout() {
        // Patent §4 describes per-band quantization but does not
        // require any particular band ordering across the block. The
        // band map can interleave bands freely.
        let q = [1_i32, 2, 1, 2, 1, 2];
        let bands = [0_u16, 1, 0, 1, 0, 1];
        let weights = [10.0_f64, 100.0];
        let mut out = [0.0_f64; 6];
        dequantize_in_place(&q, &bands, &weights, 1.0, &mut out);
        assert_eq!(out, [10.0, 200.0, 10.0, 200.0, 10.0, 200.0]);
    }
}

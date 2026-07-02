//! WMA encoder-side forward quantization step.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 fixes the
//! quantizer arrangement whose decoder-side inverse [`crate::invquant`]
//! and [`crate::dequant`] already implement. The load-bearing
//! citations:
//!
//! > Each coefficient is quantized by the **product of its band's
//! > matrix weight `Q[c][d]` and a single overall step size** for the
//! > whole block, the step size being chosen to meet rate/quality
//! > targets.
//! >   — [PATENT US7,930,171 — overall step-size description]
//!
//! > The quantizer is an **adaptive, uniform, scalar quantizer that
//! > computes one quantization factor per tile.**
//! >   — [PATENT US7,383,180 — quantizer 560]
//!
//! [`crate::invquant`] records the two directions the patent draws:
//!
//! ```text
//! forward:  q[k]         = round( coeff[k] / (Q[d(k)] * step) )
//! inverse:  coeff_hat[k] = q[k] * Q[d(k)] * step
//! ```
//!
//! This module implements the **forward** direction so the crate's
//! encoder-side chain mirrors the assembled decoder chain
//! stage-for-stage.
//!
//! ## Scope of this module
//!
//! * [`quantize_sample`] — per-coefficient helper applying
//!   `round(coeff / (weight * step))` once.
//! * [`quantize_in_place`] — whole-block helper over a band map,
//!   mirroring [`crate::invquant::dequantize_in_place`].
//! * [`QuantStage`] — the per-block forward stage mirroring
//!   [`crate::dequant::DequantStage`]: the per-band divisor
//!   `Q[d] * step` is folded once at construction (reusing
//!   [`BandScale`]) and the band map is materialised once, so each
//!   [`QuantStage::block`] call is one division-and-round per
//!   coefficient.
//!
//! ## What is NOT in this module
//!
//! * **Step-size selection.** The patent describes the step as chosen
//!   by a rate-control loop (US7,343,291 quantization-loop iteration);
//!   that is encoder tuning with no bitstream-pinned rule, so `step`
//!   is a caller-supplied [`OverallStepSize`], exactly as the decoder
//!   side takes it.
//! * **The rounding-rule fine print.** The patents say "round" without
//!   pinning the tie-breaking rule. This module uses round-half-away-
//!   from-zero (`f64::round`), which is symmetric about the patent's
//!   zero-centred dead zone; the tie rule only matters for
//!   coefficients landing exactly on a bin boundary, and no staged
//!   document pins WMA's choice, so this realization detail is
//!   documented rather than claimed.
//! * **Degenerate divisors.** A zero (or non-finite) folded divisor
//!   `Q[d] * step` makes the patent's quotient undefined; this module
//!   quantizes such a coefficient to `0` (a silent band) instead of
//!   emitting `NaN`-derived garbage, and saturates quotients beyond
//!   the `i32` range at the type bounds. Both are defensive
//!   boundaries, not WMA facts.

use crate::block::BlockSize;
use crate::invquant::BandScale;
use crate::qband::QuantBandLayout;
use crate::step_size::OverallStepSize;

/// Forward-quantize a single coefficient.
///
/// Computes `round(coeff / (weight * step))`, the patent's encoder-side
/// forward of the decoder's `q * weight * step` inverse
/// (US7,930,171 / US7,383,180 quantizer 560; see [`crate::invquant`]
/// for the paired inverse).
///
/// A non-finite quotient (zero or non-finite divisor, non-finite
/// coefficient) quantizes to `0`; a finite quotient beyond the `i32`
/// range saturates at the type bounds. Both boundaries are defensive
/// realization details — see the module docs.
#[inline]
pub fn quantize_sample(coeff: f64, weight: f64, step: f64) -> i32 {
    quantize_with_divisor(coeff, weight * step)
}

/// [`quantize_sample`] with the two patent factors already folded into
/// one divisor `Q[d] * step` (the [`BandScale`] arrangement).
#[inline]
fn quantize_with_divisor(coeff: f64, divisor: f64) -> i32 {
    let quotient = coeff / divisor;
    if !quotient.is_finite() {
        return 0;
    }
    let rounded = quotient.round();
    if rounded >= f64::from(i32::MAX) {
        i32::MAX
    } else if rounded <= f64::from(i32::MIN) {
        i32::MIN
    } else {
        rounded as i32
    }
}

/// Forward-quantize a contiguous block of coefficients.
///
/// `bands[k]` is the band index of coefficient `k`; `weights[d]` is the
/// per-band matrix weight `Q[d]`; `step` is the block-wide overall step
/// size. After the call `out[k] = round(coeffs[k] / (weights[bands[k]] *
/// step))` for every position, mirroring
/// [`crate::invquant::dequantize_in_place`].
///
/// # Panics
///
/// Panics if `coeffs.len() != bands.len()`, if `coeffs.len() !=
/// out.len()`, or if any `bands[k]` is out of range for `weights` —
/// the same contracts the inverse helper enforces.
pub fn quantize_in_place(
    coeffs: &[f64],
    bands: &[u16],
    weights: &[f64],
    step: f64,
    out: &mut [i32],
) {
    assert_eq!(
        coeffs.len(),
        bands.len(),
        "oxideav-wma::quant::quantize_in_place: coefficient slice and band map must have equal length",
    );
    assert_eq!(
        coeffs.len(),
        out.len(),
        "oxideav-wma::quant::quantize_in_place: input and output slices must have equal length",
    );
    for k in 0..coeffs.len() {
        let d = bands[k] as usize;
        assert!(
            d < weights.len(),
            "oxideav-wma::quant::quantize_in_place: band index {d} out of range for weights table of length {}",
            weights.len(),
        );
        out[k] = quantize_sample(coeffs[k], weights[d], step);
    }
}

/// Per-block forward quantization stage — the encoder-side mirror of
/// [`crate::dequant::DequantStage`], per §4 of the patent trace
/// (US7,930,171 overall step-size description; US7,383,180 quantizer
/// 560).
///
/// One [`QuantStage::block`] call consumes `M` real-valued MLT
/// coefficients and emits `M` quantized integer coefficients
/// `q[k] = round(coeff[k] / (Q[d(k)] * step))`, exactly the input the
/// entropy stage codes and [`crate::dequant::DequantStage::block`]
/// reverses.
///
/// The per-band divisors `Q[d] * step` are folded once at construction
/// (the same [`BandScale`] fold the decoder side uses) and the
/// per-coefficient band map `d(k)` is materialised once.
#[derive(Debug, Clone, PartialEq)]
pub struct QuantStage {
    block_size: BlockSize,
    /// Per-coefficient weight-index map `d(k)`; length `M`.
    band_map: Vec<u16>,
    /// Folded per-band divisor `Q[d] * step`, keyed by band index `d`.
    scale: BandScale,
}

impl QuantStage {
    /// Construct the forward stage for a block from its
    /// quantization-band layout, the per-band weights `Q[d]`, and the
    /// per-block overall step size — the same triple
    /// [`crate::dequant::DequantStage::new`] takes, validated by the
    /// same rules so an encoder/decoder pair built from one parameter
    /// set is guaranteed to agree.
    ///
    /// # Errors
    ///
    /// * [`InvalidQuant::BlockSizeMismatch`] if the layout's total
    ///   coefficient count differs from `block_size`'s sample count.
    /// * [`InvalidQuant::WeightIndexOutOfRange`] if any band's weight
    ///   index is `>= weights.len()`.
    pub fn new(
        block_size: BlockSize,
        layout: &QuantBandLayout,
        weights: &[f64],
        step: OverallStepSize,
    ) -> Result<Self, InvalidQuant> {
        let m = block_size.samples() as usize;
        if layout.total_coeffs() != m {
            return Err(InvalidQuant::BlockSizeMismatch {
                block_size: m,
                layout_total: layout.total_coeffs(),
            });
        }
        for band in layout.bands() {
            let d = band.weight_index() as usize;
            if d >= weights.len() {
                return Err(InvalidQuant::WeightIndexOutOfRange {
                    weight_index: band.weight_index(),
                    weights_len: weights.len(),
                });
            }
        }
        Ok(Self {
            block_size,
            band_map: layout.band_map(),
            scale: BandScale::from_weights(weights, step.value()),
        })
    }

    /// Block size `M` for this stage.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.block_size
    }

    /// `M`, the per-call input length (real coefficient count) and the
    /// per-call output length (quantized coefficient count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.band_map.len()
    }

    /// Read-only view of the per-coefficient band map `d(k)` (length
    /// `M`).
    #[inline]
    pub fn band_map(&self) -> &[u16] {
        &self.band_map
    }

    /// Read-only view of the folded per-band divisor `Q[d] * step`.
    #[inline]
    pub fn scale(&self) -> &BandScale {
        &self.scale
    }

    /// Forward-quantize one block: consume `M` real-valued MLT
    /// coefficients, emit `M` quantized integers
    /// `q[k] = round(coeff[k] / (Q[d(k)] * step))`.
    ///
    /// Returns [`InvalidQuant::CoeffLenMismatch`] if `coeffs.len() != M`.
    pub fn block(&self, coeffs: &[f64]) -> Result<Vec<i32>, InvalidQuant> {
        let m = self.block_len();
        if coeffs.len() != m {
            return Err(InvalidQuant::CoeffLenMismatch {
                expected: m,
                got: coeffs.len(),
            });
        }
        let mut out = vec![0i32; m];
        for k in 0..m {
            let d = self.band_map[k] as usize;
            // `new` rejected any out-of-range weight index, so the scale
            // lookup cannot fail.
            out[k] = quantize_with_divisor(coeffs[k], self.scale.scale[d]);
        }
        Ok(out)
    }
}

/// Rejection reasons for [`QuantStage`] construction and use. Mirrors
/// [`crate::dequant::InvalidDequant`] variant-for-variant so the
/// forward and inverse stages fail the same way on the same bad
/// parameter set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidQuant {
    /// The quantization-band layout's total coefficient count does not
    /// match the stage's block size.
    BlockSizeMismatch {
        /// Coefficient count the block size implies (`M`).
        block_size: usize,
        /// Total coefficient count the layout declares.
        layout_total: usize,
    },
    /// A band's weight index has no corresponding entry in the
    /// per-band weights slice.
    WeightIndexOutOfRange {
        /// The offending weight index `d`.
        weight_index: u16,
        /// Length of the weights slice it indexed past.
        weights_len: usize,
    },
    /// [`QuantStage::block`] was given a coefficient slice whose
    /// length is not `M`.
    CoeffLenMismatch {
        /// Coefficient count the stage requires (`M`).
        expected: usize,
        /// Coefficient count actually offered.
        got: usize,
    },
}

impl core::fmt::Display for InvalidQuant {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidQuant::BlockSizeMismatch {
                block_size,
                layout_total,
            } => write!(
                f,
                "oxideav-wma::quant: layout covers {layout_total} coefficients but the block size implies {block_size}",
            ),
            InvalidQuant::WeightIndexOutOfRange {
                weight_index,
                weights_len,
            } => write!(
                f,
                "oxideav-wma::quant: band weight index {weight_index} is out of range for weights table of length {weights_len}",
            ),
            InvalidQuant::CoeffLenMismatch { expected, got } => write!(
                f,
                "oxideav-wma::quant: block expected {expected} coefficients, got {got}",
            ),
        }
    }
}

impl std::error::Error for InvalidQuant {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dequant::DequantStage;
    use crate::invquant::dequantize_sample;
    use crate::qband::QuantBand;

    fn layout_one_band(m: usize) -> QuantBandLayout {
        let band = QuantBand::new(0, m as u16, 0).unwrap();
        QuantBandLayout::new(vec![band], m).unwrap()
    }

    fn step(v: f64) -> OverallStepSize {
        OverallStepSize::new(v).unwrap()
    }

    // ---------- Per-sample forward ----------

    #[test]
    fn quantize_sample_rounds_quotient() {
        // Unit divisor: quantization is plain rounding.
        assert_eq!(quantize_sample(0.0, 1.0, 1.0), 0);
        assert_eq!(quantize_sample(0.4, 1.0, 1.0), 0);
        assert_eq!(quantize_sample(0.6, 1.0, 1.0), 1);
        assert_eq!(quantize_sample(-0.6, 1.0, 1.0), -1);
        assert_eq!(quantize_sample(24.0, 2.0, 4.0), 3);
        assert_eq!(quantize_sample(-30.0, 3.0, 5.0), -2);
    }

    #[test]
    fn quantize_sample_dead_zone_is_symmetric_about_zero() {
        // Coefficients within half a divisor of zero quantize to 0 —
        // the zero-centred dead zone the inverse's q == 0 → 0 pairs
        // with.
        let (w, s) = (2.0, 3.0); // divisor 6.0
        for c in [-2.9_f64, -1.0, 0.0, 1.0, 2.9] {
            assert_eq!(quantize_sample(c, w, s), 0, "c={c}");
        }
        assert_eq!(quantize_sample(3.1, w, s), 1);
        assert_eq!(quantize_sample(-3.1, w, s), -1);
    }

    #[test]
    fn quantize_sample_inverts_dequantize_on_grid_points() {
        // For coefficients that already lie on the quantization grid
        // the forward step recovers the integer exactly — the patent's
        // paired forward/inverse arrangement.
        let weights = [2.0_f64, 4.0];
        let s = 1.5;
        for (q, d) in [(0_i32, 0), (1, 0), (-3, 0), (5, 1), (-7, 1), (1000, 1)] {
            let coeff = dequantize_sample(q, weights[d], s);
            assert_eq!(quantize_sample(coeff, weights[d], s), q, "q={q} d={d}");
        }
    }

    #[test]
    fn quantize_error_is_bounded_by_half_a_divisor() {
        // The defining property of the patent's uniform scalar
        // quantizer: |coeff - q * divisor| <= divisor / 2.
        let (w, s) = (0.7_f64, 1.3);
        let divisor = w * s;
        for i in 0..200 {
            let coeff = (i as f64) * 0.317 - 31.0;
            let q = quantize_sample(coeff, w, s);
            let err = (coeff - dequantize_sample(q, w, s)).abs();
            assert!(
                err <= divisor / 2.0 + 1e-12,
                "coeff={coeff} q={q} err={err}"
            );
        }
    }

    #[test]
    fn quantize_sample_zero_divisor_yields_silent_zero() {
        // Defensive boundary: zero weight (or step folded to zero)
        // quantizes everything to 0 rather than emitting NaN-derived
        // garbage.
        assert_eq!(quantize_sample(5.0, 0.0, 1.0), 0);
        assert_eq!(quantize_sample(-5.0, 1.0, 0.0), 0);
        assert_eq!(quantize_sample(0.0, 0.0, 0.0), 0);
    }

    #[test]
    fn quantize_sample_non_finite_coefficient_yields_zero() {
        assert_eq!(quantize_sample(f64::NAN, 1.0, 1.0), 0);
        assert_eq!(quantize_sample(f64::INFINITY, 1.0, 1.0), 0);
        assert_eq!(quantize_sample(f64::NEG_INFINITY, 1.0, 1.0), 0);
    }

    #[test]
    fn quantize_sample_saturates_at_i32_bounds() {
        // A huge finite quotient saturates rather than wrapping.
        assert_eq!(quantize_sample(1e300, 1.0, 1.0), i32::MAX);
        assert_eq!(quantize_sample(-1e300, 1.0, 1.0), i32::MIN);
    }

    // ---------- Whole-block helper ----------

    #[test]
    fn quantize_in_place_applies_per_band_divisors() {
        // Mirrors invquant's per-band test with the roles reversed.
        let coeffs = [2.0_f64, 4.0, 9.0, 12.0];
        let bands = [0_u16, 0, 1, 1];
        let weights = [2.0_f64, 3.0];
        let mut out = [0_i32; 4];
        quantize_in_place(&coeffs, &bands, &weights, 1.0, &mut out);
        assert_eq!(out, [1, 2, 3, 4]);
    }

    #[test]
    fn quantize_in_place_empty_block_is_noop() {
        let coeffs: [f64; 0] = [];
        let bands: [u16; 0] = [];
        let weights = [1.0_f64];
        let mut out: [i32; 0] = [];
        quantize_in_place(&coeffs, &bands, &weights, 1.0, &mut out);
        assert_eq!(out, [] as [i32; 0]);
    }

    #[test]
    #[should_panic(expected = "coefficient slice and band map must have equal length")]
    fn quantize_in_place_panics_on_band_map_mismatch() {
        let coeffs = [1.0_f64, 2.0];
        let bands = [0_u16];
        let weights = [1.0_f64];
        let mut out = [0_i32; 2];
        quantize_in_place(&coeffs, &bands, &weights, 1.0, &mut out);
    }

    #[test]
    #[should_panic(expected = "input and output slices must have equal length")]
    fn quantize_in_place_panics_on_output_mismatch() {
        let coeffs = [1.0_f64, 2.0];
        let bands = [0_u16, 0];
        let weights = [1.0_f64];
        let mut out = [0_i32; 1];
        quantize_in_place(&coeffs, &bands, &weights, 1.0, &mut out);
    }

    #[test]
    #[should_panic(expected = "band index 5 out of range")]
    fn quantize_in_place_panics_on_band_index_overflow() {
        let coeffs = [1.0_f64];
        let bands = [5_u16];
        let weights = [1.0_f64];
        let mut out = [0_i32; 1];
        quantize_in_place(&coeffs, &bands, &weights, 1.0, &mut out);
    }

    // ---------- QuantStage construction ----------

    #[test]
    fn stage_new_accepts_matching_layout() {
        let m = 256;
        let stage =
            QuantStage::new(BlockSize::S256, &layout_one_band(m), &[1.0], step(1.0)).unwrap();
        assert_eq!(stage.block_size(), BlockSize::S256);
        assert_eq!(stage.block_len(), m);
        assert_eq!(stage.band_map().len(), m);
        assert_eq!(stage.scale().len(), 1);
    }

    #[test]
    fn stage_new_rejects_layout_total_mismatch() {
        let err =
            QuantStage::new(BlockSize::S512, &layout_one_band(256), &[1.0], step(1.0)).unwrap_err();
        assert_eq!(
            err,
            InvalidQuant::BlockSizeMismatch {
                block_size: 512,
                layout_total: 256,
            }
        );
    }

    #[test]
    fn stage_new_rejects_weight_index_out_of_range() {
        // A band pointing at weight slot 3 with only 1 weight supplied.
        let band = QuantBand::new(0, 256, 3).unwrap();
        let layout = QuantBandLayout::new(vec![band], 256).unwrap();
        let err = QuantStage::new(BlockSize::S256, &layout, &[1.0], step(1.0)).unwrap_err();
        assert_eq!(
            err,
            InvalidQuant::WeightIndexOutOfRange {
                weight_index: 3,
                weights_len: 1,
            }
        );
    }

    #[test]
    fn stage_rejects_wrong_block_len() {
        let stage =
            QuantStage::new(BlockSize::S256, &layout_one_band(256), &[1.0], step(1.0)).unwrap();
        let err = stage.block(&vec![0.0; 255]).unwrap_err();
        assert_eq!(
            err,
            InvalidQuant::CoeffLenMismatch {
                expected: 256,
                got: 255,
            }
        );
    }

    // ---------- QuantStage arithmetic ----------

    #[test]
    fn stage_block_matches_free_helper() {
        // The folded-divisor stage must agree with the two-factor free
        // helper for exactly-representable inputs.
        let m = 256;
        let bands: Vec<QuantBand> = vec![
            QuantBand::new(0, 128, 0).unwrap(),
            QuantBand::new(128, 128, 1).unwrap(),
        ];
        let layout = QuantBandLayout::new(bands, m).unwrap();
        let weights = [2.0_f64, 4.0];
        let s = 0.5;

        let coeffs: Vec<f64> = (0..m).map(|k| (k as f64) * 0.37 - 40.0).collect();

        let stage = QuantStage::new(BlockSize::S256, &layout, &weights, step(s)).unwrap();
        let via_stage = stage.block(&coeffs).unwrap();

        let mut via_helper = vec![0_i32; m];
        quantize_in_place(&coeffs, &layout.band_map(), &weights, s, &mut via_helper);

        assert_eq!(via_stage, via_helper);
    }

    #[test]
    fn stage_round_trips_with_dequant_stage_on_grid() {
        // Encoder QuantStage and decoder DequantStage built from the
        // SAME (layout, weights, step) triple: integers → dequantize →
        // quantize recovers the integers exactly.
        let m = 256;
        let layout = layout_one_band(m);
        let weights = [3.0_f64];
        let s = step(0.25);

        let forward = QuantStage::new(BlockSize::S256, &layout, &weights, s).unwrap();
        let inverse = DequantStage::new(BlockSize::S256, &layout, &weights, s).unwrap();

        let q_in: Vec<i32> = (0..m).map(|k| (k as i32) - 128).collect();
        let coeffs = inverse.block(&q_in).unwrap();
        let q_out = forward.block(&coeffs).unwrap();
        assert_eq!(q_out, q_in);
    }

    #[test]
    fn stage_quantization_error_bounded_across_all_block_sizes() {
        // |coeff - dequant(quant(coeff))| <= divisor/2 for every block
        // size in the patent set.
        for bs in BlockSize::ALL {
            let m = bs.samples() as usize;
            let layout = layout_one_band(m);
            let weights = [1.5_f64];
            let s = step(0.5);
            let divisor = 1.5 * 0.5;

            let forward = QuantStage::new(bs, &layout, &weights, s).unwrap();
            let inverse = DequantStage::new(bs, &layout, &weights, s).unwrap();

            let coeffs: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.013).sin() * 10.0).collect();
            let q = forward.block(&coeffs).unwrap();
            let coeff_hat = inverse.block(&q).unwrap();
            for k in 0..m {
                let err = (coeffs[k] - coeff_hat[k]).abs();
                assert!(err <= divisor / 2.0 + 1e-12, "bs={bs:?} k={k} err={err}");
            }
        }
    }

    #[test]
    fn stage_zero_block_quantizes_to_zero() {
        let stage =
            QuantStage::new(BlockSize::S256, &layout_one_band(256), &[1.0], step(1.0)).unwrap();
        let q = stage.block(&vec![0.0; 256]).unwrap();
        assert!(q.iter().all(|&v| v == 0));
    }

    // ---------- Error Display / trait ----------

    #[test]
    fn error_display_names_each_variant() {
        let a = InvalidQuant::BlockSizeMismatch {
            block_size: 512,
            layout_total: 256,
        };
        assert!(format!("{a}").contains("512"));
        assert!(format!("{a}").contains("256"));

        let b = InvalidQuant::WeightIndexOutOfRange {
            weight_index: 3,
            weights_len: 1,
        };
        assert!(format!("{b}").contains("weight index 3"));

        let c = InvalidQuant::CoeffLenMismatch {
            expected: 256,
            got: 255,
        };
        assert!(format!("{c}").contains("255"));

        let dyn_err: &dyn std::error::Error = &c;
        assert!(dyn_err.source().is_none());
    }
}

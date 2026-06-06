//! WMA per-block overall step-size carrier.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 lifts the
//! patent-disclosed arrangement that pairs the per-band quantization
//! matrix with a single block-wide step:
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
//! > The step size is varied across a rate-control loop (inner/outer
//! > loop style).
//! >   — [PATENT US7,343,291 — quantization-loop iteration]
//!
//! ## What this module models
//!
//! The patent disclosure names three load-bearing properties for the
//! step:
//!
//! 1. **One step per block** — the patent says "for the whole block"
//!    (US7,930,171) and "one quantization factor per tile"
//!    (US7,383,180). A tile is the patent's per-block coding unit, so
//!    one step covers all coefficients in the block.
//! 2. **The step is positive** — both the forward step
//!    `q[k] = round(coeff[k] / (Q[d(k)] * step))` (US7,930,171) and the
//!    decoder inverse `coeff_hat[k] = q[k] * Q[d(k)] * step`
//!    (US7,383,180 inverse quantizer-weighter) are ill-defined at
//!    `step == 0` (division by zero on the forward side; total
//!    sign-information loss on the inverse). Negative steps would
//!    sign-flip the reconstructed coefficients vs. the encoder's
//!    intent.
//! 3. **The step is uniform-scalar across the block** — every
//!    coefficient is divided / multiplied by `Q[d] * step`. There is
//!    no per-coefficient or per-frequency-bin override above the
//!    per-band matrix `Q[d]`; the step itself does not vary inside the
//!    block.
//!
//! The typed primitive in this module captures all three: a non-zero
//! finite positive `f64` for the step, paired with the patent's
//! per-block scope.
//!
//! ## Composition with [`crate::invquant`]
//!
//! [`crate::invquant::BandScale::from_weights`] takes a raw `f64` step
//! today; that surface is intentionally unchanged because the patent's
//! decoder step `q * Q[d] * step` is parametric on the step alone — it
//! does not need to know whether the step was extracted from a typed
//! carrier or supplied directly. This module adds a typed *encoder
//! and decoder-front-end* surface that names the patent's step-size
//! contract, with a single accessor [`OverallStepSize::value`] that
//! produces the same `f64` [`crate::invquant::BandScale::from_weights`]
//! consumes, and a convenience [`PerBlockStep::fold_with_weights`] that
//! threads the per-block typed step through to the decoder's per-band
//! folded scale.
//!
//! ## What is NOT in this module
//!
//! * **The encoder's rate-control loop** that picks the step
//!   (Thumpudi-180 / Thumpudi-291 inner/outer loop). The patent
//!   describes it as encoder analysis whose only bitstream effect is
//!   the carried step value; this module accepts a step as opaque
//!   input.
//! * **The bitstream carriage of the step** — exactly how the step is
//!   quantized, log-coded, and packed into the per-block side
//!   information is **[GAP]** in the patents-only trace
//!   (`wma-bitstream-from-patents.md` §4 names the *fact* of the
//!   carriage, not the field layout).
//! * **Forward (encoder) quantization.** The patent describes
//!   `q = round(coeff / (Q[d] * step))`; only the inverse step lives
//!   in [`crate::invquant`]. The typed `OverallStepSize` is symmetric:
//!   a forward stage can [`OverallStepSize::value`] it the same way.

use core::fmt;

use crate::block::BlockSize;
use crate::invquant::BandScale;

/// The per-block overall step size from the patent's "single overall
/// step size for the whole block" (US7,930,171) / "one quantization
/// factor per tile" (US7,383,180).
///
/// Carries a single non-zero finite positive `f64`. The patent fixes
/// the role (it multiplies the per-band matrix weight `Q[d]` to give
/// the per-coefficient quantizer factor) but not the numeric range; the
/// type enforces only the positivity and finiteness constraints the
/// quantizer arrangement requires for the round-trip to be well
/// defined.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OverallStepSize(f64);

impl OverallStepSize {
    /// Build an [`OverallStepSize`] after enforcing the patent-implied
    /// positivity, finiteness, and non-NaN constraints on the input.
    ///
    /// Returns [`InvalidStepSize`] for any input the patent's
    /// `q = round(coeff / (Q * step))` / `coeff_hat = q * Q * step`
    /// arrangement cannot accommodate without sign loss or
    /// undefined arithmetic.
    pub fn new(step: f64) -> Result<Self, InvalidStepSize> {
        if step.is_nan() {
            return Err(InvalidStepSize::NotANumber);
        }
        if !step.is_finite() {
            return Err(InvalidStepSize::NotFinite { given: step });
        }
        if step <= 0.0 {
            return Err(InvalidStepSize::NotPositive { given: step });
        }
        Ok(Self(step))
    }

    /// The underlying `f64`. Hand this to
    /// [`crate::invquant::BandScale::from_weights`] (or
    /// [`crate::invquant::dequantize_in_place`]) as the patent's
    /// `step` factor.
    pub fn value(self) -> f64 {
        self.0
    }

    /// Multiply this step by a per-band matrix weight `Q[d]`.
    ///
    /// The patent's per-coefficient factor is `Q[d] * step`; this
    /// helper realises that product for a single band so callers can
    /// build their own per-band tables without first materialising a
    /// full [`BandScale`]. The operation is `f64`-equivalent to the
    /// `weight * self.value()` expansion.
    pub fn apply_to_weight(self, weight: f64) -> f64 {
        weight * self.0
    }

    /// Build the patent's folded `Q[d] * step` table for a slice of
    /// per-band weights, returning a [`BandScale`] sized to the slice.
    ///
    /// Equivalent to [`BandScale::from_weights`]`(weights, self.value())`;
    /// exposed on the typed carrier so call sites that already hold a
    /// typed step do not need to re-extract the `f64`.
    pub fn band_scale_from_weights(self, weights: &[f64]) -> BandScale {
        BandScale::from_weights(weights, self.0)
    }
}

impl fmt::Display for OverallStepSize {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "oxideav-wma::OverallStepSize({})", self.0)
    }
}

/// Reasons [`OverallStepSize::new`] rejects an input.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InvalidStepSize {
    /// The supplied value was `NaN`. The patent's
    /// `q = round(coeff / (Q * step))` is undefined for `NaN`.
    NotANumber,
    /// The supplied value was infinite. An infinite step would
    /// reconstruct every coefficient to `±∞`.
    NotFinite { given: f64 },
    /// The supplied value was zero or negative. A zero step would make
    /// the forward quantizer undefined (division by zero); a negative
    /// step would sign-flip every reconstructed coefficient relative
    /// to the encoder's intent. The patent's step-size loop produces
    /// strictly positive values.
    NotPositive { given: f64 },
}

impl fmt::Display for InvalidStepSize {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            InvalidStepSize::NotANumber => {
                f.write_str("oxideav-wma: overall step size must not be NaN")
            }
            InvalidStepSize::NotFinite { given } => write!(
                f,
                "oxideav-wma: overall step size must be finite (got {given})",
            ),
            InvalidStepSize::NotPositive { given } => write!(
                f,
                "oxideav-wma: overall step size must be positive (got {given})",
            ),
        }
    }
}

impl std::error::Error for InvalidStepSize {}

/// Per-block carrier pairing a transform [`BlockSize`] with the
/// [`OverallStepSize`] for that block.
///
/// Models the patent's "one step per tile" arrangement
/// (US7,383,180 quantizer 560) where the step size is bound to a
/// specific block's coding step. The block size lives alongside the
/// step because the patent's rate-control loop selects them jointly
/// (block-size switching changes the coefficient count, which feeds
/// the rate target that drives step selection — Thumpudi-180 / -291).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PerBlockStep {
    /// The block this step governs.
    pub block_size: BlockSize,
    /// The step size for every coefficient in this block.
    pub step: OverallStepSize,
}

impl PerBlockStep {
    /// Bundle a block size and an already-validated step into a typed
    /// per-block carrier.
    pub fn new(block_size: BlockSize, step: OverallStepSize) -> Self {
        Self { block_size, step }
    }

    /// Block size for this per-block step.
    pub fn block_size(self) -> BlockSize {
        self.block_size
    }

    /// Step size for this per-block step.
    pub fn step(self) -> OverallStepSize {
        self.step
    }

    /// Per-block coefficient count, identical to
    /// [`BlockSize::samples`] for this block.
    pub fn coefficient_count(self) -> u16 {
        self.block_size.samples()
    }

    /// Fold the patent's per-coefficient factor `Q[d] * step` across a
    /// slice of per-band weights, materialising a [`BandScale`] keyed
    /// by band index. Equivalent to
    /// [`OverallStepSize::band_scale_from_weights`] with the step
    /// extracted from this carrier; the block size is *not* used
    /// inside the fold (it is patent-disclosed metadata identifying
    /// which block the step belongs to).
    pub fn fold_with_weights(self, weights: &[f64]) -> BandScale {
        self.step.band_scale_from_weights(weights)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- OverallStepSize::new accept/reject ----------

    #[test]
    fn new_accepts_a_typical_positive_step() {
        // A plausible mid-range step value; the patent does not pin
        // any specific value, only the positivity and finiteness of
        // the choice.
        let s = OverallStepSize::new(1.5).unwrap();
        assert_eq!(s.value(), 1.5);
    }

    #[test]
    fn new_accepts_smallest_subnormal_positive_step() {
        // f64::MIN_POSITIVE is the smallest positive normal value;
        // anything strictly above zero is patent-acceptable.
        let s = OverallStepSize::new(f64::MIN_POSITIVE).unwrap();
        assert_eq!(s.value(), f64::MIN_POSITIVE);
    }

    #[test]
    fn new_rejects_zero() {
        // Forward quantizer is undefined at step=0 (division by zero).
        let err = OverallStepSize::new(0.0).unwrap_err();
        assert_eq!(err, InvalidStepSize::NotPositive { given: 0.0 });
    }

    #[test]
    fn new_rejects_negative_zero() {
        // Negative zero would still be ill-defined and is rejected by
        // the strict <= 0 guard.
        let err = OverallStepSize::new(-0.0).unwrap_err();
        assert_eq!(err, InvalidStepSize::NotPositive { given: -0.0 });
    }

    #[test]
    fn new_rejects_negative_step() {
        // A negative step would sign-flip every reconstructed
        // coefficient relative to the encoder's intent.
        let err = OverallStepSize::new(-1.5).unwrap_err();
        assert_eq!(err, InvalidStepSize::NotPositive { given: -1.5 });
    }

    #[test]
    fn new_rejects_positive_infinity() {
        let err = OverallStepSize::new(f64::INFINITY).unwrap_err();
        assert_eq!(
            err,
            InvalidStepSize::NotFinite {
                given: f64::INFINITY
            }
        );
    }

    #[test]
    fn new_rejects_negative_infinity() {
        // Caught by the finiteness guard *before* the positivity
        // guard so the reported error reflects the more specific
        // condition.
        let err = OverallStepSize::new(f64::NEG_INFINITY).unwrap_err();
        assert_eq!(
            err,
            InvalidStepSize::NotFinite {
                given: f64::NEG_INFINITY
            }
        );
    }

    #[test]
    fn new_rejects_nan() {
        let err = OverallStepSize::new(f64::NAN).unwrap_err();
        assert_eq!(err, InvalidStepSize::NotANumber);
    }

    // ---------- OverallStepSize accessors ----------

    #[test]
    fn value_returns_constructor_input() {
        let s = OverallStepSize::new(3.25).unwrap();
        assert_eq!(s.value(), 3.25);
    }

    #[test]
    fn apply_to_weight_multiplies_step_by_weight() {
        let s = OverallStepSize::new(2.0).unwrap();
        assert_eq!(s.apply_to_weight(3.0), 6.0);
        assert_eq!(s.apply_to_weight(1.0), 2.0);
        assert_eq!(s.apply_to_weight(0.0), 0.0);
    }

    #[test]
    fn apply_to_weight_is_commutative_with_value() {
        // weight * step.value() must equal step.apply_to_weight(weight)
        // for any weight on exactly-representable inputs.
        let s = OverallStepSize::new(4.0).unwrap();
        for w in [1.0_f64, 2.5, 7.0, 0.125] {
            assert_eq!(s.apply_to_weight(w), w * s.value(), "w={w}");
        }
    }

    #[test]
    fn band_scale_from_weights_matches_invquant_constructor() {
        // The typed surface must produce the same BandScale the
        // free function would.
        let weights = [2.0_f64, 3.0, 5.0];
        let s = OverallStepSize::new(4.0).unwrap();
        let via_typed = s.band_scale_from_weights(&weights);
        let via_direct = BandScale::from_weights(&weights, 4.0);
        assert_eq!(via_typed, via_direct);
    }

    #[test]
    fn band_scale_from_weights_on_empty_weights_is_empty() {
        let s = OverallStepSize::new(1.0).unwrap();
        let bs = s.band_scale_from_weights(&[]);
        assert!(bs.is_empty());
    }

    // ---------- OverallStepSize Display ----------

    #[test]
    fn display_quotes_value() {
        let s = OverallStepSize::new(1.5).unwrap();
        assert_eq!(format!("{s}"), "oxideav-wma::OverallStepSize(1.5)");
    }

    // ---------- InvalidStepSize Display + Error ----------

    #[test]
    fn invalid_display_messages_name_each_variant() {
        let nan = InvalidStepSize::NotANumber;
        assert_eq!(
            format!("{nan}"),
            "oxideav-wma: overall step size must not be NaN"
        );
        let inf = InvalidStepSize::NotFinite {
            given: f64::INFINITY,
        };
        assert_eq!(
            format!("{inf}"),
            "oxideav-wma: overall step size must be finite (got inf)"
        );
        let neg = InvalidStepSize::NotPositive { given: -2.0 };
        assert_eq!(
            format!("{neg}"),
            "oxideav-wma: overall step size must be positive (got -2)"
        );
    }

    #[test]
    fn invalid_step_size_implements_std_error() {
        fn assert_error<E: std::error::Error>(_: &E) {}
        assert_error(&InvalidStepSize::NotANumber);
    }

    // ---------- PerBlockStep ----------

    #[test]
    fn per_block_step_carries_block_size_and_step() {
        let s = OverallStepSize::new(2.5).unwrap();
        let pbs = PerBlockStep::new(BlockSize::S1024, s);
        assert_eq!(pbs.block_size(), BlockSize::S1024);
        assert_eq!(pbs.step(), s);
    }

    #[test]
    fn per_block_step_coefficient_count_matches_block_size() {
        for bs in BlockSize::ALL {
            let s = OverallStepSize::new(1.0).unwrap();
            let pbs = PerBlockStep::new(bs, s);
            assert_eq!(pbs.coefficient_count(), bs.samples(), "bs={bs:?}");
        }
    }

    #[test]
    fn per_block_step_fold_with_weights_matches_typed_step() {
        // Per-block carrier's fold helper must agree with the typed
        // step's band_scale_from_weights.
        let s = OverallStepSize::new(3.0).unwrap();
        let pbs = PerBlockStep::new(BlockSize::S2048, s);
        let weights = [1.0_f64, 2.0, 4.0, 8.0];
        let via_pbs = pbs.fold_with_weights(&weights);
        let via_step = s.band_scale_from_weights(&weights);
        assert_eq!(via_pbs, via_step);
    }

    #[test]
    fn per_block_step_fold_with_weights_threads_step_into_band_scale() {
        // Folded scale must be element-wise weights[d] * step.
        let s = OverallStepSize::new(4.0).unwrap();
        let pbs = PerBlockStep::new(BlockSize::S512, s);
        let weights = [2.0_f64, 3.0, 5.0];
        let bs = pbs.fold_with_weights(&weights);
        assert_eq!(bs.len(), 3);
        assert_eq!(bs.get(0), Some(8.0));
        assert_eq!(bs.get(1), Some(12.0));
        assert_eq!(bs.get(2), Some(20.0));
    }

    #[test]
    fn per_block_step_copy_semantics() {
        // Copy makes the per-block carrier ergonomic to thread through
        // a decoder loop; both copies are equal to the original.
        let s = OverallStepSize::new(1.25).unwrap();
        let pbs = PerBlockStep::new(BlockSize::S256, s);
        let copy = pbs;
        assert_eq!(pbs, copy);
        assert_eq!(pbs.block_size(), copy.block_size());
        assert_eq!(pbs.step(), copy.step());
    }

    // ---------- Cross-module composition: invquant ----------

    #[test]
    fn fold_with_weights_drives_band_scale_apply() {
        use crate::invquant::dequantize_in_place;

        // End-to-end: build per-block step, fold with per-band
        // weights, hand the resulting BandScale to invquant's
        // dequant loop. The patent's two-factor arrangement
        // `q * Q[d] * step` should match the typed-carrier path.
        let s = OverallStepSize::new(0.5).unwrap();
        let pbs = PerBlockStep::new(BlockSize::S256, s);
        let weights = [2.0_f64, 4.0];
        let bs = pbs.fold_with_weights(&weights);

        let q = [1_i32, -2, 3, -4];
        let bands = [0_u16, 0, 1, 1];

        let mut via_typed = [0.0_f64; 4];
        bs.apply(&q, &bands, &mut via_typed);

        let mut via_direct = [0.0_f64; 4];
        dequantize_in_place(&q, &bands, &weights, s.value(), &mut via_direct);

        assert_eq!(via_typed, via_direct);
    }

    #[test]
    fn typed_step_produces_same_factor_as_apply_to_weight() {
        // The patent's per-coefficient factor `Q[d] * step` must come
        // out identically whether the caller multiplies directly,
        // calls apply_to_weight, or extracts the typed step's value.
        let s = OverallStepSize::new(7.5).unwrap();
        for q_weight in [1.0_f64, 2.5, 10.0, 0.25] {
            let via_apply = s.apply_to_weight(q_weight);
            let via_value = q_weight * s.value();
            assert_eq!(via_apply, via_value, "q_weight={q_weight}");
        }
    }

    // ---------- Patent invariants ----------

    #[test]
    fn typed_step_drives_dequantize_factor_for_every_block_size() {
        // The patent's "step is per-block" / "per-tile" arrangement
        // means the same typed step must produce the same per-band
        // factor regardless of block-size context.
        let s = OverallStepSize::new(2.0).unwrap();
        let weight = 3.0_f64;
        let expected = weight * 2.0; // 6.0

        for bs in BlockSize::ALL {
            let pbs = PerBlockStep::new(bs, s);
            assert_eq!(pbs.step().apply_to_weight(weight), expected, "bs={bs:?}");
        }
    }

    #[test]
    fn distinct_steps_produce_distinct_carriers() {
        // PerBlockStep equality must depend on both block and step.
        let s1 = OverallStepSize::new(1.0).unwrap();
        let s2 = OverallStepSize::new(2.0).unwrap();
        let a = PerBlockStep::new(BlockSize::S1024, s1);
        let b = PerBlockStep::new(BlockSize::S1024, s2);
        let c = PerBlockStep::new(BlockSize::S2048, s1);
        assert_ne!(a, b);
        assert_ne!(a, c);
        assert_eq!(a, PerBlockStep::new(BlockSize::S1024, s1));
    }

    #[test]
    fn fold_with_weights_for_empty_band_table_is_empty() {
        let s = OverallStepSize::new(1.0).unwrap();
        let pbs = PerBlockStep::new(BlockSize::S1024, s);
        let bs = pbs.fold_with_weights(&[]);
        assert!(bs.is_empty());
    }
}

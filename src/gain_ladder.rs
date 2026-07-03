//! The staged dequantization gain ladder wired into the §4 chain.
//!
//! ## Source
//!
//! [`crate::wire_tables::DEQUANT_GAIN_LUT`] — 113 monotone
//! fixed-point multipliers, log-spaced with ratio `10^(1/16)`
//! (**1.25 dB of amplitude per step**), extracted as data from the
//! vendor WMA Standard decoder module
//! (`docs/audio/wma/tables/dequant-gain-lut.{csv,meta}`). The staged
//! role: "indexed by a decoded exponent/scale value to give a
//! fixed-point linear dequantization multiplier" — i.e. the ladder is
//! the vendor realization of the §4 per-band weight `Q[d]` lookup the
//! patent trace describes abstractly (exponent/critical-band coded
//! envelope; the same pass confirms no LSP path exists).
//!
//! ## What this module provides
//!
//! * [`gain`] / [`linear_gain`] — the raw ladder value for a decoded
//!   exponent/scale index (integer and `f64` views).
//! * [`gain_ratio`] — the relative amplitude between two ladder
//!   indices (the scale-free quantity; the ladder's absolute
//!   fixed-point normalization is a vendor implementation detail).
//! * [`band_weights`] — map a per-band exponent-index vector to the
//!   `Q[d]` weight vector the §4 [`crate::dequant::DequantStage`] /
//!   [`crate::quant::QuantStage`] constructor pair consumes,
//!   normalized to the largest band so the overall level rides on the
//!   [`crate::step_size::OverallStepSize`] exactly as the patent
//!   trace's `q * Q[d] * step` factorization expects.
//!
//! ## What stays `[GAP]`
//!
//! How the per-band exponent indices are **carried in the bitstream**
//! (the ~121-entry scale / ~37-entry gain Huffman tables were located
//! but not emitted, and the sign/packing layout is unstaged), so the
//! index vector is an input here, never fabricated.

use crate::wire_tables::DEQUANT_GAIN_LUT;

/// Number of steps in the staged ladder.
pub const STEP_COUNT: usize = DEQUANT_GAIN_LUT.len();

/// Amplitude spacing of adjacent ladder steps: `20·log10(10^(1/16))`
/// = 1.25 dB per step (the staged `.meta` closed form).
pub const DB_PER_STEP: f64 = 1.25;

/// Failure modes for the gain-ladder helpers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainLadderError {
    /// An exponent/scale index was outside the ladder (`>= 113`).
    IndexOutOfRange {
        /// The rejected index.
        index: usize,
    },
    /// [`band_weights`] was offered an empty index vector; a block
    /// has at least one band.
    EmptyBands,
}

impl core::fmt::Display for GainLadderError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GainLadderError::IndexOutOfRange { index } => write!(
                f,
                "oxideav-wma::gain_ladder: exponent index {index} outside the {STEP_COUNT}-step ladder",
            ),
            GainLadderError::EmptyBands => {
                f.write_str("oxideav-wma::gain_ladder: empty band index vector")
            }
        }
    }
}

impl std::error::Error for GainLadderError {}

/// The raw fixed-point ladder value at `index`, or `None` out of
/// range.
pub fn gain(index: usize) -> Option<u32> {
    DEQUANT_GAIN_LUT.get(index).copied()
}

/// The ladder value at `index` as `f64`, or `None` out of range.
pub fn linear_gain(index: usize) -> Option<f64> {
    gain(index).map(f64::from)
}

/// Relative amplitude between two ladder indices:
/// `gain(a) / gain(b)`. The scale-free quantity — a step of `+16`
/// indices is exactly one decade of the staged closed form (20 dB).
///
/// # Errors
///
/// [`GainLadderError::IndexOutOfRange`] for either index.
pub fn gain_ratio(a: usize, b: usize) -> Result<f64, GainLadderError> {
    let ga = linear_gain(a).ok_or(GainLadderError::IndexOutOfRange { index: a })?;
    let gb = linear_gain(b).ok_or(GainLadderError::IndexOutOfRange { index: b })?;
    Ok(ga / gb)
}

/// Map per-band exponent/scale indices to the §4 weight vector
/// `Q[d]`, normalized so the largest band has weight `1.0` (the
/// block's absolute level is the [`crate::step_size`] job in the
/// `q * Q[d] * step` factorization; the ladder's fixed-point
/// normalization is a vendor detail that cancels in the ratio).
///
/// The output feeds [`crate::dequant::DequantStage::new`] /
/// [`crate::quant::QuantStage::new`] as their `weights` argument,
/// with band `d`'s weight at slot `d`.
///
/// # Errors
///
/// * [`GainLadderError::EmptyBands`] for an empty index vector.
/// * [`GainLadderError::IndexOutOfRange`] for an index `>= 113`.
pub fn band_weights(indices: &[u8]) -> Result<Vec<f64>, GainLadderError> {
    if indices.is_empty() {
        return Err(GainLadderError::EmptyBands);
    }
    let mut raw = Vec::with_capacity(indices.len());
    for &idx in indices {
        raw.push(
            linear_gain(usize::from(idx)).ok_or(GainLadderError::IndexOutOfRange {
                index: usize::from(idx),
            })?,
        );
    }
    let max = raw.iter().copied().fold(f64::MIN, f64::max);
    Ok(raw.into_iter().map(|g| g / max).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::BlockSize;
    use crate::dequant::DequantStage;
    use crate::exponent_bands::exponent_band_layout;
    use crate::step_size::OverallStepSize;

    #[test]
    fn raw_lookups_match_the_staged_table() {
        assert_eq!(STEP_COUNT, 113);
        assert_eq!(gain(0), Some(1));
        assert_eq!(gain(112), Some(5_758_375));
        assert_eq!(gain(113), None);
        assert_eq!(linear_gain(112), Some(5_758_375.0));
    }

    #[test]
    fn sixteen_steps_are_one_decade() {
        // The staged closed form: ratio 10^(1/16) per step, so +16
        // steps = ×10 amplitude (+20 dB). Exact on the fitted tail.
        for a in (46..96).step_by(7) {
            let r = gain_ratio(a + 16, a).unwrap();
            assert!((r - 10.0).abs() < 0.01, "index {a}: decade ratio {r}");
        }
        // And one step is DB_PER_STEP within integer rounding.
        for a in 60..80 {
            let db = 20.0 * gain_ratio(a + 1, a).unwrap().log10();
            assert!((db - DB_PER_STEP).abs() < 0.01, "index {a}: {db} dB");
        }
    }

    #[test]
    fn out_of_range_and_empty_are_typed_errors() {
        assert_eq!(
            gain_ratio(113, 0),
            Err(GainLadderError::IndexOutOfRange { index: 113 })
        );
        assert_eq!(band_weights(&[]), Err(GainLadderError::EmptyBands));
        assert_eq!(
            band_weights(&[5, 200]),
            Err(GainLadderError::IndexOutOfRange { index: 200 })
        );
        assert!(
            format!("{}", GainLadderError::IndexOutOfRange { index: 113 })
                .contains("113-step ladder")
        );
    }

    #[test]
    fn band_weights_normalize_to_the_loudest_band() {
        let w = band_weights(&[96, 80, 64]).unwrap();
        assert_eq!(w[0], 1.0);
        // 16 ladder steps apart: each next band is one decade down.
        assert!((w[1] - 0.1).abs() < 1e-3, "{w:?}");
        assert!((w[2] - 0.01).abs() < 1e-4, "{w:?}");
    }

    #[test]
    fn ladder_weights_drive_the_real_partition_dequant_chain() {
        // The §4 chain built entirely from staged data: the
        // 44.1 kHz / 2048 critical-band partition (25 bands) with a
        // ladder-derived Q[d] per band.
        let block = BlockSize::S2048;
        let layout = exponent_band_layout(44_100, block).unwrap();
        let indices: Vec<u8> = (0..layout.band_count())
            .map(|d| u8::try_from(100 - 2 * d).unwrap())
            .collect();
        let weights = band_weights(&indices).unwrap();
        let stage = DequantStage::new(block, &layout, &weights, OverallStepSize::new(0.5).unwrap())
            .unwrap();
        let q = vec![1i32; 2048];
        let out = stage.block(&q).unwrap();
        // Band 0 is the loudest (index 100): q * 1.0 * step.
        assert!((out[0] - 0.5).abs() < 1e-12);
        // Each later band sits 2 ladder steps (2.5 dB) lower; the last
        // band is 48 steps = 60 dB below the first.
        let last = *out.last().unwrap();
        let db = 20.0 * (out[0] / last).log10();
        assert!((db - 60.0).abs() < 0.1, "span {db} dB");
        // Monotone non-increasing across the block (indices descend).
        for w in out.windows(2) {
            assert!(w[1] <= w[0] + 1e-12);
        }
    }
}

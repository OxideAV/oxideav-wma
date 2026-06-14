//! WMA decoder-side inverse-quantize + inverse-weighting stage.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 and the §8 decoder
//! pipeline diagram fix the structure of the decoder step that turns the
//! entropy-decoded integer spectral coefficients back into weighted
//! real-valued coefficients. The load-bearing citations:
//!
//! > Each coefficient is quantized by the **product of its band's matrix
//! > weight `Q[c][d]` and a single overall step size** for the whole
//! > block, the step size being chosen to meet rate/quality targets.
//! >   — [PATENT US7,930,171 — overall step-size description]
//!
//! > The decoder applies inverse quantization and inverse weighting so
//! > that reconstructed coefficients carry quantization noise shaped by
//! > the weighting function.
//! >   — [PATENT US7,383,180 — inverse quantizer/weighter, FIG.6]
//! >   — [PATENT US6,240,380 — re-weighting at decoder]
//!
//! and the same FIG.6 step as drawn in §8 of the trace:
//!
//! > ```text
//! >  → entropy decode (run-level → coefficients; matrix deltas)
//! >  → inverse quantize + inverse weighting        (US7,383,180; US6,240,380)
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (decoder
//! >     pipeline, Thumpudi-180 FIG.6)
//!
//! The patent draws the per-coefficient decoder inverse as
//!
//! ```text
//! coeff_hat[k] = q[k] * Q[d(k)] * step
//! ```
//!
//! where `d(k)` is the band index of coefficient `k`, `Q[d]` is the
//! per-band quantization-matrix weight carried as side information, and
//! `step` is the per-block overall step size.
//!
//! ## Scope of this module
//!
//! This module is the **assembler** that wires the three §4 primitives
//! already landed into the single decoder inverse-quantize/inverse-weight
//! stage the patent's FIG.6 draws, exactly as [`crate::synthesis`]
//! assembled the §3 reconstruction chain:
//!
//! * [`crate::qband::QuantBandLayout`] (Round 8) — supplies the
//!   per-coefficient band map `d(k)` (which band each coefficient belongs
//!   to) via [`crate::qband::QuantBandLayout::band_map`].
//! * the per-band weights `Q[d]` (Round 4 [`crate::qmatrix`] carriage;
//!   Round 15 [`crate::excitation`] derivation) — the quantization-matrix
//!   row for the block, one weight per band index.
//! * [`crate::step_size::OverallStepSize`] (Round 10) — the single
//!   per-block overall step size that multiplies every band weight.
//!
//! [`DequantStage::new`] folds `Q[d] * step` once per band into the
//! Round 5 [`crate::invquant::BandScale`] table and materialises the band
//! map once, so each [`DequantStage::block`] call is a single
//! multiply-per-coefficient pass — the patent's two-factor product folded
//! to one, with the band lookup precomputed. The output `M`-coefficient
//! vector is exactly the input [`crate::synthesis::Synthesis::block`]
//! consumes, so the two assemblers chain into the FIG.6 decoder tail
//! *inverse quantize/weight → inverse MLT → window → overlap-add*.
//!
//! ## What is NOT in this module
//!
//! * **The entropy decode that produces the integer coefficients.** The
//!   `M`-coefficient `i32` input is the output of the run-level entropy
//!   stage (§6, [`crate::runlevel`] / [`crate::codebook`] /
//!   [`crate::escape`]); this module starts where §8's FIG.6 chain places
//!   the inverse quantizer, immediately after entropy decode.
//! * **How the weights `Q[d]` were computed.** They arrive as opaque
//!   per-band scalars — whether differentially decoded from the bitstream
//!   side information ([`crate::qmatrix`]) or derived from an excitation
//!   pattern ([`crate::excitation`]); the Bark-scale masking model that
//!   shaped them is encoder analysis (§4) and out of scope.
//! * **The overall step-size selection.** The step is a rate-control
//!   choice (Thumpudi-180 / -291 inner/outer loop); the decoder receives
//!   the chosen value. This module takes an already-validated
//!   [`crate::step_size::OverallStepSize`].
//! * **Per-coefficient sign reconstruction.** The patent describes levels
//!   as magnitudes; sign-bit placement is `[GAP]` per §6. This stage
//!   accepts already-signed `i32` coefficients, exactly as
//!   [`crate::invquant`] does.
//! * **Noise-substituted / truncated band fills.** §7's noise generator
//!   (decoder module 240) and the band-truncation cutoff act on which
//!   bands are coded at all; whichever coefficients reach this stage are
//!   dequantized uniformly. The band policy carrier is
//!   [`crate::bands::BandPolicy`]; the noise-pattern construction stays
//!   `[GAP]` per §7.

use crate::block::BlockSize;
use crate::invquant::BandScale;
use crate::qband::QuantBandLayout;
use crate::step_size::OverallStepSize;

/// Stateless decoder-side inverse-quantize + inverse-weighting stage for
/// one [`BlockSize`] `M`, per §4 of the patent trace (US7,930,171 overall
/// step-size description; US7,383,180 inverse quantizer/weighter FIG.6;
/// US6,240,380 re-weighting at decoder).
///
/// One [`DequantStage::block`] call consumes `M` entropy-decoded integer
/// spectral coefficients and emits `M` dequantized real-valued
/// coefficients `coeff_hat[k] = q[k] * Q[d(k)] * step`, ready for the
/// inverse MLT in [`crate::synthesis::Synthesis::block`].
///
/// The per-band weights and the overall step are folded once at
/// construction into a [`BandScale`] (`scale[d] = Q[d] * step`), and the
/// per-coefficient band map `d(k)` is materialised once, so each
/// `block` call is one multiplication per coefficient with the band
/// lookup precomputed.
#[derive(Debug, Clone, PartialEq)]
pub struct DequantStage {
    block_size: BlockSize,
    /// Per-coefficient weight-index map `d(k)`; length `M`.
    band_map: Vec<u16>,
    /// Folded per-band scale `Q[d] * step`, keyed by band index `d`.
    scale: BandScale,
}

impl DequantStage {
    /// Construct the dequantization stage for a block from its
    /// quantization-band layout, the per-band weights `Q[d]`, and the
    /// per-block overall step size.
    ///
    /// `layout` partitions the block into bands and supplies the
    /// per-coefficient band map; `weights[d]` is the matrix weight for
    /// band index `d`; `step` is the block-wide step. The fold
    /// `scale[d] = weights[d] * step` is computed once here.
    ///
    /// # Errors
    ///
    /// * [`InvalidDequant::BlockSizeMismatch`] if the layout's total
    ///   coefficient count differs from `block_size`'s sample count — the
    ///   band map length must equal `M` for the per-block contract to
    ///   hold.
    /// * [`InvalidDequant::WeightIndexOutOfRange`] if any band's weight
    ///   index is `>= weights.len()` — the fold would have no scale to
    ///   look up for that band.
    pub fn new(
        block_size: BlockSize,
        layout: &QuantBandLayout,
        weights: &[f64],
        step: OverallStepSize,
    ) -> Result<Self, InvalidDequant> {
        let m = block_size.samples() as usize;
        if layout.total_coeffs() != m {
            return Err(InvalidDequant::BlockSizeMismatch {
                block_size: m,
                layout_total: layout.total_coeffs(),
            });
        }
        // Every band must address a weight slot, or the per-band fold has
        // no `Q[d]` to multiply the step by.
        for band in layout.bands() {
            let d = band.weight_index() as usize;
            if d >= weights.len() {
                return Err(InvalidDequant::WeightIndexOutOfRange {
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

    /// `M`, the per-call input length (integer coefficient count) and the
    /// per-call output length (dequantized coefficient count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.band_map.len()
    }

    /// Read-only view of the per-coefficient band map `d(k)` (length
    /// `M`), materialised once at construction.
    #[inline]
    pub fn band_map(&self) -> &[u16] {
        &self.band_map
    }

    /// Read-only view of the folded per-band scale `Q[d] * step`.
    #[inline]
    pub fn scale(&self) -> &BandScale {
        &self.scale
    }

    /// Dequantize one block: consume `M` entropy-decoded integer
    /// coefficients, emit `M` dequantized real-valued coefficients
    /// `coeff_hat[k] = q[k] * Q[d(k)] * step`.
    ///
    /// Delegates the per-coefficient arithmetic to the folded
    /// [`BandScale::apply`] (one multiplication per coefficient against
    /// the precomputed `Q[d] * step`), preserving the patent's per-band
    /// arrangement.
    ///
    /// Returns [`InvalidDequant::CoeffLenMismatch`] if `q.len() != M`.
    pub fn block(&self, q: &[i32]) -> Result<Vec<f64>, InvalidDequant> {
        let m = self.block_len();
        if q.len() != m {
            return Err(InvalidDequant::CoeffLenMismatch {
                expected: m,
                got: q.len(),
            });
        }
        let mut out = vec![0.0_f64; m];
        // Band map and scale share the band-index domain by construction
        // (`new` rejected any out-of-range weight index), and lengths all
        // equal `M`, so `apply` cannot panic here.
        self.scale.apply(q, &self.band_map, &mut out);
        Ok(out)
    }
}

/// Rejection reasons for [`DequantStage`] construction and use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidDequant {
    /// The quantization-band layout's total coefficient count does not
    /// match the stage's block size.
    BlockSizeMismatch {
        /// Coefficient count the block size implies (`M`).
        block_size: usize,
        /// Total coefficient count the layout declares.
        layout_total: usize,
    },
    /// A band's weight index has no corresponding entry in the per-band
    /// weights slice.
    WeightIndexOutOfRange {
        /// The offending weight index `d`.
        weight_index: u16,
        /// Length of the weights slice it indexed past.
        weights_len: usize,
    },
    /// [`DequantStage::block`] was given a coefficient slice whose length
    /// is not `M`.
    CoeffLenMismatch {
        /// Coefficient count the stage requires (`M`).
        expected: usize,
        /// Coefficient count actually offered.
        got: usize,
    },
}

impl core::fmt::Display for InvalidDequant {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidDequant::BlockSizeMismatch {
                block_size,
                layout_total,
            } => write!(
                f,
                "dequant stage block size {block_size} coefficients does not match quantization-band layout total {layout_total}",
            ),
            InvalidDequant::WeightIndexOutOfRange {
                weight_index,
                weights_len,
            } => write!(
                f,
                "dequant stage band weight index {weight_index} out of range for weights table of length {weights_len}",
            ),
            InvalidDequant::CoeffLenMismatch { expected, got } => write!(
                f,
                "dequant stage block expected {expected} coefficients, got {got}",
            ),
        }
    }
}

impl std::error::Error for InvalidDequant {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::invquant::dequantize_sample;
    use crate::qband::QuantBand;
    use crate::step_size::OverallStepSize;

    fn step(v: f64) -> OverallStepSize {
        OverallStepSize::new(v).unwrap()
    }

    /// Single-band layout over a whole block of the given size.
    fn single_band_layout(block: BlockSize, weight_index: u16) -> QuantBandLayout {
        let m = block.samples();
        QuantBandLayout::for_block(vec![QuantBand::new(0, m, weight_index).unwrap()], block)
            .unwrap()
    }

    // ---------- Construction accept paths ----------

    #[test]
    fn new_accepts_single_band_layout_for_every_block_size() {
        for bs in BlockSize::ALL {
            let layout = single_band_layout(bs, 0);
            let stage = DequantStage::new(bs, &layout, &[2.0], step(1.0)).unwrap();
            assert_eq!(stage.block_size(), bs);
            assert_eq!(stage.block_len(), bs.samples() as usize);
            assert_eq!(stage.band_map().len(), bs.samples() as usize);
        }
    }

    #[test]
    fn new_folds_weight_and_step_into_scale() {
        let bs = BlockSize::S256;
        // Two bands: first half weight index 0, second half weight index 1.
        let half = bs.samples() / 2;
        let layout = QuantBandLayout::for_block(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, half, 1).unwrap(),
            ],
            bs,
        )
        .unwrap();
        let stage = DequantStage::new(bs, &layout, &[2.0, 3.0], step(5.0)).unwrap();
        // scale[d] == weights[d] * step
        assert_eq!(stage.scale().get(0), Some(10.0));
        assert_eq!(stage.scale().get(1), Some(15.0));
        // band map: first half -> 0, second half -> 1.
        assert_eq!(stage.band_map()[0], 0);
        assert_eq!(stage.band_map()[(half as usize) - 1], 0);
        assert_eq!(stage.band_map()[half as usize], 1);
    }

    #[test]
    fn new_accepts_shared_weight_index_across_bands() {
        // Patent allows multiple bands to reference one weight index.
        let bs = BlockSize::S256;
        let half = bs.samples() / 2;
        let layout = QuantBandLayout::for_block(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, half, 0).unwrap(),
            ],
            bs,
        )
        .unwrap();
        let stage = DequantStage::new(bs, &layout, &[4.0], step(0.5)).unwrap();
        assert_eq!(stage.scale().len(), 1);
        assert!(stage.band_map().iter().all(|&d| d == 0));
    }

    // ---------- Construction reject paths ----------

    #[test]
    fn new_rejects_block_size_layout_mismatch() {
        // Layout total for S256 fed to an S512 stage.
        let layout = single_band_layout(BlockSize::S256, 0);
        let err = DequantStage::new(BlockSize::S512, &layout, &[1.0], step(1.0)).unwrap_err();
        assert_eq!(
            err,
            InvalidDequant::BlockSizeMismatch {
                block_size: BlockSize::S512.samples() as usize,
                layout_total: BlockSize::S256.samples() as usize,
            }
        );
    }

    #[test]
    fn new_rejects_weight_index_out_of_range() {
        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 2);
        // weights has only indices 0 and 1; band references index 2.
        let err = DequantStage::new(bs, &layout, &[1.0, 1.0], step(1.0)).unwrap_err();
        assert_eq!(
            err,
            InvalidDequant::WeightIndexOutOfRange {
                weight_index: 2,
                weights_len: 2,
            }
        );
    }

    #[test]
    fn new_rejects_empty_weights_for_nonempty_layout() {
        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 0);
        let err = DequantStage::new(bs, &layout, &[], step(1.0)).unwrap_err();
        assert_eq!(
            err,
            InvalidDequant::WeightIndexOutOfRange {
                weight_index: 0,
                weights_len: 0,
            }
        );
    }

    // ---------- block() arithmetic ----------

    #[test]
    fn block_applies_q_times_weight_times_step_per_coefficient() {
        let bs = BlockSize::S256;
        let half = bs.samples() / 2;
        let layout = QuantBandLayout::for_block(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, half, 1).unwrap(),
            ],
            bs,
        )
        .unwrap();
        let weights = [2.0, 3.0];
        let s = step(5.0);
        let stage = DequantStage::new(bs, &layout, &weights, s).unwrap();

        let m = bs.samples() as usize;
        let mut q = vec![0_i32; m];
        q[0] = 1; // band 0
        q[1] = -4; // band 0
        q[half as usize] = 7; // band 1
        q[m - 1] = -2; // band 1

        let out = stage.block(&q).unwrap();
        // coeff_hat = q * Q[d] * step
        assert_eq!(out[0], dequantize_sample(1, 2.0, 5.0));
        assert_eq!(out[1], dequantize_sample(-4, 2.0, 5.0));
        assert_eq!(out[half as usize], dequantize_sample(7, 3.0, 5.0));
        assert_eq!(out[m - 1], dequantize_sample(-2, 3.0, 5.0));
    }

    #[test]
    fn block_all_zero_quantized_dequantizes_to_zero() {
        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 0);
        let stage = DequantStage::new(bs, &layout, &[7.5], step(3.0)).unwrap();
        let out = stage.block(&vec![0; bs.samples() as usize]).unwrap();
        assert!(out.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn block_is_linear_in_q() {
        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 0);
        let stage = DequantStage::new(bs, &layout, &[1.5], step(2.5)).unwrap();
        let m = bs.samples() as usize;
        let mut q1 = vec![0_i32; m];
        let mut q2 = vec![0_i32; m];
        q1[3] = 2;
        q2[3] = 6; // 3x
        let o1 = stage.block(&q1).unwrap();
        let o2 = stage.block(&q2).unwrap();
        assert_eq!(o2[3], 3.0 * o1[3]);
    }

    #[test]
    fn block_matches_dequantize_sample_over_whole_block() {
        // End-to-end equivalence against the per-sample primitive across
        // a multi-band layout and a pseudo-deterministic coefficient set.
        let bs = BlockSize::S512;
        let m = bs.samples();
        let quarter = m / 4;
        let bands = vec![
            QuantBand::new(0, quarter, 0).unwrap(),
            QuantBand::new(quarter, quarter, 1).unwrap(),
            QuantBand::new(2 * quarter, quarter, 2).unwrap(),
            QuantBand::new(3 * quarter, quarter, 0).unwrap(), // shared index 0
        ];
        let layout = QuantBandLayout::for_block(bands, bs).unwrap();
        let weights = [1.25, 2.5, 0.75];
        let s = step(1.5);
        let stage = DequantStage::new(bs, &layout, &weights, s).unwrap();

        let band_map = layout.band_map();
        let q: Vec<i32> = (0..m as i32).map(|k| ((k * 7) % 11) - 5).collect();
        let out = stage.block(&q).unwrap();
        for k in 0..m as usize {
            let d = band_map[k] as usize;
            let expected = dequantize_sample(q[k], weights[d], 1.5);
            assert_eq!(out[k], expected, "k={k}");
        }
    }

    // ---------- block() reject path ----------

    #[test]
    fn block_rejects_wrong_coefficient_count() {
        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 0);
        let stage = DequantStage::new(bs, &layout, &[1.0], step(1.0)).unwrap();
        let m = bs.samples() as usize;
        // Too short.
        let err = stage.block(&vec![0; m - 1]).unwrap_err();
        assert_eq!(
            err,
            InvalidDequant::CoeffLenMismatch {
                expected: m,
                got: m - 1,
            }
        );
        // Too long.
        let err = stage.block(&vec![0; m + 1]).unwrap_err();
        assert_eq!(
            err,
            InvalidDequant::CoeffLenMismatch {
                expected: m,
                got: m + 1,
            }
        );
        // Empty.
        let err = stage.block(&[]).unwrap_err();
        assert_eq!(
            err,
            InvalidDequant::CoeffLenMismatch {
                expected: m,
                got: 0,
            }
        );
    }

    // ---------- Cross-module composition with synthesis ----------

    #[test]
    fn dequant_output_feeds_synthesis_block_unchanged() {
        // The dequant stage output length must equal the synthesis stage
        // input length (`M`), so the two FIG.6 assemblers chain directly.
        use crate::synthesis::Synthesis;
        use crate::window::WindowPair;

        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 0);
        let dq = DequantStage::new(bs, &layout, &[1.0], step(1.0)).unwrap();

        let pair = WindowPair::orthogonal_sine(bs);
        let mut synth = Synthesis::new(bs, pair).unwrap();

        let m = bs.samples() as usize;
        let mut q = vec![0_i32; m];
        q[5] = 3;
        let coeffs = dq.block(&q).unwrap();
        assert_eq!(coeffs.len(), synth.block_len());
        // The synthesis stage accepts the dequant output without a length
        // error.
        let recon = synth.block(&coeffs).unwrap();
        assert_eq!(recon.len(), m);
    }

    // ---------- Equivalence with the precomputed BandScale path ----------

    #[test]
    fn block_equals_manual_bandscale_apply() {
        let bs = BlockSize::S256;
        let half = bs.samples() / 2;
        let layout = QuantBandLayout::for_block(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, half, 1).unwrap(),
            ],
            bs,
        )
        .unwrap();
        let weights = [3.0, 4.0];
        let s = step(2.0);
        let stage = DequantStage::new(bs, &layout, &weights, s).unwrap();

        let m = bs.samples() as usize;
        let q: Vec<i32> = (0..m as i32).map(|k| (k % 5) - 2).collect();

        let scale = BandScale::from_weights(&weights, 2.0);
        let band_map = layout.band_map();
        let mut manual = vec![0.0_f64; m];
        scale.apply(&q, &band_map, &mut manual);

        assert_eq!(stage.block(&q).unwrap(), manual);
    }

    // ---------- Error Display + Error impl ----------

    #[test]
    fn error_display_names_each_variant() {
        let e = InvalidDequant::BlockSizeMismatch {
            block_size: 256,
            layout_total: 128,
        };
        let s = e.to_string();
        assert!(s.contains("256"));
        assert!(s.contains("128"));
        assert!(s.contains("block size"));

        let e = InvalidDequant::WeightIndexOutOfRange {
            weight_index: 9,
            weights_len: 4,
        };
        let s = e.to_string();
        assert!(s.contains("weight index 9"));
        assert!(s.contains("4"));

        let e = InvalidDequant::CoeffLenMismatch {
            expected: 256,
            got: 7,
        };
        let s = e.to_string();
        assert!(s.contains("256"));
        assert!(s.contains("7"));
    }

    #[test]
    fn error_implements_std_error() {
        fn assert_error<E: std::error::Error>(_: &E) {}
        assert_error(&InvalidDequant::CoeffLenMismatch {
            expected: 1,
            got: 0,
        });
    }

    // ---------- Carrier semantics ----------

    #[test]
    fn stage_is_clone_and_eq() {
        let bs = BlockSize::S256;
        let layout = single_band_layout(bs, 0);
        let a = DequantStage::new(bs, &layout, &[2.0], step(3.0)).unwrap();
        let b = a.clone();
        assert_eq!(a, b);
        let c = DequantStage::new(bs, &layout, &[2.0], step(4.0)).unwrap();
        assert_ne!(a, c);
    }
}

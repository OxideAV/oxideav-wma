//! Decoder-side **noise-substitution fill** — the §7 patent-disclosed
//! noise generator that reconstructs a band's coefficients from a single
//! transmitted energy parameter.
//!
//! The trace doc's load-bearing citation:
//!
//! > **Noise substitution.** At low/mid bitrates the encoder can "use
//! > noise substitution to convey information in certain bands" — instead
//! > of coding coefficients, it signals that a band should be filled with
//! > a generated noise pattern **of the appropriate energy**. The
//! > decoder's **noise generator** produces the patterns for the
//! > indicated bands.
//! >   — [PATENT US7,383,180 / US7,343,291 — noise substitution;
//! >      decoder noise generator 240]
//!
//! This module is the decoder companion the [`crate::bands`] module
//! (which carries the per-band policy) explicitly deferred: its docs note
//! that "a future trace pinning the construction would extend this module
//! with a fill helper." This is that helper, scoped to exactly the part
//! the patent *does* fix — the energy contract — and no more.
//!
//! ## What the patent fixes, and what stays a `[GAP]`
//!
//! The patent pins one quantitative property of the reconstructed band:
//! the synthesized pattern carries the **appropriate energy** — i.e. the
//! single energy value the encoder transmitted for that band
//! ([`crate::bands::BandPolicy::NoiseSubstituted`]'s `energy` field). The
//! *shape* of the noise (white vs. coloured spectrum, the PRNG algorithm,
//! and any seed derivation) is **not** disclosed by the patents read for
//! this trace and stays a `[GAP]` — no generator coefficients are
//! fabricated here.
//!
//! So this module takes the noise *pattern* as a caller-supplied input
//! (whatever unit-scale sequence the upstream generator produced) and
//! applies the one transform the patent fixes: scale it so that its band
//! energy equals the transmitted `energy`. This is the same
//! caller-supplies-the-`[GAP]`-quantity posture
//! [`crate::excitation`] uses for the band-size exponent and
//! [`crate::channel_decision`] uses for its decision thresholds.
//!
//! The energy convention is the patent's own (US7,930,171: "coefficient
//! values are squared to get energies, then energies are summed within
//! each band"); this module reuses [`crate::excitation::band_raw_energy`]
//! so the squared-sum convention is pinned in one place across the crate.
//!
//! ## What is NOT in this module
//!
//! * **The noise generator itself.** The PRNG / spectral colour / seed
//!   are `[GAP]`; the caller supplies the pattern.
//! * **The per-band flag-bit encoding.** Which bands are
//!   noise-substituted is decoded upstream into a [`crate::bands::BandPlan`]
//!   (`[GAP]` wire format — see that module). This module consumes the
//!   decoded plan.
//! * **Where the fill sits in the pipeline.** Per the §8 decoder
//!   diagram, noise fill happens *after* inverse-quantize/inverse-weight
//!   and *before* the inverse MLT (module 240 precedes the transform);
//!   this module produces the coefficient block that
//!   [`crate::synthesis::Synthesis::block`] then transforms.

use crate::bands::{BandPlan, BandPolicy};
use crate::excitation::band_raw_energy;
use crate::qband::QuantBandLayout;

/// Energy of a coefficient slice under the patent's squared-sum
/// convention.
///
/// A thin re-export of [`crate::excitation::band_raw_energy`] under a
/// name that reads naturally at the noise-fill call site. The energy of
/// a band is the sum of its squared coefficients (US7,930,171); an empty
/// slice has zero energy.
#[inline]
pub fn pattern_energy(pattern: &[f64]) -> f64 {
    band_raw_energy(pattern)
}

/// The scalar gain that rescales a unit noise pattern so its band energy
/// equals `target_energy`.
///
/// Band energy is a sum of squares, so it scales as the *square* of a
/// uniform gain `g`: scaling a pattern by `g` multiplies its energy by
/// `g²`. To move from a pattern whose current energy is
/// `pattern_energy` to the transmitted `target_energy`, the gain is
/// therefore `sqrt(target_energy / pattern_energy)`.
///
/// Boundary behaviour, chosen so the decoder never produces a `NaN` or
/// `±∞` from a degenerate band:
///
/// * `target_energy <= 0.0` → gain `0.0` (the band is silent; a negative
///   target is treated as zero rather than producing a `NaN` root).
/// * `pattern_energy <= 0.0` with a positive target → gain `0.0`: an
///   all-zero pattern carries no energy to rescale, so no finite gain can
///   reach a positive target; the band is left silent rather than
///   amplified to infinity.
///
/// Both arguments are taken as already-computed energies (each a sum of
/// squares), not raw coefficient slices, so the caller controls how the
/// pattern energy was measured.
pub fn noise_scale(target_energy: f64, pattern_energy: f64) -> f64 {
    if target_energy <= 0.0 || pattern_energy <= 0.0 {
        return 0.0;
    }
    (target_energy / pattern_energy).sqrt()
}

/// Rescale a caller-supplied noise pattern in place so its band energy
/// equals `target_energy`.
///
/// Computes the pattern's current energy, derives the [`noise_scale`]
/// gain, and multiplies every sample by it. After the call
/// `pattern_energy(pattern)` equals `target_energy` (up to floating-point
/// rounding) whenever the gain was finite and non-zero. An all-zero or
/// zero-target pattern is left all-zero (the [`noise_scale`] boundary
/// rules).
pub fn fill_band_in_place(target_energy: f64, pattern: &mut [f64]) {
    let g = noise_scale(target_energy, pattern_energy(pattern));
    if g == 1.0 {
        return;
    }
    for c in pattern.iter_mut() {
        *c *= g;
    }
}

/// Rescale a caller-supplied noise pattern so its band energy equals
/// `target_energy`, returning a fresh `Vec`.
///
/// The fresh-`Vec` companion to [`fill_band_in_place`]; see that function
/// for the energy contract and the degenerate-band boundary rules.
pub fn fill_band(target_energy: f64, pattern: &[f64]) -> Vec<f64> {
    let mut out = pattern.to_vec();
    fill_band_in_place(target_energy, &mut out);
    out
}

/// Failure modes for the block-level [`NoiseFiller`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InvalidNoiseFill {
    /// The supplied [`BandPlan`] and [`QuantBandLayout`] disagree on the
    /// number of bands. The fill walks them in lockstep, so they must
    /// describe the same band partition.
    BandCountMismatch {
        /// Band count declared by the [`BandPlan`].
        plan_bands: usize,
        /// Band count declared by the [`QuantBandLayout`].
        layout_bands: usize,
    },
    /// The coefficient block handed to [`NoiseFiller::fill`] does not have
    /// `layout.total_coeffs()` entries, so it cannot be partitioned by the
    /// layout.
    CoeffLenMismatch {
        /// Coefficient count the layout tiles.
        expected: usize,
        /// Coefficient count the caller supplied.
        got: usize,
    },
    /// A noise pattern supplied for band `band` did not have exactly that
    /// band's length, so it cannot fill the band's coefficient range.
    PatternLenMismatch {
        /// Index of the offending band.
        band: usize,
        /// Length the band requires.
        expected: usize,
        /// Length the caller's pattern had.
        got: usize,
    },
}

impl core::fmt::Display for InvalidNoiseFill {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidNoiseFill::BandCountMismatch {
                plan_bands,
                layout_bands,
            } => write!(
                f,
                "oxideav-wma::noisefill: band-plan band count {plan_bands} does not match layout band count {layout_bands}",
            ),
            InvalidNoiseFill::CoeffLenMismatch { expected, got } => write!(
                f,
                "oxideav-wma::noisefill: coefficient block length {got} does not match layout total {expected}",
            ),
            InvalidNoiseFill::PatternLenMismatch {
                band,
                expected,
                got,
            } => write!(
                f,
                "oxideav-wma::noisefill: noise pattern for band {band} has length {got}, expected {expected}",
            ),
        }
    }
}

impl std::error::Error for InvalidNoiseFill {}

/// Block-level decoder noise-substitution filler.
///
/// Pairs a [`BandPlan`] (the per-band coding policy decoded upstream)
/// with the [`QuantBandLayout`] that partitions the block's
/// coefficients. [`NoiseFiller::fill`] then writes the noise-substituted
/// bands of a coefficient block from caller-supplied unit patterns,
/// each rescaled to its transmitted band energy, while leaving the
/// literal-coded bands untouched and zeroing the truncated bands (the
/// patent's "completely eliminate the coefficients in certain bands").
///
/// The filler is stateless beyond the plan and layout it holds; it can
/// be reused across blocks that share the same band partition.
#[derive(Debug, Clone)]
pub struct NoiseFiller {
    plan: BandPlan,
    layout: QuantBandLayout,
}

impl NoiseFiller {
    /// Build a filler from a decoded plan and the matching layout.
    ///
    /// Rejects with [`InvalidNoiseFill::BandCountMismatch`] when the two
    /// describe a different number of bands.
    pub fn new(plan: BandPlan, layout: QuantBandLayout) -> Result<Self, InvalidNoiseFill> {
        if plan.len() != layout.band_count() {
            return Err(InvalidNoiseFill::BandCountMismatch {
                plan_bands: plan.len(),
                layout_bands: layout.band_count(),
            });
        }
        Ok(Self { plan, layout })
    }

    /// The plan this filler carries.
    pub fn plan(&self) -> &BandPlan {
        &self.plan
    }

    /// The layout this filler carries.
    pub fn layout(&self) -> &QuantBandLayout {
        &self.layout
    }

    /// Total coefficient count the filler operates over.
    pub fn total_coeffs(&self) -> usize {
        self.layout.total_coeffs()
    }

    /// Fill the noise-substituted and truncated bands of `coeffs` in
    /// place.
    ///
    /// `coeffs` is the block's coefficient buffer; on entry the
    /// literal-coded bands already hold their dequantized values (e.g.
    /// the output of [`crate::dequant::DequantStage::block`]). For each
    /// band, indexed by `d`:
    ///
    /// * [`BandPolicy::Coded`] — left untouched (its literal coefficients
    ///   stand).
    /// * [`BandPolicy::NoiseSubstituted`] — `patterns[d]` is rescaled to
    ///   the band's transmitted `energy` (via [`fill_band`]) and written
    ///   over the band's coefficient range. The pattern must have exactly
    ///   the band's length.
    /// * [`BandPolicy::Truncated`] — the band's coefficients are set to
    ///   zero (the patent's high-band elimination).
    ///
    /// `patterns` is indexed by band `d` in lockstep with the plan and
    /// layout; the entries for non-noise bands are ignored (an empty
    /// slice is fine for those).
    ///
    /// # Errors
    ///
    /// * [`InvalidNoiseFill::CoeffLenMismatch`] if `coeffs.len()` does
    ///   not equal [`NoiseFiller::total_coeffs`].
    /// * [`InvalidNoiseFill::BandCountMismatch`] if `patterns.len()` does
    ///   not equal the band count (the same lockstep invariant the
    ///   constructor enforces between plan and layout).
    /// * [`InvalidNoiseFill::PatternLenMismatch`] if a noise band's
    ///   pattern length is not that band's length.
    ///
    /// On any error `coeffs` is left unmodified.
    pub fn fill(&self, coeffs: &mut [f64], patterns: &[&[f64]]) -> Result<(), InvalidNoiseFill> {
        if coeffs.len() != self.layout.total_coeffs() {
            return Err(InvalidNoiseFill::CoeffLenMismatch {
                expected: self.layout.total_coeffs(),
                got: coeffs.len(),
            });
        }
        if patterns.len() != self.plan.len() {
            return Err(InvalidNoiseFill::BandCountMismatch {
                plan_bands: self.plan.len(),
                layout_bands: patterns.len(),
            });
        }

        // Validate every noise band's pattern length up front so a
        // mismatch leaves `coeffs` untouched (no partial fill).
        for (d, band) in self.layout.bands().enumerate() {
            if let Some(BandPolicy::NoiseSubstituted { .. }) = self.plan.policy_of(d) {
                let need = band.length() as usize;
                let got = patterns[d].len();
                if got != need {
                    return Err(InvalidNoiseFill::PatternLenMismatch {
                        band: d,
                        expected: need,
                        got,
                    });
                }
            }
        }

        // Apply. Coded bands are skipped; noise bands are rescaled;
        // truncated bands are zeroed.
        for (d, band) in self.layout.bands().enumerate() {
            let start = band.start() as usize;
            let end = band.end() as usize;
            match self.plan.policy_of(d) {
                Some(BandPolicy::Coded) => {}
                Some(BandPolicy::NoiseSubstituted { energy }) => {
                    let filled = fill_band(energy, patterns[d]);
                    coeffs[start..end].copy_from_slice(&filled);
                }
                Some(BandPolicy::Truncated) => {
                    coeffs[start..end].fill(0.0);
                }
                None => {
                    // Unreachable: the constructor pinned plan.len() ==
                    // layout.band_count(), and `d` ranges over the layout
                    // bands. Treat as a no-op defensively.
                }
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::BlockSize;
    use crate::qband::{QuantBand, QuantBandLayout};

    fn layout(bands: &[(u16, u16)], total: usize) -> QuantBandLayout {
        let v: Vec<QuantBand> = bands
            .iter()
            .enumerate()
            .map(|(i, &(start, len))| QuantBand::new(start, len, i as u16).unwrap())
            .collect();
        QuantBandLayout::new(v, total).unwrap()
    }

    // ---------- pattern_energy ----------

    #[test]
    fn pattern_energy_is_sum_of_squares() {
        assert_eq!(pattern_energy(&[]), 0.0);
        assert_eq!(pattern_energy(&[0.0, 0.0]), 0.0);
        assert_eq!(pattern_energy(&[1.0, 2.0, 2.0]), 9.0);
        assert_eq!(pattern_energy(&[-3.0, 4.0]), 25.0);
    }

    #[test]
    fn pattern_energy_matches_excitation_convention() {
        let s = [0.5, -1.5, 2.25, -0.125];
        assert_eq!(pattern_energy(&s), band_raw_energy(&s));
    }

    // ---------- noise_scale ----------

    #[test]
    fn noise_scale_is_sqrt_of_energy_ratio() {
        // unit-energy pattern → target 9 → gain 3.
        assert_eq!(noise_scale(9.0, 1.0), 3.0);
        // pattern energy 4 → target 9 → gain 1.5.
        assert_eq!(noise_scale(9.0, 4.0), 1.5);
        // equal energies → unit gain.
        assert_eq!(noise_scale(7.0, 7.0), 1.0);
    }

    #[test]
    fn noise_scale_zero_target_is_silent() {
        assert_eq!(noise_scale(0.0, 5.0), 0.0);
        assert_eq!(noise_scale(-1.0, 5.0), 0.0);
    }

    #[test]
    fn noise_scale_zero_pattern_energy_is_silent_not_infinite() {
        let g = noise_scale(9.0, 0.0);
        assert_eq!(g, 0.0);
        assert!(g.is_finite());
    }

    // ---------- fill_band ----------

    #[test]
    fn fill_band_reaches_target_energy() {
        let pattern = [1.0, -1.0, 1.0, -1.0]; // energy 4
        let filled = fill_band(16.0, &pattern);
        // gain = sqrt(16/4) = 2 → each ±2 → energy 16.
        assert!((pattern_energy(&filled) - 16.0).abs() < 1e-12);
        for &c in &filled {
            assert!((c.abs() - 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn fill_band_preserves_pattern_shape() {
        // Rescaling is a uniform gain, so the *ratios* between samples
        // are preserved.
        let pattern = [1.0, 2.0, -3.0];
        let filled = fill_band(56.0, &pattern); // energy 14 → gain 2
        assert!((filled[0] - 2.0).abs() < 1e-12);
        assert!((filled[1] - 4.0).abs() < 1e-12);
        assert!((filled[2] + 6.0).abs() < 1e-12);
    }

    #[test]
    fn fill_band_zero_target_silences() {
        let filled = fill_band(0.0, &[3.0, 4.0]);
        assert_eq!(filled, vec![0.0, 0.0]);
    }

    #[test]
    fn fill_band_all_zero_pattern_stays_zero() {
        let filled = fill_band(9.0, &[0.0, 0.0, 0.0]);
        assert_eq!(filled, vec![0.0, 0.0, 0.0]);
    }

    #[test]
    fn fill_band_unit_gain_leaves_pattern_unchanged() {
        let pattern = [0.3, -0.7, 1.1];
        let e = pattern_energy(&pattern);
        let filled = fill_band(e, &pattern);
        assert_eq!(filled, pattern.to_vec());
    }

    #[test]
    fn fill_band_in_place_matches_fresh_vec() {
        let pattern = [0.2, -1.3, 4.4, -0.9];
        let fresh = fill_band(10.0, &pattern);
        let mut inplace = pattern.to_vec();
        fill_band_in_place(10.0, &mut inplace);
        assert_eq!(fresh, inplace);
    }

    #[test]
    fn fill_band_empty_is_noop() {
        assert_eq!(fill_band(5.0, &[]), Vec::<f64>::new());
    }

    // ---------- NoiseFiller construction ----------

    #[test]
    fn filler_new_accepts_matching_band_counts() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 1.0 },
        ]);
        assert!(NoiseFiller::new(plan, lay).is_ok());
    }

    #[test]
    fn filler_new_rejects_band_count_mismatch() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let err = NoiseFiller::new(plan, lay).unwrap_err();
        assert_eq!(
            err,
            InvalidNoiseFill::BandCountMismatch {
                plan_bands: 1,
                layout_bands: 2,
            }
        );
    }

    #[test]
    fn filler_accessors_report_carried_state() {
        let lay = layout(&[(0, 3), (3, 1)], 4);
        let plan = BandPlan::new(vec![BandPolicy::Coded, BandPolicy::Truncated]);
        let f = NoiseFiller::new(plan.clone(), lay.clone()).unwrap();
        assert_eq!(f.plan(), &plan);
        assert_eq!(f.layout(), &lay);
        assert_eq!(f.total_coeffs(), 4);
    }

    // ---------- NoiseFiller::fill ----------

    #[test]
    fn fill_leaves_coded_bands_untouched() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![BandPolicy::Coded, BandPolicy::Coded]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [1.0, 2.0, 3.0, 4.0];
        let empty: &[f64] = &[];
        f.fill(&mut coeffs, &[empty, empty]).unwrap();
        assert_eq!(coeffs, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn fill_writes_noise_band_at_target_energy() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 8.0 },
        ]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [5.0, 6.0, 0.0, 0.0];
        let noise = [1.0, -1.0]; // pattern energy 2 → gain 2
        let empty: &[f64] = &[];
        f.fill(&mut coeffs, &[empty, &noise]).unwrap();
        // Coded band 0 preserved.
        assert_eq!(&coeffs[0..2], &[5.0, 6.0]);
        // Noise band 1 rescaled to energy 8.
        let band_energy = coeffs[2] * coeffs[2] + coeffs[3] * coeffs[3];
        assert!((band_energy - 8.0).abs() < 1e-12);
    }

    #[test]
    fn fill_zeroes_truncated_bands() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![BandPolicy::Coded, BandPolicy::Truncated]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [1.0, 2.0, 9.9, 9.9];
        let empty: &[f64] = &[];
        f.fill(&mut coeffs, &[empty, empty]).unwrap();
        assert_eq!(coeffs, [1.0, 2.0, 0.0, 0.0]);
    }

    #[test]
    fn fill_handles_all_three_policies_in_one_block() {
        let lay = layout(&[(0, 2), (2, 2), (4, 2)], 6);
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 18.0 },
            BandPolicy::Truncated,
        ]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [1.0, 1.0, 0.0, 0.0, 7.0, 7.0];
        let noise = [1.0, 2.0]; // energy 5 → gain sqrt(18/5)
        let empty: &[f64] = &[];
        f.fill(&mut coeffs, &[empty, &noise, empty]).unwrap();
        assert_eq!(&coeffs[0..2], &[1.0, 1.0]); // coded
        let be = coeffs[2] * coeffs[2] + coeffs[3] * coeffs[3];
        assert!((be - 18.0).abs() < 1e-12); // noise
        assert_eq!(&coeffs[4..6], &[0.0, 0.0]); // truncated
    }

    #[test]
    fn fill_rejects_coeff_len_mismatch() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![BandPolicy::Coded, BandPolicy::Coded]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [1.0, 2.0, 3.0]; // wrong length
        let empty: &[f64] = &[];
        assert_eq!(
            f.fill(&mut coeffs, &[empty, empty]),
            Err(InvalidNoiseFill::CoeffLenMismatch {
                expected: 4,
                got: 3,
            })
        );
        // unmodified on error
        assert_eq!(coeffs, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn fill_rejects_pattern_count_mismatch() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 1.0 },
        ]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [1.0, 2.0, 3.0, 4.0];
        let noise = [1.0, 1.0];
        assert_eq!(
            f.fill(&mut coeffs, &[&noise]), // only one pattern for two bands
            Err(InvalidNoiseFill::BandCountMismatch {
                plan_bands: 2,
                layout_bands: 1,
            })
        );
        assert_eq!(coeffs, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn fill_rejects_pattern_len_mismatch_without_mutating() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 4.0 },
        ]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [1.0, 2.0, 3.0, 4.0];
        let noise = [1.0, 1.0, 1.0]; // length 3, band needs 2
        let empty: &[f64] = &[];
        assert_eq!(
            f.fill(&mut coeffs, &[empty, &noise]),
            Err(InvalidNoiseFill::PatternLenMismatch {
                band: 1,
                expected: 2,
                got: 3,
            })
        );
        // no partial fill — block untouched.
        assert_eq!(coeffs, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn fill_zero_energy_noise_band_is_silenced() {
        let lay = layout(&[(0, 2)], 2);
        let plan = BandPlan::new(vec![BandPolicy::NoiseSubstituted { energy: 0.0 }]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let mut coeffs = [9.0, 9.0];
        let noise = [1.0, 1.0];
        f.fill(&mut coeffs, &[&noise]).unwrap();
        assert_eq!(coeffs, [0.0, 0.0]);
    }

    #[test]
    fn fill_over_every_block_size_single_noise_band() {
        // The filler partitions and fills a whole block for each member
        // of the patent-disclosed transform-block-size set.
        for bs in BlockSize::ALL {
            let n = bs.samples() as usize;
            let lay =
                QuantBandLayout::for_block(vec![QuantBand::new(0, bs.samples(), 0).unwrap()], bs)
                    .unwrap();
            let plan = BandPlan::new(vec![BandPolicy::NoiseSubstituted { energy: 100.0 }]);
            let f = NoiseFiller::new(plan, lay).unwrap();
            let mut coeffs = vec![0.0; n];
            let pattern = vec![1.0; n]; // energy n → gain sqrt(100/n)
            f.fill(&mut coeffs, &[&pattern]).unwrap();
            assert!((pattern_energy(&coeffs) - 100.0).abs() < 1e-6);
        }
    }

    #[test]
    fn fill_is_reusable_across_blocks() {
        let lay = layout(&[(0, 2), (2, 2)], 4);
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 2.0 },
        ]);
        let f = NoiseFiller::new(plan, lay).unwrap();
        let noise = [1.0, 1.0]; // energy 2 → gain 1
        let empty: &[f64] = &[];

        let mut b1 = [1.0, 1.0, 0.0, 0.0];
        f.fill(&mut b1, &[empty, &noise]).unwrap();
        let mut b2 = [2.0, 2.0, 0.0, 0.0];
        f.fill(&mut b2, &[empty, &noise]).unwrap();

        assert_eq!(&b1[2..4], &[1.0, 1.0]);
        assert_eq!(&b2[2..4], &[1.0, 1.0]);
    }

    // ---------- error Display + trait impls ----------

    #[test]
    fn error_display_names_the_module_and_fields() {
        let e = InvalidNoiseFill::BandCountMismatch {
            plan_bands: 2,
            layout_bands: 3,
        };
        let s = format!("{e}");
        assert!(s.contains("noisefill"));
        assert!(s.contains('2') && s.contains('3'));

        let e = InvalidNoiseFill::CoeffLenMismatch {
            expected: 4,
            got: 5,
        };
        assert!(format!("{e}").contains("noisefill"));

        let e = InvalidNoiseFill::PatternLenMismatch {
            band: 1,
            expected: 2,
            got: 3,
        };
        let s = format!("{e}");
        assert!(s.contains("band 1"));
    }

    #[test]
    fn error_is_std_error() {
        fn assert_err<E: std::error::Error>(_: &E) {}
        assert_err(&InvalidNoiseFill::CoeffLenMismatch {
            expected: 1,
            got: 0,
        });
    }
}

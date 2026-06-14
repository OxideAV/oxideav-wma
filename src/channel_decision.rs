//! WMA open-loop stereo (channel-coding) decision.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §5 describes, beside
//! the sum/difference transform itself (realised in [`crate::stereo`]),
//! the encoder-side *decision* that selects whether a given stereo
//! block is coded with the two channels independent or jointly as
//! sum/difference:
//!
//! > The decision to code channels independently vs. jointly is an
//! > **open-loop decision** based on inter-channel energy separation and
//! > the disparity of excitation patterns.
//! >   — [PATENT US7,502,743]   [DSP]
//!
//! "Open-loop" means the choice is made from an analysis of the input
//! signal — *before* the rate/distortion quantization loop runs — rather
//! than by trial-coding both arrangements and measuring the result. The
//! two patent-named inputs to that analysis are:
//!
//! 1. **inter-channel energy separation** — how differently the left and
//!    right channels are excited (correlated channels concentrate energy
//!    in the sum channel, so sum/difference wins; uncorrelated channels
//!    spread energy and independent coding wins); and
//! 2. **disparity of excitation patterns** — how differently the two
//!    channels' per-band excitation patterns (the §4 energy-derived
//!    quantization-matrix inputs, [`crate::excitation`]) are shaped:
//!    closely-matched patterns favour joint coding, divergent ones
//!    favour independent coding.
//!
//! ## Scope of this module — what is patent-backed and what is `[GAP]`
//!
//! The patent discloses *that* the decision exists and *which two
//! signal-analysis quantities drive it*. It does **not** disclose:
//!
//! * the exact combining rule that turns the two quantities into a
//!   binary choice (the threshold constants are encoder-analysis tuning
//!   values), nor
//! * the v1/v2 bitstream flag that records the chosen mode (the §5
//!   trace marks the flag layout `[GAP]`).
//!
//! So this module models the decision the way Round 15's
//! [`crate::excitation`] modelled the energy-derived matrix: it
//! **computes the patent-named analysis quantities exactly** from the
//! input, and exposes a decision helper whose **thresholds are
//! caller-supplied** — never fabricated. The endpoints with closed-form
//! meaning (perfectly-correlated / perfectly-decorrelated channels,
//! identical / maximally-disparate excitation patterns) are pinned by
//! tests; the tuning between them is the encoder's `[GAP]` policy.
//!
//! No bitstream flag is emitted or parsed here — only the open-loop
//! analysis the patent names.

use crate::qband::QuantBandLayout;

/// The two channel-coding arrangements the §5 decision selects between.
///
/// Per the trace the WMA Standard v1/v2 multi-channel coding is exactly
/// these two alternatives; the richer multi-channel transform selection
/// (identity / Hadamard / DCT-II / Givens rotations) is a WMA Pro–era
/// superset and is out of scope (see [`crate::stereo`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ChannelMode {
    /// Code the left and right channels independently.
    Independent,
    /// Code the channels jointly as sum/difference (mid/side), applying
    /// the [`crate::stereo`] transform before quantization.
    SumDifference,
}

impl ChannelMode {
    /// Both modes in a fixed iteration order (independent first).
    pub const ALL: [ChannelMode; 2] = [ChannelMode::Independent, ChannelMode::SumDifference];

    /// The other mode. Involutive: `m.opposite().opposite() == m`.
    #[inline]
    pub fn opposite(self) -> ChannelMode {
        match self {
            ChannelMode::Independent => ChannelMode::SumDifference,
            ChannelMode::SumDifference => ChannelMode::Independent,
        }
    }

    /// `true` for the joint sum/difference arrangement.
    #[inline]
    pub fn is_joint(self) -> bool {
        matches!(self, ChannelMode::SumDifference)
    }
}

impl core::fmt::Display for ChannelMode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ChannelMode::Independent => f.write_str("independent"),
            ChannelMode::SumDifference => f.write_str("sum/difference"),
        }
    }
}

/// Error raised when an open-loop analysis is asked of mismatched or
/// empty inputs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InvalidStereoAnalysis {
    /// The two channel slices have different lengths.
    LengthMismatch {
        /// Length of the left-channel slice.
        left: usize,
        /// Length of the right-channel slice.
        right: usize,
    },
    /// An analysis quantity is undefined for empty input (the
    /// inter-channel energy separation divides by total energy, which is
    /// zero for an empty or all-zero pair).
    NoEnergy,
}

impl core::fmt::Display for InvalidStereoAnalysis {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidStereoAnalysis::LengthMismatch { left, right } => write!(
                f,
                "oxideav-wma::channel_decision: channel slices differ in length (left {left}, right {right})",
            ),
            InvalidStereoAnalysis::NoEnergy => f.write_str(
                "oxideav-wma::channel_decision: inter-channel energy separation is undefined for a pair with zero total energy",
            ),
        }
    }
}

impl std::error::Error for InvalidStereoAnalysis {}

/// The patent's first decision input: **inter-channel energy
/// separation**.
///
/// This is computed as the fraction of the total stereo energy that the
/// sum/difference transform would place in the *difference* (side)
/// channel:
///
/// ```text
///     separation = E_side / (E_mid + E_side)
/// ```
///
/// where `E_mid` and `E_side` are the energies of the mid and side
/// signals the [`crate::stereo`] transform produces. Because the
/// sum/difference transform is energy-preserving up to its `1/2` scaling
/// (`E_mid + E_side = (E_L + E_R) / 2`), this is equivalently the
/// fraction of total energy that is *anti-correlated* between the
/// channels:
///
/// * `0.0` — perfectly correlated channels (`L == R`): all energy is in
///   the sum channel, none in the difference. Joint coding is maximally
///   favoured.
/// * `0.5` — fully decorrelated channels of equal power: energy is split
///   evenly, so the sum/difference transform buys nothing.
/// * `1.0` — perfectly anti-correlated channels (`L == -R`): all energy
///   in the difference channel.
///
/// A *low* separation is the signal that channels are correlated and the
/// sum/difference arrangement concentrates energy — the patent's
/// rationale for choosing joint coding.
///
/// Returns [`InvalidStereoAnalysis::LengthMismatch`] when the slices
/// differ in length and [`InvalidStereoAnalysis::NoEnergy`] when the
/// total energy is zero (an all-zero or empty pair).
pub fn inter_channel_energy_separation(
    left: &[f64],
    right: &[f64],
) -> Result<f64, InvalidStereoAnalysis> {
    if left.len() != right.len() {
        return Err(InvalidStereoAnalysis::LengthMismatch {
            left: left.len(),
            right: right.len(),
        });
    }
    let mut e_mid = 0.0f64;
    let mut e_side = 0.0f64;
    for (&l, &r) in left.iter().zip(right.iter()) {
        let m = crate::stereo::mid(l, r);
        let s = crate::stereo::side(l, r);
        e_mid += m * m;
        e_side += s * s;
    }
    let total = e_mid + e_side;
    if total == 0.0 {
        return Err(InvalidStereoAnalysis::NoEnergy);
    }
    Ok(e_side / total)
}

/// The patent's second decision input: **disparity of excitation
/// patterns**.
///
/// The §4 excitation pattern ([`crate::excitation::excitation_pattern`])
/// is the per-band energy shape that drives each channel's quantization
/// matrix. This function measures how *differently* the two channels are
/// shaped by comparing their normalised excitation patterns band by
/// band.
///
/// Each channel's per-band excitation vector is first `L1`-normalised to
/// a unit-sum shape (so the comparison is of *shape*, not of overall
/// loudness — a quiet and a loud channel with identical spectral balance
/// have zero disparity). The disparity is then half the `L1` distance
/// between the two normalised shapes:
///
/// ```text
///     disparity = 0.5 * Σ_d | p_L[d] − p_R[d] |
/// ```
///
/// which lies in `[0.0, 1.0]`:
///
/// * `0.0` — identical excitation shapes; joint coding is favoured (one
///   shared quantization arrangement serves both channels well).
/// * `1.0` — disjoint excitation shapes (the channels put their energy
///   in entirely different bands); independent coding is favoured.
///
/// Both channels' coefficients are partitioned by the same `layout`
/// (the decision is per-tile, so both channels share the band
/// structure). The `exponent` is the [`crate::excitation`] band-size
/// exponent (a `[GAP]` encoder constant, supplied by the caller).
///
/// Returns [`InvalidStereoAnalysis::NoEnergy`] when either channel's
/// excitation pattern sums to zero (an all-zero channel has no shape to
/// compare).
pub fn excitation_pattern_disparity(
    left_coeffs: &[f64],
    right_coeffs: &[f64],
    layout: &QuantBandLayout,
    exponent: f64,
) -> Result<f64, InvalidStereoAnalysis> {
    let p_l = crate::excitation::excitation_pattern(left_coeffs, layout, exponent);
    let p_r = crate::excitation::excitation_pattern(right_coeffs, layout, exponent);
    let sum_l: f64 = p_l.iter().sum();
    let sum_r: f64 = p_r.iter().sum();
    if sum_l == 0.0 || sum_r == 0.0 {
        return Err(InvalidStereoAnalysis::NoEnergy);
    }
    let mut acc = 0.0f64;
    for (&a, &b) in p_l.iter().zip(p_r.iter()) {
        acc += (a / sum_l - b / sum_r).abs();
    }
    Ok(0.5 * acc)
}

/// The patent's open-loop channel-coding decision, parameterised by the
/// two caller-supplied thresholds the patent leaves as `[GAP]` tuning.
///
/// The patent names the two analysis quantities but not the rule that
/// combines them. This carrier holds the two decision thresholds and
/// applies the only combining rule consistent with the patent's stated
/// rationale: choose joint sum/difference coding when **both** patent
/// criteria favour it — the channels are correlated (low inter-channel
/// energy separation) **and** their excitation shapes are similar (low
/// disparity). If either criterion is unfavourable the channels are
/// coded independently.
///
/// Both thresholds are upper bounds: a quantity at or below its
/// threshold is "favourable" to joint coding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpenLoopDecision {
    /// Upper bound on [`inter_channel_energy_separation`] for joint
    /// coding to be favoured by the energy criterion.
    pub max_energy_separation: f64,
    /// Upper bound on [`excitation_pattern_disparity`] for joint coding
    /// to be favoured by the excitation criterion.
    pub max_excitation_disparity: f64,
}

impl OpenLoopDecision {
    /// Build a decision carrier from the two `[GAP]` tuning thresholds.
    ///
    /// The thresholds are not validated against a fixed range because
    /// the patent does not disclose one; a caller that passes a
    /// threshold outside `[0.0, 1.0]` simply makes that criterion
    /// always- or never-favourable, which is a legitimate degenerate
    /// policy.
    #[inline]
    pub fn new(max_energy_separation: f64, max_excitation_disparity: f64) -> Self {
        OpenLoopDecision {
            max_energy_separation,
            max_excitation_disparity,
        }
    }

    /// Decide the channel mode from pre-computed analysis quantities.
    ///
    /// Returns [`ChannelMode::SumDifference`] iff the energy separation
    /// is at or below [`OpenLoopDecision::max_energy_separation`] **and**
    /// the excitation disparity is at or below
    /// [`OpenLoopDecision::max_excitation_disparity`]; otherwise
    /// [`ChannelMode::Independent`].
    #[inline]
    pub fn decide(&self, energy_separation: f64, excitation_disparity: f64) -> ChannelMode {
        let energy_favours_joint = energy_separation <= self.max_energy_separation;
        let excitation_favours_joint = excitation_disparity <= self.max_excitation_disparity;
        if energy_favours_joint && excitation_favours_joint {
            ChannelMode::SumDifference
        } else {
            ChannelMode::Independent
        }
    }

    /// End-to-end open-loop decision over raw stereo coefficient blocks.
    ///
    /// Computes both patent-named analysis quantities from the inputs
    /// ([`inter_channel_energy_separation`] over the channel samples and
    /// [`excitation_pattern_disparity`] over the same coefficients
    /// partitioned by `layout`) and applies [`OpenLoopDecision::decide`].
    ///
    /// The same coefficient slices serve both analyses: the §4
    /// excitation pattern is computed on the spectral coefficients, and
    /// the inter-channel energy separation is the energy relationship
    /// between those same per-channel coefficient vectors.
    ///
    /// Propagates [`InvalidStereoAnalysis`] from either sub-analysis.
    pub fn decide_blocks(
        &self,
        left_coeffs: &[f64],
        right_coeffs: &[f64],
        layout: &QuantBandLayout,
        exponent: f64,
    ) -> Result<ChannelMode, InvalidStereoAnalysis> {
        let separation = inter_channel_energy_separation(left_coeffs, right_coeffs)?;
        let disparity = excitation_pattern_disparity(left_coeffs, right_coeffs, layout, exponent)?;
        Ok(self.decide(separation, disparity))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qband::{QuantBand, QuantBandLayout};

    fn layout_two_bands(total: u16) -> QuantBandLayout {
        let half = total / 2;
        QuantBandLayout::new(
            vec![
                QuantBand::new(0, half, 0).unwrap(),
                QuantBand::new(half, total - half, 1).unwrap(),
            ],
            total as usize,
        )
        .unwrap()
    }

    // ---------- ChannelMode ----------

    #[test]
    fn mode_all_order_is_independent_first() {
        assert_eq!(
            ChannelMode::ALL,
            [ChannelMode::Independent, ChannelMode::SumDifference]
        );
    }

    #[test]
    fn mode_opposite_is_involutive() {
        for m in ChannelMode::ALL {
            assert_eq!(m.opposite().opposite(), m);
            assert_ne!(m.opposite(), m);
        }
    }

    #[test]
    fn mode_is_joint_only_for_sum_difference() {
        assert!(ChannelMode::SumDifference.is_joint());
        assert!(!ChannelMode::Independent.is_joint());
    }

    #[test]
    fn mode_display_names() {
        assert_eq!(ChannelMode::Independent.to_string(), "independent");
        assert_eq!(ChannelMode::SumDifference.to_string(), "sum/difference");
    }

    // ---------- inter_channel_energy_separation ----------

    #[test]
    fn separation_zero_for_perfectly_correlated_channels() {
        // L == R: all energy in the mid channel, none in side.
        let l = [0.5, -0.3, 0.9, 0.1];
        let r = l;
        let sep = inter_channel_energy_separation(&l, &r).unwrap();
        assert_eq!(sep, 0.0);
    }

    #[test]
    fn separation_one_for_perfectly_anticorrelated_channels() {
        // L == -R: all energy in the side channel.
        let l = [0.5, -0.3, 0.9, 0.1];
        let r = [-0.5, 0.3, -0.9, -0.1];
        let sep = inter_channel_energy_separation(&l, &r).unwrap();
        assert_eq!(sep, 1.0);
    }

    #[test]
    fn separation_half_for_independent_equal_power_channels() {
        // One channel carries all the signal, the other is silent: the
        // mid and side energies are equal, so separation is 0.5.
        let l = [1.0, 2.0, 3.0];
        let r = [0.0, 0.0, 0.0];
        let sep = inter_channel_energy_separation(&l, &r).unwrap();
        assert!((sep - 0.5).abs() < 1e-12, "sep = {sep}");
    }

    #[test]
    fn separation_length_mismatch_rejected() {
        let e = inter_channel_energy_separation(&[1.0, 2.0], &[1.0]).unwrap_err();
        assert_eq!(
            e,
            InvalidStereoAnalysis::LengthMismatch { left: 2, right: 1 }
        );
    }

    #[test]
    fn separation_no_energy_rejected() {
        let e = inter_channel_energy_separation(&[0.0, 0.0], &[0.0, 0.0]).unwrap_err();
        assert_eq!(e, InvalidStereoAnalysis::NoEnergy);
        // Empty pair is also energy-free.
        let e2 = inter_channel_energy_separation(&[], &[]).unwrap_err();
        assert_eq!(e2, InvalidStereoAnalysis::NoEnergy);
    }

    #[test]
    fn separation_is_scale_invariant_in_amplitude() {
        // Scaling both channels by the same factor scales E_mid and
        // E_side equally, so the fraction is unchanged.
        let l = [0.5, -0.3, 0.9, 0.1];
        let r = [0.2, 0.4, -0.1, 0.8];
        let s1 = inter_channel_energy_separation(&l, &r).unwrap();
        let l2: Vec<f64> = l.iter().map(|x| x * 7.0).collect();
        let r2: Vec<f64> = r.iter().map(|x| x * 7.0).collect();
        let s2 = inter_channel_energy_separation(&l2, &r2).unwrap();
        assert!((s1 - s2).abs() < 1e-12, "{s1} vs {s2}");
    }

    // ---------- excitation_pattern_disparity ----------

    #[test]
    fn disparity_zero_for_identical_channels() {
        let l = [1.0, 2.0, 3.0, 4.0];
        let r = l;
        let layout = layout_two_bands(4);
        let d = excitation_pattern_disparity(&l, &r, &layout, 1.0).unwrap();
        assert!(d.abs() < 1e-12, "d = {d}");
    }

    #[test]
    fn disparity_zero_for_same_shape_different_loudness() {
        // Right channel is the left scaled up: identical *shape*, so the
        // normalised disparity is zero.
        let l = [1.0, 2.0, 3.0, 4.0];
        let r: Vec<f64> = l.iter().map(|x| x * 10.0).collect();
        let layout = layout_two_bands(4);
        let d = excitation_pattern_disparity(&l, &r, &layout, 1.0).unwrap();
        assert!(d.abs() < 1e-12, "d = {d}");
    }

    #[test]
    fn disparity_one_for_disjoint_band_energy() {
        // Left channel puts all energy in band 0, right in band 1.
        let l = [3.0, 4.0, 0.0, 0.0];
        let r = [0.0, 0.0, 5.0, 12.0];
        let layout = layout_two_bands(4);
        let d = excitation_pattern_disparity(&l, &r, &layout, 1.0).unwrap();
        assert!((d - 1.0).abs() < 1e-12, "d = {d}");
    }

    #[test]
    fn disparity_no_energy_when_a_channel_is_silent() {
        let l = [1.0, 2.0, 3.0, 4.0];
        let r = [0.0, 0.0, 0.0, 0.0];
        let layout = layout_two_bands(4);
        let e = excitation_pattern_disparity(&l, &r, &layout, 1.0).unwrap_err();
        assert_eq!(e, InvalidStereoAnalysis::NoEnergy);
    }

    #[test]
    fn disparity_in_unit_interval_for_arbitrary_inputs() {
        let l = [0.5, -1.5, 2.0, 0.25];
        let r = [-0.3, 0.8, 0.1, -2.0];
        let layout = layout_two_bands(4);
        let d = excitation_pattern_disparity(&l, &r, &layout, 1.0).unwrap();
        assert!((0.0..=1.0).contains(&d), "d = {d}");
    }

    // ---------- OpenLoopDecision::decide ----------

    #[test]
    fn decide_joint_when_both_criteria_favourable() {
        let dec = OpenLoopDecision::new(0.2, 0.3);
        assert_eq!(dec.decide(0.1, 0.1), ChannelMode::SumDifference);
        // Exactly at both bounds is still favourable (inclusive).
        assert_eq!(dec.decide(0.2, 0.3), ChannelMode::SumDifference);
    }

    #[test]
    fn decide_independent_when_energy_criterion_fails() {
        let dec = OpenLoopDecision::new(0.2, 0.3);
        assert_eq!(dec.decide(0.5, 0.1), ChannelMode::Independent);
    }

    #[test]
    fn decide_independent_when_excitation_criterion_fails() {
        let dec = OpenLoopDecision::new(0.2, 0.3);
        assert_eq!(dec.decide(0.1, 0.9), ChannelMode::Independent);
    }

    #[test]
    fn decide_independent_when_both_criteria_fail() {
        let dec = OpenLoopDecision::new(0.2, 0.3);
        assert_eq!(dec.decide(0.9, 0.9), ChannelMode::Independent);
    }

    #[test]
    fn decision_carrier_fields_round_trip() {
        let dec = OpenLoopDecision::new(0.4, 0.6);
        assert_eq!(dec.max_energy_separation, 0.4);
        assert_eq!(dec.max_excitation_disparity, 0.6);
    }

    // ---------- OpenLoopDecision::decide_blocks (end to end) ----------

    #[test]
    fn decide_blocks_picks_joint_for_correlated_matched_channels() {
        // Identical channels: separation 0.0, disparity 0.0 — both
        // criteria favour joint coding under any non-negative threshold.
        let l = [1.0, 2.0, 3.0, 4.0];
        let r = l;
        let layout = layout_two_bands(4);
        let dec = OpenLoopDecision::new(0.1, 0.1);
        let mode = dec.decide_blocks(&l, &r, &layout, 1.0).unwrap();
        assert_eq!(mode, ChannelMode::SumDifference);
    }

    #[test]
    fn decide_blocks_picks_independent_for_anticorrelated_channels() {
        // L == -R: separation 1.0 — the energy criterion fails.
        let l = [0.5, -0.3, 0.9, 0.1];
        let r = [-0.5, 0.3, -0.9, -0.1];
        let layout = layout_two_bands(4);
        let dec = OpenLoopDecision::new(0.5, 0.5);
        let mode = dec.decide_blocks(&l, &r, &layout, 1.0).unwrap();
        assert_eq!(mode, ChannelMode::Independent);
    }

    #[test]
    fn decide_blocks_picks_independent_for_disjoint_excitation() {
        // Disjoint band energy: disparity 1.0 — the excitation criterion
        // fails even though the channels are not anti-correlated.
        let l = [3.0, 4.0, 0.0, 0.0];
        let r = [0.0, 0.0, 5.0, 12.0];
        let layout = layout_two_bands(4);
        let dec = OpenLoopDecision::new(0.9, 0.5);
        let mode = dec.decide_blocks(&l, &r, &layout, 1.0).unwrap();
        assert_eq!(mode, ChannelMode::Independent);
    }

    #[test]
    fn decide_blocks_propagates_length_mismatch() {
        let layout = layout_two_bands(4);
        let dec = OpenLoopDecision::new(0.5, 0.5);
        let e = dec
            .decide_blocks(&[1.0, 2.0, 3.0, 4.0], &[1.0, 2.0], &layout, 1.0)
            .unwrap_err();
        assert!(matches!(e, InvalidStereoAnalysis::LengthMismatch { .. }));
    }

    #[test]
    fn decide_blocks_propagates_no_energy() {
        let layout = layout_two_bands(4);
        let dec = OpenLoopDecision::new(0.5, 0.5);
        let e = dec
            .decide_blocks(&[0.0; 4], &[0.0; 4], &layout, 1.0)
            .unwrap_err();
        assert_eq!(e, InvalidStereoAnalysis::NoEnergy);
    }

    // ---------- error Display ----------

    #[test]
    fn error_display_names_the_cause() {
        let m = InvalidStereoAnalysis::LengthMismatch { left: 4, right: 2 };
        assert!(m.to_string().contains("differ in length"));
        let n = InvalidStereoAnalysis::NoEnergy;
        assert!(n.to_string().contains("zero total energy"));
    }

    #[test]
    fn chosen_mode_composes_with_stereo_transform() {
        // When the decision picks joint coding, the corresponding
        // transform is the one in `crate::stereo`; verify the two layers
        // agree on what "joint" means by round-tripping a correlated
        // pair through the transform the decision selected.
        let l = [0.5, 1.5, -2.5, 7.0];
        let r = [0.5, 1.5, -2.5, 7.0];
        let layout = layout_two_bands(4);
        let dec = OpenLoopDecision::new(0.1, 0.1);
        let mode = dec.decide_blocks(&l, &r, &layout, 1.0).unwrap();
        assert_eq!(mode, ChannelMode::SumDifference);
        // Apply the transform the chosen mode implies.
        let mut mid = l;
        let mut side = r;
        crate::stereo::forward_in_place(&mut mid, &mut side);
        // Correlated input -> all energy in mid, side is zero.
        assert_eq!(side, [0.0, 0.0, 0.0, 0.0]);
        // And the transform inverts back to the original channels.
        crate::stereo::inverse_in_place(&mut mid, &mut side);
        assert_eq!(mid, l);
        assert_eq!(side, r);
    }
}

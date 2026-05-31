//! WMA sum/difference (mid/side) stereo transform.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §5 lifts the
//! patent-disclosed two-channel transform for WMA Standard (WMA7,
//! codec IDs `0x160` / `0x161`):
//!
//! > For stereo, WMA7 can code the two channels as **sum and difference
//! > channels** — the sum being the channel average and the difference
//! > being half the channel difference (i.e. mid/side).
//! >   — [PATENT US7,930,171 — WMA7 sum/difference]
//! >   — [PATENT US7,502,743 — prior-art sum/difference baseline]
//!
//! In equations, with `L` and `R` denoting the per-sample left/right
//! amplitudes:
//!
//! ```text
//!     sum  =  (L + R) / 2
//!     diff =  (L - R) / 2
//! ```
//!
//! The inverse is the exact algebraic inverse:
//!
//! ```text
//!     L = sum + diff
//!     R = sum - diff
//! ```
//!
//! ## Scope of this module
//!
//! This module exposes the per-sample forward and inverse transform
//! plus slice-wise helpers that fan the transform across a whole
//! block of paired samples. The transform is exact for `f64` and
//! algebraically invertible.
//!
//! The **decision** to switch a given block between "code channels
//! independently" and "code channels as sum/difference" is a separate
//! open-loop encoder-side decision based on inter-channel energy
//! separation per §5 of the trace; that decision is **[GAP]** at the
//! bit-stream level (the patents disclose the decision *exists* but
//! the v1/v2 flag layout is not stated) and is therefore not
//! implemented in this module.
//!
//! ## Why a dedicated module
//!
//! The transform is the only patent-disclosed multi-channel coding
//! primitive for WMA Standard v1/v2 (the richer multi-channel
//! transform selection — identity / Hadamard / DCT-II / Givens —
//! is a WMA Pro–era superset per §5 and is out of scope for this
//! rebuild).

/// Per-sample sum (mid) of a stereo pair.
///
/// Computes `(left + right) / 2` in `f64`. The denominator is exactly
/// representable, so the only source of rounding is the floating-point
/// addition itself.
#[inline]
pub fn mid(left: f64, right: f64) -> f64 {
    (left + right) * 0.5
}

/// Per-sample difference (side) of a stereo pair.
///
/// Computes `(left - right) / 2` in `f64`.
#[inline]
pub fn side(left: f64, right: f64) -> f64 {
    (left - right) * 0.5
}

/// Forward sum/difference transform of a stereo pair, returning
/// `(mid, side) = ((L+R)/2, (L-R)/2)`.
#[inline]
pub fn forward(left: f64, right: f64) -> (f64, f64) {
    (mid(left, right), side(left, right))
}

/// Inverse sum/difference transform of a `(mid, side)` pair, returning
/// `(left, right) = (mid + side, mid - side)`.
///
/// This is the exact algebraic inverse of [`forward`] under real
/// arithmetic; in `f64` the round-trip introduces at most one ULP of
/// error per sample.
#[inline]
pub fn inverse(mid: f64, side: f64) -> (f64, f64) {
    (mid + side, mid - side)
}

/// Apply the forward sum/difference transform across paired sample
/// slices in-place. After return, `left[i]` holds the mid sample and
/// `right[i]` holds the side sample for every `i`.
///
/// Returns the number of sample pairs transformed. Both slices must
/// have the same length.
///
/// # Panics
///
/// Panics if `left.len() != right.len()`.
pub fn forward_in_place(left: &mut [f64], right: &mut [f64]) -> usize {
    assert_eq!(
        left.len(),
        right.len(),
        "oxideav-wma::stereo::forward_in_place: channel slices must have equal length",
    );
    for (l, r) in left.iter_mut().zip(right.iter_mut()) {
        let (m, s) = forward(*l, *r);
        *l = m;
        *r = s;
    }
    left.len()
}

/// Apply the inverse sum/difference transform across paired sample
/// slices in-place. Treats `mid[i]` and `side[i]` as the transformed
/// pair and overwrites them with `(left, right)`.
///
/// Returns the number of sample pairs transformed. Both slices must
/// have the same length.
///
/// # Panics
///
/// Panics if `mid.len() != side.len()`.
pub fn inverse_in_place(mid: &mut [f64], side: &mut [f64]) -> usize {
    assert_eq!(
        mid.len(),
        side.len(),
        "oxideav-wma::stereo::inverse_in_place: channel slices must have equal length",
    );
    for (m, s) in mid.iter_mut().zip(side.iter_mut()) {
        let (l, r) = inverse(*m, *s);
        *m = l;
        *s = r;
    }
    mid.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- Per-sample identities ----------

    #[test]
    fn mid_is_channel_average() {
        // The patent text: "the sum being the channel average".
        assert_eq!(mid(0.0, 0.0), 0.0);
        assert_eq!(mid(1.0, 1.0), 1.0);
        assert_eq!(mid(1.0, -1.0), 0.0);
        assert_eq!(mid(0.5, -0.5), 0.0);
        assert_eq!(mid(2.0, 4.0), 3.0);
    }

    #[test]
    fn side_is_half_the_channel_difference() {
        // The patent text: "the difference being half the channel
        // difference".
        assert_eq!(side(0.0, 0.0), 0.0);
        assert_eq!(side(1.0, 1.0), 0.0);
        assert_eq!(side(1.0, -1.0), 1.0);
        assert_eq!(side(2.0, 4.0), -1.0);
        assert_eq!(side(0.5, -0.5), 0.5);
    }

    #[test]
    fn forward_packs_mid_and_side_in_order() {
        let (m, s) = forward(3.0, 1.0);
        assert_eq!(m, 2.0);
        assert_eq!(s, 1.0);
    }

    #[test]
    fn inverse_recovers_left_right_exactly_for_integers() {
        // For inputs that produce exact-power-of-two divisions, the
        // round trip is bit-exact.
        for (l, r) in [(0.0, 0.0), (1.0, 1.0), (2.0, 4.0), (-3.0, 5.0)] {
            let (m, s) = forward(l, r);
            let (l2, r2) = inverse(m, s);
            assert_eq!(l2, l, "left round-trip mismatch for ({l}, {r})");
            assert_eq!(r2, r, "right round-trip mismatch for ({l}, {r})");
        }
    }

    // ---------- Algebraic invariants ----------

    #[test]
    fn forward_then_inverse_is_identity_under_finite_arithmetic() {
        // Sweep a deterministic set of inputs that all give
        // exactly-representable sums and differences.
        let cases = [
            (0.0f64, 0.0),
            (0.5, 0.5),
            (0.25, -0.25),
            (1.5, 2.5),
            (-7.0, 3.0),
            (1024.0, 512.0),
        ];
        for (l, r) in cases {
            let (m, s) = forward(l, r);
            let (l2, r2) = inverse(m, s);
            assert_eq!((l2, r2), (l, r), "round-trip failed for ({l}, {r})");
        }
    }

    #[test]
    fn inverse_then_forward_is_identity_for_exact_inputs() {
        // Going the other way: if the caller hands us pre-computed
        // mid/side values that are exactly representable, the
        // forward transform of the inverse must round-trip.
        let cases = [(0.0f64, 0.0), (1.0, 0.0), (0.0, 1.0), (3.0, -2.0)];
        for (m, s) in cases {
            let (l, r) = inverse(m, s);
            let (m2, s2) = forward(l, r);
            assert_eq!((m2, s2), (m, s), "round-trip failed for ({m}, {s})");
        }
    }

    #[test]
    fn correlated_input_concentrates_energy_in_mid() {
        // L == R is the maximally-correlated case; the patent claims
        // the sum/difference transform is useful precisely because
        // such inputs put all the energy in the mid channel and zero
        // in the side channel.
        let (m, s) = forward(0.7, 0.7);
        assert_eq!(m, 0.7);
        assert_eq!(s, 0.0);
    }

    #[test]
    fn anticorrelated_input_concentrates_energy_in_side() {
        // L == -R is the inverse: all energy in side, zero in mid.
        let (m, s) = forward(0.7, -0.7);
        assert_eq!(m, 0.0);
        assert_eq!(s, 0.7);
    }

    // ---------- Slice helpers ----------

    #[test]
    fn forward_in_place_returns_pair_count() {
        let mut l = vec![1.0, 2.0, 3.0, 4.0];
        let mut r = vec![1.0, 0.0, -1.0, 4.0];
        let n = forward_in_place(&mut l, &mut r);
        assert_eq!(n, 4);
        // Mid:  (1+1)/2=1.0, (2+0)/2=1.0, (3-1)/2=1.0, (4+4)/2=4.0
        // Side: (1-1)/2=0.0, (2-0)/2=1.0, (3+1)/2=2.0, (4-4)/2=0.0
        assert_eq!(l, vec![1.0, 1.0, 1.0, 4.0]);
        assert_eq!(r, vec![0.0, 1.0, 2.0, 0.0]);
    }

    #[test]
    fn forward_then_inverse_in_place_is_identity() {
        let original_l = vec![0.5, 1.5, -2.5, 7.0, 0.0];
        let original_r = vec![1.0, -1.5, 2.0, 0.5, 0.0];
        let mut l = original_l.clone();
        let mut r = original_r.clone();
        forward_in_place(&mut l, &mut r);
        inverse_in_place(&mut l, &mut r);
        assert_eq!(l, original_l);
        assert_eq!(r, original_r);
    }

    #[test]
    fn forward_in_place_on_empty_slices_returns_zero() {
        let mut l: Vec<f64> = vec![];
        let mut r: Vec<f64> = vec![];
        assert_eq!(forward_in_place(&mut l, &mut r), 0);
    }

    #[test]
    #[should_panic(expected = "channel slices must have equal length")]
    fn forward_in_place_panics_on_length_mismatch() {
        let mut l = vec![0.0, 0.0];
        let mut r = vec![0.0];
        let _ = forward_in_place(&mut l, &mut r);
    }

    #[test]
    #[should_panic(expected = "channel slices must have equal length")]
    fn inverse_in_place_panics_on_length_mismatch() {
        let mut m = vec![0.0];
        let mut s = vec![0.0, 0.0];
        let _ = inverse_in_place(&mut m, &mut s);
    }
}

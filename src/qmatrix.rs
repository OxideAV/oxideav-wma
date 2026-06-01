//! WMA quantization-matrix differential coding step.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 ("Quantization
//! matrix carriage in the bitstream (side information)") lifts the
//! three-step direct-compression pipeline the patents disclose for
//! shipping the per-band quantization matrix as side information:
//!
//! > The matrix is compressed for transmission by a **direct-compression
//! > technique** (FIG.1 of Chen-171):
//! >
//! >   1. Uniformly quantize each matrix element. **[PATENT US7,930,171
//! >      — step 110]**
//! >   2. Differentially code the quantized elements relative to
//! >      preceding elements in the matrix. **[PATENT US7,930,171 —
//! >      step 120]** **[PATENT US7,502,743 — "differentially codes the
//! >      quantized elements relative to preceding elements in the
//! >      matrix"]**
//! >   3. Huffman-code the differentially-coded elements. **[PATENT
//! >      US7,930,171 — step 130]** **[PATENT US7,502,743]**
//! >
//! > An efficiency detail: unneeded weighting factors may be set equal
//! > to the next needed one so the differential coder emits a zero
//! > delta. **[PATENT US7,502,743]**
//!
//! ## Scope of this module
//!
//! Step 2 — the differential-coding stage — is the one stage of the
//! three whose *operation* is fully specified by the patent text and
//! whose *code-word tables* are not required. The first step is a
//! uniform scalar quantization (parametric, not WMA-specific), and
//! the third step requires Huffman tables the trace marks **[GAP]**.
//!
//! This module exposes the differential transform as a pair of
//! invertible helpers:
//!
//! * [`differential_encode`] — turn a sequence of quantized matrix
//!   elements into a sequence of deltas, anchored at an explicit seed.
//! * [`differential_decode`] — invert the deltas back to the absolute
//!   quantized elements.
//!
//! The transform is integer-exact and reversible: for any seed `s` and
//! any sequence `q`, `differential_decode(s, differential_encode(s, q))`
//! reproduces `q`.
//!
//! ## What is NOT in this module
//!
//! * **Step 1 (uniform quantization).** The patent text says elements
//!   are quantized uniformly but the bin width / clip range are
//!   encoder-policy parameters, not WMA-specific patent disclosures.
//! * **Step 3 (Huffman coding of deltas).** The wiki notes a "scale
//!   Huffman table (121 entries)" and a "gain Huffman table (37
//!   entries)" for these deltas but the **table contents themselves
//!   are [GAP]** per §4 of the trace. No codewords are implemented
//!   here.
//! * **The bitstream framing.** Where in the per-frame bitstream the
//!   matrix-delta block sits, and how its length is signalled, is
//!   **[GAP]** at the byte/bit level.
//! * **The "set equal to next needed" encoder policy.** The patent
//!   discloses this as an encoder-side efficiency hint that makes the
//!   differential coder emit a zero delta. The hint is implemented in
//!   the encoder by setting `q[i] = q[i+1]` for unneeded `i`; once
//!   that is done, this module's `differential_encode` already emits
//!   the resulting zero delta. We expose [`zero_delta_pad`] as a
//!   convenience that performs the substitution against a "needed"
//!   bitmap, but the **policy of which elements count as unneeded**
//!   is encoder analysis, not a patent-disclosed bitstream rule.

use core::num::Wrapping;

/// Encode an absolute quantization-matrix sequence as deltas relative
/// to a leading `seed` value, in place over a slice.
///
/// After the call `q[0]` is `q[0] - seed`, `q[1]` is `q[1] - q[0]`
/// (using the *original* `q[0]`), and so on; i.e. the i-th output is
/// `original[i] - original[i-1]` with `original[-1] := seed`.
///
/// Arithmetic is wrapping `i32` so the operation is information-
/// preserving for *any* `i32` input. The patent does not specify a
/// signed/unsigned interpretation for the delta domain; using
/// wrapping `i32` keeps the transform bijective regardless.
pub fn differential_encode_in_place(seed: i32, q: &mut [i32]) {
    let mut prev = Wrapping(seed);
    for slot in q.iter_mut() {
        let cur = Wrapping(*slot);
        *slot = (cur - prev).0;
        prev = cur;
    }
}

/// Encode an absolute quantization-matrix sequence as deltas into a
/// fresh `Vec`. Equivalent to copying the input and calling
/// [`differential_encode_in_place`] on the copy.
pub fn differential_encode(seed: i32, q: &[i32]) -> Vec<i32> {
    let mut out = q.to_vec();
    differential_encode_in_place(seed, &mut out);
    out
}

/// Invert [`differential_encode_in_place`] in place. Given a slice of
/// deltas, replaces each entry with the absolute reconstructed value
/// using the supplied `seed` as the leading anchor.
pub fn differential_decode_in_place(seed: i32, deltas: &mut [i32]) {
    let mut prev = Wrapping(seed);
    for slot in deltas.iter_mut() {
        let cur = Wrapping(*slot) + prev;
        *slot = cur.0;
        prev = cur;
    }
}

/// Invert [`differential_encode`] into a fresh `Vec`. Equivalent to
/// copying the input and calling [`differential_decode_in_place`] on
/// the copy.
pub fn differential_decode(seed: i32, deltas: &[i32]) -> Vec<i32> {
    let mut out = deltas.to_vec();
    differential_decode_in_place(seed, &mut out);
    out
}

/// Apply the patent's "unneeded element = next needed element"
/// encoder-side substitution in place over a quantized matrix slice.
///
/// `needed[i] == true` marks element `i` as a matrix slot whose value
/// matters at the decoder; `false` marks an unneeded slot. After the
/// call every unneeded slot has been overwritten with the value of the
/// next needed slot to its right, so a subsequent
/// [`differential_encode_in_place`] pass emits a zero delta at each
/// substituted position — the patent-disclosed efficiency outcome.
///
/// Returns the number of slots actually substituted.
///
/// # Behaviour at the right edge
///
/// If a trailing run of unneeded slots has no needed slot to its
/// right (the entire tail is unneeded), the slots are **left
/// unchanged** and the count of substituted slots does not include
/// them. The patent text frames the substitution as "set unneeded =
/// next needed", which is undefined when no next needed exists; we
/// surface the absence as a no-op rather than fabricating a fill
/// value.
///
/// # Panics
///
/// Panics if `q.len() != needed.len()`.
pub fn zero_delta_pad(q: &mut [i32], needed: &[bool]) -> usize {
    assert_eq!(
        q.len(),
        needed.len(),
        "oxideav-wma::qmatrix::zero_delta_pad: matrix and needed-mask must have equal length",
    );

    let n = q.len();
    if n == 0 {
        return 0;
    }

    // Single right-to-left pass: remember the most recent "needed"
    // value seen and propagate it leftward across unneeded slots.
    let mut substituted = 0usize;
    let mut next_needed: Option<i32> = None;
    for i in (0..n).rev() {
        if needed[i] {
            next_needed = Some(q[i]);
        } else if let Some(v) = next_needed {
            q[i] = v;
            substituted += 1;
        }
    }
    substituted
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- Round-trip identity ----------

    #[test]
    fn empty_sequence_round_trips() {
        let mut q: [i32; 0] = [];
        differential_encode_in_place(0, &mut q);
        differential_decode_in_place(0, &mut q);
        assert_eq!(q, [] as [i32; 0]);
    }

    #[test]
    fn single_element_encodes_to_difference_against_seed() {
        let mut q = [42i32];
        differential_encode_in_place(10, &mut q);
        // 42 - 10 = 32
        assert_eq!(q, [32]);
        differential_decode_in_place(10, &mut q);
        assert_eq!(q, [42]);
    }

    #[test]
    fn encode_then_decode_recovers_input_for_arbitrary_sequences() {
        let cases: &[(i32, &[i32])] = &[
            (0, &[0, 0, 0, 0]),
            (0, &[1, 2, 3, 4, 5]),
            (5, &[5, 5, 5, 5]),
            (0, &[100, 50, 75, 25, 200]),
            (-7, &[-3, -1, 0, 2, 4]),
            (0, &[i32::MIN, i32::MAX, 0, -1, 1]),
        ];
        for (seed, q) in cases {
            let encoded = differential_encode(*seed, q);
            let decoded = differential_decode(*seed, &encoded);
            assert_eq!(decoded.as_slice(), *q, "seed={seed} q={q:?}");
        }
    }

    // ---------- Patent-disclosed delta semantics ----------

    #[test]
    fn equal_sequence_produces_zero_deltas_after_seed_match() {
        // Per the patent: "unneeded weighting factors may be set equal
        // to the next needed one so the differential coder emits a
        // zero delta." Equal-valued runs collapse to zeros.
        let q = vec![7i32; 6];
        let deltas = differential_encode(7, &q);
        assert_eq!(deltas, vec![0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn monotone_sequence_produces_constant_deltas() {
        // 1, 3, 5, 7, 9 with seed 1 → deltas: 0, 2, 2, 2, 2.
        let q = vec![1i32, 3, 5, 7, 9];
        let deltas = differential_encode(1, &q);
        assert_eq!(deltas, vec![0, 2, 2, 2, 2]);
    }

    #[test]
    fn decreasing_sequence_produces_negative_deltas() {
        // 10, 7, 4, 1 with seed 10 → 0, -3, -3, -3.
        let q = vec![10i32, 7, 4, 1];
        let deltas = differential_encode(10, &q);
        assert_eq!(deltas, vec![0, -3, -3, -3]);
    }

    #[test]
    fn encode_does_not_panic_on_i32_extremes() {
        // Wrapping arithmetic absorbs the i32::MIN / MAX boundary.
        let q = vec![i32::MIN, i32::MAX];
        let deltas = differential_encode(0, &q);
        // Both deltas are well-defined under wrapping arithmetic.
        let decoded = differential_decode(0, &deltas);
        assert_eq!(decoded, q);
    }

    // ---------- zero_delta_pad ----------

    #[test]
    fn zero_delta_pad_overwrites_unneeded_with_next_needed() {
        // [needed, unneeded, unneeded, needed, unneeded, needed]
        // [10,     ?,        ?,        20,     ?,        30    ]
        // After substitution:
        // [10,     20,       20,       20,     30,       30    ]
        let mut q = [10i32, 99, 99, 20, 99, 30];
        let needed = [true, false, false, true, false, true];
        let n = zero_delta_pad(&mut q, &needed);
        assert_eq!(q, [10, 20, 20, 20, 30, 30]);
        assert_eq!(n, 3);
    }

    #[test]
    fn zero_delta_pad_substituted_slots_emit_zero_delta_under_diff_encode() {
        // After zero_delta_pad, encoding the result with the matching
        // seed must emit zeros at the unneeded positions — that is the
        // patent-disclosed point of the substitution.
        let mut q = [10i32, 99, 99, 20, 99, 30];
        let needed = [true, false, false, true, false, true];
        zero_delta_pad(&mut q, &needed);
        let deltas = differential_encode(10, &q);
        // Position 0: 10 - 10 = 0 (seed match)
        // Position 1: 20 - 10 = 10
        // Position 2: 20 - 20 = 0 (substituted)
        // Position 3: 20 - 20 = 0 (substituted)
        // Position 4: 30 - 20 = 10
        // Position 5: 30 - 30 = 0 (substituted)
        assert_eq!(deltas, vec![0, 10, 0, 0, 10, 0]);
    }

    #[test]
    fn zero_delta_pad_leaves_trailing_unneeded_run_untouched() {
        // Trailing run with no next-needed → no fill value defined.
        let mut q = [10i32, 20, 99, 99];
        let needed = [true, true, false, false];
        let n = zero_delta_pad(&mut q, &needed);
        // The two trailing unneeded slots stay at 99.
        assert_eq!(q, [10, 20, 99, 99]);
        assert_eq!(n, 0);
    }

    #[test]
    fn zero_delta_pad_on_all_needed_is_noop() {
        let mut q = [1i32, 2, 3, 4];
        let needed = [true, true, true, true];
        let n = zero_delta_pad(&mut q, &needed);
        assert_eq!(q, [1, 2, 3, 4]);
        assert_eq!(n, 0);
    }

    #[test]
    fn zero_delta_pad_on_empty_returns_zero() {
        let mut q: [i32; 0] = [];
        let needed: [bool; 0] = [];
        assert_eq!(zero_delta_pad(&mut q, &needed), 0);
    }

    #[test]
    #[should_panic(expected = "matrix and needed-mask must have equal length")]
    fn zero_delta_pad_panics_on_length_mismatch() {
        let mut q = [0i32; 3];
        let needed = [true, false];
        let _ = zero_delta_pad(&mut q, &needed);
    }

    // ---------- Cross-helper: in-place == fresh-Vec ----------

    #[test]
    fn in_place_matches_fresh_vec_helper() {
        let q = vec![3i32, 1, 4, 1, 5, 9, 2, 6, 5, 3];
        let seed = 7;
        let mut in_place = q.clone();
        differential_encode_in_place(seed, &mut in_place);
        let fresh = differential_encode(seed, &q);
        assert_eq!(in_place, fresh);

        let mut in_place = fresh.clone();
        differential_decode_in_place(seed, &mut in_place);
        let fresh = differential_decode(seed, &fresh);
        assert_eq!(in_place, fresh);
        assert_eq!(in_place, q);
    }
}

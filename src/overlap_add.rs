//! WMA decoder-side overlap-add (overlapper/adder) carrier.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §3 lifts the
//! patent-disclosed reconstruction stage that closes out the inverse
//! transform pipeline. The load-bearing citations:
//!
//! > Reconstruction at the decoder is an inverse transform followed by
//! > an **overlap-add (overlapper/adder)** stage.
//! >   — [PATENT US7,383,180 — decoder FIG.6]
//!
//! > The MLT is, in DSP terms, the same transform commonly called the
//! > **MDCT** (oddly-stacked TDAC cosine-modulated filter bank with 50%
//! > overlap and 2M-length windowing over M-length blocks).
//! >   — [PATENT US6,029,126 / US6,240,380 — MLT defined as the
//! >     oddly-stacked TDAC filter bank, basis = windowed DCT-IV,
//! >     FIG.7]
//!
//! > Malvar's patents define the analysis/synthesis windows `ha(n)`,
//! > `hs(n)` and the biorthogonal generalization (**MLBT / NMLBT**)
//! > where analysis and synthesis windows may differ to raise stopband
//! > attenuation.
//! >   — [PATENT US6,240,380 — Eqns.1–2, window params; NMLBT element
//! >     510, FIG.5]
//!
//! ## What this module models
//!
//! The patent disclosure fixes three load-bearing properties for the
//! reconstruction-side overlap-add:
//!
//! 1. **The post-window inverse-transform output is `2M` samples long
//!    for an `M`-sample transform block.** This is the standard MDCT /
//!    MLT framing the Malvar patents define explicitly (US6,029,126 /
//!    US6,240,380) and that US7,383,180 inherits at the decoder.
//! 2. **Adjacent blocks overlap by `M` samples (50 %).** Per the
//!    Malvar definition of the oddly-stacked TDAC filter bank, the
//!    right half of block `n` and the left half of block `n+1` share
//!    the same `M` reconstructed time-domain positions and are summed
//!    sample-by-sample to produce the final `M` output samples.
//! 3. **The overlapper/adder is stateful between calls.** US7,383,180
//!    FIG.6 draws the overlap-add as a stage that buffers the previous
//!    block's tail across calls; without that memory the very first
//!    block's left half would have nothing to add to and the last
//!    block's right half would never reach the output.
//!
//! The typed primitive in this module captures all three: a per-instance
//! [`BlockSize`] fixes `M` (and therefore `2M`), an internal `tail`
//! buffer carries the previous block's right half across calls, and
//! [`OverlapAdd::step`] enforces the `2M`-input-length contract before
//! producing `M` output samples.
//!
//! ## Composition with the (future) MLT inverse transform
//!
//! This module accepts an already-post-windowed `2M`-sample inverse-MLT
//! output block. It does **not** apply the synthesis window itself; the
//! window choice (sine, MLBT, NMLBT) is a separate patent-disclosed
//! decision (§3 of the trace doc) whose typed carrier is **[GAP]** until
//! a future round stages the analysis/synthesis window pair as its own
//! primitive. By taking the post-windowed block as input the carrier
//! commits only to the patent's overlap-add semantics, not to a window
//! shape.
//!
//! Likewise, the inverse MLT itself is not yet implemented — this is
//! the *adder* downstream of it. A complete decoder pipeline will:
//!
//! 1. dequantize coefficients via [`crate::invquant`],
//! 2. run an inverse MLT to produce a `2M`-sample time-domain block,
//! 3. multiply by the synthesis window `hs(n)`,
//! 4. feed the result into [`OverlapAdd::step`] to emit `M` samples.
//!
//! Stages 2 and 3 are not staged in the patents-only trace at the
//! pseudocode level (the MLT/MDCT formulas are public DSP, but the
//! patent does not pin the synthesis-window coefficients for WMA v1/v2;
//! that is **[GAP]**). This module covers exactly stage 4.
//!
//! ## What is NOT in this module
//!
//! * **The inverse MLT / MDCT itself.** §3 names the transform but the
//!   forward / inverse formulas are general DSP; a future round will
//!   stage the MLT as its own primitive.
//! * **The synthesis window `hs(n)`.** The Malvar patents define the
//!   window-pair structure (analysis vs synthesis, NMLBT
//!   biorthogonality) but the patents do not fix the WMA v1/v2
//!   window coefficients. The carrier here accepts a post-windowed
//!   block so the window choice is upstream.
//! * **Block-size switching transitions.** The patent's variable
//!   `{256, 512, 1024, 2048, 4096}` set (§2) means adjacent blocks may
//!   have different sizes; the overlap-add at a size transition needs
//!   "transition windows" whose shape is **[GAP]** at the patent
//!   level. This module enforces a single [`BlockSize`] per instance
//!   and exposes [`OverlapAdd::tail_len`] so a future transition-aware
//!   wrapper can replay the boundary explicitly.

use crate::block::BlockSize;

/// Stateful overlap-add (overlapper/adder) stage for the MLT inverse
/// transform output blocks, per US7,383,180 decoder FIG.6.
///
/// One instance covers a single channel. The instance is parameterised
/// by its [`BlockSize`] `M`, which fixes both the expected input length
/// (`2M`) and the produced output length (`M`).
///
/// The internal `tail` buffer is `M` samples long and stores the right
/// half of the most recently fed input block. Per the patent's
/// 50 %-overlap arrangement (US6,029,126 / US6,240,380), the left half
/// of the *next* input block will be summed into this tail to produce
/// the next output frame.
#[derive(Debug, Clone)]
pub struct OverlapAdd {
    block_size: BlockSize,
    /// Right-half tail saved from the previous [`OverlapAdd::step`]
    /// call. Length is always `M` (= [`BlockSize::samples`]); zeroed at
    /// construction so the very first call emits the first block's
    /// left half unchanged, as the patent's overlapper/adder does for
    /// the leading edge of a stream.
    tail: Vec<f64>,
}

impl OverlapAdd {
    /// Construct a fresh overlap-add carrier for the given block size.
    ///
    /// The tail buffer starts zeroed, modelling the leading-edge
    /// behaviour of the patent's overlapper/adder: the first block has
    /// no predecessor to overlap with, so its left half is emitted
    /// unchanged.
    pub fn new(block_size: BlockSize) -> Self {
        Self {
            block_size,
            tail: vec![0.0; block_size.samples() as usize],
        }
    }

    /// Block size `M` for this instance. The expected input length is
    /// `2 * M`; the produced output length is `M`.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.block_size
    }

    /// `M`, the per-call output length and the tail-buffer length.
    /// Equivalent to `self.block_size().samples() as usize`.
    #[inline]
    pub fn output_len(&self) -> usize {
        self.block_size.samples() as usize
    }

    /// `2M`, the expected input length per [`OverlapAdd::step`] call.
    #[inline]
    pub fn input_len(&self) -> usize {
        2 * self.output_len()
    }

    /// `M`, the length of the carried-across tail buffer.
    ///
    /// Exposed so a transition-aware wrapper can inspect the boundary
    /// when adjacent blocks are different patent-disclosed sizes
    /// (§2). For a uniform-block-size stream this is identical to
    /// [`OverlapAdd::output_len`].
    #[inline]
    pub fn tail_len(&self) -> usize {
        self.tail.len()
    }

    /// Read-only access to the carried-across tail. Length equals
    /// [`OverlapAdd::tail_len`].
    ///
    /// The patent's overlapper/adder buffers the previous block's
    /// right half across calls; this accessor exposes that state for
    /// inspection (e.g. tests that verify the overlap arithmetic
    /// without driving a full sequence).
    #[inline]
    pub fn tail(&self) -> &[f64] {
        &self.tail
    }

    /// Reset the carried-across tail to all-zeros, returning the
    /// carrier to its as-constructed state. Equivalent to dropping the
    /// instance and constructing a fresh one with the same block size,
    /// but without reallocating the tail buffer.
    ///
    /// Useful at a discontinuity (seek, decoder flush) where the
    /// "previous block" is not the stream's true predecessor and
    /// summing it in would carry old samples forward.
    pub fn reset(&mut self) {
        self.tail.fill(0.0);
    }

    /// Feed one post-windowed `2M`-sample inverse-MLT block and emit
    /// `M` time-domain samples per the patent's overlap-add rule.
    ///
    /// The arithmetic mirrors the standard MDCT/MLT decoder:
    ///
    /// ```text
    /// out[k]      = tail_prev[k] + input[k]            for k in 0..M
    /// tail_new[k] = input[M + k]                       for k in 0..M
    /// ```
    ///
    /// Returns the produced output as a `Vec<f64>` of length `M`.
    /// Returns [`InvalidInputLen`] if `input.len() != 2 * M` — the
    /// patent fixes the post-windowed inverse-MLT block to `2M` samples
    /// (US6,029,126 / US6,240,380), so any other length is malformed
    /// at this stage.
    pub fn step(&mut self, input: &[f64]) -> Result<Vec<f64>, InvalidInputLen> {
        let m = self.output_len();
        let expected = 2 * m;
        if input.len() != expected {
            return Err(InvalidInputLen {
                expected,
                got: input.len(),
            });
        }
        let mut output = Vec::with_capacity(m);
        for (k, tail_k) in self.tail.iter().enumerate() {
            output.push(tail_k + input[k]);
        }
        // Save the right half of the input as the new tail.
        self.tail.copy_from_slice(&input[m..]);
        Ok(output)
    }

    /// Drain the overlap-add carrier's pending tail as the final
    /// trailing-edge output.
    ///
    /// The patent's overlap-add is symmetric at the trailing edge:
    /// after the last input block, the right-half tail still holds
    /// `M` samples that have not been emitted. Calling `flush` returns
    /// those samples (a clone of [`OverlapAdd::tail`]) and zeroes the
    /// internal tail so subsequent [`OverlapAdd::step`] calls behave
    /// as if the carrier were freshly constructed.
    ///
    /// A decoder that processes a finite stream calls
    /// [`OverlapAdd::step`] for every received block and then calls
    /// [`OverlapAdd::flush`] once to retrieve the remaining samples.
    pub fn flush(&mut self) -> Vec<f64> {
        let out = self.tail.clone();
        self.reset();
        out
    }
}

/// Construction-time rejection for [`OverlapAdd::step`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InvalidInputLen {
    /// Length the carrier requires (`2 * M`).
    pub expected: usize,
    /// Length the caller actually supplied.
    pub got: usize,
}

impl core::fmt::Display for InvalidInputLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-wma: overlap-add input must be exactly 2*M samples \
             (expected {}, got {})",
            self.expected, self.got,
        )
    }
}

impl std::error::Error for InvalidInputLen {}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- construction + accessors ----------

    #[test]
    fn new_starts_with_zeroed_tail_for_each_block_size() {
        for bs in BlockSize::ALL {
            let oa = OverlapAdd::new(bs);
            assert_eq!(oa.block_size(), bs);
            assert_eq!(oa.tail_len(), bs.samples() as usize);
            assert!(oa.tail().iter().all(|&s| s == 0.0));
        }
    }

    #[test]
    fn output_len_equals_block_samples() {
        for bs in BlockSize::ALL {
            let oa = OverlapAdd::new(bs);
            assert_eq!(oa.output_len(), bs.samples() as usize);
        }
    }

    #[test]
    fn input_len_is_twice_output_len() {
        for bs in BlockSize::ALL {
            let oa = OverlapAdd::new(bs);
            assert_eq!(oa.input_len(), 2 * oa.output_len());
        }
    }

    #[test]
    fn tail_len_equals_output_len() {
        for bs in BlockSize::ALL {
            let oa = OverlapAdd::new(bs);
            assert_eq!(oa.tail_len(), oa.output_len());
        }
    }

    // ---------- step: input-length contract ----------

    #[test]
    fn step_rejects_too_short_input() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let input = vec![0.0_f64; 511]; // one short of 2*256
        let err = oa.step(&input).unwrap_err();
        assert_eq!(
            err,
            InvalidInputLen {
                expected: 512,
                got: 511,
            }
        );
    }

    #[test]
    fn step_rejects_too_long_input() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let input = vec![0.0_f64; 513];
        let err = oa.step(&input).unwrap_err();
        assert_eq!(
            err,
            InvalidInputLen {
                expected: 512,
                got: 513,
            }
        );
    }

    #[test]
    fn step_rejects_empty_input() {
        let mut oa = OverlapAdd::new(BlockSize::S512);
        let err = oa.step(&[]).unwrap_err();
        assert_eq!(
            err,
            InvalidInputLen {
                expected: 1024,
                got: 0,
            }
        );
    }

    #[test]
    fn step_rejects_input_sized_for_a_different_block() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let input = vec![0.0_f64; 2 * 512]; // sized for S512
        let err = oa.step(&input).unwrap_err();
        assert_eq!(
            err,
            InvalidInputLen {
                expected: 512,
                got: 1024,
            }
        );
    }

    #[test]
    fn step_does_not_mutate_tail_when_input_length_is_wrong() {
        // Patent's overlap-add is a stateful stage; a malformed input
        // must not corrupt the carried-across tail.
        let mut oa = OverlapAdd::new(BlockSize::S256);
        // Prime the tail via a well-formed call first.
        let mut prime = vec![0.0_f64; 512];
        for k in 0..256 {
            prime[256 + k] = (k as f64) + 1.0;
        }
        oa.step(&prime).unwrap();
        let tail_before: Vec<f64> = oa.tail().to_vec();
        // Now feed a malformed input.
        let bad = vec![99.0_f64; 100];
        assert!(oa.step(&bad).is_err());
        // Tail should be unchanged.
        assert_eq!(oa.tail(), tail_before.as_slice());
    }

    // ---------- step: arithmetic ----------

    #[test]
    fn first_step_emits_left_half_unchanged() {
        // With a zeroed tail (leading edge), the first M output samples
        // equal the input's left half exactly.
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let mut input = vec![0.0_f64; 512];
        for (k, slot) in input.iter_mut().enumerate().take(256) {
            *slot = (k as f64) * 0.5;
        }
        let out = oa.step(&input).unwrap();
        assert_eq!(out.len(), 256);
        for (k, &o) in out.iter().enumerate() {
            assert_eq!(o, input[k]);
        }
    }

    #[test]
    fn first_step_saves_right_half_as_tail() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let mut input = vec![0.0_f64; 512];
        for k in 0..256 {
            input[256 + k] = (k as f64) + 100.0;
        }
        oa.step(&input).unwrap();
        assert_eq!(oa.tail().len(), 256);
        for k in 0..256 {
            assert_eq!(oa.tail()[k], input[256 + k]);
        }
    }

    #[test]
    fn second_step_sums_prev_tail_with_current_left_half() {
        // The defining property of the patent's overlapper/adder:
        // out[k] = prev_tail[k] + curr_input[k] for the M-sample
        // overlap region.
        let mut oa = OverlapAdd::new(BlockSize::S256);
        // First block: right half is all 7.0
        let mut block1 = vec![0.0_f64; 512];
        for k in 0..256 {
            block1[256 + k] = 7.0;
        }
        oa.step(&block1).unwrap();
        assert!(oa.tail().iter().all(|&s| s == 7.0));
        // Second block: left half is all 3.0
        let mut block2 = vec![0.0_f64; 512];
        for slot in block2.iter_mut().take(256) {
            *slot = 3.0;
        }
        let out = oa.step(&block2).unwrap();
        assert!(out.iter().all(|&s| s == 10.0));
    }

    #[test]
    fn three_block_sequence_overlap_chain() {
        // Verify the full chain across three blocks: each output
        // depends only on (prev right half) + (curr left half).
        let mut oa = OverlapAdd::new(BlockSize::S256);
        // Build three blocks where block i has left=i*2 and right=i*2+1.
        let make = |left: f64, right: f64| -> Vec<f64> {
            let mut b = vec![0.0_f64; 512];
            for k in 0..256 {
                b[k] = left;
                b[256 + k] = right;
            }
            b
        };
        let b0 = make(0.0, 1.0);
        let b1 = make(2.0, 3.0);
        let b2 = make(4.0, 5.0);

        let o0 = oa.step(&b0).unwrap();
        // First emission: 0 (tail) + 0 (left of b0) = 0
        assert!(o0.iter().all(|&s| s == 0.0));

        let o1 = oa.step(&b1).unwrap();
        // Second: 1 (right of b0) + 2 (left of b1) = 3
        assert!(o1.iter().all(|&s| s == 3.0));

        let o2 = oa.step(&b2).unwrap();
        // Third: 3 (right of b1) + 4 (left of b2) = 7
        assert!(o2.iter().all(|&s| s == 7.0));

        // Tail after three calls holds the right half of b2 (= 5).
        assert!(oa.tail().iter().all(|&s| s == 5.0));
    }

    #[test]
    fn output_length_matches_block_size_for_every_size() {
        for bs in BlockSize::ALL {
            let mut oa = OverlapAdd::new(bs);
            let m = bs.samples() as usize;
            let input = vec![1.0_f64; 2 * m];
            let out = oa.step(&input).unwrap();
            assert_eq!(out.len(), m);
        }
    }

    // ---------- reset ----------

    #[test]
    fn reset_zeroes_the_tail() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let mut input = vec![0.0_f64; 512];
        for k in 0..256 {
            input[256 + k] = 42.0;
        }
        oa.step(&input).unwrap();
        assert!(oa.tail().iter().all(|&s| s == 42.0));
        oa.reset();
        assert!(oa.tail().iter().all(|&s| s == 0.0));
    }

    #[test]
    fn step_after_reset_behaves_as_if_freshly_constructed() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        // Prime with some non-zero tail.
        let mut prime = vec![0.0_f64; 512];
        for k in 0..256 {
            prime[256 + k] = 11.0;
        }
        oa.step(&prime).unwrap();
        oa.reset();
        // Now first call should emit its left half unchanged.
        let mut input = vec![0.0_f64; 512];
        for (k, slot) in input.iter_mut().enumerate().take(256) {
            *slot = (k as f64) * 2.0;
        }
        let out = oa.step(&input).unwrap();
        for (k, &o) in out.iter().enumerate() {
            assert_eq!(o, input[k]);
        }
    }

    // ---------- flush ----------

    #[test]
    fn flush_returns_current_tail() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let mut input = vec![0.0_f64; 512];
        for k in 0..256 {
            input[256 + k] = (k as f64) * 0.25;
        }
        oa.step(&input).unwrap();
        let snapshot: Vec<f64> = oa.tail().to_vec();
        let flushed = oa.flush();
        assert_eq!(flushed, snapshot);
    }

    #[test]
    fn flush_zeroes_the_tail() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let input = vec![5.0_f64; 512];
        oa.step(&input).unwrap();
        oa.flush();
        assert!(oa.tail().iter().all(|&s| s == 0.0));
    }

    #[test]
    fn flush_on_fresh_carrier_returns_zeros() {
        let mut oa = OverlapAdd::new(BlockSize::S512);
        let flushed = oa.flush();
        assert_eq!(flushed.len(), 512);
        assert!(flushed.iter().all(|&s| s == 0.0));
    }

    #[test]
    fn step_after_flush_behaves_as_if_freshly_constructed() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let input1 = vec![9.0_f64; 512];
        oa.step(&input1).unwrap();
        let _drained = oa.flush();
        let mut input2 = vec![0.0_f64; 512];
        for (k, slot) in input2.iter_mut().enumerate().take(256) {
            *slot = (k as f64) + 1.0;
        }
        let out = oa.step(&input2).unwrap();
        // Tail was just flushed to zero, so output equals left half.
        for (k, &o) in out.iter().enumerate() {
            assert_eq!(o, input2[k]);
        }
    }

    // ---------- canonical decoder usage ----------

    #[test]
    fn end_to_end_two_blocks_plus_flush_emits_all_samples() {
        // A finite stream of two MLT blocks produces:
        //   out0 = step(block0)  -> M samples (block0 left half)
        //   out1 = step(block1)  -> M samples (block0 right + block1 left)
        //   tail = flush()       -> M samples (block1 right half)
        // Total: 3M output samples for two 2M input blocks; that
        // matches the 50%-overlap arithmetic the patent describes.
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let b0: Vec<f64> = (0..512).map(|k| k as f64).collect();
        let b1: Vec<f64> = (512..1024).map(|k| k as f64).collect();
        let mut all = Vec::new();
        all.extend(oa.step(&b0).unwrap());
        all.extend(oa.step(&b1).unwrap());
        all.extend(oa.flush());
        assert_eq!(all.len(), 3 * 256);

        // Spot-check the boundary: out1[k] = b0[256+k] + b1[k]
        for k in 0..256 {
            let expected = b0[256 + k] + b1[k];
            assert_eq!(all[256 + k], expected, "k={k}");
        }
        // And the flushed tail equals b1's right half.
        for k in 0..256 {
            assert_eq!(all[2 * 256 + k], b1[256 + k], "k={k}");
        }
    }

    // ---------- error display ----------

    #[test]
    fn invalid_input_len_display_mentions_expected_and_got() {
        let err = InvalidInputLen {
            expected: 2048,
            got: 1000,
        };
        let s = format!("{err}");
        assert!(s.contains("2048"));
        assert!(s.contains("1000"));
        assert!(s.contains("overlap-add"));
    }

    // ---------- clone independence ----------

    #[test]
    fn clone_is_independent_state() {
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let mut input = vec![0.0_f64; 512];
        for k in 0..256 {
            input[256 + k] = 4.0;
        }
        oa.step(&input).unwrap();
        let mut clone = oa.clone();
        // Mutate the clone; original tail must be untouched.
        clone.reset();
        assert!(clone.tail().iter().all(|&s| s == 0.0));
        assert!(oa.tail().iter().all(|&s| s == 4.0));
    }
}

//! WMA decoder-side **stereo** time-domain reconstruction stage.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §8 draws the decoder
//! pipeline (Thumpudi-180 FIG.6) and fixes the order of its trailing
//! stages:
//!
//! > ```text
//! >  → inverse MLT                         (US6,029,126/380)
//! >  → overlap-add                         (US7,383,180 overlapper/adder)
//! >  → [inverse sum-difference / multi-channel post-process]  (US7,502,743)
//! >  → PCM
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (decoder
//! >     pipeline, Thumpudi-180 FIG.6)
//!
//! The §5 two-channel transform that the post-process inverts is the
//! patent-disclosed sum/difference (mid/side) coding for WMA Standard:
//!
//! > For stereo, WMA7 can code the two channels as **sum and difference
//! > channels** — the sum being the channel average and the difference
//! > being half the channel difference (i.e. mid/side).
//! >   — [PATENT US7,930,171 — WMA7 sum/difference]
//! >   — [PATENT US7,502,743 — prior-art sum/difference baseline]
//!
//! and the position of the inverse transform in the chain — *after* the
//! inverse MLT / overlap-add, in the time domain — is the position the
//! §8 FIG.6 diagram draws for the `[inverse sum-difference /
//! multi-channel post-process]` box.
//!
//! ## Scope of this module
//!
//! This module is the **assembler** that wires the per-channel
//! reconstruction stage ([`crate::synthesis::Synthesis`], Round 16) and
//! the §5 inverse sum/difference transform
//! ([`crate::stereo::inverse_in_place`]) into the single stereo decoder
//! tail the patent's FIG.6 draws. For one block of two dequantized
//! coefficient channels it produces two reconstructed time-domain
//! channels by:
//!
//! 1. running each channel's coefficients through its own
//!    [`crate::synthesis::Synthesis`] stage (inverse MLT → synthesis
//!    window `hs(n)` → overlap-add), giving two `M`-sample time-domain
//!    channels, then
//! 2. applying the §8 `[inverse sum-difference]` post-process **only
//!    when the block was coded jointly** — i.e. when the per-block
//!    [`crate::channel_decision::ChannelMode`] is
//!    [`ChannelMode::SumDifference`] the two reconstructed channels are
//!    the *mid* and *side* time-domain signals and
//!    [`crate::stereo::inverse_in_place`] folds them back to *left* and
//!    *right*; when the mode is [`ChannelMode::Independent`] the two
//!    channels are already left/right and the post-process is skipped,
//!    exactly as the FIG.6 box is bypassed for independently-coded
//!    channels.
//!
//! Each channel keeps its **own** overlap-add carry, so the two
//! [`crate::synthesis::Synthesis`] stages are independent across the
//! block sequence; only the per-block sum/difference fold couples them,
//! and only when the mode says so.
//!
//! ## What this module deliberately does NOT do
//!
//! * **No new arithmetic.** The transform/window/overlap math lives in
//!   [`crate::mlt`] / [`crate::window`] / [`crate::overlap_add`] (via
//!   [`crate::synthesis::Synthesis`]) and the sum/difference fold lives
//!   in [`crate::stereo`]; this stage only sequences them in the
//!   patent-fixed FIG.6 order. It is the stereo analogue of the
//!   single-channel [`crate::synthesis::Synthesis`] assembler.
//! * **No flag parsing.** The per-block [`ChannelMode`] is
//!   caller-supplied. §5 marks the v1/v2 channel-mode flag layout
//!   `[GAP]`; this module never fabricates that bit. The §5 *decision*
//!   that an encoder makes is modelled separately by
//!   [`crate::channel_decision::OpenLoopDecision`]; here the chosen mode
//!   is an input.
//! * **No block-size-transition handling.** As with
//!   [`crate::synthesis::Synthesis`], the carrier is one uniform
//!   [`crate::block::BlockSize`] `M`; adjacent blocks of different
//!   patent-disclosed sizes (§2) need transition handling whose shape is
//!   `[GAP]`.

use crate::block::BlockSize;
use crate::channel_decision::ChannelMode;
use crate::synthesis::{InvalidCoeffLen, MismatchedBlockSize, Synthesis};
use crate::window::WindowPair;

/// Stateful decoder-side stereo time-domain reconstruction stage for one
/// uniform [`BlockSize`] `M`, per §8 of the patent trace (Thumpudi-180
/// FIG.6: inverse MLT → overlap-add → `[inverse sum-difference]` → PCM).
///
/// Owns two independent per-channel [`Synthesis`] stages — one for each
/// of the two channels — and applies the §5 inverse sum/difference
/// transform as the FIG.6 post-process, gated by the per-block
/// [`ChannelMode`]. One [`StereoSynthesis::block`] call consumes two
/// `M`-coefficient channels and emits two `M`-sample reconstructed
/// channels, carrying each channel's overlap-add tail across calls.
#[derive(Debug, Clone)]
pub struct StereoSynthesis {
    left: Synthesis,
    right: Synthesis,
}

/// Two reconstructed time-domain channels, each `M` samples long.
///
/// After a [`StereoSynthesis::block`] call these are the final decoded
/// **left** and **right** PCM channels for the block, with any §8
/// inverse sum/difference post-process already applied.
#[derive(Debug, Clone, PartialEq)]
pub struct StereoBlock {
    /// Reconstructed left channel, `M` samples.
    pub left: Vec<f64>,
    /// Reconstructed right channel, `M` samples.
    pub right: Vec<f64>,
}

impl StereoSynthesis {
    /// Construct the stereo reconstruction stage for a given block size
    /// and analysis/synthesis [`WindowPair`].
    ///
    /// Both channels share the same `M` and the same window shape — the
    /// patent's per-block window/block-size decision is one decision for
    /// the tile, so both channels of a stereo block transform with the
    /// same window (§2 tile note). The pair is cloned into each
    /// per-channel [`Synthesis`].
    ///
    /// Returns [`MismatchedBlockSize`] if the window pair's block size
    /// differs from `block_size`, propagated from [`Synthesis::new`].
    pub fn new(
        block_size: BlockSize,
        window_pair: WindowPair,
    ) -> Result<Self, MismatchedBlockSize> {
        let left = Synthesis::new(block_size, window_pair.clone())?;
        let right = Synthesis::new(block_size, window_pair)?;
        Ok(Self { left, right })
    }

    /// Block size `M` for this stage (shared by both channels).
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.left.block_size()
    }

    /// `M`, the per-call per-channel input length (dequantized
    /// coefficient count) and per-channel output length (reconstructed
    /// sample count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.left.block_len()
    }

    /// Reconstruct one stereo block.
    ///
    /// `ch0` and `ch1` are the two dequantized coefficient channels for
    /// the block, each exactly `M` long. `mode` is the per-block
    /// [`ChannelMode`] the (caller-parsed) bitstream selected:
    ///
    /// * [`ChannelMode::Independent`] — `ch0`/`ch1` are already the
    ///   left/right channels; each is reconstructed through its own
    ///   [`Synthesis`] and returned as-is (the FIG.6 sum/difference box
    ///   is bypassed).
    /// * [`ChannelMode::SumDifference`] — `ch0`/`ch1` are the *mid* /
    ///   *side* channels; each is reconstructed through its own
    ///   [`Synthesis`] in the time domain, then the §8 inverse
    ///   sum/difference post-process folds the two reconstructed signals
    ///   back to left/right via [`crate::stereo::inverse_in_place`].
    ///
    /// The per-channel overlap-add carry advances for **both** channels
    /// on every call regardless of `mode`, because the inverse MLT /
    /// overlap-add runs per channel before the post-process — the
    /// FIG.6 chain places the sum/difference fold *after* the
    /// overlap-add, so the carriers see the per-channel (mid/side or
    /// left/right) signals, never the folded output.
    ///
    /// Returns [`InvalidCoeffLen`] if either channel's length differs
    /// from `M`, propagated from [`Synthesis::block`].
    pub fn block(
        &mut self,
        ch0: &[f64],
        ch1: &[f64],
        mode: ChannelMode,
    ) -> Result<StereoBlock, InvalidCoeffLen> {
        // Per-channel reconstruction (inverse MLT -> window -> overlap-add).
        // Run ch0 first so a length error on it surfaces before ch1's
        // carry is advanced, keeping the two carriers in lock-step.
        let mut a = self.left.block(ch0)?;
        let mut b = self.right.block(ch1)?;

        // §8 [inverse sum-difference] post-process, only when the block
        // was coded jointly. For Independent, a/b are already L/R.
        match mode {
            ChannelMode::Independent => {}
            ChannelMode::SumDifference => {
                // a = mid, b = side -> (left, right) in place.
                crate::stereo::inverse_in_place(&mut a, &mut b);
            }
        }

        Ok(StereoBlock { left: a, right: b })
    }

    /// Drain the trailing-edge tail after the last block, returning the
    /// final reconstructed samples for both channels and zeroing both
    /// carries.
    ///
    /// `mode` selects whether the drained tail is post-processed: the
    /// trailing block's coding mode governs the fold of the final
    /// overlap-add tail exactly as [`StereoSynthesis::block`] governs it
    /// for a coded block. A decoder calls [`StereoSynthesis::block`] for
    /// every received block, then [`StereoSynthesis::flush`] once with
    /// the last block's mode to retrieve the remaining tail.
    pub fn flush(&mut self, mode: ChannelMode) -> StereoBlock {
        let mut a = self.left.flush();
        let mut b = self.right.flush();
        if mode == ChannelMode::SumDifference {
            crate::stereo::inverse_in_place(&mut a, &mut b);
        }
        StereoBlock { left: a, right: b }
    }

    /// Clear both channels' overlap-add carry at a discontinuity (seek /
    /// decoder flush), so the next [`StereoSynthesis::block`] behaves as
    /// if this stage were freshly constructed.
    pub fn reset(&mut self) {
        self.left.reset();
        self.right.reset();
    }

    /// Read-only access to the two per-channel overlap-add tails (each
    /// the previous block's right half, length `M`). Exposed so tests
    /// and transition-aware callers can inspect the boundary state.
    /// These are the *per-channel* (mid/side or left/right) carries —
    /// the FIG.6 fold runs after the carriers, so the tails are never
    /// the post-processed output.
    #[inline]
    pub fn tails(&self) -> (&[f64], &[f64]) {
        (self.left.tail(), self.right.tail())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mlt::Mlt;
    use crate::overlap_add::OverlapAdd;

    fn stage(bs: BlockSize) -> StereoSynthesis {
        StereoSynthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap()
    }

    #[test]
    fn new_accepts_matching_block_size() {
        let s = stage(BlockSize::S256);
        assert_eq!(s.block_size(), BlockSize::S256);
        assert_eq!(s.block_len(), 256);
    }

    #[test]
    fn new_rejects_mismatched_window_pair() {
        let pair = WindowPair::orthogonal_sine(BlockSize::S512);
        let err = StereoSynthesis::new(BlockSize::S256, pair).unwrap_err();
        assert_eq!(err.stage, BlockSize::S256);
        assert_eq!(err.window_pair, BlockSize::S512);
    }

    #[test]
    fn block_rejects_wrong_left_len() {
        let mut s = stage(BlockSize::S256);
        let err = s
            .block(&[0.0; 255], &[0.0; 256], ChannelMode::Independent)
            .unwrap_err();
        assert_eq!(err.expected, 256);
        assert_eq!(err.got, 255);
    }

    #[test]
    fn block_rejects_wrong_right_len() {
        let mut s = stage(BlockSize::S256);
        let err = s
            .block(&[0.0; 256], &[0.0; 255], ChannelMode::Independent)
            .unwrap_err();
        assert_eq!(err.expected, 256);
        assert_eq!(err.got, 255);
    }

    #[test]
    fn block_emits_m_samples_per_channel() {
        let mut s = stage(BlockSize::S256);
        let out = s
            .block(&[1.0; 256], &[0.5; 256], ChannelMode::Independent)
            .unwrap();
        assert_eq!(out.left.len(), 256);
        assert_eq!(out.right.len(), 256);
    }

    #[test]
    fn tails_start_zeroed_and_length_m() {
        let s = stage(BlockSize::S256);
        let (tl, tr) = s.tails();
        assert_eq!(tl.len(), 256);
        assert_eq!(tr.len(), 256);
        assert!(tl.iter().all(|&x| x == 0.0));
        assert!(tr.iter().all(|&x| x == 0.0));
    }

    /// Independent mode must equal running two bare `Synthesis` stages by
    /// hand: this pins that the assembler adds no arithmetic and bypasses
    /// the sum/difference box when the mode is independent.
    #[test]
    fn independent_equals_two_bare_synthesis_stages() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let c0: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.013).sin()).collect();
        let c1: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.021).cos()).collect();

        let mut sa = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let mut sb = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let la = sa.block(&c0).unwrap();
        let rb = sb.block(&c1).unwrap();

        let mut s = stage(bs);
        let out = s.block(&c0, &c1, ChannelMode::Independent).unwrap();
        assert_eq!(out.left, la);
        assert_eq!(out.right, rb);
    }

    /// Sum/difference mode must equal: reconstruct each channel with a
    /// bare `Synthesis`, then fold with `stereo::inverse_in_place`. This
    /// pins that the post-process runs *after* the per-channel
    /// reconstruction, in the FIG.6 order.
    #[test]
    fn sum_difference_equals_synthesis_then_inverse_fold() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let mid_c: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.013).sin()).collect();
        let side_c: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.021).cos()).collect();

        let mut sa = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let mut sb = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let mut mid_t = sa.block(&mid_c).unwrap();
        let mut side_t = sb.block(&side_c).unwrap();
        crate::stereo::inverse_in_place(&mut mid_t, &mut side_t);

        let mut s = stage(bs);
        let out = s
            .block(&mid_c, &side_c, ChannelMode::SumDifference)
            .unwrap();
        assert_eq!(out.left, mid_t);
        assert_eq!(out.right, side_t);
    }

    /// The per-channel overlap-add carries advance identically whether
    /// the block is coded independent or joint — the fold runs after the
    /// carriers, so the tails see the per-channel signals either way.
    #[test]
    fn carries_advance_regardless_of_mode() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let c0: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.01).cos()).collect();
        let c1: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.02).sin()).collect();

        // Hand-wired per-channel carriers (no fold touches them).
        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);
        let mut oa0 = OverlapAdd::new(bs);
        let mut oa1 = OverlapAdd::new(bs);
        let _ = oa0
            .step(
                &pair
                    .synthesis()
                    .windowed(&mlt.inverse(&c0).unwrap())
                    .unwrap(),
            )
            .unwrap();
        let _ = oa1
            .step(
                &pair
                    .synthesis()
                    .windowed(&mlt.inverse(&c1).unwrap())
                    .unwrap(),
            )
            .unwrap();
        let tail0 = oa0.tail().to_vec();
        let tail1 = oa1.tail().to_vec();

        // Joint mode: carriers must still equal the per-channel ones.
        let mut s = stage(bs);
        let _ = s.block(&c0, &c1, ChannelMode::SumDifference).unwrap();
        let (tl, tr) = s.tails();
        assert_eq!(tl, tail0.as_slice());
        assert_eq!(tr, tail1.as_slice());
    }

    /// Round-trip through the analysis side: encode a correlated stereo
    /// pair as mid/side, push the mid/side coefficients through this
    /// stage in sum/difference mode, and recover the original channels in
    /// the steady-state interior. End-to-end proof the post-process
    /// inverts the §5 transform in the time domain.
    #[test]
    fn steady_state_sum_difference_round_trip() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);

        // Constant L and R over a 2M frame; their mid/side are constants.
        let l_val = 0.8_f64;
        let r_val = 0.2_f64;
        let mid_val = (l_val + r_val) * 0.5;
        let side_val = (l_val - r_val) * 0.5;

        let analysis_coeffs = |val: f64| {
            let mut frame = vec![val; 2 * m];
            pair.analysis().apply_in_place(&mut frame).unwrap();
            mlt.forward(&frame).unwrap()
        };
        let mid_c = analysis_coeffs(mid_val);
        let side_c = analysis_coeffs(side_val);

        let mut s = stage(bs);
        // Prime to steady state.
        let _ = s
            .block(&mid_c, &side_c, ChannelMode::SumDifference)
            .unwrap();
        let _ = s
            .block(&mid_c, &side_c, ChannelMode::SumDifference)
            .unwrap();
        let steady = s
            .block(&mid_c, &side_c, ChannelMode::SumDifference)
            .unwrap();
        for &x in &steady.left {
            assert!((x - l_val).abs() < 1e-9, "left {x} != {l_val}");
        }
        for &x in &steady.right {
            assert!((x - r_val).abs() < 1e-9, "right {x} != {r_val}");
        }
    }

    #[test]
    fn flush_drains_both_tails_and_zeroes_carries() {
        let mut s = stage(BlockSize::S256);
        let _ = s
            .block(&[0.5; 256], &[0.25; 256], ChannelMode::Independent)
            .unwrap();
        let (tl, tr) = s.tails();
        let (pl, pr) = (tl.to_vec(), tr.to_vec());
        let flushed = s.flush(ChannelMode::Independent);
        assert_eq!(flushed.left, pl);
        assert_eq!(flushed.right, pr);
        let (tl, tr) = s.tails();
        assert!(tl.iter().all(|&x| x == 0.0));
        assert!(tr.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn flush_folds_when_mode_is_joint() {
        let mut s = stage(BlockSize::S256);
        let _ = s
            .block(&[0.5; 256], &[0.25; 256], ChannelMode::SumDifference)
            .unwrap();
        let (tl, tr) = s.tails();
        let mut pl = tl.to_vec();
        let mut pr = tr.to_vec();
        crate::stereo::inverse_in_place(&mut pl, &mut pr);
        let flushed = s.flush(ChannelMode::SumDifference);
        assert_eq!(flushed.left, pl);
        assert_eq!(flushed.right, pr);
    }

    #[test]
    fn reset_clears_both_carries() {
        let mut s = stage(BlockSize::S256);
        let _ = s
            .block(&[0.5; 256], &[0.25; 256], ChannelMode::Independent)
            .unwrap();
        let (tl, tr) = s.tails();
        assert!(tl.iter().any(|&x| x != 0.0) || tr.iter().any(|&x| x != 0.0));
        s.reset();
        let (tl, tr) = s.tails();
        assert!(tl.iter().all(|&x| x == 0.0));
        assert!(tr.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn works_across_all_block_sizes() {
        for bs in BlockSize::ALL {
            let m = bs.samples() as usize;
            let mut s = stage(bs);
            let out = s
                .block(&vec![0.25; m], &vec![0.1; m], ChannelMode::SumDifference)
                .unwrap();
            assert_eq!(out.left.len(), m);
            assert_eq!(out.right.len(), m);
            assert_eq!(s.flush(ChannelMode::SumDifference).left.len(), m);
        }
    }
}

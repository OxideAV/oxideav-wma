//! Frame driver — runs a frame's sequence of blocks through the §8
//! per-channel / stereo decoder chains and concatenates the PCM.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §2 establishes the
//! coding hierarchy: the encoder "partitions a frame of audio samples
//! into overlapping sub-frame blocks" (Chen-171 FIG.3 / Thumpudi-180
//! module 520), and the in-repo wiki snapshot names the same nesting —
//! "blocks → frames (one or more blocks) → superframes (one or more
//! frames)". The §8 decoder pipeline (Thumpudi-180 FIG.6) is drawn
//! per-block; a *frame* is the unit of one-or-more consecutive blocks,
//! and the decoder reconstructs the frame's PCM by running each block
//! through the FIG.6 chain in order and concatenating the time-domain
//! output (the overlap-add carrier already threads the inter-block
//! continuity, per [`crate::overlap_add`]).
//!
//! ## Scope of this module
//!
//! This module is the **frame-loop assembler** one layer above
//! [`crate::decode::ChannelDecoder`] (mono) and
//! [`crate::stereo_decode::StereoDecoder`] (stereo). Given a frame's
//! ordered list of already-demuxed per-block parameter sets, it drives
//! the underlying decoder block-by-block and concatenates the per-block
//! PCM into one frame PCM vector. It adds **no arithmetic of its own** —
//! it is pure sequencing of the per-block decoders the earlier rounds
//! already built and tested.
//!
//! ## What is NOT in this module
//!
//! * **The bitstream reader / DEMUX.** Per §6 the codeword tables and
//!   the per-process parameter demux (US7,885,819 FIG.7) are `[GAP]`;
//!   this driver consumes the **already-demuxed, already-decoded**
//!   per-block parameter sets, exactly as the per-block decoders do. The
//!   number of blocks in the frame and each block's parameters are
//!   therefore caller-supplied, never derived from a fabricated layout.
//! * **Block-size-transition frames.** The underlying
//!   [`crate::decode::ChannelDecoder`] carries one uniform
//!   [`crate::block::BlockSize`] `M` across calls (its overlap-add
//!   carrier is sized for one `M`); a frame whose blocks switch size
//!   needs window-transition handling whose shape is `[GAP]` per §2/§3,
//!   the same deferral [`crate::decode`] and [`crate::synthesis`]
//!   record. This driver therefore runs a **uniform-block-size** frame
//!   — every block at the decoder's `M`, the non-variable-block-length
//!   case the wiki's `frame_length = 1 << frame_length_bits` describes.
//!   A frame whose `variable_block_length` flag is set, and whose blocks
//!   draw different sizes from the `{256…4096}` set, is out of scope
//!   until the transition handling is staged.
//! * **The superframe / packet boundary.** §2 marks the
//!   superframe/packet byte layout and the bit-reservoir field widths
//!   `[GAP]`; this driver decodes one frame's already-delimited blocks
//!   and does not parse frame/superframe boundaries.

use crate::decode::{ChannelDecoder, DecodeError};
use crate::runlevel::RunLevelPair;
use crate::stereo_decode::{StereoDecodeError, StereoDecoder};
use crate::stereo_synthesis::StereoBlock;
use crate::ChannelMode;

/// One mono block's already-demuxed, already-decoded per-block
/// parameters — the owned analogue of the borrowed argument triple
/// [`ChannelDecoder::block`] takes.
///
/// The driver borrows each field per block, so the noise `patterns`
/// are owned `Vec<Vec<f64>>` here (one entry per band, lockstep with
/// the band plan) and reborrowed as `&[&[f64]]` at decode time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BlockParams {
    /// Level-mode head symbols for the spectral stage (length `split`).
    pub levels: Vec<i32>,
    /// Run-level `(R, L)` pairs for the high-frequency tail.
    pub pairs: Vec<RunLevelPair>,
    /// One noise pattern per band, lockstep with the band plan; entries
    /// for non-noise bands are ignored (an empty inner `Vec` is fine).
    pub patterns: Vec<Vec<f64>>,
}

impl BlockParams {
    /// Construct a block-parameter set from its three components.
    pub fn new(levels: Vec<i32>, pairs: Vec<RunLevelPair>, patterns: Vec<Vec<f64>>) -> Self {
        BlockParams {
            levels,
            pairs,
            patterns,
        }
    }
}

/// Mono frame driver — runs a frame's ordered blocks through one
/// [`ChannelDecoder`] and concatenates the per-block PCM.
///
/// The driver owns the per-channel decoder (so its overlap-add carrier
/// persists across the frame's blocks and across frames) and is the
/// §2/§8 frame-loop above it. Per [`FrameDecoder::decode_frame`] the
/// blocks are decoded in order and their `M`-sample outputs
/// concatenated into one `n_blocks * M` PCM vector.
#[derive(Debug, Clone)]
pub struct FrameDecoder {
    decoder: ChannelDecoder,
}

impl FrameDecoder {
    /// Wrap a per-channel [`ChannelDecoder`] in the frame loop.
    pub fn new(decoder: ChannelDecoder) -> Self {
        FrameDecoder { decoder }
    }

    /// `M`, the per-block reconstructed-sample length of the underlying
    /// decoder.
    #[inline]
    pub fn block_len(&self) -> usize {
        self.decoder.block_len()
    }

    /// Borrow the underlying per-channel decoder.
    #[inline]
    pub const fn decoder(&self) -> &ChannelDecoder {
        &self.decoder
    }

    /// Decode a frame of `blocks.len()` consecutive blocks into one
    /// concatenated PCM vector of `blocks.len() * M` samples.
    ///
    /// The blocks are decoded in order; the overlap-add carrier threads
    /// inter-block continuity exactly as a sequence of
    /// [`ChannelDecoder::block`] calls would. An empty `blocks` slice
    /// yields an empty PCM vector (a frame of zero blocks). This call
    /// does **not** flush the trailing overlap-add tail — that belongs
    /// at the *stream* end, not every frame boundary, so the carrier
    /// stays live for the next frame; use [`FrameDecoder::flush`] once at
    /// stream end.
    ///
    /// # Errors
    ///
    /// Returns the first block's [`DecodeError`] (with no further blocks
    /// decoded), naming the failing stage. On error the overlap-add
    /// carry is left at whatever the last successful block produced.
    pub fn decode_frame(&mut self, blocks: &[BlockParams]) -> Result<Vec<f64>, DecodeError> {
        let m = self.decoder.block_len();
        let mut pcm = Vec::with_capacity(blocks.len() * m);
        for block in blocks {
            let patterns: Vec<&[f64]> = block.patterns.iter().map(Vec::as_slice).collect();
            let out = self.decoder.block(&block.levels, &block.pairs, &patterns)?;
            pcm.extend_from_slice(&out);
        }
        Ok(pcm)
    }

    /// Drain the trailing-edge overlap-add tail at stream end, returning
    /// the final `M` reconstructed samples. Delegates to
    /// [`ChannelDecoder::flush`].
    pub fn flush(&mut self) -> Vec<f64> {
        self.decoder.flush()
    }

    /// Clear the overlap-add carry at a discontinuity (seek / decoder
    /// flush). Delegates to [`ChannelDecoder::reset`].
    pub fn reset(&mut self) {
        self.decoder.reset();
    }
}

/// One stereo block's already-demuxed per-block parameters for both
/// channels plus the per-block channel-coding mode — the owned analogue
/// of the borrowed arguments [`StereoDecoder::block`] takes.
///
/// The `mode` (the §5 independent-vs-sum/difference decision) is
/// per-block: its v1/v2 flag layout is `[GAP]`, so it is a caller input,
/// never fabricated.
#[derive(Debug, Clone, PartialEq)]
pub struct StereoBlockParams {
    /// Channel 0's per-block parameters (mid, under sum/difference).
    pub ch0: BlockParams,
    /// Channel 1's per-block parameters (side, under sum/difference).
    pub ch1: BlockParams,
    /// The per-block channel-coding mode (§5; flag layout `[GAP]`).
    pub mode: ChannelMode,
}

impl StereoBlockParams {
    /// Construct a stereo block-parameter set from both channels' params
    /// and the per-block channel-coding mode.
    pub fn new(ch0: BlockParams, ch1: BlockParams, mode: ChannelMode) -> Self {
        StereoBlockParams { ch0, ch1, mode }
    }
}

/// Stereo frame driver — runs a frame's ordered stereo blocks through
/// one [`StereoDecoder`] and concatenates the per-block L/R PCM.
///
/// The stereo analogue of [`FrameDecoder`]: it owns the two-channel
/// decoder (both overlap-add carriers persist across the frame) and
/// concatenates each block's [`StereoBlock`] into one frame
/// [`StereoBlock`] of `n_blocks * M` samples per channel.
#[derive(Debug, Clone)]
pub struct StereoFrameDecoder {
    decoder: StereoDecoder,
}

impl StereoFrameDecoder {
    /// Wrap a two-channel [`StereoDecoder`] in the frame loop.
    pub fn new(decoder: StereoDecoder) -> Self {
        StereoFrameDecoder { decoder }
    }

    /// `M`, the per-block reconstructed-sample length per channel.
    #[inline]
    pub fn block_len(&self) -> usize {
        self.decoder.block_len()
    }

    /// Borrow the underlying two-channel decoder.
    #[inline]
    pub const fn decoder(&self) -> &StereoDecoder {
        &self.decoder
    }

    /// Decode a frame of `blocks.len()` consecutive stereo blocks into
    /// one [`StereoBlock`] whose `left` / `right` each hold
    /// `blocks.len() * M` concatenated samples.
    ///
    /// Each block runs through the full two-channel §8 chain, including
    /// the per-block [`ChannelMode`]-gated inverse sum/difference fold.
    /// Like [`FrameDecoder::decode_frame`] this does not flush the
    /// trailing tail — use [`StereoFrameDecoder::flush`] once at stream
    /// end with the last block's mode.
    ///
    /// An empty `blocks` slice yields a `StereoBlock` with two empty
    /// channels.
    ///
    /// # Errors
    ///
    /// Returns the first failing block's [`StereoDecodeError`] (naming
    /// the channel and wrapping its [`DecodeError`]); no further blocks
    /// are decoded.
    pub fn decode_frame(
        &mut self,
        blocks: &[StereoBlockParams],
    ) -> Result<StereoBlock, StereoDecodeError> {
        let m = self.decoder.block_len();
        let mut left = Vec::with_capacity(blocks.len() * m);
        let mut right = Vec::with_capacity(blocks.len() * m);
        for block in blocks {
            let pat0: Vec<&[f64]> = block.ch0.patterns.iter().map(Vec::as_slice).collect();
            let pat1: Vec<&[f64]> = block.ch1.patterns.iter().map(Vec::as_slice).collect();
            let out = self.decoder.block(
                &block.ch0.levels,
                &block.ch0.pairs,
                &pat0,
                &block.ch1.levels,
                &block.ch1.pairs,
                &pat1,
                block.mode,
            )?;
            left.extend_from_slice(&out.left);
            right.extend_from_slice(&out.right);
        }
        Ok(StereoBlock { left, right })
    }

    /// Drain both channels' trailing-edge overlap-add tails at stream
    /// end, folding the tail when the last block was joint. Delegates to
    /// [`StereoDecoder::flush`].
    pub fn flush(&mut self, mode: ChannelMode) -> StereoBlock {
        self.decoder.flush(mode)
    }

    /// Clear both overlap-add carries at a discontinuity. Delegates to
    /// [`StereoDecoder::reset`].
    pub fn reset(&mut self) {
        self.decoder.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bands::BandPlan;
    use crate::block::BlockSize;
    use crate::dequant::DequantStage;
    use crate::entropy_mode::Partition;
    use crate::noisefill::NoiseFiller;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::spectral::SpectralDecode;
    use crate::step_size::OverallStepSize;
    use crate::synthesis::Synthesis;
    use crate::window::WindowPair;

    // Build a minimal but complete single-channel decoder for BlockSize
    // S256 (M = 256): one Coded band spanning the whole block, unit
    // weights and step, a level-mode-only partition (no run-level tail).
    fn channel_decoder() -> ChannelDecoder {
        let m = BlockSize::S256.samples() as usize; // 256
        let layout =
            QuantBandLayout::new(vec![QuantBand::new(0, m as u16, 0).unwrap()], m).unwrap();
        let spectral = SpectralDecode::new(Partition::new(m as u32, m as u32, false).unwrap());
        let step = OverallStepSize::new(1.0).unwrap();
        let dequant = DequantStage::new(BlockSize::S256, &layout, &[1.0], step).unwrap();
        let plan = BandPlan::new(vec![crate::bands::BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(
            BlockSize::S256,
            WindowPair::orthogonal_sine(BlockSize::S256),
        )
        .unwrap();
        ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap()
    }

    // A level-mode-only block: `m` already-signed integer level symbols,
    // no run-level pairs, and one (ignored) pattern entry for the single
    // Coded band — the noise filler requires `patterns.len()` to equal
    // the band count in lockstep, even for bands it does not fill.
    fn level_block(m: usize, fill: i32) -> BlockParams {
        BlockParams::new(vec![fill; m], Vec::new(), vec![Vec::new()])
    }

    // ---------- BlockParams plumbing ----------

    #[test]
    fn block_params_new_round_trips_fields() {
        let p = BlockParams::new(vec![1, 2, 3], Vec::new(), vec![vec![0.5]]);
        assert_eq!(p.levels, vec![1, 2, 3]);
        assert!(p.pairs.is_empty());
        assert_eq!(p.patterns, vec![vec![0.5]]);
    }

    #[test]
    fn block_params_default_is_empty() {
        let p = BlockParams::default();
        assert!(p.levels.is_empty());
        assert!(p.pairs.is_empty());
        assert!(p.patterns.is_empty());
    }

    // ---------- mono frame driver ----------

    #[test]
    fn frame_block_len_matches_decoder() {
        let fd = FrameDecoder::new(channel_decoder());
        assert_eq!(fd.block_len(), 256);
        assert_eq!(fd.decoder().block_len(), 256);
    }

    #[test]
    fn empty_frame_yields_empty_pcm() {
        let mut fd = FrameDecoder::new(channel_decoder());
        let pcm = fd.decode_frame(&[]).unwrap();
        assert!(pcm.is_empty());
    }

    #[test]
    fn single_block_frame_length_is_m() {
        let mut fd = FrameDecoder::new(channel_decoder());
        let pcm = fd.decode_frame(&[level_block(256, 0)]).unwrap();
        assert_eq!(pcm.len(), 256);
    }

    #[test]
    fn multi_block_frame_concatenates_m_per_block() {
        let mut fd = FrameDecoder::new(channel_decoder());
        let blocks = vec![
            level_block(256, 1),
            level_block(256, -1),
            level_block(256, 0),
        ];
        let pcm = fd.decode_frame(&blocks).unwrap();
        assert_eq!(pcm.len(), 3 * 256);
        assert!(pcm.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn frame_loop_equals_manual_per_block_chain() {
        // The driver must produce exactly what hand-running the same
        // ChannelDecoder block-by-block produces (no arithmetic of its
        // own).
        let blocks = vec![level_block(256, 2), level_block(256, -3)];

        let mut driven = FrameDecoder::new(channel_decoder());
        let via_driver = driven.decode_frame(&blocks).unwrap();

        let mut manual = channel_decoder();
        let mut expected = Vec::new();
        for b in &blocks {
            let pats: Vec<&[f64]> = b.patterns.iter().map(Vec::as_slice).collect();
            let out = manual.block(&b.levels, &b.pairs, &pats).unwrap();
            expected.extend_from_slice(&out);
        }
        assert_eq!(via_driver, expected);
    }

    #[test]
    fn frame_carry_persists_across_decode_frame_calls() {
        // Two separate decode_frame calls must equal one call over the
        // concatenated block list — the overlap-add carry is not reset
        // at the frame boundary.
        let f1 = vec![level_block(256, 1), level_block(256, 2)];
        let f2 = vec![level_block(256, 3)];

        let mut split = FrameDecoder::new(channel_decoder());
        let mut out = split.decode_frame(&f1).unwrap();
        out.extend(split.decode_frame(&f2).unwrap());

        let mut whole = FrameDecoder::new(channel_decoder());
        let all: Vec<BlockParams> = f1.iter().chain(f2.iter()).cloned().collect();
        let out_whole = whole.decode_frame(&all).unwrap();

        assert_eq!(out, out_whole);
    }

    #[test]
    fn reset_clears_carry_between_frames() {
        let mut fd = FrameDecoder::new(channel_decoder());
        let _ = fd.decode_frame(&[level_block(256, 5)]).unwrap();
        fd.reset();
        // After reset the next frame must equal a fresh decoder's first
        // frame.
        let after = fd.decode_frame(&[level_block(256, 7)]).unwrap();
        let mut fresh = FrameDecoder::new(channel_decoder());
        let first = fresh.decode_frame(&[level_block(256, 7)]).unwrap();
        assert_eq!(after, first);
    }

    #[test]
    fn flush_drains_trailing_tail() {
        let mut fd = FrameDecoder::new(channel_decoder());
        let _ = fd.decode_frame(&[level_block(256, 1)]).unwrap();
        let tail = fd.flush();
        assert_eq!(tail.len(), 256);
    }

    // ---------- stereo frame driver ----------

    fn stereo_decoder() -> StereoDecoder {
        StereoDecoder::new(channel_decoder(), channel_decoder()).unwrap()
    }

    fn stereo_block(m: usize, l: i32, r: i32, mode: ChannelMode) -> StereoBlockParams {
        StereoBlockParams::new(level_block(m, l), level_block(m, r), mode)
    }

    #[test]
    fn stereo_block_params_round_trip() {
        let p = stereo_block(256, 1, 2, ChannelMode::Independent);
        assert_eq!(p.ch0.levels, vec![1; 256]);
        assert_eq!(p.ch1.levels, vec![2; 256]);
        assert_eq!(p.mode, ChannelMode::Independent);
    }

    #[test]
    fn stereo_empty_frame_yields_two_empty_channels() {
        let mut sf = StereoFrameDecoder::new(stereo_decoder());
        let out = sf.decode_frame(&[]).unwrap();
        assert!(out.left.is_empty());
        assert!(out.right.is_empty());
    }

    #[test]
    fn stereo_multi_block_concatenates_per_channel() {
        let mut sf = StereoFrameDecoder::new(stereo_decoder());
        let blocks = vec![
            stereo_block(256, 1, 2, ChannelMode::Independent),
            stereo_block(256, -1, -2, ChannelMode::Independent),
        ];
        let out = sf.decode_frame(&blocks).unwrap();
        assert_eq!(out.left.len(), 2 * 256);
        assert_eq!(out.right.len(), 2 * 256);
        assert!(out.left.iter().all(|v| v.is_finite()));
        assert!(out.right.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn stereo_frame_loop_equals_manual_chain() {
        let blocks = vec![
            stereo_block(256, 2, 3, ChannelMode::SumDifference),
            stereo_block(256, -1, 4, ChannelMode::Independent),
        ];

        let mut driven = StereoFrameDecoder::new(stereo_decoder());
        let via_driver = driven.decode_frame(&blocks).unwrap();

        let mut manual = stereo_decoder();
        let mut el = Vec::new();
        let mut er = Vec::new();
        for b in &blocks {
            let p0: Vec<&[f64]> = b.ch0.patterns.iter().map(Vec::as_slice).collect();
            let p1: Vec<&[f64]> = b.ch1.patterns.iter().map(Vec::as_slice).collect();
            let out = manual
                .block(
                    &b.ch0.levels,
                    &b.ch0.pairs,
                    &p0,
                    &b.ch1.levels,
                    &b.ch1.pairs,
                    &p1,
                    b.mode,
                )
                .unwrap();
            el.extend_from_slice(&out.left);
            er.extend_from_slice(&out.right);
        }
        assert_eq!(via_driver.left, el);
        assert_eq!(via_driver.right, er);
    }

    #[test]
    fn stereo_sum_difference_differs_from_independent() {
        // The per-block mode is honoured: a joint block folds, an
        // independent block does not, so the two outputs differ for a
        // mid/side pair that is not already L/R.
        let joint = vec![stereo_block(256, 4, 2, ChannelMode::SumDifference)];
        let indep = vec![stereo_block(256, 4, 2, ChannelMode::Independent)];

        let mut a = StereoFrameDecoder::new(stereo_decoder());
        let mut b = StereoFrameDecoder::new(stereo_decoder());
        let ja = a.decode_frame(&joint).unwrap();
        let ib = b.decode_frame(&indep).unwrap();
        assert_ne!(ja.left, ib.left);
    }

    #[test]
    fn stereo_reset_and_flush() {
        let mut sf = StereoFrameDecoder::new(stereo_decoder());
        let _ = sf
            .decode_frame(&[stereo_block(256, 1, 1, ChannelMode::Independent)])
            .unwrap();
        let tail = sf.flush(ChannelMode::Independent);
        assert_eq!(tail.left.len(), 256);
        assert_eq!(tail.right.len(), 256);
        sf.reset();
        // Fresh after reset.
        let after = sf
            .decode_frame(&[stereo_block(256, 9, 9, ChannelMode::Independent)])
            .unwrap();
        let mut fresh = StereoFrameDecoder::new(stereo_decoder());
        let first = fresh
            .decode_frame(&[stereo_block(256, 9, 9, ChannelMode::Independent)])
            .unwrap();
        assert_eq!(after.left, first.left);
        assert_eq!(after.right, first.right);
    }

    #[test]
    fn stereo_block_len_matches_decoder() {
        let sf = StereoFrameDecoder::new(stereo_decoder());
        assert_eq!(sf.block_len(), 256);
        assert_eq!(sf.decoder().block_len(), 256);
    }
}

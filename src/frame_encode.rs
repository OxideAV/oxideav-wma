//! WMA frame-level encoder loop.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §2 fixes the
//! block→frame grouping this module drives forward, exactly as
//! [`crate::frame`] drives it in the decode direction:
//!
//! > The encoder partitions a **frame** of audio samples into
//! > overlapping sub-frame blocks (windows) of time-varying size.
//! >   — [PATENT US7,930,171 — generalized encoder, FIG.3]
//! >   — [PATENT US7,383,180 — partitioner/tile configurer module 520]
//!
//! and the wiki snapshot's nesting orientation: *blocks → frames (one
//! or more blocks) → superframes* **[WIKI]**.
//!
//! ## Scope of this module
//!
//! The forward mirror of [`crate::frame`]: [`FrameEncoder`] wraps a
//! [`crate::encode::ChannelEncoder`] (mono) and [`StereoFrameEncoder`]
//! wraps a [`crate::stereo_encode::StereoEncoder`] (stereo).
//! `encode_frame` partitions a frame's PCM into consecutive
//! `M`-sample blocks, runs each through the underlying §8 chain, and
//! collects the per-block symbol sets — the exact
//! [`crate::frame::BlockParams`] / [`crate::frame::StereoBlockParams`]
//! lists the paired [`crate::frame::FrameDecoder`] /
//! [`crate::frame::StereoFrameDecoder`] consume (via the
//! `into_block_params` bridges). The 50%-overlap frame buffer threads
//! across frames — `encode_frame` does **not** flush, so a stream's
//! frames encode contiguously; `flush` emits the single trailing
//! block once at stream end, and `reset` clears the buffers at a
//! discontinuity.
//!
//! ## What is NOT in this module
//!
//! * **The per-frame block count / size plan.** This driver runs
//!   uniform-block-size frames (the non-variable-block-length case
//!   `frame_length = 1 << frame_length_bits` describes); the
//!   variable-block-length plan derived from the upper `flags2` bits
//!   is `[GAP]` at the bit level (§1), and block-size transitions are
//!   the same `[GAP]` [`crate::frame`] records.
//! * **The superframe / packet byte layout.** `[GAP]` per §2/§9; the
//!   emitted frames are typed symbol lists, not bytes.

use crate::channel_decision::ChannelMode;
use crate::encode::{ChannelEncoder, EncodeError, EncodedBlock};
use crate::stereo_encode::{StereoEncodeError, StereoEncodedBlock, StereoEncoder};

/// The offered frame PCM length is not a whole number of `M`-sample
/// blocks (or the two channels of a stereo frame disagree).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidFrameLen {
    /// The PCM length is not a multiple of the block length `M`.
    NotBlockAligned {
        /// The block length `M` the frame must be a multiple of.
        block_len: usize,
        /// The PCM sample count actually offered (per channel).
        got: usize,
    },
    /// A stereo frame's two channels have different sample counts.
    ChannelLenMismatch {
        /// Left-channel sample count.
        left: usize,
        /// Right-channel sample count.
        right: usize,
    },
    /// A stereo frame's per-block mode list does not have one entry
    /// per block.
    ModeCountMismatch {
        /// Block count the PCM length implies.
        expected_blocks: usize,
        /// Mode entries actually supplied.
        got_modes: usize,
    },
}

impl core::fmt::Display for InvalidFrameLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidFrameLen::NotBlockAligned { block_len, got } => write!(
                f,
                "oxideav-wma::frame_encode: frame length {got} is not a multiple of the block length {block_len}",
            ),
            InvalidFrameLen::ChannelLenMismatch { left, right } => write!(
                f,
                "oxideav-wma::frame_encode: stereo frame channels disagree ({left} left samples vs {right} right)",
            ),
            InvalidFrameLen::ModeCountMismatch {
                expected_blocks,
                got_modes,
            } => write!(
                f,
                "oxideav-wma::frame_encode: stereo frame has {expected_blocks} blocks but {got_modes} channel-mode entries",
            ),
        }
    }
}

impl std::error::Error for InvalidFrameLen {}

/// Failure modes for [`FrameEncoder::encode_frame`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrameEncodeError {
    /// The frame PCM failed the length contract.
    FrameLen(InvalidFrameLen),
    /// A block failed inside the per-block chain; earlier blocks'
    /// buffers have advanced (the §8 chain is stateful), later blocks
    /// were not encoded.
    Block(EncodeError),
}

impl core::fmt::Display for FrameEncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FrameEncodeError::FrameLen(e) => write!(f, "{e}"),
            FrameEncodeError::Block(e) => write!(f, "oxideav-wma::frame_encode: {e}"),
        }
    }
}

impl std::error::Error for FrameEncodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            FrameEncodeError::FrameLen(e) => Some(e),
            FrameEncodeError::Block(e) => Some(e),
        }
    }
}

/// Failure modes for [`StereoFrameEncoder::encode_frame`].
#[derive(Debug, Clone, PartialEq)]
pub enum StereoFrameEncodeError {
    /// The frame PCM failed the length contract.
    FrameLen(InvalidFrameLen),
    /// A block failed inside the two-channel chain.
    Block(StereoEncodeError),
}

impl core::fmt::Display for StereoFrameEncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            StereoFrameEncodeError::FrameLen(e) => write!(f, "{e}"),
            StereoFrameEncodeError::Block(e) => write!(f, "oxideav-wma::frame_encode: {e}"),
        }
    }
}

impl std::error::Error for StereoFrameEncodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            StereoFrameEncodeError::FrameLen(e) => Some(e),
            StereoFrameEncodeError::Block(e) => Some(e),
        }
    }
}

/// Mono frame-encoder driver — partitions a frame's PCM into
/// `M`-sample blocks and runs each through one
/// [`crate::encode::ChannelEncoder`], per the §2 frame loop.
#[derive(Debug, Clone)]
pub struct FrameEncoder {
    encoder: ChannelEncoder,
}

impl FrameEncoder {
    /// Wrap a single-channel [`ChannelEncoder`] in the frame loop.
    pub fn new(encoder: ChannelEncoder) -> Self {
        FrameEncoder { encoder }
    }

    /// `M`, the per-block fresh-sample length.
    #[inline]
    pub fn block_len(&self) -> usize {
        self.encoder.block_len()
    }

    /// Borrow the underlying per-channel encoder.
    #[inline]
    pub const fn encoder(&self) -> &ChannelEncoder {
        &self.encoder
    }

    /// Encode a frame of `pcm.len() / M` consecutive blocks into their
    /// per-block symbol sets.
    ///
    /// The 50%-overlap buffer threads across calls (no per-frame
    /// flush): a stream's frames encode contiguously, exactly as
    /// [`crate::frame::FrameDecoder::decode_frame`] decodes them. An
    /// empty `pcm` yields an empty list.
    ///
    /// # Errors
    ///
    /// * [`FrameEncodeError::FrameLen`] if `pcm.len()` is not a
    ///   multiple of `M` (nothing encoded, no state advanced).
    /// * [`FrameEncodeError::Block`] on the first failing block.
    pub fn encode_frame(&mut self, pcm: &[f64]) -> Result<Vec<EncodedBlock>, FrameEncodeError> {
        let m = self.block_len();
        if pcm.len() % m != 0 {
            return Err(FrameEncodeError::FrameLen(
                InvalidFrameLen::NotBlockAligned {
                    block_len: m,
                    got: pcm.len(),
                },
            ));
        }
        let mut blocks = Vec::with_capacity(pcm.len() / m);
        for chunk in pcm.chunks_exact(m) {
            blocks.push(self.encoder.block(chunk).map_err(FrameEncodeError::Block)?);
        }
        Ok(blocks)
    }

    /// Emit the single trailing block that carries the stream's last
    /// `M` real samples ([`ChannelEncoder::flush`]), once at stream
    /// end.
    pub fn flush(&mut self) -> Result<EncodedBlock, FrameEncodeError> {
        self.encoder.flush().map_err(FrameEncodeError::Block)
    }

    /// Clear the frame buffer at a discontinuity. Delegates to
    /// [`ChannelEncoder::reset`].
    pub fn reset(&mut self) {
        self.encoder.reset();
    }
}

/// Stereo frame-encoder driver — the stereo analogue of
/// [`FrameEncoder`], wrapping one
/// [`crate::stereo_encode::StereoEncoder`].
#[derive(Debug, Clone)]
pub struct StereoFrameEncoder {
    encoder: StereoEncoder,
}

impl StereoFrameEncoder {
    /// Wrap a two-channel [`StereoEncoder`] in the frame loop.
    pub fn new(encoder: StereoEncoder) -> Self {
        StereoFrameEncoder { encoder }
    }

    /// `M`, the per-block fresh-sample length per channel.
    #[inline]
    pub fn block_len(&self) -> usize {
        self.encoder.block_len()
    }

    /// Borrow the underlying two-channel encoder.
    #[inline]
    pub const fn encoder(&self) -> &StereoEncoder {
        &self.encoder
    }

    /// Encode a stereo frame of `left.len() / M` consecutive blocks,
    /// each folded and coded under its caller-supplied per-block
    /// [`ChannelMode`] (`modes[t]` for block `t` — the §5 decision's
    /// v1/v2 flag layout is `[GAP]`, so the plan is an input, never
    /// fabricated; see
    /// [`crate::channel_decision::OpenLoopDecision`] for the analysis
    /// that produces one).
    ///
    /// # Errors
    ///
    /// * [`StereoFrameEncodeError::FrameLen`] if the two channels
    ///   disagree in length ([`InvalidFrameLen::ChannelLenMismatch`]),
    ///   the length is not a multiple of `M`
    ///   ([`InvalidFrameLen::NotBlockAligned`]), or `modes` does not
    ///   have one entry per block
    ///   ([`InvalidFrameLen::ModeCountMismatch`]). Nothing is encoded
    ///   and no state advances.
    /// * [`StereoFrameEncodeError::Block`] on the first failing block.
    pub fn encode_frame(
        &mut self,
        left: &[f64],
        right: &[f64],
        modes: &[ChannelMode],
    ) -> Result<Vec<StereoEncodedBlock>, StereoFrameEncodeError> {
        let m = self.block_len();
        if left.len() != right.len() {
            return Err(StereoFrameEncodeError::FrameLen(
                InvalidFrameLen::ChannelLenMismatch {
                    left: left.len(),
                    right: right.len(),
                },
            ));
        }
        if left.len() % m != 0 {
            return Err(StereoFrameEncodeError::FrameLen(
                InvalidFrameLen::NotBlockAligned {
                    block_len: m,
                    got: left.len(),
                },
            ));
        }
        if left.len() / m != modes.len() {
            return Err(StereoFrameEncodeError::FrameLen(
                InvalidFrameLen::ModeCountMismatch {
                    expected_blocks: left.len() / m,
                    got_modes: modes.len(),
                },
            ));
        }
        let mut blocks = Vec::with_capacity(modes.len());
        for (t, mode) in modes.iter().enumerate() {
            let l = &left[t * m..(t + 1) * m];
            let r = &right[t * m..(t + 1) * m];
            blocks.push(
                self.encoder
                    .block(l, r, *mode)
                    .map_err(StereoFrameEncodeError::Block)?,
            );
        }
        Ok(blocks)
    }

    /// Emit the trailing stereo block at stream end
    /// ([`StereoEncoder::flush`]); `mode` travels with it for the
    /// paired decoder's trailing block and final flush.
    pub fn flush(
        &mut self,
        mode: ChannelMode,
    ) -> Result<StereoEncodedBlock, StereoFrameEncodeError> {
        self.encoder
            .flush(mode)
            .map_err(StereoFrameEncodeError::Block)
    }

    /// Clear both frame buffers at a discontinuity. Delegates to
    /// [`StereoEncoder::reset`].
    pub fn reset(&mut self) {
        self.encoder.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::Analysis;
    use crate::bands::{BandPlan, BandPolicy};
    use crate::block::BlockSize;
    use crate::decode::ChannelDecoder;
    use crate::dequant::DequantStage;
    use crate::entropy_mode::Partition;
    use crate::frame::{FrameDecoder, StereoFrameDecoder};
    use crate::noisefill::NoiseFiller;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::quant::QuantStage;
    use crate::spectral::{SpectralDecode, SpectralEncode};
    use crate::step_size::OverallStepSize;
    use crate::stereo_decode::StereoDecoder;
    use crate::synthesis::Synthesis;
    use crate::window::WindowPair;

    const STEP: f64 = 1e-3;
    const BS: BlockSize = BlockSize::S256;
    const M: usize = 256;

    fn channel_encoder() -> ChannelEncoder {
        let layout =
            QuantBandLayout::new(vec![QuantBand::new(0, M as u16, 0).unwrap()], M).unwrap();
        let analysis = Analysis::new(BS, WindowPair::orthogonal_sine(BS)).unwrap();
        let quant =
            QuantStage::new(BS, &layout, &[1.0], OverallStepSize::new(STEP).unwrap()).unwrap();
        let spectral = SpectralEncode::new(Partition::new(M as u32, M as u32, false).unwrap());
        ChannelEncoder::new(analysis, quant, spectral).unwrap()
    }

    fn channel_decoder() -> ChannelDecoder {
        let layout =
            QuantBandLayout::new(vec![QuantBand::new(0, M as u16, 0).unwrap()], M).unwrap();
        let spectral = SpectralDecode::new(Partition::new(M as u32, M as u32, false).unwrap());
        let dequant =
            DequantStage::new(BS, &layout, &[1.0], OverallStepSize::new(STEP).unwrap()).unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(BS, WindowPair::orthogonal_sine(BS)).unwrap();
        ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap()
    }

    fn pseudo_random(len: usize, seed: u64) -> Vec<f64> {
        let mut state = seed.wrapping_mul(0x9E3779B97F4A7C15).wrapping_add(1);
        (0..len)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((state >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
            })
            .collect()
    }

    // ---------- mono driver ----------

    #[test]
    fn mono_accessors_and_empty_frame() {
        let mut fe = FrameEncoder::new(channel_encoder());
        assert_eq!(fe.block_len(), M);
        assert_eq!(fe.encoder().block_len(), M);
        assert!(fe.encode_frame(&[]).unwrap().is_empty());
    }

    #[test]
    fn mono_rejects_unaligned_frame_without_advancing() {
        let mut fe = FrameEncoder::new(channel_encoder());
        let err = fe.encode_frame(&vec![0.0; M + 1]).unwrap_err();
        assert_eq!(
            err,
            FrameEncodeError::FrameLen(InvalidFrameLen::NotBlockAligned {
                block_len: M,
                got: M + 1,
            })
        );
        assert!(fe.encoder().analysis().prev().iter().all(|&s| s == 0.0));
    }

    #[test]
    fn mono_frame_equals_manual_block_loop() {
        let x = pseudo_random(3 * M, 101);
        let mut fe = FrameEncoder::new(channel_encoder());
        let via_frame = fe.encode_frame(&x).unwrap();

        let mut enc = channel_encoder();
        for (t, block) in via_frame.iter().enumerate() {
            let manual = enc.block(&x[t * M..(t + 1) * M]).unwrap();
            assert_eq!(*block, manual, "t={t}");
        }
        assert_eq!(via_frame.len(), 3);
    }

    #[test]
    fn mono_two_frames_equal_one_concatenated_frame() {
        // The buffer is NOT reset at the frame boundary.
        let x = pseudo_random(4 * M, 102);
        let mut split_enc = FrameEncoder::new(channel_encoder());
        let mut a = split_enc.encode_frame(&x[..2 * M]).unwrap();
        a.extend(split_enc.encode_frame(&x[2 * M..]).unwrap());

        let mut whole_enc = FrameEncoder::new(channel_encoder());
        let b = whole_enc.encode_frame(&x).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn mono_frame_round_trips_through_frame_decoder() {
        let blocks = 3usize;
        let x = pseudo_random(blocks * M, 103);
        let mut fe = FrameEncoder::new(channel_encoder());
        let mut fd = FrameDecoder::new(channel_decoder());

        let mut params: Vec<_> = fe
            .encode_frame(&x)
            .unwrap()
            .into_iter()
            .map(|b| b.into_block_params(1))
            .collect();
        params.push(fe.flush().unwrap().into_block_params(1));

        let out = fd.decode_frame(&params).unwrap();
        assert_eq!(out.len(), (blocks + 1) * M);
        let tolerance = 4.0 * STEP;
        for i in 0..blocks * M {
            let err = (out[M + i] - x[i]).abs();
            assert!(err < tolerance, "i={i} err={err}");
        }
    }

    #[test]
    fn mono_reset_restores_fresh_behaviour() {
        let x = pseudo_random(M, 104);
        let mut used = FrameEncoder::new(channel_encoder());
        let _ = used.encode_frame(&pseudo_random(M, 105)).unwrap();
        used.reset();
        let mut fresh = FrameEncoder::new(channel_encoder());
        assert_eq!(
            used.encode_frame(&x).unwrap(),
            fresh.encode_frame(&x).unwrap()
        );
    }

    // ---------- stereo driver ----------

    fn stereo_encoder() -> StereoFrameEncoder {
        StereoFrameEncoder::new(StereoEncoder::new(channel_encoder(), channel_encoder()).unwrap())
    }

    fn stereo_decoder() -> StereoFrameDecoder {
        StereoFrameDecoder::new(StereoDecoder::new(channel_decoder(), channel_decoder()).unwrap())
    }

    #[test]
    fn stereo_rejects_channel_len_mismatch() {
        let mut fe = stereo_encoder();
        let err = fe
            .encode_frame(
                &vec![0.0; M],
                &vec![0.0; 2 * M],
                &[ChannelMode::Independent],
            )
            .unwrap_err();
        assert_eq!(
            err,
            StereoFrameEncodeError::FrameLen(InvalidFrameLen::ChannelLenMismatch {
                left: M,
                right: 2 * M,
            })
        );
    }

    #[test]
    fn stereo_rejects_mode_count_mismatch() {
        let mut fe = stereo_encoder();
        let err = fe
            .encode_frame(
                &vec![0.0; 2 * M],
                &vec![0.0; 2 * M],
                &[ChannelMode::Independent],
            )
            .unwrap_err();
        assert_eq!(
            err,
            StereoFrameEncodeError::FrameLen(InvalidFrameLen::ModeCountMismatch {
                expected_blocks: 2,
                got_modes: 1,
            })
        );
    }

    #[test]
    fn stereo_frame_honours_per_block_modes() {
        // Per-block mode list [Independent, SumDifference]: block 0
        // matches an independent hand encode, block 1 a folded one.
        let l = pseudo_random(2 * M, 106);
        let r = pseudo_random(2 * M, 107);
        let mut fe = stereo_encoder();
        let via_frame = fe
            .encode_frame(
                &l,
                &r,
                &[ChannelMode::Independent, ChannelMode::SumDifference],
            )
            .unwrap();

        let mut manual = StereoEncoder::new(channel_encoder(), channel_encoder()).unwrap();
        let m0 = manual
            .block(&l[..M], &r[..M], ChannelMode::Independent)
            .unwrap();
        let m1 = manual
            .block(&l[M..], &r[M..], ChannelMode::SumDifference)
            .unwrap();
        assert_eq!(via_frame, vec![m0, m1]);
    }

    #[test]
    fn stereo_frame_round_trips_through_stereo_frame_decoder() {
        let blocks = 3usize;
        let l = pseudo_random(blocks * M, 108);
        let r = pseudo_random(blocks * M, 109);
        let modes = vec![ChannelMode::SumDifference; blocks];

        let mut fe = stereo_encoder();
        let mut fd = stereo_decoder();

        let mut params: Vec<_> = fe
            .encode_frame(&l, &r, &modes)
            .unwrap()
            .into_iter()
            .map(|b| b.into_stereo_block_params(1))
            .collect();
        params.push(
            fe.flush(ChannelMode::SumDifference)
                .unwrap()
                .into_stereo_block_params(1),
        );

        let out = fd.decode_frame(&params).unwrap();
        assert_eq!(out.left.len(), (blocks + 1) * M);
        let tolerance = 4.0 * STEP;
        for i in 0..blocks * M {
            let el = (out.left[M + i] - l[i]).abs();
            let er = (out.right[M + i] - r[i]).abs();
            assert!(el < tolerance, "L i={i} err={el}");
            assert!(er < tolerance, "R i={i} err={er}");
        }
    }

    #[test]
    fn stereo_empty_frame_and_reset() {
        let mut fe = stereo_encoder();
        assert!(fe.encode_frame(&[], &[], &[]).unwrap().is_empty());
        let l = pseudo_random(M, 110);
        let r = pseudo_random(M, 111);
        let _ = fe
            .encode_frame(&l, &r, &[ChannelMode::Independent])
            .unwrap();
        fe.reset();
        let mut fresh = stereo_encoder();
        assert_eq!(
            fe.encode_frame(&l, &r, &[ChannelMode::Independent])
                .unwrap(),
            fresh
                .encode_frame(&l, &r, &[ChannelMode::Independent])
                .unwrap()
        );
    }

    // ---------- error plumbing ----------

    #[test]
    fn error_displays_and_sources() {
        let e = FrameEncodeError::FrameLen(InvalidFrameLen::NotBlockAligned {
            block_len: 256,
            got: 257,
        });
        assert!(format!("{e}").contains("257"));
        assert!(std::error::Error::source(&e).is_some());

        let e = StereoFrameEncodeError::FrameLen(InvalidFrameLen::ChannelLenMismatch {
            left: 1,
            right: 2,
        });
        assert!(format!("{e}").contains("disagree"));
        assert!(std::error::Error::source(&e).is_some());
    }
}

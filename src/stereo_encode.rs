//! WMA full two-channel encoder-block chain.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §8 draws the encoder
//! pipeline with the multi-channel pre-process **first**:
//!
//! > ```text
//! > PCM
//! >  → [optional multi-channel pre-process / sum-difference]
//! >  → partition into variable-size blocks {256,512,1024,2048,4096}
//! >  → MLT (per block)
//! >  → uniform scalar quantize (matrix weight × overall step)
//! >  → entropy code: …
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (encoder
//! >     pipeline, Thumpudi-180 FIG.5; US7,502,743 / US7,930,171
//! >     sum/difference)
//!
//! and §5 fixes the two-channel transform and its selection:
//!
//! > For stereo, WMA7 can code the two channels as **sum and
//! > difference channels** — the sum being the channel average and the
//! > difference being half the channel difference (i.e. mid/side).
//! >   — [PATENT US7,930,171 — WMA7 sum/difference]
//! >   — [PATENT US7,502,743 — prior-art sum/difference baseline]
//!
//! ## Scope of this module
//!
//! The stereo analogue of [`crate::encode::ChannelEncoder`], mirroring
//! [`crate::stereo_decode::StereoDecoder`]: [`StereoEncoder`] applies
//! the §5 forward sum/difference fold ([`crate::stereo::forward_in_place`])
//! to one block's left/right time-domain samples — **only** when the
//! caller-supplied per-block [`ChannelMode`] is
//! [`ChannelMode::SumDifference`], in its §8-fixed position *before*
//! the per-channel chains — then runs two complete
//! [`crate::encode::ChannelEncoder`] chains (analysis → quantize →
//! entropy code). The fold-before-analysis here and the decoder's
//! fold-after-overlap-add are the two ends of the same linear chain,
//! so a constant-mode stream round-trips exactly (within the
//! quantizer bound); the cross-module tests pin it against
//! [`crate::stereo_decode::StereoDecoder`] for both modes.
//!
//! ## What is NOT in this module
//!
//! * **The channel-coding decision.** The §5 open-loop
//!   independent-vs-joint analysis is
//!   [`crate::channel_decision::OpenLoopDecision`]'s job (its two
//!   thresholds are `[GAP]` encoder tuning); the per-block mode
//!   arrives here as an input, exactly as the decoder side takes it,
//!   and its v1/v2 flag layout stays `[GAP]`.
//! * **Mode-transition blending.** The §8 chain is linear, so a
//!   mid-stream mode switch reconstructs approximately (the two
//!   overlap-add carriers straddle the switch); the patents do not
//!   disclose any v1/v2 transition handling, so none is fabricated —
//!   constant-mode streams are the exact-reconstruction contract.
//! * **Everything the per-channel chain already scopes out** (tables,
//!   MUX, parameter selection — see [`crate::encode`]).

use crate::block::BlockSize;
use crate::channel_decision::ChannelMode;
use crate::encode::{ChannelEncoder, EncodeError, EncodedBlock};
use crate::frame::StereoBlockParams;
use crate::stereo_decode::{StereoAssemblyError, StereoChannel};

/// One encoded stereo block: both channels' entropy symbols plus the
/// per-block channel-coding mode — the encoder-side product whose
/// fields feed [`crate::stereo_decode::StereoDecoder::block`]
/// argument-for-argument.
#[derive(Debug, Clone, PartialEq)]
pub struct StereoEncodedBlock {
    /// Channel 0's symbols (mid, under sum/difference coding).
    pub ch0: EncodedBlock,
    /// Channel 1's symbols (side, under sum/difference coding).
    pub ch1: EncodedBlock,
    /// The per-block channel-coding mode this block was folded with
    /// (§5; the flag's v1/v2 bit layout is `[GAP]`, so the typed value
    /// travels with the block).
    pub mode: ChannelMode,
}

impl StereoEncodedBlock {
    /// Repackage as the owned stereo per-block parameter set the
    /// [`crate::frame::StereoFrameDecoder`] consumes. `band_count` is
    /// the paired decoder's per-channel band count (every band is
    /// literal-coded by this chain, so each gets the empty ignored
    /// pattern).
    pub fn into_stereo_block_params(self, band_count: usize) -> StereoBlockParams {
        StereoBlockParams::new(
            self.ch0.into_block_params(band_count),
            self.ch1.into_block_params(band_count),
            self.mode,
        )
    }
}

/// Stateful two-channel encoder-block chain for one uniform block size
/// `M`, per §8 of the patent trace: `[sum-difference pre-process]` →
/// two per-channel chains (analysis → quantize → entropy code).
///
/// The forward mirror of [`crate::stereo_decode::StereoDecoder`]. The
/// two per-channel 50%-overlap frame buffers stay independent across
/// the block sequence — the fold runs on the fresh input samples
/// before either buffer advances, so under sum/difference coding the
/// buffers carry the mid/side signals, exactly the signals the paired
/// decoder's overlap-add carriers hold.
#[derive(Debug, Clone)]
pub struct StereoEncoder {
    ch0: ChannelEncoder,
    ch1: ChannelEncoder,
}

impl StereoEncoder {
    /// Assemble two per-channel encoders into one stereo encode chain.
    ///
    /// Both must declare the same block size (the §2 per-block
    /// window/block-size decision is one decision for the tile) — the
    /// same cross-check [`crate::stereo_decode::StereoDecoder::new`]
    /// applies, reusing its [`StereoAssemblyError`] so a mirrored
    /// encoder/decoder pair fails identically.
    pub fn new(ch0: ChannelEncoder, ch1: ChannelEncoder) -> Result<Self, StereoAssemblyError> {
        let m0 = ch0.block_size();
        let m1 = ch1.block_size();
        if m0 != m1 {
            return Err(StereoAssemblyError::BlockSizeMismatch { ch0: m0, ch1: m1 });
        }
        Ok(Self { ch0, ch1 })
    }

    /// Block size `M` for this encoder (shared by both channels).
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.ch0.block_size()
    }

    /// `M`, the per-channel fresh-sample input length per call.
    #[inline]
    pub fn block_len(&self) -> usize {
        self.ch0.block_len()
    }

    /// The first (channel-0) per-channel encoder.
    #[inline]
    pub const fn ch0(&self) -> &ChannelEncoder {
        &self.ch0
    }

    /// The second (channel-1) per-channel encoder.
    #[inline]
    pub const fn ch1(&self) -> &ChannelEncoder {
        &self.ch1
    }

    /// Encode one stereo block: consume `M` fresh left samples and `M`
    /// fresh right samples, emit both channels' entropy symbols.
    ///
    /// Applies, in the §8-fixed order:
    ///
    /// 1. **`[sum-difference pre-process]`** — for
    ///    [`ChannelMode::SumDifference`] the left/right pair is folded
    ///    to mid/side via [`crate::stereo::forward_in_place`]; for
    ///    [`ChannelMode::Independent`] the box is bypassed and the
    ///    channels pass through as-is.
    /// 2. **Two per-channel chains** — channel 0 (left / mid) then
    ///    channel 1 (right / side), each running analysis → quantize →
    ///    entropy code. Channel 0 is encoded first so its error
    ///    surfaces before channel 1's frame buffer advances, keeping
    ///    the two buffers in lock-step under error (the mirror of the
    ///    decoder's ordering guarantee).
    ///
    /// Both input lengths are pre-checked against `M` (naming the
    /// offending channel) before the fold touches anything, so a
    /// length error never advances either buffer.
    ///
    /// # Errors
    ///
    /// Returns [`StereoEncodeError`] naming the failing channel and
    /// wrapping its [`EncodeError`].
    pub fn block(
        &mut self,
        left: &[f64],
        right: &[f64],
        mode: ChannelMode,
    ) -> Result<StereoEncodedBlock, StereoEncodeError> {
        let m = self.block_len();
        if left.len() != m {
            return Err(StereoEncodeError {
                channel: StereoChannel::Ch0,
                source: EncodeError::Analysis(crate::analysis::InvalidSampleLen {
                    expected: m,
                    got: left.len(),
                }),
            });
        }
        if right.len() != m {
            return Err(StereoEncodeError {
                channel: StereoChannel::Ch1,
                source: EncodeError::Analysis(crate::analysis::InvalidSampleLen {
                    expected: m,
                    got: right.len(),
                }),
            });
        }

        // 1. §8 [sum-difference pre-process], only when the block is
        //    coded jointly. For Independent, a/b are already the two
        //    coded channels.
        let mut a = left.to_vec();
        let mut b = right.to_vec();
        if mode == ChannelMode::SumDifference {
            // (left, right) -> (mid, side) in place.
            crate::stereo::forward_in_place(&mut a, &mut b);
        }

        // 2. Per-channel encode. Channel 0 first so its error surfaces
        //    before channel 1's frame buffer advances.
        let ch0 = self.ch0.block(&a).map_err(|e| StereoEncodeError {
            channel: StereoChannel::Ch0,
            source: e,
        })?;
        let ch1 = self.ch1.block(&b).map_err(|e| StereoEncodeError {
            channel: StereoChannel::Ch1,
            source: e,
        })?;

        Ok(StereoEncodedBlock { ch0, ch1, mode })
    }

    /// Close the stream: encode both channels' final all-zero blocks
    /// ([`crate::encode::ChannelEncoder::flush`]) so the paired
    /// decoder can drain its overlap-add carries.
    ///
    /// `mode` travels with the emitted block so the caller feeds the
    /// decoder's trailing block (and its
    /// [`crate::stereo_decode::StereoDecoder::flush`]) the same mode
    /// the stream ended on; an all-zero pair folds to an all-zero
    /// pair, so the flush samples themselves are mode-independent.
    ///
    /// # Errors
    ///
    /// Returns [`StereoEncodeError`] naming the failing channel, as
    /// [`StereoEncoder::block`] does.
    pub fn flush(&mut self, mode: ChannelMode) -> Result<StereoEncodedBlock, StereoEncodeError> {
        let ch0 = self.ch0.flush().map_err(|e| StereoEncodeError {
            channel: StereoChannel::Ch0,
            source: e,
        })?;
        let ch1 = self.ch1.flush().map_err(|e| StereoEncodeError {
            channel: StereoChannel::Ch1,
            source: e,
        })?;
        Ok(StereoEncodedBlock { ch0, ch1, mode })
    }

    /// Clear both channels' frame buffers at a discontinuity, so the
    /// next [`StereoEncoder::block`] behaves as if freshly
    /// constructed.
    pub fn reset(&mut self) {
        self.ch0.reset();
        self.ch1.reset();
    }
}

/// Failure mode for [`StereoEncoder::block`] / [`StereoEncoder::flush`];
/// names the failing channel and wraps its per-channel [`EncodeError`]
/// — the mirror of [`crate::stereo_decode::StereoDecodeError`].
#[derive(Debug, Clone, PartialEq)]
pub struct StereoEncodeError {
    /// Which channel's encode chain rejected the block.
    pub channel: StereoChannel,
    /// The per-channel encode error.
    pub source: EncodeError,
}

impl core::fmt::Display for StereoEncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-wma::stereo_encode: {} encode failed: {}",
            self.channel, self.source
        )
    }
}

impl std::error::Error for StereoEncodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::Analysis;
    use crate::bands::{BandPlan, BandPolicy};
    use crate::decode::ChannelDecoder;
    use crate::dequant::DequantStage;
    use crate::entropy_mode::Partition;
    use crate::noisefill::NoiseFiller;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::quant::QuantStage;
    use crate::spectral::{SpectralDecode, SpectralEncode};
    use crate::step_size::OverallStepSize;
    use crate::stereo_decode::StereoDecoder;
    use crate::synthesis::Synthesis;
    use crate::window::WindowPair;

    const STEP: f64 = 1e-3;

    fn channel_encoder(bs: BlockSize) -> ChannelEncoder {
        let m = bs.samples() as usize;
        let layout =
            QuantBandLayout::new(vec![QuantBand::new(0, m as u16, 0).unwrap()], m).unwrap();
        let analysis = Analysis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        let quant =
            QuantStage::new(bs, &layout, &[1.0], OverallStepSize::new(STEP).unwrap()).unwrap();
        let spectral = SpectralEncode::new(Partition::new(m as u32, m as u32, false).unwrap());
        ChannelEncoder::new(analysis, quant, spectral).unwrap()
    }

    fn channel_decoder(bs: BlockSize) -> ChannelDecoder {
        let m = bs.samples() as usize;
        let layout =
            QuantBandLayout::new(vec![QuantBand::new(0, m as u16, 0).unwrap()], m).unwrap();
        let spectral = SpectralDecode::new(Partition::new(m as u32, m as u32, false).unwrap());
        let dequant =
            DequantStage::new(bs, &layout, &[1.0], OverallStepSize::new(STEP).unwrap()).unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();
        ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap()
    }

    fn encoder(bs: BlockSize) -> StereoEncoder {
        StereoEncoder::new(channel_encoder(bs), channel_encoder(bs)).unwrap()
    }

    fn decoder(bs: BlockSize) -> StereoDecoder {
        StereoDecoder::new(channel_decoder(bs), channel_decoder(bs)).unwrap()
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

    // ---------- construction / accessors ----------

    #[test]
    fn new_accepts_matching_block_sizes() {
        let enc = encoder(BlockSize::S256);
        assert_eq!(enc.block_size(), BlockSize::S256);
        assert_eq!(enc.block_len(), 256);
        assert_eq!(enc.ch0().block_len(), 256);
        assert_eq!(enc.ch1().block_len(), 256);
    }

    #[test]
    fn new_rejects_mismatched_block_sizes() {
        let err = StereoEncoder::new(
            channel_encoder(BlockSize::S256),
            channel_encoder(BlockSize::S512),
        )
        .unwrap_err();
        assert_eq!(
            err,
            StereoAssemblyError::BlockSizeMismatch {
                ch0: BlockSize::S256,
                ch1: BlockSize::S512,
            }
        );
    }

    // ---------- length pre-checks name the channel ----------

    #[test]
    fn block_rejects_wrong_left_len_without_advancing() {
        let mut enc = encoder(BlockSize::S256);
        let right = vec![0.0; 256];
        let err = enc
            .block(&[0.0; 255], &right, ChannelMode::Independent)
            .unwrap_err();
        assert_eq!(err.channel, StereoChannel::Ch0);
        assert!(matches!(err.source, EncodeError::Analysis(_)));
        // Neither buffer advanced.
        assert!(enc.ch0().analysis().prev().iter().all(|&s| s == 0.0));
        assert!(enc.ch1().analysis().prev().iter().all(|&s| s == 0.0));
    }

    #[test]
    fn block_rejects_wrong_right_len_without_advancing() {
        let mut enc = encoder(BlockSize::S256);
        let left = vec![0.0; 256];
        let err = enc
            .block(&left, &[0.0; 100], ChannelMode::SumDifference)
            .unwrap_err();
        assert_eq!(err.channel, StereoChannel::Ch1);
        assert!(enc.ch0().analysis().prev().iter().all(|&s| s == 0.0));
        assert!(enc.ch1().analysis().prev().iter().all(|&s| s == 0.0));
    }

    // ---------- the chain adds no arithmetic of its own ----------

    #[test]
    fn block_equals_manual_fold_plus_two_channel_chains() {
        let bs = BlockSize::S256;
        let m = 256usize;
        let left = pseudo_random(m, 61);
        let right = pseudo_random(m, 62);

        for mode in [ChannelMode::Independent, ChannelMode::SumDifference] {
            let mut enc = encoder(bs);
            let via_stage = enc.block(&left, &right, mode).unwrap();

            let mut a = left.clone();
            let mut b = right.clone();
            if mode == ChannelMode::SumDifference {
                crate::stereo::forward_in_place(&mut a, &mut b);
            }
            let mut c0 = channel_encoder(bs);
            let mut c1 = channel_encoder(bs);
            let manual0 = c0.block(&a).unwrap();
            let manual1 = c1.block(&b).unwrap();

            assert_eq!(via_stage.ch0, manual0, "mode={mode:?}");
            assert_eq!(via_stage.ch1, manual1, "mode={mode:?}");
            assert_eq!(via_stage.mode, mode);
        }
    }

    // ---------- encode → decode round trips ----------

    /// A constant-mode stereo stream round-trips through the paired
    /// StereoDecoder: after the chain's M-sample leading latency both
    /// channels reproduce the input within the quantizer bound.
    fn assert_stereo_round_trip(mode: ChannelMode, seed: u64) {
        let bs = BlockSize::S256;
        let m = 256usize;
        let blocks = 3usize;
        let left = pseudo_random(blocks * m, seed);
        let right = pseudo_random(blocks * m, seed + 1);

        let mut enc = encoder(bs);
        let mut dec = decoder(bs);

        let mut out_l = Vec::new();
        let mut out_r = Vec::new();
        for t in 0..blocks {
            let eb = enc
                .block(&left[t * m..(t + 1) * m], &right[t * m..(t + 1) * m], mode)
                .unwrap();
            let sb = dec
                .block(
                    &eb.ch0.levels,
                    &eb.ch0.pairs,
                    &[&[]],
                    &eb.ch1.levels,
                    &eb.ch1.pairs,
                    &[&[]],
                    eb.mode,
                )
                .unwrap();
            out_l.extend(sb.left);
            out_r.extend(sb.right);
        }
        let eb = enc.flush(mode).unwrap();
        let sb = dec
            .block(
                &eb.ch0.levels,
                &eb.ch0.pairs,
                &[&[]],
                &eb.ch1.levels,
                &eb.ch1.pairs,
                &[&[]],
                eb.mode,
            )
            .unwrap();
        out_l.extend(sb.left);
        out_r.extend(sb.right);

        assert_eq!(out_l.len(), (blocks + 1) * m);
        let tolerance = 4.0 * STEP;
        for i in 0..blocks * m {
            let el = (out_l[m + i] - left[i]).abs();
            let er = (out_r[m + i] - right[i]).abs();
            assert!(el < tolerance, "mode={mode:?} L i={i}: err {el}");
            assert!(er < tolerance, "mode={mode:?} R i={i}: err {er}");
        }
    }

    #[test]
    fn stereo_round_trip_independent() {
        assert_stereo_round_trip(ChannelMode::Independent, 71);
    }

    #[test]
    fn stereo_round_trip_sum_difference() {
        assert_stereo_round_trip(ChannelMode::SumDifference, 73);
    }

    #[test]
    fn correlated_signal_concentrates_energy_in_mid_channel() {
        // The §5 rationale: for strongly correlated channels the
        // sum/difference fold concentrates energy in channel 0 (mid),
        // leaving channel 1 (side) nearly empty — observable as far
        // fewer non-zero quantized symbols on the side channel.
        let bs = BlockSize::S256;
        let m = 256usize;
        let base = pseudo_random(m, 81);
        let left = base.clone();
        // Right = left with a decorrelation far below the quantizer
        // step (accounting for the reference forward MLT's ~2M gain):
        // the side channel (L - R) / 2 falls inside the dead zone and
        // quantizes away entirely.
        let right: Vec<f64> = base.iter().map(|&x| x * (1.0 - 1e-7)).collect();

        let mut enc = encoder(bs);
        let eb = enc
            .block(&left, &right, ChannelMode::SumDifference)
            .unwrap();
        let nz = |v: &Vec<i32>| v.iter().filter(|&&q| q != 0).count();
        assert_eq!(nz(&eb.ch1.levels), 0, "side channel should quantize away");
        assert!(
            nz(&eb.ch0.levels) > 0,
            "mid channel should carry the correlated energy"
        );
    }

    // ---------- flush / reset / plumbing ----------

    #[test]
    fn flush_carries_the_trailing_mode() {
        let mut enc = encoder(BlockSize::S256);
        let x = pseudo_random(256, 91);
        let _ = enc.block(&x, &x, ChannelMode::SumDifference).unwrap();
        let eb = enc.flush(ChannelMode::SumDifference).unwrap();
        assert_eq!(eb.mode, ChannelMode::SumDifference);
    }

    #[test]
    fn reset_restores_fresh_behaviour() {
        let mut used = encoder(BlockSize::S256);
        let mut fresh = encoder(BlockSize::S256);
        let x = pseudo_random(256, 92);
        let y = pseudo_random(256, 93);
        let _ = used.block(&x, &y, ChannelMode::Independent).unwrap();
        used.reset();
        assert_eq!(
            used.block(&x, &y, ChannelMode::Independent).unwrap(),
            fresh.block(&x, &y, ChannelMode::Independent).unwrap()
        );
    }

    #[test]
    fn into_stereo_block_params_carries_both_channels_and_mode() {
        let eb = StereoEncodedBlock {
            ch0: EncodedBlock {
                levels: vec![1, 2],
                pairs: vec![],
            },
            ch1: EncodedBlock {
                levels: vec![-3, 4],
                pairs: vec![],
            },
            mode: ChannelMode::SumDifference,
        };
        let sbp = eb.clone().into_stereo_block_params(1);
        assert_eq!(sbp.ch0.levels, eb.ch0.levels);
        assert_eq!(sbp.ch1.levels, eb.ch1.levels);
        assert_eq!(sbp.mode, ChannelMode::SumDifference);
        assert_eq!(sbp.ch0.patterns.len(), 1);
        assert_eq!(sbp.ch1.patterns.len(), 1);
    }

    #[test]
    fn error_display_names_the_channel() {
        let e = StereoEncodeError {
            channel: StereoChannel::Ch1,
            source: EncodeError::Analysis(crate::analysis::InvalidSampleLen {
                expected: 256,
                got: 255,
            }),
        };
        let s = format!("{e}");
        assert!(s.contains("channel 1"));
        assert!(std::error::Error::source(&e).is_some());
    }
}

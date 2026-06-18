//! WMA two-channel decoder-block assembler — the full §8 FIG.6 decoder
//! chain for one block of a **stereo** pair.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §8 draws the decoder
//! pipeline (Thumpudi-180 FIG.6, inverse of the encoder FIG.5) and fixes
//! the order of its stages, ending with the multi-channel post-process:
//!
//! > ```text
//! > bitstream
//! >  → DEMUX (each process extracts its own parameters)           (US7,885,819 FIG.7)
//! >  → entropy decode (run-level → coefficients; matrix deltas)   (US6,223,162; US7,930,171)
//! >  → inverse quantize + inverse weighting                       (US7,383,180; US6,240,380)
//! >  → fill noise-substituted bands (noise generator)             (US7,383,180 mod 240)
//! >  → inverse MLT                                                (US6,029,126/380)
//! >  → overlap-add                                                (US7,383,180 overlapper/adder)
//! >  → [inverse sum-difference / multi-channel post-process]      (US7,502,743)
//! >  → PCM
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (decoder
//! >     pipeline, Thumpudi-180 FIG.6)
//!
//! The §5 two-channel transform that the trailing box inverts is the
//! patent-disclosed sum/difference (mid/side) coding for WMA Standard:
//!
//! > For stereo, WMA7 can code the two channels as **sum and difference
//! > channels** — the sum being the channel average and the difference
//! > being half the channel difference (i.e. mid/side).
//! >   — [PATENT US7,930,171 — WMA7 sum/difference]
//! >   — [PATENT US7,502,743 — prior-art sum/difference baseline]
//!
//! ## Scope of this module
//!
//! This module is the **stereo analogue of [`crate::decode`]**: where
//! [`crate::decode::ChannelDecoder`] assembles the §8 *per-channel
//! front-to-back* chain (entropy decode → inverse quantize/weight →
//! noise-fill → inverse MLT → window → overlap-add) for one channel, this
//! module wires **two** of those full per-channel chains and closes the
//! pipeline with the §8 `[inverse sum-difference]` multi-channel
//! post-process box that operates *across* the two reconstructed channels.
//!
//! It is the same relationship that holds at the synthesis-only layer:
//! [`crate::stereo_synthesis::StereoSynthesis`] is the stereo analogue of
//! [`crate::synthesis::Synthesis`], wiring two synthesis tails + the fold.
//! That stage, however, consumes **already-dequantized coefficients** per
//! channel — it begins at the inverse MLT. This module begins one stage
//! earlier still, at the *entropy decode* box, so it is the first
//! assembler that takes one stereo block's already-demuxed per-channel
//! entropy symbols all the way to final L/R PCM.
//!
//! For one block of two channels the chain runs, in patent order:
//!
//! 1. **Per-channel decode (×2)** — each channel's already-demuxed
//!    entropy symbols are run through its own
//!    [`crate::decode::ChannelDecoder`], producing `M` reconstructed
//!    time-domain samples per channel. Each channel carries its **own**
//!    overlap-add tail, so the two per-channel chains are independent
//!    across the block sequence.
//! 2. **Inverse sum/difference post-process** — applied **only when the
//!    block was coded jointly** (the per-block
//!    [`crate::channel_decision::ChannelMode`] is
//!    [`ChannelMode::SumDifference`]): the two reconstructed channels are
//!    the *mid* / *side* time-domain signals and
//!    [`crate::stereo::inverse_in_place`] folds them back to *left* /
//!    *right*. For [`ChannelMode::Independent`] the two channels are
//!    already left/right and the box is bypassed.
//!
//! The fold runs **after** each channel's overlap-add — exactly where the
//! FIG.6 diagram places the `[inverse sum-difference]` box — so the
//! per-channel overlap-add carriers always see the per-channel (mid/side
//! or left/right) signals, never the folded output. This matches
//! [`crate::stereo_synthesis::StereoSynthesis`] precisely, because the
//! fold position is a property of the FIG.6 chain, not of which front-half
//! feeds it.
//!
//! ## What this module deliberately does NOT do
//!
//! * **No new arithmetic.** Every transform / quantization / entropy /
//!   noise-fill / overlap-add operation lives in the underlying stages
//!   (via [`crate::decode::ChannelDecoder`]) and the sum/difference fold
//!   lives in [`crate::stereo`]; this stage only sequences them in the
//!   patent-fixed FIG.6 order across the two channels.
//! * **No flag parsing / DEMUX.** Per §6 the codeword tables and the
//!   per-process parameter demux (US7,885,819 FIG.7) are `[GAP]`, and per
//!   §5 the v1/v2 channel-mode flag layout is `[GAP]`; this assembler
//!   consumes the already-demuxed per-channel symbols and the
//!   caller-supplied per-block [`ChannelMode`], never fabricating either.
//!   The §5 *decision* an encoder makes is modelled separately by
//!   [`crate::channel_decision::OpenLoopDecision`]; here the chosen mode
//!   is an input.
//! * **No block-size-transition handling.** Both per-channel
//!   [`crate::decode::ChannelDecoder`] carry one uniform
//!   [`crate::block::BlockSize`] `M` (the patent's per-block window/
//!   block-size decision is one decision for the tile, §2, so both
//!   channels of a stereo block share `M`); adjacent blocks of different
//!   patent-disclosed sizes (§2) need transition handling whose shape is
//!   `[GAP]`, the same deferral the per-channel chains record.

use crate::block::BlockSize;
use crate::channel_decision::ChannelMode;
use crate::decode::{ChannelDecoder, DecodeError};
use crate::runlevel::RunLevelPair;
use crate::stereo_synthesis::StereoBlock;

/// Stateful two-channel decoder-block assembler for one uniform
/// [`BlockSize`] `M`, per §8 of the patent trace (Thumpudi-180 FIG.6:
/// per-channel entropy decode → inverse quantize/weight → noise-fill →
/// inverse MLT → window → overlap-add, then the `[inverse sum-difference]`
/// post-process across the two channels).
///
/// Owns two independent per-channel [`ChannelDecoder`] stages — one for
/// each of the two channels — and applies the §5 inverse sum/difference
/// transform as the FIG.6 post-process, gated by the per-block
/// [`ChannelMode`]. One [`StereoDecoder::block`] call consumes both
/// channels' already-demuxed entropy symbols and emits the final L/R PCM
/// for the block, carrying each channel's overlap-add tail across calls.
///
/// This is the stereo analogue of [`ChannelDecoder`]; construct the two
/// per-channel decoders independently (each validates its own
/// block-size / band-count / length invariants), then assemble them here —
/// the constructor cross-checks that both agree on the same `M`.
#[derive(Debug, Clone)]
pub struct StereoDecoder {
    ch0: ChannelDecoder,
    ch1: ChannelDecoder,
}

impl StereoDecoder {
    /// Assemble two per-channel decoders into one stereo decode chain.
    ///
    /// Both per-channel [`ChannelDecoder`] must describe the same block
    /// size: the patent's per-block window/block-size decision is one
    /// decision for the tile (§2), so both channels of a stereo block
    /// transform with the same `M`. The constructor cross-checks the two
    /// block sizes so the per-channel length contracts and the
    /// sum/difference fold (which operates element-wise across the two
    /// `M`-sample channels) cannot fail at decode time.
    ///
    /// # Errors
    ///
    /// Returns [`StereoAssemblyError::BlockSizeMismatch`] naming the two
    /// channels' declared block sizes when they disagree.
    pub fn new(ch0: ChannelDecoder, ch1: ChannelDecoder) -> Result<Self, StereoAssemblyError> {
        let m0 = ch0.block_size();
        let m1 = ch1.block_size();
        if m0 != m1 {
            return Err(StereoAssemblyError::BlockSizeMismatch { ch0: m0, ch1: m1 });
        }
        Ok(Self { ch0, ch1 })
    }

    /// Block size `M` for this decoder (shared by both channels).
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.ch0.block_size()
    }

    /// `M`, the per-channel reconstructed-sample output length.
    #[inline]
    pub fn block_len(&self) -> usize {
        self.ch0.block_len()
    }

    /// The first (channel-0) per-channel decoder.
    #[inline]
    pub const fn ch0(&self) -> &ChannelDecoder {
        &self.ch0
    }

    /// The second (channel-1) per-channel decoder.
    #[inline]
    pub const fn ch1(&self) -> &ChannelDecoder {
        &self.ch1
    }

    /// Decode one stereo block into the final L/R PCM, running the full §8
    /// FIG.6 chain for both channels and the trailing multi-channel
    /// post-process in patent order.
    ///
    /// Each channel's arguments are the already-demuxed, already-decoded
    /// per-block parameters [`ChannelDecoder::block`] consumes:
    ///
    /// * `levels0` / `levels1` — the level-mode head symbols for each
    ///   channel's spectral stage (length `split`).
    /// * `pairs0` / `pairs1` — the run-level `(R, L)` pairs for each
    ///   channel's high-frequency tail.
    /// * `patterns0` / `patterns1` — one entry per band for each channel's
    ///   noise filler.
    ///
    /// `mode` is the per-block [`ChannelMode`] the (caller-parsed)
    /// bitstream selected:
    ///
    /// * [`ChannelMode::Independent`] — the two reconstructed channels are
    ///   already left/right; the FIG.6 sum/difference box is bypassed.
    /// * [`ChannelMode::SumDifference`] — the two reconstructed channels
    ///   are the *mid* / *side* time-domain signals; the §8 inverse
    ///   sum/difference post-process folds them back to left/right via
    ///   [`crate::stereo::inverse_in_place`].
    ///
    /// The per-channel overlap-add carry advances for **both** channels on
    /// every call regardless of `mode`, because each channel's full decode
    /// (including overlap-add) runs before the fold — the FIG.6 chain
    /// places the post-process *after* the overlap-add.
    ///
    /// Channel 0 is decoded first, so a [`DecodeError`] from it surfaces
    /// before channel 1's carry is advanced, keeping the two carriers in
    /// lock-step under error.
    ///
    /// # Errors
    ///
    /// Returns [`StereoDecodeError`] naming the failing channel and
    /// wrapping its [`DecodeError`].
    #[allow(clippy::too_many_arguments)]
    pub fn block(
        &mut self,
        levels0: &[i32],
        pairs0: &[RunLevelPair],
        patterns0: &[&[f64]],
        levels1: &[i32],
        pairs1: &[RunLevelPair],
        patterns1: &[&[f64]],
        mode: ChannelMode,
    ) -> Result<StereoBlock, StereoDecodeError> {
        // 1. Per-channel decode (entropy -> dequant -> noise-fill ->
        //    inverse MLT -> window -> overlap-add). Channel 0 first so its
        //    error surfaces before channel 1's carry advances.
        let mut a = self
            .ch0
            .block(levels0, pairs0, patterns0)
            .map_err(|e| StereoDecodeError {
                channel: StereoChannel::Ch0,
                source: e,
            })?;
        let mut b = self
            .ch1
            .block(levels1, pairs1, patterns1)
            .map_err(|e| StereoDecodeError {
                channel: StereoChannel::Ch1,
                source: e,
            })?;

        // 2. §8 [inverse sum-difference] post-process, only when the block
        //    was coded jointly. For Independent, a/b are already L/R.
        match mode {
            ChannelMode::Independent => {}
            ChannelMode::SumDifference => {
                // a = mid, b = side -> (left, right) in place.
                crate::stereo::inverse_in_place(&mut a, &mut b);
            }
        }

        Ok(StereoBlock { left: a, right: b })
    }

    /// Drain the trailing-edge overlap-add tail after the last block,
    /// returning the final reconstructed samples for both channels and
    /// zeroing both carries.
    ///
    /// `mode` selects whether the drained tail is post-processed, exactly
    /// as [`StereoDecoder::block`] governs it for a coded block: the
    /// trailing block's coding mode governs the fold of the final
    /// overlap-add tail. A decoder calls [`StereoDecoder::block`] for every
    /// received block, then [`StereoDecoder::flush`] once with the last
    /// block's mode to retrieve the remaining tail.
    pub fn flush(&mut self, mode: ChannelMode) -> StereoBlock {
        let mut a = self.ch0.flush();
        let mut b = self.ch1.flush();
        if mode == ChannelMode::SumDifference {
            crate::stereo::inverse_in_place(&mut a, &mut b);
        }
        StereoBlock { left: a, right: b }
    }

    /// Clear both channels' overlap-add carry at a discontinuity (seek /
    /// decoder flush), so the next [`StereoDecoder::block`] behaves as if
    /// this stage were freshly constructed.
    pub fn reset(&mut self) {
        self.ch0.reset();
        self.ch1.reset();
    }
}

/// Names the two channels of a stereo block, for error reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StereoChannel {
    /// The first channel (mid, under sum/difference coding).
    Ch0,
    /// The second channel (side, under sum/difference coding).
    Ch1,
}

impl core::fmt::Display for StereoChannel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            StereoChannel::Ch0 => "channel 0",
            StereoChannel::Ch1 => "channel 1",
        })
    }
}

/// Rejection reason for [`StereoDecoder::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StereoAssemblyError {
    /// The two per-channel decoders declare different block sizes. The
    /// patent's per-block window/block-size decision is one decision for
    /// the tile (§2), so both channels must share `M`.
    BlockSizeMismatch {
        /// Block size channel 0 declared.
        ch0: BlockSize,
        /// Block size channel 1 declared.
        ch1: BlockSize,
    },
}

impl core::fmt::Display for StereoAssemblyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            StereoAssemblyError::BlockSizeMismatch { ch0, ch1 } => write!(
                f,
                "oxideav-wma::stereo_decode: channel 0 declares block size {} but channel 1 declares {}",
                ch0.samples(),
                ch1.samples(),
            ),
        }
    }
}

impl std::error::Error for StereoAssemblyError {}

/// Failure mode for [`StereoDecoder::block`]; names the failing channel
/// and wraps its per-channel [`DecodeError`].
#[derive(Debug, Clone, PartialEq)]
pub struct StereoDecodeError {
    /// Which channel's decode chain rejected the block.
    pub channel: StereoChannel,
    /// The per-channel decode error.
    pub source: DecodeError,
}

impl core::fmt::Display for StereoDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-wma::stereo_decode: {} decode failed: {}",
            self.channel, self.source
        )
    }
}

impl std::error::Error for StereoDecodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bands::{BandPlan, BandPolicy};
    use crate::dequant::DequantStage;
    use crate::entropy_mode::Partition;
    use crate::noisefill::NoiseFiller;
    use crate::qband::{QuantBand, QuantBandLayout};
    use crate::spectral::SpectralDecode;
    use crate::step_size::OverallStepSize;
    use crate::window::WindowPair;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    fn single_band_layout(bs: BlockSize) -> QuantBandLayout {
        QuantBandLayout::for_block(vec![QuantBand::new(0, bs.samples(), 0).unwrap()], bs).unwrap()
    }

    /// An all-coded single-band per-channel decoder (whole-block run-level
    /// entropy, unit weight, unit step) — the building block under test.
    fn coded_decoder(bs: BlockSize) -> ChannelDecoder {
        let m = bs.samples();
        let spectral = SpectralDecode::new(Partition::new(m as u32, 0, false).unwrap());
        let layout = single_band_layout(bs);
        let dequant =
            DequantStage::new(bs, &layout, &[1.0_f64], OverallStepSize::new(1.0).unwrap()).unwrap();
        let plan = BandPlan::new(vec![BandPolicy::Coded]);
        let noise = NoiseFiller::new(plan, layout).unwrap();
        let synthesis = make_synthesis(bs);
        ChannelDecoder::new(spectral, dequant, noise, synthesis).unwrap()
    }

    fn make_synthesis(bs: BlockSize) -> crate::synthesis::Synthesis {
        crate::synthesis::Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap()
    }

    fn stereo(bs: BlockSize) -> StereoDecoder {
        StereoDecoder::new(coded_decoder(bs), coded_decoder(bs)).unwrap()
    }

    // ---------- construction / accessors ----------

    #[test]
    fn new_accepts_matching_block_size_and_accessors_agree() {
        let bs = BlockSize::from_samples(256).unwrap();
        let dec = stereo(bs);
        assert_eq!(dec.block_size(), bs);
        assert_eq!(dec.block_len(), 256);
        assert_eq!(dec.ch0().block_len(), 256);
        assert_eq!(dec.ch1().block_len(), 256);
    }

    #[test]
    fn new_rejects_block_size_mismatch() {
        let small = BlockSize::from_samples(256).unwrap();
        let big = BlockSize::from_samples(512).unwrap();
        let err = StereoDecoder::new(coded_decoder(small), coded_decoder(big)).unwrap_err();
        assert_eq!(
            err,
            StereoAssemblyError::BlockSizeMismatch {
                ch0: small,
                ch1: big,
            }
        );
    }

    // ---------- full-chain decode ----------

    /// Independent mode must equal running two bare `ChannelDecoder`s by
    /// hand and pairing their outputs: pins that the assembler adds no
    /// arithmetic and bypasses the sum/difference box when independent.
    #[test]
    fn independent_equals_two_bare_channel_decoders() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let empty: &[f64] = &[];

        // Distinct inputs per channel so a channel swap would be caught.
        let p0 = [pair((m - 1) as u32, 3)];
        let p1 = [pair((m - 1) as u32, 7)];

        let mut dec = stereo(bs);
        let got = dec
            .block(
                &[],
                &p0,
                &[empty],
                &[],
                &p1,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();

        let mut h0 = coded_decoder(bs);
        let mut h1 = coded_decoder(bs);
        let a = h0.block(&[], &p0, &[empty]).unwrap();
        let b = h1.block(&[], &p1, &[empty]).unwrap();
        assert_eq!(got.left, a);
        assert_eq!(got.right, b);
    }

    /// Sum/difference mode must equal: decode each channel with a bare
    /// `ChannelDecoder`, then fold with `stereo::inverse_in_place`. Pins
    /// that the post-process runs *after* the per-channel decode, in FIG.6
    /// order.
    #[test]
    fn sum_difference_equals_decode_then_inverse_fold() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let empty: &[f64] = &[];
        let p0 = [pair((m - 1) as u32, 4)];
        let p1 = [pair((m - 1) as u32, 9)];

        let mut dec = stereo(bs);
        let got = dec
            .block(
                &[],
                &p0,
                &[empty],
                &[],
                &p1,
                &[empty],
                ChannelMode::SumDifference,
            )
            .unwrap();

        let mut h0 = coded_decoder(bs);
        let mut h1 = coded_decoder(bs);
        let mut a = h0.block(&[], &p0, &[empty]).unwrap();
        let mut b = h1.block(&[], &p1, &[empty]).unwrap();
        crate::stereo::inverse_in_place(&mut a, &mut b);
        assert_eq!(got.left, a);
        assert_eq!(got.right, b);

        // And the fold genuinely changed the output: an independent-mode
        // decode of the same inputs differs.
        let mut dec2 = stereo(bs);
        let indep = dec2
            .block(
                &[],
                &p0,
                &[empty],
                &[],
                &p1,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        assert_ne!(got.left, indep.left);
    }

    /// End-to-end proof the post-process inverts the §5 transform in the
    /// time domain through the *full* per-channel decode chain. A
    /// constant-amplitude correlated stereo pair is coded as mid/side; the
    /// mid/side MLT coefficients are quantized to integer levels and fed
    /// through the entropy stage's level-mode verbatim path (`split == M`)
    /// with identity dequant (unit weight, unit step) and an all-`Coded`
    /// band. After priming to steady state, the inverse sum/difference fold
    /// recovers the original L/R within the integer-rounding tolerance.
    #[test]
    fn steady_state_sum_difference_round_trip() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let mlt = crate::mlt::Mlt::new(bs);
        let wpair = WindowPair::orthogonal_sine(bs);

        let l_val = 0.8_f64;
        let r_val = 0.2_f64;
        let mid_val = (l_val + r_val) * 0.5;
        let side_val = (l_val - r_val) * 0.5;

        // Analysis coefficients of a constant channel over a 2M frame.
        let analysis_coeffs = |val: f64| {
            let mut frame = vec![val; 2 * m];
            wpair.analysis().apply_in_place(&mut frame).unwrap();
            mlt.forward(&frame).unwrap()
        };
        // Round to integer levels; identity dequant reproduces them
        // verbatim, so the only error vs. the real coefficients is the
        // integer rounding.
        let mid_i: Vec<i32> = analysis_coeffs(mid_val)
            .iter()
            .map(|&c| c.round() as i32)
            .collect();
        let side_i: Vec<i32> = analysis_coeffs(side_val)
            .iter()
            .map(|&c| c.round() as i32)
            .collect();

        // A per-channel decoder whose entropy stage copies level-mode head
        // symbols verbatim across the whole block (split == M).
        let build_levelmode = || {
            let spectral = SpectralDecode::new(Partition::new(m as u32, m as u32, false).unwrap());
            let layout = single_band_layout(bs);
            let dequant =
                DequantStage::new(bs, &layout, &[1.0_f64], OverallStepSize::new(1.0).unwrap())
                    .unwrap();
            let plan = BandPlan::new(vec![BandPolicy::Coded]);
            let noise = NoiseFiller::new(plan, layout).unwrap();
            ChannelDecoder::new(spectral, dequant, noise, make_synthesis(bs)).unwrap()
        };
        let mut dec = StereoDecoder::new(build_levelmode(), build_levelmode()).unwrap();

        let empty: &[f64] = &[];
        let decode_block = |dec: &mut StereoDecoder| {
            dec.block(
                &mid_i,
                &[],
                &[empty],
                &side_i,
                &[],
                &[empty],
                ChannelMode::SumDifference,
            )
            .unwrap()
        };
        // Prime to steady state, then read the third block.
        let _ = decode_block(&mut dec);
        let _ = decode_block(&mut dec);
        let steady = decode_block(&mut dec);

        for &x in &steady.left {
            assert!((x - l_val).abs() < 0.5, "left {x} != {l_val}");
        }
        for &x in &steady.right {
            assert!((x - r_val).abs() < 0.5, "right {x} != {r_val}");
        }
    }

    // ---------- state: per-channel carries ----------

    #[test]
    fn carry_persists_across_blocks_and_flush_drains_tail() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let empty: &[f64] = &[];
        let p = [pair((m - 1) as u32, 5)];

        let mut dec = stereo(bs);
        let _ = dec
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        let b1 = dec
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();

        let mut fresh = stereo(bs);
        let b1_fresh = fresh
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        assert_ne!(b1.left, b1_fresh.left, "carry must affect block 1");

        let tail = dec.flush(ChannelMode::Independent);
        assert_eq!(tail.left.len(), m);
        assert_eq!(tail.right.len(), m);
    }

    #[test]
    fn reset_clears_both_carries_to_fresh_state() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let empty: &[f64] = &[];
        let p = [pair((m - 1) as u32, 5)];

        let mut dec = stereo(bs);
        let _ = dec
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        dec.reset();
        let after = dec
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();

        let mut fresh = stereo(bs);
        let first = fresh
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        assert_eq!(after.left, first.left);
        assert_eq!(after.right, first.right);
    }

    #[test]
    fn flush_folds_when_mode_is_joint() {
        let bs = BlockSize::from_samples(256).unwrap();
        let m = bs.samples() as usize;
        let empty: &[f64] = &[];
        let p = [pair((m - 1) as u32, 5)];

        let mut dec = stereo(bs);
        let _ = dec
            .block(
                &[],
                &p,
                &[empty],
                &[],
                &p,
                &[empty],
                ChannelMode::SumDifference,
            )
            .unwrap();

        // Hand-wired: the two per-channel flush tails, then folded.
        let mut h0 = coded_decoder(bs);
        let mut h1 = coded_decoder(bs);
        let _ = h0.block(&[], &p, &[empty]).unwrap();
        let _ = h1.block(&[], &p, &[empty]).unwrap();
        let mut ta = h0.flush();
        let mut tb = h1.flush();
        crate::stereo::inverse_in_place(&mut ta, &mut tb);

        let flushed = dec.flush(ChannelMode::SumDifference);
        assert_eq!(flushed.left, ta);
        assert_eq!(flushed.right, tb);
    }

    // ---------- error propagation ----------

    #[test]
    fn block_propagates_channel0_error_naming_channel() {
        let bs = BlockSize::from_samples(256).unwrap();
        let mut dec = stereo(bs);
        let empty: &[f64] = &[];
        let good = [pair(255, 1)];
        // split == 0, so a non-empty levels vector on channel 0 is a
        // spectral length mismatch.
        let err = dec
            .block(
                &[1, 2, 3],
                &[pair(256, 1)],
                &[empty],
                &[],
                &good,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap_err();
        assert_eq!(err.channel, StereoChannel::Ch0);
        assert!(matches!(err.source, DecodeError::Spectral(_)));
    }

    #[test]
    fn block_propagates_channel1_error_naming_channel() {
        let bs = BlockSize::from_samples(256).unwrap();
        let mut dec = stereo(bs);
        let empty: &[f64] = &[];
        let good = [pair(255, 1)];
        let err = dec
            .block(
                &[],
                &good,
                &[empty],
                &[1, 2, 3],
                &[pair(256, 1)],
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap_err();
        assert_eq!(err.channel, StereoChannel::Ch1);
        assert!(matches!(err.source, DecodeError::Spectral(_)));
    }

    /// On a channel-0 error, channel 1's carry must NOT have advanced
    /// (channel 0 is decoded first and bails before channel 1 runs).
    #[test]
    fn channel0_error_leaves_channel1_carry_untouched() {
        let bs = BlockSize::from_samples(256).unwrap();
        let mut dec = stereo(bs);
        let empty: &[f64] = &[];
        let good = [pair(255, 1)];
        let _ = dec
            .block(
                &[1, 2, 3],
                &[pair(256, 1)],
                &[empty],
                &[],
                &good,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap_err();
        // A subsequent good block on a fresh decoder must match the next
        // block here, proving channel 1 never advanced on the failed call.
        let next = dec
            .block(
                &[],
                &good,
                &[empty],
                &[],
                &good,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        let mut fresh = stereo(bs);
        let fresh_first = fresh
            .block(
                &[],
                &good,
                &[empty],
                &[],
                &good,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap();
        assert_eq!(next.right, fresh_first.right);
    }

    // ---------- error Display / source ----------

    #[test]
    fn assembly_error_display_names_both_sizes() {
        let e = StereoAssemblyError::BlockSizeMismatch {
            ch0: BlockSize::from_samples(256).unwrap(),
            ch1: BlockSize::from_samples(512).unwrap(),
        };
        let s = format!("{e}");
        assert!(s.contains("256") && s.contains("512"));
    }

    #[test]
    fn decode_error_display_and_source() {
        let bs = BlockSize::from_samples(256).unwrap();
        let mut dec = stereo(bs);
        let empty: &[f64] = &[];
        let good = [pair(255, 1)];
        let err = dec
            .block(
                &[1],
                &[pair(256, 1)],
                &[empty],
                &[],
                &good,
                &[empty],
                ChannelMode::Independent,
            )
            .unwrap_err();
        assert!(format!("{err}").contains("channel 0"));
        assert!(std::error::Error::source(&err).is_some());
    }

    #[test]
    fn stereo_channel_display() {
        assert_eq!(format!("{}", StereoChannel::Ch0), "channel 0");
        assert_eq!(format!("{}", StereoChannel::Ch1), "channel 1");
    }

    // ---------- every block size ----------

    #[test]
    fn decodes_every_patent_block_size() {
        for bs in BlockSize::ALL {
            let m = bs.samples() as usize;
            let mut dec = stereo(bs);
            let empty: &[f64] = &[];
            let p = [pair((m - 1) as u32, 2)];
            let out = dec
                .block(
                    &[],
                    &p,
                    &[empty],
                    &[],
                    &p,
                    &[empty],
                    ChannelMode::SumDifference,
                )
                .unwrap();
            assert_eq!(out.left.len(), m);
            assert_eq!(out.right.len(), m);
            assert!(out.left.iter().all(|v| v.is_finite()));
            assert!(out.right.iter().all(|v| v.is_finite()));
        }
    }
}

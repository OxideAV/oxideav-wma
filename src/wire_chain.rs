//! Header-to-chain assembler over the staged real data.
//!
//! ## What this is
//!
//! The first assembler whose per-block geometry comes from the
//! **stream itself** rather than from caller-invented parameters:
//! given a parsed [`WmaHeader`], it derives the block configuration
//! the vendor decoder derives at open time —
//!
//! 1. the long-block [`BlockSize`] from the header's
//!    `frame_length_bits` decision tree (wiki snapshot),
//! 2. the **exponent/quantization-band partition** by scaling the
//!    staged critical-band Hz seed to coefficient bins
//!    ([`crate::exponent_bands`], per the staged provenance's
//!    derived-not-tabulated band rule and the wiki's "compute the
//!    scale factor band sizes for each MDCT block size" init step),
//! 3. the **noise/high-band grid** from the staged octave subband
//!    seed, and
//! 4. the per-band `Q[d]` weights from the staged 113-step
//!    **dequantization gain ladder** ([`crate::gain_ladder`]), driven
//!    by a per-band exponent-index vector.
//!
//! and assembles the §8 encoder/decoder block chains
//! ([`crate::encode::ChannelEncoder`] /
//! [`crate::decode::ChannelDecoder`]) over that real geometry — the
//! r383 encoder mirror acting as the self-consistency oracle for the
//! staged data path.
//!
//! ## What stays `[GAP]` (and is therefore still an input)
//!
//! * The per-band **exponent indices** as carried in the bitstream
//!   (scale/gain VLC tables located but unstaged) — supplied per
//!   stream here.
//! * The **overall step size** selection (encoder rate-control
//!   tuning per the patent trace) — supplied.
//! * The spectral **partition split** / entropy-mode rule and the
//!   symbol → `(R, L)` mapping of the real coefficient VLCs — the
//!   entropy stage still runs on typed symbols; the real-VLC bit
//!   level is exercised separately by [`crate::coef_vlc`].
//! * The per-band coding-policy (noise decisions) — default
//!   all-coded here, overridable through the plan argument.

use crate::analysis::Analysis;
use crate::bands::{BandPlan, BandPolicy};
use crate::bitio::{BitReader, BitWriter};
use crate::block::BlockSize;
use crate::coef_vlc::{CoefDecodeMode, CoefVlc, CoefVlcError};
use crate::decode::{AssemblyError, ChannelDecoder};
use crate::dequant::{DequantStage, InvalidDequant};
use crate::encode::{ChannelEncoder, EncodeAssemblyError};
use crate::entropy_mode::Partition;
use crate::envelope_vlc::{GainVlc, ScaleVlc};
use crate::exponent_bands::{exponent_band_layout, noise_band_layout, BandDeriveError};
use crate::frame_bits::{
    read_frame, read_packet, write_frame, write_packet, BlockPlan, FrameBitsError,
    FrameFieldWidths, WireFrame,
};
use crate::gain_ladder::{band_weights, GainLadderError};
use crate::header::WmaHeader;
use crate::noisefill::{InvalidNoiseFill, NoiseFiller};
use crate::paircode::EscapeWidths;
use crate::qband::QuantBandLayout;
use crate::quant::{InvalidQuant, QuantStage};
use crate::setup::SetupParams;
use crate::spectral::{SpectralDecode, SpectralEncode};
use crate::step_size::OverallStepSize;
use crate::stereo_decode::{StereoAssemblyError, StereoDecoder};
use crate::stereo_encode::StereoEncoder;
use crate::synthesis::Synthesis;
use crate::window::WindowPair;
use crate::wire_tables;

/// The real-data block configuration derived once from a parsed
/// header — the typed carrier of steps 1–3 above.
#[derive(Debug, Clone)]
pub struct WireBlockConfig {
    block: BlockSize,
    exponent_layout: QuantBandLayout,
    noise_layout: QuantBandLayout,
}

/// Failure modes for the wire-chain assembly.
#[derive(Debug, Clone, PartialEq)]
pub enum WireChainError {
    /// The header's `frame_length_bits` fell outside the patent
    /// block-size set (defensive; the parser's decision tree only
    /// produces 9/10/11).
    BadBlockSize(crate::Error),
    /// Band derivation failed (zero sample rate).
    BandDerive(BandDeriveError),
    /// The exponent-index vector did not match the derived band count.
    BandCountMismatch {
        /// Bands in the derived exponent partition.
        expected: usize,
        /// Exponent indices supplied.
        got: usize,
    },
    /// A ladder lookup failed (index outside the 113-step ladder).
    GainLadder(GainLadderError),
    /// A §4 stage rejected the assembled parameters.
    Quant(InvalidQuant),
    /// The decoder-side §4 stage rejected the assembled parameters.
    Dequant(InvalidDequant),
    /// The noise filler rejected the plan/layout pairing.
    NoiseFill(InvalidNoiseFill),
    /// The §8 encoder assembly cross-check failed.
    EncodeAssembly(EncodeAssemblyError),
    /// The §8 decoder assembly cross-check failed.
    DecodeAssembly(AssemblyError),
    /// The two-channel assembly cross-check failed (defensive; both
    /// channels are built from one config so their block sizes agree).
    StereoAssembly(StereoAssemblyError),
    /// A derived frame-field width fell outside `1..=32` (malformed
    /// header rates) or the escape-width pairing was rejected.
    BadFieldWidth {
        /// Which width was rejected.
        field: &'static str,
        /// The rejected value.
        value: u32,
    },
    /// The header's channel count is outside the staged layout's
    /// `1..=2` (the B3 flag is defined for exactly 2 channels).
    UnsupportedChannels {
        /// The header's channel count.
        channels: u8,
    },
    /// A coefficient VLC could not be built (defensive; the staged
    /// tables construct — pinned by test in [`crate::coef_vlc`]).
    CoefVlc(CoefVlcError),
    /// The frame bit layer failed during encode/decode.
    FrameBits(FrameBitsError),
    /// The staged §4b rule does not pin the decode class for this
    /// stream: at or above the 32 kHz gate the class is
    /// bitrate-gated, and the region → class branch directions are
    /// the unstaged residual — the caller resolves the class and uses
    /// [`WireFrameCodec::from_header`].
    ClassNotPinned {
        /// The stream's sample rate (at or above
        /// [`DECODE_CLASS_RATE_THRESHOLD_HZ`]).
        sample_rate: u32,
        /// Where the supplied rate float sits on the selector axis.
        region: RateFloatRegion,
    },
}

impl core::fmt::Display for WireChainError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WireChainError::BadBlockSize(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::BandDerive(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::BandCountMismatch { expected, got } => write!(
                f,
                "oxideav-wma::wire_chain: {got} exponent indices for a {expected}-band partition",
            ),
            WireChainError::GainLadder(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::Quant(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::Dequant(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::NoiseFill(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::EncodeAssembly(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::DecodeAssembly(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::StereoAssembly(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::BadFieldWidth { field, value } => write!(
                f,
                "oxideav-wma::wire_chain: derived width `{field}` = {value} is outside 1..=32",
            ),
            WireChainError::UnsupportedChannels { channels } => write!(
                f,
                "oxideav-wma::wire_chain: {channels} channels (the staged block layout covers 1..=2)",
            ),
            WireChainError::CoefVlc(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::FrameBits(e) => write!(f, "oxideav-wma::wire_chain: {e}"),
            WireChainError::ClassNotPinned {
                sample_rate,
                region,
            } => write!(
                f,
                "oxideav-wma::wire_chain: the staged rule does not pin the decode class at \
                 {sample_rate} Hz (bitrate-gated, rate float in {region:?}; the branch \
                 directions are the unstaged residual)",
            ),
        }
    }
}

impl std::error::Error for WireChainError {}

impl From<BandDeriveError> for WireChainError {
    fn from(e: BandDeriveError) -> Self {
        WireChainError::BandDerive(e)
    }
}

impl From<GainLadderError> for WireChainError {
    fn from(e: GainLadderError) -> Self {
        WireChainError::GainLadder(e)
    }
}

impl From<CoefVlcError> for WireChainError {
    fn from(e: CoefVlcError) -> Self {
        WireChainError::CoefVlc(e)
    }
}

impl From<FrameBitsError> for WireChainError {
    fn from(e: FrameBitsError) -> Self {
        WireChainError::FrameBits(e)
    }
}

impl WireBlockConfig {
    /// Derive the long-block configuration from a parsed header: the
    /// typed block size plus both staged-seed band partitions at the
    /// header's sample rate.
    ///
    /// # Errors
    ///
    /// * [`WireChainError::BadBlockSize`] if `frame_length_bits` maps
    ///   outside the patent set (defensive).
    /// * [`WireChainError::BandDerive`] for a zero sample rate
    ///   (unreachable through the parser, which rejects it).
    pub fn from_header(header: &WmaHeader) -> Result<Self, WireChainError> {
        let block = header
            .long_block_size()
            .map_err(WireChainError::BadBlockSize)?;
        Ok(Self {
            block,
            exponent_layout: exponent_band_layout(header.sample_rate, block)?,
            noise_layout: noise_band_layout(header.sample_rate, block)?,
        })
    }

    /// The long-block transform size.
    pub fn block_size(&self) -> BlockSize {
        self.block
    }

    /// The derived exponent/quantization-band partition.
    pub fn exponent_layout(&self) -> &QuantBandLayout {
        &self.exponent_layout
    }

    /// The derived noise/high-band grid.
    pub fn noise_layout(&self) -> &QuantBandLayout {
        &self.noise_layout
    }

    /// Number of exponent bands — the length the per-stream
    /// exponent-index vector must have.
    pub fn exponent_band_count(&self) -> usize {
        self.exponent_layout.band_count()
    }

    /// The ladder-derived `Q[d]` weight vector for a per-band
    /// exponent-index vector (one index per exponent band).
    ///
    /// # Errors
    ///
    /// * [`WireChainError::BandCountMismatch`] if the vector length
    ///   differs from [`WireBlockConfig::exponent_band_count`].
    /// * [`WireChainError::GainLadder`] for an out-of-ladder index.
    pub fn weights(&self, exponent_indices: &[u8]) -> Result<Vec<f64>, WireChainError> {
        if exponent_indices.len() != self.exponent_band_count() {
            return Err(WireChainError::BandCountMismatch {
                expected: self.exponent_band_count(),
                got: exponent_indices.len(),
            });
        }
        Ok(band_weights(exponent_indices)?)
    }

    /// Assemble the §8 single-channel **encoder** block chain over
    /// this configuration (real partition + ladder weights; step and
    /// entropy split remain the documented `[GAP]` inputs).
    ///
    /// # Errors
    ///
    /// Propagates the stage constructors' validation failures.
    pub fn channel_encoder(
        &self,
        exponent_indices: &[u8],
        step: OverallStepSize,
        split: u32,
    ) -> Result<ChannelEncoder, WireChainError> {
        let weights = self.weights(exponent_indices)?;
        let m = u32::from(self.block.samples());
        let analysis = Analysis::new(self.block, WindowPair::orthogonal_sine(self.block))
            .expect("window pair built for the same block size");
        let quant = QuantStage::new(self.block, &self.exponent_layout, &weights, step)
            .map_err(WireChainError::Quant)?;
        let spectral = SpectralEncode::new(
            Partition::new(split, m, false).expect("split validated against M by caller contract"),
        );
        ChannelEncoder::new(analysis, quant, spectral).map_err(WireChainError::EncodeAssembly)
    }

    /// Assemble the §8 single-channel **decoder** block chain over
    /// this configuration — the mirror of
    /// [`WireBlockConfig::channel_encoder`], built from the same
    /// staged-data geometry. `plan` carries the per-band coding
    /// policy over the **exponent** partition; `None` means all-coded.
    ///
    /// # Errors
    ///
    /// Propagates the stage constructors' validation failures.
    pub fn channel_decoder(
        &self,
        exponent_indices: &[u8],
        step: OverallStepSize,
        split: u32,
        plan: Option<BandPlan>,
    ) -> Result<ChannelDecoder, WireChainError> {
        let weights = self.weights(exponent_indices)?;
        let m = u32::from(self.block.samples());
        let spectral = SpectralDecode::new(
            Partition::new(split, m, false).expect("split validated against M by caller contract"),
        );
        let dequant = DequantStage::new(self.block, &self.exponent_layout, &weights, step)
            .map_err(WireChainError::Dequant)?;
        let plan = plan.unwrap_or_else(|| {
            BandPlan::new(vec![BandPolicy::Coded; self.exponent_layout.band_count()])
        });
        let noise = NoiseFiller::new(plan, self.exponent_layout.clone())
            .map_err(WireChainError::NoiseFill)?;
        let synthesis = Synthesis::new(self.block, WindowPair::orthogonal_sine(self.block))
            .expect("window pair built for the same block size");
        ChannelDecoder::new(spectral, dequant, noise, synthesis)
            .map_err(WireChainError::DecodeAssembly)
    }

    /// Like [`WireBlockConfig::channel_decoder`], but with the noise
    /// filler running over the **staged noise/high-band grid**
    /// (the octave-seed partition) instead of the exponent partition
    /// — the grid the staged provenance associates with noise
    /// substitution / high-band gain. `noise_plan` carries one policy
    /// per noise band ([`WireBlockConfig::noise_layout`] order).
    ///
    /// # Errors
    ///
    /// Propagates the stage constructors' validation failures
    /// ([`WireChainError::NoiseFill`] if the plan's band count differs
    /// from the grid's).
    pub fn channel_decoder_with_noise_grid(
        &self,
        exponent_indices: &[u8],
        step: OverallStepSize,
        split: u32,
        noise_plan: BandPlan,
    ) -> Result<ChannelDecoder, WireChainError> {
        let weights = self.weights(exponent_indices)?;
        let m = u32::from(self.block.samples());
        let spectral = SpectralDecode::new(
            Partition::new(split, m, false).expect("split validated against M by caller contract"),
        );
        let dequant = DequantStage::new(self.block, &self.exponent_layout, &weights, step)
            .map_err(WireChainError::Dequant)?;
        let noise = NoiseFiller::new(noise_plan, self.noise_layout.clone())
            .map_err(WireChainError::NoiseFill)?;
        let synthesis = Synthesis::new(self.block, WindowPair::orthogonal_sine(self.block))
            .expect("window pair built for the same block size");
        ChannelDecoder::new(spectral, dequant, noise, synthesis)
            .map_err(WireChainError::DecodeAssembly)
    }

    /// Assemble the §8 **two-channel encoder** chain over this
    /// configuration: two [`WireBlockConfig::channel_encoder`] chains
    /// (each channel with its own per-band exponent profile, as real
    /// streams carry) behind the §5 sum/difference fold.
    ///
    /// # Errors
    ///
    /// Propagates the per-channel constructors' failures; the stereo
    /// cross-check cannot fail (both channels share this config's
    /// block size).
    pub fn stereo_encoder(
        &self,
        exponent_indices_ch0: &[u8],
        exponent_indices_ch1: &[u8],
        step: OverallStepSize,
        split: u32,
    ) -> Result<StereoEncoder, WireChainError> {
        let ch0 = self.channel_encoder(exponent_indices_ch0, step, split)?;
        let ch1 = self.channel_encoder(exponent_indices_ch1, step, split)?;
        StereoEncoder::new(ch0, ch1).map_err(WireChainError::StereoAssembly)
    }

    /// Assemble the §8 **two-channel decoder** chain — the mirror of
    /// [`WireBlockConfig::stereo_encoder`], with the §8 inverse
    /// sum/difference fold gated per block by the (still-`[GAP]`)
    /// channel-mode flag at decode time.
    ///
    /// # Errors
    ///
    /// Propagates the per-channel constructors' failures.
    pub fn stereo_decoder(
        &self,
        exponent_indices_ch0: &[u8],
        exponent_indices_ch1: &[u8],
        step: OverallStepSize,
        split: u32,
    ) -> Result<StereoDecoder, WireChainError> {
        let ch0 = self.channel_decoder(exponent_indices_ch0, step, split, None)?;
        let ch1 = self.channel_decoder(exponent_indices_ch1, step, split, None)?;
        StereoDecoder::new(ch0, ch1).map_err(WireChainError::StereoAssembly)
    }
}

/// Sample-rate threshold of the staged decode-class selection rule
/// (`provenance/02` §4b): below it the class is pinned to 3.
pub const DECODE_CLASS_RATE_THRESHOLD_HZ: u32 = 32_000;

/// Where a stream's rate float falls relative to the two staged
/// class-branch thresholds
/// ([`crate::wire_tables::CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD`] =
/// 0.72 and
/// [`crate::wire_tables::CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD`] =
/// 1.16) — the typed carrier of the staged comparison's *operands*.
///
/// The staged trace pins the constants, their roles (0.72 is the
/// class-1 branch, 1.16 the class-2 branch), and the bounds of the
/// float axis; it deliberately does **not** pin the branch
/// *directions* — which side of each threshold selects which class —
/// so this enum names the three axis regions after the thresholds
/// that delimit them, never after a class outcome. Boundary
/// inclusivity (the region an input exactly equal to a threshold
/// lands in) is likewise unstaged; this realization puts each exact
/// threshold value in the region above it, a documented realization
/// detail affecting only bit-exact-threshold inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RateFloatRegion {
    /// Clamped rate float strictly below the class-1 branch
    /// threshold (0.72).
    BelowClass1Threshold,
    /// Clamped rate float in `0.72..1.16` — at or above the class-1
    /// branch threshold and strictly below the class-2 branch
    /// threshold.
    BetweenThresholds,
    /// Clamped rate float at or above the class-2 branch threshold
    /// (1.16).
    AtOrAboveClass2Threshold,
}

/// Saturate a per-stream rate float into the staged selector axis
/// `[0.125, 1.6]`
/// ([`crate::wire_tables::CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND`] ..
/// [`crate::wire_tables::CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND`]).
///
/// The staged roles name the two outer constants the float's "lower
/// bound" / "upper bound"; this helper realizes them as a saturating
/// clamp. A `NaN` input (impossible from real stream config; purely
/// defensive) normalizes to the lower bound. Clamping never moves an
/// input across a branch threshold, so
/// [`rate_float_region`]`(x) == rate_float_region(clamp_rate_float(x))`
/// for every non-`NaN` input.
pub fn clamp_rate_float(rate_float: f32) -> f32 {
    if rate_float.is_nan() {
        wire_tables::CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND
    } else {
        rate_float.clamp(
            wire_tables::CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND,
            wire_tables::CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND,
        )
    }
}

/// Locate a per-stream rate float on the staged selector axis: clamp
/// to the staged bounds, then partition by the two staged branch
/// thresholds (see [`RateFloatRegion`] for the boundary-side
/// realization note).
pub fn rate_float_region(rate_float: f32) -> RateFloatRegion {
    let x = clamp_rate_float(rate_float);
    if x < wire_tables::CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD {
        RateFloatRegion::BelowClass1Threshold
    } else if x < wire_tables::CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD {
        RateFloatRegion::BetweenThresholds
    } else {
        RateFloatRegion::AtOrAboveClass2Threshold
    }
}

/// The staged decode-class selection outcome for one stream.
///
/// The staged rule (`provenance/02` §4b + the staged threshold
/// extraction): the class **defaults to 3** and moves to 1 or 2 only
/// when `sample_rate >=` 32 kHz **and** a per-stream
/// bitrate/quality float compares against the staged branch
/// thresholds (0.72 for the class-1 branch, 1.16 for the class-2
/// branch, float axis bounded to `[0.125, 1.6]`). Low-rate streams
/// are fully pinned; high-rate streams resolve to a typed
/// [`RateFloatRegion`], and the remaining `[GAP]` is exactly the
/// region → class mapping (the two comparisons' branch directions,
/// including whether the between-thresholds region keeps the
/// class-3 default), which the caller resolves by black-box
/// observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeClassSelection {
    /// The rule pins the class outright (`sample_rate <` 32 kHz →
    /// class 3, primary descriptor; an alt-configured stream maps
    /// the same class through
    /// [`CoefDecodeMode::from_class_and_variant`]).
    Pinned(CoefDecodeMode),
    /// `sample_rate >=` 32 kHz: the staged comparisons place the
    /// stream's rate float in this region of the selector axis; the
    /// region → class branch directions are the unstaged residual.
    BitrateGated {
        /// Where the (clamped) rate float sits relative to the two
        /// staged branch thresholds.
        region: RateFloatRegion,
    },
}

/// Apply the staged §4b decode-class rule: the sample-rate gate, and
/// past it the staged threshold comparison operands.
///
/// `rate_float` is the per-stream bitrate/quality scalar the staged
/// rule compares against the thresholds. Its init formula is **not**
/// staged (a documented gap): the caller supplies the value (for a
/// sub-32-kHz stream it is ignored — the class pins to 3 before the
/// comparison is reached).
pub fn select_decode_class(sample_rate: u32, rate_float: f32) -> DecodeClassSelection {
    if sample_rate >= DECODE_CLASS_RATE_THRESHOLD_HZ {
        DecodeClassSelection::BitrateGated {
            region: rate_float_region(rate_float),
        }
    } else {
        DecodeClassSelection::Pinned(CoefDecodeMode::Mode3)
    }
}

/// Header-derived **wire frame codec**: everything needed to emit and
/// parse whole frames at the bit level — the staged frame/block
/// layout ([`crate::frame_bits`]) instantiated with the real VLCs and
/// the header-derived geometry.
///
/// Derivations (all staged):
///
/// * S1 width = `byte_offset_bits` from the staged formula
///   ([`SetupParams`]).
/// * Escape literal widths: the staged §4e correction pins their
///   *sources* — the escape reads its literal run at the frame
///   side-field width (`ctx+0x35c`) and its literal level at
///   `byte_offset_bits` (`ctx+0x88`). The side-field width itself has
///   no staged formula (`[GAP]`), so the caller threads in an
///   observed value and both dependent widths follow.
/// * Scale symbols per block = the derived exponent band count.
///
/// Still caller-supplied (typed gaps): the coefficient decode mode
/// when [`select_decode_class`] returns the bitrate-gated case, the
/// gain-delta count per block (sub-stream count semantics unstaged;
/// defaults to 1), and the envelope gate.
#[derive(Debug, Clone)]
pub struct WireFrameCodec {
    config: WireBlockConfig,
    widths: FrameFieldWidths,
    escape: EscapeWidths,
    coef_vlc: CoefVlc,
    gain_vlc: GainVlc,
    scale_vlc: ScaleVlc,
    channels: u8,
    gain_count: usize,
    envelope_coded: bool,
}

impl WireFrameCodec {
    /// Derive the codec from a parsed header, a resolved coefficient
    /// decode mode, and the observed frame side-field width.
    ///
    /// # Errors
    ///
    /// * [`WireChainError::UnsupportedChannels`] outside 1–2 channels.
    /// * [`WireChainError::BadFieldWidth`] when a derived width falls
    ///   outside `1..=32`.
    /// * Propagated geometry/VLC construction failures.
    pub fn from_header(
        header: &WmaHeader,
        mode: CoefDecodeMode,
        side_field_bits: u8,
    ) -> Result<Self, WireChainError> {
        if header.channels == 0 || header.channels > 2 {
            return Err(WireChainError::UnsupportedChannels {
                channels: header.channels,
            });
        }
        let config = WireBlockConfig::from_header(header)?;
        let setup = SetupParams::from_header(header);
        let byte_offset_bits = u8::try_from(setup.byte_offset_bits)
            .ok()
            .filter(|&b| (1..=32).contains(&b))
            .ok_or(WireChainError::BadFieldWidth {
                field: "byte_offset_bits",
                value: setup.byte_offset_bits,
            })?;
        let widths = FrameFieldWidths::new(byte_offset_bits, side_field_bits).map_err(|_| {
            WireChainError::BadFieldWidth {
                field: "side_field_bits",
                value: u32::from(side_field_bits),
            }
        })?;
        // §4e: escape literal run at the side-field width, literal
        // level at byte_offset_bits.
        let escape = EscapeWidths::new(side_field_bits, byte_offset_bits).map_err(|_| {
            WireChainError::BadFieldWidth {
                field: "escape widths",
                value: u32::from(side_field_bits),
            }
        })?;
        Ok(Self {
            config,
            widths,
            escape,
            coef_vlc: CoefVlc::new(mode)?,
            gain_vlc: GainVlc::new(),
            scale_vlc: ScaleVlc::new(),
            channels: header.channels,
            gain_count: 1,
            envelope_coded: true,
        })
    }

    /// Derive the codec with the decode class resolved by the staged
    /// §4b rule where the rule **pins** it: below the 32 kHz gate the
    /// class is 3 (the rule's retained default), realized here as the
    /// primary class-3 descriptor — an alt-configured stream maps the
    /// same class through [`CoefDecodeMode::from_class_and_variant`]
    /// and [`WireFrameCodec::from_header`].
    ///
    /// `rate_float` is the per-stream bitrate/quality scalar of the
    /// staged threshold comparison (its init formula is a documented
    /// gap — caller-observed); it is consumed only to type the error
    /// when the class is *not* pinned.
    ///
    /// # Errors
    ///
    /// * [`WireChainError::ClassNotPinned`] when the stream sits at
    ///   or above the 32 kHz gate: the staged constants place the
    ///   rate float in a typed [`RateFloatRegion`], but the
    ///   region → class branch directions are unstaged, so the caller
    ///   resolves the class and calls
    ///   [`WireFrameCodec::from_header`].
    /// * Everything [`WireFrameCodec::from_header`] can raise.
    pub fn from_header_pinned_class(
        header: &WmaHeader,
        rate_float: f32,
        side_field_bits: u8,
    ) -> Result<Self, WireChainError> {
        match select_decode_class(header.sample_rate, rate_float) {
            DecodeClassSelection::Pinned(mode) => Self::from_header(header, mode, side_field_bits),
            DecodeClassSelection::BitrateGated { region } => Err(WireChainError::ClassNotPinned {
                sample_rate: header.sample_rate,
                region,
            }),
        }
    }

    /// Override the gain-delta count per block (sub-stream count
    /// semantics are unstaged; the default is 1).
    #[must_use]
    pub fn with_gain_count(mut self, gain_count: usize) -> Self {
        self.gain_count = gain_count;
        self
    }

    /// Override the envelope gate (B4/B5 presence; staged as a config
    /// flag, default on).
    #[must_use]
    pub fn with_envelope_coded(mut self, envelope_coded: bool) -> Self {
        self.envelope_coded = envelope_coded;
        self
    }

    /// The derived block-geometry configuration.
    pub fn config(&self) -> &WireBlockConfig {
        &self.config
    }

    /// The derived frame header field widths.
    pub fn widths(&self) -> &FrameFieldWidths {
        &self.widths
    }

    /// The derived escape literal widths.
    pub fn escape_widths(&self) -> EscapeWidths {
        self.escape
    }

    /// The coefficient decode mode in force.
    pub fn mode(&self) -> CoefDecodeMode {
        self.coef_vlc.mode()
    }

    /// The per-block layout plan this codec parses and emits.
    pub fn plan(&self) -> BlockPlan<'_> {
        BlockPlan {
            coef_vlc: &self.coef_vlc,
            gain_vlc: &self.gain_vlc,
            scale_vlc: &self.scale_vlc,
            escape: self.escape,
            channels: self.channels,
            envelope_coded: self.envelope_coded,
            gain_count: self.gain_count,
            scale_count: self.config.exponent_band_count(),
            coef_count: usize::from(self.config.block_size().samples()),
        }
    }

    /// Emit one frame as bytes, returning `(bytes, bit_len)` (frames
    /// are bit-packed; the trailing byte may be partial).
    ///
    /// # Errors
    ///
    /// Propagates [`FrameBitsError`] through
    /// [`WireChainError::FrameBits`].
    pub fn encode_frame(&self, frame: &WireFrame) -> Result<(Vec<u8>, usize), WireChainError> {
        let blocks = frame
            .channel_blocks
            .first()
            .map(Vec::len)
            .unwrap_or_default();
        let plans = vec![self.plan(); blocks];
        let mut writer = BitWriter::new();
        write_frame(frame, &self.widths, &plans, &mut writer)?;
        let bit_len = writer.bit_len();
        Ok((writer.into_bytes(), bit_len))
    }

    /// Emit one packet (superframe) of back-to-back bit-contiguous
    /// frames per the staged §1 rule, as `(bytes, bit_len)`. All
    /// frames must share one uniform block count.
    ///
    /// # Errors
    ///
    /// Propagates [`FrameBitsError`] through
    /// [`WireChainError::FrameBits`].
    pub fn encode_packet(&self, frames: &[WireFrame]) -> Result<(Vec<u8>, usize), WireChainError> {
        let blocks = frames
            .first()
            .and_then(|f| f.channel_blocks.first())
            .map(Vec::len)
            .unwrap_or_default();
        let plans = vec![self.plan(); blocks];
        let mut writer = BitWriter::new();
        write_packet(frames, &self.widths, &plans, &mut writer)?;
        let bit_len = writer.bit_len();
        Ok((writer.into_bytes(), bit_len))
    }

    /// Parse one packet of `frame_count` frames of
    /// `blocks_per_channel` uniform blocks each (both counts are
    /// runtime state per the staged trace — the caller owns them).
    ///
    /// # Errors
    ///
    /// Propagates [`FrameBitsError`] through
    /// [`WireChainError::FrameBits`].
    pub fn decode_packet(
        &self,
        bytes: &[u8],
        bit_len: usize,
        frame_count: usize,
        blocks_per_channel: usize,
    ) -> Result<Vec<WireFrame>, WireChainError> {
        let plans = vec![self.plan(); blocks_per_channel];
        let mut reader = BitReader::with_bit_len(bytes, bit_len);
        Ok(read_packet(&self.widths, &plans, frame_count, &mut reader)?)
    }

    /// Parse one frame of `blocks_per_channel` uniform blocks from
    /// `bytes` (`bit_len` valid bits).
    ///
    /// # Errors
    ///
    /// Propagates [`FrameBitsError`] through
    /// [`WireChainError::FrameBits`].
    pub fn decode_frame(
        &self,
        bytes: &[u8],
        bit_len: usize,
        blocks_per_channel: usize,
    ) -> Result<WireFrame, WireChainError> {
        let plans = vec![self.plan(); blocks_per_channel];
        let mut reader = BitReader::with_bit_len(bytes, bit_len);
        Ok(read_frame(&self.widths, &plans, &mut reader)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::channel_decision::ChannelMode;
    use crate::frame::FrameDecoder;
    use crate::frame::StereoFrameDecoder;
    use crate::frame_encode::{FrameEncoder, StereoFrameEncoder};
    use crate::header::Version;

    fn header_44k() -> WmaHeader {
        // v2, 44.1 kHz stereo header: frame_length_bits 11 -> S2048.
        WmaHeader::parse(Version::V2, 44_100, 2, 128_000, 0, &[0; 6]).unwrap()
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

    #[test]
    fn config_derives_the_real_geometry_from_the_header() {
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        assert_eq!(cfg.block_size(), BlockSize::S2048);
        // 44.1 kHz / 2048: the full 25-band critical partition and the
        // 10-band octave grid (pinned in exponent_bands tests).
        assert_eq!(cfg.exponent_band_count(), 25);
        assert_eq!(cfg.noise_layout().band_count(), 10);
        assert_eq!(cfg.exponent_layout().total_coeffs(), 2048);
    }

    #[test]
    fn weight_vector_length_is_enforced() {
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        assert_eq!(
            cfg.weights(&[80; 3]),
            Err(WireChainError::BandCountMismatch {
                expected: 25,
                got: 3,
            })
        );
        assert!(matches!(
            cfg.weights(&[200; 25]),
            Err(WireChainError::GainLadder(_))
        ));
        let w = cfg.weights(&[80; 25]).unwrap();
        assert_eq!(w.len(), 25);
        assert!(w.iter().all(|&x| x == 1.0), "flat indices -> flat weights");
    }

    #[test]
    fn low_rate_header_derives_the_collapsed_partition() {
        let h = WmaHeader::parse(Version::V1, 8_000, 1, 32_000, 0, &[0; 4]).unwrap();
        let cfg = WireBlockConfig::from_header(&h).unwrap();
        assert_eq!(cfg.block_size(), BlockSize::S512);
        assert_eq!(cfg.exponent_band_count(), 18);
    }

    #[test]
    fn real_geometry_chain_round_trips_pcm() {
        // The full §8 loop over staged data: header -> real partition
        // -> ladder weights -> encode -> decode -> PCM, within the §4
        // quantizer bound. A sloped exponent profile (loud lows,
        // -1.25 dB per band) exercises distinct per-band scales.
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        let m = usize::from(cfg.block_size().samples());
        let indices: Vec<u8> = (0..cfg.exponent_band_count())
            .map(|d| u8::try_from(100 - d).unwrap())
            .collect();
        let step = OverallStepSize::new(1e-3).unwrap();
        let split = u32::try_from(m).unwrap(); // all-levels mode: exact symbol carriage

        let mut fe = FrameEncoder::new(cfg.channel_encoder(&indices, step, split).unwrap());
        let mut fd = FrameDecoder::new(cfg.channel_decoder(&indices, step, split, None).unwrap());

        let blocks = 3usize;
        let x = pseudo_random(blocks * m, 388);
        let band_count = cfg.exponent_band_count();
        let params: Vec<_> = fe
            .encode_frame(&x)
            .unwrap()
            .into_iter()
            .map(|b| b.into_block_params(band_count))
            .collect();
        let mut pcm = fd.decode_frame(&params).unwrap();
        // Drain the chain's M-sample latency.
        let tail_params = vec![fe.flush().unwrap().into_block_params(band_count)];
        pcm.extend(fd.decode_frame(&tail_params).unwrap());

        // decode(encode(x)) reproduces x after M-sample latency. The
        // error bound: q*scale reconstruction is within scale/2 per
        // coefficient; the weakest band sits 24 ladder steps (30 dB)
        // below the loudest, so the worst per-band scale is
        // step / 10^(-1.5) — comfortably bounded by 0.05 here.
        for (t, (&want, &got)) in x.iter().zip(pcm.iter().skip(m)).enumerate() {
            assert!((want - got).abs() < 0.05, "sample {t}: {want} vs {got}");
        }
    }

    #[test]
    fn finer_step_tightens_the_real_chain_bound() {
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        let m = usize::from(cfg.block_size().samples());
        let indices = vec![90u8; cfg.exponent_band_count()];
        let split = u32::try_from(m).unwrap();
        let x = pseudo_random(2 * m, 389);

        let mut worst = Vec::new();
        for step in [1e-2, 1e-4] {
            let step = OverallStepSize::new(step).unwrap();
            let mut fe = FrameEncoder::new(cfg.channel_encoder(&indices, step, split).unwrap());
            let mut fd =
                FrameDecoder::new(cfg.channel_decoder(&indices, step, split, None).unwrap());
            let band_count = cfg.exponent_band_count();
            let mut params: Vec<_> = fe
                .encode_frame(&x)
                .unwrap()
                .into_iter()
                .map(|b| b.into_block_params(band_count))
                .collect();
            params.push(fe.flush().unwrap().into_block_params(band_count));
            let pcm = fd.decode_frame(&params).unwrap();
            let err = x
                .iter()
                .zip(pcm.iter().skip(m))
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f64, f64::max);
            worst.push(err);
        }
        assert!(
            worst[1] < worst[0] / 10.0,
            "bound must shrink with the step: {worst:?}"
        );
    }
    #[test]
    fn stereo_real_geometry_round_trips_both_modes() {
        // The two-channel §8 loop over staged data, per-channel
        // exponent profiles, both channel modes.
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        let m = usize::from(cfg.block_size().samples());
        let n = cfg.exponent_band_count();
        let idx0 = vec![95u8; n];
        let idx1: Vec<u8> = (0..n).map(|d| u8::try_from(100 - d).unwrap()).collect();
        let step = OverallStepSize::new(1e-3).unwrap();
        let split = u32::try_from(m).unwrap();

        for mode in [ChannelMode::Independent, ChannelMode::SumDifference] {
            let mut fe =
                StereoFrameEncoder::new(cfg.stereo_encoder(&idx0, &idx1, step, split).unwrap());
            let mut fd =
                StereoFrameDecoder::new(cfg.stereo_decoder(&idx0, &idx1, step, split).unwrap());
            let blocks = 2usize;
            let l = pseudo_random(blocks * m, 400);
            let r = pseudo_random(blocks * m, 401);
            let modes = vec![mode; blocks];
            let mut params: Vec<_> = fe
                .encode_frame(&l, &r, &modes)
                .unwrap()
                .into_iter()
                .map(|b| b.into_stereo_block_params(n))
                .collect();
            params.push(fe.flush(mode).unwrap().into_stereo_block_params(n));
            let out = fd.decode_frame(&params).unwrap();
            for i in 0..blocks * m {
                assert!((out.left[m + i] - l[i]).abs() < 0.05, "{mode:?} L i={i}");
                assert!((out.right[m + i] - r[i]).abs() < 0.05, "{mode:?} R i={i}");
            }
        }
    }

    #[test]
    fn noise_grid_decoder_matches_the_exponent_grid_when_all_coded() {
        // With every band coded the noise filler is a no-op, so the
        // two grid choices must produce identical PCM from identical
        // symbols — pinning that the octave-grid wiring changes only
        // the noise geometry, not the coded path.
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        let m = usize::from(cfg.block_size().samples());
        let n = cfg.exponent_band_count();
        let indices = vec![90u8; n];
        let step = OverallStepSize::new(1e-3).unwrap();
        let split = u32::try_from(m).unwrap();
        let noise_bands = cfg.noise_layout().band_count();

        let mut fe = FrameEncoder::new(cfg.channel_encoder(&indices, step, split).unwrap());
        let x = pseudo_random(2 * m, 402);
        let blocks_exp: Vec<_> = fe.encode_frame(&x).unwrap();

        let mut fd_exp =
            FrameDecoder::new(cfg.channel_decoder(&indices, step, split, None).unwrap());
        let mut fd_noise = FrameDecoder::new(
            cfg.channel_decoder_with_noise_grid(
                &indices,
                step,
                split,
                BandPlan::new(vec![BandPolicy::Coded; noise_bands]),
            )
            .unwrap(),
        );
        let params_exp: Vec<_> = blocks_exp
            .iter()
            .cloned()
            .map(|b| b.into_block_params(n))
            .collect();
        let params_noise: Vec<_> = blocks_exp
            .into_iter()
            .map(|b| b.into_block_params(noise_bands))
            .collect();
        let a = fd_exp.decode_frame(&params_exp).unwrap();
        let b = fd_noise.decode_frame(&params_noise).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn decode_class_rule_matches_the_staged_fork() {
        // provenance/02 §4b: default class 3; the 32 kHz gate opens
        // the bitrate fork, now carrying the staged-threshold region.
        for sr in [8_000, 11_025, 16_000, 22_050, 24_000, 31_999] {
            for rate_float in [0.0, 0.5, 1.0, 100.0] {
                assert_eq!(
                    select_decode_class(sr, rate_float),
                    DecodeClassSelection::Pinned(CoefDecodeMode::Mode3),
                    "{sr} @ {rate_float}"
                );
            }
        }
        for sr in [32_000, 44_100, 48_000] {
            assert_eq!(
                select_decode_class(sr, 0.5),
                DecodeClassSelection::BitrateGated {
                    region: RateFloatRegion::BelowClass1Threshold,
                },
                "{sr}"
            );
            assert_eq!(
                select_decode_class(sr, 1.0),
                DecodeClassSelection::BitrateGated {
                    region: RateFloatRegion::BetweenThresholds,
                },
                "{sr}"
            );
            assert_eq!(
                select_decode_class(sr, 1.45),
                DecodeClassSelection::BitrateGated {
                    region: RateFloatRegion::AtOrAboveClass2Threshold,
                },
                "{sr}"
            );
        }
    }

    #[test]
    fn clamp_saturates_into_the_staged_axis() {
        use crate::wire_tables::{
            CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND, CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND,
        };
        // Inside the axis: identity, bit-for-bit.
        for x in [0.125_f32, 0.3, 0.72, 1.0, 1.16, 1.5, 1.6] {
            assert_eq!(clamp_rate_float(x).to_bits(), x.to_bits(), "{x}");
        }
        // Outside: saturate to the staged bounds.
        for x in [-1.0_f32, 0.0, 0.1249, f32::NEG_INFINITY] {
            assert_eq!(
                clamp_rate_float(x).to_bits(),
                CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND.to_bits(),
                "{x}"
            );
        }
        for x in [1.6001_f32, 2.9, 100.0, f32::INFINITY] {
            assert_eq!(
                clamp_rate_float(x).to_bits(),
                CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND.to_bits(),
                "{x}"
            );
        }
        // Defensive NaN normalization (documented realization detail).
        assert_eq!(
            clamp_rate_float(f32::NAN).to_bits(),
            CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND.to_bits()
        );
    }

    #[test]
    fn region_partition_follows_the_staged_thresholds() {
        use crate::wire_tables::{
            CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD, CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD,
        };
        // Interior representatives of the three regions.
        for x in [0.125_f32, 0.3, 0.5, 0.719] {
            assert_eq!(
                rate_float_region(x),
                RateFloatRegion::BelowClass1Threshold,
                "{x}"
            );
        }
        for x in [0.7201_f32, 0.9, 1.0, 1.15] {
            assert_eq!(
                rate_float_region(x),
                RateFloatRegion::BetweenThresholds,
                "{x}"
            );
        }
        for x in [1.1601_f32, 1.45, 1.6] {
            assert_eq!(
                rate_float_region(x),
                RateFloatRegion::AtOrAboveClass2Threshold,
                "{x}"
            );
        }
        // The documented boundary-side realization: an input exactly
        // equal to a threshold lands in the region above it, and the
        // next representable f32 below lands under it.
        let c1 = CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD;
        let c2 = CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD;
        assert_eq!(rate_float_region(c1), RateFloatRegion::BetweenThresholds);
        assert_eq!(
            rate_float_region(f32::from_bits(c1.to_bits() - 1)),
            RateFloatRegion::BelowClass1Threshold
        );
        assert_eq!(
            rate_float_region(c2),
            RateFloatRegion::AtOrAboveClass2Threshold
        );
        assert_eq!(
            rate_float_region(f32::from_bits(c2.to_bits() - 1)),
            RateFloatRegion::BetweenThresholds
        );
    }

    #[test]
    fn region_is_clamp_stable_and_monotone() {
        // Clamping never moves an input across a branch threshold
        // (the thresholds sit strictly inside the staged bounds), so
        // the region of a raw input equals the region of its clamp;
        // and the region index is monotone non-decreasing along the
        // float axis.
        let rank = |r: RateFloatRegion| match r {
            RateFloatRegion::BelowClass1Threshold => 0,
            RateFloatRegion::BetweenThresholds => 1,
            RateFloatRegion::AtOrAboveClass2Threshold => 2,
        };
        let mut prev = 0;
        for i in 0..=4_000 {
            // Sweep -0.5 .. 3.5, comfortably past both bounds.
            let x = -0.5_f32 + (i as f32) * 0.001;
            let r = rate_float_region(x);
            assert_eq!(r, rate_float_region(clamp_rate_float(x)), "{x}");
            let k = rank(r);
            assert!(k >= prev, "region must be monotone along the axis: {x}");
            prev = k;
        }
    }

    #[test]
    fn pinned_class_codec_builds_below_the_gate_and_types_the_gap_above() {
        // Below 32 kHz the staged rule pins class 3: the codec builds
        // with the primary class-3 descriptor, no caller-resolved
        // mode needed.
        let low = WmaHeader::parse(Version::V1, 22_050, 1, 64_000, 0, &[0; 4]).unwrap();
        let codec = WireFrameCodec::from_header_pinned_class(&low, 0.9, 6).unwrap();
        assert_eq!(codec.mode(), CoefDecodeMode::Mode3);
        assert_eq!(codec.mode().class(), 3);
        assert!(!codec.mode().is_alt());

        // At/above the gate the class is bitrate-gated and the branch
        // directions are unstaged: the constructor refuses with the
        // typed region rather than guessing a class.
        let high = header_44k();
        assert_eq!(
            WireFrameCodec::from_header_pinned_class(&high, 1.45, 6).unwrap_err(),
            WireChainError::ClassNotPinned {
                sample_rate: 44_100,
                region: RateFloatRegion::AtOrAboveClass2Threshold,
            }
        );
        assert_eq!(
            WireFrameCodec::from_header_pinned_class(&high, 0.5, 6).unwrap_err(),
            WireChainError::ClassNotPinned {
                sample_rate: 44_100,
                region: RateFloatRegion::BelowClass1Threshold,
            }
        );
    }

    #[test]
    fn codec_derives_the_staged_widths_from_the_header() {
        let codec = WireFrameCodec::from_header(&header_44k(), CoefDecodeMode::Mode2, 6).unwrap();
        // bps = 128000 / (2 * 44100) = 1 (integer);
        // byte_offset_bits = floor(log2(1 * 2048 / 8)) + 2 = 10.
        assert_eq!(codec.widths(), &FrameFieldWidths::new(10, 6).unwrap());
        // §4e: escape run at the side-field width, escape level at
        // byte_offset_bits.
        assert_eq!(codec.escape_widths().run_bits, 6);
        assert_eq!(codec.escape_widths().level_bits, 10);
        assert_eq!(codec.mode(), CoefDecodeMode::Mode2);
        let plan = codec.plan();
        assert_eq!(plan.channels, 2);
        assert_eq!(plan.scale_count, 25);
        assert_eq!(plan.coef_count, 2048);
        assert_eq!(plan.gain_count, 1);
        assert!(plan.envelope_coded);
    }

    #[test]
    fn codec_rejects_unsupported_channel_counts() {
        let h = WmaHeader::parse(Version::V2, 44_100, 6, 128_000, 0, &[0; 6]).unwrap();
        assert_eq!(
            WireFrameCodec::from_header(&h, CoefDecodeMode::Mode2, 6).unwrap_err(),
            WireChainError::UnsupportedChannels { channels: 6 }
        );
    }

    #[test]
    fn mono_pcm_round_trips_through_the_wire_bit_layout() {
        // THE r390 milestone loop: PCM -> §8 encoder chain (real
        // geometry) -> quantized coefficients -> the staged frame bit
        // layout with the real mode-2 VLC (pairs, escapes, signs) ->
        // bytes -> frame_bits parse -> §8 decoder chain -> PCM.
        // 512 kb/s mono: bps = 11, so the staged formula gives
        // byte_offset_bits = floor(log2(11 * 2048 / 8)) + 2 = 13 and
        // the escape level literal spans 13 bits (max 8191).
        let header = WmaHeader::parse(Version::V2, 44_100, 1, 512_000, 0, &[0; 6]).unwrap();
        let codec = WireFrameCodec::from_header(&header, CoefDecodeMode::Mode2, 6).unwrap();
        let cfg = codec.config().clone();
        let m = usize::from(cfg.block_size().samples());
        let n = cfg.exponent_band_count();
        let indices = vec![80u8; n]; // ladder value 1024000/1024000: flat unit weights
        let step = OverallStepSize::new(5e-3).unwrap();
        let split = u32::try_from(m).unwrap(); // all-levels: EncodedBlock.levels is the full coef vector

        // Forward: PCM -> quantized coefficient vectors.
        let mut fe = FrameEncoder::new(cfg.channel_encoder(&indices, step, split).unwrap());
        let blocks = 2usize;
        let x: Vec<f64> = pseudo_random(blocks * m, 390)
            .into_iter()
            .map(|v| v * 0.5)
            .collect();
        let mut encoded = fe.encode_frame(&x).unwrap();
        encoded.push(fe.flush().unwrap());

        // Package as wire blocks (envelope/gain symbol streams carried
        // verbatim; their delta chaining is the documented GAP).
        let frame = WireFrame {
            header: crate::frame_bits::FrameHeaderFields {
                reservoir_offset: 100,
                side_field: 3,
                flag: false,
            },
            channel_blocks: vec![encoded
                .iter()
                .map(|b| {
                    assert!(b.pairs.is_empty(), "all-levels mode has no tail pairs");
                    crate::frame_bits::WireBlock {
                        header: 0x11,
                        gain_symbols: vec![18],
                        stereo_coupling: None,
                        envelope_base: 5,
                        scale_symbols: vec![60; n],
                        coefficients: b.levels.clone(),
                    }
                })
                .collect()],
        };

        // Wire: bits out, bits back, field-exact.
        let (bytes, bit_len) = codec.encode_frame(&frame).unwrap();
        let decoded = codec.decode_frame(&bytes, bit_len, blocks + 1).unwrap();
        assert_eq!(decoded, frame, "wire round trip must be field-exact");

        // Inverse: decoded coefficients -> PCM.
        let mut fd = FrameDecoder::new(cfg.channel_decoder(&indices, step, split, None).unwrap());
        let params: Vec<_> = decoded.channel_blocks[0]
            .iter()
            .map(|b| {
                crate::frame::BlockParams::new(b.coefficients.clone(), vec![], vec![Vec::new(); n])
            })
            .collect();
        let pcm = fd.decode_frame(&params).unwrap();
        for (t, (&want, &got)) in x.iter().zip(pcm.iter().skip(m)).enumerate() {
            assert!((want - got).abs() < 0.1, "sample {t}: {want} vs {got}");
        }
    }

    #[test]
    fn stereo_pcm_round_trips_through_the_wire_bit_layout() {
        // Two-channel wire loop (B3 stereo flag in every block),
        // independent channels, real mode-1 VLC with escapes.
        // 1024 kb/s stereo: bps = 11 -> byte_offset_bits = 13.
        let header = WmaHeader::parse(Version::V2, 44_100, 2, 1_024_000, 0, &[0; 6]).unwrap();
        let codec = WireFrameCodec::from_header(&header, CoefDecodeMode::Mode1, 6).unwrap();
        let cfg = codec.config().clone();
        let m = usize::from(cfg.block_size().samples());
        let n = cfg.exponent_band_count();
        let indices = vec![80u8; n];
        let step = OverallStepSize::new(5e-3).unwrap();
        let split = u32::try_from(m).unwrap();

        let soften = |v: Vec<f64>| v.into_iter().map(|s| s * 0.5).collect::<Vec<_>>();
        let inputs = [soften(pseudo_random(m, 391)), soften(pseudo_random(m, 392))];
        let mut frames_per_channel = Vec::new();
        let mut decoders = Vec::new();
        for x in &inputs {
            let mut fe = FrameEncoder::new(cfg.channel_encoder(&indices, step, split).unwrap());
            let mut encoded = fe.encode_frame(x).unwrap();
            encoded.push(fe.flush().unwrap());
            frames_per_channel.push(encoded);
            decoders.push(FrameDecoder::new(
                cfg.channel_decoder(&indices, step, split, None).unwrap(),
            ));
        }

        let frame = WireFrame {
            header: crate::frame_bits::FrameHeaderFields {
                reservoir_offset: 512,
                side_field: 63,
                flag: true,
            },
            channel_blocks: frames_per_channel
                .iter()
                .map(|blocks| {
                    blocks
                        .iter()
                        .map(|b| crate::frame_bits::WireBlock {
                            header: 0x22,
                            gain_symbols: vec![17],
                            stereo_coupling: Some(false),
                            envelope_base: 9,
                            scale_symbols: vec![60; n],
                            coefficients: b.levels.clone(),
                        })
                        .collect()
                })
                .collect(),
        };
        let (bytes, bit_len) = codec.encode_frame(&frame).unwrap();
        let decoded = codec.decode_frame(&bytes, bit_len, 2).unwrap();
        assert_eq!(decoded, frame);

        for (ch, x) in inputs.iter().enumerate() {
            let params: Vec<_> = decoded.channel_blocks[ch]
                .iter()
                .map(|b| {
                    crate::frame::BlockParams::new(
                        b.coefficients.clone(),
                        vec![],
                        vec![Vec::new(); n],
                    )
                })
                .collect();
            let pcm = decoders[ch].decode_frame(&params).unwrap();
            for (t, (&want, &got)) in x.iter().zip(pcm.iter().skip(m)).enumerate() {
                assert!(
                    (want - got).abs() < 0.1,
                    "ch {ch} sample {t}: {want} vs {got}"
                );
            }
        }
    }

    #[test]
    fn noise_grid_plan_length_is_enforced() {
        let cfg = WireBlockConfig::from_header(&header_44k()).unwrap();
        let m = usize::from(cfg.block_size().samples());
        let err = cfg
            .channel_decoder_with_noise_grid(
                &vec![90u8; cfg.exponent_band_count()],
                OverallStepSize::new(1e-3).unwrap(),
                u32::try_from(m).unwrap(),
                BandPlan::new(vec![BandPolicy::Coded; 3]),
            )
            .unwrap_err();
        assert!(matches!(err, WireChainError::NoiseFill(_)), "{err:?}");
    }
}

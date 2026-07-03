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
use crate::block::BlockSize;
use crate::decode::{AssemblyError, ChannelDecoder};
use crate::dequant::{DequantStage, InvalidDequant};
use crate::encode::{ChannelEncoder, EncodeAssemblyError};
use crate::entropy_mode::Partition;
use crate::exponent_bands::{exponent_band_layout, noise_band_layout, BandDeriveError};
use crate::gain_ladder::{band_weights, GainLadderError};
use crate::header::WmaHeader;
use crate::noisefill::{InvalidNoiseFill, NoiseFiller};
use crate::qband::QuantBandLayout;
use crate::quant::{InvalidQuant, QuantStage};
use crate::spectral::{SpectralDecode, SpectralEncode};
use crate::step_size::OverallStepSize;
use crate::synthesis::Synthesis;
use crate::window::WindowPair;

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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::FrameDecoder;
    use crate::frame_encode::FrameEncoder;
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
}

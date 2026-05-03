//! Pure-Rust **Windows Media Audio** decoder.
//!
//! Round 1 implements the **WMA v1 / v2 baseline** — the single-block-
//! size MDCT path that FFmpeg's `wmav1` / `wmav2` encoders produce by
//! default. The full envelope (bit reservoir, variable block lengths,
//! noise-coded high band) is documented and gated behind explicit
//! `Unsupported` errors; round-2 work item.
//!
//! WMA Pro (`0x0162`) and WMA Lossless (`0x0163`) are scheduled as a
//! whole-decoder addition in round 2 (see `pro.rs` / `lossless.rs`
//! placeholders below).
//!
//! Reference material:
//! * `audio/wma/wma-trace-reverse-engineering.md`
//! * `audio/wma/data/wma-bands-by-rate.md`
//! * `audio/wma/data/wma-spectral-vlc.md`
//!
//! All numeric tables are transcribed from the corresponding sidecar
//! markdown files (which themselves cite their FFmpeg / Microsoft
//! provenance); no third-party source is reused.

#![allow(clippy::needless_range_loop, clippy::excessive_precision)]

pub mod asf;
pub mod common;
pub mod tables;
pub mod v1;
pub mod v2;

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry, CodecTag,
    Decoder, Error, Frame, Packet, Result, SampleFormat,
};

use crate::common::{Version, WmaContext};

pub const CODEC_ID_V1: &str = "wmav1";
pub const CODEC_ID_V2: &str = "wmav2";

/// Register both decoders with the supplied [`CodecRegistry`]. WAVEFORMATEX
/// tags `0x0160` / `0x0161` are bound to the v1 / v2 codec ids.
pub fn register(reg: &mut CodecRegistry) {
    let cid_v1 = CodecId::new(CODEC_ID_V1);
    let dec_caps_v1 = CodecCapabilities::audio("wmav1_sw_dec")
        .with_lossy(true)
        .with_intra_only(true)
        .with_max_channels(2)
        .with_max_sample_rate(48_000);
    reg.register(
        CodecInfo::new(cid_v1.clone())
            .capabilities(dec_caps_v1)
            .decoder(make_v1_decoder)
            .tag(CodecTag::wave_format(0x0160)),
    );

    let cid_v2 = CodecId::new(CODEC_ID_V2);
    let dec_caps_v2 = CodecCapabilities::audio("wmav2_sw_dec")
        .with_lossy(true)
        .with_intra_only(true)
        .with_max_channels(2)
        .with_max_sample_rate(48_000);
    reg.register(
        CodecInfo::new(cid_v2)
            .capabilities(dec_caps_v2)
            .decoder(make_v2_decoder)
            .tag(CodecTag::wave_format(0x0161)),
    );
}

fn make_v1_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    let sample_rate = params
        .sample_rate
        .ok_or_else(|| Error::invalid("wmav1: sample_rate required"))?;
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("wmav1: channels required"))?;
    let bit_rate = params
        .bit_rate
        .ok_or_else(|| Error::invalid("wmav1: bit_rate required"))? as u32;
    let ctx = v1::make_context(sample_rate, channels, bit_rate, &params.extradata)?;
    Ok(Box::new(WmaDecoder {
        codec_id: CodecId::new(CODEC_ID_V1),
        ctx,
        version: Version::V1,
        pending: None,
        eof: false,
    }))
}

fn make_v2_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    let sample_rate = params
        .sample_rate
        .ok_or_else(|| Error::invalid("wmav2: sample_rate required"))?;
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("wmav2: channels required"))?;
    let bit_rate = params
        .bit_rate
        .ok_or_else(|| Error::invalid("wmav2: bit_rate required"))? as u32;
    let ctx = v2::make_context(sample_rate, channels, bit_rate, &params.extradata)?;
    Ok(Box::new(WmaDecoder {
        codec_id: CodecId::new(CODEC_ID_V2),
        ctx,
        version: Version::V2,
        pending: None,
        eof: false,
    }))
}

struct WmaDecoder {
    codec_id: CodecId,
    ctx: WmaContext,
    version: Version,
    pending: Option<Packet>,
    eof: bool,
}

impl Decoder for WmaDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.pending.is_some() {
            return Err(Error::other(
                "wma: receive_frame must be called before sending another packet",
            ));
        }
        self.pending = Some(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let pkt = match self.pending.take() {
            Some(p) => p,
            None => {
                return if self.eof {
                    Err(Error::Eof)
                } else {
                    Err(Error::NeedMore)
                };
            }
        };
        let mut out: Vec<Vec<f32>> = (0..self.ctx.channels as usize)
            .map(|_| Vec::new())
            .collect();
        self.ctx.decode_frame(&pkt.data, &mut out)?;

        let _ = self.version;
        let samples = out[0].len() as u32;
        // Output as F32P (planar float) — one Vec<u8> per channel.
        let data: Vec<Vec<u8>> = out
            .into_iter()
            .map(|plane| {
                let mut bytes = Vec::with_capacity(plane.len() * 4);
                for v in plane {
                    bytes.extend_from_slice(&v.to_le_bytes());
                }
                bytes
            })
            .collect();
        Ok(Frame::Audio(AudioFrame {
            samples,
            pts: pkt.pts,
            data,
        }))
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }
}

/// Sample format produced by both decoders.
pub const OUTPUT_SAMPLE_FORMAT: SampleFormat = SampleFormat::F32P;

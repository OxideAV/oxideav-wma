//! [`oxideav_core`] registration and the direct decoder factory —
//! the crate's dual API surface: [`register`] installs the codec into
//! a [`RuntimeContext`], and [`make_decoder`] builds a boxed
//! [`Decoder`] directly from [`CodecParameters`].
//!
//! ## Stream contract
//!
//! One [`Packet`] = one codec packet of exactly `block_align` bytes —
//! the payload unit the container carries for WMA (each reassembled
//! media object is exactly `nBlockAlign` bytes; the staged §1
//! measurement holds this on all 787 vendor packets). `block_align`
//! itself is not in [`CodecParameters`], so the decoder locks it from
//! the first packet's length.
//!
//! Because the §1 bit reservoir lets a frame's tail ride into the
//! next packet, the decoder holds each packet until its successor
//! arrives: output for packet `k` appears after `send_packet` of
//! packet `k + 1` (or at [`Decoder::flush`]). Output frames are
//! interleaved [`oxideav_core::SampleFormat::F32`] in the reference ±1.0
//! convention (`vendor_decode::ABS_SCALE`), one [`AudioFrame`] per
//! bitstream frame of `frame_length` samples per channel, plus the
//! synthesiser's final half-frame at flush.
//!
//! A frame that fails to parse (the known mono 22.05 kHz F1 anomaly,
//! or corrupt input) is replaced by silence of the declared length —
//! the §1 frame counts keep the timeline — and the synthesiser
//! resynchronises at the next packet's carry boundary.
//!
//! ## Registered ids and tags
//!
//! | id | wave tag | notes |
//! | -- | -------- | ----- |
//! | `wma1` | `0x0160` | WMA v1 |
//! | `wma2` | `0x0161` | WMA v2 |

use std::collections::VecDeque;

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecTag, Decoder, Encoder,
    Error, Frame, Packet, Result, RuntimeContext, SampleFormat, TimeBase,
};

use crate::header::Version;
use crate::packet::PacketAssembler;
use crate::stream_config::StreamConfig;
use crate::vendor_decode::BlockSynth;
use crate::vendor_frame::FrameParser;

/// Per-stream state derived once the first packet fixes `block_align`.
struct StreamState {
    cfg: StreamConfig,
    asm: PacketAssembler,
    parser: FrameParser,
    synth: BlockSynth,
    /// Index of the next packet whose frames have not been decoded.
    next_to_decode: usize,
    /// Bit cursor in the assembled stream.
    cursor: u64,
    /// Whether the previous packet ended as clean padding (zero-carry
    /// successor) rather than a mis-parse.
    clean_pad: bool,
}

/// A registered WMA decoder ([`Decoder`] impl over the vendor decode
/// chain: §1 packet assembly → §2–§4 frame parse → §5 mid/side +
/// calibrated dequantisation → variable-size lapped reconstruction).
pub struct WmaDecoder {
    codec_id: CodecId,
    version: Version,
    sample_rate: u32,
    channels: u8,
    bit_rate: u32,
    extradata: Vec<u8>,
    state: Option<StreamState>,
    out: VecDeque<AudioFrame>,
    eof: bool,
}

impl WmaDecoder {
    /// Build a decoder from codec parameters. `sample_rate`,
    /// `channels`, `bit_rate` and the version-length `extradata` are
    /// required (all four are wire-level configuration: the §0
    /// derivation needs them before the first bit can be read).
    pub fn from_params(params: &CodecParameters) -> Result<Self> {
        let version = version_for(params)
            .ok_or_else(|| Error::unsupported("oxideav-wma: not a WMA v1/v2 stream"))?;
        let sample_rate = params
            .sample_rate
            .ok_or_else(|| Error::invalid("oxideav-wma: sample_rate is required"))?;
        let channels = params
            .channels
            .filter(|&c| (1..=2).contains(&c))
            .ok_or_else(|| Error::invalid("oxideav-wma: 1 or 2 channels required"))?
            as u8;
        let bit_rate = params
            .bit_rate
            .filter(|&b| b > 0 && b <= u64::from(u32::MAX))
            .ok_or_else(|| Error::invalid("oxideav-wma: bit_rate is required"))?
            as u32;
        if params.extradata.len() < version.extradata_len() {
            return Err(Error::invalid(format!(
                "oxideav-wma: extradata must carry at least {} bytes for {version:?}",
                version.extradata_len()
            )));
        }
        Ok(Self {
            codec_id: params.codec_id.clone(),
            version,
            sample_rate,
            channels,
            bit_rate,
            extradata: params.extradata.clone(),
            state: None,
            out: VecDeque::new(),
            eof: false,
        })
    }

    /// Derive the stream state from the first packet's size.
    fn init_state(&self, block_align: usize) -> Result<StreamState> {
        let block_align = u16::try_from(block_align)
            .ok()
            .filter(|&b| b > 0)
            .ok_or_else(|| Error::invalid("oxideav-wma: packet size out of range"))?;
        // Validate the full header (extradata length, sample-rate
        // normalisation) before deriving the §0 configuration.
        let header = crate::header::WmaHeader::parse(
            self.version,
            self.sample_rate,
            self.channels,
            self.bit_rate,
            block_align,
            &self.extradata,
        )
        .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;
        let cfg = StreamConfig::derive(
            self.version,
            header.sample_rate,
            self.channels,
            self.bit_rate / 8,
            block_align,
            header.flags2,
        )
        .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;
        let asm = PacketAssembler::new(&cfg);
        let parser = FrameParser::new(&cfg, &[]);
        let synth = BlockSynth::new(&cfg);
        Ok(StreamState {
            cfg,
            asm,
            parser,
            synth,
            next_to_decode: 0,
            cursor: 0,
            clean_pad: false,
        })
    }

    /// Decode the frames of packet `k` (its successor's body — the
    /// carry landing zone — must already be assembled, or `last`
    /// must be set for the flush path).
    fn decode_packet(&mut self, k: usize, last: bool) {
        let st = self.state.as_mut().expect("state exists");
        let rec = st.asm.packets()[k];
        if st.cursor != rec.frames_start_bit() {
            // Padding skip or upstream mis-parse: resynchronise at
            // the §1 carry boundary, as the decoder does at every
            // packet header.
            st.cursor = rec.frames_start_bit();
            st.parser.raise_latch();
            if !st.clean_pad {
                st.synth.reset();
            }
        }
        let frame_len = usize::from(st.cfg.frame_length);
        let channels = usize::from(st.cfg.channels);
        let mut reader = st.asm.reader_at(st.cursor);
        let mut failed = false;
        for f in 0..rec.header.frame_count {
            if failed {
                break;
            }
            match st.parser.parse_frame(&mut reader) {
                Ok(frame) => {
                    let mut pcm: Vec<Vec<f64>> = vec![Vec::with_capacity(frame_len); channels];
                    for block in &frame.blocks {
                        for (ch, chan) in st.synth.block(block).into_iter().enumerate() {
                            pcm[ch].extend_from_slice(&chan);
                        }
                    }
                    self.out.push_back(interleave_f32(&pcm));
                }
                Err(_) => {
                    // §1 keeps the timeline: silence for the
                    // remaining declared frames.
                    let remaining = usize::from(rec.header.frame_count - f);
                    let silent = vec![vec![0.0f64; frame_len * remaining]; channels];
                    self.out.push_back(interleave_f32(&silent));
                    st.synth.reset();
                    failed = true;
                }
            }
        }
        st.cursor = reader.position() as u64;
        st.clean_pad = if last {
            false
        } else {
            let next = st.asm.packets()[k + 1];
            !failed && next.header.carry_bits == 0 && st.cursor <= next.body_start_bit
        };
        st.next_to_decode = k + 1;
    }
}

impl Decoder for WmaDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.eof {
            return Err(Error::invalid(
                "oxideav-wma: send_packet after flush (reset the decoder first)",
            ));
        }
        if self.state.is_none() {
            self.state = Some(self.init_state(packet.data.len())?);
        }
        let st = self.state.as_mut().expect("just initialised");
        let rec = st
            .asm
            .push_packet(&packet.data)
            .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;
        st.parser.note_body_start(rec.body_start_bit);
        if st.next_to_decode == 0 {
            st.cursor = st.asm.packets()[0].frames_start_bit();
        }
        // Decode every packet whose successor has arrived (the §1
        // reservoir carry means a frame can end inside the next
        // packet's body).
        while self
            .state
            .as_ref()
            .is_some_and(|s| s.next_to_decode + 1 < s.asm.packets().len())
        {
            let k = self.state.as_ref().expect("checked").next_to_decode;
            self.decode_packet(k, false);
        }
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        if let Some(f) = self.out.pop_front() {
            return Ok(Frame::Audio(f));
        }
        if self.eof {
            Err(Error::Eof)
        } else {
            Err(Error::NeedMore)
        }
    }

    fn flush(&mut self) -> Result<()> {
        if let Some(st) = self.state.as_ref() {
            let pending = st.next_to_decode < st.asm.packets().len();
            if pending {
                let k = self.state.as_ref().expect("checked").next_to_decode;
                self.decode_packet(k, true);
            }
            let st = self.state.as_mut().expect("state exists");
            let tail = st.synth.flush();
            if tail.iter().any(|c| !c.is_empty()) {
                self.out.push_back(interleave_f32(&tail));
            }
        }
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.state = None;
        self.out.clear();
        self.eof = false;
        Ok(())
    }
}

/// Interleave per-channel `f64` PCM into one F32 [`AudioFrame`].
fn interleave_f32(pcm: &[Vec<f64>]) -> AudioFrame {
    let channels = pcm.len().max(1);
    let samples = pcm.first().map_or(0, |c| c.len());
    let mut bytes = Vec::with_capacity(samples * channels * 4);
    for t in 0..samples {
        for chan in pcm {
            let v = chan.get(t).copied().unwrap_or(0.0) as f32;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    AudioFrame {
        samples: samples as u32,
        pts: None,
        data: vec![bytes],
    }
}

/// Resolve the WMA version from the codec id, falling back to the
/// container's wave tag.
fn version_for(params: &CodecParameters) -> Option<Version> {
    match params.codec_id.as_str() {
        "wma1" => return Some(Version::V1),
        "wma2" => return Some(Version::V2),
        _ => {}
    }
    match params.tag {
        Some(CodecTag::WaveFormat(t)) => Version::from_codec_id(t),
        _ => None,
    }
}

/// Direct decoder factory (the dual-API counterpart of the registry
/// path): build a boxed [`Decoder`] from codec parameters.
///
/// # Errors
///
/// [`Error::Unsupported`] when the parameters name neither `wma1` nor
/// `wma2` (by id or wave tag); [`Error::InvalidData`] when a required
/// audio parameter is missing.
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(WmaDecoder::from_params(params)?))
}

/// A registered WMA encoder ([`Encoder`] impl over the vendor encode
/// chain: forward lapped analysis → envelope/gain quantisation → the
/// §2–§4 frame emitter → §1 packetisation).
///
/// ## Stream contract
///
/// Input frames are interleaved [`SampleFormat::F32`] in the ±1.0
/// convention (the mirror of [`WmaDecoder`]'s output). Output packets
/// are `block_align`-byte §1 codec packets; because the §1 packetiser
/// derives every packet's P2/P3 from the finished frame layout, all
/// packets become available at [`Encoder::flush`].
///
/// `block_align` is chosen as eight average frames per packet — the
/// ratio the staged vendor-stream configurations themselves exhibit
/// — and is recoverable from any emitted packet's length (each is
/// exactly `block_align` bytes; a WAVEFORMATEX-carrying container
/// needs it as `nBlockAlign`).
///
/// `flags2` comes from the input parameters' extradata when one is
/// supplied (v1: bytes 2–3, v2: bytes 4–5), else defaults to
/// `0x000f` (VLC envelopes + bit reservoir + variable block length —
/// the staged vendor CBR configuration). Bit 0 must be set: the
/// §3.1 LSP envelope path's conversion tables are a staged gap, so
/// no LSP stream can be encoded.
pub struct WmaEncoder {
    codec_id: CodecId,
    params: CodecParameters,
    sample_rate: u32,
    channels: u8,
    enc: Option<crate::vendor_analysis::VendorEncoder>,
    out: VecDeque<Packet>,
    eof: bool,
}

/// The default `flags2` for encoding: VLC envelopes, bit reservoir,
/// variable block length, block-count shift k = 1 — the staged
/// vendor CBR configuration.
const DEFAULT_ENCODE_FLAGS2: u16 = 0x000f;

impl WmaEncoder {
    /// Build an encoder from codec parameters (`sample_rate`,
    /// `channels` ∈ {1, 2} and `bit_rate` required).
    pub fn from_params(params: &CodecParameters) -> Result<Self> {
        let version = version_for(params)
            .ok_or_else(|| Error::unsupported("oxideav-wma: not a WMA v1/v2 stream"))?;
        let sample_rate = params
            .sample_rate
            .ok_or_else(|| Error::invalid("oxideav-wma: sample_rate is required"))?;
        let channels = params
            .channels
            .filter(|&c| (1..=2).contains(&c))
            .ok_or_else(|| Error::invalid("oxideav-wma: 1 or 2 channels required"))?
            as u8;
        let bit_rate = params
            .bit_rate
            .filter(|&b| b > 0 && b <= u64::from(u32::MAX))
            .ok_or_else(|| Error::invalid("oxideav-wma: bit_rate is required"))?
            as u32;
        let flags2 = match version {
            Version::V1 if params.extradata.len() >= 4 => {
                u16::from_le_bytes([params.extradata[2], params.extradata[3]])
            }
            Version::V2 if params.extradata.len() >= 6 => {
                u16::from_le_bytes([params.extradata[4], params.extradata[5]])
            }
            _ => DEFAULT_ENCODE_FLAGS2,
        };
        let avg_bytes_per_sec = (bit_rate / 8).max(1);

        // Probe the frame length, then size the packet at eight
        // average frames (the staged vendor configurations' own
        // ratio), clamped to sane container bounds.
        let probe = StreamConfig::derive(
            version,
            sample_rate,
            channels,
            avg_bytes_per_sec,
            2048,
            flags2,
        )
        .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;
        let avg_frame_bytes = u64::from(avg_bytes_per_sec) * u64::from(probe.frame_length)
            / u64::from(sample_rate.max(1));
        let block_align = (avg_frame_bytes * 8).clamp(128, 32_768) as u16;
        let cfg = StreamConfig::derive(
            version,
            sample_rate,
            channels,
            avg_bytes_per_sec,
            block_align,
            flags2,
        )
        .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;
        let enc = crate::vendor_analysis::VendorEncoder::new(&cfg)
            .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;

        // Output parameters a muxer needs: the WAVEFORMATEX-shaped
        // extradata tail carrying flags2 at its versioned offset.
        let mut out_params = CodecParameters::audio(params.codec_id.clone());
        out_params.sample_rate = Some(sample_rate);
        out_params.channels = Some(u16::from(channels));
        out_params.sample_format = Some(SampleFormat::F32);
        out_params.bit_rate = Some(u64::from(bit_rate));
        out_params.extradata = match version {
            Version::V1 => {
                let mut e = vec![0u8; 4];
                e[2..4].copy_from_slice(&flags2.to_le_bytes());
                e
            }
            Version::V2 => {
                let mut e = vec![0u8; 10];
                e[4..6].copy_from_slice(&flags2.to_le_bytes());
                e
            }
        };
        out_params.tag = Some(CodecTag::wave_format(match version {
            Version::V1 => 0x0160,
            Version::V2 => 0x0161,
        }));

        Ok(Self {
            codec_id: params.codec_id.clone(),
            params: out_params,
            sample_rate,
            channels,
            enc: Some(enc),
            out: VecDeque::new(),
            eof: false,
        })
    }

    /// The chosen codec packet size (`nBlockAlign`).
    pub fn block_align(&self) -> u16 {
        self.enc
            .as_ref()
            .map(|e| e.config().block_align)
            .unwrap_or(0)
    }
}

impl Encoder for WmaEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.eof {
            return Err(Error::invalid("oxideav-wma: send_frame after flush"));
        }
        let audio = match frame {
            Frame::Audio(a) => a,
            _ => return Err(Error::invalid("oxideav-wma: audio frames only")),
        };
        let channels = usize::from(self.channels);
        let data = audio
            .data
            .first()
            .ok_or_else(|| Error::invalid("oxideav-wma: empty audio frame"))?;
        let want = audio.samples as usize * channels * 4;
        if data.len() < want {
            return Err(Error::invalid(
                "oxideav-wma: frame data shorter than samples x channels x 4 (interleaved F32)",
            ));
        }
        let mut planar: Vec<Vec<f64>> = vec![Vec::with_capacity(audio.samples as usize); channels];
        for (t, group) in data[..want].chunks_exact(4 * channels).enumerate() {
            let _ = t;
            for (ch, chunk) in group.chunks_exact(4).enumerate() {
                planar[ch].push(f64::from(f32::from_le_bytes([
                    chunk[0], chunk[1], chunk[2], chunk[3],
                ])));
            }
        }
        self.enc
            .as_mut()
            .expect("encoder present until flush")
            .push(&planar)
            .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        if let Some(p) = self.out.pop_front() {
            return Ok(p);
        }
        if self.eof {
            Err(Error::Eof)
        } else {
            Err(Error::NeedMore)
        }
    }

    fn flush(&mut self) -> Result<()> {
        if let Some(enc) = self.enc.take() {
            let packets = enc
                .finish()
                .map_err(|e| Error::invalid(format!("oxideav-wma: {e}")))?;
            let tb = TimeBase::new(1, i64::from(self.sample_rate));
            for data in packets {
                self.out.push_back(Packet::new(0, tb, data));
            }
        }
        self.eof = true;
        Ok(())
    }
}

/// Direct encoder factory (the dual-API counterpart of the registry
/// path): build a boxed [`Encoder`] from codec parameters.
///
/// # Errors
///
/// [`Error::Unsupported`] when the parameters name neither `wma1` nor
/// `wma2` (by id or wave tag); [`Error::InvalidData`] when a required
/// audio parameter is missing or the extradata selects the
/// unencodable §3.1 LSP envelope path.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Ok(Box::new(WmaEncoder::from_params(params)?))
}

/// Install the WMA codec into a [`RuntimeContext`]: decoder factories
/// for `wma1` / `wma2` and their `WAVEFORMATEX` tag claims
/// (`0x0160` / `0x0161`).
pub fn register(ctx: &mut RuntimeContext) {
    for (id, tag) in [("wma1", 0x0160u16), ("wma2", 0x0161u16)] {
        let mut caps = CodecCapabilities::audio("oxideav-wma");
        caps.decode = true;
        caps.encode = true;
        caps.lossy = true;
        caps.max_channels = Some(2);
        ctx.codecs.register(
            CodecInfo::new(CodecId::new(id))
                .capabilities(caps)
                .decoder(make_decoder)
                .encoder(make_encoder)
                .tag(CodecTag::wave_format(tag)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitio::BitWriter;
    use crate::wire_vlc::{coef_vlc, scale_vlc};

    /// Parameters for a fixed-block mono 44.1 kHz v2 stream with the
    /// reservoir off (`flags2 = 0x0001`): one frame per packet, no
    /// packet header — the craftable-by-hand configuration.
    fn mono_params() -> CodecParameters {
        let mut p = CodecParameters::audio(CodecId::new("wma2"));
        p.sample_rate = Some(44_100);
        p.channels = Some(1);
        p.bit_rate = Some(64_024);
        // v2 extradata: u32 LE flags1, u16 LE flags2 = 0x0001.
        p.extradata = vec![0, 0, 0, 0, 0x01, 0x00];
        p
    }

    /// One crafted packet: a single frame (coded channel, total gain,
    /// flat envelope, immediate EOB), zero-padded to `block_align`.
    fn crafted_packet(block_align: usize) -> Vec<u8> {
        let bands = crate::band_partition::exponent_band_count(44_100, 2048);
        let mut w = BitWriter::new();
        w.write_bit(true); // F2: channel coded
        w.write_bits(50, 7); // B1: total gain 51
        for _ in 0..bands {
            assert!(scale_vlc().encode_symbol(60, &mut w)); // delta 0
        }
        assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w)); // EOB
        let mut bytes = w.into_bytes();
        assert!(bytes.len() <= block_align);
        bytes.resize(block_align, 0);
        bytes
    }

    fn packet_of(data: Vec<u8>) -> Packet {
        Packet::new(0, oxideav_core::TimeBase::new(1, 44_100), data)
    }

    #[test]
    fn registry_path_decodes_crafted_packets() {
        let mut ctx = RuntimeContext::new();
        register(&mut ctx);
        let params = mono_params();
        let mut dec = ctx
            .codecs
            .first_decoder(&params)
            .expect("factory installed");

        let ba = 512usize;
        dec.send_packet(&packet_of(crafted_packet(ba))).unwrap();
        dec.send_packet(&packet_of(crafted_packet(ba))).unwrap();
        // Packet 0 decodes once packet 1 arrives.
        let f = match dec.receive_frame().unwrap() {
            Frame::Audio(f) => f,
            other => panic!("unexpected frame {other:?}"),
        };
        assert_eq!(f.samples, 2048);
        assert_eq!(f.data.len(), 1);
        assert_eq!(f.data[0].len(), 2048 * 4);
        assert!(matches!(dec.receive_frame(), Err(Error::NeedMore)));

        // Flush drains packet 1 and the synthesiser tail.
        dec.flush().unwrap();
        let f1 = match dec.receive_frame().unwrap() {
            Frame::Audio(f) => f,
            other => panic!("unexpected frame {other:?}"),
        };
        assert_eq!(f1.samples, 2048);
        let tail = match dec.receive_frame().unwrap() {
            Frame::Audio(f) => f,
            other => panic!("unexpected frame {other:?}"),
        };
        assert_eq!(tail.samples, 1024, "flush drains frame_length / 2");
        assert!(matches!(dec.receive_frame(), Err(Error::Eof)));

        // All output is finite F32 in a sane range.
        for frame in [&f, &f1, &tail] {
            for chunk in frame.data[0].chunks_exact(4) {
                let v = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                assert!(v.is_finite() && v.abs() <= 4.0, "sample {v}");
            }
        }
    }

    #[test]
    fn tag_resolution_and_direct_factory_agree() {
        let mut ctx = RuntimeContext::new();
        register(&mut ctx);
        // Tag-keyed lookup: 0x0161 resolves to wma2.
        assert!(ctx.codecs.has_decoder(&CodecId::new("wma1")));
        assert!(ctx.codecs.has_decoder(&CodecId::new("wma2")));

        // Direct factory: id path.
        assert!(make_decoder(&mono_params()).is_ok());

        // Direct factory: tag fallback with a non-registry id.
        let mut p = mono_params();
        p.codec_id = CodecId::new("something-else");
        assert!(make_decoder(&p).is_err());
        p.tag = Some(CodecTag::wave_format(0x0161));
        let dec = make_decoder(&p).unwrap();
        assert_eq!(dec.codec_id().as_str(), "something-else");
    }

    #[test]
    fn missing_parameters_are_typed_errors() {
        let mut p = mono_params();
        p.bit_rate = None;
        assert!(make_decoder(&p).is_err());
        let mut p = mono_params();
        p.sample_rate = None;
        assert!(make_decoder(&p).is_err());
        let mut p = mono_params();
        p.extradata = vec![0; 3];
        assert!(make_decoder(&p).is_err());
        let mut p = mono_params();
        p.channels = Some(6);
        assert!(make_decoder(&p).is_err());
    }

    #[test]
    fn reset_recovers_a_reusable_decoder() {
        let mut dec = WmaDecoder::from_params(&mono_params()).unwrap();
        let ba = 512usize;
        dec.send_packet(&packet_of(crafted_packet(ba))).unwrap();
        dec.flush().unwrap();
        assert!(dec.receive_frame().is_ok());
        dec.reset().unwrap();
        // After reset the decoder accepts packets again — and can
        // re-lock a different block_align.
        dec.send_packet(&packet_of(crafted_packet(600))).unwrap();
        dec.send_packet(&packet_of(crafted_packet(600))).unwrap();
        assert!(matches!(dec.receive_frame(), Ok(Frame::Audio(_))));
    }

    #[test]
    fn corrupt_packets_produce_silence_not_panics() {
        let mut dec = WmaDecoder::from_params(&mono_params()).unwrap();
        let ba = 512usize;
        // Arbitrary bytes: every declared frame either parses or is
        // replaced by silence; nothing panics.
        let junk: Vec<u8> = (0..ba).map(|i| (i * 37 + 11) as u8).collect();
        dec.send_packet(&packet_of(junk.clone())).unwrap();
        dec.send_packet(&packet_of(junk)).unwrap();
        dec.flush().unwrap();
        let mut frames = 0;
        while let Ok(Frame::Audio(f)) = dec.receive_frame() {
            frames += 1;
            for chunk in f.data[0].chunks_exact(4) {
                let v = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                assert!(v.is_finite());
            }
        }
        assert!(frames >= 2, "the timeline is kept: one frame per packet");
    }

    #[test]
    fn registered_encoder_round_trips_through_the_registered_decoder() {
        let mut ctx = RuntimeContext::new();
        register(&mut ctx);
        let mut p = CodecParameters::audio(CodecId::new("wma2"));
        p.sample_rate = Some(22_050);
        p.channels = Some(2);
        p.bit_rate = Some(64_000);
        let mut enc = ctx.codecs.first_encoder(&p).expect("factory installed");

        // Two seconds of correlated stereo material.
        let n = 22_050usize;
        let left: Vec<f32> = (0..n)
            .map(|t| {
                0.25 * (2.0 * std::f32::consts::PI * 441.0 * t as f32 / 22_050.0).sin()
                    + 0.1 * (2.0 * std::f32::consts::PI * 1234.0 * t as f32 / 22_050.0).sin()
            })
            .collect();
        let mut bytes = Vec::with_capacity(n * 8);
        for &v in &left {
            bytes.extend_from_slice(&v.to_le_bytes());
            bytes.extend_from_slice(&(v * 0.8).to_le_bytes());
        }
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: n as u32,
            pts: None,
            data: vec![bytes],
        }))
        .unwrap();
        assert!(matches!(enc.receive_packet(), Err(Error::NeedMore)));
        enc.flush().unwrap();
        let mut packets = Vec::new();
        while let Ok(pkt) = enc.receive_packet() {
            packets.push(pkt);
        }
        assert!(!packets.is_empty());
        let ba = packets[0].data.len();
        assert!(packets.iter().all(|p| p.data.len() == ba));

        // Decode with the registered decoder built from the
        // encoder's own output parameters.
        let out_params = {
            let mut q = enc.output_params().clone();
            q.codec_id = CodecId::new("wma2");
            q
        };
        let mut dec = make_decoder(&out_params).unwrap();
        let mut pcm: Vec<f32> = Vec::new();
        for pkt in &packets {
            dec.send_packet(pkt).unwrap();
            while let Ok(Frame::Audio(f)) = dec.receive_frame() {
                pcm.extend(
                    f.data[0]
                        .chunks_exact(4)
                        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
                );
            }
        }
        dec.flush().unwrap();
        while let Ok(Frame::Audio(f)) = dec.receive_frame() {
            pcm.extend(
                f.data[0]
                    .chunks_exact(4)
                    .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
            );
        }

        // SNR at the chain's fixed frame_length/2 lead-in.
        let flen = 1024usize; // 22.05 kHz -> 1024-sample frames
        let lead = flen / 2;
        let (mut sig, mut err) = (0.0f64, 0.0f64);
        for (t, &l) in left.iter().enumerate().take(n) {
            let a = f64::from(l);
            let b = pcm
                .get((t + lead) * 2)
                .copied()
                .map(f64::from)
                .unwrap_or(0.0);
            sig += a * a;
            err += (a - b) * (a - b);
        }
        let snr = 10.0 * (sig / err.max(1e-30)).log10();
        assert!(snr > 15.0, "registry round-trip SNR {snr:.2} dB");
    }

    #[test]
    fn encoder_parameter_validation_is_typed() {
        let mut p = CodecParameters::audio(CodecId::new("wma2"));
        assert!(make_encoder(&p).is_err()); // no sample rate
        p.sample_rate = Some(22_050);
        p.channels = Some(6);
        p.bit_rate = Some(64_000);
        assert!(make_encoder(&p).is_err()); // channel count
        p.channels = Some(2);
        p.bit_rate = None;
        assert!(make_encoder(&p).is_err()); // bit rate
        p.bit_rate = Some(64_000);
        assert!(make_encoder(&p).is_ok());
        // LSP-path extradata (flags2 bit 0 clear) is refused.
        p.extradata = vec![0, 0, 0, 0, 0x26, 0x00];
        assert!(make_encoder(&p).is_err());
        // Unknown id without a tag is unsupported.
        p.extradata.clear();
        p.codec_id = CodecId::new("something-else");
        assert!(make_encoder(&p).is_err());
    }
}

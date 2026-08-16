//! §1 packet (superframe) layer — the packet header and the
//! bit-reservoir carry across packet boundaries.
//!
//! ## Source
//!
//! `docs/audio/wma/frame-bit-layout.md` §1: the codec consumes one
//! container packet of `block_align` bytes at a time. With the
//! reservoir off there is no packet header at all — one frame per
//! packet, starting at bit 0. With the reservoir on, each packet
//! opens with:
//!
//! | field | width | meaning |
//! | ----- | ----- | ------- |
//! | P1 | 4 | sequence number, +1 mod 16 per packet; a break is a discontinuity |
//! | P2 | 4 | frames in this packet, decremented per decoded frame |
//! | P3 | `byte_offset_bits + 3` | reservoir carry: bits at the start of the body belonging to the previous packet's unfinished frame |
//!
//! On a discontinuity exactly P3 bits are skipped (the carried tail
//! has lost its head). Frames follow back-to-back; a frame that does
//! not finish inside its packet continues into the next packet's
//! carry region. The staged measurement over the six committed
//! vendor streams (`tables/vendor-stream-packet-headers.csv`) shows a
//! non-zero carry is the *normal* case — 724 of 787 packets.
//!
//! **Zero-carry packets mark the previous packet as padded**
//! (vendor-measured calibration, r446): a P3 of 0 means the previous
//! packet's declared frames all completed inside it, and any body
//! bits left there after the last frame are padding, not frame data.
//! The VBR-configured vendor streams pad most packets this way — the
//! 96 kbps 44.1 kHz stream closes all 133 carry boundaries under
//! this reading and only 83 under a strict frames-fill-the-body
//! reading (`tests/vendor_streams.rs`).
//!
//! [`PacketAssembler`] validates and strips the §1 header from each
//! packet and concatenates the bodies into one contiguous bit
//! stream, recording per packet where its body landed and what its
//! header declared — exactly what a frame-level parser needs to
//! (a) decode frames across packet boundaries and (b) check itself
//! against the next packet's carry boundary.

use crate::bitio::{BitReader, BitWriter, BitstreamEnd};
use crate::stream_config::StreamConfig;

/// A parsed §1 packet header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketHeader {
    /// P1 — 4-bit sequence number.
    pub sequence: u8,
    /// P2 — 4-bit frames-in-packet count.
    pub frame_count: u8,
    /// P3 — reservoir carry, in bits.
    pub carry_bits: u32,
}

/// Packet-layer failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketError {
    /// The packet is not exactly `block_align` bytes.
    WrongPacketSize {
        /// Expected size (the configuration's `block_align`).
        expected: u16,
        /// Actual pushed length.
        got: usize,
    },
    /// The carry field claims more bits than the packet body holds.
    CarryOutOfBounds {
        /// The declared carry.
        carry_bits: u32,
        /// The packet body size in bits.
        body_bits: u32,
    },
    /// The packet ended inside the header (unreachable once the size
    /// check passed; kept for defence in depth).
    Truncated,
}

impl core::fmt::Display for PacketError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PacketError::WrongPacketSize { expected, got } => write!(
                f,
                "oxideav-wma: packet is {got} bytes, the stream's block_align is {expected}"
            ),
            PacketError::CarryOutOfBounds {
                carry_bits,
                body_bits,
            } => write!(
                f,
                "oxideav-wma: reservoir carry of {carry_bits} bits exceeds the {body_bits}-bit packet body"
            ),
            PacketError::Truncated => f.write_str("oxideav-wma: packet ended inside the header"),
        }
    }
}

impl std::error::Error for PacketError {}

impl From<BitstreamEnd> for PacketError {
    fn from(_: BitstreamEnd) -> Self {
        PacketError::Truncated
    }
}

impl PacketHeader {
    /// Parse the §1 header off the front of a packet's bits.
    ///
    /// # Errors
    ///
    /// [`PacketError::Truncated`] if the reader runs dry inside the
    /// header.
    pub fn parse(cfg: &StreamConfig, reader: &mut BitReader<'_>) -> Result<Self, PacketError> {
        let sequence = reader.read_bits(4)? as u8;
        let frame_count = reader.read_bits(4)? as u8;
        let carry_bits = reader.read_bits(cfg.carry_field_bits())? as u32;
        Ok(PacketHeader {
            sequence,
            frame_count,
            carry_bits,
        })
    }
}

/// One packet's record inside an [`AssembledStream`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketRecord {
    /// The parsed header (for a no-reservoir stream: sequence 0,
    /// frame count 1, carry 0 — the §1 degenerate case).
    pub header: PacketHeader,
    /// Bit offset of this packet's body inside the assembled stream.
    pub body_start_bit: u64,
    /// Body size in bits (`block_align · 8` less the header).
    pub body_bits: u32,
    /// Whether the sequence number broke against the previous packet
    /// (the §1 discontinuity rule: the carried bits are skipped).
    pub discontinuity: bool,
}

impl PacketRecord {
    /// Where this packet's *own* frames begin in the assembled
    /// stream: the body start plus the carry region (which belongs
    /// to the previous packet's unfinished frame).
    pub fn frames_start_bit(&self) -> u64 {
        self.body_start_bit + u64::from(self.header.carry_bits)
    }
}

/// The §1 layer's output: every packet body concatenated into one
/// contiguous bit stream, with per-packet records.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssembledStream {
    /// The concatenated packet bodies, MSB-first bit-packed.
    pub bytes: Vec<u8>,
    /// Total valid bits in `bytes`.
    pub total_bits: u64,
    /// One record per pushed packet, in order.
    pub packets: Vec<PacketRecord>,
}

impl AssembledStream {
    /// A reader positioned at `bit_offset` in the assembled stream.
    pub fn reader_at(&self, bit_offset: u64) -> BitReader<'_> {
        let mut r = BitReader::with_bit_len(&self.bytes, self.total_bits as usize);
        // Skip in u32-sized strides (BitReader::read_bits caps at 64).
        let mut left = bit_offset;
        while left > 0 {
            let step = left.min(32) as u8;
            // A caller-supplied offset beyond the stream is a caller
            // bug; saturate by stopping at the end.
            if r.read_bits(step).is_err() {
                break;
            }
            left -= u64::from(step);
        }
        r
    }
}

/// Stateful §1 packet walker: validates each packet's header,
/// tracks sequence continuity, and concatenates bodies.
#[derive(Debug, Clone)]
pub struct PacketAssembler {
    cfg: StreamConfig,
    writer: BitWriter,
    packets: Vec<PacketRecord>,
    expected_sequence: Option<u8>,
    body_cursor_bits: u64,
}

impl PacketAssembler {
    /// A fresh assembler for one stream configuration.
    pub fn new(cfg: &StreamConfig) -> Self {
        Self {
            cfg: cfg.clone(),
            writer: BitWriter::new(),
            packets: Vec::new(),
            expected_sequence: None,
            body_cursor_bits: 0,
        }
    }

    /// Push one container packet (exactly `block_align` bytes).
    ///
    /// Returns the packet's record (also retained internally).
    ///
    /// # Errors
    ///
    /// * [`PacketError::WrongPacketSize`] for a mis-sized packet.
    /// * [`PacketError::CarryOutOfBounds`] when P3 is not strictly
    ///   inside the packet body — the staged measurement holds this
    ///   for all 787 vendor packets, so a violation means garbage
    ///   input rather than a vendor stream.
    pub fn push_packet(&mut self, packet: &[u8]) -> Result<PacketRecord, PacketError> {
        if packet.len() != usize::from(self.cfg.block_align) {
            return Err(PacketError::WrongPacketSize {
                expected: self.cfg.block_align,
                got: packet.len(),
            });
        }
        let mut reader = BitReader::new(packet);
        let (header, discontinuity) = if self.cfg.bit_reservoir {
            let header = PacketHeader::parse(&self.cfg, &mut reader)?;
            let body_bits = self.cfg.packet_body_bits();
            if header.carry_bits >= body_bits {
                return Err(PacketError::CarryOutOfBounds {
                    carry_bits: header.carry_bits,
                    body_bits,
                });
            }
            let discontinuity = match self.expected_sequence {
                Some(expected) => header.sequence != expected,
                // The first packet establishes the phase.
                None => false,
            };
            self.expected_sequence = Some((header.sequence + 1) & 0xf);
            (header, discontinuity)
        } else {
            // §1: no reservoir → no header, exactly one frame.
            (
                PacketHeader {
                    sequence: 0,
                    frame_count: 1,
                    carry_bits: 0,
                },
                false,
            )
        };

        let body_bits = self.cfg.packet_body_bits();
        let record = PacketRecord {
            header,
            body_start_bit: self.body_cursor_bits,
            body_bits,
            discontinuity,
        };
        // Append the body bits (everything after the header).
        let mut left = body_bits;
        while left > 0 {
            let step = left.min(32) as u8;
            let v = reader.read_bits(step)?;
            self.writer.write_bits(v, step);
            left -= u32::from(step);
        }
        self.body_cursor_bits += u64::from(body_bits);
        self.packets.push(record);
        Ok(record)
    }

    /// Records of the packets pushed so far.
    pub fn packets(&self) -> &[PacketRecord] {
        &self.packets
    }

    /// Close the walk and hand back the assembled stream.
    pub fn finish(self) -> AssembledStream {
        let total_bits = self.body_cursor_bits;
        AssembledStream {
            bytes: self.writer.into_bytes(),
            total_bits,
            packets: self.packets,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::Version;

    fn reservoir_cfg() -> StreamConfig {
        // cand_mono22k_16kbps's configuration: byte_offset_bits 8,
        // header 19 bits, body 5933 bits.
        StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap()
    }

    fn make_packet(cfg: &StreamConfig, seq: u8, frames: u8, carry: u32, fill: u8) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_bits(u64::from(seq), 4);
        w.write_bits(u64::from(frames), 4);
        w.write_bits(u64::from(carry), cfg.carry_field_bits());
        let mut bytes = w.into_bytes();
        bytes.resize(usize::from(cfg.block_align), fill);
        bytes
    }

    #[test]
    fn header_fields_parse_at_the_staged_widths() {
        let cfg = reservoir_cfg();
        let pkt = make_packet(&cfg, 9, 7, 1023, 0);
        let mut r = BitReader::new(&pkt);
        let h = PacketHeader::parse(&cfg, &mut r).unwrap();
        assert_eq!(h.sequence, 9);
        assert_eq!(h.frame_count, 7);
        assert_eq!(h.carry_bits, 1023);
        assert_eq!(r.position(), 19); // byte_offset_bits 8 → 19-bit header
    }

    #[test]
    fn bodies_concatenate_and_sequence_tracks_mod_16() {
        let cfg = reservoir_cfg();
        let mut asm = PacketAssembler::new(&cfg);
        for i in 0..20u8 {
            let rec = asm
                .push_packet(&make_packet(&cfg, (3 + i) & 0xf, 5, 100, i))
                .unwrap();
            assert!(!rec.discontinuity, "packet {i}");
            assert_eq!(rec.body_start_bit, u64::from(i) * 5933);
            assert_eq!(rec.frames_start_bit(), u64::from(i) * 5933 + 100);
        }
        let stream = asm.finish();
        assert_eq!(stream.total_bits, 20 * 5933);
        assert_eq!(stream.packets.len(), 20);
    }

    #[test]
    fn sequence_break_is_flagged_as_discontinuity() {
        let cfg = reservoir_cfg();
        let mut asm = PacketAssembler::new(&cfg);
        asm.push_packet(&make_packet(&cfg, 0, 5, 0, 0)).unwrap();
        asm.push_packet(&make_packet(&cfg, 1, 5, 0, 0)).unwrap();
        // Jump: expected 2, got 7.
        let rec = asm.push_packet(&make_packet(&cfg, 7, 5, 0, 0)).unwrap();
        assert!(rec.discontinuity);
        // Continuity resumes from the new phase.
        let rec = asm.push_packet(&make_packet(&cfg, 8, 5, 0, 0)).unwrap();
        assert!(!rec.discontinuity);
    }

    #[test]
    fn carry_must_stay_inside_the_body() {
        // A small packet whose carry field (13 bits here) can express
        // more bits than the body holds.
        let cfg = StreamConfig::derive(Version::V2, 44_100, 1, 8003, 500, 0x000f).unwrap();
        assert_eq!(cfg.carry_field_bits(), 13);
        let body_bits = cfg.packet_body_bits();
        assert!(body_bits < 1 << 13);
        let mut asm = PacketAssembler::new(&cfg);
        let err = asm
            .push_packet(&make_packet(&cfg, 0, 5, body_bits, 0))
            .unwrap_err();
        assert_eq!(
            err,
            PacketError::CarryOutOfBounds {
                carry_bits: body_bits,
                body_bits,
            }
        );
    }

    #[test]
    fn wrong_size_packets_are_rejected() {
        let cfg = reservoir_cfg();
        let mut asm = PacketAssembler::new(&cfg);
        assert!(matches!(
            asm.push_packet(&[0u8; 100]),
            Err(PacketError::WrongPacketSize { expected: 744, .. })
        ));
    }

    #[test]
    fn no_reservoir_streams_have_headerless_single_frame_packets() {
        // flags2 bit 1 clear → §1 degenerate case.
        let cfg = StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x0000).unwrap();
        assert_eq!(cfg.packet_header_bits(), 0);
        let mut asm = PacketAssembler::new(&cfg);
        let rec = asm.push_packet(&vec![0xa5; 744]).unwrap();
        assert_eq!(rec.header.frame_count, 1);
        assert_eq!(rec.header.carry_bits, 0);
        assert_eq!(rec.body_bits, 744 * 8);
        let stream = asm.finish();
        // Body is the whole packet, bit-identical.
        assert_eq!(stream.bytes[..744], vec![0xa5; 744][..]);
    }

    #[test]
    fn reader_at_lands_on_the_requested_bit() {
        let cfg = reservoir_cfg();
        let mut asm = PacketAssembler::new(&cfg);
        // Fill bodies with a recognisable ramp.
        for i in 0..3u8 {
            asm.push_packet(&make_packet(&cfg, i, 5, 0, 0x0f)).unwrap();
        }
        let stream = asm.finish();
        // The body of packet 1 starts at 5933; its fill bytes are
        // 0x0f from some offset — read 8 bits at a byte-aligned spot
        // relative to the original packet: header is 19 bits, so
        // packet byte 3 starts at body bit 5 (24 - 19).
        let mut r = stream.reader_at(5933 + 5);
        assert_eq!(r.read_bits(8).unwrap(), 0x0f);
    }
}

//! Bit-level writer/reader plumbing for the entropy stage.
//!
//! ## Source and posture
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 establishes that
//! the WMA Standard payload is a stream of variable-length
//! (Huffman/VLC) codewords — run-level `(R, L)` codes, escape
//! literals, matrix-delta codes — and §8 draws the MUX/DEMUX that
//! packs and unpacks them. Reading and writing prefix codes requires a
//! bit-granular cursor over a byte buffer; that machinery is
//! format-neutral **[DSP]**-tier plumbing (no WMA-specific fact lives
//! in it), and this module provides it so the entropy stages have a
//! carrier to run on.
//!
//! **What stays `[GAP]`:** the byte/bit packing order of the shipping
//! WMA v1/v2 bitstream (which end of each byte fills first, the
//! superframe byte layout, any padding/alignment rules) is not
//! disclosed by the staged material. This module fixes an internal
//! **MSB-first** convention — the conventional order for prefix-code
//! I/O, in which the first bit written occupies the highest bit of
//! byte 0 — as a *realization detail of this crate's self-consistent
//! coder*, explicitly **not** as a WMA wire-format claim. When a trace
//! pins the real packing order, this is the single point to swap.
//!
//! ## Scope
//!
//! * [`BitWriter`] — append bits / n-bit fields to a growing byte
//!   buffer, MSB-first.
//! * [`BitReader`] — the exact inverse cursor over a byte slice, with
//!   an optional bit-precise stream length.
//!
//! Write→read round trips are pinned bit-for-bit by tests.

/// Append-only MSB-first bit sink over a growing byte buffer.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BitWriter {
    bytes: Vec<u8>,
    /// Bits already used in the final byte (`0..8`; `0` means the
    /// buffer is byte-aligned).
    used: u8,
}

impl BitWriter {
    /// Fresh, empty writer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Total bits written so far.
    pub fn bit_len(&self) -> usize {
        if self.used == 0 {
            self.bytes.len() * 8
        } else {
            (self.bytes.len() - 1) * 8 + self.used as usize
        }
    }

    /// Whether nothing has been written.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Append a single bit.
    pub fn write_bit(&mut self, bit: bool) {
        if self.used == 0 {
            self.bytes.push(0);
        }
        if bit {
            let last = self.bytes.len() - 1;
            self.bytes[last] |= 1 << (7 - self.used);
        }
        self.used = (self.used + 1) % 8;
    }

    /// Append the low `n` bits of `value`, most-significant first.
    ///
    /// # Panics
    ///
    /// Panics if `n > 64`.
    pub fn write_bits(&mut self, value: u64, n: u8) {
        assert!(
            n <= 64,
            "oxideav-wma::bitio::BitWriter::write_bits: n must be <= 64, got {n}",
        );
        for i in (0..n).rev() {
            self.write_bit((value >> i) & 1 == 1);
        }
    }

    /// Pad with zero bits to the next byte boundary (no-op when
    /// already aligned).
    pub fn align_to_byte(&mut self) {
        self.used = 0;
    }

    /// Finish, returning the byte buffer. A trailing partial byte is
    /// zero-padded (the padding is *not* part of [`BitWriter::bit_len`]).
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }
}

/// The reader ran past the end of the bit stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BitstreamEnd {
    /// Bits the failed request asked for.
    pub requested: usize,
    /// Bits that were actually left.
    pub remaining: usize,
}

impl core::fmt::Display for BitstreamEnd {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-wma::bitio: bit stream exhausted ({} bits requested, {} remaining)",
            self.requested, self.remaining
        )
    }
}

impl std::error::Error for BitstreamEnd {}

/// MSB-first bit cursor over a byte slice — the exact inverse of
/// [`BitWriter`].
#[derive(Debug, Clone)]
pub struct BitReader<'a> {
    bytes: &'a [u8],
    /// Cursor position, in bits from the start of `bytes`.
    pos: usize,
    /// Total readable bits (allows a bit-precise stream length below
    /// the byte-padded buffer size).
    len_bits: usize,
}

impl<'a> BitReader<'a> {
    /// Reader over the whole byte slice (`bytes.len() * 8` bits).
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            pos: 0,
            len_bits: bytes.len() * 8,
        }
    }

    /// Reader over the first `len_bits` bits of the slice — the shape
    /// a [`BitWriter::bit_len`]-aware caller uses to exclude the final
    /// byte's zero padding.
    ///
    /// # Panics
    ///
    /// Panics if `len_bits > bytes.len() * 8`.
    pub fn with_bit_len(bytes: &'a [u8], len_bits: usize) -> Self {
        assert!(
            len_bits <= bytes.len() * 8,
            "oxideav-wma::bitio::BitReader::with_bit_len: len_bits {len_bits} exceeds buffer capacity {}",
            bytes.len() * 8,
        );
        Self {
            bytes,
            pos: 0,
            len_bits,
        }
    }

    /// Bits not yet consumed.
    pub fn remaining_bits(&self) -> usize {
        self.len_bits - self.pos
    }

    /// Cursor position in bits from the stream start.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Read one bit.
    pub fn read_bit(&mut self) -> Result<bool, BitstreamEnd> {
        if self.pos >= self.len_bits {
            return Err(BitstreamEnd {
                requested: 1,
                remaining: 0,
            });
        }
        let byte = self.bytes[self.pos / 8];
        let bit = (byte >> (7 - (self.pos % 8))) & 1 == 1;
        self.pos += 1;
        Ok(bit)
    }

    /// Read `n` bits as the low bits of a `u64`, most-significant
    /// first — the exact inverse of [`BitWriter::write_bits`].
    ///
    /// On failure the cursor is left where it was (no partial
    /// consumption).
    ///
    /// # Panics
    ///
    /// Panics if `n > 64`.
    pub fn read_bits(&mut self, n: u8) -> Result<u64, BitstreamEnd> {
        assert!(
            n <= 64,
            "oxideav-wma::bitio::BitReader::read_bits: n must be <= 64, got {n}",
        );
        if (n as usize) > self.remaining_bits() {
            return Err(BitstreamEnd {
                requested: n as usize,
                remaining: self.remaining_bits(),
            });
        }
        let mut value = 0u64;
        for _ in 0..n {
            value = (value << 1) | u64::from(self.read_bit().expect("length checked above"));
        }
        Ok(value)
    }

    /// Skip to the next byte boundary (no-op when already aligned or
    /// when doing so would pass the stream end, in which case the
    /// cursor moves to the stream end).
    pub fn align_to_byte(&mut self) {
        let rem = self.pos % 8;
        if rem != 0 {
            self.pos = (self.pos + 8 - rem).min(self.len_bits);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- writer ----------

    #[test]
    fn fresh_writer_is_empty() {
        let w = BitWriter::new();
        assert!(w.is_empty());
        assert_eq!(w.bit_len(), 0);
        assert!(w.into_bytes().is_empty());
    }

    #[test]
    fn write_bit_fills_msb_first() {
        let mut w = BitWriter::new();
        w.write_bit(true);
        w.write_bit(false);
        w.write_bit(true);
        assert_eq!(w.bit_len(), 3);
        // 101 in the top three bits: 1010_0000.
        assert_eq!(w.into_bytes(), vec![0b1010_0000]);
    }

    #[test]
    fn write_bits_emits_most_significant_first() {
        let mut w = BitWriter::new();
        w.write_bits(0b1011, 4);
        w.write_bits(0b0110, 4);
        assert_eq!(w.bit_len(), 8);
        assert_eq!(w.into_bytes(), vec![0b1011_0110]);
    }

    #[test]
    fn write_bits_zero_width_is_noop() {
        let mut w = BitWriter::new();
        w.write_bits(0xFFFF, 0);
        assert!(w.is_empty());
    }

    #[test]
    fn write_bits_crosses_byte_boundaries() {
        let mut w = BitWriter::new();
        w.write_bits(0xABCD, 16);
        w.write_bits(0b101, 3);
        assert_eq!(w.bit_len(), 19);
        assert_eq!(w.into_bytes(), vec![0xAB, 0xCD, 0b1010_0000]);
    }

    #[test]
    fn write_bits_full_u64_width() {
        let mut w = BitWriter::new();
        w.write_bits(u64::MAX, 64);
        assert_eq!(w.bit_len(), 64);
        assert_eq!(w.into_bytes(), vec![0xFF; 8]);
    }

    #[test]
    #[should_panic(expected = "n must be <= 64")]
    fn write_bits_rejects_overwide_field() {
        let mut w = BitWriter::new();
        w.write_bits(0, 65);
    }

    #[test]
    fn align_to_byte_pads_with_zeros() {
        let mut w = BitWriter::new();
        w.write_bits(0b11, 2);
        w.align_to_byte();
        w.write_bits(0xFF, 8);
        assert_eq!(w.bit_len(), 16);
        assert_eq!(w.into_bytes(), vec![0b1100_0000, 0xFF]);
    }

    // ---------- reader ----------

    #[test]
    fn read_bit_consumes_msb_first() {
        let bytes = [0b1010_0000];
        let mut r = BitReader::new(&bytes);
        assert!(r.read_bit().unwrap());
        assert!(!r.read_bit().unwrap());
        assert!(r.read_bit().unwrap());
        assert_eq!(r.position(), 3);
        assert_eq!(r.remaining_bits(), 5);
    }

    #[test]
    fn read_bits_matches_write_bits() {
        let bytes = [0xAB, 0xCD];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.read_bits(4).unwrap(), 0xA);
        assert_eq!(r.read_bits(8).unwrap(), 0xBC);
        assert_eq!(r.read_bits(4).unwrap(), 0xD);
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn read_bits_zero_width_reads_nothing() {
        let bytes = [0xFF];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.read_bits(0).unwrap(), 0);
        assert_eq!(r.position(), 0);
    }

    #[test]
    fn read_past_end_fails_without_consuming() {
        let bytes = [0xFF];
        let mut r = BitReader::new(&bytes);
        let _ = r.read_bits(6).unwrap();
        let err = r.read_bits(3).unwrap_err();
        assert_eq!(
            err,
            BitstreamEnd {
                requested: 3,
                remaining: 2,
            }
        );
        // Cursor untouched by the failed read.
        assert_eq!(r.position(), 6);
        assert_eq!(r.read_bits(2).unwrap(), 0b11);
    }

    #[test]
    fn with_bit_len_excludes_padding() {
        let bytes = [0b1110_0000];
        let mut r = BitReader::with_bit_len(&bytes, 3);
        assert_eq!(r.read_bits(3).unwrap(), 0b111);
        assert_eq!(
            r.read_bit().unwrap_err(),
            BitstreamEnd {
                requested: 1,
                remaining: 0,
            }
        );
    }

    #[test]
    #[should_panic(expected = "exceeds buffer capacity")]
    fn with_bit_len_rejects_overlong_claim() {
        let bytes = [0u8];
        let _ = BitReader::with_bit_len(&bytes, 9);
    }

    #[test]
    fn reader_align_to_byte_skips_to_boundary() {
        let bytes = [0b1010_1010, 0xFF];
        let mut r = BitReader::new(&bytes);
        let _ = r.read_bits(3).unwrap();
        r.align_to_byte();
        assert_eq!(r.position(), 8);
        assert_eq!(r.read_bits(8).unwrap(), 0xFF);
        r.align_to_byte(); // already aligned: no-op
        assert_eq!(r.position(), 16);
    }

    // ---------- write → read round trip ----------

    #[test]
    fn write_read_round_trip_mixed_widths() {
        // A deterministic pseudo-random field sequence with widths
        // 1..=24 round-trips bit-for-bit.
        let mut state = 0xC0FFEE_u64;
        let mut fields = Vec::new();
        for i in 0..200u64 {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let width = (i % 24 + 1) as u8;
            let value = state & ((1u64 << width) - 1);
            fields.push((value, width));
        }

        let mut w = BitWriter::new();
        for &(value, width) in &fields {
            w.write_bits(value, width);
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();

        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        for &(value, width) in &fields {
            assert_eq!(r.read_bits(width).unwrap(), value, "width={width}");
        }
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn bitstream_end_display_and_error() {
        let e = BitstreamEnd {
            requested: 3,
            remaining: 2,
        };
        let s = format!("{e}");
        assert!(s.contains('3'));
        assert!(s.contains('2'));
        let dyn_err: &dyn std::error::Error = &e;
        assert!(dyn_err.source().is_none());
    }
}

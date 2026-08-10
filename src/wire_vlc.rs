//! Prefix-code machinery over the **exact vendor codeword
//! assignments** of [`crate::wire_codes`].
//!
//! The staged trace pins the reader as MSB-first in stream order
//! (`frame-bit-layout.md`, bit-reader primitive), so decoding walks
//! the stream one bit at a time, extending the candidate codeword
//! until it matches a staged `(length, code)` entry. Every staged
//! table satisfies the Kraft equality exactly (checked at
//! construction), so on a complete table any bit pattern resolves to
//! a symbol — a decode can only fail by running out of bits.
//!
//! [`ExactVlc::new`] validates the table it is handed: in-range
//! codewords, no duplicate `(length, code)`, and Kraft completeness.
//! The shared singletons ([`coef_vlc`], [`scale_vlc`], [`gain_vlc`])
//! build once and are cheap to reuse; [`runlevel_map`] pairs each
//! coefficient table with its staged 2-based companion map.

use std::collections::HashMap;
use std::sync::OnceLock;

use crate::bitio::{BitReader, BitWriter, BitstreamEnd};
use crate::wire_codes::{
    COEF_CODES_CLASS1, COEF_CODES_CLASS1_ALT, COEF_CODES_CLASS2, COEF_CODES_CLASS2_ALT,
    COEF_CODES_CLASS3, COEF_CODES_CLASS3_ALT, GAIN_CODES, RUNLEVEL_CLASS1, RUNLEVEL_CLASS1_ALT,
    RUNLEVEL_CLASS2, RUNLEVEL_CLASS2_ALT, RUNLEVEL_CLASS3, RUNLEVEL_CLASS3_ALT, SCALE_CODES,
};

/// Construction failures for [`ExactVlc`] — none is reachable from
/// the staged tables (pinned by test), but the validation is kept so
/// a transcription defect fails loudly rather than desynchronising a
/// parse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExactVlcError {
    /// A codeword does not fit in its declared length.
    CodeOutOfRange {
        /// Offending symbol index.
        symbol: usize,
    },
    /// Two symbols share one `(length, code)` pair.
    DuplicateCode {
        /// The second symbol carrying the duplicate.
        symbol: usize,
    },
    /// A zero-length or over-long (> 32) codeword.
    BadLength {
        /// Offending symbol index.
        symbol: usize,
    },
    /// The lengths do not satisfy the Kraft equality (the staged
    /// tables are all complete prefix codes).
    KraftIncomplete,
}

impl core::fmt::Display for ExactVlcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ExactVlcError::CodeOutOfRange { symbol } => {
                write!(
                    f,
                    "oxideav-wma::wire_vlc: codeword of symbol {symbol} exceeds its length"
                )
            }
            ExactVlcError::DuplicateCode { symbol } => {
                write!(
                    f,
                    "oxideav-wma::wire_vlc: symbol {symbol} duplicates an earlier codeword"
                )
            }
            ExactVlcError::BadLength { symbol } => {
                write!(
                    f,
                    "oxideav-wma::wire_vlc: symbol {symbol} has a zero or >32 bit length"
                )
            }
            ExactVlcError::KraftIncomplete => {
                write!(
                    f,
                    "oxideav-wma::wire_vlc: table is not a complete prefix code"
                )
            }
        }
    }
}

impl std::error::Error for ExactVlcError {}

/// Decode failures for [`ExactVlc::decode_symbol`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VlcDecodeError {
    /// The bitstream ended inside a codeword.
    Bitstream(BitstreamEnd),
    /// No staged codeword matches (unreachable on a complete table
    /// with bits remaining; kept for defence in depth).
    InvalidCodeword,
}

impl core::fmt::Display for VlcDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            VlcDecodeError::Bitstream(e) => write!(f, "oxideav-wma::wire_vlc: {e}"),
            VlcDecodeError::InvalidCodeword => {
                write!(f, "oxideav-wma::wire_vlc: bits match no staged codeword")
            }
        }
    }
}

impl std::error::Error for VlcDecodeError {}

impl From<BitstreamEnd> for VlcDecodeError {
    fn from(e: BitstreamEnd) -> Self {
        VlcDecodeError::Bitstream(e)
    }
}

/// A prefix code realised from an explicit staged `(length, code)`
/// table — the vendor's own bit assignment, not a canonical
/// reconstruction.
#[derive(Debug, Clone)]
pub struct ExactVlc {
    entries: &'static [(u8, u32)],
    max_len: u8,
    /// `by_len[l]` maps a codeword of length `l` to its symbol.
    by_len: Vec<HashMap<u32, u16>>,
}

impl ExactVlc {
    /// Build and validate a decoder over a staged table.
    ///
    /// # Errors
    ///
    /// See [`ExactVlcError`]; unreachable for the staged tables.
    pub fn new(entries: &'static [(u8, u32)]) -> Result<Self, ExactVlcError> {
        let max_len = entries.iter().map(|&(l, _)| l).max().unwrap_or(0);
        if max_len == 0 || max_len > 32 {
            return Err(ExactVlcError::BadLength { symbol: 0 });
        }
        let mut by_len: Vec<HashMap<u32, u16>> = vec![HashMap::new(); usize::from(max_len) + 1];
        // Kraft sum in units of 2^-max_len.
        let mut kraft: u64 = 0;
        for (symbol, &(len, code)) in entries.iter().enumerate() {
            if len == 0 || len > max_len {
                return Err(ExactVlcError::BadLength { symbol });
            }
            if len < 32 && code >= (1u32 << len) {
                return Err(ExactVlcError::CodeOutOfRange { symbol });
            }
            let slot = u16::try_from(symbol).map_err(|_| ExactVlcError::BadLength { symbol })?;
            if by_len[usize::from(len)].insert(code, slot).is_some() {
                return Err(ExactVlcError::DuplicateCode { symbol });
            }
            kraft += 1u64 << (max_len - len);
        }
        if kraft != 1u64 << max_len {
            return Err(ExactVlcError::KraftIncomplete);
        }
        Ok(Self {
            entries,
            max_len,
            by_len,
        })
    }

    /// Symbol alphabet size.
    pub fn symbol_count(&self) -> usize {
        self.entries.len()
    }

    /// Longest codeword, in bits.
    pub fn max_len(&self) -> u8 {
        self.max_len
    }

    /// The staged `(length, code)` of `symbol`, if in range.
    pub fn entry(&self, symbol: usize) -> Option<(u8, u32)> {
        self.entries.get(symbol).copied()
    }

    /// Read one codeword MSB-first, returning its symbol.
    ///
    /// # Errors
    ///
    /// [`VlcDecodeError::Bitstream`] when the reader runs dry inside
    /// a codeword.
    pub fn decode_symbol(&self, reader: &mut BitReader<'_>) -> Result<u16, VlcDecodeError> {
        let mut code: u32 = 0;
        for len in 1..=self.max_len {
            code = (code << 1) | u32::from(reader.read_bit()?);
            if let Some(&symbol) = self.by_len[usize::from(len)].get(&code) {
                return Ok(symbol);
            }
        }
        Err(VlcDecodeError::InvalidCodeword)
    }

    /// Append `symbol`'s staged codeword to `writer`. Returns `false`
    /// (writing nothing) if the symbol is out of range.
    pub fn encode_symbol(&self, symbol: usize, writer: &mut BitWriter) -> bool {
        match self.entries.get(symbol) {
            Some(&(len, code)) => {
                writer.write_bits(u64::from(code), len);
                true
            }
            None => false,
        }
    }
}

fn singleton(cell: &'static OnceLock<ExactVlc>, table: &'static [(u8, u32)]) -> &'static ExactVlc {
    cell.get_or_init(|| ExactVlc::new(table).expect("staged table validated by tests"))
}

/// The coefficient VLC for `(class, alt)` — the staged six-descriptor
/// registration crossing (`frame-bit-layout.md` §4). Returns `None`
/// for a class outside `1..=3`.
pub fn coef_vlc(class: u8, alt: bool) -> Option<&'static ExactVlc> {
    static C1: OnceLock<ExactVlc> = OnceLock::new();
    static C2: OnceLock<ExactVlc> = OnceLock::new();
    static C3: OnceLock<ExactVlc> = OnceLock::new();
    static C1A: OnceLock<ExactVlc> = OnceLock::new();
    static C2A: OnceLock<ExactVlc> = OnceLock::new();
    static C3A: OnceLock<ExactVlc> = OnceLock::new();
    Some(match (class, alt) {
        (1, false) => singleton(&C1, &COEF_CODES_CLASS1),
        (2, false) => singleton(&C2, &COEF_CODES_CLASS2),
        (3, false) => singleton(&C3, &COEF_CODES_CLASS3),
        (1, true) => singleton(&C1A, &COEF_CODES_CLASS1_ALT),
        (2, true) => singleton(&C2A, &COEF_CODES_CLASS2_ALT),
        (3, true) => singleton(&C3A, &COEF_CODES_CLASS3_ALT),
        _ => return None,
    })
}

/// The staged 2-based `(run, |level|)` companion map for
/// `(class, alt)`; `map[s - 2]` expands coefficient symbol `s >= 2`.
pub fn runlevel_map(class: u8, alt: bool) -> Option<&'static [(u16, u16)]> {
    Some(match (class, alt) {
        (1, false) => &RUNLEVEL_CLASS1,
        (2, false) => &RUNLEVEL_CLASS2,
        (3, false) => &RUNLEVEL_CLASS3,
        (1, true) => &RUNLEVEL_CLASS1_ALT,
        (2, true) => &RUNLEVEL_CLASS2_ALT,
        (3, true) => &RUNLEVEL_CLASS3_ALT,
        _ => return None,
    })
}

/// The 121-symbol spectral-envelope exponent delta VLC
/// (delta = `symbol − 60`, `frame-bit-layout.md` §3).
pub fn scale_vlc() -> &'static ExactVlc {
    static SCALE: OnceLock<ExactVlc> = OnceLock::new();
    singleton(&SCALE, &SCALE_CODES)
}

/// The 37-symbol noise-band gain delta VLC (delta = `symbol − 18`,
/// `frame-bit-layout.md` §2.1).
pub fn gain_vlc() -> &'static ExactVlc {
    static GAIN: OnceLock<ExactVlc> = OnceLock::new();
    singleton(&GAIN, &GAIN_CODES)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::HuffmanCode;

    const ALL_CODE_TABLES: [(&str, &[(u8, u32)]); 8] = [
        ("class1", &COEF_CODES_CLASS1),
        ("class2", &COEF_CODES_CLASS2),
        ("class3", &COEF_CODES_CLASS3),
        ("class1_alt", &COEF_CODES_CLASS1_ALT),
        ("class2_alt", &COEF_CODES_CLASS2_ALT),
        ("class3_alt", &COEF_CODES_CLASS3_ALT),
        ("scale", &SCALE_CODES),
        ("gain", &GAIN_CODES),
    ];

    #[test]
    fn all_staged_tables_build_as_complete_prefix_codes() {
        for (name, table) in ALL_CODE_TABLES {
            let vlc = ExactVlc::new(table).unwrap_or_else(|e| panic!("{name}: {e}"));
            assert_eq!(vlc.symbol_count(), table.len(), "{name}");
        }
    }

    #[test]
    fn staged_alphabet_sizes_match_the_docs_index() {
        // docs/audio/wma/tables/README.md index row counts.
        assert_eq!(COEF_CODES_CLASS1.len(), 666);
        assert_eq!(COEF_CODES_CLASS2.len(), 1336);
        assert_eq!(COEF_CODES_CLASS3.len(), 476);
        assert_eq!(COEF_CODES_CLASS1_ALT.len(), 555);
        assert_eq!(COEF_CODES_CLASS2_ALT.len(), 1072);
        assert_eq!(COEF_CODES_CLASS3_ALT.len(), 435);
        assert_eq!(SCALE_CODES.len(), 121);
        assert_eq!(GAIN_CODES.len(), 37);
        assert_eq!(RUNLEVEL_CLASS1.len(), 664);
        assert_eq!(RUNLEVEL_CLASS2.len(), 1334);
        assert_eq!(RUNLEVEL_CLASS3.len(), 474);
        assert_eq!(RUNLEVEL_CLASS1_ALT.len(), 553);
        assert_eq!(RUNLEVEL_CLASS2_ALT.len(), 1070);
        assert_eq!(RUNLEVEL_CLASS3_ALT.len(), 433);
    }

    #[test]
    fn every_coef_table_pairs_with_a_companion_map_two_short() {
        // The companion maps are 2-based: alphabet = map + the two
        // reserved sentinels (escape = 0, EOB = 1 per §4).
        for (class, alt) in [
            (1, false),
            (2, false),
            (3, false),
            (1, true),
            (2, true),
            (3, true),
        ] {
            let vlc = coef_vlc(class, alt).unwrap();
            let map = runlevel_map(class, alt).unwrap();
            assert_eq!(vlc.symbol_count(), map.len() + 2, "class {class} alt {alt}");
        }
    }

    #[test]
    fn decode_round_trips_every_symbol_of_every_table() {
        for (name, table) in ALL_CODE_TABLES {
            let vlc = ExactVlc::new(table).unwrap();
            let mut w = crate::bitio::BitWriter::new();
            for s in 0..vlc.symbol_count() {
                assert!(vlc.encode_symbol(s, &mut w), "{name} symbol {s}");
            }
            let total_bits: usize = table.iter().map(|&(l, _)| usize::from(l)).sum();
            let bytes = w.into_bytes();
            let mut r = BitReader::with_bit_len(&bytes, total_bits);
            for s in 0..vlc.symbol_count() {
                assert_eq!(
                    vlc.decode_symbol(&mut r).unwrap(),
                    u16::try_from(s).unwrap(),
                    "{name} symbol {s}"
                );
            }
        }
    }

    #[test]
    fn complete_tables_resolve_arbitrary_bits() {
        // Kraft-complete ⇒ any bit pattern with enough bits decodes.
        let vlc = coef_vlc(3, false).unwrap();
        let bytes: Vec<u8> = (0..64u32).map(|i| (i * 37 + 11) as u8).collect();
        let mut r = BitReader::new(&bytes);
        while r.remaining_bits() >= usize::from(vlc.max_len()) {
            vlc.decode_symbol(&mut r).unwrap();
        }
    }

    #[test]
    fn vendor_assignment_is_not_the_canonical_reconstruction() {
        // The staged codes are the vendor's own bit assignment; the
        // canonical MSB-first rebuild from the same lengths differs
        // on every table (docs/audio/wma/tables/README.md). Guard the
        // fact so nobody "simplifies" back to from_lengths.
        for (name, table) in ALL_CODE_TABLES {
            let lengths: Vec<u8> = table.iter().map(|&(l, _)| l).collect();
            let canonical = HuffmanCode::from_lengths(&lengths).unwrap();
            let differs =
                (0..table.len()).any(|s| canonical.code_of(s) != Some(u64::from(table[s].1)));
            assert!(
                differs,
                "{name}: canonical reconstruction unexpectedly matches"
            );
        }
    }

    #[test]
    fn scale_and_gain_spot_checks_against_the_staged_csvs() {
        // First rows of wma-huffman-scale-codelen.csv /
        // wma-huffman-gain-codelen.csv.
        assert_eq!(SCALE_CODES[0], (18, 0b111111111111101000));
        assert_eq!(SCALE_CODES[60], SCALE_CODES[60]); // center exists
        assert_eq!(GAIN_CODES.len(), 37);
        // §3 zero-delta center: symbol 60 carries the shortest code.
        let min_len = SCALE_CODES.iter().map(|&(l, _)| l).min().unwrap();
        assert_eq!(SCALE_CODES[60].0, min_len);
    }

    #[test]
    fn runlevel_maps_ramp_within_constant_level_groups() {
        // Staged layout property: RUN counts up within each
        // constant-LEVEL group; LEVEL is nondecreasing group-to-group.
        for (class, alt) in [
            (1, false),
            (2, false),
            (3, false),
            (1, true),
            (2, true),
            (3, true),
        ] {
            let map = runlevel_map(class, alt).unwrap();
            for (i, w) in map.windows(2).enumerate() {
                let (r0, l0) = w[0];
                let (r1, l1) = w[1];
                // The two class-2 maps close with a literal (0, 0)
                // row — staged as-is, outside the ramp.
                if i + 2 == map.len() && (r1, l1) == (0, 0) {
                    continue;
                }
                assert!(
                    (l1 == l0 && r1 == r0 + 1) || (l1 > l0),
                    "class {class} alt {alt}: ({r0},{l0}) -> ({r1},{l1})"
                );
            }
        }
    }
}

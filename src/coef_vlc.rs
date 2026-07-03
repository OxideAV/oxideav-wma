//! Coefficient run-level VLCs built from the staged real tables.
//!
//! ## Source
//!
//! The per-symbol code *lengths* live in [`crate::wire_tables`]
//! (transcribed from `docs/audio/wma/tables/`, extracted as data from
//! the vendor WMA Standard decoder module). The staged notes pin the
//! surrounding facts this module realises:
//!
//! * The vendor module registers **one of three coefficient decode
//!   trees** selected by a per-stream decode mode in `{1, 2, 3}` —
//!   this module's [`CoefDecodeMode`].
//! * Each tree leaf carries `(symbol, length)`; the staged CSVs emit
//!   the exact lengths plus the **canonical MSB-first `(length,
//!   symbol)`** codeword reconstruction. That is precisely the
//!   arrangement [`HuffmanCode::from_lengths`] realises, so building
//!   from the staged lengths reproduces the staged codewords
//!   bit-for-bit (pinned by test below).
//!
//! ## What stays `[GAP]`
//!
//! * **Mode 2 cannot be constructed yet**: its escape codewords
//!   (symbols `1016..=1023`, recurring at several lengths) are the
//!   extraction's documented residual, so the real-symbol lengths do
//!   not form a complete prefix code. Building a decoder over them
//!   would misparse any stream that uses an escape codeword;
//!   [`CoefVlc::new`] returns
//!   [`CoefVlcError::Mode2EscapeEnumerationUnstaged`] instead of
//!   guessing. Data-staging gap, not a code gap.
//! * The **symbol → `(R, L)` mapping** (the vendor module's companion
//!   index-ramp tables, located but not pinned), so decoded symbols
//!   are surfaced as raw symbol indices, not run-level pairs.
//! * How the decode mode is **selected from the stream header**
//!   (bitrate/rate classes) is not in the staged material; the mode
//!   is caller-supplied.
//! * The vendor tree's internal **bit assignment** is not yet
//!   verified against the canonical reconstruction (documented
//!   residual); the lengths are exact data either way.

use crate::bitio::{BitReader, BitWriter};
use crate::huffman::{HuffmanCode, HuffmanError};
use crate::wire_tables::{
    COEF_VLC_MODE1_LENGTHS, COEF_VLC_MODE2_REAL_LENGTHS, COEF_VLC_MODE3_LENGTHS,
};

/// The vendor module's per-stream coefficient decode mode: it
/// registers one of three coefficient VLC trees selected by a context
/// value in `{1, 2, 3}` (staged provenance notes). How the mode is
/// derived from the stream header is `[GAP]`, so callers supply it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CoefDecodeMode {
    /// Decode mode 1 — 666-symbol coefficient VLC.
    Mode1,
    /// Decode mode 2 — 1016 real symbols + 8 escape symbols
    /// (escape enumeration unstaged; not yet constructible).
    Mode2,
    /// Decode mode 3 — 476-symbol coefficient VLC.
    Mode3,
}

impl CoefDecodeMode {
    /// All three vendor decode modes.
    pub const ALL: [CoefDecodeMode; 3] = [
        CoefDecodeMode::Mode1,
        CoefDecodeMode::Mode2,
        CoefDecodeMode::Mode3,
    ];

    /// The staged per-symbol code-length table for this mode. For
    /// [`CoefDecodeMode::Mode2`] this is the **incomplete** real-symbol
    /// table (see [`crate::wire_tables::COEF_VLC_MODE2_REAL_LENGTHS`]).
    pub fn lengths(self) -> &'static [u8] {
        match self {
            CoefDecodeMode::Mode1 => &COEF_VLC_MODE1_LENGTHS,
            CoefDecodeMode::Mode2 => &COEF_VLC_MODE2_REAL_LENGTHS,
            CoefDecodeMode::Mode3 => &COEF_VLC_MODE3_LENGTHS,
        }
    }

    /// The mode's context value in the vendor module (`1..=3`).
    pub fn context_value(self) -> u8 {
        match self {
            CoefDecodeMode::Mode1 => 1,
            CoefDecodeMode::Mode2 => 2,
            CoefDecodeMode::Mode3 => 3,
        }
    }
}

/// A coefficient run-level VLC realised from the staged length table
/// of one [`CoefDecodeMode`] — the first wire-data-backed entropy
/// front end in the crate.
///
/// Symbols are raw VLC symbol indices (`0..symbol_count()`); the
/// symbol → `(R, L)` expansion is `[GAP]` (see module docs).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CoefVlc {
    mode: CoefDecodeMode,
    code: HuffmanCode,
}

/// Failure modes for [`CoefVlc`] construction and coding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoefVlcError {
    /// [`CoefDecodeMode::Mode2`] was requested, but its escape
    /// codeword enumeration is unstaged so the real-symbol lengths do
    /// not form a complete prefix code. Blocked on docs staging.
    Mode2EscapeEnumerationUnstaged,
    /// The underlying canonical-code machinery failed (propagated
    /// unchanged from [`crate::huffman`]).
    Huffman(HuffmanError),
}

impl core::fmt::Display for CoefVlcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CoefVlcError::Mode2EscapeEnumerationUnstaged => f.write_str(
                "oxideav-wma::coef_vlc: decode mode 2 is blocked on the unstaged escape codeword enumeration (docs/audio/wma/tables/wma-huffman-coef-mode2-codelen.meta)",
            ),
            CoefVlcError::Huffman(e) => write!(f, "oxideav-wma::coef_vlc: {e}"),
        }
    }
}

impl std::error::Error for CoefVlcError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            CoefVlcError::Huffman(e) => Some(e),
            _ => None,
        }
    }
}

impl From<HuffmanError> for CoefVlcError {
    fn from(e: HuffmanError) -> Self {
        CoefVlcError::Huffman(e)
    }
}

impl CoefVlc {
    /// Build the coefficient VLC for `mode` from the staged length
    /// table.
    ///
    /// # Errors
    ///
    /// [`CoefVlcError::Mode2EscapeEnumerationUnstaged`] for
    /// [`CoefDecodeMode::Mode2`] — its table is documented-incomplete
    /// and a decoder built over it would misparse escape codewords.
    pub fn new(mode: CoefDecodeMode) -> Result<Self, CoefVlcError> {
        if mode == CoefDecodeMode::Mode2 {
            return Err(CoefVlcError::Mode2EscapeEnumerationUnstaged);
        }
        let code = HuffmanCode::from_lengths(mode.lengths())?;
        Ok(Self { mode, code })
    }

    /// The decode mode this VLC realises.
    pub fn mode(&self) -> CoefDecodeMode {
        self.mode
    }

    /// Symbol alphabet size (666 for mode 1, 476 for mode 3).
    pub fn symbol_count(&self) -> usize {
        self.code.len()
    }

    /// Longest codeword in bits (20 for mode 1, 21 for mode 3).
    pub fn max_len(&self) -> u8 {
        self.code.max_len()
    }

    /// The underlying canonical code (lengths exact from the staged
    /// data; codewords the canonical MSB-first reconstruction).
    pub fn code(&self) -> &HuffmanCode {
        &self.code
    }

    /// Append `symbol`'s codeword to `writer`.
    ///
    /// # Errors
    ///
    /// Propagates [`HuffmanError::SymbolOutOfRange`] for a symbol
    /// outside the mode's alphabet.
    pub fn encode_symbol(&self, symbol: usize, writer: &mut BitWriter) -> Result<(), CoefVlcError> {
        self.code.encode_symbol(symbol, writer)?;
        Ok(())
    }

    /// Read one codeword from `reader`, returning its symbol index.
    ///
    /// # Errors
    ///
    /// Propagates [`HuffmanError::Bitstream`] if the stream ends
    /// inside a codeword.
    pub fn decode_symbol(&self, reader: &mut BitReader<'_>) -> Result<usize, CoefVlcError> {
        Ok(self.code.decode_symbol(reader)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parse a staged-CSV binary codeword string into (value, length).
    fn bits(s: &str) -> (u64, u8) {
        (
            u64::from_str_radix(s, 2).unwrap(),
            u8::try_from(s.len()).unwrap(),
        )
    }

    fn assert_code(vlc: &CoefVlc, symbol: usize, csv_code: &str) {
        let (value, len) = bits(csv_code);
        assert_eq!(
            vlc.code().length_of(symbol),
            Some(len),
            "mode {:?} symbol {symbol} length",
            vlc.mode()
        );
        assert_eq!(
            vlc.code().code_of(symbol),
            Some(value),
            "mode {:?} symbol {symbol} codeword",
            vlc.mode()
        );
    }

    #[test]
    fn mode1_reproduces_the_staged_csv_codewords() {
        let vlc = CoefVlc::new(CoefDecodeMode::Mode1).unwrap();
        assert_eq!(vlc.symbol_count(), 666);
        assert_eq!(vlc.max_len(), 20);
        // Spot pins: rows copied verbatim from
        // docs/audio/wma/tables/wma-huffman-coef-mode1-codelen.csv.
        assert_code(&vlc, 0, "11110100110");
        assert_code(&vlc, 1, "101010");
        assert_code(&vlc, 2, "00");
        assert_code(&vlc, 3, "010");
        assert_code(&vlc, 19, "1101000");
        assert_code(&vlc, 333, "111110100011");
        assert_code(&vlc, 665, "11111111111101110");
    }

    #[test]
    fn mode3_reproduces_the_staged_csv_codewords() {
        let vlc = CoefVlc::new(CoefDecodeMode::Mode3).unwrap();
        assert_eq!(vlc.symbol_count(), 476);
        assert_eq!(vlc.max_len(), 21);
        // Spot pins from wma-huffman-coef-mode3-codelen.csv.
        assert_code(&vlc, 0, "111110101000");
        assert_code(&vlc, 1, "101100");
        assert_code(&vlc, 2, "00");
        assert_code(&vlc, 3, "010");
        assert_code(&vlc, 238, "111111000110");
        assert_code(&vlc, 475, "1111111111111111110");
    }

    #[test]
    fn mode2_is_a_typed_docs_gap() {
        assert_eq!(
            CoefVlc::new(CoefDecodeMode::Mode2),
            Err(CoefVlcError::Mode2EscapeEnumerationUnstaged)
        );
        let msg = format!("{}", CoefVlcError::Mode2EscapeEnumerationUnstaged);
        assert!(msg.contains("escape codeword enumeration"));
    }

    #[test]
    fn mode_metadata_matches_the_staged_tables() {
        assert_eq!(CoefDecodeMode::Mode1.lengths().len(), 666);
        assert_eq!(CoefDecodeMode::Mode2.lengths().len(), 1016);
        assert_eq!(CoefDecodeMode::Mode3.lengths().len(), 476);
        for (i, mode) in CoefDecodeMode::ALL.iter().enumerate() {
            assert_eq!(mode.context_value() as usize, i + 1);
        }
    }

    #[test]
    fn real_table_symbol_streams_round_trip_through_bits() {
        for mode in [CoefDecodeMode::Mode1, CoefDecodeMode::Mode3] {
            let vlc = CoefVlc::new(mode).unwrap();
            // A deterministic pseudo-random symbol stream over the full
            // alphabet — the encoder-mirror self-consistency oracle.
            let mut state = 0xC0EF_u64;
            let symbols: Vec<usize> = (0..2000)
                .map(|_| {
                    state = state
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    (state >> 33) as usize % vlc.symbol_count()
                })
                .collect();
            let mut w = BitWriter::new();
            for &s in &symbols {
                vlc.encode_symbol(s, &mut w).unwrap();
            }
            let bit_len = w.bit_len();
            let bytes = w.into_bytes();
            let mut r = BitReader::with_bit_len(&bytes, bit_len);
            for &s in &symbols {
                assert_eq!(vlc.decode_symbol(&mut r).unwrap(), s, "mode {mode:?}");
            }
            assert_eq!(r.remaining_bits(), 0);
        }
    }

    #[test]
    fn truncated_stream_fails_cleanly() {
        let vlc = CoefVlc::new(CoefDecodeMode::Mode1).unwrap();
        let mut w = BitWriter::new();
        vlc.encode_symbol(0, &mut w).unwrap(); // 11 bits
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, 5);
        assert!(matches!(
            vlc.decode_symbol(&mut r),
            Err(CoefVlcError::Huffman(HuffmanError::Bitstream(_)))
        ));
    }

    #[test]
    fn every_short_codeword_is_no_less_probable_than_longer_ones_exist() {
        // Structural sanity on the staged data: both complete tables
        // put their shortest code on a low symbol (the most frequent
        // run-level event) — symbol 2 carries the 2-bit code in both.
        for mode in [CoefDecodeMode::Mode1, CoefDecodeMode::Mode3] {
            let vlc = CoefVlc::new(mode).unwrap();
            let min_len = (0..vlc.symbol_count())
                .map(|s| vlc.code().length_of(s).unwrap())
                .min()
                .unwrap();
            assert_eq!(min_len, 2);
            assert_eq!(vlc.code().length_of(2), Some(2), "mode {mode:?}");
        }
    }
}

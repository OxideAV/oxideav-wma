//! Coefficient run-level VLCs built from the staged real tables.
//!
//! ## Source
//!
//! The per-symbol code *lengths* live in [`crate::wire_tables`] and
//! the symbol → `(run, |level|)` companion maps in
//! [`crate::runlevel_tables`] (both transcribed from
//! `docs/audio/wma/tables/`, extracted as data from the vendor WMA
//! Standard decoder module). The staged notes pin the surrounding
//! facts this module realises:
//!
//! * The vendor module registers one coefficient decode tree selected
//!   by a **decode class** in `{1, 2, 3}` crossed with an **alt
//!   variant** configuration flag — this module's [`CoefDecodeMode`].
//! * Each tree leaf carries `(symbol, length)`; the staged CSVs emit
//!   the exact lengths plus the **canonical MSB-first `(length,
//!   symbol)`** codeword reconstruction. That is precisely the
//!   arrangement [`HuffmanCode::from_lengths`] realises, so building
//!   from the staged lengths reproduces the staged codewords
//!   bit-for-bit (pinned by test below).
//! * Symbols 0 and 1 are the reserved **end-of-block** / **escape**
//!   sentinels (the staged §4e correction); symbols `>= 2` map to
//!   `(run, |level|)` pairs through the 2-based companion tables.
//!   [`CoefVlc::expand`] surfaces that as a typed [`CoefEvent`].
//! * The coefficient **sign** is not in the VLC symbol: it is a
//!   separate trailing bit per non-zero coefficient (realised by the
//!   frame-layout parser, not here).
//!
//! ## Mode 2 (decode class 2 primary)
//!
//! Mode 2 is built over its 1016 staged symbols (`0..=1015`, exact
//! lengths) with [`HuffmanCode::from_lengths_prefix`]: the vendor
//! decode table is a space-shared DAG that replicates a few high
//! symbols across several code lengths, so the flat scan leaves
//! `COEF_VLC_MODE2_KRAFT_DEFICIT / 2^22` of the code space
//! unassigned. Decoding a real stream that lands in that unassigned
//! space surfaces [`HuffmanError::InvalidCodeword`] — the documented
//! static residual (no codeword is *missing*; the exact codes of the
//! replicated symbols are what stays unpinned).
//!
//! ## What stays `[GAP]`
//!
//! * The **class-2 alt-variant** VLC (located in the vendor module,
//!   not staged), so [`CoefDecodeMode`] has no variant for it.
//! * The **alt variants' run/level companion tables** (their tree
//!   lengths are staged; their `+0x18`/`+0x1c` companions are not),
//!   so [`CoefVlc::expand`] is a typed docs-gap for the alt modes.
//! * The vendor tree's internal **bit assignment** is not yet
//!   verified against the canonical reconstruction (documented
//!   residual); the lengths are exact data either way.

use crate::bitio::{BitReader, BitWriter};
use crate::huffman::{HuffmanCode, HuffmanError};
use crate::runlevel_tables::{RunLevel, RUNLEVEL_MODE1, RUNLEVEL_MODE2, RUNLEVEL_MODE3};
use crate::wire_tables::{
    COEF_EOB_SYMBOL, COEF_ESCAPE_SYMBOL, COEF_RUNLEVEL_BASE_SYMBOL, COEF_VLC_CLASS1_ALT_LENGTHS,
    COEF_VLC_CLASS3_ALT_LENGTHS, COEF_VLC_MODE1_LENGTHS, COEF_VLC_MODE2_REAL_LENGTHS,
    COEF_VLC_MODE3_LENGTHS,
};

/// The vendor module's coefficient decode-table selector: a decode
/// class in `{1, 2, 3}` crossed with an alt-variant configuration
/// flag. Five of the six descriptors are staged (the class-2 alt
/// variant is located but unextracted, so it has no variant here).
///
/// The class ← `f(sample_rate, bitrate)` selection rule is realised
/// in [`crate::wire_chain`]; the alt flag is per-configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CoefDecodeMode {
    /// Decode class 1, primary tree — 666-symbol coefficient VLC.
    Mode1,
    /// Decode class 2, primary tree — 1016 staged symbols of the
    /// 1024-symbol alphabet (see the module docs on the DAG residual).
    Mode2,
    /// Decode class 3, primary tree — 476-symbol coefficient VLC.
    Mode3,
    /// Decode class 1, alt variant — 555-symbol coefficient VLC
    /// (registered when the vendor `params+0x380 == 1` flag is set).
    Class1Alt,
    /// Decode class 3, alt variant — 435-symbol coefficient VLC.
    Class3Alt,
}

impl CoefDecodeMode {
    /// All five staged decode-table selections.
    pub const ALL: [CoefDecodeMode; 5] = [
        CoefDecodeMode::Mode1,
        CoefDecodeMode::Mode2,
        CoefDecodeMode::Mode3,
        CoefDecodeMode::Class1Alt,
        CoefDecodeMode::Class3Alt,
    ];

    /// The staged per-symbol code-length table for this selection.
    /// For [`CoefDecodeMode::Mode2`] this is the documented-incomplete
    /// flat scan (see [`crate::wire_tables::COEF_VLC_MODE2_REAL_LENGTHS`]).
    pub fn lengths(self) -> &'static [u8] {
        match self {
            CoefDecodeMode::Mode1 => &COEF_VLC_MODE1_LENGTHS,
            CoefDecodeMode::Mode2 => &COEF_VLC_MODE2_REAL_LENGTHS,
            CoefDecodeMode::Mode3 => &COEF_VLC_MODE3_LENGTHS,
            CoefDecodeMode::Class1Alt => &COEF_VLC_CLASS1_ALT_LENGTHS,
            CoefDecodeMode::Class3Alt => &COEF_VLC_CLASS3_ALT_LENGTHS,
        }
    }

    /// The staged symbol → `(run, |level|)` companion map for this
    /// selection, or `None` for the alt variants whose companions are
    /// not staged (a typed docs-gap, see the module docs).
    pub fn runlevel_map(self) -> Option<&'static [RunLevel]> {
        match self {
            CoefDecodeMode::Mode1 => Some(&RUNLEVEL_MODE1),
            CoefDecodeMode::Mode2 => Some(&RUNLEVEL_MODE2),
            CoefDecodeMode::Mode3 => Some(&RUNLEVEL_MODE3),
            CoefDecodeMode::Class1Alt | CoefDecodeMode::Class3Alt => None,
        }
    }

    /// The selection's decode class in the vendor module (`1..=3`).
    pub fn class(self) -> u8 {
        match self {
            CoefDecodeMode::Mode1 | CoefDecodeMode::Class1Alt => 1,
            CoefDecodeMode::Mode2 => 2,
            CoefDecodeMode::Mode3 | CoefDecodeMode::Class3Alt => 3,
        }
    }

    /// Whether this is the alt-variant tree of its class.
    pub fn is_alt(self) -> bool {
        matches!(self, CoefDecodeMode::Class1Alt | CoefDecodeMode::Class3Alt)
    }

    /// The mode's class context value in the vendor module (`1..=3`).
    /// Kept as the historical accessor name; equals [`Self::class`].
    pub fn context_value(self) -> u8 {
        self.class()
    }
}

/// A decoded coefficient-VLC symbol classified per the staged
/// sentinel/companion-table facts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoefEvent {
    /// Symbol 0 — the reserved end-of-block sentinel.
    EndOfBlock,
    /// Symbol 1 — the reserved escape sentinel; a literal run and a
    /// literal level follow in the bitstream at the runtime-signalled
    /// widths (read by the frame-layout parser, not here).
    Escape,
    /// A real `(run, |level|)` pair from the companion table. The
    /// sign is a separate trailing bit in the bitstream.
    Pair {
        /// Zero-run preceding the coefficient.
        run: u16,
        /// Coefficient magnitude (`>= 1`).
        abs_level: u16,
    },
}

/// A coefficient run-level VLC realised from the staged length table
/// of one [`CoefDecodeMode`] — the wire-data entropy front end.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CoefVlc {
    mode: CoefDecodeMode,
    code: HuffmanCode,
}

/// Failure modes for [`CoefVlc`] construction and coding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoefVlcError {
    /// A symbol's `(run, |level|)` expansion was requested but the
    /// mode's companion table is not staged (the alt variants) or the
    /// symbol lies outside it.
    RunLevelUnavailable {
        /// The offending mode.
        mode: CoefDecodeMode,
        /// The symbol whose expansion was requested.
        symbol: usize,
    },
    /// The underlying canonical-code machinery failed (propagated
    /// unchanged from [`crate::huffman`]).
    Huffman(HuffmanError),
}

impl core::fmt::Display for CoefVlcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CoefVlcError::RunLevelUnavailable { mode, symbol } => write!(
                f,
                "oxideav-wma::coef_vlc: no staged (run, level) expansion for symbol {symbol} of mode {mode:?} (alt-variant companion tables are a docs-staging gap)",
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
    /// table. [`CoefDecodeMode::Mode2`] builds as an **incomplete**
    /// canonical prefix code over its 1016 staged symbols (see the
    /// module docs); every other mode is Kraft-complete.
    ///
    /// # Errors
    ///
    /// Propagates [`HuffmanError`] — unreachable for the staged
    /// tables (pinned by test).
    pub fn new(mode: CoefDecodeMode) -> Result<Self, CoefVlcError> {
        let code = if mode == CoefDecodeMode::Mode2 {
            HuffmanCode::from_lengths_prefix(mode.lengths())?
        } else {
            HuffmanCode::from_lengths(mode.lengths())?
        };
        Ok(Self { mode, code })
    }

    /// The decode mode this VLC realises.
    pub fn mode(&self) -> CoefDecodeMode {
        self.mode
    }

    /// Symbol alphabet size (the staged table's row count).
    pub fn symbol_count(&self) -> usize {
        self.code.len()
    }

    /// Longest codeword in bits.
    pub fn max_len(&self) -> u8 {
        self.code.max_len()
    }

    /// The underlying canonical code (lengths exact from the staged
    /// data; codewords the canonical MSB-first reconstruction).
    pub fn code(&self) -> &HuffmanCode {
        &self.code
    }

    /// Classify a decoded symbol per the staged sentinel and
    /// companion-table facts: symbol 0 → [`CoefEvent::EndOfBlock`],
    /// symbol 1 → [`CoefEvent::Escape`], symbol `s >= 2` →
    /// [`CoefEvent::Pair`] via the 2-based companion table.
    ///
    /// # Errors
    ///
    /// [`CoefVlcError::RunLevelUnavailable`] when the mode's
    /// companion table is unstaged (alt variants) or the symbol lies
    /// outside it.
    pub fn expand(&self, symbol: usize) -> Result<CoefEvent, CoefVlcError> {
        if symbol == usize::from(COEF_EOB_SYMBOL) {
            return Ok(CoefEvent::EndOfBlock);
        }
        if symbol == usize::from(COEF_ESCAPE_SYMBOL) {
            return Ok(CoefEvent::Escape);
        }
        let map = self
            .mode
            .runlevel_map()
            .ok_or(CoefVlcError::RunLevelUnavailable {
                mode: self.mode,
                symbol,
            })?;
        let (run, abs_level) = *map
            .get(symbol - usize::from(COEF_RUNLEVEL_BASE_SYMBOL))
            .ok_or(CoefVlcError::RunLevelUnavailable {
                mode: self.mode,
                symbol,
            })?;
        Ok(CoefEvent::Pair { run, abs_level })
    }

    /// Find the VLC symbol carrying `(run, abs_level)`, if the pair
    /// is in this mode's staged companion table **and** within the
    /// VLC alphabet (mode 2's companion tail beyond the alphabet is
    /// escape-reachable only, so it does not qualify).
    pub fn symbol_for_pair(&self, run: u16, abs_level: u16) -> Option<usize> {
        let map = self.mode.runlevel_map()?;
        let limit = self
            .symbol_count()
            .saturating_sub(usize::from(COEF_RUNLEVEL_BASE_SYMBOL));
        map.iter()
            .take(limit)
            .position(|&(r, l)| r == run && l == abs_level)
            .map(|i| i + usize::from(COEF_RUNLEVEL_BASE_SYMBOL))
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
    /// * Propagates [`HuffmanError::Bitstream`] if the stream ends
    ///   inside a codeword.
    /// * [`HuffmanError::InvalidCodeword`] if the bits land in mode
    ///   2's unassigned (DAG-replicated) code space.
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
    fn mode2_builds_and_reproduces_the_staged_csv_codewords() {
        let vlc = CoefVlc::new(CoefDecodeMode::Mode2).unwrap();
        assert_eq!(vlc.symbol_count(), 1016);
        assert_eq!(vlc.max_len(), 22);
        // Spot pins from wma-huffman-coef-mode2-codelen.csv
        // (the corrected staging — canonical over symbols 0..=1015).
        assert_code(&vlc, 0, "11110110110");
        assert_code(&vlc, 1, "111011010");
        assert_code(&vlc, 2, "00");
        assert_code(&vlc, 3, "010");
        assert_code(&vlc, 1015, "1111110011101");
    }

    #[test]
    fn alt_variants_reproduce_the_staged_csv_codewords() {
        let c1 = CoefVlc::new(CoefDecodeMode::Class1Alt).unwrap();
        assert_eq!(c1.symbol_count(), 555);
        assert_eq!(c1.max_len(), 19);
        // Spot pins from wma-huffman-coef-class1-alt-codelen.csv.
        assert_code(&c1, 0, "110110110");
        assert_code(&c1, 2, "00");
        assert_code(&c1, 554, "111111111101111");

        let c3 = CoefVlc::new(CoefDecodeMode::Class3Alt).unwrap();
        assert_eq!(c3.symbol_count(), 435);
        assert_eq!(c3.max_len(), 18);
        // Spot pins from wma-huffman-coef-class3-alt-codelen.csv.
        assert_code(&c3, 0, "1110111000");
        assert_code(&c3, 2, "00");
        assert_code(&c3, 434, "1111111111111010");
    }

    #[test]
    fn mode_metadata_matches_the_staged_tables() {
        assert_eq!(CoefDecodeMode::Mode1.lengths().len(), 666);
        assert_eq!(CoefDecodeMode::Mode2.lengths().len(), 1016);
        assert_eq!(CoefDecodeMode::Mode3.lengths().len(), 476);
        assert_eq!(CoefDecodeMode::Class1Alt.lengths().len(), 555);
        assert_eq!(CoefDecodeMode::Class3Alt.lengths().len(), 435);
        for mode in CoefDecodeMode::ALL {
            assert_eq!(mode.context_value(), mode.class());
            assert_eq!(
                mode.runlevel_map().is_none(),
                mode.is_alt(),
                "companion maps are staged exactly for the primaries"
            );
        }
        assert_eq!(CoefDecodeMode::Mode1.class(), 1);
        assert_eq!(CoefDecodeMode::Mode2.class(), 2);
        assert_eq!(CoefDecodeMode::Mode3.class(), 3);
        assert_eq!(CoefDecodeMode::Class1Alt.class(), 1);
        assert_eq!(CoefDecodeMode::Class3Alt.class(), 3);
    }

    #[test]
    fn expand_classifies_sentinels_and_pairs() {
        let vlc = CoefVlc::new(CoefDecodeMode::Mode2).unwrap();
        assert_eq!(vlc.expand(0).unwrap(), CoefEvent::EndOfBlock);
        assert_eq!(vlc.expand(1).unwrap(), CoefEvent::Escape);
        // First real symbol: (run 0, |level| 1).
        assert_eq!(
            vlc.expand(2).unwrap(),
            CoefEvent::Pair {
                run: 0,
                abs_level: 1
            }
        );
        // The staged correction's worked examples.
        assert_eq!(
            vlc.expand(1016).unwrap(),
            CoefEvent::Pair {
                run: 1,
                abs_level: 51
            }
        );
        assert_eq!(
            vlc.expand(1023).unwrap(),
            CoefEvent::Pair {
                run: 0,
                abs_level: 55
            }
        );
    }

    #[test]
    fn expand_on_alt_variants_is_a_typed_docs_gap() {
        let vlc = CoefVlc::new(CoefDecodeMode::Class1Alt).unwrap();
        assert_eq!(vlc.expand(0).unwrap(), CoefEvent::EndOfBlock);
        assert_eq!(vlc.expand(1).unwrap(), CoefEvent::Escape);
        assert_eq!(
            vlc.expand(2),
            Err(CoefVlcError::RunLevelUnavailable {
                mode: CoefDecodeMode::Class1Alt,
                symbol: 2
            })
        );
        let msg = format!("{}", vlc.expand(2).unwrap_err());
        assert!(msg.contains("docs-staging gap"));
    }

    #[test]
    fn symbol_for_pair_inverts_expand_within_the_alphabet() {
        for mode in [
            CoefDecodeMode::Mode1,
            CoefDecodeMode::Mode2,
            CoefDecodeMode::Mode3,
        ] {
            let vlc = CoefVlc::new(mode).unwrap();
            for symbol in [2usize, 3, 57, vlc.symbol_count() - 1] {
                let CoefEvent::Pair { run, abs_level } = vlc.expand(symbol).unwrap() else {
                    panic!("symbol {symbol} must be a pair");
                };
                assert_eq!(
                    vlc.symbol_for_pair(run, abs_level),
                    Some(symbol),
                    "mode {mode:?} symbol {symbol}"
                );
            }
        }
    }

    #[test]
    fn mode2_companion_tail_is_escape_only() {
        // Companion entries beyond the 1024-symbol alphabet must not
        // be claimed by symbol_for_pair — they are escape-reachable
        // only per the staged provenance.
        let vlc = CoefVlc::new(CoefDecodeMode::Mode2).unwrap();
        let map = CoefDecodeMode::Mode2.runlevel_map().unwrap();
        let (run, abs_level) = map[1332]; // symbol 1334 — beyond the alphabet
        assert_eq!((run, abs_level), (0, 339));
        assert_eq!(vlc.symbol_for_pair(run, abs_level), None);
    }

    #[test]
    fn real_table_symbol_streams_round_trip_through_bits() {
        for mode in CoefDecodeMode::ALL {
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
    fn mode2_unassigned_code_space_is_a_clean_decode_error() {
        // The all-ones 22-bit pattern is outside every canonical range
        // of the incomplete mode-2 code (the DAG-replication room).
        let vlc = CoefVlc::new(CoefDecodeMode::Mode2).unwrap();
        let bytes = [0xFF, 0xFF, 0xFF];
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            vlc.decode_symbol(&mut r),
            Err(CoefVlcError::Huffman(HuffmanError::InvalidCodeword))
        );
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
    fn every_mode_puts_its_shortest_code_on_the_first_pair_symbol() {
        // Structural sanity on the staged data: all five tables put
        // their 2-bit shortest code on symbol 2 — the (0, 1) pair,
        // the most frequent run-level event.
        for mode in CoefDecodeMode::ALL {
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

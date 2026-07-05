//! The scale-factor (exponent) and gain delta VLCs — the two
//! envelope-side Huffman codes of the per-block bit layout.
//!
//! ## Source
//!
//! Per-symbol code lengths transcribed from the staged extraction
//! (`docs/audio/wma/tables/wma-huffman-{scale,gain}-codelen.csv`,
//! carried in [`crate::wire_tables`]); the surrounding structural
//! facts are from the staged frame-layout trace
//! (`docs/audio/wma/frame-bit-layout.md` §2):
//!
//! * The **gain** VLC (37 symbols, Kraft-complete) codes the
//!   per-block gain deltas — sub-stream B2, right after the 7-bit
//!   block header.
//! * The **scale** VLC (121 symbols, Kraft-complete) codes the
//!   spectral-envelope exponent deltas — sub-stream B5, after the
//!   5-bit envelope base field.
//! * Both are read by the same tree walker as the coefficient VLCs,
//!   so the canonical MSB-first reconstruction applies identically.
//!
//! ## Delta convention (documented realization detail)
//!
//! Both alphabets are odd-sized and delta-shaped. For the scale VLC
//! the staged data itself pins the center: the unique 1-bit codeword
//! sits on symbol 60, the midpoint of the 121-symbol alphabet — the
//! most-probable zero-delta event. For the gain VLC the alphabet
//! midpoint is symbol 18 (37 = 2·18 + 1) and the shortest codes
//! cluster around it. [`ScaleVlc::delta_of`] / [`GainVlc::delta_of`]
//! expose the `symbol - center` reading as the *symmetric-alphabet
//! convention*; how a decoded delta chains into the running
//! exponent/gain value (initial value, per-band order, wrap rule) is
//! **not** in the staged material — a documented `[GAP]` the caller
//! owns.

use crate::bitio::{BitReader, BitWriter};
use crate::huffman::{HuffmanCode, HuffmanError};
use crate::wire_tables::{GAIN_VLC_LENGTHS, SCALE_VLC_LENGTHS};

/// Center symbol of the 121-symbol scale delta alphabet — carries the
/// unique 1-bit codeword (the staged data's own zero-delta pin).
pub const SCALE_DELTA_CENTER: usize = 60;

/// Center symbol of the 37-symbol gain delta alphabet (the
/// symmetric-alphabet midpoint; a documented convention, not a staged
/// fact — see the module docs).
pub const GAIN_DELTA_CENTER: usize = 18;

macro_rules! envelope_vlc {
    ($name:ident, $lengths:expr, $center:expr, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name {
            code: HuffmanCode,
        }

        impl $name {
            /// Build the VLC from the staged length table
            /// (Kraft-complete — construction cannot fail; pinned by
            /// test).
            pub fn new() -> Self {
                let code =
                    HuffmanCode::from_lengths(&$lengths).expect("staged table is Kraft-complete");
                Self { code }
            }

            /// Symbol alphabet size.
            pub fn symbol_count(&self) -> usize {
                self.code.len()
            }

            /// Longest codeword in bits.
            pub fn max_len(&self) -> u8 {
                self.code.max_len()
            }

            /// The underlying canonical code.
            pub fn code(&self) -> &HuffmanCode {
                &self.code
            }

            /// The symmetric-alphabet delta reading of a symbol:
            /// `symbol - center` (see the module docs for what this
            /// convention does and does not pin).
            pub fn delta_of(&self, symbol: usize) -> Option<i32> {
                if symbol < self.symbol_count() {
                    Some(symbol as i32 - $center as i32)
                } else {
                    None
                }
            }

            /// The symbol carrying `delta` under the
            /// symmetric-alphabet convention (the inverse of
            /// [`Self::delta_of`]).
            pub fn symbol_of_delta(&self, delta: i32) -> Option<usize> {
                let symbol = delta.checked_add($center as i32)?;
                let symbol = usize::try_from(symbol).ok()?;
                (symbol < self.symbol_count()).then_some(symbol)
            }

            /// Append `symbol`'s codeword to `writer`.
            ///
            /// # Errors
            ///
            /// [`HuffmanError::SymbolOutOfRange`] outside the
            /// alphabet.
            pub fn encode_symbol(
                &self,
                symbol: usize,
                writer: &mut BitWriter,
            ) -> Result<(), HuffmanError> {
                self.code.encode_symbol(symbol, writer)
            }

            /// Read one codeword from `reader`, returning its symbol.
            ///
            /// # Errors
            ///
            /// [`HuffmanError::Bitstream`] if the stream ends inside
            /// a codeword.
            pub fn decode_symbol(&self, reader: &mut BitReader<'_>) -> Result<usize, HuffmanError> {
                self.code.decode_symbol(reader)
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

envelope_vlc!(
    ScaleVlc,
    SCALE_VLC_LENGTHS,
    SCALE_DELTA_CENTER,
    "The 121-symbol scale-factor / spectral-envelope **exponent \
     delta** VLC (sub-stream B5 of the staged per-block layout)."
);

envelope_vlc!(
    GainVlc,
    GAIN_VLC_LENGTHS,
    GAIN_DELTA_CENTER,
    "The 37-symbol per-block **gain delta** VLC (sub-stream B2 of the \
     staged per-block layout)."
);

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(s: &str) -> (u64, u8) {
        (
            u64::from_str_radix(s, 2).unwrap(),
            u8::try_from(s.len()).unwrap(),
        )
    }

    #[test]
    fn scale_vlc_reproduces_the_staged_csv_codewords() {
        let vlc = ScaleVlc::new();
        assert_eq!(vlc.symbol_count(), 121);
        assert_eq!(vlc.max_len(), 19);
        // Spot pins from wma-huffman-scale-codelen.csv.
        for (symbol, code) in [
            (0usize, "111111111111100010"),
            (59, "100"),
            (60, "0"),
            (61, "1011"),
            (120, "1111111111111111111"),
        ] {
            let (value, len) = bits(code);
            assert_eq!(vlc.code().length_of(symbol), Some(len), "symbol {symbol}");
            assert_eq!(vlc.code().code_of(symbol), Some(value), "symbol {symbol}");
        }
    }

    #[test]
    fn gain_vlc_reproduces_the_staged_csv_codewords() {
        let vlc = GainVlc::new();
        assert_eq!(vlc.symbol_count(), 37);
        assert_eq!(vlc.max_len(), 13);
        // Spot pins from wma-huffman-gain-codelen.csv.
        for (symbol, code) in [
            (0usize, "1111111010"),
            (13, "000"),
            (18, "1011"),
            (36, "1111111111111"),
        ] {
            let (value, len) = bits(code);
            assert_eq!(vlc.code().length_of(symbol), Some(len), "symbol {symbol}");
            assert_eq!(vlc.code().code_of(symbol), Some(value), "symbol {symbol}");
        }
    }

    #[test]
    fn scale_zero_delta_is_the_one_bit_codeword() {
        let vlc = ScaleVlc::new();
        assert_eq!(vlc.delta_of(SCALE_DELTA_CENTER), Some(0));
        assert_eq!(vlc.code().length_of(SCALE_DELTA_CENTER), Some(1));
        assert_eq!(vlc.delta_of(0), Some(-60));
        assert_eq!(vlc.delta_of(120), Some(60));
        assert_eq!(vlc.delta_of(121), None);
    }

    #[test]
    fn delta_round_trips_across_both_alphabets() {
        let scale = ScaleVlc::new();
        for delta in -60..=60 {
            let s = scale.symbol_of_delta(delta).unwrap();
            assert_eq!(scale.delta_of(s), Some(delta));
        }
        assert_eq!(scale.symbol_of_delta(61), None);
        assert_eq!(scale.symbol_of_delta(-61), None);

        let gain = GainVlc::new();
        for delta in -18..=18 {
            let s = gain.symbol_of_delta(delta).unwrap();
            assert_eq!(gain.delta_of(s), Some(delta));
        }
        assert_eq!(gain.symbol_of_delta(19), None);
        assert_eq!(gain.symbol_of_delta(-19), None);
    }

    #[test]
    fn symbol_streams_round_trip_through_bits() {
        let scale = ScaleVlc::new();
        let gain = GainVlc::new();
        let mut w = BitWriter::new();
        let scale_syms: Vec<usize> = (0..121).chain([60, 60, 0, 120]).collect();
        let gain_syms: Vec<usize> = (0..37).chain([18, 0, 36]).collect();
        for &s in &scale_syms {
            scale.encode_symbol(s, &mut w).unwrap();
        }
        for &s in &gain_syms {
            gain.encode_symbol(s, &mut w).unwrap();
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        for &s in &scale_syms {
            assert_eq!(scale.decode_symbol(&mut r).unwrap(), s);
        }
        for &s in &gain_syms {
            assert_eq!(gain.decode_symbol(&mut r).unwrap(), s);
        }
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn default_matches_new() {
        assert_eq!(ScaleVlc::default(), ScaleVlc::new());
        assert_eq!(GainVlc::default(), GainVlc::new());
    }
}

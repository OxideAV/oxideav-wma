//! Bit-level run-level `(R, L)` pair coder built from a probability
//! grid — the §6 entropy back end assembled end-to-end.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 discloses every
//! piece this module wires together:
//!
//! > **Joint (R,L) Huffman coding.** Run and level are combined into a
//! > 2-D array `(R, L)` and Huffman-coded together.
//! >   — [PATENT US7,885,819] — [PATENT US6,223,162]
//!
//! > **Code-book construction.** A 2-D probability grid over `(R,L)`
//! > pairings is built; pairings above a **probability threshold** get
//! > Huffman codewords, pairings below it are excluded to bound table
//! > size.
//! >   — [PATENT US6,223,162 — grid 500, threshold 518, FIG.6]
//!
//! > **Escape coding.** A pairing that falls below the threshold (not
//! > in the code book) is emitted with an **escape/special symbol**
//! > followed by enough literal information to identify the zero-run
//! > length and the non-zero sample value.
//! >   — [PATENT US6,223,162 — escape symbol; Claim 4; Claims 5–6]
//!
//! ## Scope of this module
//!
//! [`RunLevelCoder`] is the assembler that runs the patent's FIG.6
//! construction end-to-end: from a [`crate::codebook::CodebookGrid`]
//! it derives the codeword alphabet (every in-codebook pairing plus
//! one escape symbol), builds the joint Huffman code from the grid's
//! own probabilities ([`crate::huffman::HuffmanCode::from_weights`];
//! the escape symbol's weight is the rectangle's residual probability
//! mass), and codes pairs to/from bits over [`crate::bitio`]:
//! in-codebook pairs as single codewords, escape pairs as the escape
//! codeword followed by fixed-width `R` / `L` literals.
//!
//! ## What is NOT in this module
//!
//! * **The real WMA tables and probabilities.** The grid (its
//!   dimensions, threshold, and probabilities) is caller-supplied; the
//!   shipping tables are `[GAP]`, so a coder built here is
//!   self-consistent, not wire-compatible (see [`crate::huffman`]).
//! * **The escape literal bit widths.** §6 marks them `[GAP]` — "the
//!   bit widths are not patent-disclosed." They are a caller-supplied
//!   [`EscapeWidths`], never fabricated; values that do not fit reject
//!   at encode time.
//! * **Sign bits, the mode flag, the block walk.** Sign placement is
//!   `[GAP]` (§6); the level/run-level partition lives in
//!   [`crate::spectral`]; this coder maps one pair to/from bits.

use crate::bitio::{BitReader, BitWriter};
use crate::codebook::{CodebookGrid, Disposition};
use crate::huffman::{HuffmanCode, HuffmanError};
use crate::runlevel::RunLevelPair;

/// Caller-supplied escape-literal field widths, in bits — the §6
/// `[GAP]` the patents leave open ("enough literal information to
/// identify the zero-run length and the non-zero sample value",
/// widths undisclosed).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EscapeWidths {
    /// Bits for the literal run field.
    pub run_bits: u8,
    /// Bits for the literal level field.
    pub level_bits: u8,
}

impl EscapeWidths {
    /// Validate the widths: each field needs at least 1 bit and at
    /// most 32 (the [`RunLevelPair`] field width).
    pub fn new(run_bits: u8, level_bits: u8) -> Result<Self, PairCodeError> {
        if run_bits == 0 || run_bits > 32 {
            return Err(PairCodeError::InvalidEscapeWidth { bits: run_bits });
        }
        if level_bits == 0 || level_bits > 32 {
            return Err(PairCodeError::InvalidEscapeWidth { bits: level_bits });
        }
        Ok(Self {
            run_bits,
            level_bits,
        })
    }

    /// Largest run value the literal run field can carry.
    pub fn max_run(&self) -> u32 {
        if self.run_bits >= 32 {
            u32::MAX
        } else {
            (1u32 << self.run_bits) - 1
        }
    }

    /// Largest level magnitude the literal level field can carry.
    pub fn max_level(&self) -> u32 {
        if self.level_bits >= 32 {
            u32::MAX
        } else {
            (1u32 << self.level_bits) - 1
        }
    }
}

/// Bit-level joint `(R, L)` coder over a probability grid, per §6 of
/// the patent trace (US6,223,162 / US7,885,819): one canonical Huffman
/// codeword per in-codebook pairing plus an escape symbol whose
/// codeword is followed by fixed-width `R` / `L` literals.
#[derive(Debug, Clone)]
pub struct RunLevelCoder {
    grid: CodebookGrid,
    /// Joint code over `alphabet = in-codebook pairs ++ [escape]`.
    code: HuffmanCode,
    /// Symbol index → pair, for the in-codebook symbols
    /// (`0..escape_symbol()`), in the grid's row-major order.
    pairs: Vec<RunLevelPair>,
    /// `(run - 1) * ln + (level - 1)` → symbol index, for in-rectangle
    /// pairs (`None` for escape cells).
    symbol_of: Vec<Option<usize>>,
    widths: EscapeWidths,
}

impl RunLevelCoder {
    /// Build the coder from a probability grid and the escape-literal
    /// widths.
    ///
    /// The alphabet is the grid's in-codebook pairings (in its
    /// row-major run-outer order) plus one trailing escape symbol.
    /// Weights are the grid's own probabilities; the escape symbol's
    /// weight is the residual mass `max(0, 1 - Σ in-codebook)` — the
    /// total probability of everything the threshold excluded, which
    /// is what the escape codeword stands for.
    ///
    /// # Errors
    ///
    /// [`PairCodeError::Huffman`] if code construction fails (never
    /// for a valid grid — the alphabet always has the escape symbol).
    pub fn from_grid(grid: &CodebookGrid, widths: EscapeWidths) -> Result<Self, PairCodeError> {
        let ln = grid.ln() as usize;
        let mut pairs = Vec::with_capacity(grid.in_codebook_count());
        let mut symbol_of = vec![None; grid.rm() as usize * ln];
        let mut weights = Vec::with_capacity(grid.in_codebook_count() + 1);
        let mut mass = 0.0_f64;
        for pair in grid.in_codebook_pairs() {
            let p = grid
                .probability_of(pair.run, pair.level.get())
                .expect("in-codebook pair is inside the rectangle");
            let cell = (pair.run as usize - 1) * ln + (pair.level.get() as usize - 1);
            symbol_of[cell] = Some(pairs.len());
            pairs.push(pair);
            weights.push(p);
            mass += p;
        }
        // Escape weight: the residual probability mass the threshold
        // excluded (clamped at zero against rounding).
        weights.push((1.0 - mass).max(0.0));

        let code = HuffmanCode::from_weights(&weights).map_err(PairCodeError::Huffman)?;
        Ok(Self {
            grid: grid.clone(),
            code,
            pairs,
            symbol_of,
            widths,
        })
    }

    /// The grid this coder was built from.
    #[inline]
    pub const fn grid(&self) -> &CodebookGrid {
        &self.grid
    }

    /// The escape-literal widths.
    #[inline]
    pub const fn widths(&self) -> EscapeWidths {
        self.widths
    }

    /// Alphabet size (in-codebook pairings + 1 escape symbol).
    pub fn alphabet_len(&self) -> usize {
        self.pairs.len() + 1
    }

    /// The escape symbol's index (always the last symbol).
    pub fn escape_symbol(&self) -> usize {
        self.pairs.len()
    }

    /// Append one pair to the writer: its joint codeword when the
    /// grid holds it, otherwise the escape codeword followed by the
    /// literal `R` / `L` fields (US6,223,162 Claim 4).
    ///
    /// # Errors
    ///
    /// * [`PairCodeError::EscapeOverflow`] if an escape pair's run or
    ///   level does not fit its literal field width.
    /// * [`PairCodeError::Huffman`] on a coding failure (internal
    ///   invariants make this unreachable; surfaced for completeness).
    pub fn encode_pair(
        &self,
        pair: RunLevelPair,
        writer: &mut BitWriter,
    ) -> Result<(), PairCodeError> {
        match self.grid.disposition(pair) {
            Disposition::InCodebook => {
                let cell = (pair.run as usize - 1) * self.grid.ln() as usize
                    + (pair.level.get() as usize - 1);
                let symbol = self.symbol_of[cell].expect("in-codebook cell has a symbol");
                self.code
                    .encode_symbol(symbol, writer)
                    .map_err(PairCodeError::Huffman)
            }
            Disposition::Escape => {
                if pair.run > self.widths.max_run() {
                    return Err(PairCodeError::EscapeOverflow {
                        value: pair.run,
                        bits: self.widths.run_bits,
                    });
                }
                if pair.level.get() > self.widths.max_level() {
                    return Err(PairCodeError::EscapeOverflow {
                        value: pair.level.get(),
                        bits: self.widths.level_bits,
                    });
                }
                self.code
                    .encode_symbol(self.escape_symbol(), writer)
                    .map_err(PairCodeError::Huffman)?;
                writer.write_bits(u64::from(pair.run), self.widths.run_bits);
                writer.write_bits(u64::from(pair.level.get()), self.widths.level_bits);
                Ok(())
            }
        }
    }

    /// Read one pair from the reader — the Claim-5/6 decoder side:
    /// decode the joint codeword; if it is the escape symbol, recover
    /// `R` and `L` from the literal trailer.
    ///
    /// # Errors
    ///
    /// * [`PairCodeError::Huffman`] on a codeword/stream failure.
    /// * [`PairCodeError::InvalidEscapeLiteral`] if the literal
    ///   trailer carries `run == 0` or `level == 0` — outside the
    ///   patent's `{1..}` sets, so a corrupt stream.
    pub fn decode_pair(&self, reader: &mut BitReader<'_>) -> Result<RunLevelPair, PairCodeError> {
        let symbol = self
            .code
            .decode_symbol(reader)
            .map_err(PairCodeError::Huffman)?;
        if symbol < self.pairs.len() {
            return Ok(self.pairs[symbol]);
        }
        // Escape: read the literal trailer.
        let run = self
            .code_read_bits(reader, self.widths.run_bits)
            .map_err(PairCodeError::Huffman)?;
        let level = self
            .code_read_bits(reader, self.widths.level_bits)
            .map_err(PairCodeError::Huffman)?;
        RunLevelPair::new(run, level)
            .map_err(|_| PairCodeError::InvalidEscapeLiteral { run, level })
    }

    fn code_read_bits(&self, reader: &mut BitReader<'_>, n: u8) -> Result<u32, HuffmanError> {
        reader
            .read_bits(n)
            .map(|v| v as u32)
            .map_err(HuffmanError::Bitstream)
    }
}

/// Failure modes for [`RunLevelCoder`] construction and coding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PairCodeError {
    /// An escape-literal field width was zero or above 32.
    InvalidEscapeWidth {
        /// The rejected width.
        bits: u8,
    },
    /// An escape pair's run or level does not fit its literal field.
    EscapeOverflow {
        /// The value that did not fit.
        value: u32,
        /// The field width it was offered.
        bits: u8,
    },
    /// The escape literal trailer decoded to `run == 0` or
    /// `level == 0` — outside the patent's sets; a corrupt stream.
    InvalidEscapeLiteral {
        /// Decoded literal run.
        run: u32,
        /// Decoded literal level.
        level: u32,
    },
    /// An underlying Huffman/bit-stream failure.
    Huffman(HuffmanError),
}

impl core::fmt::Display for PairCodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PairCodeError::InvalidEscapeWidth { bits } => write!(
                f,
                "oxideav-wma::paircode: escape-literal width {bits} bits is outside 1..=32",
            ),
            PairCodeError::EscapeOverflow { value, bits } => write!(
                f,
                "oxideav-wma::paircode: escape value {value} does not fit a {bits}-bit literal",
            ),
            PairCodeError::InvalidEscapeLiteral { run, level } => write!(
                f,
                "oxideav-wma::paircode: escape literal ({run}, {level}) is outside the patent's {{1..}} sets",
            ),
            PairCodeError::Huffman(e) => write!(f, "oxideav-wma::paircode: {e}"),
        }
    }
}

impl std::error::Error for PairCodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            PairCodeError::Huffman(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    /// A 3×3 grid whose diagonal-ish high-probability cells clear the
    /// threshold: (1,1)=.4, (1,2)=.2, (2,1)=.2 in; the rest below.
    fn grid_3x3() -> CodebookGrid {
        CodebookGrid::from_probabilities(
            3,
            3,
            0.1,
            vec![0.4, 0.2, 0.01, 0.2, 0.05, 0.01, 0.02, 0.01, 0.01],
        )
        .unwrap()
    }

    fn coder() -> RunLevelCoder {
        RunLevelCoder::from_grid(&grid_3x3(), EscapeWidths::new(8, 8).unwrap()).unwrap()
    }

    // ---------- EscapeWidths ----------

    #[test]
    fn escape_widths_validate_and_bound() {
        assert!(EscapeWidths::new(0, 8).is_err());
        assert!(EscapeWidths::new(8, 0).is_err());
        assert!(EscapeWidths::new(33, 8).is_err());
        let w = EscapeWidths::new(4, 32).unwrap();
        assert_eq!(w.max_run(), 15);
        assert_eq!(w.max_level(), u32::MAX);
    }

    // ---------- construction ----------

    #[test]
    fn from_grid_builds_alphabet_of_in_codebook_plus_escape() {
        let c = coder();
        // grid_3x3 has 3 in-codebook cells.
        assert_eq!(c.alphabet_len(), 4);
        assert_eq!(c.escape_symbol(), 3);
        assert_eq!(c.widths(), EscapeWidths::new(8, 8).unwrap());
        assert_eq!(c.grid().rm(), 3);
    }

    #[test]
    fn most_probable_pair_gets_shortest_code() {
        // (1,1) has probability 0.4 vs escape mass 0.4 vs 0.2/0.2 —
        // its code can be no longer than the rarer pairs'.
        let c = coder();
        let mut w_frequent = BitWriter::new();
        c.encode_pair(pair(1, 1), &mut w_frequent).unwrap();
        let mut w_rare = BitWriter::new();
        c.encode_pair(pair(2, 1), &mut w_rare).unwrap();
        assert!(w_frequent.bit_len() <= w_rare.bit_len());
    }

    // ---------- encode/decode: in-codebook ----------

    #[test]
    fn in_codebook_pair_round_trips_as_single_codeword() {
        let c = coder();
        for p in [pair(1, 1), pair(1, 2), pair(2, 1)] {
            let mut w = BitWriter::new();
            c.encode_pair(p, &mut w).unwrap();
            let bit_len = w.bit_len();
            let bytes = w.into_bytes();
            let mut r = BitReader::with_bit_len(&bytes, bit_len);
            assert_eq!(c.decode_pair(&mut r).unwrap(), p);
            assert_eq!(r.remaining_bits(), 0);
        }
    }

    // ---------- encode/decode: escape ----------

    #[test]
    fn below_threshold_pair_escapes_with_literals() {
        let c = coder();
        // (3, 3) has probability 0.01 < 0.1 → escape.
        let p = pair(3, 3);
        let mut w = BitWriter::new();
        c.encode_pair(p, &mut w).unwrap();
        // Escape codeword + 8-bit run + 8-bit level: strictly longer
        // than any single in-codebook codeword.
        assert!(w.bit_len() > 16);
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(c.decode_pair(&mut r).unwrap(), p);
    }

    #[test]
    fn outside_rectangle_pair_escapes_too() {
        // Beyond (rm, ln): the patent's "≥ Rm" tail — still an escape.
        let c = coder();
        let p = pair(100, 42);
        let mut w = BitWriter::new();
        c.encode_pair(p, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(c.decode_pair(&mut r).unwrap(), p);
    }

    #[test]
    fn escape_overflow_rejects_unfittable_values() {
        let c = RunLevelCoder::from_grid(&grid_3x3(), EscapeWidths::new(4, 4).unwrap()).unwrap();
        let mut w = BitWriter::new();
        // run 16 does not fit 4 bits (max 15).
        assert_eq!(
            c.encode_pair(pair(16, 1), &mut w),
            Err(PairCodeError::EscapeOverflow { value: 16, bits: 4 })
        );
        // level 16 does not fit either.
        assert_eq!(
            c.encode_pair(pair(4, 16), &mut w),
            Err(PairCodeError::EscapeOverflow { value: 16, bits: 4 })
        );
    }

    #[test]
    fn corrupt_escape_literal_is_rejected() {
        // Hand-craft a stream: escape codeword + zero run literal.
        let c = coder();
        let mut w = BitWriter::new();
        // Recover the escape codeword by encoding a real escape pair,
        // then rebuild the stream manually: the escape codeword bits
        // followed by a zero run literal and level 1.
        let mut probe = BitWriter::new();
        c.encode_pair(pair(3, 3), &mut probe).unwrap();
        let escape_len = probe.bit_len() - 16; // minus the two 8-bit literals
        let probe_bytes = probe.into_bytes();
        let mut pr = BitReader::new(&probe_bytes);
        let codeword = pr.read_bits(escape_len as u8).unwrap();
        w.write_bits(codeword, escape_len as u8);
        w.write_bits(0, 8); // run = 0: invalid
        w.write_bits(1, 8);
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(
            c.decode_pair(&mut r),
            Err(PairCodeError::InvalidEscapeLiteral { run: 0, level: 1 })
        );
    }

    #[test]
    fn truncated_stream_surfaces_bitstream_error() {
        let c = coder();
        let mut w = BitWriter::new();
        c.encode_pair(pair(3, 3), &mut w).unwrap();
        let bytes = w.into_bytes();
        // Hand the decoder only 2 bits.
        let mut r = BitReader::with_bit_len(&bytes, 2);
        assert!(matches!(
            c.decode_pair(&mut r),
            Err(PairCodeError::Huffman(HuffmanError::Bitstream(_)))
        ));
    }

    // ---------- stream round trip ----------

    #[test]
    fn mixed_pair_stream_round_trips() {
        let c = coder();
        // In-codebook and escape pairs interleaved.
        let stream = [
            pair(1, 1),
            pair(3, 3),
            pair(2, 1),
            pair(1, 2),
            pair(9, 7),
            pair(1, 1),
            pair(2, 2), // below threshold → escape
            pair(1, 1),
        ];
        let mut w = BitWriter::new();
        for &p in &stream {
            c.encode_pair(p, &mut w).unwrap();
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        for &p in &stream {
            assert_eq!(c.decode_pair(&mut r).unwrap(), p);
        }
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn spectral_tail_pairs_survive_the_bit_layer() {
        // End-to-end §6 chain: a sparse tail → compress → pair-code to
        // bits → decode pairs → expand — the first symbol path in the
        // crate that crosses the bit level.
        use crate::runlevel;

        let tail: Vec<u32> = vec![0, 2, 0, 0, 1, 0, 0, 0, 3, 0, 0, 0, 0];
        let compressed = runlevel::compress(&tail).unwrap();
        let feed = compressed.pairs_with_implicit_terminator();

        let c = coder();
        let mut w = BitWriter::new();
        for &p in &feed {
            c.encode_pair(p, &mut w).unwrap();
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();

        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let mut decoded = Vec::new();
        for _ in 0..feed.len() {
            decoded.push(c.decode_pair(&mut r).unwrap());
        }
        let mut out = vec![0u32; tail.len()];
        runlevel::expand_into(&decoded, tail.len() as u64, &mut out).unwrap();
        assert_eq!(out, tail);
    }

    // ---------- error plumbing ----------

    #[test]
    fn error_display_and_source() {
        let e = PairCodeError::EscapeOverflow { value: 16, bits: 4 };
        assert!(format!("{e}").contains("16"));
        assert!(std::error::Error::source(&e).is_none());
        let e = PairCodeError::Huffman(HuffmanError::InvalidCodeword);
        assert!(std::error::Error::source(&e).is_some());
    }
}

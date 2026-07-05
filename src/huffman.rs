//! Huffman code construction and bit-level coding for the entropy
//! stage.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 discloses the
//! code-book construction *method* the WMA Standard entropy stage is
//! built by:
//!
//! > **Code-book construction.** A 2-D probability grid over `(R,L)`
//! > pairings is built; pairings above a **probability threshold**
//! > get Huffman codewords, pairings below it are excluded to bound
//! > table size. … Implementations: Huffman tree or Rice-Golomb.
//! >   — [PATENT US6,223,162 — grid 500, threshold 518, FIG.6;
//! >      Claims 8–10]
//!
//! > **Joint (R,L) Huffman coding.** Run and level are combined into
//! > a 2-D array `(R, L)` and Huffman-coded together.
//! >   — [PATENT US7,885,819]
//!
//! and §4 the matrix-side use:
//!
//! > 3. **Huffman-code** the differentially-coded elements.
//! >   — [PATENT US7,930,171 — step 130] — [PATENT US7,502,743]
//!
//! The Huffman algorithm itself (weight-ordered merging producing an
//! optimal prefix code) and the canonical-code arrangement are
//! general public CS/DSP material — the trace's **[DSP]** framing
//! tier. This module implements that machinery so the §6/§4 stages
//! have a working coder to run on.
//!
//! **What stays `[GAP]`:** the literal WMA v1/v2 codeword tables (the
//! wiki names their sizes — coefficient/level/scale/gain tables — but
//! not their contents). A [`HuffmanCode`] built here from
//! caller-supplied weights is **self-consistent, not
//! wire-compatible**; when staged tables land, they plug in as
//! explicit code lengths via [`HuffmanCode::from_lengths`].
//!
//! ## Scope
//!
//! * [`HuffmanCode::from_weights`] — build an optimal prefix code
//!   from non-negative symbol weights (the patent's grid-probability
//!   input), realised canonically.
//! * [`HuffmanCode::from_lengths`] — build the canonical code from
//!   explicit per-symbol code lengths (the plug-in point for real
//!   tables; validates the Kraft equality).
//! * [`HuffmanCode::encode_symbol`] / [`HuffmanCode::decode_symbol`]
//!   — bit-level coding over [`crate::bitio`].

use crate::bitio::{BitReader, BitWriter, BitstreamEnd};

/// A canonical prefix (Huffman) code over the symbol alphabet
/// `0..len()`.
///
/// Canonical arrangement: codes are assigned in order of
/// `(code length, symbol index)`, numerically increasing — the
/// conventional normal form that makes a code reconstructible from
/// its length vector alone (general public CS material; `[DSP]`
/// tier).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HuffmanCode {
    /// Per-symbol code length in bits (`lengths[s]`), `1..=64`.
    lengths: Vec<u8>,
    /// Per-symbol canonical codeword (`codes[s]`, low `lengths[s]`
    /// bits significant).
    codes: Vec<u64>,
    /// Largest code length.
    max_len: u8,
    /// First canonical code at each length (index `1..=max_len`).
    first: Vec<u64>,
    /// Symbol count at each length (same indexing).
    count: Vec<u64>,
    /// Index into `sorted` of the first symbol at each length.
    offset: Vec<usize>,
    /// Symbols in canonical `(length, symbol)` order.
    sorted: Vec<usize>,
}

impl HuffmanCode {
    /// Build an optimal prefix code from per-symbol weights — the
    /// patent's construction step run on a caller-supplied
    /// probability set (grid probabilities for the §6 run-level
    /// coder, delta statistics for the §4 matrix coder).
    ///
    /// Weights must be finite and non-negative; zero weights are
    /// legal (the patent's threshold can sit at `0.0`) and simply
    /// receive the longest codes. A single-symbol alphabet gets the
    /// 1-bit code `0` (a prefix code needs at least one bit on the
    /// wire to be self-delimiting).
    ///
    /// # Errors
    ///
    /// * [`HuffmanError::EmptyAlphabet`] for zero symbols.
    /// * [`HuffmanError::InvalidWeight`] for a negative or non-finite
    ///   weight.
    pub fn from_weights(weights: &[f64]) -> Result<Self, HuffmanError> {
        if weights.is_empty() {
            return Err(HuffmanError::EmptyAlphabet);
        }
        for (symbol, &w) in weights.iter().enumerate() {
            if !w.is_finite() || w < 0.0 {
                return Err(HuffmanError::InvalidWeight { symbol, weight: w });
            }
        }
        if weights.len() == 1 {
            return Self::from_lengths(&[1]);
        }

        // Standard Huffman merge over a binary heap of (weight, node).
        // Ties broken deterministically by node index so the build is
        // reproducible across runs.
        #[derive(PartialEq)]
        struct Entry {
            weight: f64,
            node: usize,
        }
        impl Eq for Entry {}
        impl Ord for Entry {
            fn cmp(&self, other: &Self) -> core::cmp::Ordering {
                // Reverse for a min-heap; weights are finite by
                // validation above so partial_cmp cannot fail.
                other
                    .weight
                    .partial_cmp(&self.weight)
                    .expect("weights validated finite")
                    .then_with(|| other.node.cmp(&self.node))
            }
        }
        impl PartialOrd for Entry {
            fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        let n = weights.len();
        // parent[i] for every tree node; leaves are 0..n, internal
        // nodes appended after.
        let mut parent: Vec<usize> = (0..n).collect();
        let mut heap = std::collections::BinaryHeap::new();
        for (i, &w) in weights.iter().enumerate() {
            heap.push(Entry { weight: w, node: i });
        }
        while heap.len() > 1 {
            let a = heap.pop().expect("len > 1");
            let b = heap.pop().expect("len > 1");
            let merged = parent.len();
            parent.push(merged); // self-parent until adopted
            parent[a.node] = merged;
            parent[b.node] = merged;
            heap.push(Entry {
                weight: a.weight + b.weight,
                node: merged,
            });
        }

        // Leaf depth = code length.
        let mut lengths = vec![0u8; n];
        for (s, len) in lengths.iter_mut().enumerate() {
            let mut depth = 0u8;
            let mut node = s;
            while parent[node] != node {
                node = parent[node];
                depth += 1;
            }
            *len = depth;
        }

        Self::from_lengths(&lengths)
    }

    /// Build the canonical code from explicit per-symbol code lengths
    /// — the plug-in point for a staged real table (a canonical code
    /// is fully determined by its length vector).
    ///
    /// # Errors
    ///
    /// * [`HuffmanError::EmptyAlphabet`] for zero symbols.
    /// * [`HuffmanError::InvalidLength`] for a zero length or one
    ///   above 64.
    /// * [`HuffmanError::KraftViolation`] unless the lengths satisfy
    ///   the Kraft **equality** `Σ 2^-len == 1` (a complete prefix
    ///   code; the single-symbol `[1]` case is accepted as the
    ///   conventional degenerate code).
    pub fn from_lengths(lengths: &[u8]) -> Result<Self, HuffmanError> {
        Self::build_from_lengths(lengths, true)
    }

    /// Build the canonical code from explicit per-symbol code lengths,
    /// accepting an **incomplete** prefix code (Kraft *inequality*
    /// `Σ 2^-len <= 1`).
    ///
    /// This is the plug-in point for a staged real table whose length
    /// vector is documented-incomplete — the WMA mode-2 coefficient
    /// VLC, whose vendor decode DAG replicates a few high symbols
    /// across several code lengths so their canonical single length is
    /// not statically determinable. Decoding a codeword that falls in
    /// the unassigned code space returns
    /// [`HuffmanError::InvalidCodeword`] instead of a symbol.
    ///
    /// # Errors
    ///
    /// As [`HuffmanCode::from_lengths`], except that
    /// [`HuffmanError::KraftViolation`] is raised only when the
    /// lengths *oversubscribe* the code space (`Σ 2^-len > 1` — not a
    /// prefix code at all).
    pub fn from_lengths_prefix(lengths: &[u8]) -> Result<Self, HuffmanError> {
        Self::build_from_lengths(lengths, false)
    }

    fn build_from_lengths(lengths: &[u8], require_complete: bool) -> Result<Self, HuffmanError> {
        if lengths.is_empty() {
            return Err(HuffmanError::EmptyAlphabet);
        }
        for (symbol, &len) in lengths.iter().enumerate() {
            if len == 0 || len > 64 {
                return Err(HuffmanError::InvalidLength { symbol, len });
            }
        }
        let max_len = *lengths.iter().max().expect("non-empty");

        // Kraft sum in units of 2^-max_len, exactly (u128 headroom:
        // max 64-bit lengths over practical alphabet sizes).
        let kraft: u128 = lengths.iter().map(|&len| 1u128 << (max_len - len)).sum();
        let full = 1u128 << max_len;
        let degenerate_single = lengths.len() == 1 && lengths[0] == 1;
        let acceptable = if require_complete {
            kraft == full || degenerate_single
        } else {
            kraft <= full
        };
        if !acceptable {
            return Err(HuffmanError::KraftViolation);
        }

        // Canonical assignment: count per length, first code per
        // length, then per-symbol codes in (length, symbol) order.
        let mut count = vec![0u64; max_len as usize + 1];
        for &len in lengths {
            count[len as usize] += 1;
        }
        let mut first = vec![0u64; max_len as usize + 1];
        let mut code = 0u64;
        for len in 1..=max_len as usize {
            code = (code + count[len - 1]) << 1;
            first[len] = code;
        }
        let mut next = first.clone();
        let mut codes = vec![0u64; lengths.len()];
        for (s, &len) in lengths.iter().enumerate() {
            codes[s] = next[len as usize];
            next[len as usize] += 1;
        }

        // Decode tables: symbols in (length, symbol) order plus the
        // start offset of each length's run.
        let mut offset = vec![0usize; max_len as usize + 1];
        let mut acc = 0usize;
        for len in 1..=max_len as usize {
            offset[len] = acc;
            acc += count[len] as usize;
        }
        let mut fill = offset.clone();
        let mut sorted = vec![0usize; lengths.len()];
        for (s, &len) in lengths.iter().enumerate() {
            sorted[fill[len as usize]] = s;
            fill[len as usize] += 1;
        }

        Ok(Self {
            lengths: lengths.to_vec(),
            codes,
            max_len,
            first,
            count,
            offset,
            sorted,
        })
    }

    /// Alphabet size.
    pub fn len(&self) -> usize {
        self.lengths.len()
    }

    /// Whether the alphabet is empty (never true for a constructed
    /// code).
    pub fn is_empty(&self) -> bool {
        self.lengths.is_empty()
    }

    /// Code length in bits for `symbol`, or `None` out of range.
    pub fn length_of(&self, symbol: usize) -> Option<u8> {
        self.lengths.get(symbol).copied()
    }

    /// Canonical codeword for `symbol` (low `length_of` bits), or
    /// `None` out of range.
    pub fn code_of(&self, symbol: usize) -> Option<u64> {
        self.codes.get(symbol).copied()
    }

    /// Largest code length in the code.
    pub fn max_len(&self) -> u8 {
        self.max_len
    }

    /// Append `symbol`'s codeword to the writer.
    ///
    /// # Errors
    ///
    /// [`HuffmanError::SymbolOutOfRange`] if `symbol >= len()`.
    pub fn encode_symbol(&self, symbol: usize, writer: &mut BitWriter) -> Result<(), HuffmanError> {
        let len = self
            .lengths
            .get(symbol)
            .copied()
            .ok_or(HuffmanError::SymbolOutOfRange {
                symbol,
                alphabet: self.lengths.len(),
            })?;
        writer.write_bits(self.codes[symbol], len);
        Ok(())
    }

    /// Read one codeword from the reader, returning its symbol.
    ///
    /// Canonical decode: extend the accumulated code one bit at a
    /// time and, at each length, check it against that length's
    /// canonical code range — `O(max_len)` per symbol with no decode
    /// table.
    ///
    /// # Errors
    ///
    /// * [`HuffmanError::Bitstream`] if the stream ends inside a
    ///   codeword.
    /// * [`HuffmanError::InvalidCodeword`] if `max_len` bits match no
    ///   symbol (unreachable for a complete code, kept for the
    ///   degenerate single-symbol case and defense in depth).
    pub fn decode_symbol(&self, reader: &mut BitReader<'_>) -> Result<usize, HuffmanError> {
        let mut code = 0u64;
        for len in 1..=self.max_len as usize {
            code = (code << 1) | u64::from(reader.read_bit().map_err(HuffmanError::Bitstream)?);
            // Canonical codes at one length are the contiguous range
            // [first[len], first[len] + count[len]); the symbol is
            // located by its offset into that range.
            if self.count[len] > 0 && code >= self.first[len] {
                let rank = code - self.first[len];
                if rank < self.count[len] {
                    return Ok(self.sorted[self.offset[len] + rank as usize]);
                }
            }
        }
        Err(HuffmanError::InvalidCodeword)
    }
}

/// Failure modes for [`HuffmanCode`] construction and coding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HuffmanError {
    /// No symbols were offered.
    EmptyAlphabet,
    /// A weight was negative or non-finite.
    InvalidWeight {
        /// Offending symbol index.
        symbol: usize,
        /// Its rejected weight.
        weight: f64,
    },
    /// A code length was zero or above 64.
    InvalidLength {
        /// Offending symbol index.
        symbol: usize,
        /// Its rejected length.
        len: u8,
    },
    /// The length vector does not satisfy the Kraft equality (is not
    /// a complete prefix code).
    KraftViolation,
    /// [`HuffmanCode::encode_symbol`] was offered a symbol outside
    /// the alphabet.
    SymbolOutOfRange {
        /// The rejected symbol.
        symbol: usize,
        /// The alphabet size.
        alphabet: usize,
    },
    /// The bit stream ended inside a codeword.
    Bitstream(BitstreamEnd),
    /// The accumulated bits match no codeword.
    InvalidCodeword,
}

impl core::fmt::Display for HuffmanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            HuffmanError::EmptyAlphabet => {
                f.write_str("oxideav-wma::huffman: empty symbol alphabet")
            }
            HuffmanError::InvalidWeight { symbol, weight } => write!(
                f,
                "oxideav-wma::huffman: symbol {symbol} has invalid weight {weight}",
            ),
            HuffmanError::InvalidLength { symbol, len } => write!(
                f,
                "oxideav-wma::huffman: symbol {symbol} has invalid code length {len}",
            ),
            HuffmanError::KraftViolation => {
                f.write_str("oxideav-wma::huffman: code lengths do not form a complete prefix code")
            }
            HuffmanError::SymbolOutOfRange { symbol, alphabet } => write!(
                f,
                "oxideav-wma::huffman: symbol {symbol} outside alphabet of {alphabet}",
            ),
            HuffmanError::Bitstream(e) => write!(f, "oxideav-wma::huffman: {e}"),
            HuffmanError::InvalidCodeword => {
                f.write_str("oxideav-wma::huffman: bits match no codeword")
            }
        }
    }
}

impl std::error::Error for HuffmanError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            HuffmanError::Bitstream(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(code: &HuffmanCode, symbols: &[usize]) {
        let mut w = BitWriter::new();
        for &s in symbols {
            code.encode_symbol(s, &mut w).unwrap();
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        for &s in symbols {
            assert_eq!(code.decode_symbol(&mut r).unwrap(), s);
        }
        assert_eq!(r.remaining_bits(), 0);
    }

    // ---------- from_weights construction ----------

    #[test]
    fn from_weights_rejects_empty_and_invalid() {
        assert_eq!(
            HuffmanCode::from_weights(&[]),
            Err(HuffmanError::EmptyAlphabet)
        );
        assert!(matches!(
            HuffmanCode::from_weights(&[1.0, -0.5]),
            Err(HuffmanError::InvalidWeight { symbol: 1, .. })
        ));
        assert!(matches!(
            HuffmanCode::from_weights(&[f64::NAN]),
            Err(HuffmanError::InvalidWeight { symbol: 0, .. })
        ));
        assert!(matches!(
            HuffmanCode::from_weights(&[f64::INFINITY, 1.0]),
            Err(HuffmanError::InvalidWeight { symbol: 0, .. })
        ));
    }

    #[test]
    fn single_symbol_gets_one_bit_code() {
        let code = HuffmanCode::from_weights(&[5.0]).unwrap();
        assert_eq!(code.len(), 1);
        assert_eq!(code.length_of(0), Some(1));
        assert_eq!(code.code_of(0), Some(0));
        round_trip(&code, &[0, 0, 0]);
    }

    #[test]
    fn two_equal_symbols_get_one_bit_each() {
        let code = HuffmanCode::from_weights(&[1.0, 1.0]).unwrap();
        assert_eq!(code.length_of(0), Some(1));
        assert_eq!(code.length_of(1), Some(1));
        // Canonical: symbol 0 gets 0, symbol 1 gets 1.
        assert_eq!(code.code_of(0), Some(0));
        assert_eq!(code.code_of(1), Some(1));
    }

    #[test]
    fn more_probable_symbols_get_no_longer_codes() {
        // The defining optimality shape: monotone weights produce
        // monotone (non-increasing) code lengths.
        let weights = [0.5, 0.25, 0.15, 0.07, 0.03];
        let code = HuffmanCode::from_weights(&weights).unwrap();
        for s in 1..weights.len() {
            assert!(
                code.length_of(s).unwrap() >= code.length_of(s - 1).unwrap(),
                "symbol {s}"
            );
        }
    }

    #[test]
    fn dyadic_weights_recover_exact_lengths() {
        // Weights 1/2, 1/4, 1/8, 1/8 → lengths 1, 2, 3, 3 exactly.
        let code = HuffmanCode::from_weights(&[0.5, 0.25, 0.125, 0.125]).unwrap();
        assert_eq!(code.length_of(0), Some(1));
        assert_eq!(code.length_of(1), Some(2));
        assert_eq!(code.length_of(2), Some(3));
        assert_eq!(code.length_of(3), Some(3));
        assert_eq!(code.max_len(), 3);
    }

    #[test]
    fn zero_weights_are_legal_and_get_longest_codes() {
        let code = HuffmanCode::from_weights(&[0.0, 1.0, 0.0, 4.0]).unwrap();
        let l0 = code.length_of(0).unwrap();
        let l3 = code.length_of(3).unwrap();
        assert!(l0 >= l3);
        round_trip(&code, &[0, 1, 2, 3, 2, 1, 0]);
    }

    #[test]
    fn built_code_is_prefix_free() {
        let weights = [5.0, 3.0, 3.0, 2.0, 1.0, 1.0, 0.5, 0.25];
        let code = HuffmanCode::from_weights(&weights).unwrap();
        for a in 0..weights.len() {
            for b in 0..weights.len() {
                if a == b {
                    continue;
                }
                let (la, lb) = (code.length_of(a).unwrap(), code.length_of(b).unwrap());
                if la <= lb {
                    let prefix = code.code_of(b).unwrap() >> (lb - la);
                    assert_ne!(
                        prefix,
                        code.code_of(a).unwrap(),
                        "code {a} is a prefix of code {b}"
                    );
                }
            }
        }
    }

    #[test]
    fn built_code_satisfies_kraft_equality() {
        let weights = [9.0, 7.0, 5.0, 3.0, 2.0, 1.0];
        let code = HuffmanCode::from_weights(&weights).unwrap();
        let kraft: f64 = (0..weights.len())
            .map(|s| 2.0_f64.powi(-i32::from(code.length_of(s).unwrap())))
            .sum();
        assert!((kraft - 1.0).abs() < 1e-12, "kraft={kraft}");
    }

    // ---------- from_lengths construction ----------

    #[test]
    fn from_lengths_accepts_complete_codes() {
        let code = HuffmanCode::from_lengths(&[1, 2, 3, 3]).unwrap();
        // Canonical codes: 0, 10, 110, 111.
        assert_eq!(code.code_of(0), Some(0b0));
        assert_eq!(code.code_of(1), Some(0b10));
        assert_eq!(code.code_of(2), Some(0b110));
        assert_eq!(code.code_of(3), Some(0b111));
    }

    #[test]
    fn from_lengths_rejects_incomplete_and_overfull() {
        // Incomplete: lengths 2, 2 leave half the code space unused.
        assert_eq!(
            HuffmanCode::from_lengths(&[2, 2]),
            Err(HuffmanError::KraftViolation)
        );
        // Overfull: 1, 1, 1 oversubscribes.
        assert_eq!(
            HuffmanCode::from_lengths(&[1, 1, 1]),
            Err(HuffmanError::KraftViolation)
        );
        // Zero length is invalid outright.
        assert_eq!(
            HuffmanCode::from_lengths(&[0, 1]),
            Err(HuffmanError::InvalidLength { symbol: 0, len: 0 })
        );
        assert_eq!(
            HuffmanCode::from_lengths(&[]),
            Err(HuffmanError::EmptyAlphabet)
        );
    }

    #[test]
    fn canonical_order_is_length_then_symbol() {
        // Same lengths in a different symbol order: codes assigned in
        // (length, symbol) order.
        let code = HuffmanCode::from_lengths(&[3, 1, 3, 2]).unwrap();
        assert_eq!(code.code_of(1), Some(0b0)); // len 1
        assert_eq!(code.code_of(3), Some(0b10)); // len 2
        assert_eq!(code.code_of(0), Some(0b110)); // len 3, lower symbol
        assert_eq!(code.code_of(2), Some(0b111)); // len 3, higher symbol
    }

    // ---------- coding ----------

    #[test]
    fn encode_rejects_out_of_range_symbol() {
        let code = HuffmanCode::from_lengths(&[1, 1]).unwrap();
        let mut w = BitWriter::new();
        assert_eq!(
            code.encode_symbol(2, &mut w),
            Err(HuffmanError::SymbolOutOfRange {
                symbol: 2,
                alphabet: 2,
            })
        );
    }

    #[test]
    fn decode_fails_cleanly_on_truncated_stream() {
        let code = HuffmanCode::from_lengths(&[1, 2, 2]).unwrap();
        // Write symbol 2 (2 bits) but hand the decoder only 1 bit.
        let mut w = BitWriter::new();
        code.encode_symbol(2, &mut w).unwrap();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, 1);
        assert!(matches!(
            code.decode_symbol(&mut r),
            Err(HuffmanError::Bitstream(_))
        ));
    }

    #[test]
    fn round_trip_weighted_alphabet() {
        let weights = [40.0, 30.0, 15.0, 8.0, 4.0, 2.0, 1.0];
        let code = HuffmanCode::from_weights(&weights).unwrap();
        // A deterministic pseudo-random symbol stream.
        let mut state = 0xDEC0DE_u64;
        let symbols: Vec<usize> = (0..500)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                (state >> 33) as usize % weights.len()
            })
            .collect();
        round_trip(&code, &symbols);
    }

    #[test]
    fn weighted_stream_codes_shorter_than_fixed_width() {
        // The point of the patent's Huffman step: a skewed source
        // codes in fewer bits than the fixed-width alternative.
        let weights = [900.0, 50.0, 25.0, 15.0, 6.0, 2.0, 1.0, 1.0];
        let code = HuffmanCode::from_weights(&weights).unwrap();
        // A stream matching the weight profile: mostly symbol 0.
        let mut symbols = vec![0usize; 900];
        symbols.extend(std::iter::repeat_n(1usize, 50));
        symbols.extend(std::iter::repeat_n(2usize, 25));
        let mut w = BitWriter::new();
        for &s in &symbols {
            code.encode_symbol(s, &mut w).unwrap();
        }
        let huffman_bits = w.bit_len();
        let fixed_bits = symbols.len() * 3; // 8 symbols → 3 bits each
        assert!(
            huffman_bits < fixed_bits / 2,
            "huffman {huffman_bits} vs fixed {fixed_bits}"
        );
    }

    #[test]
    fn error_display_and_source() {
        let e = HuffmanError::Bitstream(BitstreamEnd {
            requested: 1,
            remaining: 0,
        });
        assert!(format!("{e}").contains("exhausted"));
        assert!(std::error::Error::source(&e).is_some());
        assert!(std::error::Error::source(&HuffmanError::KraftViolation).is_none());
        assert!(format!("{}", HuffmanError::KraftViolation).contains("complete prefix code"));
    }
}

//! Quantization-matrix side-information coding — the §4 FIG.1
//! direct-compression chain assembled down to bits.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §4 ("Quantization
//! matrix carriage in the bitstream") is the most directly
//! bitstream-relevant disclosure in the trace:
//!
//! > "Because the decoder needs the quantization matrices used to
//! > compress the audio data, the encoder transmits them **as side
//! > information in the bitstream** of compressed output."
//! >   — [PATENT US7,930,171 — WMA7]
//! >
//! > The matrix is compressed for transmission by a
//! > **direct-compression technique** (FIG.1 of Chen-171):
//! >
//! > 1. **Uniformly quantize** each matrix element. — [step 110]
//! > 2. **Differentially code** the quantized elements relative to
//! >    preceding elements in the matrix. — [step 120]
//! >    — [PATENT US7,502,743]
//! > 3. **Huffman-code** the differentially-coded elements. — [step
//! >    130] — [PATENT US7,502,743]
//!
//! plus the efficiency detail:
//!
//! > An efficiency detail: unneeded weighting factors may be set equal
//! > to the next needed one so the differential coder emits a zero
//! > delta. — [PATENT US7,502,743]
//!
//! ## Scope of this module
//!
//! [`MatrixCoder`] assembles the three FIG.1 steps end-to-end over the
//! existing primitives: step 110 as a uniform scalar quantize/round
//! against a caller-supplied step (the §4 quantizer arrangement with
//! unit weight — see [`crate::quant`]), step 120 via
//! [`crate::qmatrix::differential_encode`] /
//! [`crate::qmatrix::differential_decode`], and step 130 via a
//! [`crate::huffman::HuffmanCode`] over a caller-supplied bounded
//! delta alphabet, emitted through [`crate::bitio`]. The decoder side
//! reverses the three steps, reconstructing each matrix element to
//! within half a step — the uniform quantizer's §4 bound.
//!
//! ## What is NOT in this module
//!
//! * **The real delta table.** The wiki names a "scale Huffman table
//!   (121 entries)" but not its contents — `[GAP]`. The delta
//!   alphabet's range and weights are caller-supplied, so a coder
//!   built here is self-consistent, not wire-compatible.
//! * **The matrix quantization step choice and seed convention.**
//!   Encoder tuning / bitstream specifics not pinned by the staged
//!   material; both are explicit parameters.

use crate::bitio::{BitReader, BitWriter};
use crate::huffman::{HuffmanCode, HuffmanError};
use crate::qmatrix;
use crate::step_size::OverallStepSize;

/// Bit-level coder for the §4 FIG.1 matrix side-information chain:
/// uniform quantize → differential → Huffman (US7,930,171 steps
/// 110/120/130).
///
/// The Huffman alphabet covers the contiguous delta range
/// `min_delta ..= min_delta + weights.len() - 1`; a delta outside it
/// rejects at encode time (the real table's range is `[GAP]`, so no
/// escape convention is fabricated).
#[derive(Debug, Clone)]
pub struct MatrixCoder {
    code: HuffmanCode,
    min_delta: i32,
}

impl MatrixCoder {
    /// Build the delta coder from the alphabet's lowest delta and the
    /// per-delta weights (`weights[i]` weighting delta
    /// `min_delta + i`).
    ///
    /// # Errors
    ///
    /// * [`MatrixCodeError::EmptyAlphabet`] for an empty weight
    ///   slice.
    /// * [`MatrixCodeError::AlphabetOverflow`] if the range would
    ///   overflow `i32`.
    /// * [`MatrixCodeError::Huffman`] if code construction rejects
    ///   the weights.
    pub fn new(min_delta: i32, weights: &[f64]) -> Result<Self, MatrixCodeError> {
        if weights.is_empty() {
            return Err(MatrixCodeError::EmptyAlphabet);
        }
        let span = i64::from(min_delta) + weights.len() as i64 - 1;
        if span > i64::from(i32::MAX) {
            return Err(MatrixCodeError::AlphabetOverflow {
                min_delta,
                len: weights.len(),
            });
        }
        let code = HuffmanCode::from_weights(weights).map_err(MatrixCodeError::Huffman)?;
        Ok(Self { code, min_delta })
    }

    /// Lowest delta in the alphabet.
    #[inline]
    pub const fn min_delta(&self) -> i32 {
        self.min_delta
    }

    /// Highest delta in the alphabet.
    pub fn max_delta(&self) -> i32 {
        self.min_delta + (self.code.len() as i32 - 1)
    }

    /// Alphabet size (contiguous delta count).
    pub fn alphabet_len(&self) -> usize {
        self.code.len()
    }

    fn symbol_of(&self, delta: i32) -> Result<usize, MatrixCodeError> {
        let offset = i64::from(delta) - i64::from(self.min_delta);
        if offset < 0 || offset >= self.code.len() as i64 {
            return Err(MatrixCodeError::DeltaOutOfRange {
                delta,
                min: self.min_delta,
                max: self.max_delta(),
            });
        }
        Ok(offset as usize)
    }

    /// Steps 120 + 130 forward: differentially code the quantized
    /// elements against `seed` (US7,930,171 step 120's "relative to
    /// preceding elements", with the first element coded against the
    /// caller's seed) and Huffman-code each delta into the writer.
    pub fn encode_quantized(
        &self,
        seed: i32,
        q: &[i32],
        writer: &mut BitWriter,
    ) -> Result<(), MatrixCodeError> {
        let deltas = qmatrix::differential_encode(seed, q);
        for delta in deltas {
            let symbol = self.symbol_of(delta)?;
            self.code
                .encode_symbol(symbol, writer)
                .map_err(MatrixCodeError::Huffman)?;
        }
        Ok(())
    }

    /// Steps 130 + 120 inverse: Huffman-decode `count` deltas and
    /// differentially decode them against `seed`, recovering the
    /// quantized elements exactly.
    pub fn decode_quantized(
        &self,
        seed: i32,
        count: usize,
        reader: &mut BitReader<'_>,
    ) -> Result<Vec<i32>, MatrixCodeError> {
        let mut deltas = Vec::with_capacity(count);
        for _ in 0..count {
            let symbol = self
                .code
                .decode_symbol(reader)
                .map_err(MatrixCodeError::Huffman)?;
            deltas.push(self.min_delta + symbol as i32);
        }
        qmatrix::differential_decode_in_place(seed, &mut deltas);
        Ok(deltas)
    }

    /// The full FIG.1 chain forward: **step 110** uniformly quantize
    /// each matrix element against `step` (`q[i] =
    /// round(weight[i] / step)`, the §4 quantizer with unit band
    /// weight), then steps 120 + 130 via
    /// [`MatrixCoder::encode_quantized`].
    ///
    /// Returns the quantized elements so the caller can (as the §4
    /// text requires) run its own encoder with the *decoder's*
    /// reconstruction of the matrix.
    pub fn compress_matrix(
        &self,
        weights: &[f64],
        step: OverallStepSize,
        seed: i32,
        writer: &mut BitWriter,
    ) -> Result<Vec<i32>, MatrixCodeError> {
        let q: Vec<i32> = weights
            .iter()
            .map(|&w| crate::quant::quantize_sample(w, 1.0, step.value()))
            .collect();
        self.encode_quantized(seed, &q, writer)?;
        Ok(q)
    }

    /// The full FIG.1 chain inverse: steps 130 + 120 via
    /// [`MatrixCoder::decode_quantized`], then the step-110 inverse
    /// `weight_hat[i] = q[i] * step` — each element within half a
    /// step of the original (the §4 uniform-quantizer bound).
    pub fn decompress_matrix(
        &self,
        count: usize,
        step: OverallStepSize,
        seed: i32,
        reader: &mut BitReader<'_>,
    ) -> Result<Vec<f64>, MatrixCodeError> {
        let q = self.decode_quantized(seed, count, reader)?;
        Ok(q.into_iter()
            .map(|v| crate::invquant::dequantize_sample(v, 1.0, step.value()))
            .collect())
    }
}

/// Failure modes for [`MatrixCoder`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MatrixCodeError {
    /// No delta weights were offered.
    EmptyAlphabet,
    /// The delta range `min_delta ..= min_delta + len - 1` overflows
    /// `i32`.
    AlphabetOverflow {
        /// Lowest delta requested.
        min_delta: i32,
        /// Alphabet length requested.
        len: usize,
    },
    /// A differential delta falls outside the coder's alphabet.
    DeltaOutOfRange {
        /// The offending delta.
        delta: i32,
        /// Lowest representable delta.
        min: i32,
        /// Highest representable delta.
        max: i32,
    },
    /// An underlying Huffman/bit-stream failure.
    Huffman(HuffmanError),
}

impl core::fmt::Display for MatrixCodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MatrixCodeError::EmptyAlphabet => {
                f.write_str("oxideav-wma::matrix_coding: empty delta alphabet")
            }
            MatrixCodeError::AlphabetOverflow { min_delta, len } => write!(
                f,
                "oxideav-wma::matrix_coding: delta alphabet {min_delta}+{len} overflows i32",
            ),
            MatrixCodeError::DeltaOutOfRange { delta, min, max } => write!(
                f,
                "oxideav-wma::matrix_coding: delta {delta} outside the alphabet {min}..={max}",
            ),
            MatrixCodeError::Huffman(e) => write!(f, "oxideav-wma::matrix_coding: {e}"),
        }
    }
}

impl std::error::Error for MatrixCodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            MatrixCodeError::Huffman(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A delta coder over -8..=8 with a zero-centred weight profile
    /// (small deltas likely — the §4 differential coder's premise).
    fn coder() -> MatrixCoder {
        let weights: Vec<f64> = (-8..=8i32)
            .map(|d| 1.0 / (1.0 + f64::from(d.abs())))
            .collect();
        MatrixCoder::new(-8, &weights).unwrap()
    }

    fn step(v: f64) -> OverallStepSize {
        OverallStepSize::new(v).unwrap()
    }

    // ---------- construction ----------

    #[test]
    fn new_validates_alphabet() {
        assert!(matches!(
            MatrixCoder::new(0, &[]),
            Err(MatrixCodeError::EmptyAlphabet)
        ));
        assert!(matches!(
            MatrixCoder::new(i32::MAX - 1, &[1.0, 1.0, 1.0]),
            Err(MatrixCodeError::AlphabetOverflow { .. })
        ));
        let c = coder();
        assert_eq!(c.min_delta(), -8);
        assert_eq!(c.max_delta(), 8);
        assert_eq!(c.alphabet_len(), 17);
    }

    // ---------- steps 120 + 130 ----------

    #[test]
    fn quantized_elements_round_trip_exactly() {
        let c = coder();
        let q = [10, 12, 12, 11, 13, 13, 13, 9];
        let seed = 10;
        let mut w = BitWriter::new();
        c.encode_quantized(seed, &q, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(c.decode_quantized(seed, q.len(), &mut r).unwrap(), q);
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn out_of_alphabet_delta_rejects() {
        let c = coder();
        // First delta = 100 - 0 = 100, outside -8..=8.
        let mut w = BitWriter::new();
        assert_eq!(
            c.encode_quantized(0, &[100], &mut w),
            Err(MatrixCodeError::DeltaOutOfRange {
                delta: 100,
                min: -8,
                max: 8,
            })
        );
    }

    #[test]
    fn zero_delta_padding_codes_cheapest() {
        // The US7,502,743 efficiency detail: padding unneeded elements
        // to the next needed one produces zero deltas, and with a
        // zero-centred weight profile those take the fewest bits.
        let c = coder();
        let seed = 5;

        // Needed mask: elements 0 and 4; the rest padded to the next
        // needed value (7). The trailing unneeded element has no next
        // needed one and is left alone.
        let mut padded = [5, 0, 0, 0, 7, 7];
        let needed = [true, false, false, false, true, false];
        qmatrix::zero_delta_pad(&mut padded, &needed);

        let raw = [5, 0, 0, 0, 7, 7]; // unpadded: big swings
        let mut w_padded = BitWriter::new();
        c.encode_quantized(seed, &padded, &mut w_padded).unwrap();
        let mut w_raw = BitWriter::new();
        c.encode_quantized(seed, &raw, &mut w_raw).unwrap();
        assert!(
            w_padded.bit_len() < w_raw.bit_len(),
            "padded {} vs raw {}",
            w_padded.bit_len(),
            w_raw.bit_len()
        );
    }

    // ---------- the full FIG.1 chain ----------

    #[test]
    fn matrix_compress_decompress_within_half_a_step() {
        let c = coder();
        let s = step(0.5);
        let seed = 0;
        // A smooth weight curve so quantized deltas stay small.
        let weights: Vec<f64> = (0..16).map(|d| 1.0 + (d as f64) * 0.3).collect();

        let mut w = BitWriter::new();
        let q = c.compress_matrix(&weights, s, seed, &mut w).unwrap();
        assert_eq!(q.len(), weights.len());

        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let back = c.decompress_matrix(weights.len(), s, seed, &mut r).unwrap();

        assert_eq!(back.len(), weights.len());
        for (i, (&orig, &rec)) in weights.iter().zip(back.iter()).enumerate() {
            assert!((orig - rec).abs() <= 0.25 + 1e-12, "i={i}: {orig} vs {rec}");
        }
    }

    #[test]
    fn decompress_matches_quantized_grid_exactly() {
        // The decoder's reconstruction equals q * step exactly — the
        // encoder can therefore mirror the decoder's matrix, as the
        // §4 side-information contract requires.
        let c = coder();
        let s = step(0.25);
        let weights = [2.0, 2.25, 2.5, 2.5, 3.0];
        let mut w = BitWriter::new();
        let q = c.compress_matrix(&weights, s, 0, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let back = c.decompress_matrix(weights.len(), s, 0, &mut r).unwrap();
        for (i, (&qi, &b)) in q.iter().zip(back.iter()).enumerate() {
            assert_eq!(b, f64::from(qi) * 0.25, "i={i}");
        }
    }

    #[test]
    fn truncated_stream_surfaces_error() {
        let c = coder();
        let mut w = BitWriter::new();
        c.encode_quantized(0, &[1, 2, 3], &mut w).unwrap();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, 1);
        assert!(matches!(
            c.decode_quantized(0, 3, &mut r),
            Err(MatrixCodeError::Huffman(_))
        ));
    }

    #[test]
    fn error_display_and_source() {
        let e = MatrixCodeError::DeltaOutOfRange {
            delta: 100,
            min: -8,
            max: 8,
        };
        assert!(format!("{e}").contains("100"));
        assert!(std::error::Error::source(&e).is_none());
        let e = MatrixCodeError::Huffman(HuffmanError::InvalidCodeword);
        assert!(std::error::Error::source(&e).is_some());
    }
}

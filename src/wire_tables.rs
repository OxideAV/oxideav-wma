//! Wire-level static data tables for WMA Standard decode.
//!
//! ## Source
//!
//! Every value in this module is transcribed verbatim from the staged
//! numeric-table extraction under `docs/audio/wma/tables/` (see that
//! directory's `README.md`, the per-table `.meta` files, and
//! `docs/audio/wma/provenance/02-extractor-univdreams-tables.md`).
//! The tables were extracted **as bytes** from the vendor WMA Standard
//! decoder module's own PE data sections — numeric data tables are
//! facts, not authorship — and each carries a self-validating
//! plausibility check in the extraction pipeline. No third-party
//! codec source was involved anywhere in the chain.
//!
//! ## What is staged (and carried here)
//!
//! * [`CRITICAL_BAND_FREQS_HZ`] — 25 critical-band upper edges in Hz,
//!   the **exponent/quantization-band partition seed**. The vendor
//!   decoder scales each Hz edge into a coefficient-bin index per
//!   block (sample rate + block length), so the runtime partition is
//!   *derived*, not tabulated — exactly what the patent trace says
//!   about band boundaries. The 25 values are the textbook Bark-scale
//!   critical-band upper edges.
//! * [`SUBBAND_FREQS_HZ`] — 11 octave-spaced Hz edges, the secondary
//!   band-partition seed (consistent with the noise-substitution /
//!   high-band gain grid; role read from the table's consumer loop).
//! * [`DEQUANT_GAIN_LUT`] — the 113-step exponent/scale →
//!   linear-multiplier dequantization ladder. Log-spaced with ratio
//!   `10^(1/16)` (1.25 dB of amplitude per step); the integer head is
//!   rounding-dominated and the tail fits
//!   `round(0.57584 * 10^(n/16))` within 0.75 %.
//! * [`COEF_VLC_MODE1_LENGTHS`] / [`COEF_VLC_MODE3_LENGTHS`] — exact
//!   per-symbol code lengths of the coefficient run-level `(R, L)`
//!   VLCs for decode modes 1 (666 symbols) and 3 (476 symbols). Both
//!   satisfy the Kraft **equality** — complete prefix codes — so
//!   [`crate::huffman::HuffmanCode::from_lengths`] accepts them
//!   directly; the staged CSVs use the same canonical MSB-first
//!   `(length, symbol)` codeword convention that constructor realises.
//! * [`COEF_VLC_MODE2_REAL_LENGTHS`] — decode mode 2's 1016 symbols
//!   (`0..=1015`), exact lengths. The staged watch-pass **correction**
//!   (docs `provenance/02` §4e): the earlier "8 escape codewords at
//!   `1016..=1023`" reading was a misinterpretation — those symbols
//!   are ordinary `(R, L)` pairs. The real end-of-block / escape are
//!   the **low reserved symbols** [`COEF_EOB_SYMBOL`] (0) and
//!   [`COEF_ESCAPE_SYMBOL`] (1); the run-level companion tables are
//!   2-based (`index = symbol - `[`COEF_RUNLEVEL_BASE_SYMBOL`]). The
//!   mode-2 decode table is a space-shared DAG in which a few high
//!   symbols (including `1016..=1023`) appear at more than one code
//!   length, so a single canonical length for them is not statically
//!   determinable — the flat-scan Kraft sum over `0..=1015` leaves
//!   [`COEF_VLC_MODE2_KRAFT_DEFICIT`]` / 2^22` unassigned. **No
//!   codeword is missing**; the residual is the exact per-symbol code
//!   of those DAG-replicated high symbols only.
//! * [`COEF_VLC_CLASS1_ALT_LENGTHS`] / [`COEF_VLC_CLASS3_ALT_LENGTHS`]
//!   — the **alt-variant** coefficient VLCs the vendor module
//!   registers when its `params+0x380 == 1` configuration flag is set
//!   (555 / 435 symbols, both Kraft-complete). The class-2 alt table
//!   is located in the vendor module but not yet staged.
//! * [`SCALE_VLC_LENGTHS`] — the 121-symbol **scale-factor /
//!   spectral-envelope exponent delta** VLC (Kraft-complete; matches
//!   the wiki's "scale Huffman table (121 entries)").
//! * [`GAIN_VLC_LENGTHS`] — the 37-symbol **gain delta** VLC
//!   (Kraft-complete; matches the wiki's "gain Huffman table
//!   (37 entries)").
//! * [`CLASS_SELECTOR_THRESHOLDS`] — the four `f32` constants of the
//!   decode-class selection rule (the per-stream rate float's
//!   lower/upper bounds plus the class-1 and class-2 branch
//!   thresholds), compared against a per-stream float at stream-open
//!   time when the sample rate is at least 32 kHz. The rule itself is
//!   realised in [`crate::wire_chain`].
//! * The **symbol → `(R, L)` companion maps** for all three decode
//!   classes are staged too — carried in [`crate::runlevel_tables`].
//! * The frame/superframe **bit-packing layout** (field order + fixed
//!   widths) is staged in `docs/audio/wma/frame-bit-layout.md`.
//!
//! ## What the staged material does NOT pin (open `[GAP]`s)
//!
//! * The class-selector **branch directions**: which side of the
//!   class-1 / class-2 branch thresholds selects which class (the
//!   staged trace deliberately records only that the float "compares
//!   a certain way"), and the init **formula of the per-stream float**
//!   the thresholds are compared against (a "bitrate/quality" scalar;
//!   its derivation from the header fields is not staged).
//! * The **class-2 alt-variant** coefficient VLC (located at vendor
//!   RVA `0x20718`, 350+ symbols, not yet extracted).
//! * The exact per-symbol codes of the mode-2 **DAG-replicated high
//!   symbols** (blocked statically by decode-DAG space sharing and
//!   dynamically behind a COM `ProcessOutput` vtable call).
//! * The **concrete widths** of the runtime-width fields (frame side
//!   field, escape literals) — their *formulas/sources* are pinned,
//!   the values are per-stream config.
//! * The vendor's internal codeword **bit assignment** is stored as a
//!   decode tree, not as code/length arrays; the staged *lengths* are
//!   exact data, and the codewords used here are the canonical
//!   MSB-first reconstruction from those lengths (the same convention
//!   the staged CSVs emit). Whether the vendor tree's own assignment
//!   coincides bit-for-bit is not yet verified — a documented
//!   residual of the extraction.
//!
//! Also confirmed by the extraction pass: **no LSP codebook exists**
//! on this decode path (static scan and load-time probe both
//! negative); the spectral envelope is exponent/critical-band coded,
//! which is exactly the machinery the tables above seed.

/// Critical-band upper edges in Hz — the exponent/quantization-band
/// partition seed (25 entries, strictly increasing, `100..=24500` Hz,
/// the textbook Bark-scale critical-band upper edges).
///
/// Staged as `docs/audio/wma/tables/critical-band-freqs.csv`. The
/// runtime per-block band boundaries are derived by scaling each edge
/// to a coefficient-bin index using the sample rate and block length;
/// the boundary table itself is never stored.
pub const CRITICAL_BAND_FREQS_HZ: [u16; 25] = [
    100, 200, 300, 400, 510, 630, 770, 920, 1080, 1270, 1480, 1720, 2000, 2320, 2700, 3150, 3700,
    4400, 5300, 6400, 7700, 9500, 12000, 15500, 24500,
];

/// Secondary band-partition seed in Hz (11 entries: a leading `0`,
/// then octave doubling `50..=12800`, capped at `24100`).
///
/// Staged as `docs/audio/wma/tables/subband-freqs.csv`. Consistent
/// with the noise-substitution / high-band gain band grid the patent
/// trace describes; the precise consumer role was read from the
/// vendor module's band loop, not from spec text.
pub const SUBBAND_FREQS_HZ: [u16; 11] = [0, 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, 24100];

/// Exponent/scale → linear dequantization multiplier ladder
/// (113 entries, monotone non-decreasing).
///
/// Staged as `docs/audio/wma/tables/dequant-gain-lut.csv`. Log-spaced
/// with ratio `10^(1/16)`, i.e. **1.25 dB of amplitude per step**;
/// indexed by a decoded exponent/scale value to give a fixed-point
/// linear multiplier. The integer head (`n < 30`) is dominated by
/// rounding; from `n = 30` the values fit
/// `round(0.57584 * 10^(n/16))` within 0.75 % relative error.
pub const DEQUANT_GAIN_LUT: [u32; 113] = [
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 21, 24, 28, 32,
    37, 43, 50, 58, 66, 77, 89, 102, 118, 137, 158, 182, 210, 243, 280, 324, 374, 432, 499, 576,
    665, 768, 887, 1024, 1182, 1366, 1577, 1821, 2103, 2428, 2804, 3238, 3739, 4318, 4987, 5758,
    6650, 7679, 8867, 10240, 11825, 13655, 15769, 18210, 21028, 24283, 28041, 32382, 37394, 43182,
    49865, 57584, 66497, 76789, 88675, 102400, 118250, 136553, 157688, 182096, 210281, 242829,
    280414, 323817, 373938, 431817, 498655, 575838, 664967, 767892, 886747, 1024000, 1182497,
    1365526, 1576885, 1820958, 2102810, 2428287, 2804142, 3238172, 3739383, 4318172, 4986547,
    5758375,
];

/// Exact per-symbol code lengths of the coefficient run-level VLC for
/// decode mode 1 (666 symbols, max length 20, Kraft equality —
/// a complete prefix code).
///
/// Staged as `docs/audio/wma/tables/wma-huffman-coef-mode1-codelen.csv`
/// (the `length` column; the extracted datum). Feed to
/// [`crate::huffman::HuffmanCode::from_lengths`] to realise the
/// canonical code the staged CSV tabulates.
pub const COEF_VLC_MODE1_LENGTHS: [u8; 666] = [
    11, 6, 2, 3, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 11, 11, 11, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 11, 12, 12, 12,
    12, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 15, 15, 14, 14, 15, 15, 15, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15, 14, 15, 15, 15, 15, 16, 16, 16, 15, 16, 15, 15, 16, 16, 16, 16,
    15, 16, 16, 16, 15, 16, 16, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 16, 16, 15, 16,
    16, 16, 17, 17, 17, 16, 16, 17, 16, 16, 16, 16, 17, 16, 17, 17, 16, 16, 15, 15, 15, 16, 17, 16,
    17, 16, 16, 17, 17, 17, 17, 17, 17, 16, 17, 17, 17, 16, 17, 17, 16, 17, 17, 17, 16, 17, 17, 16,
    16, 17, 17, 17, 18, 17, 17, 17, 17, 17, 18, 18, 17, 17, 17, 19, 17, 19, 18, 17, 17, 18, 17, 17,
    18, 17, 17, 17, 18, 17, 17, 18, 17, 17, 17, 17, 17, 16, 17, 17, 17, 17, 18, 16, 17, 4, 6, 8, 9,
    9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13,
    13, 13, 13, 13, 13, 13, 13, 13, 14, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15,
    15, 15, 15, 16, 15, 15, 15, 15, 15, 15, 17, 17, 17, 16, 18, 16, 17, 17, 16, 16, 17, 17, 18, 17,
    16, 17, 17, 17, 16, 17, 17, 18, 17, 18, 17, 17, 17, 18, 17, 17, 5, 8, 10, 10, 11, 11, 12, 12,
    12, 13, 13, 14, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    16, 16, 15, 16, 16, 15, 15, 15, 15, 15, 16, 16, 15, 15, 16, 16, 17, 17, 18, 17, 16, 17, 18, 19,
    17, 16, 16, 17, 17, 17, 6, 9, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 15, 15, 15,
    15, 15, 15, 16, 16, 16, 16, 17, 18, 16, 16, 16, 18, 17, 16, 17, 18, 17, 17, 16, 17, 17, 16, 17,
    16, 17, 18, 18, 18, 17, 19, 19, 17, 20, 19, 18, 19, 20, 18, 16, 18, 17, 7, 10, 12, 13, 13, 14,
    14, 14, 15, 15, 16, 16, 16, 16, 16, 18, 16, 17, 17, 8, 11, 13, 14, 14, 15, 16, 16, 16, 16, 17,
    17, 17, 18, 18, 17, 17, 8, 12, 14, 15, 15, 15, 17, 17, 18, 17, 9, 12, 14, 15, 16, 16, 17, 9,
    13, 15, 16, 16, 17, 9, 13, 16, 16, 16, 10, 13, 16, 18, 17, 10, 14, 17, 10, 14, 17, 11, 14, 16,
    11, 14, 11, 15, 12, 16, 12, 16, 12, 16, 12, 16, 12, 17, 13, 13, 17, 13, 17, 13, 13, 14, 14, 14,
    14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 15, 16, 16, 16, 16, 16, 16, 17, 16, 16, 16, 16,
    17, 16, 17, 16, 17, 17, 17,
];

/// Exact per-symbol code lengths of the coefficient run-level VLC for
/// decode mode 2 — symbols `0..=1015` (max length 22).
///
/// Staged as `docs/audio/wma/tables/wma-huffman-coef-mode2-codelen.csv`
/// under the corrected §4e reading (see the module docs): the vendor
/// decode table is a space-shared DAG that replicates a few high
/// symbols across several code lengths, so this flat scan does not
/// satisfy the Kraft **equality** — build it with
/// [`crate::huffman::HuffmanCode::from_lengths_prefix`], which
/// accepts the documented-incomplete prefix code and turns the
/// unassigned code space into a clean decode error. No codeword is
/// missing; the exact codes of the DAG-replicated symbols are the
/// documented static residual.
pub const COEF_VLC_MODE2_REAL_LENGTHS: [u8; 1016] = [
    11, 9, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
    11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    15, 16, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17,
    17, 17, 17, 17, 17, 17, 17, 18, 17, 17, 17, 17, 17, 17, 17, 18, 18, 17, 17, 18, 17, 17, 18, 17,
    18, 18, 18, 18, 19, 18, 18, 18, 18, 18, 18, 20, 18, 18, 18, 19, 19, 18, 19, 18, 19, 19, 18, 19,
    19, 18, 19, 19, 19, 19, 18, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 19, 19, 20, 19, 20, 19, 19,
    20, 19, 19, 20, 20, 20, 20, 19, 20, 21, 19, 3, 5, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13,
    13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 15, 15, 15, 15, 16,
    16, 16, 16, 17, 16, 17, 17, 16, 17, 17, 17, 17, 17, 17, 16, 17, 17, 17, 17, 18, 17, 17, 18, 18,
    18, 18, 18, 19, 18, 18, 18, 18, 18, 18, 19, 19, 18, 18, 18, 18, 19, 18, 19, 19, 19, 20, 19, 18,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 19, 20, 19, 20, 19, 20, 19, 19, 21, 20, 20, 19,
    4, 7, 8, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16,
    16, 16, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 17, 17, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 19, 18, 18, 18, 19, 18, 19, 19, 19, 20, 20, 20, 19, 19, 19, 19, 19,
    19, 19, 21, 21, 20, 19, 5, 8, 10, 11, 12, 13, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16,
    16, 17, 17, 17, 17, 17, 17, 17, 17, 18, 17, 18, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 19,
    18, 19, 18, 18, 18, 18, 18, 19, 18, 17, 17, 18, 18, 19, 19, 19, 19, 18, 18, 18, 19, 6, 9, 11,
    12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 17, 18, 18, 17, 18, 18, 18, 18, 18, 18, 19, 19, 18, 18, 18, 19, 19, 19, 20, 19,
    19, 18, 19, 19, 20, 21, 21, 19, 19, 18, 6, 10, 12, 13, 14, 14, 14, 15, 15, 15, 16, 16, 17, 17,
    17, 17, 17, 17, 17, 18, 18, 19, 18, 18, 18, 19, 18, 18, 18, 19, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 19, 20, 20, 19, 19, 19, 19, 20, 20, 19, 20, 19, 19, 19, 20, 20, 20, 19, 19, 18, 19,
    7, 10, 12, 13, 14, 15, 15, 15, 16, 16, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 19, 18, 19, 19,
    19, 20, 19, 18, 19, 19, 18, 18, 19, 19, 19, 18, 19, 19, 20, 19, 18, 20, 21, 20, 20, 19, 19, 21,
    20, 21, 20, 20, 20, 19, 19, 20, 20, 21, 20, 19, 7, 11, 13, 14, 15, 15, 15, 16, 16, 17, 17, 17,
    17, 18, 18, 18, 18, 18, 19, 20, 19, 19, 20, 19, 19, 19, 19, 19, 19, 19, 19, 18, 18, 19, 20, 19,
    19, 19, 20, 19, 19, 19, 20, 19, 20, 20, 21, 20, 20, 20, 21, 22, 20, 19, 20, 20, 21, 20, 21, 20,
    19, 8, 11, 13, 14, 15, 16, 16, 16, 17, 17, 17, 18, 18, 18, 18, 18, 19, 18, 19, 19, 19, 19, 21,
    19, 19, 21, 19, 20, 20, 20, 19, 18, 18, 8, 12, 14, 15, 16, 16, 16, 16, 17, 17, 17, 19, 18, 18,
    19, 19, 20, 19, 18, 20, 19, 20, 20, 19, 19, 20, 20, 21, 21, 20, 19, 19, 19, 19, 19, 19, 20, 21,
    20, 19, 19, 8, 12, 14, 15, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 19, 19, 19, 20, 21, 20,
    21, 19, 21, 20, 20, 20, 20, 21, 20, 19, 20, 19, 20, 20, 20, 19, 22, 21, 21, 19, 9, 12, 14, 15,
    16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 19, 20, 19, 19, 19, 9, 13, 15, 16, 17, 17, 18, 18, 18,
    19, 18, 20, 19, 20, 20, 20, 19, 9, 13, 15, 16, 17, 17, 18, 18, 18, 20, 18, 19, 20, 20, 20, 20,
    19, 20, 19, 9, 13, 15, 16, 17, 18, 18, 18, 19, 19, 19, 19, 10, 14, 16, 17, 18, 18, 19, 19, 19,
    19, 19, 10, 14, 16, 17, 18, 18, 18, 19, 19, 10, 14, 16, 17, 18, 18, 18, 19, 19, 20, 19, 10, 14,
    16, 18, 18, 18, 19, 20, 19, 19, 10, 14, 17, 18, 18, 18, 10, 15, 17, 18, 19, 19, 21, 19, 11, 15,
    17, 18, 18, 19, 19, 11, 15, 17, 18, 19, 19, 11, 15, 17, 18, 11, 15, 18, 19, 19, 11, 15, 18, 19,
    19, 11, 16, 18, 19, 11, 15, 18, 19, 11, 16, 18, 12, 16, 18, 19, 12, 16, 19, 12, 16, 19, 19, 19,
    12, 16, 19, 12, 16, 19, 19, 12, 16, 18, 12, 16, 19, 12, 17, 19, 12, 17, 19, 12, 17, 19, 12, 17,
    19, 13, 17, 13, 17, 13, 17, 19, 19, 13, 17, 13, 17, 19, 13, 17, 13, 18, 19, 13, 17, 19, 13, 18,
    13, 17, 13,
];

/// Exact per-symbol code lengths of the coefficient run-level VLC for
/// decode mode 3 (476 symbols, max length 21, Kraft equality —
/// a complete prefix code).
///
/// Staged as `docs/audio/wma/tables/wma-huffman-coef-mode3-codelen.csv`
/// (the `length` column). Feed to
/// [`crate::huffman::HuffmanCode::from_lengths`].
pub const COEF_VLC_MODE3_LENGTHS: [u8; 476] = [
    12, 6, 2, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9,
    9, 10, 10, 10, 10, 10, 10, 10, 11, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 15, 15, 15, 15, 15, 16, 16, 15, 16, 16, 17, 16, 16, 16,
    17, 18, 18, 17, 17, 17, 17, 17, 17, 17, 17, 17, 4, 6, 7, 8, 8, 8, 9, 9, 10, 10, 10, 10, 10, 10,
    11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 14, 13, 14, 14, 14,
    13, 13, 14, 14, 16, 16, 15, 16, 16, 16, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 16, 16, 16,
    16, 17, 17, 17, 18, 16, 5, 8, 9, 10, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13,
    13, 13, 13, 13, 14, 14, 13, 14, 14, 13, 14, 14, 15, 14, 15, 15, 15, 16, 15, 16, 16, 15, 15, 15,
    18, 18, 18, 17, 18, 17, 17, 6, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14,
    14, 14, 15, 15, 15, 16, 15, 15, 15, 15, 15, 15, 16, 16, 15, 16, 16, 16, 16, 17, 18, 17, 16, 16,
    16, 7, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 14, 15, 15, 15, 16, 15, 15, 15, 15, 16, 16,
    16, 17, 16, 17, 16, 15, 16, 16, 16, 16, 18, 17, 17, 19, 19, 18, 16, 7, 11, 12, 13, 14, 14, 15,
    15, 16, 16, 15, 16, 16, 15, 16, 16, 16, 16, 16, 16, 16, 17, 16, 17, 17, 16, 17, 18, 16, 17, 17,
    17, 8, 11, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 16, 17, 17, 17, 17, 18,
    18, 18, 17, 17, 8, 12, 14, 14, 15, 15, 16, 17, 17, 16, 16, 17, 17, 20, 17, 9, 12, 14, 16, 16,
    16, 17, 21, 18, 17, 9, 13, 15, 16, 16, 10, 13, 16, 10, 14, 16, 11, 15, 16, 11, 15, 17, 11, 15,
    12, 15, 12, 16, 12, 16, 13, 16, 13, 13, 13, 14, 14, 13, 14, 14, 14, 15, 15, 14, 15, 15, 15, 15,
    15, 15, 15, 16, 17, 16, 16, 16, 16, 17, 16, 17, 16, 18, 17, 17, 17, 16, 17, 17, 16, 18, 17, 21,
    17, 18, 17, 18, 17, 18, 17, 17, 17, 17, 19,
];

/// The coefficient VLC's reserved **end-of-block** symbol.
///
/// The staged correction (docs `provenance/02` §4e): the run-level
/// companion tables are 2-based, so symbols 0 and 1 carry no `(R, L)`
/// pair — they are the low reserved sentinels. Symbol 0 is the EOB
/// half of that reserved pair.
pub const COEF_EOB_SYMBOL: u16 = 0;

/// The coefficient VLC's reserved **escape** symbol.
///
/// The vendor decode path treats symbol 1 as the escape: it then
/// reads a literal run and a literal level with the runtime-signalled
/// widths (the frame side-field width and `byte_offset_bits`
/// respectively — sources pinned, concrete values per-stream config).
pub const COEF_ESCAPE_SYMBOL: u16 = 1;

/// First coefficient VLC symbol that carries a real `(R, L)` pair:
/// the run-level companion tables are indexed `symbol - 2`.
pub const COEF_RUNLEVEL_BASE_SYMBOL: u16 = 2;

/// The code space the mode-2 lengths over symbols `0..=1015` leave
/// unassigned, in units of `2^-22` (the mode's max code length):
/// `Σ 2^(22-len)` over [`COEF_VLC_MODE2_REAL_LENGTHS`] is `2^22 -`
/// this value. That mass is where the vendor decode DAG replicates a
/// few high symbols across several code lengths (no codeword is
/// *missing* — the exact codes of those replicated symbols are the
/// documented static residual).
pub const COEF_VLC_MODE2_KRAFT_DEFICIT: u32 = 16_502;

/// Exact per-symbol code lengths of the **class-1 alt-variant**
/// coefficient run-level VLC (555 symbols, max length 19, Kraft
/// equality — a complete prefix code). The vendor module registers
/// this tree instead of [`COEF_VLC_MODE1_LENGTHS`] when its
/// `params+0x380 == 1` configuration flag is set.
///
/// Staged as
/// `docs/audio/wma/tables/wma-huffman-coef-class1-alt-codelen.csv`.
pub const COEF_VLC_CLASS1_ALT_LENGTHS: [u8; 555] = [
    9, 5, 2, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 9, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 9, 9, 9,
    9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
    11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 13, 12, 12, 12, 12, 12, 12, 12, 13, 12, 12, 12, 12, 12, 12, 12, 12, 13, 12, 12, 12, 13, 13,
    13, 13, 12, 12, 12, 12, 12, 12, 13, 12, 13, 13, 13, 13, 13, 13, 13, 14, 14, 13, 13, 13, 13, 13,
    13, 13, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 13, 14, 13, 13,
    13, 13, 13, 14, 13, 14, 14, 13, 14, 14, 13, 14, 13, 13, 14, 14, 13, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 15, 14, 14, 14, 14, 15, 15, 15, 14, 14, 13, 13, 12, 12, 13, 13, 13, 14, 14, 15, 14, 15,
    15, 14, 13, 14, 15, 15, 15, 14, 14, 14, 14, 15, 14, 14, 15, 15, 15, 14, 15, 14, 14, 14, 14, 14,
    15, 15, 16, 15, 15, 15, 14, 15, 15, 15, 15, 14, 14, 16, 14, 15, 14, 14, 15, 15, 15, 15, 16, 15,
    14, 15, 15, 15, 16, 15, 15, 14, 14, 14, 4, 7, 8, 8, 9, 9, 9, 9, 10, 10, 11, 11, 11, 11, 11, 11,
    12, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13,
    12, 12, 13, 13, 13, 13, 14, 14, 13, 14, 13, 13, 13, 14, 14, 15, 15, 14, 13, 13, 13, 14, 14, 15,
    15, 15, 16, 14, 15, 17, 17, 15, 15, 15, 15, 15, 14, 16, 14, 16, 16, 16, 16, 16, 16, 15, 15, 17,
    15, 16, 15, 6, 8, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 13, 14, 13, 14, 13, 14, 14, 14,
    14, 14, 15, 15, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 16, 15, 15, 16, 15, 15, 15, 14, 16, 15,
    15, 18, 17, 16, 17, 15, 14, 15, 16, 16, 19, 17, 19, 16, 17, 15, 7, 10, 11, 12, 12, 12, 12, 13,
    13, 13, 14, 15, 14, 15, 15, 16, 15, 14, 14, 15, 16, 15, 16, 16, 16, 16, 15, 15, 7, 11, 12, 13,
    13, 14, 14, 15, 15, 15, 8, 11, 13, 14, 14, 15, 9, 12, 14, 14, 15, 9, 13, 10, 13, 10, 14, 10,
    14, 11, 15, 11, 15, 11, 14, 12, 15, 12, 13, 13, 13, 13, 13, 13, 14, 13, 14, 14, 14, 14, 14, 14,
    15, 14, 15, 16, 15, 14, 15, 16, 15, 15,
];

/// Exact per-symbol code lengths of the **class-3 alt-variant**
/// coefficient run-level VLC (435 symbols, max length 18, Kraft
/// equality — a complete prefix code). The `params+0x380 == 1`
/// alternative to [`COEF_VLC_MODE3_LENGTHS`].
///
/// Staged as
/// `docs/audio/wma/tables/wma-huffman-coef-class3-alt-codelen.csv`.
pub const COEF_VLC_CLASS3_ALT_LENGTHS: [u8; 435] = [
    10, 4, 2, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 7, 8,
    8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 9, 10, 10, 10, 10, 10, 9, 10, 10,
    10, 10, 10, 10, 10, 10, 11, 11, 10, 10, 11, 11, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12,
    11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 12, 12,
    13, 13, 13, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 13, 13, 13, 13, 13, 14, 14,
    14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 13, 14, 14, 14, 14, 14, 14, 15, 14, 15, 14, 15, 15,
    15, 15, 15, 15, 16, 15, 15, 14, 15, 16, 15, 14, 14, 15, 14, 14, 15, 14, 15, 15, 15, 16, 15, 17,
    16, 15, 15, 15, 15, 16, 16, 16, 16, 17, 15, 16, 14, 16, 16, 17, 16, 16, 16, 16, 16, 15, 15, 15,
    16, 16, 16, 16, 17, 15, 15, 15, 15, 16, 15, 15, 4, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 10,
    10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13,
    13, 13, 13, 12, 13, 14, 14, 15, 15, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 15, 15, 15, 14,
    14, 15, 15, 15, 16, 16, 18, 17, 15, 15, 15, 6, 9, 10, 10, 11, 11, 12, 12, 12, 13, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 14, 14, 15,
    16, 15, 14, 14, 15, 7, 10, 11, 12, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15,
    15, 14, 15, 16, 15, 15, 16, 15, 15, 15, 16, 15, 16, 18, 17, 15, 15, 16, 16, 17, 15, 8, 11, 13,
    13, 14, 15, 14, 16, 15, 16, 15, 15, 15, 15, 15, 15, 17, 15, 9, 12, 14, 15, 10, 13, 14, 15, 10,
    13, 11, 14, 11, 14, 11, 15, 12, 15, 12, 12, 13, 15, 13, 14, 13, 14, 14, 14, 14, 14, 15, 15, 15,
    15, 14, 15, 15, 16, 16, 16, 15, 16, 16, 15, 16, 17, 16,
];

/// Exact per-symbol code lengths of the **scale-factor /
/// spectral-envelope exponent delta** VLC (121 symbols, max length
/// 19, Kraft equality — a complete prefix code; matches the wiki's
/// "scale Huffman table (121 entries)").
///
/// The unique 1-bit codeword sits on symbol 60 — the alphabet's
/// center, the most-probable zero-delta event of a 121-symbol
/// `-60..=+60` delta alphabet (a property of the staged data itself).
///
/// Staged as `docs/audio/wma/tables/wma-huffman-scale-codelen.csv`.
pub const SCALE_VLC_LENGTHS: [u8; 121] = [
    18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 18, 19, 18, 17, 17,
    16, 17, 16, 16, 16, 16, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 12, 12, 12, 11, 12, 11, 10, 10,
    10, 9, 9, 8, 8, 8, 7, 6, 6, 5, 4, 3, 1, 4, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11,
    11, 11, 12, 12, 13, 13, 13, 14, 14, 16, 15, 16, 15, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
];

/// Exact per-symbol code lengths of the **gain delta** VLC
/// (37 symbols, max length 13, Kraft equality — a complete prefix
/// code; matches the wiki's "gain Huffman table (37 entries)").
///
/// The shortest codewords cluster around symbols 13–18 — the small
/// magnitudes of a delta-shaped alphabet (a property of the staged
/// data itself).
///
/// Staged as `docs/audio/wma/tables/wma-huffman-gain-codelen.csv`.
pub const GAIN_VLC_LENGTHS: [u8; 37] = [
    10, 12, 10, 13, 9, 13, 9, 8, 7, 5, 5, 4, 4, 3, 3, 3, 4, 3, 4, 4, 5, 5, 6, 8, 7, 10, 8, 10, 9,
    8, 9, 9, 13, 10, 13, 13, 13,
];

/// Lower bound of the decode-class selector's per-stream rate float
/// (`f32`, staged role "lower bound").
///
/// Staged as `docs/audio/wma/tables/wma-class-selector-thresholds.csv`
/// row 1. The four class-selector constants are the `f32` values the
/// vendor module's stream-open path compares a per-stream
/// bitrate/quality float against when the sample rate is at least
/// 32 kHz, choosing the coefficient-VLC **decode class**
/// (see [`crate::wire_chain::select_decode_class`]).
pub const CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND: f32 = 0.125;

/// The decode-class selector's **class-1 branch threshold** (`f32`,
/// staged role "class-1 branch threshold"; shortest-round-trip
/// rendering `0.72000003`).
///
/// Staged as `docs/audio/wma/tables/wma-class-selector-thresholds.csv`
/// row 3. Which *side* of the threshold selects class 1 is not staged
/// (the branch direction of the comparison is a documented gap) — see
/// the module docs.
pub const CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD: f32 = 0.72;

/// The decode-class selector's **class-2 branch threshold** (`f32`,
/// staged role "class-2 branch threshold").
///
/// Staged as `docs/audio/wma/tables/wma-class-selector-thresholds.csv`
/// row 2. Which *side* of the threshold selects class 2 is not staged
/// — see the module docs.
pub const CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD: f32 = 1.16;

/// Upper bound of the decode-class selector's per-stream rate float
/// (`f32`, staged role "upper bound").
///
/// Staged as `docs/audio/wma/tables/wma-class-selector-thresholds.csv`
/// row 4.
pub const CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND: f32 = 1.6;

/// The four decode-class selector constants in their staged storage
/// order (ascending vendor-module address): lower bound, class-2
/// branch threshold, class-1 branch threshold, upper bound.
///
/// Staged as `docs/audio/wma/tables/wma-class-selector-thresholds.csv`
/// (4 × `f32` LE). Note the storage order interleaves the two branch
/// thresholds between the bounds — the *rule* order (thresholds
/// ascending along the float axis) is bounds-outer, `0.72 < 1.16`
/// inner.
pub const CLASS_SELECTOR_THRESHOLDS: [f32; 4] = [
    CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND,
    CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD,
    CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD,
    CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND,
];

#[cfg(test)]
mod tests {
    use super::*;

    /// Kraft sum in units of 2^-max_len.
    fn kraft_num(lengths: &[u8]) -> (u128, u128) {
        let max = *lengths.iter().max().unwrap();
        let sum = lengths.iter().map(|&l| 1u128 << (max - l)).sum::<u128>();
        (sum, 1u128 << max)
    }

    // ---------- band-partition seeds ----------

    #[test]
    fn critical_bands_are_strictly_increasing_bark_edges() {
        assert_eq!(CRITICAL_BAND_FREQS_HZ.len(), 25);
        for w in CRITICAL_BAND_FREQS_HZ.windows(2) {
            assert!(w[0] < w[1], "edges must strictly increase: {w:?}");
        }
        // Endpoint + interior pins straight from the staged CSV.
        assert_eq!(CRITICAL_BAND_FREQS_HZ[0], 100);
        assert_eq!(CRITICAL_BAND_FREQS_HZ[4], 510);
        assert_eq!(CRITICAL_BAND_FREQS_HZ[12], 2000);
        assert_eq!(CRITICAL_BAND_FREQS_HZ[22], 12_000);
        assert_eq!(CRITICAL_BAND_FREQS_HZ[24], 24_500);
    }

    #[test]
    fn subbands_are_an_octave_grid_with_zero_head_and_cap() {
        assert_eq!(SUBBAND_FREQS_HZ.len(), 11);
        assert_eq!(SUBBAND_FREQS_HZ[0], 0);
        // Octave doubling across the interior run 50..=12800.
        for i in 1..9 {
            assert_eq!(
                SUBBAND_FREQS_HZ[i + 1],
                SUBBAND_FREQS_HZ[i] * 2,
                "octave doubling at slot {i}"
            );
        }
        assert_eq!(SUBBAND_FREQS_HZ[1], 50);
        assert_eq!(SUBBAND_FREQS_HZ[10], 24_100);
    }

    // ---------- gain ladder ----------

    #[test]
    fn gain_lut_is_monotone_with_pinned_endpoints() {
        assert_eq!(DEQUANT_GAIN_LUT.len(), 113);
        for w in DEQUANT_GAIN_LUT.windows(2) {
            assert!(w[0] <= w[1], "ladder must be non-decreasing: {w:?}");
        }
        assert_eq!(DEQUANT_GAIN_LUT[0], 1);
        assert_eq!(DEQUANT_GAIN_LUT[109], 3_739_383);
        assert_eq!(DEQUANT_GAIN_LUT[112], 5_758_375);
    }

    #[test]
    fn gain_lut_tail_fits_the_staged_closed_form() {
        // The .meta closed form: value[n] ~= round(0.57584 * 10^(n/16)),
        // tail max relative error 0.75 % (integer head excluded — small
        // values are rounding-dominated).
        for (n, &v) in DEQUANT_GAIN_LUT.iter().enumerate().skip(30) {
            let ideal = 0.57584_f64 * 10.0_f64.powf(n as f64 / 16.0);
            let rel = ((v as f64) - ideal).abs() / ideal;
            assert!(
                rel <= 0.0075,
                "index {n}: value {v} vs {ideal:.2} ({rel:.4})"
            );
        }
    }

    #[test]
    fn gain_lut_tail_step_ratio_is_ten_to_the_sixteenth() {
        let ratio = 10.0_f64.powf(1.0 / 16.0); // 1.25 dB per step
        for n in 30..112 {
            let r = DEQUANT_GAIN_LUT[n + 1] as f64 / DEQUANT_GAIN_LUT[n] as f64;
            assert!((r - ratio).abs() < 0.02, "step {n}: ratio {r} vs {ratio}");
        }
    }

    // ---------- coefficient VLC length tables ----------

    #[test]
    fn mode1_lengths_form_a_complete_prefix_code() {
        assert_eq!(COEF_VLC_MODE1_LENGTHS.len(), 666);
        assert_eq!(*COEF_VLC_MODE1_LENGTHS.iter().max().unwrap(), 20);
        assert_eq!(*COEF_VLC_MODE1_LENGTHS.iter().min().unwrap(), 2);
        let (sum, full) = kraft_num(&COEF_VLC_MODE1_LENGTHS);
        assert_eq!(sum, full, "Kraft equality");
    }

    #[test]
    fn mode3_lengths_form_a_complete_prefix_code() {
        assert_eq!(COEF_VLC_MODE3_LENGTHS.len(), 476);
        assert_eq!(*COEF_VLC_MODE3_LENGTHS.iter().max().unwrap(), 21);
        assert_eq!(*COEF_VLC_MODE3_LENGTHS.iter().min().unwrap(), 2);
        let (sum, full) = kraft_num(&COEF_VLC_MODE3_LENGTHS);
        assert_eq!(sum, full, "Kraft equality");
    }

    #[test]
    fn mode2_real_lengths_leave_exactly_the_documented_dag_deficit() {
        assert_eq!(COEF_VLC_MODE2_REAL_LENGTHS.len(), 1016);
        assert_eq!(*COEF_VLC_MODE2_REAL_LENGTHS.iter().max().unwrap(), 22);
        assert_eq!(*COEF_VLC_MODE2_REAL_LENGTHS.iter().min().unwrap(), 2);
        let (sum, full) = kraft_num(&COEF_VLC_MODE2_REAL_LENGTHS);
        assert!(sum < full, "mode 2's flat scan is documented incomplete");
        assert_eq!(
            full - sum,
            u128::from(COEF_VLC_MODE2_KRAFT_DEFICIT),
            "unassigned code space must match the documented DAG-replication room"
        );
    }

    #[test]
    fn reserved_sentinel_symbols_precede_the_runlevel_base() {
        // The staged correction: symbols 0/1 are EOB/escape, the
        // run-level companion tables start at symbol 2.
        assert_eq!(COEF_EOB_SYMBOL, 0);
        assert_eq!(COEF_ESCAPE_SYMBOL, 1);
        assert_eq!(COEF_RUNLEVEL_BASE_SYMBOL, 2);
    }

    #[test]
    fn alt_variant_lengths_form_complete_prefix_codes() {
        assert_eq!(COEF_VLC_CLASS1_ALT_LENGTHS.len(), 555);
        assert_eq!(*COEF_VLC_CLASS1_ALT_LENGTHS.iter().max().unwrap(), 19);
        let (sum, full) = kraft_num(&COEF_VLC_CLASS1_ALT_LENGTHS);
        assert_eq!(sum, full, "class-1 alt Kraft equality");

        assert_eq!(COEF_VLC_CLASS3_ALT_LENGTHS.len(), 435);
        assert_eq!(*COEF_VLC_CLASS3_ALT_LENGTHS.iter().max().unwrap(), 18);
        let (sum, full) = kraft_num(&COEF_VLC_CLASS3_ALT_LENGTHS);
        assert_eq!(sum, full, "class-3 alt Kraft equality");

        // Both alt tables put the 2-bit shortest code on symbol 2 —
        // the first real (R, L) symbol — like the primaries.
        assert_eq!(COEF_VLC_CLASS1_ALT_LENGTHS[2], 2);
        assert_eq!(COEF_VLC_CLASS3_ALT_LENGTHS[2], 2);
    }

    #[test]
    fn scale_lengths_form_a_complete_prefix_code_centered_at_60() {
        assert_eq!(SCALE_VLC_LENGTHS.len(), 121);
        assert_eq!(*SCALE_VLC_LENGTHS.iter().max().unwrap(), 19);
        let (sum, full) = kraft_num(&SCALE_VLC_LENGTHS);
        assert_eq!(sum, full, "scale Kraft equality");
        // The unique 1-bit code sits on the center symbol (zero delta).
        assert_eq!(SCALE_VLC_LENGTHS[60], 1);
        assert_eq!(
            SCALE_VLC_LENGTHS.iter().filter(|&&l| l == 1).count(),
            1,
            "exactly one 1-bit codeword"
        );
    }

    #[test]
    fn gain_lengths_form_a_complete_prefix_code() {
        assert_eq!(GAIN_VLC_LENGTHS.len(), 37);
        assert_eq!(*GAIN_VLC_LENGTHS.iter().max().unwrap(), 13);
        assert_eq!(*GAIN_VLC_LENGTHS.iter().min().unwrap(), 3);
        let (sum, full) = kraft_num(&GAIN_VLC_LENGTHS);
        assert_eq!(sum, full, "gain Kraft equality");
    }

    // ---------- class-selector thresholds ----------

    #[test]
    fn class_selector_constants_are_bit_exact_f32() {
        // The staged CSV renders each `f32 LE` value in shortest
        // round-trip form (`0.125`, `1.16`, `0.72000003`, `1.6`);
        // shortest round-trip uniquely identifies the bit pattern, so
        // these pins are exact against the extracted bytes.
        assert_eq!(CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND.to_bits(), 0x3E00_0000);
        assert_eq!(
            CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD.to_bits(),
            0x3F94_7AE1
        );
        assert_eq!(
            CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD.to_bits(),
            0x3F38_51EC
        );
        assert_eq!(CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND.to_bits(), 0x3FCC_CCCD);
    }

    #[test]
    fn class_selector_storage_order_matches_the_staged_rows() {
        // CSV row order = ascending vendor storage address: lower
        // bound, class-2 threshold, class-1 threshold, upper bound.
        assert_eq!(CLASS_SELECTOR_THRESHOLDS.len(), 4);
        assert_eq!(
            CLASS_SELECTOR_THRESHOLDS[0].to_bits(),
            CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND.to_bits()
        );
        assert_eq!(
            CLASS_SELECTOR_THRESHOLDS[1].to_bits(),
            CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD.to_bits()
        );
        assert_eq!(
            CLASS_SELECTOR_THRESHOLDS[2].to_bits(),
            CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD.to_bits()
        );
        assert_eq!(
            CLASS_SELECTOR_THRESHOLDS[3].to_bits(),
            CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND.to_bits()
        );
    }

    #[test]
    fn class_selector_rule_axis_is_strictly_ordered() {
        // Along the rate-float axis the rule order is bounds-outer
        // with the two branch thresholds strictly inside:
        // 0.125 < 0.72 < 1.16 < 1.6 (a property of the staged data).
        let axis = [
            CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND,
            CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD,
            CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD,
            CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND,
        ];
        for w in axis.windows(2) {
            assert!(w[0] < w[1], "axis must strictly increase: {w:?}");
        }
    }

    #[test]
    fn new_length_table_spot_pins_from_the_staged_csvs() {
        // Direct row pins (symbol, length) from the staged CSVs.
        assert_eq!(GAIN_VLC_LENGTHS[0], 10);
        assert_eq!(GAIN_VLC_LENGTHS[1], 12);
        assert_eq!(GAIN_VLC_LENGTHS[18], 4);
        assert_eq!(GAIN_VLC_LENGTHS[36], 13);
        assert_eq!(SCALE_VLC_LENGTHS[0], 18);
        assert_eq!(SCALE_VLC_LENGTHS[61], 4);
        assert_eq!(SCALE_VLC_LENGTHS[120], 19);
        assert_eq!(COEF_VLC_CLASS1_ALT_LENGTHS[0], 9);
        assert_eq!(COEF_VLC_CLASS1_ALT_LENGTHS[554], 15);
        assert_eq!(COEF_VLC_CLASS3_ALT_LENGTHS[0], 10);
        assert_eq!(COEF_VLC_CLASS3_ALT_LENGTHS[434], 16);
    }

    #[test]
    fn length_table_spot_pins_from_the_staged_csvs() {
        // Direct row pins from the staged CSVs (symbol, length).
        assert_eq!(COEF_VLC_MODE1_LENGTHS[0], 11);
        assert_eq!(COEF_VLC_MODE1_LENGTHS[2], 2);
        assert_eq!(COEF_VLC_MODE1_LENGTHS[333], 12);
        assert_eq!(COEF_VLC_MODE1_LENGTHS[665], 17);
        assert_eq!(COEF_VLC_MODE2_REAL_LENGTHS[0], 11);
        assert_eq!(COEF_VLC_MODE2_REAL_LENGTHS[1], 9);
        assert_eq!(COEF_VLC_MODE2_REAL_LENGTHS[508], 17);
        assert_eq!(COEF_VLC_MODE2_REAL_LENGTHS[1015], 13);
        assert_eq!(COEF_VLC_MODE3_LENGTHS[0], 12);
        assert_eq!(COEF_VLC_MODE3_LENGTHS[238], 12);
        assert_eq!(COEF_VLC_MODE3_LENGTHS[475], 19);
    }
}

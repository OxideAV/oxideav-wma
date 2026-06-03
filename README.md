# oxideav-wma

Pure-Rust Windows Media Audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 1** landed the WAVEFORMATEX-extradata header parser and the
sample-rate → MDCT long-block decision tree, sourced from the
multimedia.cx wiki snapshot at `docs/audio/wma/wiki/Windows_Media_Audio.wiki`:

* WMA v1 (codec ID `0x160`) and v2 (codec ID `0x161`) extradata
  layouts inside `WAVEFORMATEX` (4 bytes for v1; 6 bytes for v2);
* the meaning of the low three bits of `flags2`
  (exponential VLCs / bit reservoir / variable block length);
* the per-frame MDCT long-block-size decision tree as a function of
  `(version, sample_rate)`, yielding `frame_length_bits ∈ {9, 10, 11}`
  and `frame_length = 1 << frame_length_bits`;
* one explicit cutoff in the v2 sample-rate normaliser
  (`sample_rate >= 44_100` snaps to `44_100`).

Round 1 ships 21 tests behind [`WmaHeader::parse`].

**Round 2** lifts the §2 patent-disclosed **block-size set** out of
the patents-only structural trace
(`docs/audio/wma/wma-bitstream-from-patents.md`, citing
US7,930,171 Chen-171 Background) into a typed
[`BlockSize`] primitive:

```text
{ S256, S512, S1024, S2048, S4096 }    // 8..=12 bits log2
```

The enum exposes [`BlockSize::ALL`] (ascending iteration), `samples()`
/ `log2_samples()` accessors, validating constructors
[`BlockSize::from_samples`] / [`BlockSize::from_log2`], and
`is_shortest()` / `is_longest()` predicates for transient-handling
code. A new [`Error::InvalidBlockSize`] variant carries the rejected
sample count when a non-set value is offered. Round 2 ships 14
additional tests; one cross-module test verifies that every
`WmaHeader::frame_length` Round 1 produces is itself a member of the
patent set, so future transform code can wrap a header-supplied frame
length without a redundant lookup.

**Round 3** (this round) lifts two more primitives from the same
patent trace:

* **§5 sum/difference (mid/side) stereo transform** ([`stereo`]) —
  the patent's `sum = (L+R)/2`, `diff = (L-R)/2` formulation
  (US7,930,171 / US7,502,743) as `f64` per-sample helpers
  `mid` / `side` / `forward` / `inverse`, plus in-place slice
  helpers `forward_in_place` / `inverse_in_place` for whole-block
  application. The transform is algebraically invertible and
  bit-exact for inputs that produce exactly-representable sums.
* **§6 run-level pairing primitive** ([`runlevel`]) — a typed
  `RunLevelPair { run: u32, level: NonZeroU32 }` matching
  US6,223,162 Claim 1 (joint `(R, L)` symbol) and Claim 2 (level
  non-zero). Constructor enforces `run ≥ 1` per the trace's
  `{1..Rm}` set. A `coefficient_count()` accessor reports the
  `run + 1` slots the pair fills, an `is_implicit_terminator_for`
  predicate detects the patent's `(N, 1)` end-of-block sentinel,
  and an `expand_into` walker decodes a pair sequence into a
  sparse coefficient block honouring both termination rules
  (implicit `(N, 1)` and explicit underrun) with `WalkError`
  surfacing both `Overflow` and `Underrun`.

Round 3 adds 33 unit tests across the two modules (13 stereo, 20
runlevel), taking the crate's test count from 36 to 69.

**Round 4** lifted two more primitives from the same patent trace:

* **§4 quantization-matrix differential coding step** ([`qmatrix`]) —
  the patent's step-120 ("differentially codes the quantized
  elements relative to preceding elements in the matrix" —
  US7,930,171 / US7,502,743) as four invertible `i32` helpers:
  `differential_encode` / `differential_decode` (fresh `Vec`) plus
  matching `_in_place` variants over a `&mut [i32]`. The transform
  is bijective under wrapping `i32` arithmetic so the round-trip is
  exact for any input. A `zero_delta_pad` companion implements the
  patent's "set unneeded element = next needed element" encoder
  policy against a `[bool]` needed-mask so that subsequent
  differential encoding emits a zero delta at every substituted
  position — the patent's stated efficiency outcome.
* **§6 entropy-mode selector + sub-range partition descriptor**
  ([`entropy_mode`]) — `EntropyMode { Level, RunLevel }` matching
  the patent's "level mode" and "run length/level mode" naming
  (US6,223,162 mode selector 400 / US7,383,180 entropy encoder 570).
  `EntropyMode::ALL` locks the low-frequency-first iteration order;
  `opposite()` is involutive. A `Partition { total_coeffs, split,
  adaptive }` descriptor exposes `mode_for(index) -> Option<EntropyMode>`,
  `level_range_len()` / `run_level_range_len()` accessors, plus
  `is_adaptive()` / `is_predetermined()` predicates for the
  patent-disclosed boundary signalling choice. `Partition::new`
  rejects out-of-block splits with `InvalidPartition::SplitOutOfBlock`.

Round 4 adds 31 unit tests across the two modules (15 qmatrix, 16
entropy_mode), taking the crate's test count from 69 to 100.

**Round 5** (this round) lifts two more decoder-side primitives from
the same patent trace:

* **§4 inverse-quantization step** ([`invquant`]) — the patent's
  decoder-side reverse of the per-coefficient quantizer:
  `coeff_hat[k] = q[k] * Q[d(k)] * step` (US7,930,171 overall
  step-size description; US7,383,180 inverse quantizer-weighter FIG.6;
  US6,240,380 re-weighting at decoder). Public `f64` helpers
  `dequantize_sample` (per-sample) and `dequantize_in_place`
  (whole-block over a band map) realise the multiplicative
  arrangement. A `BandScale { scale: Vec<f64> }` carrier precomputes
  the per-band product `Q[d] * step` once per block so the inner
  dequant loop multiplies once per coefficient instead of twice; its
  `apply` whole-block helper is f64-equivalent to the two-factor
  helper for inputs that hit exact-representable products. The
  module's dead-zone, linearity-in-q, and factor-commutativity
  invariants are exercised explicitly.
* **§7 per-band coding-policy carrier** ([`bands`]) — typed
  [`BandPolicy`] enum covering the three patent-disclosed
  per-band alternatives: `Coded` (literal entropy coding;
  US7,383,180 default), `NoiseSubstituted { energy: f64 }` (decoder
  module 240's noise generator; US7,383,180 / US7,343,291), and
  `Truncated` (high-band cutoff; US7,383,180 "completely eliminate
  the coefficients in certain (high) bands"). A `BandPlan { policies,
  cutoff }` descriptor exposes the per-band table plus lookups
  (`policy_of`, `coded_band_count`, `noise_band_count`,
  `truncated_band_count`). A validating `BandPlan::new_with_cutoff`
  constructor enforces the patent's stated cutoff shape (truncated
  bands form a contiguous tail) and reports the cutoff index;
  `BandPlan::new` accepts arbitrary tables when the shape is not
  required. A new `InvalidBandPlan::TruncatedNotContiguousTail`
  variant identifies the offending boundary.

Round 5 adds 36 unit tests across the two modules (18 invquant, 18
bands), taking the crate's test count from 100 to 136.

**Round 6** (this round) lifts the §6 patent-disclosed **run-level
codebook construction model** from the same patent trace into a new
[`codebook`] module:

* The patent's "2-D probability grid over `(R, L)` pairings is built;
  pairings above a probability threshold get Huffman codewords,
  pairings below it are excluded to bound table size" (US6,223,162
  grid 500 / threshold 518 / FIG.6 / Claims 8–10) becomes a typed
  [`CodebookGrid`] holding a row-major `(rm × ln)` probability table
  and the cutoff threshold. The constructor
  [`CodebookGrid::from_probabilities`] enforces `rm >= 1`, `ln >= 1`,
  the `[0.0, 1.0]` probability range for both the threshold and the
  per-pair entries, and the `probabilities.len() == rm * ln` invariant.
* The patent's escape branch ("A pairing that falls below the
  threshold (not in the code book) is emitted with an escape/special
  symbol" — US6,223,162 Claim 4 / Claims 5–6) becomes a typed
  [`Disposition`] enum with `InCodebook` / `Escape` variants;
  `disposition(pair)`, `is_in_codebook(pair)`, and `is_escape(pair)`
  report what a downstream entropy stage should do with a given
  [`runlevel::RunLevelPair`]. Pairings outside the `(rm, ln)`
  rectangle are reported as `Escape` (they are not represented in the
  codebook at all).
* Counting and iteration: `in_codebook_count()`,
  `escape_count_in_rectangle()`, and `in_codebook_pairs()` walk the
  above-threshold positions in row-major `(run outer, level inner)`
  order, materialising each as a [`runlevel::RunLevelPair`].

Round 6 adds 27 unit tests covering the constructor accept/reject
paths, row-major lookup semantics, the inclusive `>=` threshold rule,
outside-rectangle escape reporting, count partitioning, iteration
order, cross-module orthogonality with the patent's `(N, 1)` implicit
terminator, and consistent error-message naming. The crate's test
count rises from 136 to 163.

**Round 7** (this round) lifts §3 of the same patent trace — the
patent-disclosed **per-block transient-handling switch** — into a new
[`transient`] module:

* The trace doc explicitly states that the *existence* of a per-block
  transient-handling switch signalled as side information is
  patent-backed, but the v1/v2 choice between the two patent-disclosed
  mechanisms is `[GAP]`. The new [`TransientMechanism`] enum names
  both alternatives side-by-side:
  * `SubbandCombineFlag` — the one-bit per-block side-information
    flag that switches high-frequency subband combining on/off,
    computed *after* the MLT so no window/block-size change is needed
    (US6,240,380 FIG.12 boxes 1210–1250 / US6,029,126 FIG.12).
  * `BlockSizeSwitch` — the alternative mechanism in which the
    encoder picks a block size from the patent-disclosed
    `{256, 512, 1024, 2048, 4096}` set based on transient detection
    (US7,930,171 Background).
* [`TransientSwitch`] is the typed per-block carrier whose two
  variants mirror [`TransientMechanism`]. `SubbandCombineFlag` carries
  the decoded one-bit `combine_high_subbands` value; `BlockSizeSwitch`
  carries the chosen `BlockSize`. Accessors `mechanism`, `block_size`,
  `subband_combine_flag`, and `is_transient_handled` route on the
  variant. For the block-size mechanism, `is_transient_handled` is
  `true` iff the chosen block size is *not* the longest member
  (`S4096`) — encoder-shortened blocks are the patent-named
  transient path per §2.
* [`TransientPlan`] is the per-frame carrier: a fixed
  [`TransientMechanism`] plus a `Vec<TransientSwitch>` whose every
  switch must share that mechanism. `TransientPlan::new` rejects
  mixed-mechanism populations via a new
  `InvalidTransientPlan::MechanismMismatch` error variant that
  reports the offending block index. Accessors expose `len`,
  `is_empty`, `switch_of`, `switches()` iteration, and the predicate
  counts `transient_handled_block_count` / `non_transient_block_count`.

Round 7 adds 23 unit tests covering both mechanism alternatives, both
switch variants, the `is_transient_handled` partition for both
mechanisms, accessor coverage including the per-variant `None`
returns, plan construction accept paths (empty, homogeneous subband,
homogeneous block-size including iteration over all five
`BlockSize::ALL` entries), the mismatch reject at first-offender
position 0 and at a later position, the predicate-count partitioning
invariant, error `Display` formatting and `std::error::Error`
implementation. The crate's test count rises from 163 to 186.

## What is NOT in this round

The wiki snapshot lists the names of WMA's data tables — the gain
Huffman table (37 entries), the LSP codebook, the scale Huffman
table (121 entries), the coefficient 0…5 Huffman tables (666, 555,
1336, 1072, 476, 435 entries), the levels 0…5 tables (60, 40, 340,
180, 70, 40 entries), the per-rate exponent-band partition tables,
and the critical-frequency curves — but the snapshot **does not
contain those tables**. The actual MDCT/Huffman bitstream decode
path therefore stays out of `src/` this round; growing it requires
either a spec PDF or a clean-room reverse-engineered trace doc
staged under `docs/audio/wma/`. See the docs-gap notes in
`src/header.rs` for the boundary cases the wiki leaves
under-specified in the v2 sample-rate normaliser as well.

## Public surface

```rust
use oxideav_wma::{Version, WmaHeader};

// codec ID 0x161 from the container's WAVEFORMATEX
let v = Version::from_codec_id(0x161).unwrap();
let h = WmaHeader::parse(
    v,
    48_000, // sample_rate from container
    2,      // channels
    192_000,// bit_rate
    1024,   // block_align
    &[0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0xCA], // extradata
)
.unwrap();
assert_eq!(h.sample_rate, 44_100);     // v2 snaps 48k → 44.1k
assert_eq!(h.frame_length_bits, 11);   // 2048-sample MDCT block
assert!(h.bit_reservoir);               // flags2 bit 1
```

The [`WmaHeader`] struct exposes every field the wiki names. The
[`oxideav_core::CodecResolver`] registration will land once the
bitstream decode path is implementable.

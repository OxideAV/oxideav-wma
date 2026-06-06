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

**Round 8** (this round) lifts the §4 patent-disclosed
**quantization-band layout** — the structural notion distinct from the
per-band coding-policy carrier already in [`bands`] — into a new
[`qband`] module:

* [`QuantBand`] models the patent's "contiguous frequency range of
  coefficients quantized with the same weighting" definition
  (US7,930,171 / US8,805,696 quantization-band grouping) as a typed
  `{ start, length, weight_index }` triple. The constructor enforces
  the contiguous-range precondition (`length >= 1`) and rejects
  `start + length` overflow with `InvalidQuantBand::ZeroLength` and
  `InvalidQuantBand::EndOverflow` variants. Accessors `start`, `end`,
  `length`, `weight_index`, and `contains(k)` route the band's
  geometric and reference-index information.
* [`QuantBandLayout`] is the ordered partition: a `Vec<QuantBand>`
  that tiles `[0, total_coeffs)` exactly. `QuantBandLayout::new`
  validates the partition shape — bands start at coefficient 0, abut
  with no gap or overlap, and cover the declared total exactly — with
  `InvalidQuantBandLayout` reporting four distinct shape failures
  (`BandCountMismatchEmptiness`, `LeadingGap`, `Gap`,
  `CoverageMismatch`). A `for_block(bands, BlockSize)` constructor
  threads the patent-disclosed transform-block-size set
  `{256, 512, 1024, 2048, 4096}` directly into the layout's declared
  total. Accessors expose `band_count`, `total_coeffs`, `is_empty`,
  `bands()` iteration, `band(i)`, `band_slot_of(k)`,
  `weight_index_of(k)`, and `bands_referencing_weight(d)` for the
  patent-allowed case of multiple bands sharing one weight index.
* [`QuantBandLayout::band_map`] materialises the per-coefficient
  weight-index vector `d(k)` that [`invquant::dequantize_in_place`]
  consumes, threading the patent's "one weight per band" arrangement
  through to the patent's "one weight per coefficient" decoder step
  with one allocation per layout.

Round 8 adds 27 unit tests covering the constructor accept paths
(minimal single-band, abutting pair, empty block, full coverage of
every [`BlockSize::ALL`] entry), all eight reject paths (zero length,
end overflow, empty/nonempty asymmetry, leading gap, gap between
bands, overlap between bands, coverage below and above declared
total), per-band `contains` semantics, `band_map` materialisation and
round-trip with `weight_index_of`, multi-band shared-weight counting,
the patent-named contract that the materialised band map drives
[`invquant::dequantize_in_place`] correctly, and consistent error
`Display` naming for both error enums. The crate's test count rises
from 186 to 213.

**Round 9** (this round) lifts the §6 patent-disclosed **end-of-block
terminator selector** for the spectral-coefficient stream into a new
[`terminator`] module. The trace doc gives both alternatives
side-by-side, without pinning the v1/v2 choice:

> "Termination uses *either a special ending signal* … *or a special
> event such as `(N, 1)`* because the decoder knows the total
> coefficient count for the block."
> — [PATENT US6,223,162 — end-of-stream discussion]

The new module models that selector exactly:

* [`terminator::TerminatorMechanism`] is the typed two-alternative
  selector: `ExplicitEndingSignal` (the patent's "special ending
  signal" branch — symbol pattern is `[GAP]`) and `ImplicitNL1Event`
  (the patent's coefficient-count-driven `(N, 1)` event branch).
  `TerminatorMechanism::ALL` locks the iteration order; `opposite`
  is involutive; `is_compatible_with(pair, total_coeffs)` is the
  patent-compatibility predicate, returning `true` unconditionally
  for the explicit branch (the patent places no structural
  constraint on the final `(R, L)` for that branch) and gating the
  implicit branch on the runlevel-module's
  `is_implicit_terminator_for` predicate.
* [`terminator::TerminatorDecision`] is the per-block carrier whose
  two variants mirror [`terminator::TerminatorMechanism`].
  `ExplicitEndingSignal` is payload-free (the symbol pattern is
  `[GAP]`); `ImplicitNL1Event { terminator_pair }` carries the
  `(R, L)` the upstream reader recognised as the implicit terminator.
  Accessors `mechanism`, `terminator_pair`,
  `is_explicit_ending_signal`, and `is_implicit_n_l1_event` route
  on the variant.
* [`terminator::TerminatorDecision::new_implicit`] is the
  patent-faithful constructor for the implicit branch: it enforces
  the patent's `(N, 1)` predicate against the block's `total_coeffs`,
  surfacing `terminator::InvalidTerminator::PairNotNL1 { run, level,
  total_coeffs }` when the candidate fails.
  [`terminator::TerminatorDecision::new_explicit`] is the matching
  constructor for the explicit branch — no validation is needed
  because the explicit-branch structural shape lives entirely in
  the (still-`[GAP]`) symbol pattern.

Round 9 adds 21 unit tests covering both mechanism variants,
mechanism iteration / opposite involution, accessor coverage
including the per-variant `None` returns, `is_compatible_with`
acceptance (any pair on the explicit branch; only the patent's
`(N, 1)` on the implicit branch — and across every block size in
[`block::BlockSize::ALL`]), `new_implicit` reject paths (level
above one, run below remaining, run above remaining, empty-block
case), cross-module composition with [`runlevel`] confirming both
layers agree on the `(N, 1)` shape, error-`Display` naming, and
`std::error::Error` implementation. The crate's test count rises
from 213 to 234.

**Round 10** lifts the §4 patent-disclosed **per-block
overall step size** into a new [`step_size`] module:

* The §4 trace states "Each coefficient is quantized by the
  **product of its band's matrix weight `Q[c][d]` and a single
  overall step size** for the whole block" (US7,930,171) and
  "an **adaptive, uniform, scalar quantizer that computes one
  quantization factor per tile**" (US7,383,180 quantizer 560).
  The new [`OverallStepSize`] is the typed carrier for that
  per-block factor: a single `f64` validated at construction to be
  finite, non-NaN, and strictly positive — the patent-implied
  preconditions for the forward step
  `q = round(coeff / (Q[d] * step))` and the decoder inverse
  `coeff_hat = q * Q[d] * step` to be well-defined and sign-faithful.
* [`OverallStepSize::new`] reports rejection via a typed
  [`step_size::InvalidStepSize`] enum (`NotANumber`, `NotFinite`,
  `NotPositive`); the type implements [`std::error::Error`].
  Accessors expose [`OverallStepSize::value`] (the `f64` for
  [`invquant::BandScale::from_weights`] consumption),
  [`OverallStepSize::apply_to_weight`] (the patent's per-band
  `Q[d] * step` factor for a single band), and
  [`OverallStepSize::band_scale_from_weights`] (build a
  [`BandScale`] sized to a slice of per-band weights).
* [`PerBlockStep`] pairs a [`BlockSize`] with an
  [`OverallStepSize`] to model the patent's "one step per tile"
  arrangement; `coefficient_count()` re-exposes the block-size's
  sample count for the per-coefficient dequant loop;
  `fold_with_weights()` materialises the patent's per-band
  `Q[d] * step` folded scale via [`BandScale`].
* Cross-module: an end-to-end test threads
  `PerBlockStep::fold_with_weights` into [`BandScale::apply`] and
  confirms the result matches [`invquant::dequantize_in_place`]
  given the same opaque step.

Round 10 adds 26 unit tests covering constructor accept/reject paths
(typical positive, smallest subnormal positive; zero, negative zero,
negative finite, ±∞, NaN), accessor coverage, the
`apply_to_weight` ↔ `value()` commutativity, the
`band_scale_from_weights` ↔ free-function equivalence, the
[`PerBlockStep`] coverage across every [`BlockSize::ALL`] entry, the
`fold_with_weights` ↔ free-function equivalence end-to-end against
[`invquant::dequantize_in_place`], `PartialEq` differentiating on
both block and step, and `Display` naming for both the carrier and
each [`step_size::InvalidStepSize`] variant. The crate's test count
rises from 234 to 260.

**Round 11** (this round) lifts the §6 patent-disclosed
**escape-symbol literal payload** into a new [`escape`] module. The
trace doc records the patent's structural disclosure as:

> "A pairing that falls below the threshold (not in the code book) is
> emitted with an **escape/special symbol** followed by enough
> literal information to identify the zero-run length and the
> non-zero sample value."
> — [PATENT US6,223,162 — escape symbol; Claim 4; Claims 5–6]

The new module realises the typed carrier for that literal trailer:

* [`escape::EscapeLiteral`] is the typed payload — `{ run: u32,
  level: NonZeroU32 }` — that follows the patent's escape symbol on
  the wire. The constructor [`escape::EscapeLiteral::new`] reuses
  [`runlevel::RunLevelPair::new`] to enforce the patent's Claim-1
  (`run ≥ 1`) and Claim-2 (`level ≥ 1`) predicates and reports
  failures via a typed [`escape::EscapeError::InvalidPair`] wrapping
  the [`runlevel::InvalidPair`] reason.
* [`escape::EscapeLiteral::for_pair`] is the grid-checked variant.
  It takes a [`CodebookGrid`] + [`runlevel::RunLevelPair`] and
  produces the typed literal **only** when
  `grid.disposition(pair) == Disposition::Escape` — the patent's
  Claim 4 condition for entering the escape branch. An in-codebook
  pair surfaces as [`escape::EscapeError::InCodebook`], whose
  `Display` cites US6,223,162 Claim 4 directly.
* The Claim-5/6 decoder side is realised as
  [`escape::EscapeLiteral::as_run_level_pair`], which rebuilds the
  [`runlevel::RunLevelPair`] the literal carries. Together with
  the constructor, this gives an exact round-trip
  `pair → EscapeLiteral → pair` for any escape-branch pair.
* The literal's `run` and `level` widths are deliberately wide
  (`u32` each). The patent fixes the structural presence of the
  literal payload but leaves the bit widths as `[GAP]` in the §6
  trace, so the carrier accepts whatever value the upstream
  entropy reader recovers — including the `u32::MAX` boundary,
  exercised by tests.

Round 11 adds 18 unit tests covering the constructor accept paths
(minimum (1, 1), large values, `u32::MAX` on both fields), all
reject paths (`run == 0`, `level == 0`, both zero), the `for_pair`
cross-check against a 2×2 codebook grid (in-codebook pair rejected;
below-threshold escape pair accepted; outside-rectangle pair
accepted), accessor coverage, round-trip through
`as_run_level_pair` for both constructors and at the `u32::MAX`
boundary, error `Display` strings (`InvalidPair` mentions "run";
`InCodebook` mentions "US6,223,162" and "Claim 4"), and a
structural-invariant test that walks every cell of a 3×3 grid and
confirms `for_pair` accepts every escape disposition and rejects
every in-codebook disposition. The crate's test count rises from
260 to 278.

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

# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `terminator` module — end-of-block terminator selector for the
  spectral-coefficient stream, covering both patent-disclosed
  alternatives the §6 trace names side-by-side (US6,223,162
  end-of-stream discussion: "either a special ending signal… or a
  special event such as `(N, 1)` because the decoder knows the total
  coefficient count for the block"). Public `TerminatorMechanism`
  enum with `ExplicitEndingSignal` and `ImplicitNL1Event` variants,
  `TerminatorMechanism::ALL`, `is_explicit_ending_signal`,
  `is_implicit_n_l1_event`, `opposite`, and a patent-faithful
  `is_compatible_with(pair, total_coeffs)` predicate. Per-block
  `TerminatorDecision` enum mirroring the mechanism with an
  `ImplicitNL1Event { terminator_pair }` payload; `new_explicit()`
  is payload-free (the symbol pattern is `[GAP]`),
  `new_implicit(pair, total_coeffs)` enforces the patent's `(N, 1)`
  predicate via `InvalidTerminator::PairNotNL1 { run, level, total_coeffs }`.
  Cross-module: composes with
  `runlevel::RunLevelPair::is_implicit_terminator_for` so the
  implicit branch and the runlevel walker share the patent's `(N, 1)`
  shape. Re-exports: `TerminatorDecision`, `TerminatorMechanism`.
  Adds 21 unit tests; crate total rises from 213 to 234.
- `qband` module — quantization-band layout carrier covering the §4
  patent-disclosed structural notion (US7,930,171 / US8,805,696
  quantization-band grouping; "contiguous frequency range of
  coefficients quantized with the same weighting"). Public
  `QuantBand { start, length, weight_index }` with `QuantBand::new`
  constructor enforcing `length >= 1` and `start + length` overflow
  guard via `InvalidQuantBand::{ZeroLength, EndOverflow}`; accessors
  `start`, `end`, `length`, `weight_index`, and a `contains(k)`
  membership predicate. `QuantBandLayout` aggregates a `Vec<QuantBand>`
  partitioning `[0, total_coeffs)`; `QuantBandLayout::new` validates
  the partition shape with `InvalidQuantBandLayout::{BandCountMismatchEmptiness,
  LeadingGap, Gap, CoverageMismatch}` reporting the offending position
  in each case. A `QuantBandLayout::for_block(bands, BlockSize)`
  convenience constructor threads the patent's transform-block-size
  set directly into the declared total. Accessors expose
  `band_count`, `total_coeffs`, `is_empty`, `bands()`, `band(i)`,
  `band_slot_of(k)`, `weight_index_of(k)`, and a
  `bands_referencing_weight(d)` count for the patent-allowed case of
  multiple bands sharing one weight index. The `band_map()` helper
  materialises the per-coefficient weight-index vector `d(k)`
  consumed by `invquant::dequantize_in_place`, threading the patent's
  per-band weight assignment into the per-coefficient dequant loop.
  27 unit tests cover the constructor accept paths (minimal
  single-band, abutting pair, empty block, every member of
  `BlockSize::ALL`), all reject paths (zero length, end overflow,
  empty/nonempty asymmetry, leading gap, gap, overlap, coverage
  below/above declared total), `contains` semantics, accessor lookup
  for in-range and out-of-range coefficients, `band_map`
  materialisation and round-trip with `weight_index_of`, multi-band
  shared-weight counting, an end-to-end check that the materialised
  map drives `invquant::dequantize_in_place` correctly, and
  `InvalidQuantBand` / `InvalidQuantBandLayout` `Display` naming.
- `transient` module — per-block transient-handling switch carrier
  from §3 of the patent trace. Public `TransientMechanism` enum names
  both patent-disclosed mechanisms side-by-side: `SubbandCombineFlag`
  (one-bit per-block side-information flag, US6,240,380 FIG.12 boxes
  1210–1250 / US6,029,126 FIG.12) and `BlockSizeSwitch` (selection
  from the patent-disclosed `{256, 512, 1024, 2048, 4096}` set,
  US7,930,171 Background). `TransientSwitch` variants mirror the
  mechanisms and carry their patent-named payloads
  (`combine_high_subbands: bool` and `block_size: BlockSize`), with
  `mechanism`, `block_size`, `subband_combine_flag`, and
  `is_transient_handled` accessors. `TransientPlan` aggregates a
  per-frame `Vec<TransientSwitch>` behind a single declared
  `TransientMechanism`, validating homogeneity at construction;
  `InvalidTransientPlan::MechanismMismatch { at_block, expected, got }`
  reports the offending block index. Plan accessors expose `len`,
  `is_empty`, `switch_of`, `switches()` iteration, and the
  `transient_handled_block_count` / `non_transient_block_count`
  partition. 23 unit tests cover both mechanism alternatives, both
  switch variants, the `is_transient_handled` partition for both
  mechanisms (block-size variant treats every non-`S4096` size as
  transient-handled), variant-specific accessor `None` returns, plan
  construction over empty / homogeneous subband / all-five
  `BlockSize::ALL` populations, mismatch rejection at first-offender
  position 0 and at a later index, the count-partitioning invariant,
  and the `InvalidTransientPlan` `Display` + `std::error::Error`
  implementation.
- `codebook` module — `(R, L)` probability-grid + threshold model for
  the run-level codebook construction step from §6 of the patent trace
  (US6,223,162 grid 500 / threshold 518 / FIG.6 / Claims 4–10). Public
  `CodebookGrid::from_probabilities(rm, ln, threshold, probs)` builder
  with constructor-side validation of `rm >= 1`, `ln >= 1`, `[0.0, 1.0]`
  threshold and probability ranges, and `probs.len() == rm * ln`.
  Lookup via `probability_of(r, l) -> Option<f64>` (`None` outside the
  `(rm, ln)` rectangle); a typed `Disposition` enum (`InCodebook` /
  `Escape`) reports the patent-disclosed disposition of a
  `runlevel::RunLevelPair` via `disposition`, `is_in_codebook`,
  `is_escape`. Above-threshold counting and iteration:
  `in_codebook_count`, `escape_count_in_rectangle`, and
  `in_codebook_pairs()` (row-major run-outer / level-inner order). A new
  `InvalidGrid` error names every constructor reject path
  (`ZeroRm`, `ZeroLn`, `ThresholdOutOfRange`, `DimensionsOverflow`,
  `ProbabilityLengthMismatch`, `ProbabilityOutOfRange`). 27 unit tests
  cover constructor accept paths, all reject paths (incl. NaN
  threshold/probability), row-major lookup vs. outside-rectangle
  `None`, the inclusive `>=` cutoff rule including the at-exact-
  threshold case, escape on outside-rectangle pairs, count
  partitioning, row-major iteration order, threshold-0.0 full and
  threshold-1.0 empty population cases, cross-module orthogonality
  with the `(N, 1)` implicit terminator from `runlevel`, and consistent
  `InvalidGrid` Display naming.
- `invquant` module — decoder-side inverse-quantization helpers from
  §4 of the patent trace (US7,930,171 overall step-size description /
  US7,383,180 inverse quantizer-weighter FIG.6 / US6,240,380
  re-weighting at decoder). Public `dequantize_sample` (per-sample
  `q * weight * step`) and `dequantize_in_place` (whole-block over a
  band map) helpers, plus a `BandScale { scale: Vec<f64> }` carrier
  precomputing the per-band `Q[d] * step` product so the inner
  dequant loop multiplies once per coefficient. 18 unit tests
  covering the dead-zone identity (q == 0 → 0 for any weight/step),
  linearity in q, factor commutativity, whole-block per-band
  threading, length-mismatch / band-index-overflow panic contracts,
  empty-block boundary, `BandScale` construction + lookup,
  `BandScale::apply` parity with `dequantize_in_place`, an
  encoder-quantizer round-trip identity at exact-grid coefficients,
  and a non-contiguous band-layout case.
- `bands` module — per-band coding-policy carrier covering the three
  patent-disclosed §7 alternatives: `BandPolicy::Coded` (literal
  entropy coding; US7,383,180 default), `BandPolicy::NoiseSubstituted
  { energy: f64 }` (US7,383,180 / US7,343,291 noise substitution +
  decoder module 240), and `BandPolicy::Truncated` (US7,383,180
  high-band truncation cutoff). Public predicates `is_coded`,
  `is_noise_substituted`, `is_truncated`, plus a `noise_energy`
  accessor. A `BandPlan { policies, cutoff }` descriptor exposes
  `policy_of`, `coded_band_count`, `noise_band_count`,
  `truncated_band_count`, and `cutoff_index`. Two constructors:
  `BandPlan::new` (no shape promise) and `BandPlan::new_with_cutoff`
  (enforces the patent's contiguous-tail truncation shape) with the
  new `InvalidBandPlan::TruncatedNotContiguousTail { at_band }` error
  variant. 18 unit tests covering the three-way predicate exclusivity,
  the noise-energy accessor's selectivity, `new_with_cutoff`'s accept
  paths (no truncation / contiguous tail / all-truncated /
  single-at-end / empty), reject paths (truncated → coded; truncated
  → noise), per-band count partition, error display naming, and a
  cross-module check that the cutoff models the patent's high-band
  truncation shape.
- `qmatrix` module — invertible differential-coding helpers for the
  per-band quantization matrix carriage from §4 of the patent trace
  (US7,930,171 step 120 / US7,502,743). Public functions
  `differential_encode` / `differential_decode` (fresh `Vec`) and
  matching `_in_place` variants over `&mut [i32]`; the transform is
  bijective under wrapping `i32` arithmetic. A `zero_delta_pad`
  helper applies the patent's "set unneeded element = next needed
  element" encoder policy against a `[bool]` needed-mask. 15 unit
  tests covering empty / single-element / arbitrary round-trip,
  equal-sequence zero-delta property, monotone and decreasing
  delta-pattern fingerprints, i32 extreme boundary handling,
  `zero_delta_pad` substitution semantics including the
  no-next-needed trailing-run no-op, and a cross-helper
  in-place-vs-fresh-Vec equivalence check.
- `entropy_mode` module — `EntropyMode { Level, RunLevel }` enum
  capturing the patent-disclosed mode-switching primitive from §6
  of the trace (US6,223,162 mode selector 400 / US7,383,180 entropy
  encoder 570). `EntropyMode::ALL`, `opposite()` (involutive
  helper), and `is_level()` / `is_run_level()` predicates. A
  companion `Partition { total_coeffs, split, adaptive }` carrier
  with `mode_for(index)`, `level_range_len()`,
  `run_level_range_len()`, `is_adaptive()` / `is_predetermined()`
  helpers and a validating `Partition::new` constructor that
  rejects out-of-block splits with
  `InvalidPartition::SplitOutOfBlock`. 16 unit tests covering the
  mode enum's predicate exclusivity and involution, the partition
  constructor's accept/reject paths (including the boundary cases
  `split == 0` and `split == total`), `mode_for` lookup for low /
  high / out-of-range indices, the adaptive-vs-predetermined
  complement, range-length accounting, and a cross-module check
  that a partition can be built for every patent-disclosed
  `BlockSize`.
- `BlockSize` enum (`block` module) capturing the patent-disclosed
  WMA Standard transform-block-size set `{256, 512, 1024, 2048, 4096}`
  samples from `docs/audio/wma/wma-bitstream-from-patents.md` §2
  (US7,930,171 / Chen-171 Background).
- `BlockSize::ALL` constant (ascending order), `samples()` and
  `log2_samples()` accessors, validating constructors
  `BlockSize::from_samples` / `BlockSize::from_log2`, and
  `is_shortest()` / `is_longest()` outer-bound predicates.
- `Error::InvalidBlockSize { samples }` variant for non-set values.
- 14 unit tests covering the patent set, ascending iteration,
  log2 ↔ samples round-trip, `from_samples` accept/reject paths
  (including non-power-of-two and zero), `from_log2` accept/reject
  paths (including saturation on absurd exponents), the outer-bound
  predicates, and a cross-module check that every `WmaHeader::frame_length`
  Round 1 produces is itself a member of the patent set.
- `stereo` module — sum/difference (mid/side) two-channel transform
  from §5 of the patent trace (US7,930,171 / US7,502,743). Public
  helpers `mid`, `side`, `forward`, `inverse` (per-sample) and
  `forward_in_place`, `inverse_in_place` (whole-block, slice-paired).
  13 unit tests covering the channel-average / half-difference
  identities, the algebraic round-trip in both directions, the
  correlated/anti-correlated energy-concentration cases, and the
  panic-on-mismatch contract of the slice helpers.
- `runlevel` module — typed `RunLevelPair { run: u32, level: NonZeroU32 }`
  from §6 of the patent trace (US6,223,162 Claims 1–2, US7,885,819).
  Constructor `RunLevelPair::new` enforces `run ≥ 1` and `level ≥ 1`
  per the patent set with `InvalidPair::{ZeroRun, ZeroLevel}` error
  variants. Accessors `coefficient_count` and
  `is_implicit_terminator_for` plus the `expand_into` walker that
  decodes a pair sequence into a sparse coefficient block, honouring
  both the implicit `(N, 1)` terminator and explicit underrun with
  `WalkError::{Overflow, Underrun}`. 20 unit tests covering the
  constructor reject paths, the `(N, 1)` terminator predicate, the
  walker's happy paths (natural fill, implicit terminator), the
  overflow / underrun error paths, the empty-block boundary, and an
  end-to-end hand-crafted sparse-spectrum round-trip.

## [0.0.2](https://github.com/OxideAV/oxideav-wma/releases/tag/v0.0.2) - 2026-05-29

### Other

- Round 1 — WAVEFORMATEX-extradata header parser + v1/v2 frame-length tree
- Round 0 — clean-room rebuild scaffold (orphan master)

### Added

- `Version` enum (v1 / v2) recoverable from `WAVEFORMATEX` codec ID
  (`0x160` / `0x161`).
- `WmaHeader` struct holding the container-supplied fields
  (`sample_rate`, `channels`, `bit_rate`, `block_align`) plus the
  parsed extradata (`flags1`, `flags2`, the three low `flags2` bits as
  named booleans, `frame_length_bits`, `frame_length`).
- `WmaHeader::parse(version, sample_rate, channels, bit_rate, block_align, extradata)`
  parser. Supports v1 (4-byte) and v2 (6-byte) extradata layouts,
  applies the version-specific frame-length-bits decision tree, and
  applies the v2 sample-rate normaliser at its single explicit cutoff
  (`sample_rate >= 44_100` snaps to `44_100`).
- `normalize_sample_rate_v2` helper exposing the v2 sample-rate
  normaliser as a standalone function.
- `Error::ExtradataTooShort { expected, got }` and
  `Error::InvalidContainerField { field }` variants.
- 21 unit tests covering the extradata layouts, every flags2 bit, every
  branch of the frame-length decision tree (including the v1-only
  32 kHz special case), the explicit v2 44.1 kHz cutoff, and the error
  paths for short extradata and zero `sample_rate`.

### Changed

- Clean-room rebuild from a fresh orphan `master`. The previous
  implementation was retired by the OxideAV docs audit dated
  2026-05-06; the prior history is preserved on the `old` branch.
  See `README.md` for the rebuild scope and the strict-isolation
  workspace the Implementer rounds will draw from.

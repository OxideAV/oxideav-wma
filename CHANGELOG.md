# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

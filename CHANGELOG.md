# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

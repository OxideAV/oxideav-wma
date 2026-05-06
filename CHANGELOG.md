# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.3](https://github.com/OxideAV/oxideav-wma/compare/v0.0.2...v0.0.3) - 2026-05-06

### Other

- prepend retirement notice (docs audit 2026-05-06)
- registry calls: rename make_decoder/make_encoder → first_decoder/first_encoder

## [0.0.2](https://github.com/OxideAV/oxideav-wma/compare/v0.0.1...v0.0.2) - 2026-05-03

### Other

- escape `>=` to dodge clippy doc-quote warning
- cargo fmt rustfmt 1.95 fn-signature collapse
- clear clippy warnings in frame-length lookup and decode_frame

### Added
- Initial scaffold (round 1).
- WMA v1 and v2 baseline decoder (single-block-size MDCT path).
- ASF Header / Stream Properties / Data Object reader to extract the
  WAVEFORMATEX-trailing extradata (`flags2`) and the per-packet payload.
- Bark-scale critical-band partitioning (live computation for v1 + v2)
  and the three precomputed `exponent_band_*[3][25]` overrides for v2 at
  22050 / 32000 / 44100 Hz.
- Six WMA v1/v2 spectral run-level VLCs (codebooks 0..5) with bit-rate /
  sample-rate-driven selection.
- AAC scale-factor codebook (re-used for VLC-coded exponents) and the
  `pow_tab[156]` pre-quantised antilog table.
- Plain sine MDCT window + IMDCT with overlap-add.
- ffmpeg-roundtrip integration tests for both wmav1 and wmav2 (PSNR
  envelope check against the original PCM).

### Deferred
- WMA Pro (vector VLCs, Givens-rotation channel transform, per-channel
  tile layout) — round 2.
- WMA Lossless (Rice-coded residues + cascaded LMS predictors) — round 2.
- Bit reservoir + variable block lengths inside a frame (the spec-
  optional `flags2.1` / `flags2.2` paths). FFmpeg's encoder does not
  set them on synthetic streams; round-2 work item.
- Noise-coded high-band synthesis (also off in our test corpus).
- Encoder.

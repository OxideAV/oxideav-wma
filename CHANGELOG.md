# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.5](https://github.com/OxideAV/oxideav-wma/compare/v0.0.4...v0.0.5) - 2026-09-06

### Other

- README for the r457 encoder campaign + noise-substitution / policy findings
- noise substitution end to end: F3/F4 emission, measured level law, decoder synthesis, cost election; cutoff-bin walk start
- per-frame allocation + stereo election by a masking-aware cost; dead zone; bisected reservoir-paced rate control
- encoder ladder — every encodable v2 catalogue cell, own chain + black-box reference per cell
- noise-substitution policy across the sample-rate axis + per-short-block B2 on every stream
- ABS_SCALE recalibrated per channel — the r450 fit absorbed the reference downmix's 1/sqrt2
- hide internal pub surface from rustdoc/semver (fleet rule 2026-09-01)

### Fixed

- **Absolute output scale recalibrated (r457)** — `vendor_decode::ABS_SCALE`
  was fitted in r450 against the black-box reference's stereo→mono
  downmix compared with this decoder's `(L + R) / 2`; the reference's
  downmix weights are `1/√2` per channel, so the fit absorbed a factor
  √2 and every decode ran 3 dB loud (the mono 22.05 kHz family's r454
  fitted gain of 1.40 was the tell). The scale is now `-4.844e-2`;
  measured per channel the reference's fitted gain is 0.99–1.00 on the
  three fully-closing vendor families and on this crate's own encoded
  streams at every total gain and envelope anchor. The vendor-stream and
  encoder acceptance tests now decode the reference at the stream's own
  channel count and fit per channel; the encoder leg also measures SNR
  over the overlap interior (the r454 leg charged the reference's shorter
  tail as error, which capped every family near 14 dB — like for like the
  reference SNR tracks the own-chain SNR within ≈ 0.5 dB, 20–30 dB on
  the four families).

- **§2.1 noise-substitution policy generalised across the
  sample-rate axis (r457)** — `vendor_frame::measured_noise_policy`
  now covers 11.025–48 kHz: enabled always at 11.025/16 kHz, below the
  staged 1.16 class-2 threshold at 22.05 **and 32 kHz**, at rate
  floats ≤ 0.6 at 44.1/48 kHz, never at 8 kHz; the walk starts at the
  band containing the bin of a critical-band-seed cutoff
  (`NoiseStart::CutoffHz`: 3700 Hz at 11.025/16 kHz, 6400/7700 Hz at
  22.05 kHz switching at the 0.72 class-1 threshold, 9500 Hz at 32 and
  48 kHz, 7700 Hz at 44.1 kHz), verified at every block size 128–2048
  through explicit mixed block schedules against the black-box
  reference; the r454 22.05 kHz 256-block start (148) stays as an
  override the vendor mono stream votes for (97/122 vs 61/122). This
  isolates the README's "16/32 kHz divergence": every 32 kHz class-2
  configuration (rate float 0.72–1.16) — nine ACM catalogue cells —
  decoded to garbage at the reference and now decodes at 21–26 dB.
- **B2 envelope-reuse bit is per short block on every stream
  (r457)** — `ReuseRule::ShortBlockPerBlock` is the default: measured
  on noise-disabled mono configurations at 48 / 44.1 / 22.05 / 8 kHz
  (every short size 1024–128) the reference decodes only the
  per-block-bit emission; r446's two-channel gate was the same rule
  restricted to the stereo streams it could see. Vendor closure
  unchanged (1738/1763).

### Added

- **§2.1 noise substitution, end to end (r457)** — the emitter writes
  F3 flags and F4 gains (`EncChannelData::noise_flags` /
  `noise_gains`; the flagged bins leave the coefficient axis; typed
  errors for a wrong walk length, gain count or gain range), the
  decoder fills flagged bands with noise, and the encoder elects
  substitution per walk band by cost
  (`EncoderSettings::noise_substitution`, default on): a band whose
  coded reconstruction would leave more than a quarter of its energy as
  error — a hole, or a few isolated ±1s standing in for noise — is sent
  as its gain instead, which is bits saved. The **level law** is
  black-box measured (`vendor_decode::noise_band_rms`): the reference
  fills a flagged band with white noise at a per-coefficient RMS of
  `10^((G − 64)/20) · w_band · |ABS_SCALE|` — the F4 gain plays the
  total gain's role on a unit-RMS generator (1 dB per gain step,
  0.0001 → 0.0003 → 0.001 → 0.003 → 0.03 across G = 10/20/30/40/60),
  following the band exponent at the ladder ratio, independent of the
  block's total gain and of the coded coefficients, one gain per band —
  verified at 22.05 kHz and at 44.1 kHz (2048 and 512 blocks). The
  reference accepts the flagged streams on every noise-enabled cell
  (`tests/encoder_streams.rs` `mono22k_16kbps_hiss`), and the vendor
  mono 22.05 kHz stream keeps its floors (its flagged bands now carry
  noise instead of zeros; corr² 0.951 → 0.946 is the uncorrelated
  fill).
- **The noise walk's first band starts at the cutoff bin rounded
  up** — `vendor_frame::noise_walk_bands` / `noise_walk_bands_for` are
  the one definition of the §2.1 walk (parser, emitter, decoder and
  encoder all read it): for the cutoff-frequency starts the first
  walked band begins at `ceil(f · 2M / sample_rate)` inside the band
  containing the rounded-to-four bin (358 instead of the 356 edge on
  512-blocks at 22.05 kHz; 716 = the edge on 1024-blocks). The vendor
  mono 22.05 kHz stream — the only vendor stream carrying the
  sub-stream — closes **103/122** §1 boundaries under it (97 band
  edge, 94 nearest, 90 one bin lower; the 256-block hard-table start
  stays at 148 — 66/122 without it). The black-box reference reads a
  flagged axis one coefficient shorter than that at every block size
  probed (22.05 / 32 / 44.1 / 48 / 16 kHz: it rejects the last index),
  a reference-vs-vendor divergence the encoder sidesteps: flags are
  elected as a top-band suffix and the last index of a flagged axis is
  never coded.
- `BlockSynth::with_zero_fill_noise` (off by default) —
  `vendor_decode::ZERO_FILL_RMS_STEPS`: black-box measured, the reference
  fills **every** zero-quantised bin of a coded channel with white noise
  at `0.4 · step` (0.0006 / 0.006 / 0.06 at g = 60 / 80 / 100, following
  the band exponents), flags or not; nothing staged describes this
  floor, so it is an option, not the default.

### Changed

- **Encoder allocation: per-frame election by a masking-aware cost
  (r457)** — `EncoderSettings::allocation` (`Allocation::Adaptive`,
  the default) rate-controls three envelopes per frame — flat, the
  patent §4 Bark-spread masking curve half-whitened (β = 0.5) and in
  full (β = 1) — and, on stereo streams under `StereoMode::Auto`,
  both the independent and the mid/side realisation, all to the same
  bit target, and keeps the candidate with the lowest
  `mse · (1 + audible/mse)²` where `audible` is the reconstruction
  noise above the masking threshold. `Allocation::Rms` (the r454
  rule), `Masking { beta }` and `Flat` remain selectable. The
  rate-matched measurement that shaped this (own chain, five catalogue
  cells × four materials): at 16–64 kbps the **flat** envelope beats
  both the RMS and the masking-shaped envelopes on SNR *and* on
  noise-to-mask (mono 22.05 kHz / 16 kbps "varying": flat 17.7 dB /
  NMR −10.4 vs masking-β0.7 14.5 / −7.7 vs RMS 10.8 / −6.3), because
  shaping spends bits on masked bands and forces a coarser global
  step; from 128 kbps, where the format's 9-bit peak ceiling binds,
  shaping wins noise-to-mask (−28.7 vs −24.1 dB) at a small SNR cost.
  The election tracks that crossover: against the r454 encoder on the
  ladder it is +3–9 dB own-chain SNR on every cell (stereo 22.05 kHz /
  32 kbps 21.0 → 30.6 dB on tones, 14.6 → 21.3 dB on modulated
  material, 2.3 → 6.3 dB on wideband noise) at the same rate, with
  the mid/side election adding 3–4 dB on correlated stereo over
  either fixed mode.
- **Quantiser dead zone** (`EncoderSettings::dead_zone`, default
  0.2 steps): +0.5–1.3 dB at equal rate on every cell.
- **Rate control**: the gain offset is bisected (finest realisation
  that fits) against a reservoir-paced per-frame target (unspent
  budget carries forward, a frame may borrow one average frame),
  replacing the ±3/−4 walk; streams land at the nominal rate.

### Added

- **Encoder ladder** (`tests/encoder_ladder.rs`): every encodable
  WMA v2 cell of the staged ACM catalogue (21 cells: 22.05–48 kHz,
  mono/stereo, 16–160 kbps, headerless and reservoir/VBL geometries)
  plus the vendor-stream geometries, each encoded under the default
  transient-splitting schedule and a fixed full-block schedule,
  decoded through the own chain and the black-box reference, reported
  per cell (own SNR / reference corr² / gain / SNR / rate used) and
  held to reference-tracks-own acceptance. Baseline at r457: own
  20–29 dB, reference 22–37 dB, gain 0.99–1.01 on all 26 cells.
  Shared helpers moved to `tests/common/mod.rs`.
- `BlockPolicy::Pattern` — explicit per-frame block schedules;
  `EncoderSettings::noise` (`NoisePolicy::Measured` / `Off` / `Spec`) and
  `FrameParser::without_noise` / `FrameEmitter::without_noise` — the
  measurement hooks the policy sweep runs through.
- `EncoderSettings::envelope_anchor` / `envelope_range` — the envelope
  anchor and depth are settings (defaults 40 / 24, the r454 constants,
  now `pub` as `ENVELOPE_ANCHOR` / `ENVELOPE_RANGE`). Black-box sweep:
  anchors 32–80 decode identically at the reference; 16/24 lose 8–16 dB
  (exponents clamp at 0); 100 is rejected outright.

## [0.0.4](https://github.com/OxideAV/oxideav-wma/compare/v0.0.3...v0.0.4) - 2026-08-31

### Other

- README/CHANGELOG for the r454 encoder + calibration arc
- vendor_encode_roundtrip — structure-aware encoder round trip
- encoder end-to-end acceptance — own-chain floors + black-box wire acceptance
- the measured par.2.1 noise-substitution policy
- computed-walk rounding is nearest-multiple-of-four (black-box calibrated)
- WmaEncoder + make_encoder — the dual-API encoder surface
- PCM -> packets encoder (forward mirror of the vendor decode chain)
- par.2-par.4 frame emitter + par.1 packet writer (encoder wire mirror)
- reverse (run, level) -> symbol index for the encoder mirror

### Added

- **Vendor-wire encoder, end-to-end (r454)** — the encoder mirror is
  no longer self-consistent-only: `vendor_encode` carries the §2–§4
  frame/block bit **emitter** (`FrameEmitter`: F1 one-ahead pipeline
  with the three-field opening latch, F2a/F2, B1 gain chaining, the
  per-block B2 rule, §3 scale-VLC envelopes incl. the v1 base, §4
  run-level coefficients over the staged vendor codes with
  companion pairs / escapes / EOB / channel-scoped ALT) and the §1
  packet writer (`VendorBitWriter`: back-to-back body stream, P1/P2/P3
  derived from where frame boundaries fell, zero-carry padding as the
  flush mechanism, hard per-frame §1 bounds). `vendor_analysis`
  supplies the signal stage (forward lapped transform at the
  synthesiser's own slot geometry, envelope extraction on the staged
  ladder scale, quantisation by the exact decode composition, §5
  mid/side fold with the encoder-side halving, a transient block
  scheduler, per-frame rate control with the gain floored at the
  escape-level ceiling), and `VendorEncoder` drives PCM → packets.
  `wire_vlc::runlevel_index` provides the reverse `(run, |level|) →
  symbol` lookup.
- **Encoder registration** — `WmaEncoder` (core `Encoder` impl,
  interleaved-F32 in / `block_align` §1 packets out at flush) and
  `make_encoder`; the registry entries install the encoder factory
  alongside the decoder.
- **Measured §2.1 noise-substitution policy**
  (`vendor_frame::measured_noise_policy`, r454 black-box
  calibration): enabled at 22.05 kHz below the staged 1.16
  class-selector threshold on the rate float; exponent-band walk
  from per-size start edges 716/356/148 (1024/512/256); every short
  block carries the B2 bit on enabled streams (supersedes r446's
  "mono reads no B2", which the then-unknown F3 bits confounded).
  Parser, emitter and synthesiser apply it by default;
  `NoiseStart::StartEdges` carries the measured starts. This opens
  the `cand_mono22k_16kbps` vendor stream — the old "F1 anomaly"
  family: closures 64/122 → 97/122, corr² 0.004 → 0.951, ≈ 14 dB
  median SNR.
- Encoder acceptance tests (`tests/encoder_streams.rs`): per-family
  own-chain SNR floors plus black-box wire-format acceptance
  (RIFF/WAVEFORMATEX wrap → reference decode, corr² 0.98–0.995 at
  fitted gain ≈ 1), and a structure-aware encoder round-trip fuzz
  target (`vendor_encode_roundtrip`).

### Changed

- **Computed exponent-band partition rounding** (r454 black-box edge
  probe): the critical-band walk rounds to the **nearest** multiple
  of four (the staged hard-table `((e + 2) >> 2) << 2` post-pass),
  not truncation — resolving the staged `.meta`'s explicit rounding
  caveat. This was the decoder's dominant residual error: vendor
  per-second median SNR moves 27.3 → 45.3 dB (stereo 22.05 kHz),
  18.2 → 60.4 dB (44.1 kHz VBR), 20.9 → 50.3 dB (96 kbps);
  regression floors raised accordingly.
- `BlockSynth` maps coefficients around flagged §2.1 noise bands
  (substituted bands zero-fill; the vendor noise generator is
  unstaged, F4 gains are parsed and carried).

### Added

- **`oxideav_core` registration + `WmaDecoder` + `make_decoder`
  (r450)** — the crate's dual API surface (`registration` module,
  `register!` entry point): decoder factories for codec ids
  `wma1`/`wma2` with their `WAVEFORMATEX` tag claims
  (`0x0160`/`0x0161`). `WmaDecoder` wraps the full vendor decode
  chain behind the core `Decoder` trait — one `Packet` = one
  `block_align`-sized codec packet (`block_align` locks from the
  first packet's length), one-packet latency for the §1 reservoir
  carry, interleaved F32 output in the reference ±1.0 convention,
  silence substitution for unparseable frames (the §1 frame counts
  keep the timeline), `reset()` for post-seek reuse. Pinned
  sample-exact against the direct chain on all six committed vendor
  streams (11.4 M samples).
- Incremental-decode plumbing: `BitWriter::as_bytes`,
  `PacketAssembler::total_bits`/`reader_at`,
  `FrameParser::note_body_start`.
- `ParsedBlock::prev_size`/`next_size` — the F1 windowing context
  (the three-field opening's previous size and the one-ahead
  pipeline's pre-read next size), previously consumed and dropped.
- The `vendor_parse` fuzz target now drives the full decode chain
  (parse → synthesis → flush).

### Changed

- **Variable-size lapped reconstruction + calibrated dequantisation
  composition (r450)**, both selected by measurement against the
  black-box reference decode of the six committed vendor streams:
  - the truncation-aligned overlap-add (which dropped the long tail
    at every long→short block transition) is replaced by the
    standard variable-block-size lapped construction — each block's
    2M transform samples centred on its M-sample slot, sine slopes
    of length `min(M, prev)`/`min(M, next)` centred on the slot
    boundaries (power-complementary across every transition; equal
    sizes reduce to the plain sine window); `BlockSynth` now runs an
    accumulator with a fixed `frame_length/2` lead-in and gains a
    `flush()`;
  - the composition rule (open in the staged docs): band weight
    stays on the staged `10^((e − e_max)/16)` ladder ratio anchored
    at the block's maximum exponent, total gain folds in at **1 dB
    per B1 step** (`10^((g − 64)/20)`; the sweep optimum is sharply
    at 1/20, corroborated by the staged escape-width table), and a
    single black-box-calibrated absolute scale
    (`vendor_decode::ABS_SCALE`) lands PCM in the reference's ±1.0
    float convention with fitted gain ≈ 1.
  - Measured (per-second median SNR vs the reference, mono
    downmix): stereo 22.05 kHz **3.10 → 27.28 dB** (corr²
    0.153 → 0.994), 44.1 kHz 96 kbps **−0.05 → 18.23 dB**
    (0.082 → 0.995), 44.1 kHz VBR **1.60 → 20.87 dB**
    (0.129 → 0.999), mono 8 kHz 3.89 → 4.73 dB (LSP envelope still
    flat — its conversion tables are the remaining staged gap on
    that family).

### Changed

- **Reservoir / variable-block-length / stereo calibration pass
  (r446)** over the round-6 staged docs, driving the vendor-stream
  frame-layer closure from **1552/1763 to 1705/1763** §1 carry
  boundaries — five of the six committed families now close
  completely (mono 8 kHz 394/394, stereo 22.05 kHz 1098/1098,
  44.1 kHz 3/3 + 13/13 + 133/133); only the mono 22.05 kHz stream
  stays partial (64/122, the open F1 anomaly below):
  - **The §2 B2 envelope-reuse bit exists** — one bit **per block**
    (not per channel), on blocks shorter than the frame, on
    two-channel streams (`vendor_frame::ReuseRule`, default
    `TwoChannelShortBlock`). This revises the r439 "no B2 bit exists
    on the wire" calibration, which had only measured the two
    unconditional readings; the staged §2 row's own condition ("more
    than one block size in this frame") is exactly the short-block
    condition. B2 = 0 skips every coded channel's envelope in favour
    of the §3 **per-block-size envelope cache**, which `vendor_decode
    ::BlockSynth` now carries per channel (`Envelope::Reused`
    resolves from it; flat only right after a reset). The committed
    corpus cannot separate `channels == 2` from `n_block_sizes ≥ 8`
    as the true gate (docs ask); repeat-scoped presence readings
    measure marginally worse; the mono 22.05 kHz stream rejects the
    bit outright.
  - **A zero §1 carry marks the previous packet as padded**: its
    declared frames all completed inside it and the remaining body
    bits are padding, not frame data (`packet` module docs). The
    VBR-configured 44.1 kHz streams pad most packets this way; the
    measurement harness and the PCM leg now treat the padding skip
    as a clean resynchronisation (no overlap-add reset).
  - The §2.1 noise-substitution hypothesis carrier (`NoiseSpec`) can
    now walk **either staged grid** — the exponent-band partition or
    the octave subband table (`tables/subband-freqs`, the staged
    noise/hgain seed) — via the new `NoiseGrid` selector. No
    start-band/grid hypothesis improves any committed stream,
    consistent with noise coding being disabled in all six.
  - **Open (docs ask):** the mono 22.05 kHz stream's F1 block-size
    fields around 512-sample transitions contradict both the
    one-ahead pipeline and the field-is-current readings (pinned by
    a boundary-constrained exhaustive re-parse of its packets);
    needs the block-size latch state flow for the
    small-`n_block_sizes` mono case.
  - `tests/vendor_streams.rs` regression floors raised to the new
    closure rates (global floor 1700/1763); the black-box PCM
    alignment now searches block-aligned lags only. The
    `vendor_parse` fuzz target exercises the new `ReuseRule` /
    `NoiseGrid` switches (bounded re-runs of the vendor-path
    targets: zero findings).

### Added

- **Vendor-bitstream decode arc (r439)** — the crate now parses and
  decodes genuine vendor-encoded WMA v2 streams end-to-end, measured
  against the six committed vendor bitstreams under
  `docs/audio/wma/reference/vendor-streams/`:
  - `wire_codes` + `wire_vlc` — the exact vendor codeword assignment
    for all **eight** staged VLC trees (class 1/2/3 primary + alt
    coefficient trees at their full alphabets — class-2 primary 1336
    symbols, class-2 alt 1072 — plus scale 121 and gain 37), and all
    six 2-based `(run, |level|)` companion maps including the three
    alt variants. A cross-check pins that **no staged table matches
    the canonical reconstruction**, so the explicit codes are
    load-bearing; the earlier canonical realisations in
    `coef_vlc`/`envelope_vlc` remain for the self-consistent loop.
  - `stream_config` — the complete §0 open-time derivation (flags2
    map, the reservoir∧VBL gate, `n_block_sizes`, frame-length tree
    with low-bitrate doubling, `byte_offset_bits`, `w_bs`, the §0.2
    class decision table with staged branch directions, §0.3
    coefficient ranges), pinned against every row of the staged
    vendor-stream measurement.
  - `packet` — the §1 superframe header (sequence / frame count /
    reservoir carry) and a `PacketAssembler` that validates packets,
    tracks sequence continuity, and concatenates bodies into one
    contiguous bit stream with per-packet carry-boundary records.
  - `band_partition` — the eight staged exponent-band partitions
    with the (sample-rate arm × block size) selector and the
    computed critical-band walk for everything else (25 bands for a
    2048-coefficient block at 44.1 kHz, per the staged cross-check).
  - `vendor_frame` — the §2–§4 frame/block parser: F1 with the
    three-field opening, F2a/F2, total-gain chaining and its
    escape-width map, the §3 exponent-delta envelope over per-size
    partitions, the §3.1 line-spectral index carriage, §4
    coefficient decode (escape = symbol 0, EOB = symbol 1, trailing
    signs), and an off-by-default §2.1 noise-substitution hypothesis
    carrier (its enable rule is still open in the staged docs).
  - `vendor_decode` — the §5 sum/difference (mid/side) inverse in
    its staged position plus staged-ladder dequantisation and the
    inverse-MLT / sine-window / overlap-add synthesis to PCM (the
    dequantisation composition rule and transition-window shape are
    open staged items; documented as approximations).
  - `tests/vendor_streams.rs` — the measurement harness over the six
    committed vendor streams (fixtures referenced from the docs
    staging area, skip-if-absent; ASF unwrapped by a black-box
    validator invocation): §1 holds on **all 1769 packets** (sequence
    continuity, carry bounds, frame counts), the frame layer closes
    **1552 of 1763** carry boundaries (mono 8 kHz 394/394, stereo
    22.05 kHz 1086/1098, mono 22.05 kHz 64/122, 44.1 kHz high-rate
    family still partial), and the PCM leg correlates against a
    black-box reference decode (corr² 0.96 on the 44.1 kHz 64 kbps
    stereo stream; ~3.7–3.9 dB per-second median on the closed
    mono/stereo families under the open dequantisation items).

### Changed

- **Vendor-measured §2/§5 calibrations (r439)** — three details of
  the staged frame-layout reading calibrate differently against the
  vendor bitstreams (each is being reported back to the docs
  staging as an erratum/extension ask): the F1 block-size field is a
  **one-ahead pipeline** (the per-block field carries the *next*
  block's size; the three-field opening re-primes previous /
  current / next, and the latch applies to the first frame starting
  in a packet); **no B2 envelope-reuse bit exists on the wire**; and
  the joint-stereo flag's ALT-tree consequence is **channel-scoped**
  (second coded channel only — the difference channel).

- **Fast MLT (r433)** — `mlt::Mlt::{forward, inverse}` now run an
  `O(M log M)` FFT factorization of the oddly-stacked TDAC basis
  (pre-twiddle → one `2M`-point radix-2 complex FFT → post-twiddle +
  real part, derived from `cos θ = Re e^{-iθ}`; general public DSP
  algebra, the trace's `[DSP]` tier). The direct `O(M·2M)` summation
  survives in-module as the test oracle: the fast path is pinned
  against it coefficient-for-coefficient at S256/S512/S1024 and
  spot-wise at S2048/S4096, the TDAC alias identities and full-chain
  perfect reconstruction now also run at the large sizes, and the
  crate test suite drops from ~10.5 s to ~0.2 s wall.
- `bitio`'s module docs no longer describe the MSB-first packing
  order as a swap-point realization detail: the staged frame-layout
  trace pins the vendor get-bits mechanism
  (`out = (acc >> shift) & MASK[n]`, fields MSB-first), so the
  module's order is the staged wire fact. Stale "not specified by any
  staged document" trailers in `header` / `setup` docs now point at
  the modules where the staging actually landed.

### Added

- **Staged-data + hardening pass (r433)**:
  - `wire_tables::BITREADER_MASK_LUT` — the last unconsumed staged
    table (`docs/audio/wma/tables/wma-bitreader-mask-lut.csv`, 32 ×
    `u32`, `(1 << n) - 1`), carried verbatim with the staged
    validation line pinned; a new `bitio` cross-check test ties
    `BitReader::read_bits` to the staged mask law.
  - `header::WmaHeader::variable_block_length_field` — typed carriage
    of the wiki-located variable-block-length configuration field
    (the upper 13 bits of `flags2`, present only when `flags2` bit 2
    is set); the block-size-determination logic it feeds is elided by
    the snapshot's own ellipses and stays a documented DOCS-GAP.
  - `wire_chain` decode-path hardening sweeps at real 8 kHz S512
    geometry with the staged class-3 table: every strict bit prefix
    of a valid frame fails with a typed error (self-delimiting rule),
    single-bit corruption never panics the frame parser, and the
    packet entry point survives arbitrary byte streams across
    several runtime frame counts.
  - `fuzz/` sub-crate with four self-contained libFuzzer targets:
    `header_parse` (both versions + open-time derivation invariants),
    `wire_decode` (arbitrary bytes through the frame/packet parsers
    at two real geometries), `wire_roundtrip` (sanitized valid frame
    → encode → decode field-exact, final-bit truncation must fail),
    and `coef_vlc_roundtrip` (symbol streams over all five staged
    tables encode/decode bit-exact; `expand` total where the
    companion map is staged, typed error on the alt variants whose
    maps are the documented gap). Initial bounded runs: 60 s per
    target, 0.9M–48M execs, zero findings.

- Internal public surface marked `#[doc(hidden)]` (44 rebuild-plumbing
  modules plus their crate-root re-exports) so cargo-semver-checks
  scores only the documented stable API (`header::{Version, WmaHeader}`,
  `block::BlockSize`, crate-root `Error` / `Result`); attributes and
  comments only, no path, signature, or semantic changes.

### Added

- **Class-selector pass (r405)** over the newly staged
  decode-class threshold extraction
  (`docs/audio/wma/tables/wma-class-selector-thresholds.csv` + the
  updated `docs/audio/wma/provenance/02-extractor-univdreams-tables.md`):
  - `wire_tables` — the four class-selector constants verbatim
    (`CLASS_SELECTOR_RATE_FLOAT_LOWER_BOUND` = 0.125,
    `CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD` = 0.72,
    `CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD` = 1.16,
    `CLASS_SELECTOR_RATE_FLOAT_UPPER_BOUND` = 1.6, plus the
    storage-order array `CLASS_SELECTOR_THRESHOLDS`), pinned
    bit-exact (`f32::to_bits`) against the staged CSV's
    shortest-round-trip renderings. Documented residual gaps: the
    branch *directions* (which side of each threshold selects which
    class) and the init formula of the per-stream float the
    thresholds are compared against.
  - `coef_vlc` — `CoefDecodeMode::from_class_and_variant`, the staged
    six-descriptor registration crossing (decode class `1..=3` × alt
    flag → registered coefficient table), with the located-but-unstaged
    class-2 alt slot as its only documented hole; round-trips
    `class()`/`is_alt()` over all five staged descriptors.
  - `wire_chain` — the staged thresholds wired into the §4b decode-class
    rule: `clamp_rate_float` (saturation into the staged `[0.125, 1.6]`
    axis), `RateFloatRegion` + `rate_float_region` (the typed
    three-region partition by the 0.72 / 1.16 branch thresholds, named
    after the thresholds — never after a class outcome, since the
    branch directions are unstaged), `select_decode_class` now takes
    the per-stream rate float and its `BitrateGated` arm carries the
    resolved region (the previous `candidates` field over-asserted a
    two-way class-1/2 choice; §4b's prose leaves the class-3 default
    retainable, so the arm now carries exactly what is staged), and
    `WireFrameCodec::from_header_pinned_class` builds the codec
    wherever the rule pins the class (below the 32 kHz gate → class-3
    primary) and refuses with the new typed
    `WireChainError::ClassNotPinned { sample_rate, region }` above it.
    `select_decode_class`'s signature is now
    `(sample_rate: u32, rate_float: f32)` — the second argument is the
    per-stream bitrate/quality scalar of the staged threshold
    comparison (ignored below the 32 kHz gate, where the class pins
    before the comparison is reached); its init formula is a
    documented gap, so callers thread in an observed value.
- **Wire-decode pass (r390)** over the newly staged frame-layout
  trace (`docs/audio/wma/frame-bit-layout.md`, docs `c1c68cd`) and
  the corrected mode-2 reading (docs `f319744`):
  - `wire_tables` — the corrected mode-2 sentinels (`COEF_EOB_SYMBOL`
    = 0, `COEF_ESCAPE_SYMBOL` = 1, 2-based run-level indexing; the
    "8 missing escape codewords" premise was overturned — the Kraft
    deficit is decode-DAG replication room, no codeword is missing)
    plus the newly staged tables verbatim: `SCALE_VLC_LENGTHS` (121,
    Kraft = 1), `GAIN_VLC_LENGTHS` (37, Kraft = 1),
    `COEF_VLC_CLASS1_ALT_LENGTHS` (555) and
    `COEF_VLC_CLASS3_ALT_LENGTHS` (435).
  - `runlevel_tables` — the symbol → `(run, |level|)` companion maps
    for decode classes 1/2/3 (664 / 1333 / 474 pairs), ramp-grouping
    law and the provenance §4e worked examples pinned.
  - `coef_vlc` — mode 2 now constructible (via the new
    `HuffmanCode::from_lengths_prefix`, accepting a
    documented-incomplete prefix code whose unassigned space decodes
    to a clean error); `Class1Alt`/`Class3Alt` variants; typed
    `CoefEvent` expansion (`EndOfBlock` / `Escape` /
    `Pair{run, abs_level}`) and its encoder-side inverse
    `symbol_for_pair`.
  - `envelope_vlc` — the scale (121) and gain (37) delta VLCs with
    CSV-pinned codewords and symmetric-alphabet delta accessors (the
    scale center 60 is pinned by the staged data's own 1-bit
    codeword).
  - `frame_bits` — the staged bit-packing layout realised: S1/S2/S3
    frame header, the B1..B6 per-block field order (7-bit header,
    gain sub-stream, 2-channel stereo flag, 5-bit envelope base,
    scale sub-stream, coefficients), one trailing sign bit per
    non-zero coefficient, the corrected escape (symbol 1 + literal
    run/level at runtime-signalled widths), self-delimiting
    coefficient counts with EOB for trailing zeros. Byte-exact layout
    pins, an exhaustive 2,152-pair all-alphabet wire sweep, escape
    boundary sweeps, and a 200-stream no-panic fuzz pass.
  - `wire_chain` — `select_decode_class` (staged §4b rule: class 3
    pinned below 32 kHz, typed class-1/2 bitrate-gated choice above)
    and `WireFrameCodec` (header-derived S1/escape widths per the
    staged formulas and §4e source pins, frame encode/decode to
    bytes). Milestone tests: mono (mode 2) and stereo (mode 1)
    PCM → §8 chain → real-VLC frame bits → bytes → parse → §8 chain
    → PCM within the quantizer bound.
- **Wire-level data pass** over the newly staged
  `docs/audio/wma/tables/` extraction (numeric tables read as bytes
  from the vendor WMA Standard decoder module's own PE sections, with
  per-table `.meta` provenance):
  - `wire_tables` — the staged tables verbatim: coefficient run-level
    VLC code lengths for decode modes 1 (666 symbols, Kraft = 1),
    2 (1016 real symbols; the escape codeword enumeration is the
    extraction's documented residual — the unassigned code space is
    pinned exactly as `16502/2^22`), and 3 (476 symbols, Kraft = 1);
    the 25-edge critical-band Hz partition seed; the 11-edge octave
    subband seed; and the 113-step `10^(1/16)` (1.25 dB/step)
    dequantization gain ladder. Invariant tests pin Kraft sums in
    exact integer arithmetic, monotonicity, octave doubling, the
    ladder's closed-form tail, and per-row CSV spot values. The same
    extraction confirms **no LSP codebook exists** on this path.
  - `coef_vlc` — decode modes 1 and 3 realised as working canonical
    codes; constructed codewords match the staged CSVs bit-for-bit
    and full-alphabet symbol streams round-trip through the bit
    cursors. Mode 2 construction is a typed docs-gap
    (`Mode2EscapeEnumerationUnstaged`).
  - `exponent_bands` — the per-block exponent/quantization-band and
    noise-grid partitions derived from the Hz seeds exactly as the
    vendor decoder derives them (scale to coefficient bins, clamp at
    Nyquist, collapse), directly into `QuantBandLayout`. Rounding tie
    behaviour is the one documented realization detail.
  - `gain_ladder` — ladder lookups, the scale-free `gain_ratio`
    (16 steps = one decade, pinned), and `band_weights` mapping
    per-band exponent indices to the §4 `Q[d]` vector the
    `DequantStage`/`QuantStage` pair consumes.
  - `wire_chain` — `WireBlockConfig::from_header` derives block size
    + both real partitions from a parsed `WmaHeader` and assembles
    the §8 encoder/decoder chains over that real geometry; PCM round
    trips over the 44.1 kHz/S2048 25-band staged partition are pinned
    within the §4 bound, shrinking with the step. Stereo constructors
    (`stereo_encoder`/`stereo_decoder`, per-channel exponent
    profiles, both channel modes round-tripped) and
    `channel_decoder_with_noise_grid` (noise filler over the staged
    octave grid; all-coded plans decode identically to the exponent
    grid) complete the assembler.
- `masking` module — the §4 encoder-side **Bark-scale masking model**
  (US6,240,380 FIGS.13–14, box 1318: "the weighting function follows
  an auditory masking curve computed on the Bark scale, with a
  simplified asymmetric spreading function (−25 dB/Bark left,
  +10 dB/Bark right) and an optional partial-whitening exponent β").
  `bark_from_hz` realises the Bark mapping via the standard public
  psychoacoustic formula (`[DSP]` tier — the patent pins that the
  curve lives on the Bark scale, the scale itself is textbook);
  `bin_frequency` gives the MLT bin centres `(k + ½)·sr/2M` that tile
  0..Nyquist (the wiki's `high frequency = sr/2` ceiling);
  `SpreadingSlopes` carries the patent-pinned `PATENT` pair (25
  dB/Bark toward lower frequencies, 10 toward higher — the disclosed
  asymmetry, masking spreading farther upward) and `spread_masking`
  combines every masker's triangular fall-off by per-position maximum;
  `partial_whitening` / `_in_place` apply the optional exponent β
  (caller-supplied encoder tuning, never fabricated) with the β = 1
  identity and β = 0 flat endpoints and zero-stays-zero. Encoder
  analysis only — it shapes the §4 weighting matrix, is carried by
  `matrix_coding`, and touches no bitstream field. 15 unit tests cover
  Bark monotonicity + conventional landmarks, bin-frequency
  tiling/spacing + zero-M panic, the patent slope values, the
  single-masker asymmetric triangle, max-combination across maskers,
  empty/mismatch handling, both whitening endpoints,
  dynamic-range compression at β = ½, in-place ↔ fresh equivalence,
  the negative-β / negative-weight panics, and an end-to-end
  bins → barks → spread → whiten weighting-pipeline shape check.
  Crate test count: 693 → 708. Re-export: `SpreadingSlopes`.

- `matrix_coding` module — the §4 FIG.1 **quantization-matrix
  side-information chain assembled down to bits**, the most directly
  bitstream-relevant disclosure in the trace ("the encoder transmits
  [the matrices] as side information in the bitstream", US7,930,171):
  `MatrixCoder` runs the direct-compression technique end-to-end —
  step 110 **uniform quantize** each element (`quant::quantize_sample`
  at unit weight against a caller step), step 120 **differentially
  code** relative to preceding elements (`qmatrix::differential_encode`
  / `differential_decode`, seed explicit), step 130 **Huffman-code**
  the deltas (US7,930,171 steps 110/120/130; US7,502,743) over a
  caller-supplied contiguous bounded delta alphabet emitted through
  `bitio`. The real "scale Huffman table (121 entries)" contents are
  `[GAP]` per the trace, so range and weights are parameters —
  self-consistent, not wire-compatible — and a delta outside the
  alphabet rejects (`DeltaOutOfRange`; no escape convention is
  fabricated). `compress_matrix` returns the quantized elements so the
  encoder can mirror the decoder's reconstruction (the §4
  side-information contract), and `decompress_matrix` reconstructs
  each element to exactly `q * step` (within half a step of the
  original). 8 unit tests cover alphabet validation (empty, i32
  overflow, accessors), the exact steps-120+130 round trip, the
  out-of-alphabet reject, the US7,502,743 zero-delta-padding
  efficiency detail (padded mask codes strictly fewer bits than the
  raw swings), the full-chain half-step reconstruction bound, the
  exact quantized-grid decoder property, the truncated-stream error
  path, and error `Display`/`source`. Crate test count: 685 → 693.
  Re-exports: `MatrixCoder`, `MatrixCodeError`.

- `paircode` module — the §6 entropy back end assembled **end-to-end
  to bits**: `RunLevelCoder` runs the patent's FIG.6 construction on a
  caller-supplied `CodebookGrid` — the codeword alphabet is every
  in-codebook pairing (row-major, run-outer) plus one trailing escape
  symbol weighted by the residual probability mass `max(0, 1 - Σ)`
  (what the escape codeword stands for: everything the threshold
  excluded) — builds the joint `(R, L)` canonical Huffman code from
  the grid's own probabilities (US6,223,162 grid 500 / threshold 518;
  US7,885,819 joint 2-D `(R, L)` Huffman), and codes pairs over the
  `bitio` cursors: in-codebook pairs as single codewords, escapes as
  the escape codeword followed by fixed-width `R` / `L` literals
  (US6,223,162 Claim 4; the Claims-5/6 decoder side recovers them).
  The literal widths are the §6 `[GAP]` ("the bit widths are not
  patent-disclosed"), so they are a typed caller-supplied
  `EscapeWidths` (validated `1..=32` per field) — never fabricated —
  with `PairCodeError::EscapeOverflow` rejecting values that do not
  fit at encode time and `InvalidEscapeLiteral` rejecting a decoded
  `run == 0` / `level == 0` trailer as stream corruption. Grids and
  probabilities stay caller-supplied: a coder built here is
  self-consistent, not wire-compatible, per the `huffman` posture.
  12 unit tests cover `EscapeWidths` validation/bounds, alphabet
  construction, the probable-pair-codes-no-longer property,
  in-codebook and escape round trips (below-threshold and
  outside-rectangle — the patent's "≥ Rm" tail), escape overflow on
  both fields, the corrupt-literal and truncated-stream error paths,
  a mixed 8-pair stream round trip, and the crate's first full §6
  chain across the bit level: sparse tail → `runlevel::compress` →
  pair-coded bits → `decode_pair` → `expand_into` reproduces the tail
  exactly. Crate test count: 673 → 685. Re-exports: `RunLevelCoder`,
  `EscapeWidths`, `PairCodeError`.

- `bitio` + `huffman` modules — the entropy stage's **bit-level
  machinery**. `bitio` is the format-neutral `[DSP]`-tier prefix-code
  plumbing the §6/§8 VLC stages run on: an MSB-first append-only
  `BitWriter` (`write_bit` / `write_bits` / `align_to_byte` /
  bit-precise `bit_len`) and its exact-inverse `BitReader` cursor
  (`with_bit_len` excludes final-byte padding; failed reads consume
  nothing; `BitstreamEnd` reports requested vs remaining). The
  shipping WMA v1/v2 byte/bit packing order is `[GAP]` per the trace,
  so the MSB-first convention is documented as a realization detail of
  this crate's self-consistent coder with a single swap point — not a
  wire-format claim. `huffman` implements the §6 patent-disclosed
  code-book construction *method* (US6,223,162 grid 500 / threshold
  518 / Claims 8–10: "pairings above a probability threshold get
  Huffman codewords"; US7,885,819 joint 2-D `(R, L)` Huffman;
  US7,930,171 step 130 Huffman over matrix deltas), realised via the
  general public Huffman/canonical-code algorithms (`[DSP]` tier):
  `HuffmanCode::from_weights` merges caller-supplied non-negative
  weights (zero weights legal — the patent's threshold can sit at 0.0
  — deterministic tie-breaking, single-symbol degenerate 1-bit code)
  into an optimal prefix code assigned canonically;
  `HuffmanCode::from_lengths` rebuilds the canonical code from
  explicit per-symbol lengths — the plug-in point for staged real
  tables — validating the Kraft **equality**; `encode_symbol` /
  `decode_symbol` code over the bit cursors with an `O(max_len)`
  canonical range decode (per-length first/count/offset tables built
  once). Codes built here are self-consistent, **not**
  wire-compatible: the literal v1/v2 tables stay `[GAP]`. 33 unit
  tests cover the writer (MSB-first fill, cross-byte fields, 64-bit
  width, alignment padding, overwide panic), the reader (inverse
  semantics, no-consumption-on-failure, bit-precise lengths,
  alignment), a 200-field mixed-width write→read round trip, code
  construction (reject paths, dyadic-weight exact lengths, monotone
  weight→length shape, prefix-freeness, Kraft equality, canonical
  (length, symbol) order, incomplete/overfull length rejects),
  bit-level round trips (weighted alphabet, 500-symbol stream), the
  truncated-stream and out-of-range error paths, the
  compression-beats-fixed-width property, and error
  `Display`/`source`. Crate test count: 640 → 673. Re-exports:
  `BitWriter`, `BitReader`, `BitstreamEnd`, `HuffmanCode`,
  `HuffmanError`.

- `frame_encode` module — the §2 **frame-loop encoder drivers**, the
  forward mirror of `frame` (US7,930,171 FIG.3 / US7,383,180 module
  520: "partitions a frame of audio samples into overlapping sub-frame
  blocks"; wiki blocks → frames → superframes nesting). `FrameEncoder`
  wraps a `ChannelEncoder` (mono) and `StereoFrameEncoder` wraps a
  `StereoEncoder` (stereo); `encode_frame` partitions a frame's PCM
  into consecutive `M`-sample blocks and collects the per-block symbol
  sets — via the `into_block_params` bridges, exactly the
  `BlockParams` / `StereoBlockParams` lists `FrameDecoder` /
  `StereoFrameDecoder` consume. The stereo driver takes a
  caller-supplied per-block `ChannelMode` plan (`modes[t]` for block
  `t`; the §5 flag layout is `[GAP]`). The 50%-overlap frame buffer
  threads across frames — `encode_frame` does **not** flush, so a
  stream's frames encode contiguously (a test pins two frames ≡ one
  concatenated frame); `flush` emits the single trailing block at
  stream end and `reset` clears the buffers. Length contracts reject
  up front with nothing encoded: `InvalidFrameLen::{NotBlockAligned,
  ChannelLenMismatch, ModeCountMismatch}` under
  `FrameEncodeError` / `StereoFrameEncodeError`. Uniform-block-size
  frames only, matching `frame` (the variable-block-length plan from
  the upper `flags2` bits and the superframe byte layout stay `[GAP]`
  per §1/§2/§9). 12 unit tests cover accessors + empty frames, the
  unaligned / channel-mismatch / mode-count rejects with the
  no-advance guarantee, equality with the manual per-block loop, the
  cross-frame buffer persistence, per-block mode honouring against a
  hand-wired stereo mirror, whole-stream encode→decode round trips
  through `FrameDecoder` (mono) and `StereoFrameDecoder` (stereo,
  sum/difference) within the quantizer bound, reset-equals-fresh, and
  error `Display`/`source`. Crate test count: 628 → 640. Re-exports:
  `FrameEncoder`, `StereoFrameEncoder`, `InvalidFrameLen`,
  `FrameEncodeError`, `StereoFrameEncodeError`.

- `stereo_encode` module — the §8 patent-disclosed **full two-channel
  encoder-block chain**, the stereo analogue of `encode` and the
  forward mirror of `stereo_decode` (§8 encoder pipeline: `[optional
  multi-channel pre-process / sum-difference]` drawn *before* the
  per-channel partition/MLT; US7,930,171 / US7,502,743 sum/difference).
  `StereoEncoder` wires two complete `ChannelEncoder` chains behind the
  §5 forward fold (`stereo::forward_in_place`), applied **only** when
  the caller-supplied per-block `ChannelMode` is `SumDifference` (the
  flag's v1/v2 layout is `[GAP]`, so the typed mode travels with the
  emitted block) — under joint coding the two frame buffers carry the
  mid/side signals, exactly the signals the paired decoder's
  overlap-add carriers hold. Channel 0 encodes first so its error
  surfaces before channel 1's buffer advances (the mirror of the
  decoder's lock-step guarantee), and both input lengths are
  pre-checked before the fold so a length error never advances either
  buffer. `StereoEncodedBlock { ch0, ch1, mode }` feeds
  `StereoDecoder::block` argument-for-argument, with
  `into_stereo_block_params(band_count)` bridging to the frame
  drivers; `flush(mode)` closes both channels (an all-zero pair folds
  to an all-zero pair, so the flush samples are mode-independent);
  constructor reuses `StereoAssemblyError`, per-block failures surface
  as the new `StereoEncodeError { channel, source }`. 11 unit tests
  cover construction accept/reject, both per-channel length pre-checks
  with the no-advance guarantee, the adds-no-arithmetic equality with
  the hand-wired fold-plus-two-chains mirror (both modes), constant-
  mode encode→decode round trips against `StereoDecoder` (Independent
  + SumDifference, within the quantizer bound after the `M`-sample
  latency), the §5 energy-concentration rationale observable as the
  side channel quantizing away for a near-identical pair, flush mode
  carriage, reset-equals-fresh, `into_stereo_block_params` plumbing,
  and error `Display`/`source`. Also adds the
  `ChannelEncoder::block_size()` accessor mirroring
  `ChannelDecoder::block_size()`. Crate test count: 616 → 628.
  Re-exports: `StereoEncoder`, `StereoEncodedBlock`,
  `StereoEncodeError`.

- `encode` module — the §8 patent-disclosed **full single-channel
  encoder-block chain**, the forward mirror of `decode` (Thumpudi-180
  FIG.5 encoder pipeline: *window + forward MLT → uniform scalar
  quantize (matrix weight × overall step) → run-level entropy code*).
  `ChannelEncoder` wires the three encode stages this round landed —
  `analysis::Analysis`, `quant::QuantStage`,
  `spectral::SpectralEncode` — with the same coefficient-count
  cross-check `decode::ChannelDecoder::new` applies
  (`EncodeAssemblyError::CoeffCountMismatch` names the first
  disagreeing pair; per-stage failures surface via
  `EncodeError::{Analysis, Quant, Spectral}`).
  `ChannelEncoder::block` maps `M` fresh time-domain samples to a
  typed `EncodedBlock { levels, pairs }` — exactly the `(levels,
  pairs)` argument pair `ChannelDecoder::block` consumes —
  `ChannelEncoder::flush` closes the stream with the zero block that
  drains the paired decoder's overlap-add carry, and
  `EncodedBlock::into_block_params(band_count)` bridges to the `frame`
  drivers (empty ignored patterns; this chain literal-codes every band
  — §7 noise/truncation selection is an encoder rate decision left
  caller-side). The headline cross-module property is pinned by tests:
  an encoder/decoder pair built from **one parameter set** round-trips
  — decode(encode(PCM)) reproduces a pseudo-random signal after the
  chain's `M`-sample leading latency within a small multiple of the
  §4 quantizer step (S256 + S512), and the worst-case error strictly
  shrinks when the step is halved (the rate/quality dial the patents
  describe). 14 unit tests cover assembly accept / both mismatch
  rejects, the per-stage error paths (wrong sample count;
  below-structural-floor partition), the adds-no-arithmetic equality
  with the hand-wired three-stage chain, the two round-trip sizes, the
  step-halving monotonicity, a sparse-spectrum run-level-branch round
  trip at the `min_split_for` floor, flush ≡ zero-block encode,
  reset-equals-fresh, `EncodedBlock` plumbing, and error `Display` /
  `source`. Crate test count: 602 → 616. Re-exports: `ChannelEncoder`,
  `EncodedBlock`, `EncodeAssemblyError`, `EncodeError`, `EncodeStage`.

- `analysis` module — the §3 patent-disclosed **encoder-side
  time-domain analysis stage**, the stateful mirror of `synthesis`:
  frame formation (previous `M` samples ‖ fresh `M` samples, the 50%
  overlap the oddly-stacked TDAC bank is defined by) → analysis window
  `ha(n)` → forward MLT (US7,930,171 FIG.3 "partitions a frame of
  audio samples into overlapping sub-frame blocks"; US7,383,180
  partitioner 520 / frequency transformer 530; US6,029,126 /
  US6,240,380 2M windowing over M-length blocks). `Analysis::block`
  consumes `M` fresh time-domain samples and emits `M` spectral
  coefficients, buffering the block across calls — the encoder-side
  counterpart of the decoder's overlap-add carry; `Analysis::flush`
  closes the stream with one all-zero block so the last real block's
  samples enter their trailing frame (an `n`-block signal encodes to
  `n + 1` coefficient blocks), and `Analysis::reset` clears the buffer
  at a discontinuity. Constructor reuses `synthesis::MismatchedBlockSize`
  so a mirrored encoder/decoder pair fails identically; the length
  contract surfaces as the new `InvalidSampleLen`. The stage adds no
  arithmetic of its own (a test pins two-block equality with the
  hand-wired window→forward chain). Block-size *decisions* stay
  caller-side (§3 transient-switch form is `[GAP]`); the stage runs
  one uniform `BlockSize`. 11 unit tests cover construction accept /
  reject, the length contract with its no-mutation guarantee, input
  buffering, hand-wired-chain equality, flush (zero-block encode +
  buffer zeroing), reset-equals-fresh, every `BlockSize::ALL` member,
  error `Display`, and the headline cross-module property: the full
  Analysis → Synthesis chain reproduces a pseudo-random input exactly
  (1e-9) after the chain's `M`-sample leading latency, at S256 and
  S512. Crate test count: 591 → 602. Re-exports: `Analysis`,
  `InvalidSampleLen`.

- `runlevel::compress` + `spectral::SpectralEncode` — the §6 entropy
  stage run **forward**, the paired encoder side of `expand_into` /
  `SpectralDecode`. `compress` walks a sparse magnitude sequence once
  and emits one `(R, L)` pair per non-zero preceded by `R ≥ 1` zeros
  (US6,223,162 Claim 1 "a run of R first-value symbols and an adjacent
  symbol of value L" / Claim 2 "the first value is zero, and L is
  non-zero"), returning trailing zeros in a typed `Compressed` carrier
  rather than encoding them — the patent names two block-closing
  alternatives, and `Compressed::pairs_with_implicit_terminator`
  realises the implicit-`(N, 1)` branch the walker recognises. A
  non-zero with no preceding zero has run `0`, outside the patent's
  `{1..Rm}` set, and surfaces as `CompressError::NoPrecedingZero` —
  per the patent's own rationale that dense statistic is what the
  level mode exists for. `SpectralEncode` mirrors `SpectralDecode`
  accessor-for-accessor: `block(&[i32])` splits at the caller-supplied
  `Partition` boundary (the tuned rule is `[GAP]` per §6), copies the
  head verbatim (already signed), and compresses the tail
  (magnitudes only — a negative tail coefficient rejects with
  `NegativeTailCoefficient`, documenting the §6 sign gap).
  `SpectralEncode::min_split_for` computes the structural **floor**
  the `{1..Rm}` set imposes on the mode boundary (every tail non-zero
  needs a preceding zero; signed values stay in the head) — the
  level-mode rationale emerging as a hard constraint, explicitly not
  the shipping encoder's tuned choice. 26 unit tests cover the
  compress walk (isolated non-zeros, trailing zeros, all-zero/empty
  blocks, both reject paths, terminator-only-when-needed), the
  compress→expand round trip (hand shapes + pseudo-random sparse
  S256), the encode accessor mirror, all four encode happy paths,
  all three encode reject paths, the `min_split_for` floor cases and
  its encodability guarantee, and full `SpectralEncode`→
  `SpectralDecode` round trips (shape table + S256 with dense signed
  head). Crate test count: 565 → 591. Re-exports: `SpectralEncode`,
  `SpectralEncodeError`.

- `quant` module — the §4 patent-disclosed **encoder-side forward
  quantization step**, the paired forward of the decoder's `invquant` /
  `dequant` stages (US7,930,171 overall step-size description:
  each coefficient quantized by the product of its band's matrix weight
  and one block-wide step; US7,383,180 quantizer 560: "adaptive,
  uniform, scalar quantizer"). `quantize_sample(coeff, weight, step)`
  computes `round(coeff / (weight * step))`; `quantize_in_place` is the
  whole-block band-map form mirroring `invquant::dequantize_in_place`
  contract-for-contract (same panics); `QuantStage` mirrors
  `dequant::DequantStage` field-for-field — same `(block_size, layout,
  weights, step)` constructor triple, same validation
  (`InvalidQuant::{BlockSizeMismatch, WeightIndexOutOfRange,
  CoeffLenMismatch}` variant-for-variant with `InvalidDequant`), same
  once-folded `BandScale` divisor table — so an encoder/decoder pair
  built from one parameter set agrees by construction. Step-size
  *selection* stays a caller-supplied `OverallStepSize` (rate-control
  tuning per US7,343,291, not a bitstream rule); the rounding tie-rule
  (`f64::round`, half-away-from-zero) and the degenerate-divisor /
  saturation boundaries (zero divisor → silent 0; out-of-`i32`-range
  quotient → saturate) are documented realization details, not claimed
  WMA facts. 21 unit tests cover the rounding/dead-zone behaviour, the
  on-grid inverse-of-`dequantize_sample` identity, the uniform-quantizer
  `|error| ≤ divisor/2` bound (per-sample and whole-stage across every
  `BlockSize::ALL` member), the zero/non-finite/saturation boundaries,
  the whole-block helper and its three panic contracts, the stage's
  constructor accept/reject paths, stage↔helper agreement, the
  `QuantStage`↔`DequantStage` on-grid round trip, and error `Display` /
  `std::error::Error`. Crate test count: 544 → 565. Re-exports:
  `QuantStage`, `InvalidQuant`.

- `WmaHeader::long_block_size()` — the bridge from the parsed header to
  the typed transform-block size. The wiki's `frame_length = 1 <<
  frame_length_bits` rule fixes the long-block size in samples (512 /
  1024 / 2048 for `frame_length_bits ∈ {9, 10, 11}`), and every value
  the decision tree produces is a member of the patent-disclosed set
  `{256, 512, 1024, 2048, 4096}` (§2, US7,930,171), so this maps the
  header exponent onto `BlockSize` via `BlockSize::from_log2`. It is the
  connective tissue a caller uses to construct the per-block
  `decode::ChannelDecoder` / `stereo_decode::StereoDecoder` (and the
  `frame` drivers above them) at the header-determined size — for any
  header from `WmaHeader::parse` it is infallible (the tree only yields
  9/10/11), with the `Result` kept for hand-built headers and a future
  variable-block-length path. 2 unit tests pin the per-exponent mapping
  and that the typed size's sample count equals the header's
  `frame_length` field. Crate test count: 542 → 544.

- `frame` module — the §2 patent-disclosed **frame loop**, the
  block→frame grouping the patents and wiki both name (Chen-171 FIG.3 /
  Thumpudi-180 module 520: a frame is "partition[ed] into overlapping
  sub-frame blocks"; wiki: "blocks → frames (one or more blocks) →
  superframes"). This is the orchestration layer one above the per-block
  decoders: `FrameDecoder` wraps a `decode::ChannelDecoder` (mono) and
  `StereoFrameDecoder` wraps a `stereo_decode::StereoDecoder` (stereo).
  `FrameDecoder::decode_frame(&[BlockParams])` /
  `StereoFrameDecoder::decode_frame(&[StereoBlockParams])` run a frame's
  ordered list of already-demuxed per-block parameter sets through the
  underlying §8 chain and concatenate the per-block PCM into the frame's
  PCM (mono: a `Vec<f64>` of `n_blocks * M`; stereo: a `StereoBlock`
  whose `left`/`right` each hold `n_blocks * M`). `BlockParams { levels,
  pairs, patterns }` is the owned analogue of `ChannelDecoder::block`'s
  borrowed argument triple (the noise `patterns` are owned
  `Vec<Vec<f64>>` reborrowed as `&[&[f64]]` at decode time);
  `StereoBlockParams { ch0, ch1, mode }` pairs both channels' params
  with the per-block `ChannelMode` (the §5 independent-vs-sum/difference
  decision whose v1/v2 flag layout is `[GAP]`, so it is a caller input).
  The overlap-add carrier threads across frames — `decode_frame` does
  **not** flush, so a stream's frames decode contiguously; `flush`
  drains the trailing tail once at stream end, and `reset` clears the
  carry at a discontinuity. The stage adds no arithmetic of its own
  (tests pin block-for-block equality with the hand-run per-block chain,
  and that two `decode_frame` calls equal one call over the concatenated
  block list — the carry is not reset at the frame boundary). The driver
  runs a **uniform-block-size** frame (every block at the decoder's `M`,
  the non-variable-block-length case `frame_length = 1 <<
  frame_length_bits` describes); block-size-transition frames need
  window-transition handling whose shape is `[GAP]` per §2/§3 (the same
  deferral `decode` and `synthesis` record), and the DEMUX / superframe
  byte layout stay `[GAP]`, so the block count and per-block parameters
  are caller-supplied inputs, never fabricated. 17 unit tests cover the
  `BlockParams` / `StereoBlockParams` plumbing, the mono driver
  (block-len agreement, empty / single / multi-block frame lengths,
  equality with the manual per-block chain, the cross-frame carry
  persistence, reset-clears-carry, flush-drains-tail) and the stereo
  driver (empty-frame two-empty-channels, multi-block per-channel
  concatenation, equality with the manual stereo chain, the
  sum/difference-vs-independent fold honoured per block, reset/flush).
  Crate test count: 525 → 542. Re-exports: `FrameDecoder`,
  `StereoFrameDecoder`, `BlockParams`, `StereoBlockParams`.

- `setup` module — the wiki snapshot's **rate-dependent stream-setup
  parameters**, the deterministic scalars a WMA decoder computes once
  at stream-open time from the already-parsed `WmaHeader`
  (`docs/audio/wma/wiki/Windows_Media_Audio.wiki`, "init rate dependent
  parameters"). `SetupParams::from_header` derives four closed-form
  values with no fabrication: `high_frequency = sample_rate / 2` (the
  wiki's "high frequency = sample rate / 2"); `bits_per_sample =
  bit_rate / (channels * sample_rate)` (the wiki's "bits/sec = bitrate
  / (channels * sr)", the dimensionless per-sample-per-channel bit
  budget despite the wiki's "bits/sec" label); `byte_offset_bits =
  log2(bps * frame_length / 8) + 2` (the wiki's "byte offset bits =
  log2(bps * frame length / 8) + 2", with `log2` the integer floor
  logarithm matching the wiki's `frame length bits = log2(frame
  length)` usage); and `noise_coding`, initialised to the wiki's
  `use noise coding = 1 as a default`. The wiki separately names a
  noise-coding *activation* decision "based on channels and sr" but
  does not spell out its threshold rule, so that selection is a
  **DOCS-GAP**: the field ships the wiki default and is overridable via
  `SetupParams::with_noise_coding` (a caller that determined the
  activation by black-box observation threads it in rather than this
  module fabricating a threshold). Degenerate container fields clamp
  instead of panicking — a zero channel count yields
  `bits_per_sample = 0` (guarded `checked_div`), and a zero
  `bps * frame_length / 8` product yields `byte_offset_bits = 2`
  (`floor_log2(0)` defined as `0`). This is the first stage past the
  Round 1 header parser to consume the parsed header, bridging
  `WmaHeader` toward a future frame-decode driver; it introduces no
  codeword tables and no bitstream parsing. 16 unit tests cover the
  `floor_log2` helper (powers of two, floor-down on non-powers, the
  zero-is-zero clamp), `high_frequency` as Nyquist, `bits_per_sample`
  for stereo / per-channel / mono-vs-stereo-halving / zero-channel
  guard, the `byte_offset_bits` formula and its small-product /
  zero-product clamps, the `noise_coding` default and override
  (including the untouched-other-scalars and idempotence properties),
  an end-to-end derivation through the real `WmaHeader::parse`, and
  `Copy`/`Eq`. Crate test count: 509 → 525. Re-export: `SetupParams`.

- `stereo_decode` module — the §8 patent-disclosed **full two-channel
  decoder-block chain**, the stereo analogue of the `decode` module's
  single-channel `ChannelDecoder`. `StereoDecoder` wires **two** complete
  per-channel `ChannelDecoder` chains (each running entropy decode →
  inverse quantize/weight → noise-fill → inverse MLT → window →
  overlap-add) and closes the pipeline with the §8 FIG.6 `[inverse
  sum-difference]` multi-channel post-process (US7,502,743), folding the
  two reconstructed time-domain channels back to left/right PCM via
  `stereo::inverse_in_place` — but **only** when the caller-supplied
  per-block `ChannelMode` is `SumDifference` (bypassed for `Independent`,
  exactly as the FIG.6 box is). Whereas `stereo_synthesis::StereoSynthesis`
  begins at the inverse MLT and consumes already-dequantized coefficients,
  `StereoDecoder` begins one stage earlier at the entropy box, so it is
  the first assembler taking one stereo block's already-demuxed
  per-channel entropy symbols all the way to final L/R PCM. The fold runs
  after each channel's overlap-add (its FIG.6-fixed position), so the two
  per-channel overlap-add carriers stay independent across the block
  sequence; channel 0 is decoded first so its error surfaces before
  channel 1's carry advances. `StereoDecoder::new` cross-checks both
  channels share one `BlockSize` `M` (`StereoAssemblyError::BlockSizeMismatch`
  otherwise); `StereoDecoder::block` names the failing channel in
  `StereoDecodeError`; `flush`/`reset` delegate to both per-channel
  decoders. The channel-mode flag layout (§5) and the per-process DEMUX
  (§6) are `[GAP]`, so both are inputs, never fabricated; the stage adds
  no arithmetic of its own (tests pin equality with two hand-wired
  `ChannelDecoder` chains for both modes, plus a constant-signal
  sum/difference time-domain round-trip). Sourced from §8 (and §5) of the
  patent trace.
- `decode` module — the §8 patent-disclosed **full single-channel
  decoder-block chain**, the FIG.6 decoder path *entropy decode →
  inverse quantize/weight → fill noise-substituted bands (module 240) →
  inverse MLT → window → overlap-add* (Thumpudi-180 FIG.6). `ChannelDecoder`
  wires the four decode stages already landed (`spectral::SpectralDecode`,
  `dequant::DequantStage`, `noisefill::NoiseFiller`,
  `synthesis::Synthesis`) into one stateful per-channel decoder.
  `ChannelDecoder::new` cross-checks that all four stages agree on one
  coefficient count `M` (the disagreeing pair is named in
  `AssemblyError::CoeffCountMismatch`); `ChannelDecoder::block(levels,
  pairs, patterns)` runs them in patent order. Its load-bearing addition
  over the existing pairwise chains is inserting the noise-fill step in
  its FIG.6-fixed position — between the inverse quantizer and the inverse
  MLT (US7,383,180 module 240), exactly where both `dequant` and
  `synthesis` explicitly deferred it. The stage carries the overlap-add
  tail across calls (`ChannelDecoder::flush` drains it,
  `ChannelDecoder::reset` clears it at a discontinuity) and adds no
  arithmetic of its own (a test pins block-for-block equality with the
  hand-wired four-stage chain; another pins that the noise-fill step
  genuinely changes the band vs. a chain that skips it). The codeword
  tables and per-process DEMUX (US7,885,819 FIG.7) are `[GAP]`, so the
  chain consumes already-demuxed, already-decoded per-stage parameters.
  Errors propagate per stage via `DecodeError::{Spectral, Dequant,
  NoiseFill, Synthesis}`. Sourced from §8 of the patent trace.
- `spectral` module — the §6 patent-disclosed entropy-stage
  **spectral-coefficient assembler**, the FIG.6 decoder step *entropy
  decode (run-level → coefficients)* that sits immediately upstream of
  the §4 inverse quantizer (US6,223,162 mode selector 400 / FIG.5–6;
  US7,383,180 entropy encoder 570; §8 decoder pipeline). `SpectralDecode`
  wraps a decoded `entropy_mode::Partition`; `SpectralDecode::block(
  levels, pairs)` copies the `split` level-mode head symbols verbatim
  into `0..split` (US6,223,162 level mode, low-frequency mostly-non-zero
  range) and expands the run-level `(R, L)` `pairs` over the
  `split..total` tail window via `runlevel::expand_into` (US6,223,162
  run-level mode, high-frequency mostly-zero range), honouring the
  implicit `(N, 1)` terminator **measured against the tail's own
  remaining-coefficient count**, not the block's. The output is the
  `M`-coefficient `i32` vector `dequant::DequantStage::block` consumes, so
  the two assemblers chain into the FIG.6 decoder front-half *entropy
  decode → inverse quantize/weight* (a test runs an assembled block
  straight into `DequantStage`). The stage adds no arithmetic of its own:
  the codeword tables and bit reader are `[GAP]` per §6, so it consumes
  **already-decoded symbols** exactly as `runlevel::expand_into` does;
  escape recovery (`escape::EscapeLiteral::as_run_level_pair`) and the
  partition decision happen upstream. Sign placement is `[GAP]` per §6 —
  the level-mode head carries already-signed `i32` levels, the run-level
  tail non-negative magnitudes. Errors: `SpectralError::LevelLenMismatch`
  (head symbol count ≠ `split`), `SpectralError::RunLevelWalk` (wraps
  `runlevel::WalkError`), `SpectralError::LevelOverflow` (a magnitude
  above `i32::MAX`). Sourced from §6 of the patent trace.
- `stereo_synthesis` module — the §8 patent-disclosed decoder-side
  **stereo** time-domain reconstruction tail, the last stage of the
  FIG.6 decoder pipeline (Thumpudi-180 decoder FIG.6: `... → overlap-add
  → [inverse sum-difference / multi-channel post-process] → PCM`;
  US7,502,743 sum/difference). `StereoSynthesis::new(block_size,
  window_pair)` builds two independent per-channel `synthesis::Synthesis`
  stages (both channels of a stereo block share one window/block-size
  decision per the §2 tile note); `StereoSynthesis::block(ch0, ch1,
  mode)` reconstructs each channel through its own `Synthesis` (inverse
  MLT → synthesis window → overlap-add) and then applies the §5 inverse
  sum/difference fold (`stereo::inverse_in_place`) **only** when the
  per-block `channel_decision::ChannelMode` is `SumDifference`, returning
  the final left/right PCM as a `StereoBlock { left, right }`; for
  `Independent` the post-process is bypassed exactly as FIG.6 bypasses
  the box. The fold runs *after* the per-channel overlap-add — the FIG.6
  position — so each channel's overlap-add carry advances every call
  regardless of mode and always sees the per-channel (mid/side or
  left/right) signal, never the folded output. `flush(mode)` drains both
  trailing-edge tails (folding them when the trailing block was joint)
  and `reset()` clears both carries at a discontinuity; `tails()`
  exposes the two per-channel carries for inspection. The stage adds no
  arithmetic of its own — it is the stereo analogue of the
  single-channel `synthesis::Synthesis` assembler, sequencing existing
  primitives in the patent-fixed order. The v1/v2 channel-mode flag
  layout is `[GAP]` per §5, so `mode` is a caller-supplied input, never
  fabricated. Length errors from either channel surface via the existing
  `synthesis::InvalidCoeffLen`; a mismatched window pair via
  `synthesis::MismatchedBlockSize`. Sourced from §8 (and §5) of the
  patent trace.
- `noisefill` module — the §7 patent-disclosed decoder-side
  noise-substitution fill, the noise generator the `bands` module
  (Round 6) explicitly deferred (US7,383,180 / US7,343,291: "it
  signals that a band should be filled with a generated noise pattern
  of the appropriate energy"; decoder noise generator 240). Implements
  the one quantitative property the patent fixes — the energy contract
  — and leaves the generator's construction (spectral colour / PRNG /
  seed) as a caller-supplied `[GAP]` pattern. `pattern_energy(&[f64])`
  reuses `excitation::band_raw_energy` so the patent's squared-sum
  energy convention is pinned in one place; `noise_scale(target,
  pattern_energy)` derives the rescaling gain `sqrt(target /
  pattern_energy)` (band energy is a sum of squares, so it scales as
  the square of a uniform gain), returning a silent `0.0` for a
  non-positive target or an all-zero pattern rather than a `NaN` /
  `±∞`. `fill_band(target, &[f64])` / `fill_band_in_place(target, &mut
  [f64])` apply the gain, producing a band at the transmitted energy
  while preserving the pattern shape. `NoiseFiller { plan, layout }`
  pairs a `bands::BandPlan` with the matching `qband::QuantBandLayout`
  and `fill(&mut [f64], &[&[f64]])` walks a coefficient block in band
  order: `BandPolicy::Coded` bands are left untouched, `NoiseSubstituted`
  bands are filled from the per-band pattern rescaled to the band
  energy, and `Truncated` bands are zeroed (the patent's high-band
  elimination). Plan/layout band-count agreement, coefficient-block
  length, and each noise pattern's length are validated up front so a
  rejection leaves the block unmodified (no partial fill); failures
  surface via `InvalidNoiseFill { BandCountMismatch, CoeffLenMismatch,
  PatternLenMismatch }`. Per the §8 decoder diagram the fill sits after
  inverse-quantize/inverse-weight (`dequant`) and before the inverse
  MLT (`synthesis`), so the output is exactly the block
  `synthesis::Synthesis::block` consumes. The per-band flag encoding
  (decoded upstream into the `BandPlan`) and the generator construction
  both stay `[GAP]`. Re-exports: `NoiseFiller`, `InvalidNoiseFill`.
  27 unit tests cover the squared-sum energy convention and its
  agreement with `excitation::band_raw_energy`, the `noise_scale`
  sqrt-ratio formula and its zero-target / zero-pattern silent
  boundaries, `fill_band` reaching the target energy / preserving
  shape / unit-gain identity / empty-slice no-op / in-place ↔ fresh-Vec
  equivalence, the `NoiseFiller` constructor accept and band-count
  reject paths, accessor coverage, the coded-untouched /
  noise-rescaled / truncated-zeroed dispositions both individually and
  in one mixed block, all three `fill` reject paths with the
  no-mutation guarantee, a zero-energy noise band silencing, a full
  single-noise-band block for every `BlockSize::ALL` member, filler
  reuse across blocks, and `InvalidNoiseFill` `Display` / `std::error::Error`.
  Crate test count: 427 → 454.

- `channel_decision` module — the §5 patent-disclosed open-loop stereo
  (channel-coding) decision (US7,502,743: "the decision to code
  channels independently vs. jointly is an open-loop decision based on
  inter-channel energy separation and the disparity of excitation
  patterns"). `ChannelMode { Independent, SumDifference }` is the typed
  selector. `inter_channel_energy_separation(left, right)` computes the
  side-channel energy fraction `E_side / (E_mid + E_side)` from the
  `stereo` mid/side energies (`0.0` for `L == R`, `1.0` for `L == -R`,
  `0.5` for an independent equal-power pair, amplitude-scale-invariant).
  `excitation_pattern_disparity(left, right, &layout, exponent)`
  measures the normalised `L1` distance between the channels' §4
  excitation *shapes* in `[0.0, 1.0]` (`0.0` for identical shape
  including same-shape/different-loudness; `1.0` for disjoint band
  energy). `OpenLoopDecision { max_energy_separation,
  max_excitation_disparity }` holds the two `[GAP]` tuning thresholds
  (caller-supplied, never fabricated) and combines them per the
  patent's rationale — joint coding iff both criteria are favourable;
  `decide` takes pre-computed quantities, `decide_blocks` runs both
  analyses end-to-end over raw coefficient blocks. No bitstream flag is
  emitted/parsed (the v1/v2 mode-flag layout is `[GAP]`). 27 unit
  tests; the crate's test count rises from 384 to 411.

- `excitation` module — the §4 patent-disclosed energy-derived
  quantization matrix `Q[c][d] = E[d]` (US7,930,171 WMA7 formula,
  Background: "coefficient values are squared to get energies, then
  energies are summed within each band"; formula (3): "adjusts the
  matrix by band size … divide by the coefficient count `Card{B[d]}`
  raised to an experimentally-derived exponent"). Public
  `coefficient_energy(c) = c*c` (step 1), `band_raw_energy(&[f64])`
  (steps 1–2 over one band's coefficients), `band_excitation(&[f64],
  exponent)` (full per-band formula incl. the `Card^exponent`
  adjustment), and the layout-level `band_energies(coeffs, &layout)` /
  `excitation_pattern(coeffs, &layout, exponent)` that partition a
  block through a `qband::QuantBandLayout` and emit one weight per
  band. The patent's "experimentally-derived" exponent is a
  caller-supplied `[GAP]` value — never fabricated — with `0.0` (raw
  summed energy) and `1.0` (mean per-coefficient energy) the two
  closed-form endpoints. Per the patent `Q[c][d] = E[d]`, so the
  output feeds `invquant::BandScale::from_weights` as the per-band
  `Q[d]`. 24 unit tests cover the squaring convention and its
  sign-independence, raw-energy summation over empty / mixed-sign /
  all-zero slices, the exponent-0 (raw) and exponent-1 (mean)
  endpoints, single-coefficient exponent-independence, the
  half-exponent sqrt(Card) case, the empty-band zero-not-NaN
  defensive boundary, the proportional-to-energy spreading property,
  layout-level partition correctness and the per-band-primitive
  equivalence, the zero-block all-zeros case, count-mismatch panic
  contracts for both layout helpers, a full S256 block-coverage case,
  and a cross-module thread through `invquant::BandScale` confirming
  the excitation pattern is the quantization matrix the decoder folds.
  Crate test count: 348 → 372.

- `mlt` module — the §3 patent-disclosed MLT forward/inverse
  transform, the primitive the `overlap_add` (Round 12) and `window`
  (Round 13) modules both explicitly deferred (US6,029,126 /
  US6,240,380: MLT = oddly-stacked TDAC cosine-modulated filter bank,
  basis = windowed DCT-IV, FIG.7; US7,383,180 frequency transformer
  530 / decoder FIG.6; US7,930,171: WMA7 applies an MLT to
  variable-size transform blocks). The patent-named bank is realised
  via its general public DSP form (the trace doc's `[DSP]` framing
  tier, as Round 13 did for the sine window): basis
  `cos((π/M)·(n + ½ + M/2)·(k + ½))`. Public `Mlt` carrier per
  `BlockSize` `M`: `Mlt::forward` maps a `2M`-sample
  analysis-windowed frame to `M` spectral coefficients; `Mlt::inverse`
  maps `M` coefficients to the `2M`-sample pre-synthesis-window frame,
  with the `2/M` normalization that makes the full
  window → MLT → overlap-add chain unity-gain for a
  power-complementary pair. Both directions enforce their length
  contracts via `InvalidMltLen { expected, got }`. Accessors
  `block_size`, `coeff_len` (= `M`), `time_len` (= `2M`).
  Re-exports: `Mlt`, `InvalidMltLen`. 24 unit tests cover accessors
  for every `BlockSize::ALL` entry, the cross-module frame-length
  agreement with `window` / `overlap_add`, every length-contract
  reject path in both directions, zeros-to-zeros, linearity, the
  defining oddly-stacked alias structure (first-half antisymmetry,
  second-half symmetry, the exact `inverse∘forward` alias identity,
  `forward∘inverse = 2·X`), end-to-end perfect reconstruction through
  the complete window → MLT → overlap-add chain at S256 / S512, error
  `Display` naming, the `std::error::Error` implementation, and
  `Copy`/`Eq` semantics. Crate test count: 324 → 348.

- `window` module — analysis/synthesis window-pair primitive for the
  §3 patent-disclosed MLT windowing stage (US7,383,180 frequency
  transformer 530: the MLT "operates like a DCT modulated by the sine
  window function(s)"; US6,029,126 / US6,240,380: 2M-length windowing
  over M-length blocks, oddly-stacked TDAC filter bank; US6,240,380
  Eqns.1–2 / NMLBT element 510: the `ha(n)` / `hs(n)` analysis/
  synthesis pair and the MLBT / NMLBT biorthogonal generalization).
  Public `WindowShape` enum names the three patent-disclosed shape
  alternatives (`Sine`, `Mlbt`, `Nmlbt`) with `WindowShape::ALL`,
  `is_realizable()` (only `Sine` — the MLBT / NMLBT parametric forms
  are cited but not reproduced by the trace, so they remain `[GAP]`
  and no coefficient values are fabricated), and `is_biorthogonal()`.
  `Window` carries the `2M` coefficients for a `BlockSize` `M`;
  `Window::sine` realises the patent-named sine shape via the general
  public DSP definition `h(n) = sin((n + ½)·π / 2M)` (the trace doc's
  `[DSP]` framing tier); accessors `shape`, `block_size`, `len`,
  `is_empty`, `coeffs`, `coeff(n)`; `apply_in_place` / `windowed`
  enforce the patent-fixed `2M` input-length contract via
  `InvalidWindowLen { expected, got }` (no mutation on error); and
  `is_power_complementary(tol)` verifies the defining 50%-overlap
  TDAC perfect-reconstruction condition `h(n)² + h(n+M)² = 1`.
  `WindowPair` models the patent's `ha(n)` / `hs(n)` arrangement:
  `new` rejects block-size disagreement via
  `InvalidWindowPair::BlockSizeMismatch { analysis, synthesis }`,
  `orthogonal_sine` builds the orthogonal-MLT pair, and
  `is_orthogonal()` reports whether `ha = hs`. Which window shape
  shipping WMA v1/v2 uses remains `[GAP]` per the trace. Re-exports:
  `Window`, `WindowPair`, `WindowShape`. 23 unit tests cover the
  shape enum (iteration order, realizability, biorthogonal
  partition), sine construction for every `BlockSize::ALL` entry
  (length `2M`, unit-interval bounds, closed-form first coefficient,
  rise/fall monotonicity, symmetry), power-complementarity acceptance
  for every block size plus corrupted-coefficient detection, the
  windowing helpers (sample-wise multiply, in-place ↔ fresh-Vec
  equivalence, every mis-size reject path with the no-mutation
  guarantee), the pair carrier (orthogonal-sine, matching-size
  acceptance, mismatch rejection), a cross-module weighted-overlap-add
  unity-gain test composing the sine pair with `overlap_add::OverlapAdd`
  (constant in → constant out across every steady-state frame), the
  window-length ↔ overlap-add input-length contract match, error
  `Display` naming, and `std::error::Error` implementations. Crate
  test count: 301 → 324.

- `overlap_add` module — stateful decoder-side overlap-add
  (overlapper/adder) carrier for the §3 patent-disclosed
  reconstruction stage (US7,383,180 decoder FIG.6 overlapper/adder;
  US6,029,126 / US6,240,380 oddly-stacked TDAC filter bank, 2M
  windowing over M-length blocks). Public `OverlapAdd` is parameterised
  by a `BlockSize` `M`, enforces the patent-fixed `2M`-sample input
  contract per call via `OverlapAdd::step(input)` (returns
  `InvalidInputLen { expected, got }` on mismatch), and sums the
  previous block's right-half tail with the current block's left half
  to produce `M` time-domain output samples while saving the new
  right half as the tail for the next call. Accessors `block_size`,
  `output_len` (= `M`), `input_len` (= `2M`), `tail_len`, and a
  read-only `tail()` view expose the carrier's state for inspection.
  `reset()` returns the tail to all-zero (e.g. after a seek or
  decoder flush). `flush()` drains the trailing-edge tail to recover
  the last `M` samples a finite stream would otherwise leave buffered,
  then zeroes the internal tail. The carrier takes a *post-windowed*
  inverse-MLT block as input — the synthesis-window shape
  (sine / MLBT / NMLBT) is patent-disclosed as a separate decision
  whose typed carrier is `[GAP]` until a future round stages it; this
  module covers only the patent's overlap-add semantics. Re-exports:
  `OverlapAdd`, `InvalidInputLen`. 23 unit tests cover constructor
  state for every `BlockSize::ALL` variant, the `output_len` /
  `input_len` / `tail_len` invariants, the input-length contract
  (too-short, too-long, empty, mis-sized-for-block rejections, and
  the no-mutation-on-error guarantee that preserves the carried tail),
  the leading-edge first-call behaviour (zeroed tail → output equals
  left half; right half saved as new tail), the defining
  prev-right + curr-left summation rule, a three-block chain that
  verifies the overlap arithmetic stays correct across multiple
  calls, per-`BlockSize` output-length matching, `reset` semantics,
  `flush` semantics including the trailing-edge return and tail
  zeroing, an end-to-end two-blocks-plus-flush sequence that
  produces the patent-arithmetic `3M` total output samples for `2`
  input blocks, error `Display` formatting, and `Clone`
  state-independence. Crate test count: 278 → 301.

- `escape` module — typed escape-symbol literal payload carrier for
  the §6 patent-disclosed run-level entropy stage (US6,223,162
  Claim 4: "the entropy code is an escape code"; Claims 5–6:
  decoder recovers `R` and `L` from the literal trailer). Public
  `EscapeLiteral { run: u32, level: NonZeroU32 }` represents the
  literal payload that follows the escape symbol when an `(R, L)`
  pair was excluded from the probability-thresholded codebook.
  Two construction paths: `EscapeLiteral::new(run, level)` checks
  the Claim-1 / Claim-2 predicates (`run ≥ 1`, `level ≥ 1`) via
  the existing `RunLevelPair::new` and reports
  `EscapeError::InvalidPair(InvalidPair)` on rejection;
  `EscapeLiteral::for_pair(&grid, pair)` consults a
  `CodebookGrid` and admits the pair to the carrier precisely
  when `grid.disposition(pair) == Disposition::Escape` (Claim 4),
  returning `EscapeError::InCodebook` otherwise. Accessors
  `run() -> u32`, `level() -> NonZeroU32`, and
  `level_raw() -> u32` expose the carried fields, and
  `as_run_level_pair() -> RunLevelPair` realises the Claim-5/6
  decoder side by rebuilding the codebook-domain pair the literal
  represents. `EscapeError` implements `std::error::Error`;
  `InCodebook`'s `Display` cites Claim 4 directly so an upstream
  reader can surface the patent-named failure mode without
  string-matching. Run / level field widths are kept at `u32` —
  the patent fixes the structural presence of the literal payload
  but leaves the bit widths as `[GAP]` in the §6 trace, so the
  carrier hosts whatever value the upstream entropy reader
  recovers. Re-exports: `EscapeLiteral`, `EscapeError`. 18 unit
  tests cover the constructor accept paths (minimum (1, 1), large
  values, `u32::MAX` boundary on both run and level), all reject
  paths (`run == 0`, `level == 0`, both zero), the `for_pair`
  cross-check against a 2×2 codebook grid (in-codebook pair
  rejected; below-threshold escape pair accepted; outside-rectangle
  pair accepted), accessor coverage including `Copy`/`Eq`,
  round-trip through `as_run_level_pair` for both constructors and
  at the `u32::MAX` boundary, error `Display` strings (`InvalidPair`
  mentions "run", `InCodebook` mentions "US6,223,162" and "Claim
  4"), and a structural-invariant test that walks every cell of a
  3×3 grid and confirms `for_pair` accepts every escape disposition
  and rejects every in-codebook disposition.

- `step_size` module — typed per-block overall step-size carrier
  for the §4 patent-disclosed arrangement that pairs the per-band
  quantization matrix with a single block-wide step (US7,930,171
  "single overall step size for the whole block"; US7,383,180
  "adaptive, uniform, scalar quantizer that computes one
  quantization factor per tile"; US7,343,291 "step size is varied
  across a rate-control loop"). Public `OverallStepSize` newtype
  carries a single non-zero finite positive `f64`; `new(step)`
  rejects `NaN` / `±∞` / zero / negative inputs via a typed
  `InvalidStepSize` enum (`NotANumber`, `NotFinite { given }`,
  `NotPositive { given }`) that implements `std::error::Error`.
  Accessors `value()`, `apply_to_weight(weight)`, and
  `band_scale_from_weights(weights) -> BandScale` thread the typed
  carrier through to the patent's per-coefficient factor
  `q * Q[d] * step` without re-extracting the inner `f64`.
  Per-block `PerBlockStep { block_size, step }` pairs a `BlockSize`
  with the typed step, exposes `block_size()`, `step()`,
  `coefficient_count()` (re-exporting the block-size sample count
  for the per-coefficient dequant loop), and `fold_with_weights()`
  which materialises the patent's per-band `Q[d] * step` folded
  scale as `BandScale`. Cross-module composition: end-to-end test
  drives `PerBlockStep::fold_with_weights` through
  `BandScale::apply` and confirms the result matches
  `invquant::dequantize_in_place` given the same opaque step.
  Re-exports: `OverallStepSize`, `PerBlockStep`. 26 unit tests
  cover constructor accept paths (typical positive, smallest
  subnormal positive), all reject paths (zero, negative zero,
  negative finite, ±∞, NaN), accessor coverage, the
  `apply_to_weight` ↔ `value()` commutativity, the
  `band_scale_from_weights` ↔ free-function equivalence, the
  `PerBlockStep` per-`BlockSize::ALL` coverage, the
  `fold_with_weights` ↔ free-function end-to-end equivalence
  against `invquant::dequantize_in_place`, `PartialEq`
  differentiating on both block and step, and `Display` naming
  for both `OverallStepSize` and each `InvalidStepSize` variant.
  Crate test count: 234 → 260.

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

## [0.0.3] - 2026-08-30

### Other

- Version rebaseline: 0.0.1 and 0.0.2 on crates.io are the yanked pre-rebuild
  lineage, so the clean-room rebuild publishes from 0.0.3 (user-authorized
  bump; release-plz resumes normal management from here).

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

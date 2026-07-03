# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

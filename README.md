# oxideav-wma

[![CI](https://github.com/OxideAV/oxideav-wma/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-wma/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-wma.svg)](https://crates.io/crates/oxideav-wma) [![docs.rs](https://docs.rs/oxideav-wma/badge.svg)](https://docs.rs/oxideav-wma) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust Windows Media Audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status — clean-room rebuild in progress

This crate is a clean-room rebuild assembled from the staged material
under `docs/audio/wma/`:

* `wiki/Windows_Media_Audio.wiki` — a multimedia.cx orientation snapshot
  describing the WMA v1/v2 extradata layout and the rule mapping
  `(version, sample_rate)` to the per-frame MDCT long-block size.
* `wma-bitstream-from-patents.md` — a patents-only structural trace
  assembled from the Microsoft USPTO patent corpus (Malvar, Chen,
  Thumpudi, Koishida).
* `tables/` + `provenance/02-…` — numeric data tables extracted as
  bytes from the vendor WMA Standard decoder module's own PE data
  sections (coefficient VLC code lengths for all five staged trees,
  the scale/gain delta VLCs, the symbol → run/level companion maps,
  band-partition Hz seeds, the dequantization gain ladder), with
  per-table `.meta` provenance and self-validating extraction.
* `frame-bit-layout.md` — the frame/superframe bit-packing layout
  traced statically from the vendor decoder's bit-parse call graph
  (field order, fixed widths, runtime-width formulas, sign/escape
  placement, sub-stream self-delimiting).

### What works

The **header parser** ([`WmaHeader::parse`]) decodes the
`WAVEFORMATEX` extradata for WMA v1 (codec ID `0x160`) and v2
(`0x161`), the low three `flags2` bits (exponential VLCs / bit
reservoir / variable block length), the per-frame MDCT long-block-size
decision tree (`frame_length_bits ∈ {9, 10, 11}`), and the v2
sample-rate normaliser.

On top of the header, the crate carries a set of typed, individually
tested **DSP and entropy primitives** lifted from the patent trace —
each pinned to the patent it is disclosed in:

* Transform core: the [`mlt`] forward/inverse Modulated Lapped
  Transform (production `O(M log M)` FFT factorization; the direct
  `O(M·2M)` summation survives in-module as the test oracle the fast
  path is pinned against at every block size), the [`window`]
  analysis/synthesis window pair (realizable sine shape; MLBT/NMLBT
  parametric forms left as gaps), the [`overlap_add`] overlapper/adder
  carrier, the [`block`] `{256,512,1024,2048,4096}` block-size set, and
  the [`synthesis`] assembler that chains inverse-MLT → window →
  overlap-add in the patent-fixed order.
* Quantization: [`qmatrix`] differential coding, [`invquant`] inverse
  quantization, [`qband`] band layout, [`step_size`] per-block overall
  step, [`excitation`] energy-derived weights, and the [`dequant`]
  decoder assembler.
* Entropy / coding: [`runlevel`] run-level pairing, [`codebook`] the
  `(R, L)` probability-grid construction model, [`escape`] escape-symbol
  literals, [`entropy_mode`] level vs run-level mode selection,
  [`terminator`] end-of-block selection, and [`spectral`] the §6
  entropy-stage assembler that wires the partition + run-level walker
  into the `M`-coefficient `i32` vector the [`dequant`] stage consumes.
* Channel / band policy: [`stereo`] sum/difference transform,
  [`channel_decision`] the open-loop independent-vs-joint analysis,
  [`stereo_synthesis`] the §8 decoder-tail assembler that wires two
  per-channel [`synthesis`] stages with the conditional inverse
  sum/difference post-process into the final L/R PCM, [`bands`] per-band
  coding policy, [`noisefill`] noise substitution, and [`transient`]
  the per-block transient-handling switch.
* Per-channel pipeline: [`decode`] the §8 FIG.6 single-channel
  decoder-block assembler that chains [`spectral`] (entropy decode) →
  [`dequant`] (inverse quantize/weight) → [`noisefill`] (noise/truncation
  band fill, inserted in its FIG.6-fixed position between the inverse
  quantizer and the inverse MLT) → [`synthesis`] (inverse MLT → window →
  overlap-add) into one stateful `ChannelDecoder` mapping a block's
  already-demuxed parameters to `M` reconstructed time samples; the
  constructor cross-checks all four stages share one `M`.
* Two-channel pipeline: [`stereo_decode`] the §8 FIG.6 **stereo**
  decoder-block assembler — the stereo analogue of [`decode`] — that runs
  two full per-channel [`decode`] chains and folds their two reconstructed
  time-domain channels back to L/R PCM with the §8 `[inverse
  sum-difference]` post-process, gated by the per-block channel mode (the
  fold in its FIG.6-fixed position after each channel's overlap-add, so the
  two overlap-add carriers stay independent). It begins one stage earlier
  than [`stereo_synthesis`] (the entropy box, not the inverse MLT), so it
  is the first assembler taking a stereo block's demuxed per-channel
  entropy symbols all the way to final L/R PCM.
* Frame loop: [`frame`] the §2 block→frame assembler one layer above the
  per-block decoders — [`FrameDecoder`] (mono) and [`StereoFrameDecoder`]
  (stereo) run a frame's ordered list of already-demuxed per-block
  parameter sets through the underlying §8 chain and concatenate the
  per-block PCM into the frame's PCM, threading the overlap-add carrier
  across frames (flushed once at stream end). Uniform-block-size frames
  only; the block-size-transition case is `[GAP]`.
* Stream setup: [`setup`] the wiki "init rate dependent parameters"
  scalars (`high frequency = sample_rate/2`, `bits/sec = bitrate /
  (channels·sr)`, `byte offset bits = log2(bps·frame_length/8)+2`, the
  `use noise coding = 1` default) derived from a parsed [`WmaHeader`],
  plus the [`WmaHeader::long_block_size`] bridge mapping the header's
  `frame_length_bits` exponent onto the typed transform [`BlockSize`]
  that constructs the per-block decoders, and the
  [`WmaHeader::variable_block_length_field`] carrier for the wiki-located
  VBL configuration field (the upper 13 bits of `flags2`; the
  block-size logic it feeds is a documented gap).
* **Encoder side (§8 FIG.5 — the forward mirror of every decode
  stage)**: [`analysis`] (frame formation at 50% TDAC overlap → `ha(n)`
  window → forward MLT, the stateful mirror of [`synthesis`]),
  [`quant`] the §4 forward uniform scalar quantizer mirroring
  [`dequant`] field-for-field, [`runlevel`]`::compress` +
  [`SpectralEncode`] the §6 entropy stage run forward (with
  `min_split_for`, the structural floor the `{1..Rm}` run set imposes
  on the mode boundary), [`ChannelEncoder`] / [`StereoEncoder`] the
  single/two-channel §8 chains (the §5 sum/difference fold in its
  pre-analysis position, per-block `ChannelMode` caller-supplied), and
  [`FrameEncoder`] / [`StereoFrameEncoder`] the §2 frame-loop drivers.
  Tests pin decode(encode(PCM)) ≡ PCM after the chain's `M`-sample
  latency within the quantizer's `divisor/2` bound — mono and stereo,
  block- and frame-level — and that the bound shrinks with the step.
* **Bit-level entropy machinery**: [`bitio`] MSB-first
  `BitWriter`/`BitReader` (MSB-first is the **staged wire fact**: the
  frame-layout trace pins the vendor get-bits mechanism as
  `(acc >> shift) & MASK[n]`, with the mask LUT staged as
  `wma-bitreader-mask-lut` and carried verbatim in [`wire_tables`];
  a cross-check test ties the reader to the staged mask law),
  [`huffman`] the §6 code-book construction *method*
  (canonical Huffman from caller-supplied weights; `from_lengths` is
  the plug-in point for staged real tables, Kraft equality validated),
  [`paircode`] the grid-driven joint `(R, L)` coder with escape
  literals at caller-supplied `[GAP]` widths, and [`matrix_coding`]
  the §4 FIG.1 matrix side-information chain (uniform-quantize →
  differential → Huffman) down to bits. All of it is self-consistent,
  **not** wire-compatible: the literal WMA tables stay `[GAP]`.
* Encoder analysis: [`masking`] the §4 Bark-scale masking model with
  the patent-pinned asymmetric spreading slopes (25 dB/Bark toward
  lower frequencies, 10 toward higher) and the optional
  partial-whitening exponent β.
* **Wire-level data (staged tables)**: [`wire_tables`] carries the
  vendor-module extraction verbatim — the coefficient run-level VLC
  code-length tables for all five staged trees (decode classes 1/2/3
  primary — 666 / 1016 / 476 symbols — plus the class-1/3 alt
  variants, 555 / 435), the scale (121) and gain (37) delta VLC
  lengths, the 25-edge critical-band Hz partition seed, the 11-edge
  octave subband seed, the 113-step `10^(1/16)` (1.25 dB/step)
  dequantization gain ladder, the four decode-class selector
  threshold constants (bit-exact `f32`: bounds `0.125` / `1.6`,
  branch thresholds `0.72` / `1.16`), and the 32-entry get-bits mask
  LUT that pins the reader's MSB-first field order. [`runlevel_tables`] carries the
  symbol → `(run, |level|)` companion maps for the three primary
  classes (2-based indexing). Two earlier readings carried here were
  since **overturned by newer staging**: the round-6 docs establish
  that the §3.1 line-spectral envelope path *does* exist (its
  codebook is not a plain const array, which is why the data-section
  search missed it), and the round-3 trie walk resolves mode 2 to a
  complete 1336-symbol alphabet with **escape = symbol 0 and
  end-of-block = symbol 1** — the vendor-exact code tables live in
  `wire_codes`/`wire_vlc` and are what the vendor decode path uses,
  while these modules survive as the earlier self-consistent loop.
  [`coef_vlc`] realises the earlier staged length tables as
  canonical codes and expands decoded symbols into typed
  `EndOfBlock` / `Escape` / `(run, |level|)` events. [`envelope_vlc`]
  realises the scale/gain delta VLCs (the scale table's own data pins
  its zero-delta center at symbol 60). [`exponent_bands`] derives the
  per-block exponent/quantization-band and noise-grid partitions the
  vendor decoder computes instead of storing. [`gain_ladder`] maps
  decoded exponent indices to the §4 `Q[d]` weights.
* **Wire bit layout** ([`frame_bits`], from the staged
  `frame-bit-layout.md` decode-path trace): the S1/S2/S3 per-frame
  header (reservoir offset at the staged `byte_offset_bits` formula
  width → side field → 1-bit flag), the pinned B1..B6 per-block field
  order (7-bit block header → gain VLC sub-stream → 1-bit
  stereo/coupling flag on 2-channel streams → 5-bit envelope base →
  scale VLC sub-stream → coefficient run-level sub-stream), one
  trailing **sign bit per non-zero coefficient**, the corrected
  escape shape (symbol 1 + literal run + literal level at
  runtime-signalled widths), and the self-delimiting
  coefficient-count rule with EOB (symbol 0) for trailing zeros.
  Byte-exact layout pins, an exhaustive all-alphabets pair sweep
  (2,152 pairs), escape boundary sweeps, and no-panic fuzz passes
  hold it down — plus codec-level hardening sweeps (every strict bit
  prefix of a valid frame fails with a typed error, single-bit
  corruption never panics the parser, arbitrary bytes never panic
  the packet entry point) and a `fuzz/` sub-crate with six
  libFuzzer targets over the header, wire-parse, wire-round-trip,
  staged-VLC, vendor-decode and vendor-encoder-round-trip surfaces.
* **Wire frame codec** ([`wire_chain`]): [`select_decode_class`]
  carries the staged §4b rule with the staged threshold constants
  wired in (class 3 pinned below 32 kHz; above the gate the
  per-stream rate float is clamped to the staged `[0.125, 1.6]` axis
  and located against the staged 0.72 / 1.16 branch thresholds as a
  typed `RateFloatRegion` — the branch *directions* stay a documented
  gap), `WireFrameCodec::from_header_pinned_class` builds the codec
  wherever the rule pins the class, `CoefDecodeMode::
  from_class_and_variant` realises the staged six-descriptor
  class × alt-variant registration crossing (the class-2 alt slot is
  the documented hole), and [`WireFrameCodec`] derives everything
  derivable from a parsed [`WmaHeader`] (S1 width, escape literal
  widths per the staged source pins — run at the side-field width,
  level at `byte_offset_bits` —, scale count = derived band count,
  coefficient count = block size) to emit and parse whole frames as
  bytes. Milestone pinned by test, mono (mode 2) and stereo (mode 1):
  **PCM → §8 encoder chain → quantized coefficients → the staged
  frame bit layout with the real VLCs → bytes → parse → §8 decoder
  chain → PCM** within the quantizer bound, wire round trip
  field-exact.

Each module computes the quantitative property the patents fix and
leaves the encoder's tuning constants (band-size exponents, decision
thresholds, generator construction) as caller-supplied parameters,
never fabricated. The patent trace marks several bitstream specifics as
gaps (`[GAP]`), which the typed carriers name side-by-side rather than
guessing. The crate carries 900+ unit tests.

With the r390 wire pass the crate is a **complete, self-consistent
codec loop at the wire-bit level**: PCM → analysis → quantize →
run-level events → the staged frame bit layout with the real vendor
tables → bytes → parse → entropy decode → dequantize → noise-fill →
synthesis → PCM, round-tripping within the §4 quantizer bound. What
separates this from decoding *vendor* WMA files is the short list of
still-unstaged semantic bindings below.

### Vendor-bitstream decode (r439, extended r446/r450/r454)

The freshly staged rounds 3–6 of `docs/audio/wma/` (exact vendor VLC
codewords for all eight trees, the §0–§5 frame-bit layout with the
packet header and stereo sections, the exponent-band partitions, and
six committed genuine vendor-encoder bitstreams) closed most of the
old gap list, and the crate now parses and decodes **real
vendor-encoded WMA v2 streams**:

* [`wire_codes`] / [`wire_vlc`] — the exact vendor `(length, code)`
  assignment for all eight staged trees (the class-2 primary at its
  full 1336-symbol alphabet and the class-2 alt at 1072 — both
  superseding the earlier partial flat-scan reading — plus all six
  2-based run/level companion maps including the alt variants). No
  staged table matches the canonical reconstruction, so these
  explicit codes are what makes vendor decode possible.
* [`stream_config`] (§0), [`packet`] (§1 superframe header +
  bit-reservoir carry assembly), [`band_partition`] (the eight
  staged partitions + computed walk), [`vendor_frame`] (the §2–§4
  frame/block parse), [`vendor_decode`] (§5 mid/side inverse +
  staged-ladder dequantisation + synthesis to PCM).
* Measured on the six committed vendor streams
  (`tests/vendor_streams.rs`, fixtures referenced from the docs
  staging area and never copied here): the §1 packet layer holds on
  **all 1769 packets**; the frame layer closes **1738 / 1763** §1
  carry boundaries — five families completely (mono 8 kHz
  **394/394**, stereo 22.05 kHz A/V **1098/1098**, the whole
  44.1 kHz high-rate family **3/3**, **13/13**, **133/133**), and
  the mono 22.05 kHz stream — the old "F1 anomaly", which turned
  out to be the §2.1 noise-substitution sub-stream, not F1 —
  **97/122** under the r454 measured noise policy (below).
* The PCM leg (r450: variable-size lapped reconstruction with
  neighbour-matched sine slopes — the thing the §2 three-field
  opening's neighbouring block sizes exist for — plus the calibrated
  dequantisation composition: staged `10^((e − e_max)/16)` ladder
  ratio anchored at the block's maximum exponent, total gain at 1 dB
  per B1 step, a single black-box-calibrated absolute scale) — sharpened in r454 by the computed band partition rounding to
  the **nearest** multiple of four (the staged hard-table post-pass,
  resolving the staged `.meta`'s rounding caveat; this was the
  dominant residual error) — reaches **per-second median SNR
  45.3 / 50.3 / 60.4 dB** with **corr² 0.998–1.000** and fitted
  gain ≈ 1 against a black-box reference decode on the three
  fully-closing envelope-coded families (stereo 22.05 kHz, 44.1 kHz
  96 kbps, 44.1 kHz VBR), **13.9 dB / corr² 0.951** on the mono
  22.05 kHz noise-substitution family, and 4.7 dB on the mono 8 kHz
  LSP-envelope stream (its conversion tables are the remaining
  staged gap on that family).
* Four §1/§2/§5 details calibrated *differently* from the staged
  reading, with the §1 carry boundary as ground truth (reported to
  the docs staging as erratum/extension asks): the F1 field is a
  one-ahead **pipeline** of block sizes; the **B2 envelope-reuse bit
  exists on short blocks of two-channel streams, one bit per block**
  (r446 — revising r439's "no B2 bit", which had only measured the
  unconditional readings; its 0-value skips the coded channels'
  envelopes in favour of the §3 per-block-size cache, and the
  committed corpus cannot separate `channels == 2` from
  `n_block_sizes ≥ 8` as the gate); a **zero §1 carry marks the
  previous packet as padded** (frames complete inside it, the
  remainder is filler — the VBR streams pad most packets); and the
  joint-stereo ALT-tree consequence is **channel-scoped** (second
  coded channel — the difference channel — only).

* The **§2.1 noise-substitution policy is measured** (r454, via the
  crate's own encoder mirror judged by the black-box reference):
  enabled at 22.05 kHz below the staged 1.16 class-selector
  threshold on the §0.2 rate float; the walk runs over the
  exponent-band partition from per-size start edges 716/356/148
  (1024/512/256-coefficient blocks, each pinned up to its 2/2/3
  flag count); on enabled streams **every short block carries the
  B2 bit, mono included** — r446's contrary mono reading was
  confounded by the then-unknown F3 bits.
  `vendor_frame::measured_noise_policy` carries the rule; parser,
  emitter and synthesiser apply it by default (substituted bands
  zero-fill: the vendor noise generator is unstaged).

### What is still open

* the **§2.1 noise-substitution closed forms**: the r454 policy is
  a black-box measurement, not a staged fact — still open are the
  closed-form start rule behind the measured per-size edges
  (716/356/148), the exact enable rule beyond the measured bracket
  (rate floats below ≈ 0.58 diverge in further unmeasured ways, and
  16/32 kHz configurations diverge for reasons not yet isolated),
  the vendor **noise generator** (flagged bands currently
  zero-fill; the F4 gains are parsed but unused), and the remaining
  25 mono-22.05 kHz carry boundaries;
* the **vendor-literal dequantisation composition and
  transition-window shape** (r450/r454 carry the measured-best
  realisation — the r454 weight-law probe shows agreement within
  0.4 dB down to 24 ladder steps below the block maximum, with a
  divergence beyond that whose closed form is unstaged);
* the **§3.1 line-spectral envelope conversion tables** (wire format
  staged and parsed; the index → envelope mapping is not — the mono
  8 kHz stream decodes with a flat envelope meanwhile);
* **WMA v1** specifics (no v1 vendor stream exists in the staged
  set) and the v1 per-channel byte-alignment rule.

(The r439 "44.1 kHz high-rate residual" is **closed** — it was the
missing B2 reuse bit plus the zero-carry padding semantic; the
family parses 149/149. The r450 "mono 22.05 kHz F1 anomaly" is
likewise **closed**: it was never F1 but the §2.1
noise-substitution sub-stream, whose measured policy the crate now
carries.)

### Vendor-wire encoder (r454)

The encoder mirror is complete end-to-end at the vendor wire level —
no longer the self-consistent-only §8 chain:

* [`vendor_encode`] — the §2–§4 **frame/block emitter**
  ([`FrameEmitter`], the field-for-field inverse of the vendor
  parser's latch / F1-pipeline state machine: three-field openings
  exactly where a packet-body boundary was crossed, the one-ahead F1
  pipeline, F2a before the channel flags, B1 `0x7f` chaining, the
  per-block B2 rule, §3 scale-VLC envelope deltas with the v1 base,
  §2.1 all-clear F3 walks under the measured policy, and §4
  run-level coefficients over the staged vendor codes — companion
  pairs via the reverse index (`wire_vlc::runlevel_index`), escapes
  at the gain-mapped widths, EOB, per-event signs, channel-scoped
  ALT in joint blocks) and the §1 **packet writer**
  ([`VendorBitWriter`]: frames laid back-to-back, every packet's
  P1/P2/P3 derived from where the boundaries fell, zero-carry
  padding as the flush mechanism, hard per-frame §1 bounds from the
  packet body and the P3 field width).
* [`vendor_analysis`] — the signal stage: forward lapped transform
  at the synthesiser's own slot geometry (TDAC identity pinned by
  test, uniform and mixed-size), per-band envelope extraction on the
  staged ladder scale kept inside the reference-measured matched
  regime, quantisation by the exact decode composition (the shared
  decode-side functions), the §5 mid/side fold with the
  encoder-side halving, a transient-probe block scheduler for VBL
  streams, and per-frame rate control (one gain offset searched
  against the configuration's average bits per frame, floored so
  the peak |q| always fits the gain tier's escape ceiling).
  [`VendorEncoder`] drives PCM → `block_align` §1 packets.
* Measured (`tests/encoder_streams.rs`), per family: **own-chain
  SNR 19–26 dB** (encode → this crate's own vendor decode), and
  **black-box wire-format acceptance** — the reference decoder
  accepts a minimal RIFF/`WAVEFORMATEX` wrap of the emitted packets
  and decodes it at **corr² 0.98–0.995 with fitted gain ≈ 1.0**
  (stereo/mono 22.05 kHz VBL incl. the noise-policy family,
  44.1 kHz 96 kbps, and the ACM catalogue's 186-byte headerless
  configuration with its vendor-declared extradata bytes).
  Acceptance is the bar; bit-parity with a vendor encoder is not
  claimed. The §3.1 LSP envelope path is not encodable (its
  conversion tables are a staged gap) and is refused at
  construction.

### Framework registration (r450, encoder r454)

The crate registers into [`oxideav_core`] with the framework's dual
API ([`registration`]): `register(ctx)` installs decoder **and
encoder** factories for codec ids `wma1` / `wma2` with their
`WAVEFORMATEX` tag claims (`0x0160` / `0x0161`); [`make_decoder`]
and [`make_encoder`] are the direct factories. [`WmaEncoder`] takes
interleaved-F32 frames and emits `block_align`-byte §1 codec packets
at flush (packets sized at eight average frames, the staged vendor
configurations' own ratio), with `output_params` carrying the
`WAVEFORMATEX`-shaped extradata a muxer needs.
[`WmaDecoder`] wraps the vendor decode chain behind the core
`Decoder` trait — one `Packet` per `block_align`-sized codec packet,
one-packet latency for the §1 reservoir carry, interleaved F32
output in the reference ±1.0 convention, silence substitution for
unparseable frames so the §1 frame counts keep the timeline, and
`reset()` for post-seek reuse. The registration layer is pinned
sample-exact against the direct chain on all six committed vendor
streams (11.4 M samples).

## Public surface

```rust
use oxideav_wma::{Version, WmaHeader};

// codec ID 0x161 from the container's WAVEFORMATEX
let v = Version::from_codec_id(0x161).unwrap();
let h = WmaHeader::parse(
    v,
    48_000,  // sample_rate from container
    2,       // channels
    192_000, // bit_rate
    1024,    // block_align
    &[0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0xCA], // extradata
)
.unwrap();
assert_eq!(h.sample_rate, 44_100);     // v2 snaps 48k → 44.1k
assert_eq!(h.frame_length_bits, 11);   // 2048-sample MDCT block
assert!(h.bit_reservoir);              // flags2 bit 1
```

## Provenance

Clean-room from `docs/audio/wma/` only. No external library source,
archived prior history, or online resources were consulted. Every
patent-disclosed primitive cites its source patent inline in the code;
DSP realizations of patent-named transforms use the general public DSP
form (the trace's `[DSP]` framing tier), not WMA-specific facts.

## License

MIT. See `LICENSE`.

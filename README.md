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
  classes (664 / 1333 / 474 pairs, 2-based indexing). The same
  extraction **confirms no LSP codebook exists** on this decode path.
  [`coef_vlc`] realises every staged table as a working canonical
  code matching the staged CSVs bit-for-bit — including mode 2 under
  the corrected reading (EOB = symbol 0, escape = symbol 1; the
  earlier "8 missing escape codewords" premise was overturned by the
  docs watch pass) — and expands decoded symbols into typed
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
  the packet entry point).
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
guessing. The crate carries 817 unit tests.

With the r390 wire pass the crate is a **complete, self-consistent
codec loop at the wire-bit level**: PCM → analysis → quantize →
run-level events → the staged frame bit layout with the real vendor
tables → bytes → parse → entropy decode → dequantize → noise-fill →
synthesis → PCM, round-tripping within the §4 quantizer bound. What
separates this from decoding *vendor* WMA files is the short list of
still-unstaged semantic bindings below.

### What is NOT implemented

**Vendor-produced WMA streams do not decode end-to-end yet.** The
crate now speaks the staged wire *layout* — real VLCs, real field
order, real widths where the formulas are staged — and round-trips
its own frames byte-exactly, but the remaining semantic bindings are
still unstaged:

* the **S2 frame side-field width formula** and therefore the
  concrete per-stream **escape literal widths** (their *sources* are
  pinned: side-field width and `byte_offset_bits`; the side-field
  width value itself is runtime/config);
* the **gain/scale delta chaining semantics** (initial values,
  per-band application order, wrap rule) and the **B1 / B4 field
  semantics** (the 7-bit block header beyond its `0x7f` marker, the
  5-bit envelope base) — the fields and symbol streams are carried
  verbatim;
* the **gain sub-stream element count** per block;
* the §4b class selector's **branch directions** and **rate-float
  formula** (the four threshold constants are now staged and wired
  in — see above — but which side of the 0.72 / 1.16 thresholds
  selects which class, whether the middle region keeps the class-3
  default, and how the per-stream float is derived from the header
  are all still caller-observed);
* the **class-2 alt-variant VLC** (located, unextracted) and the
  **alt variants' run/level companion maps**;
* **frames-per-packet / the bit-reservoir walk** and the
  variable-block-length split (runtime-gated per the staged trace);
* verification that the vendor decode tree's internal **bit
  assignment** matches the canonical reconstruction, and the exact
  codes of mode 2's DAG-replicated high symbols (blocked statically
  by decode-DAG space sharing, dynamically behind a COM
  `ProcessOutput` vtable call).

Each is a data-staging item under `docs/audio/wma/`. The
[`oxideav_core`] registration will land once vendor streams decode.

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

# oxideav-wma

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
  sections (coefficient VLC code lengths, band-partition Hz seeds,
  the dequantization gain ladder), with per-table `.meta` provenance
  and self-validating extraction.

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
  Transform (reference `O(M·2M)` summation), the [`window`]
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
  that constructs the per-block decoders.
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
  `BitWriter`/`BitReader` (the packing order of the real bitstream is
  `[GAP]`; the convention is a documented realization detail with one
  swap point), [`huffman`] the §6 code-book construction *method*
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
  code-length tables for decode modes 1 (666 symbols, Kraft = 1),
  2 (1016 real symbols; the 8 escape symbols' codeword enumeration is
  the extraction's documented residual, deficit pinned exactly), and
  3 (476 symbols, Kraft = 1), the 25-edge critical-band Hz partition
  seed, the 11-edge octave subband seed, and the 113-step `10^(1/16)`
  (1.25 dB/step) dequantization gain ladder. The same extraction
  **confirms no LSP codebook exists** on this decode path.
  [`coef_vlc`] realises modes 1/3 as working canonical codes whose
  codewords match the staged CSVs bit-for-bit (mode 2 is a typed
  docs-gap). [`exponent_bands`] derives the per-block
  exponent/quantization-band and noise-grid partitions the vendor
  decoder computes instead of storing (Hz seed → coefficient bins,
  round-half-up as the one documented realization detail).
  [`gain_ladder`] maps decoded exponent indices to the §4 `Q[d]`
  weights. [`wire_chain`] assembles it all from a parsed
  [`WmaHeader`]: header → block size → real partitions → ladder
  weights → the §8 encoder/decoder chains, with PCM round trips over
  the real geometry pinned by test.

Each module computes the quantitative property the patents fix and
leaves the encoder's tuning constants (band-size exponents, decision
thresholds, generator construction) as caller-supplied parameters,
never fabricated. The patent trace marks several bitstream specifics as
gaps (`[GAP]`), which the typed carriers name side-by-side rather than
guessing. The crate carries 742 unit tests.

With the encoder mirror in place the crate is a **complete,
self-consistent codec loop at the typed-symbol level**: PCM → analysis
→ quantize → run-level symbols → (optionally, self-consistent bits via
[`paircode`]/[`matrix_coding`]) → entropy decode → dequantize →
noise-fill → synthesis → PCM, round-tripping within the §4 quantizer
bound. What separates this from a *WMA* codec is wire compatibility
(see below).

### What is NOT implemented

There is **no real-WMA bitstream-byte → PCM decode yet** (and no
wire-compatible encode). The staged extraction closed the biggest
data gaps — the coefficient VLC lengths, the band-partition seeds,
the gain ladder, and the LSP negative are all in-tree now — but the
remaining wire specifics are still unstaged:

* the **symbol → `(R, L)` mapping** of the coefficient VLCs (the
  vendor module's companion index-ramp tables were located but their
  per-column role is not pinned);
* the **mode-2 escape codeword enumeration** (needs the verified
  decode-tree walk);
* the smaller **scale (~121) / gain (~37) VLC tables** that carry the
  per-band exponent indices;
* the **sign-bit placement, escape literal widths, per-band
  noise/cutoff flag encoding, and frame/superframe bit layout**
  (bit-reader path, needs a validator round over real streams);
* verification that the vendor decode tree's internal **bit
  assignment** matches the canonical reconstruction of its exact
  lengths;
* how the **decode mode (1/2/3) is selected** from the stream header.

Each is a data-staging item under `docs/audio/wma/`; the machinery on
this side ([`bitio`], [`huffman`]`::from_lengths`, [`coef_vlc`],
[`wire_chain`]) is built and waiting. The [`oxideav_core`]
registration will land once the wire format is pinned.

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

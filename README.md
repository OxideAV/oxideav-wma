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
  literals, [`entropy_mode`] level vs run-level mode selection, and
  [`terminator`] end-of-block selection.
* Channel / band policy: [`stereo`] sum/difference transform,
  [`channel_decision`] the open-loop independent-vs-joint analysis,
  [`stereo_synthesis`] the §8 decoder-tail assembler that wires two
  per-channel [`synthesis`] stages with the conditional inverse
  sum/difference post-process into the final L/R PCM, [`bands`] per-band
  coding policy, [`noisefill`] noise substitution, and [`transient`]
  the per-block transient-handling switch.

Each module computes the quantitative property the patents fix and
leaves the encoder's tuning constants (band-size exponents, decision
thresholds, generator construction) as caller-supplied parameters,
never fabricated. The patent trace marks several bitstream specifics as
gaps (`[GAP]`), which the typed carriers name side-by-side rather than
guessing. The crate carries 468 unit tests.

### What is NOT implemented

There is **no end-to-end bitstream decode**. The wiki snapshot lists
the names of WMA's data tables — the gain / LSP / scale / coefficient /
level Huffman tables, the per-rate exponent-band partition tables, and
the critical-frequency curves — but does not contain the tables
themselves. Growing the actual MDCT/Huffman decode path requires either
a spec PDF or a clean-room reverse-engineered trace doc staged under
`docs/audio/wma/`. The crate is therefore a library of validated
primitives, not yet a usable codec; the [`oxideav_core`] registration
will land once the bitstream decode path is implementable.

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

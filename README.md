# oxideav-wma

Pure-Rust Windows Media Audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 1** landed the WAVEFORMATEX-extradata header parser and the
sample-rate → MDCT long-block decision tree, sourced from the
multimedia.cx wiki snapshot at `docs/audio/wma/wiki/Windows_Media_Audio.wiki`:

* WMA v1 (codec ID `0x160`) and v2 (codec ID `0x161`) extradata
  layouts inside `WAVEFORMATEX` (4 bytes for v1; 6 bytes for v2);
* the meaning of the low three bits of `flags2`
  (exponential VLCs / bit reservoir / variable block length);
* the per-frame MDCT long-block-size decision tree as a function of
  `(version, sample_rate)`, yielding `frame_length_bits ∈ {9, 10, 11}`
  and `frame_length = 1 << frame_length_bits`;
* one explicit cutoff in the v2 sample-rate normaliser
  (`sample_rate >= 44_100` snaps to `44_100`).

Round 1 ships 21 tests behind [`WmaHeader::parse`].

**Round 2** (this round) lifts the §2 patent-disclosed **block-size
set** out of the patents-only structural trace
(`docs/audio/wma/wma-bitstream-from-patents.md`, citing
US7,930,171 Chen-171 Background) into a typed
[`BlockSize`] primitive:

```text
{ S256, S512, S1024, S2048, S4096 }    // 8..=12 bits log2
```

The enum exposes [`BlockSize::ALL`] (ascending iteration), `samples()`
/ `log2_samples()` accessors, validating constructors
[`BlockSize::from_samples`] / [`BlockSize::from_log2`], and
`is_shortest()` / `is_longest()` predicates for transient-handling
code. A new [`Error::InvalidBlockSize`] variant carries the rejected
sample count when a non-set value is offered. Round 2 ships 14
additional tests; one cross-module test verifies that every
`WmaHeader::frame_length` Round 1 produces is itself a member of the
patent set, so future transform code can wrap a header-supplied frame
length without a redundant lookup.

## What is NOT in this round

The wiki snapshot lists the names of WMA's data tables — the gain
Huffman table (37 entries), the LSP codebook, the scale Huffman
table (121 entries), the coefficient 0…5 Huffman tables (666, 555,
1336, 1072, 476, 435 entries), the levels 0…5 tables (60, 40, 340,
180, 70, 40 entries), the per-rate exponent-band partition tables,
and the critical-frequency curves — but the snapshot **does not
contain those tables**. The actual MDCT/Huffman bitstream decode
path therefore stays out of `src/` this round; growing it requires
either a spec PDF or a clean-room reverse-engineered trace doc
staged under `docs/audio/wma/`. See the docs-gap notes in
`src/header.rs` for the boundary cases the wiki leaves
under-specified in the v2 sample-rate normaliser as well.

## Public surface

```rust
use oxideav_wma::{Version, WmaHeader};

// codec ID 0x161 from the container's WAVEFORMATEX
let v = Version::from_codec_id(0x161).unwrap();
let h = WmaHeader::parse(
    v,
    48_000, // sample_rate from container
    2,      // channels
    192_000,// bit_rate
    1024,   // block_align
    &[0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0xCA], // extradata
)
.unwrap();
assert_eq!(h.sample_rate, 44_100);     // v2 snaps 48k → 44.1k
assert_eq!(h.frame_length_bits, 11);   // 2048-sample MDCT block
assert!(h.bit_reservoir);               // flags2 bit 1
```

The [`WmaHeader`] struct exposes every field the wiki names. The
[`oxideav_core::CodecResolver`] registration will land once the
bitstream decode path is implementable.

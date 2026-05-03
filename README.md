# oxideav-wma

Pure-Rust **Windows Media Audio** decoder. Round 1 lands the v1 + v2
baseline (Microsoft codec tags `0x0160` / `0x0161`); WMA Pro
(`0x0162`) and WMA Lossless (`0x0163`) are scheduled for round 2.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone. Zero C dependencies.

## Status — round 1

| Variant       | Codec ID | Decode | Encode | Notes                                                  |
|---------------|----------|--------|--------|--------------------------------------------------------|
| WMA v1        | 0x0160   | yes    | no     | Single-block-size MDCT path (the baseline shape)       |
| WMA v2        | 0x0161   | yes    | no     | Same as v1 + v2 escape coding + v2 band overrides      |
| WMA Pro       | 0x0162   | no     | no     | Round 2 — vector VLCs + per-channel tile layout        |
| WMA Lossless  | 0x0163   | no     | no     | Round 2 — Rice residues + cascaded LMS predictors      |

The v1/v2 decoder accepts the configurations FFmpeg's `wmav1` and
`wmav2` encoders produce by default (`flags2 = 0x0001`):

- single fixed block size equal to `frame_len`,
- VLC-coded exponents (AAC scale-factor codebook),
- noise-coded high band off,
- bit reservoir off (one frame per ASF data packet).

Real-world Microsoft-encoded streams that flip the `use_bit_reservoir`
or `use_variable_block_len` bits in `flags2` will fail with
`Unsupported`; that path is the round-2 work item.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-wma  = "0.0"
```

## Decoder API

```rust
use oxideav_core::{CodecId, CodecParameters, Frame, Packet};
use oxideav_core::registry::CodecRegistry;

let mut codecs = CodecRegistry::new();
oxideav_wma::register(&mut codecs);

// Build a decoder for a stream whose ASF WAVEFORMATEX tag was 0x0160:
let mut params = CodecParameters::audio(CodecId::new("wmav1"));
params.extradata = Some(/* the 4-byte WMA v1 extradata */ vec![0, 0, 0x01, 0x00]);
let mut dec = codecs.make_decoder(&params)?;

dec.send_packet(&Packet::new(/* one ASF data packet payload */))?;
let frame = dec.receive_frame()?;
```

## References

- `audio/wma/wma-trace-reverse-engineering.md` — observed bitstream
  shape of all four WMA variants from instrumented FFmpeg traces.
- `audio/wma/data/wma-bands-by-rate.md` — per-rate critical-band
  partition tables.
- `audio/wma/data/wma-spectral-vlc.md` — six run-level VLC codebooks +
  AAC scale-factor codebook + LSP codebook + `pow_tab[]`.
- Microsoft *Advanced Systems Format Specification* rev 01.20.06 — ASF
  container.

## Round-2 backlog

- WMA Pro: 18-byte extradata, per-channel tile header, three vector
  VLCs (`vec4` / `vec2` / `vec1`), two run-level tail VLCs, scale-
  factor DPCM/RL VLCs, Givens-rotation custom decorrelation matrices,
  default decorrelation matrices for n ∈ 1..6 channels.
- WMA Lossless: Rice-style residue coder, cascaded LMS predictors
  (CDLMS, MCLMS), AC autoregressive filter, integer pipeline.
- Bit reservoir + variable block lengths for v1/v2 (Microsoft-encoded
  fixtures).
- Noise-coded high band (`use_noise_coding` path) for low-bitrate
  v1/v2 streams.

## License

MIT.

#![no_main]

//! Fuzz: structure-aware wire round trip at real 8 kHz S512 geometry.
//!
//! Fuzzer bytes are sanitized into a **valid** `WireFrame` — frame
//! header fields masked to the staged widths, gain/scale symbols
//! folded into their staged alphabets, non-zero coefficients placed
//! with runs inside the escape-literal envelope so every `(R, L)`
//! event is emittable. Contract:
//!
//! * `encode_frame` accepts the sanitized frame;
//! * `decode_frame` of the emitted bytes is **field-exact** equal;
//! * dropping the final bit fails with a typed error (the staged
//!   self-delimiting rule: the parser always knows what it is owed).

use libfuzzer_sys::fuzz_target;
use oxideav_wma::coef_vlc::CoefDecodeMode;
use oxideav_wma::frame_bits::{FrameHeaderFields, WireBlock, WireFrame};
use oxideav_wma::wire_chain::WireFrameCodec;
use oxideav_wma::{Version, WmaHeader};
use std::sync::OnceLock;

fn codec() -> &'static WireFrameCodec {
    static CODEC: OnceLock<WireFrameCodec> = OnceLock::new();
    CODEC.get_or_init(|| {
        let header = WmaHeader::parse(Version::V2, 8_000, 1, 64_000, 0, &[0; 6]).unwrap();
        WireFrameCodec::from_header(&header, CoefDecodeMode::Mode3, 6).unwrap()
    })
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    let codec = codec();
    let n = codec.config().exponent_band_count();
    let m = usize::from(codec.config().block_size().samples());
    let widths = codec.widths();

    let byte = |i: usize| data[i % data.len()];

    // Frame header fields, masked to the staged runtime widths.
    let reservoir_offset =
        (u32::from(byte(0)) | (u32::from(byte(1)) << 8)) & ((1u32 << widths.byte_offset_bits) - 1);
    let side_field = u32::from(byte(2)) & ((1u32 << widths.side_field_bits) - 1);
    let flag = byte(3) & 1 == 1;

    // Non-zero coefficients: up to 8, strictly increasing positions
    // inside the first 60 bins (runs stay far below the 6-bit escape
    // run ceiling), levels in ±255 (below the escape level ceiling).
    let mut coefficients = vec![0i32; m];
    let count = usize::from(byte(4) & 0x7) + 1;
    let mut pos = 0usize;
    for i in 0..count {
        pos += usize::from(byte(5 + 2 * i) & 0x7);
        if pos >= 60 {
            break;
        }
        let mut level = i32::from(byte(6 + 2 * i)) - 128;
        if level == 0 {
            level = 1;
        }
        coefficients[pos] = level;
        pos += 1;
    }

    let block = WireBlock {
        header: byte(21) & 0x7f,
        gain_symbols: vec![usize::from(byte(22)) % 37],
        stereo_coupling: None,
        envelope_base: byte(23) & 0x1f,
        scale_symbols: (0..n).map(|d| usize::from(byte(24 + d)) % 121).collect(),
        coefficients,
    };
    let frame = WireFrame {
        header: FrameHeaderFields {
            reservoir_offset,
            side_field,
            flag,
        },
        channel_blocks: vec![vec![block]],
    };

    let (bytes, bit_len) = codec
        .encode_frame(&frame)
        .expect("sanitized frame must encode");
    let decoded = codec
        .decode_frame(&bytes, bit_len, 1)
        .expect("emitted bytes must parse");
    assert_eq!(decoded, frame, "wire round trip must be field-exact");

    assert!(
        codec.decode_frame(&bytes, bit_len - 1, 1).is_err(),
        "dropping the final bit must fail (self-delimiting rule)",
    );
});

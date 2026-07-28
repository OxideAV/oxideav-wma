#![no_main]

//! Fuzz: arbitrary bytes through the wire frame/packet parsers.
//!
//! Two real header geometries (the staged class-3 pin below the
//! 32 kHz gate, and a 44.1 kHz stereo stream on the class-1 table),
//! several runtime frame/block counts each. Contract: pure
//! panic-freedom — `decode_frame` / `decode_packet` return a typed
//! `Result` for every input, never panic, never run unbounded.

use libfuzzer_sys::fuzz_target;
use oxideav_wma::coef_vlc::CoefDecodeMode;
use oxideav_wma::wire_chain::WireFrameCodec;
use oxideav_wma::{Version, WmaHeader};
use std::sync::OnceLock;

fn codecs() -> &'static Vec<WireFrameCodec> {
    static CODECS: OnceLock<Vec<WireFrameCodec>> = OnceLock::new();
    CODECS.get_or_init(|| {
        let mono = WmaHeader::parse(Version::V2, 8_000, 1, 64_000, 0, &[0; 6]).unwrap();
        let stereo = WmaHeader::parse(Version::V2, 44_100, 2, 128_000, 0, &[0; 6]).unwrap();
        vec![
            WireFrameCodec::from_header(&mono, CoefDecodeMode::Mode3, 6).unwrap(),
            WireFrameCodec::from_header(&stereo, CoefDecodeMode::Mode1, 6).unwrap(),
        ]
    })
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    let sel = data[0];
    let payload = &data[1..];
    let frame_count = usize::from(sel & 0x3) + 1;
    let blocks = usize::from((sel >> 2) & 0x3) + 1;
    // Also sweep a bit-precise (non-byte-aligned) stream length.
    let full_bits = payload.len() * 8;
    let bit_len = full_bits - usize::from((sel >> 4) & 0x7).min(full_bits);

    for codec in codecs() {
        let _ = codec.decode_frame(payload, bit_len, blocks);
        let _ = codec.decode_packet(payload, bit_len, frame_count, blocks);
    }
});

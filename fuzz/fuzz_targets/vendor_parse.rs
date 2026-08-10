#![no_main]

//! Fuzz: arbitrary bytes through the vendor-path §1 packet layer and
//! the §2–§4 frame parser (the calibrated vendor decode front end).
//!
//! Four real stream configurations spanning the staged vendor-stream
//! families (mono/stereo, LSP/exp-VLC envelopes, 1/2/3-bit F1
//! fields). Contract: pure panic-freedom — every input either parses
//! to typed frames or returns a typed error; nothing panics and
//! nothing runs unbounded.

use libfuzzer_sys::fuzz_target;
use oxideav_wma::header::Version;
use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_frame::{FrameParser, NoiseSpec, NoiseStart};
use std::sync::OnceLock;

fn configs() -> &'static Vec<StreamConfig> {
    static CONFIGS: OnceLock<Vec<StreamConfig>> = OnceLock::new();
    CONFIGS.get_or_init(|| {
        vec![
            // The staged vendor-stream geometries (small block_align
            // variants keep the fuzz corpus effective).
            StreamConfig::derive(Version::V2, 8_000, 1, 1_000, 160, 0x0026).unwrap(),
            StreamConfig::derive(Version::V2, 22_050, 2, 4_006, 186, 0x0017).unwrap(),
            StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 320, 0x000f).unwrap(),
            StreamConfig::derive(Version::V1, 32_000, 2, 4_000, 192, 0x0003).unwrap(),
        ]
    })
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    let sel = data[0];
    let payload = &data[1..];
    let cfg = &configs()[usize::from(sel & 0x3)];
    let noise_first_band = usize::from((sel >> 2) & 0x7);
    let with_noise = sel & 0x20 != 0;

    let ba = usize::from(cfg.block_align);
    let mut asm = PacketAssembler::new(cfg);
    for pkt in payload.chunks_exact(ba).take(8) {
        // Errors are fine; panics are not.
        let _ = asm.push_packet(pkt);
    }
    let stream = asm.finish();
    if stream.packets.is_empty() {
        return;
    }
    let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
    let mut parser = FrameParser::new(cfg, &body_starts);
    if with_noise {
        parser = parser.with_noise(NoiseSpec {
            start: NoiseStart::Band(noise_first_band),
        });
    }
    let mut reader = stream.reader_at(u64::from(stream.packets[0].header.carry_bits));
    for _ in 0..24 {
        if parser.parse_frame(&mut reader).is_err() {
            break;
        }
    }
});

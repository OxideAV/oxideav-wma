#![no_main]

//! Fuzz: structure-aware vendor-wire **encoder** round trip.
//!
//! Two legs behind a selector bit:
//!
//! * **Wire leg** — fuzzer bytes are sanitised into contract-valid
//!   `EncBlockData` frames (valid block-size schedules summing to the
//!   frame length, chain-clamped envelopes, levels inside the gain
//!   tier's escape ceiling, per-config joint flags), emitted through
//!   `VendorBitWriter` (which applies the measured noise/B2 policy),
//!   packetised, then decoded back through `PacketAssembler` +
//!   `FrameParser` + `BlockSynth`. Contract: every committed frame
//!   parses back **field-exact** (sizes, gain, envelope, coefficients,
//!   joint flag), every §1 carry boundary closes, and synthesis emits
//!   exactly `block_size` samples per channel. `FrameTooLong` is the
//!   one accepted refusal (tiny packets, dense frame) — the frame is
//!   skipped, nothing must desync.
//! * **PCM leg** — fuzzer bytes become a short PCM buffer pushed
//!   through `VendorEncoder`; the emitted packets must decode with
//!   finite output and the §1 layer intact.
//!
//! Configurations span the measured families: the 22.05 kHz low-rate
//! noise-policy config, stereo VBL, the headerless tiny-packet
//! geometry, and a 44.1 kHz reservoir stream.

use libfuzzer_sys::fuzz_target;
use oxideav_wma::header::Version;
use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_decode::BlockSynth;
use oxideav_wma::vendor_encode::{EmitError, EncBlockData, EncChannelData, EncEnvelope, VendorBitWriter};
use oxideav_wma::vendor_frame::{escape_level_width, Envelope, FrameParser};
use oxideav_wma::VendorEncoder;
use std::sync::OnceLock;

fn configs() -> &'static Vec<StreamConfig> {
    static CONFIGS: OnceLock<Vec<StreamConfig>> = OnceLock::new();
    CONFIGS.get_or_init(|| {
        vec![
            // The measured noise-policy family (mono 22.05 kHz low rate).
            StreamConfig::derive(Version::V2, 22_050, 1, 2_003, 744, 0x000f).unwrap(),
            // Stereo VBL + reservoir (staged cand_stereo22k geometry).
            StreamConfig::derive(Version::V2, 22_050, 2, 4_006, 744, 0x0017).unwrap(),
            // Headerless tiny packets (ACM catalogue format 17).
            StreamConfig::derive(Version::V2, 22_050, 2, 4_005, 186, 0x0001).unwrap(),
            // 44.1 kHz mono reservoir, small packets.
            StreamConfig::derive(Version::V2, 44_100, 1, 8_003, 800, 0x0003).unwrap(),
        ]
    })
}

struct ByteFeed<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> ByteFeed<'a> {
    fn next(&mut self) -> u8 {
        let b = self.data.get(self.pos).copied().unwrap_or(0x5a);
        self.pos += 1;
        b
    }
}

fn max_size_index(cfg: &StreamConfig) -> u8 {
    let mut idx = 0u8;
    while cfg.block_size_for_index(idx + 1).is_some() {
        idx += 1;
    }
    idx
}

fn sanitized_block(cfg: &StreamConfig, feed: &mut ByteFeed<'_>, size_index: u8) -> EncBlockData {
    let block_size = cfg.block_size_for_index(size_index).expect("valid index");
    let bands = oxideav_wma::band_partition::exponent_band_count(cfg.sample_rate, block_size);
    let n_coef = usize::from(cfg.coef_end(block_size) - cfg.coef_start(block_size));
    let total_gain = u32::from(feed.next() % 200) + 1;
    let ceiling = (1i64 << escape_level_width(total_gain)) - 1;
    let joint_stereo = cfg.channels == 2 && feed.next() & 1 == 1;
    let channels = (0..cfg.channels)
        .map(|_| {
            if feed.next() % 8 == 0 {
                return EncChannelData {
                    coded: false,
                    envelope: None,
                    coefficients: Vec::new(),
                };
            }
            // Chain-clamped envelope inside [0, 60].
            let mut exponents = Vec::with_capacity(bands);
            let mut prev = 36i32;
            for _ in 0..bands {
                let delta = i32::from(feed.next() % 41) - 20;
                let e = (prev + delta).clamp(0, 60);
                exponents.push(e);
                prev = e;
            }
            // Sparse coefficients: increasing positions, clamped levels.
            let mut coefficients = vec![0i32; n_coef];
            let mut pos = 0usize;
            for _ in 0..16 {
                pos += usize::from(feed.next()) * 3 + 1;
                if pos >= n_coef {
                    break;
                }
                let mut level = i64::from(feed.next()) * i64::from(feed.next()) - 8000;
                level = level.clamp(-ceiling, ceiling);
                if level == 0 {
                    level = 1;
                }
                coefficients[pos] = level as i32;
                pos += 1;
            }
            EncChannelData {
                coded: true,
                envelope: Some(EncEnvelope::Exponents(exponents)),
                coefficients,
            }
        })
        .collect();
    EncBlockData {
        size_index,
        joint_stereo,
        total_gain,
        channels,
    }
}

fn wire_leg(cfg: &StreamConfig, feed: &mut ByteFeed<'_>) {
    let max_idx = max_size_index(cfg);
    // 1..=5 frames of sanitized blocks with valid schedules.
    let frame_count = usize::from(feed.next() % 5) + 1;
    let mut frames: Vec<Vec<EncBlockData>> = Vec::with_capacity(frame_count);
    for _ in 0..frame_count {
        let mut remaining = cfg.frame_length;
        let mut blocks = Vec::new();
        while remaining > 0 {
            let mut idx = feed.next() % (max_idx + 1);
            while cfg.block_size_for_index(idx).expect("valid") > remaining {
                idx += 1;
            }
            let block = sanitized_block(cfg, feed, idx);
            remaining -= cfg.block_size_for_index(idx).expect("valid");
            blocks.push(block);
        }
        frames.push(blocks);
    }

    let mut writer = VendorBitWriter::new(cfg).expect("supported configs only");
    let mut committed: Vec<Vec<EncBlockData>> = Vec::new();
    for (i, frame) in frames.iter().enumerate() {
        let next_first = frames.get(i + 1).map(|f| f[0].size_index);
        match writer.write_frame(frame, next_first) {
            Ok(()) => committed.push(frame.clone()),
            // Tiny packets can refuse a dense frame; anything else is
            // a sanitiser bug worth crashing on.
            Err(EmitError::FrameTooLong { .. }) => {}
            Err(e) => panic!("sanitized frame refused: {e}"),
        }
    }
    if committed.is_empty() {
        return;
    }
    let packets = writer.finish().expect("finish after clean writes");
    assert!(packets
        .iter()
        .all(|p| p.len() == usize::from(cfg.block_align)));

    // Decode back: field-exact frames, closing boundaries, sized PCM.
    let mut asm = PacketAssembler::new(cfg);
    for p in &packets {
        asm.push_packet(p).expect("emitted packets are valid");
    }
    let stream = asm.finish();
    let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
    let mut parser = FrameParser::new(cfg, &body_starts);
    let mut synth = BlockSynth::new(cfg);
    let mut got: Vec<oxideav_wma::vendor_frame::ParsedFrame> = Vec::new();
    let mut cursor = stream.packets[0].frames_start_bit();
    for (i, rec) in stream.packets.iter().enumerate() {
        if cursor != rec.frames_start_bit() {
            cursor = rec.frames_start_bit();
            parser.raise_latch();
        }
        let mut reader = stream.reader_at(cursor);
        for f in 0..rec.header.frame_count {
            let frame = parser
                .parse_frame(&mut reader)
                .unwrap_or_else(|e| panic!("packet {i} frame {f}: {e}"));
            for block in &frame.blocks {
                for chan in synth.block(block) {
                    assert_eq!(chan.len(), usize::from(block.block_size));
                    assert!(chan.iter().all(|v| v.is_finite()));
                }
            }
            got.push(frame);
        }
        cursor = reader.position() as u64;
        if let Some(next) = stream.packets.get(i + 1) {
            if next.header.carry_bits > 0 {
                assert_eq!(cursor, next.frames_start_bit(), "carry boundary must close");
            } else {
                assert!(cursor <= next.body_start_bit, "padding boundary must close");
            }
        }
    }
    let _ = synth.flush();

    assert_eq!(got.len(), committed.len(), "frame count");
    for (sent, parsed) in committed.iter().zip(got.iter()) {
        assert_eq!(parsed.blocks.len(), sent.len());
        for (sb, pb) in sent.iter().zip(parsed.blocks.iter()) {
            assert_eq!(
                pb.block_size,
                cfg.block_size_for_index(sb.size_index).expect("valid")
            );
            assert_eq!(pb.joint_stereo, sb.joint_stereo);
            if sb.channels.iter().any(|c| c.coded) {
                assert_eq!(pb.total_gain, sb.total_gain);
            }
            for (sc, pc) in sb.channels.iter().zip(pb.channels.iter()) {
                assert_eq!(pc.coded, sc.coded);
                if !sc.coded {
                    continue;
                }
                match (sc.envelope.as_ref().unwrap(), pc.envelope.as_ref().unwrap()) {
                    (EncEnvelope::Exponents(e), Envelope::Exponents(d)) => assert_eq!(d, e),
                    (s, d) => panic!("envelope mismatch: {s:?} vs {d:?}"),
                }
                assert_eq!(pc.coefficients, sc.coefficients);
            }
        }
    }
}

fn pcm_leg(cfg: &StreamConfig, feed: &mut ByteFeed<'_>) {
    let mut enc = match VendorEncoder::new(cfg) {
        Ok(e) => e,
        Err(_) => return,
    };
    // Up to ~2.5 frames of fuzzer-shaped PCM.
    let samples = usize::from(cfg.frame_length) * 2 + usize::from(cfg.frame_length) / 2;
    let pcm: Vec<Vec<f64>> = (0..cfg.channels)
        .map(|_| {
            (0..samples)
                .map(|_| f64::from(i16::from(feed.next() as i8)) / 130.0)
                .collect()
        })
        .collect();
    enc.push(&pcm).expect("valid input shape");
    let packets = enc.finish().expect("encode completes");
    let mut asm = PacketAssembler::new(cfg);
    for p in &packets {
        asm.push_packet(p).expect("emitted packets are valid");
    }
    let stream = asm.finish();
    if stream.packets.is_empty() {
        return;
    }
    let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
    let mut parser = FrameParser::new(cfg, &body_starts);
    let mut synth = BlockSynth::new(cfg);
    let mut cursor = stream.packets[0].frames_start_bit();
    for rec in stream.packets.iter() {
        if cursor != rec.frames_start_bit() {
            cursor = rec.frames_start_bit();
            parser.raise_latch();
        }
        let mut reader = stream.reader_at(cursor);
        for _ in 0..rec.header.frame_count {
            let frame = parser.parse_frame(&mut reader).expect("own stream parses");
            for block in &frame.blocks {
                for chan in synth.block(block) {
                    assert!(chan.iter().all(|v| v.is_finite()));
                }
            }
        }
        cursor = reader.position() as u64;
    }
    let _ = synth.flush();
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let sel = data[0];
    let mut feed = ByteFeed {
        data: &data[1..],
        pos: 0,
    };
    let cfg = &configs()[usize::from(sel & 0x3)];
    if sel & 0x4 == 0 {
        wire_leg(cfg, &mut feed);
    } else {
        pcm_leg(cfg, &mut feed);
    }
});

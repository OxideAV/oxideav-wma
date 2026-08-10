//! Vendor-bitstream validation — the six committed Microsoft-encoder
//! streams under `docs/audio/wma/reference/vendor-streams/`
//! (black-box fixture bytes; see that directory's README for
//! provenance and licences).
//!
//! The fixtures are **not** copied into this repo: the tests locate
//! them via `OXIDEAV_WMA_VENDOR_STREAMS_DIR` or the umbrella
//! workspace layout, and unwrap the ASF container with a black-box
//! `ffmpeg -c copy -f data` invocation (an opaque validator binary —
//! its output was verified to be exactly the `nBlockAlign`-sized
//! codec packets the staged measurement describes). When the
//! fixtures or the binary are absent the tests skip.

use std::path::PathBuf;
use std::process::Command;

use oxideav_wma::header::Version;
use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_frame::FrameParser;

/// One committed vendor stream's container configuration, from the
/// staged `reference/vendor-streams/README.md` table.
struct StreamSpec {
    file: &'static str,
    sample_rate: u32,
    channels: u8,
    avg_bytes_per_sec: u32,
    block_align: u16,
    flags2: u16,
    /// Staged packet-header measurement (packets, frames_total) from
    /// `tables/vendor-stream-packet-headers.csv`. The staged reader
    /// capped at 256 packets per file; `None` marks the two capped
    /// rows (the full stream is longer).
    staged_counts: Option<(usize, u32)>,
}

const SPECS: [StreamSpec; 6] = [
    StreamSpec {
        file: "cand_apollo8.wma",
        sample_rate: 44_100,
        channels: 2,
        avg_bytes_per_sec: 8003,
        block_align: 2973,
        flags2: 0x000f,
        staged_counts: Some((4, 33)),
    },
    StreamSpec {
        file: "cand_mono22k_16kbps.wma",
        sample_rate: 22_050,
        channels: 1,
        avg_bytes_per_sec: 2003,
        block_align: 744,
        flags2: 0x000f,
        staged_counts: Some((123, 968)),
    },
    StreamSpec {
        file: "cand_mono8k_8kbps_v8.wma",
        sample_rate: 8000,
        channels: 1,
        avg_bytes_per_sec: 1000,
        block_align: 640,
        flags2: 0x0026,
        staged_counts: None, // staged row capped at 256 of 395 packets
    },
    StreamSpec {
        file: "cand_stereo22k_32kbps_av.wma",
        sample_rate: 22_050,
        channels: 2,
        avg_bytes_per_sec: 4006,
        block_align: 744,
        flags2: 0x0017,
        staged_counts: None, // staged row capped at 256 of 1099 packets
    },
    StreamSpec {
        file: "cand_vbr_q75_stereo.wma",
        sample_rate: 44_100,
        channels: 2,
        avg_bytes_per_sec: 11_111,
        block_align: 4459,
        flags2: 0x000f,
        staged_counts: Some((14, 173)),
    },
    StreamSpec {
        file: "cand_wmp12_96kbps.wma",
        sample_rate: 44_100,
        channels: 2,
        avg_bytes_per_sec: 12_003,
        block_align: 4459,
        flags2: 0x000f,
        staged_counts: Some((134, 1072)),
    },
];

fn vendor_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("OXIDEAV_WMA_VENDOR_STREAMS_DIR") {
        let p = PathBuf::from(dir);
        return p.is_dir().then_some(p);
    }
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/audio/wma/reference/vendor-streams");
    p.is_dir().then_some(p)
}

/// Black-box ASF unwrap: the audio stream's codec packets,
/// concatenated (every packet is exactly `block_align` bytes).
fn extract_packets(path: &std::path::Path) -> Option<Vec<u8>> {
    let out = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(path)
        .args(["-map", "0:a:0", "-c", "copy", "-f", "data", "-"])
        .output()
        .ok()?;
    out.status.success().then_some(out.stdout)
}

fn config_for(spec: &StreamSpec) -> StreamConfig {
    StreamConfig::derive(
        Version::V2,
        spec.sample_rate,
        spec.channels,
        spec.avg_bytes_per_sec,
        spec.block_align,
        spec.flags2,
    )
    .expect("staged configurations derive")
}

struct Loaded {
    spec: &'static StreamSpec,
    cfg: StreamConfig,
    packets: Vec<Vec<u8>>,
}

fn load_all() -> Option<Vec<Loaded>> {
    let dir = vendor_dir()?;
    let mut out = Vec::new();
    for spec in &SPECS {
        let raw = extract_packets(&dir.join(spec.file))?;
        let cfg = config_for(spec);
        let ba = usize::from(spec.block_align);
        assert_eq!(
            raw.len() % ba,
            0,
            "{}: extracted bytes are not whole packets",
            spec.file
        );
        let packets: Vec<Vec<u8>> = raw.chunks_exact(ba).map(|c| c.to_vec()).collect();
        out.push(Loaded { spec, cfg, packets });
    }
    Some(out)
}

macro_rules! skip_unless_fixtures {
    () => {
        match load_all() {
            Some(loaded) => loaded,
            None => {
                eprintln!(
                    "skipping: vendor streams or ffmpeg unavailable \
                     (set OXIDEAV_WMA_VENDOR_STREAMS_DIR to run)"
                );
                return;
            }
        }
    };
}

/// The §1 packet layer must hold on every packet of every committed
/// vendor stream: sequence continuity mod 16, frame counts in range,
/// carry strictly inside the body — the three checks the staged
/// measurement used, here over the *full* streams (the staged reader
/// capped at 256 packets; totals for the uncapped files must match
/// its rows exactly).
#[test]
fn packet_layer_holds_on_all_vendor_packets() {
    let loaded = skip_unless_fixtures!();
    for l in &loaded {
        let mut asm = PacketAssembler::new(&l.cfg);
        for (i, pkt) in l.packets.iter().enumerate() {
            let rec = asm
                .push_packet(pkt)
                .unwrap_or_else(|e| panic!("{} packet {i}: {e}", l.spec.file));
            assert!(
                !rec.discontinuity,
                "{} packet {i}: unexpected sequence break",
                l.spec.file
            );
            assert!(
                (1..=15).contains(&rec.header.frame_count),
                "{} packet {i}: frame count {}",
                l.spec.file,
                rec.header.frame_count
            );
        }
        let stream = asm.finish();
        let frames_total: u32 = stream
            .packets
            .iter()
            .map(|p| u32::from(p.header.frame_count))
            .sum();
        let carry_nonzero = stream
            .packets
            .iter()
            .filter(|p| p.header.carry_bits > 0)
            .count();
        eprintln!(
            "{}: {} packets, {} frames declared, {} non-zero carries",
            l.spec.file,
            stream.packets.len(),
            frames_total,
            carry_nonzero
        );
        if let Some((packets, staged_frames)) = l.spec.staged_counts {
            assert_eq!(stream.packets.len(), packets, "{}", l.spec.file);
            assert_eq!(frames_total, staged_frames, "{}", l.spec.file);
        } else {
            // The staged rows for these two files capped at 256; the
            // full stream must be at least that long.
            assert!(stream.packets.len() > 256, "{}", l.spec.file);
        }
    }
}

/// Frame-level §2–§4 measurement: parse each packet's declared
/// frames and compare the landing position against the next packet's
/// carry boundary — the ground truth §1 embeds. Prints the per-file
/// closure rate.
#[test]
fn frame_parse_closes_on_packet_carry_boundaries() {
    let loaded = skip_unless_fixtures!();
    let mut all_aligned = 0usize;
    let mut all_packets = 0usize;
    for l in &loaded {
        let mut asm = PacketAssembler::new(&l.cfg);
        for pkt in &l.packets {
            asm.push_packet(pkt).unwrap();
        }
        let stream = asm.finish();
        let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
        let mut parser = FrameParser::new(&l.cfg, &body_starts);

        let mut aligned = 0usize;
        let mut parse_errors = 0usize;
        let mut boundaries = 0usize;
        let mut cursor = stream.packets[0].frames_start_bit();
        for (i, rec) in stream.packets.iter().enumerate() {
            if cursor != rec.frames_start_bit() {
                // Mis-parse upstream: resynchronise at the §1 carry
                // boundary, as a real decoder would after an error.
                cursor = rec.frames_start_bit();
                parser.raise_latch();
            }
            let mut reader = stream.reader_at(cursor);
            let mut failed = false;
            for _ in 0..rec.header.frame_count {
                match parser.parse_frame(&mut reader) {
                    Ok(_) => {}
                    Err(_) => {
                        parse_errors += 1;
                        failed = true;
                        break;
                    }
                }
            }
            cursor = reader.position() as u64;
            if let Some(next) = stream.packets.get(i + 1) {
                boundaries += 1;
                if !failed && cursor == next.frames_start_bit() {
                    aligned += 1;
                }
            }
        }
        eprintln!(
            "{}: {}/{} boundaries closed exactly, {} frame-parse errors",
            l.spec.file, aligned, boundaries, parse_errors
        );
        // Per-family floors at the measured r439 closure rates (the
        // vendor-calibrated F1 pipeline / channel-scoped ALT /
        // no-reuse-bit parser). A regression below any floor means a
        // parser change broke a previously-closing family.
        let floor = match l.spec.file {
            "cand_mono8k_8kbps_v8.wma" => 394,      // 394/394 (100 %)
            "cand_stereo22k_32kbps_av.wma" => 1080, // 1086/1098
            "cand_mono22k_16kbps.wma" => 60,        // 64/122
            "cand_wmp12_96kbps.wma" => 5,           // 5/133
            "cand_vbr_q75_stereo.wma" => 2,         // 2/13
            "cand_apollo8.wma" => 1,                // 1/3
            _ => 0,
        };
        assert!(
            aligned >= floor,
            "{}: closure regressed to {aligned}/{boundaries} (floor {floor})",
            l.spec.file
        );
        all_aligned += aligned;
        all_packets += boundaries;
    }
    eprintln!("total: {all_aligned}/{all_packets} boundaries closed");
    assert!(
        all_aligned >= 1540,
        "global closure regressed: {all_aligned}/{all_packets}"
    );
}

/// PCM leg: decode each vendor stream through the full chain
/// (§1 assembly → §2–§4 parse → §5 mid/side + staged-ladder
/// dequantisation → synthesis) and compare a mono downmix against a
/// black-box reference decode (best-lag + scalar-gain fit — the
/// staged docs leave the absolute dequantisation scale open, so the
/// gain is fitted; the correlation and SNR are the signal).
#[test]
fn vendor_pcm_decodes_and_correlates() {
    use oxideav_wma::vendor_decode::BlockSynth;

    let dir = match vendor_dir() {
        Some(d) => d,
        None => {
            eprintln!("skipping: vendor streams unavailable");
            return;
        }
    };
    let loaded = skip_unless_fixtures!();
    for l in &loaded {
        // Black-box reference decode: mono downmix, f32le.
        let out = Command::new("ffmpeg")
            .args(["-v", "error", "-i"])
            .arg(dir.join(l.spec.file))
            .args(["-f", "f32le", "-ac", "1", "-"])
            .output();
        let reference: Vec<f64> = match out {
            Ok(o) if o.status.success() => o
                .stdout
                .chunks_exact(4)
                .map(|c| f64::from(f32::from_le_bytes([c[0], c[1], c[2], c[3]])))
                .collect(),
            _ => {
                eprintln!("skipping PCM leg: reference decoder unavailable");
                return;
            }
        };

        let mut asm = PacketAssembler::new(&l.cfg);
        for pkt in &l.packets {
            asm.push_packet(pkt).unwrap();
        }
        let stream = asm.finish();
        let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
        let mut parser = FrameParser::new(&l.cfg, &body_starts);
        let mut synth = BlockSynth::new(&l.cfg);

        let mut pcm: Vec<f64> = Vec::new();
        let mut cursor = stream.packets[0].frames_start_bit();
        for rec in stream.packets.iter() {
            if cursor != rec.frames_start_bit() {
                cursor = rec.frames_start_bit();
                parser.raise_latch();
                synth.reset();
            }
            let mut reader = stream.reader_at(cursor);
            for f in 0..rec.header.frame_count {
                match parser.parse_frame(&mut reader) {
                    Ok(frame) => {
                        for block in &frame.blocks {
                            let chans = synth.block(block);
                            for t in 0..chans[0].len() {
                                let sum: f64 = chans.iter().map(|c| c[t]).sum();
                                pcm.push(sum / chans.len() as f64);
                            }
                        }
                    }
                    Err(_) => {
                        // Zero-fill the remaining declared frames so
                        // the timeline stays aligned to the §1 counts.
                        let remaining = usize::from(rec.header.frame_count - f);
                        pcm.extend(
                            std::iter::repeat(0.0)
                                .take(usize::from(l.cfg.frame_length) * remaining),
                        );
                        synth.reset();
                        break;
                    }
                }
            }
            cursor = reader.position() as u64;
        }
        assert!(!pcm.is_empty(), "{}: no PCM produced", l.spec.file);
        assert!(
            pcm.iter().all(|x| x.is_finite()),
            "{}: non-finite PCM",
            l.spec.file
        );

        // Best-lag + gain fit over a middle window.
        let sr = l.spec.sample_rate as usize;
        let win = (sr / 2).min(pcm.len().saturating_sub(1));
        let start = (reference.len() / 3).min(reference.len().saturating_sub(win + 1));
        let mut best = (0i64, f64::NEG_INFINITY, 0.0f64);
        for lag in -5000i64..=5000 {
            let (mut dot, mut ee, mut rr) = (0.0, 0.0, 0.0);
            for t in (start..start + win).step_by(4) {
                let u = t as i64 + lag;
                if u < 0 || u as usize >= pcm.len() {
                    continue;
                }
                let (a, b) = (reference[t], pcm[u as usize]);
                dot += a * b;
                ee += b * b;
                rr += a * a;
            }
            if ee == 0.0 || rr == 0.0 {
                continue;
            }
            let corr = dot * dot / (ee * rr);
            if corr > best.1 {
                best = (lag, corr, dot / ee);
            }
        }
        let (lag, corr2, gain) = best;

        // Per-second SNR over live (non-filled) chunks; median.
        let mut chunk_snrs: Vec<f64> = Vec::new();
        let mut t0 = sr;
        while t0 + sr < reference.len().saturating_sub(sr) {
            let (mut num, mut den) = (0.0, 0.0);
            let mut live = 0usize;
            for (t, &a) in reference.iter().enumerate().skip(t0).take(sr) {
                let u = t as i64 + lag;
                if u < 0 || u as usize >= pcm.len() {
                    continue;
                }
                let b = pcm[u as usize] * gain;
                if b != 0.0 {
                    live += 1;
                }
                num += a * a;
                den += (a - b) * (a - b);
            }
            if live > sr / 2 && num > 1e-9 {
                chunk_snrs.push(10.0 * (num / den.max(1e-30)).log10());
            }
            t0 += sr;
        }
        chunk_snrs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = chunk_snrs
            .get(chunk_snrs.len() / 2)
            .copied()
            .unwrap_or(f64::NAN);
        eprintln!(
            "{}: lag {lag}, corr² {corr2:.3}, per-sec median SNR {median:.2} dB ({} chunks)",
            l.spec.file,
            chunk_snrs.len()
        );

        // Floors at the measured r439 quality for the families the
        // parse closes (the dequantisation composition rule is still
        // an open staged item, so these are correlation floors, not
        // fidelity claims).
        match l.spec.file {
            "cand_apollo8.wma" => {
                assert!(corr2 > 0.9, "apollo8 corr² regressed: {corr2}");
            }
            "cand_mono8k_8kbps_v8.wma" | "cand_stereo22k_32kbps_av.wma" => {
                assert!(
                    median > 2.5,
                    "{}: median SNR regressed to {median:.2} dB",
                    l.spec.file
                );
            }
            _ => {}
        }
    }
}

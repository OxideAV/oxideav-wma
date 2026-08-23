//! Vendor-bitstream validation — the six committed Microsoft-encoder
//! streams under `docs/audio/wma/reference/vendor-streams/`
//! (black-box fixture bytes; see that directory's README for
//! provenance and licences).
//!
//! The fixtures are **not** copied into this repo: the tests locate
//! them via `OXIDEAV_WMA_VENDOR_STREAMS_DIR` or the umbrella
//! workspace layout, and unwrap the ASF container with a black-box
//! validator invocation (`extract_packets` — an opaque binary whose
//! output was verified to be exactly the `nBlockAlign`-sized codec
//! packets the staged measurement describes). When the fixtures or
//! the binary are absent the tests skip.

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
/// carry boundary — the ground truth §1 embeds. A boundary closes
/// when the parse lands exactly on the next packet's carry offset,
/// or — when the next packet declares a **zero** carry — when the
/// declared frames all completed inside the packet (the remainder is
/// padding; the VBR streams pad most packets). Prints the per-file
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
                // Padding skip or mis-parse upstream: resynchronise
                // at the §1 carry boundary, as the decoder does at
                // every packet header.
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
                let closed = if next.header.carry_bits > 0 {
                    cursor == next.frames_start_bit()
                } else {
                    // Zero carry: the frames ended inside this
                    // packet; the rest of its body is padding.
                    cursor <= next.body_start_bit
                };
                if !failed && closed {
                    aligned += 1;
                }
            }
        }
        eprintln!(
            "{}: {}/{} boundaries closed, {} frame-parse errors",
            l.spec.file, aligned, boundaries, parse_errors
        );
        // Per-family floors at the measured r446 closure rates (the
        // vendor-calibrated F1 pipeline / channel-scoped ALT /
        // two-channel short-block B2 / zero-carry-padding parser).
        // A regression below any floor means a parser change broke a
        // previously-closing family. Five of the six families close
        // completely; the mono 22.05 kHz stream carries the open F1
        // anomaly (see the round report).
        let floor = match l.spec.file {
            "cand_mono8k_8kbps_v8.wma" => 394,      // 394/394 (100 %)
            "cand_stereo22k_32kbps_av.wma" => 1098, // 1098/1098 (100 %)
            "cand_mono22k_16kbps.wma" => 60,        // 64/122
            "cand_wmp12_96kbps.wma" => 133,         // 133/133 (100 %)
            "cand_vbr_q75_stereo.wma" => 13,        // 13/13 (100 %)
            "cand_apollo8.wma" => 3,                // 3/3 (100 %)
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
        all_aligned >= 1700,
        "global closure regressed: {all_aligned}/{all_packets}"
    );
}

/// The registered [`oxideav_wma::WmaDecoder`] must produce exactly
/// the PCM the direct chain produces (same §1/§2 parse, same
/// synthesiser, same silence-substitution policy), stream-for-stream
/// — the registration layer adds packet bookkeeping and f32
/// interleaving, nothing decode-semantic.
#[test]
fn registered_decoder_matches_the_direct_chain() {
    use oxideav_core::{CodecId, CodecParameters, Decoder, Frame, TimeBase};
    use oxideav_wma::vendor_decode::BlockSynth;

    let loaded = skip_unless_fixtures!();
    for l in &loaded {
        // Direct chain (the PCM-leg loop below, without the fit).
        let mut asm = PacketAssembler::new(&l.cfg);
        for pkt in &l.packets {
            asm.push_packet(pkt).unwrap();
        }
        let stream = asm.finish();
        let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
        let mut parser = FrameParser::new(&l.cfg, &body_starts);
        let mut synth = BlockSynth::new(&l.cfg);
        let mut direct: Vec<Vec<f64>> = vec![Vec::new(); usize::from(l.spec.channels)];
        let mut cursor = stream.packets[0].frames_start_bit();
        let mut clean_pad = false;
        for (i, rec) in stream.packets.iter().enumerate() {
            if cursor != rec.frames_start_bit() {
                cursor = rec.frames_start_bit();
                parser.raise_latch();
                if !clean_pad {
                    synth.reset();
                }
            }
            let mut reader = stream.reader_at(cursor);
            let mut failed = false;
            for f in 0..rec.header.frame_count {
                match parser.parse_frame(&mut reader) {
                    Ok(frame) => {
                        for block in &frame.blocks {
                            for (ch, chan) in synth.block(block).into_iter().enumerate() {
                                direct[ch].extend_from_slice(&chan);
                            }
                        }
                    }
                    Err(_) => {
                        let remaining = usize::from(rec.header.frame_count - f);
                        for chan in &mut direct {
                            chan.extend(
                                std::iter::repeat(0.0)
                                    .take(usize::from(l.cfg.frame_length) * remaining),
                            );
                        }
                        synth.reset();
                        failed = true;
                        break;
                    }
                }
            }
            cursor = reader.position() as u64;
            clean_pad = !failed
                && stream
                    .packets
                    .get(i + 1)
                    .is_some_and(|n| n.header.carry_bits == 0 && cursor <= n.body_start_bit);
        }
        for (ch, chan) in synth.flush().into_iter().enumerate() {
            direct[ch].extend_from_slice(&chan);
        }

        // Registered decoder over the same packets.
        let mut params = CodecParameters::audio(CodecId::new("wma2"));
        params.sample_rate = Some(l.spec.sample_rate);
        params.channels = Some(u16::from(l.spec.channels));
        params.bit_rate = Some(u64::from(l.spec.avg_bytes_per_sec) * 8);
        let mut extradata = vec![0u8; 4];
        extradata.extend_from_slice(&l.spec.flags2.to_le_bytes());
        params.extradata = extradata;
        let mut dec = oxideav_wma::make_decoder(&params).unwrap();
        let tb = TimeBase::new(1, i64::from(l.spec.sample_rate));
        let mut registered: Vec<f32> = Vec::new();
        let drain = |dec: &mut Box<dyn Decoder>, out: &mut Vec<f32>| loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(f)) => {
                    out.extend(
                        f.data[0]
                            .chunks_exact(4)
                            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
                    );
                }
                Ok(_) => panic!("non-audio frame"),
                Err(_) => break,
            }
        };
        for pkt in &l.packets {
            dec.send_packet(&oxideav_core::Packet::new(0, tb, pkt.clone()))
                .unwrap();
            drain(&mut dec, &mut registered);
        }
        dec.flush().unwrap();
        drain(&mut dec, &mut registered);

        let channels = usize::from(l.spec.channels);
        assert_eq!(
            registered.len(),
            direct[0].len() * channels,
            "{}: sample-count mismatch",
            l.spec.file
        );
        for (t, frame) in registered.chunks_exact(channels).enumerate() {
            for (ch, &v) in frame.iter().enumerate() {
                let want = direct[ch][t] as f32;
                assert!(
                    (v - want).abs() <= want.abs() * 1e-6 + 1e-9,
                    "{}: sample {t} ch {ch}: {v} vs {want}",
                    l.spec.file
                );
            }
        }
        eprintln!(
            "{}: registered decoder matches the direct chain over {} samples/ch",
            l.spec.file,
            direct[0].len()
        );
    }
}

/// PCM leg: decode each vendor stream through the full chain
/// (§1 assembly → §2–§4 parse → §5 mid/side + calibrated
/// dequantisation → variable-size lapped reconstruction) and compare
/// a mono downmix against a black-box reference decode (best-lag +
/// scalar-gain fit; with the calibrated absolute scale the fitted
/// gain converges to ≈ 1, which the closing envelope-coded families
/// pin below).
///
/// The r450 calibration measured here (sweep over composition
/// candidates, this fit as the score):
/// * neighbour-matched variable-size lapped reconstruction replaces
///   the truncation-aligned overlap-add — the three fully-closing
///   44.1/22.05 kHz families move from ≈ 3 / ≈ 0 dB to ≈ 11–15 dB;
/// * total gain at 1 dB/step (`10^((g − 64) / 20)`) instead of the
///   ladder's 1.25 dB/step — those families then reach ≈ 18–27 dB
///   (the 1/16 and 1/32 exponents both lose ≥ 9 dB);
/// * the envelope stays on the staged `10^((e − e_max)/16)` ladder
///   ratio anchored at the block's maximum exponent (a fixed anchor
///   loses ≥ 10 dB).
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
        let mut clean_pad = false;
        for (i, rec) in stream.packets.iter().enumerate() {
            if cursor != rec.frames_start_bit() {
                cursor = rec.frames_start_bit();
                parser.raise_latch();
                if !clean_pad {
                    // A real mis-parse (not a padding skip): the
                    // overlap-add state is unreliable.
                    synth.reset();
                }
            }
            let mut reader = stream.reader_at(cursor);
            let mut failed = false;
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
                        failed = true;
                        break;
                    }
                }
            }
            cursor = reader.position() as u64;
            // A zero-carry successor after a clean parse means this
            // packet's tail is padding: the coming cursor jump is
            // *not* a decode discontinuity.
            clean_pad = !failed
                && stream
                    .packets
                    .get(i + 1)
                    .is_some_and(|n| n.header.carry_bits == 0 && cursor <= n.body_start_bit);
        }
        assert!(!pcm.is_empty(), "{}: no PCM produced", l.spec.file);
        assert!(
            pcm.iter().all(|x| x.is_finite()),
            "{}: non-finite PCM",
            l.spec.file
        );

        // Best-lag + gain fit over a middle window. The decoder's
        // leading latency is block-aligned, so only block-aligned
        // lags are candidates (a free election drifts onto spurious
        // correlation peaks when the residual error is large).
        let sr = l.spec.sample_rate as usize;
        let win = (sr / 2).min(pcm.len().saturating_sub(1));
        let start = (reference.len() / 3).min(reference.len().saturating_sub(win + 1));
        let mut best = (0i64, f64::NEG_INFINITY, 0.0f64);
        for lag in (-5120i64..=5120).step_by(256) {
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

        // Floors at the measured r450 quality (the calibrated
        // composition + variable-size lapped reconstruction), with
        // headroom for float drift. The fitted-gain ≈ 1 pin holds on
        // the fully-closing envelope-coded families because the
        // absolute scale is now part of the decode
        // (`vendor_decode::ABS_SCALE`).
        match l.spec.file {
            "cand_apollo8.wma" => {
                assert!(corr2 > 0.9, "apollo8 corr² regressed: {corr2}");
            }
            "cand_mono8k_8kbps_v8.wma" => {
                // LSP envelope path (conversion tables unstaged):
                // flat-envelope decode.
                assert!(
                    median > 4.0,
                    "{}: median SNR regressed to {median:.2} dB",
                    l.spec.file
                );
            }
            "cand_stereo22k_32kbps_av.wma" => {
                assert!(corr2 > 0.98, "corr² regressed: {corr2}");
                assert!(median > 24.0, "median SNR regressed to {median:.2} dB");
                assert!(
                    (0.7..1.4).contains(&gain),
                    "fitted gain {gain} strayed from 1"
                );
            }
            "cand_wmp12_96kbps.wma" => {
                assert!(corr2 > 0.98, "corr² regressed: {corr2}");
                assert!(median > 15.0, "median SNR regressed to {median:.2} dB");
                assert!(
                    (0.7..1.4).contains(&gain),
                    "fitted gain {gain} strayed from 1"
                );
            }
            "cand_vbr_q75_stereo.wma" => {
                assert!(corr2 > 0.98, "corr² regressed: {corr2}");
                assert!(median > 17.0, "median SNR regressed to {median:.2} dB");
                assert!(
                    (0.7..1.4).contains(&gain),
                    "fitted gain {gain} strayed from 1"
                );
            }
            _ => {}
        }
    }
}

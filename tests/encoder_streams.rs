//! Encoder end-to-end validation — the r454 encoder mirror measured
//! two ways on each supported stream family:
//!
//! 1. **Self-consistency**: encode synthetic material, decode with
//!    this crate's own vendor chain, assert per-family SNR floors at
//!    the chain's fixed `frame_length / 2` lead-in.
//! 2. **Wire-format acceptance**: wrap the emitted `block_align`
//!    codec packets in a minimal RIFF/WAVE container (the
//!    `WAVEFORMATEX` framing the staged docs point to for
//!    WMA-outside-ASF) and hand the file to the black-box reference
//!    decoder (`ffmpeg` binary, opaque oracle). Acceptance — the
//!    reference parses the stream and produces PCM that correlates
//!    with the input at fitted gain ≈ 1 — is the bar; bit-parity
//!    with a vendor encoder is not claimed. Skipped when the binary
//!    is unavailable.
//!
//! The WAV path itself is validated against the reference decoder's
//! own ASF unwrap in `tests/vendor_streams.rs` fixtures territory:
//! remuxing a committed vendor stream ASF → WAV decodes
//! byte-identically, so WAV acceptance is equivalent to codec-level
//! acceptance.

use std::process::Command;

use oxideav_wma::header::Version;
use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_decode::BlockSynth;
use oxideav_wma::vendor_frame::FrameParser;
use oxideav_wma::VendorEncoder;

/// One encoder family under test.
struct Family {
    name: &'static str,
    cfg: StreamConfig,
    /// v2 `WAVEFORMATEX` extradata tail (10 bytes; flags2 at [4..6]).
    extradata: [u8; 10],
    /// Own-chain SNR floor, dB.
    own_floor: f64,
    /// Black-box reference SNR floor, dB.
    reference_floor: f64,
}

fn families() -> Vec<Family> {
    let mk_extra = |flags2: u16| -> [u8; 10] {
        let mut e = [0u8; 10];
        e[4..6].copy_from_slice(&flags2.to_le_bytes());
        e
    };
    vec![
        Family {
            // The ACM catalogue's headerless geometry (format 17):
            // one frame per 186-byte packet, no reservoir — the
            // vendor's own extradata bytes for this configuration.
            name: "stereo22k_32kbps_headerless",
            cfg: StreamConfig::derive(Version::V2, 22_050, 2, 4005, 186, 0x0001).unwrap(),
            extradata: *b"\x00\x04\x00\x00\x01\x00\xba\x00\x00\x00",
            own_floor: 5.0,
            reference_floor: 3.0,
        },
        Family {
            // The staged cand_stereo22k geometry: stereo VBL + reservoir.
            name: "stereo22k_32kbps_vbl",
            cfg: StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap(),
            extradata: mk_extra(0x0017),
            own_floor: 8.0,
            reference_floor: 5.0,
        },
        Family {
            // The staged cand_mono22k geometry: mono VBL + reservoir.
            name: "mono22k_16kbps_vbl",
            cfg: StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap(),
            extradata: mk_extra(0x000f),
            own_floor: 6.0,
            reference_floor: 4.0,
        },
        Family {
            // The staged cand_wmp12 geometry: stereo 44.1 kHz 96 kbps.
            name: "stereo44k_96kbps_vbl",
            cfg: StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 4459, 0x000f).unwrap(),
            extradata: mk_extra(0x000f),
            own_floor: 14.0,
            reference_floor: 10.0,
        },
    ]
}

/// Band-limited synthetic material: inharmonic tones with slow
/// amplitude drift plus a soft click every ~0.7 s (block-schedule
/// exercise).
fn material(sample_rate: u32, len: usize, seed: u64) -> Vec<f64> {
    let mut out = vec![0.0f64; len];
    let freqs = [211.0, 487.0, 1021.0, 2333.0];
    for (i, f) in freqs.iter().enumerate() {
        let amp = 0.12 / (i + 1) as f64;
        let phase = (seed as f64) * 0.37 + i as f64;
        for (t, o) in out.iter_mut().enumerate() {
            let x = t as f64 / f64::from(sample_rate);
            let drift = 1.0 + 0.3 * (2.0 * std::f64::consts::PI * 0.7 * x + phase).sin();
            *o += amp * drift * (2.0 * std::f64::consts::PI * f * x + phase).sin();
        }
    }
    let click_period = (sample_rate as usize) * 7 / 10;
    for (t, o) in out.iter_mut().enumerate() {
        if t % click_period < 40 {
            *o += 0.3 * (((t * 13) % 7) as f64 / 3.5 - 1.0);
        }
    }
    out
}

fn encode_family(fam: &Family, pcm: &[Vec<f64>]) -> Vec<Vec<u8>> {
    let mut enc = VendorEncoder::new(&fam.cfg).unwrap();
    enc.push(pcm).unwrap();
    enc.finish().unwrap()
}

/// Decode with the crate's own chain (the vendor harness loop).
fn decode_own(cfg: &StreamConfig, packets: &[Vec<u8>]) -> Vec<Vec<f64>> {
    let mut asm = PacketAssembler::new(cfg);
    for p in packets {
        asm.push_packet(p).unwrap();
    }
    let stream = asm.finish();
    let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
    let mut parser = FrameParser::new(cfg, &body_starts);
    let mut synth = BlockSynth::new(cfg);
    let mut pcm: Vec<Vec<f64>> = vec![Vec::new(); usize::from(cfg.channels)];
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
                for (ch, chan) in synth.block(block).into_iter().enumerate() {
                    pcm[ch].extend_from_slice(&chan);
                }
            }
        }
        cursor = reader.position() as u64;
    }
    for (ch, chan) in synth.flush().into_iter().enumerate() {
        pcm[ch].extend_from_slice(&chan);
    }
    pcm
}

/// SNR at the chain's fixed `frame_length / 2` lead-in.
fn snr_at_lead(cfg: &StreamConfig, original: &[f64], decoded: &[f64]) -> f64 {
    let lead = usize::from(cfg.frame_length) / 2;
    let n = original.len().min(decoded.len().saturating_sub(lead));
    let (mut sig, mut err) = (0.0f64, 0.0f64);
    for (t, &a) in original.iter().enumerate().take(n) {
        let b = decoded[t + lead];
        sig += a * a;
        err += (a - b) * (a - b);
    }
    10.0 * (sig / err.max(1e-30)).log10()
}

/// Minimal RIFF/WAVE wrapper around the codec packets: a
/// `WAVEFORMATEX` fmt chunk (tag 0x0161, `cbSize` 10 extradata tail)
/// and the packets as the data chunk.
fn wav_wrap(cfg: &StreamConfig, extradata: &[u8; 10], packets: &[Vec<u8>]) -> Vec<u8> {
    let data_len: usize = packets.iter().map(|p| p.len()).sum();
    let fmt_len = 18 + extradata.len();
    let riff_len = 4 + (8 + fmt_len) + (8 + data_len);
    let mut out = Vec::with_capacity(8 + riff_len);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(riff_len as u32).to_le_bytes());
    out.extend_from_slice(b"WAVE");
    out.extend_from_slice(b"fmt ");
    out.extend_from_slice(&(fmt_len as u32).to_le_bytes());
    out.extend_from_slice(&0x0161u16.to_le_bytes()); // wFormatTag
    out.extend_from_slice(&u16::from(cfg.channels).to_le_bytes());
    out.extend_from_slice(&cfg.sample_rate.to_le_bytes());
    out.extend_from_slice(&cfg.avg_bytes_per_sec.to_le_bytes());
    out.extend_from_slice(&cfg.block_align.to_le_bytes());
    out.extend_from_slice(&16u16.to_le_bytes()); // wBitsPerSample
    out.extend_from_slice(&(extradata.len() as u16).to_le_bytes()); // cbSize
    out.extend_from_slice(extradata);
    out.extend_from_slice(b"data");
    out.extend_from_slice(&(data_len as u32).to_le_bytes());
    for p in packets {
        out.extend_from_slice(p);
    }
    out
}

/// Black-box reference decode of a WAV file to mono f32le PCM.
fn reference_decode_mono(wav: &[u8]) -> Option<Vec<f64>> {
    let dir = std::env::temp_dir().join(format!("oxideav-wma-encoder-e2e-{}", std::process::id()));
    std::fs::create_dir_all(&dir).ok()?;
    let path = dir.join("probe.wav");
    std::fs::write(&path, wav).ok()?;
    let out = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(&path)
        .args(["-f", "f32le", "-ac", "1", "-"])
        .output()
        .ok()?;
    let _ = std::fs::remove_file(&path);
    if !out.status.success() || out.stdout.is_empty() {
        eprintln!(
            "reference decoder rejected the stream: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        return None;
    }
    Some(
        out.stdout
            .chunks_exact(4)
            .map(|c| f64::from(f32::from_le_bytes([c[0], c[1], c[2], c[3]])))
            .collect(),
    )
}

/// Best block-aligned lag + scalar gain fit of `decoded` against
/// `original`; returns `(lag, corr2, gain, snr_db)`.
fn fit(original: &[f64], decoded: &[f64]) -> (i64, f64, f64, f64) {
    let n = original.len();
    let mut best = (0i64, f64::NEG_INFINITY, 0.0f64);
    for lag in (-4096i64..=8192).step_by(64) {
        let (mut dot, mut ee, mut rr) = (0.0, 0.0, 0.0);
        for (t, &a) in original.iter().enumerate().take(n).step_by(4) {
            let u = t as i64 + lag;
            if u < 0 || u as usize >= decoded.len() {
                continue;
            }
            let b = decoded[u as usize];
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
    let (mut sig, mut err) = (0.0f64, 0.0f64);
    for (t, &a) in original.iter().enumerate().take(n) {
        let u = t as i64 + lag;
        let b = if u >= 0 && (u as usize) < decoded.len() {
            decoded[u as usize] * gain
        } else {
            0.0
        };
        sig += a * a;
        err += (a - b) * (a - b);
    }
    (lag, corr2, gain, 10.0 * (sig / err.max(1e-30)).log10())
}

fn family_pcm(fam: &Family) -> Vec<Vec<f64>> {
    let sr = fam.cfg.sample_rate;
    let len = (sr as usize) * 3;
    let left = material(sr, len, 1);
    if fam.cfg.channels == 2 {
        let right: Vec<f64> = material(sr, len, 2)
            .iter()
            .zip(left.iter())
            .map(|(r, l)| 0.6 * l + 0.4 * r)
            .collect();
        vec![left, right]
    } else {
        vec![left]
    }
}

/// Leg 1: every family encodes and decodes through the crate's own
/// chain above its floor; the §1 layer holds packet-for-packet.
#[test]
fn encoded_families_decode_through_the_own_chain() {
    for fam in &families() {
        let pcm = family_pcm(fam);
        let packets = encode_family(fam, &pcm);
        assert!(
            packets
                .iter()
                .all(|p| p.len() == usize::from(fam.cfg.block_align)),
            "{}: packet sizing",
            fam.name
        );
        let decoded = decode_own(&fam.cfg, &packets);
        let mut worst = f64::INFINITY;
        for (ch, orig) in pcm.iter().enumerate() {
            let snr = snr_at_lead(&fam.cfg, orig, &decoded[ch]);
            worst = worst.min(snr);
        }
        let bytes: usize = packets.iter().map(|p| p.len()).sum();
        let nominal = fam.cfg.avg_bytes_per_sec as f64 * 3.0;
        eprintln!(
            "{}: own-chain SNR {worst:.2} dB, {} packets, {bytes} bytes ({}% of nominal)",
            fam.name,
            packets.len(),
            (bytes as f64 / nominal * 100.0).round()
        );
        assert!(
            worst > fam.own_floor,
            "{}: own-chain SNR {worst:.2} dB below floor {}",
            fam.name,
            fam.own_floor
        );
        // Rate sanity: the emitted stream stays within 2.5x nominal.
        assert!(
            (bytes as f64) < 2.5 * nominal + 3.0 * f64::from(fam.cfg.block_align),
            "{}: {bytes} bytes vs nominal {nominal}",
            fam.name
        );
    }
}

/// Leg 2: the black-box reference decoder accepts the emitted wire
/// format (WMA-in-WAV) and decodes it to PCM correlating with the
/// input at fitted gain ≈ 1.
#[test]
fn black_box_reference_accepts_the_emitted_wire_format() {
    // Probe for the reference binary once.
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        eprintln!("skipping: reference decoder unavailable");
        return;
    }
    for fam in &families() {
        let pcm = family_pcm(fam);
        let packets = encode_family(fam, &pcm);
        let wav = wav_wrap(&fam.cfg, &fam.extradata, &packets);
        let reference = match reference_decode_mono(&wav) {
            Some(r) => r,
            None => panic!("{}: reference decoder rejected the stream", fam.name),
        };
        // Mono mix of the input for the fit.
        let mix: Vec<f64> = (0..pcm[0].len())
            .map(|t| pcm.iter().map(|c| c[t]).sum::<f64>() / pcm.len() as f64)
            .collect();
        let (lag, corr2, gain, snr) = fit(&mix, &reference);
        eprintln!(
            "{}: reference decode {} samples, lag {lag}, corr2 {corr2:.3}, gain {gain:.3}, SNR {snr:.2} dB",
            fam.name,
            reference.len()
        );
        assert!(
            reference.len() * 2 > pcm[0].len(),
            "{}: reference produced too few samples",
            fam.name
        );
        assert!(corr2 > 0.8, "{}: corr2 {corr2:.3}", fam.name);
        assert!(
            (0.4..2.5).contains(&gain),
            "{}: fitted gain {gain:.3} strayed from 1",
            fam.name
        );
        assert!(
            snr > fam.reference_floor,
            "{}: reference SNR {snr:.2} dB below floor {}",
            fam.name,
            fam.reference_floor
        );
    }
}

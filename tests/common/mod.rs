//! Shared helpers for the encoder acceptance suites: synthetic
//! material, the crate's own decode chain, the RIFF/`WAVEFORMATEX`
//! wrap the black-box reference decoder reads, the reference decode
//! itself, and the lag/gain/SNR fit.
#![allow(dead_code)]

use std::process::Command;

use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_decode::BlockSynth;
use oxideav_wma::vendor_frame::FrameParser;

/// Band-limited synthetic material: inharmonic tones with slow
/// amplitude drift plus a soft click every ~0.7 s (block-schedule
/// exercise).
pub fn material(sample_rate: u32, len: usize, seed: u64) -> Vec<f64> {
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

/// Decode with the crate's own chain (the vendor harness loop).
pub fn decode_own(cfg: &StreamConfig, packets: &[Vec<u8>]) -> Vec<Vec<f64>> {
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
pub fn snr_at_lead(cfg: &StreamConfig, original: &[f64], decoded: &[f64]) -> f64 {
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
pub fn wav_wrap(cfg: &StreamConfig, extradata: &[u8; 10], packets: &[Vec<u8>]) -> Vec<u8> {
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

/// Black-box reference decode of a WAV file to per-channel f32 PCM.
pub fn reference_decode(wav: &[u8], channels: usize) -> Option<Vec<Vec<f64>>> {
    let dir = std::env::temp_dir().join(format!("oxideav-wma-encoder-e2e-{}", std::process::id()));
    std::fs::create_dir_all(&dir).ok()?;
    let path = dir.join("probe.wav");
    std::fs::write(&path, wav).ok()?;
    let out = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(&path)
        .args(["-f", "f32le", "-ac", &channels.to_string(), "-"])
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
    let mut per = vec![Vec::new(); channels];
    for (i, c) in out.stdout.chunks_exact(4).enumerate() {
        per[i % channels].push(f64::from(f32::from_le_bytes([c[0], c[1], c[2], c[3]])));
    }
    Some(per)
}

/// Best lag (block-aligned coarse search, then sample-exact) +
/// scalar gain fit of `decoded` against `original`; returns
/// `(lag, corr2, gain, snr_db)`. The SNR is taken over the interior
/// of the overlap (2048 samples off either edge), so a decoder that
/// emits a shorter tail is not charged for it.
pub fn fit(original: &[f64], decoded: &[f64]) -> (i64, f64, f64, f64) {
    let n = original.len();
    let score = |lag: i64, step: usize| -> (f64, f64) {
        let (mut dot, mut ee, mut rr) = (0.0, 0.0, 0.0);
        for (t, &a) in original.iter().enumerate().take(n).step_by(step) {
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
            return (f64::NEG_INFINITY, 0.0);
        }
        (dot * dot / (ee * rr), dot / ee)
    };
    let mut best = (0i64, f64::NEG_INFINITY, 0.0f64);
    for lag in (-8192i64..=8192).step_by(64) {
        let (c, g) = score(lag, 8);
        if c > best.1 {
            best = (lag, c, g);
        }
    }
    let centre = best.0;
    for lag in centre - 96..=centre + 96 {
        let (c, g) = score(lag, 1);
        if c > best.1 {
            best = (lag, c, g);
        }
    }
    let (lag, corr2, gain) = best;
    let (mut sig, mut err) = (0.0f64, 0.0f64);
    for (t, &a) in original.iter().enumerate().take(n) {
        let u = t as i64 + lag;
        if u < 2048 || (u as usize) + 2048 >= decoded.len() || t < 2048 || t + 2048 >= n {
            continue;
        }
        let b = decoded[u as usize] * gain;
        sig += a * a;
        err += (a - b) * (a - b);
    }
    (lag, corr2, gain, 10.0 * (sig / err.max(1e-30)).log10())
}

/// Whether the black-box reference binary is available.
pub fn reference_available() -> bool {
    Command::new("ffmpeg").arg("-version").output().is_ok()
}

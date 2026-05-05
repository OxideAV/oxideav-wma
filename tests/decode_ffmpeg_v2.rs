//! Same pipeline as `decode_ffmpeg_v1` but for WMA v2 (codec tag
//! `0x0161`) — exercises the v2-specific exponent init (hard-coded
//! `last_exp = 36`, no 5-bit prefix), v2 escape coefficients
//! (8/16/24/31-bit progressive "large value"), and v2 band partition
//! tables.

use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};
use oxideav_wma::{asf, register};
use std::process::Command;

fn have_tool(name: &str) -> bool {
    Command::new(name)
        .arg("-version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn run_ffmpeg(args: &[&str]) {
    let status = Command::new("ffmpeg")
        .args(args)
        .status()
        .expect("ffmpeg failed to spawn");
    assert!(status.success(), "ffmpeg failed for args: {:?}", args);
}

fn extract_wma_frames(path: &std::path::Path) -> Vec<Vec<u8>> {
    let output = Command::new("ffprobe")
        .args([
            "-i",
            path.to_str().unwrap(),
            "-show_packets",
            "-show_data",
            "-of",
            "json",
            "-loglevel",
            "error",
        ])
        .output()
        .expect("ffprobe spawn");
    assert!(output.status.success(), "ffprobe failed");
    let s = String::from_utf8(output.stdout).expect("ffprobe stdout utf8");
    let mut frames = Vec::new();
    for line in s.lines() {
        let line = line.trim_end_matches(',').trim();
        if let Some(rest) = line.strip_prefix("\"data\":") {
            let inner = rest.trim().trim_matches('"');
            let bytes = parse_hex_dump_lines(inner.split("\\n"));
            frames.push(bytes);
        }
    }
    frames
}

fn parse_hex_dump_lines<'a, I: Iterator<Item = &'a str>>(lines: I) -> Vec<u8> {
    let mut out = Vec::new();
    for line in lines {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(rest) = line.split_once(':').map(|x| x.1) {
            let mut hex = String::new();
            for tok in rest.split_whitespace() {
                if tok.chars().all(|c| c.is_ascii_hexdigit()) {
                    hex.push_str(tok);
                } else {
                    break;
                }
            }
            if let Ok(b) = hex::decode(&hex) {
                out.extend_from_slice(&b);
            }
        }
    }
    out
}

fn psnr_db(a: &[f32], b: &[f32]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let mut acc: f64 = 0.0;
    for i in 0..n {
        let d = a[i] as f64 - b[i] as f64;
        acc += d * d;
    }
    let mse = acc / n as f64;
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (1.0f64 / mse).log10()
    }
}

#[test]
fn wmav2_roundtrip_pipeline_ok() {
    if !have_tool("ffmpeg") || !have_tool("ffprobe") {
        eprintln!("skipping: ffmpeg/ffprobe not on PATH");
        return;
    }

    let tmp = tempdir_path("wma_v2_psnr");
    let wma_path = tmp.join("mono_v2_22k.wma");
    let ref_path = tmp.join("mono_22k_ref.f32");

    run_ffmpeg(&[
        "-y",
        "-f",
        "lavfi",
        "-i",
        "sine=frequency=440:duration=0.5:sample_rate=22050",
        "-c:a",
        "wmav2",
        "-b:a",
        "32k",
        wma_path.to_str().unwrap(),
    ]);
    run_ffmpeg(&[
        "-y",
        "-f",
        "lavfi",
        "-i",
        "sine=frequency=440:duration=0.5:sample_rate=22050",
        "-c:a",
        "pcm_f32le",
        "-f",
        "f32le",
        ref_path.to_str().unwrap(),
    ]);

    let asf_bytes = std::fs::read(&wma_path).expect("read wma");
    let asf_file = asf::parse(&asf_bytes).expect("parse asf");
    let wfe = &asf_file.waveformatex;
    assert_eq!(wfe.format_tag, 0x0161);
    assert_eq!(wfe.channels, 1);

    let frames = extract_wma_frames(&wma_path);
    assert!(!frames.is_empty(), "ffprobe extracted zero frames");
    eprintln!(
        "ffprobe extracted {} WMA v2 frames; first frame len = {}",
        frames.len(),
        frames[0].len()
    );

    let mut reg = oxideav_core::CodecRegistry::new();
    register(&mut reg);
    let mut params = CodecParameters::audio(CodecId::new("wmav2"));
    params.sample_rate = Some(wfe.sample_rate);
    params.channels = Some(wfe.channels);
    params.bit_rate = Some((wfe.avg_bytes_per_sec as u64) * 8);
    params.extradata = wfe.extradata.clone();
    let mut dec = reg.first_decoder(&params).expect("make wmav2 decoder");

    let mut decoded: Vec<f32> = Vec::new();
    for (i, frame_bytes) in frames.iter().enumerate() {
        let pkt = Packet::new(
            0,
            TimeBase::new(1, wfe.sample_rate as i64),
            frame_bytes.clone(),
        );
        if dec.send_packet(&pkt).is_err() {
            continue;
        }
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    if !af.data.is_empty() {
                        let plane = &af.data[0];
                        for chunk in plane.chunks_exact(4) {
                            decoded
                                .push(f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
                        }
                    }
                }
                Ok(_) => {}
                Err(oxideav_core::Error::NeedMore) => break,
                Err(e) => {
                    eprintln!("frame {i}: {e:?}");
                    break;
                }
            }
        }
    }

    let reference_bytes = std::fs::read(&ref_path).expect("read ref pcm");
    let reference: Vec<f32> = reference_bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect();

    assert!(
        decoded.len() >= 1024,
        "decoder produced too few samples: {}",
        decoded.len()
    );

    let skip = 2048usize;
    let n = decoded.len().saturating_sub(skip).min(reference.len());
    if n == 0 {
        eprintln!("post-skip empty");
        return;
    }
    let dec_slice = &decoded[skip..skip + n];
    let ref_slice = &reference[..n];
    let dec_peak = dec_slice.iter().fold(0f32, |a, &b| a.max(b.abs()));
    let ref_peak = ref_slice.iter().fold(0f32, |a, &b| a.max(b.abs()));
    let scale = if dec_peak > 0.0 {
        ref_peak / dec_peak
    } else {
        1.0
    };
    let dec_normalised: Vec<f32> = dec_slice.iter().map(|&v| v * scale).collect();
    let psnr = psnr_db(ref_slice, &dec_normalised);
    eprintln!(
        "wmav2 22k mono PSNR over {n} samples (post-norm): {psnr:.2} dB \
         (raw_dec_peak={dec_peak:.4}, scale={scale:.4})"
    );
    assert!(
        psnr.is_finite() && psnr >= 8.0,
        "PSNR too low ({psnr:.2} dB)"
    );
}

fn tempdir_path(tag: &str) -> std::path::PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!("oxideav-wma-{tag}-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&p);
    p
}

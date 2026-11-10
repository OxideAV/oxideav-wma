//! Conformance vectors — the nine LSP-path WMA v2 streams under
//! `docs/audio/wma/reference/sandbox-encoded/` (vendor-encoded from
//! synthetic signals, each with the vendor decoder's complete PCM
//! output as `expected.pcm`; `provenance/12`).
//!
//! The fixtures are **not** copied into this repo: the tests locate
//! them via `OXIDEAV_WMA_SANDBOX_ENCODED_DIR` or the umbrella
//! workspace layout and unwrap the ASF container with a black-box
//! validator invocation (`ffmpeg -c copy`, whose output is checked
//! against the codec-packet size of every fixture). When the
//! fixtures or the binary are absent the tests skip.

mod common;

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_wma::header::Version;
use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_decode::BlockSynth;
use oxideav_wma::vendor_frame::FrameParser;

/// One fixture's container configuration (the fixture README table).
struct Fixture {
    dir: &'static str,
    sample_rate: u32,
    channels: u8,
    avg_bytes_per_sec: u32,
    block_align: u16,
    flags2: u16,
    packets: usize,
}

const FIXTURES: [Fixture; 9] = [
    Fixture {
        dir: "v2-8k-mono-5kbps-lsp-noise",
        sample_rate: 8000,
        channels: 1,
        avg_bytes_per_sec: 625,
        block_align: 40,
        flags2: 0x0000,
        packets: 95,
    },
    Fixture {
        dir: "v2-8k-mono-6kbps-lsp-noise",
        sample_rate: 8000,
        channels: 1,
        avg_bytes_per_sec: 750,
        block_align: 48,
        flags2: 0x0000,
        packets: 95,
    },
    Fixture {
        dir: "v2-16k-mono-12kbps-lsp-vbl",
        sample_rate: 16_000,
        channels: 1,
        avg_bytes_per_sec: 1500,
        block_align: 384,
        flags2: 0x0006,
        packets: 20,
    },
    Fixture {
        dir: "v2-22k-mono-16kbps-lsp-vbl",
        sample_rate: 22_050,
        channels: 1,
        avg_bytes_per_sec: 2002,
        block_align: 595,
        flags2: 0x000e,
        packets: 16,
    },
    Fixture {
        dir: "v2-8k-stereo-12kbps-lsp-lr",
        sample_rate: 8000,
        channels: 2,
        avg_bytes_per_sec: 1500,
        block_align: 96,
        flags2: 0x0000,
        packets: 95,
    },
    Fixture {
        dir: "v2-8k-stereo-12kbps-lsp-ms",
        sample_rate: 8000,
        channels: 2,
        avg_bytes_per_sec: 1500,
        block_align: 96,
        flags2: 0x0000,
        packets: 95,
    },
    Fixture {
        dir: "v2-16k-stereo-16kbps-lsp-vbl-lr",
        sample_rate: 16_000,
        channels: 2,
        avg_bytes_per_sec: 2004,
        block_align: 342,
        flags2: 0x000e,
        packets: 31,
    },
    Fixture {
        dir: "v2-16k-stereo-20kbps-lsp-vbl-lr",
        sample_rate: 16_000,
        channels: 2,
        avg_bytes_per_sec: 2502,
        block_align: 427,
        flags2: 0x0016,
        packets: 31,
    },
    Fixture {
        dir: "v2-16k-stereo-16kbps-lsp-vbl-ms",
        sample_rate: 16_000,
        channels: 2,
        avg_bytes_per_sec: 2004,
        block_align: 342,
        flags2: 0x000e,
        packets: 31,
    },
];

fn fixtures_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("OXIDEAV_WMA_SANDBOX_ENCODED_DIR") {
        let p = PathBuf::from(dir);
        return p.is_dir().then_some(p);
    }
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/audio/wma/reference/sandbox-encoded");
    p.is_dir().then_some(p)
}

/// Black-box ASF unwrap: the audio stream's codec packets.
fn extract_packets(path: &Path) -> Option<Vec<u8>> {
    let out = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(path)
        .args(["-map", "0:a:0", "-c", "copy", "-f", "data", "-"])
        .output()
        .ok()?;
    out.status.success().then_some(out.stdout)
}

struct Loaded {
    fx: &'static Fixture,
    cfg: StreamConfig,
    packets: Vec<Vec<u8>>,
    /// `expected.pcm` de-interleaved per channel.
    expected: Vec<Vec<i16>>,
}

fn load_all() -> Option<Vec<Loaded>> {
    let dir = fixtures_dir()?;
    let mut out = Vec::new();
    for fx in &FIXTURES {
        let d = dir.join(fx.dir);
        let raw = extract_packets(&d.join("input.wma"))?;
        let ba = usize::from(fx.block_align);
        assert_eq!(raw.len(), ba * fx.packets, "{}: packet bytes", fx.dir);
        let packets = raw.chunks_exact(ba).map(<[u8]>::to_vec).collect();
        let pcm = std::fs::read(d.join("expected.pcm")).ok()?;
        let ch = usize::from(fx.channels);
        let mut expected = vec![Vec::new(); ch];
        for (i, s) in pcm.as_chunks::<2>().0.iter().enumerate() {
            expected[i % ch].push(i16::from_le_bytes(*s));
        }
        let cfg = StreamConfig::derive(
            Version::V2,
            fx.sample_rate,
            fx.channels,
            fx.avg_bytes_per_sec,
            fx.block_align,
            fx.flags2,
        )
        .expect("catalogue configurations derive");
        out.push(Loaded {
            fx,
            cfg,
            packets,
            expected,
        });
    }
    Some(out)
}

macro_rules! skip_unless_fixtures {
    () => {
        match load_all() {
            Some(loaded) => loaded,
            None => {
                eprintln!(
                    "skipping: sandbox-encoded fixtures or ffmpeg unavailable \
                     (set OXIDEAV_WMA_SANDBOX_ENCODED_DIR to run)"
                );
                return;
            }
        }
    };
}

/// Decode with the crate's chain; every frame must parse.
fn decode(l: &Loaded, pcm16: bool) -> (Vec<Vec<f64>>, usize) {
    let cfg = &l.cfg;
    let mut asm = PacketAssembler::new(cfg);
    for p in &l.packets {
        asm.push_packet(p).expect("packet layer");
    }
    let stream = asm.finish();
    let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
    let mut parser = FrameParser::new(cfg, &body_starts);
    let mut synth = BlockSynth::new(cfg).with_pcm16_output(pcm16);
    let mut pcm: Vec<Vec<f64>> = vec![Vec::new(); usize::from(cfg.channels)];
    let mut cursor = stream.packets[0].frames_start_bit();
    let mut frames = 0usize;
    for (i, rec) in stream.packets.iter().enumerate() {
        if cursor != rec.frames_start_bit() {
            cursor = rec.frames_start_bit();
            parser.raise_latch();
        }
        let mut reader = stream.reader_at(cursor);
        for f in 0..rec.header.frame_count {
            let frame = parser
                .parse_frame(&mut reader)
                .unwrap_or_else(|e| panic!("{} packet {i} frame {f}: {e:?}", l.fx.dir));
            frames += 1;
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
    (pcm, frames)
}

/// Per-fixture comparison against `expected.pcm`.
#[derive(Debug, Default)]
struct Score {
    total: usize,
    exact: usize,
    max_abs: i32,
    /// Fitted least-squares gain of ours onto expected.
    gain: f64,
}

/// Compare at the vendor's start-up alignment: its output is this
/// chain's output from sample `frame_length` on (the vendor decoder
/// withholds its first frame: the first packet returns no audio, and
/// the half-frame before the first block's left slope never
/// completes); it holds whole frames, at most `frames − 1` of them
/// (on the reservoir streams it stops one frame earlier still).
fn score(l: &Loaded, (pcm, frames): &(Vec<Vec<f64>>, usize)) -> Score {
    let lag = usize::from(l.cfg.frame_length);
    let n = l.expected[0].len();
    assert!(
        n.is_multiple_of(lag) && n <= (frames - 1) * lag,
        "{}: expected length {n} vs {frames} frames",
        l.fx.dir
    );
    let mut s = Score::default();
    let (mut dot, mut ee) = (0.0f64, 0.0f64);
    for (e, p) in l.expected.iter().zip(pcm.iter()) {
        assert!(p.len() >= n + lag, "{}: decoded too short", l.fx.dir);
        for (t, &want) in e.iter().enumerate() {
            let x = p[t + lag] * 32768.0;
            let ours = x.round_ties_even().clamp(-32768.0, 32767.0) as i32;
            let d = (ours - i32::from(want)).abs();
            s.total += 1;
            if d == 0 {
                s.exact += 1;
            }
            s.max_abs = s.max_abs.max(d);
            dot += x * f64::from(want);
            ee += x * x;
        }
    }
    s.gain = if ee > 0.0 { dot / ee } else { 0.0 };
    s
}

/// The vendor's 16-bit output stage: every fixture reproduces the
/// vendor PCM to within one LSB, bit-exactly on ≥ 99.8 % of samples
/// (r485: 962 869 / 963 584 = 99.926 % over all nine; the
/// residual is the inverse transform's own arithmetic, gap G10).
#[test]
fn vendor_pcm_reproduced_by_the_16_bit_output_stage() {
    let loaded = skip_unless_fixtures!();
    let (mut all_exact, mut all_total) = (0usize, 0usize);
    for l in &loaded {
        let s = score(l, &decode(l, true));
        eprintln!(
            "{:34} exact {:>6}/{:>6} ({:.3} %)  max |err| {} LSB  gain {:.7}",
            l.fx.dir,
            s.exact,
            s.total,
            100.0 * s.exact as f64 / s.total as f64,
            s.max_abs,
            s.gain
        );
        assert!(s.max_abs <= 1, "{}: max |err| {} LSB", l.fx.dir, s.max_abs);
        assert!(
            s.exact * 1000 >= s.total * 998,
            "{}: only {}/{} samples bit-exact",
            l.fx.dir,
            s.exact,
            s.total
        );
        all_exact += s.exact;
        all_total += s.total;
    }
    eprintln!(
        "all fixtures: {all_exact}/{all_total} bit-exact ({:.3} %)",
        100.0 * all_exact as f64 / all_total as f64
    );
}

/// The default float output stage: unit gain onto the vendor PCM (the
/// absolute scale is the 16-bit full scale, sign inverted — no fitted
/// constant), every sample within one LSB, and the residual the
/// single-rounding triangle (≈ 75 % exact).
#[test]
fn float_output_matches_the_vendor_scale() {
    let loaded = skip_unless_fixtures!();
    for l in &loaded {
        let s = score(l, &decode(l, false));
        eprintln!(
            "{:34} float stage: exact {}/{} max |err| {} gain {:.7}",
            l.fx.dir, s.exact, s.total, s.max_abs, s.gain
        );
        assert!((s.gain - 1.0).abs() < 1e-5, "{}: gain {}", l.fx.dir, s.gain);
        assert!(s.max_abs <= 1, "{}: max |err| {}", l.fx.dir, s.max_abs);
        assert!(
            s.exact * 10 >= s.total * 7,
            "{}: exact {}",
            l.fx.dir,
            s.exact
        );
    }
}

/// Long-term spectral power per bin (512-bin lapped analysis, sine
/// window), summed over every frame of `x`.
fn power_spectrum(x: &[f64]) -> Vec<f64> {
    use oxideav_wma::block::BlockSize;
    use oxideav_wma::mlt::Mlt;
    let m = 512usize;
    let mlt = Mlt::new(BlockSize::from_samples(m as u16).unwrap());
    let win: Vec<f64> = (0..2 * m)
        .map(|n| (std::f64::consts::PI * (n as f64 + 0.5) / (2 * m) as f64).sin())
        .collect();
    let mut acc = vec![0.0; m];
    let mut start = 0;
    while start + 2 * m <= x.len() {
        let frame: Vec<f64> = x[start..start + 2 * m]
            .iter()
            .zip(win.iter())
            .map(|(a, w)| a * w)
            .collect();
        for (a, c) in acc.iter_mut().zip(mlt.forward(&frame).unwrap()) {
            *a += c * c;
        }
        start += m;
    }
    acc
}

/// Band power ratio (dB) of `y` over `x` on the fraction range
/// `[lo, hi)` of the half sample rate.
fn band_level_db(x: &[f64], y: &[f64], lo: f64, hi: f64) -> f64 {
    let (px, py) = (power_spectrum(x), power_spectrum(y));
    let m = px.len();
    let (a, b) = ((lo * m as f64) as usize, (hi * m as f64) as usize);
    let ex: f64 = px[a..b].iter().sum();
    let ey: f64 = py[a..b].iter().sum();
    10.0 * (ey / ex.max(1e-300)).log10()
}

/// The encoder against the same targets: the vendor PCM of every
/// fixture re-encoded by this crate's encoder at the fixture's exact
/// configuration (`flags2` bit 5 clear, v2), decoded by this crate
/// and — when the black-box reference is available — by it. Every
/// stream must decode in both, track at unit gain, and put the
/// noise-substituted region back at the input's level.
#[test]
fn encoder_reencodes_every_fixture_configuration() {
    use oxideav_wma::vendor_analysis::VendorEncoder;
    use oxideav_wma::vendor_frame::noise_cutoff;
    let loaded = skip_unless_fixtures!();
    let with_reference = common::reference_available();
    for l in &loaded {
        let input: Vec<Vec<f64>> = l
            .expected
            .iter()
            .map(|c| c.iter().map(|&s| f64::from(s) / 32768.0).collect())
            .collect();
        let mut enc = VendorEncoder::new(&l.cfg).expect("fixture configuration encodable");
        enc.push(&input).unwrap();
        let packets = enc.finish().unwrap();
        assert!(
            packets
                .iter()
                .all(|p| p.len() == usize::from(l.fx.block_align))
        );
        let own = common::decode_own(&l.cfg, &packets);
        let cutoff = noise_cutoff(&l.cfg).expect("noise substitution on every fixture");
        let coef_end =
            f64::from(l.cfg.coef_end(l.cfg.frame_length)) / f64::from(l.cfg.frame_length);
        let mut line = format!("{:34}", l.fx.dir);
        let mut own_corr = Vec::new();
        for (ch, orig) in input.iter().enumerate() {
            let (lag, corr2, gain, snr) = common::fit(orig, &own[ch]);
            let aligned: Vec<f64> = (0..orig.len())
                .map(|t| {
                    let u = t as i64 + lag;
                    if u < 0 || u as usize >= own[ch].len() {
                        0.0
                    } else {
                        own[ch][u as usize]
                    }
                })
                .collect();
            let low = band_level_db(orig, &aligned, 0.0, cutoff);
            let high = band_level_db(orig, &aligned, cutoff, coef_end);
            line += &format!(
                " | ch{ch} own corr2 {corr2:.3} gain {gain:.3} SNR {snr:.1} dB, coded band {low:+.2} dB, noise band {high:+.2} dB"
            );
            // r485: own SNR 5.0–9.5 dB on these deliberately
            // noise-heavy signals (the vendor encoder's own sanity
            // SNRs against their sources: 3.7–7.9 dB), coded region
            // −0.2 … −0.7 dB, noise region within ±0.6 dB (was
            // +0.3 … +5.0 dB before the encoder accounted for the
            // decoder's ratio_k and the joint-stereo noise spread).
            assert!(corr2 > 0.65, "{}: own corr2 {corr2}", l.fx.dir);
            assert!((0.85..1.1).contains(&gain), "{}: own gain {gain}", l.fx.dir);
            assert!(snr > 4.0, "{}: own SNR {snr:.2} dB", l.fx.dir);
            assert!(low.abs() < 1.5, "{}: coded region {low:+.2} dB", l.fx.dir);
            assert!(high.abs() < 1.5, "{}: noise region {high:+.2} dB", l.fx.dir);
            own_corr.push(corr2);
        }
        if with_reference {
            let mut extradata = [0u8; 10];
            extradata[4..6].copy_from_slice(&l.fx.flags2.to_le_bytes());
            let wav = common::wav_wrap(&l.cfg, &extradata, &packets);
            let r = common::reference_decode(&wav, input.len())
                .unwrap_or_else(|| panic!("{}: reference rejected the re-encode", l.fx.dir));
            for (ch, orig) in input.iter().enumerate() {
                let (_, corr2, gain, snr) = common::fit(orig, &r[ch]);
                line += &format!(" | ref ch{ch} corr2 {corr2:.3} gain {gain:.3} SNR {snr:.1}");
                assert!(
                    corr2 > own_corr[ch] - 0.02 && (0.8..1.25).contains(&gain),
                    "{} ch{ch}: reference corr2 {corr2:.3} gain {gain:.3} does not track own {:.3}",
                    l.fx.dir,
                    own_corr[ch]
                );
            }
        }
        eprintln!("{line}");
    }
}

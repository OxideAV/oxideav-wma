//! §3.1 LSP envelope — bit-exactness of [`oxideav_wma::lsp_envelope`]
//! against the staged model `docs/audio/wma/tables/lsp_envelope_exact.py`
//! (round 09, validated bit-for-bit against the sandboxed vendor
//! decoder in round 10), and of the run-time root LUTs against the
//! staged `wma-lsp-root-lut-*.csv`.
//!
//! The staged script is run as a black box through `python3`; the
//! docs tables directory is located via `OXIDEAV_WMA_TABLES_DIR` or
//! the umbrella workspace layout. When either is absent the tests
//! skip. The vendor-stream leg additionally needs the committed
//! `cand_mono8k_8kbps_v8.wma` fixture (see `tests/vendor_streams.rs`).

use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

use oxideav_wma::header::Version;
use oxideav_wma::lsp_envelope::{lsp_envelope, RootLuts, LSP_ORDER};
use oxideav_wma::packet::PacketAssembler;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_frame::{Envelope, FrameParser, LSP_INDEX_WIDTHS};

fn tables_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("OXIDEAV_WMA_TABLES_DIR") {
        let p = PathBuf::from(dir);
        return p.is_dir().then_some(p);
    }
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/audio/wma/tables");
    p.join("lsp_envelope_exact.py").is_file().then_some(p)
}

/// One conversion request: `(grid_len, block_len, indices)`.
type Request = (usize, usize, [u8; LSP_ORDER]);

/// Run the staged model on `requests`; returns per request
/// `(status, max_bits, env_bits)`. The requests go through a temp
/// file (a stdin pipe would deadlock against the model's stdout).
fn staged_model(dir: &std::path::Path, requests: &[Request]) -> Option<Vec<(u32, u32, Vec<u32>)>> {
    let script = "import sys\n\
        sys.path.insert(0, sys.argv[1])\n\
        import lsp_envelope_exact as m\n\
        tab = m.Tables()\n\
        for line in open(sys.argv[2]):\n\
        \x20   parts = line.split()\n\
        \x20   if not parts:\n\
        \x20       continue\n\
        \x20   L = int(parts[0]); N = int(parts[1]); idx = [int(x) for x in parts[2:12]]\n\
        \x20   env, mx, st = m.envelope(idx, N, L, tab)\n\
        \x20   print(st, m.f32_bits(mx) if mx is not None else 0, \
        ' '.join('%08x' % m.f32_bits(v) for v in env))\n";
    let tmp = std::env::temp_dir().join(format!(
        "oxideav-wma-lsp-model-{}-{}.txt",
        std::process::id(),
        requests.len()
    ));
    {
        let mut f = std::fs::File::create(&tmp).ok()?;
        for (l, n, idx) in requests {
            let line: Vec<String> = [l.to_string(), n.to_string()]
                .into_iter()
                .chain(idx.iter().map(|i| i.to_string()))
                .collect();
            writeln!(f, "{}", line.join(" ")).ok()?;
        }
    }
    let out = Command::new("python3")
        .arg("-c")
        .arg(script)
        .arg(dir)
        .arg(&tmp)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .output();
    let _ = std::fs::remove_file(&tmp);
    let out = out.ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8(out.stdout).ok()?;
    let mut parsed = Vec::with_capacity(requests.len());
    for line in text.lines() {
        let mut it = line.split_whitespace();
        let status: u32 = it.next()?.parse().ok()?;
        let max: u32 = it.next()?.parse().ok()?;
        let env: Vec<u32> = it.map(|h| u32::from_str_radix(h, 16).unwrap()).collect();
        parsed.push((status, max, env));
    }
    (parsed.len() == requests.len()).then_some(parsed)
}

fn random_indices(seed: &mut u64) -> [u8; LSP_ORDER] {
    let mut out = [0u8; LSP_ORDER];
    for (slot, &w) in out.iter_mut().zip(LSP_INDEX_WIDTHS.iter()) {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        *slot = (*seed % (1u64 << w)) as u8;
    }
    out
}

fn check(requests: &[Request], expected: &[(u32, u32, Vec<u32>)]) -> usize {
    let mut bins = 0usize;
    for ((l, n, idx), (status, max_bits, env_bits)) in requests.iter().zip(expected.iter()) {
        let ours = lsp_envelope(idx, *n, *l);
        if *status != 0 {
            assert!(
                ours.is_err(),
                "L={l} N={n} {idx:?}: model fails, we succeed"
            );
            continue;
        }
        let ours = ours.unwrap_or_else(|e| panic!("L={l} N={n} {idx:?}: {e}"));
        assert_eq!(
            ours.weights.len(),
            env_bits.len(),
            "L={l} N={n} {idx:?}: length"
        );
        for (i, (w, &bits)) in ours.weights.iter().zip(env_bits.iter()).enumerate() {
            assert_eq!(
                w.to_bits(),
                bits,
                "L={l} N={n} {idx:?} bin {i}: ours {} model {}",
                w,
                f32::from_bits(bits)
            );
        }
        assert_eq!(ours.max.to_bits(), *max_bits, "L={l} N={n} {idx:?}: max");
        bins += env_bits.len();
    }
    bins
}

#[test]
fn root_luts_match_the_staged_csvs() {
    let Some(dir) = tables_dir() else {
        eprintln!("skipping: staged tables unavailable");
        return;
    };
    let luts = RootLuts::build();
    let read = |name: &str| -> Vec<(usize, f32)> {
        std::fs::read_to_string(dir.join(name))
            .unwrap()
            .lines()
            .filter_map(|l| {
                let mut it = l.split(',');
                let i: usize = it.next()?.trim().parse().ok()?;
                let v: f32 = it.next()?.trim().parse().ok()?;
                Some((i, v))
            })
            .collect()
    };
    let mantissa = read("wma-lsp-root-lut-mantissa.csv");
    assert_eq!(mantissa.len(), 4096);
    for (i, v) in mantissa {
        assert_eq!(luts.mantissa[i].to_bits(), v.to_bits(), "M[{i}]");
    }
    let exponent = read("wma-lsp-root-lut-exponent.csv");
    assert_eq!(exponent.len(), 256);
    for (i, v) in exponent {
        assert_eq!(luts.exponent[i].to_bits(), v.to_bits(), "E[{i}]");
    }
}

#[test]
fn random_inputs_are_bit_exact_at_every_grid_length() {
    let Some(dir) = tables_dir() else {
        eprintln!("skipping: staged tables unavailable");
        return;
    };
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut requests: Vec<Request> = Vec::new();
    for &l in &[16usize, 32, 64, 128, 256, 512, 1024, 2048] {
        for trial in 0..12 {
            let idx = random_indices(&mut seed);
            // N = L, and the grid-scaling form N < L on the larger grids.
            let n = match trial % 3 {
                0 => l,
                1 => (l / 2).max(1),
                _ => (l / 4).max(1),
            };
            requests.push((l, n, idx));
        }
    }
    // Every index at its extremes.
    requests.push((512, 512, [0; LSP_ORDER]));
    requests.push((512, 512, [7, 15, 15, 15, 15, 15, 15, 15, 7, 7]));
    let Some(expected) = staged_model(&dir, &requests) else {
        eprintln!("skipping: python3 / staged model unavailable");
        return;
    };
    let bins = check(&requests, &expected);
    eprintln!(
        "lsp_model: {} random conversions, {bins} bins bit-exact against the staged model",
        requests.len()
    );
}

/// The vendor stream's own conversions: every LSP index set the
/// parser reads from `cand_mono8k_8kbps_v8.wma`, converted at the
/// stream's geometry (`flags2 = 0x0026`: bit 5 set, so `L = N = 512`),
/// bit-exact against the staged model — which round 10 validated
/// bit-for-bit against the vendor decoder on exactly these inputs
/// (3 931 conversions, 2 012 672 bins).
#[test]
fn vendor_stream_conversions_are_bit_exact() {
    let Some(dir) = tables_dir() else {
        eprintln!("skipping: staged tables unavailable");
        return;
    };
    let vendor = match std::env::var("OXIDEAV_WMA_VENDOR_STREAMS_DIR") {
        Ok(d) => PathBuf::from(d),
        Err(_) => PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../docs/audio/wma/reference/vendor-streams"),
    };
    let file = vendor.join("cand_mono8k_8kbps_v8.wma");
    if !file.is_file() {
        eprintln!("skipping: vendor stream unavailable");
        return;
    }
    let Some(raw) = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(&file)
        .args(["-map", "0:a:0", "-c", "copy", "-f", "data", "-"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| o.stdout)
    else {
        eprintln!("skipping: black-box unwrap unavailable");
        return;
    };
    let cfg = StreamConfig::derive(Version::V2, 8000, 1, 1000, 640, 0x0026).unwrap();
    let mut asm = PacketAssembler::new(&cfg);
    for pkt in raw.chunks_exact(usize::from(cfg.block_align)) {
        asm.push_packet(pkt).unwrap();
    }
    let stream = asm.finish();
    let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
    let mut parser = FrameParser::new(&cfg, &body_starts);
    let mut requests: Vec<Request> = Vec::new();
    let mut cursor = stream.packets[0].frames_start_bit();
    for rec in stream.packets.iter() {
        if cursor != rec.frames_start_bit() {
            cursor = rec.frames_start_bit();
            parser.raise_latch();
        }
        let mut reader = stream.reader_at(cursor);
        for _ in 0..rec.header.frame_count {
            let Ok(frame) = parser.parse_frame(&mut reader) else {
                break;
            };
            for block in &frame.blocks {
                let n = usize::from(block.block_size);
                let l = usize::from(cfg.lsp_grid_len(block.block_size));
                for chan in &block.channels {
                    if let Some(Envelope::LspIndices(idx)) = chan.envelope {
                        requests.push((l, n, idx));
                    }
                }
            }
        }
        cursor = reader.position() as u64;
    }
    assert!(
        requests.len() >= 3900,
        "expected the stream's 3 931 conversions, parsed {}",
        requests.len()
    );
    let Some(expected) = staged_model(&dir, &requests) else {
        eprintln!("skipping: python3 / staged model unavailable");
        return;
    };
    let bins = check(&requests, &expected);
    eprintln!(
        "lsp_model: cand_mono8k_8kbps_v8 — {} conversions, {bins} bins bit-exact against the staged model",
        requests.len()
    );
    assert_eq!(
        requests.len(),
        3931,
        "the vendor stream carries 3 931 conversions"
    );
}

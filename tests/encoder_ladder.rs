//! Encoder bitrate/quality **ladder** — every WMA v2 (`0x0161`)
//! configuration of the staged ACM format catalogue
//! (`docs/audio/wma/tables/wma-acm-standard-formats.csv`): the cells
//! whose `flags2` selects the VLC envelope path (bit 0 set) and,
//! since r459, the low-rate cells on the §3.1 LSP envelope path (bit
//! 0 clear; 8–32 kHz, 5–22 kbps), plus the six committed
//! vendor-stream geometries, each encoded from the same synthetic
//! material under the default (transient-splitting) block policy
//! and, on VBL cells, under a fixed full-length schedule for the
//! block-switching delta.
//!
//! Every cell is decoded two ways — this crate's own vendor chain
//! and the black-box reference decoder (skipped when unavailable) —
//! and reported as a per-cell row: own-chain SNR, reference SNR /
//! corr² / fitted gain, and the rate actually used. The reference is
//! held to *track* the own chain (corr² within 0.02, SNR within
//! 3 dB, gain ≈ 1): that is the wire-format acceptance bar per cell,
//! and it is what catches a configuration-dependent policy
//! divergence (the r457 32 kHz class-2 family decoded to garbage at
//! the reference before the noise-substitution policy covered it).

mod common;

use common::{
    decode_own, fit, material, reference_available, reference_decode, snr_at_lead, wav_wrap,
};
use oxideav_wma::header::Version;
use oxideav_wma::stream_config::StreamConfig;
use oxideav_wma::vendor_analysis::{BlockPolicy, EncoderSettings, VendorEncoder};

/// One ladder cell: `(channels, sample rate, avg bytes/s, block_align, flags2)`.
type Cell = (u8, u32, u32, u16, u16);

/// The encodable v2 catalogue cells (format indices 16, 17, 21, 22,
/// 24–30, 32–41) followed by the vendor-stream geometries that are
/// not already catalogue rows.
const CELLS: &[Cell] = &[
    (2, 22_050, 4005, 744, 0x0017),
    (2, 22_050, 4005, 186, 0x0001),
    (1, 32_000, 4004, 820, 0x000f),
    (1, 32_000, 4000, 256, 0x0001),
    (2, 32_000, 8003, 1639, 0x000f),
    (2, 32_000, 8000, 512, 0x0001),
    (2, 32_000, 6001, 1229, 0x001f),
    (2, 32_000, 5503, 1127, 0x001f),
    (2, 32_000, 5000, 1024, 0x0017),
    (2, 32_000, 4502, 922, 0x0017),
    (2, 32_000, 4004, 820, 0x0017),
    (1, 44_100, 4005, 744, 0x000f),
    (2, 44_100, 20_004, 3716, 0x000f),
    (2, 44_100, 16_005, 2973, 0x000f),
    (2, 44_100, 12_005, 2230, 0x000f),
    (2, 44_100, 10_002, 1858, 0x000f),
    (2, 44_100, 8005, 1487, 0x000f),
    (2, 44_100, 8010, 372, 0x0001),
    (2, 44_100, 6002, 1115, 0x0017),
    (2, 48_000, 20_000, 3901, 0x000f),
    (2, 48_000, 16_001, 3121, 0x000f),
    // Vendor-stream geometries.
    (1, 22_050, 2003, 744, 0x000f),
    (2, 22_050, 4006, 744, 0x0017),
    (2, 44_100, 12_003, 4459, 0x000f),
    (2, 44_100, 11_111, 4459, 0x000f),
    (2, 44_100, 8003, 2973, 0x000f),
];

/// The §3.1 LSP-envelope cells of the catalogue (format indices
/// 0–2, 4–15, 18–20, 23, 31; index 3 — 128 bit/s, `block_align` 1 —
/// is skipped) followed by the vendor mono 8 kHz geometry.
const LSP_CELLS: &[Cell] = &[
    (1, 8_000, 1000, 64, 0x0000),
    (1, 8_000, 750, 48, 0x0000),
    (1, 8_000, 625, 40, 0x0000),
    (2, 8_000, 1500, 96, 0x0000),
    (1, 11_025, 1249, 58, 0x0000),
    (1, 11_025, 1012, 47, 0x0000),
    (1, 16_000, 2000, 512, 0x000e),
    (1, 16_000, 2000, 64, 0x0000),
    (1, 16_000, 1500, 384, 0x0006),
    (1, 16_000, 1250, 320, 0x0006),
    (2, 16_000, 2502, 427, 0x0016),
    (2, 16_000, 2004, 342, 0x000e),
    (1, 22_050, 2503, 744, 0x0016),
    (1, 22_050, 2519, 117, 0x0000),
    (1, 22_050, 2002, 595, 0x000e),
    (2, 22_050, 2751, 511, 0x000e),
    (2, 22_050, 2503, 744, 0x000e),
    (2, 22_050, 2519, 117, 0x0000),
    (1, 32_000, 2502, 854, 0x0016),
    (2, 32_000, 2751, 939, 0x0016),
    // Vendor-stream geometry (cand_mono8k_8kbps_v8).
    (1, 8_000, 1000, 640, 0x0026),
];

/// Own-chain SNR floor every cell must clear (dB). The synthetic
/// material carries a click train, so this is a transient-heavy
/// figure; steady tones measure 10–20 dB higher.
const OWN_FLOOR_DB: f64 = 16.0;

/// Own-chain floor for the LSP cells (dB): a tenth of the bits per
/// sample of the VLC cells and a ten-parameter envelope.
const LSP_OWN_FLOOR_DB: f64 = 10.0;

fn cell_pcm(sample_rate: u32, channels: u8) -> Vec<Vec<f64>> {
    let len = sample_rate as usize * 2;
    let left = material(sample_rate, len, 1);
    if channels == 2 {
        let right: Vec<f64> = material(sample_rate, len, 2)
            .iter()
            .zip(left.iter())
            .map(|(r, l)| 0.6 * l + 0.4 * r)
            .collect();
        vec![left, right]
    } else {
        vec![left]
    }
}

fn extradata(flags2: u16) -> [u8; 10] {
    let mut e = [0u8; 10];
    e[4..6].copy_from_slice(&flags2.to_le_bytes());
    e
}

struct Row {
    own_snr: f64,
    own_corr2: f64,
    rate_pct: f64,
    /// Per channel `(corr2, gain, snr)` at the reference.
    reference: Option<Vec<(f64, f64, f64)>>,
}

fn run_cell(cell: &Cell, blocks: BlockPolicy, with_reference: bool) -> Row {
    let (channels, sample_rate, bps, block_align, flags2) = *cell;
    let cfg = StreamConfig::derive(Version::V2, sample_rate, channels, bps, block_align, flags2)
        .expect("catalogue cell derives");
    let pcm = cell_pcm(sample_rate, channels);
    let settings = EncoderSettings {
        blocks,
        ..EncoderSettings::default()
    };
    let mut enc = VendorEncoder::with_settings(&cfg, settings).expect("encodable cell");
    enc.push(&pcm).unwrap();
    let packets = enc.finish().unwrap();
    assert!(
        packets.iter().all(|p| p.len() == usize::from(block_align)),
        "packet sizing"
    );
    let own = decode_own(&cfg, &packets);
    let mut own_snr = f64::INFINITY;
    let mut own_corr2 = f64::INFINITY;
    for (ch, orig) in pcm.iter().enumerate() {
        own_snr = own_snr.min(snr_at_lead(&cfg, orig, &own[ch]));
        own_corr2 = own_corr2.min(fit(orig, &own[ch]).1);
    }
    let bytes: usize = packets.iter().map(|p| p.len()).sum();
    let rate_pct = bytes as f64 / (f64::from(bps) * 2.0) * 100.0;
    let reference = if with_reference {
        let wav = wav_wrap(&cfg, &extradata(flags2), &packets);
        reference_decode(&wav, pcm.len()).map(|r| {
            pcm.iter()
                .enumerate()
                .map(|(ch, orig)| {
                    let (_, c, g, s) = fit(orig, &r[ch]);
                    (c, g, s)
                })
                .collect()
        })
    } else {
        None
    };
    Row {
        own_snr,
        own_corr2,
        rate_pct,
        reference,
    }
}

fn cell_name(cell: &Cell) -> String {
    let (ch, sr, bps, ba, fl) = *cell;
    format!(
        "{ch}ch {sr:>5} Hz {:>3} kbps ba{ba:<4} f{fl:04x}",
        bps * 8 / 1000
    )
}

/// Every cell encodes, decodes through the own chain above the
/// floor, and — where the reference is available — is accepted by
/// it with PCM tracking the own chain.
#[test]
fn every_catalogue_cell_encodes_and_is_accepted() {
    let with_reference = reference_available();
    if !with_reference {
        eprintln!("reference decoder unavailable: own-chain leg only");
    }
    let mut failures = Vec::new();
    for cell in CELLS.iter().chain(LSP_CELLS.iter()) {
        let cfg =
            StreamConfig::derive(Version::V2, cell.1, cell.0, cell.2, cell.3, cell.4).unwrap();
        let own_floor = if cfg.exp_vlc {
            OWN_FLOOR_DB
        } else {
            LSP_OWN_FLOOR_DB
        };
        let row = run_cell(cell, BlockPolicy::Auto, with_reference);
        let fixed = if cfg.vbl_enabled {
            Some(run_cell(cell, BlockPolicy::Fixed(0), with_reference))
        } else {
            None
        };
        let reference_text = match &row.reference {
            Some(chs) => chs
                .iter()
                .map(|(c, g, s)| format!("ref corr2 {c:.3} gain {g:.2} SNR {s:.1}"))
                .collect::<Vec<_>>()
                .join(" | "),
            None if with_reference => "ref REJECTED".to_string(),
            None => String::new(),
        };
        eprintln!(
            "{} {} cls{} nbs{:<2} | own SNR {:.1} dB (fixed-block {}) rate {:.0}% | {}",
            cell_name(cell),
            if cfg.exp_vlc { "vlc" } else { "lsp" },
            cfg.vlc_class,
            cfg.n_block_sizes,
            row.own_snr,
            fixed
                .as_ref()
                .map(|f| format!("{:.1}", f.own_snr))
                .unwrap_or_else(|| "-".into()),
            row.rate_pct,
            reference_text
        );
        if row.own_snr < own_floor {
            failures.push(format!(
                "{}: own SNR {:.2} dB below {own_floor}",
                cell_name(cell),
                row.own_snr
            ));
        }
        if with_reference {
            match &row.reference {
                None => failures.push(format!("{}: reference rejected", cell_name(cell))),
                Some(chs) => {
                    for (ch, (c, g, s)) in chs.iter().enumerate() {
                        if *c < row.own_corr2 - 0.02
                            || !(0.8..1.25).contains(g)
                            || (s - row.own_snr).abs() > 3.0 && *s < row.own_snr
                        {
                            failures.push(format!(
                                "{} ch{ch}: reference corr2 {c:.3} gain {g:.2} SNR {s:.1} does not track own (corr2 {:.3}, SNR {:.1})",
                                cell_name(cell),
                                row.own_corr2,
                                row.own_snr
                            ));
                        }
                    }
                }
            }
            if let Some(f) = &fixed {
                if f.reference.is_none() {
                    failures.push(format!(
                        "{}: reference rejected the fixed-block stream",
                        cell_name(cell)
                    ));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "ladder failures:\n{}",
        failures.join("\n")
    );
}

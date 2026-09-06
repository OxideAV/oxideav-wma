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
//!    is unavailable. The reference is decoded at the stream's own
//!    channel count and fitted per channel over the overlapping
//!    interior (r457: the r454 leg compared a stereo→mono downmix
//!    and counted the reference's shorter tail as error, which
//!    capped every family near 14 dB; measured like for like the
//!    reference SNR tracks the own-chain SNR within ≈ 0.5 dB).
//!
//! The WAV path itself is validated against the reference decoder's
//! own ASF unwrap in `tests/vendor_streams.rs` fixtures territory:
//! remuxing a committed vendor stream ASF → WAV decodes
//! byte-identically, so WAV acceptance is equivalent to codec-level
//! acceptance.

mod common;

use common::{
    decode_own, fit, hiss, material, reference_available, reference_decode, snr_at_lead, wav_wrap,
};
use oxideav_wma::header::Version;
use oxideav_wma::stream_config::StreamConfig;
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
    /// Level of a white hiss mixed into the material (0 = none) — the
    /// §2.1 noise-substitution exercise: at 16 kbps the hiss above
    /// the cutoff quantises to nothing and travels as F3/F4 noise
    /// bands, which the reference must accept.
    hiss: f64,
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
            reference_floor: 12.0,
            hiss: 0.0,
        },
        Family {
            // The staged cand_stereo22k geometry: stereo VBL + reservoir.
            name: "stereo22k_32kbps_vbl",
            cfg: StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap(),
            extradata: mk_extra(0x0017),
            own_floor: 8.0,
            reference_floor: 12.0,
            hiss: 0.0,
        },
        Family {
            // The staged cand_mono22k geometry: mono VBL + reservoir.
            name: "mono22k_16kbps_vbl",
            cfg: StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap(),
            extradata: mk_extra(0x000f),
            own_floor: 6.0,
            reference_floor: 11.0,
            hiss: 0.0,
        },
        Family {
            // The mono 22.05 kHz geometry again, with a hiss: noise
            // substitution on the encode side.
            name: "mono22k_16kbps_hiss",
            cfg: StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap(),
            extradata: mk_extra(0x000f),
            own_floor: 6.0,
            reference_floor: 11.0,
            hiss: 0.01,
        },
        Family {
            // The staged cand_wmp12 geometry: stereo 44.1 kHz 96 kbps.
            name: "stereo44k_96kbps_vbl",
            cfg: StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 4459, 0x000f).unwrap(),
            extradata: mk_extra(0x000f),
            own_floor: 14.0,
            reference_floor: 12.0,
            hiss: 0.0,
        },
    ]
}

fn encode_family(fam: &Family, pcm: &[Vec<f64>]) -> Vec<Vec<u8>> {
    let mut enc = VendorEncoder::new(&fam.cfg).unwrap();
    enc.push(pcm).unwrap();
    enc.finish().unwrap()
}

fn family_pcm(fam: &Family) -> Vec<Vec<f64>> {
    let sr = fam.cfg.sample_rate;
    let len = (sr as usize) * 3;
    let mut left = material(sr, len, 1);
    if fam.hiss > 0.0 {
        for (v, h) in left.iter_mut().zip(hiss(len, 7)) {
            *v += fam.hiss * h;
        }
    }
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
    if !reference_available() {
        eprintln!("skipping: reference decoder unavailable");
        return;
    }
    for fam in &families() {
        let pcm = family_pcm(fam);
        let packets = encode_family(fam, &pcm);
        let wav = wav_wrap(&fam.cfg, &fam.extradata, &packets);
        let reference = match reference_decode(&wav, pcm.len()) {
            Some(r) => r,
            None => panic!("{}: reference decoder rejected the stream", fam.name),
        };
        assert!(
            reference[0].len() * 2 > pcm[0].len(),
            "{}: reference produced too few samples",
            fam.name
        );
        for (ch, orig) in pcm.iter().enumerate() {
            let (lag, corr2, gain, snr) = fit(orig, &reference[ch]);
            eprintln!(
                "{} ch{ch}: reference decode {} samples, lag {lag}, corr2 {corr2:.3}, gain {gain:.3}, SNR {snr:.2} dB",
                fam.name,
                reference[ch].len()
            );
            assert!(corr2 > 0.95, "{}: corr2 {corr2:.3}", fam.name);
            assert!(
                (0.8..1.25).contains(&gain),
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
}

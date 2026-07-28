#![no_main]

//! Fuzz: the five staged coefficient-VLC tables, both directions.
//!
//! * fuzzer-chosen symbol streams encode → decode bit-exactly and
//!   consume exactly the written bit length;
//! * `expand()` is total over the alphabet of every table whose
//!   symbol → `(R, L)` companion map is staged (the three primary
//!   classes); on the alt variants — whose companion maps are a
//!   documented unstaged gap — it must fail *typed*, never panic;
//! * arbitrary input bits decode without panicking (bounded walk;
//!   mode 2's documented-incomplete prefix code may surface its
//!   typed unassigned-space error).

use libfuzzer_sys::fuzz_target;
use oxideav_wma::bitio::{BitReader, BitWriter};
use oxideav_wma::coef_vlc::{CoefDecodeMode, CoefVlc};
use std::sync::OnceLock;

fn vlcs() -> &'static Vec<CoefVlc> {
    static VLCS: OnceLock<Vec<CoefVlc>> = OnceLock::new();
    VLCS.get_or_init(|| {
        CoefDecodeMode::ALL
            .iter()
            .map(|&mode| CoefVlc::new(mode).expect("all five staged tables construct"))
            .collect()
    })
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    for vlc in vlcs() {
        let count = vlc.symbol_count();

        // Symbol stream from byte pairs, folded into the alphabet.
        let symbols: Vec<usize> = data
            .chunks_exact(2)
            .take(64)
            .map(|p| (usize::from(p[0]) << 8 | usize::from(p[1])) % count)
            .collect();

        let has_map = vlc.mode().runlevel_map().is_some();
        let mut w = BitWriter::new();
        for &s in &symbols {
            vlc.encode_symbol(s, &mut w)
                .expect("every alphabet symbol has a staged codeword");
            let expanded = vlc.expand(s);
            if has_map {
                expanded.expect("expand is total when the companion map is staged");
            } else {
                // Alt variants: pair symbols must surface the typed
                // companion-map gap; the reserved sentinels still
                // expand. Either way: no panic.
                let _ = expanded;
            }
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();

        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        for &s in &symbols {
            assert_eq!(
                vlc.decode_symbol(&mut r).expect("own emission must decode"),
                s,
                "{:?}",
                vlc.mode(),
            );
        }
        assert_eq!(r.remaining_bits(), 0, "no trailing bits");

        // Arbitrary bits: bounded panic-freedom walk.
        let mut r = BitReader::new(data);
        for _ in 0..64 {
            if vlc.decode_symbol(&mut r).is_err() {
                break;
            }
        }
    }
});

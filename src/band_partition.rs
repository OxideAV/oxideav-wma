//! §3 exponent/quantisation band partitions — the per-block-size
//! band edge lists a variable-block-length decoder must select.
//!
//! ## Source
//!
//! * `docs/audio/wma/tables/exponent-band-partitions.csv` (+ `.meta`)
//!   — the eight hard-coded partitions the vendor decoder installs
//!   for the smaller block sizes, staged post round-to-multiple-of-
//!   four (`edge` column, the value a decoder must use). Selection is
//!   on (sample-rate arm, block size): `hi` ≥ 44100 Hz with 128/256/
//!   512-coefficient tables, `mid` 32000–44099 Hz likewise, `lo`
//!   22050–31999 Hz with 128/256 only; anything else falls through
//!   to the computed partition.
//! * The computed fallback (staged as a corroborated reconstruction
//!   in the same `.meta`): each Bark critical-band edge frequency
//!   (`tables/critical-band-freqs.csv`, carried in
//!   [`crate::wire_tables::CRITICAL_BAND_FREQS_HZ`]) is converted to
//!   a coefficient index truncated to a multiple of four —
//!   `trunc(f_Hz · 2 · block_coefficients / sample_rate / 4) · 4` —
//!   appended when it exceeds the previous edge; the walk stops at
//!   the first edge past the block and that edge is replaced by the
//!   block's coefficient count. For a 2048-coefficient block at
//!   44100 Hz this yields 25 bands, the count `frame-bit-layout.md`
//!   §3 recorded independently from the decoder's envelope loop.
//!
//! Band `b` covers coefficients `[edge[b], edge[b+1])`; the band
//! count (`edges.len() − 1`) is the number of spectral-envelope
//! symbols a coded channel sends for a block of that size — the
//! detail a variable-block-length parse desynchronises on if it
//! assumes the full-size count (per the staged `.meta` role note).

use crate::wire_tables::CRITICAL_BAND_FREQS_HZ;

// The eight staged partitions (`edge` column — post rounding).
const HI_128: [u16; 13] = [0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 72, 92, 128];
const HI_256: [u16; 16] = [
    0, 4, 12, 16, 24, 32, 36, 44, 52, 64, 76, 88, 112, 140, 180, 256,
];
const HI_512: [u16; 18] = [
    0, 4, 12, 20, 24, 36, 48, 56, 64, 88, 104, 124, 148, 180, 220, 280, 360, 512,
];
const MID_128: [u16; 12] = [0, 4, 8, 16, 20, 24, 36, 52, 76, 96, 124, 128];
const MID_256: [u16; 16] = [
    0, 4, 12, 16, 20, 28, 36, 52, 72, 84, 104, 124, 152, 192, 248, 256,
];
const MID_512: [u16; 17] = [
    0, 8, 12, 20, 28, 40, 56, 76, 100, 140, 172, 204, 248, 304, 384, 496, 512,
];
const LO_128: [u16; 11] = [0, 4, 12, 16, 24, 32, 44, 64, 88, 112, 128];
const LO_256: [u16; 15] = [
    0, 4, 12, 20, 24, 36, 48, 64, 88, 104, 124, 148, 180, 220, 256,
];

/// The staged sample-rate arm of the band-partition selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RateArm {
    Hi,
    Mid,
    Lo,
    /// Below 22050 Hz: no hard table at all.
    Computed,
}

fn rate_arm(sample_rate: u32) -> RateArm {
    if sample_rate >= 44_100 {
        RateArm::Hi
    } else if sample_rate >= 32_000 {
        RateArm::Mid
    } else if sample_rate >= 22_050 {
        RateArm::Lo
    } else {
        RateArm::Computed
    }
}

/// The hard-coded partition for `(sample_rate, block_coefficients)`,
/// if the staged selector lists one.
fn hard_partition(sample_rate: u32, block_coefficients: u16) -> Option<&'static [u16]> {
    match (rate_arm(sample_rate), block_coefficients) {
        (RateArm::Hi, 128) => Some(&HI_128),
        (RateArm::Hi, 256) => Some(&HI_256),
        (RateArm::Hi, 512) => Some(&HI_512),
        (RateArm::Mid, 128) => Some(&MID_128),
        (RateArm::Mid, 256) => Some(&MID_256),
        (RateArm::Mid, 512) => Some(&MID_512),
        (RateArm::Lo, 128) => Some(&LO_128),
        (RateArm::Lo, 256) => Some(&LO_256),
        _ => None,
    }
}

/// The computed partition: the critical-band walk described in the
/// staged `.meta` (`computed_fallback`).
fn computed_partition(sample_rate: u32, block_coefficients: u16) -> Vec<u16> {
    let mut edges: Vec<u16> = vec![0];
    for &f_hz in &CRITICAL_BAND_FREQS_HZ {
        let edge =
            (u64::from(f_hz) * 2 * u64::from(block_coefficients) / u64::from(sample_rate) / 4) * 4;
        if edge >= u64::from(block_coefficients) {
            edges.push(block_coefficients);
            break;
        }
        let edge = edge as u16;
        if edge > *edges.last().expect("non-empty") {
            edges.push(edge);
        }
    }
    // Degenerate safety: a walk that never reached the block's end
    // (possible only for configurations outside the staged rate
    // range) still closes at the block boundary.
    if *edges.last().expect("non-empty") != block_coefficients {
        edges.push(block_coefficients);
    }
    edges
}

/// The exponent-band edge list for a block of `block_coefficients`
/// coefficients in a stream at `sample_rate` Hz: the staged
/// hard-coded partition when the selector lists one, the computed
/// critical-band walk otherwise. Edges are inclusive of both `0` and
/// `block_coefficients`; the band count is `edges.len() − 1`.
pub fn exponent_band_edges(sample_rate: u32, block_coefficients: u16) -> Vec<u16> {
    match hard_partition(sample_rate, block_coefficients) {
        Some(edges) => edges.to_vec(),
        None => computed_partition(sample_rate, block_coefficients),
    }
}

/// The per-block-size band count (`exponent_band_edges` length − 1).
pub fn exponent_band_count(sample_rate: u32, block_coefficients: u16) -> usize {
    exponent_band_edges(sample_rate, block_coefficients).len() - 1
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_HARD: [(&[u16], u32, u16); 8] = [
        (&HI_128, 44_100, 128),
        (&HI_256, 44_100, 256),
        (&HI_512, 44_100, 512),
        (&MID_128, 32_000, 128),
        (&MID_256, 32_000, 256),
        (&MID_512, 32_000, 512),
        (&LO_128, 22_050, 128),
        (&LO_256, 22_050, 256),
    ];

    #[test]
    fn staged_invariants_hold_for_all_eight_partitions() {
        // The staged .meta validation: edge 0 is 0, the final edge is
        // the block's coefficient count, strictly increasing, every
        // edge a multiple of four.
        for (edges, sr, bc) in ALL_HARD {
            assert_eq!(edges[0], 0);
            assert_eq!(*edges.last().unwrap(), bc);
            for w in edges.windows(2) {
                assert!(w[0] < w[1], "sr {sr} bc {bc}");
            }
            assert!(edges.iter().all(|e| e % 4 == 0), "sr {sr} bc {bc}");
            // And the selector resolves to exactly this table.
            assert_eq!(exponent_band_edges(sr, bc), edges.to_vec());
        }
    }

    #[test]
    fn band_counts_span_the_staged_range() {
        // "Band counts differ per block size (10 to 17 in the
        // tabulated cases)" — frame-bit-layout.md §3.
        let counts: Vec<usize> = ALL_HARD
            .iter()
            .map(|&(_, sr, bc)| exponent_band_count(sr, bc))
            .collect();
        assert_eq!(counts.iter().min(), Some(&10));
        assert_eq!(counts.iter().max(), Some(&17));
    }

    #[test]
    fn computed_partition_yields_25_bands_for_a_full_block_at_44100() {
        // The staged cross-check: 2048 coefficients at 44100 Hz →
        // 25 bands (frame-bit-layout.md §3 / the .meta cross-check).
        assert_eq!(exponent_band_count(44_100, 2048), 25);
        let edges = exponent_band_edges(44_100, 2048);
        assert_eq!(edges[0], 0);
        assert_eq!(*edges.last().unwrap(), 2048);
        assert!(edges.iter().all(|e| e % 4 == 0));
    }

    #[test]
    fn sub_22050_streams_always_compute() {
        // 8000 Hz never selects a hard table, any block size.
        for bc in [128u16, 256, 512] {
            let edges = exponent_band_edges(8000, bc);
            assert_eq!(edges[0], 0);
            assert_eq!(*edges.last().unwrap(), bc);
            for w in edges.windows(2) {
                assert!(w[0] < w[1]);
            }
        }
    }

    #[test]
    fn large_blocks_fall_through_to_the_computed_walk() {
        // 1024/2048-coefficient blocks are not tabulated in any arm.
        for sr in [22_050u32, 32_000, 44_100] {
            for bc in [1024u16, 2048] {
                let edges = exponent_band_edges(sr, bc);
                assert_eq!(*edges.last().unwrap(), bc, "sr {sr} bc {bc}");
                assert!(edges.len() > 2);
            }
        }
    }

    #[test]
    fn selector_boundaries_match_the_staged_arms() {
        // hi ≥ 44100; mid 32000–44099; lo 22050–31999.
        assert_eq!(exponent_band_edges(48_000, 128), HI_128.to_vec());
        assert_eq!(exponent_band_edges(44_100, 128), HI_128.to_vec());
        assert_eq!(exponent_band_edges(44_099, 128), MID_128.to_vec());
        assert_eq!(exponent_band_edges(32_000, 128), MID_128.to_vec());
        assert_eq!(exponent_band_edges(31_999, 128), LO_128.to_vec());
        assert_eq!(exponent_band_edges(22_050, 128), LO_128.to_vec());
        assert_ne!(exponent_band_edges(22_049, 128), LO_128.to_vec());
    }
}

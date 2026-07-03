//! Per-block band partitions derived from the staged Hz seed tables.
//!
//! ## Source
//!
//! The staged extraction (`docs/audio/wma/tables/`, provenance 02)
//! pins how the vendor WMA Standard decoder builds its band
//! partitions: the **exponent/quantization-band boundaries are not
//! stored** — a const table of 25 critical-band upper edges in Hz
//! ([`crate::wire_tables::CRITICAL_BAND_FREQS_HZ`]) is walked once
//! per block configuration and "each Hz edge is multiplied by a
//! frequency-per-bin term and rounded into a coefficient-bin index,
//! producing the per-block exponent / quantization-band boundaries".
//! A second, octave-spaced seed
//! ([`crate::wire_tables::SUBBAND_FREQS_HZ`]) is walked the same way
//! by a second band loop (the noise-substitution / high-band gain
//! grid), with its reader starting **past the leading `0`** entry.
//! This matches both the patent trace ("band boundaries are derived,
//! not tabulated") and the wiki snapshot's "compute the scale factor
//! band sizes for each MDCT block size" init step.
//!
//! ## Realization detail (documented, single swap point)
//!
//! An `M`-coefficient block spans `0..sample_rate/2` Hz, so the
//! frequency-per-bin term is `sample_rate / 2M` and an edge maps to
//! `bin = round(freq_hz * 2M / sample_rate)`. The staged notes say
//! "rounded" without pinning the tie behaviour; this module realises
//! **round-half-up in exact integer arithmetic**
//! (`(freq * 2M + sr/2) / sr`). If a validator pass later pins a
//! different tie rule, [`hz_edge_to_bin`] is the one place to swap.
//!
//! Edges are clamped to `M` and collapsed when two adjacent edges
//! round to the same bin (which happens as soon as the sample rate is
//! low enough that several seed edges sit above Nyquist), so the
//! resulting boundary list is strictly increasing and always ends at
//! `M` — a covering partition [`crate::qband::QuantBandLayout`]
//! accepts directly.

use crate::block::BlockSize;
use crate::qband::{QuantBand, QuantBandLayout};
use crate::wire_tables::{CRITICAL_BAND_FREQS_HZ, SUBBAND_FREQS_HZ};

/// Failure modes for the band-derivation helpers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BandDeriveError {
    /// `sample_rate == 0` would make the frequency-per-bin term
    /// ill-defined.
    ZeroSampleRate,
}

impl core::fmt::Display for BandDeriveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BandDeriveError::ZeroSampleRate => {
                f.write_str("oxideav-wma::exponent_bands: sample_rate must be non-zero")
            }
        }
    }
}

impl std::error::Error for BandDeriveError {}

/// Map one Hz edge to a coefficient-bin index for an `M`-coefficient
/// block at `sample_rate`: `round(freq_hz * 2M / sample_rate)`,
/// clamped to `M`.
///
/// Round-half-up in exact integer arithmetic — the module's single
/// documented realization detail (the staged notes say "rounded"
/// without pinning ties).
///
/// # Errors
///
/// [`BandDeriveError::ZeroSampleRate`] when `sample_rate == 0`.
pub fn hz_edge_to_bin(
    freq_hz: u16,
    sample_rate: u32,
    block: BlockSize,
) -> Result<u16, BandDeriveError> {
    if sample_rate == 0 {
        return Err(BandDeriveError::ZeroSampleRate);
    }
    let m = u64::from(block.samples());
    let sr = u64::from(sample_rate);
    let bin = (u64::from(freq_hz) * 2 * m + sr / 2) / sr;
    Ok(u16::try_from(bin.min(m)).expect("clamped to M which fits u16"))
}

/// Scale a strictly-increasing Hz edge seed into the strictly
/// increasing bin-boundary list `[0, .., M]` for one block
/// configuration: scale each edge, clamp to `M`, drop collapsed
/// edges, and close the partition at `M`.
fn derive_boundaries(
    seed: &[u16],
    sample_rate: u32,
    block: BlockSize,
) -> Result<Vec<u16>, BandDeriveError> {
    let m = block.samples();
    let mut boundaries = vec![0u16];
    for &freq in seed {
        let bin = hz_edge_to_bin(freq, sample_rate, block)?;
        let last = *boundaries.last().expect("non-empty by construction");
        if bin > last && bin <= m {
            boundaries.push(bin);
        }
        if bin >= m {
            break;
        }
    }
    if *boundaries.last().expect("non-empty") < m {
        boundaries.push(m);
    }
    Ok(boundaries)
}

/// The per-block **exponent/quantization-band** bin boundaries derived
/// from the critical-band seed: strictly increasing, starting at `0`
/// and ending at `M` (`block.samples()`).
///
/// # Errors
///
/// [`BandDeriveError::ZeroSampleRate`] when `sample_rate == 0`.
pub fn exponent_band_boundaries(
    sample_rate: u32,
    block: BlockSize,
) -> Result<Vec<u16>, BandDeriveError> {
    derive_boundaries(&CRITICAL_BAND_FREQS_HZ, sample_rate, block)
}

/// The per-block **noise / high-band gain** bin boundaries derived
/// from the octave subband seed. Mirrors the vendor reader, which
/// walks the seed from its second entry (the leading `0` is the
/// implicit partition start).
///
/// # Errors
///
/// [`BandDeriveError::ZeroSampleRate`] when `sample_rate == 0`.
pub fn noise_band_boundaries(
    sample_rate: u32,
    block: BlockSize,
) -> Result<Vec<u16>, BandDeriveError> {
    derive_boundaries(&SUBBAND_FREQS_HZ[1..], sample_rate, block)
}

/// Turn a boundary list into the [`QuantBandLayout`] the §4 dequant
/// chain consumes, one weight slot per band in ascending order.
fn layout_from_boundaries(
    boundaries: &[u16],
    block: BlockSize,
) -> Result<QuantBandLayout, BandDeriveError> {
    let bands: Vec<QuantBand> = boundaries
        .windows(2)
        .enumerate()
        .map(|(d, w)| {
            QuantBand::new(
                w[0],
                w[1] - w[0],
                u16::try_from(d).expect("band count fits u16"),
            )
            .expect("derived boundaries are strictly increasing")
        })
        .collect();
    Ok(QuantBandLayout::for_block(bands, block)
        .expect("derived boundaries form a covering tiling of the block"))
}

/// The exponent/quantization-band [`QuantBandLayout`] for one block
/// configuration — the real-data replacement for the caller-invented
/// layouts the §4 chain has used so far. Weight index `d` is the band
/// slot, ascending.
///
/// # Errors
///
/// [`BandDeriveError::ZeroSampleRate`] when `sample_rate == 0`.
pub fn exponent_band_layout(
    sample_rate: u32,
    block: BlockSize,
) -> Result<QuantBandLayout, BandDeriveError> {
    layout_from_boundaries(&exponent_band_boundaries(sample_rate, block)?, block)
}

/// The noise/high-band-gain [`QuantBandLayout`] for one block
/// configuration, derived from the octave subband seed.
///
/// # Errors
///
/// [`BandDeriveError::ZeroSampleRate`] when `sample_rate == 0`.
pub fn noise_band_layout(
    sample_rate: u32,
    block: BlockSize,
) -> Result<QuantBandLayout, BandDeriveError> {
    layout_from_boundaries(&noise_band_boundaries(sample_rate, block)?, block)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The (sample_rate, long block) combinations the header decision
    /// tree actually produces (v2 normalised rates + v1 32 kHz case).
    const REAL_CONFIGS: [(u32, BlockSize); 6] = [
        (8_000, BlockSize::S512),
        (11_025, BlockSize::S512),
        (16_000, BlockSize::S512),
        (22_050, BlockSize::S1024),
        (32_000, BlockSize::S1024),
        (44_100, BlockSize::S2048),
    ];

    #[test]
    fn zero_sample_rate_is_rejected() {
        assert_eq!(
            hz_edge_to_bin(100, 0, BlockSize::S2048),
            Err(BandDeriveError::ZeroSampleRate)
        );
        assert_eq!(
            exponent_band_boundaries(0, BlockSize::S2048),
            Err(BandDeriveError::ZeroSampleRate)
        );
        assert!(format!("{}", BandDeriveError::ZeroSampleRate).contains("non-zero"));
    }

    #[test]
    fn hz_scaling_pins_round_half_up_and_clamp() {
        // 100 Hz at 44.1 kHz / M=2048: 100*4096/44100 = 9.288 -> 9.
        assert_eq!(hz_edge_to_bin(100, 44_100, BlockSize::S2048), Ok(9));
        // 200 Hz: 18.57 -> 19 (rounds up).
        assert_eq!(hz_edge_to_bin(200, 44_100, BlockSize::S2048), Ok(19));
        // Nyquist edge: 22050 Hz maps exactly to M.
        assert_eq!(hz_edge_to_bin(22_050, 44_100, BlockSize::S2048), Ok(2048));
        // Above Nyquist clamps to M.
        assert_eq!(hz_edge_to_bin(24_500, 44_100, BlockSize::S2048), Ok(2048));
        // Exact half tie rounds up: 400 Hz at 16 kHz / M=512:
        // 400*1024/16000 = 25.6 -> 26; and 250*1024/16000 = 16.0 exact.
        assert_eq!(hz_edge_to_bin(250, 16_000, BlockSize::S512), Ok(16));
    }

    #[test]
    fn boundaries_are_strictly_increasing_and_cover_the_block() {
        for &(sr, block) in &REAL_CONFIGS {
            for boundaries in [
                exponent_band_boundaries(sr, block).unwrap(),
                noise_band_boundaries(sr, block).unwrap(),
            ] {
                assert_eq!(boundaries[0], 0, "{sr} Hz {block:?}");
                assert_eq!(
                    *boundaries.last().unwrap(),
                    block.samples(),
                    "{sr} Hz {block:?}"
                );
                for w in boundaries.windows(2) {
                    assert!(w[0] < w[1], "{sr} Hz {block:?}: {boundaries:?}");
                }
            }
        }
    }

    #[test]
    fn full_rate_uses_every_critical_band() {
        // At 44.1 kHz / M=2048 every seed edge below Nyquist maps to a
        // distinct bin, and only the 24 500 Hz cap clamps: 25 bands.
        let b = exponent_band_boundaries(44_100, BlockSize::S2048).unwrap();
        assert_eq!(b.len(), 26);
        // Head pinned by hand against the seed edges.
        assert_eq!(&b[..6], &[0, 9, 19, 28, 37, 47]);
        let layout = exponent_band_layout(44_100, BlockSize::S2048).unwrap();
        assert_eq!(layout.band_count(), 25);
        assert_eq!(layout.total_coeffs(), 2048);
    }

    #[test]
    fn low_rate_collapses_bands_above_nyquist() {
        // At 8 kHz / M=512, Nyquist is 4 kHz: the 17 seed edges below
        // it survive, everything above collapses into the closing edge.
        let b = exponent_band_boundaries(8_000, BlockSize::S512).unwrap();
        assert_eq!(b.len(), 19); // 0, 17 scaled edges, 512
        assert_eq!(b[1], 13); // 100*1024/8000 = 12.8 -> 13
        assert_eq!(b[17], 474); // 3700*1024/8000 = 473.6 -> 474
        assert_eq!(*b.last().unwrap(), 512);
        let layout = exponent_band_layout(8_000, BlockSize::S512).unwrap();
        assert_eq!(layout.band_count(), 18);
    }

    #[test]
    fn noise_bands_derive_from_the_octave_grid() {
        // 44.1 kHz / M=2048: edges 50..12800 all land distinct; the
        // 24 100 Hz cap clamps to M.
        let b = noise_band_boundaries(44_100, BlockSize::S2048).unwrap();
        assert_eq!(b, vec![0, 5, 9, 19, 37, 74, 149, 297, 594, 1189, 2048]);
        let layout = noise_band_layout(44_100, BlockSize::S2048).unwrap();
        assert_eq!(layout.band_count(), 10);
        assert_eq!(layout.total_coeffs(), 2048);
    }

    #[test]
    fn layouts_assign_ascending_weight_slots() {
        let layout = exponent_band_layout(22_050, BlockSize::S1024).unwrap();
        for (d, band) in layout.bands().enumerate() {
            assert_eq!(usize::from(band.weight_index()), d);
        }
        // The band map is usable by the §4 dequant chain directly.
        let map = layout.band_map();
        assert_eq!(map.len(), 1024);
        assert_eq!(map[0], 0);
        assert_eq!(usize::from(*map.last().unwrap()), layout.band_count() - 1);
    }

    #[test]
    fn every_block_size_partitions_cleanly_at_every_real_rate() {
        // Short blocks (transient path) share the same derivation.
        for &(sr, _) in &REAL_CONFIGS {
            for block in BlockSize::ALL {
                let exp = exponent_band_layout(sr, block).unwrap();
                assert_eq!(exp.total_coeffs(), usize::from(block.samples()));
                let noise = noise_band_layout(sr, block).unwrap();
                assert_eq!(noise.total_coeffs(), usize::from(block.samples()));
                assert!(noise.band_count() <= 10);
                assert!(exp.band_count() <= 25);
            }
        }
    }
}

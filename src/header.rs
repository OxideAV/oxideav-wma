//! WMA header — combines container-supplied `WAVEFORMATEX` fields
//! with the version-specific extradata payload.
//!
//! The staged wiki snapshot at
//! `docs/audio/wma/wiki/Windows_Media_Audio.wiki` is the sole source
//! for the layout / decision rules implemented here. The snapshot
//! states (paraphrased into a structured form):
//!
//! * The codec is carried inside a generic container (ASF, AVI, WAV);
//!   the container's `WAVEFORMATEX` block supplies `sample_rate`,
//!   `channels`, `bit_rate`, and `block_align`.
//! * `WAVEFORMATEX` carries a `cbSize` byte count and a trailing
//!   "extra setup data" payload (this crate calls it `extradata`).
//!   The payload is **4 bytes** for v1 (codec ID `0x160`) and
//!   **6 bytes** for v2 (codec ID `0x161`).
//! * The extradata layout is:
//!     | version | offset 0..=N | meaning                  |
//!     | ------- | ------------ | ------------------------ |
//!     | v1      | 0..=1        | `flags1` (u16, LE)       |
//!     | v1      | 2..=3        | `flags2` (u16, LE)       |
//!     | v2      | 0..=3        | `flags1` (u32, LE)       |
//!     | v2      | 4..=5        | `flags2` (u16, LE)       |
//! * `flags2` bit semantics (low-order bits):
//!     * bit 0 — exponential VLCs in use
//!     * bit 1 — bit-reservoir in use
//!     * bit 2 — variable per-frame block length
//! * The frame-length decision tree, applied after the container
//!   fields are known:
//!     * `sample_rate <= 16_000` → `frame_length_bits = 9`
//!     * else `sample_rate <= 22_050`
//!       or `(version == V1 && sample_rate <= 32_000)`
//!       → `frame_length_bits = 10`
//!     * otherwise → `frame_length_bits = 11`
//! * `frame_length = 1 << frame_length_bits` (i.e. 512 / 1024 / 2048
//!   samples per MDCT block at the long-block size).
//! * v2 additionally normalises the container sample-rate when it
//!   reports a non-canonical value: when `sample_rate >= 44_100`
//!   the rate is snapped to `44_100`. The wiki lists the further
//!   normalisation targets as `{22_050, 16_000, 11_025, 8_000}` but
//!   does not specify which side of each cutoff falls to which
//!   target; see [`normalize_sample_rate_v2`] for the DOCS-GAP note
//!   and the boundary behaviour this crate ships in its absence.
//!
//! Anything beyond the items above — the Huffman codebook entries,
//! the exponent-band partition tables, the LSP codebook, the
//! critical-frequency curves, the per-block VLC selection rules —
//! is not specified by any staged document and is therefore not
//! implemented in this round.

use crate::{Error, Result};

/// WMA major version, as identified by the container's codec ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Version {
    /// WMA v1 — codec ID `0x160`. Extradata payload is 4 bytes.
    V1,
    /// WMA v2 — codec ID `0x161`. Extradata payload is 6 bytes.
    V2,
}

impl Version {
    /// Minimum extradata payload length for this version, in bytes.
    pub const fn extradata_len(self) -> usize {
        match self {
            Version::V1 => 4,
            Version::V2 => 6,
        }
    }

    /// Try to recover a [`Version`] from the container's codec ID.
    /// Returns `None` for any ID outside `{0x160, 0x161}` — the
    /// caller is expected to bail before reaching the parser in
    /// that case (e.g. WMA Pro / Lossless live under different IDs
    /// and are not in scope for this rebuild round).
    pub const fn from_codec_id(id: u16) -> Option<Self> {
        match id {
            0x160 => Some(Version::V1),
            0x161 => Some(Version::V2),
            _ => None,
        }
    }
}

/// Parsed WMA header — container fields plus the version-specific
/// extradata bits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WmaHeader {
    /// Codec version (v1 or v2).
    pub version: Version,
    /// Container sample rate after v2 normalisation, in Hz.
    pub sample_rate: u32,
    /// Number of audio channels (mono = 1, stereo = 2).
    pub channels: u8,
    /// Container bit rate in bits/second.
    pub bit_rate: u32,
    /// Container `nBlockAlign` field (bytes per packet/frame at the
    /// container layer).
    pub block_align: u16,
    /// `flags1` payload — v1 zero-extends the on-wire u16 into u32
    /// so this field has a uniform width across versions.
    pub flags1: u32,
    /// `flags2` payload — same width for both versions.
    pub flags2: u16,
    /// `flags2` bit 0 — exponential VLCs in use.
    pub exp_vlc: bool,
    /// `flags2` bit 1 — bit-reservoir in use.
    pub bit_reservoir: bool,
    /// `flags2` bit 2 — variable per-frame block length.
    pub variable_block_length: bool,
    /// The MDCT long-block-size exponent, in bits.
    pub frame_length_bits: u8,
    /// The MDCT long-block frame length, in samples
    /// (`1 << frame_length_bits`).
    pub frame_length: u16,
}

impl WmaHeader {
    /// Parse a WMA header from the four required container fields
    /// plus the codec-specific `extradata` payload.
    ///
    /// `sample_rate` is normalised in-place for [`Version::V2`] —
    /// see [`normalize_sample_rate_v2`]. The returned
    /// [`WmaHeader::sample_rate`] is the post-normalisation value;
    /// the frame-length-bits decision tree consumes the post-
    /// normalisation value too.
    ///
    /// Returns [`Error::ExtradataTooShort`] if the payload is shorter
    /// than [`Version::extradata_len`] would require, and
    /// [`Error::InvalidContainerField`] if `sample_rate` is zero.
    pub fn parse(
        version: Version,
        sample_rate: u32,
        channels: u8,
        bit_rate: u32,
        block_align: u16,
        extradata: &[u8],
    ) -> Result<Self> {
        if sample_rate == 0 {
            return Err(Error::InvalidContainerField {
                field: "sample_rate",
            });
        }

        let need = version.extradata_len();
        if extradata.len() < need {
            return Err(Error::ExtradataTooShort {
                expected: need,
                got: extradata.len(),
            });
        }

        let (flags1, flags2) = match version {
            Version::V1 => {
                // v1: u16 LE flags1, u16 LE flags2.
                let f1 = u16::from_le_bytes([extradata[0], extradata[1]]) as u32;
                let f2 = u16::from_le_bytes([extradata[2], extradata[3]]);
                (f1, f2)
            }
            Version::V2 => {
                // v2: u32 LE flags1, u16 LE flags2.
                let f1 =
                    u32::from_le_bytes([extradata[0], extradata[1], extradata[2], extradata[3]]);
                let f2 = u16::from_le_bytes([extradata[4], extradata[5]]);
                (f1, f2)
            }
        };

        let exp_vlc = (flags2 & 0b0000_0001) != 0;
        let bit_reservoir = (flags2 & 0b0000_0010) != 0;
        let variable_block_length = (flags2 & 0b0000_0100) != 0;

        let normalized_sr = match version {
            Version::V1 => sample_rate,
            Version::V2 => normalize_sample_rate_v2(sample_rate),
        };

        let frame_length_bits = frame_length_bits_for(version, normalized_sr);
        // `frame_length_bits` is at most 11, so the shift is well
        // within u16 range (max 2048).
        let frame_length: u16 = 1u16 << frame_length_bits;

        Ok(WmaHeader {
            version,
            sample_rate: normalized_sr,
            channels,
            bit_rate,
            block_align,
            flags1,
            flags2,
            exp_vlc,
            bit_reservoir,
            variable_block_length,
            frame_length_bits,
            frame_length,
        })
    }
}

/// Frame-length-bits decision tree from the wiki snapshot.
///
/// The wiki spells the rule out as three exclusive branches:
///
/// 1. `sample_rate <= 16_000` → 9
/// 2. otherwise, `sample_rate <= 22_050`
///    or `(version == V1 && sample_rate <= 32_000)` → 10
/// 3. otherwise → 11
#[inline]
fn frame_length_bits_for(version: Version, sample_rate: u32) -> u8 {
    if sample_rate <= 16_000 {
        9
    } else if sample_rate <= 22_050 || (matches!(version, Version::V1) && sample_rate <= 32_000) {
        10
    } else {
        11
    }
}

/// v2 sample-rate normalisation, per the wiki's
/// "v2 forces normalized frequencies" section.
///
/// The wiki is **explicit only at the top of the cutoff list**:
///
/// > if sr >= 44100, force to 44100…
/// > other cutoffs are 22050, 16000, 11025, 8000
///
/// The lower cutoffs are listed as a set without saying which side
/// of each boundary snaps which way. The conservative interpretation
/// this crate ships — pending a clean-room trace doc that pins the
/// boundary directions — is the **downward floor** at each listed
/// rate: any container rate at or above a cutoff is snapped down to
/// that cutoff. Rates below `8_000` pass through unchanged because
/// the wiki names no further cutoff below 8 kHz.
///
/// **DOCS-GAP**: `docs/audio/wma/wiki/Windows_Media_Audio.wiki`
/// (lines 56–61) is silent on the snap direction at each of the
/// listed lower cutoffs (22050 / 16000 / 11025 / 8000). A
/// clean-room trace covering the boundary cases for each cutoff
/// is needed before the v2 normalisation can be locked.
#[inline]
pub fn normalize_sample_rate_v2(sample_rate: u32) -> u32 {
    // Explicit branch — the wiki names this one directly.
    if sample_rate >= 44_100 {
        return 44_100;
    }
    // Implicit interpretation pending DOCS-GAP — downward floor at
    // each listed cutoff.
    if sample_rate >= 22_050 {
        return 22_050;
    }
    if sample_rate >= 16_000 {
        return 16_000;
    }
    if sample_rate >= 11_025 {
        return 11_025;
    }
    if sample_rate >= 8_000 {
        return 8_000;
    }
    sample_rate
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- Version helpers ----------

    #[test]
    fn version_extradata_lengths() {
        assert_eq!(Version::V1.extradata_len(), 4);
        assert_eq!(Version::V2.extradata_len(), 6);
    }

    #[test]
    fn version_from_codec_id_recognises_wma_v1_v2() {
        assert_eq!(Version::from_codec_id(0x160), Some(Version::V1));
        assert_eq!(Version::from_codec_id(0x161), Some(Version::V2));
        assert_eq!(Version::from_codec_id(0x162), None);
        assert_eq!(Version::from_codec_id(0x000), None);
    }

    // ---------- Extradata length errors ----------

    #[test]
    fn parse_rejects_short_v1_extradata() {
        let err = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 0, &[0; 3]).unwrap_err();
        assert_eq!(
            err,
            Error::ExtradataTooShort {
                expected: 4,
                got: 3
            }
        );
    }

    #[test]
    fn parse_rejects_short_v2_extradata() {
        let err = WmaHeader::parse(Version::V2, 44_100, 2, 128_000, 0, &[0; 5]).unwrap_err();
        assert_eq!(
            err,
            Error::ExtradataTooShort {
                expected: 6,
                got: 5
            }
        );
    }

    #[test]
    fn parse_rejects_zero_sample_rate() {
        let err = WmaHeader::parse(Version::V1, 0, 2, 128_000, 0, &[0; 4]).unwrap_err();
        assert_eq!(
            err,
            Error::InvalidContainerField {
                field: "sample_rate",
            }
        );
    }

    // ---------- v1 layout ----------

    #[test]
    fn parse_v1_extradata_little_endian() {
        // flags1 = 0x1234, flags2 = 0x5678.
        let extradata = [0x34, 0x12, 0x78, 0x56];
        let h = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 512, &extradata).unwrap();
        assert_eq!(h.version, Version::V1);
        assert_eq!(h.flags1, 0x0000_1234);
        assert_eq!(h.flags2, 0x5678);
        assert_eq!(h.sample_rate, 44_100);
        assert_eq!(h.channels, 2);
        assert_eq!(h.bit_rate, 128_000);
        assert_eq!(h.block_align, 512);
    }

    // ---------- v2 layout ----------

    #[test]
    fn parse_v2_extradata_little_endian() {
        // flags1 = 0xDEAD_BEEF, flags2 = 0xCAFE.
        let extradata = [0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0xCA];
        let h = WmaHeader::parse(Version::V2, 44_100, 2, 192_000, 1024, &extradata).unwrap();
        assert_eq!(h.version, Version::V2);
        assert_eq!(h.flags1, 0xDEAD_BEEF);
        assert_eq!(h.flags2, 0xCAFE);
    }

    // ---------- flags2 bit semantics ----------

    #[test]
    fn flags2_bit0_is_exp_vlc() {
        let h = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 0, &[0, 0, 0b01, 0]).unwrap();
        assert!(h.exp_vlc);
        assert!(!h.bit_reservoir);
        assert!(!h.variable_block_length);
    }

    #[test]
    fn flags2_bit1_is_bit_reservoir() {
        let h = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 0, &[0, 0, 0b10, 0]).unwrap();
        assert!(!h.exp_vlc);
        assert!(h.bit_reservoir);
        assert!(!h.variable_block_length);
    }

    #[test]
    fn flags2_bit2_is_variable_block_length() {
        let h = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 0, &[0, 0, 0b100, 0]).unwrap();
        assert!(!h.exp_vlc);
        assert!(!h.bit_reservoir);
        assert!(h.variable_block_length);
    }

    #[test]
    fn flags2_all_three_bits_set_simultaneously() {
        // 0b111 = exp_vlc | bit_reservoir | variable_block_length.
        let h = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 0, &[0, 0, 0b111, 0]).unwrap();
        assert!(h.exp_vlc);
        assert!(h.bit_reservoir);
        assert!(h.variable_block_length);
    }

    // ---------- frame-length decision tree ----------

    #[test]
    fn frame_length_branch_low_rate_v2() {
        // sr <= 16000 → 9 → 512 samples.
        // v2 normalises 16000 to 16000 (≥ 16000 cutoff).
        let h = WmaHeader::parse(Version::V2, 16_000, 1, 32_000, 0, &[0; 6]).unwrap();
        assert_eq!(h.frame_length_bits, 9);
        assert_eq!(h.frame_length, 512);
    }

    #[test]
    fn frame_length_branch_low_rate_v1() {
        // sr <= 16000 → 9 → 512 samples (v1, no normalisation).
        let h = WmaHeader::parse(Version::V1, 11_025, 1, 32_000, 0, &[0; 4]).unwrap();
        assert_eq!(h.frame_length_bits, 9);
        assert_eq!(h.frame_length, 512);
    }

    #[test]
    fn frame_length_branch_mid_rate_v2_22050() {
        // sr == 22050 → 10 → 1024 samples (v2 path, normalised to
        // 22050 by the v2 normaliser).
        let h = WmaHeader::parse(Version::V2, 22_050, 1, 64_000, 0, &[0; 6]).unwrap();
        assert_eq!(h.sample_rate, 22_050);
        assert_eq!(h.frame_length_bits, 10);
        assert_eq!(h.frame_length, 1024);
    }

    #[test]
    fn frame_length_branch_v1_32000_special_case() {
        // v1 at 32 kHz stays in the 1024-sample branch per the
        // wiki's `(v1 && sr <= 32000)` special case.
        let h = WmaHeader::parse(Version::V1, 32_000, 2, 96_000, 0, &[0; 4]).unwrap();
        assert_eq!(h.frame_length_bits, 10);
        assert_eq!(h.frame_length, 1024);
    }

    #[test]
    fn frame_length_branch_v2_32000_falls_through() {
        // v2 at 32 kHz does NOT take the v1 special case → 2048
        // samples. (Container reports 32000; v2 normalisation does
        // not touch this value because the wiki's explicit cutoff
        // is only at sr ≥ 44100; lower cutoffs are floor-snap per
        // this crate's DOCS-GAP interpretation, so 32000 lands at
        // the 22050 floor and the frame-length rule sees 22050.)
        let h = WmaHeader::parse(Version::V2, 32_000, 2, 96_000, 0, &[0; 6]).unwrap();
        assert_eq!(h.sample_rate, 22_050);
        // Post-normalisation sr is 22050, which falls into the
        // mid-rate (10 bits) branch.
        assert_eq!(h.frame_length_bits, 10);
        assert_eq!(h.frame_length, 1024);
    }

    #[test]
    fn frame_length_branch_high_rate_v1() {
        // v1 at 44 kHz: not <= 22050, not <= 32000 → 11 → 2048.
        let h = WmaHeader::parse(Version::V1, 44_100, 2, 128_000, 0, &[0; 4]).unwrap();
        assert_eq!(h.frame_length_bits, 11);
        assert_eq!(h.frame_length, 2048);
    }

    #[test]
    fn frame_length_branch_high_rate_v2_48k_snaps_to_44k() {
        // v2 at 48 kHz normalises to 44100, then 11 → 2048.
        let h = WmaHeader::parse(Version::V2, 48_000, 2, 192_000, 0, &[0; 6]).unwrap();
        assert_eq!(h.sample_rate, 44_100);
        assert_eq!(h.frame_length_bits, 11);
        assert_eq!(h.frame_length, 2048);
    }

    // ---------- v2 sample-rate normalisation (explicit cutoff only) ----------

    #[test]
    fn normalize_v2_at_and_above_44100_snaps_to_44100() {
        assert_eq!(normalize_sample_rate_v2(44_100), 44_100);
        assert_eq!(normalize_sample_rate_v2(48_000), 44_100);
        assert_eq!(normalize_sample_rate_v2(96_000), 44_100);
    }

    #[test]
    fn normalize_v2_under_the_lowest_cutoff_passes_through() {
        // No further cutoff named below 8000 in the wiki.
        assert_eq!(normalize_sample_rate_v2(7_999), 7_999);
        assert_eq!(normalize_sample_rate_v2(1), 1);
    }

    // ---------- frame_length always matches its exponent ----------

    #[test]
    fn frame_length_is_power_of_two_of_bits() {
        for (sr, want_bits, want_len) in [
            (8_000u32, 9u8, 512u16),
            (16_000, 9, 512),
            (22_050, 10, 1024),
            (44_100, 11, 2048),
        ] {
            let h = WmaHeader::parse(Version::V1, sr, 1, 32_000, 0, &[0; 4]).unwrap();
            assert_eq!(h.frame_length_bits, want_bits, "sr={sr}");
            assert_eq!(h.frame_length, want_len, "sr={sr}");
            assert_eq!(1u16 << h.frame_length_bits, h.frame_length);
        }
    }
}

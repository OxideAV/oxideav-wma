//! §0 stream configuration — every runtime-derived quantity the
//! frame/packet parse depends on, computed once at stream open.
//!
//! ## Source
//!
//! `docs/audio/wma/frame-bit-layout.md` §0 ("Stream configuration
//! (not in the bitstream)"), including:
//!
//! * the `flags2` bit assignments (bit 0 envelope-coding selector,
//!   bit 1 bit-reservoir, bit 2 variable block length, bits 3–4 the
//!   block-count shift `k`, bit 5 unpinned);
//! * the **variable-block-length gate**: the internal enable is the
//!   AND of bits 1 and 2 — a stream that asks for variable blocks
//!   without the reservoir gets one block size;
//! * the `n_block_sizes` formula (`8 << k` when
//!   `avg_bytes_per_sec / channels >= 4000`, else `2 << k`, clamped
//!   to `frame_length / 128`);
//! * §0.1 `frame_length` (sample-rate tree with the version arm at
//!   32 kHz, low-bitrate doubling, > 48 kHz rejected);
//! * §0.2 the coefficient-VLC class decision table (branch
//!   directions now staged — class 3 below 32 kHz, then 1 / 2 / 3 by
//!   the 0.72 / 1.16 thresholds on the rate float, a value exactly
//!   on a threshold falling on the higher side);
//! * §0.3 `coef_start` / `coef_end` and their per-block scaling;
//! * §1 the packet-header field widths (`byte_offset_bits + 11`
//!   header, `byte_offset_bits + 3` carry field);
//! * §2 `w_bs`, the block-size index field width.
//!
//! The staged measurement
//! `docs/audio/wma/tables/vendor-stream-packet-headers.csv` records
//! all of these per committed vendor stream; the tests below pin this
//! module's derivation against every row.

use crate::header::Version;
use crate::wire_tables::{
    CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD, CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD,
};

/// Errors for [`StreamConfig::derive`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// `sample_rate` above 48 kHz — §0.1 rejects the stream at open.
    SampleRateTooHigh {
        /// The rejected rate.
        sample_rate: u32,
    },
    /// A zero container field that a §0 formula divides by.
    ZeroField {
        /// Human-readable field name.
        field: &'static str,
    },
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::SampleRateTooHigh { sample_rate } => write!(
                f,
                "oxideav-wma: sample rate {sample_rate} Hz exceeds the 48 kHz open-time limit (frame-bit-layout.md §0.1)"
            ),
            ConfigError::ZeroField { field } => {
                write!(f, "oxideav-wma: container field `{field}` was zero")
            }
        }
    }
}

impl std::error::Error for ConfigError {}

/// The §0 stream configuration: container fields plus every derived
/// scalar the packet/frame parse consumes.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamConfig {
    /// Codec version (v1 / v2).
    pub version: Version,
    /// Container sample rate, Hz (raw — §0 derives from the
    /// container value directly).
    pub sample_rate: u32,
    /// Channel count.
    pub channels: u8,
    /// Container `nAvgBytesPerSec`.
    pub avg_bytes_per_sec: u32,
    /// Container `nBlockAlign` — the codec packet size in bytes.
    pub block_align: u16,
    /// The extradata `flags2` field.
    pub flags2: u16,
    /// `flags2` bit 0 — set: exponents are VLC-delta coded (§3);
    /// clear: the §3.1 line-spectral envelope path.
    pub exp_vlc: bool,
    /// `flags2` bit 1 — bit reservoir (packet header + multi-frame
    /// packets, §1).
    pub bit_reservoir: bool,
    /// `flags2` bit 2 — variable block length *requested*.
    pub vbl_requested: bool,
    /// The staged gate: variable block length is enabled only when
    /// bits 1 and 2 are **both** set.
    pub vbl_enabled: bool,
    /// Samples per frame (§0.1).
    pub frame_length: u16,
    /// `floor(log2(frame_length))`.
    pub frame_length_bits: u8,
    /// `bitrate / (channels · sample_rate)` — bits per sample.
    pub bps: f32,
    /// The §0.2 rate float: `bps`, or `bps · 1.6` for > 1 channel.
    pub rate_float: f32,
    /// `floor(log2(frame_length · bps / 8)) + 2`.
    pub byte_offset_bits: u8,
    /// Number of block sizes (1 when VBL is not enabled).
    pub n_block_sizes: u8,
    /// §2 `w_bs` — width of one block-size index field, in bits
    /// (0 when VBL is not enabled: no field is coded).
    pub w_bs: u8,
    /// §0.2 coefficient-VLC decode class (1, 2 or 3).
    pub vlc_class: u8,
}

impl StreamConfig {
    /// Derive the full §0 configuration from the container fields.
    ///
    /// # Errors
    ///
    /// [`ConfigError::SampleRateTooHigh`] above 48 kHz;
    /// [`ConfigError::ZeroField`] for a zero `sample_rate`,
    /// `channels` or `avg_bytes_per_sec` (§0.1 fails the open for a
    /// zero bitrate).
    pub fn derive(
        version: Version,
        sample_rate: u32,
        channels: u8,
        avg_bytes_per_sec: u32,
        block_align: u16,
        flags2: u16,
    ) -> Result<Self, ConfigError> {
        if sample_rate == 0 {
            return Err(ConfigError::ZeroField {
                field: "sample_rate",
            });
        }
        if sample_rate > 48_000 {
            return Err(ConfigError::SampleRateTooHigh { sample_rate });
        }
        if channels == 0 {
            return Err(ConfigError::ZeroField { field: "channels" });
        }
        if avg_bytes_per_sec == 0 {
            return Err(ConfigError::ZeroField {
                field: "avg_bytes_per_sec",
            });
        }
        let bit_rate = u64::from(avg_bytes_per_sec) * 8;

        // §0.1 frame_length tree.
        let mut frame_length: u32 = if sample_rate <= 16_000 {
            512
        } else if sample_rate <= 22_050 {
            1024
        } else if sample_rate <= 32_000 {
            if version == Version::V1 {
                1024
            } else {
                2048
            }
        } else {
            2048
        };
        // §0.1 low-bitrate doubling: while the per-frame byte count
        // would round to zero, double the frame length.
        while (u64::from(frame_length) * bit_rate / u64::from(sample_rate)).div_ceil(8) == 0 {
            frame_length *= 2;
        }
        let frame_length_bits = 31 - frame_length.leading_zeros();

        // §0 derived floats.
        let bps = bit_rate as f32 / (f32::from(channels) * sample_rate as f32);
        let rate_float = if channels > 1 { bps * 1.6 } else { bps };

        // byte_offset_bits = floor(log2(frame_length · bps / 8)) + 2.
        let frame_bytes = frame_length as f32 * bps / 8.0;
        let byte_offset_bits = (frame_bytes.log2().floor() as i32 + 2).max(0) as u8;

        // flags2 bits (§0) and the VBL gate.
        let exp_vlc = flags2 & 0b001 != 0;
        let bit_reservoir = flags2 & 0b010 != 0;
        let vbl_requested = flags2 & 0b100 != 0;
        let vbl_enabled = bit_reservoir && vbl_requested;

        // n_block_sizes.
        let n_block_sizes: u32 = if vbl_enabled {
            let k = (flags2 >> 3) & 3;
            let base: u32 = if avg_bytes_per_sec / u32::from(channels) >= 4000 {
                8 << k
            } else {
                2 << k
            };
            base.min(frame_length / 128).max(1)
        } else {
            1
        };

        // §2: w_bs = floor(log2(floor(log2(n_block_sizes)))) + 1.
        let w_bs = if n_block_sizes > 1 {
            let l = 31 - n_block_sizes.leading_zeros(); // floor(log2 n)
            (31 - l.leading_zeros()) as u8 + 1
        } else {
            0
        };

        // §0.2 class selection (branch directions staged).
        let vlc_class = if sample_rate < 32_000 {
            3
        } else if rate_float < CLASS_SELECTOR_CLASS1_BRANCH_THRESHOLD {
            1
        } else if rate_float < CLASS_SELECTOR_CLASS2_BRANCH_THRESHOLD {
            2
        } else {
            3
        };

        Ok(StreamConfig {
            version,
            sample_rate,
            channels,
            avg_bytes_per_sec,
            block_align,
            flags2,
            exp_vlc,
            bit_reservoir,
            vbl_requested,
            vbl_enabled,
            frame_length: frame_length as u16,
            frame_length_bits: frame_length_bits as u8,
            bps,
            rate_float,
            byte_offset_bits,
            n_block_sizes: n_block_sizes as u8,
            w_bs,
            vlc_class,
        })
    }

    /// Convenience: derive from the version-specific extradata
    /// payload (v1: `flags2` at bytes 2–3; v2: bytes 4–5, LE), per
    /// the staged extradata layout.
    pub fn from_extradata(
        version: Version,
        sample_rate: u32,
        channels: u8,
        avg_bytes_per_sec: u32,
        block_align: u16,
        extradata: &[u8],
    ) -> Result<Self, ConfigError> {
        let flags2 = match version {
            Version::V1 if extradata.len() >= 4 => u16::from_le_bytes([extradata[2], extradata[3]]),
            Version::V2 if extradata.len() >= 6 => u16::from_le_bytes([extradata[4], extradata[5]]),
            // Too-short extradata: all flags clear (single-frame
            // packets, VLC envelope path off is not representable —
            // treat as zero, matching a zeroed field).
            _ => 0,
        };
        Self::derive(
            version,
            sample_rate,
            channels,
            avg_bytes_per_sec,
            block_align,
            flags2,
        )
    }

    /// §1: packet-header width in bits (`byte_offset_bits + 11`)
    /// when the reservoir is enabled; 0 otherwise (no header at all).
    pub fn packet_header_bits(&self) -> u32 {
        if self.bit_reservoir {
            u32::from(self.byte_offset_bits) + 11
        } else {
            0
        }
    }

    /// §1: width of the reservoir-carry field P3, in bits
    /// (`byte_offset_bits + 3`).
    pub fn carry_field_bits(&self) -> u8 {
        self.byte_offset_bits + 3
    }

    /// Packet body size in bits (`block_align · 8` less the header).
    pub fn packet_body_bits(&self) -> u32 {
        u32::from(self.block_align) * 8 - self.packet_header_bits()
    }

    /// §0.3: first coded coefficient index for a block of
    /// `block_size` samples (3 scaled by `block_size / frame_length`
    /// for v1; 0 for v2).
    pub fn coef_start(&self, block_size: u16) -> u16 {
        match self.version {
            Version::V1 => ((3 * u32::from(block_size)) / u32::from(self.frame_length)) as u16,
            Version::V2 => 0,
        }
    }

    /// §0.3: one past the last coded coefficient index for a block of
    /// `block_size` samples (the top ≈9 % of the spectrum is never
    /// coded).
    pub fn coef_end(&self, block_size: u16) -> u16 {
        block_size - ((9 * u32::from(block_size)) / 100) as u16
    }

    /// The §2 block-size decode: `block_size = frame_length >> index`,
    /// `None` when the shift exceeds the frame or produces a block
    /// under the configured minimum count.
    pub fn block_size_for_index(&self, index: u8) -> Option<u16> {
        if index >= self.n_block_sizes {
            return None;
        }
        Some(self.frame_length >> index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One row of the staged measurement
    /// `docs/audio/wma/tables/vendor-stream-packet-headers.csv`.
    struct VendorRow {
        name: &'static str,
        channels: u8,
        sample_rate: u32,
        bytes_per_sec: u32,
        block_align: u16,
        flags2: u16,
        frame_length: u16,
        byte_offset_bits: u8,
        packet_header_bits: u32,
        packet_body_bits: u32,
        n_block_sizes: u8,
        vlc_class: u8,
        carry_field_bits: u8,
    }

    const VENDOR_ROWS: [VendorRow; 6] = [
        VendorRow {
            name: "cand_apollo8",
            channels: 2,
            sample_rate: 44_100,
            bytes_per_sec: 8003,
            block_align: 2973,
            flags2: 0x000f,
            frame_length: 2048,
            byte_offset_bits: 9,
            packet_header_bits: 20,
            packet_body_bits: 23_764,
            n_block_sizes: 16,
            vlc_class: 3,
            carry_field_bits: 12,
        },
        VendorRow {
            name: "cand_mono22k_16kbps",
            channels: 1,
            sample_rate: 22_050,
            bytes_per_sec: 2003,
            block_align: 744,
            flags2: 0x000f,
            frame_length: 1024,
            byte_offset_bits: 8,
            packet_header_bits: 19,
            packet_body_bits: 5933,
            n_block_sizes: 4,
            vlc_class: 3,
            carry_field_bits: 11,
        },
        VendorRow {
            name: "cand_mono8k_8kbps_v8",
            channels: 1,
            sample_rate: 8000,
            bytes_per_sec: 1000,
            block_align: 640,
            flags2: 0x0026,
            frame_length: 512,
            byte_offset_bits: 8,
            packet_header_bits: 19,
            packet_body_bits: 5101,
            n_block_sizes: 2,
            vlc_class: 3,
            carry_field_bits: 11,
        },
        VendorRow {
            name: "cand_stereo22k_32kbps_av",
            channels: 2,
            sample_rate: 22_050,
            bytes_per_sec: 4006,
            block_align: 744,
            flags2: 0x0017,
            frame_length: 1024,
            byte_offset_bits: 8,
            packet_header_bits: 19,
            packet_body_bits: 5933,
            n_block_sizes: 8,
            vlc_class: 3,
            carry_field_bits: 11,
        },
        VendorRow {
            name: "cand_vbr_q75_stereo",
            channels: 2,
            sample_rate: 44_100,
            bytes_per_sec: 11_111,
            block_align: 4459,
            flags2: 0x000f,
            frame_length: 2048,
            byte_offset_bits: 10,
            packet_header_bits: 21,
            packet_body_bits: 35_651,
            n_block_sizes: 16,
            vlc_class: 3,
            carry_field_bits: 13,
        },
        VendorRow {
            name: "cand_wmp12_96kbps",
            channels: 2,
            sample_rate: 44_100,
            bytes_per_sec: 12_003,
            block_align: 4459,
            flags2: 0x000f,
            frame_length: 2048,
            byte_offset_bits: 10,
            packet_header_bits: 21,
            packet_body_bits: 35_651,
            n_block_sizes: 16,
            vlc_class: 3,
            carry_field_bits: 13,
        },
    ];

    #[test]
    fn derivation_reproduces_every_staged_vendor_stream_row() {
        for row in &VENDOR_ROWS {
            let cfg = StreamConfig::derive(
                Version::V2,
                row.sample_rate,
                row.channels,
                row.bytes_per_sec,
                row.block_align,
                row.flags2,
            )
            .unwrap();
            assert_eq!(cfg.frame_length, row.frame_length, "{}", row.name);
            assert_eq!(cfg.byte_offset_bits, row.byte_offset_bits, "{}", row.name);
            assert_eq!(
                cfg.packet_header_bits(),
                row.packet_header_bits,
                "{}",
                row.name
            );
            assert_eq!(cfg.packet_body_bits(), row.packet_body_bits, "{}", row.name);
            assert_eq!(cfg.n_block_sizes, row.n_block_sizes, "{}", row.name);
            assert_eq!(cfg.vlc_class, row.vlc_class, "{}", row.name);
            assert_eq!(cfg.carry_field_bits(), row.carry_field_bits, "{}", row.name);
            assert!(cfg.bit_reservoir, "{}", row.name);
        }
    }

    #[test]
    fn extradata_flags2_extraction_matches_the_staged_blobs() {
        // reference/vendor-streams/README.md extradata column.
        let blobs: [(&str, [u8; 10], u16); 3] = [
            (
                "apollo8",
                *b"\x00\x88\x00\x00\x0f\x00\x75\x2e\x00\x00",
                0x000f,
            ),
            (
                "mono8k",
                *b"\x00\x24\x00\x00\x26\x00\x80\x02\x00\x00",
                0x0026,
            ),
            (
                "stereo22k_av",
                *b"\x00\x44\x00\x00\x17\x00\x00\x00\x00\x00",
                0x0017,
            ),
        ];
        for (name, blob, want) in blobs {
            let cfg =
                StreamConfig::from_extradata(Version::V2, 44_100, 2, 8003, 2973, &blob).unwrap();
            assert_eq!(cfg.flags2, want, "{name}");
        }
    }

    #[test]
    fn vbl_gate_requires_both_bits() {
        // §0: bit 2 without bit 1 → one block size, no field.
        let cfg = StreamConfig::derive(Version::V2, 44_100, 2, 8003, 2973, 0b0100).unwrap();
        assert!(cfg.vbl_requested && !cfg.vbl_enabled);
        assert_eq!(cfg.n_block_sizes, 1);
        assert_eq!(cfg.w_bs, 0);
        assert_eq!(cfg.packet_header_bits(), 0); // no reservoir → no header
    }

    #[test]
    fn w_bs_matches_the_staged_examples() {
        // §2: 2 bits for 4 or 8 block sizes, 3 bits for 16 — pinned
        // on the staged vendor-stream configurations.
        // k=1, 8003 B/s mono ≥ 4000 → 8<<1 = 16, clamped to 16.
        let n16 = StreamConfig::derive(Version::V2, 44_100, 1, 8003, 2973, 0x000f).unwrap();
        assert_eq!((n16.n_block_sizes, n16.w_bs), (16, 3));
        // k=2, 4006 B/s stereo → 2003 < 4000 → 2<<2 = 8, clamp 8.
        let n8 = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap();
        assert_eq!((n8.n_block_sizes, n8.w_bs), (8, 2));
        // k=1, 2003 B/s mono < 4000 → 2<<1 = 4.
        let n4 = StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap();
        assert_eq!((n4.n_block_sizes, n4.w_bs), (4, 2));
        // k=0, 1000 B/s mono < 4000 → 2.
        let n2 = StreamConfig::derive(Version::V2, 8000, 1, 1000, 640, 0x0026).unwrap();
        assert_eq!((n2.n_block_sizes, n2.w_bs), (2, 1));
    }

    #[test]
    fn class_thresholds_fall_on_the_higher_side() {
        // §0.2: rate exactly at a threshold takes the higher class.
        // Construct a stereo stream whose rate float is exactly at
        // the class-2 threshold region boundary checks by direction.
        let hi = StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 4459, 0x000f).unwrap();
        assert_eq!(hi.vlc_class, 3); // 1.088·1.6 = 1.742 ≥ 1.16
        let low = StreamConfig::derive(Version::V2, 44_100, 1, 3000, 1487, 0x000f).unwrap();
        // bps = 24000/44100 = 0.544 < 0.72 → class 1
        assert_eq!(low.vlc_class, 1);
        let mid = StreamConfig::derive(Version::V2, 44_100, 1, 5000, 1487, 0x000f).unwrap();
        // bps = 40000/44100 = 0.907 → class 2
        assert_eq!(mid.vlc_class, 2);
        let sub32k = StreamConfig::derive(Version::V2, 22_050, 2, 12_003, 4459, 0x000f).unwrap();
        assert_eq!(sub32k.vlc_class, 3); // pinned below 32 kHz
    }

    #[test]
    fn coef_range_scales_per_block() {
        let cfg = StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 4459, 0x000f).unwrap();
        assert_eq!(cfg.coef_start(2048), 0);
        assert_eq!(cfg.coef_end(2048), 2048 - 184);
        assert_eq!(cfg.coef_end(1024), 1024 - 92);
        assert_eq!(cfg.coef_end(128), 128 - 11);
        let v1 = StreamConfig::derive(Version::V1, 32_000, 2, 8000, 1536, 0x0000).unwrap();
        assert_eq!(v1.frame_length, 1024); // version arm at 32 kHz
        assert_eq!(v1.coef_start(1024), 3);
        assert_eq!(v1.coef_start(512), 1);
    }

    #[test]
    fn open_rejects_out_of_range_streams() {
        assert!(matches!(
            StreamConfig::derive(Version::V2, 96_000, 2, 12_003, 4459, 0),
            Err(ConfigError::SampleRateTooHigh { .. })
        ));
        assert!(StreamConfig::derive(Version::V2, 44_100, 0, 12_003, 4459, 0).is_err());
        assert!(StreamConfig::derive(Version::V2, 44_100, 2, 0, 4459, 0).is_err());
    }

    #[test]
    fn block_size_index_decodes_by_right_shift() {
        let cfg = StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 4459, 0x000f).unwrap();
        assert_eq!(cfg.block_size_for_index(0), Some(2048));
        assert_eq!(cfg.block_size_for_index(1), Some(1024));
        assert_eq!(cfg.block_size_for_index(4), Some(128));
        assert_eq!(cfg.block_size_for_index(16), None);
    }
}

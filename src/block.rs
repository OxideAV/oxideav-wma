//! WMA block-size primitive — the patent-disclosed long-block
//! transform set for WMA Standard (WMA7, codec IDs `0x160`/`0x161`).
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §2 cites
//! **US7,930,171 Background (WMA7 prior-art description)**:
//!
//! > For the WMA7 prior-art codec specifically, Chen-171 states the
//! > block sizes are drawn from the set
//! > **{256, 512, 1024, 2048, 4096} samples.**
//!
//! That five-element set is the single patent-disclosed structural
//! fact about the per-block transform sizes used by WMA Standard.
//! This module exposes the set as a typed enum so downstream
//! transform code can refer to a block size without re-checking the
//! patent each time.
//!
//! ## Scope of this round
//!
//! Round 1 landed [`crate::header::WmaHeader`] with a `frame_length`
//! field whose only valid values were 512 / 1024 / 2048 (the long-
//! block sizes the wiki orientation document maps from sample rate).
//! That field is the *long-block* selection; the patent set adds the
//! two outer sizes (256 and 4096) used when transient detection
//! switches the block size for the current frame, per §2:
//!
//! > "Small blocks allow for greater preservation of time detail"
//! > (transients); "large blocks have better frequency resolution".
//!
//! The patents disclose the *set* and the *rationale*; they do not
//! disclose the bitstream syntax that signals which of the five sizes
//! is in use for a given block. That signalling is **[GAP]** in the
//! trace doc and is therefore not implemented here — this module is a
//! typed primitive only.
//!
//! ## Why a closed enum, not a `u16`
//!
//! A `u16` would let callers construct (and tests pass) any of the
//! 65 536 possible widths. The patent says the set has exactly five
//! members, so the type captures that constraint: any code path that
//! holds a [`BlockSize`] holds a value the patent disclosed.
//! [`BlockSize::from_samples`] is the one entry point that performs
//! the validation.

use crate::{Error, Result};

/// One of the five patent-disclosed WMA Standard transform-block
/// sizes, measured in time-domain samples per block.
///
/// The set is fixed by **US7,930,171 (Chen-171), Background**:
/// `{256, 512, 1024, 2048, 4096}`. No other value is valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum BlockSize {
    /// 256-sample block (`log2 = 8`). The smallest size; used when
    /// transient detection chooses maximum time resolution.
    S256,
    /// 512-sample block (`log2 = 9`).
    S512,
    /// 1024-sample block (`log2 = 10`).
    S1024,
    /// 2048-sample block (`log2 = 11`).
    S2048,
    /// 4096-sample block (`log2 = 12`). The largest size; used when
    /// the encoder prefers frequency resolution over time resolution.
    S4096,
}

impl BlockSize {
    /// The full patent-disclosed set, in ascending order. Useful for
    /// iteration in tests and for code that wants to enumerate the
    /// possibilities (e.g. block-switching decision tables).
    pub const ALL: [BlockSize; 5] = [
        BlockSize::S256,
        BlockSize::S512,
        BlockSize::S1024,
        BlockSize::S2048,
        BlockSize::S4096,
    ];

    /// Number of time-domain samples in a block of this size.
    pub const fn samples(self) -> u16 {
        match self {
            BlockSize::S256 => 256,
            BlockSize::S512 => 512,
            BlockSize::S1024 => 1024,
            BlockSize::S2048 => 2048,
            BlockSize::S4096 => 4096,
        }
    }

    /// `log2(samples)` — i.e. the bit-width of the block-size
    /// exponent. The patent set spans `{8, 9, 10, 11, 12}`.
    pub const fn log2_samples(self) -> u8 {
        match self {
            BlockSize::S256 => 8,
            BlockSize::S512 => 9,
            BlockSize::S1024 => 10,
            BlockSize::S2048 => 11,
            BlockSize::S4096 => 12,
        }
    }

    /// Try to build a [`BlockSize`] from a sample count. Returns
    /// [`Error::InvalidBlockSize`] for any value outside the
    /// patent-disclosed set.
    pub const fn from_samples(samples: u16) -> Result<Self> {
        match samples {
            256 => Ok(BlockSize::S256),
            512 => Ok(BlockSize::S512),
            1024 => Ok(BlockSize::S1024),
            2048 => Ok(BlockSize::S2048),
            4096 => Ok(BlockSize::S4096),
            other => Err(Error::InvalidBlockSize { samples: other }),
        }
    }

    /// Try to build a [`BlockSize`] from an MDCT exponent (i.e. the
    /// `frame_length_bits` field that [`crate::header::WmaHeader`]
    /// already exposes). The patent set spans `log2 ∈ {8..=12}`.
    pub const fn from_log2(log2_samples: u8) -> Result<Self> {
        match log2_samples {
            8 => Ok(BlockSize::S256),
            9 => Ok(BlockSize::S512),
            10 => Ok(BlockSize::S1024),
            11 => Ok(BlockSize::S2048),
            12 => Ok(BlockSize::S4096),
            other => Err(Error::InvalidBlockSize {
                // Round the failing exponent back into samples for
                // the error message — saturate at u16::MAX rather
                // than wrapping, since exponents > 15 would overflow.
                samples: if other >= 16 { u16::MAX } else { 1u16 << other },
            }),
        }
    }

    /// `true` when this size is the smallest patent-disclosed block,
    /// i.e. the one used at maximum time resolution.
    pub const fn is_shortest(self) -> bool {
        matches!(self, BlockSize::S256)
    }

    /// `true` when this size is the largest patent-disclosed block,
    /// i.e. the one used at maximum frequency resolution.
    pub const fn is_longest(self) -> bool {
        matches!(self, BlockSize::S4096)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- ALL: the patent-disclosed set ----------

    #[test]
    fn all_contains_the_five_patent_disclosed_sizes() {
        // US7,930,171 Background lists exactly these five values.
        let samples: Vec<u16> = BlockSize::ALL.iter().map(|b| b.samples()).collect();
        assert_eq!(samples, vec![256, 512, 1024, 2048, 4096]);
    }

    #[test]
    fn all_is_strictly_ascending() {
        // Iteration order matters for any consumer that does a
        // "smallest size that fits" walk; lock it down.
        for w in BlockSize::ALL.windows(2) {
            assert!(w[0].samples() < w[1].samples(), "{:?} !< {:?}", w[0], w[1]);
        }
    }

    // ---------- samples() / log2_samples() round-trip ----------

    #[test]
    fn samples_is_power_of_two_of_log2() {
        for b in BlockSize::ALL {
            assert_eq!(1u16 << b.log2_samples(), b.samples(), "{b:?}");
        }
    }

    #[test]
    fn log2_set_is_8_through_12() {
        let log2s: Vec<u8> = BlockSize::ALL.iter().map(|b| b.log2_samples()).collect();
        assert_eq!(log2s, vec![8, 9, 10, 11, 12]);
    }

    // ---------- from_samples ----------

    #[test]
    fn from_samples_accepts_every_patent_value() {
        for b in BlockSize::ALL {
            assert_eq!(BlockSize::from_samples(b.samples()), Ok(b));
        }
    }

    #[test]
    fn from_samples_rejects_powers_of_two_outside_the_set() {
        // 128 (one below the set) and 8192 (one above) are
        // mathematically reasonable transform sizes but the patent
        // does NOT name them — the typed constructor must refuse.
        assert_eq!(
            BlockSize::from_samples(128),
            Err(Error::InvalidBlockSize { samples: 128 }),
        );
        assert_eq!(
            BlockSize::from_samples(8192),
            Err(Error::InvalidBlockSize { samples: 8192 }),
        );
    }

    #[test]
    fn from_samples_rejects_non_power_of_two() {
        // 1000 sits between 512 and 1024 but is not a power of two.
        assert_eq!(
            BlockSize::from_samples(1000),
            Err(Error::InvalidBlockSize { samples: 1000 }),
        );
    }

    #[test]
    fn from_samples_rejects_zero() {
        assert_eq!(
            BlockSize::from_samples(0),
            Err(Error::InvalidBlockSize { samples: 0 }),
        );
    }

    // ---------- from_log2 ----------

    #[test]
    fn from_log2_accepts_every_patent_exponent() {
        for b in BlockSize::ALL {
            assert_eq!(BlockSize::from_log2(b.log2_samples()), Ok(b));
        }
    }

    #[test]
    fn from_log2_rejects_exponent_below_set() {
        // log2 = 7 → 128 samples, just below the set.
        assert_eq!(
            BlockSize::from_log2(7),
            Err(Error::InvalidBlockSize { samples: 128 }),
        );
    }

    #[test]
    fn from_log2_rejects_exponent_above_set() {
        // log2 = 13 → 8192 samples, just above the set.
        assert_eq!(
            BlockSize::from_log2(13),
            Err(Error::InvalidBlockSize { samples: 8192 }),
        );
    }

    #[test]
    fn from_log2_saturates_on_huge_exponents() {
        // Exponents that would shift a u16 out of range saturate to
        // u16::MAX in the error message — the diagnostic should not
        // panic for absurd input.
        assert_eq!(
            BlockSize::from_log2(64),
            Err(Error::InvalidBlockSize { samples: u16::MAX }),
        );
    }

    // ---------- Outer-bound predicates ----------

    #[test]
    fn is_shortest_only_for_s256() {
        for b in BlockSize::ALL {
            assert_eq!(b.is_shortest(), b == BlockSize::S256, "{b:?}");
        }
    }

    #[test]
    fn is_longest_only_for_s4096() {
        for b in BlockSize::ALL {
            assert_eq!(b.is_longest(), b == BlockSize::S4096, "{b:?}");
        }
    }

    // ---------- Cross-module: header.frame_length lands in the set ----------

    #[test]
    fn header_long_block_frame_lengths_are_patent_subset() {
        // Round 1's WmaHeader::frame_length range is {512, 1024, 2048}
        // (the long-block selections drawn by the wiki sample-rate
        // tree). All three must be members of the patent set so that
        // a future block.rs consumer can wrap a header-supplied frame
        // length into a BlockSize without an extra validation table.
        use crate::header::{Version, WmaHeader};

        let cases = [
            (Version::V1, 11_025u32, BlockSize::S512),
            (Version::V1, 22_050, BlockSize::S1024),
            (Version::V1, 44_100, BlockSize::S2048),
            (Version::V2, 22_050, BlockSize::S1024),
            (Version::V2, 48_000, BlockSize::S2048),
        ];

        for (v, sr, want) in cases {
            let extra_len = v.extradata_len();
            let h = WmaHeader::parse(v, sr, 1, 32_000, 0, &vec![0u8; extra_len]).unwrap();
            let got = BlockSize::from_samples(h.frame_length).unwrap();
            assert_eq!(got, want, "version={v:?} sr={sr}");
        }
    }
}

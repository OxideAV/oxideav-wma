//! WMA entropy-coder mode selector.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 lifts the
//! patent-disclosed mode-switching arrangement that the WMA Standard
//! entropy coder uses across the spectrum:
//!
//! > **Mode switching.** The encoder switches between a **level mode**
//! > and a **run-length/level mode** depending on the sub-range
//! > (low-frequency mostly-non-zero range vs high-frequency
//! > mostly-zero range), selected by a mode selector across N
//! > predefined sub-coders.
//! >   — [PATENT US6,223,162 — mode selector 400, encoders 402–406]
//! >   — [PATENT US7,383,180 — entropy encoder 570 "switches between
//! >      level and run length/level modes"]
//!
//! > After quantization the spectrum is dominated by zeros (especially
//! > at high frequencies). The entropy stage is built around that
//! > statistic: "coefficients are most likely non-zero at lower
//! > frequency ranges, and mostly zero at higher frequencies."
//! >   — [PATENT US6,223,162 — FIG.3/FIG.4]
//!
//! > **Partition / flag overhead.** The boundary between sub-ranges may
//! > be predetermined (no overhead) or adaptive, in which case a flag
//! > is embedded to indicate the change of applicable coder.
//! >   — [PATENT US6,223,162 — partition 306; adaptive-flag discussion]
//!
//! ## Scope of this module
//!
//! This module exposes the entropy-mode selector as a typed enum and a
//! [`Partition`] descriptor that names the boundary between the two
//! sub-ranges. Both pieces are patent-disclosed at the structural
//! level (the *existence* of the modes, the *existence* of the
//! partition between them, and the *direction* of the assignment —
//! level mode for the low-frequency, mostly-non-zero range, run-level
//! mode for the high-frequency, mostly-zero range).
//!
//! The decision *rule* that chooses the boundary (transient analysis,
//! masking-driven heuristics, etc.) is an encoder analysis detail and
//! is **not** patent-disclosed at the bit level — the [`Partition`]
//! type here is the carrier, not the decision algorithm.
//!
//! ## What is NOT in this module
//!
//! * **The codeword tables.** Per §6 of the trace, the Huffman code
//!   books for both modes are `[GAP]`. This module produces no
//!   codewords.
//! * **The boundary-flag bit width.** The patent text says the
//!   adaptive boundary is signalled by "a flag … to indicate the
//!   change of applicable coder", but the bit width is `[GAP]`.
//!   [`Partition::is_adaptive`] reports the kind, not a wire layout.
//! * **The N predefined sub-coders.** US6,223,162 names N as "across
//!   N predefined sub-coders" without fixing N for WMA v1/v2. We
//!   model only the two modes the trace explicitly names; richer
//!   sub-coder tables would extend [`EntropyMode`] when a trace stages
//!   them.

/// Entropy-coding mode for a contiguous sub-range of spectral
/// coefficients within a WMA Standard block.
///
/// Per §6 of the trace, the WMA Standard entropy coder switches
/// between two patent-disclosed modes:
///
/// * [`EntropyMode::Level`] — used in the low-frequency, mostly-non-
///   zero sub-range, where the symbol is the coefficient level itself.
/// * [`EntropyMode::RunLevel`] — used in the high-frequency, mostly-
///   zero sub-range, where the symbol is the joint `(R, L)` run-level
///   pairing defined in [`crate::runlevel`].
///
/// The names mirror the patent's "level mode" and "run length/level
/// mode" terminology verbatim.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EntropyMode {
    /// Level mode — symbols are coefficient levels. Patent-named as
    /// optimal for the low-frequency, mostly-non-zero sub-range.
    Level,
    /// Run-length/level mode — symbols are joint `(R, L)` pairings.
    /// Patent-named as optimal for the high-frequency, mostly-zero
    /// sub-range.
    RunLevel,
}

impl EntropyMode {
    /// All patent-disclosed entropy modes, in their natural order
    /// (low-frequency mode first, high-frequency mode second).
    pub const ALL: [EntropyMode; 2] = [EntropyMode::Level, EntropyMode::RunLevel];

    /// Returns the *other* entropy mode — i.e. the mode the coder
    /// switches to when crossing the partition boundary.
    pub const fn opposite(self) -> EntropyMode {
        match self {
            EntropyMode::Level => EntropyMode::RunLevel,
            EntropyMode::RunLevel => EntropyMode::Level,
        }
    }

    /// `true` for the patent's low-frequency, mostly-non-zero mode.
    pub const fn is_level(self) -> bool {
        matches!(self, EntropyMode::Level)
    }

    /// `true` for the patent's high-frequency, mostly-zero mode.
    pub const fn is_run_level(self) -> bool {
        matches!(self, EntropyMode::RunLevel)
    }
}

/// Partition descriptor naming the boundary between the low-frequency
/// and high-frequency sub-ranges of a block.
///
/// Per the patent the partition is one of two kinds:
///
/// * **Predetermined** — both encoder and decoder know the boundary
///   from setup-time configuration; no bits are spent on signalling.
/// * **Adaptive** — the boundary is recomputed per block (or
///   per-frame) and the decoder is told via an embedded flag.
///
/// In both cases the partition carries an explicit boundary index
/// `split`: coefficient indices `0..split` are coded in the
/// low-frequency mode, indices `split..total` are coded in the
/// high-frequency mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Partition {
    /// Total number of coefficients in the block this partition
    /// describes.
    pub total_coeffs: u32,
    /// First coefficient index of the high-frequency sub-range. May
    /// equal `0` (entire block is high-frequency) or `total_coeffs`
    /// (entire block is low-frequency).
    pub split: u32,
    /// Whether the boundary is adaptive (`true`) or predetermined
    /// (`false`).
    pub adaptive: bool,
}

/// Construction-time rejection for [`Partition::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidPartition {
    /// `split > total_coeffs`. The boundary is outside the block.
    SplitOutOfBlock {
        /// The rejected split index.
        split: u32,
        /// The block's total coefficient count.
        total_coeffs: u32,
    },
}

impl core::fmt::Display for InvalidPartition {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidPartition::SplitOutOfBlock {
                split,
                total_coeffs,
            } => write!(
                f,
                "oxideav-wma::entropy_mode: partition split {split} is outside block of {total_coeffs} coefficients",
            ),
        }
    }
}

impl std::error::Error for InvalidPartition {}

impl Partition {
    /// Try to build a partition. Returns
    /// [`InvalidPartition::SplitOutOfBlock`] when the boundary lies
    /// strictly outside `0..=total_coeffs`.
    pub const fn new(
        total_coeffs: u32,
        split: u32,
        adaptive: bool,
    ) -> core::result::Result<Self, InvalidPartition> {
        if split > total_coeffs {
            return Err(InvalidPartition::SplitOutOfBlock {
                split,
                total_coeffs,
            });
        }
        Ok(Partition {
            total_coeffs,
            split,
            adaptive,
        })
    }

    /// `true` when the partition's boundary is signalled per-block by
    /// an embedded flag (patent-disclosed adaptive mode).
    #[inline]
    pub const fn is_adaptive(self) -> bool {
        self.adaptive
    }

    /// `true` when the partition's boundary is fixed at setup time and
    /// not signalled in the bitstream (patent-disclosed predetermined
    /// mode).
    #[inline]
    pub const fn is_predetermined(self) -> bool {
        !self.adaptive
    }

    /// Return the [`EntropyMode`] that should code coefficient at
    /// position `index`. Coefficients in `0..split` are coded in
    /// [`EntropyMode::Level`]; coefficients in `split..total_coeffs`
    /// are coded in [`EntropyMode::RunLevel`].
    ///
    /// Returns `None` when `index >= total_coeffs`.
    #[inline]
    pub const fn mode_for(self, index: u32) -> Option<EntropyMode> {
        if index >= self.total_coeffs {
            None
        } else if index < self.split {
            Some(EntropyMode::Level)
        } else {
            Some(EntropyMode::RunLevel)
        }
    }

    /// Number of coefficients carried by the low-frequency sub-range
    /// (coded in [`EntropyMode::Level`]).
    #[inline]
    pub const fn level_range_len(self) -> u32 {
        self.split
    }

    /// Number of coefficients carried by the high-frequency sub-range
    /// (coded in [`EntropyMode::RunLevel`]).
    #[inline]
    pub const fn run_level_range_len(self) -> u32 {
        self.total_coeffs - self.split
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- EntropyMode ----------

    #[test]
    fn entropy_mode_all_contains_both_patent_modes() {
        assert_eq!(EntropyMode::ALL.len(), 2);
        assert!(EntropyMode::ALL.contains(&EntropyMode::Level));
        assert!(EntropyMode::ALL.contains(&EntropyMode::RunLevel));
    }

    #[test]
    fn entropy_mode_low_frequency_first_in_all() {
        // The trace lists "level mode" first (low-frequency, mostly-
        // non-zero); lock the iteration order so a future
        // partition-walker can rely on it.
        assert_eq!(EntropyMode::ALL[0], EntropyMode::Level);
        assert_eq!(EntropyMode::ALL[1], EntropyMode::RunLevel);
    }

    #[test]
    fn entropy_mode_opposite_is_involutive() {
        for m in EntropyMode::ALL {
            assert_eq!(m.opposite().opposite(), m, "involution failed for {m:?}");
            assert_ne!(m.opposite(), m, "opposite must differ from {m:?}");
        }
    }

    #[test]
    fn entropy_mode_predicates_are_exclusive() {
        for m in EntropyMode::ALL {
            assert!(m.is_level() ^ m.is_run_level(), "{m:?}");
        }
    }

    // ---------- Partition construction ----------

    #[test]
    fn partition_new_accepts_split_at_zero() {
        // split = 0 → entire block is high-frequency (run-level mode).
        let p = Partition::new(2048, 0, false).unwrap();
        assert_eq!(p.split, 0);
        assert_eq!(p.level_range_len(), 0);
        assert_eq!(p.run_level_range_len(), 2048);
    }

    #[test]
    fn partition_new_accepts_split_at_total() {
        // split = total → entire block is low-frequency (level mode).
        let p = Partition::new(2048, 2048, false).unwrap();
        assert_eq!(p.split, 2048);
        assert_eq!(p.level_range_len(), 2048);
        assert_eq!(p.run_level_range_len(), 0);
    }

    #[test]
    fn partition_new_accepts_mid_split() {
        let p = Partition::new(2048, 512, true).unwrap();
        assert_eq!(p.level_range_len(), 512);
        assert_eq!(p.run_level_range_len(), 1536);
    }

    #[test]
    fn partition_new_rejects_split_beyond_block() {
        let err = Partition::new(2048, 2049, false).unwrap_err();
        assert_eq!(
            err,
            InvalidPartition::SplitOutOfBlock {
                split: 2049,
                total_coeffs: 2048,
            }
        );
    }

    #[test]
    fn partition_new_rejects_far_out_of_block() {
        let err = Partition::new(1024, u32::MAX, true).unwrap_err();
        assert_eq!(
            err,
            InvalidPartition::SplitOutOfBlock {
                split: u32::MAX,
                total_coeffs: 1024,
            }
        );
    }

    // ---------- Partition: adaptive vs predetermined ----------

    #[test]
    fn partition_adaptive_and_predetermined_are_complementary() {
        let p_adapt = Partition::new(1024, 256, true).unwrap();
        assert!(p_adapt.is_adaptive());
        assert!(!p_adapt.is_predetermined());

        let p_fixed = Partition::new(1024, 256, false).unwrap();
        assert!(!p_fixed.is_adaptive());
        assert!(p_fixed.is_predetermined());
    }

    // ---------- Partition: mode_for index lookup ----------

    #[test]
    fn partition_mode_for_low_index_is_level() {
        let p = Partition::new(1024, 256, false).unwrap();
        assert_eq!(p.mode_for(0), Some(EntropyMode::Level));
        assert_eq!(p.mode_for(255), Some(EntropyMode::Level));
    }

    #[test]
    fn partition_mode_for_high_index_is_run_level() {
        let p = Partition::new(1024, 256, false).unwrap();
        // split = 256 → index 256 is the first run-level slot.
        assert_eq!(p.mode_for(256), Some(EntropyMode::RunLevel));
        assert_eq!(p.mode_for(1023), Some(EntropyMode::RunLevel));
    }

    #[test]
    fn partition_mode_for_out_of_range_is_none() {
        let p = Partition::new(1024, 256, false).unwrap();
        assert_eq!(p.mode_for(1024), None);
        assert_eq!(p.mode_for(u32::MAX), None);
    }

    #[test]
    fn partition_mode_for_split_at_zero_is_all_run_level() {
        let p = Partition::new(64, 0, false).unwrap();
        for i in 0..64 {
            assert_eq!(p.mode_for(i), Some(EntropyMode::RunLevel), "i={i}");
        }
    }

    #[test]
    fn partition_mode_for_split_at_total_is_all_level() {
        let p = Partition::new(64, 64, false).unwrap();
        for i in 0..64 {
            assert_eq!(p.mode_for(i), Some(EntropyMode::Level), "i={i}");
        }
    }

    // ---------- Partition: range-length accounting ----------

    #[test]
    fn partition_range_lengths_sum_to_total() {
        for split in [0u32, 1, 256, 1023, 1024] {
            let p = Partition::new(1024, split, false).unwrap();
            assert_eq!(p.level_range_len() + p.run_level_range_len(), 1024);
        }
    }

    // ---------- Cross-module: pairs with the BlockSize set ----------

    #[test]
    fn partition_can_be_built_for_every_patent_block_size() {
        // Every patent-disclosed long-block size should accept a
        // partition; pair the entropy mode selector with the
        // BlockSize primitive that round 2 landed.
        use crate::block::BlockSize;
        for b in BlockSize::ALL {
            let total = b.samples() as u32;
            let p = Partition::new(total, total / 2, true).unwrap();
            assert_eq!(p.total_coeffs, total);
            assert_eq!(p.level_range_len(), total / 2);
            assert_eq!(p.run_level_range_len(), total - total / 2);
        }
    }
}

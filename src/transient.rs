//! WMA per-block transient-handling switch.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §3 lifts the
//! patent-disclosed existence of a per-block side-information switch
//! the WMA Standard decoder must consume in order to apply
//! transient-aware handling within a frame. Two patent-backed
//! mechanisms are named for the same role:
//!
//! > Malvar-380 / Malvar-126 disclose a **one-bit side-information
//! > flag, sent per block,** that switches the combining of
//! > high-frequency subbands on/off for better time resolution on
//! > transients — computed *after* the MLT, so "there is no need to
//! > switch the MLT window functions or block size M." The encoder
//! > enables it when high-frequency power exceeds low-frequency power
//! > by a threshold.
//! >   — [PATENT US6,240,380 — FIG.12, boxes 1210–1250; one-bit flag]
//! >   — [PATENT US6,029,126 — FIG.12]
//!
//! > Whether WMA Standard v1/v2 uses *this* subband-combining flag, or
//! > the alternative of literally switching window/block size from the
//! > {256…4096} set (Chen-171), is `[GAP]`: both mechanisms are
//! > Microsoft-patented and both achieve transient handling; the
//! > patents do not state which one the shipping v1/v2 bitstream uses.
//! > The *existence of a per-block transient-handling switch signalled
//! > as side information* is patent-backed; its exact form in v1/v2 is
//! > not.
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §3
//!
//! ## Scope of this module
//!
//! This module exposes the patent-disclosed **existence** of the
//! per-block transient switch as a typed primitive a downstream
//! reader can carry, without committing to either of the two
//! mechanisms the patents disclose for it:
//!
//! * [`TransientMechanism`] names the two patent-disclosed mechanism
//!   choices side-by-side ([`TransientMechanism::SubbandCombineFlag`]
//!   matches US6,240,380 FIG.12 / US6,029,126 FIG.12;
//!   [`TransientMechanism::BlockSizeSwitch`] matches the US7,930,171
//!   Background `{256, 512, 1024, 2048, 4096}` set lifted in
//!   [`crate::block::BlockSize`]).
//! * [`TransientSwitch`] carries one decoded side-information value
//!   for one block: which mechanism it selects, and (for the
//!   subband-combining mechanism) the one-bit decision; (for the
//!   block-size-switching mechanism) the chosen [`BlockSize`].
//! * [`TransientPlan`] carries a per-block sequence of switches across
//!   a frame, with predicate counts for downstream code that needs to
//!   reason about how many blocks in the frame were transient-handled.
//!
//! The plan is a carrier, **not** a wire-format reader. The patent
//! disclosures fix the *existence* of these switches and the *two
//! mechanism alternatives*; the v1/v2 bitstream bit-positions of the
//! flag (or of the block-size selector) are `[GAP]` per §3 of the
//! trace and are therefore not implemented here.
//!
//! ## What is NOT in this module
//!
//! * **The bit-level encoding of the side-information flag.** Both
//!   mechanisms are signalled "as side information" per §3 of the
//!   trace, but neither patent fixes the v1/v2 byte-level layout.
//!   This module accepts an already-decoded [`TransientSwitch`] from
//!   upstream.
//! * **The encoder decision rule.** "Enabled when high-frequency power
//!   exceeds low-frequency power by a threshold" (US6,240,380) is an
//!   encoder analysis detail; it does not change the bitstream and is
//!   not modelled. The plan accepts whatever decision the upstream
//!   reader decoded.
//! * **The high-frequency subband-combining transform itself.** §3 of
//!   the trace cites US6,240,380 FIG.12 boxes 1210–1250 for the
//!   combining operation. This module carries only the on/off
//!   *switch*; the combining operation belongs in a future transform
//!   module that lands when the relevant trace section is staged.
//! * **A choice between the two mechanisms for v1/v2.** §3 explicitly
//!   marks the v1/v2 choice as `[GAP]`. The plan therefore lets the
//!   caller select either, and downstream code can be written against
//!   the mechanism a future trace pins down.

use crate::block::BlockSize;

/// Which patent-disclosed mechanism a per-block transient-handling
/// switch uses.
///
/// Per §3 of the trace doc, both mechanisms achieve transient
/// handling and both are Microsoft-patented; v1/v2 has not been
/// pinned to one or the other from the patent corpus alone. Modelling
/// both side-by-side lets downstream code adapt when a future trace
/// fixes the choice without re-organising the carrier type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TransientMechanism {
    /// One-bit side-information flag, sent per block, that switches
    /// the high-frequency subband combining on/off. Computed *after*
    /// the MLT, so no window/block-size change is needed.
    /// [PATENT US6,240,380 — FIG.12 boxes 1210–1250]
    /// [PATENT US6,029,126 — FIG.12]
    SubbandCombineFlag,
    /// Block-size-switching mechanism: the encoder picks a block size
    /// from the patent-disclosed `{256, 512, 1024, 2048, 4096}` set
    /// based on transient detection. Small blocks preserve time
    /// detail; large blocks give better frequency resolution.
    /// [PATENT US7,930,171 — Background WMA7 description]
    BlockSizeSwitch,
}

impl TransientMechanism {
    /// Every mechanism variant, in declaration order, for callers
    /// that need to enumerate the patent-disclosed alternatives.
    pub const ALL: [TransientMechanism; 2] = [
        TransientMechanism::SubbandCombineFlag,
        TransientMechanism::BlockSizeSwitch,
    ];

    /// `true` iff this is the US6,240,380 / US6,029,126
    /// subband-combine-flag mechanism.
    #[inline]
    pub const fn is_subband_combine_flag(self) -> bool {
        matches!(self, TransientMechanism::SubbandCombineFlag)
    }

    /// `true` iff this is the US7,930,171 block-size-switching
    /// mechanism.
    #[inline]
    pub const fn is_block_size_switch(self) -> bool {
        matches!(self, TransientMechanism::BlockSizeSwitch)
    }
}

/// One decoded per-block transient-handling switch.
///
/// The variant identifies which patent-disclosed mechanism the
/// upstream reader decoded; the inner data carries the patent-named
/// payload for that mechanism. Per §3 of the trace the *existence* of
/// such a switch is patent-backed; the *bit-level encoding* is
/// `[GAP]`, so this type is a carrier and not a wire-format reader.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TransientSwitch {
    /// The block uses the US6,240,380 / US6,029,126 subband-combining
    /// mechanism. The `combine_high_subbands` field is the decoded
    /// one-bit flag: `true` means the decoder must combine the
    /// high-frequency subbands for this block (transient-aware path);
    /// `false` means the default non-combining path is used.
    SubbandCombineFlag {
        /// Decoded value of the one-bit per-block flag.
        combine_high_subbands: bool,
    },
    /// The block uses the US7,930,171 block-size-switching mechanism.
    /// The `block_size` field is the chosen member of the
    /// patent-disclosed `{256, 512, 1024, 2048, 4096}` set.
    BlockSizeSwitch {
        /// Chosen block size from the patent-disclosed set.
        block_size: BlockSize,
    },
}

impl TransientSwitch {
    /// The patent-disclosed mechanism this switch carries.
    #[inline]
    pub const fn mechanism(self) -> TransientMechanism {
        match self {
            TransientSwitch::SubbandCombineFlag { .. } => TransientMechanism::SubbandCombineFlag,
            TransientSwitch::BlockSizeSwitch { .. } => TransientMechanism::BlockSizeSwitch,
        }
    }

    /// `true` iff this switch indicates that the block was
    /// transient-handled.
    ///
    /// For the subband-combining mechanism this is the literal flag
    /// value: `true` means high-frequency subband combining was
    /// enabled for the block (US6,240,380 FIG.12). For the
    /// block-size-switching mechanism this is `true` iff the chosen
    /// block size is **not** the patent-disclosed longest size
    /// (4096 samples) — any shorter size is the encoder having
    /// switched away from the long block for time-resolution
    /// reasons, per §2 of the trace.
    pub fn is_transient_handled(self) -> bool {
        match self {
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands,
            } => combine_high_subbands,
            TransientSwitch::BlockSizeSwitch { block_size } => !block_size.is_longest(),
        }
    }

    /// Return the chosen [`BlockSize`] for switches that carry one,
    /// or `None` for switches whose mechanism does not record a
    /// block-size choice.
    #[inline]
    pub const fn block_size(self) -> Option<BlockSize> {
        match self {
            TransientSwitch::BlockSizeSwitch { block_size } => Some(block_size),
            TransientSwitch::SubbandCombineFlag { .. } => None,
        }
    }

    /// Return the decoded one-bit subband-combine flag for switches
    /// that carry one, or `None` for switches whose mechanism does
    /// not record such a flag.
    #[inline]
    pub const fn subband_combine_flag(self) -> Option<bool> {
        match self {
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands,
            } => Some(combine_high_subbands),
            TransientSwitch::BlockSizeSwitch { .. } => None,
        }
    }
}

/// Per-frame plan of per-block transient-handling switches.
///
/// The plan models the patent-disclosed structure of a frame as "one
/// or more blocks" (§2 of the trace) by pairing each block with its
/// decoded [`TransientSwitch`]. All switches in a plan share a single
/// [`TransientMechanism`] — both patent-disclosed mechanisms exist
/// for the same role but they are not mixed within a frame in any
/// description either patent gives; mixing them would be a property
/// of a third, undisclosed bitstream signal that §3 of the trace does
/// not stage.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransientPlan {
    mechanism: TransientMechanism,
    switches: Vec<TransientSwitch>,
}

/// Construction failure mode for [`TransientPlan::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvalidTransientPlan {
    /// One of the supplied switches reported a mechanism that did not
    /// match the plan's declared mechanism. The offending index in
    /// the switches slice is reported.
    MechanismMismatch {
        /// Index in the input slice of the first switch whose
        /// mechanism did not match the plan's mechanism.
        at_block: usize,
        /// Mechanism the plan was constructed with.
        expected: TransientMechanism,
        /// Mechanism the offending switch carried.
        got: TransientMechanism,
    },
}

impl core::fmt::Display for InvalidTransientPlan {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidTransientPlan::MechanismMismatch {
                at_block,
                expected,
                got,
            } => write!(
                f,
                "oxideav-wma::transient: switch at block {at_block} uses mechanism {got:?} but the plan was constructed for {expected:?}",
            ),
        }
    }
}

impl std::error::Error for InvalidTransientPlan {}

impl TransientPlan {
    /// Build a plan from a mechanism choice and a per-block switch
    /// table. Returns
    /// [`InvalidTransientPlan::MechanismMismatch`] if any switch's
    /// own mechanism disagrees with the plan-wide mechanism.
    pub fn new(
        mechanism: TransientMechanism,
        switches: Vec<TransientSwitch>,
    ) -> Result<Self, InvalidTransientPlan> {
        for (idx, s) in switches.iter().enumerate() {
            let got = s.mechanism();
            if got != mechanism {
                return Err(InvalidTransientPlan::MechanismMismatch {
                    at_block: idx,
                    expected: mechanism,
                    got,
                });
            }
        }
        Ok(Self {
            mechanism,
            switches,
        })
    }

    /// The mechanism every switch in the plan uses.
    #[inline]
    pub const fn mechanism(&self) -> TransientMechanism {
        self.mechanism
    }

    /// Number of blocks the plan covers.
    #[inline]
    pub fn len(&self) -> usize {
        self.switches.len()
    }

    /// `true` iff the plan covers zero blocks.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.switches.is_empty()
    }

    /// Look up the switch for block `idx`, or `None` if out of range.
    pub fn switch_of(&self, idx: usize) -> Option<TransientSwitch> {
        self.switches.get(idx).copied()
    }

    /// Iterate over every switch in declaration order.
    pub fn switches(&self) -> impl Iterator<Item = TransientSwitch> + '_ {
        self.switches.iter().copied()
    }

    /// Count of blocks whose [`TransientSwitch::is_transient_handled`]
    /// reports `true`.
    pub fn transient_handled_block_count(&self) -> usize {
        self.switches
            .iter()
            .filter(|s| s.is_transient_handled())
            .count()
    }

    /// Count of blocks whose [`TransientSwitch::is_transient_handled`]
    /// reports `false`.
    pub fn non_transient_block_count(&self) -> usize {
        self.switches.len() - self.transient_handled_block_count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- TransientMechanism ----------

    #[test]
    fn mechanism_all_lists_both_patent_disclosed_variants() {
        let all = TransientMechanism::ALL;
        assert_eq!(all.len(), 2);
        assert!(all.contains(&TransientMechanism::SubbandCombineFlag));
        assert!(all.contains(&TransientMechanism::BlockSizeSwitch));
    }

    #[test]
    fn mechanism_predicates_are_exhaustive_and_disjoint() {
        for m in TransientMechanism::ALL {
            let a = m.is_subband_combine_flag();
            let b = m.is_block_size_switch();
            assert!(a ^ b, "{m:?} predicates not disjoint");
        }
    }

    // ---------- TransientSwitch ----------

    #[test]
    fn subband_combine_switch_reports_subband_mechanism() {
        let s = TransientSwitch::SubbandCombineFlag {
            combine_high_subbands: true,
        };
        assert_eq!(s.mechanism(), TransientMechanism::SubbandCombineFlag);
    }

    #[test]
    fn block_size_switch_reports_block_size_mechanism() {
        let s = TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S256,
        };
        assert_eq!(s.mechanism(), TransientMechanism::BlockSizeSwitch);
    }

    #[test]
    fn subband_switch_is_transient_handled_iff_flag_true() {
        let on = TransientSwitch::SubbandCombineFlag {
            combine_high_subbands: true,
        };
        let off = TransientSwitch::SubbandCombineFlag {
            combine_high_subbands: false,
        };
        assert!(on.is_transient_handled());
        assert!(!off.is_transient_handled());
    }

    #[test]
    fn block_size_switch_is_transient_handled_iff_not_longest() {
        // Any size other than the longest (4096) counts as transient
        // handling per §2 rationale: smaller blocks are selected for
        // time-resolution preservation.
        assert!(TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S256,
        }
        .is_transient_handled());
        assert!(TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S512,
        }
        .is_transient_handled());
        assert!(TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S1024,
        }
        .is_transient_handled());
        assert!(TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S2048,
        }
        .is_transient_handled());
        assert!(!TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S4096,
        }
        .is_transient_handled());
    }

    #[test]
    fn subband_switch_carries_no_block_size() {
        let s = TransientSwitch::SubbandCombineFlag {
            combine_high_subbands: true,
        };
        assert_eq!(s.block_size(), None);
    }

    #[test]
    fn block_size_switch_carries_block_size() {
        let s = TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S2048,
        };
        assert_eq!(s.block_size(), Some(BlockSize::S2048));
    }

    #[test]
    fn subband_switch_carries_subband_flag() {
        for combine in [true, false] {
            let s = TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: combine,
            };
            assert_eq!(s.subband_combine_flag(), Some(combine));
        }
    }

    #[test]
    fn block_size_switch_carries_no_subband_flag() {
        let s = TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S256,
        };
        assert_eq!(s.subband_combine_flag(), None);
    }

    // ---------- TransientPlan ----------

    #[test]
    fn empty_plan_has_zero_blocks() {
        let plan = TransientPlan::new(TransientMechanism::SubbandCombineFlag, vec![]).unwrap();
        assert_eq!(plan.len(), 0);
        assert!(plan.is_empty());
        assert_eq!(plan.transient_handled_block_count(), 0);
        assert_eq!(plan.non_transient_block_count(), 0);
    }

    #[test]
    fn plan_mechanism_is_recorded() {
        let plan = TransientPlan::new(TransientMechanism::BlockSizeSwitch, vec![]).unwrap();
        assert_eq!(plan.mechanism(), TransientMechanism::BlockSizeSwitch);
    }

    #[test]
    fn plan_switch_of_returns_the_input() {
        let switches = vec![
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: false,
            },
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: true,
            },
        ];
        let plan =
            TransientPlan::new(TransientMechanism::SubbandCombineFlag, switches.clone()).unwrap();
        assert_eq!(plan.len(), 2);
        assert_eq!(plan.switch_of(0), Some(switches[0]));
        assert_eq!(plan.switch_of(1), Some(switches[1]));
        assert_eq!(plan.switch_of(2), None);
    }

    #[test]
    fn plan_switches_iterates_in_declaration_order() {
        let switches = vec![
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S256,
            },
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S4096,
            },
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S512,
            },
        ];
        let plan =
            TransientPlan::new(TransientMechanism::BlockSizeSwitch, switches.clone()).unwrap();
        let collected: Vec<_> = plan.switches().collect();
        assert_eq!(collected, switches);
    }

    #[test]
    fn plan_counts_partition_block_population() {
        let switches = vec![
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: true,
            },
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: false,
            },
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: true,
            },
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: false,
            },
        ];
        let plan = TransientPlan::new(TransientMechanism::SubbandCombineFlag, switches).unwrap();
        assert_eq!(plan.transient_handled_block_count(), 2);
        assert_eq!(plan.non_transient_block_count(), 2);
        assert_eq!(
            plan.transient_handled_block_count() + plan.non_transient_block_count(),
            plan.len()
        );
    }

    #[test]
    fn plan_with_block_size_mechanism_counts_non_longest_as_transient() {
        let switches = vec![
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S4096,
            },
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S256,
            },
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S4096,
            },
        ];
        let plan = TransientPlan::new(TransientMechanism::BlockSizeSwitch, switches).unwrap();
        assert_eq!(plan.transient_handled_block_count(), 1);
        assert_eq!(plan.non_transient_block_count(), 2);
    }

    #[test]
    fn plan_rejects_mechanism_mismatch_at_first_offender() {
        // First two switches match; third does not.
        let switches = vec![
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: true,
            },
            TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: false,
            },
            TransientSwitch::BlockSizeSwitch {
                block_size: BlockSize::S1024,
            },
        ];
        let err = TransientPlan::new(TransientMechanism::SubbandCombineFlag, switches).unwrap_err();
        assert_eq!(
            err,
            InvalidTransientPlan::MechanismMismatch {
                at_block: 2,
                expected: TransientMechanism::SubbandCombineFlag,
                got: TransientMechanism::BlockSizeSwitch,
            }
        );
    }

    #[test]
    fn plan_rejects_block_size_switch_in_subband_plan_at_index_zero() {
        let switches = vec![TransientSwitch::BlockSizeSwitch {
            block_size: BlockSize::S256,
        }];
        let err = TransientPlan::new(TransientMechanism::SubbandCombineFlag, switches).unwrap_err();
        match err {
            InvalidTransientPlan::MechanismMismatch { at_block, .. } => {
                assert_eq!(at_block, 0);
            }
        }
    }

    #[test]
    fn plan_accepts_homogeneous_subband_population() {
        let switches: Vec<_> = (0..16)
            .map(|i| TransientSwitch::SubbandCombineFlag {
                combine_high_subbands: i % 2 == 0,
            })
            .collect();
        let plan = TransientPlan::new(TransientMechanism::SubbandCombineFlag, switches).unwrap();
        assert_eq!(plan.len(), 16);
        assert_eq!(plan.transient_handled_block_count(), 8);
    }

    #[test]
    fn plan_accepts_homogeneous_block_size_population() {
        let switches: Vec<_> = BlockSize::ALL
            .into_iter()
            .map(|bs| TransientSwitch::BlockSizeSwitch { block_size: bs })
            .collect();
        let plan = TransientPlan::new(TransientMechanism::BlockSizeSwitch, switches).unwrap();
        assert_eq!(plan.len(), BlockSize::ALL.len());
        // Exactly one of the five sizes is the longest, so four
        // blocks should be reported as transient-handled.
        assert_eq!(plan.transient_handled_block_count(), 4);
        assert_eq!(plan.non_transient_block_count(), 1);
    }

    // ---------- Error formatting ----------

    #[test]
    fn invalid_transient_plan_display_names_module_and_block() {
        let err = InvalidTransientPlan::MechanismMismatch {
            at_block: 7,
            expected: TransientMechanism::SubbandCombineFlag,
            got: TransientMechanism::BlockSizeSwitch,
        };
        let msg = format!("{err}");
        assert!(msg.contains("oxideav-wma::transient"));
        assert!(msg.contains("block 7"));
        assert!(msg.contains("SubbandCombineFlag"));
        assert!(msg.contains("BlockSizeSwitch"));
    }

    #[test]
    fn invalid_transient_plan_implements_error() {
        fn assert_error<E: std::error::Error>(_: &E) {}
        let err = InvalidTransientPlan::MechanismMismatch {
            at_block: 0,
            expected: TransientMechanism::SubbandCombineFlag,
            got: TransientMechanism::BlockSizeSwitch,
        };
        assert_error(&err);
    }

    // ---------- Cross-module: block-size set comes from BlockSize ----------

    #[test]
    fn block_size_switch_can_carry_every_block_size_variant() {
        for bs in BlockSize::ALL {
            let s = TransientSwitch::BlockSizeSwitch { block_size: bs };
            assert_eq!(s.block_size(), Some(bs));
            assert_eq!(s.mechanism(), TransientMechanism::BlockSizeSwitch);
        }
    }
}

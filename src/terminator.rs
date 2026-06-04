//! WMA spectral-stream end-of-block terminator selector.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §6 lifts the
//! patent-disclosed structure of how the WMA Standard entropy stream
//! signals the end of a block's coefficient sequence. The trace
//! discloses two patent-backed alternatives side by side:
//!
//! > **End of block / end of stream.** Termination uses "either a
//! > special ending signal… or a special event such as `(N, 1)`"
//! > because the decoder knows the total coefficient count for the
//! > block.
//! >   — [PATENT US6,223,162 — end-of-stream discussion]
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §6
//!
//! ## Scope of this module
//!
//! This module exposes the patent-disclosed **selector** between the
//! two named termination mechanisms as a typed primitive, plus a
//! per-block carrier that records the concrete decision the upstream
//! reader committed to for one block:
//!
//! * [`TerminatorMechanism`] names the two patent-disclosed mechanism
//!   choices side-by-side:
//!   * [`TerminatorMechanism::ExplicitEndingSignal`] — the patent's
//!     "special ending signal" branch. The entropy stream carries a
//!     distinguished symbol that the decoder recognises as
//!     end-of-block; no implicit `(N, 1)` event is relied on.
//!   * [`TerminatorMechanism::ImplicitNL1Event`] — the patent's "the
//!     decoder knows the total coefficient count for the block" branch.
//!     The last pair encountered satisfies the
//!     [`RunLevelPair::is_implicit_terminator_for`] predicate
//!     (`run == remaining_coeffs`, `level == 1`), so no extra symbol
//!     is needed.
//! * [`TerminatorDecision`] is the per-block carrier whose two variants
//!   mirror [`TerminatorMechanism`]. The `ExplicitEndingSignal` variant
//!   has no payload (the wire-format symbol itself is `[GAP]`); the
//!   `ImplicitNL1Event` variant carries the `(N, 1)` pair the upstream
//!   reader recognised as the implicit terminator.
//! * [`TerminatorMechanism::is_compatible_with`] checks at construction
//!   time that a candidate `(R, L)` pair satisfies the patent's
//!   `(N, 1)` predicate before the upstream reader commits to the
//!   implicit branch for a block of known length.
//!
//! ## What is NOT in this module
//!
//! * **The explicit-ending-signal bit width or symbol pattern.** §6 of
//!   the trace establishes the *existence* of the explicit branch but
//!   states the v1/v2 wire-format symbol is `[GAP]`. The variant has
//!   no payload — when a future trace section pins the symbol, it
//!   becomes the responsibility of the bitstream reader to recognise
//!   the symbol and emit
//!   [`TerminatorDecision::ExplicitEndingSignal`].
//! * **Which mechanism v1/v2 actually uses.** §6 cites both
//!   alternatives without pinning the v1/v2 choice; this module
//!   carries both for the same reason [`crate::transient`] carries
//!   both transient-handling mechanisms.
//! * **Per-frame plans across multiple blocks.** Each block in a frame
//!   is independent at the terminator level (the implicit `(N, 1)`
//!   event consults each block's own `total_coeffs`). The
//!   `terminator` module therefore exposes per-block decisions; a
//!   per-frame plan over them would be a thin `Vec<TerminatorDecision>`
//!   wrapper a future bitstream reader can add at its layer.

use crate::runlevel::RunLevelPair;

/// Which patent-disclosed mechanism a per-block end-of-block
/// terminator uses.
///
/// Per §6 of the trace, both mechanisms terminate the coefficient
/// stream and both are patent-backed; v1/v2 is not pinned to one or
/// the other from the patent corpus alone. Modelling both side by
/// side lets downstream code adapt when a future trace fixes the
/// choice without re-organising the carrier type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TerminatorMechanism {
    /// The "special ending signal" branch. The entropy stream emits a
    /// distinguished symbol that the decoder recognises as
    /// end-of-block. The symbol pattern and bit width are `[GAP]` per
    /// §6 of the trace.
    /// [PATENT US6,223,162 — "either a special ending signal …"]
    ExplicitEndingSignal,
    /// The implicit `(N, 1)` event branch. The decoder knows the
    /// block's total coefficient count `N`; the final `(R, L)` pair
    /// satisfies `R == remaining`, `L == 1`, so the decoder recognises
    /// end-of-block from the coefficient-count accounting alone.
    /// [PATENT US6,223,162 — "a special event such as `(N, 1)` because
    /// the decoder knows the total coefficient count"]
    ImplicitNL1Event,
}

impl TerminatorMechanism {
    /// Every mechanism variant, in declaration order, for callers that
    /// need to enumerate the patent-disclosed alternatives.
    pub const ALL: [TerminatorMechanism; 2] = [
        TerminatorMechanism::ExplicitEndingSignal,
        TerminatorMechanism::ImplicitNL1Event,
    ];

    /// `true` iff this is the patent's "special ending signal" branch.
    #[inline]
    pub const fn is_explicit_ending_signal(self) -> bool {
        matches!(self, TerminatorMechanism::ExplicitEndingSignal)
    }

    /// `true` iff this is the patent's implicit `(N, 1)` event branch.
    #[inline]
    pub const fn is_implicit_n_l1_event(self) -> bool {
        matches!(self, TerminatorMechanism::ImplicitNL1Event)
    }

    /// Returns the *other* mechanism — the alternative the upstream
    /// reader would have committed to had the chosen mechanism been
    /// the other one. Useful for downstream code that needs to
    /// represent the binary mechanism switch as a flip.
    #[inline]
    pub const fn opposite(self) -> TerminatorMechanism {
        match self {
            TerminatorMechanism::ExplicitEndingSignal => TerminatorMechanism::ImplicitNL1Event,
            TerminatorMechanism::ImplicitNL1Event => TerminatorMechanism::ExplicitEndingSignal,
        }
    }

    /// Patent-compatibility check between a mechanism choice and a
    /// candidate final `(R, L)` pair for a block whose total
    /// coefficient count is `total_coeffs`.
    ///
    /// * For [`TerminatorMechanism::ImplicitNL1Event`] the pair must
    ///   satisfy the patent's `(N, 1)` predicate — i.e. its
    ///   [`RunLevelPair::is_implicit_terminator_for`] must return `true`
    ///   against `total_coeffs`. Otherwise the upstream reader has
    ///   misclassified the terminator.
    /// * For [`TerminatorMechanism::ExplicitEndingSignal`] the
    ///   coefficient stream's last pair has no patent-disclosed
    ///   structural constraint (the patent's text places the
    ///   constraint on a distinct "special ending signal", not on the
    ///   final `(R, L)`). The method therefore reports `true`
    ///   unconditionally for that branch — the explicit-signal
    ///   compatibility lives entirely in the (still-`[GAP]`) symbol
    ///   pattern, not in the `(R, L)` neighbourhood.
    #[inline]
    pub fn is_compatible_with(self, pair: RunLevelPair, total_coeffs: u64) -> bool {
        match self {
            TerminatorMechanism::ExplicitEndingSignal => true,
            TerminatorMechanism::ImplicitNL1Event => pair.is_implicit_terminator_for(total_coeffs),
        }
    }
}

/// One per-block end-of-block terminator decision.
///
/// The variant identifies which patent-disclosed mechanism the
/// upstream reader committed to for one block; the inner data carries
/// the patent-named payload for that mechanism. Per §6 of the trace
/// the *existence* of these two alternatives is patent-backed; the
/// explicit-signal *symbol pattern* is `[GAP]` and is therefore not
/// stored in the explicit variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TerminatorDecision {
    /// The block was terminated by the patent's "special ending
    /// signal" branch. No payload: the symbol pattern is `[GAP]` per
    /// §6 and is the responsibility of the bitstream reader.
    ExplicitEndingSignal,
    /// The block was terminated by the patent's implicit `(N, 1)`
    /// event. The pair carried by the variant is the `(R, L)` the
    /// reader recognised as the implicit terminator; it satisfies the
    /// patent's `R == remaining`, `L == 1` predicate against the
    /// block's `total_coeffs`.
    ImplicitNL1Event {
        /// The `(N, 1)` pair the upstream reader recognised as the
        /// implicit terminator.
        terminator_pair: RunLevelPair,
    },
}

impl TerminatorDecision {
    /// The patent-disclosed mechanism this decision carries.
    #[inline]
    pub const fn mechanism(self) -> TerminatorMechanism {
        match self {
            TerminatorDecision::ExplicitEndingSignal => TerminatorMechanism::ExplicitEndingSignal,
            TerminatorDecision::ImplicitNL1Event { .. } => TerminatorMechanism::ImplicitNL1Event,
        }
    }

    /// Return the implicit-terminator pair for decisions that carry
    /// one, or `None` for decisions whose mechanism does not record
    /// such a pair.
    #[inline]
    pub const fn terminator_pair(self) -> Option<RunLevelPair> {
        match self {
            TerminatorDecision::ImplicitNL1Event { terminator_pair } => Some(terminator_pair),
            TerminatorDecision::ExplicitEndingSignal => None,
        }
    }

    /// `true` iff this decision is the explicit-ending-signal branch.
    #[inline]
    pub const fn is_explicit_ending_signal(self) -> bool {
        matches!(self, TerminatorDecision::ExplicitEndingSignal)
    }

    /// `true` iff this decision is the implicit `(N, 1)` event branch.
    #[inline]
    pub const fn is_implicit_n_l1_event(self) -> bool {
        matches!(self, TerminatorDecision::ImplicitNL1Event { .. })
    }

    /// Construct an implicit-terminator decision after checking that
    /// the candidate pair satisfies the patent's `(N, 1)` predicate
    /// against the block's `total_coeffs`. Returns
    /// [`InvalidTerminator::PairNotNL1`] when the predicate fails.
    ///
    /// This is the patent-faithful constructor for the implicit branch:
    /// the upstream reader is required by the patent to commit to the
    /// implicit branch *only* when its final pair satisfies the
    /// predicate; this constructor enforces that.
    pub fn new_implicit(
        terminator_pair: RunLevelPair,
        total_coeffs: u64,
    ) -> core::result::Result<Self, InvalidTerminator> {
        if !terminator_pair.is_implicit_terminator_for(total_coeffs) {
            return Err(InvalidTerminator::PairNotNL1 {
                run: terminator_pair.run,
                level: terminator_pair.level.get(),
                total_coeffs,
            });
        }
        Ok(TerminatorDecision::ImplicitNL1Event { terminator_pair })
    }

    /// Construct an explicit-ending-signal decision. No validation is
    /// needed — the explicit branch's structural shape lives entirely
    /// in the (`[GAP]`) symbol pattern, not in any pair the decision
    /// carries.
    #[inline]
    pub const fn new_explicit() -> Self {
        TerminatorDecision::ExplicitEndingSignal
    }
}

/// Construction-time rejection for [`TerminatorDecision::new_implicit`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidTerminator {
    /// The candidate pair does not satisfy the patent's
    /// `(N, 1)` implicit-terminator predicate: either `level != 1`
    /// or `run != total_coeffs`.
    PairNotNL1 {
        /// The candidate pair's run component.
        run: u32,
        /// The candidate pair's level magnitude.
        level: u32,
        /// The block's total coefficient count `N`.
        total_coeffs: u64,
    },
}

impl core::fmt::Display for InvalidTerminator {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidTerminator::PairNotNL1 {
                run,
                level,
                total_coeffs,
            } => write!(
                f,
                "oxideav-wma::terminator: pair (run={run}, level={level}) does not satisfy the patent's (N, 1) predicate for a block of N={total_coeffs} coefficients",
            ),
        }
    }
}

impl std::error::Error for InvalidTerminator {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runlevel::RunLevelPair;

    fn pair(run: u32, level: u32) -> RunLevelPair {
        RunLevelPair::new(run, level).expect("test pair must be valid")
    }

    // ---------- TerminatorMechanism: enum shape ----------

    #[test]
    fn mechanism_all_enumerates_both_patent_disclosed_alternatives() {
        assert_eq!(
            TerminatorMechanism::ALL,
            [
                TerminatorMechanism::ExplicitEndingSignal,
                TerminatorMechanism::ImplicitNL1Event,
            ]
        );
    }

    #[test]
    fn mechanism_predicates_are_exclusive() {
        for m in TerminatorMechanism::ALL {
            assert_ne!(m.is_explicit_ending_signal(), m.is_implicit_n_l1_event());
        }
    }

    #[test]
    fn mechanism_opposite_is_involutive() {
        for m in TerminatorMechanism::ALL {
            assert_eq!(m.opposite().opposite(), m);
            assert_ne!(m.opposite(), m);
        }
    }

    #[test]
    fn mechanism_opposite_flips_each_variant() {
        assert_eq!(
            TerminatorMechanism::ExplicitEndingSignal.opposite(),
            TerminatorMechanism::ImplicitNL1Event,
        );
        assert_eq!(
            TerminatorMechanism::ImplicitNL1Event.opposite(),
            TerminatorMechanism::ExplicitEndingSignal,
        );
    }

    // ---------- TerminatorMechanism: is_compatible_with ----------

    #[test]
    fn explicit_mechanism_is_compatible_with_any_pair() {
        // The patent places no structural constraint on the final
        // (R, L) for the explicit branch — the constraint lives in
        // the (still-[GAP]) symbol pattern, not the pair.
        let m = TerminatorMechanism::ExplicitEndingSignal;
        assert!(m.is_compatible_with(pair(1, 1), 256));
        assert!(m.is_compatible_with(pair(99, 99), 256));
        assert!(m.is_compatible_with(pair(7, 3), 8));
    }

    #[test]
    fn implicit_mechanism_accepts_n_l1_pair() {
        // (N, 1) with run==N, level==1 matches the patent predicate.
        let m = TerminatorMechanism::ImplicitNL1Event;
        assert!(m.is_compatible_with(pair(8, 1), 8));
        assert!(m.is_compatible_with(pair(256, 1), 256));
    }

    #[test]
    fn implicit_mechanism_rejects_pair_with_level_above_one() {
        let m = TerminatorMechanism::ImplicitNL1Event;
        assert!(!m.is_compatible_with(pair(8, 2), 8));
    }

    #[test]
    fn implicit_mechanism_rejects_pair_with_wrong_run() {
        let m = TerminatorMechanism::ImplicitNL1Event;
        // run < remaining
        assert!(!m.is_compatible_with(pair(7, 1), 8));
        // run > remaining
        assert!(!m.is_compatible_with(pair(9, 1), 8));
    }

    // ---------- TerminatorDecision: mechanism + accessors ----------

    #[test]
    fn explicit_decision_reports_explicit_mechanism() {
        let d = TerminatorDecision::new_explicit();
        assert_eq!(d.mechanism(), TerminatorMechanism::ExplicitEndingSignal);
        assert!(d.is_explicit_ending_signal());
        assert!(!d.is_implicit_n_l1_event());
        assert_eq!(d.terminator_pair(), None);
    }

    #[test]
    fn implicit_decision_reports_implicit_mechanism() {
        let d = TerminatorDecision::new_implicit(pair(8, 1), 8).unwrap();
        assert_eq!(d.mechanism(), TerminatorMechanism::ImplicitNL1Event);
        assert!(d.is_implicit_n_l1_event());
        assert!(!d.is_explicit_ending_signal());
        assert_eq!(d.terminator_pair(), Some(pair(8, 1)));
    }

    // ---------- TerminatorDecision::new_implicit: accept paths ----------

    #[test]
    fn new_implicit_accepts_n_l1_pair_at_block_length() {
        // Each transform block size from the patent-disclosed set is
        // a valid N — the terminator (N, 1) must construct cleanly
        // for any of them.
        for n in [256_u64, 512, 1024, 2048, 4096] {
            let p = RunLevelPair::new(n as u32, 1).unwrap();
            let d = TerminatorDecision::new_implicit(p, n).expect("must accept (N, 1)");
            assert_eq!(
                d,
                TerminatorDecision::ImplicitNL1Event { terminator_pair: p }
            );
        }
    }

    #[test]
    fn new_implicit_accepts_n_l1_for_small_block() {
        // A small synthetic block; (3, 1) is the patent terminator.
        let p = pair(3, 1);
        let d = TerminatorDecision::new_implicit(p, 3).unwrap();
        assert!(d.is_implicit_n_l1_event());
    }

    // ---------- TerminatorDecision::new_implicit: reject paths ----------

    #[test]
    fn new_implicit_rejects_pair_with_level_above_one() {
        let err = TerminatorDecision::new_implicit(pair(8, 2), 8).unwrap_err();
        assert_eq!(
            err,
            InvalidTerminator::PairNotNL1 {
                run: 8,
                level: 2,
                total_coeffs: 8,
            }
        );
    }

    #[test]
    fn new_implicit_rejects_pair_with_run_below_remaining() {
        let err = TerminatorDecision::new_implicit(pair(7, 1), 8).unwrap_err();
        assert_eq!(
            err,
            InvalidTerminator::PairNotNL1 {
                run: 7,
                level: 1,
                total_coeffs: 8,
            }
        );
    }

    #[test]
    fn new_implicit_rejects_pair_with_run_above_remaining() {
        let err = TerminatorDecision::new_implicit(pair(9, 1), 8).unwrap_err();
        assert_eq!(
            err,
            InvalidTerminator::PairNotNL1 {
                run: 9,
                level: 1,
                total_coeffs: 8,
            }
        );
    }

    #[test]
    fn new_implicit_rejects_pair_for_empty_block() {
        // An empty block has no pair — and the predicate
        // `run == 0 && level == 1` is excluded by RunLevelPair::new
        // (run must be >= 1). The closest "valid" implicit-decision
        // candidate is (1, 1), which is rejected because run==1 !=
        // total_coeffs==0.
        let err = TerminatorDecision::new_implicit(pair(1, 1), 0).unwrap_err();
        assert!(matches!(err, InvalidTerminator::PairNotNL1 { .. }));
    }

    // ---------- TerminatorDecision::new_explicit: shape ----------

    #[test]
    fn new_explicit_has_no_payload() {
        let d = TerminatorDecision::new_explicit();
        assert_eq!(d, TerminatorDecision::ExplicitEndingSignal);
        assert_eq!(d.terminator_pair(), None);
    }

    // ---------- Cross-module: composes with runlevel ----------

    #[test]
    fn implicit_decision_round_trips_runlevel_terminator_predicate() {
        // For a block of N coefficients, the (N, 1) pair recognised by
        // RunLevelPair::is_implicit_terminator_for is exactly the pair
        // TerminatorDecision::new_implicit accepts.
        let p = pair(8, 1);
        assert!(p.is_implicit_terminator_for(8));
        let d = TerminatorDecision::new_implicit(p, 8).unwrap();
        assert_eq!(d.terminator_pair(), Some(p));
    }

    #[test]
    fn implicit_decision_rejects_pair_runlevel_rejects() {
        // Any pair that runlevel's predicate rejects must also be
        // rejected by the implicit constructor — the two layers agree
        // on the patent's (N, 1) shape.
        let p = pair(7, 1);
        assert!(!p.is_implicit_terminator_for(8));
        assert!(TerminatorDecision::new_implicit(p, 8).is_err());
    }

    // ---------- Error display naming ----------

    #[test]
    fn invalid_terminator_display_names_the_variant() {
        let msg = format!(
            "{}",
            InvalidTerminator::PairNotNL1 {
                run: 9,
                level: 1,
                total_coeffs: 8,
            }
        );
        assert!(msg.starts_with("oxideav-wma::terminator:"), "got: {msg}");
        assert!(msg.contains("run=9"));
        assert!(msg.contains("level=1"));
        assert!(msg.contains("N=8"));
    }

    #[test]
    fn invalid_terminator_implements_std_error() {
        // Trait object construction is the cheapest standard-library
        // proof we satisfy std::error::Error.
        let err = InvalidTerminator::PairNotNL1 {
            run: 1,
            level: 1,
            total_coeffs: 0,
        };
        let _: &dyn std::error::Error = &err;
    }
}

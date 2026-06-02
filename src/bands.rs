//! WMA per-band coding-policy descriptor.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §7 lifts three
//! patent-disclosed alternatives the WMA Standard encoder can choose
//! from on a per-band basis instead of (or in addition to) spending
//! bits coding the band's MDCT coefficients literally:
//!
//! > **Noise substitution.** At low/mid bitrates the encoder can "use
//! > noise substitution to convey information in certain bands" —
//! > instead of coding coefficients, it signals that a band should be
//! > filled with a generated noise pattern of the appropriate energy.
//! > The decoder's noise generator produces the patterns for the
//! > indicated bands.
//! >   — [PATENT US7,383,180 / US7,343,291 — noise substitution;
//! >      decoder noise generator 240]
//!
//! > **Band truncation.** The encoder may "completely eliminate the
//! > coefficients in certain (high) bands," signalling a cutoff above
//! > which no coefficients are coded.
//! >   — [PATENT US7,383,180]
//!
//! ## Scope of this module
//!
//! Three patent-disclosed per-band policies are modelled as a typed
//! [`BandPolicy`] enum:
//!
//! * [`BandPolicy::Coded`] — the band's coefficients are carried
//!   literally in the entropy stage (the default, all-bands path).
//! * [`BandPolicy::NoiseSubstituted`] — the band carries an energy
//!   level only; the decoder synthesizes a noise pattern at that
//!   energy. [PATENT US7,383,180 §"noise substitution"]
//! * [`BandPolicy::Truncated`] — the band is cut off; the decoder
//!   treats every coefficient as zero. [PATENT US7,383,180 §"band
//!   truncation"]
//!
//! A companion [`BandPlan`] descriptor carries the per-band policy
//! table plus a [`BandPlan::cutoff_index`] convenience accessor that
//! returns the lowest-index `Truncated` boundary when the high-band
//! truncation is a contiguous tail (the patent's stated "cutoff"
//! shape). The plan also exposes lookups (`policy_of`, `is_coded`,
//! etc.) and a validating `new` constructor that rejects mixed
//! `Truncated`-then-non-`Truncated` arrangements with
//! [`InvalidBandPlan::TruncatedNotContiguousTail`] when callers ask
//! for the cutoff-contiguous shape via [`BandPlan::new_with_cutoff`].
//!
//! ## What is NOT in this module
//!
//! * **The per-band flag-bit encoding.** The patent says the per-band
//!   noise-substitution decision and the truncation cutoff are
//!   signalled in the bitstream, but the bit widths and ordering are
//!   `[GAP]` (no patent fixes them and the wiki snapshot does not
//!   stage codeword tables for them). This module is the carrier, not
//!   the wire-format reader.
//! * **The noise generator.** [`BandPolicy::NoiseSubstituted`] carries
//!   only the energy parameter the patent names. The actual
//!   noise-pattern synthesis (white noise scaled to the energy?
//!   coloured noise? PRNG seed?) is `[GAP]` — the patent describes the
//!   generator's *existence* (decoder module 240) but not its
//!   coefficient-level construction. A future trace pinning the
//!   construction would extend this module with a fill helper.
//! * **The decision rule.** "Noise coding default = 1" and "depends on
//!   channels and sample rate" appear in the wiki snapshot
//!   ([WIKI] orientation) but are encoder analysis, not bitstream
//!   syntax. The plan accepts whatever the upstream reader decoded.

/// Per-band coding policy for one WMA Standard quantization band.
///
/// The three variants exhaust the patent-disclosed options the
/// encoder can pick from §7 of the trace. The default — literal
/// coefficient coding via the entropy stage — is named [`BandPolicy::Coded`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BandPolicy {
    /// The band's coefficients are carried literally in the entropy
    /// stage. This is the default coding path used whenever neither
    /// noise substitution nor truncation has been selected for the
    /// band.
    Coded,
    /// The band carries a single energy parameter; the decoder's
    /// noise generator (US7,383,180 module 240) synthesizes a noise
    /// pattern at that energy and substitutes it for the band's
    /// coefficients.
    ///
    /// The `energy` field is the patent-named "appropriate energy" the
    /// generator must hit. Sign / scale conventions are encoder policy
    /// (the patent fixes only the existence of the parameter, not its
    /// units); callers feed it in whatever unit the upstream reader
    /// produced.
    NoiseSubstituted {
        /// Patent-named "appropriate energy" carried for the band.
        energy: f64,
    },
    /// The band's coefficients have been "completely eliminate[d]"
    /// (US7,383,180); the decoder substitutes zero for every
    /// coefficient in the band. Used for the high-frequency
    /// band-truncation cutoff.
    Truncated,
}

impl BandPolicy {
    /// `true` iff this band is the patent's literal-coding default.
    pub fn is_coded(&self) -> bool {
        matches!(self, BandPolicy::Coded)
    }

    /// `true` iff this band carries a synthesized noise pattern.
    pub fn is_noise_substituted(&self) -> bool {
        matches!(self, BandPolicy::NoiseSubstituted { .. })
    }

    /// `true` iff this band has been truncated to zero.
    pub fn is_truncated(&self) -> bool {
        matches!(self, BandPolicy::Truncated)
    }

    /// Return the patent-named "appropriate energy" if the band uses
    /// noise substitution, or `None` otherwise.
    pub fn noise_energy(&self) -> Option<f64> {
        match self {
            BandPolicy::NoiseSubstituted { energy } => Some(*energy),
            _ => None,
        }
    }
}

/// Per-block band plan: one [`BandPolicy`] per quantization band.
///
/// `policies[d]` is the policy chosen for band `d`. The plan exposes
/// lookups, predicate counts, and (when constructed via
/// [`BandPlan::new_with_cutoff`]) a [`BandPlan::cutoff_index`] giving
/// the index at which the patent's "high-band truncation" tail
/// begins.
#[derive(Debug, Clone, PartialEq)]
pub struct BandPlan {
    /// Per-band policy, indexed by band `d`.
    pub policies: Vec<BandPolicy>,
    /// Index of the first `Truncated` band when the plan was built via
    /// [`BandPlan::new_with_cutoff`] and the truncation is a contiguous
    /// tail. `None` for plans built via [`BandPlan::new`] (no shape
    /// promise) and for plans whose truncated bands do not form a
    /// contiguous tail.
    cutoff: Option<usize>,
}

/// Constructor failure mode for [`BandPlan::new_with_cutoff`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvalidBandPlan {
    /// The caller asked for the patent's cutoff-contiguous shape via
    /// [`BandPlan::new_with_cutoff`] but the supplied policy table has
    /// a `Truncated` band followed by a non-`Truncated` band, i.e. the
    /// truncated bands do not form a contiguous tail. The offending
    /// boundary position is reported as the index of the first
    /// `Truncated` band whose successor was not also `Truncated`.
    TruncatedNotContiguousTail {
        /// Index of the first `Truncated` band whose immediate
        /// successor was not also `Truncated`.
        at_band: usize,
    },
}

impl core::fmt::Display for InvalidBandPlan {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidBandPlan::TruncatedNotContiguousTail { at_band } => write!(
                f,
                "oxideav-wma::bands: truncated bands do not form a contiguous tail (band {at_band} is Truncated but its successor is not)",
            ),
        }
    }
}

impl std::error::Error for InvalidBandPlan {}

impl BandPlan {
    /// Build a plan from an arbitrary per-band policy table with no
    /// shape promise. [`BandPlan::cutoff_index`] returns `None` for
    /// plans built this way.
    pub fn new(policies: Vec<BandPolicy>) -> Self {
        Self {
            policies,
            cutoff: None,
        }
    }

    /// Build a plan from a policy table that promises the patent's
    /// cutoff-contiguous shape (all `Truncated` bands form a
    /// contiguous tail at the high-band end). After the call
    /// [`BandPlan::cutoff_index`] reports the start of the tail, or
    /// `None` if no band is truncated.
    ///
    /// Rejects with [`InvalidBandPlan::TruncatedNotContiguousTail`] if
    /// any `Truncated` band has a non-`Truncated` successor.
    pub fn new_with_cutoff(policies: Vec<BandPolicy>) -> Result<Self, InvalidBandPlan> {
        // Scan for the first Truncated band; once seen, every later
        // band must also be Truncated.
        let mut cutoff: Option<usize> = None;
        for (d, p) in policies.iter().enumerate() {
            match (cutoff, p.is_truncated()) {
                (None, true) => cutoff = Some(d),
                (Some(start), false) => {
                    return Err(InvalidBandPlan::TruncatedNotContiguousTail { at_band: start });
                }
                _ => {}
            }
        }
        Ok(Self { policies, cutoff })
    }

    /// Number of bands the plan covers.
    pub fn len(&self) -> usize {
        self.policies.len()
    }

    /// Whether the plan is empty.
    pub fn is_empty(&self) -> bool {
        self.policies.is_empty()
    }

    /// Look up the policy for band `d`, or `None` if out of range.
    pub fn policy_of(&self, d: usize) -> Option<BandPolicy> {
        self.policies.get(d).copied()
    }

    /// Start of the patent's contiguous truncation tail, if any.
    ///
    /// Returns `Some(start)` when the plan was built via
    /// [`BandPlan::new_with_cutoff`] and at least one band is
    /// truncated; otherwise `None`. For plans built via
    /// [`BandPlan::new`], the cutoff is always `None`.
    pub fn cutoff_index(&self) -> Option<usize> {
        self.cutoff
    }

    /// Number of bands carrying literal coefficient coding.
    pub fn coded_band_count(&self) -> usize {
        self.policies.iter().filter(|p| p.is_coded()).count()
    }

    /// Number of bands carrying a noise substitution.
    pub fn noise_band_count(&self) -> usize {
        self.policies
            .iter()
            .filter(|p| p.is_noise_substituted())
            .count()
    }

    /// Number of bands truncated to zero.
    pub fn truncated_band_count(&self) -> usize {
        self.policies.iter().filter(|p| p.is_truncated()).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- BandPolicy predicates ----------

    #[test]
    fn coded_predicate_is_only_true_for_coded() {
        let coded = BandPolicy::Coded;
        let noise = BandPolicy::NoiseSubstituted { energy: 1.0 };
        let trunc = BandPolicy::Truncated;
        assert!(coded.is_coded());
        assert!(!noise.is_coded());
        assert!(!trunc.is_coded());
    }

    #[test]
    fn noise_predicate_is_only_true_for_noise_substituted() {
        let coded = BandPolicy::Coded;
        let noise = BandPolicy::NoiseSubstituted { energy: 1.0 };
        let trunc = BandPolicy::Truncated;
        assert!(!coded.is_noise_substituted());
        assert!(noise.is_noise_substituted());
        assert!(!trunc.is_noise_substituted());
    }

    #[test]
    fn truncated_predicate_is_only_true_for_truncated() {
        let coded = BandPolicy::Coded;
        let noise = BandPolicy::NoiseSubstituted { energy: 1.0 };
        let trunc = BandPolicy::Truncated;
        assert!(!coded.is_truncated());
        assert!(!noise.is_truncated());
        assert!(trunc.is_truncated());
    }

    #[test]
    fn noise_energy_accessor_only_returns_for_noise_substituted() {
        assert_eq!(BandPolicy::Coded.noise_energy(), None);
        assert_eq!(BandPolicy::Truncated.noise_energy(), None);
        assert_eq!(
            BandPolicy::NoiseSubstituted { energy: 3.5 }.noise_energy(),
            Some(3.5)
        );
        assert_eq!(
            BandPolicy::NoiseSubstituted { energy: 0.0 }.noise_energy(),
            Some(0.0)
        );
    }

    #[test]
    fn three_variants_are_mutually_exclusive() {
        let cases = [
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 1.0 },
            BandPolicy::Truncated,
        ];
        for p in cases {
            let flags = [p.is_coded(), p.is_noise_substituted(), p.is_truncated()];
            let count = flags.iter().filter(|f| **f).count();
            assert_eq!(count, 1, "variant {p:?} flagged {count} predicates");
        }
    }

    // ---------- BandPlan::new (no shape promise) ----------

    #[test]
    fn new_accepts_any_policy_table() {
        let plan = BandPlan::new(vec![
            BandPolicy::Truncated,
            BandPolicy::Coded,
            BandPolicy::Truncated,
        ]);
        assert_eq!(plan.len(), 3);
        // No cutoff promise on this constructor.
        assert_eq!(plan.cutoff_index(), None);
    }

    #[test]
    fn new_empty_plan_is_empty() {
        let plan = BandPlan::new(vec![]);
        assert!(plan.is_empty());
        assert_eq!(plan.len(), 0);
        assert_eq!(plan.policy_of(0), None);
    }

    // ---------- BandPlan::new_with_cutoff ----------

    #[test]
    fn new_with_cutoff_accepts_no_truncation() {
        let plan = BandPlan::new_with_cutoff(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 1.0 },
            BandPolicy::Coded,
        ])
        .unwrap();
        assert_eq!(plan.cutoff_index(), None);
        assert_eq!(plan.truncated_band_count(), 0);
    }

    #[test]
    fn new_with_cutoff_accepts_contiguous_tail() {
        let plan = BandPlan::new_with_cutoff(vec![
            BandPolicy::Coded,
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 2.0 },
            BandPolicy::Truncated,
            BandPolicy::Truncated,
        ])
        .unwrap();
        assert_eq!(plan.cutoff_index(), Some(3));
        assert_eq!(plan.truncated_band_count(), 2);
    }

    #[test]
    fn new_with_cutoff_accepts_all_truncated() {
        let plan = BandPlan::new_with_cutoff(vec![
            BandPolicy::Truncated,
            BandPolicy::Truncated,
            BandPolicy::Truncated,
        ])
        .unwrap();
        assert_eq!(plan.cutoff_index(), Some(0));
        assert_eq!(plan.truncated_band_count(), 3);
    }

    #[test]
    fn new_with_cutoff_rejects_truncated_followed_by_coded() {
        let err = BandPlan::new_with_cutoff(vec![
            BandPolicy::Coded,
            BandPolicy::Truncated,
            BandPolicy::Coded,
        ])
        .unwrap_err();
        assert_eq!(
            err,
            InvalidBandPlan::TruncatedNotContiguousTail { at_band: 1 }
        );
    }

    #[test]
    fn new_with_cutoff_rejects_truncated_followed_by_noise() {
        let err = BandPlan::new_with_cutoff(vec![
            BandPolicy::Truncated,
            BandPolicy::NoiseSubstituted { energy: 1.0 },
        ])
        .unwrap_err();
        assert_eq!(
            err,
            InvalidBandPlan::TruncatedNotContiguousTail { at_band: 0 }
        );
    }

    #[test]
    fn new_with_cutoff_accepts_single_truncated_at_end() {
        let plan =
            BandPlan::new_with_cutoff(vec![BandPolicy::Coded, BandPolicy::Truncated]).unwrap();
        assert_eq!(plan.cutoff_index(), Some(1));
    }

    #[test]
    fn new_with_cutoff_empty_plan_has_no_cutoff() {
        let plan = BandPlan::new_with_cutoff(vec![]).unwrap();
        assert!(plan.is_empty());
        assert_eq!(plan.cutoff_index(), None);
    }

    // ---------- Lookups + counts ----------

    #[test]
    fn policy_of_in_range_returns_band_policy() {
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 7.0 },
            BandPolicy::Truncated,
        ]);
        assert_eq!(plan.policy_of(0), Some(BandPolicy::Coded));
        assert_eq!(
            plan.policy_of(1),
            Some(BandPolicy::NoiseSubstituted { energy: 7.0 })
        );
        assert_eq!(plan.policy_of(2), Some(BandPolicy::Truncated));
        assert_eq!(plan.policy_of(3), None);
    }

    #[test]
    fn band_counts_partition_the_plan() {
        let plan = BandPlan::new(vec![
            BandPolicy::Coded,
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 1.0 },
            BandPolicy::Truncated,
            BandPolicy::Truncated,
            BandPolicy::Truncated,
        ]);
        assert_eq!(plan.coded_band_count(), 2);
        assert_eq!(plan.noise_band_count(), 1);
        assert_eq!(plan.truncated_band_count(), 3);
        // The three counts together cover every band.
        assert_eq!(
            plan.coded_band_count() + plan.noise_band_count() + plan.truncated_band_count(),
            plan.len(),
        );
    }

    // ---------- Display impl on the error ----------

    #[test]
    fn invalid_band_plan_display_names_offending_band() {
        let err = InvalidBandPlan::TruncatedNotContiguousTail { at_band: 4 };
        let s = format!("{err}");
        assert!(
            s.contains("band 4"),
            "display did not name offending band: {s}"
        );
        assert!(
            s.contains("contiguous tail"),
            "display did not name the shape requirement: {s}",
        );
    }

    // ---------- Cross-module: cutoff and the patent's "high-band truncation" picture ----------

    #[test]
    fn cutoff_models_the_patent_high_band_truncation_shape() {
        // Patent §7: "signal a cutoff above which no coefficients are
        // coded". The contiguous-tail constructor enforces exactly this
        // shape: coded / noise-substituted bands first, then a tail of
        // Truncated bands.
        let plan = BandPlan::new_with_cutoff(vec![
            BandPolicy::Coded,
            BandPolicy::Coded,
            BandPolicy::Coded,
            BandPolicy::NoiseSubstituted { energy: 0.1 },
            BandPolicy::Truncated,
            BandPolicy::Truncated,
        ])
        .unwrap();
        // Above the cutoff: every band is Truncated.
        let cutoff = plan.cutoff_index().expect("expected a cutoff");
        for d in cutoff..plan.len() {
            assert_eq!(plan.policy_of(d), Some(BandPolicy::Truncated));
        }
        // Below the cutoff: no band is Truncated.
        for d in 0..cutoff {
            assert!(
                !plan.policy_of(d).unwrap().is_truncated(),
                "band {d} below cutoff was Truncated",
            );
        }
    }
}

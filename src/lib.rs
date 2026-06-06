//! Pure-Rust Windows Media Audio codec.
//!
//! **Clean-room rebuild in progress.** This crate is in clean-room
//! rebuild following the OxideAV docs audit dated 2026-05-06. The
//! staged material under `docs/audio/wma/` is:
//!
//! * `wiki/Windows_Media_Audio.wiki` — an 81-line multimedia.cx
//!   orientation snapshot describing the WMA v1/v2 extradata layout
//!   and the deterministic rule that maps `(version, sample_rate)` to
//!   the per-frame MDCT long-block size. Round 1 implemented those.
//! * `wma-bitstream-from-patents.md` — a patents-only structural
//!   trace assembled from the Microsoft USPTO patent corpus
//!   (Malvar-126/380, Chen-162/171, Thumpudi-180/291/743, Koishida-819).
//!   Round 2 lifted the §2 block-size set; Round 3 lifted the §5
//!   sum/difference stereo transform and the §6 run-level pairing
//!   model; Round 4 lifted the §4 quantization-matrix differential
//!   coding step into [`qmatrix`] and the §6 mode selector / partition
//!   descriptor into [`entropy_mode`]; Round 5 lifted the §4 decoder
//!   inverse-quantization step into [`invquant`] and the §7 per-band
//!   coding-policy carrier (noise substitution + high-band truncation
//!   cutoff) into [`bands`]; Round 6 lifted the §6 patent-disclosed
//!   run-level codebook construction model — the 2-D `(R, L)`
//!   probability grid with a threshold separating in-codebook from
//!   escape pairings — into [`codebook`] (US6,223,162 grid 500 /
//!   threshold 518 / Claims 4–10); Round 7 lifted the §3
//!   patent-disclosed per-block transient-handling switch as a typed
//!   carrier that covers both mechanism alternatives the patents
//!   disclose side-by-side (US6,240,380 / US6,029,126 one-bit
//!   subband-combining flag and US7,930,171 block-size switching from
//!   the `{256, 512, 1024, 2048, 4096}` set) — see [`transient`];
//!   Round 8 lifts the §4 patent-disclosed
//!   quantization-band layout — a contiguous coefficient-range
//!   partition of a transform block, one weight-table index per band
//!   (US7,930,171 / US8,805,696 quantization-band definition) — into
//!   [`qband`], with a `band_map` helper that threads the per-band
//!   weight assignment into the per-coefficient form
//!   [`invquant::dequantize_in_place`] consumes; Round 9
//!   lifts the §6 patent-disclosed end-of-block terminator selector
//!   — two patent-backed alternatives, an "explicit ending signal"
//!   and the implicit `(N, 1)` event — into [`terminator`], with a
//!   patent-faithful constructor that rejects an implicit-branch
//!   commitment whose final `(R, L)` does not satisfy the `(N, 1)`
//!   predicate;
//!   Round 10 lifts the §4 patent-disclosed
//!   **per-block overall step size** — the single-`f64` quantization
//!   factor that multiplies the per-band matrix weight `Q[d]` to
//!   give the per-coefficient quantizer factor (US7,930,171
//!   "single overall step size for the whole block"; US7,383,180
//!   "one quantization factor per tile") — into [`step_size`], with
//!   a typed [`OverallStepSize`] carrier that enforces positivity,
//!   finiteness, and non-NaN at construction, and a [`PerBlockStep`]
//!   that pairs it with a [`BlockSize`] and folds into a [`BandScale`]
//!   via `fold_with_weights`;
//!   Round 11 (this round) lifts the §6 patent-disclosed
//!   **escape-symbol literal payload** — the typed carrier for the
//!   literal trailer that follows the patent's escape symbol when an
//!   `(R, L)` pair was excluded from the probability-thresholded
//!   codebook (US6,223,162 Claim 4: "the entropy code is an escape
//!   code"; Claims 5–6: the decoder recovers `R` and `L` from the
//!   literal trailer) — into [`escape`], with a typed
//!   [`EscapeLiteral`] whose `new` rejects the Claim-1 / Claim-2
//!   violations (`run == 0` / `level == 0`) and whose `for_pair`
//!   cross-checks the [`CodebookGrid`] disposition so the carrier is
//!   only inhabited by pairings whose escape branch is the right
//!   emission.
//!
//! Tables (Huffman codebooks, exponent bands, LSP codebook,
//! critical-frequency curves) are not yet staged so the actual
//! bitstream decode path is intentionally absent.
//!
//! ## Public surface
//!
//! * [`Version`] — WMA v1 vs. v2 selector (from container codec ID
//!   `0x160` and `0x161` respectively).
//! * [`WmaHeader`] — parsed combination of container-supplied
//!   `WAVEFORMATEX` fields plus the extradata payload.
//! * [`BlockSize`] — the five patent-disclosed long-block transform
//!   sizes for WMA Standard (`{256, 512, 1024, 2048, 4096}` samples),
//!   sourced from US7,930,171 (Chen-171) Background. Drawn from
//!   `docs/audio/wma/wma-bitstream-from-patents.md` §2.
//! * [`stereo`] — sum/difference (mid/side) two-channel transform
//!   for WMA Standard, sourced from §5 of the patent trace
//!   (US7,930,171 / US7,502,743).
//! * [`runlevel`] — typed `(R, L)` pairing primitive and
//!   sequence-walker for the spectral entropy stage, sourced from §6
//!   of the patent trace (US6,223,162 / US7,885,819).
//! * [`qmatrix`] — invertible differential-coding helpers for the
//!   per-band quantization matrix carriage, sourced from §4 of the
//!   patent trace (US7,930,171 step 120 / US7,502,743).
//! * [`entropy_mode`] — typed level / run-level mode selector and
//!   sub-range [`Partition`] descriptor, sourced from §6 of the
//!   patent trace (US6,223,162 mode selector 400 / US7,383,180
//!   entropy encoder 570).
//! * [`invquant`] — decoder-side inverse-quantization helpers
//!   (`q * Q[d] * step`) plus a precomputable [`BandScale`] table that
//!   folds the per-band weight and per-block step into one
//!   multiplication, sourced from §4 of the patent trace
//!   (US7,930,171 / US7,383,180 inverse quantizer-weighter / US6,240,380
//!   re-weighting at decoder).
//! * [`bands`] — per-band coding-policy carrier covering the three
//!   patent-disclosed options ([`BandPolicy::Coded`] / `NoiseSubstituted`
//!   / `Truncated`) and a [`BandPlan`] descriptor that models the
//!   patent's high-band truncation as a contiguous cutoff tail, sourced
//!   from §7 of the patent trace (US7,383,180 noise substitution +
//!   band truncation / US7,343,291).
//! * [`codebook`] — `(R, L)` probability-grid + threshold model for the
//!   run-level codebook construction step, plus a typed [`Disposition`]
//!   reporting whether a pair is in-codebook or must use the patent's
//!   escape branch, sourced from §6 of the patent trace (US6,223,162
//!   grid 500 / threshold 518 / FIG.6 / Claims 4–10).
//! * [`transient`] — per-block transient-handling switch carrier
//!   covering both patent-disclosed mechanisms ([`TransientMechanism`]
//!   `::SubbandCombineFlag` from US6,240,380 FIG.12 / US6,029,126
//!   FIG.12 and `::BlockSizeSwitch` from US7,930,171 Background),
//!   with a per-frame [`TransientPlan`] descriptor that pairs each
//!   block with its decoded switch, sourced from §3 of the patent
//!   trace.
//! * [`qband`] — quantization-band layout: a contiguous-range
//!   partition of a transform block, one weight-table index per band,
//!   sourced from §4 of the patent trace (US7,930,171 / US8,805,696
//!   quantization-band definition). The [`qband::QuantBandLayout::band_map`]
//!   helper threads the patent's per-band weight assignment into the
//!   per-coefficient form [`invquant::dequantize_in_place`] consumes.
//! * [`terminator`] — end-of-block terminator selector for the
//!   spectral-coefficient stream: both patent-disclosed alternatives
//!   ([`terminator::TerminatorMechanism::ExplicitEndingSignal`] and
//!   [`terminator::TerminatorMechanism::ImplicitNL1Event`]) named
//!   side-by-side, with a per-block [`terminator::TerminatorDecision`]
//!   carrier whose patent-faithful constructor enforces the `(N, 1)`
//!   predicate against the block's total coefficient count. Sourced
//!   from §6 of the patent trace (US6,223,162).
//! * [`step_size`] — typed per-block overall step-size carrier for the
//!   patent's "single overall step size for the whole block"
//!   (US7,930,171) / "one quantization factor per tile"
//!   (US7,383,180 quantizer 560) arrangement. [`OverallStepSize`]
//!   enforces positivity, finiteness, and non-NaN at construction;
//!   [`PerBlockStep`] pairs a [`BlockSize`] with the step and folds
//!   into a [`BandScale`] via `fold_with_weights`. Sourced from §4 of
//!   the patent trace.
//! * [`escape`] — typed escape-symbol literal payload carrier for the
//!   §6 entropy stage. [`EscapeLiteral::new`] rejects the patent's
//!   `run == 0` / `level == 0` violations (US6,223,162 Claim 1 / Claim
//!   2); [`EscapeLiteral::for_pair`] takes a [`CodebookGrid`] +
//!   [`runlevel::RunLevelPair`] and produces the literal precisely
//!   when the grid reports [`Disposition::Escape`] (US6,223,162 Claim
//!   4). The Claim-5/6 decoder side is realised as
//!   [`EscapeLiteral::as_run_level_pair`], which rebuilds the
//!   `(R, L)` pair the literal carries.
//! * [`Error`] — crate-local error type; new variants land as the
//!   pipeline grows.
//!
//! [`Partition`]: entropy_mode::Partition
//! [`BandPolicy::Coded`]: bands::BandPolicy::Coded
//! [`BandPlan`]: bands::BandPlan
//! [`BandScale`]: invquant::BandScale
//! [`Disposition`]: codebook::Disposition
//! [`TransientMechanism`]: transient::TransientMechanism
//! [`TransientPlan`]: transient::TransientPlan
//! [`qband`]: crate::qband
//! [`OverallStepSize`]: step_size::OverallStepSize
//! [`PerBlockStep`]: step_size::PerBlockStep
//! [`EscapeLiteral`]: escape::EscapeLiteral
//! [`EscapeLiteral::new`]: escape::EscapeLiteral::new
//! [`EscapeLiteral::for_pair`]: escape::EscapeLiteral::for_pair
//! [`EscapeLiteral::as_run_level_pair`]: escape::EscapeLiteral::as_run_level_pair

#![forbid(unsafe_code)]

pub mod bands;
pub mod block;
pub mod codebook;
pub mod entropy_mode;
pub mod escape;
pub mod header;
pub mod invquant;
pub mod qband;
pub mod qmatrix;
pub mod runlevel;
pub mod step_size;
pub mod stereo;
pub mod terminator;
pub mod transient;

pub use bands::{BandPlan, BandPolicy};
pub use block::BlockSize;
pub use codebook::{CodebookGrid, Disposition};
pub use entropy_mode::{EntropyMode, Partition};
pub use escape::{EscapeError, EscapeLiteral};
pub use header::{Version, WmaHeader};
pub use invquant::BandScale;
pub use qband::{QuantBand, QuantBandLayout};
pub use step_size::{OverallStepSize, PerBlockStep};
pub use terminator::{TerminatorDecision, TerminatorMechanism};
pub use transient::{TransientMechanism, TransientPlan, TransientSwitch};

/// Crate-local error type. Concrete variants land as the rebuild
/// rounds populate the codec pipeline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Reserved placeholder for surfaces that are still scaffolds.
    NotImplemented,
    /// The container-supplied extradata payload is shorter than the
    /// version-specific minimum (4 bytes for v1, 6 bytes for v2).
    ExtradataTooShort {
        /// Minimum length required by the version's layout.
        expected: usize,
        /// Actual length the caller passed in.
        got: usize,
    },
    /// A container field that must be positive was supplied as zero.
    /// At present this is raised only for `sample_rate == 0`, which
    /// would make the frame-length decision tree ill-defined.
    InvalidContainerField {
        /// Human-readable field name (e.g. `"sample_rate"`).
        field: &'static str,
    },
    /// A transform block size was not a member of the patent-disclosed
    /// WMA Standard set `{256, 512, 1024, 2048, 4096}` samples (per
    /// US7,930,171 Background). Raised by [`BlockSize::from_samples`]
    /// and [`BlockSize::from_log2`].
    InvalidBlockSize {
        /// The rejected sample count. For [`BlockSize::from_log2`]
        /// this is reconstructed as `1 << exponent`, saturated to
        /// `u16::MAX` when the exponent would overflow `u16`.
        samples: u16,
    },
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => f.write_str(
                "oxideav-wma: clean-room rebuild in progress — see crates/oxideav-wma/README.md",
            ),
            Error::ExtradataTooShort { expected, got } => write!(
                f,
                "oxideav-wma: extradata too short (expected {expected} bytes, got {got})",
            ),
            Error::InvalidContainerField { field } => {
                write!(f, "oxideav-wma: container field `{field}` was zero",)
            }
            Error::InvalidBlockSize { samples } => write!(
                f,
                "oxideav-wma: block size {samples} samples is not a member of the patent-disclosed set {{256, 512, 1024, 2048, 4096}}",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local Result alias.
pub type Result<T> = core::result::Result<T, Error>;

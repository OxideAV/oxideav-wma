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
//!   Round 11 lifts the §6 patent-disclosed
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
//!   emission;
//!   Round 12 lifts the §3 patent-disclosed
//!   **decoder-side overlap-add (overlapper/adder) stage** — the
//!   reconstruction step that closes the inverse-MLT pipeline by
//!   summing the previous block's right half with the current block's
//!   left half (US7,383,180 decoder FIG.6 overlapper/adder; US6,029,126
//!   / US6,240,380 oddly-stacked TDAC filter bank, 2M windowing over
//!   M-length blocks) — into [`overlap_add`], with a stateful
//!   [`OverlapAdd`] carrier parameterised by a [`BlockSize`] `M`,
//!   a `2M`-sample input-length contract enforced per call, and a
//!   `flush` method that drains the trailing-edge tail;
//!   Round 13 (this round) lifts the §3 patent-disclosed
//!   **analysis/synthesis window-pair primitive** — the `2M`-sample
//!   windowing the patents define the MLT by ("a DCT modulated by the
//!   sine window function(s)", US7,383,180 frequency transformer 530;
//!   `ha(n)` / `hs(n)` window pair and the MLBT / NMLBT biorthogonal
//!   generalization, US6,240,380 Eqns.1–2 / element 510) — into
//!   [`window`], with a [`WindowShape`] enum naming all three
//!   patent-disclosed shape alternatives (only the sine shape is
//!   realizable; the MLBT / NMLBT parametric forms are `[GAP]`), a
//!   [`Window`] carrier holding the `2M` sine coefficients with a
//!   TDAC power-complementarity predicate, and a [`WindowPair`]
//!   carrier for the patent's analysis/synthesis arrangement;
//!   Round 14 (this round) lifts the §3 patent-disclosed **MLT
//!   forward/inverse transform** itself — the primitive Rounds 12 and
//!   13 both explicitly deferred — into [`mlt`]: the oddly-stacked
//!   TDAC cosine bank ("basis = windowed DCT-IV", US6,029,126 /
//!   US6,240,380 FIG.7; US7,383,180 frequency transformer 530 /
//!   decoder FIG.6; US7,930,171 WMA7 MLT over variable-size blocks)
//!   realised via the general public DSP form of that filter bank
//!   (the trace's `[DSP]` tier), with an [`Mlt`] carrier per
//!   [`BlockSize`] whose `forward` maps a `2M`-sample
//!   analysis-windowed frame to `M` coefficients and whose `inverse`
//!   maps `M` coefficients back to the `2M`-sample
//!   pre-synthesis-window frame, normalized so the full
//!   window → transform → overlap-add chain is unity-gain.
//!   Round 15 lifts the §4 patent-disclosed
//!   **energy-derived quantization matrix** — `Q[c][d] = E[d]` where
//!   the excitation pattern `E[d]` squares the band's MLT
//!   coefficients, sums the energies within the band, and divides by
//!   `Card{B[d]}` raised to the patent's experimentally-derived
//!   exponent (US7,930,171 WMA7 formula / formula (3)) — into
//!   [`excitation`], computing the per-band weight vector a
//!   [`qband::QuantBandLayout`] partitions a block into, with the
//!   `[GAP]` exponent supplied by the caller (never fabricated).
//!   Round 16 (this round) assembles the §3 patent-disclosed
//!   **decoder-side time-domain reconstruction stage** — the FIG.6
//!   chain *inverse transform → synthesis window → overlapper/adder*
//!   (US7,383,180 decoder FIG.6) — into [`synthesis`], wiring the three
//!   primitives Rounds 12–14 already landed ([`mlt`], [`window`],
//!   [`overlap_add`]) into one stateful [`Synthesis`] stage that maps
//!   `M` dequantized coefficients to `M` reconstructed time samples per
//!   block, carrying the overlap-add tail across calls; it adds no
//!   arithmetic of its own (a test pins block-for-block equality with
//!   the hand-wired chain).
//!   Round 17 (this round) lifts the §5 patent-disclosed **open-loop
//!   stereo (channel-coding) decision** — the encoder-side choice
//!   between coding the two channels independently and jointly as
//!   sum/difference, made "based on inter-channel energy separation and
//!   the disparity of excitation patterns" (US7,502,743) — into
//!   [`channel_decision`], computing both patent-named analysis
//!   quantities exactly from the input while leaving the combining-rule
//!   thresholds (the encoder's `[GAP]` tuning) and the v1/v2 mode flag
//!   (`[GAP]` bitstream layout) caller-supplied / out of scope, exactly
//!   as Round 15's [`excitation`] handled the `[GAP]` band-size
//!   exponent.
//!   Round 18 (this round) assembles the §4 patent-disclosed
//!   **decoder-side inverse-quantize + inverse-weighting stage** — the
//!   §8 FIG.6 decoder step `coeff_hat[k] = q[k] * Q[d(k)] * step`
//!   (US7,930,171 overall step-size description; US7,383,180 inverse
//!   quantizer/weighter FIG.6; US6,240,380 re-weighting at decoder) —
//!   into [`dequant`], wiring the three §4 primitives already landed
//!   ([`qband::QuantBandLayout`] band map, the per-band weights `Q[d]`,
//!   and [`step_size::OverallStepSize`]) into one [`DequantStage`] that
//!   folds `Q[d] * step` once per band into a [`BandScale`], materialises
//!   the band map once, and maps `M` entropy-decoded integer
//!   coefficients to `M` dequantized real coefficients per block. Like
//!   Round 16's [`synthesis`] it adds no arithmetic of its own; its
//!   output is exactly the input [`synthesis::Synthesis::block`]
//!   consumes, so the two assemblers chain into the FIG.6 decoder tail.
//!   Round 19 (this round) assembles the §8 patent-disclosed **full
//!   single-channel decoder-block chain** — the FIG.6 path *entropy
//!   decode → inverse quantize/weight → fill noise-substituted bands
//!   (module 240) → inverse MLT → window → overlap-add* (Thumpudi-180
//!   FIG.6) — into [`decode`], wiring the four decode stages already
//!   landed ([`spectral::SpectralDecode`], [`dequant::DequantStage`],
//!   [`noisefill::NoiseFiller`], [`synthesis::Synthesis`]) into one
//!   stateful [`ChannelDecoder`] that maps one block's already-demuxed
//!   per-stage parameters to `M` reconstructed time samples. Its key
//!   addition over the existing pairwise chains is inserting the
//!   noise-fill step in its FIG.6-fixed position — between the inverse
//!   quantizer and the inverse MLT, where both [`dequant`] and
//!   [`synthesis`] explicitly deferred it; the constructor cross-checks
//!   that all four stages agree on `M`, and the stage adds no arithmetic
//!   of its own (a test pins block-for-block equality with the
//!   hand-wired four-stage chain).
//!   Round 20 (this round) assembles the §8 patent-disclosed **full
//!   two-channel decoder-block chain** — the stereo analogue of Round
//!   19's [`decode::ChannelDecoder`]: two complete per-channel decode
//!   chains run front-to-back (entropy decode → inverse quantize/weight →
//!   noise-fill → inverse MLT → window → overlap-add) and the §8 FIG.6
//!   `[inverse sum-difference]` multi-channel post-process (US7,502,743)
//!   folds their two reconstructed time-domain channels back to L/R PCM,
//!   gated by the per-block [`ChannelMode`] — into [`stereo_decode`].
//!   Where [`stereo_synthesis::StereoSynthesis`] (Round 17's tail)
//!   begins at the inverse MLT and consumes already-dequantized
//!   coefficients, [`stereo_decode::StereoDecoder`] begins one stage
//!   earlier at the entropy box, so it is the first assembler taking one
//!   stereo block's already-demuxed per-channel entropy symbols all the
//!   way to final L/R PCM. The fold runs after each channel's overlap-add
//!   (its FIG.6-fixed position), so the per-channel overlap-add carriers
//!   are independent across the block sequence; the constructor
//!   cross-checks both channels share one `M`, and the stage adds no
//!   arithmetic of its own (tests pin equality with two hand-wired
//!   [`decode::ChannelDecoder`] chains for both modes).
//!   Round 22 (this round) assembles the §2 patent-disclosed
//!   **frame loop** — the block→frame grouping the patents and wiki
//!   both name (Chen-171 FIG.3 / Thumpudi-180 module 520: a frame is
//!   partitioned into overlapping sub-frame blocks; wiki: "blocks →
//!   frames (one or more blocks) → superframes") — into [`frame`],
//!   wiring [`decode::ChannelDecoder`] (mono) and
//!   [`stereo_decode::StereoDecoder`] (stereo) into the layer above
//!   them: [`FrameDecoder`] / [`StereoFrameDecoder`] run a frame's
//!   ordered list of already-demuxed per-block parameter sets
//!   ([`BlockParams`] / [`StereoBlockParams`]) through the underlying
//!   per-block §8 chain and concatenate the per-block PCM into the
//!   frame's PCM, carrying the overlap-add tail across frames (flushed
//!   once at stream end, not per frame). It adds no arithmetic of its
//!   own (tests pin equality with the hand-run per-block chain). The
//!   driver runs a **uniform-block-size** frame (the
//!   non-variable-block-length case `frame_length = 1 <<
//!   frame_length_bits` describes); block-size-transition frames need
//!   window-transition handling whose shape is `[GAP]` per §2/§3 (the
//!   same deferral [`decode`] records), and the DEMUX / superframe byte
//!   layout stay `[GAP]`, so the per-block parameters and block count
//!   are caller-supplied inputs, never fabricated.
//!   Round 21 (this round) lifts the wiki snapshot's "init rate
//!   dependent parameters" — the deterministic stream-setup scalars a
//!   decoder computes once at open-time from the parsed header
//!   (`high frequency = sample rate / 2`; `bits/sec = bitrate /
//!   (channels * sr)`; `byte offset bits = log2(bps * frame length /
//!   8) + 2`; `use noise coding = 1 as a default`) — into [`setup`],
//!   deriving each closed-form scalar from a [`WmaHeader`] with no
//!   fabrication; the noise-coding *activation override* the wiki
//!   names but does not specify ("based on channels and sr") stays a
//!   DOCS-GAP, shipped as the wiki default and overridable.
//!
//! Round 24 (this round) lands the first **wire-level data**: the
//! staged numeric-table extraction under `docs/audio/wma/tables/`
//! (coefficient run-level VLC code lengths for decode modes 1/2/3,
//! the critical-band and subband Hz partition seeds, and the
//! 113-step dequantization gain ladder) is transcribed into
//! [`wire_tables`], each table pinned by invariant tests (Kraft
//! equality for the complete modes, the documented escape deficit
//! for mode 2, monotonicity, the ladder's closed form). The same
//! extraction confirms **no LSP codebook exists** on this decode
//! path. Still `[GAP]`: the symbol → `(R, L)` mapping, the mode-2
//! escape enumeration, the scale/gain VLCs, sign-bit placement, and
//! the frame/superframe bit layout — so the full
//! bitstream-byte → PCM path remains intentionally absent.
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
//! * [`channel_decision`] — the §5 open-loop stereo decision that
//!   selects [`ChannelMode::Independent`] vs.
//!   [`ChannelMode::SumDifference`] from the two patent-named analysis
//!   quantities: [`channel_decision::inter_channel_energy_separation`]
//!   (the fraction of stereo energy the sum/difference transform places
//!   in the side channel) and
//!   [`channel_decision::excitation_pattern_disparity`] (the normalised
//!   `L1` distance between the channels' §4 excitation shapes). The
//!   [`OpenLoopDecision`] carrier holds the two `[GAP]` tuning
//!   thresholds (caller-supplied, never fabricated) and combines them
//!   per the patent's stated rationale (joint coding iff both criteria
//!   favour it). Sourced from §5 of the patent trace (US7,502,743).
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
//! * [`overlap_add`] — stateful decoder-side overlap-add
//!   (overlapper/adder) stage for the inverse-MLT output blocks, per
//!   the patent's reconstruction pipeline (US7,383,180 decoder FIG.6
//!   overlapper/adder; US6,029,126 / US6,240,380 oddly-stacked TDAC
//!   filter bank with 2M-length windowing over M-length blocks). The
//!   typed [`OverlapAdd`] carrier is parameterised by a [`BlockSize`]
//!   `M`, enforces the patent-fixed `2M`-sample input contract per
//!   call, sums the previous block's right-half tail with the current
//!   block's left half to emit `M` time-domain output samples, and
//!   exposes [`OverlapAdd::flush`] to drain the trailing-edge tail.
//!   Sourced from §3 of the patent trace.
//! * [`window`] — analysis/synthesis window-pair primitive for the
//!   MLT stage: [`WindowShape`] names the three patent-disclosed
//!   alternatives (sine per US7,383,180; MLBT / NMLBT per US6,240,380
//!   — parametric forms `[GAP]`, named but not realizable);
//!   [`Window`] carries the `2M` sine-window coefficients for a
//!   [`BlockSize`] `M` with `apply_in_place` / `windowed` helpers and
//!   a TDAC power-complementarity predicate; [`WindowPair`] models the
//!   patent's `ha(n)` / `hs(n)` arrangement with a block-size-match
//!   constructor and an `orthogonal_sine` convenience pair. Sourced
//!   from §3 of the patent trace.
//! * [`mlt`] — the MLT forward/inverse transform: the oddly-stacked
//!   TDAC cosine filter bank the patents define the transform stage by
//!   (US6,029,126 / US6,240,380 FIG.7 basis = windowed DCT-IV;
//!   US7,383,180 frequency transformer 530 / decoder FIG.6;
//!   US7,930,171). [`Mlt`] is parameterised by a [`BlockSize`] `M`;
//!   [`Mlt::forward`] consumes a `2M`-sample analysis-windowed frame
//!   and produces `M` spectral coefficients, [`Mlt::inverse`] produces
//!   the `2M`-sample pre-synthesis-window frame, both enforcing their
//!   length contracts via [`InvalidMltLen`]. The inverse `2/M`
//!   normalization makes the [`window`] → [`mlt`] → [`overlap_add`]
//!   chain unity-gain for a power-complementary window pair (covered
//!   by a cross-module perfect-reconstruction test). Sourced from §3
//!   of the patent trace.
//! * [`excitation`] — the §4 patent-disclosed energy-derived
//!   quantization matrix: `Q[c][d] = E[d]` where the excitation
//!   pattern `E[d]` squares the band's MLT coefficients, sums the
//!   energies within the band, and divides by `Card{B[d]}` raised to
//!   the patent's experimentally-derived exponent (US7,930,171 WMA7
//!   formula / formula (3)). [`excitation::excitation_pattern`]
//!   computes the whole-block per-band weight vector over a
//!   [`qband::QuantBandLayout`]; the exponent is a caller-supplied
//!   `[GAP]` value (never fabricated), with `0.0` (raw summed energy)
//!   and `1.0` (mean per-coefficient energy) the two closed-form
//!   endpoints. The output feeds [`invquant::BandScale`] as the
//!   per-band `Q[d]` weights. Sourced from §4 of the patent trace.
//! * [`synthesis`] — the §3 patent-disclosed decoder-side time-domain
//!   reconstruction stage: the FIG.6 chain *inverse transform →
//!   synthesis window → overlapper/adder* (US7,383,180 decoder FIG.6)
//!   assembled into one stateful [`Synthesis`] carrier over a
//!   [`BlockSize`] `M`. [`Synthesis::block`] consumes `M` dequantized
//!   coefficients, applies [`Mlt::inverse`] (`M` → `2M`), the pair's
//!   synthesis window `hs(n)` ([`Window::windowed`]), and
//!   [`OverlapAdd::step`] (`2M` → `M`) in the patent-fixed order, and
//!   carries the overlap-add tail across calls; [`Synthesis::flush`]
//!   drains the trailing edge and [`Synthesis::reset`] clears the carry
//!   at a discontinuity. The stage adds no arithmetic beyond sequencing
//!   the three existing primitives. Sourced from §3 of the patent
//!   trace.
//! * [`dequant`] — the §4 patent-disclosed decoder-side
//!   inverse-quantize + inverse-weighting stage: the FIG.6 step
//!   `coeff_hat[k] = q[k] * Q[d(k)] * step` (US7,930,171 overall
//!   step-size description; US7,383,180 inverse quantizer/weighter FIG.6;
//!   US6,240,380 re-weighting at decoder) assembled into one
//!   [`DequantStage`] over a [`BlockSize`] `M`.
//!   [`DequantStage::new`] folds the per-band weights `Q[d]` and the
//!   [`step_size::OverallStepSize`] into a [`invquant::BandScale`]
//!   (`scale[d] = Q[d] * step`) and materialises the
//!   [`qband::QuantBandLayout`] band map once; [`DequantStage::block`]
//!   maps `M` entropy-decoded integer coefficients to `M` dequantized
//!   real coefficients, the exact input
//!   [`synthesis::Synthesis::block`] consumes. The stage adds no
//!   arithmetic beyond sequencing the existing §4 primitives. Sourced
//!   from §4 of the patent trace.
//! * [`spectral`] — the §6 patent-disclosed entropy-stage
//!   **spectral-coefficient assembler**: the FIG.6 step *entropy decode
//!   (run-level → coefficients)* (US6,223,162 mode selector 400 /
//!   FIG.5–6; US7,383,180 entropy encoder 570) assembled into one
//!   stateless [`SpectralDecode`] over a decoded
//!   [`entropy_mode::Partition`]. [`SpectralDecode::block`] copies the
//!   `split` level-mode head symbols verbatim into `0..split` and expands
//!   the run-level `(R, L)` pairs over the `split..total` tail window via
//!   [`runlevel::expand_into`] (implicit `(N, 1)` terminator measured
//!   against the *tail's* remaining count), producing the
//!   `M`-coefficient `i32` vector the §4 [`dequant::DequantStage::block`]
//!   consumes — so the two assemblers chain into the FIG.6 decoder
//!   front-half *entropy decode → inverse quantize/weight*. The codeword
//!   tables and bit reader are `[GAP]`, so the stage consumes
//!   already-decoded symbols, exactly as [`runlevel::expand_into`] does;
//!   it adds no arithmetic of its own. Sourced from §6 of the patent
//!   trace.
//! * [`stereo_synthesis`] — the §8 patent-disclosed decoder-side
//!   **stereo** time-domain reconstruction tail: the FIG.6 chain
//!   *(per channel) inverse MLT → overlap-add → `[inverse
//!   sum-difference]` → PCM* (Thumpudi-180 decoder FIG.6;
//!   US7,502,743 sum/difference post-process) assembled into one
//!   stateful [`StereoSynthesis`] over a [`BlockSize`] `M`.
//!   [`StereoSynthesis::block`] runs each of the two channels through
//!   its own [`Synthesis`] stage (each with its own overlap-add carry),
//!   then applies the §5 inverse sum/difference fold
//!   ([`stereo::inverse_in_place`]) **only** when the caller-supplied
//!   per-block [`ChannelMode`] is [`ChannelMode::SumDifference`],
//!   bypassing the post-process for [`ChannelMode::Independent`]. The
//!   fold runs after the per-channel overlap-add, exactly where FIG.6
//!   draws it, producing the final L/R PCM as a [`StereoBlock`]. The
//!   channel-mode flag layout is `[GAP]` per §5, so the mode is an
//!   input, never fabricated; the stage adds no arithmetic beyond
//!   sequencing [`Synthesis`] and [`stereo`]. Sourced from §8 (and §5)
//!   of the patent trace.
//! * [`decode`] — the §8 patent-disclosed **full single-channel
//!   decoder-block chain**: the FIG.6 path *entropy decode → inverse
//!   quantize/weight → fill noise-substituted bands (module 240) →
//!   inverse MLT → window → overlap-add* (Thumpudi-180 FIG.6) assembled
//!   into one stateful [`ChannelDecoder`] over a [`BlockSize`] `M`.
//!   [`ChannelDecoder::new`] cross-checks that the
//!   [`spectral::SpectralDecode`], [`dequant::DequantStage`],
//!   [`noisefill::NoiseFiller`], and [`synthesis::Synthesis`] stages all
//!   agree on the same coefficient count `M`;
//!   [`ChannelDecoder::block`] runs them in patent order, inserting the
//!   noise-fill step in its FIG.6-fixed position between the inverse
//!   quantizer and the inverse MLT (where both [`dequant`] and
//!   [`synthesis`] deferred it), and carries the overlap-add tail across
//!   calls. [`ChannelDecoder::flush`] / [`ChannelDecoder::reset`]
//!   delegate to the synthesis stage. The codeword tables and the
//!   per-process DEMUX are `[GAP]`, so the chain consumes already-decoded
//!   per-stage parameters; it adds no arithmetic of its own. Sourced from
//!   §8 of the patent trace.
//! * [`stereo_decode`] — the §8 patent-disclosed **full two-channel
//!   decoder-block chain**: two complete per-channel
//!   [`decode::ChannelDecoder`] chains plus the FIG.6 `[inverse
//!   sum-difference]` multi-channel post-process (US7,502,743) assembled
//!   into one stateful [`stereo_decode::StereoDecoder`] over a
//!   [`BlockSize`] `M`. [`stereo_decode::StereoDecoder::new`] cross-checks
//!   both per-channel decoders share `M`;
//!   [`stereo_decode::StereoDecoder::block`] runs each channel's full
//!   decode (entropy → dequant → noise-fill → inverse MLT → window →
//!   overlap-add), then folds the two reconstructed channels back to L/R
//!   via [`stereo::inverse_in_place`] only when the per-block
//!   [`ChannelMode`] is [`ChannelMode::SumDifference`], bypassing the box
//!   for [`ChannelMode::Independent`] — the fold in its FIG.6-fixed
//!   position after each channel's overlap-add, so the per-channel carries
//!   stay independent. It is the stereo analogue of [`decode`] just as
//!   [`stereo_synthesis`] is the stereo analogue of [`synthesis`]; the
//!   channel-mode flag layout and DEMUX are `[GAP]`, so both are inputs,
//!   never fabricated, and the stage adds no arithmetic of its own.
//!   Sourced from §8 (and §5) of the patent trace.
//! * [`frame`] — the §2 frame-loop assembler above the per-block
//!   decoders: [`FrameDecoder`] (mono) drives a [`decode::ChannelDecoder`]
//!   and [`StereoFrameDecoder`] drives a [`stereo_decode::StereoDecoder`]
//!   over a frame's ordered list of already-demuxed per-block parameter
//!   sets ([`BlockParams`] / [`StereoBlockParams`]), concatenating the
//!   per-block PCM into the frame's PCM and threading the overlap-add
//!   carrier across frames. Runs a uniform-block-size frame; the
//!   block-size-transition case and the DEMUX / superframe byte layout
//!   are `[GAP]`, so block count and per-block parameters are inputs.
//!   Sourced from §2 (and §8) of the patent trace and the wiki block→
//!   frame→superframe nesting.
//! * [`setup`] — the wiki snapshot's rate-dependent stream-setup
//!   scalars, derived once at open-time from a [`WmaHeader`]:
//!   [`SetupParams::high_frequency`] (`sample_rate / 2`),
//!   [`SetupParams::bits_per_sample`] (`bit_rate / (channels *
//!   sample_rate)`), [`SetupParams::byte_offset_bits`] (`log2(bps *
//!   frame_length / 8) + 2`), and [`SetupParams::noise_coding`] (the
//!   `use noise coding = 1` default; its activation override is a
//!   DOCS-GAP, so the field ships the default and is overridable via
//!   [`SetupParams::with_noise_coding`]). Sourced from the wiki "init
//!   rate dependent parameters" section.
//! * [`bitio`] + [`huffman`] — the entropy stage's **bit-level
//!   machinery**: an MSB-first [`BitWriter`] / [`BitReader`] pair
//!   (format-neutral `[DSP]` plumbing; the shipping WMA packing order
//!   is `[GAP]`, so the convention is a documented realization detail
//!   with one swap point) and [`HuffmanCode`], the §6 patent-disclosed
//!   code-book construction step run on caller-supplied weights
//!   (US6,223,162 grid 500 / threshold 518: "pairings above a
//!   probability threshold get Huffman codewords"; US7,930,171 step
//!   130 for the §4 matrix deltas). `from_weights` builds the optimal
//!   prefix code canonically; `from_lengths` is the plug-in point for
//!   staged real tables (validating the Kraft equality); encode /
//!   decode run over the bit cursors. Codes built here are
//!   self-consistent, **not** wire-compatible — the literal v1/v2
//!   tables stay `[GAP]`.
//! * [`masking`] — the §4 encoder-side **Bark-scale masking model**
//!   (US6,240,380 FIGS.13–14, box 1318): [`bark_from_hz`] (the
//!   textbook Bark mapping, `[DSP]` tier), [`bin_frequency`] (MLT
//!   bin centres), [`SpreadingSlopes`] with the patent-pinned
//!   `PATENT` pair (25 dB/Bark toward lower frequencies, 10 dB/Bark
//!   toward higher — masking spreads farther upward) driving the
//!   max-combined triangular [`spread_masking`] curve, and the
//!   optional [`partial_whitening`] exponent β (caller-supplied
//!   tuning) compressing the weighting's dynamic range between the
//!   β = 1 identity and β = 0 flat endpoints. Encoder analysis only:
//!   it shapes the §4 matrix, never the bitstream.
//! * [`matrix_coding`] — the §4 FIG.1 **matrix side-information chain
//!   down to bits**: [`MatrixCoder`] assembles US7,930,171's
//!   direct-compression technique end-to-end — step 110 uniform
//!   quantize, step 120 differential coding (via [`qmatrix`]), step
//!   130 Huffman over a caller-supplied bounded delta alphabet (the
//!   real "scale table (121 entries)" contents are `[GAP]`) — with
//!   the exact-quantized-grid decoder reconstruction the §4
//!   side-information contract requires, and the US7,502,743
//!   zero-delta padding efficiency detail pinned by test.
//! * [`paircode`] — the §6 entropy back end assembled **end-to-end to
//!   bits**: [`RunLevelCoder`] derives the codeword alphabet from a
//!   [`CodebookGrid`] (every in-codebook pairing + one escape symbol
//!   weighted by the residual probability mass), builds the joint
//!   `(R, L)` Huffman code from the grid's own probabilities
//!   (US6,223,162 grid 500 / US7,885,819), and codes pairs to/from
//!   the bit level — in-codebook pairs as single codewords, escapes
//!   as the escape codeword plus fixed-width `R`/`L` literals whose
//!   widths are a caller-supplied [`EscapeWidths`] (`[GAP]` per §6).
//!   Tests pin mixed in-codebook/escape stream round trips and the
//!   first full §6 chain crossing the bit level: sparse tail →
//!   `compress` → bits → `decode_pair` → `expand_into` → tail.
//! * [`frame_encode`] — the §2 **frame-loop encoder drivers**, the
//!   forward mirror of [`frame`]: [`FrameEncoder`] (mono, wrapping a
//!   [`ChannelEncoder`]) and [`StereoFrameEncoder`] (stereo, wrapping
//!   a [`StereoEncoder`] with a caller-supplied per-block
//!   [`ChannelMode`] plan) partition a frame's PCM into `M`-sample
//!   blocks and collect the per-block symbol sets the [`frame`]
//!   decoders consume; the 50%-overlap buffer threads across frames
//!   (flush once at stream end). Whole-stream encode→decode round
//!   trips through [`FrameDecoder`] / [`StereoFrameDecoder`] are
//!   pinned by tests. Uniform-block-size frames only — the
//!   variable-block-length plan and superframe byte layout stay
//!   `[GAP]`.
//! * [`stereo_encode`] — the §8 patent-disclosed **full two-channel
//!   encoder-block chain**, the stereo analogue of [`encode`] and the
//!   forward mirror of [`stereo_decode`]: [`StereoEncoder`] applies
//!   the §5 forward sum/difference fold in its §8-fixed position
//!   *before* the per-channel chains — gated by the caller-supplied
//!   per-block [`ChannelMode`], whose flag layout stays `[GAP]` — then
//!   runs two complete [`ChannelEncoder`] chains (channel 0 first, so
//!   its error surfaces before channel 1's buffer advances).
//!   [`StereoEncodedBlock`] feeds
//!   [`stereo_decode::StereoDecoder::block`] argument-for-argument
//!   (plus an `into_stereo_block_params` bridge to the [`frame`]
//!   drivers); constant-mode streams round-trip within the quantizer
//!   bound for both modes (tests pin it), and the §5 energy-
//!   concentration rationale is observable (a correlated pair's side
//!   channel quantizes far sparser than its mid).
//! * [`encode`] — the §8 patent-disclosed **full single-channel
//!   encoder-block chain**, the forward mirror of [`decode`]
//!   (Thumpudi-180 FIG.5: window + forward MLT → uniform scalar
//!   quantize → run-level entropy code). [`ChannelEncoder`] wires
//!   [`Analysis`] → [`QuantStage`] → [`SpectralEncode`] with the same
//!   `M` cross-check [`ChannelDecoder::new`] applies; its
//!   [`EncodedBlock`] output is exactly the `(levels, pairs)` input
//!   [`ChannelDecoder::block`] consumes (plus an
//!   [`EncodedBlock::into_block_params`] bridge to the [`frame`]
//!   drivers). An encoder/decoder pair built from one parameter set
//!   round-trips: decode(encode(PCM)) reproduces the PCM after the
//!   chain's `M`-sample latency within the §4 quantizer's error
//!   bound, and the bound shrinks with the step (both pinned by
//!   tests). Parameter *selection* (weights, step, partition, noise
//!   decisions) stays caller-side per the trace's `[GAP]`s.
//! * [`analysis`] — the §3 encoder-side **time-domain analysis stage**
//!   run forward: the stateful mirror of [`synthesis`] wiring frame
//!   formation (previous `M` ‖ fresh `M`, the 50% TDAC overlap),
//!   the analysis window `ha(n)`, and [`Mlt::forward`] into one
//!   [`Analysis`] carrier per [`BlockSize`] (US7,930,171 FIG.3
//!   partition into overlapping blocks; US7,383,180 modules 520/530).
//!   [`Analysis::flush`] closes the stream with one all-zero block so
//!   the paired decode chain drains the last real samples; the full
//!   Analysis → Synthesis chain reproduces the input exactly after an
//!   `M`-sample leading latency (cross-module tests pin it at 1e-9).
//! * [`SpectralEncode`] (in [`spectral`]) and [`runlevel::compress`] —
//!   the §6 entropy stage run **forward**: `compress` is the
//!   encoder-side inverse of `expand_into` (each non-zero of magnitude
//!   `L` preceded by `R ≥ 1` zeros emits `(R, L)` per US6,223,162
//!   Claims 1–2, trailing zeros reported for the caller's terminator
//!   choice), and `SpectralEncode` mirrors [`SpectralDecode`] — head
//!   copied verbatim as level-mode symbols, tail compressed to pairs
//!   with the implicit `(N, 1)` terminator appended when trailing
//!   zeros remain. [`SpectralEncode::min_split_for`] exposes the
//!   structural floor the `{1..Rm}` run set imposes on the partition
//!   boundary; the shipping encoder's tuned rule stays `[GAP]`.
//! * [`quant`] — the §4 patent-disclosed **encoder-side forward
//!   quantization step**: `q[k] = round(coeff[k] / (Q[d(k)] * step))`
//!   (US7,930,171 overall step-size description; US7,383,180 quantizer
//!   560 "adaptive, uniform, scalar quantizer"), the paired forward of
//!   [`invquant`] / [`dequant`]. [`QuantStage`] mirrors
//!   [`DequantStage`] field-for-field (same `(layout, weights, step)`
//!   constructor triple, same validation, same [`BandScale`] fold), so
//!   an encoder/decoder pair built from one parameter set is
//!   guaranteed to agree; step-size *selection* stays caller-supplied
//!   (rate-control tuning per US7,343,291, not a bitstream rule).
//!   Sourced from §4 of the patent trace.
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
//! [`OverlapAdd`]: overlap_add::OverlapAdd
//! [`OverlapAdd::flush`]: overlap_add::OverlapAdd::flush
//! [`OverlapAdd::step`]: overlap_add::OverlapAdd::step
//! [`Window::windowed`]: window::Window::windowed
//! [`Synthesis`]: synthesis::Synthesis
//! [`Synthesis::block`]: synthesis::Synthesis::block
//! [`Synthesis::flush`]: synthesis::Synthesis::flush
//! [`Synthesis::reset`]: synthesis::Synthesis::reset
//! [`StereoSynthesis`]: stereo_synthesis::StereoSynthesis
//! [`StereoSynthesis::block`]: stereo_synthesis::StereoSynthesis::block
//! [`StereoBlock`]: stereo_synthesis::StereoBlock
//! [`stereo_synthesis`]: crate::stereo_synthesis
//! [`stereo_synthesis::StereoSynthesis`]: crate::stereo_synthesis::StereoSynthesis
//! [`stereo_decode`]: crate::stereo_decode
//! [`stereo_decode::StereoDecoder`]: crate::stereo_decode::StereoDecoder
//! [`stereo_decode::StereoDecoder::new`]: crate::stereo_decode::StereoDecoder::new
//! [`stereo_decode::StereoDecoder::block`]: crate::stereo_decode::StereoDecoder::block
//! [`decode::ChannelDecoder`]: crate::decode::ChannelDecoder
//! [`decode`]: crate::decode
//! [`synthesis`]: crate::synthesis
//! [`ChannelMode`]: channel_decision::ChannelMode
//! [`ChannelMode::Independent`]: channel_decision::ChannelMode::Independent
//! [`ChannelMode::SumDifference`]: channel_decision::ChannelMode::SumDifference
//! [`stereo`]: crate::stereo
//! [`stereo::inverse_in_place`]: crate::stereo::inverse_in_place
//! [`WindowShape`]: window::WindowShape
//! [`Window`]: window::Window
//! [`WindowPair`]: window::WindowPair
//! [`Mlt`]: mlt::Mlt
//! [`Mlt::forward`]: mlt::Mlt::forward
//! [`Mlt::inverse`]: mlt::Mlt::inverse
//! [`InvalidMltLen`]: mlt::InvalidMltLen
//! [`DequantStage`]: dequant::DequantStage
//! [`DequantStage::new`]: dequant::DequantStage::new
//! [`DequantStage::block`]: dequant::DequantStage::block
//! [`SpectralDecode`]: spectral::SpectralDecode
//! [`SpectralDecode::block`]: spectral::SpectralDecode::block
//! [`spectral::SpectralDecode`]: crate::spectral::SpectralDecode
//! [`dequant::DequantStage`]: crate::dequant::DequantStage
//! [`noisefill::NoiseFiller`]: crate::noisefill::NoiseFiller
//! [`synthesis::Synthesis`]: crate::synthesis::Synthesis
//! [`ChannelDecoder`]: decode::ChannelDecoder
//! [`ChannelDecoder::new`]: decode::ChannelDecoder::new
//! [`ChannelDecoder::block`]: decode::ChannelDecoder::block
//! [`ChannelDecoder::flush`]: decode::ChannelDecoder::flush
//! [`ChannelDecoder::reset`]: decode::ChannelDecoder::reset
//! [`entropy_mode::Partition`]: crate::entropy_mode::Partition
//! [`runlevel::expand_into`]: crate::runlevel::expand_into
//! [`NoiseFiller`]: noisefill::NoiseFiller
//! [`NoiseFiller::fill`]: noisefill::NoiseFiller::fill
//! [`InvalidNoiseFill`]: noisefill::InvalidNoiseFill
//! [`frame`]: crate::frame
//! [`FrameDecoder`]: frame::FrameDecoder
//! [`StereoFrameDecoder`]: frame::StereoFrameDecoder
//! [`BlockParams`]: frame::BlockParams
//! [`StereoBlockParams`]: frame::StereoBlockParams
//! [`quant`]: crate::quant
//! [`QuantStage`]: quant::QuantStage
//! [`analysis`]: crate::analysis
//! [`Analysis`]: analysis::Analysis
//! [`Analysis::flush`]: analysis::Analysis::flush
//! [`encode`]: crate::encode
//! [`bitio`]: crate::bitio
//! [`huffman`]: crate::huffman
//! [`BitWriter`]: bitio::BitWriter
//! [`BitReader`]: bitio::BitReader
//! [`HuffmanCode`]: huffman::HuffmanCode
//! [`masking`]: crate::masking
//! [`bark_from_hz`]: masking::bark_from_hz
//! [`bin_frequency`]: masking::bin_frequency
//! [`SpreadingSlopes`]: masking::SpreadingSlopes
//! [`spread_masking`]: masking::spread_masking
//! [`partial_whitening`]: masking::partial_whitening
//! [`matrix_coding`]: crate::matrix_coding
//! [`MatrixCoder`]: matrix_coding::MatrixCoder
//! [`paircode`]: crate::paircode
//! [`RunLevelCoder`]: paircode::RunLevelCoder
//! [`EscapeWidths`]: paircode::EscapeWidths
//! [`frame_encode`]: crate::frame_encode
//! [`FrameEncoder`]: frame_encode::FrameEncoder
//! [`StereoFrameEncoder`]: frame_encode::StereoFrameEncoder
//! [`stereo_encode`]: crate::stereo_encode
//! [`StereoEncoder`]: stereo_encode::StereoEncoder
//! [`StereoEncodedBlock`]: stereo_encode::StereoEncodedBlock
//! [`ChannelEncoder`]: encode::ChannelEncoder
//! [`EncodedBlock`]: encode::EncodedBlock
//! [`EncodedBlock::into_block_params`]: encode::EncodedBlock::into_block_params
//! [`SpectralEncode`]: spectral::SpectralEncode
//! [`SpectralEncode::min_split_for`]: spectral::SpectralEncode::min_split_for
//! [`runlevel::compress`]: crate::runlevel::compress
//! [`setup`]: crate::setup
//! [`SetupParams`]: setup::SetupParams
//! [`SetupParams::high_frequency`]: setup::SetupParams::high_frequency
//! [`SetupParams::bits_per_sample`]: setup::SetupParams::bits_per_sample
//! [`SetupParams::byte_offset_bits`]: setup::SetupParams::byte_offset_bits
//! [`SetupParams::noise_coding`]: setup::SetupParams::noise_coding
//! [`SetupParams::with_noise_coding`]: setup::SetupParams::with_noise_coding

#![forbid(unsafe_code)]

pub mod analysis;
pub mod bands;
pub mod bitio;
pub mod block;
pub mod channel_decision;
pub mod codebook;
pub mod coef_vlc;
pub mod decode;
pub mod dequant;
pub mod encode;
pub mod entropy_mode;
pub mod escape;
pub mod excitation;
pub mod exponent_bands;
pub mod frame;
pub mod frame_encode;
pub mod gain_ladder;
pub mod header;
pub mod huffman;
pub mod invquant;
pub mod masking;
pub mod matrix_coding;
pub mod mlt;
pub mod noisefill;
pub mod overlap_add;
pub mod paircode;
pub mod qband;
pub mod qmatrix;
pub mod quant;
pub mod runlevel;
pub mod setup;
pub mod spectral;
pub mod step_size;
pub mod stereo;
pub mod stereo_decode;
pub mod stereo_encode;
pub mod stereo_synthesis;
pub mod synthesis;
pub mod terminator;
pub mod transient;
pub mod window;
pub mod wire_chain;
pub mod wire_tables;

pub use analysis::{Analysis, InvalidSampleLen};
pub use bands::{BandPlan, BandPolicy};
pub use bitio::{BitReader, BitWriter, BitstreamEnd};
pub use block::BlockSize;
pub use channel_decision::{ChannelMode, OpenLoopDecision};
pub use codebook::{CodebookGrid, Disposition};
pub use coef_vlc::{CoefDecodeMode, CoefVlc, CoefVlcError};
pub use decode::{AssemblyError, ChannelDecoder, DecodeError, Stage};
pub use dequant::{DequantStage, InvalidDequant};
pub use encode::{ChannelEncoder, EncodeAssemblyError, EncodeError, EncodeStage, EncodedBlock};
pub use entropy_mode::{EntropyMode, Partition};
pub use escape::{EscapeError, EscapeLiteral};
pub use exponent_bands::BandDeriveError;
pub use frame::{BlockParams, FrameDecoder, StereoBlockParams, StereoFrameDecoder};
pub use frame_encode::{
    FrameEncodeError, FrameEncoder, InvalidFrameLen, StereoFrameEncodeError, StereoFrameEncoder,
};
pub use gain_ladder::GainLadderError;
pub use header::{Version, WmaHeader};
pub use huffman::{HuffmanCode, HuffmanError};
pub use invquant::BandScale;
pub use masking::SpreadingSlopes;
pub use matrix_coding::{MatrixCodeError, MatrixCoder};
pub use mlt::{InvalidMltLen, Mlt};
pub use noisefill::{InvalidNoiseFill, NoiseFiller};
pub use overlap_add::{InvalidInputLen, OverlapAdd};
pub use paircode::{EscapeWidths, PairCodeError, RunLevelCoder};
pub use qband::{QuantBand, QuantBandLayout};
pub use quant::{InvalidQuant, QuantStage};
pub use setup::SetupParams;
pub use spectral::{SpectralDecode, SpectralEncode, SpectralEncodeError, SpectralError};
pub use step_size::{OverallStepSize, PerBlockStep};
pub use stereo_decode::{StereoAssemblyError, StereoChannel, StereoDecodeError, StereoDecoder};
pub use stereo_encode::{StereoEncodeError, StereoEncodedBlock, StereoEncoder};
pub use stereo_synthesis::{StereoBlock, StereoSynthesis};
pub use synthesis::{InvalidCoeffLen, MismatchedBlockSize, Synthesis};
pub use terminator::{TerminatorDecision, TerminatorMechanism};
pub use transient::{TransientMechanism, TransientPlan, TransientSwitch};
pub use window::{Window, WindowPair, WindowShape};
pub use wire_chain::{WireBlockConfig, WireChainError};

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

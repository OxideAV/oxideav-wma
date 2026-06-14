//! WMA decoder-side time-domain reconstruction stage.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §3 fixes the order of
//! the decoder reconstruction chain. The load-bearing citations:
//!
//! > Reconstruction at the decoder is an inverse transform followed by
//! > an **overlap-add (overlapper/adder)** stage.
//! >   — [PATENT US7,383,180 — decoder FIG.6]
//!
//! and the same FIG.6 chain as drawn in §8 of the trace:
//!
//! > ```text
//! >  → inverse MLT                       (US6,029,126/380)
//! >  → overlap-add                       (US7,383,180 overlapper/adder)
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (decoder
//! >     pipeline, Thumpudi-180 FIG.6)
//!
//! §3's [`crate::mlt`] doc-comment names the full inner ordering the
//! patent draws for the synthesis side: *inverse transform → window →
//! overlapper/adder* (US7,383,180 FIG.6). The synthesis window `hs(n)`
//! is applied between the inverse MLT and the overlap-add — the MLT
//! returns the `2M`-sample *pre-synthesis-window* frame and the
//! overlap-add consumes the *post-windowed* `2M` frame, exactly the
//! contract [`crate::mlt::Mlt::inverse`] and
//! [`crate::overlap_add::OverlapAdd::step`] already declare at their
//! boundaries.
//!
//! ## Scope of this module
//!
//! This module is the **assembler** that wires the three §3 primitives
//! already landed — [`crate::mlt::Mlt`] (Round 14),
//! [`crate::window::WindowPair`] (Round 13), and
//! [`crate::overlap_add::OverlapAdd`] (Round 12) — into the single
//! decoder reconstruction stage the patent's FIG.6 draws. For one block
//! of `M` dequantized spectral coefficients it produces `M`
//! reconstructed time-domain samples by applying, in patent order:
//!
//! 1. **Inverse MLT** — `M` coefficients → `2M` pre-synthesis-window
//!    samples ([`crate::mlt::Mlt::inverse`]).
//! 2. **Synthesis window `hs(n)`** — multiply the `2M` frame by the
//!    pair's synthesis window ([`crate::window::Window::windowed`]).
//! 3. **Overlap-add** — fold the post-windowed `2M` frame against the
//!    carried-across previous-block tail, emitting `M` samples
//!    ([`crate::overlap_add::OverlapAdd::step`]).
//!
//! The stage is **stateful**: it owns the overlap-add carrier, so the
//! previous block's right half is buffered across [`Synthesis::block`]
//! calls exactly as the patent's overlapper/adder buffers it. A
//! [`Synthesis::flush`] drains the trailing-edge tail and a
//! [`Synthesis::reset`] clears the carry at a discontinuity (seek /
//! decoder flush), both delegating to the underlying carrier.
//!
//! ## What is NOT in this module
//!
//! * **Any new transform / window / overlap math.** This stage adds no
//!   arithmetic of its own beyond sequencing the three existing
//!   primitives in the patent-fixed order; the math lives in
//!   [`crate::mlt`], [`crate::window`], and [`crate::overlap_add`] and
//!   stays the single source of truth for each step.
//! * **The dequantization that produces the coefficients.** The input
//!   `M`-coefficient slice is the output of the inverse-quantize /
//!   inverse-weight stage ([`crate::invquant`]); this module starts at
//!   the inverse transform, where §3's FIG.6 chain begins.
//! * **Block-size-transition frames.** The carrier is one uniform
//!   [`crate::block::BlockSize`] `M`; adjacent blocks of different
//!   patent-disclosed sizes (§2) need transition handling whose shape is
//!   `[GAP]` at the patent level — the same deferral
//!   [`crate::mlt`] and [`crate::overlap_add`] already record.
//! * **The noise-substituted / truncated band fill.** §7's noise
//!   generator (decoder module 240) and band-truncation cutoff act on
//!   the *coefficients* upstream of this stage; whichever coefficients
//!   reach [`Synthesis::block`] are transformed uniformly. The band
//!   policy carrier is [`crate::bands::BandPolicy`]; the noise-pattern
//!   construction itself stays `[GAP]` per §7.

use crate::block::BlockSize;
use crate::mlt::Mlt;
use crate::overlap_add::OverlapAdd;
use crate::window::WindowPair;

/// Stateful decoder-side time-domain reconstruction stage for one
/// uniform [`BlockSize`] `M`, per §3 of the patent trace
/// (US7,383,180 decoder FIG.6: inverse transform → window →
/// overlapper/adder).
///
/// One [`Synthesis::block`] call consumes `M` dequantized spectral
/// coefficients and emits `M` reconstructed time-domain samples,
/// carrying the overlap-add tail across calls. Construct with a
/// [`WindowPair`] whose synthesis window `hs(n)` matches the block
/// size; the stage validates that the pair, the MLT, and the
/// overlap-add carrier all agree on `M`.
#[derive(Debug, Clone)]
pub struct Synthesis {
    mlt: Mlt,
    window_pair: WindowPair,
    overlap: OverlapAdd,
}

impl Synthesis {
    /// Construct the reconstruction stage for a given block size and
    /// analysis/synthesis [`WindowPair`].
    ///
    /// Returns [`MismatchedBlockSize`] if the window pair's block size
    /// differs from `block_size` — the inverse MLT (`2M` output), the
    /// synthesis window (`2M` length), and the overlap-add carrier
    /// (`2M` input) must all share one `M`, so a mismatched pair would
    /// fail the per-step length contract.
    pub fn new(
        block_size: BlockSize,
        window_pair: WindowPair,
    ) -> Result<Self, MismatchedBlockSize> {
        if window_pair.block_size() != block_size {
            return Err(MismatchedBlockSize {
                stage: block_size,
                window_pair: window_pair.block_size(),
            });
        }
        Ok(Self {
            mlt: Mlt::new(block_size),
            window_pair,
            overlap: OverlapAdd::new(block_size),
        })
    }

    /// Block size `M` for this stage.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.mlt.block_size()
    }

    /// `M`, the per-call input length (dequantized coefficient count)
    /// and the per-call output length (reconstructed sample count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.mlt.coeff_len()
    }

    /// The analysis/synthesis [`WindowPair`] this stage applies. The
    /// synthesis window `hs(n)` is the one folded between the inverse
    /// MLT and the overlap-add; the analysis window `ha(n)` is carried
    /// for the encoder/forward side and is not used here.
    #[inline]
    pub const fn window_pair(&self) -> &WindowPair {
        &self.window_pair
    }

    /// Read-only access to the overlap-add carrier's pending tail (the
    /// previous block's right half), length `M`. Exposed so tests and
    /// transition-aware callers can inspect the boundary state without
    /// driving a full sequence.
    #[inline]
    pub fn tail(&self) -> &[f64] {
        self.overlap.tail()
    }

    /// Reconstruct one block: consume `M` dequantized spectral
    /// coefficients, emit `M` time-domain samples.
    ///
    /// Applies, in the patent-fixed FIG.6 order
    /// (US7,383,180 decoder FIG.6):
    ///
    /// 1. inverse MLT (`M` → `2M`),
    /// 2. synthesis window `hs(n)` (`2M` → `2M`),
    /// 3. overlap-add against the carried tail (`2M` → `M`).
    ///
    /// Returns [`InvalidCoeffLen`] if `coeffs.len() != M`.
    pub fn block(&mut self, coeffs: &[f64]) -> Result<Vec<f64>, InvalidCoeffLen> {
        let m = self.block_len();
        if coeffs.len() != m {
            return Err(InvalidCoeffLen {
                expected: m,
                got: coeffs.len(),
            });
        }
        // 1. Inverse MLT: M coefficients -> 2M pre-synthesis-window frame.
        //    The length contract is guaranteed by the `coeffs.len() == M`
        //    check above, so `inverse` cannot reject here.
        let frame = self
            .mlt
            .inverse(coeffs)
            .expect("inverse MLT length guaranteed by M check");
        // 2. Synthesis window hs(n): multiply the 2M frame by hs(n).
        //    The window length equals 2M for the same block size, so the
        //    contract holds by construction.
        let windowed = self
            .window_pair
            .synthesis()
            .windowed(&frame)
            .expect("synthesis window length matches 2M by construction");
        // 3. Overlap-add: fold the post-windowed 2M frame against the
        //    carried tail, emitting M reconstructed samples.
        Ok(self
            .overlap
            .step(&windowed)
            .expect("overlap-add input length is 2M by construction"))
    }

    /// Drain the trailing-edge tail after the last block, returning the
    /// final `M` reconstructed samples and zeroing the carry.
    ///
    /// Mirrors [`crate::overlap_add::OverlapAdd::flush`]: a decoder
    /// processing a finite stream calls [`Synthesis::block`] for every
    /// received block, then [`Synthesis::flush`] once to retrieve the
    /// remaining overlap-add tail.
    pub fn flush(&mut self) -> Vec<f64> {
        self.overlap.flush()
    }

    /// Clear the overlap-add carry at a discontinuity (seek / decoder
    /// flush), so the next [`Synthesis::block`] behaves as if this stage
    /// were freshly constructed. Delegates to
    /// [`crate::overlap_add::OverlapAdd::reset`].
    pub fn reset(&mut self) {
        self.overlap.reset();
    }
}

/// The window pair offered to [`Synthesis::new`] disagrees with the
/// stage's block size.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MismatchedBlockSize {
    /// Block size requested for the reconstruction stage.
    pub stage: BlockSize,
    /// Block size carried by the offered window pair.
    pub window_pair: BlockSize,
}

impl core::fmt::Display for MismatchedBlockSize {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "synthesis stage block size {} samples does not match window pair {} samples",
            self.stage.samples(),
            self.window_pair.samples()
        )
    }
}

impl std::error::Error for MismatchedBlockSize {}

/// Length-contract rejection for [`Synthesis::block`], which expects
/// exactly `M` dequantized coefficients.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InvalidCoeffLen {
    /// Coefficient count the stage requires (`M`).
    pub expected: usize,
    /// Coefficient count actually offered.
    pub got: usize,
}

impl core::fmt::Display for InvalidCoeffLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "synthesis block expected {} coefficients, got {}",
            self.expected, self.got
        )
    }
}

impl std::error::Error for InvalidCoeffLen {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mlt::Mlt;
    use crate::window::WindowPair;

    fn stage(bs: BlockSize) -> Synthesis {
        Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap()
    }

    #[test]
    fn new_accepts_matching_block_size() {
        let s = stage(BlockSize::S256);
        assert_eq!(s.block_size(), BlockSize::S256);
        assert_eq!(s.block_len(), 256);
    }

    #[test]
    fn new_rejects_mismatched_window_pair() {
        let pair = WindowPair::orthogonal_sine(BlockSize::S512);
        let err = Synthesis::new(BlockSize::S256, pair).unwrap_err();
        assert_eq!(err.stage, BlockSize::S256);
        assert_eq!(err.window_pair, BlockSize::S512);
    }

    #[test]
    fn block_rejects_wrong_coeff_len() {
        let mut s = stage(BlockSize::S256);
        let err = s.block(&[0.0; 255]).unwrap_err();
        assert_eq!(err.expected, 256);
        assert_eq!(err.got, 255);
    }

    #[test]
    fn block_emits_m_samples() {
        let mut s = stage(BlockSize::S256);
        let out = s.block(&[1.0; 256]).unwrap();
        assert_eq!(out.len(), 256);
    }

    #[test]
    fn tail_starts_zeroed_and_length_m() {
        let s = stage(BlockSize::S256);
        assert_eq!(s.tail().len(), 256);
        assert!(s.tail().iter().all(|&x| x == 0.0));
    }

    /// The assembler must produce exactly what the three primitives
    /// produce when wired by hand in the patent-fixed FIG.6 order. This
    /// pins that the stage adds no arithmetic of its own.
    #[test]
    fn block_equals_manual_inverse_window_overlap_chain() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        // A non-trivial coefficient pattern.
        let coeffs: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.013).sin()).collect();

        // Manual chain with the same primitives.
        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);
        let mut overlap = OverlapAdd::new(bs);
        let frame = mlt.inverse(&coeffs).unwrap();
        let windowed = pair.synthesis().windowed(&frame).unwrap();
        let manual = overlap.step(&windowed).unwrap();

        // Assembler.
        let mut s = stage(bs);
        let via_stage = s.block(&coeffs).unwrap();

        assert_eq!(via_stage, manual);
    }

    /// Across a two-block sequence the assembler's carried tail must
    /// equal the hand-wired carrier's tail, proving state is buffered
    /// across calls exactly like the underlying overlap-add.
    #[test]
    fn carries_overlap_tail_across_blocks() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let a: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.01).cos()).collect();
        let b: Vec<f64> = (0..m).map(|k| ((k as f64) * 0.02).sin()).collect();

        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);
        let mut overlap = OverlapAdd::new(bs);
        let _ = overlap.step(
            &pair
                .synthesis()
                .windowed(&mlt.inverse(&a).unwrap())
                .unwrap(),
        );
        let manual_second = overlap
            .step(
                &pair
                    .synthesis()
                    .windowed(&mlt.inverse(&b).unwrap())
                    .unwrap(),
            )
            .unwrap();
        let manual_tail = overlap.tail().to_vec();

        let mut s = stage(bs);
        let _ = s.block(&a).unwrap();
        let stage_second = s.block(&b).unwrap();
        assert_eq!(stage_second, manual_second);
        assert_eq!(s.tail(), manual_tail.as_slice());
    }

    /// Perfect reconstruction: a constant signal pushed through the
    /// analysis chain (window → forward MLT) and back through this
    /// synthesis stage reconstructs in the steady-state interior, since
    /// the orthogonal sine pair is power-complementary. We verify the
    /// steady-state block (third of three identical blocks) reproduces
    /// the constant.
    #[test]
    fn steady_state_perfect_reconstruction_constant() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);

        // Build the analysis-windowed forward coefficients for a frame of
        // a constant-1 signal: analysis window over a 2M constant frame,
        // then forward MLT. With 50% overlap a constant interior signal
        // yields identical coefficients every block.
        let mut frame = vec![1.0_f64; 2 * m];
        pair.analysis().apply_in_place(&mut frame).unwrap();
        let coeffs = mlt.forward(&frame).unwrap();

        let mut s = stage(bs);
        // Prime two blocks so overlap-add reaches steady state.
        let _ = s.block(&coeffs).unwrap();
        let _ = s.block(&coeffs).unwrap();
        let steady = s.block(&coeffs).unwrap();
        for &x in &steady {
            assert!((x - 1.0).abs() < 1e-9, "steady-state sample {x} != 1.0");
        }
    }

    #[test]
    fn flush_drains_tail_and_zeroes_carry() {
        let mut s = stage(BlockSize::S256);
        let _ = s.block(&[0.5; 256]).unwrap();
        let pending = s.tail().to_vec();
        let flushed = s.flush();
        assert_eq!(flushed, pending);
        assert!(s.tail().iter().all(|&x| x == 0.0));
    }

    #[test]
    fn reset_clears_carry() {
        let mut s = stage(BlockSize::S256);
        let _ = s.block(&[0.5; 256]).unwrap();
        assert!(s.tail().iter().any(|&x| x != 0.0));
        s.reset();
        assert!(s.tail().iter().all(|&x| x == 0.0));
    }

    #[test]
    fn works_across_all_block_sizes() {
        for bs in BlockSize::ALL {
            let m = bs.samples() as usize;
            let mut s = stage(bs);
            let out = s.block(&vec![0.25_f64; m]).unwrap();
            assert_eq!(out.len(), m);
            assert_eq!(s.flush().len(), m);
        }
    }

    #[test]
    fn error_displays_are_descriptive() {
        let mismatch = MismatchedBlockSize {
            stage: BlockSize::S256,
            window_pair: BlockSize::S512,
        };
        let msg = format!("{mismatch}");
        assert!(msg.contains("256"));
        assert!(msg.contains("512"));

        let bad = InvalidCoeffLen {
            expected: 256,
            got: 255,
        };
        let msg = format!("{bad}");
        assert!(msg.contains("256"));
        assert!(msg.contains("255"));
    }
}

//! WMA encoder-side time-domain analysis (forward transform) stage.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §2/§3 fix the
//! encoder-side front end this module assembles. The load-bearing
//! citations:
//!
//! > The encoder partitions a frame of audio samples into
//! > **overlapping sub-frame blocks (windows) of time-varying size.**
//! >   — [PATENT US7,930,171 — generalized encoder, FIG.3]
//! >   — [PATENT US7,383,180 — partitioner/tile configurer module 520]
//!
//! > The frequency transform is a **Modulated Lapped Transform (MLT)**
//! > … "operates like a DCT modulated by the sine window function(s)."
//! >   — [PATENT US7,383,180 — frequency transformer 530, FIG.5]
//!
//! and the §8 encoder pipeline as the trace draws it:
//!
//! > ```text
//! >  → partition into variable-size blocks {256,512,1024,2048,4096}
//! >  → MLT (per block)
//! > ```
//! >   — `docs/audio/wma/wma-bitstream-from-patents.md` §8 (encoder
//! >     pipeline, Thumpudi-180 FIG.5)
//!
//! The MLT is the oddly-stacked TDAC filter bank with **50% overlap and
//! 2M-length windowing over M-length blocks** (US6,029,126 /
//! US6,240,380; realised in [`crate::mlt`]), so consuming `M` fresh
//! samples per block means each analysis frame is the previous block's
//! `M` samples followed by the current block's `M` samples — the
//! encoder-side buffering that pairs with the decoder-side
//! overlap-add carry ([`crate::overlap_add`]).
//!
//! ## Scope of this module
//!
//! This module is the **assembler** that wires two §3 primitives
//! already landed — [`crate::window::WindowPair`] (the analysis window
//! `ha(n)`) and [`crate::mlt::Mlt::forward`] — into the stateful
//! encoder front end mirroring [`crate::synthesis::Synthesis`]: one
//! [`Analysis::block`] call consumes `M` fresh time-domain samples and
//! emits `M` spectral coefficients by applying, in patent order:
//!
//! 1. **Frame formation** — previous `M` samples ‖ current `M` samples
//!    (the 50% overlap the TDAC bank is defined by).
//! 2. **Analysis window `ha(n)`** — multiply the `2M` frame
//!    ([`crate::window::Window::windowed`]).
//! 3. **Forward MLT** — `2M` windowed samples → `M` coefficients
//!    ([`crate::mlt::Mlt::forward`]).
//!
//! [`Analysis::flush`] pushes one final all-zero block so the last real
//! block's trailing half enters a frame — the encoder-side counterpart
//! of [`crate::synthesis::Synthesis::flush`]: an `n`-block signal
//! encodes to `n + 1` coefficient blocks, and the paired decode chain
//! reproduces the signal exactly, preceded by the chain's `M`-sample
//! leading latency (see the cross-module tests).
//!
//! ## What is NOT in this module
//!
//! * **Any new transform / window math.** The stage adds no arithmetic
//!   of its own beyond the frame buffering; the math lives in
//!   [`crate::mlt`] and [`crate::window`].
//! * **Block-size decisions.** The transient-driven choice from the
//!   §2 set is encoder tuning signalled as side information whose
//!   v1/v2 form is `[GAP]` per §3 (see [`crate::transient`]); this
//!   carrier runs one uniform [`BlockSize`], and block-size-transition
//!   windowing is the same `[GAP]` deferral [`crate::synthesis`]
//!   records.
//! * **The perceptual model.** The weighting matrix derived from the
//!   coefficients (§4) is downstream analysis
//!   ([`crate::excitation`]); this stage stops at the raw MLT output.

use crate::block::BlockSize;
use crate::mlt::Mlt;
use crate::synthesis::MismatchedBlockSize;
use crate::window::WindowPair;

/// Stateful encoder-side analysis stage for one uniform [`BlockSize`]
/// `M`, per §3 of the patent trace (US7,383,180 frequency transformer
/// 530: analysis window → forward MLT over 50%-overlapping frames).
///
/// One [`Analysis::block`] call consumes `M` fresh time-domain samples
/// and emits `M` spectral coefficients, buffering the block across
/// calls so each frame overlaps its predecessor by 50% — the mirror of
/// [`crate::synthesis::Synthesis`]'s overlap-add carry.
#[derive(Debug, Clone)]
pub struct Analysis {
    mlt: Mlt,
    window_pair: WindowPair,
    /// The previous block's `M` samples — the leading half of the next
    /// analysis frame. Zeroed at construction (leading edge) and by
    /// [`Analysis::reset`].
    prev: Vec<f64>,
}

impl Analysis {
    /// Construct the analysis stage for a given block size and
    /// analysis/synthesis [`WindowPair`].
    ///
    /// Returns [`MismatchedBlockSize`] if the window pair's block size
    /// differs from `block_size` — the same cross-check
    /// [`crate::synthesis::Synthesis::new`] applies, reusing its error
    /// type so a mirrored encoder/decoder pair fails identically on a
    /// bad parameter set.
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
        let m = block_size.samples() as usize;
        Ok(Self {
            mlt: Mlt::new(block_size),
            window_pair,
            prev: vec![0.0; m],
        })
    }

    /// Block size `M` for this stage.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.mlt.block_size()
    }

    /// `M`, the per-call input length (fresh sample count) and the
    /// per-call output length (spectral coefficient count).
    #[inline]
    pub fn block_len(&self) -> usize {
        self.mlt.coeff_len()
    }

    /// The analysis/synthesis [`WindowPair`] this stage applies. The
    /// analysis window `ha(n)` is the one folded before the forward
    /// MLT; the synthesis window is carried for the paired decoder
    /// side and is not used here.
    #[inline]
    pub const fn window_pair(&self) -> &WindowPair {
        &self.window_pair
    }

    /// Read-only access to the buffered previous block (the leading
    /// half of the next analysis frame), length `M`.
    #[inline]
    pub fn prev(&self) -> &[f64] {
        &self.prev
    }

    /// Analyse one block: consume `M` fresh time-domain samples, emit
    /// `M` spectral coefficients.
    ///
    /// Applies, in the patent-fixed order (US7,383,180 FIG.5):
    ///
    /// 1. frame formation — buffered previous `M` ‖ fresh `M` (50%
    ///    TDAC overlap),
    /// 2. analysis window `ha(n)` (`2M` → `2M`),
    /// 3. forward MLT (`2M` → `M`),
    ///
    /// then buffers the fresh samples as the next frame's leading
    /// half.
    ///
    /// Returns [`InvalidSampleLen`] if `samples.len() != M`.
    pub fn block(&mut self, samples: &[f64]) -> Result<Vec<f64>, InvalidSampleLen> {
        let m = self.block_len();
        if samples.len() != m {
            return Err(InvalidSampleLen {
                expected: m,
                got: samples.len(),
            });
        }
        // 1. Frame formation: previous M samples ‖ current M samples.
        let mut frame = Vec::with_capacity(2 * m);
        frame.extend_from_slice(&self.prev);
        frame.extend_from_slice(samples);
        // 2. Analysis window ha(n) over the 2M frame. Length holds by
        //    construction.
        self.window_pair
            .analysis()
            .apply_in_place(&mut frame)
            .expect("analysis window length matches 2M by construction");
        // 3. Forward MLT: 2M windowed samples -> M coefficients.
        let coeffs = self
            .mlt
            .forward(&frame)
            .expect("forward MLT length is 2M by construction");
        // Buffer the fresh samples as the next frame's leading half.
        self.prev.copy_from_slice(samples);
        Ok(coeffs)
    }

    /// Close the stream: push one final all-zero block so the last
    /// real block's samples enter their trailing frame, returning that
    /// final coefficient block and zeroing the buffer.
    ///
    /// The encoder-side counterpart of
    /// [`crate::synthesis::Synthesis::flush`]: feeding the flush block
    /// to the paired decode chain drains the last `M` real samples out
    /// of its overlap-add carry.
    pub fn flush(&mut self) -> Vec<f64> {
        let m = self.block_len();
        let zeros = vec![0.0; m];
        self.block(&zeros)
            .expect("flush block length is M by construction")
    }

    /// Clear the buffered previous block at a discontinuity (seek /
    /// encoder flush), so the next [`Analysis::block`] behaves as if
    /// this stage were freshly constructed.
    pub fn reset(&mut self) {
        self.prev.fill(0.0);
    }
}

/// Length-contract rejection for [`Analysis::block`], which expects
/// exactly `M` fresh time-domain samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InvalidSampleLen {
    /// Sample count the stage requires (`M`).
    pub expected: usize,
    /// Sample count actually offered.
    pub got: usize,
}

impl core::fmt::Display for InvalidSampleLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "analysis block expected {} samples, got {}",
            self.expected, self.got
        )
    }
}

impl std::error::Error for InvalidSampleLen {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthesis::Synthesis;

    fn stage(bs: BlockSize) -> Analysis {
        Analysis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap()
    }

    /// Deterministic pseudo-random signal in [-1, 1], mirroring the
    /// mlt module's perfect-reconstruction fixture style.
    fn pseudo_random(len: usize, seed: u64) -> Vec<f64> {
        let mut state = seed.wrapping_mul(0x9E3779B97F4A7C15).wrapping_add(1);
        (0..len)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((state >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
            })
            .collect()
    }

    // ---------- construction ----------

    #[test]
    fn new_accepts_matching_block_size() {
        let a = stage(BlockSize::S256);
        assert_eq!(a.block_size(), BlockSize::S256);
        assert_eq!(a.block_len(), 256);
        assert_eq!(a.prev().len(), 256);
        assert!(a.prev().iter().all(|&x| x == 0.0));
    }

    #[test]
    fn new_rejects_mismatched_window_pair() {
        let pair = WindowPair::orthogonal_sine(BlockSize::S512);
        let err = Analysis::new(BlockSize::S256, pair).unwrap_err();
        assert_eq!(err.stage, BlockSize::S256);
        assert_eq!(err.window_pair, BlockSize::S512);
    }

    #[test]
    fn block_rejects_wrong_sample_len() {
        let mut a = stage(BlockSize::S256);
        let err = a.block(&[0.0; 255]).unwrap_err();
        assert_eq!(err.expected, 256);
        assert_eq!(err.got, 255);
        // No mutation on error: the buffer stays zeroed.
        assert!(a.prev().iter().all(|&x| x == 0.0));
    }

    #[test]
    fn block_emits_m_coefficients_and_buffers_input() {
        let mut a = stage(BlockSize::S256);
        let x = vec![0.5; 256];
        let coeffs = a.block(&x).unwrap();
        assert_eq!(coeffs.len(), 256);
        assert_eq!(a.prev(), x.as_slice());
    }

    // ---------- equality with the hand-wired chain ----------

    #[test]
    fn block_equals_manual_window_forward_chain() {
        // The assembler must produce exactly what the primitives
        // produce when wired by hand over the same overlapped frames.
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let signal = pseudo_random(2 * m, 7);

        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);

        // Manual frame 0: zeros ‖ x0. Manual frame 1: x0 ‖ x1.
        let mut frame0 = vec![0.0; m];
        frame0.extend_from_slice(&signal[..m]);
        pair.analysis().apply_in_place(&mut frame0).unwrap();
        let manual0 = mlt.forward(&frame0).unwrap();

        let mut frame1 = signal[..2 * m].to_vec();
        pair.analysis().apply_in_place(&mut frame1).unwrap();
        let manual1 = mlt.forward(&frame1).unwrap();

        let mut a = stage(bs);
        assert_eq!(a.block(&signal[..m]).unwrap(), manual0);
        assert_eq!(a.block(&signal[m..]).unwrap(), manual1);
    }

    // ---------- flush / reset ----------

    #[test]
    fn flush_encodes_zero_block_and_zeroes_buffer() {
        let bs = BlockSize::S256;
        let m = bs.samples() as usize;
        let x = pseudo_random(m, 3);

        // Manual: the flush frame is x ‖ zeros.
        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);
        let mut frame = x.clone();
        frame.extend_from_slice(&vec![0.0; m]);
        pair.analysis().apply_in_place(&mut frame).unwrap();
        let manual = mlt.forward(&frame).unwrap();

        let mut a = stage(bs);
        let _ = a.block(&x).unwrap();
        let flushed = a.flush();
        assert_eq!(flushed, manual);
        assert!(a.prev().iter().all(|&s| s == 0.0));
    }

    #[test]
    fn reset_clears_buffer() {
        let mut a = stage(BlockSize::S256);
        let _ = a.block(&[0.5; 256]).unwrap();
        assert!(a.prev().iter().any(|&s| s != 0.0));
        a.reset();
        assert!(a.prev().iter().all(|&s| s == 0.0));
        // After reset the stage behaves as freshly constructed.
        let mut fresh = stage(BlockSize::S256);
        let x = pseudo_random(256, 11);
        assert_eq!(a.block(&x).unwrap(), fresh.block(&x).unwrap());
    }

    #[test]
    fn works_across_all_block_sizes() {
        for bs in BlockSize::ALL {
            let m = bs.samples() as usize;
            let mut a = stage(bs);
            let out = a.block(&vec![0.25_f64; m]).unwrap();
            assert_eq!(out.len(), m);
            assert_eq!(a.flush().len(), m);
        }
    }

    // ---------- cross-module: Analysis → Synthesis reconstruction ----------

    /// The full paired chain — Analysis (window → forward MLT) into
    /// Synthesis (inverse MLT → window → overlap-add) — reproduces the
    /// input exactly after the chain's M-sample leading latency: an
    /// n-block signal encodes to n + 1 coefficient blocks (the last
    /// from `flush`), and decoding all of them yields M leading zeros
    /// followed by the n·M input samples.
    fn assert_chain_reconstruction(bs: BlockSize, blocks: usize, seed: u64) {
        let m = bs.samples() as usize;
        let signal = pseudo_random(blocks * m, seed);

        let mut analysis = stage(bs);
        let mut synthesis = Synthesis::new(bs, WindowPair::orthogonal_sine(bs)).unwrap();

        let mut output = Vec::new();
        for t in 0..blocks {
            let coeffs = analysis.block(&signal[t * m..(t + 1) * m]).unwrap();
            output.extend(synthesis.block(&coeffs).unwrap());
        }
        let final_coeffs = analysis.flush();
        output.extend(synthesis.block(&final_coeffs).unwrap());

        assert_eq!(output.len(), (blocks + 1) * m);
        // Leading latency: the first M outputs are exactly zero (the
        // first frame's leading half is the zeroed buffer).
        for (p, &y) in output[..m].iter().enumerate() {
            assert!(y.abs() < 1e-9, "bs={bs:?} leading p={p}: got {y}");
        }
        // Every subsequent sample reproduces the input.
        for p in 0..blocks * m {
            assert!(
                (output[m + p] - signal[p]).abs() < 1e-9,
                "bs={bs:?} p={p}: got {} want {}",
                output[m + p],
                signal[p],
            );
        }
    }

    #[test]
    fn chain_reconstruction_s256() {
        assert_chain_reconstruction(BlockSize::S256, 4, 21);
    }

    #[test]
    fn chain_reconstruction_s512() {
        assert_chain_reconstruction(BlockSize::S512, 3, 22);
    }

    #[test]
    fn error_display_is_descriptive() {
        let e = InvalidSampleLen {
            expected: 256,
            got: 255,
        };
        let s = format!("{e}");
        assert!(s.contains("256"));
        assert!(s.contains("255"));
        let dyn_err: &dyn std::error::Error = &e;
        assert!(dyn_err.source().is_none());
    }
}

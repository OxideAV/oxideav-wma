//! WMA MLT (Modulated Lapped Transform) forward/inverse primitive.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §3 lifts the
//! patent-disclosed transform stage. The load-bearing citations:
//!
//! > The frequency transform is a **Modulated Lapped Transform (MLT)**
//! > — "a time-varying Modulated Lapped Transform [MLT]" that "operates
//! > like a DCT modulated by the sine window function(s)."
//! >   — [PATENT US7,383,180 — frequency transformer 530, FIG.5]
//! >   — [PATENT US7,930,171 — WMA7 applies an MLT to variable-size
//! >     transform blocks]
//!
//! > The MLT is, in DSP terms, the same transform commonly called the
//! > **MDCT** (oddly-stacked TDAC cosine-modulated filter bank with 50%
//! > overlap and 2M-length windowing over M-length blocks).
//! >   — [PATENT US6,029,126 / US6,240,380 — MLT defined as the
//! >     oddly-stacked TDAC filter bank, basis = windowed DCT-IV,
//! >     FIG.7]
//!
//! > Reconstruction at the decoder is an inverse transform followed by
//! > an **overlap-add (overlapper/adder)** stage.
//! >   — [PATENT US7,383,180 — decoder FIG.6]
//!
//! ## What this module models
//!
//! This is the transform primitive both [`crate::overlap_add`]
//! (Round 12) and [`crate::window`] (Round 13) explicitly deferred:
//! the lapped cosine transform sitting between the windowing stage and
//! the spectral domain. The patents fix its structural shape:
//!
//! 1. **An `M`-sample transform block maps to/from a `2M`-sample
//!    time-domain frame** ("2M-length windowing over M-length blocks",
//!    US6,029,126 / US6,240,380). The forward direction consumes `2M`
//!    windowed time samples and produces `M` spectral coefficients;
//!    the inverse direction consumes `M` coefficients and produces the
//!    `2M`-sample pre-synthesis-window frame [`crate::overlap_add`]
//!    eventually folds at 50 % overlap.
//! 2. **The basis is the windowed DCT-IV of the oddly-stacked TDAC
//!    filter bank** (US6,029,126 / US6,240,380 FIG.7). The sample
//!    values follow the general public DSP definition of that filter
//!    bank — the trace doc's **[DSP]** framing tier, used here exactly
//!    as Round 13 used it for the sine window (to realise the
//!    patent-named transform, not as a WMA-specific fact):
//!
//!    ```text
//!    basis(n, k) = cos( (π / M) · (n + 1/2 + M/2) · (k + 1/2) )
//!                                  for n in 0..2M, k in 0..M
//!    forward:  X[k] =           Σₙ  x[n] · basis(n, k)
//!    inverse:  y[n] = (2 / M) · Σₖ  X[k] · basis(n, k)
//!    ```
//!
//!    The `(n + 1/2 + M/2)` phase term is what makes the bank
//!    *oddly stacked*; it produces the defining time-domain aliasing
//!    structure (first output half antisymmetric, second half
//!    symmetric) that the 50 %-overlap-add cancels.
//! 3. **The window is applied upstream, by [`crate::window`].** The
//!    forward direction takes an already analysis-windowed frame and
//!    the inverse direction returns a frame still awaiting the
//!    synthesis window, mirroring how the patents draw the decoder
//!    chain (US7,383,180 FIG.6: inverse transform → window →
//!    overlapper/adder) and how Round 12's [`crate::overlap_add`]
//!    declared its input contract ("post-windowed").
//!
//! The inverse normalization `2/M` is chosen so the complete decoder
//! chain — inverse MLT → synthesis window → overlap-add — has exactly
//! unity gain when the analysis/synthesis pair satisfies the TDAC
//! power-complementarity condition [`crate::window`] verifies. The
//! cross-module perfect-reconstruction test in this module pins that
//! end-to-end property numerically.
//!
//! Two defining algebraic identities follow from the basis and are
//! covered by tests:
//!
//! * `inverse(forward(u))[n] = u[n] - u[M-1-n]` for `n < M` and
//!   `u[n] + u[3M-1-n]` for `n >= M` — the time-domain aliasing the
//!   50 %-overlap-add cancels (a lapped transform is *not* invertible
//!   block-wise; only the overlapped chain reconstructs).
//! * `forward(inverse(X)) = 2·X` — the inverse image lies in the
//!   alias-invariant subspace, where the analysis sum double-counts
//!   each coefficient because every time-domain sample is shared by
//!   two overlapping frames (the 50 % redundancy of the lapped
//!   arrangement).
//!
//! ## Fast factorization (the production path)
//!
//! [`Mlt::forward`] and [`Mlt::inverse`] run an `O(M log M)` **FFT
//! factorization** of the same basis — general public DSP algebra
//! (the trace's **[DSP]** tier), derived directly from
//! `cos θ = Re e^{-iθ}` with no WMA-specific fact involved:
//!
//! ```text
//! basis exponent  -(π/M)(n + a)(k + ½),  a = ½ + M/2, splits as
//!   -(2π n k / 2M)  −  (π n / 2M)  −  (π/M)·a·(k + ½)      (forward)
//!   -(2π n k / 2M)  −  (π/M)·a·k   −  (π/M)·(n + a)/2      (inverse)
//! ```
//!
//! so each direction is a **pre-twiddle → one `2M`-point complex FFT
//! → post-twiddle + real part** chain (the inverse zero-pads its `M`
//! coefficients to `2M` before the FFT and applies the `2/M`
//! normalization after). The direct `O(M·2M)` summation of the basis
//! is retained in-module as the **test oracle**: equivalence between
//! the fast path and the direct summation is pinned by tests at every
//! patent-disclosed block size (exhaustively at the small sizes,
//! spot-wise per coefficient/sample at the large ones), alongside the
//! algebraic identities above, which the fast path must satisfy
//! within floating-point tolerance.
//!
//! ## What is NOT in this module
//!
//! * **The window itself.** [`crate::window`] owns the `ha(n)` /
//!   `hs(n)` pair (and the MLBT / NMLBT `[GAP]`s).
//! * **Block-size-transition frames.** Adjacent blocks of different
//!   patent-disclosed sizes (§2) need transition handling whose shape
//!   is **[GAP]** at the patent level; an [`Mlt`] instance is one
//!   uniform [`BlockSize`].

use crate::block::BlockSize;

/// Minimal complex value for the internal FFT — format-neutral
/// **[DSP]** plumbing (no WMA-specific fact lives in it).
#[derive(Debug, Clone, Copy)]
struct Complex {
    re: f64,
    im: f64,
}

/// In-place iterative radix-2 decimation-in-time FFT with the
/// `e^{-i·2πnk/N}` kernel, `N` a power of two — the standard public
/// algorithm (bit-reversal permutation, then butterfly stages of
/// doubling span), format-neutral **[DSP]** plumbing for the fast MLT
/// factorization.
fn fft_in_place(buf: &mut [Complex]) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two());
    // Bit-reversal permutation.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            buf.swap(i, j);
        }
    }
    // Butterfly stages; each stage's twiddle w_k = e^{-i·2πk/len} is
    // computed once per k and shared across every block of the stage.
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let step = -core::f64::consts::PI / half as f64; // -2π/len
        for k in 0..half {
            let (w_im, w_re) = (step * k as f64).sin_cos();
            let mut i = k;
            while i < n {
                let u = buf[i];
                let t = buf[i + half];
                let v = Complex {
                    re: t.re * w_re - t.im * w_im,
                    im: t.re * w_im + t.im * w_re,
                };
                buf[i] = Complex {
                    re: u.re + v.re,
                    im: u.im + v.im,
                };
                buf[i + half] = Complex {
                    re: u.re - v.re,
                    im: u.im - v.im,
                };
                i += len;
            }
        }
        len <<= 1;
    }
}

/// Forward/inverse MLT for one patent-disclosed [`BlockSize`] `M`, per
/// §3 of the patent trace (US6,029,126 / US6,240,380 oddly-stacked
/// TDAC filter bank; US7,383,180 frequency transformer 530 / decoder
/// FIG.6; US7,930,171 WMA7 MLT over variable-size blocks).
///
/// The instance is stateless apart from its block size: the
/// inter-block state of the lapped arrangement lives downstream in
/// [`crate::overlap_add::OverlapAdd`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Mlt {
    block_size: BlockSize,
}

impl Mlt {
    /// Construct the transform for the given patent-disclosed block
    /// size `M` (§2's `{256, 512, 1024, 2048, 4096}` set).
    pub const fn new(block_size: BlockSize) -> Self {
        Self { block_size }
    }

    /// Block size `M` for this instance.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.block_size
    }

    /// `M`, the spectral-coefficient count per block (the forward
    /// output length and the inverse input length).
    #[inline]
    pub fn coeff_len(&self) -> usize {
        self.block_size.samples() as usize
    }

    /// `2M`, the time-domain frame length (the forward input length
    /// and the inverse output length). Matches
    /// [`crate::window::Window::len`] and
    /// [`crate::overlap_add::OverlapAdd::input_len`] for the same
    /// block size.
    #[inline]
    pub fn time_len(&self) -> usize {
        2 * self.coeff_len()
    }

    /// The oddly-stacked TDAC cosine basis,
    /// `cos((π/M)·(n + ½ + M/2)·(k + ½))` (US6,029,126 / US6,240,380
    /// FIG.7; general public DSP form, the trace's **[DSP]** tier).
    /// Production I/O runs the FFT factorization of this same basis;
    /// the summation over it survives as the in-module test oracle.
    #[cfg(test)]
    #[inline]
    fn basis(&self, n: usize, k: usize) -> f64 {
        let m = self.coeff_len() as f64;
        let phase = (n as f64 + 0.5 + m / 2.0) * core::f64::consts::PI / m;
        (phase * (k as f64 + 0.5)).cos()
    }

    /// Forward (analysis) MLT: consume a `2M`-sample
    /// analysis-windowed time frame, produce `M` spectral
    /// coefficients.
    ///
    /// ```text
    /// X[k] = Σₙ x[n] · cos((π/M)·(n + ½ + M/2)·(k + ½))    k in 0..M
    /// ```
    ///
    /// The analysis window `ha(n)` is applied upstream by
    /// [`crate::window::Window::apply_in_place`]; this is the bare
    /// cosine bank. Returns [`InvalidMltLen`] if
    /// `windowed.len() != 2M` — the patent fixes the windowed frame to
    /// `2M` samples (US6,029,126 / US6,240,380), so any other length
    /// is malformed at this stage.
    pub fn forward(&self, windowed: &[f64]) -> Result<Vec<f64>, InvalidMltLen> {
        let two_m = self.time_len();
        if windowed.len() != two_m {
            return Err(InvalidMltLen {
                expected: two_m,
                got: windowed.len(),
            });
        }
        Ok(self.forward_fast(windowed))
    }

    /// The `O(M log M)` FFT factorization of the forward direction
    /// (see the module docs): pre-twiddle `x[n]·e^{-iπn/(2M)}`, one
    /// `2M`-point FFT, post-twiddle `e^{-i(π/M)·a·(k+½)}` and real
    /// part, with `a = ½ + M/2`.
    fn forward_fast(&self, windowed: &[f64]) -> Vec<f64> {
        let m = self.coeff_len();
        let two_m = self.time_len();
        let pi = core::f64::consts::PI;
        let mut buf: Vec<Complex> = windowed
            .iter()
            .enumerate()
            .map(|(n, &x)| {
                let (s, c) = (-pi * n as f64 / two_m as f64).sin_cos();
                Complex {
                    re: x * c,
                    im: x * s,
                }
            })
            .collect();
        fft_in_place(&mut buf);
        let a = 0.5 + m as f64 / 2.0;
        buf[..m]
            .iter()
            .enumerate()
            .map(|(k, y)| {
                let (s, c) = (-(pi / m as f64) * a * (k as f64 + 0.5)).sin_cos();
                y.re * c - y.im * s
            })
            .collect()
    }

    /// The direct `O(M·2M)` summation of the forward basis — the
    /// in-module **test oracle** the fast path is pinned against.
    #[cfg(test)]
    fn forward_direct(&self, windowed: &[f64]) -> Vec<f64> {
        let m = self.coeff_len();
        (0..m)
            .map(|k| self.forward_direct_one(windowed, k))
            .collect()
    }

    /// One coefficient of the direct forward summation,
    /// `X[k] = Σₙ x[n]·basis(n, k)` — `O(2M)`, cheap enough to spot
    /// oracle-check individual coefficients at the largest block
    /// sizes.
    #[cfg(test)]
    fn forward_direct_one(&self, windowed: &[f64], k: usize) -> f64 {
        windowed
            .iter()
            .enumerate()
            .map(|(n, &x)| x * self.basis(n, k))
            .sum()
    }

    /// Inverse (synthesis) MLT: consume `M` spectral coefficients,
    /// produce the `2M`-sample time frame that — after the synthesis
    /// window `hs(n)` and [`crate::overlap_add::OverlapAdd::step`] —
    /// reconstructs the signal (US7,383,180 decoder FIG.6).
    ///
    /// ```text
    /// y[n] = (2/M) · Σₖ X[k] · cos((π/M)·(n + ½ + M/2)·(k + ½))
    ///                                                   n in 0..2M
    /// ```
    ///
    /// The `2/M` normalization makes the windowed 50 %-overlap-add
    /// chain unity-gain for a power-complementary window pair. Returns
    /// [`InvalidMltLen`] if `coeffs.len() != M`.
    pub fn inverse(&self, coeffs: &[f64]) -> Result<Vec<f64>, InvalidMltLen> {
        let m = self.coeff_len();
        if coeffs.len() != m {
            return Err(InvalidMltLen {
                expected: m,
                got: coeffs.len(),
            });
        }
        Ok(self.inverse_fast(coeffs))
    }

    /// The `O(M log M)` FFT factorization of the inverse direction
    /// (see the module docs): pre-twiddle `X[k]·e^{-i(π/M)·a·k}`
    /// zero-padded to `2M`, one `2M`-point FFT, post-twiddle
    /// `e^{-i(π/M)·(n+a)/2}`, real part, and the `2/M` normalization,
    /// with `a = ½ + M/2`.
    fn inverse_fast(&self, coeffs: &[f64]) -> Vec<f64> {
        let m = self.coeff_len();
        let two_m = self.time_len();
        let pi = core::f64::consts::PI;
        let a = 0.5 + m as f64 / 2.0;
        let mut buf = vec![Complex { re: 0.0, im: 0.0 }; two_m];
        for (k, (&x, slot)) in coeffs.iter().zip(buf.iter_mut()).enumerate() {
            let (s, c) = (-(pi / m as f64) * a * k as f64).sin_cos();
            slot.re = x * c;
            slot.im = x * s;
        }
        fft_in_place(&mut buf);
        let scale = 2.0 / m as f64;
        buf.iter()
            .enumerate()
            .map(|(n, u)| {
                let (s, c) = (-(pi / m as f64) * (n as f64 + a) * 0.5).sin_cos();
                scale * (u.re * c - u.im * s)
            })
            .collect()
    }

    /// The direct `O(M·2M)` summation of the inverse basis — the
    /// in-module **test oracle** the fast path is pinned against.
    #[cfg(test)]
    fn inverse_direct(&self, coeffs: &[f64]) -> Vec<f64> {
        let two_m = self.time_len();
        (0..two_m)
            .map(|n| self.inverse_direct_one(coeffs, n))
            .collect()
    }

    /// One sample of the direct inverse summation,
    /// `y[n] = (2/M)·Σₖ X[k]·basis(n, k)` — `O(M)`, cheap enough to
    /// spot oracle-check individual samples at the largest block
    /// sizes.
    #[cfg(test)]
    fn inverse_direct_one(&self, coeffs: &[f64], n: usize) -> f64 {
        let scale = 2.0 / self.coeff_len() as f64;
        scale
            * coeffs
                .iter()
                .enumerate()
                .map(|(k, &c)| c * self.basis(n, k))
                .sum::<f64>()
    }
}

/// Length-contract rejection for [`Mlt::forward`] (expects `2M`) and
/// [`Mlt::inverse`] (expects `M`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InvalidMltLen {
    /// Length the transform direction requires (`2M` forward, `M`
    /// inverse).
    pub expected: usize,
    /// Length the caller actually supplied.
    pub got: usize,
}

impl core::fmt::Display for InvalidMltLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-wma: MLT input length mismatch (expected {}, got {})",
            self.expected, self.got,
        )
    }
}

impl std::error::Error for InvalidMltLen {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::overlap_add::OverlapAdd;
    use crate::window::WindowPair;

    /// Deterministic pseudo-random samples in roughly [-1, 1] — a
    /// plain LCG so the tests need no external crates.
    fn pseudo_random(len: usize, mut seed: u64) -> Vec<f64> {
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            // Take the top 53 bits for an f64 in [0, 1), then center.
            let unit = (seed >> 11) as f64 / (1u64 << 53) as f64;
            out.push(2.0 * unit - 1.0);
        }
        out
    }

    // ---------- construction + accessors ----------

    #[test]
    fn accessors_for_every_block_size() {
        for bs in BlockSize::ALL {
            let mlt = Mlt::new(bs);
            let m = bs.samples() as usize;
            assert_eq!(mlt.block_size(), bs);
            assert_eq!(mlt.coeff_len(), m);
            assert_eq!(mlt.time_len(), 2 * m);
        }
    }

    #[test]
    fn time_len_matches_window_and_overlap_add_contracts() {
        // The patent draws one chain (US7,383,180 FIG.6); the three
        // modules must agree on the 2M frame length per block size.
        for bs in BlockSize::ALL {
            let mlt = Mlt::new(bs);
            let pair = WindowPair::orthogonal_sine(bs);
            let oa = OverlapAdd::new(bs);
            assert_eq!(mlt.time_len(), pair.synthesis().len());
            assert_eq!(mlt.time_len(), oa.input_len());
        }
    }

    // ---------- forward: input-length contract ----------

    #[test]
    fn forward_rejects_too_short_input() {
        let mlt = Mlt::new(BlockSize::S256);
        let err = mlt.forward(&vec![0.0; 511]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 512,
                got: 511,
            }
        );
    }

    #[test]
    fn forward_rejects_too_long_input() {
        let mlt = Mlt::new(BlockSize::S256);
        let err = mlt.forward(&vec![0.0; 513]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 512,
                got: 513,
            }
        );
    }

    #[test]
    fn forward_rejects_empty_input() {
        let mlt = Mlt::new(BlockSize::S512);
        let err = mlt.forward(&[]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 1024,
                got: 0,
            }
        );
    }

    #[test]
    fn forward_rejects_coeff_sized_input() {
        // Feeding an M-length buffer (the *inverse* input size) to the
        // forward direction is the natural mix-up; it must be caught.
        let mlt = Mlt::new(BlockSize::S256);
        let err = mlt.forward(&vec![0.0; 256]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 512,
                got: 256,
            }
        );
    }

    // ---------- inverse: input-length contract ----------

    #[test]
    fn inverse_rejects_too_short_input() {
        let mlt = Mlt::new(BlockSize::S256);
        let err = mlt.inverse(&vec![0.0; 255]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 256,
                got: 255,
            }
        );
    }

    #[test]
    fn inverse_rejects_frame_sized_input() {
        let mlt = Mlt::new(BlockSize::S256);
        let err = mlt.inverse(&vec![0.0; 512]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 256,
                got: 512,
            }
        );
    }

    #[test]
    fn inverse_rejects_empty_input() {
        let mlt = Mlt::new(BlockSize::S1024);
        let err = mlt.inverse(&[]).unwrap_err();
        assert_eq!(
            err,
            InvalidMltLen {
                expected: 1024,
                got: 0,
            }
        );
    }

    // ---------- output lengths ----------

    #[test]
    fn forward_produces_m_coefficients() {
        let mlt = Mlt::new(BlockSize::S256);
        let coeffs = mlt.forward(&vec![1.0; 512]).unwrap();
        assert_eq!(coeffs.len(), 256);
    }

    #[test]
    fn inverse_produces_two_m_samples() {
        let mlt = Mlt::new(BlockSize::S256);
        let frame = mlt.inverse(&vec![1.0; 256]).unwrap();
        assert_eq!(frame.len(), 512);
    }

    // ---------- zeros map to zeros ----------

    #[test]
    fn forward_of_zeros_is_zeros() {
        let mlt = Mlt::new(BlockSize::S256);
        let coeffs = mlt.forward(&vec![0.0; 512]).unwrap();
        assert!(coeffs.iter().all(|&c| c == 0.0));
    }

    #[test]
    fn inverse_of_zeros_is_zeros() {
        let mlt = Mlt::new(BlockSize::S256);
        let frame = mlt.inverse(&vec![0.0; 256]).unwrap();
        assert!(frame.iter().all(|&s| s == 0.0));
    }

    // ---------- linearity ----------

    #[test]
    fn forward_is_linear() {
        let mlt = Mlt::new(BlockSize::S256);
        let x = pseudo_random(512, 1);
        let y = pseudo_random(512, 2);
        let combined: Vec<f64> = x.iter().zip(&y).map(|(&a, &b)| 3.0 * a + b).collect();
        let fx = mlt.forward(&x).unwrap();
        let fy = mlt.forward(&y).unwrap();
        let fc = mlt.forward(&combined).unwrap();
        for k in 0..256 {
            let expected = 3.0 * fx[k] + fy[k];
            assert!((fc[k] - expected).abs() < 1e-9, "k={k}");
        }
    }

    #[test]
    fn inverse_is_linear() {
        let mlt = Mlt::new(BlockSize::S256);
        let x = pseudo_random(256, 3);
        let y = pseudo_random(256, 4);
        let combined: Vec<f64> = x.iter().zip(&y).map(|(&a, &b)| -2.0 * a + b).collect();
        let ix = mlt.inverse(&x).unwrap();
        let iy = mlt.inverse(&y).unwrap();
        let ic = mlt.inverse(&combined).unwrap();
        for n in 0..512 {
            let expected = -2.0 * ix[n] + iy[n];
            assert!((ic[n] - expected).abs() < 1e-9, "n={n}");
        }
    }

    // ---------- defining oddly-stacked TDAC structure ----------

    #[test]
    fn inverse_output_first_half_is_antisymmetric() {
        // The (n + ½ + M/2) phase makes basis(M-1-n, k) = -basis(n, k)
        // for n in 0..M, so any inverse output satisfies
        // y[n] = -y[M-1-n] there. This is the time-domain-aliasing
        // shape the 50%-overlap-add cancels — the defining property of
        // the patent's *oddly-stacked* TDAC bank.
        let mlt = Mlt::new(BlockSize::S256);
        let coeffs = pseudo_random(256, 5);
        let y = mlt.inverse(&coeffs).unwrap();
        for n in 0..256 {
            assert!((y[n] + y[255 - n]).abs() < 1e-9, "n={n}");
        }
    }

    #[test]
    fn inverse_output_second_half_is_symmetric() {
        // Likewise basis(3M-1-n, k) = basis(n, k) for n in M..2M, so
        // y[n] = y[3M-1-n] across the second half.
        let mlt = Mlt::new(BlockSize::S256);
        let coeffs = pseudo_random(256, 6);
        let y = mlt.inverse(&coeffs).unwrap();
        for n in 256..512 {
            assert!((y[n] - y[3 * 256 - 1 - n]).abs() < 1e-9, "n={n}");
        }
    }

    #[test]
    fn inverse_of_forward_exposes_the_alias_structure() {
        // A lapped transform is not invertible block-wise: the
        // round-trip returns the input *plus* its time-domain alias,
        //   y[n] = u[n] - u[M-1-n]      for n in 0..M
        //   y[n] = u[n] + u[3M-1-n]     for n in M..2M
        // which the overlap of adjacent frames cancels.
        let mlt = Mlt::new(BlockSize::S256);
        let u = pseudo_random(512, 7);
        let y = mlt.inverse(&mlt.forward(&u).unwrap()).unwrap();
        for n in 0..256 {
            let expected = u[n] - u[255 - n];
            assert!((y[n] - expected).abs() < 1e-9, "n={n}");
        }
        for n in 256..512 {
            let expected = u[n] + u[3 * 256 - 1 - n];
            assert!((y[n] - expected).abs() < 1e-9, "n={n}");
        }
    }

    #[test]
    fn forward_of_inverse_is_twice_identity() {
        // The inverse image lies in the alias-invariant subspace; the
        // analysis sum there double-counts each coefficient because
        // every time-domain sample is shared by two overlapping frames
        // (the 50% redundancy of the lapped arrangement).
        let mlt = Mlt::new(BlockSize::S256);
        let coeffs = pseudo_random(256, 8);
        let back = mlt.forward(&mlt.inverse(&coeffs).unwrap()).unwrap();
        for k in 0..256 {
            assert!((back[k] - 2.0 * coeffs[k]).abs() < 1e-9, "k={k}");
        }
    }

    // ---------- end-to-end perfect reconstruction ----------

    /// Drive the complete patent chain — analysis window → forward MLT
    /// → inverse MLT → synthesis window → overlap-add (US7,383,180
    /// FIG.5 / FIG.6) — over a pseudo-random signal and assert unity
    /// gain in every steady-state output sample.
    fn assert_perfect_reconstruction(bs: BlockSize, frames: usize, seed: u64) {
        let m = bs.samples() as usize;
        let mlt = Mlt::new(bs);
        let pair = WindowPair::orthogonal_sine(bs);
        let mut oa = OverlapAdd::new(bs);
        let signal = pseudo_random((frames + 1) * m, seed);

        let mut output = Vec::new();
        for t in 0..frames {
            let mut frame = signal[t * m..t * m + 2 * m].to_vec();
            pair.analysis().apply_in_place(&mut frame).unwrap();
            let coeffs = mlt.forward(&frame).unwrap();
            let mut synth = mlt.inverse(&coeffs).unwrap();
            pair.synthesis().apply_in_place(&mut synth).unwrap();
            output.extend(oa.step(&synth).unwrap());
        }
        assert_eq!(output.len(), frames * m);
        // The leading M samples lack a left-neighbour frame (and the
        // trailing tail its right neighbour); every sample in between
        // is steady-state and must match the input exactly.
        for p in m..frames * m {
            assert!(
                (output[p] - signal[p]).abs() < 1e-9,
                "bs={bs:?} p={p}: got {} want {}",
                output[p],
                signal[p],
            );
        }
    }

    #[test]
    fn perfect_reconstruction_s256() {
        assert_perfect_reconstruction(BlockSize::S256, 5, 9);
    }

    #[test]
    fn perfect_reconstruction_s512() {
        assert_perfect_reconstruction(BlockSize::S512, 3, 10);
    }

    // ---------- fast path vs the direct-summation oracle ----------

    /// Absolute tolerance scaled by the oracle's magnitude: the FFT
    /// factorization and the direct summation are the same algebra,
    /// so they must agree to floating-point rounding.
    fn assert_close(fast: &[f64], oracle: &[f64], what: &str) {
        assert_eq!(fast.len(), oracle.len(), "{what}: length");
        let scale = oracle.iter().fold(1.0_f64, |m, &v| m.max(v.abs()));
        for (i, (&f, &o)) in fast.iter().zip(oracle).enumerate() {
            assert!(
                (f - o).abs() < 1e-9 * scale,
                "{what}[{i}]: fast {f} vs oracle {o} (scale {scale})",
            );
        }
    }

    #[test]
    fn fast_forward_matches_the_direct_oracle_small_sizes() {
        // Exhaustive coefficient-for-coefficient equivalence at the
        // three smaller block sizes (the direct oracle is O(M·2M)).
        for (bs, seed) in [
            (BlockSize::S256, 11),
            (BlockSize::S512, 12),
            (BlockSize::S1024, 13),
        ] {
            let mlt = Mlt::new(bs);
            let x = pseudo_random(mlt.time_len(), seed);
            let fast = mlt.forward(&x).unwrap();
            let oracle = mlt.forward_direct(&x);
            assert_close(&fast, &oracle, "forward");
        }
    }

    #[test]
    fn fast_inverse_matches_the_direct_oracle_small_sizes() {
        for (bs, seed) in [
            (BlockSize::S256, 14),
            (BlockSize::S512, 15),
            (BlockSize::S1024, 16),
        ] {
            let mlt = Mlt::new(bs);
            let coeffs = pseudo_random(mlt.coeff_len(), seed);
            let fast = mlt.inverse(&coeffs).unwrap();
            let oracle = mlt.inverse_direct(&coeffs);
            assert_close(&fast, &oracle, "inverse");
        }
    }

    #[test]
    fn fast_forward_matches_spot_oracle_coefficients_large_sizes() {
        // At S2048/S4096 the full O(M·2M) oracle is disproportionate;
        // individual oracle coefficients are O(2M) each, so spot-pin
        // the band edges, the mid-band, and both ends.
        for (bs, seed) in [(BlockSize::S2048, 17), (BlockSize::S4096, 18)] {
            let mlt = Mlt::new(bs);
            let m = mlt.coeff_len();
            let x = pseudo_random(mlt.time_len(), seed);
            let fast = mlt.forward(&x).unwrap();
            let scale = fast.iter().fold(1.0_f64, |mx, &v| mx.max(v.abs()));
            for k in [0, 1, m / 2 - 1, m / 2, m - 2, m - 1] {
                let oracle = mlt.forward_direct_one(&x, k);
                assert!(
                    (fast[k] - oracle).abs() < 1e-9 * scale,
                    "bs={bs:?} k={k}: fast {} vs oracle {oracle}",
                    fast[k],
                );
            }
        }
    }

    #[test]
    fn fast_inverse_matches_spot_oracle_samples_large_sizes() {
        for (bs, seed) in [(BlockSize::S2048, 19), (BlockSize::S4096, 20)] {
            let mlt = Mlt::new(bs);
            let m = mlt.coeff_len();
            let coeffs = pseudo_random(m, seed);
            let fast = mlt.inverse(&coeffs).unwrap();
            let scale = fast.iter().fold(1.0_f64, |mx, &v| mx.max(v.abs()));
            for n in [0, 1, m - 1, m, 2 * m - 2, 2 * m - 1] {
                let oracle = mlt.inverse_direct_one(&coeffs, n);
                assert!(
                    (fast[n] - oracle).abs() < 1e-9 * scale,
                    "bs={bs:?} n={n}: fast {} vs oracle {oracle}",
                    fast[n],
                );
            }
        }
    }

    #[test]
    fn alias_identities_hold_at_the_largest_size() {
        // The defining TDAC algebra, checked through the fast path
        // alone at S4096 (both directions are O(M log M) there):
        // inverse∘forward exposes the alias structure and
        // forward∘inverse doubles the coefficients.
        let mlt = Mlt::new(BlockSize::S4096);
        let m = 4096usize;
        let u = pseudo_random(2 * m, 21);
        let y = mlt.inverse(&mlt.forward(&u).unwrap()).unwrap();
        for n in 0..m {
            let expected = u[n] - u[m - 1 - n];
            assert!((y[n] - expected).abs() < 1e-8, "n={n}");
        }
        for n in m..2 * m {
            let expected = u[n] + u[3 * m - 1 - n];
            assert!((y[n] - expected).abs() < 1e-8, "n={n}");
        }

        let coeffs = pseudo_random(m, 22);
        let back = mlt.forward(&mlt.inverse(&coeffs).unwrap()).unwrap();
        for k in 0..m {
            assert!((back[k] - 2.0 * coeffs[k]).abs() < 1e-8, "k={k}");
        }
    }

    #[test]
    fn perfect_reconstruction_s2048() {
        // The full patent chain at a large block size — tractable now
        // that the transform is O(M log M).
        assert_perfect_reconstruction(BlockSize::S2048, 2, 23);
    }

    // ---------- error display + trait ----------

    #[test]
    fn invalid_mlt_len_display_mentions_expected_and_got() {
        let err = InvalidMltLen {
            expected: 8192,
            got: 4096,
        };
        let s = format!("{err}");
        assert!(s.contains("8192"));
        assert!(s.contains("4096"));
        assert!(s.contains("MLT"));
    }

    #[test]
    fn invalid_mlt_len_implements_std_error() {
        let err = InvalidMltLen {
            expected: 512,
            got: 0,
        };
        let dyn_err: &dyn std::error::Error = &err;
        assert!(dyn_err.source().is_none());
    }

    // ---------- copy/eq semantics ----------

    #[test]
    fn mlt_is_copy_and_eq_on_block_size() {
        let a = Mlt::new(BlockSize::S1024);
        let b = a; // Copy
        assert_eq!(a, b);
        assert_ne!(a, Mlt::new(BlockSize::S2048));
    }
}

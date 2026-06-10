//! WMA analysis/synthesis window-pair primitive for the MLT stage.
//!
//! ## Source
//!
//! `docs/audio/wma/wma-bitstream-from-patents.md` §3 lifts the
//! patent-disclosed window structure of the transform stage. The
//! load-bearing citations:
//!
//! > The frequency transform is a **Modulated Lapped Transform (MLT)**
//! > — "a time-varying Modulated Lapped Transform [MLT]" that "operates
//! > like a DCT modulated by the **sine window function(s)**."
//! >   — [PATENT US7,383,180 — frequency transformer 530, FIG.5]
//!
//! > The MLT is, in DSP terms, the same transform commonly called the
//! > **MDCT** (oddly-stacked TDAC cosine-modulated filter bank with 50%
//! > overlap and 2M-length windowing over M-length blocks).
//! >   — [PATENT US6,029,126 / US6,240,380 — MLT defined as the
//! >     oddly-stacked TDAC filter bank, basis = windowed DCT-IV,
//! >     FIG.7]
//!
//! > Malvar's patents define the analysis/synthesis windows `ha(n)`,
//! > `hs(n)` and the biorthogonal generalization (**MLBT / NMLBT**)
//! > where analysis and synthesis windows may differ to raise stopband
//! > attenuation.
//! >   — [PATENT US6,240,380 — Eqns.1–2, window params; NMLBT element
//! >     510, FIG.5]
//!
//! ## What this module models
//!
//! Three patent-fixed structural facts:
//!
//! 1. **A window is `2M` samples long for an `M`-sample transform
//!    block** (the patent's "2M-length windowing over M-length
//!    blocks", US6,029,126 / US6,240,380). The typed [`Window`] carrier
//!    is parameterised by a [`BlockSize`] `M` and holds exactly `2M`
//!    coefficients.
//! 2. **The window comes as an analysis/synthesis *pair* `ha(n)`,
//!    `hs(n)`** that may coincide (the orthogonal MLT case) or differ
//!    (the biorthogonal MLBT / NMLBT generalization, US6,240,380).
//!    [`WindowPair`] is the typed two-window carrier; its constructor
//!    rejects a pair whose two halves disagree on the block size.
//! 3. **Three patent-named shape alternatives** exist side-by-side:
//!    the **sine** window the MLT modulates by (US7,383,180), and the
//!    **MLBT** / **NMLBT** biorthogonal windows (US6,240,380 element
//!    510). [`WindowShape`] names all three.
//!
//! ## The sine window — the one realizable shape
//!
//! For the plain (orthogonal) MLT, the modulating window named by
//! US7,383,180 is the sine window. Its sample values follow from the
//! general public DSP definition of the MLT/MDCT sine window over a
//! `2M`-length frame — the trace doc's **[DSP]** framing tier, used
//! here exactly as the trace uses it (to realise the patent-named
//! shape, not as a WMA-specific fact):
//!
//! ```text
//! h(n) = sin( (n + 1/2) * π / (2M) )      for n in 0..2M
//! ```
//!
//! This window satisfies the defining 50 %-overlap
//! perfect-reconstruction (power-complementarity) condition of the
//! oddly-stacked TDAC filter bank the patents define the MLT as:
//!
//! ```text
//! h(n)² + h(n + M)² = 1                   for n in 0..M
//! ```
//!
//! since `sin(x)² + sin(x + π/2)² = sin(x)² + cos(x)² = 1`. The
//! [`Window::is_power_complementary`] predicate verifies this property
//! numerically; combined with [`crate::overlap_add::OverlapAdd`] it
//! yields unity gain across the steady-state overlap region (covered
//! by a cross-module test).
//!
//! ## What is NOT in this module
//!
//! * **MLBT / NMLBT window coefficients.** The trace doc cites
//!   US6,240,380 Eqns.1–2 as *defining* the biorthogonal window
//!   parameters but does not reproduce the equations, so the
//!   parametric forms are **[GAP]** here. [`WindowShape::Mlbt`] and
//!   [`WindowShape::Nmlbt`] name the alternatives
//!   ([`WindowShape::is_realizable`] reports them unrealizable); no
//!   coefficient values are fabricated.
//! * **Which shape shipping WMA v1/v2 uses.** The patents name the
//!   sine window for the MLT and the MLBT/NMLBT generalization
//!   side-by-side; the trace does not pin the v1/v2 choice — that
//!   remains **[GAP]**.
//! * **The MLT / inverse MLT itself.** §3 names the transform (basis =
//!   windowed DCT-IV) but this module covers only the windowing
//!   primitive; the transform is a future round's primitive.
//! * **Block-size-transition windows.** Adjacent blocks of different
//!   patent-disclosed sizes (§2) need asymmetric transition windows
//!   whose shape is **[GAP]** at the patent level. A [`Window`] is one
//!   uniform [`BlockSize`] per instance.

use crate::block::BlockSize;

/// Patent-named window-shape alternatives for the MLT stage, per §3 of
/// the patent trace.
///
/// * [`WindowShape::Sine`] — the window the MLT "operates like a DCT
///   modulated by" (US7,383,180 frequency transformer 530).
/// * [`WindowShape::Mlbt`] / [`WindowShape::Nmlbt`] — the biorthogonal
///   generalization where analysis and synthesis windows may differ to
///   raise stopband attenuation (US6,240,380 Eqns.1–2; NMLBT element
///   510, FIG.5). Their parametric forms are **[GAP]** (the trace does
///   not reproduce the equations), so they are named but not
///   realizable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WindowShape {
    /// The sine window of the orthogonal MLT (US7,383,180).
    Sine,
    /// The Modulated Lapped Biorthogonal Transform window
    /// (US6,240,380). Parameters **[GAP]** — named, not realizable.
    Mlbt,
    /// The Nonuniform Modulated Lapped Biorthogonal Transform window
    /// (US6,240,380 element 510, FIG.5). Parameters **[GAP]** — named,
    /// not realizable.
    Nmlbt,
}

impl WindowShape {
    /// Every patent-named shape, in the order the trace doc introduces
    /// them (sine first as the plain-MLT window, then the biorthogonal
    /// generalizations).
    pub const ALL: [WindowShape; 3] = [WindowShape::Sine, WindowShape::Mlbt, WindowShape::Nmlbt];

    /// `true` iff this crate can construct a [`Window`] with this
    /// shape. Only the sine window is realizable: the MLBT / NMLBT
    /// parametric forms live in US6,240,380 Eqns.1–2, which the trace
    /// doc cites but does not reproduce — **[GAP]**.
    #[inline]
    pub const fn is_realizable(self) -> bool {
        matches!(self, WindowShape::Sine)
    }

    /// `true` for the biorthogonal generalizations (MLBT / NMLBT)
    /// where the analysis and synthesis windows may differ
    /// (US6,240,380); `false` for the orthogonal sine case where
    /// `ha(n) = hs(n)`.
    #[inline]
    pub const fn is_biorthogonal(self) -> bool {
        matches!(self, WindowShape::Mlbt | WindowShape::Nmlbt)
    }
}

/// A `2M`-sample window for an `M`-sample transform block, per the
/// patent's "2M-length windowing over M-length blocks" framing
/// (US6,029,126 / US6,240,380).
#[derive(Debug, Clone, PartialEq)]
pub struct Window {
    shape: WindowShape,
    block_size: BlockSize,
    /// Exactly `2M` coefficients.
    coeffs: Vec<f64>,
}

impl Window {
    /// Construct the sine window for the given block size — the shape
    /// US7,383,180 names for the MLT ("a DCT modulated by the sine
    /// window function(s)"). Sample values follow the general public
    /// DSP definition of the MLT/MDCT sine window
    /// (`h(n) = sin((n + 1/2)·π / (2M))`), the trace doc's **[DSP]**
    /// framing tier.
    pub fn sine(block_size: BlockSize) -> Self {
        let two_m = 2 * block_size.samples() as usize;
        let coeffs = (0..two_m)
            .map(|n| ((n as f64 + 0.5) * core::f64::consts::PI / two_m as f64).sin())
            .collect();
        Self {
            shape: WindowShape::Sine,
            block_size,
            coeffs,
        }
    }

    /// The patent-named shape this window realises.
    #[inline]
    pub const fn shape(&self) -> WindowShape {
        self.shape
    }

    /// Block size `M` this window belongs to. The window itself is
    /// `2M` samples long.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.block_size
    }

    /// `2M`, the window length (= the post-windowed inverse-MLT block
    /// length [`crate::overlap_add::OverlapAdd::step`] consumes).
    #[inline]
    pub fn len(&self) -> usize {
        self.coeffs.len()
    }

    /// Always `false` — every patent-disclosed block size is non-zero,
    /// so a window is never empty. Provided alongside [`Window::len`]
    /// for the conventional pairing.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.coeffs.is_empty()
    }

    /// The `2M` window coefficients, index `n` holding `h(n)`.
    #[inline]
    pub fn coeffs(&self) -> &[f64] {
        &self.coeffs
    }

    /// Single-coefficient lookup; `None` when `n >= 2M`.
    #[inline]
    pub fn coeff(&self, n: usize) -> Option<f64> {
        self.coeffs.get(n).copied()
    }

    /// Verify the defining 50 %-overlap perfect-reconstruction
    /// condition of the oddly-stacked TDAC filter bank the patents
    /// define the MLT as (US6,029,126 / US6,240,380):
    /// `h(n)² + h(n + M)² = 1` for every `n in 0..M`, to within
    /// `tolerance` per position.
    pub fn is_power_complementary(&self, tolerance: f64) -> bool {
        let m = self.block_size.samples() as usize;
        (0..m).all(|n| {
            let sum = self.coeffs[n] * self.coeffs[n] + self.coeffs[n + m] * self.coeffs[n + m];
            (sum - 1.0).abs() <= tolerance
        })
    }

    /// Multiply a `2M`-sample block by the window, in place.
    ///
    /// Returns [`InvalidWindowLen`] if `block.len() != 2M` — the
    /// patent fixes the windowed frame to `2M` samples (US6,029,126 /
    /// US6,240,380), so any other length is malformed at this stage.
    /// On error the block is left untouched.
    pub fn apply_in_place(&self, block: &mut [f64]) -> Result<(), InvalidWindowLen> {
        if block.len() != self.coeffs.len() {
            return Err(InvalidWindowLen {
                expected: self.coeffs.len(),
                got: block.len(),
            });
        }
        for (sample, &h) in block.iter_mut().zip(&self.coeffs) {
            *sample *= h;
        }
        Ok(())
    }

    /// Multiply a `2M`-sample block by the window into a fresh `Vec`.
    /// Same length contract as [`Window::apply_in_place`].
    pub fn windowed(&self, block: &[f64]) -> Result<Vec<f64>, InvalidWindowLen> {
        if block.len() != self.coeffs.len() {
            return Err(InvalidWindowLen {
                expected: self.coeffs.len(),
                got: block.len(),
            });
        }
        Ok(block
            .iter()
            .zip(&self.coeffs)
            .map(|(&sample, &h)| sample * h)
            .collect())
    }
}

/// The patent's analysis/synthesis window pair `ha(n)`, `hs(n)`
/// (US6,240,380): coincident for the orthogonal MLT, possibly distinct
/// for the biorthogonal MLBT / NMLBT generalization.
#[derive(Debug, Clone, PartialEq)]
pub struct WindowPair {
    analysis: Window,
    synthesis: Window,
}

impl WindowPair {
    /// Pair an analysis window `ha(n)` with a synthesis window
    /// `hs(n)`. Both must belong to the same [`BlockSize`] — the
    /// patent's pair covers the *same* `2M`-length frame on both
    /// sides of the transform; a size mismatch is rejected as
    /// [`InvalidWindowPair::BlockSizeMismatch`].
    pub fn new(analysis: Window, synthesis: Window) -> Result<Self, InvalidWindowPair> {
        if analysis.block_size() != synthesis.block_size() {
            return Err(InvalidWindowPair::BlockSizeMismatch {
                analysis: analysis.block_size(),
                synthesis: synthesis.block_size(),
            });
        }
        Ok(Self {
            analysis,
            synthesis,
        })
    }

    /// The orthogonal-MLT pair: `ha(n) = hs(n) =` the sine window
    /// (US7,383,180), for the given block size.
    pub fn orthogonal_sine(block_size: BlockSize) -> Self {
        let w = Window::sine(block_size);
        Self {
            analysis: w.clone(),
            synthesis: w,
        }
    }

    /// The analysis window `ha(n)`.
    #[inline]
    pub const fn analysis(&self) -> &Window {
        &self.analysis
    }

    /// The synthesis window `hs(n)`.
    #[inline]
    pub const fn synthesis(&self) -> &Window {
        &self.synthesis
    }

    /// The block size `M` both windows share.
    #[inline]
    pub const fn block_size(&self) -> BlockSize {
        self.analysis.block_size()
    }

    /// `true` iff the pair is the orthogonal arrangement
    /// (`ha(n) = hs(n)` coefficient-for-coefficient). The biorthogonal
    /// MLBT / NMLBT generalization is exactly the case where this is
    /// `false` (US6,240,380).
    #[inline]
    pub fn is_orthogonal(&self) -> bool {
        self.analysis == self.synthesis
    }
}

/// Length rejection for [`Window::apply_in_place`] /
/// [`Window::windowed`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InvalidWindowLen {
    /// Length the window requires (`2M`).
    pub expected: usize,
    /// Length the caller actually supplied.
    pub got: usize,
}

impl core::fmt::Display for InvalidWindowLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-wma: window input must be exactly 2*M samples \
             (expected {}, got {})",
            self.expected, self.got,
        )
    }
}

impl std::error::Error for InvalidWindowLen {}

/// Construction-time rejection for [`WindowPair::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvalidWindowPair {
    /// The analysis and synthesis windows disagree on the block size.
    BlockSizeMismatch {
        /// Block size of the offered analysis window.
        analysis: BlockSize,
        /// Block size of the offered synthesis window.
        synthesis: BlockSize,
    },
}

impl core::fmt::Display for InvalidWindowPair {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InvalidWindowPair::BlockSizeMismatch {
                analysis,
                synthesis,
            } => write!(
                f,
                "oxideav-wma: window pair block-size mismatch (analysis {} \
                 samples, synthesis {} samples)",
                analysis.samples(),
                synthesis.samples(),
            ),
        }
    }
}

impl std::error::Error for InvalidWindowPair {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::overlap_add::OverlapAdd;

    // ---------- WindowShape ----------

    #[test]
    fn shape_all_lists_the_three_patent_named_alternatives_in_order() {
        assert_eq!(
            WindowShape::ALL,
            [WindowShape::Sine, WindowShape::Mlbt, WindowShape::Nmlbt]
        );
    }

    #[test]
    fn only_the_sine_shape_is_realizable() {
        assert!(WindowShape::Sine.is_realizable());
        assert!(!WindowShape::Mlbt.is_realizable());
        assert!(!WindowShape::Nmlbt.is_realizable());
    }

    #[test]
    fn biorthogonal_predicate_partitions_the_shapes() {
        assert!(!WindowShape::Sine.is_biorthogonal());
        assert!(WindowShape::Mlbt.is_biorthogonal());
        assert!(WindowShape::Nmlbt.is_biorthogonal());
        // Every shape is exactly one of {plain-MLT sine, biorthogonal}.
        for shape in WindowShape::ALL {
            assert_ne!(shape.is_biorthogonal(), shape == WindowShape::Sine);
        }
    }

    // ---------- sine construction ----------

    #[test]
    fn sine_window_has_2m_coefficients_for_every_block_size() {
        for bs in BlockSize::ALL {
            let w = Window::sine(bs);
            assert_eq!(w.shape(), WindowShape::Sine);
            assert_eq!(w.block_size(), bs);
            assert_eq!(w.len(), 2 * bs.samples() as usize);
            assert_eq!(w.coeffs().len(), w.len());
            assert!(!w.is_empty());
        }
    }

    #[test]
    fn sine_coefficients_are_strictly_inside_the_unit_interval() {
        // (n + 1/2)·π/(2M) never reaches 0 or π for n in 0..2M, so
        // every coefficient is strictly in (0, 1].
        let w = Window::sine(BlockSize::S256);
        for &h in w.coeffs() {
            assert!(h > 0.0 && h <= 1.0);
        }
    }

    #[test]
    fn sine_first_coefficient_matches_the_closed_form() {
        // h(0) = sin(0.5·π / (2M)) = sin(π / (4M)).
        let w = Window::sine(BlockSize::S256);
        let expected = (core::f64::consts::PI / (4.0 * 256.0)).sin();
        assert_eq!(w.coeff(0).unwrap(), expected);
    }

    #[test]
    fn sine_window_rises_over_the_first_half_and_falls_over_the_second() {
        let w = Window::sine(BlockSize::S256);
        let m = 256;
        for n in 1..m {
            assert!(w.coeffs()[n] > w.coeffs()[n - 1], "rise at n={n}");
        }
        for n in (m + 1)..(2 * m) {
            assert!(w.coeffs()[n] < w.coeffs()[n - 1], "fall at n={n}");
        }
    }

    #[test]
    fn sine_window_is_symmetric() {
        // h(2M - 1 - n) = sin(π - (n + 1/2)·π/(2M)) = h(n).
        let w = Window::sine(BlockSize::S512);
        let two_m = w.len();
        for n in 0..two_m {
            let diff = (w.coeffs()[n] - w.coeffs()[two_m - 1 - n]).abs();
            assert!(diff <= 1e-12, "asymmetry at n={n}: {diff}");
        }
    }

    #[test]
    fn coeff_lookup_returns_none_past_the_window() {
        let w = Window::sine(BlockSize::S256);
        assert!(w.coeff(511).is_some());
        assert!(w.coeff(512).is_none());
        assert!(w.coeff(usize::MAX).is_none());
    }

    // ---------- TDAC power complementarity ----------

    #[test]
    fn sine_window_is_power_complementary_for_every_block_size() {
        // The defining 50%-overlap perfect-reconstruction condition of
        // the oddly-stacked TDAC filter bank (US6,029,126 /
        // US6,240,380): h(n)² + h(n+M)² = 1.
        for bs in BlockSize::ALL {
            let w = Window::sine(bs);
            assert!(w.is_power_complementary(1e-12), "block size {bs:?}");
        }
    }

    #[test]
    fn power_complementarity_predicate_detects_a_broken_window() {
        // A deliberately corrupted coefficient must trip the
        // predicate, pinning it to the per-position |sum - 1| check
        // rather than a vacuous pass.
        let w = Window::sine(BlockSize::S256);
        assert!(w.is_power_complementary(1e-9));
        let mut broken = w.clone();
        broken.coeffs[0] *= 2.0;
        assert!(!broken.is_power_complementary(1e-9));
    }

    // ---------- apply / windowed ----------

    #[test]
    fn apply_in_place_multiplies_sample_wise() {
        let w = Window::sine(BlockSize::S256);
        let mut block = vec![2.0_f64; 512];
        w.apply_in_place(&mut block).unwrap();
        for (n, &s) in block.iter().enumerate() {
            assert_eq!(s, 2.0 * w.coeffs()[n], "n={n}");
        }
    }

    #[test]
    fn windowed_matches_apply_in_place() {
        let w = Window::sine(BlockSize::S256);
        let block: Vec<f64> = (0..512).map(|n| (n as f64) * 0.25 - 30.0).collect();
        let fresh = w.windowed(&block).unwrap();
        let mut in_place = block.clone();
        w.apply_in_place(&mut in_place).unwrap();
        assert_eq!(fresh, in_place);
    }

    #[test]
    fn apply_in_place_rejects_wrong_lengths_without_mutating() {
        let w = Window::sine(BlockSize::S256);
        for bad_len in [0usize, 511, 513, 1024] {
            let mut block = vec![3.0_f64; bad_len];
            let err = w.apply_in_place(&mut block).unwrap_err();
            assert_eq!(
                err,
                InvalidWindowLen {
                    expected: 512,
                    got: bad_len,
                }
            );
            assert!(block.iter().all(|&s| s == 3.0), "mutated at len {bad_len}");
        }
    }

    #[test]
    fn windowed_rejects_wrong_lengths() {
        let w = Window::sine(BlockSize::S512);
        let err = w.windowed(&[1.0; 4]).unwrap_err();
        assert_eq!(
            err,
            InvalidWindowLen {
                expected: 1024,
                got: 4,
            }
        );
    }

    // ---------- WindowPair ----------

    #[test]
    fn orthogonal_sine_pair_shares_one_sine_window() {
        for bs in BlockSize::ALL {
            let pair = WindowPair::orthogonal_sine(bs);
            assert_eq!(pair.block_size(), bs);
            assert_eq!(pair.analysis(), pair.synthesis());
            assert!(pair.is_orthogonal());
            assert_eq!(pair.analysis().shape(), WindowShape::Sine);
        }
    }

    #[test]
    fn pair_constructor_accepts_matching_block_sizes() {
        let pair = WindowPair::new(
            Window::sine(BlockSize::S1024),
            Window::sine(BlockSize::S1024),
        )
        .unwrap();
        assert_eq!(pair.block_size(), BlockSize::S1024);
        assert!(pair.is_orthogonal());
    }

    #[test]
    fn pair_constructor_rejects_mismatched_block_sizes() {
        let err = WindowPair::new(Window::sine(BlockSize::S256), Window::sine(BlockSize::S512))
            .unwrap_err();
        assert_eq!(
            err,
            InvalidWindowPair::BlockSizeMismatch {
                analysis: BlockSize::S256,
                synthesis: BlockSize::S512,
            }
        );
    }

    // ---------- cross-module: window + overlap-add ----------

    #[test]
    fn sine_pair_plus_overlap_add_gives_unity_gain_in_steady_state() {
        // Weighted-overlap-add identity: feeding a constant signal
        // through analysis windowing, (identity in place of the
        // transform), synthesis windowing, and the overlap-add stage
        // reproduces the constant in every steady-state frame, because
        // out[k] = c·h(k+M)·h(k+M) + c·h(k)·h(k)
        //        = c·(h(k)² + h(k+M)²) = c
        // by the TDAC power-complementarity the patents define the MLT
        // by (US6,029,126 / US6,240,380; overlap-add per US7,383,180
        // decoder FIG.6).
        let pair = WindowPair::orthogonal_sine(BlockSize::S256);
        let mut oa = OverlapAdd::new(BlockSize::S256);
        let c = 0.75_f64;
        let segment = vec![c; 512];

        // First frame is the leading edge (half-windowed); skip it.
        let mut frame = pair.analysis().windowed(&segment).unwrap();
        pair.synthesis().apply_in_place(&mut frame).unwrap();
        let _leading = oa.step(&frame).unwrap();

        // Steady-state frames must reproduce the constant.
        for _ in 0..3 {
            let mut frame = pair.analysis().windowed(&segment).unwrap();
            pair.synthesis().apply_in_place(&mut frame).unwrap();
            let out = oa.step(&frame).unwrap();
            for (k, &s) in out.iter().enumerate() {
                assert!((s - c).abs() <= 1e-12, "k={k}: {s}");
            }
        }
    }

    #[test]
    fn windowed_block_length_matches_overlap_add_input_contract() {
        // The window's 2M output is exactly the input length the
        // overlap-add stage requires, for every patent-disclosed
        // block size.
        for bs in BlockSize::ALL {
            let w = Window::sine(bs);
            let oa = OverlapAdd::new(bs);
            assert_eq!(w.len(), oa.input_len());
        }
    }

    // ---------- error display ----------

    #[test]
    fn invalid_window_len_display_mentions_expected_and_got() {
        let err = InvalidWindowLen {
            expected: 8192,
            got: 17,
        };
        let s = format!("{err}");
        assert!(s.contains("8192"));
        assert!(s.contains("17"));
        assert!(s.contains("window"));
    }

    #[test]
    fn invalid_window_pair_display_names_both_sizes() {
        let err = InvalidWindowPair::BlockSizeMismatch {
            analysis: BlockSize::S256,
            synthesis: BlockSize::S4096,
        };
        let s = format!("{err}");
        assert!(s.contains("256"));
        assert!(s.contains("4096"));
        assert!(s.contains("mismatch"));
    }

    #[test]
    fn errors_implement_std_error() {
        fn assert_error<E: std::error::Error>(_e: &E) {}
        assert_error(&InvalidWindowLen {
            expected: 2,
            got: 1,
        });
        assert_error(&InvalidWindowPair::BlockSizeMismatch {
            analysis: BlockSize::S256,
            synthesis: BlockSize::S512,
        });
    }
}

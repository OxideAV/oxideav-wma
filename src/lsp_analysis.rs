//! Encoder side of the §3.1 line-spectral envelope: choose the ten
//! wire indices whose **decoder-exact** envelope
//! ([`crate::lsp_envelope`]) best follows a target amplitude
//! envelope.
//!
//! ## Method (encoder tuning, not a staged fact)
//!
//! The decoder's envelope is `W(ω) = |A(e^{jω})|^(−1/2)` for the
//! order-10 monic polynomial `A` the codebook indices select through
//! its line-spectral pair. `1/|A|²` is the classical all-pole model
//! spectrum, so a target amplitude envelope `T` is fitted as the
//! all-pole model of the power spectrum `T⁴` (`W ∝ (1/|A|²)^(1/4)`):
//!
//! 1. autocorrelation of `T⁴` on the decoder's own grid `ω_i = iπ/L`,
//!    with a small white-noise floor for conditioning;
//! 2. Levinson–Durbin recursion, order 10, to the predictor `A`;
//! 3. the line-spectral pair of `A`: `P(z) = A(z) + z⁻¹¹A(z⁻¹)`,
//!    `Q(z) = A(z) − z⁻¹¹A(z⁻¹)`, whose unit-circle roots (found by
//!    sign-change scan + bisection on the real symmetric forms)
//!    interleave — the `P` roots go to the even codebook rows and the
//!    `Q` roots to the odd rows, ascending, exactly as the decoder's
//!    `P`/`Q` builder consumes them;
//! 4. each `c_i = −2cos ω_i` quantised to the nearest reachable entry
//!    of its codebook row;
//! 5. a few passes of coordinate descent over the indices on the
//!    **exact** decoder envelope, minimising the gain-free log-spectral
//!    distance to `T` over the coded bins — this is what absorbs the
//!    codebook rows' limited ranges and the fourth-power dynamic range
//!    of step 1.
//!
//! The returned envelope is the decoder's, bit for bit, so the
//! quantiser normalises by exactly what the decoder will multiply by.

use crate::lsp_envelope::{codebook_entry, lsp_envelope, LspEnvelope, LSP_ORDER};
use crate::vendor_frame::LSP_INDEX_WIDTHS;

/// Number of reachable entries of codebook row `i` (`2^width`).
fn reach(i: usize) -> usize {
    1usize << LSP_INDEX_WIDTHS[i]
}

/// Autocorrelation `r[0..=order]` of the power spectrum sampled on
/// `ω_i = iπ/L`, `i = 0..=L` (`power.len() == L + 1`), i.e. the
/// inverse DFT of the symmetric spectrum on the full circle.
fn autocorrelation(power: &[f64], order: usize) -> Vec<f64> {
    let l = power.len() - 1;
    (0..=order)
        .map(|k| {
            let mut acc = 0.5 * power[0];
            for (i, &p) in power.iter().enumerate().take(l).skip(1) {
                acc += p * (k as f64 * i as f64 * std::f64::consts::PI / l as f64).cos();
            }
            acc += 0.5 * power[l] * if k % 2 == 0 { 1.0 } else { -1.0 };
            acc / l as f64
        })
        .collect()
}

/// Levinson–Durbin: the monic predictor `a[1..=order]` of the
/// autocorrelation `r`. `None` when the recursion is not minimum
/// phase (a reflection coefficient reaches 1).
fn levinson(r: &[f64], order: usize) -> Option<Vec<f64>> {
    let mut a = vec![0.0f64; order + 1];
    a[0] = 1.0;
    let mut err = r[0];
    if err <= 0.0 {
        return None;
    }
    for m in 1..=order {
        let mut acc = r[m];
        for k in 1..m {
            acc += a[k] * r[m - k];
        }
        let refl = -acc / err;
        if refl.abs() >= 1.0 {
            return None;
        }
        let prev = a.clone();
        for k in 1..m {
            a[k] = prev[k] + refl * prev[m - k];
        }
        a[m] = refl;
        err *= 1.0 - refl * refl;
        if err <= 0.0 {
            return None;
        }
    }
    Some(a)
}

/// The five unit-circle roots in `(0, π)` of a symmetric (`sym`) or
/// antisymmetric polynomial of degree 11 given through its first six
/// coefficients `c[0..=5]`: the real forms `2 Σ c_k cos((5.5 − k)ω)`
/// / `2 Σ c_k sin((5.5 − k)ω)`, scanned for sign changes and
/// bisected. `None` unless exactly five roots are found.
fn unit_circle_roots(c: &[f64; 6], sym: bool) -> Option<[f64; 5]> {
    let eval = |w: f64| -> f64 {
        c.iter()
            .enumerate()
            .map(|(k, &ck)| {
                let m = (5.5 - k as f64) * w;
                ck * if sym { m.cos() } else { m.sin() }
            })
            .sum::<f64>()
    };
    const GRID: usize = 4096;
    let mut roots = Vec::with_capacity(6);
    let step = std::f64::consts::PI / GRID as f64;
    let mut w0 = step * 0.5;
    let mut f0 = eval(w0);
    for g in 1..GRID {
        let w1 = step * (g as f64 + 0.5);
        let f1 = eval(w1);
        if f0 == 0.0 {
            roots.push(w0);
        } else if (f0 < 0.0) != (f1 < 0.0) {
            let (mut lo, mut hi, mut flo) = (w0, w1, f0);
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                let fm = eval(mid);
                if (fm < 0.0) == (flo < 0.0) {
                    lo = mid;
                    flo = fm;
                } else {
                    hi = mid;
                }
            }
            roots.push(0.5 * (lo + hi));
        }
        w0 = w1;
        f0 = f1;
    }
    (roots.len() == 5).then(|| {
        let mut out = [0.0f64; 5];
        out.copy_from_slice(&roots);
        out
    })
}

/// The ten line-spectral frequencies of the predictor `a[0..=10]`
/// (`a[0] == 1`) in codebook-row order: `P` roots on the even rows,
/// `Q` roots on the odd rows, each ascending.
fn lpc_to_lsp(a: &[f64]) -> Option<[f64; LSP_ORDER]> {
    // p_k = a_k + a_{11-k}, q_k = a_k − a_{11-k}, a_11 = 0.
    let coef = |k: usize| if k <= LSP_ORDER { a[k] } else { 0.0 };
    let mut p = [0.0f64; 6];
    let mut q = [0.0f64; 6];
    for k in 0..6 {
        p[k] = coef(k) + coef(11 - k);
        q[k] = coef(k) - coef(11 - k);
    }
    let pr = unit_circle_roots(&p, true)?;
    let qr = unit_circle_roots(&q, false)?;
    let mut out = [0.0f64; LSP_ORDER];
    for i in 0..5 {
        out[2 * i] = pr[i];
        out[2 * i + 1] = qr[i];
    }
    Some(out)
}

/// Nearest reachable codebook entry of row `i` to `c = −2cos ω`.
fn quantise_row(i: usize, c: f64) -> u8 {
    (0..reach(i))
        .min_by(|&x, &y| {
            let dx = (f64::from(codebook_entry(i, x as u8)) - c).abs();
            let dy = (f64::from(codebook_entry(i, y as u8)) - c).abs();
            dx.partial_cmp(&dy).unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(0) as u8
}

/// Gain-free log-spectral distance between the decoder envelope of
/// `indices` and the target over `lo..hi`; `f64::INFINITY` when the
/// conversion fails.
fn distance(indices: &[u8; LSP_ORDER], log_target: &[f64], lo: usize, hi: usize, l: usize) -> f64 {
    let n = log_target.len();
    let Ok(env) = lsp_envelope(indices, n, l) else {
        return f64::INFINITY;
    };
    let diff: Vec<f64> = env.weights[lo..hi]
        .iter()
        .zip(log_target[lo..hi].iter())
        .map(|(&w, &t)| f64::from(w).max(1e-30).ln() - t)
        .collect();
    if diff.is_empty() {
        return 0.0;
    }
    let mean = diff.iter().sum::<f64>() / diff.len() as f64;
    diff.iter().map(|d| (d - mean) * (d - mean)).sum::<f64>()
}

/// The default index set the search starts from when the all-pole
/// fit is unusable (a flat target): every row at its middle entry.
fn default_indices() -> [u8; LSP_ORDER] {
    let mut out = [0u8; LSP_ORDER];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = (reach(i) / 2) as u8;
    }
    out
}

/// Fit the ten §3.1 indices to a target amplitude envelope `target`
/// (one value per bin, `target.len() == block_len`; only the bins
/// `coded_lo..coded_hi` count) on the decoder grid `grid_len`
/// (`block_len ≤ grid_len`, both as [`crate::lsp_envelope::lsp_envelope`]
/// takes them). Returns the indices and their decoder-exact envelope.
pub fn fit_lsp_envelope(
    target: &[f64],
    coded_lo: usize,
    coded_hi: usize,
    grid_len: usize,
) -> ([u8; LSP_ORDER], LspEnvelope) {
    let n = target.len();
    let hi = coded_hi.min(n);
    let lo = coded_lo.min(hi);
    let t_max = target[lo..hi].iter().cloned().fold(0.0, f64::max);
    // Clamp the fit target's dynamic range (the exponent path's
    // 30 dB envelope range): deep valleys are where the fourth power
    // would spend the model's poles for nothing audible.
    let floor = t_max * 10f64.powf(-24.0 / 16.0);
    let clamped: Vec<f64> = (0..n)
        .map(|i| {
            let j = i.clamp(lo, hi.saturating_sub(1).max(lo));
            target[j].max(floor).max(1e-300)
        })
        .collect();
    let log_target: Vec<f64> = clamped.iter().map(|t| t.ln()).collect();

    // Steps 1–4: all-pole fit of T⁴ on the decoder grid, LSP, quantise.
    let mut start = default_indices();
    if t_max > 0.0 && hi > lo {
        let mut power = vec![0.0f64; grid_len + 1];
        for (i, p) in power.iter_mut().enumerate() {
            let j = i.min(n - 1);
            let t = clamped[j] / t_max;
            *p = t * t * t * t;
        }
        let mut r = autocorrelation(&power, LSP_ORDER);
        r[0] *= 1.0 + 1e-6;
        if let Some(a) = levinson(&r, LSP_ORDER) {
            if let Some(omegas) = lpc_to_lsp(&a) {
                for (i, &w) in omegas.iter().enumerate() {
                    start[i] = quantise_row(i, -2.0 * w.cos());
                }
            }
        }
    }

    // Step 5: coordinate descent on the exact envelope.
    let mut best = start;
    let mut best_d = distance(&best, &log_target, lo, hi, grid_len);
    for _pass in 0..3 {
        let mut improved = false;
        for i in 0..LSP_ORDER {
            let cur = best[i] as i32;
            for delta in [-2i32, -1, 1, 2] {
                let cand = cur + delta;
                if cand < 0 || cand >= reach(i) as i32 {
                    continue;
                }
                let mut trial = best;
                trial[i] = cand as u8;
                let d = distance(&trial, &log_target, lo, hi, grid_len);
                if d < best_d {
                    best_d = d;
                    best = trial;
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
    }
    let env = lsp_envelope(&best, n, grid_len)
        .or_else(|_| lsp_envelope(&default_indices(), n, grid_len))
        .expect("the default index set converts");
    (best, env)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levinson_recovers_a_known_predictor() {
        // Autocorrelation of an AR(2) process x[n] = 0.5x[n-1] −
        // 0.3x[n-2] + e[n]: the recursion must return a = (−0.5, 0.3).
        let (a1, a2) = (-0.5f64, 0.3f64);
        // Solve the Yule–Walker relations for r[0..=2] (r[0] = 1).
        let r1 = -a1 / (1.0 + a2);
        let r2 = -a1 * r1 - a2;
        let r = [1.0, r1, r2, -a1 * r2 - a2 * r1];
        let a = levinson(&r, 2).unwrap();
        assert!(
            (a[1] - a1).abs() < 1e-12 && (a[2] - a2).abs() < 1e-12,
            "{a:?}"
        );
    }

    #[test]
    fn lsp_of_a_flat_predictor_is_the_uniform_grid() {
        // A = 1: P = 1 + z⁻¹¹, Q = 1 − z⁻¹¹, roots at ω = kπ/11.
        let mut a = vec![0.0f64; 11];
        a[0] = 1.0;
        let w = lpc_to_lsp(&a).unwrap();
        for (i, &wi) in w.iter().enumerate() {
            let want = (i + 1) as f64 * std::f64::consts::PI / 11.0;
            assert!((wi - want).abs() < 1e-9, "root {i}: {wi} vs {want}");
        }
    }

    #[test]
    fn fit_follows_a_shaped_target() {
        // A low-pass target (loud below a quarter of the band, 20 dB
        // quieter above): the fitted decoder envelope must be
        // markedly larger in the loud region than in the quiet one.
        let n = 512usize;
        let target: Vec<f64> = (0..n).map(|i| if i < n / 4 { 1.0 } else { 0.1 }).collect();
        let (idx, env) = fit_lsp_envelope(&target, 0, 466, n);
        for (i, &v) in idx.iter().enumerate() {
            assert!((v as usize) < reach(i));
        }
        let mean = |r: std::ops::Range<usize>| {
            env.weights[r.clone()]
                .iter()
                .map(|&w| f64::from(w))
                .sum::<f64>()
                / r.len() as f64
        };
        let loud = mean(0..n / 4);
        let quiet = mean(n / 4 + 32..466);
        let ratio_db = 20.0 * (loud / quiet).log10();
        assert!(
            ratio_db > 10.0,
            "loud/quiet envelope ratio {ratio_db:.1} dB"
        );
    }

    #[test]
    fn flat_target_fits_without_panicking_on_every_geometry() {
        for &(n, l) in &[
            (128usize, 128usize),
            (256, 512),
            (512, 512),
            (1024, 1024),
            (2048, 2048),
        ] {
            let target = vec![1.0f64; n];
            let (idx, env) = fit_lsp_envelope(&target, 0, n - n / 11, l);
            assert_eq!(env.weights.len(), n);
            assert!(env.max > 0.0, "{idx:?}");
        }
        // And a silent target.
        let (_, env) = fit_lsp_envelope(&[0.0; 512], 0, 466, 512);
        assert!(env.max > 0.0);
    }
}

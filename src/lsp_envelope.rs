//! §3.1 line-spectral envelope conversion — ten wire indices → the
//! per-bin spectral envelope `W[i] = |A(e^{jω_i})|^(−1/2)` and its
//! maximum, **bit-exact** to the vendor decoder.
//!
//! ## Source
//!
//! * `docs/audio/wma/frame-bit-layout.md` §3.1 — the field widths
//!   (3,4,4,4,4,4,4,4,3,3 bits), the codebook → `P`/`Q` → `A`
//!   construction, the evaluation grid `ω_i = iπ/L` (`L` = the block
//!   length when `flags2` bit 5 is set, else `frame_length`), the
//!   `|A|²` → envelope mapping through the two fourth-root look-up
//!   tables the decoder builds at stream open, the recorded maximum
//!   and the zero-maximum decode error.
//! * `docs/audio/wma/tables/lsp_envelope_exact.py` — the round-09
//!   **bit-exact model** of the conversion: the decoder's x87 dataflow
//!   lifted operation by operation (codebook → symmetric polynomial
//!   products → `A` → the three-stage radix-4 `|A(ω)|²` evaluator →
//!   LUT lookup → running maximum), with one binary32 rounding at
//!   every 32-bit store and binary64 arithmetic everywhere else (the
//!   Windows-default x87 control word, which the module never
//!   changes). Round 10 validated that model against the sandboxed
//!   vendor decoder on all 3 931 conversions (2 012 672 bins) of the
//!   committed `cand_mono8k_8kbps_v8` stream, bit for bit. This
//!   module is a transliteration of that model; `tests/lsp_model.rs`
//!   pins it bit-exact against the staged script on random inputs at
//!   every grid length and on the vendor stream's own 3 931
//!   conversions.
//! * `docs/audio/wma/tables/wma-lsp-*.csv` — the codebook and the two
//!   twiddle tables ([`crate::lsp_tables`]); the root LUTs are
//!   regenerated from the staged builder arithmetic ([`RootLuts`]).
//!
//! ## Floating-point discipline
//!
//! Every quantity the decoder keeps in an x87 register is an `f64`
//! here; every quantity it stores to a 32-bit slot is rounded with
//! `as f32` (round-to-nearest-even, the same rounding as the store).
//! Products of two `f32` values are exact in `f64`, so the only
//! roundings are the sums the decoder rounds and the stores. The
//! association of every expression follows the instruction stream —
//! that ordering, not the algebra, is what makes the result
//! bit-exact. No fused multiply-add is ever used.

use crate::lsp_tables::{LSP_CODEBOOK_BITS, LSP_TWIDDLES_3_2_1_BITS, LSP_TWIDDLES_8_4_BITS};
use std::sync::OnceLock;

/// Number of line-spectral indices per coded channel (§3.1).
pub const LSP_ORDER: usize = 10;

/// `sqrt(2)` as the decoder's `f32` constant (`.rdata 0x1e0d0`, bits
/// `0x3fb504f3`), widened exactly.
const SQRT2: f64 = 1.414_213_538_169_860_8;
/// `1/sqrt(2)` as the decoder's `f32` constant (`.rdata 0x1e0cc`, bits
/// `0x3f3504f3`), widened exactly.
const SQRT1_2: f64 = 0.707_106_769_084_930_4;

/// Binary64 → binary32 round-to-nearest-even (what a 32-bit store
/// does).
#[inline(always)]
fn r32(x: f64) -> f32 {
    x as f32
}

/// The §3.1 conversion's failure modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LspError {
    /// The grid length is not a power of two in `16..=2048`, or the
    /// block length exceeds it.
    BadGeometry {
        /// The requested block length `N`.
        block_len: usize,
        /// The requested grid length `L`.
        grid_len: usize,
    },
    /// A wire index outside its field's range (rows 0, 8 and 9 reach
    /// entries 0–7 only; every row is 16 wide).
    IndexOutOfRange {
        /// Which of the ten indices.
        position: usize,
        /// Its value.
        index: u8,
    },
    /// The envelope maximum is zero — the decoder's own decode error
    /// for this conversion.
    ZeroMaximum,
}

impl core::fmt::Display for LspError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            LspError::BadGeometry {
                block_len,
                grid_len,
            } => write!(
                f,
                "oxideav-wma: LSP envelope geometry N={block_len} L={grid_len} unsupported"
            ),
            LspError::IndexOutOfRange { position, index } => write!(
                f,
                "oxideav-wma: LSP index {index} at position {position} outside the codebook"
            ),
            LspError::ZeroMaximum => f.write_str("oxideav-wma: LSP envelope maximum is zero"),
        }
    }
}

impl std::error::Error for LspError {}

/// The two fourth-root look-up tables the decoder builds once at
/// stream open (`.text 0x6dc0`, staged as `wma-lsp-root-lut-mantissa`
/// / `-exponent`, labelled DERIVED): `M[m] = f32(1 / sqrt(sqrt(x)))`
/// for `x` the `f32` with bits `0x3f800000 | (m << 11)`
/// (`1 + m/4096`), and `E[e] = f32(1 / sqrt(sqrt(x)))` for `x` the
/// `f32` with bits `e << 23` (`2^(e − 127)`), `e = 1..=254`; slots
/// `E[0]` and `E[255]` stay zero. Each of the three x87 operations
/// (two square roots and the division) rounds to binary64, the store
/// to binary32 — exactly what `f64::sqrt` and `as f32` do.
#[derive(Debug, Clone)]
pub struct RootLuts {
    /// Mantissa part, 4096 entries.
    pub mantissa: Vec<f32>,
    /// Exponent part, 256 entries.
    pub exponent: Vec<f32>,
}

impl RootLuts {
    /// Build both tables from the staged generation arithmetic.
    pub fn build() -> Self {
        let mut mantissa = vec![0.0f32; 4096];
        for (m, slot) in mantissa.iter_mut().enumerate() {
            let x = f64::from(f32::from_bits(0x3f80_0000 | ((m as u32) << 11)));
            *slot = r32(1.0 / x.sqrt().sqrt());
        }
        let mut exponent = vec![0.0f32; 256];
        for (e, slot) in exponent.iter_mut().enumerate().take(255).skip(1) {
            let x = f64::from(f32::from_bits((e as u32) << 23));
            *slot = r32(1.0 / x.sqrt().sqrt());
        }
        Self { mantissa, exponent }
    }

    /// The process-wide tables (built on first use).
    pub fn shared() -> &'static RootLuts {
        static LUTS: OnceLock<RootLuts> = OnceLock::new();
        LUTS.get_or_init(RootLuts::build)
    }

    /// `|A|²` (as its `f32` bits) → `|A|^(−1/2)` through the split
    /// lookup: `M[(b >> 11) & 0xfff] · E[(b >> 23) & 0xff]`, the
    /// binary64 product of the two `f32` entries (exact).
    #[inline]
    pub fn lookup(&self, mag2_bits: u32) -> f64 {
        let m = ((mag2_bits >> 11) & 0xfff) as usize;
        let e = ((mag2_bits >> 23) & 0xff) as usize;
        f64::from(self.mantissa[m]) * f64::from(self.exponent[e])
    }
}

/// The staged codebook entry `c_i` for LSP index `i`, wire value
/// `index` (`wma-lsp-codebook`, row `i`, column `index`).
#[inline]
pub fn codebook_entry(i: usize, index: u8) -> f32 {
    f32::from_bits(LSP_CODEBOOK_BITS[i][usize::from(index) & 15])
}

/// Product of a symmetric polynomial `poly` with the symmetric
/// quadratic `quad` (`.text 0x6b10` even-length / `0x69f0` odd-length
/// variants): only the low half is accumulated, then mirrored. Each
/// accumulation step is `fld; fmul; fadd dword; fstp dword` — the
/// product exact in binary64, the add rounded to binary64, the store
/// to binary32.
fn sym_product(poly: &[f32], quad: &[f32; 3], odd: bool) -> Vec<f32> {
    let n = poly.len();
    let m = quad.len();
    let length = m + n - 1;
    let mut tmp = vec![0.0f32; length];
    for (i, slot) in tmp.iter_mut().enumerate().take(m.min(n)) {
        let mut acc = 0.0f32;
        for j in 0..=i {
            acc = r32(f64::from(poly[i - j]) * f64::from(quad[j]) + f64::from(acc));
        }
        *slot = acc;
    }
    let half = (m + n) >> 1;
    for i in m..half {
        let mut acc = 0.0f32;
        for (j, &q) in quad.iter().enumerate() {
            acc = r32(f64::from(poly[i - j]) * f64::from(q) + f64::from(acc));
        }
        tmp[i] = acc;
    }
    let mut out = vec![0.0f32; length];
    for i in 0..(length >> 1) {
        out[i] = tmp[i];
        out[length - 1 - i] = tmp[i];
    }
    if odd {
        out[length >> 1] = tmp[length >> 1];
    }
    out
}

/// The P/Q builder (`.text 0x6c20`): `P(z) = (1 + z⁻¹) ∏ (1 + c_{2k}
/// z⁻¹ + z⁻²)` (length 12) and `Q'(z) = (1 − z⁻¹) Q(z)` given as the
/// five differences `Q'[i] = f32(Q[i] − Q[i−1])`, `i = 1..=5`, of
/// `Q(z) = ∏ (1 + c_{2k+1} z⁻¹ + z⁻²)` (length 11). Returns
/// `(P, Q', Q)`.
pub fn lsp_to_pq(indices: &[u8; LSP_ORDER]) -> (Vec<f32>, [f32; 6], Vec<f32>) {
    let mut p = vec![1.0f32, 1.0f32];
    for i in [0usize, 2, 4, 6, 8] {
        p = sym_product(&p, &[1.0, codebook_entry(i, indices[i]), 1.0], false);
    }
    debug_assert_eq!(p.len(), 12);
    let mut q = vec![1.0f32, codebook_entry(1, indices[1]), 1.0f32];
    for i in [3usize, 5, 7, 9] {
        q = sym_product(&q, &[1.0, codebook_entry(i, indices[i]), 1.0], true);
    }
    debug_assert_eq!(q.len(), 11);
    let mut qd = [0.0f32; 6];
    for i in 1..=5 {
        qd[i] = r32(f64::from(q[i]) - f64::from(q[i - 1]));
    }
    (p, qd, q)
}

/// `A = (P + Q')/2` (`.text 0x6990`), output as `−a_1 .. −a_10`:
/// `out[i−1] = f32(−((Q'[i] + P[i]) · 0.5))`,
/// `out[10−i] = f32(−((P[i] − Q'[i]) · 0.5))` for `i = 1..=5` (the
/// halving and the negation are exact; each output is one rounding of
/// the sum).
pub fn pq_to_neg_a(p: &[f32], qd: &[f32; 6]) -> [f32; LSP_ORDER] {
    let mut out = [0.0f32; LSP_ORDER];
    for i in 1..=5 {
        out[i - 1] = r32(-((f64::from(qd[i]) + f64::from(p[i])) * 0.5));
        out[10 - i] = r32(-((f64::from(p[i]) - f64::from(qd[i])) * 0.5));
    }
    out
}

/// Whether `l` is a legal evaluation grid length.
fn legal_grid(l: usize) -> bool {
    l.is_power_of_two() && (16..=2048).contains(&l)
}

/// `|A(e^{jω})|²` on `ω = iπ/L` (`.text 0x6e40`): the decoder's
/// three-stage evaluator over a `2L`-cell `f32` buffer whose cells
/// `0..L` hold `|A|²` on return (cells `L+1..2L` hold sub-transform
/// intermediates). `neg_a` is the output of [`pq_to_neg_a`]. The
/// buffer layout: with `s = L/16`, `q = L/2`, four `q`-cell blocks
/// hold the stride-4 sub-sequences of `(1, a_1, …, a_10)` in
/// Hartley-like `cos + sin` form — block 0 `{1, a4, a8}`, block 1
/// `{a2, a6, a10}`, block 2 `{a1, a5, a9}`, block 3 `{a3, a7}` — so
/// that stage 3 recovers `Re` and `Im` from mirrored cells by
/// half-sums.
///
/// Returns `None` for an illegal grid length.
pub fn evaluate_mag2(neg_a: &[f32; LSP_ORDER], l: usize) -> Option<Vec<f32>> {
    if !legal_grid(l) {
        return None;
    }
    let a: Vec<f64> = neg_a.iter().map(|&v| -f64::from(v)).collect();
    let (a1, a2, a3, a4, a5, a6, a7, a8, a9, a10) =
        (a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]);
    let s = l / 16;
    let q = l / 2;
    let step = 2048 / l;
    let mut b = vec![0.0f32; 2 * l];

    // ---- stage 1: the eight base positions of each block ----------
    // P = a8 + 1 is computed once and kept on the x87 stack
    // (binary64, never rounded) for all of stage 1.
    let p = a8 + 1.0;
    let a4s2 = a4 * SQRT2;
    b[s] = r32(a4s2 + p);
    let om8 = 1.0 - a8;
    b[2 * s] = r32(om8 + a4);
    b[3 * s] = r32(om8);
    b[5 * s] = r32(p - a4s2);
    b[6 * s] = r32(om8 - a4);
    b[7 * s] = r32(om8);
    // block 1 {a2, a6, a10}, base cell 8s
    let m4 = f64::from(r32(a10 + a2));
    let a6s2 = a6 * SQRT2;
    b[9 * s] = r32(m4 + a6s2);
    let d2 = a2 - a10;
    b[10 * s] = r32(d2 + a6);
    b[11 * s] = r32(d2);
    b[13 * s] = r32(m4 - a6s2);
    b[14 * s] = r32(d2 - a6);
    b[15 * s] = r32(d2);
    // block 2 {a1, a5, a9}, base cell 16s
    let mpc = f64::from(r32(a9 + a1));
    let a5s2 = a5 * SQRT2;
    b[17 * s] = r32(mpc + a5s2);
    let d1 = a1 - a9;
    b[18 * s] = r32(d1 + a5);
    b[19 * s] = r32(d1);
    b[21 * s] = r32(mpc - a5s2);
    b[22 * s] = r32(d1 - a5);
    b[23 * s] = r32(d1);
    // block 3 {a3, a7}, base cell 24s
    let a7s2 = a7 * SQRT2;
    b[25 * s] = r32(a7s2 + a3);
    b[26 * s] = r32(a7 + a3);
    b[27 * s] = r32(a3); // bit copy (a3 is exactly representable)
    b[29 * s] = r32(a3 - a7s2);
    b[30 * s] = r32(a3 - a7);
    b[31 * s] = r32(a3);
    // the j = 0 and j = 4 positions of all four blocks
    let m20 = f64::from(r32(a7 + a5));
    let m1c = f64::from(r32(a5 - a7));
    let m10 = f64::from(r32(a4 + p));
    let pm = p - a4;
    let mc = f64::from(r32(a6 + m4));
    let t = (m20 + a3) + mpc;
    let m5c = f64::from(r32(t));
    b[0] = r32((t + mc) + m10);
    let rr = ((mpc - m1c) - a3) * SQRT1_2;
    b[4 * s] = r32(rr + pm);
    b[8 * s] = r32(m10 - mc);
    b[12 * s] = r32(pm - rr);
    b[16 * s] = r32((mc + m10) - m5c);
    let r2 = ((mpc - m20) + a3) * SQRT1_2;
    b[20 * s] = r32((r2 - m4) + a6);
    b[24 * s] = r32((m1c + mpc) - a3);
    b[28 * s] = r32((m4 - a6) + r2);
    // squares of the cells that are already final
    let sq = |x: f32| f64::from(x) * f64::from(x);
    b[0] = r32(sq(b[0]));
    b[4 * s] = r32(sq(b[28 * s]) + sq(b[4 * s]));
    b[8 * s] = r32(sq(b[24 * s]) + sq(b[8 * s]));
    b[12 * s] = r32(sq(b[20 * s]) + sq(b[12 * s]));
    b[16 * s] = r32(sq(b[16 * s]));

    // ---- stage 2: k = 1..s-1 (skipped when s <= 1) ----------------
    for k in 1..s {
        let tw = &LSP_TWIDDLES_8_4_BITS[k * step];
        let (c0, c1, c2, c3) = (
            f64::from(f32::from_bits(tw[0])),
            f64::from(f32::from_bits(tw[1])),
            f64::from(f32::from_bits(tw[2])),
            f64::from(f32::from_bits(tw[3])),
        );
        // block 0 {1, a4, a8}
        let c1a4 = c1 * a4;
        let mpc = f64::from(r32(c1a4));
        let c0a8 = c0 * a8;
        b[k] = r32((c1a4 + c0a8) + 1.0);
        let c2a8 = c2 * a8;
        let m4 = f64::from(r32(c2a8));
        b[2 * s - k] = r32((c2a8 + mpc) + 1.0);
        let m8 = f64::from(r32(c3 * a4));
        let m18 = f64::from(r32(1.0 - c0a8));
        b[2 * s + k] = r32(m18 - m8);
        let om4 = 1.0 - m4;
        let m14 = f64::from(r32(om4));
        b[4 * s - k] = r32(om4 + m8);
        b[4 * s + k] = r32((c0a8 + 1.0) - mpc);
        b[6 * s - k] = r32((m4 + 1.0) - mpc);
        b[6 * s + k] = r32(m18 + m8);
        b[8 * s - k] = r32(m14 - m8);
        // block 1 {a2, a6, a10}, base 8s
        let c1a6 = c1 * a6;
        let mpc = f64::from(r32(c1a6));
        let c0a10 = c0 * a10;
        b[8 * s + k] = r32((c1a6 + c0a10) + a2);
        let c2a10 = c2 * a10;
        let m4 = f64::from(r32(c2a10));
        b[10 * s - k] = r32((c2a10 + mpc) + a2);
        let m8 = f64::from(r32(c3 * a6));
        let m14 = f64::from(r32(a2 - c0a10));
        b[10 * s + k] = r32(m14 - m8);
        let a2m4 = a2 - m4;
        let m18 = f64::from(r32(a2m4));
        b[12 * s - k] = r32(a2m4 + m8);
        b[12 * s + k] = r32((c0a10 + a2) - mpc);
        b[14 * s - k] = r32((m4 + a2) - mpc);
        b[14 * s + k] = r32(m14 + m8);
        b[16 * s - k] = r32(m18 - m8);
        // block 2 {a1, a5, a9}, base 16s
        let c1a5 = c1 * a5;
        let mpc = f64::from(r32(c1a5));
        let c0a9 = c0 * a9;
        b[16 * s + k] = r32((c1a5 + c0a9) + a1);
        let c2a9 = c2 * a9;
        let m4 = f64::from(r32(c2a9));
        b[18 * s - k] = r32((c2a9 + mpc) + a1);
        let m8 = f64::from(r32(c3 * a5));
        let m14 = f64::from(r32(a1 - c0a9));
        b[18 * s + k] = r32(m14 - m8);
        let a1m4 = a1 - m4;
        let m18 = f64::from(r32(a1m4));
        b[20 * s - k] = r32(a1m4 + m8);
        b[20 * s + k] = r32((c0a9 + a1) - mpc);
        b[22 * s - k] = r32((m4 + a1) - mpc);
        b[22 * s + k] = r32(m14 + m8);
        b[24 * s - k] = r32(m18 - m8);
        // block 3 {a3, a7}, base 24s
        let c1a7 = c1 * a7;
        let mpc = f64::from(r32(c1a7));
        let v = r32(c1a7 + a3);
        b[26 * s - k] = v;
        b[24 * s + k] = v;
        let c3a7 = c3 * a7;
        b[26 * s + k] = r32(a3 - c3a7);
        b[28 * s - k] = r32(c3a7 + a3);
        let w = r32(a3 - mpc);
        b[28 * s + k] = w;
        b[30 * s - k] = w;
        b[30 * s + k] = r32(c3a7 + a3);
        b[32 * s - k] = r32(a3 - c3a7);
    }

    // ---- stage 3: k = 1..q/2-1 (skipped when L/4 <= 1) ------------
    for k in 1..q / 2 {
        let tw = &LSP_TWIDDLES_3_2_1_BITS[k * step];
        let d: [f64; 6] = core::array::from_fn(|i| f64::from(f32::from_bits(tw[i])));
        let (d0, d1, d2, d3, d4, d5) = (d[0], d[1], d[2], d[3], d[4], d[5]);
        let (x0k, x0m) = (f64::from(b[k]), f64::from(b[q - k]));
        let (x1k, x1m) = (f64::from(b[q + k]), f64::from(b[2 * q - k]));
        let (x2k, x2m) = (f64::from(b[2 * q + k]), f64::from(b[3 * q - k]));
        let (x3k, x3m) = (f64::from(b[3 * q + k]), f64::from(b[4 * q - k]));
        let mpc = f64::from(r32((d3 * x1m) + (d2 * x1k)));
        let m10 = f64::from(r32((d3 * x1k) - (d2 * x1m)));
        let u = (d5 * x2m) + (d4 * x2k);
        let v = (d1 * x3m) + (d0 * x3k);
        let mc = f64::from(r32(x0k + x0m));
        let m28 = f64::from(r32(x0k - x0m));
        let m68 = f64::from(r32((((mc + v) + u) + mpc) * 0.5));
        let m64 = f64::from(r32((((u + m10) - v) - m28) * 0.5));
        let m14 = f64::from(r32((((u - m10) - v) + m28) * 0.5));
        let m18 = f64::from(r32((((mpc - u) - v) + mc) * 0.5));
        let m2c = f64::from(r32((d5 * x2k) - (d4 * x2m)));
        let w = (d1 * x3k) - (d0 * x3m);
        let r1 = (((w + m2c) + m28) + m10) * 0.5;
        b[k] = r32((r1 * r1) + (m68 * m68));
        let r2 = (((m2c - mpc) - w) + mc) * 0.5;
        b[q - k] = r32((r2 * r2) + (m64 * m64));
        let r3 = ((w - (m2c + mpc)) + mc) * 0.5;
        b[q + k] = r32((r3 * r3) + (m14 * m14));
        let r4 = (((w - m10) + m2c) - m28) * 0.5;
        b[l - k] = r32((r4 * r4) + (m18 * m18));
    }
    Some(b)
}

/// The converted envelope of one coded channel.
#[derive(Debug, Clone, PartialEq)]
pub struct LspEnvelope {
    /// `W[i]`, `i = 0..block_len` — the decoder's stored `f32` values.
    pub weights: Vec<f32>,
    /// The recorded maximum (`chan+0x70`), strictly positive.
    pub max: f32,
}

/// The epilogue (`.text 0x770c`–`0x779c`): for `i = 0..N`, look
/// `|A|²[i]` up through [`RootLuts::lookup`], store the `f32`, and
/// track the maximum (compared in binary64, stored as `f32`). A zero
/// maximum is the decoder's decode error.
pub fn envelope_from_mag2(
    mag2: &[f32],
    block_len: usize,
    luts: &RootLuts,
) -> Result<LspEnvelope, LspError> {
    let mut weights = Vec::with_capacity(block_len);
    let mut max = 0.0f64;
    for &cell in mag2.iter().take(block_len) {
        let v = luts.lookup(cell.to_bits());
        weights.push(r32(v));
        if v > max {
            max = f64::from(r32(v));
        }
    }
    if block_len == 0 || max == 0.0 {
        return Err(LspError::ZeroMaximum);
    }
    Ok(LspEnvelope {
        weights,
        max: max as f32,
    })
}

/// The whole §3.1 conversion: ten wire indices → the `block_len`-bin
/// envelope evaluated on the `grid_len`-point grid (`grid_len` = the
/// block length when `flags2` bit 5 is set, else `frame_length`;
/// `block_len ≤ grid_len`).
///
/// # Errors
///
/// [`LspError::BadGeometry`] for an unsupported `(N, L)`,
/// [`LspError::IndexOutOfRange`] for an index outside its field,
/// [`LspError::ZeroMaximum`] when the decoder itself would fail the
/// block.
pub fn lsp_envelope(
    indices: &[u8; LSP_ORDER],
    block_len: usize,
    grid_len: usize,
) -> Result<LspEnvelope, LspError> {
    if !legal_grid(grid_len) || block_len > grid_len {
        return Err(LspError::BadGeometry {
            block_len,
            grid_len,
        });
    }
    for (position, &index) in indices.iter().enumerate() {
        let limit = 1u8 << crate::vendor_frame::LSP_INDEX_WIDTHS[position];
        if index >= limit {
            return Err(LspError::IndexOutOfRange { position, index });
        }
    }
    let (p, qd, _) = lsp_to_pq(indices);
    let neg_a = pq_to_neg_a(&p, &qd);
    let mag2 = evaluate_mag2(&neg_a, grid_len).ok_or(LspError::BadGeometry {
        block_len,
        grid_len,
    })?;
    envelope_from_mag2(&mag2, block_len, RootLuts::shared())
}

/// The §3.1 nearest-neighbour resampler (`.text 0x5c20`): a cached
/// envelope of `from.len()` bins reused by a block of `to` bins is
/// decimated (shorter block) or replicated (longer block) in place —
/// `out[i] = from[i · from.len() / to]`.
pub fn resample_envelope(from: &[f32], to: usize) -> Vec<f32> {
    if from.is_empty() || to == 0 {
        return vec![1.0; to];
    }
    (0..to).map(|i| from[i * from.len() / to]).collect()
}

/// The round-08 **formula** (binary64, not bit-exact): `|A(e^{jω})|²`
/// on `ω = iπ/L` for `i = 0..L`, from the same `−a_n`. Kept as the
/// reference the exact evaluator is sanity-checked against (the two
/// agree to `f32` accuracy relative to the spectral peak; deep
/// valleys are where the decoder's `f32` accumulation legitimately
/// departs from the formula).
pub fn reference_mag2(neg_a: &[f32; LSP_ORDER], l: usize) -> Vec<f64> {
    let mut a = [1.0f64; LSP_ORDER + 1];
    for (slot, &v) in a.iter_mut().skip(1).zip(neg_a.iter()) {
        *slot = -f64::from(v);
    }
    (0..l)
        .map(|i| {
            let w = i as f64 * std::f64::consts::PI / l as f64;
            let re: f64 = a
                .iter()
                .enumerate()
                .map(|(n, &an)| an * (n as f64 * w).cos())
                .sum();
            let im: f64 = -a
                .iter()
                .enumerate()
                .map(|(n, &an)| an * (n as f64 * w).sin())
                .sum::<f64>();
            re * re + im * im
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A small deterministic index generator (xorshift) for sweeps.
    fn random_indices(seed: &mut u64) -> [u8; LSP_ORDER] {
        let mut out = [0u8; LSP_ORDER];
        for (slot, &w) in out
            .iter_mut()
            .zip(crate::vendor_frame::LSP_INDEX_WIDTHS.iter())
        {
            *seed ^= *seed << 13;
            *seed ^= *seed >> 7;
            *seed ^= *seed << 17;
            *slot = (*seed % (1u64 << w)) as u8;
        }
        out
    }

    #[test]
    fn widened_constants_are_the_decoder_f32_values() {
        assert_eq!(SQRT2, f64::from(f32::from_bits(0x3fb5_04f3)));
        assert_eq!(SQRT1_2, f64::from(f32::from_bits(0x3f35_04f3)));
    }

    #[test]
    fn root_luts_match_the_staged_invariants() {
        let luts = RootLuts::build();
        assert_eq!(luts.mantissa[0], 1.0);
        assert!(luts.mantissa.windows(2).all(|w| w[0] > w[1]));
        assert_eq!(luts.exponent[127], 1.0);
        assert_eq!(luts.exponent[0], 0.0);
        assert_eq!(luts.exponent[255], 0.0);
        assert!(luts.exponent[1..255].windows(2).all(|w| w[0] > w[1]));
        // The staged closed forms: E[e+4] == E[e]/2, M[4095] ≈ 2^-0.25.
        for e in 1..251 {
            assert_eq!(luts.exponent[e + 4], luts.exponent[e] / 2.0);
        }
        let last = f64::from(luts.mantissa[4095]);
        assert!((last - 2f64.powf(-0.25)).abs() < 1e-4, "{last}");
        // The staged spot values (wma-lsp-root-lut-*.csv heads).
        let near = |x: f32, y: f64| (f64::from(x) - y).abs() <= y.abs() * 1e-7;
        assert!(near(luts.mantissa[1], 0.999_938_965));
        assert!(near(luts.exponent[1], 3.037_000_45e9));
    }

    #[test]
    fn codebook_spot_values_match_the_staged_csv() {
        // wma-lsp-codebook.csv rows 0..4 of row 0, and the documented
        // reachable-range rule (rows 0, 8, 9 use entries 0–7).
        let near = |x: f32, y: f64| (f64::from(x) - y).abs() <= 1e-7;
        assert!(near(codebook_entry(0, 0), -1.987_329_01));
        assert!(near(codebook_entry(0, 4), -1.950_384_02));
        for row in [0usize, 8, 9] {
            for idx in 8..16 {
                assert_eq!(codebook_entry(row, idx), 0.0, "row {row} idx {idx}");
            }
        }
        // Every reachable value lies strictly inside (−2, 2).
        for row in 0..LSP_ORDER {
            let reach = if matches!(row, 0 | 8 | 9) { 8 } else { 16 };
            for idx in 0..reach {
                let c = codebook_entry(row, idx);
                assert!(c > -2.0 && c < 2.0, "row {row} idx {idx}: {c}");
            }
        }
    }

    #[test]
    fn p_and_q_are_symmetric() {
        let mut seed = 0x1234_5678_9abc_def1u64;
        for _ in 0..200 {
            let idx = random_indices(&mut seed);
            let (p, _, q) = lsp_to_pq(&idx);
            assert_eq!(p.len(), 12);
            assert_eq!(q.len(), 11);
            for i in 0..12 {
                assert_eq!(p[i], p[11 - i]);
            }
            for i in 0..11 {
                assert_eq!(q[i], q[10 - i]);
            }
        }
    }

    #[test]
    fn exact_evaluator_tracks_the_formula_to_f32_accuracy() {
        // The staged model's own self-check: the exact dataflow agrees
        // with the binary64 formula to within a few f32 ulps of the spectral peak,
        // and within 1e-4 relative on bins at or above 1e-3 · peak.
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut worst_peak = 0.0f64;
        let mut worst_rel = 0.0f64;
        for trial in 0..60 {
            let idx = random_indices(&mut seed);
            let (p, qd, _) = lsp_to_pq(&idx);
            let neg_a = pq_to_neg_a(&p, &qd);
            let l = [128usize, 256, 512, 1024, 2048][trial % 5];
            let exact = evaluate_mag2(&neg_a, l).unwrap();
            let formula = reference_mag2(&neg_a, l);
            let peak = formula.iter().cloned().fold(0.0, f64::max);
            for i in 0..l {
                let err = (f64::from(exact[i]) - formula[i]).abs();
                worst_peak = worst_peak.max(err / peak);
                if formula[i] >= 1e-3 * peak {
                    worst_rel = worst_rel.max(err / formula[i]);
                }
            }
        }
        assert!(worst_peak < 4e-6, "peak-relative error {worst_peak:e}");
        assert!(worst_rel < 1e-4, "relative error {worst_rel:e}");
    }

    #[test]
    fn envelope_maximum_is_the_maximum_of_the_stored_bins() {
        let mut seed = 42u64;
        for trial in 0..40 {
            let idx = random_indices(&mut seed);
            let l = [16usize, 32, 64, 128, 256, 512, 1024, 2048][trial % 8];
            let n = l >> (trial % 3).min(l.trailing_zeros() as usize);
            let env = lsp_envelope(&idx, n, l).unwrap();
            assert_eq!(env.weights.len(), n);
            let mx = env.weights.iter().cloned().fold(0.0f32, f32::max);
            assert_eq!(env.max, mx);
            assert!(env.max > 0.0);
        }
    }

    #[test]
    fn geometry_and_index_errors_are_typed() {
        let idx = [0u8; LSP_ORDER];
        assert_eq!(
            lsp_envelope(&idx, 512, 96),
            Err(LspError::BadGeometry {
                block_len: 512,
                grid_len: 96
            })
        );
        assert_eq!(
            lsp_envelope(&idx, 1024, 512),
            Err(LspError::BadGeometry {
                block_len: 1024,
                grid_len: 512
            })
        );
        let mut bad = idx;
        bad[0] = 8;
        assert_eq!(
            lsp_envelope(&bad, 512, 512),
            Err(LspError::IndexOutOfRange {
                position: 0,
                index: 8
            })
        );
        assert!(evaluate_mag2(&[0.0; LSP_ORDER], 4096).is_none());
    }

    #[test]
    fn grid_scaling_uses_the_low_bins_of_the_longer_grid() {
        // N < L: the envelope is the first N bins of the L-grid
        // evaluation (the flags2 bit 5 = 0 form), which differs from
        // the N-grid evaluation of the same indices.
        let idx = [3u8, 5, 7, 9, 4, 6, 2, 8, 1, 2];
        let long = lsp_envelope(&idx, 1024, 1024).unwrap();
        let low = lsp_envelope(&idx, 512, 1024).unwrap();
        assert_eq!(&long.weights[..512], &low.weights[..]);
        let own = lsp_envelope(&idx, 512, 512).unwrap();
        assert_ne!(own.weights, low.weights);
    }

    #[test]
    fn resampler_decimates_and_replicates() {
        let from: Vec<f32> = (0..8).map(|i| i as f32).collect();
        assert_eq!(resample_envelope(&from, 4), vec![0.0, 2.0, 4.0, 6.0]);
        assert_eq!(
            resample_envelope(&from[..4], 8),
            vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0]
        );
        assert_eq!(resample_envelope(&[], 3), vec![1.0; 3]);
    }
}

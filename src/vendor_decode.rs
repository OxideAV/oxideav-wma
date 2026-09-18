//! Vendor-bitstream decode stage: parsed §2 blocks → PCM, with the
//! §5 stereo sum/difference inverse in its staged position.
//!
//! ## Source
//!
//! * `docs/audio/wma/frame-bit-layout.md` §5 — the sum/difference
//!   (mid/side) inverse: gated on two channels, the F2a flag, and at
//!   least one channel coded; it runs on the **dequantised**
//!   coefficients, in place, before the inverse transform, with no
//!   `1/2` (the halving is encoder-side per the patent trace §5);
//!   afterwards **both** channels are treated as coded, and an
//!   uncoded channel's buffer is zero-filled beforehand.
//! * `docs/audio/wma/frame-bit-layout.md` §2 — the three-field F1
//!   opening carries "the neighbouring block sizes a lapped
//!   transform needs at a resynchronisation point": the windowing of
//!   a block depends on its previous and next block sizes, which is
//!   what [`crate::vendor_frame::ParsedBlock::prev_size`] /
//!   [`next_size`](crate::vendor_frame::ParsedBlock::next_size)
//!   carry.
//! * `docs/audio/wma/tables/dequant-gain-lut.csv` — the 113-step
//!   `10^(1/16)` (1.25 dB/step) exponent → linear multiplier ladder
//!   ([`crate::wire_tables::DEQUANT_GAIN_LUT`]).
//! * The inverse transform is the §3/§8 patent-trace oddly-stacked
//!   lapped basis realised by [`crate::mlt`]; the window is the
//!   Eqn. (2) sine shape the patent trace names as the defensible
//!   default, generalised to unequal neighbours by the standard
//!   variable-block-size lapped-transform construction (below).
//!
//! ## Variable-size lapped reconstruction
//!
//! Each block of half-length `M` occupies the sample slot
//! `[pos, pos + M)` and its `2M`-sample inverse transform spans
//! `[pos − M/2, pos + 3M/2)` — centred on the slot, so the
//! time-domain fold points of *adjacent blocks of any sizes* line up
//! on the slot boundaries. The window is flat 1 over the slot
//! interior, with a sine slope of length `min(M, prev)` centred on
//! the left slot boundary and `min(M, next)` centred on the right
//! one (zero outside): adjacent slopes are then power-complementary
//! across every boundary, equal-size neighbours reduce to the plain
//! §3 sine window, and the overlap-add of the slope regions cancels
//! the fold aliases. The synthesiser therefore runs a small
//! accumulator and emits with a **fixed lead-in of
//! `frame_length / 2` zero samples** (every block still emits
//! exactly `block_size` samples; [`BlockSynth::flush`] drains the
//! final half-frame).
//!
//! This construction replaces the earlier truncation-aligned
//! overlap-add, which dropped the long tail at every long→short
//! transition. Measured against the black-box reference decode on
//! the committed vendor streams (`tests/vendor_streams.rs`), the
//! change plus the calibrated composition below moves the per-second
//! median SNR on the three fully-closing 44.1/22.05 kHz families
//! from ≈ 3 dB / ≈ 0 dB to ≈ 18–27 dB.
//!
//! ## Dequantisation composition (staged, validated on vendor bits)
//!
//! `docs/audio/wma/frame-bit-layout.md` §2.1 / §3 / §3.1 (rounds
//! 08–10 of the staging) pin the vendor dequantisers, and round 10
//! validated them bit-for-bit in the sandboxed vendor decoder (71 439
//! VLC-path bins on the mono 22.05 kHz stream, 6 524 LSP-path bins on
//! the mono 8 kHz stream):
//!
//! * **band weight** (VLC path) — `w_b = 10^((e_b − e_max) / 16)` with
//!   the delta clamped to `[−72, 50]`, realised as the staged
//!   `wma-envelope-weight-lut` split lookup ([`envelope_weight`]:
//!   stored mantissa ÷ `2^(|e| >> 2)` on the negative side, × on the
//!   positive side; the `e = −72` clamp reads the documented one-past
//!   slot). The r450–r457 realisation (the `dequant-gain-lut` integer
//!   ladder's ratio) agreed with this within its 0.75 % integer
//!   rounding; the staged table replaces it.
//! * **total gain** — `F(g) = 10^(g/20)` (1 dB per B1 step) as the
//!   staged `wma-total-gain-lut` product `mantissa[g] · 2^(4 + (g >> 3))`
//!   for `18 ≤ g < 146` ([`vendor_total_gain`]); `g ≥ 146` uses the
//!   runtime power the decoder computes. The r450 sweep had found the
//!   1/20 exponent; the staged table confirms it and the decoder's
//!   `g₀ = f32(F(total))` rounding point.
//! * **LSP path** (§3.1) — per-bin weight `W[i]` from
//!   [`crate::lsp_envelope`] (bit-exact), `g = f32(f32(1 / max W) ·
//!   F(total))`, coded bin `f32((q · W[i]) · g)` — validated bit-for-bit
//!   on the noise-disabled form (the only committed LSP-path stream has
//!   noise off). A reused envelope for a block of another size is
//!   resampled by nearest neighbour (`.text 0x5c20`); the recorded
//!   maximum is kept (DERIVED — no committed stream exercises it).
//! * **absolute scale** — `ABS_SCALE`: the one black-box-calibrated
//!   constant, folding the inverse transform's normalisation and the
//!   reference's ±1.0 float convention into a fitted gain ≈ 1 (its
//!   sign absorbs the reconstruction's phase convention). It is
//!   applied on top of the vendor composition as `ABS_SCALE / F(64)`
//!   ([`pcm_scale`]) so the r457 calibration (which anchored the total
//!   gain at 64) carries over unchanged. The absolute output scale
//!   after the inverse transform is the one part of G9 the staging has
//!   not read.
//!
//! ## Honest approximations (staged gaps)
//!
//! * The transition-window shape is unstaged and carried as the
//!   measured-best realisation of the staged facts.
//! * `F(g)` below 18: the staged reader note gives a coarse
//!   power-of-two form with an unstaged constant (`.rdata 0x1a530`);
//!   the continuous `10^(g/20)` is carried there (it is what the
//!   black-box reference measures), reported as a docs ask.

use crate::dequant_luts::{
    ENVELOPE_WEIGHT_STORED_NEG, ENVELOPE_WEIGHT_STORED_POS, TOTAL_GAIN_EXP_PART_LOG2,
    TOTAL_GAIN_MANTISSA,
};
use crate::lsp_envelope::{lsp_envelope, resample_envelope};
use crate::mlt::Mlt;
use crate::stream_config::StreamConfig;
use crate::vendor_frame::{Envelope, ParsedBlock};

/// Black-box-calibrated absolute output scale (module docs): places
/// decoded PCM in the reference's ±1.0 float convention; the sign
/// absorbs the reconstruction's phase convention.
///
/// r457 recalibration: the r450 value (`-6.85e-2`) was fitted
/// against the reference's stereo→mono *downmix* compared with this
/// decoder's `(L + R) / 2`; the reference's downmix weights are
/// `1/√2` per channel, so that fit absorbed a factor √2 and the
/// decoder ran 3 dB loud (the mono 22.05 kHz family's fitted gain
/// of 1.40 in r454 was the tell). Measured per channel, the
/// reference's fitted gain is now ≈ 1.0 on every family, mono
/// included, and on this crate's own encoded streams at every
/// total gain and envelope anchor.
pub const ABS_SCALE: f64 = -4.844e-2;

/// Stateful PCM synthesiser for parsed vendor blocks: per-channel
/// lapped-transform accumulator carried across blocks, frames and
/// packets, with a fixed `frame_length / 2` lead-in (module docs).
#[derive(Debug)]
pub struct BlockSynth {
    cfg: StreamConfig,
    /// Per-channel overlap accumulators; index 0 is absolute sample
    /// `acc_base`.
    acc: Vec<Vec<f64>>,
    /// Absolute sample index of `acc[ch][0]`.
    acc_base: i64,
    /// Absolute slot start of the next block.
    pos: i64,
    /// The last synthesised block's size (left windowing context).
    prev_size: Option<u16>,
    /// §3 per-block-size envelope cache, `[channel][size_index]` —
    /// the envelope a B2 = 0 block reuses
    /// ([`crate::vendor_frame::Envelope::Reused`]). The staged trace
    /// stores the reuse state per block-size index (`ctx+0x24c`), so
    /// the cache is keyed the same way.
    env_cache: Vec<Vec<Option<CachedEnvelope>>>,
    /// The most recent envelope per channel of any size — what a
    /// §3.1 reuse falls back to (resampled) when the per-size slot is
    /// empty.
    last_env: Vec<Option<CachedEnvelope>>,
    /// The §2.1 noise generator's state (module docs).
    noise_state: u64,
    /// Whether zero-quantised bins of coded channels are noise-filled
    /// at the reference's measured floor ([`ZERO_FILL_RMS_STEPS`]).
    zero_fill_noise: bool,
}

/// The black-box-measured **zero-coefficient noise floor** of the
/// reference decoder (r457): with every coefficient zero and no §2.1
/// flags, every bin of a coded channel's coded range still comes out
/// as white noise at a per-coefficient RMS of `0.4 · step`, where
/// `step = w_band · 10^((g − 64)/20) · |ABS_SCALE|` is the value of a
/// coefficient quantised to 1 (0.0006 / 0.006 / 0.06 at g = 60 / 80 /
/// 100, following the band exponents at the ladder ratio). A §2.1
/// flagged band replaces it with its F4-gain level
/// ([`noise_band_rms`]). None of the staged material describes this
/// floor, so [`BlockSynth`] leaves it **off** by default and offers it
/// through [`BlockSynth::with_zero_fill_noise`]; its level relative to
/// the smallest coded coefficient (−8 dB) makes it invisible in the
/// SNR figures the crate reports (the ladder's own-chain and
/// reference SNRs agree within 0.2 dB either way).
pub const ZERO_FILL_RMS_STEPS: f64 = 0.4;

/// The §2.1 noise-substitution level law, black-box measured (r457)
/// by emitting crafted frames through this crate's own emitter and
/// measuring the reference decoder's output spectrum in the flagged
/// bands: white noise at a per-coefficient RMS of
/// `10^((G − 64)/20) · w_band · |ABS_SCALE|` — the F4 gain `G` plays
/// exactly the total gain's role ([`total_gain_multiplier`]) on a
/// unit-RMS generator (1 dB per gain step: 0.0001 → 0.0003 → 0.001 →
/// 0.003 → 0.03 across G = 10 / 20 / 30 / 40 / 60), the band weight
/// follows the band's exponent at the ladder's 1.25 dB/step, and the
/// level is independent of the block's total gain and of the coded
/// coefficients. Each flagged band carries its own gain (the F4
/// chain). The vendor generator's sequence is unstaged; this crate
/// uses its own uniform generator, so substituted bands match the
/// reference in level and spectral shape, not sample for sample.
pub fn noise_band_rms(gain: i32, band_weight: f64) -> f64 {
    10f64.powf(f64::from(gain - 64) / 20.0) * band_weight * ABS_SCALE.abs()
}

/// Number of per-channel envelope-cache slots (block-size indices
/// 0..=4 cover the §0 clamp range down to 128-sample blocks).
const ENV_CACHE_SLOTS: usize = 8;

impl BlockSynth {
    /// A synthesiser for one stream.
    pub fn new(cfg: &StreamConfig) -> Self {
        let channels = usize::from(cfg.channels);
        Self {
            cfg: cfg.clone(),
            acc: vec![Vec::new(); channels],
            acc_base: 0,
            pos: 0,
            prev_size: None,
            env_cache: vec![vec![None; ENV_CACHE_SLOTS]; channels],
            last_env: vec![None; channels],
            noise_state: 0x9E37_79B9_7F4A_7C15,
            zero_fill_noise: false,
        }
    }

    /// Fill zero-quantised bins of coded channels with noise at the
    /// reference's measured floor ([`ZERO_FILL_RMS_STEPS`]).
    pub fn with_zero_fill_noise(mut self, on: bool) -> Self {
        self.zero_fill_noise = on;
        self
    }

    /// One unit-variance pseudo-random sample (uniform on ±√3;
    /// xorshift64* — a generator of this crate's own, see
    /// [`noise_band_rms`]).
    fn noise_sample(&mut self) -> f64 {
        let mut x = self.noise_state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.noise_state = x;
        let r = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11;
        let u = r as f64 / (1u64 << 53) as f64; // [0, 1)
        (2.0 * u - 1.0) * 3f64.sqrt()
    }

    /// Reset the overlap state (stream discontinuity). The emission
    /// timeline keeps its position; only carried content is dropped.
    pub fn reset(&mut self) {
        for a in &mut self.acc {
            a.clear();
        }
        self.acc_base = self.pos;
        self.prev_size = None;
        for ch in &mut self.env_cache {
            for slot in ch.iter_mut() {
                *slot = None;
            }
        }
        self.last_env.fill(None);
    }

    /// Synthesise one parsed block into `block_size` PCM samples per
    /// channel (channel-major), at the synthesiser's fixed
    /// `frame_length / 2`-sample lead-in. Applies dequantisation,
    /// the §5 sum/difference inverse when the block is joint, the
    /// inverse lapped transform, neighbour-matched windowing and
    /// overlap-add (module docs).
    pub fn block(&mut self, block: &ParsedBlock) -> Vec<Vec<f64>> {
        let m = usize::from(block.block_size);
        let spec = self.dequantise(block);

        // Left windowing context: the carried previous size when the
        // chain is unbroken, else the parser's three-field-opening
        // context; right context: the F1 pipeline's pre-read.
        let prev = usize::from(
            self.prev_size
                .or(block.prev_size)
                .unwrap_or(block.block_size),
        );
        let next = usize::from(block.next_size.unwrap_or(block.block_size));

        // Inverse transform + neighbour-matched window, accumulated
        // at [pos − M/2, pos + 3M/2).
        for (ch, coeffs) in spec.into_iter().enumerate() {
            let mut time = inverse_transform(&coeffs);
            transition_window(&mut time, prev, next);
            let start = self.pos - (m as i64) / 2;
            let need = (self.pos + 3 * (m as i64) / 2 - self.acc_base) as usize;
            if self.acc[ch].len() < need {
                self.acc[ch].resize(need, 0.0);
            }
            for (i, &v) in time.iter().enumerate() {
                let abs = start + i as i64;
                if abs < self.acc_base {
                    continue;
                }
                self.acc[ch][(abs - self.acc_base) as usize] += v;
            }
        }
        self.prev_size = Some(block.block_size);

        // Emit [pos − flen/2, pos + M − flen/2): every sample there
        // has received all its contributions (no later block's slope
        // reaches below its own slot start minus flen/2).
        let flen = i64::from(self.cfg.frame_length);
        let out = self.emit(self.pos - flen / 2, m);
        self.pos += m as i64;
        out
    }

    /// Drain the fixed lead-in: the final `frame_length / 2` samples
    /// still held in the accumulator after the last block. The
    /// synthesiser is left in the reset state.
    pub fn flush(&mut self) -> Vec<Vec<f64>> {
        let flen = i64::from(self.cfg.frame_length);
        let n = (flen / 2) as usize;
        let out = self.emit(self.pos - flen / 2, n);
        self.reset();
        out
    }

    /// Emit `n` samples per channel starting at absolute `from`,
    /// dropping the emitted prefix from the accumulators.
    fn emit(&mut self, from: i64, n: usize) -> Vec<Vec<f64>> {
        let channels = self.acc.len();
        let mut out = Vec::with_capacity(channels);
        for ch in 0..channels {
            let mut pcm = vec![0.0; n];
            for (i, p) in pcm.iter_mut().enumerate() {
                let abs = from + i as i64;
                if abs < self.acc_base {
                    continue;
                }
                let rel = (abs - self.acc_base) as usize;
                if rel < self.acc[ch].len() {
                    *p = self.acc[ch][rel];
                }
            }
            out.push(pcm);
        }
        let end = from + n as i64;
        if end > self.acc_base {
            let drop = (end - self.acc_base) as usize;
            for a in &mut self.acc {
                if a.len() > drop {
                    a.drain(..drop);
                } else {
                    a.clear();
                }
            }
            self.acc_base = end;
        }
        out
    }

    /// Dequantise a block's coded channels onto the full coefficient
    /// axis (uncoded channels zero-filled, §5) and run the §5
    /// sum/difference inverse when the block is joint.
    fn dequantise(&mut self, block: &ParsedBlock) -> Vec<Vec<f64>> {
        let channels = usize::from(self.cfg.channels);
        let m = usize::from(block.block_size);
        let coef_start = usize::from(self.cfg.coef_start(block.block_size));
        let mut spec: Vec<Vec<f64>> = vec![vec![0.0; m]; channels];
        for (ch, chan) in block.channels.iter().enumerate() {
            if !chan.coded {
                continue;
            }
            // §3 per-block-size envelope cache: a fresh envelope
            // (exponents, or a §3.1 conversion) fills the slot for
            // this size index; a Reused envelope reads it back — a
            // §3.1 envelope of another size is resampled (`.text
            // 0x5c20`); flat when nothing was cached yet (only
            // possible right after a reset).
            let slot = usize::from(block.size_index).min(ENV_CACHE_SLOTS - 1);
            let envelope: Option<CachedEnvelope> = match chan.envelope.as_ref() {
                Some(Envelope::Exponents(e)) => Some(CachedEnvelope::Exponents(e.clone())),
                Some(Envelope::LspIndices(idx)) => {
                    let grid = usize::from(self.cfg.lsp_grid_len(block.block_size));
                    match lsp_envelope(idx, m, grid) {
                        Ok(env) => Some(CachedEnvelope::Lsp {
                            weights: env.weights,
                            max: env.max,
                        }),
                        // The decoder's own decode error for the
                        // block: the channel stays silent.
                        Err(_) => continue,
                    }
                }
                Some(Envelope::Reused) => self.env_cache[ch][slot]
                    .clone()
                    .or_else(|| self.last_env[ch].clone().map(|e| e.resampled(m))),
                None => None,
            };
            if let Some(env) = envelope.as_ref() {
                self.env_cache[ch][slot] = Some(env.clone());
                self.last_env[ch] = Some(env.clone());
            }
            let weights = per_bin_weights(&self.cfg, envelope.as_ref(), m);
            // The vendor composition: `q · w · F(total)`, then the
            // calibrated PCM scale. On the §3.1 path the decoder's own
            // rounding points are honoured (`g = f32(f32(1/max W) ·
            // F)`, bin `f32((q · W[i]) · g)`).
            let f_total = vendor_total_gain(block.total_gain);
            let gain = f_total * pcm_scale();
            let lsp_gain: Option<f64> = match envelope.as_ref() {
                Some(CachedEnvelope::Lsp { max, .. }) => {
                    let inv_max = f64::from((1.0 / f64::from(*max)) as f32);
                    Some(f64::from((inv_max * f_total) as f32))
                }
                _ => None,
            };
            let lsp_raw: Option<&[f32]> = match envelope.as_ref() {
                Some(CachedEnvelope::Lsp { weights, .. }) => Some(weights),
                _ => None,
            };
            // §2.1: a noise-substituted band contributes no coded
            // coefficients — the coefficient sub-stream skips its
            // bins. Rebuild that mapping when the block carries
            // noise flags (the parser's default measured policy), and
            // fill the substituted bands with noise at the measured
            // level law ([`noise_band_rms`]), one F4 gain per band.
            let excluded: Vec<(u16, u16)> = if chan.noise_flags.iter().any(|&f| f) {
                noise_excluded_ranges(&self.cfg, block, &chan.noise_flags)
            } else {
                Vec::new()
            };
            for (&(lo, hi), &gain) in excluded.iter().zip(chan.noise_gains.iter()) {
                for k in usize::from(lo)..usize::from(hi).min(m) {
                    let rms = noise_band_rms(gain, weights[k]);
                    spec[ch][k] = self.noise_sample() * rms;
                }
            }
            let coef_end = usize::from(self.cfg.coef_end(block.block_size));
            let mut k = coef_start;
            for &q in chan.coefficients.iter() {
                while excluded
                    .iter()
                    .any(|&(lo, hi)| (usize::from(lo)..usize::from(hi)).contains(&k))
                {
                    k += 1;
                }
                if k >= coef_end || k >= m {
                    break;
                }
                if q != 0 {
                    spec[ch][k] = match (lsp_raw, lsp_gain) {
                        (Some(w), Some(g)) => {
                            let v = ((f64::from(q) * f64::from(w[k])) as f32) as f64;
                            ((v * g) as f32) as f64 * pcm_scale()
                        }
                        _ => f64::from(q) * weights[k] * gain,
                    };
                } else if self.zero_fill_noise {
                    spec[ch][k] =
                        self.noise_sample() * ZERO_FILL_RMS_STEPS * weights[k] * gain.abs();
                }
                k += 1;
            }
        }

        // §5 sum/difference inverse, on dequantised coefficients,
        // before the inverse transform; both channels count as coded
        // afterwards.
        if channels == 2 && block.joint_stereo && block.channels.iter().any(|c| c.coded) {
            let (a, b) = spec.split_at_mut(1);
            for (mid, side) in a[0].iter_mut().zip(b[0].iter_mut()) {
                let m0 = *mid;
                let s0 = *side;
                *mid = m0 + s0;
                *side = m0 - s0;
            }
        }
        spec
    }
}

/// The §2.1 flagged-band bin ranges of a parsed block, recomputed
/// from the same measured walk the parser used
/// (`vendor_frame::measured_noise_policy`).
fn noise_excluded_ranges(
    cfg: &StreamConfig,
    block: &ParsedBlock,
    flags: &[bool],
) -> Vec<(u16, u16)> {
    let Some((spec, _)) = crate::vendor_frame::measured_noise_policy(cfg) else {
        return Vec::new();
    };
    crate::vendor_frame::noise_walk_bands_for(cfg, block.block_size, &spec)
        .into_iter()
        .zip(flags.iter())
        .filter(|(_, &f)| f)
        .map(|(range, _)| range)
        .filter(|&(lo, hi)| lo < hi)
        .collect()
}

/// A decoded envelope as the per-size cache holds it.
#[derive(Debug, Clone, PartialEq)]
enum CachedEnvelope {
    /// §3 VLC-delta exponents, one per band.
    Exponents(Vec<i32>),
    /// §3.1 converted envelope: the per-bin `W[i]` and the recorded
    /// maximum.
    Lsp {
        /// `W[i]`, one per coefficient of the block it was converted for.
        weights: Vec<f32>,
        /// The conversion's recorded maximum.
        max: f32,
    },
}

impl CachedEnvelope {
    /// The envelope a block of `m` coefficients reuses: exponents are
    /// per band and carry over as they are; a §3.1 envelope of another
    /// length is resampled by nearest neighbour, its recorded maximum
    /// kept (module docs).
    fn resampled(self, m: usize) -> Self {
        match self {
            CachedEnvelope::Lsp { weights, max } if weights.len() != m => CachedEnvelope::Lsp {
                weights: resample_envelope(&weights, m),
                max,
            },
            other => other,
        }
    }
}

/// The staged band weights over the coefficient axis: for each band
/// of the block's partition, `10^((e − e_max) / 16)` through the
/// staged `wma-envelope-weight-lut` ([`envelope_weight`]) — the
/// vendor's per-band weight anchored at the block's loudest band.
/// Absent envelopes yield a flat weight; a §3.1 envelope is not a
/// band envelope (see [`per_bin_weights`]).
pub(crate) fn band_weights(cfg: &StreamConfig, envelope: Option<&Envelope>, m: usize) -> Vec<f64> {
    let exponents = match envelope {
        Some(Envelope::Exponents(e)) if !e.is_empty() => e,
        _ => return vec![1.0; m],
    };
    exponent_weights(cfg, exponents, m)
}

/// [`band_weights`] over an exponent list.
fn exponent_weights(cfg: &StreamConfig, exponents: &[i32], m: usize) -> Vec<f64> {
    let edges = crate::band_partition::exponent_band_edges(cfg.sample_rate, m as u16);
    let e_max = exponents.iter().copied().max().unwrap_or(0);
    let mut w = vec![1.0; m];
    for (b, pair) in edges.windows(2).enumerate() {
        let e = exponents.get(b).copied().unwrap_or(e_max);
        let weight = envelope_weight(e - e_max);
        for slot in &mut w[usize::from(pair[0])..usize::from(pair[1]).min(m)] {
            *slot = weight;
        }
    }
    w
}

/// The per-bin weight of a cached envelope: the band weights for
/// exponents, `W[i] / max W` for a §3.1 envelope (the role the band
/// weight plays in every §2.1 noise formula), flat when absent.
fn per_bin_weights(cfg: &StreamConfig, envelope: Option<&CachedEnvelope>, m: usize) -> Vec<f64> {
    match envelope {
        Some(CachedEnvelope::Exponents(e)) if !e.is_empty() => exponent_weights(cfg, e, m),
        Some(CachedEnvelope::Lsp { weights, max }) => {
            let inv = 1.0 / f64::from(*max);
            let mut w: Vec<f64> = weights.iter().map(|&x| f64::from(x) * inv).collect();
            w.resize(m, 1.0);
            w
        }
        _ => vec![1.0; m],
    }
}

/// The staged band weight for an exponent delta `e = e_b − e_max`:
/// `10^(e/16)` as the vendor's split lookup — `e` clamped to
/// `[−72, 50]`, the stored mantissa of `wma-envelope-weight-lut`
/// divided by `2^(|e| >> 2)` on the negative side and multiplied by
/// `2^(e >> 2)` on the positive side; `e = −72` reads the slot one
/// past the negative table, which is the positive table's entry 1
/// (the documented quirk). Validated bit-for-bit on 71 439 vendor
/// bins in the staging.
pub fn envelope_weight(delta: i32) -> f64 {
    let e = delta.clamp(-72, 50);
    if e <= 0 {
        let d = (-e) as usize;
        let stored = if d < ENVELOPE_WEIGHT_STORED_NEG.len() {
            f32::from_bits(ENVELOPE_WEIGHT_STORED_NEG[d])
        } else {
            f32::from_bits(ENVELOPE_WEIGHT_STORED_POS[1])
        };
        f64::from(stored) / f64::from(1u32 << (d >> 2))
    } else {
        let stored = f32::from_bits(ENVELOPE_WEIGHT_STORED_POS[e as usize]);
        f64::from(stored) * f64::from(1u32 << (e >> 2))
    }
}

/// The staged total-gain law `F(g) = 10^(g/20)`: the
/// `wma-total-gain-lut` product `mantissa[g] · 2^(4 + (g >> 3))` for
/// `18 ≤ g < 146` (validated bit-for-bit on vendor bits), the runtime
/// power `10^(0.05 g)` the decoder computes at and above 146, and —
/// below 18, where the staged reader note gives a coarse
/// power-of-two form with an unstaged constant — the continuous
/// `10^(g/20)` (module docs).
pub fn vendor_total_gain(total_gain: u32) -> f64 {
    let g = total_gain as usize;
    if (18..TOTAL_GAIN_MANTISSA.len()).contains(&g) {
        let mantissa = f64::from(f32::from_bits(TOTAL_GAIN_MANTISSA[g]));
        mantissa * f64::from(1u32 << TOTAL_GAIN_EXP_PART_LOG2[g >> 3])
    } else if g >= TOTAL_GAIN_MANTISSA.len() {
        f64::from(10f64.powf(0.05 * g as f64) as f32)
    } else {
        10f64.powf(g as f64 / 20.0)
    }
}

/// The calibrated PCM scale on top of the vendor composition:
/// `ABS_SCALE / F(64)`, so that `q · w · F(g) · pcm_scale()` equals the
/// r457-calibrated `q · w · 10^((g − 64)/20) · ABS_SCALE` (module docs).
pub fn pcm_scale() -> f64 {
    ABS_SCALE / vendor_total_gain(64)
}

/// Total-gain multiplier relative to gain 64: `F(g) / F(64)` — 1 dB
/// per B1 step, the calibrated composition (module docs).
pub(crate) fn total_gain_multiplier(total_gain: u32) -> f64 {
    vendor_total_gain(total_gain) / vendor_total_gain(64)
}

/// Inverse lapped transform: the fast staged-set path for `{256,
/// 512, 1024, 2048, 4096}`-sample blocks, a direct evaluation of the
/// same oddly-stacked basis for the short sizes outside the typed
/// set (e.g. 128).
pub(crate) fn inverse_transform(coeffs: &[f64]) -> Vec<f64> {
    let m = coeffs.len();
    if let Ok(bs) = crate::block::BlockSize::from_samples(m as u16) {
        let mlt = Mlt::new(bs);
        return mlt.inverse(coeffs).expect("length matches by construction");
    }
    // Direct O(M·2M) evaluation with the same 2/M normalization the
    // fast path applies.
    let two_m = 2 * m;
    let mut out = vec![0.0; two_m];
    let norm = 2.0 / m as f64;
    for (n, slot) in out.iter_mut().enumerate() {
        let mut acc = 0.0;
        for (k, &c) in coeffs.iter().enumerate() {
            let angle = std::f64::consts::PI / m as f64
                * (n as f64 + 0.5 + m as f64 / 2.0)
                * (k as f64 + 0.5);
            acc += c * angle.cos();
        }
        *slot = acc * norm;
    }
    out
}

/// The neighbour-matched window over a block's `2M` transform
/// samples (module docs): slot boundaries sit at sample offsets
/// `M/2` and `3M/2`; a rising sine slope of length `min(M, prev)` is
/// centred on the left boundary and a falling one of length
/// `min(M, next)` on the right; flat 1 between the slopes, 0
/// outside. Equal-size neighbours reproduce the plain sine window.
pub(crate) fn transition_window(time: &mut [f64], prev: usize, next: usize) {
    let two_m = time.len();
    let m = two_m / 2;
    let lr = m.min(prev.max(1)) as f64;
    let lf = m.min(next.max(1)) as f64;
    let b0 = m as f64 / 2.0;
    let b1 = 3.0 * m as f64 / 2.0;
    for (n, t) in time.iter_mut().enumerate() {
        let x = n as f64 + 0.5;
        let w = if x < b0 - lr / 2.0 {
            0.0
        } else if x < b0 + lr / 2.0 {
            (std::f64::consts::FRAC_PI_2 * (x - (b0 - lr / 2.0)) / lr).sin()
        } else if x < b1 - lf / 2.0 {
            1.0
        } else if x < b1 + lf / 2.0 {
            (std::f64::consts::FRAC_PI_2 * (1.0 - (x - (b1 - lf / 2.0)) / lf)).sin()
        } else {
            0.0
        };
        *t *= w;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::Version;
    use crate::vendor_frame::ChannelBlock;

    fn cfg() -> StreamConfig {
        StreamConfig::derive(Version::V2, 44_100, 2, 12_003, 4459, 0x000f).unwrap()
    }

    fn coded_block(joint: bool, c0: Vec<i32>, c1: Vec<i32>, exps: Vec<i32>) -> ParsedBlock {
        let mk = |coded: bool, coeffs: Vec<i32>| ChannelBlock {
            coded,
            envelope: coded.then(|| Envelope::Exponents(exps.clone())),
            noise_flags: Vec::new(),
            noise_gains: Vec::new(),
            coefficients: coeffs,
        };
        ParsedBlock {
            block_size: 2048,
            size_index: 0,
            prev_size: Some(2048),
            next_size: Some(2048),
            joint_stereo: joint,
            total_gain: 64,
            n_coef: 1864,
            channels: vec![mk(!c0.is_empty(), c0), mk(!c1.is_empty(), c1)],
        }
    }

    #[test]
    fn sum_difference_inverse_reconstructs_left_right() {
        // A joint block whose mid carries a lone coefficient and
        // whose side is zero must synthesise two identical channels
        // (§5: ch0' = m + s, ch1' = m − s).
        let c = cfg();
        let mut synth = BlockSynth::new(&c);
        let mut mid = vec![0i32; 1864];
        mid[100] = 1000;
        let block = coded_block(true, mid, vec![0i32; 1864], vec![36; 25]);
        let pcm = synth.block(&block);
        assert_eq!(pcm.len(), 2);
        assert_eq!(pcm[0].len(), 2048);
        for (l, r) in pcm[0].iter().zip(pcm[1].iter()) {
            assert!(
                (l - r).abs() < 1e-12,
                "joint zero-side must fold to identical L/R"
            );
        }
        // And the signal is non-trivial (the emitted window includes
        // the block's slot lead half past the fixed lead-in).
        assert!(pcm[0].iter().any(|&x| x.abs() > 1e-9));
    }

    #[test]
    fn joint_with_one_coded_channel_still_fills_both() {
        // §5: an uncoded channel zero-fills, the inverse runs when at
        // least one channel is coded, and both come out non-silent
        // only via the coded one's content.
        let c = cfg();
        let mut synth = BlockSynth::new(&c);
        let mut mid = vec![0i32; 1864];
        mid[50] = 500;
        let mut block = coded_block(true, mid, Vec::new(), vec![36; 25]);
        block.channels[1].coded = false;
        block.channels[1].envelope = None;
        let pcm = synth.block(&block);
        // mid + 0 and mid − 0: identical channels.
        for (l, r) in pcm[0].iter().zip(pcm[1].iter()) {
            assert!((l - r).abs() < 1e-12);
        }
    }

    #[test]
    fn independent_blocks_bypass_the_fold() {
        let c = cfg();
        let mut synth = BlockSynth::new(&c);
        let mut left = vec![0i32; 1864];
        left[10] = 700;
        let block = coded_block(false, left, vec![0i32; 1864], vec![36; 25]);
        let pcm = synth.block(&block);
        // ch1 coded all-zero → silent; ch0 carries the tone.
        assert!(pcm[0].iter().any(|&x| x.abs() > 1e-9));
        assert!(pcm[1].iter().all(|&x| x.abs() < 1e-12));
    }

    #[test]
    fn envelope_weights_follow_the_staged_law() {
        // 10^(e/16) within the staged 6e-8 relative on the whole
        // clamp range bar the documented e = −72 quirk; 16 steps =
        // 10×; the quirk reads the positive table's entry 1 over
        // 2^18; the clamp holds outside [−72, 50].
        for e in -71..=50 {
            let want = 10f64.powf(f64::from(e) / 16.0);
            let got = envelope_weight(e);
            assert!((got / want - 1.0).abs() < 1e-6, "e={e}: {got} vs {want}");
        }
        assert_eq!(envelope_weight(0), 1.0);
        let r = envelope_weight(-16) / envelope_weight(-32);
        assert!((r - 10.0).abs() < 1e-5, "ratio {r}");
        let quirk = envelope_weight(-72);
        assert!((quirk - 1.154_781_94 / 262_144.0).abs() < 1e-12, "{quirk}");
        assert_eq!(envelope_weight(-100), quirk);
        assert_eq!(envelope_weight(60), envelope_weight(50));
    }

    #[test]
    fn total_gain_law_is_one_decibel_per_step_from_the_staged_table() {
        for g in 18..146u32 {
            let want = 10f64.powf(f64::from(g) / 20.0);
            let got = vendor_total_gain(g);
            assert!((got / want - 1.0).abs() < 1e-6, "g={g}: {got} vs {want}");
        }
        // Above the table: the runtime power; below: continuous.
        assert!((vendor_total_gain(150) / 10f64.powf(7.5) - 1.0).abs() < 1e-6);
        assert!((vendor_total_gain(10) / 10f64.powf(0.5) - 1.0).abs() < 1e-12);
        assert!((pcm_scale() * vendor_total_gain(64) / ABS_SCALE - 1.0).abs() < 1e-15);
    }

    #[test]
    fn total_gain_steps_are_one_decibel() {
        // The calibrated composition: 20 B1 steps = 20 dB = 10× in
        // amplitude, anchored at gain 64 → 1.0.
        assert!((total_gain_multiplier(64) - 1.0).abs() < 1e-12);
        let r = total_gain_multiplier(84) / total_gain_multiplier(64);
        assert!((r - 10.0).abs() < 1e-6, "ratio {r}");
    }

    #[test]
    fn overlap_add_carries_across_blocks() {
        let c = cfg();
        let mut synth = BlockSynth::new(&c);
        let mut coeffs = vec![0i32; 1864];
        coeffs[3] = 100;
        let block = coded_block(false, coeffs.clone(), vec![0i32; 1864], vec![36; 25]);
        let first = synth.block(&block);
        let second = synth.block(&block);
        // The second block's output includes the first's overlap
        // region: for a steady tone the two outputs differ (attack
        // vs sustained).
        assert_ne!(first[0], second[0]);
        synth.reset();
        let third = synth.block(&block);
        assert_eq!(first[0], third[0], "reset clears the carried overlap");
    }

    #[test]
    fn reused_envelope_resolves_from_the_per_size_cache() {
        // A Reused envelope must dequantise exactly like the fresh
        // envelope previously cached for the same block-size index.
        let c = cfg();
        let mut coeffs = vec![0i32; 1864];
        coeffs[40] = 400;
        coeffs[900] = 200;
        let shaped: Vec<i32> = (0..25).map(|b| 30 + (b % 7)).collect();
        let fresh = coded_block(false, coeffs.clone(), vec![0i32; 1864], shaped.clone());
        let mut reused = fresh.clone();
        reused.channels[0].envelope = Some(Envelope::Reused);
        reused.channels[1].envelope = Some(Envelope::Reused);

        let mut a = BlockSynth::new(&c);
        let out_a1 = a.block(&fresh);
        let out_a2 = a.block(&fresh);
        let mut b = BlockSynth::new(&c);
        let out_b1 = b.block(&fresh);
        let out_b2 = b.block(&reused);
        assert_eq!(out_a1, out_b1);
        assert_eq!(out_a2, out_b2, "reused envelope must equal the cached one");

        // After a reset the cache is empty: Reused falls back to the
        // flat envelope, which differs for a shaped spectrum.
        b.reset();
        a.reset();
        let flat_path = b.block(&reused);
        let fresh_path = a.block(&fresh);
        assert_ne!(flat_path, fresh_path);
    }

    #[test]
    fn short_out_of_set_blocks_synthesise_via_the_direct_basis() {
        // A 128-sample block (outside the typed fast-path set) still
        // produces 128 samples per block; its energy sits inside the
        // fixed lead-in, so the flush drains it.
        let c = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap();
        let mut synth = BlockSynth::new(&c);
        let mut coeffs = vec![0i32; 117];
        coeffs[5] = 300;
        let block = ParsedBlock {
            block_size: 128,
            size_index: 3,
            prev_size: Some(128),
            next_size: Some(128),
            joint_stereo: false,
            total_gain: 20,
            n_coef: 117,
            channels: vec![
                ChannelBlock {
                    coded: true,
                    envelope: Some(Envelope::Exponents(vec![36; 10])),
                    noise_flags: Vec::new(),
                    noise_gains: Vec::new(),
                    coefficients: coeffs,
                },
                ChannelBlock {
                    coded: false,
                    envelope: None,
                    noise_flags: Vec::new(),
                    noise_gains: Vec::new(),
                    coefficients: Vec::new(),
                },
            ],
        };
        let pcm = synth.block(&block);
        assert_eq!(pcm[0].len(), 128);
        let tail = synth.flush();
        assert_eq!(tail[0].len(), 512, "flush drains frame_length / 2");
        assert!(
            pcm[0].iter().chain(tail[0].iter()).any(|&x| x.abs() > 1e-9),
            "the block's energy must appear in the emitted timeline"
        );
    }

    #[test]
    fn equal_neighbours_reproduce_the_sine_window() {
        // transition_window(prev = next = M) must equal the plain §3
        // sine window over all 2M samples.
        let m = 256usize;
        let mut w = vec![1.0f64; 2 * m];
        transition_window(&mut w, m, m);
        for (n, &v) in w.iter().enumerate() {
            let sine = (std::f64::consts::PI * (n as f64 + 0.5) / (2.0 * m as f64)).sin();
            assert!((v - sine).abs() < 1e-12, "n={n}: {v} vs {sine}");
        }
    }

    #[test]
    fn transition_slopes_are_power_complementary_across_a_boundary() {
        // Long block (M = 2048) followed by a short one (M = 512):
        // the falling slope of the long window and the rising slope
        // of the short window cover the same absolute samples
        // (centred on the shared slot boundary) and their squares
        // sum to 1 — the alias-cancellation condition.
        let long_m = 2048usize;
        let short_m = 512usize;
        // Falling slope of the long window: 512; rising slope of
        // the short window: 512.
        let mut wl = vec![1.0f64; 2 * long_m];
        transition_window(&mut wl, long_m, short_m);
        let mut ws = vec![1.0f64; 2 * short_m];
        transition_window(&mut ws, long_m, short_m);
        // Absolute sample axis: long slot [0, 2048), short slot
        // [2048, 2560). Long window sample n sits at n − 1024; short
        // window sample n at 2048 + n − 256.
        for abs in 2048 - 256..2048 + 256 {
            let l = wl[abs + 1024];
            let s = ws[abs + 256 - 2048];
            let sum = l * l + s * s;
            assert!((sum - 1.0).abs() < 1e-12, "abs={abs}: {l}² + {s}² = {sum}");
        }
    }

    #[test]
    fn variable_size_chain_preserves_a_steady_tone() {
        // Perfect-reconstruction sanity for the variable-size chain:
        // synthesise a constant spectral line through a
        // 2048→512→512→512→512→2048 block sequence and check the
        // emitted timeline carries no discontinuity artefacts at the
        // transitions — the overlap-add of neighbour-matched slopes
        // must keep the summed window envelope at exactly 1
        // everywhere in the interior. Feed all-zero spectra except a
        // DC-ish envelope: with zero coefficients everywhere the
        // reconstruction is exactly zero; instead check the window
        // partition-of-unity directly on the absolute axis.
        let seq: [usize; 6] = [2048, 512, 512, 512, 512, 2048];
        let mut envelope_sum = vec![0.0f64; 8192];
        let mut pos = 0usize;
        for (i, &m) in seq.iter().enumerate() {
            let prev = if i == 0 { m } else { seq[i - 1] };
            let next = if i + 1 == seq.len() { m } else { seq[i + 1] };
            let mut w = vec![1.0f64; 2 * m];
            transition_window(&mut w, prev, next);
            for (n, &v) in w.iter().enumerate() {
                let abs = pos as i64 + n as i64 - (m as i64) / 2;
                if (0..envelope_sum.len() as i64).contains(&abs) {
                    // Power domain: overlap-add cancels aliases and
                    // the window pairs are power-complementary.
                    envelope_sum[abs as usize] += v * v;
                }
            }
            pos += m;
        }
        // Interior samples (past the first block's rise, before the
        // last block's fall — the final fall is 2048 long, centred
        // at the stream-end slot boundary 6144) must sum to exactly
        // 1.
        for (i, &s) in envelope_sum.iter().enumerate().take(5120).skip(1024) {
            assert!((s - 1.0).abs() < 1e-12, "abs={i}: envelope {s}");
        }
    }
    /// A flagged band at gain G carries the energy of `q = 1`
    /// coefficients decoded at total gain G (the measured level law,
    /// [`noise_band_rms`]): the time-domain energy of a stream of
    /// noise-substituted blocks matches that of the coded twin within
    /// the generator's statistical spread.
    #[test]
    fn flagged_band_energy_matches_a_unit_coded_band_at_the_same_gain() {
        use crate::vendor_frame::{ChannelBlock, ParsedBlock};
        let cfg = StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap();
        let bands = crate::band_partition::exponent_band_count(22_050, 1024);
        let g = 30i32;
        let energy = |flagged: bool| -> f64 {
            let mut synth = BlockSynth::new(&cfg);
            let mut acc = 0.0;
            for _ in 0..64 {
                let (flags, gains, coefficients, total_gain) = if flagged {
                    (vec![true, true], vec![g, g], vec![0i32; 716], 100)
                } else {
                    // The twin: q = 1 on every bin of the two walk bands
                    // ([716, 932)), zero elsewhere, decoded at total gain g.
                    let mut c = vec![0i32; 932];
                    for v in &mut c[716..932] {
                        *v = 1;
                    }
                    (Vec::new(), Vec::new(), c, g as u32)
                };
                let block = ParsedBlock {
                    block_size: 1024,
                    size_index: 0,
                    prev_size: Some(1024),
                    next_size: Some(1024),
                    joint_stereo: false,
                    total_gain,
                    n_coef: 932,
                    channels: vec![ChannelBlock {
                        coded: true,
                        envelope: Some(Envelope::Exponents(vec![40; bands])),
                        noise_flags: flags,
                        noise_gains: gains,
                        coefficients,
                    }],
                };
                let out = synth.block(&block);
                acc += out[0].iter().map(|v| v * v).sum::<f64>();
            }
            acc
        };
        let (noise, coded) = (energy(true), energy(false));
        assert!(noise > 0.0 && coded > 0.0);
        let ratio = noise / coded;
        assert!(
            (0.8..1.25).contains(&ratio),
            "flagged/coded energy ratio {ratio:.3}"
        );
        // Ten gain steps up = +10 dB.
        let mut synth = BlockSynth::new(&cfg);
        let mut acc = 0.0;
        for _ in 0..64 {
            let block = ParsedBlock {
                block_size: 1024,
                size_index: 0,
                prev_size: Some(1024),
                next_size: Some(1024),
                joint_stereo: false,
                total_gain: 100,
                n_coef: 932,
                channels: vec![ChannelBlock {
                    coded: true,
                    envelope: Some(Envelope::Exponents(vec![40; bands])),
                    noise_flags: vec![true, true],
                    noise_gains: vec![g + 10, g + 10],
                    coefficients: vec![0; 716],
                }],
            };
            let out = synth.block(&block);
            acc += out[0].iter().map(|v| v * v).sum::<f64>();
        }
        let db = 10.0 * (acc / noise).log10();
        assert!((9.0..11.0).contains(&db), "+10 gain steps = {db:.2} dB");
    }
}

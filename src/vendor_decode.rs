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
//! ## Dequantisation composition (calibrated)
//!
//! The staged docs pin the ladder's ratio but leave the composition
//! rule open ("Still open" in the layout doc). The composition
//! carried here was selected by sweeping candidate rules against the
//! black-box reference decode of the six committed vendor streams
//! (best-lag/best-gain fit, median per-second SNR as the score; the
//! sweep and its optimum are quoted in `tests/vendor_streams.rs`):
//!
//! * **band weight** — `10^((e − e_max) / 16)`: the staged ladder's
//!   own 1.25 dB/step ratio anchored at the block's maximum
//!   exponent (the §3 note that the decoder records the envelope's
//!   maximum, which "drives dequantization", is consistent with
//!   exactly this anchoring; a fixed anchor scores far worse on
//!   every stream);
//! * **total gain** — `10^((g − 64) / 20)`, i.e. exactly **1 dB per
//!   B1 step**: the sweep's optimum is sharply at the 1/20 exponent
//!   (1/16 and 1/32 both lose ≥ 9 dB), and it is corroborated by the
//!   staged escape-width table, whose level-field width drops ≈ 1
//!   bit per ≈ 10 gain steps (≈ 6 dB per bit);
//! * **absolute scale** — `ABS_SCALE`: a single black-box-calibrated
//!   constant (the fitted gain against the ±1.0-float reference
//!   converges to the same value on every fully-closing
//!   envelope-coded stream once the two rules above are in place),
//!   folded in so decoded PCM lands in the reference's ±1.0 float
//!   convention with a fitted gain ≈ 1. Its sign also absorbs the
//!   phase convention of the reconstruction relative to the
//!   reference waveform. The true closed-form constant remains a
//!   staged gap.
//!
//! ## Honest approximations (staged gaps)
//!
//! * The vendor's literal composition rule and transition-window
//!   shape are still unstaged; both are carried here as the
//!   measured-best realisation of the staged facts, not as staged
//!   facts themselves.
//! * **The §3.1 line-spectral envelope** conversion tables are not
//!   staged; LSP-path blocks decode with a flat envelope.

use crate::mlt::Mlt;
use crate::stream_config::StreamConfig;
use crate::vendor_frame::{Envelope, ParsedBlock};
use crate::wire_tables::DEQUANT_GAIN_LUT;

/// Black-box-calibrated absolute output scale (module docs): places
/// decoded PCM in the reference's ±1.0 float convention; the sign
/// absorbs the reconstruction's phase convention.
pub const ABS_SCALE: f64 = -6.85e-2;

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
    /// the exponents a B2 = 0 block reuses
    /// ([`crate::vendor_frame::Envelope::Reused`]). The staged trace
    /// stores the reuse state per block-size index (`ctx+0x24c`), so
    /// the cache is keyed the same way.
    env_cache: Vec<Vec<Option<Vec<i32>>>>,
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
        }
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
            // §3 per-block-size envelope cache: fresh exponents fill
            // the slot for this size index; a Reused envelope reads
            // it back (flat when nothing was cached yet — only
            // possible right after a reset).
            let slot = usize::from(block.size_index).min(ENV_CACHE_SLOTS - 1);
            let envelope = match chan.envelope.as_ref() {
                Some(Envelope::Exponents(e)) => {
                    self.env_cache[ch][slot] = Some(e.clone());
                    Some(Envelope::Exponents(e.clone()))
                }
                Some(Envelope::Reused) => self.env_cache[ch][slot]
                    .as_ref()
                    .map(|e| Envelope::Exponents(e.clone())),
                other => other.cloned(),
            };
            let weights = band_weights(&self.cfg, envelope.as_ref(), m);
            let gain = total_gain_multiplier(block.total_gain) * ABS_SCALE;
            for (i, &q) in chan.coefficients.iter().enumerate() {
                if q == 0 {
                    continue;
                }
                let k = coef_start + i;
                if k < m {
                    spec[ch][k] = f64::from(q) * weights[k] * gain;
                }
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

/// The staged-ladder band weights over the coefficient axis: for each
/// band of the block's partition, `10^((e − e_max) / 16)` — the
/// ladder's own 1.25 dB/step ratio anchored at the block's loudest
/// band (module docs). LSP-path and absent envelopes yield a flat
/// weight.
fn band_weights(cfg: &StreamConfig, envelope: Option<&Envelope>, m: usize) -> Vec<f64> {
    let exponents = match envelope {
        Some(Envelope::Exponents(e)) if !e.is_empty() => e,
        // §3.1 conversion tables unstaged / uncoded: flat envelope.
        _ => return vec![1.0; m],
    };
    let edges = crate::band_partition::exponent_band_edges(cfg.sample_rate, m as u16);
    let e_max = exponents.iter().copied().max().unwrap_or(0);
    let mut w = vec![1.0; m];
    for (b, pair) in edges.windows(2).enumerate() {
        let e = exponents.get(b).copied().unwrap_or(e_max);
        let weight = ladder_ratio(e, e_max);
        for slot in &mut w[usize::from(pair[0])..usize::from(pair[1]).min(m)] {
            *slot = weight;
        }
    }
    w
}

/// `ladder[a] / ladder[b]` on the staged dequant ladder, with the
/// ladder's own `10^(1/16)` ratio extended outside its 113 entries.
fn ladder_ratio(a: i32, b: i32) -> f64 {
    let idx = |e: i32| -> f64 {
        let clamped = e.clamp(0, (DEQUANT_GAIN_LUT.len() - 1) as i32);
        let base = f64::from(DEQUANT_GAIN_LUT[clamped as usize]);
        // Extend beyond the table at the staged step ratio.
        base * 10f64.powf(f64::from(e - clamped) / 16.0)
    };
    idx(a) / idx(b)
}

/// Total-gain multiplier: `10^((g − 64) / 20)` — 1 dB per B1 step,
/// the calibrated composition (module docs).
fn total_gain_multiplier(total_gain: u32) -> f64 {
    10f64.powf(f64::from(total_gain as i32 - 64) / 20.0)
}

/// Inverse lapped transform: the fast staged-set path for `{256,
/// 512, 1024, 2048, 4096}`-sample blocks, a direct evaluation of the
/// same oddly-stacked basis for the short sizes outside the typed
/// set (e.g. 128).
fn inverse_transform(coeffs: &[f64]) -> Vec<f64> {
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
fn transition_window(time: &mut [f64], prev: usize, next: usize) {
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
    fn envelope_weights_follow_the_staged_ladder_ratio() {
        // Two bands 16 steps apart weight 10× apart (the ladder is
        // 10^(1/16) per step).
        let r = ladder_ratio(52, 36) / ladder_ratio(36, 36);
        assert!((r - 10.0).abs() < 0.11, "ratio {r}");
    }

    #[test]
    fn total_gain_steps_are_one_decibel() {
        // The calibrated composition: 20 B1 steps = 20 dB = 10× in
        // amplitude, anchored at gain 64 → 1.0.
        assert!((total_gain_multiplier(64) - 1.0).abs() < 1e-12);
        let r = total_gain_multiplier(84) / total_gain_multiplier(64);
        assert!((r - 10.0).abs() < 1e-9, "ratio {r}");
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
}

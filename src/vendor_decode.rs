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
//! * `docs/audio/wma/tables/dequant-gain-lut.csv` — the 113-step
//!   `10^(1/16)` (1.25 dB/step) exponent → linear multiplier ladder
//!   ([`crate::wire_tables::DEQUANT_GAIN_LUT`]).
//! * The synthesis chain (inverse MLT → sine window → overlap-add)
//!   is the §3/§8 patent-trace pipeline realised by [`crate::mlt`] /
//!   [`crate::window`].
//!
//! ## Honest approximations (staged gaps)
//!
//! The staged docs pin the *parse* completely but leave two decode
//! semantics open, so this stage is quality-approximate until they
//! are staged:
//!
//! * **Dequantisation scaling** ("Still open" in the layout doc):
//!   this stage maps a band exponent `e` to the staged ladder value
//!   `10^(e/16)` relative to the block's maximum exponent, and folds
//!   the block's total gain in at the same 1.25 dB/step ratio — the
//!   ladder ratio is staged data, the composition rule is not.
//! * **Window transitions between unequal block sizes**: symmetric
//!   sine windows with truncation-aligned overlap-add; the vendor
//!   transition-window shape is unstaged.
//! * **The §3.1 line-spectral envelope** conversion tables are not
//!   staged; LSP-path blocks decode with a flat envelope.

use crate::block::BlockSize;
use crate::mlt::Mlt;
use crate::stream_config::StreamConfig;
use crate::vendor_frame::{Envelope, ParsedBlock};
use crate::window::Window;
use crate::wire_tables::DEQUANT_GAIN_LUT;

/// Stateful PCM synthesiser for parsed vendor blocks: per-channel
/// overlap-add carried across blocks, frames and packets.
#[derive(Debug)]
pub struct BlockSynth {
    cfg: StreamConfig,
    /// Per-channel overlap tails (second half of the previous
    /// windowed inverse transform).
    tails: Vec<Vec<f64>>,
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
            tails: vec![Vec::new(); channels],
            env_cache: vec![vec![None; ENV_CACHE_SLOTS]; channels],
        }
    }

    /// Reset the overlap state (stream discontinuity).
    pub fn reset(&mut self) {
        for t in &mut self.tails {
            t.clear();
        }
        for ch in &mut self.env_cache {
            for slot in ch.iter_mut() {
                *slot = None;
            }
        }
    }

    /// Synthesise one parsed block into `block_size` PCM samples per
    /// channel (channel-major). Applies dequantisation, the §5
    /// sum/difference inverse when the block is joint, the inverse
    /// MLT, sine windowing and overlap-add.
    pub fn block(&mut self, block: &ParsedBlock) -> Vec<Vec<f64>> {
        let channels = usize::from(self.cfg.channels);
        let m = usize::from(block.block_size);
        let coef_start = usize::from(self.cfg.coef_start(block.block_size));

        // Dequantise each coded channel onto the full coefficient
        // axis; uncoded channels are zero-filled (§5).
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
            let weights = band_weights(&self.cfg, block, envelope.as_ref(), m);
            let gain = total_gain_multiplier(block.total_gain);
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

        // Inverse transform + sine window + overlap-add per channel.
        let mut out = Vec::with_capacity(channels);
        for (ch, coeffs) in spec.into_iter().enumerate() {
            let time = inverse_transform(&coeffs);
            let windowed = apply_sine_window(time);
            let (pcm, tail) = overlap_add(std::mem::take(&mut self.tails[ch]), windowed, m);
            self.tails[ch] = tail;
            out.push(pcm);
        }
        out
    }
}

/// The staged-ladder band weights over the coefficient axis: for each
/// band of the block's partition, `10^((e − e_max) / 16)` — the
/// ladder's own 1.25 dB/step ratio anchored at the block's loudest
/// band. LSP-path and absent envelopes yield a flat weight.
fn band_weights(
    cfg: &StreamConfig,
    _block: &ParsedBlock,
    envelope: Option<&Envelope>,
    m: usize,
) -> Vec<f64> {
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

/// Total gain folded in at the ladder's staged 1.25 dB/step ratio
/// (composition rule unstaged — see the module docs).
fn total_gain_multiplier(total_gain: u32) -> f64 {
    10f64.powf(f64::from(total_gain) / 16.0) / 10f64.powf(64.0 / 16.0)
}

/// Inverse MLT: the fast staged-set path for `{256, 512, 1024,
/// 2048, 4096}`-sample blocks, a direct evaluation of the same
/// oddly-stacked basis for the short sizes outside the typed set
/// (e.g. 128).
fn inverse_transform(coeffs: &[f64]) -> Vec<f64> {
    let m = coeffs.len();
    if let Ok(bs) = BlockSize::from_samples(m as u16) {
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

fn apply_sine_window(mut time: Vec<f64>) -> Vec<f64> {
    let m = time.len() / 2;
    if let Ok(bs) = BlockSize::from_samples(m as u16) {
        let w = Window::sine(bs);
        for (t, &c) in time.iter_mut().zip(w.coeffs()) {
            *t *= c;
        }
    } else {
        let two_m = time.len();
        for (n, t) in time.iter_mut().enumerate() {
            *t *= (std::f64::consts::PI * (n as f64 + 0.5) / two_m as f64).sin();
        }
    }
    time
}

/// Overlap-add: sum the carried tail with the first half, keep the
/// second half as the new tail. Unequal sizes at a block-size
/// transition are truncation-aligned (the staged transition-window
/// shape is open — module docs).
fn overlap_add(tail: Vec<f64>, windowed: Vec<f64>, m: usize) -> (Vec<f64>, Vec<f64>) {
    let mut pcm = windowed[..m].to_vec();
    for (p, t) in pcm.iter_mut().zip(tail.iter()) {
        *p += t;
    }
    let new_tail = windowed[m..].to_vec();
    (pcm, new_tail)
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
        // And the signal is non-trivial.
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
    fn overlap_add_carries_across_blocks() {
        let c = cfg();
        let mut synth = BlockSynth::new(&c);
        let mut coeffs = vec![0i32; 1864];
        coeffs[3] = 100;
        let block = coded_block(false, coeffs.clone(), vec![0i32; 1864], vec![36; 25]);
        let first = synth.block(&block);
        let second = synth.block(&block);
        // The second block's output includes the first's tail: for a
        // steady tone the two outputs differ (attack vs sustained).
        assert_ne!(first[0], second[0]);
        synth.reset();
        let third = synth.block(&block);
        assert_eq!(first[0], third[0], "reset clears the carried tail");
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
        // produces 128 samples.
        let c = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap();
        let mut synth = BlockSynth::new(&c);
        let mut coeffs = vec![0i32; 117];
        coeffs[5] = 300;
        let block = ParsedBlock {
            block_size: 128,
            size_index: 3,
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
        assert!(pcm[0].iter().any(|&x| x.abs() > 1e-9));
    }
}

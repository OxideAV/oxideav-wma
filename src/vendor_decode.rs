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
//!   runtime power the decoder computes, and `g < 18` the staged coarse
//!   power-of-two step (`wma-total-gain-coarse-consts`,
//!   [`vendor_gain_law`] — the F4 noise-band gains, which reach below
//!   zero, share the law). The r450 sweep had found the 1/20 exponent;
//!   the staged table confirms it and the decoder's `g₀ = f32(F(total))`
//!   rounding point.
//! * **LSP path** (§3.1, `BlockSynth::dequantise_lsp`) — the staged
//!   dataflow as validated bit-for-bit on the sandbox-encoded vendor
//!   streams (round 12): one envelope buffer per channel (a coded
//!   channel's conversion or an uncoded channel's 1.0 fill writes its
//!   first `N` slots on a new-envelope block; a reuse block resamples
//!   it in place from the previous block of the frame, `.text
//!   0x5c20`), the binary32 `ratio_k` normaliser, the four
//!   noise-enabled store forms with their rounding points, the shared
//!   generator, and the §5 inverse over all `N` bins.
//! * **absolute scale** — read off the vendor PCM of the
//!   sandbox-encoded conformance vectors (`tests/sandbox_encoded.rs`):
//!   the inverse transform of the dequantised binary32 coefficients on
//!   [`crate::mlt`]'s `2/M` normalisation is the vendor's 16-bit
//!   sample with its **sign inverted** (fitted gain 1.000 000 ± 3·10⁻⁷
//!   on all nine; [`pcm_scale`] = `−1/32768` in the ±1.0 convention).
//!   The r457 black-box calibration (`−4.844·10⁻²` at gain 64) was
//!   this value 0.15 % high; [`ABS_SCALE`] keeps its meaning (the
//!   scale at total gain 64) for the encoder's level laws.
//! * **16-bit output stage** ([`BlockSynth::with_pcm16_output`], opt-in)
//!   — the vendor rounds each block's windowed contribution to the
//!   16-bit grid before the overlap-add: with it the nine conformance
//!   vectors reproduce **99.93 %** of the vendor's samples bit-exactly
//!   and the rest within ±1 LSB (the inverse transform's own
//!   arithmetic, untraced — gap G10).
//!
//! ## Honest approximations (staged gaps)
//!
//! * The transition-window shape is unstaged; the realisation above
//!   (sine slopes of `min(M, neighbour)` centred on the slot
//!   boundaries) is carried — on the sandbox-encoded streams (512 /
//!   256 / 128 and 1024 / 512 / 256 transitions) it reproduces the
//!   vendor PCM to the same ±1 LSB residual as the steady state.
//! * The inverse transform's arithmetic (operation order, binary32
//!   stores) is untraced; this crate's transform runs in binary64.

use crate::dequant_luts::{
    ENVELOPE_WEIGHT_STORED_NEG, ENVELOPE_WEIGHT_STORED_POS, TOTAL_GAIN_EXP_PART_LOG2,
    TOTAL_GAIN_MANTISSA,
};
use crate::lsp_envelope::lsp_envelope;
use crate::mlt::Mlt;
use crate::stream_config::StreamConfig;
use crate::vendor_frame::{Envelope, ParsedBlock};

/// The output scale at total gain 64 (module docs): `−F(64) / 32768`,
/// i.e. a coefficient `q · w` at gain 64 contributes `q · w ·
/// ABS_SCALE` in the ±1.0 float convention. Read off the vendor PCM
/// of the sandbox-encoded conformance vectors (r485; the r457
/// black-box fit was `−4.844·10⁻²`, 0.15 % high); the sign is the
/// vendor's output polarity relative to [`crate::mlt`]'s basis.
pub const ABS_SCALE: f64 = -(f32::from_bits(TOTAL_GAIN_MANTISSA[64]) as f64)
    * (1u32 << TOTAL_GAIN_EXP_PART_LOG2[64 >> 3]) as f64
    / PCM_FULL_SCALE;

/// The decoder's 16-bit output full scale: an inverse-transform
/// output of `x` (on [`crate::mlt`]'s `2/M` normalisation, from the
/// binary32 dequantised coefficients) is the 16-bit sample `−x`; in
/// the ±1.0 float convention that is `−x / 32768`.
pub const PCM_FULL_SCALE: f64 = 32768.0;

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
    /// The reference-measured zero-fill generator's state
    /// ([`ZERO_FILL_RMS_STEPS`]; the crate's own xorshift).
    noise_state: u64,
    /// The staged §2.1 generator (`s ← s · 0x19660d + 0x3c6ef35f`,
    /// from the zero seed) and its previous `r` (module docs).
    lcg_state: u32,
    lcg_prev: i32,
    /// Whether the stream's §2.1 sub-stream is active
    /// ([`crate::vendor_frame::noise_policy`]): draws and the
    /// dither happen only then.
    noise_enabled: bool,
    /// Whether zero-quantised bins of coded channels are noise-filled
    /// at the reference's measured floor ([`ZERO_FILL_RMS_STEPS`]).
    zero_fill_noise: bool,
    /// §3.1 — each channel's **single** per-bin envelope buffer `W`
    /// (`frame_length` slots; a conversion or an uncoded fill writes
    /// the first `N`, a reuse resamples it in place).
    lsp_w: Vec<Vec<f32>>,
    /// §3.1 — each channel's recorded envelope maximum (untouched by
    /// the reuse resample).
    lsp_max: Vec<f32>,
    /// Sample offset of the next block inside its frame (0 = the next
    /// block opens a frame): the §3.1 reuse resample runs only when
    /// the block is not the first of its frame.
    frame_pos: u32,
    /// The size of the previous block of the current frame (the
    /// resample's `prev`).
    prev_in_frame: u16,
    /// §3 VLC path: each channel's single exponent array and the block
    /// size it was coded at (`chan+4` / `ctx+0xbc`).
    vlc_exp: Vec<Option<(Vec<i32>, u16)>>,
    /// Whether the output stage is the vendor's 16-bit one
    /// ([`BlockSynth::with_pcm16_output`]).
    pcm16: bool,
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

/// The §2.1 generator's output scale: the decoder multiplies each
/// draw by `2⁻²⁹`.
pub const NOISE_UNIT: f64 = 1.0 / (1u64 << 29) as f64;

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
/// chain). Since r459 the decoder carries the staged vendor
/// generator and the validated flagged-band form `n · 2⁻²⁹ · w ·
/// ratio_k · F(G)` (module docs), of which this law is the
/// single-flagged-band RMS (the generator's first-difference output
/// has unit RMS to within 2 %); the encoder inverts it to choose F4
/// gains.
pub fn noise_band_rms(gain: i32, band_weight: f64) -> f64 {
    10f64.powf(f64::from(gain - 64) / 20.0) * band_weight * ABS_SCALE.abs()
}

/// The §2.1 dither constant on the VLC path (`.rdata 0x1a56c`, 0.02;
/// the §3.1 path's is 0.04).
const VLC_DITHER: f32 = 0.02;

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
            noise_state: 0x9E37_79B9_7F4A_7C15,
            lcg_state: 0,
            lcg_prev: 0,
            noise_enabled: crate::vendor_frame::noise_policy(cfg).is_some(),
            zero_fill_noise: false,
            lsp_w: vec![vec![0.0; usize::from(cfg.frame_length)]; channels],
            lsp_max: vec![0.0; channels],
            frame_pos: 0,
            prev_in_frame: cfg.frame_length,
            pcm16: false,
            vlc_exp: vec![None; channels],
        }
    }

    /// Select the vendor decoder's **16-bit output stage**: every
    /// block's windowed inverse-transform output is rounded to the
    /// 16-bit grid (`round(x · 32768)`, ties to even) before it is
    /// overlap-added, and the sum saturates at the 16-bit range, so
    /// every emitted sample is an exact `k / 32768`.
    ///
    /// This is a black-box measurement against the vendor decoder's
    /// own PCM on the nine sandbox-encoded conformance vectors
    /// (`docs/audio/wma/reference/sandbox-encoded/`, round 12): with
    /// the per-block rounding the residual against the vendor's
    /// 16-bit output is ±1 LSB on ≈ 0.1 % of samples (the inverse
    /// transform's arithmetic, which the staging has not traced —
    /// gap G10), where a single rounding of the float sum leaves a
    /// triangular ±1 residual on 25 % of them. The default output
    /// stage keeps the unrounded float sum (higher fidelity than the
    /// 16-bit stage); this switch exists for conformance comparison.
    pub fn with_pcm16_output(mut self, on: bool) -> Self {
        self.pcm16 = on;
        self
    }

    /// Fill zero-quantised bins of coded channels with noise at the
    /// reference's measured floor ([`ZERO_FILL_RMS_STEPS`]).
    pub fn with_zero_fill_noise(mut self, on: bool) -> Self {
        self.zero_fill_noise = on;
        self
    }

    /// One draw of the staged §2.1 generator: `s ← s · 0x19660d +
    /// 0x3c6ef35f` (32-bit wrap), `r = (s ≫ 2) + (s ≫ 4)` (arithmetic
    /// shifts), output `r − r_prev` — the integer the decoder scales
    /// by `2⁻²⁹` ([`NOISE_UNIT`]). Validated bit-for-bit on 76 544
    /// vendor draws in the staging.
    fn noise_draw(&mut self) -> i32 {
        self.lcg_state = self
            .lcg_state
            .wrapping_mul(0x0019_660d)
            .wrapping_add(0x3c6e_f35f);
        let s = self.lcg_state as i32;
        let r = (s >> 2) + (s >> 4);
        let out = r.wrapping_sub(self.lcg_prev);
        self.lcg_prev = r;
        out
    }

    /// One unit-variance pseudo-random sample (uniform on ±√3;
    /// xorshift64* — a generator of this crate's own, used only for
    /// the reference-measured zero-fill option, see
    /// [`ZERO_FILL_RMS_STEPS`]).
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
        self.vlc_exp.fill(None);
        self.frame_pos = 0;
        self.prev_in_frame = self.cfg.frame_length;
    }

    /// Synthesise one parsed block into `block_size` PCM samples per
    /// channel (channel-major), at the synthesiser's fixed
    /// `frame_length / 2`-sample lead-in. Applies dequantisation,
    /// the §5 sum/difference inverse when the block is joint, the
    /// inverse lapped transform, neighbour-matched windowing and
    /// overlap-add (module docs).
    pub fn block(&mut self, block: &ParsedBlock) -> Vec<Vec<f64>> {
        let m = usize::from(block.block_size);
        let first_in_frame = self.frame_pos == 0;
        let spec = if self.cfg.exp_vlc {
            self.dequantise(block)
        } else {
            self.dequantise_lsp(block, first_in_frame)
        };
        self.prev_in_frame = block.block_size;
        self.frame_pos = (self.frame_pos + u32::from(block.block_size))
            % u32::from(self.cfg.frame_length).max(1);

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
            if self.pcm16 {
                // The vendor's 16-bit output stage (measured on the
                // sandbox-encoded conformance vectors): each block's
                // windowed contribution is rounded to the 16-bit grid
                // before the overlap-add.
                for v in time.iter_mut() {
                    *v = (*v * PCM_FULL_SCALE).round_ties_even() / PCM_FULL_SCALE;
                }
            }
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
                    if self.pcm16 {
                        *p = p.clamp(-1.0, 32767.0 / PCM_FULL_SCALE);
                    }
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

    /// The §3 VLC path (`flags2` bit 0 set): dequantise a block's coded
    /// channels onto the full coefficient axis (uncoded channels
    /// zero-filled, §5) and run the §5 sum/difference inverse when the
    /// block is joint.
    ///
    /// Each channel keeps **one** exponent array and the block size it
    /// was coded at (`chan+4` / `ctx+0xbc`, `frame-bit-layout.md` §3,
    /// round 11): a B2 = 0 block of any size reads that array on the
    /// coded partition through `m(x) = (x ≪ s) ≫ 6`, `s = 6 +
    /// log2(coded / block)`, and the §2.1 `ratio_k` follows the `.text
    /// 0x5390` law on the integer exponents ([`vlc_band_ratios`]).
    /// Store forms (round 10, validated on vendor bits): coded bin
    /// `f32((0.02·2⁻²⁹·n + q) · f32(w_b · g₀))`, flagged band
    /// `f32((n·2⁻²⁹) · f32(w_b · f32(ratio_k · F(gain_k))))`, tail
    /// `f32((n·2⁻²⁹) · f32(0.02 · (w_b · g₀)))`, `g₀ = f32(F(total))`.
    fn dequantise(&mut self, block: &ParsedBlock) -> Vec<Vec<f64>> {
        let channels = usize::from(self.cfg.channels);
        let m = usize::from(block.block_size);
        let coef_start = usize::from(self.cfg.coef_start(block.block_size));
        let coef_end = usize::from(self.cfg.coef_end(block.block_size)).min(m);
        let f_total = vendor_total_gain(block.total_gain);
        let g = f64::from(f_total as f32);
        let scale = pcm_scale();
        let gain = f_total * scale;
        let ns = f64::from(VLC_DITHER) * NOISE_UNIT;
        let mut spec: Vec<Vec<f64>> = vec![vec![0.0; m]; channels];
        for (ch, chan) in block.channels.iter().enumerate() {
            if !chan.coded {
                continue;
            }
            if let Some(Envelope::Exponents(e)) = chan.envelope.as_ref() {
                self.vlc_exp[ch] = Some((e.clone(), block.block_size));
            }
            // Flat when nothing was coded yet (only right after a
            // reset).
            let w: Vec<f64> = match self.vlc_exp[ch].as_ref() {
                Some((e, coded)) if !e.is_empty() => {
                    coded_size_weights(&self.cfg, e, *coded, block.block_size)
                }
                _ => vec![1.0; m],
            };
            // §2.1: a noise-substituted band's bins leave the
            // coefficient axis and are filled from the generator at
            // its F4 gain.
            let excluded: Vec<(u16, u16)> = if chan.noise_flags.iter().any(|&f| f) {
                noise_excluded_ranges(&self.cfg, block, &chan.noise_flags)
            } else {
                Vec::new()
            };
            let ratios: Vec<f64> = match self.vlc_exp[ch].as_ref() {
                Some((e, coded)) if !e.is_empty() => {
                    vlc_band_ratios(&self.cfg, e, *coded, block.block_size, &excluded)
                }
                _ => vec![1.0; excluded.len()],
            };
            let band_scales: Vec<f64> = ratios
                .iter()
                .zip(chan.noise_gains.iter())
                .map(|(&ratio, &gain_k)| f64::from((ratio * vendor_gain_law(gain_k)) as f32))
                .collect();
            let band_of = |k: usize| -> Option<usize> {
                excluded
                    .iter()
                    .position(|&(lo, hi)| (usize::from(lo)..usize::from(hi)).contains(&k))
            };
            let mut coefs = chan.coefficients.iter().copied();
            if self.noise_enabled {
                // One generator draw per bin, in bin order.
                for k in 0..m {
                    let n = f64::from(self.noise_draw());
                    let flagged = band_of(k);
                    let v = if let Some(b) = flagged {
                        let s = band_scales.get(b).copied().unwrap_or(0.0);
                        f64::from(((n * NOISE_UNIT) * f64::from((w[k] * s) as f32)) as f32)
                    } else if k < coef_start || k >= coef_end {
                        // Below the coded range (v1) / the tail: the
                        // weight of the nearest coded bin.
                        let edge = if k < coef_start {
                            coef_start
                        } else {
                            coef_end.saturating_sub(1)
                        };
                        let dither = f64::from((f64::from(VLC_DITHER) * (w[edge] * g)) as f32);
                        f64::from(((n * NOISE_UNIT) * dither) as f32)
                    } else {
                        // Coded bin: every one, zero or not, carries
                        // the dither.
                        let q = coefs.next().unwrap_or(0);
                        let wg = f64::from((w[k] * g) as f32);
                        let v = f64::from(((ns * n + f64::from(q)) * wg) as f32);
                        if v == 0.0 && q == 0 && self.zero_fill_noise {
                            spec[ch][k] =
                                self.noise_sample() * ZERO_FILL_RMS_STEPS * w[k] * gain.abs();
                            continue;
                        }
                        v
                    };
                    spec[ch][k] = v * scale;
                }
            } else {
                for k in coef_start..coef_end {
                    let Some(q) = coefs.next() else { break };
                    if q != 0 {
                        let v = f64::from((f64::from(q) * w[k]) as f32);
                        spec[ch][k] = f64::from((v * g) as f32) * scale;
                    } else if self.zero_fill_noise {
                        spec[ch][k] = self.noise_sample() * ZERO_FILL_RMS_STEPS * w[k] * gain.abs();
                    }
                }
            }
        }

        // §5 sum/difference inverse over all N bins, on dequantised
        // coefficients, before the inverse transform; both channels
        // count as coded afterwards.
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

    /// The §3.1 path (`flags2` bit 0 clear), in the staged dataflow
    /// (`frame-bit-layout.md` §3.1, validated bit-for-bit on the
    /// sandbox-encoded vendor streams in round 12):
    ///
    /// * **envelope stage** — on a block that codes new envelopes
    ///   (B2 absent or 1) a coded channel converts its ten indices
    ///   into the first `N` slots of its single buffer `W` and records
    ///   the maximum; an uncoded channel sets those slots and the
    ///   maximum to 1.0. On a reuse block (B2 = 0) every channel keeps
    ///   its buffer and, unless the block opens its frame, resamples
    ///   it in place from the previous block's size to this one's
    ///   (`prev > cur`: `W[i] = W[i·r]` ascending; `prev < cur`:
    ///   `W[j·r … j·r+r−1] = W[j]` descending); the maximum is kept;
    /// * **normaliser** — `ratio_k = f32(sqrt(P_k / P_{G−1}))`, `P_k =
    ///   f32(f32(Σ W²) · (1/(hi − lo)))` with the sum accumulated in
    ///   binary32 over the flagged band `[lo, hi)`;
    /// * **dequantiser** — `inv = f32(1/max W)`, `g = f32(inv ·
    ///   F(total))`, `ns = f32(0.04 · 2⁻²⁹)`, one generator draw `n`
    ///   per bin in bin order (shared state, channel 0's bins then
    ///   channel 1's): coded / unflagged bins `f32(((q + n·ns)·W[i])·g)`,
    ///   flagged band `k` `f32((n·W[i])·S_k)` with `S_k =
    ///   f32(((F(gain_k)·ratio_k)·inv)·2⁻²⁹)`, tail `f32(n·T)` with
    ///   `T = f32((g·W[coef_end−1])·ns)`; noise disabled
    ///   `f32((q·W[i])·g)` on the coded range, zero elsewhere;
    /// * **§5** — the sum/difference inverse over all `N` bins,
    ///   binary32 stores.
    ///
    /// The returned coefficients are the decoder's binary32 values
    /// (widened), before the PCM scale.
    fn dequantise_lsp(&mut self, block: &ParsedBlock, first_in_frame: bool) -> Vec<Vec<f64>> {
        let channels = usize::from(self.cfg.channels);
        let m = usize::from(block.block_size);
        let mut spec: Vec<Vec<f64>> = vec![vec![0.0; m]; channels];
        if !block.channels.iter().any(|c| c.coded) {
            // The block ends after the channel flags: no envelope
            // stage, no dequantiser.
            return spec;
        }
        let new_envelopes = !block
            .channels
            .iter()
            .any(|c| c.coded && matches!(c.envelope, Some(Envelope::Reused)));
        let grid = usize::from(self.cfg.lsp_grid_len(block.block_size));
        let mut failed = vec![false; channels];
        for (ch, chan) in block.channels.iter().enumerate() {
            let buf = &mut self.lsp_w[ch];
            if buf.len() < m {
                buf.resize(m, 0.0);
            }
            if new_envelopes {
                if chan.coded {
                    let converted = match chan.envelope.as_ref() {
                        Some(Envelope::LspIndices(idx)) => lsp_envelope(idx, m, grid).ok(),
                        _ => None,
                    };
                    match converted {
                        Some(env) => {
                            buf[..m].copy_from_slice(&env.weights[..m]);
                            self.lsp_max[ch] = env.max;
                        }
                        // The decoder's own decode error for the
                        // channel: it stays silent this block.
                        None => failed[ch] = true,
                    }
                } else {
                    buf[..m].fill(1.0);
                    self.lsp_max[ch] = 1.0;
                }
            } else if !first_in_frame {
                resample_in_place(buf, usize::from(self.prev_in_frame), m);
            }
        }

        let coef_start = usize::from(self.cfg.coef_start(block.block_size));
        let coef_end = usize::from(self.cfg.coef_end(block.block_size)).min(m);
        let f_total = vendor_gain_law(block.total_gain as i32);
        let ns = f64::from((f64::from(LSP_DITHER) * NOISE_UNIT) as f32);
        for (ch, chan) in block.channels.iter().enumerate() {
            if !chan.coded || failed[ch] {
                continue;
            }
            let max = self.lsp_max[ch];
            if max == 0.0 {
                continue;
            }
            let inv = f64::from((1.0 / f64::from(max)) as f32);
            let g = f64::from((inv * f_total) as f32);
            let w: Vec<f64> = self.lsp_w[ch][..m].iter().map(|&x| f64::from(x)).collect();
            let out = &mut spec[ch];
            let mut coefs = chan.coefficients.iter().copied();
            if !self.noise_enabled {
                for (k, slot) in out.iter_mut().enumerate().take(coef_end).skip(coef_start) {
                    let Some(q) = coefs.next() else { break };
                    *slot = f64::from(((f64::from(q) * w[k]) * g) as f32);
                }
                continue;
            }
            let excluded: Vec<(usize, usize)> = if chan.noise_flags.iter().any(|&f| f) {
                noise_excluded_ranges(&self.cfg, block, &chan.noise_flags)
                    .into_iter()
                    .map(|(lo, hi)| (usize::from(lo), usize::from(hi).min(m)))
                    .collect()
            } else {
                Vec::new()
            };
            // `.text 0x5200`: binary32 band energies, ratios against
            // the last flagged band.
            let energy = |&(lo, hi): &(usize, usize)| -> f64 {
                let mut a = 0.0f32;
                for &x in &w[lo..hi.max(lo)] {
                    a = (f64::from(a) + x * x) as f32;
                }
                let len = hi.saturating_sub(lo).max(1) as f64;
                f64::from((f64::from(a) * (1.0 / len)) as f32)
            };
            let p_last = excluded.last().map(energy).unwrap_or(0.0);
            let band_scales: Vec<f64> = excluded
                .iter()
                .enumerate()
                .map(|(k, range)| {
                    let ratio = if k + 1 == excluded.len() {
                        1.0
                    } else {
                        f64::from((energy(range) / p_last).sqrt() as f32)
                    };
                    let gain = chan.noise_gains.get(k).copied().unwrap_or(0);
                    f64::from((((vendor_gain_law(gain) * ratio) * inv) * NOISE_UNIT) as f32)
                })
                .collect();
            let tail = f64::from(((g * w[coef_end.saturating_sub(1)]) * ns) as f32);
            let mut band = 0usize;
            for k in 0..m {
                let n = f64::from(self.noise_draw());
                while band < excluded.len() && k >= excluded[band].1 {
                    band += 1;
                }
                let flagged = band < excluded.len() && k >= excluded[band].0;
                out[k] = if flagged {
                    f64::from(((n * w[k]) * band_scales[band]) as f32)
                } else if k >= coef_end {
                    f64::from((n * tail) as f32)
                } else if k < coef_start {
                    // v1 only (static reading): constant weight.
                    f64::from((((n * ns) * w[coef_start]) * g) as f32)
                } else {
                    let q = f64::from(coefs.next().unwrap_or(0));
                    f64::from((((q + n * ns) * w[k]) * g) as f32)
                };
            }
        }

        if channels == 2 && block.joint_stereo {
            let (a, b) = spec.split_at_mut(1);
            for (mid, side) in a[0].iter_mut().zip(b[0].iter_mut()) {
                let (m0, s0) = (*mid, *side);
                *mid = f64::from((m0 + s0) as f32);
                *side = f64::from((m0 - s0) as f32);
            }
        }
        let scale = pcm_scale();
        for chan in &mut spec {
            for v in chan.iter_mut() {
                *v *= scale;
            }
        }
        spec
    }
}

/// `m(x) = (x ≪ s) ≫ 6` with `s = 6 + log2(coded / block)` — a bin
/// of a `block`-sized block mapped onto the coordinates of the
/// partition the exponents were coded on (§2.1 / §3, `.text 0x5ce0`,
/// `0x5390`).
fn coded_coord(x: usize, coded: u16, block: u16) -> usize {
    let (c, b) = (usize::from(coded), usize::from(block));
    if c >= b { x * (c / b) } else { x / (b / c) }
}

/// The band of `edges` containing `x` (the last band past the end).
fn band_containing(edges: &[u16], x: usize) -> usize {
    let bands = edges.len().saturating_sub(1);
    (0..bands)
        .find(|&j| x < usize::from(edges[j + 1]))
        .unwrap_or(bands.saturating_sub(1))
}

/// §3 VLC path: per-bin band weights of a block of `block` bins from
/// the channel's single exponent array coded at `coded` bins — each
/// bin reads the exponent of the coded partition's band containing
/// `m(i)` (§3 erratum candidate, round 11), weighted `10^((e − e_max)/16)`.
fn coded_size_weights(cfg: &StreamConfig, exps: &[i32], coded: u16, block: u16) -> Vec<f64> {
    let edges = crate::band_partition::exponent_band_edges(cfg.sample_rate, coded);
    let e_max = exps.iter().copied().max().unwrap_or(0);
    (0..usize::from(block))
        .map(|i| {
            let j = band_containing(&edges, coded_coord(i, coded, block));
            envelope_weight(exps.get(j).copied().unwrap_or(e_max) - e_max)
        })
        .collect()
}

/// The `wma-envelope-weight-lut` law on a raw exponent (`.text
/// 0x5390`'s `w(e)`): `e ≤ 0` → `LUTneg[−e] / 2^((−e) ≫ 2)` with
/// `e < −72` read as `−71`; `e > 0` → `2^(e ≫ 2) · LUTpos[min(e, 50)]`.
fn raw_exponent_weight(e: i32) -> f64 {
    if e <= 0 {
        envelope_weight(if e < -72 { -71 } else { e })
    } else {
        let stored = f32::from_bits(ENVELOPE_WEIGHT_STORED_POS[e.min(50) as usize]);
        f64::from(stored) * 2f64.powi(e >> 2)
    }
}

/// §2.1 `ratio_k` on the VLC path (`.text 0x5390`, round 11, validated
/// on 2 975 vendor ratios including cross-size reuse): from the
/// channel's integer exponents coded at `coded` bins, for the flagged
/// bands `ranges` (`[lo, hi)` in the block's own bins, `hi` clipped to
/// `coef_end`) of a `block`-bin block.
fn vlc_band_ratios(
    cfg: &StreamConfig,
    exps: &[i32],
    coded: u16,
    block: u16,
    ranges: &[(u16, u16)],
) -> Vec<f64> {
    let g = ranges.len();
    if g == 0 {
        return Vec::new();
    }
    let edges = crate::band_partition::exponent_band_edges(cfg.sample_rate, coded);
    let x = |j: usize| exps.get(j).copied().unwrap_or(0);
    let s = 6 + (i32::from(coded).ilog2() as i32 - i32::from(block).ilog2() as i32);
    let mid_rate =
        cfg.version != crate::header::Version::V1 && (32_000..44_100).contains(&cfg.sample_rate);
    if s == 6 || mid_rate {
        let xs: Vec<i32> = ranges
            .iter()
            .map(|&(lo, _)| {
                x(band_containing(
                    &edges,
                    coded_coord(usize::from(lo), coded, block),
                ))
            })
            .collect();
        let last = xs[g - 1];
        return xs
            .iter()
            .enumerate()
            .map(|(k, &xk)| {
                if k + 1 == g {
                    1.0
                } else {
                    f64::from(raw_exponent_weight(xk - last) as f32)
                }
            })
            .collect();
    }
    // Cross-size reuse.
    let coef_end_full = usize::from(cfg.coef_end(cfg.frame_length));
    let l = usize::from(coded) * coef_end_full / usize::from(cfg.frame_length);
    let p: Vec<f64> = ranges
        .iter()
        .map(|&(lo, hi)| {
            let (lo, hi) = (usize::from(lo), usize::from(hi));
            let j1 = band_containing(&edges, coded_coord(lo, coded, block));
            let j2 = band_containing(
                &edges,
                coded_coord(hi, coded, block).min(l).saturating_sub(1),
            );
            let w1 = raw_exponent_weight(x(j1));
            if j1 == j2 || hi <= lo {
                return f64::from((w1 * w1) as f32);
            }
            let w2 = raw_exponent_weight(x(j2));
            let t = usize::from(edges[(j1 + 1).min(edges.len() - 1)]).min(l);
            let t = if s > 6 {
                (t + (1 << (s - 7))) >> (s - 6)
            } else {
                t << (6 - s)
            };
            let (hi_f, lo_f, t_f) = (hi as f64, lo as f64, t as f64);
            f64::from((((hi_f - t_f) * w2 * w2 + (t_f - lo_f) * w1 * w1) / (hi_f - lo_f)) as f32)
        })
        .collect();
    let last = p[g - 1];
    p.iter()
        .enumerate()
        .map(|(k, &pk)| {
            if k + 1 == g || last <= 0.0 {
                1.0
            } else {
                f64::from((pk / last).sqrt() as f32)
            }
        })
        .collect()
}

/// The §3.1 reuse resample (`.text 0x5c20`), in place on a channel's
/// envelope buffer from the previous block's size `prev` to this
/// block's size `cur`: `prev > cur` decimates (`W[i] = W[i·r]`,
/// ascending), `prev < cur` replicates (`W[j·r … j·r + r − 1] =
/// W[j]`, descending), equal sizes leave it unchanged.
pub(crate) fn resample_in_place(w: &mut [f32], prev: usize, cur: usize) {
    if prev == 0 || cur == 0 || prev == cur {
        return;
    }
    if prev > cur {
        let r = prev / cur;
        for i in 0..cur.min(w.len()) {
            if i * r < w.len() {
                w[i] = w[i * r];
            }
        }
    } else {
        let r = cur / prev;
        for j in (0..cur / r).rev() {
            let v = w[j];
            for t in 0..r {
                if let Some(slot) = w.get_mut(j * r + t) {
                    *slot = v;
                }
            }
        }
    }
}

/// The §3.1 dither constant `ctx+0x388` on the LSP path (0.04; the
/// VLC path's is 0.02).
const LSP_DITHER: f32 = 0.04;

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
        .filter(|&(_, &f)| f)
        .map(|(range, _)| range)
        .filter(|&(lo, hi)| lo < hi)
        .collect()
}

/// The staged band weights over the coefficient axis: for each band
/// of the block's partition, `10^((e − e_max) / 16)` through the
/// staged `wma-envelope-weight-lut` ([`envelope_weight`]) — the
/// vendor's per-band weight anchored at the block's loudest band.
/// Absent envelopes yield a flat weight; a §3.1 envelope is not a
/// band envelope.
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
    vendor_gain_law(i32::try_from(total_gain).unwrap_or(i32::MAX))
}

/// The staged gain law `F(g)` over every integer `g` (`.text 0x48b0`;
/// the B1 total gain and the F4 noise-band gains share it), returned
/// unrounded as the decoder leaves it on the x87 stack:
///
/// * `18 ≤ g < 146` — `mantissa[g] · 2^(4 + (g >> 3))`
///   (`wma-total-gain-lut`);
/// * `0 ≤ g < 18` — the coarse step `2^((g >> 3) + 1) · c₀`, `c₀` the
///   binary32 `0x3f5e939b` (`wma-total-gain-coarse-consts`);
/// * `g < 0` — `c₁ / 2^(((−g) >> 3) + 1)`, `c₁` the binary32
///   `0x3ec41138`;
/// * `g ≥ 146` — the runtime power `10^(0.05 · g)`.
pub fn vendor_gain_law(g: i32) -> f64 {
    if g < 0 {
        let c1 = f64::from(f32::from_bits(GAIN_COARSE_NEG_BITS));
        let k = ((-i64::from(g)) >> 3) + 1;
        c1 / 2f64.powi(k.min(1074) as i32)
    } else if g < 18 {
        let c0 = f64::from(f32::from_bits(GAIN_COARSE_LOW_BITS));
        c0 * f64::from(1u32 << ((g >> 3) + 1))
    } else if (g as usize) < TOTAL_GAIN_MANTISSA.len() {
        let mantissa = f64::from(f32::from_bits(TOTAL_GAIN_MANTISSA[g as usize]));
        mantissa * f64::from(1u32 << TOTAL_GAIN_EXP_PART_LOG2[(g as usize) >> 3])
    } else {
        10f64.powf(0.05 * f64::from(g))
    }
}

/// `wma-total-gain-coarse-consts` `.rdata 0x1a530`: the `0 ≤ g < 18`
/// factor (0.869439781).
const GAIN_COARSE_LOW_BITS: u32 = 0x3f5e_939b;
/// `wma-total-gain-coarse-consts` `.rdata 0x1a534`: the `g < 0`
/// numerator (0.382943869).
const GAIN_COARSE_NEG_BITS: u32 = 0x3ec4_1138;

/// The calibrated PCM scale on top of the vendor composition:
/// `ABS_SCALE / F(64)`, so that `q · w · F(g) · pcm_scale()` equals the
/// r457-calibrated `q · w · 10^((g − 64)/20) · ABS_SCALE` (module docs).
pub fn pcm_scale() -> f64 {
    -1.0 / PCM_FULL_SCALE
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
        // Above the table: the runtime power; below: the staged
        // coarse step (6 dB per 8 steps, `wma-total-gain-coarse-consts`).
        assert!((vendor_total_gain(150) / 10f64.powf(7.5) - 1.0).abs() < 1e-6);
        for (g, want) in [
            (0, 1.738_879_6),
            (7, 1.738_879_6),
            (8, 3.477_759_1),
            (17, 6.955_518_2),
        ] {
            let got = vendor_gain_law(g);
            assert!((got - want).abs() < 1e-6, "g={g}: {got}");
        }
        assert_eq!(vendor_total_gain(1), vendor_gain_law(1));
        // g < 0: 0.382943869 / 2^(((−g) >> 3) + 1).
        assert!((vendor_gain_law(-1) - 0.382_943_869 / 2.0).abs() < 1e-9);
        assert!((vendor_gain_law(-8) - 0.382_943_869 / 4.0).abs() < 1e-9);
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
    /// coefficients decoded at total gain G (the level law,
    /// [`noise_band_rms`], the single-band case of the staged flagged
    /// form): the time-domain energy of a stream of noise-substituted
    /// blocks matches that of the coded twin within the generator's
    /// statistical spread. Both blocks sit at the same total gain so
    /// that the coded bins' `0.02 · noise` dither (which does scale
    /// with the total gain) is the same negligible floor in both.
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
                    (vec![true, true], vec![g, g], vec![0i32; 717], g as u32)
                } else {
                    // The twin: q = 1 on every bin of the two walk bands
                    // ([717, 932)), zero elsewhere, decoded at total gain g.
                    let mut c = vec![0i32; 932];
                    for v in &mut c[717..932] {
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
                total_gain: g as u32,
                n_coef: 932,
                channels: vec![ChannelBlock {
                    coded: true,
                    envelope: Some(Envelope::Exponents(vec![40; bands])),
                    noise_flags: vec![true, true],
                    noise_gains: vec![g + 10, g + 10],
                    coefficients: vec![0; 717],
                }],
            };
            let out = synth.block(&block);
            acc += out[0].iter().map(|v| v * v).sum::<f64>();
        }
        let db = 10.0 * (acc / noise).log10();
        assert!((9.0..11.0).contains(&db), "+10 gain steps = {db:.2} dB");
    }

    #[test]
    fn reuse_resample_follows_the_staged_law() {
        // prev > cur: W[i] = W[i·r] ascending.
        let mut w: Vec<f32> = (0..8).map(|i| i as f32).collect();
        resample_in_place(&mut w, 8, 4);
        assert_eq!(&w[..4], &[0.0, 2.0, 4.0, 6.0]);
        // prev < cur: W[j·r … j·r + r − 1] = W[j] descending.
        let mut w: Vec<f32> = (0..8).map(|i| i as f32).collect();
        resample_in_place(&mut w, 2, 8);
        assert_eq!(w, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
        // Equal sizes: untouched.
        let mut w: Vec<f32> = (0..4).map(|i| i as f32).collect();
        resample_in_place(&mut w, 4, 4);
        assert_eq!(w, vec![0.0, 1.0, 2.0, 3.0]);
    }

    #[test]
    fn coded_partition_mapping_and_vlc_ratio_law() {
        // m(x) = (x ≪ s) ≫ 6: coded at 512, read by a 256 block → 2x;
        // coded at 256, read by a 512 block → x / 2.
        assert_eq!(coded_coord(100, 512, 256), 200);
        assert_eq!(coded_coord(101, 256, 512), 50);
        assert_eq!(coded_coord(77, 512, 512), 77);
        // The raw-exponent weight: e > 50 keeps its own power of two.
        assert_eq!(raw_exponent_weight(0), envelope_weight(0));
        assert_eq!(raw_exponent_weight(-80), envelope_weight(-71));
        assert!((raw_exponent_weight(54) / raw_exponent_weight(50) - 2.0).abs() < 1e-12);
        // Same-size ratios: f32(w(x_k − x_last)), the last band 1.
        let cfg = StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap();
        let edges = crate::band_partition::exponent_band_edges(22_050, 1024);
        let exps: Vec<i32> = (0..edges.len() as i32 - 1).map(|b| 40 - b).collect();
        let ranges = vec![(717u16, 884u16), (884, 932)];
        let r = vlc_band_ratios(&cfg, &exps, 1024, 1024, &ranges);
        let j0 = band_containing(&edges, 717);
        let j1 = band_containing(&edges, 884);
        assert_eq!(r[1], 1.0);
        assert_eq!(
            r[0],
            f64::from(raw_exponent_weight(exps[j0] - exps[j1]) as f32)
        );
        // Cross-size reuse with both bands inside one coded band each:
        // `ratio = f32(sqrt(f32(w0²) / f32(w1²)))`.
        let r = vlc_band_ratios(&cfg, &exps, 1024, 512, &[(358, 440), (440, 466)]);
        assert_eq!(r.len(), 2);
        assert_eq!(r[1], 1.0);
        assert!(r[0] > 0.0 && r[0].is_finite());
    }
}

//! Vendor-wire encoder **signal stage**: PCM → forward lapped
//! transform → envelope extraction → quantisation → rate control —
//! the forward mirror of [`crate::vendor_decode`], feeding
//! [`crate::vendor_encode::VendorBitWriter`].
//!
//! ## Mirror contract
//!
//! Every numeric rule here is the exact inverse of the decode
//! composition (single source of truth — the decode-side functions
//! themselves are called for the shared pieces):
//!
//! * **Geometry** — a block of half-length `M` at slot
//!   `[pos, pos + M)` is analysed from the `2M` input samples
//!   `[pos − M/2, pos + 3M/2)`, windowed by the same
//!   neighbour-matched transition window the synthesiser applies
//!   (`vendor_decode::transition_window`), then forward-transformed
//!   by the same oddly-stacked basis (bare cosine bank; the decoder's
//!   `2/M`-normalised inverse + windowed overlap-add reconstructs —
//!   the TDAC identity is pinned by test).
//! * **Quantisation** — the decoder computes
//!   `spec = q · w_band · 10^((g − 64)/20) · ABS_SCALE` with
//!   `w_band = 10^((e − e_max)/16)` (`vendor_decode::band_weights`,
//!   `total_gain_multiplier`); the encoder divides by exactly that
//!   factor and rounds. Envelope exponents come from per-band RMS on
//!   the staged ladder's own 1.25 dB/step scale, anchored so the
//!   loudest band sits at exponent 40, chain-clamped to the scale
//!   VLC's ±60 delta range.
//! * **Stereo** — the §5 mid/side forward carries the halving
//!   (`m = (l + r)/2`, `s = (l − r)/2`), since the decoder's inverse
//!   is the unhalved `l = m + s`, `r = m − s`.
//! * **Rate control** — per frame, one shared gain offset over the
//!   per-block base gains is searched so the emitted frame fits the
//!   §1 bounds ([`crate::vendor_encode::VendorBitWriter::max_frame_bits`])
//!   and tracks the configuration's average bits per frame. Raising
//!   the total gain enlarges the quantisation step (the decode
//!   composition multiplies by it), which is the coarseness knob.
//!
//! The emitted timeline mirrors the decoder's fixed
//! `frame_length / 2` lead-in: decoding an encoded stream yields
//! `frame_length / 2` near-zero samples followed by the input.

use crate::stream_config::StreamConfig;
use crate::vendor_decode::{band_weights, total_gain_multiplier, transition_window, ABS_SCALE};
use crate::vendor_encode::{EmitError, EncBlockData, EncChannelData, EncEnvelope, VendorBitWriter};
use crate::vendor_frame::{escape_level_width, Envelope};

/// Per-block stereo policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StereoMode {
    /// Decide per block: joint (mid/side) when the side channel
    /// carries much less energy than the pair.
    #[default]
    Auto,
    /// Never fold; both channels code independently.
    Independent,
    /// Always fold to mid/side.
    Joint,
}

/// Per-frame block-size policy (VBL streams).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BlockPolicy {
    /// Full-length blocks on steady frames; split to shorter blocks
    /// (size index 1, or 2 where the stream allows it) on frames with
    /// a strong intra-frame energy transient.
    #[default]
    Auto,
    /// Every frame uses this size index for all its blocks.
    Fixed(u8),
}

/// Encoder tuning (all fields have working defaults).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EncoderSettings {
    /// Stereo folding policy.
    pub stereo: StereoMode,
    /// Block-size scheduling policy.
    pub blocks: BlockPolicy,
    /// Target peak |q| the per-block base gain aims at (clamped to
    /// the escape ceiling of the resulting gain tier).
    pub target_peak_q: f64,
}

impl Default for EncoderSettings {
    fn default() -> Self {
        Self {
            stereo: StereoMode::Auto,
            blocks: BlockPolicy::Auto,
            target_peak_q: 350.0,
        }
    }
}

/// Encoder failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// A wire-emission failure (see [`EmitError`]).
    Emit(EmitError),
    /// `push` was handed per-channel slices of unequal length or the
    /// wrong channel count.
    BadInput {
        /// What was wrong, statically.
        what: &'static str,
    },
    /// A [`BlockPolicy::Fixed`] index outside the stream's block-size
    /// set.
    BadPolicyIndex {
        /// The offending index.
        index: u8,
    },
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EncodeError::Emit(e) => write!(f, "{e}"),
            EncodeError::BadInput { what } => write!(f, "oxideav-wma: {what}"),
            EncodeError::BadPolicyIndex { index } => {
                write!(
                    f,
                    "oxideav-wma: fixed block-size index {index} out of range"
                )
            }
        }
    }
}

impl std::error::Error for EncodeError {}

impl From<EmitError> for EncodeError {
    fn from(e: EmitError) -> Self {
        EncodeError::Emit(e)
    }
}

/// The vendor-wire encoder: buffered PCM in (±1.0 float,
/// channel-planar), `block_align`-byte §1 codec packets out.
///
/// Latency: two frames of input are buffered before the first frame
/// encodes (one for the analysis window's forward reach, one for the
/// F1 one-ahead pipeline's next-frame schedule); [`Self::finish`]
/// zero-pads and drains.
#[derive(Debug, Clone)]
pub struct VendorEncoder {
    cfg: StreamConfig,
    settings: EncoderSettings,
    writer: VendorBitWriter,
    /// Per-channel buffered input; `input[ch][0]` is absolute sample
    /// `input_base`.
    input: Vec<Vec<f64>>,
    input_base: u64,
    /// Total samples pushed per channel.
    pushed: u64,
    /// Frames fully encoded so far.
    encoded_frames: u64,
    /// The previous frame's last block size (windowing context across
    /// frames).
    prev_block_size: Option<u16>,
    /// Per-frame average bit budget from the container bitrate.
    target_frame_bits: u64,
}

impl VendorEncoder {
    /// An encoder for one stream configuration with default settings.
    ///
    /// # Errors
    ///
    /// [`EmitError::LspPathUnsupported`] for `flags2` bit 0 clear
    /// (the §3.1 conversion tables are a staged gap).
    pub fn new(cfg: &StreamConfig) -> Result<Self, EncodeError> {
        Self::with_settings(cfg, EncoderSettings::default())
    }

    /// An encoder with explicit [`EncoderSettings`].
    ///
    /// # Errors
    ///
    /// As [`Self::new`], plus [`EncodeError::BadPolicyIndex`] for an
    /// out-of-range [`BlockPolicy::Fixed`] index.
    pub fn with_settings(
        cfg: &StreamConfig,
        settings: EncoderSettings,
    ) -> Result<Self, EncodeError> {
        if let BlockPolicy::Fixed(index) = settings.blocks {
            if cfg.block_size_for_index(index).is_none() || (!cfg.vbl_enabled && index != 0) {
                return Err(EncodeError::BadPolicyIndex { index });
            }
        }
        let writer = VendorBitWriter::new(cfg)?;
        let target_frame_bits =
            (u64::from(cfg.avg_bytes_per_sec) * 8 * u64::from(cfg.frame_length))
                / u64::from(cfg.sample_rate);
        Ok(Self {
            cfg: cfg.clone(),
            settings,
            writer,
            input: vec![Vec::new(); usize::from(cfg.channels)],
            input_base: 0,
            pushed: 0,
            encoded_frames: 0,
            prev_block_size: None,
            target_frame_bits,
        })
    }

    /// The stream configuration this encoder emits for.
    pub fn config(&self) -> &StreamConfig {
        &self.cfg
    }

    /// Append PCM (one slice per channel, equal lengths, ±1.0
    /// convention) and encode every frame that has become ready.
    ///
    /// # Errors
    ///
    /// [`EncodeError`] — the encoder is unusable after an error.
    pub fn push(&mut self, pcm: &[Vec<f64>]) -> Result<(), EncodeError> {
        if pcm.len() != self.input.len() {
            return Err(EncodeError::BadInput {
                what: "push: one slice per channel required",
            });
        }
        let len = pcm[0].len();
        if pcm.iter().any(|c| c.len() != len) {
            return Err(EncodeError::BadInput {
                what: "push: per-channel slices must have equal length",
            });
        }
        for (buf, chan) in self.input.iter_mut().zip(pcm.iter()) {
            buf.extend_from_slice(chan);
        }
        self.pushed += len as u64;
        // Frame k is ready once frame k+1's PCM is complete (the F1
        // pipeline needs the next frame's schedule, and the analysis
        // window reaches frame_length/2 past the frame end).
        let flen = u64::from(self.cfg.frame_length);
        while (self.encoded_frames + 2) * flen <= self.pushed {
            let k = self.encoded_frames;
            self.encode_frame(k, false)?;
        }
        Ok(())
    }

    /// Close the stream: zero-pad the input to whole frames, encode
    /// the tail, and return every §1 packet.
    ///
    /// # Errors
    ///
    /// [`EncodeError`].
    pub fn finish(mut self) -> Result<Vec<Vec<u8>>, EncodeError> {
        let flen = u64::from(self.cfg.frame_length);
        let total_frames = self.pushed.div_ceil(flen).max(1);
        // Pad so every frame's PCM and analysis lookahead exist.
        let need = (total_frames + 1) * flen;
        let pad = (need - self.pushed) as usize;
        for buf in &mut self.input {
            buf.extend(std::iter::repeat(0.0).take(pad));
        }
        self.pushed = need;
        while self.encoded_frames < total_frames {
            let k = self.encoded_frames;
            let last = k + 1 == total_frames;
            self.encode_frame(k, last)?;
        }
        Ok(self.writer.finish()?)
    }

    /// Input sample `abs` of channel `ch` (0 outside the pushed
    /// range).
    fn sample(&self, ch: usize, abs: i64) -> f64 {
        if abs < 0 {
            return 0.0;
        }
        let abs = abs as u64;
        if abs < self.input_base || abs >= self.pushed {
            return 0.0;
        }
        self.input[ch][(abs - self.input_base) as usize]
    }

    /// Drop input no analysis window can reach any more.
    fn drain_input(&mut self) {
        let flen = u64::from(self.cfg.frame_length);
        let keep_from = (self.encoded_frames * flen).saturating_sub(flen / 2);
        if keep_from > self.input_base {
            let drop = (keep_from - self.input_base) as usize;
            for buf in &mut self.input {
                buf.drain(..drop);
            }
            self.input_base = keep_from;
        }
    }

    /// The frame's block-size schedule (a list of size indices whose
    /// sizes sum to `frame_length`).
    fn schedule(&self, frame: u64) -> Vec<u8> {
        if !self.cfg.vbl_enabled || self.cfg.n_block_sizes <= 1 {
            return vec![0];
        }
        let split_index = match self.settings.blocks {
            BlockPolicy::Fixed(i) => return vec![i; 1usize << i],
            BlockPolicy::Auto => {
                // Deepest split considered: index 2 (quarter frame)
                // where the stream allows it, else 1.
                if self.cfg.n_block_sizes >= 4 {
                    2u8
                } else {
                    1u8
                }
            }
        };
        // Transient probe: max/min energy over 16 sub-windows of the
        // frame (channel mix).
        let flen = i64::from(self.cfg.frame_length);
        let base = frame as i64 * flen;
        let sub = usize::from(self.cfg.frame_length) / 16;
        let mut energies = [0.0f64; 16];
        for (w, e) in energies.iter_mut().enumerate() {
            let mut acc = 0.0;
            for i in 0..sub {
                let abs = base + (w * sub + i) as i64;
                for ch in 0..self.input.len() {
                    let v = self.sample(ch, abs);
                    acc += v * v;
                }
            }
            *e = acc / sub as f64;
        }
        let peak = energies.iter().cloned().fold(0.0, f64::max);
        let floor = energies.iter().cloned().fold(f64::INFINITY, f64::min);
        let transient = peak > 1e-9 && peak > 60.0 * (floor + 1e-12);
        if transient {
            vec![split_index; 1usize << split_index]
        } else {
            vec![0]
        }
    }

    /// Encode frame `k` (schedule → analyse → quantise under rate
    /// control → emit).
    fn encode_frame(&mut self, k: u64, last: bool) -> Result<(), EncodeError> {
        let sched = self.schedule(k);
        let next_first = if last {
            None
        } else {
            Some(self.schedule(k + 1)[0])
        };

        // Analyse every block of the frame once (gain-independent).
        let flen = i64::from(self.cfg.frame_length);
        let mut pos = k as i64 * flen;
        let mut prepared: Vec<PreparedBlock> = Vec::with_capacity(sched.len());
        for (bi, &idx) in sched.iter().enumerate() {
            let m = i64::from(
                self.cfg
                    .block_size_for_index(idx)
                    .expect("schedule uses valid indices"),
            );
            let prev = self
                .prev_block_size
                .filter(|_| bi == 0)
                .map(i64::from)
                .or_else(|| {
                    bi.checked_sub(1)
                        .map(|p| i64::from(self.cfg.block_size_for_index(sched[p]).expect("valid")))
                })
                .unwrap_or(m);
            let next = sched
                .get(bi + 1)
                .map(|&i| i64::from(self.cfg.block_size_for_index(i).expect("valid")))
                .or_else(|| {
                    next_first.map(|i| i64::from(self.cfg.block_size_for_index(i).expect("valid")))
                })
                .unwrap_or(m);
            // 2M input window at [pos − M/2, pos + 3M/2), transition
            // windowed exactly as the synthesiser will window its
            // inverse.
            let mut specs: Vec<Vec<f64>> = Vec::with_capacity(self.input.len());
            for ch in 0..self.input.len() {
                let mut time: Vec<f64> = (0..2 * m)
                    .map(|n| self.sample(ch, pos - m / 2 + n))
                    .collect();
                transition_window(&mut time, prev as usize, next as usize);
                specs.push(forward_transform(&time));
            }
            prepared.push(self.prepare_block(idx, specs));
            self.prev_block_size = Some(m as u16);
            pos += m;
        }

        // Rate control: shared gain offset over the per-block base
        // gains; §1 bounds are hard, the configuration's average
        // bits/frame is the soft target.
        let max_bits = self.writer.max_frame_bits().saturating_sub(8);
        let target = self.target_frame_bits.min(max_bits);
        let mut offset: i32 = 0;
        let mut best: Option<(u64, Vec<EncBlockData>)> = None;
        let mut prev_bits: Option<u64> = None;
        for _ in 0..16 {
            let blocks: Vec<EncBlockData> =
                prepared.iter().map(|p| realise_block(p, offset)).collect();
            let bits = self.writer.trial_frame_bits(&blocks, next_first)?;
            if bits <= max_bits {
                let better = match &best {
                    Some((b, _)) => bits.abs_diff(target) < b.abs_diff(target),
                    None => true,
                };
                if better {
                    best = Some((bits, blocks));
                }
                if bits > target {
                    offset += 3; // coarser, toward the target
                } else if bits * 2 < target && offset > -40 && prev_bits != Some(bits) {
                    offset -= 4; // finer, use the budget
                } else {
                    break;
                }
                prev_bits = Some(bits);
            } else {
                // Over the hard bound: markedly coarser.
                offset += 8;
            }
            if offset > 140 {
                break;
            }
        }
        let blocks = match best {
            Some((_, b)) => b,
            None => prepared.iter().map(silent_block).collect(),
        };
        self.writer.write_frame(&blocks, next_first)?;
        self.encoded_frames = k + 1;
        self.drain_input();
        Ok(())
    }

    /// Envelope + normalised spectra + base gain for one block
    /// (everything gain-offset-independent).
    fn prepare_block(&self, size_index: u8, mut specs: Vec<Vec<f64>>) -> PreparedBlock {
        let block_size = self
            .cfg
            .block_size_for_index(size_index)
            .expect("valid index");
        let channels = specs.len();

        // §5 stereo fold (with the encoder-side halving).
        let joint = if channels == 2 {
            let (e_mid, e_side) = {
                let (mut em, mut es) = (0.0f64, 0.0f64);
                for (l, r) in specs[0].iter().zip(specs[1].iter()) {
                    let m = (l + r) / 2.0;
                    let s = (l - r) / 2.0;
                    em += m * m;
                    es += s * s;
                }
                (em, es)
            };
            match self.settings.stereo {
                StereoMode::Independent => false,
                StereoMode::Joint => true,
                StereoMode::Auto => e_side * 4.0 < e_mid + 1e-24,
            }
        } else {
            false
        };
        if joint {
            let (a, b) = specs.split_at_mut(1);
            for (l, r) in a[0].iter_mut().zip(b[0].iter_mut()) {
                let m = (*l + *r) / 2.0;
                let s = (*l - *r) / 2.0;
                *l = m;
                *r = s;
            }
        }

        let coef_start = usize::from(self.cfg.coef_start(block_size));
        let coef_end = usize::from(self.cfg.coef_end(block_size));
        let edges = crate::band_partition::exponent_band_edges(self.cfg.sample_rate, block_size);

        let mut chans = Vec::with_capacity(channels);
        for spec in &specs {
            chans.push(prepare_channel(
                &self.cfg,
                spec,
                block_size,
                &edges,
                coef_start,
                coef_end,
                self.settings.target_peak_q,
            ));
        }
        let peak = chans
            .iter()
            .flatten()
            .map(|c| c.peak)
            .fold(0.0f64, f64::max);
        PreparedBlock {
            size_index,
            joint_stereo: joint,
            chans,
            min_gain: min_gain_for_peak(peak),
        }
    }
}

/// The smallest gain in `[1, 250]` whose escape ceiling
/// (`2^w_lvl(g) - 1`) admits `peak` under the decode composition —
/// the floor that keeps the rate loop's "finer" direction from
/// clipping the loudest coefficients.
fn min_gain_for_peak(peak: f64) -> u32 {
    if peak <= 0.0 {
        return 1;
    }
    for g in 1..=250u32 {
        let ceiling = ((1i64 << escape_level_width(g)) - 1) as f64;
        if peak / (total_gain_multiplier(g) * ABS_SCALE.abs()) <= ceiling {
            return g;
        }
    }
    250
}

/// A block after analysis, before gain-offset realisation.
#[derive(Debug, Clone)]
struct PreparedBlock {
    size_index: u8,
    joint_stereo: bool,
    chans: Vec<Option<PreparedChannel>>,
    /// Smallest B1 gain whose escape-level ceiling admits the
    /// block's peak |q| (the level literal is `w_lvl(g)` bits, so a
    /// finer gain must never push the peak past `2^w_lvl - 1`).
    min_gain: u32,
}

/// A channel with signal: its envelope and envelope-normalised
/// coded-axis spectrum.
#[derive(Debug, Clone)]
struct PreparedChannel {
    exponents: Vec<i32>,
    /// `spec[k] / w_band[k]` over the coded axis.
    normalised: Vec<f64>,
    /// Heuristic B1 gain putting the peak |q| at the target.
    base_gain: u32,
    /// Peak |spec / w| over the coded axis.
    peak: f64,
}

/// The anchor exponent the loudest band sits at (inside both the
/// staged ladder's 113 entries and the v1 base field's range).
const ENVELOPE_ANCHOR: i32 = 40;
/// How far below the anchor a band's exponent may fall — 24 steps
/// (30 dB). Measured against the black-box reference decoder
/// (crafted single-coefficient frames sweeping the band exponent
/// with the anchor pinned): the per-band weight laws agree within
/// 0.4 dB down to 24 steps below the anchor, then diverge sharply
/// (the reference decays ~5 dB faster by 40 steps and saturates,
/// and clamps below exponent 0), so the encoder keeps every emitted
/// exponent inside the well-matched regime — quieter bands spend a
/// few more bits instead of landing in the divergent region.
const ENVELOPE_FLOOR: i32 = ENVELOPE_ANCHOR - 24;

fn prepare_channel(
    cfg: &StreamConfig,
    spec: &[f64],
    block_size: u16,
    edges: &[u16],
    coef_start: usize,
    coef_end: usize,
    target_peak_q: f64,
) -> Option<PreparedChannel> {
    // Per-band RMS over the coded range.
    let bands = edges.len() - 1;
    let mut rms = vec![0.0f64; bands];
    for (b, r) in rms.iter_mut().enumerate() {
        let lo = usize::from(edges[b]).max(coef_start);
        let hi = usize::from(edges[b + 1]).min(coef_end);
        if lo >= hi {
            continue;
        }
        let e: f64 = spec[lo..hi].iter().map(|v| v * v).sum();
        *r = (e / (hi - lo) as f64).sqrt();
    }
    let rms_max = rms.iter().cloned().fold(0.0, f64::max);
    if rms_max <= 1e-12 {
        return None; // silent channel: F2 = 0
    }

    // Exponents on the ladder's 1.25 dB/step scale, loudest band at
    // the anchor, chain-clamped to the scale VLC's ±60 delta range
    // (the ±60 clamp is vacuous inside [floor, anchor] but kept for
    // the v1 base path, whose first exponent is clamped to the 5-bit
    // field).
    let mut exponents = Vec::with_capacity(bands);
    let mut prev: Option<i32> = None;
    for r in &rms {
        let rel = if *r > 0.0 {
            (16.0 * (r / rms_max).log10()).round() as i32
        } else {
            ENVELOPE_FLOOR - ENVELOPE_ANCHOR
        };
        let mut e = (ENVELOPE_ANCHOR + rel).clamp(ENVELOPE_FLOOR, ENVELOPE_ANCHOR);
        if let Some(p) = prev {
            e = e.clamp(p - 60, p + 60);
        } else if cfg.version == crate::header::Version::V1 {
            e = e.clamp(10, 41);
        }
        exponents.push(e);
        prev = Some(e);
    }

    // Envelope-normalised spectrum over the coded axis, using the
    // decoder's own weight function.
    let weights = band_weights(
        cfg,
        Some(&Envelope::Exponents(exponents.clone())),
        usize::from(block_size),
    );
    let normalised: Vec<f64> = (coef_start..coef_end)
        .map(|k| spec[k] / weights[k])
        .collect();

    // Base gain: peak |q| ≈ target under the decode composition
    // |spec| = |q| · w · 10^((g − 64)/20) · |ABS_SCALE|.
    let peak = normalised.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    let base_gain = if peak > 0.0 {
        let mult = peak / (target_peak_q * ABS_SCALE.abs());
        (64.0 + 20.0 * mult.log10()).round().clamp(1.0, 250.0) as u32
    } else {
        64
    };
    Some(PreparedChannel {
        exponents,
        normalised,
        base_gain,
        peak,
    })
}

/// Realise a prepared block at a gain offset: round the normalised
/// spectra with the block's B1 gain, clamping |q| to the escape
/// ceiling of the gain's level-width tier.
fn realise_block(prep: &PreparedBlock, offset: i32) -> EncBlockData {
    let base = prep
        .chans
        .iter()
        .flatten()
        .map(|c| c.base_gain)
        .max()
        .unwrap_or(64);
    let gain = ((base as i32 + offset).clamp(1, 250) as u32).max(prep.min_gain);
    let ceiling = (1i64 << escape_level_width(gain)) - 1;
    let divisor = total_gain_multiplier(gain) * ABS_SCALE;
    let channels = prep
        .chans
        .iter()
        .map(|prepared| match prepared {
            None => EncChannelData {
                coded: false,
                envelope: None,
                coefficients: Vec::new(),
            },
            Some(p) => {
                let mut any = false;
                let coefficients: Vec<i32> = p
                    .normalised
                    .iter()
                    .map(|v| {
                        let q = (v / divisor).round() as i64;
                        let q = q.clamp(-ceiling, ceiling) as i32;
                        any |= q != 0;
                        q
                    })
                    .collect();
                if any {
                    EncChannelData {
                        coded: true,
                        envelope: Some(EncEnvelope::Exponents(p.exponents.clone())),
                        coefficients,
                    }
                } else {
                    EncChannelData {
                        coded: false,
                        envelope: None,
                        coefficients: Vec::new(),
                    }
                }
            }
        })
        .collect();
    EncBlockData {
        size_index: prep.size_index,
        joint_stereo: prep.joint_stereo,
        total_gain: gain,
        channels,
    }
}

/// The all-uncoded fallback (rate control found nothing feasible).
fn silent_block(prep: &PreparedBlock) -> EncBlockData {
    EncBlockData {
        size_index: prep.size_index,
        joint_stereo: false,
        total_gain: 1,
        channels: prep
            .chans
            .iter()
            .map(|_| EncChannelData {
                coded: false,
                envelope: None,
                coefficients: Vec::new(),
            })
            .collect(),
    }
}

/// Forward lapped transform (bare cosine bank), the mirror of
/// `vendor_decode::inverse_transform`: the fast staged-set path where
/// the size is typed, a direct evaluation of the same oddly-stacked
/// basis otherwise.
pub(crate) fn forward_transform(windowed: &[f64]) -> Vec<f64> {
    let two_m = windowed.len();
    let m = two_m / 2;
    if let Ok(bs) = crate::block::BlockSize::from_samples(m as u16) {
        let mlt = crate::mlt::Mlt::new(bs);
        return mlt
            .forward(windowed)
            .expect("length matches by construction");
    }
    let mut out = vec![0.0; m];
    for (k, slot) in out.iter_mut().enumerate() {
        let mut acc = 0.0;
        for (n, &x) in windowed.iter().enumerate() {
            let angle = std::f64::consts::PI / m as f64
                * (n as f64 + 0.5 + m as f64 / 2.0)
                * (k as f64 + 0.5);
            acc += x * angle.cos();
        }
        *slot = acc;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::Version;
    use crate::packet::PacketAssembler;
    use crate::vendor_decode::BlockSynth;
    use crate::vendor_frame::FrameParser;

    /// Decode packets with the crate's own chain (mirroring the
    /// vendor test harness) into per-channel PCM.
    fn decode_packets(cfg: &StreamConfig, packets: &[Vec<u8>]) -> Vec<Vec<f64>> {
        let mut asm = PacketAssembler::new(cfg);
        for p in packets {
            asm.push_packet(p).unwrap();
        }
        let stream = asm.finish();
        let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
        let mut parser = FrameParser::new(cfg, &body_starts);
        let mut synth = BlockSynth::new(cfg);
        let mut pcm: Vec<Vec<f64>> = vec![Vec::new(); usize::from(cfg.channels)];
        let mut cursor = stream.packets[0].frames_start_bit();
        for (i, rec) in stream.packets.iter().enumerate() {
            if cursor != rec.frames_start_bit() {
                cursor = rec.frames_start_bit();
                parser.raise_latch();
            }
            let mut reader = stream.reader_at(cursor);
            for f in 0..rec.header.frame_count {
                let frame = parser
                    .parse_frame(&mut reader)
                    .unwrap_or_else(|e| panic!("packet {i} frame {f}: {e}"));
                for block in &frame.blocks {
                    for (ch, chan) in synth.block(block).into_iter().enumerate() {
                        pcm[ch].extend_from_slice(&chan);
                    }
                }
            }
            cursor = reader.position() as u64;
        }
        for (ch, chan) in synth.flush().into_iter().enumerate() {
            pcm[ch].extend_from_slice(&chan);
        }
        pcm
    }

    /// SNR of `decoded` against `original` at the chain's fixed
    /// `frame_length / 2` lead-in.
    fn snr_db(cfg: &StreamConfig, original: &[f64], decoded: &[f64]) -> f64 {
        let lead = usize::from(cfg.frame_length) / 2;
        let n = original.len().min(decoded.len().saturating_sub(lead));
        assert!(n > 0, "no overlap");
        let (mut sig, mut err) = (0.0f64, 0.0f64);
        for t in 0..n {
            let a = original[t];
            let b = decoded[t + lead];
            sig += a * a;
            err += (a - b) * (a - b);
        }
        10.0 * (sig / err.max(1e-30)).log10()
    }

    /// Rich but band-limited test material: a few inharmonic tones
    /// with slow amplitude drift.
    fn material(sample_rate: u32, len: usize, seed: u64) -> Vec<f64> {
        let mut out = vec![0.0f64; len];
        let freqs = [211.0, 487.0, 1021.0, 2333.0];
        for (i, f) in freqs.iter().enumerate() {
            let amp = 0.12 / (i + 1) as f64;
            let phase = (seed as f64) * 0.37 + i as f64;
            for (t, o) in out.iter_mut().enumerate() {
                let drift = 1.0
                    + 0.3
                        * (2.0 * std::f64::consts::PI * 0.7 * t as f64 / sample_rate as f64
                            + phase)
                            .sin();
                *o += amp
                    * drift
                    * (2.0 * std::f64::consts::PI * f * t as f64 / sample_rate as f64 + phase)
                        .sin();
            }
        }
        out
    }

    #[test]
    fn forward_transform_is_the_tdac_inverse_of_the_decode_chain() {
        // Pure transform identity, no quantisation: window → forward
        // → (2/M) inverse → window → overlap-add over equal-size
        // blocks reconstructs the interior exactly.
        let m = 256usize;
        let n_blocks = 6;
        let signal: Vec<f64> = (0..m * n_blocks)
            .map(|t| ((t * 37 + 11) % 101) as f64 / 50.5 - 1.0)
            .collect();
        let sample = |i: i64| -> f64 {
            if i < 0 || i as usize >= signal.len() {
                0.0
            } else {
                signal[i as usize]
            }
        };
        let mut recon = vec![0.0f64; m * n_blocks + m];
        for b in 0..n_blocks {
            let pos = (b * m) as i64;
            let mut time: Vec<f64> = (0..2 * m as i64)
                .map(|n| sample(pos - (m as i64) / 2 + n))
                .collect();
            crate::vendor_decode::transition_window(&mut time, m, m);
            let spec = forward_transform(&time);
            let mut inv = crate::vendor_decode::inverse_transform(&spec);
            crate::vendor_decode::transition_window(&mut inv, m, m);
            for (n, &v) in inv.iter().enumerate() {
                let abs = pos - (m as i64) / 2 + n as i64;
                if abs >= 0 && (abs as usize) < recon.len() {
                    recon[abs as usize] += v;
                }
            }
        }
        // Interior (past the first half-block, before the last).
        for t in m / 2..m * n_blocks - m / 2 {
            assert!(
                (recon[t] - signal[t]).abs() < 1e-9,
                "t={t}: {} vs {}",
                recon[t],
                signal[t]
            );
        }
    }

    #[test]
    fn mixed_size_tdac_identity_holds_across_transitions() {
        // 512 → 128 → 128 → 512 with neighbour-matched windows.
        let sizes = [512usize, 128, 128, 512, 256, 512];
        let total: usize = sizes.iter().sum();
        let signal: Vec<f64> = (0..total)
            .map(|t| ((t * 29 + 5) % 97) as f64 / 48.5 - 1.0)
            .collect();
        let sample = |i: i64| -> f64 {
            if i < 0 || i as usize >= signal.len() {
                0.0
            } else {
                signal[i as usize]
            }
        };
        let mut recon = vec![0.0f64; total + 1024];
        let mut pos = 0i64;
        for (bi, &m) in sizes.iter().enumerate() {
            let prev = if bi == 0 { m } else { sizes[bi - 1] };
            let next = if bi + 1 == sizes.len() {
                m
            } else {
                sizes[bi + 1]
            };
            let mut time: Vec<f64> = (0..2 * m as i64)
                .map(|n| sample(pos - (m as i64) / 2 + n))
                .collect();
            crate::vendor_decode::transition_window(&mut time, prev, next);
            let spec = forward_transform(&time);
            let mut inv = crate::vendor_decode::inverse_transform(&spec);
            crate::vendor_decode::transition_window(&mut inv, prev, next);
            for (n, &v) in inv.iter().enumerate() {
                let abs = pos - (m as i64) / 2 + n as i64;
                if abs >= 0 && (abs as usize) < recon.len() {
                    recon[abs as usize] += v;
                }
            }
            pos += m as i64;
        }
        for t in 256..total - 256 {
            assert!(
                (recon[t] - signal[t]).abs() < 1e-9,
                "t={t}: {} vs {}",
                recon[t],
                signal[t]
            );
        }
    }

    #[test]
    fn mono_encode_decode_reaches_transparent_snr_at_generous_bitrate() {
        // Mono 44.1 kHz at a high bitrate: the quantiser, not the
        // budget, limits fidelity. The wire format itself bounds the
        // peak-relative resolution: a loud block's feasible gains all
        // fall in the w_lvl = 9 tier (escape ceiling 511), so ~9 bits
        // of peak dynamic range plus the 25-band envelope shaping is
        // the format's own ceiling — the same 18-27 dB envelope the
        // vendor streams measure against the black-box reference.
        let cfg = StreamConfig::derive(Version::V2, 44_100, 1, 24_000, 8918, 0x0003).unwrap();
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        let signal = material(44_100, 44_100, 1);
        enc.push(std::slice::from_ref(&signal)).unwrap();
        let packets = enc.finish().unwrap();
        assert!(!packets.is_empty());
        let decoded = decode_packets(&cfg, &packets);
        let snr = snr_db(&cfg, &signal, &decoded[0]);
        assert!(snr > 24.0, "mono high-rate SNR {snr:.2} dB");
    }

    #[test]
    fn stereo_vbl_encode_decode_round_trips_with_joint_blocks() {
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 8000, 1488, 0x0017).unwrap();
        assert!(cfg.vbl_enabled);
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        let left = material(22_050, 33_075, 2);
        // Highly correlated right channel: Auto picks joint.
        let right: Vec<f64> = left.iter().map(|v| v * 0.9).collect();
        enc.push(&[left.clone(), right.clone()]).unwrap();
        let packets = enc.finish().unwrap();
        let decoded = decode_packets(&cfg, &packets);
        let snr_l = snr_db(&cfg, &left, &decoded[0]);
        let snr_r = snr_db(&cfg, &right, &decoded[1]);
        assert!(snr_l > 18.0, "left SNR {snr_l:.2} dB");
        assert!(snr_r > 18.0, "right SNR {snr_r:.2} dB");
    }

    #[test]
    fn transient_material_splits_blocks_under_auto_policy() {
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 8000, 1488, 0x0017).unwrap();
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        let n = 8192usize;
        let mut left = vec![0.0f64; n];
        // A click train: sharp attacks inside otherwise quiet frames.
        for (i, v) in left.iter_mut().enumerate() {
            if i % 1500 > 1400 {
                *v = 0.7 * (((i * 13) % 7) as f64 / 3.5 - 1.0);
            }
        }
        let right = left.clone();
        // The schedule probe must split at least one frame.
        enc.push(&[left, right]).unwrap();
        let any_split =
            (0..(n as u64 / u64::from(cfg.frame_length))).any(|k| enc.schedule(k).len() > 1);
        assert!(any_split, "transient frames must split");
        let packets = enc.finish().unwrap();
        // And the result still decodes cleanly through the own chain.
        let decoded = decode_packets(&cfg, &packets);
        assert!(decoded[0].iter().all(|v| v.is_finite()));
    }

    #[test]
    fn low_bitrate_frames_respect_the_s1_bounds() {
        // The staged cand_stereo22k geometry: 744-byte packets,
        // 2047-bit frame cap. Dense material must still emit legal
        // frames (rate control coarsens).
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap();
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        let left = material(22_050, 22_050, 3);
        let right = material(22_050, 22_050, 4);
        enc.push(&[left.clone(), right]).unwrap();
        let packets = enc.finish().unwrap();
        // Every packet parses and every boundary closes (the decode
        // harness asserts internally via unwraps).
        let decoded = decode_packets(&cfg, &packets);
        let snr = snr_db(&cfg, &left, &decoded[0]);
        assert!(snr > 5.0, "32 kbps stereo SNR {snr:.2} dB");
        // Rough rate sanity: within 3x of the nominal bitrate.
        let seconds = 22_050.0 / 22_050.0;
        let bytes: usize = packets.iter().map(|p| p.len()).sum();
        assert!(
            (bytes as f64) < 3.0 * seconds * 4006.0 + 2.0 * 744.0,
            "{bytes} bytes"
        );
    }

    #[test]
    fn silence_encodes_to_uncoded_blocks_and_decodes_to_silence() {
        let cfg = StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap();
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        enc.push(&[vec![0.0; 4096]]).unwrap();
        let packets = enc.finish().unwrap();
        let decoded = decode_packets(&cfg, &packets);
        assert!(decoded[0].iter().all(|&v| v.abs() < 1e-12));
    }

    #[test]
    fn bad_input_and_policy_are_typed_errors() {
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap();
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        assert!(matches!(
            enc.push(&[vec![0.0; 10]]),
            Err(EncodeError::BadInput { .. })
        ));
        assert!(matches!(
            enc.push(&[vec![0.0; 10], vec![0.0; 11]]),
            Err(EncodeError::BadInput { .. })
        ));
        assert!(matches!(
            VendorEncoder::with_settings(
                &cfg,
                EncoderSettings {
                    blocks: BlockPolicy::Fixed(7),
                    ..EncoderSettings::default()
                }
            ),
            Err(EncodeError::BadPolicyIndex { index: 7 })
        ));
        // LSP-path configuration refused.
        let lsp = StreamConfig::derive(Version::V2, 8000, 1, 1000, 640, 0x0026).unwrap();
        assert!(matches!(
            VendorEncoder::new(&lsp),
            Err(EncodeError::Emit(EmitError::LspPathUnsupported))
        ));
    }

    #[test]
    fn no_reservoir_profile_encodes_one_frame_per_packet() {
        // The ACM catalogue's headerless geometry (flags2 = 0x0001).
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 4005, 186, 0x0001).unwrap();
        assert!(!cfg.bit_reservoir);
        let mut enc = VendorEncoder::new(&cfg).unwrap();
        let left = material(22_050, 8192, 7);
        let right: Vec<f64> = left.iter().map(|v| v * 0.8).collect();
        enc.push(&[left.clone(), right]).unwrap();
        let packets = enc.finish().unwrap();
        // frames = ceil(8192 / 1024) = 8 packets of 186 bytes.
        assert_eq!(packets.len(), 8);
        assert!(packets.iter().all(|p| p.len() == 186));
        let decoded = decode_packets(&cfg, &packets);
        let snr = snr_db(&cfg, &left, &decoded[0]);
        assert!(snr > 3.0, "tiny-packet profile SNR {snr:.2} dB");
    }
}

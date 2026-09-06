//! Vendor-wire **encoder** mirror — the §2–§4 frame/block bit
//! emitter and the §1 packet (superframe) writer, field-for-field
//! inverses of [`crate::vendor_frame::FrameParser`] and
//! [`crate::packet::PacketAssembler`].
//!
//! ## Source
//!
//! Everything here is the emission mirror of the staged decode-path
//! layout (`docs/audio/wma/frame-bit-layout.md`) as realised by this
//! crate's own vendor parser, calibrations included:
//!
//! * §1 — the packet header (P1 sequence mod 16, P2 frames-in-packet,
//!   P3 reservoir carry at `byte_offset_bits + 3` bits) and the
//!   bit-reservoir carry: frames are laid back-to-back in one
//!   continuous body bitstream and the packetiser derives each
//!   packet's P2/P3 from where the frame boundaries actually fell.
//!   The r446 **zero-carry padding** semantic is the emitter's
//!   flush mechanism: padding the current packet's body and starting
//!   the next frame at the next packet's body start (P3 = 0) is how
//!   a §1 stream pads — there is no in-frame padding field.
//! * §2 — the F1 one-ahead pipeline (each block's field carries the
//!   *next* block's size index) with the **three-field opening**
//!   (previous, current, next) on the first block of the first frame
//!   starting in a packet; F2a the joint-stereo flag before the
//!   channel flags; F2 the per-channel coded flags; B1 the total-gain
//!   accumulator (7-bit fields, `0x7f` extends); B2 the per-block
//!   envelope-reuse bit on short blocks of two-channel VBL streams
//!   (the vendor-measured [`crate::vendor_frame::ReuseRule`]
//!   default).
//! * §3 — exponent deltas via the staged 121-symbol scale VLC
//!   (`symbol = delta + 60`, initial predictor 36; v1: the 5-bit
//!   absolute base `exponent[0] − 10`, deltas from band 1).
//! * §4 — the coefficient run-level sub-stream over the staged vendor
//!   codes ([`crate::wire_vlc`]): companion-map pairs where the
//!   `(run, |level|)` pair is tabulated ([`crate::wire_vlc::runlevel_index`]),
//!   the escape (symbol 0 + literal `|level|` at the gain-mapped
//!   width + run at `frame_length_bits` + sign) otherwise, EOB
//!   (symbol 1) for a trailing zero tail, one sign bit per non-EOB
//!   event (1 = positive), and the **channel-scoped ALT tree** in
//!   joint blocks (second coded channel only — the r439
//!   calibration).
//!
//! The emitter tracks the same latch state as the parser: the §2
//! three-field opening is required exactly when a packet-body
//! boundary was crossed at or before the frame's start bit, and a
//! padding flush mirrors the decoder's cursor-jump resync
//! (`raise_latch`), which also restarts the F1 pipeline.
//!
//! ## Frame-size bounds
//!
//! Two hard §1 bounds fall out of the packet-header field widths and
//! are enforced per frame ([`EmitError::FrameTooLong`]):
//!
//! * a frame must fit inside one packet body (`≤ packet_body_bits`),
//!   so that every packet contains at least one frame start and the
//!   carry never spans a whole packet;
//! * a frame must fit the P3 carry field (`< 2^(byte_offset_bits+3)`),
//!   since a frame that straddles a boundary puts (almost) its whole
//!   length into the next packet's carry.
//!
//! The §3.1 LSP envelope path is **not emittable**: its index →
//! envelope conversion tables are a staged gap, so an encoder cannot
//! choose indices for a target envelope. Streams with `flags2` bit 0
//! clear are refused at construction ([`EmitError::LspPathUnsupported`]).

use crate::bitio::{BitReader, BitWriter};
use crate::header::Version;
use crate::stream_config::StreamConfig;
use crate::vendor_frame::{
    escape_level_width, measured_noise_policy, NoiseGrid, NoiseSpec, ReuseRule,
};
use crate::wire_vlc::{coef_vlc, runlevel_index, scale_vlc};

/// A channel's envelope, encoder side. The §3.1 LSP form is absent
/// by design (module docs).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EncEnvelope {
    /// §3 fresh VLC-delta exponents, one per band of the block's
    /// partition.
    Exponents(Vec<i32>),
    /// §2 B2 = 0 — reuse the envelope cached for this block-size
    /// index. Only valid on blocks that carry the B2 bit, and only
    /// when **every** coded channel reuses (B2 is per block).
    Reuse,
}

/// One channel's share of a block to emit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncChannelData {
    /// F2 — whether the channel codes anything this block.
    pub coded: bool,
    /// The envelope (required iff coded).
    pub envelope: Option<EncEnvelope>,
    /// Quantised coefficients on the coded axis: exactly
    /// `coef_end − coef_start` entries for a coded channel (entry `i`
    /// is spectral bin `coef_start + i`), empty otherwise.
    pub coefficients: Vec<i32>,
}

/// One §2 block to emit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncBlockData {
    /// F1 size index (`block_size = frame_length >> index`); 0 for a
    /// fixed-block stream.
    pub size_index: u8,
    /// F2a — joint (mid/side) stereo; must be `false` for mono.
    pub joint_stereo: bool,
    /// B1 — total gain (≥ 1; the accumulator starts at 1).
    pub total_gain: u32,
    /// Per-channel data, one entry per stream channel.
    pub channels: Vec<EncChannelData>,
}

/// Emission failures. Everything is validated before any state
/// mutates — a failed frame leaves the writer untouched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmitError {
    /// The stream's `flags2` bit 0 is clear: the §3.1 LSP envelope
    /// conversion tables are a staged gap, so no envelope can be
    /// chosen (module docs).
    LspPathUnsupported,
    /// A size index outside the configured block-size count.
    BadBlockSizeIndex {
        /// The offending index.
        index: u8,
    },
    /// The frame's block sizes do not sum to `frame_length`.
    FrameSizeMismatch {
        /// Sum of the frame's block sizes.
        total: u32,
        /// The stream's frame length.
        frame_length: u16,
    },
    /// A block's channel count differs from the stream's.
    WrongChannelCount {
        /// The block's channel-entry count.
        got: usize,
        /// The stream's channel count.
        expected: u8,
    },
    /// F2a set on a mono stream.
    JointStereoOnMono,
    /// B1 total gain of 0 (the accumulator starts at 1).
    ZeroTotalGain,
    /// A coded channel without an envelope, or an uncoded channel
    /// with one.
    EnvelopeMismatch,
    /// `Reuse` on a block that carries no B2 bit, or a mix of fresh
    /// and reused envelopes on one block (B2 is per block).
    BadReuse,
    /// An envelope whose band count differs from the block's
    /// partition.
    WrongBandCount {
        /// The envelope's band count.
        got: usize,
        /// The block partition's band count.
        expected: usize,
    },
    /// A v1 envelope base outside the 5-bit field
    /// (`exponent[0] − 10 ∉ [0, 31]`).
    BaseOutOfRange {
        /// The offending first exponent.
        exponent: i32,
    },
    /// An exponent delta outside the scale VLC's ±60 range.
    DeltaOutOfRange {
        /// Band index of the offending delta.
        band: usize,
        /// The delta.
        delta: i32,
    },
    /// A coded channel's coefficient count differs from the block's
    /// `coef_end − coef_start`.
    WrongCoefficientCount {
        /// The channel's count.
        got: usize,
        /// The block's coded-axis width.
        expected: u16,
    },
    /// A |level| above the escape ceiling for the block's total gain
    /// (`2^w_lvl − 1`).
    LevelTooLarge {
        /// The offending absolute level.
        level: u32,
        /// The ceiling.
        max: u32,
    },
    /// The frame exceeds a §1 bound (module docs).
    FrameTooLong {
        /// The frame's emitted size.
        bits: u64,
        /// The applicable bound.
        max_bits: u64,
    },
    /// A non-reservoir stream can only hold one frame per packet.
    /// (Internal misuse guard; unreachable through
    /// [`VendorBitWriter::write_frame`].)
    PacketOverflow,
    /// More than 15 frames started in one packet (P2 is 4 bits).
    /// (Internal misuse guard; unreachable through
    /// [`VendorBitWriter::write_frame`], which pads first.)
    TooManyFrameStarts,
}

impl core::fmt::Display for EmitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EmitError::LspPathUnsupported => f.write_str(
                "oxideav-wma: flags2 bit 0 clear selects the LSP envelope path, \
                 whose conversion tables are a staged gap (not encodable)",
            ),
            EmitError::BadBlockSizeIndex { index } => {
                write!(f, "oxideav-wma: block-size index {index} out of range")
            }
            EmitError::FrameSizeMismatch {
                total,
                frame_length,
            } => write!(
                f,
                "oxideav-wma: frame blocks sum to {total} samples, frame length is {frame_length}"
            ),
            EmitError::WrongChannelCount { got, expected } => {
                write!(f, "oxideav-wma: block has {got} channels, stream has {expected}")
            }
            EmitError::JointStereoOnMono => {
                f.write_str("oxideav-wma: joint-stereo flag on a mono stream")
            }
            EmitError::ZeroTotalGain => f.write_str("oxideav-wma: total gain must be >= 1"),
            EmitError::EnvelopeMismatch => {
                f.write_str("oxideav-wma: envelope presence must match the coded flag")
            }
            EmitError::BadReuse => f.write_str(
                "oxideav-wma: envelope reuse requires the B2 bit and must cover every coded channel",
            ),
            EmitError::WrongBandCount { got, expected } => {
                write!(f, "oxideav-wma: envelope has {got} bands, partition has {expected}")
            }
            EmitError::BaseOutOfRange { exponent } => {
                write!(f, "oxideav-wma: v1 envelope base {exponent} outside [10, 41]")
            }
            EmitError::DeltaOutOfRange { band, delta } => {
                write!(f, "oxideav-wma: exponent delta {delta} at band {band} outside +/-60")
            }
            EmitError::WrongCoefficientCount { got, expected } => {
                write!(f, "oxideav-wma: {got} coefficients, block codes {expected}")
            }
            EmitError::LevelTooLarge { level, max } => {
                write!(f, "oxideav-wma: |level| {level} above the escape ceiling {max}")
            }
            EmitError::FrameTooLong { bits, max_bits } => {
                write!(f, "oxideav-wma: {bits}-bit frame exceeds the {max_bits}-bit s1 bound")
            }
            EmitError::PacketOverflow => {
                f.write_str("oxideav-wma: one frame per packet without the bit reservoir")
            }
            EmitError::TooManyFrameStarts => {
                f.write_str("oxideav-wma: more than 15 frames started in one packet")
            }
        }
    }
}

impl std::error::Error for EmitError {}

/// Stateful §2 frame emitter — the mirror of
/// [`crate::vendor_frame::FrameParser`]'s latch / F1-pipeline state
/// machine. It never owns the output writer: each frame is emitted
/// into a caller-supplied [`BitWriter`] at a caller-stated absolute
/// body-bit position (which is what makes dry runs possible — clone
/// the emitter, emit into a scratch writer at the same position).
#[derive(Debug, Clone)]
pub struct FrameEmitter {
    cfg: StreamConfig,
    /// §2 B2 presence rule (mirror of the parser's `with_reuse`).
    reuse: ReuseRule,
    /// §2.1 noise-substitution emission hypothesis (mirror of the
    /// parser's `with_noise`): when set, every coded channel emits
    /// the F3 band-flag walk — all-clear (this encoder never
    /// substitutes a band, so no F4 gains follow), which is the
    /// wire-legal "no noise in this block" spelling a
    /// noise-enabled decoder expects.
    noise: Option<NoiseSpec>,
    /// The §2 three-field latch, raised at stream start, on every
    /// packet-body boundary crossing, and by [`Self::raise_latch`].
    latch: bool,
    /// The F1 pipeline: the size index the previous block promised
    /// as "next".
    promised_next: Option<u8>,
    /// The last emitted block's size index (the three-field
    /// opening's "previous" value).
    last_size_index: Option<u8>,
    /// Absolute bit position of the next uncrossed packet-body
    /// boundary.
    next_boundary: u64,
}

impl FrameEmitter {
    /// A fresh emitter for one stream configuration.
    ///
    /// # Errors
    ///
    /// [`EmitError::LspPathUnsupported`] when `flags2` bit 0 is clear
    /// (module docs).
    pub fn new(cfg: &StreamConfig) -> Result<Self, EmitError> {
        if !cfg.exp_vlc {
            return Err(EmitError::LspPathUnsupported);
        }
        let (noise, reuse) = match measured_noise_policy(cfg) {
            Some((spec, rule)) => (Some(spec), rule),
            None => (None, ReuseRule::default()),
        };
        Ok(Self {
            cfg: cfg.clone(),
            reuse,
            noise,
            latch: true,
            promised_next: None,
            last_size_index: None,
            next_boundary: u64::from(cfg.packet_body_bits()),
        })
    }

    /// Enable the §2.1 noise-substitution emission hypothesis
    /// (mirror of [`crate::vendor_frame::FrameParser::with_noise`]).
    pub fn with_noise(mut self, noise: NoiseSpec) -> Self {
        self.noise = Some(noise);
        self
    }

    /// Select the §2 B2 envelope-reuse presence rule (mirror of
    /// [`crate::vendor_frame::FrameParser::with_reuse`]).
    pub fn with_reuse(mut self, reuse: ReuseRule) -> Self {
        self.reuse = reuse;
        self
    }

    /// Disable the §2.1 sub-stream (no F3/F4 bits at all), whatever
    /// the measured policy says — a measurement hook.
    pub fn without_noise(mut self) -> Self {
        self.noise = None;
        self
    }

    /// Mirror of the parser's `raise_latch`: force the three-field
    /// opening on the next frame and restart the F1 pipeline — the
    /// state a decoder reaches after a padding skip (§1 zero-carry).
    pub fn raise_latch(&mut self) {
        self.latch = true;
        self.promised_next = None;
        self.last_size_index = None;
    }

    /// Mirror of the parser's per-frame boundary watch.
    fn update_latch(&mut self, pos: u64) {
        let body = u64::from(self.cfg.packet_body_bits());
        while self.next_boundary <= pos {
            self.latch = true;
            self.next_boundary += body;
        }
    }

    /// Emit one frame at absolute body position `start_pos` into
    /// `out`. `next_frame_first` is the first block-size index of the
    /// **next** frame (the F1 one-ahead pipeline needs it); `None`
    /// at stream end repeats the final block's own index into the
    /// dangling pipeline field.
    ///
    /// # Errors
    ///
    /// [`EmitError`] — nothing is written on failure only if the
    /// caller uses a scratch writer (see [`VendorBitWriter`], which
    /// always does).
    pub fn emit_frame(
        &mut self,
        out: &mut BitWriter,
        start_pos: u64,
        blocks: &[EncBlockData],
        next_frame_first: Option<u8>,
    ) -> Result<(), EmitError> {
        self.update_latch(start_pos);
        let mut total: u32 = 0;
        for b in blocks {
            total += u32::from(self.block_size(b.size_index)?);
        }
        if total != u32::from(self.cfg.frame_length) {
            return Err(EmitError::FrameSizeMismatch {
                total,
                frame_length: self.cfg.frame_length,
            });
        }
        for (bi, block) in blocks.iter().enumerate() {
            let next_index = match blocks.get(bi + 1) {
                Some(b) => b.size_index,
                None => next_frame_first.unwrap_or(block.size_index),
            };
            self.emit_block(out, block, next_index)?;
        }
        Ok(())
    }

    fn block_size(&self, index: u8) -> Result<u16, EmitError> {
        self.cfg
            .block_size_for_index(index)
            .ok_or(EmitError::BadBlockSizeIndex { index })
    }

    fn emit_block(
        &mut self,
        out: &mut BitWriter,
        block: &EncBlockData,
        next_index: u8,
    ) -> Result<(), EmitError> {
        let block_size = self.block_size(block.size_index)?;
        let channels = usize::from(self.cfg.channels);
        if block.channels.len() != channels {
            return Err(EmitError::WrongChannelCount {
                got: block.channels.len(),
                expected: self.cfg.channels,
            });
        }

        // F1 — pipelined size indices (VBL streams only).
        if self.cfg.vbl_enabled {
            self.block_size(next_index)?;
            if self.latch {
                let prev = self.last_size_index.unwrap_or(block.size_index);
                out.write_bits(u64::from(prev), self.cfg.w_bs);
                out.write_bits(u64::from(block.size_index), self.cfg.w_bs);
            } else {
                // The pipeline promised this block's size one block
                // ago; a mismatch is a driver bug (the parser would
                // decode the promised size, not this one).
                debug_assert_eq!(self.promised_next, Some(block.size_index));
            }
            self.latch = false;
            out.write_bits(u64::from(next_index), self.cfg.w_bs);
            self.promised_next = Some(next_index);
        }
        self.last_size_index = Some(block.size_index);

        // F2a — joint-stereo flag (two-channel streams only).
        if channels == 2 {
            out.write_bit(block.joint_stereo);
        } else if block.joint_stereo {
            return Err(EmitError::JointStereoOnMono);
        }

        // F2 — channel-coded flags.
        for ch in &block.channels {
            out.write_bit(ch.coded);
        }
        if block.channels.iter().all(|c| !c.coded) {
            return Ok(()); // §2: the block ends here.
        }

        // B1 — total gain: accumulator starts at 1, 0x7f extends.
        if block.total_gain == 0 {
            return Err(EmitError::ZeroTotalGain);
        }
        let mut rem = block.total_gain - 1;
        while rem >= 0x7f {
            out.write_bits(0x7f, 7);
            rem -= 0x7f;
        }
        out.write_bits(u64::from(rem), 7);

        // F3 — noise-substitution band flags (hypothesis-gated, all
        // channels' flags before all channels' gains; this encoder
        // flags nothing, so no F4 gains follow). The walk mirrors
        // the parser's exactly.
        if let Some(spec) = self.noise {
            let walk_edges: Vec<u16> = match spec.grid {
                NoiseGrid::ExponentBands => {
                    crate::band_partition::exponent_band_edges(self.cfg.sample_rate, block_size)
                }
                NoiseGrid::OctaveSubbands => {
                    crate::vendor_frame::octave_noise_edges_for(self.cfg.sample_rate, block_size)
                }
            };
            let walk_count = walk_edges.len() - 1;
            let coef_end = self.cfg.coef_end(block_size);
            for ch in &block.channels {
                if !ch.coded {
                    continue;
                }
                let mut band = crate::vendor_frame::noise_walk_start(
                    &spec.start,
                    &walk_edges,
                    walk_count,
                    block_size,
                    self.cfg.sample_rate,
                );
                while band < walk_count && walk_edges[band] < coef_end {
                    out.write_bit(false);
                    band += 1;
                }
            }
        }

        // B2 — the vendor-measured per-block reuse bit
        // (ReuseRule::TwoChannelShortBlock).
        let b2_short = self.cfg.vbl_enabled && block_size < self.cfg.frame_length;
        let b2_present = match self.reuse {
            ReuseRule::TwoChannelShortBlock => b2_short && channels == 2,
            ReuseRule::ShortBlockPerBlock => b2_short,
            ReuseRule::Never => false,
        };
        let reuse_flags: Vec<bool> = block
            .channels
            .iter()
            .filter(|c| c.coded)
            .map(|c| matches!(c.envelope, Some(EncEnvelope::Reuse)))
            .collect();
        let reuse = reuse_flags.iter().any(|&r| r);
        if reuse && (!b2_present || !reuse_flags.iter().all(|&r| r)) {
            return Err(EmitError::BadReuse);
        }
        if b2_present {
            out.write_bit(!reuse); // 1 = fresh envelopes follow.
        }

        // B3/B4 — envelopes per coded channel (skipped on reuse).
        let edges = crate::band_partition::exponent_band_edges(self.cfg.sample_rate, block_size);
        let band_count = edges.len() - 1;
        for ch in &block.channels {
            match (&ch.envelope, ch.coded) {
                (Some(_), true) => {}
                (None, false) => continue,
                _ => return Err(EmitError::EnvelopeMismatch),
            }
            let exponents = match ch.envelope.as_ref() {
                Some(EncEnvelope::Exponents(e)) => e,
                Some(EncEnvelope::Reuse) => continue, // covered by B2 = 0
                None => unreachable!("checked above"),
            };
            if reuse {
                // A fresh envelope alongside a reused one.
                return Err(EmitError::BadReuse);
            }
            if exponents.len() != band_count {
                return Err(EmitError::WrongBandCount {
                    got: exponents.len(),
                    expected: band_count,
                });
            }
            let (mut prev, start_band) = match self.cfg.version {
                Version::V1 => {
                    let base = exponents[0] - 10;
                    if !(0..=31).contains(&base) {
                        return Err(EmitError::BaseOutOfRange {
                            exponent: exponents[0],
                        });
                    }
                    out.write_bits(base as u64, 5);
                    (exponents[0], 1usize)
                }
                Version::V2 => (36, 0usize),
            };
            for (band, &e) in exponents.iter().enumerate().skip(start_band) {
                let delta = e - prev;
                if !(-60..=60).contains(&delta) {
                    return Err(EmitError::DeltaOutOfRange { band, delta });
                }
                let ok = scale_vlc().encode_symbol((delta + 60) as usize, out);
                debug_assert!(ok, "symbol {} in the 121-entry table", delta + 60);
                prev = e;
            }
        }

        // B5 — coefficient sub-streams per coded channel; ALT tree on
        // the second coded channel of a joint block (channel-scoped).
        let w_lvl = escape_level_width(block.total_gain);
        let n_coef = self.cfg.coef_end(block_size) - self.cfg.coef_start(block_size);
        for (chi, ch) in block.channels.iter().enumerate() {
            if !ch.coded {
                if !ch.coefficients.is_empty() {
                    return Err(EmitError::WrongCoefficientCount {
                        got: ch.coefficients.len(),
                        expected: 0,
                    });
                }
                continue;
            }
            if ch.coefficients.len() != usize::from(n_coef) {
                return Err(EmitError::WrongCoefficientCount {
                    got: ch.coefficients.len(),
                    expected: n_coef,
                });
            }
            let alt = block.joint_stereo && chi == 1;
            emit_coefficients(
                out,
                self.cfg.vlc_class,
                alt,
                &ch.coefficients,
                w_lvl,
                self.cfg.frame_length_bits,
            )?;
        }
        Ok(())
    }
}

/// §4: emit one channel's run-level coefficient sub-stream — the
/// mirror of the parser's `decode_coefficients`.
fn emit_coefficients(
    out: &mut BitWriter,
    class: u8,
    alt: bool,
    coefficients: &[i32],
    escape_level_bits: u8,
    escape_run_bits: u8,
) -> Result<(), EmitError> {
    let vlc = coef_vlc(class, alt).expect("class validated at stream open");
    let index = runlevel_index(class, alt).expect("class validated at stream open");
    let max_level = (1u32 << escape_level_bits) - 1;
    let mut idx = 0usize;
    for (i, &c) in coefficients.iter().enumerate() {
        if c == 0 {
            continue;
        }
        let run = i - idx;
        let level = c.unsigned_abs();
        if level > max_level {
            return Err(EmitError::LevelTooLarge {
                level,
                max: max_level,
            });
        }
        let pair_symbol = u16::try_from(run)
            .ok()
            .zip(u16::try_from(level).ok())
            .and_then(|(r, l)| index.symbol(r, l));
        match pair_symbol {
            Some(sym) => {
                let ok = vlc.encode_symbol(usize::from(sym), out);
                debug_assert!(ok, "index symbols are in-alphabet");
            }
            None => {
                // Escape: symbol 0, |level|, run, then the sign bit.
                let ok = vlc.encode_symbol(0, out);
                debug_assert!(ok);
                out.write_bits(u64::from(level), escape_level_bits);
                out.write_bits(run as u64, escape_run_bits);
            }
        }
        out.write_bit(c > 0); // §4: 1 = positive.
        idx = i + 1;
    }
    if idx < coefficients.len() {
        let ok = vlc.encode_symbol(1, out); // EOB
        debug_assert!(ok);
    }
    Ok(())
}

/// One recorded frame interval in the body bitstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FrameInterval {
    start: u64,
    end: u64,
}

/// The §1 body-and-packet writer: frames are appended back-to-back
/// into one continuous body bitstream (padding inserted only via the
/// zero-carry mechanism), and [`VendorBitWriter::finish`] derives
/// every packet's P1/P2/P3 from where the frame boundaries fell —
/// the exact inverse of [`crate::packet::PacketAssembler`].
#[derive(Debug, Clone)]
pub struct VendorBitWriter {
    cfg: StreamConfig,
    emitter: FrameEmitter,
    body: BitWriter,
    frames: Vec<FrameInterval>,
}

impl VendorBitWriter {
    /// A writer for one stream configuration.
    ///
    /// # Errors
    ///
    /// [`EmitError::LspPathUnsupported`] (see [`FrameEmitter::new`]).
    pub fn new(cfg: &StreamConfig) -> Result<Self, EmitError> {
        Ok(Self {
            cfg: cfg.clone(),
            emitter: FrameEmitter::new(cfg)?,
            body: BitWriter::new(),
            frames: Vec::new(),
        })
    }

    /// Enable the §2.1 noise-substitution emission hypothesis on the
    /// underlying emitter (all-clear F3 walks; see
    /// [`FrameEmitter::with_noise`]).
    pub fn with_noise(mut self, noise: crate::vendor_frame::NoiseSpec) -> Self {
        self.emitter = self.emitter.clone().with_noise(noise);
        self
    }

    /// Select the §2 B2 presence rule on the underlying emitter.
    pub fn with_reuse(mut self, reuse: crate::vendor_frame::ReuseRule) -> Self {
        self.emitter = self.emitter.clone().with_reuse(reuse);
        self
    }

    /// Disable the §2.1 sub-stream on the underlying emitter (see
    /// [`FrameEmitter::without_noise`]).
    pub fn without_noise(mut self) -> Self {
        self.emitter = self.emitter.clone().without_noise();
        self
    }

    /// Current absolute body-bit position.
    pub fn position(&self) -> u64 {
        self.body.bit_len() as u64
    }

    fn body_bits(&self) -> u64 {
        u64::from(self.cfg.packet_body_bits())
    }

    /// The §1 per-frame bound (module docs): a frame must fit one
    /// packet body and the P3 carry field.
    pub fn max_frame_bits(&self) -> u64 {
        let body = self.body_bits();
        if self.cfg.bit_reservoir {
            body.min((1u64 << self.cfg.carry_field_bits()) - 1)
        } else {
            body
        }
    }

    /// Frames already started in the packet that `pos` falls in.
    fn starts_in_packet_of(&self, pos: u64) -> usize {
        let body = self.body_bits();
        let lo = (pos / body) * body;
        let hi = lo + body;
        let a = self.frames.partition_point(|f| f.start < lo);
        let b = self.frames.partition_point(|f| f.start < hi);
        b - a
    }

    /// Pad the body with zero bits to the next packet boundary and
    /// re-raise the emitter latch — the §1 zero-carry padding flush
    /// (the decoder resyncs with a cursor jump, which restarts the
    /// F1 pipeline; the emitter mirrors that).
    fn pad_to_boundary(&mut self) {
        let body = self.body_bits();
        let rem = self.position() % body;
        if rem != 0 {
            let mut left = body - rem;
            while left > 0 {
                let step = left.min(32) as u8;
                self.body.write_bits(0, step);
                left -= u64::from(step);
            }
        }
        self.emitter.raise_latch();
    }

    /// Measure a frame's emitted size in bits without committing it —
    /// the encoder's rate-control probe. State is untouched.
    pub fn trial_frame_bits(
        &self,
        blocks: &[EncBlockData],
        next_frame_first: Option<u8>,
    ) -> Result<u64, EmitError> {
        let mut emitter = self.emitter.clone();
        let mut scratch = BitWriter::new();
        // Mirror write_frame's padding decision so the latch (and
        // therefore the F1 field count) matches the real emission.
        let mut start = self.position();
        if self.cfg.bit_reservoir && self.starts_in_packet_of(start) >= 15 {
            start = (start / self.body_bits() + 1) * self.body_bits();
            emitter.raise_latch();
        }
        emitter.emit_frame(&mut scratch, start, blocks, next_frame_first)?;
        Ok(scratch.bit_len() as u64)
    }

    /// Emit one frame into the body stream, padding to the next
    /// packet boundary first when required (15 frame starts already
    /// in the current packet — P2 is 4 bits — or, without the
    /// reservoir, always after the previous frame: one frame per
    /// packet).
    ///
    /// # Errors
    ///
    /// [`EmitError`]; the writer is unchanged on failure.
    pub fn write_frame(
        &mut self,
        blocks: &[EncBlockData],
        next_frame_first: Option<u8>,
    ) -> Result<(), EmitError> {
        if self.cfg.bit_reservoir {
            if self.starts_in_packet_of(self.position()) >= 15 {
                self.pad_to_boundary();
            }
        } else if self.position() % self.body_bits() != 0 {
            // §1 degenerate case: one frame per packet, each starting
            // at bit 0 of its packet.
            self.pad_to_boundary();
        }
        let start = self.position();
        // Dry-run into a scratch writer so a failed frame leaves the
        // body untouched, then commit.
        let mut emitter = self.emitter.clone();
        let mut scratch = BitWriter::new();
        emitter.emit_frame(&mut scratch, start, blocks, next_frame_first)?;
        let bits = scratch.bit_len() as u64;
        if bits > self.max_frame_bits() {
            return Err(EmitError::FrameTooLong {
                bits,
                max_bits: self.max_frame_bits(),
            });
        }
        self.emitter = emitter;
        append_bits(&mut self.body, scratch.as_bytes(), scratch.bit_len());
        self.frames.push(FrameInterval {
            start,
            end: start + bits,
        });
        Ok(())
    }

    /// Frames written so far.
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    /// Number of packets the body stream spans so far (including the
    /// final partial one).
    pub fn packet_count(&self) -> usize {
        (self.position().div_ceil(self.body_bits())) as usize
    }

    /// Close the stream: pad the final packet and derive every §1
    /// packet (header + body slice) — `block_align` bytes each.
    ///
    /// # Errors
    ///
    /// [`EmitError::TooManyFrameStarts`] / [`EmitError::PacketOverflow`]
    /// are internal-misuse guards, unreachable through
    /// [`Self::write_frame`].
    pub fn finish(mut self) -> Result<Vec<Vec<u8>>, EmitError> {
        self.pad_to_boundary();
        let body_bits = self.body_bits();
        let n_packets = (self.position() / body_bits) as usize;
        let bytes = self.body.as_bytes().to_vec();
        let mut packets = Vec::with_capacity(n_packets);
        for k in 0..n_packets as u64 {
            let lo = k * body_bits;
            let hi = lo + body_bits;
            let mut w = BitWriter::new();
            if self.cfg.bit_reservoir {
                // Carry: the frame in progress at this packet's body
                // start, if any.
                let before = self.frames.partition_point(|f| f.start < lo);
                let carry = before
                    .checked_sub(1)
                    .map(|i| self.frames[i].end.saturating_sub(lo))
                    .unwrap_or(0);
                debug_assert!(carry < body_bits, "bounded by max_frame_bits");
                let count = {
                    let b = self.frames.partition_point(|f| f.start < hi);
                    b - before
                };
                if count > 15 {
                    return Err(EmitError::TooManyFrameStarts);
                }
                w.write_bits(k & 0xf, 4); // P1
                w.write_bits(count as u64, 4); // P2
                w.write_bits(carry, self.cfg.carry_field_bits()); // P3
            } else {
                // One frame per packet, at bit 0.
                let starts: Vec<&FrameInterval> = self
                    .frames
                    .iter()
                    .filter(|f| f.start >= lo && f.start < hi)
                    .collect();
                if starts.len() != 1 || starts[0].start != lo {
                    return Err(EmitError::PacketOverflow);
                }
            }
            // Body slice [lo, hi).
            let mut r = BitReader::with_bit_len(&bytes, self.position() as usize);
            skip_bits(&mut r, lo);
            let mut left = body_bits;
            while left > 0 {
                let step = left.min(32) as u8;
                let v = r.read_bits(step).expect("inside the padded body");
                w.write_bits(v, step);
                left -= u64::from(step);
            }
            let pkt = w.into_bytes();
            debug_assert_eq!(pkt.len(), usize::from(self.cfg.block_align));
            packets.push(pkt);
        }
        Ok(packets)
    }
}

/// Append `bits` bits of `src` (MSB-first packed) to `dst`.
fn append_bits(dst: &mut BitWriter, src: &[u8], bits: usize) {
    let mut r = BitReader::with_bit_len(src, bits);
    let mut left = bits;
    while left > 0 {
        let step = left.min(32) as u8;
        let v = r.read_bits(step).expect("within bit_len");
        dst.write_bits(v, step);
        left -= usize::from(step);
    }
}

/// Advance a reader by `bits`.
fn skip_bits(r: &mut BitReader<'_>, bits: u64) {
    let mut left = bits;
    while left > 0 {
        let step = left.min(32) as u8;
        let _ = r.read_bits(step);
        left -= u64::from(step);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::PacketAssembler;
    use crate::vendor_frame::{Envelope, FrameParser, ParsedFrame};

    fn stereo_vbl_cfg() -> StreamConfig {
        // The staged cand_stereo22k row: 8 block sizes, w_bs 2.
        StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap()
    }

    fn mono_vbl_cfg() -> StreamConfig {
        // The staged cand_mono22k row: 4 block sizes.
        StreamConfig::derive(Version::V2, 22_050, 1, 2003, 744, 0x000f).unwrap()
    }

    fn flat_envelope(cfg: &StreamConfig, block_size: u16) -> EncEnvelope {
        let bands = crate::band_partition::exponent_band_count(cfg.sample_rate, block_size);
        EncEnvelope::Exponents(vec![36; bands])
    }

    fn shaped_envelope(cfg: &StreamConfig, block_size: u16, seed: i32) -> EncEnvelope {
        let bands = crate::band_partition::exponent_band_count(cfg.sample_rate, block_size);
        EncEnvelope::Exponents(
            (0..bands as i32)
                .map(|b| 30 + ((b * 7 + seed) % 23))
                .collect(),
        )
    }

    fn coded_channel(
        cfg: &StreamConfig,
        block_size: u16,
        envelope: EncEnvelope,
        seed: u64,
    ) -> EncChannelData {
        let n = usize::from(cfg.coef_end(block_size) - cfg.coef_start(block_size));
        let mut coefficients = vec![0i32; n];
        let mut state = seed | 1;
        let mut i = 0usize;
        while i < n {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1442695040888963407);
            let gap = 40 + ((state >> 33) % 80) as usize;
            i += gap;
            if i >= n {
                break;
            }
            let mut level = ((state >> 13) % 300) as i32 - 150;
            if level == 0 {
                level = 7;
            }
            coefficients[i] = level;
            i += 1;
        }
        EncChannelData {
            coded: true,
            envelope: Some(envelope),
            coefficients,
        }
    }

    fn simple_block(cfg: &StreamConfig, size_index: u8, seed: u64) -> EncBlockData {
        let bs = cfg.block_size_for_index(size_index).unwrap();
        let channels = (0..cfg.channels)
            .map(|ch| {
                coded_channel(
                    cfg,
                    bs,
                    shaped_envelope(cfg, bs, i32::from(ch)),
                    seed + u64::from(ch),
                )
            })
            .collect();
        EncBlockData {
            size_index,
            joint_stereo: false,
            total_gain: 40 + (seed % 30) as u32,
            channels,
        }
    }

    /// Decode `packets` with the crate's own §1 + §2 chain, mirroring
    /// the vendor test harness (padding resync via cursor jump), and
    /// assert every carry boundary closes.
    fn decode_and_check(cfg: &StreamConfig, packets: &[Vec<u8>]) -> Vec<ParsedFrame> {
        let mut asm = PacketAssembler::new(cfg);
        for (i, p) in packets.iter().enumerate() {
            let rec = asm
                .push_packet(p)
                .unwrap_or_else(|e| panic!("packet {i}: {e}"));
            assert!(!rec.discontinuity, "packet {i}: sequence break");
        }
        let stream = asm.finish();
        let body_starts: Vec<u64> = stream.packets.iter().map(|p| p.body_start_bit).collect();
        let mut parser = FrameParser::new(cfg, &body_starts);
        let mut frames = Vec::new();
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
                frames.push(frame);
            }
            cursor = reader.position() as u64;
            if let Some(next) = stream.packets.get(i + 1) {
                if next.header.carry_bits > 0 {
                    assert_eq!(
                        cursor,
                        next.frames_start_bit(),
                        "packet {i}: carry boundary must close"
                    );
                } else {
                    assert!(
                        cursor <= next.body_start_bit,
                        "packet {i}: zero-carry padding boundary must close"
                    );
                }
            }
        }
        frames
    }

    fn assert_frames_match(cfg: &StreamConfig, sent: &[Vec<EncBlockData>], got: &[ParsedFrame]) {
        assert_eq!(sent.len(), got.len(), "frame count");
        for (fi, (s, g)) in sent.iter().zip(got.iter()).enumerate() {
            assert_eq!(s.len(), g.blocks.len(), "frame {fi}: block count");
            for (bi, (sb, gb)) in s.iter().zip(g.blocks.iter()).enumerate() {
                let bs = cfg.block_size_for_index(sb.size_index).unwrap();
                assert_eq!(gb.block_size, bs, "frame {fi} block {bi}");
                assert_eq!(gb.joint_stereo, sb.joint_stereo, "frame {fi} block {bi}");
                let any_coded = sb.channels.iter().any(|c| c.coded);
                if any_coded {
                    assert_eq!(gb.total_gain, sb.total_gain, "frame {fi} block {bi}");
                }
                for (ci, (sc, gc)) in sb.channels.iter().zip(gb.channels.iter()).enumerate() {
                    assert_eq!(gc.coded, sc.coded, "frame {fi} block {bi} ch {ci}");
                    if !sc.coded {
                        continue;
                    }
                    match (sc.envelope.as_ref().unwrap(), gc.envelope.as_ref().unwrap()) {
                        (EncEnvelope::Exponents(e), Envelope::Exponents(d)) => {
                            assert_eq!(d, e, "frame {fi} block {bi} ch {ci}: envelope")
                        }
                        (EncEnvelope::Reuse, Envelope::Reused) => {}
                        (s, d) => panic!("frame {fi} block {bi} ch {ci}: {s:?} vs {d:?}"),
                    }
                    assert_eq!(
                        gc.coefficients, sc.coefficients,
                        "frame {fi} block {bi} ch {ci}: coefficients"
                    );
                }
            }
        }
    }

    #[test]
    fn stereo_vbl_frames_round_trip_across_packets() {
        let cfg = stereo_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        // A mix of full and split frames; enough to span several
        // 744-byte packets.
        let mut sent: Vec<Vec<EncBlockData>> = Vec::new();
        for f in 0..24u64 {
            let frame = match f % 3 {
                0 => vec![simple_block(&cfg, 0, f * 17 + 1)],
                1 => vec![
                    simple_block(&cfg, 1, f * 17 + 1),
                    simple_block(&cfg, 1, f * 17 + 5),
                ],
                _ => vec![
                    simple_block(&cfg, 1, f * 17 + 1),
                    simple_block(&cfg, 2, f * 17 + 5),
                    simple_block(&cfg, 2, f * 17 + 9),
                ],
            };
            sent.push(frame);
        }
        for (i, frame) in sent.iter().enumerate() {
            let next_first = sent.get(i + 1).map(|f| f[0].size_index);
            w.write_frame(frame, next_first).unwrap();
        }
        let packets = w.finish().unwrap();
        assert!(
            packets.len() > 2,
            "spans several packets: {}",
            packets.len()
        );
        assert!(packets.iter().all(|p| p.len() == 744));
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
    }

    #[test]
    fn joint_stereo_uses_the_alt_tree_on_channel_1_and_round_trips() {
        let cfg = stereo_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        let mut sent = Vec::new();
        for f in 0..6u64 {
            let mut b = simple_block(&cfg, 0, f * 31 + 3);
            b.joint_stereo = true;
            sent.push(vec![b]);
        }
        for (i, frame) in sent.iter().enumerate() {
            let next_first = sent.get(i + 1).map(|f| f[0].size_index);
            w.write_frame(frame, next_first).unwrap();
        }
        let packets = w.finish().unwrap();
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
        assert!(frames.iter().all(|f| f.blocks[0].joint_stereo));
    }

    #[test]
    fn b2_reuse_skips_envelopes_and_round_trips() {
        let cfg = stereo_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        let bs = cfg.block_size_for_index(1).unwrap();
        // Frame 1: two fresh short blocks. Frame 2: two blocks whose
        // envelopes reuse the size-1 cache.
        let fresh = vec![simple_block(&cfg, 1, 11), simple_block(&cfg, 1, 12)];
        let mut reused_block = simple_block(&cfg, 1, 13);
        for ch in &mut reused_block.channels {
            ch.envelope = Some(EncEnvelope::Reuse);
        }
        let reused = vec![reused_block.clone(), {
            let mut b = simple_block(&cfg, 1, 14);
            for ch in &mut b.channels {
                ch.envelope = Some(EncEnvelope::Reuse);
            }
            b
        }];
        let sent = vec![fresh, reused];
        for (i, frame) in sent.iter().enumerate() {
            let next_first = sent.get(i + 1).map(|f| f[0].size_index);
            w.write_frame(frame, next_first).unwrap();
        }
        let packets = w.finish().unwrap();
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
        let _ = bs;
    }

    #[test]
    fn mono_vbl_round_trips_without_b2_bits() {
        let cfg = mono_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        let mut sent = Vec::new();
        for f in 0..12u64 {
            let frame = if f % 2 == 0 {
                vec![simple_block(&cfg, 0, f * 13 + 1)]
            } else {
                vec![
                    simple_block(&cfg, 1, f * 13 + 1),
                    simple_block(&cfg, 1, f * 13 + 7),
                ]
            };
            sent.push(frame);
        }
        for (i, frame) in sent.iter().enumerate() {
            let next_first = sent.get(i + 1).map(|f| f[0].size_index);
            w.write_frame(frame, next_first).unwrap();
        }
        let packets = w.finish().unwrap();
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
    }

    #[test]
    fn fifteen_frame_cap_pads_via_the_zero_carry_mechanism() {
        let cfg = stereo_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        // Tiny frames: all-uncoded blocks (a handful of bits each) —
        // dozens would start in one packet without the cap.
        let uncoded = EncChannelData {
            coded: false,
            envelope: None,
            coefficients: Vec::new(),
        };
        let tiny = vec![EncBlockData {
            size_index: 0,
            joint_stereo: false,
            total_gain: 1,
            channels: vec![uncoded.clone(), uncoded],
        }];
        let sent: Vec<_> = (0..40).map(|_| tiny.clone()).collect();
        for frame in &sent {
            w.write_frame(frame, Some(0)).unwrap();
        }
        let packets = w.finish().unwrap();
        assert!(packets.len() >= 3, "the cap must force padding flushes");
        // Every packet declares at most 15 frames; total is 40.
        let mut asm = PacketAssembler::new(&cfg);
        let mut total = 0u32;
        for p in &packets {
            let rec = asm.push_packet(p).unwrap();
            assert!(rec.header.frame_count <= 15);
            total += u32::from(rec.header.frame_count);
        }
        assert_eq!(total, 40);
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
    }

    #[test]
    fn no_reservoir_stream_writes_headerless_one_frame_packets() {
        // flags2 = 0x0001: exp VLC, no reservoir, no VBL.
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0001).unwrap();
        assert!(!cfg.bit_reservoir && !cfg.vbl_enabled);
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        let mut sent = Vec::new();
        for f in 0..5u64 {
            sent.push(vec![simple_block(&cfg, 0, f * 7 + 2)]);
        }
        for frame in &sent {
            w.write_frame(frame, None).unwrap();
        }
        let packets = w.finish().unwrap();
        assert_eq!(packets.len(), 5, "one frame per packet");
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
    }

    #[test]
    fn v1_envelope_base_round_trips() {
        let cfg = StreamConfig::derive(Version::V1, 32_000, 1, 6000, 1500, 0x0003).unwrap();
        assert_eq!(cfg.version, Version::V1);
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        let bs = cfg.frame_length;
        let bands = crate::band_partition::exponent_band_count(cfg.sample_rate, bs);
        let envelope = EncEnvelope::Exponents((0..bands as i32).map(|b| 14 + (b % 9)).collect());
        let block = EncBlockData {
            size_index: 0,
            joint_stereo: false,
            total_gain: 25,
            channels: vec![coded_channel(&cfg, bs, envelope, 99)],
        };
        let sent = vec![vec![block]];
        w.write_frame(&sent[0], None).unwrap();
        let packets = w.finish().unwrap();
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
    }

    #[test]
    fn escape_pairs_eob_and_exact_fill_round_trip() {
        let cfg = stereo_vbl_cfg();
        let bs = cfg.frame_length;
        let n = usize::from(cfg.coef_end(bs) - cfg.coef_start(bs));
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        // Channel 0: a huge level (escape), a long run (escape), a
        // mapped pair, and a nonzero final coefficient (no EOB).
        let mut c0 = vec![0i32; n];
        c0[0] = -5000; // above every companion level: escape
        c0[500] = 1; // run 499: escape (mapped runs are short)
        c0[501] = 2; // run 0, level 2: companion pair
        c0[n - 1] = -1; // exact fill: parser stops without EOB
                        // Channel 1: leading zeros then EOB tail.
        let mut c1 = vec![0i32; n];
        c1[3] = 1;
        let mk = |coeffs: Vec<i32>| EncChannelData {
            coded: true,
            envelope: Some(flat_envelope(&cfg, bs)),
            coefficients: coeffs,
        };
        let block = EncBlockData {
            size_index: 0,
            joint_stereo: false,
            total_gain: 10, // w_lvl = 13: 5000 fits the escape field
            channels: vec![mk(c0), mk(c1)],
        };
        let sent = vec![vec![block]];
        let mut wtr = VendorBitWriter::new(&cfg).unwrap();
        wtr.write_frame(&sent[0], None).unwrap();
        let packets = wtr.finish().unwrap();
        let frames = decode_and_check(&cfg, &packets);
        assert_frames_match(&cfg, &sent, &frames);
        let _ = &mut w;
    }

    #[test]
    fn emit_errors_are_typed_and_leave_the_writer_untouched() {
        let cfg = stereo_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        let good = simple_block(&cfg, 0, 5);

        // LSP config refused at construction.
        let lsp = StreamConfig::derive(Version::V2, 8000, 1, 1000, 640, 0x0026).unwrap();
        assert_eq!(
            VendorBitWriter::new(&lsp).unwrap_err(),
            EmitError::LspPathUnsupported
        );

        // Bad size index.
        let mut b = good.clone();
        b.size_index = 7;
        assert!(matches!(
            w.write_frame(&[b], None),
            Err(EmitError::BadBlockSizeIndex { index: 7 })
        ));

        // Frame size mismatch.
        let half = simple_block(&cfg, 1, 6);
        assert!(matches!(
            w.write_frame(std::slice::from_ref(&half), None),
            Err(EmitError::FrameSizeMismatch { .. })
        ));

        // Zero gain.
        let mut b = good.clone();
        b.total_gain = 0;
        assert_eq!(w.write_frame(&[b], None), Err(EmitError::ZeroTotalGain));

        // Level above the escape ceiling (gain 51 -> w_lvl 9).
        let mut b = good.clone();
        b.total_gain = 51;
        b.channels[0].coefficients[10] = 1 << 10;
        assert!(matches!(
            w.write_frame(&[b], None),
            Err(EmitError::LevelTooLarge { .. })
        ));

        // Reuse on a full-length block (no B2 bit).
        let mut b = good.clone();
        for ch in &mut b.channels {
            ch.envelope = Some(EncEnvelope::Reuse);
        }
        assert_eq!(w.write_frame(&[b], None), Err(EmitError::BadReuse));

        // Mixed fresh/reuse on a short block.
        let mut b1 = simple_block(&cfg, 1, 7);
        b1.channels[0].envelope = Some(EncEnvelope::Reuse);
        let b2 = simple_block(&cfg, 1, 8);
        assert_eq!(w.write_frame(&[b1, b2], None), Err(EmitError::BadReuse));

        // Delta out of range.
        let mut b = good.clone();
        if let Some(EncEnvelope::Exponents(e)) = b.channels[0].envelope.as_mut() {
            e[1] = e[0] + 61;
        }
        assert!(matches!(
            w.write_frame(&[b], None),
            Err(EmitError::DeltaOutOfRange { .. })
        ));

        // Wrong coefficient count.
        let mut b = good.clone();
        b.channels[1].coefficients.pop();
        assert!(matches!(
            w.write_frame(&[b], None),
            Err(EmitError::WrongCoefficientCount { .. })
        ));

        // Joint on mono.
        let mono = mono_vbl_cfg();
        let mut wm = VendorBitWriter::new(&mono).unwrap();
        let mut b = simple_block(&mono, 0, 9);
        b.joint_stereo = true;
        assert_eq!(
            wm.write_frame(&[b], None),
            Err(EmitError::JointStereoOnMono)
        );

        // The writer is still usable and the good frame commits.
        assert_eq!(w.frame_count(), 0, "failed frames must not commit");
        w.write_frame(&[good], None).unwrap();
        assert_eq!(w.frame_count(), 1);
    }

    #[test]
    fn frame_too_long_is_refused_before_committing() {
        // A tiny block_align makes the body / carry bound easy to hit.
        let cfg = StreamConfig::derive(Version::V2, 22_050, 2, 4006, 100, 0x0017).unwrap();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        assert!(w.max_frame_bits() <= 800);
        let block = simple_block(&cfg, 0, 42);
        let err = w.write_frame(&[block], None).unwrap_err();
        assert!(matches!(err, EmitError::FrameTooLong { .. }), "{err}");
        assert_eq!(w.frame_count(), 0);
    }

    #[test]
    fn trial_frame_bits_matches_the_committed_size() {
        let cfg = stereo_vbl_cfg();
        let mut w = VendorBitWriter::new(&cfg).unwrap();
        for f in 0..10u64 {
            let frame = vec![simple_block(&cfg, 0, f * 3 + 1)];
            let before = w.position();
            let trial = w.trial_frame_bits(&frame, Some(0)).unwrap();
            w.write_frame(&frame, Some(0)).unwrap();
            assert_eq!(w.position() - before, trial, "frame {f}");
        }
    }
}

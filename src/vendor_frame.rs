//! §2–§4 frame/block bit parser over the assembled §1 stream — the
//! wire layout of vendor-encoded frames.
//!
//! ## Source
//!
//! `docs/audio/wma/frame-bit-layout.md`:
//!
//! * §2 — the per-block field order: F1 block-size indices (see
//!   below), F2a the joint-stereo flag (stereo only, read *before*
//!   the channel flags), F2 the per-channel channel-coded flags, B1
//!   the total-gain accumulator, F3/F4 the noise-substitution
//!   sub-stream (parse-active only when the open-time enable is set
//!   — the enable rule itself is a staged gap, so it is a
//!   caller-supplied hypothesis here, default off), B3 the version-1
//!   envelope base, B4 the exponent delta VLC, B5 the coefficient
//!   sub-stream. B3–B5 are per coded channel, envelopes before
//!   coefficient sub-streams.
//! * §3 — total-gain chaining (7-bit fields, `0x7f` extends) and its
//!   escape-level width mapping; exponent deltas `symbol − 60`
//!   against the initial predictor 36 (v1: 5-bit absolute base
//!   `+ 10`, deltas from band 1).
//! * §3.1 — the line-spectral envelope path when `flags2` bit 0 is
//!   clear: ten fixed-width indices (3,4,4,4,4,4,4,4,3,3 bits) per
//!   coded channel; the index → envelope conversion tables are not
//!   staged, so the indices are carried as data.
//! * §4 — the coefficient run-level sub-stream: symbol 0 =
//!   **escape** (literal `|level|` at the gain-mapped width, run at
//!   `frame_length_bits`, sign), symbol 1 = **end of block**,
//!   symbols ≥ 2 through the 2-based companion map, one trailing
//!   sign bit per non-EOB symbol (1 = positive, 0 = negative).
//! * §5 — F2a's reconstruction consequence (the sum/difference
//!   inverse) runs on dequantised coefficients and lives in the
//!   decode stage.
//!
//! ## Vendor-measured calibrations (this round)
//!
//! Three §2/§5 details were calibrated against the six committed
//! vendor bitstreams, using the §1 carry boundary as ground truth
//! (`tests/vendor_streams.rs` measures them; the closure counts
//! quoted below are reproduced there):
//!
//! 1. **F1 is a one-ahead pipeline.** The per-block field carries
//!    the *next* block's size index; the three-field opening after a
//!    packet header re-primes (previous, current, next). Under the
//!    last-field-is-current reading the multi-size streams lose most
//!    boundaries; under the pipeline the 22.05 kHz stereo stream
//!    closes 1086 of 1098.
//! 2. **No B2 reuse bit exists on the wire.** Reading the §2 B2
//!    row's 1-bit flag desynchronises every stream; without it the
//!    mono 8 kHz stream closes 394 of 394. Envelopes are always sent
//!    for coded channels.
//! 3. **The ALT coefficient tree is channel-scoped.** In a joint
//!    block only the second channel (the difference channel) uses
//!    the class's ALT tree; channel 0 keeps the primary tree.
//!
//! Each calibration is flagged in the final report as a
//! docs-erratum/extension ask rather than silently diverging from
//! the staged text.
//!
//! The parser is deliberately *measurable*: it works over the
//! [`crate::packet::AssembledStream`] and raises the three-field
//! latch whenever the cursor crosses a packet-body boundary, so a
//! driver can compare its landing position against the next packet's
//! declared carry boundary — the §1-provided ground truth for the
//! frame-level layout.

use crate::bitio::{BitReader, BitstreamEnd};
use crate::header::Version;
use crate::stream_config::StreamConfig;
use crate::wire_vlc::{coef_vlc, gain_vlc, runlevel_map, scale_vlc, ExactVlc, VlcDecodeError};

/// §3.1 fixed index widths of the line-spectral envelope path.
pub const LSP_INDEX_WIDTHS: [u8; 10] = [3, 4, 4, 4, 4, 4, 4, 4, 3, 3];

/// §3: escape-level literal width from the block's total gain.
pub fn escape_level_width(total_gain: u32) -> u8 {
    match total_gain {
        0..=14 => 13,
        15..=31 => 12,
        32..=39 => 11,
        40..=44 => 10,
        _ => 9,
    }
}

/// A coded channel's spectral envelope, as parsed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Envelope {
    /// §3 VLC-delta exponents, one per band of the block's partition.
    Exponents(Vec<i32>),
    /// §3.1 line-spectral indices (their conversion tables are a
    /// staged gap; the wire data is carried verbatim).
    LspIndices([u8; 10]),
}

/// One channel's share of a parsed block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelBlock {
    /// F2 — whether the channel codes anything this block.
    pub coded: bool,
    /// The envelope (present iff coded).
    pub envelope: Option<Envelope>,
    /// Noise-substitution band flags (empty unless the F3 sub-stream
    /// was active), one per walked band.
    pub noise_flags: Vec<bool>,
    /// F4 band gains for the flagged bands, in band order.
    pub noise_gains: Vec<i32>,
    /// The entropy-decoded integer coefficients (length = the
    /// block's `n_coef` for a coded channel, empty otherwise).
    pub coefficients: Vec<i32>,
}

/// One parsed §2 block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedBlock {
    /// Block size in samples (`frame_length >> size_index`).
    pub block_size: u16,
    /// The F1 index (0 for a fixed-block stream).
    pub size_index: u8,
    /// F2a — joint-stereo flag (always `false` for mono).
    pub joint_stereo: bool,
    /// B1 — the accumulated total gain.
    pub total_gain: u32,
    /// Number of coded coefficients per coded channel.
    pub n_coef: u16,
    /// Per-channel data (one entry per stream channel).
    pub channels: Vec<ChannelBlock>,
}

/// One parsed frame: its blocks in order (block sizes sum to the
/// stream's `frame_length`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedFrame {
    /// The frame's blocks.
    pub blocks: Vec<ParsedBlock>,
}

/// Frame-parse failures. Each carries enough context to localise the
/// defect when measuring against vendor streams.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameParseError {
    /// The assembled stream ended inside a field.
    Bitstream,
    /// An F1 index outside the configured block-size count.
    BadBlockSizeIndex {
        /// The decoded index.
        index: u8,
    },
    /// A block that does not fit the frame's remaining samples.
    BlockOverflow {
        /// The offending block size.
        block_size: u16,
        /// Samples remaining in the frame before the block.
        remaining: u16,
    },
    /// The coefficient sub-stream wrote past `n_coef`.
    CoefficientOverrun {
        /// Index the write would have landed at.
        index: u32,
        /// The block's coefficient budget.
        n_coef: u16,
    },
    /// A VLC decode failed (bits ran out; the staged tables are
    /// Kraft-complete so no in-stream pattern is undecodable).
    Vlc,
    /// A decoded coefficient symbol has no companion-map entry.
    SymbolOutOfRange {
        /// The offending symbol.
        symbol: u16,
    },
}

impl core::fmt::Display for FrameParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FrameParseError::Bitstream => f.write_str("oxideav-wma: stream ended inside a frame"),
            FrameParseError::BadBlockSizeIndex { index } => {
                write!(f, "oxideav-wma: block-size index {index} out of range")
            }
            FrameParseError::BlockOverflow {
                block_size,
                remaining,
            } => write!(
                f,
                "oxideav-wma: {block_size}-sample block exceeds the frame's remaining {remaining}"
            ),
            FrameParseError::CoefficientOverrun { index, n_coef } => write!(
                f,
                "oxideav-wma: coefficient write at {index} past the block's budget {n_coef}"
            ),
            FrameParseError::Vlc => f.write_str("oxideav-wma: VLC decode failed"),
            FrameParseError::SymbolOutOfRange { symbol } => {
                write!(
                    f,
                    "oxideav-wma: coefficient symbol {symbol} outside the companion map"
                )
            }
        }
    }
}

impl std::error::Error for FrameParseError {}

impl From<BitstreamEnd> for FrameParseError {
    fn from(_: BitstreamEnd) -> Self {
        FrameParseError::Bitstream
    }
}

impl From<VlcDecodeError> for FrameParseError {
    fn from(e: VlcDecodeError) -> Self {
        match e {
            VlcDecodeError::Bitstream(_) => FrameParseError::Bitstream,
            VlcDecodeError::InvalidCodeword => FrameParseError::Vlc,
        }
    }
}

/// Where the §2.1 noise-band walk starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NoiseStart {
    /// A fixed band index into the block's partition.
    Band(usize),
    /// The first band whose lower edge is at or above this fraction
    /// of the block's coefficient count (a frequency cutoff expressed
    /// on the coefficient axis, so it scales across block sizes).
    CoefFraction(f64),
}

/// The §2.1 noise-substitution parse hypothesis. The open-time
/// enable rule and the identity of "the band table" §2.1 walks are
/// still open in the staged docs (`frame-bit-layout.md` "Still
/// open"), so the sub-stream is **off by default** and this carrier
/// exists to make the enabled hypothesis measurable, not to assert
/// it: when set, the walk runs over the block's exponent-band
/// partition starting at the resolved first band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NoiseSpec {
    /// The §2.1 walk's starting band.
    pub start: NoiseStart,
}

/// Stateful §2 frame parser over an assembled §1 stream.
#[derive(Debug, Clone)]
pub struct FrameParser {
    cfg: StreamConfig,
    /// §2.1 hypothesis switch (default `None` = no F3/F4 bits).
    noise: Option<NoiseSpec>,
    /// Packet-body start bits, for raising the three-field latch.
    body_starts: Vec<u64>,
    next_boundary: usize,
    /// The §2 F1 latch: raised after every packet-header parse.
    latch: bool,
    /// The F1 pipeline: the pre-read next-block size index.
    next_size: Option<u8>,
    /// The previous-block size index from the three-field opening
    /// (windowing context; carried for the decode stage).
    prev_size: Option<u8>,
}

impl FrameParser {
    /// A parser for `cfg` over a stream whose packet bodies start at
    /// `body_starts` (bit offsets in the assembled stream, in
    /// order — [`crate::packet::AssembledStream`] provides them).
    /// The latch starts raised: the first block of the stream
    /// follows the first packet header.
    pub fn new(cfg: &StreamConfig, body_starts: &[u64]) -> Self {
        Self {
            cfg: cfg.clone(),
            noise: None,
            body_starts: body_starts.to_vec(),
            next_boundary: if body_starts.first() == Some(&0) {
                1
            } else {
                0
            },
            latch: true,
            next_size: None,
            prev_size: None,
        }
    }

    /// Enable the §2.1 noise-substitution parse hypothesis.
    pub fn with_noise(mut self, noise: NoiseSpec) -> Self {
        self.noise = Some(noise);
        self
    }

    /// Re-raise the three-field latch (a §1 discontinuity resync —
    /// the block-sequencer's packet-start state raises it too).
    pub fn raise_latch(&mut self) {
        self.latch = true;
        self.next_size = None;
        self.prev_size = None;
    }

    /// Lower the latch (diagnostic use).
    #[doc(hidden)]
    pub fn lower_latch(&mut self) {
        self.latch = false;
    }

    /// Advance the boundary watch to `cursor`, raising the latch for
    /// every packet-body start crossed.
    fn update_latch(&mut self, cursor: u64) {
        while self
            .body_starts
            .get(self.next_boundary)
            .is_some_and(|&b| b <= cursor)
        {
            self.latch = true;
            self.next_boundary += 1;
        }
    }

    /// Parse one frame at the reader's position. The reader must sit
    /// on the first bit of a frame in the assembled stream.
    ///
    /// # Errors
    ///
    /// [`FrameParseError`] — the caller decides whether to resync at
    /// the next packet's carry boundary (§1 gives it the offset).
    pub fn parse_frame(
        &mut self,
        reader: &mut BitReader<'_>,
    ) -> Result<ParsedFrame, FrameParseError> {
        // The three-field latch applies to the first block of the
        // first frame *starting* in a packet: measured on the vendor
        // streams, re-raising it for a mid-frame block right after a
        // boundary crossing closes strictly fewer carry boundaries
        // (55 vs 64 of 122 on the 22.05 kHz mono stream), so the
        // crossing check runs per frame, not per block.
        self.update_latch(reader.position() as u64);
        let mut blocks = Vec::new();
        let mut remaining = self.cfg.frame_length;
        while remaining > 0 {
            let block = self.parse_block(reader, remaining)?;
            remaining -= block.block_size;
            blocks.push(block);
        }
        Ok(ParsedFrame { blocks })
    }

    fn parse_block(
        &mut self,
        reader: &mut BitReader<'_>,
        remaining: u16,
    ) -> Result<ParsedBlock, FrameParseError> {
        // F1 — block-size indices (VBL streams only), pipelined one
        // block ahead: the sizes a lapped transform needs before it
        // can window the current block. The three-field opening after
        // a packet header carries (previous, current, next); every
        // later block reads exactly one field — the *next* block's
        // size — and takes its own size from the pipeline. This is
        // the reading the vendor bitstreams themselves select: on the
        // staged streams the carry-boundary closure rate is strictly
        // higher than under the last-field-is-current reading of the
        // §2 F1 note (measured in tests/vendor_streams.rs).
        let size_index = if self.cfg.vbl_enabled {
            let cur = if self.latch {
                let prev = reader.read_bits(self.cfg.w_bs)? as u8;
                self.prev_size = Some(prev);
                reader.read_bits(self.cfg.w_bs)? as u8
            } else {
                self.next_size.unwrap_or(0)
            };
            self.latch = false;
            self.next_size = Some(reader.read_bits(self.cfg.w_bs)? as u8);
            cur
        } else {
            0
        };
        let block_size = self
            .cfg
            .block_size_for_index(size_index)
            .ok_or(FrameParseError::BadBlockSizeIndex { index: size_index })?;
        if block_size > remaining {
            return Err(FrameParseError::BlockOverflow {
                block_size,
                remaining,
            });
        }

        let channels = usize::from(self.cfg.channels);

        // F2a — joint-stereo / VLC-variant flag, before the channel
        // flags; two-channel streams only.
        let joint_stereo = if channels == 2 {
            reader.read_bit()?
        } else {
            false
        };

        // F2 — channel-coded flags.
        let mut coded = Vec::with_capacity(channels);
        for _ in 0..channels {
            coded.push(reader.read_bit()?);
        }

        // "If every channel's bit is 0 the block ends here."
        if coded.iter().all(|&c| !c) {
            return Ok(ParsedBlock {
                block_size,
                size_index,
                joint_stereo,
                total_gain: 0,
                n_coef: 0,
                channels: coded
                    .into_iter()
                    .map(|c| ChannelBlock {
                        coded: c,
                        envelope: None,
                        noise_flags: Vec::new(),
                        noise_gains: Vec::new(),
                        coefficients: Vec::new(),
                    })
                    .collect(),
            });
        }

        // B1 — total gain: start at 1, 0x7f extends.
        let mut total_gain: u32 = 1;
        loop {
            let v = reader.read_bits(7)? as u32;
            total_gain += v;
            if v != 0x7f {
                break;
            }
        }

        let coef_start = self.cfg.coef_start(block_size);
        let coef_end = self.cfg.coef_end(block_size);
        let base_coef = coef_end - coef_start;

        // Band partition for this block size (envelope + noise walk).
        let edges = crate::band_partition::exponent_band_edges(self.cfg.sample_rate, block_size);
        let band_count = edges.len() - 1;

        // F3/F4 — noise-substitution sub-stream (hypothesis-gated;
        // all channels' flags precede all channels' gains).
        let mut noise_flags: Vec<Vec<bool>> = vec![Vec::new(); channels];
        let mut noise_gains: Vec<Vec<i32>> = vec![Vec::new(); channels];
        let mut noise_widths: Vec<u16> = vec![0; channels];
        if let Some(spec) = self.noise {
            for ch in 0..channels {
                if !coded[ch] {
                    continue;
                }
                let mut band = match spec.start {
                    NoiseStart::Band(b) => b,
                    NoiseStart::CoefFraction(frac) => {
                        let cutoff = frac * f64::from(block_size);
                        (0..band_count)
                            .find(|&b| f64::from(edges[b]) >= cutoff)
                            .unwrap_or(band_count)
                    }
                };
                while band < band_count && edges[band] < coef_end {
                    let flagged = reader.read_bit()?;
                    if flagged {
                        let lo = edges[band].max(coef_start);
                        let hi = edges[band + 1].min(coef_end);
                        noise_widths[ch] += hi.saturating_sub(lo);
                    }
                    noise_flags[ch].push(flagged);
                    band += 1;
                }
            }
            for ch in 0..channels {
                let flagged_count = noise_flags[ch].iter().filter(|&&f| f).count();
                if flagged_count == 0 {
                    continue;
                }
                // First gain absolute (7 bits − 19), then VLC deltas
                // (symbol − 18) chained per channel.
                let mut gain = reader.read_bits(7)? as i32 - 19;
                noise_gains[ch].push(gain);
                for _ in 1..flagged_count {
                    let sym = gain_vlc().decode_symbol(reader)?;
                    gain += i32::from(sym) - 18;
                    noise_gains[ch].push(gain);
                }
            }
        }

        // B3/B4 — per coded channel envelopes, channel 0 first.
        //
        // No envelope-reuse flag is read here. The §2 B2 row claims a
        // 1-bit reuse flag under the VBL gate, but the vendor
        // bitstreams contradict it: with the flag the carry-boundary
        // closure collapses (e.g. 210 of 394 on the mono 8 kHz
        // stream), without it the same stream closes 394 of 394 —
        // measured in tests/vendor_streams.rs. Flagged to the docs
        // collaborator as an erratum candidate.
        let mut envelopes: Vec<Option<Envelope>> = vec![None; channels];
        for ch in 0..channels {
            if !coded[ch] {
                continue;
            }
            if self.cfg.exp_vlc {
                // B3 — version-1 absolute base; v2 predicts from 36.
                let (mut prev, start_band) = match self.cfg.version {
                    Version::V1 => ((reader.read_bits(5)? as i32) + 10, 1usize),
                    Version::V2 => (36, 0usize),
                };
                let mut exponents = Vec::with_capacity(band_count);
                if start_band == 1 {
                    exponents.push(prev);
                }
                for _ in start_band..band_count {
                    let sym = scale_vlc().decode_symbol(reader)?;
                    prev += i32::from(sym) - 60;
                    exponents.push(prev);
                }
                envelopes[ch] = Some(Envelope::Exponents(exponents));
            } else {
                // §3.1 — ten fixed-width line-spectral indices.
                let mut idx = [0u8; 10];
                for (slot, &w) in idx.iter_mut().zip(LSP_INDEX_WIDTHS.iter()) {
                    *slot = reader.read_bits(w)? as u8;
                }
                envelopes[ch] = Some(Envelope::LspIndices(idx));
            }
        }

        // B5 — per coded channel coefficient sub-streams. In a joint
        // (F2a set) block the ALT tree applies to the **second**
        // channel only — the difference channel, whose statistics the
        // alt tables fit — while channel 0 keeps the primary tree.
        // This per-channel split is what the vendor bitstreams
        // select: with the ALT tree on both channels of a joint
        // block the stereo streams close no boundaries at all; with
        // it on the second channel alone the 22.05 kHz stereo stream
        // closes 1086 of 1098 (tests/vendor_streams.rs). §5's "set
        // selects the ALT coefficient VLC table" is thereby
        // channel-scoped, not block-scoped.
        let w_lvl = escape_level_width(total_gain);
        let mut coefficients: Vec<Vec<i32>> = vec![Vec::new(); channels];
        for ch in 0..channels {
            if !coded[ch] {
                continue;
            }
            let alt = joint_stereo && ch == 1;
            let vlc = coef_vlc(self.cfg.vlc_class, alt).ok_or(FrameParseError::Vlc)?;
            let map = runlevel_map(self.cfg.vlc_class, alt).ok_or(FrameParseError::Vlc)?;
            let n_coef = base_coef - noise_widths[ch];
            coefficients[ch] =
                decode_coefficients(reader, vlc, map, n_coef, w_lvl, self.cfg.frame_length_bits)?;
        }

        Ok(ParsedBlock {
            block_size,
            size_index,
            joint_stereo,
            total_gain,
            n_coef: base_coef,
            channels: (0..channels)
                .map(|ch| ChannelBlock {
                    coded: coded[ch],
                    envelope: envelopes[ch].take(),
                    noise_flags: std::mem::take(&mut noise_flags[ch]),
                    noise_gains: std::mem::take(&mut noise_gains[ch]),
                    coefficients: std::mem::take(&mut coefficients[ch]),
                })
                .collect(),
        })
    }
}

/// §4: decode one channel's run-level coefficient sub-stream into
/// `n_coef` integers.
fn decode_coefficients(
    reader: &mut BitReader<'_>,
    vlc: &ExactVlc,
    map: &[(u16, u16)],
    n_coef: u16,
    escape_level_bits: u8,
    escape_run_bits: u8,
) -> Result<Vec<i32>, FrameParseError> {
    let n = u32::from(n_coef);
    let mut out = vec![0i32; n_coef as usize];
    let mut idx: u32 = 0;
    while idx < n {
        let symbol = vlc.decode_symbol(reader)?;
        let (run, abs_level) = match symbol {
            // §4: symbol 0 = ESCAPE — literal level, run, sign.
            0 => {
                let level = reader.read_bits(escape_level_bits)? as u32;
                let run = reader.read_bits(escape_run_bits)? as u32;
                let sign_positive = reader.read_bit()?;
                let target = idx + run;
                if target >= n {
                    return Err(FrameParseError::CoefficientOverrun {
                        index: target,
                        n_coef,
                    });
                }
                out[target as usize] = if sign_positive {
                    level as i32
                } else {
                    -(level as i32)
                };
                idx = target + 1;
                continue;
            }
            // §4: symbol 1 = END OF BLOCK — the rest stays zero.
            1 => break,
            s => *map
                .get(usize::from(s) - 2)
                .ok_or(FrameParseError::SymbolOutOfRange { symbol: s })?,
        };
        let sign_positive = reader.read_bit()?;
        let target = idx + u32::from(run);
        if target >= n {
            return Err(FrameParseError::CoefficientOverrun {
                index: target,
                n_coef,
            });
        }
        out[target as usize] = if sign_positive {
            i32::from(abs_level)
        } else {
            -i32::from(abs_level)
        };
        idx = target + 1;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitio::BitWriter;
    use crate::stream_config::StreamConfig;

    /// Mono 44.1 kHz fixed-block reservoir stream (class 3 via the
    /// high-rate arm: bps = 64024/44100 = 1.45 ≥ 1.16).
    fn mono_fixed_cfg() -> StreamConfig {
        StreamConfig::derive(Version::V2, 44_100, 1, 8003, 2973, 0b0011).unwrap()
    }

    /// Stereo 22.05 kHz VBL stream — the staged cand_stereo22k row.
    fn stereo_vbl_cfg() -> StreamConfig {
        StreamConfig::derive(Version::V2, 22_050, 2, 4006, 744, 0x0017).unwrap()
    }

    fn write_scale_delta_zero_run(w: &mut BitWriter, count: usize) {
        for _ in 0..count {
            assert!(scale_vlc().encode_symbol(60, w));
        }
    }

    #[test]
    fn minimal_mono_frame_parses_and_lands_exactly() {
        let cfg = mono_fixed_cfg();
        assert!(!cfg.vbl_enabled);
        assert_eq!(cfg.vlc_class, 3);
        let bands = crate::band_partition::exponent_band_count(44_100, 2048);
        assert_eq!(bands, 25);

        let mut w = BitWriter::new();
        w.write_bit(true); // F2: channel coded
        w.write_bits(50, 7); // B1: total gain 51
        write_scale_delta_zero_run(&mut w, bands); // B4: all deltas 0
        let vlc = coef_vlc(3, false).unwrap();
        assert!(vlc.encode_symbol(1, &mut w)); // B5: EOB immediately
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();

        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len, "must land exactly on the frame end");
        assert_eq!(frame.blocks.len(), 1);
        let b = &frame.blocks[0];
        assert_eq!(b.block_size, 2048);
        assert_eq!(b.total_gain, 51);
        assert!(!b.joint_stereo);
        assert_eq!(b.n_coef, 2048 - 184);
        let ch = &b.channels[0];
        assert!(ch.coded);
        match ch.envelope.as_ref().unwrap() {
            Envelope::Exponents(e) => {
                assert_eq!(e.len(), bands);
                assert!(e.iter().all(|&x| x == 36)); // predictor 36, deltas 0
            }
            other => panic!("unexpected envelope {other:?}"),
        }
        assert!(ch.coefficients.iter().all(|&c| c == 0));
    }

    #[test]
    fn total_gain_extends_through_0x7f_and_maps_widths() {
        let cfg = mono_fixed_cfg();
        let bands = crate::band_partition::exponent_band_count(44_100, 2048);
        let mut w = BitWriter::new();
        w.write_bit(true);
        w.write_bits(0x7f, 7); // extend
        w.write_bits(0x7f, 7); // extend
        w.write_bits(3, 7); // terminate: 1 + 127 + 127 + 3 = 258
        write_scale_delta_zero_run(&mut w, bands);
        assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(frame.blocks[0].total_gain, 258);

        // §3 escape width mapping boundaries.
        for (gain, width) in [
            (0, 13),
            (14, 13),
            (15, 12),
            (31, 12),
            (32, 11),
            (39, 11),
            (40, 10),
            (44, 10),
            (45, 9),
            (200, 9),
        ] {
            assert_eq!(escape_level_width(gain), width, "gain {gain}");
        }
    }

    #[test]
    fn run_level_pairs_escapes_and_signs_reconstruct() {
        let cfg = mono_fixed_cfg();
        let bands = crate::band_partition::exponent_band_count(44_100, 2048);
        let vlc = coef_vlc(3, false).unwrap();
        let map = runlevel_map(3, false).unwrap();
        // Find a symbol with a known small pair: symbol 2 is the
        // first companion entry.
        let (r0, l0) = map[0];

        let mut w = BitWriter::new();
        w.write_bit(true);
        w.write_bits(50, 7); // gain 51 → w_lvl 9
        write_scale_delta_zero_run(&mut w, bands);
        // Pair via symbol 2, negative sign.
        assert!(vlc.encode_symbol(2, &mut w));
        w.write_bit(false); // negative
                            // Escape: |level| = 300 (9 bits), run = 5, positive.
        assert!(vlc.encode_symbol(0, &mut w));
        w.write_bits(300, 9);
        w.write_bits(5, cfg.frame_length_bits);
        w.write_bit(true);
        // EOB.
        assert!(vlc.encode_symbol(1, &mut w));
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();

        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        let coeffs = &frame.blocks[0].channels[0].coefficients;
        let first = usize::from(r0);
        assert_eq!(coeffs[first], -i32::from(l0));
        assert_eq!(coeffs[first + 1 + 5], 300);
        assert!(coeffs[first + 1..first + 1 + 5].iter().all(|&c| c == 0));
    }

    #[test]
    fn vbl_stereo_three_field_opening_and_channel_scoped_alt() {
        let cfg = stereo_vbl_cfg();
        assert!(cfg.vbl_enabled);
        assert_eq!((cfg.n_block_sizes, cfg.w_bs), (8, 2));
        let bands = crate::band_partition::exponent_band_count(22_050, 1024);

        let mut w = BitWriter::new();
        // F1 × 3 (latch raised at stream start): previous, current,
        // next — current index 0 → 1024-sample block.
        w.write_bits(0, 2); // previous
        w.write_bits(0, 2); // current
        w.write_bits(0, 2); // next (pipeline)
        w.write_bit(true); // F2a: joint stereo
        w.write_bit(true); // ch0 coded
        w.write_bit(true); // ch1 coded
        w.write_bits(10, 7); // B1 → total gain 11
        write_scale_delta_zero_run(&mut w, bands); // ch0 envelope
        write_scale_delta_zero_run(&mut w, bands); // ch1 envelope
                                                   // Channel-scoped ALT: ch0 from the primary tree, ch1 from
                                                   // the ALT tree.
        assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
        assert!(coef_vlc(3, true).unwrap().encode_symbol(1, &mut w));
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();

        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        let b = &frame.blocks[0];
        assert_eq!(b.block_size, 1024);
        assert!(b.joint_stereo);
        assert!(b.channels.iter().all(|c| c.coded));
        assert!(b.channels[0].coefficients.iter().all(|&c| c == 0));
        assert!(b.channels[1].coefficients.iter().all(|&c| c == 0));

        // A second frame reads ONE field (the next-size pipeline) and
        // takes its own size from the previous block's field.
        let mut w = BitWriter::new();
        w.write_bits(1, 2); // next-size field (pipeline)
        w.write_bit(false); // F2a clear → primary for both channels
        w.write_bit(true);
        w.write_bit(false);
        w.write_bits(10, 7);
        write_scale_delta_zero_run(&mut w, bands);
        assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        // Size came from the pipeline (previous block wrote next=0).
        assert_eq!(frame.blocks[0].block_size, 1024);
        assert!(!frame.blocks[0].joint_stereo);
    }

    #[test]
    fn all_channels_uncoded_ends_the_block_immediately() {
        let cfg = stereo_vbl_cfg();
        let mut w = BitWriter::new();
        w.write_bits(0, 2);
        w.write_bits(0, 2);
        w.write_bits(0, 2);
        w.write_bit(false); // F2a
        w.write_bit(false); // ch0
        w.write_bit(false); // ch1
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        assert_eq!(frame.blocks[0].total_gain, 0);
        assert!(frame.blocks[0].channels.iter().all(|c| !c.coded));
    }

    #[test]
    fn vbl_frame_tiles_from_multiple_blocks() {
        let cfg = stereo_vbl_cfg();
        let bands_512 = crate::band_partition::exponent_band_count(22_050, 512);
        let mut w = BitWriter::new();
        // Two 512-sample blocks tile the 1024-sample frame. Under
        // the pipeline the opening carries (prev, cur, next) =
        // (1, 1, 1); the second block reads only its own next field.
        for i in 0..2 {
            if i == 0 {
                w.write_bits(1, 2); // previous
                w.write_bits(1, 2); // current: index 1 → 512
            }
            w.write_bits(1, 2); // next (pipeline)
            w.write_bit(false); // F2a
            w.write_bit(true); // ch0
            w.write_bit(false); // ch1
            w.write_bits(20, 7);
            write_scale_delta_zero_run(&mut w, bands_512);
            assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
        }
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        assert_eq!(frame.blocks.len(), 2);
        assert!(frame.blocks.iter().all(|b| b.block_size == 512));
        // The 512-coefficient partition at 22.05 kHz is the staged
        // "lo"-arm… no: 512 is not tabulated for lo — computed walk.
        match frame.blocks[0].channels[0].envelope.as_ref().unwrap() {
            Envelope::Exponents(e) => assert_eq!(e.len(), bands_512),
            other => panic!("unexpected envelope {other:?}"),
        }
    }

    #[test]
    fn no_reuse_bit_is_read_before_the_envelope() {
        // The vendor-measured calibration: no B2 bit exists — the
        // envelope follows the total gain directly. A frame written
        // that way parses back exactly.
        let cfg = stereo_vbl_cfg();
        let bands = crate::band_partition::exponent_band_count(22_050, 1024);
        let mut w = BitWriter::new();
        w.write_bits(0, 2);
        w.write_bits(0, 2);
        w.write_bits(0, 2);
        w.write_bit(false); // F2a
        w.write_bit(true); // ch0
        w.write_bit(false); // ch1
        w.write_bits(10, 7);
        write_scale_delta_zero_run(&mut w, bands); // envelope, immediately
        assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        match frame.blocks[0].channels[0].envelope.as_ref().unwrap() {
            Envelope::Exponents(e) => assert!(e.iter().all(|&x| x == 36)),
            other => panic!("unexpected envelope {other:?}"),
        }
    }

    #[test]
    fn lsp_path_reads_exactly_37_bits_per_coded_channel() {
        // flags2 bit 0 clear + reservoir: mono8k-style configuration.
        let cfg = StreamConfig::derive(Version::V2, 8000, 1, 1000, 640, 0x0026).unwrap();
        assert!(!cfg.exp_vlc && cfg.vbl_enabled);
        let mut w = BitWriter::new();
        w.write_bits(0, 1); // F1 ×3 at w_bs = 1
        w.write_bits(0, 1);
        w.write_bits(0, 1);
        w.write_bit(true); // F2 (mono: no F2a)
        w.write_bits(10, 7); // B1
        for (i, &width) in LSP_INDEX_WIDTHS.iter().enumerate() {
            w.write_bits(i as u64 % (1 << width.min(3)), width);
        }
        assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut parser = FrameParser::new(&cfg, &[0]);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), bit_len);
        match frame.blocks[0].channels[0].envelope.as_ref().unwrap() {
            Envelope::LspIndices(idx) => {
                assert_eq!(idx.len(), 10);
            }
            other => panic!("unexpected envelope {other:?}"),
        }
    }

    #[test]
    fn boundary_crossing_re_raises_the_latch() {
        let cfg = stereo_vbl_cfg();
        let bands = crate::band_partition::exponent_band_count(22_050, 1024);
        // Frame 1 sits before bit 200; boundary at 200; frame 2 after.
        let make_frame = |three_fields: bool| {
            let mut w = BitWriter::new();
            let n = if three_fields { 3 } else { 1 };
            for _ in 0..n {
                w.write_bits(0, 2);
            }
            w.write_bit(false);
            w.write_bit(true);
            w.write_bit(false);
            w.write_bits(10, 7);
            write_scale_delta_zero_run(&mut w, bands);
            assert!(coef_vlc(3, false).unwrap().encode_symbol(1, &mut w));
            (w.bit_len(), w.into_bytes())
        };
        let (len1, bytes1) = make_frame(true);
        assert!(len1 < 200);
        // Assemble: frame1 | padding to 200 | frame2(three fields).
        let (len2, bytes2) = make_frame(true);
        let mut w = BitWriter::new();
        let mut r1 = BitReader::with_bit_len(&bytes1, len1);
        for _ in 0..len1 {
            w.write_bit(r1.read_bit().unwrap());
        }
        for _ in len1..200 {
            w.write_bit(false);
        }
        let mut r2 = BitReader::with_bit_len(&bytes2, len2);
        for _ in 0..len2 {
            w.write_bit(r2.read_bit().unwrap());
        }
        let total = w.bit_len();
        let bytes = w.into_bytes();

        let mut parser = FrameParser::new(&cfg, &[0, 200]);
        let mut r = BitReader::with_bit_len(&bytes, total);
        parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), len1);
        // Jump the reader to bit 200 (the next packet's body) — the
        // parser sees the crossing and expects three F1 fields again.
        let mut r = BitReader::with_bit_len(&bytes, total);
        r.read_bits(64).unwrap();
        let mut skipped = 64;
        while skipped < 200 {
            r.read_bit().unwrap();
            skipped += 1;
        }
        let frame = parser.parse_frame(&mut r).unwrap();
        assert_eq!(r.position(), 200 + len2);
        assert_eq!(frame.blocks[0].block_size, 1024);
    }
}

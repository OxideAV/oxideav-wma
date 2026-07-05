//! The frame / per-block **bit-packing layout** — the wire realization
//! of `docs/audio/wma/frame-bit-layout.md`.
//!
//! ## Source
//!
//! The staged layout trace pins (statically, from the vendor decode
//! parse path):
//!
//! * **MSB-first bit order** for every field (the get-bits primitive
//!   masks with `(1 << n) - 1` — exactly this crate's
//!   [`crate::bitio`] convention).
//! * **Per-frame header order** (§1): S1 reservoir/byte-offset field
//!   (width = the staged `byte_offset_bits` formula, realised by
//!   [`crate::setup::SetupParams`]) → S2 frame side field (width is
//!   runtime/config — its *formula is not staged*, so the caller
//!   supplies an observed width, a typed `[GAP]`) → S3 1-bit flag.
//! * **Per-block field order** (§2): B1 7-bit block header (`0x7f` =
//!   all-ones marker) → B2 gain VLC sub-stream → B3 1-bit
//!   stereo/coupling flag (2-channel streams only) → B4 5-bit
//!   envelope base field → B5 scale/exponent VLC sub-stream → B6
//!   coefficient run-level sub-stream.
//! * **Coefficients** (§3): joint run-level VLC symbols; the **sign
//!   is one bit immediately after each non-zero coefficient**
//!   (negation); the reserved escape symbol (1) pulls a literal run
//!   and a literal level at runtime-signalled widths; sub-streams are
//!   **self-delimiting** — the coefficient stream ends when the
//!   block's coefficient count is satisfied (or at the reserved
//!   end-of-block symbol 0).
//!
//! ## What stays `[GAP]` (typed, not guessed)
//!
//! * The S2 side-field **width formula** and the concrete **escape
//!   literal widths** (their *sources* are pinned — frame-geometry
//!   config — their values are per-stream; both are caller-supplied
//!   parameters here).
//! * The **semantic content** of B1/B4 and the gain/scale delta
//!   chaining (initial values, per-band order): this module carries
//!   the fields and symbol streams verbatim, exactly as wide as the
//!   trace pins them.
//! * **Sub-stream element counts** (gain symbols per block, scale
//!   symbols per block): self-delimiting by decoded counts derived
//!   from the stream geometry; the caller derives and supplies them
//!   (for scale, [`crate::exponent_bands`] derives the band count).
//! * **Frames-per-packet / bit-reservoir walk** (runtime-gated per
//!   the trace) and the variable-block-length split (`flags2` bit 2).
//! * **Sign polarity**: the trace pins "1 bit, then negation"; this
//!   module writes `1` = negative as the documented single-swap-point
//!   convention.

use crate::bitio::{BitReader, BitWriter, BitstreamEnd};
use crate::coef_vlc::{CoefEvent, CoefVlc, CoefVlcError};
use crate::envelope_vlc::{GainVlc, ScaleVlc};
use crate::huffman::HuffmanError;
use crate::paircode::EscapeWidths;

/// Width in bits of the B1 per-block header field (staged: a `push 7`
/// immediate at the block-parse site).
pub const BLOCK_HEADER_BITS: u8 = 7;

/// The all-ones B1 value the trace singles out as an escape/marker
/// (`0x7f`).
pub const BLOCK_HEADER_MARKER: u8 = 0x7f;

/// Width in bits of the B4 envelope base field (staged: `push 5`).
pub const ENVELOPE_BASE_BITS: u8 = 5;

/// Width in bits of the B3 stereo/coupling flag and the S3 frame flag
/// and the per-coefficient sign (staged: `push 1`).
pub const FLAG_BITS: u8 = 1;

/// The two runtime-width per-frame header fields' widths.
///
/// `byte_offset_bits` follows the staged formula
/// `floor(log2(bps * frame_length / 8)) + 2`
/// ([`crate::setup::SetupParams::byte_offset_bits`]); the side-field
/// width's formula is **not** staged (typed `[GAP]`) — callers thread
/// in a black-box-observed value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameFieldWidths {
    /// S1 width — the reservoir/byte-offset field.
    pub byte_offset_bits: u8,
    /// S2 width — the frame side field (formula unstaged).
    pub side_field_bits: u8,
}

impl FrameFieldWidths {
    /// Validate the two widths (each `1..=32`).
    ///
    /// # Errors
    ///
    /// [`FrameBitsError::WidthOutOfRange`] outside `1..=32`.
    pub fn new(byte_offset_bits: u8, side_field_bits: u8) -> Result<Self, FrameBitsError> {
        for (field, bits) in [
            ("byte_offset_bits", byte_offset_bits),
            ("side_field_bits", side_field_bits),
        ] {
            if bits == 0 || bits > 32 {
                return Err(FrameBitsError::WidthOutOfRange { field, bits });
            }
        }
        Ok(Self {
            byte_offset_bits,
            side_field_bits,
        })
    }
}

/// The three per-frame header fields, in their staged order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeaderFields {
    /// S1 — frame bit-offset / reservoir carry.
    pub reservoir_offset: u32,
    /// S2 — the frame side field (semantics unstaged; carried
    /// verbatim).
    pub side_field: u32,
    /// S3 — the 1-bit per-frame flag.
    pub flag: bool,
}

impl FrameHeaderFields {
    /// Write the header in the staged order S1 → S2 → S3.
    ///
    /// # Errors
    ///
    /// [`FrameBitsError::ValueTooWide`] when a field value does not
    /// fit its width.
    pub fn write(
        &self,
        widths: &FrameFieldWidths,
        writer: &mut BitWriter,
    ) -> Result<(), FrameBitsError> {
        write_checked(
            writer,
            "reservoir_offset",
            self.reservoir_offset,
            widths.byte_offset_bits,
        )?;
        write_checked(
            writer,
            "side_field",
            self.side_field,
            widths.side_field_bits,
        )?;
        writer.write_bit(self.flag);
        Ok(())
    }

    /// Read the header in the staged order S1 → S2 → S3.
    ///
    /// # Errors
    ///
    /// [`FrameBitsError::Bitstream`] on truncation.
    pub fn read(
        widths: &FrameFieldWidths,
        reader: &mut BitReader<'_>,
    ) -> Result<Self, FrameBitsError> {
        let reservoir_offset = reader.read_bits(widths.byte_offset_bits)? as u32;
        let side_field = reader.read_bits(widths.side_field_bits)? as u32;
        let flag = reader.read_bit()?;
        Ok(Self {
            reservoir_offset,
            side_field,
            flag,
        })
    }
}

/// Everything needed to parse/emit one block's bit layout: the three
/// real VLCs, the escape literal widths, and the self-delimiting
/// element counts the caller derives from the stream geometry.
#[derive(Debug, Clone)]
pub struct BlockPlan<'a> {
    /// The coefficient run-level VLC for the stream's decode class.
    pub coef_vlc: &'a CoefVlc,
    /// The 37-symbol gain delta VLC (sub-stream B2).
    pub gain_vlc: &'a GainVlc,
    /// The 121-symbol scale delta VLC (sub-stream B5).
    pub scale_vlc: &'a ScaleVlc,
    /// Escape literal widths (values per-stream config — `[GAP]`,
    /// caller-observed; the staged trace pins their *sources* as the
    /// frame-geometry fields).
    pub escape: EscapeWidths,
    /// Channel count (`1` or `2`) — the B3 flag exists only for 2.
    pub channels: u8,
    /// Whether the envelope fields (B4 + B5) are present — the staged
    /// trace gates the scale sub-stream on a config flag
    /// (`ctx+0x90 == 1`); carried as a typed switch.
    pub envelope_coded: bool,
    /// Gain symbols per block (self-delimiting count, caller-derived).
    pub gain_count: usize,
    /// Scale symbols per block (the exponent band count,
    /// caller-derived via [`crate::exponent_bands`]).
    pub scale_count: usize,
    /// Coefficients per block (the transform length `M`).
    pub coef_count: usize,
}

/// One block's demuxed wire fields, in the staged B1..B6 order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireBlock {
    /// B1 — the 7-bit block header value.
    pub header: u8,
    /// B2 — the gain VLC symbol stream.
    pub gain_symbols: Vec<usize>,
    /// B3 — the stereo/coupling flag (`Some` iff 2-channel).
    pub stereo_coupling: Option<bool>,
    /// B4 — the 5-bit envelope base field (present iff
    /// `envelope_coded`).
    pub envelope_base: u8,
    /// B5 — the scale/exponent VLC symbol stream.
    pub scale_symbols: Vec<usize>,
    /// B6 — the reconstructed signed coefficient vector (length `M`).
    pub coefficients: Vec<i32>,
}

/// Failure modes of the frame/block bit layer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrameBitsError {
    /// A configured field width was zero or above 32.
    WidthOutOfRange {
        /// Field name.
        field: &'static str,
        /// Rejected width.
        bits: u8,
    },
    /// A field value does not fit its configured width.
    ValueTooWide {
        /// Field name.
        field: &'static str,
        /// The rejected value.
        value: u32,
        /// The width it had to fit.
        bits: u8,
    },
    /// A plan/block mismatch (wrong vector length, bad channel count,
    /// coefficient vector not `M` long, …).
    PlanMismatch {
        /// What did not line up.
        what: &'static str,
    },
    /// An `(R, L)` pair is neither in the VLC alphabet nor
    /// representable by the escape literal widths.
    EscapeOverflow {
        /// The run that had to be coded.
        run: u16,
        /// The magnitude that had to be coded.
        abs_level: u32,
    },
    /// A decoded run stepped past the block's coefficient count.
    CoefficientOverflow {
        /// Position after applying the run.
        position: usize,
        /// The block's coefficient count.
        coef_count: usize,
    },
    /// A decoded escape literal level was zero (a non-event the
    /// encoder never emits; treated as stream corruption).
    EscapeLevelZero,
    /// The bit stream ended inside a field.
    Bitstream(BitstreamEnd),
    /// A coefficient VLC error (construction/decode).
    Coef(CoefVlcError),
    /// A gain/scale VLC error.
    Huffman(HuffmanError),
}

impl core::fmt::Display for FrameBitsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FrameBitsError::WidthOutOfRange { field, bits } => write!(
                f,
                "oxideav-wma::frame_bits: field `{field}` width {bits} is outside 1..=32",
            ),
            FrameBitsError::ValueTooWide { field, value, bits } => write!(
                f,
                "oxideav-wma::frame_bits: value {value} of field `{field}` does not fit {bits} bits",
            ),
            FrameBitsError::PlanMismatch { what } => {
                write!(f, "oxideav-wma::frame_bits: plan mismatch — {what}")
            }
            FrameBitsError::EscapeOverflow { run, abs_level } => write!(
                f,
                "oxideav-wma::frame_bits: pair (run {run}, |level| {abs_level}) fits neither the VLC alphabet nor the escape literal widths",
            ),
            FrameBitsError::CoefficientOverflow {
                position,
                coef_count,
            } => write!(
                f,
                "oxideav-wma::frame_bits: decoded run reaches position {position} in a {coef_count}-coefficient block",
            ),
            FrameBitsError::EscapeLevelZero => f.write_str(
                "oxideav-wma::frame_bits: escape literal level was zero (stream corruption)",
            ),
            FrameBitsError::Bitstream(e) => write!(f, "oxideav-wma::frame_bits: {e}"),
            FrameBitsError::Coef(e) => write!(f, "oxideav-wma::frame_bits: {e}"),
            FrameBitsError::Huffman(e) => write!(f, "oxideav-wma::frame_bits: {e}"),
        }
    }
}

impl std::error::Error for FrameBitsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            FrameBitsError::Bitstream(e) => Some(e),
            FrameBitsError::Coef(e) => Some(e),
            FrameBitsError::Huffman(e) => Some(e),
            _ => None,
        }
    }
}

impl From<BitstreamEnd> for FrameBitsError {
    fn from(e: BitstreamEnd) -> Self {
        FrameBitsError::Bitstream(e)
    }
}

impl From<CoefVlcError> for FrameBitsError {
    fn from(e: CoefVlcError) -> Self {
        FrameBitsError::Coef(e)
    }
}

impl From<HuffmanError> for FrameBitsError {
    fn from(e: HuffmanError) -> Self {
        FrameBitsError::Huffman(e)
    }
}

fn write_checked(
    writer: &mut BitWriter,
    field: &'static str,
    value: u32,
    bits: u8,
) -> Result<(), FrameBitsError> {
    if bits < 32 && u64::from(value) >= (1u64 << bits) {
        return Err(FrameBitsError::ValueTooWide { field, value, bits });
    }
    writer.write_bits(u64::from(value), bits);
    Ok(())
}

/// Emit one block in the staged B1..B6 order.
///
/// # Errors
///
/// [`FrameBitsError`] on any plan/block mismatch, VLC failure, or an
/// `(R, L)` event outside both the alphabet and the escape widths.
pub fn write_block(
    block: &WireBlock,
    plan: &BlockPlan<'_>,
    writer: &mut BitWriter,
) -> Result<(), FrameBitsError> {
    validate_plan(plan)?;
    if block.gain_symbols.len() != plan.gain_count {
        return Err(FrameBitsError::PlanMismatch {
            what: "gain symbol count",
        });
    }
    if block.stereo_coupling.is_some() != (plan.channels == 2) {
        return Err(FrameBitsError::PlanMismatch {
            what: "stereo flag presence",
        });
    }
    if block.scale_symbols.len()
        != if plan.envelope_coded {
            plan.scale_count
        } else {
            0
        }
    {
        return Err(FrameBitsError::PlanMismatch {
            what: "scale symbol count",
        });
    }
    if block.coefficients.len() != plan.coef_count {
        return Err(FrameBitsError::PlanMismatch {
            what: "coefficient count",
        });
    }

    // B1 — 7-bit block header.
    write_checked(
        writer,
        "block_header",
        u32::from(block.header),
        BLOCK_HEADER_BITS,
    )?;
    // B2 — gain sub-stream.
    for &s in &block.gain_symbols {
        plan.gain_vlc.encode_symbol(s, writer)?;
    }
    // B3 — stereo/coupling flag (2-channel only).
    if let Some(flag) = block.stereo_coupling {
        writer.write_bit(flag);
    }
    if plan.envelope_coded {
        // B4 — 5-bit envelope base field.
        write_checked(
            writer,
            "envelope_base",
            u32::from(block.envelope_base),
            ENVELOPE_BASE_BITS,
        )?;
        // B5 — scale sub-stream.
        for &s in &block.scale_symbols {
            plan.scale_vlc.encode_symbol(s, writer)?;
        }
    }
    // B6 — coefficient run-level sub-stream.
    write_coefficients(&block.coefficients, plan.coef_vlc, plan.escape, writer)
}

/// Parse one block in the staged B1..B6 order.
///
/// # Errors
///
/// [`FrameBitsError`] on truncation, an invalid codeword, or a run
/// stepping past the block's coefficient count.
pub fn read_block(
    plan: &BlockPlan<'_>,
    reader: &mut BitReader<'_>,
) -> Result<WireBlock, FrameBitsError> {
    validate_plan(plan)?;
    let header = reader.read_bits(BLOCK_HEADER_BITS)? as u8;
    let mut gain_symbols = Vec::with_capacity(plan.gain_count);
    for _ in 0..plan.gain_count {
        gain_symbols.push(plan.gain_vlc.decode_symbol(reader)?);
    }
    let stereo_coupling = if plan.channels == 2 {
        Some(reader.read_bit()?)
    } else {
        None
    };
    let (envelope_base, scale_symbols) = if plan.envelope_coded {
        let base = reader.read_bits(ENVELOPE_BASE_BITS)? as u8;
        let mut symbols = Vec::with_capacity(plan.scale_count);
        for _ in 0..plan.scale_count {
            symbols.push(plan.scale_vlc.decode_symbol(reader)?);
        }
        (base, symbols)
    } else {
        (0, Vec::new())
    };
    let coefficients = read_coefficients(plan.coef_count, plan.coef_vlc, plan.escape, reader)?;
    Ok(WireBlock {
        header,
        gain_symbols,
        stereo_coupling,
        envelope_base,
        scale_symbols,
        coefficients,
    })
}

fn validate_plan(plan: &BlockPlan<'_>) -> Result<(), FrameBitsError> {
    if plan.channels != 1 && plan.channels != 2 {
        return Err(FrameBitsError::PlanMismatch {
            what: "channel count (must be 1 or 2)",
        });
    }
    if plan.coef_count == 0 {
        return Err(FrameBitsError::PlanMismatch {
            what: "coefficient count (must be positive)",
        });
    }
    Ok(())
}

/// Emit the B6 coefficient run-level sub-stream for one block: run
/// gaps between non-zero coefficients coded jointly with the level
/// through the real VLC where the pair is in the alphabet, through
/// the escape (symbol 1 + literal run + literal level) otherwise; one
/// trailing sign bit per non-zero coefficient (`1` = negative, the
/// documented convention); the reserved end-of-block symbol (0) when
/// — and only when — trailing zeros remain (the sub-stream is
/// self-delimited by the coefficient count otherwise).
///
/// # Errors
///
/// [`FrameBitsError::EscapeOverflow`] when a pair fits neither the
/// alphabet nor the escape widths.
pub fn write_coefficients(
    coefficients: &[i32],
    vlc: &CoefVlc,
    escape: EscapeWidths,
    writer: &mut BitWriter,
) -> Result<(), FrameBitsError> {
    let mut run: u16 = 0;
    let mut emitted_up_to = 0usize;
    for (i, &value) in coefficients.iter().enumerate() {
        if value == 0 {
            run = run.saturating_add(1);
            continue;
        }
        let abs_level = value.unsigned_abs();
        let in_alphabet = u16::try_from(abs_level)
            .ok()
            .and_then(|lvl| vlc.symbol_for_pair(run, lvl));
        match in_alphabet {
            Some(symbol) => {
                vlc.encode_symbol(symbol, writer)?;
            }
            None => {
                if u32::from(run) > escape.max_run() || abs_level > escape.max_level() {
                    return Err(FrameBitsError::EscapeOverflow { run, abs_level });
                }
                vlc.encode_symbol(usize::from(crate::wire_tables::COEF_ESCAPE_SYMBOL), writer)?;
                writer.write_bits(u64::from(run), escape.run_bits);
                writer.write_bits(u64::from(abs_level), escape.level_bits);
            }
        }
        // Trailing sign bit per non-zero coefficient (1 = negative).
        writer.write_bit(value < 0);
        run = 0;
        emitted_up_to = i + 1;
    }
    // Trailing zeros: the reserved end-of-block symbol. A block whose
    // last coefficient is non-zero is self-delimited by the count.
    if emitted_up_to < coefficients.len() {
        vlc.encode_symbol(usize::from(crate::wire_tables::COEF_EOB_SYMBOL), writer)?;
    }
    Ok(())
}

/// Parse the B6 coefficient sub-stream of one block: decode run-level
/// events (VLC pairs, escapes, end-of-block) until `coef_count`
/// coefficients are placed or the end-of-block symbol arrives — the
/// staged self-delimiting rule.
///
/// # Errors
///
/// [`FrameBitsError::CoefficientOverflow`] when a decoded run steps
/// past `coef_count`; [`FrameBitsError::EscapeLevelZero`] on a
/// zero-magnitude escape literal; VLC/bitstream errors propagated.
pub fn read_coefficients(
    coef_count: usize,
    vlc: &CoefVlc,
    escape: EscapeWidths,
    reader: &mut BitReader<'_>,
) -> Result<Vec<i32>, FrameBitsError> {
    let mut coefficients = vec![0i32; coef_count];
    let mut position = 0usize;
    while position < coef_count {
        let symbol = vlc.decode_symbol(reader)?;
        let (run, abs_level) = match vlc.expand(symbol)? {
            CoefEvent::EndOfBlock => break,
            CoefEvent::Escape => {
                let run = reader.read_bits(escape.run_bits)? as u32;
                let abs_level = reader.read_bits(escape.level_bits)? as u32;
                if abs_level == 0 {
                    return Err(FrameBitsError::EscapeLevelZero);
                }
                (run, abs_level)
            }
            CoefEvent::Pair { run, abs_level } => (u32::from(run), u32::from(abs_level)),
        };
        let target = position + run as usize;
        if target >= coef_count {
            return Err(FrameBitsError::CoefficientOverflow {
                position: target,
                coef_count,
            });
        }
        // Trailing sign bit per non-zero coefficient (1 = negative).
        let negative = reader.read_bit()?;
        let magnitude = abs_level as i32;
        coefficients[target] = if negative { -magnitude } else { magnitude };
        position = target + 1;
    }
    Ok(coefficients)
}

/// One frame's demuxed wire fields: the staged S1/S2/S3 header, then
/// per-channel, per-block field sets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireFrame {
    /// The per-frame header fields.
    pub header: FrameHeaderFields,
    /// Per-channel block lists (`channel_blocks[ch][blk]`), matching
    /// the staged frame loop → per-channel → per-block nesting.
    pub channel_blocks: Vec<Vec<WireBlock>>,
}

/// Emit one frame: the S1/S2/S3 header, then every channel's blocks
/// in the staged nesting order.
///
/// `plans` is one [`BlockPlan`] per block position (uniform frames
/// pass the same geometry `blocks_per_channel` times); every plan
/// must agree on the channel count and match
/// `frame.channel_blocks`.
///
/// # Errors
///
/// [`FrameBitsError::PlanMismatch`] on any shape disagreement, plus
/// everything [`write_block`] raises.
pub fn write_frame(
    frame: &WireFrame,
    widths: &FrameFieldWidths,
    plans: &[BlockPlan<'_>],
    writer: &mut BitWriter,
) -> Result<(), FrameBitsError> {
    let channels = frame_channels(plans)?;
    if frame.channel_blocks.len() != usize::from(channels) {
        return Err(FrameBitsError::PlanMismatch {
            what: "channel list length",
        });
    }
    frame.header.write(widths, writer)?;
    for blocks in &frame.channel_blocks {
        if blocks.len() != plans.len() {
            return Err(FrameBitsError::PlanMismatch {
                what: "blocks per channel",
            });
        }
        for (block, plan) in blocks.iter().zip(plans) {
            write_block(block, plan, writer)?;
        }
    }
    Ok(())
}

/// Parse one frame: the S1/S2/S3 header, then every channel's blocks
/// in the staged nesting order.
///
/// # Errors
///
/// As [`read_block`], plus [`FrameBitsError::PlanMismatch`] when the
/// plans disagree on the channel count.
pub fn read_frame(
    widths: &FrameFieldWidths,
    plans: &[BlockPlan<'_>],
    reader: &mut BitReader<'_>,
) -> Result<WireFrame, FrameBitsError> {
    let channels = frame_channels(plans)?;
    let header = FrameHeaderFields::read(widths, reader)?;
    let mut channel_blocks = Vec::with_capacity(usize::from(channels));
    for _ in 0..channels {
        let mut blocks = Vec::with_capacity(plans.len());
        for plan in plans {
            blocks.push(read_block(plan, reader)?);
        }
        channel_blocks.push(blocks);
    }
    Ok(WireFrame {
        header,
        channel_blocks,
    })
}

fn frame_channels(plans: &[BlockPlan<'_>]) -> Result<u8, FrameBitsError> {
    let Some(first) = plans.first() else {
        return Err(FrameBitsError::PlanMismatch {
            what: "empty plan list",
        });
    };
    if plans.iter().any(|p| p.channels != first.channels) {
        return Err(FrameBitsError::PlanMismatch {
            what: "plans disagree on channel count",
        });
    }
    Ok(first.channels)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coef_vlc::CoefDecodeMode;

    fn vlcs() -> (CoefVlc, GainVlc, ScaleVlc) {
        (
            CoefVlc::new(CoefDecodeMode::Mode3).unwrap(),
            GainVlc::new(),
            ScaleVlc::new(),
        )
    }

    fn plan<'a>(
        coef: &'a CoefVlc,
        gain: &'a GainVlc,
        scale: &'a ScaleVlc,
        channels: u8,
        m: usize,
    ) -> BlockPlan<'a> {
        BlockPlan {
            coef_vlc: coef,
            gain_vlc: gain,
            scale_vlc: scale,
            escape: EscapeWidths::new(9, 12).unwrap(),
            channels,
            envelope_coded: true,
            gain_count: 1,
            scale_count: 3,
            coef_count: m,
        }
    }

    #[test]
    fn frame_header_round_trips_in_the_staged_order() {
        let widths = FrameFieldWidths::new(8, 5).unwrap();
        let header = FrameHeaderFields {
            reservoir_offset: 0xA5,
            side_field: 0x13,
            flag: true,
        };
        let mut w = BitWriter::new();
        header.write(&widths, &mut w).unwrap();
        assert_eq!(w.bit_len(), 8 + 5 + 1);
        // Staged order pin: S1 (MSB-first) then S2 then S3.
        // 10100101 10011 1 -> bytes 10100101 10011100
        let bytes = w.into_bytes();
        assert_eq!(bytes, vec![0b1010_0101, 0b1001_1100]);
        let mut r = BitReader::with_bit_len(&bytes, 14);
        assert_eq!(FrameHeaderFields::read(&widths, &mut r).unwrap(), header);
    }

    #[test]
    fn header_widths_and_values_are_validated() {
        assert_eq!(
            FrameFieldWidths::new(0, 5),
            Err(FrameBitsError::WidthOutOfRange {
                field: "byte_offset_bits",
                bits: 0
            })
        );
        assert_eq!(
            FrameFieldWidths::new(8, 33),
            Err(FrameBitsError::WidthOutOfRange {
                field: "side_field_bits",
                bits: 33
            })
        );
        let widths = FrameFieldWidths::new(4, 4).unwrap();
        let header = FrameHeaderFields {
            reservoir_offset: 16, // does not fit 4 bits
            side_field: 0,
            flag: false,
        };
        let mut w = BitWriter::new();
        assert_eq!(
            header.write(&widths, &mut w),
            Err(FrameBitsError::ValueTooWide {
                field: "reservoir_offset",
                value: 16,
                bits: 4
            })
        );
    }

    #[test]
    fn coefficients_round_trip_with_pairs_escapes_and_eob() {
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(9, 12).unwrap();
        // (run 0, |level| 1) pairs, a large-level escape event, a
        // long-run escape event, trailing zeros -> EOB.
        let mut coeffs = vec![0i32; 256];
        coeffs[0] = 1;
        coeffs[1] = -1;
        coeffs[2] = 2000; // beyond mode-3 max level 70 -> escape
        coeffs[200] = -3; // run 197 beyond mode-3 max run 112 -> escape
        let mut w = BitWriter::new();
        write_coefficients(&coeffs, &coef, escape, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(
            read_coefficients(256, &coef, escape, &mut r).unwrap(),
            coeffs
        );
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn exactly_full_block_is_self_delimited_without_eob() {
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(9, 12).unwrap();
        let coeffs = vec![3i32, -1, 0, 0, 2]; // last coefficient non-zero
        let mut w = BitWriter::new();
        write_coefficients(&coeffs, &coef, escape, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(read_coefficients(5, &coef, escape, &mut r).unwrap(), coeffs);
        // Nothing left: no end-of-block symbol was spent.
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn all_zero_block_is_a_lone_eob_symbol() {
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(9, 12).unwrap();
        let coeffs = vec![0i32; 64];
        let mut w = BitWriter::new();
        write_coefficients(&coeffs, &coef, escape, &mut w).unwrap();
        // Mode-3 symbol 0 (EOB) is 12 bits in the staged table.
        assert_eq!(w.bit_len(), 12);
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(
            read_coefficients(64, &coef, escape, &mut r).unwrap(),
            coeffs
        );
    }

    #[test]
    fn sign_bit_follows_each_nonzero_coefficient() {
        // Layout pin: (run 0, |level| 1) is symbol 2 = "00" in every
        // staged table; with a positive then a negative coefficient
        // the stream must be 00|0|00|1 + EOB.
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(9, 12).unwrap();
        let coeffs = vec![1i32, -1, 0, 0];
        let mut w = BitWriter::new();
        write_coefficients(&coeffs, &coef, escape, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        // 00 0 00 1 then EOB "111110101000" (12 bits) = 18 bits.
        assert_eq!(bit_len, 18);
        assert_eq!(bytes[0], 0b0000_0111);
        assert_eq!(bytes[1], 0b1110_1010);
        assert_eq!(bytes[2] & 0b1100_0000, 0b0000_0000);
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(read_coefficients(4, &coef, escape, &mut r).unwrap(), coeffs);
    }

    #[test]
    fn escape_wire_shape_is_symbol_run_level_sign() {
        // Layout pin for the corrected escape reading: symbol 1 then
        // the literal run then the literal level then the sign bit.
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(4, 8).unwrap();
        let mut coeffs = vec![0i32; 8];
        coeffs[2] = -200; // run 2, level 200: mode-3 has no such pair
        let mut w = BitWriter::new();
        write_coefficients(&coeffs, &coef, escape, &mut w).unwrap();
        // Mode-3 escape symbol 1 = "101100" (6 bits) + run 0010 (4)
        // + level 11001000 (8) + sign 1 + EOB (12 bits).
        let expected_bits = 6 + 4 + 8 + 1 + 12;
        assert_eq!(w.bit_len(), expected_bits);
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        assert_eq!(bytes[0], 0b1011_0000);
        assert_eq!(bytes[1], 0b1011_0010);
        assert_eq!(bytes[2] & 0b1100_0000, 0b0000_0000); // 00 of level tail + sign 1 is next
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(read_coefficients(8, &coef, escape, &mut r).unwrap(), coeffs);
    }

    #[test]
    fn escape_overflow_is_a_typed_error() {
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(4, 8).unwrap();
        let mut coeffs = vec![0i32; 64];
        coeffs[0] = 1; // (0, 1) = symbol 2, in the alphabet
        coeffs[63] = 300; // run 62 > 15 AND level 300 > 255: escape cannot carry it
        let mut w = BitWriter::new();
        let err = write_coefficients(&coeffs, &coef, escape, &mut w).unwrap_err();
        assert_eq!(
            err,
            FrameBitsError::EscapeOverflow {
                run: 62,
                abs_level: 300
            }
        );
    }

    #[test]
    fn overlong_run_in_the_stream_is_a_typed_error() {
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(6, 8).unwrap();
        // Hand-compose: escape symbol "101100" + run 63 + level 5 + sign 0.
        let mut w = BitWriter::new();
        w.write_bits(0b101100, 6);
        w.write_bits(63, 6);
        w.write_bits(5, 8);
        w.write_bit(false);
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(
            read_coefficients(32, &coef, escape, &mut r),
            Err(FrameBitsError::CoefficientOverflow {
                position: 63,
                coef_count: 32
            })
        );
    }

    #[test]
    fn zero_escape_level_is_a_typed_error() {
        let (coef, _, _) = vlcs();
        let escape = EscapeWidths::new(6, 8).unwrap();
        let mut w = BitWriter::new();
        w.write_bits(0b101100, 6); // escape symbol
        w.write_bits(3, 6);
        w.write_bits(0, 8); // zero level
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(
            read_coefficients(32, &coef, escape, &mut r),
            Err(FrameBitsError::EscapeLevelZero)
        );
    }

    #[test]
    fn block_round_trips_mono_and_stereo() {
        let (coef, gain, scale) = vlcs();
        for channels in [1u8, 2] {
            let plan = plan(&coef, &gain, &scale, channels, 128);
            let mut coefficients = vec![0i32; 128];
            coefficients[0] = 4;
            coefficients[9] = -2;
            coefficients[127] = 1;
            let block = WireBlock {
                header: 0x2A,
                gain_symbols: vec![18],
                stereo_coupling: (channels == 2).then_some(true),
                envelope_base: 7,
                scale_symbols: vec![60, 59, 61],
                coefficients,
            };
            let mut w = BitWriter::new();
            write_block(&block, &plan, &mut w).unwrap();
            let bit_len = w.bit_len();
            let bytes = w.into_bytes();
            let mut r = BitReader::with_bit_len(&bytes, bit_len);
            assert_eq!(
                read_block(&plan, &mut r).unwrap(),
                block,
                "channels {channels}"
            );
            assert_eq!(r.remaining_bits(), 0);
        }
    }

    #[test]
    fn block_field_order_is_the_staged_b1_to_b6_sequence() {
        // Full-layout pin over hand-checkable codewords:
        // B1 0101010 | B2 gain 18 "1011" | B4 00111 | B5 scale 60 "0"
        // ×3 | B6 EOB "111110101000".
        let (coef, gain, scale) = vlcs();
        let mut p = plan(&coef, &gain, &scale, 1, 16);
        p.scale_count = 3;
        let block = WireBlock {
            header: 0x2A,
            gain_symbols: vec![18],
            stereo_coupling: None,
            envelope_base: 7,
            scale_symbols: vec![60, 60, 60],
            coefficients: vec![0; 16],
        };
        let mut w = BitWriter::new();
        write_block(&block, &p, &mut w).unwrap();
        assert_eq!(w.bit_len(), 7 + 4 + 5 + 3 + 12);
        let bytes = w.into_bytes();
        // 0101010 1011 00111 000 111110101000
        assert_eq!(
            bytes,
            vec![0b0101_0101, 0b0110_0111, 0b0001_1111, 0b0101_0000]
        );
    }

    #[test]
    fn envelope_gate_removes_b4_and_b5() {
        let (coef, gain, scale) = vlcs();
        let mut p = plan(&coef, &gain, &scale, 1, 16);
        p.envelope_coded = false;
        let block = WireBlock {
            header: 1,
            gain_symbols: vec![18],
            stereo_coupling: None,
            envelope_base: 0,
            scale_symbols: vec![],
            coefficients: vec![0; 16],
        };
        let mut w = BitWriter::new();
        write_block(&block, &p, &mut w).unwrap();
        // B1 7 + gain 4 + EOB 12 only.
        assert_eq!(w.bit_len(), 23);
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(read_block(&p, &mut r).unwrap(), block);
    }

    #[test]
    fn plan_mismatches_are_typed_errors() {
        let (coef, gain, scale) = vlcs();
        let p = plan(&coef, &gain, &scale, 1, 16);
        let good = WireBlock {
            header: 1,
            gain_symbols: vec![18],
            stereo_coupling: None,
            envelope_base: 0,
            scale_symbols: vec![60, 60, 60],
            coefficients: vec![0; 16],
        };
        let mut w = BitWriter::new();

        let mut bad = good.clone();
        bad.gain_symbols = vec![];
        assert!(matches!(
            write_block(&bad, &p, &mut w),
            Err(FrameBitsError::PlanMismatch { .. })
        ));
        let mut bad = good.clone();
        bad.stereo_coupling = Some(false);
        assert!(matches!(
            write_block(&bad, &p, &mut w),
            Err(FrameBitsError::PlanMismatch { .. })
        ));
        let mut bad = good.clone();
        bad.coefficients = vec![0; 15];
        assert!(matches!(
            write_block(&bad, &p, &mut w),
            Err(FrameBitsError::PlanMismatch { .. })
        ));
        let mut bad_plan = p.clone();
        bad_plan.channels = 3;
        assert!(matches!(
            write_block(&good, &bad_plan, &mut w),
            Err(FrameBitsError::PlanMismatch { .. })
        ));
    }

    #[test]
    fn frame_round_trips_across_channels_and_blocks() {
        let (coef, gain, scale) = vlcs();
        let widths = FrameFieldWidths::new(10, 6).unwrap();
        let p = plan(&coef, &gain, &scale, 2, 64);
        let plans = vec![p.clone(), p];
        let mk_block = |seed: i32| {
            let mut coefficients = vec![0i32; 64];
            coefficients[(seed as usize) % 64] = seed;
            coefficients[63] = -seed;
            WireBlock {
                header: (seed as u8) & 0x7f,
                gain_symbols: vec![(seed as usize) % 37],
                stereo_coupling: Some(seed % 2 == 0),
                envelope_base: (seed as u8) & 0x1f,
                scale_symbols: vec![60, (seed as usize) % 121, 0],
                coefficients,
            }
        };
        let frame = WireFrame {
            header: FrameHeaderFields {
                reservoir_offset: 513,
                side_field: 42,
                flag: false,
            },
            channel_blocks: vec![
                vec![mk_block(3), mk_block(17)],
                vec![mk_block(5), mk_block(29)],
            ],
        };
        let mut w = BitWriter::new();
        write_frame(&frame, &widths, &plans, &mut w).unwrap();
        let bit_len = w.bit_len();
        let bytes = w.into_bytes();
        let mut r = BitReader::with_bit_len(&bytes, bit_len);
        assert_eq!(read_frame(&widths, &plans, &mut r).unwrap(), frame);
        assert_eq!(r.remaining_bits(), 0);
    }

    #[test]
    fn every_alphabet_pair_survives_the_wire() {
        // Exhaustive self-consistency sweep: every (run, |level|)
        // pair of every staged primary alphabet goes through the B6
        // wire layer — one pair per exactly-full block (no EOB, the
        // self-delimiting path) with alternating signs.
        use crate::coef_vlc::CoefDecodeMode;
        let escape = EscapeWidths::new(9, 12).unwrap();
        for mode in [
            CoefDecodeMode::Mode1,
            CoefDecodeMode::Mode2,
            CoefDecodeMode::Mode3,
        ] {
            let vlc = CoefVlc::new(mode).unwrap();
            for symbol in 2..vlc.symbol_count() {
                let CoefEvent::Pair { run, abs_level } = vlc.expand(symbol).unwrap() else {
                    panic!("symbol {symbol} of {mode:?} must be a pair");
                };
                let m = usize::from(run) + 1;
                let mut coeffs = vec![0i32; m];
                let sign = if symbol % 2 == 0 { 1 } else { -1 };
                coeffs[m - 1] = sign * i32::from(abs_level);
                let mut w = BitWriter::new();
                write_coefficients(&coeffs, &vlc, escape, &mut w).unwrap();
                // The pair is in the alphabet: it must ride the VLC
                // symbol (its length + 1 sign bit), not the escape.
                let expected = usize::from(vlc.code().length_of(symbol).unwrap()) + 1;
                assert_eq!(w.bit_len(), expected, "{mode:?} symbol {symbol}");
                let bit_len = w.bit_len();
                let bytes = w.into_bytes();
                let mut r = BitReader::with_bit_len(&bytes, bit_len);
                assert_eq!(
                    read_coefficients(m, &vlc, escape, &mut r).unwrap(),
                    coeffs,
                    "{mode:?} symbol {symbol}"
                );
                assert_eq!(r.remaining_bits(), 0);
            }
        }
    }

    #[test]
    fn escape_boundary_values_survive_the_wire() {
        // The widest values the escape literals can carry, plus the
        // smallest, on a pair guaranteed outside every alphabet.
        let (coef, _, _) = vlcs();
        for (run_bits, level_bits) in [(5u8, 9u8), (9, 12), (12, 16)] {
            let escape = EscapeWidths::new(run_bits, level_bits).unwrap();
            let max_run = escape.max_run() as usize;
            let max_level = escape.max_level() as i32;
            let m = max_run + 2;
            for (run, level) in [
                (max_run, max_level),
                (max_run, -max_level),
                (0, max_level), // level beyond every staged table
            ] {
                let mut coeffs = vec![0i32; m];
                coeffs[run] = level;
                let mut w = BitWriter::new();
                write_coefficients(&coeffs, &coef, escape, &mut w).unwrap();
                let bit_len = w.bit_len();
                let bytes = w.into_bytes();
                let mut r = BitReader::with_bit_len(&bytes, bit_len);
                assert_eq!(
                    read_coefficients(m, &coef, escape, &mut r).unwrap(),
                    coeffs,
                    "widths ({run_bits}, {level_bits}) run {run} level {level}"
                );
            }
        }
    }

    #[test]
    fn arbitrary_bytes_never_panic_the_frame_parser() {
        // Robustness: whatever bits arrive, read_frame returns a
        // Result — no panics, no unbounded work.
        let (coef, gain, scale) = vlcs();
        let plans = vec![plan(&coef, &gain, &scale, 2, 64)];
        let widths = FrameFieldWidths::new(11, 7).unwrap();
        let mut state = 0x5EEDu64;
        for round in 0..200 {
            let len = (round % 64) + 1;
            let bytes: Vec<u8> = (0..len)
                .map(|_| {
                    state = state
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    (state >> 33) as u8
                })
                .collect();
            let mut r = BitReader::new(&bytes);
            let _ = read_frame(&widths, &plans, &mut r);
        }
    }

    #[test]
    fn truncated_frame_fails_cleanly() {
        let (coef, gain, scale) = vlcs();
        let widths = FrameFieldWidths::new(10, 6).unwrap();
        let plans = vec![plan(&coef, &gain, &scale, 1, 64)];
        let bytes = [0xFFu8; 1];
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            read_frame(&widths, &plans, &mut r),
            Err(FrameBitsError::Bitstream(_))
        ));
    }
}

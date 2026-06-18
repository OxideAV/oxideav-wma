//! Rate-dependent stream-setup parameters.
//!
//! The staged wiki snapshot at
//! `docs/audio/wma/wiki/Windows_Media_Audio.wiki` lists, under
//! "init rate dependent parameters", a small set of **deterministic
//! formulas** the decoder evaluates once at stream-open time from the
//! already-parsed [`WmaHeader`] fields. Paraphrased into a structured
//! form, the snapshot states:
//!
//! * `use noise coding = 1 as a default`.
//! * `high frequency = sample rate / 2`.
//! * `bits/sec = bitrate / (channels * sr)`.
//! * `byte offset bits = log2(bps * frame length / 8) + 2`.
//! * "compute high frequency value and choose if noise coding should
//!   be activated based on channels and sr".
//!
//! The first four items are exact, closed-form derivations of the
//! container-supplied fields — they introduce no codeword tables, no
//! per-block decisions, and no bitstream parsing, so they are
//! implementable directly from the staged snapshot. This module
//! computes exactly those.
//!
//! The fifth item — the *activation* decision that may override the
//! `use noise coding = 1` default "based on channels and sr" — is
//! named by the snapshot but its threshold rule is **not** spelled
//! out (which channel counts / sample rates flip the default off).
//! That selection is therefore a **DOCS-GAP**: this module ships the
//! patent/​wiki-stated **default** (`noise_coding = true`) and exposes
//! it as an overridable field, never fabricating a threshold. See
//! [`SetupParams::noise_coding`].
//!
//! Anything past these setup scalars — the exponent-band partition
//! tables, the Huffman codebooks, the critical-frequency curves, the
//! superframe/packet byte layout — is not specified by any staged
//! document (the patent trace marks every one `[GAP]` in its §9) and
//! is not computed here.

use crate::header::WmaHeader;

/// Rate-dependent stream-setup scalars, derived once at stream-open
/// time from a parsed [`WmaHeader`].
///
/// Every field is a closed-form function of the container-supplied
/// header fields per the wiki "init rate dependent parameters"
/// section, with the single exception of [`SetupParams::noise_coding`],
/// whose *activation override* is a DOCS-GAP (see the module docs); the
/// field ships the wiki-stated default and is overridable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetupParams {
    /// Highest coded frequency in Hz, `sample_rate / 2` (the Nyquist
    /// frequency). The wiki: "high frequency = sample rate / 2".
    pub high_frequency: u32,
    /// Per-channel coding rate in bits/second/channel,
    /// `bit_rate / (channels * sample_rate)`. The wiki:
    /// "bits/sec = bitrate / (channels * sr)". Despite the wiki's
    /// "bits/sec" label this is the dimensionless per-sample-per-channel
    /// bit budget (bits ÷ (channels · samples-per-second)); it is the
    /// `bps` term the `byte_offset_bits` formula consumes.
    pub bits_per_sample: u32,
    /// Number of bits used to encode the per-frame byte offset field,
    /// `log2(bps * frame_length / 8) + 2`. The wiki: "byte offset bits
    /// = log2(bps * frame length / 8) + 2". `log2` here is the
    /// integer floor logarithm (bit index of the most-significant set
    /// bit), matching the wiki's earlier `frame length bits =
    /// log2(frame length)` usage where `frame_length` is an exact
    /// power of two.
    pub byte_offset_bits: u32,
    /// Whether noise coding is active for this stream. The wiki states
    /// `use noise coding = 1 as a default` and separately that the
    /// activation may be reconsidered "based on channels and sr"; that
    /// override rule is a DOCS-GAP, so this field ships the default
    /// `true` and is overridable by a caller that has determined the
    /// activation by black-box observation.
    pub noise_coding: bool,
}

/// The wiki-stated default for [`SetupParams::noise_coding`]
/// (`use noise coding = 1 as a default`).
pub const DEFAULT_NOISE_CODING: bool = true;

impl SetupParams {
    /// Derive the rate-dependent setup scalars from a parsed header.
    ///
    /// All four scalars are exact closed-form functions of the header
    /// fields. [`SetupParams::noise_coding`] is initialised to the
    /// wiki default [`DEFAULT_NOISE_CODING`]; use
    /// [`SetupParams::with_noise_coding`] to override it once the
    /// activation rule (a DOCS-GAP) is known for a given stream.
    ///
    /// The arithmetic is saturating / floor in the few degenerate
    /// cases the container could present (e.g. a `bps * frame_length`
    /// product whose `/ 8` is zero): `byte_offset_bits` is then the
    /// `+ 2` constant alone, because `log2(0)` is defined here as `0`
    /// rather than panicking — the wiki gives no behaviour for a
    /// zero argument and a setup-time panic on a malformed header
    /// would be a worse failure mode than a clamped scalar.
    pub fn from_header(header: &WmaHeader) -> Self {
        let high_frequency = header.sample_rate / 2;

        // bits/sec = bitrate / (channels * sr). Guard the product
        // against a zero channel count (a malformed container field)
        // so the division is well-defined; `sample_rate` is already
        // non-zero (the header parser rejects `sample_rate == 0`).
        let denom = (header.channels as u64) * (header.sample_rate as u64);
        let bits_per_sample = (header.bit_rate as u64).checked_div(denom).unwrap_or(0) as u32;

        // byte offset bits = log2(bps * frame length / 8) + 2.
        let product = (bits_per_sample as u64) * (header.frame_length as u64) / 8;
        let byte_offset_bits = floor_log2(product) + 2;

        SetupParams {
            high_frequency,
            bits_per_sample,
            byte_offset_bits,
            noise_coding: DEFAULT_NOISE_CODING,
        }
    }

    /// Return a copy with [`SetupParams::noise_coding`] overridden.
    ///
    /// The activation rule the wiki names but does not specify ("choose
    /// if noise coding should be activated based on channels and sr")
    /// is a DOCS-GAP; a caller that has determined the activation for a
    /// concrete stream by black-box observation threads it in here
    /// rather than this module fabricating a threshold.
    #[must_use]
    pub fn with_noise_coding(mut self, noise_coding: bool) -> Self {
        self.noise_coding = noise_coding;
        self
    }
}

/// Integer floor base-2 logarithm: the index of the most-significant
/// set bit, with `floor_log2(0) == 0`.
///
/// The wiki's `byte offset bits` formula and its `frame length bits =
/// log2(frame length)` rule both use `log2` over exact-power-of-two or
/// near-power-of-two integer arguments; the integer floor logarithm is
/// the matching operation. `0` is mapped to `0` (rather than the
/// mathematically-undefined `-∞`) so a degenerate setup argument
/// clamps instead of panicking — see [`SetupParams::from_header`].
#[inline]
fn floor_log2(value: u64) -> u32 {
    if value == 0 {
        0
    } else {
        63 - value.leading_zeros()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::Version;

    fn header(sample_rate: u32, channels: u8, bit_rate: u32, frame_length: u16) -> WmaHeader {
        WmaHeader {
            version: Version::V2,
            sample_rate,
            channels,
            bit_rate,
            block_align: 0,
            flags1: 0,
            flags2: 0,
            exp_vlc: false,
            bit_reservoir: false,
            variable_block_length: false,
            // frame_length_bits is not consumed by the setup formulas;
            // only frame_length (the sample count) is.
            frame_length_bits: frame_length.trailing_zeros() as u8,
            frame_length,
        }
    }

    // ---------- floor_log2 ----------

    #[test]
    fn floor_log2_powers_of_two() {
        assert_eq!(floor_log2(1), 0);
        assert_eq!(floor_log2(2), 1);
        assert_eq!(floor_log2(256), 8);
        assert_eq!(floor_log2(2048), 11);
        assert_eq!(floor_log2(4096), 12);
    }

    #[test]
    fn floor_log2_non_powers_floor_down() {
        assert_eq!(floor_log2(3), 1);
        assert_eq!(floor_log2(255), 7);
        assert_eq!(floor_log2(257), 8);
    }

    #[test]
    fn floor_log2_zero_is_zero() {
        assert_eq!(floor_log2(0), 0);
    }

    // ---------- high_frequency = sample_rate / 2 ----------

    #[test]
    fn high_frequency_is_nyquist() {
        let p = SetupParams::from_header(&header(44_100, 2, 128_000, 2048));
        assert_eq!(p.high_frequency, 22_050);
        let p = SetupParams::from_header(&header(22_050, 1, 64_000, 1024));
        assert_eq!(p.high_frequency, 11_025);
        let p = SetupParams::from_header(&header(8_000, 1, 16_000, 512));
        assert_eq!(p.high_frequency, 4_000);
    }

    // ---------- bits_per_sample = bitrate / (channels * sr) ----------

    #[test]
    fn bits_per_sample_stereo() {
        // 128000 / (2 * 44100) = 128000 / 88200 = 1 (floor).
        let p = SetupParams::from_header(&header(44_100, 2, 128_000, 2048));
        assert_eq!(p.bits_per_sample, 1);
    }

    #[test]
    fn bits_per_sample_high_rate_per_channel() {
        // 192000 / (1 * 8000) = 24.
        let p = SetupParams::from_header(&header(8_000, 1, 192_000, 512));
        assert_eq!(p.bits_per_sample, 24);
    }

    #[test]
    fn bits_per_sample_mono_vs_stereo_halves() {
        let mono = SetupParams::from_header(&header(16_000, 1, 96_000, 512));
        let stereo = SetupParams::from_header(&header(16_000, 2, 96_000, 512));
        // mono: 96000/16000 = 6; stereo: 96000/32000 = 3.
        assert_eq!(mono.bits_per_sample, 6);
        assert_eq!(stereo.bits_per_sample, 3);
    }

    #[test]
    fn bits_per_sample_zero_channels_is_zero_not_panic() {
        // Malformed container field: channels == 0. The division is
        // guarded so it clamps to 0 rather than dividing by zero.
        let p = SetupParams::from_header(&header(44_100, 0, 128_000, 2048));
        assert_eq!(p.bits_per_sample, 0);
        // byte_offset_bits then sees a zero product → log2(0)+2 = 2.
        assert_eq!(p.byte_offset_bits, 2);
    }

    // ---------- byte_offset_bits = log2(bps * frame_length / 8) + 2 ----------

    #[test]
    fn byte_offset_bits_formula() {
        // bps = 24 (8000 Hz mono, 192000 bps), frame_length = 512.
        // 24 * 512 / 8 = 1536; log2(1536) = 10; + 2 = 12.
        let p = SetupParams::from_header(&header(8_000, 1, 192_000, 512));
        assert_eq!(p.bits_per_sample, 24);
        assert_eq!(p.byte_offset_bits, 12);
    }

    #[test]
    fn byte_offset_bits_small_product_clamps_to_plus_two() {
        // bps = 1, frame_length = 2048: 1 * 2048 / 8 = 256;
        // log2(256) = 8; + 2 = 10.
        let p = SetupParams::from_header(&header(44_100, 2, 128_000, 2048));
        assert_eq!(p.bits_per_sample, 1);
        assert_eq!(p.byte_offset_bits, 10);
    }

    #[test]
    fn byte_offset_bits_zero_product_is_two() {
        // bps = 0 (bit_rate below one bit/sample): product = 0 →
        // log2(0) defined as 0 → byte_offset_bits = 2.
        let p = SetupParams::from_header(&header(44_100, 2, 1_000, 2048));
        assert_eq!(p.bits_per_sample, 0);
        assert_eq!(p.byte_offset_bits, 2);
    }

    // ---------- noise_coding default + override ----------

    #[test]
    fn noise_coding_defaults_to_wiki_default() {
        let p = SetupParams::from_header(&header(44_100, 2, 128_000, 2048));
        assert!(p.noise_coding);
        assert_eq!(p.noise_coding, DEFAULT_NOISE_CODING);
    }

    #[test]
    fn noise_coding_override_flips_the_default() {
        let p =
            SetupParams::from_header(&header(44_100, 2, 128_000, 2048)).with_noise_coding(false);
        assert!(!p.noise_coding);
        // The other scalars are untouched by the override.
        assert_eq!(p.high_frequency, 22_050);
        assert_eq!(p.bits_per_sample, 1);
        assert_eq!(p.byte_offset_bits, 10);
    }

    #[test]
    fn override_is_idempotent_on_same_value() {
        let p = SetupParams::from_header(&header(16_000, 1, 64_000, 512));
        assert_eq!(p, p.with_noise_coding(true));
    }

    // ---------- end-to-end through the real header parser ----------

    #[test]
    fn derives_from_parsed_header() {
        // Parse a real v2 header, then derive the setup scalars from
        // it, confirming the two stages chain.
        let h = WmaHeader::parse(Version::V2, 44_100, 2, 128_000, 1024, &[0; 6]).unwrap();
        let p = SetupParams::from_header(&h);
        assert_eq!(p.high_frequency, h.sample_rate / 2);
        assert!(p.noise_coding);
        // frame_length for 44100/v2 is 2048; bps = 1; 1*2048/8 = 256;
        // log2 = 8; +2 = 10.
        assert_eq!(h.frame_length, 2048);
        assert_eq!(p.byte_offset_bits, 10);
    }

    #[test]
    fn setup_is_copy_and_eq() {
        let p = SetupParams::from_header(&header(44_100, 2, 128_000, 2048));
        let q = p; // Copy
        assert_eq!(p, q);
    }
}

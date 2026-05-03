//! WMA v1 (codec ID `0x0160`) decoder front-end.
//!
//! v1's only variant-specific input is the 4-byte extradata: the
//! `flags2` field lives at bytes 2..3 (little-endian). Everything else
//! flows from the WAVEFORMATEX (sample rate, channels, bit rate / block
//! align) and the bitstream itself.

use crate::common::{Version, WmaContext};
use oxideav_core::{Error, Result};

/// Build a fresh WMA v1 decoder context from the WAVEFORMATEX-trailing
/// extradata blob plus the surrounding sample rate / channels / bit rate.
pub fn make_context(
    sample_rate: u32,
    channels: u16,
    bit_rate: u32,
    extradata: &[u8],
) -> Result<WmaContext> {
    if extradata.len() < 4 {
        return Err(Error::invalid("wmav1: extradata must be at least 4 bytes"));
    }
    let flags2 = u16::from_le_bytes([extradata[2], extradata[3]]);
    WmaContext::new(Version::V1, sample_rate, channels, bit_rate, flags2)
}

//! WMA v2 (codec ID `0x0161`) decoder front-end.
//!
//! v2's extradata is at least 6 bytes; `flags2` lives at bytes 4..5
//! (little-endian). v2 normalises the sample rate to its bucket
//! boundary before deriving `frame_len_bits`.

use crate::common::{Version, WmaContext};
use oxideav_core::{Error, Result};

/// Build a fresh WMA v2 decoder context from the WAVEFORMATEX-trailing
/// extradata blob plus the surrounding sample rate / channels / bit rate.
pub fn make_context(
    sample_rate: u32,
    channels: u16,
    bit_rate: u32,
    extradata: &[u8],
) -> Result<WmaContext> {
    if extradata.len() < 6 {
        return Err(Error::invalid("wmav2: extradata must be at least 6 bytes"));
    }
    let flags2 = u16::from_le_bytes([extradata[4], extradata[5]]);
    WmaContext::new(Version::V2, sample_rate, channels, bit_rate, flags2)
}

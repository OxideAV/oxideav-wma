#![no_main]

//! Fuzz: `WmaHeader::parse` + the open-time derivations, for both
//! versions, against arbitrary container fields and extradata.
//!
//! Contract: pure panic-freedom, plus the internal-consistency
//! invariants every accepted header must satisfy —
//!
//! * the staged frame-length rule (`frame_length_bits ∈ {9, 10, 11}`,
//!   `frame_length = 1 << frame_length_bits`), so
//!   `long_block_size()` is infallible on a parsed header;
//! * the VBL-field gating (`variable_block_length_field()` is
//!   `Some(flags2 >> 3)` exactly when `flags2` bit 2 is set);
//! * the setup scalars (`high_frequency = sample_rate / 2`,
//!   `byte_offset_bits >= 2` — the `+ 2` floor of the staged
//!   formula).

use libfuzzer_sys::fuzz_target;
use oxideav_wma::setup::SetupParams;
use oxideav_wma::{Version, WmaHeader};

fuzz_target!(|data: &[u8]| {
    if data.len() < 11 {
        return;
    }
    let sample_rate = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let channels = data[4];
    let bit_rate = u32::from_le_bytes([data[5], data[6], data[7], data[8]]);
    let block_align = u16::from_le_bytes([data[9], data[10]]);
    let extradata = &data[11..];

    for version in [Version::V1, Version::V2] {
        let Ok(h) = WmaHeader::parse(
            version,
            sample_rate,
            channels,
            bit_rate,
            block_align,
            extradata,
        ) else {
            continue;
        };

        assert!(
            (9..=11).contains(&h.frame_length_bits),
            "frame_length_bits {} outside the staged rule",
            h.frame_length_bits,
        );
        assert_eq!(h.frame_length, 1u16 << h.frame_length_bits);
        h.long_block_size()
            .expect("9..=11 always maps onto the patent block-size set");

        assert_eq!(
            h.variable_block_length_field().is_some(),
            h.variable_block_length,
        );
        if let Some(field) = h.variable_block_length_field() {
            assert_eq!(field, h.flags2 >> 3);
            assert!(field <= 0x1FFF, "the VBL field is 13 bits wide");
        }

        let s = SetupParams::from_header(&h);
        assert_eq!(s.high_frequency, h.sample_rate / 2);
        assert!(s.byte_offset_bits >= 2);
        assert!(s.noise_coding, "the wiki default is use-noise-coding = 1");
    }
});

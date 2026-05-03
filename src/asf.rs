//! Minimal ASF demuxer — just enough to feed integration tests.
//!
//! ASF Header Object → Stream Properties Object → WAVEFORMATEX. Then
//! the Data Object is followed by a sequence of fixed-size container
//! packets whose size is given by File Properties' `min_packet_size`
//! (== `max_packet_size` for ASF). Inside each packet a small
//! "Payload Parsing Information" header introduces one or more
//! payloads; each payload's data bytes get concatenated and then
//! re-sliced into WMA frames of `WAVEFORMATEX::nBlockAlign` bytes
//! each (the "superframe" size in WMA v1/v2 vocabulary).
//!
//! This is **not** a production ASF reader — multi-stream files,
//! seeking, error correction, and the compressed-payload-type variant
//! are all out of scope. It handles every file `ffmpeg -c:a wmav{1,2}`
//! produces with default options.
//!
//! Spec reference: Microsoft *Advanced Systems Format Specification*,
//! revision 01.20.06 (January 2007), §3 (Header Object), §5 (Data
//! Object), §5.2 (Data Packet structure).

use oxideav_core::{Error, Result};

const ASF_HEADER_GUID: [u8; 16] = [
    0x30, 0x26, 0xB2, 0x75, 0x8E, 0x66, 0xCF, 0x11, 0xA6, 0xD9, 0x00, 0xAA, 0x00, 0x62, 0xCE, 0x6C,
];
const ASF_DATA_GUID: [u8; 16] = [
    0x36, 0x26, 0xB2, 0x75, 0x8E, 0x66, 0xCF, 0x11, 0xA6, 0xD9, 0x00, 0xAA, 0x00, 0x62, 0xCE, 0x6C,
];
// Stream Properties Object GUID (B7DC0791-A9B7-11CF-8EE6-00C00C205365),
// Microsoft mixed-endian. NOTE: the leading bytes `B5 03 BF 5F ...`
// in the trace doc actually identify the *File Properties* object —
// the Stream Properties GUID is the one below.
const ASF_STREAM_PROPS_GUID: [u8; 16] = [
    0x91, 0x07, 0xDC, 0xB7, 0xB7, 0xA9, 0xCF, 0x11, 0x8E, 0xE6, 0x00, 0xC0, 0x0C, 0x20, 0x53, 0x65,
];
// File Properties Object GUID (8CABDCA1-A947-11CF-8EE4-00C00C205365)
// — supplies `min/max_packet_size` we need for the data-packet stride.
const ASF_FILE_PROPS_GUID: [u8; 16] = [
    0xA1, 0xDC, 0xAB, 0x8C, 0x47, 0xA9, 0xCF, 0x11, 0x8E, 0xE4, 0x00, 0xC0, 0x0C, 0x20, 0x53, 0x65,
];

#[derive(Debug, Clone)]
pub struct WaveFormatEx {
    pub format_tag: u16,
    pub channels: u16,
    pub sample_rate: u32,
    pub avg_bytes_per_sec: u32,
    pub block_align: u16,
    pub bits_per_sample: u16,
    pub extradata: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct AsfFile {
    pub waveformatex: WaveFormatEx,
    /// One [`WaveFormatEx::block_align`]-sized WMA superframe per
    /// entry. Concatenated across all ASF data packets in stream
    /// order.
    pub packets: Vec<Vec<u8>>,
}

fn read_u16_le(buf: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([buf[at], buf[at + 1]])
}
fn read_u32_le(buf: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]])
}
fn read_u64_le(buf: &[u8], at: usize) -> u64 {
    u64::from_le_bytes([
        buf[at], buf[at + 1], buf[at + 2], buf[at + 3], buf[at + 4], buf[at + 5], buf[at + 6],
        buf[at + 7],
    ])
}

fn parse_stream_props(obj: &[u8]) -> Result<WaveFormatEx> {
    // Stream Properties body (after GUID+size): per ASF spec §3.5
    //   [stream type GUID 16]
    //   [error correction type GUID 16]
    //   [time offset u64]
    //   [type-specific data length u32]
    //   [error correction data length u32]
    //   [flags u16]
    //   [reserved u32]
    //   [type-specific data] = WAVEFORMATEX (≥ 18 bytes) for audio.
    if obj.len() < 16 + 16 + 8 + 4 + 4 + 2 + 4 + 18 {
        return Err(Error::invalid("asf: stream properties truncated"));
    }
    let type_specific_len = read_u32_le(obj, 16 + 16 + 8) as usize;
    let payload_off = 16 + 16 + 8 + 4 + 4 + 2 + 4;
    if obj.len() < payload_off + type_specific_len {
        return Err(Error::invalid("asf: stream type-specific data truncated"));
    }
    let wfe = &obj[payload_off..payload_off + type_specific_len];
    if wfe.len() < 18 {
        return Err(Error::invalid("asf: WAVEFORMATEX too small"));
    }
    let format_tag = read_u16_le(wfe, 0);
    let channels = read_u16_le(wfe, 2);
    let sample_rate = read_u32_le(wfe, 4);
    let avg_bytes_per_sec = read_u32_le(wfe, 8);
    let block_align = read_u16_le(wfe, 12);
    let bits_per_sample = read_u16_le(wfe, 14);
    let cb_size = read_u16_le(wfe, 16) as usize;
    let extradata = if wfe.len() >= 18 + cb_size {
        wfe[18..18 + cb_size].to_vec()
    } else {
        Vec::new()
    };
    Ok(WaveFormatEx {
        format_tag,
        channels,
        sample_rate,
        avg_bytes_per_sec,
        block_align,
        bits_per_sample,
        extradata,
    })
}

fn parse_file_props_packet_size(obj: &[u8]) -> Option<u32> {
    // File Properties body (post-GUID+size): per ASF spec §3.4
    //   file id GUID 16, file size u64, creation date u64,
    //   data packets count u64, play duration u64, send duration u64,
    //   preroll u64, flags u32, min packet size u32, max packet size u32,
    //   max bitrate u32.
    if obj.len() < 16 + 8 * 6 + 4 + 4 + 4 + 4 {
        return None;
    }
    let min_pkt = read_u32_le(obj, 16 + 8 * 6 + 4);
    Some(min_pkt)
}

fn lb(t: u8) -> usize {
    [0usize, 1, 2, 4][t as usize & 3]
}

/// Parse one ASF data packet into a list of payload byte strings
/// (audio data only — single-payload or compressed multi-payload).
fn parse_one_packet(pkt: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    if pkt.is_empty() {
        return out;
    }
    let mut p = 0usize;
    let ecc = pkt[p];
    p += 1;
    if (ecc & 0x80) != 0 {
        let ecc_data_len = (ecc & 0x0f) as usize;
        if p + ecc_data_len > pkt.len() {
            return out;
        }
        p += ecc_data_len;
    }
    if p + 2 > pkt.len() {
        return out;
    }
    let ltf = pkt[p];
    p += 1;
    let pf = pkt[p];
    p += 1;

    let paddlt = (ltf >> 1) & 3;
    let seqlt = (ltf >> 3) & 3;
    let pktlt = (ltf >> 5) & 3;
    let mp = (ltf >> 7) & 1;
    let _snlt = pf & 3;
    let monlt = (pf >> 2) & 3;
    let oimlt = (pf >> 4) & 3;
    let rdlt = (pf >> 6) & 3;

    let pktlen = if pktlt > 0 {
        let n = lb(pktlt);
        if p + n > pkt.len() { return out; }
        let v = u32::from_le_bytes({
            let mut b = [0u8; 4];
            for (i, x) in pkt[p..p + n].iter().enumerate() { b[i] = *x; }
            b
        }) as usize;
        p += n;
        v.min(pkt.len())
    } else {
        pkt.len()
    };
    if seqlt > 0 {
        let n = lb(seqlt);
        if p + n > pkt.len() { return out; }
        p += n;
    }
    let padding = if paddlt > 0 {
        let n = lb(paddlt);
        if p + n > pkt.len() { return out; }
        let v = u32::from_le_bytes({
            let mut b = [0u8; 4];
            for (i, x) in pkt[p..p + n].iter().enumerate() { b[i] = *x; }
            b
        }) as usize;
        p += n;
        v
    } else {
        0
    };
    if p + 6 > pkt.len() { return out; }
    p += 4; // send_time
    p += 2; // duration

    // Multi-payload?
    let (payload_count, payload_lt) = if mp != 0 {
        if p >= pkt.len() { return out; }
        let b = pkt[p];
        p += 1;
        ((b & 0x3f) as usize, (b >> 6) & 3)
    } else {
        (1usize, 0u8)
    };

    // Available bytes for payloads.
    let avail_end = pktlen.saturating_sub(padding);

    for _ in 0..payload_count {
        if p >= avail_end {
            break;
        }
        // Payload header.
        if p + 1 > pkt.len() { break; }
        let _stream_num = pkt[p];
        p += 1;
        if p + lb(monlt) > pkt.len() { break; }
        p += lb(monlt);
        if p + lb(oimlt) > pkt.len() { break; }
        p += lb(oimlt);
        if p + lb(rdlt) > pkt.len() { break; }
        let n = lb(rdlt);
        let rdl = u32::from_le_bytes({
            let mut b = [0u8; 4];
            for (i, x) in pkt[p..p + n].iter().enumerate() { b[i] = *x; }
            b
        }) as usize;
        p += n;
        if p + rdl > pkt.len() { break; }
        p += rdl;

        let plen = if mp != 0 && payload_lt > 0 {
            if p + lb(payload_lt) > pkt.len() { break; }
            let n = lb(payload_lt);
            let v = u32::from_le_bytes({
                let mut b = [0u8; 4];
                for (i, x) in pkt[p..p + n].iter().enumerate() { b[i] = *x; }
                b
            }) as usize;
            p += n;
            v
        } else {
            avail_end - p
        };
        if p + plen > pkt.len() { break; }
        out.push(pkt[p..p + plen].to_vec());
        p += plen;
    }

    out
}

/// Parse an ASF file from an in-memory buffer. Returns the
/// `WaveFormatEx` from the Stream Properties Object plus a flat list
/// of `block_align`-sized WMA superframes extracted from the Data
/// Object.
pub fn parse(data: &[u8]) -> Result<AsfFile> {
    if data.len() < 30 || data[..16] != ASF_HEADER_GUID {
        return Err(Error::invalid("asf: missing Header Object GUID"));
    }
    let header_size = read_u64_le(data, 16) as usize;
    if header_size < 30 || header_size > data.len() {
        return Err(Error::invalid("asf: Header Object size out of range"));
    }
    let header_end = header_size;
    let mut waveformatex = None;
    let mut min_pkt_size: Option<u32> = None;
    let mut p = 30usize;
    while p + 24 <= header_end {
        let guid = &data[p..p + 16];
        let size = read_u64_le(data, p + 16) as usize;
        if size < 24 || p + size > header_end {
            break;
        }
        let body = &data[p + 24..p + size];
        if guid == ASF_STREAM_PROPS_GUID {
            if let Ok(wfe) = parse_stream_props(body) {
                waveformatex = Some(wfe);
            }
        } else if guid == ASF_FILE_PROPS_GUID {
            min_pkt_size = parse_file_props_packet_size(body);
        }
        p += size;
    }
    let waveformatex = waveformatex
        .ok_or_else(|| Error::invalid("asf: no audio Stream Properties Object found"))?;
    let pkt_stride = min_pkt_size
        .ok_or_else(|| Error::invalid("asf: no File Properties Object found"))?
        as usize;

    // ── Data Object: starts at `header_end`. ──
    if data.len() < header_end + 24 || data[header_end..header_end + 16] != ASF_DATA_GUID {
        return Err(Error::invalid("asf: missing Data Object GUID"));
    }
    let data_obj_size = read_u64_le(data, header_end + 16) as usize;
    if data_obj_size < 50 || header_end + data_obj_size > data.len() {
        return Err(Error::invalid("asf: Data Object size out of range"));
    }
    // Body: 16 file id, 8 total data packets, 2 reserved → packets follow.
    let packets_start = header_end + 24 + 16 + 8 + 2;
    let mut payload_concat: Vec<u8> = Vec::new();
    let mut p = packets_start;
    let data_end = (header_end + data_obj_size).min(data.len());
    while p + pkt_stride <= data_end {
        let pkt = &data[p..p + pkt_stride];
        for payload in parse_one_packet(pkt) {
            payload_concat.extend_from_slice(&payload);
        }
        p += pkt_stride;
    }

    // Split concatenated payload into WMA superframes of block_align
    // bytes each (round 1 — `use_bit_reservoir = 0`, so frames don't
    // span superframes).
    let block_align = waveformatex.block_align as usize;
    if block_align == 0 {
        return Err(Error::invalid("asf: block_align is zero"));
    }
    let mut packets = Vec::new();
    let mut q = 0usize;
    while q + block_align <= payload_concat.len() {
        packets.push(payload_concat[q..q + block_align].to_vec());
        q += block_align;
    }
    Ok(AsfFile {
        waveformatex,
        packets,
    })
}

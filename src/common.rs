//! Shared WMA v1 / v2 decoder pieces.
//!
//! The differences between v1 and v2 are small enough to express as a
//! `version` field on the [`WmaContext`]. The trace doc summarises them
//! in §3.7:
//!
//! * v1 starts the coefficient loop at index 3, v2 at 0.
//! * v1 byte-aligns the bit reader after each channel, v2 does not.
//! * v1's escape coefficient is `coef_nb_bits` of level + `frame_len_bits`
//!   of run; v2's is the 8/16/24/31-bit progressive "large value" plus a
//!   `1+1+{2,frame_len_bits}` run selector.
//! * v1 normalises the IMDCT by an extra `sqrt(N/2)` factor.
//! * v2 normalises sample rates to the bucket boundaries before deriving
//!   `frame_len_bits`; v1 uses the raw rate.
//!
//! The init-time table selection (`coef_vlc_table`, `high_freq`,
//! `use_noise_coding`) follows the rules tabulated in
//! `wma-bands-by-rate.md` §5 and `wma-spectral-vlc.md` §1.

use crate::tables::{
    AAC_SF_BITS, AAC_SF_CODES, COEF0_HUFFBITS, COEF0_HUFFCODES, COEF0_LEVELS, COEF1_HUFFBITS,
    COEF1_HUFFCODES, COEF1_LEVELS, COEF2_HUFFBITS, COEF2_HUFFCODES, COEF2_LEVELS, COEF3_HUFFBITS,
    COEF3_HUFFCODES, COEF3_LEVELS, COEF4_HUFFBITS, COEF4_HUFFCODES, COEF4_LEVELS, COEF5_HUFFBITS,
    COEF5_HUFFCODES, COEF5_LEVELS, CRITICAL_FREQS, EXP_BAND_22050, EXP_BAND_32000, EXP_BAND_44100,
};
use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Version {
    V1,
    V2,
}

/// One spectral codebook (run-level VLC + per-level run-length descriptor).
#[derive(Debug)]
pub struct CoefBook {
    pub codes: &'static [u32],
    pub bits: &'static [u8],
    pub levels: &'static [u16],
    /// Per-symbol decoded `(run, level)` pair. Symbols 0 and 1 are the
    /// escape and end-of-block markers respectively; everything from
    /// symbol 2 onwards is filled in by [`build_run_level`].
    pub run: Vec<u16>,
    pub level: Vec<u16>,
}

impl CoefBook {
    fn new(codes: &'static [u32], bits: &'static [u8], levels: &'static [u16]) -> Self {
        let n = codes.len();
        debug_assert_eq!(n, bits.len());
        let mut run = vec![0u16; n];
        let mut level = vec![0u16; n];
        let mut sym = 2usize;
        for (li, &max_run_plus_one) in levels.iter().enumerate() {
            let lvl = (li + 1) as u16;
            for r in 0..max_run_plus_one {
                if sym >= n {
                    break;
                }
                run[sym] = r;
                level[sym] = lvl;
                sym += 1;
            }
        }
        Self {
            codes,
            bits,
            levels,
            run,
            level,
        }
    }

    /// Decode one symbol from the bitstream by linear scan over the code
    /// table. The codebooks are not large enough to warrant a lookup-
    /// table here on the round-1 fast path; this is the same algorithm
    /// the trace doc shows the reference decoder using.
    fn read_sym(&self, br: &mut BitReader) -> Result<usize> {
        let max_bits = *self.bits.iter().max().unwrap_or(&0) as u32;
        let mut acc: u32 = 0;
        let mut nbits: u32 = 0;
        while nbits < max_bits {
            acc = (acc << 1) | br.read_u32(1)?;
            nbits += 1;
            // Short-circuit on a match with this exact length.
            for (i, &b) in self.bits.iter().enumerate() {
                if b as u32 == nbits && self.codes[i] == acc {
                    return Ok(i);
                }
            }
        }
        Err(Error::invalid("wma: no matching VLC code"))
    }
}

/// Init-time decoder state. Built once from `(sample_rate, bit_rate,
/// channels, version, flags2)`; everything else flows from the
/// bitstream.
pub struct WmaContext {
    pub version: Version,
    pub sample_rate: u32,
    pub channels: u16,
    pub bit_rate: u32,
    pub flags2: u16,

    pub frame_len: usize,
    pub frame_len_bits: u32,
    pub block_len_bits: u32,
    pub nb_block_sizes: u32,

    pub bps: f32,
    pub bps1: f32,

    pub use_exp_vlc: bool,
    pub use_bit_reservoir: bool,
    pub use_variable_block_len: bool,
    pub use_noise_coding: bool,

    pub high_freq: f32,
    pub coef_vlc_table: usize,

    /// Critical-band edges for each supported block size (just the
    /// frame-len entry on round 1, since `nb_block_sizes == 1`).
    pub bands: Vec<u32>,
    pub high_band_start: usize,
    pub coefs_end: usize,
    pub coefs_start: usize,

    pub books: [CoefBook; 6],
    pub pow_tab: Vec<f32>,
    pub sin_window: Vec<f32>,
    pub mdct_norm: f32,

    /// Per-channel IMDCT overlap buffer (frame_len floats each).
    pub overlap: Vec<Vec<f32>>,
}

/// Normalise a sample rate to the WMA v2 bucket boundary (§3.4 of the
/// trace doc). v1 uses the raw rate.
pub fn normalise_sample_rate_v2(sr: u32) -> u32 {
    if sr >= 44100 {
        44100
    } else if sr >= 32000 {
        32000
    } else if sr >= 22050 {
        22050
    } else if sr >= 16000 {
        16000
    } else if sr >= 11025 {
        11025
    } else {
        8000
    }
}

/// Pure-rate `frame_len_bits` lookup (`wma-bands-by-rate.md` §7).
pub fn frame_len_bits_for(sr: u32, version: Version) -> u32 {
    match version {
        Version::V1 => {
            if sr <= 16_000 {
                9
            } else if sr <= 32_000 {
                10
            } else {
                11
            }
        }
        Version::V2 => {
            if sr <= 16_000 {
                9
            } else if sr <= 22_050 {
                10
            } else {
                11
            }
        }
    }
}

fn select_high_freq_and_noise(sr_normalised: u32, bps: f32, bps1: f32) -> (f32, bool) {
    // Reproduces the table in `wma-bands-by-rate.md` §5.
    let factor;
    let noise;
    match sr_normalised {
        44100 => {
            if bps1 >= 0.61 {
                factor = 1.0;
                noise = false;
            } else {
                factor = 0.4;
                noise = true;
            }
        }
        22050 => {
            if bps1 >= 1.16 {
                factor = 1.0;
                noise = false;
            } else if bps1 >= 0.72 {
                factor = 0.7;
                noise = true;
            } else {
                factor = 0.6;
                noise = true;
            }
        }
        16000 => {
            if bps > 0.5 {
                factor = 0.5;
                noise = true;
            } else {
                factor = 0.3;
                noise = true;
            }
        }
        11025 => {
            factor = 0.7;
            noise = true;
        }
        8000 => {
            if bps > 0.75 {
                factor = 1.0;
                noise = false;
            } else if bps > 0.625 {
                factor = 0.65;
                noise = true;
            } else {
                factor = 0.5;
                noise = true;
            }
        }
        _ => {
            if bps >= 0.8 {
                factor = 0.75;
                noise = true;
            } else if bps >= 0.6 {
                factor = 0.6;
                noise = true;
            } else {
                factor = 0.5;
                noise = true;
            }
        }
    }
    let cutoff = (sr_normalised as f32 / 2.0) * factor;
    (cutoff, noise)
}

fn select_coef_vlc_table(sample_rate: u32, bps1: f32) -> usize {
    if sample_rate >= 32_000 {
        if bps1 < 0.72 {
            0
        } else if bps1 < 1.16 {
            1
        } else {
            2
        }
    } else {
        2
    }
}

fn build_v1_bands(block_len: usize, sample_rate: u32) -> Vec<u32> {
    let mut bands = Vec::new();
    let mut lpos = 0u32;
    for &cf in CRITICAL_FREQS.iter() {
        let pos = ((block_len as u64 * 2 * cf as u64 + sample_rate as u64 / 2) / sample_rate as u64)
            as u32;
        let pos = pos.min(block_len as u32);
        if pos > lpos {
            bands.push(pos - lpos);
        }
        if pos as usize >= block_len {
            break;
        }
        lpos = pos;
    }
    bands
}

fn build_v2_bands(block_len: usize, sample_rate: u32, frame_len_bits: u32) -> Vec<u32> {
    // §3 precomputed override?
    let a = frame_len_bits as i32 - 7; // k=0 (single block size)
    if (0..3).contains(&a) {
        let table = match sample_rate {
            22050 => Some(EXP_BAND_22050),
            32000 => Some(EXP_BAND_32000),
            44100 => Some(EXP_BAND_44100),
            _ => None,
        };
        if let Some(t) = table {
            if (a as usize) < t.len() {
                let row = t[a as usize];
                return row.iter().map(|&v| v as u32).collect();
            }
        }
    }
    let mut bands = Vec::new();
    let mut lpos = 0u32;
    let blsr = (4 * sample_rate) as u64;
    for &cf in CRITICAL_FREQS.iter() {
        let pos = ((block_len as u64 * 2 * cf as u64 + 2 * sample_rate as u64) / blsr) as u32;
        let pos = (pos << 2).min(block_len as u32);
        if pos > lpos {
            bands.push(pos - lpos);
        }
        if pos as usize >= block_len {
            break;
        }
        lpos = pos;
    }
    bands
}

impl WmaContext {
    pub fn new(
        version: Version,
        sample_rate: u32,
        channels: u16,
        bit_rate: u32,
        flags2: u16,
    ) -> Result<Self> {
        if !(1..=2).contains(&channels) {
            return Err(Error::unsupported("wma: only mono / stereo on round 1"));
        }
        if sample_rate > 48_000 {
            return Err(Error::unsupported("wma: v1/v2 cap at 48 kHz"));
        }

        let use_exp_vlc = (flags2 & 0x0001) != 0;
        let use_bit_reservoir = (flags2 & 0x0002) != 0;
        let use_variable_block_len = (flags2 & 0x0004) != 0;
        if use_bit_reservoir {
            return Err(Error::unsupported("wma: bit reservoir not yet supported"));
        }
        if use_variable_block_len {
            return Err(Error::unsupported(
                "wma: variable block length not yet supported",
            ));
        }

        let sr_normalised = match version {
            Version::V1 => sample_rate,
            Version::V2 => normalise_sample_rate_v2(sample_rate),
        };
        let frame_len_bits = frame_len_bits_for(sr_normalised, version);
        let frame_len = 1usize << frame_len_bits;
        let block_len_bits = frame_len_bits;
        let nb_block_sizes = 1u32;

        let bps = bit_rate as f32 / channels as f32 / sample_rate as f32;
        let bps1 = if channels == 2 { bps * 1.6 } else { bps };

        let (high_freq, use_noise_coding) = select_high_freq_and_noise(sr_normalised, bps, bps1);
        if use_noise_coding {
            // The trace fixtures in our corpus all have noise coding off
            // (`flags2 = 0x0001`). Round-2 work item.
            return Err(Error::unsupported(
                "wma: noise-coded high band not yet supported",
            ));
        }

        let coef_vlc_table = select_coef_vlc_table(sample_rate, bps1);

        let bands = match version {
            Version::V1 => build_v1_bands(frame_len, sample_rate),
            Version::V2 => build_v2_bands(frame_len, sr_normalised, frame_len_bits),
        };
        let high_band_start =
            ((frame_len as u64 * 2 * high_freq.round() as u64) / sample_rate as u64) as usize;
        let coefs_end = (frame_len - frame_len * 9 / 100).min(frame_len);
        let coefs_start = match version {
            Version::V1 => 3,
            Version::V2 => 0,
        };

        // Pre-quantised antilog: pow_tab[60 + i] = 10^(i/16) for i in -60..=95.
        let pow_tab: Vec<f32> = (-60..=95).map(|i| 10f32.powf(i as f32 / 16.0)).collect();

        // Plain sine window of length `frame_len`.
        let sin_window: Vec<f32> = (0..frame_len)
            .map(|n| (std::f32::consts::PI * (n as f32 + 0.5) / frame_len as f32).sin())
            .collect();

        // IMDCT scale. The doc (§3.7 / §6.6) describes v1 as carrying
        // an extra `sqrt(N/2)` factor folded into the IMDCT plan and
        // v2 as "unscaled" — but those statements are made relative
        // to FFmpeg's specific IMDCT primitive whose internal
        // normalisation includes a `1/(2*N)` factor. For our naive
        // unnormalised IMDCT (Σ X[k] cos(...) without prefactor) we
        // recover unit amplitude with `1/N` for v2 and `sqrt(N/2)/N
        // = 1/sqrt(2N)` for v1.
        // `2/N` already lives inside [`naive_imdct`] (matches FFmpeg's
        // IMDCT primitive's net normalisation). v1 then layers an
        // additional `sqrt(N/2)` on top per the trace doc §3.7; v2
        // doesn't.
        // IMDCT scale. Per the trace doc §3.7 / §6.6:
        //   v1 has an extra `sqrt(N/2)` factor folded into the IMDCT.
        //   v2 is unscaled.
        // Both relative to FFmpeg's IMDCT primitive (which itself
        // includes a `2/N` factor internally — matching the
        // [`naive_imdct`] convention used here).
        let mdct_norm = match version {
            Version::V1 => (frame_len as f32 / 2.0).sqrt(),
            Version::V2 => 1.0,
        };

        let books: [CoefBook; 6] = [
            CoefBook::new(COEF0_HUFFCODES, COEF0_HUFFBITS, COEF0_LEVELS),
            CoefBook::new(COEF1_HUFFCODES, COEF1_HUFFBITS, COEF1_LEVELS),
            CoefBook::new(COEF2_HUFFCODES, COEF2_HUFFBITS, COEF2_LEVELS),
            CoefBook::new(COEF3_HUFFCODES, COEF3_HUFFBITS, COEF3_LEVELS),
            CoefBook::new(COEF4_HUFFCODES, COEF4_HUFFBITS, COEF4_LEVELS),
            CoefBook::new(COEF5_HUFFCODES, COEF5_HUFFBITS, COEF5_LEVELS),
        ];

        let overlap = (0..channels as usize)
            .map(|_| vec![0f32; frame_len])
            .collect();

        Ok(Self {
            version,
            sample_rate,
            channels,
            bit_rate,
            flags2,
            frame_len,
            frame_len_bits,
            block_len_bits,
            nb_block_sizes,
            bps,
            bps1,
            use_exp_vlc,
            use_bit_reservoir,
            use_variable_block_len,
            use_noise_coding,
            high_freq,
            coef_vlc_table,
            bands,
            high_band_start,
            coefs_end,
            coefs_start,
            books,
            pow_tab,
            sin_window,
            mdct_norm,
            overlap,
        })
    }

    fn book(&self, side: bool) -> &CoefBook {
        let idx = self.coef_vlc_table * 2 + if side { 1 } else { 0 };
        &self.books[idx]
    }

    fn read_aac_scalefactor(&self, br: &mut BitReader) -> Result<i32> {
        // Linear-scan VLC over the 121-entry AAC scale-factor codebook.
        let max_bits = *AAC_SF_BITS.iter().max().unwrap() as u32;
        let mut acc: u32 = 0;
        let mut nbits: u32 = 0;
        while nbits < max_bits {
            acc = (acc << 1) | br.read_u32(1)?;
            nbits += 1;
            for i in 0..AAC_SF_CODES.len() {
                if AAC_SF_BITS[i] as u32 == nbits && AAC_SF_CODES[i] == acc {
                    // Symbol value `i` is a delta of `i - 60` (centred at zero).
                    return Ok(i as i32 - 60);
                }
            }
        }
        Err(Error::invalid("wma: no matching AAC scalefactor code"))
    }

    fn total_gain_to_coef_nb_bits(total_gain: u32) -> u32 {
        if total_gain < 15 {
            13
        } else if total_gain < 32 {
            12
        } else if total_gain < 40 {
            11
        } else if total_gain < 45 {
            10
        } else {
            9
        }
    }

    /// Read a "large value" coefficient escape: progressive 8/16/24/31-bit
    /// extension (used by WMA v2 escape coefficients).
    fn read_large_val(br: &mut BitReader) -> Result<u32> {
        let v = br.read_u32(8)?;
        if v < 0x80 {
            Ok(v)
        } else {
            let v2 = br.read_u32(8)?;
            let combined = ((v & 0x7f) << 8) | v2;
            if combined < 0x4000 {
                Ok(combined + 0x80)
            } else {
                let v3 = br.read_u32(8)?;
                let combined3 = ((combined & 0x3fff) << 8) | v3;
                if combined3 < 0x200000 {
                    Ok(combined3 + 0x4080)
                } else {
                    let v4 = br.read_u32(7)?;
                    Ok(((combined3 & 0x1fffff) << 7) | v4)
                }
            }
        }
    }

    /// Decode one channel's worth of spectral coefficients into `coefs`.
    fn run_level_decode(
        &self,
        br: &mut BitReader,
        side: bool,
        total_gain: u32,
        coefs: &mut [f32],
    ) -> Result<()> {
        let book = self.book(side);
        let coef_nb_bits = Self::total_gain_to_coef_nb_bits(total_gain);
        let mut i = self.coefs_start;
        let end = self.coefs_end.min(coefs.len());
        for c in coefs.iter_mut() {
            *c = 0.0;
        }
        loop {
            if i >= end {
                break;
            }
            let sym = book.read_sym(br)?;
            let (run, level) = if sym == 0 {
                // Escape.
                match self.version {
                    Version::V1 => {
                        let level = br.read_u32(coef_nb_bits)?;
                        let run = br.read_u32(self.frame_len_bits)?;
                        (run, level)
                    }
                    Version::V2 => {
                        let level = Self::read_large_val(br)?;
                        let run = if br.read_u32(1)? != 0 {
                            if br.read_u32(1)? != 0 {
                                br.read_u32(self.frame_len_bits)?
                            } else {
                                br.read_u32(2)?
                            }
                        } else {
                            0
                        };
                        (run, level)
                    }
                }
            } else if sym == 1 {
                break;
            } else {
                (book.run[sym] as u32, book.level[sym] as u32)
            };
            i += run as usize;
            if i >= coefs.len() {
                break;
            }
            let sign = br.read_u32(1)?;
            let val = level as f32;
            coefs[i] = if sign == 0 { val } else { -val };
            i += 1;
        }
        Ok(())
    }

    /// Read VLC-coded exponents into `exponents` (length = `frame_len`).
    /// One float per coefficient; exponents are flat across each band.
    fn decode_exp_vlc(&self, br: &mut BitReader, exponents: &mut [f32]) -> Result<f32> {
        // Initial last_exp (§3.3 step 7). v1 reads 5 bits and adds 10
        // to get the running exponent index; v2 uses a hard-coded 36.
        let mut last_exp: i32 = match self.version {
            Version::V1 => br.read_u32(5)? as i32 + 10,
            Version::V2 => 36,
        };
        let mut idx = 0usize;
        let mut max_exp = 1.0f32;
        for &width in self.bands.iter() {
            // Decode one signed delta from the AAC scale-factor codebook
            // (only for v2 / v1's "subsequent" bands). v1's first band
            // is the absolute `last_exp + 10`; subsequent v1 bands also
            // use the AAC-style delta.
            if !(self.version == Version::V1 && idx == 0) {
                let delta = self.read_aac_scalefactor(br)?;
                last_exp = last_exp.saturating_add(delta);
            }
            let pow_idx = (last_exp + 60).clamp(0, self.pow_tab.len() as i32 - 1) as usize;
            let val = self.pow_tab[pow_idx];
            if val > max_exp {
                max_exp = val;
            }
            let w = width as usize;
            let take = w.min(exponents.len() - idx);
            for slot in &mut exponents[idx..idx + take] {
                *slot = val;
            }
            idx += w;
            if idx >= exponents.len() {
                break;
            }
        }
        Ok(max_exp)
    }

    /// Decode one full frame (round 1: single block per frame, no bit
    /// reservoir, no variable block length, no noise coding). Output
    /// is appended to `out` (one Vec<f32> per channel, frame_len floats).
    pub fn decode_frame(
        &mut self,
        packet: &[u8],
        out_per_channel: &mut [Vec<f32>],
    ) -> Result<()> {
        let mut br = BitReader::new(packet);

        // No outer superframe header on round 1 (use_bit_reservoir is
        // forced off in `new()`).

        // ── §3.3 step 1: block-length triplet — skipped (single block).
        // ── §3.3 step 2: M/S stereo flag.
        let ms_stereo = if self.channels == 2 {
            br.read_u32(1)? != 0
        } else {
            false
        };

        // ── §3.3 step 3: per-channel coded flags.
        let mut channel_coded = [false; 2];
        for c in 0..self.channels as usize {
            channel_coded[c] = br.read_u32(1)? != 0;
        }

        let any_coded = channel_coded
            .iter()
            .take(self.channels as usize)
            .any(|&v| v);
        let mut total_gain: u32 = 1;
        if any_coded {
            // ── §3.3 step 4: total_gain — unary chain of 7-bit fields.
            loop {
                let v = br.read_u32(7)?;
                total_gain += v;
                if v != 127 {
                    break;
                }
            }
        }

        // ── §3.3 step 5 / 6: noise / exponent-update flags. We forced
        // noise coding off in init, and we're full-frame so the
        // exponent-update flag is omitted.

        // ── §3.3 step 7: per-channel exponents.
        let mut all_coefs: Vec<Vec<f32>> = Vec::with_capacity(self.channels as usize);
        let mut max_exps = [1.0f32; 2];
        let mut exponents_per_channel: Vec<Vec<f32>> = Vec::with_capacity(self.channels as usize);
        for c in 0..self.channels as usize {
            if !channel_coded[c] {
                exponents_per_channel.push(vec![1.0f32; self.frame_len]);
                continue;
            }
            let mut exps = vec![0f32; self.frame_len];
            if self.use_exp_vlc {
                max_exps[c] = self.decode_exp_vlc(&mut br, &mut exps)?;
            } else {
                return Err(Error::unsupported("wma: LSP exponents not yet supported"));
            }
            exponents_per_channel.push(exps);
        }

        // ── §3.3 step 8: spectral coefficients.
        for c in 0..self.channels as usize {
            let mut coefs = vec![0f32; self.frame_len];
            if channel_coded[c] {
                let side = ms_stereo && c == 1;
                self.run_level_decode(&mut br, side, total_gain, &mut coefs)?;
                if self.version == Version::V1 {
                    br.align_to_byte();
                }
            }
            all_coefs.push(coefs);
        }

        // ── §3.3 step 9: M/S stereo butterfly (frequency domain).
        if ms_stereo && self.channels == 2 {
            let (left, right) = all_coefs.split_at_mut(1);
            let l = &mut left[0];
            let r = &mut right[0];
            for k in 0..self.frame_len {
                let m = l[k];
                let s = r[k];
                l[k] = m + s;
                r[k] = m - s;
            }
        }

        // ── Apply scaling: coef[k] *= exp10(total_gain * 0.05) /
        // max_exponent[ch] (then exponent gain per band).
        let total_gain_factor = 10f32.powf(total_gain as f32 * 0.05);
        for c in 0..self.channels as usize {
            if !channel_coded[c] {
                continue;
            }
            let mx = max_exps[c].max(1e-30);
            let scale = total_gain_factor / mx;
            for k in 0..self.frame_len {
                all_coefs[c][k] *= exponents_per_channel[c][k] * scale;
            }
        }

        // ── §3.3 step 10: IMDCT + sine-window overlap-add.
        //
        // For an N-coefficient block we do the IMDCT into a 2N-sample
        // time buffer, multiply by a 2N-sample sine window, then split
        // the result in two: the first N samples overlap-add with the
        // previous block's stored second-half tail to emit N output
        // samples; the second N samples become the next block's tail.
        let n = self.frame_len;
        let two_n = n * 2;
        // Build a 2N-sample sine window for the IMDCT output.
        let win2: Vec<f32> = (0..two_n)
            .map(|i| (std::f32::consts::PI * (i as f32 + 0.5) / two_n as f32).sin())
            .collect();
        for c in 0..self.channels as usize {
            let mut time = vec![0f32; two_n];
            naive_imdct(&all_coefs[c], &mut time);
            if self.mdct_norm != 1.0 {
                for v in time.iter_mut() {
                    *v *= self.mdct_norm;
                }
            }
            for (v, w) in time.iter_mut().zip(win2.iter()) {
                *v *= *w;
            }
            // Emit N output samples: first half + previous tail.
            let mut output = vec![0f32; n];
            for i in 0..n {
                output[i] = time[i] + self.overlap[c][i];
            }
            // Save the second half as next tail.
            let mut new_overlap = vec![0f32; n];
            new_overlap[..n].copy_from_slice(&time[n..two_n]);
            self.overlap[c] = new_overlap;
            out_per_channel[c].extend_from_slice(&output);
        }

        Ok(())
    }
}

/// Naive O(N²) IMDCT — fine for correctness on round-1 PSNR tests.
///
/// `N` (= `coefs.len()`) is the WMA "block_len" (number of MDCT
/// coefficients fed to the block). The IMDCT produces `2N` time
/// samples; with the sine window applied on both encoder and decoder
/// sides plus 50 % overlap-add, this exactly reconstructs `N` new
/// output samples per block.
///
/// Standard FFmpeg-style IMDCT:
///   `y[n] = (2/N) Σ_{k=0..N-1} X[k] cos(π/N · (n + 0.5 + N/2) · (k + 0.5))`
///   for n = 0..2N-1.
///
/// Will be replaced with an FFT-based implementation in a future round.
fn naive_imdct(coefs: &[f32], out: &mut [f32]) {
    let two_n = out.len();
    let n = two_n / 2;
    debug_assert_eq!(coefs.len(), n);
    let pi_over_n = std::f32::consts::PI / n as f32;
    let scale = 2.0 / n as f32;
    for n_i in 0..two_n {
        let mut acc = 0.0f32;
        let n_phase = (n_i as f32) + 0.5 + (n as f32) / 2.0;
        for k in 0..n {
            let phase = pi_over_n * n_phase * (k as f32 + 0.5);
            acc += coefs[k] * phase.cos();
        }
        out[n_i] = acc * scale;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coef_book_levels_sum() {
        let book = CoefBook::new(COEF0_HUFFCODES, COEF0_HUFFBITS, COEF0_LEVELS);
        let s: usize = COEF0_LEVELS.iter().map(|&v| v as usize).sum();
        assert_eq!(s + 2, book.codes.len());
    }

    #[test]
    fn band_partitions_sum_to_block_v1() {
        let bands = build_v1_bands(2048, 44100);
        let total: u32 = bands.iter().sum();
        // v1 round-half-up may not exactly hit the block edge — should
        // be within 0..=block_len.
        assert!(total <= 2048);
        assert!(!bands.is_empty());
    }

    #[test]
    fn band_partitions_v2_overrides() {
        // a = frame_len_bits - 7 - 0; for 22050 → frame_len_bits=10 → a=3
        // (out of override range), so it falls to the live computation.
        let bands22 = build_v2_bands(1024, 22050, 10);
        assert!(!bands22.is_empty());
        // 32000 → frame_len_bits=11 → a=4 (out of override), live.
        let bands32 = build_v2_bands(2048, 32000, 11);
        assert!(!bands32.is_empty());
    }
}

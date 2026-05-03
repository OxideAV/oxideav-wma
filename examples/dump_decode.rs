//! Debug utility: read a WMA file, walk every packet, decode, and
//! dump per-frame stats (rms, max abs, sample count) to stdout.

use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};
use oxideav_wma::{asf, register};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: dump_decode <file.wma>");
        std::process::exit(2);
    }
    let bytes = std::fs::read(&args[1]).expect("read input");
    let asf_file = asf::parse(&bytes).expect("asf parse");
    let wfe = &asf_file.waveformatex;
    println!(
        "tag=0x{:04x} ch={} rate={} avg_bps={} block_align={} bps={} extra={}",
        wfe.format_tag,
        wfe.channels,
        wfe.sample_rate,
        wfe.avg_bytes_per_sec,
        wfe.block_align,
        wfe.bits_per_sample,
        hex::encode(&wfe.extradata),
    );
    println!("packets: {}", asf_file.packets.len());

    let codec_id = match wfe.format_tag {
        0x0160 => "wmav1",
        0x0161 => "wmav2",
        _ => panic!("unsupported codec tag"),
    };
    let mut reg = oxideav_core::CodecRegistry::new();
    register(&mut reg);
    let mut params = CodecParameters::audio(CodecId::new(codec_id));
    params.sample_rate = Some(wfe.sample_rate);
    params.channels = Some(wfe.channels);
    params.bit_rate = Some((wfe.avg_bytes_per_sec as u64) * 8);
    params.extradata = wfe.extradata.clone();
    let mut dec = reg.make_decoder(&params).expect("make decoder");

    for (i, pkt_bytes) in asf_file.packets.iter().enumerate() {
        let pkt = Packet::new(0, TimeBase::new(1, wfe.sample_rate as i64), pkt_bytes.clone());
        if let Err(e) = dec.send_packet(&pkt) {
            println!("pkt {i}: send err {e:?}");
            continue;
        }
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    let plane = &af.data[0];
                    let mut max = 0f32;
                    let mut sumsq = 0f64;
                    let mut n = 0;
                    for chunk in plane.chunks_exact(4) {
                        let s = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                        max = max.max(s.abs());
                        sumsq += (s as f64) * (s as f64);
                        n += 1;
                    }
                    let rms = (sumsq / n.max(1) as f64).sqrt();
                    println!("frame {i} samples={n} max={max:.4} rms={rms:.4}");
                }
                Ok(_) => {}
                Err(oxideav_core::Error::NeedMore) => break,
                Err(e) => {
                    println!("pkt {i}: recv err {e:?}");
                    break;
                }
            }
        }
    }
}


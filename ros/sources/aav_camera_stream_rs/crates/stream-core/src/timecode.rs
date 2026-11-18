//! A clock time drawn into a picture, to measure a stream's delay end to end
//! (`timecode.py`).
//!
//! A row of black and white squares along the top left corner: a white and a black square
//! (the reader's levels), 32 bits of the time in ms (mod 2^32), and a CRC-8 of it, most
//! significant first. Squares 1/80 of the picture wide (16 px at 1280) survive H.264.

use crate::{Error, Result};

/// The code's ms wrap round here (49 days).
pub const WRAP: u64 = 1 << 32;
const DATA_BITS: u32 = 32;
const CRC_BITS: u32 = 8;
/// Squares in the row.
pub const SQUARES: usize = 2 + (DATA_BITS + CRC_BITS) as usize;

/// A picture in memory: rows of `width` pixels of `channels` bytes, packed.
#[derive(Debug, Clone, Copy)]
pub struct Frame<'a> {
    pub data: &'a [u8],
    pub width: usize,
    pub height: usize,
    pub channels: usize,
}

/// CRC-8 (polynomial 0x07) of the 4 bytes of `value`, most significant first.
pub fn crc8(value: u32) -> u8 {
    value.to_be_bytes().iter().fold(0u8, |crc, &byte| {
        (0..8).fold(crc ^ byte, |c, _| {
            if c & 0x80 != 0 {
                (c << 1) ^ 0x07
            } else {
                c << 1
            }
        })
    })
}

/// Pixels per square for a picture this wide: 1/80 of it, at least 4.
pub fn square_size(width: usize) -> usize {
    (width / 80).max(4)
}

/// The squares' values (true = white) for a time in ms.
pub fn bits_of(ms: u64) -> Vec<bool> {
    let value = (ms % WRAP) as u32;
    let word = (u64::from(value) << CRC_BITS) | u64::from(crc8(value));
    [true, false]
        .into_iter()
        .chain(
            (0..DATA_BITS + CRC_BITS)
                .rev()
                .map(|i| (word >> i) & 1 == 1),
        )
        .collect()
}

/// Whether the picture is big enough for the row.
fn fits(width: usize, height: usize) -> bool {
    let size = square_size(width);
    height >= size && width >= size * SQUARES
}

/// Draw the time `ms` into a packed picture.
pub fn stamp(data: &mut [u8], width: usize, height: usize, channels: usize, ms: u64) -> Result<()> {
    if !fits(width, height) {
        return Err(Error::Invalid(format!(
            "a {width}x{height} picture is too small for the time code"
        )));
    }
    let size = square_size(width);
    for (i, bit) in bits_of(ms).into_iter().enumerate() {
        let level = if bit { 255 } else { 0 };
        for row in data.chunks_exact_mut(width * channels).take(size) {
            row[i * size * channels..(i + 1) * size * channels].fill(level);
        }
    }
    Ok(())
}

/// The mean of a square's middle, all channels (its edges blur in the encoder).
fn level(frame: &Frame<'_>, square: usize, size: usize) -> f64 {
    let (lo, hi) = (size / 4, size - size / 4);
    let c = frame.channels;
    let x0 = square * size;
    let row_len = frame.width * c;
    let sum: u64 = frame
        .data
        .chunks_exact(row_len)
        .skip(lo)
        .take(hi - lo)
        .flat_map(|row| &row[(x0 + lo) * c..(x0 + hi) * c])
        .map(|&b| u64::from(b))
        .sum();
    sum as f64 / ((hi - lo) * (hi - lo) * c) as f64
}

/// The time in ms drawn into the picture, or None if there is none (or it is damaged).
pub fn read(frame: &Frame<'_>) -> Option<u32> {
    if !fits(frame.width, frame.height)
        || frame.data.len() < frame.width * frame.height * frame.channels
    {
        return None;
    }
    let size = square_size(frame.width);
    let levels: Vec<f64> = (0..SQUARES).map(|i| level(frame, i, size)).collect();
    let (white, black) = (levels[0], levels[1]);
    if white - black < 64.0 {
        return None; // no reference squares: no code here
    }
    let middle = (white + black) / 2.0;
    let word = levels[2..]
        .iter()
        .fold(0u64, |w, &l| (w << 1) | u64::from(l > middle));
    let value = (word >> CRC_BITS) as u32;
    let crc = (word & 0xFF) as u8;
    (crc8(value) == crc).then_some(value)
}

/// A wall-clock time in s as the code's ms, mod 2^32.
pub fn now_ms(wall_s: f64) -> u64 {
    (crate::round_even(wall_s * 1000.0) as i64).rem_euclid(WRAP as i64) as u64
}

/// Seconds from the code's time to `wall_s`, across the 2^32 ms wrap.
pub fn age_s(sent_ms: u32, wall_s: f64) -> f64 {
    let wrap = WRAP as i64;
    let mut diff = (now_ms(wall_s) as i64 - i64::from(sent_ms)).rem_euclid(wrap);
    if diff >= wrap / 2 {
        diff -= wrap; // sent after wall_s: a clock step, or a misread
    }
    diff as f64 / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stamped_picture_reads_back() {
        let (w, h) = (1280, 720);
        let mut data = vec![128u8; w * h * 3];
        assert!(stamp(&mut data, w, h, 3, 1_234_567).is_ok());
        let frame = Frame {
            data: &data,
            width: w,
            height: h,
            channels: 3,
        };
        assert_eq!(read(&frame), Some(1_234_567));
    }

    #[test]
    fn a_plain_picture_has_no_code() {
        let data = vec![90u8; 640 * 360];
        let frame = Frame {
            data: &data,
            width: 640,
            height: 360,
            channels: 1,
        };
        assert_eq!(read(&frame), None);
    }
}

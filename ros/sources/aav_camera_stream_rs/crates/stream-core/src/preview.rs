//! The small preview: the picture shrunk to `width` (OpenCV's `INTER_AREA`: each output
//! pixel the mean of the input pixels it covers), as a JPEG at quality 70.

use jpeg_encoder::{ColorType, Encoder};

use crate::{round_even, Error, Result};

/// The preview's height for a picture of `width` x `height` shrunk to `to_width`, as the
/// Python's `int(round(height * scale))`.
pub fn preview_height(width: u32, height: u32, to_width: u32) -> u32 {
    let scale = f64::from(to_width) / f64::from(width);
    round_even(f64::from(height) * scale) as u32
}

/// How much of each input pixel along one axis falls in each output pixel.
fn shares(from: usize, to: usize) -> Vec<Vec<(usize, f64)>> {
    let scale = from as f64 / to as f64;
    (0..to)
        .map(|o| {
            let (start, end) = (o as f64 * scale, (o + 1) as f64 * scale);
            let first = start.floor() as usize;
            let last = (end.ceil() as usize).min(from);
            (first..last)
                .map(|i| {
                    let lo = start.max(i as f64);
                    let hi = end.min((i + 1) as f64);
                    (i, (hi - lo) / scale)
                })
                .filter(|&(_, s)| s > 0.0)
                .collect()
        })
        .collect()
}

/// A packed BGR picture shrunk by area averaging.
pub fn shrink(
    bgr: &[u8],
    width: u32,
    height: u32,
    to_width: u32,
    to_height: u32,
) -> Result<Vec<u8>> {
    let (w, h, tw, th) = (
        width as usize,
        height as usize,
        to_width as usize,
        to_height as usize,
    );
    if bgr.len() != w * h * 3 || tw == 0 || th == 0 {
        return Err(Error::Invalid(format!(
            "can't shrink {} bytes of {w}x{h} BGR to {tw}x{th}",
            bgr.len()
        )));
    }
    let (xs, ys) = (shares(w, tw), shares(h, th));
    let mut out = Vec::with_capacity(tw * th * 3);
    for row in &ys {
        for col in &xs {
            let mut sum = [0.0f64; 3];
            for &(y, sy) in row {
                for &(x, sx) in col {
                    let at = (y * w + x) * 3;
                    for (s, &b) in sum.iter_mut().zip(&bgr[at..at + 3]) {
                        *s += f64::from(b) * sx * sy;
                    }
                }
            }
            out.extend(sum.map(|s| round_even(s).clamp(0.0, 255.0) as u8));
        }
    }
    Ok(out)
}

/// The preview's JPEG: the picture `to_width` wide, quality 70.
pub fn jpeg(bgr: &[u8], width: u32, height: u32, to_width: u32) -> Result<Vec<u8>> {
    let to_height = preview_height(width, height, to_width).max(1);
    let small = shrink(bgr, width, height, to_width, to_height)?;
    let mut out = Vec::new();
    let size = |n: u32| {
        u16::try_from(n).map_err(|_| Error::Invalid(format!("a preview {n} px is too big")))
    };
    Encoder::new(&mut out, 70)
        .encode(&small, size(to_width)?, size(to_height)?, ColorType::Bgr)
        .map_err(|e| Error::Invalid(format!("JPEG: {e}")))?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_quarter_is_the_mean_of_each_4_by_4() {
        let (w, h) = (8, 4);
        let bgr: Vec<u8> = (0..w * h)
            .flat_map(|i| {
                let v = if (i % w) < 4 { 40 } else { 200 };
                [v, v, v]
            })
            .collect();
        let small = shrink(&bgr, w as u32, h as u32, 2, 1).expect("sizes");
        assert_eq!(small, vec![40, 40, 40, 200, 200, 200]);
    }

    #[test]
    fn the_preview_is_a_jpeg_of_the_pythons_size() {
        assert_eq!(preview_height(1280, 720, 320), 180);
        let bgr = vec![99u8; 64 * 36 * 3];
        let out = jpeg(&bgr, 64, 36, 32).expect("a JPEG");
        assert_eq!(&out[..2], &[0xFF, 0xD8]);
    }
}

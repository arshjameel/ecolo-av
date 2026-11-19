//! The picture as the model takes it: the rows below the horizon, scaled exactly as Pillow's
//! `resize(..., BILINEAR)` scaled them in training (`tools/lane_data.py`'s `crop`).

use lane_core::Picture;

use crate::{Meta, Result};

/// Pillow's fixed point for 8-bit pictures: weights in 2^22ths.
const PRECISION_BITS: u32 = 32 - 8 - 2;

/// One output pixel's source pixels along one axis: the first, and each one's weight.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Taps {
    pub first: usize,
    pub weights: Vec<i32>,
}

/// The taps for scaling `size_in` pixels to `size_out`: Pillow's `precompute_coeffs` with
/// its bilinear (triangle) filter, widened when shrinking, and `normalize_coeffs_8bpc`.
pub fn resample(size_in: usize, size_out: usize) -> Vec<Taps> {
    let scale = size_in as f64 / size_out as f64;
    let filter_scale = scale.max(1.0);
    let support = filter_scale;
    (0..size_out)
        .map(|xx| {
            let centre = (xx as f64 + 0.5) * scale;
            // C's (int) cast: toward zero.
            let first = ((centre - support + 0.5) as i64).max(0) as usize;
            let last = ((centre + support + 0.5) as i64).min(size_in as i64) as usize;
            let raw: Vec<f64> = (first..last)
                .map(|x| {
                    let t = ((x as f64 - centre + 0.5) / filter_scale).abs();
                    if t < 1.0 {
                        1.0 - t
                    } else {
                        0.0
                    }
                })
                .collect();
            let total: f64 = raw.iter().sum();
            let one = f64::from(1u32 << PRECISION_BITS);
            let weights = raw
                .iter()
                .map(|w| {
                    let k = if total != 0.0 { w / total } else { 0.0 };
                    (k * one + if k < 0.0 { -0.5 } else { 0.5 }) as i32
                })
                .collect();
            Taps { first, weights }
        })
        .collect()
}

fn clip8(sum: i64) -> u8 {
    (sum >> PRECISION_BITS).clamp(0, 255) as u8
}

fn apply(taps: &Taps, sample: impl Fn(usize) -> u8) -> u8 {
    let start = 1i64 << (PRECISION_BITS - 1);
    clip8(taps.weights.iter().enumerate().fold(start, |s, (i, &w)| {
        s + i64::from(sample(taps.first + i)) * i64::from(w)
    }))
}

/// The model's input: 3 x `input_h` x `input_w` numbers, red, green, blue, 0 to 1.
///
/// The crop is `meta.crop_top` scaled to this picture's height, so a picture of another size
/// (the same lens, scaled) still shows the model the same part of the road.
pub fn input(picture: &Picture<'_>, meta: &Meta) -> Result<Vec<f32>> {
    let (w, h) = (picture.width(), picture.height());
    let top = (meta.crop_top as f64 * h as f64 / meta.height as f64).round() as usize;
    let rows_in = h.saturating_sub(top).max(1);
    let across = resample(w, meta.input_w);
    let down = resample(rows_in, meta.input_h);
    // Across first, then down, rounding to 8 bits between: as Pillow does.
    let mut wide = vec![[0u8; 3]; rows_in * meta.input_w];
    for y in 0..rows_in {
        let row: Vec<[u8; 3]> = (0..w).map(|x| picture.rgb(x, top + y)).collect();
        for (x, taps) in across.iter().enumerate() {
            wide[y * meta.input_w + x] = [0, 1, 2].map(|c| apply(taps, |i| row[i][c]));
        }
    }
    let plane = meta.input_w * meta.input_h;
    let mut out = vec![0.0f32; 3 * plane];
    for (y, taps) in down.iter().enumerate() {
        for x in 0..meta.input_w {
            for c in 0..3 {
                let v = apply(taps, |i| wide[i * meta.input_w + x][c]);
                out[c * plane + y * meta.input_w + x] = f32::from(v) / 255.0;
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use lane_core::Encoding;

    #[test]
    fn taps_add_up_to_one_and_stay_inside() {
        for (a, b) in [(640, 480), (192, 144), (100, 100), (10, 30)] {
            for t in resample(a, b) {
                let sum: i64 = t.weights.iter().map(|&w| i64::from(w)).sum();
                assert!((sum - (1 << PRECISION_BITS)).abs() <= t.weights.len() as i64);
                assert!(t.first + t.weights.len() <= a);
            }
        }
    }

    #[test]
    fn the_same_size_copies() {
        for t in resample(50, 50) {
            assert_eq!(t.weights.iter().filter(|&&w| w != 0).count(), 1);
        }
    }

    #[test]
    fn a_flat_picture_stays_flat_and_channels_stay_apart() {
        let meta = crate::meta::example();
        let data: Vec<u8> = (0..640 * 360).flat_map(|_| [200u8, 100, 50]).collect();
        let p = Picture::packed(&data, 640, 360, Encoding::Rgb8).unwrap();
        let x = input(&p, &meta).unwrap();
        let plane = 480 * 144;
        assert_eq!(x.len(), 3 * plane);
        assert!(x[..plane].iter().all(|&v| v == 200.0 / 255.0));
        assert!(x[plane..2 * plane].iter().all(|&v| v == 100.0 / 255.0));
        assert!(x[2 * plane..].iter().all(|&v| v == 50.0 / 255.0));
        // bgr8: the same picture, read in its own order.
        let bgr: Vec<u8> = (0..640 * 360).flat_map(|_| [50u8, 100, 200]).collect();
        let p = Picture::packed(&bgr, 640, 360, Encoding::Bgr8).unwrap();
        assert_eq!(input(&p, &meta).unwrap(), x);
    }
}

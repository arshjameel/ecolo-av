//! The picture as the model's input: resized, scaled to 0-1, normalised, channels first.

use crate::Picture;

/// ImageNet's mean, red, green, blue, on 0-1 values: what RF-DETR was trained with.
pub const IMAGENET_MEAN: [f32; 3] = [0.485, 0.456, 0.406];
/// ImageNet's spread (standard deviation), red, green, blue.
pub const IMAGENET_STD: [f32; 3] = [0.229, 0.224, 0.225];

/// The model's input size, in pixels: 384 x 384 for RF-DETR Nano.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputSize {
    /// Rows.
    pub height: usize,
    /// Columns.
    pub width: usize,
}

impl InputSize {
    /// The number of floats in one input, (1, 3, height, width).
    pub fn len(&self) -> usize {
        3 * self.height * self.width
    }

    /// True for a size of no pixels.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// One output pixel's two source pixels along an axis, and the second one's weight.
#[derive(Debug, Clone, Copy)]
struct Tap {
    first: usize,
    second: usize,
    weight: f32,
}

/// The taps of a bilinear resize from `from` pixels to `to`, as OpenCV's `INTER_LINEAR`:
/// pixel centres line up (half-pixel offsets), no anti-aliasing, edges clamped.
fn taps(from: usize, to: usize) -> Vec<Tap> {
    let scale = from as f64 / to as f64;
    (0..to)
        .map(|i| {
            let x = (i as f64 + 0.5) * scale - 0.5;
            let floor = x.floor();
            let (first, weight) = if floor < 0.0 {
                (0, 0.0)
            } else if floor as usize >= from - 1 {
                (from - 1, 0.0)
            } else {
                (floor as usize, (x - floor) as f32)
            };
            Tap {
                first,
                second: (first + 1).min(from - 1),
                weight,
            }
        })
        .collect()
}

/// Return the picture as the model's input, (1, 3, height, width), row-major.
///
/// As `detr.py`'s `preprocess`: RGB scaled to 0-1, resized bilinearly (OpenCV's
/// `INTER_LINEAR`, which is torchvision's resize without anti-aliasing), then normalised
/// with ImageNet's mean and spread.
pub fn preprocess(picture: &Picture<'_>, size: InputSize) -> Vec<f32> {
    let (width, height) = (picture.width(), picture.height());
    let columns = taps(width, size.width);
    // Each source row resized along x first, then rows blended along y, as OpenCV does.
    let wide: Vec<Vec<[f32; 3]>> = (0..height)
        .map(|y| {
            let row: Vec<[f32; 3]> = picture
                .rgb_row(y)
                .map(|p| p.map(|v| f32::from(v) / 255.0))
                .collect();
            columns
                .iter()
                .map(|t| {
                    let (a, b) = (row[t.first], row[t.second]);
                    std::array::from_fn(|c| a[c] * (1.0 - t.weight) + b[c] * t.weight)
                })
                .collect()
        })
        .collect();
    let plane = size.height * size.width;
    let mut out = vec![0.0; size.len()];
    for (y, t) in taps(height, size.height).iter().enumerate() {
        let (top, bottom) = (&wide[t.first], &wide[t.second]);
        for (x, (a, b)) in top.iter().zip(bottom).enumerate() {
            for c in 0..3 {
                let v = a[c] * (1.0 - t.weight) + b[c] * t.weight;
                out[c * plane + y * size.width + x] = (v - IMAGENET_MEAN[c]) / IMAGENET_STD[c];
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PixelOrder;

    #[test]
    fn taps_match_opencv() {
        // 4 to 2: each output pixel sits between two inputs, half and half.
        let t = taps(4, 2);
        assert_eq!((t[0].first, t[0].second, t[0].weight), (0, 1, 0.5));
        assert_eq!((t[1].first, t[1].second, t[1].weight), (2, 3, 0.5));
        // 2 to 4: the outer pixels clamp to the edges.
        let t = taps(2, 4);
        assert_eq!((t[0].first, t[0].weight), (0, 0.0));
        assert_eq!((t[1].first, t[1].second, t[1].weight), (0, 1, 0.25));
        assert_eq!((t[3].first, t[3].weight), (1, 0.0));
    }

    #[test]
    fn one_colour_stays_one_colour_normalised() {
        let grey = vec![128u8; 5 * 3 * 3];
        let picture = Picture::packed(&grey, 5, 3, PixelOrder::Rgb).unwrap();
        let size = InputSize {
            height: 4,
            width: 4,
        };
        let input = preprocess(&picture, size);
        assert_eq!(input.len(), 48);
        for (c, plane) in input.chunks(16).enumerate() {
            let want = (128.0 / 255.0 - IMAGENET_MEAN[c]) / IMAGENET_STD[c];
            assert!(plane.iter().all(|v| (v - want).abs() < 1e-6));
        }
    }

    #[test]
    fn bgr_comes_out_as_rgb_planes() {
        // A blue picture, BGR: the third plane (blue) is the bright one.
        let blue = [255u8, 0, 0].repeat(4);
        let picture = Picture::packed(&blue, 2, 2, PixelOrder::Bgr).unwrap();
        let input = preprocess(
            &picture,
            InputSize {
                height: 2,
                width: 2,
            },
        );
        assert!(input[8] > 2.0 && input[0] < -2.0);
    }
}

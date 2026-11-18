//! Boxes drawn on a picture, for `detections/image`: a red outline, 2 px, no text.

use crate::{Corners, PixelOrder};

/// A picture's pixels to draw on: 8-bit, three channels, rows of `stride` bytes.
#[derive(Debug)]
pub struct Canvas<'a> {
    /// The pixels.
    pub data: &'a mut [u8],
    /// Width in pixels.
    pub width: usize,
    /// Height in pixels.
    pub height: usize,
    /// Bytes per row.
    pub stride: usize,
    /// The order of each pixel's bytes.
    pub order: PixelOrder,
}

impl Canvas<'_> {
    /// Draw a red outline `thickness` pixels wide just inside `corners`; parts outside the
    /// picture are cut off.
    pub fn outline(&mut self, corners: &Corners, thickness: usize) {
        let red = match self.order {
            PixelOrder::Rgb => [255, 0, 0],
            PixelOrder::Bgr => [0, 0, 255],
        };
        let clamp = |v: f32, max: usize| (v.round().max(0.0) as usize).min(max.saturating_sub(1));
        let (x1, x2) = (clamp(corners.x1, self.width), clamp(corners.x2, self.width));
        let (y1, y2) = (
            clamp(corners.y1, self.height),
            clamp(corners.y2, self.height),
        );
        let t = thickness.max(1);
        for y in y1..=y2 {
            let edge_row = y < y1 + t || y + t > y2;
            for x in x1..=x2 {
                if edge_row || x < x1 + t || x + t > x2 {
                    let at = y * self.stride + 3 * x;
                    if let Some(pixel) = self.data.get_mut(at..at + 3) {
                        pixel.copy_from_slice(&red);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_outline_not_a_fill() {
        let mut data = vec![0u8; 6 * 6 * 3];
        let mut canvas = Canvas {
            data: &mut data,
            width: 6,
            height: 6,
            stride: 18,
            order: PixelOrder::Bgr,
        };
        canvas.outline(
            &Corners {
                x1: 1.0,
                y1: 1.0,
                x2: 4.0,
                y2: 4.0,
            },
            1,
        );
        let red = |x: usize, y: usize| data[y * 18 + 3 * x + 2] == 255;
        assert!(red(1, 1) && red(4, 4) && red(1, 4) && red(2, 1));
        assert!(!red(2, 2) && !red(0, 0) && !red(5, 5));
    }

    #[test]
    fn off_the_picture_is_cut_off() {
        let mut data = vec![0u8; 6 * 6 * 3];
        let mut canvas = Canvas {
            data: &mut data,
            width: 6,
            height: 6,
            stride: 18,
            order: PixelOrder::Rgb,
        };
        let huge = Corners {
            x1: -10.0,
            y1: -10.0,
            x2: 100.0,
            y2: 100.0,
        };
        canvas.outline(&huge, 2);
        let red = |x: usize, y: usize| data[y * 18 + 3 * x] == 255;
        assert!(red(0, 0) && red(5, 5) && red(1, 3));
        assert!(!red(2, 2) && !red(3, 3));
    }
}

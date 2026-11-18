//! A camera picture, borrowed: 8-bit colour, three channels, rows of any stride.

use crate::{Error, Result};

/// The order of a pixel's three bytes: ROS's `rgb8` or `bgr8`, OpenCV's BGR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelOrder {
    /// Red, green, blue.
    Rgb,
    /// Blue, green, red.
    Bgr,
}

/// A picture the detector reads, borrowed from a camera message or a file.
#[derive(Debug, Clone, Copy)]
pub struct Picture<'a> {
    data: &'a [u8],
    width: usize,
    height: usize,
    stride: usize,
    order: PixelOrder,
}

impl<'a> Picture<'a> {
    /// Wrap `data`: `height` rows of `stride` bytes (at least 3 x `width`), as ROS's
    /// `sensor_msgs/Image` has them (`step` is the stride).
    pub fn new(
        data: &'a [u8],
        width: usize,
        height: usize,
        stride: usize,
        order: PixelOrder,
    ) -> Result<Self> {
        let needed = if height == 0 {
            0
        } else {
            stride * (height - 1) + 3 * width
        };
        if width == 0 || height == 0 || stride < 3 * width || data.len() < needed {
            return Err(Error::Picture {
                needed: needed.max(3 * width * height),
                given: data.len(),
            });
        }
        Ok(Self {
            data,
            width,
            height,
            stride,
            order,
        })
    }

    /// Wrap tightly packed pixels: `width` x `height` x 3 bytes.
    pub fn packed(data: &'a [u8], width: usize, height: usize, order: PixelOrder) -> Result<Self> {
        Self::new(data, width, height, 3 * width, order)
    }

    /// Width in pixels.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Height in pixels.
    pub fn height(&self) -> usize {
        self.height
    }

    /// Row `y`'s pixels, as red, green, blue.
    pub(crate) fn rgb_row(&self, y: usize) -> impl Iterator<Item = [u8; 3]> + '_ {
        let start = y * self.stride;
        let order = self.order;
        self.data[start..start + 3 * self.width]
            .as_chunks::<3>()
            .0
            .iter()
            .map(move |&[first, middle, last]| match order {
                PixelOrder::Rgb => [first, middle, last],
                PixelOrder::Bgr => [last, middle, first],
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_come_out_rgb_whatever_the_order() {
        let bgr = [1, 2, 3, 4, 5, 6, 0, 0];
        let picture = Picture::new(&bgr, 1, 2, 4, PixelOrder::Bgr).unwrap();
        assert_eq!(picture.rgb_row(0).collect::<Vec<_>>(), vec![[3, 2, 1]]);
        // The second row starts at the stride, 4 bytes in.
        assert_eq!(picture.rgb_row(1).collect::<Vec<_>>(), vec![[0, 6, 5]]);
    }

    #[test]
    fn too_few_bytes_is_an_error() {
        let err = Picture::packed(&[0; 11], 2, 2, PixelOrder::Rgb).unwrap_err();
        assert!(matches!(
            err,
            Error::Picture {
                needed: 12,
                given: 11
            }
        ));
        assert!(Picture::packed(&[], 0, 0, PixelOrder::Rgb).is_err());
    }
}

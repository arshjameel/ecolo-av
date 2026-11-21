//! A camera picture, borrowed from a message or a file; and the bird's-eye view.

use crate::{Error, Result};

/// How a picture's pixels are laid out, as `sensor_msgs/Image`'s `encoding` names them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Encoding {
    Rgb8,
    Bgr8,
    Rgba8,
    Bgra8,
    Mono8,
}

impl Encoding {
    /// The encoding of a ROS name (`rgb8`...), or an error naming it.
    pub fn from_ros(name: &str) -> Result<Self> {
        match name {
            "rgb8" => Ok(Self::Rgb8),
            "bgr8" => Ok(Self::Bgr8),
            "rgba8" => Ok(Self::Rgba8),
            "bgra8" => Ok(Self::Bgra8),
            "mono8" | "8UC1" => Ok(Self::Mono8),
            other => Err(Error::Encoding(other.to_string())),
        }
    }

    /// Bytes per pixel.
    pub fn bytes(self) -> usize {
        match self {
            Self::Rgb8 | Self::Bgr8 => 3,
            Self::Rgba8 | Self::Bgra8 => 4,
            Self::Mono8 => 1,
        }
    }

    /// Where blue, green and red are in a pixel's bytes.
    pub(crate) fn bgr_at(self) -> [usize; 3] {
        match self {
            Self::Rgb8 | Self::Rgba8 => [2, 1, 0],
            Self::Bgr8 | Self::Bgra8 => [0, 1, 2],
            Self::Mono8 => [0, 0, 0],
        }
    }
}

/// A picture the finder reads: `height` rows of `stride` bytes.
#[derive(Debug, Clone, Copy)]
pub struct Picture<'a> {
    data: &'a [u8],
    width: usize,
    height: usize,
    stride: usize,
    encoding: Encoding,
}

impl<'a> Picture<'a> {
    /// Wrap `data` as `sensor_msgs/Image` has it (`step` is the stride).
    pub fn new(
        data: &'a [u8],
        width: usize,
        height: usize,
        stride: usize,
        encoding: Encoding,
    ) -> Result<Self> {
        let row = encoding.bytes() * width;
        let needed = match height {
            0 => 0,
            h => stride * (h - 1) + row,
        };
        if stride < row || data.len() < needed {
            return Err(Error::TooFewBytes {
                needed: needed.max(row * height),
                given: data.len(),
            });
        }
        Ok(Self {
            data,
            width,
            height,
            stride,
            encoding,
        })
    }

    /// Wrap tightly packed pixels.
    pub fn packed(data: &'a [u8], width: usize, height: usize, encoding: Encoding) -> Result<Self> {
        Self::new(data, width, height, encoding.bytes() * width, encoding)
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Red, green and blue at (x, y) (grey three times for `mono8`): for the learned lane
    /// finder (`aav_lane_finder_learned`), which takes colour pictures.
    pub fn rgb(&self, x: usize, y: usize) -> [u8; 3] {
        let p = self.pixel(x, y);
        let [b, g, r] = self.encoding.bgr_at();
        [p[r], p[g], p[b]]
    }

    /// The pixel's bytes at (x, y), then where blue, green and red are in them.
    pub(crate) fn pixel(&self, x: usize, y: usize) -> &'a [u8] {
        let n = self.encoding.bytes();
        let start = y * self.stride + x * n;
        &self.data[start..start + n]
    }

    pub(crate) fn encoding(&self) -> Encoding {
        self.encoding
    }
}

/// The bird's-eye view: `rows` x `cols` pixels, blue, green, red; row 0 the far edge,
/// column 0 the left edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bev {
    pub rows: usize,
    pub cols: usize,
    pub pixels: Vec<[u8; 3]>,
}

impl Bev {
    /// Each pixel's brightest channel: white and yellow paint both stand out.
    pub fn grey(&self) -> impl Iterator<Item = u8> + '_ {
        self.pixels.iter().map(|p| p[0].max(p[1]).max(p[2]))
    }

    /// The pixels as packed bgr8 bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        self.pixels.iter().flatten().copied().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_short_picture_is_refused() {
        assert!(Picture::packed(&[0; 11], 2, 2, Encoding::Rgb8).is_err());
        assert!(Picture::packed(&[0; 12], 2, 2, Encoding::Rgb8).is_ok());
        // The last row needs only its own pixels, not the whole stride.
        assert!(Picture::new(&[0; 14], 2, 2, 8, Encoding::Rgb8).is_ok());
    }

    #[test]
    fn rgb_is_read_as_bgr() {
        let p = Picture::packed(&[1, 2, 3], 1, 1, Encoding::Rgb8).unwrap();
        let px = p.pixel(0, 0);
        let [b, g, r] = p.encoding().bgr_at().map(|i| px[i]);
        assert_eq!([b, g, r], [3, 2, 1]);
        assert!(Encoding::from_ros("yuv422").is_err());
    }
}

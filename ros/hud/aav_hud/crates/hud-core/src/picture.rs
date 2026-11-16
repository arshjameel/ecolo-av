//! A camera picture (sensor_msgs/msg/Image) turned into RGBA pixels for the screen.
//!
//! Only uncompressed 8-bit pictures: what usb_cam, mock_camera and image_publisher send
//! on `/sensors/camera/front_left/image_raw`.

use std::str::FromStr;

use thiserror::Error;

/// The pixel layouts the HUD can show (sensor_msgs/image_encodings names).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Encoding {
    Rgb8,
    Bgr8,
    Rgba8,
    Bgra8,
    Mono8,
}

impl Encoding {
    /// Bytes in one pixel.
    #[must_use]
    pub fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Rgb8 | Self::Bgr8 => 3,
            Self::Rgba8 | Self::Bgra8 => 4,
            Self::Mono8 => 1,
        }
    }

    /// One pixel's bytes as red, green, blue, alpha.
    fn to_rgba(self, p: &[u8]) -> [u8; 4] {
        match (self, p) {
            (Self::Rgb8, &[r, g, b, ..]) => [r, g, b, 255],
            (Self::Bgr8, &[b, g, r, ..]) => [r, g, b, 255],
            (Self::Rgba8, &[r, g, b, a, ..]) => [r, g, b, a],
            (Self::Bgra8, &[b, g, r, a, ..]) => [r, g, b, a],
            (Self::Mono8, &[v, ..]) => [v, v, v, 255],
            // chunks_exact always hands over a whole pixel, so this never happens.
            _ => [0, 0, 0, 255],
        }
    }
}

impl FromStr for Encoding {
    type Err = PictureError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "rgb8" => Ok(Self::Rgb8),
            "bgr8" => Ok(Self::Bgr8),
            "rgba8" => Ok(Self::Rgba8),
            "bgra8" => Ok(Self::Bgra8),
            "mono8" | "8UC1" => Ok(Self::Mono8),
            other => Err(PictureError::Encoding(other.to_owned())),
        }
    }
}

/// Why a picture can't be shown.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum PictureError {
    #[error("the HUD can't show {0:?} pictures (only rgb8, bgr8, rgba8, bgra8, mono8)")]
    Encoding(String),
    #[error("an empty picture ({width}x{height})")]
    Empty { width: u32, height: u32 },
    #[error("a row of {width} pixels needs {needed} bytes, but step is {step}")]
    Step {
        width: u32,
        step: u32,
        needed: usize,
    },
    #[error("{height} rows of {step} bytes need {needed} bytes, but the data has {got}")]
    Short {
        height: u32,
        step: u32,
        needed: usize,
        got: usize,
    },
}

/// A picture ready for the screen: width x height pixels, 4 bytes each (RGBA).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Picture {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

impl Picture {
    /// Converts one Image message's fields. `step` is the bytes in one row, which may be
    /// more than width x bytes per pixel (padding at each row's end).
    pub fn from_image(
        encoding: &str,
        width: u32,
        height: u32,
        step: u32,
        data: &[u8],
    ) -> Result<Self, PictureError> {
        let encoding: Encoding = encoding.parse()?;
        if width == 0 || height == 0 {
            return Err(PictureError::Empty { width, height });
        }
        let row_bytes = width as usize * encoding.bytes_per_pixel();
        if (step as usize) < row_bytes {
            return Err(PictureError::Step {
                width,
                step,
                needed: row_bytes,
            });
        }
        // The last row needs no padding after it.
        let needed = step as usize * (height as usize - 1) + row_bytes;
        if data.len() < needed {
            return Err(PictureError::Short {
                height,
                step,
                needed,
                got: data.len(),
            });
        }
        let rgba = data
            .chunks(step as usize)
            .take(height as usize)
            .flat_map(|row| row[..row_bytes].chunks_exact(encoding.bytes_per_pixel()))
            .flat_map(|pixel| encoding.to_rgba(pixel))
            .collect();
        Ok(Self {
            width,
            height,
            rgba,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bgr8_is_swapped_to_rgb() {
        let p = Picture::from_image("bgr8", 2, 1, 6, &[1, 2, 3, 4, 5, 6]).expect("valid");
        assert_eq!(p.rgba, [3, 2, 1, 255, 6, 5, 4, 255]);
    }

    #[test]
    fn rgb8_and_mono8() {
        let p = Picture::from_image("rgb8", 1, 1, 3, &[10, 20, 30]).expect("valid");
        assert_eq!(p.rgba, [10, 20, 30, 255]);
        let p = Picture::from_image("mono8", 2, 1, 2, &[7, 9]).expect("valid");
        assert_eq!(p.rgba, [7, 7, 7, 255, 9, 9, 9, 255]);
    }

    #[test]
    fn row_padding_is_skipped() {
        // Two rows of one rgb8 pixel, each padded to 4 bytes; no padding after the last.
        let p = Picture::from_image("rgb8", 1, 2, 4, &[1, 2, 3, 0, 4, 5, 6]).expect("valid");
        assert_eq!(p.rgba, [1, 2, 3, 255, 4, 5, 6, 255]);
    }

    #[test]
    fn bad_pictures_are_refused() {
        assert!(matches!(
            Picture::from_image("yuv422", 1, 1, 2, &[0, 0]),
            Err(PictureError::Encoding(_))
        ));
        assert!(matches!(
            Picture::from_image("rgb8", 2, 1, 3, &[0; 6]),
            Err(PictureError::Step { .. })
        ));
        assert!(matches!(
            Picture::from_image("rgb8", 2, 2, 6, &[0; 11]),
            Err(PictureError::Short { .. })
        ));
        assert!(matches!(
            Picture::from_image("rgb8", 0, 2, 6, &[]),
            Err(PictureError::Empty { .. })
        ));
    }
}

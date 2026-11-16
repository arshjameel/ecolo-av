//! A finished picture, as sensor_msgs/Image will carry it: rgb8, rows top to bottom.

use std::{fmt, time::Duration};

/// One camera's picture at one moment.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// The camera's index in the rig.
    pub camera: usize,
    /// The simulator's time of the car pose it was drawn from: the header's stamp.
    pub stamp: Duration,
    pub width: u32,
    pub height: u32,
    /// rgb8, `width * 3` bytes a row, no padding.
    pub rgb: Vec<u8>,
}

impl Frame {
    /// sensor_msgs/Image's encoding.
    pub const ENCODING: &'static str = "rgb8";

    /// Bytes a row: the Image's `step`.
    #[must_use]
    pub fn step(&self) -> u32 {
        self.width * 3
    }

    /// The colour of pixel (column, row).
    #[must_use]
    pub fn pixel(&self, u: u32, v: u32) -> Option<[u8; 3]> {
        if u >= self.width || v >= self.height {
            return None;
        }
        let i = ((v * self.width + u) * 3) as usize;
        self.rgb.get(i..i + 3).map(|p| [p[0], p[1], p[2]])
    }
}

/// The GPU's copy is shorter than its picture.
#[derive(Clone, Debug, PartialEq)]
pub struct ShortCopy {
    pub got: usize,
    pub wanted: usize,
}

impl fmt::Display for ShortCopy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "the GPU's copy has {} bytes; {} wanted",
            self.got, self.wanted
        )
    }
}

impl std::error::Error for ShortCopy {}

/// RGBA rows as a GPU copies them (each row padded to `row_bytes`, at least `width * 4`)
/// into packed rgb8.
pub fn rgba_rows_to_rgb(
    data: &[u8],
    width: u32,
    height: u32,
    row_bytes: usize,
) -> Result<Vec<u8>, ShortCopy> {
    let (w, h) = (width as usize, height as usize);
    let row_bytes = row_bytes.max(w * 4);
    let wanted = row_bytes * h.saturating_sub(1) + w * 4;
    if data.len() < wanted {
        return Err(ShortCopy {
            got: data.len(),
            wanted,
        });
    }
    Ok(data
        .chunks(row_bytes)
        .take(h)
        .flat_map(|row| {
            row[..w * 4]
                .as_chunks::<4>()
                .0
                .iter()
                .flat_map(|&[r, g, b, _]| [r, g, b])
        })
        .collect())
}

/// wgpu's copy alignment: a row of a texture copy is a multiple of 256 bytes.
#[must_use]
pub fn padded_row_bytes(width: u32) -> usize {
    (width as usize * 4).div_ceil(256) * 256
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strips_padding_and_alpha() {
        // 3 x 2 pixels, rows padded to 16 bytes.
        let mut data = vec![0u8; 32];
        for (row, base) in [(0usize, 10u8), (1, 100)] {
            for px in 0..3 {
                let i = row * 16 + px * 4;
                data[i..i + 4].copy_from_slice(&[base + px as u8, 1, 2, 255]);
            }
        }
        let rgb = rgba_rows_to_rgb(&data, 3, 2, 16).unwrap();
        assert_eq!(
            rgb,
            [10, 1, 2, 11, 1, 2, 12, 1, 2, 100, 1, 2, 101, 1, 2, 102, 1, 2]
        );
        let f = Frame {
            camera: 0,
            stamp: Duration::ZERO,
            width: 3,
            height: 2,
            rgb,
        };
        assert_eq!(f.pixel(2, 1), Some([102, 1, 2]));
        assert_eq!(f.pixel(3, 0), None);
    }

    #[test]
    fn short_copy_is_an_error() {
        assert!(rgba_rows_to_rgb(&[0; 10], 3, 2, 16).is_err());
    }

    #[test]
    fn row_alignment() {
        assert_eq!(padded_row_bytes(640), 2560);
        assert_eq!(padded_row_bytes(100), 512);
    }
}

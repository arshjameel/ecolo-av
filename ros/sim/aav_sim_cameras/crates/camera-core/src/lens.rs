//! A pinhole lens: what `camera_info` says, and where a point lands in the picture.
//!
//! Gazebo's camera_info (gz-sensors `CameraSensor::PopulateInfo`, gz-rendering
//! `projectionToCameraIntrinsic`) for a camera with only `horizontal_fov`:
//! fx = fy = width / 2 / tan(hfov / 2), cx = width / 2, cy = height / 2, no distortion
//! (plumb_bob, all zero), R the identity, P = [K | 0]. Ours is the same.
//!
//! Pixel coordinates: u right, v down, from the picture's top-left corner. Pixel (i, j)
//! covers u in [i, i + 1) and v in [j, j + 1), as a GPU draws it, so the point (u, v) is in
//! pixel (floor u, floor v). OpenCV puts pixel centres on whole numbers instead: half a
//! pixel apart, the same as Gazebo's pictures.

use std::fmt;

use crate::geom::{Rotation, Vec3};

/// Where a point lands, in pixels (u right, v down).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pixel {
    pub u: f64,
    pub v: f64,
}

impl Pixel {
    /// The pixel's column and row, if it's inside a `width` x `height` picture.
    #[must_use]
    pub fn index(self, width: u32, height: u32) -> Option<(u32, u32)> {
        let (i, j) = (self.u.floor(), self.v.floor());
        (i >= 0.0 && j >= 0.0 && i < f64::from(width) && j < f64::from(height))
            .then_some((i as u32, j as u32))
    }
}

/// Why a lens can't be made.
#[derive(Clone, Debug, PartialEq)]
pub enum LensError {
    /// Width or height is zero.
    EmptyPicture,
    /// The focal length isn't a positive number.
    BadFocal(f64),
    /// The field of view isn't between 0 and pi.
    BadFov(f64),
}

impl fmt::Display for LensError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyPicture => write!(f, "the picture has no pixels"),
            Self::BadFocal(v) => write!(f, "focal length {v} px: must be above 0"),
            Self::BadFov(v) => write!(f, "field of view {v} rad: must be between 0 and pi"),
        }
    }
}

impl std::error::Error for LensError {}

/// A pinhole lens with square pixels and its centre in the middle of the picture: what a
/// GPU's perspective camera draws, and what Gazebo's cameras give.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lens {
    width: u32,
    height: u32,
    focal: f64,
}

impl Lens {
    /// From the focal length in pixels (`aav.sensors.gazebo.xacro`'s `camera_focal`).
    pub fn from_focal(width: u32, height: u32, focal_px: f64) -> Result<Self, LensError> {
        if width == 0 || height == 0 {
            return Err(LensError::EmptyPicture);
        }
        if !(focal_px.is_finite() && focal_px > 0.0) {
            return Err(LensError::BadFocal(focal_px));
        }
        Ok(Self {
            width,
            height,
            focal: focal_px,
        })
    }

    /// From the horizontal field of view (radians), as SDF's `horizontal_fov`.
    pub fn from_horizontal_fov(width: u32, height: u32, fov: f64) -> Result<Self, LensError> {
        if !(fov > 0.0 && fov < std::f64::consts::PI) {
            return Err(LensError::BadFov(fov));
        }
        Self::from_focal(width, height, f64::from(width) / 2.0 / (fov / 2.0).tan())
    }

    #[must_use]
    pub fn width(&self) -> u32 {
        self.width
    }

    #[must_use]
    pub fn height(&self) -> u32 {
        self.height
    }

    #[must_use]
    pub fn fx(&self) -> f64 {
        self.focal
    }

    #[must_use]
    pub fn fy(&self) -> f64 {
        self.focal
    }

    #[must_use]
    pub fn cx(&self) -> f64 {
        f64::from(self.width) / 2.0
    }

    #[must_use]
    pub fn cy(&self) -> f64 {
        f64::from(self.height) / 2.0
    }

    /// Top to bottom, radians: what a GPU's perspective camera takes.
    #[must_use]
    pub fn vertical_fov(&self) -> f64 {
        2.0 * (f64::from(self.height) / 2.0 / self.focal).atan()
    }

    #[must_use]
    pub fn horizontal_fov(&self) -> f64 {
        2.0 * (f64::from(self.width) / 2.0 / self.focal).atan()
    }

    /// camera_info's K, row by row.
    #[must_use]
    pub fn k(&self) -> [f64; 9] {
        [
            self.fx(),
            0.0,
            self.cx(),
            0.0,
            self.fy(),
            self.cy(),
            0.0,
            0.0,
            1.0,
        ]
    }

    /// camera_info's P: K with a zero fourth column (one camera, no stereo shift).
    #[must_use]
    pub fn p(&self) -> [f64; 12] {
        [
            self.fx(),
            0.0,
            self.cx(),
            0.0,
            0.0,
            self.fy(),
            self.cy(),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
    }

    /// camera_info's R: the identity (one camera).
    #[must_use]
    pub fn r(&self) -> [f64; 9] {
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    }

    /// camera_info's D (k1, k2, p1, p2, k3): none.
    #[must_use]
    pub fn d(&self) -> [f64; 5] {
        [0.0; 5]
    }

    /// camera_info's distortion_model, as Gazebo's bridge sends it.
    #[must_use]
    pub fn distortion_model(&self) -> &'static str {
        "plumb_bob"
    }

    /// A point in the optical frame (x right, y down, z ahead) to a pixel. `None` behind the
    /// lens. The pixel may be outside the picture: check with [`Pixel::index`].
    #[must_use]
    pub fn project(&self, optical: Vec3) -> Option<Pixel> {
        (optical.z > 0.0).then(|| Pixel {
            u: self.fx() * optical.x / optical.z + self.cx(),
            v: self.fy() * optical.y / optical.z + self.cy(),
        })
    }
}

/// The optical frame in a camera's link frame: the link looks along x with z up (ROS, and
/// Gazebo's camera), the optical frame along z with y down (REP-103's `_optical` suffix).
#[must_use]
pub fn optical_in_link() -> Rotation {
    Rotation::from_axes(
        Vec3::new(0.0, -1.0, 0.0), // optical x: right = link -y
        Vec3::new(0.0, 0.0, -1.0), // optical y: down = link -z
        Vec3::new(1.0, 0.0, 0.0),  // optical z: ahead = link x
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Gazebo car's front camera: 640 x 360, focal 343 px.
    fn gazebo_front() -> Lens {
        Lens::from_horizontal_fov(640, 360, 2.0 * (320.0_f64 / 343.0).atan()).unwrap()
    }

    #[test]
    fn matches_gazebos_camera_info() {
        let lens = gazebo_front();
        let k = lens.k();
        assert!((k[0] - 343.0).abs() < 1e-9 && (k[4] - 343.0).abs() < 1e-9);
        assert_eq!((k[2], k[5]), (320.0, 180.0));
        assert_eq!(lens.p()[3], 0.0);
        assert_eq!(lens.d(), [0.0; 5]);
    }

    #[test]
    fn centre_and_offsets() {
        let lens = gazebo_front();
        let p = lens.project(Vec3::new(0.0, 0.0, 10.0)).unwrap();
        assert_eq!((p.u, p.v), (320.0, 180.0));
        // 1 m right and 0.5 m down, 10 m away: 34.3 and 17.15 px from the centre.
        let p = lens.project(Vec3::new(1.0, 0.5, 10.0)).unwrap();
        assert!((p.u - 354.3).abs() < 1e-9 && (p.v - 197.15).abs() < 1e-9);
        assert_eq!(p.index(640, 360), Some((354, 197)));
        assert!(lens.project(Vec3::new(0.0, 0.0, -1.0)).is_none());
    }

    #[test]
    fn rejects_nonsense() {
        assert_eq!(Lens::from_focal(0, 10, 1.0), Err(LensError::EmptyPicture));
        assert!(Lens::from_focal(10, 10, -1.0).is_err());
        assert!(Lens::from_horizontal_fov(10, 10, 4.0).is_err());
    }

    #[test]
    fn optical_axes() {
        let r = optical_in_link();
        assert_eq!(r.apply(Vec3::new(0.0, 0.0, 1.0)), Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(r.apply(Vec3::new(1.0, 0.0, 0.0)), Vec3::new(0.0, -1.0, 0.0));
    }
}

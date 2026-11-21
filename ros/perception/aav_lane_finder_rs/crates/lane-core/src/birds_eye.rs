//! The bird's-eye view: the road ahead, seen from above, made from one camera picture.
//!
//! The road is taken to be flat, so each pixel below the horizon sees one point on the
//! ground. Where that point is follows from the lens (focal length, centre, distortion),
//! where the camera sits on the car, and how it is tilted.
//!
//! The view is a grid on the ground in base_link (x forward, y left, metres, REP-103):
//!
//! ```text
//!     row 0          the far edge (x_max)       column 0         the left edge (+y_max)
//!     the last row   the near edge (x_min)      the last column  the right edge (-y_max)
//! ```
//!
//! Cells the camera can't see stay black, and [`BirdsEye::valid`] says which ones they are.

use crate::picture::{Bev, Picture};
use crate::{Error, Result};

/// Rays that meet the ground closer than this in front of the lens are left out (m).
pub const NEAREST: f64 = 0.1;

/// OpenCV's fixed-point remap: 1/32 of a pixel, weights summing to 2^15.
const TAB_BITS: u32 = 5;
const TAB_SIZE: i32 = 1 << TAB_BITS;
const COEF_BITS: u32 = 15;

/// OpenCV's distortion coefficients, in its order: k1, k2, p1, p2, k3, k4, k5, k6, s1,
/// s2, s3, s4 (the missing ones 0).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Distortion([f64; 12]);

impl Distortion {
    /// The distortion a `CameraInfo` gives: None when every coefficient is 0.
    ///
    /// `plumb_bob` and `rational_polynomial` are the models OpenCV's `projectPoints`
    /// knows; 4, 5, 8 or 12 coefficients, as it takes them.
    pub fn from_camera_info(model: &str, d: &[f64]) -> Result<Option<Self>> {
        if d.iter().all(|&v| v == 0.0) {
            return Ok(None);
        }
        if !matches!(model, "plumb_bob" | "rational_polynomial") {
            return Err(Error::DistortionModel(model.to_string()));
        }
        Self::from_coefficients(d).map(Some)
    }

    /// OpenCV's distortion from its coefficients (4, 5, 8 or 12 of them).
    pub fn from_coefficients(d: &[f64]) -> Result<Self> {
        if !matches!(d.len(), 4 | 5 | 8 | 12) {
            return Err(Error::DistortionLength(d.len()));
        }
        let mut k = [0.0; 12];
        k.iter_mut().zip(d).for_each(|(k, d)| *k = *d);
        Ok(Self(k))
    }

    /// Where a point on the normalised picture plane (x/z, y/z) lands after the lens
    /// bends it (OpenCV's `projectPoints`, its sums in its order).
    pub fn apply(&self, x: f64, y: f64) -> (f64, f64) {
        let k = &self.0;
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        let a1 = 2.0 * x * y;
        let a2 = r2 + 2.0 * x * x;
        let a3 = r2 + 2.0 * y * y;
        let cdist = 1.0 + k[0] * r2 + k[1] * r4 + k[4] * r6;
        let icdist2 = 1.0 / (1.0 + k[5] * r2 + k[6] * r4 + k[7] * r6);
        let xd = x * cdist * icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
        let yd = y * cdist * icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;
        (xd, yd)
    }
}

/// A camera's intrinsics, as in `sensor_msgs/CameraInfo`: K, and D.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lens {
    /// px
    pub width: usize,
    /// px
    pub height: usize,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// None: no distortion.
    pub distortion: Option<Distortion>,
}

impl Lens {
    /// A lens with no distortion, centred on the picture: the bench camera's kind.
    pub fn pinhole(width: usize, height: usize, focal: f64) -> Self {
        Self {
            width,
            height,
            fx: focal,
            fy: focal,
            cx: width as f64 / 2.0,
            cy: height as f64 / 2.0,
            distortion: None,
        }
    }
}

/// Where the camera sits on the car, as vehicle.yaml gives it (base_link to the camera).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mount {
    /// m; z is the height above the road.
    pub xyz: [f64; 3],
    /// rad: roll, pitch (+ = looking down), yaw.
    pub rpy: [f64; 3],
}

/// The patch of road the bird's-eye view covers, in base_link.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    /// m ahead of base_link: the near edge.
    pub x_min: f64,
    /// m ahead: the far edge.
    pub x_max: f64,
    /// m either side.
    pub y_max: f64,
    /// m per pixel.
    pub resolution: f64,
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            x_min: 4.0,
            x_max: 16.0,
            y_max: 4.0,
            resolution: 0.05,
        }
    }
}

impl Grid {
    pub fn rows(&self) -> usize {
        ((self.x_max - self.x_min) / self.resolution)
            .round_ties_even()
            .max(0.0) as usize
    }

    pub fn cols(&self) -> usize {
        (2.0 * self.y_max / self.resolution)
            .round_ties_even()
            .max(0.0) as usize
    }

    /// The forward distance (m) of a row's centre.
    pub fn x_of_row(&self, row: f64) -> f64 {
        self.x_max - (row + 0.5) * self.resolution
    }

    /// The sideways distance (m, + = left) of a column's centre.
    pub fn y_of_col(&self, col: f64) -> f64 {
        self.y_max - (col + 0.5) * self.resolution
    }

    pub fn row_of_x(&self, x: f64) -> f64 {
        (self.x_max - x) / self.resolution - 0.5
    }

    pub fn col_of_y(&self, y: f64) -> f64 {
        (self.y_max - y) / self.resolution - 0.5
    }
}

/// The matrix that turns the camera's axes into base_link's (roll, pitch, yaw).
pub fn rotation(rpy: [f64; 3]) -> [[f64; 3]; 3] {
    let [roll, pitch, yaw] = rpy;
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// Where points in base_link (m) land in the picture: Some(u, v) in pixels for the points
/// in front of the lens and inside the picture. Pixel centres are whole numbers, as in
/// OpenCV.
pub fn project(points: &[[f64; 3]], lens: &Lens, mount: &Mount) -> Vec<Option<[f64; 2]>> {
    let r = rotation(mount.rpy);
    let (w, h) = (lens.width as f64, lens.height as f64);
    points
        .iter()
        .map(|p| {
            let d = [
                p[0] - mount.xyz[0],
                p[1] - mount.xyz[1],
                p[2] - mount.xyz[2],
            ];
            // (p - xyz) @ R: the point in the camera's axes.
            let c: [f64; 3] =
                std::array::from_fn(|j| d[0] * r[0][j] + d[1] * r[1][j] + d[2] * r[2][j]);
            // The optical frame: x right, y down, z along the lens's axis.
            let (ox, oy, oz) = (-c[1], -c[2], c[0]);
            if oz <= NEAREST {
                return None;
            }
            // Distortion models go wild far outside the picture, so judge "nearly in view"
            // by the plain pinhole first.
            let plain = [lens.fx * ox / oz + lens.cx, lens.fy * oy / oz + lens.cy];
            let near_view = plain[0] > -0.25 * w
                && plain[0] < 1.25 * w
                && plain[1] > -0.25 * h
                && plain[1] < 1.25 * h;
            if !near_view {
                return None;
            }
            let uv = match &lens.distortion {
                None => plain,
                Some(d) => {
                    let z = 1.0 / oz;
                    let (xd, yd) = d.apply(ox * z, oy * z);
                    [xd * lens.fx + lens.cx, yd * lens.fy + lens.cy]
                }
            };
            let inside = uv[0] >= 0.0 && uv[0] <= w - 1.0 && uv[1] >= 0.0 && uv[1] <= h - 1.0;
            inside.then_some(uv)
        })
        .collect()
}

/// One cell's four source pixels: the top-left one, and each one's weight (of 2^15).
#[derive(Debug, Clone, Copy)]
struct Tap {
    x: i32,
    y: i32,
    weights: [i32; 4],
}

impl Tap {
    /// OpenCV's `convertMaps` to `CV_16SC2` (the place, as a float32, to 1/32 of a pixel,
    /// rounded half to even), and its bilinear weights.
    fn new(u: f32, v: f32) -> Self {
        let ix = (u * TAB_SIZE as f32).round_ties_even() as i32;
        let iy = (v * TAB_SIZE as f32).round_ties_even() as i32;
        let (fx, fy) = (ix & (TAB_SIZE - 1), iy & (TAB_SIZE - 1));
        // (1 - a)(1 - b) and the rest, in 1/1024ths, then as 2^15ths: exact, so they sum
        // to 2^15 as OpenCV's table does.
        let s = 1 << (COEF_BITS - 2 * TAB_BITS);
        Tap {
            x: ix >> TAB_BITS,
            y: iy >> TAB_BITS,
            weights: [
                (TAB_SIZE - fx) * (TAB_SIZE - fy) * s,
                fx * (TAB_SIZE - fy) * s,
                (TAB_SIZE - fx) * fy * s,
                fx * fy * s,
            ],
        }
    }
}

/// Warps camera pictures into the bird's-eye view: one look-up table, made once.
#[derive(Debug, Clone)]
pub struct BirdsEye {
    lens: Lens,
    mount: Mount,
    grid: Grid,
    taps: Vec<Tap>,
    valid: Vec<bool>,
}

impl BirdsEye {
    pub fn new(lens: Lens, mount: Mount, grid: Grid) -> Self {
        let (rows, cols) = (grid.rows(), grid.cols());
        let ground: Vec<[f64; 3]> = (0..rows)
            .flat_map(|r| {
                (0..cols).map(move |c| [grid.x_of_row(r as f64), grid.y_of_col(c as f64), 0.0])
            })
            .collect();
        let uv = project(&ground, &lens, &mount);
        // Outside the picture: -10 px, so every tap reads the border colour, black.
        let taps = uv
            .iter()
            .map(|p| {
                let [u, v] = p.unwrap_or([-10.0, -10.0]);
                Tap::new(u as f32, v as f32)
            })
            .collect();
        let valid = uv.iter().map(Option::is_some).collect();
        Self {
            lens,
            mount,
            grid,
            taps,
            valid,
        }
    }

    pub fn lens(&self) -> &Lens {
        &self.lens
    }

    pub fn mount(&self) -> &Mount {
        &self.mount
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    /// Which cells the camera sees, row by row.
    pub fn valid(&self) -> &[bool] {
        &self.valid
    }

    /// The bird's-eye view of one picture: OpenCV's `remap`, `INTER_LINEAR`, a black
    /// border, in fixed point, so the same grey levels.
    pub fn warp(&self, picture: &Picture<'_>) -> Result<Bev> {
        let (w, h) = (picture.width(), picture.height());
        if (w, h) != (self.lens.width, self.lens.height) {
            return Err(Error::WrongSize {
                picture: (w, h),
                lens: (self.lens.width, self.lens.height),
            });
        }
        let at = picture.encoding().bgr_at();
        let (wi, hi) = (w as i32, h as i32);
        let half = 1 << (COEF_BITS - 1);
        let pixels = self
            .taps
            .iter()
            .map(|tap| {
                let corners = [
                    (tap.x, tap.y),
                    (tap.x + 1, tap.y),
                    (tap.x, tap.y + 1),
                    (tap.x + 1, tap.y + 1),
                ];
                let mut sum = [0i32; 3];
                for ((x, y), weight) in corners.into_iter().zip(tap.weights) {
                    if weight == 0 || x < 0 || y < 0 || x >= wi || y >= hi {
                        continue; // the border: black
                    }
                    let px = picture.pixel(x as usize, y as usize);
                    for (s, i) in sum.iter_mut().zip(at) {
                        *s += i32::from(px[i]) * weight;
                    }
                }
                sum.map(|s| ((s + half) >> COEF_BITS).clamp(0, 255) as u8)
            })
            .collect();
        Ok(Bev {
            rows: self.grid.rows(),
            cols: self.grid.cols(),
            pixels,
        })
    }

    /// How far ahead of base_link (m) the camera first sees the road, in the grid; None
    /// if it sees none of it.
    pub fn nearest_seen(&self) -> Option<f64> {
        let cols = self.grid.cols().max(1);
        self.valid
            .chunks(cols)
            .rposition(|row| row.iter().any(|&v| v))
            .map(|row| self.grid.x_of_row(row as f64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::Encoding;

    const BENCH: [f64; 3] = [1.80, 0.29, 1.20];

    fn lens() -> Lens {
        Lens::pinhole(640, 360, 343.0)
    }

    fn bench() -> Mount {
        Mount {
            xyz: BENCH,
            rpy: [0.0; 3],
        }
    }

    #[test]
    fn a_point_on_the_ground_lands_where_the_pinhole_says() {
        let uv = project(&[[11.8, 0.29, 0.0]], &lens(), &bench())[0].unwrap();
        assert!((uv[0] - 320.0).abs() < 1e-6);
        assert!((uv[1] - (180.0 + 343.0 * 1.2 / 10.0)).abs() < 1e-6);
    }

    #[test]
    fn tilting_the_camera_down_lifts_the_horizon() {
        let tilted = Mount {
            xyz: BENCH,
            rpy: [0.0, 0.1, 0.0],
        };
        let uv = project(&[[1e5, 0.29, 0.0]], &lens(), &tilted)[0].unwrap();
        assert!((uv[1] - (180.0 - 343.0 * 0.1f64.tan())).abs() < 0.5);
    }

    #[test]
    fn points_behind_or_outside_the_picture_are_not_seen() {
        let seen = project(
            &[[-5.0, 0.0, 0.0], [5.0, 30.0, 0.0], [2.5, 0.29, 0.0]],
            &lens(),
            &bench(),
        );
        assert!(seen.iter().all(Option::is_none));
    }

    #[test]
    fn the_bench_camera_sees_the_road_from_about_4_m() {
        let grid = Grid {
            x_min: 2.0,
            ..Grid::default()
        };
        let view = BirdsEye::new(lens(), bench(), grid);
        let near = view.nearest_seen().unwrap();
        assert!((near - (1.8 + 1.2 * 343.0 / 179.5)).abs() < 0.06);
    }

    #[test]
    fn a_picture_of_the_wrong_size_is_refused() {
        let view = BirdsEye::new(lens(), bench(), Grid::default());
        let data = vec![0u8; 640 * 480 * 3];
        let picture = Picture::packed(&data, 640, 480, Encoding::Bgr8).unwrap();
        assert!(matches!(view.warp(&picture), Err(Error::WrongSize { .. })));
    }

    #[test]
    fn a_flat_picture_warps_flat_and_the_unseen_stays_black() {
        let view = BirdsEye::new(lens(), bench(), Grid::default());
        let data = vec![100u8; 640 * 360];
        let picture = Picture::packed(&data, 640, 360, Encoding::Mono8).unwrap();
        let bev = view.warp(&picture).unwrap();
        for (p, &seen) in bev.pixels.iter().zip(view.valid()) {
            assert_eq!(*p, if seen { [100; 3] } else { [0; 3] });
        }
    }

    #[test]
    fn distortion_needs_a_known_model_and_length() {
        assert_eq!(
            Distortion::from_camera_info("plumb_bob", &[0.0; 5]),
            Ok(None)
        );
        assert!(Distortion::from_camera_info("equidistant", &[0.1, 0.0, 0.0, 0.0]).is_err());
        assert!(Distortion::from_camera_info("plumb_bob", &[0.1, 0.0, 0.0]).is_err());
        assert!(
            Distortion::from_camera_info("plumb_bob", &[0.1, 0.0, 0.0, 0.0, 0.0])
                .unwrap()
                .is_some()
        );
    }
}

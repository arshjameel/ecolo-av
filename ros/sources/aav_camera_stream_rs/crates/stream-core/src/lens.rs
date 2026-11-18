//! A camera's calibration, and undistorting its pictures (`lens.py`).
//!
//! Reads ROS's two calibration formats: the INI that camera_calibration writes and
//! camera_info_manager's YAML. A calibration made at another size of the same picture is
//! scaled; one of another shape is refused, since a crop moves the centre. Only the lens
//! (K, D) is used, never a stereo pair's rectification.
//!
//! [`Undistorter`] is OpenCV's `getOptimalNewCameraMatrix`, `initUndistortRectifyMap` to
//! `CV_16SC2` and `remap` with `INTER_LINEAR` and a black border, in plain Rust: the map
//! is made once, then each picture is one pass over it, in OpenCV's own fixed point (1/32
//! of a pixel, weights in 2^15ths).

use std::path::Path;

use crate::{Error, Result};

/// The distortion models OpenCV undistorts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Model {
    PlumbBob,
    RationalPolynomial,
    /// Any other name a file gives: read, but not undistorted.
    Other(String),
}

impl Model {
    pub fn parse(name: &str) -> Self {
        match name {
            "plumb_bob" => Model::PlumbBob,
            "rational_polynomial" => Model::RationalPolynomial,
            other => Model::Other(other.to_string()),
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Model::PlumbBob => "plumb_bob",
            Model::RationalPolynomial => "rational_polynomial",
            Model::Other(name) => name,
        }
    }
}

/// A camera's lens, as `sensor_msgs/CameraInfo` has it.
#[derive(Debug, Clone, PartialEq)]
pub struct Lens {
    /// px
    pub width: u32,
    /// px
    pub height: u32,
    /// 3x3, row by row.
    pub k: Vec<f64>,
    pub d: Vec<f64>,
    pub model: Model,
}

impl Lens {
    pub fn new(width: u32, height: u32, k: Vec<f64>) -> Self {
        Lens {
            width,
            height,
            k,
            d: vec![0.0; 5],
            model: Model::PlumbBob,
        }
    }
}

fn numbers<'a>(lines: impl Iterator<Item = &'a str>) -> Result<Vec<f64>> {
    lines
        .flat_map(str::split_whitespace)
        .map(|x| {
            x.parse::<f64>().map_err(|_| {
                Error::Calibration(format!("could not convert string to float: '{x}'"))
            })
        })
        .collect()
}

/// A camera_calibration INI: width, height, camera matrix, distortion.
pub fn parse_ini(text: &str) -> Result<Lens> {
    let lines: Vec<&str> = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect();
    let block = |name: &str, rows: usize| -> Result<Vec<f64>> {
        let i = lines
            .iter()
            .position(|l| *l == name)
            .ok_or_else(|| Error::Calibration(format!("'{name}' is not in list")))?;
        numbers(lines.iter().skip(i + 1).take(rows).copied())
    };
    let first = |v: Vec<f64>| -> Result<u32> {
        v.first()
            .map(|&x| x as u32)
            .ok_or_else(|| Error::Calibration("list index out of range".to_string()))
    };
    let read = || -> Result<Lens> {
        let width = first(block("width", 1)?)?;
        let height = first(block("height", 1)?)?;
        let k = block("camera matrix", 3)?;
        let d = block("distortion", 1)?;
        Ok(Lens {
            width,
            height,
            k,
            d,
            model: Model::PlumbBob,
        })
    };
    read().map_err(|e| Error::Calibration(format!("not a camera_calibration INI file: {e}")))
}

/// camera_info_manager's YAML: image_width, image_height, camera_matrix.data...
pub fn parse_yaml(text: &str) -> Result<Lens> {
    use serde_yaml_ng::Value;
    let data: Value = serde_yaml_ng::from_str(text)
        .map_err(|e| Error::Calibration(format!("not a camera_info YAML file: {e}")))?;
    let missing =
        |key: &str| Error::Calibration(format!("not a camera_info YAML file: missing '{key}'"));
    let whole = |key: &str| -> Result<u32> {
        let v = data.get(key).ok_or_else(|| missing(key))?;
        v.as_u64()
            .or_else(|| v.as_f64().map(|f| f as u64))
            .and_then(|n| u32::try_from(n).ok())
            .ok_or_else(|| missing(key))
    };
    let list = |v: &Value, key: &str| -> Result<Vec<f64>> {
        v.get("data")
            .and_then(Value::as_sequence)
            .ok_or_else(|| missing("data"))?
            .iter()
            .map(|x| x.as_f64().ok_or_else(|| missing(key)))
            .collect()
    };
    let k = list(
        data.get("camera_matrix")
            .ok_or_else(|| missing("camera_matrix"))?,
        "camera_matrix",
    )?;
    let d = match data.get("distortion_coefficients") {
        Some(v) => list(v, "distortion_coefficients")?,
        None => vec![0.0; 5],
    };
    let model = data
        .get("distortion_model")
        .and_then(Value::as_str)
        .map_or(Model::PlumbBob, Model::parse);
    Ok(Lens {
        width: whole("image_width")?,
        height: whole("image_height")?,
        k,
        d,
        model,
    })
}

/// A calibration file, INI or YAML by its ending.
pub fn load(path: &Path) -> Result<Lens> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| Error::Calibration(format!("{}: {e}", path.display())))?;
    let lens = if path.extension().is_some_and(|e| e == "ini") {
        parse_ini(&text)?
    } else {
        parse_yaml(&text)?
    };
    if lens.k.len() != 9 {
        return Err(Error::Calibration(format!(
            "{}: the camera matrix needs 9 numbers, not {}",
            path.display(),
            lens.k.len()
        )));
    }
    Ok(lens)
}

/// The same lens for the picture scaled to width x height (the same shape).
pub fn scaled(lens: &Lens, width: u32, height: u32) -> Result<Lens> {
    let sx = f64::from(width) / f64::from(lens.width);
    let sy = f64::from(height) / f64::from(lens.height);
    if (sx - sy).abs() > 0.01 * sx.max(sy) {
        return Err(Error::Calibration(format!(
            "a {}x{} calibration does not fit a {width}x{height} picture: another shape, a crop",
            lens.width, lens.height
        )));
    }
    let [fx, _, cx, _, fy, cy, ..] = k9(lens)?;
    // Pixel centres: OpenCV's pixel (0, 0) is the middle of the first pixel.
    Ok(Lens {
        width,
        height,
        k: vec![
            fx * sx,
            0.0,
            (cx + 0.5) * sx - 0.5,
            0.0,
            fy * sy,
            (cy + 0.5) * sy - 0.5,
            0.0,
            0.0,
            1.0,
        ],
        d: lens.d.clone(),
        model: lens.model.clone(),
    })
}

fn k9(lens: &Lens) -> Result<[f64; 9]> {
    <[f64; 9]>::try_from(lens.k.as_slice()).map_err(|_| {
        Error::Calibration(format!(
            "the camera matrix needs 9 numbers, not {}",
            lens.k.len()
        ))
    })
}

/// A lens with no distortion: a simulator's camera.
pub fn identity(width: u32, height: u32, focal: f64) -> Lens {
    Lens::new(
        width,
        height,
        vec![
            focal,
            0.0,
            (f64::from(width) - 1.0) / 2.0,
            0.0,
            focal,
            (f64::from(height) - 1.0) / 2.0,
            0.0,
            0.0,
            1.0,
        ],
    )
}

/// The 3x4 P of a lens with no distortion: K beside a zero column.
pub fn projection(lens: &Lens) -> [f64; 12] {
    let k = |i: usize| lens.k.get(i).copied().unwrap_or(0.0);
    [
        k(0),
        k(1),
        k(2),
        0.0,
        k(3),
        k(4),
        k(5),
        0.0,
        k(6),
        k(7),
        k(8),
        0.0,
    ]
}

/// OpenCV's distortion coefficients in its order (k1, k2, p1, p2, k3, k4, k5, k6, s1-s4),
/// the missing ones 0. It takes 4, 5, 8 or 12.
fn coefficients(d: &[f64]) -> Result<[f64; 12]> {
    if !matches!(d.len(), 4 | 5 | 8 | 12) {
        return Err(Error::Calibration(format!(
            "{} distortion coefficients: OpenCV takes 4, 5, 8 or 12",
            d.len()
        )));
    }
    let mut k = [0.0; 12];
    k.iter_mut().zip(d).for_each(|(k, d)| *k = *d);
    Ok(k)
}

/// OpenCV's `undistortPoints` to normalised coordinates (no R, no P): 5 rounds of its
/// fixed-point iteration, as its default criteria.
fn undistort_point(k: &[f64; 9], d: &[f64; 12], u: f64, v: f64) -> (f64, f64) {
    let (fx, fy, cx, cy) = (k[0], k[4], k[2], k[5]);
    let (ifx, ify) = (1.0 / fx, 1.0 / fy);
    let x0 = (u - cx) * ifx;
    let y0 = (v - cy) * ify;
    let (mut x, mut y) = (x0, y0);
    for _ in 0..5 {
        let r2 = x * x + y * y;
        let icdist = (1.0 + ((d[7] * r2 + d[6]) * r2 + d[5]) * r2)
            / (1.0 + ((d[4] * r2 + d[1]) * r2 + d[0]) * r2);
        if icdist < 0.0 {
            return (x0, y0);
        }
        let delta_x = 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x) + d[8] * r2 + d[9] * r2 * r2;
        let delta_y = d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y + d[10] * r2 + d[11] * r2 * r2;
        x = (x0 - delta_x) * icdist;
        y = (y0 - delta_y) * icdist;
    }
    (x, y)
}

/// OpenCV's `getOptimalNewCameraMatrix` (the principal point not centred): K with its
/// focal lengths and centre between those of the rectangle inside every undistorted pixel
/// (alpha 0) and the one around them all (alpha 1).
pub fn optimal_new_camera_matrix(lens: &Lens, alpha: f64) -> Result<[f64; 9]> {
    let k = k9(lens)?;
    let d = coefficients(&lens.d)?;
    const N: usize = 9;
    let (w, h) = (f64::from(lens.width), f64::from(lens.height));
    let max = f64::from(f32::MAX);
    let (mut ix0, mut ix1, mut iy0, mut iy1) = (-max, max, -max, max);
    let (mut ox0, mut ox1, mut oy0, mut oy1) = (max, -max, max, -max);
    for yi in 0..N {
        for xi in 0..N {
            let (x, y) = undistort_point(
                &k,
                &d,
                xi as f64 * (w - 1.0) / (N - 1) as f64,
                yi as f64 * (h - 1.0) / (N - 1) as f64,
            );
            ox0 = ox0.min(x);
            ox1 = ox1.max(x);
            oy0 = oy0.min(y);
            oy1 = oy1.max(y);
            if xi == 0 {
                ix0 = ix0.max(x);
            }
            if xi == N - 1 {
                ix1 = ix1.min(x);
            }
            if yi == 0 {
                iy0 = iy0.max(y);
            }
            if yi == N - 1 {
                iy1 = iy1.min(y);
            }
        }
    }
    let fx0 = (w - 1.0) / (ix1 - ix0);
    let fy0 = (h - 1.0) / (iy1 - iy0);
    let cx0 = -fx0 * ix0;
    let cy0 = -fy0 * iy0;
    let fx1 = (w - 1.0) / (ox1 - ox0);
    let fy1 = (h - 1.0) / (oy1 - oy0);
    let cx1 = -fx1 * ox0;
    let cy1 = -fy1 * oy0;
    let mut m = k;
    m[0] = fx0 * (1.0 - alpha) + fx1 * alpha;
    m[4] = fy0 * (1.0 - alpha) + fy1 * alpha;
    m[2] = cx0 * (1.0 - alpha) + cx1 * alpha;
    m[5] = cy0 * (1.0 - alpha) + cy1 * alpha;
    Ok(m)
}

/// OpenCV's `invert` of a 3x3 matrix (`DECOMP_LU` takes its closed form at this size).
fn invert3(m: &[f64; 9]) -> Result<[f64; 9]> {
    let s = |r: usize, c: usize| m[r * 3 + c];
    let det = s(0, 0) * (s(1, 1) * s(2, 2) - s(1, 2) * s(2, 1))
        - s(0, 1) * (s(1, 0) * s(2, 2) - s(1, 2) * s(2, 0))
        + s(0, 2) * (s(1, 0) * s(2, 1) - s(1, 1) * s(2, 0));
    if det == 0.0 {
        return Err(Error::Calibration(
            "the new camera matrix has no inverse".to_string(),
        ));
    }
    let d = 1.0 / det;
    Ok([
        (s(1, 1) * s(2, 2) - s(1, 2) * s(2, 1)) * d,
        (s(0, 2) * s(2, 1) - s(0, 1) * s(2, 2)) * d,
        (s(0, 1) * s(1, 2) - s(0, 2) * s(1, 1)) * d,
        (s(1, 2) * s(2, 0) - s(1, 0) * s(2, 2)) * d,
        (s(0, 0) * s(2, 2) - s(0, 2) * s(2, 0)) * d,
        (s(0, 2) * s(1, 0) - s(0, 0) * s(1, 2)) * d,
        (s(1, 0) * s(2, 1) - s(1, 1) * s(2, 0)) * d,
        (s(0, 1) * s(2, 0) - s(0, 0) * s(2, 1)) * d,
        (s(0, 0) * s(1, 1) - s(0, 1) * s(1, 0)) * d,
    ])
}

/// OpenCV's `initUndistortRectifyMap` with no rectification: for each pixel of the
/// undistorted picture (row by row), where it comes from in the camera's (px).
pub fn undistort_map(lens: &Lens, new_k: &[f64; 9]) -> Result<Vec<(f64, f64)>> {
    let k = k9(lens)?;
    let d = coefficients(&lens.d)?;
    let ir = invert3(new_k)?;
    let (u0, v0, fx, fy) = (k[2], k[5], k[0], k[4]);
    let (w, h) = (lens.width as usize, lens.height as usize);
    let mut out = Vec::with_capacity(w * h);
    for i in 0..h {
        let i = i as f64;
        let (row_x, row_y, row_w) = (i * ir[1] + ir[2], i * ir[4] + ir[5], i * ir[7] + ir[8]);
        out.extend((0..w).map(|j| {
            let j = j as f64;
            let ww = 1.0 / (row_w + j * ir[6]);
            let x = (row_x + j * ir[0]) * ww;
            let y = (row_y + j * ir[3]) * ww;
            let (x2, y2) = (x * x, y * y);
            let r2 = x2 + y2;
            let xy2 = 2.0 * x * y;
            let kr = (1.0 + ((d[4] * r2 + d[1]) * r2 + d[0]) * r2)
                / (1.0 + ((d[7] * r2 + d[6]) * r2 + d[5]) * r2);
            let xd = x * kr + d[2] * xy2 + d[3] * (r2 + 2.0 * x2) + d[8] * r2 + d[9] * r2 * r2;
            let yd = y * kr + d[2] * (r2 + 2.0 * y2) + d[3] * xy2 + d[10] * r2 + d[11] * r2 * r2;
            (fx * xd + u0, fy * yd + v0)
        }));
    }
    Ok(out)
}

/// OpenCV's fixed-point remap: 1/32 of a pixel, weights summing to 2^15.
const TAB_BITS: u32 = 5;
const TAB_SIZE: i32 = 1 << TAB_BITS;
const COEF_BITS: u32 = 15;
/// Threads one picture is shared between, at most (OpenCV's remap splits it too).
const MAX_THREADS: usize = 4;

/// One output pixel's source: the top-left of its four pixels, and where it falls
/// between them in 1/32 px (OpenCV's `CV_16SC2` map and its table index). 12 bytes, so
/// a 1280 x 720 map is 11 MB.
#[derive(Debug, Clone, Copy)]
struct Tap {
    x: i32,
    y: i32,
    frac: u16,
}

impl Tap {
    /// The place to 1/32 of a pixel, rounded half to even, as OpenCV's `CV_16SC2`.
    fn new(u: f64, v: f64) -> Self {
        let clamp = |z: f64| (z * f64::from(TAB_SIZE)).round_ties_even().clamp(-1e9, 1e9) as i32;
        let (ix, iy) = (clamp(u), clamp(v));
        Tap {
            x: ix >> TAB_BITS,
            y: iy >> TAB_BITS,
            frac: ((iy & (TAB_SIZE - 1)) * TAB_SIZE + (ix & (TAB_SIZE - 1))) as u16,
        }
    }

    /// OpenCV's bilinear weights: exact, so they sum to 2^15 as its table does.
    fn weights(self) -> [i32; 4] {
        let fx = i32::from(self.frac) & (TAB_SIZE - 1);
        let fy = i32::from(self.frac) >> TAB_BITS;
        let s = 1 << (COEF_BITS - 2 * TAB_BITS);
        [
            (TAB_SIZE - fx) * (TAB_SIZE - fy) * s,
            fx * (TAB_SIZE - fy) * s,
            (TAB_SIZE - fx) * fy * s,
            fx * fy * s,
        ]
    }
}

/// Undistorts pictures of one size. alpha 0 keeps only valid pixels; 1 keeps them all.
#[derive(Debug, Clone)]
pub struct Undistorter {
    out: Lens,
    taps: Vec<Tap>,
}

impl Undistorter {
    pub fn new(lens: &Lens, alpha: f64) -> Result<Self> {
        if let Model::Other(name) = &lens.model {
            return Err(Error::Calibration(format!(
                "distortion model {name}: only plumb_bob and rational_polynomial"
            )));
        }
        let new_k = optimal_new_camera_matrix(lens, alpha)?;
        let taps = undistort_map(lens, &new_k)?
            .into_iter()
            .map(|(u, v)| Tap::new(u, v))
            .collect();
        Ok(Undistorter {
            out: Lens::new(lens.width, lens.height, new_k.to_vec()),
            taps,
        })
    }

    /// The picture after undistortion: K', no distortion.
    pub fn lens(&self) -> &Lens {
        &self.out
    }

    /// The undistorted picture of a packed BGR one of the lens's size: rows shared
    /// between up to 4 threads, each pixel the same either way.
    pub fn apply(&self, bgr: &[u8]) -> Result<Vec<u8>> {
        let (w, h) = (self.out.width as usize, self.out.height as usize);
        if bgr.len() != w * h * 3 {
            return Err(Error::Invalid(format!(
                "a picture of {} bytes is not {w}x{h} BGR",
                bgr.len()
            )));
        }
        let mut out = vec![0u8; bgr.len()];
        let threads = std::thread::available_parallelism()
            .map_or(1, |n| n.get())
            .clamp(1, MAX_THREADS);
        let rows = h.div_ceil(threads).max(1);
        std::thread::scope(|scope| {
            for (taps, part) in self.taps.chunks(rows * w).zip(out.chunks_mut(rows * w * 3)) {
                scope.spawn(move || remap(bgr, w, h, taps, part));
            }
        });
        Ok(out)
    }
}

/// OpenCV's `remap`, `INTER_LINEAR`, a black border, for some of the output's pixels.
fn remap(bgr: &[u8], w: usize, h: usize, taps: &[Tap], out: &mut [u8]) {
    let half = 1 << (COEF_BITS - 1);
    let row = 3 * w;
    let (wi, hi) = (w as i32, h as i32);
    for (tap, px) in taps.iter().zip(out.as_chunks_mut::<3>().0) {
        let [w0, w1, w2, w3] = tap.weights();
        // Most taps: all four pixels inside, no checks per pixel.
        if tap.x >= 0 && tap.y >= 0 && tap.x + 1 < wi && tap.y + 1 < hi {
            let at = 3 * (tap.y as usize * w + tap.x as usize);
            let (top, bottom) = (&bgr[at..at + 6], &bgr[at + row..at + row + 6]);
            for (c, p) in px.iter_mut().enumerate() {
                let s = i32::from(top[c]) * w0
                    + i32::from(top[c + 3]) * w1
                    + i32::from(bottom[c]) * w2
                    + i32::from(bottom[c + 3]) * w3;
                *p = ((s + half) >> COEF_BITS).clamp(0, 255) as u8;
            }
            continue;
        }
        let corners = [
            (tap.x, tap.y, w0),
            (tap.x + 1, tap.y, w1),
            (tap.x, tap.y + 1, w2),
            (tap.x + 1, tap.y + 1, w3),
        ];
        let mut sum = [0i32; 3];
        for (x, y, weight) in corners {
            if weight == 0 || x < 0 || y < 0 || x >= wi || y >= hi {
                continue; // the border: black
            }
            let at = 3 * (y as usize * w + x as usize);
            for (s, &b) in sum.iter_mut().zip(&bgr[at..at + 3]) {
                *s += i32::from(b) * weight;
            }
        }
        for (p, s) in px.iter_mut().zip(sum) {
            *p = ((s + half) >> COEF_BITS).clamp(0, 255) as u8;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_distortion_keeps_the_picture() {
        let lens = identity(32, 18, 30.0);
        let u = Undistorter::new(&lens, 0.0).expect("a lens");
        let picture: Vec<u8> = (0..32 * 18 * 3).map(|i| (i % 251) as u8).collect();
        assert_eq!(u.apply(&picture).expect("the size"), picture);
    }

    #[test]
    fn another_shape_is_refused() {
        let lens = identity(1920, 1080, 1000.0);
        assert!(scaled(&lens, 640, 480).is_err());
        assert!(scaled(&lens, 1280, 720).is_ok());
    }
}

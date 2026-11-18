//! Names for the things: the camera's boxes matched to the lidar's things.
//!
//! TODO 192 (Ben, 15 Nov: "How the camera perception informs the lidar distance stuff"):
//!
//! ```text
//! a thing (lidar: where, how big) ─► its box drawn into each camera's picture
//! the detector's boxes (camera: what) ─► the one overlapping it most (IoU ≥ min_iou)
//! ─► the thing takes that box's label: person, car, stop sign...; kept `hold` s
//! ```
//!
//! The lidar's cubes stop at a ceiling (`aav_voxels`' max_height, 2 m): a thing reaching
//! it has an unknown top. A stop sign's plate sits mostly above it, so its thing is the
//! pole. Such a thing is matched on its bearing instead: the boxes overlap left to right
//! by at least `min_side` of the narrower, and at all top to bottom. A thing seen whole
//! keeps the strict IoU. A thing no camera sees keeps its last name for `hold` s, then
//! goes back to unknown: never guessed.
//!
//! The pinhole is `aav_detector`'s `ground.py` (`Lens`, `Mount`), ported here.

use std::collections::HashMap;

use crate::UNKNOWN;

/// Pixels: left, top, right, bottom.
pub type PixelBox = [f64; 4];

/// A pinhole camera's intrinsics, in pixels: CameraInfo's K, and the picture's size.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lens {
    /// Focal length along u.
    pub fx: f64,
    /// Focal length along v.
    pub fy: f64,
    /// The optical centre's u.
    pub cx: f64,
    /// The optical centre's v.
    pub cy: f64,
    /// Pixels across.
    pub width: u32,
    /// Pixels down.
    pub height: u32,
}

impl Lens {
    /// CameraInfo's K (row by row, 3 × 3) and the picture's size; `None` for an
    /// uncalibrated camera (K is 0).
    #[must_use]
    pub fn from_k(k: &[f64; 9], width: u32, height: u32) -> Option<Self> {
        let (fx, fy) = (k[0], k[4]);
        (fx > 0.0 && fy > 0.0 && width > 0 && height > 0).then_some(Self {
            fx,
            fy,
            cx: k[2],
            cy: k[5],
            width,
            height,
        })
    }
}

/// Which way a camera's frame faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axes {
    /// As base_link: x forward, y left, z up (`camera_left_link`).
    Body,
    /// REP-103's optical frame: z forward, x right, y down.
    Optical,
}

impl Axes {
    /// An optical frame by its name (ends in `_optical` or `_optical_frame`).
    #[must_use]
    pub fn of_frame(frame_id: &str) -> Self {
        if frame_id.ends_with("_optical") || frame_id.ends_with("_optical_frame") {
            Self::Optical
        } else {
            Self::Body
        }
    }
}

/// Where a camera is on the car: its position in base_link, and its rotation to it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraMount {
    /// m, in base_link.
    pub position: [f64; 3],
    /// Turns a direction in the camera's frame into base_link (row by row).
    pub rotation: [[f64; 3]; 3],
    /// Which way its frame faces.
    pub axes: Axes,
}

/// The rotation matrix of a unit quaternion (TF's x, y, z, w).
#[must_use]
pub fn rotation(x: f64, y: f64, z: f64, w: f64) -> [[f64; 3]; 3] {
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// A base_link point in the camera's own frame (the mount's rotation undone).
fn to_camera(mount: &CameraMount, p: [f64; 3]) -> [f64; 3] {
    let d = [0, 1, 2].map(|i| p[i] - mount.position[i]);
    let r = &mount.rotation;
    // The transpose: a rotation's inverse.
    [0, 1, 2].map(|col| r[0][col] * d[0] + r[1][col] * d[1] + r[2][col] * d[2])
}

/// The pixel (u, v) a base_link point falls on; `None` behind the camera.
#[must_use]
pub fn project(lens: &Lens, mount: &CameraMount, p: [f64; 3]) -> Option<[f64; 2]> {
    let [x, y, z] = to_camera(mount, p);
    let (forward, right, down) = match mount.axes {
        Axes::Optical => (z, x, y),
        Axes::Body => (x, -y, -z),
    };
    (forward > 0.05).then(|| {
        [
            lens.cx + lens.fx * right / forward,
            lens.cy + lens.fy * down / forward,
        ]
    })
}

/// The picture's box round a thing standing at (x, y) in base_link: an upright box from
/// the road to `height`, `length` along x and `width` along y. Its corners in front of the
/// camera are drawn and the box round them clipped to the picture; `None` if no corner is
/// in front, or it falls off the picture.
#[must_use]
pub fn thing_box(
    lens: &Lens,
    mount: &CameraMount,
    x: f64,
    y: f64,
    length: f64,
    width: f64,
    height: f64,
) -> Option<PixelBox> {
    let mut pixels = Vec::with_capacity(8);
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for z in [0.0, height] {
                let corner = [x + sx * length / 2.0, y + sy * width / 2.0, z];
                pixels.extend(project(lens, mount, corner));
            }
        }
    }
    if pixels.is_empty() {
        return None;
    }
    let lo = |i: usize| pixels.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min);
    let hi = |i: usize| {
        pixels
            .iter()
            .map(|p| p[i])
            .fold(f64::NEG_INFINITY, f64::max)
    };
    let u0 = lo(0).max(0.0);
    let u1 = hi(0).min(f64::from(lens.width));
    let v0 = lo(1).max(0.0);
    let v1 = hi(1).min(f64::from(lens.height));
    (u1 > u0 && v1 > v0).then_some([u0, v0, u1, v1])
}

/// The overlap of two boxes over their union, 0..1.
#[must_use]
pub fn iou(a: &PixelBox, b: &PixelBox) -> f64 {
    let w = a[2].min(b[2]) - a[0].max(b[0]);
    let h = a[3].min(b[3]) - a[1].max(b[1]);
    if w <= 0.0 || h <= 0.0 {
        return 0.0;
    }
    let inter = w * h;
    let union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
    if union > 0.0 {
        inter / union
    } else {
        0.0
    }
}

/// How much two boxes overlap left to right, over the narrower's width, 0..1.
#[must_use]
pub fn side_overlap(a: &PixelBox, b: &PixelBox) -> f64 {
    let w = a[2].min(b[2]) - a[0].max(b[0]);
    let narrower = (a[2] - a[0]).min(b[2] - b[0]);
    if w > 0.0 && narrower > 0.0 {
        w / narrower
    } else {
        0.0
    }
}

/// One detector box: its label, score and pixels.
#[derive(Clone, Debug, PartialEq)]
pub struct Seen {
    /// The class: `person`, `stop sign`...
    pub label: String,
    /// 0..1.
    pub score: f64,
    /// Where in the picture.
    pub area: PixelBox,
}

/// The label of the detector box overlapping `area` most, if any matches.
///
/// A match has an IoU of at least `min_iou`; with `min_side` (a thing whose top the lidar
/// can't see), also a box overlapping it top to bottom and by `min_side` left to right.
/// The highest IoU wins; the first of equals.
#[must_use]
pub fn best_label<'a>(
    area: &PixelBox,
    seen: &'a [Seen],
    min_iou: f64,
    min_side: Option<f64>,
) -> Option<&'a str> {
    let mut best: Option<&str> = None;
    let mut best_score = -1.0;
    for s in seen {
        let overlap = iou(area, &s.area);
        let tall = min_side.is_some_and(|side| {
            side_overlap(area, &s.area) >= side && area[3].min(s.area[3]) > area[1].max(s.area[1])
        });
        if (overlap >= min_iou || tall) && overlap > best_score {
            best = Some(&s.label);
            best_score = overlap;
        }
    }
    best
}

/// One camera: its lens and mount, and its newest boxes with their time (s).
#[derive(Clone, Debug, PartialEq)]
pub struct Camera {
    /// Its intrinsics.
    pub lens: Lens,
    /// Where it is.
    pub mount: CameraMount,
    /// Its newest boxes.
    pub seen: Vec<Seen>,
    /// s: when they came.
    pub stamp: f64,
}

/// Names things from every camera's newest boxes; a name is held `hold` s unseen.
#[derive(Clone, Debug, PartialEq)]
pub struct Namer {
    /// The least IoU that matches.
    pub min_iou: f64,
    /// An open-topped thing's least overlap left to right.
    pub min_side: f64,
    /// m: the lidar's cubes stop here (`aav_voxels`' max_height).
    pub ceiling: f64,
    /// s: a name kept unseen.
    pub hold: f64,
    /// s: older boxes don't count.
    pub max_age: f64,
    /// Each camera by its frame, in the order they first came.
    pub cameras: Vec<(String, Camera)>,
    named: HashMap<u32, (String, f64)>,
}

impl Default for Namer {
    fn default() -> Self {
        Self {
            min_iou: 0.3,
            min_side: 0.5,
            ceiling: 2.0,
            hold: 3.0,
            max_age: 0.5,
            cameras: Vec::new(),
            named: HashMap::new(),
        }
    }
}

impl Namer {
    /// Keep a camera's newest boxes (in its place, or after the others if new).
    pub fn set_camera(&mut self, frame: &str, camera: Camera) {
        match self.cameras.iter_mut().find(|(f, _)| f == frame) {
            Some((_, slot)) => *slot = camera,
            None => self.cameras.push((frame.to_string(), camera)),
        }
    }

    /// The name of thing `id`, standing at (x, y) in base_link, `now` (s).
    #[allow(clippy::too_many_arguments)]
    pub fn name(
        &mut self,
        id: u32,
        x: f64,
        y: f64,
        length: f64,
        width: f64,
        height: f64,
        now: f64,
    ) -> String {
        let open_top = height >= self.ceiling;
        for (_, camera) in &self.cameras {
            if now - camera.stamp > self.max_age {
                continue;
            }
            let label = thing_box(&camera.lens, &camera.mount, x, y, length, width, height)
                .and_then(|area| {
                    best_label(
                        &area,
                        &camera.seen,
                        self.min_iou,
                        open_top.then_some(self.min_side),
                    )
                });
            if let Some(label) = label {
                self.named.insert(id, (label.to_string(), now));
                return label.to_string();
            }
        }
        match self.named.get(&id) {
            Some((name, when)) if now - when <= self.hold => name.clone(),
            _ => {
                self.named.remove(&id);
                UNKNOWN.to_string()
            }
        }
    }
}

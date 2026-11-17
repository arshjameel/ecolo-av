//! The lidar's cubes as things with an id, a speed and a memory. No ROS here.
//!
//! A port of `aav_things`' Python modules (TODO 185; the design:
//! `docs/designs/2026-11-14_the-whole-car.md`, "Seeing: things").
//!
//! ```text
//! cubes (base_link) ─► background: drop the saved map's walls ─► groups ─► to odom
//!                                                                   │
//!   radar clusters ─► a speed along the ray ─────────────────► tracker ─► things
//!                                                                   ▲
//!                                    the cameras' boxes ─► names ───┘
//! ```
//!
//! | Module | Holds | From |
//! |---|---|---|
//! | [`groups`] | [`Group`], [`GroupShape`], [`group`], [`to_frame`] | `groups.py` |
//! | [`tracker`] | [`Tracker`], [`Track`], [`TrackerShape`], [`RadarCluster`], [`predict`], [`ground_radial_speed`] | `tracker.py` |
//! | [`radar`] | [`radar::clusters`]: a cluster cloud's rows to [`RadarCluster`]s | `things_node.py` `use_clusters` |
//! | [`background`] | [`Background`], [`ChunkBackground`], [`new_only`] | `background.py` |
//! | [`tf_wait`] | [`Waiting`], [`Step`], [`decide`] | `tf_wait.py` |
//! | [`naming`] | [`Namer`], [`Lens`], [`CameraMount`], [`thing_box`], [`iou`] | `naming.py`, `aav_detector`'s `ground.py` (the pinhole) |
//! | [`looks`] | [`looks::colour`], [`looks::caption`] | `looks.py` |
//! | [`cloud`] | named columns of a `PointCloud2`'s bytes, any number type | new |
//! | [`pairwise_sum`] | numpy's sum, so a mean has numpy's last bit | new |
//!
//! The same answers as the Python, checked on recorded and made-up runs
//! (`tests/vectors.rs`, written by `gen_vectors.py`).
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod background;
pub mod cloud;
pub mod groups;
pub mod looks;
pub mod naming;
pub mod radar;
pub mod tf_wait;
pub mod tracker;

pub use background::{new_only, Background, ChunkBackground};
pub use groups::{group, to_frame, Group, GroupShape};
pub use naming::{iou, thing_box, CameraMount, Lens, Namer};
pub use tf_wait::{decide, Step, WaitShape, Waiting};
pub use tracker::{ground_radial_speed, predict, RadarCluster, Track, Tracker, TrackerShape};

/// A thing no camera has named.
pub const UNKNOWN: &str = "unknown";

/// A pose in a plane: where a frame is in another, and which way it faces (m, rad).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left (counter-clockwise).
    pub yaw: f64,
}

/// The yaw of a quaternion (x, y, z, w), as the Python's `yaw_of`.
#[must_use]
pub fn yaw_of(x: f64, y: f64, z: f64, w: f64) -> f64 {
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// The sum of `values` as numpy's `sum` (and so `mean`) adds float64s: pairwise, eight
/// running sums in blocks of up to 128, so a mean has numpy's last bit.
#[must_use]
pub fn pairwise_sum(values: &[f64]) -> f64 {
    let n = values.len();
    if n < 8 {
        // numpy starts from -0.0, so a sum of nothing, or of -0.0s, stays -0.0.
        return values.iter().fold(-0.0, |sum, v| sum + v);
    }
    if n <= 128 {
        let mut r = [0.0; 8];
        r.copy_from_slice(&values[..8]);
        let mut i = 8;
        while i < n - n % 8 {
            for (k, slot) in r.iter_mut().enumerate() {
                *slot += values[i + k];
            }
            i += 8;
        }
        let mut sum = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for v in &values[i..] {
            sum += v;
        }
        return sum;
    }
    let mut half = n / 2;
    half -= half % 8;
    pairwise_sum(&values[..half]) + pairwise_sum(&values[half..])
}

/// numpy's mean: the pairwise sum over the count.
#[must_use]
pub fn mean(values: &[f64]) -> f64 {
    pairwise_sum(values) / values.len() as f64
}

#[cfg(test)]
mod tests;

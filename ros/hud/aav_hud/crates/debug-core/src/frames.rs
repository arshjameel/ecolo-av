//! Frames: stamps, 3D transforms, and the static tree from `/tf_static`.
//!
//! rclrs has no tf2 buffer, but `tf2_msgs/TFMessage` is plain: the node copies each
//! transform in here. The debug HUD needs two: the static `base_link -> lidar_link` (a
//! chain through other static frames is followed), and the latest `map -> odom`
//! ([`MapToOdom`]), so map-frame layers sit on the cubes.

use std::collections::HashMap;

use map_core::geom::{Point, Pose, Quaternion};
use thiserror::Error;

/// A message's stamp, nanoseconds. Stamps are only ever compared with stamps (Gazebo's
/// are sim time; the window runs on wall time).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Stamp(pub i64);

impl Stamp {
    /// From `builtin_interfaces/Time`.
    pub fn new(sec: i32, nanosec: u32) -> Self {
        Self(i64::from(sec) * 1_000_000_000 + i64::from(nanosec))
    }

    /// Seconds from `earlier` to `self` (negative if `self` is earlier).
    pub fn seconds_since(self, earlier: Stamp) -> f64 {
        (self.0 - earlier.0) as f64 * 1e-9
    }
}

/// A point or a vector in 3D, metres.
pub type Vec3 = [f64; 3];

/// A rigid transform: `p' = rotation * p + translation`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3 {
    /// Row-major.
    pub rotation: [[f64; 3]; 3],
    pub translation: Vec3,
}

impl Transform3 {
    pub const IDENTITY: Transform3 = Transform3 {
        rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation: [0.0, 0.0, 0.0],
    };

    /// From a ROS transform (or pose): a translation and a quaternion. A zero quaternion
    /// (an unset message) is no rotation; any other is normalised.
    pub fn from_ros(translation: Vec3, q: Quaternion) -> Self {
        let n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        if !(n.is_finite() && n > 1e-12) {
            return Self {
                translation,
                ..Self::IDENTITY
            };
        }
        let (x, y, z, w) = (q.x / n, q.y / n, q.z / n, q.w / n);
        let rotation = [
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
        ];
        Self {
            rotation,
            translation,
        }
    }

    /// A pose on the ground (a 2D EKF's): x, y and the heading about z; z = 0, no roll
    /// or pitch.
    pub fn from_planar(pose: Pose) -> Self {
        let (s, c) = pose.yaw.sin_cos();
        Self {
            rotation: [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            translation: [pose.position.x, pose.position.y, 0.0],
        }
    }

    /// Seen from above: x, y and the heading. Roll, pitch and z are dropped.
    pub fn planar(&self) -> Pose {
        let r = &self.rotation;
        Pose::new(
            self.translation[0],
            self.translation[1],
            r[1][0].atan2(r[0][0]),
        )
    }

    /// `p` moved by this transform.
    pub fn apply(&self, p: Vec3) -> Vec3 {
        let r = &self.rotation;
        let t = &self.translation;
        [0, 1, 2].map(|i| r[i][0] * p[0] + r[i][1] * p[1] + r[i][2] * p[2] + t[i])
    }

    /// `self * other`: first `other`, then `self`.
    pub fn then_after(&self, other: &Transform3) -> Transform3 {
        let a = &self.rotation;
        let b = &other.rotation;
        let rotation =
            [0, 1, 2].map(|i| [0, 1, 2].map(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()));
        Transform3 {
            rotation,
            translation: self.apply(other.translation),
        }
    }

    /// The inverse: `p = R^T (p' - t)`.
    pub fn inverse(&self) -> Transform3 {
        let r = &self.rotation;
        let rotation = [0, 1, 2].map(|i| [0, 1, 2].map(|j| r[j][i]));
        let t = self.translation;
        let back = [0, 1, 2].map(|i| -(0..3).map(|k| rotation[i][k] * t[k]).sum::<f64>());
        Transform3 {
            rotation,
            translation: back,
        }
    }
}

/// A frame's name as tf2 compares them: without a leading `/`.
pub fn frame_name(name: &str) -> &str {
    name.trim_start_matches('/')
}

/// Why a frame can't be placed.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum FrameError {
    #[error("no transform from {from} to {to} on /tf_static yet")]
    NoChain { from: String, to: String },
}

/// The static frames: each child's parent and its transform (parent from child).
#[derive(Debug, Clone, Default)]
pub struct StaticFrames {
    parents: HashMap<String, (String, Transform3)>,
}

impl StaticFrames {
    /// One `/tf_static` transform: `child`'s pose in `parent`.
    pub fn insert(&mut self, parent: &str, child: &str, transform: Transform3) {
        self.parents.insert(
            frame_name(child).to_owned(),
            (frame_name(parent).to_owned(), transform),
        );
    }

    /// How many frames are known.
    pub fn len(&self) -> usize {
        self.parents.len()
    }

    pub fn is_empty(&self) -> bool {
        self.parents.is_empty()
    }

    /// The transform that takes points in `source` into `target`, when `target` is
    /// `source` or one of its ancestors (base_link above lidar_link).
    pub fn lookup(&self, target: &str, source: &str) -> Result<Transform3, FrameError> {
        let target = frame_name(target);
        let mut frame = frame_name(source);
        let mut total = Transform3::IDENTITY;
        // A loop in a broken tree can't hang us: at most one step per known frame.
        for _ in 0..=self.parents.len() {
            if frame == target {
                return Ok(total);
            }
            let Some((parent, t)) = self.parents.get(frame) else {
                break;
            };
            total = t.then_after(&total);
            frame = parent;
        }
        Err(FrameError::NoChain {
            from: source.to_owned(),
            to: target.to_owned(),
        })
    }
}

/// The latest `map -> odom`: where odom sits on the map.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MapToOdom {
    /// odom's pose in map; `None` until `/tf` has sent one.
    pub odom_in_map: Option<Pose>,
}

impl MapToOdom {
    /// A point on the map, in odom. Without `map -> odom` yet: `None`.
    pub fn to_odom(&self, p: Point) -> Option<Point> {
        let o = self.odom_in_map?;
        Some((p - o.position).rotated(-o.yaw))
    }

    /// A pose on the map, in odom.
    pub fn pose_to_odom(&self, p: Pose) -> Option<Pose> {
        let o = self.odom_in_map?;
        let at = self.to_odom(p.position)?;
        Some(Pose {
            position: at,
            yaw: p.yaw - o.yaw,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn close(a: Vec3, b: Vec3) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    fn about_z(angle: f64) -> Quaternion {
        let (s, c) = (angle / 2.0).sin_cos();
        Quaternion {
            x: 0.0,
            y: 0.0,
            z: s,
            w: c,
        }
    }

    #[test]
    fn stamps_count_seconds() {
        let a = Stamp::new(10, 500_000_000);
        let b = Stamp::new(11, 0);
        assert!((b.seconds_since(a) - 0.5).abs() < 1e-12);
        assert!(a < b);
    }

    #[test]
    fn a_quarter_turn_about_z() {
        let t = Transform3::from_ros([1.0, 0.0, 2.0], about_z(FRAC_PI_2));
        // One metre forward in the child is one metre +y in the parent.
        assert!(close(t.apply([1.0, 0.0, 0.0]), [1.0, 1.0, 2.0]));
        assert!((t.planar().yaw - FRAC_PI_2).abs() < 1e-12);
        let back = t.inverse().apply(t.apply([0.3, -0.7, 0.2]));
        assert!(close(back, [0.3, -0.7, 0.2]));
    }

    #[test]
    fn an_unset_quaternion_is_no_turn() {
        let zero = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        let t = Transform3::from_ros([0.0, 0.0, 1.0], zero);
        assert!(close(t.apply([1.0, 2.0, 3.0]), [1.0, 2.0, 4.0]));
    }

    #[test]
    fn planar_poses_agree_with_map_core() {
        let pose = Pose::new(10.0, 5.0, FRAC_PI_2);
        let t = Transform3::from_planar(pose);
        let p = pose.transform(Point::new(1.0, 2.0));
        assert!(close(t.apply([1.0, 2.0, 0.5]), [p.x, p.y, 0.5]));
    }

    #[test]
    fn a_static_chain_is_followed_up_to_base_link() {
        let mut frames = StaticFrames::default();
        // base_link -> roof (1.5 m up), roof -> lidar_link (0.2 m forward, turned left).
        frames.insert(
            "base_link",
            "roof",
            Transform3::from_ros([0.0, 0.0, 1.5], about_z(0.0)),
        );
        frames.insert(
            "/roof",
            "lidar_link",
            Transform3::from_ros([0.2, 0.0, 0.0], about_z(FRAC_PI_2)),
        );
        let t = frames.lookup("base_link", "lidar_link").expect("a chain");
        // The lidar's +x is base_link's +y.
        assert!(close(t.apply([1.0, 0.0, 0.0]), [0.2, 1.0, 1.5]));
        assert_eq!(
            frames.lookup("base_link", "base_link"),
            Ok(Transform3::IDENTITY)
        );
        assert!(frames.lookup("base_link", "camera").is_err());
        assert!(frames.lookup("lidar_link", "base_link").is_err());
    }

    #[test]
    fn a_loop_in_the_tree_ends() {
        let mut frames = StaticFrames::default();
        frames.insert("a", "b", Transform3::IDENTITY);
        frames.insert("b", "a", Transform3::IDENTITY);
        assert!(frames.lookup("base_link", "a").is_err());
    }

    #[test]
    fn map_points_move_into_odom() {
        let m = MapToOdom {
            odom_in_map: Some(Pose::new(10.0, 0.0, FRAC_PI_2)),
        };
        // odom's origin is map (10, 0), its x axis map's north.
        let p = m.to_odom(Point::new(10.0, 3.0)).expect("known");
        assert!((p.x - 3.0).abs() < 1e-9 && p.y.abs() < 1e-9);
        let pose = m
            .pose_to_odom(Pose::new(10.0, 0.0, FRAC_PI_2))
            .expect("known");
        assert!(pose.yaw.abs() < 1e-12);
        assert_eq!(MapToOdom::default().to_odom(Point::new(0.0, 0.0)), None);
    }
}

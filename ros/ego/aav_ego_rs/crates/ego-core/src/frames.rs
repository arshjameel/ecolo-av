//! A small TF tree for look_ahead: the newest transform of each frame from its parent,
//! from `/tf` and `/tf_static`, and a lookup between any two frames of one tree.
//!
//! tf2's buffer keeps a history and looks up the newest time the whole chain shares; this
//! keeps only the newest of each link, which is what look_ahead asks for (the newest TF).
//!
//! ```text
//! map ──▶ odom ──▶ base_link ──▶ sensors        lookup(target, source): source's points
//!  └─ the root     (each link: its parent's)      in target, through the common root
//! ```

use std::collections::HashMap;

/// A rigid transform: a translation (m) and a unit quaternion (x, y, z, w).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

impl Default for Transform {
    fn default() -> Self {
        Transform::IDENTITY
    }
}

fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

impl Transform {
    pub const IDENTITY: Transform = Transform {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    /// Rotate a vector by this transform's rotation.
    pub fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let q = self.rotation;
        let p = quat_mul(
            quat_mul(q, [v[0], v[1], v[2], 0.0]),
            [-q[0], -q[1], -q[2], q[3]],
        );
        [p[0], p[1], p[2]]
    }

    /// `self ∘ other`: first `other`, then `self` (a point in other's source, into self's
    /// target).
    pub fn then(&self, other: &Transform) -> Transform {
        let r = self.rotate(other.translation);
        Transform {
            translation: [
                self.translation[0] + r[0],
                self.translation[1] + r[1],
                self.translation[2] + r[2],
            ],
            rotation: quat_mul(self.rotation, other.rotation),
        }
    }

    pub fn inverse(&self) -> Transform {
        let q = self.rotation;
        let conj = Transform {
            translation: [0.0; 3],
            rotation: [-q[0], -q[1], -q[2], q[3]],
        };
        let t = conj.rotate(self.translation);
        Transform {
            translation: [-t[0], -t[1], -t[2]],
            rotation: conj.rotation,
        }
    }

    /// The yaw of the rotation (rad).
    pub fn yaw(&self) -> f64 {
        let [x, y, z, w] = self.rotation;
        crate::look_ahead::yaw_of(x, y, z, w)
    }
}

/// The newest transform of each frame from its parent.
#[derive(Debug, Clone, Default)]
pub struct Tree {
    /// child -> (parent, parent ← child).
    links: HashMap<String, (String, Transform)>,
}

/// The deepest a chain may go: more means a loop.
const MAX_DEPTH: usize = 64;

fn clean(frame: &str) -> &str {
    frame.trim_start_matches('/')
}

impl Tree {
    pub fn new() -> Self {
        Tree::default()
    }

    /// `child`'s pose in `parent` (tf2_msgs' header.frame_id and child_frame_id).
    pub fn set(&mut self, parent: &str, child: &str, transform: Transform) {
        self.links.insert(
            clean(child).to_string(),
            (clean(parent).to_string(), transform),
        );
    }

    /// root ← frame, and the root's name.
    fn to_root(&self, frame: &str) -> Option<(String, Transform)> {
        let mut name = frame.to_string();
        let mut total = Transform::IDENTITY;
        for _ in 0..MAX_DEPTH {
            match self.links.get(&name) {
                None => return Some((name, total)),
                Some((parent, t)) => {
                    total = t.then(&total);
                    name = parent.clone();
                }
            }
        }
        None
    }

    /// target ← source: a point in `source` into `target`. `None` when they share no tree.
    pub fn lookup(&self, target: &str, source: &str) -> Option<Transform> {
        let (target, source) = (clean(target), clean(source));
        if target == source {
            return Some(Transform::IDENTITY);
        }
        let (root_s, root_from_source) = self.to_root(source)?;
        let (root_t, root_from_target) = self.to_root(target)?;
        if root_s != root_t {
            return None;
        }
        Some(root_from_target.inverse().then(&root_from_source))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planar(x: f64, y: f64, yaw: f64) -> Transform {
        Transform {
            translation: [x, y, 0.0],
            rotation: [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()],
        }
    }

    #[test]
    fn a_chain_composes_and_inverts() {
        let mut tree = Tree::new();
        tree.set(
            "map",
            "odom",
            planar(10.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        tree.set("/odom", "base_link", planar(1.0, 0.0, 0.0));
        tree.set("base_link", "lidar", planar(0.5, 0.0, 0.0));
        let car = tree.lookup("map", "base_link").unwrap();
        assert!((car.translation[0] - 10.0).abs() < 1e-12);
        assert!((car.translation[1] - 1.0).abs() < 1e-12);
        assert!((car.yaw() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        // And back: the map's origin seen from the car.
        let back = tree.lookup("base_link", "map").unwrap();
        let p = back.then(&car);
        assert!(p.translation.iter().all(|v| v.abs() < 1e-12));
        let lidar_in_odom = tree.lookup("odom", "lidar").unwrap();
        assert!((lidar_in_odom.translation[0] - 1.5).abs() < 1e-12);
        assert!(tree.lookup("map", "nowhere").is_none());
        assert_eq!(tree.lookup("map", "map"), Some(Transform::IDENTITY));
    }
}

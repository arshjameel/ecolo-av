//! A TF buffer with history, the part of tf2's the behaviour node uses: each link's
//! transforms over the last `cache` seconds, looked up at a time (interpolated, as tf2's
//! `TimeCache`) or at the newest time the whole chain shares (tf2's `Time()`).
//!
//! ```text
//! odom ──▶ base_link ──▶ camera        lookup("odom", "camera", at): the camera's points
//!  (dynamic: a history)   (static)      in odom, each link interpolated at `at`
//! ```
//!
//! The sign's point is kept in odom at the picture's stamp: a GPS jump moves map, and
//! mustn't move the sign.

use std::collections::{HashMap, VecDeque};
use std::fmt;

use ego_core::frames::Transform;

/// ns: tf2's default cache, 10 s.
pub const CACHE: i64 = 10_000_000_000;
/// The deepest a chain may go: more means a loop.
const MAX_DEPTH: usize = 64;

/// A lookup that can't be answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LookupError {
    /// The frames share no tree (or one is unknown).
    Unconnected {
        /// The target frame.
        target: String,
        /// The source frame.
        source: String,
    },
    /// A link has no transform at that time (before its oldest or after its newest).
    Extrapolation {
        /// The link's child frame.
        frame: String,
        /// ns: the time asked.
        at: i64,
    },
}

impl fmt::Display for LookupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LookupError::Unconnected { target, source } => {
                write!(f, "{source} and {target} are not in one tree")
            }
            LookupError::Extrapolation { frame, at } => write!(
                f,
                "lookup would require extrapolation: {frame} has no transform at {:.3} s",
                *at as f64 * 1e-9
            ),
        }
    }
}

impl std::error::Error for LookupError {}

#[derive(Debug, Clone)]
struct Link {
    parent: String,
    /// Static (`/tf_static`): one transform for every time.
    fixed: bool,
    /// (ns, parent ← child), oldest first.
    history: VecDeque<(i64, Transform)>,
}

fn clean(frame: &str) -> &str {
    frame.trim_start_matches('/')
}

/// Spherical interpolation, as tf2's `Quaternion::slerp`.
fn slerp(a: [f64; 4], b: [f64; 4], t: f64) -> [f64; 4] {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let len2 = |q: [f64; 4]| q.iter().map(|v| v * v).sum::<f64>();
    let product = dot / (len2(a) * len2(b)).sqrt();
    if product.abs() >= 1.0 {
        return a;
    }
    // The long-angle case: go the short way round.
    let sign = if product < 0.0 { -1.0 } else { 1.0 };
    let theta = (sign * product).acos();
    let s1 = (sign * t * theta).sin();
    let d = 1.0 / theta.sin();
    let s0 = ((1.0 - t) * theta).sin();
    [
        (a[0] * s0 + b[0] * s1) * d,
        (a[1] * s0 + b[1] * s1) * d,
        (a[2] * s0 + b[2] * s1) * d,
        (a[3] * s0 + b[3] * s1) * d,
    ]
}

fn interpolate(one: &(i64, Transform), two: &(i64, Transform), at: i64) -> Transform {
    if one.0 == two.0 {
        return two.1;
    }
    let ratio = (at - one.0) as f64 / (two.0 - one.0) as f64;
    let (p, q) = (one.1.translation, two.1.translation);
    Transform {
        translation: [
            p[0] + ratio * (q[0] - p[0]),
            p[1] + ratio * (q[1] - p[1]),
            p[2] + ratio * (q[2] - p[2]),
        ],
        rotation: slerp(one.1.rotation, two.1.rotation, ratio),
    }
}

impl Link {
    fn newest(&self) -> Option<i64> {
        if self.fixed {
            None
        } else {
            self.history.back().map(|h| h.0)
        }
    }

    /// parent ← child at `at` (ns; `None`: the newest).
    fn at(&self, at: Option<i64>, child: &str) -> Result<Transform, LookupError> {
        let gap = || LookupError::Extrapolation {
            frame: child.to_string(),
            at: at.unwrap_or(0),
        };
        if self.fixed {
            return self.history.back().map(|h| h.1).ok_or_else(gap);
        }
        let Some(at) = at else {
            return self.history.back().map(|h| h.1).ok_or_else(gap);
        };
        // The first record at or after `at`.
        let after = self.history.partition_point(|h| h.0 < at);
        match (
            after.checked_sub(1).map(|i| &self.history[i]),
            self.history.get(after),
        ) {
            (_, Some(two)) if two.0 == at => Ok(two.1),
            (Some(one), Some(two)) => Ok(interpolate(one, two, at)),
            _ => Err(gap()),
        }
    }
}

/// Each frame's link to its parent, with its history.
#[derive(Debug, Clone, Default)]
pub struct Buffer {
    links: HashMap<String, Link>,
}

impl Buffer {
    /// No frames yet.
    #[must_use]
    pub fn new() -> Self {
        Buffer::default()
    }

    /// `child`'s pose in `parent` at `stamp` (ns); `fixed` for `/tf_static`. A history
    /// older than [`CACHE`] behind the newest is dropped.
    pub fn set(
        &mut self,
        parent: &str,
        child: &str,
        stamp: i64,
        transform: Transform,
        fixed: bool,
    ) {
        let link = self
            .links
            .entry(clean(child).to_string())
            .or_insert_with(|| Link {
                parent: String::new(),
                fixed,
                history: VecDeque::new(),
            });
        link.parent = clean(parent).to_string();
        link.fixed = fixed;
        if fixed {
            link.history.clear();
            link.history.push_back((stamp, transform));
            return;
        }
        // Kept in time order; the same stamp again replaces it.
        let at = link.history.partition_point(|h| h.0 < stamp);
        if link.history.get(at).is_some_and(|h| h.0 == stamp) {
            link.history[at] = (stamp, transform);
        } else {
            link.history.insert(at, (stamp, transform));
        }
        let newest = link.history.back().map_or(stamp, |h| h.0);
        while link.history.front().is_some_and(|h| h.0 < newest - CACHE) {
            link.history.pop_front();
        }
    }

    /// The chain from `frame` up to its root: (child, link) pairs.
    fn chain(&self, frame: &str) -> Option<(String, Vec<(&str, &Link)>)> {
        let mut name = frame;
        let mut out = Vec::new();
        for _ in 0..MAX_DEPTH {
            match self.links.get_key_value(name) {
                None => return Some((name.to_string(), out)),
                Some((child, link)) => {
                    out.push((child.as_str(), link));
                    name = &link.parent;
                }
            }
        }
        None
    }

    /// target ← source at `at` (ns), or at the newest time the whole chain shares
    /// (`None`, tf2's `Time()`): a point in `source` into `target`.
    ///
    /// # Errors
    ///
    /// [`LookupError`] when the frames share no tree, or a link has no transform then.
    pub fn lookup(
        &self,
        target: &str,
        source: &str,
        at: Option<i64>,
    ) -> Result<Transform, LookupError> {
        let (target, source) = (clean(target), clean(source));
        let unconnected = || LookupError::Unconnected {
            target: target.to_string(),
            source: source.to_string(),
        };
        if target == source {
            return Ok(Transform::IDENTITY);
        }
        let (root_s, up_s) = self.chain(source).ok_or_else(unconnected)?;
        let (root_t, up_t) = self.chain(target).ok_or_else(unconnected)?;
        if root_s != root_t {
            return Err(unconnected());
        }
        // Time(): the newest time every moving link has.
        let at = at.or_else(|| {
            up_s.iter()
                .chain(&up_t)
                .filter_map(|(_, link)| link.newest())
                .min()
        });
        let to_root = |up: &[(&str, &Link)]| -> Result<Transform, LookupError> {
            let mut total = Transform::IDENTITY;
            for (child, link) in up {
                total = link.at(at, child)?.then(&total);
            }
            Ok(total)
        };
        Ok(to_root(&up_t)?.inverse().then(&to_root(&up_s)?))
    }

    /// ns: the newest time every moving link up from `target` and `source` has (the
    /// stamp tf2's `lookup_transform(target, source, Time())` answers with); `None` when
    /// every link is static (any time will do).
    ///
    /// # Errors
    ///
    /// [`LookupError::Unconnected`] when the frames share no tree.
    pub fn newest(&self, target: &str, source: &str) -> Result<Option<i64>, LookupError> {
        let (target, source) = (clean(target), clean(source));
        let unconnected = || LookupError::Unconnected {
            target: target.to_string(),
            source: source.to_string(),
        };
        let (root_s, up_s) = self.chain(source).ok_or_else(unconnected)?;
        let (root_t, up_t) = self.chain(target).ok_or_else(unconnected)?;
        if root_s != root_t {
            return Err(unconnected());
        }
        Ok(up_s
            .iter()
            .chain(&up_t)
            .filter_map(|(_, link)| link.newest())
            .min())
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

    const S: i64 = 1_000_000_000;

    #[test]
    fn a_moving_link_is_interpolated_and_a_static_one_holds_for_all_time() {
        let mut tf = Buffer::new();
        tf.set("odom", "base_link", 10 * S, planar(0.0, 0.0, 0.0), false);
        tf.set("odom", "base_link", 11 * S, planar(2.0, 0.0, 1.0), false);
        tf.set("base_link", "camera", 0, planar(1.0, 0.0, 0.0), true);
        let mid = tf.lookup("odom", "camera", Some(10 * S + S / 2)).unwrap();
        // Half way: x 1.0, yaw 0.5; the camera 1 m ahead along that yaw.
        assert!((mid.translation[0] - (1.0 + 0.5_f64.cos())).abs() < 1e-12);
        assert!((mid.yaw() - 0.5).abs() < 1e-12);
        // Time(): the newest.
        let now = tf.lookup("odom", "camera", None).unwrap();
        assert_eq!(tf.newest("odom", "camera"), Ok(Some(11 * S)));
        let mut fixed = Buffer::new();
        fixed.set("base_link", "camera", 0, planar(1.0, 0.0, 0.0), true);
        assert_eq!(fixed.newest("base_link", "camera"), Ok(None));
        assert!(tf.newest("map", "camera").is_err());
        assert!((now.translation[0] - (2.0 + 1.0_f64.cos())).abs() < 1e-12);
        // Before the oldest, after the newest: no answer.
        assert!(matches!(
            tf.lookup("odom", "camera", Some(9 * S)),
            Err(LookupError::Extrapolation { .. })
        ));
        assert!(tf.lookup("odom", "camera", Some(12 * S)).is_err());
        assert!(matches!(
            tf.lookup("map", "camera", None),
            Err(LookupError::Unconnected { .. })
        ));
        // And back, inverted.
        let back = tf.lookup("camera", "odom", Some(11 * S)).unwrap();
        let there = tf.lookup("odom", "camera", Some(11 * S)).unwrap();
        assert!(back
            .then(&there)
            .translation
            .iter()
            .all(|v| v.abs() < 1e-12));
    }

    #[test]
    fn the_history_keeps_ten_seconds() {
        let mut tf = Buffer::new();
        for i in 0..30 {
            tf.set(
                "odom",
                "base_link",
                i * S,
                planar(i as f64, 0.0, 0.0),
                false,
            );
        }
        assert!(tf.lookup("odom", "base_link", Some(18 * S)).is_err());
        assert!(tf.lookup("odom", "base_link", Some(20 * S)).is_ok());
    }
}

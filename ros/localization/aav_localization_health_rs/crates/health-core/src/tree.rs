//! The TF tree, as far as the health check looks at it: which frame hangs from which, and
//! each moving link's oldest and newest stamps. No transforms: the check only asks "is
//! base_link -> imu_link there?" and "how old is map -> base_link?", tf2's
//! `lookup_transform(target, source, Time())`: the newest time the chain shares
//! (`getLatestCommonTime`), then every moving link must have data at that time.
//!
//! ```text
//! map ──(ekf_global)──▶ odom ──(ekf_local)──▶ base_link ──(static)──▶ imu_link, gps_link
//! ```
//!
//! Like tf2's buffer, a moving link keeps `CACHE` s of history behind its newest stamp.

use std::collections::HashMap;

/// The deepest a chain may go: more means a loop (tf2's `MAX_GRAPH_DEPTH`).
const MAX_DEPTH: usize = 1000;
/// s of history a moving link keeps (tf2's default cache time).
pub const CACHE: f64 = 10.0;

#[derive(Debug, Clone)]
struct Link {
    parent: String,
    /// (oldest, newest) stamps (s); `None` for `/tf_static` (good for all time).
    span: Option<(f64, f64)>,
}

impl Link {
    /// tf2's latest time: 0 for a static link.
    fn latest(&self) -> f64 {
        self.span.map_or(0.0, |(_, newest)| newest)
    }

    /// Whether it has data at `at` (s; 0: the newest).
    fn covers(&self, at: f64) -> bool {
        match self.span {
            None => true,
            Some(_) if at == 0.0 => true,
            Some((oldest, newest)) => oldest <= at && at <= newest,
        }
    }
}

/// Each frame's parent, and its stamps.
#[derive(Debug, Clone, Default)]
pub struct Tree {
    links: HashMap<String, Link>,
}

/// tf2's frame names: no leading `/`.
fn clean(frame: &str) -> &str {
    frame.trim_start_matches('/')
}

impl Tree {
    /// No frames yet.
    #[must_use]
    pub fn new() -> Self {
        Tree::default()
    }

    /// `child` hangs from `parent` at `stamp` (s); `fixed` for `/tf_static`.
    pub fn set(&mut self, parent: &str, child: &str, stamp: f64, fixed: bool) {
        let link = self
            .links
            .entry(clean(child).to_string())
            .or_insert_with(|| Link {
                parent: String::new(),
                span: None,
            });
        link.parent = clean(parent).to_string();
        link.span = if fixed {
            None
        } else {
            let (oldest, newest) = link.span.map_or((stamp, stamp), |(o, n)| {
                (
                    if stamp < o { stamp } else { o },
                    if stamp > n { stamp } else { n },
                )
            });
            // History older than CACHE behind the newest is dropped.
            Some((
                if oldest < newest - CACHE {
                    newest - CACHE
                } else {
                    oldest
                },
                newest,
            ))
        };
    }

    /// tf2's `lookup_transform(target, source, Time())`'s stamp (s): the oldest of the
    /// chain's moving links' newest stamps (0 when all of it is static). `None` when tf2
    /// would refuse: no path between them, a loop, or a link with no data then.
    #[must_use]
    pub fn newest_common(&self, target: &str, source: &str) -> Option<f64> {
        let (target, source) = (clean(target), clean(source));
        if target == source {
            return Some(self.links.get(source).map_or(0.0, Link::latest));
        }
        let min = |a: f64, b: f64| if b < a { b } else { a };
        // Walk up from the source, noting each link; done if it reaches the target.
        let mut up: Vec<&Link> = Vec::new();
        let mut common = f64::INFINITY;
        let mut frame = source;
        while let Some(link) = self.links.get(frame) {
            if link.latest() != 0.0 {
                common = min(common, link.latest());
            }
            up.push(link);
            frame = &link.parent;
            if frame == target {
                return Self::finish(common, &up);
            }
            if up.len() > MAX_DEPTH {
                return None;
            }
        }
        // Up from the target, until a parent the source's walk passed (or the source).
        let mut common = f64::INFINITY;
        let mut down: Vec<&Link> = Vec::new();
        let mut frame = target;
        let meet = loop {
            let link = self.links.get(frame)?;
            if link.latest() != 0.0 {
                common = min(common, link.latest());
            }
            down.push(link);
            if let Some(i) = up.iter().position(|l| l.parent == link.parent) {
                break i;
            }
            frame = &link.parent;
            if frame == source {
                return Self::finish(common, &down);
            }
            if down.len() > MAX_DEPTH {
                return None;
            }
        };
        for link in &up[..=meet] {
            if link.latest() != 0.0 {
                common = min(common, link.latest());
            }
        }
        down.extend(&up[..=meet]);
        Self::finish(common, &down)
    }

    fn finish(common: f64, path: &[&Link]) -> Option<f64> {
        let at = if common.is_finite() { common } else { 0.0 };
        path.iter().all(|l| l.covers(at)).then_some(at)
    }

    /// Whether tf2 would give `target <- source` now.
    #[must_use]
    pub fn connected(&self, target: &str, source: &str) -> bool {
        self.newest_common(target, source).is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn static_frames_connect_and_the_tree_age_is_its_oldest_moving_link() {
        let mut tf = Tree::new();
        assert!(!tf.connected("base_link", "imu_link"));
        tf.set("base_link", "imu_link", 0.0, true);
        assert!(tf.connected("base_link", "imu_link"));
        assert!(!tf.connected("base_link", "gps_link"));
        assert_eq!(tf.newest_common("base_link", "imu_link"), Some(0.0));
        tf.set("odom", "base_link", 10.0, false);
        tf.set("odom", "/base_link", 10.5, false);
        tf.set("odom", "base_link", 10.2, false); // late: no newer
        assert_eq!(tf.newest_common("map", "base_link"), None);
        tf.set("map", "odom", 9.0, false);
        // 9.0 is before base_link's oldest (10.0): no data then, as tf2.
        assert_eq!(tf.newest_common("map", "base_link"), None);
        tf.set("map", "odom", 11.0, false);
        assert_eq!(tf.newest_common("map", "base_link"), Some(10.5));
        assert_eq!(tf.newest_common("map", "imu_link"), Some(10.5));
        assert_eq!(tf.newest_common("odom", "map"), Some(11.0));
    }

    #[test]
    fn a_loop_is_no_tree() {
        let mut tf = Tree::new();
        tf.set("a", "b", 1.0, false);
        tf.set("b", "a", 1.0, false);
        assert!(!tf.connected("a", "c"));
    }
}

//! The road ahead, the readers' half (`aav_description/road_ahead.py`), and its limit on
//! the automatic drivers' commands (`aav_ego/road_limit.py`).
//!
//! ```text
//! car ●──────── the line: a point every `spacing` m ──────────► ║ END
//! speed limit:  base … base … cap(end − s) … 0                  ║ (may not pass)
//! ```
//!
//! | Case | Allowed forward speed |
//! |---|---|
//! | a strip, a fresh pose | [`limit_at`]: falls to 0 at the strip's end |
//! | no strip yet, or a stale pose | 0 |
//!
//! Reversing is not limited. A person is never trimmed (the node decides whose command).

use crate::hypot;
use crate::stop_law::StopCurve;

/// One stop on the line: metres along it, and why.
#[derive(Debug, Clone, PartialEq)]
pub struct StopOnLine {
    /// m along the line.
    pub along: f64,
    /// Why the car stops there.
    pub reason: String,
}

/// The road ahead (`aav_interfaces/RoadAhead`), what its readers use.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Strip {
    /// m between the line's points.
    pub spacing: f64,
    /// The points' x (m).
    pub x: Vec<f64>,
    /// The points' y (m).
    pub y: Vec<f64>,
    /// m/s at each point.
    pub speed_limit: Vec<f64>,
    /// Where the car must stop.
    pub stops: Vec<StopOnLine>,
    /// m along the line: the car may not pass it.
    pub end: f64,
}

impl Strip {
    /// How many points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// No points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    fn point(&self, i: usize) -> (f64, f64) {
        (self.x[i], self.y.get(i).copied().unwrap_or(f64::NAN))
    }
}

/// (segment index, fraction along it) of the point on the path nearest (x, y).
fn foot(points: &[(f64, f64)], x: f64, y: f64) -> (usize, f64) {
    let mut best = (f64::INFINITY, 0, 0.0);
    for (i, pair) in points.windows(2).enumerate() {
        let (a, b) = (pair[0], pair[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let n = dx * dx + dy * dy;
        let t = if n == 0.0 {
            0.0
        } else {
            1.0_f64.min(0.0_f64.max(((x - a.0) * dx + (y - a.1) * dy) / n))
        };
        let d = hypot(a.0 + t * dx - x, a.1 + t * dy - y);
        if d < best.0 {
            best = (d, i, t);
        }
    }
    (best.1, best.2)
}

/// (metres along the strip's line of the point nearest (x, y), distance off it).
#[must_use]
pub fn along_of(strip: &Strip, x: f64, y: f64) -> (f64, f64) {
    match strip.len() {
        0 => (0.0, f64::INFINITY),
        1 => (0.0, hypot(strip.x[0] - x, strip.point(0).1 - y)),
        n => {
            let points: Vec<(f64, f64)> = (0..n).map(|i| strip.point(i)).collect();
            let (i, t) = foot(&points, x, y);
            let (a, b) = (points[i], points[i + 1]);
            let seg = hypot(b.0 - a.0, b.1 - a.1);
            let (fx, fy) = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
            (i as f64 * strip.spacing + t * seg, hypot(fx - x, fy - y))
        }
    }
}

/// The metres from the car at (x, y) to the strip's end: 0 or less, no further.
#[must_use]
pub fn room_at(strip: &Strip, x: f64, y: f64) -> f64 {
    if strip.is_empty() {
        return 0.0;
    }
    strip.end - along_of(strip, x, y).0
}

/// The speed the strip allows the car at (x, y): m/s, 0 with no strip or past its end.
///
/// With a `curve` (the reader's own stopping law), the limit is worked out again from the
/// room left, never above `base`; without one, the writer's limit at the nearest point.
#[must_use]
pub fn limit_at(
    strip: Option<&Strip>,
    x: f64,
    y: f64,
    curve: Option<&StopCurve>,
    base: Option<f64>,
) -> f64 {
    let Some(strip) = strip.filter(|s| !s.is_empty()) else {
        return 0.0;
    };
    let (s, _) = along_of(strip, x, y);
    let room = strip.end - s;
    if room <= 0.0 {
        return 0.0;
    }
    // Python's int() truncates toward 0; s is 0 or more here, as is spacing.
    let i = (strip.len() - 1).min((s / strip.spacing) as usize);
    let written = strip.speed_limit.get(i).copied().unwrap_or(0.0);
    let Some(curve) = curve else {
        return written;
    };
    let mut mine = curve.cap(room);
    if let Some(base) = base {
        mine = mine.min(base);
    }
    if written > 0.0 {
        written.min(mine)
    } else {
        mine
    }
}

/// The forward speed the road ahead allows now (m/s, 0 or more).
#[must_use]
pub fn allowed(
    strip: Option<&Strip>,
    pose: Option<(f64, f64)>,
    pose_age: f64,
    pose_timeout: f64,
    curve: &StopCurve,
    base: f64,
) -> f64 {
    let (Some(strip), Some((x, y))) = (strip, pose) else {
        return 0.0;
    };
    // NaN ages count as stale, as the Python's `not (age <= timeout)`.
    if pose_age
        .partial_cmp(&pose_timeout)
        .is_none_or(|o| o.is_gt())
    {
        return 0.0;
    }
    0.0_f64.max(limit_at(Some(strip), x, y, Some(curve), Some(base)))
}

/// (v, w) with v at most `limit` going forward, keeping the curvature w / v. A reverse
/// command and a stop pass unchanged.
#[must_use]
pub fn cap(cmd: (f64, f64), limit: f64) -> (f64, f64) {
    let (v, w) = cmd;
    if v <= limit || v <= 0.0 {
        return cmd;
    }
    if limit <= 0.0 || !limit.is_finite() {
        return if limit <= 0.0 { (0.0, 0.0) } else { cmd };
    }
    (limit, w * limit / v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight(n: usize, end: f64) -> Strip {
        Strip {
            spacing: 0.5,
            x: (0..n).map(|i| i as f64 * 0.5).collect(),
            y: vec![0.0; n],
            speed_limit: vec![1.0; n],
            stops: vec![],
            end,
        }
    }

    #[test]
    fn no_road_or_a_stale_pose_allows_nothing() {
        let curve = StopCurve::new(1.0, 2.0, 0.35).unwrap();
        let strip = straight(21, 10.0);
        assert_eq!(allowed(None, Some((0.0, 0.0)), 0.0, 1.0, &curve, 1.0), 0.0);
        assert_eq!(allowed(Some(&strip), None, 0.0, 1.0, &curve, 1.0), 0.0);
        assert_eq!(
            allowed(Some(&strip), Some((0.0, 0.0)), 2.0, 1.0, &curve, 1.0),
            0.0
        );
        assert_eq!(
            allowed(Some(&strip), Some((0.0, 0.0)), 0.1, 1.0, &curve, 1.0),
            1.0
        );
        assert_eq!(
            allowed(Some(&strip), Some((10.0, 0.0)), 0.1, 1.0, &curve, 1.0),
            0.0
        );
    }

    #[test]
    fn the_cap_keeps_the_curvature_and_leaves_reverse() {
        assert_eq!(cap((1.0, 0.4), 0.5), (0.5, 0.2));
        assert_eq!(cap((-1.0, 0.4), 0.5), (-1.0, 0.4));
        assert_eq!(cap((1.0, 0.4), 0.0), (0.0, 0.0));
    }
}

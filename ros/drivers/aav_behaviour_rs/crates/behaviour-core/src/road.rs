//! The road ahead, the writer's half (`aav_description/road_ahead.py`: `resample`, `build`).
//! The readers' half is ego-core's `road_ahead`.
//!
//! ```text
//!    car ●──────── the line: a point every `spacing` m ──────────► ║ END
//!    speed limit:  base … base … cap(end − s) … 0                  ║ (may not pass)
//!                  └──────── the stopping law, from the end ───────┘
//! ```
//!
//! **The end is the safety idea**: the strip always ends in a stop. If the writer dies, the
//! car runs out of strip and stops by itself.

use std::fmt;

use ego_core::stop_law::StopCurve;

use crate::machine::{Stop, StopReason};
use crate::paths::Point;
use crate::{hypot, py_max, py_min};

/// m: how much road the writer grants at a time.
pub const LENGTH: f64 = 40.0;
/// m between the line's points.
pub const SPACING: f64 = 0.5;

/// A stop's kind (`aav_interfaces/StopPoint`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopKind {
    /// Stop, then go on once the reason clears (a sign, a person).
    Wait,
    /// Stop and stay (the goal, the fence).
    Final,
}

impl StopKind {
    /// The message's number (`StopPoint.WAIT`, `StopPoint.FINAL`).
    #[must_use]
    pub fn code(self) -> u8 {
        match self {
            StopKind::Wait => 0,
            StopKind::Final => 1,
        }
    }

    /// Each reason's kind (`KIND_OF`).
    #[must_use]
    pub fn of(reason: StopReason) -> Self {
        match reason {
            StopReason::Sign | StopReason::Obstacle => StopKind::Wait,
            StopReason::Fence | StopReason::Goal => StopKind::Final,
        }
    }
}

/// One stop: metres along the line, why, and its kind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopOnLine {
    /// m along the line.
    pub along: f64,
    /// Why.
    pub reason: StopReason,
    /// Wait or final.
    pub kind: StopKind,
}

/// The road ahead. Lists are per point of the line; metres, radians, m/s.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Strip {
    /// m between points.
    pub spacing: f64,
    /// m.
    pub x: Vec<f64>,
    /// m.
    pub y: Vec<f64>,
    /// rad: the line's direction at each point.
    pub heading: Vec<f64>,
    /// m/s at each point.
    pub speed_limit: Vec<f64>,
    /// Every stop on the line, nearest first.
    pub stops: Vec<StopOnLine>,
    /// m along the line: the car may not pass it.
    pub end: f64,
    /// The machine's state.
    pub state: String,
    /// Why.
    pub reason: String,
    /// m from the line; NaN: not known.
    pub left_hard: Vec<f64>,
    /// m from the line; NaN: not known.
    pub right_hard: Vec<f64>,
    /// m from the line; NaN: not known.
    pub left_soft: Vec<f64>,
    /// m from the line; NaN: not known.
    pub right_soft: Vec<f64>,
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
}

/// A road-ahead number out of range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RoadError {
    /// The spacing must be above 0 (m).
    Spacing(f64),
    /// The base speed must be above 0 (m/s).
    Base(f64),
}

impl fmt::Display for RoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RoadError::Spacing(v) => write!(f, "spacing must be above 0 (got {v})"),
            RoadError::Base(v) => write!(f, "base must be above 0 (got {v})"),
        }
    }
}

impl std::error::Error for RoadError {}

/// (segment index, fraction along it) of the point on the path nearest (x, y).
fn foot(points: &[Point], x: f64, y: f64) -> (usize, f64) {
    let (mut best, mut best_i, mut best_t) = (f64::INFINITY, 0, 0.0);
    for (i, w) in points.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let n = dx * dx + dy * dy;
        let t = if n == 0.0 {
            0.0
        } else {
            py_min(1.0, py_max(0.0, ((x - a.0) * dx + (y - a.1) * dy) / n))
        };
        let d = hypot(a.0 + t * dx - x, a.1 + t * dy - y);
        if d < best {
            (best, best_i, best_t) = (d, i, t);
        }
    }
    (best_i, best_t)
}

/// A point of the line: (x, y, heading).
pub type Pose = (f64, f64, f64);

/// The path ahead of (x, y) as (x, y, heading), one point every `spacing` m.
///
/// The first point is the car's foot on the path; the last is the path's end or `length`
/// m along, whichever is nearer. Fewer than two path points: nothing.
///
/// # Errors
///
/// [`RoadError::Spacing`] unless `spacing` > 0.
pub fn resample(
    points: &[Point],
    x: f64,
    y: f64,
    spacing: f64,
    length: f64,
) -> Result<Vec<Pose>, RoadError> {
    if spacing <= 0.0 || spacing.is_nan() {
        return Err(RoadError::Spacing(spacing));
    }
    if points.len() < 2 {
        return Ok(Vec::new());
    }
    let (i, t) = foot(points, x, y);
    let (a, b) = (points[i], points[i + 1]);
    // The rest of the path, from the foot.
    let mut rest = vec![(a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))];
    rest.extend_from_slice(&points[i + 1..]);
    let mut out: Vec<Pose> = Vec::new();
    let mut want = 0.0; // m along: the next point to place
    let mut done = 0.0; // m along at the start of this segment
    let mut heading = (b.1 - a.1).atan2(b.0 - a.0);
    for w in rest.windows(2) {
        let (p, q) = (w[0], w[1]);
        let seg = hypot(q.0 - p.0, q.1 - p.1);
        if seg == 0.0 {
            continue;
        }
        heading = (q.1 - p.1).atan2(q.0 - p.0);
        while want <= done + seg + 1e-9 && want <= length + 1e-9 {
            let f = (want - done) / seg;
            out.push((p.0 + f * (q.0 - p.0), p.1 + f * (q.1 - p.1), heading));
            want += spacing;
        }
        done += seg;
        if want > length + 1e-9 {
            break;
        }
    }
    // Keep the path's real end when it falls between two points.
    let total = py_min(done, length);
    if !out.is_empty() && total - (out.len() - 1) as f64 * spacing > 1e-6 && done <= length + 1e-9 {
        if let Some(end) = rest.last() {
            out.push((end.0, end.1, heading));
        }
    }
    Ok(out)
}

/// The metres along the line to point i (the last may be nearer than spacing).
fn along(line: &[Pose], spacing: f64, i: usize) -> f64 {
    if i + 1 < line.len() {
        return i as f64 * spacing;
    }
    if i == 0 {
        return 0.0;
    }
    let (p, q) = (line[i - 1], line[i]);
    (i - 1) as f64 * spacing + hypot(q.0 - p.0, q.1 - p.1)
}

/// What [`build`] writes besides the line: the machine's words, the line's spacing and
/// length.
#[derive(Debug, Clone, PartialEq)]
pub struct BuildOptions<'a> {
    /// The machine's state.
    pub state: &'a str,
    /// Why.
    pub reason: &'a str,
    /// m between points.
    pub spacing: f64,
    /// m: the longest line.
    pub length: f64,
}

impl Default for BuildOptions<'_> {
    fn default() -> Self {
        BuildOptions {
            state: "",
            reason: "",
            spacing: SPACING,
            length: LENGTH,
        }
    }
}

/// The road ahead for the car at `car` on `plan`.
///
/// `stops`: the machine's stops (`left`, metres from the car to its final stop); a stop
/// behind the car sits at 0: stop now. No plan or no pose: an empty strip whose end is 0,
/// so every reader allows 0.
///
/// # Errors
///
/// [`RoadError`] for a base speed or spacing not above 0.
pub fn build(
    plan: &[Point],
    car: Option<Point>,
    stops: &[Stop],
    curve: &StopCurve,
    base: f64,
    o: &BuildOptions<'_>,
) -> Result<Strip, RoadError> {
    if base <= 0.0 {
        return Err(RoadError::Base(base));
    }
    let line = match car {
        Some((x, y)) => resample(plan, x, y, o.spacing, o.length)?,
        None => Vec::new(),
    };
    let empty = Strip {
        spacing: o.spacing,
        state: o.state.to_string(),
        reason: o.reason.to_string(),
        ..Strip::default()
    };
    if line.is_empty() {
        return Ok(empty);
    }
    let last = along(&line, o.spacing, line.len() - 1);
    let mut on_line: Vec<StopOnLine> = stops
        .iter()
        .filter(|s| s.left <= last)
        .map(|s| StopOnLine {
            along: py_max(0.0, s.left),
            reason: s.reason,
            kind: StopKind::of(s.reason),
        })
        .collect();
    // Stable, as Python's sorted().
    on_line.sort_by(|a, b| a.along.total_cmp(&b.along));
    let end = on_line.iter().map(|s| s.along).fold(last, py_min);
    let speed_limit = (0..line.len())
        .map(|i| {
            let room = end - along(&line, o.spacing, i);
            if room <= 0.0 {
                0.0
            } else {
                py_min(base, curve.cap(room))
            }
        })
        .collect();
    let nan = vec![f64::NAN; line.len()];
    Ok(Strip {
        x: line.iter().map(|p| p.0).collect(),
        y: line.iter().map(|p| p.1).collect(),
        heading: line.iter().map(|p| p.2).collect(),
        speed_limit,
        stops: on_line,
        end,
        left_hard: nan.clone(),
        right_hard: nan.clone(),
        left_soft: nan.clone(),
        right_soft: nan,
        ..empty
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn curve() -> StopCurve {
        StopCurve::from_numbers(5.0, 2.0, 2.0).unwrap()
    }

    #[test]
    fn a_straight_plan_gives_a_point_every_spacing_and_ends_at_the_nearest_stop() {
        let plan = [(0.0, 0.0), (10.0, 0.0)];
        let stops = [
            Stop::new(StopReason::Goal, 9.0),
            Stop::new(StopReason::Sign, 4.0),
            Stop::new(StopReason::Obstacle, 50.0),
        ];
        let s = build(
            &plan,
            Some((1.0, 0.2)),
            &stops,
            &curve(),
            2.0,
            &BuildOptions::default(),
        )
        .unwrap();
        assert_eq!(s.len(), 19);
        assert_eq!(s.x[0], 1.0);
        assert_eq!(s.end, 4.0);
        assert_eq!(
            s.stops.iter().map(|x| x.reason).collect::<Vec<_>>(),
            vec![StopReason::Sign, StopReason::Goal]
        );
        assert_eq!(s.stops[1].kind, StopKind::Final);
        assert_eq!(s.speed_limit[8], 0.0);
        assert!(s.speed_limit[0] > 0.0 && s.speed_limit[0] <= 2.0);
        assert!(s.left_hard.iter().all(|v| v.is_nan()));
    }

    #[test]
    fn no_plan_or_no_pose_is_an_empty_strip_ending_at_0() {
        let s = build(
            &[],
            Some((0.0, 0.0)),
            &[],
            &curve(),
            1.0,
            &BuildOptions::default(),
        )
        .unwrap();
        assert!(s.is_empty() && s.end == 0.0 && s.left_hard.is_empty());
        let s = build(
            &[(0.0, 0.0), (1.0, 0.0)],
            None,
            &[],
            &curve(),
            1.0,
            &BuildOptions::default(),
        )
        .unwrap();
        assert!(s.is_empty());
    }

    #[test]
    fn bad_numbers_are_refused() {
        assert_eq!(
            resample(&[(0.0, 0.0), (1.0, 0.0)], 0.0, 0.0, 0.0, 1.0),
            Err(RoadError::Spacing(0.0))
        );
        let e = build(&[], None, &[], &curve(), 0.0, &BuildOptions::default());
        assert_eq!(e, Err(RoadError::Base(0.0)));
    }

    #[test]
    fn the_real_end_is_kept_between_two_points() {
        let line = resample(&[(0.0, 0.0), (1.2, 0.0)], 0.0, 0.0, 0.5, 40.0).unwrap();
        let xs: Vec<f64> = line.iter().map(|p| p.0).collect();
        assert_eq!(xs, vec![0.0, 0.5, 1.0, 1.2]);
    }
}

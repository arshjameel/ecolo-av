//! The squiggly line's maths (`look_ahead.py`): the plan beside the trace, and the room
//! left at the stop point. Every point is (x, y) in `base_link`: the car at (0, 0), facing +x.
//!
//! ```text
//! the plan (in base_link) ──▶ crop_ahead ──▶ the faint line: what the planner WANTS
//! the trace + the plan    ──▶ gap        ──▶ how far the bold line strays: green, amber, red
//! the stop point + plan   ──▶ to_goal    ──▶ m from the stop point to the plan's end
//! the stop point + grid   ──▶ first_hit  ──▶ m from the stop point to the first obstacle
//! ```

use std::f64::consts::TAU;

use crate::hypot;
use crate::road_ahead::{along_of, limit_at, Strip};

/// (x, y), m.
pub type Point = (f64, f64);
/// (x, y, yaw): m, m, rad.
pub type Pose2 = (f64, f64, f64);

fn dist(a: Point, b: Point) -> f64 {
    hypot(a.0 - b.0, a.1 - b.1)
}

/// How long the polyline is (m).
pub fn length(points: &[Point]) -> f64 {
    points.windows(2).map(|p| dist(p[0], p[1])).sum()
}

/// How far along the polyline p's nearest point is, and how far p is from it:
/// `(along, off)`. A single point: (0, its distance). `None` for no points.
pub fn project(points: &[Point], p: Point) -> Option<(f64, f64)> {
    match points {
        [] => None,
        [only] => Some((0.0, dist(*only, p))),
        _ => {
            let mut best = (f64::INFINITY, 0.0); // (off, along)
            let mut walked = 0.0;
            for pair in points.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let seg = hypot(dx, dy);
                let u = if seg == 0.0 {
                    0.0
                } else {
                    0.0_f64.max(1.0_f64.min(((p.0 - a.0) * dx + (p.1 - a.1) * dy) / (seg * seg)))
                };
                let off = dist((a.0 + u * dx, a.1 + u * dy), p);
                if off < best.0 {
                    best = (off, walked + u * seg);
                }
                walked += seg;
            }
            Some((best.1, best.0))
        }
    }
}

/// The plan from the point nearest the car (the origin) to `ahead` m further on.
pub fn crop_ahead(points: &[Point], ahead: f64) -> Vec<Point> {
    if points.len() < 2 {
        return points.to_vec();
    }
    let start = project(points, (0.0, 0.0)).map_or(0.0, |(along, _)| along);
    cut(points, start, start + ahead)
}

/// The part of a polyline between `start` and `end` m along it.
pub fn cut(points: &[Point], start: f64, end: f64) -> Vec<Point> {
    let mut out = Vec::new();
    let mut walked = 0.0;
    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let seg = dist(a, b);
        let (lo, hi) = (walked, walked + seg);
        if hi >= start && lo <= end && seg > 0.0 {
            if out.is_empty() {
                out.push(along_segment(a, b, (start.max(lo) - lo) / seg));
            }
            out.push(along_segment(a, b, (end.min(hi) - lo) / seg));
        }
        walked = hi;
        if walked > end {
            break;
        }
    }
    out
}

pub fn along_segment(a: Point, b: Point, u: f64) -> Point {
    (a.0 + u * (b.0 - a.0), a.1 + u * (b.1 - a.1))
}

/// The largest distance of a trace point from the plan; `None` without both lines.
pub fn gap(trace: &[Point], plan: &[Point]) -> Option<f64> {
    if trace.is_empty() || plan.len() < 2 {
        return None;
    }
    trace
        .iter()
        .filter_map(|&p| project(plan, p).map(|(_, off)| off))
        .reduce(|a, b| if b > a { b } else { a })
}

/// The gap's grade: green, amber or red (DiagnosticStatus's OK, WARN, ERROR).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Ok,
    Warn,
    Bad,
}

impl Level {
    /// diagnostic_msgs/DiagnosticStatus's level byte.
    pub fn byte(self) -> u8 {
        match self {
            Level::Ok => 0,
            Level::Warn => 1,
            Level::Bad => 2,
        }
    }
}

/// m: amber past `warn`, red past `bad`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub warn: f64,
    pub bad: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            warn: 0.3,
            bad: 0.6,
        }
    }
}

/// OK, WARN or BAD for a gap; OK with no plan.
pub fn grade(value: Option<f64>, limits: &Limits) -> Level {
    match value {
        None => Level::Ok,
        Some(v) if v > limits.bad => Level::Bad,
        Some(v) if v > limits.warn => Level::Warn,
        Some(_) => Level::Ok,
    }
}

/// The metres from the stop point to the plan's end, along the plan: + short of the goal,
/// - past it. `None` without a plan.
pub fn to_goal(plan: &[Point], stop: Point) -> Option<f64> {
    if plan.len() < 2 {
        return None;
    }
    let (along, _) = project(plan, stop)?;
    let total = length(plan);
    if along >= total - 1e-9 {
        let ((ax, ay), (bx, by)) = (plan[plan.len() - 2], plan[plan.len() - 1]);
        let seg = hypot(bx - ax, by - ay);
        let seg = if seg == 0.0 { 1.0 } else { seg };
        return Some(-(0.0_f64.max(((stop.0 - bx) * (bx - ax) + (stop.1 - by) * (by - ay)) / seg)));
    }
    Some(total - along)
}

/// An occupancy grid (nav_msgs/OccupancyGrid's numbers), already in `base_link`.
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    pub width: usize,
    pub height: usize,
    /// m per cell.
    pub resolution: f64,
    /// x, y, yaw of cell (0, 0)'s corner.
    pub origin: Pose2,
    /// Row-major, -1 unknown, 0..100.
    pub data: Vec<i8>,
}

impl Grid {
    /// The cell's value under p, -1 outside the grid.
    pub fn cost(&self, p: Point) -> i32 {
        let (ox, oy, yaw) = self.origin;
        let (dx, dy) = (p.0 - ox, p.1 - oy);
        let (c, s) = (yaw.cos(), yaw.sin());
        let gx = (c * dx + s * dy) / self.resolution;
        let gy = (-s * dx + c * dy) / self.resolution;
        let (i, j) = (gx.floor(), gy.floor());
        if i >= 0.0 && j >= 0.0 && i < self.width as f64 && j < self.height as f64 {
            let index = j as usize * self.width + i as usize;
            return self.data.get(index).map_or(-1, |&v| i32::from(v));
        }
        -1
    }
}

/// A path of (x, y, yaw) poses, carried on straight to `total` m long.
pub fn extend(path: &[Pose2], total: f64) -> Vec<Pose2> {
    let mut out = if path.is_empty() {
        vec![(0.0, 0.0, 0.0)]
    } else {
        path.to_vec()
    };
    let xy: Vec<Point> = out.iter().map(|&(x, y, _)| (x, y)).collect();
    let have = length(&xy);
    if have < total {
        if let Some(&(x, y, yaw)) = out.last() {
            let more = total - have;
            out.push((x + more * yaw.cos(), y + more * yaw.sin(), yaw));
        }
    }
    out
}

/// IEEE remainder, as Python's `math.remainder`: x - n y, n the nearest whole (ties even).
pub fn remainder(x: f64, y: f64) -> f64 {
    x - (x / y).round_ties_even() * y
}

/// The metres `base_link` travels along `path` before the bumper meets a lethal cell.
///
/// `front` is the bumper's distance ahead of `base_link`, `half_width` half the body's.
/// The bumper is checked across its width at most `step` m apart, every `step` m along.
/// `None`: nothing in the way.
pub fn first_hit(
    grid: &Grid,
    path: &[Pose2],
    front: f64,
    half_width: f64,
    lethal: i32,
    step: f64,
) -> Option<f64> {
    let across = 1_usize.max((2.0 * half_width / step).ceil() as usize);
    let sides: Vec<f64> = (0..=across)
        .map(|k| -half_width + 2.0 * half_width * k as f64 / across as f64)
        .collect();
    let mut walked = 0.0;
    for pair in path.windows(2) {
        let ((ax, ay, ayaw), (bx, by, byaw)) = (pair[0], pair[1]);
        let seg = hypot(bx - ax, by - ay);
        let n = 1_usize.max((seg / step).ceil() as usize);
        for k in 0..=n {
            let u = k as f64 / n as f64;
            let (x, y) = (ax + u * (bx - ax), ay + u * (by - ay));
            let yaw = ayaw + u * remainder(byaw - ayaw, TAU);
            let (c, s) = (yaw.cos(), yaw.sin());
            let (fx, fy) = (x + front * c, y + front * s);
            if sides
                .iter()
                .any(|side| grid.cost((fx - side * s, fy + side * c)) >= lethal)
            {
                return Some(walked + u * seg);
            }
        }
        walked += seg;
    }
    None
}

/// One line for people, such as `ON PLAN 0.12 m - stops in 1.4 m, 0.8 m short of the goal`.
pub fn describe(
    gap_m: Option<f64>,
    level: Level,
    stop_m: Option<f64>,
    goal_m: Option<f64>,
    room_m: Option<f64>,
) -> String {
    let word = match level {
        Level::Ok => "ON PLAN",
        Level::Warn => "OFF PLAN",
        Level::Bad => "FAR OFF PLAN",
    };
    let head = match gap_m {
        None => "NO PLAN".to_string(),
        Some(g) => format!("{word} {g:.2} m"),
    };
    let mut parts = vec![match stop_m {
        None => "stop point unknown".to_string(),
        Some(s) => format!("stops in {s:.1} m"),
    }];
    if let Some(g) = goal_m {
        parts.push(format!(
            "{:.1} m {} the goal",
            g.abs(),
            if g >= 0.0 { "short of" } else { "past" }
        ));
    }
    if let Some(r) = room_m {
        parts.push(if r >= 0.0 {
            format!("{r:.1} m before an obstacle")
        } else {
            format!("HITS an obstacle ({:.1} m short)", -r)
        });
    }
    format!("{head} - {}", parts.join(", "))
}

/// What the road ahead says where the car is.
#[derive(Debug, Clone, PartialEq)]
pub struct Road {
    /// m/s: the speed it allows here (0: none).
    pub limit: f64,
    /// m left to its end (0 or less: there).
    pub room: f64,
    /// Why it ends there: the nearest stop's reason ('' for none).
    pub why: String,
}

/// The road ahead at the car (x, y, the strip's frame); `None` with no strip. The writer's
/// own limit (no curve): the screens show what the road says.
pub fn road(strip: Option<&Strip>, x: f64, y: f64) -> Option<Road> {
    let strip = strip.filter(|s| !s.is_empty())?;
    let (s, _) = along_of(strip, x, y);
    let why = strip
        .stops
        .iter()
        .filter(|stop| stop.along >= s - strip.spacing)
        .min_by(|a, b| a.along.total_cmp(&b.along))
        .map_or(String::new(), |stop| stop.reason.clone());
    Some(Road {
        limit: limit_at(Some(strip), x, y, None, None),
        room: strip.end - s,
        why,
    })
}

/// The yaw of a quaternion (x, y, z, w).
pub fn yaw_of(x: f64, y: f64, z: f64, w: f64) -> f64 {
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// A 2D transform: a point in the source frame into the target frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame2D {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    c: f64,
    s: f64,
}

impl Default for Frame2D {
    fn default() -> Self {
        Frame2D::new(0.0, 0.0, 0.0)
    }
}

impl Frame2D {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Frame2D {
            x,
            y,
            yaw,
            c: yaw.cos(),
            s: yaw.sin(),
        }
    }

    pub fn point(&self, px: f64, py: f64) -> Point {
        (
            self.x + self.c * px - self.s * py,
            self.y + self.s * px + self.c * py,
        )
    }

    pub fn pose(&self, px: f64, py: f64, pyaw: f64) -> Pose2 {
        let (x, y) = self.point(px, py);
        (x, y, pyaw + self.yaw)
    }
}

/// The index of the point nearest the origin (the first of equals); 0 for none.
pub fn nearest_index(points: &[Point]) -> usize {
    let mut best = (f64::INFINITY, 0);
    for (i, p) in points.iter().enumerate() {
        let d = hypot(p.0, p.1);
        if d < best.0 {
            best = (d, i);
        }
    }
    best.1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plan_is_cut_from_beside_the_car() {
        let plan = [(-5.0, 1.0), (20.0, 1.0)];
        let faint = crop_ahead(&plan, 10.0);
        assert_eq!(faint, vec![(0.0, 1.0), (10.0, 1.0)]);
        assert_eq!(gap(&[(0.0, 0.0), (5.0, 0.5)], &faint), Some(1.0));
        assert_eq!(grade(Some(1.0), &Limits::default()), Level::Bad);
        assert_eq!(grade(None, &Limits::default()), Level::Ok);
    }

    #[test]
    fn the_goal_is_ahead_or_behind() {
        let plan = [(0.0, 0.0), (10.0, 0.0)];
        assert_eq!(to_goal(&plan, (4.0, 0.0)), Some(6.0));
        assert_eq!(to_goal(&plan, (12.0, 0.0)), Some(-2.0));
        assert_eq!(to_goal(&plan[..1], (4.0, 0.0)), None);
    }

    #[test]
    fn the_bumper_finds_the_first_lethal_cell() {
        let mut data = vec![0_i8; 100 * 20];
        data[10 * 100 + 50] = 100; // the cell at x 5.0..5.1, y 0.0..0.1 (origin -1 m)
        let grid = Grid {
            width: 100,
            height: 20,
            resolution: 0.1,
            origin: (0.0, -1.0, 0.0),
            data,
        };
        let path = extend(&[(0.0, 0.0, 0.0)], 15.0);
        let hit = first_hit(&grid, &path, 2.0, 0.4, 100, 0.1).unwrap();
        assert!((hit - 3.0).abs() < 0.11, "{hit}");
        assert_eq!(grid.cost((-1.0, 0.0)), -1);
    }

    #[test]
    fn describe_reads_as_the_python() {
        assert_eq!(
            describe(Some(0.12), Level::Ok, Some(1.4), Some(0.8), Some(3.0)),
            "ON PLAN 0.12 m - stops in 1.4 m, 0.8 m short of the goal, 3.0 m before an obstacle"
        );
        assert_eq!(
            describe(None, Level::Ok, None, None, Some(-0.5)),
            "NO PLAN - stop point unknown, HITS an obstacle (0.5 m short)"
        );
    }
}

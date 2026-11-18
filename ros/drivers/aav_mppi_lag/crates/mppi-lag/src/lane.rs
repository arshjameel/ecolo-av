//! The lane: the road ahead's line and its four edges, for [`crate::critics::LaneCritic`].
//!
//! `/world/road_ahead` (aav_interfaces/RoadAhead) gives a line, a point every 0.5 m, and
//! for each point how far its edges lie to either side. The node moves the line into
//! odom; the edges are distances, so they move with it unchanged.
//!
//! ```text
//!   ══════════ left hard edge (the kerb): the outline never crosses it ═══════
//!   ── ── ──── left soft edge (the lane line): crossing it costs ── ── ── ──
//!   ●───────── the line (the lane's centre) ─────────────────────────────►
//!   ── ── ──── right soft edge ── ── ── ── ── ── ── ── ── ── ── ── ── ── ──
//!   ══════════ right hard edge ═══════════════════════════════════════════
//! ```

/// How far each edge lies from the line, sideways (m, 0 or more). `None`: not known (the
/// message's NaN: no lane map).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Edges {
    /// The left kerb.
    pub left_hard: Option<f64>,
    /// The right kerb.
    pub right_hard: Option<f64>,
    /// The left lane line.
    pub left_soft: Option<f64>,
    /// The right lane line.
    pub right_soft: Option<f64>,
}

/// A distance as the message gives it: NaN, infinite or negative is not known.
fn known(v: f64) -> Option<f64> {
    (v.is_finite() && v >= 0.0).then_some(v)
}

/// Between `a` and `b` at `t` (0..1); not known if either end isn't.
fn lerp(a: Option<f64>, b: Option<f64>, t: f64) -> Option<f64> {
    Some(a? + (b? - a?) * t)
}

impl Edges {
    /// The edges from the message's numbers (m): NaN, infinite or negative is not known.
    #[must_use]
    pub fn from_numbers(left_hard: f64, right_hard: f64, left_soft: f64, right_soft: f64) -> Self {
        Self {
            left_hard: known(left_hard),
            right_hard: known(right_hard),
            left_soft: known(left_soft),
            right_soft: known(right_soft),
        }
    }

    /// True if any edge is known.
    pub fn any(&self) -> bool {
        [
            self.left_hard,
            self.right_hard,
            self.left_soft,
            self.right_soft,
        ]
        .iter()
        .any(Option::is_some)
    }

    fn between(&self, other: &Self, t: f64) -> Self {
        Self {
            left_hard: lerp(self.left_hard, other.left_hard, t),
            right_hard: lerp(self.right_hard, other.right_hard, t),
            left_soft: lerp(self.left_soft, other.left_soft, t),
            right_soft: lerp(self.right_soft, other.right_soft, t),
        }
    }
}

/// One point of the line and its edges.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LanePoint {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// Its edges.
    pub edges: Edges,
}

/// Where a point lies beside the lane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Beside {
    /// The segment it is beside (from point `index` to `index + 1`).
    pub index: usize,
    /// m from the line, + left (REP-103).
    pub lateral: f64,
    /// The edges there, between the segment's two points.
    pub edges: Edges,
}

/// The lane: a line of at least two points, with at least one edge known somewhere.
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    points: Vec<LanePoint>,
}

impl Lane {
    /// The lane through `points`, or `None` if it has no line (fewer than two distinct
    /// points with numbers) or no edge is known anywhere (no lane map: nothing to keep to).
    pub fn new(points: impl IntoIterator<Item = LanePoint>) -> Option<Self> {
        let mut kept: Vec<LanePoint> = Vec::new();
        for p in points
            .into_iter()
            .filter(|p| p.x.is_finite() && p.y.is_finite())
        {
            match kept.last() {
                Some(last) if (p.x - last.x).hypot(p.y - last.y) < 1e-6 => {}
                _ => kept.push(p),
            }
        }
        (kept.len() >= 2 && kept.iter().any(|p| p.edges.any())).then_some(Self { points: kept })
    }

    /// The line's points.
    #[must_use]
    pub fn points(&self) -> &[LanePoint] {
        &self.points
    }

    /// The same lane with each point moved by `f` (a rigid move, map into odom): the
    /// edges are distances, so they stay.
    #[must_use]
    pub fn moved(&self, f: impl Fn(f64, f64) -> (f64, f64)) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|p| {
                    let (x, y) = f(p.x, p.y);
                    LanePoint { x, y, ..*p }
                })
                .collect(),
        }
    }

    fn segments(&self) -> usize {
        self.points.len() - 1
    }

    /// (squared distance to segment `i`, the fraction along it, clamped).
    fn to_segment(&self, i: usize, x: f64, y: f64) -> (f64, f64) {
        let (a, b) = (&self.points[i], &self.points[i + 1]);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let t = (((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
        let (fx, fy) = (a.x + t * dx - x, a.y + t * dy - y);
        (fx * fx + fy * fy, t)
    }

    /// The segment nearest (x, y), searching the whole line (once a tick).
    #[must_use]
    pub fn nearest(&self, x: f64, y: f64) -> usize {
        (0..self.segments())
            .map(|i| (i, self.to_segment(i, x, y).0))
            .fold(
                (0, f64::INFINITY),
                |best, (i, d)| {
                    if d < best.1 {
                        (i, d)
                    } else {
                        best
                    }
                },
            )
            .0
    }

    /// Where (x, y) lies beside the lane, searching from segment `hint` to the nearest one
    /// either way (a sample's poses move a little each step, so a few segments at most).
    #[must_use]
    pub fn beside(&self, x: f64, y: f64, hint: usize) -> Beside {
        let last = self.segments() - 1;
        let mut i = hint.min(last);
        let mut here = self.to_segment(i, x, y);
        while i < last {
            let next = self.to_segment(i + 1, x, y);
            if next.0 >= here.0 {
                break;
            }
            i += 1;
            here = next;
        }
        while i > 0 {
            let before = self.to_segment(i - 1, x, y);
            if before.0 >= here.0 {
                break;
            }
            i -= 1;
            here = before;
        }
        let (a, b) = (&self.points[i], &self.points[i + 1]);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        // Sideways from the segment's line, + left: past either end it carries on straight.
        let lateral = (dx * (y - a.y) - dy * (x - a.x)) / dx.hypot(dy);
        Beside {
            index: i,
            lateral,
            edges: a.edges.between(&b.edges, here.1),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edges(soft: f64, hard: f64) -> Edges {
        Edges::from_numbers(hard, hard, soft, soft)
    }

    fn straight(soft: f64, hard: f64) -> Lane {
        Lane::new((0..=40).map(|i| LanePoint {
            x: f64::from(i) * 0.5,
            y: 0.0,
            edges: edges(soft, hard),
        }))
        .expect("a lane")
    }

    #[test]
    fn nan_edges_or_one_point_is_no_lane() {
        let nan = Edges::from_numbers(f64::NAN, f64::NAN, f64::NAN, f64::NAN);
        assert!(!nan.any());
        let line = |e: Edges| {
            (0..5).map(move |i| LanePoint {
                x: f64::from(i),
                y: 0.0,
                edges: e,
            })
        };
        assert!(Lane::new(line(nan)).is_none());
        assert!(Lane::new(line(edges(1.5, 2.0)).take(1)).is_none());
        assert!(Lane::new(line(edges(1.5, 2.0))).is_some());
        assert_eq!(Edges::from_numbers(-1.0, 2.0, 1.0, 1.0).left_hard, None);
    }

    #[test]
    fn beside_is_signed_left_and_finds_its_segment_from_any_hint() {
        let lane = straight(1.5, 2.0);
        let left = lane.beside(7.2, 0.8, 0);
        assert_eq!(left.index, 14);
        assert!((left.lateral - 0.8).abs() < 1e-12);
        assert_eq!(left.edges.left_soft, Some(1.5));
        let right = lane.beside(3.1, -1.1, 39);
        assert_eq!(right.index, 6);
        assert!((right.lateral + 1.1).abs() < 1e-12);
        assert_eq!(lane.nearest(7.2, 0.8), 14);
    }

    #[test]
    fn edges_between_points_and_unknown_ends() {
        let lane = Lane::new([
            LanePoint {
                x: 0.0,
                y: 0.0,
                edges: edges(1.0, 2.0),
            },
            LanePoint {
                x: 1.0,
                y: 0.0,
                edges: Edges {
                    left_soft: Some(2.0),
                    ..edges(f64::NAN, 3.0)
                },
            },
        ])
        .expect("a lane");
        let b = lane.beside(0.5, 0.0, 0);
        assert_eq!(b.edges.left_soft, Some(1.5));
        assert_eq!(b.edges.left_hard, Some(2.5));
        assert_eq!(b.edges.right_soft, None, "one end not known");
    }

    #[test]
    fn moved_keeps_the_edges() {
        let lane = straight(1.5, 2.0).moved(|x, y| (y, x));
        assert_eq!(lane.points()[2].y, 1.0);
        assert_eq!(lane.points()[2].edges, edges(1.5, 2.0));
        // Running north now: x = 1 is to the right.
        assert!((lane.beside(1.0, 5.0, 0).lateral + 1.0).abs() < 1e-12);
    }
}

//! Where the car is on the road ahead: the path frame.
//!
//! ```text
//!            left edge  ─────────────────────────────────────────
//!                              ▲ e_y (+ left)    ╱ the car's heading
//!   line ●────●────●────●──────●─────●────●   ╱  e_psi = heading − line's
//!                         foot ┘  ◄── along ──►        ║ end
//!            right edge ─────────────────────────────────────────
//! ```
//!
//! [`Lane`] is the road ahead's line and edges (`aav_interfaces/RoadAhead`), and
//! [`Lane::frame`] turns a pose into a [`Frame`]: how far along, how far off, how
//! twisted, how curved, and the room to the end and to each edge.

use ego_core::hypot;

/// The road ahead's line, one point every `spacing` m, with its edges.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Lane {
    /// m between points.
    pub spacing: f64,
    /// The points' x (m, the map frame).
    pub x: Vec<f64>,
    /// The points' y (m).
    pub y: Vec<f64>,
    /// The line's direction at each point (rad).
    pub heading: Vec<f64>,
    /// m from each point to the lane's left line (soft); NaN: not known.
    pub left: Vec<f64>,
    /// m from each point to the lane's right line (soft); NaN: not known.
    pub right: Vec<f64>,
    /// m along the line: the car may not pass it.
    pub end: f64,
}

/// The car in the lane's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// m along the line, from its first point.
    pub along: f64,
    /// m off the line, + left of it.
    pub offset: f64,
    /// rad: the car's heading less the line's, in (−π, π].
    pub twist: f64,
    /// 1/m: the line's curvature at the foot, + turning left.
    pub curvature: f64,
    /// m to the end (0 or less: at or past it).
    pub room: f64,
    /// m from the line to the left edge at the foot; NaN: not known.
    pub left: f64,
    /// m from the line to the right edge at the foot; NaN: not known.
    pub right: f64,
}

/// An angle in (−π, π].
#[must_use]
pub fn wrap(a: f64) -> f64 {
    let t = std::f64::consts::TAU;
    let w = a - t * (a / t).round();
    if w <= -std::f64::consts::PI {
        w + t
    } else {
        w
    }
}

impl Lane {
    /// How many points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.x.len().min(self.y.len()).min(self.heading.len())
    }

    /// No points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The edge at point `i` (NaN when the list is short).
    fn edge(list: &[f64], i: usize) -> f64 {
        list.get(i).copied().unwrap_or(f64::NAN)
    }

    /// The car at (x, y), heading `yaw`, in the lane's frame; `None` with fewer than two
    /// points (no line to measure from).
    #[must_use]
    pub fn frame(&self, x: f64, y: f64, yaw: f64) -> Option<Frame> {
        let n = self.len();
        if n < 2 {
            return None;
        }
        // The nearest segment and the fraction along it.
        let mut best: Option<(f64, usize, f64)> = None;
        for i in 0..n - 1 {
            let (ax, ay, bx, by) = (self.x[i], self.y[i], self.x[i + 1], self.y[i + 1]);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            // The first and last segments run on past their ends, so a car before the
            // line's start or past its end reads as such (room below 0), not as "at it".
            let lo = if i == 0 { f64::NEG_INFINITY } else { 0.0 };
            let hi = if i == n - 2 { f64::INFINITY } else { 1.0 };
            let t = if len2 > 0.0 {
                (((x - ax) * dx + (y - ay) * dy) / len2).clamp(lo, hi)
            } else {
                0.0
            };
            let d = hypot(ax + t * dx - x, ay + t * dy - y);
            if best.is_none_or(|(b, _, _)| d < b) {
                best = Some((d, i, t));
            }
        }
        let (_, i, t) = best?;
        let (ax, ay, bx, by) = (self.x[i], self.y[i], self.x[i + 1], self.y[i + 1]);
        let seg = hypot(bx - ax, by - ay);
        let (fx, fy) = (ax + t * (bx - ax), ay + t * (by - ay));
        // + left: the segment's direction crossed with foot -> car.
        let offset = if seg > 0.0 {
            ((bx - ax) * (y - fy) - (by - ay) * (x - fx)) / seg
        } else {
            hypot(x - fx, y - fy)
        };
        let h0 = self.heading[i];
        let dh = wrap(self.heading[i + 1] - h0);
        let line_heading = h0 + t.clamp(0.0, 1.0) * dh;
        let curvature = if seg > 0.0 { dh / seg } else { 0.0 };
        let tc = t.clamp(0.0, 1.0);
        let mix = |list: &[f64]| {
            let (a, b) = (Self::edge(list, i), Self::edge(list, i + 1));
            a + tc * (b - a)
        };
        let along = i as f64 * self.spacing + t * seg;
        Some(Frame {
            along,
            offset,
            twist: wrap(yaw - line_heading),
            curvature,
            room: self.end - along,
            left: mix(&self.left),
            right: mix(&self.right),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight(n: usize, width: f64) -> Lane {
        Lane {
            spacing: 0.5,
            x: (0..n).map(|i| i as f64 * 0.5).collect(),
            y: vec![0.0; n],
            heading: vec![0.0; n],
            left: vec![width; n],
            right: vec![width; n],
            end: (n - 1) as f64 * 0.5,
        }
    }

    #[test]
    fn left_of_the_line_is_positive() {
        let lane = straight(21, 1.5);
        let f = lane.frame(3.0, 0.4, 0.1).unwrap();
        assert!((f.offset - 0.4).abs() < 1e-12);
        assert!((f.twist - 0.1).abs() < 1e-12);
        assert!((f.along - 3.0).abs() < 1e-12);
        assert!((f.room - 7.0).abs() < 1e-12);
        assert!((f.left - 1.5).abs() < 1e-12);
        let g = lane.frame(3.0, -0.4, 0.0).unwrap();
        assert!((g.offset + 0.4).abs() < 1e-12);
    }

    #[test]
    fn a_circle_has_its_curvature() {
        // Radius 10 m, turning left: curvature 0.1 1/m.
        let r = 10.0;
        let n = 60;
        let step = 0.5 / r;
        let lane = Lane {
            spacing: 0.5,
            x: (0..n).map(|i| r * (i as f64 * step).sin()).collect(),
            y: (0..n).map(|i| r - r * (i as f64 * step).cos()).collect(),
            heading: (0..n).map(|i| i as f64 * step).collect(),
            left: vec![f64::NAN; n],
            right: vec![f64::NAN; n],
            end: 20.0,
        };
        let f = lane
            .frame(r * 0.3_f64.sin(), r - r * 0.3_f64.cos(), 0.3)
            .unwrap();
        assert!((f.curvature - 0.1).abs() < 1e-3, "{}", f.curvature);
        assert!(f.offset.abs() < 1e-3);
        assert!(f.twist.abs() < 1e-2);
        assert!(f.left.is_nan());
    }

    #[test]
    fn past_the_end_the_room_is_negative() {
        let lane = straight(21, 1.5);
        let f = lane.frame(12.0, 0.2, 0.0).unwrap();
        assert!((f.room + 2.0).abs() < 1e-12, "{}", f.room);
        assert!((f.offset - 0.2).abs() < 1e-12);
        let g = lane.frame(-1.0, 0.0, 0.0).unwrap();
        assert!((g.along + 1.0).abs() < 1e-12);
    }

    #[test]
    fn too_short_a_line_gives_no_frame() {
        assert!(Lane::default().frame(0.0, 0.0, 0.0).is_none());
        let mut one = straight(1, 1.0);
        one.end = 0.0;
        assert!(one.frame(0.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn wrap_stays_in_range() {
        for a in [-7.0, -3.2, -1.0, 0.0, 3.0, 3.2, 9.0] {
            let w = wrap(a);
            assert!(w > -std::f64::consts::PI && w <= std::f64::consts::PI);
        }
        assert!((wrap(std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-12);
    }
}

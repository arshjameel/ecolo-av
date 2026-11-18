//! The stop lines on the route (TODO 220): stop at each, then go, as at a seen stop sign
//! (`behaviour.py`'s `StopLines`).
//!
//! The planner sends the route's stop lines (`/drivers/nav/planner/route_stops`, from the
//! Lanelet2 map). Each one on the plan is a stop (`stop_line`, WAIT) whose `left` puts the
//! front bumper on the line. Standing within `near` m of it for `dwell` s serves it: it
//! goes, and the car goes on (GO, gently), unless something else holds it (a person
//! crossing: the moving hold). One passed by more than `near` m is served too. A served
//! line is forgotten once the car is `forget` m from it, so the next lap stops again.
//!
//! ```text
//!   car ●──────────────▶│ line          left = along to the line - front
//!             front ◀──▶│
//! ```

use crate::hypot;
use crate::paths::{path_left, Point};

/// When a stop line counts as stopped at. m, m/s, s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopLineSettings {
    /// m: the car's stop within this of the line's: standing at it.
    pub near: f64,
    /// m/s: slower than this is standing.
    pub still: f64,
    /// s standing at it: a full stop, then go.
    pub dwell: f64,
    /// m: a line further than this off the plan isn't on the way.
    pub off_path: f64,
    /// m: a served line this far from the car is forgotten.
    pub forget: f64,
    /// m: two lines this close are the same line.
    pub same: f64,
}

impl Default for StopLineSettings {
    fn default() -> Self {
        Self {
            near: 1.5,
            still: 0.05,
            dwell: 2.0,
            off_path: 3.0,
            forget: 30.0,
            same: 1.0,
        }
    }
}

/// The route's stop lines, and which the car has stopped at.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StopLines {
    s: StopLineSettings,
    lines: Vec<Point>,
    served: Vec<Point>,
    still_since: Option<f64>,
}

impl StopLines {
    /// No lines yet.
    #[must_use]
    pub fn new(settings: StopLineSettings) -> Self {
        Self {
            s: settings,
            ..Self::default()
        }
    }

    /// Take the route's stop lines (map frame, in order).
    pub fn set_lines(&mut self, lines: Vec<Point>) {
        self.lines = lines;
    }

    fn is_served(&self, line: Point) -> bool {
        self.served
            .iter()
            .any(|p| hypot(line.0 - p.0, line.1 - p.1) < self.s.same)
    }

    /// The unserved lines' `left`s (m from the car to its stop), nearest first.
    pub fn update(
        &mut self,
        now: f64,
        plan: &[Point],
        car: Option<Point>,
        speed: Option<f64>,
        front: f64,
    ) -> Vec<f64> {
        let s = self.s;
        let Some((x, y)) = car.filter(|_| plan.len() >= 2) else {
            self.still_since = None;
            return Vec::new();
        };
        self.served.retain(|p| hypot(p.0 - x, p.1 - y) <= s.forget);
        let (car_left, _) = path_left(plan, x, y);
        let mut ahead: Vec<(f64, Point)> = Vec::new();
        for &line in &self.lines {
            if self.is_served(line) {
                continue;
            }
            let (line_left, off) = path_left(plan, line.0, line.1);
            if off > s.off_path {
                continue;
            }
            let left = car_left - line_left - front;
            if left < -s.near {
                // Passed it.
                self.served.push(line);
                continue;
            }
            ahead.push((left, line));
        }
        // The nearest (the first of equals, as Python's min()).
        let nearest =
            ahead
                .iter()
                .enumerate()
                .fold(None::<(usize, f64)>, |best, (i, a)| match best {
                    Some((_, b)) if b <= a.0 => best,
                    _ => Some((i, a.0)),
                });
        match nearest {
            Some((k, left)) if left.abs() <= s.near && speed.is_some_and(|v| v.abs() < s.still) => {
                let since = *self.still_since.get_or_insert(now);
                if now - since >= s.dwell {
                    self.served.push(ahead[k].1);
                    self.still_since = None;
                    ahead.remove(k);
                }
            }
            _ => self.still_since = None,
        }
        let mut lefts: Vec<f64> = ahead.into_iter().map(|a| a.0).collect();
        lefts.sort_by(f64::total_cmp);
        lefts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_stand_then_go_and_stop_again_next_lap() {
        let plan: Vec<Point> = (0..=60).map(|i| (f64::from(i), 0.0)).collect();
        let mut lines = StopLines::new(StopLineSettings::default());
        // One on the way at x = 20, one 10 m off it.
        lines.set_lines(vec![(20.0, 0.5), (40.0, 10.0)]);
        let front = 2.0;
        let lefts = lines.update(0.0, &plan, Some((5.0, 0.0)), Some(2.0), front);
        assert_eq!(lefts.len(), 1);
        assert!((lefts[0] - 13.0).abs() < 1e-9, "{lefts:?}");
        // At the line, standing: still there until 2 s have passed, then gone.
        let at = Some((17.8, 0.0));
        assert_eq!(lines.update(1.0, &plan, at, Some(0.0), front).len(), 1);
        assert_eq!(lines.update(2.9, &plan, at, Some(0.0), front).len(), 1);
        assert!(lines.update(3.0, &plan, at, Some(0.0), front).is_empty());
        // Driving off: still served, then forgotten 30 m on (the line behind is off the
        // plan's end there, so not a stop).
        assert!(lines
            .update(4.0, &plan, Some((25.0, 0.0)), Some(2.0), front)
            .is_empty());
        assert!(lines
            .update(9.0, &plan, Some((52.0, 0.0)), Some(2.0), front)
            .is_empty());
        // A creep that never stands still is never served.
        let mut creep = StopLines::new(StopLineSettings::default());
        creep.set_lines(vec![(20.0, 0.0)]);
        for i in 0..100 {
            let got = creep.update(f64::from(i) * 0.1, &plan, at, Some(0.2), front);
            assert_eq!(got.len(), 1);
        }
        // No plan or no pose: nothing.
        assert!(creep
            .update(11.0, &plan[..1], at, Some(0.0), front)
            .is_empty());
        assert!(creep.update(11.0, &plan, None, Some(0.0), front).is_empty());
    }
}

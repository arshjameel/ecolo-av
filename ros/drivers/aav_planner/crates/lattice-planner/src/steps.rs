//! Far goals in steps (TODO 175, the three levels' step 3): when the goal is off the
//! costmap (a global costmap made of the chunks round the car), plan to the farthest
//! point of the way there that is on it, and again as the car goes.
//!
//! ```text
//!   ┌──────────────── the costmap (the chunks held) ───────────┐
//!   │  car ════ the route level's way (the lane line) ═══ ✕ ←──┼── EDGE_MARGIN
//!   └──────────────────────────────────────────────────────────┘ ═══════ ▶ goal
//!                                     plan to ✕ now; the chunks follow the car
//! ```
//!
//! The route level (the lane map's road graph, or a straight line off it) always covers
//! the whole way; only the middle level is cut to what is loaded.

use costmap::cost::{INSCRIBED, NO_INFORMATION};
use costmap::Grid;

use crate::{Plan, PlanError, Planner, Pose};

/// m: a step's end stays this far inside the costmap's edge, past the inflation's reach
/// (3 m), so walls just off the costmap can't be missed near it, with room to join.
/// Only a goal off the grid is cut: one on it, however near its edge, is planned to as
/// before (the test lot's goals are 5-8 m from its map's edge: cut, the car never got
/// there, 16 Nov).
pub const EDGE_MARGIN: f64 = 10.0;
/// m between the points tried along a straight way.
const STEP: f64 = 0.5;

/// Is (`x`, `y`) on `grid`, at least `margin` m from its edge?
#[must_use]
pub fn inside(grid: &Grid, x: f64, y: f64, margin: f64) -> bool {
    let (x1, y1) = grid.end();
    x >= grid.origin_x + margin
        && y >= grid.origin_y + margin
        && x <= x1 - margin
        && y <= y1 - margin
}

/// Where a way (in driving order) first leaves `grid` (less than `margin` m from its edge):
/// the index of its last pose before that. `None` when it never leaves, or the first pose
/// is already out.
#[must_use]
pub fn last_inside(grid: &Grid, way: &[Pose], margin: f64) -> Option<usize> {
    let out = way.iter().position(|p| !inside(grid, p.x, p.y, margin))?;
    out.checked_sub(1)
}

/// A cell the planner may end on: not lethal nor inscribed (unknown is allowed, as
/// `allow_unknown`).
fn free(grid: &Grid, x: f64, y: f64) -> bool {
    let c = grid.cost_at(x, y, NO_INFORMATION);
    c < INSCRIBED || c == NO_INFORMATION
}

/// Off the lane map: the step toward a far `goal` straight from `start`, the farthest
/// point on the line that is on `grid` (`margin` m in) and free, facing along the line.
/// `None` when the goal is on the grid already, or no such point is.
#[must_use]
pub fn toward(grid: &Grid, start: &Pose, goal: &Pose, margin: f64) -> Option<Pose> {
    if inside(grid, goal.x, goal.y, 0.0) {
        return None;
    }
    let (dx, dy) = (goal.x - start.x, goal.y - start.y);
    let length = dx.hypot(dy);
    if length < STEP {
        return None;
    }
    let yaw = dy.atan2(dx);
    let at = |s: f64| Pose::new(start.x + dx * s / length, start.y + dy * s / length, yaw);
    let n = (length / STEP) as usize;
    let last = (0..=n)
        .map(|k| at(k as f64 * STEP))
        .take_while(|p| inside(grid, p.x, p.y, margin))
        .count()
        .checked_sub(1)?;
    (1..=last)
        .rev()
        .map(|k| at(k as f64 * STEP))
        .find(|p| free(grid, p.x, p.y))
}

/// The lattice to `goal`, or, when it is off `grid`, to the [`toward`] step: the plan,
/// and whether it was cut short.
///
/// # Errors
///
/// The lattice's [`PlanError`].
pub fn plan_toward(
    planner: &Planner,
    grid: &Grid,
    start: &Pose,
    goal: &Pose,
    cancelled: &dyn Fn() -> bool,
) -> Result<(Plan, bool), PlanError> {
    match toward(grid, start, goal, EDGE_MARGIN) {
        Some(step) => planner
            .plan(grid, start, &step, cancelled)
            .map(|plan| (plan, true)),
        None => planner
            .plan(grid, start, goal, cancelled)
            .map(|plan| (plan, false)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::cost::{FREE, LETHAL};

    /// 100 x 60 m from (0, 0), 1 m cells, free.
    fn grid() -> Grid {
        Grid::new((0.0, 0.0), 1.0, 100, 60, FREE)
    }

    #[test]
    fn inside_keeps_off_the_edge() {
        let g = grid();
        assert!(inside(&g, 50.0, 30.0, 10.0));
        assert!(!inside(&g, 95.0, 30.0, 10.0));
        assert!(!inside(&g, -1.0, 30.0, 0.0));
        assert!(inside(&g, 100.0, 60.0, 0.0));
    }

    #[test]
    fn a_way_is_cut_where_it_leaves() {
        let g = grid();
        let way: Vec<Pose> = (0..20)
            .map(|k| Pose::new(10.0 + 10.0 * f64::from(k), 30.0, 0.0))
            .collect();
        // 90 m is the last at least 10 m in from x = 100.
        assert_eq!(last_inside(&g, &way, 10.0), Some(8));
        assert_eq!(last_inside(&g, &way[..5], 10.0), None);
        assert_eq!(last_inside(&g, &way[10..], 10.0), None, "starts outside");
    }

    #[test]
    fn a_far_goal_gives_the_farthest_free_point_on_the_way() {
        let mut g = grid();
        let start = Pose::new(10.0, 30.0, 0.0);
        let far = Pose::new(300.0, 30.0, 0.0);
        let step = toward(&g, &start, &far, 10.0).expect("a step");
        assert!((step.x - 90.0).abs() <= STEP && (step.y - 30.0).abs() < 1e-9);
        // A wall there: it backs off to a free cell.
        for y in 0..60 {
            for x in 86..=90 {
                g.set(x, y, LETHAL);
            }
        }
        let step = toward(&g, &start, &far, 10.0).expect("a step");
        assert!(step.x < 86.0 && step.x > 84.0, "{step:?}");
        // A goal on the grid: no step, even within the margin of its edge.
        assert!(toward(&g, &start, &Pose::new(40.0, 30.0, 0.0), 10.0).is_none());
        assert!(toward(&g, &start, &Pose::new(97.0, 55.0, 0.0), 10.0).is_none());
    }
}

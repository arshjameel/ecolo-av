//! Where the stops are: along the plan, in the lane ahead (`behaviour.py`'s helpers).
//!
//! ```text
//!  car ●──foot──────────── the plan ─────────────► end      path_left: (to the end, off it)
//!      │◄── front ──►│ lane: |y| <= half_width │◄ reach    lane_ahead, moving_ahead
//! ```

use crate::{hypot, py_max, py_min, py_sum};

/// A point (x, y), m.
pub type Point = (f64, f64);

/// Whose plan the car is driving.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Whose {
    /// A drive to a point's line A → B (it outranks Nav2 in twist_mux).
    Point,
    /// Nav2's `/plan`.
    Nav,
    /// No plan.
    Nobody,
}

/// The plan the car is driving, and whose: the point driver's line while it runs (two or
/// more points), else Nav2's plan if fresh, else nothing.
#[must_use]
pub fn active_plan<'a>(
    point: &'a [Point],
    nav: &'a [Point],
    nav_fresh: bool,
) -> (&'a [Point], Whose) {
    if point.len() >= 2 {
        (point, Whose::Point)
    } else if !nav.is_empty() && nav_fresh {
        (nav, Whose::Nav)
    } else {
        (&[], Whose::Nobody)
    }
}

/// (metres along the path from the car's foot to its end, the car's distance off it).
#[must_use]
pub fn path_left(points: &[Point], x: f64, y: f64) -> (f64, f64) {
    match points {
        [] => (f64::INFINITY, f64::INFINITY),
        [p] => {
            let d = hypot(p.0 - x, p.1 - y);
            (d, d)
        }
        _ => {
            let lengths: Vec<f64> = points
                .windows(2)
                .map(|w| hypot(w[1].0 - w[0].0, w[1].1 - w[0].1))
                .collect();
            let total = py_sum(lengths.iter().copied());
            let (mut best, mut best_s, mut done) = (f64::INFINITY, 0.0, 0.0);
            for (w, &length) in points.windows(2).zip(&lengths) {
                let (a, b) = (w[0], w[1]);
                let t = if length > 0.0 {
                    let t = ((x - a.0) * (b.0 - a.0) + (y - a.1) * (b.1 - a.1)) / (length * length);
                    py_min(1.0, py_max(0.0, t))
                } else {
                    0.0
                };
                let (px, py) = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
                let d = hypot(px - x, py - y);
                if d < best {
                    best = d;
                    best_s = done + t * length;
                }
                done += length;
            }
            (total - best_s, best)
        }
    }
}

/// The metres along the path from the car's foot to the first point where `bad(x, y)`;
/// infinity if no point on the rest of the path is bad.
pub fn along_until(points: &[Point], x: f64, y: f64, mut bad: impl FnMut(f64, f64) -> bool) -> f64 {
    if points.is_empty() {
        return f64::INFINITY;
    }
    let (left, _) = path_left(points, x, y);
    let total = py_sum(
        points
            .windows(2)
            .map(|w| hypot(w[1].0 - w[0].0, w[1].1 - w[0].1)),
    );
    let start = total - left;
    let mut done = 0.0;
    for (i, p) in points.iter().enumerate() {
        if i > 0 {
            let q = points[i - 1];
            done += hypot(p.0 - q.0, p.1 - q.1);
        }
        if done >= start && bad(p.0, p.1) {
            return done - start;
        }
    }
    f64::INFINITY
}

/// m: the lane ahead's reach.
pub const REACH: f64 = 30.0;

/// The nearest x (m, base_link) of a scan point in the lane straight ahead, or infinity.
///
/// `ranges`: a LaserScan in base_link; the lane is |y| <= `half_width`, from the front
/// bumper (`front` m ahead of base_link) out to `reach`.
#[must_use]
pub fn lane_ahead(
    ranges: &[f64],
    angle_min: f64,
    angle_increment: f64,
    front: f64,
    half_width: f64,
    reach: f64,
) -> f64 {
    let mut nearest = f64::INFINITY;
    for (i, &r) in ranges.iter().enumerate() {
        if !r.is_finite() || r <= 0.0 {
            continue;
        }
        let a = angle_min + i as f64 * angle_increment;
        let (px, py) = (r * a.cos(), r * a.sin());
        if front < px && px <= reach && py.abs() <= half_width {
            nearest = py_min(nearest, px);
        }
    }
    nearest
}

/// The metres from base_link, along the road ahead, to the nearest kerb on it; infinity
/// if none (`kerb_ahead()`, TODO 219).
///
/// `steps`: the voxels node's kerbs (`/perception/voxels/steps`, x y in base_link, those
/// under every beam remembered). One counts within `half_width` of the road ahead's line
/// (`road`, base_link; fewer than 2 points: straight ahead), past the bumper (`front`)
/// and within `reach` along it; the first stretch of line that near is its place. A kerb
/// beside the road, off the line, never slows the car.
#[must_use]
pub fn kerb_ahead(steps: &[Point], road: &[Point], front: f64, half_width: f64, reach: f64) -> f64 {
    let straight = [(0.0, 0.0), (reach, 0.0)];
    let line = if road.len() >= 2 { road } else { &straight[..] };
    let start = hypot(line[0].0, line[0].1);
    let mut nearest = f64::INFINITY;
    for &(px, py) in steps {
        if !(px.is_finite() && py.is_finite()) {
            continue;
        }
        let mut done = start;
        for pair in line.windows(2) {
            let ((ax, ay), (bx, by)) = (pair[0], pair[1]);
            let (dx, dy) = (bx - ax, by - ay);
            let length = hypot(dx, dy);
            let t = if length > 0.0 {
                py_min(
                    1.0,
                    py_max(0.0, ((px - ax) * dx + (py - ay) * dy) / (length * length)),
                )
            } else {
                0.0
            };
            let along = done + t * length;
            if hypot(px - (ax + t * dx), py - (ay + t * dy)) <= half_width
                && front < along
                && along <= reach
            {
                nearest = py_min(nearest, along);
                break;
            }
            done += length;
        }
    }
    nearest
}

/// One thing from `/world/things`, in odom: centre, speed over the ground, size (m, m/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// m/s.
    pub vx: f64,
    /// m/s.
    pub vy: f64,
    /// m along x.
    pub length: f64,
    /// m along y.
    pub width: f64,
}

/// m/s over the ground: slower is parked (the planner goes round it).
pub const MOVING: f64 = 0.3;

/// When a thing beside the lane counts as in it: it will walk in before the car comes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Crossing {
    /// m/s towards the lane, at least: slower is walking along it.
    pub across: f64,
    /// s: in the lane up to this long after the car would reach it.
    pub slack: f64,
    /// s: no further ahead than this.
    pub horizon: f64,
}

impl Default for Crossing {
    fn default() -> Self {
        Crossing {
            across: 0.2,
            slack: 2.0,
            horizon: 6.0,
        }
    }
}

/// The car, in odom: (x, y, yaw).
pub type Car = (f64, f64, f64);

/// The nearest x (m, base_link) of a moving thing in the lane straight ahead, or
/// infinity, as `moving_ahead()`.
///
/// The lane is |y| <= `half_width`, widened by each thing's half width, from the front
/// bumper out to `reach`. A thing slower than `moving` is parked: the planner's. A thing
/// beside the lane, walking into it before the car gets there, counts too (`crossing`;
/// `None`: only what is in the lane now). `speed`: the car's, m/s.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn moving_ahead(
    things: &[Seen],
    car: Car,
    front: f64,
    half_width: f64,
    reach: f64,
    moving: f64,
    speed: f64,
    crossing: Option<Crossing>,
) -> f64 {
    let (cx, cy, yaw) = car;
    let (c, s) = (yaw.cos(), yaw.sin());
    let mut nearest = f64::INFINITY;
    for t in things {
        if hypot(t.vx, t.vy) < moving {
            continue;
        }
        let (dx, dy) = (t.x - cx, t.y - cy);
        let (px, py) = (c * dx + s * dy, -s * dx + c * dy);
        let near = px - t.length / 2.0;
        if !(front < px + t.length / 2.0 && near <= reach) {
            continue;
        }
        let lane = half_width + t.width / 2.0;
        let walks_in = crossing.is_some_and(|crossing| {
            walks_in(
                py,
                -s * t.vx + c * t.vy,
                lane,
                near - front,
                speed - (c * t.vx + s * t.vy),
                crossing,
            )
        });
        if py.abs() <= lane || walks_in {
            nearest = py_min(nearest, py_max(near, front));
        }
    }
    nearest
}

/// Whether a thing `y` m to the side, moving `vy` m/s across, is in the lane in time.
fn walks_in(y: f64, vy: f64, lane: f64, gap: f64, closing: f64, crossing: Crossing) -> bool {
    let towards = -1.0_f64.copysign(y) * vy;
    if towards < crossing.across {
        return false;
    }
    let enters = (y.abs() - lane) / towards;
    let reaches = if closing > 0.0 {
        py_max(0.0, gap) / closing
    } else {
        f64::INFINITY
    };
    enters <= py_min(reaches + crossing.slack, crossing.horizon)
}

/// Which things have moved long enough to hold the car (`MovingFor`): `settle` s at
/// `moving` m/s or more.
///
/// The things' tracker gives a parked box a speed for a moment as the car passes, and a new
/// track a guessed one. A run starts only on a thing seen this cloud (not coasting); a sure
/// thing stays sure while it coasts (it walked into the blind zone), until it is slow or
/// gone. Nor while it grows (a building coming into view): a run starts again while its
/// size goes over [`MovingFor::GROW`] times its smallest plus [`MovingFor::GROW_SLACK`].
/// The node's `settle` is 1.5 s (`behaviour.py`'s `MovingFor` says why).
#[derive(Debug, Clone, PartialEq)]
pub struct MovingFor {
    settle: f64,
    moving: f64,
    /// id -> when its run began, and its smallest and largest size (m) since.
    since: std::collections::HashMap<u32, (f64, f64, f64)>,
    /// The ids that have moved `settle` s.
    sure: std::collections::HashSet<u32>,
}

impl MovingFor {
    /// Its size over this times its smallest in the run: the run starts again.
    pub const GROW: f64 = 1.5;
    /// m.
    pub const GROW_SLACK: f64 = 0.5;

    /// None moving yet.
    #[must_use]
    pub fn new(settle: f64, moving: f64) -> Self {
        MovingFor {
            settle,
            moving,
            since: std::collections::HashMap::new(),
            sure: std::collections::HashSet::new(),
        }
    }

    /// The things (id, coasting, seen), in order, that count as moving at `now` (s).
    pub fn update(&mut self, now: f64, things: &[(u32, bool, Seen)]) -> Vec<Seen> {
        let mut since = std::collections::HashMap::new();
        let mut sure = std::collections::HashSet::new();
        let mut out = Vec::new();
        for &(id, coasting, t) in things {
            let mut start = self.since.get(&id).copied();
            let mut was = self.sure.contains(&id);
            let size = py_max(t.length, t.width);
            if hypot(t.vx, t.vy) < self.moving {
                start = None;
                was = false;
            } else if !coasting {
                let run = match start {
                    None => (now, size, size),
                    Some((began, low, high)) => {
                        let (low, high) = (py_min(low, size), py_max(high, size));
                        if !was && high > Self::GROW * low + Self::GROW_SLACK {
                            (now, size, size)
                        } else {
                            (began, low, high)
                        }
                    }
                };
                start = Some(run);
                was = was || now - run.0 >= self.settle;
            } else if !was {
                start = None;
            }
            if let Some(run) = start {
                since.insert(id, run);
            }
            if was {
                sure.insert(id);
                out.push(t);
            }
        }
        self.since = since;
        self.sure = sure;
        out
    }
}

/// When a moving thing holds the car (`Nearby`; Ben, 16 Nov: "we should likely just stop").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nearby {
    /// m from the car: a moving thing this near holds it.
    pub radius: f64,
    /// s: one on the road ahead, at its speed, this soon holds it.
    pub within: f64,
    /// s between the places it is checked at.
    pub step: f64,
    /// m/s over the ground: slower is still (passed, not waited for).
    pub moving: f64,
}

impl Default for Nearby {
    fn default() -> Self {
        Nearby {
            radius: 10.0,
            within: 3.0,
            step: 0.25,
            moving: MOVING,
        }
    }
}

/// The metres from the car to the nearest moving thing that holds it, or infinity
/// (`moving_near`).
///
/// `car`: (x, y, yaw) in odom, as the things are; `front`: the front bumper, m ahead of
/// base_link (the car is the line base_link .. front). `road`: the road ahead's line in
/// base_link (empty: none). A thing at least `nearby.moving` m/s holds the car if it is
/// within `nearby.radius` (its centre, less half its narrow side), or if at its speed it
/// is within `half_width` + half its narrow side of the road's line at some time 0, step,
/// ... up to `nearby.within` s. A still thing never holds it: the planner passes it.
#[must_use]
pub fn moving_near(
    things: &[Seen],
    car: Car,
    front: f64,
    road: &[Point],
    half_width: f64,
    nearby: Nearby,
) -> f64 {
    let (cx, cy, yaw) = car;
    let (c, s) = (yaw.cos(), yaw.sin());
    // Python's int(): towards zero; never below 0 here.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let steps = if nearby.step > 0.0 {
        (nearby.within / nearby.step + 1e-9) as usize
    } else {
        0
    };
    let mut nearest = f64::INFINITY;
    for t in things.iter().filter(|t| hypot(t.vx, t.vy) >= nearby.moving) {
        let (dx, dy) = (t.x - cx, t.y - cy);
        let (px, py) = (c * dx + s * dy, -s * dx + c * dy);
        let (vx, vy) = (c * t.vx + s * t.vy, -s * t.vx + c * t.vy);
        let size = py_min(t.length, t.width) / 2.0;
        let gap = hypot(px - py_min(py_max(px, 0.0), front), py) - size;
        let lane = half_width + size;
        #[allow(clippy::cast_precision_loss)]
        let on_road = |k: usize| {
            let k = k as f64;
            path_left(road, px + vx * k * nearby.step, py + vy * k * nearby.step).1 <= lane
        };
        if gap <= nearby.radius || (!road.is_empty() && (0..=steps).any(on_road)) {
            nearest = py_min(nearest, py_max(0.0, gap));
        }
    }
    nearest
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn path_left_and_along_until() {
        let path: Vec<Point> = (0..11).map(|x| (f64::from(x), 0.0)).collect();
        let (left, off) = path_left(&path, 3.2, 0.5);
        assert!(close(left, 6.8) && close(off, 0.5));
        assert!(close(along_until(&path, 3.0, 0.0, |x, _| x >= 7.5), 5.0));
        assert_eq!(along_until(&path, 3.0, 0.0, |_, _| false), f64::INFINITY);
        assert_eq!(path_left(&[], 0.0, 0.0), (f64::INFINITY, f64::INFINITY));
    }

    #[test]
    fn lane_ahead_finds_the_nearest_thing_in_the_lane() {
        let n = 360;
        let inc = 2.0 * std::f64::consts::PI / f64::from(n);
        let mut ranges = vec![f64::INFINITY; n as usize];
        ranges[0] = 8.0;
        ranges[90] = 1.0;
        ranges[5] = 6.0;
        let got = lane_ahead(&ranges, 0.0, inc, 2.0, 0.6, REACH);
        assert!(close(got, 6.0 * (5.0 * inc).cos()));
        let none = vec![f64::INFINITY; n as usize];
        assert_eq!(lane_ahead(&none, 0.0, inc, 2.0, 0.6, REACH), f64::INFINITY);
    }

    #[test]
    fn the_point_drivers_line_outranks_nav2s_plan() {
        let line = [(0.0, 0.0), (20.0, 0.0)];
        let plan = [(0.0, 0.0), (5.0, 5.0)];
        assert_eq!(active_plan(&line, &plan, true), (&line[..], Whose::Point));
        assert_eq!(active_plan(&[], &plan, true), (&plan[..], Whose::Nav));
        assert_eq!(active_plan(&[], &plan, false).1, Whose::Nobody);
        assert_eq!(active_plan(&[(1.0, 1.0)], &[], true).1, Whose::Nobody);
    }

    fn seen(x: f64, y: f64, vx: f64, vy: f64, size: f64) -> Seen {
        Seen {
            x,
            y,
            vx,
            vy,
            length: size,
            width: size,
        }
    }

    fn ahead(things: &[Seen], car: Car, speed: f64, crossing: Option<Crossing>) -> f64 {
        moving_ahead(things, car, 1.8, 0.6, REACH, MOVING, speed, crossing)
    }

    #[test]
    fn moving_ahead_waits_for_what_moves_and_leaves_parked_things_to_the_planner() {
        let car = (10.0, 5.0, std::f64::consts::FRAC_PI_2);
        let walker = seen(10.3, 15.0, 0.8, 0.0, 0.5);
        let parked = seen(10.0, 9.0, 0.05, 0.0, 1.0);
        let beside = seen(14.0, 12.0, 1.0, 0.0, 0.5);
        let behind = seen(10.0, 2.0, 1.0, 0.0, 0.5);
        let c = Some(Crossing::default());
        assert!(close(
            ahead(&[walker, parked, beside, behind], car, 0.0, c),
            9.75
        ));
        assert_eq!(ahead(&[parked], car, 0.0, c), f64::INFINITY);
        let near = seen(10.0, 6.6, 0.5, 0.0, 0.6);
        assert!(close(ahead(&[near], car, 0.0, c), 1.8));
    }

    #[test]
    fn moving_ahead_counts_a_person_who_will_walk_into_the_lane_before_the_car_comes() {
        let car = (0.0, 0.0, 0.0);
        let c = Some(Crossing::default());
        let walker = seen(5.6, -3.4, 0.0, 0.8, 0.5);
        assert!(close(ahead(&[walker], car, 0.9, c), 5.35));
        assert_eq!(ahead(&[walker], car, 0.9, None), f64::INFINITY);
        let away = seen(5.6, -3.4, 0.0, -0.8, 0.5);
        let along = seen(5.6, -3.4, 1.2, 0.1, 0.5);
        assert_eq!(ahead(&[away, along], car, 0.9, c), f64::INFINITY);
        let left = seen(5.6, 3.4, 0.0, -0.8, 0.5);
        assert!(close(ahead(&[left], car, 0.9, c), 5.35));
        let slow = seen(20.0, -8.0, 0.0, 0.4, 0.5);
        assert_eq!(ahead(&[slow], car, 0.9, c), f64::INFINITY);
        let late = seen(4.0, -2.0, 0.0, 0.4, 0.5);
        let slack = Some(Crossing {
            slack: 1.0,
            ..Crossing::default()
        });
        assert_eq!(ahead(&[late], car, 2.0, slack), f64::INFINITY);
        assert!(close(ahead(&[walker], car, 0.0, c), 5.35));
    }

    #[test]
    fn a_thing_that_grows_as_it_comes_into_view_never_holds() {
        let mut sure = MovingFor::new(1.0, MOVING);
        for (now, size) in [(0.0, 0.8), (0.5, 3.0), (1.0, 4.5), (1.5, 6.0), (1.9, 6.2)] {
            let building = Seen {
                length: size,
                width: 1.0,
                ..seen(20.0, 12.0, 1.5, 0.0, 1.0)
            };
            assert!(sure.update(now, &[(9, false, building)]).is_empty());
        }
        for (now, size) in [(0.0, 0.5), (0.5, 0.8)] {
            let walker = Seen {
                length: size,
                width: 0.4,
                ..seen(5.0, -3.0, 0.0, 0.8, 0.5)
            };
            assert!(sure.update(now, &[(1, false, walker)]).is_empty());
        }
        let walker = Seen {
            length: 0.6,
            width: 0.4,
            ..seen(5.0, -3.0, 0.0, 0.8, 0.5)
        };
        assert_eq!(sure.update(1.0, &[(1, false, walker)]), [walker]);
    }

    #[test]
    fn a_thing_holds_only_once_it_has_moved_settle_s() {
        let mut sure = MovingFor::new(1.0, MOVING);
        let walker = seen(5.0, -3.0, 0.0, 0.8, 0.5);
        let boxed = seen(8.0, 0.0, 0.6, 0.0, 1.0);
        assert!(sure
            .update(0.0, &[(1, false, walker), (2, false, boxed)])
            .is_empty());
        let still = seen(8.0, 0.0, 0.05, 0.0, 1.0);
        assert!(sure
            .update(0.6, &[(1, false, walker), (2, false, still)])
            .is_empty());
        assert_eq!(
            sure.update(1.0, &[(1, false, walker), (2, false, boxed)]),
            [walker]
        );
        assert_eq!(
            sure.update(1.1, &[(1, true, walker), (3, true, boxed)]),
            [walker]
        );
        assert_eq!(
            sure.update(2.5, &[(1, true, walker), (3, true, boxed)]),
            [walker]
        );
        let halt = seen(5.0, -1.0, 0.0, 0.1, 0.5);
        assert!(sure.update(2.6, &[(1, false, halt)]).is_empty());
        assert!(sure.update(2.7, &[(1, false, walker)]).is_empty());
        assert_eq!(sure.update(3.7, &[(1, false, walker)]), [walker]);
    }

    #[test]
    fn moving_near_holds_for_what_moves_near_or_onto_the_road_and_passes_still_things() {
        let car = (0.0, 0.0, 0.0);
        let road: Vec<Point> = (0..80).map(|i| (0.5 * f64::from(i), 0.0)).collect();
        let near = |t: Seen| moving_near(&[t], car, 2.0, &road, 0.6, Nearby::default());
        let thing = |x, y, vx, vy, length, width| Seen {
            x,
            y,
            vx,
            vy,
            length,
            width,
        };
        // An oncoming car in the other lane; a person along the roadside 5 m away.
        assert!(close(
            near(thing(10.0, 3.5, -1.0, 0.0, 4.5, 1.8)),
            hypot(8.0, 3.5) - 0.9
        ));
        assert!(close(near(seen(1.0, -5.0, 1.0, 0.0, 0.5)), 4.75));
        // The parked box: never.
        assert_eq!(near(seen(8.0, 0.0, 0.1, 0.1, 1.0)), f64::INFINITY);
        // Cross traffic 25 m ahead, on the road in 1.5 s; away from it, or 5 s off: not.
        let cross = thing(25.0, -6.0, 0.0, 3.0, 4.5, 1.8);
        assert!(close(near(cross), hypot(23.0, 6.0) - 0.9));
        assert_eq!(near(thing(25.0, -6.0, 0.0, -3.0, 4.5, 1.8)), f64::INFINITY);
        assert_eq!(near(thing(25.0, -16.0, 0.0, 3.0, 4.5, 1.8)), f64::INFINITY);
        let no_road = moving_near(&[cross], car, 2.0, &[], 0.6, Nearby::default());
        assert_eq!(no_road, f64::INFINITY);
    }
}

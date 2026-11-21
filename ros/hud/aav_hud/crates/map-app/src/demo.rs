//! `--demo`: pretend data, so the map window runs on any computer with no ROS and no car.
//!
//! A small parking lot (50 m x 30 m: two rows of parked cars, a building), a car that
//! drives a route through it at up to 3 m/s, and a plan from the car to the goal every
//! second, as Nav2 sends. At the goal it waits 5 s, then drives the route back. The lot
//! sits on Carleton's campus, so the street map shows around it.
//!
//! A goal sent from the place search: the demo has no Nav2, so it only shows the goal
//! (WAITING FOR A PLAN, the distance in a straight line) and the car stops.
//!
//! The debug page (D) gets pretend news too, on a 30 s loop: the navigator drives (12 s),
//! a person takes over on the keyboard (6 s), the sensors lock holds the navigator while
//! the GPS is stale (6 s), then the navigator again. Only the page's words change: the
//! car on the map keeps its drive.

use std::time::{Duration, Instant};

use hud_core::{
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Mode, VehicleStatus},
};
use map_core::{
    debug::{Ask, MuxReport, Sent},
    geom::{Point, Pose},
    grid::{Grid, GridInfo},
    route::Route,
    state::{Car, Update},
};

use crate::source::DataSource;

/// The pretend map: 60 m x 40 m at 0.25 m a cell, from (-5, -5).
const RESOLUTION: f64 = 0.25;
const CELLS_X: u32 = 240;
const CELLS_Y: u32 = 160;
const ORIGIN: (f64, f64) = (-5.0, -5.0);
/// Where the lot sits on the map: the coordinates above are the lot's own, moved by
/// this. It puts the pretend lot on a parking lot of Carleton's campus, so
/// the street map (when downloaded) shows real streets around it.
const PLACE: Point = Point::new(188.0, 136.0);

/// The drive, before its corners are rounded.
const ROUTE: [(f64, f64); 6] = [
    (3.0, 3.0),
    (46.0, 3.0),
    (46.0, 16.2),
    (5.0, 16.2),
    (5.0, 26.0),
    (33.0, 26.0),
];
const TOP_SPEED: f64 = 3.0;
const WAIT_AT_GOAL: Duration = Duration::from_secs(5);
const CAR_PERIOD: Duration = Duration::from_millis(50);
const PLAN_PERIOD: Duration = Duration::from_secs(1);
/// The debug page's news: 10 a second, as the command mux's state.
const DEBUG_PERIOD: Duration = Duration::from_millis(100);
/// The debug page's loop.
const DEBUG_LOOP: f64 = 30.0;
/// The car's wheelbase, for the pretend steering angle (m).
const WHEELBASE: f64 = 1.9;
/// Nav2's plans have a point about every 5 cm; every 25 cm is plenty here.
const PLAN_SPACING: f64 = 0.25;

/// The demo source.
pub struct Demo {
    /// The drive, as dense points with rounded corners.
    path: Vec<Point>,
    /// Each point's distance from the start, along the path.
    along: Vec<f64>,
    /// How far along the car is.
    s: f64,
    speed: f64,
    last: Instant,
    /// When the car got to the goal.
    arrived_at: Option<Instant>,
    /// Driving the route forward, or back.
    forward: bool,
    sent_map: bool,
    car_due: Instant,
    plan_due: Instant,
    /// A goal from the place search, to show on the next drain.
    sent_goal: Option<(Pose, String)>,
    /// A goal came: the car stops and no more plans come.
    held: bool,
    /// When the demo started, and when the debug page's news is next due.
    started: Instant,
    debug_due: Instant,
    /// The car's last heading, for its turn rate.
    last_yaw: Option<f64>,
}

impl Demo {
    pub fn new(now: Instant) -> Self {
        let mut demo = Self {
            path: Vec::new(),
            along: Vec::new(),
            s: 0.0,
            speed: 0.0,
            last: now,
            arrived_at: None,
            forward: true,
            sent_map: false,
            car_due: now,
            plan_due: now,
            sent_goal: None,
            held: false,
            started: now,
            debug_due: now,
            last_yaw: None,
        };
        demo.set_path(corners(true));
        // Start part of the way along, so a screenshot shows a car on the move.
        demo.s = 14.0;
        demo.speed = TOP_SPEED;
        demo
    }

    fn set_path(&mut self, corners: Vec<Point>) {
        // Corners rounded over about 2 m, then a point every PLAN_SPACING, as Nav2 sends.
        self.path = densify(&rounded(&densify(&corners, 2.0), 4), PLAN_SPACING);
        self.along = std::iter::once(0.0)
            .chain(self.path.windows(2).scan(0.0, |total, w| {
                *total += w[0].distance(w[1]).0;
                Some(*total)
            }))
            .collect();
        self.s = 0.0;
    }

    fn length(&self) -> f64 {
        self.along.last().copied().unwrap_or(0.0)
    }

    /// The point `s` metres along the path.
    fn at(&self, s: f64) -> Point {
        let i = self.along.partition_point(|&a| a <= s);
        match (self.path.get(i.wrapping_sub(1)), self.path.get(i)) {
            (Some(&a), Some(&b)) => {
                let (sa, sb) = (self.along[i - 1], self.along[i]);
                let k = if sb > sa { (s - sa) / (sb - sa) } else { 0.0 };
                a + (b - a) * k
            }
            (Some(&a), None) => a,
            (None, Some(&b)) => b,
            (None, None) => Point::default(),
        }
    }

    fn pose(&self) -> Pose {
        let here = self.at(self.s);
        let ahead = self.at(self.s + 1.0);
        let behind = self.at(self.s - 1.0);
        let d = ahead - behind;
        Pose {
            position: here,
            yaw: d.y.atan2(d.x),
        }
    }

    /// Drives the car forward to `now`.
    fn step(&mut self, now: Instant) {
        let dt = now
            .saturating_duration_since(self.last)
            .as_secs_f64()
            .min(0.2);
        self.last = now;
        if self.held {
            self.speed = 0.0;
            return;
        }
        if let Some(at) = self.arrived_at {
            if now.saturating_duration_since(at) > WAIT_AT_GOAL {
                // Back the way it came.
                self.forward = !self.forward;
                self.set_path(corners(self.forward));
                self.arrived_at = None;
            }
            return;
        }
        let left = self.length() - self.s;
        // Slow down for the goal: about 1 m/s^2.
        let target = TOP_SPEED.min((2.0 * left).sqrt()).max(0.0);
        let change = (target - self.speed).clamp(-1.5 * dt, 1.0 * dt);
        self.speed = (self.speed + change).max(if left > 0.3 { 0.3 } else { 0.0 });
        self.s = (self.s + self.speed * dt).min(self.length());
        if self.length() - self.s < 0.05 {
            self.speed = 0.0;
            self.arrived_at = Some(now);
        }
    }

    /// The debug page's news at `now`: who drives (the 30 s loop), what it asks, what ego
    /// sends, the gate, the car, the watchdog.
    fn debug_news(&mut self, now: Instant) -> Vec<Update> {
        let yaw = self.pose().yaw;
        let dt = DEBUG_PERIOD.as_secs_f64();
        let turn = self.last_yaw.map_or(0.0, |y| {
            let d = (yaw - y + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            d / dt
        });
        self.last_yaw = Some(yaw);
        let t = now.duration_since(self.started).as_secs_f64() % DEBUG_LOOP;
        let row = |driver: &str, priority: &str, locks: &str, held: &str| {
            MuxReport::from_values(
                "",
                [
                    ("driver", driver),
                    ("priority", priority),
                    ("age_s", "0.04"),
                    ("locks", locks),
                    ("held", held),
                ],
            )
        };
        let mut out = Vec::new();
        let (ask, watchdog) = if (12.0..18.0).contains(&t) {
            out.push(Update::MuxState(row("keyboard", "90", "none", "none")));
            (
                Some(Ask {
                    speed: 0.5,
                    turn: 0.0,
                }),
                "OK",
            )
        } else if (18.0..24.0).contains(&t) {
            out.push(Update::MuxState(row("none", "0", "sensors 30", "nav")));
            (None, "STOP: gps STALE (nothing for 2.3 s (more than 2 s))")
        } else {
            out.push(Update::MuxState(row("nav", "20", "none", "none")));
            (
                Some(Ask {
                    speed: self.speed,
                    turn,
                }),
                "OK",
            )
        };
        out.push(Update::Watchdog(watchdog.into()));
        let asked = ask.unwrap_or_default();
        if let Some(a) = ask {
            out.push(Update::Command(a));
        }
        // Ego turns the turn rate into a steering angle (the bicycle model).
        let steering = if asked.speed.abs() > 0.05 {
            (WHEELBASE * asked.turn / asked.speed).atan()
        } else {
            0.0
        };
        let accel = if ask.is_none() { -1.5 } else { 0.2 };
        out.push(Update::Actuator(Sent {
            speed: asked.speed,
            steering,
            accel,
        }));
        out.push(Update::Gate("OPEN - sim: every source SIM".into()));
        out.push(Update::Vehicle(VehicleStatus {
            mode: Mode::Computer,
            armed: true,
            speed: MetresPerSecond((asked.speed * 0.97) as f32),
            steering_angle: Radians((steering * 0.9) as f32),
            throttle: Fraction::new(if accel > 0.0 { 0.3 } else { 0.0 }),
            brake: Fraction::new(if accel < 0.0 { 0.6 } else { 0.0 }),
            ..VehicleStatus::default()
        }));
        out
    }

    fn plan(&self) -> Route {
        let start = self.along.partition_point(|&a| a <= self.s);
        Route::new(
            std::iter::once(self.at(self.s))
                .chain(self.path[start.min(self.path.len())..].iter().copied()),
        )
    }
}

impl DataSource for Demo {
    fn label(&self) -> &'static str {
        "DEMO"
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        self.step(now);
        let mut out = Vec::new();
        if !self.sent_map {
            self.sent_map = true;
            out.push(lot());
        }
        if now >= self.car_due {
            self.car_due = now + CAR_PERIOD;
            out.push(Update::Car(Car {
                pose: self.pose(),
                speed: MetresPerSecond(self.speed as f32),
                frame: "map".into(),
            }));
        }
        if let Some((pose, frame)) = self.sent_goal.take() {
            out.push(Update::Goal { pose, frame });
        }
        if now >= self.debug_due {
            if self.debug_due == self.started {
                out.push(Update::Follower("mppi_lag (aav_mppi_lag)".into()));
            }
            self.debug_due = now + DEBUG_PERIOD;
            out.extend(self.debug_news(now));
        }
        // Nav2 stops planning at the goal.
        if now >= self.plan_due && self.arrived_at.is_none() && !self.held {
            self.plan_due = now + PLAN_PERIOD;
            out.push(Update::Plan {
                route: self.plan(),
                frame: "map".into(),
            });
        }
        out
    }

    fn send_goal(&mut self, goal: Pose, frame: &str) -> anyhow::Result<()> {
        self.sent_goal = Some((goal, frame.to_owned()));
        self.held = true;
        Ok(())
    }
}

/// The route's corners, forward or back.
fn corners(forward: bool) -> Vec<Point> {
    let mut c: Vec<Point> = ROUTE
        .iter()
        .map(|&(x, y)| Point::new(x, y) + PLACE)
        .collect();
    if !forward {
        c.reverse();
    }
    c
}

/// Points every `spacing` metres along the corners' straight lines.
fn densify(corners: &[Point], spacing: f64) -> Vec<Point> {
    let mut out: Vec<Point> = corners
        .windows(2)
        .flat_map(|w| {
            let (a, b) = (w[0], w[1]);
            let n = (a.distance(b).0 / spacing).ceil().max(1.0) as usize;
            (0..n).map(move |i| a + (b - a) * (i as f64 / n as f64))
        })
        .collect();
    out.extend(corners.last());
    out
}

/// Rounds the corners (Chaikin's method, `passes` times); keeps both ends.
fn rounded(points: &[Point], passes: usize) -> Vec<Point> {
    (0..passes).fold(points.to_vec(), |p, _| {
        let (Some(&first), Some(&last)) = (p.first(), p.last()) else {
            return p;
        };
        std::iter::once(first)
            .chain(
                p.windows(2)
                    .flat_map(|w| [w[0] * 0.75 + w[1] * 0.25, w[0] * 0.25 + w[1] * 0.75]),
            )
            .chain(std::iter::once(last))
            .collect()
    })
}

/// The pretend parking lot, as Nav2's map_server would send it.
fn lot() -> Update {
    let cell_value = |x: f64, y: f64| -> i8 {
        let inside =
            |x0: f64, y0: f64, x1: f64, y1: f64| (x0..x1).contains(&x) && (y0..y1).contains(&y);
        // Outside the lot's fence: nobody has looked.
        if !inside(-1.0, -1.0, 51.0, 31.0) {
            return -1;
        }
        let fence = !inside(-0.5, -0.5, 50.5, 30.5);
        // Two rows of parked cars, 2 m x 4.5 m, a gap every fourth.
        let parked = |row_y: f64| {
            (row_y..row_y + 4.5).contains(&y)
                && (9.0..41.0).contains(&x)
                && ((x - 9.0) % 2.8) < 2.0
                && (((x - 9.0) / 2.8) as i32) % 4 != 3
        };
        let building = inside(38.0, 21.0, 50.5, 30.5);
        let lamp = [(6.5, 13.0), (47.5, 13.0)]
            .iter()
            .any(|&(lx, ly)| (x - lx).hypot(y - ly) < 0.4);
        if fence || parked(8.5) || parked(19.5) || building || lamp {
            100
        } else {
            0
        }
    };
    let cells = (0..CELLS_Y)
        .flat_map(|row| (0..CELLS_X).map(move |col| (col, row)))
        .map(|(col, row)| {
            let x = ORIGIN.0 + (f64::from(col) + 0.5) * RESOLUTION;
            let y = ORIGIN.1 + (f64::from(row) + 0.5) * RESOLUTION;
            cell_value(x, y)
        })
        .collect();
    let info = GridInfo {
        width: CELLS_X,
        height: CELLS_Y,
        resolution: RESOLUTION,
        origin: Pose::new(ORIGIN.0 + PLACE.x, ORIGIN.1 + PLACE.y, 0.0),
    };
    Grid::new(info, "map".into(), cells).map_or_else(|e| Update::BadMap(e.to_string()), Update::Map)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_demo_sends_a_map_a_car_and_a_plan() {
        let t0 = Instant::now();
        let mut demo = Demo::new(t0);
        let first = demo.drain(t0);
        assert!(first.iter().any(|u| matches!(u, Update::Map(_))));
        assert!(first.iter().any(|u| matches!(u, Update::Car(_))));
        assert!(first.iter().any(|u| matches!(u, Update::Plan { .. })));
    }

    /// The debug page's loop: the navigator, then a person, then the sensors lock.
    #[test]
    fn the_debug_page_sees_the_navigator_a_person_then_a_lock() {
        use map_core::{config::MapConfig, debug::cards, state::MapState};
        let t0 = Instant::now();
        let mut demo = Demo::new(t0);
        let mut state = MapState::new(MapConfig::default());
        let mut who_at = |secs: u64| {
            let now = t0 + Duration::from_secs(secs);
            for u in demo.drain(now) {
                state.apply(u, now);
            }
            let (path, safety) = cards(&state, now);
            (path[0].big.clone(), safety[0].big.clone())
        };
        assert_eq!(who_at(1).0, "NAVIGATOR (AI)");
        assert_eq!(who_at(13).0, "KEYBOARD");
        assert_eq!(who_at(19), ("NOBODY".into(), "SENSORS".into()));
        assert_eq!(who_at(25).0, "NAVIGATOR (AI)");
    }

    #[test]
    fn the_car_gets_there_and_comes_back() {
        let t0 = Instant::now();
        let mut demo = Demo::new(t0);
        let start = demo.pose().position;
        // 0.1 s steps: long enough to drive the ~90 m route and wait.
        let mut arrived = false;
        for i in 1..1200 {
            demo.drain(t0 + Duration::from_millis(100 * i));
            arrived |= demo.arrived_at.is_some();
        }
        assert!(arrived);
        assert!(demo.pose().position.distance(start).0 > 1.0);
    }

    #[test]
    fn a_sent_goal_shows_and_the_car_stops() {
        let t0 = Instant::now();
        let mut demo = Demo::new(t0);
        demo.drain(t0);
        demo.send_goal(Pose::new(1.0, 2.0, 0.5), "map")
            .expect("the demo takes it");
        let t1 = t0 + Duration::from_secs(2);
        let next = demo.drain(t1);
        assert!(next.contains(&Update::Goal {
            pose: Pose::new(1.0, 2.0, 0.5),
            frame: "map".into()
        }));
        assert!(!next.iter().any(|u| matches!(u, Update::Plan { .. })));
        let here = demo.pose().position;
        demo.drain(t1 + Duration::from_secs(2));
        assert_eq!(demo.pose().position, here);
    }

    #[test]
    fn corners_are_rounded_but_the_ends_stay() {
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
        ];
        let r = rounded(&densify(&pts, 1.0), 3);
        assert_eq!(r.first(), Some(&pts[0]));
        assert_eq!(r.last(), Some(&pts[2]));
        // The corner itself is cut.
        assert!(r.iter().all(|p| p.distance(pts[1]).0 > 0.1));
    }
}

//! Pretend data, as a drop-in for ROS: a walled lot, a car driving a loop, cubes, a camera
//! picture and log lines, through the same [`Feed`] and updates. No Bevy, no ROS, so its
//! tests run anywhere.

use std::{f64::consts::TAU, time::Instant};

use debug_core::{
    cubes::{Band, Cube, CubeScan, Metres},
    frames::Stamp,
    ndt::{MapCells, NdtStatus},
};
use hud_core::{
    origin::{Announcement, Entry, Origin},
    picture::Picture,
    things::Thing,
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use map_core::{
    geom::{Point, Pose},
    grid::{Grid, GridInfo},
};

use crate::feed::{self, Feed, Level, RoadAhead, Update};

/// A straight piece of wall on the map, metres.
pub type Wall = (Point, Point);

/// The picture's size: what Gazebo's chase camera sends today.
pub const PICTURE_WIDTH: u32 = 640;
pub const PICTURE_HEIGHT: u32 = 360;

/// The lot's walls: the outside with a gap, a building and a row of parked cars.
pub fn walls() -> Vec<Wall> {
    let p = Point::new;
    vec![
        (p(0.0, 0.0), p(50.0, 0.0)),
        (p(50.0, 0.0), p(50.0, 40.0)),
        (p(50.0, 40.0), p(0.0, 40.0)),
        (p(0.0, 40.0), p(0.0, 14.0)),
        (p(0.0, 10.0), p(0.0, 0.0)),
        (p(19.0, 20.0), p(31.0, 20.0)),
        (p(31.0, 20.0), p(31.0, 30.0)),
        (p(31.0, 30.0), p(19.0, 30.0)),
        (p(19.0, 30.0), p(19.0, 20.0)),
        (p(38.0, 7.0), p(38.0, 33.0)),
    ]
}

/// The street route: out of the lot's gap and 200 m along a street to the east, then
/// north (the coarse chunks follow it, `--chunks`).
pub fn route() -> Vec<Point> {
    let p = Point::new;
    vec![
        p(10.0, 12.0),
        p(0.0, 12.0),
        p(-6.0, 12.0),
        p(-6.0, -10.0),
        p(200.0, -10.0),
        p(200.0, 120.0),
    ]
}

/// The car at `t` seconds: a slow loop inside the lot, at about 1 m/s.
pub fn car_at(t: f64) -> Pose {
    const RADIUS: f64 = 9.0;
    let angle = t / RADIUS;
    Pose::new(
        10.0 + RADIUS * angle.cos(),
        12.0 + RADIUS * angle.sin(),
        (angle + TAU / 4.0).rem_euclid(TAU),
    )
}

/// A repeatable number in 0..1 for a cube.
fn noise(i: u32, salt: u32) -> f32 {
    let mut x = i.wrapping_mul(2_654_435_761) ^ salt.wrapping_mul(805_459_861);
    x ^= x >> 15;
    x = x.wrapping_mul(2_246_822_519);
    x ^= x >> 13;
    x as f32 / u32::MAX as f32
}

/// Cube `i`'s centre, relative to the car: x and y up to 30 m away, z up to 3 m.
pub fn cube_offset(i: u32) -> [f32; 3] {
    [
        (noise(i, 1) - 0.5) * 60.0,
        (noise(i, 2) - 0.5) * 60.0,
        noise(i, 3) * 3.0,
    ]
}

/// One rgb8 frame at `t` seconds, as a camera message carries it: sky, road, moving posts.
pub fn picture_at(t: f64) -> Vec<u8> {
    let (width, height) = (PICTURE_WIDTH as usize, PICTURE_HEIGHT as usize);
    let shift = (t * 40.0) as usize;
    let mut data = Vec::with_capacity(width * height * 3);
    for y in 0..height {
        for x in 0..width {
            let pixel: [u8; 3] = if y < height * 2 / 5 {
                [190, 195, 200]
            } else if (x + shift) % 160 < 14 && y < height * 3 / 5 {
                [80, 45, 30]
            } else {
                let shade = 70 + (y * 60 / height) as u8;
                [shade, shade, shade]
            };
            data.extend_from_slice(&pixel);
        }
    }
    data
}

/// Log line number `n`, as the console will get them from `/rosout`.
pub fn log_line(n: u32) -> (Level, String) {
    match n % 9 {
        4 => (
            Level::Warn,
            format!("[controller_server] running late, {} ms", 40 + n % 30),
        ),
        8 => (
            Level::Error,
            "[sensor_watchdog] pretend: GPS stale for 0.6 s".to_owned(),
        ),
        k if k % 2 == 0 => (
            Level::Info,
            format!("[ndt] matched, score {:.2}", 2.5 + f64::from(n % 7) * 0.05),
        ),
        _ => (
            Level::Info,
            format!("[voxels] {} cubes, 6 ms", 900 + n % 60),
        ),
    }
}

/// Where odom sits on the map, as a real run's isn't the same either.
const ODOM_SHIFT: (f64, f64) = (0.5, -0.3);
/// Below this height a cube counts as road: `debug_hud.yaml`'s `ground_height`.
const GROUND_HEIGHT: f64 = 0.25;
/// The cubes' edge, metres: `aav_voxels`' size.
pub const CUBE_SIDE: f32 = 0.25;
/// The pretend grids' cell, metres.
const CELL: f64 = 0.25;
/// Nav2's danger halo round a wall, metres.
const HALO: f64 = 1.5;

/// The distance from `p` to a piece of wall.
fn to_wall(p: Point, (a, b): Wall) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let k = (((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
    p.distance(Point::new(a.x + k * dx, a.y + k * dy)).0
}

/// A grid of the lot, each cell's value from its distance to the nearest wall.
fn lot_grid(value: impl Fn(f64) -> i8) -> Option<Grid> {
    const WIDTH: u32 = 220;
    const HEIGHT: u32 = 180;
    const MARGIN: f64 = 2.5;
    let walls = walls();
    let mut cells = Vec::with_capacity((WIDTH * HEIGHT) as usize);
    for row in 0..HEIGHT {
        for column in 0..WIDTH {
            let p = Point::new(
                (f64::from(column) + 0.5) * CELL - MARGIN,
                (f64::from(row) + 0.5) * CELL - MARGIN,
            );
            let nearest = walls
                .iter()
                .map(|w| to_wall(p, *w))
                .fold(f64::MAX, f64::min);
            cells.push(value(nearest));
        }
    }
    let info = GridInfo {
        width: WIDTH,
        height: HEIGHT,
        resolution: CELL,
        origin: Pose::new(-MARGIN, -MARGIN, 0.0),
    };
    Grid::new(info, "map".to_owned(), cells).ok()
}

/// The lot as NDT's map would see it.
pub fn ndt_cells() -> MapCells {
    let points = walls().into_iter().flat_map(|(a, b)| {
        (0..=200).flat_map(move |step| {
            let k = f64::from(step) / 200.0;
            [0.5, 1.5].map(|z| [a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k, z])
        })
    });
    MapCells::from_points(points, 1.0)
}

/// A pretend road ahead (`/world/road_ahead`): from the gate east along the lot's lower
/// lane, slowing to a stop for the goal, as the behaviour planner draws it.
pub fn road_ahead() -> RoadAhead {
    const SPACING: f64 = 1.0;
    const LENGTH: u32 = 30;
    const TOP: f64 = 2.5;
    let points: Vec<_> = (0..=LENGTH)
        .map(|i| {
            let along = f64::from(i) * SPACING;
            let to_end = f64::from(LENGTH) * SPACING - along;
            // The stopping law's shape: v = sqrt(2 a d), a = 0.5 m/s², capped.
            let limit = (2.0 * 0.5 * to_end).sqrt().min(TOP);
            (2.0 + along, 10.0, 0.0, limit)
        })
        .collect();
    let end = f64::from(LENGTH) * SPACING;
    RoadAhead::from_parts(&points, SPACING, end, &[(end, "goal".to_owned())])
}

/// Pretend ROS.
pub struct PretendFeed {
    started: Instant,
    cubes: u32,
    sent_once: bool,
    next_scan: f64,
    next_picture: f64,
    next_slow: f64,
    next_line: u32,
    next_ahead: f64,
    plan: Option<Point>,
}

/// The squiggly line's four updates at `t` seconds (`hud_core::ahead::pretend`), base_link.
pub fn ahead_at(t: f64) -> Vec<Update> {
    let ahead = hud_core::ahead::pretend(std::time::Duration::from_secs_f64(t.max(0.0)));
    let mut updates = vec![Update::Trace(ahead.trace), Update::PlanAhead(ahead.plan)];
    updates.extend(ahead.stop.map(Update::StopPoint));
    updates.extend(ahead.summary.map(Update::Ahead));
    updates
}

/// The pretend things at `t` seconds, round the car (odom): a person crossing 6 m ahead,
/// a stop sign by the road, a car further on and a box no camera has named. They ride
/// along with the car, so they stay in view.
pub fn things_at(t: f64, car: Pose) -> Vec<Thing> {
    let thing = |id, name: &str, at: (f64, f64), (length, width, height)| {
        let p = feed::from_base(car, at);
        Thing {
            id,
            name: name.to_owned(),
            x: p.x,
            y: p.y,
            length,
            width,
            height,
            ..Thing::default()
        }
    };
    let walk = (t / 12.0 * TAU).sin() * 3.0;
    vec![
        thing(3, "person", (6.0, walk), (0.5, 0.5, 1.7)),
        thing(7, "stop sign", (9.0, -2.5), (0.3, 0.3, 2.1)),
        thing(9, "car", (16.0, 1.5), (4.0, 1.8, 1.5)),
        thing(4, "unknown", (4.0, 3.0), (0.6, 0.6, 0.5)),
    ]
}

impl PretendFeed {
    pub fn new(cubes: u32) -> Self {
        Self {
            started: Instant::now(),
            cubes,
            sent_once: false,
            next_scan: 0.0,
            next_picture: 0.0,
            next_slow: 0.0,
            next_line: 0,
            next_ahead: 0.0,
            plan: None,
        }
    }

    fn scan(&self, car: Pose) -> (CubeScan, CubeScan) {
        let snap = |v: f32| (v / CUBE_SIDE).round() * CUBE_SIDE;
        let (mut high, mut road) = (Vec::new(), Vec::new());
        for i in 0..self.cubes {
            let [x, y, z] = cube_offset(i);
            let band = Band::of(Metres(f64::from(z)), Metres(GROUND_HEIGHT));
            let centre = [
                snap(car.position.x as f32 + x),
                snap(car.position.y as f32 + y),
                snap(z),
            ];
            let cube = Cube { centre, band };
            if band == Band::Ground {
                road.push(cube);
            } else {
                high.push(cube);
            }
        }
        let scan = |cubes| CubeScan {
            stamp: Stamp::new(0, 0),
            car,
            cubes,
            size: CUBE_SIDE,
            ..Default::default()
        };
        (scan(high), scan(road))
    }
}

impl Feed for PretendFeed {
    fn label(&self) -> &'static str {
        "PRETEND"
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        let t = now.saturating_duration_since(self.started).as_secs_f64();
        let mut updates = Vec::new();
        let car_map = car_at(t);
        let car_odom = Pose::new(
            car_map.position.x - ODOM_SHIFT.0,
            car_map.position.y - ODOM_SHIFT.1,
            car_map.yaw,
        );
        updates.push(Update::CarInMap(car_map));
        updates.push(Update::CarInOdom(car_odom));
        if !self.sent_once {
            self.sent_once = true;
            let occupied = |d: f64| if d < CELL { 100 } else { 0 };
            let cost = |d: f64| {
                if d < CELL {
                    100
                } else if d < HALO {
                    (70.0 * (1.0 - d / HALO)) as i8 + 10
                } else {
                    0
                }
            };
            updates.extend(lot_grid(occupied).map(Update::Map));
            updates.push(Update::RoadAhead(road_ahead()));
            updates.push(Update::Route(route()));
            updates.extend(lot_grid(cost).map(Update::Costmap));
            updates.push(Update::Origin(Announcement {
                announcer: "pretend".to_owned(),
                entries: vec![Entry {
                    name: "everything".to_owned(),
                    topic: "/pretend".to_owned(),
                    origin: Origin::Sim,
                }],
            }));
        }
        if t >= self.next_scan {
            self.next_scan = t + 0.1;
            let (high, road) = self.scan(car_odom);
            updates.push(Update::Cubes(high));
            updates.push(Update::Road(road));
        }
        if t >= self.next_picture {
            self.next_picture = t + 0.1;
            let picture = Picture::from_image(
                "rgb8",
                PICTURE_WIDTH,
                PICTURE_HEIGHT,
                PICTURE_WIDTH * 3,
                &picture_at(t),
            );
            updates.extend(picture.ok().map(Update::Picture));
        }
        if t >= self.next_ahead {
            // ego and look_ahead send theirs at 10 Hz.
            self.next_ahead = t + 0.1;
            updates.extend(ahead_at(t));
            // things sends at 10 Hz too.
            updates.push(Update::Things(things_at(t, car_odom)));
        }
        if t >= self.next_slow {
            self.next_slow = t + 0.5;
            let (level, line) = log_line(self.next_line);
            self.next_line = self.next_line.wrapping_add(1);
            updates.push(Update::Log(level, line));
            updates.push(Update::Ndt(NdtStatus::from_message(
                0,
                "TRACKING",
                [
                    ("score_per_point", "2.67"),
                    ("match_ms", "2.5"),
                    ("overlap", "1.0"),
                ],
            )));
            updates.push(Update::NdtPose(Pose::new(
                car_map.position.x + 0.09,
                car_map.position.y,
                car_map.yaw,
            )));
            updates.push(Update::Gate("OPEN - sim: every source SIM".to_owned()));
            updates.push(Update::Watchdog("OK".to_owned()));
            updates.push(Update::Driving("nav (20)".to_owned()));
            updates.push(Update::Vehicle(VehicleStatus {
                mode: Mode::from(0),
                armed: true,
                estop_active: false,
                speed: MetresPerSecond(1.0),
                steering_angle: Radians(0.0),
                throttle: Fraction::new(0.2),
                brake: Fraction::new(0.0),
                faults: Faults(0),
            }));
        }
        if let Some(goal) = self.plan {
            updates.push(Update::Plan(vec![car_map.position, goal]));
        }
        updates
    }

    fn may_command(&self) -> Result<(), String> {
        Ok(())
    }

    fn send_goal(&mut self, goal: Pose) -> Result<(), String> {
        self.plan = Some(goal.position);
        Ok(())
    }

    fn press_estop(&mut self) -> Result<(), String> {
        Err("pretend data: there is no e-stop to set".to_owned())
    }

    fn cancel_goal(&mut self) -> Result<(), String> {
        self.plan = None;
        Ok(())
    }

    fn swap_tracker(&mut self, name: &'static str) -> Result<(), String> {
        Err(format!("pretend data: no track node to swap to {name}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_car_stays_inside_the_lot() {
        for second in 0..600 {
            let car = car_at(f64::from(second));
            assert!((0.0..=50.0).contains(&car.position.x));
            assert!((0.0..=40.0).contains(&car.position.y));
        }
    }

    #[test]
    fn a_cube_is_the_same_every_time() {
        assert_eq!(cube_offset(7), cube_offset(7));
        assert_ne!(cube_offset(7), cube_offset(8));
    }

    #[test]
    fn cubes_stay_in_their_box() {
        for i in 0..1000 {
            let [x, y, z] = cube_offset(i);
            assert!(x.abs() <= 30.0 && y.abs() <= 30.0);
            assert!((0.0..=3.0).contains(&z));
        }
    }

    #[test]
    fn a_picture_has_every_byte_a_message_would() {
        let expected = (PICTURE_WIDTH * PICTURE_HEIGHT * 3) as usize;
        assert_eq!(picture_at(1.5).len(), expected);
    }

    #[test]
    fn the_shared_reader_accepts_the_pretend_picture() {
        let picture = hud_core::picture::Picture::from_image(
            "rgb8",
            PICTURE_WIDTH,
            PICTURE_HEIGHT,
            PICTURE_WIDTH * 3,
            &picture_at(0.0),
        );
        assert!(picture.is_ok());
    }

    #[test]
    fn the_first_drain_has_a_map_and_a_picture() {
        let mut feed = PretendFeed::new(100);
        let updates = feed.drain(Instant::now());
        assert!(updates.iter().any(|u| matches!(u, Update::Map(_))));
        assert!(updates.iter().any(|u| matches!(u, Update::Picture(_))));
        assert!(updates.iter().any(|u| matches!(u, Update::Cubes(_))));
    }

    #[test]
    fn the_first_drain_has_the_road_ahead_ending_at_the_goal() {
        let mut feed = PretendFeed::new(10);
        let updates = feed.drain(Instant::now());
        let road = updates
            .iter()
            .find_map(|u| match u {
                Update::RoadAhead(r) => Some(r.clone()),
                _ => None,
            })
            .expect("a road ahead");
        assert_eq!(road.line.len(), 31);
        let (end, heading) = road.end.expect("an end");
        assert!((end.x - 32.0).abs() < 1e-9 && heading.abs() < 1e-9);
        assert_eq!(road.stops.len(), 1);
        assert_eq!(road.stops[0].1, "goal");
        assert!(road.top_speed() > 2.4);
    }

    #[test]
    fn the_first_drain_has_the_squiggly_line() {
        let mut feed = PretendFeed::new(10);
        let updates = feed.drain(Instant::now());
        assert!(updates
            .iter()
            .any(|u| matches!(u, Update::Trace(t) if t.len() == 21)));
        assert!(updates.iter().any(|u| matches!(u, Update::PlanAhead(_))));
        assert!(updates.iter().any(|u| matches!(u, Update::StopPoint(_))));
        assert!(updates.iter().any(|u| matches!(u, Update::Ahead(_))));
    }

    #[test]
    fn the_pretend_trace_bends_left_mid_loop() {
        let trace = ahead_at(6.0)
            .into_iter()
            .find_map(|u| match u {
                Update::Trace(t) => Some(t),
                _ => None,
            })
            .expect("a trace");
        assert!(trace.last().is_some_and(|p| p.1 > 0.05));
    }

    #[test]
    fn the_pretend_things_are_named_and_ahead_of_the_car() {
        let car = car_at(5.0);
        let things = things_at(5.0, car);
        let named =
            hud_core::things::named_ahead(&things, (car.position.x, car.position.y, car.yaw), 30.0);
        let labels: Vec<_> = named.iter().map(|(t, _)| t.label()).collect();
        assert_eq!(labels, vec!["person 3", "stop sign 7", "car 9"]);
        assert!(things.iter().any(|t| !t.is_named()));
    }

    #[test]
    fn a_wall_is_occupied_on_the_grid() {
        let grid = lot_grid(|d| if d < CELL { 100 } else { 0 }).expect("a good grid");
        assert_eq!(
            crate::feed::refusal(Point::new(25.0, 0.0), Some(&grid)),
            Some("that spot is occupied")
        );
        assert_eq!(
            crate::feed::refusal(Point::new(10.0, 12.0), Some(&grid)),
            None
        );
    }

    #[test]
    fn the_log_has_every_level() {
        let levels: Vec<Level> = (0..9).map(|n| log_line(n).0).collect();
        assert!(levels.contains(&Level::Info));
        assert!(levels.contains(&Level::Warn));
        assert!(levels.contains(&Level::Error));
    }
}

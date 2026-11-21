//! What the window is told, wherever it comes from: pretend data or ROS. The window only
//! sees [`Feed`], so either is a drop-in for the other. No Bevy, no ROS.

use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use debug_core::{
    cubes::CubeScan,
    ndt::{Fit, NdtStatus},
    swap,
};
use hud_core::{
    ahead::{StopMark, Summary},
    gate::{self, Verdict},
    health::Health,
    origin::{Announcement, Origins},
    picture::Picture,
    things::Thing,
    vehicle::VehicleStatus,
};
use map_core::{
    geom::{Point, Pose},
    grid::Grid,
};
use serde::Deserialize;

/// A topic older than this counts as silent, as the HUD's own limit.
pub const STALE_AFTER: Duration = Duration::from_secs(2);

/// How bad a log line is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Info,
    Warn,
    Error,
}

impl Level {
    /// From `rcl_interfaces/Log`'s level: 10 debug, 20 info, 30 warn, 40 error, 50 fatal.
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    pub fn from_ros(level: u8) -> Self {
        match level {
            0..=29 => Self::Info,
            30..=39 => Self::Warn,
            _ => Self::Error,
        }
    }
}

/// One piece of news.
#[derive(Debug, Clone)]
pub enum Update {
    /// The car's cubes, in odom.
    Cubes(CubeScan),
    /// The road's cubes, in odom.
    Road(CubeScan),
    CarInOdom(Pose),
    CarInMap(Pose),
    Picture(Picture),
    Ndt(NdtStatus),
    NdtPose(Pose),
    Log(Level, String),
    Plan(Vec<Point>),
    Map(Grid),
    Costmap(Grid),
    Origin(Announcement),
    Vehicle(VehicleStatus),
    Gate(String),
    Watchdog(String),
    /// Who is driving: the command mux's state's line (aav_mux_state), "keyboard (90)".
    Driving(String),
    /// The squiggly line (`hud_core::ahead`), all in base_link: ego's trace, the bold line.
    Trace(Vec<(f64, f64)>),
    /// What the planner wants: the faint line.
    PlanAhead(Vec<(f64, f64)>),
    /// Where a full brake now stops the car.
    StopPoint(StopMark),
    /// The road ahead (`/world/road_ahead`), on the map.
    RoadAhead(RoadAhead),
    /// The squiggly line's numbers: the gap, the stop, the room.
    Ahead(Summary),
    /// The things near the car (`/world/things`, odom), named by the camera
    /// (`hud_core::things`: kind, words, colour).
    Things(Vec<Thing>),
    /// The street route (`/drivers/nav/route/path`, map): the zoomed-out level's line.
    Route(Vec<Point>),
}

/// The road ahead (`/world/road_ahead`, the whole-car design), on the map: where the car
/// may go, how fast, and the end it may not pass.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RoadAhead {
    /// The line, with the speed limit at each point (m/s).
    pub line: Vec<(Point, f64)>,
    /// The end: where it is, and the line's heading there (rad).
    pub end: Option<(Point, f64)>,
    /// Each stop: where, and why.
    pub stops: Vec<(Point, String)>,
}

impl RoadAhead {
    /// From the message's parts: the points, their headings and limits, the spacing
    /// (m), the end and each stop as metres along the line.
    pub fn from_parts(
        points: &[(f64, f64, f64, f64)],
        spacing: f64,
        end: f64,
        stops: &[(f64, String)],
    ) -> Self {
        let line = points
            .iter()
            .map(|&(x, y, _, limit)| (Point::new(x, y), limit))
            .collect();
        let at = |along: f64| along_line(points, spacing, along);
        Self {
            line,
            end: at(end),
            stops: stops
                .iter()
                .filter_map(|(along, why)| at(*along).map(|(p, _)| (p, why.clone())))
                .collect(),
        }
    }

    /// The fastest limit on the strip (m/s), for the colours; 0 for none.
    pub fn top_speed(&self) -> f64 {
        self.line.iter().map(|&(_, v)| v).fold(0.0, f64::max)
    }
}

/// How fast a point is, 0 (stop) to 1 (the strip's top speed): the colour's share.
pub fn speed_share(limit: f64, top: f64) -> f32 {
    if top > 0.0 && limit.is_finite() {
        (limit / top).clamp(0.0, 1.0) as f32
    } else {
        0.0
    }
}

/// The point `along` metres down the line, and the heading there; `None` with no line.
fn along_line(points: &[(f64, f64, f64, f64)], spacing: f64, along: f64) -> Option<(Point, f64)> {
    let last = points.len().checked_sub(1)?;
    if spacing <= 0.0 || !along.is_finite() {
        return None;
    }
    let step = (along.max(0.0) / spacing).floor();
    let i = if step >= last as f64 {
        last
    } else {
        step as usize
    };
    let (x, y, heading, _) = points[i];
    if i == last {
        return Some((Point::new(x, y), heading));
    }
    let (nx, ny, _, _) = points[i + 1];
    let t = (along.max(0.0) - i as f64 * spacing) / spacing;
    Some((Point::new(x + t * (nx - x), y + t * (ny - y)), heading))
}

/// A base_link point, placed with the car's pose in odom or on the map.
pub fn from_base(car: Pose, (x, y): (f64, f64)) -> Point {
    let (sin, cos) = car.yaw.sin_cos();
    Point::new(
        car.position.x + x * cos - y * sin,
        car.position.y + x * sin + y * cos,
    )
}

/// Where the news comes from, and where the window's two commands go.
pub trait Feed: Send + Sync {
    /// A short label for the window: "ROS" or "PRETEND".
    fn label(&self) -> &'static str;

    /// Everything that arrived since the last call. Never blocks.
    fn drain(&mut self, now: Instant) -> Vec<Update>;

    /// Whether this run may command anything at all: `Err` says why not.
    fn may_command(&self) -> Result<(), String>;

    /// Asks Nav2 to go there. The caller has passed [`command_allowed`].
    fn send_goal(&mut self, goal: Pose) -> Result<(), String>;

    /// Sets the e-stop. Only ever sets it: clearing is `pixi run clear estop`.
    fn press_estop(&mut self) -> Result<(), String>;

    /// Cancels the navigator's goal (TODO 114): Nav2's, or ours under the same action name.
    fn cancel_goal(&mut self) -> Result<(), String> {
        Err("this feed can't cancel a goal".to_owned())
    }

    /// Swaps `aav_track`'s follower while it drives (TODO 114): `tracker := name` on the
    /// track node, which refuses unless started with `live_swap:=true` (the simulator).
    fn swap_tracker(&mut self, name: &'static str) -> Result<(), String> {
        Err(format!("this feed can't swap to {name}"))
    }

    /// A frame reached the screen. A live feed's heartbeat says so, while frames keep coming.
    fn frame_drawn(&self) {}
}

/// The last gate before a goal or e-stop leaves the window: this run may command, and every
/// sensor says SIM (`debug_core::swap`: also refuses BENCH, a real car and "not heard yet").
pub fn command_allowed(may_command: Result<(), String>, origins: &Origins) -> Result<(), String> {
    may_command?;
    swap::allowed(origins).map_err(|refused| refused.to_string())
}

/// Why a goal can't go to `at`, in words, or `None` if it can.
pub fn refusal(at: Point, map: Option<&Grid>) -> Option<&'static str> {
    let Some(map) = map else {
        return Some("no map heard yet");
    };
    if !map.info.contains(at) {
        return Some("that spot is off the map");
    }
    let (column, row) = cell_at(map, at)?;
    match map.cell(column, row) {
        Some(map_core::grid::Cell::Occupancy(p)) if p < 50 => None,
        Some(map_core::grid::Cell::Occupancy(_)) => Some("that spot is occupied"),
        _ => Some("that spot is unknown on the map"),
    }
}

/// The cell under a map point, for a grid that isn't turned.
fn cell_at(map: &Grid, at: Point) -> Option<(u32, u32)> {
    let resolution = map.info.resolution;
    let column = ((at.x - map.info.origin.position.x) / resolution).floor();
    let row = ((at.y - map.info.origin.position.y) / resolution).floor();
    (column >= 0.0 && row >= 0.0).then_some((column as u32, row as u32))
}

/// The goal's pose: at the spot, facing away from the car.
pub fn goal_pose(car: Pose, at: Point) -> Pose {
    Pose::new(
        at.x,
        at.y,
        (at.y - car.position.y).atan2(at.x - car.position.x),
    )
}

/// A point in odom, on the map: from where the car is in each.
pub fn odom_to_map(point: [f32; 2], car_odom: Pose, car_map: Pose) -> Point {
    let (dx, dy) = (
        f64::from(point[0]) - car_odom.position.x,
        f64::from(point[1]) - car_odom.position.y,
    );
    let turn = car_map.yaw - car_odom.yaw;
    let (sin, cos) = turn.sin_cos();
    Point::new(
        car_map.position.x + dx * cos - dy * sin,
        car_map.position.y + dx * sin + dy * cos,
    )
}

/// What the NDT line's colour says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NdtLight {
    Good,
    Poor,
    Lost,
    Off,
}

/// NDT in one line.
#[derive(Clone, Debug, PartialEq)]
pub struct NdtLine {
    pub state: NdtLight,
    pub words: String,
}

/// Localization's state for the debug view: its word, its numbers and how far from the EKF.
pub fn ndt_line(status: Option<&NdtStatus>, pose: Option<Pose>, ekf: Pose) -> NdtLine {
    let Some(status) = status else {
        return NdtLine {
            state: NdtLight::Off,
            words: "NDT off: ndt:=true".to_owned(),
        };
    };
    let state = match (status.headline(), status.fit) {
        ("LOST", _) | (_, Fit::Lost) => NdtLight::Lost,
        (_, Fit::Poor) => NdtLight::Poor,
        _ => NdtLight::Good,
    };
    let mut words = format!("NDT {}   {}", status.headline(), status.numbers());
    if let Some(pose) = pose {
        words.push_str(&format!(
            "   {:.2} m from the EKF, newest of each",
            pose.position.distance(ekf.position).0
        ));
    }
    NdtLine { state, words }
}

/// How the live cubes sit on NDT's map, this scan.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapMatch {
    /// One for each of the map's cells: a live cube is in it now.
    pub lit: Vec<bool>,
    /// Each live cube, by number: it's on none of the map's cells.
    pub off_map: Vec<usize>,
}

impl MapMatch {
    /// The share of live cubes on the map, 0 to 1; `None` with no cubes.
    pub fn share_on_map(&self, cubes: usize) -> Option<f64> {
        (cubes > 0).then(|| 1.0 - self.off_map.len() as f64 / cubes as f64)
    }
}

/// NDT's cells, ready to be looked up: a campus has too many to index again for every scan.
#[derive(Clone, Debug)]
pub struct MapIndex {
    size: f32,
    cells: HashMap<[i32; 3], usize>,
    count: usize,
    centres: Vec<[f32; 3]>,
}

impl MapIndex {
    pub fn new(centres: &[[f32; 3]], size: f32) -> Self {
        let cells = centres
            .iter()
            .enumerate()
            .map(|(i, c)| (c.map(|v| (v / size).floor() as i32), i))
            .collect();
        Self {
            size,
            cells,
            count: centres.len(),
            centres: centres.to_vec(),
        }
    }

    /// Which cells the live cubes light, and which cubes are near no cell. A wall's face can
    /// fall on the edge of its cell, so a cube counts if a cell's centre is within one cell's
    /// edge of it. Past that, the world has something the map doesn't.
    /// Cubes below `floor` count as neither: they're the ground.
    pub fn match_cubes(&self, cubes: &[[f32; 3]], floor: f32) -> MapMatch {
        let mut result = MapMatch {
            lit: vec![false; self.count],
            off_map: Vec::new(),
        };
        let above = cubes.iter().enumerate().filter(|(_, c)| c[2] >= floor);
        for (number, cube) in above {
            match self.nearest(cube) {
                Some(cell) => result.lit[cell] = true,
                None => result.off_map.push(number),
            }
        }
        result
    }

    fn nearest(&self, cube: &[f32; 3]) -> Option<usize> {
        let key = cube.map(|v| (v / self.size).floor() as i32);
        // The cube's own cell is the usual answer, so look there before its neighbours.
        if let Some(&cell) = self.cells.get(&key) {
            return Some(cell);
        }
        let reach = self.size * self.size;
        let mut best = None;
        let mut best_distance = reach;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let near = [key[0] + dx, key[1] + dy, key[2] + dz];
                    if let Some(&cell) = self.cells.get(&near) {
                        let c = self.centres[cell];
                        let distance = (c[0] - cube[0]).powi(2)
                            + (c[1] - cube[1]).powi(2)
                            + (c[2] - cube[2]).powi(2);
                        if distance <= best_distance {
                            best_distance = distance;
                            best = Some(cell);
                        }
                    }
                }
            }
        }
        best
    }
}

/// The lidar's place on the car, from `vehicle.yaml`: the only place car measurements live.
pub fn lidar_mount() -> Result<[f32; 3], String> {
    #[derive(Deserialize)]
    struct File {
        sensors: Sensors,
    }
    #[derive(Deserialize)]
    struct Sensors {
        lidar_link: Mount,
    }
    #[derive(Deserialize)]
    struct Mount {
        xyz: [f32; 3],
    }
    let text = include_str!("../../../bringup/aav_description/config/vehicle.yaml");
    serde_yaml_ng::from_str::<File>(text)
        .map(|file| file.sensors.lidar_link.xyz)
        .map_err(|e| e.to_string())
}

/// Where the driver's view looks from and to, in odom. `mount` is the eyes' place on the car,
/// and `look` how far the head is turned left and tilted up, radians.
pub fn driver_eye(car: Pose, mount: [f32; 3], look: [f32; 2]) -> ([f32; 3], [f32; 3]) {
    let yaw = car.yaw as f32;
    let (sin, cos) = yaw.sin_cos();
    let from = [
        car.position.x as f32 + mount[0] * cos - mount[1] * sin,
        car.position.y as f32 + mount[0] * sin + mount[1] * cos,
        mount[2],
    ];
    let heading = yaw + look[0];
    let (head_sin, head_cos) = heading.sin_cos();
    let (tilt_sin, tilt_cos) = look[1].sin_cos();
    let target = [
        from[0] + head_cos * tilt_cos,
        from[1] + head_sin * tilt_cos,
        from[2] + tilt_sin,
    ];
    (from, target)
}

/// A text topic and when it was last heard.
#[derive(Debug, Clone, Default)]
pub struct Heard {
    text: Option<String>,
    at: Option<Instant>,
}

impl Heard {
    pub fn set(&mut self, text: String, now: Instant) {
        self.text = Some(text);
        self.at = Some(now);
    }

    /// The text, while it is fresh.
    pub fn fresh(&self, now: Instant) -> Option<&str> {
        self.health(now)
            .is_ok()
            .then_some(self.text.as_deref())
            .flatten()
    }

    fn health(&self, now: Instant) -> Health {
        match self.at {
            None => Health::Never,
            Some(at) => {
                let age = now.saturating_duration_since(at);
                if age > STALE_AFTER {
                    Health::Stale(age)
                } else {
                    Health::Ok(age)
                }
            }
        }
    }
}

/// GO or STOP, by the HUD's own logic: this window may never disagree with the car's screen.
pub fn verdict(estop_on: bool, gate_state: &Heard, watchdog: &Heard, now: Instant) -> Verdict {
    gate::verdict(
        estop_on,
        (gate_state.health(now), gate_state.text.as_deref()),
        (watchdog.health(now), watchdog.text.as_deref()),
    )
}

#[cfg(test)]
mod tests {
    use hud_core::{
        gate::Light,
        origin::{Entry, Origin},
    };
    use map_core::grid::GridInfo;

    use super::*;

    fn map() -> Grid {
        let info = GridInfo {
            width: 2,
            height: 1,
            resolution: 1.0,
            origin: Pose::new(0.0, 0.0, 0.0),
        };
        Grid::new(info, "map".into(), vec![0, 100]).expect("a good grid")
    }

    fn origins(origin: Origin) -> Origins {
        let mut origins = Origins::default();
        origins.apply(Announcement {
            announcer: "test".into(),
            entries: vec![Entry {
                name: "gps".into(),
                topic: "/gps".into(),
                origin,
            }],
        });
        origins
    }

    #[test]
    fn the_road_ahead_places_its_end_and_stops_on_the_line() {
        let points: Vec<_> = (0..5)
            .map(|i| (f64::from(i), 0.0, 0.0, 1.0 - 0.25 * f64::from(i)))
            .collect();
        let road = RoadAhead::from_parts(&points, 1.0, 2.5, &[(2.5, "sign".into())]);
        assert_eq!(road.end, Some((Point::new(2.5, 0.0), 0.0)));
        assert_eq!(road.stops, vec![(Point::new(2.5, 0.0), "sign".to_string())]);
        assert_eq!(road.top_speed(), 1.0);
        assert_eq!(speed_share(0.5, road.top_speed()), 0.5);
        assert_eq!(speed_share(0.5, 0.0), 0.0);
        // Past the last point: the last point.
        let far = RoadAhead::from_parts(&points, 1.0, 9.0, &[]);
        assert_eq!(far.end, Some((Point::new(4.0, 0.0), 0.0)));
    }

    #[test]
    fn an_empty_road_ahead_has_no_end() {
        let road = RoadAhead::from_parts(&[], 0.5, 0.0, &[]);
        assert_eq!(road, RoadAhead::default());
    }

    #[test]
    fn a_free_cell_takes_a_goal() {
        assert_eq!(refusal(Point::new(0.5, 0.5), Some(&map())), None);
    }

    #[test]
    fn goals_are_refused_in_words() {
        assert_eq!(
            refusal(Point::new(1.5, 0.5), Some(&map())),
            Some("that spot is occupied")
        );
        assert_eq!(
            refusal(Point::new(9.0, 0.5), Some(&map())),
            Some("that spot is off the map")
        );
        assert_eq!(
            refusal(Point::new(0.5, 0.5), None),
            Some("no map heard yet")
        );
    }

    #[test]
    fn a_run_without_permission_commands_nothing() {
        let refused = command_allowed(Err("watch only".into()), &origins(Origin::Sim));
        assert_eq!(refused, Err("watch only".to_owned()));
    }

    #[test]
    fn only_all_sim_may_command() {
        assert!(command_allowed(Ok(()), &origins(Origin::Sim)).is_ok());
        assert!(command_allowed(Ok(()), &origins(Origin::Real)).is_err());
        assert!(command_allowed(Ok(()), &origins(Origin::Bench)).is_err());
        assert!(command_allowed(Ok(()), &Origins::default()).is_err());
    }

    #[test]
    fn nothing_heard_is_never_go() {
        let now = Instant::now();
        let v = verdict(false, &Heard::default(), &Heard::default(), now);
        assert_ne!(v.light, Light::Go);
    }

    #[test]
    fn the_estop_is_always_stop() {
        let now = Instant::now();
        let v = verdict(true, &Heard::default(), &Heard::default(), now);
        assert_eq!(v.light, Light::Stop);
    }

    #[test]
    fn a_goal_faces_away_from_the_car() {
        let goal = goal_pose(Pose::new(0.0, 0.0, 1.0), Point::new(0.0, 5.0));
        assert!((goal.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn odom_points_land_on_the_map() {
        let car_odom = Pose::new(1.0, 0.0, 0.0);
        let car_map = Pose::new(10.0, 10.0, std::f64::consts::FRAC_PI_2);
        let p = odom_to_map([2.0, 0.0], car_odom, car_map);
        assert!((p.x - 10.0).abs() < 1e-6 && (p.y - 11.0).abs() < 1e-6);
    }

    #[test]
    fn base_link_points_follow_the_car() {
        let car = Pose::new(10.0, 5.0, std::f64::consts::FRAC_PI_2);
        let ahead = from_base(car, (2.0, 0.0));
        assert!((ahead.x - 10.0).abs() < 1e-9 && (ahead.y - 7.0).abs() < 1e-9);
        let left = from_base(car, (0.0, 1.0));
        assert!((left.x - 9.0).abs() < 1e-9 && (left.y - 5.0).abs() < 1e-9);
    }

    #[test]
    fn the_eyes_are_the_lidars_from_the_cars_measurements() {
        let mount = lidar_mount().expect("vehicle.yaml has the lidar's mount");
        assert!(mount[2] > 1.0, "the lidar is on the roof: {mount:?}");
    }

    #[test]
    fn the_driver_looks_where_the_car_points() {
        let car = Pose::new(10.0, 0.0, std::f64::consts::FRAC_PI_2);
        let (from, target) = driver_eye(car, [1.0, 0.0, 1.5], [0.0, 0.0]);
        assert!((from[0] - 10.0).abs() < 1e-5 && (from[1] - 1.0).abs() < 1e-5);
        assert_eq!(from[2], 1.5);
        assert!(target[1] > from[1] && (target[0] - from[0]).abs() < 1e-5);
        assert!((target[2] - from[2]).abs() < 1e-5);
    }

    #[test]
    fn turning_the_head_left_looks_left() {
        let car = Pose::new(0.0, 0.0, 0.0);
        let (from, target) = driver_eye(car, [0.0, 0.0, 1.5], [std::f32::consts::FRAC_PI_2, 0.0]);
        assert!(target[1] > from[1] && (target[0] - from[0]).abs() < 1e-5);
    }

    #[test]
    fn a_live_cube_lights_its_map_cell() {
        let centres = [[0.5, 0.5, 0.5], [5.5, 0.5, 0.5]];
        let found = MapIndex::new(&centres, 1.0).match_cubes(&[[0.2, 0.9, 0.1]], 0.0);
        assert_eq!(found.lit, vec![true, false]);
        assert!(found.off_map.is_empty());
    }

    #[test]
    fn a_cube_off_the_map_is_listed() {
        let centres = [[0.5, 0.5, 0.5]];
        let found =
            MapIndex::new(&centres, 1.0).match_cubes(&[[0.2, 0.2, 0.2], [9.0, 9.0, 0.2]], 0.0);
        assert_eq!(found.off_map, vec![1]);
        assert_eq!(found.share_on_map(2), Some(0.5));
    }

    #[test]
    fn a_cube_just_over_a_cells_edge_still_lights_it() {
        let centres = [[0.5, 0.5, 0.5]];
        let found = MapIndex::new(&centres, 1.0).match_cubes(&[[1.1, 0.5, 0.5]], 0.0);
        assert_eq!(found.lit, vec![true]);
        assert!(found.off_map.is_empty());
    }

    #[test]
    fn a_cube_past_one_cell_away_is_off_the_map() {
        let centres = [[0.5, 0.5, 0.5]];
        let found = MapIndex::new(&centres, 1.0).match_cubes(&[[1.9, 0.5, 0.5]], 0.0);
        assert_eq!(found.lit, vec![false]);
        assert_eq!(found.off_map, vec![0]);
    }

    #[test]
    fn the_ground_is_neither_on_the_map_nor_new() {
        let centres = [[0.5, 0.5, 0.5]];
        let found = MapIndex::new(&centres, 1.0).match_cubes(&[[9.0, 9.0, 0.2]], 0.45);
        assert!(found.off_map.is_empty());
        assert_eq!(found.lit, vec![false]);
    }

    #[test]
    fn no_cubes_has_no_share() {
        assert_eq!(MapMatch::default().share_on_map(0), None);
    }

    #[test]
    fn no_ndt_says_how_to_turn_it_on() {
        let line = ndt_line(None, None, Pose::new(0.0, 0.0, 0.0));
        assert_eq!(line.state, NdtLight::Off);
        assert!(line.words.contains("ndt:=true"));
    }

    #[test]
    fn lost_is_lost_and_the_gap_is_shown() {
        let lost = NdtStatus::from_message(2, "LOST: no fit", []);
        let line = ndt_line(Some(&lost), None, Pose::new(0.0, 0.0, 0.0));
        assert_eq!(line.state, NdtLight::Lost);
        let good = NdtStatus::from_message(0, "TRACKING", []);
        let line = ndt_line(
            Some(&good),
            Some(Pose::new(3.0, 4.0, 0.0)),
            Pose::new(0.0, 0.0, 0.0),
        );
        assert_eq!(line.state, NdtLight::Good);
        assert!(line.words.contains("5.00 m from the EKF"), "{}", line.words);
    }

    #[test]
    fn ros_levels_map_to_ours() {
        assert_eq!(Level::from_ros(20), Level::Info);
        assert_eq!(Level::from_ros(30), Level::Warn);
        assert_eq!(Level::from_ros(50), Level::Error);
    }
}

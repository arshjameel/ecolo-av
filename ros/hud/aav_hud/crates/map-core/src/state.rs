//! Everything the map window knows, as one value: the map, the car, the plan, the goal,
//! and when each was last heard. The ROS node and the demo send [`Update`]s; the screen
//! draws a [`MapState`].

use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use hud_core::{
    health::{Health, LastSeen},
    origin::{Announcement, Origins},
    units::MetresPerSecond,
    watchdog::{self, Alarm},
};

use crate::{
    config::{MapConfig, Topic},
    geom::{Metres, Point, Pose},
    grid::Grid,
    route::{Progress, Route},
    trip::Trip,
};

/// The frame everything is drawn in when no map has said otherwise.
pub const DEFAULT_FRAME: &str = "map";

/// The car, from one odometry message.
#[derive(Debug, Clone, PartialEq)]
pub struct Car {
    pub pose: Pose,
    /// Forward speed; negative in reverse.
    pub speed: MetresPerSecond,
    /// The frame the pose is in: the map's, normally.
    pub frame: String,
}

/// One piece of news.
#[derive(Debug, Clone, PartialEq)]
pub enum Update {
    Map(Grid),
    /// A map message that can't be drawn, and why.
    BadMap(String),
    Car(Car),
    Plan {
        route: Route,
        frame: String,
    },
    Goal {
        pose: Pose,
        frame: String,
    },
    Source(Announcement),
    /// The sensor watchdog's verdict: "OK", or "STOP: ..." / "WARN: ...".
    Watchdog(String),
    /// street_router's street route: drawn, but not what Nav2 drives (that's the plan).
    StreetRoute {
        route: Route,
        frame: String,
    },
}

/// How far is left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Remaining {
    /// Along Nav2's plan.
    AlongPlan(Metres),
    /// No plan for this goal yet: as the crow flies.
    StraightLine(Metres),
}

impl Remaining {
    pub fn metres(self) -> Metres {
        match self {
            Self::AlongPlan(m) | Self::StraightLine(m) => m,
        }
    }
}

/// Everything the screen draws.
#[derive(Debug, Clone)]
pub struct MapState {
    pub config: MapConfig,
    pub grid: Option<Grid>,
    grid_changed: bool,
    /// Why the last map couldn't be drawn.
    pub map_problem: Option<String>,
    pub car: Option<Car>,
    pub route: Route,
    plan_frame: String,
    /// street_router's street route, and its frame.
    pub street_route: Route,
    street_route_frame: String,
    plan_at: Option<Instant>,
    /// RViz's goal, and when it came.
    pub goal_pose: Option<Pose>,
    goal_frame: String,
    goal_at: Option<Instant>,
    pub origins: Origins,
    /// The sensor watchdog's last verdict.
    pub watchdog: Option<String>,
    seen: HashMap<Topic, LastSeen>,
}

impl MapState {
    pub fn new(config: MapConfig) -> Self {
        Self {
            config,
            grid: None,
            grid_changed: false,
            map_problem: None,
            car: None,
            route: Route::default(),
            plan_frame: String::new(),
            street_route: Route::default(),
            street_route_frame: String::new(),
            plan_at: None,
            goal_pose: None,
            goal_frame: String::new(),
            goal_at: None,
            origins: Origins::default(),
            watchdog: None,
            seen: HashMap::new(),
        }
    }

    /// Takes in one piece of news that arrived at `now`.
    pub fn apply(&mut self, update: Update, now: Instant) {
        let topic = match &update {
            Update::Map(_) | Update::BadMap(_) => Topic::Map,
            Update::Car(_) => Topic::Odom,
            Update::Plan { .. } => Topic::Plan,
            Update::Goal { .. } => Topic::Goal,
            Update::Source(_) => Topic::Source,
            Update::Watchdog(_) => Topic::Watchdog,
            Update::StreetRoute { .. } => Topic::StreetRoute,
        };
        self.seen.entry(topic).or_default().heard(now);
        match update {
            Update::Map(g) => {
                self.grid = Some(g);
                self.grid_changed = true;
                self.map_problem = None;
            }
            Update::BadMap(why) => self.map_problem = Some(why),
            Update::Car(c) => self.car = Some(c),
            Update::Plan { route, frame } => {
                self.route = route;
                self.plan_frame = frame;
                self.plan_at = Some(now);
            }
            Update::Goal { pose, frame } => {
                self.goal_pose = Some(pose);
                self.goal_frame = frame;
                self.goal_at = Some(now);
            }
            Update::Source(a) => self.origins.apply(a),
            Update::Watchdog(w) => self.watchdog = Some(w),
            Update::StreetRoute { route, frame } => {
                self.street_route = route;
                self.street_route_frame = frame;
            }
        }
    }

    /// Whether a new map came since the last call: the screen makes its picture again.
    pub fn take_grid_change(&mut self) -> bool {
        std::mem::take(&mut self.grid_changed)
    }

    /// OK / STALE / NEVER for one topic. The map, the goal and the source are sent once
    /// (latched, or by a person), so they never go stale.
    pub fn health(&self, topic: Topic, now: Instant) -> Health {
        let limit = match topic {
            Topic::Odom | Topic::Watchdog => self.config.stale_after(),
            Topic::Plan => self.plan_limit(),
            Topic::Map | Topic::Goal | Topic::Source | Topic::StreetRoute => Duration::MAX,
        };
        self.seen
            .get(&topic)
            .copied()
            .unwrap_or_default()
            .health(now, limit)
    }

    /// How old a plan may be. Nav2 replans every second, except on the final approach
    /// (the car within `final_approach` of the plan's end), where its behaviour trees
    /// keep the path they have (TODO 17).
    fn plan_limit(&self) -> Duration {
        let near = match (self.car_position(), self.route.goal()) {
            (Some(car), Some(end)) => car.distance(end) <= Metres(self.config.final_approach),
            _ => false,
        };
        if near {
            self.config.plan_kept_for()
        } else {
            self.config.plan_stale_after()
        }
    }

    /// What the sensor watchdog says is wrong, for a banner; None when all is well. The
    /// same words as on the HUD ([`watchdog::alarm`]).
    pub fn sensor_alarm(&self, now: Instant) -> Option<Alarm> {
        watchdog::alarm(self.health(Topic::Watchdog, now), self.watchdog.as_deref())
    }

    /// The car's position, if any has come.
    pub fn car_position(&self) -> Option<Point> {
        self.car.as_ref().map(|c| c.pose.position)
    }

    /// Whether RViz's goal is newer than the plan: a new goal, not planned yet.
    fn goal_is_newer(&self) -> bool {
        match (self.goal_at, self.plan_at) {
            (Some(g), Some(p)) => g > p,
            (Some(_), None) => true,
            (None, _) => false,
        }
    }

    /// Where the car is going: the plan's end, or RViz's newer goal.
    pub fn goal(&self) -> Option<Point> {
        if self.goal_is_newer() {
            self.goal_pose.map(|p| p.position)
        } else {
            self.route.goal()
        }
    }

    /// Where the car is along the plan.
    pub fn progress(&self) -> Option<Progress> {
        self.route.progress(self.car_position()?)
    }

    /// How far is left: along the plan, or in a straight line to a goal with no plan yet.
    pub fn remaining(&self) -> Option<Remaining> {
        let car = self.car_position()?;
        if self.goal_is_newer() {
            let goal = self.goal_pose?.position;
            return Some(Remaining::StraightLine(car.distance(goal)));
        }
        self.progress().map(|p| Remaining::AlongPlan(p.remaining))
    }

    /// The trip's status at `now`.
    pub fn trip(&self, now: Instant) -> Trip {
        let plan = if self.goal_is_newer() {
            Health::Never
        } else {
            self.health(Topic::Plan, now)
        };
        Trip::of(
            self.goal(),
            self.car_position(),
            plan,
            Metres(self.config.arrive_within),
        )
    }

    /// The frame the map is in.
    pub fn frame(&self) -> &str {
        self.grid
            .as_ref()
            .map_or(DEFAULT_FRAME, |g| g.frame.as_str())
    }

    /// What's drawn in a frame other than the map's: it would be in the wrong place.
    pub fn frame_problems(&self) -> Vec<String> {
        let map = self.frame();
        let car = self.car.as_ref().map(|c| ("the car", c.frame.as_str()));
        let plan = (!self.route.is_empty()).then_some(("the plan", self.plan_frame.as_str()));
        let goal = self
            .goal_pose
            .map(|_| ("the goal", self.goal_frame.as_str()));
        let street = (!self.street_route.is_empty())
            .then_some(("the street route", self.street_route_frame.as_str()));
        [car, plan, goal, street]
            .into_iter()
            .flatten()
            .filter(|(_, frame)| !frame.is_empty() && *frame != map)
            .map(|(what, frame)| format!("{what} is in {frame}, not {map}"))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::Pose,
        grid::{Grid, GridInfo},
    };

    #[test]
    fn the_watchdog_banner_is_the_huds() {
        let start = Instant::now();
        let mut s = MapState::new(MapConfig::default());
        assert_eq!(s.sensor_alarm(start), None); // no watchdog in this run
        s.apply(Update::Watchdog("OK".into()), start);
        assert_eq!(s.sensor_alarm(start), None);
        let stop = "STOP: gps JUMP (jumped 20.0 m in 0.20 s)";
        s.apply(Update::Watchdog(stop.into()), start);
        assert_eq!(s.sensor_alarm(start), Some(Alarm::Stop(stop.into())));
        let later = start + s.config.stale_after() + Duration::from_millis(100);
        assert_eq!(
            s.sensor_alarm(later),
            Some(Alarm::Stop("SENSOR WATCHDOG SILENT".into()))
        );
    }

    fn car_at(x: f64, y: f64) -> Update {
        Update::Car(Car {
            pose: Pose::new(x, y, 0.0),
            speed: MetresPerSecond(2.0),
            frame: "map".into(),
        })
    }

    fn plan(points: &[(f64, f64)]) -> Update {
        Update::Plan {
            route: Route::new(points.iter().map(|&(x, y)| Point::new(x, y))),
            frame: "map".into(),
        }
    }

    #[test]
    fn nothing_heard() {
        let s = MapState::new(MapConfig::default());
        let now = Instant::now();
        assert_eq!(s.trip(now), Trip::NoGoal);
        assert_eq!(s.health(Topic::Map, now), Health::Never);
        assert_eq!(s.remaining(), None);
        assert!(s.frame_problems().is_empty());
    }

    #[test]
    fn driving_along_a_plan_then_arriving() {
        let mut s = MapState::new(MapConfig::default());
        let t0 = Instant::now();
        s.apply(car_at(0.0, 0.0), t0);
        s.apply(plan(&[(0.0, 0.0), (20.0, 0.0)]), t0);
        assert_eq!(s.trip(t0), Trip::Driving);
        assert_eq!(s.remaining(), Some(Remaining::AlongPlan(Metres(20.0))));
        s.apply(car_at(19.5, 0.0), t0 + Duration::from_secs(10));
        // The plan is old now, but the car is there.
        assert_eq!(s.trip(t0 + Duration::from_secs(10)), Trip::Arrived);
    }

    #[test]
    fn a_plan_that_stops_coming_says_so() {
        let mut s = MapState::new(MapConfig::default());
        let t0 = Instant::now();
        s.apply(car_at(0.0, 0.0), t0);
        s.apply(plan(&[(0.0, 0.0), (20.0, 0.0)]), t0);
        assert_eq!(s.trip(t0 + Duration::from_secs(10)), Trip::PlanStopped);
        assert!(!s.health(Topic::Plan, t0 + Duration::from_secs(10)).is_ok());
        // On the final approach Nav2 keeps its path (TODO 17): an older plan is fine there.
        let config = MapConfig::default();
        let kept = config.plan_kept_for();
        let mut s = MapState::new(config);
        s.apply(car_at(15.0, 0.0), t0);
        s.apply(plan(&[(0.0, 0.0), (20.0, 0.0)]), t0);
        assert_eq!(s.trip(t0 + Duration::from_secs(10)), Trip::Driving);
        assert!(s.health(Topic::Plan, t0 + Duration::from_secs(10)).is_ok());
        assert_eq!(
            s.trip(t0 + kept + Duration::from_secs(1)),
            Trip::PlanStopped
        );
        // The map never goes stale: it's sent once.
        let info = GridInfo {
            width: 1,
            height: 1,
            resolution: 1.0,
            origin: Pose::default(),
        };
        let g = Grid::new(info, "map".into(), vec![0]).expect("valid");
        s.apply(Update::Map(g), t0);
        assert!(s.take_grid_change());
        assert!(!s.take_grid_change());
        assert!(s.health(Topic::Map, t0 + Duration::from_secs(3600)).is_ok());
    }

    #[test]
    fn a_new_goal_from_rviz_wins_until_planned() {
        let mut s = MapState::new(MapConfig::default());
        let t0 = Instant::now();
        s.apply(car_at(0.0, 0.0), t0);
        s.apply(plan(&[(0.0, 0.0), (20.0, 0.0)]), t0);
        let later = t0 + Duration::from_millis(500);
        s.apply(
            Update::Goal {
                pose: Pose::new(0.0, 30.0, 0.0),
                frame: "map".into(),
            },
            later,
        );
        assert_eq!(s.goal(), Some(Point::new(0.0, 30.0)));
        assert_eq!(s.trip(later), Trip::WaitingForPlan);
        assert_eq!(s.remaining(), Some(Remaining::StraightLine(Metres(30.0))));
        let planned = later + Duration::from_millis(200);
        s.apply(plan(&[(0.0, 0.0), (0.0, 31.0)]), planned);
        assert_eq!(s.goal(), Some(Point::new(0.0, 31.0)));
        assert_eq!(s.trip(planned), Trip::Driving);
    }

    #[test]
    fn the_wrong_frame_is_reported() {
        let mut s = MapState::new(MapConfig::default());
        s.apply(
            Update::Car(Car {
                pose: Pose::default(),
                speed: MetresPerSecond(0.0),
                frame: "odom".into(),
            }),
            Instant::now(),
        );
        assert_eq!(s.frame_problems(), ["the car is in odom, not map"]);
    }

    /// A street route is drawn, but it's not the plan: the trip and the distance left
    /// still follow Nav2's plan alone. It's latched, so it never goes stale.
    #[test]
    fn a_street_route_is_only_shown() {
        let now = Instant::now();
        let mut s = MapState::new(MapConfig::default());
        s.apply(car_at(0.0, 0.0), now);
        let street = Route::new([Point::new(0.0, 0.0), Point::new(0.0, 500.0)]);
        s.apply(
            Update::StreetRoute {
                route: street.clone(),
                frame: "map".into(),
            },
            now,
        );
        assert_eq!(s.street_route, street);
        assert_eq!(s.trip(now), Trip::NoGoal);
        assert_eq!(s.remaining(), None);
        let much_later = now + Duration::from_secs(3600);
        assert!(s.health(Topic::StreetRoute, much_later).is_ok());
        s.apply(
            Update::StreetRoute {
                route: street,
                frame: "utm".into(),
            },
            now,
        );
        assert_eq!(s.frame_problems(), ["the street route is in utm, not map"]);
    }
}

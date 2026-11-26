//! The world view's pages (wrap-up step 4, TODO 278): what the debug screen (`debug_hud`)
//! showed and the developer window didn't, named for the layers it comes from.
//!
//! | Page | Shows |
//! |---|---|
//! | SENSORS | each sensor from its own topic: OK, STALE or NEVER, and what it said |
//! | WHERE AM I | each localizer: where it puts the car, how far from the local EKF; map > odom; NDT; the health checks |
//! | WHAT IT SEES | the costmap's cubes and the voxels node's road fit; how many cubes sit on NDT's map; the things |
//! | PLAN | who drives, the plan, the tracker: its name, how far off the path, its loop time |
//!
//! The rows come from debug-core's [`DebugState`], the debug screen's own logic. No Bevy,
//! no egui, tested.

use std::time::Instant;

use debug_core::{
    localization_health::Verdict,
    ndt::{Fit, NdtState},
    state::{DebugState, Localizer, Source},
    voxels::Road,
};
use hud_core::health::Health;
use map_core::geom::{Point, Pose};

/// One page of the world view's corner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Page {
    #[default]
    Sensors,
    WhereAmI,
    WhatItSees,
    Plan,
}

impl Page {
    /// Every page, in the tabs' order.
    pub const ALL: [Page; 4] = [Page::Sensors, Page::WhereAmI, Page::WhatItSees, Page::Plan];

    /// Its tab's name.
    pub fn title(self) -> &'static str {
        match self {
            Self::Sensors => "SENSORS",
            Self::WhereAmI => "WHERE AM I",
            Self::WhatItSees => "WHAT IT SEES",
            Self::Plan => "PLAN",
        }
    }

    /// From `--page`'s word: sensors, where, sees, plan.
    pub fn from_word(word: &str) -> Option<Self> {
        match word {
            "sensors" => Some(Self::Sensors),
            "where" => Some(Self::WhereAmI),
            "sees" => Some(Self::WhatItSees),
            "plan" => Some(Self::Plan),
            _ => None,
        }
    }
}

/// A row's light: how its words are coloured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Light {
    /// Heard lately, and nothing wrong.
    Ok,
    /// Heard lately, but it says something is wrong (a WARN, LOST, a flat road).
    Warn,
    /// Heard once, not lately.
    Stale,
    /// Never heard.
    Never,
    /// Not read by this window on purpose: the words say why.
    Off,
}

impl Light {
    /// A topic's health as a light.
    pub fn of(health: Health) -> Self {
        match health {
            Health::Ok(_) => Self::Ok,
            Health::Stale(_) => Self::Stale,
            Health::Never => Self::Never,
        }
    }

    /// The word in the row's pill.
    pub fn word(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::Warn => "WARN",
            Self::Stale => "STALE",
            Self::Never => "NEVER",
            Self::Off => "OFF",
        }
    }
}

/// One line of a page.
#[derive(Clone, Debug, PartialEq)]
pub struct Row {
    pub name: &'static str,
    pub light: Light,
    pub words: String,
}

impl Row {
    fn new(name: &'static str, light: Light, words: impl Into<String>) -> Self {
        Self {
            name,
            light,
            words: words.into(),
        }
    }
}

/// What the window itself knows that the debug state doesn't: from its own topics.
#[derive(Clone, Debug, Default)]
pub struct Extras<'a> {
    /// How many of the newest cubes sit on NDT's map, and how many cubes were compared.
    pub on_map: Option<(f64, usize)>,
    /// How many things are near the car now.
    pub things: usize,
    /// Who is driving (the command mux's state), while it is fresh.
    pub driving: Option<&'a str>,
    /// The plan, in the map frame.
    pub plan: &'a [Point],
    /// The car on the map.
    pub car_map: Pose,
}

/// The words for a row that hasn't been heard lately: how long ago, or how to start it.
fn quiet(health: Health, source: Source) -> String {
    match (health.age(), source) {
        (Some(age), _) => format!("{:.1} s ago", age.as_secs_f32()),
        (None, Source::Nav2Cubes) => "off (voxels:=false)".to_owned(),
        (None, Source::LocalizationHealth) => "not running (health:=false)".to_owned(),
        (None, Source::Estimate(Localizer::Amcl)) => "not running (amcl:=true)".to_owned(),
        (None, Source::Estimate(Localizer::Truth)) => "the simulator's only".to_owned(),
        (None, Source::Track) => "none (mppi_lag, the default follower)".to_owned(),
        (None, _) => String::new(),
    }
}

/// A row from the debug state's own detail for `source`.
fn heard_row(debug: &DebugState, source: Source, now: Instant) -> Row {
    let health = debug.health(source, now);
    let words = match debug.detail(source) {
        Some(text) if health.is_ok() => text.to_owned(),
        _ => quiet(health, source),
    };
    Row::new(source.label(), Light::of(health), words)
}

/// SENSORS: each sensor from its own topic.
pub fn sensors(debug: &DebugState, now: Instant) -> Vec<Row> {
    let mut rows: Vec<Row> = [Source::Imu, Source::Gps]
        .into_iter()
        .map(|s| heard_row(debug, s, now))
        .collect();
    rows.push(Row::new(
        Source::Lidar.label(),
        Light::Off,
        "not read here: 9 MB/s; the car's cubes stand for it (WHAT IT SEES)",
    ));
    rows.extend(
        [Source::Wheels, Source::WheelOdom]
            .into_iter()
            .map(|s| heard_row(debug, s, now)),
    );
    rows.push(vehicle(debug, now));
    rows
}

/// The car's own status, in one line.
fn vehicle(debug: &DebugState, now: Instant) -> Row {
    let health = debug.health(Source::Vehicle, now);
    let (light, words) = match (health.is_ok(), debug.vehicle) {
        (true, Some(v)) => {
            let estop = if v.estop_active { ", E-STOP" } else { "" };
            let faults = if v.faults.is_empty() {
                String::new()
            } else {
                format!(", faults {}", v.faults)
            };
            let light = if v.estop_active || !v.faults.is_empty() {
                Light::Warn
            } else {
                Light::Ok
            };
            let words = format!(
                "{}, {}, {:+.2} m/s, steer {:+.1} deg, gear {}{estop}{faults}",
                v.mode,
                if v.armed { "armed" } else { "disarmed" },
                v.speed.0,
                v.steering_angle.to_degrees(),
                v.gear().letter(),
            );
            (light, words)
        }
        _ => (Light::of(health), quiet(health, Source::Vehicle)),
    };
    Row::new(Source::Vehicle.label(), light, words)
}

/// WHERE AM I: each localizer, map > odom, NDT and the health checks.
pub fn where_am_i(debug: &DebugState, now: Instant) -> Vec<Row> {
    let mut rows: Vec<Row> = Localizer::ALL
        .into_iter()
        .map(|who| localizer(debug, who, now))
        .collect();
    let health = debug.health(Source::MapToOdom, now);
    let words = match (health.is_ok(), debug.map_to_odom.odom_in_map) {
        (true, Some(o)) => format!(
            "odom at x {:.2}, y {:.2}, {:+.1} deg on the map",
            o.position.x,
            o.position.y,
            o.yaw.to_degrees()
        ),
        _ => quiet(health, Source::MapToOdom),
    };
    rows.push(Row::new(
        Source::MapToOdom.label(),
        Light::of(health),
        words,
    ));
    rows.push(ndt(debug, now));
    rows.push(localization_health(debug, now));
    if let Some(problem) = &debug.frame_problem {
        rows.push(Row::new("frames", Light::Warn, problem.clone()));
    }
    rows
}

/// One localizer: where it puts the car, and how far that is from the local EKF.
fn localizer(debug: &DebugState, who: Localizer, now: Instant) -> Row {
    let source = Source::Estimate(who);
    let health = debug.health(source, now);
    let Some(pose) = debug.pose_in_map(who).filter(|_| health.is_ok()) else {
        let words = match (health.is_ok(), who) {
            (true, Localizer::Local) => "heard: needs map > odom to place it".to_owned(),
            _ => quiet(health, source),
        };
        return Row::new(who.label(), Light::of(health), words);
    };
    let mut words = format!(
        "x {:.2}, y {:.2}, {:+.0} deg",
        pose.position.x,
        pose.position.y,
        pose.yaw.to_degrees()
    );
    match (who, debug.offset_from_local(who)) {
        (Localizer::Local, _) => {
            if let Some(v) = debug.speed {
                words.push_str(&format!(", {v:+.2} m/s"));
            }
        }
        (_, Some(off)) => words.push_str(&format!(", {off:.2} m from EKF local")),
        (_, None) => {}
    }
    Row::new(who.label(), Light::Ok, words)
}

/// NDT's status: TRACKING with a good fit is OK; LOST or a poor fit warns.
fn ndt(debug: &DebugState, now: Instant) -> Row {
    let health = debug.health(Source::NdtStatus, now);
    match (&debug.ndt, health.is_ok()) {
        (Some(status), true) => {
            let light = match (status.state, status.fit) {
                (Some(NdtState::Tracking), Fit::Good) => Light::Ok,
                _ => Light::Warn,
            };
            let words = format!("{}: {}", status.headline(), status.numbers());
            Row::new("NDT match", light, words)
        }
        _ => {
            let words = match health.age() {
                Some(_) => quiet(health, Source::NdtStatus),
                None => "not running (ndt:=true; its map: pixi run ndt-map drive)".to_owned(),
            };
            Row::new("NDT match", Light::of(health), words)
        }
    }
}

/// Betsi's health checks: OK, or WARN and each check that isn't.
fn localization_health(debug: &DebugState, now: Instant) -> Row {
    let source = Source::LocalizationHealth;
    let health = debug.health(source, now);
    match (&debug.localization_health, health.is_ok()) {
        (Some(verdict), true) => {
            let light = if *verdict == Verdict::Ok {
                Light::Ok
            } else {
                Light::Warn
            };
            Row::new(source.label(), light, verdict.line())
        }
        _ => Row::new(source.label(), Light::of(health), quiet(health, source)),
    }
}

/// WHAT IT SEES: the costmap's cubes, the road's fit, the cubes on NDT's map, the things.
pub fn what_it_sees(debug: &DebugState, extras: &Extras<'_>, now: Instant) -> Vec<Row> {
    let mut rows = vec![heard_row(debug, Source::Nav2Cubes, now)];
    let health = debug.health(Source::VoxelsStatus, now);
    rows.push(match (&debug.voxels, health.is_ok()) {
        (Some(status), true) => {
            let light = match status.road {
                Road::Fitted => Light::Ok,
                Road::Flat => Light::Warn,
            };
            Row::new("road", light, status.details())
        }
        _ => Row::new("road", Light::of(health), quiet(health, Source::Nav2Cubes)),
    });
    rows.push(match extras.on_map {
        Some((share, cubes)) => {
            let light = if share >= 0.5 { Light::Ok } else { Light::Warn };
            Row::new(
                "on NDT's map",
                light,
                format!("{:.0} % of {cubes} cubes land on a cell", share * 100.0),
            )
        }
        None => Row::new("on NDT's map", Light::Never, "no NDT map, or no cubes yet"),
    });
    rows.push(Row::new(
        "things",
        if extras.things > 0 {
            Light::Ok
        } else {
            Light::Never
        },
        format!("{} near the car (key K)", extras.things),
    ));
    rows
}

/// PLAN: who drives, the plan, and the tracker.
pub fn plan(debug: &DebugState, extras: &Extras<'_>, now: Instant) -> Vec<Row> {
    let driving = match extras.driving {
        Some(who) => Row::new("driving", Light::Ok, who),
        None => Row::new("driving", Light::Never, "the command mux is quiet"),
    };
    let plan = match extras.plan {
        [] | [_] => Row::new("plan", Light::Never, "no plan"),
        points => {
            let length: f64 = points.windows(2).map(|w| w[0].distance(w[1]).0).sum();
            let end = points[points.len() - 1];
            let left = extras.car_map.position.distance(end).0;
            Row::new(
                "plan",
                Light::Ok,
                format!(
                    "{} points, {length:.1} m long; its end {left:.1} m away",
                    points.len()
                ),
            )
        }
    };
    let health = debug.health(Source::Track, now);
    let tracker = match (&debug.track, health.is_ok()) {
        (Some(t), true) => {
            let mut parts = vec![t.tracker.clone()];
            if let Some(e) = t.cross_track {
                parts.push(format!("{e:+.2} m off the path"));
            }
            if let Some(ms) = t.loop_ms {
                parts.push(format!("{ms:.1} ms a loop"));
            }
            if let Some(left) = t.left {
                parts.push(format!("{left:.1} m left"));
            }
            let late = t.missed_ticks.filter(|&n| n > 0);
            if let Some(n) = late {
                parts.push(format!("{n} ticks missed"));
            }
            let light = if late.is_some() {
                Light::Warn
            } else {
                Light::Ok
            };
            Row::new("tracker", light, parts.join(", "))
        }
        _ => Row::new("tracker", Light::of(health), quiet(health, Source::Track)),
    };
    vec![driving, plan, tracker]
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use debug_core::{
        config::DebugConfig, ndt::NdtStatus, state::Update, track::TrackStatus, voxels::VoxelStatus,
    };

    use super::*;

    fn state() -> DebugState {
        DebugState::new(&DebugConfig::default())
    }

    fn pose(who: Localizer, x: f64) -> Update {
        Update::Pose {
            who,
            pose: Pose::new(x, 0.0, 0.0),
            frame: String::new(),
            speed: None,
        }
    }

    #[test]
    fn every_page_has_a_word_and_a_tab() {
        for page in Page::ALL {
            assert!(!page.title().is_empty());
        }
        assert_eq!(Page::from_word("where"), Some(Page::WhereAmI));
        assert_eq!(Page::from_word("cubes"), None);
    }

    #[test]
    fn a_sensor_heard_says_what_it_said_then_goes_stale() {
        let (mut s, t0) = (state(), Instant::now());
        s.apply(
            Update::Heard {
                source: Source::Imu,
                detail: "yaw rate +1.0 deg/s".into(),
            },
            t0,
        );
        let rows = sensors(&s, t0);
        assert_eq!(rows[0].light, Light::Ok);
        assert_eq!(rows[0].words, "yaw rate +1.0 deg/s");
        assert_eq!(rows[1].light, Light::Never, "the GPS was never heard");
        let lidar = rows
            .iter()
            .find(|r| r.name == "lidar")
            .expect("a lidar row");
        assert_eq!(lidar.light, Light::Off, "the window never reads the cloud");
        let later = sensors(&s, t0 + Duration::from_secs(5));
        assert_eq!(later[0].light, Light::Stale);
        assert!(later[0].words.ends_with("s ago"));
    }

    #[test]
    fn each_localizer_says_how_far_it_is_from_ekf_local() {
        let (mut s, now) = (state(), Instant::now());
        s.apply(Update::MapToOdom(Pose::new(10.0, 0.0, 0.0)), now);
        s.apply(pose(Localizer::Local, 1.0), now);
        s.apply(pose(Localizer::Amcl, 11.5), now);
        let rows = where_am_i(&s, now);
        let local = &rows[0];
        assert_eq!(local.light, Light::Ok);
        assert!(local.words.starts_with("x 11.00"), "{}", local.words);
        let amcl = rows.iter().find(|r| r.name == "AMCL").expect("AMCL's row");
        assert!(
            amcl.words.ends_with("0.50 m from EKF local"),
            "{}",
            amcl.words
        );
        let truth = rows
            .iter()
            .find(|r| r.name == "truth (sim)")
            .expect("truth");
        assert_eq!(truth.light, Light::Never);
        let map_odom = rows.iter().find(|r| r.name == "map > odom").expect("row");
        assert!(map_odom.words.contains("x 10.00"));
    }

    #[test]
    fn lost_and_warn_light_amber() {
        let (mut s, now) = (state(), Instant::now());
        s.apply(
            Update::Ndt(NdtStatus::from_message(2, "LOST", [("match_ms", "3")])),
            now,
        );
        s.apply(
            Update::LocalizationHealth(Verdict::parse("WARN tf ERROR: map > odom old")),
            now,
        );
        let rows = where_am_i(&s, now);
        let ndt = rows
            .iter()
            .find(|r| r.name == "NDT match")
            .expect("its row");
        assert_eq!(ndt.light, Light::Warn);
        assert!(ndt.words.starts_with("LOST"));
        let health = rows
            .iter()
            .find(|r| r.name == "loc. health")
            .expect("its row");
        assert_eq!(health.light, Light::Warn);
        assert!(health.words.contains("tf"));
    }

    #[test]
    fn a_flat_road_and_few_cubes_on_the_map_warn() {
        let (mut s, now) = (state(), Instant::now());
        s.apply(
            Update::VoxelsStatus(VoxelStatus::from_message(1, "flat", [("cubes", "40")])),
            now,
        );
        let extras = Extras {
            on_map: Some((0.2, 100)),
            ..Extras::default()
        };
        let rows = what_it_sees(&s, &extras, now);
        assert_eq!(rows[1].light, Light::Warn, "the road taken as flat");
        assert_eq!(rows[2].light, Light::Warn);
        assert_eq!(rows[2].words, "20 % of 100 cubes land on a cell");
    }

    #[test]
    fn the_plan_page_measures_the_plan_and_reads_the_tracker() {
        let (mut s, now) = (state(), Instant::now());
        s.apply(
            Update::Track(TrackStatus::from_values(
                "lqr",
                [("cross_track_m", "-0.05"), ("missed_ticks", "2")],
            )),
            now,
        );
        let plan_points = [Point::new(0.0, 0.0), Point::new(3.0, 4.0)];
        let extras = Extras {
            driving: Some("nav (20)"),
            plan: &plan_points,
            ..Extras::default()
        };
        let rows = plan(&s, &extras, now);
        assert_eq!(rows[0].words, "nav (20)");
        assert_eq!(rows[1].words, "2 points, 5.0 m long; its end 5.0 m away");
        assert_eq!(rows[2].light, Light::Warn, "ticks were missed");
        assert!(rows[2].words.starts_with("lqr, -0.05 m off the path"));
    }
}

//! Everything the screen shows, as one value: the newest of each message, and when
//! each topic was last heard.
//!
//! A data source (the ROS node, or the demo) sends [`Update`]s; the screen calls
//! [`HudState::apply`] for each, then draws from the state.

use std::{collections::HashMap, time::Instant};

pub use crate::watchdog::Alarm;

use crate::{
    ahead,
    config::{HudConfig, Topic},
    gate::{self, Verdict},
    gps::Fix,
    health::{Health, LastSeen},
    origin::{Announcement, Origins},
    picture::Picture,
    self_test::{self, Readiness, Report},
    things::{CarPose, Thing},
    vehicle::VehicleStatus,
    watchdog,
};

/// The battery (sensor_msgs/msg/BatteryState). None = not measured (NaN in the message).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Battery {
    /// Volts.
    pub voltage: Option<f32>,
    /// 0 to 100 (the message gives 0 to 1).
    pub percent: Option<f32>,
    /// Amps; negative while discharging (REP: BatteryState).
    pub current: Option<f32>,
}

impl Battery {
    /// From the message's fields: volts, charge 0..1, amps. NaN means not measured.
    #[must_use]
    pub fn from_message(voltage: f32, percentage: f32, current: f32) -> Self {
        let measured = |x: f32| x.is_finite().then_some(x);
        Self {
            voltage: measured(voltage),
            percent: measured(percentage).map(|p| (p * 100.0).clamp(0.0, 100.0)),
            current: measured(current),
        }
    }
}

/// What came of the last e-stop message the source published.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstopReport {
    /// Published; this many nodes subscribe to the e-stop topic.
    Sent {
        /// Nodes subscribed to the e-stop topic.
        subscribers: usize,
    },
    /// Couldn't publish at all.
    Failed,
}

/// One piece of news from a data source.
#[derive(Debug, Clone, PartialEq)]
pub enum Update {
    /// A `/vehicle/status` message.
    Vehicle(VehicleStatus),
    /// The GPS receiver's own fix.
    Gps(Fix),
    /// The EKF's position, as a fix.
    GpsFiltered(Fix),
    /// Whether a stop sign is seen.
    StopSign(bool),
    /// The battery.
    Battery(Battery),
    /// A camera picture.
    Picture(Picture),
    /// The camera sent a picture the HUD can't show; why.
    BadPicture(String),
    /// What came of an e-stop press.
    Estop(EstopReport),
    /// A node said where its data comes from (`/system/source`).
    Source(Announcement),
    /// The sensor watchdog's verdict: "OK", or "STOP: ..." / "WARN: ...".
    Watchdog(String),
    /// The output mux's gate: "OPEN - ...", "WAIT - ..." or "STOP - ...".
    OutputMux(String),
    /// The boot self-test's answer: READY, or NOT READY and why.
    SelfTest(Report),
    /// The squiggly line's bold line: ego's trace, base_link.
    Trace(Vec<ahead::Point>),
    /// Its faint line: the plan ahead, base_link (empty: no plan).
    PlanAhead(Vec<ahead::Point>),
    /// Its stop bar.
    StopPoint(ahead::StopMark),
    /// Its numbers.
    Ahead(ahead::Summary),
    /// The things near the car (odom), named by the camera.
    Things(Vec<Thing>),
    /// The car's pose in odom: x, y (m), yaw (rad).
    CarPose(CarPose),
}

impl Update {
    /// The topic this news came from, if any.
    #[must_use]
    pub fn topic(&self) -> Option<Topic> {
        match self {
            Self::Vehicle(_) => Some(Topic::VehicleStatus),
            Self::Gps(_) => Some(Topic::Gps),
            Self::GpsFiltered(_) => Some(Topic::GpsFiltered),
            Self::StopSign(_) => Some(Topic::StopSign),
            Self::Battery(_) => Some(Topic::Battery),
            Self::Picture(_) | Self::BadPicture(_) => Some(Topic::Camera),
            Self::Watchdog(_) => Some(Topic::Watchdog),
            Self::OutputMux(_) => Some(Topic::OutputMux),
            Self::SelfTest(_) => Some(Topic::SelfTest),
            Self::Trace(_) => Some(Topic::Trace),
            Self::PlanAhead(_) => Some(Topic::PlanAhead),
            Self::StopPoint(_) => Some(Topic::StopPoint),
            Self::Ahead(_) => Some(Topic::Ahead),
            Self::Things(_) => Some(Topic::Things),
            Self::CarPose(_) => Some(Topic::Odom),
            // Latched, sent once: its age says nothing about the data.
            Self::Estop(_) | Self::Source(_) => None,
        }
    }
}

/// What the screen draws.
#[derive(Debug, Clone, Default)]
pub struct HudState {
    /// The settings.
    pub config: HudConfig,
    /// The car's last status.
    pub vehicle: Option<VehicleStatus>,
    /// The GPS receiver's last fix.
    pub gps: Option<Fix>,
    /// The EKF's last position.
    pub gps_filtered: Option<Fix>,
    /// A stop sign is seen.
    pub stop_sign: bool,
    /// The battery's last state.
    pub battery: Option<Battery>,
    /// Why the last camera picture couldn't be shown; None once one could.
    pub picture_problem: Option<String>,
    /// The last e-stop report since the button was last pressed.
    pub estop: Option<EstopReport>,
    /// Where the data comes from, as announced.
    pub origins: Origins,
    /// The sensor watchdog's last verdict.
    pub watchdog: Option<String>,
    /// The output mux's last state.
    pub output_mux: Option<String>,
    /// The boot self-test's last answer.
    pub self_test: Option<Report>,
    /// The squiggly line, as last heard (hud_core::ahead).
    pub ahead: ahead::Ahead,
    /// The things near the car, as last heard (odom).
    pub things: Vec<Thing>,
    /// The car's pose in odom, as last heard: x, y, yaw.
    pub car_pose: Option<CarPose>,
    new_picture: Option<Picture>,
    last_seen: HashMap<Topic, LastSeen>,
}

impl HudState {
    /// A state with nothing heard yet.
    #[must_use]
    pub fn new(config: HudConfig) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    /// Takes in one update that arrived at `now`.
    pub fn apply(&mut self, update: Update, now: Instant) {
        if let Some(topic) = update.topic() {
            self.last_seen.entry(topic).or_default().heard(now);
        }
        match update {
            Update::Vehicle(v) => self.vehicle = Some(v),
            Update::Gps(f) => self.gps = Some(f),
            Update::GpsFiltered(f) => self.gps_filtered = Some(f),
            Update::StopSign(s) => self.stop_sign = s,
            Update::Battery(b) => self.battery = Some(b),
            Update::Picture(p) => {
                self.picture_problem = None;
                self.new_picture = Some(p);
            }
            Update::BadPicture(why) => self.picture_problem = Some(why),
            Update::Estop(report) => self.estop = Some(report),
            Update::Source(a) => self.origins.apply(a),
            Update::Watchdog(w) => self.watchdog = Some(w),
            Update::OutputMux(g) => self.output_mux = Some(g),
            Update::SelfTest(r) => self.self_test = Some(r),
            Update::Trace(t) => self.ahead.trace = t,
            Update::PlanAhead(p) => self.ahead.plan = p,
            Update::StopPoint(s) => self.ahead.stop = Some(s),
            Update::Ahead(s) => self.ahead.summary = Some(s),
            Update::Things(t) => self.things = t,
            Update::CarPose(p) => self.car_pose = Some(p),
        }
    }

    /// The things to draw and the car's pose to place them, both fresh; None if either
    /// is quiet (old things would sit where the car no longer is).
    #[must_use]
    pub fn things_now(&self, now: Instant) -> Option<(&[Thing], CarPose)> {
        let fresh =
            self.health(Topic::Things, now).is_ok() && self.health(Topic::Odom, now).is_ok();
        self.car_pose
            .filter(|_| fresh)
            .map(|pose| (self.things.as_slice(), pose))
    }

    /// The e-stop button was pressed: forget the last press's report.
    pub fn estop_pressed(&mut self) {
        self.estop = None;
    }

    /// The newest picture, once: the screen uploads it, then it's gone from here.
    pub fn take_new_picture(&mut self) -> Option<Picture> {
        self.new_picture.take()
    }

    /// OK / STALE / NEVER for one topic, at `now`.
    #[must_use]
    pub fn health(&self, topic: Topic, now: Instant) -> Health {
        self.last_seen
            .get(&topic)
            .copied()
            .unwrap_or_default()
            .health(now, self.config.stale_after())
    }

    /// What the sensor watchdog says is wrong, for a banner; None when all is well
    /// ([`watchdog::alarm`]: the map window shows the same).
    #[must_use]
    pub fn sensor_alarm(&self, now: Instant) -> Option<Alarm> {
        watchdog::alarm(self.health(Topic::Watchdog, now), self.watchdog.as_deref())
    }

    /// The boot self-test: READY, NOT READY and why, or silent; None when never heard
    /// ([`self_test::readiness`]). It speaks at 2 Hz: quiet only after [`self_test::QUIET`].
    #[must_use]
    pub fn readiness(&self, now: Instant) -> Option<Readiness> {
        let limit = self.config.stale_after().max(self_test::QUIET);
        let health = self
            .last_seen
            .get(&Topic::SelfTest)
            .copied()
            .unwrap_or_default()
            .health(now, limit);
        self_test::readiness(health, self.self_test.as_ref())
    }

    /// May the car move? GO / WAIT / STOP and the one main reason ([`gate::verdict`]).
    #[must_use]
    pub fn verdict(&self, now: Instant) -> Verdict {
        let fresh = self.health(Topic::VehicleStatus, now).is_ok();
        let estop_on = fresh && self.vehicle.is_some_and(|v| v.estop_active);
        // The output mux speaks twice a second: its own, longer limit.
        let limit = self.config.stale_after().max(gate::GATE_QUIET);
        let gate_health = self
            .last_seen
            .get(&Topic::OutputMux)
            .copied()
            .unwrap_or_default()
            .health(now, limit);
        gate::verdict(
            estop_on,
            (gate_health, self.output_mux.as_deref()),
            (self.health(Topic::Watchdog, now), self.watchdog.as_deref()),
        )
    }

    /// Every listened topic's name and health, in screen order.
    pub fn topic_health(&self, now: Instant) -> impl Iterator<Item = (&str, Health)> + '_ {
        Topic::LISTENED
            .into_iter()
            .map(move |t| (self.config.topic(t), self.health(t, now)))
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::gps::{FixKind, RawFix};
    use crate::units::MetresPerSecond;

    #[test]
    fn updates_mark_their_topic_heard() {
        let start = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        assert_eq!(s.health(Topic::VehicleStatus, start), Health::Never);
        let v = VehicleStatus {
            speed: MetresPerSecond(2.0),
            ..VehicleStatus::default()
        };
        s.apply(Update::Vehicle(v), start);
        assert!(s.health(Topic::VehicleStatus, start).is_ok());
        assert_eq!(s.vehicle.map(|v| v.speed), Some(MetresPerSecond(2.0)));
        let later = start + Duration::from_secs(2);
        assert!(matches!(
            s.health(Topic::VehicleStatus, later),
            Health::Stale(_)
        ));
        assert_eq!(s.health(Topic::Gps, later), Health::Never);
    }

    #[test]
    fn things_show_only_while_they_and_the_car_pose_are_fresh() {
        let start = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        let person = Thing {
            id: 3,
            name: "person".into(),
            x: 4.0,
            ..Thing::default()
        };
        s.apply(Update::Things(vec![person]), start);
        assert!(s.things_now(start).is_none(), "no car pose yet");
        s.apply(Update::CarPose((0.0, 0.0, 0.0)), start);
        let (things, pose) = s.things_now(start).expect("both fresh");
        assert_eq!(things[0].label(), "person 3");
        assert_eq!(pose, (0.0, 0.0, 0.0));
        assert!(s.things_now(start + Duration::from_secs(2)).is_none());
    }

    #[test]
    fn the_watchdog_banner_shows_only_trouble() {
        let start = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        assert_eq!(s.sensor_alarm(start), None); // no watchdog in this run
        s.apply(Update::Watchdog("OK".into()), start);
        assert_eq!(s.sensor_alarm(start), None);
        let stop = "STOP: gps STALE (nothing for 2.3 s)";
        s.apply(Update::Watchdog(stop.into()), start);
        assert_eq!(s.sensor_alarm(start), Some(Alarm::Stop(stop.into())));
        s.apply(Update::Watchdog("WARN: radar NEVER".into()), start);
        assert!(matches!(s.sensor_alarm(start), Some(Alarm::Warn(_))));
        let later = start + s.config.stale_after() + Duration::from_millis(100);
        assert_eq!(
            s.sensor_alarm(later),
            Some(Alarm::Stop("SENSOR WATCHDOG SILENT".into()))
        );
    }

    #[test]
    fn the_self_test_shows_ready_or_why_not() {
        let start = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        assert_eq!(s.readiness(start), None); // no self-test in this run
        let why = "NOT READY - actuators (no /vehicle/status)";
        s.apply(Update::SelfTest(Report::from_status(2, why)), start);
        assert_eq!(
            s.readiness(start),
            Some(Readiness::NotReady(
                "NOT READY: actuators (no /vehicle/status)".into()
            ))
        );
        s.apply(Update::SelfTest(Report::from_status(0, "READY")), start);
        // 1.5 s: past stale_after (1 s), not yet the self-test's own 2 s.
        let later = start + Duration::from_millis(1500);
        assert_eq!(s.readiness(later), Some(Readiness::Ready));
        let silent = start + Duration::from_secs(3);
        assert_eq!(
            s.readiness(silent),
            Some(Readiness::NotReady("SELF-TEST SILENT".into()))
        );
    }

    #[test]
    fn the_verdict_follows_the_gate_the_watchdog_and_the_estop() {
        use crate::gate::Light;
        let start = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        assert_eq!(s.verdict(start).light, Light::Unknown); // nothing heard yet
        s.apply(
            Update::OutputMux("OPEN - car: every source REAL".into()),
            start,
        );
        assert_eq!(s.verdict(start).light, Light::Go);
        // 1.5 s later: past stale_after (1 s), not yet the gate's own 2 s.
        let later = start + Duration::from_millis(1500);
        assert_eq!(s.verdict(later).light, Light::Go);
        let silent = start + Duration::from_secs(3);
        assert_eq!(s.verdict(silent).reason, "gate silent");
        s.apply(
            Update::OutputMux("OPEN - car: every source REAL".into()),
            silent,
        );
        s.apply(Update::Watchdog("STOP: gps STALE (x)".into()), silent);
        assert_eq!(s.verdict(silent).reason, "GPS STALE");
        let stopped = VehicleStatus {
            estop_active: true,
            ..VehicleStatus::default()
        };
        s.apply(Update::Vehicle(stopped), silent);
        assert_eq!(s.verdict(silent).reason, "E-STOP ON");
        // An old status says nothing about the e-stop now.
        let old = silent + Duration::from_secs(2);
        s.apply(
            Update::OutputMux("OPEN - car: every source REAL".into()),
            old,
        );
        s.apply(Update::Watchdog("OK".into()), old);
        assert_eq!(s.verdict(old).light, Light::Go);
    }

    #[test]
    fn topic_health_lists_names_from_the_config() {
        let mut config = HudConfig::default();
        config.set_topic(Topic::Gps, "/sensors/gps/fix".into());
        let s = HudState::new(config);
        let names: Vec<&str> = s.topic_health(Instant::now()).map(|(n, _)| n).collect();
        assert_eq!(names.len(), Topic::LISTENED.len());
        assert!(names.contains(&"/sensors/gps/fix"));
    }

    #[test]
    fn a_no_fix_message_still_counts_as_heard() {
        // "Is there GPS data?" A receiver with no fix is alive; a silent one isn't.
        let now = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        let raw = RawFix {
            status: -1,
            ..RawFix::default()
        };
        s.apply(Update::Gps(Fix::from(raw)), now);
        assert!(s.health(Topic::Gps, now).is_ok());
        assert_eq!(s.gps.map(|f| f.kind), Some(FixKind::NoFix));
    }

    #[test]
    fn a_picture_is_taken_once() {
        let now = Instant::now();
        let mut s = HudState::new(HudConfig::default());
        s.apply(Update::BadPicture("yuv422".into()), now);
        assert!(s.picture_problem.is_some());
        let p = Picture {
            width: 1,
            height: 1,
            rgba: vec![0, 0, 0, 255],
        };
        s.apply(Update::Picture(p), now);
        assert!(s.picture_problem.is_none());
        assert!(s.take_new_picture().is_some());
        assert!(s.take_new_picture().is_none());
    }

    #[test]
    fn battery_from_message() {
        let b = Battery::from_message(50.9, 0.997, f32::NAN);
        assert_eq!(b.voltage, Some(50.9));
        assert!(b.percent.is_some_and(|p| (p - 99.7).abs() < 1e-3));
        assert_eq!(b.current, None);
    }

    #[test]
    fn a_source_announcement_sets_the_banner() {
        use crate::origin::{Entry, Origin};
        let mut s = HudState::new(HudConfig::default());
        assert_eq!(s.origins.banner(), None);
        let a = Announcement {
            announcer: "sim_adapter".into(),
            entries: vec![Entry {
                name: "vehicle".into(),
                topic: "/vehicle/status".into(),
                origin: Origin::Sim,
            }],
        };
        s.apply(Update::Source(a), Instant::now());
        assert_eq!(s.origins.banner().as_deref(), Some("SIMULATED"));
        assert_eq!(s.origins.of_topic("/vehicle/status"), Some(Origin::Sim));
    }

    #[test]
    fn estop_reports_reset_on_press() {
        let mut s = HudState::new(HudConfig::default());
        s.apply(
            Update::Estop(EstopReport::Sent { subscribers: 0 }),
            Instant::now(),
        );
        assert!(s.estop.is_some());
        s.estop_pressed();
        assert!(s.estop.is_none());
    }
}

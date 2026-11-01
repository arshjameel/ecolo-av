//! Everything the hardware panel shows, as one value: each sensor's and actuator's
//! latest number from its own node, with OK / STALE / NEVER, and what the command path
//! says (`/system/source`, the e-stop, twist_mux's output, the locks).
//!
//! News comes in as [`Update`]s (the ROS thread or the demo makes them). The big
//! topics (lidar, radar, camera) are only counted: their rate is the number.

use std::{
    collections::{HashMap, VecDeque},
    time::{Duration, Instant},
};

use hud_core::{
    gps::{Fix, FixKind},
    health::{Health, LastSeen},
    origin::{Announcement, Origins},
    state::Battery,
    vehicle::VehicleStatus,
};

use crate::{
    buttons::Command,
    rules::{source_kind, Kind},
    vehicle::SpeedWheels,
};

/// A row of the panel: one sensor or actuator, from its own topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Row {
    Vehicle,
    Steering,
    Throttle,
    Brake,
    Wheels,
    Imu,
    Gps,
    Lidar,
    Radar,
    Camera,
    Battery,
    /// twist_mux's output, `/control/cmd_vel`.
    Mux,
    GeofenceLock,
}

impl Row {
    /// The sensor rows, top to bottom.
    pub const SENSORS: [Row; 8] = [
        Row::Imu,
        Row::Wheels,
        Row::Gps,
        Row::Lidar,
        Row::Radar,
        Row::Camera,
        Row::Battery,
        Row::Vehicle,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Vehicle => "vehicle",
            Self::Steering => "steering",
            Self::Throttle => "throttle",
            Self::Brake => "brake",
            Self::Wheels => "wheels",
            Self::Imu => "IMU",
            Self::Gps => "GPS",
            Self::Lidar => "lidar",
            Self::Radar => "radar",
            Self::Camera => "camera",
            Self::Battery => "battery",
            Self::Mux => "twist_mux",
            Self::GeofenceLock => "geofence",
        }
    }
}

/// One actuator's state (aav_interfaces/ActuatorState): asked and done. NaN = unknown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Actuator {
    pub command: f32,
    pub measured: f32,
}

impl Actuator {
    /// What it did, if known.
    pub fn measured(self) -> Option<f32> {
        self.measured.is_finite().then_some(self.measured)
    }
}

/// Each wheel's speed, m/s; NaN = not measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheels {
    pub front_left: f32,
    pub front_right: f32,
    pub rear_left: f32,
    pub rear_right: f32,
}

/// One IMU message: m/s² and rad/s, x y z.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
}

/// One piece of news.
#[derive(Debug, Clone, PartialEq)]
pub enum Update {
    Vehicle(VehicleStatus),
    Steering(Actuator),
    Throttle(Actuator),
    Brake(Actuator),
    Wheels(Wheels),
    Imu(ImuReading),
    Gps(Fix),
    /// A big message (lidar, radar, camera) arrived: only counted.
    Counted(Row),
    Battery(Battery),
    /// `/system/source`.
    Source(Announcement),
    /// `/safety/estop`.
    Estop(bool),
    /// `/control/cmd_vel`: twist_mux's output.
    Mux(Command),
    GeofenceLock(bool),
    NoTimeoutLock(bool),
    /// The E-STOP button's message went out: to this many subscribers (None: failed).
    EstopSent(Option<usize>),
}

/// Messages counted over this long for a rate.
const RATE_OVER: Duration = Duration::from_secs(2);

/// The panel's state.
#[derive(Debug, Clone)]
pub struct PanelState {
    stale_after: Duration,
    seen: HashMap<Row, LastSeen>,
    arrivals: HashMap<Row, VecDeque<Instant>>,
    pub vehicle: Option<VehicleStatus>,
    pub steering: Option<Actuator>,
    pub throttle: Option<Actuator>,
    pub brake: Option<Actuator>,
    pub wheels: Option<Wheels>,
    pub imu: Option<ImuReading>,
    pub gps: Option<Fix>,
    pub battery: Option<Battery>,
    pub origins: Origins,
    /// Whether anyone announced on `/system/source` yet.
    announced: bool,
    /// `/safety/estop`'s last value.
    pub estop: Option<bool>,
    /// twist_mux's last output, and when.
    pub mux: Option<(Instant, Command)>,
    pub geofence_lock: Option<bool>,
    pub no_timeout_lock: Option<bool>,
    /// The E-STOP button's last message: sent to how many (None: failed).
    pub estop_sent: Option<Option<usize>>,
}

impl PanelState {
    pub fn new(stale_after: Duration) -> Self {
        Self {
            stale_after,
            seen: HashMap::new(),
            arrivals: HashMap::new(),
            vehicle: None,
            steering: None,
            throttle: None,
            brake: None,
            wheels: None,
            imu: None,
            gps: None,
            battery: None,
            origins: Origins::default(),
            announced: false,
            estop: None,
            mux: None,
            geofence_lock: None,
            no_timeout_lock: None,
            estop_sent: None,
        }
    }

    fn heard(&mut self, row: Row, now: Instant) {
        self.seen.entry(row).or_default().heard(now);
        let times = self.arrivals.entry(row).or_default();
        times.push_back(now);
        while times
            .front()
            .is_some_and(|&t| now.saturating_duration_since(t) > RATE_OVER)
        {
            times.pop_front();
        }
    }

    /// Takes one update, which arrived at `now`.
    pub fn apply(&mut self, update: Update, now: Instant) {
        match update {
            Update::Vehicle(v) => {
                self.vehicle = Some(v);
                self.heard(Row::Vehicle, now);
            }
            Update::Steering(a) => {
                self.steering = Some(a);
                self.heard(Row::Steering, now);
            }
            Update::Throttle(a) => {
                self.throttle = Some(a);
                self.heard(Row::Throttle, now);
            }
            Update::Brake(a) => {
                self.brake = Some(a);
                self.heard(Row::Brake, now);
            }
            Update::Wheels(w) => {
                self.wheels = Some(w);
                self.heard(Row::Wheels, now);
            }
            Update::Imu(i) => {
                self.imu = Some(i);
                self.heard(Row::Imu, now);
            }
            Update::Gps(f) => {
                self.gps = Some(f);
                self.heard(Row::Gps, now);
            }
            Update::Counted(row) => self.heard(row, now),
            Update::Battery(b) => {
                self.battery = Some(b);
                self.heard(Row::Battery, now);
            }
            Update::Source(a) => {
                self.announced = true;
                self.origins.apply(a);
            }
            Update::Estop(on) => self.estop = Some(on),
            Update::Mux(c) => {
                self.mux = Some((now, c));
                self.heard(Row::Mux, now);
            }
            Update::GeofenceLock(on) => {
                self.geofence_lock = Some(on);
                self.heard(Row::GeofenceLock, now);
            }
            Update::NoTimeoutLock(on) => self.no_timeout_lock = Some(on),
            Update::EstopSent(n) => self.estop_sent = Some(n),
        }
    }

    /// One row's health at `now`.
    pub fn health(&self, row: Row, now: Instant) -> Health {
        self.seen
            .get(&row)
            .copied()
            .unwrap_or_default()
            .health(now, self.stale_after)
    }

    /// Messages a second over the last 2 s, by arrival.
    pub fn rate(&self, row: Row, now: Instant) -> Option<f64> {
        let times = self.arrivals.get(&row)?;
        let recent: Vec<Instant> = times
            .iter()
            .copied()
            .filter(|&t| now.saturating_duration_since(t) <= RATE_OVER)
            .collect();
        let (first, last) = (recent.first()?, recent.last()?);
        let span = last.saturating_duration_since(*first).as_secs_f64();
        (span > 0.0).then(|| (recent.len() - 1) as f64 / span)
    }

    /// What `/system/source` says we're driving (safety.py's rules).
    pub fn kind(&self) -> Kind {
        if !self.announced {
            return Kind::Unknown;
        }
        source_kind(self.origins.entries().map(|e| e.origin))
    }

    /// The e-stop is on: `/safety/estop` said true last, or `/vehicle/status` says so.
    pub fn estop_on(&self) -> bool {
        self.estop == Some(true) || self.vehicle.is_some_and(|v| v.estop_active)
    }

    /// The geofence's lock holds the buttons (51 is above their 45).
    pub fn geofence_holds(&self, now: Instant) -> bool {
        self.geofence_lock == Some(true) && self.health(Row::GeofenceLock, now).is_ok()
    }

    /// The measured speed, m/s (`/vehicle/status`), while fresh.
    pub fn seen_speed(&self, now: Instant) -> Option<f64> {
        let v = self.vehicle?;
        self.health(Row::Vehicle, now)
            .is_ok()
            .then_some(f64::from(v.speed.0))
    }

    /// The measured steering, rad (`/vehicle/steering`), while fresh.
    pub fn seen_steer(&self, now: Instant) -> Option<f64> {
        let m = self.steering?.measured()?;
        self.health(Row::Steering, now)
            .is_ok()
            .then_some(f64::from(m))
    }

    /// One sensor row's number, in a few words. `wheels_note`: on stands, which wheels
    /// are measured when they aren't the driven ones ("front, not driven").
    pub fn detail(&self, row: Row, now: Instant, wheels_note: Option<&str>) -> String {
        let hz = || {
            self.rate(row, now)
                .map_or_else(|| "-".to_owned(), |hz| format!("{hz:.1} Hz"))
        };
        match row {
            Row::Imu => self.imu.map_or_else(String::new, |i| {
                let [ax, ay, az] = i.accel;
                let [gx, gy, gz] = i.gyro.map(f64::to_degrees);
                format!(
                    "acc {ax:+.2} {ay:+.2} {az:+.2} m/s2   gyro {gx:+.1} {gy:+.1} {gz:+.1} deg/s"
                )
            }),
            Row::Wheels => self.wheels.map_or_else(String::new, |w| {
                let v = |x: f32| {
                    if x.is_finite() {
                        format!("{x:+.2}")
                    } else {
                        "-".to_owned()
                    }
                };
                let mut s = format!(
                    "FL {} FR {}  RL {} RR {} m/s",
                    v(w.front_left),
                    v(w.front_right),
                    v(w.rear_left),
                    v(w.rear_right)
                );
                if let Some(note) = wheels_note {
                    s.push_str(&format!("  ({note})"));
                }
                s
            }),
            Row::Gps => self.gps.map_or_else(String::new, |f| {
                let mut s = f.kind.to_string();
                if let Some(sigma) = f.horizontal_sigma.filter(|_| f.kind != FixKind::NoFix) {
                    s.push_str(&format!(", {sigma:.2} m"));
                }
                if let Some((lat, lon)) = f.position {
                    s.push_str(&format!("   {lat:.6}, {lon:.6}"));
                }
                s
            }),
            Row::Lidar | Row::Radar | Row::Camera | Row::Mux | Row::GeofenceLock => hz(),
            Row::Battery => self.battery.map_or_else(String::new, |b| {
                let mut parts = Vec::new();
                if let Some(v) = b.voltage {
                    parts.push(format!("{v:.1} V"));
                }
                if let Some(p) = b.percent {
                    parts.push(format!("{p:.0} %"));
                }
                if let Some(a) = b.current {
                    parts.push(format!("{a:+.1} A"));
                }
                parts.join("   ")
            }),
            Row::Vehicle => self.vehicle.map_or_else(String::new, |v| {
                format!(
                    "{}  {}{}  {:.2} m/s",
                    v.mode,
                    if v.armed { "ARMED" } else { "DISARMED" },
                    if v.estop_active { "  E-STOP" } else { "" },
                    v.speed.0
                )
            }),
            Row::Steering | Row::Throttle | Row::Brake => {
                let a = match row {
                    Row::Steering => self.steering,
                    Row::Throttle => self.throttle,
                    _ => self.brake,
                };
                a.and_then(Actuator::measured)
                    .map_or_else(String::new, |m| format!("{m:+.3}"))
            }
        }
    }
}

/// The wheel note on stands: the measured wheels when they aren't the driven ones
/// ("front, not driven"; TODO 49b).
pub fn wheels_note(stands: bool, kind: Kind, wheels: Option<&SpeedWheels>) -> Option<String> {
    let measured = wheels?;
    (stands && kind.is_real() && !measured.are_driven()).then(|| format!("{measured}, not driven"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hud_core::origin::{Entry, Origin};

    fn state() -> PanelState {
        PanelState::new(Duration::from_secs(1))
    }

    fn announce(who: &str, origin: Origin) -> Update {
        Update::Source(Announcement {
            announcer: who.into(),
            entries: vec![Entry {
                name: "imu".into(),
                topic: "/sensors/imu/data".into(),
                origin,
            }],
        })
    }

    #[test]
    fn ok_stale_never() {
        let t = Instant::now();
        let mut s = state();
        assert_eq!(s.health(Row::Imu, t), Health::Never);
        s.apply(
            Update::Imu(ImuReading {
                accel: [0.0, 0.0, 9.81],
                gyro: [0.0, 0.0, 0.1],
            }),
            t,
        );
        assert!(s.health(Row::Imu, t).is_ok());
        assert!(matches!(
            s.health(Row::Imu, t + Duration::from_secs(2)),
            Health::Stale(_)
        ));
        let d = s.detail(Row::Imu, t, None);
        assert!(d.contains("+9.81") && d.contains("+5.7 deg/s"), "{d}");
    }

    #[test]
    fn the_big_topics_are_a_rate() {
        let t = Instant::now();
        let mut s = state();
        for i in 0..11 {
            s.apply(
                Update::Counted(Row::Lidar),
                t + Duration::from_millis(100 * i),
            );
        }
        let now = t + Duration::from_millis(1000);
        let hz = s.rate(Row::Lidar, now).expect("ten a second");
        assert!((hz - 10.0).abs() < 1e-9);
        assert_eq!(s.detail(Row::Lidar, now, None), "10.0 Hz");
        assert_eq!(s.rate(Row::Lidar, now + Duration::from_secs(5)), None);
    }

    #[test]
    fn the_source_errs_towards_real() {
        let t = Instant::now();
        let mut s = state();
        assert_eq!(s.kind(), Kind::Unknown);
        s.apply(announce("gazebo_adapter", Origin::Sim), t);
        assert_eq!(s.kind(), Kind::Sim);
        s.apply(announce("legacy_uart_bridge", Origin::Real), t);
        assert_eq!(s.kind(), Kind::Real);
        // An announcer with no entries is still an announcement: nothing real in it.
        let mut s = state();
        s.apply(
            Update::Source(Announcement {
                announcer: "x".into(),
                entries: vec![],
            }),
            t,
        );
        assert_eq!(s.kind(), Kind::Unknown);
    }

    #[test]
    fn the_estop_from_either_place() {
        let t = Instant::now();
        let mut s = state();
        assert!(!s.estop_on());
        s.apply(Update::Estop(true), t);
        assert!(s.estop_on());
        s.apply(Update::Estop(false), t);
        assert!(!s.estop_on());
    }

    #[test]
    fn on_stands_the_front_wheels_say_not_driven() {
        let front = SpeedWheels::Front;
        assert_eq!(
            wheels_note(true, Kind::Real, Some(&front)).as_deref(),
            Some("front, not driven")
        );
        assert_eq!(wheels_note(false, Kind::Real, Some(&front)), None);
        assert_eq!(wheels_note(true, Kind::Sim, Some(&front)), None);
        assert_eq!(
            wheels_note(true, Kind::Real, Some(&SpeedWheels::Rear)),
            None
        );
        let t = Instant::now();
        let mut s = state();
        s.apply(
            Update::Wheels(Wheels {
                front_left: 0.0,
                front_right: 0.01,
                rear_left: f32::NAN,
                rear_right: f32::NAN,
            }),
            t,
        );
        let d = s.detail(Row::Wheels, t, Some("front, not driven"));
        assert_eq!(d, "FL +0.00 FR +0.01  RL - RR - m/s  (front, not driven)");
    }

    #[test]
    fn the_geofence_holds_only_while_fresh_and_true() {
        let t = Instant::now();
        let mut s = state();
        assert!(!s.geofence_holds(t));
        s.apply(Update::GeofenceLock(true), t);
        assert!(s.geofence_holds(t));
        assert!(!s.geofence_holds(t + Duration::from_secs(3)));
    }
}

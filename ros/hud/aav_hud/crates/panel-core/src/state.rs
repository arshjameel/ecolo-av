//! Everything the hardware panel shows, as one value: each sensor's and actuator's
//! latest number from its own node, with OK / STALE / NEVER, and what the command path
//! says (`/system/source`, the e-stop, twist_mux's output, the locks).
//!
//! News comes in as [`Update`]s (the ROS thread or the demo makes them). The big
//! topics (lidar, radar, camera) are only counted: their rate is the number.

use std::{
    collections::{HashMap, VecDeque},
    fmt,
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
    calibration::{Raw, Situation as CalSituation},
    esp32::{Heartbeat, Status, StatusLine},
    graphs::{Channel, Drive, Graphs, PiFrame, UplinkHealth},
    rules::{source_kind, Kind},
    vehicle::SpeedWheels,
};

/// A row of the panel: one sensor or actuator, from its own topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Row {
    /// `/vehicle/status`.
    Vehicle,
    /// The steering actuator.
    Steering,
    /// The throttle.
    Throttle,
    /// The brake actuator.
    Brake,
    /// The wheel speeds.
    Wheels,
    /// The IMU.
    Imu,
    /// The GPS receiver.
    Gps,
    /// The lidar.
    Lidar,
    /// The radar.
    Radar,
    /// The camera.
    Camera,
    /// The battery.
    Battery,
    /// twist_mux's output, `/control/cmd_vel`.
    Mux,
    /// The geofence's lock.
    GeofenceLock,
    /// `/vehicle/steering_actuator/raw`: the calibration page's position.
    SteeringRaw,
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

    /// Its name on the screen.
    #[must_use]
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
            Self::SteeringRaw => "steering raw",
        }
    }
}

/// The car's speed, as far as the panel knows it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Speed {
    /// m/s, fresh.
    Measured(f64),
    /// Not known, and why.
    Unknown(SpeedUnknown),
}

/// Why the speed isn't known.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpeedUnknown {
    /// No `/vehicle/status` yet.
    Never,
    /// `/vehicle/status` went quiet.
    Stale,
    /// It comes, but its speed is NaN: nothing measures it (today's ESP32 uplink).
    NotANumber,
}

impl fmt::Display for SpeedUnknown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Never => "no /vehicle/status yet",
            Self::Stale => "/vehicle/status went quiet",
            Self::NotANumber => "/vehicle/status says NaN: nothing measures the speed",
        })
    }
}

/// One actuator's state (aav_interfaces/ActuatorState): asked and done. NaN = unknown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Actuator {
    /// What was asked.
    pub command: f32,
    /// What was measured.
    pub measured: f32,
}

impl Actuator {
    /// What it did, if known.
    #[must_use]
    pub fn measured(self) -> Option<f32> {
        self.measured.is_finite().then_some(self.measured)
    }
}

/// Each wheel's speed, m/s; NaN = not measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheels {
    /// m/s.
    pub front_left: f32,
    /// m/s.
    pub front_right: f32,
    /// m/s.
    pub rear_left: f32,
    /// m/s.
    pub rear_right: f32,
}

/// One IMU message: m/s² and rad/s, x y z.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    /// m/s², x y z.
    pub accel: [f64; 3],
    /// rad/s, x y z.
    pub gyro: [f64; 3],
}

/// One piece of news.
#[derive(Debug, Clone, PartialEq)]
pub enum Update {
    /// `/vehicle/status`.
    Vehicle(VehicleStatus),
    /// `/vehicle/steering`.
    Steering(Actuator),
    /// `/vehicle/throttle`.
    Throttle(Actuator),
    /// `/vehicle/brake`.
    Brake(Actuator),
    /// `/vehicle/wheels`.
    Wheels(Wheels),
    /// The IMU.
    Imu(ImuReading),
    /// The GPS receiver's fix.
    Gps(Fix),
    /// A big message (lidar, radar, camera) arrived: only counted.
    Counted(Row),
    /// `/vehicle/battery`.
    Battery(Battery),
    /// `/system/source`.
    Source(Announcement),
    /// `/safety/estop`.
    Estop(bool),
    /// `/control/cmd_vel`: twist_mux's output.
    Mux(Command),
    /// The geofence's lock.
    GeofenceLock(bool),
    /// The legacy bridge's lock (no command timeout).
    NoTimeoutLock(bool),
    /// The E-STOP button's message went out: to this many subscribers (None: failed).
    EstopSent(Option<usize>),
    /// `/vehicle/steering_actuator/raw`.
    SteeringRaw(Raw),
    /// `/vehicle/steering_actuator/calibration`: the bridge's line.
    CalibrationLine(String),
    /// `/ego/actuator_cmd`.
    EgoCommand(Drive),
    /// `/control/car/actuator_cmd`: the output mux's.
    CarCommand(Drive),
    /// `/control/car/link_frame`: a frame the Pi wrote.
    LinkFrame(PiFrame),
    /// `/vehicle/esp32/uplink_health`.
    UplinkHealth(UplinkHealth),
    /// The ESP32's WebSocket (`--esp32`): read only.
    Esp32(Esp32News),
}

/// News from the ESP32's WebSocket.
#[derive(Debug, Clone, PartialEq)]
pub enum Esp32News {
    /// Trying to reach this address.
    Connecting(String),
    /// Connected.
    Connected,
    /// One status line.
    Line(StatusLine),
    /// A line that couldn't be read, and why.
    Unreadable(String),
    /// The connection failed or ended, and why; it tries again.
    Lost(String),
}

/// The ESP32's WebSocket, as the panel sees it.
#[derive(Debug, Clone, PartialEq)]
pub enum Esp32Link {
    /// Trying to reach this address.
    Connecting(String),
    /// Connected.
    Connected,
    /// The connection failed or ended, and why.
    Lost(String),
}

/// What the ESP32's status lines said last.
#[derive(Debug, Clone, PartialEq)]
pub struct Esp32Seen {
    /// The connection.
    pub link: Esp32Link,
    /// The last status line, and when.
    pub status: Option<(Instant, Box<Status>)>,
    /// The last heartbeat, and when.
    pub heartbeat: Option<(Instant, Heartbeat)>,
    /// Lines that couldn't be read, and the last one's why.
    pub unreadable: u64,
    /// Why the last unreadable line couldn't be read.
    pub last_problem: Option<String>,
}

/// Messages counted over this long for a rate.
const RATE_OVER: Duration = Duration::from_secs(2);

/// The panel's state.
#[derive(Debug, Clone)]
pub struct PanelState {
    stale_after: Duration,
    seen: HashMap<Row, LastSeen>,
    arrivals: HashMap<Row, VecDeque<Instant>>,
    /// The car's last status.
    pub vehicle: Option<VehicleStatus>,
    /// The steering actuator's last state.
    pub steering: Option<Actuator>,
    /// The throttle's last state.
    pub throttle: Option<Actuator>,
    /// The brake actuator's last state.
    pub brake: Option<Actuator>,
    /// The last wheel speeds.
    pub wheels: Option<Wheels>,
    /// The last IMU reading.
    pub imu: Option<ImuReading>,
    /// The GPS receiver's last fix.
    pub gps: Option<Fix>,
    /// The battery's last state.
    pub battery: Option<Battery>,
    /// Where each sensor's data comes from.
    pub origins: Origins,
    /// Whether anyone announced on `/system/source` yet.
    announced: bool,
    /// `/safety/estop`'s last value.
    pub estop: Option<bool>,
    /// twist_mux's last output, and when.
    pub mux: Option<(Instant, Command)>,
    /// The geofence's lock, last heard.
    pub geofence_lock: Option<bool>,
    /// The legacy bridge's lock, last heard.
    pub no_timeout_lock: Option<bool>,
    /// The E-STOP button's last message: sent to how many (None: failed).
    pub estop_sent: Option<Option<usize>>,
    /// The last raw steering message.
    pub steering_raw: Option<Raw>,
    /// The bridge's last calibration line, and when.
    pub calibration_line: Option<(Instant, String)>,
    /// The GRAPHS page's last 60 s.
    pub graphs: Graphs,
    /// The ESP32's WebSocket, once `--esp32` asked for it.
    pub esp32: Option<Esp32Seen>,
}

impl PanelState {
    /// Nothing heard yet; a topic this quiet shows STALE.
    #[must_use]
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
            steering_raw: None,
            calibration_line: None,
            graphs: Graphs::default(),
            esp32: None,
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
                let command = f64::from(a.command);
                self.graphs
                    .record_uplink_output(now, Channel::Throttle, command);
            }
            Update::Brake(a) => {
                self.brake = Some(a);
                self.heard(Row::Brake, now);
                let command = f64::from(a.command);
                self.graphs
                    .record_uplink_output(now, Channel::Brake, command);
            }
            Update::Wheels(w) => {
                self.wheels = Some(w);
                self.heard(Row::Wheels, now);
            }
            Update::Imu(i) => {
                self.imu = Some(i);
                self.heard(Row::Imu, now);
                self.graphs.record_imu_yaw(now, i.gyro[2].to_degrees());
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
            Update::SteeringRaw(r) => {
                self.steering_raw = Some(r);
                self.heard(Row::SteeringRaw, now);
            }
            Update::CalibrationLine(line) => self.calibration_line = Some((now, line)),
            Update::EgoCommand(d) => self.graphs.record_ego(now, d),
            Update::CarCommand(d) => self.graphs.record_mux(now, d),
            Update::LinkFrame(f) => self.graphs.record_pi(now, f),
            Update::UplinkHealth(h) => self.graphs.record_uplink(now, h),
            Update::Esp32(news) => self.esp32_news(news, now),
        }
    }

    fn esp32_news(&mut self, news: Esp32News, now: Instant) {
        let seen = self.esp32.get_or_insert_with(|| Esp32Seen {
            link: Esp32Link::Connecting(String::new()),
            status: None,
            heartbeat: None,
            unreadable: 0,
            last_problem: None,
        });
        match news {
            Esp32News::Connecting(url) => seen.link = Esp32Link::Connecting(url),
            Esp32News::Connected => seen.link = Esp32Link::Connected,
            Esp32News::Lost(why) => seen.link = Esp32Link::Lost(why),
            Esp32News::Unreadable(why) => {
                seen.unreadable += 1;
                seen.last_problem = Some(why);
            }
            Esp32News::Line(line) => {
                self.graphs.record_esp32(now, &line);
                match line {
                    StatusLine::Status(s) => seen.status = Some((now, s)),
                    StatusLine::Heartbeat(h) => seen.heartbeat = Some((now, h)),
                    StatusLine::Unknown(u) => seen.last_problem = Some(u.to_string()),
                    StatusLine::Event(_) => {}
                }
            }
        }
    }

    /// One row's health at `now`.
    #[must_use]
    pub fn health(&self, row: Row, now: Instant) -> Health {
        self.seen
            .get(&row)
            .copied()
            .unwrap_or_default()
            .health(now, self.stale_after)
    }

    /// Messages a second over the last 2 s, by arrival.
    #[must_use]
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
    #[must_use]
    pub fn kind(&self) -> Kind {
        if !self.announced {
            return Kind::Unknown;
        }
        source_kind(self.origins.entries().map(|e| e.origin))
    }

    /// The e-stop is on: `/safety/estop` said true last, or `/vehicle/status` says so.
    #[must_use]
    pub fn estop_on(&self) -> bool {
        self.estop == Some(true) || self.vehicle.is_some_and(|v| v.estop_active)
    }

    /// The geofence's lock holds the buttons (51 is above their 45).
    #[must_use]
    pub fn geofence_holds(&self, now: Instant) -> bool {
        self.geofence_lock == Some(true) && self.health(Row::GeofenceLock, now).is_ok()
    }

    /// The speed `/vehicle/status` gives: measured, or why it isn't known. NaN is
    /// unknown, never stopped: the legacy ESP32's uplink has no wheel speed, so its
    /// bridge sends NaN all the time (review 2, M5).
    #[must_use]
    pub fn speed(&self, now: Instant) -> Speed {
        let Some(v) = self.vehicle else {
            return Speed::Unknown(SpeedUnknown::Never);
        };
        if !self.health(Row::Vehicle, now).is_ok() {
            return Speed::Unknown(SpeedUnknown::Stale);
        }
        let speed = f64::from(v.speed.0);
        if speed.is_finite() {
            Speed::Measured(speed)
        } else {
            Speed::Unknown(SpeedUnknown::NotANumber)
        }
    }

    /// The measured speed, m/s (`/vehicle/status`), while fresh and a number.
    #[must_use]
    pub fn seen_speed(&self, now: Instant) -> Option<f64> {
        match self.speed(now) {
            Speed::Measured(v) => Some(v),
            Speed::Unknown(_) => None,
        }
    }

    /// The measured steering, rad (`/vehicle/steering`), while fresh.
    #[must_use]
    pub fn seen_steer(&self, now: Instant) -> Option<f64> {
        let m = self.steering?.measured()?;
        self.health(Row::Steering, now)
            .is_ok()
            .then_some(f64::from(m))
    }

    /// The raw position message, while fresh.
    #[must_use]
    pub fn fresh_raw(&self, now: Instant) -> Option<Raw> {
        self.steering_raw
            .filter(|_| self.health(Row::SteeringRaw, now).is_ok())
    }

    /// What the calibration page needs to know to be allowed. `still_confirmed`: a
    /// person pressed ON STANDS, STILL (the page's [`crate::calibration::CalPage`]).
    #[must_use]
    pub fn calibration_situation(&self, now: Instant, still_confirmed: bool) -> CalSituation {
        CalSituation {
            estop: self.estop_on(),
            speed: self.speed(now),
            still_confirmed,
            raw: self.fresh_raw(now),
        }
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
            Row::SteeringRaw => self.steering_raw.map_or_else(String::new, |r| {
                let at = r
                    .position
                    .map_or_else(|| "no position".to_owned(), |p| format!("{p} counts"));
                format!("{at}, {}", r.state)
            }),
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
#[must_use]
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
        s.apply(announce("sim_adapter", Origin::Sim), t);
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

    #[test]
    fn the_calibration_page_sees_only_a_fresh_position() {
        use crate::calibration::{readiness, FirmwareState, NotReady};
        let t = Instant::now();
        let mut s = state();
        let raw = Raw {
            position: Some(2010),
            state: FirmwareState::Limp,
        };
        s.apply(Update::SteeringRaw(raw), t);
        assert_eq!(s.fresh_raw(t), Some(raw));
        assert_eq!(
            s.detail(Row::SteeringRaw, t, None),
            "2010 counts, LIMP (the motor off)"
        );
        let later = t + Duration::from_secs(2);
        assert_eq!(s.fresh_raw(later), None);
        let why = readiness(&s.calibration_situation(later, false));
        assert_eq!(
            why,
            vec![NotReady::SpeedUnknown(SpeedUnknown::Never), NotReady::NoRaw]
        );
    }

    #[test]
    fn the_esp32s_news_reaches_the_graphs_and_its_own_state() {
        use crate::esp32::parse_line;
        let t = Instant::now();
        let mut s = state();
        assert!(s.esp32.is_none(), "nothing until --esp32");
        s.apply(Update::Esp32(Esp32News::Connecting("ws://h:81/".into())), t);
        assert_eq!(
            s.esp32.as_ref().map(|e| &e.link),
            Some(&Esp32Link::Connecting("ws://h:81/".into()))
        );
        s.apply(Update::Esp32(Esp32News::Connected), t);
        let golden = include_str!("../../../../../../firmware/status/golden.jsonl");
        for line in golden.lines() {
            let l = parse_line(line).expect("golden");
            s.apply(Update::Esp32(Esp32News::Line(l)), t);
        }
        s.apply(Update::Esp32(Esp32News::Unreadable("not JSON".into())), t);
        let e = s.esp32.as_ref().expect("seen");
        assert_eq!(e.link, Esp32Link::Connected);
        assert!(e.status.is_some() && e.heartbeat.is_some());
        assert_eq!(e.unreadable, 1);
        let c = s.graphs.counts(t);
        assert_eq!(
            c.esp32.map(|(total, _)| total.frames),
            Some(0),
            "the last line's"
        );
        assert_eq!(c.lines_lost, 0);
        // Nothing here changes what the buttons or the calibration page see.
        assert_eq!(s.speed(t), Speed::Unknown(SpeedUnknown::Never));
        assert!(!s.estop_on());
    }

    fn status(speed: f32) -> Update {
        use hud_core::{
            units::{Fraction, MetresPerSecond, Radians},
            vehicle::{Faults, Mode},
        };
        Update::Vehicle(VehicleStatus {
            mode: Mode::from(2),
            armed: true,
            estop_active: false,
            speed: MetresPerSecond(speed),
            steering_angle: Radians(0.0),
            throttle: Fraction::new(0.0),
            brake: Fraction::new(1.0),
            faults: Faults(0),
        })
    }

    #[test]
    fn a_nan_speed_is_unknown_never_stopped() {
        let t = Instant::now();
        let mut s = state();
        assert_eq!(s.speed(t), Speed::Unknown(SpeedUnknown::Never));
        // Today's ESP32 uplink: the bridge sends NaN, all the time.
        s.apply(status(f32::NAN), t);
        assert_eq!(s.speed(t), Speed::Unknown(SpeedUnknown::NotANumber));
        assert_eq!(s.seen_speed(t), None);
        s.apply(status(f32::INFINITY), t);
        assert_eq!(s.seen_speed(t), None);
        s.apply(status(0.0), t);
        assert_eq!(s.speed(t), Speed::Measured(0.0));
        assert_eq!(s.seen_speed(t), Some(0.0));
        assert_eq!(
            s.speed(t + Duration::from_secs(2)),
            Speed::Unknown(SpeedUnknown::Stale)
        );
    }

    #[test]
    fn a_nan_speed_never_lets_the_calibration_page_ask_on_its_own() {
        use crate::calibration::{allowance, readiness, Allowance, FirmwareState, NotReady};
        let t = Instant::now();
        let mut s = state();
        s.apply(status(f32::NAN), t);
        s.apply(
            Update::SteeringRaw(Raw {
                position: Some(2000),
                state: FirmwareState::Off,
            }),
            t,
        );
        let why = readiness(&s.calibration_situation(t, false));
        assert_eq!(why, vec![NotReady::SpeedUnknown(SpeedUnknown::NotANumber)]);
        // A person says ON STANDS, STILL: the software allows it, on their word.
        let confirmed = s.calibration_situation(t, true);
        assert!(readiness(&confirmed).is_empty());
        assert!(matches!(
            allowance(&confirmed),
            Allowance::SoftwareAllows {
                speed: crate::calibration::StillBy::PersonSaid
            }
        ));
    }
}

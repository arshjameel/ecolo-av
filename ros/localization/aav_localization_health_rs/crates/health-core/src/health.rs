//! Every check's state, fed message by message (`health.py`'s `Health`).
//!
//! ```text
//! imu / odom / gps_fix / wheels / ekf / tf ──▶ Health ──▶ report(now) ──▶ 8 Status
//!                                                                └──▶ summary()
//! ```
//!
//! Warn only: nothing here stops the car or changes what the EKFs see.

use std::cmp::Ordering;
use std::collections::{HashMap, VecDeque};
use std::fmt;

use ego_core::hypot;
use watchdog_core::{py_max, py_min, py_sum};

use crate::{degrees, fmt_g, radians, G};

/// A check's level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// Believable.
    Ok,
    /// Something to look at.
    Warn,
    /// An input it needs has gone quiet.
    Stale,
}

impl Level {
    /// The Python's word: `OK`, `WARN` or `STALE`.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Ok => "OK",
            Level::Warn => "WARN",
            Level::Stale => "STALE",
        }
    }
}

impl fmt::Display for Level {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The eight checks, in the report's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Check {
    /// Parked, the gyro's mean turn rate.
    GyroBias,
    /// Parked, gravity in the IMU's frame.
    ImuMounting,
    /// The newest fix: a fix, a covariance, a believable sigma.
    GpsQuality,
    /// Each input's stamp against this node's clock.
    Timestamps,
    /// Each input's frame.
    Frames,
    /// base_link -> imu_link, gps_link; map -> base_link fresh.
    Tf,
    /// The front wheels against each other and the turn.
    Wheels,
    /// Each EKF's output: nothing the car can't do.
    Motion,
}

/// Every check, in `CHECKS` order.
pub const CHECKS: [Check; 8] = [
    Check::GyroBias,
    Check::ImuMounting,
    Check::GpsQuality,
    Check::Timestamps,
    Check::Frames,
    Check::Tf,
    Check::Wheels,
    Check::Motion,
];

impl Check {
    /// The Python's name (`gyro_bias`...).
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Check::GyroBias => "gyro_bias",
            Check::ImuMounting => "imu_mounting",
            Check::GpsQuality => "gps_quality",
            Check::Timestamps => "timestamps",
            Check::Frames => "frames",
            Check::Tf => "tf",
            Check::Wheels => "wheels",
            Check::Motion => "motion",
        }
    }

    /// The inputs it can't do without (`NEEDS`).
    fn needs(self) -> &'static [Input] {
        match self {
            Check::GyroBias | Check::ImuMounting => &[Input::Imu, Input::Odom],
            Check::GpsQuality => &[Input::Gps],
            Check::Wheels => &[Input::Wheels, Input::Imu, Input::Odom],
            Check::Motion => &[Input::Local, Input::Global],
            Check::Timestamps | Check::Frames | Check::Tf => &[],
        }
    }
}

/// What the node hears.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Input {
    /// `/sensors/imu/data`.
    Imu,
    /// `/sensors/gps/fix`.
    Gps,
    /// `/vehicle/odom`.
    Odom,
    /// `/vehicle/wheels`.
    Wheels,
    /// `/localization/odom/local`.
    Local,
    /// `/localization/odom/global`.
    Global,
}

impl Input {
    /// The Python's name.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Input::Imu => "imu",
            Input::Gps => "gps",
            Input::Odom => "odom",
            Input::Wheels => "wheels",
            Input::Local => "local",
            Input::Global => "global",
        }
    }

    /// The frame it must be in, and its child frame where it has one (`FRAMES`).
    fn frames(self) -> (&'static str, Option<&'static str>) {
        match self {
            Input::Imu => ("imu_link", None),
            Input::Gps => ("gps_link", None),
            Input::Odom | Input::Local => ("odom", Some("base_link")),
            Input::Wheels => ("base_link", None),
            Input::Global => ("map", Some("base_link")),
        }
    }

    /// One whose silence (or absence) makes its checks STALE (`REQUIRED`).
    fn required(self) -> bool {
        matches!(self, Input::Imu | Input::Odom | Input::Local)
    }
}

/// An EKF's output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ekf {
    /// ekf_local: never jumps, by design.
    Local,
    /// ekf_global: may jump (GPS corrections).
    Global,
}

impl Ekf {
    fn input(self) -> Input {
        match self {
            Ekf::Local => Input::Local,
            Ekf::Global => Input::Global,
        }
    }
}

/// The thresholds: `config/localization_health.yaml`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// s: a needed input this quiet is STALE.
    pub stale_after: f64,
    /// m/s: slower is parked.
    pub still_speed: f64,
    /// s parked before the gyro and gravity are judged.
    pub still_min: f64,
    /// s: the newest parked samples used.
    pub still_window: f64,
    /// rad/s.
    pub gyro_bias_warn: f64,
    /// rad.
    pub tilt_warn: f64,
    /// m/s².
    pub gravity_tolerance: f64,
    /// m.
    pub gps_sigma_max: f64,
    /// m.
    pub gps_sigma_min: f64,
    /// s: a stamp this much older than now.
    pub skew_behind: f64,
    /// s: a stamp from the future.
    pub skew_ahead: f64,
    /// m/s.
    pub wheel_min_speed: f64,
    /// m/s, plus `wheel_share` of the speed.
    pub wheel_tolerance: f64,
    /// Of the speed.
    pub wheel_share: f64,
    /// s a wheel gap must last.
    pub persist: f64,
    /// m/s.
    pub max_speed: f64,
    /// rad/s.
    pub max_yaw_rate: f64,
    /// m/s².
    pub max_accel: f64,
    /// m an EKF may move beyond `max_speed` in a message.
    pub step_margin: f64,
    /// s a motion warning stays up.
    pub hold: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            stale_after: 2.0,
            still_speed: 0.02,
            still_min: 5.0,
            still_window: 10.0,
            gyro_bias_warn: radians(0.2),
            tilt_warn: radians(10.0),
            gravity_tolerance: 1.0,
            gps_sigma_max: 5.0,
            gps_sigma_min: 0.005,
            skew_behind: 0.5,
            skew_ahead: 0.1,
            wheel_min_speed: 0.3,
            wheel_tolerance: 0.15,
            wheel_share: 0.1,
            persist: 1.0,
            max_speed: 5.0,
            max_yaw_rate: 2.0,
            max_accel: 6.0,
            step_margin: 2.0,
            hold: 5.0,
        }
    }
}

impl Settings {
    /// Every setting by its parameter's name, in the Python dataclass's order.
    #[must_use]
    pub fn fields(&self) -> [(&'static str, f64); 20] {
        [
            ("stale_after", self.stale_after),
            ("still_speed", self.still_speed),
            ("still_min", self.still_min),
            ("still_window", self.still_window),
            ("gyro_bias_warn", self.gyro_bias_warn),
            ("tilt_warn", self.tilt_warn),
            ("gravity_tolerance", self.gravity_tolerance),
            ("gps_sigma_max", self.gps_sigma_max),
            ("gps_sigma_min", self.gps_sigma_min),
            ("skew_behind", self.skew_behind),
            ("skew_ahead", self.skew_ahead),
            ("wheel_min_speed", self.wheel_min_speed),
            ("wheel_tolerance", self.wheel_tolerance),
            ("wheel_share", self.wheel_share),
            ("persist", self.persist),
            ("max_speed", self.max_speed),
            ("max_yaw_rate", self.max_yaw_rate),
            ("max_accel", self.max_accel),
            ("step_margin", self.step_margin),
            ("hold", self.hold),
        ]
    }

    /// Settings from (name, value) pairs; a name it doesn't know is `None`.
    #[must_use]
    pub fn from_fields(fields: &[(&str, f64)]) -> Option<Self> {
        let mut s = Settings::default();
        for &(name, v) in fields {
            let slot = match name {
                "stale_after" => &mut s.stale_after,
                "still_speed" => &mut s.still_speed,
                "still_min" => &mut s.still_min,
                "still_window" => &mut s.still_window,
                "gyro_bias_warn" => &mut s.gyro_bias_warn,
                "tilt_warn" => &mut s.tilt_warn,
                "gravity_tolerance" => &mut s.gravity_tolerance,
                "gps_sigma_max" => &mut s.gps_sigma_max,
                "gps_sigma_min" => &mut s.gps_sigma_min,
                "skew_behind" => &mut s.skew_behind,
                "skew_ahead" => &mut s.skew_ahead,
                "wheel_min_speed" => &mut s.wheel_min_speed,
                "wheel_tolerance" => &mut s.wheel_tolerance,
                "wheel_share" => &mut s.wheel_share,
                "persist" => &mut s.persist,
                "max_speed" => &mut s.max_speed,
                "max_yaw_rate" => &mut s.max_yaw_rate,
                "max_accel" => &mut s.max_accel,
                "step_margin" => &mut s.step_margin,
                "hold" => &mut s.hold,
                _ => return None,
            };
            *slot = v;
        }
        Some(s)
    }
}

/// One check's verdict.
#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    /// Which check.
    pub check: Check,
    /// OK, WARN or STALE.
    pub level: Level,
    /// A few words for a person.
    pub message: String,
    /// The numbers behind it, in the Python dict's order.
    pub values: Vec<(String, f64)>,
}

impl Status {
    fn new(check: Check, level: Level, message: impl Into<String>) -> Self {
        Status {
            check,
            level,
            message: message.into(),
            values: Vec::new(),
        }
    }

    fn with(mut self, values: &[(&str, f64)]) -> Self {
        self.values = values.iter().map(|&(k, v)| (k.to_string(), v)).collect();
        self
    }
}

/// The left and right front wheels' ground speeds (m/s) for the car's motion.
#[must_use]
pub fn front_wheel_speeds(v: f64, yaw_rate: f64, wheelbase: f64, track: f64) -> (f64, f64) {
    let left = hypot(v - yaw_rate * track / 2.0, yaw_rate * wheelbase);
    let right = hypot(v + yaw_rate * track / 2.0, yaw_rate * wheelbase);
    (left, right)
}

/// The mean, and the standard deviation of the mean (0 for one value). `None` for none.
#[must_use]
pub fn mean_and_sigma(values: &[f64]) -> Option<(f64, f64)> {
    let n = values.len();
    if n == 0 {
        return None;
    }
    let mean = py_sum(values.iter().copied()) / n as f64;
    if n < 2 {
        return Some((mean, 0.0));
    }
    let var = py_sum(values.iter().map(|x| (x - mean) * (x - mean))) / (n - 1) as f64;
    Some((mean, (var / n as f64).sqrt()))
}

/// A parked IMU sample: (t, wz, ax, ay, az).
type Parked = (f64, f64, f64, f64, f64);

/// An EKF's last output: (stamp, x, y, v).
type Pose = (f64, f64, f64, f64);

/// Every check's state, fed message by message.
#[derive(Debug, Clone)]
pub struct Health {
    wheelbase: f64,
    track: f64,
    s: Settings,
    /// Input -> when it last arrived.
    seen: HashMap<Input, f64>,
    /// Input -> now - stamp, newest; in the order each first arrived (a Python dict).
    skew: Vec<(Input, f64)>,
    bad_frames: HashMap<Input, String>,
    speed: Option<f64>,
    gyro_z: Option<f64>,
    parked: VecDeque<Parked>,
    gps: Option<Status>,
    wheel_gap: f64,
    wheel_bad_since: Option<f64>,
    last_pose: HashMap<Input, Pose>,
    /// EKF -> (until, why).
    motion_bad: HashMap<Input, (f64, String)>,
    /// (static frames missing, map -> base_link's age).
    tf_state: Option<(Vec<String>, Option<f64>)>,
}

impl Health {
    /// A car of this wheelbase and front track (m), with these thresholds.
    #[must_use]
    pub fn new(wheelbase: f64, track: f64, settings: Settings) -> Self {
        Health {
            wheelbase,
            track,
            s: settings,
            seen: HashMap::new(),
            skew: Vec::new(),
            bad_frames: HashMap::new(),
            speed: None,
            gyro_z: None,
            parked: VecDeque::new(),
            gps: None,
            wheel_gap: 0.0,
            wheel_bad_since: None,
            last_pose: HashMap::new(),
            motion_bad: HashMap::new(),
            tf_state: None,
        }
    }

    /// The IMU samples kept while parked.
    #[must_use]
    pub fn parked_samples(&self) -> usize {
        self.parked.len()
    }

    // --- what arrived ------------------------------------------------------------------

    fn arrived(&mut self, name: Input, now: f64, stamp: f64, frame: &str, child: Option<&str>) {
        self.seen.insert(name, now);
        match self.skew.iter_mut().find(|(n, _)| *n == name) {
            Some(slot) => slot.1 = now - stamp,
            None => self.skew.push((name, now - stamp)),
        }
        let (want, want_child) = name.frames();
        let child_wrong = want_child.is_some_and(|w| child != Some(w));
        if frame != want || child_wrong {
            let got = match want_child {
                None => frame.to_string(),
                Some(_) => format!("{frame} -> {}", child.unwrap_or("")),
            };
            let got = if got.is_empty() { "(none)".into() } else { got };
            let ask = match want_child {
                None => want.to_string(),
                Some(c) => format!("{want} -> {c}"),
            };
            self.bad_frames
                .insert(name, format!("{} in {got}, not {ask}", name.as_str()));
        } else {
            self.bad_frames.remove(&name);
        }
    }

    /// An IMU message: its turn rate (rad/s) and acceleration (m/s²).
    pub fn imu(&mut self, now: f64, stamp: f64, frame: &str, wz: f64, accel: [f64; 3]) {
        self.arrived(Input::Imu, now, stamp, frame, None);
        self.gyro_z = Some(wz);
        if self.speed.is_some_and(|v| v.abs() < self.s.still_speed) {
            self.parked
                .push_back((now, wz, accel[0], accel[1], accel[2]));
            while self
                .parked
                .front()
                .is_some_and(|p| p.0 < now - self.s.still_window)
            {
                self.parked.pop_front();
            }
        } else {
            self.parked.clear();
        }
    }

    /// The wheels' odometry: the car's speed (m/s).
    pub fn odom(&mut self, now: f64, stamp: f64, frame: &str, child: &str, v: f64) {
        self.arrived(Input::Odom, now, stamp, frame, Some(child));
        self.speed = Some(v);
        if v.abs() >= self.s.still_speed {
            self.parked.clear();
        }
    }

    /// A NavSatFix: its status, covariance type (0 unknown) and covariance (9 numbers).
    pub fn gps_fix(
        &mut self,
        now: f64,
        stamp: f64,
        frame: &str,
        status: i8,
        covariance_type: u8,
        covariance: &[f64; 9],
    ) {
        self.arrived(Input::Gps, now, stamp, frame, None);
        let sigma = py_max(py_max(covariance[0], covariance[4]), 0.0).sqrt();
        let values = [("sigma_m", sigma), ("status", f64::from(status))];
        let s = &self.s;
        let (level, message) = if status < 0 {
            (Level::Warn, "no fix".to_string())
        } else if covariance_type == 0 {
            (
                Level::Warn,
                "covariance unknown: the EKF guesses how far to trust it".to_string(),
            )
        } else if sigma > s.gps_sigma_max {
            (Level::Warn, format!("poor fix: sigma {sigma:.1} m"))
        } else if sigma < s.gps_sigma_min {
            (
                Level::Warn,
                format!("claims sigma {:.2} cm: too sure", sigma * 100.0),
            )
        } else {
            (Level::Ok, format!("sigma {sigma:.2} m"))
        };
        self.gps = Some(Status::new(Check::GpsQuality, level, message).with(&values));
    }

    /// The front wheels' speeds (m/s); NaN is a wheel not measured.
    pub fn wheels(&mut self, now: f64, stamp: f64, frame: &str, left: f64, right: f64) {
        self.arrived(Input::Wheels, now, stamp, frame, None);
        let (Some(speed), Some(gyro_z)) = (self.speed, self.gyro_z) else {
            self.wheel_bad_since = None;
            return;
        };
        // `abs(speed) < min` is false for NaN, as the Python's.
        if left.is_nan() || right.is_nan() || speed.abs() < self.s.wheel_min_speed {
            self.wheel_bad_since = None;
            return;
        }
        let (want_l, want_r) = front_wheel_speeds(speed.abs(), gyro_z, self.wheelbase, self.track);
        self.wheel_gap = ((right.abs() - left.abs()) - (want_r - want_l)).abs();
        if self.wheel_gap > self.s.wheel_tolerance + self.s.wheel_share * speed.abs() {
            if self.wheel_bad_since.is_none() {
                self.wheel_bad_since = Some(now);
            }
        } else {
            self.wheel_bad_since = None;
        }
    }

    /// An EKF's output: its position, speed and turn rate.
    #[allow(clippy::too_many_arguments)]
    pub fn ekf(
        &mut self,
        which: Ekf,
        now: f64,
        stamp: f64,
        frame: &str,
        child: &str,
        position: (f64, f64),
        v: f64,
        yaw_rate: f64,
    ) {
        let name = which.input();
        let (x, y) = position;
        self.arrived(name, now, stamp, frame, Some(child));
        let s = &self.s;
        let mut why = None;
        if v.abs() > s.max_speed {
            why = Some(format!("{v:.1} m/s"));
        } else if yaw_rate.abs() > s.max_yaw_rate {
            why = Some(format!("turning {yaw_rate:.1} rad/s"));
        }
        if let Some(&(t0, x0, y0, v0)) = self.last_pose.get(&name) {
            if why.is_none() && stamp - t0 > 1e-3 {
                let dt = stamp - t0;
                let accel = (v - v0) / dt;
                let step = hypot(x - x0, y - y0);
                if accel.abs() > s.max_accel && dt >= 0.05 {
                    why = Some(format!("{accel:.1} m/s^2"));
                } else if which == Ekf::Local && step > s.max_speed * dt + s.step_margin {
                    // Only ekf_local promises never to jump; ekf_global's jumps are GPS
                    // corrections (metres, after a dropout), and its gate bounds them.
                    why = Some(format!("jumped {step:.1} m"));
                }
            }
        }
        self.last_pose.insert(name, (stamp, x, y, v));
        if let Some(why) = why {
            self.motion_bad.insert(name, (now + s.hold, why));
        }
    }

    /// The TF check: static frames missing, and map -> base_link's age (`None`: no tree).
    pub fn tf(&mut self, missing: Vec<String>, tree_age: Option<f64>) {
        self.tf_state = Some((missing, tree_age));
    }

    // --- the verdicts ------------------------------------------------------------------

    fn stale(&self, now: f64, check: Check) -> Option<String> {
        for &name in check.needs() {
            match self.seen.get(&name) {
                Some(&seen) if now - seen > self.s.stale_after => {
                    return Some(format!(
                        "nothing on {} for {:.1} s",
                        name.as_str(),
                        now - seen
                    ));
                }
                Some(_) => {}
                None if name.required() => return Some(format!("no {} yet", name.as_str())),
                None => {}
            }
        }
        None
    }

    fn not_here(&self, check: Check) -> bool {
        check
            .needs()
            .iter()
            .any(|n| !self.seen.contains_key(n) && !n.required())
    }

    fn gyro_and_gravity(&self, now: f64) -> (Status, Status) {
        let s = &self.s;
        let enough = self
            .parked
            .front()
            .is_some_and(|p| now - p.0 >= s.still_min)
            && self.parked.len() >= 20;
        let wz: Vec<f64> = self.parked.iter().map(|p| p.1).collect();
        // mean_and_sigma is None only for no samples, which isn't enough either.
        let Some((bias, sigma)) = mean_and_sigma(&wz).filter(|_| enough) else {
            let wait = format!("judged after {} s parked", fmt_g(s.still_min));
            return (
                Status::new(Check::GyroBias, Level::Ok, wait.clone()),
                Status::new(Check::ImuMounting, Level::Ok, wait),
            );
        };
        let bias_dps = degrees(bias);
        let values = [
            ("bias_dps", bias_dps),
            ("sigma_dps", degrees(sigma)),
            ("drift_deg_per_min", bias_dps * 60.0),
        ];
        let gyro = if bias.abs() > s.gyro_bias_warn && bias.abs() > 3.0 * sigma {
            Status::new(
                Check::GyroBias,
                Level::Warn,
                format!(
                    "{bias_dps:+.2} deg/s parked: the heading drifts {:.0} deg a minute \
                     without GPS",
                    bias_dps.abs() * 60.0
                ),
            )
        } else {
            Status::new(Check::GyroBias, Level::Ok, format!("{bias_dps:+.3} deg/s"))
        }
        .with(&values);
        let n = self.parked.len() as f64;
        let gx = py_sum(self.parked.iter().map(|p| p.2)) / n;
        let gy = py_sum(self.parked.iter().map(|p| p.3)) / n;
        let gz = py_sum(self.parked.iter().map(|p| p.4)) / n;
        let size = (gx * gx + gy * gy + gz * gz).sqrt();
        let tilt = if size > 0.0 {
            py_max(-1.0, py_min(1.0, gz / size)).acos()
        } else {
            std::f64::consts::PI
        };
        let tilt_deg = degrees(tilt);
        let gvalues = [
            ("gravity_x", gx),
            ("gravity_y", gy),
            ("gravity_z", gz),
            ("tilt_deg", tilt_deg),
        ];
        let mount = if gz < 0.0 {
            Status::new(
                Check::ImuMounting,
                Level::Warn,
                format!("upside down: gravity z {gz:+.1} m/s^2 (imu_link rpy?)"),
            )
        } else if (size - G).abs() > s.gravity_tolerance {
            Status::new(
                Check::ImuMounting,
                Level::Warn,
                format!("gravity reads {size:.1} m/s^2, not {G:.1}"),
            )
        } else if tilt > s.tilt_warn {
            Status::new(
                Check::ImuMounting,
                Level::Warn,
                format!("tilted {tilt_deg:.0} deg (imu_link rpy?)"),
            )
        } else {
            Status::new(
                Check::ImuMounting,
                Level::Ok,
                format!("level within {tilt_deg:.1} deg"),
            )
        }
        .with(&gvalues);
        (gyro, mount)
    }

    /// Every check's status, in [`CHECKS`] order.
    #[must_use]
    pub fn report(&self, now: f64) -> Vec<Status> {
        let s = &self.s;
        let (gyro, mount) = self.gyro_and_gravity(now);
        let gps = self
            .gps
            .clone()
            .unwrap_or_else(|| Status::new(Check::GpsQuality, Level::Ok, "not on this profile"));
        let fresh: Vec<(Input, f64)> = self
            .skew
            .iter()
            .copied()
            .filter(|(n, _)| self.seen.get(n).is_some_and(|&t| now - t <= s.stale_after))
            .collect();
        let mut late: Vec<(Input, f64)> = fresh
            .iter()
            .copied()
            .filter(|&(_, k)| k > s.skew_behind || k < -s.skew_ahead)
            .collect();
        late.sort_by(|a, b| a.0.as_str().cmp(b.0.as_str()));
        let timestamps = Status {
            check: Check::Timestamps,
            level: if late.is_empty() {
                Level::Ok
            } else {
                Level::Warn
            },
            message: if late.is_empty() {
                format!("{} inputs on time", fresh.len())
            } else {
                late.iter()
                    .map(|(n, k)| format!("{} stamped {k:+.2} s off", n.as_str()))
                    .collect::<Vec<_>>()
                    .join("; ")
            },
            values: fresh
                .iter()
                .map(|(n, k)| (format!("{}_s", n.as_str()), *k))
                .collect(),
        };
        let mut bad_frames: Vec<&String> = self.bad_frames.values().collect();
        bad_frames.sort();
        let frames = if bad_frames.is_empty() {
            Status::new(Check::Frames, Level::Ok, "as expected")
        } else {
            let words: Vec<&str> = bad_frames.iter().map(|w| w.as_str()).collect();
            Status::new(Check::Frames, Level::Warn, words.join("; "))
        };
        let tf = match &self.tf_state {
            None => Status::new(Check::Tf, Level::Ok, "not looked yet"),
            Some((missing, _)) if !missing.is_empty() => Status::new(
                Check::Tf,
                Level::Warn,
                format!("no base_link -> {}", missing.join(", ")),
            ),
            // `age > stale_after` is the Python's test: a NaN age passes it.
            Some((_, Some(age))) if age.partial_cmp(&s.stale_after) != Some(Ordering::Greater) => {
                Status::new(
                    Check::Tf,
                    Level::Ok,
                    format!("map -> base_link {age:.2} s old"),
                )
                .with(&[("tree_age_s", *age)])
            }
            Some(_) => Status::new(Check::Tf, Level::Warn, "map -> base_link missing or old"),
        };
        let wheels = match self.wheel_bad_since {
            Some(since) if now - since >= s.persist => Status::new(
                Check::Wheels,
                Level::Warn,
                format!(
                    "front wheels {:.2} m/s apart from what the turn says",
                    self.wheel_gap
                ),
            ),
            _ => Status::new(Check::Wheels, Level::Ok, "front wheels agree"),
        }
        .with(&[("gap", self.wheel_gap)]);
        let mut bad: Vec<(Input, &str)> = self
            .motion_bad
            .iter()
            .filter(|(_, (until, _))| now < *until)
            .map(|(n, (_, why))| (*n, why.as_str()))
            .collect();
        bad.sort_by(|a, b| a.0.as_str().cmp(b.0.as_str()));
        let motion = if bad.is_empty() {
            Status::new(Check::Motion, Level::Ok, "believable")
        } else {
            let words: Vec<String> = bad
                .iter()
                .map(|(n, w)| format!("ekf_{}: {w}", n.as_str()))
                .collect();
            Status::new(Check::Motion, Level::Warn, words.join("; "))
        };
        let mut out = [gyro, mount, gps, timestamps, frames, tf, wheels, motion];
        for status in &mut out {
            let check = status.check;
            if let Some(why) = self.stale(now, check) {
                *status = Status::new(check, Level::Stale, why);
            } else if self.not_here(check) && status.level == Level::Ok && check != Check::Motion {
                *status = Status::new(check, Level::Ok, "not on this profile");
            }
        }
        out.into()
    }
}

/// OK, or WARN and every check that isn't OK.
#[must_use]
pub fn summary(statuses: &[Status]) -> String {
    let bad: Vec<String> = statuses
        .iter()
        .filter(|s| s.level != Level::Ok)
        .map(|s| format!("{} {}: {}", s.check.as_str(), s.level, s.message))
        .collect();
    if bad.is_empty() {
        "OK".into()
    } else {
        format!("WARN {}", bad.join("; "))
    }
}

//! Each input's watch, and the verdict (`aav_safety/watchdog.py`: `Rule`, `Watch`,
//! `Watchdog`). What each state means: the crate's front page.

use std::fmt;
use std::str::FromStr;

use ego_core::fmt_g;

use crate::wheels::{radians, CrossCheck, WheelCheck, EARTH_RADIUS};
use crate::{hypot, py_max};

/// m/s: faster than this, a camera's picture or a cloud (the lidar's) must change.
pub const MOVING: f64 = 0.2;
/// s: the overspeed check's look back, for "is it slowing?".
pub const SLOWING_WINDOW: f64 = 0.5;

/// Which message an input is, and what is in it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    /// nav_msgs/Odometry.
    Odom,
    /// sensor_msgs/Imu.
    Imu,
    /// sensor_msgs/NavSatFix.
    Gps,
    /// sensor_msgs/Image.
    Image,
    /// sensor_msgs/PointCloud2.
    Cloud,
    /// std_msgs/UInt64: a program's frame count.
    Heartbeat,
    /// vision_msgs/Detection2DArray, one camera's by frame_id.
    Detections,
    /// One of our nodes, in the ROS graph.
    Node,
    /// aav_interfaces/ActuatorState: command against measured.
    Actuator,
    /// ackermann_msgs/AckermannDriveStamped: ego's speed against the wheels'.
    Speed,
    /// diagnostic_msgs/DiagnosticStatus: one of our nodes in its own words (TODO 171).
    Status,
}

impl Kind {
    /// Every kind, in the Python's order (`KINDS`).
    pub const ALL: [Kind; 11] = [
        Kind::Odom,
        Kind::Imu,
        Kind::Gps,
        Kind::Image,
        Kind::Cloud,
        Kind::Heartbeat,
        Kind::Detections,
        Kind::Node,
        Kind::Actuator,
        Kind::Speed,
        Kind::Status,
    ];

    /// Its name in the YAML.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Kind::Odom => "odom",
            Kind::Imu => "imu",
            Kind::Gps => "gps",
            Kind::Image => "image",
            Kind::Cloud => "cloud",
            Kind::Heartbeat => "heartbeat",
            Kind::Detections => "detections",
            Kind::Node => "node",
            Kind::Actuator => "actuator",
            Kind::Speed => "speed",
            Kind::Status => "status",
        }
    }
}

impl Kind {
    /// Whether its FROZEN check counts only while the car moves (`MOVING_KINDS`): a
    /// camera's picture, a lidar's cloud.
    #[must_use]
    pub fn frozen_only_moving(self) -> bool {
        matches!(self, Kind::Image | Kind::Cloud)
    }
}

impl FromStr for Kind {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Kind::ALL
            .into_iter()
            .find(|k| k.as_str() == s)
            .ok_or_else(|| s.to_string())
    }
}

/// What an input is: OK, or what is wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum State {
    /// Fine.
    Ok,
    /// Nothing yet.
    Never,
    /// Nothing good for max_age s.
    Stale,
    /// NaN or inf in the newest message.
    Nan,
    /// The same values for frozen_after s.
    Frozen,
    /// Two GPS fixes too far apart. Latched.
    Jump,
    /// The wheels against the IMU and the GPS. Latched.
    Disagree,
    /// An actuator off its command too long. Latched.
    Stuck,
    /// Over the commanded speed: a warning only.
    Over,
    /// Over too long. Latched.
    Overspeed,
    /// A latch input dropped out after its start-up. Latched.
    Lost,
    /// A status input's node says min_level or worse.
    Fault,
}

/// How bad a node says things are (`diagnostic_msgs/DiagnosticStatus`'s levels above OK),
/// in order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Severity {
    /// 1.
    Warn,
    /// 2.
    Error,
    /// 3.
    Stale,
}

impl Severity {
    /// The message's level byte: None for OK (0); anything above 3 counts as STALE.
    #[must_use]
    pub fn from_level(level: u8) -> Option<Severity> {
        match level {
            0 => None,
            1 => Some(Severity::Warn),
            2 => Some(Severity::Error),
            _ => Some(Severity::Stale),
        }
    }

    /// A rule's `min_level`: 1, 2 or 3 only.
    #[must_use]
    pub fn from_min_level(level: i64) -> Option<Severity> {
        match level {
            1 => Some(Severity::Warn),
            2 => Some(Severity::Error),
            3 => Some(Severity::Stale),
            _ => None,
        }
    }

    /// The level's name, as the Python's `LEVELS`.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Severity::Warn => "WARN",
            Severity::Error => "ERROR",
            Severity::Stale => "STALE",
        }
    }
}

impl State {
    /// The Python's name.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            State::Ok => "OK",
            State::Never => "NEVER",
            State::Stale => "STALE",
            State::Nan => "NAN",
            State::Frozen => "FROZEN",
            State::Jump => "JUMP",
            State::Disagree => "DISAGREE",
            State::Stuck => "STUCK",
            State::Over => "OVER",
            State::Overspeed => "OVERSPEED",
            State::Lost => "LOST",
            State::Fault => "FAULT",
        }
    }

    /// Warns, but never stops the drivers (`WARN_ONLY`).
    #[must_use]
    pub fn warn_only(self) -> bool {
        self == State::Over
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// How to watch one input. 0 turns a check off.
#[derive(Debug, Clone, PartialEq)]
pub struct Rule {
    /// Its name.
    pub name: String,
    /// The topic (a node input: the node's full name).
    pub topic: String,
    /// Which message.
    pub kind: Kind,
    /// s without a good message: STALE.
    pub max_age: f64,
    /// Stop the automatic drivers while it isn't OK.
    pub stop: bool,
    /// s of identical values: FROZEN.
    pub frozen_after: f64,
    /// m/s between two GPS fixes: JUMP.
    pub max_jump_speed: f64,
    /// m more, for the noise of two fixes.
    pub jump_margin: f64,
    /// Odom only: the wheels against the IMU.
    pub cross_check: Option<CrossCheck>,
    /// Detections only: the camera whose messages count.
    pub frame_id: String,
    /// A dropout after its start-up latches LOST.
    pub latch: bool,
    /// s after its first OK: one dropout forgiven.
    pub latch_grace: f64,
    /// Actuator only: |command − measured| above it ...
    pub follow_tolerance: f64,
    /// ... for this many s: STUCK.
    pub follow_persist: f64,
    /// Speed only: m/s over the command (0 off) ...
    pub over_margin: f64,
    /// ... for this many s while not slowing: OVER.
    pub over_warn: f64,
    /// ... for this many s: OVERSPEED.
    pub over_persist: f64,
    /// m/s²: slowing at least this fast doesn't count.
    pub over_slowing: f64,
    /// Status only: the messages of this name count (empty: all).
    pub status_name: String,
    /// Status only: this or worse is a FAULT.
    pub min_level: Severity,
    /// Status only: silence is quiet, not STALE.
    pub quiet_ok: bool,
}

/// A rule that makes no sense, and why (the Python's words).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuleError(pub String);

impl fmt::Display for RuleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for RuleError {}

impl Rule {
    /// A rule with the Python's defaults: stops, no checks but the age.
    #[must_use]
    pub fn new(name: &str, topic: &str, kind: Kind, max_age: f64) -> Self {
        Rule {
            name: name.into(),
            topic: topic.into(),
            kind,
            max_age,
            stop: true,
            frozen_after: 0.0,
            max_jump_speed: 0.0,
            jump_margin: 5.0,
            cross_check: None,
            frame_id: String::new(),
            latch: false,
            latch_grace: 5.0,
            follow_tolerance: 0.0,
            follow_persist: 1.0,
            over_margin: 0.0,
            over_warn: 1.0,
            over_persist: 2.0,
            over_slowing: 0.3,
            status_name: String::new(),
            min_level: Severity::Error,
            quiet_ok: false,
        }
    }

    /// This rule, checked as the Python's `__post_init__`.
    ///
    /// # Errors
    ///
    /// [`RuleError`], saying which rule and what is wrong.
    pub fn checked(self) -> Result<Self, RuleError> {
        let n = &self.name;
        let fail = |why: &str| Err(RuleError(format!("{n}: {why}")));
        if self.max_age.is_nan() || self.max_age <= 0.0 {
            return fail("max_age must be a positive number of seconds");
        }
        if self.cross_check.is_some() && self.kind != Kind::Odom {
            return fail("only an odom input has a cross check");
        }
        if (self.kind == Kind::Detections) != !self.frame_id.is_empty() {
            return fail("a detections input, and only one, names its camera's frame_id");
        }
        if self.kind == Kind::Detections && self.frozen_after > 0.0 {
            return fail("detections have no frozen check (an empty list repeats on a clear road)");
        }
        if self.kind == Kind::Node && self.stop {
            return fail("a node input only warns (stop: false; Ben, 14 Nov)");
        }
        if self.kind == Kind::Node && !self.topic.starts_with('/') {
            return fail("a node input's topic is the node's full name, such as /planner");
        }
        if self.follow_tolerance > 0.0 && self.kind != Kind::Actuator {
            return fail("only an actuator input has a follow check");
        }
        if !(self.follow_tolerance >= 0.0 && self.follow_persist > 0.0) {
            return fail("follow_tolerance must be 0 or more, follow_persist above 0 s");
        }
        if self.over_margin > 0.0 && self.kind != Kind::Speed {
            return fail("only a speed input has an overspeed check");
        }
        if !(self.over_margin >= 0.0
            && 0.0 < self.over_warn
            && self.over_warn <= self.over_persist
            && self.over_slowing >= 0.0)
        {
            return fail(
                "over_margin and over_slowing must be 0 or more, and 0 < over_warn <= \
                 over_persist",
            );
        }
        if self.latch_grace.is_nan() || self.latch_grace < 0.0 {
            return fail("latch_grace must be 0 or more seconds");
        }
        if self.kind != Kind::Status && (!self.status_name.is_empty() || self.quiet_ok) {
            return fail("only a status input has status_name or quiet_ok");
        }
        if let Some(c) = self.cross_check {
            c.checked().map_err(|e| RuleError(e.0))?;
        }
        Ok(self)
    }

    /// Whether a message from this frame is this input's (detections: its camera).
    #[must_use]
    pub fn counts(&self, frame_id: &str) -> bool {
        self.kind != Kind::Detections || frame_id == self.frame_id
    }

    /// Whether a status message of this name is this input's.
    #[must_use]
    pub fn hears(&self, status_name: &str) -> bool {
        self.kind != Kind::Status || self.status_name.is_empty() || status_name == self.status_name
    }
}

/// What a message looked like, to spot a frozen sensor.
#[derive(Debug, Clone, PartialEq)]
pub enum Digest {
    /// Its numbers (compared as floats: 0.0 == -0.0).
    Values(Vec<f64>),
    /// A picture's CRC-32.
    Crc(u32),
    /// A heartbeat's count.
    Count(u64),
}

/// One message's news, for [`Watch::arrived`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Arrival {
    /// Its numbers, for the NaN check (an actuator or speed input: command, measured).
    pub values: Vec<f64>,
    /// For the frozen check.
    pub digest: Option<Digest>,
    /// A GPS fix: (latitude, longitude), degrees.
    pub fix: Option<(f64, f64)>,
    /// The GPS says it has no fix.
    pub no_fix: bool,
    /// The car's newest wheel speed (m/s), for a camera's or a cloud's frozen check.
    pub speed: Option<f64>,
    /// The fix's own time (s).
    pub stamp: Option<f64>,
    /// A status message: its level byte and its words.
    pub status: Option<(u8, String)>,
}

/// What was wrong with the newest message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bad {
    Nan,
    NoFix,
}

/// The metres between two (latitude, longitude) points, in degrees: short spans.
#[must_use]
pub fn ground_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let lat = radians((a.0 + b.0) / 2.0);
    let dy = radians(b.0 - a.0) * EARTH_RADIUS;
    let dx = radians(b.1 - a.1) * EARTH_RADIUS * lat.cos();
    hypot(dx, dy)
}

fn finite(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite())
}

/// One input's state, from what arrived.
#[derive(Debug, Clone)]
pub struct Watch {
    rule: Rule,
    heard: Option<f64>,
    good: Option<f64>,
    bad: Option<Bad>,
    digest: Option<Digest>,
    same_since: Option<f64>,
    /// (stamp, latitude, longitude).
    fix: Option<(f64, f64, f64)>,
    jumped: Option<String>,
    off_since: Option<f64>,
    stuck: Option<String>,
    over_since: Option<f64>,
    over_words: String,
    speeds: Vec<(f64, f64)>,
    overspeed: Option<String>,
    /// Status: the newest message, if it says min_level or worse.
    said: Option<(Severity, String)>,
    first_good: Option<f64>,
    was_ok: bool,
    forgiven: bool,
    lost: Option<(State, f64)>,
    cross: Option<WheelCheck>,
    count: u64,
    window: Option<f64>,
    gap: f64,
    worst_gap: f64,
}

impl Watch {
    /// Nothing heard yet.
    #[must_use]
    pub fn new(rule: Rule) -> Self {
        let cross = rule.cross_check.map(WheelCheck::new);
        Watch {
            rule,
            heard: None,
            good: None,
            bad: None,
            digest: None,
            same_since: None,
            fix: None,
            jumped: None,
            off_since: None,
            stuck: None,
            over_since: None,
            over_words: String::new(),
            speeds: Vec::new(),
            overspeed: None,
            said: None,
            first_good: None,
            was_ok: false,
            forgiven: false,
            lost: None,
            cross,
            count: 0,
            window: None,
            gap: 0.0,
            worst_gap: 0.0,
        }
    }

    /// Its rule.
    #[must_use]
    pub fn rule(&self) -> &Rule {
        &self.rule
    }

    /// The wheels' cross check (odom only).
    #[must_use]
    pub fn cross(&self) -> Option<&WheelCheck> {
        self.cross.as_ref()
    }

    /// Note a message at `now` (s).
    pub fn arrived(&mut self, now: f64, a: Arrival) {
        let gap_before = self.heard.map(|heard| now - heard);
        match self.heard {
            None => self.window = Some(now),
            Some(heard) => {
                self.gap = py_max(self.gap, now - heard);
                self.worst_gap = py_max(self.worst_gap, self.gap);
            }
        }
        self.count += 1;
        self.heard = Some(now);
        let kind = self.rule.kind;
        if kind == Kind::Gps && (a.no_fix || !a.fix.is_some_and(|(x, y)| finite(&[x, y]))) {
            self.bad = Some(Bad::NoFix);
            return;
        }
        if matches!(kind, Kind::Actuator | Kind::Speed) {
            if kind == Kind::Speed && gap_before.is_some_and(|g| g > self.rule.max_age) {
                // A stale command: start afresh.
                self.over_since = None;
                self.speeds.clear();
            }
            self.bad = None;
            self.good = Some(now);
            if kind == Kind::Actuator {
                self.follow(now, &a.values);
            } else {
                self.over(now, &a.values);
            }
            return;
        }
        if !finite(&a.values) {
            self.bad = Some(Bad::Nan);
            return;
        }
        self.bad = None;
        self.good = Some(now);
        if kind == Kind::Status {
            let min = self.rule.min_level;
            self.said = a.status.and_then(|(level, words)| {
                Severity::from_level(level)
                    .filter(|s| *s >= min)
                    .map(|s| (s, words))
            });
        }
        if let Some(cross) = &mut self.cross {
            if a.values.len() >= 3 {
                cross.wheels(now, a.values[0], a.values[2]); // forward speed, turn rate
            }
        }
        if self.rule.frozen_after > 0.0 {
            if let Some(digest) = a.digest {
                // A camera or a cloud only while the car moves: a still car in a still
                // world sees the same picture, and a noiseless lidar the same cloud.
                let moving =
                    !kind.frozen_only_moving() || a.speed.is_some_and(|v| v.abs() > MOVING);
                if Some(&digest) != self.digest.as_ref() || !moving {
                    self.same_since = Some(now);
                }
                self.digest = Some(digest);
            }
        }
        if kind == Kind::Gps {
            let Some(fix) = a.fix else {
                return;
            };
            let taken = a.stamp.unwrap_or(now);
            if let Some((t, lat, lon)) = self.fix {
                if self.rule.max_jump_speed > 0.0 && self.jumped.is_none() {
                    let dt = taken - t;
                    let metres = ground_distance((lat, lon), fix);
                    if dt > 0.0 && metres > self.rule.max_jump_speed * dt + self.rule.jump_margin {
                        self.jumped = Some(format!("{metres:.1} m in {dt:.2} s"));
                    }
                }
            }
            if self.fix.is_none_or(|f| taken > f.0) {
                self.fix = Some((taken, fix.0, fix.1)); // not an old fix, late
            }
        }
    }

    /// Actuator: (command, measured); latch STUCK once off too long.
    fn follow(&mut self, now: f64, values: &[f64]) {
        let rule = &self.rule;
        if rule.follow_tolerance <= 0.0 || values.len() < 2 || !finite(&values[..2]) {
            self.off_since = None;
            return;
        }
        let (command, measured) = (values[0], values[1]);
        if (command - measured).abs() <= rule.follow_tolerance {
            self.off_since = None;
            return;
        }
        let since = *self.off_since.get_or_insert(now);
        if self.stuck.is_none() && now - since >= rule.follow_persist {
            self.stuck = Some(format!(
                "asked {command:+.2}, did {measured:+.2} for {:.1} s",
                now - since
            ));
        }
    }

    /// Speed: (command, measured\[, age\]) in m/s; latch OVERSPEED once over too long.
    ///
    /// `age`: the measured speed's age (s). Older than `max_age`, it is no check: the
    /// wheels went quiet (odometry's own fault), and their last speed says nothing now.
    fn over(&mut self, now: f64, values: &[f64]) {
        let rule = &self.rule;
        // A NaN age is stale too.
        let stale = values
            .get(2)
            .is_some_and(|&age| age.is_nan() || age > rule.max_age);
        if rule.over_margin <= 0.0 || values.len() < 2 || !finite(&values[..2]) || stale {
            self.over_since = None;
            self.speeds.clear();
            return;
        }
        let (command, measured) = (values[0].abs(), values[1].abs());
        self.speeds.retain(|(t, _)| now - t <= SLOWING_WINDOW);
        let oldest = self.speeds.first().copied();
        self.speeds.push((now, measured));
        // Only a full window says whether it is slowing: a shorter one is noise.
        let slowing = oldest.is_some_and(|(t, v)| {
            now - t >= 0.8 * SLOWING_WINDOW && (v - measured) / (now - t) >= rule.over_slowing
        });
        if measured <= command + rule.over_margin || slowing {
            self.over_since = None;
            return;
        }
        let since = *self.over_since.get_or_insert(now);
        self.over_words = format!(
            "asked {command:.2} m/s, did {measured:.2} for {:.1} s",
            now - since
        );
        if self.overspeed.is_none() && now - since >= rule.over_persist {
            self.overspeed = Some(self.over_words.clone());
        }
    }

    /// (OK or what is wrong, a few words for a person).
    pub fn state(&mut self, now: f64) -> (State, String) {
        if let Some((went, since)) = self.lost {
            let (current, _) = self.current(now);
            return (
                State::Lost,
                format!(
                    "went {went} {:.0} s ago, {current} now: clear it once it runs right",
                    now - since
                ),
            );
        }
        self.current(now)
    }

    /// (state, words) as the input is now, without a latched LOST.
    pub fn current(&mut self, now: f64) -> (State, String) {
        if let Some(j) = &self.jumped {
            return (
                State::Jump,
                format!("jumped {j}: clear it once the position is right"),
            );
        }
        if let Some(s) = &self.stuck {
            return (
                State::Stuck,
                format!("{s}: clear it once the actuator follows again"),
            );
        }
        if let Some(o) = &self.overspeed {
            return (
                State::Overspeed,
                format!("{o}: clear it once the throttle obeys again"),
            );
        }
        if let Some(why) = self.cross.as_mut().and_then(|c| c.update(now)) {
            return (
                State::Disagree,
                format!("{why}: clear it once the wheels are right"),
            );
        }
        if self.heard.is_none() {
            return if self.rule.quiet_ok {
                (State::Ok, "quiet: nothing yet".into())
            } else {
                (State::Never, "nothing yet".into())
            };
        }
        if self.bad == Some(Bad::Nan) {
            return (State::Nan, "NaN or inf in the newest message".into());
        }
        let rule = &self.rule;
        let age = self.good.map_or(f64::INFINITY, |g| now - g);
        if age > rule.max_age && rule.quiet_ok {
            return (State::Ok, format!("quiet for {age:.1} s"));
        }
        if age > rule.max_age {
            let why = if self.bad == Some(Bad::NoFix) {
                "no fix"
            } else {
                "nothing"
            };
            let since = if age.is_finite() {
                format!("for {age:.1} s")
            } else {
                "yet".into()
            };
            return (
                State::Stale,
                format!("{why} {since} (more than {} s)", fmt_g(rule.max_age)),
            );
        }
        if rule.frozen_after > 0.0 {
            if let Some(same) = self.same_since {
                if now - same > rule.frozen_after {
                    return (State::Frozen, format!("the same for {:.1} s", now - same));
                }
            }
        }
        if let Some((level, words)) = &self.said {
            return (State::Fault, format!("says {}: {words}", level.as_str()));
        }
        if let (Some(over), Some(good)) = (self.over_since, self.good) {
            if good - over >= rule.over_warn {
                return (State::Over, self.over_words.clone());
            }
        }
        (State::Ok, format!("{age:.1} s old"))
    }

    /// (messages a second, longest gap, longest gap ever) in s, and a new window starts.
    pub fn traffic(&mut self, now: f64) -> (f64, f64, f64) {
        let (Some(heard), Some(window)) = (self.heard, self.window) else {
            return (0.0, 0.0, 0.0);
        };
        let span = now - window;
        let rate = if span > 0.0 {
            self.count as f64 / span
        } else {
            0.0
        };
        let gap = py_max(self.gap, now - heard);
        self.worst_gap = py_max(self.worst_gap, gap);
        let out = (rate, gap, self.worst_gap);
        self.count = 0;
        self.window = Some(now);
        self.gap = 0.0;
        out
    }

    /// Latch rules: latch a dropout, unless it is the one of its start-up. Once a tick.
    pub fn update(&mut self, now: f64) {
        if !self.rule.latch || self.lost.is_some() {
            return;
        }
        let (state, _) = self.current(now);
        if state == State::Ok {
            self.was_ok = true;
            self.first_good.get_or_insert(now);
        } else if self.was_ok {
            self.was_ok = false;
            let first = self.first_good.unwrap_or(now);
            if !self.forgiven && now - first <= self.rule.latch_grace {
                self.forgiven = true; // starting up: holds only while it lasts
            } else {
                self.lost = Some((state, now));
            }
        }
    }

    /// Forget what latched; a latch rule starts afresh, with its grace again.
    pub fn clear(&mut self) {
        self.jumped = None;
        self.stuck = None;
        self.off_since = None;
        self.overspeed = None;
        self.over_since = None;
        self.speeds.clear();
        if let Some(c) = &mut self.cross {
            c.clear();
        }
        self.lost = None;
        self.first_good = None;
        self.was_ok = false;
        self.forgiven = false;
    }
}

/// One input's line in [`Watchdog::states`].
#[derive(Debug, Clone, PartialEq)]
pub struct Verdict {
    /// The input's name.
    pub name: String,
    /// Its state.
    pub state: State,
    /// Why, for a person.
    pub detail: String,
    /// It stops the automatic drivers.
    pub stops: bool,
}

/// Every input's watch, and the verdict: whether the automatic drivers may drive.
#[derive(Debug, Clone)]
pub struct Watchdog {
    watches: Vec<Watch>,
    stall: f64,
    settle: f64,
    ticked: Option<f64>,
    quiet_until: Option<f64>,
}

impl Watchdog {
    /// `stall`: steady s between two updates that mean the watchdog itself was stopped;
    /// `settle`: steady s after such a stall in which nothing latches. Two rules of one
    /// name: the later one, in the first one's place (as a Python dict).
    #[must_use]
    pub fn new(rules: Vec<Rule>, stall: f64, settle: f64) -> Self {
        let mut watches: Vec<Watch> = Vec::new();
        for rule in rules {
            match watches.iter_mut().find(|w| w.rule.name == rule.name) {
                Some(w) => *w = Watch::new(rule),
                None => watches.push(Watch::new(rule)),
            }
        }
        Watchdog {
            watches,
            stall,
            settle,
            ticked: None,
            quiet_until: None,
        }
    }

    /// Every watch, in the rules' order.
    #[must_use]
    pub fn watches(&self) -> &[Watch] {
        &self.watches
    }

    /// The watch of this name.
    pub fn watch_mut(&mut self, name: &str) -> Option<&mut Watch> {
        self.watches.iter_mut().find(|w| w.rule.name == name)
    }

    /// Note the time, once a tick (`now` the node's clock, `steady` a steady clock's s):
    /// where a rule says latch, a dropout latches here, unless the watchdog itself was
    /// just stalled.
    pub fn update(&mut self, now: f64, steady: f64) {
        if let Some(ticked) = self.ticked {
            if steady - ticked > self.stall {
                self.quiet_until = Some(steady + self.settle);
            }
        }
        self.ticked = Some(steady);
        if self.quiet_until.is_some_and(|q| steady < q) {
            return;
        }
        for w in &mut self.watches {
            w.update(now);
        }
    }

    /// An IMU sample for every cross check: forward m/s², yaw rate rad/s.
    pub fn imu(&mut self, now: f64, forward: f64, yaw_rate: f64) {
        for c in self.watches.iter_mut().filter_map(|w| w.cross.as_mut()) {
            c.imu(now, forward, yaw_rate);
        }
    }

    /// A good GPS fix for every cross check: its stamp (s), (latitude, longitude).
    pub fn gps(&mut self, stamp: f64, fix: (f64, f64)) {
        for c in self.watches.iter_mut().filter_map(|w| w.cross.as_mut()) {
            c.gps(stamp, fix.0, fix.1);
        }
    }

    /// Any input has a cross check.
    #[must_use]
    pub fn cross_checked(&self) -> bool {
        self.watches.iter().any(|w| w.cross.is_some())
    }

    /// Every input's verdict, in the rules' order.
    pub fn states(&mut self, now: f64) -> Vec<Verdict> {
        self.watches
            .iter_mut()
            .map(|w| {
                let (state, detail) = w.state(now);
                Verdict {
                    name: w.rule.name.clone(),
                    state,
                    detail,
                    stops: w.rule.stop && state != State::Ok && !state.warn_only(),
                }
            })
            .collect()
    }

    /// Whether any input whose rule says stop isn't OK.
    pub fn locked(&mut self, now: f64) -> bool {
        self.states(now).iter().any(|v| v.stops)
    }

    /// One line for the HUD and the log: OK, or what is wrong and what it does.
    pub fn summary(&mut self, now: f64) -> String {
        let mut wrong: Vec<Verdict> = self
            .states(now)
            .into_iter()
            .filter(|v| v.state != State::Ok)
            .collect();
        if wrong.is_empty() {
            return "OK".into();
        }
        // What stops the drivers first: the HUD's reason. Stable, as Python's sort.
        wrong.sort_by_key(|v| !v.stops);
        let verdict = if wrong.iter().any(|v| v.stops) {
            "STOP"
        } else {
            "WARN"
        };
        let parts: Vec<String> = wrong
            .iter()
            .map(|v| format!("{} {} ({})", v.name, v.state, v.detail))
            .collect();
        format!("{verdict}: {}", parts.join("; "))
    }

    /// Forget latched jumps, disagreements, stuck actuators, overspeed, lost inputs.
    pub fn clear(&mut self) {
        for w in &mut self.watches {
            w.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    //! `test_watchdog.py`, the main cases, ported.

    use super::*;

    const HERE: (f64, f64) = (45.3860, -75.6960);

    fn odom() -> Rule {
        Rule::new("odom", "/vehicle/odom", Kind::Odom, 0.5)
    }
    fn gps_rule() -> Rule {
        Rule {
            max_jump_speed: 15.0,
            ..Rule::new("gps", "/sensors/gps/fix", Kind::Gps, 2.0)
        }
    }
    fn radar() -> Rule {
        Rule {
            stop: false,
            ..Rule::new("radar", "/sensors/radar/points", Kind::Cloud, 1.0)
        }
    }
    fn lidar() -> Rule {
        Rule {
            frozen_after: 1.0,
            ..Rule::new("lidar", "/sensors/lidar/points", Kind::Cloud, 1.0)
        }
    }
    fn cloud(digest: u32, speed: f64) -> Arrival {
        Arrival {
            digest: Some(Digest::Crc(digest)),
            speed: Some(speed),
            ..Arrival::default()
        }
    }
    fn latched_hud() -> Rule {
        Rule {
            frozen_after: 1.0,
            latch: true,
            ..Rule::new("hud", "/hud/heartbeat", Kind::Heartbeat, 1.0)
        }
    }
    fn steering() -> Rule {
        Rule {
            follow_tolerance: 0.15,
            ..Rule::new("steering", "/vehicle/steering", Kind::Actuator, 1.0)
        }
    }
    fn speed() -> Rule {
        Rule {
            over_margin: 0.2,
            ..Rule::new("speed", "/ego/actuator_cmd", Kind::Speed, 1.0)
        }
    }

    fn dog(rules: Vec<Rule>) -> Watchdog {
        Watchdog::new(rules, 0.5, 3.0)
    }

    fn state(dog: &mut Watchdog, name: &str, now: f64) -> State {
        dog.states(now)
            .into_iter()
            .find(|v| v.name == name)
            .map(|v| v.state)
            .expect("an input of that name")
    }

    fn arrive(dog: &mut Watchdog, name: &str, now: f64, a: Arrival) {
        dog.watch_mut(name).expect("input").arrived(now, a);
    }

    fn values(v: &[f64]) -> Arrival {
        Arrival {
            values: v.to_vec(),
            ..Arrival::default()
        }
    }

    fn fix(lat: f64, lon: f64) -> Arrival {
        Arrival {
            fix: Some((lat, lon)),
            ..Arrival::default()
        }
    }

    /// Beat for the HUD from start to end (s), the watchdog ticking at 10 Hz.
    fn beat(dog: &mut Watchdog, start: f64, end: f64, count: u64) {
        let mut k = 0_u32;
        while start + f64::from(k) / 10.0 <= end + 1e-9 {
            let t = start + f64::from(k) / 10.0;
            let a = Arrival {
                digest: Some(Digest::Count(count + u64::from(k) + 1)),
                ..Arrival::default()
            };
            arrive(dog, "hud", t, a);
            dog.update(t, t);
            k += 1;
        }
    }

    /// Tick at 10 Hz from start to end (s), with nothing arriving.
    fn tick(dog: &mut Watchdog, start: f64, end: f64) {
        let mut k = 0_u32;
        while start + f64::from(k) * 0.1 <= end + 1e-9 {
            let t = start + f64::from(k) * 0.1;
            dog.update(t, t);
            k += 1;
        }
    }

    /// Feed (command, measured) at 50 Hz; each a function of t.
    fn drive(
        dog: &mut Watchdog,
        start: f64,
        seconds: f64,
        command: impl Fn(f64) -> f64,
        measured: impl Fn(f64) -> f64,
    ) -> f64 {
        let mut t = start;
        for i in 0..(seconds / 0.02).round() as u32 {
            t = start + f64::from(i) * 0.02;
            arrive(dog, "speed", t, values(&[command(t), measured(t)]));
        }
        t
    }

    #[test]
    fn a_silent_lidar_goes_stale_and_stops_the_drivers() {
        let mut d = dog(vec![lidar()]);
        for k in 0..10_u32 {
            arrive(&mut d, "lidar", f64::from(k) * 0.1, cloud(k, 1.0));
        }
        assert_eq!(state(&mut d, "lidar", 1.8), State::Ok);
        assert!(!d.locked(1.8));
        assert_eq!(state(&mut d, "lidar", 2.0), State::Stale);
        assert!(d.locked(2.0));
        arrive(&mut d, "lidar", 2.1, cloud(99, 1.0));
        assert_eq!(state(&mut d, "lidar", 2.1), State::Ok);
    }

    #[test]
    fn a_repeated_lidar_cloud_is_frozen_only_while_the_car_moves() {
        let mut d = dog(vec![lidar()]);
        for k in 0..30_u32 {
            arrive(&mut d, "lidar", f64::from(k) * 0.1, cloud(7, 0.0)); // parked
        }
        assert_eq!(state(&mut d, "lidar", 2.9), State::Ok);
        for k in 30..45_u32 {
            arrive(&mut d, "lidar", f64::from(k) * 0.1, cloud(7, 1.0)); // driving
        }
        assert_eq!(state(&mut d, "lidar", 4.4), State::Frozen);
        assert!(d.locked(4.4));
        arrive(&mut d, "lidar", 4.5, cloud(8, 1.0));
        assert_eq!(state(&mut d, "lidar", 4.5), State::Ok);
    }

    #[test]
    fn a_cloud_without_frozen_after_never_freezes() {
        let mut d = dog(vec![radar()]);
        for k in 0..30_u32 {
            arrive(&mut d, "radar", f64::from(k) * 0.1, cloud(7, 2.0));
        }
        assert_eq!(state(&mut d, "radar", 2.9), State::Ok);
    }

    #[test]
    fn nothing_yet_stops_the_drivers() {
        let mut d = dog(vec![odom()]);
        assert_eq!(state(&mut d, "odom", 0.0), State::Never);
        assert!(d.locked(0.0));
    }

    #[test]
    fn fresh_is_ok_and_silence_goes_stale() {
        let mut d = dog(vec![odom()]);
        arrive(&mut d, "odom", 10.0, values(&[1.0, 0.0]));
        assert_eq!(state(&mut d, "odom", 10.4), State::Ok);
        assert!(!d.locked(10.4));
        assert_eq!(state(&mut d, "odom", 10.6), State::Stale);
        assert!(d.locked(10.6));
        arrive(&mut d, "odom", 10.7, values(&[1.0, 0.0]));
        assert!(!d.locked(10.7));
    }

    #[test]
    fn nan_stops_at_once() {
        let mut d = dog(vec![odom()]);
        arrive(&mut d, "odom", 1.0, values(&[f64::NAN, 0.0]));
        assert_eq!(state(&mut d, "odom", 1.0), State::Nan);
        assert!(d.locked(1.0));
    }

    #[test]
    fn a_gps_without_a_fix_counts_as_lost_after_max_age() {
        let mut d = dog(vec![gps_rule()]);
        arrive(&mut d, "gps", 0.0, fix(HERE.0, HERE.1));
        arrive(&mut d, "gps", 1.0, fix(f64::NAN, f64::NAN));
        assert_eq!(state(&mut d, "gps", 1.5), State::Ok);
        let a = Arrival {
            no_fix: true,
            ..fix(HERE.0, HERE.1)
        };
        arrive(&mut d, "gps", 2.1, a);
        let (s, detail) = d.watch_mut("gps").unwrap().state(2.1);
        assert_eq!(s, State::Stale);
        assert!(detail.contains("no fix"), "{detail}");
    }

    #[test]
    fn a_gps_jump_latches_until_cleared() {
        let mut d = dog(vec![gps_rule()]);
        let far = HERE.0 + 20.0 / 111_195.0;
        arrive(&mut d, "gps", 0.0, fix(HERE.0, HERE.1));
        arrive(&mut d, "gps", 0.2, fix(HERE.0 + 0.5e-5, HERE.1));
        assert_eq!(state(&mut d, "gps", 0.2), State::Ok);
        arrive(&mut d, "gps", 0.4, fix(far, HERE.1));
        assert_eq!(state(&mut d, "gps", 0.4), State::Jump);
        arrive(&mut d, "gps", 0.6, fix(far, HERE.1));
        assert_eq!(state(&mut d, "gps", 10.0), State::Jump);
        d.clear();
        arrive(&mut d, "gps", 10.0, fix(far, HERE.1));
        assert_eq!(state(&mut d, "gps", 10.0), State::Ok);
    }

    #[test]
    fn one_dropout_while_the_hud_starts_is_forgiven() {
        let mut d = dog(vec![latched_hud()]);
        beat(&mut d, 0.0, 2.0, 0);
        tick(&mut d, 2.1, 3.5);
        assert_eq!(state(&mut d, "hud", 3.5), State::Stale);
        assert!(d.locked(3.5));
        beat(&mut d, 3.6, 10.0, 100);
        assert_eq!(state(&mut d, "hud", 10.0), State::Ok);
        assert!(!d.locked(10.0));
    }

    #[test]
    fn a_hud_that_drops_out_after_its_start_up_latches_until_cleared() {
        let mut d = dog(vec![latched_hud()]);
        beat(&mut d, 0.0, 20.0, 0);
        tick(&mut d, 20.1, 22.0);
        assert_eq!(state(&mut d, "hud", 22.0), State::Lost);
        beat(&mut d, 27.0, 40.0, 0);
        let (s, detail) = d.watch_mut("hud").unwrap().state(40.0);
        assert_eq!(s, State::Lost);
        assert!(
            detail.contains("STALE") && detail.contains("OK now"),
            "{detail}"
        );
        assert!(d.locked(40.0));
        assert!(d.summary(40.0).starts_with("STOP: hud LOST"));
        d.clear();
        assert_eq!(state(&mut d, "hud", 40.0), State::Ok);
        assert!(!d.locked(40.0));
    }

    #[test]
    fn a_hud_that_keeps_crashing_while_it_starts_latches_the_second_time() {
        let mut d = dog(vec![latched_hud()]);
        beat(&mut d, 0.0, 2.0, 0);
        tick(&mut d, 2.1, 3.5);
        beat(&mut d, 3.6, 5.0, 0);
        tick(&mut d, 5.1, 6.5);
        assert_eq!(state(&mut d, "hud", 6.5), State::Lost);
    }

    #[test]
    fn a_stalled_watchdog_latches_nothing() {
        let mut d = dog(vec![latched_hud()]);
        beat(&mut d, 0.0, 20.0, 0);
        tick(&mut d, 23.0, 24.0);
        assert_eq!(state(&mut d, "hud", 24.0), State::Stale);
        beat(&mut d, 24.1, 30.0, 1000);
        assert_eq!(state(&mut d, "hud", 30.0), State::Ok);
        assert!(!d.locked(30.0));
    }

    #[test]
    fn a_hud_that_dies_in_a_stall_still_latches_after_it() {
        let mut d = dog(vec![latched_hud()]);
        beat(&mut d, 0.0, 20.0, 0);
        tick(&mut d, 23.0, 27.0);
        assert_eq!(state(&mut d, "hud", 27.0), State::Lost);
    }

    #[test]
    fn the_summary_puts_what_stops_first_and_a_warn_input_never_stops() {
        let mut d = dog(vec![radar(), odom()]);
        arrive(&mut d, "odom", 0.0, values(&[0.0]));
        assert!(!d.locked(0.3));
        assert!(d.summary(0.3).starts_with("WARN: radar NEVER"));
        assert!(d.summary(0.6).starts_with("STOP: odom STALE"));
        assert!(d.summary(0.6).contains("radar NEVER"));
    }

    #[test]
    fn steering_stuck_off_its_command_latches_stuck_and_stops_the_drivers() {
        let mut d = dog(vec![steering()]);
        for i in 0..16 {
            arrive(&mut d, "steering", f64::from(i) * 0.1, values(&[0.4, 0.05]));
        }
        let (s, words) = d.watch_mut("steering").unwrap().state(1.5);
        assert_eq!(s, State::Stuck);
        assert!(words.contains("asked +0.40, did +0.05"), "{words}");
        assert!(d.locked(1.5));
        arrive(&mut d, "steering", 1.6, values(&[0.05, 0.05]));
        assert_eq!(state(&mut d, "steering", 1.6), State::Stuck);
        d.clear();
        arrive(&mut d, "steering", 1.7, values(&[0.05, 0.05]));
        assert_eq!(state(&mut d, "steering", 1.7), State::Ok);
    }

    #[test]
    fn a_short_miss_resets_and_nan_is_no_fault() {
        let mut d = dog(vec![steering()]);
        for (t, m) in [(0.0, 0.0), (0.9, 0.0), (1.0, 0.38), (1.5, 0.0), (2.2, 0.0)] {
            arrive(&mut d, "steering", t, values(&[0.4, m]));
        }
        assert_eq!(state(&mut d, "steering", 2.2), State::Ok);
        for i in 0..30 {
            let t = 3.0 + f64::from(i) * 0.1;
            arrive(&mut d, "steering", t, values(&[0.4, f64::NAN]));
        }
        assert_eq!(state(&mut d, "steering", 5.9), State::Ok);
        assert_eq!(state(&mut d, "steering", 8.0), State::Stale);
    }

    #[test]
    fn a_stuck_throttle_warns_then_latches_overspeed_and_stops_the_drivers() {
        let mut d = dog(vec![speed()]);
        let t = drive(&mut d, 0.0, 1.2, |_| 1.0, |_| 1.3);
        let (s, words) = d.watch_mut("speed").unwrap().state(t);
        assert_eq!(s, State::Over);
        assert!(words.contains("asked 1.00 m/s, did 1.30"), "{words}");
        assert!(!d.locked(t) && d.summary(t).starts_with("WARN"));
        let t = drive(&mut d, t, 1.0, |_| 1.0, |_| 1.3);
        assert_eq!(state(&mut d, "speed", t), State::Overspeed);
        assert!(d.locked(t) && d.summary(t).starts_with("STOP"));
        let t = drive(&mut d, t, 1.0, |_| 1.0, |_| 1.0);
        assert_eq!(state(&mut d, "speed", t), State::Overspeed);
        d.clear();
        let t = drive(&mut d, t, 0.1, |_| 1.0, |_| 1.0);
        assert_eq!(state(&mut d, "speed", t), State::Ok);
    }

    #[test]
    fn a_stale_wheel_speed_or_command_never_counts() {
        // sim-check --faults' "odom stop": the wheels' last 0.5 m/s, kept, is no overspeed.
        let mut d = dog(vec![speed()]);
        let max_age = speed().max_age;
        drive(&mut d, 0.0, 2.0, |_| 0.5, |_| 0.5);
        // The wheels go quiet at 2.0 s; ego, stopped by the lock, asks 0 for 10 s more.
        let mut now = 2.0;
        for i in 0..500 {
            now = 2.0 + f64::from(i) * 0.02;
            arrive(&mut d, "speed", now, values(&[0.0, 0.5, now - 2.0]));
            if now - 2.0 > max_age {
                assert_ne!(state(&mut d, "speed", now), State::Over, "{now}");
            }
        }
        assert_eq!(state(&mut d, "speed", now), State::Ok);
        assert!(!d.locked(now));
        // A fresh wheel speed that runs over still counts, as before.
        for i in 0..150 {
            now = 12.0 + f64::from(i) * 0.02;
            arrive(&mut d, "speed", now, values(&[0.0, 0.5, 0.01]));
        }
        assert_eq!(state(&mut d, "speed", now), State::Overspeed);
        // A command after a gap longer than max_age starts afresh: no OVER across it.
        let mut d = dog(vec![speed()]);
        let t = drive(&mut d, 0.0, 0.9, |_| 1.0, |_| 1.3);
        let t = drive(&mut d, t + 1.5, 0.2, |_| 1.0, |_| 1.3);
        assert_eq!(state(&mut d, "speed", t), State::Ok);
    }

    #[test]
    fn a_healthy_stop_lagging_its_command_never_counts() {
        let mut d = dog(vec![speed()]);
        let t = drive(&mut d, 0.0, 2.0, |_| 1.0, |_| 1.0);
        let t = drive(
            &mut d,
            t,
            3.0,
            |s| (1.0 - (s - 2.0)).max(0.0),
            |s| {
                if s < 2.7 {
                    1.0
                } else {
                    (1.0 - (s - 2.7)).max(0.0)
                }
            },
        );
        assert_eq!(state(&mut d, "speed", t), State::Ok);
        let t = drive(&mut d, t, 2.0, |_| 2.5, |_| 2.5);
        let t0 = t;
        let t = drive(&mut d, t, 2.5, |_| 0.0, |s| (2.5 - 1.5 * (s - t0)).max(0.0));
        assert_eq!(state(&mut d, "speed", t), State::Ok);
    }

    #[test]
    fn rules_are_checked() {
        assert!("lidar".parse::<Kind>().is_err());
        assert!(Rule::new("x", "/x", Kind::Odom, 0.0).checked().is_err());
        let bad = Rule {
            follow_tolerance: 0.1,
            ..odom()
        };
        assert!(bad.checked().unwrap_err().0.contains("actuator"));
        let node = Rule::new("n", "/planner", Kind::Node, 3.0);
        assert!(node.checked().unwrap_err().0.contains("only warns"));
        let det = Rule::new("d", "/perception/detections", Kind::Detections, 1.0);
        assert!(det.checked().is_err());
        assert!(speed().checked().is_ok() && latched_hud().checked().is_ok());
    }

    #[test]
    fn ground_distance_is_metres() {
        let north = (HERE.0 + 100.0 / 111_195.0, HERE.1);
        assert!((ground_distance(HERE, north) - 100.0).abs() < 0.1);
    }

    fn said(level: u8, words: &str) -> Arrival {
        Arrival {
            status: Some((level, words.into())),
            ..Arrival::default()
        }
    }

    fn costmap() -> Rule {
        Rule {
            stop: false,
            status_name: "costmap local".into(),
            min_level: Severity::Warn,
            ..Rule::new("costmap", "/drivers/nav/costmap/status", Kind::Status, 3.0)
        }
    }

    #[test]
    fn a_node_that_says_it_waits_warns_in_its_own_words() {
        let mut dog = Watchdog::new(vec![costmap().checked().expect("rule")], 0.5, 3.0);
        assert_eq!(state(&mut dog, "costmap", 0.0), State::Never);
        let w = dog.watch_mut("costmap").expect("watch");
        w.arrived(1.0, said(0, "fine"));
        w.arrived(2.0, said(1, "local costmap: waiting for odometry"));
        assert_eq!(
            w.state(2.0),
            (
                State::Fault,
                "says WARN: local costmap: waiting for odometry".into()
            )
        );
        assert!(!dog.locked(2.0), "warn only");
        assert_eq!(state(&mut dog, "costmap", 5.1), State::Stale);
        assert!(costmap().hears("costmap local") && !costmap().hears("costmap global"));
    }

    #[test]
    fn a_quiet_node_is_ok_and_can_be_switched_to_stop() {
        let rule = Rule {
            quiet_ok: true,
            ..Rule::new("plans", "/drivers/nav/planner/status", Kind::Status, 3.0)
        };
        let mut dog = Watchdog::new(vec![rule.checked().expect("rule")], 0.5, 3.0);
        assert_eq!(state(&mut dog, "plans", 0.0), State::Ok);
        let w = dog.watch_mut("plans").expect("watch");
        w.arrived(1.0, said(2, "no path"));
        assert!(dog.locked(1.0), "stop: true holds the drivers");
        let w = dog.watch_mut("plans").expect("watch");
        assert_eq!(w.state(4.5), (State::Ok, "quiet for 3.5 s".into()));
        w.arrived(5.0, said(1, "slow"));
        assert_eq!(state(&mut dog, "plans", 5.0), State::Ok, "under ERROR");
    }

    #[test]
    fn status_settings_belong_to_status_inputs() {
        let odom = Rule {
            quiet_ok: true,
            ..Rule::new("odom", "/vehicle/odom", Kind::Odom, 0.5)
        };
        assert!(odom.checked().is_err());
        assert_eq!(Severity::from_min_level(0), None);
        assert_eq!(Severity::from_level(7), Some(Severity::Stale));
    }
}

//! What the bridge writes on the serial port each tick (`uart_relay.py`).
//!
//! ```text
//!   ego's command (/control/car/actuator_cmd) ─┐
//!   the Orin's counter (/control/car/heartbeat) ┼─> Relay::frame(now) ─> one frame, 50 Hz
//!   /safety/estop ──────────────────────────────┘
//! ```
//!
//! A fresh command and no e-stop: ego's values, armed. Otherwise the stale-stop values
//! (throttle 0, brake full, steering held), not armed. Versions 2 and 3 carry the Orin's
//! counter as it last came: the relay never makes one up, so when the Orin goes quiet the
//! counter stays where it was and the Rust ESP32 brakes 100 ms later. Version 3 also
//! carries the hardware panel's steering calibration (limp, jog), only as
//! [`Relay::cal_refusal`] allows; while it does, every frame is a stop.

use core::fmt;
use core::str::FromStr;

use adaptive_brake::{AdaptiveBrake, BrakeOut};
use legacy_link::{CalRequest, Flags, Frame, FrameV2, FrameV3, COUNTS_MAX};

use crate::mapping::{from_pedals, stop_command, to_legacy, Mapping, Values};
use crate::speed::{Pedals, SpeedControl};

/// s: the panel asks again at 10 Hz while its person holds.
pub const CAL_REQUEST_TIMEOUT: f64 = 0.5;
/// m/s: a faster wheel speed is a moving car.
pub const STANDSTILL: f64 = 0.05;
/// s: the speed loop's first step, before there is a last tick.
const FIRST_STEP: f64 = 0.02;
/// s: the shortest step the speed loop is given.
const MIN_STEP: f64 = 1e-3;

/// Which serial frame goes out (`frame_version`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameVersion {
    /// The 2025-26 13-byte frame, for the C++ firmware: no counter, no timeout.
    V1,
    /// 17 bytes: the Orin's counter, armed and e-stop flags, a CRC-8. The Rust firmware.
    V2,
    /// 20 bytes: version 2 and the steering calibration's mode and jog target.
    V3,
}

impl FrameVersion {
    /// True if the firmware that reads this version brakes by itself when the frames or
    /// the Orin's counter stop (2 and 3, the Rust firmware, 100 ms).
    #[must_use]
    pub fn has_timeout(self) -> bool {
        !matches!(self, FrameVersion::V1)
    }

    /// The version's number: 1, 2 or 3.
    #[must_use]
    pub fn number(self) -> u8 {
        match self {
            FrameVersion::V1 => 1,
            FrameVersion::V2 => 2,
            FrameVersion::V3 => 3,
        }
    }
}

impl TryFrom<i64> for FrameVersion {
    type Error = SettingError;

    fn try_from(n: i64) -> Result<Self, SettingError> {
        match n {
            1 => Ok(FrameVersion::V1),
            2 => Ok(FrameVersion::V2),
            3 => Ok(FrameVersion::V3),
            other => Err(SettingError::FrameVersion(other)),
        }
    }
}

impl fmt::Display for FrameVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.number())
    }
}

/// Where the steering calibration may run (`steering_calibration`): the hardware lead's
/// to set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CalWhere {
    /// Never (the default). `none`, not `off`: YAML reads off as false.
    #[default]
    Nowhere,
    /// The car on stands, wheels off the ground.
    Stands,
    /// The ESP32 on a desk.
    Bench,
}

impl FromStr for CalWhere {
    type Err = SettingError;

    fn from_str(s: &str) -> Result<Self, SettingError> {
        match s {
            "none" => Ok(CalWhere::Nowhere),
            "stands" => Ok(CalWhere::Stands),
            "bench" => Ok(CalWhere::Bench),
            other => Err(SettingError::CalWhere(other.into())),
        }
    }
}

/// A setting the relay can't take.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettingError {
    /// `frame_version` isn't 1, 2 or 3.
    FrameVersion(i64),
    /// `steering_calibration` isn't none, stands or bench.
    CalWhere(String),
    /// A calibration mode that isn't 0 off, 1 limp or 2 jog.
    CalMode(u8),
}

impl fmt::Display for SettingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SettingError::FrameVersion(n) => {
                write!(f, "frame_version {n}: use 1 (the C++ firmware), 2 or 3")
            }
            SettingError::CalWhere(s) => {
                write!(f, "steering_calibration '{s}': use none, stands or bench")
            }
            SettingError::CalMode(m) => {
                write!(f, "calibration mode {m}: use 0 off, 1 limp or 2 jog")
            }
        }
    }
}

impl std::error::Error for SettingError {}

/// What the panel asks of the steering actuator (`SteeringCalibration.mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalMode {
    /// Drive as usual.
    Off,
    /// The motor off: turn the wheels by hand.
    Limp,
    /// Go to a target in counts, slowly.
    Jog,
}

impl TryFrom<u8> for CalMode {
    type Error = SettingError;

    fn try_from(n: u8) -> Result<Self, SettingError> {
        match n {
            0 => Ok(CalMode::Off),
            1 => Ok(CalMode::Limp),
            2 => Ok(CalMode::Jog),
            other => Err(SettingError::CalMode(other)),
        }
    }
}

/// Why the panel's calibration request isn't passed on now (`cal_refusal`'s words).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalRefusal {
    /// `steering_calibration` is none.
    NotHere,
    /// The frame version isn't 3.
    Version(FrameVersion),
    /// No fresh uplink frame: nothing to read the position with.
    NoUplink,
    /// The e-stop is on.
    Estop,
    /// Ego asks for this speed (m/s): someone is driving.
    Driving(f64),
    /// The car moves at this speed (m/s).
    Moving(f64),
}

impl fmt::Display for CalRefusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CalRefusal::NotHere => f.write_str(
                "steering_calibration is none: stands or bench only, set by the hardware lead",
            ),
            CalRefusal::Version(v) => write!(
                f,
                "frame_version {v}: the calibration needs 3 (the Rust firmware)"
            ),
            CalRefusal::NoUplink => {
                f.write_str("no uplink from the ESP32: nothing to read the position with")
            }
            CalRefusal::Estop => f.write_str("the e-stop is on"),
            CalRefusal::Driving(v) => write!(f, "ego asks for {v:.2} m/s: someone is driving"),
            CalRefusal::Moving(v) => write!(f, "the car moves ({v:.2} m/s)"),
        }
    }
}

/// True if twist_mux must hold the programs (scripted, replay, point, navigation, AI): a
/// frame version whose firmware has no command timeout (1, the C++), unless
/// `allow_programs_without_timeout` (a supervised stand only).
#[must_use]
pub fn programs_locked(version: FrameVersion, allow_programs_without_timeout: bool) -> bool {
    !version.has_timeout() && !allow_programs_without_timeout
}

/// The relay's settings (the bridge's parameters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelaySettings {
    /// Ego's numbers to the frame's.
    pub mapping: Mapping,
    /// s: an older command is stale.
    pub cmd_timeout: f64,
    /// Which frame goes out.
    pub version: FrameVersion,
    /// s: an older wheel speed means feed-forward alone.
    pub odom_timeout: f64,
    /// Where the steering calibration may run.
    pub calibration: CalWhere,
    /// s: an older uplink frame is stale.
    pub uplink_timeout: f64,
}

impl Default for RelaySettings {
    fn default() -> Self {
        RelaySettings {
            mapping: Mapping::default(),
            cmd_timeout: 0.3,
            version: FrameVersion::V2,
            odom_timeout: 0.3,
            calibration: CalWhere::Nowhere,
            uplink_timeout: 0.2,
        }
    }
}

/// Ego's last command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// m/s.
    pub speed: f64,
    /// rad, + left.
    pub steering: f64,
    /// s, when it came.
    pub at: f64,
}

/// The panel's last request.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CalAsk {
    mode: CalMode,
    target: i64,
    at: f64,
}

/// The bridge's state: the last command, the Orin's counter, the e-stop (`Relay`).
#[derive(Debug, Clone)]
pub struct Relay {
    settings: RelaySettings,
    cmd: Option<Command>,
    counter: u8,
    estop: bool,
    last_steering: f64,
    speed_loop: Option<SpeedControl>,
    measured: Option<(f64, f64)>,
    last_tick: Option<f64>,
    pedals: Option<Pedals>,
    cal_request: Option<CalAsk>,
    uplink_at: Option<f64>,
    adaptive: Option<AdaptiveBrake>,
    brake_out: Option<BrakeOut>,
}

impl Relay {
    /// A relay with no command yet; `speed_loop` for `speed_loop.mode: pi`.
    #[must_use]
    pub fn new(settings: RelaySettings, speed_loop: Option<SpeedControl>) -> Self {
        Relay {
            settings,
            cmd: None,
            counter: 0,
            estop: false,
            last_steering: 0.0,
            speed_loop,
            measured: None,
            last_tick: None,
            pedals: None,
            cal_request: None,
            uplink_at: None,
            adaptive: None,
            brake_out: None,
        }
    }

    /// The same relay with the adaptive brake (TODO 258) between the pi speed loop and the
    /// brake: harder when the car slows less than the model says, never above the brake's
    /// safe max. The stops (stale, e-stop, calibration) never pass through it: they send
    /// the full stop brake as before. Without a speed loop (`open`) it never runs.
    #[must_use]
    pub fn with_adaptive_brake(mut self, brake: AdaptiveBrake) -> Self {
        self.adaptive = Some(brake);
        self
    }

    /// The adaptive brake's last answer, if it runs and has run since the last stop.
    #[must_use]
    pub fn brake_out(&self) -> Option<BrakeOut> {
        self.brake_out
    }

    /// The settings it runs with.
    #[must_use]
    pub fn settings(&self) -> &RelaySettings {
        &self.settings
    }

    /// Ego's last command, if any.
    #[must_use]
    pub fn command_seen(&self) -> Option<Command> {
        self.cmd
    }

    /// The speed loop's last output, to log.
    #[must_use]
    pub fn pedals(&self) -> Option<Pedals> {
        self.pedals
    }

    /// Notes the measured forward speed (m/s, /vehicle/odom) at `now` (s).
    pub fn wheel_speed(&mut self, speed: f64, now: f64) {
        self.measured = Some((speed, now));
    }

    /// Notes ego's command, passed on by the output mux, at `now` (s).
    pub fn command(&mut self, speed: f64, steering: f64, now: f64) {
        self.cmd = Some(Command {
            speed,
            steering,
            at: now,
        });
    }

    /// Notes the Orin's counter: it goes into every frame until the next one.
    pub fn heartbeat(&mut self, counter: u8) {
        self.counter = counter;
    }

    /// Notes /safety/estop; true if it changed.
    pub fn set_estop(&mut self, on: bool) -> bool {
        let changed = on != self.estop;
        self.estop = on;
        if on {
            // An e-stop: the adaptive brake forgets what it learnt, as adaptive_brake.py says.
            if let Some(b) = self.adaptive.as_mut() {
                b.reset();
            }
            self.brake_out = None;
        }
        changed
    }

    /// Notes the panel's request: off, limp, or jog to `target` counts.
    pub fn calibrate(&mut self, mode: CalMode, target: i64, now: f64) {
        self.cal_request = Some(CalAsk {
            mode,
            target,
            at: now,
        });
    }

    /// Notes that an uplink frame came at `now`: the counts are coming back.
    pub fn uplink_seen(&mut self, now: f64) {
        self.uplink_at = Some(now);
    }

    /// True if the panel asks for limp or jog, and asked within 0.5 s.
    #[must_use]
    pub fn cal_wanted(&self, now: f64) -> bool {
        self.cal_request
            .is_some_and(|r| r.mode != CalMode::Off && now - r.at <= CAL_REQUEST_TIMEOUT)
    }

    /// Why the panel's calibration request isn't passed on now; None: it is.
    #[must_use]
    pub fn cal_refusal(&self, now: f64) -> Option<CalRefusal> {
        let s = &self.settings;
        if s.calibration == CalWhere::Nowhere {
            return Some(CalRefusal::NotHere);
        }
        if s.version != FrameVersion::V3 {
            return Some(CalRefusal::Version(s.version));
        }
        if self.uplink_at.is_none_or(|at| now - at > s.uplink_timeout) {
            return Some(CalRefusal::NoUplink);
        }
        if self.estop {
            return Some(CalRefusal::Estop);
        }
        if let Some(cmd) = self.fresh_command(now) {
            if cmd.speed.abs() >= s.mapping.stop_speed {
                return Some(CalRefusal::Driving(cmd.speed));
            }
        }
        if let Some((speed, at)) = self.measured {
            if now - at <= s.odom_timeout && speed.abs() >= STANDSTILL {
                return Some(CalRefusal::Moving(speed));
            }
        }
        None
    }

    /// True if this tick's frame carries the panel's limp or jog.
    #[must_use]
    pub fn cal_active(&self, now: f64) -> bool {
        self.cal_wanted(now) && self.cal_refusal(now).is_none()
    }

    /// True if ego's last command is no older than the timeout.
    #[must_use]
    pub fn fresh(&self, now: f64) -> bool {
        self.fresh_command(now).is_some()
    }

    /// Ego's last command, if it is fresh.
    #[must_use]
    pub fn fresh_command(&self, now: f64) -> Option<Command> {
        self.cmd.filter(|c| now - c.at <= self.settings.cmd_timeout)
    }

    /// This tick's frame.
    pub fn frame(&mut self, now: f64) -> Vec<u8> {
        if let Some(ask) = self.cal_request.filter(|_| self.cal_active(now)) {
            self.forget_loop();
            let values = stop_command(&self.settings.mapping, self.last_steering);
            let calibration = match ask.mode {
                CalMode::Off => CalRequest::Off,
                CalMode::Limp => CalRequest::Limp,
                CalMode::Jog => CalRequest::Jog(jog_counts(ask.target)),
            };
            return pack_v3(values, self.counter, flags(false, self.estop), calibration);
        }
        let command = self.fresh_command(now).filter(|_| !self.estop);
        let values = match command {
            Some(cmd) => {
                let values = if self.speed_loop.is_some() {
                    self.drive(cmd, now)
                } else {
                    to_legacy(cmd.speed, cmd.steering, &self.settings.mapping)
                };
                self.last_steering = values.steering;
                values
            }
            None => {
                self.forget_loop();
                stop_command(&self.settings.mapping, self.last_steering)
            }
        };
        self.pack(values, command.is_some())
    }

    /// A stale-stop frame, for leaving the car braked.
    #[must_use]
    pub fn stop_frame(&self) -> Vec<u8> {
        self.pack(
            stop_command(&self.settings.mapping, self.last_steering),
            false,
        )
    }

    fn forget_loop(&mut self) {
        if let Some(l) = self.speed_loop.as_mut() {
            l.reset();
            self.last_tick = None;
            self.pedals = None;
        }
        // A stop it didn't make: forget this stop, keep the effectiveness (a load stays).
        if let Some(b) = self.adaptive.as_mut() {
            b.reset_stop();
        }
        self.brake_out = None;
    }

    /// Runs the speed loop one tick; the frame's values.
    fn drive(&mut self, cmd: Command, now: f64) -> Values {
        let dt = self.last_tick.map_or(FIRST_STEP, |last| now - last);
        self.last_tick = Some(now);
        let odom_timeout = self.settings.odom_timeout;
        let measured = self
            .measured
            .filter(|(_, at)| now - at <= odom_timeout)
            .map(|(speed, _)| speed);
        let Some(speed_loop) = self.speed_loop.as_mut() else {
            return to_legacy(cmd.speed, cmd.steering, &self.settings.mapping);
        };
        // max(dt, 1e-3) is always positive; a NaN clock is the only way to fail: stop.
        match speed_loop.update(cmd.speed, measured, crate::py_max(dt, MIN_STEP)) {
            Ok(p) => {
                self.pedals = Some(p);
                // The adaptive brake: never weaker than asked, never above the safe max. If
                // it can't run this tick (a dead time too long for dt), the asked brake.
                let step = crate::py_max(dt, MIN_STEP);
                let brake = match self.adaptive.as_mut() {
                    Some(b) => match b.update(p.throttle, p.brake, measured, step) {
                        Ok(out) => {
                            self.brake_out = Some(out);
                            out.brake
                        }
                        Err(_) => p.brake,
                    },
                    None => p.brake,
                };
                from_pedals(cmd.steering, p.throttle, brake, &self.settings.mapping)
            }
            Err(_) => stop_command(&self.settings.mapping, self.last_steering),
        }
    }

    fn pack(&self, values: Values, armed: bool) -> Vec<u8> {
        let flags = flags(armed, self.estop);
        match self.settings.version {
            FrameVersion::V1 => frame_of(values).encode().to_vec(),
            FrameVersion::V2 => FrameV2 {
                values: frame_of(values),
                counter: self.counter,
                flags,
            }
            .encode()
            .to_vec(),
            FrameVersion::V3 => pack_v3(values, self.counter, flags, CalRequest::Off),
        }
    }
}

/// A jog target within 0..4095 counts, as `pack_frame_v3` clamps it.
fn jog_counts(target: i64) -> u16 {
    u16::try_from(target.clamp(0, i64::from(COUNTS_MAX))).unwrap_or(COUNTS_MAX)
}

/// The values clamped to -1..1, as f32: the frame's three numbers.
fn frame_of(v: Values) -> Frame {
    use crate::mapping::clamp1;
    Frame {
        steering: clamp1(v.steering) as f32,
        speed: clamp1(v.speed) as f32,
        accel: clamp1(v.accel) as f32,
    }
}

fn pack_v3(values: Values, counter: u8, flags: Flags, calibration: CalRequest) -> Vec<u8> {
    FrameV3 {
        values: frame_of(values),
        counter,
        flags,
        calibration,
    }
    .encode()
    .to_vec()
}

fn flags(armed: bool, estop: bool) -> Flags {
    Flags { armed, estop }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn relay(version: FrameVersion) -> Relay {
        Relay::new(
            RelaySettings {
                version,
                ..RelaySettings::default()
            },
            None,
        )
    }

    /// A pi relay (version 1 frames), with or without the adaptive brake.
    fn pi_relay(adaptive: bool) -> Relay {
        let r = Relay::new(
            RelaySettings {
                version: FrameVersion::V1,
                ..RelaySettings::default()
            },
            Some(SpeedControl::new(crate::SpeedConfig::default())),
        );
        if adaptive {
            r.with_adaptive_brake(AdaptiveBrake::new(
                adaptive_brake::CarModel::default(),
                adaptive_brake::BrakeLimits::default(),
                adaptive_brake::Tuning::default(),
            ))
        } else {
            r
        }
    }

    fn accel(frame: &[u8]) -> f32 {
        let bytes: [u8; legacy_link::FRAME_LEN] = frame.try_into().expect("a version 1 frame");
        Frame::decode(&bytes).expect("a good frame").accel
    }

    #[test]
    fn the_adaptive_brake_brakes_harder_on_a_weak_car_never_past_the_safe_max() {
        // ego slows from 1.5 m/s to a stop; the car slows at half what the model says.
        let (mut plain, mut adaptive) = (pi_relay(false), pi_relay(true));
        let (mut v, mut harder) = (1.5, 0);
        let safe = adaptive_brake::BrakeLimits::default().safe_max_fraction();
        for k in 0..250 {
            let now = f64::from(k) * 0.02;
            let target = (1.5 - now * 0.5).max(0.0);
            for r in [&mut plain, &mut adaptive] {
                r.command(target, 0.0, now);
                r.wheel_speed(v, now);
            }
            let (a, b) = (accel(&plain.frame(now)), accel(&adaptive.frame(now)));
            assert!(
                b <= a + 1e-6,
                "never weaker than the plain loop: tick {k}, {b} > {a}"
            );
            harder += usize::from(b < a - 1e-4);
            if let Some(out) = adaptive.brake_out() {
                assert!(out.brake <= safe.max(out.asked) + 1e-12, "{out:?}");
            }
            v = (v - 0.02 * 0.5 * 0.5).max(0.0);
        }
        assert!(harder > 10, "it added brake on only {harder} ticks");
        assert!(adaptive.brake_out().is_some_and(|o| o.effectiveness < 0.9));
    }

    #[test]
    fn the_stops_never_pass_through_the_adaptive_brake() {
        let (mut plain, mut adaptive) = (pi_relay(false), pi_relay(true));
        for r in [&mut plain, &mut adaptive] {
            r.command(1.0, 0.0, 0.0);
            r.wheel_speed(1.0, 0.0);
            r.frame(0.0);
            r.set_estop(true);
        }
        // The e-stop: the full stop brake, the same bytes.
        assert_eq!(plain.frame(0.02), adaptive.frame(0.02));
        assert_eq!(adaptive.brake_out(), None);
        // A stale command: the same.
        for r in [&mut plain, &mut adaptive] {
            r.set_estop(false);
        }
        assert_eq!(plain.frame(1.0), adaptive.frame(1.0));
    }

    #[test]
    fn the_counter_is_relayed_never_made_up() {
        let mut r = relay(FrameVersion::V2);
        r.command(0.6, 0.0, 0.0);
        let a = r.frame(0.0);
        let b = r.frame(0.02);
        assert_eq!(a[14], 0);
        assert_eq!(b[14], 0);
        r.heartbeat(7);
        assert_eq!(r.frame(0.04)[14], 7);
    }

    #[test]
    fn a_stale_command_brakes() {
        let mut r = relay(FrameVersion::V1);
        r.command(0.6, 0.0, 0.0);
        let f = Frame::decode(&r.frame(0.31).try_into().unwrap()).unwrap();
        assert_eq!((f.speed, f.accel), (0.0, -1.0));
    }

    #[test]
    fn version_one_locks_the_programs() {
        assert!(programs_locked(FrameVersion::V1, false));
        assert!(!programs_locked(FrameVersion::V1, true));
        assert!(!programs_locked(FrameVersion::V2, false));
        assert_eq!(
            FrameVersion::try_from(4),
            Err(SettingError::FrameVersion(4))
        );
    }
}

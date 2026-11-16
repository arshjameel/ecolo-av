//! A velocity smoother that knows it drives a car. No ROS.
//!
//! Nav2's `velocity_smoother` limits the speed, the sideways speed and the turn rate, each
//! with its own acceleration, as if the robot could turn on the spot. This one keeps
//! Nav2's behaviour for the speed (limits, acceleration, deceleration, the timeout, the
//! deadband, open or closed loop) and steers like the car:
//!
//! | Nav2's | Ours |
//! |---|---|
//! | `w` limited to `max_velocity[2]`, changed by `max_accel[2]` | `w = v tan(steer) / L`: the steering angle moves at most `steering_rate` rad/s, and never past full lock |
//! | `scale_velocities`: `v` and `w` scaled together, keeping their ratio | the curvature `w / v` is kept: the speed may change, the path's bend changes only as fast as the wheels can turn |
//! | — | a turn-rate cap (`max_velocity[2]`) slows the car in a tight turn, rather than bending it less |
//! | — | the speed's limits also stay inside the car's (`max_accel`, `max_brake`) |
//!
//! One tick, every `1 / frequency` s:
//!
//! 1. No fresh command for `timeout` s: aim for a stop (the last command forgotten), and
//!    once stopped, say nothing (as Nav2's).
//! 2. The speed asked is held within `[min_speed, max_speed]`; the steering angle it asks
//!    is `atan(L w / v)`, within full lock (held when `v` is about 0: a car can't turn on
//!    the spot).
//! 3. The speed is lowered so `|v| tan(steer) / L` stays under `max_turn_rate`, for the
//!    sharper of the wheels now and where they are heading.
//! 4. The speed moves toward it at most `accel` (faster) or `decel` (slower) m/s².
//! 5. The wheels move toward their angle at most `steering_rate` rad/s, once the car
//!    moves at `min_speed_for_steering` (ego holds them below it).
//! 6. Out: `(v, v tan(steer) / L)`, each under its deadband sent as 0.
//!
//! Units: m, s, rad. Positive steering and turn rate = left (REP-103).
//!
//! ```
//! use car_smoother::{Car, Settings, Smoother};
//!
//! let mut s = Smoother::new(Settings::default(), Car::default()).unwrap();
//! s.command(1.0, 0.3, 0.0).unwrap();
//! let out = s.tick(0.05, None).unwrap();
//! assert!(out.v > 0.0 && out.v <= 0.05 + 1e-9);
//! ```
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use std::fmt;

/// The car, from `vehicle.yaml` (through the car model) and ego.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Car {
    /// m, rear axle to front axle.
    pub wheelbase: f64,
    /// rad: full lock, the virtual centre wheel.
    pub max_steering: f64,
    /// rad/s: the fastest the wheels turn (the steering actuator, or ego's tighter ramp).
    pub steering_rate: f64,
    /// m/s: below this ego holds the wheels where they are.
    pub min_speed_for_steering: f64,
    /// m/s² at full throttle.
    pub max_accel: f64,
    /// m/s² at full brake.
    pub max_brake: f64,
}

impl Default for Car {
    /// `vehicle.yaml`'s guesses and ego.yaml's steering ramp, 8 Nov 2026.
    fn default() -> Self {
        Car {
            wheelbase: 1.561,
            max_steering: 0.5,
            steering_rate: 0.6,
            min_speed_for_steering: 0.05,
            max_accel: 1.5,
            max_brake: 3.0,
        }
    }
}

impl Car {
    /// The path's bend (1/m) for a steering angle.
    #[must_use]
    pub fn curvature(&self, steer: f64) -> f64 {
        steer.tan() / self.wheelbase
    }

    /// The steering angle for a bend, within full lock.
    #[must_use]
    pub fn steer_for(&self, curvature: f64) -> f64 {
        (self.wheelbase * curvature)
            .atan()
            .clamp(-self.max_steering, self.max_steering)
    }

    /// The fastest the car can turn at speed `v`: `|v| tan(max_steering) / L`.
    #[must_use]
    pub fn max_turn_rate(&self, v: f64) -> f64 {
        v.abs() * self.curvature(self.max_steering)
    }
}

/// Where the speed now comes from, as Nav2's `feedback`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Feedback {
    /// The last speed sent (`OPEN_LOOP`).
    OpenLoop,
    /// The measured speed, from odometry (`CLOSED_LOOP`).
    ClosedLoop,
}

impl std::str::FromStr for Feedback {
    type Err = SettingsError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "OPEN_LOOP" => Ok(Feedback::OpenLoop),
            "CLOSED_LOOP" => Ok(Feedback::ClosedLoop),
            other => Err(SettingsError::Feedback(other.to_string())),
        }
    }
}

/// Nav2's velocity smoother's settings, the parts a car uses. Speeds in m/s, rates in
/// rad/s, accelerations as magnitudes in m/s² (Nav2's `max_decel` is negative; see
/// [`Settings::from_nav2`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// Hz: `smoothing_frequency`.
    pub frequency: f64,
    /// Where the speed now comes from: `feedback`.
    pub feedback: Feedback,
    /// m/s forward: `max_velocity[0]`.
    pub max_speed: f64,
    /// m/s, 0 or below: `min_velocity[0]` (backwards).
    pub min_speed: f64,
    /// rad/s: `max_velocity[2]`, the most the car may turn (a tight turn is slowed).
    pub max_turn_rate: f64,
    /// m/s², speeding up: `max_accel[0]`.
    pub accel: f64,
    /// m/s², slowing down (a magnitude): `-max_decel[0]`.
    pub decel: f64,
    /// m/s: a speed under this goes out as 0: `deadband_velocity[0]`.
    pub deadband_speed: f64,
    /// rad/s: a turn rate under this goes out as 0: `deadband_velocity[2]`.
    pub deadband_turn: f64,
    /// s without a command, then stop: `velocity_timeout`.
    pub timeout: f64,
}

impl Default for Settings {
    /// Our `nav2.yaml`'s, forward only.
    fn default() -> Self {
        Settings {
            frequency: 20.0,
            feedback: Feedback::OpenLoop,
            max_speed: 1.0,
            min_speed: 0.0,
            max_turn_rate: 1.0,
            accel: 1.0,
            decel: 2.0,
            deadband_speed: 0.0,
            deadband_turn: 0.0,
            timeout: 1.0,
        }
    }
}

/// Nav2's lists of three (x, y, theta), as its parameters hold them.
#[derive(Debug, Clone, PartialEq)]
pub struct Nav2Limits {
    /// `max_velocity`.
    pub max_velocity: Vec<f64>,
    /// `min_velocity`.
    pub min_velocity: Vec<f64>,
    /// `max_accel`.
    pub max_accel: Vec<f64>,
    /// `max_decel`.
    pub max_decel: Vec<f64>,
    /// `deadband_velocity`.
    pub deadband_velocity: Vec<f64>,
}

impl Settings {
    /// Settings from Nav2's parameters, refused where Nav2 refuses them (the signs, the
    /// lists' length, min above max). y is ignored: a car doesn't slide sideways.
    ///
    /// # Errors
    ///
    /// [`SettingsError`] for the first setting Nav2 would refuse.
    pub fn from_nav2(
        frequency: f64,
        feedback: Feedback,
        limits: &Nav2Limits,
        timeout: f64,
    ) -> Result<Self, SettingsError> {
        let lists = [
            ("max_velocity", &limits.max_velocity),
            ("min_velocity", &limits.min_velocity),
            ("max_accel", &limits.max_accel),
            ("max_decel", &limits.max_decel),
            ("deadband_velocity", &limits.deadband_velocity),
        ];
        if let Some((name, _)) = lists.iter().find(|(_, l)| l.len() != 3) {
            return Err(SettingsError::NotThree(name));
        }
        for i in 0..3 {
            if limits.max_decel[i] > 0.0 {
                return Err(SettingsError::Sign("max_decel", "negative"));
            }
            if limits.max_accel[i] < 0.0 {
                return Err(SettingsError::Sign("max_accel", "positive"));
            }
            if limits.min_velocity[i] > 0.0 {
                return Err(SettingsError::Sign("min_velocity", "negative"));
            }
            if limits.max_velocity[i] < 0.0 {
                return Err(SettingsError::Sign("max_velocity", "positive"));
            }
        }
        if !(frequency > 0.0 && frequency.is_finite()) {
            return Err(SettingsError::Frequency(frequency));
        }
        Ok(Settings {
            frequency,
            feedback,
            max_speed: limits.max_velocity[0],
            min_speed: limits.min_velocity[0],
            max_turn_rate: limits.max_velocity[2],
            accel: limits.max_accel[0],
            decel: -limits.max_decel[0],
            deadband_speed: limits.deadband_velocity[0],
            deadband_turn: limits.deadband_velocity[2],
            timeout,
        })
    }
}

/// Why the settings were refused.
#[derive(Debug, Clone, PartialEq)]
pub enum SettingsError {
    /// A list isn't (x, y, theta).
    NotThree(&'static str),
    /// A list has a value of the wrong sign: (list, the sign it needs).
    Sign(&'static str, &'static str),
    /// `feedback` is neither OPEN_LOOP nor CLOSED_LOOP.
    Feedback(String),
    /// `smoothing_frequency` isn't above 0.
    Frequency(f64),
    /// A car number isn't above 0 (or, for the speed for steering, 0 or more).
    Car(&'static str),
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SettingsError::NotThree(n) => write!(f, "{n} must hold 3 values (x, y, theta)"),
            SettingsError::Sign(n, s) => write!(f, "{n}'s values must be {s}"),
            SettingsError::Feedback(s) => {
                write!(f, "feedback {s}: OPEN_LOOP or CLOSED_LOOP")
            }
            SettingsError::Frequency(h) => write!(f, "smoothing_frequency {h}: above 0"),
            SettingsError::Car(n) => write!(f, "the car's {n} must be above 0"),
        }
    }
}

impl std::error::Error for SettingsError {}

/// A command that can't be used: NaN or infinite, as Nav2 refuses it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotFinite;

impl fmt::Display for NotFinite {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "the command holds a NaN or an infinity: ignored")
    }
}

impl std::error::Error for NotFinite {}

/// A speed and a turn rate: m/s forward, rad/s left.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Motion {
    /// m/s, + forward.
    pub v: f64,
    /// rad/s, + left.
    pub w: f64,
}

/// One tick's result: what to send, and what the car's limits changed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Out {
    /// m/s, after the deadband.
    pub v: f64,
    /// rad/s, after the deadband.
    pub w: f64,
    /// rad: where the smoother has the wheels now.
    pub steer: f64,
    /// rad: where the command wanted them (within full lock).
    pub steer_asked: f64,
    /// True when the command's steering angle was beyond full lock.
    pub lock_limited: bool,
}

/// The smoother: settings, the car, and what it last sent.
#[derive(Debug, Clone)]
pub struct Smoother {
    settings: Settings,
    car: Car,
    /// The command, and when it came (s).
    command: Option<(Motion, f64)>,
    /// What went out last, before the deadband (open loop's "now").
    last: Motion,
    /// rad: where the wheels are, as far as the smoother knows.
    steer: f64,
    stopped: bool,
}

/// Below this (m/s) a command's turn rate says nothing about its bend.
const STILL: f64 = 1e-3;

impl Smoother {
    /// A smoother at rest, the wheels straight.
    ///
    /// # Errors
    ///
    /// [`SettingsError::Car`] for a car number out of range.
    pub fn new(settings: Settings, car: Car) -> Result<Self, SettingsError> {
        let positive = [
            ("wheelbase", car.wheelbase),
            ("max_steering", car.max_steering),
            ("steering_rate", car.steering_rate),
            ("max_accel", car.max_accel),
            ("max_brake", car.max_brake),
        ];
        if let Some((name, _)) = positive.iter().find(|(_, x)| !(*x > 0.0 && x.is_finite())) {
            return Err(SettingsError::Car(name));
        }
        if car.min_speed_for_steering.is_nan() || car.min_speed_for_steering < 0.0 {
            return Err(SettingsError::Car("min_speed_for_steering"));
        }
        if !(settings.frequency > 0.0 && settings.frequency.is_finite()) {
            return Err(SettingsError::Frequency(settings.frequency));
        }
        Ok(Smoother {
            settings,
            car,
            command: None,
            last: Motion::default(),
            steer: 0.0,
            stopped: false,
        })
    }

    /// The settings it runs with.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The car it smooths for.
    #[must_use]
    pub fn car(&self) -> &Car {
        &self.car
    }

    /// rad: where the smoother has the wheels.
    #[must_use]
    pub fn steer(&self) -> f64 {
        self.steer
    }

    /// A new command `(v, w)`, arrived or stamped at `at` s.
    ///
    /// # Errors
    ///
    /// [`NotFinite`] for a NaN or an infinity: the command is ignored.
    pub fn command(&mut self, v: f64, w: f64, at: f64) -> Result<(), NotFinite> {
        if !(v.is_finite() && w.is_finite() && at.is_finite()) {
            return Err(NotFinite);
        }
        self.command = Some((Motion { v, w }, at));
        Ok(())
    }

    /// One tick at `now` s. `measured`: the car's speed (m/s) for closed loop; open loop
    /// ignores it, and closed loop without it uses the last speed sent. None: nothing to
    /// send (no command yet, or stopped after the timeout).
    pub fn tick(&mut self, now: f64, measured: Option<f64>) -> Option<Out> {
        let (mut asked, at) = self.command?;
        if now - at > self.settings.timeout {
            if self.last == Motion::default() || self.stopped {
                self.stopped = true;
                return None;
            }
            // Stale: aim for a stop, from now on.
            asked = Motion::default();
            self.command = Some((asked, now));
        }
        self.stopped = false;
        let s = &self.settings;
        let car = &self.car;
        let dt = 1.0 / s.frequency;
        let current = match (s.feedback, measured) {
            (Feedback::ClosedLoop, Some(v)) if v.is_finite() => v,
            _ => self.last.v,
        };

        // Where the command wants the wheels; held when it hardly moves.
        let (steer_asked, lock_limited) = if asked.v.abs() >= STILL {
            let raw = (car.wheelbase * asked.w / asked.v).atan();
            (
                car.steer_for(asked.w / asked.v),
                raw.abs() > car.max_steering + 1e-9,
            )
        } else {
            (self.steer, false)
        };

        // The speed: within the limits, slowed for a turn the car may not take faster.
        let mut target = asked.v.clamp(s.min_speed, s.max_speed);
        let bend = car
            .curvature(steer_asked)
            .abs()
            .max(car.curvature(self.steer).abs());
        if s.max_turn_rate > 0.0 && bend > 0.0 {
            let cap = s.max_turn_rate / bend;
            target = target.clamp(-cap, cap);
        }
        let v = slew(
            current,
            target,
            s.accel.min(car.max_accel),
            s.decel.min(car.max_brake),
            dt,
        );

        // The wheels: only as fast as they turn, and only once the car moves.
        if v.abs() >= car.min_speed_for_steering {
            let step = car.steering_rate * dt;
            self.steer += (steer_asked - self.steer).clamp(-step, step);
        }
        let w = v * car.curvature(self.steer);
        self.last = Motion { v, w };
        Some(Out {
            v: if v.abs() < s.deadband_speed { 0.0 } else { v },
            w: if w.abs() < s.deadband_turn { 0.0 } else { w },
            steer: self.steer,
            steer_asked,
            lock_limited,
        })
    }
}

/// The car's measured speed (closed loop), and when it arrived.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    /// m/s.
    pub speed: f64,
    /// s: when it arrived.
    pub at: f64,
}

impl Reading {
    /// The speed while the reading is at most `max_age` s old; `None` once odometry has
    /// stopped, so closed loop runs on the last speed sent (as open loop) rather than on
    /// one reading for ever. Nav2's own smoother keeps the last reading.
    #[must_use]
    pub fn fresh(&self, now: f64, max_age: f64) -> Option<f64> {
        (now - self.at <= max_age).then_some(self.speed)
    }
}

/// Move `current` toward `target`: at most `accel * dt` getting faster (the same way),
/// `decel * dt` otherwise. Nav2's rule (velocity_smoother.cpp `applyConstraints`).
#[must_use]
pub fn slew(current: f64, target: f64, accel: f64, decel: f64, dt: f64) -> f64 {
    let faster = target.abs() >= current.abs() && current * target >= 0.0;
    let step = if faster { accel } else { decel } * dt;
    current + (target - current).clamp(-step, step)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Our nav2.yaml's settings, with a long timeout: the drives below send one command.
    fn settings() -> Settings {
        Settings {
            timeout: 10.0,
            ..Settings::default()
        }
    }

    fn smoother() -> Smoother {
        Smoother::new(settings(), Car::default()).unwrap()
    }

    fn run(s: &mut Smoother, from: f64, seconds: f64) -> Vec<Out> {
        let dt = 1.0 / s.settings.frequency;
        let n = (seconds / dt).round() as usize;
        (1..=n)
            .filter_map(|i| s.tick(from + i as f64 * dt, None))
            .collect()
    }

    #[test]
    fn nothing_before_the_first_command() {
        assert!(smoother().tick(1.0, None).is_none());
    }

    #[test]
    fn speeds_up_and_slows_down_at_the_limits() {
        let mut s = smoother();
        s.command(1.0, 0.0, 0.0).unwrap();
        let out = run(&mut s, 0.0, 0.5);
        assert!((out[0].v - 0.05).abs() < 1e-9);
        assert!((out.last().unwrap().v - 0.5).abs() < 1e-9);
        s.command(0.0, 0.0, 0.5).unwrap();
        let out = run(&mut s, 0.5, 0.1);
        // 2 m/s² slowing down: 0.1 m/s a tick.
        assert!((out[0].v - 0.4).abs() < 1e-9);
    }

    #[test]
    fn the_car_limits_win_when_tighter() {
        let settings = Settings {
            accel: 5.0,
            decel: 9.0,
            ..settings()
        };
        let mut s = Smoother::new(settings, Car::default()).unwrap();
        s.command(1.0, 0.0, 0.0).unwrap();
        assert!((s.tick(0.05, None).unwrap().v - 1.5 / 20.0).abs() < 1e-9);
    }

    #[test]
    fn speed_held_within_min_and_max() {
        let mut s = smoother();
        s.command(-1.0, 0.0, 0.0).unwrap();
        assert!(run(&mut s, 0.0, 1.0).iter().all(|o| o.v == 0.0));
        s.command(5.0, 0.0, 1.0).unwrap();
        assert!(run(&mut s, 1.0, 3.0).iter().all(|o| o.v <= 1.0 + 1e-12));
    }

    #[test]
    fn the_wheels_turn_no_faster_than_the_steering() {
        let mut s = smoother();
        s.command(1.0, 1.0 * Car::default().curvature(0.4), 0.0)
            .unwrap();
        let out = run(&mut s, 0.0, 1.0);
        let step = 0.6 / 20.0;
        let mut before = 0.0;
        for o in &out {
            assert!((o.steer - before).abs() <= step + 1e-12);
            before = o.steer;
        }
        // 0.4 rad at 0.6 rad/s: there after about 0.7 s.
        assert!((out.last().unwrap().steer - 0.4).abs() < 1e-9);
        assert!((out.last().unwrap().steer_asked - 0.4).abs() < 1e-9);
    }

    #[test]
    fn the_turn_rate_never_beats_the_turning_circle() {
        let mut s = smoother();
        // A spin on the spot, and a turn far past full lock.
        s.command(0.3, 2.0, 0.0).unwrap();
        for o in run(&mut s, 0.0, 3.0) {
            assert!(o.w.abs() <= Car::default().max_turn_rate(o.v) + 1e-12);
            assert!(o.lock_limited);
        }
    }

    #[test]
    fn the_bend_is_kept_while_the_speed_ramps() {
        let mut s = smoother();
        let car = Car::default();
        let k = car.curvature(0.2);
        s.command(1.0, k, 0.0).unwrap();
        let out = run(&mut s, 0.0, 2.0);
        let o = out.last().unwrap();
        assert!((o.w / o.v - k).abs() < 1e-9);
    }

    #[test]
    fn a_tight_turn_is_slowed_under_max_turn_rate() {
        let settings = Settings {
            max_turn_rate: 0.2,
            ..settings()
        };
        let mut s = Smoother::new(settings, Car::default()).unwrap();
        let k = Car::default().curvature(0.5);
        s.command(1.0, k, 0.0).unwrap();
        for o in run(&mut s, 0.0, 4.0) {
            assert!(o.w.abs() <= 0.2 + 1e-9);
        }
    }

    #[test]
    fn the_wheels_hold_while_the_car_is_too_slow() {
        let mut s = smoother();
        s.command(0.01, 0.01, 0.0).unwrap();
        let out = run(&mut s, 0.0, 0.5);
        assert!(out.iter().all(|o| o.steer == 0.0 && o.w == 0.0));
        // A stop asked for: the wheels stay where they were.
        let mut s = smoother();
        s.command(1.0, Car::default().curvature(0.3), 0.0).unwrap();
        run(&mut s, 0.0, 1.0);
        let held = s.steer();
        s.command(0.0, 0.0, 1.0).unwrap();
        run(&mut s, 1.0, 1.0);
        assert_eq!(s.steer(), held);
    }

    #[test]
    fn backwards_bends_the_same_way_as_asked() {
        let settings = Settings {
            min_speed: -0.5,
            ..settings()
        };
        let mut s = Smoother::new(settings, Car::default()).unwrap();
        // Backwards, turning right (w < 0): the wheels left, as ego would steer.
        let k = Car::default().curvature(0.2);
        s.command(-0.5, -0.5 * k, 0.0).unwrap();
        let o = *run(&mut s, 0.0, 2.0).last().unwrap();
        assert!((o.v + 0.5).abs() < 1e-9);
        assert!((o.steer - 0.2).abs() < 1e-9);
        assert!((o.w + 0.5 * k).abs() < 1e-9);
    }

    #[test]
    fn a_stale_command_stops_the_car_then_goes_quiet() {
        let mut s = Smoother::new(Settings::default(), Car::default()).unwrap();
        s.command(1.0, 0.0, 0.0).unwrap();
        run(&mut s, 0.0, 1.0);
        // No command since 0 s: after 1 s, slow to a stop at 2 m/s², then nothing.
        let out = run(&mut s, 1.0, 2.0);
        assert!(out.windows(2).all(|p| p[1].v <= p[0].v));
        assert_eq!(out.last().unwrap().v, 0.0);
        assert!(s.tick(3.1, None).is_none());
        assert!(s.tick(3.2, None).is_none());
        // A fresh command wakes it.
        s.command(0.5, 0.0, 3.2).unwrap();
        assert!(s.tick(3.25, None).is_some());
    }

    #[test]
    fn never_moved_and_stale_sends_nothing() {
        let mut s = Smoother::new(Settings::default(), Car::default()).unwrap();
        s.command(0.0, 0.0, 0.0).unwrap();
        assert!(s.tick(2.0, None).is_none());
    }

    #[test]
    fn the_deadband_sends_zero_but_keeps_ramping() {
        let settings = Settings {
            deadband_speed: 0.12,
            ..settings()
        };
        let mut s = Smoother::new(settings, Car::default()).unwrap();
        s.command(1.0, 0.0, 0.0).unwrap();
        let out = run(&mut s, 0.0, 0.15);
        assert_eq!((out[0].v, out[1].v), (0.0, 0.0));
        assert!((out[2].v - 0.15).abs() < 1e-9);
    }

    #[test]
    fn closed_loop_ramps_from_the_measured_speed() {
        let settings = Settings {
            feedback: Feedback::ClosedLoop,
            ..settings()
        };
        let mut s = Smoother::new(settings, Car::default()).unwrap();
        s.command(1.0, 0.0, 0.0).unwrap();
        let o = s.tick(0.05, Some(0.6)).unwrap();
        assert!((o.v - 0.65).abs() < 1e-9);
        // No measurement yet: from the last speed sent.
        assert!((s.tick(0.1, None).unwrap().v - 0.7).abs() < 1e-9);
    }

    #[test]
    fn a_stale_reading_is_not_used() {
        let r = Reading {
            speed: 0.6,
            at: 1.0,
        };
        assert_eq!(r.fresh(1.5, 1.0), Some(0.6));
        assert_eq!(r.fresh(2.0, 1.0), Some(0.6));
        assert_eq!(r.fresh(2.01, 1.0), None);
        // Odometry stopped at 0.6 m/s: closed loop ramps from what it sent, not 0.6.
        let settings = Settings {
            feedback: Feedback::ClosedLoop,
            ..settings()
        };
        let mut s = Smoother::new(settings, Car::default()).unwrap();
        s.command(1.0, 0.0, 0.0).unwrap();
        let o = s.tick(0.05, r.fresh(0.05, 1.0)).unwrap();
        assert!((o.v - 0.65).abs() < 1e-9);
        let o = s.tick(3.0, r.fresh(3.0, 1.0)).unwrap();
        assert!((o.v - 0.7).abs() < 1e-9);
    }

    #[test]
    fn not_finite_commands_are_refused() {
        let mut s = smoother();
        assert_eq!(s.command(f64::NAN, 0.0, 0.0), Err(NotFinite));
        assert!(s.tick(0.05, None).is_none());
    }

    #[test]
    fn nav2s_settings_are_checked_as_nav2_checks_them() {
        let good = Nav2Limits {
            max_velocity: vec![1.0, 0.0, 1.0],
            min_velocity: vec![-0.5, 0.0, -1.0],
            max_accel: vec![1.0, 0.0, 1.5],
            max_decel: vec![-2.0, 0.0, -1.5],
            deadband_velocity: vec![0.0, 0.0, 0.0],
        };
        let s = Settings::from_nav2(20.0, Feedback::OpenLoop, &good, 1.0).unwrap();
        assert_eq!((s.min_speed, s.decel, s.max_turn_rate), (-0.5, 2.0, 1.0));
        let bad = Nav2Limits {
            max_decel: vec![2.0, 0.0, -1.5],
            ..good.clone()
        };
        assert_eq!(
            Settings::from_nav2(20.0, Feedback::OpenLoop, &bad, 1.0),
            Err(SettingsError::Sign("max_decel", "negative"))
        );
        let short = Nav2Limits {
            max_accel: vec![1.0],
            ..good.clone()
        };
        assert_eq!(
            Settings::from_nav2(20.0, Feedback::OpenLoop, &short, 1.0),
            Err(SettingsError::NotThree("max_accel"))
        );
        assert!("SIDEWAYS".parse::<Feedback>().is_err());
        assert_eq!("CLOSED_LOOP".parse::<Feedback>(), Ok(Feedback::ClosedLoop));
        let car = Car {
            steering_rate: 0.0,
            ..Car::default()
        };
        assert_eq!(
            Smoother::new(settings(), car).unwrap_err(),
            SettingsError::Car("steering_rate")
        );
    }
}

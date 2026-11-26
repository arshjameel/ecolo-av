//! A small closed loop for tests and experiments: a driver, the filter, a car with lags.
//!
//! ```text
//!  driver ──(v, w)──► filter ──(v, w)──► ego-like turn ──(v_set, δ_set)──► car: speed lag,
//!    ▲                  ▲                                                  steering rate
//!    └──── the car's pose and speed, every tick ◄──────────────────────────────┘
//! ```
//!
//! The car is the kinematic bicycle, with the actuators the filter's model leaves out: a
//! first-order speed lag with acceleration and braking limits, and a steering motor that
//! moves no faster than its top rate. So the tests see the gap between the filter's model
//! and a car, as the real car will show it. Not Ravensim: no tyres, no sensors.

use crate::filter::{Command, Inputs, Report, SafetyFilter};
use crate::lane::Lane;

/// The car the loop drives: the bicycle model with lagging actuators.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Car {
    /// m (map).
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad.
    pub yaw: f64,
    /// m/s.
    pub v: f64,
    /// rad, the virtual front wheel.
    pub steer: f64,
}

/// How the car answers (vehicle.yaml's `actuators`, as guessed today).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarModel {
    /// m.
    pub wheelbase: f64,
    /// rad.
    pub max_steer: f64,
    /// rad/s: the steering motor's top speed.
    pub steer_rate: f64,
    /// s: the speed's first-order lag.
    pub speed_lag: f64,
    /// m/s²: the most it speeds up.
    pub max_accel: f64,
    /// m/s²: the most it slows.
    pub max_brake: f64,
}

impl Default for CarModel {
    fn default() -> Self {
        CarModel {
            wheelbase: 1.561,
            max_steer: 0.5,
            steer_rate: 0.8,
            speed_lag: 0.3,
            max_accel: 1.5,
            max_brake: 3.0,
        }
    }
}

impl Car {
    /// One step of `dt` s toward a set speed and steering angle.
    pub fn step(&mut self, m: &CarModel, v_set: f64, steer_set: f64, dt: f64) {
        let want = (v_set - self.v) / m.speed_lag;
        let accel = want.clamp(-m.max_brake, m.max_accel);
        let target = steer_set.clamp(-m.max_steer, m.max_steer);
        let turn = (target - self.steer).clamp(-m.steer_rate * dt, m.steer_rate * dt);
        self.steer += turn;
        // Semi-implicit Euler: the new speed moves the car.
        let v_new = self.v + accel * dt;
        // Never through zero by braking: a stop is a stop.
        self.v = if self.v > 0.0 && v_new < 0.0 && v_set >= 0.0 {
            0.0
        } else {
            v_new
        };
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v * self.steer.tan() / m.wheelbase * dt;
    }
}

/// ego's turn: (v, w) to a set speed and steering angle; at a standstill the wheels hold.
#[must_use]
pub fn ackermann(cmd: Command, wheelbase: f64, hold: f64) -> (f64, f64) {
    if cmd.v.abs() < 1e-3 {
        return (cmd.v, hold);
    }
    (cmd.v, (wheelbase * cmd.w / cmd.v).atan())
}

/// One tick of the loop, for logging.
#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// s since the start.
    pub t: f64,
    /// The car after this tick.
    pub car: Car,
    /// The filter's report (its input is the driver's command).
    pub report: Report,
    /// The lane frame the filter saw (NaNs: none).
    pub offset: f64,
    /// m to the end.
    pub room: f64,
}

/// Run `driver` through the filter for `seconds`, the filter at `rate_hz`, the car at 100 Hz.
/// `driver(t, car)` is the untrusted driver: anything it likes.
pub fn run<D>(
    filter: &mut SafetyFilter,
    model: &CarModel,
    lane: &Lane,
    start: Car,
    seconds: f64,
    rate_hz: f64,
    mut driver: D,
) -> Vec<Tick>
where
    D: FnMut(f64, &Car) -> Command,
{
    let sub = 0.01;
    let per_tick = ((1.0 / rate_hz) / sub).round().max(1.0) as usize;
    let dt = per_tick as f64 * sub;
    let steps = (seconds / dt).ceil() as usize;
    let mut car = start;
    let mut log = Vec::with_capacity(steps);
    for k in 0..steps {
        let t = k as f64 * dt;
        let frame = lane.frame(car.x, car.y, car.yaw);
        let inputs = Inputs {
            frame,
            speed: Some(car.v),
            dt,
        };
        let cmd = driver(t, &car);
        let report = filter.step(cmd, &inputs);
        let (v_set, steer_set) = ackermann(report.output, model.wheelbase, car.steer);
        for _ in 0..per_tick {
            car.step(model, v_set, steer_set, sub);
        }
        let after = lane.frame(car.x, car.y, car.yaw);
        log.push(Tick {
            t: t + dt,
            car,
            report,
            offset: after.map_or(f64::NAN, |f| f.offset),
            room: after.map_or(f64::NAN, |f| f.room),
        });
    }
    log
}

/// A straight lane along x: `length` m, `half_lane` m to each line.
#[must_use]
pub fn straight_lane(length: f64, half_lane: f64) -> Lane {
    let n = (length / 0.5).round() as usize + 1;
    Lane {
        spacing: 0.5,
        x: (0..n).map(|i| i as f64 * 0.5).collect(),
        y: vec![0.0; n],
        heading: vec![0.0; n],
        left: vec![half_lane; n],
        right: vec![half_lane; n],
        end: (n - 1) as f64 * 0.5,
    }
}

/// A lane bending left on a circle of `radius` m.
#[must_use]
pub fn bending_lane(length: f64, half_lane: f64, radius: f64) -> Lane {
    let n = (length / 0.5).round() as usize + 1;
    let step = 0.5 / radius;
    Lane {
        spacing: 0.5,
        x: (0..n).map(|i| radius * (i as f64 * step).sin()).collect(),
        y: (0..n)
            .map(|i| radius - radius * (i as f64 * step).cos())
            .collect(),
        heading: (0..n).map(|i| i as f64 * step).collect(),
        left: vec![half_lane; n],
        right: vec![half_lane; n],
        end: (n - 1) as f64 * 0.5,
    }
}

//! The car model: how the car answers a command. The Rust twin of
//! `aav_description/car_model.py`, with the same maths, step for step; the shared vectors
//! (`vectors.json`, from `gen_vectors.py`) check both.
//!
//! One step of `dt` seconds:
//!
//! 1. Dead time: each input waits `floor(dead_time / dt + 0.5)` steps in a delay line.
//! 2. Throttle and brake follow their inputs with first-order lags, stepped exactly.
//! 3. Speed: full throttle gives `max_accel`; the brake and drag take speed off, never
//!    past 0. The throttle's push and the brake's both scale by
//!    [`ActuatorConfig::share`], `mass / (mass + payload)`: a loaded car speeds up and
//!    stops slower.
//! 4. Steering: a deadband, the motor's rate limit, the linkage's lag, the offset, the
//!    limit.
//! 5. The kinematic bicycle about the rear axle (`base_link`).
//!
//! Units: m/s, m/s², rad, s. Throttle and brake 0..1. Positive steering turns left.
//! Every number in `vehicle.yaml`'s `actuators:` is a guess until the car-day tests.
//!
//! ```
//! use car_model::{ActuatorConfig, CarState, Command};
//!
//! let cfg = ActuatorConfig::default();
//! let trace = cfg
//!     .predict(&CarState::default(), &[Command::pedals(0.5, 0.0, 0.1)], 0.02, 2.0, None)
//!     .unwrap();
//! assert_eq!(trace.len(), 100);
//! assert!(trace[99].x > 0.0 && trace[99].y > 0.0);
//! ```
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use std::collections::VecDeque;
use std::fmt;
use std::path::Path;

use serde::Deserialize;
pub use spine_core::speed_loop::{SpeedLoop, SpeedLoopConfig, SpeedLoopProfile};

/// `vehicle.yaml`'s `actuators:` section, with the steering limit and the wheelbase from
/// its `vehicle:` section. The defaults are `vehicle.yaml`'s guesses.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default)]
pub struct ActuatorConfig {
    /// m/s² at full throttle, on the flat.
    pub max_accel: f64,
    /// Throttle (0..1) below which the car doesn't push.
    pub throttle_deadband: f64,
    /// s: the throttle's first-order lag.
    pub throttle_lag: f64,
    /// m/s² at full brake.
    pub max_brake: f64,
    /// s: the brake actuator's first-order lag.
    pub brake_lag: f64,
    /// 1/s: coasting, m/s lost per second per m/s.
    pub drag: f64,
    /// rad/s: the fastest the steering motor turns the wheels.
    pub steer_rate: f64,
    /// s: the linkage's first-order lag after the motor.
    pub steer_lag: f64,
    /// rad: where the wheels point when told straight.
    pub steer_offset: f64,
    /// rad: a change smaller than this doesn't move the motor.
    pub steer_deadband: f64,
    /// rad: full lock (`vehicle: max_steering_angle`).
    pub max_steering: f64,
    /// s before the throttle starts to answer.
    pub throttle_dead_time: f64,
    /// s before the brake starts to answer.
    pub brake_dead_time: f64,
    /// s before the steering motor starts to answer.
    pub steer_dead_time: f64,
    /// m, rear axle to front axle (`vehicle: wheelbase`).
    pub wheelbase: f64,
    /// kg, the empty car (`vehicle: mass`).
    pub mass: f64,
    /// kg aboard: people, cargo. Not in `vehicle.yaml`: a run's own.
    pub payload: f64,
}

impl Default for ActuatorConfig {
    fn default() -> Self {
        ActuatorConfig {
            max_accel: 1.5,
            throttle_deadband: 0.0,
            throttle_lag: 0.1,
            max_brake: 3.0,
            brake_lag: 0.2,
            drag: 0.1,
            steer_rate: 0.8,
            steer_lag: 0.1,
            steer_offset: 0.0,
            steer_deadband: 0.005,
            max_steering: 0.5,
            throttle_dead_time: 0.0,
            brake_dead_time: 0.0,
            steer_dead_time: 0.0,
            wheelbase: 1.561,
            mass: 181.0,
            payload: 0.0,
        }
    }
}

/// Why `vehicle.yaml` couldn't be read.
#[derive(Debug)]
pub enum ConfigError {
    /// The file couldn't be opened or read.
    Read(std::io::Error),
    /// It isn't the YAML expected: a `vehicle:` with `wheelbase` and `max_steering_angle`.
    Yaml(serde_yaml_ng::Error),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Read(e) => write!(f, "can't read vehicle.yaml: {e}"),
            ConfigError::Yaml(e) => write!(f, "vehicle.yaml isn't as expected: {e}"),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConfigError::Read(e) => Some(e),
            ConfigError::Yaml(e) => Some(e),
        }
    }
}

#[derive(Deserialize)]
struct VehicleFile {
    vehicle: VehicleSection,
    #[serde(default)]
    actuators: Option<ActuatorConfig>,
}

#[derive(Deserialize)]
struct VehicleSection {
    wheelbase: f64,
    max_steering_angle: f64,
    #[serde(default)]
    mass: Option<f64>,
}

impl ActuatorConfig {
    /// Read it from a `vehicle.yaml` file. Keys left out (the dead times, today) keep
    /// their defaults; keys it doesn't know are skipped.
    ///
    /// # Errors
    ///
    /// [`ConfigError::Read`] if the file can't be read; [`ConfigError::Yaml`] if it
    /// isn't the YAML expected.
    pub fn from_vehicle_yaml(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path).map_err(ConfigError::Read)?;
        Self::from_vehicle_yaml_str(&text)
    }

    /// Read it from the text of a `vehicle.yaml`.
    ///
    /// # Errors
    ///
    /// [`ConfigError::Yaml`] if it isn't the YAML expected.
    pub fn from_vehicle_yaml_str(text: &str) -> Result<Self, ConfigError> {
        let file: VehicleFile = serde_yaml_ng::from_str(text).map_err(ConfigError::Yaml)?;
        let base = file.actuators.unwrap_or_default();
        Ok(ActuatorConfig {
            max_steering: file.vehicle.max_steering_angle,
            wheelbase: file.vehicle.wheelbase,
            mass: file.vehicle.mass.unwrap_or(base.mass),
            ..base
        })
    }

    /// `mass / (mass + payload)`: the share of the empty car's push and braking left
    /// with a load aboard (a = F / m). 1 with no payload.
    #[must_use]
    pub fn share(&self) -> f64 {
        self.mass / (self.mass + self.payload)
    }
}

/// How a command moves the car: the pedals as they are, or a set speed for a speed loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drive {
    /// Throttle and brake (0..1), and the gear.
    Pedals {
        /// 0..1.
        throttle: f64,
        /// 0..1.
        brake: f64,
        /// The gear: true for reverse.
        reverse: bool,
    },
    /// m/s, negative backwards: a speed loop turns it into the pedals.
    Speed(f64),
}

/// One step's command: how to move, and the steering target (rad, + left).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// The pedals, or a set speed.
    pub drive: Drive,
    /// The steering target (rad, + left).
    pub steer: f64,
}

impl Command {
    /// Forward pedals and a steering target.
    #[must_use]
    pub fn pedals(throttle: f64, brake: f64, steer: f64) -> Self {
        Command {
            drive: Drive::Pedals {
                throttle,
                brake,
                reverse: false,
            },
            steer,
        }
    }

    /// A set speed (m/s, - backwards) and a steering target.
    #[must_use]
    pub fn speed(speed: f64, steer: f64) -> Self {
        Command {
            drive: Drive::Speed(speed),
            steer,
        }
    }
}

/// The whole car at one instant. `x`, `y`, `yaw` are in the frame of the first state
/// (all 0: `base_link`, as ego's trace). The delay lines hold the inputs still waiting
/// out their dead time, oldest first; the throttle's is signed (- in reverse).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CarState {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left.
    pub yaw: f64,
    /// m/s, + forward.
    pub v: f64,
    /// rad: the wheels' real angle, offset included.
    pub steer: f64,
    /// 0..1, after the lag.
    pub throttle_level: f64,
    /// 0..1, after the lag.
    pub brake_level: f64,
    /// rad: the steering motor, before the linkage and offset.
    pub steer_motor: f64,
    /// rad: where the motor drives to (moves only past the deadband).
    pub steer_setpoint: f64,
    /// rad: after the linkage's lag, before the offset.
    pub steer_linkage: f64,
    /// The throttle commands still in the dead time, oldest first.
    pub throttle_delay: VecDeque<f64>,
    /// The brake commands still in the dead time, oldest first.
    pub brake_delay: VecDeque<f64>,
    /// The steering targets still in the dead time, oldest first.
    pub steer_delay: VecDeque<f64>,
}

/// Why a step or a prediction couldn't run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModelError {
    /// `dt` must be above 0.
    NonPositiveDt(f64),
    /// A [`Drive::Speed`] command needs a speed loop.
    NoSpeedLoop,
    /// [`ActuatorConfig::predict`] needs at least one command.
    NoCommands,
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ModelError::NonPositiveDt(dt) => write!(f, "dt must be positive, not {dt}"),
            ModelError::NoSpeedLoop => f.write_str("a set speed needs a speed loop"),
            ModelError::NoCommands => f.write_str("predict needs at least one command"),
        }
    }
}

impl std::error::Error for ModelError {}

/// How many steps of `dt` a dead time waits: `floor(dead_time / dt + 0.5)`.
#[must_use]
pub fn delay_steps(dead_time: f64, dt: f64) -> usize {
    (dead_time / dt + 0.5).floor().max(0.0) as usize
}

/// Push `value` into a delay line; return what comes out now: the value pushed `steps`
/// steps ago, or `rest` until the line is that long. A longer line (dt grew) drops its
/// oldest values.
pub fn delay(line: &mut VecDeque<f64>, value: f64, steps: usize, rest: f64) -> f64 {
    if steps == 0 {
        line.clear();
        return value;
    }
    line.push_back(value);
    if line.len() > steps {
        while line.len() > steps + 1 {
            line.pop_front();
        }
        line.pop_front().unwrap_or(rest)
    } else {
        rest
    }
}

/// First-order lag: `value` toward `target` with time constant `tau` (0: at once).
#[must_use]
pub fn lag(value: f64, target: f64, tau: f64, dt: f64) -> f64 {
    if tau <= 0.0 {
        return target;
    }
    value + (target - value) * (1.0 - (-dt / tau).exp())
}

fn clamp(x: f64, low: f64, high: f64) -> f64 {
    x.max(low).min(high)
}

impl ActuatorConfig {
    /// Advance the car one step of `dt` seconds. A [`Drive::Speed`] command needs
    /// `speed_loop`, which this call advances.
    ///
    /// # Errors
    ///
    /// [`ModelError::NonPositiveDt`] for `dt` not above 0; [`ModelError::NoSpeedLoop`]
    /// for a [`Drive::Speed`] without one.
    pub fn step(
        &self,
        state: &CarState,
        command: Command,
        dt: f64,
        speed_loop: Option<&mut SpeedLoop>,
    ) -> Result<CarState, ModelError> {
        if dt <= 0.0 || dt.is_nan() {
            return Err(ModelError::NonPositiveDt(dt));
        }
        let c = self;
        let (throttle, brake, reverse) = match command.drive {
            Drive::Pedals {
                throttle,
                brake,
                reverse,
            } => (throttle, brake, reverse),
            Drive::Speed(speed) => {
                let lp = speed_loop.ok_or(ModelError::NoSpeedLoop)?;
                let d = lp.update(speed as f32, state.v as f32, dt as f32);
                (f64::from(d.throttle), f64::from(d.brake), d.reverse)
            }
        };
        let mut next = state.clone();

        // 1. Dead times.
        let signed = delay(
            &mut next.throttle_delay,
            if reverse { -throttle } else { throttle },
            delay_steps(c.throttle_dead_time, dt),
            0.0,
        );
        let brake = delay(
            &mut next.brake_delay,
            brake,
            delay_steps(c.brake_dead_time, dt),
            0.0,
        );
        let target = delay(
            &mut next.steer_delay,
            clamp(command.steer, -c.max_steering, c.max_steering),
            delay_steps(c.steer_dead_time, dt),
            state.steer_setpoint,
        );

        // 2, 3. The pedals through their lags; the speed, never reversed by the brake.
        next.throttle_level = lag(state.throttle_level, signed.abs(), c.throttle_lag, dt);
        next.brake_level = lag(state.brake_level, brake, c.brake_lag, dt);
        let pushing = if next.throttle_level > c.throttle_deadband {
            next.throttle_level
        } else {
            0.0
        };
        let share = c.share();
        let push = pushing * c.max_accel * share * if signed < 0.0 { -1.0 } else { 1.0 };
        let v = state.v + push * dt;
        let resist = (next.brake_level * c.max_brake * share + c.drag * v.abs()) * dt;
        next.v = if v > 0.0 {
            (v - resist).max(0.0)
        } else {
            (v + resist).min(0.0)
        };

        // 4. Steering: deadband, the motor's rate limit, the linkage's lag.
        if (target - state.steer_setpoint).abs() > c.steer_deadband {
            next.steer_setpoint = target;
        }
        let most = c.steer_rate * dt;
        next.steer_motor =
            state.steer_motor + clamp(next.steer_setpoint - state.steer_motor, -most, most);
        next.steer_linkage = lag(state.steer_linkage, next.steer_motor, c.steer_lag, dt);
        next.steer = clamp(
            next.steer_linkage + c.steer_offset,
            -c.max_steering,
            c.max_steering,
        );

        // 5. The kinematic bicycle about the rear axle.
        next.yaw = state.yaw + next.v * next.steer.tan() / c.wheelbase * dt;
        let mid = 0.5 * (state.yaw + next.yaw);
        next.x = state.x + next.v * mid.cos() * dt;
        next.y = state.y + next.v * mid.sin() * dt;
        Ok(next)
    }

    /// The trace: the states every `dt` for `horizon` seconds ahead, the first one `dt`
    /// ahead. One command a step, the last one held. `speed_loop` (for set speeds) is
    /// copied: the caller's own is never advanced.
    ///
    /// # Errors
    ///
    /// [`ModelError::NoCommands`] for no command; [`ModelError::NonPositiveDt`];
    /// [`ModelError::NoSpeedLoop`] for a [`Drive::Speed`] without one.
    pub fn predict(
        &self,
        state: &CarState,
        commands: &[Command],
        dt: f64,
        horizon: f64,
        speed_loop: Option<&SpeedLoop>,
    ) -> Result<Vec<CarState>, ModelError> {
        let last = *commands.last().ok_or(ModelError::NoCommands)?;
        if dt <= 0.0 || dt.is_nan() {
            return Err(ModelError::NonPositiveDt(dt));
        }
        let mut speed_loop = speed_loop.cloned();
        let steps = delay_steps(horizon, dt);
        let mut states = Vec::with_capacity(steps);
        let mut now = state.clone();
        for i in 0..steps {
            let command = commands.get(i).copied().unwrap_or(last);
            now = self.step(&now, command, dt, speed_loop.as_mut())?;
            states.push(now.clone());
        }
        Ok(states)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delay_line_gives_the_value_from_n_steps_ago() {
        let mut line = VecDeque::new();
        let outs: Vec<f64> = [1.0, 2.0, 3.0, 4.0]
            .iter()
            .map(|&v| delay(&mut line, v, 2, 0.0))
            .collect();
        assert_eq!(outs, [0.0, 0.0, 1.0, 2.0]);
        let mut long = VecDeque::from([1.0, 2.0, 3.0]);
        assert_eq!(delay(&mut long, 4.0, 1, 0.0), 3.0);
        assert_eq!(long, VecDeque::from([4.0]));
    }

    #[test]
    fn delay_steps_round_half_up() {
        assert_eq!(delay_steps(0.05, 0.02), 3);
        assert_eq!(delay_steps(0.0, 0.02), 0);
    }

    #[test]
    fn bad_calls_say_why() {
        let cfg = ActuatorConfig::default();
        let s = CarState::default();
        assert_eq!(
            cfg.step(&s, Command::pedals(0.0, 0.0, 0.0), 0.0, None),
            Err(ModelError::NonPositiveDt(0.0))
        );
        assert_eq!(
            cfg.step(&s, Command::speed(1.0, 0.0), 0.02, None),
            Err(ModelError::NoSpeedLoop)
        );
        assert_eq!(
            cfg.predict(&s, &[], 0.02, 1.0, None),
            Err(ModelError::NoCommands)
        );
    }

    #[test]
    fn predict_leaves_the_callers_loop_alone() {
        let cfg = ActuatorConfig::default();
        let lp = SpeedLoop::new(SpeedLoopProfile::Default.config());
        let trace = cfg
            .predict(
                &CarState::default(),
                &[Command::speed(1.0, 0.2)],
                0.02,
                2.0,
                Some(&lp),
            )
            .unwrap();
        assert!(trace[99].v > 0.5 && trace[99].y > 0.0);
        let mut fresh = SpeedLoop::new(SpeedLoopProfile::Default.config());
        let mut used = lp.clone();
        assert_eq!(used.update(1.0, 0.0, 0.02), fresh.update(1.0, 0.0, 0.02));
    }

    #[test]
    fn reads_the_real_vehicle_yaml() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../config/vehicle.yaml");
        let cfg = ActuatorConfig::from_vehicle_yaml(path).unwrap();
        assert_eq!(cfg.wheelbase, 1.561);
        assert_eq!(cfg.max_steering, 0.5);
        assert_eq!(cfg.throttle_dead_time, 0.0);
        assert_eq!(cfg.max_brake, 3.0);
        assert_eq!(cfg.mass, 181.0);
        assert_eq!(cfg.payload, 0.0);
    }

    #[test]
    fn a_loaded_car_stops_further() {
        let stop = |payload: f64| {
            let cfg = ActuatorConfig {
                payload,
                ..ActuatorConfig::default()
            };
            let mut s = CarState {
                v: 2.0,
                ..CarState::default()
            };
            while s.v > 0.0 {
                s = cfg
                    .step(&s, Command::pedals(0.0, 1.0, 0.0), 0.01, None)
                    .unwrap();
            }
            s.x
        };
        let (empty, loaded) = (stop(0.0), stop(90.0));
        assert!(loaded > empty * 1.3, "empty {empty} m, loaded {loaded} m");
    }

    #[test]
    fn a_missing_file_is_a_read_error() {
        assert!(matches!(
            ActuatorConfig::from_vehicle_yaml("/no/such/vehicle.yaml"),
            Err(ConfigError::Read(_))
        ));
    }
}

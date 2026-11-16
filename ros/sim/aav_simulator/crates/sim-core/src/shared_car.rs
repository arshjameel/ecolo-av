//! The shared car model (crate `car-model`, `aav_description`'s Rust twin of
//! `car_model.py`) behind [`CarModel`]: the car ego predicts with, MPPI plans with and
//! Gazebo's adapter drives, here as the simulator's car (feature `shared`).
//!
//! It takes the actuators' demands ([`Command::Drive`]), as [`ActuatorCar`] does, and
//! gives the same states: `tests` drives both side by side. The node doesn't use it yet:
//! it runs [`TwistFollower`] behind `gazebo_adapter`'s actuator model, as Gazebo does.
//!
//! [`ActuatorCar`]: crate::car::ActuatorCar
//! [`TwistFollower`]: crate::car::TwistFollower

use car_model::{ActuatorConfig as SharedConfig, CarState as SharedState, ConfigError};

use crate::car::{ActuatorLevels, CarModel, CarState, Command, Drive};
use crate::math::wrap;

/// The shared car model as the simulator's car.
#[derive(Debug, Clone)]
pub struct SharedCar {
    cfg: SharedConfig,
    state: SharedState,
    accel: f64,
}

impl SharedCar {
    /// The shared car with these actuator settings, standing still at the origin.
    #[must_use]
    pub fn new(cfg: SharedConfig) -> Self {
        Self {
            cfg,
            state: SharedState::default(),
            accel: 0.0,
        }
    }

    /// The car from a vehicle.yaml file.
    ///
    /// # Errors
    ///
    /// [`ConfigError`] if the file can't be read or a number in it can't work.
    pub fn from_vehicle_yaml(path: impl AsRef<std::path::Path>) -> Result<Self, ConfigError> {
        SharedConfig::from_vehicle_yaml(path).map(Self::new)
    }

    /// The actuator settings this car runs with.
    #[must_use]
    pub fn config(&self) -> &SharedConfig {
        &self.cfg
    }
}

impl CarModel for SharedCar {
    fn step(&mut self, command: &Command, dt: f64) -> CarState {
        let d = match *command {
            Command::Drive(d) => d,
            // A twist isn't a demand: hold the car, as a silent spine does.
            Command::Twist { .. } => Drive::SILENT,
        };
        let asked = car_model::Command {
            drive: car_model::Drive::Pedals {
                throttle: d.throttle.clamp(0.0, 1.0),
                brake: d.brake.clamp(0.0, 1.0),
                reverse: d.reverse,
            },
            steer: d.steering,
        };
        let before = self.state.v;
        // Only a dt of 0 or less fails: then nothing moves.
        if let Ok(next) = self.cfg.step(&self.state, asked, dt, None) {
            self.state = next;
            self.accel = (self.state.v - before) / dt;
        }
        self.state()
    }

    fn state(&self) -> CarState {
        let s = &self.state;
        CarState {
            x: s.x,
            y: s.y,
            yaw: wrap(s.yaw),
            speed: s.v,
            accel: self.accel,
            yaw_rate: s.v * s.steer.tan() / self.cfg.wheelbase,
            steering: s.steer,
        }
    }

    fn reset(&mut self, x: f64, y: f64, yaw: f64) {
        self.state = SharedState {
            x,
            y,
            yaw,
            ..SharedState::default()
        };
        self.accel = 0.0;
    }

    fn actuators(&self) -> Option<ActuatorLevels> {
        Some(ActuatorLevels {
            throttle: self.state.throttle_level,
            brake: self.state.brake_level,
            motor: self.state.steer_motor,
        })
    }

    fn hold(&mut self, at: &CarState) {
        self.state.x = at.x;
        self.state.y = at.y;
        self.state.yaw = at.yaw;
        self.state.v = 0.0;
        self.accel = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::car::{ActuatorCar, ActuatorConfig};

    fn same_numbers(c: &SharedConfig) -> ActuatorConfig {
        ActuatorConfig {
            max_accel: c.max_accel,
            throttle_deadband: c.throttle_deadband,
            throttle_lag: c.throttle_lag,
            max_brake: c.max_brake,
            brake_lag: c.brake_lag,
            drag: c.drag,
            steer_rate: c.steer_rate,
            steer_lag: c.steer_lag,
            steer_offset: c.steer_offset,
            steer_deadband: c.steer_deadband,
            max_steering_angle: c.max_steering,
            wheelbase: c.wheelbase,
            throttle_dead_time: c.throttle_dead_time,
            brake_dead_time: c.brake_dead_time,
            steer_dead_time: c.steer_dead_time,
        }
    }

    fn demand(i: usize) -> Command {
        let (throttle, brake, steering) = match i {
            0..=299 => (0.6, 0.0, 0.0),
            300..=599 => (0.4, 0.0, 0.3),
            600..=799 => (0.0, 0.0, -0.2),
            _ => (0.0, 0.8, 0.0),
        };
        Command::Drive(Drive {
            throttle,
            brake,
            reverse: false,
            steering,
        })
    }

    #[test]
    fn it_drives_as_the_actuator_car_does() {
        let dt = 0.01;
        for shared in [
            SharedConfig::default(),
            SharedConfig {
                throttle_dead_time: 0.05,
                brake_dead_time: 0.1,
                steer_dead_time: 0.03,
                steer_offset: 0.01,
                ..SharedConfig::default()
            },
        ] {
            let mut a = SharedCar::new(shared);
            let mut b = ActuatorCar::new(same_numbers(&shared), dt);
            for i in 0..1000 {
                let (s, t) = (a.step(&demand(i), dt), b.step(&demand(i), dt));
                assert!((s.x - t.x).abs() < 1e-9 && (s.y - t.y).abs() < 1e-9, "{i}");
                assert!((s.yaw - t.yaw).abs() < 1e-9 && (s.speed - t.speed).abs() < 1e-9);
                assert!((s.steering - t.steering).abs() < 1e-12);
            }
            let (la, lb) = (a.actuators().unwrap(), b.actuators().unwrap());
            assert!((la.brake - lb.brake).abs() < 1e-12 && (la.motor - lb.motor).abs() < 1e-12);
        }
    }

    #[test]
    fn reset_and_hold() {
        let mut car = SharedCar::new(SharedConfig::default());
        car.reset(1.0, 2.0, 0.5);
        let s = car.step(&demand(0), 0.01);
        assert!((s.x - 1.0).abs() < 0.01 && (s.y - 2.0).abs() < 0.01);
        car.hold(&s);
        assert_eq!(car.state().speed, 0.0);
        // A twist holds the car, as a silent spine does.
        let still = car.step(
            &Command::Twist {
                speed: 1.0,
                yaw_rate: 0.0,
            },
            0.01,
        );
        assert!(still.speed.abs() < 1e-12);
    }
}

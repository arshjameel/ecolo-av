//! The shared car model (crate `car-model`, `aav_description`'s Rust twin) behind
//! [`CarModel`]: the same maths as `car_model.py`, ego's trace and our simulator.
//!
//! [`SharedModel`] is the 0.3 merge of [`crate::LagModel`]: the same rules (exact lags, dead
//! times, the steering motor, the mid-step bicycle), with the spine's own speed loop
//! (`spine-core`, f32) instead of a copy of its `default` profile, so any profile
//! (`default`, `smooth`, `lqi`) can be planned with. A state holds the car model's state
//! and its speed loop; MPPI clones it once per sample.

use car_model::{ActuatorConfig, CarState, SpeedLoop, SpeedLoopProfile};

use crate::model::{wrap, CarModel, Command, Observation, Pose};

/// The shared car model, for MPPI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SharedModel {
    /// vehicle.yaml's `actuators:`, wheelbase and steering limit.
    pub car: ActuatorConfig,
    /// The spine's speed loop profile the car runs.
    pub profile: SpeedLoopProfile,
    /// s: the longest step (a longer `dt` is cut into equal steps no longer).
    pub max_step: f64,
}

impl Default for SharedModel {
    fn default() -> Self {
        Self {
            car: ActuatorConfig::default(),
            profile: SpeedLoopProfile::Default,
            max_step: 0.02,
        }
    }
}

/// The car model's state and its speed loop.
#[derive(Debug, Clone)]
pub struct SharedState {
    pub car: CarState,
    pub speed_loop: SpeedLoop,
}

impl SharedModel {
    /// Put the loop in the gear a car moving at `speed` is in (none while stopped). The
    /// loop's gear is its own: it changes only on a call that finds the car stopped with
    /// a target the other way, so ask once as if stopped, for no time (nothing integrates
    /// over 0 s).
    fn in_gear(&self, speed_loop: &mut SpeedLoop, speed: f64) {
        if speed.abs() >= f64::from(self.profile.config().stopped_speed) {
            let way = if speed < 0.0 { -1.0 } else { 1.0 };
            speed_loop.update(way, 0.0, 0.0);
        }
    }
}

impl CarModel for SharedModel {
    type State = SharedState;

    fn at_rest(&self, pose: Pose, speed: f64) -> SharedState {
        // Steering straight: the linkage holds minus the offset, as the lag model's does.
        let straight = -self.car.steer_offset;
        let mut speed_loop = SpeedLoop::new(self.profile.config());
        self.in_gear(&mut speed_loop, speed);
        SharedState {
            car: CarState {
                x: pose.x,
                y: pose.y,
                yaw: pose.yaw,
                v: speed,
                steer: 0.0,
                steer_motor: straight,
                steer_setpoint: straight,
                steer_linkage: straight,
                ..CarState::default()
            },
            speed_loop,
        }
    }

    fn step(&self, state: &mut SharedState, command: Command, dt: f64) {
        if dt.is_nan() || dt <= 0.0 {
            return;
        }
        let n = (dt / self.max_step.max(1e-4)).ceil().max(1.0);
        let h = dt / n;
        // A negative speed reverses: the spine's loop changes gear only while stopped.
        let asked = car_model::Command::speed(command.speed, command.steer);
        for _ in 0..n as usize {
            match self
                .car
                .step(&state.car, asked, h, Some(&mut state.speed_loop))
            {
                Ok(next) => state.car = next,
                Err(_) => return,
            }
        }
    }

    fn observe(&self, state: &mut SharedState, seen: &Observation) {
        state.car.x = seen.pose.x;
        state.car.y = seen.pose.y;
        state.car.yaw = seen.pose.yaw;
        state.car.v = seen.speed;
        self.in_gear(&mut state.speed_loop, seen.speed);
        if let Some(angle) = seen.steering.filter(|a| a.is_finite()) {
            state.car.steer = angle;
            state.car.steer_linkage = angle - self.car.steer_offset;
        }
        if let Some(level) = seen.throttle.filter(|l| l.is_finite()) {
            state.car.throttle_level = level.clamp(0.0, 1.0);
        }
        if let Some(level) = seen.brake.filter(|l| l.is_finite()) {
            state.car.brake_level = level.clamp(0.0, 1.0);
        }
    }

    fn pose(&self, state: &SharedState) -> Pose {
        Pose::new(state.car.x, state.car.y, wrap(state.car.yaw))
    }

    fn speed(&self, state: &SharedState) -> f64 {
        state.car.v
    }

    fn steering(&self, state: &SharedState) -> f64 {
        state.car.steer
    }

    fn wheelbase(&self) -> f64 {
        self.car.wheelbase
    }

    fn lag_aware(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lag::{Actuators, LagModel};

    /// The lag model with the shared model's numbers.
    fn twin(car: &ActuatorConfig) -> LagModel {
        LagModel {
            actuators: Actuators {
                max_accel: car.max_accel,
                throttle_deadband: car.throttle_deadband,
                throttle_lag: car.throttle_lag,
                max_brake: car.max_brake,
                brake_lag: car.brake_lag,
                drag: car.drag,
                steer_rate: car.steer_rate,
                steer_lag: car.steer_lag,
                steer_offset: car.steer_offset,
                steer_deadband: car.steer_deadband,
                throttle_dead_time: car.throttle_dead_time,
                brake_dead_time: car.brake_dead_time,
                steer_dead_time: car.steer_dead_time,
            },
            wheelbase: car.wheelbase,
            max_steering_angle: car.max_steering,
            ..LagModel::default()
        }
    }

    fn drive<M: CarModel>(model: &M, commands: &[Command]) -> Vec<(Pose, f64, f64)> {
        let mut s = model.at_rest(Pose::default(), 0.0);
        commands
            .iter()
            .map(|&c| {
                model.step(&mut s, c, 0.05);
                (model.pose(&s), model.speed(&s), model.steering(&s))
            })
            .collect()
    }

    #[test]
    fn it_drives_as_the_lag_model_does() {
        // Speed up, turn, slow, stop: the shared model (the spine's f32 loop) and the lag
        // model (its f64 copy of the default profile) agree to a centimetre.
        let commands: Vec<Command> = (0..320)
            .map(|i| match i {
                0..=59 => Command::new(1.0, 0.0),
                60..=119 => Command::new(1.5, 0.3),
                120..=159 => Command::new(0.5, -0.2),
                160..=199 => Command::new(0.0, 0.0),
                // Then backwards, turning, and a stop again.
                200..=279 => Command::new(-0.5, 0.25),
                _ => Command::new(0.0, 0.0),
            })
            .collect();
        for car in [
            ActuatorConfig::default(),
            ActuatorConfig {
                throttle_dead_time: 0.06,
                brake_dead_time: 0.1,
                steer_dead_time: 0.04,
                ..ActuatorConfig::default()
            },
        ] {
            let shared = drive(
                &SharedModel {
                    car,
                    ..SharedModel::default()
                },
                &commands,
            );
            let lag = drive(&twin(&car), &commands);
            for ((a, va, sa), (b, vb, sb)) in shared.iter().zip(&lag) {
                assert!(a.distance(b) < 0.01, "{a:?} {b:?}");
                assert!((va - vb).abs() < 0.01 && (sa - sb).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn observe_takes_what_was_measured() {
        let m = SharedModel::default();
        let mut s = m.at_rest(Pose::default(), 0.0);
        m.observe(
            &mut s,
            &Observation {
                pose: Pose::new(1.0, 2.0, 0.5),
                speed: 1.5,
                steering: Some(0.2),
                throttle: Some(2.0),
                brake: None,
            },
        );
        assert_eq!(m.pose(&s), Pose::new(1.0, 2.0, 0.5));
        assert_eq!((m.speed(&s), m.steering(&s)), (1.5, 0.2));
        assert_eq!(s.car.throttle_level, 1.0);
        assert!(m.lag_aware());
    }
}

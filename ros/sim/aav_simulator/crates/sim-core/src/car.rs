//! The car: how it answers a command, and where that takes it.
//!
//! Behind one trait, [`CarModel`], so the shared car-model crate (Rust and Python twins,
//! another piece of 0.3's work: `ros/bringup/aav_description/car-model`, crate `car-model`)
//! can replace what's here without the simulator changing: one `impl CarModel` round its
//! `ActuatorConfig::step`. [`ActuatorCar`] keeps that crate's rules meanwhile.
//! Two models today:
//!
//! | Model | Takes | Is |
//! |---|---|---|
//! | [`TwistFollower`] | a speed and a turn rate (`/vehicle/sim/cmd_vel`, from `sim_adapter`) | Gazebo's AckermannSteering system: the car follows within ±10 m/s², the wheels steer at once (P gain 100). `sim_adapter`'s actuator model sits in front, as with Gazebo |
//! | [`ActuatorCar`] | throttle, brake, gear and a steering target ([`Drive`]) | the actuator model itself: lags, rates, deadbands and dead time, from `vehicle.yaml`'s `actuators:` |
//!
//! Both end in the same kinematic bicycle on flat ground: x' = v cos yaw, y' = v sin yaw,
//! yaw' = v tan(steering) / wheelbase. Units: m, m/s, m/s², rad, s; throttle and brake 0..1;
//! positive steering turns left.

use std::collections::VecDeque;

use crate::math::wrap;

/// Where the car is and what it's doing: `base_link` (the ground under the rear axle)
/// in the world frame, which is the map frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CarState {
    /// m, east, in the world frame.
    pub x: f64,
    /// m, north, in the world frame.
    pub y: f64,
    /// rad, from east, + left.
    pub yaw: f64,
    /// m/s along the car, + forward.
    pub speed: f64,
    /// m/s², this step.
    pub accel: f64,
    /// rad/s, + left.
    pub yaw_rate: f64,
    /// rad: the virtual centre wheel's real angle.
    pub steering: f64,
}

/// What a car model is given each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// A speed (m/s) and a turn rate (rad/s): Gazebo's `cmd_vel`.
    Twist {
        /// m/s, + forward.
        speed: f64,
        /// rad/s, + left.
        yaw_rate: f64,
    },
    /// What a speed loop or firmware decided.
    Drive(Drive),
}

/// The actuators' demands: what the spine (or a stand-in for its speed loop) sends.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Drive {
    /// 0..1.
    pub throttle: f64,
    /// 0..1.
    pub brake: f64,
    /// Drive backwards.
    pub reverse: bool,
    /// rad: the steering target, + left.
    pub steering: f64,
}

impl Drive {
    /// No throttle and the brake full on: what a silent spine leaves.
    pub const SILENT: Drive = Drive {
        throttle: 0.0,
        brake: 1.0,
        reverse: false,
        steering: 0.0,
    };
}

/// The actuators' levels after a step, for `/vehicle/throttle`, brake and steering.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ActuatorLevels {
    /// 0..1.
    pub throttle: f64,
    /// 0..1.
    pub brake: f64,
    /// rad: the steering motor's angle, before the linkage's lag.
    pub motor: f64,
}

/// A car: commands in, motion out. The plug for the shared car-model crate.
pub trait CarModel: Send {
    /// Move the car on by `dt` seconds under this command.
    fn step(&mut self, command: &Command, dt: f64) -> CarState;

    /// Where the car is now.
    fn state(&self) -> CarState;

    /// Put the car here, stopped, its actuators at rest.
    fn reset(&mut self, x: f64, y: f64, yaw: f64);

    /// The actuators' levels, for models that have them.
    fn actuators(&self) -> Option<ActuatorLevels> {
        None
    }

    /// Something stopped the car: put it back at `at`, standing still, its actuators as
    /// they are (a wall stops the car, not the motor).
    fn hold(&mut self, at: &CarState);
}

/// The kinematic bicycle: move `s` on by `dt` at its speed and steering.
pub fn bicycle(s: &mut CarState, wheelbase: f64, dt: f64) {
    s.yaw_rate = s.speed * s.steering.tan() / wheelbase;
    // The midpoint heading: second order, so a circle stays a circle at 1 kHz.
    let mid = s.yaw + 0.5 * s.yaw_rate * dt;
    s.x += s.speed * mid.cos() * dt;
    s.y += s.speed * mid.sin() * dt;
    s.yaw = wrap(s.yaw + s.yaw_rate * dt);
}

fn clamp(x: f64, limit: f64) -> f64 {
    x.clamp(-limit, limit)
}

/// Gazebo's AckermannSteering system, as Gazebo's URDF set it until 16 Nov.
#[derive(Debug, Clone)]
pub struct TwistFollower {
    wheelbase: f64,
    /// rad: the centre wheel's limit (vehicle.yaml max_steering_angle).
    max_steering: f64,
    /// m/s²: Gazebo's min and max acceleration, ±10 (wide on purpose: the adapter limits).
    max_accel: f64,
    /// m/s: Gazebo's ±6.
    max_speed: f64,
    /// s: how fast the steering joints follow (1 / steer_p_gain).
    steer_lag: f64,
    target_steering: f64,
    state: CarState,
}

impl TwistFollower {
    /// A follower for a car this long (m) with this full lock (rad).
    #[must_use]
    pub fn new(wheelbase: f64, max_steering: f64) -> Self {
        Self {
            wheelbase,
            max_steering,
            max_accel: 10.0,
            max_speed: 6.0,
            steer_lag: 0.01,
            target_steering: 0.0,
            state: CarState::default(),
        }
    }
}

impl CarModel for TwistFollower {
    fn step(&mut self, command: &Command, dt: f64) -> CarState {
        let (speed, yaw_rate) = match *command {
            Command::Twist { speed, yaw_rate } => (speed, yaw_rate),
            // Not its kind of command: hold still, as Gazebo's car would with none.
            Command::Drive(_) => (0.0, 0.0),
        };
        let s = &mut self.state;
        let target = clamp(speed, self.max_speed);
        let before = s.speed;
        s.speed += clamp(target - before, self.max_accel * dt);
        s.accel = (s.speed - before) / dt;
        // The plugin turns the turn rate into a steering angle; at a standstill it can't.
        if speed.abs() > 1e-6 {
            self.target_steering = clamp(
                (self.wheelbase * yaw_rate / speed).atan(),
                self.max_steering,
            );
        }
        s.steering += (self.target_steering - s.steering) * (dt / self.steer_lag).min(1.0);
        bicycle(s, self.wheelbase, dt);
        *s
    }

    fn state(&self) -> CarState {
        self.state
    }

    fn hold(&mut self, at: &CarState) {
        self.state = CarState {
            speed: 0.0,
            accel: 0.0,
            yaw_rate: 0.0,
            ..*at
        };
    }

    fn reset(&mut self, x: f64, y: f64, yaw: f64) {
        self.target_steering = 0.0;
        self.state = CarState {
            x,
            y,
            yaw,
            ..CarState::default()
        };
    }
}

/// vehicle.yaml's `actuators:` section, plus the steering limit and wheelbase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorConfig {
    /// m/s² at full throttle, on the flat.
    pub max_accel: f64,
    /// Throttle (0..1) below which the car doesn't push.
    pub throttle_deadband: f64,
    /// s: the throttle's first-order lag.
    pub throttle_lag: f64,
    /// m/s² at full brake.
    pub max_brake: f64,
    /// s: the brake's first-order lag.
    pub brake_lag: f64,
    /// 1/s: coasting, m/s lost per second per m/s.
    pub drag: f64,
    /// rad/s: the steering motor's top speed.
    pub steer_rate: f64,
    /// s: the linkage's first-order lag after the motor.
    pub steer_lag: f64,
    /// rad: where the wheels point when told straight.
    pub steer_offset: f64,
    /// rad: a smaller change of target doesn't move the motor.
    pub steer_deadband: f64,
    /// rad: full lock (vehicle.yaml `max_steering_angle`).
    pub max_steering_angle: f64,
    /// m.
    pub wheelbase: f64,
    /// s before the throttle starts to move; 0 if not measured.
    pub throttle_dead_time: f64,
    /// s before the brake starts to move; 0 if not measured.
    pub brake_dead_time: f64,
    /// s before the steering starts to move; 0 if not measured.
    pub steer_dead_time: f64,
}

impl Default for ActuatorConfig {
    /// vehicle.yaml's guesses on 7 Nov, and actuator_model.py's defaults.
    fn default() -> Self {
        Self {
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
            max_steering_angle: 0.5,
            wheelbase: 1.561,
            throttle_dead_time: 0.0,
            brake_dead_time: 0.0,
            steer_dead_time: 0.0,
        }
    }
}

/// A pure delay: what goes in comes out `floor(dead_time / dt + 0.5)` steps later.
#[derive(Debug, Clone)]
struct DeadTime {
    queue: VecDeque<f64>,
    steps: usize,
}

impl DeadTime {
    fn new(seconds: f64, dt: f64, rest: f64) -> Self {
        let steps = if seconds > 0.0 && dt > 0.0 {
            (seconds / dt + 0.5).floor() as usize
        } else {
            0
        };
        Self {
            queue: std::iter::repeat_n(rest, steps).collect(),
            steps,
        }
    }

    fn pass(&mut self, value: f64) -> f64 {
        if self.steps == 0 {
            return value;
        }
        self.queue.push_back(value);
        self.queue.pop_front().unwrap_or(value)
    }
}

/// First-order lag, discretised exactly: `(1 - exp(-dt / tau))` of the way (`tau` 0: at once).
/// The shared car-model crate's rule, and actuator_model.py's.
fn lag(value: f64, target: f64, tau: f64, dt: f64) -> f64 {
    if tau <= 0.0 {
        target
    } else {
        value + (target - value) * (1.0 - (-dt / tau).exp())
    }
}

/// The actuator model: throttle, brake and steering through their lags, then the bicycle.
///
/// The exact rules (the shared car-model crate keeps the same):
/// - throttle and brake levels: first-order lags (exact: 1 - exp(-dt/lag)) toward their demands;
/// - accel = max_accel x throttle (only above the deadband) - max_brake x brake - drag x v;
///   the brake and drag only slow the car, never reverse it;
/// - the steering motor moves toward its target at most steer_rate x dt, and only for a
///   change bigger than the deadband; the wheels lag the motor by steer_lag, plus the
///   offset, clamped to full lock;
/// - each dead time delays its input by floor(dead_time / dt + 0.5) steps;
/// - the bicycle moves on the new speed and steering, at the mid-step heading.
#[derive(Debug, Clone)]
pub struct ActuatorCar {
    cfg: ActuatorConfig,
    dt: f64,
    throttle_delay: DeadTime,
    brake_delay: DeadTime,
    steer_delay: DeadTime,
    throttle: f64,
    brake: f64,
    setpoint: f64,
    motor: f64,
    wheels: f64,
    state: CarState,
}

impl ActuatorCar {
    /// `dt`: the step the dead times are counted in (the simulator's).
    #[must_use]
    pub fn new(cfg: ActuatorConfig, dt: f64) -> Self {
        Self {
            throttle_delay: DeadTime::new(cfg.throttle_dead_time, dt, 0.0),
            brake_delay: DeadTime::new(cfg.brake_dead_time, dt, 0.0),
            steer_delay: DeadTime::new(cfg.steer_dead_time, dt, 0.0),
            cfg,
            dt,
            throttle: 0.0,
            brake: 0.0,
            setpoint: 0.0,
            motor: 0.0,
            wheels: 0.0,
            state: CarState::default(),
        }
    }

    /// The actuators' settings this car runs with.
    #[must_use]
    pub fn config(&self) -> &ActuatorConfig {
        &self.cfg
    }

    fn drive(&mut self, d: &Drive, dt: f64) -> CarState {
        let c = self.cfg;
        let throttle_in = self.throttle_delay.pass(d.throttle.clamp(0.0, 1.0));
        let brake_in = self.brake_delay.pass(d.brake.clamp(0.0, 1.0));
        let steer_in = self.steer_delay.pass(d.steering);

        self.throttle = lag(self.throttle, throttle_in, c.throttle_lag, dt);
        self.brake = lag(self.brake, brake_in, c.brake_lag, dt);
        let pushing = if self.throttle > c.throttle_deadband {
            self.throttle
        } else {
            0.0
        };
        let push = pushing * c.max_accel * if d.reverse { -1.0 } else { 1.0 };
        let s = &mut self.state;
        let before = s.speed;
        let v = before + push * dt;
        let resist = (self.brake * c.max_brake + c.drag * v.abs()) * dt;
        s.speed = if v > 0.0 {
            (v - resist).max(0.0)
        } else {
            (v + resist).min(0.0)
        };
        s.accel = (s.speed - before) / dt;

        let target = clamp(steer_in, c.max_steering_angle);
        if (target - self.setpoint).abs() > c.steer_deadband {
            self.setpoint = target;
        }
        let reach = c.steer_rate * dt;
        self.motor += clamp(self.setpoint - self.motor, reach);
        self.wheels = lag(self.wheels, self.motor, c.steer_lag, dt);
        s.steering = clamp(self.wheels + c.steer_offset, c.max_steering_angle);

        bicycle(s, c.wheelbase, dt);
        *s
    }
}

impl CarModel for ActuatorCar {
    fn step(&mut self, command: &Command, dt: f64) -> CarState {
        let drive = match *command {
            Command::Drive(d) => d,
            // A twist isn't a demand: hold the car, as a silent spine does.
            Command::Twist { .. } => Drive::SILENT,
        };
        debug_assert!((dt - self.dt).abs() < 1e-9 || self.throttle_delay.steps == 0);
        self.drive(&drive, dt)
    }

    fn state(&self) -> CarState {
        self.state
    }

    fn reset(&mut self, x: f64, y: f64, yaw: f64) {
        *self = ActuatorCar::new(self.cfg, self.dt);
        self.state.x = x;
        self.state.y = y;
        self.state.yaw = yaw;
    }

    fn hold(&mut self, at: &CarState) {
        self.state = CarState {
            speed: 0.0,
            accel: 0.0,
            yaw_rate: 0.0,
            ..*at
        };
    }

    fn actuators(&self) -> Option<ActuatorLevels> {
        Some(ActuatorLevels {
            throttle: self.throttle,
            brake: self.brake,
            motor: self.motor,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.001;

    fn run(car: &mut dyn CarModel, command: Command, seconds: f64) -> CarState {
        let steps = (seconds / DT).round() as usize;
        (0..steps).fold(car.state(), |_, _| car.step(&command, DT))
    }

    #[test]
    fn full_throttle_tends_to_max_accel_over_drag() {
        let mut car = ActuatorCar::new(ActuatorConfig::default(), DT);
        let s = run(
            &mut car,
            Command::Drive(Drive {
                throttle: 1.0,
                ..Drive::default()
            }),
            60.0,
        );
        // Terminal speed: max_accel / drag = 15 m/s.
        assert!((s.speed - 15.0).abs() < 0.05, "{}", s.speed);
    }

    #[test]
    fn braking_stops_but_never_reverses() {
        let mut car = ActuatorCar::new(ActuatorConfig::default(), DT);
        let go = Command::Drive(Drive {
            throttle: 1.0,
            ..Drive::default()
        });
        run(&mut car, go, 2.0);
        let s = run(&mut car, Command::Drive(Drive::SILENT), 5.0);
        assert_eq!(s.speed, 0.0);
    }

    #[test]
    fn the_steering_motor_is_rate_limited() {
        let cfg = ActuatorConfig {
            steer_lag: 0.0,
            ..ActuatorConfig::default()
        };
        let mut car = ActuatorCar::new(cfg, DT);
        let turn = Command::Drive(Drive {
            steering: 0.4,
            brake: 0.0,
            ..Drive::default()
        });
        let s = run(&mut car, turn, 0.25);
        // 0.8 rad/s for 0.25 s: 0.2 rad, not yet the 0.4 asked.
        assert!((s.steering - 0.2).abs() < 1e-6, "{}", s.steering);
    }

    #[test]
    fn a_change_inside_the_deadband_does_not_move_the_motor() {
        let mut car = ActuatorCar::new(ActuatorConfig::default(), DT);
        let s = run(
            &mut car,
            Command::Drive(Drive {
                steering: 0.004,
                ..Drive::default()
            }),
            1.0,
        );
        assert_eq!(s.steering, 0.0);
    }

    #[test]
    fn dead_time_delays_the_throttle() {
        let cfg = ActuatorConfig {
            throttle_dead_time: 0.05,
            ..ActuatorConfig::default()
        };
        let mut car = ActuatorCar::new(cfg, DT);
        let go = Command::Drive(Drive {
            throttle: 1.0,
            ..Drive::default()
        });
        run(&mut car, go, 0.049);
        assert_eq!(car.actuators().map(|a| a.throttle), Some(0.0));
        run(&mut car, go, 0.01);
        assert!(car.actuators().is_some_and(|a| a.throttle > 0.0));
    }

    #[test]
    fn the_bicycle_drives_a_circle_of_the_right_radius() {
        let mut s = CarState {
            speed: 1.0,
            steering: 0.3,
            ..CarState::default()
        };
        let wheelbase = 1.561;
        let radius = wheelbase / 0.3f64.tan();
        let lap = std::f64::consts::TAU * radius;
        for _ in 0..(lap / DT).round() as usize {
            bicycle(&mut s, wheelbase, DT);
        }
        assert!(
            s.x.hypot(s.y) < 0.01,
            "back at the start: ({}, {})",
            s.x,
            s.y
        );
    }

    #[test]
    fn the_twist_follower_steers_to_match_the_turn_rate() {
        let mut car = TwistFollower::new(1.561, 0.5);
        let s = run(
            &mut car,
            Command::Twist {
                speed: 1.0,
                yaw_rate: 0.1,
            },
            2.0,
        );
        assert!((s.speed - 1.0).abs() < 1e-9);
        assert!((s.yaw_rate - 0.1).abs() < 1e-6, "{}", s.yaw_rate);
    }
}

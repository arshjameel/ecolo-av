//! The car with its actuators' lag: the 0.3 car-model spec, behind [`CarModel`].
//!
//! ```text
//! speed command ─> speed loop (the spine's PI) ─> throttle, brake demand
//!                    ─> dead time ─> first-order lag ─> acceleration ─> speed ─┐
//! steer command ─> clamp ─> dead time ─> deadband ─> motor (steer_rate) ─>     ├─> bicycle
//!                    linkage lag (steer_lag) ─> + offset, clamp ─> wheel angle ┘
//! ```
//!
//! | Stage | Rule (vehicle.yaml `actuators:`) |
//! |---|---|
//! | throttle, brake | each level moves 1 - exp(-dt / lag) of the way to its demand |
//! | acceleration | max_accel x throttle (above throttle_deadband) along the gear, - max_brake x brake - drag x v; braking and drag stop the car, never reverse it |
//! | steering | the motor moves toward the target at most steer_rate x dt, and only for a change over steer_deadband; the wheels lag the motor by steer_lag; + steer_offset, clamped to full lock |
//! | pose | bicycle: x' = v cos yaw, y' = v sin yaw, yaw' = v tan(steer) / wheelbase |
//! | dead time | each actuator's demand is delayed floor(dead_time / dt + 0.5) steps |
//!
//! The same rules as the shared `car-model` crate (`ros/bringup/aav_description/car-model`,
//! 0.3, merging first): exact first-order lags, the deadband against the motor's
//! setpoint, the new speed and angle with the mid-step heading. At its merge, its model
//! implements [`CarModel`] in this one's place.
//!
//! Speed commands: the spine's speed loop is modelled as aav_gazebo's `SpeedLoop` does
//! with its `default` profile (spine-core `speed_loop.rs`: a PI on the throttle, the
//! brake for overspeed past a coast band, a hold brake at a stop), both ways as
//! spine-core's loop runs: a negative speed asks for reverse; the gear changes only while
//! stopped (until then the loop brakes); the loop works along the gear, so reversing uses
//! the same law as going forward. The car is stepped in steps
//! of at most `max_step` (the spine's loop runs at 50 Hz: 0.02 s), so a 0.05 s MPPI step
//! is three Euler steps.

use crate::model::{bicycle, CarModel, Command, Observation, Pose};

/// The most steps a dead time may last: 64 x 0.02 s = 1.28 s.
pub const MAX_DELAY_STEPS: usize = 64;

/// vehicle.yaml's `actuators:` section. Units: m/s², s, rad, 1/s; levels 0..1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Actuators {
    /// m/s² at full throttle.
    pub max_accel: f64,
    /// Throttle below this doesn't push.
    pub throttle_deadband: f64,
    /// s: the throttle's first-order lag.
    pub throttle_lag: f64,
    /// m/s² at full brake.
    pub max_brake: f64,
    /// s: the brake's first-order lag.
    pub brake_lag: f64,
    /// 1/s: m/s lost per second per m/s, coasting.
    pub drag: f64,
    /// rad/s: the steering motor's top speed.
    pub steer_rate: f64,
    /// s: the linkage's lag after the motor.
    pub steer_lag: f64,
    /// rad: where the wheels point when told straight.
    pub steer_offset: f64,
    /// rad: a smaller change of target doesn't move the motor.
    pub steer_deadband: f64,
    /// s before the throttle starts to answer (0: none).
    pub throttle_dead_time: f64,
    /// s before the brake starts to answer.
    pub brake_dead_time: f64,
    /// s before the steering starts to answer.
    pub steer_dead_time: f64,
}

impl Default for Actuators {
    /// vehicle.yaml's guesses (7 Nov), no dead times.
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
            throttle_dead_time: 0.0,
            brake_dead_time: 0.0,
            steer_dead_time: 0.0,
        }
    }
}

/// The spine's speed loop: spine-core's `default` profile (aav_gazebo `SpeedLoopConfig()`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedLoop {
    /// Throttle per m/s of error.
    pub kp: f64,
    /// Throttle per (m/s x s).
    pub ki: f64,
    /// The most throttle the integral may add.
    pub integral_limit: f64,
    /// Brake per m/s of overspeed past `coast_band`.
    pub brake_gain: f64,
    /// m/s too fast that coasting handles.
    pub coast_band: f64,
    /// m/s: slower counts as stopped.
    pub stopped_speed: f64,
    /// Brake held while stopped with a zero target.
    pub hold_brake: f64,
}

impl Default for SpeedLoop {
    fn default() -> Self {
        Self {
            kp: 0.3,
            ki: 0.1,
            integral_limit: 0.3,
            brake_gain: 0.5,
            coast_band: 0.2,
            stopped_speed: 0.05,
            hold_brake: 0.3,
        }
    }
}

/// The car with lag: actuators, speed loop and size.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LagModel {
    /// The actuators' numbers.
    pub actuators: Actuators,
    /// The spine's speed loop.
    pub speed_loop: SpeedLoop,
    /// m, rear axle to front axle.
    pub wheelbase: f64,
    /// rad, full lock either side.
    pub max_steering_angle: f64,
    /// s: the longest Euler step (a longer `dt` is cut into equal steps no longer).
    pub max_step: f64,
}

impl Default for LagModel {
    fn default() -> Self {
        Self {
            actuators: Actuators::default(),
            speed_loop: SpeedLoop::default(),
            wheelbase: 1.561,
            max_steering_angle: 0.5,
            max_step: 0.02,
        }
    }
}

/// A fixed-length delay: what went in `n` pushes ago. No heap, so a state clones cheaply.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DelayLine {
    values: [f64; MAX_DELAY_STEPS],
    head: usize,
}

impl DelayLine {
    /// A line that has held `value` for ever.
    pub fn filled(value: f64) -> Self {
        Self {
            values: [value; MAX_DELAY_STEPS],
            head: 0,
        }
    }

    /// Put `value` in; return what went in `steps` pushes ago (`value` itself for 0).
    pub fn push(&mut self, value: f64, steps: usize) -> f64 {
        self.head = (self.head + 1) % MAX_DELAY_STEPS;
        self.values[self.head] = value;
        let back = steps.min(MAX_DELAY_STEPS - 1);
        self.values[(self.head + MAX_DELAY_STEPS - back) % MAX_DELAY_STEPS]
    }
}

/// Which way the throttle pushes: the spine's direction output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gear {
    /// The throttle pushes the car forwards.
    #[default]
    Forward,
    /// The throttle pushes the car backwards.
    Reverse,
}

impl Gear {
    /// The gear a speed asks for; `None` for 0 (a stop keeps the gear).
    pub fn of(speed: f64) -> Option<Self> {
        if speed > 0.0 {
            Some(Self::Forward)
        } else if speed < 0.0 {
            Some(Self::Reverse)
        } else {
            None
        }
    }

    /// +1 forward, -1 reverse: a speed along the gear is this times the signed speed.
    pub fn sign(self) -> f64 {
        match self {
            Self::Forward => 1.0,
            Self::Reverse => -1.0,
        }
    }
}

/// Everything the lag model carries: the pose, the actuators, the loop, the dead times.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LagState {
    /// base_link in odom.
    pub pose: Pose,
    /// m/s.
    pub speed: f64,
    /// 0..1, the throttle's level.
    pub throttle: f64,
    /// 0..1, the brake's level.
    pub brake: f64,
    /// rad: what the steering motor drives to (moves past the deadband only).
    pub setpoint: f64,
    /// rad: the steering motor's angle.
    pub motor: f64,
    /// rad: the wheels, after the linkage's lag (before the offset).
    pub wheels: f64,
    /// The speed loop's integral (throttle).
    pub integral: f64,
    /// The gear: [`Gear::Reverse`] pushes the car backwards. Changes only while stopped.
    pub gear: Gear,
    /// Throttle demands in their dead time.
    pub throttle_delay: DelayLine,
    /// Brake demands in their dead time.
    pub brake_delay: DelayLine,
    /// Steering targets in their dead time.
    pub steer_delay: DelayLine,
}

/// Move `level` toward `target` with a first-order lag, exactly: 1 - exp(-dt / lag) of
/// the way (0 lag: at once).
fn lagged(level: f64, target: f64, lag: f64, dt: f64) -> f64 {
    if lag <= 0.0 {
        target
    } else {
        level + (target - level) * (1.0 - (-dt / lag).exp())
    }
}

/// Steps of `dt` in `time` seconds: floor(time / dt + 0.5).
fn steps(time: f64, dt: f64) -> usize {
    if time <= 0.0 || dt <= 0.0 {
        0
    } else {
        (time / dt + 0.5).floor() as usize
    }
}

impl LagModel {
    /// The spine's speed loop: target and measured speed in, (throttle, brake) out, the
    /// throttle along `s.gear` (spine-core `SpeedLoop::update`, its PI law).
    fn demand(&self, s: &mut LagState, target: f64, dt: f64) -> (f64, f64) {
        let c = &self.speed_loop;
        let stopped = s.speed.abs() < c.stopped_speed;
        // The gear changes only while stopped; until then, brake.
        if let Some(wanted) = Gear::of(target).filter(|g| *g != s.gear) {
            s.integral = 0.0;
            if stopped {
                s.gear = wanted;
            } else {
                let brake = c
                    .hold_brake
                    .max(c.brake_gain * s.speed.abs())
                    .clamp(0.0, 1.0);
                return (0.0, brake);
            }
        }
        // Along the direction of travel, so both directions use one law.
        let sign = s.gear.sign();
        let (target, measured) = (target * sign, s.speed * sign);
        if target <= 0.0 {
            s.integral = 0.0;
            let brake = if stopped {
                c.hold_brake
            } else {
                c.hold_brake
                    .max(c.brake_gain * (measured - c.coast_band))
                    .clamp(0.0, 1.0)
            };
            return (0.0, brake);
        }
        let error = target - measured;
        let proportional = c.kp * error;
        s.integral = (s.integral + c.ki * error * dt).clamp(0.0, c.integral_limit.max(0.0));
        let overspeed = -error - c.coast_band;
        if overspeed > 0.0 {
            s.integral = 0.0;
            return (0.0, (c.brake_gain * overspeed).clamp(0.0, 1.0));
        }
        ((proportional + s.integral).clamp(0.0, 1.0), 0.0)
    }

    /// One Euler step of `dt` (at most `max_step`).
    fn euler(&self, s: &mut LagState, command: Command, dt: f64) {
        let a = &self.actuators;
        let (throttle, brake) = self.demand(s, command.speed, dt);
        // The gear rides with the throttle through its dead time (as the shared model's).
        let signed = s
            .throttle_delay
            .push(throttle * s.gear.sign(), steps(a.throttle_dead_time, dt));
        let brake = s.brake_delay.push(brake, steps(a.brake_dead_time, dt));
        s.throttle = lagged(s.throttle, signed.abs(), a.throttle_lag, dt);
        s.brake = lagged(s.brake, brake, a.brake_lag, dt);

        let pushing = if s.throttle > a.throttle_deadband {
            s.throttle
        } else {
            0.0
        };
        let way = if signed < 0.0 { -1.0 } else { 1.0 };
        let v = s.speed + way * pushing * a.max_accel * dt;
        // The brake and drag oppose the motion: they stop the car, never reverse it.
        let resist = (s.brake * a.max_brake + a.drag * v.abs()) * dt;
        s.speed = if v > 0.0 {
            (v - resist).max(0.0)
        } else {
            (v + resist).min(0.0)
        };

        let full = self.max_steering_angle;
        let target = s.steer_delay.push(
            command.steer.clamp(-full, full),
            steps(a.steer_dead_time, dt),
        );
        if (target - s.setpoint).abs() > a.steer_deadband {
            s.setpoint = target;
        }
        let most = a.steer_rate * dt;
        s.motor += (s.setpoint - s.motor).clamp(-most, most);
        s.wheels = lagged(s.wheels, s.motor, a.steer_lag, dt);

        let steering = self.wheel_angle(s);
        bicycle(&mut s.pose, s.speed, steering, self.wheelbase, dt);
    }

    fn wheel_angle(&self, s: &LagState) -> f64 {
        let full = self.max_steering_angle;
        (s.wheels + self.actuators.steer_offset).clamp(-full, full)
    }
}

impl CarModel for LagModel {
    type State = LagState;

    fn at_rest(&self, pose: Pose, speed: f64) -> LagState {
        let steer = -self.actuators.steer_offset;
        LagState {
            pose,
            speed,
            throttle: 0.0,
            brake: 0.0,
            setpoint: steer,
            motor: steer,
            wheels: steer,
            integral: 0.0,
            gear: Gear::of(speed).unwrap_or_default(),
            throttle_delay: DelayLine::filled(0.0),
            brake_delay: DelayLine::filled(0.0),
            // Before any command, the steering holds where it is.
            steer_delay: DelayLine::filled(steer),
        }
    }

    fn step(&self, state: &mut LagState, command: Command, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let n = (dt / self.max_step.max(1e-4)).ceil().max(1.0);
        let h = dt / n;
        for _ in 0..n as usize {
            self.euler(state, command, h);
        }
    }

    fn observe(&self, state: &mut LagState, seen: &Observation) {
        state.pose = seen.pose;
        state.speed = seen.speed;
        // A car seen moving is in the gear it moves in.
        if seen.speed.abs() >= self.speed_loop.stopped_speed {
            state.gear = Gear::of(seen.speed).unwrap_or(state.gear);
        }
        if let Some(angle) = seen.steering.filter(|a| a.is_finite()) {
            // Measured after the offset; the linkage's state is before it.
            state.wheels = angle - self.actuators.steer_offset;
        }
        if let Some(level) = seen.throttle.filter(|l| l.is_finite()) {
            state.throttle = level.clamp(0.0, 1.0);
        }
        if let Some(level) = seen.brake.filter(|l| l.is_finite()) {
            state.brake = level.clamp(0.0, 1.0);
        }
    }

    fn pose(&self, state: &LagState) -> Pose {
        state.pose
    }

    fn speed(&self, state: &LagState) -> f64 {
        state.speed
    }

    fn steering(&self, state: &LagState) -> f64 {
        self.wheel_angle(state)
    }

    fn wheelbase(&self) -> f64 {
        self.wheelbase
    }

    fn lag_aware(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(model: &LagModel, cmd: Command, seconds: f64, dt: f64) -> LagState {
        let mut s = model.at_rest(Pose::default(), 0.0);
        for _ in 0..(seconds / dt).round() as usize {
            model.step(&mut s, cmd, dt);
        }
        s
    }

    #[test]
    fn lagged_is_an_exact_first_order_lag() {
        assert!((lagged(0.0, 1.0, 0.1, 0.05) - (1.0 - (-0.5f64).exp())).abs() < 1e-15);
        assert_eq!(lagged(0.0, 1.0, 0.0, 0.05), 1.0);
        // Never overshoots, even with dt past the lag.
        assert!(lagged(0.0, 1.0, 0.01, 0.05) <= 1.0);
    }

    #[test]
    fn delay_line_returns_what_went_in_n_pushes_ago() {
        let mut d = DelayLine::filled(0.0);
        assert_eq!(d.push(1.0, 0), 1.0);
        let out: Vec<f64> = (2..6).map(|i| d.push(f64::from(i), 2)).collect();
        assert_eq!(out, vec![0.0, 1.0, 2.0, 3.0]);
    }

    #[test]
    fn speed_settles_near_the_command() {
        let m = LagModel::default();
        let s = run(&m, Command::new(1.0, 0.0), 20.0, 0.05);
        assert!((s.speed - 1.0).abs() < 0.05, "{}", s.speed);
    }

    #[test]
    fn braking_stops_without_reversing_the_car() {
        let m = LagModel::default();
        let mut s = run(&m, Command::new(2.0, 0.0), 10.0, 0.05);
        for _ in 0..200 {
            m.step(&mut s, Command::new(0.0, 0.0), 0.05);
            assert!(s.speed >= 0.0);
        }
        assert_eq!(s.speed, 0.0);
    }

    #[test]
    fn reverses_and_settles_near_a_negative_command() {
        let m = LagModel::default();
        let s = run(&m, Command::new(-0.5, 0.0), 20.0, 0.05);
        assert_eq!(s.gear, Gear::Reverse);
        assert!((s.speed + 0.5).abs() < 0.05, "{}", s.speed);
        assert!(s.pose.x < -5.0, "backwards: {:?}", s.pose);
        assert!(s.pose.yaw.abs() < 1e-12, "still facing forward");
    }

    #[test]
    fn a_change_of_direction_stops_first() {
        // Forward at 1 m/s, then asked to reverse: the loop brakes to a stop, the gear
        // changes, then it reverses.
        let m = LagModel::default();
        let mut s = run(&m, Command::new(1.0, 0.0), 10.0, 0.05);
        let mut stopped_forward = false;
        for _ in 0..200 {
            m.step(&mut s, Command::new(-0.5, 0.0), 0.05);
            if s.gear == Gear::Forward {
                assert!(s.speed >= 0.0, "braking, never reversing in forward gear");
                stopped_forward |= s.speed < m.speed_loop.stopped_speed;
            }
        }
        assert!(stopped_forward);
        assert_eq!(s.gear, Gear::Reverse);
        assert!(s.speed < -0.4, "{}", s.speed);
    }

    #[test]
    fn a_car_seen_reversing_is_in_reverse() {
        let m = LagModel::default();
        let mut s = m.at_rest(Pose::default(), 0.0);
        m.observe(
            &mut s,
            &Observation {
                speed: -0.4,
                ..Observation::default()
            },
        );
        assert_eq!(s.gear, Gear::Reverse);
        m.step(&mut s, Command::new(-0.4, 0.0), 0.05);
        assert!(s.speed < -0.3, "keeps reversing, no stop: {}", s.speed);
        assert_eq!(m.at_rest(Pose::default(), -0.3).gear, Gear::Reverse);
    }

    #[test]
    fn steering_moves_no_faster_than_the_motor() {
        let m = LagModel::default();
        let mut s = m.at_rest(Pose::default(), 0.0);
        let mut last = 0.0;
        for _ in 0..20 {
            m.step(&mut s, Command::new(0.0, 0.5), 0.05);
            assert!(s.motor - last <= m.actuators.steer_rate * 0.05 + 1e-12);
            last = s.motor;
        }
        // 1 s at 0.8 rad/s: the motor reached full lock; the wheels lag it.
        assert!((s.motor - 0.5).abs() < 1e-9);
        assert!(m.steering(&s) < s.motor);
    }

    #[test]
    fn deadband_ignores_a_small_change() {
        let m = LagModel::default();
        let s = run(&m, Command::new(0.0, 0.004), 1.0, 0.05);
        assert_eq!(s.motor, 0.0);
    }

    #[test]
    fn steering_dead_time_delays_the_start() {
        let m = LagModel {
            actuators: Actuators {
                steer_dead_time: 0.2,
                ..Actuators::default()
            },
            ..LagModel::default()
        };
        let mut s = m.at_rest(Pose::default(), 0.0);
        for _ in 0..9 {
            m.step(&mut s, Command::new(0.0, 0.3), 0.02);
        }
        assert_eq!(s.motor, 0.0, "nothing before 0.2 s");
        for _ in 0..5 {
            m.step(&mut s, Command::new(0.0, 0.3), 0.02);
        }
        assert!(s.motor > 0.0);
    }

    #[test]
    fn offset_shifts_the_wheels_and_is_clamped() {
        let m = LagModel {
            actuators: Actuators {
                steer_offset: 0.05,
                ..Actuators::default()
            },
            ..LagModel::default()
        };
        let s = run(&m, Command::new(0.0, 0.5), 5.0, 0.05);
        assert!((m.steering(&s) - 0.5).abs() < 1e-9, "clamped to full lock");
        let mut straight = m.at_rest(Pose::default(), 0.0);
        straight.wheels = 0.0;
        assert!((m.steering(&straight) - 0.05).abs() < 1e-12);
    }

    #[test]
    fn throttle_deadband_holds_a_small_push() {
        let m = LagModel {
            actuators: Actuators {
                throttle_deadband: 0.5,
                ..Actuators::default()
            },
            ..LagModel::default()
        };
        // 0.3 m/s asks for a throttle under 0.5 at first: the car doesn't move until the
        // integral builds it past the deadband.
        let s = run(&m, Command::new(0.3, 0.0), 0.2, 0.05);
        assert_eq!(s.speed, 0.0);
    }

    #[test]
    fn a_long_step_is_cut_into_short_ones() {
        let m = LagModel::default();
        let mut one = m.at_rest(Pose::default(), 0.0);
        m.step(&mut one, Command::new(1.0, 0.2), 0.06);
        let mut three = m.at_rest(Pose::default(), 0.0);
        for _ in 0..3 {
            m.step(&mut three, Command::new(1.0, 0.2), 0.02);
        }
        assert_eq!(one, three);
    }

    #[test]
    fn observe_takes_what_was_measured() {
        let m = LagModel::default();
        let mut s = m.at_rest(Pose::default(), 0.0);
        m.observe(
            &mut s,
            &Observation {
                pose: Pose::new(1.0, 2.0, 0.3),
                speed: 1.5,
                steering: Some(0.2),
                throttle: Some(0.4),
                brake: None,
            },
        );
        assert_eq!(
            (s.pose.x, s.speed, s.throttle, s.brake),
            (1.0, 1.5, 0.4, 0.0)
        );
        assert!((m.steering(&s) - 0.2).abs() < 1e-12);
        assert!(m.lag_aware());
    }
}

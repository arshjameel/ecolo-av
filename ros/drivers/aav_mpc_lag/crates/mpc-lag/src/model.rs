//! The car with its lags, in the path's frame: the plan's model. A port of
//! `aav_track/lag_model.py`, step for step (`vectors.json`, from `gen_vectors.py`, holds
//! the two together).
//!
//! The numbers are the shared car model's ([`car_model::ActuatorConfig`], vehicle.yaml's
//! `actuators:`), and the speed loop's are spine-core's (its PI profiles). The maths is
//! the shared car model's again, in the path's frame (Frenet), because the plan linearises
//! it by finite differences 13 times a stage, 30 stages a tick:
//!
//! | Part | One step `h` |
//! |---|---|
//! | speed loop | the spine's PI (forward only), in f64 |
//! | throttle, brake | first-order lags, exact: `x += (command - x)(1 - exp(-h / lag))` |
//! | speed | `a = max_accel throttle (past its deadband) - max_brake brake - drag v`; never reversed |
//! | steering motor | to its setpoint at most `steer_rate h` a step; past `steer_deadband` only |
//! | wheels | lag the motor by `steer_lag`, + `steer_offset`, within full lock |
//! | the car | the kinematic bicycle about base_link, in the path's frame; the new speed and steering, the heading at mid-step |
//!
//! The dead times are the tracker's (whole ticks of the commands already sent).

use car_model::ActuatorConfig;
use spine_core::speed_loop::{AntiWindup, SpeedLaw, SpeedLoopConfig, SpeedLoopProfile};

/// How many numbers a state has.
pub const NX: usize = 10;
/// How many numbers an input has.
pub const NU: usize = 2;

/// The rows of a state.
pub mod row {
    /// Cross track, m (+ left of the path).
    pub const E_Y: usize = 0;
    /// Heading error, rad.
    pub const E_TH: usize = 1;
    /// Distance along the path, m.
    pub const S: usize = 2;
    /// Speed, m/s (forward only).
    pub const V: usize = 3;
    /// The throttle's real level, 0..1.
    pub const THROTTLE: usize = 4;
    /// The brake's real level, 0..1.
    pub const BRAKE: usize = 5;
    /// The steering motor's target, rad.
    pub const SETPOINT: usize = 6;
    /// The steering motor's angle, rad.
    pub const MOTOR: usize = 7;
    /// The wheels' angle after the linkage, before the offset, rad.
    pub const WHEELS: usize = 8;
    /// The speed loop's integral, throttle.
    pub const INTEGRAL: usize = 9;
    /// The steering target, rad (an input's row).
    pub const STEER: usize = 0;
    /// The set speed, m/s (an input's row).
    pub const SPEED: usize = 1;
}

/// A state: the rows of [`row`].
pub type State = [f64; NX];
/// An input: the steering target and the set speed.
pub type Input = [f64; NU];

/// Why a speed loop can't be modelled here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopError {
    /// The LQI isn't modelled yet: pick the default or smooth PI.
    Lqi,
}

impl std::fmt::Display for LoopError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("the speed loop lqi (the LQI) isn't modelled here yet: pick default or smooth")
    }
}

impl std::error::Error for LoopError {}

/// The spine's PI speed loop, its gains in f64 (spine-core keeps them in f32).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PiLoop {
    pub kp: f64,
    pub ki: f64,
    pub integral_limit: f64,
    pub setpoint_weight: f64,
    pub anti_windup: AntiWindup,
    pub brake_gain: f64,
    pub coast_band: f64,
    pub stopped_speed: f64,
    pub hold_brake: f64,
}

/// An f32 gain as the decimal it was written as (0.3f32 is 0.3, not 0.30000001192...).
fn widen(x: f32) -> f64 {
    x.to_string().parse().unwrap_or(f64::from(x))
}

impl PiLoop {
    /// A spine-core profile's PI; the LQI is refused.
    pub fn from_profile(profile: SpeedLoopProfile) -> Result<Self, LoopError> {
        Self::from_config(&profile.config())
    }

    /// A spine-core config's PI; the LQI is refused.
    pub fn from_config(c: &SpeedLoopConfig) -> Result<Self, LoopError> {
        if let SpeedLaw::Lqi(_) = c.law {
            return Err(LoopError::Lqi);
        }
        Ok(PiLoop {
            kp: widen(c.kp),
            ki: widen(c.ki),
            integral_limit: widen(c.integral_limit),
            setpoint_weight: widen(c.setpoint_weight),
            anti_windup: c.anti_windup,
            brake_gain: widen(c.brake_gain),
            coast_band: widen(c.coast_band),
            stopped_speed: widen(c.stopped_speed),
            hold_brake: widen(c.hold_brake),
        })
    }

    /// (throttle, brake, integral): the spine's PI, forward only (lag_model.pedals).
    pub fn pedals(&self, target: f64, measured: f64, integral: f64, dt: f64) -> (f64, f64, f64) {
        if target <= 0.0 {
            // A stop: brake until stopped, then hold (the spine's rule: never coast).
            let brake = if measured.abs() < self.stopped_speed {
                self.hold_brake
            } else {
                clip(
                    self.hold_brake
                        .max(self.brake_gain * (measured - self.coast_band)),
                    0.0,
                    1.0,
                )
            };
            return (0.0, brake, 0.0);
        }
        let error = target - measured;
        let prop = self.kp * (self.setpoint_weight * target - measured);
        let ceiling =
            (self.integral_limit + (1.0 - self.setpoint_weight) * self.kp * target).max(0.0);
        let grown = clip(integral + self.ki * error * dt, 0.0, ceiling);
        let integral = match self.anti_windup {
            AntiWindup::Bounds => grown,
            AntiWindup::Clamp => {
                let demand = prop + grown;
                let pinned = (demand > 1.0 && error > 0.0) || (demand < 0.0 && error < 0.0);
                if pinned {
                    integral
                } else {
                    grown
                }
            }
        };
        let overspeed = -error - self.coast_band;
        if overspeed > 0.0 {
            (0.0, clip(self.brake_gain * overspeed, 0.0, 1.0), 0.0)
        } else {
            (clip(prop + integral, 0.0, 1.0), 0.0, integral)
        }
    }
}

/// `min(max(x, lo), hi)`, as numpy's clip.
pub fn clip(x: f64, lo: f64, hi: f64) -> f64 {
    x.max(lo).min(hi)
}

/// The share of the way a first-order lag moves in `dt`: `1 - exp(-dt / tau)`.
pub fn lag_factor(dt: f64, tau: f64) -> f64 {
    if tau <= 0.0 {
        1.0
    } else {
        1.0 - (-dt / tau).exp()
    }
}

/// Whole ticks of a dead time: `floor(t / period + 0.5)`, as the shared car model rounds.
pub fn dead_ticks(dead_time: f64, period: f64) -> usize {
    (dead_time / period + 0.5).floor().max(0.0) as usize
}

/// The numbers with every lag and dead time `k` times longer.
pub fn lags_scaled(car: &ActuatorConfig, k: f64) -> ActuatorConfig {
    ActuatorConfig {
        throttle_lag: k * car.throttle_lag,
        brake_lag: k * car.brake_lag,
        steer_lag: k * car.steer_lag,
        throttle_dead_time: k * car.throttle_dead_time,
        brake_dead_time: k * car.brake_dead_time,
        steer_dead_time: k * car.steer_dead_time,
        ..*car
    }
}

/// The numbers with no deadbands: for a plan, whose gradients they'd flatten.
pub fn without_deadbands(car: &ActuatorConfig) -> ActuatorConfig {
    ActuatorConfig {
        throttle_deadband: 0.0,
        steer_deadband: 0.0,
        ..*car
    }
}

/// The numbers with no dead times: the tracker keeps those itself, a tick at a time.
pub fn without_dead_times(car: &ActuatorConfig) -> ActuatorConfig {
    ActuatorConfig {
        throttle_dead_time: 0.0,
        brake_dead_time: 0.0,
        steer_dead_time: 0.0,
        ..*car
    }
}

/// The state `dt` later, in `substeps` equal steps. `kappa`: the path's curvature here
/// (1/m, + left), held over the step.
pub fn step(
    car: &ActuatorConfig,
    lp: &PiLoop,
    x: &State,
    u: &Input,
    kappa: f64,
    dt: f64,
    substeps: usize,
) -> State {
    use row::*;
    let substeps = substeps.max(1);
    let h = dt / substeps as f64;
    let [mut e_y, mut e_th, mut s, mut v, mut thr, mut brk, mut setpoint, mut motor, mut wheels, mut integral] =
        *x;
    let (kt, kb, ks) = (
        lag_factor(h, car.throttle_lag),
        lag_factor(h, car.brake_lag),
        lag_factor(h, car.steer_lag),
    );
    let target = clip(u[STEER], -car.max_steering, car.max_steering);
    let reach = car.steer_rate * h;
    for _ in 0..substeps {
        // The speed loop, then throttle and brake through their lags.
        let (thr_cmd, brk_cmd, next_integral) = lp.pedals(u[SPEED], v, integral, h);
        integral = next_integral;
        thr += kt * (thr_cmd - thr);
        brk += kb * (brk_cmd - brk);
        let pushing = if thr > car.throttle_deadband {
            thr
        } else {
            0.0
        };
        let push = pushing * car.max_accel;
        let moved = v + push * h;
        let v_new = (moved - (brk * car.max_brake + car.drag * moved.abs()) * h).max(0.0);
        // Steering: the deadband, the motor's rate, the linkage's lag, the offset.
        if (target - setpoint).abs() > car.steer_deadband {
            setpoint = target;
        }
        motor += clip(setpoint - motor, -reach, reach);
        wheels += ks * (motor - wheels);
        let delta = clip(
            wheels + car.steer_offset,
            -car.max_steering,
            car.max_steering,
        );
        // The bicycle, in the path's frame: the new speed and steering, mid-step heading.
        let along_scale = 1.0 - kappa * e_y;
        let turn =
            (v_new * delta.tan() / car.wheelbase - kappa * v_new * e_th.cos() / along_scale) * h;
        let mid = e_th + 0.5 * turn;
        let along = v_new * mid.cos() / along_scale;
        e_y += v_new * mid.sin() * h;
        e_th += turn;
        s += along * h;
        v = v_new;
    }
    [e_y, e_th, s, v, thr, brk, setpoint, motor, wheels, integral]
}

#[cfg(test)]
mod tests {
    use super::*;
    use row::*;

    fn default_loop() -> PiLoop {
        PiLoop::from_profile(SpeedLoopProfile::Default).unwrap()
    }

    #[test]
    fn the_gains_are_the_decimals_written() {
        let lp = default_loop();
        assert_eq!((lp.kp, lp.ki, lp.integral_limit), (0.3, 0.1, 0.3));
        assert_eq!(
            PiLoop::from_profile(SpeedLoopProfile::Lqi),
            Err(LoopError::Lqi)
        );
    }

    #[test]
    fn the_wheels_follow_the_motor_at_its_rate() {
        let car = ActuatorConfig::default();
        let x = step(&car, &default_loop(), &[0.0; NX], &[0.5, 0.0], 0.0, 0.1, 5);
        assert!((x[MOTOR] - car.steer_rate * 0.1).abs() < 1e-12);
        assert!(x[WHEELS] > 0.0 && x[WHEELS] < x[MOTOR]);
    }

    #[test]
    fn the_brake_never_reverses_the_car() {
        let car = ActuatorConfig::default();
        let mut x = [0.0; NX];
        x[V] = 0.2;
        x[BRAKE] = 1.0;
        for _ in 0..20 {
            x = step(&car, &default_loop(), &x, &[0.0, 0.0], 0.0, 0.05, 2);
        }
        assert_eq!(x[V], 0.0);
    }

    #[test]
    fn on_a_bend_driven_straight_the_car_drifts_outward() {
        let car = ActuatorConfig::default();
        let mut x = [0.0; NX];
        x[V] = 1.0;
        let lp = default_loop();
        for _ in 0..20 {
            x = step(&car, &lp, &x, &[0.0, 1.0], 0.2, 0.05, 2);
        }
        // The path bends left; a car going straight ends up right of it, heading right.
        assert!(x[E_Y] < 0.0 && x[E_TH] < 0.0 && x[S] > 0.9);
    }

    #[test]
    fn dead_ticks_round_half_up() {
        assert_eq!(dead_ticks(0.2, 0.05), 4);
        assert_eq!(dead_ticks(0.125, 0.05), 3);
        assert_eq!(dead_ticks(0.0, 0.05), 0);
    }
}

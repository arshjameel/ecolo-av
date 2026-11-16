//! The maths behind ego: a Twist (v, w) becomes an Ackermann (speed, steering)
//! (`ackermann.py`).
//!
//! The bicycle model about the rear axle (`base_link`): `d = atan(L w / v)`, clamped to
//! `max_steering_angle`. Below `min_speed_for_steering` the wheels hold. With a [`Body`],
//! the loaded car never speeds up harder than its motor can and slows for a turn. Speeding
//! up is jerk-limited; slowing down never is. While a person drives, the [`Pedal`] grades
//! how hard a slow-down is.

use crate::clamp;
use crate::dynamics::Body;
use crate::lag_lead::LagLead;
use crate::pedal::Pedal;

/// rad/s: the steering rate a running ego accepts (`ros2 param set /ego max_steering_rate`).
pub const STEERING_RATE_RANGE: (f64, f64) = (0.05, 5.0);

/// ego's limits: ego.yaml and vehicle.yaml.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// m, rear axle to front axle.
    pub wheelbase: f64,
    /// rad, either side.
    pub max_steering_angle: f64,
    /// m/s forward.
    pub max_speed: f64,
    /// m/s backwards.
    pub max_reverse_speed: f64,
    /// m/s² while speeding up.
    pub max_acceleration: f64,
    /// m/s² while slowing down.
    pub max_deceleration: f64,
    /// rad/s.
    pub max_steering_rate: f64,
    /// m/s; below this the steering holds.
    pub min_speed_for_steering: f64,
    /// m/s³ while speeding up; 0: no limit.
    pub max_jerk: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            wheelbase: 1.57,
            max_steering_angle: 0.5,
            max_speed: 1.0,
            max_reverse_speed: 0.5,
            max_acceleration: 1.0,
            max_deceleration: 2.0,
            max_steering_rate: 0.6,
            min_speed_for_steering: 0.05,
            max_jerk: 0.0,
        }
    }
}

/// The front-wheel angle for (v, w), or `hold` if the car is too slow to tell.
pub fn steering_angle(v: f64, w: f64, lim: &Limits, hold: f64) -> f64 {
    if v.abs() < lim.min_speed_for_steering {
        return hold;
    }
    let d = (lim.wheelbase * w / v).atan();
    clamp(d, -lim.max_steering_angle, lim.max_steering_angle)
}

/// The yaw rate of the bicycle model, the inverse of [`steering_angle`].
pub fn turn_rate(v: f64, d: f64, wheelbase: f64) -> f64 {
    v * d.tan() / wheelbase
}

/// How sharply the path bends (1/m) for a front-wheel angle.
pub fn curvature(steering: f64, wheelbase: f64) -> f64 {
    steering.tan() / wheelbase
}

/// Move the speed toward `target` within the limits; return (speed, rate used).
///
/// `speed_up` replaces `max_acceleration` (the body's lower limit); `slow_down` replaces
/// `max_deceleration`, never above it (the pedal's).
pub fn slew_speed(
    current: f64,
    target: f64,
    lim: &Limits,
    dt: f64,
    speed_up: Option<f64>,
    slow_down: Option<f64>,
) -> (f64, f64) {
    let mut target = target;
    if current != 0.0 && target != 0.0 && (current > 0.0) != (target > 0.0) {
        target = 0.0; // change direction through a stop
    }
    let speeding_up = target.abs() > current.abs();
    let up = speed_up.unwrap_or(lim.max_acceleration);
    let down = slow_down.map_or(lim.max_deceleration, |s| s.min(lim.max_deceleration));
    let rate = if speeding_up { up } else { down };
    let step = rate * dt;
    if target > current {
        (target.min(current + step), rate)
    } else {
        (target.max(current - step), rate)
    }
}

/// Speed up toward `target`, the acceleration changing at most `jerk` m/s³; return
/// (speed, signed acceleration). `accel` is the last tick's signed acceleration.
pub fn ease_speed(
    current: f64,
    target: f64,
    accel: f64,
    rate: f64,
    jerk: f64,
    dt: f64,
) -> (f64, f64) {
    let gap = target - current;
    if gap == 0.0 {
        return (current, 0.0);
    }
    let sign = if gap > 0.0 { 1.0 } else { -1.0 };
    let held = 0.0_f64.max(accel * sign);
    let half = dt / 2.0;
    let ease_off = jerk * ((half * half + 2.0 * gap.abs() / jerk).sqrt() - half);
    let a = (held + jerk * dt)
        .min(rate)
        .min(ease_off.max(held - jerk * dt));
    let step = (a * dt).min(gap.abs());
    (current + sign * step, sign * a)
}

/// The car as ego sees it: the speed and steering it last commanded.
#[derive(Debug, Clone, PartialEq)]
pub struct EgoModel {
    pub lim: Limits,
    pub body: Option<Body>,
    pub speed: f64,
    pub steering: f64,
    /// m/s², signed: the last tick's change of speed.
    pub accel: f64,
    /// The pedal while a person drives: the node sets it each tick.
    pub pedal: Option<Pedal>,
}

/// One tick's output.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Output {
    /// m/s, + forward.
    pub speed: f64,
    /// rad, + left.
    pub steering: f64,
    /// m/s²: the limit used (the eased acceleration while speeding up with a jerk limit).
    pub accel: f64,
}

impl EgoModel {
    pub fn new(lim: Limits, body: Option<Body>) -> Self {
        EgoModel {
            lim,
            body,
            speed: 0.0,
            steering: 0.0,
            accel: 0.0,
            pedal: None,
        }
    }

    /// Advance one tick. `cmd` is (v, w), or `None` when missing or stale: stop. `lead`,
    /// while it runs, changes the request before every limit, never a stop.
    pub fn step(&mut self, cmd: Option<(f64, f64)>, dt: f64, lead: Option<&mut LagLead>) -> Output {
        let lim = self.lim;
        let (mut target_v, target_d) = match cmd {
            None => (0.0, self.steering),
            Some((v, w)) => {
                let mut target_d = steering_angle(v, w, &lim, self.steering);
                let mut v = v;
                if let Some(lead) = lead {
                    (v, target_d) = lead.step(v, target_d, dt);
                    target_d = clamp(target_d, -lim.max_steering_angle, lim.max_steering_angle);
                }
                (clamp(v, -lim.max_reverse_speed, lim.max_speed), target_d)
            }
        };
        let asked = target_v;
        let mut speed_up = None;
        if let Some(body) = &self.body {
            speed_up = Some(lim.max_acceleration.min(body.speed_up_limit()));
            // The sharper of the turn now and the one the wheels are heading for.
            let k = curvature(target_d, lim.wheelbase)
                .abs()
                .max(curvature(self.steering, lim.wheelbase).abs());
            let cap = body.curve_speed(k);
            target_v = clamp(target_v, -cap, cap);
        }
        // The pedal grades a person's slowing; never a stop or a turn's cap.
        let slow_down = match (self.pedal, cmd) {
            (Some(pedal), Some((v, _))) if target_v == asked => {
                Some(pedal.slow_rate(self.speed, v, lim.max_deceleration))
            }
            _ => None,
        };
        let before = self.speed;
        let (speed, mut accel) = slew_speed(self.speed, target_v, &lim, dt, speed_up, slow_down);
        self.speed = speed;
        if lim.max_jerk > 0.0 && self.speed.abs() > before.abs() {
            // Speeding up, so toward target_v (a change of direction stops first).
            let (speed, signed) = ease_speed(before, target_v, self.accel, accel, lim.max_jerk, dt);
            self.speed = speed;
            accel = signed.abs();
        }
        self.accel = if dt > 0.0 {
            (self.speed - before) / dt
        } else {
            0.0
        };
        let step = lim.max_steering_rate * dt;
        self.steering = clamp(target_d, self.steering - step, self.steering + step);
        Output {
            speed: self.speed,
            steering: self.steering,
            accel,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steering_is_the_bicycle_and_holds_when_slow() {
        let lim = Limits::default();
        let d = steering_angle(1.0, 0.2, &lim, 0.0);
        assert!((turn_rate(1.0, d, lim.wheelbase) - 0.2).abs() < 1e-12);
        assert_eq!(steering_angle(0.01, 0.2, &lim, 0.3), 0.3);
        assert_eq!(steering_angle(1.0, 5.0, &lim, 0.0), 0.5);
    }

    #[test]
    fn a_change_of_direction_passes_through_a_stop() {
        let lim = Limits::default();
        let (v, _) = slew_speed(0.1, -0.5, &lim, 1.0, None, None);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn a_stale_command_stops_at_full_deceleration_and_holds_the_wheels() {
        let mut m = EgoModel::new(Limits::default(), None);
        m.speed = 1.0;
        m.steering = 0.2;
        let out = m.step(None, 0.1, None);
        assert!((out.speed - 0.8).abs() < 1e-12);
        assert_eq!(out.steering, 0.2);
        assert_eq!(out.accel, 2.0);
    }

    #[test]
    fn jerk_eases_the_start() {
        let lim = Limits {
            max_jerk: 2.0,
            ..Limits::default()
        };
        let mut m = EgoModel::new(lim, Some(Body::default()));
        let first = m.step(Some((1.0, 0.0)), 0.02, None);
        assert!(first.accel < 0.1, "{first:?}");
    }
}

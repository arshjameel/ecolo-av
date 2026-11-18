//! The speed loop (`speed_loop.mode: pi`): feed-forward plus PI, one signed acceleration,
//! out as throttle OR brake (`speed_control.py`).
//!
//! ```text
//!   ego's speed ──> wanted acceleration (its slope) ──────────── feed-forward
//!               └─> speed error ──> PI ───────────────────────── feedback
//!                                    │
//!                   one signed acceleration ──> the car model, inverted ──> throttle OR brake
//! ```
//!
//! The PI's reference is ego's speed delayed by `response_lag`. The smooth stop follows
//! the design of Autoware's `pid_longitudinal_controller` (Apache-2.0; our own code). The
//! numbers and every rule are the Python's: `tests/vectors.rs` checks them tick by tick.

use crate::{py_max, py_min};

/// The loop's gains, and the car model it inverts (`SpeedControlConfig`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedConfig {
    /// m/s² per m/s of speed error.
    pub kp: f64,
    /// m/s² per (m/s × s).
    pub ki: f64,
    /// m/s²: the most the integral may add or take.
    pub integral_limit: f64,
    /// m/s: integrate only this close to the reference.
    pub integral_band: f64,
    /// s: smooths the slope of ego's speed.
    pub accel_filter: f64,
    /// s: how far behind ego the car is expected to run.
    pub response_lag: f64,
    /// m/s² either side of zero: coast, neither pedal.
    pub dead_band: f64,
    /// m/s: slower counts as stopped.
    pub stopped_speed: f64,
    /// Brake (0..1) held while stopped with a zero target.
    pub hold_brake: f64,
    /// m/s: ego and the car this slow, the slowing fades (0: no smooth stop).
    pub smooth_stop_speed: f64,
    /// 1/s: slowing at most this × the speed near the stop.
    pub smooth_stop_rate: f64,
    /// m/s²: slowing at least this much, so it never creeps.
    pub weak_stop_decel: f64,
    /// s: ego at 0 and still rolling this long...
    pub weak_stop_time: f64,
    /// m/s²: ...then at least this much.
    pub strong_stop_decel: f64,
    /// Brake (0..1) held once stopped, until ego asks again.
    pub stopped_brake: f64,
    /// Brake (0..1) per s: how fast it rises to `stopped_brake`.
    pub stopped_brake_rate: f64,
    /// m/s² at full throttle (vehicle.yaml's `actuators:`).
    pub max_accel: f64,
    /// m/s² at full brake.
    pub max_brake: f64,
    /// 1/s: m/s lost per second per m/s, coasting.
    pub drag: f64,
}

impl Default for SpeedConfig {
    fn default() -> Self {
        SpeedConfig {
            kp: 2.0,
            ki: 0.5,
            integral_limit: 0.5,
            integral_band: 0.3,
            accel_filter: 0.1,
            response_lag: 0.3,
            dead_band: 0.05,
            stopped_speed: 0.05,
            hold_brake: 0.2,
            smooth_stop_speed: 0.3,
            smooth_stop_rate: 1.5,
            weak_stop_decel: 0.2,
            weak_stop_time: 2.0,
            strong_stop_decel: 1.0,
            stopped_brake: 0.5,
            stopped_brake_rate: 1.0,
            max_accel: 1.5,
            max_brake: 3.0,
            drag: 0.1,
        }
    }
}

impl SpeedConfig {
    /// Every field's parameter name (after `speed_loop.`) and default, in the Python's order.
    #[must_use]
    pub fn fields() -> [(&'static str, f64); 19] {
        let d = SpeedConfig::default();
        [
            ("kp", d.kp),
            ("ki", d.ki),
            ("integral_limit", d.integral_limit),
            ("integral_band", d.integral_band),
            ("accel_filter", d.accel_filter),
            ("response_lag", d.response_lag),
            ("dead_band", d.dead_band),
            ("stopped_speed", d.stopped_speed),
            ("hold_brake", d.hold_brake),
            ("smooth_stop_speed", d.smooth_stop_speed),
            ("smooth_stop_rate", d.smooth_stop_rate),
            ("weak_stop_decel", d.weak_stop_decel),
            ("weak_stop_time", d.weak_stop_time),
            ("strong_stop_decel", d.strong_stop_decel),
            ("stopped_brake", d.stopped_brake),
            ("stopped_brake_rate", d.stopped_brake_rate),
            ("max_accel", d.max_accel),
            ("max_brake", d.max_brake),
            ("drag", d.drag),
        ]
    }

    /// Sets one field by its parameter name (after `speed_loop.`).
    ///
    /// # Errors
    ///
    /// [`UnknownField`] for a name that isn't one of [`SpeedConfig::fields`].
    pub fn set(&mut self, name: &str, value: f64) -> Result<(), UnknownField> {
        let field = match name {
            "kp" => &mut self.kp,
            "ki" => &mut self.ki,
            "integral_limit" => &mut self.integral_limit,
            "integral_band" => &mut self.integral_band,
            "accel_filter" => &mut self.accel_filter,
            "response_lag" => &mut self.response_lag,
            "dead_band" => &mut self.dead_band,
            "stopped_speed" => &mut self.stopped_speed,
            "hold_brake" => &mut self.hold_brake,
            "smooth_stop_speed" => &mut self.smooth_stop_speed,
            "smooth_stop_rate" => &mut self.smooth_stop_rate,
            "weak_stop_decel" => &mut self.weak_stop_decel,
            "weak_stop_time" => &mut self.weak_stop_time,
            "strong_stop_decel" => &mut self.strong_stop_decel,
            "stopped_brake" => &mut self.stopped_brake,
            "stopped_brake_rate" => &mut self.stopped_brake_rate,
            "max_accel" => &mut self.max_accel,
            "max_brake" => &mut self.max_brake,
            "drag" => &mut self.drag,
            _ => return Err(UnknownField),
        };
        *field = value;
        Ok(())
    }
}

/// A [`SpeedConfig`] field name that doesn't exist.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnknownField;

impl core::fmt::Display for UnknownField {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("not a speed_loop setting")
    }
}

impl std::error::Error for UnknownField {}

/// A time step that isn't positive: the loop can't take one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BadStep(pub f64);

impl core::fmt::Display for BadStep {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "dt must be positive, not {}", self.0)
    }
}

impl std::error::Error for BadStep {}

/// One tick's output: throttle and brake, 0..1, never both above 0.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pedals {
    /// 0..1.
    pub throttle: f64,
    /// 0..1.
    pub brake: f64,
    /// m/s²: the signed acceleration asked of the car.
    pub accel: f64,
    /// m/s²: of which, from ego's slope.
    pub feedforward: f64,
    /// m/s²: of which, the integral.
    pub integral: f64,
}

/// Python's `min(high, max(low, x))`.
fn clamp(x: f64, low: f64, high: f64) -> f64 {
    py_min(high, py_max(low, x))
}

/// Feed-forward plus PI on the wheel speed, out as throttle or brake (`SpeedControl`).
#[derive(Debug, Clone)]
pub struct SpeedControl {
    /// The gains and the model.
    pub cfg: SpeedConfig,
    integral: f64,
    feedforward: f64,
    last_target: Option<f64>,
    expected: Option<f64>,
    standstill: bool,
    rolling: f64,
    stop_decel: Option<f64>,
    last_accel: f64,
    last_brake: f64,
}

impl SpeedControl {
    /// A loop with no past.
    #[must_use]
    pub fn new(cfg: SpeedConfig) -> Self {
        SpeedControl {
            cfg,
            integral: 0.0,
            feedforward: 0.0,
            last_target: None,
            expected: None,
            standstill: false,
            rolling: 0.0,
            stop_decel: None,
            last_accel: 0.0,
            last_brake: 0.0,
        }
    }

    /// Forgets the past: on a stale command, an e-stop, or a new drive.
    pub fn reset(&mut self) {
        *self = SpeedControl::new(self.cfg);
    }

    /// This tick's pedals: `target` is ego's speed (m/s), `measured` the wheel speed (None
    /// if none is fresh), `dt` the time since the last update (s).
    ///
    /// # Errors
    ///
    /// [`BadStep`] if `dt` isn't positive.
    pub fn update(
        &mut self,
        target: f64,
        measured: Option<f64>,
        dt: f64,
    ) -> Result<Pedals, BadStep> {
        let c = self.cfg;
        if dt <= 0.0 || dt.is_nan() {
            return Err(BadStep(dt));
        }
        let target = py_max(0.0, target);
        let slope = self.last_target.map_or(0.0, |last| (target - last) / dt);
        self.last_target = Some(target);
        self.feedforward +=
            (slope - self.feedforward) * clamp(dt / py_max(c.accel_filter, dt), 0.0, 1.0);

        // The speed the car should have now, given the actuators' lag: the PI's reference.
        let expected = match self.expected {
            None => target,
            Some(e) => e + (target - e) * clamp(dt / py_max(c.response_lag, dt), 0.0, 1.0),
        };
        self.expected = Some(expected);

        let speed = match measured {
            None => {
                // Open loop: assume the car is where ego says, and trust the model alone.
                self.integral = 0.0;
                target
            }
            Some(m) => py_max(0.0, m),
        };

        let stopping = target < c.stopped_speed;
        let smooth = c.smooth_stop_speed > 0.0 && measured.is_some();
        if !stopping {
            self.standstill = false;
            self.rolling = 0.0;
            self.stop_decel = None;
        }
        if stopping && smooth {
            self.integral = 0.0;
            self.feedforward = 0.0;
            let p = self.smooth_stop(speed, dt);
            return Ok(self.remember(p));
        }
        if stopping {
            self.integral = 0.0;
            self.feedforward = 0.0;
            if speed < c.stopped_speed {
                return Ok(self.remember(Pedals {
                    brake: c.hold_brake,
                    ..Pedals::default()
                }));
            }
        }

        let error = if measured.is_some() {
            expected - speed
        } else {
            0.0
        };
        let mut integral = self.integral;
        if error.abs() < c.integral_band && !stopping {
            integral = clamp(
                integral + c.ki * error * dt,
                -c.integral_limit,
                c.integral_limit,
            );
        }
        let mut accel = self.feedforward + c.kp * error + integral;
        if smooth && target < c.smooth_stop_speed && speed < c.smooth_stop_speed {
            // Nearly stopped, ego nearly at 0: slowing fades with the speed already.
            accel = py_max(
                accel,
                -py_max(c.weak_stop_decel, c.smooth_stop_rate * speed),
            );
        }
        let (mut throttle, mut brake) = self.pedals(accel, speed);
        if stopping {
            throttle = 0.0;
            brake = py_max(brake, c.hold_brake);
        }
        // Anti-windup: keep the new integral unless a pedal is pinned and it would grow more.
        let pinned = (throttle >= 1.0 && error > 0.0) || (brake >= 1.0 && error < 0.0);
        if !pinned {
            self.integral = integral;
        }
        let p = Pedals {
            throttle,
            brake,
            accel,
            feedforward: self.feedforward,
            integral: self.integral,
        };
        Ok(self.remember(p))
    }

    /// Keeps this tick's asked acceleration and brake: the smooth stop starts from them.
    fn remember(&mut self, p: Pedals) -> Pedals {
        self.last_accel = p.accel;
        self.last_brake = p.brake;
        p
    }

    /// The pedals once ego says 0 and the car still rolls: the smooth stop.
    fn smooth_stop(&mut self, speed: f64, dt: f64) -> Pedals {
        let c = self.cfg;
        if self.standstill || speed < c.stopped_speed {
            self.standstill = true;
            // Up to stopped_brake gently (a step would jolt the car); never less.
            let brake = py_max(
                self.last_brake,
                py_min(c.stopped_brake, self.last_brake + c.stopped_brake_rate * dt),
            );
            return Pedals {
                brake,
                ..Pedals::default()
            };
        }
        let stop_decel = *self
            .stop_decel
            .get_or_insert(py_max(c.weak_stop_decel, -self.last_accel));
        self.rolling += dt;
        let mut weakest = c.weak_stop_decel;
        if self.rolling > c.weak_stop_time {
            weakest = py_max(weakest, c.strong_stop_decel);
        }
        let decel = clamp(
            c.smooth_stop_rate * speed,
            weakest,
            py_max(stop_decel, weakest),
        );
        let (_, brake) = self.pedals(-decel, speed);
        Pedals {
            brake,
            accel: -decel,
            ..Pedals::default()
        }
    }

    /// Inverts the model: the throttle or brake that gives `accel` at `speed`.
    fn pedals(&self, accel: f64, speed: f64) -> (f64, f64) {
        let c = self.cfg;
        // Coasting already slows the car by drag × speed: the pedals make up the rest.
        let push = accel + c.drag * speed;
        if push > c.dead_band {
            (clamp(push / c.max_accel, 0.0, 1.0), 0.0)
        } else if push < -c.dead_band {
            (0.0, clamp(-push / c.max_brake, 0.0, 1.0))
        } else {
            (0.0, 0.0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_step_must_be_positive() {
        let mut s = SpeedControl::new(SpeedConfig::default());
        assert_eq!(s.update(1.0, None, 0.0), Err(BadStep(0.0)));
    }

    #[test]
    fn every_field_has_a_name() {
        let mut c = SpeedConfig::default();
        for (name, _) in SpeedConfig::fields() {
            assert!(c.set(name, 9.0).is_ok(), "{name}");
        }
        assert_eq!(c.drag, 9.0);
        assert_eq!(c.set("nope", 1.0), Err(UnknownField));
    }
}

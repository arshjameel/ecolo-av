//! The adaptive brake: brake harder when the car slows less than the car model says
//! (TODO 258). A port of `aav_description`'s `adaptive_brake.py`, the sim's copy; both
//! check `firmware/spine/adaptive_brake_vectors.txt`.
//!
//! Ben, 8 Nov: "if we brake without a person vs with a person ... we are going to need to
//! dynamically realize that we aren't braking as fast as we expect and start applying more
//! force. We don't want to overwhelm the brakes though."
//!
//! Where it sits: after the speed loop, before the brake actuator. Every safety stop goes
//! round it (an e-stop, a stale command, a silent link: the full stop brake, as before).
//!
//! ```text
//!  ego's speed ─▶ speed loop ─▶ brake asked ─▶ AdaptiveBrake ─▶ brake sent ─▶ brake actuator
//!                     ▲                           ▲      │
//!               wheel speed ──────────────────────┘      └─ never above the safe max
//! ```
//!
//! One tick of `dt`:
//!
//! | Step | What |
//! |---|---|
//! | 1. measured | the slowing from the wheel speed, smoothed (`filter_time`), less the drag |
//! | 2. expected | the model's brake level for what was sent (dead time, lag) x `max_brake` |
//! | 3. learn | effectiveness follows measured / expected (`adapt_time`), only while braking for real |
//! | 4. gain | 1 / effectiveness, from 1 (never weaker than asked) to `max_gain` |
//! | 5. extra | asked x (gain - 1), rising at most `rise_rate` a second, falling at once |
//! | 6. cap | asked + extra, never above the safe max; the asked brake itself passes |
//! | 7. time | near the safe max with extra on for `max_hold_time` s: the extra is off until the brake is let go |
//!
//! Units: m/s, m/s², s; brake 0 (released) to 1 (full). `f64`, as the Python, so the two
//! agree to the last few bits. `no_std`, no allocator: the spine could run it too.

#![no_std]

use core::fmt;

/// The longest brake dead time, in ticks, the delay line holds (5 s at 50 Hz).
pub const MAX_DEAD_STEPS: usize = 255;
const LINE: usize = MAX_DEAD_STEPS + 1;

/// Python's `min(a, b)`: `a` unless `b` is smaller (so a NaN `b` gives `a`).
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's `max(a, b)`: `a` unless `b` is larger.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// `car_model.clamp`: `min(high, max(low, x))`, NaN giving `low`.
fn clamp(x: f64, low: f64, high: f64) -> f64 {
    py_min(high, py_max(low, x))
}

/// First-order lag: move `value` toward `target` with time constant `tau` (0: at once).
fn lag(value: f64, target: f64, tau: f64, dt: f64) -> f64 {
    if tau <= 0.0 {
        return target;
    }
    value + (target - value) * (1.0 - libm::exp(-dt / tau))
}

/// How many ticks of `dt` a dead time waits: `floor(dead_time / dt + 0.5)`, at least 0.
fn delay_steps(dead_time: f64, dt: f64) -> f64 {
    py_max(0.0, libm::floor(dead_time / dt + 0.5))
}

/// Why the brake's limits, or a tick, can't be used.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Error {
    /// `full` and `released` are the same target: no travel to map 0..1 onto.
    NoTravel,
    /// `touch` or `safe_max` (named) is not between `released` and `full`.
    OutsideTravel(&'static str, f64),
    /// `max_hold_time` is negative.
    NegativeHoldTime(f64),
    /// A tick's `dt` is not positive.
    BadStep(f64),
    /// The brake's dead time is longer than [`MAX_DEAD_STEPS`] ticks of this `dt`.
    DeadTimeTooLong(f64),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::NoTravel => f.write_str("brake: full and released are the same target"),
            Error::OutsideTravel(name, v) => {
                write!(f, "brake: {name} ({v}) is not between released and full")
            }
            Error::NegativeHoldTime(v) => write!(f, "brake: max_hold_time ({v}) is negative"),
            Error::BadStep(dt) => write!(f, "dt must be positive, not {dt}"),
            Error::DeadTimeTooLong(dt) => write!(
                f,
                "the brake's dead time is over {MAX_DEAD_STEPS} ticks of {dt} s"
            ),
        }
    }
}

impl core::error::Error for Error {}

/// `vehicle.yaml`'s `brake:` section: the brake actuator's travel, in its targets (counts).
/// The firmware maps 0..1 onto `released..full`. PLACEHOLDERS until car day A7.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakeLimits {
    released: f64,
    touch: f64,
    full: f64,
    safe_max: f64,
    max_hold_time: f64,
}

impl Default for BrakeLimits {
    /// `adaptive_brake.py`'s `BrakeLimits()`: 400, 800, 3500, 3200 and 5 s.
    fn default() -> Self {
        BrakeLimits {
            released: 400.0,
            touch: 800.0,
            full: 3500.0,
            safe_max: 3200.0,
            max_hold_time: 5.0,
        }
    }
}

impl BrakeLimits {
    /// The limits, checked: `touch` and `safe_max` inside the travel, the hold time not
    /// negative.
    ///
    /// # Errors
    ///
    /// [`Error::NoTravel`], [`Error::OutsideTravel`] or [`Error::NegativeHoldTime`].
    pub fn new(
        released: f64,
        touch: f64,
        full: f64,
        safe_max: f64,
        max_hold_time: f64,
    ) -> Result<Self, Error> {
        if full == released {
            return Err(Error::NoTravel);
        }
        let limits = BrakeLimits {
            released,
            touch,
            full,
            safe_max,
            max_hold_time,
        };
        for (name, value) in [("touch", touch), ("safe_max", safe_max)] {
            if !(0.0..=1.0).contains(&limits.raw_fraction(value)) {
                return Err(Error::OutsideTravel(name, value));
            }
        }
        if max_hold_time < 0.0 || max_hold_time.is_nan() {
            return Err(Error::NegativeHoldTime(max_hold_time));
        }
        Ok(limits)
    }

    fn raw_fraction(&self, target: f64) -> f64 {
        (target - self.released) / (self.full - self.released)
    }

    /// A target as the brake command (0 released, 1 full), as the firmware maps it.
    #[must_use]
    pub fn fraction(&self, target: f64) -> f64 {
        clamp(self.raw_fraction(target), 0.0, 1.0)
    }

    /// Where the pads start to grip, 0..1.
    #[must_use]
    pub fn touch_fraction(&self) -> f64 {
        self.fraction(self.touch)
    }

    /// The most the brake may be pushed, 0..1.
    #[must_use]
    pub fn safe_max_fraction(&self) -> f64 {
        self.fraction(self.safe_max)
    }

    /// Seconds near the safe max before the extra comes off.
    #[must_use]
    pub fn max_hold_time(&self) -> f64 {
        self.max_hold_time
    }
}

/// The empty car's brake, from `vehicle.yaml`'s `actuators:` (`car_model.ActuatorConfig`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarModel {
    /// m/s² at full brake.
    pub max_brake: f64,
    /// s: the brake actuator's first-order response.
    pub brake_lag: f64,
    /// s before the brake starts to answer.
    pub brake_dead_time: f64,
    /// 1/s: coasting, m/s lost per second per m/s.
    pub drag: f64,
    /// s before the throttle starts to answer.
    pub throttle_dead_time: f64,
    /// s: the throttle's first-order response.
    pub throttle_lag: f64,
}

impl Default for CarModel {
    /// `ActuatorConfig()`'s: 3.0 m/s², 0.2 s, 0 s, 0.1 /s, 0 s, 0.1 s.
    fn default() -> Self {
        CarModel {
            max_brake: 3.0,
            brake_lag: 0.2,
            brake_dead_time: 0.0,
            drag: 0.1,
            throttle_dead_time: 0.0,
            throttle_lag: 0.1,
        }
    }
}

/// How quickly it learns and how hard it may push. Guesses: tune in the sim, then 4.3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// s: smooths both slowings (the wheel speed is coarse).
    pub filter_time: f64,
    /// s: how fast the effectiveness follows what it sees.
    pub adapt_time: f64,
    /// m/s²: expected braking below this teaches nothing.
    pub min_decel: f64,
    /// m/s: slower, the slowing can't be read.
    pub min_speed: f64,
    /// The most it multiplies the asked brake.
    pub max_gain: f64,
    /// Brake (0..1) a second: the extra rises no faster.
    pub rise_rate: f64,
    /// Of the safe max: "high force", timed by `max_hold_time`.
    pub high_share: f64,
}

impl Default for Tuning {
    /// `AdaptiveBrakeConfig()`'s.
    fn default() -> Self {
        Tuning {
            filter_time: 0.15,
            adapt_time: 0.3,
            min_decel: 0.3,
            min_speed: 0.2,
            max_gain: 3.0,
            rise_rate: 2.0,
            high_share: 0.9,
        }
    }
}

/// One tick's answer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakeOut {
    /// 0..1: what to send the brake actuator.
    pub brake: f64,
    /// 0..1: what the speed loop asked.
    pub asked: f64,
    /// What the asked brake is multiplied by (1: as asked).
    pub gain: f64,
    /// Measured / expected braking (1: the empty car).
    pub effectiveness: f64,
    /// Held at the safe max.
    pub capped: bool,
    /// The high-force time ran out this stop: no extra.
    pub spent: bool,
}

/// The model's brake dead time: a fixed ring of what was sent, oldest first.
#[derive(Debug, Clone)]
struct DelayLine {
    values: [f64; LINE],
    start: usize,
    len: usize,
}

impl DelayLine {
    const fn new() -> Self {
        DelayLine {
            values: [0.0; LINE],
            start: 0,
            len: 0,
        }
    }

    /// Push `value`; return what was pushed `steps` pushes ago, or `rest` until the line
    /// is that long (`car_model.delay`). A shorter `steps` drops the oldest.
    fn delay(&mut self, value: f64, steps: usize, rest: f64) -> f64 {
        if steps == 0 {
            self.len = 0;
            return value;
        }
        // len <= the last steps <= MAX_DEAD_STEPS, so one more always fits.
        self.values[(self.start + self.len) % LINE] = value;
        self.len += 1;
        if self.len > steps {
            let out = self.values[(self.start + self.len - steps - 1) % LINE];
            self.start = (self.start + self.len - steps) % LINE;
            self.len = steps;
            out
        } else {
            rest
        }
    }
}

/// The brake asked in, a stronger one out when the car slows less than the model.
#[derive(Debug, Clone)]
pub struct AdaptiveBrake {
    car: CarModel,
    limits: BrakeLimits,
    tuning: Tuning,
    effectiveness: f64,
    level: f64,
    line: DelayLine,
    sent: f64,
    speed: Option<f64>,
    measured: f64,
    expected: f64,
    extra: f64,
    high_time: f64,
    spent: bool,
    quiet: f64,
}

impl AdaptiveBrake {
    /// A brake that has learnt nothing yet (effectiveness 1: the empty car).
    #[must_use]
    pub fn new(car: CarModel, limits: BrakeLimits, tuning: Tuning) -> Self {
        AdaptiveBrake {
            car,
            limits,
            tuning,
            effectiveness: 1.0,
            level: 0.0,
            line: DelayLine::new(),
            sent: 0.0,
            speed: None,
            measured: 0.0,
            expected: 0.0,
            extra: 0.0,
            high_time: 0.0,
            spent: false,
            quiet: 0.0,
        }
    }

    /// Forget everything, the effectiveness too: an e-stop, a new drive.
    pub fn reset(&mut self) {
        self.effectiveness = 1.0;
        self.reset_stop();
    }

    /// Forget this stop: the filters, the extra, the high-force time. The effectiveness
    /// stays (a person aboard stays aboard).
    pub fn reset_stop(&mut self) {
        let effectiveness = self.effectiveness;
        *self = AdaptiveBrake::new(self.car, self.limits, self.tuning);
        self.effectiveness = effectiveness;
    }

    /// Measured / expected braking so far (1: the empty car).
    #[must_use]
    pub fn effectiveness(&self) -> f64 {
        self.effectiveness
    }

    /// What the asked brake is multiplied by, 1 to `max_gain`.
    #[must_use]
    pub fn gain(&self) -> f64 {
        clamp(
            1.0 / py_max(self.effectiveness, 1e-6),
            1.0,
            self.tuning.max_gain,
        )
    }

    /// The brake to send, from the speed loop's throttle and brake (0..1) and the
    /// measured wheel speed (m/s; `None` if none is fresh: then it learns nothing and adds
    /// nothing).
    ///
    /// # Errors
    ///
    /// [`Error::BadStep`] if `dt` isn't positive; [`Error::DeadTimeTooLong`] if the brake's
    /// dead time is more than [`MAX_DEAD_STEPS`] ticks of `dt`. Nothing changes then.
    pub fn update(
        &mut self,
        throttle: f64,
        brake: f64,
        speed: Option<f64>,
        dt: f64,
    ) -> Result<BrakeOut, Error> {
        // Not `dt <= 0.0`: a NaN dt is refused too.
        if dt.partial_cmp(&0.0) != Some(core::cmp::Ordering::Greater) {
            return Err(Error::BadStep(dt));
        }
        let steps = delay_steps(self.car.brake_dead_time, dt);
        if steps.is_nan() || steps > MAX_DEAD_STEPS as f64 {
            return Err(Error::DeadTimeTooLong(dt));
        }
        let (c, t) = (self.car, self.tuning);
        let asked = clamp(brake, 0.0, 1.0);
        let k = dt / (t.filter_time + dt);

        // 2. What the empty car's brake would be doing now, for what was sent.
        // steps is a whole number from 0 to MAX_DEAD_STEPS: the cast is exact.
        let delayed = self.line.delay(self.sent, steps as usize, 0.0);
        self.level = lag(self.level, delayed, c.brake_lag, dt);
        self.expected += (self.level * c.max_brake - self.expected) * k;

        // 1. What the car really does: the slowing, less what coasting gives anyway.
        let Some(v) = speed.filter(|s| s.is_finite()).map(|s| s.abs()) else {
            self.speed = None;
            return Ok(self.send(asked, asked, 0.0));
        };
        if let Some(before) = self.speed {
            let slowing = (before - v) / dt - c.drag * v;
            self.measured += (slowing - self.measured) * k;
        }
        self.speed = Some(v);

        // A new stop starts when the brake is let go (or the throttle is asked for).
        self.quiet = if throttle > 0.0 { 0.0 } else { self.quiet + dt };
        if throttle > 0.0 || asked < self.limits.touch_fraction() {
            self.extra = 0.0;
            self.high_time = 0.0;
            self.spent = false;
        }

        // 3. Learn, only while braking for real.
        let settled = self.quiet > c.throttle_dead_time + 3.0 * c.throttle_lag;
        if settled && v > t.min_speed && self.expected > t.min_decel {
            let seen = clamp(self.measured / self.expected, 1.0 / t.max_gain, 1.5);
            self.effectiveness += (seen - self.effectiveness) * dt / (t.adapt_time + dt);
        }

        // 4, 5. The extra: up at most rise_rate, down at once; none once spent.
        let want = if self.spent {
            0.0
        } else {
            asked * (self.gain() - 1.0)
        };
        self.extra = py_min(want, self.extra + t.rise_rate * dt);
        Ok(self.send(asked, asked + self.extra, dt))
    }

    /// 6, 7. Cap at the safe max (the asked brake passes as it is); time high force.
    fn send(&mut self, asked: f64, wanted: f64, dt: f64) -> BrakeOut {
        let safe_max = self.limits.safe_max_fraction();
        let top = py_max(asked, safe_max);
        let mut out = py_min(wanted, top);
        let capped = wanted > top;
        if self.extra > 0.0 && out >= self.tuning.high_share * safe_max {
            self.high_time += dt;
            if self.high_time > self.limits.max_hold_time {
                self.spent = true;
                self.extra = 0.0;
                out = asked;
            }
        }
        self.sent = out;
        BrakeOut {
            brake: out,
            asked,
            gain: self.gain(),
            effectiveness: self.effectiveness,
            capped,
            spent: self.spent,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn brake() -> AdaptiveBrake {
        AdaptiveBrake::new(
            CarModel::default(),
            BrakeLimits::default(),
            Tuning::default(),
        )
    }

    #[test]
    fn the_delay_line_waits_its_steps_and_drops_the_oldest_when_shorter() {
        let mut line = DelayLine::new();
        let outs: [f64; 4] = core::array::from_fn(|i| line.delay(i as f64 + 1.0, 2, -1.0));
        assert_eq!(outs, [-1.0, -1.0, 1.0, 2.0]);
        // Shorter: the newest one step ago.
        assert_eq!(line.delay(5.0, 1, -1.0), 4.0);
        assert_eq!(line.delay(6.0, 0, -1.0), 6.0);
        assert_eq!(line.delay(7.0, 1, -1.0), -1.0);
    }

    #[test]
    fn the_delay_line_wraps_round_its_ring() {
        let mut line = DelayLine::new();
        for i in 0..3 * LINE {
            let out = line.delay(i as f64, MAX_DEAD_STEPS, -1.0);
            let expected = if i >= MAX_DEAD_STEPS {
                (i - MAX_DEAD_STEPS) as f64
            } else {
                -1.0
            };
            assert_eq!(out, expected);
        }
    }

    #[test]
    fn never_above_the_safe_max_unless_asked() {
        let mut b = brake();
        b.effectiveness = 0.2; // a very weak brake: gain 3
        let safe = BrakeLimits::default().safe_max_fraction();
        for i in 0..500 {
            let asked = (i as f64 / 100.0).min(1.0);
            let out = b.update(0.0, asked, Some(2.0), 0.02).unwrap();
            assert!(out.brake <= safe.max(asked) + 1e-12, "{out:?}");
            assert!(
                out.brake >= asked - 1e-12,
                "never weaker than asked: {out:?}"
            );
        }
    }

    #[test]
    fn bad_steps_and_limits_are_errors() {
        let mut b = brake();
        assert_eq!(b.update(0.0, 0.5, Some(1.0), 0.0), Err(Error::BadStep(0.0)));
        assert!(b.update(0.0, 0.5, Some(1.0), f64::NAN).is_err());
        let slow = CarModel {
            brake_dead_time: 10.0,
            ..CarModel::default()
        };
        let mut b = AdaptiveBrake::new(slow, BrakeLimits::default(), Tuning::default());
        assert_eq!(
            b.update(0.0, 0.5, Some(1.0), 0.02),
            Err(Error::DeadTimeTooLong(0.02))
        );
        assert_eq!(
            BrakeLimits::new(400.0, 800.0, 400.0, 3200.0, 5.0),
            Err(Error::NoTravel)
        );
        assert_eq!(
            BrakeLimits::new(400.0, 800.0, 3500.0, 3600.0, 5.0),
            Err(Error::OutsideTravel("safe_max", 3600.0))
        );
        assert_eq!(
            BrakeLimits::new(400.0, 800.0, 3500.0, 3200.0, -1.0),
            Err(Error::NegativeHoldTime(-1.0))
        );
        assert_eq!(
            BrakeLimits::new(400.0, 800.0, 3500.0, 3200.0, 5.0),
            Ok(BrakeLimits::default())
        );
    }

    #[test]
    fn reset_stop_keeps_what_it_learnt_and_reset_forgets() {
        let mut b = brake();
        b.effectiveness = 0.5;
        b.reset_stop();
        assert_eq!(b.effectiveness(), 0.5);
        assert_eq!(b.gain(), 2.0);
        b.reset();
        assert_eq!(b.effectiveness(), 1.0);
    }
}

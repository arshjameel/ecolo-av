//! The pedal: a person's stick read as an accelerator and a brake (`pedal.py`, Ben, 15 Nov).
//!
//! ```text
//! let off to a fraction f of the speed now:  let_off x (1 - f)
//! pulled back by b (0..1 of a full stick):   let_off + b x (max_deceleration - let_off)
//! ```
//!
//! A stale command, the e-stop or a turn's cap still brake at `max_deceleration`: the
//! node and [`crate::ackermann::EgoModel`] decide when the pedal applies.
//!
//! A change of direction waits at a standstill (Ben, 17 Nov: "come to a complete stop and
//! shift into drive or reverse"): ego's speed reaches 0 by its own model while the wheels
//! may still turn, so it holds 0, the brake on, for `shift_hold` seconds; and while a
//! fresh wheel speed exists, until that is under `stop_speed` too ([`Pedal::may_shift`]).

/// The pedal's numbers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pedal {
    /// m/s²: slowing with the stick let go.
    pub let_off: f64,
    /// m/s: the speed a full stick asks for.
    pub full: f64,
    /// s at a standstill before the other way (17 Nov; a guess to tune on stands).
    pub shift_hold: f64,
    /// m/s: a fresh wheel speed under this counts as stopped.
    pub stop_speed: f64,
}

impl Default for Pedal {
    fn default() -> Self {
        Pedal {
            let_off: 1.0,
            full: 1.0,
            shift_hold: 1.0,
            stop_speed: 0.05,
        }
    }
}

impl Pedal {
    /// How hard (m/s²) to slow from `current` toward `target`, both signed m/s.
    ///
    /// Called only while slowing. Against the motion, a target's size over `full` is how
    /// far the stick is pulled back.
    #[must_use]
    pub fn slow_rate(&self, current: f64, target: f64, max_deceleration: f64) -> f64 {
        let let_off = self.let_off.min(max_deceleration);
        if current == 0.0 {
            return max_deceleration;
        }
        if target != 0.0 && (target > 0.0) != (current > 0.0) {
            let back = if self.full > 0.0 {
                1.0_f64.min(target.abs() / self.full)
            } else {
                1.0
            };
            return let_off + back * (max_deceleration - let_off);
        }
        let kept = 1.0_f64.min(target.abs() / current.abs());
        let_off * (1.0 - kept)
    }

    /// Whether the car may change direction: `held` s at a standstill, and stopped.
    ///
    /// `wheel_speed` is a fresh measured speed (m/s), or `None` when there is none (the
    /// Motor Pi alone): then the time alone decides.
    #[must_use]
    pub fn may_shift(&self, held: f64, wheel_speed: Option<f64>) -> bool {
        held >= self.shift_hold && wheel_speed.is_none_or(|v| v.abs() < self.stop_speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn let_go_slows_at_let_off_and_full_back_at_max() {
        let p = Pedal::default();
        assert_eq!(p.slow_rate(1.0, 0.0, 2.0), 1.0);
        assert_eq!(p.slow_rate(1.0, -1.0, 2.0), 2.0);
        assert_eq!(p.slow_rate(1.0, 0.5, 2.0), 0.5);
        assert_eq!(p.slow_rate(0.0, 0.5, 2.0), 2.0);
    }

    #[test]
    fn shifts_after_the_hold_and_once_the_wheels_stop() {
        let p = Pedal::default();
        assert!(!p.may_shift(0.5, None));
        assert!(p.may_shift(1.0, None));
        assert!(p.may_shift(2.0, Some(-0.04)));
        assert!(!p.may_shift(2.0, Some(0.06)));
    }
}

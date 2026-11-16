//! The pedal: a person's stick read as an accelerator and a brake (`pedal.py`, Ben, 15 Nov).
//!
//! ```text
//! let off to a fraction f of the speed now:  let_off x (1 - f)
//! pulled back by b (0..1 of a full stick):   let_off + b x (max_deceleration - let_off)
//! ```
//!
//! A stale command, the e-stop or a turn's cap still brake at `max_deceleration`: the
//! node and [`crate::ackermann::EgoModel`] decide when the pedal applies.

/// The pedal's two numbers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pedal {
    /// m/s²: slowing with the stick let go.
    pub let_off: f64,
    /// m/s: the speed a full stick asks for.
    pub full: f64,
}

impl Default for Pedal {
    fn default() -> Self {
        Pedal {
            let_off: 1.0,
            full: 1.0,
        }
    }
}

impl Pedal {
    /// How hard (m/s²) to slow from `current` toward `target`, both signed m/s.
    ///
    /// Called only while slowing. Against the motion, a target's size over `full` is how
    /// far the stick is pulled back.
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
}

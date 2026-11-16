//! Wheel encoder: quadrature decoding and speed from edge timing.
//!
//! Two sensors A and B a quarter of a slot apart give four counts per slot and
//! the direction. Decoding uses the transition table already in AAV_cortex's
//! knob code: only valid one-bit steps count, so a bouncing sensor adds +1, -1,
//! +1 and nets out to one step. Speed comes from the time between counts, not
//! from counts per window, so it stays smooth at walking pace.

/// Step for each (previous AB, current AB) pair, indexed by `prev << 2 | cur`.
const STEP: [i8; 16] = [0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0];

#[derive(Debug, Clone, Copy)]
pub struct Quadrature {
    last: u8,
    count: i32,
}

impl Quadrature {
    #[must_use]
    pub fn new(a: bool, b: bool) -> Self {
        Quadrature {
            last: (a as u8) << 1 | b as u8,
            count: 0,
        }
    }

    /// Feed the current sensor levels; returns the step taken (-1, 0 or +1).
    pub fn update(&mut self, a: bool, b: bool) -> i8 {
        let cur = (a as u8) << 1 | b as u8;
        let step = STEP[((self.last << 2) | cur) as usize];
        self.last = cur;
        self.count = self.count.wrapping_add(step as i32);
        step
    }

    #[must_use]
    pub fn count(&self) -> i32 {
        self.count
    }
}

/// Speed from the time between encoder counts.
#[derive(Debug, Clone, Copy)]
pub struct SpeedEstimator {
    metres_per_count: f32,
    timeout_us: u32,
    last_count_us: Option<u32>,
    period_us: Option<u32>,
    direction: i8,
}

impl SpeedEstimator {
    /// `timeout_us`: with no count for this long, the car counts as stopped.
    #[must_use]
    pub fn new(metres_per_count: f32, timeout_us: u32) -> Self {
        SpeedEstimator {
            metres_per_count,
            timeout_us,
            last_count_us: None,
            period_us: None,
            direction: 0,
        }
    }

    /// Call with every non-zero step from [`Quadrature::update`].
    pub fn on_count(&mut self, now_us: u32, step: i8) {
        if step == 0 {
            return;
        }
        if step != self.direction {
            // A direction change: the previous period says nothing about now.
            self.period_us = None;
        } else if let Some(t) = self.last_count_us {
            self.period_us = Some(now_us.wrapping_sub(t));
        }
        self.direction = step;
        self.last_count_us = Some(now_us);
    }

    /// Signed speed in m/s.
    #[must_use]
    pub fn speed(&self, now_us: u32) -> f32 {
        let (Some(last), Some(period)) = (self.last_count_us, self.period_us) else {
            return 0.0;
        };
        let since = now_us.wrapping_sub(last);
        if since > self.timeout_us || period == 0 {
            return 0.0;
        }
        // No count for longer than the last period means we are slower now.
        let dt_us = if since > period { since } else { period };
        self.direction as f32 * self.metres_per_count * 1_000_000.0 / dt_us as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_four_per_cycle_both_ways() {
        let mut q = Quadrature::new(false, false);
        for (a, b) in [(false, true), (true, true), (true, false), (false, false)] {
            q.update(a, b);
        }
        assert_eq!(q.count(), 4);
        for (a, b) in [(true, false), (true, true), (false, true), (false, false)] {
            q.update(a, b);
        }
        assert_eq!(q.count(), 0);
    }

    #[test]
    fn bounce_nets_out() {
        let mut q = Quadrature::new(false, false);
        for (a, b) in [(false, true), (false, false), (false, true)] {
            q.update(a, b);
        }
        assert_eq!(q.count(), 1);
    }

    #[test]
    fn invalid_jump_is_ignored() {
        let mut q = Quadrature::new(false, false);
        assert_eq!(q.update(true, true), 0);
    }

    #[test]
    fn speed_from_period() {
        let mut s = SpeedEstimator::new(0.03, 500_000);
        for i in 0..5 {
            s.on_count(i * 10_000, 1);
        }
        let v = s.speed(40_000);
        assert!((v - 3.0).abs() < 1e-3, "{v}");
    }

    #[test]
    fn speed_decays_then_stops() {
        let mut s = SpeedEstimator::new(0.03, 500_000);
        s.on_count(0, 1);
        s.on_count(10_000, 1);
        assert!(s.speed(50_000) < 1.0); // 40 ms without a count: at most 0.75 m/s
        assert_eq!(s.speed(600_000), 0.0);
    }

    #[test]
    fn reverse_is_negative() {
        let mut s = SpeedEstimator::new(0.03, 500_000);
        s.on_count(0, -1);
        s.on_count(20_000, -1);
        assert!(s.speed(20_000) < 0.0);
    }
}

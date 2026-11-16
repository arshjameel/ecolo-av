//! Clamping and rate limiting.

/// Move `current` toward `target` by at most `max_rate * dt`.
#[must_use]
pub fn slew(current: f32, target: f32, max_rate: f32, dt: f32) -> f32 {
    let step = max_rate * dt;
    if target > current + step {
        current + step
    } else if target < current - step {
        current - step
    } else {
        target
    }
}

#[must_use]
pub fn abs(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slew_limits_both_ways() {
        assert_eq!(slew(0.0, 10.0, 2.0, 0.5), 1.0);
        assert_eq!(slew(0.0, -10.0, 2.0, 0.5), -1.0);
        assert_eq!(slew(0.0, 0.5, 2.0, 0.5), 0.5);
    }
}

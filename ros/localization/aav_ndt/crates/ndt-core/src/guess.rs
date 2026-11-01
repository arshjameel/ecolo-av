//! The guess for the next scan: the last NDT pose, moved on by what the local EKF saw the
//! car do since.
//!
//! `guess = last NDT pose ∘ (odom at the last cloud)⁻¹ ∘ (odom at this cloud)`, with odom
//! interpolated at each cloud's own stamp from a short buffer of `/localization/odom/local`.

use std::{collections::VecDeque, time::Duration};

use crate::Pose2;

/// The local EKF's recent poses (`odom → base_link`), by stamp.
#[derive(Clone, Debug)]
pub struct OdomBuffer {
    window: Duration,
    hold: Duration,
    samples: VecDeque<(Duration, Pose2)>,
}

impl OdomBuffer {
    /// Keeps `window` of poses (the design: 1 s). A stamp up to `hold` past the newest pose
    /// gets the newest (the EKF's next pose may not be in yet).
    pub fn new(window: Duration, hold: Duration) -> Self {
        Self {
            window,
            hold,
            samples: VecDeque::new(),
        }
    }

    /// Adds a pose. One older than the newest is dropped: messages out of order.
    pub fn push(&mut self, stamp: Duration, pose: Pose2) {
        if self.samples.back().is_some_and(|(last, _)| stamp <= *last) || !pose.is_finite() {
            return;
        }
        self.samples.push_back((stamp, pose));
        while self
            .samples
            .front()
            .is_some_and(|(first, _)| stamp.saturating_sub(*first) > self.window)
        {
            self.samples.pop_front();
        }
    }

    /// The pose at `stamp`, interpolated between the two around it; `None` if the buffer
    /// doesn't reach that far back, or that far forward.
    pub fn at(&self, stamp: Duration) -> Option<Pose2> {
        let (newest, last) = self.samples.back()?;
        if stamp >= *newest {
            return (stamp - *newest <= self.hold).then_some(*last);
        }
        let after = self.samples.iter().position(|(t, _)| *t >= stamp)?;
        let (t1, p1) = self.samples[after];
        if after == 0 {
            return (t1 == stamp).then_some(p1);
        }
        let (t0, p0) = self.samples[after - 1];
        let span = (t1 - t0).as_secs_f64();
        Some(p0.lerp(&p1, (stamp - t0).as_secs_f64() / span))
    }

    pub fn clear(&mut self) {
        self.samples.clear();
    }
}

/// The last NDT pose moved on by odometry: `last ∘ odom_then⁻¹ ∘ odom_now`.
pub fn predict(last: &Pose2, odom_then: &Pose2, odom_now: &Pose2) -> Pose2 {
    last.compose(&odom_then.between(odom_now))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn the_buffer_interpolates_and_forgets() {
        let mut odom = OdomBuffer::new(ms(1000), ms(50));
        odom.push(ms(0), Pose2::new(0.0, 0.0, 0.0));
        odom.push(ms(100), Pose2::new(1.0, 0.0, 0.2));
        let mid = odom.at(ms(25)).expect("inside");
        assert!((mid.x - 0.25).abs() < 1e-12 && (mid.yaw - 0.05).abs() < 1e-12);
        assert_eq!(odom.at(ms(140)), Some(Pose2::new(1.0, 0.0, 0.2)));
        assert_eq!(odom.at(ms(200)), None);
        odom.push(ms(1200), Pose2::new(2.0, 0.0, 0.0));
        assert_eq!(odom.at(ms(50)), None);
        // Out of order: dropped.
        odom.push(ms(1100), Pose2::new(9.0, 9.0, 0.0));
        assert_eq!(odom.at(ms(1100)), None);
    }

    #[test]
    fn the_guess_follows_the_odometry() {
        // The map says the car faced north at (10, 0); odom says it then drove 1 m ahead.
        let last = Pose2::new(10.0, 0.0, std::f64::consts::FRAC_PI_2);
        let then = Pose2::new(5.0, 5.0, 0.0);
        let now = Pose2::new(6.0, 5.0, 0.0);
        let guess = predict(&last, &then, &now);
        let (metres, radians) = guess.distance(&Pose2::new(10.0, 1.0, last.yaw));
        assert!(metres < 1e-12 && radians < 1e-12);
    }
}

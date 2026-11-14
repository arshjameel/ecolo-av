//! The lag-free car: what Nav2's MPPI assumes (its Ackermann model).
//!
//! The speed follows the command within an acceleration limit, and the wheels point
//! where they're told at once. Kept for comparison: the same MPPI with this model is
//! "MPPI without lag" (`model:=ideal`), so a test or a Gazebo run can show what the lag
//! model buys.

use crate::ego::slew_speed;
use crate::model::{bicycle, CarModel, Command, Observation, Pose};

/// A car with no actuator lag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdealModel {
    /// m, rear axle to front axle.
    pub wheelbase: f64,
    /// rad, full lock either side.
    pub max_steering_angle: f64,
    /// m/s² speeding up.
    pub accel: f64,
    /// m/s² slowing down (positive).
    pub decel: f64,
}

impl Default for IdealModel {
    fn default() -> Self {
        Self {
            wheelbase: 1.561,
            max_steering_angle: 0.5,
            accel: 1.0,
            decel: 2.0,
        }
    }
}

/// The lag-free car's state: where it is, how fast, and its wheels' angle.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct IdealState {
    /// base_link in odom.
    pub pose: Pose,
    /// m/s.
    pub speed: f64,
    /// rad.
    pub steering: f64,
}

impl CarModel for IdealModel {
    type State = IdealState;

    fn at_rest(&self, pose: Pose, speed: f64) -> IdealState {
        IdealState {
            pose,
            speed,
            steering: 0.0,
        }
    }

    fn step(&self, s: &mut IdealState, command: Command, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        s.speed = slew_speed(s.speed, command.speed, self.accel, self.decel, dt);
        let full = self.max_steering_angle;
        s.steering = command.steer.clamp(-full, full);
        bicycle(&mut s.pose, s.speed, s.steering, self.wheelbase, dt);
    }

    fn observe(&self, s: &mut IdealState, seen: &Observation) {
        s.pose = seen.pose;
        s.speed = seen.speed;
    }

    fn pose(&self, s: &IdealState) -> Pose {
        s.pose
    }

    fn speed(&self, s: &IdealState) -> f64 {
        s.speed
    }

    fn steering(&self, s: &IdealState) -> f64 {
        s.steering
    }

    fn wheelbase(&self) -> f64 {
        self.wheelbase
    }

    fn lag_aware(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steers_at_once_and_ramps_the_speed() {
        let m = IdealModel::default();
        let mut s = m.at_rest(Pose::default(), 0.0);
        m.step(&mut s, Command::new(1.0, 0.3), 0.1);
        assert_eq!(s.steering, 0.3);
        assert!((s.speed - 0.1).abs() < 1e-12);
        m.step(&mut s, Command::new(0.0, 0.9), 0.1);
        assert_eq!(s.steering, 0.5);
        assert!(s.speed.abs() < 1e-12);
        assert!(!m.lag_aware());
    }
}

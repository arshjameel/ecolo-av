//! The car model MPPI rolls its samples through: the [`CarModel`] trait.
//!
//! The trait is the seam for the shared car-model crate (another 0.3 job). Until it
//! merges, [`crate::LagModel`] implements the agreed spec here, and [`crate::IdealModel`]
//! is the lag-free model Nav2's MPPI uses, for comparison. At the merge, the shared
//! crate's model implements this trait (or this trait becomes theirs) and nothing else
//! changes.

use std::fmt::Debug;

/// Where the car is: base_link in the odom frame (m, m, rad; REP-103).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    /// m, forward at yaw 0.
    pub x: f64,
    /// m, left at yaw 0.
    pub y: f64,
    /// rad, + anticlockwise.
    pub yaw: f64,
}

impl Pose {
    /// A pose at (x, y), heading `yaw`.
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// The straight-line distance to `other` (m).
    #[must_use]
    pub fn distance(&self, other: &Pose) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

/// What a driver asks for: a speed and a front-wheel angle.
///
/// On the wire it is a `TwistStamped` (v, w = v tan(steer) / wheelbase), which ego turns
/// back into this angle.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Command {
    /// m/s, + forward.
    pub speed: f64,
    /// rad, + left: the virtual centre wheel.
    pub steer: f64,
}

impl Command {
    /// A command of `speed` m/s and `steer` rad.
    #[must_use]
    pub const fn new(speed: f64, steer: f64) -> Self {
        Self { speed, steer }
    }

    /// The yaw rate this command asks for on a car with this wheelbase (rad/s).
    #[must_use]
    pub fn yaw_rate(&self, wheelbase: f64) -> f64 {
        self.speed * self.steer.tan() / wheelbase
    }
}

/// What the node measured this tick. `None`: not measured (the model keeps its own).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Observation {
    /// The pose, from odometry.
    pub pose: Pose,
    /// m/s, from odometry.
    pub speed: f64,
    /// rad: the wheels' measured angle (`/vehicle/steering`), offset included.
    pub steering: Option<f64>,
    /// 0..1: the throttle's measured level (`/vehicle/throttle`).
    pub throttle: Option<f64>,
    /// 0..1: the brake's measured level (`/vehicle/brake`).
    pub brake: Option<f64>,
}

/// A car model: its state, and one step of it under a command.
///
/// The state holds everything the future depends on: the pose and speed, and for a model
/// with lag, the actuators' levels and the commands still in their dead time. MPPI
/// clones it once per sample, so keep it a plain value (no heap) if you can.
pub trait CarModel: Sync {
    /// Everything the model carries from one step to the next.
    type State: Clone + Send + Sync + Debug;

    /// A car at `pose`, at `speed`, its actuators settled (no commands in flight).
    fn at_rest(&self, pose: Pose, speed: f64) -> Self::State;

    /// Move `state` on by `dt` seconds under `command`.
    fn step(&self, state: &mut Self::State, command: Command, dt: f64);

    /// Put what was measured into `state`; keep the model's own guess for the rest.
    fn observe(&self, state: &mut Self::State, seen: &Observation);

    /// The pose in `state`.
    fn pose(&self, state: &Self::State) -> Pose;

    /// The speed in `state` (m/s).
    fn speed(&self, state: &Self::State) -> f64;

    /// The wheels' real angle in `state` (rad, + left).
    fn steering(&self, state: &Self::State) -> f64;

    /// The wheelbase (m): turns a steering angle into a yaw rate.
    fn wheelbase(&self) -> f64;

    /// True if this model knows the actuators' lag: the node then says so on
    /// `lag_aware`, and ego's lead must stay off.
    fn lag_aware(&self) -> bool;
}

/// Wrap an angle into [-pi, pi).
#[must_use]
pub fn wrap(angle: f64) -> f64 {
    let a = (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU);
    a - std::f64::consts::PI
}

/// One bicycle-model step from the rear axle: yaw' = v tan(steer) / wheelbase, then
/// x' = v cos yaw, y' = v sin yaw at the step's mid heading (as the shared car model).
pub fn bicycle(pose: &mut Pose, speed: f64, steer: f64, wheelbase: f64, dt: f64) {
    let turn = speed * steer.tan() / wheelbase * dt;
    let mid = pose.yaw + 0.5 * turn;
    pose.x += speed * mid.cos() * dt;
    pose.y += speed * mid.sin() * dt;
    pose.yaw = wrap(pose.yaw + turn);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_stays_in_range() {
        assert!((wrap(3.0 * std::f64::consts::PI).abs() - std::f64::consts::PI).abs() < 1e-9);
        assert!((wrap(-0.5) + 0.5).abs() < 1e-12);
        assert!((wrap(7.0) - (7.0 - std::f64::consts::TAU)).abs() < 1e-12);
    }

    #[test]
    fn bicycle_drives_a_circle_of_the_right_radius() {
        // At steer d the radius is L / tan(d): one full lap returns to the start.
        let (l, d, v) = (1.561, 0.3_f64, 1.0);
        let radius = l / d.tan();
        let lap = std::f64::consts::TAU * radius / v;
        let dt = 1e-4;
        let mut pose = Pose::default();
        for _ in 0..(lap / dt).round() as usize {
            bicycle(&mut pose, v, d, l, dt);
        }
        assert!(pose.x.hypot(pose.y) < 0.01, "{pose:?}");
    }
}

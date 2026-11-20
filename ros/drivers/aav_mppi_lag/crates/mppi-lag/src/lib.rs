//! MPPI path following, ported from Nav2's `nav2_mppi_controller`, on a car model that
//! knows its actuators' lag (0.3: "have the algorithms know about lag", Ben, 7 Nov).
//!
//! No ROS here (CLAUDE.md: keep the maths out of ROS). The node (`mppi-node`) feeds it
//! odometry and the path, and sends what [`Follower::tick`] says.
//!
//! | Module | Holds |
//! |---|---|
//! | [`model`] | the [`CarModel`] trait: the seam for the shared car-model crate |
//! | [`lag`] | [`LagModel`]: the 0.3 spec (lags, dead times, speed loop, bicycle) |
//! | [`ego`] | ego's ramps, as a stage in front of any car model ([`ego::Driven`]) |
//! | [`ideal`] | [`IdealModel`]: no lag, as Nav2's MPPI assumes; for comparison |
//! | [`optimizer`] | MPPI itself: sample, roll out, score, weigh, smooth |
//! | [`critics`] | Nav2's eight critics that nav2.yaml runs, and our `LaneCritic` |
//! | [`map_correction`] | follow in odom, correct slowly: the held map -> odom, rate-limited |
//! | [`lane`] | the road ahead's lane and its edges, for `LaneCritic` |
//! | [`smoothing`] | Nav2's Savitzky-Golay filter |
//! | [`path`] | the path window (Nav2's PathHandler) and the cross track |
//! | [`costmap`] | the local costmap and the car's outline on it |
//! | [`age`] | the costmap's age: Nav2's `costmap_update_timeout` |
//! | [`follower`] | one path: goal and progress checks, the estimate, MPPI |
//! | [`rng`] | seedable random numbers |
//! | `shared` (feature `shared`) | `SharedModel`: the shared `car-model` crate behind [`CarModel`] |
//!
//! The feature `trace` (TODO 196) wraps each stage of a tick in a `tracing` span
//! (`mppi.tick`, `mppi.rollout`, `mppi.critics`, `mppi.weigh`, `mppi.smooth`); without it
//! [`span!`] is nothing, and the crate has no dependencies.
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

/// Time the rest of the enclosing block as the stage `$name` (feature `trace`).
#[cfg(feature = "trace")]
macro_rules! span {
    ($name:literal) => {
        let _span = tracing::info_span!($name).entered();
    };
}

/// Without the feature `trace`: nothing.
#[cfg(not(feature = "trace"))]
macro_rules! span {
    ($name:literal) => {};
}

pub mod age;
pub mod costmap;
pub mod critics;
pub mod ego;
pub mod follower;
pub mod ideal;
pub mod lag;
pub mod lane;
pub mod map_correction;
pub mod model;
pub mod optimizer;
pub mod path;
pub mod rng;
#[cfg(feature = "shared")]
pub mod shared;
pub mod smoothing;

pub use age::{Arrival, Stale};
/// The shared car model crate (its `ActuatorConfig`, and spine-core's `SpeedLoopProfile`).
#[cfg(feature = "shared")]
pub use car_model;
pub use costmap::Costmap;
pub use critics::{Critic, PathAngleMode};
pub use ego::{Driven, Ego};
pub use follower::{Follower, FollowerSettings, Outcome, Report, Tick};
pub use ideal::IdealModel;
pub use lag::{Actuators, Gear, LagModel, SpeedLoop};
pub use lane::{Edges, Lane, LanePoint};
pub use map_correction::{Correction, MapCorrection, Shift, SlewLimits};
pub use model::{CarModel, Command, Observation, Pose};
pub use optimizer::{Optimizer, Plan, Sampling, Scene, Settings, UnknownSampling};
pub use path::{GlobalPath, Inversion};
#[cfg(feature = "shared")]
pub use shared::{NotPi, SharedModel, SharedState};

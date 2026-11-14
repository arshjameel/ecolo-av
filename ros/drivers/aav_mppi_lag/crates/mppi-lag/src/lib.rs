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
//! | [`critics`] | Nav2's eight critics that nav2.yaml runs |
//! | [`smoothing`] | Nav2's Savitzky-Golay filter |
//! | [`path`] | the path window (Nav2's PathHandler) and the cross track |
//! | [`costmap`] | the local costmap and the car's outline on it |
//! | [`age`] | the costmap's age: Nav2's `costmap_update_timeout` |
//! | [`follower`] | one path: goal and progress checks, the estimate, MPPI |
//! | [`rng`] | seedable random numbers |
//! | `shared` (feature `shared`) | `SharedModel`: the shared `car-model` crate behind [`CarModel`] |

pub mod age;
pub mod costmap;
pub mod critics;
pub mod ego;
pub mod follower;
pub mod ideal;
pub mod lag;
pub mod model;
pub mod optimizer;
pub mod path;
pub mod rng;
#[cfg(feature = "shared")]
pub mod shared;
pub mod smoothing;

pub use age::{Arrival, Stale};
pub use costmap::Costmap;
pub use critics::{Critic, PathAngleMode};
pub use ego::{Driven, Ego};
pub use follower::{Follower, FollowerSettings, Outcome, Report, Tick};
pub use ideal::IdealModel;
pub use lag::{Actuators, Gear, LagModel, SpeedLoop};
pub use model::{CarModel, Command, Observation, Pose};
pub use optimizer::{Optimizer, Plan, Scene, Settings};
pub use path::{GlobalPath, Inversion};
#[cfg(feature = "shared")]
pub use shared::{SharedModel, SharedState};

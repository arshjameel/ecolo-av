//! NDT scan matching: where the car is on a saved lidar map, in x, y and yaw.
//!
//! The map is cut into cubes; each cube with enough points keeps their mean and covariance
//! (a bell curve). A scan is slid and turned until its points sit best on those curves
//! (Biber and Strasser 2003; Magnusson 2009, chapter 6). No ROS here: `ndt-node` feeds it
//! messages. The design: `docs/designs/2026-10-29_ndt.md`.
//!
//! | Module | Holds |
//! |---|---|
//! | [`pose`] | [`Pose2`] (x, y, yaw) and the lidar's [`Mount`] |
//! | [`cloud`] | the scan filters (ground, body, range, non-finite) and thinning; PointCloud2 bytes |
//! | [`map`] | the [`CubeMap`]: one bell curve per cube |
//! | [`score`] | the score of a pose, its gradient and Hessian |
//! | [`matcher`] | Newton with a step cap, a line search and damping, coarse to fine |
//! | [`guess`] | the odometry buffer and the guess for the next scan |
//! | [`localizer`] | all of it per cloud: the guess, the match, LOST and the restarts, the covariance |
//! | [`pcd`] | the map file: PCL's PCD, binary or ASCII |

pub mod cloud;
pub mod guess;
pub mod localizer;
pub mod map;
pub mod matcher;
pub mod pcd;
pub mod pose;
pub mod score;

pub use cloud::{Ground, ScanFilter};
pub use guess::OdomBuffer;
pub use localizer::{Localizer, LocalizerSettings, Outcome, Report, Tracking};
pub use map::{CubeMap, MapError, MapSettings};
pub use matcher::{Match, MatchError, MatchSettings, Matcher};
pub use pose::{Mount, Pose2};

/// A point, in metres.
pub type Point = nalgebra::Point3<f64>;

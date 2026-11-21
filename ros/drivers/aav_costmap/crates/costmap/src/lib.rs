//! Our own costmap: Nav2's `costmap_2d` layers, ported to Rust, with no ROS.
//!
//! | Module | Nav2's |
//! |---|---|
//! | [`cost`] | the cost codes, and `Costmap2DPublisher`'s table to an `OccupancyGrid` |
//! | [`grid`] | `Costmap2D`: cells, `updateOrigin` (the rolling window), `raytraceLine` |
//! | [`footprint`] | `footprint.cpp`: the padded outline, its inscribed radius, its cells |
//! | [`static_layer`] | `StaticLayer`: the map's cells, read as free, lethal or unknown |
//! | [`obstacle`] | `ObstacleLayer`: marks the sensors' points, clears along each ray |
//! | [`inflation`] | `InflationLayer`: the cost falling away from each lethal cell |
//! | [`lanes`] | none: ours, the roads as a soft cost, lowest at the right lane's centre |
//! | [`camera_lanes`] | none: ours, the same costs from the camera's fitted lines (TODO 264) |
//! | [`chunked`] | none: ours, the global costmap from map chunks (TODO 175) |
//! | [`layered`] | `LayeredCostmap`: the layers in order, into one grid |
//! | [`age`] | `isCurrent` and `transform_tolerance`: how old an input is, poses by stamp |
//!
//! The node (`costmap-node`) only moves messages in and out of a [`Costmap`].

#![forbid(unsafe_code)]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod age;
pub mod camera_lanes;
pub mod chunked;
pub mod cost;
pub mod footprint;
pub mod grid;
pub mod inflation;
pub mod lanes;
pub mod layered;
pub mod obstacle;
pub mod static_layer;
pub mod transform;

pub use age::{Arrival, PoseHistory, Stale};
pub use camera_lanes::{CameraLanes, CameraSettings, Paint, Seen, SeenLine};
pub use chunked::{ChunkError, ChunkSource, Window};
pub use grid::{Bounds, Grid};
pub use inflation::Inflation;
pub use lanes::{LaneError, Lanes, Ramp, Road, Way};
pub use layered::{Changed, Costmap, Moved, Settings};
pub use obstacle::{Observation, ObstacleLayer, Source};
pub use static_layer::StaticMap;
pub use transform::{Pose2, Transform3};

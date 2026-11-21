//! Our own simulator's core: Gazebo's job, in Rust, with no ROS and no Bevy.
//!
//! | Module | Holds |
//! |---|---|
//! | [`world`] | the worlds Gazebo reads (`*.sdf`, `model://`, OBJ meshes), as triangles |
//! | [`scene`] | what a ray can hit: a bounding volume tree, and objects placed while running |
//! | [`car`] | the car: the [`car::CarModel`] trait, Gazebo's twist follower, the actuator model |
//! | `shared_car` (feature `shared`) | `SharedCar`: the shared car-model crate behind the [`car::CarModel`] trait |
//! | [`lidar`] | the roof lidar and the radar stand-in, by ray casting, packed as Gazebo's |
//! | [`ground`] | the ground under each wheel: slopes tilt the car, a kerb too high stops it |
//! | [`geo`] | the datum, and world positions as latitude and longitude |
//! | [`sim`] | the loop: step the car, and which stream speaks when; the camera plug-in |
//! | [`lockstep`] | the cameras' gate: the simulated time waits for a slow picture (software rendering) |
//! | [`noise`] | seeded noise, so the same seed gives the same sensor streams |
//! | [`node`] | the `simulator` node's decisions without ROS: parameter checks, its services' answers, stamps, its pace |
//! | [`math`] | vectors, rotations, poses |
//!
//! Units: m, m/s, rad, s. Frames: REP-103 and REP-105, the world = the map (x east, y
//! north), `base_link` on the ground under the rear axle.
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod car;
pub mod geo;
pub mod ground;
pub mod lidar;
pub mod lockstep;
pub mod math;
pub mod node;
pub mod noise;
pub mod scene;
#[cfg(feature = "shared")]
pub mod shared_car;
pub mod sim;
pub mod world;

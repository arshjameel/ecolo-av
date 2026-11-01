//! The debug HUD's logic, with no ROS and no raylib, so it is tested on any laptop.
//!
//! The debug HUD (`debug_hud`, in debug-app) shows what the car sees and what it
//! thinks: the lidar's cubes around the car, where each localizer puts the car, and a
//! panel of every sensor straight from its own topic (docs/designs/2026-10-29_debug-hud.md).
//!
//! | Module | What it holds |
//! |---|---|
//! | [`config`] | the topic names, frames and limits, from `config/debug_hud.yaml` |
//! | [`frames`] | stamps, 3D transforms, the static frames from `/tf_static`, map -> odom |
//! | [`poses`] | the local EKF's last second of poses, interpolated at a cloud's stamp |
//! | [`cloud`] | a PointCloud2's x, y, z (radar-clustering's reader) |
//! | [`cubes`] | one scan's cubes: snapped to a grid in odom, a height band each, a cap |
//! | [`scan`] | a cloud start to finish: frames, pose, points, cubes |
//! | [`persist`] | cubes kept in odom that fade when not seen again (key K) |
//! | [`ndt`] | NDT's status in plain words; its map's occupied cells (key M) |
//! | [`keys`] | the keys, the layers they toggle, the on-screen key legend |
//! | [`orbit`] | the 3D camera: chase, follow, free; turn, tilt, zoom, pan |
//! | [`track`] | aav_track's status: the tracker, cross track, loop time |
//! | [`swap`] | keys 1-4: which tracker, and whether a swap may be asked for (sim only) |
//! | [`topdown`] | the seeing screen's top-downs: the map frame in a rectangle of the window |
//! | [`state`] | everything above, as one value the screen draws |

pub mod cloud;
pub mod config;
pub mod cubes;
pub mod frames;
pub mod keys;
pub mod ndt;
pub mod orbit;
pub mod persist;
pub mod poses;
pub mod scan;
pub mod state;
pub mod swap;
pub mod topdown;
pub mod track;

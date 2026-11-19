//! The diffusion planner's model, in plain Rust (no ROS): a costmap crop, the start and
//! goal, and noise in; 16 paths out. `aav_diffusion_planner`'s `worker.py`, ported.
//!
//! | Part | What |
//! |---|---|
//! | [`Model`] | the `.onnx` (`pixi run diffusion train`) through ONNX Runtime, on the provider asked for, the CPU if that one can't |
//! | [`batches`] | any number of samples through a model made for 16 |
//! | [`wire`] | the pipe between the Python node and its worker: the same bytes as `wire.py` |
//!
//! The binary `diffusion_worker` is the worker itself: the node starts it in the Python
//! worker's place with `worker:=rust` (launch: `diffusion_impl:=rust`). The guard, the
//! window and the fallback to the lattice stay in the node, unchanged.

mod error;
mod model;
pub mod wire;

pub use error::Error;
pub use model::{batches, Model, CHANNELS, COND, POINTS, SIZE};
pub use onnx_session::{Coverage, Provider};

/// This crate's `Result`.
pub type Result<T> = std::result::Result<T, Error>;

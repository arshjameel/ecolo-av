//! Our ONNX models on ONNX Runtime, in plain Rust (no ROS): one session builder for every
//! model (the detector, the diffusion planner), so each picks its hardware the same way.
//!
//! | Part | What |
//! |---|---|
//! | [`Provider`] | where the model runs: the CPU, Core ML (a Mac), CUDA or TensorRT (the Orin) |
//! | [`open`] | a session on that provider, and only that one; what it compiles kept in [`cache_dir`] |
//! | [`Coverage`] | how much of the graph the provider took (ONNX Runtime's own log line) |
//! | [`placements`] | which provider ran each node (ONNX Runtime's profile) |
//!
//! The coverage is the best guess, on a Mac, of whether TensorRT will run a model whole:
//! a node Core ML hands back to the CPU is usually one TensorRT's parser balks at too.

mod coverage;
mod error;
mod provider;
mod session;

pub use coverage::{placements, Coverage, Placements};
pub use error::Error;
pub use provider::Provider;
pub use session::{cache_dir, open, Opened, Options};

/// This crate's `Result`.
pub type Result<T> = std::result::Result<T, Error>;

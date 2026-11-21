//! The lag-aware MPC in Rust: aav_track's `mpc_lag` (Python), ported step for step, a
//! challenger beside it (TODO 132, 199).
//!
//! No ROS here (CLAUDE.md: keep the maths out of ROS). The node (`mpc-node`) feeds it the
//! car's pose and speed and the path, and sends what [`MpcLag::step`] says.
//!
//! | Module | Holds | Python |
//! |---|---|---|
//! | [`path`] | [`Path`]: the foot, the curvature, `wrap` | `aav_track/path.py` |
//! | [`model`] | the car with its lags, in the path's frame: [`model::step`] | `aav_track/lag_model.py` |
//! | [`qp`] | OSQP's ADMM, dense P, sparse A | `aav_track/qp.py` |
//! | [`tracker`] | [`MpcLag`]: the estimate, the plan, the QP, the command | `aav_track/trackers/mpc_lag.py` |
//! | [`gears`] | [`Gears`]: the path cut at its cusps; a backwards stretch mirrored | `aav_track/gears.py` |
//! | [`geared`] | [`Geared`]: the MPC forward and back, a stretch at a time | `aav_track/track_node.py`'s gears |
//!
//! The estimate runs the shared car model (`car-model`, the twin of `car_model.py`), with
//! the spine's own speed loop (spine-core).
//!
//! ```
//! use mpc_lag::{Limits, MpcLag, Observed, Path, Settings};
//!
//! let limits = Limits { wheelbase: 1.561, max_steer: 0.5, max_steer_rate: 0.6 };
//! let mut mpc = MpcLag::new(limits, Default::default(), Settings::default()).unwrap();
//! let line: Vec<_> = (0..100).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
//! mpc.reset(Path::new(&line).unwrap());
//! let cmd = mpc.step(&Observed::default(), 0.05).unwrap();
//! assert!(cmd.speed > 0.0 && cmd.steer.abs() < 1e-3);
//! ```
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod geared;
pub mod gears;
pub mod model;
pub mod path;
pub mod qp;
pub mod tracker;

pub use geared::{FollowError, Geared};
pub use gears::{CuspRules, Direction, Gears, Pose, Stretch};
pub use model::{LoopError, PiLoop};
pub use path::{wrap, Foot, Path, PathError};
pub use tracker::{Command, Limits, MpcLag, Observed, Report, Settings, TrackError};

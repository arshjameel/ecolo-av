//! The lane follower without ROS: `control_interface`'s five states, and three ways to
//! steer by the lane the camera sees.
//!
//! - [`Follower`]: the 2025-26 team's state machine (aav_lane_geometry's `lane_follow.py`,
//!   ported): keep the lane, ride out a lane change, slow down and stop when the lines are
//!   lost or the messages stop, resume when they come back.
//! - [`Law`]: how it steers while keeping the lane. [`Law::Pid`] is the 2025-26 PID on the
//!   offset (the Python's answers, `tests/vectors.rs`); [`Law::PurePursuit`] and
//!   [`Law::Stanley`] are the challengers, on the fitted lane centre, with the car's lag
//!   (README.md says why).
//!
//! Units: metres, seconds, radians; x forward, y left, + turn = left (REP-103). Two clocks,
//! as the Python has them: `wall` (the PID, the offset's rate, how long a state lasts) and
//! `now` (the node's clock: the watchdog, the last message, the last command).

mod follower;
mod pyfmt;
mod steering;

pub use follower::{Command, Follower, Reply, Settings, State};
pub use pyfmt::fixed;
pub use steering::{
    Geometry, Law, LawName, Line, Pid, PidGains, PursuitSettings, StanleySettings, UnknownLaw,
};

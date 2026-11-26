//! The safety filter for untrusted drivers (TODO 104; the design: "How the car stops",
//! the filter and K1).
//!
//! An automatic driver (navigation, the point driver, the AI or LLM slot) sends `(v, w)`.
//! The filter sends on the command nearest to it that keeps three things true:
//!
//! | Kept | How | Hard? |
//! |---|---|---|
//! | the car can still stop before the road ahead's end | a control barrier function on the stopping law `S(v)` | yes |
//! | the car stays inside its lane | control barrier functions on a look-ahead point | yes, while the wheels can |
//! | the car settles on the lane's line | a Lyapunov function, with a slack | no: a pull |
//!
//! It only ever slows the car or bends its turn: it never speeds it up, never turns a
//! stop into motion, never reverses a direction. It sits in front of twist_mux, so a person
//! on the joystick or RC never goes through it (K1).
//!
//! | Module | Holds |
//! |---|---|
//! | [`lane`] | the road ahead's line and edges, and the car's frame in it |
//! | [`filter`] | the filter: [`filter::filter`], [`filter::SafetyFilter`], its [`filter::Report`] |
//! | [`sim`] | a small closed loop with a lagging car, for tests and experiments |
//!
//! No ROS here: `cargo test -p safety-filter` on any laptop.

pub mod filter;
pub mod lane;
pub mod sim;

pub use filter::{Command, Inputs, Report, Rule, SafetyFilter, Settings, SettingsError};
pub use lane::{Frame, Lane};

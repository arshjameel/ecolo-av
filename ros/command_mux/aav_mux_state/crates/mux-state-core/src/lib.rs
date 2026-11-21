//! Who is driving? twist_mux's choice, worked out beside it, with no ROS.
//!
//! twist_mux passes on one driver's commands but never says which. This crate reads the
//! same parameter files twist_mux reads ([`config`]), hears the same inputs and locks,
//! and applies the same rule ([`mux`]) to say who it picked, and why the others lost
//! ([`report`]): the debug screen's "who is driving" (TODO 247).
//!
//! | Module | What it holds |
//! |---|---|
//! | [`config`] | twist_mux's inputs and locks, from its YAML files, later files winning |
//! | [`mux`] | the rule: the highest-priority fresh input not held by a lock above it |
//! | [`report`] | the answer as a short line and as key-value rows |
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod config;
pub mod mux;
pub mod report;

pub use config::{ConfigError, MuxConfig, Slot};
pub use mux::{Choice, Driver, LockOn, Mux, Why};
pub use report::Level;

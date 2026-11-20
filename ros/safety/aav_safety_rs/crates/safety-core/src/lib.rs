//! The safety locks' rules, with no ROS: the Rust twins of `aav_safety`'s `geofence.py`,
//! `hold.py` and `leash.py`. The nodes (`safety-node`) only carry messages to and from
//! them. Each module is checked, event by event, against the Python's answers
//! (`tests/vectors.json`, written by `gen_vectors.py`).
//!
//! | Module | Lock | Python |
//! |---|---|---|
//! | [`geofence`] | twist_mux's geofence lock (51): every program held near the fence | `geofence.py` |
//! | [`hold`] | twist_mux's stop_zone lock (21): a stop held until the way stays clear | `hold.py` |
//! | [`leash`] | a laptop driver's goals cancelled when it goes quiet | `leash.py` |
//!
//! Times are seconds on any steady clock; lengths metres.
#![warn(missing_docs, clippy::must_use_candidate)]

pub mod geofence;
pub mod hold;
pub mod leash;
pub mod pyfmt;

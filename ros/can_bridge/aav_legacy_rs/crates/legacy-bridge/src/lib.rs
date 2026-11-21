//! `legacy_uart_bridge`'s logic, with no ROS: the Rust twin of `aav_legacy`'s
//! `legacy_protocol.py` (the mapping), `uart_relay.py`, `speed_control.py`,
//! `uplink_reader.py` and `link_report.py`.
//!
//! ```text
//!   ego's command, the Orin's counter, /safety/estop ──> Relay ──> one frame a tick
//!                                                         │       (legacy-link encodes it)
//!   the ESP32's uplink bytes ──> UplinkReader ──> status, steering, brake, RC, raw counts
//!   each frame written ──> LinkReport ──> /control/car/link_frame's fields and counts
//! ```
//!
//! The frames are the firmware's own code (`firmware/spine/crates/legacy-link`), so the
//! bridge and the ESP32 can't disagree about a byte. `tests/vectors.rs` checks every
//! answer against the Python's (`gen_vectors.py` writes `tests/vectors.json`).
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod link;
pub mod mapping;
pub mod relay;
pub mod speed;
pub mod uplink;

pub use adaptive_brake::{AdaptiveBrake, BrakeLimits, BrakeOut, CarModel, Tuning};
pub use link::{frame_fields, LinkError, LinkFields, LinkReport};
pub use mapping::{Mapping, Values};
pub use relay::{
    programs_locked, CalMode, CalRefusal, CalWhere, FrameVersion, Relay, RelaySettings,
    SettingError,
};
pub use speed::{Pedals, SpeedConfig, SpeedControl};
pub use uplink::{UplinkParser, UplinkReader};

/// Python's `max(a, b)`: `a`, unless `b` is greater (so NaN answers as Python's does).
#[must_use]
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python's `min(a, b)`: `a`, unless `b` is smaller.
#[must_use]
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

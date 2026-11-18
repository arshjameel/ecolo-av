//! Drive to one point B and stop there: the point driver's maths and rules. No ROS here.
//! The Rust twin of `aav_drive_to_point`'s `approach.py`, `line.py`, `drive.py`,
//! `rules.py` and `run.py`, with the same answers (`tests/vectors.json`, from
//! `gen_vectors.py`).
//!
//! ```text
//!  goal ─> run::plan (ego's caps) ─> Approach ┐
//!  pose, speed ─> run::Run::tick ─> drive::Drive::tick ─> (speed, curvature) ─> /drivers/point/cmd_vel
//!                      └─ rules::stop_reason, the corridor (line::Fence), the drive's limits
//! ```
//!
//! | Module | Holds |
//! |---|---|
//! | [`approach`] | the speed: speed up, hold, slow by the distance left |
//! | [`line`] | the line to B, pure pursuit, the corridor |
//! | [`drive`] | speed and steering together, one tick |
//! | [`rules`] | when a drive may start, when it must stop |
//! | [`run`] | one drive, from its plan to its result |
//! | [`cli`] | the `drive_to_point` command's line, words and sums |
#![warn(missing_docs, clippy::must_use_candidate)]

pub mod approach;
pub mod cli;
pub mod drive;
pub mod line;
pub mod rules;
pub mod run;

/// ego's crate: the car model and the stopping law, for the node.
pub use ego_core;
pub use ego_core::{fmt_g, hypot};

/// Python's `max(a, b)`: `a`, unless `b` is greater (so a NaN `b` loses).
#[must_use]
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python's `min(a, b)`: `a`, unless `b` is smaller (so a NaN `b` loses).
#[must_use]
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's `repr(x)` of a float, as its f-strings write `{x}`: `1.0`, `-0.5`, `nan`,
/// `inf`, `1e-05`.
#[must_use]
pub fn py_repr(x: f64) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf" } else { "-inf" }.into();
    }
    let a = x.abs();
    if a != 0.0 && !(1e-4..1e16).contains(&a) {
        // Rust writes 1e-5 and 1.5e16; Python 1e-05 and 1.5e+16.
        let text = format!("{x:e}");
        let (mantissa, exponent) = text.split_once('e').unwrap_or((&text, "0"));
        let (sign, digits) = match exponent.strip_prefix('-') {
            Some(d) => ('-', d),
            None => ('+', exponent),
        };
        return format!("{mantissa}e{sign}{digits:0>2}");
    }
    let text = format!("{x}");
    if text.contains('.') {
        text
    } else {
        format!("{text}.0")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floats_read_as_python_writes_them() {
        assert_eq!(py_repr(1.0), "1.0");
        assert_eq!(py_repr(-0.5), "-0.5");
        assert_eq!(py_repr(f64::NAN), "nan");
        assert_eq!(py_repr(f64::NEG_INFINITY), "-inf");
        assert_eq!(py_repr(1e-5), "1e-05");
        assert_eq!(py_repr(1.5e16), "1.5e+16");
        assert_eq!(py_repr(0.1), "0.1");
        assert_eq!(py_max(1.0, f64::NAN), 1.0);
        assert_eq!(py_min(1.0, f64::NAN), 1.0);
    }
}

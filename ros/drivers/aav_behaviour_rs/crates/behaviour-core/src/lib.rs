//! The behaviour planner's maths, with no ROS: the Rust twin of `aav_behaviour`'s
//! `behaviour.py`, and of the `aav_description` modules its node uses.
//!
//! ```text
//! locks, odometry, stops ──▶ Behaviour::step ──▶ Decision: state, reason, cap (Nav2's limit)
//! the plan + the car + stops ──▶ road::build (+ lanes::lane_line, with_edges) ──▶ the road ahead
//! ```
//!
//! | Module | The Python it ports |
//! |---|---|
//! | [`machine`] | `behaviour.py`: `Behaviour`, `Settings`, `Stop`, `World`, `Decision`, `diagnostic` |
//! | [`paths`] | `behaviour.py`: `active_plan`, `path_left`, `along_until`, `lane_ahead`, `moving_ahead`, `kerb_ahead` |
//! | [`road`] | `aav_description/road_ahead.py`, the writer's half: `resample`, `build` |
//! | [`lanes`] | `aav_description/lane_map.py`: `LaneMap`, `lane_line`, `with_edges` |
//! | [`fence`] | `aav_safety/geofence.py`: `Polygon` and its depth |
//! | [`tf`] | tf2's buffer, the part the node uses: each link's history, looked up at a time |
//!
//! The same answers as the Python: `tests/vectors.rs` reads `vectors.json`, written by
//! `gen_vectors.py` from the Python modules. Units: m, s, rad, m/s (REP-103).

// The Rust API Guidelines, kept: every public item documented, its errors too, and a pure
// function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod fence;
pub mod lanes;
pub mod machine;
pub mod paths;
pub mod road;
pub mod stop_lines;
pub mod tf;

pub use ego_core;
pub use ego_core::hypot;

/// Python 3.12's `sum()` of floats: Neumaier's compensated sum, as CPython adds them.
///
/// A plain `Iterator::sum` may differ in the last bit, and the goal's distance with it.
#[must_use]
pub fn py_sum(values: impl IntoIterator<Item = f64>) -> f64 {
    let (mut total, mut c) = (0.0_f64, 0.0_f64);
    for x in values {
        let t = total + x;
        if total.abs() >= x.abs() {
            c += (total - t) + x;
        } else {
            c += (x - t) + total;
        }
        total = t;
    }
    if c != 0.0 && c.is_finite() {
        total += c;
    }
    total
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sum_is_compensated_as_pythons() {
        // A plain sum gives 0.0 here; Python 3.12 gives 2.0.
        assert_eq!(py_sum([1e100, 1.0, -1e100, 1.0]), 2.0);
        assert_eq!(py_sum([0.1; 10]), 1.0);
        assert_eq!(py_sum(std::iter::empty()), 0.0);
    }

    #[test]
    fn max_and_min_keep_the_first_on_a_tie_or_a_nan() {
        assert_eq!(py_max(0.0, f64::NAN), 0.0);
        assert!(py_max(f64::NAN, 0.0).is_nan());
        assert_eq!(py_min(1.0, 0.5), 0.5);
    }
}

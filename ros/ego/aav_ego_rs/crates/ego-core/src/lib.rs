//! ego's and look_ahead's maths, with no ROS: the Rust twin of `aav_ego`'s Python modules.
//!
//! ```text
//! /control/cmd_vel (v, w) ──▶ road_limit ──▶ EgoModel::step (lead, pedal, body, ramps)
//!                                              ──▶ (speed, steering, accel): /ego/actuator_cmd
//! the twin (car model) ──▶ trace ──▶ /ego/trace;  stop_point ──▶ /ego/stop_point
//! the trace + the plan + the costmap + the road ahead ──▶ look_ahead ──▶ /ego/ahead
//! ```
//!
//! | Module | The Python it ports |
//! |---|---|
//! | [`ackermann`] | `aav_ego/ackermann.py` |
//! | [`dynamics`] | `aav_ego/dynamics.py` |
//! | [`pedal`] | `aav_ego/pedal.py` |
//! | [`lag_lead`] | `aav_ego/lag_lead.py` |
//! | [`stop_law`] | `aav_description/stop_law.py`: the curve ego and its readers use |
//! | [`road_ahead`] | `aav_description/road_ahead.py`, the readers' half; `aav_ego/road_limit.py` |
//! | [`trace`] | `aav_ego/trace_ahead.py` |
//! | [`stop_ahead`] | `aav_ego/stop_ahead.py` |
//! | [`look_ahead`] | `aav_ego/look_ahead.py`, and the node's `Frame2D` |
//! | [`frames`] | a small TF tree for look_ahead (tf2's buffer, newest only) |
//!
//! The same answers as the Python: `tests/vectors.rs` reads `vectors.json`, written by
//! `gen_vectors.py` from the Python modules. Units: m, s, rad, m/s (REP-103).
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod ackermann;
pub mod dynamics;
pub mod frames;
pub mod lag_lead;
pub mod look_ahead;
pub mod pedal;
pub mod road_ahead;
pub mod stop_ahead;
pub mod stop_law;
pub mod trace;

pub use car_model;

/// `max(lo, min(hi, x))`, as the Python's `clamp`.
#[must_use]
pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    lo.max(hi.min(x))
}

/// √(x² + y²) as Python's `math.hypot` and `math.dist` work it out (CPython 3.12's
/// `vector_norm`: scaled, summed in double-double, one differential correction).
///
/// libm's `hypot` may differ in the last bit; a step count `ceil(seg / step)` on a segment
/// exactly `step` long then differs by one, and look_ahead's obstacle distance by a step.
#[must_use]
pub fn hypot(x: f64, y: f64) -> f64 {
    let (x, y) = (x.abs(), y.abs());
    if x.is_nan() || y.is_nan() {
        return if x.is_infinite() || y.is_infinite() {
            f64::INFINITY
        } else {
            f64::NAN
        };
    }
    let max = x.max(y);
    if max.is_infinite() {
        return max;
    }
    if max == 0.0 {
        return 0.0;
    }
    if !max.is_normal() {
        // Subnormal: no number here is ever that small. libm's answer is close enough.
        return x.hypot(y);
    }
    // frexp: max = m 2^e, 0.5 <= m < 1.
    let e = ((max.to_bits() >> 52) & 0x7ff) as i32 - 1022;
    let scale = 2.0_f64.powi(-e);
    let (mut csum, mut frac1, mut frac2) = (1.0_f64, 0.0_f64, 0.0_f64);
    for v in [x * scale, y * scale] {
        let hi = v * v;
        let lo = v.mul_add(v, -hi);
        let sum = csum + hi;
        frac2 += (csum - sum) + hi;
        csum = sum;
        frac1 += lo;
    }
    let mut h = (csum - 1.0 + (frac1 + frac2)).sqrt();
    let hi = -h * h;
    let lo = (-h).mul_add(h, -hi);
    let sum = csum + hi;
    frac2 += (csum - sum) + hi;
    csum = sum;
    frac1 += lo;
    let r = csum - 1.0 + (frac1 + frac2);
    h += r / (2.0 * h);
    h / scale
}

/// Python's `round()`: halves go to the even number.
#[must_use]
pub fn round_half_even(x: f64) -> f64 {
    x.round_ties_even()
}

/// An `f32` from spine-core, as the decimal the Python writes for it (`0.2f32` is `0.2`,
/// not `0.20000000298`): the speed loop's numbers are written once, in decimals, in both.
#[must_use]
pub fn widen(x: f32) -> f64 {
    x.to_string().parse().unwrap_or(f64::from(x))
}

/// Python's `format(x, 'g')` for the numbers ego logs: six significant digits, no
/// trailing zeros (`1.0` is `1`, `0.4` is `0.4`).
#[must_use]
pub fn fmt_g(x: f64) -> String {
    if !x.is_finite() {
        return if x.is_nan() {
            "nan".into()
        } else if x > 0.0 {
            "inf".into()
        } else {
            "-inf".into()
        };
    }
    if x == 0.0 {
        return "0".into();
    }
    let digits = 6_i32 - 1 - x.abs().log10().floor() as i32;
    let text = format!("{:.*}", digits.max(0) as usize, x);
    if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn g_formats_as_python() {
        assert_eq!(fmt_g(1.0), "1");
        assert_eq!(fmt_g(0.4), "0.4");
        assert_eq!(fmt_g(2.5), "2.5");
        assert_eq!(fmt_g(0.0), "0");
        assert_eq!(fmt_g(181.0), "181");
    }

    #[test]
    fn widen_keeps_the_decimal() {
        assert_eq!(widen(0.2), 0.2);
        assert_eq!(widen(1.832), 1.832);
    }

    #[test]
    fn hypot_is_pythons() {
        // Python: math.hypot(-0.053511986537932626, -0.08447761417537852); libm's is
        // 0.09999999999999998.
        assert_eq!(
            hypot(-0.053511986537932626, -0.08447761417537852),
            0.09999999999999999
        );
        assert_eq!(hypot(3.0, 4.0), 5.0);
        assert_eq!(hypot(0.0, 0.0), 0.0);
        assert_eq!(hypot(f64::NAN, f64::INFINITY), f64::INFINITY);
        assert!(hypot(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn rounds_half_to_even() {
        assert_eq!(round_half_even(2.5), 2.0);
        assert_eq!(round_half_even(3.5), 4.0);
    }
}

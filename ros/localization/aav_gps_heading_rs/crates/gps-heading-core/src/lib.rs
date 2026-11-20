//! The car's heading from the GPS receiver's own velocity (course over ground). No ROS
//! here. The Rust twin of `aav_gps_heading`'s `course.py`.
//!
//! A receiver measures its velocity from the satellites' Doppler shift, good to about
//! 0.05 m/s even while the position wanders by a metre. Its direction is where the antenna
//! goes; on a car that doesn't slide, where the car points (or the opposite, reversing):
//!
//! ```text
//! heading = the velocity's direction - the antenna's own direction of travel on the car
//! beta    = atan2(w * x_a, v - w * y_a)      (the antenna at (x_a, y_a) from base_link)
//! ```
//!
//! Every sum is in the Python's order, and `hypot` is CPython's, so the answers match
//! `course.py` to the bit (`tests/vectors.rs`). m/s, rad.
#![warn(missing_docs, clippy::must_use_candidate)]

use std::f64::consts::PI;

/// When a velocity counts as a heading, and how sure it is. m/s and rad.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CourseSettings {
    /// Slower than this (the GPS's speed or the wheels'), no heading.
    pub min_speed: f64,
    /// The GPS's speed and the wheels' must agree to this.
    pub max_speed_gap: f64,
    /// Added to every heading's standard deviation (rad).
    pub floor: f64,
}

impl Default for CourseSettings {
    fn default() -> Self {
        Self {
            min_speed: 0.3,
            max_speed_gap: 0.3,
            floor: radians(2.0),
        }
    }
}

/// Python's `math.radians`: `x * (pi / 180)`.
#[must_use]
pub fn radians(x: f64) -> f64 {
    x * (PI / 180.0)
}

/// Python's `math.degrees`: `x * (180 / pi)`.
#[must_use]
pub fn degrees(x: f64) -> f64 {
    x * (180.0 / PI)
}

/// An angle in [-pi, pi), as Python's `(a + pi) % (2 pi) - pi`.
#[must_use]
pub fn wrap(angle: f64) -> f64 {
    py_mod(angle + PI, 2.0 * PI) - PI
}

/// Python's float `%`: the sign of the divisor, and +0.0 (for a positive divisor) for 0.
fn py_mod(x: f64, m: f64) -> f64 {
    let r = x % m;
    if r != 0.0 {
        if (m < 0.0) != (r < 0.0) {
            r + m
        } else {
            r
        }
    } else {
        0.0_f64.copysign(m)
    }
}

/// The direction (rad) the antenna travels in base_link, at this speed and turn rate.
#[must_use]
pub fn antenna_direction(speed: f64, turn: f64, antenna: (f64, f64)) -> f64 {
    let (x_a, y_a) = antenna;
    (turn * x_a).atan2(speed - turn * y_a)
}

/// The heading (rad) and its variance (rad^2) from the GPS's velocity, or `None`.
///
/// `east`, `north`: the antenna's velocity (m/s, ENU) and `var_east`, `var_north` their
/// variances. `speed`: the wheels' signed speed; `turn`: the turn rate (rad/s). `None`
/// when a number isn't finite, either speed is under `min_speed`, or they disagree by
/// more than `max_speed_gap`.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn heading(
    east: f64,
    north: f64,
    var_east: f64,
    var_north: f64,
    speed: f64,
    turn: f64,
    antenna: (f64, f64),
    settings: &CourseSettings,
) -> Option<(f64, f64)> {
    let ground = hypot(east, north);
    if ![east, north, var_east, var_north, speed, turn]
        .iter()
        .all(|v| v.is_finite())
        || ground < settings.min_speed
        || ground == 0.0
        || speed.abs() < settings.min_speed
        || (ground - hypot(speed - turn * antenna.1, turn * antenna.0)).abs()
            > settings.max_speed_gap
    {
        return None;
    }
    let course = north.atan2(east);
    // The noise across the direction of travel turns the course; along it, only the speed.
    let across = var_east * sin(course).powi(2) + var_north * cos(course).powi(2);
    let variance = py_max(across, 0.0) / ground.powi(2) + settings.floor.powi(2);
    Some((
        wrap(course - antenna_direction(speed, turn, antenna)),
        variance,
    ))
}

/// (x, y, z, w) of a turn by `yaw` about z.
#[must_use]
pub fn quaternion_z(yaw: f64) -> (f64, f64, f64, f64) {
    (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))
}

/// `math.sin` alone: an optimised build may join a `sin` and a `cos` of one angle into a
/// `sincos` call, which can differ in the last bit (seen on macOS). Hiding the angle keeps
/// them apart, as Python's are.
fn sin(x: f64) -> f64 {
    std::hint::black_box(x).sin()
}

/// `math.cos` alone (see [`sin`]).
fn cos(x: f64) -> f64 {
    std::hint::black_box(x).cos()
}

/// Python's `max(a, b)`: `a` unless `b > a`.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// CPython 3.12's `math.hypot(x, y)` (its own `vector_norm`, not libm's `hypot`, which
/// may differ in the last bit). The same as `aav_safety_rs`'s `pyfmt::hypot`.
#[must_use]
pub fn hypot(x: f64, y: f64) -> f64 {
    let (x, y) = (x.abs(), y.abs());
    let top = [x, y]
        .into_iter()
        .fold(0.0, |top, v| if v > top { v } else { top });
    if top.is_infinite() {
        return top;
    }
    if x.is_nan() || y.is_nan() {
        return f64::NAN;
    }
    if top == 0.0 {
        return top;
    }
    norm(&[x, y], top)
}

fn norm(values: &[f64], top: f64) -> f64 {
    let e = exponent(top);
    if e < -1023 {
        let scaled: Vec<f64> = values.iter().map(|v| v / f64::MIN_POSITIVE).collect();
        return f64::MIN_POSITIVE * norm(&scaled, top / f64::MIN_POSITIVE);
    }
    let scale = power_of_two(-e);
    let (mut csum, mut frac1, mut frac2) = (1.0_f64, 0.0_f64, 0.0_f64);
    for &v in values {
        let x = v * scale;
        let (hi, lo) = mul_exact(x, x);
        let (s_hi, s_lo) = fast_sum(csum, hi);
        csum = s_hi;
        frac1 += lo;
        frac2 += s_lo;
    }
    let mut h = (csum - 1.0 + (frac1 + frac2)).sqrt();
    let (hi, lo) = mul_exact(-h, h);
    let (s_hi, s_lo) = fast_sum(csum, hi);
    csum = s_hi;
    frac1 += lo;
    frac2 += s_lo;
    let x = csum - 1.0 + (frac1 + frac2);
    h += x / (2.0 * h);
    h / scale
}

fn exponent(v: f64) -> i32 {
    let biased = ((v.to_bits() >> 52) & 0x7ff) as i32;
    if biased == 0 {
        exponent(v * 2f64.powi(64)) - 64
    } else {
        biased - 1022
    }
}

fn power_of_two(e: i32) -> f64 {
    if e >= -1022 {
        f64::from_bits(u64::from((e + 1023).unsigned_abs()) << 52)
    } else {
        f64::from_bits(1u64 << (e + 1074).unsigned_abs())
    }
}

fn mul_exact(x: f64, y: f64) -> (f64, f64) {
    let z = x * y;
    (z, x.mul_add(y, -z))
}

fn fast_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    (x, b - (x - a))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_ahead_east_and_reversing() {
        let s = CourseSettings::default();
        let (yaw, _) = heading(1.0, 0.0, 0.01, 0.01, 1.0, 0.0, (0.5, 0.0), &s).expect("a heading");
        assert!(yaw.abs() < 1e-12);
        let (yaw, _) =
            heading(-1.0, 0.0, 0.01, 0.01, -1.0, 0.0, (0.5, 0.0), &s).expect("a heading");
        assert!(yaw.abs() < 1e-12 || (yaw + PI).abs() < 1e-12);
    }

    #[test]
    fn slow_or_disagreeing_is_none() {
        let s = CourseSettings::default();
        assert!(heading(0.1, 0.0, 0.0, 0.0, 1.0, 0.0, (0.0, 0.0), &s).is_none());
        assert!(heading(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, (0.0, 0.0), &s).is_none());
        assert!(heading(f64::NAN, 1.0, 0.0, 0.0, 1.0, 0.0, (0.0, 0.0), &s).is_none());
    }

    #[test]
    fn wrap_is_pythons() {
        assert_eq!(wrap(PI), -PI);
        assert!((wrap(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-12);
    }
}

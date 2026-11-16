//! localization_health's checks, with no ROS: the Rust twin of `aav_localization_health`'s
//! `health.py`. Warn only: nothing here stops the car or changes what the EKFs see.
//!
//! | Check | WARN when |
//! |---|---|
//! | `gyro_bias` | parked, the gyro's mean turn rate stands out (0.2 deg/s) |
//! | `imu_mounting` | parked, gravity says upside down, tilted, or wrong units |
//! | `gps_quality` | no fix, an unknown covariance, a sigma too big or too small |
//! | `timestamps` | an input's stamp too old, or from the future |
//! | `frames` | an input's `frame_id` isn't the one the EKFs expect |
//! | `tf` | `base_link -> imu_link / gps_link` missing, or `map -> base_link` old |
//! | `wheels` | the front wheels disagree with the gyro's turn |
//! | `motion` | an EKF says what the car can't do; `ekf_local` jumps |
//!
//! | Module | Holds | From |
//! |---|---|---|
//! | [`health`] | `Health`, `Settings`, `Status`, `Level`, `Check`, `Input`, `Ekf`, `summary` | `health.py` |
//! | [`tree`] | `Tree`: which frames hang from which, and how old a chain is | tf2's buffer, the part the node uses |
//! | here | `G`, `format_g` (Python's `format(x, '.4g')`, for `/diagnostics`) | |

pub mod health;
pub mod tree;

pub use ego_core::fmt_g;

/// Standard gravity, m/s².
pub const G: f64 = 9.80665;

/// Python's `math.degrees`: `x * (180 / pi)`, with 180 / pi worked out in f64 as CPython
/// does (Rust's `to_degrees` uses the exactly rounded constant, which can differ by a bit).
#[must_use]
pub fn degrees(x: f64) -> f64 {
    x * (180.0 / std::f64::consts::PI)
}

/// Python's `math.radians`: `x * (pi / 180)`.
#[must_use]
pub fn radians(x: f64) -> f64 {
    x * (std::f64::consts::PI / 180.0)
}

/// Python's `format(x, f'.{precision}g')`: `precision` significant digits, no trailing
/// zeros, an exponent (`1.2e-05`) below 1e-4 or from 10^precision up.
#[must_use]
pub fn format_g(x: f64, precision: usize) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf".into() } else { "-inf".into() };
    }
    let p = precision.max(1);
    // Round to p digits first: the exponent after rounding picks the form (as C's %g).
    let sci = format!("{:.*e}", p - 1, x);
    let (mantissa, exp) = sci.split_once('e').unwrap_or((sci.as_str(), "0"));
    let exp: i32 = exp.parse().unwrap_or(0);
    let trim = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s.to_string()
        }
    };
    if exp < -4 || exp >= p as i32 {
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{}e{sign}{:02}", trim(mantissa), exp.abs())
    } else {
        let decimals = (p as i32 - 1 - exp).max(0) as usize;
        trim(&format!("{x:.decimals$}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn g_formats_as_python() {
        let cases = [
            (0.0, "0"),
            (-0.0, "-0"),
            (1.0, "1"),
            (9.80665, "9.807"),
            (0.000012345, "1.234e-05"),
            (0.0001, "0.0001"),
            (12345.0, "1.234e+04"),
            (9999.5, "1e+04"),
            (-2.5, "-2.5"),
            (0.5, "0.5"),
            (123.45, "123.5"),
            (f64::NAN, "nan"),
        ];
        for (x, want) in cases {
            assert_eq!(format_g(x, 4), want, "{x}");
        }
    }
}

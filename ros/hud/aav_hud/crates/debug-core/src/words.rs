//! The sensor rows' words, from a message's numbers: what the debug screen's panel and the
//! world view's Sensors page say for the IMU, the GPS, the wheels and the wheel odometry.
//! Both screens read the same messages, so both say the same thing (wrap-up step 4).

/// The IMU's row: its yaw rate (rad/s in, degrees a second out).
#[must_use]
pub fn imu(yaw_rate: f64) -> String {
    format!("yaw rate {:+.1} deg/s", yaw_rate.to_degrees())
}

/// The GPS's row: the fix's kind (sensor_msgs/NavSatStatus' `status`) and its horizontal
/// spread, from the position covariance's east and north variances (m²).
#[must_use]
pub fn gps(status: i8, east_variance: f64, north_variance: f64) -> String {
    let kind = match status {
        s if s < 0 => "no fix",
        0 => "fix",
        1 => "SBAS fix",
        _ => "RTK / GBAS fix",
    };
    let spread = east_variance.max(north_variance);
    if spread.is_finite() && spread > 0.0 {
        format!("{kind}, {:.1} m", spread.sqrt())
    } else {
        kind.to_owned()
    }
}

/// The wheels' row: the measured wheels' mean speed. A wheel that isn't measured is NaN
/// (aav_interfaces/WheelSpeeds) and left out.
#[must_use]
pub fn wheels(speeds: [f32; 4]) -> String {
    let measured: Vec<f32> = speeds.into_iter().filter(|v| v.is_finite()).collect();
    if measured.is_empty() {
        return "none measured".to_owned();
    }
    let mean = measured.iter().sum::<f32>() / measured.len() as f32;
    format!("{mean:+.2} m/s ({} wheels)", measured.len())
}

/// A speed's row (the wheel odometry's): m/s forward.
#[must_use]
pub fn speed(forward: f64) -> String {
    format!("{forward:+.2} m/s")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fix_says_its_kind() {
        assert_eq!(gps(-1, 0.0, 0.0), "no fix");
        assert_eq!(gps(0, 4.0, 1.0), "fix, 2.0 m");
        assert_eq!(gps(2, f64::NAN, 0.0), "RTK / GBAS fix");
    }

    #[test]
    fn unmeasured_wheels_are_left_out() {
        assert_eq!(
            wheels([1.0, 2.0, f32::NAN, f32::NAN]),
            "+1.50 m/s (2 wheels)"
        );
        assert_eq!(wheels([f32::NAN; 4]), "none measured");
    }

    #[test]
    fn the_imu_and_a_speed_in_words() {
        assert_eq!(imu(std::f64::consts::FRAC_PI_2), "yaw rate +90.0 deg/s");
        assert_eq!(speed(-0.5), "-0.50 m/s");
    }
}

//! Numbers turned into the short text the screen shows.

use std::time::Duration;

use crate::{health::Health, units::MetresPerSecond};

/// Speed in whole km/h, without a sign: "0", "12". The gear letter says the direction.
#[must_use]
pub fn speed_km_per_hour(speed: MetresPerSecond) -> String {
    format!("{:.0}", speed.to_km_per_hour().abs())
}

/// How long ago: "0.3 s", "12 s", "4 min", "2 h".
#[must_use]
pub fn age(age: Duration) -> String {
    let s = age.as_secs_f64();
    if s < 10.0 {
        format!("{s:.1} s")
    } else if s < 120.0 {
        format!("{s:.0} s")
    } else if s < 7200.0 {
        format!("{:.0} min", s / 60.0)
    } else {
        format!("{:.0} h", s / 3600.0)
    }
}

/// A topic's line: "OK 0.1 s", "STALE 12 s", "NEVER".
#[must_use]
pub fn health(health: Health) -> String {
    match health.age() {
        Some(a) => format!("{} {}", health.word(), age(a)),
        None => health.word().to_owned(),
    }
}

/// Latitude or longitude: 6 decimals (about 0.1 m), with N/S or E/W.
#[must_use]
pub fn latitude(deg: f64) -> String {
    format!("{:.6} {}", deg.abs(), if deg < 0.0 { 'S' } else { 'N' })
}

/// See [`latitude`].
#[must_use]
pub fn longitude(deg: f64) -> String {
    format!("{:.6} {}", deg.abs(), if deg < 0.0 { 'W' } else { 'E' })
}

/// A distance in metres, sized to its value: "0.02 m", "1.5 m", "12 m".
#[must_use]
pub fn metres(m: f64) -> String {
    if m < 1.0 {
        format!("{m:.2} m")
    } else if m < 10.0 {
        format!("{m:.1} m")
    } else {
        format!("{m:.0} m")
    }
}

/// A steering angle in degrees with L or R: "12° L". Positive turns left (REP-103).
/// raylib's built-in font has no degree sign, so it says "deg".
#[must_use]
pub fn steering(deg: f32) -> String {
    let side = if deg > 0.5 {
        " L"
    } else if deg < -0.5 {
        " R"
    } else {
        ""
    };
    format!("{:.0} deg{side}", deg.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speeds() {
        assert_eq!(speed_km_per_hour(MetresPerSecond(0.0)), "0");
        assert_eq!(speed_km_per_hour(MetresPerSecond(10.0)), "36");
        assert_eq!(speed_km_per_hour(MetresPerSecond(-1.0)), "4");
    }

    #[test]
    fn ages() {
        assert_eq!(age(Duration::from_millis(340)), "0.3 s");
        assert_eq!(age(Duration::from_secs(12)), "12 s");
        assert_eq!(age(Duration::from_secs(300)), "5 min");
        assert_eq!(age(Duration::from_secs(3 * 3600)), "3 h");
    }

    #[test]
    fn health_lines() {
        assert_eq!(health(Health::Never), "NEVER");
        assert_eq!(health(Health::Ok(Duration::from_millis(100))), "OK 0.1 s");
        assert_eq!(health(Health::Stale(Duration::from_secs(12))), "STALE 12 s");
    }

    #[test]
    fn coordinates() {
        assert_eq!(latitude(45.3876), "45.387600 N");
        assert_eq!(longitude(-75.696), "75.696000 W");
    }

    #[test]
    fn distances_and_angles() {
        assert_eq!(metres(0.023), "0.02 m");
        assert_eq!(metres(1.54), "1.5 m");
        assert_eq!(metres(12.4), "12 m");
        assert_eq!(steering(12.3), "12 deg L");
        assert_eq!(steering(-3.0), "3 deg R");
        assert_eq!(steering(0.1), "0 deg");
    }
}

//! A GPS fix (sensor_msgs/msg/NavSatFix) in plain words: is there a position, and how
//! good is it?
//!
//! NavSatFix says less than a receiver knows. What each kind means here:
//!
//! | Kind | From NavSatFix |
//! |---|---|
//! | NO FIX | status -1 (NO_FIX), or a position that is NaN or off the globe |
//! | FIX ? | status -2 (UNKNOWN, Jazzy's default): a position, but the driver didn't say how good |
//! | 2D | status 0 (FIX) and no altitude (NaN) |
//! | 3D | status 0 (FIX) with an altitude |
//! | DGPS | status 1 (SBAS: satellite corrections) |
//! | RTK FLOAT, RTK FIXED | status 2 (GBAS: ground corrections). NavSatFix can't tell float from fixed, so we guess from the accuracy: 5 cm or better = fixed |

use std::fmt;

/// Better than this (1 sigma, horizontal) with ground corrections counts as RTK fixed.
pub const RTK_FIXED_SIGMA_M: f64 = 0.05;

/// NavSatStatus's `status` numbers.
const STATUS_UNKNOWN: i8 = -2;
const STATUS_NO_FIX: i8 = -1;
const STATUS_FIX: i8 = 0;
const STATUS_SBAS_FIX: i8 = 1;
const STATUS_GBAS_FIX: i8 = 2;
/// NavSatFix's `position_covariance_type` for "no covariance given".
const COVARIANCE_TYPE_UNKNOWN: u8 = 0;

/// How good a fix is, from worst to best.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FixKind {
    /// No position at all.
    NoFix,
    /// A position, but the driver didn't say what kind.
    Unknown,
    /// A fix without height.
    Fix2d,
    /// A fix with height.
    Fix3d,
    /// Helped by corrections (SBAS or DGPS).
    Dgps,
    /// RTK, not yet settled: decimetres.
    RtkFloat,
    /// RTK, settled: centimetres.
    RtkFixed,
}

impl FixKind {
    /// Whether there is a position to show.
    #[must_use]
    pub fn has_position(self) -> bool {
        self != Self::NoFix
    }
}

impl fmt::Display for FixKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NoFix => "NO FIX",
            Self::Unknown => "FIX ?",
            Self::Fix2d => "2D FIX",
            Self::Fix3d => "3D FIX",
            Self::Dgps => "DGPS",
            Self::RtkFloat => "RTK FLOAT",
            Self::RtkFixed => "RTK FIXED",
        })
    }
}

/// The fields of one NavSatFix that the HUD uses, as the message has them.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RawFix {
    /// NavSatStatus `status`: -2 unknown, -1 no fix, 0 fix, 1 SBAS, 2 GBAS.
    pub status: i8,
    /// Degrees, + north.
    pub latitude: f64,
    /// Degrees, + east.
    pub longitude: f64,
    /// Metres above the WGS 84 ellipsoid; NaN when the receiver has none.
    pub altitude: f64,
    /// Row-major 3x3, east-north-up, m^2.
    pub position_covariance: [f64; 9],
    /// 0 unknown, 1 approximated, 2 diagonal known, 3 known.
    pub position_covariance_type: u8,
}

/// A fix, judged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    /// How good it is.
    pub kind: FixKind,
    /// (latitude, longitude) in degrees, when there is a position.
    pub position: Option<(f64, f64)>,
    /// Metres; None without an altitude.
    pub altitude: Option<f64>,
    /// 1-sigma horizontal accuracy in metres; None when the driver gives no covariance.
    pub horizontal_sigma: Option<f64>,
}

impl From<RawFix> for Fix {
    fn from(raw: RawFix) -> Self {
        let position = valid_position(raw.latitude, raw.longitude);
        let altitude = raw.altitude.is_finite().then_some(raw.altitude);
        let horizontal_sigma = horizontal_sigma(&raw);
        let kind = match (raw.status, position) {
            (STATUS_NO_FIX, _) | (_, None) => FixKind::NoFix,
            (STATUS_FIX, _) if altitude.is_none() => FixKind::Fix2d,
            (STATUS_FIX, _) => FixKind::Fix3d,
            (STATUS_SBAS_FIX, _) => FixKind::Dgps,
            (STATUS_GBAS_FIX, _) => match horizontal_sigma {
                Some(s) if s <= RTK_FIXED_SIGMA_M => FixKind::RtkFixed,
                _ => FixKind::RtkFloat,
            },
            (STATUS_UNKNOWN, _) | (_, Some(_)) => FixKind::Unknown,
        };
        Self {
            kind,
            position: position.filter(|_| kind.has_position()),
            altitude,
            horizontal_sigma,
        }
    }
}

/// The position, or None when it's NaN or off the globe (the map would jump to 0, 0).
fn valid_position(lat: f64, lon: f64) -> Option<(f64, f64)> {
    let ok = lat.is_finite() && lon.is_finite() && lat.abs() <= 90.0 && lon.abs() <= 180.0;
    ok.then_some((lat, lon))
}

/// The mean of the east and north variances, as a standard deviation.
fn horizontal_sigma(raw: &RawFix) -> Option<f64> {
    if raw.position_covariance_type == COVARIANCE_TYPE_UNKNOWN {
        return None;
    }
    let (east, north) = (raw.position_covariance[0], raw.position_covariance[4]);
    let mean = (east + north) / 2.0;
    (mean.is_finite() && mean >= 0.0).then(|| mean.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw(status: i8) -> RawFix {
        RawFix {
            status,
            latitude: 45.3876,
            longitude: -75.6960,
            altitude: 80.0,
            ..RawFix::default()
        }
    }

    fn with_sigma(mut r: RawFix, sigma: f64) -> RawFix {
        r.position_covariance[0] = sigma * sigma;
        r.position_covariance[4] = sigma * sigma;
        r.position_covariance_type = 2;
        r
    }

    #[test]
    fn kinds() {
        assert_eq!(Fix::from(raw(-1)).kind, FixKind::NoFix);
        assert_eq!(Fix::from(raw(-2)).kind, FixKind::Unknown);
        assert_eq!(Fix::from(raw(0)).kind, FixKind::Fix3d);
        assert_eq!(Fix::from(raw(1)).kind, FixKind::Dgps);
        assert_eq!(Fix::from(raw(2)).kind, FixKind::RtkFloat);
        assert_eq!(Fix::from(with_sigma(raw(2), 0.02)).kind, FixKind::RtkFixed);
        assert_eq!(Fix::from(with_sigma(raw(2), 0.3)).kind, FixKind::RtkFloat);
        let mut flat = raw(0);
        flat.altitude = f64::NAN;
        assert_eq!(Fix::from(flat).kind, FixKind::Fix2d);
    }

    #[test]
    fn no_fix_has_no_position() {
        assert_eq!(Fix::from(raw(-1)).position, None);
        assert_eq!(Fix::from(raw(0)).position, Some((45.3876, -75.6960)));
    }

    #[test]
    fn bad_positions_are_no_fix() {
        let mut r = raw(0);
        r.latitude = f64::NAN;
        assert_eq!(Fix::from(r).kind, FixKind::NoFix);
        r.latitude = 91.0;
        assert_eq!(Fix::from(r).kind, FixKind::NoFix);
    }

    #[test]
    fn accuracy_only_when_given() {
        assert_eq!(Fix::from(raw(0)).horizontal_sigma, None);
        let s = Fix::from(with_sigma(raw(0), 1.5)).horizontal_sigma;
        assert!(s.is_some_and(|s| (s - 1.5).abs() < 1e-9));
    }

    #[test]
    fn kinds_order_worst_to_best() {
        assert!(FixKind::NoFix < FixKind::Fix3d);
        assert!(FixKind::RtkFloat < FixKind::RtkFixed);
        assert_eq!(FixKind::RtkFixed.to_string(), "RTK FIXED");
    }
}

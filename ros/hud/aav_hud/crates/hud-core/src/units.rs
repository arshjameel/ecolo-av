//! Small types for units, so a speed in m/s can't be shown as km/h by mistake.

/// A speed in metres per second (REP-103). Negative in reverse.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct MetresPerSecond(pub f32);

impl MetresPerSecond {
    /// The same speed in kilometres per hour.
    #[must_use]
    pub fn to_km_per_hour(self) -> f32 {
        self.0 * 3.6
    }
}

/// An angle in radians. For steering, positive turns left (REP-103).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Radians(pub f32);

impl Radians {
    /// The same angle in degrees, for people.
    #[must_use]
    pub fn to_degrees(self) -> f32 {
        self.0.to_degrees()
    }
}

/// A level from 0 to 1, such as throttle or brake. Kept inside 0..=1.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Fraction(f32);

impl Fraction {
    /// A level, clamped to 0..=1. NaN counts as 0: nothing measured, nothing shown.
    #[must_use]
    pub fn new(value: f32) -> Self {
        if value.is_nan() {
            Self(0.0)
        } else {
            Self(value.clamp(0.0, 1.0))
        }
    }

    /// The level, from 0 to 1.
    #[must_use]
    pub fn get(self) -> f32 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metres_per_second_to_km_per_hour() {
        assert!((MetresPerSecond(10.0).to_km_per_hour() - 36.0).abs() < 1e-4);
        assert!((MetresPerSecond(-1.0).to_km_per_hour() + 3.6).abs() < 1e-4);
    }

    #[test]
    fn fraction_stays_between_zero_and_one() {
        assert_eq!(Fraction::new(1.5).get(), 1.0);
        assert_eq!(Fraction::new(-0.2).get(), 0.0);
        assert_eq!(Fraction::new(f32::NAN).get(), 0.0);
        assert_eq!(Fraction::new(0.25).get(), 0.25);
    }

    #[test]
    fn radians_to_degrees() {
        assert!((Radians(std::f32::consts::PI).to_degrees() - 180.0).abs() < 1e-3);
    }
}

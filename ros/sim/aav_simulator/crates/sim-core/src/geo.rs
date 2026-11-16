//! The world's origin on the Earth, and world positions as latitude, longitude and height.
//!
//! As Gazebo's `<spherical_coordinates>` with ENU and heading 0: world x east, y north,
//! z up, from the datum. Exact on the WGS84 ellipsoid (through Earth-centred coordinates),
//! not a flat-Earth guess, so the campus's far corners come out as Gazebo's do.

use crate::math::Vec3;

const A: f64 = 6_378_137.0; // WGS84: the equator's radius, m
const F: f64 = 1.0 / 298.257_223_563; // flattening
const E2: f64 = F * (2.0 - F); // eccentricity squared

/// Where the world's origin is: navsat.yaml's datum, and the world file's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Datum {
    pub latitude: f64,  // degrees
    pub longitude: f64, // degrees
    pub elevation: f64, // m above the ellipsoid
}

/// A place on the Earth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geodetic {
    pub latitude: f64,  // degrees
    pub longitude: f64, // degrees
    pub altitude: f64,  // m
}

impl Datum {
    /// The test lot's, the road's and navsat.yaml's: by the campus, 70 m up.
    pub const CARLETON: Datum = Datum {
        latitude: 45.3860,
        longitude: -75.6960,
        elevation: 70.0,
    };

    /// A world position (m east, north, up of the datum) on the Earth.
    #[must_use]
    pub fn to_geodetic(&self, enu: Vec3) -> Geodetic {
        let (lat0, lon0) = (self.latitude.to_radians(), self.longitude.to_radians());
        let origin = ecef(lat0, lon0, self.elevation);
        let (sl, cl) = lat0.sin_cos();
        let (so, co) = lon0.sin_cos();
        // ENU into Earth-centred axes.
        let d = Vec3::new(
            -so * enu.x - sl * co * enu.y + cl * co * enu.z,
            co * enu.x - sl * so * enu.y + cl * so * enu.z,
            cl * enu.y + sl * enu.z,
        );
        geodetic(origin + d)
    }
}

fn ecef(lat: f64, lon: f64, h: f64) -> Vec3 {
    let (sl, cl) = lat.sin_cos();
    let n = A / (1.0 - E2 * sl * sl).sqrt();
    Vec3::new(
        (n + h) * cl * lon.cos(),
        (n + h) * cl * lon.sin(),
        (n * (1.0 - E2) + h) * sl,
    )
}

/// Earth-centred back to latitude, longitude, height: Bowring's start, then a few rounds.
fn geodetic(p: Vec3) -> Geodetic {
    let lon = p.y.atan2(p.x);
    let r = (p.x * p.x + p.y * p.y).sqrt();
    let mut lat = p.z.atan2(r * (1.0 - E2));
    let mut h = 0.0;
    for _ in 0..6 {
        let sl = lat.sin();
        let n = A / (1.0 - E2 * sl * sl).sqrt();
        h = r / lat.cos() - n;
        lat = p.z.atan2(r * (1.0 - E2 * n / (n + h)));
    }
    Geodetic {
        latitude: lat.to_degrees(),
        longitude: lon.to_degrees(),
        altitude: h,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_origin_is_the_datum() {
        let g = Datum::CARLETON.to_geodetic(Vec3::ZERO);
        assert!((g.latitude - 45.386).abs() < 1e-10);
        assert!((g.longitude + 75.696).abs() < 1e-10);
        assert!((g.altitude - 70.0).abs() < 1e-6);
    }

    #[test]
    fn a_hundred_metres_north_and_east() {
        // A degree's length there, from WGS84's two radii of curvature.
        let s2 = 45.386f64.to_radians().sin().powi(2);
        let meridian = A * (1.0 - E2) / (1.0 - E2 * s2).powf(1.5);
        let normal = A / (1.0 - E2 * s2).sqrt();
        let g = Datum::CARLETON.to_geodetic(Vec3::new(100.0, 100.0, 0.0));
        let north = (g.latitude - 45.386).to_radians() * meridian;
        let east = (g.longitude + 75.696).to_radians() * normal * 45.386f64.to_radians().cos();
        assert!((north - 100.0).abs() < 0.05, "north {north}");
        assert!((east - 100.0).abs() < 0.05, "east {east}");
        // The Earth curves away under a tangent plane: 141 m out, about 1.6 mm down.
        assert!((g.altitude - 70.0).abs() < 0.01);
    }
}

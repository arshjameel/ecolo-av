//! Latitude and longitude to the map frame, and back, exactly as navsat_transform does it.
//!
//! navsat_transform (robot_localization, `use_local_cartesian: false`, the default) puts a
//! GPS fix on the map like this:
//!
//! 1. lat/lon to UTM (GeographicLib), in the datum's UTM zone;
//! 2. take away the datum's UTM point, so the datum is map (0, 0);
//! 3. turn by minus the meridian convergence at the datum (and the datum's yaw, 0 here),
//!    so the map's y axis points to true north at the datum, not to UTM's grid north.
//!
//! [`Datum`] does the same steps, so a street drawn from OpenStreetMap lands where
//! navsat_transform puts the car. UTM here is the Transverse Mercator projection with
//! Krüger's series to the third order (Karney 2011): better than a millimetre within a
//! UTM zone. The tests check it against GeographicLib's `GeoConvert`.

use std::f64::consts::PI;

use serde::Deserialize;
use thiserror::Error;

use crate::geom::Point;

/// WGS 84: the GPS ellipsoid. Equatorial radius (m) and flattening.
const A: f64 = 6_378_137.0;
const F: f64 = 1.0 / 298.257_223_563;
/// UTM's scale on the central meridian, and its false easting and northing (m).
const K0: f64 = 0.9996;
const FALSE_EASTING: f64 = 500_000.0;
const FALSE_NORTHING_SOUTH: f64 = 10_000_000.0;

/// A place on the Earth, in degrees (WGS 84).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LatLon {
    /// Degrees north.
    pub lat: f64,
    /// Degrees east.
    pub lon: f64,
}

impl LatLon {
    /// A place from its latitude and longitude (degrees).
    #[must_use]
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// On the globe: finite, |lat| <= 90, |lon| <= 180.
    #[must_use]
    pub fn is_valid(self) -> bool {
        self.lat.is_finite()
            && self.lon.is_finite()
            && self.lat.abs() <= 90.0
            && self.lon.abs() <= 180.0
    }
}

/// A UTM zone: its number (1 to 60) and its hemisphere.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UtmZone {
    /// 1 to 60.
    pub number: u8,
    /// North of the equator.
    pub north: bool,
}

impl UtmZone {
    /// The standard zone of a place (6 degrees wide; Norway's and Svalbard's exceptions
    /// are left out: they are far from Ottawa).
    #[must_use]
    pub fn of(p: LatLon) -> Self {
        let number = (((p.lon + 180.0) / 6.0).floor() as i32).clamp(0, 59) + 1;
        Self {
            number: number as u8,
            north: p.lat >= 0.0,
        }
    }

    /// The zone's central meridian, in radians.
    fn central_meridian(self) -> f64 {
        (f64::from(self.number) * 6.0 - 183.0).to_radians()
    }

    fn false_northing(self) -> f64 {
        if self.north {
            0.0
        } else {
            FALSE_NORTHING_SOUTH
        }
    }
}

/// A point in UTM, with the projection's local properties there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Utm {
    /// The zone.
    pub zone: UtmZone,
    /// m east, from the zone's false origin.
    pub easting: f64,
    /// m north, from the equator (or its false northing in the south).
    pub northing: f64,
    /// The meridian convergence, radians: the bearing of grid north, clockwise from true
    /// north (GeographicLib's gamma).
    pub convergence: f64,
}

/// Krüger's series' numbers for WGS 84, computed once.
struct Series {
    /// The rectifying radius times K0.
    k0a: f64,
    /// The first eccentricity.
    e: f64,
    alpha: [f64; 3],
    beta: [f64; 3],
    delta: [f64; 3],
}

impl Series {
    fn wgs84() -> Self {
        let n = F / (2.0 - F);
        let (n2, n3) = (n * n, n * n * n);
        let rectifying = A / (1.0 + n) * (1.0 + n2 / 4.0 + n2 * n2 / 64.0);
        Self {
            k0a: K0 * rectifying,
            e: (F * (2.0 - F)).sqrt(),
            alpha: [
                n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0,
                13.0 * n2 / 48.0 - 3.0 * n3 / 5.0,
                61.0 * n3 / 240.0,
            ],
            beta: [
                n / 2.0 - 2.0 * n2 / 3.0 + 37.0 * n3 / 96.0,
                n2 / 48.0 + n3 / 15.0,
                17.0 * n3 / 480.0,
            ],
            delta: [
                2.0 * n - 2.0 * n2 / 3.0 - 2.0 * n3,
                7.0 * n2 / 3.0 - 8.0 * n3 / 5.0,
                56.0 * n3 / 15.0,
            ],
        }
    }
}

/// A place in UTM, in a given zone (navsat_transform keeps the datum's zone).
#[must_use]
pub fn to_utm(p: LatLon, zone: UtmZone) -> Utm {
    let s = Series::wgs84();
    let phi = p.lat.to_radians();
    let lam = p.lon.to_radians() - zone.central_meridian();
    // The conformal latitude, as tan.
    let t = (phi.sin().atanh() - s.e * (s.e * phi.sin()).atanh()).sinh();
    let xi1 = t.atan2(lam.cos());
    let eta1 = (lam.sin() / (1.0 + t * t).sqrt()).atanh();
    let (mut xi, mut eta, mut sigma, mut tau) = (xi1, eta1, 1.0, 0.0);
    for (j, a) in (1..).zip(s.alpha) {
        let k = 2.0 * f64::from(j);
        xi += a * (k * xi1).sin() * (k * eta1).cosh();
        eta += a * (k * xi1).cos() * (k * eta1).sinh();
        sigma += k * a * (k * xi1).cos() * (k * eta1).cosh();
        tau += k * a * (k * xi1).sin() * (k * eta1).sinh();
    }
    let (txi, theta) = (xi1.tan(), eta1.tanh());
    let gamma = (tau + sigma * txi * theta).atan2(sigma - tau * txi * theta);
    Utm {
        zone,
        easting: FALSE_EASTING + s.k0a * eta,
        northing: zone.false_northing() + s.k0a * xi,
        convergence: gamma,
    }
}

/// The opposite of [`to_utm`].
#[must_use]
pub fn from_utm(easting: f64, northing: f64, zone: UtmZone) -> LatLon {
    let s = Series::wgs84();
    let xi = (northing - zone.false_northing()) / s.k0a;
    let eta = (easting - FALSE_EASTING) / s.k0a;
    let (mut xi1, mut eta1) = (xi, eta);
    for (j, b) in (1..).zip(s.beta) {
        let k = 2.0 * f64::from(j);
        xi1 -= b * (k * xi).sin() * (k * eta).cosh();
        eta1 -= b * (k * xi).cos() * (k * eta).sinh();
    }
    let chi = (xi1.sin() / eta1.cosh()).asin();
    let phi = chi
        + (1..)
            .zip(s.delta)
            .map(|(j, d)| d * (2.0 * f64::from(j) * chi).sin())
            .sum::<f64>();
    let lam = eta1.sinh().atan2(xi1.cos());
    LatLon::new(
        phi.to_degrees(),
        (zone.central_meridian() + lam).to_degrees(),
    )
}

/// Why a datum can't be used.
#[derive(Debug, Error)]
pub enum DatumError {
    /// The text isn't a navsat_transform parameters file with a datum.
    #[error("not a navsat_transform parameters file with a datum: {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    /// The datum isn't three numbers.
    #[error("the datum must be [lat, lon, yaw]: {0:?}")]
    Shape(Vec<f64>),
    /// The datum isn't a place on the Earth.
    #[error("the datum isn't on the globe: {0:?}")]
    OffTheGlobe(LatLon),
}

/// navsat_transform's parameters (aav_localization's `config/navsat.yaml`), built in:
/// the datum when no `--navsat` file is given.
pub const BUILT_IN_NAVSAT_YAML: &str =
    include_str!("../../../../../localization/aav_localization/config/navsat.yaml");

/// The map frame's anchor on the Earth: map (0, 0) is the datum, x east, y north.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Datum {
    /// Map (0, 0) on the Earth.
    pub origin: LatLon,
    zone: UtmZone,
    easting: f64,
    northing: f64,
    /// How far the map frame is turned from UTM's grid, radians, counter-clockwise: the
    /// convergence plus the datum's own yaw, as navsat_transform does.
    turn: f64,
}

#[derive(Deserialize)]
struct NavsatFile {
    navsat_transform: NavsatNode,
}

#[derive(Deserialize)]
struct NavsatNode {
    #[serde(rename = "ros__parameters")]
    params: NavsatParams,
}

#[derive(Deserialize)]
struct NavsatParams {
    datum: Vec<f64>,
}

impl Datum {
    /// A datum at `origin`, with the map's x axis east (yaw 0).
    ///
    /// # Errors
    ///
    /// [`DatumError::OffTheGlobe`] if `origin` isn't a place on the Earth.
    pub fn new(origin: LatLon) -> Result<Self, DatumError> {
        Self::with_yaw(origin, 0.0)
    }

    /// A datum with navsat_transform's third number, the yaw (radians): it turns the map
    /// frame. Our navsat.yaml keeps it 0.
    ///
    /// # Errors
    ///
    /// [`DatumError::OffTheGlobe`] if `origin` isn't a place on the Earth, or `yaw` isn't a number.
    pub fn with_yaw(origin: LatLon, yaw: f64) -> Result<Self, DatumError> {
        if !(origin.is_valid() && yaw.is_finite()) {
            return Err(DatumError::OffTheGlobe(origin));
        }
        let zone = UtmZone::of(origin);
        let u = to_utm(origin, zone);
        Ok(Self {
            origin,
            zone,
            easting: u.easting,
            northing: u.northing,
            // navsat_transform: imu_yaw += utm_meridian_convergence (GeographicLib's
            // gamma, in radians), and the map is that pose's inverse.
            turn: yaw + u.convergence,
        })
    }

    /// From navsat_transform's parameters file (`navsat.yaml`):
    /// `navsat_transform: ros__parameters: datum: [lat, lon, yaw]`.
    ///
    /// # Errors
    ///
    /// [`DatumError`] if the file has no datum, it isn't three numbers, or it isn't on the Earth.
    pub fn from_navsat_yaml(text: &str) -> Result<Self, DatumError> {
        let file: NavsatFile = serde_yaml_ng::from_str(text)?;
        match file.navsat_transform.params.datum.as_slice() {
            &[lat, lon, yaw] => Self::with_yaw(LatLon::new(lat, lon), yaw),
            &[lat, lon] => Self::new(LatLon::new(lat, lon)),
            other => Err(DatumError::Shape(other.to_vec())),
        }
    }

    /// A place on the map, in metres from the datum.
    #[must_use]
    pub fn to_map(&self, p: LatLon) -> Point {
        let u = to_utm(p, self.zone);
        Point::new(u.easting - self.easting, u.northing - self.northing).rotated(-self.turn)
    }

    /// A map point on the Earth: the opposite of [`Datum::to_map`].
    #[must_use]
    pub fn to_lat_lon(&self, p: Point) -> LatLon {
        let grid = p.rotated(self.turn);
        from_utm(grid.x + self.easting, grid.y + self.northing, self.zone)
    }
}

/// Web Mercator (the tiles' projection): x and y from 0 to 1 across the world, y down.
#[must_use]
pub fn to_web_mercator(p: LatLon) -> (f64, f64) {
    let lat = p.lat.clamp(-85.051_128_78, 85.051_128_78).to_radians();
    let x = (p.lon + 180.0) / 360.0;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0;
    (x, y)
}

/// The opposite of [`to_web_mercator`].
#[must_use]
pub fn from_web_mercator(x: f64, y: f64) -> LatLon {
    let lat = (PI * (1.0 - 2.0 * y)).sinh().atan().to_degrees();
    LatLon::new(lat, x * 360.0 - 180.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Our navsat.yaml's datum: roughly Carleton University.
    const DATUM: LatLon = LatLon::new(45.3860, -75.6960);

    /// From GeographicLib 2.x (`GeoConvert -u -p 4` and `GeoConvert -c -p 6`): the
    /// library navsat_transform calls.
    const GEOCONVERT: [(LatLon, u8, bool, f64, f64, f64); 5] = [
        (DATUM, 18, true, 445514.1385, 5026067.1450, -0.49546284270),
        (
            LatLon::new(45.4215, -75.6972),
            18,
            true,
            445454.3720,
            5030011.8344,
            -0.49662041848,
        ),
        (
            LatLon::new(45.2, -76.0),
            18,
            true,
            421459.4012,
            5005654.6176,
            -0.70960687062,
        ),
        (LatLon::new(0.0, -75.0), 18, true, 500000.0, 0.0, 0.0),
        (
            LatLon::new(-33.9, 151.2),
            56,
            false,
            333568.9410,
            6247473.3368,
            1.00417196344,
        ),
    ];

    #[test]
    fn utm_matches_geographiclib_to_a_millimetre() {
        for (p, number, north, e, n, gamma) in GEOCONVERT {
            let zone = UtmZone::of(p);
            assert_eq!(zone, UtmZone { number, north }, "{p:?}");
            let u = to_utm(p, zone);
            assert!((u.easting - e).abs() < 1e-3, "{p:?}: E {}", u.easting);
            assert!((u.northing - n).abs() < 1e-3, "{p:?}: N {}", u.northing);
            let got = u.convergence.to_degrees();
            assert!((got - gamma).abs() < 1e-8, "{p:?}: gamma {got}");
        }
    }

    #[test]
    fn utm_back_to_lat_lon() {
        for (p, ..) in GEOCONVERT {
            let u = to_utm(p, UtmZone::of(p));
            let back = from_utm(u.easting, u.northing, u.zone);
            // 1e-8 degrees is about a millimetre: the third-order series.
            assert!(
                (back.lat - p.lat).abs() < 1e-8 && (back.lon - p.lon).abs() < 1e-8,
                "{p:?}: {back:?}"
            );
        }
    }

    /// The map frame, as navsat_transform makes it: the UTM offset from the datum,
    /// turned by the convergence. Expected values from GeographicLib's numbers above.
    #[test]
    fn the_map_frame_matches_navsat_transform() {
        let d = Datum::new(DATUM).expect("a valid datum");
        assert_eq!(d.to_map(DATUM), Point::new(0.0, 0.0));
        let cases = [
            // Carleton University, Parliament Hill (4 km north), the extract's corners.
            (LatLon::new(45.3835, -75.6974), -109.6068, -277.7464),
            (LatLon::new(45.4215, -75.6972), -93.8754, 3944.0251),
            (LatLon::new(45.2, -76.0), -23877.3236, -20619.7739),
            (LatLon::new(45.55, -75.4), 23105.7591, 18262.5286),
        ];
        for (p, x, y) in cases {
            let m = d.to_map(p);
            assert!(
                (m.x - x).abs() < 0.01 && (m.y - y).abs() < 0.01,
                "{p:?}: {m:?}"
            );
            let back = d.to_lat_lon(m);
            assert!((back.lat - p.lat).abs() < 1e-8 && (back.lon - p.lon).abs() < 1e-8);
        }
    }

    /// The convergence matters: without the turn, Parliament Hill (4 km away) would be
    /// about 34 m off. With it, north on the map is true north at the datum.
    #[test]
    fn north_is_true_north_at_the_datum() {
        let d = Datum::new(DATUM).expect("a valid datum");
        let north = d.to_map(LatLon::new(DATUM.lat + 0.01, DATUM.lon));
        assert!(north.x.abs() < 0.01, "{north:?}");
        let u = to_utm(LatLon::new(45.4215, -75.6972), UtmZone::of(DATUM));
        let unturned = Point::new(u.easting - 445514.1385, u.northing - 5026067.1450);
        let turned = d.to_map(LatLon::new(45.4215, -75.6972));
        assert!(unturned.distance(turned).0 > 30.0);
    }

    #[test]
    fn the_datum_comes_from_navsat_yaml() {
        let d = Datum::from_navsat_yaml(BUILT_IN_NAVSAT_YAML).expect("navsat.yaml has a datum");
        assert_eq!(d.origin, DATUM);
        let bad = "navsat_transform:\n  ros__parameters:\n    datum: [45.0]\n";
        assert!(matches!(
            Datum::from_navsat_yaml(bad),
            Err(DatumError::Shape(_))
        ));
        let off = "navsat_transform:\n  ros__parameters:\n    datum: [95.0, 0.0, 0.0]\n";
        assert!(matches!(
            Datum::from_navsat_yaml(off),
            Err(DatumError::OffTheGlobe(_))
        ));
        assert!(Datum::from_navsat_yaml("ekf: {}").is_err());
    }

    #[test]
    fn web_mercator_there_and_back() {
        assert_eq!(to_web_mercator(LatLon::new(0.0, 0.0)), (0.5, 0.5));
        let p = LatLon::new(45.3835, -75.6974);
        let (x, y) = to_web_mercator(p);
        let back = from_web_mercator(x, y);
        assert!((back.lat - p.lat).abs() < 1e-10 && (back.lon - p.lon).abs() < 1e-10);
        // North is up: smaller y.
        assert!(to_web_mercator(LatLon::new(46.0, p.lon)).1 < y);
    }
}

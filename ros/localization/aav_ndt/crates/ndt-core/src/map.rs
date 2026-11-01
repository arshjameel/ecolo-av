//! The map as bell curves: one per cube with enough points, its mean and covariance.

use std::{collections::HashMap, error::Error, fmt};

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use crate::Point;

/// A cube's place in the grid.
pub type Key = [i32; 3];

/// The six cubes that share a face with a cube, and the cube itself (Magnusson 2009:
/// enough for a point near an edge to feel the curve next door, at a quarter of the 27).
const NEIGHBOURS: [Key; 7] = [
    [0, 0, 0],
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
];

/// How the cubes are made.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MapSettings {
    /// The side of a cube (m).
    pub cube: f64,
    /// A cube needs at least this many points for a curve (fewer can't give a spread).
    pub min_points: usize,
    /// No spread narrower than this share of the cube's widest (a wall's curve is flat:
    /// the paper's fix, as `tools/ndt_demo.py`).
    pub eigen_ratio: f64,
    /// And none narrower than this (m², a 1 cm standard deviation by default), so a cube of
    /// identical points still has an inverse.
    pub eigen_min: f64,
}

impl MapSettings {
    /// Cubes of `cube` m, with the design's floors: 6 points, 1 %, (1 cm)².
    pub fn with_cube(cube: f64) -> Self {
        Self {
            cube,
            min_points: 6,
            eigen_ratio: 0.01,
            eigen_min: 1e-4,
        }
    }
}

/// One cube's bell curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cube {
    pub mean: Vector3<f64>,
    pub covariance: Matrix3<f64>,
    /// The covariance's inverse, kept: the score needs it for every point.
    pub inverse: Matrix3<f64>,
    pub points: usize,
}

/// Why a map couldn't be made.
#[derive(Clone, Debug, PartialEq)]
pub enum MapError {
    /// The cube's side isn't a positive, finite number of metres.
    BadCube(f64),
    /// No cube had `min_points` points: too few points, or cubes too small.
    NoCubes { points: usize, cube: f64 },
}

impl fmt::Display for MapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadCube(side) => write!(f, "a cube of {side} m: it must be above 0"),
            Self::NoCubes { points, cube } => {
                write!(
                    f,
                    "{points} points made no {cube} m cube with enough points"
                )
            }
        }
    }
}

impl Error for MapError {}

/// The map's cubes, by their place in the grid.
#[derive(Clone, Debug)]
pub struct CubeMap {
    side: f64,
    cubes: HashMap<Key, Cube>,
}

impl CubeMap {
    /// Cuts `points` (map frame) into cubes and keeps a curve for each with enough points.
    pub fn build(points: &[Point], settings: &MapSettings) -> Result<Self, MapError> {
        let side = settings.cube;
        if !(side.is_finite() && side > 0.0) {
            return Err(MapError::BadCube(side));
        }
        let mut groups: HashMap<Key, Vec<Vector3<f64>>> = HashMap::new();
        for p in points.iter().filter(|p| p.iter().all(|v| v.is_finite())) {
            groups.entry(key(p, side)).or_default().push(p.coords);
        }
        let cubes: HashMap<Key, Cube> = groups
            .into_iter()
            .filter(|(_, inside)| inside.len() >= settings.min_points.max(2))
            .filter_map(|(k, inside)| Some((k, curve(&inside, settings)?)))
            .collect();
        if cubes.is_empty() {
            return Err(MapError::NoCubes {
                points: points.len(),
                cube: side,
            });
        }
        Ok(Self { side, cubes })
    }

    /// The side of a cube (m).
    pub fn side(&self) -> f64 {
        self.side
    }

    /// How many cubes have a curve.
    pub fn len(&self) -> usize {
        self.cubes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cubes.is_empty()
    }

    /// The cube a point falls in, if it has a curve.
    pub fn get(&self, p: &Point) -> Option<&Cube> {
        self.cubes.get(&key(p, self.side))
    }

    /// The curves a point is scored against: its own cube's, and those of the six that
    /// share a face with it.
    pub fn neighbours<'a>(&'a self, p: &Point) -> impl Iterator<Item = &'a Cube> + 'a {
        let [x, y, z] = key(p, self.side);
        NEIGHBOURS
            .iter()
            .filter_map(move |[dx, dy, dz]| self.cubes.get(&[x + dx, y + dy, z + dz]))
    }

    /// Every cube's curve, in no set order.
    pub fn cubes(&self) -> impl Iterator<Item = &Cube> {
        self.cubes.values()
    }
}

fn key(p: &Point, side: f64) -> Key {
    [
        (p.x / side).floor() as i32,
        (p.y / side).floor() as i32,
        (p.z / side).floor() as i32,
    ]
}

/// The mean and covariance of a cube's points, the narrow directions held up by the
/// floors; `None` if it can't be inverted even then.
fn curve(inside: &[Vector3<f64>], settings: &MapSettings) -> Option<Cube> {
    let n = inside.len() as f64;
    let mean = inside.iter().sum::<Vector3<f64>>() / n;
    let covariance = inside
        .iter()
        .map(|p| (p - mean) * (p - mean).transpose())
        .sum::<Matrix3<f64>>()
        / (n - 1.0);
    let eigen = SymmetricEigen::new(covariance);
    let widest = eigen.eigenvalues.max();
    let floor = (settings.eigen_ratio * widest).max(settings.eigen_min);
    let values = eigen.eigenvalues.map(|v| v.max(floor));
    let vectors = eigen.eigenvectors;
    let covariance = vectors * Matrix3::from_diagonal(&values) * vectors.transpose();
    let inverse = vectors * Matrix3::from_diagonal(&values.map(|v| 1.0 / v)) * vectors.transpose();
    inverse.iter().all(|v| v.is_finite()).then_some(Cube {
        mean,
        covariance,
        inverse,
        points: inside.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wall() -> Vec<Point> {
        // A 2 m wide, 1 m high flat wall at x = 0.5: every point in the plane.
        (0..20)
            .flat_map(|i| (0..10).map(move |j| Point::new(0.5, i as f64 * 0.1, j as f64 * 0.1)))
            .collect()
    }

    #[test]
    fn a_flat_wall_still_has_an_inverse() {
        let map = CubeMap::build(&wall(), &MapSettings::with_cube(1.0)).expect("cubes");
        assert_eq!(map.len(), 2);
        for cube in map.cubes() {
            let thinnest = SymmetricEigen::new(cube.covariance).eigenvalues.min();
            assert!(thinnest >= 1e-4 - 1e-12);
            let identity = cube.covariance * cube.inverse;
            assert!((identity - Matrix3::identity()).norm() < 1e-9);
        }
    }

    #[test]
    fn cubes_with_too_few_points_are_dropped() {
        let few = [Point::new(0.5, 0.5, 0.5); 3];
        assert_eq!(
            CubeMap::build(&few, &MapSettings::with_cube(1.0)).err(),
            Some(MapError::NoCubes {
                points: 3,
                cube: 1.0
            })
        );
        assert_eq!(
            CubeMap::build(&wall(), &MapSettings::with_cube(0.0)).err(),
            Some(MapError::BadCube(0.0))
        );
    }

    #[test]
    fn a_point_sees_its_cube_and_the_face_neighbours() {
        let map = CubeMap::build(&wall(), &MapSettings::with_cube(1.0)).expect("cubes");
        // In the first cube: it and the one above it in y.
        assert_eq!(map.neighbours(&Point::new(0.5, 0.5, 0.5)).count(), 2);
        // Two cubes away: nothing.
        assert_eq!(map.neighbours(&Point::new(0.5, 3.5, 0.5)).count(), 0);
    }
}

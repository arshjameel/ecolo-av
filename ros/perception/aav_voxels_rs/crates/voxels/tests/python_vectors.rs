//! The Rust gives the Python's answers: `aav_voxels`' `cubes.py` on nine recorded clouds
//! (`python_vectors.txt`, made by `test/capture_python_vectors.py`): the lot with a post,
//! a leaning car with noise, walls all round, parked cars, a ramp, a crowned road, odd
//! points (NaN, inf, the car's own), an empty cloud, and a random scene on a tilted mount.
//!
//! | Check | Held to |
//! |---|---|
//! | fitted or not, and the candidates it used | the same |
//! | the road's plane a, b, c | 1e-6 (least squares by another route: numpy's SVD) |
//! | the points that count | the same |
//! | obstacle cubes, on the fitted road and on a flat one | the same points (1e-9 m: the mount's sums) |
//! | ground squares | the same squares; z the road's height |

mod vectors;

use voxels::{cubes, cubes_on, Point, Road};

const CLOSE: f64 = 1e-9;

fn same(a: &Point, b: &Point) -> bool {
    (a.x - b.x).abs() <= CLOSE && (a.y - b.y).abs() <= CLOSE && (a.z - b.z).abs() <= CLOSE
}

fn sorted(mut points: Vec<Point>) -> Vec<Point> {
    points.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then(a.y.total_cmp(&b.y))
            .then(a.z.total_cmp(&b.z))
    });
    points
}

/// Whether the two hold the same points, as sets.
fn same_set(name: &str, what: &str, rust: Vec<Point>, python: Vec<Point>) {
    let (rust, python) = (sorted(rust), sorted(python));
    assert_eq!(
        rust.len(),
        python.len(),
        "{name}: {what}: {} in Rust, {} in Python",
        rust.len(),
        python.len()
    );
    for (r, p) in rust.iter().zip(&python) {
        assert!(same(r, p), "{name}: {what}: Rust {r:?}, Python {p:?}");
    }
}

#[test]
fn the_rust_gives_the_pythons_cubes() {
    let (shape, clouds) = vectors::read(include_str!("python_vectors.txt"));
    assert_eq!(clouds.len(), 9);
    check(&shape, &clouds);
}

/// A file of your own (`capture_python_vectors.py --bench FILE`: a whole turn), checked the
/// same way: `VOXELS_VECTORS=FILE cargo test -p voxels`.
#[test]
fn the_rust_gives_the_pythons_cubes_on_a_file_of_your_own() {
    if let Some(path) = std::env::var_os("VOXELS_VECTORS") {
        let text = std::fs::read_to_string(&path).expect("VOXELS_VECTORS: a vectors file");
        let (shape, clouds) = vectors::read(&text);
        check(&shape, &clouds);
    }
}

fn check(shape: &voxels::CubeShape, clouds: &[vectors::Recorded]) {
    let shape = *shape;
    for c in clouds {
        let base: Vec<Point> = c.lidar.iter().map(|p| c.mount.to_base(p)).collect();
        let found = cubes(&base, &shape);
        let name = &c.name;

        assert_eq!(found.road.is_fitted(), c.fitted, "{name}: fitted");
        assert_eq!(found.road.candidates(), c.candidates, "{name}: candidates");
        let plane = found.road.plane();
        let off = [plane.a, plane.b, plane.c]
            .iter()
            .zip(c.plane)
            .map(|(rust, python)| (rust - python).abs())
            .fold(0.0, f64::max);
        assert!(off < 1e-6, "{name}: plane {plane:?}, {:?}", c.plane);
        println!("{name}: the plane {off:.1e} from the Python's");
        assert_eq!(found.points, c.counted, "{name}: points that count");

        let at = |i: &usize| base[*i];
        same_set(
            name,
            "obstacles",
            found.obstacles.clone(),
            c.obstacles.iter().map(at).collect(),
        );
        let flat = cubes_on(&base, &shape, Road::default());
        same_set(
            name,
            "flat obstacles",
            flat.obstacles,
            c.flat_obstacles.iter().map(at).collect(),
        );

        let python_ground = c
            .ground
            .iter()
            .map(|&[x, y]| Point::new(x, y, found.road.height(x, y)))
            .collect();
        same_set(name, "ground", found.ground, python_ground);
    }
}

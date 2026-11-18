//! `aav_voxels`' `test/test_cubes.py`, the cases that need no lidar turn. Those that do
//! are in `tests/python_vectors.rs`, on the Python's own clouds and answers.

use super::*;

const BODY: Body = Body {
    back: -0.5,
    front: 1.9,
    half_width: 0.8,
};

fn shape() -> CubeShape {
    CubeShape {
        body: BODY,
        ..CubeShape::default()
    }
}

fn p(x: f64, y: f64, z: f64) -> Point {
    Point::new(x, y, z)
}

#[test]
fn a_cube_holds_its_points_once() {
    let points = [p(0.01, 0.01, 0.01), p(0.2, 0.2, 0.2), p(0.3, 0.0, 0.0)];
    assert_eq!(
        cube_centres(&points, 0.25),
        vec![p(0.125, 0.125, 0.125), p(0.375, 0.125, 0.125)]
    );
    assert!(cube_centres(&[], 0.25).is_empty());
    let flat = square_centres(&points, 0.25);
    assert_eq!(flat, vec![p(0.125, 0.125, 0.0), p(0.375, 0.125, 0.0)]);
}

#[test]
fn negative_coordinates_floor_not_truncate() {
    assert_eq!(
        cube_centres(&[p(-0.1, -0.1, 0.5)], 0.25),
        vec![p(-0.125, -0.125, 0.625)]
    );
}

#[test]
fn lowest_per_cell_keeps_one_per_square() {
    let points = [p(0.5, 0.5, 1.0), p(0.6, 0.4, 0.2), p(3.0, 0.0, 0.7)];
    let mut low: Vec<f64> = lowest_per_cell(&points, 2.0).iter().map(|p| p.z).collect();
    low.sort_by(f64::total_cmp);
    assert_eq!(low, vec![0.2, 0.7]);
}

#[test]
fn each_cube_keeps_its_nearest_point() {
    let points = [
        p(5.2, 0.1, 1.0),
        p(5.01, 0.2, 1.1),
        p(5.3, 0.3, 1.2),
        p(-5.2, 0.1, 1.0),
    ];
    let mut kept: Vec<f64> = nearest_per_cube(&points, 0.5).iter().map(|p| p.x).collect();
    kept.sort_by(f64::total_cmp);
    assert_eq!(kept, vec![-5.2, 5.01]);
    assert!(nearest_per_cube(&[], 0.25).is_empty());
}

#[test]
fn ties_go_to_the_first_point() {
    // The same distance from base_link: the first, as numpy's stable sort keeps it.
    let points = [p(0.1, 0.0, 0.1), p(0.0, 0.1, 0.2), p(0.0, -0.1, 0.0)];
    let kept = nearest_per_cube(&points, 1.0);
    assert_eq!(kept, vec![p(0.0, -0.1, 0.0), p(0.1, 0.0, 0.1)]);
}

#[test]
fn the_percentile_is_numpys_linear_one() {
    // numpy.percentile(v, 75) for each, from numpy 2.5.
    let cases: [(&[f64], f64); 4] = [
        (&[3.0], 3.0),
        (&[1.0, 2.0], 1.75),
        (&[4.0, 1.0, 3.0, 2.0, 5.0], 4.0),
        (&[0.1, -0.2, 0.3, 0.05, 0.0, 0.2], 0.175),
    ];
    for (values, expected) in cases {
        let got = percentile(&mut values.to_vec(), 75.0).expect("some values");
        assert!((got - expected).abs() < 1e-15, "{values:?}: {got}");
    }
    assert_eq!(percentile(&mut [], 75.0), None);
}

#[test]
fn a_plane_through_points_on_it_is_exact() {
    let plane = Plane {
        a: 0.03,
        b: -0.02,
        c: 0.1,
    };
    let points: Vec<Point> = (0..30)
        .map(|i| {
            let (x, y) = (f64::from(i % 6) * 1.7, f64::from(i / 6) * 2.3 - 4.0);
            p(x, y, plane.height(x, y))
        })
        .collect();
    let fitted = fit_plane(points.iter()).expect("not on one line");
    assert!((fitted.a - plane.a).abs() < 1e-12);
    assert!((fitted.b - plane.b).abs() < 1e-12);
    assert!((fitted.c - plane.c).abs() < 1e-12);
    let line: Vec<Point> = (0..10).map(|i| p(f64::from(i), 0.0, 0.0)).collect();
    assert_eq!(fit_plane(line.iter()), None);
}

#[test]
fn too_few_candidates_keep_the_road_flat() {
    let road = fit_ground(&[p(5.0, 0.0, 0.0), p(6.0, 3.0, 0.0)], &shape());
    assert_eq!(road, Road::Flat { candidates: 2 });
    assert_eq!(road.height(3.0, 4.0), 0.0);
}

/// A grid of points, z from (x, y).
fn grid(xs: (f64, f64), ys: (f64, f64), step: f64, z: impl Fn(f64, f64) -> f64) -> Vec<Point> {
    let count = |(from, to): (f64, f64)| ((to - from) / step).round() as i32;
    (0..count(ys))
        .flat_map(|j| (0..count(xs)).map(move |i| (i, j)))
        .map(|(i, j)| {
            let (x, y) = (xs.0 + f64::from(i) * step, ys.0 + f64::from(j) * step);
            p(x, y, z(x, y))
        })
        .collect()
}

#[test]
fn a_steep_fit_is_not_believed() {
    // A ramp at 10 degrees: steeper than max_tilt, so taken as a fit gone wrong.
    let slope = 10.0_f64.to_radians().tan();
    let ramp = grid((2.0, 20.0), (-10.0, 10.0), 0.5, |x, _| x * slope);
    assert!(!fit_ground(&ramp, &shape()).is_fitted());
}

#[test]
fn a_tilted_road_is_fitted_and_its_lean_read() {
    let (roll, pitch) = (2.0_f64.to_radians(), -1.0_f64.to_radians());
    let road = grid((-20.0, 20.0), (-20.0, 20.0), 0.5, |x, y| {
        x * pitch.tan() - y * roll.tan() + 0.05
    });
    let fit = fit_ground(&road, &shape());
    assert!(fit.is_fitted());
    assert!((fit.roll() - roll).abs() < 1e-9 && (fit.pitch() - pitch).abs() < 1e-9);
    let c = cubes(&road, &shape());
    assert!(c.obstacles.is_empty() && !c.ground.is_empty());
    // The flat cut sees the low side's road as obstacles.
    assert!(!cubes_on(&road, &shape(), Road::default())
        .obstacles
        .is_empty());
}

#[test]
fn a_crowned_road_never_lifts_the_road_or_becomes_an_obstacle() {
    // The road falls away from the centre line: 0.43 m lower 12 m out.
    let crown = grid((2.0, 20.0), (-12.0, 12.0), 0.25, |_, y| -0.003 * y * y);
    assert!(fit_ground(&crown, &shape()).plane().c <= 0.01);
    assert!(cubes(&crown, &shape()).obstacles.is_empty());
}

#[test]
fn the_car_itself_the_sky_and_far_away_never_count() {
    let mut points = grid((-20.0, 20.0), (-20.0, 20.0), 0.5, |_, _| 0.0);
    let road = points.len();
    points.extend([
        p(1.75, 0.29, 1.55), // a camera on the roof
        p(0.0, 0.0, 1.5),    // the roof
        p(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        p(f64::NAN, 0.0, 0.0),
        p(40.0, 0.0, 1.0), // past range_max
        p(6.0, 0.0, 2.5),  // overhead
    ]);
    let c = cubes(&points, &shape());
    assert!(c.road.is_fitted());
    assert!(c.obstacles.is_empty());
    let on_car = grid((-20.0, 20.0), (-20.0, 20.0), 0.5, |_, _| 0.0)
        .iter()
        .filter(|q| BODY.contains(q))
        .count();
    assert_eq!(c.points, road - on_car + 1); // the overhead point counts, but isn't a cube
}

#[test]
fn a_post_is_cubes_measured_at_its_face() {
    let mut points = grid((-20.0, 20.0), (-20.0, 20.0), 0.5, |_, _| 0.0);
    points.extend((0..20).map(|k| p(10.0, -0.2 + 0.02 * f64::from(k), 0.3 + 0.07 * f64::from(k))));
    let c = cubes(&points, &shape());
    assert!(!c.obstacles.is_empty());
    assert!(c.obstacles.iter().all(|q| q.x == 10.0));
    assert!(c.ground.iter().all(|g| g.z.abs() < 1e-9));
}

#[test]
fn a_bad_shape_is_refused() {
    let bad = |s: CubeShape| s.checked().is_err();
    assert!(shape().checked().is_ok());
    assert!(bad(CubeShape {
        size: 0.0,
        ..shape()
    }));
    assert!(bad(CubeShape {
        min_height: 2.0,
        max_height: 0.3,
        ..shape()
    }));
    assert!(bad(CubeShape {
        range_max: f64::NAN,
        ..shape()
    }));
}

#[test]
fn the_mount_turns_then_moves() {
    let level = Mount::new([0.49, 0.0, 1.52], [0.0; 3]);
    assert_eq!(level.to_base(&p(1.0, 2.0, -1.0)), p(1.49, 2.0, 0.52));
    // Yawed a quarter turn left: the lidar's x is base_link's y.
    let turned = Mount::new([0.0; 3], [0.0, 0.0, std::f64::consts::FRAC_PI_2]);
    let q = turned.to_base(&p(1.0, 0.0, 0.0));
    assert!(q.x.abs() < 1e-15 && (q.y - 1.0).abs() < 1e-15);
}

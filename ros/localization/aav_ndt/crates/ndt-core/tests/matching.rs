//! The design's checks (docs/designs/2026-10-29_ndt.md, "Checks"), on made-up worlds:
//!
//! | Check | Pass |
//! |---|---|
//! | a known shift and turn | 1 cm, 0.1° |
//! | gradient and Hessian against finite differences, f64 | relative 1e-6 |
//! | a bad start, 1.5 m and 10° off, on a lot like Gazebo's | pulls in |
//! | the time per match | printed (`cargo test --release -- --nocapture`); this computer, not the Orin |

mod common;

use std::time::Instant;

use common::{map_from_route, route, Rng, World};
use ndt_core::{MapSettings, MatchSettings, Matcher, Point, Pose2, ScanFilter};

fn matcher(map: &[Point], cubes: &[f64]) -> Matcher {
    Matcher::new(map, cubes, MapSettings::with_cube, MatchSettings::default())
        .expect("the map has cubes")
}

/// A scan as the node prepares it: into base_link, then the design's filters.
fn prepared(world: &World, car: &Pose2, rng: &mut Rng) -> Vec<Point> {
    let (_, mount) = World::mount();
    ScanFilter::default().apply(
        world
            .scan(car, 1800, 0.03, rng)
            .iter()
            .map(|p| mount.to_base_link(p)),
    )
}

fn degrees(r: f64) -> f64 {
    r.to_degrees()
}

#[test]
fn a_known_shift_and_turn_comes_back_within_1_cm_and_a_tenth_of_a_degree() {
    let world = World::yard();
    let mut rng = Rng::new(1);
    let corners = [
        (-14.0, -8.0),
        (14.0, -8.0),
        (14.0, 10.0),
        (-14.0, 10.0),
        (-14.0, -8.0),
    ];
    let map = map_from_route(&world, &route(&corners, 2.0), &mut rng);
    let ndt = matcher(&map, &[2.0, 1.0]);
    let truth = Pose2::new(1.0, 2.0, 0.4);
    let scan = prepared(&world, &truth, &mut rng);
    let guess = Pose2::new(truth.x + 0.4, truth.y - 0.3, truth.yaw + 4f64.to_radians());
    let started = Instant::now();
    let found = ndt.align(&scan, guess).expect("a match");
    let ms = started.elapsed().as_secs_f64() * 1e3;
    let (metres, radians) = found.pose.distance(&truth);
    println!(
        "yard: {} map points, {} scan points; off {:.1} mm, {:.3}°; {} steps, {:.2} per point, \
         {ms:.1} ms",
        map.len(),
        scan.len(),
        metres * 1e3,
        degrees(radians),
        found.iterations,
        found.score_per_point,
    );
    assert!(found.converged);
    assert!(metres < 0.01, "off by {metres} m");
    assert!(degrees(radians) < 0.1, "off by {}°", degrees(radians));
}

#[test]
fn the_gradient_and_hessian_match_finite_differences() {
    let world = World::yard();
    let mut rng = Rng::new(2);
    let map = map_from_route(&world, &route(&[(-10.0, 0.0), (10.0, 0.0)], 2.0), &mut rng);
    let ndt = matcher(&map, &[1.0]);
    let scan = prepared(&world, &Pose2::new(0.0, 0.0, 0.0), &mut rng);
    // Off the answer, where every term matters.
    let at = Pose2::new(0.13, -0.08, 0.021);
    let exact = ndt.evaluate(&scan, &at);
    let h = 1e-6;
    let nudge = |p: &Pose2, i: usize, by: f64| {
        let mut q = *p;
        match i {
            0 => q.x += by,
            1 => q.y += by,
            _ => q.yaw += by,
        }
        q
    };
    for i in 0..3 {
        let up = ndt.evaluate(&scan, &nudge(&at, i, h));
        let down = ndt.evaluate(&scan, &nudge(&at, i, -h));
        let slope = (up.score - down.score) / (2.0 * h);
        let rel = (slope - exact.gradient[i]).abs() / exact.gradient.norm();
        assert!(
            rel < 1e-6,
            "gradient {i}: {slope} vs {} ({rel:e})",
            exact.gradient[i]
        );
        let column = (up.gradient - down.gradient) / (2.0 * h);
        for j in 0..3 {
            let rel = (column[j] - exact.hessian[(j, i)]).abs() / exact.hessian.norm();
            assert!(
                rel < 1e-6,
                "hessian {j},{i}: {} vs {} ({rel:e})",
                column[j],
                exact.hessian[(j, i)]
            );
        }
    }
}

#[test]
fn a_bad_start_on_a_lot_like_gazebos_pulls_in() {
    let world = World::test_lot();
    let mut rng = Rng::new(3);
    // The mapping drive: round the lot, not the scored tours.
    let corners = [
        (-4.0, -4.0),
        (64.0, -4.0),
        (64.0, 44.0),
        (-4.0, 44.0),
        (-4.0, -4.0),
        (30.0, 12.0),
    ];
    let built = Instant::now();
    let map = map_from_route(&world, &route(&corners, 3.0), &mut rng);
    let ndt = matcher(&map, &[4.0, 2.0, 1.0]);
    println!(
        "test lot: {} map points, cubes {:?}, built in {:.0} ms",
        map.len(),
        ndt.levels().collect::<Vec<_>>(),
        built.elapsed().as_secs_f64() * 1e3
    );
    // Off the mapping route, each with its own wrong start: 1.5 m and 10° off.
    let cases = [
        (Pose2::new(15.0, 15.0, 1.57), (1.5, 0.0, 10.0)),
        (Pose2::new(45.0, 20.0, -1.2), (0.0, -1.5, -10.0)),
        (Pose2::new(20.0, 0.0, 0.0), (-1.06, 1.06, 10.0)),
        (Pose2::new(8.0, 30.0, 2.0), (1.06, -1.06, -10.0)),
    ];
    let mut times = Vec::new();
    for (truth, (dx, dy, dyaw)) in cases {
        let scan = prepared(&world, &truth, &mut rng);
        let guess = Pose2::new(
            truth.x + dx,
            truth.y + dy,
            truth.yaw + f64::to_radians(dyaw),
        );
        let started = Instant::now();
        let found = ndt.align(&scan, guess).expect("a match");
        let ms = started.elapsed().as_secs_f64() * 1e3;
        times.push(ms);
        let (metres, radians) = found.pose.distance(&truth);
        println!(
            "  from {truth:?}, {dx}, {dy} m and {dyaw}° off: left {:.1} cm, {:.2}°; {} points, \
             {} steps, {:.2} per point, overlap {:.0} %, {ms:.1} ms",
            metres * 100.0,
            degrees(radians),
            scan.len(),
            found.iterations,
            found.score_per_point,
            found.overlap * 100.0
        );
        assert!(metres < 0.05, "{truth:?}: off by {metres} m");
        assert!(
            degrees(radians) < 0.3,
            "{truth:?}: off by {}°",
            degrees(radians)
        );
    }
    let worst = times.iter().copied().fold(0.0, f64::max);
    println!(
        "  time per match: mean {:.1} ms, worst {worst:.1} ms (this computer; the test profile, \
         opt-level 2)",
        times.iter().sum::<f64>() / times.len() as f64
    );
}

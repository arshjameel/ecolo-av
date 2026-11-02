//! The localizer, cloud after cloud: the guess from drifting odometry, LOST after five poor
//! fits, and the restart from `ekf_global`.

mod common;

use std::time::Duration;

use common::{map_from_route, route, Rng, World};
use ndt_core::{
    localizer::{Outcome, Skip},
    LocalizerSettings, MapSettings, MatchSettings, Matcher, Point, Pose2, Tracking,
};

fn localizer(rng: &mut Rng) -> (World, ndt_core::Localizer) {
    let world = World::test_lot();
    let corners = [
        (-4.0, -4.0),
        (64.0, -4.0),
        (64.0, 44.0),
        (-4.0, 44.0),
        (-4.0, -4.0),
    ];
    let map: Vec<Point> = map_from_route(&world, &route(&corners, 3.0), rng);
    let matcher = Matcher::new(
        &map,
        &[4.0, 2.0, 1.0],
        MapSettings::with_cube,
        MatchSettings::default(),
    )
    .expect("cubes");
    let settings = LocalizerSettings {
        mount: World::mount().1,
        ..LocalizerSettings::default()
    };
    (world, ndt_core::Localizer::new(matcher, settings))
}

fn at(seconds: f64) -> Duration {
    Duration::from_secs_f64(seconds)
}

#[test]
fn it_follows_a_drive_on_drifting_odometry() {
    let mut rng = Rng::new(10);
    let (world, mut ndt) = localizer(&mut rng);
    let start = Pose2::new(0.0, 0.0, 0.0);
    ndt.set_start(start, None);
    // 1 m/s east, then turning left; odom drifts: 5 % long and 2°/s of gyro bias.
    for i in 0..=30 {
        let t = i as f64 * 0.2;
        let truth = Pose2::new(t, 0.05 * t * t, 0.1 * t);
        let odom = Pose2::new(
            1.05 * truth.x + 3.0,
            1.05 * truth.y - 2.0,
            truth.yaw + 0.035 * t,
        );
        ndt.odom(at(t), odom);
        let report = ndt.cloud(at(t), world.scan(&truth, 1800, 0.03, &mut rng));
        let Outcome::Pose(fit) = report.outcome else {
            panic!("cloud {i}: {:?}", report.outcome);
        };
        let (pose, covariance) = (fit.pose, ndt.covariance(&fit));
        let (metres, radians) = pose.distance(&truth);
        assert!(
            metres < 0.05 && radians.to_degrees() < 0.3,
            "cloud {i}: {pose:?} vs {truth:?}"
        );
        assert!(covariance[0] > 0.0 && covariance[14] >= 1e2);
    }
    assert_eq!(ndt.tracking(), Tracking::Tracking);
}

#[test]
fn five_poor_fits_make_it_lost_and_it_starts_again_from_the_global_ekf() {
    let mut rng = Rng::new(11);
    let (world, mut ndt) = localizer(&mut rng);
    assert_eq!(
        ndt.cloud(at(0.0), []).outcome,
        Outcome::Skipped(Skip::NoStart)
    );
    let truth = Pose2::new(20.0, 0.0, 0.0);
    // Told it is 20 m away, facing the wrong way: nothing fits.
    ndt.set_start(Pose2::new(40.0, 10.0, 3.0), None);
    ndt.global(at(0.0), Pose2::new(20.4, -0.3, 0.05));
    for i in 1..=5 {
        let report = ndt.cloud(at(i as f64 * 0.1), world.scan(&truth, 1800, 0.03, &mut rng));
        assert!(
            !matches!(report.outcome, Outcome::Pose(_)),
            "cloud {i} fitted: {:?}",
            report.outcome
        );
        let expected = if i < 5 {
            Tracking::Tracking
        } else {
            Tracking::Lost
        };
        assert_eq!(report.tracking, expected, "cloud {i}");
    }
    assert_eq!(ndt.restarts(), 1);
    // From ekf_global's pose it fits again.
    let report = ndt.cloud(at(0.6), world.scan(&truth, 1800, 0.03, &mut rng));
    let Outcome::Pose(fit) = report.outcome else {
        panic!("after the restart: {:?}", report.outcome);
    };
    assert!(fit.pose.distance(&truth).0 < 0.05);
    assert_eq!(report.tracking, Tracking::Tracking);
    assert_eq!(report.restarts, 1);
}

#[test]
fn a_jump_while_tracking_is_poor_but_not_after_a_start() {
    let mut rng = Rng::new(12);
    let (world, mut ndt) = localizer(&mut rng);
    let here = Pose2::new(20.0, 0.0, 0.0);
    // From a start 0.8 m off, the first fit may move that far: a start isn't trusted.
    ndt.set_start(Pose2::new(20.8, 0.0, 0.0), None);
    ndt.odom(at(0.0), Pose2::new(0.0, 0.0, 0.0));
    let report = ndt.cloud(at(0.0), world.scan(&here, 1800, 0.03, &mut rng));
    assert!(
        matches!(report.outcome, Outcome::Pose(_)),
        "{:?}",
        report.outcome
    );
    // Then odometry says the car stood still, but the scan is from 0.8 m further east:
    // a fit there is a jump, not published.
    ndt.odom(at(0.1), Pose2::new(0.0, 0.0, 0.0));
    let moved = Pose2::new(20.8, 0.0, 0.0);
    let report = ndt.cloud(at(0.1), world.scan(&moved, 1800, 0.03, &mut rng));
    assert!(
        matches!(
            report.outcome,
            Outcome::Poor {
                why: ndt_core::localizer::Poor::Jumped,
                ..
            }
        ),
        "{:?}",
        report.outcome
    );
    assert_eq!(report.poor_in_row, 1);
}

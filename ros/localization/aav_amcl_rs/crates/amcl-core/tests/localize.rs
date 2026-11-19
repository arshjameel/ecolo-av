//! AMCL on a made-up yard: it finds the car from a near guess, finds it again after a
//! kidnap, keeps its particle count inside KLD's bounds, and its motion model spreads
//! particles as nav2's alphas say.

mod common;

use std::time::Duration;

use amcl_core::{
    filter::kld_limit, Amcl, DifferentialMotion, LikelihoodField, Particle, Pose2, Rng, Step,
};
use common::{amcl, cast, route, yard};

/// Odometry from the truth's steps, a little wrong (1 % long, 0.2° per step to the
/// left), as wheels are: in its own frame, from the origin.
struct Odometry {
    pose: Pose2,
    last_truth: Option<Pose2>,
}

impl Odometry {
    fn new() -> Self {
        Self {
            pose: Pose2::default(),
            last_truth: None,
        }
    }

    fn step(&mut self, truth: &Pose2) -> Pose2 {
        if let Some(last) = self.last_truth {
            let mut d = last.between(truth);
            d.x *= 1.01;
            d.y *= 1.01;
            d.yaw += 0.2f64.to_radians();
            self.pose = self.pose.compose(&d);
        }
        self.last_truth = Some(*truth);
        self.pose
    }
}

/// Feeds AMCL one scan per truth pose; returns each published pose's error (m, rad).
fn drive(
    amcl: &mut Amcl<LikelihoodField>,
    map: &amcl_core::OccupancyMap,
    odom: &mut Odometry,
    truths: &[Pose2],
    clock: &mut f64,
    counts: &mut Vec<usize>,
) -> Vec<(f64, f64)> {
    let mut errors = Vec::new();
    for truth in truths {
        *clock += 0.1;
        let t = Duration::from_secs_f64(*clock);
        amcl.odom(t, odom.step(truth));
        if let Step::Updated { pose, .. } = amcl.scan(t, &cast(map, truth)) {
            counts.push(amcl.particles().len());
            if let Some(e) = pose {
                let (d, a) = e.pose.distance(truth);
                errors.push((d.0, a.0));
            }
        }
    }
    errors
}

fn mean(xs: impl Iterator<Item = f64>) -> f64 {
    let v: Vec<f64> = xs.collect();
    v.iter().sum::<f64>() / v.len() as f64
}

fn cov(xy: f64, yaw: f64) -> [[f64; 3]; 3] {
    [[xy, 0.0, 0.0], [0.0, xy, 0.0], [0.0, 0.0, yaw]]
}

#[test]
fn converges_from_a_near_guess() {
    let map = yard();
    let mut amcl = amcl(&map, (0.0, 0.0), 1);
    let truths = route(80);
    let start = truths[0];
    // 0.5 m and 6° off, with nav2's initial spread (0.5 m, 15°).
    let guess = Pose2::new(start.x + 0.4, start.y - 0.3, start.yaw + 0.1);
    amcl.set_pose(
        None,
        guess,
        &cov(0.25, (std::f64::consts::PI / 12.0).powi(2)),
    );
    let (mut clock, mut counts) = (0.0, Vec::new());
    let errors = drive(
        &mut amcl,
        &map,
        &mut Odometry::new(),
        &truths,
        &mut clock,
        &mut counts,
    );
    let last: Vec<_> = errors.iter().rev().take(10).collect();
    let (d, a) = (
        mean(last.iter().map(|e| e.0)),
        mean(last.iter().map(|e| e.1)),
    );
    println!(
        "published {}, last 10: {d:.3} m, {:.2}°",
        errors.len(),
        a.to_degrees()
    );
    assert!(errors.len() >= 20, "{}", errors.len());
    assert!(d < 0.10, "{d} m off");
    assert!(a < 0.03, "{a} rad off");
}

#[test]
fn recovers_after_a_kidnap() {
    let map = yard();
    // Recovery's rates: nav2 suggests 0.001 for the slow average, which learns what a
    // good fit scores over ~1000 updates; 0.01 learns it within this test's warm-up.
    let mut amcl = amcl(&map, (0.01, 0.1), 2);
    let warm_up = 250;
    let truths = route(warm_up + 400);
    amcl.set_pose(None, truths[0], &cov(0.25, 0.07));
    let mut odom = Odometry::new();
    let (mut clock, mut counts) = (0.0, Vec::new());
    let before = drive(
        &mut amcl,
        &map,
        &mut odom,
        &truths[..warm_up],
        &mut clock,
        &mut counts,
    );
    assert!(
        before.last().is_some_and(|e| e.0 < 0.15),
        "{:?}",
        before.last()
    );
    // Kidnapped: the car is suddenly 3.6 m away (3 m east, 2 m north), on a loop of the
    // same shape; the odometry saw nothing of it. (Not across the middle facing back: the
    // yard's walls look the same from there, a test of the boxes only.)
    let kidnapped: Vec<Pose2> = truths[warm_up..]
        .iter()
        .map(|p| Pose2::new(p.x + 3.0, p.y + 2.0, p.yaw))
        .collect();
    odom.last_truth = None;
    let after = drive(
        &mut amcl,
        &map,
        &mut odom,
        &kidnapped,
        &mut clock,
        &mut counts,
    );
    let lost_at_first = after.first().is_some_and(|e| e.0 > 3.0);
    let found = after.iter().position(|e| e.0 < 0.3 && e.1 < 0.1);
    println!(
        "lost at first: {lost_at_first}; found after {found:?} poses of {}",
        after.len()
    );
    assert!(lost_at_first);
    let found = found.expect("found again");
    // ... and it stays found.
    let rest = &after[found..];
    assert!(mean(rest.iter().map(|e| e.0)) < 0.3, "{rest:?}");
}

#[test]
fn the_particle_count_stays_in_klds_bounds() {
    let map = yard();
    let mut amcl = amcl(&map, (0.0, 0.0), 3);
    let truths = route(60);
    amcl.set_pose(None, truths[0], &cov(1.0, 0.2));
    let settings = *amcl.filter().settings();
    let (mut clock, mut counts) = (0.0, Vec::new());
    drive(
        &mut amcl,
        &map,
        &mut Odometry::new(),
        &truths,
        &mut clock,
        &mut counts,
    );
    println!("counts: {counts:?}");
    assert!(counts
        .iter()
        .all(|n| (settings.min_particles..=settings.max_particles).contains(n)));
    assert_eq!(counts[0], settings.max_particles, "a start fills the set");
    // Found: the cloud is tight, so KLD needs few.
    assert!(*counts.last().expect("updates") < 1000, "{counts:?}");
    assert!(kld_limit(2, &settings) == settings.min_particles);
}

#[test]
fn the_motion_model_spreads_as_the_alphas_say() {
    // Only alpha3 (distance from driving) and alpha2 (heading from driving).
    let model = DifferentialMotion {
        alpha1: 0.0,
        alpha2: 0.01,
        alpha3: 0.04,
        alpha4: 0.0,
    };
    let n = 50_000;
    let mut ps = vec![Particle::new(Pose2::default(), 1.0 / n as f64); n];
    let mut rng = Rng::new(11);
    // 2 m straight ahead.
    model.apply(
        &mut ps,
        &Pose2::default(),
        &Pose2::new(2.0, 0.0, 0.0),
        &mut rng,
    );
    let dist: Vec<f64> = ps.iter().map(|p| p.pose.x.hypot(p.pose.y)).collect();
    let yaws: Vec<f64> = ps.iter().map(|p| p.pose.yaw).collect();
    let sd = |v: &[f64]| {
        let m = v.iter().sum::<f64>() / v.len() as f64;
        (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt()
    };
    let mean_d = dist.iter().sum::<f64>() / n as f64;
    // Distance: sd sqrt(alpha3) * 2 m = 0.4 m, about the move.
    assert!((mean_d - 2.0).abs() < 0.01, "{mean_d}");
    assert!((sd(&dist) - 0.4).abs() < 0.01, "{}", sd(&dist));
    // Heading: rot1 and rot2 each sd sqrt(alpha2) * 2 m = 0.2 rad, so their sum sqrt(2) x.
    assert!(
        (sd(&yaws) - 0.2 * 2f64.sqrt()).abs() < 0.01,
        "{}",
        sd(&yaws)
    );
    // No sideways slip on its own: the sideways spread comes only from rot1's turn.
    let side = sd(&ps.iter().map(|p| p.pose.y).collect::<Vec<_>>());
    assert!((side - 2.0 * 0.2).abs() < 0.03, "{side}");
}

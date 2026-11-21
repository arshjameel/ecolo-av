//! Synthetic scans: a world of walls and posts, scanned from a car driving an arc.

use super::*;

/// A 60 m x 40 m yard: four walls, 12 posts; a point every 0.1 m, from 0.4 to 2 m up.
fn yard() -> Vec<Point> {
    let mut out = Vec::new();
    let mut wall = |a: (f64, f64), b: (f64, f64)| {
        let n = ((b.0 - a.0).hypot(b.1 - a.1) / 0.1) as usize;
        for i in 0..=n {
            let t = i as f64 / n as f64;
            for k in 0..9 {
                out.push([
                    a.0 + t * (b.0 - a.0),
                    a.1 + t * (b.1 - a.1),
                    0.4 + 0.2 * f64::from(k),
                ]);
            }
        }
    };
    wall((-20.0, -20.0), (40.0, -20.0));
    wall((40.0, -20.0), (40.0, 20.0));
    wall((40.0, 20.0), (-20.0, 20.0));
    wall((-20.0, 20.0), (-20.0, -20.0));
    for i in 0..12 {
        let (x, y) = (
            -10.0 + 4.0 * f64::from(i),
            if i % 2 == 0 { 6.0 } else { -7.0 },
        );
        for a in 0..16 {
            let angle = f64::from(a) * std::f64::consts::TAU / 16.0;
            for k in 0..9 {
                out.push([
                    x + 0.3 * angle.cos(),
                    y + 0.3 * angle.sin(),
                    0.4 + 0.2 * f64::from(k),
                ]);
            }
        }
    }
    out
}

/// A corridor: two long parallel walls 6 m apart, nothing along it. The points fall at
/// random along it (a regular grid would give the match a lattice to lock onto).
fn corridor() -> Vec<Point> {
    let mut out = Vec::new();
    let mut seed: u64 = 7;
    let mut random = move || {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    for _ in 0..80_000 {
        let x = -40.0 + 440.0 * random();
        let y = if random() < 0.5 { -3.0 } else { 3.0 };
        out.push([x, y, 0.4 + 1.6 * random()]);
    }
    out
}

/// What the car at `pose` sees within 30 m, in `base_link`.
fn scan(world: &[Point], pose: &Pose2) -> Vec<Point> {
    let (s, c) = pose.yaw.sin_cos();
    world
        .iter()
        .map(|p| {
            let (dx, dy) = (p[0] - pose.x, p[1] - pose.y);
            [c * dx + s * dy, -s * dx + c * dy, p[2]]
        })
        .filter(|p| p[0].hypot(p[1]) < 30.0)
        .collect()
}

/// Drive `steps` scans at `speed` m/s turning `rate` rad/s, 10 Hz; the wheels report
/// `speed x wheel_scale`. Return (truth, estimate) at the end and the steps.
fn drive(
    world: &[Point],
    speed: f64,
    rate: f64,
    wheel_scale: Option<f64>,
    steps: usize,
) -> (Pose2, Pose2, Vec<Step>) {
    let dt = 0.1;
    let mut odo = Odometry::new(Settings::default());
    let mut truth = Pose2::default();
    let mut out = Vec::new();
    for i in 0..=steps {
        if i > 0 {
            truth = truth.then(&Motion::from_twist(speed, rate, dt).pose());
        }
        let wheels = wheel_scale.map(|k| (k * speed, rate));
        out.push(odo.step(&scan(world, &truth), i as f64 * dt, wheels));
    }
    let end = out.last().map_or(Pose2::default(), |s| s.pose);
    (truth, end, out)
}

#[test]
fn an_arc_step_ends_on_the_circle() {
    // A quarter circle of radius 2: arc pi, chord 2 sqrt 2, ends at (2, 2) facing left.
    let m = Motion::from_twist(std::f64::consts::PI, std::f64::consts::FRAC_PI_2, 1.0);
    let p = m.pose();
    assert!(
        (p.x - 2.0).abs() < 1e-12 && (p.y - 2.0).abs() < 1e-12,
        "{p:?}"
    );
    assert!((p.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    // Straight: all along x.
    let s = Motion::from_twist(2.0, 0.0, 0.5).pose();
    assert_eq!((s.x, s.y, s.yaw), (1.0, 0.0, 0.0));
}

#[test]
fn the_map_finds_the_nearest_point_round_its_voxel() {
    let mut map = VoxelMap::new(0.5, 20);
    map.add(&[[0.0, 0.0, 1.0], [0.6, 0.0, 1.0], [5.0, 5.0, 1.0]]);
    let (q, d2) = map.nearest(&[0.45, 0.0, 1.0]).expect("a point");
    assert_eq!(q, [0.6, 0.0, 1.0]);
    assert!((d2 - 0.0225).abs() < 1e-12);
    assert!(map.nearest(&[3.0, 3.0, 1.0]).is_none());
    map.forget_beyond(0.0, 0.0, 2.0);
    assert_eq!(map.len(), 2);
}

#[test]
fn the_ground_and_the_near_points_are_cropped() {
    let s = Settings::default();
    let kept = crop(
        &[
            [5.0, 0.0, 0.1],
            [5.0, 0.0, 1.0],
            [0.5, 0.0, 1.0],
            [f64::NAN, 0.0, 1.0],
        ],
        &s,
    );
    assert_eq!(kept, vec![[5.0, 0.0, 1.0]]);
}

#[test]
fn a_drive_round_the_yard_with_honest_wheels_stays_on_the_truth() {
    let (truth, est, steps) = drive(&yard(), 2.0, 0.15, Some(1.0), 40);
    let off = (truth.x - est.x).hypot(truth.y - est.y);
    assert!(off < 0.05, "{off}: {truth:?} vs {est:?}");
    assert!(wrap(truth.yaw - est.yaw).abs() < 0.005);
    // The twist: 2 m/s, 0.15 rad/s.
    let last = steps.last().expect("steps");
    assert!((last.speed - 2.0).abs() < 0.05, "{}", last.speed);
    assert!((last.yaw_rate - 0.15).abs() < 0.02, "{}", last.yaw_rate);
}

#[test]
fn the_points_correct_wheels_that_read_20_percent_fast() {
    // The wheels say 2.4 m/s; the car does 2.0 (a slip, a wrong tyre size).
    let (truth, est, steps) = drive(&yard(), 2.0, 0.1, Some(1.2), 40);
    let off = (truth.x - est.x).hypot(truth.y - est.y);
    let by_wheels = 0.2 * 2.0 * 4.0;
    assert!(
        off < 0.1 * by_wheels,
        "{off} (the wheels alone: {by_wheels})"
    );
    let last = steps.last().expect("steps");
    assert!((last.speed - 2.0).abs() < 0.1, "{}", last.speed);
}

#[test]
fn without_wheels_it_holds_the_last_step_as_kiss_icp_does() {
    let (truth, est, _) = drive(&yard(), 1.5, -0.1, None, 30);
    let off = (truth.x - est.x).hypot(truth.y - est.y);
    assert!(off < 0.1, "{off}");
}

#[test]
fn in_a_corridor_the_wheels_carry_the_step_and_nothing_slides_sideways() {
    // Along the corridor the walls say nothing: the wheels' 2 m/s holds. Across it they
    // do, and the car can't slide: it stays on y = 0.
    let (truth, est, steps) = drive(&corridor(), 2.0, 0.0, Some(1.0), 40);
    assert!((truth.x - est.x).abs() < 0.2, "{truth:?} vs {est:?}");
    assert!(est.y.abs() < 0.02 && est.yaw.abs() < 0.005, "{est:?}");
    for s in &steps[1..] {
        assert!((s.speed - 2.0).abs() < 0.05, "{}", s.speed);
    }
}

#[test]
fn the_threshold_shrinks_once_the_answers_agree_with_their_priors() {
    let (_, _, steps) = drive(&yard(), 2.0, 0.1, Some(1.0), 20);
    let first = steps[0].threshold;
    let last = steps.last().map_or(first, |s| s.threshold);
    assert!(last < first, "{first} -> {last}");
    assert!(last >= Settings::default().min_threshold);
}

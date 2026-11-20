//! The step finder on clouds cast as the roof lidar casts them: a VLP-16 (16 rings, 2
//! degrees apart, -15 to +15; 1,800 steps round) at `vehicle.yaml`'s lidar_link, 1.52 m up,
//! with 3 cm of range noise, as our simulator's. Each ground is a height under (x, y).

use super::*;

/// vehicle.yaml's lidar_link, in base_link.
const LIDAR: [f64; 3] = [0.49, 0.0, 1.52];
/// The front bumper, in base_link (vehicle.yaml: length 2.00, rear overhang 0.20).
const BUMPER: f64 = 1.8;

/// A small, fixed random stream: the same cloud every run, without a crate.
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Gaussian, by the sum of 12 uniforms.
    fn gauss(&mut self, sigma: f64) -> f64 {
        ((0..12).map(|_| self.uniform()).sum::<f64>() - 6.0) * sigma
    }
}

/// One lidar turn over the ground `height(x, y)`, in base_link, `sigma` m of range noise.
fn cast(height: impl Fn(f64, f64) -> f64, sigma: f64) -> Vec<Point> {
    let mut noise = Noise(0x9E37_79B9_7F4A_7C15);
    let mut out = Vec::new();
    for ring in 0..8 {
        let e = (-15.0 + 2.0 * f64::from(ring)).to_radians();
        let (se, ce) = e.sin_cos();
        for step in 0..1800 {
            let a = (f64::from(step) * 0.2).to_radians();
            let (sa, ca) = a.sin_cos();
            let at = |t: f64| {
                let (x, y, z) = (
                    LIDAR[0] + t * ce * ca,
                    LIDAR[1] + t * ce * sa,
                    LIDAR[2] + t * se,
                );
                (x, y, z, z - height(x, y))
            };
            let (mut lo, mut hi) = (0.0, None);
            let mut t = 0.5;
            while t < 40.0 {
                if at(t).3 <= 0.0 {
                    hi = Some(t);
                    break;
                }
                lo = t;
                t += 0.02;
            }
            let Some(mut hi) = hi else { continue };
            for _ in 0..12 {
                let mid = 0.5 * (lo + hi);
                if at(mid).3 <= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            let (x, y, z, _) = at(hi + noise.gauss(sigma));
            out.push(Point::new(x, y, z));
        }
    }
    out
}

fn found(height: impl Fn(f64, f64) -> f64) -> Vec<Point> {
    steps(&cast(height, 0.03), &StepShape::default())
}

/// A kerb across the road, its face `x0` m ahead of base_link, `h` high, `depth` deep
/// (`f64::INFINITY`: a pavement behind it), 10 m wide.
fn kerb_across(x0: f64, h: f64, depth: f64) -> impl Fn(f64, f64) -> f64 {
    move |x, y| {
        if x >= x0 && x <= x0 + depth && y.abs() <= 5.0 {
            h
        } else {
            0.0
        }
    }
}

#[test]
fn a_flat_road_has_no_steps() {
    assert!(found(|_, _| 0.0).is_empty());
}

#[test]
fn a_kerb_across_the_road_is_found_where_it_is() {
    // sim-check --kerb's: 0.3 m high, 0.3 m deep, 10 m across.
    for (x0, h, depth) in [
        (BUMPER + 4.0, 0.3, 0.3),
        (BUMPER + 3.5, 0.15, f64::INFINITY),
        (BUMPER + 4.0, 0.15, 0.3),
    ] {
        let s = found(kerb_across(x0, h, depth));
        assert!(
            s.len() >= 10,
            "{h} m kerb at {x0} m: {} step points",
            s.len()
        );
        for p in &s {
            assert!(
                p.x >= x0 - 0.1 && p.x <= x0 + 0.35 && p.y.abs() <= 5.1,
                "{h} m kerb at {x0} m: a step at {p:?}"
            );
        }
    }
}

#[test]
fn a_kerb_beside_the_road_is_found_along_it() {
    // A 0.15 m kerb 2 m to the right, a pavement behind it: as on the campus. One turn
    // finds it only where the two lowest rings cross it near each other (about 5.5 m
    // ahead and behind the lidar); the costmap remembers each, so a drive draws the line.
    let s = found(|_, y| if y <= -2.0 { 0.15 } else { 0.0 });
    assert!(s.len() >= 4, "{} step points", s.len());
    assert!(s.iter().all(|p| (-2.3..=-1.95).contains(&p.y)), "{s:?}");
    // Ahead and behind the car both.
    assert!(s.iter().any(|p| p.x > BUMPER) && s.iter().any(|p| p.x < 0.0));
}

#[test]
fn slopes_and_ramps_are_not_steps() {
    for degrees in [3.0_f64, 6.0, 10.0] {
        let g = degrees.to_radians().tan();
        // Uphill ahead, downhill ahead, and sideways.
        assert!(found(|x, _| g * x).is_empty(), "{degrees} deg up");
        assert!(found(|x, _| -g * x).is_empty(), "{degrees} deg down");
        assert!(found(|_, y| g * y).is_empty(), "{degrees} deg sideways");
        // A ramp starting 5 m ahead, from flat: the corner where it starts.
        assert!(
            found(|x, _| g * (x - 5.0).max(0.0)).is_empty(),
            "a {degrees} deg ramp"
        );
    }
}

#[test]
fn the_roads_camber_is_not_a_step() {
    // Crowned, 2.5 % down each side from the centre line (y = 0).
    assert!(found(|_, y| -0.025 * y.abs()).is_empty());
}

#[test]
fn a_low_speed_bump_is_not_a_step() {
    // 7.5 cm high, 0.6 m long, rounded, 6 m ahead: drive over it slowly.
    let bump = |x: f64, _| {
        let u = (x - 6.0) / 0.3;
        if u.abs() < 1.0 {
            0.075 * (0.5 + 0.5 * (std::f64::consts::PI * u).cos())
        } else {
            0.0
        }
    };
    assert!(found(bump).is_empty());
}

#[test]
fn grass_beside_the_road_is_not_a_step() {
    // Rough grass 2 m to the right, level with the road: 0 to 6 cm, changing every 5 cm.
    let grass = |x: f64, y: f64| {
        if y <= -2.0 {
            let n = ((x * 20.0).floor() * 7919.0 + (y * 20.0).floor() * 104_729.0).sin();
            0.03 + 0.03 * n
        } else {
            0.0
        }
    };
    assert!(found(grass).is_empty());
}

#[test]
fn what_hangs_over_the_road_is_not_a_step() {
    // A sign's board, 2.2 m up, 6 m ahead.
    let s = steps(
        &[
            Point::new(6.0, 0.0, 0.0),
            Point::new(6.0, 0.05, 2.2),
            Point::new(6.02, 0.05, 2.2),
        ],
        &StepShape::default(),
    );
    assert!(s.is_empty());
}

#[test]
fn one_point_alone_is_noise() {
    let road = [Point::new(6.0, 0.0, 0.0), Point::new(6.1, 0.0, 0.0)];
    let lone = [&road[..], &[Point::new(6.05, 0.05, 0.2)]].concat();
    assert!(steps(&lone, &StepShape::default()).is_empty());
    let two = [&lone[..], &[Point::new(6.06, 0.06, 0.2)]].concat();
    assert_eq!(steps(&two, &StepShape::default()).len(), 1);
}

#[test]
fn a_bad_shape_is_refused() {
    let d = StepShape::default();
    assert!(d.checked().is_ok());
    assert!(StepShape { radius: 0.0, ..d }.checked().is_err());
    assert!(StepShape { min_step: 0.6, ..d }.checked().is_err());
    assert!(StepShape {
        range_max: f64::NAN,
        ..d
    }
    .checked()
    .is_err());
}

/// The farthest a kerb across the road is found, from the front bumper (m): kerbs
/// 0.1 m apart from 1 m to 12 m past it; None if never.
fn first_seen(h: f64, depth: f64) -> Option<f64> {
    (10..=120)
        .rev()
        .map(|k| f64::from(k) * 0.1)
        .find(|ahead| found(kerb_across(BUMPER + ahead, h, depth)).len() >= 3)
}

#[test]
fn a_kerb_is_found_before_the_car_reaches_it() {
    // At 2 m/s the car stops in about 1.5 m: each kerb must be found farther than that.
    for (h, depth) in [(0.3, 0.3), (0.15, f64::INFINITY)] {
        let far = first_seen(h, depth).unwrap_or(0.0);
        assert!(far >= 3.0, "a {h} m kerb: first found {far:.1} m ahead");
    }
}

/// `cargo test -p voxels --release -- --ignored --nocapture steps_range`: from how far
/// ahead of the bumper each kerb is found, and how many step points at each distance.
#[test]
#[ignore = "a table for the docs, not a check"]
fn steps_range() {
    for (h, depth) in [
        (0.3, 0.3),
        (0.3, f64::INFINITY),
        (0.15, 0.3),
        (0.15, f64::INFINITY),
        (0.10, f64::INFINITY),
    ] {
        let row: Vec<String> = (2..=12)
            .map(|k| {
                let ahead = f64::from(k) * 0.5 + 1.0;
                format!(
                    "{ahead:.1}:{}",
                    found(kerb_across(BUMPER + ahead, h, depth)).len()
                )
            })
            .collect();
        println!(
            "kerb {h} m, {} deep: first found {:?} m past the bumper; points {}",
            if depth.is_finite() {
                "0.3 m"
            } else {
                "pavement"
            },
            first_seen(h, depth),
            row.join(" ")
        );
    }
}

/// The road, flat, and a post (0.5 m square, 1.8 m: sim-check's walker) centred at
/// (`x0`, `y0`).
fn post_at(x0: f64, y0: f64) -> impl Fn(f64, f64) -> f64 {
    move |x, y| {
        if (x - x0).abs() <= 0.25 && (y - y0).abs() <= 0.25 {
            1.8
        } else {
            0.0
        }
    }
}

/// One cloud's kerbs, as the node finds them: the steps, less the feet of tall things.
fn kerbs_in(cloud: &[Point]) -> Vec<Point> {
    let shape = crate::CubeShape::default();
    let good = crate::usable(cloud, &shape);
    let road = crate::cubes(cloud, &shape).road;
    let s = StepShape::default();
    kerbs_only(steps(&good, &s), &good, &road, &s, shape.max_height)
}

#[test]
fn a_post_makes_steps_but_is_no_kerb() {
    // TODO 189: its face is hit ring after ring, 0.1-0.5 m apart: steps. It is a cube.
    for ahead in [3.0, 6.0] {
        let cloud = cast(post_at(BUMPER + ahead, 0.0), 0.03);
        assert!(
            !steps(&cloud, &StepShape::default()).is_empty(),
            "a post {ahead} m ahead: no steps at all, the test shows nothing"
        );
        let k = kerbs_in(&cloud);
        assert!(k.is_empty(), "a post {ahead} m ahead: kerbs {k:?}");
    }
}

#[test]
fn a_kerb_is_still_a_kerb() {
    for (x0, h, depth) in [
        (BUMPER + 4.0, 0.3, 0.3),
        (BUMPER + 3.5, 0.15, f64::INFINITY),
        (BUMPER + 4.0, 0.15, 0.3),
    ] {
        let cloud = cast(kerb_across(x0, h, depth), 0.03);
        let all = steps(&cloud, &StepShape::default());
        assert_eq!(kerbs_in(&cloud), all, "{h} m kerb at {x0} m");
    }
    // A post on the pavement beside the road: the kerb away from it stays, every point.
    let (px, py) = (BUMPER + 4.5, -2.0);
    let kerb = kerb_across(BUMPER + 4.0, 0.3, f64::INFINITY);
    let post = post_at(px, py);
    let cloud = cast(|x, y| kerb(x, y).max(post(x, y)), 0.03);
    let away: Vec<Point> = steps(&cloud, &StepShape::default())
        .into_iter()
        .filter(|p| (p.x - px).hypot(p.y - py) > 1.0)
        .collect();
    let k = kerbs_in(&cloud);
    assert!(
        away.len() >= 10,
        "{} kerb points away from the post",
        away.len()
    );
    assert!(away.iter().all(|p| k.contains(p)), "{k:?}");
    assert!(k.iter().all(|p| (p.x - px).hypot(p.y - py) > 0.25), "{k:?}");
}

#[test]
fn a_walker_crossing_under_every_beam_leaves_no_kerb_behind() {
    // TODO 189 (sim-check --moving, 20 Nov): the car stands; a person crosses the lane
    // 3 m past the bumper at 0.8 m/s, from 4 m right to 6 m left, and walks on. Their
    // foot, kept under every beam, stood across the lane for ever: the behaviour planner
    // slowed for it as a kerb (APPROACH(obstacle)) and the stop zone held the car.
    let blind = crate::Blind::under(LIDAR, crate::LOWEST_BEAM);
    let here = crate::Pose2::default();
    let mut kept = crate::StepMemory::new(10.0, 12.0, 0.1).with_blind(blind);
    let mut raw = crate::StepMemory::new(10.0, 12.0, 0.1).with_blind(blind);
    let mut t = 0.0;
    while t <= 12.5 {
        let cloud = cast(post_at(BUMPER + 3.0, -4.0 + 0.8 * t), 0.03);
        kept.update(t, here, &kerbs_in(&cloud));
        raw.update(t, here, &steps(&cloud, &StepShape::default()));
        t += 0.5;
    }
    // Gone by: the empty road for longer than the memory keeps anything.
    let road = cast(|_, _| 0.0, 0.03);
    let end = t + 11.0;
    while t <= end {
        kept.update(t, here, &kerbs_in(&road));
        raw.update(t, here, &steps(&road, &StepShape::default()));
        t += 0.5;
    }
    let left = kept.update(t, here, &[]);
    assert!(left.is_empty(), "{} kerb points left: {left:?}", left.len());
    // The old way, every step kept: the walker's foot under every beam never aged.
    assert!(
        !raw.update(t, here, &[]).is_empty(),
        "the old way kept nothing: the test shows nothing"
    );
}

//! The tracker on a pretend car, as aav_track's test_trackers.py drives the Python one:
//! the tracker at 20 Hz; ego's ramps and the shared car model (the spine's speed loop, the
//! actuators, the bicycle) at 100 Hz in between. No ROS.

use std::f64::consts::PI;
use std::time::Instant;

use car_model::{ActuatorConfig, CarState, Command as CarCommand, SpeedLoop, SpeedLoopProfile};
use mpc_lag::model::lags_scaled;
use mpc_lag::{wrap, CuspRules, Geared, Limits, MpcLag, Observed, Path, Settings, TrackError};

/// sim_car.py's LIMITS: ego's steering rate is the tighter one.
const LIMITS: Limits = Limits {
    wheelbase: 1.57,
    max_steer: 0.5,
    max_steer_rate: 0.6,
};
const PERIOD: f64 = 0.05;
const PHYSICS: f64 = 0.01;

/// Ego as the pretend car sees it (aav_ego EgoModel): the speed within its ramps, the
/// steering within its rate, held below 0.05 m/s.
struct Ego {
    speed: f64,
    steering: f64,
}

impl Ego {
    fn step(&mut self, v: f64, steer: f64, dt: f64) -> (f64, f64) {
        let target = if v.abs() > 0.05 { steer } else { self.steering };
        let target = target.clamp(-LIMITS.max_steer, LIMITS.max_steer);
        let v = v.clamp(0.0, 5.0);
        let dv = (v - self.speed).clamp(-2.0 * dt, 1.0 * dt);
        self.speed += dv;
        let most = 0.6 * dt;
        self.steering += (target - self.steering).clamp(-most, most);
        (self.speed, self.steering)
    }
}

/// Every tick: (t, x, y, yaw, speed, steer asked, cross track, speed asked).
#[derive(Default)]
struct Run {
    rows: Vec<[f64; 8]>,
    reached: bool,
    step_ms: Vec<f64>,
}

impl Run {
    fn worst_cross(&self) -> f64 {
        self.rows.iter().map(|r| r[6].abs()).fold(0.0, f64::max)
    }
    fn top_speed(&self) -> f64 {
        self.rows.iter().map(|r| r[4]).fold(0.0, f64::max)
    }
    /// The largest |y| any corner of the car's body reached (2 x 1 m, 0.2 m behind the axle).
    fn widest(&self) -> f64 {
        let body = [(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)];
        self.rows
            .iter()
            .flat_map(|r| {
                body.iter()
                    .map(move |(bx, by)| (r[2] + bx * r[3].sin() + by * r[3].cos()).abs())
            })
            .fold(0.0, f64::max)
    }
    /// Every steering within full lock and the rate; every speed asked within [0, cruise],
    /// changing within the throttle's 1 m/s² and the brake's 2.
    fn check_limits(&self, cruise: f64) {
        let mut last = (0.0, 0.0);
        for r in &self.rows {
            let (steer, asked) = (r[5], r[7]);
            assert!(steer.abs() <= LIMITS.max_steer + 1e-9);
            assert!((steer - last.0).abs() <= LIMITS.max_steer_rate * PERIOD + 1e-9);
            assert!((0.0..=cruise + 1e-9).contains(&asked));
            assert!(asked - last.1 <= 1.0 * PERIOD + 1e-9 && last.1 - asked <= 2.0 * PERIOD + 1e-9);
            last = (steer, asked);
        }
    }
}

fn tracker(cruise: f64) -> MpcLag {
    let settings = Settings {
        cruise,
        ..Settings::default()
    };
    MpcLag::new(LIMITS, ActuatorConfig::default(), settings).expect("a tracker")
}

/// Drive the pretend car along `poses` from the start, its lags `lag_scale` times
/// vehicle.yaml's (the tracker keeps vehicle.yaml's).
fn follow(
    mut mpc: MpcLag,
    poses: &[(f64, f64, f64)],
    start: (f64, f64, f64),
    lag_scale: f64,
) -> Run {
    let path = Path::new(poses).expect("a path");
    mpc.reset(path.clone());
    let car = lags_scaled(&ActuatorConfig::default(), lag_scale);
    let car = ActuatorConfig {
        // lags_scaled scales the dead times too: vehicle.yaml's are 0.
        wheelbase: LIMITS.wheelbase,
        ..car
    };
    let mut lp = SpeedLoop::new(SpeedLoopProfile::Default.config());
    let mut ego = Ego {
        speed: 0.0,
        steering: 0.0,
    };
    let mut state = CarState {
        x: start.0,
        y: start.1,
        yaw: start.2,
        ..CarState::default()
    };
    let (gx, gy) = path.end();
    let (mut t, mut hint, mut steer) = (0.0, 0, 0.0);
    let mut run = Run::default();
    let mut xy_done = false;
    while t < 120.0 {
        let foot = path.project((state.x, state.y), hint, 10.0);
        hint = foot.index;
        // Nav2's goal check: within 1 m, then the heading within 0.75 rad.
        xy_done |= (gx - state.x).hypot(gy - state.y) <= 1.0;
        if xy_done && wrap(path.goal_yaw() - state.yaw).abs() <= 0.75 {
            run.reached = true;
            return run;
        }
        let seen = Observed {
            x: state.x,
            y: state.y,
            yaw: state.yaw,
            v: state.v,
            steer,
        };
        let t0 = Instant::now();
        let cmd = mpc.step(&seen, PERIOD).expect("a command");
        run.step_ms.push(t0.elapsed().as_secs_f64() * 1000.0);
        steer = cmd.steer;
        run.rows.push([
            t, state.x, state.y, state.yaw, state.v, steer, foot.cross, cmd.speed,
        ]);
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            let (v, d) = ego.step(cmd.speed, steer, PHYSICS);
            state = car
                .step(&state, CarCommand::speed(v, d), PHYSICS, Some(&mut lp))
                .expect("a step");
            t += PHYSICS;
        }
    }
    run
}

fn straight(length: f64) -> Vec<(f64, f64, f64)> {
    let n = (length / 0.1) as usize;
    (0..=n).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect()
}

/// Straight on for 5 m, a left turn of `radius` through 90°, straight 10 m (sim_car.arc).
fn arc(radius: f64) -> Vec<(f64, f64, f64)> {
    let (lead, step) = (5.0, 0.1);
    let mut pts: Vec<_> = (0..(lead / step) as usize)
        .map(|i| (i as f64 * step, 0.0, 0.0))
        .collect();
    for i in 0..=((radius * PI / 2.0 / step) as usize) {
        let a = i as f64 * step / radius;
        pts.push((lead + radius * a.sin(), radius - radius * a.cos(), a));
    }
    let (ex, ey, h) = pts[pts.len() - 1];
    for i in 1..=100 {
        let d = i as f64 * step;
        pts.push((ex + d * h.cos(), ey + d * h.sin(), h));
    }
    pts
}

/// A forward-only U-turn, Smac's light bulb (sim_car.uturn): right 60°, left 300°, right
/// 60°, every bend 3.6 m; it comes back along y = 0.
fn uturn() -> Vec<(f64, f64, f64)> {
    let (radius, swing, step) = (3.6, PI / 3.0, 0.05);
    let (mut x, mut y, mut h): (f64, f64, f64) = (0.0, 0.0, 0.0);
    let mut pts = vec![(0.0, 0.0, 0.0)];
    for (length, k) in [
        (6.0, 0.0),
        (radius * swing, -1.0 / radius),
        (radius * (PI + 2.0 * swing), 1.0 / radius),
        (radius * swing, -1.0 / radius),
        (12.0, 0.0),
    ] {
        let n = ((length / step) as usize).max(1);
        let ds = length / n as f64;
        for _ in 0..n {
            if k == 0.0 {
                x += ds * h.cos();
                y += ds * h.sin();
            } else {
                let h2 = h + k * ds;
                x += (h2.sin() - h.sin()) / k;
                y -= (h2.cos() - h.cos()) / k;
                h = h2;
            }
            pts.push((x, y, h));
        }
    }
    pts
}

#[test]
fn a_straight_line_stays_on_it() {
    for cruise in [1.0, 3.0] {
        let run = follow(tracker(cruise), &straight(40.0), (0.0, 0.0, 0.0), 1.0);
        assert!(run.reached && run.worst_cross() < 0.01);
        run.check_limits(cruise);
    }
}

#[test]
fn it_holds_its_speed_without_the_loops_overshoot() {
    // The spine's PI overshoots a step of set speed by about 14 % (TODO 27). mpc_lag
    // predicts it and eases the set speed in.
    let run = follow(tracker(1.0), &straight(40.0), (0.0, 0.0, 0.0), 1.0);
    assert!(run.reached);
    assert!(run.top_speed() < 1.03, "top speed {}", run.top_speed());
}

#[test]
fn a_step_offset_converges() {
    for cruise in [1.0, 3.0] {
        let run = follow(tracker(cruise), &straight(40.0), (0.0, 1.0, 0.0), 1.0);
        assert!(run.reached);
        let late = run
            .rows
            .iter()
            .filter(|r| r[1] > 20.0)
            .map(|r| r[6].abs())
            .fold(0.0, f64::max);
        assert!(late < 0.05, "{late} m off at {cruise} m/s");
        assert!(run.rows.iter().all(|r| r[6] > -0.3));
        run.check_limits(cruise);
    }
}

#[test]
fn in_the_tube_it_holds_its_line_and_outside_it_comes_back() {
    let with_tube = |cruise| {
        let settings = Settings {
            cruise,
            tube: 0.2,
            ..Settings::default()
        };
        MpcLag::new(LIMITS, ActuatorConfig::default(), settings).expect("a tracker")
    };
    let steering_work = |run: &Run| {
        run.rows
            .windows(2)
            .map(|w| (w[1][5] - w[0][5]).abs())
            .sum::<f64>()
    };
    // 0.15 m off, inside the tube: no pull back to the centre, so it barely steers.
    let held = follow(with_tube(1.0), &straight(30.0), (0.0, 0.15, 0.0), 1.0);
    let pulled = follow(tracker(1.0), &straight(30.0), (0.0, 0.15, 0.0), 1.0);
    assert!(held.reached && pulled.reached);
    let late = |run: &Run| {
        run.rows
            .iter()
            .filter(|r| r[1] > 20.0)
            .map(|r| r[6])
            .sum::<f64>()
            / run.rows.iter().filter(|r| r[1] > 20.0).count() as f64
    };
    assert!(late(&held) > 0.1, "{} m: pulled to the centre", late(&held));
    assert!(late(&pulled).abs() < 0.05);
    assert!(steering_work(&held) < steering_work(&pulled));
    // 1 m off: back into the tube, and no further.
    for cruise in [1.0, 3.0] {
        let run = follow(with_tube(cruise), &straight(40.0), (0.0, 1.0, 0.0), 1.0);
        assert!(run.reached);
        let worst_late = run
            .rows
            .iter()
            .filter(|r| r[1] > 25.0)
            .map(|r| r[6].abs())
            .fold(0.0, f64::max);
        assert!(worst_late < 0.25, "{worst_late} m off at {cruise} m/s");
        run.check_limits(cruise);
    }
}

#[test]
fn a_curve_stays_close() {
    for cruise in [1.0, 3.0] {
        let run = follow(tracker(cruise), &arc(8.0), (0.0, 0.0, 0.0), 1.0);
        assert!(run.reached);
        assert!(
            run.worst_cross() < 0.05,
            "{} m at {cruise} m/s",
            run.worst_cross()
        );
        run.check_limits(cruise);
    }
}

#[test]
fn the_u_turn_at_2_m_s_with_the_lags_and_with_them_doubled() {
    // aav_track's bounds for the Python: 0.12 m with vehicle.yaml's lags, 0.25 m with the
    // car's lags twice what it was told.
    for (scale, bound) in [(1.0, 0.12), (2.0, 0.25)] {
        let run = follow(tracker(2.0), &uturn(), (0.0, 0.0, 0.0), scale);
        assert!(run.reached && run.widest() < 5.5);
        run.check_limits(2.0);
        let worst = run.worst_cross();
        println!("u-turn at 2 m/s, lags x{scale}: {worst:.3} m off the path");
        assert!(worst < bound, "{worst} m with the lags x{scale}");
    }
}

#[test]
fn it_stops_on_the_goal() {
    let run = follow(tracker(3.0), &straight(12.0), (0.0, 0.0, 0.0), 1.0);
    assert!(run.reached);
    for r in &run.rows {
        // Within the planned braking (half of 3 m/s²) of the end, and a little.
        assert!(r[7] <= (2.0 * 1.5 * (12.0 - r[1]).max(0.0)).sqrt() + 0.15);
    }
    run.check_limits(3.0);
}

#[test]
fn it_keeps_the_commands_inside_a_dead_time() {
    // 0.1 s of steering dead time and the 0.1 s chain: four ticks of commands in its state.
    let car = ActuatorConfig {
        steer_dead_time: 0.1,
        throttle_dead_time: 0.05,
        ..ActuatorConfig::default()
    };
    let mut mpc = MpcLag::new(LIMITS, car, Settings::default()).expect("a tracker");
    assert_eq!(mpc.dead_ticks(), (4, 3));
    mpc.reset(Path::new(&straight(40.0)).expect("a path"));
    for _ in 0..3 {
        mpc.step(&Observed::default(), PERIOD).expect("a command");
    }
    assert_eq!(mpc.queued(), (4, 3));
}

#[test]
fn it_plans_well_inside_a_tick() {
    let run = follow(tracker(2.0), &uturn(), (0.0, 0.0, 0.0), 1.0);
    let mean = run.step_ms.iter().sum::<f64>() / run.step_ms.len() as f64;
    let worst = run.step_ms.iter().copied().fold(0.0, f64::max);
    println!("mpc_lag (Rust) step: {mean:.2} ms mean, {worst:.1} ms worst");
    assert!(mean < 10.0, "{mean} ms a tick");
}

#[test]
fn it_refuses_bad_numbers() {
    let bad = |s: Settings| MpcLag::new(LIMITS, ActuatorConfig::default(), s).err();
    assert_eq!(
        bad(Settings {
            horizon: 1,
            ..Settings::default()
        }),
        Some(TrackError::Setting("horizon"))
    );
    assert_eq!(
        bad(Settings {
            latency: -0.1,
            ..Settings::default()
        }),
        Some(TrackError::Setting("latency"))
    );
    assert!(matches!(
        bad(Settings {
            speed_loop: SpeedLoopProfile::Lqi,
            ..Settings::default()
        }),
        Some(TrackError::Loop(_))
    ));
    let mut mpc = tracker(1.0);
    assert_eq!(
        mpc.step(&Observed::default(), PERIOD).err(),
        Some(TrackError::NoPath)
    );
}

// --- Forward and back (TODO 213) ---------------------------------------------------------

/// m/s: vehicle.yaml's max_reverse_speed.
const REVERSE: f64 = 0.5;

/// Ego both ways: the speed within its ramps (through 0 at a change of direction), the
/// steering within its rate, held below 0.05 m/s.
struct SignedEgo {
    speed: f64,
    steering: f64,
}

impl SignedEgo {
    fn step(&mut self, v: f64, steer: f64, dt: f64) -> (f64, f64) {
        let target = if v.abs() > 0.05 { steer } else { self.steering };
        let target = target.clamp(-LIMITS.max_steer, LIMITS.max_steer);
        let v = v.clamp(-REVERSE, 5.0);
        // Slowing (toward 0) at the brake's 2 m/s², speeding up at 1.
        let slowing = (v - self.speed) * self.speed.signum() < 0.0;
        let most = if slowing { 2.0 * dt } else { 1.0 * dt };
        self.speed += (v - self.speed).clamp(-most, most);
        let most = 0.6 * dt;
        self.steering += (target - self.steering).clamp(-most, most);
        (self.speed, self.steering)
    }
}

/// A geared run: every tick (t, x, y, yaw, speed, steer, cross track, speed asked,
/// stretch); and where and how fast the car was at each change of stretch.
#[derive(Default)]
struct GearedRun {
    rows: Vec<[f64; 9]>,
    changes: Vec<(f64, f64, f64)>,
    reached: bool,
}

/// Drive `poses` with its cusps as the node does: `Geared` on the pretend car (signed).
fn follow_geared(poses: &[(f64, f64, f64)], cruise: f64) -> GearedRun {
    let settings = Settings {
        cruise,
        ..Settings::default()
    };
    let mpc = MpcLag::new(LIMITS, ActuatorConfig::default(), settings).expect("a tracker");
    let mut g = Geared::new(mpc, REVERSE, CuspRules::default());
    g.follow(poses).expect("a path");
    let car = ActuatorConfig {
        wheelbase: LIMITS.wheelbase,
        ..ActuatorConfig::default()
    };
    let mut lp = SpeedLoop::new(SpeedLoopProfile::Default.config());
    let mut ego = SignedEgo {
        speed: 0.0,
        steering: 0.0,
    };
    let mut state = CarState::default();
    let (gx, gy, gyaw) = g.goal().expect("a goal");
    let (mut t, mut steer, mut xy_done) = (0.0, 0.0, false);
    let mut run = GearedRun::default();
    while t < 120.0 {
        if g.advance(state.x, state.y, state.v, t) {
            run.changes.push((state.x, state.y, state.v));
        }
        if g.on_last() {
            xy_done |= (gx - state.x).hypot(gy - state.y) <= 1.0;
            if xy_done && wrap(gyaw - state.yaw).abs() <= 0.75 {
                run.reached = true;
                return run;
            }
        }
        let seen = Observed {
            x: state.x,
            y: state.y,
            yaw: state.yaw,
            v: state.v,
            steer,
        };
        let cmd = g.step(&seen, PERIOD).expect("a command");
        steer = cmd.steer;
        let cross = cmd.report.foot.map_or(0.0, |f| f.cross);
        let stretch = g.gears().map_or(0, |gs| gs.index()) as f64;
        run.rows.push([
            t, state.x, state.y, state.yaw, state.v, steer, cross, cmd.speed, stretch,
        ]);
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            let (v, d) = ego.step(cmd.speed, steer, PHYSICS);
            state = car
                .step(&state, CarCommand::speed(v, d), PHYSICS, Some(&mut lp))
                .expect("a step");
            t += PHYSICS;
        }
    }
    run
}

/// Backing up 8 m along -x: the poses face +x (the car's way), the path runs -x.
fn back_straight() -> Vec<(f64, f64, f64)> {
    (0..=80).map(|i| (-(i as f64) * 0.1, 0.0, 0.0)).collect()
}

/// 6 m east, a cusp, then backwards round a quarter circle of 5 m to the left (aav_track's
/// sim_car.forward_then_back): it ends 5 m behind the cusp and 5 m left, facing south.
fn forward_then_back() -> Vec<(f64, f64, f64)> {
    let (ahead, radius, step) = (6.0, 5.0, 0.1);
    let mut pts: Vec<_> = (0..=60).map(|i| (i as f64 * step, 0.0, 0.0)).collect();
    let n = (radius * PI / 2.0 / step) as usize;
    for i in 1..=n {
        let a = i as f64 * step / radius;
        pts.push((ahead - radius * a.sin(), radius - radius * a.cos(), -a));
    }
    pts
}

#[test]
fn it_backs_up_a_straight_line() {
    let run = follow_geared(&back_straight(), 1.0);
    assert!(run.reached);
    assert!(run.changes.is_empty());
    for r in &run.rows {
        assert!((-REVERSE - 1e-9..=0.0).contains(&r[7]), "asked {}", r[7]);
        assert!(
            r[4] >= -0.65,
            "the speed loop overshoots a little: {}",
            r[4]
        );
        assert!(r[6].abs() < 0.05, "{} m off the line", r[6]);
    }
}

#[test]
fn it_stops_at_the_cusp_then_backs_round_a_bend() {
    let run = follow_geared(&forward_then_back(), 1.0);
    assert!(run.reached, "the goal");
    // One change of gear, at the cusp (6, 0), standing.
    assert_eq!(run.changes.len(), 1);
    let (x, y, v) = run.changes[0];
    println!("at the cusp: ({x:.2}, {y:.2}) at {v:.3} m/s");
    assert!(
        (x - 6.0).hypot(y) < 0.3,
        "the gear changed at {x:.2}, {y:.2}"
    );
    assert!(v.abs() <= 0.05, "the gear changed at {v} m/s");
    let forward: Vec<_> = run.rows.iter().filter(|r| r[8] == 0.0).collect();
    let back: Vec<_> = run.rows.iter().filter(|r| r[8] == 1.0).collect();
    assert!(!forward.is_empty() && !back.is_empty());
    assert!(forward.iter().all(|r| r[7] >= 0.0) && back.iter().all(|r| r[7] <= 0.0));
    assert!(back.iter().all(|r| r[7] >= -REVERSE - 1e-9));
    // Never further past the cusp than the stretch's own stop allows.
    assert!(forward.iter().all(|r| r[1] < 6.3));
    // Close round the bend backwards (after the first quarter: it starts from a stop with
    // the wheels straight; ego holds them while it stands).
    let worst = back[back.len() / 4..]
        .iter()
        .map(|r| r[6].abs())
        .fold(0.0, f64::max);
    println!("backwards round the 5 m bend: {worst:.3} m off the path");
    assert!(worst < 0.25, "{worst} m off backwards");
}

#[test]
fn it_goes_forward_back_and_forward_again() {
    // A three-point turn's shape: 4 m east, 3 m back, 4 m east again.
    let mut p: Vec<_> = (0..=40).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
    p.extend((1..=30).map(|i| (4.0 - i as f64 * 0.1, 0.0, 0.0)));
    p.extend((1..=40).map(|i| (1.0 + i as f64 * 0.1, 0.0, 0.0)));
    let run = follow_geared(&p, 1.0);
    assert!(run.reached);
    assert_eq!(run.changes.len(), 2);
    for (&(x, _, v), cusp) in run.changes.iter().zip([4.0, 1.0]) {
        println!("cusp at {cusp} m: changed at {x:.2} m, {v:.3} m/s");
        assert!((x - cusp).abs() < 0.3 && v.abs() <= 0.05, "at {x}, {v} m/s");
    }
}

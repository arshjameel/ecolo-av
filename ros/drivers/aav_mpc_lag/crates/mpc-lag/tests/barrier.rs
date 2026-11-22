//! The barriers on a pretend car (research, separate from the capstone; 22 Nov; the
//! package's README, "Barriers"): the
//! tracker at 20 Hz, ego's ramps and the shared car model at 100 Hz in between, as
//! `closed_loop.rs`. Each test asks one thing of the car itself, not of the plan: did its
//! body stay out of the object, and did H keep its rule from tick to tick?

use car_model::{ActuatorConfig, CarState, Command as CarCommand, SpeedLoop, SpeedLoopProfile};
use ego_core::stop_law::StopCurve;
use mpc_lag::barrier::decays_no_faster;
use mpc_lag::{Circle, Limits, MpcLag, Observed, Path, Rules, Settings};

const LIMITS: Limits = Limits {
    wheelbase: 1.57,
    max_steer: 0.5,
    max_steer_rate: 0.6,
};
const PERIOD: f64 = 0.05;
const PHYSICS: f64 = 0.01;
/// The car's body about base_link, m: 2 x 1, 0.2 behind the axle (closed_loop.rs's).
const BODY: [(f64, f64); 4] = [(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)];
/// From the body's middle (0.8 m ahead of base_link) to a corner, m.
const BODY_RADIUS: f64 = 1.118_034;

/// A round object of the world.
#[derive(Clone, Copy)]
struct Object {
    x: f64,
    y: f64,
    radius: f64,
}

impl Object {
    /// Its keep-out circle on `path`: the object, the body's own radius, a margin.
    fn circle(&self, path: &Path, margin: f64) -> Circle {
        Circle::from_world(path, self.x, self.y, self.radius + BODY_RADIUS + margin)
    }

    /// m from the object's edge to the nearest point of the car's body (below 0: hit).
    fn gap(&self, x: f64, y: f64, yaw: f64) -> f64 {
        let (sin, cos) = yaw.sin_cos();
        // The object's centre in the car's frame, then the nearest point of the rectangle.
        let (dx, dy) = (self.x - x, self.y - y);
        let (fx, fy) = (dx * cos + dy * sin, -dx * sin + dy * cos);
        let (nx, ny) = (
            fx.clamp(BODY[2].0, BODY[0].0),
            fy.clamp(BODY[1].1, BODY[0].1),
        );
        (fx - nx).hypot(fy - ny) - self.radius
    }
}

/// What a run saw, a tick each: (t, x, y, yaw, v, cross track, H, the object's gap).
#[derive(Default)]
struct Run {
    rows: Vec<[f64; 8]>,
    reached: bool,
    unsolved: usize,
    worst_slack: f64,
    stops: usize,
    step_ms: Vec<f64>,
}

impl Run {
    fn least_gap(&self) -> f64 {
        self.rows.iter().map(|r| r[7]).fold(f64::INFINITY, f64::min)
    }
    fn least_h(&self) -> f64 {
        self.rows.iter().map(|r| r[6]).fold(f64::INFINITY, f64::min)
    }
    fn widest(&self) -> f64 {
        self.rows.iter().map(|r| r[5].abs()).fold(0.0, f64::max)
    }
    fn last(&self) -> &[f64; 8] {
        self.rows.last().expect("a tick")
    }
    /// Ticks where the car's own H fell faster than the rule allows (beyond `tolerance`).
    fn rule_breaks(&self, gamma: f64, tolerance: f64) -> usize {
        self.rows
            .windows(2)
            .filter(|w| !decays_no_faster(w[0][6], w[1][6], gamma, tolerance))
            .count()
    }
    /// m along x where the car first left the path by 5 cm.
    fn turned_away_at(&self) -> Option<f64> {
        self.rows.iter().find(|r| r[5].abs() > 0.05).map(|r| r[1])
    }
}

/// The rules with the pretend car's own stopping law: vehicle.yaml's brake, ego's
/// deceleration and jerk, the spine's default speed loop.
fn rules() -> Rules {
    Rules::new(StopCurve::from_numbers(3.0, 2.0, 2.0).expect("a stopping law"))
}

fn straight(length: f64) -> Vec<(f64, f64, f64)> {
    (0..=(length / 0.1) as usize)
        .map(|i| (i as f64 * 0.1, 0.0, 0.0))
        .collect()
}

/// Drive the pretend car along `poses` past `object`. `rules`: `None` for the tracker as
/// it always was. `dead_time`: s on the car's steering and pedals, known to the tracker.
fn drive(
    poses: &[(f64, f64, f64)],
    object: Object,
    rules: Option<Rules>,
    cruise: f64,
    dead_time: f64,
    seconds: f64,
) -> Run {
    let path = Path::new(poses).expect("a path");
    let car = ActuatorConfig {
        wheelbase: LIMITS.wheelbase,
        steer_dead_time: dead_time,
        throttle_dead_time: dead_time,
        brake_dead_time: dead_time,
        ..ActuatorConfig::default()
    };
    let settings = Settings {
        cruise,
        ..Settings::default()
    };
    let mut mpc = MpcLag::new(LIMITS, car, settings).expect("a tracker");
    mpc.reset(path.clone());
    let circle = object.circle(&path, 0.2);
    if let Some(rules) = rules {
        mpc.set_barriers(rules, &[circle]).expect("barriers");
    }
    let ahead = rules.unwrap_or_else(self::rules).ahead;
    let mut lp = SpeedLoop::new(SpeedLoopProfile::Default.config());
    let (mut ego_v, mut ego_steer) = (0.0_f64, 0.0_f64);
    let mut state = CarState::default();
    let (gx, gy) = path.end();
    let (mut t, mut hint, mut steer) = (0.0, 0, 0.0);
    let mut run = Run::default();
    while t < seconds {
        let foot = path.project((state.x, state.y), hint, 10.0);
        hint = foot.index;
        if (gx - state.x).hypot(gy - state.y) <= 1.0 {
            run.reached = true;
            break;
        }
        let seen = Observed {
            x: state.x,
            y: state.y,
            yaw: state.yaw,
            v: state.v,
            steer,
        };
        let cmd = mpc.step(&seen, PERIOD).expect("a command");
        run.step_ms.push(cmd.report.solve_ms);
        steer = cmd.steer;
        run.unsolved += usize::from(!cmd.report.solved);
        run.worst_slack = run.worst_slack.max(cmd.report.barrier_slack);
        run.stops += usize::from(cmd.report.barrier_stop);
        let h = circle.h(foot.cross, state.yaw - foot.heading, foot.s, ahead);
        run.rows.push([
            t,
            state.x,
            state.y,
            state.yaw,
            state.v,
            foot.cross,
            h,
            object.gap(state.x, state.y, state.yaw),
        ]);
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            // Ego: the speed within its ramps, the steering within its rate.
            ego_v += (cmd.speed.clamp(0.0, 5.0) - ego_v).clamp(-2.0 * PHYSICS, PHYSICS);
            let target = if cmd.speed.abs() > 0.05 {
                steer
            } else {
                ego_steer
            };
            ego_steer += (target.clamp(-LIMITS.max_steer, LIMITS.max_steer) - ego_steer)
                .clamp(-0.6 * PHYSICS, 0.6 * PHYSICS);
            state = car
                .step(
                    &state,
                    CarCommand::speed(ego_v, ego_steer),
                    PHYSICS,
                    Some(&mut lp),
                )
                .expect("a step");
            t += PHYSICS;
        }
    }
    run
}

#[test]
#[ignore = "prints the numbers: cargo test --release -p mpc-lag --test barrier -- --ignored --nocapture"]
fn the_numbers() {
    let line = straight(40.0);
    for (name, object) in [
        (
            "off-centre 0.4",
            Object {
                x: 15.0,
                y: 0.4,
                radius: 0.5,
            },
        ),
        (
            "dead centre",
            Object {
                x: 15.0,
                y: 0.0,
                radius: 0.5,
            },
        ),
        (
            "beside 1.5",
            Object {
                x: 15.0,
                y: 1.5,
                radius: 0.5,
            },
        ),
    ] {
        for cruise in [1.0, 3.0] {
            for (label, rules, dead) in [
                ("none      ", None, 0.0),
                (
                    "gamma 0.02",
                    Some(Rules {
                        gamma: 0.02,
                        ..rules()
                    }),
                    0.0,
                ),
                (
                    "gamma 0.03",
                    Some(Rules {
                        gamma: 0.03,
                        ..rules()
                    }),
                    0.0,
                ),
                ("gamma 0.05", Some(rules()), 0.0),
                (
                    "gamma 0.2 ",
                    Some(Rules {
                        gamma: 0.2,
                        ..rules()
                    }),
                    0.0,
                ),
                (
                    "gamma 1.0 ",
                    Some(Rules {
                        gamma: 1.0,
                        ..rules()
                    }),
                    0.0,
                ),
                ("0.05 +0.2s", Some(rules()), 0.2),
            ] {
                let gamma = rules.map_or(0.05, |r| r.gamma);
                let run = drive(&line, object, rules, cruise, dead, 80.0);
                println!(
                    "{name:15} {cruise} m/s {label}: reached {:5} gap {:6.3} m  H {:7.3}  widest {:5.2} m  \
                     turned at x {:5.1}  breaks {:3}  slack {:8.4}  unsolved {:3}  stops {:4}  end x {:5.1} v {:4.2}  tick {:4.2} ms (worst {:4.1})",
                    run.reached,
                    run.least_gap(),
                    run.least_h(),
                    run.widest(),
                    run.turned_away_at().unwrap_or(f64::NAN),
                    run.rule_breaks(gamma, 1e-3),
                    run.worst_slack,
                    run.unsolved,
                    run.stops,
                    run.last()[1],
                    run.last()[4],
                    run.step_ms.iter().sum::<f64>() / run.step_ms.len() as f64,
                    run.step_ms.iter().copied().fold(0.0, f64::max),
                );
            }
        }
    }
}

/// On the path's line, a little to its left; right on it; and beside it, its keep-out
/// circle 0.3 m over the line.
const OBJECTS: [Object; 3] = [
    Object {
        x: 15.0,
        y: 0.4,
        radius: 0.5,
    },
    Object {
        x: 15.0,
        y: 0.0,
        radius: 0.5,
    },
    Object {
        x: 15.0,
        y: 1.5,
        radius: 0.5,
    },
];

fn with_gamma(gamma: f64) -> Option<Rules> {
    Some(Rules { gamma, ..rules() })
}

#[test]
fn without_barriers_it_drives_through_and_with_them_it_never_touches() {
    let line = straight(40.0);
    for object in &OBJECTS[..2] {
        let plain = drive(&line, *object, None, 3.0, 0.0, 60.0);
        assert!(
            plain.reached && plain.least_gap() < -0.4,
            "the path goes through it"
        );
    }
    // Every object, slow and fast, with and without a dead time the tracker knows.
    for object in OBJECTS {
        for cruise in [1.0, 3.0] {
            for dead_time in [0.0, 0.2] {
                let run = drive(&line, object, Some(rules()), cruise, dead_time, 60.0);
                assert!(
                    run.least_gap() > 0.15,
                    "{} m at y {} {cruise} m/s dead {dead_time}",
                    run.least_gap(),
                    object.y
                );
                // The keep-out circle itself: its 0.2 m margin is barely used.
                assert!(run.least_h() > -0.05, "H {}", run.least_h());
            }
        }
    }
}

#[test]
fn at_speed_it_goes_round_and_comes_back_to_the_path() {
    for object in OBJECTS {
        let run = drive(&straight(40.0), object, Some(rules()), 3.0, 0.0, 60.0);
        assert!(run.reached, "y {}", object.y);
        assert!(run.widest() < 2.0, "{} m off the path", run.widest());
        // Back on the line by the end.
        assert!(
            run.last()[5].abs() < 0.1,
            "{} m off at the end",
            run.last()[5]
        );
    }
}

#[test]
fn slowly_and_head_on_it_stops_short_instead_safe_and_stuck() {
    // At 1 m/s the plan sees 1.5 m: too late to steer round a thing 1.8 m wide. The
    // barrier slows it, the stop rule stands it still. Safe, and going nowhere: choosing
    // a side is the planner's job above.
    let run = drive(&straight(40.0), OBJECTS[1], Some(rules()), 1.0, 0.0, 60.0);
    assert!(!run.reached && run.stops > 0);
    assert!(run.last()[4] < 0.01, "it stands still");
    assert!(run.least_gap() > 0.15 && run.least_h() > 0.0);
    assert!(
        run.last()[1] > 12.0,
        "and not far short: x {}",
        run.last()[1]
    );
}

#[test]
fn a_smaller_gamma_turns_away_sooner_and_wider() {
    let turn = |gamma| {
        let run = drive(
            &straight(40.0),
            OBJECTS[0],
            with_gamma(gamma),
            3.0,
            0.0,
            60.0,
        );
        assert!(run.reached);
        (run.turned_away_at().expect("it turned"), run.least_gap())
    };
    let (early, wide) = turn(0.02);
    let (late, close) = turn(0.05);
    assert!(early < late - 2.0, "{early} against {late} m");
    assert!(wide > close + 0.5, "{wide} against {close} m");
}

#[test]
fn the_stop_rule_alone_still_keeps_it_out() {
    // gamma 1: the barrier asks only "never inside", as late as a distance limit. The
    // plan is then too late to steer; the stop rule is what holds.
    for cruise in [1.0, 3.0] {
        let run = drive(
            &straight(40.0),
            OBJECTS[1],
            with_gamma(1.0),
            cruise,
            0.0,
            60.0,
        );
        assert!(
            run.stops > 0 && run.least_gap() > 0.15,
            "{}",
            run.least_gap()
        );
    }
}

#[test]
fn a_far_object_changes_nothing() {
    let far = Object {
        x: 15.0,
        y: 8.0,
        radius: 0.5,
    };
    let plain = drive(&straight(40.0), far, None, 3.0, 0.0, 60.0);
    let fenced = drive(&straight(40.0), far, Some(rules()), 3.0, 0.0, 60.0);
    assert!(fenced.reached && fenced.stops == 0 && fenced.worst_slack < 1e-6);
    assert!(
        fenced.unsolved <= plain.unsolved + 5,
        "the QP as easy as without"
    );
    assert!(fenced.widest() < 0.01);
    assert!(
        (fenced.last()[0] - plain.last()[0]).abs() < 0.11,
        "as quick as without"
    );
}

#[test]
fn bad_rules_and_circles_are_refused() {
    let mut mpc =
        MpcLag::new(LIMITS, ActuatorConfig::default(), Settings::default()).expect("a tracker");
    let circle = Circle {
        s: 5.0,
        e_y: 0.0,
        radius: 1.0,
    };
    for rules in [
        Rules {
            gamma: 0.0,
            ..rules()
        },
        Rules {
            gamma: 1.5,
            ..rules()
        },
        Rules {
            weight: 0.0,
            ..rules()
        },
    ] {
        assert!(mpc.set_barriers(rules, &[circle]).is_err());
    }
    let flat = Circle {
        radius: 0.0,
        ..circle
    };
    assert!(mpc.set_barriers(rules(), &[flat]).is_err());
    assert!(mpc.set_barriers(rules(), &[circle]).is_ok());
    assert_eq!(mpc.barriers().len(), 1);
}

#[test]
fn a_tick_with_a_barrier_fits_well_inside_the_period() {
    // 20 Hz: 50 ms a tick. Mean and worst over a run that goes round the object.
    let run = drive(&straight(40.0), OBJECTS[0], Some(rules()), 3.0, 0.0, 60.0);
    let mean = run.step_ms.iter().sum::<f64>() / run.step_ms.len() as f64;
    let worst = run.step_ms.iter().copied().fold(0.0, f64::max);
    println!("a tick with a barrier: {mean:.2} ms mean, {worst:.1} ms worst");
    assert!(mean < 25.0, "{mean} ms");
}

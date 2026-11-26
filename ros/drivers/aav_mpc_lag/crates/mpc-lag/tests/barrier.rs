//! The barriers on a pretend car (outside the capstone's scope: Ben's MASc research; 22 Nov; the
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

/// A timing check, so on a quiet machine only: a busy one (a build beside it, a shared CI
/// runner) measured 53.7 ms mean, 403 ms worst on 25 Nov, and one GitHub ARM run failed.
/// `cargo test --release -p mpc-lag --test barrier a_tick -- --ignored`.
#[test]
#[ignore = "timing: run it alone on a quiet machine (--ignored)"]
fn a_tick_with_a_barrier_fits_well_inside_the_period() {
    // 20 Hz: 50 ms a tick. Mean and worst over a run that goes round the object.
    let run = drive(&straight(40.0), OBJECTS[0], Some(rules()), 3.0, 0.0, 60.0);
    let mean = run.step_ms.iter().sum::<f64>() / run.step_ms.len() as f64;
    let worst = run.step_ms.iter().copied().fold(0.0, f64::max);
    println!("a tick with a barrier: {mean:.2} ms mean, {worst:.1} ms worst");
    assert!(mean < 25.0, "{mean} ms");
}

/// Every tick of a few runs as CSV, for the graphs (`tools/barrier_graphs.py`):
/// `AAV_BARRIER_OUT=<folder> cargo test --release -p mpc-lag --test barrier the_traces -- --ignored`.
#[test]
#[ignore = "writes CSV traces to $AAV_BARRIER_OUT"]
fn the_traces() {
    let Ok(out) = std::env::var("AAV_BARRIER_OUT") else {
        println!("set AAV_BARRIER_OUT to a folder");
        return;
    };
    let line = straight(40.0);
    let cases = [
        ("plain", None),
        ("limit", with_gamma(1.0)),
        ("g005", with_gamma(0.05)),
        ("g002", with_gamma(0.02)),
    ];
    for (scene, object, cruise) in [
        ("left_fast", OBJECTS[0], 3.0),
        ("centre_slow", OBJECTS[1], 1.0),
        ("centre_fast", OBJECTS[1], 3.0),
    ] {
        for (name, rules) in cases {
            let run = drive(&line, object, rules, cruise, 0.0, 40.0);
            let mut text = String::from("t,x,y,yaw,v,cross,h,gap\n");
            for r in &run.rows {
                let row: Vec<String> = r.iter().map(|v| format!("{v:.4}")).collect();
                text.push_str(&row.join(","));
                text.push('\n');
            }
            std::fs::write(format!("{out}/{scene}_{name}.csv"), text).expect("a file");
        }
    }
    // A step: the car starts 1 m off the path, the object far away. With barriers on and
    // nothing near, the answer to the step is the plain tracker's.
    let far = Object {
        x: 15.0,
        y: 9.0,
        radius: 0.5,
    };
    for (name, rules) in [("plain", None), ("g005", with_gamma(0.05))] {
        let path: Vec<_> = line.iter().map(|&(x, _, h)| (x, 1.0, h)).collect();
        let run = drive(&path, far, rules, 3.0, 0.0, 40.0);
        let mut text = String::from("t,x,y,yaw,v,cross,h,gap\n");
        for r in &run.rows {
            let row: Vec<String> = r.iter().map(|v| format!("{v:.4}")).collect();
            text.push_str(&row.join(","));
            text.push('\n');
        }
        std::fs::write(format!("{out}/step_{name}.csv"), text).expect("a file");
    }
}

/// A left bend of `radius` m, `length` m long.
fn bend(radius: f64, length: f64) -> Vec<(f64, f64, f64)> {
    (0..=(length / 0.1) as usize)
        .map(|i| {
            let turn = i as f64 * 0.1 / radius;
            (radius * turn.sin(), radius * (1.0 - turn.cos()), turn)
        })
        .collect()
}

/// One run of the sweep: which set it belongs to, the road, the object, the numbers.
#[derive(Clone, Copy)]
struct Job {
    set: &'static str,
    case: &'static str,
    rules: Option<Rules>,
    bend: bool,
    speed: f64,
    offset: f64,
    dead_time: f64,
}

impl Job {
    /// The line of `sweep.csv` this run makes.
    fn run(&self) -> String {
        const RADIUS: f64 = 25.0;
        let (poses, object) = if self.bend {
            // The object 15 m along the bend, `offset` to the left of it.
            let turn = 15.0 / RADIUS;
            let from_centre = RADIUS - self.offset;
            let object = Object {
                x: from_centre * turn.sin(),
                y: RADIUS - from_centre * turn.cos(),
                radius: 0.5,
            };
            (bend(RADIUS, 40.0), object)
        } else {
            let object = Object {
                x: 15.0,
                y: self.offset,
                radius: 0.5,
            };
            (straight(40.0), object)
        };
        // Time for the whole road at this speed and half as much again: a run that is
        // still short of the end then has stopped, not run out of time.
        let seconds = 60.0 / self.speed + 15.0;
        let run = drive(
            &poses,
            object,
            self.rules,
            self.speed,
            self.dead_time,
            seconds,
        );
        let gap = run.least_gap();
        let ended = match (gap < 0.0, run.reached) {
            (true, _) => "touched",
            (false, true) => "reached",
            (false, false) if run.last()[4] < 0.05 => "stopped",
            (false, false) => "moving",
        };
        let mut ms = run.step_ms.clone();
        ms.sort_by(f64::total_cmp);
        format!(
            "{},{},{:.2},{:.2},{:.2},{:.4},{},{:.2},{:.3},{},{},{:.4},{:.3},{:.3}\n",
            self.set,
            self.case,
            self.speed,
            self.offset,
            self.dead_time,
            gap,
            ended,
            run.last()[0],
            run.widest(),
            run.unsolved,
            run.rows.len(),
            run.worst_slack,
            ms[ms.len() / 2],
            ms[ms.len() - 1],
        )
    }
}

/// Hundreds of runs, one line each, for the graphs' second half: the cruise speed, where
/// the object stands, the car's dead time and a bend, each swept for the plain tracker, a
/// distance limit alone (share 1: the step) and two barriers.
/// `AAV_BARRIER_OUT=<folder> cargo test --release -p mpc-lag --test barrier the_sweep -- --ignored`.
#[test]
#[ignore = "writes sweep.csv to $AAV_BARRIER_OUT"]
fn the_sweep() {
    let Ok(out) = std::env::var("AAV_BARRIER_OUT") else {
        println!("set AAV_BARRIER_OUT to a folder");
        return;
    };
    let cases = [
        ("plain", None),
        ("limit", with_gamma(1.0)),
        ("g005", with_gamma(0.05)),
        ("g002", with_gamma(0.02)),
    ];
    let steps = |from: f64, by: f64, count: usize| (0..count).map(move |i| from + by * i as f64);
    let mut jobs = Vec::new();
    for (case, rules) in cases {
        let job = Job {
            set: "",
            case,
            rules,
            bend: false,
            speed: 3.0,
            offset: 0.0,
            dead_time: 0.0,
        };
        jobs.extend(steps(0.5, 0.25, 15).map(|speed| Job {
            set: "speed",
            speed,
            ..job
        }));
        jobs.extend(steps(0.5, 0.25, 15).map(|speed| Job {
            set: "bend",
            bend: true,
            speed,
            ..job
        }));
        for (set, speed) in [("offset_fast", 3.0), ("offset_slow", 1.0)] {
            jobs.extend(steps(-1.6, 0.1, 33).map(|offset| Job {
                set,
                speed,
                offset,
                ..job
            }));
        }
        jobs.extend(steps(0.0, 0.05, 9).map(|dead_time| Job {
            set: "dead_time",
            dead_time,
            ..job
        }));
    }
    let next = std::sync::atomic::AtomicUsize::new(0);
    let workers = std::thread::available_parallelism().map_or(4, |n| usize::from(n) / 2 + 1);
    let lines: Vec<(usize, String)> = std::thread::scope(|scope| {
        let hands: Vec<_> = (0..workers)
            .map(|_| {
                scope.spawn(|| {
                    let mut mine = Vec::new();
                    loop {
                        let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        let Some(job) = jobs.get(i) else { break };
                        mine.push((i, job.run()));
                    }
                    mine
                })
            })
            .collect();
        hands
            .into_iter()
            .flat_map(|h| h.join().expect("a worker"))
            .collect()
    });
    let mut lines = lines;
    lines.sort_by_key(|(i, _)| *i);
    let mut text = String::from(
        "set,case,speed,offset,dead_time,gap,ended,seconds,widest,unsolved,ticks,slack,ms_median,ms_worst\n",
    );
    text.extend(lines.into_iter().map(|(_, line)| line));
    std::fs::write(format!("{out}/sweep.csv"), text).expect("a file");
    println!("{} runs", jobs.len());
}

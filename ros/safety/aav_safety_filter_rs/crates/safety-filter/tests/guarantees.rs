//! The filter's promises, checked on thousands of random, hostile drivers (proptest).
//!
//! | Promise | Checked |
//! |---|---|
//! | it never speeds a command up, nor turns a stop or a direction around | every tick |
//! | the car never passes the road's end | every tick, with a car that lags |
//! | the car's body never crosses a lane line | every tick, with a car that lags |
//! | without the filter, the same drivers do break these | so the drivers are hostile enough |
//!
//! The car here lags (speed 0.3 s, steering 0.8 rad/s), which the filter's model leaves
//! out: the margins are what absorb it. Change a setting and these say if a margin is gone.
//!
//! `guarantees.proptest-regressions` holds the cases that once failed; proptest runs them
//! first. It is in git on purpose: each is a finding (the README, "What the tests found").

use ego_core::stop_law::StopCurve;
use proptest::prelude::*;
use safety_filter::sim::{bending_lane, run, straight_lane, Car, CarModel};
use safety_filter::{Command, Lane, SafetyFilter, Settings};

/// m from the line to each lane line in the tests.
const HALF_LANE: f64 = 1.5;
/// m: half the car's width (vehicle.yaml's body: 1.00 m wide).
const HALF_WIDTH: f64 = 0.5;

fn settings() -> Settings {
    Settings {
        curve: StopCurve::new(1.0, 2.0, 0.35).expect("a curve"),
        margin: 0.5,
        max_offset: 3.0,
        alpha_speed: 1.0,
        look_ahead: 1.5,
        alpha_lane: 1.0,
        half_width: HALF_WIDTH,
        lane_margin: 0.2,
        clf_rate: 0.5,
        clf_weight: 1.0,
        wheelbase: 1.561,
        max_steer: 0.5,
        max_speed: 2.0,
        max_reverse: 0.5,
    }
    .check()
    .expect("settings")
}

/// A hostile driver: pieces of constant (v, w), switched every `hold` s.
#[derive(Debug, Clone)]
struct Hostile {
    pieces: Vec<(f64, f64)>,
    hold: f64,
}

impl Hostile {
    fn at(&self, t: f64) -> Command {
        let i = ((t / self.hold) as usize).min(self.pieces.len() - 1);
        let (v, w) = self.pieces[i];
        Command { v, w }
    }
}

fn hostile() -> impl Strategy<Value = Hostile> {
    (
        prop::collection::vec((0.0..3.0f64, -1.5..1.5f64), 1..12),
        0.3..4.0f64,
    )
        .prop_map(|(pieces, hold)| Hostile { pieces, hold })
}

fn start() -> impl Strategy<Value = Car> {
    (-0.4..0.4f64, -0.3..0.3f64, 0.0..1.5f64).prop_map(|(y, yaw, v)| Car {
        x: 1.0,
        y,
        yaw,
        v,
        steer: 0.0,
    })
}

/// (deepest past the end, widest over a lane line) over a run, in m: both ≤ 0 is safe.
fn worst(ticks: &[safety_filter::sim::Tick]) -> (f64, f64) {
    let past = ticks
        .iter()
        .map(|k| -k.room)
        .fold(f64::NEG_INFINITY, f64::max);
    let over = ticks
        .iter()
        .map(|k| k.offset.abs() + HALF_WIDTH - HALF_LANE)
        .fold(f64::NEG_INFINITY, f64::max);
    (past, over)
}

fn drive(lane: &Lane, car: Car, driver: &Hostile, filtered: bool) -> Vec<safety_filter::sim::Tick> {
    let mut s = settings();
    if !filtered {
        // A filter that can't say no: huge room and wheels, no lane.
        s.max_speed = 100.0;
        s.max_steer = 1.5;
        s.margin = 0.0;
        s.max_offset = 1e9;
    }
    let mut f = SafetyFilter::new(s);
    let open = Lane {
        end: f64::INFINITY,
        left: vec![f64::NAN; lane.len()],
        right: vec![f64::NAN; lane.len()],
        ..lane.clone()
    };
    let used = if filtered { lane } else { &open };
    let mut ticks = run(
        &mut f,
        &CarModel::default(),
        used,
        car,
        40.0,
        20.0,
        |t, _| driver.at(t),
    );
    if !filtered {
        // Score the open run against the real lane.
        for k in &mut ticks {
            if let Some(fr) = lane.frame(k.car.x, k.car.y, k.car.yaw) {
                k.offset = fr.offset;
                k.room = fr.room;
            }
        }
    }
    ticks
}

/// 400 cases a property, or `PROPTEST_CASES` (a soak: `PROPTEST_CASES=5000 cargo test`).
fn cases() -> u32 {
    std::env::var("PROPTEST_CASES")
        .ok()
        .and_then(|n| n.parse().ok())
        .unwrap_or(400)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(cases()))]

    #[test]
    fn never_faster_never_reversed(cmds in prop::collection::vec((-3.0..3.0f64, -2.0..2.0f64), 1..50),
                                   room in -1.0..30.0f64, offset in -1.2..1.2f64, speed in 0.0..2.0f64) {
        let mut f = SafetyFilter::new(settings());
        let frame = safety_filter::Frame {
            along: 0.0, offset, twist: 0.0, curvature: 0.0, room, left: HALF_LANE, right: HALF_LANE,
        };
        for (v, w) in cmds {
            let r = f.step(Command { v, w }, &safety_filter::Inputs { frame: Some(frame), speed: Some(speed), dt: 0.05 });
            prop_assert!(r.output.v.abs() <= v.abs() + 1e-12);
            prop_assert!(r.output.v * v >= 0.0, "direction turned: {} from {}", r.output.v, v);
            if v == 0.0 { prop_assert_eq!(r.output.v, 0.0); }
        }
    }

    #[test]
    fn a_straight_road_holds_any_driver(driver in hostile(), car in start()) {
        let lane = straight_lane(30.0, HALF_LANE);
        let (past, over) = worst(&drive(&lane, car, &driver, true));
        prop_assert!(past <= 0.0, "passed the end by {past:.3} m");
        prop_assert!(over <= 0.0, "over a lane line by {over:.3} m");
    }

    #[test]
    fn a_bend_holds_any_driver(driver in hostile(), car in start()) {
        let lane = bending_lane(30.0, HALF_LANE, 15.0);
        let (past, over) = worst(&drive(&lane, car, &driver, true));
        prop_assert!(past <= 0.0, "passed the end by {past:.3} m");
        prop_assert!(over <= 0.0, "over a lane line by {over:.3} m");
    }
}

#[test]
fn the_drivers_are_hostile_without_the_filter() {
    // Full speed, hard left: off the road and past its end, unless something says no.
    let lane = straight_lane(30.0, HALF_LANE);
    let car = Car {
        x: 1.0,
        y: 0.0,
        yaw: 0.0,
        v: 0.0,
        steer: 0.0,
    };
    let wall = Hostile {
        pieces: vec![(2.0, 0.0)],
        hold: 100.0,
    };
    let (past, _) = worst(&drive(&lane, car, &wall, false));
    assert!(past > 1.0, "the open car should run off the end: {past}");
    let swerve = Hostile {
        pieces: vec![(1.5, 0.0), (1.5, 0.8)],
        hold: 2.0,
    };
    let (_, over) = worst(&drive(&lane, car, &swerve, false));
    assert!(over > 0.5, "the open car should leave the lane: {over}");
    // The same drivers through the filter.
    let (past, _) = worst(&drive(&lane, car, &wall, true));
    assert!(past <= 0.0, "{past}");
    let (_, over) = worst(&drive(&lane, car, &swerve, true));
    assert!(over <= 0.0, "{over}");
}

/// The case proptest found on 23 Nov (`guarantees.proptest-regressions`), kept as a plain
/// test. The driver rides the right lane line of a left bend at full speed, then asks for
/// 0.019 m/s with a hard right turn, then for full speed again.
///
/// Before the fix the lane rules looked only at the *commanded* speed: under 0.02 m/s
/// they stood aside, though the car still rolled at 1.5 m/s. The hard right went straight
/// to the wheels, the car turned out while it slowed, and the body went 0.044 m over the
/// line. Now the lane is judged at the speed the car has.
#[test]
fn a_creep_command_while_the_car_still_rolls_cannot_swing_it_over_the_line() {
    let lane = bending_lane(30.0, HALF_LANE, 15.0);
    let car = Car {
        x: 1.0,
        y: 0.0,
        yaw: 0.0,
        v: 0.0,
        steer: 0.0,
    };
    let driver = Hostile {
        pieces: vec![
            (2.7579002506139427, -0.09505525071709515),
            (0.01856879439669645, -1.2389701371307913),
            (2.9041956209582316, 0.0),
        ],
        hold: 3.550266885039675,
    };
    let ticks = drive(&lane, car, &driver, true);
    let (past, over) = worst(&ticks);
    assert!(past <= 0.0, "passed the end by {past:.3} m");
    assert!(over <= 0.0, "over a lane line by {over:.3} m");
    // The lane margin (0.2 m) is kept too, give or take the car's lag.
    assert!(
        over <= -0.15,
        "the body came within {:.3} m of a line",
        -over
    );
    // And the filter did not buy that by parking the car: it drives on to the end.
    let start = ticks[0].room;
    let end = ticks.last().expect("ticks").room;
    assert!(
        start - end > 20.0,
        "the car only covered {:.1} m",
        start - end
    );
}

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

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

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

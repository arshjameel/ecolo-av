//! The stop zone's rules, checked on random points and commands (proptest, TODO 204).
//!
//! The hand-written tests (`src/tests.rs`) pick their cases; these pick thousands: any
//! points, any command, any time. A safety lock must hold for all of them.

use proptest::prelude::*;
use stop_zone::{
    Command, Mount, Point, Polygon, Reading, Settings, Source, SourceKind, SpeedBand, StopZone,
};

/// nav2.yaml's boxes: 1 m ahead of the front bumper (x 1.8 to 2.8), 1 m behind the rear
/// (x -0.2 to -1.2), 1.4 m wide; 3 points stop. One lidar scan at base_link.
fn zone() -> StopZone {
    let square = |x0: f64, x1: f64| {
        Polygon(vec![
            Point::new(x1, 0.7),
            Point::new(x1, -0.7),
            Point::new(x0, -0.7),
            Point::new(x0, 0.7),
        ])
    };
    let settings = Settings {
        bands: vec![
            SpeedBand {
                name: "forward".into(),
                polygon: square(1.8, 2.8),
                speed: 0.0..=10.0,
                turn: -10.0..=10.0,
            },
            SpeedBand {
                name: "backward".into(),
                polygon: square(-1.2, -0.2),
                speed: -10.0..=0.0,
                turn: -10.0..=10.0,
            },
        ],
        min_points: 3,
        source_timeout: 1.0,
        stop_pub_timeout: 2.0,
    };
    let lidar = Source {
        name: "lidar".into(),
        kind: SourceKind::Scan,
        mount: Mount::default(),
        min_range: 0.0,
    };
    StopZone::new(settings, vec![lidar]).expect("good settings")
}

fn in_forward_box(p: &[f64; 3]) -> bool {
    (1.8..=2.8).contains(&p[0]) && (-0.7..=0.7).contains(&p[1])
}

fn point() -> impl Strategy<Value = [f64; 3]> {
    (-6.0..6.0f64, -3.0..3.0f64).prop_map(|(x, y)| [x, y, 0.0])
}

fn forward() -> impl Strategy<Value = Command> {
    (0.01..3.0f64, -1.0..1.0f64).prop_map(|(x, turn)| Command { x, turn })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    /// What comes out is the command itself, a zero, or nothing: never another command.
    #[test]
    fn it_only_ever_passes_or_stops(
        cmd in (-3.0..3.0f64, -1.0..1.0f64).prop_map(|(x, turn)| Command { x, turn }),
        points in prop::collection::vec(point(), 0..20),
    ) {
        let mut z = zone();
        let r = Reading { stamp: 10.0, points };
        let d = z.step(10.0, cmd, &[Some(&r)]);
        prop_assert!(d.out == Some(cmd) || d.out == Some(Command::STOP) || d.out.is_none());
        if d.stop.is_none() {
            prop_assert_eq!(d.out, Some(cmd));
        }
    }

    /// Three or more points in the forward box always stop a forward command.
    #[test]
    fn three_points_in_the_box_always_stop(
        cmd in forward(),
        inside in prop::collection::vec(
            (1.81..2.79f64, -0.69..0.69f64).prop_map(|(x, y)| [x, y, 0.0]), 3..10),
        outside in prop::collection::vec(point(), 0..10),
    ) {
        let mut z = zone();
        let r = Reading { stamp: 10.0, points: [inside, outside].concat() };
        let d = z.step(10.0, cmd, &[Some(&r)]);
        prop_assert_eq!(d.out, Some(Command::STOP));
        prop_assert!(d.stop.is_some());
    }

    /// Points only outside the forward box never stop a forward command.
    #[test]
    fn points_outside_the_box_never_stop(
        cmd in forward(),
        points in prop::collection::vec(point(), 0..30),
    ) {
        let points: Vec<_> = points.into_iter().filter(|p| !in_forward_box(p)).collect();
        let mut z = zone();
        let r = Reading { stamp: 10.0, points };
        let d = z.step(10.0, cmd, &[Some(&r)]);
        prop_assert_eq!(d.out, Some(cmd));
    }

    /// More points never turn a stop into a go.
    #[test]
    fn more_points_never_clear_a_stop(
        cmd in forward(),
        points in prop::collection::vec(point(), 0..20),
        more in prop::collection::vec(point(), 0..20),
    ) {
        let few = zone().step(10.0, cmd, &[Some(&Reading { stamp: 10.0, points: points.clone() })]);
        let all = [points, more].concat();
        let many = zone().step(10.0, cmd, &[Some(&Reading { stamp: 10.0, points: all })]);
        prop_assert!(few.stop.is_none() || many.stop.is_some());
    }

    /// A sensor older than source_timeout stops the car, whatever it saw.
    #[test]
    fn a_silent_sensor_always_stops(
        cmd in forward(),
        age in 1.001..60.0f64,
        points in prop::collection::vec(point(), 0..20),
    ) {
        let mut z = zone();
        let r = Reading { stamp: 10.0, points };
        let d = z.step(10.0 + age, cmd, &[Some(&r)]);
        prop_assert_eq!(d.out, Some(Command::STOP));
        let none = zone().step(10.0, cmd, &[None]);
        prop_assert_eq!(none.out, Some(Command::STOP));
    }

    /// Points that aren't numbers are dropped, never counted, never a crash.
    #[test]
    fn nan_and_infinite_points_are_dropped(
        cmd in forward(),
        bad in prop::collection::vec(
            prop_oneof![Just(f64::NAN), Just(f64::INFINITY), Just(f64::NEG_INFINITY)], 3..10),
    ) {
        let points: Vec<_> = bad.iter().map(|&v| [v, 0.0, 0.0]).collect();
        let d = zone().step(10.0, cmd, &[Some(&Reading { stamp: 10.0, points })]);
        prop_assert_eq!(d.out, Some(cmd));
    }

    /// The ray-crossing test agrees with plain bounds on a rectangle, off its edges.
    #[test]
    fn contains_matches_a_rectangle(x in -5.0..5.0f64, y in -5.0..5.0f64) {
        let rect = Polygon(vec![
            Point::new(2.8, 0.7), Point::new(2.8, -0.7),
            Point::new(1.8, -0.7), Point::new(1.8, 0.7),
        ]);
        let near_edge = (x - 1.8).abs() < 1e-9 || (x - 2.8).abs() < 1e-9
            || (y - 0.7).abs() < 1e-9 || (y + 0.7).abs() < 1e-9;
        prop_assume!(!near_edge);
        let inside = x > 1.8 && x < 2.8 && y > -0.7 && y < 0.7;
        prop_assert_eq!(rect.contains(Point::new(x, y)), inside);
    }
}

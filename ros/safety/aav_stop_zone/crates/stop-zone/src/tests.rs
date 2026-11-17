//! The stop zone with nav2.yaml's own numbers: a 1 m box ahead of the front bumper
//! (x 1.8 to 2.8), one behind the rear (x -0.2 to -1.2), each 1.4 m wide; 3 points stop.

use super::*;

fn square(x0: f64, x1: f64) -> Polygon {
    Polygon(vec![
        Point::new(x1, 0.7),
        Point::new(x1, -0.7),
        Point::new(x0, -0.7),
        Point::new(x0, 0.7),
    ])
}

fn zone() -> StopZone {
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
    let radar = Source {
        name: "radar".into(),
        kind: SourceKind::Cloud {
            min_height: -0.5,
            max_height: 2.0,
        },
        mount: Mount {
            x: 1.8,
            y: 0.0,
            z: 0.5,
            yaw: 0.0,
        },
        min_range: 0.2,
    };
    let lidar = Source {
        name: "lidar".into(),
        kind: SourceKind::Scan,
        mount: Mount::default(),
        min_range: 0.0,
    };
    StopZone::new(settings, vec![radar, lidar]).unwrap()
}

/// `n` radar points `ahead` m in front of the radar (so 1.8 + ahead from base_link).
fn radar(stamp: f64, ahead: f64, n: usize) -> Reading {
    Reading {
        stamp,
        points: (0..n).map(|k| [ahead, 0.1 * k as f64 - 0.2, 0.0]).collect(),
    }
}

fn empty(stamp: f64) -> Reading {
    Reading {
        stamp,
        points: Vec::new(),
    }
}

const GO: Command = Command { x: 1.0, turn: 0.0 };
const BACK: Command = Command { x: -0.5, turn: 0.0 };

#[test]
fn a_clear_box_lets_the_command_through() {
    let mut z = zone();
    let d = z.step(10.0, GO, &[Some(&radar(10.0, 3.0, 5)), Some(&empty(10.0))]);
    assert_eq!(d.out, Some(GO)); // 4.8 m from base_link: past the box
    assert_eq!(d.stop, None);
    assert_eq!(d.band.as_deref(), Some("forward"));
}

#[test]
fn three_points_in_the_box_stop_and_two_dont() {
    let mut z = zone();
    let two = z.step(10.0, GO, &[Some(&radar(10.0, 0.5, 2)), Some(&empty(10.0))]);
    assert_eq!(two.out, Some(GO));
    let three = z.step(10.1, GO, &[Some(&radar(10.1, 0.5, 3)), Some(&empty(10.1))]);
    assert_eq!(three.out, Some(Command::STOP));
    assert_eq!(
        three.stop,
        Some(Why::Points {
            band: "forward".into(),
            count: 3
        })
    );
}

#[test]
fn points_from_every_sensor_count_together() {
    let mut z = zone();
    let scan = Reading {
        stamp: 10.0,
        points: vec![[2.0, 0.0, 0.0]],
    };
    let d = z.step(10.0, GO, &[Some(&radar(10.0, 0.5, 2)), Some(&scan)]);
    assert_eq!(d.out, Some(Command::STOP));
}

#[test]
fn reversing_checks_the_box_behind() {
    let mut z = zone();
    let behind = Reading {
        stamp: 10.0,
        points: vec![[-0.5, 0.0, 0.0]; 3],
    };
    let d = z.step(10.0, BACK, &[Some(&radar(10.0, 0.5, 5)), Some(&behind)]);
    assert_eq!(d.band.as_deref(), Some("backward"));
    assert_eq!(
        d.stop,
        Some(Why::Points {
            band: "backward".into(),
            count: 3
        })
    );
    // Going forward, the same points behind don't count, but the radar's ahead do.
    let mut z = zone();
    let d = z.step(10.0, GO, &[Some(&empty(10.0)), Some(&behind)]);
    assert_eq!(d.out, Some(GO));
}

#[test]
fn standing_still_counts_as_forward() {
    let mut z = zone();
    let d = z.step(
        10.0,
        Command::STOP,
        &[Some(&empty(10.0)), Some(&empty(10.0))],
    );
    assert_eq!(d.band.as_deref(), Some("forward"));
}

#[test]
fn a_silent_sensor_stops_the_car() {
    let mut z = zone();
    let none = z.step(10.0, GO, &[Some(&empty(10.0)), None]);
    assert_eq!(
        none.stop,
        Some(Why::Silent {
            source: "lidar".into()
        })
    );
    assert_eq!(none.out, Some(Command::STOP));
    // A reading 1.5 s old is past the 1 s timeout; 0.9 s old is fine.
    let mut z = zone();
    let old = z.step(11.5, GO, &[Some(&empty(10.0)), Some(&empty(11.5))]);
    assert_eq!(
        old.stop,
        Some(Why::Silent {
            source: "radar".into()
        })
    );
    let ok = z.step(11.5, GO, &[Some(&empty(10.6)), Some(&empty(11.5))]);
    assert_eq!(ok.out, Some(GO));
}

#[test]
fn the_radars_own_clutter_and_points_overhead_dont_count() {
    let mut z = zone();
    let close = Reading {
        stamp: 10.0,
        points: vec![[0.1, 0.0, 0.0]; 5], // 0.1 m from the radar: its own clutter
    };
    assert_eq!(
        z.step(10.0, GO, &[Some(&close), Some(&empty(10.0))]).out,
        Some(GO)
    );
    let high = Reading {
        stamp: 10.0,
        points: vec![[0.5, 0.0, 1.8]; 5], // 2.3 m up in base_link: over the car
    };
    assert_eq!(
        z.step(10.0, GO, &[Some(&high), Some(&empty(10.0))]).out,
        Some(GO)
    );
}

#[test]
fn zeros_go_out_for_stop_pub_timeout_then_nothing() {
    let mut z = zone();
    let blocked = |t: f64| radar(t, 0.5, 3);
    let lidar = |t: f64| empty(t);
    assert_eq!(
        z.step(10.0, GO, &[Some(&blocked(10.0)), Some(&lidar(10.0))])
            .out,
        Some(Command::STOP)
    );
    assert_eq!(
        z.step(11.9, GO, &[Some(&blocked(11.9)), Some(&lidar(11.9))])
            .out,
        Some(Command::STOP)
    );
    assert_eq!(
        z.step(12.1, GO, &[Some(&blocked(12.1)), Some(&lidar(12.1))])
            .out,
        None
    );
    // Clear again: the command goes through at once.
    assert_eq!(
        z.step(12.2, GO, &[Some(&empty(12.2)), Some(&lidar(12.2))])
            .out,
        Some(GO)
    );
}

#[test]
fn the_point_test_is_the_ray_crossing_one() {
    let box_ = square(1.8, 2.8);
    assert!(box_.contains(Point::new(2.0, 0.0)));
    assert!(!box_.contains(Point::new(3.0, 0.0)));
    assert!(!box_.contains(Point::new(2.0, 0.8)));
    let l = Polygon(vec![
        Point::new(0.0, 0.0),
        Point::new(2.0, 0.0),
        Point::new(2.0, 1.0),
        Point::new(1.0, 1.0),
        Point::new(1.0, 2.0),
        Point::new(0.0, 2.0),
    ]);
    assert!(l.contains(Point::new(0.5, 1.5)));
    assert!(!l.contains(Point::new(1.5, 1.5))); // the L's missing corner
}

#[test]
fn a_box_needs_three_corners() {
    let mut s = zone().settings.clone();
    s.bands[0].polygon.0.truncate(2);
    assert_eq!(
        StopZone::new(s, Vec::new()).unwrap_err(),
        SettingsError::BadBox("forward".into())
    );
}

#[test]
fn the_state_names_the_box_or_the_silent_sensor_as_collision_monitor() {
    let points = Why::Points {
        band: "forward".into(),
        count: 3,
    };
    let silent = Why::Silent {
        source: "radar".into(),
    };
    assert_eq!(points.polygon_name("StopZone"), "StopZone");
    assert_eq!(silent.polygon_name("StopZone"), INVALID_SOURCE);
}

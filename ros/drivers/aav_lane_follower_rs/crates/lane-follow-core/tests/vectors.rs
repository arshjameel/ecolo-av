//! lane_follow.py's answers (gen_vectors.py → vectors.json), and ours with the PID: the
//! same commands, status lines, states, transitions and watchdog trips at every event.
//! The sums run in the same order, so the commands agree to the last bit or two.

use lane_follow_core::{Follower, Geometry, Law, Line, Pid, PidGains, Settings, State};
use serde_json::Value;

const TOLERANCE: f64 = 1e-12;

fn f(v: &Value) -> f64 {
    v.as_f64().expect("a number")
}

fn state(name: &str) -> State {
    match name {
        "LANE_KEEP" => State::LaneKeep,
        "LANE_CHANGING" => State::LaneChanging,
        "LANE_COMMITTED" => State::LaneCommitted,
        "NO_LANE_STOPPING" => State::NoLaneStopping,
        "RESUMING" => State::Resuming,
        other => panic!("unknown state {other}"),
    }
}

fn geometry(g: &Value) -> Geometry {
    let flat = Line {
        a: 0.0,
        b: 0.0,
        c: 0.0,
    };
    Geometry {
        offset: f(&g["vehicle_offset_m"]),
        mean_radius: f(&g["mean_curvature_m"]),
        lane_width: 3.5,
        left_detected: g["left_lane_detected"].as_bool().expect("bool"),
        right_detected: g["right_lane_detected"].as_bool().expect("bool"),
        both_detected: g["both_lanes_detected"].as_bool().expect("bool"),
        left: flat,
        right: flat,
    }
}

#[test]
fn every_event_the_same() {
    let v: Value = serde_json::from_str(include_str!("vectors.json")).expect("vectors.json");
    let (mut events, mut commands) = (0, 0);
    for run in v["runs"].as_array().expect("runs") {
        let s = &run["settings"];
        let settings = Settings {
            base_speed: f(&s["base_speed_mps"]),
            max_angular_z: f(&s["max_angular_z"]),
            min_speed: f(&s["min_speed_mps"]),
            min_curvature: f(&s["min_curvature_m"]),
            lane_change_rate_threshold: f(&s["lane_change_rate_threshold"]),
            lane_change_offset_threshold: f(&s["lane_change_offset_threshold"]),
            lane_change_speed: f(&s["lane_change_speed"]),
            lane_change_timeout: f(&s["lane_change_timeout_s"]),
            stop_wait: f(&s["stop_wait_s"]),
            decel_rate: f(&s["decel_rate"]),
            accel_rate: f(&s["accel_rate"]),
        };
        let gains = PidGains {
            kp: f(&s["pid_kp"]),
            ki: f(&s["pid_ki"]),
            kd: f(&s["pid_kd"]),
        };
        let invert = s["invert_offset_sign"].as_bool().expect("bool");
        // Started on the two clocks as the Python's was.
        let (start_wall, start_now) = (f(&run["start"]["wall"]), f(&run["start"]["now"]));
        let mut fol = Follower::new(
            settings,
            Law::Pid {
                pid: Pid::new(gains, start_wall),
                invert_offset_sign: invert,
            },
            start_wall,
            start_now,
        );
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let (wall, now) = (f(&e["wall"]), f(&e["now"]));
            let got = if e["op"] == "geo" {
                fol.on_geometry(&geometry(&e["g"]), wall, now)
            } else {
                fol.tick(wall, now)
            };
            let want = &e["reply"];
            let cmds = want["commands"].as_array().expect("commands");
            assert_eq!(got.commands.len(), cmds.len(), "event {i}: {got:?}");
            for (c, w) in got.commands.iter().zip(cmds) {
                assert!(
                    (c.linear_x - f(&w[0])).abs() <= TOLERANCE,
                    "event {i}: {c:?} {w}"
                );
                assert!(
                    (c.angular_z - f(&w[1])).abs() <= TOLERANCE,
                    "event {i}: {c:?} {w}"
                );
                commands += 1;
            }
            assert_eq!(got.status.as_deref(), want["status"].as_str(), "event {i}");
            assert_eq!(got.state, want["state"].as_str().map(state), "event {i}");
            let transitions: Vec<(State, State)> = want["transitions"]
                .as_array()
                .expect("transitions")
                .iter()
                .map(|t| {
                    (
                        state(t[0].as_str().expect("s")),
                        state(t[1].as_str().expect("s")),
                    )
                })
                .collect();
            assert_eq!(got.transitions, transitions, "event {i}");
            match (got.quiet_for, want["quiet_for"].as_f64()) {
                (None, None) => {}
                (Some(a), Some(b)) => assert!((a - b).abs() <= TOLERANCE, "event {i}"),
                other => panic!("event {i}: quiet_for {other:?}"),
            }
            events += 1;
        }
    }
    assert!(events > 1500 && commands > 1000, "only {events} events");
    println!("{events} events, {commands} commands, all the same");
}

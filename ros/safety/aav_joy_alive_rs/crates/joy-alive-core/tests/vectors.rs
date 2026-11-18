//! joy_alive.py's answers (gen_vectors.py → vectors.json), and ours: the same answer, trip,
//! sizes and line for people at every event. No sums here, so no tolerance is needed.

use joy_alive_core::{JoyAlive, Settings, Trip};
use serde_json::Value;

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "nan" => f64::NAN,
        other => other.as_f64().expect("a number"),
    }
}

fn index(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("an index")).expect("usize")
}

fn trip(v: &Value) -> Option<Trip> {
    match v.as_str() {
        None => None,
        Some("WAITING") => Some(Trip::Waiting),
        Some("SILENT") => Some(Trip::Silent),
        Some("FROZEN") => Some(Trip::Frozen),
        Some(other) => panic!("unknown trip {other}"),
    }
}

#[test]
fn every_event_the_same() {
    let text = include_str!("vectors.json");
    let v: Value = serde_json::from_str(text).expect("vectors.json");
    let mut events = 0;
    for run in v["runs"].as_array().expect("runs") {
        let s = &run["settings"];
        let mut check = JoyAlive::new(Settings {
            enable_button: index(&s["enable_button"]),
            motion_axes: s["motion_axes"]
                .as_array()
                .expect("axes")
                .iter()
                .map(index)
                .collect(),
            motion_threshold: f(&s["motion_threshold"]),
            silent_after: f(&s["silent_after"]),
            frozen_after: f(&s["frozen_after"]),
        })
        .expect("settings");
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let t = f(&e["t"]);
            if e["op"] == "joy" {
                // f32, as sensor_msgs/Joy: gen_vectors.py rounded them so.
                let axes: Vec<f32> = e["axes"]
                    .as_array()
                    .expect("axes")
                    .iter()
                    .map(|a| f(a) as f32)
                    .collect();
                let buttons: Vec<i32> = e["buttons"]
                    .as_array()
                    .expect("buttons")
                    .iter()
                    .map(|b| i32::try_from(b.as_i64().expect("button")).expect("i32"))
                    .collect();
                let ok = check.on_joy(t, &axes, &buttons);
                assert_eq!(Some(ok), e["ok"].as_bool(), "event {i}");
                let sizes = e["sizes"].as_array().expect("sizes");
                assert_eq!(
                    check.sizes(),
                    (index(&sizes[0]), index(&sizes[1])),
                    "event {i}"
                );
            } else {
                let now = check.tick(t);
                assert_eq!(Some(now), e["tripped_now"].as_bool(), "event {i}");
                assert_eq!(
                    check.state(t),
                    e["state"].as_str().expect("state"),
                    "event {i}"
                );
            }
            assert_eq!(check.tripped(), trip(&e["tripped"]), "event {i}");
            events += 1;
        }
    }
    assert!(events > 1000, "only {events} events");
    println!("{events} events, all the same");
}

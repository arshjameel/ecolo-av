//! rc_drive.py's answers (gen_vectors.py → vectors.json), and ours: the same, exactly.

use rc_core::{Limits, RcDrive, Sticks, Switch};
use serde_json::Value;

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "nan" => f64::NAN,
        other => other.as_f64().expect("a number"),
    }
}

#[test]
fn every_tick_the_same_command() {
    let text = include_str!("vectors.json");
    let v: Value = serde_json::from_str(text).expect("vectors.json");
    let mut ticks = 0;
    for run in v["runs"].as_array().expect("runs") {
        let l = &run["limits"];
        let mut drive = RcDrive::new(Limits {
            max_speed: f(&l["max_speed"]),
            max_reverse_speed: f(&l["max_reverse_speed"]),
            max_steering_angle: f(&l["max_steering_angle"]),
            wheelbase: f(&l["wheelbase"]),
            stale: f(&l["stale"]),
        });
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let t = f(&e["t"]);
            if e["op"] == "rc" {
                let switch = u8::try_from(e["switch"].as_u64().expect("u8")).expect("u8");
                drive.heard(
                    Sticks {
                        switch: Switch::from_byte(switch),
                        throttle: f(&e["throttle"]),
                        steering: f(&e["steering"]),
                        brake: e["brake"] == true,
                    },
                    t,
                );
                continue;
            }
            let ours = drive.command(t);
            match (ours, &e["cmd"]) {
                (None, Value::Null) => {}
                (Some(c), Value::Array(theirs)) => {
                    assert_eq!(c.v, f(&theirs[0]), "event {i}");
                    assert_eq!(c.w, f(&theirs[1]), "event {i}");
                }
                (ours, theirs) => panic!("event {i}: ours {ours:?}, Python's {theirs}"),
            }
            assert_eq!(drive.driving(), e["driving"] == true, "event {i}");
            ticks += 1;
        }
    }
    assert!(ticks >= 3000);
}

//! rc_drive.py's answers (gen_vectors.py → vectors.json), and ours: the same, to 1e-12.
//! Not bit for bit: the turn rate goes through tan(), and Linux's maths library gave a
//! last digit one off macOS's (CI, 18 Nov).

use rc_core::{Limits, RcDrive, Sticks, Switch};
use serde_json::Value;

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "nan" => f64::NAN,
        other => other.as_f64().expect("a number"),
    }
}

/// Equal to 1e-12, or both NaN.
fn same(ours: f64, theirs: f64, event: usize) {
    assert!(
        (ours.is_nan() && theirs.is_nan()) || (ours - theirs).abs() <= 1e-12,
        "event {event}: ours {ours}, Python's {theirs}"
    );
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
                    same(c.v, f(&theirs[0]), i);
                    same(c.w, f(&theirs[1]), i);
                }
                (ours, theirs) => panic!("event {i}: ours {ours:?}, Python's {theirs}"),
            }
            assert_eq!(drive.driving(), e["driving"] == true, "event {i}");
            ticks += 1;
        }
    }
    assert!(ticks >= 3000);
}

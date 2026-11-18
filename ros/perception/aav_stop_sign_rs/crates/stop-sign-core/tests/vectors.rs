//! stop_sign.py's answers (gen_vectors.py → vectors.json), and ours: states and booleans
//! the same, numbers within 1e-12 (not bit for bit: Linux's maths library may differ in a
//! last digit, as rc-core's did on CI).

use serde_json::Value;
use stop_sign_core::{
    biggest_sign, is_optical, lidar_sign, normal_label, sign_distance_m, sign_point, Sighting,
    State, StopRule, StopSignWatcher,
};

const CLOSE: f64 = 1e-12;

fn vectors() -> Value {
    serde_json::from_str(include_str!("vectors.json")).expect("vectors.json")
}

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "nan" => f64::NAN,
        Value::String(s) if s == "inf" => f64::INFINITY,
        Value::String(s) if s == "-inf" => f64::NEG_INFINITY,
        other => other
            .as_f64()
            .unwrap_or_else(|| panic!("not a number: {other}")),
    }
}

fn of(v: &Value) -> Option<f64> {
    (!v.is_null()).then(|| f(v))
}

fn same(ours: f64, theirs: f64, what: &str) {
    assert!(
        ours == theirs || (ours - theirs).abs() <= CLOSE || (ours.is_nan() && theirs.is_nan()),
        "{what}: ours {ours}, Python's {theirs}"
    );
}

fn rule(v: &Value) -> StopRule {
    StopRule {
        stop_distance_m: f(&v["stop_distance_m"]),
        focal_px: f(&v["focal_px"]),
        sign_height_m: f(&v["sign_height_m"]),
        min_score: f(&v["min_score"]),
        hold_s: f(&v["hold_s"]),
        ignore_s: f(&v["ignore_s"]),
    }
}

fn state(v: &Value) -> State {
    match v.as_str() {
        Some("clear") => State::Clear,
        Some("stopping") => State::Stopping,
        Some("passing") => State::Passing,
        other => panic!("unknown state {other:?}"),
    }
}

fn sightings(v: &Value) -> Vec<Sighting<'_>> {
    v.as_array()
        .expect("boxes")
        .iter()
        .map(|b| Sighting {
            label: b[0].as_str().expect("label"),
            score: f(&b[1]),
            height: f(&b[2]),
            centre: f(&b[3]),
        })
        .collect()
}

#[test]
fn the_watcher_steps_as_the_python() {
    let v = vectors();
    let mut steps = 0;
    for run in v["runs"].as_array().expect("runs") {
        let mut w = StopSignWatcher::new(rule(&run["rule"]));
        for (i, s) in run["steps"].as_array().expect("steps").iter().enumerate() {
            let t = f(&s["t"]);
            let stop = if s["op"] == "tick" {
                w.tick(t)
            } else {
                w.update(&sightings(&s["boxes"]), t, of(&s["focal"]))
            };
            assert_eq!(Some(stop), s["stop"].as_bool(), "step {i}");
            assert_eq!(w.state(), state(&s["state"]), "step {i}");
            steps += 1;
        }
    }
    assert!(steps > 2000);
}

#[test]
fn points_signs_and_lidar_as_the_python() {
    let v = vectors();
    for (i, c) in v["points"].as_array().expect("points").iter().enumerate() {
        let a = &c["args"];
        let lens = match (of(&a[3]), of(&a[4])) {
            (Some(fx), Some(cx)) => Some((fx, cx)),
            _ => None,
        };
        let ours = sign_point(
            f(&a[0]),
            f(&a[1]),
            f(&a[2]),
            lens,
            a[5].as_bool().expect("optical"),
            f(&a[6]),
        );
        for (k, theirs) in c["point"].as_array().expect("point").iter().enumerate() {
            same(ours[k], f(theirs), &format!("point {i}[{k}]"));
        }
    }
    for (i, c) in v["biggest"].as_array().expect("biggest").iter().enumerate() {
        let ours = biggest_sign(&sightings(&c["boxes"]), f(&c["min_score"]));
        match (ours, &c["best"]) {
            (None, Value::Null) => {}
            (Some((centre, height)), Value::Array(b)) => {
                same(centre, f(&b[0]), &format!("biggest {i}"));
                same(height, f(&b[1]), &format!("biggest {i}"));
            }
            (ours, theirs) => panic!("biggest {i}: ours {ours:?}, Python's {theirs}"),
        }
    }
    for (i, c) in v["lidar"].as_array().expect("lidar").iter().enumerate() {
        let things: Vec<(String, f64, f64)> = c["things"]
            .as_array()
            .expect("things")
            .iter()
            .map(|t| (t[0].as_str().expect("kind").to_string(), f(&t[1]), f(&t[2])))
            .collect();
        let g = &c["guess"];
        let ours = lidar_sign((f(&g[0]), f(&g[1])), &things, f(&c["gate"]));
        match (ours, &c["found"]) {
            (None, Value::Null) => {}
            (Some((x, y)), Value::Array(b)) => {
                same(x, f(&b[0]), &format!("lidar {i}"));
                same(y, f(&b[1]), &format!("lidar {i}"));
            }
            (ours, theirs) => panic!("lidar {i}: ours {ours:?}, Python's {theirs}"),
        }
    }
}

#[test]
fn words_distances_and_heights_as_the_python() {
    let v = vectors();
    let w = &v["words"];
    for c in w["labels"].as_array().expect("labels") {
        assert_eq!(
            normal_label(c[0].as_str().expect("label")),
            c[1].as_str().expect("normal")
        );
    }
    for c in w["optical"].as_array().expect("optical") {
        assert_eq!(
            Some(is_optical(c[0].as_str().expect("frame"))),
            c[1].as_bool()
        );
    }
    for c in w["distances"].as_array().expect("distances") {
        same(
            sign_distance_m(f(&c[0]), f(&c[1]), f(&c[2])),
            f(&c[3]),
            &c.to_string(),
        );
    }
    for c in w["min_heights"].as_array().expect("min_heights") {
        same(
            rule(&c[0]).min_height_px(of(&c[1])),
            f(&c[2]),
            &c.to_string(),
        );
    }
}

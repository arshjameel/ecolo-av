//! The same answers as the Python, exactly: `vectors.json`, written by `gen_vectors.py`
//! from `health.py` (and tf2's own Buffer for the TF trees). Text compared as text,
//! numbers bit for bit.

use serde_json::Value;

use health_core::format_g;
use health_core::health::{summary, Ekf, Health, Settings};
use health_core::tree::Tree;

const WHEELBASE: f64 = 1.561;
const TRACK: f64 = 0.75;

fn vectors() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors.json");
    let text = std::fs::read_to_string(path).expect("vectors.json: run gen_vectors.py");
    serde_json::from_str(&text).expect("vectors.json is JSON")
}

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) => match s.as_str() {
            "nan" => f64::NAN,
            "inf" => f64::INFINITY,
            "-inf" => f64::NEG_INFINITY,
            other => panic!("not a number: {other}"),
        },
        _ => v.as_f64().expect("a number"),
    }
}

fn text(v: &Value) -> &str {
    v.as_str().expect("text")
}

/// The same number: bit for bit on the computer that wrote the vectors, within 1e-9 on any
/// other (any NaN matches a NaN): Linux's maths library rounds atan2, sin, cos and acos
/// apart from macOS's in the last bit (20 Nov, CI).
fn same(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits()
        || (a.is_nan() && b.is_nan())
        || (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

#[test]
fn every_report_is_the_pythons() {
    let v = vectors();
    let runs = v["runs"].as_array().expect("runs");
    assert!(runs.len() >= 4);
    let mut compared = 0;
    for (r, run) in runs.iter().enumerate() {
        let fields: Vec<(&str, f64)> = run["settings"]
            .as_object()
            .expect("settings")
            .iter()
            .map(|(k, x)| (k.as_str(), f(x)))
            .collect();
        let settings = Settings::from_fields(&fields).expect("known settings");
        let mut h = Health::new(WHEELBASE, TRACK, settings);
        let mut want = run["reports"].as_array().expect("reports").iter();
        for e in run["events"].as_array().expect("events") {
            let e = e.as_array().expect("an event");
            match text(&e[0]) {
                "imu" => {
                    let a = e[5].as_array().expect("accel");
                    h.imu(
                        f(&e[1]),
                        f(&e[2]),
                        text(&e[3]),
                        f(&e[4]),
                        [f(&a[0]), f(&a[1]), f(&a[2])],
                    );
                }
                "odom" => h.odom(f(&e[1]), f(&e[2]), text(&e[3]), text(&e[4]), f(&e[5])),
                "wheels" => h.wheels(f(&e[1]), f(&e[2]), text(&e[3]), f(&e[4]), f(&e[5])),
                "gps" => {
                    let c: Vec<f64> = e[6].as_array().expect("cov").iter().map(f).collect();
                    let cov: [f64; 9] = c.try_into().expect("9 numbers");
                    let status = i8::try_from(e[4].as_i64().expect("status")).expect("i8");
                    let ctype = u8::try_from(e[5].as_u64().expect("type")).expect("u8");
                    h.gps_fix(f(&e[1]), f(&e[2]), text(&e[3]), status, ctype, &cov);
                }
                "ekf" => {
                    let which = if text(&e[1]) == "local" {
                        Ekf::Local
                    } else {
                        Ekf::Global
                    };
                    h.ekf(
                        which,
                        f(&e[2]),
                        f(&e[3]),
                        text(&e[4]),
                        text(&e[5]),
                        (f(&e[6]), f(&e[7])),
                        f(&e[8]),
                        f(&e[9]),
                    );
                }
                "tf" => {
                    let missing = e[2]
                        .as_array()
                        .expect("missing")
                        .iter()
                        .map(|m| text(m).to_string())
                        .collect();
                    let age = if e[3].is_null() { None } else { Some(f(&e[3])) };
                    h.tf(missing, age);
                }
                "report" => {
                    let now = f(&e[1]);
                    let got = h.report(now);
                    let w = want.next().expect("a report to compare");
                    let at = format!("run {r}, {now} s");
                    let ws = w["statuses"].as_array().expect("statuses");
                    assert_eq!(got.len(), ws.len(), "{at}");
                    for (g, s) in got.iter().zip(ws) {
                        let name = text(&s[0]);
                        assert_eq!(g.check.as_str(), name, "{at}");
                        assert_eq!(g.level.as_str(), text(&s[1]), "{at}: {name}");
                        assert_eq!(g.message, text(&s[2]), "{at}: {name}");
                        let values = s[3].as_array().expect("values");
                        assert_eq!(g.values.len(), values.len(), "{at}: {name}");
                        let shown = s[4].as_array().expect("shown");
                        for (((k, x), kv), show) in g.values.iter().zip(values).zip(shown) {
                            assert_eq!(k, text(&kv[0]), "{at}: {name}");
                            let want_x = f(&kv[1]);
                            // tilt_deg goes through libm's acos, which may differ by a bit
                            // between the Mac that wrote the file and Linux: 4 ulps for it.
                            let close = k == "tilt_deg"
                                && (x.to_bits() as i64 - want_x.to_bits() as i64).abs() <= 4;
                            assert!(
                                same(*x, want_x) || close,
                                "{at}: {name} {k}: {x} != {want_x}"
                            );
                            assert_eq!(format_g(*x, 4), text(show), "{at}: {name} {k}");
                        }
                    }
                    assert_eq!(summary(&got), text(&w["summary"]), "{at}");
                    compared += 1;
                }
                other => panic!("unknown event {other}"),
            }
        }
        assert!(want.next().is_none(), "run {r}: reports left over");
    }
    assert!(compared > 3000, "{compared}");
}

#[test]
fn every_tree_answers_as_tf2() {
    let v = vectors();
    let trees = v["trees"].as_array().expect("trees");
    for (i, t) in trees.iter().enumerate() {
        let mut tree = Tree::new();
        for s in t["sets"].as_array().expect("sets") {
            let s = s.as_array().expect("a set");
            let stamp = s[2].as_f64().expect("sec") + s[3].as_f64().expect("nanosec") * 1e-9;
            tree.set(
                text(&s[0]),
                text(&s[1]),
                stamp,
                s[4].as_bool().expect("fixed"),
            );
        }
        for a in t["asks"].as_array().expect("asks") {
            let a = a.as_array().expect("an ask");
            let got = tree.newest_common(text(&a[0]), text(&a[1]));
            let want = if a[2].is_null() { None } else { Some(f(&a[2])) };
            match (got, want) {
                (Some(g), Some(w)) => assert!(same(g, w), "tree {i}: {a:?}: {g} != {w}"),
                (g, w) => assert_eq!(g.is_some(), w.is_some(), "tree {i}: {a:?}: {g:?} {w:?}"),
            }
        }
    }
}

//! course.py's answers (gen_vectors.py → vectors.json), and ours: the same heading and
//! variance to the bit, or None alike.

use gps_heading_core::{degrees, heading, quaternion_z, wrap, CourseSettings};
use serde_json::Value;

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "nan" => f64::NAN,
        Value::String(s) if s == "inf" => f64::INFINITY,
        Value::String(s) if s == "-inf" => f64::NEG_INFINITY,
        other => other.as_f64().expect("a number"),
    }
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
fn every_case_the_same() {
    let v: Value = serde_json::from_str(include_str!("vectors.json")).expect("vectors.json");
    assert!(same(
        CourseSettings::default().floor,
        f(&v["default_floor"])
    ));
    assert!(same(
        degrees(CourseSettings::default().floor),
        f(&v["floor_deg_default"])
    ));
    let mut answered = 0;
    let cases = v["heading"].as_array().expect("cases");
    for (i, c) in cases.iter().enumerate() {
        let a = &c["args"];
        let s = &c["settings"];
        let settings = CourseSettings {
            min_speed: f(&s[0]),
            max_speed_gap: f(&s[1]),
            floor: f(&s[2]),
        };
        let got = heading(
            f(&a[0]),
            f(&a[1]),
            f(&a[2]),
            f(&a[3]),
            f(&a[4]),
            f(&a[5]),
            (f(&a[6][0]), f(&a[6][1])),
            &settings,
        );
        match (got, c["out"].as_array()) {
            (None, None) => {}
            (Some((yaw, var)), Some(want)) => {
                assert!(
                    same(yaw, f(&want[0])) && same(var, f(&want[1])),
                    "case {i}: ({yaw}, {var}) vs {want:?}"
                );
                answered += 1;
            }
            (got, _) => panic!("case {i}: {got:?} vs {}", c["out"]),
        }
    }
    assert!(
        answered > 1000 && answered < cases.len(),
        "{answered} answered"
    );
    for w in v["wrap"].as_array().expect("wrap") {
        assert!(same(wrap(f(&w[0])), f(&w[1])), "wrap {}", w[0]);
    }
    for q in v["quaternion"].as_array().expect("quaternion") {
        let (x, y, z, w) = quaternion_z(f(&q[0]));
        let want = &q[1];
        assert!(
            same(x, f(&want[0]))
                && same(y, f(&want[1]))
                && same(z, f(&want[2]))
                && same(w, f(&want[3])),
            "quaternion {}",
            q[0]
        );
    }
}

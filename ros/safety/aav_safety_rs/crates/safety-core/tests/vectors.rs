//! geofence.py's, hold.py's and leash.py's answers (gen_vectors.py → vectors.json), and
//! ours: the same lock, state and line at every event, the same refusals, and the same
//! numbers to the bit.

use safety_core::geofence::{
    along_arc, site_fence, stopping_distance, Circle, Fence, Geofence, Mode, Motion, Polygon, Pose,
    Settings,
};
use safety_core::hold::StopHold;
use safety_core::leash::Leash;
use serde_json::Value;

fn vectors() -> Value {
    serde_json::from_str(include_str!("vectors.json")).expect("vectors.json")
}

fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "nan" => f64::NAN,
        Value::String(s) if s == "inf" => f64::INFINITY,
        Value::String(s) if s == "-inf" => f64::NEG_INFINITY,
        other => other.as_f64().expect("a number"),
    }
}

fn fs(v: &Value) -> Vec<f64> {
    v.as_array().expect("a list").iter().map(f).collect()
}

/// The same number: bit for bit on the computer that wrote the vectors, within 1e-9 on any
/// other (any NaN matches a NaN). Linux's maths library rounds sin, cos and atan2 apart
/// from macOS's in the last bit (20 Nov: CI failed on Linux and ARM only).
fn same(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits()
        || (a.is_nan() && b.is_nan())
        || (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn fence(spec: &Value) -> Fence {
    if let Some(c) = spec.get("circle") {
        let c = fs(c);
        Fence::Circle(Circle::new(c[0], c[1], c[2]).expect("circle"))
    } else {
        Fence::Polygon(Polygon::from_flat(&fs(&spec["polygon"])).expect("polygon"))
    }
}

fn motion(m: &Value) -> Motion {
    Motion {
        reaction_time: f(&m["reaction_time"]),
        braking: f(&m["braking"]),
        max_accel: f(&m["max_accel"]),
        max_speed: f(&m["max_speed"]),
        max_reverse_speed: f(&m["max_reverse_speed"]),
        max_curvature: f(&m["max_curvature"]),
    }
}

fn settings(s: &Value) -> Settings {
    let d = Settings::default();
    let get = |k: &str, default: f64| s.get(k).map_or(default, f);
    Settings {
        margin: get("margin", d.margin),
        hysteresis: get("hysteresis", d.hysteresis),
        max_std: get("max_std", d.max_std),
        max_age: get("max_age", d.max_age),
        gps_max_age: get("gps_max_age", d.gps_max_age),
        settle_time: get("settle_time", d.settle_time),
        jump_margin: get("jump_margin", d.jump_margin),
    }
}

fn pose(p: &Value) -> Pose {
    let p = fs(p);
    Pose {
        x: p[0],
        y: p[1],
        yaw: p[2],
        speed: p[3],
        std: p[4],
    }
}

fn footprint(v: &Value) -> Vec<(f64, f64)> {
    v["footprint"]
        .as_array()
        .expect("footprint")
        .iter()
        .map(|c| (f(&c[0]), f(&c[1])))
        .collect()
}

#[test]
fn geofence_every_event_the_same() {
    let v = vectors();
    let corners = footprint(&v);
    let mut events = 0;
    for (r, run) in v["geofence"].as_array().expect("runs").iter().enumerate() {
        let m = &run["mode"];
        let mut g = Geofence::new(
            corners.clone(),
            fence(&run["fence"]),
            motion(&run["motion"]),
            settings(&run["settings"]),
            Mode {
                gps: m["gps"].as_bool().expect("gps"),
                coasting: m["coasting"].as_bool().expect("coasting"),
                stands: m["stands"].as_bool().expect("stands"),
            },
        )
        .expect("geofence");
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let t = f(&e["t"]);
            let out = &e["out"];
            match e["op"].as_str().expect("op") {
                "pose" => g.update(t, pose(&e["pose"])),
                "gps" => g.gps_arrived(t),
                "check" => {
                    let got = g.check(t);
                    let want = (
                        out[0].as_bool().expect("locked"),
                        out[1].as_str().expect("state"),
                        out[2].as_str().expect("why"),
                    );
                    assert_eq!(
                        (got.locked, got.state.word(), got.why.as_str()),
                        want,
                        "run {r} event {i}"
                    );
                }
                "clear" => {
                    let (ok, message) = g.clear(t);
                    assert_eq!(
                        (ok, message.as_str()),
                        (
                            out[0].as_bool().expect("ok"),
                            out[1].as_str().expect("message")
                        ),
                        "run {r} event {i}"
                    );
                }
                other => panic!("unknown op {other}"),
            }
            events += 1;
        }
    }
    assert!(events > 10_000, "only {events} events");
    println!("{events} geofence events, all the same");
}

#[test]
fn geofence_maths_to_the_bit() {
    let v = vectors();
    let m = &v["maths"];
    for (i, h) in m["hypot"].as_array().expect("hypot").iter().enumerate() {
        let a = fs(&h["args"]);
        let got = safety_core::pyfmt::hypot(a[0], a[1]);
        assert!(
            same(got, f(&h["h"])),
            "hypot {i} {a:?}: {got} vs {}",
            h["h"]
        );
    }
    for (i, d) in m["depth"].as_array().expect("depth").iter().enumerate() {
        let p = fs(&d["p"]);
        let got = fence(&d["fence"]).depth(p[0], p[1]);
        assert!(same(got, f(&d["d"])), "depth {i}: {got} vs {}", d["d"]);
    }
    for (i, o) in m["outline"].as_array().expect("outline").iter().enumerate() {
        let got = fence(&o["fence"]).outline();
        let want = o["points"].as_array().expect("points");
        assert_eq!(got.len(), want.len(), "outline {i}");
        for (g, w) in got.iter().zip(want) {
            assert!(same(g.0, f(&w[0])) && same(g.1, f(&w[1])), "outline {i}");
        }
    }
    for (i, s) in m["stopping"]
        .as_array()
        .expect("stopping")
        .iter()
        .enumerate()
    {
        let a = fs(&s["args"]);
        let got = stopping_distance(a[0], a[1], a[2], a[3], a[4]);
        assert!(same(got, f(&s["d"])), "stopping {i}: {got} vs {}", s["d"]);
    }
    for (i, s) in m["arc"].as_array().expect("arc").iter().enumerate() {
        let a = fs(&s["args"]);
        let got = along_arc(a[0], a[1], a[2], a[3], a[4]);
        let want = fs(&s["pose"]);
        assert!(
            same(got.0, want[0]) && same(got.1, want[1]) && same(got.2, want[2]),
            "arc {i}: {got:?} vs {want:?}"
        );
    }
    let corners = footprint(&v);
    for (i, s) in m["points"].as_array().expect("points").iter().enumerate() {
        let g = Geofence::new(
            corners.clone(),
            Fence::Circle(Circle::new(0.0, 0.0, 30.0).expect("circle")),
            motion(&s["motion"]),
            Settings::default(),
            Mode::default(),
        )
        .expect("geofence");
        let p = pose(&s["pose"]);
        let (fwd, back) = g.stopping(p.speed);
        let stop = fs(&s["stopping"]);
        assert!(same(fwd, stop[0]) && same(back, stop[1]), "points {i}");
        let got = g.points(&p);
        let want = s["points"].as_array().expect("points");
        assert_eq!(got.len(), want.len(), "points {i}");
        for (k, (a, w)) in got.iter().zip(want).enumerate() {
            assert!(
                same(a.0, f(&w[0])) && same(a.1, f(&w[1])),
                "points {i}.{k}: {a:?} vs {w}"
            );
        }
    }
}

#[test]
fn geofence_refuses_what_the_python_refuses() {
    let v = vectors();
    for (i, r) in v["refusals"]
        .as_array()
        .expect("refusals")
        .iter()
        .enumerate()
    {
        let want = r["error"].as_str();
        let got: Option<String> = match r["kind"].as_str().expect("kind") {
            "circle" => {
                let a = fs(&r["args"]);
                Circle::new(a[0], a[1], a[2]).err().map(|e| e.0)
            }
            "polygon" => Polygon::from_flat(&fs(&r["args"])).err().map(|e| e.0),
            "site" => site_fence(
                r["site"].as_str().expect("site"),
                &fs(&r["corners"]),
                &fs(&r["site_datum"]),
                &fs(&r["datum"]),
                1e-7,
            )
            .err()
            .map(|e| e.0),
            "motion" => motion(&r["args"]).checked().err().map(|e| e.0),
            "settings" => settings(&r["args"]).checked().err().map(|e| e.0),
            other => panic!("unknown kind {other}"),
        };
        assert_eq!(got.as_deref(), want, "refusal {i}");
    }
}

#[test]
fn stop_hold_every_event_the_same() {
    let v = vectors();
    let mut events = 0;
    for run in v["hold"]["runs"].as_array().expect("runs") {
        let mut h = StopHold::new(f(&run["clear_time"])).expect("hold");
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let t = f(&e["t"]);
            if e["op"] == "update" {
                h.update(t, e["stop"].as_bool().expect("stop"));
            } else {
                assert_eq!(Some(h.locked(t)), e["locked"].as_bool(), "event {i}");
            }
            events += 1;
        }
    }
    for r in v["hold"]["refused"].as_array().expect("refused") {
        let ct = f(&r["clear_time"]);
        assert_eq!(
            Some(StopHold::new(ct).is_ok()),
            r["ok"].as_bool(),
            "clear_time {ct}"
        );
    }
    assert!(events > 1000, "only {events} events");
}

#[test]
fn leash_every_event_the_same() {
    let v = vectors();
    let mut events = 0;
    for run in v["leash"].as_array().expect("runs") {
        let mut l = Leash::new(f(&run["timeout"]));
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let t = f(&e["t"]);
            if e["op"] == "beat" {
                l.beat(e["value"].as_u64().expect("value"), t);
            } else {
                assert_eq!(Some(l.check(t)), e["trip"].as_bool(), "event {i}");
            }
            assert_eq!(Some(l.armed()), e["armed"].as_bool(), "event {i}");
            events += 1;
        }
    }
    assert!(events > 1000, "only {events} events");
}

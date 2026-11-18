//! The Python's answers (`tests/vectors.json`, from `gen_vectors.py`): every number to
//! 1e-12, every word the same. The turn rate and pursuit go through sin, cos, atan2 and
//! tan, whose last digit may differ between platforms' libm.

use point_core::approach::{planned_deceleration, Approach, Phase};
use point_core::drive::{Drive, Steering};
use point_core::line::{
    bearing_error, pursuit_curvature, reachable, turn_speed_limit, Fence, Line, Pose,
};
use point_core::rules::{self, Kind, Limits, Now};
use point_core::run::{self, Caps, Run, Settings, Status};
use point_core::{fmt_g, py_repr};
use serde_json::Value;

use ego_core::car_model::ActuatorConfig;
use ego_core::stop_law::{StopCurve, INPUT_DELAY};

const TOL: f64 = 1e-12;

fn vectors() -> Value {
    let text = include_str!("vectors.json");
    serde_json::from_str(text).expect("vectors.json")
}

fn num(v: &Value) -> f64 {
    match v {
        Value::String(s) => match s.as_str() {
            "nan" => f64::NAN,
            "inf" => f64::INFINITY,
            "-inf" => f64::NEG_INFINITY,
            other => panic!("not a number: {other}"),
        },
        _ => v.as_f64().unwrap_or_else(|| panic!("not a number: {v}")),
    }
}

fn opt(v: &Value) -> Option<f64> {
    (!v.is_null()).then(|| num(v))
}

fn close(a: f64, b: f64, what: &str) {
    let same = (a.is_nan() && b.is_nan()) || a == b || (a - b).abs() <= TOL * b.abs().max(1.0);
    assert!(same, "{what}: rust {a} python {b}");
}

fn point(v: &Value) -> (f64, f64) {
    (num(&v[0]), num(&v[1]))
}

fn pose(v: &Value) -> Pose {
    Pose {
        x: num(&v[0]),
        y: num(&v[1]),
        yaw: num(&v[2]),
    }
}

fn text(v: &Value) -> Option<String> {
    v.as_str().map(String::from)
}

fn kind(word: &str) -> Kind {
    match word {
        "REAL" => Kind::Real,
        "SIM" => Kind::Sim,
        "BENCH" => Kind::Bench,
        "REPLAY" => Kind::Replay,
        _ => Kind::Unknown,
    }
}

fn now(v: &Value) -> Now {
    Now {
        pose_age: opt(&v[0]),
        fix_age: opt(&v[1]),
        estop: v[2].as_bool().unwrap(),
        locked: v[3].as_bool().unwrap(),
        kind: kind(v[4].as_str().unwrap()),
        taken_over: v[5].as_bool().unwrap(),
        lock_age: opt(&v[6]),
    }
}

fn limits(v: &Value) -> Limits {
    Limits {
        max_distance: num(&v["max_distance"]),
        pose_timeout: num(&v["pose_timeout"]),
        gps_timeout: num(&v["gps_timeout"]),
        require_gps: v["require_gps"].as_bool().unwrap(),
        watchdog_timeout: num(&v["watchdog_timeout"]),
    }
}

fn settings(v: &Value) -> Settings {
    let n = |k: &str| num(&v[k]);
    Settings {
        cruise_speed: n("cruise_speed"),
        acceleration: n("acceleration"),
        max_brake: n("max_brake"),
        brake_fraction: n("brake_fraction"),
        deceleration: n("deceleration"),
        brake_lead: n("brake_lead"),
        stop_band: n("stop_band"),
        creep: n("creep"),
        lookahead_min: n("lookahead_min"),
        lookahead_time: n("lookahead_time"),
        lateral_accel: n("lateral_accel"),
        still_speed: n("still_speed"),
        stuck_time: n("stuck_time"),
        still_time: n("still_time"),
        max_off_line: n("max_off_line"),
        max_overrun: n("max_overrun"),
    }
}

fn caps(v: &Value) -> Caps {
    Caps {
        max_speed: opt(&v["max_speed"]),
        max_acceleration: opt(&v["max_acceleration"]),
        max_deceleration: opt(&v["max_deceleration"]),
        max_jerk: opt(&v["max_jerk"]),
    }
}

fn curve() -> StopCurve {
    StopCurve::from_car_model(
        &ActuatorConfig::default(),
        2.0,
        2.0,
        INPUT_DELAY,
        Some(&Default::default()),
    )
    .unwrap()
}

fn phase_code(v: &Value) -> u8 {
    v.as_u64().unwrap() as u8
}

#[test]
fn the_stopping_law_is_the_pythons() {
    let v = vectors();
    let c = curve();
    let want = &v["approach"]["curve"];
    close(c.a(), num(&want[0]), "a");
    close(c.j(), num(&want[1]), "j");
    close(c.rho(), num(&want[2]), "rho");
}

#[test]
fn approach_steps_as_the_python() {
    let v = vectors();
    for (i, case) in v["approach"]["cases"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
    {
        let p = &case["plan"];
        let curve = case["curve"].as_bool().unwrap().then(curve);
        let a = Approach::new(
            num(&p["cruise"]),
            num(&p["accel"]),
            num(&p["decel"]),
            num(&p["lead"]),
            num(&p["stop_band"]),
            num(&p["creep"]),
            curve,
        )
        .unwrap();
        let s = &case["step"];
        let (left, measured) = (num(&s["left"]), num(&s["measured"]));
        let (speed, phase) = a.step(
            num(&s["previous"]),
            num(&s["dt"]),
            left,
            measured,
            num(&s["limit"]),
        );
        close(speed, num(&case["speed"]), &format!("case {i} speed"));
        assert_eq!(phase.code(), phase_code(&case["phase"]), "case {i} phase");
        close(
            a.braking_speed(left, measured),
            num(&case["braking"]),
            "braking",
        );
        close(a.reach(left), num(&case["reach"]), "reach");
        close(a.ideal_time(left.abs() + 1.0), num(&case["ideal"]), "ideal");
    }
    for case in v["approach"]["planned"].as_array().unwrap() {
        let got = planned_deceleration(num(&case["args"][0]), num(&case["args"][1]));
        match (&got, case.get("value")) {
            (Ok(d), Some(want)) => close(*d, num(want), "planned"),
            (Err(e), None) => assert_eq!(e.to_string(), case["error"].as_str().unwrap()),
            _ => panic!("planned_deceleration {case}: {got:?}"),
        }
    }
}

#[test]
fn the_line_and_its_corridor_are_the_pythons() {
    let v = vectors();
    let radius = num(&v["line"]["radius"]);
    for (i, c) in v["line"]["cases"].as_array().unwrap().iter().enumerate() {
        let pose = pose(&c["pose"]);
        let target = point(&c["point"]);
        close(
            pursuit_curvature(&pose, target, 0.35),
            num(&c["curvature"]),
            "curvature",
        );
        close(bearing_error(&pose, target), num(&c["bearing"]), "bearing");
        assert_eq!(
            reachable(&pose, target, radius, 0.5),
            c["reachable"].as_bool().unwrap(),
            "case {i} reachable"
        );
        let line = Line {
            a: point(&c["line"][0]),
            b: point(&c["line"][1]),
        };
        let p = point(&c["p"]);
        let (left, cross) = line.progress(p).unwrap();
        close(left, num(&c["progress"][0]), "left");
        close(cross, num(&c["progress"][1]), "cross");
        let carrot = line.carrot(p, num(&c["look"])).unwrap();
        close(carrot.0, num(&c["carrot"][0]), "carrot x");
        close(carrot.1, num(&c["carrot"][1]), "carrot y");
        let fence = Fence::new(
            line.a,
            line.b,
            radius,
            num(&c["fence"][0]),
            num(&c["fence"][1]),
        )
        .unwrap();
        assert_eq!(
            fence.breach(p, Some(&line)).map(|b| b.to_string()),
            text(&c["breach_line"]),
            "case {i} breach on the line"
        );
        assert_eq!(
            fence.breach(p, None).map(|b| b.to_string()),
            text(&c["breach"]),
            "case {i} breach"
        );
        let _ = turn_speed_limit(0.0, 1.5);
    }
}

#[test]
fn whole_drives_tick_as_the_python() {
    let v = vectors();
    for (d, drive) in v["drives"].as_array().unwrap().iter().enumerate() {
        let p = &drive["plan"];
        let a = Approach::new(
            num(&p["cruise"]),
            num(&p["accel"]),
            num(&p["decel"]),
            num(&p["lead"]),
            num(&p["stop_band"]),
            num(&p["creep"]),
            None,
        )
        .unwrap();
        let start = pose(&drive["start"]);
        let mut run = Drive::new(
            point(&drive["target"]),
            a,
            Steering::new(num(&drive["max_curvature"])),
            &start,
        );
        for (i, t) in drive["ticks"].as_array().unwrap().iter().enumerate() {
            let cmd = run.tick(&pose(&t[0]), num(&t[1]), num(&t[2]));
            let want = &t[3];
            let what = format!("drive {d} tick {i}");
            close(cmd.speed, num(&want[0]), &what);
            close(cmd.curvature, num(&want[1]), &what);
            assert_eq!(cmd.phase.code(), phase_code(&want[2]), "{what} phase");
            close(cmd.left, num(&want[3]), &what);
            close(cmd.cross, num(&want[4]), &what);
        }
    }
}

#[test]
fn the_rules_say_the_same() {
    let v = vectors();
    let radius = num(&v["rules"]["radius"]);
    for (i, c) in v["rules"]["cases"].as_array().unwrap().iter().enumerate() {
        let now = now(&c["now"]);
        let lim = limits(&c["limits"]);
        let confirmed = c["confirmed"].as_bool().unwrap();
        let what = format!("case {i}");
        assert_eq!(
            rules::refuse_start(&now, &lim, confirmed).map(|r| r.to_string()),
            text(&c["start"]),
            "{what} start"
        );
        assert_eq!(
            rules::stop_reason(&now, &lim).map(|r| r.to_string()),
            text(&c["stop"]),
            "{what} stop"
        );
        assert_eq!(
            rules::refuse_target(&pose(&c["pose"]), point(&c["target"]), &lim, radius)
                .map(|r| r.to_string()),
            text(&c["target_why"]),
            "{what} target"
        );
        let n = &c["numbers"];
        assert_eq!(
            rules::check_numbers(num(&n[0]), num(&n[1]), num(&n[2])).map(|r| r.to_string()),
            text(&c["numbers_why"]),
            "{what} numbers"
        );
        assert_eq!(
            rules::refuse_outside(opt(&c["depth"])).map(|r| r.to_string()),
            text(&c["outside"]),
            "{what} geofence"
        );
    }
    let small = &v["rules"]["small"];
    for c in small["tolerance"].as_array().unwrap() {
        let status = c[0].as_i64().map(|s| s as i8);
        close(rules::tolerance(status), num(&c[1]), "tolerance");
    }
    for c in small["confirmed"].as_array().unwrap() {
        assert_eq!(
            rules::confirmed(c[0].as_str()),
            c[1].as_bool().unwrap(),
            "{c}"
        );
    }
    for c in small["not_moving"].as_array().unwrap() {
        assert_eq!(
            rules::not_moving(num(&c[0]), num(&c[1]), 0.05, 0.2),
            c[2].as_bool().unwrap(),
            "{c}"
        );
    }
    for c in small["source_kind"].as_array().unwrap() {
        let values = c[0]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| x.as_u64().unwrap() as u8);
        assert_eq!(
            rules::source_kind(values).word(),
            c[1].as_str().unwrap(),
            "{c}"
        );
    }
    for c in small["taken_over"].as_array().unwrap() {
        let ours = c[0].as_array().unwrap().iter().map(point);
        assert_eq!(
            rules::taken_over(ours, point(&c[1])),
            c[2].as_bool().unwrap(),
            "{c}"
        );
    }
}

#[test]
fn plans_are_the_pythons() {
    let v = vectors();
    let car = ActuatorConfig::default();
    for (i, c) in v["plans"].as_array().unwrap().iter().enumerate() {
        let g = &c["goal"];
        let got = run::plan(
            num(&g[0]),
            num(&g[1]),
            num(&g[2]),
            &caps(&c["caps"]),
            &settings(&c["settings"]),
            c["car"].as_bool().unwrap().then_some(&car),
        );
        let what = format!("plan {i}");
        match (&got, &c["approach"]) {
            (Ok(a), want) if !want.is_null() => {
                close(a.cruise, num(&want["cruise"]), &what);
                close(a.accel, num(&want["accel"]), &what);
                close(a.decel, num(&want["decel"]), &what);
                close(a.lead, num(&want["lead"]), &what);
                close(a.stop_band, num(&want["stop_band"]), &what);
                close(a.creep, num(&want["creep"]), &what);
                match (&a.curve, &want["curve"]) {
                    (None, Value::Null) => {}
                    (Some(curve), w) => {
                        close(curve.a(), num(&w[0]), &what);
                        close(curve.j(), num(&w[1]), &what);
                        close(curve.rho(), num(&w[2]), &what);
                    }
                    other => panic!("{what}: curve {other:?}"),
                }
            }
            (Err(e), Value::Null) => {
                assert_eq!(Some(e.to_string()), text(&c["why"]), "{what}");
            }
            other => panic!("{what}: {other:?}"),
        }
    }
}

#[test]
fn whole_runs_end_as_the_pythons() {
    let v = vectors();
    let car = ActuatorConfig::default();
    for r in v["runs"].as_array().unwrap() {
        let name = r["name"].as_str().unwrap();
        let s = settings(&r["settings"]);
        let approach = run::plan(
            0.0,
            0.0,
            0.0,
            &caps(&r["caps"]),
            &s,
            r["hybrid"].as_bool().unwrap().then_some(&car),
        )
        .unwrap();
        let mut run = Run::new(
            point(&r["target"]),
            approach,
            s,
            limits(&r["limits"]),
            num(&r["radius"]),
            &pose(&r["start"]),
            num(&r["t0"]),
        )
        .unwrap();
        close(run.max_path(), num(&r["max_path"]), name);
        close(run.max_time(), num(&r["max_time"]), name);
        for (i, t) in r["ticks"].as_array().unwrap().iter().enumerate() {
            let what = format!("{name} tick {i}");
            let step = run.tick(
                num(&t[0]),
                &now(&t[1]),
                &pose(&t[2]),
                num(&t[3]),
                t[4].as_bool().unwrap(),
            );
            match (step.send, &t[5]) {
                (None, Value::Null) => {}
                (Some((speed, k)), w) if !w.is_null() => {
                    close(speed, num(&w[0]), &what);
                    close(k, num(&w[1]), &what);
                }
                other => panic!("{what}: send {other:?}"),
            }
            assert_eq!(step.stopped.map(|h| h.to_string()), text(&t[6]), "{what}");
            assert_eq!(step.finished, t[7].as_bool().unwrap(), "{what} finished");
            match (step.feedback, &t[8]) {
                (None, Value::Null) => {}
                (Some(c), w) if !w.is_null() => {
                    close(c.speed, num(&w[0]), &what);
                    assert_eq!(c.phase.code(), phase_code(&w[2]), "{what}");
                    close(c.left, num(&w[3]), &what);
                    close(c.cross, num(&w[4]), &what);
                }
                other => panic!("{what}: feedback {other:?}"),
            }
        }
        let fix = r["fix_status"].as_i64().map(|s| s as i8);
        let end = run.outcome(&pose(&r["end_pose"]), fix, true, num(&r["end_t"]));
        let want = &r["outcome"];
        let status = match end.status {
            Status::Succeeded => "succeeded",
            Status::Aborted => "aborted",
            Status::Cancelled => "cancelled",
        };
        assert_eq!(status, want["status"].as_str().unwrap(), "{name}");
        assert_eq!(end.success, want["success"].as_bool().unwrap(), "{name}");
        assert_eq!(end.message, want["message"].as_str().unwrap(), "{name}");
        close(end.left, num(&want["left"]), name);
        close(end.tolerance, num(&want["tolerance"]), name);
        close(end.travelled, num(&want["travelled"]), name);
        close(end.duration, num(&want["duration"]), name);
    }
}

#[test]
fn small_words_match_python() {
    assert_eq!(fmt_g(2.0), "2");
    assert_eq!(py_repr(-0.5), "-0.5");
    assert_eq!(Phase::Stopping.code(), 4);
}

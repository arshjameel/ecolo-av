//! The Python's answers (`vectors.json`, from `gen_vectors.py`): watchdog-core must give
//! exactly the same, every tick: each input's state, its words, whether it stops the
//! drivers, the lock, the state line, what is logged, and `/diagnostics` once a second.
//! A safety lock: no tolerance, the text compared as text.

use std::collections::HashSet;
use std::fs;

use serde_json::Value;
use watchdog_core::feed::{Feed, Said};
use watchdog_core::picture_digest;
use watchdog_core::watch::{Kind, Rule, Severity, Watchdog};
use watchdog_core::wheels::CrossCheck;

fn vectors() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors.json");
    serde_json::from_str(&fs::read_to_string(path).expect("vectors.json")).expect("json")
}

/// A number; 'inf', '-inf' and 'nan' as written by gen_vectors.py.
fn f(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "inf" => f64::INFINITY,
        Value::String(s) if s == "-inf" => f64::NEG_INFINITY,
        Value::String(s) if s == "nan" => f64::NAN,
        _ => v.as_f64().unwrap_or_else(|| panic!("not a number: {v}")),
    }
}

fn text(v: &Value) -> &str {
    v.as_str().unwrap_or_else(|| panic!("not text: {v}"))
}

fn rule(d: &Value) -> Rule {
    let cross = d["cross_check"].as_object().map(|c| CrossCheck {
        window: f(&c["window"]),
        speed_tolerance: f(&c["speed_tolerance"]),
        yaw_tolerance: f(&c["yaw_tolerance"]),
        persist: f(&c["persist"]),
        bias_time: f(&c["bias_time"]),
        gps_window: f(&c["gps_window"]),
        gps_tolerance: f(&c["gps_tolerance"]),
    });
    Rule {
        name: text(&d["name"]).into(),
        topic: text(&d["topic"]).into(),
        kind: text(&d["kind"]).parse::<Kind>().expect("kind"),
        max_age: f(&d["max_age"]),
        stop: d["stop"].as_bool().expect("stop"),
        frozen_after: f(&d["frozen_after"]),
        max_jump_speed: f(&d["max_jump_speed"]),
        jump_margin: f(&d["jump_margin"]),
        cross_check: cross,
        frame_id: text(&d["frame_id"]).into(),
        latch: d["latch"].as_bool().expect("latch"),
        latch_grace: f(&d["latch_grace"]),
        follow_tolerance: f(&d["follow_tolerance"]),
        follow_persist: f(&d["follow_persist"]),
        over_margin: f(&d["over_margin"]),
        over_warn: f(&d["over_warn"]),
        over_persist: f(&d["over_persist"]),
        over_slowing: f(&d["over_slowing"]),
        status_name: text(&d["status_name"]).into(),
        min_level: Severity::from_min_level(d["min_level"].as_i64().expect("min_level"))
            .expect("1, 2 or 3"),
        quiet_ok: d["quiet_ok"].as_bool().expect("quiet_ok"),
    }
    .checked()
    .expect("a good rule")
}

fn numbers<const N: usize>(v: &Value) -> [f64; N] {
    let a = v.as_array().expect("numbers");
    std::array::from_fn(|i| f(&a[i]))
}

#[test]
fn every_tick_is_the_pythons() {
    let v = vectors();
    let mut ticks_seen = 0;
    let mut states_seen = HashSet::new();
    for (r, run) in v["runs"].as_array().expect("runs").iter().enumerate() {
        let rules: Vec<Rule> = run["rules"]
            .as_array()
            .expect("rules")
            .iter()
            .map(rule)
            .collect();
        // The node's: stall five ticks at 10 Hz (0.5 s), settle 3 s.
        let mut feed = Feed::new(Watchdog::new(rules, 0.5, 3.0));
        let mut want = run["ticks"].as_array().expect("ticks").iter();
        let mut tick_no = 0;
        for e in run["events"].as_array().expect("events") {
            let t = f(&e[1]);
            match text(&e[0]) {
                "odom" => feed.odom(t, text(&e[2]), numbers::<5>(&e[3])),
                "imu" => feed.imu(t, text(&e[2]), numbers::<6>(&e[3])),
                "gps" => feed.gps(
                    t,
                    text(&e[2]),
                    f(&e[3]),
                    (f(&e[4]), f(&e[5])),
                    e[6].as_bool().expect("has_fix"),
                ),
                "image" => {
                    let digest = u32::try_from(e[3].as_u64().expect("digest")).expect("u32");
                    feed.image(t, text(&e[2]), digest);
                }
                "detections" => {
                    let values = e[4].as_array().expect("boxes").iter().map(f).collect();
                    feed.detections(t, text(&e[2]), text(&e[3]), values);
                }
                "heartbeat" => feed.heartbeat(t, text(&e[2]), e[3].as_u64().expect("count")),
                "actuator" => feed.actuator(t, text(&e[2]), f(&e[3]), f(&e[4])),
                "speed" => feed.speed(t, text(&e[2]), f(&e[3])),
                "cloud" => {
                    let digest = u32::try_from(e[3].as_u64().expect("digest")).expect("u32");
                    feed.cloud(t, text(&e[2]), digest);
                }
                "status" => {
                    let level = u8::try_from(e[4].as_u64().expect("level")).expect("u8");
                    feed.status(t, text(&e[2]), text(&e[3]), level, text(&e[5]));
                }
                "clear" => feed.clear(),
                "tick" => {
                    let running: HashSet<String> = e[3]
                        .as_array()
                        .expect("running")
                        .iter()
                        .map(|n| text(n).to_string())
                        .collect();
                    let got = feed.tick(t, f(&e[2]), &running);
                    let w = want.next().expect("a tick to compare");
                    let at = format!("run {r}, tick {tick_no} ({t} s)");
                    let ws = w["states"].as_array().expect("states");
                    assert_eq!(got.states.len(), ws.len(), "{at}");
                    for (g, s) in got.states.iter().zip(ws) {
                        assert_eq!(g.name, text(&s[0]), "{at}");
                        assert_eq!(g.state.as_str(), text(&s[1]), "{at}: {}", g.name);
                        assert_eq!(g.detail, text(&s[2]), "{at}: {}", g.name);
                        assert_eq!(g.stops, s[3].as_bool().expect("stops"), "{at}: {}", g.name);
                        states_seen.insert(g.state);
                    }
                    assert_eq!(got.locked, w["locked"].as_bool().expect("locked"), "{at}");
                    assert_eq!(got.summary, text(&w["summary"]), "{at}");
                    let said: Vec<(String, String)> = got
                        .said
                        .iter()
                        .map(|s| match s {
                            Said::Back(x) => ("back".to_string(), x.clone()),
                            Said::Stops(x) => ("stops".to_string(), x.clone()),
                            Said::Warns(x) => ("warns".to_string(), x.clone()),
                        })
                        .collect();
                    let want_said: Vec<(String, String)> = w["said"]
                        .as_array()
                        .expect("said")
                        .iter()
                        .map(|s| (text(&s[0]).to_string(), text(&s[1]).to_string()))
                        .collect();
                    assert_eq!(said, want_said, "{at}");
                    if let Some(diag) = w.get("diagnostics") {
                        let statuses = feed.diagnostics(t, &got.states);
                        let wd = diag.as_array().expect("diagnostics");
                        assert_eq!(statuses.len(), wd.len(), "{at}");
                        for (g, d) in statuses.iter().zip(wd) {
                            assert_eq!(g.name, text(&d[0]), "{at}");
                            assert_eq!(u64::from(g.level.byte()), d[1].as_u64().expect("level"));
                            assert_eq!(g.message, text(&d[2]), "{at}");
                            assert_eq!(g.hardware_id, text(&d[3]), "{at}");
                            let values: Vec<(String, String)> = d[4]
                                .as_array()
                                .expect("values")
                                .iter()
                                .map(|kv| (text(&kv[0]).to_string(), text(&kv[1]).to_string()))
                                .collect();
                            assert_eq!(g.values, values, "{at}: {}", g.name);
                        }
                    }
                    tick_no += 1;
                    ticks_seen += 1;
                }
                other => panic!("unknown event {other}"),
            }
        }
        assert!(want.next().is_none(), "run {r}: ticks left over");
    }
    println!(
        "{ticks_seen} ticks the same, states seen: {:?}",
        states_seen
    );
    assert!(ticks_seen >= 1600);
    // Every state was exercised.
    assert_eq!(states_seen.len(), 12, "{states_seen:?}");
}

#[test]
fn a_pictures_digest_is_pythons() {
    let v = vectors();
    for p in v["pictures"].as_array().expect("pictures") {
        let hex = text(&p["hex"]);
        let data: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex"))
            .collect();
        assert_eq!(
            u64::from(picture_digest(&data)),
            p["digest"].as_u64().expect("digest"),
            "{} bytes",
            data.len()
        );
    }
}

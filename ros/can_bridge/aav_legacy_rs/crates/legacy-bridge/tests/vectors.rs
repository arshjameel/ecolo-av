//! The Python modules' answers (gen_vectors.py → vectors.json), and ours: the same, exactly.

use legacy_bridge::link::frame_fields;
use legacy_bridge::mapping::{from_pedals, to_legacy, Mapping};
use legacy_bridge::uplink::{self, UplinkReader};
use legacy_bridge::{
    programs_locked, CalMode, CalRefusal, CalWhere, FrameVersion, LinkReport, Relay, RelaySettings,
    SpeedConfig, SpeedControl,
};
use serde_json::Value;

fn vectors() -> Value {
    let text = include_str!("vectors.json");
    serde_json::from_str(text).expect("vectors.json")
}

/// A number as Python wrote it ("nan" for NaN), or None for null.
fn num(v: &Value) -> Option<f64> {
    match v {
        Value::Null => None,
        Value::String(s) if s == "nan" => Some(f64::NAN),
        Value::String(s) if s == "inf" => Some(f64::INFINITY),
        Value::String(s) if s == "-inf" => Some(f64::NEG_INFINITY),
        other => Some(other.as_f64().expect("a number")),
    }
}

fn f(v: &Value) -> f64 {
    num(v).expect("not null")
}

/// The same number, bit for bit (NaN is NaN).
fn same(a: f64, b: f64) -> bool {
    (a.is_nan() && b.is_nan()) || a == b
}

fn assert_same(ours: f64, theirs: &Value, what: &str) {
    let t = f(theirs);
    assert!(same(ours, t), "{what}: ours {ours:?}, Python's {t:?}");
}

fn mapping(v: &Value) -> Mapping {
    Mapping {
        max_steering_angle: f(&v["max_steering_angle"]),
        steering_sign: f(&v["steering_sign"]),
        full_throttle_speed: f(&v["full_throttle_speed"]),
        stop_speed: f(&v["stop_speed"]),
        hold_brake: f(&v["hold_brake"]),
        stop_brake: f(&v["stop_brake"]),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex"))
        .collect()
}

#[test]
fn the_mapping_matches() {
    let v = vectors();
    let m = mapping(&v["mapping"]);
    let cases = v["mapping_cases"].as_array().expect("cases");
    for c in cases {
        let out = match c["op"].as_str() {
            Some("to_legacy") => to_legacy(f(&c["speed"]), f(&c["steering"]), &m),
            _ => from_pedals(f(&c["steering"]), f(&c["throttle"]), f(&c["brake"]), &m),
        };
        let what = format!("{c}");
        assert_same(out.steering, &c["out"][0], &what);
        assert_same(out.speed, &c["out"][1], &what);
        assert_same(out.accel, &c["out"][2], &what);
    }
    assert!(cases.len() > 100);
}

#[test]
fn the_speed_loop_matches_tick_by_tick() {
    let v = vectors();
    for run in v["speed_runs"].as_array().expect("runs") {
        let mut cfg = SpeedConfig::default();
        for (name, value) in run["config"].as_object().expect("config") {
            cfg.set(name, f(value)).expect("a setting");
        }
        let mut loop_ = SpeedControl::new(cfg);
        for (i, s) in run["steps"].as_array().expect("steps").iter().enumerate() {
            let p = loop_
                .update(f(&s["target"]), num(&s["measured"]), f(&s["dt"]))
                .expect("a step");
            let ours = [p.throttle, p.brake, p.accel, p.feedforward, p.integral];
            for (k, o) in ours.iter().enumerate() {
                assert_same(*o, &s["out"][k], &format!("step {i}, output {k}"));
            }
        }
    }
}

fn refusal_words(r: Option<CalRefusal>) -> Value {
    r.map_or(Value::Null, |r| Value::String(r.to_string()))
}

#[test]
fn the_relay_writes_the_same_bytes() {
    let v = vectors();
    let m = mapping(&v["mapping"]);
    let mut frames = 0;
    for run in v["relay_runs"].as_array().expect("runs") {
        let version =
            FrameVersion::try_from(run["version"].as_i64().expect("version")).expect("1, 2 or 3");
        let calibration: CalWhere = run["calibration"]
            .as_str()
            .expect("where")
            .parse()
            .expect("none or stands");
        assert_eq!(programs_locked(version, false), run["locked"] == true);
        assert_eq!(
            programs_locked(version, true),
            run["locked_allowed"] == true
        );
        let speed_loop = (run["mode"] == "pi").then(|| SpeedControl::new(SpeedConfig::default()));
        let mut relay = Relay::new(
            RelaySettings {
                mapping: m,
                cmd_timeout: 0.3,
                version,
                odom_timeout: 0.3,
                calibration,
                uplink_timeout: 0.2,
            },
            speed_loop,
        );
        for (i, e) in run["events"].as_array().expect("events").iter().enumerate() {
            let what = format!(
                "{} {} {}: event {i} {e}",
                run["version"], run["mode"], run["calibration"]
            );
            match e["op"].as_str().expect("op") {
                "cmd" => relay.command(f(&e["speed"]), f(&e["steering"]), f(&e["t"])),
                "hb" => {
                    relay.heartbeat(u8::try_from(e["counter"].as_u64().expect("u8")).expect("u8"))
                }
                "odom" => relay.wheel_speed(f(&e["speed"]), f(&e["t"])),
                "estop" => assert_eq!(
                    relay.set_estop(e["on"] == true),
                    e["changed"] == true,
                    "{what}"
                ),
                "cal" => {
                    let mode = u8::try_from(e["mode"].as_u64().expect("mode")).expect("u8");
                    relay.calibrate(
                        CalMode::try_from(mode).expect("a mode"),
                        e["target"].as_i64().expect("target"),
                        f(&e["t"]),
                    );
                }
                "uplink" => relay.uplink_seen(f(&e["t"])),
                "frame" => {
                    let t = f(&e["t"]);
                    let frame = relay.frame(t);
                    assert_eq!(hex(&frame), e["hex"], "{what}");
                    assert_eq!(relay.fresh(t), e["fresh"] == true, "{what}");
                    assert_eq!(relay.cal_wanted(t), e["cal_wanted"] == true, "{what}");
                    assert_eq!(refusal_words(relay.cal_refusal(t)), e["refusal"], "{what}");
                    frames += 1;
                }
                "stop_frame" => assert_eq!(hex(&relay.stop_frame()), e["hex"], "{what}"),
                other => panic!("unknown op {other}"),
            }
        }
    }
    assert!(frames > 3000);
}

#[test]
fn the_uplink_reads_the_same() {
    let v = vectors();
    let u = &v["uplink"];
    let mut reader = UplinkReader::new(0.2);
    let mut found_total = 0;
    for (i, r) in u["reads"].as_array().expect("reads").iter().enumerate() {
        let t = f(&r["t"]);
        let found = reader.feed(&unhex(r["hex"].as_str().expect("hex")), t);
        let theirs = r["found"].as_array().expect("found");
        assert_eq!(found.len(), theirs.len(), "read {i}");
        for (ours, th) in found.iter().zip(theirs) {
            let what = format!("read {i}, sequence {}", th["sequence"]);
            assert_eq!(
                u64::from(ours.sequence),
                th["sequence"].as_u64().expect("seq")
            );
            let s = uplink::status(ours);
            let ts = &th["status"];
            assert_eq!(
                u64::from(s.mode),
                ts["mode"].as_u64().expect("mode"),
                "{what}"
            );
            assert_eq!(s.armed, ts["armed"] == true, "{what}");
            assert_eq!(s.estop_active, ts["estop_active"] == true, "{what}");
            assert_same(f64::from(s.speed), &ts["speed"], &what);
            assert_same(f64::from(s.steering_angle), &ts["steering_angle"], &what);
            assert_same(f64::from(s.throttle), &ts["throttle"], &what);
            assert_same(f64::from(s.brake), &ts["brake"], &what);
            assert_eq!(u64::from(s.faults), ts["faults"].as_u64().expect("faults"));
            let pairs = [
                (
                    uplink::steering_state(ours, num(&th["command"])),
                    "steering",
                ),
                (uplink::brake_state(ours), "brake"),
                (uplink::throttle_state(ours), "throttle"),
            ];
            for ((c, m), name) in pairs {
                assert_same(f64::from(c), &th[name][0], &format!("{what} {name}"));
                assert_same(f64::from(m), &th[name][1], &format!("{what} {name}"));
            }
            let rc = uplink::rc(ours);
            assert_eq!(
                u64::from(rc.mode_switch),
                th["rc"]["mode_switch"].as_u64().expect("sw")
            );
            assert_same(f64::from(rc.throttle), &th["rc"]["throttle"], &what);
            assert_same(f64::from(rc.steering), &th["rc"]["steering"], &what);
            assert_eq!(rc.brake, th["rc"]["brake"] == true, "{what}");
            for (raw, name) in [
                (
                    uplink::raw(ours.counts.steering, ours.calibration),
                    "raw_steering",
                ),
                (
                    uplink::raw(ours.counts.brake, legacy_link::CalState::Off),
                    "raw_brake",
                ),
            ] {
                let t = &th[name];
                assert_eq!(raw.valid, t["valid"] == true, "{what} {name}");
                assert_eq!(
                    u64::from(raw.position),
                    t["position"].as_u64().expect("pos")
                );
                assert_eq!(
                    u64::from(raw.calibration),
                    t["calibration"].as_u64().expect("cal")
                );
            }
            found_total += 1;
        }
        for (name, at) in [("health", t), ("health_later", t + 0.25)] {
            let h = reader.health(at);
            let th = &r[name];
            assert_eq!(h.fresh, th["fresh"] == true, "read {i} {name}");
            assert_eq!(
                u64::from(h.frames),
                th["frames"].as_u64().expect("n"),
                "read {i}"
            );
            assert_eq!(
                u64::from(h.lost),
                th["lost"].as_u64().expect("n"),
                "read {i}"
            );
            assert_eq!(u64::from(h.bad), th["bad"].as_u64().expect("n"), "read {i}");
        }
        assert_eq!(
            uplink::calibration_line(false, None, reader.latest()),
            r["line"],
            "read {i}"
        );
        assert_eq!(
            uplink::calibration_line(true, Some(CalRefusal::Estop), reader.latest()),
            r["line_refused"],
            "read {i}"
        );
    }
    assert!(found_total >= 60);
}

#[test]
fn the_link_report_says_the_same() {
    let v = vectors();
    let link = &v["link"];
    for c in link["frames"].as_array().expect("frames") {
        let bytes = unhex(c["hex"].as_str().expect("hex"));
        match (frame_fields(&bytes), &c["fields"]) {
            (Err(_), Value::Null) => {}
            (Ok(ours), t) if !t.is_null() => {
                assert_eq!(u64::from(ours.version), t["version"].as_u64().expect("v"));
                assert_eq!(u64::from(ours.counter), t["counter"].as_u64().expect("c"));
                assert_eq!(ours.armed, t["armed"] == true);
                assert_eq!(ours.estop, t["estop"] == true);
                assert_same(f64::from(ours.steering), &t["steering"], "steering");
                assert_same(f64::from(ours.speed), &t["speed"], "speed");
                assert_same(f64::from(ours.accel), &t["accel"], "accel");
                assert_eq!(u64::from(ours.cal_mode), t["cal_mode"].as_u64().expect("m"));
                assert_eq!(
                    u64::from(ours.jog_target),
                    t["jog_target"].as_u64().expect("j")
                );
                assert_eq!(u64::from(ours.check), t["check"].as_u64().expect("check"));
            }
            (ours, theirs) => panic!("{c}: ours {ours:?}, Python's {theirs}"),
        }
    }
    for r in link["reports"].as_array().expect("reports") {
        let mut rep = LinkReport::new(f(&r["rate"])).expect("a rate");
        for s in r["steps"].as_array().expect("steps") {
            let sent = usize::try_from(s[1].as_u64().expect("sent")).expect("usize");
            assert_eq!(rep.wrote(&[0; 17], sent), s[2] == true, "{s}");
            assert_eq!(rep.due(f(&s[0])), s[3] == true, "{s}");
        }
        assert_eq!(u64::from(rep.written()), r["written"].as_u64().expect("n"));
        assert_eq!(
            u64::from(rep.write_errors()),
            r["write_errors"].as_u64().expect("n")
        );
    }
}

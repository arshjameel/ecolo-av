//! The Python's answers (`vectors.json`, from `gen_vectors.py`): io-mux-core must give
//! exactly the same at every step: each SourceCheck and what the sensor mux logs; each
//! state line and what the output mux logs; each command passed on or held, with its
//! heartbeat counter; each clear's answer; each refusal at start. A safety lock: no
//! tolerance, the text compared as text.

use std::collections::{HashMap, HashSet};
use std::fs;

use io_mux_core::muxes::{screen_names, Log, OutputMux, Published, SensorMux};
use io_mux_core::provenance::{Check, Entry, Kind, SENSORS};
use serde_json::{json, Value};

fn vectors() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors.json");
    serde_json::from_str(&fs::read_to_string(path).expect("vectors.json")).expect("json")
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("not a number: {v}"))
}

fn text(v: &Value) -> &str {
    v.as_str().unwrap_or_else(|| panic!("not text: {v}"))
}

fn texts(v: &Value) -> Vec<String> {
    v.as_array()
        .expect("a list")
        .iter()
        .map(|s| text(s).to_string())
        .collect()
}

fn kind(v: &Value) -> Kind {
    Kind::from_byte(u8::try_from(v.as_u64().expect("a byte")).expect("a byte"))
}

fn alive(step: &Value) -> HashSet<String> {
    texts(&step["alive"]).into_iter().collect()
}

fn entries(announcer: &str, v: &Value) -> Vec<Entry> {
    v.as_array()
        .expect("entries")
        .iter()
        .map(|e| Entry {
            announcer: announcer.to_string(),
            name: text(&e[0]).to_string(),
            topic: text(&e[1]).to_string(),
            kind: kind(&e[2]),
        })
        .collect()
}

fn log_json(log: Option<&Log>) -> Value {
    log.map_or(Value::Null, |l| json!([l.level.as_str(), l.text]))
}

fn check_json(c: &Check) -> Value {
    json!({
        "source": c.source.byte(),
        "ok": c.ok,
        "problems": c.problems,
        "doubts": c.doubts,
        "sensors": c.sensors.iter()
            .map(|s| json!([s.name, s.topic, s.kind.byte()]))
            .collect::<Vec<_>>(),
    })
}

fn published_json(p: Option<&Published>) -> Value {
    p.map_or(
        Value::Null,
        |p| json!({"line": p.line, "log": log_json(p.log.as_ref())}),
    )
}

/// `want` as written: 'same' means the last non-null one in the run.
fn expect(want: &Value, last: &mut HashMap<&'static str, Value>, key: &'static str) -> Value {
    let full = if want == "same" {
        last.get(key).cloned().expect("a 'same' after one")
    } else {
        want.clone()
    };
    if !full.is_null() {
        last.insert(key, full.clone());
    }
    full
}

fn sensor_run(run: &Value, r: usize) -> usize {
    let p = &run["params"];
    let mux = SensorMux::new(
        text(&p["source"]),
        f(&p["start"]),
        f(&p["startup"]),
        f(&p["hold"]),
        f(&p["grace"]),
    );
    let mut mux = match mux {
        Ok(m) => {
            assert!(run["error"].is_null(), "run {r}: the Python refused");
            m
        }
        Err(e) => {
            assert_eq!(e.to_string(), text(&run["error"]), "run {r}");
            return 0;
        }
    };
    let mut last = HashMap::new();
    for (i, step) in run["steps"].as_array().expect("steps").iter().enumerate() {
        let now = f(&step["t"]);
        match text(&step["do"]) {
            "hear" => {
                let a = text(&step["announcer"]);
                mux.hear(a, entries(a, &step["entries"]), now);
            }
            "tick" => {
                let publishers: HashMap<String, Vec<String>> = step["publishers"]
                    .as_object()
                    .expect("publishers")
                    .iter()
                    .map(|(t, p)| (t.clone(), texts(p)))
                    .collect();
                let got = mux.tick(now, &alive(step), &publishers);
                let (check, log) = match &got {
                    Some((c, l)) => (check_json(c), log_json(l.as_ref())),
                    None => (Value::Null, Value::Null),
                };
                let want = expect(&step["check"], &mut last, "check");
                assert_eq!(check, want, "run {r} step {i} (t {now}): the check");
                assert_eq!(log, step["log"], "run {r} step {i} (t {now}): the log");
            }
            other => panic!("run {r}: a sensor mux step {other}"),
        }
    }
    run["steps"].as_array().map_or(0, Vec::len)
}

fn output_run(run: &Value, r: usize, counters: &mut Vec<u8>) -> usize {
    let p = &run["params"];
    let mux = OutputMux::new(
        text(&p["target"]),
        f(&p["settle"]),
        f(&p["stale_after"]),
        screen_names(text(&p["screens"])),
    );
    let mut mux = match mux {
        Ok(m) => {
            assert!(run["error"].is_null(), "run {r}: the Python refused");
            m
        }
        Err(e) => {
            assert_eq!(e.to_string(), text(&run["error"]), "run {r}");
            return 0;
        }
    };
    let mut last = HashMap::new();
    for (i, step) in run["steps"].as_array().expect("steps").iter().enumerate() {
        let now = f(&step["t"]);
        let at = format!("run {r} step {i} (t {now}, {})", step["do"]);
        let published = match text(&step["do"]) {
            "check" => Some(mux.check(
                now,
                &alive(step),
                kind(&step["source"]),
                step["ok"].as_bool().expect("ok"),
                texts(&step["problems"]),
                texts(&step["doubts"]),
            )),
            "source" => {
                let a = text(&step["announcer"]);
                Some(mux.source(now, &alive(step), a, entries(a, &step["entries"])))
            }
            "timer" => Some(Some(mux.timer(now, &alive(step)))),
            "clear" => {
                let (cleared, published) = mux.clear(now, &alive(step));
                let (success, message) = match cleared {
                    Ok(m) => (true, m),
                    Err(m) => (false, m),
                };
                assert_eq!(json!(success), step["success"], "{at}: success");
                assert_eq!(message, text(&step["message"]), "{at}: message");
                Some(published)
            }
            "command" => {
                let counter = mux.command(now);
                assert_eq!(json!(counter), step["counter"], "{at}: the heartbeat");
                counters.extend(counter);
                None
            }
            "beat" => {
                mux.beat(text(&step["screen"]), now);
                None
            }
            other => panic!("{at}: an output mux step {other}"),
        };
        if let Some(published) = published {
            let want = expect(&step["published"], &mut last, "published");
            assert_eq!(published_json(published.as_ref()), want, "{at}: the state");
        }
    }
    run["steps"].as_array().map_or(0, Vec::len)
}

#[test]
fn every_step_is_the_pythons() {
    let v = vectors();
    let watched: Vec<Value> = SENSORS.iter().map(|(n, t)| json!([n, t])).collect();
    assert_eq!(v["sensors"], json!(watched), "the watched sensors");
    let (mut steps, mut runs, mut refused) = (0, 0, 0);
    let mut counters = Vec::new();
    let mut lines = HashSet::new();
    for (r, run) in v["runs"].as_array().expect("runs").iter().enumerate() {
        let n = match text(&run["node"]) {
            "sensor_mux" => sensor_run(run, r),
            "output_mux" => output_run(run, r, &mut counters),
            other => panic!("run {r}: node {other}"),
        };
        if run["error"].is_null() {
            runs += 1;
        } else {
            refused += 1;
        }
        steps += n;
        for step in run["steps"].as_array().expect("steps") {
            if let Some(line) = step["published"]["line"].as_str() {
                lines.insert(line.split(' ').next().unwrap_or("").to_string());
            }
        }
    }
    // The vectors reach what they claim to: every state, a wrap of the heartbeat,
    // and each refusal.
    assert!(runs >= 80 && steps >= 5000, "{runs} runs, {steps} steps");
    assert!(refused >= 5, "{refused} refusals");
    for state in ["OPEN", "WAIT", "STOP"] {
        assert!(lines.contains(state), "no {state} line in the vectors");
    }
    assert!(
        counters.windows(2).any(|w| w == [255, 0]),
        "the heartbeat never wraps"
    );
}

//! The Python's answers (gen_vectors.py, from aav_camera_stream's modules) replayed: the
//! same command lines, choices, frames, rates, delays, stamps, time codes, lenses and
//! undistorted pictures. Floats to 1e-12 (relative), not bit for bit.

use std::collections::BTreeMap;
use std::sync::OnceLock;
use std::time::Duration;

use serde_json::Value;
use stream_core::decoders::{self, Decoder, Env, Platform, Runner, Stream};
use stream_core::latest::{stamp_ns, Delays, Latest, Rate};
use stream_core::lens::{self, Lens, Model, Undistorter};
use stream_core::models::Detector;
use stream_core::timecode::{self, Frame};

fn vectors() -> &'static Value {
    static V: OnceLock<Value> = OnceLock::new();
    V.get_or_init(|| {
        let text = include_str!("vectors.json");
        serde_json::from_str(text).expect("vectors.json is JSON")
    })
}

fn section(a: &str, b: &str) -> &'static [Value] {
    vectors()[a][b].as_array().expect("a list")
}

fn s(v: &Value) -> &str {
    v.as_str().expect("a string")
}

fn f(v: &Value) -> f64 {
    v.as_f64().expect("a number")
}

fn u(v: &Value) -> u64 {
    v.as_u64().expect("a whole number")
}

fn strings(v: &Value) -> Vec<String> {
    v.as_array()
        .expect("a list")
        .iter()
        .map(|x| s(x).to_string())
        .collect()
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array().expect("a list").iter().map(f).collect()
}

fn close(a: f64, b: f64) -> bool {
    a == b || (a - b).abs() <= 1e-12 * a.abs().max(b.abs()).max(1.0)
}

fn all_close(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| close(*x, *y))
}

fn bytes(v: &Value) -> Vec<u8> {
    // base64, without a crate: the standard alphabet, padded.
    const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = Vec::new();
    let mut acc = 0u32;
    let mut bits = 0;
    for c in s(v).bytes().filter(|&c| c != b'=') {
        let i = ALPHABET.iter().position(|&a| a == c).expect("base64") as u32;
        acc = (acc << 6) | i;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
    }
    out
}

/// The Python's error message, when it raised ValueError (not KeyError).
fn python_error(case: &Value) -> Option<&str> {
    case.get("error").map(s)
}

#[test]
fn check_names_the_same_choices() {
    for case in section("decoders", "check") {
        let got = decoders::check(s(&case["decoder"]), s(&case["transport"]));
        match python_error(case) {
            Some(e) => assert_eq!(got.err().map(|e| e.to_string()).as_deref(), Some(e)),
            None => assert!(got.is_ok(), "{case}"),
        }
    }
}

#[test]
fn every_command_line_is_the_pythons() {
    let cases = section("decoders", "commands");
    for case in cases {
        let got = decoders::check(s(&case["decoder"]), s(&case["transport"])).and_then(
            |(decoder, transport)| {
                let stream = Stream {
                    url: s(&case["url"]).to_string(),
                    width: u(&case["width"]) as u32,
                    height: u(&case["height"]) as u32,
                    transport,
                };
                decoders::command(
                    decoder,
                    &stream,
                    s(&case["program"]),
                    s(&case["pipeline"]),
                    s(&case["hwaccel"]),
                )
            },
        );
        match python_error(case) {
            Some(e) if !e.starts_with("KeyError") => {
                assert_eq!(
                    got.err().map(|e| e.to_string()).as_deref(),
                    Some(e),
                    "{case}"
                )
            }
            Some(_) => assert!(got.is_err(), "{case}"),
            None => assert_eq!(got.ok(), Some(strings(&case["ok"])), "{case}"),
        }
    }
    assert!(cases.len() >= 300);
}

#[test]
fn shlex_splits_as_pythons() {
    for case in section("decoders", "shlex") {
        let got = decoders::shlex_split(s(&case["text"]));
        match python_error(case) {
            Some(e) => assert_eq!(
                got.err().map(|e| e.to_string()).as_deref(),
                Some(e),
                "{case}"
            ),
            None => assert_eq!(got.ok(), Some(strings(&case["ok"])), "{case}"),
        }
    }
}

#[test]
fn choose_picks_the_same_decoder_and_warns_alike() {
    for case in section("decoders", "choose") {
        let decoder: Decoder = s(&case["decoder"]).parse().expect("a decoder");
        let platform = if s(&case["platform"]) == "darwin" {
            Platform::Mac
        } else {
            Platform::Other
        };
        let (mac_why, soft_why) = (s(&case["mac_why"]), s(&case["soft_why"]));
        let mut asked: Vec<Vec<String>> = Vec::new();
        let choice = decoders::choose(decoder, s(&case["program"]), "", platform, |gst, d| {
            asked.push(vec![gst.to_string(), d.name().to_string()]);
            let why = if d == Decoder::GstMac {
                mac_why
            } else {
                soft_why
            };
            if why.is_empty() {
                Ok(())
            } else {
                Err(why.to_string())
            }
        });
        let expected: Vec<Vec<String>> = case["asked"]
            .as_array()
            .expect("a list")
            .iter()
            .map(strings)
            .collect();
        assert_eq!(asked, expected, "{case}");
        assert_eq!(choice.decoder.name(), s(&case["chosen"]), "{case}");
        assert_eq!(choice.program, s(&case["chosen_program"]), "{case}");
        assert_eq!(choice.warning, s(&case["warning"]), "{case}");
    }
}

/// A runner with scripted answers, as the Python's fake subprocess.run.
struct Scripted {
    missing: String,
    tries: Vec<bool>,
    calls: Vec<Vec<String>>,
}

impl Runner for Scripted {
    fn run(&mut self, argv: &[String], _env: &Env) -> std::io::Result<bool> {
        self.calls.push(argv.to_vec());
        if argv[1] == "--exists" {
            return Ok(argv[2] != self.missing);
        }
        Ok(self.tries.remove(0))
    }
}

#[test]
fn gst_works_asks_the_same_and_says_why_alike() {
    for case in section("decoders", "gst_works") {
        let decoder: Decoder = s(&case["decoder"]).parse().expect("a decoder");
        let mut runner = Scripted {
            missing: s(&case["missing"]).to_string(),
            tries: case["vtdec_runs"]
                .as_array()
                .expect("a list")
                .iter()
                .map(|b| b.as_bool().expect("true or false"))
                .collect(),
            calls: Vec::new(),
        };
        let why = decoders::gst_works("/x/bin/gst-launch-1.0", decoder, &Env::new(), &mut runner);
        let calls: Vec<Vec<String>> = case["calls"]
            .as_array()
            .expect("a list")
            .iter()
            .map(strings)
            .collect();
        assert_eq!(runner.calls, calls, "{case}");
        assert_eq!(why.err().unwrap_or_default(), s(&case["why"]), "{case}");
    }
}

fn env_of(v: &Value) -> Env {
    v.as_object()
        .expect("a map")
        .iter()
        .map(|(k, v)| (k.clone(), s(v).to_string()))
        .collect::<BTreeMap<_, _>>()
}

#[test]
fn child_env_drops_pixis_paths_alike() {
    for case in section("decoders", "child_env") {
        let got = decoders::child_env(
            &env_of(&case["environ"]),
            s(&case["program"]),
            s(&case["prefix"]),
        );
        assert_eq!(got, env_of(&case["env"]), "{case}");
    }
}

#[test]
fn detectors_are_named_alike() {
    for case in section("decoders", "detectors") {
        let got = s(&case["name"]).parse::<Detector>();
        match python_error(case) {
            Some(e) => assert_eq!(got.err().map(|e| e.to_string()).as_deref(), Some(e)),
            None => assert_eq!(got.ok().map(Detector::name), Some(s(&case["ok"]))),
        }
    }
}

#[test]
fn latest_takes_and_drops_alike() {
    for case in section("latest", "latest") {
        let slot = Latest::default();
        for (op, want) in case["ops"]
            .as_array()
            .expect("ops")
            .iter()
            .zip(case["results"].as_array().expect("results"))
        {
            let op = s(op);
            if op == "take" {
                assert_eq!(slot.take(Duration::ZERO), want.as_i64(), "{case}");
            } else {
                let n: i64 = op[4..].parse().expect("a number");
                slot.put(n);
            }
        }
        assert_eq!(slot.dropped(), u(&case["dropped"]), "{case}");
        assert_eq!(slot.put_count(), u(&case["put_count"]), "{case}");
    }
}

#[test]
fn rates_are_the_pythons() {
    for case in section("latest", "rate") {
        let mut rate = Rate::new(f(&case["window"]));
        for (event, want) in case["events"]
            .as_array()
            .expect("events")
            .iter()
            .zip(case["results"].as_array().expect("results"))
        {
            let (kind, t) = (s(&event[0]), f(&event[1]));
            if kind == "tick" {
                rate.tick(t);
            } else {
                let hz = rate.hz(t);
                assert!(close(hz, f(want)), "{hz} vs {want} in {case}");
            }
        }
    }
}

fn summary_matches(got: Option<stream_core::latest::Summary>, want: &Value) -> bool {
    match (got, want.is_null()) {
        (None, true) => true,
        (Some(g), false) => {
            g.n as u64 == u(&want["n"])
                && close(g.median, f(&want["median"]))
                && close(g.p95, f(&want["p95"]))
                && close(g.max, f(&want["max"]))
        }
        _ => false,
    }
}

#[test]
fn delays_summarise_alike() {
    for case in section("latest", "delays") {
        let mut delays = Delays::new(u(&case["keep"]) as usize);
        assert!(summary_matches(delays.summary(), &case["empty"]));
        for (v, want) in floats(&case["values"])
            .into_iter()
            .zip(case["summaries"].as_array().expect("summaries"))
        {
            delays.add(v);
            assert!(summary_matches(delays.summary(), want), "{want} in {case}");
        }
    }
}

#[test]
fn stamps_are_the_pythons() {
    for case in section("latest", "stamp") {
        let i = |k: &str| case[k].as_i64().expect("ns");
        assert_eq!(
            stamp_ns(i("arrival"), i("clock"), i("wall"), f(&case["delay"])),
            i("stamp"),
            "{case}"
        );
    }
}

#[test]
fn time_codes_are_the_pythons() {
    for case in section("timecode", "crc8") {
        assert_eq!(
            u64::from(timecode::crc8(u(&case["value"]) as u32)),
            u(&case["crc"])
        );
    }
    for case in section("timecode", "bits") {
        let want: Vec<bool> = case["bits"]
            .as_array()
            .expect("bits")
            .iter()
            .map(|b| u(b) == 1)
            .collect();
        assert_eq!(timecode::bits_of(u(&case["ms"])), want);
    }
    for case in section("timecode", "read") {
        let data = bytes(&case["picture"]);
        let frame = Frame {
            data: &data,
            width: u(&case["width"]) as usize,
            height: u(&case["height"]) as usize,
            channels: u(&case["channels"]) as usize,
        };
        assert_eq!(
            timecode::read(&frame).map(u64::from),
            case["read"].as_u64(),
            "{}",
            case["ms"]
        );
    }
    for case in section("timecode", "now_ms") {
        assert_eq!(timecode::now_ms(f(&case["wall"])), u(&case["ms"]), "{case}");
    }
    for case in section("timecode", "age") {
        let age = timecode::age_s(u(&case["sent"]) as u32, f(&case["wall"]));
        assert!(close(age, f(&case["age"])), "{age} in {case}");
    }
}

/// The Python's `vars(Lens)`.
fn lens_of(v: &Value) -> Lens {
    Lens {
        width: u(&v["width"]) as u32,
        height: u(&v["height"]) as u32,
        k: floats(&v["k"]),
        d: floats(&v["d"]),
        model: Model::parse(s(&v["model"])),
    }
}

fn same_lens(got: &Lens, want: &Value) -> bool {
    let want = lens_of(want);
    got.width == want.width
        && got.height == want.height
        && all_close(&got.k, &want.k)
        && all_close(&got.d, &want.d)
        && got.model == want.model
}

#[test]
fn calibration_files_read_alike() {
    for case in section("lens", "ini") {
        match (lens::parse_ini(s(&case["text"])), case.get("ok")) {
            (Ok(got), Some(want)) => assert!(same_lens(&got, want), "{}", case["name"]),
            (Err(_), None) => {}
            (got, _) => panic!("{}: {got:?}", case["name"]),
        }
    }
    for case in section("lens", "yaml") {
        match (lens::parse_yaml(s(&case["text"])), case.get("ok")) {
            (Ok(got), Some(want)) => assert!(same_lens(&got, want), "{case}"),
            (Err(_), None) => {}
            (got, _) => panic!("{case}: {got:?}"),
        }
    }
}

#[test]
fn lenses_scale_alike() {
    for case in section("lens", "scaled") {
        let got = lens::scaled(
            &lens_of(&case["lens"]),
            u(&case["width"]) as u32,
            u(&case["height"]) as u32,
        );
        match (got, case.get("ok")) {
            (Ok(got), Some(want)) => assert!(same_lens(&got, want), "{case}"),
            (Err(e), None) => assert_eq!(e.to_string(), s(&case["error"])),
            (got, _) => panic!("{case}: {got:?}"),
        }
    }
    for case in section("lens", "identity") {
        let got = lens::identity(
            u(&case["width"]) as u32,
            u(&case["height"]) as u32,
            f(&case["focal"]),
        );
        assert!(same_lens(&got, &case["lens"]));
        assert!(all_close(
            &lens::projection(&got),
            &floats(&case["projection"])
        ));
    }
}

#[test]
fn the_new_camera_matrix_is_opencvs() {
    for case in section("lens", "new_k") {
        let lens = lens_of(&case["lens"]);
        let got = lens::optimal_new_camera_matrix(&lens, f(&case["alpha"])).expect("a matrix");
        let want = floats(&case["k"]);
        // OpenCV's undistortPoints works in doubles, and so does this: 1e-12 holds
        let near = got
            .iter()
            .zip(&want)
            .all(|(a, b)| (a - b).abs() <= 1e-12 * a.abs().max(b.abs()).max(1.0));
        assert!(near, "{got:?} vs {want:?} for {case}");
    }
}

#[test]
fn undistorted_pictures_are_opencvs() {
    let mut worst = 0;
    let mut differ = 0usize;
    let mut total = 0usize;
    for case in section("lens", "undistort") {
        let lens = lens_of(&case["lens"]);
        let (w, h) = (lens.width as usize, lens.height as usize);
        // gen_vectors.py's pattern(): ramps plus a checkerboard.
        let picture: Vec<u8> = (0..h)
            .flat_map(|y| {
                (0..w).flat_map(move |x| {
                    (0..3).map(move |c| {
                        ((x * 7 + y * 5 + c * 60) % 200 + 40 * (((x / 6) + (y / 6)) % 2)) as u8
                    })
                })
            })
            .collect();
        let undistorter = Undistorter::new(&lens, f(&case["alpha"])).expect("a lens");
        let got = undistorter.apply(&picture).expect("the size");
        let want = bytes(&case["picture"]);
        assert_eq!(got.len(), want.len());
        for (a, b) in got.iter().zip(&want) {
            let d = (i32::from(*a) - i32::from(*b)).abs();
            worst = worst.max(d);
            differ += usize::from(d != 0);
        }
        total += got.len();
    }
    // The map is OpenCV's formula in doubles; its SIMD loop sums in another order, so a
    // place can round to the next 1/32 px. Nearly all grey levels are the same.
    eprintln!("undistorted: {differ} of {total} grey levels differ, by {worst} at most");
    assert!(worst <= 2, "worst {worst}");
    assert!(
        differ * 1000 <= total,
        "{differ} of {total} grey levels differ"
    );
}

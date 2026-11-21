//! The Python's answers: `gen_vectors.py` ran `aav_things`' modules on these runs.
//!
//! Each run plays the node's pipeline cloud by cloud (the saved map's walls dropped, the
//! groups, to odom, the tracker, the radar), then the list it would publish. The ids,
//! seen counts, flags and group counts must be the same; places, speeds, outlines and
//! the rest within [`TOL`].

use std::{fs, path::Path};

use serde_json::Value;
use things_core::{
    background::{new_only, Background},
    group,
    naming::{best_label, project, thing_box, Axes, Camera, CameraMount, Lens, Namer, Seen},
    radar::{clusters, RadarPoint},
    to_frame, GroupShape, Pose2, Track, Tracker, TrackerShape,
};

/// The largest difference allowed in a number (m, m/s, or none): the two languages'
/// matrix sums may round the last bit differently.
const TOL: f64 = 1e-9;

fn number(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "inf" => f64::INFINITY,
        Value::String(s) if s == "-inf" => f64::NEG_INFINITY,
        Value::String(_) => f64::NAN,
        v => v.as_f64().expect("a number"),
    }
}

fn numbers(v: &Value) -> Vec<f64> {
    v.as_array().expect("a list").iter().map(number).collect()
}

fn pose(v: &Value) -> Pose2 {
    let n = numbers(v);
    Pose2 {
        x: n[0],
        y: n[1],
        yaw: n[2],
    }
}

/// float32 values in the file, as the cubes' cloud carries them.
fn cubes(v: &Value) -> Vec<[f64; 3]> {
    numbers(v)
        .iter()
        .map(|&x| f64::from(x as f32))
        .collect::<Vec<_>>()
        .as_chunks::<3>()
        .0
        .to_vec()
}

thread_local! {
    /// The largest difference seen, over max(1, |the Python's|).
    static LARGEST: std::cell::Cell<f64> = const { std::cell::Cell::new(0.0) };
}

fn same(what: &str, got: f64, want: f64) {
    let scale = want.abs().max(1.0);
    if got.is_finite() && want.is_finite() {
        LARGEST.set(LARGEST.get().max((got - want).abs() / scale));
    }
    assert!(
        (got - want).abs() <= TOL * scale || (got.is_nan() && want.is_nan()),
        "{what}: {got} against the Python's {want}"
    );
}

fn background(spec: &Value) -> Background {
    let (w, h) = (
        spec["width"].as_u64().expect("width") as usize,
        spec["height"].as_u64().expect("height") as usize,
    );
    let mut data = vec![0i8; w * h];
    for i in spec["walls"].as_array().expect("walls") {
        data[i.as_u64().expect("an index") as usize] = 100;
    }
    let origin = numbers(&spec["origin"]);
    Background::from_grid(
        &data,
        w,
        h,
        number(&spec["resolution"]),
        [origin[0], origin[1]],
        number(&spec["slack"]),
    )
    .expect("a grid")
}

fn vectors() -> Value {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/vectors.json");
    serde_json::from_str(&fs::read_to_string(path).expect("vectors.json")).expect("JSON")
}

/// A thing's kind by its outline, for the per-kind runs: gen_vectors.py's rule.
fn kind_by_shape(t: &Track) -> String {
    let size = t.length.max(t.width);
    if size > 2.5 {
        "car"
    } else if size <= 0.8 && t.height > 1.2 {
        "person"
    } else {
        "unknown"
    }
    .to_string()
}

#[test]
fn the_same_things_as_the_python() {
    let all = vectors();
    let runs = all["runs"].as_array().expect("runs");
    let (mut clouds, mut listed) = (0, 0);
    for run in runs {
        let name = run["name"].as_str().expect("name");
        // TODO 217: the per-kind runs set per_kind (and refind_gap); the kinds by shape.
        let given = run.get("tracker").filter(|t| t.is_object());
        let d = TrackerShape::default();
        let shape = TrackerShape {
            per_kind: given
                .and_then(|t| t.get("per_kind"))
                .and_then(Value::as_bool)
                .unwrap_or(d.per_kind),
            refind_gap: given
                .and_then(|t| t.get("refind_gap"))
                .and_then(Value::as_f64)
                .unwrap_or(d.refind_gap),
            ..d
        };
        let kinds = run.get("kinds").and_then(Value::as_bool).unwrap_or(false);
        let mut tracker = Tracker::new(shape);
        let bg = run.get("map").filter(|m| !m.is_null()).map(background);
        let below = run.get("below").and_then(Value::as_f64);
        let frames = run["frames"].as_array().expect("frames");
        let expected = run["expected"].as_array().expect("expected");
        assert_eq!(frames.len(), expected.len());
        for (k, (frame, want)) in frames.iter().zip(expected).enumerate() {
            let at = format!("{name}, cloud {k}");
            let mut points = cubes(&frame["cubes"]);
            if let (Some(bg), Some(map_pose)) =
                (&bg, frame.get("map_pose").filter(|p| !p.is_null()))
            {
                if !points.is_empty() {
                    points = new_only(&points, pose(map_pose), bg, below);
                }
            }
            assert_eq!(
                points.len() as u64,
                want["kept"].as_u64().unwrap(),
                "{at}: kept"
            );
            let odom = pose(&frame["pose"]);
            let groups = to_frame(&group(&points, &GroupShape::default()), odom);
            let want_groups = want["groups"].as_array().expect("groups");
            assert_eq!(groups.len(), want_groups.len(), "{at}: groups");
            for (g, w) in groups.iter().zip(want_groups) {
                let w = numbers(w);
                for (got, want) in [g.x, g.y, g.length, g.width, g.height].iter().zip(&w) {
                    same(&format!("{at}: a group"), *got, *want);
                }
                assert_eq!(g.cubes as f64, w[5], "{at}: a group's cubes");
            }
            let car = [odom.x, odom.y];
            tracker.step(number(&frame["t"]), &groups, car);
            if kinds {
                tracker.rename(kind_by_shape);
            }
            if let Some(radar) = frame.get("radar").filter(|r| !r.is_null()) {
                let rows: Vec<RadarPoint> = radar["rows"]
                    .as_array()
                    .expect("rows")
                    .iter()
                    .map(|r| {
                        let n = numbers(r);
                        RadarPoint {
                            x: n[0],
                            y: n[1],
                            velocity: n[2],
                            cluster: n[3],
                        }
                    })
                    .collect();
                let v = numbers(&radar["car_velocity"]);
                let found = clusters(&rows, pose(&radar["pose"]), [v[0], v[1]]);
                tracker.radar(number(&radar["t"]), &found);
            }
            let now = number(&frame["publish"]);
            tracker.expire(now, car);
            let things = tracker.at(now);
            let want_things = want["things"].as_array().expect("things");
            assert_eq!(
                things.iter().map(|t| t.id).collect::<Vec<_>>(),
                want_things
                    .iter()
                    .map(|t| t["id"].as_u64().unwrap() as u32)
                    .collect::<Vec<_>>(),
                "{at}: the things' ids"
            );
            for (t, w) in things.iter().zip(want_things) {
                let what = format!("{at}: thing {}", t.id);
                for (i, want) in numbers(&w["x"]).iter().enumerate() {
                    same(&what, t.x[i], *want);
                }
                same(&what, t.length, number(&w["length"]));
                same(&what, t.width, number(&w["width"]));
                same(&what, t.height, number(&w["height"]));
                same(&what, t.position_sigma(), number(&w["sigma"]));
                same(
                    &what,
                    t.confidence(now, &tracker.shape),
                    number(&w["confidence"]),
                );
                same(&what, t.last_seen, number(&w["last_seen"]));
                assert_eq!(u64::from(t.seen), w["seen"].as_u64().unwrap(), "{what}");
                assert_eq!(t.radar, w["radar"].as_bool().unwrap(), "{what}: radar");
                assert_eq!(t.coasting, w["coasting"].as_bool().unwrap(), "{what}");
                listed += 1;
            }
            clouds += 1;
        }
    }
    assert!(
        clouds >= 180 && listed > 1000,
        "{clouds} clouds, {listed} things"
    );
    println!(
        "{clouds} clouds, {listed} things listed, all the Python's; the largest difference {:e}",
        LARGEST.get()
    );
}

#[test]
fn the_same_names_as_the_python() {
    let all = vectors();
    for (i, case) in all["names"].as_array().expect("names").iter().enumerate() {
        let m = case["mount"].as_array().expect("mount");
        let p = numbers(&m[0]);
        let rows: Vec<Vec<f64>> = m[1].as_array().unwrap().iter().map(numbers).collect();
        let mount = CameraMount {
            position: [p[0], p[1], p[2]],
            rotation: [0, 1, 2].map(|r| [rows[r][0], rows[r][1], rows[r][2]]),
            axes: if m[2].as_bool().unwrap() {
                Axes::Optical
            } else {
                Axes::Body
            },
        };
        let l = numbers(&case["lens"]);
        let lens = Lens {
            fx: l[0],
            fy: l[1],
            cx: l[2],
            cy: l[3],
            width: 640,
            height: 480,
        };
        let t = numbers(&case["thing"]);
        let point = project(&lens, &mount, [t[0], t[1], t[4]]);
        match (&point, case["point"].is_null()) {
            (None, true) => {}
            (Some(got), false) => {
                for (g, w) in got.iter().zip(numbers(&case["point"])) {
                    same(&format!("name {i}: point"), *g, w);
                }
            }
            _ => panic!("name {i}: point {point:?}, the Python's {}", case["point"]),
        }
        let area = thing_box(&lens, &mount, t[0], t[1], t[2], t[3], t[4]);
        match (&area, case["box"].is_null()) {
            (None, true) => {}
            (Some(got), false) => {
                for (g, w) in got.iter().zip(numbers(&case["box"])) {
                    same(&format!("name {i}: box"), *g, w);
                }
            }
            _ => panic!("name {i}: box {area:?}, the Python's {}", case["box"]),
        }
        let seen: Vec<Seen> = case["seen"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| {
                let s = s.as_array().unwrap();
                Seen {
                    label: s[0].as_str().unwrap().to_string(),
                    score: 0.9,
                    area: [1, 2, 3, 4].map(|k| number(&s[k])),
                }
            })
            .collect();
        let strict = area.and_then(|a| best_label(&a, &seen, 0.3, None));
        assert_eq!(
            strict,
            case["strict"].as_str(),
            "name {i}: the strict label"
        );
        let mut namer = Namer::default();
        namer.set_camera(
            "cam",
            Camera {
                lens,
                mount,
                seen,
                stamp: 10.0,
            },
        );
        let label = namer.name(i as u32, t[0], t[1], t[2], t[3], t[4], 10.1);
        assert_eq!(Some(label.as_str()), case["label"].as_str(), "name {i}");
    }
}

#[test]
fn the_same_background_as_the_python() {
    let all = vectors();
    let bg = background(&all["background"]["map"]);
    for (i, case) in all["background"]["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .enumerate()
    {
        let points: Vec<[f64; 3]> = numbers(&case["cubes"]).as_chunks::<3>().0.to_vec();
        let kept = new_only(&points, pose(&case["pose"]), &bg, case["below"].as_f64());
        let want: Vec<[f64; 3]> = numbers(&case["kept"]).as_chunks::<3>().0.to_vec();
        assert_eq!(kept, want, "background case {i}");
    }
}

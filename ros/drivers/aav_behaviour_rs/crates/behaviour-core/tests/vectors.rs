//! The Python's answers (`vectors.json`, from `gen_vectors.py`): behaviour-core must agree,
//! within 1e-9, every state, reason and warning the same text.
//!
//! `cargo test -p behaviour-core --test vectors -- --nocapture` prints the largest
//! difference of each part.

use std::collections::BTreeSet;
use std::fs;

use behaviour_core::ego_core::stop_law::StopCurve;
use behaviour_core::fence::Polygon;
use behaviour_core::lanes::{from_yaml, lane_line, with_edges, SMOOTH};
use behaviour_core::machine::{
    diagnostic, nav2_cap, Behaviour, Lock, Settings, Stop, StopReason, World,
};
use behaviour_core::paths::{
    along_until, kerb_ahead, lane_ahead, moving_ahead, moving_near, path_left, Crossing, MovingFor,
    Nearby, Point, Seen, MOVING, REACH,
};
use behaviour_core::road::{build, capped, BuildOptions, Strip};
use behaviour_core::stop_lines::{StopLineSettings, StopLines};
use serde_json::Value;

const EXACT: f64 = 1e-9;

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

fn fs_(v: &Value) -> Vec<f64> {
    v.as_array().expect("array").iter().map(f).collect()
}

fn points(v: &Value) -> Vec<Point> {
    v.as_array()
        .expect("points")
        .iter()
        .map(|p| (f(&p[0]), f(&p[1])))
        .collect()
}

fn text(v: &Value) -> &str {
    v.as_str().expect("text")
}

/// The largest difference seen, and where.
#[derive(Default)]
struct Worst {
    diff: f64,
    count: usize,
    at: String,
}

impl Worst {
    fn check(&mut self, got: f64, want: f64, at: impl FnOnce() -> String) {
        self.count += 1;
        let diff = if got.is_nan() && want.is_nan() || got == want {
            0.0
        } else {
            (got - want).abs()
        };
        assert!(
            diff <= EXACT,
            "{}: got {got}, the Python {want} (off by {diff})",
            at()
        );
        if diff > self.diff {
            self.diff = diff;
            self.at = at();
        }
    }

    fn all(&mut self, got: &[f64], want: &[f64], at: &str) {
        assert_eq!(got.len(), want.len(), "{at}: lengths");
        for (i, (g, w)) in got.iter().zip(want).enumerate() {
            self.check(*g, *w, || format!("{at}[{i}]"));
        }
    }

    fn report(&self, what: &str) {
        println!(
            "{what}: {} numbers, largest difference {:e} {}",
            self.count, self.diff, self.at
        );
    }
}

fn curves(v: &Value) -> impl Fn(&str) -> StopCurve + '_ {
    move |name: &str| {
        let c = fs_(&v["curves"][name]);
        StopCurve::new(c[0], c[1], c[2]).expect("curve")
    }
}

fn reason(v: &Value) -> StopReason {
    text(v).parse().expect("reason")
}

fn stops(v: &Value) -> Vec<Stop> {
    v.as_array()
        .expect("stops")
        .iter()
        .map(|s| Stop::new(reason(&s[0]), f(&s[1])))
        .collect()
}

#[test]
fn the_machine_gives_the_pythons_every_tick() {
    let v = vectors();
    let curve = curves(&v);
    let mut worst = Worst::default();
    let mut ticks = 0;
    for run in v["runs"].as_array().expect("runs") {
        let s = fs_(&v["settings"][text(&run["settings"])]);
        let settings = Settings {
            base: s[0],
            floor: s[1],
            accel: s[2],
            fall: s[3],
            still: s[4],
            ask_after: s[5],
            odom_stale: s[6],
            sign_lost: s[7],
            passed: s[8],
            release_hold: s[9],
        };
        let mut b = Behaviour::new(curve(text(&run["curve"])), settings).expect("settings");
        let name = format!("{} / {}", text(&run["settings"]), text(&run["curve"]));
        for (i, t) in run["ticks"].as_array().expect("ticks").iter().enumerate() {
            let w = &t["world"];
            let world = World {
                now: f(&w["now"]),
                speed: (!w["speed"].is_null()).then(|| f(&w["speed"])),
                odom_age: f(&w["odom_age"]),
                locks: w["locks"]
                    .as_array()
                    .expect("locks")
                    .iter()
                    .map(|l| text(l).parse::<Lock>().expect("lock"))
                    .collect::<BTreeSet<_>>(),
                near: w["near"].as_bool().expect("near"),
                stops: stops(&w["stops"]),
                sign_age: f(&w["sign_age"]),
                goal_active: w["goal_active"].as_bool().expect("goal"),
                goal_aborted: w["goal_aborted"].as_bool().expect("aborted"),
                moving: w["moving"].as_bool().expect("moving"),
            };
            let d = b.step(&world);
            let at = || format!("{name}, tick {i}");
            assert_eq!(d.state.as_str(), text(&t["state"]), "{}", at());
            assert_eq!(d.reason.to_string(), text(&t["reason"]), "{}", at());
            assert_eq!(
                b.holding(),
                t["holding"].as_bool().expect("holding"),
                "{}",
                at()
            );
            let warnings: Vec<&str> = t["warnings"]
                .as_array()
                .expect("warnings")
                .iter()
                .map(text)
                .collect();
            assert_eq!(d.warnings, warnings, "{}", at());
            worst.check(d.cap, f(&t["cap"]), at);
            worst.check(d.left, f(&t["left"]), at);
            for (key, value) in diagnostic(&d) {
                assert_eq!(value, text(&t["diagnostic"][key]), "{}: {key}", at());
            }
            ticks += 1;
        }
    }
    assert!(ticks > 1000);
    worst.report(&format!("the machine, {ticks} ticks: cap and left"));
}

#[test]
fn the_cap_for_a_stop_is_the_pythons() {
    let v = vectors();
    let curve = curves(&v);
    let mut worst = Worst::default();
    for (i, c) in v["caps"].as_array().expect("caps").iter().enumerate() {
        let got = nav2_cap(
            f(&c["left"]),
            &curve(text(&c["curve"])),
            f(&c["base"]),
            f(&c["floor"]),
        );
        worst.check(got, f(&c["cap"]), || format!("cap {i}"));
    }
    worst.report("nav2_cap");
}

#[test]
fn paths_and_the_fence_are_the_pythons() {
    let v = vectors();
    let p = &v["paths"];
    let fence = Polygon::new(points(&p["fence"])).expect("fence");
    let mut worst = Worst::default();
    for d in p["depths"].as_array().expect("depths") {
        let (x, y) = (f(&d[0]), f(&d[1]));
        worst.check(fence.depth(x, y), f(&d[2]), || format!("depth at {x}, {y}"));
    }
    for (i, c) in p["cases"].as_array().expect("cases").iter().enumerate() {
        let pts = points(&c["points"]);
        let (x, y, depth) = (f(&c["x"]), f(&c["y"]), f(&c["depth"]));
        let (left, off) = path_left(&pts, x, y);
        worst.check(left, f(&c["left"][0]), || format!("path {i}: left"));
        worst.check(off, f(&c["left"][1]), || format!("path {i}: off"));
        let until = along_until(&pts, x, y, |px, py| fence.depth(px, py) < depth);
        worst.check(until, f(&c["until"]), || format!("path {i}: until"));
    }
    worst.report("path_left, along_until, the fence's depth");
}

#[test]
fn the_lane_ahead_and_moving_things_are_the_pythons() {
    let v = vectors();
    let mut worst = Worst::default();
    for (i, s) in v["scans"].as_array().expect("scans").iter().enumerate() {
        let got = lane_ahead(
            &fs_(&s["ranges"]),
            f(&s["angle_min"]),
            f(&s["angle_increment"]),
            f(&s["front"]),
            f(&s["half_width"]),
            REACH,
        );
        worst.check(got, f(&s["ahead"]), || format!("scan {i}"));
    }
    for (i, m) in v["movers"].as_array().expect("movers").iter().enumerate() {
        let car = fs_(&m["car"]);
        let things = seen_things(&m["things"]);
        let crossing = (!m["crossing"].is_null()).then(|| {
            let c = fs_(&m["crossing"]);
            Crossing {
                across: c[0],
                slack: c[1],
                horizon: c[2],
            }
        });
        let got = moving_ahead(
            &things,
            (car[0], car[1], car[2]),
            1.8,
            0.6,
            REACH,
            MOVING,
            f(&m["speed"]),
            crossing,
        );
        worst.check(got, f(&m["ahead"]), || format!("movers {i}"));
    }
    worst.report("lane_ahead, moving_ahead");
}

#[test]
fn the_kerbs_ahead_are_the_pythons() {
    // TODO 219: the steps along the road ahead.
    let v = vectors();
    let mut worst = Worst::default();
    for (i, k) in v["kerbs"].as_array().expect("kerbs").iter().enumerate() {
        let got = kerb_ahead(
            &points(&k["steps"]),
            &points(&k["road"]),
            f(&k["front"]),
            f(&k["half_width"]),
            REACH,
        );
        worst.check(got, f(&k["ahead"]), || format!("kerbs {i}"));
    }
    worst.report("kerb_ahead");
}

fn seen_things(v: &Value) -> Vec<Seen> {
    v.as_array()
        .expect("things")
        .iter()
        .map(|t| {
            let t = fs_(t);
            Seen {
                x: t[0],
                y: t[1],
                vx: t[2],
                vy: t[3],
                length: t[4],
                width: t[5],
            }
        })
        .collect()
}

#[test]
fn which_things_have_moved_long_enough_is_the_pythons() {
    let v = vectors();
    let mut worst = Worst::default();
    let mut ticks = 0;
    for (r, run) in v["settles"].as_array().expect("settles").iter().enumerate() {
        let mut sure = MovingFor::new(f(&run["settle"]), MOVING);
        for (i, t) in run["ticks"].as_array().expect("ticks").iter().enumerate() {
            let things: Vec<(u32, bool, Seen)> = t["things"]
                .as_array()
                .expect("things")
                .iter()
                .map(|x| {
                    let id = u32::try_from(x[0].as_u64().expect("id")).expect("id");
                    let s = seen_things(&Value::Array(vec![x[2].clone()]))[0];
                    (id, x[1].as_bool().expect("coasting"), s)
                })
                .collect();
            let got = sure.update(f(&t["now"]), &things);
            let want = points(&t["sure"]);
            assert_eq!(got.len(), want.len(), "settles {r}, tick {i}");
            for (g, w) in got.iter().zip(&want) {
                worst.check(g.x, w.0, || format!("settles {r}, tick {i}"));
                worst.check(g.y, w.1, || format!("settles {r}, tick {i}"));
            }
            ticks += 1;
        }
    }
    assert!(ticks >= 2000);
    worst.report("MovingFor");
}

#[test]
fn moving_things_near_are_the_pythons() {
    let v = vectors();
    let mut worst = Worst::default();
    let cases = v["nears"].as_array().expect("nears");
    for (i, m) in cases.iter().enumerate() {
        let car = fs_(&m["car"]);
        let n = fs_(&m["nearby"]);
        let nearby = Nearby {
            radius: n[0],
            within: n[1],
            step: n[2],
            moving: n[3],
        };
        let got = moving_near(
            &seen_things(&m["things"]),
            (car[0], car[1], car[2]),
            f(&m["front"]),
            &points(&m["road"]),
            f(&m["half_width"]),
            nearby,
        );
        worst.check(got, f(&m["near"]), || format!("nears {i}"));
    }
    assert!(cases.len() >= 300);
    worst.report("moving_near");
}

fn check_strip(worst: &mut Worst, s: &Strip, b: &Value, at: &str) {
    worst.all(&s.x, &fs_(&b["x"]), &format!("{at} x"));
    worst.all(&s.y, &fs_(&b["y"]), &format!("{at} y"));
    worst.all(&s.heading, &fs_(&b["heading"]), &format!("{at} heading"));
}

#[test]
fn the_road_ahead_and_the_lane_map_are_the_pythons() {
    let v = vectors();
    let r = &v["roads"];
    let curve = curves(&v)("car");
    let lanes = from_yaml(text(&r["yaml"])).expect("lanes");
    let mut worst = Worst::default();
    for (i, b) in r["builds"].as_array().expect("builds").iter().enumerate() {
        let at = format!("build {i}");
        let car = fs_(&b["car"]);
        let o = BuildOptions {
            state: "APPROACH",
            reason: "sign",
            spacing: f(&b["spacing"]),
            length: f(&b["length"]),
        };
        let s = build(
            &points(&b["route"]),
            Some((car[0], car[1])),
            &stops(&b["stops"]),
            &curve,
            f(&b["base"]),
            &o,
        )
        .expect("build");
        check_strip(&mut worst, &s, b, &at);
        worst.all(
            &s.speed_limit,
            &fs_(&b["speed_limit"]),
            &format!("{at} limit"),
        );
        worst.check(s.end, f(&b["end"]), || format!("{at} end"));
        worst.all(
            &capped(s.clone(), f(&b["cap"])).speed_limit,
            &fs_(&b["capped"]),
            &format!("{at} capped"),
        );
        let want = b["on_line"].as_array().expect("on_line");
        assert_eq!(s.stops.len(), want.len(), "{at}: stops");
        for (got, want) in s.stops.iter().zip(want) {
            worst.check(got.along, f(&want[0]), || format!("{at} along"));
            assert_eq!(got.reason.as_str(), text(&want[1]), "{at}");
            assert_eq!(u64::from(got.kind.code()), want[2].as_u64().expect("kind"));
        }
    }
    let lines = r["lines"].as_array().expect("lines");
    let edges = r["edges"].as_array().expect("edges");
    for (i, (l, e)) in lines.iter().zip(edges).enumerate() {
        let at = format!("lane line {i}");
        let car = fs_(&l["car"]);
        let (spacing, length) = (f(&l["spacing"]), f(&l["length"]));
        let passing = l["passing"].as_bool().expect("passing");
        let line = lane_line(
            &lanes,
            &points(&l["route"]),
            (car[0], car[1]),
            spacing,
            length,
            SMOOTH,
            passing,
        )
        .expect("line");
        let want = points(&l["line"]);
        assert_eq!(line.len(), want.len(), "{at}: points");
        for (j, (g, w)) in line.iter().zip(&want).enumerate() {
            worst.check(g.0, w.0, || format!("{at}[{j}] x"));
            worst.check(g.1, w.1, || format!("{at}[{j}] y"));
        }
        let o = BuildOptions {
            spacing,
            length,
            ..BuildOptions::default()
        };
        let s = build(&line, Some((car[0], car[1])), &[], &curve, 1.0, &o).expect("build");
        let s = with_edges(s, &lanes, passing);
        let at = format!("edges {i}");
        check_strip(&mut worst, &s, e, &at);
        worst.all(
            &s.left_hard,
            &fs_(&e["left_hard"]),
            &format!("{at} left_hard"),
        );
        worst.all(
            &s.right_hard,
            &fs_(&e["right_hard"]),
            &format!("{at} right_hard"),
        );
        worst.all(
            &s.left_soft,
            &fs_(&e["left_soft"]),
            &format!("{at} left_soft"),
        );
        worst.all(
            &s.right_soft,
            &fs_(&e["right_soft"]),
            &format!("{at} right_soft"),
        );
    }
    for (i, a) in r["lane_at"].as_array().expect("lane_at").iter().enumerate() {
        let lane = lanes.lane_at(f(&a["x"]), f(&a["y"]), f(&a["heading"]));
        match (&lane, a["lane"].as_array()) {
            (None, None) => {}
            (Some(lane), Some(w)) => {
                let got = [
                    lane.x,
                    lane.y,
                    lane.heading,
                    lane.left_soft,
                    lane.right_soft,
                    lane.left_hard,
                    lane.right_hard,
                ];
                for (k, g) in got.iter().enumerate() {
                    worst.check(*g, f(&w[k]), || format!("lane_at {i}[{k}]"));
                }
                assert_eq!(lane.road, text(&w[7]), "lane_at {i}");
            }
            (got, want) => panic!("lane_at {i}: got {got:?}, the Python {want:?}"),
        }
    }
    worst.report("the road ahead, the lane line, its edges, lane_at");
}

#[test]
fn the_stop_lines_are_the_pythons() {
    let v = vectors();
    let mut worst = Worst::default();
    let mut ticks = 0;
    let mut stops = 0;
    for (r, run) in v["stop_lines"]
        .as_array()
        .expect("stop_lines")
        .iter()
        .enumerate()
    {
        let plan = points(&run["plan"]);
        let front = f(&run["front"]);
        let mut lines = StopLines::new(StopLineSettings::default());
        for (i, t) in run["ticks"].as_array().expect("ticks").iter().enumerate() {
            lines.set_lines(points(&t["lines"]));
            let car = (!t["car"].is_null()).then(|| (f(&t["car"][0]), f(&t["car"][1])));
            let speed = (!t["speed"].is_null()).then(|| f(&t["speed"]));
            let got = lines.update(f(&t["now"]), &plan, car, speed, front);
            let want = fs_(&t["lefts"]);
            assert_eq!(got.len(), want.len(), "stop lines {r}, tick {i}");
            worst.all(&got, &want, &format!("stop lines {r}, tick {i}"));
            stops += got.len();
            ticks += 1;
        }
    }
    assert!(ticks >= 6000 && stops > 500, "{ticks} {stops}");
    worst.report("StopLines");
}

//! The Python's answers (`vectors.json`, from `gen_vectors.py`): ego-core must agree.
//!
//! | Part | Within |
//! |---|---|
//! | ego's step, the lead, the pedal, the body, the stopping law, the road ahead, the stop point, the squiggly line, ego's commands for the trace | 1e-9 |
//! | the twin and the trace (spine-core's speed loop is f32) | 1e-3 |
//!
//! `cargo test -p ego-core --test vectors -- --nocapture` prints the largest difference of each.

use std::fs;

use ego_core::ackermann::{EgoModel, Limits};
use ego_core::car_model::{ActuatorConfig, CarState, SpeedLoopProfile};
use ego_core::dynamics::{describe, Body, Described, LogLine};
use ego_core::lag_lead::{LagLead, LeadLimits, LeadTimes};
use ego_core::look_ahead as la;
use ego_core::pedal::Pedal;
use ego_core::road_ahead::{self as ra, StopOnLine, Strip};
use ego_core::stop_ahead::stop_point;
use ego_core::stop_law::{StopCurve, INPUT_DELAY};
use ego_core::trace::{ego_commands, trace, Twin};
use serde_json::Value;

const EXACT: f64 = 1e-9;
const LOOP: f64 = 1e-3;

fn vectors() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors.json");
    serde_json::from_str(&fs::read_to_string(path).expect("vectors.json")).expect("json")
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("not a number: {v}"))
}

fn of(v: &Value) -> Option<f64> {
    v.as_f64()
}

fn fs_(v: &Value) -> Vec<f64> {
    v.as_array().expect("array").iter().map(f).collect()
}

fn points(v: &Value) -> Vec<(f64, f64)> {
    v.as_array()
        .expect("points")
        .iter()
        .map(|p| (f(&p[0]), f(&p[1])))
        .collect()
}

fn poses(v: &Value) -> Vec<(f64, f64, f64)> {
    v.as_array()
        .expect("poses")
        .iter()
        .map(|p| (f(&p[0]), f(&p[1]), f(&p[2])))
        .collect()
}

/// The largest difference seen, and where.
struct Worst {
    name: &'static str,
    tol: f64,
    max: f64,
    count: usize,
}

impl Worst {
    fn new(name: &'static str, tol: f64) -> Self {
        Worst {
            name,
            tol,
            max: 0.0,
            count: 0,
        }
    }

    fn check(&mut self, got: f64, want: f64, what: &str) {
        self.count += 1;
        if got == want || (got.is_nan() && want.is_nan()) {
            return;
        }
        let d = (got - want).abs();
        assert!(
            d <= self.tol,
            "{}: {what}: got {got}, want {want} (off {d:e})",
            self.name
        );
        self.max = self.max.max(d);
    }

    fn opt(&mut self, got: Option<f64>, want: Option<f64>, what: &str) {
        match (got, want) {
            (Some(g), Some(w)) => self.check(g, w, what),
            (None, None) => self.count += 1,
            _ => panic!("{}: {what}: got {got:?}, want {want:?}", self.name),
        }
    }

    fn report(&self) {
        println!(
            "{:<28} {:>6} numbers, largest difference {:e} (allowed {:e})",
            self.name, self.count, self.max, self.tol
        );
    }
}

fn limits(v: &Value) -> Limits {
    Limits {
        wheelbase: f(&v["wheelbase"]),
        max_steering_angle: f(&v["max_steering_angle"]),
        max_speed: f(&v["max_speed"]),
        max_reverse_speed: f(&v["max_reverse_speed"]),
        max_acceleration: f(&v["max_acceleration"]),
        max_deceleration: f(&v["max_deceleration"]),
        max_steering_rate: f(&v["max_steering_rate"]),
        min_speed_for_steering: f(&v["min_speed_for_steering"]),
        max_jerk: f(&v["max_jerk"]),
    }
}

fn body(v: &Value) -> Option<Body> {
    if v.is_null() {
        return None;
    }
    Some(Body {
        mass: f(&v["mass"]),
        payload: f(&v["payload"]),
        track: f(&v["track"]),
        cg_height: f(&v["cg_height"]),
        tyre_grip: f(&v["tyre_grip"]),
        full_throttle_accel: f(&v["full_throttle_accel"]),
        full_brake_decel: f(&v["full_brake_decel"]),
        max_lateral_accel: f(&v["max_lateral_accel"]),
    })
}

fn pedal(v: &Value) -> Option<Pedal> {
    (!v.is_null()).then(|| Pedal {
        let_off: f(&v["let_off"]),
        full: f(&v["full"]),
    })
}

fn lead(v: &Value) -> Option<LagLead> {
    if v.is_null() {
        return None;
    }
    let t = &v["times"];
    let l = &v["limits"];
    let mut lead = LagLead::new(
        LeadTimes {
            speed_up: f(&t["speed_up"]),
            slow_down: f(&t["slow_down"]),
            steer: f(&t["steer"]),
        },
        LeadLimits {
            filter_time: f(&l["filter_time"]),
            max_speed_lead: f(&l["max_speed_lead"]),
            max_steer_lead: f(&l["max_steer_lead"]),
        },
    );
    if let Some(fl) = v.get("filtered").filter(|x| !x.is_null()) {
        lead.set_filtered(Some((f(&fl[0]), f(&fl[1]))));
    }
    Some(lead)
}

fn cmd(v: &Value) -> Option<(f64, f64)> {
    (!v.is_null()).then(|| (f(&v[0]), f(&v[1])))
}

fn cfg(v: &Value) -> ActuatorConfig {
    let n = |k: &str| f(&v[k]);
    ActuatorConfig {
        max_accel: n("max_accel"),
        throttle_deadband: n("throttle_deadband"),
        throttle_lag: n("throttle_lag"),
        max_brake: n("max_brake"),
        brake_lag: n("brake_lag"),
        drag: n("drag"),
        steer_rate: n("steer_rate"),
        steer_lag: n("steer_lag"),
        steer_offset: n("steer_offset"),
        steer_deadband: n("steer_deadband"),
        max_steering: n("max_steering"),
        throttle_dead_time: n("throttle_dead_time"),
        brake_dead_time: n("brake_dead_time"),
        steer_dead_time: n("steer_dead_time"),
        wheelbase: n("wheelbase"),
        mass: n("mass"),
        payload: n("payload"),
    }
}

fn state(v: &Value) -> CarState {
    let n = |k: &str| f(&v[k]);
    CarState {
        x: n("x"),
        y: n("y"),
        yaw: n("yaw"),
        v: n("v"),
        steer: n("steer"),
        throttle_level: n("throttle_level"),
        brake_level: n("brake_level"),
        steer_motor: n("steer_motor"),
        steer_setpoint: n("steer_setpoint"),
        steer_linkage: n("steer_linkage"),
        throttle_delay: fs_(&v["throttle_delay"]).into(),
        brake_delay: fs_(&v["brake_delay"]).into(),
        steer_delay: fs_(&v["steer_delay"]).into(),
    }
}

fn profile(name: &str) -> Option<ego_core::car_model::SpeedLoopConfig> {
    match name {
        "none" => None,
        other => Some(
            other
                .parse::<SpeedLoopProfile>()
                .expect("a profile")
                .config(),
        ),
    }
}

#[test]
fn ego_steps_as_the_python() {
    let v = vectors();
    let mut w = Worst::new("ego step (speed, steer, accel)", EXACT);
    for run in v["ego_runs"].as_array().unwrap() {
        let name = run["name"].as_str().unwrap();
        let mut model = EgoModel::new(limits(&run["limits"]), body(&run["body"]));
        let mut lead = lead(&run["lead"]);
        let the_pedal = pedal(&run["pedal"]);
        for (i, (step, out)) in run["steps"]
            .as_array()
            .unwrap()
            .iter()
            .zip(run["outs"].as_array().unwrap())
            .enumerate()
        {
            let c = cmd(&step[0]);
            let dt = f(&step[1]);
            let on = lead.is_some() && c.is_some();
            if !on {
                if let Some(l) = lead.as_mut() {
                    l.reset();
                }
            }
            model.pedal = the_pedal.filter(|_| c.is_some());
            let got = model.step(c, dt, if on { lead.as_mut() } else { None });
            let what = format!("{name} step {i}");
            w.check(got.speed, f(&out[0]), &what);
            w.check(got.steering, f(&out[1]), &what);
            w.check(got.accel, f(&out[2]), &what);
        }
    }
    w.report();
}

#[test]
fn the_pedal_and_the_body_as_the_python() {
    let v = vectors();
    let mut w = Worst::new("pedal slow_rate", EXACT);
    for c in v["slow_rates"].as_array().unwrap() {
        let p = pedal(&c["pedal"]).unwrap();
        let got = p.slow_rate(f(&c["current"]), f(&c["target"]), f(&c["max_deceleration"]));
        w.check(got, f(&c["rate"]), &c.to_string());
    }
    w.report();

    let mut w = Worst::new("body limits", EXACT);
    let lim = Described {
        wheelbase: 1.561,
        max_steering_angle: 0.5,
        max_speed: 1.0,
        max_acceleration: 1.0,
        max_deceleration: 2.0,
        max_jerk: 2.0,
    };
    for c in v["bodies"].as_array().unwrap() {
        let b = body(&c["body"]).unwrap();
        w.check(b.speed_up_limit(), f(&c["speed_up"]), "speed_up");
        w.check(b.braking_limit(), f(&c["braking"]), "braking");
        w.check(b.lateral_limit(), f(&c["lateral"]), "lateral");
        for k in c["curve"].as_array().unwrap() {
            w.check(b.curve_speed(f(&k[0])), f(&k[1]), "curve_speed");
        }
        for s in c["stopping"].as_array().unwrap() {
            w.check(
                b.stopping_distance(f(&s[0]), f(&s[1])),
                f(&s[2]),
                "stopping",
            );
        }
        for e in c["energy"].as_array().unwrap() {
            w.check(b.energy(f(&e[0])), f(&e[1]), "energy");
        }
        let lines: Vec<String> = describe(&b, &lim)
            .into_iter()
            .map(|l| match l {
                LogLine::Info(s) => s,
                LogLine::Warning(s) => format!("WARNING: {s}"),
            })
            .collect();
        let want: Vec<String> = c["describe"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| s.as_str().unwrap().to_string())
            .collect();
        assert_eq!(lines, want, "describe");
    }
    w.report();
}

#[test]
fn the_stopping_law_as_the_python() {
    let v = vectors();
    let cfgs = &v["cfgs"];
    let mut w = Worst::new("stop curve", EXACT);
    for c in v["stop_curves"].as_array().unwrap() {
        let curve = if let Some(brake) = c.get("numbers") {
            StopCurve::from_numbers(f(brake), 2.0, 2.0).unwrap()
        } else {
            StopCurve::from_car_model(
                &cfg(&cfgs[c["cfg"].as_str().unwrap()]),
                f(&c["max_deceleration"]),
                f(&c["max_jerk"]),
                INPUT_DELAY,
                profile(c["speed_loop"].as_str().unwrap()).as_ref(),
            )
            .unwrap()
        };
        w.check(curve.a(), f(&c["a"]), "a");
        w.check(curve.j(), f(&c["j"]), "j");
        w.check(curve.rho(), f(&c["rho"]), &format!("rho {c}"));
        if c.get("numbers").is_some() {
            continue;
        }
        w.check(curve.tau(), f(&c["tau"]), "tau");
        for d in c["distance"].as_array().unwrap() {
            w.check(curve.distance(f(&d[0])), f(&d[1]), "distance");
        }
        for d in c["cap"].as_array().unwrap() {
            w.check(curve.cap(f(&d[0])), f(&d[1]), "cap");
        }
        assert_eq!(curve.describe(), c["describe"].as_str().unwrap());
    }
    w.report();
}

fn strip(v: &Value) -> Strip {
    Strip {
        spacing: f(&v["spacing"]),
        x: fs_(&v["x"]),
        y: fs_(&v["y"]),
        speed_limit: fs_(&v["speed_limit"]),
        end: f(&v["end"]),
        stops: v["stops"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| StopOnLine {
                along: f(&s[0]),
                reason: s[1].as_str().unwrap().to_string(),
            })
            .collect(),
    }
}

#[test]
fn the_road_ahead_as_the_python() {
    let v = vectors();
    let r = &v["roads"];
    let c = &r["curve"];
    let curve = StopCurve::new(f(&c["a"]), f(&c["j"]), f(&c["rho"])).unwrap();
    let mut w = Worst::new("road ahead", EXACT);
    for s in r["strips"].as_array().unwrap() {
        let strip = strip(&s["strip"]);
        for q in s["queries"].as_array().unwrap() {
            let (x, y) = (f(&q["x"]), f(&q["y"]));
            let (along, off) = ra::along_of(&strip, x, y);
            w.check(along, f(&q["along_of"][0]), "along");
            w.check(off, f(&q["along_of"][1]), "off");
            w.check(ra::room_at(&strip, x, y), f(&q["room"]), "room");
            w.check(
                ra::limit_at(Some(&strip), x, y, None, None),
                f(&q["limit"]),
                "limit",
            );
            w.check(
                ra::limit_at(Some(&strip), x, y, Some(&curve), None),
                f(&q["limit_curve"]),
                "limit, curve",
            );
            w.check(
                ra::limit_at(Some(&strip), x, y, Some(&curve), Some(0.6)),
                f(&q["limit_base"]),
                "limit, base",
            );
            w.check(
                ra::allowed(Some(&strip), Some((x, y)), f(&q["age"]), 1.0, &curve, 1.0),
                f(&q["allowed"]),
                "allowed",
            );
            let road = la::road(Some(&strip), x, y);
            match (&road, q["road"].as_array()) {
                (Some(got), Some(want)) => {
                    w.check(got.limit, f(&want[0]), "road limit");
                    w.check(got.room, f(&want[1]), "road room");
                    assert_eq!(got.why, want[2].as_str().unwrap());
                }
                (None, None) => {}
                (got, want) => panic!("road: got {got:?}, want {want:?}"),
            }
        }
    }
    for c in r["caps"].as_array().unwrap() {
        let (v_, w_) = ra::cap((f(&c["cmd"][0]), f(&c["cmd"][1])), f(&c["limit"]));
        w.check(v_, f(&c["out"][0]), "cap v");
        w.check(w_, f(&c["out"][1]), "cap w");
    }
    w.report();
}

#[test]
fn the_stop_point_as_the_python() {
    let v = vectors();
    let mut w = Worst::new("stop point", EXACT);
    for c in v["stop_points"].as_array().unwrap() {
        let mut config = cfg(&v["cfgs"][c["cfg"].as_str().unwrap()]);
        if let Some(o) = c.get("cfg_override") {
            config.max_brake = f(&o["max_brake"]);
            config.drag = f(&o["drag"]);
        }
        let max_time = c.get("max_time").map_or(ego_core::stop_ahead::MAX_TIME, f);
        let p = stop_point(&config, &state(&c["state"]), f(&c["dt"]), max_time).unwrap();
        let o = &c["out"];
        w.check(p.x, f(&o[0]), "x");
        w.check(p.y, f(&o[1]), "y");
        w.check(p.yaw, f(&o[2]), "yaw");
        w.check(p.distance, f(&o[3]), "distance");
        w.check(p.time, f(&o[4]), "time");
        assert_eq!(p.found, o[5].as_bool().unwrap());
    }
    w.report();
}

#[test]
fn the_trace_as_the_python() {
    let v = vectors();
    let mut cmds = Worst::new("trace: ego's commands", EXACT);
    let mut twins = Worst::new("trace: the twin (f32 loop)", LOOP);
    let mut traces = Worst::new("trace: the poses (f32 loop)", LOOP);
    for c in v["traces"].as_array().unwrap() {
        let config = cfg(&v["cfgs"][c["cfg"].as_str().unwrap()]);
        let loop_config = profile(c["speed_loop"].as_str().unwrap()).unwrap();
        let mut twin = Twin::new(config, loop_config);
        for op in c["ops"].as_array().unwrap() {
            match op[0].as_str().unwrap() {
                "measured" => twin.measured(f(&op[1])),
                _ => twin.sent(f(&op[1]), f(&op[2]), f(&op[3])).unwrap(),
            }
        }
        let want = state(&c["twin"]);
        for (got, want, what) in [
            (twin.state.x, want.x, "x"),
            (twin.state.y, want.y, "y"),
            (twin.state.yaw, want.yaw, "yaw"),
            (twin.state.v, want.v, "v"),
            (twin.state.steer, want.steer, "steer"),
            (twin.state.throttle_level, want.throttle_level, "throttle"),
            (twin.state.brake_level, want.brake_level, "brake"),
        ] {
            twins.check(got, want, what);
        }
        let m = &c["model"];
        let mut model = EgoModel::new(limits(&m["limits"]), body(&m["body"]));
        model.speed = f(&m["speed"]);
        model.steering = f(&m["steering"]);
        model.accel = f(&m["accel"]);
        let lead = lead(&c["lead"]);
        let held = cmd(&c["cmd"]);
        let got = ego_commands(&model, held, lead.as_ref(), 100, 0.02);
        for (g, want) in got.iter().zip(c["commands"].as_array().unwrap()) {
            let speed = match g.drive {
                ego_core::car_model::Drive::Speed(s) => s,
                ego_core::car_model::Drive::Pedals { .. } => f64::NAN,
            };
            cmds.check(speed, f(&want[0]), "command speed");
            cmds.check(g.steer, f(&want[1]), "command steer");
        }
        let poses = trace(&twin, &model, held, lead.as_ref(), 2.0, 0.02, 5).unwrap();
        let want = c["poses"].as_array().unwrap();
        assert_eq!(poses.len(), want.len());
        for (p, w) in poses.iter().zip(want) {
            traces.check(p.t, f(&w[0]), "t");
            traces.check(p.x, f(&w[1]), "x");
            traces.check(p.y, f(&w[2]), "y");
            traces.check(p.yaw, f(&w[3]), "yaw");
            traces.check(p.v, f(&w[4]), "v");
            traces.check(p.steer, f(&w[5]), "steer");
        }
    }
    cmds.report();
    twins.report();
    traces.report();
}

#[test]
fn the_squiggly_line_as_the_python() {
    let v = vectors();
    let l = &v["look_ahead"];
    let mut w = Worst::new("look_ahead lines", EXACT);
    for c in l["cases"].as_array().unwrap() {
        let plan = points(&c["plan"]);
        let trace = points(&c["trace"]);
        let stop = (f(&c["stop"][0]), f(&c["stop"][1]));
        w.check(la::length(&plan), f(&c["length"]), "length");
        match la::project(&plan, stop) {
            Some((a, o)) => {
                w.check(a, f(&c["project"][0]), "project along");
                w.check(o, f(&c["project"][1]), "project off");
            }
            None => assert!(c["project"].is_null()),
        }
        for (name, got, want) in [
            ("faint", la::crop_ahead(&plan, 10.0), points(&c["faint"])),
            ("cut", la::cut(&plan, 1.0, 6.5), points(&c["cut"])),
        ] {
            assert_eq!(got.len(), want.len(), "{name}: {c}");
            for (g, wa) in got.iter().zip(&want) {
                w.check(g.0, wa.0, name);
                w.check(g.1, wa.1, name);
            }
        }
        let gap = la::gap(&trace, &la::crop_ahead(&plan, 10.0));
        w.opt(gap, of(&c["gap"]), "gap");
        assert_eq!(
            u64::from(la::grade(gap, &la::Limits::default()).byte()),
            c["grade"].as_u64().unwrap()
        );
        w.opt(la::to_goal(&plan, stop), of(&c["to_goal"]), "to_goal");
        if let Some(n) = c["nearest"].as_u64() {
            assert_eq!(la::nearest_index(&plan) as u64, n);
        }
    }
    w.report();

    let mut w = Worst::new("look_ahead costmap", EXACT);
    for g in l["grids"].as_array().unwrap() {
        let grid = la::Grid {
            width: g["width"].as_u64().unwrap() as usize,
            height: g["height"].as_u64().unwrap() as usize,
            resolution: f(&g["resolution"]),
            origin: (f(&g["origin"][0]), f(&g["origin"][1]), f(&g["origin"][2])),
            data: g["data"]
                .as_array()
                .unwrap()
                .iter()
                .map(|d| d.as_i64().unwrap() as i8)
                .collect(),
        };
        let ext = la::extend(&poses(&g["path"]), 15.0);
        let want = poses(&g["extend"]);
        assert_eq!(ext.len(), want.len());
        for (a, b) in ext.iter().zip(&want) {
            w.check(a.0, b.0, "extend x");
            w.check(a.1, b.1, "extend y");
            w.check(a.2, b.2, "extend yaw");
        }
        w.opt(
            la::first_hit(&grid, &want, 1.9, 0.6, 100, 0.1),
            of(&g["hit"]),
            "first_hit",
        );
        w.opt(
            la::first_hit(&grid, &want, 1.9, 0.6, 50, 0.25),
            of(&g["hit_soft"]),
            "first_hit, 50 and 0.25 m",
        );
        for p in g["costs"].as_array().unwrap() {
            assert_eq!(
                i64::from(grid.cost((f(&p[0]), f(&p[1])))),
                p[2].as_i64().unwrap()
            );
        }
    }
    w.report();

    for d in l["describes"].as_array().unwrap() {
        let level = match d["level"].as_u64().unwrap() {
            0 => la::Level::Ok,
            1 => la::Level::Warn,
            _ => la::Level::Bad,
        };
        assert_eq!(
            la::describe(
                of(&d["gap"]),
                level,
                of(&d["stop"]),
                of(&d["goal"]),
                of(&d["room"])
            ),
            d["text"].as_str().unwrap()
        );
    }
}

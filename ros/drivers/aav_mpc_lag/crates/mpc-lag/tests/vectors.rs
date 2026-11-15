//! The Python's own numbers (vectors.json, from gen_vectors.py): the model step for step,
//! and the tracker's commands on the same observations.

use car_model::ActuatorConfig;
use mpc_lag::model::{self, PiLoop, NU, NX};
use mpc_lag::{Limits, MpcLag, Observed, Path, Settings};
use serde_json::Value;
use spine_core::speed_loop::SpeedLoopProfile;

fn vectors() -> Value {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/vectors.json"))
        .expect("vectors.json: run gen_vectors.py");
    serde_json::from_str(&text).expect("vectors.json is JSON")
}

fn numbers(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("a list")
        .iter()
        .map(|x| x.as_f64().expect("a number"))
        .collect()
}

fn car(v: &Value) -> ActuatorConfig {
    serde_json::from_value(v.clone()).expect("an ActuatorConfig")
}

fn profile(name: &str) -> SpeedLoopProfile {
    name.parse().expect("a speed loop profile")
}

#[test]
fn the_model_gives_the_pythons_states() {
    let data = vectors();
    let cars = &data["cars"];
    let mut worst: f64 = 0.0;
    let cases = data["model"].as_array().expect("model cases");
    assert!(cases.len() >= 200);
    for (i, case) in cases.iter().enumerate() {
        let c = car(&cars[case["car"].as_str().expect("a car")]);
        let lp =
            PiLoop::from_profile(profile(case["loop"].as_str().expect("a loop"))).expect("a PI");
        let x: [f64; NX] = numbers(&case["x"]).try_into().expect("NX numbers");
        let u: [f64; NU] = numbers(&case["u"]).try_into().expect("NU numbers");
        let substeps = case["substeps"].as_u64().expect("substeps") as usize;
        let kappa = case["kappa"].as_f64().expect("kappa");
        let dt = case["dt"].as_f64().expect("dt");
        let out = model::step(&c, &lp, &x, &u, kappa, dt, substeps);
        for (row, (got, want)) in out.iter().zip(numbers(&case["out"])).enumerate() {
            let err = (got - want).abs();
            worst = worst.max(err);
            assert!(
                err < 1e-9,
                "case {i} ({}, {}) row {row}: {got} against {want}",
                case["car"],
                case["loop"]
            );
        }
    }
    println!("model: {} cases, worst {worst:.1e}", cases.len());
}

#[test]
fn the_tracker_sends_the_pythons_commands() {
    let data = vectors();
    for case in data["tracker"].as_array().expect("tracker cases") {
        let name = case["name"].as_str().expect("a name");
        let l = &case["limits"];
        let limits = Limits {
            wheelbase: l["wheelbase"].as_f64().expect("wheelbase"),
            max_steer: l["max_steer"].as_f64().expect("max_steer"),
            max_steer_rate: l["max_steer_rate"].as_f64().expect("max_steer_rate"),
        };
        let settings = Settings {
            cruise: case["cruise"].as_f64().expect("cruise"),
            ..Settings::default()
        };
        let mut mpc = MpcLag::new(limits, car(&case["car"]), settings).expect("a tracker");
        let poses: Vec<(f64, f64, f64)> = case["poses"]
            .as_array()
            .expect("poses")
            .iter()
            .map(|p| {
                let p = numbers(p);
                (p[0], p[1], p[2])
            })
            .collect();
        mpc.reset(Path::new(&poses).expect("a path"));
        let (mut steer_worst, mut speed_worst): (f64, f64) = (0.0, 0.0);
        let ticks = case["ticks"].as_array().expect("ticks");
        for (k, t) in ticks.iter().enumerate() {
            let f = |key: &str| t[key].as_f64().expect("a number");
            let seen = Observed {
                x: f("x"),
                y: f("y"),
                yaw: f("yaw"),
                v: f("v"),
                steer: f("steer"),
            };
            let cmd = mpc.step(&seen, f("dt")).expect("a command");
            let (ds, dv) = (
                (cmd.steer - f("cmd_steer")).abs(),
                (cmd.speed - f("cmd_speed")).abs(),
            );
            steer_worst = steer_worst.max(ds);
            speed_worst = speed_worst.max(dv);
            assert!(
                ds < 2e-3 && dv < 2e-3,
                "{name} tick {k}: steer {} against {}, speed {} against {}",
                cmd.steer,
                f("cmd_steer"),
                cmd.speed,
                f("cmd_speed")
            );
        }
        println!(
            "{name}: {} ticks, worst steering {steer_worst:.1e} rad, speed {speed_worst:.1e} m/s",
            ticks.len()
        );
    }
}

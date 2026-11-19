//! /spine/arm and the boot self-test (TODO 255): with the ready gate (the car), the bridge
//! refuses to arm until the self-test says READY, and was heard recently. The same cases
//! as aav_spine's test_output_gate.py's ready gate.

use std::time::{Duration, Instant};

use spine_bridge::{
    ArmError, Arming, Bridge, Geometry, NotReady, ReadyGate, Role, Settings, Target, NOT_READY,
};

const SETTINGS: Settings = Settings {
    role: Role::Spine,
    geometry: Geometry {
        wheelbase: 1.57,
        track: 0.77,
    },
    cmd_timeout: Duration::from_millis(100),
    wheel_timeout: Duration::from_millis(100),
    target: Target::Car,
    gate_timeout: Duration::from_millis(1500),
};

const OPEN: &str = "OPEN - car: every source REAL";
const NOT_YET: &str = "NOT READY - sensors (STOP: lidar STALE)";

fn secs(s: f64) -> Duration {
    Duration::from_secs_f64(s)
}

fn car(t0: Instant) -> Bridge {
    let mut b = Bridge::new(SETTINGS).with_ready_gate(secs(2.0));
    b.output_mux_state(OPEN, t0);
    b
}

fn refused(bridge: &mut Bridge, now: Instant) -> String {
    match bridge.arm(true, now) {
        Err(ArmError::NotReady(why)) => why.to_string(),
        other => panic!("expected the self-test to refuse, got {other:?}"),
    }
}

#[test]
fn nothing_heard_refuses_and_says_what_to_wait_for() {
    let t0 = Instant::now();
    let mut b = car(t0);
    let why = refused(&mut b, t0);
    assert!(why.starts_with(NOT_READY), "{why}");
    assert!(why.contains("/safety/self_test/status"), "{why}");
}

#[test]
fn not_ready_refuses_with_its_line_and_ready_arms() {
    let t0 = Instant::now();
    let mut b = car(t0);
    b.self_test(false, NOT_YET, t0);
    assert_eq!(refused(&mut b, t0), format!("{NOT_READY}: {NOT_YET}"));
    b.self_test(true, "READY", t0);
    assert_eq!(b.arm(true, t0), Ok(Arming::Armed));
}

#[test]
fn a_quiet_self_test_refuses_but_disarming_always_works() {
    let t0 = Instant::now();
    let mut b = car(t0);
    b.self_test(true, "READY", t0);
    let later = t0 + secs(2.5);
    b.output_mux_state(OPEN, later);
    assert!(refused(&mut b, later).contains("quiet for 2.5 s"));
    assert_eq!(b.arm(false, later), Ok(Arming::Disarmed));
}

#[test]
fn the_e_stop_and_the_output_gate_are_checked_first() {
    let t0 = Instant::now();
    let mut b = Bridge::new(SETTINGS).with_ready_gate(secs(2.0));
    b.self_test(true, "READY", t0);
    assert!(matches!(b.arm(true, t0), Err(ArmError::GateShut(_))));
    b.output_mux_state(OPEN, t0);
    let _ = b.set_estop(true);
    assert_eq!(b.arm(true, t0), Err(ArmError::EstopActive));
}

#[test]
fn without_the_ready_gate_the_self_test_is_not_needed() {
    // The bench and Ravensim's spine: arm_when_open arms once the output gate opens.
    let t0 = Instant::now();
    let mut b = Bridge::new(SETTINGS);
    b.output_mux_state(OPEN, t0);
    b.self_test(false, NOT_YET, t0);
    assert_eq!(b.arm(true, t0), Ok(Arming::Armed));
}

#[test]
fn the_gate_alone() {
    let t0 = Instant::now();
    let mut gate = ReadyGate::new(secs(1.0));
    assert_eq!(gate.check(t0), Err(NotReady::NotHeard));
    gate.heard(true, "READY", t0);
    assert_eq!(gate.check(t0 + secs(1.0)), Ok(()));
    assert_eq!(gate.check(t0 + secs(1.5)), Err(NotReady::Quiet(secs(1.5))));
}

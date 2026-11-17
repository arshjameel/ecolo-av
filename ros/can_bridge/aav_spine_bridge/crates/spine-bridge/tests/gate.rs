//! /spine/arm and the safety gate (Ben, 24 Oct): the bridge refuses to arm unless the
//! output mux says OPEN for its own target, and was heard recently. The same cases as
//! aav_spine's test_output_gate.py.

use std::time::{Duration, Instant};

use spine_bridge::{
    ArmError, Arming, Bridge, Command, Drive, GateShut, Geometry, OutputGate, Role, Settings,
    Target, GATE_SHUT,
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
const WAIT: &str = "WAIT - car: all REAL, settling for 2 s";
const STOP: &str = "STOP - target car needs REAL - camera_front_left is SIM (sim_adapter) \
                    - latched: /safety/output_mux/clear";

fn secs(s: f64) -> Duration {
    Duration::from_secs_f64(s)
}

fn refused(bridge: &mut Bridge, now: Instant) -> String {
    match bridge.arm(true, now) {
        Err(ArmError::GateShut(shut)) => shut.to_string(),
        other => panic!("expected the gate to refuse, got {other:?}"),
    }
}

#[test]
fn refused_before_the_output_mux_is_heard() {
    let mut bridge = Bridge::new(SETTINGS);
    let message = refused(&mut bridge, Instant::now());
    assert!(message.starts_with(GATE_SHUT), "{message}");
    assert!(message.contains("/safety/output_mux/state"), "{message}");
}

#[test]
fn refused_while_it_waits_or_stops_with_its_reason() {
    let t0 = Instant::now();
    for line in [WAIT, STOP] {
        let mut bridge = Bridge::new(SETTINGS);
        bridge.output_mux_state(line, t0);
        let message = refused(&mut bridge, t0);
        assert_eq!(
            message,
            format!("{GATE_SHUT}: {line}; arm once it says OPEN")
        );
    }
}

#[test]
fn accepted_once_open_and_the_edge_follows() {
    let t0 = Instant::now();
    let mut bridge = Bridge::new(SETTINGS);
    bridge.output_mux_state(STOP, t0);
    assert!(bridge.arm(true, t0).is_err());
    bridge.output_mux_state(OPEN, t0 + secs(1.0));
    assert_eq!(bridge.arm(true, t0 + secs(1.0)), Ok(Arming::Armed));
    bridge.command(
        Command {
            speed: 1.0,
            ..Command::default()
        },
        Duration::ZERO,
    );
    let armed: Vec<bool> = (0..5)
        .filter_map(|_| match bridge.drive(Duration::ZERO) {
            Drive::Send(cmd) => Some(cmd.armed),
            Drive::NoCommand | Drive::Stale => None,
        })
        .collect();
    assert_eq!(armed, [false, false, false, true, true]);
}

#[test]
fn a_refusal_changes_nothing_else() {
    let t0 = Instant::now();
    let mut bridge = Bridge::new(SETTINGS);
    bridge.output_mux_state(OPEN, t0);
    bridge.arm(true, t0).ok();
    bridge.output_mux_state(STOP, t0);
    assert!(bridge.arm(true, t0).is_err());
    // Refused, but it doesn't disarm by itself: the output mux already stops the
    // commands, and the spine's watchdog latches.
    bridge.command(Command::default(), Duration::ZERO);
    assert!((0..5).any(|_| matches!(bridge.drive(Duration::ZERO), Drive::Send(c) if c.armed)));
}

#[test]
fn a_quiet_output_mux_counts_as_shut() {
    let t0 = Instant::now();
    let mut bridge = Bridge::new(SETTINGS);
    bridge.output_mux_state(OPEN, t0);
    assert_eq!(bridge.arm(true, t0 + secs(1.5)), Ok(Arming::Armed));
    let message = refused(&mut bridge, t0 + secs(2.0));
    assert_eq!(
        message,
        format!("{GATE_SHUT}: the output mux has been quiet for 2.0 s; arm once it says OPEN")
    );
}

#[test]
fn open_for_another_target_is_not_this_bridges_gate() {
    let t0 = Instant::now();
    let mut bridge = Bridge::new(SETTINGS);
    bridge.output_mux_state("OPEN - sim: every source SIM", t0);
    let message = refused(&mut bridge, t0);
    assert_eq!(
        message,
        format!(
            "{GATE_SHUT}: the output mux is for another target \
             (OPEN - sim: every source SIM); this bridge needs target car"
        )
    );
    // The bench's and Gazebo's bridges read their own.
    for (target, line) in [
        (Target::Bench, "OPEN - bench: every source BENCH"),
        (Target::Sim, "OPEN - sim: every source SIM"),
    ] {
        let mut gate = OutputGate::new(target, secs(1.5));
        gate.heard(line, t0);
        assert_eq!(gate.check(t0), Ok(()));
        gate.heard(OPEN, t0);
        assert!(matches!(gate.check(t0), Err(GateShut::OtherTarget { .. })));
    }
}

#[test]
fn disarming_works_with_the_gate_shut() {
    let mut bridge = Bridge::new(SETTINGS);
    assert_eq!(bridge.arm(false, Instant::now()), Ok(Arming::Disarmed));
}

#[test]
fn the_estop_is_said_first() {
    let t0 = Instant::now();
    let mut bridge = Bridge::new(SETTINGS);
    bridge.set_estop(true);
    assert_eq!(bridge.arm(true, t0), Err(ArmError::EstopActive));
}

#[test]
fn targets_by_name() {
    assert_eq!("car".parse(), Ok(Target::Car));
    assert_eq!("bench".parse(), Ok(Target::Bench));
    assert_eq!("sim".parse(), Ok(Target::Sim));
    assert!("turtle".parse::<Target>().is_err());
    assert_eq!(Target::Sim.to_string(), "sim");
}

//! The boot self-test's rules: READY only once every check passes for `settle` s.

use self_test_core::{Actuators, ClockSource, Config, Lock, SelfTest, DRIVER_FAULTS};

fn car() -> Config {
    Config {
        locks: vec![
            Lock::Watchdog,
            Lock::Geofence,
            Lock::OutputGate,
            Lock::StopZone,
        ],
        clocks: vec![ClockSource::Imu, ClockSource::Gps],
        ..Config::default()
    }
}

/// Everything healthy at `now`.
fn all_good(t: &mut SelfTest, now: f64) {
    t.watchdog_line("OK", now);
    t.lock_heard(Lock::Geofence, now);
    t.lock_heard(Lock::OutputGate, now);
    t.stop_zone_publishers(1);
    t.vehicle_status(0, now);
    t.stamp(ClockSource::Imu, now - 0.02, now);
    t.stamp(ClockSource::Gps, now - 0.1, now);
}

fn failing(t: &mut SelfTest, now: f64) -> Vec<String> {
    t.verdict(now)
        .outcomes
        .into_iter()
        .filter(|o| !o.ok())
        .map(|o| o.name)
        .collect()
}

#[test]
fn nothing_heard_is_not_ready_and_says_every_check() {
    let mut t = SelfTest::new(car());
    let v = t.verdict(0.0);
    assert!(!v.ready);
    assert_eq!(
        v.outcomes
            .iter()
            .map(|o| o.name.as_str())
            .collect::<Vec<_>>(),
        [
            "sensors",
            "actuators",
            "watchdog",
            "geofence",
            "output_gate",
            "stop_zone",
            "clock imu",
            "clock gps"
        ]
    );
    assert!(v.outcomes.iter().all(|o| !o.ok()));
    assert!(v.line().starts_with("NOT READY - sensors (no word"));
}

#[test]
fn ready_only_after_settling_and_lost_at_once() {
    let mut t = SelfTest::new(car());
    let mut now = 0.0;
    while now < 1.9 {
        all_good(&mut t, now);
        assert!(!t.verdict(now).ready, "settling at {now}");
        now += 0.1;
    }
    for _ in 0..3 {
        now += 0.1;
        all_good(&mut t, now);
    }
    let v = t.verdict(now);
    assert!(v.ready, "{}", v.line());
    assert_eq!(v.line(), "READY");
    // One STOP from the watchdog: NOT READY at once, and the settling starts again.
    t.watchdog_line("STOP: gps STALE (nothing for 2.3 s)", now);
    let v = t.verdict(now);
    assert!(!v.ready);
    assert_eq!(
        v.line(),
        "NOT READY - sensors (STOP: gps STALE (nothing for 2.3 s))"
    );
    all_good(&mut t, now + 0.1);
    assert!(!t.verdict(now + 0.1).ready);
}

#[test]
fn a_warning_watchdog_still_passes() {
    let mut t = SelfTest::new(car());
    all_good(&mut t, 0.0);
    t.watchdog_line("WARN: camera SLOW", 0.0);
    assert!(failing(&mut t, 0.0).is_empty());
}

#[test]
fn a_silent_lock_fails_whatever_it_said_last() {
    let mut t = SelfTest::new(car());
    all_good(&mut t, 0.0);
    // The geofence said LOST: it holds the programs, as it should. Alive is what counts.
    for now in [0.5, 1.0] {
        all_good(&mut t, now);
    }
    t.watchdog_line("OK", 1.6);
    t.lock_heard(Lock::OutputGate, 1.6);
    t.vehicle_status(0, 1.6);
    t.stamp(ClockSource::Imu, 1.6, 1.6);
    t.stamp(ClockSource::Gps, 1.6, 1.6);
    assert_eq!(failing(&mut t, 2.1), ["geofence"]);
    assert!(t
        .verdict(2.1)
        .line()
        .contains("geofence (word from it quiet for 1.1 s)"));
}

#[test]
fn a_stop_zone_that_isnt_running_fails() {
    let mut t = SelfTest::new(car());
    all_good(&mut t, 0.0);
    t.stop_zone_publishers(0);
    assert_eq!(failing(&mut t, 0.0), ["stop_zone"]);
}

#[test]
fn a_driver_fault_fails_but_a_command_timeout_does_not() {
    let mut t = SelfTest::new(car());
    all_good(&mut t, 0.0);
    // FAULT_CMD_TIMEOUT (1): the spine says it before anyone arms. Not an actuator.
    t.vehicle_status(1, 0.0);
    assert!(failing(&mut t, 0.0).is_empty());
    t.vehicle_status(16, 0.0);
    let v = t.verdict(0.0);
    assert!(
        v.line()
            .contains("actuators (driver faults 0x010: brake driver)"),
        "{}",
        v.line()
    );
    t.vehicle_status(DRIVER_FAULTS, 0.0);
    assert!(t
        .verdict(0.0)
        .line()
        .contains("steering driver, brake driver, throttle DAC, CAN bus"));
}

#[test]
fn the_uart_car_checks_its_link_and_none_checks_nothing() {
    let mut t = SelfTest::new(Config {
        actuators: Actuators::Link,
        ..car()
    });
    all_good(&mut t, 0.0);
    assert_eq!(failing(&mut t, 0.0), ["actuators"]);
    t.link_frame(0.0);
    assert!(failing(&mut t, 0.0).is_empty());
    // The link quiet for 0.6 s (more than status_timeout); the lines still fresh.
    assert_eq!(failing(&mut t, 0.6), ["actuators"]);
    let mut t = SelfTest::new(Config {
        actuators: Actuators::Unchecked,
        ..car()
    });
    all_good(&mut t, 0.0);
    assert!(failing(&mut t, 0.0).is_empty());
}

#[test]
fn clocks_out_of_step_fail_with_their_offset() {
    let mut t = SelfTest::new(car());
    all_good(&mut t, 100.0);
    t.stamp(ClockSource::Gps, 99.2, 100.0);
    assert_eq!(failing(&mut t, 100.0), ["clock gps"]);
    assert!(t.verdict(100.0).line().contains("+0.80 s from this clock"));
    // A clock ahead of ours too.
    t.stamp(ClockSource::Gps, 100.7, 100.0);
    assert!(t.verdict(100.0).line().contains("-0.70 s"));
    t.stamp(ClockSource::Gps, f64::NAN, 100.0);
    assert_eq!(failing(&mut t, 100.0), ["clock gps"]);
}

#[test]
fn names_parse_and_unknown_ones_say_what_is_known() {
    assert_eq!("stop_zone".parse(), Ok(Lock::StopZone));
    assert_eq!("odom".parse(), Ok(ClockSource::Odom));
    assert_eq!("link".parse(), Ok(Actuators::Link));
    let err = "fence".parse::<Lock>().unwrap_err().to_string();
    assert_eq!(
        err,
        "lock 'fence': use watchdog, geofence, output_gate, stop_zone"
    );
    assert!("both".parse::<Actuators>().is_err());
}

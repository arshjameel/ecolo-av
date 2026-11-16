//! `test_health.py`, ported: each check on a made-up car.

use health_core::health::{
    front_wheel_speeds, mean_and_sigma, summary, Check, Ekf, Health, Level, Settings, Status,
};
use health_core::G;

const WHEELBASE: f64 = 1.561; // vehicle.yaml, 31 Oct
const TRACK: f64 = 0.75;

fn health() -> Health {
    Health::new(WHEELBASE, TRACK, Settings::default())
}

fn get(report: &[Status], check: Check) -> &Status {
    report
        .iter()
        .find(|s| s.check == check)
        .expect("every check")
}

fn value(s: &Status, key: &str) -> f64 {
    s.values
        .iter()
        .find(|(k, _)| k == key)
        .map(|(_, v)| *v)
        .expect("the value")
}

/// A parked car at 50 Hz: wheels at 0 m/s, the IMU as given, the local EKF.
fn parked(h: &mut Health, seconds: f64, wz: f64, accel: [f64; 3], start: f64) -> f64 {
    let mut t = start;
    while t < start + seconds {
        h.odom(t, t, "odom", "base_link", 0.0);
        h.imu(t, t, "imu_link", wz, accel);
        h.ekf(Ekf::Local, t, t, "odom", "base_link", (0.0, 0.0), 0.0, 0.0);
        t += 0.02;
    }
    t
}

fn level_g() -> [f64; 3] {
    [0.0, 0.0, G]
}

#[test]
fn front_wheels_turn_apart_in_a_bend() {
    let (l, r) = front_wheel_speeds(2.0, 0.0, WHEELBASE, TRACK);
    assert!((l - 2.0).abs() < 1e-12 && (r - 2.0).abs() < 1e-12);
    let (l, r) = front_wheel_speeds(2.0, 0.5, WHEELBASE, TRACK);
    assert!(r > l); // a left turn: the right wheel goes further
}

#[test]
fn mean_and_sigma_of_the_mean() {
    assert_eq!(mean_and_sigma(&[1.0]), Some((1.0, 0.0)));
    let (mean, sigma) = mean_and_sigma(&[1.0, 3.0]).expect("two");
    assert_eq!(mean, 2.0);
    assert!((sigma - 1.0).abs() < 1e-12);
    assert_eq!(mean_and_sigma(&[]), None);
}

#[test]
fn a_healthy_parked_car_is_ok() {
    let mut h = health();
    let now = parked(&mut h, 8.0, 0.0, level_g(), 0.0);
    let r = h.report(now);
    assert_eq!(get(&r, Check::GyroBias).level, Level::Ok);
    assert_eq!(get(&r, Check::ImuMounting).level, Level::Ok);
    assert_eq!(get(&r, Check::GpsQuality).message, "not on this profile");
    assert_eq!(summary(&r), "OK");
}

#[test]
fn gyro_and_gravity_wait_until_parked_long_enough() {
    let mut h = health();
    let now = parked(&mut h, 2.0, 1.0_f64.to_radians(), level_g(), 0.0);
    assert!(get(&h.report(now), Check::GyroBias)
        .message
        .contains("judged after"));
}

#[test]
fn a_gyro_bias_warns_only_when_it_stands_out() {
    let mut h = health();
    let now = parked(&mut h, 8.0, 0.6_f64.to_radians(), level_g(), 0.0);
    let r = h.report(now);
    let gyro = get(&r, Check::GyroBias);
    assert_eq!(gyro.level, Level::Warn);
    assert!((value(gyro, "bias_dps") - 0.6).abs() < 1e-9);
    let mut h = health();
    let now = parked(&mut h, 8.0, 0.1_f64.to_radians(), level_g(), 0.0);
    assert_eq!(get(&h.report(now), Check::GyroBias).level, Level::Ok);
}

#[test]
fn imu_mounting_problems() {
    let cases = [
        ([0.0, 0.0, -G], "upside down"),
        ([0.0, 0.0, G / 9.80665], "gravity reads"), // wrong units: g, not m/s^2
        ([G * 0.35_f64.sin(), 0.0, G * 0.35_f64.cos()], "tilted"),
    ];
    for (accel, words) in cases {
        let mut h = health();
        let now = parked(&mut h, 8.0, 0.0, accel, 0.0);
        let r = h.report(now);
        let mount = get(&r, Check::ImuMounting);
        assert_eq!(mount.level, Level::Warn, "{words}");
        assert!(mount.message.contains(words), "{}", mount.message);
    }
}

#[test]
fn moving_clears_the_parked_samples() {
    let mut h = health();
    let t = parked(&mut h, 8.0, 0.6_f64.to_radians(), level_g(), 0.0);
    h.odom(t, t, "odom", "base_link", 1.0);
    assert_eq!(h.parked_samples(), 0);
}

#[test]
fn gps_quality() {
    let cases = [
        (-1, 2, 1.0, Level::Warn, "no fix"),
        (0, 0, 1.0, Level::Warn, "covariance unknown"),
        (0, 2, 9.0, Level::Warn, "poor fix"),
        (2, 2, 0.001, Level::Warn, "too sure"),
        (0, 2, 0.5, Level::Ok, "sigma 0.50 m"),
    ];
    for (status, cov_type, sigma, level, words) in cases {
        let mut h = health();
        let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
        let s2 = sigma * sigma;
        let cov = [s2, 0.0, 0.0, 0.0, s2, 0.0, 0.0, 0.0, 4.0 * s2];
        h.gps_fix(t, t, "gps_link", status, cov_type, &cov);
        let r = h.report(t);
        let gps = get(&r, Check::GpsQuality);
        assert_eq!(gps.level, level, "{words}");
        assert!(gps.message.contains(words), "{}", gps.message);
    }
}

#[test]
fn late_and_future_stamps_warn() {
    let mut h = health();
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    h.imu(t, t - 1.0, "imu_link", 0.0, level_g()); // a second behind
    let r = h.report(t);
    let stamps = get(&r, Check::Timestamps);
    assert_eq!(stamps.level, Level::Warn);
    assert!(stamps.message.contains("imu stamped +1.00 s off"));
    h.imu(t, t + 0.5, "imu_link", 0.0, level_g()); // from the future
    assert!(get(&h.report(t), Check::Timestamps)
        .message
        .contains("imu stamped -0.50 s off"));
    h.imu(t, t, "imu_link", 0.0, level_g());
    assert_eq!(get(&h.report(t), Check::Timestamps).level, Level::Ok);
}

#[test]
fn a_wrong_frame_warns_until_it_is_right() {
    let mut h = health();
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    h.odom(t, t, "odom", "base_footprint", 0.0);
    let r = h.report(t);
    let frames = get(&r, Check::Frames);
    assert_eq!(frames.level, Level::Warn);
    assert!(frames.message.contains("odom -> base_footprint"));
    h.odom(t, t, "odom", "base_link", 0.0);
    assert_eq!(get(&h.report(t), Check::Frames).level, Level::Ok);
}

#[test]
fn tf() {
    let mut h = health();
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    assert_eq!(get(&h.report(t), Check::Tf).message, "not looked yet");
    h.tf(vec!["gps_link".into()], Some(0.1));
    assert!(get(&h.report(t), Check::Tf).message.contains("gps_link"));
    h.tf(vec![], None);
    assert_eq!(get(&h.report(t), Check::Tf).level, Level::Warn);
    h.tf(vec![], Some(0.05));
    assert_eq!(get(&h.report(t), Check::Tf).level, Level::Ok);
}

/// A car driving at v and wz, its front wheels reading left and right.
fn driving(h: &mut Health, seconds: f64, v: f64, wz: f64, wheels: (f64, f64), start: f64) -> f64 {
    let mut t = start;
    while t < start + seconds {
        h.odom(t, t, "odom", "base_link", v);
        h.imu(t, t, "imu_link", wz, level_g());
        h.ekf(Ekf::Local, t, t, "odom", "base_link", (v * t, 0.0), v, wz);
        h.wheels(t, t, "base_link", wheels.0, wheels.1);
        t += 0.02;
    }
    t
}

#[test]
fn front_wheels_that_agree_with_the_turn_are_ok() {
    let mut h = health();
    let wheels = front_wheel_speeds(2.0, 0.4, WHEELBASE, TRACK);
    let t = driving(&mut h, 2.0, 2.0, 0.4, wheels, 0.0);
    assert_eq!(get(&h.report(t), Check::Wheels).level, Level::Ok);
}

#[test]
fn a_dead_wheel_warns_after_it_persists() {
    let mut h = health();
    let t = driving(&mut h, 0.5, 2.0, 0.0, (2.0, 0.0), 0.0); // the right one reads 0
    assert_eq!(get(&h.report(t), Check::Wheels).level, Level::Ok); // persist is 1 s
    let t = driving(&mut h, 1.0, 2.0, 0.0, (2.0, 0.0), t);
    assert_eq!(get(&h.report(t), Check::Wheels).level, Level::Warn);
}

#[test]
fn a_runaway_or_jumping_local_ekf_warns_for_a_while() {
    let mut h = health();
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    h.ekf(Ekf::Local, t, t, "odom", "base_link", (0.0, 0.0), 9.0, 0.0); // 9 m/s
    assert!(get(&h.report(t), Check::Motion)
        .message
        .contains("ekf_local: 9.0 m/s"));
    assert_ne!(get(&h.report(t + 6.0), Check::Motion).level, Level::Warn); // hold is 5 s
    let mut h = health();
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    let later = t + 0.1;
    h.ekf(
        Ekf::Local,
        later,
        later,
        "odom",
        "base_link",
        (10.0, 0.0),
        0.0,
        0.0,
    );
    assert!(get(&h.report(later), Check::Motion)
        .message
        .contains("jumped"));
}

#[test]
fn the_global_ekf_may_jump() {
    let mut h = health();
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    h.ekf(Ekf::Global, t, t, "map", "base_link", (0.0, 0.0), 0.0, 0.0);
    let later = t + 0.1;
    h.ekf(
        Ekf::Global,
        later,
        later,
        "map",
        "base_link",
        (10.0, 0.0),
        0.0,
        0.0,
    );
    assert_eq!(get(&h.report(later), Check::Motion).level, Level::Ok);
}

#[test]
fn a_quiet_required_input_is_stale() {
    let mut h = health();
    assert_eq!(get(&h.report(0.0), Check::GyroBias).level, Level::Stale); // no imu yet
    let t = parked(&mut h, 0.1, 0.0, level_g(), 0.0);
    let r = h.report(t + Settings::default().stale_after + 1.0);
    let gyro = get(&r, Check::GyroBias);
    assert_eq!(gyro.level, Level::Stale);
    assert!(gyro.message.contains("nothing on"));
    assert!(summary(&h.report(t + 5.0)).starts_with("WARN"));
}

#[test]
fn settings_round_trip_by_name() {
    let s = Settings::default();
    let back = Settings::from_fields(&s.fields()).expect("every name");
    assert_eq!(back, s);
    assert!(Settings::from_fields(&[("no_such", 1.0)]).is_none());
}

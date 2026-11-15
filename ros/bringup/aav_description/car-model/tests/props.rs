//! The car model's rules, checked on random states and commands (proptest, TODO 204).

use car_model::{ActuatorConfig, CarState, Command};
use proptest::prelude::*;

const DT: f64 = 0.02;

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    /// Full brake, no throttle: the car only ever slows, and never rolls the other way.
    #[test]
    fn braking_only_slows(v in -3.0..3.0f64, steer in -0.5..0.5f64, payload in 0.0..150.0f64) {
        let cfg = ActuatorConfig { payload, ..ActuatorConfig::default() };
        let mut s = CarState { v, ..CarState::default() };
        let reverse = v < 0.0;
        for _ in 0..1000 {
            let mut cmd = Command::pedals(0.0, 1.0, steer);
            if let car_model::Drive::Pedals { reverse: r, .. } = &mut cmd.drive {
                *r = reverse;
            }
            let next = cfg.step(&s, cmd, DT, None).expect("a good step");
            prop_assert!(next.v.abs() <= s.v.abs() + 1e-12, "{} -> {}", s.v, next.v);
            prop_assert!(next.v == 0.0 || next.v.signum() == v.signum());
            s = next;
        }
    }

    /// Whatever the steering asks, the wheels never pass full lock.
    #[test]
    fn the_wheels_stay_within_full_lock(asks in prop::collection::vec(-3.0..3.0f64, 1..300),
                                        throttle in 0.0..1.0f64) {
        let cfg = ActuatorConfig::default();
        let mut s = CarState::default();
        for steer in asks {
            s = cfg.step(&s, Command::pedals(throttle, 0.0, steer), DT, None).expect("a good step");
            prop_assert!(s.steer.abs() <= cfg.max_steering + 1e-9, "steer {}", s.steer);
            prop_assert!(s.v.is_finite() && s.x.is_finite() && s.y.is_finite());
        }
    }

    /// A step of no time, negative or NaN time is an error, never a panic.
    #[test]
    fn a_bad_dt_is_an_error(dt in prop_oneof![-10.0..=0.0f64, Just(f64::NAN)]) {
        let cfg = ActuatorConfig::default();
        prop_assert!(cfg.step(&CarState::default(), Command::pedals(0.5, 0.0, 0.0), dt, None).is_err());
    }
}

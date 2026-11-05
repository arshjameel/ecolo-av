//! The hardware panel's easter egg: when the little robot (hud-core's `robot`) may show.
//!
//! [`watch`] sums up a frame for hud-core's `EasterEgg`: is the car still, does an
//! e-stop or a warning show, and a fingerprint of what the screen says, so any change
//! hides the robot. [`PassWatch`] spots a lock-to-lock check that has just PASSED: the
//! robot drives across to celebrate.

use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    mem::discriminant,
    time::Instant,
};

use hud_core::{health::Health, robot::Watch};

use crate::{
    buttons::Buttons,
    calibration::{CalPage, Mode},
    gate::Gate,
    state::{PanelState, Row},
    sweep::Outcome,
};

/// Slower than this (m/s) is standing still.
pub const STILL: f64 = 0.05;

/// What the panel shows this frame, for the easter egg. `page`: which page (0 = DRIVE),
/// `input`: a key, a click, the mouse or the wheel this frame.
pub fn watch(
    state: &PanelState,
    buttons: &Buttons,
    gate: &Gate,
    cal: &CalPage,
    page: u8,
    input: bool,
    now: Instant,
) -> Watch {
    let sweeping = matches!(cal.mode(), Mode::Sweep(_) | Mode::Ending { .. });
    let still = state.seen_speed(now).is_none_or(|v| v.abs() < STILL)
        && buttons.asked_speed().abs() < STILL
        && !buttons.is_driving()
        && !buttons.is_centring()
        && !sweeping;
    let stale = Row::SENSORS
        .into_iter()
        .any(|row| matches!(state.health(row, now), Health::Stale(_)));
    let alarm = state.estop_on()
        || stale
        || buttons.needs_release()
        || buttons.last_zero().is_some()
        || state.geofence_holds(now);
    let mut h = DefaultHasher::new();
    page.hash(&mut h);
    state.kind().word().hash(&mut h);
    state.origins.banner().hash(&mut h);
    matches!(gate, Gate::On { .. }).hash(&mut h);
    for row in Row::SENSORS {
        discriminant(&state.health(row, now)).hash(&mut h);
    }
    buttons.asked_steer().to_bits().hash(&mut h);
    discriminant(cal.mode()).hash(&mut h);
    if let Mode::Jog { target } = cal.mode() {
        target.hash(&mut h);
    }
    cal.marks().hash(&mut h);
    cal.note().hash(&mut h);
    Watch {
        input,
        still,
        alarm,
        scene: h.finish(),
    }
}

/// Spots a lock-to-lock check that has just PASSED.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PassWatch {
    passed: bool,
}

impl PassWatch {
    /// True on the first frame the page's last check says PASSED.
    pub fn just_passed(&mut self, cal: &CalPage) -> bool {
        let passed = matches!(cal.checked(), Some(Outcome::Passed { .. }));
        let new = passed && !self.passed;
        self.passed = passed;
        new
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        buttons::{Press, Steps},
        config::PanelConfig,
        gate::Caps,
        rules::Kind,
        state::Update,
    };
    use hud_core::{
        units::{Fraction, MetresPerSecond, Radians},
        vehicle::{Faults, Mode as DriveMode, VehicleStatus},
    };
    use std::time::Duration;

    fn panel() -> (PanelState, Buttons, CalPage) {
        let config = PanelConfig::default();
        let state = PanelState::new(config.stale_after());
        let buttons = Buttons::new(Steps {
            speed: 0.1,
            steer: 0.05,
            creep: 0.1,
            max_steer: 0.5,
            wheelbase: 1.6,
        });
        (state, buttons, CalPage::new(config.calibration()))
    }

    fn vehicle(speed: f32, estop: bool) -> Update {
        Update::Vehicle(VehicleStatus {
            mode: DriveMode::Rc,
            armed: true,
            estop_active: estop,
            speed: MetresPerSecond(speed),
            steering_angle: Radians(0.0),
            throttle: Fraction::new(0.0),
            brake: Fraction::new(0.0),
            faults: Faults(0),
        })
    }

    fn on() -> Gate {
        crate::gate::gate(
            &crate::gate::Situation {
                kind: Kind::Sim,
                stands: false,
                speed_wheels: None,
                no_timeout_lock: None,
                speed_loop_mode: None,
            },
            Caps {
                sim: 1.0,
                stands: 0.3,
            },
        )
    }

    #[test]
    fn a_still_quiet_panel_is_calm() {
        let (mut s, b, cal) = panel();
        let t = Instant::now();
        s.apply(vehicle(0.0, false), t);
        let w = watch(&s, &b, &on(), &cal, 0, false, t);
        assert!(w.still && !w.alarm && !w.input);
        // The same screen, the same fingerprint.
        assert_eq!(w.scene, watch(&s, &b, &on(), &cal, 0, false, t).scene);
        // Another page: another fingerprint.
        assert_ne!(w.scene, watch(&s, &b, &on(), &cal, 1, false, t).scene);
    }

    #[test]
    fn moving_or_an_estop_is_not_calm() {
        let (mut s, b, cal) = panel();
        let t = Instant::now();
        s.apply(vehicle(1.2, false), t);
        assert!(!watch(&s, &b, &on(), &cal, 0, false, t).still);
        s.apply(vehicle(0.0, true), t);
        let w = watch(&s, &b, &on(), &cal, 0, false, t);
        assert!(w.still && w.alarm);
    }

    #[test]
    fn a_stale_sensor_is_a_warning() {
        let (mut s, b, cal) = panel();
        let t = Instant::now();
        s.apply(vehicle(0.0, false), t);
        let later = t + Duration::from_secs(5);
        let w = watch(&s, &b, &on(), &cal, 0, false, later);
        assert!(w.alarm, "the vehicle row went STALE");
        // NEVER (nothing heard yet) is not a warning.
        let (s, b, cal) = panel();
        assert!(!watch(&s, &b, &on(), &cal, 0, false, t).alarm);
    }

    #[test]
    fn asking_for_speed_is_not_still() {
        let (mut s, mut b, cal) = panel();
        let t = Instant::now();
        s.apply(vehicle(0.0, false), t);
        let gate = on();
        b.press(Press::SpeedUp, t, &gate);
        assert!(!watch(&s, &b, &gate, &cal, 0, false, t).still);
    }

    #[test]
    fn a_pass_is_seen_once() {
        let (_, _, cal) = panel();
        let mut p = PassWatch::default();
        assert!(!p.just_passed(&cal));
        assert!(!p.just_passed(&cal));
    }
}

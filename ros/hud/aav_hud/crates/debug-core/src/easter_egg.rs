//! The debug screen's easter egg: when the little robot's eyes (hud-core's `robot`) may
//! peek over the bottom of the 3D view.
//!
//! [`watch`] sums up a frame for hud-core's `EasterEgg`: is the car still, does a
//! warning show (an e-stop, a STALE row, the health checks' WARN, a frame or cloud
//! problem), and a fingerprint of what the screen says, so any change hides the eyes.

use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    mem::discriminant,
    time::Instant,
};

use hud_core::{health::Health, robot::Watch};

use crate::{
    localization_health::Verdict,
    state::{DebugState, Source},
};

/// Slower than this (m/s) is standing still.
pub const STILL: f64 = 0.05;

/// What the debug screen shows this frame, for the easter egg. `input`: a key, a click,
/// the mouse or the wheel this frame.
pub fn watch(state: &DebugState, input: bool, now: Instant) -> Watch {
    let ekf_still = state.speed.is_none_or(|v| v.abs() < STILL);
    let wheels_still = state
        .vehicle
        .is_none_or(|v| f64::from(v.speed.0).abs() < STILL);
    let stale = Source::PANEL
        .into_iter()
        .any(|s| matches!(state.health(s, now), Health::Stale(_)));
    let warned = matches!(state.localization_health, Some(Verdict::Warn(_)));
    let estop = state.vehicle.is_some_and(|v| v.estop_active);
    let alarm =
        stale || warned || estop || state.frame_problem.is_some() || state.scan_problem.is_some();
    let mut h = DefaultHasher::new();
    format!("{:?}", state.layers).hash(&mut h);
    state.origins.banner().hash(&mut h);
    for s in Source::PANEL {
        discriminant(&state.health(s, now)).hash(&mut h);
    }
    state.swap_reply.as_ref().map(|r| r.asked).hash(&mut h);
    Watch {
        input,
        still: ekf_still && wheels_still,
        alarm,
        scene: h.finish(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{config::DebugConfig, state::Update};
    use map_core::geom::Pose;
    use std::time::Duration;

    fn state() -> DebugState {
        DebugState::new(&DebugConfig::default())
    }

    fn moving(speed: f64) -> Update {
        Update::Pose {
            who: crate::state::Localizer::Local,
            pose: Pose::new(0.0, 0.0, 0.0),
            frame: DebugConfig::default().odom_frame,
            speed: Some(speed),
        }
    }

    #[test]
    fn nothing_heard_is_calm() {
        let s = state();
        let w = watch(&s, false, Instant::now());
        assert!(w.still && !w.alarm && !w.input);
    }

    #[test]
    fn a_moving_car_is_not_still() {
        let mut s = state();
        let t = Instant::now();
        s.apply(moving(2.5), t);
        assert!(!watch(&s, false, t).still);
        s.apply(moving(0.0), t);
        assert!(watch(&s, false, t).still);
    }

    #[test]
    fn a_stale_row_is_a_warning_and_changes_the_screen() {
        let mut s = state();
        let t = Instant::now();
        s.apply(moving(0.0), t);
        let fresh = watch(&s, false, t);
        let later = watch(&s, false, t + Duration::from_secs(10));
        assert!(!fresh.alarm && later.alarm);
        assert_ne!(fresh.scene, later.scene);
    }

    #[test]
    fn a_layer_switched_changes_the_screen() {
        let mut s = state();
        let t = Instant::now();
        let before = watch(&s, false, t).scene;
        s.layers.keys = !s.layers.keys;
        assert_ne!(before, watch(&s, false, t).scene);
    }
}

//! ego's trace: where the car is going over the next few seconds (`trace_ahead.py`).
//!
//! ```text
//! request held ──▶ copy of ego (its lead, ramps, limits) ──▶ speed, angle each step
//!              ──▶ car model (speed loop, dead times, lags, steering motor, bicycle)
//!              ──▶ poses every `every` steps, from now to now + horizon
//! ```
//!
//! ego keeps a [`Twin`] of the car: the shared car model, stepped every tick with what ego
//! sent, its speed set to the measured one.

use car_model::{ActuatorConfig, CarState, Command, ModelError, SpeedLoop, SpeedLoopConfig};

use crate::ackermann::EgoModel;
use crate::lag_lead::LagLead;
use crate::round_half_even;

/// One pose of the trace, in `base_link` now.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// s from now.
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub steer: f64,
}

/// ego's copy of the car: the car model, stepped with what ego sends.
#[derive(Debug, Clone)]
pub struct Twin {
    pub cfg: ActuatorConfig,
    pub speed_loop: SpeedLoop,
    pub state: CarState,
}

impl Twin {
    pub fn new(cfg: ActuatorConfig, loop_config: SpeedLoopConfig) -> Self {
        Twin {
            cfg,
            speed_loop: SpeedLoop::new(loop_config),
            state: CarState::default(),
        }
    }

    /// Take the car's measured speed (m/s).
    pub fn measured(&mut self, speed: f64) {
        self.state.v = speed;
    }

    /// Answer what ego sent this tick (m/s, rad) with the twin's actuators.
    pub fn sent(&mut self, speed: f64, angle: f64, dt: f64) -> Result<(), ModelError> {
        if dt > 0.0 {
            self.state = self.cfg.step(
                &self.state,
                Command::speed(speed, angle),
                dt,
                Some(&mut self.speed_loop),
            )?;
        }
        Ok(())
    }
}

/// What ego would send each step for `steps` steps, the request held. The model and the
/// lead are copied: the caller's own never change.
pub fn ego_commands(
    model: &EgoModel,
    cmd: Option<(f64, f64)>,
    lead: Option<&LagLead>,
    steps: usize,
    dt: f64,
) -> Vec<Command> {
    let mut model = model.clone();
    let mut lead = lead.cloned();
    (0..steps)
        .map(|_| {
            let out = model.step(cmd, dt, lead.as_mut());
            Command::speed(out.speed, out.steering)
        })
        .collect()
}

/// The poses every `every` steps of `dt`, from now to `horizon` s ahead, in `base_link`.
///
/// `cmd` is ego's request (`None`: stopping), `lead` its lead while it runs. Nothing
/// passed in is changed.
pub fn trace(
    twin: &Twin,
    model: &EgoModel,
    cmd: Option<(f64, f64)>,
    lead: Option<&LagLead>,
    horizon: f64,
    dt: f64,
    every: usize,
) -> Result<Vec<Pose>, ModelError> {
    if dt <= 0.0 || dt.is_nan() {
        return Err(ModelError::NonPositiveDt(dt));
    }
    let steps = round_half_even(horizon / dt).max(1.0) as usize;
    let start = CarState {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
        ..twin.state.clone()
    };
    let commands = ego_commands(model, cmd, lead, steps, dt);
    let states = twin
        .cfg
        .predict(&start, &commands, dt, horizon, Some(&twin.speed_loop))?;
    let every = every.max(1);
    let last = states.len();
    let mut poses = vec![Pose {
        t: 0.0,
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
        v: start.v,
        steer: start.steer,
    }];
    poses.extend(
        states
            .iter()
            .enumerate()
            .map(|(i, s)| (i + 1, s))
            .filter(|(i, _)| i % every == 0 || *i == last)
            .map(|(i, s)| Pose {
                t: i as f64 * dt,
                x: s.x,
                y: s.y,
                yaw: s.yaw,
                v: s.v,
                steer: s.steer,
            }),
    );
    Ok(poses)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ackermann::Limits;

    #[test]
    fn a_held_request_goes_ahead_and_turns_left() {
        let twin = Twin::new(ActuatorConfig::default(), SpeedLoopConfig::default());
        let model = EgoModel::new(Limits::default(), None);
        let poses = trace(&twin, &model, Some((1.0, 0.3)), None, 2.0, 0.02, 5).unwrap();
        assert_eq!(poses.len(), 21);
        let end = poses[20];
        assert!(end.x > 0.3 && end.y > 0.0, "{end:?}");
        assert!((end.t - 2.0).abs() < 1e-12);
        // Nothing passed in changed.
        assert_eq!(model.speed, 0.0);
    }
}

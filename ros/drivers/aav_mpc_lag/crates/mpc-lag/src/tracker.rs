//! The lag-aware MPC: plan the steering and the set speed through the car's own lags.
//! A port of `aav_track/trackers/mpc_lag.py`, step for step.
//!
//! ```text
//! set speed ──> the spine's PI ──> throttle, brake (lags) ──> speed (drag)
//! steering  ──> dead time ──> motor (rate, deadband) ──> linkage (lag) ──> wheels
//!                                                  └──> the bicycle, in the path's frame
//! ```
//!
//! Every tick (real-time iteration, at the node's 20 Hz):
//!
//! 1. Its state: the pose and speed measured; the throttle, brake, steering motor, wheels
//!    and the speed loop's integral estimated with the shared car model (`car-model`),
//!    from the commands it sent; the commands still inside a dead time, in queues.
//! 2. A nominal plan: the last plan shifted by a tick, run through the model ([`model`])
//!    for `horizon` ticks, linearised about it by finite differences, and condensed:
//!    `X = X_bar + M dU`.
//! 3. One QP ([`qp`], OSQP's ADMM) in `dU = [steering (N), set speed (N), slack (N)]`:
//!
//! ```text
//! cost   sum q.(e_y - tube(e_y), e_theta, wheels - r)^2 + q_speed (v - v_lim)^2
//!          + r (u - r)^2 + r_rate (du)^2 + r_speed (dc)^2 + w_slack slack^2
//! within |u| <= full lock, |du| <= rate dt
//!        0 <= c <= cruise, -decel dt <= dc <= accel dt
//!        v - slack <= v_lim, slack >= 0      (v_lim: cruise, the bends, the stop)
//! ```
//!
//! 4. It sends `u[0]` and `c[0]`, within the limits exactly.
//!
//! The deadbands are left out of the plan (they'd flatten its gradients) but kept in the
//! estimate. The dead times are whole ticks; `latency` adds the ROS chain's own delay.
//!
//! [`model`]: crate::model
//! [`qp`]: crate::qp

use std::collections::VecDeque;
use std::fmt;
use std::time::Instant;

use car_model::{ActuatorConfig, CarState, Command as CarCommand, SpeedLoop};
use spine_core::speed_loop::SpeedLoopProfile;

use crate::model::{self, row, Input, PiLoop, State, NU, NX};
use crate::path::{wrap, Foot, Path};
use crate::qp::{self, RowMatrix};

/// The share of full braking a plan uses (aav_drive_to_point approach.py BRAKE_FRACTION,
/// docs/tests.md 5.2).
pub const BRAKE_FRACTION: f64 = 0.5;

/// The states the cost sees: e_y, e_theta, wheels, v.
const COSTED: [usize; 4] = [row::E_Y, row::E_TH, row::WHEELS, row::V];
/// m: how far ahead the foot is searched for (path.py's window).
const SEARCH: f64 = 10.0;

/// The steering's limits (vehicle.yaml; the tighter of ego's rate and the motor's).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// m.
    pub wheelbase: f64,
    /// rad, full lock either side.
    pub max_steer: f64,
    /// rad/s.
    pub max_steer_rate: f64,
}

/// The tracker's settings: mpc_lag.py's defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// m/s: the most it plans (the policy's cruise).
    pub cruise: f64,
    /// m/s²: how fast the set speed may rise (the velocity smoother's).
    pub accel: f64,
    /// m/s²: how fast it may fall.
    pub decel: f64,
    /// m/s² at full brake (vehicle.yaml); half is planned.
    pub max_brake: f64,
    /// m/s² sideways: slow for a bend.
    pub lateral_accel: f64,
    /// s a tick.
    pub period: f64,
    /// Ticks planned.
    pub horizon: usize,
    /// Weights on e_y, e_theta, wheels - r.
    pub q: [f64; 3],
    /// m each side of the path where being off it costs nothing (Ben, 21 Nov: "It
    /// doesn't need to be super aggressive sticking to the line"): e_y is costed only
    /// beyond it, so the car holds its line instead of chasing the centre. 0 = off.
    pub tube: f64,
    /// On the command's distance from the bend's own steering.
    pub r: f64,
    /// On each tick's change of steering.
    pub r_rate: f64,
    /// On (v - the speed limit there)².
    pub q_speed: f64,
    /// On each tick's change of set speed.
    pub r_speed: f64,
    /// On the speed limit's slack.
    pub w_slack: f64,
    /// The last tick's weights, this many times.
    pub terminal: f64,
    /// s the ROS chain adds before the actuators, added to the dead times.
    pub latency: f64,
    /// The spine's speed loop it predicts (a PI: default or smooth).
    pub speed_loop: SpeedLoopProfile,
    /// Model steps a tick.
    pub substeps: usize,
    /// The QP's iterations at most.
    pub max_iter: usize,
    /// The QP's rho.
    pub rho: f64,
    /// QPs a tick (sequential QP).
    pub sqp: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            cruise: 1.0,
            accel: 1.0,
            decel: 2.0,
            max_brake: 3.0,
            lateral_accel: 1.0,
            period: 0.05,
            horizon: 30,
            q: [1.0, 2.0, 0.05],
            tube: 0.0,
            r: 0.2,
            r_rate: 1.0,
            q_speed: 0.5,
            r_speed: 0.5,
            w_slack: 50.0,
            terminal: 5.0,
            latency: 0.1,
            speed_loop: SpeedLoopProfile::Default,
            substeps: 2,
            max_iter: 400,
            rho: 1.0,
            sqp: 1,
        }
    }
}

/// Why the tracker can't be built or can't plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrackError {
    /// A setting out of its range: which one.
    Setting(&'static str),
    /// The steering limits are wrong: wheelbase or rate not above 0, full lock not in
    /// (0, pi/2).
    Limits,
    /// The LQI speed loop isn't modelled.
    Loop(model::LoopError),
    /// `step` before `reset(path)`.
    NoPath,
    /// The QP couldn't be solved.
    Qp(qp::QpError),
}

impl fmt::Display for TrackError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TrackError::Setting(name) => write!(f, "the setting {name} is out of its range"),
            TrackError::Limits => {
                f.write_str("wheelbase and steering rate must be above 0, full lock in (0, pi/2)")
            }
            TrackError::Loop(e) => e.fmt(f),
            TrackError::NoPath => f.write_str("step() before reset(path)"),
            TrackError::Qp(e) => write!(f, "the QP: {e}"),
        }
    }
}

impl std::error::Error for TrackError {}

impl From<qp::QpError> for TrackError {
    fn from(e: qp::QpError) -> Self {
        TrackError::Qp(e)
    }
}

/// The car now: base_link in the path's frame, its speed, and the last steering asked.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Observed {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left of x.
    pub yaw: f64,
    /// m/s, + forward.
    pub v: f64,
    /// rad: the last command sent (+ left).
    pub steer: f64,
}

/// What a tick says beside its command (the status topic's extra keys).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Report {
    /// ms the tick took.
    pub solve_ms: f64,
    /// The QP's iterations.
    pub iterations: usize,
    /// Every QP converged.
    pub solved: bool,
    /// rad: the wheels' angle estimated.
    pub wheels_est: f64,
    /// m/s: the speed limit where the car is.
    pub speed_limit: f64,
    /// rad: the bend's own steering now.
    pub feed_forward: f64,
    /// Where the car is on the path.
    pub foot: Option<Foot>,
}

/// One tick's command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// rad, + left.
    pub steer: f64,
    /// m/s: its own set speed.
    pub speed: f64,
    /// What the tick saw and chose, for the status.
    pub report: Report,
}

/// One QP's outcome: the plan, iterations, solved, the bend's steering and v_lim now.
struct Solved {
    plan: Vec<Input>,
    iterations: usize,
    solved: bool,
    feed_forward: f64,
    speed_limit: f64,
}

/// The lag-aware MPC.
#[derive(Debug, Clone)]
pub struct MpcLag {
    limits: Limits,
    settings: Settings,
    /// The estimate's car (no dead times: the queues keep those).
    car: ActuatorConfig,
    /// The plan's car (no deadbands either).
    plan_car: ActuatorConfig,
    pi: PiLoop,
    model_dt: f64,
    brake: f64,
    d_steer: usize,
    d_speed: usize,
    path: Option<Path>,
    hint: usize,
    car_state: Option<CarState>,
    speed_loop: SpeedLoop,
    sent: (f64, f64),
    acting: (f64, f64),
    steer_queue: VecDeque<f64>,
    speed_queue: VecDeque<f64>,
    plan_u: Option<Vec<Input>>,
    warm_y: Option<Vec<f64>>,
    predicted: Vec<[f64; 4]>,
    /// Set speed 0 (within the ramp), whatever the plan says: at a cusp (`geared`).
    holding: bool,
}

impl MpcLag {
    /// The tracker for this car (`car`: vehicle.yaml's actuators; its wheelbase and full
    /// lock are taken from `limits`).
    ///
    /// # Errors
    ///
    /// [`TrackError::Limits`], [`TrackError::Setting`] or [`TrackError::Loop`].
    pub fn new(
        limits: Limits,
        car: ActuatorConfig,
        settings: Settings,
    ) -> Result<Self, TrackError> {
        let s = &settings;
        if !(limits.wheelbase > 0.0
            && limits.max_steer > 0.0
            && limits.max_steer < std::f64::consts::FRAC_PI_2
            && limits.max_steer_rate > 0.0)
        {
            return Err(TrackError::Limits);
        }
        let checks: [(&'static str, bool); 16] = [
            ("tube", s.tube >= 0.0),
            ("cruise", s.cruise > 0.0),
            ("accel", s.accel > 0.0),
            ("decel", s.decel > 0.0),
            ("max_brake", s.max_brake > 0.0),
            ("lateral_accel", s.lateral_accel > 0.0),
            ("period", s.period > 0.0),
            ("horizon", s.horizon >= 2),
            ("r", s.r > 0.0),
            ("r_rate", s.r_rate >= 0.0),
            ("q_speed", s.q_speed >= 0.0),
            ("r_speed", s.r_speed >= 0.0),
            ("w_slack", s.w_slack > 0.0),
            ("terminal", s.terminal >= 1.0),
            ("latency", s.latency >= 0.0),
            ("substeps", s.substeps >= 1 && s.sqp >= 1),
        ];
        if let Some((name, _)) = checks.iter().find(|(_, ok)| !ok) {
            return Err(TrackError::Setting(name));
        }
        let pi = PiLoop::from_profile(s.speed_loop).map_err(TrackError::Loop)?;
        let model = ActuatorConfig {
            wheelbase: limits.wheelbase,
            max_steering: limits.max_steer,
            ..car
        };
        let estimate_car = model::without_dead_times(&model);
        Ok(MpcLag {
            limits,
            settings,
            car: estimate_car,
            plan_car: model::without_deadbands(&estimate_car),
            pi,
            model_dt: s.period / s.substeps as f64,
            brake: s.max_brake * BRAKE_FRACTION,
            d_steer: model::dead_ticks(model.steer_dead_time + s.latency, s.period),
            d_speed: model::dead_ticks(
                model.throttle_dead_time.max(model.brake_dead_time) + s.latency,
                s.period,
            ),
            path: None,
            hint: 0,
            car_state: None,
            speed_loop: SpeedLoop::new(s.speed_loop.config()),
            sent: (0.0, 0.0),
            acting: (0.0, 0.0),
            steer_queue: VecDeque::new(),
            speed_queue: VecDeque::new(),
            plan_u: None,
            warm_y: None,
            predicted: Vec::new(),
            holding: false,
        })
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// Plan up to this speed from now on (m/s, above 0).
    pub fn set_cruise(&mut self, cruise: f64) {
        if cruise > 0.0 {
            self.settings.cruise = cruise;
        }
    }

    /// Whole ticks of dead time: (steering, set speed).
    #[must_use]
    pub fn dead_ticks(&self) -> (usize, usize) {
        (self.d_steer, self.d_speed)
    }

    /// The commands waiting in the dead times: (steering, set speed).
    #[must_use]
    pub fn queued(&self) -> (usize, usize) {
        (self.steer_queue.len(), self.speed_queue.len())
    }

    /// What the last plan expects, a tick at a time: (e_y, e_theta, wheels, v).
    #[must_use]
    pub fn predicted(&self) -> &[[f64; 4]] {
        &self.predicted
    }

    /// The path it follows.
    #[must_use]
    pub fn path(&self) -> Option<&Path> {
        self.path.as_ref()
    }

    /// Take a new path. The estimate and the plan carry on: it starts where the car is.
    pub fn reset(&mut self, path: Path) {
        self.path = Some(path);
        self.hint = 0;
    }

    /// Move the path (the held map -> odom moved), keeping the place on it.
    pub fn move_path(&mut self, f: impl Fn(f64, f64, f64) -> (f64, f64, f64)) {
        if let Some(path) = self.path.as_mut() {
            *path = path.moved(f);
        }
    }

    /// Forget the estimate and the plan: the next tick starts from the car as measured
    /// (after a stop, when the commands sent no longer tell what the actuators do).
    pub fn forget(&mut self) {
        self.car_state = None;
        self.plan_u = None;
        self.warm_y = None;
    }

    /// Hold the car: send set speed 0 (slowing within the ramp) until released. The plan
    /// runs on as before. `geared` holds it at a cusp it has reached, so it stands there
    /// for the change of gear instead of creeping past the stretch's end.
    pub fn hold(&mut self, on: bool) {
        self.holding = on;
    }

    /// The car turned round at a cusp (the next stretch is driven the other way, mirrored,
    /// `gears`): the estimate's steering, the commands in the dead time and the plan's
    /// steering change sign, so they stay true in the new stretch's frame. The speeds stay
    /// (the car stands at a cusp); the QP starts cold.
    pub fn turn_round(&mut self) {
        if let Some(car) = self.car_state.as_mut() {
            car.steer = -car.steer;
            car.steer_motor = -car.steer_motor;
            car.steer_setpoint = -car.steer_setpoint;
            car.steer_linkage = -car.steer_linkage;
        }
        self.steer_queue.iter_mut().for_each(|s| *s = -*s);
        self.sent.0 = -self.sent.0;
        self.acting.0 = -self.acting.0;
        if let Some(plan) = self.plan_u.as_mut() {
            plan.iter_mut().for_each(|u| u[0] = -u[0]);
        }
        self.warm_y = None;
    }

    /// m from (x, y)'s foot on the path to its end (searched from the last foot), or None
    /// before a path.
    #[must_use]
    pub fn left_from(&self, x: f64, y: f64) -> Option<f64> {
        let path = self.path.as_ref()?;
        Some(path.left(path.project((x, y), self.hint, SEARCH).s))
    }

    fn foot(&mut self, x: f64, y: f64) -> Result<Foot, TrackError> {
        let path = self.path.as_ref().ok_or(TrackError::NoPath)?;
        let f = path.project((x, y), self.hint, SEARCH);
        self.hint = f.index;
        Ok(f)
    }

    /// The steering within full lock, and within the rate of `previous`.
    fn limited(&self, steer: f64, previous: f64, dt: f64) -> f64 {
        let lim = &self.limits;
        let step = lim.max_steer_rate * dt.max(0.0);
        model::clip(
            model::clip(steer, previous - step, previous + step),
            -lim.max_steer,
            lim.max_steer,
        )
    }

    // --- the estimate ---------------------------------------------------------------------

    /// The state now: the pose and speed measured, the rest from what was sent.
    fn estimate(&mut self, seen: &Observed, dt: f64, foot: &Foot) -> State {
        let v = seen.v.max(0.0);
        let car = match self.car_state.take() {
            None => {
                self.speed_loop = SpeedLoop::new(self.settings.speed_loop.config());
                self.steer_queue = std::iter::repeat_n(seen.steer, self.d_steer).collect();
                self.speed_queue = std::iter::repeat_n(v, self.d_speed).collect();
                self.sent = (seen.steer, v);
                CarState {
                    steer: seen.steer,
                    steer_motor: seen.steer,
                    steer_setpoint: seen.steer,
                    steer_linkage: seen.steer,
                    ..CarState::default()
                }
            }
            Some(mut car) => {
                // The commands that acted over the last tick (sent a dead time before it).
                let command = CarCommand::speed(self.acting.1, self.acting.0);
                let steps = ((dt / self.model_dt).round() as usize).max(1);
                for _ in 0..steps {
                    // model_dt > 0 and a speed loop given: step can't fail.
                    if let Ok(next) =
                        self.car
                            .step(&car, command, self.model_dt, Some(&mut self.speed_loop))
                    {
                        car = next;
                    }
                }
                car
            }
        };
        let mut x = [0.0; NX];
        x[row::E_Y] = foot.cross;
        x[row::E_TH] = wrap(seen.yaw - foot.heading);
        x[row::S] = foot.s;
        x[row::V] = v;
        x[row::THROTTLE] = car.throttle_level;
        x[row::BRAKE] = car.brake_level;
        x[row::SETPOINT] = car.steer_setpoint;
        x[row::MOTOR] = car.steer_motor;
        x[row::WHEELS] = car.steer_linkage;
        x[row::INTEGRAL] = f64::from(self.speed_loop.integral());
        // The next tick starts from the speed measured now.
        self.car_state = Some(CarState { v, ..car });
        x
    }

    // --- the speed limit ahead ------------------------------------------------------------

    /// (s, v_lim) on a grid from `s0`: cruise, bends, the stop; eased back.
    fn limits_ahead(&self, path: &Path, s0: f64, length: f64) -> (Vec<f64>, Vec<f64>) {
        let ds = 0.25;
        let cruise = self.settings.cruise;
        let reach = cruise * cruise / (2.0 * self.brake);
        let count = ((length + reach) / ds).ceil() as usize + 2;
        let s: Vec<f64> = (0..count).map(|k| s0 + ds * k as f64).collect();
        let mut w: Vec<f64> = s
            .iter()
            .map(|&a| {
                let bend = path.curvature_at(a).abs();
                let mut w = cruise * cruise;
                if bend > 1e-9 {
                    w = w.min(self.settings.lateral_accel / bend.max(1e-9));
                }
                w.min(2.0 * self.brake * (path.length() - a).max(0.0))
            })
            .collect();
        for k in (0..count - 1).rev() {
            w[k] = w[k].min(w[k + 1] + 2.0 * self.brake * ds);
        }
        (s, w.into_iter().map(|w| w.max(0.0).sqrt()).collect())
    }

    // --- the plan -------------------------------------------------------------------------

    /// The last plan shifted by a tick, or holding what was sent.
    fn nominal_inputs(&self, x0: &State, steer_now: f64) -> Vec<Input> {
        let n = self.settings.horizon;
        let mut u = match &self.plan_u {
            None => vec![[steer_now, x0[row::V]]; n],
            Some(plan) => {
                let mut u: Vec<Input> = plan[1..].to_vec();
                u.push(plan[n - 1]);
                u
            }
        };
        let fastest = u.iter().map(|c| c[1]).fold(f64::NEG_INFINITY, f64::max);
        if fastest < self.pi.stopped_speed {
            // A plan that stands still is linearised where the brake holds the car: no
            // gradient to start from. Start from speeding up to cruise; the QP slows it.
            for (k, c) in u.iter_mut().enumerate() {
                c[1] = self
                    .settings
                    .cruise
                    .min(self.settings.accel * self.settings.period * (k + 1) as f64);
            }
        }
        u
    }

    /// The input acting at tick `k` of the plan: the queue's, or the plan's.
    fn effective(&self, u: &[Input], k: usize, steer_buf: &[f64], speed_buf: &[f64]) -> Input {
        let steer = if k < self.d_steer {
            steer_buf[k]
        } else {
            u[k - self.d_steer][0]
        };
        let speed = if k < self.d_speed {
            speed_buf[k]
        } else {
            u[k - self.d_speed][1]
        };
        [steer, speed]
    }

    /// (X_bar, A, B, kappa) along the nominal plan.
    #[allow(clippy::type_complexity)]
    fn linearise(
        &self,
        path: &Path,
        x0: &State,
        u_bar: &[Input],
        bufs: (&[f64], &[f64]),
    ) -> (
        Vec<State>,
        Vec<[[f64; NX]; NX]>,
        Vec<[[f64; NU]; NX]>,
        Vec<f64>,
    ) {
        let n = self.settings.horizon;
        let eps = 1e-4;
        let (period, substeps) = (self.settings.period, self.settings.substeps);
        let mut xs = Vec::with_capacity(n);
        let mut a = Vec::with_capacity(n);
        let mut b = Vec::with_capacity(n);
        let mut kappas = Vec::with_capacity(n);
        let mut x = *x0;
        for k in 0..n {
            let u = self.effective(u_bar, k, bufs.0, bufs.1);
            let kappa = path.curvature_at(x[row::S]);
            let run = |x: &State, u: &Input| {
                model::step(&self.plan_car, &self.pi, x, u, kappa, period, substeps)
            };
            let nominal = run(&x, &u);
            let mut ak = [[0.0; NX]; NX];
            let mut bk = [[0.0; NU]; NX];
            for j in 0..NX {
                let mut xj = x;
                xj[j] += eps;
                let out = run(&xj, &u);
                for i in 0..NX {
                    ak[i][j] = (out[i] - nominal[i]) / eps;
                }
            }
            for j in 0..NU {
                let mut uj = u;
                uj[j] += eps;
                let out = run(&x, &uj);
                for i in 0..NX {
                    bk[i][j] = (out[i] - nominal[i]) / eps;
                }
            }
            a.push(ak);
            b.push(bk);
            kappas.push(kappa);
            xs.push(nominal);
            x = nominal;
        }
        (xs, a, b, kappas)
    }

    /// M (4N x 2N, row k * 4 + c): how each planned input moves the costed states.
    fn condense(&self, a: &[[[f64; NX]; NX]], b: &[[[f64; NU]; NX]]) -> Vec<f64> {
        let n = self.settings.horizon;
        let cols = 2 * n;
        let mut m = vec![0.0; 4 * n * cols];
        let mut acc = vec![0.0; NX * cols];
        let mut next = vec![0.0; NX * cols];
        for k in 0..n {
            for (i, out) in next.chunks_exact_mut(cols).enumerate() {
                out.fill(0.0);
                for (l, &ail) in a[k][i].iter().enumerate() {
                    if ail != 0.0 {
                        let src = &acc[l * cols..(l + 1) * cols];
                        for (o, &s) in out.iter_mut().zip(src) {
                            *o += ail * s;
                        }
                    }
                }
            }
            std::mem::swap(&mut acc, &mut next);
            // Which planned steering and set speed act at k.
            if let Some(j) = k.checked_sub(self.d_steer) {
                for i in 0..NX {
                    acc[i * cols + j] += b[k][i][row::STEER];
                }
            }
            if let Some(j) = k.checked_sub(self.d_speed) {
                for i in 0..NX {
                    acc[i * cols + n + j] += b[k][i][row::SPEED];
                }
            }
            for (c, &r) in COSTED.iter().enumerate() {
                let dst = (k * 4 + c) * cols;
                m[dst..dst + cols].copy_from_slice(&acc[r * cols..(r + 1) * cols]);
            }
        }
        m
    }

    /// One tick: the steering and set speed to send now. `dt`: s since the last tick.
    ///
    /// # Errors
    ///
    /// [`TrackError::NoPath`] before a path; [`TrackError::Qp`] if the QP fails.
    pub fn step(&mut self, seen: &Observed, dt: f64) -> Result<Command, TrackError> {
        let t0 = Instant::now();
        let period = self.settings.period;
        let foot = self.foot(seen.x, seen.y)?;
        let x0 = self.estimate(seen, dt, &foot);
        let steer_buf: Vec<f64> = self.steer_queue.iter().copied().collect();
        let speed_buf: Vec<f64> = self.speed_queue.iter().copied().collect();
        let mut plan = self.nominal_inputs(&x0, seen.steer);
        let (mut iterations, mut solved) = (0, true);
        let (mut feed_forward, mut speed_limit) = (0.0, 0.0);
        for _ in 0..self.settings.sqp {
            let s = self.solve(&x0, &plan, seen.steer, (&steer_buf, &speed_buf))?;
            plan = s.plan;
            iterations += s.iterations;
            solved &= s.solved;
            feed_forward = s.feed_forward;
            speed_limit = s.speed_limit;
        }

        // Exactly within the limits: the steering's lock and rate, the speed's.
        let tick = dt.max(period);
        let steer = self.limited(plan[0][0], seen.steer, tick);
        let wanted = if self.holding { 0.0 } else { plan[0][1] };
        let speed = model::clip(
            wanted,
            self.sent.1 - self.settings.decel * tick,
            self.sent.1 + self.settings.accel * tick,
        );
        let speed = model::clip(speed, 0.0, self.settings.cruise);
        self.sent = (steer, speed);
        self.acting = (
            passed(&mut self.steer_queue, steer),
            passed(&mut self.speed_queue, speed),
        );
        plan[0] = [steer, speed];
        self.plan_u = Some(plan);
        Ok(Command {
            steer,
            speed,
            report: Report {
                solve_ms: t0.elapsed().as_secs_f64() * 1000.0,
                iterations,
                solved,
                wheels_est: x0[row::WHEELS],
                speed_limit,
                feed_forward,
                foot: Some(foot),
            },
        })
    }

    fn solve(
        &mut self,
        x0: &State,
        u_bar: &[Input],
        steer_now: f64,
        bufs: (&[f64], &[f64]),
    ) -> Result<Solved, TrackError> {
        let path = self.path.take().ok_or(TrackError::NoPath)?;
        let out = self.solve_on(&path, x0, u_bar, steer_now, bufs);
        self.path = Some(path);
        out
    }

    fn solve_on(
        &mut self,
        path: &Path,
        x0: &State,
        u_bar: &[Input],
        steer_now: f64,
        bufs: (&[f64], &[f64]),
    ) -> Result<Solved, TrackError> {
        let lim = self.limits;
        let s = self.settings;
        let n = s.horizon;
        let period = s.period;
        let (xs, a, b, kappa) = self.linearise(path, x0, u_bar, bufs);
        let m = self.condense(&a, &b);
        let cols = 2 * n;

        // The targets along the nominal plan.
        let r: Vec<f64> = kappa
            .iter()
            .map(|k| model::clip((lim.wheelbase * k).atan(), -lim.max_steer, lim.max_steer))
            .collect();
        let (grid, v_grid) =
            self.limits_ahead(path, x0[row::S], xs[n - 1][row::S] - x0[row::S] + 1.0);
        let v_lim: Vec<f64> = xs
            .iter()
            .map(|x| interp(x[row::S], &grid, &v_grid))
            .collect();
        let q = [s.q[0], s.q[1], s.q[2], s.q_speed];
        let mut weights = vec![0.0; 4 * n];
        let mut err = vec![0.0; 4 * n];
        for k in 0..n {
            // In the tube the target is where the plan already is: no pull to the centre.
            let e_y = xs[k][row::E_Y];
            let target = [model::clip(e_y, -s.tube, s.tube), 0.0, r[k], v_lim[k]];
            let scale = if k + 1 == n { s.terminal } else { 1.0 };
            for c in 0..4 {
                weights[k * 4 + c] = q[c] * scale;
                err[k * 4 + c] = xs[k][COSTED[c]] - target[c];
            }
        }

        // The cost in z = [dU_steer (N), dU_speed (N), slack (N)].
        let nz = 3 * n;
        let mut p = vec![0.0; nz * nz];
        let mut qv = vec![0.0; nz];
        // M' W M and M' W err, on the first 2N.
        for (rw, (&w, &e)) in weights.iter().zip(&err).enumerate() {
            if w == 0.0 {
                continue;
            }
            let mr = &m[rw * cols..(rw + 1) * cols];
            for (i, &mi) in mr.iter().enumerate() {
                if mi == 0.0 {
                    continue;
                }
                qv[i] += mi * w * e;
                let wmi = w * mi;
                let prow = &mut p[i * nz..i * nz + cols];
                for (pj, &mj) in prow.iter_mut().zip(mr) {
                    *pj += wmi * mj;
                }
            }
        }
        // The steering near the bend's own (where it will act), and smooth.
        let us: Vec<f64> = u_bar.iter().map(|c| c[0]).collect();
        let cs: Vec<f64> = u_bar.iter().map(|c| c[1]).collect();
        let r_cmd: Vec<f64> = (0..n).map(|k| r[(k + self.d_steer).min(n - 1)]).collect();
        // D u - e: the change each tick, the first from what is now.
        let du_s = diff_minus(&us, steer_now);
        let du_c = diff_minus(&cs, self.sent.1);
        let dt_s = diff_t(&du_s);
        let dt_c = diff_t(&du_c);
        for i in 0..n {
            let (si, ci) = (i, n + i);
            p[si * nz + si] += s.r + s.r_rate * dtd(i, i, n);
            p[ci * nz + ci] += s.r_speed * dtd(i, i, n);
            if i + 1 < n {
                for (a, b) in [(i, i + 1), (i + 1, i)] {
                    p[a * nz + b] += s.r_rate * dtd(a, b, n);
                    p[(n + a) * nz + n + b] += s.r_speed * dtd(a, b, n);
                }
            }
            qv[si] += s.r * (us[i] - r_cmd[i]) + s.r_rate * dt_s[i];
            qv[ci] += s.r_speed * dt_c[i];
            let k = 2 * n + i;
            p[k * nz + k] = s.w_slack;
        }

        // The constraints: lo <= A z <= hi.
        let mut am = RowMatrix::new(nz);
        let mut lo = Vec::with_capacity(6 * n);
        let mut hi = Vec::with_capacity(6 * n);
        let step_s = lim.max_steer_rate * period;
        let d_row = |am: &mut RowMatrix, base: usize, i: usize| {
            if i == 0 {
                am.push_row([(base, 1.0)]);
            } else {
                am.push_row([(base + i - 1, -1.0), (base + i, 1.0)]);
            }
        };
        for (i, &u) in us.iter().enumerate() {
            am.push_row([(i, 1.0)]); // full lock
            lo.push(-lim.max_steer - u);
            hi.push(lim.max_steer - u);
        }
        for (i, &d) in du_s.iter().enumerate() {
            d_row(&mut am, 0, i); // the steering rate
            lo.push(-step_s - d);
            hi.push(step_s - d);
        }
        for (i, &c) in cs.iter().enumerate() {
            am.push_row([(n + i, 1.0)]); // 0 <= c <= cruise
            lo.push(-c);
            hi.push(s.cruise - c);
        }
        for (i, &d) in du_c.iter().enumerate() {
            d_row(&mut am, n, i); // accel, decel
            lo.push(-s.decel * period - d);
            hi.push(s.accel * period - d);
        }
        for i in 0..n {
            am.push_row([(2 * n + i, 1.0)]); // slack >= 0
            lo.push(0.0);
            hi.push(f64::INFINITY);
        }
        for k in 0..n {
            // v - slack <= v_lim, v = v_bar + M_v dU.
            let mv = &m[(k * 4 + 3) * cols..(k * 4 + 4) * cols];
            am.push_row(
                mv.iter()
                    .copied()
                    .enumerate()
                    .chain(std::iter::once((2 * n + k, -1.0))),
            );
            lo.push(f64::NEG_INFINITY);
            hi.push(v_lim[k] - xs[k][row::V]);
        }
        for (h, &l) in hi.iter_mut().zip(&lo) {
            *h = h.max(l);
        }

        let warm = self.warm_y.as_deref().filter(|y| y.len() == lo.len());
        let settings = qp::Settings {
            eps: 1e-4,
            max_iter: s.max_iter,
            rho: s.rho,
            ..qp::Settings::default()
        };
        let zeros = vec![0.0; nz];
        let res = qp::solve(&p, &qv, &am, &lo, &hi, Some(&zeros), warm, &settings)?;
        let plan: Vec<Input> = u_bar
            .iter()
            .enumerate()
            .map(|(k, u)| [u[0] + res.z[k], u[1] + res.z[n + k]])
            .collect();
        // What the plan expects (e_y, e_theta, wheels, v per tick), for checking the model.
        self.predicted = (0..n)
            .map(|k| {
                let mut out = [0.0; 4];
                for (c, o) in out.iter_mut().enumerate() {
                    let mr = &m[(k * 4 + c) * cols..(k * 4 + c + 1) * cols];
                    *o = xs[k][COSTED[c]] + mr.iter().zip(&res.z).map(|(a, b)| a * b).sum::<f64>();
                }
                out
            })
            .collect();
        self.warm_y = Some(res.y);
        Ok(Solved {
            plan,
            iterations: res.iterations,
            solved: res.solved,
            feed_forward: r[0],
            speed_limit: v_lim[0],
        })
    }
}

/// Queue `newest`; return the command that acts over the next tick.
fn passed(queue: &mut VecDeque<f64>, newest: f64) -> f64 {
    queue.push_back(newest);
    queue.pop_front().unwrap_or(newest)
}

/// `D u - e`: each tick's change, the first from `now`.
fn diff_minus(u: &[f64], now: f64) -> Vec<f64> {
    u.iter()
        .enumerate()
        .map(|(i, &v)| v - if i == 0 { now } else { u[i - 1] })
        .collect()
}

/// `D' v`: `v[i] - v[i + 1]`, the last alone.
fn diff_t(v: &[f64]) -> Vec<f64> {
    (0..v.len())
        .map(|i| v[i] - v.get(i + 1).copied().unwrap_or(0.0))
        .collect()
}

/// `(D' D)[i][j]`: 2 on the diagonal (1 at the end), -1 beside it.
fn dtd(i: usize, j: usize, n: usize) -> f64 {
    if i == j {
        if i + 1 < n {
            2.0
        } else {
            1.0
        }
    } else if i.abs_diff(j) == 1 {
        -1.0
    } else {
        0.0
    }
}

/// numpy's `interp`: linear between the grid's points, its end values beyond.
fn interp(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    let last = xp.len() - 1;
    if x <= xp[0] {
        return fp[0];
    }
    if x >= xp[last] {
        return fp[last];
    }
    let i = xp.partition_point(|&a| a <= x) - 1;
    let t = (x - xp[i]) / (xp[i + 1] - xp[i]);
    fp[i] + t * (fp[i + 1] - fp[i])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interp_is_numpys() {
        let (xp, fp) = ([0.0, 1.0, 2.0], [0.0, 10.0, 0.0]);
        assert_eq!(interp(-1.0, &xp, &fp), 0.0);
        assert_eq!(interp(0.5, &xp, &fp), 5.0);
        assert_eq!(interp(1.5, &xp, &fp), 5.0);
        assert_eq!(interp(3.0, &xp, &fp), 0.0);
    }

    #[test]
    fn the_rate_cost_is_d_transpose_d() {
        let n = 4;
        // D: 1 on the diagonal, -1 below it.
        let d = |i: usize, j: usize| {
            if i == j {
                1.0
            } else if i == j + 1 {
                -1.0
            } else {
                0.0
            }
        };
        for i in 0..n {
            for j in 0..n {
                let want: f64 = (0..n).map(|k| d(k, i) * d(k, j)).sum();
                assert_eq!(dtd(i, j, n), want);
            }
        }
        let v = [1.0, 3.0, 6.0, 10.0];
        assert_eq!(diff_minus(&v, 0.5), vec![0.5, 2.0, 3.0, 4.0]);
        assert_eq!(diff_t(&v), vec![-2.0, -3.0, -4.0, 10.0]);
    }

    #[test]
    fn a_queue_passes_the_oldest() {
        let mut q = VecDeque::from([1.0, 2.0]);
        assert_eq!(passed(&mut q, 3.0), 1.0);
        assert_eq!(q, VecDeque::from([2.0, 3.0]));
        let mut empty = VecDeque::new();
        assert_eq!(passed(&mut empty, 7.0), 7.0);
    }
}

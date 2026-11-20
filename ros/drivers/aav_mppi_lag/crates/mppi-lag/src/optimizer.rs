//! MPPI: Nav2's Optimizer (nav2_mppi_controller `optimizer.cpp`), on any [`CarModel`].
//!
//! Each tick, for `iteration_count` rounds:
//!
//! 1. **Sample**: `batch_size` copies of the control sequence, each step's speed and
//!    steering plus normal noise (`speed_std`, `steer_std`), held to the speed range
//!    and full lock. Ours can correlate each sample's noise from step to step
//!    (`noise_correlation`, AR(1); Nav2's is white, 0): a lagging actuator smooths white
//!    noise away, so its samples would all drive alike and MPPI would learn nothing
//!    (colored-noise MPPI; see docs/mppi-lag.md). Ego's ramps are not clipped here: they are part of the model
//!    ([`crate::ego::Driven`]). Above `steer_std_speed` the steering noise shrinks with
//!    the speed squared ([`Settings::steer_std_at`]), so the futures spread as far
//!    sideways at 2.5 m/s as at 1 m/s (ours; Nav2's wz noise shrinks as 1 / speed).
//! 2. **Roll out**: each sample through the model from the car's state now: its pose
//!    and speed, ego's output, the actuators' levels and the commands still in their
//!    dead time. This is where the lag comes in; Nav2's own model has none. Split across
//!    threads.
//! 3. **Score**: the critics, plus Nav2's control cost gamma x u' Σ⁻¹ (u_sample - u),
//!    Σ the noise's covariance ([`control_cost`]). White noise (Nav2's): Σ⁻¹ = 1 / std²,
//!    so gamma / std² x sum(u x (u_sample - u)), Nav2's own. Correlated noise: Σ⁻¹ is
//!    AR(1)'s, so the cost sees only each step's fresh part of the noise. Until 14 Nov
//!    ours used the white form on correlated noise: its spread grew 3x (at 0.8) and
//!    swamped the critics, so MPPI pulled the speed and steering towards 0 and the car
//!    stood still beside a cusp (docs/mppi-lag.md, Cusps).
//!
//!    `sampling: smooth` (TODO 212, a challenger; SMPPI's idea, Kim et al. 2022, as in
//!    `pytorch_mppi`, MIT): the steering's noise is drawn on its **change** a step
//!    (`steer_rate_std` rad/s) and summed, a random walk, so each sample's steering moves
//!    a little a step instead of jumping. The commands come out smooth without a larger
//!    control cost; the cost uses the random walk's own Σ⁻¹ ([`control_cost`]).
//! 4. **Weigh**: softmax of -(cost - min) / temperature; the new sequence is the
//!    weighted mean of the samples, held to the limits again.
//!
//! Then the sequence is smoothed (Savitzky-Golay, as Nav2), its first command sent, and
//! it is shifted one step for the next tick (model_dt must equal the tick period, as in
//! Nav2's `shift_control_sequence`). Nav2 sends element 1, counting element 0 as already
//! running; ours sends element 0, because the model's state already holds every command
//! sent so far (the dead times and lags carry it).

use crate::costmap::Costmap;
use crate::critics::{furthest_reached, path_clear, Critic, CriticData, Prepared, Rollout};
use crate::lane::Lane;
use crate::model::{CarModel, Command, Pose};
use crate::rng::Rng;
use crate::smoothing::{savitzky_golay, History};

/// How each sample's steering noise is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Sampling {
    /// AR(1) noise on the steering itself (`noise_correlation`): the champion.
    #[default]
    Correlated,
    /// White noise on the steering's change a step, summed (a random walk): SMPPI's idea
    /// (TODO 212), a challenger.
    Smooth,
}

impl std::str::FromStr for Sampling {
    type Err = UnknownSampling;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim() {
            "" | "correlated" => Ok(Self::Correlated),
            "smooth" => Ok(Self::Smooth),
            other => Err(UnknownSampling(other.to_owned())),
        }
    }
}

/// A `sampling` that is neither `correlated` nor `smooth`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownSampling(pub String);

impl std::fmt::Display for UnknownSampling {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "sampling {:?}: correlated or smooth", self.0)
    }
}

impl std::error::Error for UnknownSampling {}

/// MPPI's settings: Nav2's names where Nav2 has them.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// Steps ahead (Nav2's 56).
    pub time_steps: usize,
    /// s per step: must equal the tick period.
    pub model_dt: f64,
    /// Samples a round.
    pub batch_size: usize,
    /// Rounds a tick.
    pub iteration_count: usize,
    /// Lower: the best samples count for more.
    pub temperature: f64,
    /// The control cost's weight.
    pub gamma: f64,
    /// The steering's control cost weight, if not `gamma` (None: `gamma`, as Nav2). The
    /// cost leans each sample toward smaller commands; on the steering alone it damps the
    /// sampling jitter (TODO 168, docs/mppi-lag.md "The jitter that was left"), where a
    /// larger `gamma` also slowed the car and dithered at cusps.
    pub steer_gamma: Option<f64>,
    /// m/s: `steer_gamma` holds from this speed up; slower, the weight eases back toward
    /// `gamma` in proportion (at a stop, `gamma`), so a cusp and a backing turn steer as
    /// before (0: `steer_gamma` at every speed).
    pub steer_gamma_speed: f64,
    /// m/s: the speed noise's spread (Nav2's vx_std).
    pub speed_std: f64,
    /// rad: the steering noise's spread (Nav2 samples wz instead; ours samples the angle),
    /// up to `steer_std_speed`.
    pub steer_std: f64,
    /// m/s: faster than this, the steering noise shrinks as (steer_std_speed / speed)², so
    /// the sampled futures spread no further sideways as the car speeds up (a steering
    /// offset moves a future's end by v² x the angle). 0: `steer_std` at every speed, as
    /// until 15 Nov, when the softmax rested on a handful of futures, each tick a
    /// different handful, and the steering wiggled (TODO 168, docs/mppi-lag.md).
    pub steer_std_speed: f64,
    /// rad: the steering noise never shrinks below this. With less, MPPI can't move its
    /// steering fast enough to follow a bend (at 2.5 m/s, 0.009 rad cut a 5 m corner by
    /// 0.5 m).
    pub steer_std_min: f64,
    /// 0..1: how much each step's noise carries the last step's (AR(1); 0: white, as
    /// Nav2). The spread stays `speed_std` and `steer_std` either way.
    pub noise_correlation: f64,
    /// How the steering's noise is drawn: on the steering (the champion) or on its change
    /// (`Smooth`, TODO 212).
    pub sampling: Sampling,
    /// rad/s: with `Smooth`, the spread of the steering's change (x `model_dt` a step;
    /// shrunk with the speed as `steer_std` is). 0.5: 0.025 rad the first step, 0.19 at
    /// the horizon's end (2.8 s).
    pub steer_rate_std: f64,
    /// m/s: the fastest command.
    pub vx_max: f64,
    /// m/s: the slowest (0: forward only).
    pub vx_min: f64,
    /// rad: full lock.
    pub max_steer: f64,
    /// Rounds of reset and retry when every sample collides.
    pub retry_attempt_limit: usize,
    /// Threads for the rollouts and critics (0: one per core).
    pub threads: usize,
    /// The random seed: the same seed, the same samples.
    pub seed: u64,
}

impl Default for Settings {
    /// Our nav2.yaml's MPPI block, and correlated noise (0.8: about 0.25 s; see
    /// `noise_correlation`).
    fn default() -> Self {
        Self {
            time_steps: 56,
            model_dt: 0.05,
            batch_size: 2000,
            iteration_count: 1,
            temperature: 0.15,
            gamma: 0.015,
            steer_gamma: None,
            steer_gamma_speed: 1.0,
            speed_std: 0.4,
            steer_std: 0.15,
            steer_std_speed: 1.0,
            steer_std_min: 0.024,
            noise_correlation: 0.8,
            sampling: Sampling::Correlated,
            steer_rate_std: 0.5,
            vx_max: 1.0,
            vx_min: 0.0,
            max_steer: 0.5,
            retry_attempt_limit: 1,
            threads: 0,
            seed: 42,
        }
    }
}

impl Settings {
    /// rad: the steering noise's spread with the car at `speed` m/s (either way):
    /// `steer_std` up to `steer_std_speed`, then x (steer_std_speed / |speed|)², but never
    /// under `steer_std_min` (nor over `steer_std`).
    #[must_use]
    pub fn steer_std_at(&self, speed: f64) -> f64 {
        let fast = speed.abs();
        if self.steer_std_speed > 0.0 && fast > self.steer_std_speed {
            let shrunk = self.steer_std * (self.steer_std_speed / fast).powi(2);
            shrunk.max(self.steer_std_min.min(self.steer_std))
        } else {
            self.steer_std
        }
    }

    /// The steering's control cost weight with the car at `speed` m/s (either way):
    /// `steer_gamma` from `steer_gamma_speed` up, easing linearly to `gamma` at a stop;
    /// `gamma` when there is no `steer_gamma`.
    #[must_use]
    pub fn steer_gamma_at(&self, speed: f64) -> f64 {
        let Some(steer) = self.steer_gamma else {
            return self.gamma;
        };
        let share = if self.steer_gamma_speed > 0.0 {
            (speed.abs() / self.steer_gamma_speed).min(1.0).powi(2)
        } else {
            1.0
        };
        self.gamma + (steer - self.gamma) * share
    }
}

/// MPPI's control cost of one sample: gamma x u' Σ⁻¹ (sample - u), summed over speed and
/// steering, with Σ the covariance of the noise the samples were drawn with.
///
/// The noise is AR(1) with correlation a and spread std (white for a = 0), so
/// Σ⁻¹ = 1 / (std² (1 - a²)) x tridiag(-a, [1, 1 + a², ..., 1 + a², 1], -a), and
///
/// ```text
/// u' Σ⁻¹ e = ((1 - a²) u₀ e₀ + Σₜ (uₜ - a uₜ₋₁)(eₜ - a eₜ₋₁)) / (std² (1 - a²))
/// ```
///
/// For a = 0 that is Nav2's gamma / std² x Σ u e. The white form on correlated noise
/// weighs each sample's whole drift, not its fresh part: at a = 0.8 its spread is 3x
/// Nav2's (9x the right one), and it outweighed the critics.
#[must_use]
pub fn control_cost(settings: &Settings, sequence: &[Command], sample: &[Command]) -> f64 {
    let a = settings.noise_correlation.clamp(0.0, 0.999);
    let steer_gamma = settings.steer_gamma.unwrap_or(settings.gamma);
    let speed = ar1_cost(sequence, sample, |c| c.speed, a, settings.speed_std);
    let steer = match settings.sampling {
        Sampling::Correlated => ar1_cost(sequence, sample, |c| c.steer, a, settings.steer_std),
        Sampling::Smooth => walk_cost(
            sequence,
            sample,
            |c| c.steer,
            settings.steer_rate_std * settings.model_dt,
        ),
    };
    settings.gamma * speed + steer_gamma * steer
}

/// u' Σ⁻¹ e for AR(1) noise with correlation `a` and spread `std` (see [`control_cost`]).
fn ar1_cost(
    sequence: &[Command],
    sample: &[Command],
    part: fn(&Command) -> f64,
    a: f64,
    std: f64,
) -> f64 {
    let fresh = 1.0 - a * a;
    let pairs = || sequence.iter().zip(sample);
    let first = pairs()
        .next()
        .map_or(0.0, |(u, c)| fresh * part(u) * (part(c) - part(u)));
    let rest: f64 = pairs()
        .zip(pairs().skip(1))
        .map(|((u0, c0), (u1, c1))| {
            let (e0, e1) = (part(c0) - part(u0), part(c1) - part(u1));
            (part(u1) - a * part(u0)) * (e1 - a * e0)
        })
        .sum();
    (first + rest) / (std * std * fresh)
}

/// u' Σ⁻¹ e for a random walk, white steps of spread `step` (TODO 212): Σ = step² L L',
/// L the lower triangle of ones, so Σ⁻¹ = D' D / step² with D the first difference, and
///
/// ```text
/// u' Σ⁻¹ e = (u₀ e₀ + Σₜ (uₜ - uₜ₋₁)(eₜ - eₜ₋₁)) / step²
/// ```
///
/// The cost sees only each step's fresh change, as the AR(1) one does.
fn walk_cost(
    sequence: &[Command],
    sample: &[Command],
    part: fn(&Command) -> f64,
    step: f64,
) -> f64 {
    let pairs = || sequence.iter().zip(sample);
    let first = pairs()
        .next()
        .map_or(0.0, |(u, c)| part(u) * (part(c) - part(u)));
    let rest: f64 = pairs()
        .zip(pairs().skip(1))
        .map(|((u0, c0), (u1, c1))| {
            let (e0, e1) = (part(c0) - part(u0), part(c1) - part(u1));
            (part(u1) - part(u0)) * (e1 - e0)
        })
        .sum();
    (first + rest) / (step * step)
}

/// One sample: `sequence` plus noise, into `controls` (not yet held to the limits). The
/// speed's noise is AR(1); the steering's is AR(1) too (`Correlated`), or a random walk of
/// white steps `steer_rate_std x model_dt` (`Smooth`). One draw a step for each, in the
/// same order either way.
pub(crate) fn sample_into(
    settings: &Settings,
    rng: &mut Rng,
    sequence: &[Command],
    controls: &mut [Command],
) {
    let a = settings.noise_correlation.clamp(0.0, 0.999);
    let fresh = (1.0 - a * a).sqrt();
    let step = settings.steer_rate_std * settings.model_dt;
    let (mut nv, mut ns) = (rng.normal(), rng.normal());
    let mut walk = 0.0;
    for (c, base) in controls.iter_mut().zip(sequence) {
        let steer = match settings.sampling {
            Sampling::Correlated => settings.steer_std * ns,
            Sampling::Smooth => {
                walk += step * ns;
                walk
            }
        };
        *c = Command::new(base.speed + settings.speed_std * nv, base.steer + steer);
        nv = a * nv + fresh * rng.normal();
        ns = match settings.sampling {
            Sampling::Correlated => a * ns + fresh * rng.normal(),
            Sampling::Smooth => rng.normal(),
        };
    }
}

/// Every sample ran into something, even after a reset (Nav2's NoValidControl).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoValidControl;

impl std::fmt::Display for NoValidControl {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("every sampled future collides")
    }
}

impl std::error::Error for NoValidControl {}

/// What one tick chose, and how the samples did.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    /// The command to send now.
    pub command: Command,
    /// The cheapest sample's cost.
    pub best_cost: f64,
    /// Share of samples that collided (0..1).
    pub collided: f64,
    /// The poses the chosen sequence leads to, through the model.
    pub poses: Vec<Pose>,
}

/// What the critics and samples need to know this tick.
#[derive(Debug, Clone, Copy)]
pub struct Scene<'a> {
    /// The path ahead (the window).
    pub path: &'a [Pose],
    /// The path's end.
    pub goal: Pose,
    /// The local costmap, if any.
    pub costmap: Option<&'a Costmap>,
    /// The car's outline, padded (base_link points).
    pub footprint: &'a [(f64, f64)],
    /// Unknown cells are drivable.
    pub tracking_unknown: bool,
    /// The road ahead's lane, in the poses' frame (odom), if one is known: for `LaneCritic`.
    pub lane: Option<&'a Lane>,
}

/// MPPI over a car model.
#[derive(Debug, Clone)]
pub struct Optimizer<M: CarModel> {
    model: M,
    settings: Settings,
    critics: Vec<Critic>,
    sequence: Vec<Command>,
    history: History,
    last: Command,
    rng: Rng,
    rollouts: Vec<Rollout>,
}

impl<M: CarModel> Optimizer<M> {
    /// An optimizer for `model`, with these settings and critics.
    pub fn new(model: M, settings: Settings, critics: Vec<Critic>) -> Self {
        let steps = settings.time_steps.max(1);
        let rollouts = vec![Rollout::with_steps(steps); settings.batch_size.max(1)];
        Self {
            model,
            rng: Rng::new(settings.seed),
            sequence: vec![Command::default(); steps],
            history: [Command::default(); 4],
            last: Command::default(),
            critics,
            settings,
            rollouts,
        }
    }

    /// The car model.
    pub fn model(&self) -> &M {
        &self.model
    }

    /// The settings.
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The control sequence as it stands.
    pub fn sequence(&self) -> &[Command] {
        &self.sequence
    }

    /// Forget the plan: hold `last`, the command the car has now (Nav2 starts from zeros;
    /// a car taking over mid-drive would brake first).
    pub fn reset(&mut self, last: Command) {
        self.sequence.fill(last);
        self.history = [last; 4];
        self.last = last;
    }

    /// Hold `controls` to the speed range and full lock (Nav2's
    /// applyControlSequenceConstraints, without the steps' limits: ego's ramps are in the
    /// model).
    pub fn constrain(settings: &Settings, controls: &mut [Command]) {
        for c in controls.iter_mut() {
            c.speed = c.speed.clamp(settings.vx_min, settings.vx_max);
            c.steer = c.steer.clamp(-settings.max_steer, settings.max_steer);
        }
    }

    fn threads(&self) -> usize {
        match self.settings.threads {
            0 => std::thread::available_parallelism().map_or(1, |n| n.get()),
            n => n,
        }
    }

    /// Sample, roll out and score one round; return the samples' costs.
    fn round(&mut self, state: &M::State, scene: &Scene) {
        let threads = self.threads().min(self.rollouts.len()).max(1);
        let chunk = self.rollouts.len().div_ceil(threads);
        let seeds: Vec<u64> = (0..threads).map(|_| self.rng.next_u64()).collect();
        // This round's steering noise, for the car's speed now; the control cost weighs
        // the samples by the same spread they were drawn with.
        // The steering's control cost weight, too, for the speed now.
        let speed = self.model.speed(state);
        let shrink = if self.settings.steer_std > 0.0 {
            self.settings.steer_std_at(speed) / self.settings.steer_std
        } else {
            1.0
        };
        let now = Settings {
            steer_std: self.settings.steer_std_at(speed),
            steer_rate_std: self.settings.steer_rate_std * shrink,
            steer_gamma: Some(self.settings.steer_gamma_at(speed)),
            ..self.settings.clone()
        };
        let (model, settings, sequence) = (&self.model, &now, &self.sequence);

        // 1 and 2: sample and roll out.
        {
            span!("mppi.rollout");
            std::thread::scope(|scope| {
                for (part, seed) in self.rollouts.chunks_mut(chunk).zip(&seeds) {
                    scope.spawn(move || {
                        let mut rng = Rng::new(*seed);
                        for r in part {
                            sample_into(settings, &mut rng, sequence, &mut r.controls);
                            Self::constrain(settings, &mut r.controls);
                            let mut s = state.clone();
                            for (t, c) in r.controls.iter().enumerate() {
                                model.step(&mut s, *c, settings.model_dt);
                                r.poses[t] = model.pose(&s);
                                r.speeds[t] = model.speed(&s);
                                r.steerings[t] = model.steering(&s);
                            }
                        }
                    });
                }
            });
        }

        // 3: the critics, on what this round's samples reached.
        span!("mppi.critics");
        let furthest = furthest_reached(scene.path, &self.rollouts);
        let valid = path_clear(scene.path, scene.costmap, scene.tracking_unknown);
        let data = CriticData {
            pose: model.pose(state),
            goal: scene.goal,
            path: scene.path,
            furthest,
            valid: &valid,
            dt: settings.model_dt,
            wheelbase: model.wheelbase(),
            costmap: scene.costmap,
            footprint: scene.footprint,
            vx_max: settings.vx_max,
            vx_min: settings.vx_min,
            min_turning_r: model.wheelbase() / settings.max_steer.tan(),
            lane: scene.lane,
        };
        let prepared = Prepared::new(&self.critics, data);
        std::thread::scope(|scope| {
            for part in self.rollouts.chunks_mut(chunk) {
                let prepared = &prepared;
                scope.spawn(move || {
                    for r in part {
                        let score = prepared.score(r);
                        let control = control_cost(settings, sequence, &r.controls);
                        r.cost = score.cost + control;
                        r.collided = score.collided;
                    }
                });
            }
        });
    }

    /// 4: weigh the samples into a new sequence.
    fn update(&mut self) {
        span!("mppi.weigh");
        let min = self
            .rollouts
            .iter()
            .map(|r| r.cost)
            .fold(f64::INFINITY, f64::min);
        let weights: Vec<f64> = self
            .rollouts
            .iter()
            .map(|r| (-(r.cost - min) / self.settings.temperature).exp())
            .collect();
        let total: f64 = weights.iter().sum();
        for (t, c) in self.sequence.iter_mut().enumerate() {
            let (mut v, mut s) = (0.0, 0.0);
            for (r, w) in self.rollouts.iter().zip(&weights) {
                v += w * r.controls[t].speed;
                s += w * r.controls[t].steer;
            }
            *c = Command::new(v / total, s / total);
        }
        Self::constrain(&self.settings, &mut self.sequence);
    }

    /// One tick: the command to send now, from the car's state `state`.
    ///
    /// # Errors
    ///
    /// [`NoValidControl`] if every sample ran into something, even after a reset.
    pub fn evaluate(&mut self, state: &M::State, scene: &Scene) -> Result<Plan, NoValidControl> {
        span!("mppi.tick");
        let mut attempts = 0;
        loop {
            for _ in 0..self.settings.iteration_count.max(1) {
                self.round(state, scene);
                self.update();
            }
            if self.rollouts.iter().any(|r| !r.collided) {
                break;
            }
            attempts += 1;
            self.reset(self.last);
            if attempts > self.settings.retry_attempt_limit {
                return Err(NoValidControl);
            }
        }
        let best_cost = self
            .rollouts
            .iter()
            .map(|r| r.cost)
            .fold(f64::INFINITY, f64::min);
        let collided =
            self.rollouts.iter().filter(|r| r.collided).count() as f64 / self.rollouts.len() as f64;

        {
            span!("mppi.smooth");
            savitzky_golay(&mut self.sequence, &mut self.history);
        }
        let command = self.sequence[0];
        let mut s = state.clone();
        let poses = self
            .sequence
            .iter()
            .map(|c| {
                self.model.step(&mut s, *c, self.settings.model_dt);
                self.model.pose(&s)
            })
            .collect();

        self.last = command;
        self.sequence.rotate_left(1);
        let n = self.sequence.len();
        if n >= 2 {
            self.sequence[n - 1] = self.sequence[n - 2];
        }
        Ok(Plan {
            command,
            best_cost,
            collided,
            poses,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ego::{Driven, Ego};
    use crate::ideal::IdealModel;
    use crate::lag::LagModel;

    fn critics() -> Vec<Critic> {
        crate::critics::NAMES
            .iter()
            .filter_map(|n| Critic::by_name(n))
            .collect()
    }

    fn straight() -> Vec<Pose> {
        (0..=60)
            .map(|i| Pose::new(f64::from(i) * 0.05, 0.0, 0.0))
            .collect()
    }

    /// u' Σ⁻¹ e the long way: Σ = std² a^|i-j| (AR(1)), solved by Gaussian elimination.
    fn brute_force(u: &[f64], e: &[f64], std: f64, a: f64) -> f64 {
        let n = u.len();
        let mut m: Vec<Vec<f64>> = (0..n)
            .map(|i| {
                let mut row: Vec<f64> = (0..n)
                    .map(|j| std * std * a.powi((i as i32 - j as i32).abs()))
                    .collect();
                row.push(e[i]);
                row
            })
            .collect();
        for col in 0..n {
            let pivot = m[col].clone();
            for row in m.iter_mut().skip(col + 1) {
                let f = row[col] / pivot[col];
                for (cell, p) in row.iter_mut().zip(&pivot).skip(col) {
                    *cell -= f * p;
                }
            }
        }
        let mut x = vec![0.0; n];
        for row in (0..n).rev() {
            let known: f64 = (row + 1..n).map(|k| m[row][k] * x[k]).sum();
            x[row] = (m[row][n] - known) / m[row][row];
        }
        u.iter().zip(&x).map(|(a, b)| a * b).sum()
    }

    #[test]
    fn control_cost_is_u_times_the_noises_precision_times_e() {
        let u = [0.5, 0.6, 0.4, -0.2, 0.1];
        let e = [0.1, -0.3, 0.2, 0.05, -0.1];
        let sequence: Vec<Command> = u.iter().map(|&v| Command::new(v, 0.0)).collect();
        let sample: Vec<Command> = u
            .iter()
            .zip(&e)
            .map(|(&v, &n)| Command::new(v + n, 0.0))
            .collect();
        for a in [0.0, 0.5, 0.8] {
            let s = Settings {
                noise_correlation: a,
                ..Settings::default()
            };
            let want = s.gamma * brute_force(&u, &e, s.speed_std, a);
            let got = control_cost(&s, &sequence, &sample);
            assert!((got - want).abs() < 1e-12, "a {a}: {got} vs {want}");
        }
        // White noise: Nav2's own, gamma / std² x sum(u e).
        let white = Settings {
            noise_correlation: 0.0,
            ..Settings::default()
        };
        let nav2: f64 = u.iter().zip(&e).map(|(u, e)| u * e).sum::<f64>() * white.gamma
            / white.speed_std.powi(2);
        assert!((control_cost(&white, &sequence, &sample) - nav2).abs() < 1e-12);
    }

    #[test]
    fn steer_gamma_holds_from_its_speed_and_eases_to_gamma_at_a_stop() {
        let s = Settings {
            steer_gamma: Some(0.05),
            ..Settings::default()
        };
        for v in [1.0, -1.0, 2.5] {
            assert!((s.steer_gamma_at(v) - 0.05).abs() < 1e-12, "{v}");
        }
        assert!((s.steer_gamma_at(0.0) - s.gamma).abs() < 1e-12);
        // Half the speed, a quarter of the way.
        let half = s.gamma + 0.25 * (0.05 - s.gamma);
        assert!((s.steer_gamma_at(-0.5) - half).abs() < 1e-12);
        // None: gamma at every speed; 0 m/s: steer_gamma at every speed.
        assert_eq!(Settings::default().steer_gamma_at(2.0), s.gamma);
        let flat = Settings {
            steer_gamma_speed: 0.0,
            ..s.clone()
        };
        assert!((flat.steer_gamma_at(0.1) - 0.05).abs() < 1e-12);
    }

    #[test]
    fn steer_gamma_weighs_the_steering_alone() {
        let u = [0.5, 0.6, 0.4, -0.2, 0.1];
        let e = [0.1, -0.3, 0.2, 0.05, -0.1];
        let sequence: Vec<Command> = u.iter().map(|&v| Command::new(v, 0.5 * v)).collect();
        let sample: Vec<Command> = u
            .iter()
            .zip(&e)
            .map(|(&v, &n)| Command::new(v + n, 0.5 * (v + n)))
            .collect();
        let nav2 = Settings::default();
        let calm = Settings {
            steer_gamma: Some(0.05),
            ..Settings::default()
        };
        let a = nav2.noise_correlation;
        let speed = nav2.gamma * brute_force(&u, &e, nav2.speed_std, a);
        let half = |v: &[f64; 5]| v.map(|x| 0.5 * x);
        let steer = brute_force(&half(&u), &half(&e), nav2.steer_std, a);
        let got = control_cost(&calm, &sequence, &sample);
        assert!((got - (speed + 0.05 * steer)).abs() < 1e-12, "{got}");
        // None: the steering's weight is gamma, as Nav2's.
        let same = control_cost(&nav2, &sequence, &sample);
        assert!(
            (same - (speed + nav2.gamma * steer)).abs() < 1e-12,
            "{same}"
        );
    }

    #[test]
    fn steering_noise_keeps_the_same_sideways_spread_above_its_speed() {
        let s = Settings::default();
        // Up to 1 m/s, either way: steer_std as it is (backing up keeps it).
        for v in [0.0, 0.5, -0.5, 1.0, -1.0] {
            assert_eq!(s.steer_std_at(v), s.steer_std, "{v}");
        }
        // Faster: v² x the spread stays what it is at 1 m/s ...
        for v in [1.5, 2.0, -2.0, 2.5] {
            let spread = v * v * s.steer_std_at(v);
            assert!((spread - s.steer_std).abs() < 1e-12, "{v}: {spread}");
        }
        assert!((s.steer_std_at(2.5) - 0.024).abs() < 1e-12);
        // ... down to the floor (from 2.5 m/s).
        for v in [3.0, -3.0, 5.0] {
            assert!((s.steer_std_at(v) - s.steer_std_min).abs() < 1e-12, "{v}");
        }
        // 0: the same spread at every speed, as before.
        let flat = Settings {
            steer_std_speed: 0.0,
            ..Settings::default()
        };
        assert_eq!(flat.steer_std_at(2.5), flat.steer_std);
    }

    #[test]
    fn constrain_holds_the_range_and_full_lock() {
        let s = Settings::default();
        let mut c = vec![
            Command::new(5.0, 1.0),
            Command::new(-1.0, -1.0),
            Command::new(0.5, 0.1),
        ];
        Optimizer::<IdealModel>::constrain(&s, &mut c);
        assert_eq!(
            c,
            vec![
                Command::new(1.0, 0.5),
                Command::new(0.0, -0.5),
                Command::new(0.5, 0.1)
            ]
        );
    }

    #[test]
    fn drives_forward_along_a_straight_path() {
        let model = Driven {
            ego: Ego::default(),
            car: LagModel::default(),
        };
        let settings = Settings {
            batch_size: 500,
            threads: 2,
            ..Settings::default()
        };
        let mut opt = Optimizer::new(model, settings, critics());
        let path = straight();
        let scene = Scene {
            path: &path,
            goal: Pose::new(30.0, 0.0, 0.0),
            costmap: None,
            footprint: &[],
            tracking_unknown: false,
            lane: None,
        };
        let state = model.at_rest(Pose::default(), 0.5);
        opt.reset(Command::new(0.5, 0.0));
        let mut speeds = Vec::new();
        for _ in 0..10 {
            let plan = opt.evaluate(&state, &scene).expect("a plan");
            assert!(plan.command.steer.abs() < 0.1, "{:?}", plan.command);
            speeds.push(plan.command.speed);
        }
        assert!(speeds[9] > 0.5, "speeds up: {speeds:?}");
    }

    #[test]
    fn turns_toward_a_path_to_the_left() {
        let model = Driven {
            ego: Ego::default(),
            car: IdealModel::default(),
        };
        let settings = Settings {
            batch_size: 500,
            threads: 2,
            ..Settings::default()
        };
        let mut opt = Optimizer::new(model, settings, critics());
        let path: Vec<Pose> = (0..=60)
            .map(|i| Pose::new(f64::from(i) * 0.05, 1.0, 0.0))
            .collect();
        let scene = Scene {
            path: &path,
            goal: Pose::new(30.0, 1.0, 0.0),
            costmap: None,
            footprint: &[],
            tracking_unknown: false,
            lane: None,
        };
        let state = model.at_rest(Pose::default(), 1.0);
        opt.reset(Command::new(1.0, 0.0));
        let mut steer = 0.0;
        for _ in 0..5 {
            steer = opt.evaluate(&state, &scene).expect("a plan").command.steer;
        }
        assert!(steer > 0.05, "{steer}");
    }

    /// u' Σ⁻¹ e the long way for a random walk: Σᵢⱼ = step² (min(i, j) + 1).
    fn brute_force_walk(u: &[f64], e: &[f64], step: f64) -> f64 {
        let n = u.len();
        let mut m: Vec<Vec<f64>> = (0..n)
            .map(|i| {
                let mut row: Vec<f64> = (0..n)
                    .map(|j| step * step * (i.min(j) + 1) as f64)
                    .collect();
                row.push(e[i]);
                row
            })
            .collect();
        for col in 0..n {
            let pivot = m[col].clone();
            for row in m.iter_mut().skip(col + 1) {
                let f = row[col] / pivot[col];
                for (cell, p) in row.iter_mut().zip(&pivot).skip(col) {
                    *cell -= f * p;
                }
            }
        }
        let mut x = vec![0.0; n];
        for row in (0..n).rev() {
            let known: f64 = (row + 1..n).map(|k| m[row][k] * x[k]).sum();
            x[row] = (m[row][n] - known) / m[row][row];
        }
        u.iter().zip(&x).map(|(a, b)| a * b).sum()
    }

    fn smooth() -> Settings {
        Settings {
            sampling: Sampling::Smooth,
            ..Settings::default()
        }
    }

    #[test]
    fn sampling_reads_its_names() {
        assert_eq!("smooth".parse(), Ok(Sampling::Smooth));
        assert_eq!("correlated".parse(), Ok(Sampling::Correlated));
        assert_eq!("".parse(), Ok(Sampling::Correlated));
        assert!("jerky".parse::<Sampling>().is_err());
        assert_eq!(Settings::default().sampling, Sampling::Correlated);
    }

    #[test]
    fn smooth_samples_change_a_little_a_step() {
        let steps = 56;
        let sequence = vec![Command::new(1.0, 0.1); steps];
        let mut controls = vec![Command::default(); steps];
        // Mean |change a step| and the spread at the first and last step, over many samples.
        let mut stats = |s: &Settings| {
            let mut rng = Rng::new(7);
            let (mut jump, mut first, mut last) = (0.0, 0.0, 0.0);
            let n = 4000;
            for _ in 0..n {
                sample_into(s, &mut rng, &sequence, &mut controls);
                jump += controls
                    .windows(2)
                    .map(|w| (w[1].steer - w[0].steer).abs())
                    .sum::<f64>()
                    / (steps - 1) as f64;
                first += (controls[0].steer - 0.1).powi(2);
                last += (controls[steps - 1].steer - 0.1).powi(2);
            }
            let n = f64::from(n);
            (jump / n, (first / n).sqrt(), (last / n).sqrt())
        };
        let (jump_ar1, _, last_ar1) = stats(&Settings::default());
        let s = smooth();
        let (jump, first, last) = stats(&s);
        let step = s.steer_rate_std * s.model_dt;
        // Each change is one white step: E|N(0, step²)| = step x sqrt(2 / pi).
        let want = step * (2.0 / std::f64::consts::PI).sqrt();
        assert!((jump - want).abs() < 0.05 * want, "{jump} vs {want}");
        assert!(jump < 0.5 * jump_ar1, "smooth {jump} vs AR(1) {jump_ar1}");
        // The spread grows as a random walk's: step at the first, step x sqrt(56) at the end.
        assert!((first - step).abs() < 0.05 * step, "{first}");
        let end = step * (steps as f64).sqrt();
        assert!((last - end).abs() < 0.05 * end, "{last} vs {end}");
        // About as wide as the champion's at the horizon's end, so MPPI still explores.
        assert!(last > 0.8 * last_ar1, "{last} vs {last_ar1}");
    }

    #[test]
    fn smooth_control_cost_is_the_random_walks_precision() {
        let u = [0.05, 0.08, 0.1, 0.06, 0.02];
        let e = [0.01, 0.015, 0.03, 0.02, 0.025];
        let sequence: Vec<Command> = u.iter().map(|&v| Command::new(0.0, v)).collect();
        let sample: Vec<Command> = u
            .iter()
            .zip(&e)
            .map(|(&v, &n)| Command::new(0.0, v + n))
            .collect();
        let s = smooth();
        let step = s.steer_rate_std * s.model_dt;
        let want = s.gamma * brute_force_walk(&u, &e, step);
        let got = control_cost(&s, &sequence, &sample);
        assert!(
            (got - want).abs() < 1e-9 * want.abs().max(1.0),
            "{got} vs {want}"
        );
        // steer_gamma weighs it, as with the champion's noise.
        let calm = Settings {
            steer_gamma: Some(0.05),
            ..smooth()
        };
        let calm_want = 0.05 * brute_force_walk(&u, &e, step);
        let calm_got = control_cost(&calm, &sequence, &sample);
        assert!((calm_got - calm_want).abs() < 1e-9 * calm_want.abs().max(1.0));
        // No noise, no cost.
        assert_eq!(control_cost(&s, &sequence, &sequence), 0.0);
    }

    #[test]
    fn smooth_sampling_still_turns_toward_the_path() {
        let model = Driven {
            ego: Ego::default(),
            car: IdealModel::default(),
        };
        let settings = Settings {
            batch_size: 500,
            threads: 2,
            ..smooth()
        };
        let mut opt = Optimizer::new(model, settings, critics());
        let path: Vec<Pose> = (0..=60)
            .map(|i| Pose::new(f64::from(i) * 0.05, 1.0, 0.0))
            .collect();
        let scene = Scene {
            path: &path,
            goal: Pose::new(30.0, 1.0, 0.0),
            costmap: None,
            footprint: &[],
            tracking_unknown: false,
            lane: None,
        };
        let state = model.at_rest(Pose::default(), 1.0);
        opt.reset(Command::new(1.0, 0.0));
        let mut steer = 0.0;
        for _ in 0..5 {
            steer = opt.evaluate(&state, &scene).expect("a plan").command.steer;
        }
        assert!(steer > 0.05, "{steer}");
    }

    #[test]
    fn same_seed_same_plan() {
        let model = Driven {
            ego: Ego::default(),
            car: LagModel::default(),
        };
        let settings = Settings {
            batch_size: 200,
            threads: 3,
            ..Settings::default()
        };
        let path = straight();
        let scene = Scene {
            path: &path,
            goal: Pose::new(30.0, 0.0, 0.0),
            costmap: None,
            footprint: &[],
            tracking_unknown: false,
            lane: None,
        };
        let state = model.at_rest(Pose::default(), 0.5);
        let mut a = Optimizer::new(model, settings.clone(), critics());
        let mut b = Optimizer::new(model, settings, critics());
        assert_eq!(a.evaluate(&state, &scene), b.evaluate(&state, &scene));
    }
}

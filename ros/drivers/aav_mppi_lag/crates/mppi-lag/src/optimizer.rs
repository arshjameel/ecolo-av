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
//!    ([`crate::ego::Driven`]).
//! 2. **Roll out**: each sample through the model from the car's state now: its pose
//!    and speed, ego's output, the actuators' levels and the commands still in their
//!    dead time. This is where the lag comes in; Nav2's own model has none. Split across
//!    threads.
//! 3. **Score**: the critics, plus Nav2's control cost
//!    gamma / std² x sum(u x (u_sample - u)).
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
    /// m/s: the speed noise's spread (Nav2's vx_std).
    pub speed_std: f64,
    /// rad: the steering noise's spread (Nav2 samples wz instead; ours samples the angle).
    pub steer_std: f64,
    /// 0..1: how much each step's noise carries the last step's (AR(1); 0: white, as
    /// Nav2). The spread stays `speed_std` and `steer_std` either way.
    pub noise_correlation: f64,
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
            speed_std: 0.4,
            steer_std: 0.15,
            noise_correlation: 0.8,
            vx_max: 1.0,
            vx_min: 0.0,
            max_steer: 0.5,
            retry_attempt_limit: 1,
            threads: 0,
            seed: 42,
        }
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
        let (model, settings, sequence) = (&self.model, &self.settings, &self.sequence);

        // 1 and 2: sample and roll out.
        std::thread::scope(|scope| {
            for (part, seed) in self.rollouts.chunks_mut(chunk).zip(&seeds) {
                scope.spawn(move || {
                    let mut rng = Rng::new(*seed);
                    let a = settings.noise_correlation.clamp(0.0, 0.999);
                    let fresh = (1.0 - a * a).sqrt();
                    for r in part {
                        let (mut nv, mut ns) = (rng.normal(), rng.normal());
                        for (c, base) in r.controls.iter_mut().zip(sequence) {
                            *c = Command::new(
                                base.speed + settings.speed_std * nv,
                                base.steer + settings.steer_std * ns,
                            );
                            nv = a * nv + fresh * rng.normal();
                            ns = a * ns + fresh * rng.normal();
                        }
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

        // 3: the critics, on what this round's samples reached.
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
        let (gv, gs) = (
            settings.gamma / settings.speed_std.powi(2),
            settings.gamma / settings.steer_std.powi(2),
        );
        std::thread::scope(|scope| {
            for part in self.rollouts.chunks_mut(chunk) {
                let prepared = &prepared;
                scope.spawn(move || {
                    for r in part {
                        let score = prepared.score(r);
                        let control: f64 = r
                            .controls
                            .iter()
                            .zip(sequence)
                            .map(|(c, u)| {
                                gv * u.speed * (c.speed - u.speed)
                                    + gs * u.steer * (c.steer - u.steer)
                            })
                            .sum();
                        r.cost = score.cost + control;
                        r.collided = score.collided;
                    }
                });
            }
        });
    }

    /// 4: weigh the samples into a new sequence.
    fn update(&mut self) {
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
    pub fn evaluate(&mut self, state: &M::State, scene: &Scene) -> Result<Plan, NoValidControl> {
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

        savitzky_golay(&mut self.sequence, &mut self.history);
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

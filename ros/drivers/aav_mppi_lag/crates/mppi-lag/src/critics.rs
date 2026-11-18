//! The critics we run in Nav2 (nav2.yaml `MPPI.critics`), ported from nav2_mppi_controller.
//!
//! Each scores one sample's future; lower is better. Same rules, defaults and names as
//! Nav2's (`src/critics/*.cpp` at the commit in PROVENANCE.md), on our model's futures:
//!
//! | Critic | Adds | Defaults (Nav2; ours in nav2.yaml) |
//! |---|---|---|
//! | `ConstraintCritic` | time spent over the speed limits or tighter than the turning radius | 4.0 |
//! | `CostCritic` | the costmap under the car's outline; a collision ends the sample | 3.81 / 254, step 2 |
//! | `GoalCritic` | the mean distance to the goal, within 1.4 m of it | 5.0 |
//! | `GoalAngleCritic` | the mean heading error to the goal's, within `threshold` (ours 2.5 m) | 3.0 |
//! | `PathAlignCritic` | the mean distance (and heading, ours) from the path at the same distance along | 10.0, offset 20, step 4 |
//! | `PathFollowCritic` | the end's distance to a path point 6 past the furthest reached | 5.0 |
//! | `PathAngleCritic` | the end's heading error to a path point 4 past the furthest, when over 45 degrees; nose first (mode 0), either end (1), or the way the path faces, tail first on a reversing stretch (2) | 2.2 |
//! | `PreferForwardCritic` | time spent reversing | 5.0 |
//! | `LaneCritic` (ours) | the outline past the lane's soft edges (squared), and past its hard edges | 100 soft, 1000 hard |
//!
//! Where ours differs: the speeds and yaw rates are the model's (with lag), not the
//! commands'. Reversing: as Nav2's, the path ends at its next change of direction
//! ([`crate::path`]), so `PathAlignCritic` never matches across a cusp.

use crate::costmap::{Costmap, INSCRIBED, LETHAL, NO_INFORMATION};
use crate::lane::Lane;
use crate::model::{wrap, Command, Pose};

/// One sample: its commands and the future the model gave them.
#[derive(Debug, Clone, Default)]
pub struct Rollout {
    /// The command at each step (noised, within the limits).
    pub controls: Vec<Command>,
    /// The pose after each step.
    pub poses: Vec<Pose>,
    /// The speed after each step (m/s).
    pub speeds: Vec<f64>,
    /// The wheels' angle after each step (rad).
    pub steerings: Vec<f64>,
    /// The critics' total.
    pub cost: f64,
    /// The cost critic found a collision.
    pub collided: bool,
}

impl Rollout {
    /// A rollout with room for `steps` steps.
    #[must_use]
    pub fn with_steps(steps: usize) -> Self {
        Self {
            controls: vec![Command::default(); steps],
            poses: vec![Pose::default(); steps],
            speeds: vec![0.0; steps],
            steerings: vec![0.0; steps],
            cost: 0.0,
            collided: false,
        }
    }
}

/// What every critic sees this tick.
#[derive(Debug, Clone, Copy)]
pub struct CriticData<'a> {
    /// The car now.
    pub pose: Pose,
    /// The path's end.
    pub goal: Pose,
    /// The path ahead (the window).
    pub path: &'a [Pose],
    /// The furthest window index any sample reaches.
    pub furthest: usize,
    /// For each window pose but the last: clear of obstacles.
    pub valid: &'a [bool],
    /// s per step.
    pub dt: f64,
    /// m: turns the steering into a yaw rate.
    pub wheelbase: f64,
    /// The local costmap, if any.
    pub costmap: Option<&'a Costmap>,
    /// The car's outline (base_link points), padded.
    pub footprint: &'a [(f64, f64)],
    /// m/s, the speed limits (vx_max, vx_min).
    pub vx_max: f64,
    /// m/s.
    pub vx_min: f64,
    /// m: the tightest turn (wheelbase / tan(full lock)).
    pub min_turning_r: f64,
    /// The road ahead's lane, in the same frame as the poses, if one is known.
    pub lane: Option<&'a Lane>,
}

/// (raw x weight)^power, or raw x weight for power 1 (Nav2's every critic).
fn weighted(raw: f64, weight: f64, power: u32) -> f64 {
    if power > 1 {
        (raw * weight).powi(power as i32)
    } else {
        raw * weight
    }
}

fn within(a: &Pose, b: &Pose, threshold: f64) -> bool {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2) < threshold * threshold
}

/// What a critic gives one sample.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Score {
    /// Added to the sample's cost.
    pub cost: f64,
    /// The sample ran into something.
    pub collided: bool,
}

impl From<f64> for Score {
    fn from(cost: f64) -> Self {
        Self {
            cost,
            collided: false,
        }
    }
}

/// `ConstraintCritic`: over the speed limits, or tighter than the turning radius.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstraintCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
}

impl Default for ConstraintCritic {
    fn default() -> Self {
        Self {
            weight: 4.0,
            power: 1,
        }
    }
}

impl ConstraintCritic {
    fn score(&self, d: &CriticData, r: &Rollout) -> f64 {
        let min_vel = if d.vx_min > 0.0 {
            d.vx_min
        } else {
            -d.vx_min.abs()
        };
        let raw: f64 = r
            .speeds
            .iter()
            .zip(&r.steerings)
            .map(|(&v, &steer)| {
                let wz = v * steer.tan() / d.wheelbase;
                let tight = if wz.abs() > 1e-12 {
                    (d.min_turning_r - v.abs() / wz.abs()).max(0.0)
                } else {
                    0.0
                };
                ((v - d.vx_max).max(0.0) + (min_vel - v).max(0.0) + tight) * d.dt
            })
            .sum();
        weighted(raw, self.weight, self.power)
    }
}

/// `CostCritic`: the costmap under the car's outline along the future.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CostCritic {
    /// cost_weight (Nav2 divides it by 254).
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// Added per pose at or over `near_collision_cost`.
    pub critical_cost: f64,
    /// A pose this costly counts as near a collision.
    pub near_collision_cost: u8,
    /// A collision's whole cost.
    pub collision_cost: f64,
    /// m: this near the goal, plain costs stop counting.
    pub near_goal_distance: f64,
    /// Every nth pose is checked.
    pub trajectory_point_step: usize,
    /// Check the outline, not just the centre (ours: true).
    pub consider_footprint: bool,
    /// Over this centre cost, check the whole outline (Nav2's circumscribed cost).
    pub possible_collision_cost: f64,
    /// Unknown cells are drivable.
    pub tracking_unknown: bool,
}

impl Default for CostCritic {
    fn default() -> Self {
        Self {
            weight: 3.81,
            power: 1,
            critical_cost: 300.0,
            near_collision_cost: 253,
            collision_cost: 1_000_000.0,
            near_goal_distance: 0.5,
            trajectory_point_step: 2,
            consider_footprint: true,
            possible_collision_cost: 0.0,
            tracking_unknown: false,
        }
    }
}

/// Nav2's inflation cost at `distance` m from an obstacle (InflationLayer::computeCost).
#[must_use]
pub fn inflation_cost(distance: f64, inscribed: f64, cost_scaling_factor: f64) -> f64 {
    if distance <= 0.0 {
        f64::from(LETHAL)
    } else if distance <= inscribed {
        f64::from(INSCRIBED)
    } else {
        f64::from(INSCRIBED - 1) * (-cost_scaling_factor * (distance - inscribed)).exp()
    }
}

impl CostCritic {
    fn in_collision(&self, d: &CriticData, map: &Costmap, cost: f64, pose: &Pose) -> bool {
        let mut score = cost;
        if self.consider_footprint
            && (cost >= self.possible_collision_cost || self.possible_collision_cost < 1.0)
        {
            score = f64::from(map.footprint_cost(pose, d.footprint));
        }
        match score as u8 {
            LETHAL => true,
            INSCRIBED => !self.consider_footprint,
            NO_INFORMATION => !self.tracking_unknown,
            _ => false,
        }
    }

    fn score(&self, d: &CriticData, r: &Rollout) -> Score {
        let Some(map) = d.costmap else {
            return Score::default();
        };
        let near_goal = within(&d.pose, &d.goal, self.near_goal_distance);
        let step = self.trajectory_point_step.max(1);
        let len = r.poses.len() / step;
        let mut total = 0.0;
        let mut collided = false;
        for pose in r.poses.iter().step_by(step).take(len) {
            let cost = f64::from(map.cost(pose.x, pose.y));
            if cost < 1.0 {
                continue;
            }
            if self.in_collision(d, map, cost, pose) {
                total = self.collision_cost;
                collided = true;
                break;
            }
            if cost >= f64::from(self.near_collision_cost) {
                total += self.critical_cost;
            } else if !near_goal {
                total += cost;
            }
        }
        Score {
            cost: weighted(total, self.weight / 254.0 / len.max(1) as f64, self.power),
            collided,
        }
    }
}

/// `GoalCritic`: near the goal, the mean distance to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GoalCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// m: only this close to the goal.
    pub threshold: f64,
}

impl Default for GoalCritic {
    fn default() -> Self {
        Self {
            weight: 5.0,
            power: 1,
            threshold: 1.4,
        }
    }
}

impl GoalCritic {
    fn score(&self, d: &CriticData, r: &Rollout) -> f64 {
        if !within(&d.pose, &d.goal, self.threshold) || r.poses.is_empty() {
            return 0.0;
        }
        let mean = r.poses.iter().map(|p| p.distance(&d.goal)).sum::<f64>() / r.poses.len() as f64;
        weighted(mean, self.weight, self.power)
    }
}

/// `GoalAngleCritic`: near the goal, the mean heading error to the path's last heading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GoalAngleCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// m: only this close to the goal (ours 2.5, Nav2's 0.5).
    pub threshold: f64,
}

impl Default for GoalAngleCritic {
    fn default() -> Self {
        Self {
            weight: 3.0,
            power: 1,
            threshold: 0.5,
        }
    }
}

impl GoalAngleCritic {
    fn score(&self, d: &CriticData, r: &Rollout) -> f64 {
        let (Some(last), false) = (d.path.last(), r.poses.is_empty()) else {
            return 0.0;
        };
        if !within(&d.pose, &d.goal, self.threshold) {
            return 0.0;
        }
        let mean = r
            .poses
            .iter()
            .map(|p| wrap(last.yaw - p.yaw).abs())
            .sum::<f64>()
            / r.poses.len() as f64;
        weighted(mean, self.weight, self.power)
    }
}

/// The index after `dist` m along (Nav2's findClosestPathPt): the closer of the two
/// path points either side of `dist`, searching from `init`.
#[must_use]
pub fn closest_path_point(along: &[f64], dist: f64, init: usize) -> usize {
    let mut before = if init != 0 { along[init] } else { 0.0 };
    for (i, &here) in along.iter().enumerate().skip(init + 1) {
        if here > dist {
            if i > 0 && dist - before < here - dist {
                return i - 1;
            }
            return i;
        }
        before = here;
    }
    along.len().saturating_sub(1)
}

/// `PathAlignCritic`: stay on the path, matched by distance along it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathAlignCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// m: off within this of the goal.
    pub threshold: f64,
    /// Skip if the samples reach fewer path points than this.
    pub offset_from_furthest: usize,
    /// Every nth pose is compared.
    pub trajectory_point_step: usize,
    /// Skip if more than this share of the path is blocked.
    pub max_path_occupancy_ratio: f64,
    /// Count the heading too (ours: true; the Hybrid-A* headings are drivable).
    pub use_path_orientations: bool,
}

impl Default for PathAlignCritic {
    fn default() -> Self {
        Self {
            weight: 10.0,
            power: 1,
            threshold: 0.5,
            offset_from_furthest: 20,
            trajectory_point_step: 4,
            max_path_occupancy_ratio: 0.07,
            use_path_orientations: false,
        }
    }
}

impl PathAlignCritic {
    /// False if the critic sits out this tick (near the goal, too little path, blocked).
    fn active(&self, d: &CriticData) -> bool {
        if within(&d.pose, &d.goal, self.threshold) {
            return false;
        }
        let count = d.furthest;
        if count < self.offset_from_furthest || count == 0 {
            return false;
        }
        let mut invalid = 0.0;
        for &ok in d.valid.iter().take(count) {
            if !ok {
                invalid += 1.0;
            }
            if invalid / count as f64 > self.max_path_occupancy_ratio && invalid > 2.0 {
                return false;
            }
        }
        true
    }

    fn score(&self, d: &CriticData, along: &[f64], r: &Rollout) -> f64 {
        let count = along.len();
        let step = self.trajectory_point_step.max(1);
        let mut sum = 0.0;
        let mut samples = 0usize;
        let mut travelled = 0.0;
        let mut path_pt = 0usize;
        let mut last = match r.poses.first() {
            Some(p) => *p,
            None => return 0.0,
        };
        for t in r.poses.iter().step_by(step).skip(1) {
            travelled += t.distance(&last);
            last = *t;
            path_pt = closest_path_point(along, travelled, path_pt);
            if d.valid.get(path_pt).copied().unwrap_or(true) {
                let p = d.path[path_pt.min(count - 1)];
                let (dx, dy) = (p.x - t.x, p.y - t.y);
                samples += 1;
                sum += if self.use_path_orientations {
                    let dyaw = wrap(t.yaw - p.yaw);
                    (dx * dx + dy * dy + dyaw * dyaw).sqrt()
                } else {
                    dx.hypot(dy)
                };
            }
        }
        let raw = if samples > 0 {
            sum / samples as f64
        } else {
            0.0
        };
        weighted(raw, self.weight, self.power)
    }
}

/// `PathFollowCritic`: end near a path point a little past the furthest reached.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathFollowCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// m: off within this of the goal.
    pub threshold: f64,
    /// Path points past the furthest reached.
    pub offset_from_furthest: usize,
}

impl Default for PathFollowCritic {
    fn default() -> Self {
        Self {
            weight: 5.0,
            power: 1,
            threshold: 1.4,
            offset_from_furthest: 6,
        }
    }
}

impl PathFollowCritic {
    /// The path point the samples should end near, or `None` if it sits out.
    fn target(&self, d: &CriticData) -> Option<Pose> {
        if d.path.len() < 2 || within(&d.pose, &d.goal, self.threshold) {
            return None;
        }
        let size = d.path.len() - 1;
        let mut idx = (d.furthest + self.offset_from_furthest).min(size);
        while idx + 1 < size && !d.valid.get(idx).copied().unwrap_or(true) {
            idx += 1;
        }
        Some(d.path[idx])
    }

    fn score(&self, target: &Pose, r: &Rollout) -> f64 {
        r.poses.last().map_or(0.0, |end| {
            weighted(end.distance(target), self.weight, self.power)
        })
    }
}

/// How `PathAngleCritic` judges the angle to the path point ahead (Nav2's `mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PathAngleMode {
    /// 0: the point should be ahead of the nose (forward only).
    #[default]
    ForwardPreference,
    /// 1: ahead of the nose or of the tail, whichever is nearer.
    NoDirectionalPreference,
    /// 2: the way the path's own heading says: ahead of the nose on a forward stretch,
    /// of the tail on a reversing one (the planner's poses face the car's way).
    ConsiderFeasiblePathOrientations,
}

impl PathAngleMode {
    /// Nav2's number for the mode; `None` for one it doesn't have.
    #[must_use]
    pub fn from_number(n: i64) -> Option<Self> {
        match n {
            0 => Some(Self::ForwardPreference),
            1 => Some(Self::NoDirectionalPreference),
            2 => Some(Self::ConsiderFeasiblePathOrientations),
            _ => None,
        }
    }
}

/// The angle (rad, 0..pi) from `yaw` to the bearing `bearing`, or to the bearing turned
/// round where that is nearer (`flip`).
fn off_by(yaw: f64, bearing: f64, flip: bool) -> f64 {
    let ahead = wrap(bearing - yaw).abs();
    if flip {
        ahead.min(wrap(bearing + std::f64::consts::PI - yaw).abs())
    } else {
        ahead
    }
}

/// `PathAngleCritic`: when the path point ahead is over `max_angle_to_furthest` off the
/// car's nose (or tail, by `mode`), end facing it that way.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathAngleCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// m: off within this of the goal.
    pub threshold: f64,
    /// Path points past the furthest reached.
    pub offset_from_furthest: usize,
    /// rad: on only when the point is further off the nose than this.
    pub max_angle_to_furthest: f64,
    /// Which way the car should face the point.
    pub mode: PathAngleMode,
}

impl Default for PathAngleCritic {
    fn default() -> Self {
        Self {
            weight: 2.2,
            power: 1,
            threshold: 0.5,
            offset_from_furthest: 4,
            max_angle_to_furthest: std::f64::consts::FRAC_PI_4,
            mode: PathAngleMode::ForwardPreference,
        }
    }
}

impl PathAngleCritic {
    /// The mode it runs in: no directional preference only if the car may reverse (as
    /// Nav2, which falls back to forward preference with `vx_min` 0).
    fn mode(&self, d: &CriticData) -> PathAngleMode {
        if self.mode == PathAngleMode::NoDirectionalPreference && d.vx_min >= 0.0 {
            PathAngleMode::ForwardPreference
        } else {
            self.mode
        }
    }

    /// The angle from a car at `pose` to the point `target`, as the mode counts it.
    fn angle(mode: PathAngleMode, pose: &Pose, target: &Pose) -> f64 {
        let bearing = (target.y - pose.y).atan2(target.x - pose.x);
        match mode {
            PathAngleMode::ForwardPreference => off_by(pose.yaw, bearing, false),
            PathAngleMode::NoDirectionalPreference => off_by(pose.yaw, bearing, true),
            PathAngleMode::ConsiderFeasiblePathOrientations => {
                // The way the path goes there: along its heading, or against it (reversing).
                let forwards = wrap(bearing - target.yaw).abs() < std::f64::consts::FRAC_PI_2;
                let way = if forwards {
                    bearing
                } else {
                    wrap(bearing + std::f64::consts::PI)
                };
                off_by(pose.yaw, way, false)
            }
        }
    }

    fn target(&self, d: &CriticData) -> Option<Pose> {
        if d.path.is_empty() || within(&d.pose, &d.goal, self.threshold) {
            return None;
        }
        let idx = (d.furthest + self.offset_from_furthest).min(d.path.len() - 1);
        let goal = d.path[idx];
        (Self::angle(self.mode(d), &d.pose, &goal) >= self.max_angle_to_furthest).then_some(goal)
    }

    fn score(&self, d: &CriticData, target: &Pose, r: &Rollout) -> f64 {
        r.poses.last().map_or(0.0, |end| {
            weighted(
                Self::angle(self.mode(d), end, target),
                self.weight,
                self.power,
            )
        })
    }
}

/// `PreferForwardCritic`: time spent going backwards.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreferForwardCritic {
    /// cost_weight.
    pub weight: f64,
    /// cost_power.
    pub power: u32,
    /// m: off within this of the goal.
    pub threshold: f64,
}

impl Default for PreferForwardCritic {
    fn default() -> Self {
        Self {
            weight: 5.0,
            power: 1,
            threshold: 0.5,
        }
    }
}

impl PreferForwardCritic {
    fn score(&self, d: &CriticData, r: &Rollout) -> f64 {
        if within(&d.pose, &d.goal, self.threshold) {
            return 0.0;
        }
        let raw: f64 = r.speeds.iter().map(|v| (-v).max(0.0) * d.dt).sum();
        weighted(raw, self.weight, self.power)
    }
}

/// `LaneCritic` (ours; Nav2 has none): keep the car's outline inside the lane, from the
/// road ahead's edges ([`crate::lane`]).
///
/// For every `trajectory_point_step`th pose, each corner of the outline is put beside the
/// lane. The furthest any corner goes past a soft edge (a lane line), squared, and whether
/// any goes past a hard edge (a kerb), averaged over the poses checked:
///
/// | Part | Adds |
/// |---|---|
/// | soft | (mean of past_soft² x `soft_weight`)^`power`: smooth, nothing inside the lines |
/// | hard | mean of (1 + past_hard) x `hard_weight`: a sample over the kerb loses |
///
/// No lane (no road ahead, or all its edges NaN), or an edge not known: 0 for that edge.
/// A kerb is not a collision: a car already over one (a lane map off by a metre) still
/// gets a plan back in, rather than Nav2's NoValidControl.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneCritic {
    /// Per m² past a soft edge (a pose 0.3 m over costs 9 at 100).
    pub soft_weight: f64,
    /// Per pose with the outline past a hard edge (plus per m past it).
    pub hard_weight: f64,
    /// cost_power, on the soft part.
    pub power: u32,
    /// Every nth pose is checked.
    pub trajectory_point_step: usize,
}

impl Default for LaneCritic {
    fn default() -> Self {
        Self {
            soft_weight: 100.0,
            hard_weight: 1000.0,
            power: 1,
            trajectory_point_step: 2,
        }
    }
}

/// m past an edge `limit` m to the side, for a point `out` m out that way (0 inside, or
/// not known).
fn past(out: f64, limit: Option<f64>) -> f64 {
    limit.map_or(0.0, |l| (out - l).max(0.0))
}

impl LaneCritic {
    /// (m past a soft edge, m past a hard edge, or -1 if inside) for one pose's outline.
    fn over(lane: &Lane, footprint: &[(f64, f64)], pose: &Pose, hint: usize) -> (f64, f64) {
        let (s, c) = pose.yaw.sin_cos();
        let centre = [(0.0, 0.0)];
        let corners = if footprint.is_empty() {
            &centre[..]
        } else {
            footprint
        };
        let mut soft = 0.0f64;
        let mut hard = -1.0f64;
        for &(fx, fy) in corners {
            let b = lane.beside(pose.x + c * fx - s * fy, pose.y + s * fx + c * fy, hint);
            let e = b.edges;
            soft = soft
                .max(past(b.lateral, e.left_soft))
                .max(past(-b.lateral, e.right_soft));
            for (out, limit) in [(b.lateral, e.left_hard), (-b.lateral, e.right_hard)] {
                if let Some(l) = limit {
                    if out > l {
                        hard = hard.max(out - l);
                    }
                }
            }
        }
        (soft, hard)
    }

    fn score(&self, d: &CriticData, start: usize, r: &Rollout) -> f64 {
        let Some(lane) = d.lane else {
            return 0.0;
        };
        let step = self.trajectory_point_step.max(1);
        let mut hint = start;
        let (mut soft, mut hard, mut n) = (0.0, 0.0, 0usize);
        for pose in r.poses.iter().step_by(step) {
            hint = lane.beside(pose.x, pose.y, hint).index;
            let (s, h) = Self::over(lane, d.footprint, pose, hint);
            soft += s * s;
            if h >= 0.0 {
                hard += 1.0 + h;
            }
            n += 1;
        }
        if n == 0 {
            return 0.0;
        }
        let n = n as f64;
        weighted(soft / n, self.soft_weight, self.power) + hard / n * self.hard_weight
    }
}

/// One critic, by Nav2's name.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Critic {
    /// `ConstraintCritic`.
    Constraint(ConstraintCritic),
    /// `CostCritic`.
    Cost(CostCritic),
    /// `GoalCritic`.
    Goal(GoalCritic),
    /// `GoalAngleCritic`.
    GoalAngle(GoalAngleCritic),
    /// `PathAlignCritic`.
    PathAlign(PathAlignCritic),
    /// `PathFollowCritic`.
    PathFollow(PathFollowCritic),
    /// `PathAngleCritic`.
    PathAngle(PathAngleCritic),
    /// `PreferForwardCritic`.
    PreferForward(PreferForwardCritic),
    /// `LaneCritic` (ours).
    Lane(LaneCritic),
}

/// Our own critics, which Nav2 doesn't have: `Critic::by_name` knows them too, but they
/// run only when the critics list names them.
pub const OWN: [&str; 1] = ["LaneCritic"];

/// Nav2's names `Critic::by_name` knows: nav2.yaml's list.
pub const NAMES: [&str; 8] = [
    "ConstraintCritic",
    "CostCritic",
    "GoalCritic",
    "GoalAngleCritic",
    "PathAlignCritic",
    "PathFollowCritic",
    "PathAngleCritic",
    "PreferForwardCritic",
];

impl Critic {
    /// The critic called `name` (Nav2's plugin name), with Nav2's defaults.
    #[must_use]
    pub fn by_name(name: &str) -> Option<Self> {
        Some(match name {
            "ConstraintCritic" => Self::Constraint(ConstraintCritic::default()),
            "CostCritic" => Self::Cost(CostCritic::default()),
            "GoalCritic" => Self::Goal(GoalCritic::default()),
            "GoalAngleCritic" => Self::GoalAngle(GoalAngleCritic::default()),
            "PathAlignCritic" => Self::PathAlign(PathAlignCritic::default()),
            "PathFollowCritic" => Self::PathFollow(PathFollowCritic::default()),
            "PathAngleCritic" => Self::PathAngle(PathAngleCritic::default()),
            "PreferForwardCritic" => Self::PreferForward(PreferForwardCritic::default()),
            "LaneCritic" => Self::Lane(LaneCritic::default()),
            _ => return None,
        })
    }

    /// Nav2's name for it.
    #[must_use]
    pub fn name(&self) -> &'static str {
        match self {
            Self::Constraint(_) => "ConstraintCritic",
            Self::Cost(_) => "CostCritic",
            Self::Goal(_) => "GoalCritic",
            Self::GoalAngle(_) => "GoalAngleCritic",
            Self::PathAlign(_) => "PathAlignCritic",
            Self::PathFollow(_) => "PathFollowCritic",
            Self::PathAngle(_) => "PathAngleCritic",
            Self::PreferForward(_) => "PreferForwardCritic",
            Self::Lane(_) => "LaneCritic",
        }
    }
}

/// The critics, and what each needs worked out once per tick, ready to score samples.
#[derive(Debug, Clone)]
pub struct Prepared<'a> {
    critics: &'a [Critic],
    data: CriticData<'a>,
    along: Vec<f64>,
    align: bool,
    follow: Option<Pose>,
    angle: Option<Pose>,
    /// The lane segment beside the car now: where each sample's search starts.
    lane_start: usize,
}

impl<'a> Prepared<'a> {
    /// Work out the per-tick parts: PathAlign's distances along, PathFollow's and
    /// PathAngle's targets, where the car is beside the lane.
    #[must_use]
    pub fn new(critics: &'a [Critic], data: CriticData<'a>) -> Self {
        let mut along = Vec::new();
        let mut align = false;
        let mut follow = None;
        let mut angle = None;
        for critic in critics {
            match critic {
                Critic::PathAlign(c) if c.active(&data) => {
                    align = true;
                    let count = data.furthest.min(data.path.len());
                    along = Vec::with_capacity(count);
                    along.push(0.0);
                    for i in 1..count {
                        let step = data.path[i].distance(&data.path[i - 1]);
                        along.push(along[i - 1] + step);
                    }
                }
                Critic::PathFollow(c) => follow = c.target(&data),
                Critic::PathAngle(c) => angle = c.target(&data),
                _ => {}
            }
        }
        let lane_start = data
            .lane
            .map_or(0, |lane| lane.nearest(data.pose.x, data.pose.y));
        Self {
            critics,
            data,
            along,
            align,
            follow,
            angle,
            lane_start,
        }
    }

    /// Every critic's score for one sample, added up.
    #[must_use]
    pub fn score(&self, r: &Rollout) -> Score {
        let d = &self.data;
        let mut total = Score::default();
        for critic in self.critics {
            let s: Score = match critic {
                Critic::Constraint(c) => c.score(d, r).into(),
                Critic::Cost(c) => c.score(d, r),
                Critic::Goal(c) => c.score(d, r).into(),
                Critic::GoalAngle(c) => c.score(d, r).into(),
                Critic::PathAlign(c) if self.align && !self.along.is_empty() => {
                    c.score(d, &self.along, r).into()
                }
                Critic::PathAlign(_) => 0.0.into(),
                Critic::PathFollow(c) => self.follow.map_or(0.0, |t| c.score(&t, r)).into(),
                Critic::PathAngle(c) => self.angle.map_or(0.0, |t| c.score(d, &t, r)).into(),
                Critic::PreferForward(c) => c.score(d, r).into(),
                Critic::Lane(c) => c.score(d, self.lane_start, r).into(),
            };
            total.cost += s.cost;
            total.collided |= s.collided;
        }
        total
    }
}

/// The furthest path index any sample's end comes nearest to (Nav2's
/// findPathFurthestReachedPoint: each sample searches from the furthest so far).
#[must_use]
pub fn furthest_reached(path: &[Pose], rollouts: &[Rollout]) -> usize {
    let mut furthest = 0usize;
    for end in rollouts.iter().filter_map(|r| r.poses.last()) {
        let mut best = (f64::MAX, 0usize);
        for (j, p) in path.iter().enumerate().skip(furthest) {
            let d2 = (p.x - end.x).powi(2) + (p.y - end.y).powi(2);
            if d2 < best.0 {
                best = (d2, j);
            }
        }
        furthest = furthest.max(best.1);
    }
    furthest
}

/// For each path pose but the last: clear of obstacles (Nav2's findPathCosts). Without a
/// costmap, all clear.
#[must_use]
pub fn path_clear(path: &[Pose], costmap: Option<&Costmap>, tracking_unknown: bool) -> Vec<bool> {
    let n = path.len().saturating_sub(1);
    let Some(map) = costmap else {
        return vec![true; n];
    };
    path.iter()
        .take(n)
        .map(
            |p| match map.cell(p.x, p.y).map(|(cx, cy)| map.at(cx, cy)) {
                None | Some(LETHAL) | Some(INSCRIBED) => false,
                Some(NO_INFORMATION) => tracking_unknown,
                Some(_) => true,
            },
        )
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lane::{Edges, LanePoint};

    const DT: f64 = 0.05;

    fn straight_path() -> Vec<Pose> {
        (0..=60)
            .map(|i| Pose::new(f64::from(i) * 0.05, 0.0, 0.0))
            .collect()
    }

    /// A sample driving straight at `v` from (0, y0) with heading `yaw`.
    fn rollout(v: f64, y0: f64, yaw: f64, steps: usize) -> Rollout {
        let mut r = Rollout::with_steps(steps);
        let mut p = Pose::new(0.0, y0, yaw);
        for i in 0..steps {
            p.x += v * yaw.cos() * DT;
            p.y += v * yaw.sin() * DT;
            r.poses[i] = p;
            r.speeds[i] = v;
            r.controls[i] = Command::new(v, 0.0);
        }
        r
    }

    fn data<'a>(path: &'a [Pose], valid: &'a [bool], pose: Pose, goal: Pose) -> CriticData<'a> {
        CriticData {
            pose,
            goal,
            path,
            furthest: path.len() - 1,
            valid,
            dt: DT,
            wheelbase: 1.561,
            costmap: None,
            footprint: &[],
            vx_max: 1.0,
            vx_min: 0.0,
            min_turning_r: 2.86,
            lane: None,
        }
    }

    #[test]
    fn weighted_matches_nav2() {
        assert_eq!(weighted(2.0, 3.0, 1), 6.0);
        assert_eq!(weighted(2.0, 3.0, 2), 36.0);
    }

    #[test]
    fn constraint_costs_overspeed_and_tight_turns() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let d = data(&path, &valid, Pose::default(), Pose::new(30.0, 0.0, 0.0));
        let c = ConstraintCritic::default();
        assert_eq!(c.score(&d, &rollout(1.0, 0.0, 0.0, 10)), 0.0);
        // 0.5 m/s over for 10 steps: 0.5 x 0.5 s x 4.
        assert!((c.score(&d, &rollout(1.5, 0.0, 0.0, 10)) - 1.0).abs() < 1e-9);
        // Steering past full lock's radius costs; within it doesn't.
        let mut r = rollout(1.0, 0.0, 0.0, 10);
        r.steerings.iter_mut().for_each(|s| *s = 0.6);
        assert!(c.score(&d, &r) > 0.0);
        r.steerings.iter_mut().for_each(|s| *s = 0.4);
        assert_eq!(c.score(&d, &r), 0.0);
    }

    #[test]
    fn goal_critics_only_near_the_goal() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let goal = Pose::new(3.0, 0.0, 0.0);
        let far = data(&path, &valid, Pose::default(), goal);
        let near = data(&path, &valid, Pose::new(2.0, 0.0, 0.0), goal);
        let r = rollout(1.0, 0.0, 0.0, 20);
        assert_eq!(GoalCritic::default().score(&far, &r), 0.0);
        assert!(GoalCritic::default().score(&near, &r) > 0.0);
        let angle = GoalAngleCritic {
            threshold: 2.5,
            ..GoalAngleCritic::default()
        };
        assert_eq!(angle.score(&near, &r), 0.0, "heading already right");
        let turned = rollout(1.0, 0.0, 0.5, 20);
        assert!(
            (angle.score(&near, &turned) - 1.5).abs() < 1e-9,
            "0.5 rad x 3"
        );
    }

    #[test]
    fn path_align_prefers_the_sample_on_the_path() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let d = data(&path, &valid, Pose::default(), Pose::new(30.0, 0.0, 0.0));
        let critics = [Critic::PathAlign(PathAlignCritic::default())];
        let p = Prepared::new(&critics, d);
        // As in Nav2, a sample's first pose is one step out, and the distance along is
        // counted from there: on the path, each match is 0.05 m (one step) ahead.
        let on = p.score(&rollout(1.0, 0.0, 0.0, 56)).cost;
        let off = p.score(&rollout(1.0, 0.5, 0.0, 56)).cost;
        assert!((on - 0.5).abs() < 1e-6, "0.05 m x 10: {on}");
        let expected = 0.5f64.hypot(0.05) * 10.0;
        assert!((off - expected).abs() < 1e-6, "{off}");
    }

    #[test]
    fn path_align_sits_out_near_the_goal_and_when_blocked() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let c = PathAlignCritic::default();
        let goal = Pose::new(3.0, 0.0, 0.0);
        assert!(!c.active(&data(&path, &valid, Pose::new(2.8, 0.0, 0.0), goal)));
        let blocked: Vec<bool> = (0..path.len() - 1).map(|i| i % 3 != 0).collect();
        assert!(!c.active(&data(&path, &blocked, Pose::default(), goal)));
        let mut short = data(&path, &valid, Pose::default(), goal);
        short.furthest = 10;
        assert!(!c.active(&short), "fewer than 20 points reached");
    }

    #[test]
    fn path_follow_wants_the_end_near_the_point_ahead() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let mut d = data(&path, &valid, Pose::default(), Pose::new(30.0, 0.0, 0.0));
        d.furthest = 20;
        let c = PathFollowCritic::default();
        let target = c.target(&d).expect("on");
        assert!((target.x - 1.3).abs() < 1e-9, "point 26");
        let r = rollout(1.0, 0.0, 0.0, 26); // ends at 1.3 m
        assert!(c.score(&target, &r) < 1e-9);
        assert!((c.score(&target, &rollout(1.0, 1.0, 0.0, 26)) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn path_angle_only_when_the_path_is_off_the_nose() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let c = PathAngleCritic::default();
        let goal = Pose::new(30.0, 0.0, 0.0);
        assert!(c
            .target(&data(&path, &valid, Pose::default(), goal))
            .is_none());
        let mut d = data(&path, &valid, Pose::new(0.0, 0.0, 1.2), goal);
        d.furthest = 0;
        let target = c.target(&d).expect("over 45 degrees off");
        let facing = rollout(0.0, 0.0, 0.0, 3);
        let away = rollout(0.0, 0.0, 1.2, 3);
        assert!(c.score(&d, &target, &facing) < c.score(&d, &target, &away));
    }

    #[test]
    fn path_angle_follows_a_reversing_path_tail_first() {
        // The path goes west, the car facing east along it (the planner's poses face the
        // car's way): a reversing stretch.
        let path: Vec<Pose> = (0..=60)
            .map(|i| Pose::new(-f64::from(i) * 0.05, 0.0, 0.0))
            .collect();
        let valid = vec![true; path.len() - 1];
        let goal = Pose::new(-30.0, 0.0, 0.0);
        let mut d = data(&path, &valid, Pose::default(), goal);
        d.vx_min = -0.5;
        d.furthest = 0;
        let forward = PathAngleCritic::default();
        assert!(
            forward.target(&d).is_some(),
            "forward only: the point is behind"
        );
        let feasible = PathAngleCritic {
            mode: PathAngleMode::ConsiderFeasiblePathOrientations,
            ..PathAngleCritic::default()
        };
        assert!(
            feasible.target(&d).is_none(),
            "tail first: already lined up"
        );
        let either = PathAngleCritic {
            mode: PathAngleMode::NoDirectionalPreference,
            ..PathAngleCritic::default()
        };
        assert!(either.target(&d).is_none());
        // Turned 1.2 rad: on; ending tail-first scores better than turning round.
        let mut turned = data(&path, &valid, Pose::new(0.0, 0.0, 1.2), goal);
        turned.vx_min = -0.5;
        turned.furthest = 0;
        let target = feasible.target(&turned).expect("off the tail");
        let tail_first = rollout(-0.5, 0.0, 0.0, 3);
        let nose_first = rollout(0.5, 0.0, std::f64::consts::PI, 3);
        assert!(
            feasible.score(&turned, &target, &tail_first)
                < feasible.score(&turned, &target, &nose_first)
        );
        // Without reversing, no directional preference falls back to forward (as Nav2).
        turned.vx_min = 0.0;
        assert_eq!(either.mode(&turned), PathAngleMode::ForwardPreference);
        assert_eq!(PathAngleMode::from_number(2), Some(feasible.mode));
        assert_eq!(PathAngleMode::from_number(3), None);
    }

    #[test]
    fn prefer_forward_costs_reversing() {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let d = data(&path, &valid, Pose::default(), Pose::new(30.0, 0.0, 0.0));
        let c = PreferForwardCritic::default();
        assert_eq!(c.score(&d, &rollout(1.0, 0.0, 0.0, 10)), 0.0);
        assert!((c.score(&d, &rollout(-0.5, 0.0, 0.0, 10)) - 1.25).abs() < 1e-9);
    }

    #[test]
    fn cost_critic_flags_a_collision_and_charges_near_misses() {
        let (w, h) = (100, 100);
        let mut cells = vec![0u8; w * h];
        for cy in 0..h {
            cells[cy * w + 30] = LETHAL; // a wall at x = 3 m
            cells[cy * w + 25] = 100; // inflation before it
        }
        let map = Costmap::new((0.0, -5.0), 0.1, w, h, cells).expect("map");
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let footprint = [(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)];
        let mut d = data(&path, &valid, Pose::default(), Pose::new(30.0, 0.0, 0.0));
        d.costmap = Some(&map);
        d.footprint = &footprint;
        let c = CostCritic::default();
        let slow = c.score(&d, &rollout(0.1, 0.0, 0.0, 56)); // 0.28 m: nothing under it
        assert_eq!(slow, Score::default());
        let fast = c.score(&d, &rollout(1.0, 0.0, 0.0, 56));
        assert!(fast.collided, "the nose reaches the wall");
        // 1e6 x 3.81 / 254 / 28 checked poses, as Nav2 scales it.
        assert!(fast.cost > 500.0, "{}", fast.cost);
        // Without a costmap the critic is silent.
        d.costmap = None;
        assert_eq!(c.score(&d, &rollout(1.0, 0.0, 0.0, 56)), Score::default());
    }

    #[test]
    fn inflation_cost_follows_nav2() {
        assert_eq!(inflation_cost(0.0, 0.25, 3.0), 254.0);
        assert_eq!(inflation_cost(0.2, 0.25, 3.0), 253.0);
        assert!((inflation_cost(1.25, 0.25, 3.0) - 252.0 * (-3.0f64).exp()).abs() < 1e-9);
    }

    #[test]
    fn closest_path_point_rounds_to_the_nearer() {
        let along = [0.0, 1.0, 2.0, 3.0];
        assert_eq!(closest_path_point(&along, 1.4, 0), 1);
        assert_eq!(closest_path_point(&along, 1.6, 0), 2);
        assert_eq!(closest_path_point(&along, 9.0, 0), 3);
    }

    #[test]
    fn furthest_reached_takes_the_furthest_end() {
        let path = straight_path();
        let rollouts = [rollout(0.5, 0.0, 0.0, 20), rollout(1.0, 0.0, 0.0, 20)];
        assert_eq!(furthest_reached(&path, &rollouts), 20); // 1.0 m
    }

    #[test]
    fn path_clear_marks_blocked_points() {
        let (w, h) = (100, 100);
        let mut cells = vec![0u8; w * h];
        cells[50 * w + 10] = LETHAL; // (1.0, 0.0)
        let map = Costmap::new((0.0, -5.0), 0.1, w, h, cells).expect("map");
        let path = straight_path();
        let clear = path_clear(&path, Some(&map), false);
        assert_eq!(clear.len(), path.len() - 1);
        assert!(!clear[20] && clear[19] && clear[22]);
        assert!(path_clear(&path, None, false).iter().all(|&c| c));
    }

    #[test]
    fn every_name_builds_its_critic() {
        for name in NAMES.iter().chain(&OWN) {
            assert_eq!(Critic::by_name(name).map(|c| c.name()), Some(*name));
        }
        assert!(Critic::by_name("TwirlingCritic").is_none());
    }

    /// A straight lane along y = 0 from x = -5 to 35 m with these edges.
    fn road_lane(edges: Edges) -> Lane {
        Lane::new((0..=80).map(|i| LanePoint {
            x: f64::from(i) * 0.5 - 5.0,
            y: 0.0,
            edges,
        }))
        .expect("a lane")
    }

    /// The road world's: lines 1.75 m either side, kerbs 2.25 m.
    fn road_edges() -> Edges {
        Edges::from_numbers(2.25, 2.25, 1.75, 1.75)
    }

    /// The car's outline, padded (vehicle.yaml's, as the node gets it).
    const OUTLINE: [(f64, f64); 4] = [(1.85, 0.55), (1.85, -0.55), (-0.25, -0.55), (-0.25, 0.55)];

    /// LaneCritic's cost for a sample driving straight along y = `y0`.
    fn lane_cost(lane: Option<&Lane>, y0: f64) -> f64 {
        let path = straight_path();
        let valid = vec![true; path.len() - 1];
        let mut d = data(&path, &valid, Pose::default(), Pose::new(30.0, 0.0, 0.0));
        d.footprint = &OUTLINE;
        d.lane = lane;
        let critics = [Critic::Lane(LaneCritic::default())];
        Prepared::new(&critics, d)
            .score(&rollout(1.0, y0, 0.0, 56))
            .cost
    }

    #[test]
    fn lane_critic_is_silent_inside_the_lines() {
        let lane = road_lane(road_edges());
        assert_eq!(lane_cost(Some(&lane), 0.0), 0.0);
        // The outline's side at 1.75 m: on the line, not over it.
        assert_eq!(lane_cost(Some(&lane), 1.2), 0.0);
        assert_eq!(lane_cost(Some(&lane), -1.2), 0.0);
    }

    #[test]
    fn lane_critic_charges_the_soft_edge_smoothly() {
        let lane = road_lane(road_edges());
        // 0.2 m over the left line all along: 0.2² x 100.
        let over = lane_cost(Some(&lane), 1.4);
        assert!((over - 4.0).abs() < 1e-9, "{over}");
        // Twice as far over, four times the cost (squared): smooth at the line.
        let further = lane_cost(Some(&lane), 1.6);
        assert!((further - 16.0).abs() < 1e-9, "{further}");
        // The right side the same.
        assert!((lane_cost(Some(&lane), -1.4) - 4.0).abs() < 1e-9);
    }

    #[test]
    fn lane_critic_charges_the_hard_edge_far_more() {
        let lane = road_lane(road_edges());
        // 0.1 m over the kerb (the outline at 2.35 m): 0.6² x 100 soft, (1 + 0.1) x 1000 hard.
        let kerb = lane_cost(Some(&lane), 1.8);
        assert!((kerb - (36.0 + 1100.0)).abs() < 1e-6, "{kerb}");
        // The outline counts, not the centre: the centre is 0.45 m inside the kerb.
        assert!(kerb > 30.0 * lane_cost(Some(&lane), 1.6));
    }

    #[test]
    fn lane_critic_is_silent_without_a_lane_or_its_edges() {
        assert_eq!(lane_cost(None, 1.8), 0.0);
        let nan = Edges::from_numbers(f64::NAN, f64::NAN, f64::NAN, f64::NAN);
        let points = (0..10).map(|i| LanePoint {
            x: f64::from(i),
            y: 0.0,
            edges: nan,
        });
        assert!(Lane::new(points).is_none(), "NaN edges: no lane to keep to");
        // Only the kerbs known: the lines cost nothing, the kerb still does.
        let kerbs = road_lane(Edges::from_numbers(2.25, 2.25, f64::NAN, f64::NAN));
        assert_eq!(lane_cost(Some(&kerbs), 1.6), 0.0);
        assert!(lane_cost(Some(&kerbs), 1.8) > 1000.0);
    }
}

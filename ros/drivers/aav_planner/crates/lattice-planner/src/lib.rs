//! Our own path planner: Nav2's Smac State Lattice and Hybrid-A* planners, ported to Rust
//! (no ROS).
//!
//! Given a costmap (the `costmap` crate's `Grid`) and the car's outline, `Planner::plan`
//! finds a path a car can drive from a start pose to a goal pose: A* over the car's moves
//! (forward, and backward if allowed), finishing with a Reeds-Shepp or Dubins curve when
//! one is clear, then smoothed. The moves are our lattice primitives file
//! (`Planner::new`, the State Lattice) or Hybrid-A*'s short arcs at full lock
//! (`Planner::hybrid`); everything else is the same code. The costs and penalties are
//! nav2.yaml's (`Settings::default`, `Settings::hybrid`). PROVENANCE.md says what came
//! from where and what changed.
//!
//! ```text
//! start ─▶ search (search.rs) ─▶ curve to the goal (curves.rs) ─▶ smoother (smoother.rs) ─▶ path
//!            │  moves: lattice.rs        fits?: collision.rs        how far?: heuristic.rs
//!            │  (the file, or Lattice::hybrid's arcs)
//! ```
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod collision;
pub mod curves;
pub mod heuristic;
pub mod jerk_pass;
pub mod lane;
pub mod lattice;
pub mod pass;
pub mod scenes;
pub mod search;
pub mod smoother;
pub mod spec;
pub mod steps;

use std::sync::{Arc, Mutex};
use std::time::Instant;

use costmap::{footprint, Grid, Inflation};

pub use collision::Checker;
pub use curves::CurveKind;
pub use lattice::{Lattice, LatticeError};
pub use search::Expansion;
pub use smoother::SmootherSettings;
pub use spec::planner_from_json;

use heuristic::DistanceTable;
use search::Search;

/// A pose: x, y (m or cells), yaw (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left of x.
    pub yaw: f64,
}

impl Pose {
    /// A pose at (`x`, `y`), facing `yaw`.
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }
}

/// The planner's settings: nav2.yaml's `Lattice` block (the defaults are its values).
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// The car may reverse (reversing:=true): Reeds-Shepp curves and backward moves.
    pub allow_reverse: bool,
    /// Unknown cells may be driven through.
    pub allow_unknown: bool,
    /// m: a path ending this close to the goal will do, if the goal can't be reached.
    pub tolerance: f64,
    /// The most nodes a search expands.
    pub max_iterations: usize,
    /// Iterations to keep looking once within the tolerance.
    pub max_on_approach_iterations: usize,
    /// s.
    pub max_planning_time: f64,
    /// Iterations between checks of the clock and of a cancel.
    pub terminal_checking_interval: usize,
    /// Nav2's: a backward move costs this many times its length.
    pub reverse_penalty: f64,
    /// Nav2's: a change of turn (left to right, say) costs this.
    pub change_penalty: f64,
    /// m of straight driving a change between forward and back costs (ours, not Nav2's:
    /// Nav2 prices a backward move, never the stop and start between gears). 0: Nav2's
    /// way. TODO 174: a 6.5 m street's turn planned 6 cusps.
    pub cusp_penalty: f64,
    /// Nav2's: a turning move costs this many times its length.
    pub non_straight_penalty: f64,
    /// Nav2's: how much the costmap's cost weighs.
    pub cost_penalty: f64,
    /// Nav2's: a nudge toward turning later rather than sooner.
    pub retrospective_penalty: f64,
    /// Nav2's: a turn on the spot (no Ackermann car makes one).
    pub rotation_penalty: f64,
    /// Nav2's: how often to try a curve straight to the goal, nearer the goal more often.
    pub analytic_expansion_ratio: f64,
    /// m.
    pub analytic_expansion_max_length: f64,
    /// Nav2's: such a curve may cross no cell dearer than this.
    pub analytic_expansion_max_cost: f64,
    /// Nav2's: near the goal, `analytic_expansion_max_cost` is let off.
    pub analytic_expansion_max_cost_override: bool,
    /// m: the side of the curve-length table round the goal.
    pub lookup_table_size: f64,
    /// Smooth the search's path.
    pub smooth_path: bool,
    /// The smoother's settings.
    pub smoother: SmootherSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            allow_reverse: false,
            allow_unknown: true,
            tolerance: 0.5,
            max_iterations: 1_000_000,
            max_on_approach_iterations: 1000,
            max_planning_time: 5.0,
            terminal_checking_interval: 5000,
            reverse_penalty: 2.0,
            change_penalty: 0.05,
            cusp_penalty: 0.0,
            non_straight_penalty: 1.05,
            cost_penalty: 2.0,
            retrospective_penalty: 0.015,
            rotation_penalty: 5.0,
            analytic_expansion_ratio: 3.5,
            analytic_expansion_max_length: 3.0,
            analytic_expansion_max_cost: 200.0,
            analytic_expansion_max_cost_override: false,
            lookup_table_size: 20.0,
            smooth_path: true,
            smoother: SmootherSettings::default(),
        }
    }
}

impl Settings {
    /// nav2.yaml's Hybrid-A* block (`GridBased`, SmacPlannerHybrid): its penalties differ
    /// from the lattice's (no change penalty, a dearer turn).
    #[must_use]
    pub fn hybrid() -> Self {
        Self {
            change_penalty: 0.0,
            non_straight_penalty: 1.2,
            ..Self::default()
        }
    }
}

/// Hybrid-A*'s own settings (nav2.yaml's `GridBased` block); the motion model is
/// `Settings::allow_reverse`: Reeds-Shepp (`REEDS_SHEPP`) if the car may reverse, else
/// Dubins (`DUBIN`).
#[derive(Debug, Clone, PartialEq)]
pub struct Hybrid {
    /// m: the radius the arcs turn at (vehicle.plan_turning_radius).
    pub minimum_turning_radius: f64,
    /// Headings: each node's heading is one of these many bins.
    pub angle_quantization_bins: usize,
    /// Also the gentler turns between straight and full lock.
    pub allow_primitive_interpolation: bool,
    /// The obstacle estimate on 2 x 2 blocks (Nav2's default).
    pub downsample_obstacle_heuristic: bool,
}

impl Hybrid {
    /// nav2.yaml's block for a car turning at `minimum_turning_radius` m.
    #[must_use]
    pub fn new(minimum_turning_radius: f64) -> Self {
        Self {
            minimum_turning_radius,
            angle_quantization_bins: 72,
            allow_primitive_interpolation: false,
            downsample_obstacle_heuristic: true,
        }
    }
}

/// Why there is no path (Nav2's `ComputePathToPose` error codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanError {
    /// The start is off the costmap.
    StartOutsideMap,
    /// The goal is off the costmap.
    GoalOutsideMap,
    /// The car at the start hits something.
    StartOccupied,
    /// The car at the goal would hit something.
    GoalOccupied,
    /// Out of time or iterations.
    Timeout,
    /// The search ran out of places to try.
    NoValidPath,
    /// Cancelled by the caller.
    Cancelled,
    /// The costmap's cells aren't the primitives file's.
    WrongResolution,
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            Self::StartOutsideMap => "the start is off the costmap",
            Self::GoalOutsideMap => "the goal is off the costmap",
            Self::StartOccupied => "the car doesn't fit at the start",
            Self::GoalOccupied => "the car doesn't fit at the goal",
            Self::Timeout => "out of time or iterations",
            Self::NoValidPath => "no path",
            Self::Cancelled => "cancelled",
            Self::WrongResolution => "the costmap's cells aren't the lattice file's",
        };
        f.write_str(text)
    }
}

impl std::error::Error for PlanError {}

/// A path found.
#[derive(Debug, Clone)]
pub struct Plan {
    /// World poses (m, rad), start first. The car faces `yaw`: on a reversing stretch the
    /// path runs behind it.
    pub poses: Vec<Pose>,
    /// The search's poses, before smoothing.
    pub raw: Vec<Pose>,
    /// How many nodes the search expanded.
    pub iterations: usize,
    /// s of wall time: the search, and the whole plan with smoothing.
    pub search_time: f64,
    /// s of wall time for the whole plan.
    pub total_time: f64,
}

/// The moves and the tables made from them, for one size of cell.
#[derive(Debug)]
struct Tables {
    lattice: Lattice,
    checker: Checker,
    distances: DistanceTable,
}

impl Tables {
    fn new(
        lattice: Lattice,
        settings: &Settings,
        outline: &[(f64, f64)],
        inflation: Option<&Inflation>,
        kind: CurveKind,
    ) -> Self {
        let possible = inflation.and_then(|i| {
            let (_, circumscribed) = footprint::radii(outline);
            (i.inflation_radius >= circumscribed)
                .then(|| i.cost(circumscribed / lattice.resolution, lattice.resolution))
        });
        let checker = Checker::new(outline, possible, settings.allow_unknown);
        let radius = lattice.turning_radius / lattice.resolution;
        let cells = (settings.lookup_table_size / lattice.resolution) as usize;
        let distances = DistanceTable::new(kind, radius, &lattice.headings, cells);
        Self {
            lattice,
            checker,
            distances,
        }
    }
}

/// Where the moves come from.
#[derive(Debug)]
enum Moves {
    /// The primitives file: made for one size of cell.
    Lattice(Arc<Tables>),
    /// Hybrid-A*'s arcs: made again for a costmap with other cells (as Nav2 does when the
    /// costmap's resolution changes), and kept.
    Hybrid {
        spec: Hybrid,
        tables: Mutex<Arc<Tables>>,
    },
}

/// The planner: the moves, the car, and the tables built once.
#[derive(Debug)]
pub struct Planner {
    settings: Settings,
    outline: Vec<(f64, f64)>,
    inflation: Option<Inflation>,
    kind: CurveKind,
    moves: Moves,
}

impl Planner {
    /// The State Lattice on a primitives file. `outline`: the car's footprint (m,
    /// base_link), already padded. `inflation`: the costmap's inflation settings (its cost
    /// at the circumscribed radius lets most poses skip the outline check); None if the
    /// costmap has none.
    #[must_use]
    pub fn new(
        lattice: Lattice,
        settings: Settings,
        outline: &[(f64, f64)],
        inflation: Option<&Inflation>,
    ) -> Self {
        let kind = curve_kind(&settings);
        let tables = Tables::new(lattice, &settings, outline, inflation, kind);
        Self {
            settings,
            outline: outline.to_vec(),
            inflation: inflation.copied(),
            kind,
            moves: Moves::Lattice(Arc::new(tables)),
        }
    }

    /// Hybrid-A* (Nav2's SmacPlannerHybrid), its tables made for `resolution` m cells (the
    /// global costmap's; another costmap's makes them again). Reeds-Shepp if
    /// `settings.allow_reverse`, else Dubins.
    ///
    /// # Errors
    ///
    /// [`LatticeError`] if the spec can't make moves (see [`Lattice::hybrid`]).
    pub fn hybrid(
        spec: Hybrid,
        settings: Settings,
        outline: &[(f64, f64)],
        inflation: Option<&Inflation>,
        resolution: f64,
    ) -> Result<Self, LatticeError> {
        let kind = curve_kind(&settings);
        let lattice = Lattice::hybrid(
            spec.minimum_turning_radius,
            spec.angle_quantization_bins,
            resolution,
            spec.allow_primitive_interpolation,
        )?;
        let tables = Tables::new(lattice, &settings, outline, inflation, kind);
        Ok(Self {
            settings,
            outline: outline.to_vec(),
            inflation: inflation.copied(),
            kind,
            moves: Moves::Hybrid {
                spec,
                tables: Mutex::new(Arc::new(tables)),
            },
        })
    }

    /// The settings it plans with.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The search's expansion: the lattice's or Hybrid-A*'s.
    pub fn expansion(&self) -> Expansion {
        match &self.moves {
            Moves::Lattice(_) => Expansion::Lattice,
            Moves::Hybrid { spec, .. } => Expansion::Hybrid {
                downsample_obstacle_heuristic: spec.downsample_obstacle_heuristic,
            },
        }
    }

    /// m: the radius the moves and the curves to the goal turn at.
    pub fn turning_radius(&self) -> f64 {
        match &self.moves {
            Moves::Lattice(t) => t.lattice.turning_radius,
            Moves::Hybrid { spec, .. } => spec.minimum_turning_radius,
        }
    }

    /// m: half the car's padded outline across (its widest point from base_link's line).
    #[must_use]
    pub fn half_width(&self) -> f64 {
        self.outline.iter().map(|p| p.1.abs()).fold(0.0, f64::max)
    }

    /// Does the car fit at `pose` (world, on `grid`'s frame)? The search's own check: the
    /// padded outline against lethal and inscribed cells; off the grid it doesn't.
    ///
    /// # Errors
    ///
    /// [`PlanError::WrongResolution`] if the lattice's cells aren't the grid's.
    pub fn fits(&self, grid: &Grid, pose: &Pose) -> Result<bool, PlanError> {
        let tables = self.tables(grid)?;
        let x = (pose.x - grid.origin_x) / grid.resolution;
        let y = (pose.y - grid.origin_y) / grid.resolution;
        Ok(matches!(
            tables.checker.fit(grid, x, y, pose.yaw),
            collision::Fit::Free(_)
        ))
    }

    /// One line for the log: which search, its headings, cells and radius.
    pub fn describe(&self) -> String {
        let reversing = self.settings.allow_reverse;
        match &self.moves {
            Moves::Lattice(t) => format!(
                "State Lattice: {} headings, {} m cells, turning radius {} m, reversing {reversing}",
                t.lattice.headings.len(),
                t.lattice.resolution,
                t.lattice.turning_radius,
            ),
            Moves::Hybrid { spec, .. } => format!(
                "Hybrid-A* ({}): {} headings, turning radius {} m, primitive interpolation {}, \
                 downsampled obstacle estimate {}",
                if reversing { "REEDS_SHEPP" } else { "DUBIN" },
                spec.angle_quantization_bins,
                spec.minimum_turning_radius,
                spec.allow_primitive_interpolation,
                spec.downsample_obstacle_heuristic,
            ),
        }
    }

    /// The tables for `grid`'s cells: the lattice's must match; Hybrid-A*'s are made
    /// again for other cells.
    fn tables(&self, grid: &Grid) -> Result<Arc<Tables>, PlanError> {
        let fits = |t: &Tables| (grid.resolution - t.lattice.resolution).abs() <= 1e-6;
        match &self.moves {
            Moves::Lattice(t) if fits(t) => Ok(Arc::clone(t)),
            Moves::Lattice(_) => Err(PlanError::WrongResolution),
            Moves::Hybrid { spec, tables } => {
                // A plan that panicked mid-build leaves no half-made tables: the lock's
                // value is always a whole set.
                let mut held = tables.lock().unwrap_or_else(|e| e.into_inner());
                if !fits(&held) {
                    let lattice = Lattice::hybrid(
                        spec.minimum_turning_radius,
                        spec.angle_quantization_bins,
                        grid.resolution,
                        spec.allow_primitive_interpolation,
                    )
                    .map_err(|_| PlanError::WrongResolution)?;
                    *held = Arc::new(Tables::new(
                        lattice,
                        &self.settings,
                        &self.outline,
                        self.inflation.as_ref(),
                        self.kind,
                    ));
                }
                Ok(Arc::clone(&held))
            }
        }
    }

    /// Plan from `start` to `goal` (world poses on `grid`'s frame). `cancelled` is asked
    /// now and then; true stops the search.
    ///
    /// # Errors
    ///
    /// [`PlanError`]: why there is no path.
    pub fn plan(
        &self,
        grid: &Grid,
        start: &Pose,
        goal: &Pose,
        cancelled: &dyn Fn() -> bool,
    ) -> Result<Plan, PlanError> {
        let began = Instant::now();
        let tables = self.tables(grid)?;
        let to_cells = |p: &Pose| {
            (
                (p.x - grid.origin_x) / grid.resolution,
                (p.y - grid.origin_y) / grid.resolution,
            )
        };
        let inside = |(x, y): (f64, f64)| {
            x >= 0.0 && y >= 0.0 && x < grid.width as f64 && y < grid.height as f64
        };
        let (sx, sy) = to_cells(start);
        if !inside((sx, sy)) {
            return Err(PlanError::StartOutsideMap);
        }
        let (gx, gy) = to_cells(goal);
        if !inside((gx, gy)) {
            return Err(PlanError::GoalOutsideMap);
        }
        // Start and goal in the same cell: the start, facing the goal's way (as Nav2).
        if sx.floor() == gx.floor() && sy.floor() == gy.floor() {
            let only = vec![Pose::new(start.x, start.y, goal.yaw)];
            return Ok(Plan {
                poses: only.clone(),
                raw: only,
                iterations: 0,
                search_time: 0.0,
                total_time: began.elapsed().as_secs_f64(),
            });
        }
        let search = Search {
            lattice: &tables.lattice,
            settings: &self.settings,
            checker: &tables.checker,
            distances: &tables.distances,
            kind: self.kind,
            expansion: self.expansion(),
        };
        let found = search.run(
            grid,
            (sx, sy, tables.lattice.nearest_heading(start.yaw)),
            (gx, gy, tables.lattice.nearest_heading(goal.yaw)),
            cancelled,
        )?;
        let mut raw: Vec<Pose> = Vec::with_capacity(found.poses.len());
        for p in &found.poses {
            let w = Pose::new(
                grid.origin_x + p.x * grid.resolution,
                grid.origin_y + p.y * grid.resolution,
                p.yaw,
            );
            // Drop repeats (Nav2 does too).
            if raw.last().is_some_and(|l: &Pose| {
                (l.x - w.x).abs() < 1e-4
                    && (l.y - w.y).abs() < 1e-4
                    && curves::wrap(l.yaw - w.yaw).abs() < 1e-4
            }) {
                continue;
            }
            raw.push(w);
        }
        let search_time = began.elapsed().as_secs_f64();
        let mut poses = raw.clone();
        if self.settings.smooth_path && found.iterations > 1 {
            let left = self.settings.max_planning_time - search_time;
            smoother::smooth(
                &mut poses,
                grid,
                tables.lattice.turning_radius,
                &self.settings.smoother,
                left,
            );
        }
        Ok(Plan {
            poses,
            raw,
            iterations: found.iterations,
            search_time,
            total_time: began.elapsed().as_secs_f64(),
        })
    }

    /// Plan from `start` through each of `goals` in turn, as Nav2's planner_server does
    /// for `ComputePathThroughPoses`: one plan a leg, each leg starting at the last goal,
    /// joined into one path. On a failure, the leg's number (0 = start to the first goal)
    /// and why. No goals: [`PlanError::NoValidPath`] on leg 0.
    ///
    /// # Errors
    ///
    /// The failed leg's number and its [`PlanError`].
    pub fn plan_through(
        &self,
        grid: &Grid,
        start: &Pose,
        goals: &[Pose],
        cancelled: &dyn Fn() -> bool,
    ) -> Result<Plan, (usize, PlanError)> {
        if goals.is_empty() {
            return Err((0, PlanError::NoValidPath));
        }
        let mut whole = Plan {
            poses: Vec::new(),
            raw: Vec::new(),
            iterations: 0,
            search_time: 0.0,
            total_time: 0.0,
        };
        let mut from = *start;
        for (leg, goal) in goals.iter().enumerate() {
            let plan = self
                .plan(grid, &from, goal, cancelled)
                .map_err(|e| (leg, e))?;
            // Each leg after the first starts where the last one ended: drop the repeat.
            let skip = usize::from(leg > 0);
            whole.poses.extend(plan.poses.iter().skip(skip));
            whole.raw.extend(plan.raw.iter().skip(skip));
            whole.iterations += plan.iterations;
            whole.search_time += plan.search_time;
            whole.total_time += plan.total_time;
            from = *goal;
        }
        Ok(whole)
    }
}

fn curve_kind(settings: &Settings) -> CurveKind {
    if settings.allow_reverse {
        CurveKind::ReedsShepp
    } else {
        CurveKind::Dubins
    }
}

/// A path's length, m.
#[must_use]
pub fn length(poses: &[Pose]) -> f64 {
    poses
        .windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .sum()
}

/// The tightest turn along a path, as a radius in m: from each three poses about `span`
/// m apart, within each stretch between changes of direction (infinite if straight).
pub fn tightest_turn(poses: &[Pose], span: f64) -> f64 {
    let mut cuts = vec![0];
    cuts.extend((1..poses.len().saturating_sub(1)).filter(|&i| {
        let (a, b, c) = (&poses[i - 1], &poses[i], &poses[i + 1]);
        (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y) < 0.0
    }));
    cuts.push(poses.len().saturating_sub(1));
    cuts.windows(2)
        .map(|w| tightest_in(&poses[w[0]..=w[1]], span))
        .fold(f64::INFINITY, f64::min)
}

fn tightest_in(poses: &[Pose], span: f64) -> f64 {
    // The index about `span` m along from `from`, and whether there was that far to go.
    let ahead = |from: usize| {
        let mut d = 0.0;
        let mut j = from;
        while j + 1 < poses.len() && d < span {
            d += (poses[j + 1].x - poses[j].x).hypot(poses[j + 1].y - poses[j].y);
            j += 1;
        }
        (j, d >= span)
    };
    let mut tightest = f64::INFINITY;
    for i in 0..poses.len() {
        let (j, far1) = ahead(i);
        let (k, far2) = ahead(j);
        if !(far1 && far2) {
            break;
        }
        let (a, b, c) = (poses[i], poses[j], poses[k]);
        let ab = (b.x - a.x).hypot(b.y - a.y);
        let bc = (c.x - b.x).hypot(c.y - b.y);
        let ca = (a.x - c.x).hypot(a.y - c.y);
        let cross = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs();
        if cross > 1e-9 {
            tightest = tightest.min(ab * bc * ca / (2.0 * cross));
        }
    }
    tightest
}

/// How many times a path changes direction (forward to reverse or back).
#[must_use]
pub fn cusps(poses: &[Pose]) -> usize {
    poses
        .windows(3)
        .filter(|w| {
            (w[1].x - w[0].x) * (w[2].x - w[1].x) + (w[1].y - w[0].y) * (w[2].y - w[1].y) < 0.0
        })
        .count()
}

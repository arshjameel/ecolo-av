//! The search: Nav2's `AStarAlgorithm` with its `AnalyticExpansion`, for both of its car
//! nodes: `NodeLattice` (the State Lattice) and `NodeHybrid` (Hybrid-A*).
//!
//! A* over (cell, heading). From each node, every move in the lattice (and, when
//! reversing is allowed, the same moves driven backwards). The State Lattice's moves come
//! from the primitives file and are checked for the car's outline along the way;
//! Hybrid-A*'s are three short arcs (`Lattice::hybrid`), checked where they end. Every so
//! often it tries to finish with one Reeds-Shepp (or Dubins) curve straight to the goal.
//!
//! The two differ only where `Expansion` says: how a move is checked, what it costs, and
//! how the obstacle estimate is searched.

use std::collections::{BinaryHeap, HashMap};
use std::time::Instant;

use costmap::Grid;

use crate::collision::{Checker, Fit};
use crate::curves::{shortest, CurveKind};
use crate::heuristic::{DistanceTable, Obstacles};
use crate::lattice::{Lattice, Primitive};
use crate::{cusps, PlanError, Pose, Settings};

/// One node: a cell and heading the search reached.
#[derive(Debug, Clone)]
struct Node {
    /// cells (continuous), where the move that reached it best ended.
    x: f64,
    y: f64,
    heading: usize,
    /// The cost so far.
    g: f64,
    parent: Option<usize>,
    /// The move from the parent: (heading group, index in it, driven backwards).
    by: Option<(usize, usize, bool)>,
    visited: bool,
    /// The car's outline checked here, at this heading: once, however many moves end
    /// here (TODO 152). Every move to a node ends at the same place (the moves land on
    /// the grid), so the answer is the same each time.
    fit: Option<Fit>,
}

#[derive(Debug, Clone, Copy)]
struct Open {
    f: f64,
    node: usize,
}
impl PartialEq for Open {
    fn eq(&self, o: &Self) -> bool {
        self.f == o.f
    }
}
impl Eq for Open {}
impl PartialOrd for Open {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Open {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.f.total_cmp(&self.f)
    }
}

/// What the search found: the poses in cells (yaw in radians), start first.
#[derive(Debug, Clone)]
pub struct Found {
    /// The path, start first.
    pub poses: Vec<Pose>,
    /// How many nodes the search expanded.
    pub iterations: usize,
}

/// Which of Nav2's two car searches: how they expand differs, the rest is shared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Expansion {
    /// The State Lattice (`NodeLattice`): the primitives file's moves, each swept.
    Lattice,
    /// Hybrid-A* (`NodeHybrid`): short arcs at full lock, checked at their ends.
    Hybrid {
        /// The obstacle estimate on 2 x 2 blocks (`downsample_obstacle_heuristic`).
        downsample_obstacle_heuristic: bool,
    },
}

/// Which way a move turns, and whether it is driven backwards: Nav2's `TurnDirection`
/// (`FORWARD`, `LEFT`, `RIGHT`, `REVERSE`, `REV_LEFT`, `REV_RIGHT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Turn {
    /// Straight on.
    Straight,
    /// Turning left.
    Left,
    /// Turning right.
    Right,
}

/// The things a search needs that don't change between plans.
#[derive(Debug)]
pub struct Search<'a> {
    /// The moves.
    pub lattice: &'a Lattice,
    /// The penalties and limits.
    pub settings: &'a Settings,
    /// The car against the costmap.
    pub checker: &'a Checker,
    /// The obstacle-free distance estimate.
    pub distances: &'a DistanceTable,
    /// Dubins or Reeds-Shepp, for the analytic expansion.
    pub kind: CurveKind,
    /// The lattice's moves, or Hybrid-A*'s.
    pub expansion: Expansion,
}

/// Fast hashing for node keys (they are already well spread).
#[derive(Default)]
struct KeyHasher(u64);
impl std::hash::Hasher for KeyHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, _: &[u8]) {
        unreachable!("only u64 keys")
    }
    fn write_u64(&mut self, k: u64) {
        self.0 = k.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}
type Keys = HashMap<u64, usize, std::hash::BuildHasherDefault<KeyHasher>>;

struct State<'g> {
    grid: &'g Grid,
    nodes: Vec<Node>,
    keys: Keys,
    obstacles: Obstacles,
    goal: (f64, f64, usize),
    best: (f32, usize),
}

impl State<'_> {
    fn key(&self, x: f64, y: f64, heading: usize, headings: usize) -> Option<u64> {
        if x < 0.0 || y < 0.0 || x >= self.grid.width as f64 || y >= self.grid.height as f64 {
            return None;
        }
        let cell = (y as u64) * self.grid.width as u64 + x as u64;
        Some(cell * headings as u64 + heading as u64)
    }

    fn node(&mut self, key: u64, x: f64, y: f64, heading: usize) -> usize {
        if let Some(&i) = self.keys.get(&key) {
            return i;
        }
        self.nodes.push(Node {
            x,
            y,
            heading,
            g: f64::INFINITY,
            parent: None,
            by: None,
            visited: false,
            fit: None,
        });
        self.keys.insert(key, self.nodes.len() - 1);
        self.nodes.len() - 1
    }
}

impl Search<'_> {
    /// Search from `start` to `goal`, both in cells with lattice headings.
    ///
    /// # Errors
    ///
    /// [`PlanError`]: the start or goal off the map or occupied, out of time, cancelled,
    /// or no path.
    pub fn run(
        &self,
        grid: &Grid,
        start: (f64, f64, usize),
        goal: (f64, f64, usize),
        cancelled: &dyn Fn() -> bool,
    ) -> Result<Found, PlanError> {
        let started = Instant::now();
        let s = self.settings;
        let n = self.lattice.headings.len();
        let tolerance = (s.tolerance / grid.resolution) as f32;
        let mut st = State {
            grid,
            nodes: Vec::with_capacity(100_000),
            keys: Keys::with_capacity_and_hasher(100_000, Default::default()),
            obstacles: Obstacles::new(
                grid,
                (start.0 as usize, start.1 as usize),
                (goal.0 as usize, goal.1 as usize),
                s.cost_penalty,
                matches!(
                    self.expansion,
                    Expansion::Hybrid {
                        downsample_obstacle_heuristic: true
                    }
                ),
            ),
            goal,
            best: (f32::MAX, 0),
        };
        let goal_key = st
            .key(goal.0, goal.1, goal.2, n)
            .ok_or(PlanError::GoalOutsideMap)?;
        let start_key = st
            .key(start.0, start.1, start.2, n)
            .ok_or(PlanError::StartOutsideMap)?;
        if tolerance < 0.001
            && self
                .checker
                .fit(grid, goal.0, goal.1, self.lattice.headings[goal.2])
                == Fit::Blocked
        {
            return Err(PlanError::GoalOccupied);
        }
        let first = st.node(start_key, start.0, start.1, start.2);
        st.nodes[first].g = 0.0;
        let mut open = BinaryHeap::new();
        open.push(Open {
            f: 0.0,
            node: first,
        });

        let mut iterations = 0;
        let mut approach = 0;
        let mut analytic_wait = 0_i64;
        let mut closest = i64::MAX;
        while iterations < s.max_iterations {
            let Some(Open { node: id, .. }) = open.pop() else {
                break;
            };
            if iterations % s.terminal_checking_interval == 0 {
                if cancelled() {
                    return Err(PlanError::Cancelled);
                }
                if started.elapsed().as_secs_f64() >= s.max_planning_time {
                    return Err(PlanError::Timeout);
                }
            }
            if st.nodes[id].visited {
                continue;
            }
            iterations += 1;
            st.nodes[id].visited = true;

            if let Some(poses) = self.try_analytic(&mut st, id, &mut analytic_wait, &mut closest) {
                return Ok(Found { poses, iterations });
            }
            let here_key = st.key(st.nodes[id].x, st.nodes[id].y, st.nodes[id].heading, n);
            if here_key == Some(goal_key) {
                return Ok(Found {
                    poses: self.backtrace(&st, id),
                    iterations,
                });
            }
            if st.best.0 < tolerance {
                approach += 1;
                if approach >= s.max_on_approach_iterations {
                    return Ok(Found {
                        poses: self.backtrace(&st, st.best.1),
                        iterations,
                    });
                }
            }
            self.expand(&mut st, id, &mut open);
        }
        if st.best.0 < tolerance {
            return Ok(Found {
                poses: self.backtrace(&st, st.best.1),
                iterations,
            });
        }
        if iterations == 1 {
            // Not one move from the start fitted.
            Err(PlanError::StartOccupied)
        } else if iterations >= s.max_iterations {
            Err(PlanError::Timeout)
        } else {
            Err(PlanError::NoValidPath)
        }
    }

    /// The estimate from a node to the goal (Nav2's `getHeuristicCost`): the larger of
    /// round-the-obstacles and the car's curve; the curve from the node's whole cell
    /// (the lattice) or its own pose (Hybrid-A*).
    fn heuristic(&self, st: &mut State, id: usize) -> f32 {
        let node = &st.nodes[id];
        let (cx, cy, h) = (node.x as usize, node.y as usize, node.heading);
        let at = match self.expansion {
            Expansion::Lattice => (cx as f64, cy as f64, h),
            Expansion::Hybrid { .. } => (node.x, node.y, h),
        };
        let obstacles = st.obstacles.get(st.grid, cx, cy);
        let curve = self.distances.get(at, st.goal, obstacles);
        let estimate = obstacles.max(curve);
        if estimate < st.best.0 {
            st.best = (estimate, id);
        }
        estimate
    }

    fn moves(&self, heading: usize) -> impl Iterator<Item = (usize, usize, bool)> + '_ {
        let forward = (0..self.lattice.moves[heading].len()).map(move |i| (heading, i, false));
        let back_heading = self.lattice.opposite(heading);
        let back = (0..self.lattice.moves[back_heading].len())
            .map(move |i| (back_heading, i, true))
            .filter(|_| self.settings.allow_reverse);
        forward.chain(back)
    }

    fn primitive(&self, (group, index, _): (usize, usize, bool)) -> &Primitive {
        &self.lattice.moves[group][index]
    }

    /// The highest cost along a move from (x, y), or None if the car doesn't fit on it
    /// (Nav2 checks the end, then a pose about every cell's diagonal along it).
    /// `end`: the check where the move ends (its node's, `Node::fit`).
    fn sweep(&self, grid: &Grid, x: f64, y: f64, by: (usize, usize, bool), end: Fit) -> Option<u8> {
        let prim = self.primitive(by);
        let res = self.lattice.resolution;
        let Fit::Free(mut worst) = end else {
            return None;
        };
        if matches!(self.expansion, Expansion::Hybrid { .. }) {
            // Hybrid-A*'s moves are about a cell's diagonal: the end is the whole check
            // (`NodeHybrid::isNodeValid`), and its cell's cost is the move's.
            return Some(worst);
        }
        let spacing = 2.0 * res * res;
        let mut last = (1e9, 1e9);
        for &(px, py, yaw) in &prim.poses {
            if (px - last.0).powi(2) + (py - last.1).powi(2) <= spacing {
                continue;
            }
            last = (px, py);
            let yaw = if by.2 {
                yaw + std::f64::consts::PI
            } else {
                yaw
            };
            match self.checker.fit(grid, x + px / res, y + py / res, yaw) {
                Fit::Free(c) => worst = worst.max(c),
                Fit::Blocked => return None,
            }
        }
        Some(worst)
    }

    /// Which way a move turns (Nav2's `TurnDirection`, with the backwards flag).
    fn turn(&self, by: (usize, usize, bool)) -> (Turn, bool) {
        let prim = self.primitive(by);
        let turn = if prim.arc_length < 0.001 {
            Turn::Straight
        } else if prim.left_turn {
            Turn::Left
        } else {
            Turn::Right
        };
        (turn, by.2)
    }

    /// What a move costs: Nav2's `NodeLattice::getTraversalCost` or
    /// `NodeHybrid::getTraversalCost`.
    fn travel(
        &self,
        from: Option<(usize, usize, bool)>,
        by: (usize, usize, bool),
        cost: u8,
    ) -> f64 {
        match self.expansion {
            Expansion::Lattice => self.travel_lattice(from, by, cost),
            Expansion::Hybrid { .. } => self.travel_hybrid(from, by, cost),
        }
    }

    /// Nav2's `NodeHybrid::getTraversalCost`: the move's length (its arc's, if it turns),
    /// dearer on expensive ground; a turn costs `non_straight_penalty` times as much, and
    /// `change_penalty` more when it isn't the turn before (left, right and straight, each
    /// forward or backwards: six kinds); backwards, `reverse_penalty` times all that.
    fn travel_hybrid(
        &self,
        from: Option<(usize, usize, bool)>,
        by: (usize, usize, bool),
        cost: u8,
    ) -> f64 {
        let s = self.settings;
        let res = self.lattice.resolution;
        let Some(from) = from else {
            // The first move from the start: the straight move's length, as Nav2's
            // `travel_distance_cost`.
            return self.lattice.moves[0][0].length / res;
        };
        let raw = self.primitive(by).length / res
            * ((1.0 - s.retrospective_penalty) + s.cost_penalty * f64::from(cost) / 252.0);
        let (turn, backwards) = self.turn(by);
        let mut travel = if turn == Turn::Straight {
            raw
        } else if self.turn(from) == (turn, backwards) {
            raw * s.non_straight_penalty
        } else {
            raw * (s.non_straight_penalty + s.change_penalty)
        };
        if backwards {
            travel *= s.reverse_penalty;
        }
        travel + self.cusp(from, by)
    }

    /// `cusp_penalty` (m, in cells) when a move changes gear from the move before.
    fn cusp(&self, from: (usize, usize, bool), by: (usize, usize, bool)) -> f64 {
        if from.2 == by.2 {
            0.0
        } else {
            self.settings.cusp_penalty / self.lattice.resolution
        }
    }

    /// Nav2's `NodeLattice::getTraversalCost`.
    fn travel_lattice(
        &self,
        from: Option<(usize, usize, bool)>,
        by: (usize, usize, bool),
        cost: u8,
    ) -> f64 {
        let s = self.settings;
        let prim = self.primitive(by);
        let length = prim.length / self.lattice.resolution;
        let Some(from) = from else {
            return length;
        };
        if prim.length < 1e-4 {
            return s.rotation_penalty * (1.0 + s.cost_penalty * f64::from(cost) / 252.0);
        }
        let raw =
            length * ((1.0 - s.retrospective_penalty) + s.cost_penalty * f64::from(cost) / 252.0);
        let mut travel = if prim.arc_length < 0.001 {
            raw
        } else if self.primitive(from).left_turn == prim.left_turn {
            raw * s.non_straight_penalty
        } else {
            raw * (s.non_straight_penalty + s.change_penalty)
        };
        if by.2 {
            travel *= s.reverse_penalty;
        }
        travel + self.cusp(from, by)
    }

    fn expand(&self, st: &mut State, id: usize, open: &mut BinaryHeap<Open>) {
        let n = self.lattice.headings.len();
        let res = self.lattice.resolution;
        let (x, y, heading, g, from) = {
            let node = &st.nodes[id];
            (node.x, node.y, node.heading, node.g, node.by)
        };
        for by in self.moves(heading) {
            let prim = self.primitive(by);
            let (ex, ey) = prim.end();
            let (nx, ny) = (x + ex / res, y + ey / res);
            let nh = if by.2 {
                self.lattice.opposite(prim.end_heading)
            } else {
                prim.end_heading
            };
            let Some(key) = st.key(nx, ny, nh, n) else {
                continue;
            };
            let next = st.node(key, nx, ny, nh);
            if st.nodes[next].visited {
                continue;
            }
            // The move at its cheapest (on free ground) can't beat the node's cost so far:
            // skip the outline check, as it can only add (TODO 152: the same paths, fewer
            // checks; a move's cost never falls as the ground's cost rises).
            if g + self.travel(from, by, 0) >= st.nodes[next].g {
                continue;
            }
            let end = match st.nodes[next].fit {
                Some(fit) => fit,
                None => {
                    let fit = self.checker.fit(st.grid, nx, ny, self.lattice.headings[nh]);
                    st.nodes[next].fit = Some(fit);
                    fit
                }
            };
            let Some(cost) = self.sweep(st.grid, x, y, by, end) else {
                continue;
            };
            let new_g = g + self.travel(from, by, cost);
            if new_g < st.nodes[next].g {
                let node = &mut st.nodes[next];
                node.g = new_g;
                node.parent = Some(id);
                node.by = Some(by);
                node.x = nx;
                node.y = ny;
                let f = new_g + f64::from(self.heuristic(st, next));
                open.push(Open { f, node: next });
            }
        }
    }

    /// The poses from the start to node `id` (cells), each move's poses in turn.
    fn backtrace(&self, st: &State, id: usize) -> Vec<Pose> {
        let res = self.lattice.resolution;
        let mut out = Vec::new();
        let mut at = Some(id);
        while let Some(i) = at {
            let node = &st.nodes[i];
            match node.by {
                Some(by) => {
                    let prim = self.primitive(by);
                    let (ex, ey) = prim.end();
                    let (sx, sy) = (node.x - ex / res, node.y - ey / res);
                    for &(px, py, yaw) in prim.poses.iter().rev() {
                        let yaw = if by.2 {
                            yaw + std::f64::consts::PI
                        } else {
                            yaw
                        };
                        out.push(Pose::new(
                            sx + px / res,
                            sy + py / res,
                            crate::curves::wrap(yaw),
                        ));
                    }
                }
                None => out.push(Pose::new(
                    node.x,
                    node.y,
                    crate::curves::wrap(self.lattice.headings[node.heading]),
                )),
            }
            at = node.parent;
        }
        out.reverse();
        out
    }

    /// Now and then, try one curve from this node (or an ancestor) to the goal (Nav2's
    /// `tryAnalyticExpansion`): the whole path if it is clear.
    fn try_analytic(
        &self,
        st: &mut State,
        id: usize,
        wait: &mut i64,
        closest: &mut i64,
    ) -> Option<Vec<Pose>> {
        let s = self.settings;
        let estimate = self.heuristic(st, id);
        *closest = (*closest).min(estimate as i64);
        let desired = ((*closest as f64 / s.analytic_expansion_ratio) as i64)
            .max(s.analytic_expansion_ratio.ceil() as i64);
        *wait = (*wait).min(desired);
        if *wait > 0 {
            *wait -= 1;
            return None;
        }
        *wait = desired;
        let radius = self.lattice.turning_radius / self.lattice.resolution;
        let mut anchor = id;
        let mut curve = self.analytic(st, anchor, radius)?;
        // Start the curve further back if it still works: fewer lattice wiggles.
        for _ in 0..8 {
            let mut back = Some(anchor);
            for _ in 0..5 {
                back = back.and_then(|b| st.nodes[b].parent);
            }
            let Some(earlier) = back else { break };
            match self.analytic(st, earlier, radius) {
                Some(c) => {
                    curve = c;
                    anchor = earlier;
                }
                None => break,
            }
        }
        // Then wider turns, up to four times the radius, if they cost less.
        let score = |c: &Vec<(Pose, u8)>| -> f64 {
            if c.len() < 2 {
                return f64::MAX;
            }
            let step = (c[1].0.x - c[0].0.x).hypot(c[1].0.y - c[0].0.y);
            c.iter()
                .map(|(_, cost)| step * (1.0 + s.cost_penalty * f64::from(*cost) / 252.0))
                .sum()
        };
        let mut best = score(&curve);
        let mut r = radius;
        while r < 4.0 * radius {
            r += 0.5;
            if let Some(c) = self.analytic(st, anchor, r) {
                let sc = score(&c);
                if sc <= best {
                    best = sc;
                    curve = c;
                }
            }
        }
        let mut poses = self.backtrace(st, anchor);
        let joined = poses.len().saturating_sub(2);
        poses.extend(curve.into_iter().map(|(p, _)| p));
        // With a price on changing gear, the curve's own changes would come free: the
        // shortest Reeds-Shepp curve near the goal often jiggles back and forth (TODO 174).
        // Leave those to the search, which prices each one.
        if s.cusp_penalty > 0.0 && cusps(&poses[joined..]) > 0 {
            return None;
        }
        Some(poses)
    }

    /// The curve from a node to the goal, sampled about every cell's diagonal, with each
    /// sample's cost; None if it is too long, too short or blocked (Nav2's `getAnalyticPath`).
    fn analytic(&self, st: &State, id: usize, radius: f64) -> Option<Vec<(Pose, u8)>> {
        let s = self.settings;
        let node = &st.nodes[id];
        let from = Pose::new(node.x, node.y, self.lattice.headings[node.heading]);
        let to = Pose::new(st.goal.0, st.goal.1, self.lattice.headings[st.goal.2]);
        let curve = shortest(self.kind, &from, &to, radius);
        let d = curve.length();
        let max_length = s.analytic_expansion_max_length / st.grid.resolution;
        if d > max_length || d < std::f64::consts::SQRT_2 {
            return None;
        }
        let steps = (d / std::f64::consts::SQRT_2).floor() as usize;
        let mut out = Vec::with_capacity(steps);
        for i in 1..=steps {
            let p = curve.sample(&from, i as f64 / steps as f64);
            match self.checker.fit(st.grid, p.x, p.y, p.yaw) {
                Fit::Free(c) => out.push((p, c)),
                Fit::Blocked => return None,
            }
        }
        // Refuse a curve that leaves expensive ground and then goes back into it.
        let max_cost = s.analytic_expansion_max_cost;
        if out.iter().any(|(_, c)| f64::from(*c) > max_cost) {
            let mut left_it = false;
            let mut back_in = false;
            for (_, c) in out.iter().rev() {
                if f64::from(*c) <= max_cost {
                    left_it = true;
                } else if left_it {
                    back_in = true;
                    break;
                }
            }
            let short = d < std::f64::consts::TAU * radius;
            if back_in && !(short && s.analytic_expansion_max_cost_override) {
                return None;
            }
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenes;

    /// Hybrid-A*'s moves on 0.25 m cells at 3.572 m, and the tables a search needs.
    fn hybrid(settings: &Settings) -> (Lattice, Checker, DistanceTable) {
        let lattice = Lattice::hybrid(3.572, 72, 0.25, false).expect("good settings");
        let checker = Checker::new(&scenes::outline(), None, true);
        let distances = DistanceTable::new(
            CurveKind::ReedsShepp,
            3.572 / 0.25,
            &lattice.headings,
            (settings.lookup_table_size / 0.25) as usize,
        );
        (lattice, checker, distances)
    }

    const HYBRID: Expansion = Expansion::Hybrid {
        downsample_obstacle_heuristic: true,
    };

    #[test]
    fn hybrid_travel_costs_are_nav2s() {
        let settings = Settings {
            allow_reverse: true,
            change_penalty: 0.1,
            ..Settings::hybrid()
        };
        let (lattice, checker, distances) = hybrid(&settings);
        let search = Search {
            lattice: &lattice,
            settings: &settings,
            checker: &checker,
            distances: &distances,
            kind: CurveKind::ReedsShepp,
            expansion: HYBRID,
        };
        // From heading 0: 0 straight, 1 left, 2 right; heading 36's, driven backwards.
        let (straight, left, right) = ((0, 0, false), (0, 1, false), (0, 2, false));
        let back_left = (36, 1, true);
        let chord = lattice.moves[0][0].length / 0.25;
        let arc = lattice.moves[0][1].length / 0.25;
        let reward = 1.0 - settings.retrospective_penalty;
        let close = |a: f64, b: f64| (a - b).abs() < 1e-9;
        // The first move: the straight move's length, whatever it is.
        assert!(close(search.travel(None, left, 100), chord));
        // Straight: its length, dearer on expensive ground.
        assert!(close(
            search.travel(Some(left), straight, 0),
            chord * reward
        ));
        let dear = chord * (reward + settings.cost_penalty * 126.0 / 252.0);
        assert!(close(search.travel(Some(left), straight, 126), dear));
        // The same turn again: non_straight_penalty; another: plus change_penalty.
        assert!(close(
            search.travel(Some(left), left, 0),
            arc * reward * 1.2
        ));
        assert!(close(
            search.travel(Some(right), left, 0),
            arc * reward * (1.2 + 0.1)
        ));
        // From straight into a turn is a change too (Nav2 compares the six kinds).
        assert!(close(
            search.travel(Some(straight), left, 0),
            arc * reward * (1.2 + 0.1)
        ));
        // Backwards: reverse_penalty times; forward left to backward left is a change.
        assert!(close(
            search.travel(Some(back_left), back_left, 0),
            arc * reward * 1.2 * 2.0
        ));
        assert!(close(
            search.travel(Some(left), back_left, 0),
            arc * reward * (1.2 + 0.1) * 2.0
        ));
        // The arc is longer than its chord: R x angle (two 5 degree bins).
        assert!(arc > chord && close(arc, 3.572 / 0.25 * 10f64.to_radians()));
    }

    #[test]
    fn a_change_of_gear_costs_the_cusp_penalty_in_cells() {
        let settings = Settings {
            allow_reverse: true,
            change_penalty: 0.1,
            cusp_penalty: 4.0,
            ..Settings::hybrid()
        };
        let (lattice, checker, distances) = hybrid(&settings);
        let search = Search {
            lattice: &lattice,
            settings: &settings,
            checker: &checker,
            distances: &distances,
            kind: CurveKind::ReedsShepp,
            expansion: HYBRID,
        };
        let (left, back_left) = ((0, 1, false), (36, 1, true));
        let arc = lattice.moves[0][1].length / 0.25;
        let reward = 1.0 - settings.retrospective_penalty;
        let close = |a: f64, b: f64| (a - b).abs() < 1e-9;
        // Forward to backward: the move's price, then 4 m (16 cells) for the stop and start.
        assert!(close(
            search.travel(Some(left), back_left, 0),
            arc * reward * (1.2 + 0.1) * 2.0 + 16.0
        ));
        // The same gear: nothing more.
        assert!(close(
            search.travel(Some(back_left), back_left, 0),
            arc * reward * 1.2 * 2.0
        ));
    }

    #[test]
    fn hybrid_moves_and_reversed_moves_are_nav2s_six() {
        let settings = Settings {
            allow_reverse: true,
            ..Settings::hybrid()
        };
        let (lattice, checker, distances) = hybrid(&settings);
        let search = Search {
            lattice: &lattice,
            settings: &settings,
            checker: &checker,
            distances: &distances,
            kind: CurveKind::ReedsShepp,
            expansion: HYBRID,
        };
        // Facing +x (heading 0): each move's end (m) and the car's heading there.
        let ends: Vec<(f64, f64, usize, bool)> = search
            .moves(0)
            .map(|by| {
                let prim = search.primitive(by);
                let (x, y) = prim.end();
                let heading = if by.2 {
                    lattice.opposite(prim.end_heading)
                } else {
                    prim.end_heading
                };
                (x, y, heading, by.2)
            })
            .collect();
        assert_eq!(ends.len(), 6);
        let forward: Vec<_> = ends.iter().filter(|e| !e.3).collect();
        let backward: Vec<_> = ends.iter().filter(|e| e.3).collect();
        assert!(forward.iter().all(|e| e.0 > 0.0));
        assert!(backward.iter().all(|e| e.0 < 0.0));
        let headings = |v: &[&(f64, f64, usize, bool)]| {
            let mut h: Vec<usize> = v.iter().map(|e| e.2).collect();
            h.sort_unstable();
            h
        };
        assert_eq!(headings(&forward), vec![0, 2, 70]);
        assert_eq!(headings(&backward), vec![0, 2, 70]);
        // Forward to the left (y > 0) turns the nose left (+2 bins); backwards to the left
        // turns it right (Nav2's REV_LEFT: -2 bins).
        let left = forward.iter().find(|e| e.1 > 0.01).expect("a LEFT move");
        assert_eq!(left.2, 2);
        let rev_left = backward
            .iter()
            .find(|e| e.1 > 0.01)
            .expect("a REV_LEFT move");
        assert_eq!(rev_left.2, 70);
        // Forward only (DUBIN): three.
        let forward_only = Settings::hybrid();
        let search = Search {
            settings: &forward_only,
            ..search
        };
        assert_eq!(search.moves(0).count(), 3);
    }
}

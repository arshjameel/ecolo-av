//! How far to the goal, at least: the two estimates Nav2's lattice takes the larger of.
//!
//! - **Round the obstacles** (`Obstacles`): a grid search out from the goal over the
//!   costmap, grown only as far as the search asks (Nav2's `getObstacleHeuristic`).
//! - **As the car drives** (`DistanceTable`): the shortest Reeds-Shepp or Dubins curve on
//!   open ground, kept in a table round the goal (Nav2's `precomputeDistanceHeuristic`).

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use costmap::cost::{INSCRIBED, LETHAL};
use costmap::Grid;

use crate::curves::{shortest, CurveKind};
use crate::Pose;

/// One cell waiting in the obstacle search: its priority and index.
#[derive(Debug, Clone, Copy)]
struct Entry {
    f: f32,
    index: u32,
}

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    // BinaryHeap pops the largest: the smallest f is the largest here.
    fn cmp(&self, other: &Self) -> Ordering {
        other.f.total_cmp(&self.f)
    }
}

/// The cost to the goal round the obstacles, cell by cell, in cells.
///
/// A cell's entry: 0 not reached yet, negative waiting (the cost so far), positive done.
#[derive(Debug, Clone)]
pub struct Obstacles {
    /// The search's grid: the costmap's, or half its size each way when downsampled.
    width: usize,
    height: usize,
    /// 1, or 2 when downsampled (Hybrid-A*'s `downsample_obstacle_heuristic`).
    scale: usize,
    table: Vec<f32>,
    queue: BinaryHeap<Entry>,
    cost_penalty: f32,
}

fn straight_line(index: usize, width: usize, to: (usize, usize)) -> f32 {
    let dx = (index % width) as f32 - to.0 as f32;
    let dy = (index / width) as f32 - to.1 as f32;
    dx.hypot(dy)
}

impl Obstacles {
    /// Start a new search from the goal's cell, aimed at the start's. `downsample`: search
    /// a grid of 2 x 2 blocks, each as cheap as its cheapest cell, and double the answer
    /// (Nav2's Hybrid-A* does, by default: a quarter of the cells, the same paths).
    #[must_use]
    pub fn new(
        grid: &Grid,
        start: (usize, usize),
        goal: (usize, usize),
        cost_penalty: f64,
        downsample: bool,
    ) -> Self {
        let scale = if downsample { 2 } else { 1 };
        let (width, height) = (grid.width.div_ceil(scale), grid.height.div_ceil(scale));
        let mut table = vec![0.0; width * height];
        // Nav2 aims the first search at the start's full-size cell, not its block.
        let goal_index = (goal.1 / scale) * width + goal.0 / scale;
        let mut queue = BinaryHeap::new();
        queue.push(Entry {
            f: straight_line(goal_index, width, start),
            index: goal_index as u32,
        });
        table[goal_index] = -0.00001;
        Self {
            width,
            height,
            scale,
            table,
            queue,
            cost_penalty: cost_penalty as f32,
        }
    }

    /// The cost from cell (x, y) to the goal; 0 if it can't be reached.
    pub fn get(&mut self, grid: &Grid, x: usize, y: usize) -> f32 {
        let scale = self.scale as f32;
        let (w, h) = (self.width, self.height);
        let (x, y) = (x / self.scale, y / self.scale);
        if x >= w || y >= h {
            return 0.0;
        }
        let target = y * w + x;
        if self.table[target] > 0.0 {
            return scale * self.table[target];
        }
        // Aim the waiting cells at this cell instead.
        let mut waiting = std::mem::take(&mut self.queue).into_vec();
        for e in &mut waiting {
            e.f = -self.table[e.index as usize] + straight_line(e.index as usize, w, (x, y));
        }
        self.queue = BinaryHeap::from(waiting);

        let steps: [(isize, isize, f32); 8] = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, std::f32::consts::SQRT_2),
            (-1, 1, std::f32::consts::SQRT_2),
            (1, -1, std::f32::consts::SQRT_2),
            (-1, -1, std::f32::consts::SQRT_2),
        ];
        while let Some(Entry { index, .. }) = self.queue.pop() {
            let index = index as usize;
            let here = self.table[index];
            if here > 0.0 {
                continue;
            }
            let here = -here;
            self.table[index] = here;
            let (cx, cy) = ((index % w) as isize, (index / w) as isize);
            for &(dx, dy, step) in &steps {
                let (nx, ny) = (cx + dx, cy + dy);
                // Nav2 keeps 4 cells clear of the edge.
                if nx <= 3 || ny <= 3 || nx >= w as isize - 3 || ny >= h as isize - 3 {
                    continue;
                }
                let next = ny as usize * w + nx as usize;
                let cost = self.cost(grid, nx as usize, ny as usize);
                if cost >= INSCRIBED {
                    continue;
                }
                let existing = self.table[next];
                if existing > 0.0 {
                    continue;
                }
                let new_cost = here + step * (1.0 + self.cost_penalty * f32::from(cost) / 252.0);
                if existing == 0.0 || -existing > new_cost {
                    self.table[next] = -new_cost;
                    self.queue.push(Entry {
                        f: new_cost + straight_line(next, w, (x, y)),
                        index: next as u32,
                    });
                }
            }
            if index == target {
                break;
            }
        }
        scale * self.table[target].max(0.0)
    }

    /// A search cell's cost: the costmap's, or the cheapest of its 2 x 2 block.
    fn cost(&self, grid: &Grid, x: usize, y: usize) -> u8 {
        if self.scale == 1 {
            return grid.data[y * grid.width + x];
        }
        let (fx, fy) = (x * self.scale, y * self.scale);
        (fy..(fy + self.scale).min(grid.height))
            .flat_map(|cy| (fx..(fx + self.scale).min(grid.width)).map(move |cx| (cx, cy)))
            .map(|(cx, cy)| grid.data[cy * grid.width + cx])
            .min()
            .unwrap_or(LETHAL)
    }
}

/// The curve's length from each nearby cell and heading to a goal at the origin facing
/// heading 0, in cells.
#[derive(Debug, Clone)]
pub struct DistanceTable {
    kind: CurveKind,
    /// cells.
    radius: f64,
    headings: Vec<f64>,
    /// cells across (odd).
    size: i64,
    values: Vec<f32>,
}

impl DistanceTable {
    /// `size_cells`: the table's width (made odd); `radius`: the turning radius in cells.
    #[must_use]
    pub fn new(kind: CurveKind, radius: f64, headings: &[f64], size_cells: usize) -> Self {
        let size = (size_cells as i64) | 1;
        let half = size / 2;
        let n = headings.len();
        let goal = Pose::new(0.0, 0.0, 0.0);
        let mut values = Vec::with_capacity((size * (half + 1)) as usize * n);
        for x in -half..=half {
            for y in 0..=half {
                for &angle in headings {
                    let from = Pose::new(x as f64, y as f64, angle);
                    values.push(shortest(kind, &from, &goal, radius).length() as f32);
                }
            }
        }
        Self {
            kind,
            radius,
            headings: headings.to_vec(),
            size,
            values,
        }
    }

    /// The estimate from a node (cells, a heading) to the goal (cells, a heading). The
    /// lattice gives its node's whole cell, Hybrid-A* its node's own pose (as Nav2's do).
    /// Off the table: the curve itself if `obstacles` is 0, else 0 (Nav2's choice).
    #[must_use]
    pub fn get(&self, node: (f64, f64, usize), goal: (f64, f64, usize), obstacles: f32) -> f32 {
        let n = self.headings.len();
        let (s, c) = self.headings[goal.2].sin_cos();
        let dx = node.0 - goal.0;
        let dy = node.1 - goal.1;
        let rx = (dx * c + dy * s).round() as i64;
        let ry = (-dx * s + dy * c).round() as i64;
        let dh = (node.2 + n - goal.2) % n;
        let half = self.size / 2;
        if rx.abs() < half && ry.abs() < half {
            // A goal's left mirrors its right: the heading mirrors too.
            let h = if ry < 0 { (n - dh) % n } else { dh };
            let index = ((rx + half) * (half + 1) + ry.abs()) as usize * n + h;
            self.values[index]
        } else if obstacles == 0.0 {
            let from = Pose::new(node.0, node.1, self.headings[node.2]);
            let to = Pose::new(goal.0, goal.1, self.headings[goal.2]);
            shortest(self.kind, &from, &to, self.radius).length() as f32
        } else {
            0.0
        }
    }
}

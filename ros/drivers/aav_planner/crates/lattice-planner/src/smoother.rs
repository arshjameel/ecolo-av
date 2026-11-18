//! Smoothing the search's path: Nav2's Smac `Smoother`.
//!
//! The path is cut where the car changes direction. Each piece longer than 10 poses is
//! pulled straighter (each point towards its neighbours, held to where it was), and its
//! ends are replaced by the shortest Dubins curve that joins the true start or goal pose,
//! if one is clear. Smoothing stops as soon as a point would land on an obstacle.

use std::time::Instant;

use costmap::cost::{INSCRIBED, NO_INFORMATION};
use costmap::Grid;

use crate::curves::{shortest, wrap, CurveKind};
use crate::Pose;

/// Nav2's `SmootherParams` (its defaults: nav2.yaml sets none).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SmootherSettings {
    /// Stop once a pass moves the path less than this.
    pub tolerance: f64,
    /// The most passes.
    pub max_iterations: usize,
    /// The weight on staying near the search's path.
    pub w_data: f64,
    /// The weight on smoothness.
    pub w_smooth: f64,
    /// Smooth the smoothed path again.
    pub do_refinement: bool,
    /// How many times, with `do_refinement`.
    pub refinement_num: usize,
}

impl Default for SmootherSettings {
    fn default() -> Self {
        Self {
            tolerance: 1e-10,
            max_iterations: 1000,
            w_data: 0.2,
            w_smooth: 0.3,
            do_refinement: true,
            refinement_num: 2,
        }
    }
}

/// The highest cost a smoothed point may sit on (Nav2's `MAX_NON_OBSTACLE`).
const MAX_NON_OBSTACLE: u8 = 252;

fn cost_at(grid: &Grid, x: f64, y: f64) -> u8 {
    grid.cell(x, y)
        .map_or(costmap::cost::LETHAL, |(cx, cy)| grid.get(cx, cy))
}

/// Smooth `path` (world poses, m) in place, within `budget` seconds. `radius`: the
/// turning radius, m. True if every piece converged.
pub fn smooth(
    path: &mut [Pose],
    grid: &Grid,
    radius: f64,
    settings: &SmootherSettings,
    budget: f64,
) -> bool {
    if settings.max_iterations == 0 || path.len() < 3 {
        return false;
    }
    let started = Instant::now();
    let mut ok = true;
    for (start, end) in segments(path) {
        if end - start <= 10 {
            continue;
        }
        let piece = &mut path[start..=end];
        let (first, last) = (piece[0], piece[piece.len() - 1]);
        let left = budget - started.elapsed().as_secs_f64();
        let mut refinements = 0;
        let (done, reversing) = smooth_piece(piece, grid, settings, left, &mut refinements);
        ok &= done;
        if done {
            start_boundary(&first, piece, grid, radius, reversing);
            end_boundary(&last, piece, grid, radius, reversing);
        }
    }
    ok
}

/// The path's pieces between changes of direction (index ranges, ends shared).
fn segments(path: &[Pose]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start = 0;
    for i in 1..path.len() - 1 {
        let (a, b, c) = (&path[i - 1], &path[i], &path[i + 1]);
        let (oa, ab) = ((b.x - a.x, b.y - a.y), (c.x - b.x, c.y - b.y));
        let cusp = oa.0 * ab.0 + oa.1 * ab.1 < 0.0;
        let spin = ab.0.abs() < 1e-4 && ab.1.abs() < 1e-4 && wrap(c.yaw - b.yaw).abs() > 1e-4;
        if cusp || spin {
            out.push((start, i));
            start = i;
        }
    }
    out.push((start, path.len() - 1));
    out
}

/// Nav2's `smoothImpl`: (converged, the piece is driven in reverse).
fn smooth_piece(
    path: &mut [Pose],
    grid: &Grid,
    s: &SmootherSettings,
    budget: f64,
    refinements: &mut usize,
) -> (bool, bool) {
    let started = Instant::now();
    let n = path.len();
    let mut new_path = path.to_vec();
    let mut last_good = path.to_vec();
    let mut iterations = 0;
    let mut change = s.tolerance;
    while change >= s.tolerance {
        iterations += 1;
        change = 0.0;
        if iterations >= s.max_iterations || started.elapsed().as_secs_f64() > budget {
            path.copy_from_slice(&last_good);
            return (false, orient(path));
        }
        for i in 1..n - 1 {
            for dim in 0..2 {
                let get = |p: &Pose| if dim == 0 { p.x } else { p.y };
                let x_i = get(&path[i]);
                let y_i = get(&new_path[i]);
                let y_m1 = get(&new_path[i - 1]);
                let y_p1 = get(&new_path[i + 1]);
                let moved = y_i + s.w_data * (x_i - y_i) + s.w_smooth * (y_p1 + y_m1 - 2.0 * y_i);
                if dim == 0 {
                    new_path[i].x = moved;
                } else {
                    new_path[i].y = moved;
                }
                change += (moved - y_i).abs();
            }
            let cost = cost_at(grid, new_path[i].x, new_path[i].y);
            if cost > MAX_NON_OBSTACLE && cost != NO_INFORMATION {
                path.copy_from_slice(&last_good);
                return (false, orient(path));
            }
        }
        last_good.copy_from_slice(&new_path);
    }
    if s.do_refinement && *refinements < s.refinement_num {
        *refinements += 1;
        smooth_piece(&mut new_path, grid, s, budget, refinements);
    }
    let reversing = orient(&mut new_path);
    path.copy_from_slice(&new_path);
    (true, reversing)
}

/// Point each pose along the path (backwards if the piece is reversed); returns whether it
/// is (Nav2's `updateApproximatePathOrientations`). The last pose keeps its heading.
fn orient(path: &mut [Pose]) -> bool {
    if path.len() < 3 {
        return false;
    }
    let along = (path[2].y - path[1].y).atan2(path[2].x - path[1].x);
    let reversing = wrap(path[1].yaw - along).abs() > std::f64::consts::FRAC_PI_2;
    for i in 0..path.len() - 1 {
        let (dx, dy) = (path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
        if dx.abs() < 1e-4 && dy.abs() < 1e-4 {
            continue;
        }
        let theta = dy.atan2(dx);
        path[i].yaw = wrap(if reversing {
            theta + std::f64::consts::PI
        } else {
            theta
        });
    }
    reversing
}

/// One way to join the true end: a Dubins curve to the pose `end_index` poses in.
struct Expansion {
    end_index: usize,
    original_length: f64,
    length: f64,
    blocked: bool,
    points: Vec<Pose>,
}

/// The poses at about one, two, pi and 2 pi turning radii along `path` from its first.
fn expansions<'a>(path: impl Iterator<Item = &'a Pose>, radius: f64) -> Vec<Expansion> {
    let marks = [
        radius,
        2.0 * radius,
        std::f64::consts::PI * radius,
        std::f64::consts::TAU * radius,
    ];
    let mut out = Vec::new();
    let mut travelled = 0.0;
    let mut last: Option<&Pose> = None;
    for (i, p) in path.enumerate() {
        if let Some(l) = last {
            travelled += (p.x - l.x).hypot(p.y - l.y);
        }
        last = Some(p);
        if out.len() < marks.len() && travelled >= marks[out.len()] {
            out.push(Expansion {
                end_index: i,
                original_length: travelled,
                length: 0.0,
                blocked: false,
                points: Vec::new(),
            });
        }
    }
    out
}

/// Fill in an expansion's Dubins curve from `from` to `to`.
fn fill(e: &mut Expansion, from: &Pose, to: &Pose, grid: &Grid, radius: f64) {
    let curve = shortest(CurveKind::Dubins, from, to, radius);
    if curve.length() > 2.0 * e.original_length {
        return;
    }
    let mut last = *from;
    for i in 0..=e.end_index {
        let p = curve.sample(from, i as f64 / e.end_index as f64);
        if cost_at(grid, p.x, p.y) >= INSCRIBED {
            e.blocked = true;
        }
        e.length += (p.x - last.x).hypot(p.y - last.y);
        last = p;
        e.points.push(p);
    }
}

fn best(expansions: &mut [Expansion]) -> Option<&mut Expansion> {
    expansions
        .iter_mut()
        .filter(|e| !e.blocked && e.end_index > 0 && e.length > 0.0)
        .min_by(|a, b| a.length.total_cmp(&b.length))
}

fn start_boundary(start: &Pose, path: &mut [Pose], grid: &Grid, radius: f64, reversing: bool) {
    let mut options = expansions(path.iter(), radius);
    for e in &mut options {
        let end = path[e.end_index];
        if reversing {
            fill(e, &end, start, grid, radius);
        } else {
            fill(e, start, &end, grid, radius);
        }
    }
    if let Some(e) = best(&mut options) {
        if reversing {
            e.points.reverse();
        }
        path[..e.points.len()].copy_from_slice(&e.points);
    }
}

fn end_boundary(end: &Pose, path: &mut [Pose], grid: &Grid, radius: f64, reversing: bool) {
    let n = path.len();
    let mut options = expansions(path.iter().rev(), radius);
    for e in &mut options {
        let from = path[n - e.end_index - 1];
        if reversing {
            fill(e, end, &from, grid, radius);
        } else {
            fill(e, &from, end, grid, radius);
        }
    }
    if let Some(e) = best(&mut options) {
        if reversing {
            e.points.reverse();
        }
        let first = n - e.end_index - 1;
        path[first..first + e.points.len()].copy_from_slice(&e.points);
    }
}

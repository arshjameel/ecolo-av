//! Nav2's `InflationLayer`: around each lethal cell, a cost that falls with the distance.
//!
//! Nav2 spreads the cost cell by cell from each obstacle (a brushfire). This finds each
//! cell's distance to the nearest lethal cell exactly instead (Felzenszwalb and
//! Huttenlocher's distance transform), then gives it Nav2's cost for that distance. The
//! brushfire can very rarely give a cell a slightly further obstacle; otherwise they agree.

use crate::cost::{INSCRIBED, LETHAL, NO_INFORMATION};
use crate::grid::Grid;

/// The inflation's settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inflation {
    /// m: cells further than this from every obstacle keep their cost.
    pub inflation_radius: f64,
    /// How fast the cost falls past the inscribed radius (1/m).
    pub cost_scaling_factor: f64,
    /// m: the padded outline's inscribed radius (footprint::radii).
    pub inscribed_radius: f64,
}

/// Far: no obstacle in this row or column (cells squared).
const FAR: f64 = 1e20;

impl Inflation {
    /// Nav2's `computeCost`: the cost `cells` cells from an obstacle.
    #[must_use]
    pub fn cost(&self, cells: f64, resolution: f64) -> u8 {
        if cells == 0.0 {
            LETHAL
        } else if cells * resolution <= self.inscribed_radius {
            INSCRIBED
        } else {
            let factor =
                (-self.cost_scaling_factor * (cells * resolution - self.inscribed_radius)).exp();
            (f64::from(INSCRIBED - 1) * factor) as u8
        }
    }

    /// Inflate round every lethal cell of the grid. A cell keeps the higher of its own
    /// cost and the inflation's; an unknown cell becomes inscribed or lethal only.
    pub fn apply(&self, grid: &mut Grid) {
        let reach = grid.cell_distance(self.inflation_radius) as f64;
        let squared = distances(grid);
        let resolution = grid.resolution;
        // One cost per squared distance in cells, as Nav2 caches one per (dx, dy).
        let mut costs: Vec<Option<u8>> = Vec::new();
        for (cell, d2) in grid.data.iter_mut().zip(squared) {
            if d2 >= FAR || d2.sqrt() > reach {
                continue;
            }
            let k = d2 as usize;
            if costs.len() <= k {
                costs.resize(k + 1, None);
            }
            let cost = *costs[k].get_or_insert_with(|| self.cost(d2.sqrt(), resolution));
            *cell = if *cell == NO_INFORMATION {
                if cost >= INSCRIBED {
                    cost
                } else {
                    NO_INFORMATION
                }
            } else {
                (*cell).max(cost)
            };
        }
    }
}

/// Each cell's squared distance (in cells) to the nearest lethal cell; FAR with none.
fn distances(grid: &Grid) -> Vec<f64> {
    let (w, h) = (grid.width, grid.height);
    let mut d: Vec<f64> = grid
        .data
        .iter()
        .map(|&c| if c == LETHAL { 0.0 } else { FAR })
        .collect();
    let mut line = Vec::with_capacity(w.max(h));
    let mut out = vec![0.0; w.max(h)];
    let mut scratch = Scratch::new(w.max(h));
    // Along each column, then along each row.
    for x in 0..w {
        line.clear();
        line.extend((0..h).map(|y| d[y * w + x]));
        transform_1d(&line, &mut out[..h], &mut scratch);
        for (y, v) in out[..h].iter().enumerate() {
            d[y * w + x] = *v;
        }
    }
    for y in 0..h {
        line.clear();
        line.extend_from_slice(&d[y * w..(y + 1) * w]);
        transform_1d(&line, &mut out[..w], &mut scratch);
        d[y * w..(y + 1) * w].copy_from_slice(&out[..w]);
    }
    d
}

struct Scratch {
    v: Vec<usize>,
    z: Vec<f64>,
}

impl Scratch {
    fn new(n: usize) -> Self {
        Self {
            v: vec![0; n],
            z: vec![0.0; n + 1],
        }
    }
}

/// The 1D squared distance transform: out[q] = min over p of (q - p)^2 + f[p] (the lower
/// envelope of parabolas).
fn transform_1d(f: &[f64], out: &mut [f64], s: &mut Scratch) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let (v, z) = (&mut s.v, &mut s.z);
    let mut k = 0usize;
    v[0] = 0;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    for q in 1..n {
        let fq = f[q] + (q * q) as f64;
        // z[0] is minus infinity, so this stops at k = 0 at the latest.
        let sect = loop {
            let p = v[k];
            let sect = (fq - (f[p] + (p * p) as f64)) / (2.0 * q as f64 - 2.0 * p as f64);
            if sect <= z[k] {
                k -= 1;
            } else {
                break sect;
            }
        };
        k += 1;
        v[k] = q;
        z[k] = sect;
        z[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, o) in out.iter_mut().enumerate() {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let p = v[k];
        let dq = q as f64 - p as f64;
        *o = dq * dq + f[p];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cost::FREE;

    /// The local costmap's (nav2.yaml): 2.0 m, scaling 3.0, the padded car's 0.25 m.
    const LOCAL: Inflation = Inflation {
        inflation_radius: 2.0,
        cost_scaling_factor: 3.0,
        inscribed_radius: 0.25,
    };

    #[test]
    fn nav2s_cost_curve() {
        assert_eq!(LOCAL.cost(0.0, 0.1), LETHAL);
        assert_eq!(LOCAL.cost(2.0, 0.1), INSCRIBED);
        assert_eq!(LOCAL.cost(2.5, 0.1), INSCRIBED);
        // 1 m out: 252 exp(-3 x 0.75) = 26.56 -> 26.
        assert_eq!(LOCAL.cost(10.0, 0.1), 26);
        assert!(LOCAL.cost(3.0, 0.1) > LOCAL.cost(4.0, 0.1));
    }

    #[test]
    fn exact_distances_against_brute_force() {
        let mut g = Grid::new((0.0, 0.0), 0.1, 37, 23, FREE);
        for &(x, y) in &[(3, 4), (30, 20), (18, 11), (0, 22), (36, 0)] {
            g.set(x, y, LETHAL);
        }
        let d = distances(&g);
        for cy in 0..g.height {
            for cx in 0..g.width {
                let brute = g
                    .data
                    .iter()
                    .enumerate()
                    .filter(|(_, &c)| c == LETHAL)
                    .map(|(i, _)| {
                        let (ox, oy) = ((i % g.width) as f64, (i / g.width) as f64);
                        (cx as f64 - ox).powi(2) + (cy as f64 - oy).powi(2)
                    })
                    .fold(f64::INFINITY, f64::min);
                assert_eq!(d[g.index(cx, cy)], brute, "cell ({cx}, {cy})");
            }
        }
    }

    #[test]
    fn one_obstacle_inflates_as_nav2() {
        let mut g = Grid::new((0.0, 0.0), 0.1, 61, 61, FREE);
        g.set(30, 30, LETHAL);
        LOCAL.apply(&mut g);
        assert_eq!(g.get(30, 30), LETHAL);
        assert_eq!(g.get(32, 30), INSCRIBED);
        assert_eq!(g.get(40, 30), 26);
        // 2.0 m is 20 cells: the last ring with a cost, then nothing.
        assert_eq!(g.get(50, 30), 1);
        assert_eq!(g.get(51, 30), FREE);
        assert_eq!(g.get(30 + 15, 30 + 15), FREE); // 21.2 cells
        assert_eq!(g.get(0, 0), FREE);
    }

    #[test]
    fn unknown_cells_only_become_inscribed_or_lethal() {
        let mut g = Grid::new((0.0, 0.0), 0.1, 30, 1, NO_INFORMATION);
        g.set(0, 0, LETHAL);
        LOCAL.apply(&mut g);
        assert_eq!(g.get(1, 0), INSCRIBED);
        assert_eq!(g.get(10, 0), NO_INFORMATION);
    }

    #[test]
    fn nothing_lethal_changes_nothing() {
        let mut g = Grid::new((0.0, 0.0), 0.1, 10, 10, 7);
        LOCAL.apply(&mut g);
        assert!(g.data.iter().all(|&c| c == 7));
    }
}

//! Does the car fit here? Nav2's `GridCollisionChecker` with its footprint check.
//!
//! The quick answer first: a cell cheaper than the cost at the car's circumscribed radius
//! (`possible_collision_cost`) is too far from every obstacle for any corner to touch
//! one. Only nearer than that is the outline's edge traced over the grid.

use std::f64::consts::TAU;

use costmap::cost::{INSCRIBED, LETHAL, NO_INFORMATION};
use costmap::Grid;

/// How many headings the outline is turned to ahead of time (Nav2's 72: every 5 degrees).
pub const OUTLINE_HEADINGS: usize = 72;

/// The car's outline, turned to every heading, and how to read the costs.
#[derive(Debug, Clone)]
pub struct Checker {
    turned: Vec<Vec<(f64, f64)>>,
    /// Below this cost the car can't touch anything (no check of the outline). None:
    /// always check the outline.
    possible_collision_cost: Option<u8>,
    allow_unknown: bool,
}

/// One pose's answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fit {
    /// The car fits; the cost of the cell under its origin.
    Free(u8),
    /// It doesn't.
    Blocked,
}

impl Checker {
    /// `outline`: the padded footprint, m, in base_link. `possible_collision_cost`: the
    /// inflation's cost at the circumscribed radius (None if the inflation is smaller
    /// than the car).
    pub fn new(
        outline: &[(f64, f64)],
        possible_collision_cost: Option<u8>,
        allow_unknown: bool,
    ) -> Self {
        let turned = (0..OUTLINE_HEADINGS)
            .map(|i| {
                let (s, c) = (TAU * i as f64 / OUTLINE_HEADINGS as f64).sin_cos();
                outline
                    .iter()
                    .map(|&(x, y)| (x * c - y * s, x * s + y * c))
                    .collect()
            })
            .collect();
        Self {
            turned,
            possible_collision_cost: possible_collision_cost.filter(|&c| c > 0),
            allow_unknown,
        }
    }

    /// The car at (x, y) in cells (continuous: cell (0, 0) spans [0, 1)), facing yaw.
    pub fn fit(&self, grid: &Grid, x: f64, y: f64, yaw: f64) -> Fit {
        if x < 0.0 || y < 0.0 || x >= grid.width as f64 || y >= grid.height as f64 {
            return Fit::Blocked;
        }
        let centre = grid.get(x as usize, y as usize);
        if self.possible_collision_cost.is_some_and(|p| centre < p) {
            return Fit::Free(centre);
        }
        if centre == NO_INFORMATION && !self.allow_unknown {
            return Fit::Blocked;
        }
        if centre == INSCRIBED || centre == LETHAL {
            return Fit::Blocked;
        }
        let bin = ((yaw.rem_euclid(TAU) / TAU * OUTLINE_HEADINGS as f64).round() as usize)
            % OUTLINE_HEADINGS;
        let (wx, wy) = (
            grid.origin_x + x * grid.resolution,
            grid.origin_y + y * grid.resolution,
        );
        let worst = outline_cost(grid, &self.turned[bin], wx, wy);
        if worst == NO_INFORMATION && self.allow_unknown {
            return Fit::Free(centre);
        }
        if worst >= LETHAL {
            Fit::Blocked
        } else {
            Fit::Free(centre)
        }
    }
}

/// The highest cost under the outline's edges at (wx, wy); lethal if a corner is off the
/// grid (Nav2's `FootprintCollisionChecker::footprintCost`).
///
/// No allocation: it runs about a million times a long campus plan (TODO 152), so each
/// corner is found as its edge is traced, and the first one kept to close the outline.
fn outline_cost(grid: &Grid, turned: &[(f64, f64)], wx: f64, wy: f64) -> u8 {
    let corner = |&(x, y): &(f64, f64)| grid.cell(wx + x, wy + y);
    let Some(first) = turned.first().map(corner) else {
        return 0;
    };
    let Some(mut a) = first else {
        return LETHAL;
    };
    let mut worst = 0;
    for next in turned[1..].iter().map(corner).chain(std::iter::once(first)) {
        let Some(b) = next else {
            return LETHAL;
        };
        grid.raytrace(a, b, usize::MAX, 0, |c| worst = worst.max(grid.data[c]));
        if worst == LETHAL {
            return LETHAL;
        }
        a = b;
    }
    worst
}

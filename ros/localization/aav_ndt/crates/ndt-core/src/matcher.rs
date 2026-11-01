//! Slide and turn a scan until it sits best on the map: Newton's method on the score,
//! made safe far from the answer, from big cubes to small.
//!
//! | Guard | Why |
//! |---|---|
//! | damping, when the Hessian isn't negative-definite | plain Newton then steps downhill, or anywhere |
//! | a step cap (0.1 m, 2°) | one step can't jump a whole cube |
//! | a backtracking line search (Armijo) | a step that lowers the score is halved until it doesn't |
//! | coarse to fine (2 m, then 1 m cubes) | a 1 m cube can't reach a start 1.5 m off |

use std::{error::Error, fmt};

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use crate::{
    map::{CubeMap, MapError, MapSettings},
    pose::wrap,
    score::{evaluate, Evaluation, Gauss},
    Point, Pose2,
};

/// How the matcher searches.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MatchSettings {
    /// Newton steps at most, per cube size.
    pub max_iterations: usize,
    /// The longest move in one step (m).
    pub step_m: f64,
    /// The biggest turn in one step (rad).
    pub step_rad: f64,
    /// Done when a step moves less than this (m)...
    pub done_m: f64,
    /// ...and turns less than this (rad).
    pub done_rad: f64,
    /// The share of points in no curve: the score's floor (Autoware: 0.55).
    pub outlier_ratio: f64,
    /// How many times the line search may halve a step.
    pub halvings: usize,
}

impl Default for MatchSettings {
    fn default() -> Self {
        Self {
            max_iterations: 30,
            step_m: 0.1,
            step_rad: 2f64.to_radians(),
            done_m: 1e-3,
            done_rad: 1e-4,
            outlier_ratio: 0.55,
            halvings: 10,
        }
    }
}

/// One cube size: its map and its score's constants.
#[derive(Clone, Debug)]
struct Level {
    map: CubeMap,
    gauss: Gauss,
}

/// The answer for one scan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Match {
    /// Where the scan sits best: `base_link` in the map.
    pub pose: Pose2,
    /// The score at `pose`, on the smallest cubes, per scan point: how good the fit is.
    pub score_per_point: f64,
    /// The share of scan points with a curve near them, on the smallest cubes.
    pub overlap: f64,
    /// Newton steps, all cube sizes together.
    pub iterations: usize,
    /// The smallest cubes' search stopped because its steps got small, not because it ran
    /// out of iterations.
    pub converged: bool,
    /// The score's Hessian at `pose`, on the smallest cubes: for the covariance.
    pub hessian: Matrix3<f64>,
}

/// Why a scan couldn't be matched.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MatchError {
    /// The scan has no points (after the filters).
    EmptyScan,
    /// At the guess, no scan point is near any of the map's curves.
    NoOverlap,
}

impl fmt::Display for MatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyScan => write!(f, "the scan has no points left after the filters"),
            Self::NoOverlap => write!(f, "no scan point is near the map at the guess"),
        }
    }
}

impl Error for MatchError {}

/// The map at each cube size, and how to search it.
#[derive(Clone, Debug)]
pub struct Matcher {
    levels: Vec<Level>,
    settings: MatchSettings,
}

impl Matcher {
    /// Builds the map's cubes at each size in `cubes` (m, biggest first: the design's
    /// `[2.0, 1.0]`), from points in the map frame.
    pub fn new(
        map_points: &[Point],
        cubes: &[f64],
        map: impl Fn(f64) -> MapSettings,
        settings: MatchSettings,
    ) -> Result<Self, MapError> {
        let levels = cubes
            .iter()
            .map(|&side| {
                Ok(Level {
                    map: CubeMap::build(map_points, &map(side))?,
                    gauss: Gauss::new(side, settings.outlier_ratio),
                })
            })
            .collect::<Result<Vec<_>, MapError>>()?;
        if levels.is_empty() {
            return Err(MapError::BadCube(f64::NAN));
        }
        Ok(Self { levels, settings })
    }

    /// The cube sizes and how many curves each has.
    pub fn levels(&self) -> impl Iterator<Item = (f64, usize)> + '_ {
        self.levels.iter().map(|l| (l.map.side(), l.map.len()))
    }

    /// The score per point of `scan` at `pose`, on the smallest cubes, with no search.
    pub fn score_per_point(&self, scan: &[Point], pose: &Pose2) -> f64 {
        let fine = self.fine();
        evaluate(&fine.map, &fine.gauss, scan, pose, false).score / scan.len().max(1) as f64
    }

    /// The derivatives at `pose` on the smallest cubes (for tests and the covariance).
    pub fn evaluate(&self, scan: &[Point], pose: &Pose2) -> Evaluation {
        let fine = self.fine();
        evaluate(&fine.map, &fine.gauss, scan, pose, true)
    }

    fn fine(&self) -> &Level {
        // new() refuses an empty list of cube sizes.
        &self.levels[self.levels.len() - 1]
    }

    /// Where `scan` (points in `base_link`) sits best on the map, starting from `guess`.
    pub fn align(&self, scan: &[Point], guess: Pose2) -> Result<Match, MatchError> {
        if scan.is_empty() {
            return Err(MatchError::EmptyScan);
        }
        let first = &self.levels[0];
        if evaluate(&first.map, &first.gauss, scan, &guess, false).matched == 0 {
            return Err(MatchError::NoOverlap);
        }
        let mut pose = guess;
        let mut iterations = 0;
        let mut converged = false;
        for level in &self.levels {
            let (found, steps, done) = self.newton(level, scan, pose);
            pose = found;
            iterations += steps;
            converged = done;
        }
        pose.yaw = wrap(pose.yaw);
        let fine = self.fine();
        let at = evaluate(&fine.map, &fine.gauss, scan, &pose, true);
        Ok(Match {
            pose,
            score_per_point: at.score / scan.len() as f64,
            overlap: at.matched as f64 / scan.len() as f64,
            iterations,
            converged,
            hessian: at.hessian,
        })
    }

    /// Newton on one cube size: (the pose, the steps taken, whether it converged).
    fn newton(&self, level: &Level, scan: &[Point], start: Pose2) -> (Pose2, usize, bool) {
        let s = &self.settings;
        let mut pose = start;
        for iteration in 1..=s.max_iterations {
            let here = evaluate(&level.map, &level.gauss, scan, &pose, true);
            let Some(step) = ascent(&here) else {
                return (pose, iteration, true);
            };
            let step = cap(step, s.step_m, s.step_rad);
            // Armijo: the score must rise by at least a little of what the slope promised.
            let slope = here.gradient.dot(&step);
            let mut alpha = 1.0;
            let mut taken = None;
            for _ in 0..=s.halvings {
                let next = moved(&pose, &(alpha * step));
                let score = evaluate(&level.map, &level.gauss, scan, &next, false).score;
                if score >= here.score + 1e-4 * alpha * slope {
                    taken = Some(next);
                    break;
                }
                alpha *= 0.5;
            }
            let Some(next) = taken else {
                // No step along the slope raises the score: as high as it goes from here.
                return (pose, iteration, true);
            };
            pose = next;
            let small = alpha * step;
            if small.xy().norm() < s.done_m && small.z.abs() < s.done_rad {
                return (pose, iteration, true);
            }
        }
        (pose, s.max_iterations, false)
    }
}

/// Newton's step uphill, `-H⁻¹ g`, with H damped until it is negative-definite;
/// `None` at a flat spot (no gradient).
fn ascent(at: &Evaluation) -> Option<Vector3<f64>> {
    if at.gradient.norm() == 0.0 || !at.gradient.iter().all(|g| g.is_finite()) {
        return None;
    }
    let h = 0.5 * (at.hessian + at.hessian.transpose());
    let eigen = SymmetricEigen::new(h);
    let top = eigen.eigenvalues.max();
    let scale = eigen.eigenvalues.amax().max(1e-12);
    // Shift every eigenvalue down until the largest is at most -1e-3 of the biggest: then
    // -H is positive-definite and the step goes uphill (Levenberg's damping).
    let margin = 1e-3 * scale;
    let damped = if top > -margin {
        h - Matrix3::identity() * (top + margin)
    } else {
        h
    };
    let step = damped.try_inverse()? * (-at.gradient);
    step.iter().all(|v| v.is_finite()).then_some(step)
}

/// The step shrunk (direction kept) so it moves at most `metres` and turns at most
/// `radians`.
fn cap(step: Vector3<f64>, metres: f64, radians: f64) -> Vector3<f64> {
    let shrink = (metres / step.xy().norm())
        .min(radians / step.z.abs())
        .min(1.0);
    step * shrink
}

/// The pose moved by a step in (x, y, yaw) (the score's own variables: added, not composed).
fn moved(pose: &Pose2, step: &Vector3<f64>) -> Pose2 {
    Pose2::new(pose.x + step.x, pose.y + step.y, pose.yaw + step.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cap_keeps_the_direction() {
        let capped = cap(Vector3::new(3.0, 4.0, 0.0), 0.1, 0.05);
        assert!((capped.xy().norm() - 0.1).abs() < 1e-12);
        assert!((capped.x / capped.y - 0.75).abs() < 1e-12);
        let turn = cap(Vector3::new(0.0, 0.0, 1.0), 0.1, 0.05);
        assert!((turn.z - 0.05).abs() < 1e-12);
        let small = Vector3::new(0.01, 0.0, 0.001);
        assert_eq!(cap(small, 0.1, 0.05), small);
    }

    #[test]
    fn damping_turns_a_bad_hessian_uphill() {
        // A saddle: Newton's own step would go down the gradient.
        let at = Evaluation {
            score: 0.0,
            gradient: Vector3::new(1.0, 1.0, 0.0),
            hessian: Matrix3::from_diagonal(&Vector3::new(2.0, -1.0, -1.0)),
            matched: 1,
        };
        let step = ascent(&at).expect("a step");
        assert!(step.dot(&at.gradient) > 0.0);
    }
}

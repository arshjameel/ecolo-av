//! How well a scan sits on the map at a pose: the score, its gradient and its Hessian in
//! (x, y, yaw).
//!
//! Magnusson 2009, section 6.2: each point scores `-d1 · exp(-d2/2 · qᵀ Σ⁻¹ q)` against
//! each nearby cube, with `q` the point's offset from the cube's mean. The constants d1, d2
//! fit a mix of a bell curve and a flat floor (outliers), so one far point can't pull the
//! answer much (eq. 6.8). Higher is better.
//!
//! In 2D, the pose moves a scan point `p` to `q = R(yaw) p + (x, y, 0) - mean`, so
//!
//! ```text
//! ∂q/∂x = (1, 0, 0)   ∂q/∂y = (0, 1, 0)   ∂q/∂yaw = (-s px - c py,  c px - s py, 0)
//! ∂²q/∂yaw² = (-c px + s py, -s px - c py, 0), the rest 0
//! ```

use nalgebra::{Matrix3, Vector3};

use crate::{map::CubeMap, Point, Pose2};

/// The score's two constants for one cube size (Magnusson 2009, eq. 6.8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gauss {
    d1: f64,
    d2: f64,
}

impl Gauss {
    /// For cubes of `side` m, when `outlier_ratio` of the points belong to no curve
    /// (Autoware's default is 0.55).
    pub fn new(side: f64, outlier_ratio: f64) -> Self {
        let c1 = 10.0 * (1.0 - outlier_ratio);
        let c2 = outlier_ratio / side.powi(3);
        let d3 = -c2.ln();
        let d1 = -(c1 + c2).ln() - d3;
        let d2 = -2.0 * ((-(c1 * (-0.5f64).exp() + c2).ln() - d3) / d1).ln();
        Self { d1, d2 }
    }

    /// The most one point can score against one cube: right on its mean.
    pub fn best(&self) -> f64 {
        -self.d1
    }
}

/// The score at one pose, and, if asked for, its derivatives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Evaluation {
    pub score: f64,
    /// ∂score/∂(x, y, yaw).
    pub gradient: Vector3<f64>,
    /// ∂²score/∂(x, y, yaw)², symmetric.
    pub hessian: Matrix3<f64>,
    /// How many scan points had at least one curve near them.
    pub matched: usize,
}

/// Scores `scan` (points in `base_link`) placed at `pose` on `map`. With `derivatives`
/// false, only the score and `matched` (the line search's cheap call).
pub fn evaluate(
    map: &CubeMap,
    gauss: &Gauss,
    scan: &[Point],
    pose: &Pose2,
    derivatives: bool,
) -> Evaluation {
    let (s, c) = pose.yaw.sin_cos();
    let mut out = Evaluation {
        score: 0.0,
        gradient: Vector3::zeros(),
        hessian: Matrix3::zeros(),
        matched: 0,
    };
    for p in scan {
        let moved = pose.apply(p);
        // ∂q/∂yaw and ∂²q/∂yaw²: the only ones that depend on the point.
        let turn = Vector3::new(-s * p.x - c * p.y, c * p.x - s * p.y, 0.0);
        let turn2 = Vector3::new(-c * p.x + s * p.y, -s * p.x - c * p.y, 0.0);
        let mut any = false;
        for cube in map.neighbours(&moved) {
            any = true;
            let q = moved.coords - cube.mean;
            let a = cube.inverse * q; // Σ⁻¹ q, symmetric Σ⁻¹
            let e = (-0.5 * gauss.d2 * q.dot(&a)).exp();
            out.score += -gauss.d1 * e;
            if !derivatives {
                continue;
            }
            // qᵀ Σ⁻¹ ∂q/∂pᵢ, for x, y and yaw.
            let along = Vector3::new(a.x, a.y, a.dot(&turn));
            let factor = gauss.d1 * gauss.d2 * e;
            out.gradient += factor * along;
            // ∂q/∂pⱼᵀ Σ⁻¹ ∂q/∂pᵢ: the columns of J = [eₓ, e_y, turn].
            let inv = &cube.inverse;
            let it = inv * turn;
            let jtj = Matrix3::new(
                inv[(0, 0)],
                inv[(0, 1)],
                it.x,
                inv[(1, 0)],
                inv[(1, 1)],
                it.y,
                it.x,
                it.y,
                turn.dot(&it),
            );
            let mut h = jtj - gauss.d2 * along * along.transpose();
            h[(2, 2)] += a.dot(&turn2);
            out.hessian += factor * h;
        }
        out.matched += usize::from(any);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_constants_match_magnussons_numbers() {
        // 1 m cubes, 0.55 outliers: d1 ≈ -2.2, d2 ≈ 0.43 (worked by hand from eq. 6.8).
        let g = Gauss::new(1.0, 0.55);
        assert!((g.d1 + 2.2172).abs() < 1e-3, "{}", g.d1);
        assert!((g.d2 - 0.4328).abs() < 1e-3, "{}", g.d2);
        assert!(g.best() > 0.0);
    }
}

//! A small QP solver: the ADMM of OSQP (Stellato et al. 2020), a port of
//! `aav_track/qp.py`, step for step.
//!
//! ```text
//! minimise   1/2 z' P z + q' z
//! subject to lo <= A z <= hi            (lo = hi for an equality; +-inf for none)
//! ```
//!
//! One factorisation of `P + sigma I + A' R A` (R: each row's rho) per problem, then cheap
//! iterations; warm-started from the last answer's multipliers. It stops when both
//! residuals are under `eps` (absolute, infinity norm, checked every 10 iterations), or
//! after `max_iter` with its best: within the bounds only to that tolerance, so callers
//! clip what must hold exactly.
//!
//! P is dense (the plan's 90 numbers); A is sparse by rows ([`RowMatrix`]): most of the
//! plan's rows hold one or two numbers, so an iteration costs about P's size, not A's.

use std::fmt;

/// A matrix kept by rows, only its non-zeros (CSR).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RowMatrix {
    cols: usize,
    starts: Vec<usize>,
    index: Vec<usize>,
    value: Vec<f64>,
}

impl RowMatrix {
    /// An empty matrix of `cols` columns; add rows with [`RowMatrix::push_row`].
    pub fn new(cols: usize) -> Self {
        RowMatrix {
            cols,
            starts: vec![0],
            index: Vec::new(),
            value: Vec::new(),
        }
    }

    /// Add a row from its (column, value) pairs.
    pub fn push_row(&mut self, entries: impl IntoIterator<Item = (usize, f64)>) {
        for (c, v) in entries {
            if v != 0.0 {
                self.index.push(c);
                self.value.push(v);
            }
        }
        self.starts.push(self.index.len());
    }

    /// How many rows.
    pub fn rows(&self) -> usize {
        self.starts.len() - 1
    }

    /// How many columns.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Row `r`'s (column, value) pairs.
    pub fn row(&self, r: usize) -> impl Iterator<Item = (usize, f64)> + '_ {
        let span = self.starts[r]..self.starts[r + 1];
        self.index[span.clone()]
            .iter()
            .copied()
            .zip(self.value[span].iter().copied())
    }

    /// `A x`, into `out`.
    pub fn mul_into(&self, x: &[f64], out: &mut [f64]) {
        for (r, o) in out.iter_mut().enumerate() {
            *o = self.row(r).map(|(c, v)| v * x[c]).sum();
        }
    }

    /// `A' y`, into `out`.
    pub fn mul_t_into(&self, y: &[f64], out: &mut [f64]) {
        out.fill(0.0);
        for (r, &yr) in y.iter().enumerate() {
            if yr != 0.0 {
                for (c, v) in self.row(r) {
                    out[c] += v * yr;
                }
            }
        }
    }
}

/// The QP's settings: qp.py's defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// The step for the constraints (equality rows: 1000 times it).
    pub rho: f64,
    /// A small regularisation of P.
    pub sigma: f64,
    /// Over-relaxation, 1..2.
    pub alpha: f64,
    /// Both residuals under this: solved.
    pub eps: f64,
    pub max_iter: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            rho: 0.1,
            sigma: 1e-6,
            alpha: 1.6,
            eps: 1e-4,
            max_iter: 400,
        }
    }
}

/// The answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Solution {
    pub z: Vec<f64>,
    /// The constraints' multipliers, for a warm start.
    pub y: Vec<f64>,
    pub iterations: usize,
    /// Both residuals under eps.
    pub solved: bool,
}

/// Why a QP couldn't be solved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QpError {
    /// A constraint with its lower bound above its upper.
    BoundsCrossed,
    /// The sizes of P, q, A, lo and hi don't fit together.
    Shape,
    /// `P + sigma I + A' R A` isn't positive definite (P not positive semidefinite).
    NotConvex,
}

impl fmt::Display for QpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            QpError::BoundsCrossed => "a constraint with lo above hi",
            QpError::Shape => "the sizes of P, q, A, lo and hi don't fit",
            QpError::NotConvex => "P isn't positive semidefinite",
        })
    }
}

impl std::error::Error for QpError {}

/// The Cholesky factor `L` (lower, row-major) of a symmetric positive definite matrix.
fn cholesky(m: &[f64], n: usize) -> Result<Vec<f64>, QpError> {
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let dot: f64 = (0..j).map(|k| l[i * n + k] * l[j * n + k]).sum();
            let v = m[i * n + j] - dot;
            if i == j {
                if v <= 0.0 || !v.is_finite() {
                    return Err(QpError::NotConvex);
                }
                l[i * n + i] = v.sqrt();
            } else {
                l[i * n + j] = v / l[j * n + j];
            }
        }
    }
    Ok(l)
}

/// Solve `L L' x = b` in place.
fn cholesky_solve(l: &[f64], n: usize, b: &mut [f64]) {
    for i in 0..n {
        let dot: f64 = (0..i).map(|k| l[i * n + k] * b[k]).sum();
        b[i] = (b[i] - dot) / l[i * n + i];
    }
    for i in (0..n).rev() {
        let dot: f64 = (i + 1..n).map(|k| l[k * n + i] * b[k]).sum();
        b[i] = (b[i] - dot) / l[i * n + i];
    }
}

/// numpy's `isclose(a, b)` (rtol 1e-5, atol 1e-8; two equal infinities are close).
fn is_close(a: f64, b: f64) -> bool {
    a == b || (a - b).abs() <= 1e-8 + 1e-5 * b.abs()
}

/// The QP's answer. `p`: n x n, row-major, symmetric positive semidefinite; `lo <= hi`.
/// `z0`, `y0`: a warm start (zeros if `None`).
#[allow(clippy::too_many_arguments)]
pub fn solve(
    p: &[f64],
    q: &[f64],
    a: &RowMatrix,
    lo: &[f64],
    hi: &[f64],
    z0: Option<&[f64]>,
    y0: Option<&[f64]>,
    settings: &Settings,
) -> Result<Solution, QpError> {
    let n = q.len();
    let m = a.rows();
    if p.len() != n * n
        || a.cols() != n
        || lo.len() != m
        || hi.len() != m
        || z0.is_some_and(|z| z.len() != n)
        || y0.is_some_and(|y| y.len() != m)
    {
        return Err(QpError::Shape);
    }
    if lo.iter().zip(hi).any(|(l, h)| l > h) {
        return Err(QpError::BoundsCrossed);
    }
    let Settings {
        rho,
        sigma,
        alpha,
        eps,
        max_iter,
    } = *settings;
    // Equality rows get a stiffer rho, as OSQP does; free rows almost none.
    let rhos: Vec<f64> = lo
        .iter()
        .zip(hi)
        .map(|(&l, &h)| {
            if l.is_infinite() && h.is_infinite() {
                1e-6
            } else if is_close(l, h) {
                1e3 * rho
            } else {
                rho
            }
        })
        .collect();
    // K = P + sigma I + A' R A.
    let mut k = p.to_vec();
    for i in 0..n {
        k[i * n + i] += sigma;
    }
    for (r, &rr) in rhos.iter().enumerate() {
        let entries: Vec<(usize, f64)> = a.row(r).collect();
        for &(i, vi) in &entries {
            for &(j, vj) in &entries {
                k[i * n + j] += rr * vi * vj;
            }
        }
    }
    let l = cholesky(&k, n)?;

    let mut x = z0.map_or_else(|| vec![0.0; n], <[f64]>::to_vec);
    let mut y = y0.map_or_else(|| vec![0.0; m], <[f64]>::to_vec);
    let mut z = vec![0.0; m];
    a.mul_into(&x, &mut z);
    for ((zi, &l), &h) in z.iter_mut().zip(lo).zip(hi) {
        *zi = zi.max(l).min(h);
    }
    let mut w = vec![0.0; m];
    let mut rhs = vec![0.0; n];
    let mut zt = vec![0.0; m];
    let mut ax = vec![0.0; m];
    let mut px = vec![0.0; n];
    let mut aty = vec![0.0; n];
    for it in 1..=max_iter {
        // xt = K^-1 (sigma x - q + A'(R z - y))
        for (wi, ((&ri, &zi), &yi)) in w.iter_mut().zip(rhos.iter().zip(&z).zip(&y)) {
            *wi = ri * zi - yi;
        }
        a.mul_t_into(&w, &mut rhs);
        for ((r, &xi), &qi) in rhs.iter_mut().zip(&x).zip(q) {
            *r += sigma * xi - qi;
        }
        cholesky_solve(&l, n, &mut rhs);
        let xt = &rhs;
        a.mul_into(xt, &mut zt);
        for (xi, &ti) in x.iter_mut().zip(xt) {
            *xi = alpha * ti + (1.0 - alpha) * *xi;
        }
        for i in 0..m {
            let mixed = alpha * zt[i] + (1.0 - alpha) * z[i];
            let z_new = (mixed + y[i] / rhos[i]).max(lo[i]).min(hi[i]);
            y[i] += rhos[i] * (mixed - z_new);
            z[i] = z_new;
        }
        if it % 10 == 0 || it == max_iter {
            a.mul_into(&x, &mut ax);
            let primal = ax
                .iter()
                .zip(&z)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max);
            for (i, pi) in px.iter_mut().enumerate() {
                *pi = p[i * n..(i + 1) * n]
                    .iter()
                    .zip(&x)
                    .map(|(a, b)| a * b)
                    .sum();
            }
            a.mul_t_into(&y, &mut aty);
            let dual = px
                .iter()
                .zip(q)
                .zip(&aty)
                .map(|((a, b), c)| (a + b + c).abs())
                .fold(0.0, f64::max);
            if primal < eps && dual < eps {
                return Ok(Solution {
                    z: x,
                    y,
                    iterations: it,
                    solved: true,
                });
            }
        }
    }
    Ok(Solution {
        z: x,
        y,
        iterations: max_iter,
        solved: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity_rows(n: usize) -> RowMatrix {
        let mut a = RowMatrix::new(n);
        for i in 0..n {
            a.push_row([(i, 1.0)]);
        }
        a
    }

    fn tight() -> Settings {
        Settings {
            eps: 1e-7,
            max_iter: 4000,
            rho: 0.1,
            ..Settings::default()
        }
    }

    #[test]
    fn unconstrained_is_minus_p_inverse_q() {
        // min (x - 1)^2 + 2 (y + 2)^2: P = diag(2, 4), q = (-2, 8). Free rows.
        let a = identity_rows(2);
        let inf = f64::INFINITY;
        let s = solve(
            &[2.0, 0.0, 0.0, 4.0],
            &[-2.0, 8.0],
            &a,
            &[-inf, -inf],
            &[inf, inf],
            None,
            None,
            &tight(),
        )
        .unwrap();
        assert!(s.solved);
        assert!((s.z[0] - 1.0).abs() < 1e-4 && (s.z[1] + 2.0).abs() < 1e-4);
    }

    #[test]
    fn a_box_holds_the_answer_at_its_edge() {
        // min (x - 3)^2, 0 <= x <= 1: x = 1, the multiplier 4 (P x + q + A'y = 0).
        let a = identity_rows(1);
        let s = solve(&[2.0], &[-6.0], &a, &[0.0], &[1.0], None, None, &tight()).unwrap();
        assert!(s.solved);
        assert!((s.z[0] - 1.0).abs() < 1e-5, "{:?}", s.z);
        assert!((s.y[0] - 4.0).abs() < 1e-4, "{:?}", s.y);
    }

    #[test]
    fn an_equality_and_a_coupling_row() {
        // min x^2 + y^2, x + y = 1: x = y = 0.5. Then with y <= 0.2: x = 0.8, y = 0.2.
        let mut a = RowMatrix::new(2);
        a.push_row([(0, 1.0), (1, 1.0)]);
        let p = [2.0, 0.0, 0.0, 2.0];
        let s = solve(&p, &[0.0, 0.0], &a, &[1.0], &[1.0], None, None, &tight()).unwrap();
        assert!((s.z[0] - 0.5).abs() < 1e-5 && (s.z[1] - 0.5).abs() < 1e-5);
        a.push_row([(1, 1.0)]);
        let lo = [1.0, f64::NEG_INFINITY];
        let s = solve(&p, &[0.0, 0.0], &a, &lo, &[1.0, 0.2], None, None, &tight()).unwrap();
        assert!(s.solved);
        assert!((s.z[0] - 0.8).abs() < 1e-5 && (s.z[1] - 0.2).abs() < 1e-5);
    }

    #[test]
    fn a_warm_start_finishes_sooner() {
        let mut a = identity_rows(3);
        a.push_row([(0, 1.0), (1, 1.0), (2, 1.0)]);
        let p = [4.0, 1.0, 0.0, 1.0, 3.0, 0.5, 0.0, 0.5, 2.0];
        let q = [-1.0, -2.0, 3.0];
        let (lo, hi) = ([-0.5, -0.5, -0.5, 0.2], [0.5, 0.5, 0.5, 0.3]);
        // qp.py gives the same: 400 iterations unsolved from cold, 50 warm.
        let s = Settings::default();
        let cold = solve(&p, &q, &a, &lo, &hi, None, None, &s).unwrap();
        let warm = solve(&p, &q, &a, &lo, &hi, Some(&cold.z), Some(&cold.y), &s).unwrap();
        assert!(!cold.solved && cold.iterations == 400);
        assert!(warm.solved && warm.iterations == 50, "{}", warm.iterations);
        for (z, want) in warm.z.iter().zip([0.2, 0.5, -0.5]) {
            assert!((z - want).abs() < 1e-3, "{:?}", warm.z);
        }
    }

    #[test]
    fn bad_problems_say_why() {
        let a = identity_rows(1);
        let s = Settings::default();
        assert_eq!(
            solve(&[1.0], &[0.0], &a, &[1.0], &[0.0], None, None, &s),
            Err(QpError::BoundsCrossed)
        );
        assert_eq!(
            solve(&[1.0, 0.0], &[0.0], &a, &[0.0], &[1.0], None, None, &s),
            Err(QpError::Shape)
        );
        assert_eq!(
            solve(&[-5.0], &[0.0], &a, &[0.0], &[1.0], None, None, &s),
            Err(QpError::NotConvex)
        );
    }
}

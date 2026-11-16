//! The shortest drivable curve between two poses: Dubins (forward only) and Reeds-Shepp
//! (forward and back), each made of arcs at one radius and straights.
//!
//! Nav2 takes these from OMPL (`DubinsStateSpace`, `ReedsSheppStateSpace`, BSD-3-Clause);
//! these are OMPL's formulas (Shkel and Lumelsky for Dubins; Reeds and Shepp, as corrected
//! by Desaulniers and Soumis) rewritten in Rust. The tests check each curve really ends on
//! the goal.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use crate::Pose;

/// One piece of a curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Piece {
    /// An arc to the left.
    Left,
    /// A straight.
    Straight,
    /// An arc to the right.
    Right,
}

/// Which curves a car may take.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveKind {
    /// Forward only.
    Dubins,
    /// Forward and back.
    ReedsShepp,
}

/// A curve: pieces with signed lengths in radii (negative: backwards).
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    /// Each piece, and its signed length in radii.
    pub pieces: Vec<(Piece, f64)>,
    /// The turning radius, in the poses' units.
    pub radius: f64,
}

impl Curve {
    /// The length driven, in the poses' units.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.radius * self.pieces.iter().map(|(_, l)| l.abs()).sum::<f64>()
    }

    /// The pose a fraction `t` (0 to 1) of the way along, starting at `from`.
    #[must_use]
    pub fn sample(&self, from: &Pose, t: f64) -> Pose {
        let total: f64 = self.pieces.iter().map(|(_, l)| l.abs()).sum();
        let mut left = t.clamp(0.0, 1.0) * total;
        let (mut x, mut y, mut yaw) = (0.0, 0.0, from.yaw);
        for &(piece, length) in &self.pieces {
            if left <= 0.0 {
                break;
            }
            let step = length.abs().min(left);
            left -= step;
            let v = step.copysign(length);
            match piece {
                Piece::Left => {
                    x += (yaw + v).sin() - yaw.sin();
                    y += -(yaw + v).cos() + yaw.cos();
                    yaw += v;
                }
                Piece::Right => {
                    x += -(yaw - v).sin() + yaw.sin();
                    y += (yaw - v).cos() - yaw.cos();
                    yaw -= v;
                }
                Piece::Straight => {
                    x += v * yaw.cos();
                    y += v * yaw.sin();
                }
            }
        }
        Pose::new(
            from.x + x * self.radius,
            from.y + y * self.radius,
            wrap(yaw),
        )
    }
}

/// An angle in (-pi, pi].
#[must_use]
pub fn wrap(a: f64) -> f64 {
    let v = a.rem_euclid(TAU);
    if v > PI {
        v - TAU
    } else {
        v
    }
}

/// The shortest curve from `from` to `to` at `radius` (> 0).
#[must_use]
pub fn shortest(kind: CurveKind, from: &Pose, to: &Pose, radius: f64) -> Curve {
    match kind {
        CurveKind::Dubins => dubins(from, to, radius),
        CurveKind::ReedsShepp => reeds_shepp(from, to, radius),
    }
}

/// Every candidate curve (for the tests: each must end on the goal).
#[must_use]
pub fn candidates(kind: CurveKind, from: &Pose, to: &Pose, radius: f64) -> Vec<Curve> {
    let words = match kind {
        CurveKind::Dubins => {
            let (d, alpha, beta) = dubins_frame(from, to, radius);
            dubins_words(d, alpha, beta)
        }
        CurveKind::ReedsShepp => {
            let (x, y, phi) = rs_frame(from, to, radius);
            rs_words(x, y, phi)
        }
    };
    words
        .into_iter()
        .map(|pieces| Curve { pieces, radius })
        .collect()
}

fn pick(words: Vec<Vec<(Piece, f64)>>, radius: f64) -> Curve {
    let length = |w: &Vec<(Piece, f64)>| w.iter().map(|(_, l)| l.abs()).sum::<f64>();
    let pieces = words
        .into_iter()
        .min_by(|a, b| length(a).total_cmp(&length(b)))
        .unwrap_or_default();
    Curve { pieces, radius }
}

// ---------------------------------------------------------------- Dubins

const DUBINS_EPS: f64 = 1e-6;
const DUBINS_ZERO: f64 = -1e-7;

fn mod_tau(a: f64) -> f64 {
    a.rem_euclid(TAU)
}

fn dubins_frame(from: &Pose, to: &Pose, radius: f64) -> (f64, f64, f64) {
    let (dx, dy) = (to.x - from.x, to.y - from.y);
    let d = dx.hypot(dy) / radius;
    let th = if d > 0.0 { dy.atan2(dx) } else { 0.0 };
    (d, mod_tau(from.yaw - th), mod_tau(to.yaw - th))
}

fn dubins(from: &Pose, to: &Pose, radius: f64) -> Curve {
    let (d, alpha, beta) = dubins_frame(from, to, radius);
    pick(dubins_words(d, alpha, beta), radius)
}

fn dubins_words(d: f64, alpha: f64, beta: f64) -> Vec<Vec<(Piece, f64)>> {
    use Piece::{Left as L, Right as R, Straight as S};
    if d < DUBINS_EPS && (alpha - beta).abs() < DUBINS_EPS {
        return vec![vec![(L, 0.0), (S, d), (L, 0.0)]];
    }
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let mut out = Vec::new();
    // LSL
    let tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
    if tmp >= DUBINS_ZERO {
        let theta = (cb - ca).atan2(d + sa - sb);
        out.push(vec![
            (L, mod_tau(-alpha + theta)),
            (S, tmp.max(0.0).sqrt()),
            (L, mod_tau(beta - theta)),
        ]);
    }
    // RSR
    let tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
    if tmp >= DUBINS_ZERO {
        let theta = (ca - cb).atan2(d - sa + sb);
        out.push(vec![
            (R, mod_tau(alpha - theta)),
            (S, tmp.max(0.0).sqrt()),
            (R, mod_tau(-beta + theta)),
        ]);
    }
    // RSL
    let tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
    if tmp >= DUBINS_ZERO {
        let p = tmp.max(0.0).sqrt();
        let theta = (ca + cb).atan2(d - sa - sb) - 2.0_f64.atan2(p);
        out.push(vec![
            (R, mod_tau(alpha - theta)),
            (S, p),
            (L, mod_tau(beta - theta)),
        ]);
    }
    // LSR
    let tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
    if tmp >= DUBINS_ZERO {
        let p = tmp.max(0.0).sqrt();
        let theta = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
        out.push(vec![
            (L, mod_tau(-alpha + theta)),
            (S, p),
            (R, mod_tau(-beta + theta)),
        ]);
    }
    // RLR
    let tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb + d * (sa - sb)));
    if tmp.abs() < 1.0 {
        let p = TAU - tmp.acos();
        let theta = (ca - cb).atan2(d - sa + sb);
        let t = mod_tau(alpha - theta + 0.5 * p);
        out.push(vec![(R, t), (L, p), (R, mod_tau(alpha - beta - t + p))]);
    }
    // LRL
    let tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb - d * (sa - sb)));
    if tmp.abs() < 1.0 {
        let p = TAU - tmp.acos();
        let theta = (-ca + cb).atan2(d + sa - sb);
        let t = mod_tau(-alpha + theta + 0.5 * p);
        out.push(vec![(L, t), (R, p), (L, mod_tau(beta - alpha - t + p))]);
    }
    out
}

// ---------------------------------------------------------------- Reeds-Shepp

const RS_ZERO: f64 = 10.0 * f64::EPSILON;

/// A base word's formula: (x, y, phi) to its three lengths, if it applies.
type Word = fn(f64, f64, f64) -> Option<(f64, f64, f64)>;

fn polar(x: f64, y: f64) -> (f64, f64) {
    (x.hypot(y), y.atan2(x))
}

fn tau_omega(u: f64, v: f64, xi: f64, eta: f64, phi: f64) -> (f64, f64) {
    let delta = wrap(u - v);
    let a = u.sin() - delta.sin();
    let b = u.cos() - delta.cos() - 1.0;
    let t1 = (eta * a - xi * b).atan2(xi * a + eta * b);
    let t2 = 2.0 * (delta.cos() - v.cos() - u.cos()) + 3.0;
    let tau = if t2 < 0.0 { wrap(t1 + PI) } else { wrap(t1) };
    (tau, wrap(tau - u + v - phi))
}

// The base words (formula numbers from Reeds and Shepp's paper, as OMPL names them).
fn lp_sp_lp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (u, t) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if t >= -RS_ZERO {
        let v = wrap(phi - t);
        if v >= -RS_ZERO {
            return Some((t, u, v));
        }
    }
    None
}

fn lp_sp_rp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (u1, t1) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    let u1 = u1 * u1;
    if u1 >= 4.0 {
        let u = (u1 - 4.0).sqrt();
        let theta = 2.0_f64.atan2(u);
        let t = wrap(t1 + theta);
        let v = wrap(t - phi);
        if t >= -RS_ZERO && v >= -RS_ZERO {
            return Some((t, u, v));
        }
    }
    None
}

fn lp_rm_l(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 <= 4.0 {
        let u = -2.0 * (0.25 * u1).asin();
        let t = wrap(theta + 0.5 * u + PI);
        let v = wrap(phi - t + u);
        if t >= -RS_ZERO && u <= RS_ZERO {
            return Some((t, u, v));
        }
    }
    None
}

fn lp_rup_lum_rm(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
    let rho = 0.25 * (2.0 + xi.hypot(eta));
    if rho <= 1.0 {
        let u = rho.acos();
        let (t, v) = tau_omega(u, -u, xi, eta, phi);
        if t >= -RS_ZERO && v <= RS_ZERO {
            return Some((t, u, v));
        }
    }
    None
}

fn lp_rum_lum_rp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
    let rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (0.0..=1.0).contains(&rho) {
        let u = -rho.acos();
        if u >= -FRAC_PI_2 {
            let (t, v) = tau_omega(u, u, xi, eta, phi);
            if t >= -RS_ZERO && v >= -RS_ZERO {
                return Some((t, u, v));
            }
        }
    }
    None
}

fn lp_rm_sm_lm(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (rho, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if rho >= 2.0 {
        let r = (rho * rho - 4.0).sqrt();
        let u = 2.0 - r;
        let t = wrap(theta + r.atan2(-2.0));
        let v = wrap(phi - FRAC_PI_2 - t);
        if t >= -RS_ZERO && u <= RS_ZERO && v <= RS_ZERO {
            return Some((t, u, v));
        }
    }
    None
}

fn lp_rm_sm_rm(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
    let (rho, theta) = polar(-eta, xi);
    if rho >= 2.0 {
        let t = theta;
        let u = 2.0 - rho;
        let v = wrap(t + FRAC_PI_2 - phi);
        if t >= -RS_ZERO && u <= RS_ZERO && v <= RS_ZERO {
            return Some((t, u, v));
        }
    }
    None
}

fn lp_rm_s_lm_rp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
    let (rho, _) = polar(xi, eta);
    if rho >= 2.0 {
        let u = 4.0 - (rho * rho - 4.0).sqrt();
        if u <= RS_ZERO {
            let t = wrap(((4.0 - u) * xi - 2.0 * eta).atan2(-2.0 * xi + (u - 4.0) * eta));
            let v = wrap(t - phi);
            if t >= -RS_ZERO && v >= -RS_ZERO {
                return Some((t, u, v));
            }
        }
    }
    None
}

fn rs_frame(from: &Pose, to: &Pose, radius: f64) -> (f64, f64, f64) {
    let (dx, dy) = (to.x - from.x, to.y - from.y);
    let (s, c) = from.yaw.sin_cos();
    (
        (c * dx + s * dy) / radius,
        (-s * dx + c * dy) / radius,
        to.yaw - from.yaw,
    )
}

fn reeds_shepp(from: &Pose, to: &Pose, radius: f64) -> Curve {
    let (x, y, phi) = rs_frame(from, to, radius);
    pick(rs_words(x, y, phi), radius)
}

/// The four symmetric uses of a base word: as is, time-flipped, reflected, both.
/// `make(t, u, v, sign)` builds the pieces with every length times `sign`; `reflect`
/// swaps left and right.
fn four(
    out: &mut Vec<Vec<(Piece, f64)>>,
    (x, y, phi): (f64, f64, f64),
    word: Word,
    make: impl Fn(f64, f64, f64) -> Vec<(Piece, f64)>,
) {
    let flip = |w: Vec<(Piece, f64)>| -> Vec<(Piece, f64)> {
        w.into_iter().map(|(p, l)| (p, -l)).collect()
    };
    let reflect = |w: Vec<(Piece, f64)>| -> Vec<(Piece, f64)> {
        w.into_iter()
            .map(|(p, l)| {
                let p = match p {
                    Piece::Left => Piece::Right,
                    Piece::Right => Piece::Left,
                    Piece::Straight => Piece::Straight,
                };
                (p, l)
            })
            .collect()
    };
    if let Some((t, u, v)) = word(x, y, phi) {
        out.push(make(t, u, v));
    }
    if let Some((t, u, v)) = word(-x, y, -phi) {
        out.push(flip(make(t, u, v)));
    }
    if let Some((t, u, v)) = word(x, -y, -phi) {
        out.push(reflect(make(t, u, v)));
    }
    if let Some((t, u, v)) = word(-x, -y, phi) {
        out.push(flip(reflect(make(t, u, v))));
    }
}

fn rs_words(x: f64, y: f64, phi: f64) -> Vec<Vec<(Piece, f64)>> {
    use Piece::{Left as L, Right as R, Straight as S};
    let mut out = Vec::new();
    let here = (x, y, phi);
    // The same words driven backwards along the curve (OMPL's "backwards" set).
    let (s, c) = phi.sin_cos();
    let back = (x * c + y * s, x * s - y * c, phi);

    // CSC
    four(&mut out, here, lp_sp_lp, |t, u, v| {
        vec![(L, t), (S, u), (L, v)]
    });
    four(&mut out, here, lp_sp_rp, |t, u, v| {
        vec![(L, t), (S, u), (R, v)]
    });
    // CCC
    four(&mut out, here, lp_rm_l, |t, u, v| {
        vec![(L, t), (R, u), (L, v)]
    });
    four(&mut out, back, lp_rm_l, |t, u, v| {
        vec![(L, v), (R, u), (L, t)]
    });
    // CCCC
    four(&mut out, here, lp_rup_lum_rm, |t, u, v| {
        vec![(L, t), (R, u), (L, -u), (R, v)]
    });
    four(&mut out, here, lp_rum_lum_rp, |t, u, v| {
        vec![(L, t), (R, u), (L, u), (R, v)]
    });
    // CCSC
    four(&mut out, here, lp_rm_sm_lm, |t, u, v| {
        vec![(L, t), (R, -FRAC_PI_2), (S, u), (L, v)]
    });
    four(&mut out, here, lp_rm_sm_rm, |t, u, v| {
        vec![(L, t), (R, -FRAC_PI_2), (S, u), (R, v)]
    });
    four(&mut out, back, lp_rm_sm_lm, |t, u, v| {
        vec![(L, v), (S, u), (R, -FRAC_PI_2), (L, t)]
    });
    four(&mut out, back, lp_rm_sm_rm, |t, u, v| {
        vec![(R, v), (S, u), (R, -FRAC_PI_2), (L, t)]
    });
    // CCSCC
    four(&mut out, here, lp_rm_s_lm_rp, |t, u, v| {
        vec![(L, t), (R, -FRAC_PI_2), (S, u), (L, -FRAC_PI_2), (R, v)]
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &Pose, b: &Pose) -> bool {
        (a.x - b.x).abs() < 1e-6 && (a.y - b.y).abs() < 1e-6 && wrap(a.yaw - b.yaw).abs() < 1e-6
    }

    fn poses() -> Vec<(Pose, Pose)> {
        // A fixed spread of starts and goals (no random numbers: the same every run).
        let mut out = Vec::new();
        for i in 0..400 {
            let f = i as f64;
            let from = Pose::new((f * 0.37).sin() * 3.0, (f * 0.11).cos() * 2.0, f * 0.7);
            let to = Pose::new(
                (f * 1.3).cos() * 8.0,
                (f * 0.53).sin() * 8.0,
                (f * 2.9).rem_euclid(TAU) - PI,
            );
            out.push((from, to));
        }
        out
    }

    #[test]
    fn every_candidate_ends_on_the_goal() {
        for kind in [CurveKind::Dubins, CurveKind::ReedsShepp] {
            for (from, to) in poses() {
                let all = candidates(kind, &from, &to, 1.7);
                assert!(!all.is_empty(), "{kind:?}: no curve {from:?} -> {to:?}");
                for c in all {
                    let end = c.sample(&from, 1.0);
                    assert!(
                        close(&end, &to),
                        "{kind:?} {:?}: {end:?} != {to:?}",
                        c.pieces
                    );
                }
            }
        }
    }

    #[test]
    fn reeds_shepp_is_never_longer_than_dubins_nor_shorter_than_the_line() {
        for (from, to) in poses() {
            let rs = shortest(CurveKind::ReedsShepp, &from, &to, 1.7).length();
            let du = shortest(CurveKind::Dubins, &from, &to, 1.7).length();
            let line = (to.x - from.x).hypot(to.y - from.y);
            assert!(rs <= du + 1e-9 && rs >= line - 1e-9, "{rs} {du} {line}");
        }
    }

    #[test]
    fn straight_ahead_is_the_line() {
        let from = Pose::new(0.0, 0.0, 0.0);
        let to = Pose::new(5.0, 0.0, 0.0);
        for kind in [CurveKind::Dubins, CurveKind::ReedsShepp] {
            assert!((shortest(kind, &from, &to, 2.0).length() - 5.0).abs() < 1e-9);
        }
        // Straight behind: Reeds-Shepp backs up, Dubins must loop.
        let back = Pose::new(-5.0, 0.0, 0.0);
        assert!((shortest(CurveKind::ReedsShepp, &from, &back, 2.0).length() - 5.0).abs() < 1e-9);
        assert!(shortest(CurveKind::Dubins, &from, &back, 2.0).length() > 10.0);
    }
}

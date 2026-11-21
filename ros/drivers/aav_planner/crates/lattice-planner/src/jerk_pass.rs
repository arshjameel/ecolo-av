//! Passing with a jerk limit (TODO 214, a challenger to [`crate::pass`]'s half-cosine
//! shift). Apollo's piecewise-jerk path (Zhang et al., IV 2020; Apache-2.0, the idea
//! only) on Autoware's static obstacle avoidance: the sideways offset `l(s)` from the lane
//! line is a QP along the lane, its third derivative held so the car's **lateral jerk**
//! stays under a limit at the speed it drives.
//!
//! ```text
//!   l ▲            lower bound: the least shift that clears, + clearance
//!     │        ┌──────────────────────┐
//!     │      ╭─┴──────────────────────┴─╮      the QP's l(s): as close to 0 as it may,
//!     │    ╭─╯                          ╰─╮    l''' ≤ jerk / v³, l'' ≤ accel / v², 1 / R
//!   0 ┼────╯                              ╰────────▶ s (along the lane line)
//!          └ ramp: v x (32 d / jerk)^⅓ ┘
//! ```
//!
//! | Step | What |
//! |---|---|
//! | blocked, stretches | as [`crate::pass`]: the search's own check, `margin` each side, close runs merged |
//! | shift | each stretch's least left step that fits (every `step` m), `clearance` more if that fits, no more than the room to the kerb |
//! | ramp | the pass's length from the speed and the jerk limit: a bang-bang jerk shifts `d` in `T = (32 d / jerk)^⅓` (or `2 sqrt(d / accel)` if the acceleration binds), so `v T` m; never sharper than the turning radius |
//! | QP | stations every `ds` m round the stretches; `l, l', l''` each; the piecewise-jerk joins as equalities; the bounds as inequalities; cost `w_l l² + w_dl l'² + w_ddl l''² + w_dddl l'''²` (Clarabel, Apache-2.0) |
//! | path | `l` between stations by the cubic each piece is; each pose moved left by it, its yaw by `atan l'` |
//! | can't | no shift fits: [`NoRoom`]. The QP infeasible (the car too close to ramp within the limits) or its path not clear: the half-cosine pass ([`crate::pass::plan`]) instead |

use clarabel::algebra::CscMatrix;
use clarabel::solver::{DefaultSettings, DefaultSolver, IPSolver, SolverStatus, SupportedConeT};

use crate::pass::{self, Bump, NoRoom, Pass, PassSettings};
use crate::Pose;

/// The jerk-limited pass's limits and weights.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JerkSettings {
    /// m/s³: the most lateral jerk (Autoware's avoidance: 1.0 nominal; ours gentler).
    pub lateral_jerk: f64,
    /// m/s²: the most lateral acceleration.
    pub lateral_accel: f64,
    /// m/s: the speed the limits are held at is never taken under this (a slow car would
    /// get a needlessly short ramp).
    pub min_speed: f64,
    /// m between the QP's stations.
    pub ds: f64,
    /// m of plain lane line kept in the QP before the first ramp and after the last.
    pub lead: f64,
    /// Weights on l², l'², l''², l'''² (Apollo's order of magnitude).
    pub weights: [f64; 4],
}

impl Default for JerkSettings {
    fn default() -> Self {
        Self {
            lateral_jerk: 0.5,
            lateral_accel: 1.0,
            min_speed: 1.0,
            ds: 0.5,
            lead: 2.0,
            weights: [1.0, 100.0, 1000.0, 10_000.0],
        }
    }
}

/// m along the lane for a pass of `shift` m at `speed` m/s within the jerk and
/// acceleration limits: a bang-bang jerk profile (4 phases of `T / 4`) moves `jerk T³ /
/// 32` sideways, peaking at `jerk T / 4`; past `accel` the acceleration holds, `T = 2
/// sqrt(d / accel)` at least. Never shorter than the turning radius allows.
#[must_use]
pub fn ramp_length(shift: f64, speed: f64, radius: f64, settings: &JerkSettings) -> f64 {
    let d = shift.abs();
    let v = speed.abs().max(settings.min_speed);
    let by_jerk = (32.0 * d / settings.lateral_jerk.max(1e-6)).cbrt();
    let by_accel = 2.0 * (d / settings.lateral_accel.max(1e-6)).sqrt();
    (v * by_jerk.max(by_accel)).max(pass::ramp_length(d, radius, 1.0))
}

/// The QP's answer at its stations: `l, l', l''` each.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    /// m along the lane line of the first station.
    pub s0: f64,
    /// m between stations.
    pub ds: f64,
    /// (l, l', l'') at each station.
    pub states: Vec<[f64; 3]>,
}

impl Profile {
    /// The offset and its slope at `s` m along: the piece's cubic (constant jerk); 0
    /// outside the stations.
    #[must_use]
    pub fn at(&self, s: f64) -> (f64, f64) {
        let n = self.states.len();
        if n < 2 || s < self.s0 || s > self.s0 + self.ds * (n - 1) as f64 {
            return (0.0, 0.0);
        }
        let k = (((s - self.s0) / self.ds).floor() as usize).min(n - 2);
        let t = s - self.s0 - k as f64 * self.ds;
        let [l, dl, ddl] = self.states[k];
        let jerk = (self.states[k + 1][2] - ddl) / self.ds;
        (
            l + dl * t + ddl * t * t / 2.0 + jerk * t * t * t / 6.0,
            dl + ddl * t + jerk * t * t / 2.0,
        )
    }
}

/// The QP couldn't be solved within the limits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Infeasible;

impl std::fmt::Display for Infeasible {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("no jerk-limited path within the bounds")
    }
}

impl std::error::Error for Infeasible {}

/// The piecewise-jerk QP from `s0`, `bounds.len()` stations `settings.ds` apart, each
/// `l` within its (low, high); `l'' ≤ curvature`, `|l'''| ≤ jerk` (both along s). The
/// first station is pinned to (0, 0, 0) when `pin_start`, the last always.
///
/// # Errors
///
/// [`Infeasible`] when Clarabel finds no answer.
pub fn solve(
    s0: f64,
    bounds: &[(f64, f64)],
    curvature: f64,
    jerk: f64,
    pin_start: bool,
    settings: &JerkSettings,
) -> Result<Profile, Infeasible> {
    let n = bounds.len();
    if n < 2 {
        return Err(Infeasible);
    }
    let ds = settings.ds;
    let [wl, wdl, wddl, wdddl] = settings.weights;
    let (l, dl, ddl) = (|i: usize| 3 * i, |i: usize| 3 * i + 1, |i: usize| 3 * i + 2);
    let vars = 3 * n;

    // P (upper triangle): the cost x 2 (Clarabel minimises ½ x'Px + q'x).
    let mut p = Triplets::default();
    let jerk_w = wdddl / (ds * ds);
    for i in 0..n {
        p.push(l(i), l(i), 2.0 * wl);
        p.push(dl(i), dl(i), 2.0 * wdl);
        p.push(ddl(i), ddl(i), 2.0 * wddl);
    }
    for i in 0..n - 1 {
        // wdddl ((l''ᵢ₊₁ - l''ᵢ) / ds)²
        p.push(ddl(i), ddl(i), 2.0 * jerk_w);
        p.push(ddl(i + 1), ddl(i + 1), 2.0 * jerk_w);
        p.push(ddl(i), ddl(i + 1), -2.0 * jerk_w);
    }

    // A x + s = b: equalities (zero cone) first, then inequalities (s ≥ 0: A x ≤ b).
    let mut a = Triplets::default();
    let mut b = Vec::new();
    let mut row = 0;
    let mut eq = |a: &mut Triplets, b: &mut Vec<f64>, terms: &[(usize, f64)], rhs: f64| {
        for &(c, v) in terms {
            a.push(row, c, v);
        }
        b.push(rhs);
        row += 1;
    };
    for i in 0..n - 1 {
        // l'ᵢ₊₁ = l'ᵢ + (l''ᵢ + l''ᵢ₊₁) ds / 2
        eq(
            &mut a,
            &mut b,
            &[
                (dl(i + 1), 1.0),
                (dl(i), -1.0),
                (ddl(i), -ds / 2.0),
                (ddl(i + 1), -ds / 2.0),
            ],
            0.0,
        );
        // lᵢ₊₁ = lᵢ + l'ᵢ ds + l''ᵢ ds² / 3 + l''ᵢ₊₁ ds² / 6
        eq(
            &mut a,
            &mut b,
            &[
                (l(i + 1), 1.0),
                (l(i), -1.0),
                (dl(i), -ds),
                (ddl(i), -ds * ds / 3.0),
                (ddl(i + 1), -ds * ds / 6.0),
            ],
            0.0,
        );
    }
    let mut pinned = vec![n - 1];
    if pin_start {
        pinned.push(0);
    }
    for i in pinned {
        for c in [l(i), dl(i), ddl(i)] {
            eq(&mut a, &mut b, &[(c, 1.0)], 0.0);
        }
    }
    let equalities = row;
    let mut le = |a: &mut Triplets, b: &mut Vec<f64>, c: &[(usize, f64)], rhs: f64| {
        for &(col, v) in c {
            a.push(row, col, v);
        }
        b.push(rhs);
        row += 1;
    };
    for (i, &(lo, hi)) in bounds.iter().enumerate() {
        le(&mut a, &mut b, &[(l(i), 1.0)], hi);
        le(&mut a, &mut b, &[(l(i), -1.0)], -lo);
        le(&mut a, &mut b, &[(ddl(i), 1.0)], curvature);
        le(&mut a, &mut b, &[(ddl(i), -1.0)], curvature);
    }
    for i in 0..n - 1 {
        let terms = [(ddl(i + 1), 1.0), (ddl(i), -1.0)];
        le(&mut a, &mut b, &terms, jerk * ds);
        le(&mut a, &mut b, &terms.map(|(c, v)| (c, -v)), jerk * ds);
    }
    let rows = row;

    let p = p.matrix(vars, vars);
    let a = a.matrix(rows, vars);
    let q = vec![0.0; vars];
    let cones = [
        SupportedConeT::ZeroConeT(equalities),
        SupportedConeT::NonnegativeConeT(rows - equalities),
    ];
    let quiet = DefaultSettings {
        verbose: false,
        ..DefaultSettings::default()
    };
    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, quiet).map_err(|_| Infeasible)?;
    solver.solve();
    match solver.solution.status {
        SolverStatus::Solved | SolverStatus::AlmostSolved => {}
        _ => return Err(Infeasible),
    }
    let x = &solver.solution.x;
    Ok(Profile {
        s0,
        ds,
        states: (0..n).map(|i| [x[l(i)], x[dl(i)], x[ddl(i)]]).collect(),
    })
}

/// A sparse matrix built entry by entry (repeats add up).
#[derive(Default)]
struct Triplets {
    rows: Vec<usize>,
    cols: Vec<usize>,
    values: Vec<f64>,
}

impl Triplets {
    fn push(&mut self, row: usize, col: usize, value: f64) {
        self.rows.push(row);
        self.cols.push(col);
        self.values.push(value);
    }

    fn matrix(self, m: usize, n: usize) -> CscMatrix<f64> {
        CscMatrix::new_from_triplets(m, n, self.rows, self.cols, self.values)
    }
}

/// `line` moved left by `profile`: each pose sideways by `l`, its yaw by `atan l'`.
#[must_use]
pub fn shifted(line: &[Pose], along: &[f64], profile: &Profile) -> Vec<Pose> {
    line.iter()
        .zip(along)
        .map(|(p, &s)| {
            let (o, slope) = profile.at(s);
            if o.abs() < 1e-9 && slope.abs() < 1e-9 {
                return *p;
            }
            let (sin, cos) = p.yaw.sin_cos();
            Pose::new(p.x - o * sin, p.y + o * cos, p.yaw + slope.atan())
        })
        .collect()
}

/// Pass whatever blocks `line` within the lateral jerk and acceleration limits at
/// `speed`: as [`pass::plan`] (the same arguments, and `None` when nothing blocks), the
/// path a piecewise-jerk QP. When the QP finds no answer, or its path isn't clear, the
/// half-cosine pass instead.
///
/// # Errors
///
/// [`NoRoom`] when the goal is blocked, or no shift up to the kerb fits.
#[allow(clippy::too_many_arguments)]
pub fn plan(
    line: &[Pose],
    along: &[f64],
    radius: f64,
    speed: f64,
    settings: &PassSettings,
    jerk: &JerkSettings,
    fits: &dyn Fn(&Pose) -> bool,
    room: &dyn Fn(&Pose) -> f64,
) -> Result<Option<Pass>, NoRoom> {
    let blocked: Vec<bool> = line.iter().map(|p| !fits(p)).collect();
    if !blocked.iter().any(|&b| b) {
        return Ok(None);
    }
    if blocked.last().copied().unwrap_or(false) {
        return Err(NoRoom);
    }
    let total = along.last().copied().unwrap_or(0.0);
    let near = |s: f64| line[along.partition_point(|&a| a < s).min(line.len() - 1)];
    // Each stretch held at a shift: the least that fits, and the clearance if it fits too.
    let mut bumps: Vec<Bump> = Vec::new();
    for (start, end) in pass::stretches(along, &blocked, settings.margin, settings.merge) {
        let most = room(&near(0.5 * (start + end)));
        let clear_at = |d: f64| {
            line.iter()
                .zip(along)
                .filter(|(_, &s)| s >= start && s <= end)
                .all(|(p, _)| {
                    let (sin, cos) = p.yaw.sin_cos();
                    fits(&Pose::new(p.x - d * sin, p.y + d * cos, p.yaw))
                })
        };
        let steps = (most / settings.step).floor().max(0.0) as usize;
        let least = (1..=steps)
            .map(|k| k as f64 * settings.step)
            .find(|&d| clear_at(d))
            .ok_or(NoRoom)?;
        let wider = (least + settings.clearance).min(most);
        let shift = if wider > least && clear_at(wider) {
            wider
        } else {
            least
        };
        let ramp = ramp_length(shift, speed, radius, jerk);
        bumps.push(Bump {
            start,
            end,
            shift,
            ramp_in: ramp.min(start.max(0.0)),
            ramp_out: ramp,
        });
    }
    let cosine = || pass::plan(line, along, radius, settings, fits, room);

    // The QP's stations: from before the first ramp to after the last (within the line).
    let first = bumps
        .iter()
        .map(|b| b.start - b.ramp_in)
        .fold(f64::MAX, f64::min);
    let last = bumps
        .iter()
        .map(|b| b.end + b.ramp_out)
        .fold(f64::MIN, f64::max);
    let s0 = (first - jerk.lead).max(0.0);
    let s1 = (last + jerk.lead).min(total);
    let stations = ((s1 - s0) / jerk.ds).ceil() as usize + 1;
    let bounds: Vec<(f64, f64)> = (0..stations)
        .map(|i| {
            let s = s0 + i as f64 * jerk.ds;
            let lo = bumps
                .iter()
                .filter(|b| s >= b.start && s <= b.end)
                .map(|b| b.shift)
                .fold(0.0, f64::max);
            (lo, room(&near(s)).max(lo))
        })
        .collect();
    let v = speed.abs().max(jerk.min_speed);
    let curvature = (jerk.lateral_accel / (v * v)).min(1.0 / radius);
    let jerk_s = jerk.lateral_jerk / (v * v * v);
    let Ok(profile) = solve(s0, &bounds, curvature, jerk_s, s0 > 0.0, jerk) else {
        return cosine();
    };
    let moved = shifted(line, along, &profile);
    if !moved.iter().all(fits) {
        return cosine();
    }
    Ok(Some(Pass { line: moved, bumps }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// East along y = 0, a pose every 0.25 m for `metres`.
    fn straight(metres: f64) -> (Vec<Pose>, Vec<f64>) {
        let n = (metres / 0.25) as usize + 1;
        let line: Vec<Pose> = (0..n)
            .map(|i| Pose::new(i as f64 * 0.25, 0.0, 0.0))
            .collect();
        let along = line.iter().map(|p| p.x).collect();
        (line, along)
    }

    /// The car (2 m long from 0.2 behind base_link, 1 m wide) against a box.
    fn boxed(x0: f64, x1: f64, y0: f64, y1: f64) -> impl Fn(&Pose) -> bool {
        move |p: &Pose| {
            let (cx0, cx1, cy0, cy1) = (p.x - 0.2, p.x + 1.8, p.y - 0.5, p.y + 0.5);
            cx1 <= x0 || cx0 >= x1 || cy1 <= y0 || cy0 >= y1
        }
    }

    /// The lateral jerk (m/s³) at `speed` along a moved straight line: v³ x l''', from
    /// the poses' y by finite differences.
    fn worst_jerk(line: &[Pose], h: f64, speed: f64) -> f64 {
        line.windows(4)
            .map(|w| (w[3].y - 3.0 * w[2].y + 3.0 * w[1].y - w[0].y).abs() / h.powi(3))
            .fold(0.0, f64::max)
            * speed.powi(3)
    }

    #[test]
    fn the_ramp_grows_with_the_speed_and_shrinks_with_the_jerk() {
        let s = JerkSettings::default();
        let slow = ramp_length(1.5, 2.0, 3.6, &s);
        let fast = ramp_length(1.5, 2.5, 3.6, &s);
        assert!((fast / slow - 1.25).abs() < 1e-9, "{slow} {fast}");
        // (32 x 1.5 / 0.5)^⅓ = 4.58 s at 2.5 m/s: 11.4 m.
        assert!((fast - 2.5 * 96f64.cbrt()).abs() < 1e-9, "{fast}");
        let firm = JerkSettings {
            lateral_jerk: 2.0,
            ..s
        };
        assert!(ramp_length(1.5, 2.5, 3.6, &firm) < fast);
        // Slow: never sharper than the turning radius.
        let crawl = ramp_length(1.5, 0.0, 3.6, &firm);
        assert!(crawl >= pass::ramp_length(1.5, 3.6, 1.0) - 1e-12);
    }

    #[test]
    fn the_profile_is_continuous_and_holds_its_bounds() {
        let s = JerkSettings::default();
        let bounds: Vec<(f64, f64)> = (0..80)
            .map(|i| {
                if (30..50).contains(&i) {
                    (1.0, 3.0)
                } else {
                    (0.0, 3.0)
                }
            })
            .collect();
        let jerk = 0.5 / 8.0;
        let p = solve(10.0, &bounds, 0.25, jerk, true, &s).expect("feasible");
        for (i, (lo, hi)) in bounds.iter().enumerate() {
            let l = p.states[i][0];
            assert!(l >= lo - 1e-6 && l <= hi + 1e-6, "{i}: {l}");
        }
        for w in p.states.windows(2) {
            assert!((w[1][2] - w[0][2]).abs() <= jerk * s.ds + 1e-6);
        }
        // The cubic joins each station's own value.
        for i in 0..79 {
            let at = p.at(10.0 + i as f64 * s.ds);
            assert!((at.0 - p.states[i][0]).abs() < 1e-6, "{i}");
        }
        assert!(p.states[0][0].abs() < 1e-6 && p.states[79][0].abs() < 1e-6);
    }

    #[test]
    fn a_box_on_the_line_is_passed_within_the_jerk_limit() {
        let (line, along) = straight(100.0);
        let fits = boxed(50.0, 51.0, -0.5, 0.5);
        let jerk = JerkSettings::default();
        let speed = 2.0;
        let pass = plan(
            &line,
            &along,
            3.6,
            speed,
            &PassSettings::default(),
            &jerk,
            &fits,
            &|_| 4.0,
        )
        .expect("room")
        .expect("a pass");
        assert!(pass.line.iter().all(&fits));
        // Held at the least shift and the clearance over the box, as the cosine pass.
        assert!((pass.shift() - 1.5).abs() < 1e-9, "{}", pass.shift());
        let over = pass
            .line
            .iter()
            .find(|p| (p.x - 50.5).abs() < 0.1)
            .expect("a pose");
        assert!(over.y >= 1.5 - 1e-3, "{over:?}");
        // The lateral jerk stays within the limit (a little over for the differences).
        let worst = worst_jerk(&pass.line, 0.25, speed);
        assert!(worst <= 1.2 * jerk.lateral_jerk, "{worst}");
        // Back on the line at both ends.
        assert!(pass.line[0].y.abs() < 1e-6 && pass.line.last().map_or(1.0, |p| p.y).abs() < 1e-6);
    }

    #[test]
    fn its_jerk_is_lower_than_the_cosine_pass_at_speed() {
        let (line, along) = straight(100.0);
        let fits = boxed(50.0, 51.0, -0.5, 0.5);
        let speed = 2.5;
        let settings = PassSettings::default();
        let jerk = plan(
            &line,
            &along,
            3.6,
            speed,
            &settings,
            &JerkSettings::default(),
            &fits,
            &|_| 4.0,
        )
        .expect("room")
        .expect("a pass");
        let cosine = pass::plan(&line, &along, 3.6, &settings, &fits, &|_| 4.0)
            .expect("room")
            .expect("a pass");
        let (a, b) = (
            worst_jerk(&jerk.line, 0.25, speed),
            worst_jerk(&cosine.line, 0.25, speed),
        );
        assert!(a < b, "jerk-limited {a} vs cosine {b}");
    }

    #[test]
    fn nothing_blocking_is_none_and_a_wall_is_no_room() {
        let (line, along) = straight(80.0);
        let none = plan(
            &line,
            &along,
            3.6,
            2.0,
            &PassSettings::default(),
            &JerkSettings::default(),
            &|_| true,
            &|_| 4.0,
        );
        assert!(matches!(none, Ok(None)));
        let wall = boxed(28.0, 33.0, -0.5, 3.5);
        let blocked = plan(
            &line,
            &along,
            3.6,
            2.0,
            &PassSettings::default(),
            &JerkSettings::default(),
            &wall,
            &|_| 3.5,
        );
        assert_eq!(blocked.err(), Some(NoRoom));
    }

    #[test]
    fn a_box_too_close_to_ramp_gently_falls_back_to_the_cosine_pass() {
        // The box 4 m ahead at 2.5 m/s: no jerk-limited ramp fits; the cosine one does.
        let (line, along) = straight(60.0);
        let fits = boxed(4.0, 5.0, -0.5, 0.5);
        let pass = plan(
            &line,
            &along,
            3.6,
            2.5,
            &PassSettings::default(),
            &JerkSettings::default(),
            &fits,
            &|_| 4.0,
        )
        .expect("room")
        .expect("a pass");
        assert!(pass.line.iter().all(&fits));
    }
}

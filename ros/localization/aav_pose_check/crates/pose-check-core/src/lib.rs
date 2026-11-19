//! NDT leads, the GPS checks (TODO 210). No ROS.
//!
//! With `pose_source:=ndt`, `ekf_global` takes its absolute pose from NDT (the lidar on a
//! saved map), and the GPS only checks it. This crate decides, every message and every
//! tick:
//!
//! | Question | Answer |
//! |---|---|
//! | which absolute pose may reach `ekf_global`? | NDT's while NDT is trusted; else the GPS's, if the GPS is healthy; else none ([`Source`]) |
//! | do they disagree? | over a window: the mean of d'(Σ_ndt + Σ_gps)⁻¹d above `disagree_nees`, **and** the mean gap above `disagree_m` |
//! | then who is wrong? | NDT, if its score per point was weak then (`trust_score`); else the GPS. The verdict holds until they agree again ([`Verdict`]) |
//! | where does a LOST NDT start again? | AMCL's pose if AMCL has converged; else the GPS's, with the fused heading; else nowhere, and the car holds ([`Seed`]) |
//!
//! Why the GPS is not fused at a low weight while NDT is good: a slow drift (a few metres
//! over a minute, no jump, never stale) would still pull the pose by its share, all the
//! time. Held out, it can't move the pose at all; it is still compared with NDT, so a
//! GPS that drifts is named, and never used as a fall-back while it disagrees.
//!
//! Times are seconds on one clock (the node's: the simulator's in Ravensim).
//! Positions are metres in the map frame (REP-105), headings radians.

use std::collections::VecDeque;
use std::fmt;

use thiserror::Error;

/// A pose on the map: x east, y north (m), yaw counter-clockwise from east (rad).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.yaw.is_finite()
    }
}

/// A position's covariance (m²), x and y.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cov2 {
    pub xx: f64,
    pub xy: f64,
    pub yy: f64,
}

impl Cov2 {
    pub fn new(xx: f64, xy: f64, yy: f64) -> Self {
        Self { xx, xy, yy }
    }

    /// The same spread in every direction: `std` m, 1 sigma.
    pub fn round(std: f64) -> Self {
        Self::new(std * std, 0.0, std * std)
    }

    /// The largest 1-sigma spread (m): the square root of the larger eigenvalue.
    pub fn std_max(&self) -> f64 {
        let mean = 0.5 * (self.xx + self.yy);
        let half = (0.25 * (self.xx - self.yy).powi(2) + self.xy * self.xy).sqrt();
        (mean + half).max(0.0).sqrt()
    }

    fn plus(self, other: Self) -> Self {
        Self::new(self.xx + other.xx, self.xy + other.xy, self.yy + other.yy)
    }

    /// d'Σ⁻¹d for a gap (dx, dy); None if Σ can't be inverted.
    fn mahalanobis_sq(&self, dx: f64, dy: f64) -> Option<f64> {
        let det = self.xx * self.yy - self.xy * self.xy;
        (det > 0.0 && det.is_finite())
            .then(|| (self.yy * dx * dx - 2.0 * self.xy * dx * dy + self.xx * dy * dy) / det)
    }

    fn is_finite(&self) -> bool {
        self.xx.is_finite() && self.xy.is_finite() && self.yy.is_finite()
    }
}

/// NDT's own state, from the message of its status (aav_ndt).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NdtState {
    Waiting,
    Tracking,
    Lost,
}

impl NdtState {
    /// Read the first word of `/localization/ndt/status`'s message.
    pub fn from_message(message: &str) -> Option<Self> {
        match message.split_whitespace().next()?.trim_end_matches(':') {
            "WAITING" => Some(Self::Waiting),
            "TRACKING" => Some(Self::Tracking),
            "LOST" => Some(Self::Lost),
            _ => None,
        }
    }
}

/// Whose absolute pose `ekf_global` takes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// NDT's: the normal case.
    Ndt,
    /// The GPS's: NDT is lost, stale or judged wrong, and the GPS is healthy.
    Gps,
    /// Neither: the car must hold (the sensor watchdog's lock, through our status).
    Hold,
}

impl fmt::Display for Source {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Ndt => "NDT",
            Self::Gps => "GPS",
            Self::Hold => "HOLD",
        })
    }
}

/// How loud the status is (diagnostic_msgs' levels).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Ok,
    Warn,
    Error,
}

impl Level {
    /// diagnostic_msgs/DiagnosticStatus's byte.
    pub fn byte(self) -> u8 {
        match self {
            Self::Ok => 0,
            Self::Warn => 1,
            Self::Error => 2,
        }
    }
}

/// Why the GPS isn't trusted now.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GpsDoubt {
    /// Nothing yet.
    Never,
    /// Nothing for this many seconds.
    Stale { age: f64 },
    /// The receiver says no fix, or sent NaN.
    NoFix,
    /// Its own 1-sigma spread is over `gps_max_std`.
    Inaccurate { std: f64 },
    /// Two fixes further apart than the car can drive: latched until it agrees with NDT.
    Jumped { metres: f64 },
    /// It disagreed with a strong NDT: latched until they agree.
    Disagrees { gap: f64 },
}

impl fmt::Display for GpsDoubt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Never => write!(f, "no GPS yet"),
            Self::Stale { age } => write!(f, "GPS silent {age:.1} s"),
            Self::NoFix => write!(f, "GPS has no fix"),
            Self::Inaccurate { std } => write!(f, "GPS spread {std:.1} m"),
            Self::Jumped { metres } => write!(f, "GPS jumped {metres:.1} m"),
            Self::Disagrees { gap } => write!(f, "GPS {gap:.1} m off NDT, judged wrong"),
        }
    }
}

/// Why NDT isn't trusted now.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NdtDoubt {
    /// Nothing yet.
    Never,
    /// No good fit for this many seconds.
    Stale { age: f64 },
    /// NDT says LOST.
    Lost,
    /// It disagreed with a healthy GPS while its score was weak: latched until they agree.
    Disagrees { gap: f64 },
}

impl fmt::Display for NdtDoubt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Never => write!(f, "no NDT fit yet"),
            Self::Stale { age } => write!(f, "no NDT fit for {age:.1} s"),
            Self::Lost => write!(f, "NDT lost"),
            Self::Disagrees { gap } => write!(
                f,
                "NDT {gap:.1} m off the GPS with a weak score, judged wrong"
            ),
        }
    }
}

/// Who was judged wrong when NDT and the GPS last disagreed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Verdict {
    Agree,
    GpsWrong { gap: f64 },
    NdtWrong { gap: f64 },
}

/// Where NDT's pose to start again came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeedFrom {
    Amcl,
    Gps,
}

impl fmt::Display for SeedFrom {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Amcl => "AMCL",
            Self::Gps => "the GPS",
        })
    }
}

/// A pose for NDT to start again from (`/localization/ndt/initial_pose`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seed {
    pub pose: Pose2,
    /// m², x and y each.
    pub var_xy: f64,
    /// rad².
    pub var_yaw: f64,
    pub from: SeedFrom,
}

/// A change of source, to log.
#[derive(Clone, Debug, PartialEq)]
pub struct Handover {
    pub from: Source,
    pub to: Source,
    pub why: String,
}

/// What the node publishes as its status.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub source: Source,
    pub level: Level,
    /// One line: the source, then why.
    pub message: String,
    pub gps: Option<GpsDoubt>,
    pub ndt: Option<NdtDoubt>,
    /// The window's mean gap NDT to GPS (m), and its mean d'Σ⁻¹d.
    pub gap: Option<f64>,
    pub nees: Option<f64>,
    /// NDT's median score per point over the window.
    pub score: Option<f64>,
    pub handovers: u32,
    pub seeds: u32,
}

/// One tick's answer.
#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    pub report: Report,
    pub handover: Option<Handover>,
    pub seed: Option<Seed>,
}

/// The numbers (config/pose_check.yaml says why each is what it is).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// s without a good NDT fit: NDT stale.
    pub ndt_max_age: f64,
    /// NDT's median score per point at which it wins a disagreement.
    pub trust_score: f64,
    /// s without a GPS position: stale.
    pub gps_max_age: f64,
    /// m: a GPS position whose own 1-sigma spread is over this isn't used.
    pub gps_max_std: f64,
    /// m/s and m: two fixes further apart than this speed allows, plus the margin: a jump.
    pub max_jump_speed: f64,
    pub jump_margin: f64,
    /// s of NDT-GPS pairs judged together, and how many at least.
    pub window: f64,
    pub min_pairs: usize,
    /// s: a GPS position pairs with the NDT fit nearest in time, within this.
    pub pair_max_dt: f64,
    /// m²: added to both covariances, for the time between a pair's stamps.
    pub pair_var: f64,
    /// They disagree when the window's mean d'Σ⁻¹d is over `disagree_nees` and its mean gap
    /// over `disagree_m`; they agree again when the gap is under `agree_m` and d'Σ⁻¹d
    /// under `disagree_nees`.
    pub disagree_nees: f64,
    pub disagree_m: f64,
    pub agree_m: f64,
    /// AMCL has converged when its pose is this fresh (s), and its spread under these.
    pub amcl_max_age: f64,
    pub amcl_max_std: f64,
    pub amcl_max_yaw_std: f64,
    /// s between two seeds for NDT while it stays lost.
    pub reseed_every: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            ndt_max_age: 1.0,
            trust_score: 2.3,
            gps_max_age: 2.0,
            gps_max_std: 3.0,
            max_jump_speed: 15.0,
            jump_margin: 5.0,
            window: 5.0,
            min_pairs: 5,
            pair_max_dt: 0.25,
            pair_var: 0.09,
            disagree_nees: 13.8,
            disagree_m: 1.5,
            agree_m: 1.0,
            amcl_max_age: 2.0,
            amcl_max_std: 0.5,
            amcl_max_yaw_std: 0.17,
            reseed_every: 5.0,
        }
    }
}

/// A setting that can't work.
#[derive(Debug, Error, PartialEq)]
pub enum SettingsError {
    #[error("{name} = {value}: must be above 0")]
    NotPositive { name: &'static str, value: f64 },
    #[error("agree_m = {agree} must be under disagree_m = {disagree}, or they would flicker")]
    NoHysteresis { agree: f64, disagree: f64 },
    #[error("min_pairs must be at least 1")]
    NoPairs,
}

impl Settings {
    /// Every number above 0, and agreement below disagreement.
    pub fn check(&self) -> Result<(), SettingsError> {
        let positive = [
            ("ndt_max_age", self.ndt_max_age),
            ("trust_score", self.trust_score),
            ("gps_max_age", self.gps_max_age),
            ("gps_max_std", self.gps_max_std),
            ("max_jump_speed", self.max_jump_speed),
            ("window", self.window),
            ("pair_max_dt", self.pair_max_dt),
            ("disagree_nees", self.disagree_nees),
            ("disagree_m", self.disagree_m),
            ("agree_m", self.agree_m),
            ("amcl_max_age", self.amcl_max_age),
            ("amcl_max_std", self.amcl_max_std),
            ("amcl_max_yaw_std", self.amcl_max_yaw_std),
            ("reseed_every", self.reseed_every),
        ];
        if let Some(&(name, value)) = positive.iter().find(|(_, v)| !(*v > 0.0 && v.is_finite())) {
            return Err(SettingsError::NotPositive { name, value });
        }
        if !(self.jump_margin >= 0.0 && self.pair_var >= 0.0) {
            let (name, value) = if self.jump_margin >= 0.0 {
                ("pair_var", self.pair_var)
            } else {
                ("jump_margin", self.jump_margin)
            };
            return Err(SettingsError::NotPositive { name, value });
        }
        if self.agree_m >= self.disagree_m {
            return Err(SettingsError::NoHysteresis {
                agree: self.agree_m,
                disagree: self.disagree_m,
            });
        }
        if self.min_pairs == 0 {
            return Err(SettingsError::NoPairs);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct Fit {
    t: f64,
    pose: Pose2,
    cov: Cov2,
}

#[derive(Clone, Copy, Debug)]
struct Fix {
    t: f64,
    x: f64,
    y: f64,
    cov: Cov2,
}

#[derive(Clone, Copy, Debug)]
struct Pair {
    t: f64,
    nees: f64,
    gap: f64,
}

#[derive(Clone, Copy, Debug)]
struct Amcl {
    t: f64,
    pose: Pose2,
    std: f64,
    yaw_std: f64,
}

/// s of NDT fits kept to pair with the GPS.
const KEEP_FITS: f64 = 2.0;

/// The checker: feed it the messages, tick it, publish what it says.
#[derive(Debug)]
pub struct Checker {
    s: Settings,
    fits: VecDeque<Fit>,
    ndt_state: Option<NdtState>,
    scores: VecDeque<(f64, f64)>,
    fix: Option<Fix>,
    fix_ok: Option<bool>,
    jumped: Option<f64>,
    pairs: VecDeque<Pair>,
    verdict: Verdict,
    amcl: Option<Amcl>,
    heading: Option<f64>,
    source: Source,
    last_seed: Option<f64>,
    handovers: u32,
    seeds: u32,
}

impl Checker {
    /// A checker with these settings; they must pass [`Settings::check`].
    pub fn new(s: Settings) -> Result<Self, SettingsError> {
        s.check()?;
        Ok(Self {
            s,
            fits: VecDeque::new(),
            ndt_state: None,
            scores: VecDeque::new(),
            fix: None,
            fix_ok: None,
            jumped: None,
            pairs: VecDeque::new(),
            verdict: Verdict::Agree,
            amcl: None,
            heading: None,
            source: Source::Hold,
            last_seed: None,
            handovers: 0,
            seeds: 0,
        })
    }

    /// The source now (as of the last tick).
    pub fn source(&self) -> Source {
        self.source
    }

    /// The last verdict on a disagreement.
    pub fn verdict(&self) -> Verdict {
        self.verdict
    }

    /// NDT's status: its state, and the score per point of this cloud's fit if it had one.
    pub fn ndt_status(&mut self, t: f64, state: NdtState, score: Option<f64>) {
        self.ndt_state = Some(state);
        if let Some(score) = score.filter(|s| s.is_finite()) {
            self.scores.push_back((t, score));
        }
        let oldest = t - self.s.window;
        while self.scores.front().is_some_and(|&(ts, _)| ts < oldest) {
            self.scores.pop_front();
        }
    }

    /// A good NDT fit (its cloud's stamp). Returns whether to pass it to `ekf_global`.
    pub fn ndt_pose(&mut self, t: f64, pose: Pose2, cov: Cov2) -> bool {
        if !(pose.is_finite() && cov.is_finite()) {
            return false;
        }
        self.fits.push_back(Fit { t, pose, cov });
        while self.fits.front().is_some_and(|f| f.t < t - KEEP_FITS) {
            self.fits.pop_front();
        }
        self.ndt_doubt(t).is_none()
    }

    /// The receiver's own say on a fix (`/sensors/gps/fix`): a fix, with finite numbers.
    pub fn gps_fix(&mut self, ok: bool) {
        self.fix_ok = Some(ok);
    }

    /// A GPS position on the map (navsat_transform's). Returns whether to pass it to
    /// `ekf_global`: only while NDT isn't trusted, and the GPS is.
    pub fn gps(&mut self, t: f64, x: f64, y: f64, cov: Cov2) -> bool {
        if !(x.is_finite() && y.is_finite() && cov.is_finite()) {
            self.fix_ok = Some(false);
            return false;
        }
        if let Some(last) = self.fix {
            let step = (x - last.x).hypot(y - last.y);
            let dt = (t - last.t).abs();
            if step > self.s.max_jump_speed * dt + self.s.jump_margin {
                self.jumped = Some(step);
            }
        }
        self.fix = Some(Fix { t, x, y, cov });
        self.pair(t, x, y, cov);
        self.ndt_doubt(t).is_some() && self.gps_doubt(t).is_none()
    }

    /// AMCL's pose and its spread (m and rad, 1 sigma).
    pub fn amcl(&mut self, t: f64, pose: Pose2, cov: Cov2, var_yaw: f64) {
        if pose.is_finite() && cov.is_finite() && var_yaw.is_finite() {
            self.amcl = Some(Amcl {
                t,
                pose,
                std: cov.std_max(),
                yaw_std: var_yaw.max(0.0).sqrt(),
            });
        }
    }

    /// The fused heading (`ekf_global`'s): a GPS seed's yaw.
    pub fn heading(&mut self, yaw: f64) {
        if yaw.is_finite() {
            self.heading = Some(yaw);
        }
    }

    /// Why NDT isn't trusted at time `now`, or None.
    pub fn ndt_doubt(&self, now: f64) -> Option<NdtDoubt> {
        if self.ndt_state == Some(NdtState::Lost) {
            return Some(NdtDoubt::Lost);
        }
        let Some(last) = self.fits.back() else {
            return Some(NdtDoubt::Never);
        };
        let age = now - last.t;
        if age > self.s.ndt_max_age {
            return Some(NdtDoubt::Stale { age });
        }
        match self.verdict {
            Verdict::NdtWrong { gap } => Some(NdtDoubt::Disagrees { gap }),
            _ => None,
        }
    }

    /// Why the GPS isn't trusted at time `now`, or None.
    pub fn gps_doubt(&self, now: f64) -> Option<GpsDoubt> {
        let Some(fix) = self.fix else {
            return Some(GpsDoubt::Never);
        };
        let age = now - fix.t;
        if age > self.s.gps_max_age {
            return Some(GpsDoubt::Stale { age });
        }
        if self.fix_ok == Some(false) {
            return Some(GpsDoubt::NoFix);
        }
        let std = fix.cov.std_max();
        if std > self.s.gps_max_std {
            return Some(GpsDoubt::Inaccurate { std });
        }
        if let Some(metres) = self.jumped {
            return Some(GpsDoubt::Jumped { metres });
        }
        match self.verdict {
            Verdict::GpsWrong { gap } => Some(GpsDoubt::Disagrees { gap }),
            _ => None,
        }
    }

    /// NDT's median score per point over the window.
    pub fn score(&self) -> Option<f64> {
        let mut s: Vec<f64> = self.scores.iter().map(|&(_, s)| s).collect();
        if s.is_empty() {
            return None;
        }
        s.sort_by(f64::total_cmp);
        Some(s[s.len() / 2])
    }

    /// The window's mean gap (m) and mean d'Σ⁻¹d, once it holds `min_pairs`.
    pub fn gap(&self) -> Option<(f64, f64)> {
        let n = self.pairs.len();
        (n >= self.s.min_pairs && n > 0).then(|| {
            let gap = self.pairs.iter().map(|p| p.gap).sum::<f64>() / n as f64;
            let nees = self.pairs.iter().map(|p| p.nees).sum::<f64>() / n as f64;
            (gap, nees)
        })
    }

    /// Pairs a GPS position with the NDT fit nearest in time, and judges the window.
    fn pair(&mut self, t: f64, x: f64, y: f64, cov: Cov2) {
        let oldest = t - self.s.window;
        while self.pairs.front().is_some_and(|p| p.t < oldest) {
            self.pairs.pop_front();
        }
        let near = self
            .fits
            .iter()
            .filter(|f| (f.t - t).abs() <= self.s.pair_max_dt)
            .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()));
        let Some(fit) = near.copied() else {
            return;
        };
        let (dx, dy) = (x - fit.pose.x, y - fit.pose.y);
        let both = cov.plus(fit.cov).plus(Cov2::round(self.s.pair_var.sqrt()));
        let Some(nees) = both.mahalanobis_sq(dx, dy) else {
            return;
        };
        self.pairs.push_back(Pair {
            t,
            nees,
            gap: dx.hypot(dy),
        });
        self.judge(t);
    }

    /// NDT is strong: tracking, with a median score at or over `trust_score`.
    fn ndt_strong(&self) -> bool {
        self.ndt_state == Some(NdtState::Tracking)
            && self.score().is_some_and(|s| s >= self.s.trust_score)
    }

    /// The window's verdict. A disagreement's verdict holds until they agree again: a
    /// GPS judged wrong while NDT was strong stays wrong through a weak moment of NDT's.
    fn judge(&mut self, now: f64) {
        let Some((gap, nees)) = self.gap() else {
            return;
        };
        if gap < self.s.agree_m && nees < self.s.disagree_nees {
            self.verdict = Verdict::Agree;
            self.jumped = None;
            return;
        }
        if !(gap > self.s.disagree_m && nees > self.s.disagree_nees) {
            return;
        }
        self.verdict = match self.verdict {
            Verdict::Agree if !self.ndt_strong() && self.gps_alone_healthy(now) => {
                Verdict::NdtWrong { gap }
            }
            Verdict::Agree | Verdict::GpsWrong { .. } => Verdict::GpsWrong { gap },
            Verdict::NdtWrong { .. } => Verdict::NdtWrong { gap },
        };
    }

    /// The GPS's own checks, without the verdict: fresh, a fix, accurate, no jump.
    fn gps_alone_healthy(&self, now: f64) -> bool {
        matches!(self.gps_doubt(now), None | Some(GpsDoubt::Disagrees { .. }))
    }

    /// AMCL's pose, if it has converged.
    fn amcl_seed(&self, now: f64) -> Option<Seed> {
        let a = self.amcl?;
        (now - a.t <= self.s.amcl_max_age
            && a.std <= self.s.amcl_max_std
            && a.yaw_std <= self.s.amcl_max_yaw_std)
            .then_some(Seed {
                pose: a.pose,
                var_xy: a.std * a.std,
                var_yaw: a.yaw_std * a.yaw_std,
                from: SeedFrom::Amcl,
            })
    }

    /// The GPS's position with the fused heading, if the GPS is healthy.
    fn gps_seed(&self, now: f64) -> Option<Seed> {
        if self.gps_doubt(now).is_some() {
            return None;
        }
        let (fix, yaw) = (self.fix?, self.heading?);
        let std = fix.cov.std_max();
        Some(Seed {
            pose: Pose2::new(fix.x, fix.y, yaw),
            var_xy: std * std,
            // 10 degrees: the fused heading, carried by the gyro while NDT was lost.
            var_yaw: 0.17f64.powi(2),
            from: SeedFrom::Gps,
        })
    }

    /// Decide the source at `now`; say why; and a seed for NDT when it is due.
    pub fn tick(&mut self, now: f64) -> Tick {
        let ndt = self.ndt_doubt(now);
        let gps = self.gps_doubt(now);
        let source = match (ndt, gps) {
            (None, _) => Source::Ndt,
            (Some(_), None) => Source::Gps,
            (Some(_), Some(_)) => Source::Hold,
        };
        let mut message = match (source, ndt, gps) {
            (Source::Ndt, _, None) => "NDT: the GPS agrees".to_string(),
            (Source::Ndt, _, Some(GpsDoubt::Never)) => "NDT: no GPS to check it".to_string(),
            (Source::Ndt, _, Some(g)) => format!("NDT: {g}; NDT carries on"),
            (Source::Gps, Some(n), _) => format!("GPS: {n}"),
            (Source::Hold, Some(n), Some(g)) => format!("HOLD: {n}, {g}"),
            _ => source.to_string(),
        };
        let handover = (source != self.source).then(|| Handover {
            from: self.source,
            to: source,
            why: message.clone(),
        });
        if handover.is_some() {
            self.source = source;
            self.handovers += 1;
        }
        let due = self
            .last_seed
            .is_none_or(|t| now - t >= self.s.reseed_every || now < t);
        let needs = matches!(ndt, Some(NdtDoubt::Lost | NdtDoubt::Disagrees { .. }));
        let seed = if needs && due {
            self.amcl_seed(now).or_else(|| self.gps_seed(now))
        } else {
            None
        };
        if let Some(seed) = seed {
            self.last_seed = Some(now);
            self.seeds += 1;
            // A fresh start: judge NDT again on new pairs only.
            self.pairs.clear();
            message.push_str(&format!("; NDT seeded from {}", seed.from));
        } else if needs && due {
            message.push_str("; no seed for NDT (AMCL not converged, GPS not trusted)");
        }
        let level = match (source, gps) {
            (Source::Ndt, None | Some(GpsDoubt::Never)) => Level::Ok,
            (Source::Ndt | Source::Gps, _) => Level::Warn,
            (Source::Hold, _) => Level::Error,
        };
        let (gap, nees) = self.gap().unzip();
        Tick {
            report: Report {
                source,
                level,
                message,
                gps,
                ndt,
                gap,
                nees,
                score: self.score(),
                handovers: self.handovers,
                seeds: self.seeds,
            },
            handover,
            seed,
        }
    }
}

#[cfg(test)]
mod tests;

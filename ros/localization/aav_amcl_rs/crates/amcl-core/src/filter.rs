//! The particle filter: guesses of where the car is, each with a weight.
//!
//! nav2_amcl's `pf.c`, in Rust: Gaussian or uniform start; weights from a scan model;
//! the running averages that decide on random particles (recovery, *Probabilistic
//! Robotics* p. 258); resampling with a KLD-adaptive count (Fox 2003); the clusters and
//! the estimate (the heaviest cluster's mean, the whole set's covariance).
//!
//! Where ours differs, on purpose: resampling is low-variance (one random comb across the
//! weights, *Probabilistic Robotics* p. 110), its picks shuffled so KLD can stop at any
//! count and still keep a fair sample (nav2 drew each pick on its own: "can't easily
//! combine low-variance with KLD"); yaw is kept in (-π, π] and its bins wrap round, so a
//! cluster across ±π is one cluster; recovery's running averages are of the weighted mean
//! likelihood (nav2's were also divided by the particle count), and after an injection
//! only the fast one starts again (nav2 zeroed both). Both are explained where they are.

use std::collections::{HashMap, HashSet, VecDeque};
use std::f64::consts::PI;

use crate::{
    amcl::SettingsError,
    map::FreeSpace,
    motion::DifferentialMotion,
    pose::{wrap, Pose2},
    rng::Rng,
    scan::{Scan, ScanModel},
};

/// One guess: a pose and how much it is believed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Particle {
    /// Where the car might be (`base_link` in the map frame).
    pub pose: Pose2,
    /// Its weight; a set's weights sum to 1.
    pub weight: f64,
}

impl Particle {
    /// A particle at `pose` with `weight`.
    #[must_use]
    pub const fn new(pose: Pose2, weight: f64) -> Self {
        Self { pose, weight }
    }
}

/// The filter's settings: nav2_amcl's names.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FilterSettings {
    /// The fewest particles KLD may leave (min_particles).
    pub min_particles: usize,
    /// The most (max_particles); a start or a reset uses this many.
    pub max_particles: usize,
    /// KLD's bound on the error between the true and the sampled distribution (pf_err).
    pub pf_err: f64,
    /// KLD's normal quantile (pf_z). nav2 passes its 0.99 straight into Fox's formula as
    /// the quantile itself (the comment says "upper standard normal quantile"); we do the
    /// same, so the counts match the stock node's.
    pub pf_z: f64,
    /// The slow running average's rate (recovery_alpha_slow); 0: no recovery.
    pub alpha_slow: f64,
    /// The fast running average's rate (recovery_alpha_fast); 0: no recovery.
    pub alpha_fast: f64,
}

impl Default for FilterSettings {
    /// nav2_amcl's defaults: 500 to 2000 particles, KLD 0.05 and 0.99, no recovery.
    fn default() -> Self {
        Self {
            min_particles: 500,
            max_particles: 2000,
            pf_err: 0.05,
            pf_z: 0.99,
            alpha_slow: 0.0,
            alpha_fast: 0.0,
        }
    }
}

impl FilterSettings {
    /// Checks the settings make sense together.
    ///
    /// # Errors
    /// [`SettingsError`] for no particles, a minimum above the maximum, KLD numbers not
    /// above 0, or recovery rates outside 0 to 1.
    pub fn check(&self) -> Result<(), SettingsError> {
        if self.max_particles == 0 {
            return Err(SettingsError::new("max_particles", "must be above 0"));
        }
        if self.min_particles > self.max_particles {
            return Err(SettingsError::new(
                "min_particles",
                "must not be above max_particles",
            ));
        }
        if !(self.pf_err > 0.0 && self.pf_z > 0.0) {
            return Err(SettingsError::new("pf_err, pf_z", "must be above 0"));
        }
        for (name, a) in [
            ("recovery_alpha_slow", self.alpha_slow),
            ("recovery_alpha_fast", self.alpha_fast),
        ] {
            if !(0.0..=1.0).contains(&a) {
                return Err(SettingsError::new(name, "must be from 0 to 1"));
            }
        }
        Ok(())
    }
}

/// The filter's answer: where the car most likely is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    /// The heaviest cluster's mean.
    pub pose: Pose2,
    /// x, y and yaw's covariance over the whole set (nav2 reports the set's, not the
    /// cluster's); yaw's is circular (-2 ln R), and not correlated with x or y.
    pub covariance: [[f64; 3]; 3],
    /// How many clusters (separate hypotheses) the particles form.
    pub clusters: usize,
    /// The heaviest cluster's share of the weight, 0 to 1.
    pub weight: f64,
}

impl Estimate {
    /// The covariance as a `PoseWithCovariance`'s 6 x 6, row major (x, y, z, roll,
    /// pitch, yaw): x and y's block and yaw's variance; the rest 0, as nav2's.
    #[must_use]
    pub fn covariance_6x6(&self) -> [f64; 36] {
        let c = &self.covariance;
        let mut out = [0.0; 36];
        out[0] = c[0][0];
        out[1] = c[0][1];
        out[6] = c[1][0];
        out[7] = c[1][1];
        out[35] = c[2][2];
        out
    }
}

/// A KLD bin: 0.5 m x 0.5 m x 10°, as nav2's k-d tree's.
type Bin = (i64, i64, i64);

const BIN_XY: f64 = 0.5;
const YAW_BINS: i64 = 36;

fn bin(p: &Pose2) -> Bin {
    let yaw_bin = ((wrap(p.yaw) + PI) / (2.0 * PI / YAW_BINS as f64)).floor() as i64;
    (
        (p.x / BIN_XY).floor() as i64,
        (p.y / BIN_XY).floor() as i64,
        yaw_bin.rem_euclid(YAW_BINS),
    )
}

/// How many particles `k` filled bins need (Fox 2003; nav2's `pf_resample_limit`),
/// within the settings' minimum and maximum.
#[must_use]
pub fn kld_limit(k: usize, s: &FilterSettings) -> usize {
    if k <= 1 {
        return s.max_particles;
    }
    let k1 = (k - 1) as f64;
    let b = 2.0 / (9.0 * k1);
    let x = 1.0 - b + b.sqrt() * s.pf_z;
    let n = (k1 / (2.0 * s.pf_err) * x * x * x).ceil();
    if n >= s.max_particles as f64 {
        s.max_particles
    } else {
        (n.max(0.0) as usize).max(s.min_particles)
    }
}

/// The particle filter.
#[derive(Clone, Debug)]
pub struct ParticleFilter {
    settings: FilterSettings,
    particles: Vec<Particle>,
    w_slow: f64,
    w_fast: f64,
    rng: Rng,
}

impl ParticleFilter {
    /// A filter with `settings`, its random numbers from `seed`; every particle at the
    /// origin until [`ParticleFilter::start_gaussian`] or [`ParticleFilter::start_uniform`].
    ///
    /// # Errors
    /// [`SettingsError`] from [`FilterSettings::check`].
    pub fn new(settings: FilterSettings, seed: u64) -> Result<Self, SettingsError> {
        settings.check()?;
        let n = settings.max_particles;
        Ok(Self {
            settings,
            particles: vec![Particle::new(Pose2::default(), 1.0 / n as f64); n],
            w_slow: 0.0,
            w_fast: 0.0,
            rng: Rng::new(seed),
        })
    }

    /// The particles now.
    #[must_use]
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> &FilterSettings {
        &self.settings
    }

    /// The slow and fast running averages of the weight (recovery's inputs).
    #[must_use]
    pub fn averages(&self) -> (f64, f64) {
        (self.w_slow, self.w_fast)
    }

    /// Starts again: `max_particles` around `mean`, from x and y's 2 x 2 block of
    /// `covariance` and yaw's variance (nav2 reads no more of it).
    pub fn start_gaussian(&mut self, mean: Pose2, covariance: &[[f64; 3]; 3]) {
        // x and y by the 2 x 2 block's Cholesky factor; yaw on its own.
        let (a, b, c) = (covariance[0][0], covariance[0][1], covariance[1][1]);
        let l11 = a.max(0.0).sqrt();
        let l21 = if l11 > 0.0 { b / l11 } else { 0.0 };
        let l22 = (c - l21 * l21).max(0.0).sqrt();
        let sd_yaw = covariance[2][2].max(0.0).sqrt();
        let n = self.settings.max_particles;
        let rng = &mut self.rng;
        self.particles = (0..n)
            .map(|_| {
                let (u, v) = (rng.normal(), rng.normal());
                let pose = Pose2::new(
                    mean.x + l11 * u,
                    mean.y + l21 * u + l22 * v,
                    wrap(mean.yaw + sd_yaw * rng.normal()),
                );
                Particle::new(pose, 1.0 / n as f64)
            })
            .collect();
        self.w_slow = 0.0;
        self.w_fast = 0.0;
    }

    /// Starts again with `max_particles` anywhere free on the map (global localization).
    /// Returns false, with nothing changed, when the map has no free cell.
    pub fn start_uniform(&mut self, free: &FreeSpace) -> bool {
        if free.is_empty() {
            return false;
        }
        let n = self.settings.max_particles;
        let rng = &mut self.rng;
        self.particles = (0..n)
            .filter_map(|_| free.sample(rng))
            .map(|pose| Particle::new(pose, 1.0 / n as f64))
            .collect();
        self.w_slow = 0.0;
        self.w_fast = 0.0;
        true
    }

    /// Moves every particle as the odometry moved from `from` to `to`.
    pub fn predict(&mut self, motion: &DifferentialMotion, from: &Pose2, to: &Pose2) {
        motion.apply(&mut self.particles, from, to, &mut self.rng);
    }

    /// Weighs every particle by `model`'s likelihood of `scan`, and updates the running
    /// averages. With no weight at all (every likelihood 0), every particle weighs the same.
    pub fn weigh<M: ScanModel + ?Sized>(&mut self, model: &M, scan: &Scan) {
        let poses: Vec<Pose2> = self.particles.iter().map(|p| p.pose).collect();
        let likelihoods = model.likelihoods(&poses, scan);
        let total: f64 = self
            .particles
            .iter_mut()
            .zip(likelihoods)
            .map(|(p, l)| {
                p.weight *= if l.is_finite() { l.max(0.0) } else { 0.0 };
                p.weight
            })
            .sum();
        let n = self.particles.len() as f64;
        if total > 0.0 && total.is_finite() {
            self.particles.iter_mut().for_each(|p| p.weight /= total);
            // The weighted mean likelihood. nav2 divides this by the particle count too,
            // so its averages jumped 4x whenever KLD went from 2000 to 500 particles, and
            // recovery fired late or never; the mean itself doesn't care how many.
            let w_avg = total;
            let ema = |w: f64, alpha: f64| {
                if w == 0.0 {
                    w_avg
                } else {
                    w + alpha * (w_avg - w)
                }
            };
            self.w_slow = ema(self.w_slow, self.settings.alpha_slow);
            self.w_fast = ema(self.w_fast, self.settings.alpha_fast);
        } else {
            self.particles.iter_mut().for_each(|p| p.weight = 1.0 / n);
        }
    }

    /// The share of new particles that recovery puts at random: 1 - w_fast / w_slow, or 0.
    #[must_use]
    pub fn random_share(&self) -> f64 {
        if self.w_slow > 0.0 {
            (1.0 - self.w_fast / self.w_slow).max(0.0)
        } else {
            0.0
        }
    }

    /// Draws a new set: as many as KLD says the spread needs, each a copy of an old one
    /// in proportion to its weight, or (recovery, with `free`) a random pose.
    pub fn resample(&mut self, free: Option<&FreeSpace>) {
        let max = self.settings.max_particles;
        let random_share = free.map_or(0.0, |_| self.random_share());
        let total: f64 = self.particles.iter().map(|p| p.weight).sum();
        let picks = if total > 0.0 && total.is_finite() {
            self.comb(total, max)
        } else {
            (0..max)
                .map(|_| self.rng.below(self.particles.len()))
                .collect()
        };
        let mut bins = HashSet::new();
        let mut next = Vec::with_capacity(max);
        for i in picks {
            let random = free
                .filter(|_| random_share > 0.0 && self.rng.uniform() < random_share)
                .and_then(|free| free.sample(&mut self.rng));
            let pose = random.unwrap_or(self.particles[i].pose);
            bins.insert(bin(&pose));
            next.push(pose);
            if next.len() >= kld_limit(bins.len(), &self.settings) {
                break;
            }
        }
        // After injecting, the fast average starts again from the slow one, so the next
        // injection waits for the fit to fall again: no spiral into randomness. nav2 set
        // both to 0, which also forgot what a good fit scores (the slow average), so a
        // filter stuck on a wrong place after one injection never injected again.
        if random_share > 0.0 {
            self.w_fast = self.w_slow;
        }
        let w = 1.0 / next.len() as f64;
        self.particles = next
            .into_iter()
            .map(|pose| Particle::new(pose, w))
            .collect();
    }

    /// `n` low-variance picks across the weights (one random offset, even spacing), in a
    /// random order: any first part of them is still a fair sample.
    fn comb(&mut self, total: f64, n: usize) -> Vec<usize> {
        let step = total / n as f64;
        let start = (1.0 - self.rng.uniform()) * step; // in [0, step)
        let last = self.particles.len() - 1;
        let mut picks = Vec::with_capacity(n);
        let (mut i, mut reached) = (0, self.particles[0].weight);
        for m in 0..n {
            let target = start + m as f64 * step;
            while target >= reached && i < last {
                i += 1;
                reached += self.particles[i].weight;
            }
            picks.push(i);
        }
        self.rng.shuffle(&mut picks);
        picks
    }

    /// True when every particle is within 0.5 m (x and y each) of their mean: nav2's
    /// `pf_update_converged`.
    #[must_use]
    pub fn converged(&self) -> bool {
        let n = self.particles.len() as f64;
        let mx = self.particles.iter().map(|p| p.pose.x).sum::<f64>() / n;
        let my = self.particles.iter().map(|p| p.pose.y).sum::<f64>() / n;
        self.particles
            .iter()
            .all(|p| (p.pose.x - mx).abs() <= 0.5 && (p.pose.y - my).abs() <= 0.5)
    }

    /// Where the car most likely is: the heaviest cluster's mean, the whole set's
    /// covariance. None when no particle has weight.
    #[must_use]
    pub fn estimate(&self) -> Option<Estimate> {
        let clusters = self.clusters();
        let mut sums = vec![Sums::default(); clusters.count];
        let mut all = Sums::default();
        for (p, &c) in self.particles.iter().zip(&clusters.of) {
            sums[c].add(p);
            all.add(p);
        }
        let best = sums
            .iter()
            .max_by(|a, b| a.w.total_cmp(&b.w))
            .filter(|s| s.w > 0.0)?;
        let mean = all.mean();
        let var = |sq: f64, m1: f64, m2: f64| sq / all.w - m1 * m2;
        let cxy = var(all.xy, mean.x, mean.y);
        let r = all.cos.hypot(all.sin) / all.w;
        let covariance = [
            [var(all.xx, mean.x, mean.x), cxy, 0.0],
            [cxy, var(all.yy, mean.y, mean.y), 0.0],
            [0.0, 0.0, -2.0 * r.max(f64::MIN_POSITIVE).ln()],
        ];
        Some(Estimate {
            pose: best.mean(),
            covariance,
            clusters: clusters.count,
            weight: best.w / all.w,
        })
    }

    /// The particles' clusters: bins that touch (by face, edge or corner, yaw wrapping
    /// round) are one cluster, as nav2's k-d tree clusters them.
    fn clusters(&self) -> Clusters {
        let keys: Vec<Bin> = self.particles.iter().map(|p| bin(&p.pose)).collect();
        let mut label: HashMap<Bin, usize> = keys.iter().map(|k| (*k, usize::MAX)).collect();
        let mut count = 0;
        let mut queue = VecDeque::new();
        for key in &keys {
            if label[key] != usize::MAX {
                continue;
            }
            label.insert(*key, count);
            queue.push_back(*key);
            while let Some((x, y, t)) = queue.pop_front() {
                for (dx, dy, dt) in neighbours() {
                    let n = (x + dx, y + dy, (t + dt).rem_euclid(YAW_BINS));
                    if let Some(l) = label.get_mut(&n) {
                        if *l == usize::MAX {
                            *l = count;
                            queue.push_back(n);
                        }
                    }
                }
            }
            count += 1;
        }
        Clusters {
            of: keys.iter().map(|k| label[k]).collect(),
            count,
        }
    }
}

/// The 26 bins round a bin.
fn neighbours() -> impl Iterator<Item = (i64, i64, i64)> {
    (-1..=1)
        .flat_map(|dx| (-1..=1).flat_map(move |dy| (-1..=1).map(move |dt| (dx, dy, dt))))
        .filter(|&d| d != (0, 0, 0))
}

struct Clusters {
    /// Each particle's cluster.
    of: Vec<usize>,
    count: usize,
}

/// Weighted sums over particles, for a mean and a covariance.
#[derive(Clone, Copy, Debug, Default)]
struct Sums {
    w: f64,
    x: f64,
    y: f64,
    cos: f64,
    sin: f64,
    xx: f64,
    xy: f64,
    yy: f64,
}

impl Sums {
    fn add(&mut self, p: &Particle) {
        let (w, q) = (p.weight, p.pose);
        self.w += w;
        self.x += w * q.x;
        self.y += w * q.y;
        self.cos += w * q.yaw.cos();
        self.sin += w * q.yaw.sin();
        self.xx += w * q.x * q.x;
        self.xy += w * q.x * q.y;
        self.yy += w * q.y * q.y;
    }

    fn mean(&self) -> Pose2 {
        Pose2::new(self.x / self.w, self.y / self.w, self.sin.atan2(self.cos))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filter(min: usize, max: usize) -> ParticleFilter {
        ParticleFilter::new(
            FilterSettings {
                min_particles: min,
                max_particles: max,
                ..FilterSettings::default()
            },
            42,
        )
        .expect("good settings")
    }

    #[test]
    fn refuses_bad_settings() {
        let bad = |s: FilterSettings| ParticleFilter::new(s, 0).is_err();
        let d = FilterSettings::default();
        assert!(bad(FilterSettings {
            max_particles: 0,
            ..d
        }));
        assert!(bad(FilterSettings {
            min_particles: 3000,
            ..d
        }));
        assert!(bad(FilterSettings { pf_err: 0.0, ..d }));
        assert!(bad(FilterSettings {
            alpha_fast: 2.0,
            ..d
        }));
    }

    #[test]
    fn kld_limit_matches_fox_and_stays_in_bounds() {
        let s = FilterSettings::default();
        assert_eq!(kld_limit(0, &s), 2000);
        assert_eq!(kld_limit(1, &s), 2000);
        // k = 2: (1 / 0.1) * (1 - 1/9 + sqrt(1/9) 0.99)^3 = 10 * 1.2189^3 → 19, raised to 500.
        assert_eq!(kld_limit(2, &s), 500);
        // Many bins: nav2's formula by hand.
        let k = 101.0f64;
        let b = 2.0 / (9.0 * (k - 1.0));
        let n = ((k - 1.0) / 0.1 * (1.0 - b + b.sqrt() * 0.99).powi(3)).ceil() as usize;
        assert_eq!(kld_limit(101, &s), n);
        assert!((500..2000).contains(&n), "{n}");
        assert_eq!(kld_limit(10_000, &s), 2000);
        // It never goes down as bins fill.
        let counts: Vec<usize> = (2..400).map(|k| kld_limit(k, &s)).collect();
        assert!(counts.windows(2).all(|w| w[0] <= w[1]));
    }

    #[test]
    fn a_tight_cloud_resamples_to_the_minimum_a_wide_one_to_more() {
        let mut pf = filter(100, 5000);
        // A single bin: Fox's formula needs k > 1, and nav2 then takes the maximum.
        let one_bin = [[1e-6, 0.0, 0.0], [0.0, 1e-6, 0.0], [0.0, 0.0, 1e-6]];
        pf.start_gaussian(Pose2::new(0.2, 0.2, 0.05), &one_bin);
        pf.resample(None);
        assert_eq!(pf.particles().len(), 5000);
        // A few bins: near the minimum.
        let tight = [[0.04, 0.0, 0.0], [0.0, 0.04, 0.0], [0.0, 0.0, 0.003]];
        pf.start_gaussian(Pose2::new(0.2, 0.2, 0.05), &tight);
        pf.resample(None);
        let few = pf.particles().len();
        assert!((100..400).contains(&few), "{few}");
        let wide = [[25.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 3.0]];
        pf.start_gaussian(Pose2::default(), &wide);
        pf.resample(None);
        let n = pf.particles().len();
        assert!((1000..=5000).contains(&n), "{n}");
        let sum: f64 = pf.particles().iter().map(|p| p.weight).sum();
        assert!((sum - 1.0).abs() < 1e-9);
    }

    #[test]
    fn low_variance_resampling_keeps_the_weights_proportions() {
        let mut pf = filter(1000, 1000);
        // Two particles, one weighing three times the other.
        pf.particles = vec![
            Particle::new(Pose2::new(0.0, 0.0, 0.0), 0.75),
            Particle::new(Pose2::new(10.0, 0.0, 0.0), 0.25),
        ];
        pf.resample(None);
        let near = pf.particles().iter().filter(|p| p.pose.x < 5.0).count();
        // A comb: exactly 750 of 1000, give or take one.
        assert!((749..=751).contains(&near), "{near}");
    }

    #[test]
    fn the_estimate_is_the_heaviest_cluster() {
        let mut pf = filter(10, 100);
        pf.particles = (0..100)
            .map(|i| {
                let x = if i < 70 { 3.0 } else { -20.0 };
                Particle::new(Pose2::new(x, 1.0, 3.1), 0.01)
            })
            .collect();
        let e = pf.estimate().expect("weights");
        assert_eq!(e.clusters, 2);
        assert!((e.pose.x - 3.0).abs() < 1e-9 && (e.weight - 0.7).abs() < 1e-9);
        // The whole set's spread in x: 0.7 * 0.3 * 23².
        assert!((e.covariance[0][0] - 0.21 * 529.0).abs() < 1e-6);
        assert!(e.covariance[2][2].abs() < 1e-9, "every yaw the same");
    }

    #[test]
    fn a_cluster_across_plus_minus_pi_is_one() {
        let mut pf = filter(10, 100);
        pf.particles = (0..100)
            .map(|i| {
                let yaw = if i % 2 == 0 { PI - 0.01 } else { -PI + 0.01 };
                Particle::new(Pose2::new(0.1, 0.1, yaw), 0.01)
            })
            .collect();
        let e = pf.estimate().expect("weights");
        assert_eq!(e.clusters, 1);
        assert!(wrap(e.pose.yaw - PI).abs() < 1e-9, "{}", e.pose.yaw);
    }

    #[test]
    fn a_gaussian_start_has_the_asked_spread() {
        let mut pf = filter(10, 20_000);
        let cov = [[0.25, 0.1, 0.0], [0.1, 0.16, 0.0], [0.0, 0.0, 0.04]];
        pf.start_gaussian(Pose2::new(1.0, 2.0, 0.5), &cov);
        let e = pf.estimate().expect("weights");
        let c = e.covariance;
        assert!((c[0][0] - 0.25).abs() < 0.015 && (c[1][1] - 0.16).abs() < 0.01);
        assert!((c[0][1] - 0.1).abs() < 0.01);
        // Circular variance of a normal with sd 0.2: -2 ln(e^(-0.02)) = 0.04.
        assert!((c[2][2] - 0.04).abs() < 0.005, "{}", c[2][2]);
        let to_6x6 = e.covariance_6x6();
        assert_eq!(
            (to_6x6[0], to_6x6[7], to_6x6[35]),
            (c[0][0], c[1][1], c[2][2])
        );
    }
}

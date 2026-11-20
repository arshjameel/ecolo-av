//! Lidar odometry that knows it is on a car (TODO 216, a challenger to KISS-ICP). No ROS.
//!
//! Kinematic-ICP's idea (T. Guadagnino et al., PRBonn, 2025; MIT, the idea only, rewritten
//! in Rust): KISS-ICP matches each scan to a small map of the last ones, free in all six
//! directions. A car can't slide sideways, so here each step is **one arc**: `rho` m along
//! it and `theta` rad of turn, the car ending at `(rho cos(θ/2), rho sin(θ/2), θ)` (the
//! non-holonomic constraint). And the wheels say how far it went, so their step is a
//! **prior**, weighed against the points: where the geometry says nothing (a long corridor,
//! an open car park), the wheels carry the step; where it does, the points correct them.
//!
//! ```text
//!   scan (base_link) ─► crop, drop the ground, thin ─► match to the map (voxel hash)
//!        wheels' step (rho, θ) ─► prior ──────────────► Gauss-Newton on (rho, θ),
//!                                                        Geman-McClure weights
//!   ─► pose (odom-like frame), twist (rho / dt, 0, θ / dt) ─► the map takes the scan
//! ```
//!
//! | Item | What |
//! |---|---|
//! | [`Motion`] | one step: `rho`, `theta`; [`Motion::pose`] its end |
//! | [`Pose2`] | a flat pose; compose, move a point |
//! | [`VoxelMap`] | the local map: points in voxels, the nearest found in the 27 round |
//! | [`register`] | one scan against the map, from a prior, by Gauss-Newton |
//! | [`Odometry`] | scan after scan: the pose, the twist, the map, the threshold |
// The Rust API Guidelines, kept (TODO 176).
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use std::collections::HashMap;

/// A point (m), in `base_link` or the odometry's frame.
pub type Point = [f64; 3];

/// A flat pose (m, m, rad).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad.
    pub yaw: f64,
}

impl Pose2 {
    /// This pose, then `step` in its frame.
    #[must_use]
    pub fn then(&self, step: &Pose2) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2 {
            x: self.x + c * step.x - s * step.y,
            y: self.y + s * step.x + c * step.y,
            yaw: wrap(self.yaw + step.yaw),
        }
    }

    /// `p` (in this pose's frame) in the outer frame.
    #[must_use]
    pub fn apply(&self, p: &Point) -> Point {
        let (s, c) = self.yaw.sin_cos();
        [
            self.x + c * p[0] - s * p[1],
            self.y + s * p[0] + c * p[1],
            p[2],
        ]
    }
}

/// `a` in -pi..pi.
#[must_use]
pub fn wrap(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// One step of a car that can't slide sideways: an arc `rho` m long (chord), turning
/// `theta` rad.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Motion {
    /// m along (the chord; + forward).
    pub rho: f64,
    /// rad of turn (+ left).
    pub theta: f64,
}

impl Motion {
    /// Where the step ends, in the frame it starts in.
    #[must_use]
    pub fn pose(&self) -> Pose2 {
        let half = self.theta / 2.0;
        Pose2 {
            x: self.rho * half.cos(),
            y: self.rho * half.sin(),
            yaw: self.theta,
        }
    }

    /// From a speed and a turn rate held for `dt` s (the wheels' step).
    #[must_use]
    pub fn from_twist(speed: f64, yaw_rate: f64, dt: f64) -> Self {
        let theta = yaw_rate * dt;
        let arc = speed * dt;
        // The chord of an arc of length `arc` turning `theta`.
        let rho = if theta.abs() < 1e-9 {
            arc
        } else {
            arc * (2.0 * (theta / 2.0).sin() / theta)
        };
        Self { rho, theta }
    }
}

/// The odometry's settings. Metres, radians, seconds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// The map's voxel edge (the scan is thinned at half of it).
    pub voxel_size: f64,
    /// Points kept a voxel.
    pub max_points_per_voxel: usize,
    /// Flat from `base_link`: nearer is the car or noise.
    pub min_range: f64,
    /// Flat from `base_link`: further is too sparse; the map forgets beyond it.
    pub max_range: f64,
    /// Above `base_link`'s ground: lower is the ground, cut out (a flat road's rings look
    /// the same every scan: "no motion"; as KISS-ICP's node here).
    pub min_height: f64,
    /// Gauss-Newton rounds at most.
    pub max_iterations: usize,
    /// Stop when a round moves less than this (m and rad together).
    pub convergence: f64,
    /// The farthest a match may be at the start (m), before the threshold adapts.
    pub initial_threshold: f64,
    /// The threshold never under this (m).
    pub min_threshold: f64,
    /// m: one point's residual, 1 sigma (the points' weight against the wheels').
    pub point_sd: f64,
    /// m a step: the wheels' distance, 1 sigma.
    pub wheel_rho_sd: f64,
    /// rad a step: the wheels' turn, 1 sigma.
    pub wheel_theta_sd: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            voxel_size: 0.5,
            max_points_per_voxel: 20,
            min_range: 1.0,
            max_range: 50.0,
            min_height: 0.3,
            max_iterations: 50,
            convergence: 1e-5,
            initial_threshold: 1.0,
            min_threshold: 0.2,
            point_sd: 0.1,
            wheel_rho_sd: 0.05,
            wheel_theta_sd: 0.02,
        }
    }
}

type Key = [i64; 3];

fn key(p: &Point, size: f64) -> Key {
    [
        (p[0] / size).floor() as i64,
        (p[1] / size).floor() as i64,
        (p[2] / size).floor() as i64,
    ]
}

/// The local map: points in voxels.
#[derive(Clone, Debug, Default)]
pub struct VoxelMap {
    size: f64,
    per_voxel: usize,
    voxels: HashMap<Key, Vec<Point>>,
}

impl VoxelMap {
    /// An empty map of `size` m voxels, `per_voxel` points each at most.
    #[must_use]
    pub fn new(size: f64, per_voxel: usize) -> Self {
        Self {
            size,
            per_voxel: per_voxel.max(1),
            voxels: HashMap::new(),
        }
    }

    /// Points in the map.
    #[must_use]
    pub fn len(&self) -> usize {
        self.voxels.values().map(Vec::len).sum()
    }

    /// No points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.voxels.is_empty()
    }

    /// Add points (the map's frame); a full voxel takes no more.
    pub fn add(&mut self, points: &[Point]) {
        for p in points {
            let v = self.voxels.entry(key(p, self.size)).or_default();
            if v.len() < self.per_voxel {
                v.push(*p);
            }
        }
    }

    /// Forget voxels further than `radius` m (flat) from (x, y).
    pub fn forget_beyond(&mut self, x: f64, y: f64, radius: f64) {
        self.voxels.retain(|_, v| {
            v.first()
                .is_some_and(|p| (p[0] - x).hypot(p[1] - y) <= radius)
        });
    }

    /// The nearest point to `p` in its voxel and the 26 round it, and the squared distance.
    #[must_use]
    pub fn nearest(&self, p: &Point) -> Option<(Point, f64)> {
        let [kx, ky, kz] = key(p, self.size);
        let mut best: Option<(Point, f64)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(v) = self.voxels.get(&[kx + dx, ky + dy, kz + dz]) else {
                        continue;
                    };
                    for q in v {
                        let d =
                            (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2) + (q[2] - p[2]).powi(2);
                        if best.is_none_or(|(_, b)| d < b) {
                            best = Some((*q, d));
                        }
                    }
                }
            }
        }
        best
    }
}

/// One point of each `size` voxel (the first).
#[must_use]
pub fn thin(points: &[Point], size: f64) -> Vec<Point> {
    let mut seen = std::collections::HashSet::new();
    points
        .iter()
        .filter(|p| seen.insert(key(p, size)))
        .copied()
        .collect()
}

/// The points that count, in `base_link`: finite, `min_range`..`max_range` flat, above
/// `min_height`.
#[must_use]
pub fn crop(points: &[Point], settings: &Settings) -> Vec<Point> {
    points
        .iter()
        .filter(|p| {
            let flat = p[0].hypot(p[1]);
            p.iter().all(|v| v.is_finite())
                && (settings.min_range..=settings.max_range).contains(&flat)
                && p[2] >= settings.min_height
        })
        .copied()
        .collect()
}

/// What a registration found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Registration {
    /// The step.
    pub motion: Motion,
    /// Points matched in the last round.
    pub matched: usize,
    /// Rounds run.
    pub iterations: usize,
}

/// Match `scan` (`base_link`, thinned) to `map` with the car at `start` before the step:
/// Gauss-Newton on (rho, theta) from `prior`, the prior weighed in (`Settings::wheel_*_sd`),
/// matches no further than `threshold` m, each weighed by Geman-McClure.
#[must_use]
pub fn register(
    scan: &[Point],
    map: &VoxelMap,
    start: &Pose2,
    prior: Motion,
    threshold: f64,
    settings: &Settings,
) -> Registration {
    let mut m = prior;
    let mut matched = 0;
    let mut iterations = 0;
    let kernel = (threshold / 3.0).powi(2);
    let (info_rho, info_theta) = (
        1.0 / settings.wheel_rho_sd.powi(2),
        1.0 / settings.wheel_theta_sd.powi(2),
    );
    let info_point = 1.0 / settings.point_sd.powi(2);
    let (s0, c0) = start.yaw.sin_cos();
    for _ in 0..settings.max_iterations.max(1) {
        iterations += 1;
        let at = start.then(&m.pose());
        let (half_s, half_c) = (m.theta / 2.0).sin_cos();
        let (st, ct) = m.theta.sin_cos();
        // [H | g] for (rho, theta): the prior first.
        let mut h = [[info_rho, 0.0], [0.0, info_theta]];
        let mut g = [
            info_rho * (m.rho - prior.rho),
            info_theta * (m.theta - prior.theta),
        ];
        matched = 0;
        for s in scan {
            let w = at.apply(s);
            let Some((q, d2)) = map.nearest(&w) else {
                continue;
            };
            if d2 > threshold * threshold {
                continue;
            }
            matched += 1;
            let r = [w[0] - q[0], w[1] - q[1]];
            let r2 = r[0] * r[0] + r[1] * r[1];
            let weight = info_point * kernel * kernel / (kernel + r2).powi(2) / kernel;
            // d(step's point)/d rho and /d theta, in the start's frame, then rotated by it.
            let d_rho = [half_c, half_s];
            let d_theta = [
                -st * s[0] - ct * s[1] - m.rho * half_s / 2.0,
                ct * s[0] - st * s[1] + m.rho * half_c / 2.0,
            ];
            let rot = |v: [f64; 2]| [c0 * v[0] - s0 * v[1], s0 * v[0] + c0 * v[1]];
            let (jr, jt) = (rot(d_rho), rot(d_theta));
            let j = [[jr[0], jt[0]], [jr[1], jt[1]]];
            for (row, ri) in j.iter().zip(r) {
                h[0][0] += weight * row[0] * row[0];
                h[0][1] += weight * row[0] * row[1];
                h[1][1] += weight * row[1] * row[1];
                g[0] += weight * row[0] * ri;
                g[1] += weight * row[1] * ri;
            }
        }
        h[1][0] = h[0][1];
        let det = h[0][0] * h[1][1] - h[0][1] * h[1][0];
        if det.abs() < 1e-18 {
            break;
        }
        let d_rho = -(h[1][1] * g[0] - h[0][1] * g[1]) / det;
        let d_theta = -(h[0][0] * g[1] - h[1][0] * g[0]) / det;
        m.rho += d_rho;
        m.theta += d_theta;
        if d_rho.abs() + d_theta.abs() < settings.convergence {
            break;
        }
    }
    Registration {
        motion: m,
        matched,
        iterations,
    }
}

/// One scan's answer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Step {
    /// The car in the odometry's frame (where the first scan was).
    pub pose: Pose2,
    /// m/s forward (sideways is 0: a car).
    pub speed: f64,
    /// rad/s.
    pub yaw_rate: f64,
    /// Points matched (0 on the first scan).
    pub matched: usize,
    /// The match threshold used (m).
    pub threshold: f64,
}

/// Lidar odometry, scan after scan.
#[derive(Clone, Debug)]
pub struct Odometry {
    settings: Settings,
    map: VoxelMap,
    pose: Pose2,
    last: Option<f64>,
    last_motion: Motion,
    deviations: Vec<f64>,
}

impl Odometry {
    /// Fresh: the first scan starts the map at the origin.
    #[must_use]
    pub fn new(settings: Settings) -> Self {
        Self {
            map: VoxelMap::new(settings.voxel_size, settings.max_points_per_voxel),
            settings,
            pose: Pose2::default(),
            last: None,
            last_motion: Motion::default(),
            deviations: Vec::new(),
        }
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The local map.
    #[must_use]
    pub fn map(&self) -> &VoxelMap {
        &self.map
    }

    /// The match threshold: 3 sigma of how far the answers fell from their priors, within
    /// `min_threshold`..`initial_threshold` (KISS-ICP's adaptive threshold, on our step).
    #[must_use]
    pub fn threshold(&self) -> f64 {
        if self.deviations.is_empty() {
            return self.settings.initial_threshold;
        }
        let mean =
            self.deviations.iter().map(|d| d * d).sum::<f64>() / self.deviations.len() as f64;
        (3.0 * mean.sqrt()).clamp(self.settings.min_threshold, self.settings.initial_threshold)
    }

    /// One scan (`base_link`, uncropped) at `stamp` s; `wheels`: the wheels' (speed, yaw
    /// rate) now, or `None` (then the last step's, held: KISS-ICP's constant velocity).
    pub fn step(&mut self, scan: &[Point], stamp: f64, wheels: Option<(f64, f64)>) -> Step {
        let cropped = crop(scan, &self.settings);
        let source = thin(&cropped, 0.5 * self.settings.voxel_size);
        let threshold = self.threshold();
        let Some(last) = self.last.filter(|_| !self.map.is_empty()) else {
            self.last = Some(stamp);
            self.map.add(&self.world(&source));
            return Step {
                pose: self.pose,
                speed: 0.0,
                yaw_rate: 0.0,
                matched: 0,
                threshold,
            };
        };
        let dt = (stamp - last).max(1e-3);
        let prior = match wheels {
            Some((v, w)) => Motion::from_twist(v, w, dt),
            None => self.last_motion,
        };
        let found = register(
            &source,
            &self.map,
            &self.pose,
            prior,
            threshold,
            &self.settings,
        );
        let m = found.motion;
        let off = m.pose();
        let guess = prior.pose();
        self.deviations
            .push((off.x - guess.x).hypot(off.y - guess.y) + (m.theta - prior.theta).abs());
        if self.deviations.len() > 100 {
            self.deviations.remove(0);
        }
        self.pose = self.pose.then(&off);
        self.last = Some(stamp);
        self.last_motion = m;
        self.map.add(&self.world(&source));
        self.map
            .forget_beyond(self.pose.x, self.pose.y, self.settings.max_range);
        // The arc's length from its chord, over the time.
        let arc = if m.theta.abs() < 1e-9 {
            m.rho
        } else {
            m.rho * (m.theta / 2.0) / (m.theta / 2.0).sin()
        };
        Step {
            pose: self.pose,
            speed: arc / dt,
            yaw_rate: m.theta / dt,
            matched: found.matched,
            threshold,
        }
    }

    fn world(&self, points: &[Point]) -> Vec<Point> {
        points.iter().map(|p| self.pose.apply(p)).collect()
    }
}

#[cfg(test)]
mod tests;

//! All of it, per cloud: the filters, the guess, the match, good or poor, LOST and the
//! restarts, and the covariance. The node only turns messages into calls.
//!
//! | Rule (the design) | Here |
//! |---|---|
//! | the guess: last NDT pose ∘ odom⁻¹ at the last cloud ∘ odom at this cloud | [`predict`] on the [`OdomBuffer`] |
//! | the first pose: a parameter or `/localization/ndt/initial_pose`; on the car `ekf_global` | [`Localizer::set_start`]; `start_from_global` |
//! | lost: 5 poor fits in a row (score per point below a floor, or out of iterations) | [`Tracking::Lost`]; start again from `ekf_global`, counted in `restarts` |
//! | covariance: x, y, yaw block k·(−H⁻¹), with a floor; height, roll, pitch large | [`covariance`] |

use std::time::Duration;

use nalgebra::Matrix3;

use crate::{
    guess::{predict, OdomBuffer},
    matcher::{Match, MatchError, Matcher},
    Mount, Point, Pose2, ScanFilter,
};

/// What the localizer is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tracking {
    /// No first pose yet.
    WaitingForStart,
    /// Following the car.
    Tracking,
    /// Too many poor fits in a row: started again from `ekf_global`, not yet a good fit.
    Lost,
}

impl Tracking {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::WaitingForStart => "WAITING",
            Self::Tracking => "TRACKING",
            Self::Lost => "LOST",
        }
    }
}

/// Why a fit is poor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Poor {
    /// The score per point is below `min_score_per_point`.
    LowScore,
    /// The finest cubes' search ran out of iterations.
    NoConvergence,
}

/// Why a cloud gave no match at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Skip {
    /// No first pose yet.
    NoStart,
    /// The match failed: no points left, or none near the map.
    Failed(MatchError),
}

/// What became of one cloud.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    /// A good fit: publish it, with [`Localizer::covariance`].
    Pose(Match),
    /// A fit, but a poor one: not published.
    Poor { fit: Match, why: Poor },
    /// No fit.
    Skipped(Skip),
}

/// One cloud's outcome and the localizer's state after it, for the status.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Report {
    pub outcome: Outcome,
    pub tracking: Tracking,
    /// The scan's points after the filters.
    pub points: usize,
    pub poor_in_row: usize,
    /// Times it started again from `ekf_global` after getting lost.
    pub restarts: usize,
}

/// The localizer's settings: the node's parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalizerSettings {
    pub filter: ScanFilter,
    /// `base_link → lidar_link`.
    pub mount: Mount,
    /// A fit with a lower score per point is poor.
    pub min_score_per_point: f64,
    /// This many poor fits in a row and it is lost.
    pub lost_after: usize,
    /// With no first pose, take `ekf_global`'s (the car); otherwise wait for one.
    pub start_from_global: bool,
    /// k in k·(−H⁻¹), fitted so Gazebo's errors match the spread.
    pub covariance_scale: f64,
    /// The least variance for x and y (m²) and yaw (rad²).
    pub variance_floor: [f64; 2],
    /// Height, roll and pitch: not measured.
    pub unknown_variance: f64,
    /// The local EKF's buffer, and how far past its newest pose a cloud may be.
    pub odom_window: Duration,
    pub odom_hold: Duration,
}

impl Default for LocalizerSettings {
    fn default() -> Self {
        Self {
            filter: ScanFilter::default(),
            mount: Mount::default(),
            min_score_per_point: 2.0,
            lost_after: 5,
            start_from_global: false,
            covariance_scale: 1.0,
            variance_floor: [0.02 * 0.02, 0.5f64.to_radians().powi(2)],
            unknown_variance: 1e3,
            odom_window: Duration::from_secs(1),
            odom_hold: Duration::from_millis(100),
        }
    }
}

/// The pose the next guess is moved on from: where the car was, and where odom said it
/// was then (`None`: no odometry; the guess is the pose itself).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Anchor {
    pose: Pose2,
    odom: Option<Pose2>,
}

/// NDT on a map, cloud after cloud.
#[derive(Clone, Debug)]
pub struct Localizer {
    matcher: Matcher,
    settings: LocalizerSettings,
    odom: OdomBuffer,
    anchor: Option<Anchor>,
    global: Option<(Duration, Pose2)>,
    tracking: Tracking,
    poor_in_row: usize,
    restarts: usize,
}

impl Localizer {
    pub fn new(matcher: Matcher, settings: LocalizerSettings) -> Self {
        Self {
            matcher,
            odom: OdomBuffer::new(settings.odom_window, settings.odom_hold),
            settings,
            anchor: None,
            global: None,
            tracking: Tracking::WaitingForStart,
            poor_in_row: 0,
            restarts: 0,
        }
    }

    pub fn tracking(&self) -> Tracking {
        self.tracking
    }

    pub fn restarts(&self) -> usize {
        self.restarts
    }

    pub fn matcher(&self) -> &Matcher {
        &self.matcher
    }

    /// A good fit's covariance, for `PoseWithCovariance`: see [`covariance`].
    pub fn covariance(&self, fit: &Match) -> [f64; 36] {
        covariance(&fit.hessian, &self.settings)
    }

    /// The local EKF's pose (`odom → base_link`) at `stamp`.
    pub fn odom(&mut self, stamp: Duration, pose: Pose2) {
        self.odom.push(stamp, pose);
    }

    /// `ekf_global`'s pose (`map → base_link`) at `stamp`: the restart point.
    pub fn global(&mut self, stamp: Duration, pose: Pose2) {
        if pose.is_finite() {
            self.global = Some((stamp, pose));
        }
    }

    /// Start (or start again) from `pose` in the map, at `stamp` if it has one (a
    /// parameter has none: the next cloud is taken to be where it was).
    pub fn set_start(&mut self, pose: Pose2, stamp: Option<Duration>) {
        self.anchor = Some(Anchor {
            pose,
            odom: stamp.and_then(|s| self.odom.at(s)),
        });
        self.tracking = Tracking::Tracking;
        self.poor_in_row = 0;
    }

    /// One lidar cloud, its points in `lidar_link`, stamped `stamp`.
    pub fn cloud(&mut self, stamp: Duration, points: impl IntoIterator<Item = Point>) -> Report {
        let mount = self.settings.mount;
        let scan = self
            .settings
            .filter
            .apply(points.into_iter().map(|p| mount.to_base_link(&p)));
        let outcome = self.match_scan(stamp, &scan);
        Report {
            outcome,
            tracking: self.tracking,
            points: scan.len(),
            poor_in_row: self.poor_in_row,
            restarts: self.restarts,
        }
    }

    fn match_scan(&mut self, stamp: Duration, scan: &[Point]) -> Outcome {
        if self.anchor.is_none() && self.settings.start_from_global {
            if let Some((at, pose)) = self.global {
                self.set_start(pose, Some(at));
            }
        }
        let Some(anchor) = self.anchor else {
            return Outcome::Skipped(Skip::NoStart);
        };
        let odom_now = self.odom.at(stamp);
        let guess = match (anchor.odom, odom_now) {
            (Some(then), Some(now)) => predict(&anchor.pose, &then, &now),
            _ => anchor.pose,
        };
        let fit = match self.matcher.align(scan, guess) {
            Ok(fit) => fit,
            Err(e) => {
                self.poor();
                return Outcome::Skipped(Skip::Failed(e));
            }
        };
        let why = if !fit.converged {
            Some(Poor::NoConvergence)
        } else if fit.score_per_point < self.settings.min_score_per_point {
            Some(Poor::LowScore)
        } else {
            None
        };
        if let Some(why) = why {
            self.poor();
            return Outcome::Poor { fit, why };
        }
        self.anchor = Some(Anchor {
            pose: fit.pose,
            odom: odom_now,
        });
        self.tracking = Tracking::Tracking;
        self.poor_in_row = 0;
        Outcome::Pose(fit)
    }

    /// A poor fit, or none: after `lost_after` in a row, lost, and start again from
    /// `ekf_global` if there is one.
    fn poor(&mut self) {
        self.poor_in_row += 1;
        if self.poor_in_row < self.settings.lost_after {
            return;
        }
        self.tracking = Tracking::Lost;
        self.poor_in_row = 0;
        if let Some((at, pose)) = self.global {
            self.anchor = Some(Anchor {
                pose,
                odom: self.odom.at(at),
            });
            self.restarts += 1;
        }
    }
}

/// The pose's covariance: x, y, yaw from k·(−H⁻¹) with the floors on the diagonal; height,
/// roll and pitch unknown. If −H isn't positive-definite, x, y and yaw are unknown too.
pub fn covariance(hessian: &Matrix3<f64>, settings: &LocalizerSettings) -> [f64; 36] {
    let unknown = settings.unknown_variance;
    let mut out = [0.0; 36];
    // x, y, z, roll, pitch, yaw: where x, y and yaw sit in the 6×6.
    const AT: [usize; 3] = [0, 1, 5];
    let block = (-hessian)
        .cholesky()
        .map(|c| c.inverse() * settings.covariance_scale)
        .filter(|b| b.iter().all(|v| v.is_finite()));
    for i in 0..6 {
        out[i * 7] = unknown;
    }
    if let Some(b) = block {
        for (r, &row) in AT.iter().enumerate() {
            for (c, &col) in AT.iter().enumerate() {
                out[row * 6 + col] = b[(r, c)];
            }
        }
        let [floor_m, floor_rad] = settings.variance_floor;
        for (&i, floor) in AT.iter().zip([floor_m, floor_m, floor_rad]) {
            out[i * 7] = out[i * 7].max(floor);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sharp_peak_hits_the_floor_and_a_flat_one_is_unknown() {
        let settings = LocalizerSettings::default();
        let sharp = covariance(&(-1e9 * Matrix3::identity()), &settings);
        assert_eq!(sharp[0], settings.variance_floor[0]);
        assert_eq!(sharp[35], settings.variance_floor[1]);
        assert_eq!(sharp[14], settings.unknown_variance);
        let wide = covariance(&(-Matrix3::identity()), &settings);
        assert!((wide[7] - 1.0).abs() < 1e-12);
        let saddle = covariance(&Matrix3::identity(), &settings);
        assert_eq!(saddle[0], settings.unknown_variance);
        assert_eq!(saddle[1], 0.0);
    }
}

//! AMCL per scan, as nav2_amcl's `laserReceived` runs it: the odometry at the scan's
//! stamp; an update only once the car moved `update_min_d` or turned `update_min_a`
//! (each axis on its own, as nav2); the motion model, then the scan model; a resample
//! every `resample_interval` updates; a pose after each resample, after a start, and
//! until the first is out.

use std::{fmt, time::Duration};

use crate::{
    filter::{Estimate, FilterSettings, Particle, ParticleFilter},
    map::FreeSpace,
    motion::DifferentialMotion,
    odom::OdomBuffer,
    pose::{wrap, Metres, Pose2, Radians},
    scan::{Scan, ScanModel},
};

/// A setting that makes no sense, and why.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettingsError {
    /// The parameter's name (nav2_amcl's).
    pub name: String,
    /// What is wrong with it.
    pub why: String,
}

impl SettingsError {
    /// `name` is wrong: `why`.
    #[must_use]
    pub fn new(name: impl Into<String>, why: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            why: why.into(),
        }
    }
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.name, self.why)
    }
}

impl std::error::Error for SettingsError {}

/// AMCL's settings: nav2_amcl's names, its defaults.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AmclSettings {
    /// The particles' count, KLD and recovery.
    pub filter: FilterSettings,
    /// The odometry's noise (alpha1-4).
    pub motion: DifferentialMotion,
    /// Driven before an update (update_min_d).
    pub update_min_d: Metres,
    /// Turned before an update (update_min_a).
    pub update_min_a: Radians,
    /// Updates per resample (resample_interval), at least 1.
    pub resample_interval: usize,
    /// How much older than a scan the newest odometry may be (transform_tolerance).
    pub odom_tolerance: Duration,
    /// The random numbers' seed: the same seed, the same run.
    pub seed: u64,
}

impl Default for AmclSettings {
    fn default() -> Self {
        Self {
            filter: FilterSettings::default(),
            motion: DifferentialMotion::default(),
            update_min_d: Metres(0.25),
            update_min_a: Radians(0.2),
            resample_interval: 1,
            odom_tolerance: Duration::from_secs(1),
            seed: 0,
        }
    }
}

/// What one scan did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// No start pose yet: nothing to update.
    NoStart,
    /// No odometry at the scan's stamp.
    NoOdometry,
    /// Moved too little since the last update: nothing changed.
    Still,
    /// The particles moved and were weighed (publish them).
    Updated {
        /// The pose to publish, when there is one to publish now.
        pose: Option<Estimate>,
        /// The particles were resampled.
        resampled: bool,
    },
}

/// AMCL: the filter, its models, the odometry, and when to do what.
#[derive(Clone, Debug)]
pub struct Amcl<M> {
    settings: AmclSettings,
    model: M,
    free: FreeSpace,
    filter: ParticleFilter,
    odom: OdomBuffer,
    started: bool,
    /// The odometry at the last update; None: the next scan updates without a move.
    reference: Option<Pose2>,
    /// Updates since the last resample.
    resample_count: usize,
    first_pose_sent: bool,
}

impl<M: ScanModel> Amcl<M> {
    /// AMCL with `settings`, weighing by `model`, putting random particles in `free`.
    /// It waits for [`Amcl::set_pose`] or [`Amcl::start_everywhere`].
    ///
    /// # Errors
    /// [`SettingsError`] for bad filter settings or a resample interval of 0.
    pub fn new(settings: AmclSettings, model: M, free: FreeSpace) -> Result<Self, SettingsError> {
        if settings.resample_interval == 0 {
            return Err(SettingsError::new("resample_interval", "must be above 0"));
        }
        let keep = settings.odom_tolerance.max(Duration::from_secs(2)) * 2;
        Ok(Self {
            filter: ParticleFilter::new(settings.filter, settings.seed)?,
            settings,
            model,
            free,
            odom: OdomBuffer::new(keep),
            started: false,
            reference: None,
            resample_count: 0,
            first_pose_sent: false,
        })
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> &AmclSettings {
        &self.settings
    }

    /// The scan model.
    #[must_use]
    pub fn model(&self) -> &M {
        &self.model
    }

    /// The particle filter.
    #[must_use]
    pub fn filter(&self) -> &ParticleFilter {
        &self.filter
    }

    /// The particles now.
    #[must_use]
    pub fn particles(&self) -> &[Particle] {
        self.filter.particles()
    }

    /// A start pose is known: the particles mean something.
    #[must_use]
    pub fn started(&self) -> bool {
        self.started
    }

    /// A new map: its scan model and free space. The particles stay where they are.
    pub fn set_map(&mut self, model: M, free: FreeSpace) {
        self.model = model;
        self.free = free;
    }

    /// Adds the odometry's pose (`odom -> base_link`) at `stamp`.
    pub fn odom(&mut self, stamp: Duration, pose: Pose2) {
        self.odom.push(stamp, pose);
    }

    /// Starts again round `pose` (map frame) with `covariance` (x, y, yaw). Given the
    /// pose's `stamp`, the odometry's move since then is added, as nav2 does.
    pub fn set_pose(&mut self, stamp: Option<Duration>, pose: Pose2, covariance: &[[f64; 3]; 3]) {
        let moved = stamp.and_then(|t| {
            let then = self.odom.at(t, self.settings.odom_tolerance)?;
            let (_, now) = self.odom.latest()?;
            Some(then.between(&now))
        });
        let pose = moved.map_or(pose, |m| pose.compose(&m));
        self.filter.start_gaussian(pose, covariance);
        self.restart();
    }

    /// Starts again with particles anywhere free (global localization). False, with
    /// nothing changed, when the map has no free cell.
    pub fn start_everywhere(&mut self) -> bool {
        let done = self.filter.start_uniform(&self.free);
        if done {
            self.restart();
        }
        done
    }

    fn restart(&mut self) {
        self.started = true;
        self.reference = None;
        self.resample_count = 0;
        self.first_pose_sent = false;
    }

    /// One scan, taken at `stamp`.
    pub fn scan(&mut self, stamp: Duration, scan: &Scan) -> Step {
        if !self.started {
            return Step::NoStart;
        }
        let Some(odom) = self.odom.at(stamp, self.settings.odom_tolerance) else {
            return Step::NoOdometry;
        };
        let force = match self.reference {
            None => true,
            Some(from) => {
                let (d, a) = (self.settings.update_min_d.0, self.settings.update_min_a.0);
                let moved = (odom.x - from.x).abs() > d
                    || (odom.y - from.y).abs() > d
                    || wrap(odom.yaw - from.yaw).abs() > a;
                if !moved {
                    return Step::Still;
                }
                self.filter.predict(&self.settings.motion, &from, &odom);
                false
            }
        };
        self.filter.weigh(&self.model, scan);
        self.reference = Some(odom);
        self.resample_count += 1;
        let resampled = self.resample_count >= self.settings.resample_interval;
        if resampled {
            self.resample_count = 0;
            self.filter.resample(Some(&self.free));
        }
        let pose = if resampled || force || !self.first_pose_sent {
            self.filter.estimate()
        } else {
            None
        };
        self.first_pose_sent |= pose.is_some();
        Step::Updated { pose, resampled }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        map::{Cell, DistanceField, OccupancyMap},
        scan::LikelihoodField,
    };

    fn amcl(interval: usize) -> Amcl<LikelihoodField> {
        let map = OccupancyMap::new(
            20,
            20,
            Metres(0.5),
            Pose2::new(-5.0, -5.0, 0.0),
            vec![Cell::Free; 400],
        )
        .expect("map");
        let model = LikelihoodField::new(DistanceField::new(&map, Metres(2.0)), 0.9, 0.1, 0.2, 60);
        let settings = AmclSettings {
            resample_interval: interval,
            ..AmclSettings::default()
        };
        Amcl::new(settings, model, FreeSpace::new(&map)).expect("settings")
    }

    fn empty_scan() -> Scan {
        Scan::from_ranges(&[], 0.0, 0.0, (0.1, 30.0), (-1.0, -1.0), Pose2::default())
    }

    #[test]
    fn waits_for_a_start_and_for_odometry() {
        let mut a = amcl(1);
        let t = Duration::from_secs(10);
        assert_eq!(a.scan(t, &empty_scan()), Step::NoStart);
        a.set_pose(
            None,
            Pose2::default(),
            &[[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
        );
        assert_eq!(a.scan(t, &empty_scan()), Step::NoOdometry);
    }

    #[test]
    fn updates_only_after_a_move_and_resamples_every_interval() {
        let mut a = amcl(2);
        let cov = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]];
        a.set_pose(None, Pose2::default(), &cov);
        let at = |s: f64| Duration::from_secs_f64(s);
        a.odom(at(1.0), Pose2::default());
        // The first scan: an update without a move, and a pose.
        let first = a.scan(at(1.0), &empty_scan());
        assert!(matches!(
            first,
            Step::Updated {
                pose: Some(_),
                resampled: false
            }
        ));
        // 0.1 m: still.
        a.odom(at(2.0), Pose2::new(0.1, 0.0, 0.0));
        assert_eq!(a.scan(at(2.0), &empty_scan()), Step::Still);
        // 0.3 m: an update, the second, so a resample and a pose.
        a.odom(at(3.0), Pose2::new(0.3, 0.0, 0.0));
        assert!(matches!(
            a.scan(at(3.0), &empty_scan()),
            Step::Updated {
                pose: Some(_),
                resampled: true
            }
        ));
        // A turn alone: an update; the third, so no resample, and no pose.
        a.odom(at(4.0), Pose2::new(0.3, 0.0, 0.3));
        assert!(matches!(
            a.scan(at(4.0), &empty_scan()),
            Step::Updated {
                pose: None,
                resampled: false
            }
        ));
    }

    #[test]
    fn a_stamped_start_pose_adds_the_move_since() {
        let mut a = amcl(1);
        a.odom(Duration::from_secs(1), Pose2::new(0.0, 0.0, 0.0));
        a.odom(Duration::from_secs(2), Pose2::new(2.0, 0.0, 0.0));
        let tiny = [[1e-8, 0.0, 0.0], [0.0, 1e-8, 0.0], [0.0, 0.0, 1e-8]];
        // Placed facing +y at the time the odometry was at x = 0: it has since driven 2 m.
        a.set_pose(
            Some(Duration::from_secs(1)),
            Pose2::new(1.0, 1.0, std::f64::consts::FRAC_PI_2),
            &tiny,
        );
        let e = a.filter().estimate().expect("weights");
        assert!(
            (e.pose.x - 1.0).abs() < 1e-3 && (e.pose.y - 3.0).abs() < 1e-3,
            "{e:?}"
        );
    }

    #[test]
    fn refuses_a_zero_interval() {
        let a = amcl(1);
        let bad = AmclSettings {
            resample_interval: 0,
            ..AmclSettings::default()
        };
        assert!(Amcl::new(bad, a.model.clone(), a.free.clone()).is_err());
    }
}

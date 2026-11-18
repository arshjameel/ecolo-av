//! The scan, and how well it fits the map from a pose: the [`ScanModel`] trait and its
//! first model, nav2_amcl's likelihood field.
//!
//! The trait is the swap point (CLAUDE.md, "everything is a drop-in replacement"): a
//! learned scan model can weigh the particles instead, and the filter never knows.

use crate::{map::DistanceField, pose::Pose2};

/// One beam: how far it reached and which way it pointed from the sensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Beam {
    /// m. At or above the scan's `range_max`, or NaN: nothing seen (skipped).
    pub range: f64,
    /// rad from the sensor's x, + left.
    pub bearing: f64,
}

/// A laser scan, ready to weigh particles with.
#[derive(Clone, Debug, PartialEq)]
pub struct Scan {
    /// Every beam, in order.
    pub beams: Vec<Beam>,
    /// m: a beam this long or longer saw nothing.
    pub range_max: f64,
    /// Where the sensor is on the car (`base_link`): identity for our flattened scan.
    pub sensor: Pose2,
}

impl Scan {
    /// A scan from a `LaserScan`'s ranges, as nav2_amcl reads it: `range_max` is the
    /// smaller of the scan's and `max_range` (when above 0), and a range at or under the
    /// larger of the scan's `range_min` and `min_range` counts as nothing seen.
    #[must_use]
    pub fn from_ranges(
        ranges: &[f32],
        angle_min: f64,
        angle_increment: f64,
        (range_min, range_max): (f64, f64),
        (min_range, max_range): (f64, f64),
        sensor: Pose2,
    ) -> Self {
        let range_max = if max_range > 0.0 {
            range_max.min(max_range)
        } else {
            range_max
        };
        let range_min = if min_range > 0.0 {
            range_min.max(min_range)
        } else {
            range_min
        };
        let beams = ranges
            .iter()
            .enumerate()
            .map(|(i, &r)| {
                let r = f64::from(r);
                Beam {
                    range: if r <= range_min { range_max } else { r },
                    bearing: angle_min + i as f64 * angle_increment,
                }
            })
            .collect();
        Self {
            beams,
            range_max,
            sensor,
        }
    }
}

/// How well a scan fits the map from a pose: higher is better. Only the ratios between
/// particles matter; the filter normalises the weights. A model must give a finite value
/// at or above 0.
pub trait ScanModel {
    /// The likelihood of `scan` with the car (`base_link`) at `pose` on the map.
    fn likelihood(&self, pose: &Pose2, scan: &Scan) -> f64;

    /// The likelihood of `scan` from each of `poses`. A model that is faster in a batch
    /// (a network on a GPU) overrides this.
    fn likelihoods(&self, poses: &[Pose2], scan: &Scan) -> Vec<f64> {
        poses
            .iter()
            .map(|pose| self.likelihood(pose, scan))
            .collect()
    }
}

/// nav2_amcl's likelihood field: each beam's end is scored by its distance to the nearest
/// occupied cell (a bell curve, `z_hit`, plus a floor for random readings, `z_rand`);
/// nav2's ad hoc sum `1 + Σ p³` combines the beams. No ray casting, and a beam that saw
/// nothing is skipped.
#[derive(Clone, Debug)]
pub struct LikelihoodField {
    field: DistanceField,
    /// The bell curve's weight.
    pub z_hit: f64,
    /// The random readings' weight.
    pub z_rand: f64,
    /// The bell curve's standard deviation, m.
    pub sigma_hit: f64,
    /// At most this many beams, evenly spaced, are used (at least 2).
    pub max_beams: usize,
}

impl LikelihoodField {
    /// The model on `field` (the map's distances, capped at laser_likelihood_max_dist).
    #[must_use]
    pub fn new(
        field: DistanceField,
        z_hit: f64,
        z_rand: f64,
        sigma_hit: f64,
        max_beams: usize,
    ) -> Self {
        Self {
            field,
            z_hit,
            z_rand,
            sigma_hit,
            max_beams,
        }
    }

    /// The distance field it reads.
    #[must_use]
    pub fn field(&self) -> &DistanceField {
        &self.field
    }

    /// Every how many beams one is used: nav2's `(n - 1) / (max_beams - 1)`, at least 1.
    #[must_use]
    pub fn step(&self, beams: usize) -> usize {
        (beams.saturating_sub(1) / self.max_beams.saturating_sub(1).max(1)).max(1)
    }
}

impl ScanModel for LikelihoodField {
    fn likelihood(&self, pose: &Pose2, scan: &Scan) -> f64 {
        let denom = 2.0 * self.sigma_hit * self.sigma_hit;
        let rand = self.z_rand / scan.range_max;
        let sensor = pose.compose(&scan.sensor);
        let (s, c) = sensor.yaw.sin_cos();
        1.0 + scan
            .beams
            .iter()
            .step_by(self.step(scan.beams.len()))
            .filter(|b| b.range < scan.range_max)
            .map(|b| {
                // The beam's end: the bearing turned by the sensor's heading.
                let (bs, bc) = b.bearing.sin_cos();
                let x = sensor.x + b.range * (c * bc - s * bs);
                let y = sensor.y + b.range * (s * bc + c * bs);
                let z = self.field.distance(x, y);
                let pz = self.z_hit * (-(z * z) / denom).exp() + rand;
                pz * pz * pz
            })
            .sum::<f64>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        map::{Cell, OccupancyMap},
        pose::Metres,
    };

    /// A wall along x = 2 m.
    fn wall() -> LikelihoodField {
        let n = 40;
        let cells = (0..n * n)
            .map(|k| {
                if k % n == 30 {
                    Cell::Occupied
                } else {
                    Cell::Free
                }
            })
            .collect();
        let map =
            OccupancyMap::new(n, n, Metres(0.1), Pose2::new(-1.0, -2.0, 0.0), cells).expect("map");
        LikelihoodField::new(DistanceField::new(&map, Metres(2.0)), 0.9, 0.1, 0.2, 60)
    }

    fn ahead(range: f32) -> Scan {
        Scan::from_ranges(
            &[range],
            0.0,
            0.0,
            (0.1, 30.0),
            (-1.0, -1.0),
            Pose2::default(),
        )
    }

    #[test]
    fn a_beam_on_the_wall_scores_best() {
        let model = wall();
        let scan = ahead(2.05); // the wall's cell centre is at x = 2.05
        let right = model.likelihood(&Pose2::default(), &scan);
        let short = model.likelihood(&Pose2::new(-0.5, 0.0, 0.0), &scan);
        let turned = model.likelihood(&Pose2::new(0.0, 0.0, 1.0), &scan);
        assert!(right > short && right > turned, "{right} {short} {turned}");
        // On the wall: 1 + (z_hit + z_rand / range_max)³.
        let best = 1.0 + (0.9 + 0.1 / 30.0f64).powi(3);
        assert!((right - best).abs() < 1e-9, "{right} vs {best}");
    }

    #[test]
    fn beams_that_saw_nothing_are_skipped() {
        let model = wall();
        for r in [f32::INFINITY, f32::NAN, 30.0, 0.05] {
            assert_eq!(model.likelihood(&Pose2::default(), &ahead(r)), 1.0, "{r}");
        }
    }

    #[test]
    fn the_range_limits_are_nav2s() {
        let scan = Scan::from_ranges(
            &[0.3, 5.0],
            0.0,
            0.1,
            (0.1, 30.0),
            (0.5, 4.0),
            Pose2::default(),
        );
        assert_eq!(scan.range_max, 4.0);
        assert_eq!(scan.beams[0].range, 4.0, "under min_range: nothing seen");
        assert_eq!(scan.beams[1].range, 5.0);
        assert!((scan.beams[1].bearing - 0.1).abs() < 1e-12);
    }

    #[test]
    fn the_step_is_nav2s() {
        let mut model = wall();
        model.max_beams = 180;
        assert_eq!(model.step(720), 4);
        model.max_beams = 60;
        assert_eq!(model.step(720), 12);
        assert_eq!(model.step(10), 1);
    }
}

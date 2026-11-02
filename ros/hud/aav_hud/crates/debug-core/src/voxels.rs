//! Nav2's cubes: what the voxels trial (aav_voxels, `voxels:=true`) tells the costmap
//! and collision_monitor to stop for. The screen's own cubes are its own reading of the
//! lidar; these are Nav2's, drawn over them as red wireframes (key V).
//!
//! `/perception/voxels/cubes` is a PointCloud2 in base_link, stamped with the lidar
//! cloud's stamp: one point per cube centre (0.25 m cubes by default). Each cloud is
//! placed as the lidar's are: base_link -> odom with the local EKF's pose **at its
//! stamp** ([`crate::poses`]). The centres are moved, not snapped again: they are
//! already one per cube.
//!
//! `/perception/voxels/status` is a diagnostic_msgs/DiagnosticStatus: level OK (the
//! road's slope was fitted) or WARN (taken as flat); values `ms`, `cubes`,
//! `ground_cubes`, `points`, `fitted`, `roll_deg`, `pitch_deg`, `clouds_dropped`
//! (aav_voxels' README).

use crate::{
    cloud::{self, CloudView},
    cubes::{keep_nearest_by, Band, Cube, CubeScan, Metres},
    frames::{Stamp, StaticFrames, Transform3},
    poses::PoseBuffer,
    scan::ScanError,
};
use map_core::geom::Pose;

/// What Nav2's cubes need from the settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VoxelSettings {
    /// One cube's edge (aav_voxels' `size`).
    pub size: Metres,
    /// The most kept; the nearest the car stay.
    pub max_cubes: usize,
}

/// Nav2's cubes from one cloud, in odom.
///
/// `points` are the cube centres in the cloud's frame; `cloud_to_base` takes them to
/// base_link; `car` is base_link's pose in odom at the cloud's stamp. Over
/// `settings.max_cubes`, the nearest stay. Each cube's band is its height above base_link.
pub fn build(
    points: impl IntoIterator<Item = [f32; 3]>,
    cloud_to_base: &Transform3,
    car: Pose,
    stamp: Stamp,
    settings: VoxelSettings,
) -> CubeScan {
    let to_odom = Transform3::from_planar(car).then_after(cloud_to_base);
    let (mut used, mut skipped) = (0, 0);
    let mut cubes: Vec<Cube> = points
        .into_iter()
        .filter_map(|p| {
            if !p.iter().all(|v| v.is_finite()) {
                skipped += 1;
                return None;
            }
            used += 1;
            let p = p.map(f64::from);
            let height = cloud_to_base.apply(p)[2];
            Some(Cube {
                centre: to_odom.apply(p).map(|v| v as f32),
                band: Band::of(Metres(height), Metres(0.0)),
            })
        })
        .collect();
    let cut = keep_nearest_by(&mut cubes, |c| c.centre, car.position, settings.max_cubes);
    CubeScan {
        stamp,
        car,
        cubes,
        size: settings.size.0 as f32,
        points: used,
        skipped,
        cut,
    }
}

/// One `/perception/voxels/cubes` cloud, start to finish: its frame (base_link, or one
/// `/tf_static` knows), the pose at its stamp, its points.
pub fn place(
    view: &CloudView<'_>,
    frame: &str,
    stamp: Stamp,
    base: &str,
    statics: &StaticFrames,
    poses: &PoseBuffer,
    settings: VoxelSettings,
) -> Result<CubeScan, ScanError> {
    let to_base = statics.lookup(base, frame)?;
    let car = poses.at(stamp)?;
    let points = cloud::positions(view)?;
    Ok(build(points, &to_base, car, stamp, settings))
}

/// How the voxels node took the road under the car.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Road {
    /// Level OK: the road's tilt was fitted to this cloud.
    Fitted,
    /// Level WARN (or worse): no fit it believed, so the road was taken as flat.
    Flat,
}

impl Road {
    pub fn from_level(level: u8) -> Self {
        if level == 0 {
            Self::Fitted
        } else {
            Self::Flat
        }
    }

    /// The panel's word.
    pub fn word(self) -> &'static str {
        match self {
            Self::Fitted => "OK",
            Self::Flat => "WARN",
        }
    }
}

/// What the panel shows of `/perception/voxels/status`.
#[derive(Debug, Clone, PartialEq)]
pub struct VoxelStatus {
    pub road: Road,
    /// The node's own words: "road fitted", or why it took the road as flat.
    pub message: String,
    /// One cloud's work, wall clock, milliseconds.
    pub ms: Option<f64>,
    /// Cubes Nav2 is told to stop for.
    pub cubes: Option<u64>,
    /// Flat squares of road seen (they clear the costmap).
    pub ground_cubes: Option<u64>,
    pub points: Option<u64>,
    pub roll_deg: Option<f64>,
    pub pitch_deg: Option<f64>,
    /// Clouds that came while one was being worked on.
    pub clouds_dropped: Option<u64>,
}

impl VoxelStatus {
    /// From the message's level, its `message` and its key-value pairs. Unreadable
    /// values are left out.
    pub fn from_message<'a>(
        level: u8,
        message: &str,
        values: impl IntoIterator<Item = (&'a str, &'a str)>,
    ) -> Self {
        let mut s = Self {
            road: Road::from_level(level),
            message: message.trim().to_owned(),
            ms: None,
            cubes: None,
            ground_cubes: None,
            points: None,
            roll_deg: None,
            pitch_deg: None,
            clouds_dropped: None,
        };
        let number = |v: &str| v.trim().parse::<f64>().ok().filter(|x| x.is_finite());
        let count = |v: &str| v.trim().parse::<u64>().ok();
        for (key, value) in values {
            match key {
                "ms" => s.ms = number(value),
                "cubes" => s.cubes = count(value),
                "ground_cubes" => s.ground_cubes = count(value),
                "points" => s.points = count(value),
                "roll_deg" => s.roll_deg = number(value),
                "pitch_deg" => s.pitch_deg = number(value),
                "clouds_dropped" => s.clouds_dropped = count(value),
                _ => {}
            }
        }
        s
    }

    /// The big numbers, in one short line: "412 cubes   3.2 ms".
    pub fn numbers(&self) -> String {
        let mut parts = Vec::new();
        if let Some(n) = self.cubes {
            parts.push(format!("{n} cubes"));
        }
        if let Some(ms) = self.ms {
            parts.push(format!("{ms:.1} ms"));
        }
        parts.join("   ")
    }

    /// The road's fit and the smaller numbers.
    pub fn details(&self) -> String {
        let mut parts = vec![self.message.clone()];
        if let (Some(r), Some(p)) = (self.roll_deg, self.pitch_deg) {
            parts.push(format!("roll {r:+.1} deg, pitch {p:+.1} deg"));
        }
        if let Some(g) = self.ground_cubes {
            parts.push(format!("{g} road squares"));
        }
        if let Some(d) = self.clouds_dropped.filter(|&d| d > 0) {
            parts.push(format!("{d} clouds dropped"));
        }
        parts.retain(|p| !p.is_empty());
        parts.join(", ")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{f64::consts::FRAC_PI_2, time::Duration};

    fn settings(max_cubes: usize) -> VoxelSettings {
        VoxelSettings {
            size: Metres(0.25),
            max_cubes,
        }
    }

    #[test]
    fn centres_move_into_odom_without_snapping() {
        // The car at (10, 0), turned a quarter left: 2 m ahead is (10, 2).
        let car = Pose::new(10.0, 0.0, FRAC_PI_2);
        let scan = build(
            [[2.0, 0.0, 0.875], [f32::NAN, 0.0, 0.0]],
            &Transform3::IDENTITY,
            car,
            Stamp(5),
            settings(100),
        );
        assert_eq!((scan.cubes.len(), scan.skipped, scan.points), (1, 1, 1));
        let [x, y, z] = scan.cubes[0].centre;
        assert!((x - 10.0).abs() < 1e-5 && (y - 2.0).abs() < 1e-5, "{x} {y}");
        assert!((z - 0.875).abs() < 1e-6);
        assert_eq!(scan.cubes[0].band, Band::Waist);
        assert_eq!((scan.size, scan.stamp), (0.25, Stamp(5)));
    }

    #[test]
    fn over_the_cap_the_nearest_stay() {
        let points = (1..=10).map(|i| [i as f32, 0.0, 1.0]);
        let scan = build(
            points,
            &Transform3::IDENTITY,
            Pose::default(),
            Stamp(0),
            settings(3),
        );
        assert_eq!((scan.cubes.len(), scan.cut), (3, 7));
        assert!(scan.cubes.iter().all(|c| c.centre[0] <= 3.0));
    }

    #[test]
    fn a_cloud_is_placed_with_the_pose_at_its_stamp() {
        use crate::cloud::Field;
        let data: Vec<u8> = [1.0_f32, 0.0, 1.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let fields = ["x", "y", "z"]
            .iter()
            .zip(0..)
            .map(|(&name, i)| Field {
                name,
                offset: 4 * i,
                datatype: 7,
                count: 1,
            })
            .collect();
        let view = CloudView {
            fields,
            width: 1,
            height: 1,
            point_step: 12,
            row_step: 12,
            is_bigendian: false,
            data: &data,
        };
        let mut poses = PoseBuffer::new(Duration::from_secs(1));
        poses.push(Stamp(0), Pose::new(0.0, 0.0, 0.0));
        poses.push(Stamp(100_000_000), Pose::new(1.0, 0.0, 0.0));
        let statics = StaticFrames::default();
        let at = |stamp, frame| {
            place(
                &view,
                frame,
                Stamp(stamp),
                "base_link",
                &statics,
                &poses,
                settings(100),
            )
        };
        // In base_link already: no /tf_static needed. Halfway, the car is at x = 0.5.
        let scan = at(50_000_000, "base_link").expect("placed");
        assert!((scan.cubes[0].centre[0] - 1.5).abs() < 1e-5);
        assert!(at(200_000_000, "base_link")
            .expect_err("no pose yet")
            .is_too_new());
        assert!(matches!(at(0, "lidar_link"), Err(ScanError::Frame(_))));
    }

    #[test]
    fn the_status_reads_in_plain_words() {
        let s = VoxelStatus::from_message(
            0,
            "road fitted",
            [
                ("ms", "3.21"),
                ("cubes", "412"),
                ("ground_cubes", "900"),
                ("points", "28800"),
                ("fitted", "true"),
                ("roll_deg", "0.40"),
                ("pitch_deg", "-1.20"),
                ("clouds_dropped", "0"),
            ],
        );
        assert_eq!(s.road, Road::Fitted);
        assert_eq!(s.numbers(), "412 cubes   3.2 ms");
        assert_eq!(
            s.details(),
            "road fitted, roll +0.4 deg, pitch -1.2 deg, 900 road squares"
        );
        let flat = VoxelStatus::from_message(1, "road taken as flat (no good fit)", [("ms", "x")]);
        assert_eq!((flat.road.word(), flat.ms), ("WARN", None));
    }
}

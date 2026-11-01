//! A cloud, start to finish: its points, its frame and its stamp in; one scan's cubes
//! out, or why not. The ROS side calls this off the drawing thread.

use thiserror::Error;

use crate::{
    cloud::{self, CloudError, CloudView},
    cubes::{self, CubeScan, CubeSettings},
    frames::{FrameError, Stamp, StaticFrames},
    poses::{Lookup, PoseBuffer},
};

/// Why a cloud became no cubes.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ScanError {
    #[error("unreadable cloud: {0}")]
    Cloud(#[from] CloudError),
    #[error("{0}")]
    Frame(#[from] FrameError),
    #[error("no pose at the cloud's stamp: {0}")]
    Pose(#[from] Lookup),
}

impl ScanError {
    /// Worth trying again shortly: the pose at the cloud's stamp hasn't come yet.
    pub fn is_too_new(&self) -> bool {
        matches!(self, Self::Pose(Lookup::TooNew(_)))
    }
}

/// Places one cloud: `frame` is its frame_id, `base` the car's (base_link).
pub fn place(
    view: &CloudView<'_>,
    frame: &str,
    stamp: Stamp,
    base: &str,
    statics: &StaticFrames,
    poses: &PoseBuffer,
    settings: &CubeSettings,
) -> Result<CubeScan, ScanError> {
    let to_base = statics.lookup(base, frame)?;
    let car = poses.at(stamp)?;
    let points = cloud::positions(view)?;
    Ok(cubes::build(points, &to_base, car, stamp, settings))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{cloud::Field, cubes::Metres, frames::Transform3};
    use map_core::geom::Pose;
    use std::time::Duration;

    fn xyz() -> Vec<Field<'static>> {
        ["x", "y", "z"]
            .iter()
            .zip(0..)
            .map(|(&name, i)| Field {
                name,
                offset: 4 * i,
                datatype: 7,
                count: 1,
            })
            .collect()
    }

    #[test]
    fn a_cloud_is_placed_with_the_pose_at_its_stamp() {
        let data: Vec<u8> = [2.0_f32, 0.0, 0.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let view = CloudView {
            fields: xyz(),
            width: 1,
            height: 1,
            point_step: 12,
            row_step: 12,
            is_bigendian: false,
            data: &data,
        };
        let mut statics = StaticFrames::default();
        statics.insert(
            "base_link",
            "lidar_link",
            Transform3 {
                translation: [0.0, 0.0, 1.0],
                ..Transform3::IDENTITY
            },
        );
        let mut poses = PoseBuffer::new(Duration::from_secs(1));
        poses.push(Stamp(0), Pose::new(0.0, 0.0, 0.0));
        poses.push(Stamp(100_000_000), Pose::new(1.0, 0.0, 0.0));
        let settings = CubeSettings {
            size: Metres(0.2),
            max_cubes: 10,
            ground_height: Metres(0.25),
        };
        let at = |stamp| {
            place(
                &view,
                "lidar_link",
                Stamp(stamp),
                "base_link",
                &statics,
                &poses,
                &settings,
            )
        };
        // Halfway: the car at x = 0.5, the point 2 m ahead of it.
        let scan = at(50_000_000).expect("placed");
        assert!((scan.cubes[0].centre[0] - 2.5).abs() < 0.11);
        assert!((scan.car.position.x - 0.5).abs() < 1e-9);
        let late = at(200_000_000).expect_err("no pose yet");
        assert!(late.is_too_new());
        let other = place(
            &view,
            "camera",
            Stamp(0),
            "base_link",
            &statics,
            &poses,
            &settings,
        );
        assert!(matches!(other, Err(ScanError::Frame(_))));
    }
}

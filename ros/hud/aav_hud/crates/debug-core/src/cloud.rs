//! The lidar's points from a `sensor_msgs/PointCloud2`'s bytes.
//!
//! The reading itself is `aav_radar_clusters`' (`radar-clustering`'s `cloud`, a path
//! dependency): x, y and z by name, float32, either endianness, `point_step` and
//! `row_step`, organised (Gazebo's 1800 x 16) or not (the VLP-16's). Only the positions
//! are kept; rays that hit nothing (inf, NaN) are left for [`crate::cubes::build`] to
//! skip and count.

pub use radar_clustering::cloud::{CloudError, CloudView, Field};

/// Every point's x, y, z, in the cloud's frame.
pub fn positions(cloud: &CloudView<'_>) -> Result<Vec<[f32; 3]>, CloudError> {
    Ok(radar_clustering::cloud::read_points(cloud)?
        .into_iter()
        .map(|p| p.position)
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gazebo's layout: x, y, z, intensity (float32), ring (uint16), 32 bytes a point;
    /// organised, inf where a ray hit nothing.
    #[test]
    fn reads_an_organised_cloud_with_inf() {
        let fields = vec![
            Field {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            Field {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            Field {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
            Field {
                name: "intensity",
                offset: 16,
                datatype: 7,
                count: 1,
            },
            Field {
                name: "ring",
                offset: 20,
                datatype: 4,
                count: 1,
            },
        ];
        let mut data = Vec::new();
        for p in [
            [1.0_f32, 2.0, 3.0],
            [f32::INFINITY, f32::INFINITY, f32::INFINITY],
        ] {
            let mut point = [0_u8; 32];
            for (i, v) in p.iter().enumerate() {
                point[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
            }
            data.extend(point);
        }
        let cloud = CloudView {
            fields,
            width: 1,
            height: 2,
            point_step: 32,
            row_step: 32,
            is_bigendian: false,
            data: &data,
        };
        let got = positions(&cloud).expect("a valid cloud");
        assert_eq!(got[0], [1.0, 2.0, 3.0]);
        assert!(got[1][0].is_infinite());
    }

    #[test]
    fn a_cloud_without_z_is_refused() {
        let fields = vec![
            Field {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            Field {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
        ];
        let cloud = CloudView {
            fields,
            width: 0,
            height: 1,
            point_step: 8,
            row_step: 0,
            is_bigendian: false,
            data: &[],
        };
        assert_eq!(positions(&cloud), Err(CloudError::MissingField("z")));
    }
}

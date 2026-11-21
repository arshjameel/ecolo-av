//! The bytes of a `sensor_msgs/PointCloud2`: its x, y and z read, and an x y z float32
//! cloud written, as `sensor_msgs_py`'s `read_points_numpy` and `create_cloud_xyz32` do.
//!
//! Both organised clouds (Gazebo's and our simulator's: one row per beam) and unorganised
//! ones (a VLP-16's) are read; every point is kept, NaN and inf included: [`crate::cubes`]
//! drops them. The reading is the shared `cloud-view` crate's (TODO 176).

pub use cloud_view::{CloudError, CloudView, Field, XYZ32_FIELDS, XYZ32_STEP};

use crate::Point;

/// PointField's number for float32.
pub const FLOAT32: u8 = cloud_view::Datatype::Float32.code();
/// PointField's number for float64.
pub const FLOAT64: u8 = cloud_view::Datatype::Float64.code();

/// Every point's x, y and z, row by row, in the cloud's frame.
///
/// # Errors
///
/// [`CloudError`] if x, y or z is missing or not a float, or the bytes are too few.
pub fn read_xyz(cloud: &CloudView<'_>) -> Result<Vec<Point>, CloudError> {
    Ok(cloud_view::xyz(cloud)?
        .map(|[x, y, z]| Point::new(x, y, z))
        .collect())
}

/// The points as an x y z float32 cloud's data, little-endian, one row.
#[must_use]
pub fn write_xyz32(points: &[Point]) -> Vec<u8> {
    cloud_view::write_xyz32(points.iter().map(|p| [p.x, p.y, p.z]))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fields(datatype: u8, size: u32) -> Vec<Field<'static>> {
        ["x", "y", "z"]
            .into_iter()
            .zip(0..)
            .map(|(name, i)| Field {
                name,
                offset: size * i,
                datatype,
                count: 1,
            })
            .collect()
    }

    #[test]
    fn xyz_comes_out_of_an_organised_cloud_with_padding() {
        // Two rows (beams) of one point: x, y, z float32 then a ring, 16 bytes a point;
        // each row padded to 24 bytes.
        let mut data = Vec::new();
        for p in [[1.0f32, 2.0, 3.0], [f32::INFINITY, f32::NAN, 0.5]] {
            p.iter().for_each(|v| data.extend(v.to_le_bytes()));
            data.extend([0u8; 12]);
        }
        let cloud = CloudView {
            fields: fields(FLOAT32, 4),
            width: 1,
            height: 2,
            point_step: 16,
            row_step: 24,
            is_bigendian: false,
            data: &data,
        };
        let points = read_xyz(&cloud).expect("a good cloud");
        assert_eq!(points[0], Point::new(1.0, 2.0, 3.0));
        assert!(points[1].x.is_infinite() && points[1].y.is_nan());
        let short = CloudView {
            data: &data[..39],
            ..cloud.clone()
        };
        assert!(matches!(read_xyz(&short), Err(CloudError::TooShort { .. })));
        let no_z = CloudView {
            fields: cloud.fields[..2].to_vec(),
            ..cloud
        };
        assert_eq!(read_xyz(&no_z), Err(CloudError::MissingField("z".into())));
    }

    #[test]
    fn doubles_and_big_endian_are_read_too() {
        let data: Vec<u8> = [1.5f64, -2.0, 0.25]
            .iter()
            .flat_map(|v| v.to_be_bytes())
            .collect();
        let cloud = CloudView {
            fields: fields(FLOAT64, 8),
            width: 1,
            height: 1,
            point_step: 24,
            row_step: 24,
            is_bigendian: true,
            data: &data,
        };
        assert_eq!(read_xyz(&cloud), Ok(vec![Point::new(1.5, -2.0, 0.25)]));
        let ints = CloudView {
            fields: fields(5, 4),
            ..cloud
        };
        assert_eq!(read_xyz(&ints), Err(CloudError::WrongType("x".into())));
    }

    #[test]
    fn written_points_read_back() {
        let points = [Point::new(1.0, -2.5, 0.125), Point::new(10.25, 0.0, 3.0)];
        let data = write_xyz32(&points);
        assert_eq!(data.len(), 2 * XYZ32_STEP as usize);
        let cloud = CloudView {
            fields: XYZ32_FIELDS.to_vec(),
            width: 2,
            height: 1,
            point_step: XYZ32_STEP,
            row_step: 2 * XYZ32_STEP,
            is_bigendian: false,
            data: &data,
        };
        assert_eq!(read_xyz(&cloud), Ok(points.to_vec()));
    }
}

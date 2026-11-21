//! The points of a `sensor_msgs/PointCloud2`: read from its bytes, and written back.
//!
//! No ROS here either: the node copies a message's fields into a [`CloudView`], and puts
//! the bytes [`write_clusters`] makes into the message it publishes. The TI radar driver
//! and the bench's pretend radar both send float32 x, y, z, intensity and velocity: 20
//! bytes a point, little-endian. The reading is the shared `cloud-view` crate's (TODO 176).

pub use cloud_view::{CloudError, CloudView, Datatype, Field};

use crate::{colour, Cluster, Point};

/// Reads every point of `cloud`, row by row. Intensity and velocity are 0 if it has none.
///
/// # Errors
///
/// [`CloudError`] if x, y or z is missing or not a float32, intensity or velocity is not
/// a float32, or the bytes are too few.
pub fn read_points(cloud: &CloudView<'_>) -> Result<Vec<Point>, CloudError> {
    let float32 = [Datatype::Float32];
    let required = |name| cloud.column(name, &float32);
    let position = [required("x")?, required("y")?, required("z")?];
    let intensity = cloud.optional_column("intensity", &float32)?;
    let velocity = cloud.optional_column("velocity", &float32)?;
    // A float32 read as f64 and back is the same float32.
    let value = |bytes: &[u8], column: &cloud_view::Column| column.read(bytes) as f32;
    Ok(cloud
        .points()?
        .map(|bytes| Point {
            position: position.map(|c| value(bytes, &c)),
            intensity: intensity.map_or(0.0, |c| value(bytes, &c)),
            velocity: velocity.map_or(0.0, |c| value(bytes, &c)),
        })
        .collect())
}

/// The fields of each point in the clusters cloud: x, y, z, intensity and velocity as
/// they came in; `rgb`, the cluster's colour (RViz's RGB8 reads it); and `cluster`, its
/// number, from 1, the same as its box's id.
pub const CLUSTER_FIELDS: [Field<'static>; 7] = [
    float32("x", 0),
    float32("y", 4),
    float32("z", 8),
    float32("intensity", 12),
    float32("velocity", 16),
    Field::new("rgb", 20, Datatype::Uint32),
    Field::new("cluster", 24, Datatype::Uint32),
];

/// Bytes per point in the clusters cloud.
pub const CLUSTER_POINT_STEP: u32 = 28;

const fn float32(name: &'static str, offset: u32) -> Field<'static> {
    Field::new(name, offset, Datatype::Float32)
}

/// The clusters cloud's bytes, little-endian, laid out as [`CLUSTER_FIELDS`]: the first
/// cluster's points, then the second's, and so on. Noise points are left out.
///
/// `clusters` come from [`crate::cluster`] on these `points`; an index past the end of
/// `points` (which it never makes) is skipped.
#[must_use]
pub fn write_clusters(points: &[Point], clusters: &[Cluster]) -> Vec<u8> {
    clusters
        .iter()
        .enumerate()
        .flat_map(|(index, cluster)| {
            let rgb = colour(index).packed();
            let number = u32::try_from(index + 1).unwrap_or(u32::MAX);
            cluster
                .members
                .iter()
                .filter_map(|&i| points.get(i))
                .flat_map(move |p| {
                    let [x, y, z] = p.position;
                    [x, y, z, p.intensity, p.velocity]
                        .into_iter()
                        .flat_map(f32::to_le_bytes)
                        .chain(rgb.to_le_bytes())
                        .chain(number.to_le_bytes())
                })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{cluster, Metres, Settings};

    /// A cloud laid out as the TI driver lays it out: x, y, z, intensity, velocity.
    fn ti_cloud(points: &[[f32; 5]]) -> (Vec<Field<'static>>, Vec<u8>) {
        let fields = ["x", "y", "z", "intensity", "velocity"]
            .iter()
            .zip(0..)
            .map(|(&name, i)| float32(name, 4 * i))
            .collect();
        let data = points
            .iter()
            .flatten()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        (fields, data)
    }

    fn view<'a>(fields: Vec<Field<'a>>, data: &'a [u8], width: u32, step: u32) -> CloudView<'a> {
        CloudView {
            fields,
            width,
            height: 1,
            point_step: step,
            row_step: step * width,
            is_bigendian: false,
            data,
        }
    }

    #[test]
    fn reads_the_ti_drivers_points() {
        let (fields, data) = ti_cloud(&[[1.0, 2.0, 3.0, 4.0, 5.0], [-1.5, 0.0, 0.25, 9.0, -2.0]]);
        let points = read_points(&view(fields, &data, 2, 20)).expect("a valid cloud");
        assert_eq!(points[0].position, [1.0, 2.0, 3.0]);
        assert_eq!((points[0].intensity, points[0].velocity), (4.0, 5.0));
        assert_eq!(points[1].position, [-1.5, 0.0, 0.25]);
        assert_eq!((points[1].intensity, points[1].velocity), (9.0, -2.0));
    }

    #[test]
    fn intensity_and_velocity_are_optional_but_x_y_z_are_not() {
        let (mut fields, data) = ti_cloud(&[[1.0, 2.0, 3.0, 4.0, 5.0]]);
        fields.truncate(3);
        let points = read_points(&view(fields.clone(), &data, 1, 20)).expect("x, y, z");
        assert_eq!((points[0].intensity, points[0].velocity), (0.0, 0.0));
        fields.remove(1);
        let missing = read_points(&view(fields, &data, 1, 20));
        assert_eq!(missing, Err(CloudError::MissingField("y".into())));
    }

    #[test]
    fn a_field_of_another_type_or_outside_the_point_is_refused() {
        let (mut fields, data) = ti_cloud(&[[1.0, 2.0, 3.0, 4.0, 5.0]]);
        fields[2].datatype = 8; // float64
        let refused = read_points(&view(fields.clone(), &data, 1, 20));
        assert_eq!(refused, Err(CloudError::WrongType("z".into())));
        fields[2].datatype = Datatype::Float32.code();
        fields[4].offset = 18; // 18 + 4 > 20
        let refused = read_points(&view(fields, &data, 1, 20));
        assert_eq!(refused, Err(CloudError::OutsidePoint("velocity".into())));
    }

    #[test]
    fn a_cloud_shorter_than_it_says_is_refused() {
        let (fields, data) = ti_cloud(&[[1.0, 2.0, 3.0, 4.0, 5.0]]);
        let refused = read_points(&view(fields, &data, 2, 20));
        assert_eq!(
            refused,
            Err(CloudError::TooShort {
                points: 2,
                bytes: 20
            })
        );
    }

    #[test]
    fn an_empty_cloud_has_no_points() {
        let (fields, _) = ti_cloud(&[]);
        assert_eq!(read_points(&view(fields, &[], 0, 20)), Ok(Vec::new()));
    }

    #[test]
    fn big_endian_clouds_and_padded_rows_are_read() {
        let (fields, _) = ti_cloud(&[]);
        // Two rows of one point each, 4 spare bytes after each row.
        let mut data = Vec::new();
        for p in [[1.0_f32, 2.0, 3.0, 0.0, 0.0], [4.0, 5.0, 6.0, 0.0, 0.0]] {
            data.extend(p.iter().flat_map(|v| v.to_be_bytes()));
            data.extend([0xee; 4]);
        }
        let cloud = CloudView {
            fields,
            width: 1,
            height: 2,
            point_step: 20,
            row_step: 24,
            is_bigendian: true,
            data: &data,
        };
        let points = read_points(&cloud).expect("a valid cloud");
        assert_eq!(points[0].position, [1.0, 2.0, 3.0]);
        assert_eq!(points[1].position, [4.0, 5.0, 6.0]);
    }

    #[test]
    fn the_clusters_cloud_reads_back_with_its_colours_and_numbers() {
        let raw = [
            [1.0, 0.0, 0.0, 7.0, 0.5],  // 0: first cluster
            [9.0, 9.0, 9.0, 1.0, 0.0],  // 1: noise
            [1.1, 0.0, 0.0, 8.0, 0.5],  // 2: first cluster
            [5.0, 0.0, 0.0, 2.0, -1.0], // 3: second cluster
            [5.1, 0.0, 0.0, 3.0, -1.0], // 4: second cluster
        ];
        let (fields, data) = ti_cloud(&raw);
        let points = read_points(&view(fields, &data, 5, 20)).expect("a valid cloud");
        let settings = Settings::new(Metres(0.5), 2).expect("valid settings");
        let clusters = cluster(&points, &settings);
        let bytes = write_clusters(&points, &clusters);
        assert_eq!(bytes.len(), 4 * 28);

        let back = view(CLUSTER_FIELDS.to_vec(), &bytes, 4, CLUSTER_POINT_STEP);
        let read = read_points(&back).expect("our own layout");
        let want: Vec<_> = [0, 2, 3, 4].iter().map(|&i| points[i]).collect();
        assert_eq!(read, want);
        let u32_at = |point: usize, offset: usize| {
            let at = point * 28 + offset;
            u32::from_le_bytes([0, 1, 2, 3].map(|i| bytes[at + i]))
        };
        let rgb = [0, 1, 2, 3].map(|p| u32_at(p, 20));
        let number = [0, 1, 2, 3].map(|p| u32_at(p, 24));
        assert_eq!(
            rgb,
            [colour(0), colour(0), colour(1), colour(1)].map(|c| c.packed())
        );
        assert_eq!(number, [1, 1, 2, 2]);
    }

    #[test]
    fn datatype_numbers_are_pointfields() {
        assert_eq!(Datatype::from_code(7), Some(Datatype::Float32));
        assert_eq!(Datatype::from_code(6), Some(Datatype::Uint32));
        assert_eq!(Datatype::from_code(9), None);
    }
}

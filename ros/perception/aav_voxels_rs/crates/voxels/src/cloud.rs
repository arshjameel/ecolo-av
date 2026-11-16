//! The bytes of a `sensor_msgs/PointCloud2`: its x, y and z read, and an x y z float32
//! cloud written, as `sensor_msgs_py`'s `read_points_numpy` and `create_cloud_xyz32` do.
//!
//! Both organised clouds (Gazebo's and our simulator's: one row per beam) and unorganised
//! ones (a VLP-16's) are read; every point is kept, NaN and inf included: [`crate::cubes`]
//! drops them.

use std::{error::Error, fmt};

use crate::Point;

/// PointField's number for float32.
pub const FLOAT32: u8 = 7;
/// PointField's number for float64.
pub const FLOAT64: u8 = 8;

/// A field of every point, as `sensor_msgs/PointField` describes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field<'a> {
    /// `x`, `y`, `intensity`...
    pub name: &'a str,
    /// Where it starts in each point, in bytes.
    pub offset: u32,
    /// PointField's number for its type: [`FLOAT32`] or [`FLOAT64`] for x, y and z.
    pub datatype: u8,
    /// How many of that type.
    pub count: u32,
}

/// The parts of a `sensor_msgs/PointCloud2` needed for its x, y and z.
#[derive(Clone, Debug)]
pub struct CloudView<'a> {
    /// What each point holds.
    pub fields: Vec<Field<'a>>,
    /// Points in a row.
    pub width: u32,
    /// Rows.
    pub height: u32,
    /// Bytes a point.
    pub point_step: u32,
    /// Bytes a row.
    pub row_step: u32,
    /// The numbers are big-endian.
    pub is_bigendian: bool,
    /// The points' bytes.
    pub data: &'a [u8],
}

/// Why a cloud's points couldn't be read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CloudError {
    /// No field of this name: x, y and z are needed.
    MissingField(&'static str),
    /// The field isn't one float32 or float64.
    NotFloat(&'static str),
    /// The field doesn't fit inside a point.
    OutsidePoint(&'static str),
    /// The cloud says it has more points than its bytes hold.
    TooShort {
        /// Points the cloud says it has.
        points: usize,
        /// Bytes it has.
        bytes: usize,
    },
}

impl fmt::Display for CloudError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingField(name) => write!(f, "the cloud has no {name} field"),
            Self::NotFloat(name) => write!(f, "the cloud's {name} field isn't one float"),
            Self::OutsidePoint(name) => write!(f, "the cloud's {name} field is outside a point"),
            Self::TooShort { points, bytes } => {
                write!(f, "the cloud says {points} points, in only {bytes} bytes")
            }
        }
    }
}

impl Error for CloudError {}

/// Where one coordinate is in a point, and its type.
#[derive(Clone, Copy)]
enum Float {
    F32(usize),
    F64(usize),
}

impl Float {
    fn of(cloud: &CloudView<'_>, name: &'static str) -> Result<Self, CloudError> {
        let f = cloud
            .fields
            .iter()
            .find(|f| f.name == name)
            .ok_or(CloudError::MissingField(name))?;
        let offset = f.offset as usize;
        let (float, size) = match (f.datatype, f.count) {
            (FLOAT32, 1) => (Self::F32(offset), 4),
            (FLOAT64, 1) => (Self::F64(offset), 8),
            _ => return Err(CloudError::NotFloat(name)),
        };
        if offset + size > cloud.point_step as usize {
            return Err(CloudError::OutsidePoint(name));
        }
        Ok(float)
    }

    /// The value in one point's bytes (`of` checked that it fits).
    fn read(self, bytes: &[u8], big: bool) -> f64 {
        match self {
            Self::F32(o) => {
                let raw = [0, 1, 2, 3].map(|i| bytes[o + i]);
                f64::from(if big {
                    f32::from_be_bytes(raw)
                } else {
                    f32::from_le_bytes(raw)
                })
            }
            Self::F64(o) => {
                let raw = [0, 1, 2, 3, 4, 5, 6, 7].map(|i| bytes[o + i]);
                if big {
                    f64::from_be_bytes(raw)
                } else {
                    f64::from_le_bytes(raw)
                }
            }
        }
    }
}

/// Every point's x, y and z, row by row, in the cloud's frame.
///
/// # Errors
///
/// [`CloudError`] if x, y or z is missing or not a float, or the bytes are too few.
pub fn read_xyz(cloud: &CloudView<'_>) -> Result<Vec<Point>, CloudError> {
    let xyz = [
        Float::of(cloud, "x")?,
        Float::of(cloud, "y")?,
        Float::of(cloud, "z")?,
    ];
    let (width, height) = (cloud.width as usize, cloud.height as usize);
    let (step, row_step) = (cloud.point_step as usize, cloud.row_step as usize);
    let too_short = || CloudError::TooShort {
        points: width.saturating_mul(height),
        bytes: cloud.data.len(),
    };
    let big = cloud.is_bigendian;
    (0..height)
        .flat_map(|row| (0..width).map(move |column| row * row_step + column * step))
        .map(|start| {
            let bytes = cloud.data.get(start..start + step).ok_or_else(too_short)?;
            let [x, y, z] = xyz.map(|f| f.read(bytes, big));
            Ok(Point::new(x, y, z))
        })
        .collect()
}

/// The fields of an x y z float32 cloud: name, offset, datatype, count.
pub const XYZ32_FIELDS: [Field<'static>; 3] = [
    Field {
        name: "x",
        offset: 0,
        datatype: FLOAT32,
        count: 1,
    },
    Field {
        name: "y",
        offset: 4,
        datatype: FLOAT32,
        count: 1,
    },
    Field {
        name: "z",
        offset: 8,
        datatype: FLOAT32,
        count: 1,
    },
];

/// One x y z float32 point's bytes.
pub const XYZ32_STEP: u32 = 12;

/// The points as an x y z float32 cloud's data, little-endian, one row.
#[must_use]
pub fn write_xyz32(points: &[Point]) -> Vec<u8> {
    points
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .flat_map(|v| (v as f32).to_le_bytes())
        .collect()
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
        assert_eq!(read_xyz(&no_z), Err(CloudError::MissingField("z")));
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
        assert_eq!(read_xyz(&ints), Err(CloudError::NotFloat("x")));
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

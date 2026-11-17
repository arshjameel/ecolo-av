//! Named columns of a `sensor_msgs/PointCloud2`'s bytes, any number type, as `f64`: what
//! `sensor_msgs_py`'s `read_points` gives for mixed fields (the radar clusters' x, y,
//! velocity float32 and cluster uint32).

use std::{error::Error, fmt};

/// One field of a point: its name, where it starts, its type and count.
#[derive(Clone, Copy, Debug)]
pub struct Field<'a> {
    /// Its name: `x`, `velocity`...
    pub name: &'a str,
    /// Bytes from the point's start.
    pub offset: u32,
    /// PointField's datatype: 1 INT8 ... 8 FLOAT64.
    pub datatype: u8,
    /// Values in it (1 for a number).
    pub count: u32,
}

/// A cloud's layout and bytes.
#[derive(Clone, Debug)]
pub struct CloudView<'a> {
    /// Its fields.
    pub fields: Vec<Field<'a>>,
    /// Points a row.
    pub width: u32,
    /// Rows.
    pub height: u32,
    /// Bytes a point.
    pub point_step: u32,
    /// Bytes a row.
    pub row_step: u32,
    /// Big-endian numbers.
    pub is_bigendian: bool,
    /// The points.
    pub data: &'a [u8],
}

/// A cloud that can't be read as asked.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CloudError {
    /// No field of that name.
    MissingField(String),
    /// A field that is not one number (count 1) of a known type.
    BadType(String),
    /// A field that runs past the point's end.
    OutsidePoint(String),
    /// Fewer bytes than the points need.
    TooShort {
        /// Points the header promises.
        points: usize,
        /// Bytes there are.
        bytes: usize,
    },
}

impl fmt::Display for CloudError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingField(n) => write!(f, "the cloud has no field {n}"),
            Self::BadType(n) => write!(f, "field {n} is not one number of a known type"),
            Self::OutsidePoint(n) => write!(f, "field {n} runs past the point's end"),
            Self::TooShort { points, bytes } => {
                write!(f, "{points} points but only {bytes} bytes")
            }
        }
    }
}

impl Error for CloudError {}

#[derive(Clone, Copy)]
enum Kind {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Kind {
    fn of(datatype: u8) -> Option<(Self, usize)> {
        Some(match datatype {
            1 => (Self::I8, 1),
            2 => (Self::U8, 1),
            3 => (Self::I16, 2),
            4 => (Self::U16, 2),
            5 => (Self::I32, 4),
            6 => (Self::U32, 4),
            7 => (Self::F32, 4),
            8 => (Self::F64, 8),
            _ => return None,
        })
    }

    fn read(self, b: &[u8], big: bool) -> f64 {
        macro_rules! num {
            ($t:ty, $n:expr) => {{
                let mut raw = [0u8; $n];
                raw.copy_from_slice(&b[..$n]);
                if big {
                    <$t>::from_be_bytes(raw)
                } else {
                    <$t>::from_le_bytes(raw)
                }
            }};
        }
        match self {
            Self::I8 => f64::from(num!(i8, 1)),
            Self::U8 => f64::from(num!(u8, 1)),
            Self::I16 => f64::from(num!(i16, 2)),
            Self::U16 => f64::from(num!(u16, 2)),
            Self::I32 => f64::from(num!(i32, 4)),
            Self::U32 => f64::from(num!(u32, 4)),
            Self::F32 => f64::from(num!(f32, 4)),
            Self::F64 => num!(f64, 8),
        }
    }
}

/// Each point's values of `names`, in that order, row by row.
///
/// # Errors
///
/// [`CloudError`]: a field missing, not one number, past the point's end, or too few bytes.
pub fn read_columns(cloud: &CloudView<'_>, names: &[&str]) -> Result<Vec<Vec<f64>>, CloudError> {
    let columns = names
        .iter()
        .map(|&name| {
            let f = cloud
                .fields
                .iter()
                .find(|f| f.name == name)
                .ok_or_else(|| CloudError::MissingField(name.to_string()))?;
            let (kind, size) = Kind::of(f.datatype)
                .filter(|_| f.count == 1)
                .ok_or_else(|| CloudError::BadType(name.to_string()))?;
            let offset = f.offset as usize;
            if offset + size > cloud.point_step as usize {
                return Err(CloudError::OutsidePoint(name.to_string()));
            }
            Ok((kind, offset))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let (width, height) = (cloud.width as usize, cloud.height as usize);
    let (step, row_step) = (cloud.point_step as usize, cloud.row_step as usize);
    (0..height)
        .flat_map(|row| (0..width).map(move |col| row * row_step + col * step))
        .map(|start| {
            let bytes = cloud
                .data
                .get(start..start + step)
                .ok_or(CloudError::TooShort {
                    points: width * height,
                    bytes: cloud.data.len(),
                })?;
            Ok(columns
                .iter()
                .map(|&(kind, offset)| kind.read(&bytes[offset..], cloud.is_bigendian))
                .collect())
        })
        .collect()
}

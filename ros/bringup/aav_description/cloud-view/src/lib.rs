//! A `sensor_msgs/PointCloud2`'s bytes, read by field name. No ROS.
//!
//! One view for every Rust crate that reads a cloud (TODO 176): the voxels, the lidar
//! scan, the things, the radar clusters, NDT and the debug screens. A node copies the
//! message's layout into a [`CloudView`]; the crate's own module reads its points.
//!
//! ```text
//!  PointCloud2 ─► CloudView ─► column("x", FLOATS) ─► Column ─┐
//!   (the node)    (no ROS)     a name, a type, an offset      ├─► each point's value
//!                              points() ─► each point's bytes ┘
//! ```
//!
//! Both organised clouds (one row per beam, padded rows) and flat ones are read, either
//! endianness. NaN and infinity come out as they are: the caller drops them.

#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use std::{error::Error, fmt};

/// A `sensor_msgs/PointField` number type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Datatype {
    /// INT8 (1).
    Int8,
    /// UINT8 (2).
    Uint8,
    /// INT16 (3).
    Int16,
    /// UINT16 (4).
    Uint16,
    /// INT32 (5).
    Int32,
    /// UINT32 (6).
    Uint32,
    /// FLOAT32 (7).
    Float32,
    /// FLOAT64 (8).
    Float64,
}

impl Datatype {
    /// Every type, in PointField's order.
    pub const ALL: [Self; 8] = [
        Self::Int8,
        Self::Uint8,
        Self::Int16,
        Self::Uint16,
        Self::Int32,
        Self::Uint32,
        Self::Float32,
        Self::Float64,
    ];

    /// The two float types: what x, y and z may be.
    pub const FLOATS: [Self; 2] = [Self::Float32, Self::Float64];

    /// The number PointField uses for it.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::Int8 => 1,
            Self::Uint8 => 2,
            Self::Int16 => 3,
            Self::Uint16 => 4,
            Self::Int32 => 5,
            Self::Uint32 => 6,
            Self::Float32 => 7,
            Self::Float64 => 8,
        }
    }

    /// The type a PointField number stands for, if any.
    #[must_use]
    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|d| d.code() == code)
    }

    /// Bytes one value takes.
    #[must_use]
    pub const fn size(self) -> usize {
        match self {
            Self::Int8 | Self::Uint8 => 1,
            Self::Int16 | Self::Uint16 => 2,
            Self::Int32 | Self::Uint32 | Self::Float32 => 4,
            Self::Float64 => 8,
        }
    }
}

/// A field of every point, as `sensor_msgs/PointField` describes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field<'a> {
    /// `x`, `y`, `intensity`...
    pub name: &'a str,
    /// Where it starts in each point, in bytes.
    pub offset: u32,
    /// PointField's number for its type: [`Datatype::code`].
    pub datatype: u8,
    /// How many values it holds (1 for a number).
    pub count: u32,
}

impl<'a> Field<'a> {
    /// One value of `datatype` at `offset`: a field for a cloud we write.
    #[must_use]
    pub const fn new(name: &'a str, offset: u32, datatype: Datatype) -> Self {
        Self {
            name,
            offset,
            datatype: datatype.code(),
            count: 1,
        }
    }
}

/// The parts of a `sensor_msgs/PointCloud2` needed to read it: its layout and its bytes.
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

/// Why a cloud couldn't be read as asked.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CloudError {
    /// No field of this name.
    MissingField(String),
    /// The field isn't one value of a type wanted here.
    WrongType(String),
    /// The field doesn't fit inside a point.
    OutsidePoint(String),
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
            Self::WrongType(name) => {
                write!(
                    f,
                    "the cloud's {name} field isn't one value of a type wanted"
                )
            }
            Self::OutsidePoint(name) => write!(f, "the cloud's {name} field is outside a point"),
            Self::TooShort { points, bytes } => {
                write!(f, "the cloud says {points} points, in only {bytes} bytes")
            }
        }
    }
}

impl Error for CloudError {}

/// Where one field is in each point, and how to read it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Column {
    datatype: Datatype,
    offset: usize,
    big_endian: bool,
}

impl Column {
    /// Its type.
    #[must_use]
    pub const fn datatype(&self) -> Datatype {
        self.datatype
    }

    /// Its value in one point's bytes (from [`CloudView::points`]), as f64: exact for every
    /// type. Bytes too short for it (another cloud's point) read as NaN.
    #[must_use]
    pub fn read(&self, point: &[u8]) -> f64 {
        let at = self.offset;
        macro_rules! num {
            ($t:ty, $n:expr) => {
                match point
                    .get(at..at + $n)
                    .and_then(|b| <[u8; $n]>::try_from(b).ok())
                {
                    Some(raw) if self.big_endian => <$t>::from_be_bytes(raw),
                    Some(raw) => <$t>::from_le_bytes(raw),
                    None => return f64::NAN,
                }
            };
        }
        match self.datatype {
            Datatype::Int8 => f64::from(num!(i8, 1)),
            Datatype::Uint8 => f64::from(num!(u8, 1)),
            Datatype::Int16 => f64::from(num!(i16, 2)),
            Datatype::Uint16 => f64::from(num!(u16, 2)),
            Datatype::Int32 => f64::from(num!(i32, 4)),
            Datatype::Uint32 => f64::from(num!(u32, 4)),
            Datatype::Float32 => f64::from(num!(f32, 4)),
            Datatype::Float64 => num!(f64, 8),
        }
    }
}

/// A u32 from a message as a usize: lossless on the 32- and 64-bit computers ROS runs on;
/// saturating otherwise, so a size check fails instead of wrapping.
fn to_usize(value: u32) -> usize {
    usize::try_from(value).unwrap_or(usize::MAX)
}

impl<'a> CloudView<'a> {
    /// The field called `name`, if there is one.
    #[must_use]
    pub fn find(&self, name: &str) -> Option<&Field<'a>> {
        self.fields.iter().find(|f| f.name == name)
    }

    /// The field `name` as a [`Column`]: one value of one of the `wanted` types, inside a
    /// point.
    ///
    /// # Errors
    ///
    /// [`CloudError::MissingField`], [`CloudError::WrongType`] or
    /// [`CloudError::OutsidePoint`].
    pub fn column(&self, name: &str, wanted: &[Datatype]) -> Result<Column, CloudError> {
        self.optional_column(name, wanted)?
            .ok_or_else(|| CloudError::MissingField(name.to_string()))
    }

    /// As [`CloudView::column`], but `None` for a cloud without the field.
    ///
    /// # Errors
    ///
    /// [`CloudError::WrongType`] or [`CloudError::OutsidePoint`].
    pub fn optional_column(
        &self,
        name: &str,
        wanted: &[Datatype],
    ) -> Result<Option<Column>, CloudError> {
        let Some(field) = self.find(name) else {
            return Ok(None);
        };
        let datatype = Datatype::from_code(field.datatype)
            .filter(|d| field.count == 1 && wanted.contains(d))
            .ok_or_else(|| CloudError::WrongType(name.to_string()))?;
        let offset = to_usize(field.offset);
        if offset.saturating_add(datatype.size()) > to_usize(self.point_step) {
            return Err(CloudError::OutsidePoint(name.to_string()));
        }
        Ok(Some(Column {
            datatype,
            offset,
            big_endian: self.is_bigendian,
        }))
    }

    /// Points the cloud says it has: width × height.
    #[must_use]
    pub fn len(&self) -> usize {
        to_usize(self.width).saturating_mul(to_usize(self.height))
    }

    /// No points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Each point's bytes (`point_step` of them), row by row.
    ///
    /// # Errors
    ///
    /// [`CloudError::TooShort`]: the last point ends past the bytes.
    pub fn points(&self) -> Result<impl Iterator<Item = &'a [u8]> + 'a, CloudError> {
        let (width, height) = (to_usize(self.width), to_usize(self.height));
        let (step, row_step) = (to_usize(self.point_step), to_usize(self.row_step));
        let data = self.data;
        if !self.is_empty() {
            let end = (height - 1)
                .checked_mul(row_step)
                .and_then(|r| r.checked_add((width - 1).checked_mul(step)?))
                .and_then(|s| s.checked_add(step));
            if end.is_none_or(|end| end > data.len()) {
                return Err(CloudError::TooShort {
                    points: self.len(),
                    bytes: data.len(),
                });
            }
        }
        Ok((0..height)
            .flat_map(move |row| (0..width).map(move |column| row * row_step + column * step))
            // In range: checked above.
            .map(move |start| data.get(start..start + step).unwrap_or_default()))
    }
}

/// Every point's x, y and z (float32 or float64), row by row, in the cloud's frame.
///
/// # Errors
///
/// [`CloudError`] if x, y or z is missing or not a float, or the bytes are too few.
pub fn xyz<'a>(cloud: &CloudView<'a>) -> Result<impl Iterator<Item = [f64; 3]> + 'a, CloudError> {
    let columns = [
        cloud.column("x", &Datatype::FLOATS)?,
        cloud.column("y", &Datatype::FLOATS)?,
        cloud.column("z", &Datatype::FLOATS)?,
    ];
    Ok(cloud.points()?.map(move |p| columns.map(|c| c.read(p))))
}

/// Each point's values of `names` (any number type), in that order, row by row: what
/// `sensor_msgs_py`'s `read_points` gives for mixed fields.
///
/// # Errors
///
/// [`CloudError`]: a field missing, not one number, past the point's end, or too few bytes.
pub fn read_columns(cloud: &CloudView<'_>, names: &[&str]) -> Result<Vec<Vec<f64>>, CloudError> {
    let columns = names
        .iter()
        .map(|name| cloud.column(name, &Datatype::ALL))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(cloud
        .points()?
        .map(|p| columns.iter().map(|c| c.read(p)).collect())
        .collect())
}

/// The fields of an x y z float32 cloud.
pub const XYZ32_FIELDS: [Field<'static>; 3] = [
    Field::new("x", 0, Datatype::Float32),
    Field::new("y", 4, Datatype::Float32),
    Field::new("z", 8, Datatype::Float32),
];

/// One x y z float32 point's bytes.
pub const XYZ32_STEP: u32 = 12;

/// Points as an x y z float32 cloud's data, little-endian, one row.
pub fn write_xyz32(points: impl IntoIterator<Item = [f64; 3]>) -> Vec<u8> {
    points
        .into_iter()
        .flatten()
        .flat_map(|v| (v as f32).to_le_bytes())
        .collect()
}

#[cfg(test)]
mod tests;

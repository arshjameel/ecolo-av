//! A scan before matching: the ground out, the car's own body out, far and broken points
//! out, then thinned to one point per cube. And the x, y, z of a `sensor_msgs/PointCloud2`,
//! read from its bytes.
//!
//! Why (the design): 70-80 % of the points are flat ground, which says nothing about x, y
//! or yaw; ~13,000 points a turn is too many to match 10 times a second.

use std::{collections::BTreeMap, error::Error, fmt};

use crate::Point;

/// How the ground is taken out.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ground {
    /// Keep points higher than this above `base_link` (m): flat ground, as in Gazebo.
    Above(f64),
    /// Keep points more than `height` (m) above the lowest point in their `cell` (m)
    /// square: ground that slopes, as on the car.
    AboveLowest { cell: f64, height: f64 },
    /// Keep the ground.
    Keep,
}

/// A box round the car's own body, in `base_link` (m): points inside are the car.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyBox {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl BodyBox {
    fn contains(&self, p: &Point) -> bool {
        (self.min[0]..=self.max[0]).contains(&p.x) && (self.min[1]..=self.max[1]).contains(&p.y)
    }
}

/// What a scan goes through before matching, in `base_link`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScanFilter {
    pub ground: Ground,
    /// The car's body; `None` keeps everything near.
    pub body: Option<BodyBox>,
    /// Points further than this from `base_link`, flat (m), are dropped.
    pub max_range: f64,
    /// One point per cube of this side (m), at the mean of its points; 0 keeps all.
    pub thin: f64,
}

impl Default for ScanFilter {
    /// The design's numbers: ground below 0.3 m out, 45 m, thinned to 1 m cubes.
    fn default() -> Self {
        Self {
            ground: Ground::Above(0.3),
            body: None,
            max_range: 45.0,
            thin: 1.0,
        }
    }
}

impl ScanFilter {
    /// The points worth matching, in `base_link`, in a fixed order.
    pub fn apply(&self, points: impl IntoIterator<Item = Point>) -> Vec<Point> {
        let near: Vec<Point> = points
            .into_iter()
            .filter(|p| p.iter().all(|v| v.is_finite()))
            .filter(|p| p.x.hypot(p.y) <= self.max_range)
            .filter(|p| !self.body.is_some_and(|b| b.contains(p)))
            .collect();
        let off_ground = remove_ground(near, self.ground);
        if self.thin > 0.0 {
            thin(&off_ground, self.thin)
        } else {
            off_ground
        }
    }
}

/// The points that aren't ground.
pub fn remove_ground(points: Vec<Point>, ground: Ground) -> Vec<Point> {
    match ground {
        Ground::Keep => points,
        Ground::Above(height) => points.into_iter().filter(|p| p.z > height).collect(),
        Ground::AboveLowest { cell, height } => {
            let mut lowest: BTreeMap<[i64; 2], f64> = BTreeMap::new();
            for p in &points {
                let low = lowest.entry(cell_of(p, cell)).or_insert(f64::INFINITY);
                *low = low.min(p.z);
            }
            points
                .into_iter()
                .filter(|p| {
                    lowest
                        .get(&cell_of(p, cell))
                        .is_some_and(|low| p.z > low + height)
                })
                .collect()
        }
    }
}

fn cell_of(p: &Point, cell: f64) -> [i64; 2] {
    [(p.x / cell).floor() as i64, (p.y / cell).floor() as i64]
}

/// One point per cube of side `size` (m): the mean of the points in it. In a fixed order
/// (by cube), so the same scan always gives the same answer.
pub fn thin(points: &[Point], size: f64) -> Vec<Point> {
    let mut cubes: BTreeMap<[i64; 3], (nalgebra::Vector3<f64>, usize)> = BTreeMap::new();
    for p in points {
        let key = [
            (p.x / size).floor() as i64,
            (p.y / size).floor() as i64,
            (p.z / size).floor() as i64,
        ];
        let (sum, n) = cubes.entry(key).or_insert((nalgebra::Vector3::zeros(), 0));
        *sum += p.coords;
        *n += 1;
    }
    cubes
        .into_values()
        .map(|(sum, n)| Point::from(sum / n as f64))
        .collect()
}

/// A field of every point, as `sensor_msgs/PointField` describes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field<'a> {
    pub name: &'a str,
    /// Where it starts in each point, in bytes.
    pub offset: u32,
    /// PointField's number for its type: 7 is float32, 8 float64.
    pub datatype: u8,
    pub count: u32,
}

/// The parts of a `sensor_msgs/PointCloud2` needed for its x, y and z.
#[derive(Clone, Debug)]
pub struct CloudView<'a> {
    pub fields: Vec<Field<'a>>,
    pub width: u32,
    pub height: u32,
    pub point_step: u32,
    pub row_step: u32,
    pub is_bigendian: bool,
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
    TooShort { points: usize, bytes: usize },
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

#[derive(Clone, Copy)]
enum Float {
    F32(usize),
    F64(usize),
}

/// Every point's x, y and z, row by row, in the cloud's frame. Points that hit nothing
/// (inf, NaN) are kept: [`ScanFilter::apply`] drops them.
pub fn read_xyz(cloud: &CloudView<'_>) -> Result<Vec<Point>, CloudError> {
    let field = |name: &'static str| -> Result<Float, CloudError> {
        let f = cloud
            .fields
            .iter()
            .find(|f| f.name == name)
            .ok_or(CloudError::MissingField(name))?;
        let offset = f.offset as usize;
        let (float, size) = match (f.datatype, f.count) {
            (7, 1) => (Float::F32(offset), 4),
            (8, 1) => (Float::F64(offset), 8),
            _ => return Err(CloudError::NotFloat(name)),
        };
        if offset + size > cloud.point_step as usize {
            return Err(CloudError::OutsidePoint(name));
        }
        Ok(float)
    };
    let xyz = [field("x")?, field("y")?, field("z")?];
    let (width, height) = (cloud.width as usize, cloud.height as usize);
    let (step, row_step) = (cloud.point_step as usize, cloud.row_step as usize);
    let too_short = || CloudError::TooShort {
        points: width.saturating_mul(height),
        bytes: cloud.data.len(),
    };
    let big = cloud.is_bigendian;
    let value = |bytes: &[u8], float: Float| -> f64 {
        // In range: field() checked that the value fits in point_step = bytes.len().
        match float {
            Float::F32(o) => {
                let raw = [0, 1, 2, 3].map(|i| bytes[o + i]);
                f64::from(if big {
                    f32::from_be_bytes(raw)
                } else {
                    f32::from_le_bytes(raw)
                })
            }
            Float::F64(o) => {
                let raw = [0, 1, 2, 3, 4, 5, 6, 7].map(|i| bytes[o + i]);
                if big {
                    f64::from_be_bytes(raw)
                } else {
                    f64::from_le_bytes(raw)
                }
            }
        }
    };
    (0..height)
        .flat_map(|row| (0..width).map(move |column| row * row_step + column * step))
        .map(|start| {
            let bytes = cloud.data.get(start..start + step).ok_or_else(too_short)?;
            let [x, y, z] = xyz.map(|f| value(bytes, f));
            Ok(Point::new(x, y, z))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_filter_drops_ground_far_broken_and_body_points() {
        let filter = ScanFilter {
            body: Some(BodyBox {
                min: [-0.5, -0.5],
                max: [2.0, 0.5],
            }),
            thin: 0.0,
            ..ScanFilter::default()
        };
        let kept = filter.apply([
            Point::new(5.0, 0.0, 1.0),           // a wall: kept
            Point::new(5.0, 0.0, 0.1),           // the ground
            Point::new(50.0, 0.0, 1.0),          // too far
            Point::new(f64::INFINITY, 0.0, 1.0), // hit nothing
            Point::new(1.0, 0.0, 1.0),           // the car's own roof
        ]);
        assert_eq!(kept, vec![Point::new(5.0, 0.0, 1.0)]);
    }

    #[test]
    fn the_lowest_point_filter_follows_a_slope() {
        // A slope rising 1 m over 10 m, and a post on it.
        let mut points: Vec<Point> = (0..100)
            .map(|i| Point::new(i as f64 * 0.1, 0.0, i as f64 * 0.01))
            .collect();
        points.push(Point::new(5.05, 0.05, 1.5));
        let kept = remove_ground(
            points,
            Ground::AboveLowest {
                cell: 1.0,
                height: 0.3,
            },
        );
        assert_eq!(kept, vec![Point::new(5.05, 0.05, 1.5)]);
    }

    #[test]
    fn thinning_keeps_one_mean_point_per_cube() {
        let points = [
            Point::new(0.1, 0.1, 0.1),
            Point::new(0.3, 0.3, 0.3),
            Point::new(1.5, 0.5, 0.5),
        ];
        let thinned = thin(&points, 1.0);
        assert_eq!(thinned.len(), 2);
        assert!((thinned[0] - Point::new(0.2, 0.2, 0.2)).norm() < 1e-12);
    }

    #[test]
    fn xyz_comes_out_of_the_bytes() {
        // x, y, z float32 then a 2-byte ring: 14 bytes, padded to 16.
        let mut data = Vec::new();
        for p in [[1.0f32, 2.0, 3.0], [f32::INFINITY, 0.0, 0.0]] {
            p.iter().for_each(|v| data.extend(v.to_le_bytes()));
            data.extend([0u8; 4]);
        }
        let fields = ["x", "y", "z"]
            .iter()
            .enumerate()
            .map(|(i, name)| Field {
                name,
                offset: 4 * i as u32,
                datatype: 7,
                count: 1,
            })
            .collect();
        let cloud = CloudView {
            fields,
            width: 2,
            height: 1,
            point_step: 16,
            row_step: 32,
            is_bigendian: false,
            data: &data,
        };
        let points = read_xyz(&cloud).expect("a good cloud");
        assert_eq!(points[0], Point::new(1.0, 2.0, 3.0));
        assert!(points[1].x.is_infinite());
        let short = CloudView {
            data: &data[..20],
            ..cloud.clone()
        };
        assert!(matches!(read_xyz(&short), Err(CloudError::TooShort { .. })));
        let no_z = CloudView {
            fields: cloud.fields[..2].to_vec(),
            ..cloud
        };
        assert_eq!(read_xyz(&no_z), Err(CloudError::MissingField("z")));
    }
}

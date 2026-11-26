//! A `visualization_msgs/Marker` as a line on the ground: the ghost's cylinder, the
//! barrier's keep-out circle. Only the shapes we publish; anything else is not drawn.

use std::f64::consts::TAU;

use map_core::geom::{Point, Pose};

/// How many straight pieces a circle is drawn with.
const CIRCLE_STEPS: u32 = 48;

/// The frame a message's points are in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Frame {
    Map,
    Odom,
    BaseLink,
}

impl Frame {
    /// From a header's `frame_id`; `None` for a frame the view can't place.
    #[must_use]
    pub fn from_id(id: &str) -> Option<Self> {
        match id.trim_start_matches('/') {
            "map" => Some(Self::Map),
            "odom" => Some(Self::Odom),
            "base_link" => Some(Self::BaseLink),
            _ => None,
        }
    }

    /// A point in this frame, on the map: from where the car is in odom and on the map.
    #[must_use]
    pub fn place(self, point: Point, car_odom: Pose, car_map: Pose) -> Point {
        match self {
            Self::Map => point,
            Self::BaseLink => turn_and_move(point, car_map.yaw, car_map.position),
            Self::Odom => {
                let from_car =
                    Point::new(point.x - car_odom.position.x, point.y - car_odom.position.y);
                turn_and_move(from_car, car_map.yaw - car_odom.yaw, car_map.position)
            }
        }
    }
}

fn turn_and_move(p: Point, yaw: f64, to: Point) -> Point {
    let (sin, cos) = yaw.sin_cos();
    Point::new(to.x + p.x * cos - p.y * sin, to.y + p.x * sin + p.y * cos)
}

/// A marker's shape (`Marker.type`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shape {
    Cube,
    Sphere,
    Cylinder,
    LineStrip,
    /// One we don't draw, with its number.
    Other(i32),
}

impl Shape {
    /// From `Marker.type`.
    #[must_use]
    pub fn from_code(code: i32) -> Self {
        match code {
            1 => Self::Cube,
            2 => Self::Sphere,
            3 => Self::Cylinder,
            4 => Self::LineStrip,
            other => Self::Other(other),
        }
    }
}

/// What a marker asks for (`Marker.action`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Draw it (ADD, which is also MODIFY).
    Add,
    /// Take it away (DELETE, DELETEALL).
    Delete,
}

impl Action {
    /// From `Marker.action`: 0 is ADD; 2 and 3 delete. Anything else draws nothing.
    #[must_use]
    pub fn from_code(code: i32) -> Self {
        if code == 0 {
            Self::Add
        } else {
            Self::Delete
        }
    }
}

/// The parts of a Marker message the view reads.
#[derive(Debug, Clone, PartialEq)]
pub struct MarkerParts<'a> {
    pub frame_id: &'a str,
    pub shape: Shape,
    pub action: Action,
    /// The marker's own pose in its frame.
    pub pose: Pose,
    /// `scale.x` and `scale.y`, metres: a cylinder's two diameters, a cube's sides.
    pub scale: (f64, f64),
    /// `points`, for a line strip: in the marker's pose.
    pub points: &'a [(f64, f64)],
    /// `color`, each 0 to 1.
    pub rgb: [f32; 3],
}

/// A marker as a line on the ground.
#[derive(Debug, Clone, PartialEq)]
pub struct Outline {
    pub frame: Frame,
    /// The line, in `frame`. A closed shape ends where it starts.
    pub points: Vec<Point>,
    /// Its middle, for a label.
    pub centre: Point,
    pub rgb: [u8; 3],
}

impl Outline {
    /// The line on the map.
    pub fn on_map(&self, car_odom: Pose, car_map: Pose) -> impl Iterator<Item = Point> + '_ {
        self.points
            .iter()
            .map(move |p| self.frame.place(*p, car_odom, car_map))
    }

    /// Its radius: the farthest point from the middle, metres.
    #[must_use]
    pub fn radius(&self) -> f64 {
        self.points
            .iter()
            .map(|p| (p.x - self.centre.x).hypot(p.y - self.centre.y))
            .fold(0.0, f64::max)
    }
}

/// The marker's outline; `None` when it is deleted, in a frame we can't place, of a
/// shape we don't draw, or empty.
#[must_use]
pub fn outline(marker: &MarkerParts<'_>) -> Option<Outline> {
    if marker.action == Action::Delete {
        return None;
    }
    let frame = Frame::from_id(marker.frame_id)?;
    let at =
        |x: f64, y: f64| turn_and_move(Point::new(x, y), marker.pose.yaw, marker.pose.position);
    let (sx, sy) = marker.scale;
    let points: Vec<Point> = match marker.shape {
        Shape::Cylinder | Shape::Sphere => (0..=CIRCLE_STEPS)
            .map(|i| {
                let a = TAU * f64::from(i) / f64::from(CIRCLE_STEPS);
                at(sx / 2.0 * a.cos(), sy / 2.0 * a.sin())
            })
            .collect(),
        Shape::Cube => [
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
            (-1.0, -1.0),
        ]
        .iter()
        .map(|&(x, y)| at(x * sx / 2.0, y * sy / 2.0))
        .collect(),
        Shape::LineStrip => marker.points.iter().map(|&(x, y)| at(x, y)).collect(),
        Shape::Other(_) => return None,
    };
    if points.len() < 2 || points.iter().any(|p| !(p.x.is_finite() && p.y.is_finite())) {
        return None;
    }
    // A closed line's last point repeats its first: leave it out of the middle.
    let body = &points[..points.len() - 1];
    let n = body.len() as f64;
    let centre = Point::new(
        body.iter().map(|p| p.x).sum::<f64>() / n,
        body.iter().map(|p| p.y).sum::<f64>() / n,
    );
    Some(Outline {
        frame,
        points,
        centre,
        rgb: marker
            .rgb
            .map(|c| (c.clamp(0.0, 1.0) * 255.0).round() as u8),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ghost() -> MarkerParts<'static> {
        MarkerParts {
            frame_id: "map",
            shape: Shape::Cylinder,
            action: Action::Add,
            pose: Pose::new(20.0, 1.0, 0.0),
            scale: (0.5, 0.5),
            points: &[],
            rgb: [0.7, 0.3, 1.0],
        }
    }

    #[test]
    fn the_ghosts_cylinder_is_a_closed_circle_of_its_width() {
        let line = outline(&ghost()).expect("a cylinder draws");
        assert_eq!(line.frame, Frame::Map);
        assert!((line.centre.x - 20.0).abs() < 1e-9 && (line.centre.y - 1.0).abs() < 1e-9);
        assert!((line.radius() - 0.25).abs() < 1e-9);
        let (first, last) = (line.points[0], line.points[line.points.len() - 1]);
        assert!((first.x - last.x).abs() < 1e-9 && (first.y - last.y).abs() < 1e-9);
        assert_eq!(line.rgb, [179, 77, 255]);
    }

    #[test]
    fn a_deleted_ghost_is_not_drawn() {
        let gone = MarkerParts {
            action: Action::from_code(2),
            ..ghost()
        };
        assert_eq!(outline(&gone), None);
        assert_eq!(Action::from_code(3), Action::Delete);
        assert_eq!(Action::from_code(0), Action::Add);
    }

    #[test]
    fn the_keep_out_circle_is_its_line_strip_as_sent() {
        let ring: Vec<(f64, f64)> = (0..=72)
            .map(|i| {
                let a = f64::from(i * 5).to_radians();
                (20.0 + 1.57 * a.cos(), 1.57 * a.sin())
            })
            .collect();
        let marker = MarkerParts {
            shape: Shape::from_code(4),
            pose: Pose::default(),
            points: &ring,
            ..ghost()
        };
        let line = outline(&marker).expect("a line strip draws");
        assert_eq!(line.points.len(), 73);
        assert!((line.radius() - 1.57).abs() < 1e-6);
        assert!((line.centre.x - 20.0).abs() < 1e-6);
    }

    #[test]
    fn an_unknown_frame_or_shape_is_not_drawn() {
        let lidar = MarkerParts {
            frame_id: "lidar_link",
            ..ghost()
        };
        assert_eq!(outline(&lidar), None);
        let arrow = MarkerParts {
            shape: Shape::from_code(0),
            ..ghost()
        };
        assert_eq!(outline(&arrow), None);
    }

    #[test]
    fn each_frame_lands_on_the_map() {
        let car_odom = Pose::new(5.0, 0.0, 0.0);
        let car_map = Pose::new(105.0, 50.0, std::f64::consts::FRAC_PI_2);
        let p = Point::new(7.0, 1.0);
        assert_eq!(Frame::Map.place(p, car_odom, car_map), p);
        // 2 m ahead of the car and 1 m to its left, in odom; the car faces north on the map.
        let odom = Frame::Odom.place(p, car_odom, car_map);
        assert!((odom.x - 104.0).abs() < 1e-9 && (odom.y - 52.0).abs() < 1e-9);
        let base = Frame::BaseLink.place(Point::new(2.0, 1.0), car_odom, car_map);
        assert!((base.x - 104.0).abs() < 1e-9 && (base.y - 52.0).abs() < 1e-9);
        assert_eq!(Frame::from_id("/odom"), Some(Frame::Odom));
    }
}

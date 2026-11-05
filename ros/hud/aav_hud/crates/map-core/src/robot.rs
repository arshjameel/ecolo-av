//! The car on the map as the little robot (`car_marker: robot`): hud-core's robot from
//! above, placed on the car's true outline and heading.
//!
//! The treads cover exactly the car's outline from `vehicle.yaml` (as the plain marker
//! does), so the position and the heading stay true: the robot's middle is the car's
//! middle, its eyes look the way the car faces. When the car would be smaller than
//! `min_length` on the screen, the robot grows about the car's middle, as the plain
//! marker turns into an arrow.

use hud_core::robot::{self, Piece, Xyz};
use serde::Deserialize;

use crate::{
    car::CarShape,
    geom::{Point, Pose},
};

/// How the map draws the car (`car_marker` in `config/map.yaml`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CarMarker {
    /// The car's outline to scale, with an arrow (the default).
    #[default]
    Outline,
    /// The little robot, from above, a little tilted (the easter egg).
    Robot,
}

impl CarMarker {
    /// Every marker, as the settings write them.
    pub const ALL: [CarMarker; 2] = [CarMarker::Outline, CarMarker::Robot];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Outline => "outline",
            Self::Robot => "robot",
        }
    }
}

impl std::str::FromStr for CarMarker {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|m| m.as_str() == s.trim())
            .ok_or_else(|| format!("no car marker called {s:?}: outline or robot"))
    }
}

/// The robot's pieces on the map, in drawing order (the back first): x and y in the
/// map frame (metres), z a height in metres, for the caller's tilt ([`robot::TILT`]).
///
/// `min_length`: metres; a car shorter than this is drawn this long, about its middle.
/// `travelled`: metres rolled ([`robot::Odometer`]), for the treads.
pub fn pieces(shape: &CarShape, pose: Pose, min_length: f64, travelled: f64) -> Vec<Piece<Xyz>> {
    let grow = (min_length / shape.length).max(1.0);
    let view = robot::top_view(shape.length, shape.width, travelled);
    let centre = shape.centre(pose);
    let place = |p: Xyz| {
        let q = centre + Point::new(p.x, p.y).rotated(pose.yaw) * grow;
        Xyz::new(q.x, q.y, p.z * grow)
    };
    // Screen y grows as map y falls: the tread with the higher map y is further away.
    let tread_y = |side: &[Piece<Xyz>]| {
        side.first()
            .map_or(0.0, |p| p.points().iter().map(|&q| place(q).y).sum::<f64>())
    };
    let [right, left] = view.treads;
    let (far, near) = if tread_y(&right) > tread_y(&left) {
        (right, left)
    } else {
        (left, right)
    };
    far.iter()
        .chain(&view.middle)
        .chain(&near)
        .map(|piece| piece.map(place, grow))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use hud_core::robot::{Form, Part};

    const CAR: CarShape = CarShape {
        length: 2.4,
        width: 1.2,
        rear_overhang: 0.4,
    };

    fn ground(pieces: &[Piece<Xyz>]) -> Vec<Point> {
        pieces
            .iter()
            .filter(|p| p.part == Part::Tread)
            .flat_map(Piece::points)
            .filter(|p| p.z == 0.0)
            .map(|p| Point::new(p.x, p.y))
            .collect()
    }

    #[test]
    fn the_robot_covers_the_cars_true_outline() {
        for yaw in [0.0, 0.7, 2.0, -2.9] {
            let pose = Pose::new(12.0, -4.0, yaw);
            let pieces = pieces(&CAR, pose, 0.0, 0.0);
            let feet = ground(&pieces);
            // Each corner of the car's outline is a corner of a tread, and every tread
            // corner lies within the outline.
            for corner in CAR.outline(pose) {
                assert!(
                    feet.iter().any(|f| f.distance(corner).0 < 1e-9),
                    "yaw {yaw}: corner {corner:?} not covered"
                );
            }
            let centre = CAR.centre(pose);
            let (fwd, left) = (
                Point::new(1.0, 0.0).rotated(yaw),
                Point::new(0.0, 1.0).rotated(yaw),
            );
            for f in &feet {
                let d = *f - centre;
                assert!(d.dot(fwd).abs() <= CAR.length / 2.0 + 1e-9);
                assert!(d.dot(left).abs() <= CAR.width / 2.0 + 1e-9);
            }
            // The middle of the footprint is the car's middle.
            let n = feet.len() as f64;
            let mid = feet.iter().fold(Point::default(), |a, &b| a + b) * (1.0 / n);
            assert!(mid.distance(centre).0 < 1e-9, "yaw {yaw}");
        }
    }

    #[test]
    fn the_eyes_look_the_way_the_car_faces() {
        for yaw in [0.0, 1.0, 3.0, -1.5] {
            let pose = Pose::new(0.0, 0.0, yaw);
            let pieces = pieces(&CAR, pose, 0.0, 0.0);
            let centre = CAR.centre(pose);
            let fwd = Point::new(1.0, 0.0).rotated(yaw);
            let eyes: Vec<Point> = pieces
                .iter()
                .filter(|p| p.part == Part::Eye)
                .map(|p| {
                    let c = p.points()[0];
                    Point::new(c.x, c.y)
                })
                .collect();
            assert_eq!(eyes.len(), 2);
            for e in eyes {
                assert!((e - centre).dot(fwd) > 0.0, "yaw {yaw}: an eye behind");
            }
        }
    }

    #[test]
    fn a_small_car_grows_about_its_middle() {
        let pose = Pose::new(5.0, 5.0, 0.3);
        let pieces = pieces(&CAR, pose, 4.8, 0.0);
        let feet = ground(&pieces);
        let centre = CAR.centre(pose);
        let far = feet
            .iter()
            .map(|f| f.distance(centre).0)
            .fold(0.0, f64::max);
        // Twice the size: the corners twice as far from the middle.
        let half_diag = (CAR.length / 2.0).hypot(CAR.width / 2.0);
        assert!((far - 2.0 * half_diag).abs() < 1e-9);
        let n = feet.len() as f64;
        let mid = feet.iter().fold(Point::default(), |a, &b| a + b) * (1.0 / n);
        assert!(mid.distance(centre).0 < 1e-9);
        // Radii grow too.
        let eye = pieces.iter().find(|p| p.part == Part::Eye).unwrap();
        let Form::Circle { radius, .. } = eye.form else {
            panic!("round");
        };
        assert!((radius - 2.0 * 0.19 * CAR.width).abs() < 1e-9);
    }

    #[test]
    fn the_far_tread_is_drawn_first() {
        // Facing east: the left tread is north, further up the screen, so first.
        let pieces = pieces(&CAR, Pose::new(0.0, 0.0, 0.0), 0.0, 0.0);
        assert!(pieces[0].points()[0].y > 0.0);
        let last = pieces.last().unwrap();
        assert!(last.points()[0].y < 0.0, "the near tread last");
    }

    #[test]
    fn markers_read() {
        assert_eq!("robot".parse::<CarMarker>(), Ok(CarMarker::Robot));
        assert_eq!("outline".parse::<CarMarker>(), Ok(CarMarker::Outline));
        assert!("wall-e".parse::<CarMarker>().is_err());
        assert_eq!(CarMarker::default(), CarMarker::Outline);
    }
}

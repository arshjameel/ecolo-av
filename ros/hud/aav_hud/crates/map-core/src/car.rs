//! The car's outline on the map, from `ros/bringup/aav_description/config/vehicle.yaml`: the one
//! place the car's measurements live. The program carries a copy of that file, taken
//! when it was built; `--vehicle FILE` reads another.

use serde::Deserialize;

use crate::geom::{Point, Pose};

/// `vehicle.yaml`, built in.
pub const BUILT_IN_YAML: &str =
    include_str!("../../../../../bringup/aav_description/config/vehicle.yaml");

/// The car's body, as seen from above. base_link is on the ground under the rear axle
/// (REP-105), so the body starts `rear_overhang` behind it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct CarShape {
    /// Bumper to bumper, metres.
    pub length: f64,
    /// Side to side, metres.
    pub width: f64,
    /// The rear bumper's distance behind the rear axle, metres.
    pub rear_overhang: f64,
}

#[derive(Deserialize)]
struct VehicleFile {
    vehicle: Vehicle,
}

#[derive(Deserialize)]
struct Vehicle {
    body: CarShape,
}

impl CarShape {
    /// Reads `vehicle: body:` from a vehicle.yaml.
    pub fn from_vehicle_yaml(text: &str) -> Result<Self, serde_yaml_ng::Error> {
        let file: VehicleFile = serde_yaml_ng::from_str(text)?;
        Ok(file.vehicle.body)
    }

    /// The four corners on the map, for the car at `pose`: rear right, front right,
    /// front left, rear left (counter-clockwise).
    pub fn outline(&self, pose: Pose) -> [Point; 4] {
        let (rear, front) = (-self.rear_overhang, self.length - self.rear_overhang);
        let half = self.width / 2.0;
        [
            Point::new(rear, -half),
            Point::new(front, -half),
            Point::new(front, half),
            Point::new(rear, half),
        ]
        .map(|p| pose.transform(p))
    }

    /// An arrow inside the body, pointing the way the car faces: tip, then the back
    /// corners (counter-clockwise).
    pub fn arrow(&self, pose: Pose) -> [Point; 3] {
        let front = self.length - self.rear_overhang;
        let (tip, back) = (front - 0.12 * self.length, front - 0.6 * self.length);
        let half = 0.3 * self.width;
        [
            Point::new(tip, 0.0),
            Point::new(back, half),
            Point::new(back, -half),
        ]
        .map(|p| pose.transform(p))
    }

    /// The middle of the body, on the map: what the view keeps centred.
    pub fn centre(&self, pose: Pose) -> Point {
        pose.transform(Point::new(self.length / 2.0 - self.rear_overhang, 0.0))
    }
}

impl Default for CarShape {
    /// The built-in vehicle.yaml's body.
    fn default() -> Self {
        Self::from_vehicle_yaml(BUILT_IN_YAML).expect("vehicle.yaml is checked by a unit test")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_vehicle_reads() {
        let c = CarShape::from_vehicle_yaml(BUILT_IN_YAML).expect("vehicle.yaml must parse");
        assert!(c.length > 0.5 && c.length < 5.0);
        assert!(c.width > 0.3 && c.width < 3.0);
        assert!(c.rear_overhang >= 0.0 && c.rear_overhang < c.length);
    }

    #[test]
    fn the_outline_sits_around_base_link() {
        let c = CarShape {
            length: 2.0,
            width: 1.0,
            rear_overhang: 0.5,
        };
        let o = c.outline(Pose::new(10.0, 0.0, 0.0));
        assert_eq!(o[0], Point::new(9.5, -0.5));
        assert_eq!(o[2], Point::new(11.5, 0.5));
        assert_eq!(c.centre(Pose::new(10.0, 0.0, 0.0)), Point::new(10.5, 0.0));
        // The arrow points forward: its tip is ahead of its back.
        let a = c.arrow(Pose::new(0.0, 0.0, 0.0));
        assert!(a[0].x > a[1].x && a[1].x == a[2].x);
    }
}

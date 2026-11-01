//! What the buttons need of the car, from `ros/bringup/aav_description/config/vehicle.yaml`
//! (the one place car measurements live): the wheelbase (a steering angle into a turn
//! rate), the steering limit, and which wheels `/vehicle/odom` measures (the stands rule,
//! TODO 49b). The program carries a copy, taken when it was built; `--vehicle FILE`
//! reads another (the launch file passes the installed one).

use std::fmt;

use serde::Deserialize;

/// `vehicle.yaml`, built in.
pub const BUILT_IN_YAML: &str =
    include_str!("../../../../../bringup/aav_description/config/vehicle.yaml");

/// Which wheels the speed comes from (vehicle.yaml `vehicle.speed_wheels`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(from = "String")]
pub enum SpeedWheels {
    Front,
    /// The driven wheels: the car is rear-wheel drive.
    Rear,
    /// Anything else, as written.
    Other(String),
}

impl From<String> for SpeedWheels {
    fn from(s: String) -> Self {
        match s.as_str() {
            "front" => Self::Front,
            "rear" => Self::Rear,
            _ => Self::Other(s),
        }
    }
}

impl SpeedWheels {
    /// The word vehicle.yaml uses.
    pub fn word(&self) -> &str {
        match self {
            Self::Front => "front",
            Self::Rear => "rear",
            Self::Other(s) => s,
        }
    }

    /// Whether these are the driven wheels (the rear: docs/learn-car.md).
    pub fn are_driven(&self) -> bool {
        *self == Self::Rear
    }
}

impl fmt::Display for SpeedWheels {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.word())
    }
}

/// The car, as the buttons see it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Vehicle {
    /// Rear axle to front axle, metres.
    pub wheelbase: f64,
    /// The virtual centre wheel's limit, radians, either way.
    pub max_steering_angle: f64,
    /// Which wheels `/vehicle/odom` measures; None if vehicle.yaml doesn't say.
    #[serde(default)]
    pub speed_wheels: Option<SpeedWheels>,
}

#[derive(Deserialize)]
struct VehicleFile {
    vehicle: Vehicle,
}

impl Vehicle {
    /// Reads `vehicle:` from a vehicle.yaml.
    pub fn from_vehicle_yaml(text: &str) -> Result<Self, serde_yaml_ng::Error> {
        let file: VehicleFile = serde_yaml_ng::from_str(text)?;
        Ok(file.vehicle)
    }
}

impl Default for Vehicle {
    /// The built-in vehicle.yaml's.
    fn default() -> Self {
        Self::from_vehicle_yaml(BUILT_IN_YAML).expect("vehicle.yaml is checked by a unit test")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_vehicle_reads() {
        let v = Vehicle::from_vehicle_yaml(BUILT_IN_YAML).expect("vehicle.yaml must parse");
        assert!(v.wheelbase > 0.5 && v.wheelbase < 5.0);
        assert!(v.max_steering_angle > 0.1 && v.max_steering_angle < 1.0);
        // Today the wheel sensors are on the front (TODO 49b): not the driven wheels.
        assert_eq!(v.speed_wheels, Some(SpeedWheels::Front));
        assert!(!SpeedWheels::Front.are_driven() && SpeedWheels::Rear.are_driven());
    }

    #[test]
    fn missing_speed_wheels_is_none() {
        let v =
            Vehicle::from_vehicle_yaml("vehicle:\n  wheelbase: 1.5\n  max_steering_angle: 0.5\n")
                .expect("parses");
        assert_eq!(v.speed_wheels, None);
    }
}

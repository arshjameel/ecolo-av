//! The hardware panel's settings: every topic name, and the buttons' steps and caps.
//!
//! They live in `config/hw_panel.yaml`, a ROS parameters file. The program carries a copy
//! of it as its defaults ([`PanelConfig::default`]), so no topic name is written in the
//! code. ROS parameters (`--ros-args -p gps_topic:=/x`) and `--config <file>` override it.

use std::time::Duration;

use serde::Deserialize;
use thiserror::Error;

/// `config/hw_panel.yaml`, built in.
pub const BUILT_IN_YAML: &str = include_str!("../../../config/hw_panel.yaml");

/// The node's name, and the top key of its parameters file.
pub const NODE_NAME: &str = "hw_panel";

/// Each topic the panel sends on or listens to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Topic {
    /// Sent: the buttons' command, into twist_mux.
    Buttons,
    /// Sent (true only) and read.
    Estop,
    Control,
    GeofenceLock,
    NoTimeoutLock,
    Source,
    VehicleStatus,
    Steering,
    Throttle,
    Brake,
    Wheels,
    Imu,
    Gps,
    Lidar,
    Radar,
    Camera,
    Battery,
}

impl Topic {
    /// Every topic.
    pub const ALL: [Topic; 17] = [
        Topic::Buttons,
        Topic::Estop,
        Topic::Control,
        Topic::GeofenceLock,
        Topic::NoTimeoutLock,
        Topic::Source,
        Topic::VehicleStatus,
        Topic::Steering,
        Topic::Throttle,
        Topic::Brake,
        Topic::Wheels,
        Topic::Imu,
        Topic::Gps,
        Topic::Lidar,
        Topic::Radar,
        Topic::Camera,
        Topic::Battery,
    ];

    /// The ROS parameter that holds this topic's name.
    pub fn parameter(self) -> &'static str {
        match self {
            Self::Buttons => "buttons_topic",
            Self::Estop => "estop_topic",
            Self::Control => "control_topic",
            Self::GeofenceLock => "geofence_lock_topic",
            Self::NoTimeoutLock => "no_timeout_lock_topic",
            Self::Source => "source_topic",
            Self::VehicleStatus => "vehicle_status_topic",
            Self::Steering => "steering_topic",
            Self::Throttle => "throttle_topic",
            Self::Brake => "brake_topic",
            Self::Wheels => "wheels_topic",
            Self::Imu => "imu_topic",
            Self::Gps => "gps_topic",
            Self::Lidar => "lidar_topic",
            Self::Radar => "radar_topic",
            Self::Camera => "camera_topic",
            Self::Battery => "battery_topic",
        }
    }
}

/// Each number in the settings: all must be above 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Number {
    StaleAfter,
    SpeedStep,
    SteerStep,
    Creep,
    SimCap,
    StandsCap,
}

impl Number {
    pub const ALL: [Number; 6] = [
        Number::StaleAfter,
        Number::SpeedStep,
        Number::SteerStep,
        Number::Creep,
        Number::SimCap,
        Number::StandsCap,
    ];

    pub fn parameter(self) -> &'static str {
        match self {
            Self::StaleAfter => "stale_after",
            Self::SpeedStep => "speed_step",
            Self::SteerStep => "steer_step",
            Self::Creep => "creep",
            Self::SimCap => "sim_cap",
            Self::StandsCap => "stands_cap",
        }
    }
}

/// The settings.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct PanelConfig {
    pub buttons_topic: String,
    pub estop_topic: String,
    pub control_topic: String,
    pub geofence_lock_topic: String,
    pub no_timeout_lock_topic: String,
    pub source_topic: String,
    pub vehicle_status_topic: String,
    pub steering_topic: String,
    pub throttle_topic: String,
    pub brake_topic: String,
    pub wheels_topic: String,
    pub imu_topic: String,
    pub gps_topic: String,
    pub lidar_topic: String,
    pub radar_topic: String,
    pub camera_topic: String,
    pub battery_topic: String,
    /// Seconds without a message before a topic shows STALE.
    pub stale_after: f64,
    /// m/s a press of Speed + or Speed -.
    pub speed_step: f64,
    /// rad a press of an arrow.
    pub steer_step: f64,
    /// m/s while CENTRE straightens the wheels.
    pub creep: f64,
    /// m/s: the most the buttons ask for in Gazebo or on the bench.
    pub sim_cap: f64,
    /// m/s: on stands.
    pub stands_cap: f64,
}

/// Why a settings file can't be used.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("not a hardware panel settings file (it needs hw_panel: ros__parameters: ...): {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    #[error("{0} must be a number above 0, not {1}")]
    Number(&'static str, f64),
    #[error("stands_cap ({stands}) must not be above sim_cap ({sim})")]
    Caps { stands: f64, sim: f64 },
}

#[derive(Deserialize)]
struct ParamsFile {
    hw_panel: NodeSection,
}

#[derive(Deserialize)]
struct NodeSection {
    #[serde(rename = "ros__parameters")]
    params: PanelConfig,
}

impl PanelConfig {
    /// Reads a ROS parameters file whose top key is `hw_panel`. Every setting must be there.
    pub fn from_yaml(text: &str) -> Result<Self, ConfigError> {
        let file: ParamsFile = serde_yaml_ng::from_str(text)?;
        let config = file.hw_panel.params;
        config.validate()?;
        Ok(config)
    }

    /// Checks the numbers. Call it after changing a setting by hand.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if let Some(n) = Number::ALL
            .into_iter()
            .find(|&n| !(self.number(n).is_finite() && self.number(n) > 0.0))
        {
            return Err(ConfigError::Number(n.parameter(), self.number(n)));
        }
        if self.stands_cap > self.sim_cap {
            return Err(ConfigError::Caps {
                stands: self.stands_cap,
                sim: self.sim_cap,
            });
        }
        Ok(())
    }

    /// The name of one topic.
    pub fn topic(&self, topic: Topic) -> &str {
        match topic {
            Topic::Buttons => &self.buttons_topic,
            Topic::Estop => &self.estop_topic,
            Topic::Control => &self.control_topic,
            Topic::GeofenceLock => &self.geofence_lock_topic,
            Topic::NoTimeoutLock => &self.no_timeout_lock_topic,
            Topic::Source => &self.source_topic,
            Topic::VehicleStatus => &self.vehicle_status_topic,
            Topic::Steering => &self.steering_topic,
            Topic::Throttle => &self.throttle_topic,
            Topic::Brake => &self.brake_topic,
            Topic::Wheels => &self.wheels_topic,
            Topic::Imu => &self.imu_topic,
            Topic::Gps => &self.gps_topic,
            Topic::Lidar => &self.lidar_topic,
            Topic::Radar => &self.radar_topic,
            Topic::Camera => &self.camera_topic,
            Topic::Battery => &self.battery_topic,
        }
    }

    /// Sets the name of one topic (from a ROS parameter).
    pub fn set_topic(&mut self, topic: Topic, name: String) {
        let slot = match topic {
            Topic::Buttons => &mut self.buttons_topic,
            Topic::Estop => &mut self.estop_topic,
            Topic::Control => &mut self.control_topic,
            Topic::GeofenceLock => &mut self.geofence_lock_topic,
            Topic::NoTimeoutLock => &mut self.no_timeout_lock_topic,
            Topic::Source => &mut self.source_topic,
            Topic::VehicleStatus => &mut self.vehicle_status_topic,
            Topic::Steering => &mut self.steering_topic,
            Topic::Throttle => &mut self.throttle_topic,
            Topic::Brake => &mut self.brake_topic,
            Topic::Wheels => &mut self.wheels_topic,
            Topic::Imu => &mut self.imu_topic,
            Topic::Gps => &mut self.gps_topic,
            Topic::Lidar => &mut self.lidar_topic,
            Topic::Radar => &mut self.radar_topic,
            Topic::Camera => &mut self.camera_topic,
            Topic::Battery => &mut self.battery_topic,
        };
        *slot = name;
    }

    pub fn number(&self, n: Number) -> f64 {
        match n {
            Number::StaleAfter => self.stale_after,
            Number::SpeedStep => self.speed_step,
            Number::SteerStep => self.steer_step,
            Number::Creep => self.creep,
            Number::SimCap => self.sim_cap,
            Number::StandsCap => self.stands_cap,
        }
    }

    pub fn set_number(&mut self, n: Number, value: f64) {
        let slot = match n {
            Number::StaleAfter => &mut self.stale_after,
            Number::SpeedStep => &mut self.speed_step,
            Number::SteerStep => &mut self.steer_step,
            Number::Creep => &mut self.creep,
            Number::SimCap => &mut self.sim_cap,
            Number::StandsCap => &mut self.stands_cap,
        };
        *slot = value;
    }

    /// [`PanelConfig::stale_after`] as a Duration.
    pub fn stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.stale_after)
    }
}

impl Default for PanelConfig {
    /// `config/hw_panel.yaml` as it was built in.
    fn default() -> Self {
        Self::from_yaml(BUILT_IN_YAML).expect("config/hw_panel.yaml is checked by a unit test")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_file_reads() {
        let c = PanelConfig::from_yaml(BUILT_IN_YAML).expect("config/hw_panel.yaml must parse");
        assert!(c.stale_after() > Duration::ZERO);
        for t in Topic::ALL {
            assert!(c.topic(t).starts_with('/'), "{t:?}: {}", c.topic(t));
        }
        // The design's numbers (docs/designs/2026-10-29_two-screens.md).
        assert_eq!((c.speed_step, c.steer_step), (0.1, 0.05));
        assert_eq!((c.sim_cap, c.stands_cap), (1.0, 0.3));
        assert_eq!(c.topic(Topic::Buttons), "/drivers/buttons/cmd_vel");
    }

    #[test]
    fn a_topic_and_a_number_can_be_changed() {
        let mut c = PanelConfig::default();
        c.set_topic(Topic::Gps, "/other/fix".into());
        assert_eq!(c.topic(Topic::Gps), "/other/fix");
        c.set_number(Number::SimCap, 0.5);
        assert_eq!(c.sim_cap, 0.5);
    }

    #[test]
    fn bad_files_are_refused() {
        assert!(matches!(
            PanelConfig::from_yaml("hud:\n  x: 1\n"),
            Err(ConfigError::Yaml(_))
        ));
        let bad = BUILT_IN_YAML.replace("speed_step: 0.1", "speed_step: -0.1");
        assert!(matches!(
            PanelConfig::from_yaml(&bad),
            Err(ConfigError::Number("speed_step", _))
        ));
        let bad = BUILT_IN_YAML.replace("stands_cap: 0.3", "stands_cap: 3.0");
        assert!(matches!(
            PanelConfig::from_yaml(&bad),
            Err(ConfigError::Caps { .. })
        ));
    }
}

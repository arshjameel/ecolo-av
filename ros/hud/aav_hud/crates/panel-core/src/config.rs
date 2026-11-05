//! The hardware panel's settings: every topic name, the buttons' steps and caps, and the
//! look.
//!
//! They live in `config/hw_panel.yaml`, a ROS parameters file. The program carries a copy
//! of it as its defaults ([`PanelConfig::default`]), so no topic name is written in the
//! code. ROS parameters (`--ros-args -p gps_topic:=/x`) and `--config <file>` override it.

use std::time::Duration;

use hud_core::{robot::EggSettings, theme::ThemeName};

use crate::{calibration::CalSettings, sweep::SweepSettings};
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
    /// Read: the steering actuator's raw counts and the firmware's calibration state.
    SteeringRaw,
    /// Sent: the calibration page's limp or jog, 10 a second while it asks.
    SteeringCalibration,
    /// Read: the bridge's one line about the calibration.
    CalibrationLine,
}

impl Topic {
    /// Every topic.
    pub const ALL: [Topic; 20] = [
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
        Topic::SteeringRaw,
        Topic::SteeringCalibration,
        Topic::CalibrationLine,
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
            Self::SteeringRaw => "steering_raw_topic",
            Self::SteeringCalibration => "steering_calibration_topic",
            Self::CalibrationLine => "calibration_line_topic",
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
    SweepRate,
    SweepStall,
    EasterEggAfter,
}

impl Number {
    pub const ALL: [Number; 9] = [
        Number::StaleAfter,
        Number::SpeedStep,
        Number::SteerStep,
        Number::Creep,
        Number::SimCap,
        Number::StandsCap,
        Number::SweepRate,
        Number::SweepStall,
        Number::EasterEggAfter,
    ];

    pub fn parameter(self) -> &'static str {
        match self {
            Self::StaleAfter => "stale_after",
            Self::SpeedStep => "speed_step",
            Self::SteerStep => "steer_step",
            Self::Creep => "creep",
            Self::SimCap => "sim_cap",
            Self::StandsCap => "stands_cap",
            Self::SweepRate => "sweep_rate",
            Self::SweepStall => "sweep_stall",
            Self::EasterEggAfter => "easter_egg_after",
        }
    }
}

/// Each count in the settings (the steering actuator's units, 0..4095): whole numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Count {
    JogStep,
    JogMin,
    JogMax,
    SweepMaxError,
}

impl Count {
    pub const ALL: [Count; 4] = [
        Count::JogStep,
        Count::JogMin,
        Count::JogMax,
        Count::SweepMaxError,
    ];

    pub fn parameter(self) -> &'static str {
        match self {
            Self::JogStep => "jog_step",
            Self::JogMin => "jog_min",
            Self::JogMax => "jog_max",
            Self::SweepMaxError => "sweep_max_error",
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
    pub steering_raw_topic: String,
    pub steering_calibration_topic: String,
    pub calibration_line_topic: String,
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
    /// Counts a STEP on the calibration page.
    pub jog_step: u16,
    /// Counts: the calibration page's jog targets stay inside these (the firmware's
    /// software limits).
    pub jog_min: u16,
    pub jog_max: u16,
    /// Counts a second: the lock-to-lock check's speed.
    pub sweep_rate: f64,
    /// Counts behind the target: resistance.
    pub sweep_max_error: u16,
    /// s without movement while the target moves: resistance.
    pub sweep_stall: f64,
    /// The colours (`hud_core::theme`), as the HUD's.
    pub theme: ThemeName,
    /// The little robot (hud-core's `robot`): its eyes peek after `easter_egg_after`
    /// seconds of quiet; it drives across at start-up and after a check passes.
    pub easter_egg: bool,
    /// Seconds without input, change or movement before the eyes peek.
    pub easter_egg_after: f64,
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
    #[error("{0} must be a count from 1 to 4095, not {1}")]
    Count(&'static str, i64),
    #[error("jog_min ({min}) must be below jog_max ({max})")]
    JogRange { min: u16, max: u16 },
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
        if let Some(c) = Count::ALL
            .into_iter()
            .find(|&c| !(1..=4095).contains(&self.count(c)))
        {
            return Err(ConfigError::Count(c.parameter(), i64::from(self.count(c))));
        }
        if self.jog_min >= self.jog_max {
            return Err(ConfigError::JogRange {
                min: self.jog_min,
                max: self.jog_max,
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
            Topic::SteeringRaw => &self.steering_raw_topic,
            Topic::SteeringCalibration => &self.steering_calibration_topic,
            Topic::CalibrationLine => &self.calibration_line_topic,
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
            Topic::SteeringRaw => &mut self.steering_raw_topic,
            Topic::SteeringCalibration => &mut self.steering_calibration_topic,
            Topic::CalibrationLine => &mut self.calibration_line_topic,
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
            Number::SweepRate => self.sweep_rate,
            Number::SweepStall => self.sweep_stall,
            Number::EasterEggAfter => self.easter_egg_after,
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
            Number::SweepRate => &mut self.sweep_rate,
            Number::SweepStall => &mut self.sweep_stall,
            Number::EasterEggAfter => &mut self.easter_egg_after,
        };
        *slot = value;
    }

    pub fn count(&self, c: Count) -> u16 {
        match c {
            Count::JogStep => self.jog_step,
            Count::JogMin => self.jog_min,
            Count::JogMax => self.jog_max,
            Count::SweepMaxError => self.sweep_max_error,
        }
    }

    /// Sets a count (from a ROS parameter, an i64): call [`PanelConfig::validate`] after.
    pub fn set_count(&mut self, c: Count, value: i64) -> Result<(), ConfigError> {
        let v = u16::try_from(value).map_err(|_| ConfigError::Count(c.parameter(), value))?;
        let slot = match c {
            Count::JogStep => &mut self.jog_step,
            Count::JogMin => &mut self.jog_min,
            Count::JogMax => &mut self.jog_max,
            Count::SweepMaxError => &mut self.sweep_max_error,
        };
        *slot = v;
        Ok(())
    }

    /// The little robot's settings.
    pub fn easter_egg_settings(&self) -> EggSettings {
        EggSettings {
            on: self.easter_egg,
            idle_after: Duration::from_secs_f64(self.easter_egg_after),
        }
    }

    /// The calibration page's numbers; the check's others are sweep's defaults.
    pub fn calibration(&self) -> CalSettings {
        CalSettings {
            jog_step: self.jog_step,
            jog_min: self.jog_min,
            jog_max: self.jog_max,
            sweep: SweepSettings {
                rate: self.sweep_rate,
                max_error: self.sweep_max_error,
                stall_after: Duration::from_secs_f64(self.sweep_stall),
                ..SweepSettings::default()
            },
        }
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
        assert_eq!(c.theme, ThemeName::InitialD);
        assert!(c.easter_egg);
        assert_eq!(c.easter_egg_settings().idle_after, Duration::from_secs(60));
        let cal = c.calibration();
        assert_eq!((cal.jog_step, cal.jog_min, cal.jog_max), (25, 400, 3500));
        assert!(cal.sweep.rate < 300.0, "slower than the firmware's own jog");
    }

    #[test]
    fn the_theme_can_change() {
        let day = BUILT_IN_YAML.replace("theme: initial_d", "theme: latte");
        let c = PanelConfig::from_yaml(&day).expect("latte is a theme");
        assert_eq!(c.theme, ThemeName::Latte);
        let bad = BUILT_IN_YAML.replace("theme: initial_d", "theme: purple");
        assert!(matches!(
            PanelConfig::from_yaml(&bad),
            Err(ConfigError::Yaml(_))
        ));
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
        let bad = BUILT_IN_YAML.replace("jog_max: 3500", "jog_max: 300");
        assert!(matches!(
            PanelConfig::from_yaml(&bad),
            Err(ConfigError::JogRange { .. })
        ));
        let bad = BUILT_IN_YAML.replace("jog_step: 25", "jog_step: 0");
        assert!(matches!(
            PanelConfig::from_yaml(&bad),
            Err(ConfigError::Count("jog_step", 0))
        ));
        let bad = BUILT_IN_YAML.replace("stands_cap: 0.3", "stands_cap: 3.0");
        assert!(matches!(
            PanelConfig::from_yaml(&bad),
            Err(ConfigError::Caps { .. })
        ));
    }
}

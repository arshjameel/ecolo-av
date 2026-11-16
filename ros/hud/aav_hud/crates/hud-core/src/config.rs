//! The HUD's settings: every topic name, when a topic counts as stale, the car's top
//! speed (the gauge's redline) and the colours.
//!
//! They live in `config/hud.yaml`, a ROS parameters file. The program carries a copy of
//! it as its defaults ([`HudConfig::default`]), so no topic name is written in the code.
//! ROS parameters (`--ros-args -p gps_topic:=/sensors/gps/fix`) and `--config <file>` override it.

use std::time::Duration;

use serde::Deserialize;
use thiserror::Error;

use crate::{
    gauge::{BadMaxSpeed, Scale},
    theme::ThemeName,
    units::MetresPerSecond,
};

/// `config/hud.yaml`, built in.
pub const BUILT_IN_YAML: &str = include_str!("../../../config/hud.yaml");

/// The node's name, and the top key of its parameters file.
pub const NODE_NAME: &str = "hud";

/// Each topic the HUD listens to or sends on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Topic {
    VehicleStatus,
    Gps,
    GpsFiltered,
    StopSign,
    Battery,
    Camera,
    Estop,
    /// `/system/source`: where the data comes from. Not listed under TOPICS: it's
    /// latched and sent once, so its age means nothing.
    Source,
    /// The sensor watchdog's verdict (aav_safety): a banner over the camera unless OK.
    Watchdog,
    /// The output mux's gate (aav_io_mux): OPEN, WAIT or STOP, in the GO / STOP panel.
    OutputMux,
    /// `/hud/heartbeat`, sent: the screen is alive (hud_core::heartbeat).
    Heartbeat,
    /// The squiggly line (hud_core::ahead): ego's trace, the bold line.
    Trace,
    /// Its faint line: the plan ahead, in base_link.
    PlanAhead,
    /// Its stop bar: where a full brake now stops the car.
    StopPoint,
    /// Its numbers: the gap, the room, the goal.
    Ahead,
    /// The things near the car (`/world/things`, odom), named by the camera: drawn in
    /// AHEAD (hud_core::things).
    Things,
    /// The car's pose in odom (the local EKF), to place the things in AHEAD.
    Odom,
}

impl Topic {
    /// The topics the HUD listens to, in the order the screen lists them.
    pub const LISTENED: [Topic; 6] = [
        Topic::VehicleStatus,
        Topic::Gps,
        Topic::GpsFiltered,
        Topic::StopSign,
        Topic::Battery,
        Topic::Camera,
    ];

    /// The topics that aren't listed: the e-stop and the heartbeat (sent), the source
    /// (latched), the watchdog (a banner of its own) and the output mux (its own panel).
    /// The squiggly line's four, the things and the car's pose have their own panel, AHEAD.
    pub const UNLISTED: [Topic; 11] = [
        Topic::Estop,
        Topic::Source,
        Topic::Watchdog,
        Topic::OutputMux,
        Topic::Heartbeat,
        Topic::Trace,
        Topic::PlanAhead,
        Topic::StopPoint,
        Topic::Ahead,
        Topic::Things,
        Topic::Odom,
    ];

    /// The ROS parameter that holds this topic's name.
    #[must_use]
    pub fn parameter(self) -> &'static str {
        match self {
            Self::VehicleStatus => "vehicle_status_topic",
            Self::Gps => "gps_topic",
            Self::GpsFiltered => "gps_filtered_topic",
            Self::StopSign => "stop_sign_topic",
            Self::Battery => "battery_topic",
            Self::Camera => "camera_topic",
            Self::Estop => "estop_topic",
            Self::Source => "source_topic",
            Self::Watchdog => "watchdog_topic",
            Self::OutputMux => "output_mux_topic",
            Self::Heartbeat => "heartbeat_topic",
            Self::Trace => "trace_topic",
            Self::PlanAhead => "plan_ahead_topic",
            Self::StopPoint => "stop_point_topic",
            Self::Ahead => "ahead_topic",
            Self::Things => "things_topic",
            Self::Odom => "odom_topic",
        }
    }
}

/// The settings.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct HudConfig {
    pub vehicle_status_topic: String,
    pub gps_topic: String,
    pub gps_filtered_topic: String,
    pub stop_sign_topic: String,
    pub battery_topic: String,
    pub camera_topic: String,
    pub estop_topic: String,
    pub source_topic: String,
    pub watchdog_topic: String,
    pub output_mux_topic: String,
    pub heartbeat_topic: String,
    pub trace_topic: String,
    pub plan_ahead_topic: String,
    pub stop_point_topic: String,
    pub ahead_topic: String,
    pub things_topic: String,
    pub odom_topic: String,
    /// Seconds without a message before a topic shows STALE.
    pub stale_after: f64,
    /// m/s: the car's forward cap, ego's `max_speed`. The arcade gauge's redline starts
    /// here and its scale fits it (`hud_core::gauge`). hud.launch.py sets it from
    /// aav_ego's ego.yaml.
    pub max_speed: f64,
    /// The colours (`hud_core::theme`).
    pub theme: ThemeName,
}

/// Why a settings file can't be used.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("not a HUD settings file (it needs hud: ros__parameters: ...): {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    #[error("stale_after must be a positive number of seconds, not {0}")]
    StaleAfter(f64),
    #[error(transparent)]
    MaxSpeed(#[from] BadMaxSpeed),
}

#[derive(Deserialize)]
struct ParamsFile {
    hud: NodeSection,
}

#[derive(Deserialize)]
struct NodeSection {
    #[serde(rename = "ros__parameters")]
    params: HudConfig,
}

impl HudConfig {
    /// Reads a ROS parameters file whose top key is `hud`. Every setting must be there.
    pub fn from_yaml(text: &str) -> Result<Self, ConfigError> {
        let file: ParamsFile = serde_yaml_ng::from_str(text)?;
        let config = file.hud.params;
        config.validate()?;
        Ok(config)
    }

    /// Checks the numbers. Call it after changing a setting by hand.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if !(self.stale_after.is_finite() && self.stale_after > 0.0) {
            return Err(ConfigError::StaleAfter(self.stale_after));
        }
        self.gauge()?;
        Ok(())
    }

    /// [`HudConfig::max_speed`] in its type.
    #[must_use]
    pub fn max_speed(&self) -> MetresPerSecond {
        MetresPerSecond(self.max_speed as f32)
    }

    /// The arcade gauge's scale, for [`HudConfig::max_speed`].
    pub fn gauge(&self) -> Result<Scale, BadMaxSpeed> {
        Scale::for_max_speed(self.max_speed())
    }

    /// The name of one topic.
    #[must_use]
    pub fn topic(&self, topic: Topic) -> &str {
        match topic {
            Topic::VehicleStatus => &self.vehicle_status_topic,
            Topic::Gps => &self.gps_topic,
            Topic::GpsFiltered => &self.gps_filtered_topic,
            Topic::StopSign => &self.stop_sign_topic,
            Topic::Battery => &self.battery_topic,
            Topic::Camera => &self.camera_topic,
            Topic::Estop => &self.estop_topic,
            Topic::Source => &self.source_topic,
            Topic::Watchdog => &self.watchdog_topic,
            Topic::OutputMux => &self.output_mux_topic,
            Topic::Heartbeat => &self.heartbeat_topic,
            Topic::Trace => &self.trace_topic,
            Topic::PlanAhead => &self.plan_ahead_topic,
            Topic::StopPoint => &self.stop_point_topic,
            Topic::Ahead => &self.ahead_topic,
            Topic::Things => &self.things_topic,
            Topic::Odom => &self.odom_topic,
        }
    }

    /// Sets the name of one topic (from a ROS parameter).
    pub fn set_topic(&mut self, topic: Topic, name: String) {
        let slot = match topic {
            Topic::VehicleStatus => &mut self.vehicle_status_topic,
            Topic::Gps => &mut self.gps_topic,
            Topic::GpsFiltered => &mut self.gps_filtered_topic,
            Topic::StopSign => &mut self.stop_sign_topic,
            Topic::Battery => &mut self.battery_topic,
            Topic::Camera => &mut self.camera_topic,
            Topic::Estop => &mut self.estop_topic,
            Topic::Source => &mut self.source_topic,
            Topic::Watchdog => &mut self.watchdog_topic,
            Topic::OutputMux => &mut self.output_mux_topic,
            Topic::Heartbeat => &mut self.heartbeat_topic,
            Topic::Trace => &mut self.trace_topic,
            Topic::PlanAhead => &mut self.plan_ahead_topic,
            Topic::StopPoint => &mut self.stop_point_topic,
            Topic::Ahead => &mut self.ahead_topic,
            Topic::Things => &mut self.things_topic,
            Topic::Odom => &mut self.odom_topic,
        };
        *slot = name;
    }

    /// [`HudConfig::stale_after`] as a Duration.
    #[must_use]
    pub fn stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.stale_after)
    }
}

impl Default for HudConfig {
    /// `config/hud.yaml` as it was built in.
    fn default() -> Self {
        Self::from_yaml(BUILT_IN_YAML).expect("config/hud.yaml is checked by a unit test")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_file_reads() {
        let c = HudConfig::from_yaml(BUILT_IN_YAML).expect("config/hud.yaml must parse");
        assert!(c.stale_after() > Duration::ZERO);
        assert_eq!(c.theme, ThemeName::InitialD);
        assert_eq!(c.max_speed(), MetresPerSecond(1.0));
        assert_eq!(c.gauge().map(|g| g.full_scale()), Ok(5.0));
        // Every name is absolute (docs/conventions.md), and none is empty.
        for t in Topic::LISTENED.into_iter().chain(Topic::UNLISTED) {
            assert!(c.topic(t).starts_with('/'), "{t:?}: {}", c.topic(t));
        }
    }

    #[test]
    fn a_topic_can_be_renamed() {
        let mut c = HudConfig::default();
        c.set_topic(Topic::Gps, "/sensors/gps/fix".into());
        assert_eq!(c.topic(Topic::Gps), "/sensors/gps/fix");
        assert_eq!(c.gps_topic, "/sensors/gps/fix");
    }

    #[test]
    fn bad_files_are_refused() {
        assert!(matches!(
            HudConfig::from_yaml("other:\n  x: 1\n"),
            Err(ConfigError::Yaml(_))
        ));
        let bad = BUILT_IN_YAML.replace("stale_after: 1.0", "stale_after: -1.0");
        assert!(matches!(
            HudConfig::from_yaml(&bad),
            Err(ConfigError::StaleAfter(_))
        ));
        let bad = BUILT_IN_YAML.replace("max_speed: 1.0", "max_speed: 0.0");
        assert!(matches!(
            HudConfig::from_yaml(&bad),
            Err(ConfigError::MaxSpeed(_))
        ));
        let bad = BUILT_IN_YAML.replace("theme: initial_d", "theme: purple");
        assert!(matches!(
            HudConfig::from_yaml(&bad),
            Err(ConfigError::Yaml(_))
        ));
        let light = BUILT_IN_YAML.replace("theme: initial_d", "theme: latte");
        let c = HudConfig::from_yaml(&light).expect("latte is a theme");
        assert_eq!(c.theme, ThemeName::Latte);
    }
}

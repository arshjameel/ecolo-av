//! The map window's settings: every topic name, its limits, and the look.
//!
//! They live in `config/map.yaml`, a ROS parameters file. The program carries a copy of
//! it as its defaults ([`MapConfig::default`]), so no topic name is written in the code.
//! ROS parameters (`--ros-args -p plan_topic:=/x`) and `--config <file>` override it.

use std::time::Duration;

use hud_core::theme::ThemeName;

use crate::robot::CarMarker;
use serde::Deserialize;
use thiserror::Error;

/// `config/map.yaml`, built in.
pub const BUILT_IN_YAML: &str = include_str!("../../../config/map.yaml");

/// The node's name, and the top key of its parameters file.
pub const NODE_NAME: &str = "map";

/// Each topic the map window listens to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Topic {
    /// Nav2's occupancy grid (latched).
    Map,
    /// The car's pose in the map frame, and its speed.
    Odom,
    /// Nav2's planned path.
    Plan,
    /// The goal: RViz's, or the place search's (the window publishes it too).
    Goal,
    /// `/system/source` (latched): the SIMULATED banner.
    Source,
    /// The sensor watchdog's verdict (aav_safety): a banner unless OK, as on the HUD.
    Watchdog,
    /// street_router's street route (aav_routes, latched): a suggestion, not Nav2's plan.
    StreetRoute,
    /// Who is driving (aav_mux_state): the debug page.
    MuxState,
    /// The command twist_mux passed on (`/control/cmd_vel`): the debug page.
    Command,
    /// Ego's command to the car (`/ego/actuator_cmd`): the debug page.
    Actuator,
    /// The output mux's gate (`/safety/output_mux/state`): the debug page.
    Gate,
    /// What the car says about itself (`/vehicle/status`): the debug page.
    Vehicle,
    /// Which algorithms the run uses (`/system/algorithms`, latched): the navigator's name.
    Algorithms,
}

impl Topic {
    /// Every topic, in the order the screen lists them.
    pub const ALL: [Topic; 13] = [
        Topic::Map,
        Topic::Odom,
        Topic::Plan,
        Topic::Goal,
        Topic::Source,
        Topic::Watchdog,
        Topic::StreetRoute,
        Topic::MuxState,
        Topic::Command,
        Topic::Actuator,
        Topic::Gate,
        Topic::Vehicle,
        Topic::Algorithms,
    ];

    /// The ROS parameter that holds this topic's name.
    #[must_use]
    pub fn parameter(self) -> &'static str {
        match self {
            Self::Map => "map_topic",
            Self::Odom => "odom_topic",
            Self::Plan => "plan_topic",
            Self::Goal => "goal_topic",
            Self::Source => "source_topic",
            Self::Watchdog => "watchdog_topic",
            Self::StreetRoute => "street_route_topic",
            Self::MuxState => "mux_state_topic",
            Self::Command => "command_topic",
            Self::Actuator => "actuator_topic",
            Self::Gate => "output_mux_topic",
            Self::Vehicle => "vehicle_status_topic",
            Self::Algorithms => "algorithms_topic",
        }
    }
}

/// The settings.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct MapConfig {
    /// Nav2's map (`/map`).
    pub map_topic: String,
    /// The car's odometry, in the map frame.
    pub odom_topic: String,
    /// Nav2's planned path.
    pub plan_topic: String,
    /// The goal: RViz's, and the place search's.
    pub goal_topic: String,
    /// `/system/source`: where the data comes from.
    pub source_topic: String,
    /// The sensor watchdog's verdict.
    pub watchdog_topic: String,
    /// street_router's street route.
    pub street_route_topic: String,
    /// Who is driving (the debug page).
    pub mux_state_topic: String,
    /// The command twist_mux passed on (the debug page).
    pub command_topic: String,
    /// Ego's command to the car (the debug page).
    pub actuator_topic: String,
    /// The output mux's gate (the debug page).
    pub output_mux_topic: String,
    /// The car's status (the debug page).
    pub vehicle_status_topic: String,
    /// The run's algorithms (the debug page: the navigator's follower).
    pub algorithms_topic: String,
    /// Seconds without a position before the car shows STALE.
    pub stale_after: f64,
    /// Seconds without a new plan before the trip shows NO NEW PLAN.
    pub plan_stale_after: f64,
    /// Metres from the plan's end in which Nav2 keeps its path instead of replanning.
    pub final_approach: f64,
    /// Seconds without a new plan, within `final_approach`, before NO NEW PLAN.
    pub plan_kept_for: f64,
    /// Metres from the goal that count as arrived.
    pub arrive_within: f64,
    /// The street map's file (PMTiles), relative to the folder the window starts in.
    pub street_map: String,
    /// The place search's file (SQLite, `ottawa.db`), found the same way.
    pub places: String,
    /// The colours (`hud_core::theme`), as the HUD's.
    pub theme: ThemeName,
    /// Frosted glass: the map blurred behind the boxes drawn over it.
    pub blur: bool,
    /// How the car is drawn: its outline (the default) or the little robot.
    pub car_marker: CarMarker,
}

/// Why a settings file can't be used.
#[derive(Debug, Error)]
pub enum ConfigError {
    /// The text isn't a map parameters file.
    #[error("not a map settings file (it needs map: ros__parameters: ...): {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    /// A setting that must be positive isn't.
    #[error("{name} must be a positive number, not {value}")]
    NotPositive {
        /// The setting.
        name: &'static str,
        /// Its value.
        value: f64,
    },
}

#[derive(Deserialize)]
struct ParamsFile {
    map: NodeSection,
}

#[derive(Deserialize)]
struct NodeSection {
    #[serde(rename = "ros__parameters")]
    params: MapConfig,
}

impl MapConfig {
    /// Reads a ROS parameters file whose top key is `map`. Every setting must be there.
    ///
    /// # Errors
    ///
    /// [`ConfigError`] if the text isn't a map parameters file, or a number isn't positive.
    pub fn from_yaml(text: &str) -> Result<Self, ConfigError> {
        let file: ParamsFile = serde_yaml_ng::from_str(text)?;
        let config = file.map.params;
        config.validate()?;
        Ok(config)
    }

    /// Checks the numbers. Call it after changing a setting by hand.
    ///
    /// # Errors
    ///
    /// [`ConfigError::NotPositive`] for the first number that isn't positive.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let numbers = [
            ("stale_after", self.stale_after),
            ("plan_stale_after", self.plan_stale_after),
            ("final_approach", self.final_approach),
            ("plan_kept_for", self.plan_kept_for),
            ("arrive_within", self.arrive_within),
        ];
        match numbers
            .into_iter()
            .find(|(_, v)| !(v.is_finite() && *v > 0.0))
        {
            Some((name, value)) => Err(ConfigError::NotPositive { name, value }),
            None => Ok(()),
        }
    }

    /// The name of one topic.
    #[must_use]
    pub fn topic(&self, topic: Topic) -> &str {
        match topic {
            Topic::Map => &self.map_topic,
            Topic::Odom => &self.odom_topic,
            Topic::Plan => &self.plan_topic,
            Topic::Goal => &self.goal_topic,
            Topic::Source => &self.source_topic,
            Topic::Watchdog => &self.watchdog_topic,
            Topic::StreetRoute => &self.street_route_topic,
            Topic::MuxState => &self.mux_state_topic,
            Topic::Command => &self.command_topic,
            Topic::Actuator => &self.actuator_topic,
            Topic::Gate => &self.output_mux_topic,
            Topic::Vehicle => &self.vehicle_status_topic,
            Topic::Algorithms => &self.algorithms_topic,
        }
    }

    /// Sets the name of one topic (from a ROS parameter).
    pub fn set_topic(&mut self, topic: Topic, name: String) {
        let slot = match topic {
            Topic::Map => &mut self.map_topic,
            Topic::Odom => &mut self.odom_topic,
            Topic::Plan => &mut self.plan_topic,
            Topic::Goal => &mut self.goal_topic,
            Topic::Source => &mut self.source_topic,
            Topic::Watchdog => &mut self.watchdog_topic,
            Topic::StreetRoute => &mut self.street_route_topic,
            Topic::MuxState => &mut self.mux_state_topic,
            Topic::Command => &mut self.command_topic,
            Topic::Actuator => &mut self.actuator_topic,
            Topic::Gate => &mut self.output_mux_topic,
            Topic::Vehicle => &mut self.vehicle_status_topic,
            Topic::Algorithms => &mut self.algorithms_topic,
        };
        *slot = name;
    }

    /// [`MapConfig::stale_after`] as a Duration.
    #[must_use]
    pub fn stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.stale_after)
    }

    /// [`MapConfig::plan_stale_after`] as a Duration.
    #[must_use]
    pub fn plan_stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.plan_stale_after)
    }

    /// [`MapConfig::plan_kept_for`] as a Duration.
    #[must_use]
    pub fn plan_kept_for(&self) -> Duration {
        Duration::from_secs_f64(self.plan_kept_for)
    }
}

impl Default for MapConfig {
    /// `config/map.yaml` as it was built in.
    fn default() -> Self {
        Self::from_yaml(BUILT_IN_YAML).expect("config/map.yaml is checked by a unit test")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_file_reads() {
        let c = MapConfig::from_yaml(BUILT_IN_YAML).expect("config/map.yaml must parse");
        assert!(c.stale_after() > Duration::ZERO);
        assert!(c.plan_stale_after() > c.stale_after());
        assert!(c.plan_kept_for() > c.plan_stale_after());
        assert!(c.final_approach > c.arrive_within);
        assert_eq!(c.theme, ThemeName::InitialD);
        assert!(c.blur);
        assert_eq!(c.car_marker, CarMarker::Outline);
        // Every name is absolute (docs/conventions.md), and none is empty.
        for t in Topic::ALL {
            assert!(c.topic(t).starts_with('/'), "{t:?}: {}", c.topic(t));
        }
    }

    #[test]
    fn a_topic_can_be_renamed() {
        let mut c = MapConfig::default();
        c.set_topic(Topic::Plan, "/other/plan".into());
        assert_eq!(c.topic(Topic::Plan), "/other/plan");
        assert_eq!(c.plan_topic, "/other/plan");
    }

    #[test]
    fn bad_files_are_refused() {
        assert!(matches!(
            MapConfig::from_yaml("hud:\n  x: 1\n"),
            Err(ConfigError::Yaml(_))
        ));
        let bad = BUILT_IN_YAML.replace("arrive_within: 1.0", "arrive_within: 0.0");
        assert!(matches!(
            MapConfig::from_yaml(&bad),
            Err(ConfigError::NotPositive {
                name: "arrive_within",
                ..
            })
        ));
        let bad = BUILT_IN_YAML.replace("theme: initial_d", "theme: purple");
        assert!(matches!(
            MapConfig::from_yaml(&bad),
            Err(ConfigError::Yaml(_))
        ));
    }

    #[test]
    fn the_look_can_change() {
        let day = BUILT_IN_YAML
            .replace("theme: initial_d", "theme: latte")
            .replace("blur: true", "blur: false");
        let c = MapConfig::from_yaml(&day).expect("latte, no blur");
        assert_eq!(c.theme, ThemeName::Latte);
        assert!(!c.blur);
        let robot = BUILT_IN_YAML.replace("car_marker: outline", "car_marker: robot");
        let c = MapConfig::from_yaml(&robot).expect("the little robot");
        assert_eq!(c.car_marker, CarMarker::Robot);
        let bad = BUILT_IN_YAML.replace("car_marker: outline", "car_marker: tank");
        assert!(MapConfig::from_yaml(&bad).is_err());
    }
}

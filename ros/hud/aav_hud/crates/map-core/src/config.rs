//! The map window's settings: every topic name, and its limits.
//!
//! They live in `config/map.yaml`, a ROS parameters file. The program carries a copy of
//! it as its defaults ([`MapConfig::default`]), so no topic name is written in the code.
//! ROS parameters (`--ros-args -p plan_topic:=/x`) and `--config <file>` override it.

use std::time::Duration;

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
}

impl Topic {
    /// Every topic, in the order the screen lists them.
    pub const ALL: [Topic; 7] = [
        Topic::Map,
        Topic::Odom,
        Topic::Plan,
        Topic::Goal,
        Topic::Source,
        Topic::Watchdog,
        Topic::StreetRoute,
    ];

    /// The ROS parameter that holds this topic's name.
    pub fn parameter(self) -> &'static str {
        match self {
            Self::Map => "map_topic",
            Self::Odom => "odom_topic",
            Self::Plan => "plan_topic",
            Self::Goal => "goal_topic",
            Self::Source => "source_topic",
            Self::Watchdog => "watchdog_topic",
            Self::StreetRoute => "street_route_topic",
        }
    }
}

/// The settings.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct MapConfig {
    pub map_topic: String,
    pub odom_topic: String,
    pub plan_topic: String,
    pub goal_topic: String,
    pub source_topic: String,
    pub watchdog_topic: String,
    pub street_route_topic: String,
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
}

/// Why a settings file can't be used.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("not a map settings file (it needs map: ros__parameters: ...): {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    #[error("{name} must be a positive number, not {value}")]
    NotPositive { name: &'static str, value: f64 },
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
    pub fn from_yaml(text: &str) -> Result<Self, ConfigError> {
        let file: ParamsFile = serde_yaml_ng::from_str(text)?;
        let config = file.map.params;
        config.validate()?;
        Ok(config)
    }

    /// Checks the numbers. Call it after changing a setting by hand.
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
    pub fn topic(&self, topic: Topic) -> &str {
        match topic {
            Topic::Map => &self.map_topic,
            Topic::Odom => &self.odom_topic,
            Topic::Plan => &self.plan_topic,
            Topic::Goal => &self.goal_topic,
            Topic::Source => &self.source_topic,
            Topic::Watchdog => &self.watchdog_topic,
            Topic::StreetRoute => &self.street_route_topic,
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
        };
        *slot = name;
    }

    /// [`MapConfig::stale_after`] as a Duration.
    pub fn stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.stale_after)
    }

    /// [`MapConfig::plan_stale_after`] as a Duration.
    pub fn plan_stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.plan_stale_after)
    }

    /// [`MapConfig::plan_kept_for`] as a Duration.
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
    }
}

//! The debug HUD's settings: every topic name, the frames, and its limits.
//!
//! They live in `config/debug_hud.yaml`, a ROS parameters file. The program carries a
//! copy of it as its defaults ([`DebugConfig::default`]), so no topic name is written in
//! the code. ROS parameters (`--ros-args -p lidar_topic:=/x`) and `--config <file>`
//! override it.

use std::time::Duration;

use serde::Deserialize;
use thiserror::Error;

use crate::cubes::{CubeSettings, Metres};

/// `config/debug_hud.yaml`, built in.
pub const BUILT_IN_YAML: &str = include_str!("../../../config/debug_hud.yaml");

/// The node's name, and the top key of its parameters file.
pub const NODE_NAME: &str = "debug_hud";

/// Each topic the debug HUD listens to. It publishes nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Topic {
    Lidar,
    LocalOdom,
    GlobalOdom,
    Truth,
    Amcl,
    AmclParticles,
    NdtPose,
    NdtStatus,
    Plan,
    Tf,
    TfStatic,
    Imu,
    Gps,
    Wheels,
    WheelOdom,
    VehicleStatus,
    TrackStatus,
    Source,
}

impl Topic {
    /// Every topic.
    pub const ALL: [Topic; 18] = [
        Topic::Lidar,
        Topic::LocalOdom,
        Topic::GlobalOdom,
        Topic::Truth,
        Topic::Amcl,
        Topic::AmclParticles,
        Topic::NdtPose,
        Topic::NdtStatus,
        Topic::Plan,
        Topic::Tf,
        Topic::TfStatic,
        Topic::Imu,
        Topic::Gps,
        Topic::Wheels,
        Topic::WheelOdom,
        Topic::VehicleStatus,
        Topic::TrackStatus,
        Topic::Source,
    ];

    /// The ROS parameter that holds this topic's name.
    pub fn parameter(self) -> &'static str {
        match self {
            Self::Lidar => "lidar_topic",
            Self::LocalOdom => "local_odom_topic",
            Self::GlobalOdom => "global_odom_topic",
            Self::Truth => "truth_topic",
            Self::Amcl => "amcl_topic",
            Self::AmclParticles => "amcl_particles_topic",
            Self::NdtPose => "ndt_pose_topic",
            Self::NdtStatus => "ndt_status_topic",
            Self::Plan => "plan_topic",
            Self::Tf => "tf_topic",
            Self::TfStatic => "tf_static_topic",
            Self::Imu => "imu_topic",
            Self::Gps => "gps_topic",
            Self::Wheels => "wheels_topic",
            Self::WheelOdom => "wheel_odom_topic",
            Self::VehicleStatus => "vehicle_status_topic",
            Self::TrackStatus => "track_status_topic",
            Self::Source => "source_topic",
        }
    }
}

/// The frames, as REP-105 names them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Frame {
    Odom,
    Base,
    Map,
}

impl Frame {
    pub const ALL: [Frame; 3] = [Frame::Odom, Frame::Base, Frame::Map];

    /// The ROS parameter that holds this frame's name.
    pub fn parameter(self) -> &'static str {
        match self {
            Self::Odom => "odom_frame",
            Self::Base => "base_frame",
            Self::Map => "map_frame",
        }
    }
}

/// The numbers, each a ROS parameter of the same name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Number {
    StaleAfter,
    CubeSize,
    PoseWindow,
    GroundHeight,
    KeepCubesFor,
    NdtMapCell,
}

impl Number {
    pub const ALL: [Number; 6] = [
        Number::StaleAfter,
        Number::CubeSize,
        Number::PoseWindow,
        Number::GroundHeight,
        Number::KeepCubesFor,
        Number::NdtMapCell,
    ];

    pub fn parameter(self) -> &'static str {
        match self {
            Self::StaleAfter => "stale_after",
            Self::CubeSize => "cube_size",
            Self::PoseWindow => "pose_window",
            Self::GroundHeight => "ground_height",
            Self::KeepCubesFor => "keep_cubes_for",
            Self::NdtMapCell => "ndt_map_cell",
        }
    }
}

/// The settings.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct DebugConfig {
    pub lidar_topic: String,
    pub local_odom_topic: String,
    pub global_odom_topic: String,
    pub truth_topic: String,
    pub amcl_topic: String,
    pub amcl_particles_topic: String,
    pub ndt_pose_topic: String,
    pub ndt_status_topic: String,
    pub plan_topic: String,
    pub tf_topic: String,
    pub tf_static_topic: String,
    pub imu_topic: String,
    pub gps_topic: String,
    pub wheels_topic: String,
    pub wheel_odom_topic: String,
    pub vehicle_status_topic: String,
    pub track_status_topic: String,
    pub source_topic: String,
    /// aav_track's parameter service, for keys 1-4 (sim only).
    pub track_parameters_service: String,
    /// NDT's map file (a PCD), for key M; relative to where the program starts.
    pub ndt_map: String,
    pub odom_frame: String,
    pub base_frame: String,
    pub map_frame: String,
    /// Seconds without a message before a topic shows STALE.
    pub stale_after: f64,
    /// One cube's edge, metres.
    pub cube_size: f64,
    /// The most cubes drawn; the nearest are kept.
    pub max_cubes: u32,
    /// Seconds of local EKF poses kept, to place a cloud at its own stamp.
    pub pose_window: f64,
    /// Metres above base_link: lower cubes are ground.
    pub ground_height: f64,
    /// Seconds (by the clouds' stamps) a kept cube fades over before it goes.
    pub keep_cubes_for: f64,
    /// One NDT map cube's edge, metres.
    pub ndt_map_cell: f64,
}

/// Why a settings file can't be used.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("not a debug_hud settings file (it needs debug_hud: ros__parameters: ...): {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    #[error("{name} must be a positive number, not {value}")]
    NotPositive { name: &'static str, value: f64 },
    #[error("max_cubes must be at least 1")]
    NoCubes,
    #[error("{name} must not be empty")]
    Empty { name: &'static str },
}

#[derive(Deserialize)]
struct ParamsFile {
    debug_hud: NodeSection,
}

#[derive(Deserialize)]
struct NodeSection {
    #[serde(rename = "ros__parameters")]
    params: DebugConfig,
}

impl DebugConfig {
    /// Reads a ROS parameters file whose top key is `debug_hud`. Every setting must be there.
    pub fn from_yaml(text: &str) -> Result<Self, ConfigError> {
        let file: ParamsFile = serde_yaml_ng::from_str(text)?;
        let config = file.debug_hud.params;
        config.validate()?;
        Ok(config)
    }

    /// Checks the numbers and names. Call it after changing a setting by hand.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if let Some(n) = Number::ALL
            .into_iter()
            .find(|&n| !(self.number(n).is_finite() && self.number(n) > 0.0))
        {
            return Err(ConfigError::NotPositive {
                name: n.parameter(),
                value: self.number(n),
            });
        }
        if self.max_cubes == 0 {
            return Err(ConfigError::NoCubes);
        }
        let names = Topic::ALL
            .into_iter()
            .map(|t| (t.parameter(), self.topic(t)))
            .chain(
                Frame::ALL
                    .into_iter()
                    .map(|f| (f.parameter(), self.frame(f))),
            );
        for (name, value) in names {
            if value.trim().is_empty() {
                return Err(ConfigError::Empty { name });
            }
        }
        Ok(())
    }

    /// The name of one topic.
    pub fn topic(&self, topic: Topic) -> &str {
        match topic {
            Topic::Lidar => &self.lidar_topic,
            Topic::LocalOdom => &self.local_odom_topic,
            Topic::GlobalOdom => &self.global_odom_topic,
            Topic::Truth => &self.truth_topic,
            Topic::Amcl => &self.amcl_topic,
            Topic::AmclParticles => &self.amcl_particles_topic,
            Topic::NdtPose => &self.ndt_pose_topic,
            Topic::NdtStatus => &self.ndt_status_topic,
            Topic::Plan => &self.plan_topic,
            Topic::Tf => &self.tf_topic,
            Topic::TfStatic => &self.tf_static_topic,
            Topic::Imu => &self.imu_topic,
            Topic::Gps => &self.gps_topic,
            Topic::Wheels => &self.wheels_topic,
            Topic::WheelOdom => &self.wheel_odom_topic,
            Topic::VehicleStatus => &self.vehicle_status_topic,
            Topic::TrackStatus => &self.track_status_topic,
            Topic::Source => &self.source_topic,
        }
    }

    /// Sets the name of one topic (from a ROS parameter).
    pub fn set_topic(&mut self, topic: Topic, name: String) {
        let slot = match topic {
            Topic::Lidar => &mut self.lidar_topic,
            Topic::LocalOdom => &mut self.local_odom_topic,
            Topic::GlobalOdom => &mut self.global_odom_topic,
            Topic::Truth => &mut self.truth_topic,
            Topic::Amcl => &mut self.amcl_topic,
            Topic::AmclParticles => &mut self.amcl_particles_topic,
            Topic::NdtPose => &mut self.ndt_pose_topic,
            Topic::NdtStatus => &mut self.ndt_status_topic,
            Topic::Plan => &mut self.plan_topic,
            Topic::Tf => &mut self.tf_topic,
            Topic::TfStatic => &mut self.tf_static_topic,
            Topic::Imu => &mut self.imu_topic,
            Topic::Gps => &mut self.gps_topic,
            Topic::Wheels => &mut self.wheels_topic,
            Topic::WheelOdom => &mut self.wheel_odom_topic,
            Topic::VehicleStatus => &mut self.vehicle_status_topic,
            Topic::TrackStatus => &mut self.track_status_topic,
            Topic::Source => &mut self.source_topic,
        };
        *slot = name;
    }

    /// The name of one frame.
    pub fn frame(&self, frame: Frame) -> &str {
        match frame {
            Frame::Odom => &self.odom_frame,
            Frame::Base => &self.base_frame,
            Frame::Map => &self.map_frame,
        }
    }

    /// Sets the name of one frame (from a ROS parameter).
    pub fn set_frame(&mut self, frame: Frame, name: String) {
        let slot = match frame {
            Frame::Odom => &mut self.odom_frame,
            Frame::Base => &mut self.base_frame,
            Frame::Map => &mut self.map_frame,
        };
        *slot = name;
    }

    /// One number.
    pub fn number(&self, n: Number) -> f64 {
        match n {
            Number::StaleAfter => self.stale_after,
            Number::CubeSize => self.cube_size,
            Number::PoseWindow => self.pose_window,
            Number::GroundHeight => self.ground_height,
            Number::KeepCubesFor => self.keep_cubes_for,
            Number::NdtMapCell => self.ndt_map_cell,
        }
    }

    /// Sets one number (from a ROS parameter).
    pub fn set_number(&mut self, n: Number, value: f64) {
        let slot = match n {
            Number::StaleAfter => &mut self.stale_after,
            Number::CubeSize => &mut self.cube_size,
            Number::PoseWindow => &mut self.pose_window,
            Number::GroundHeight => &mut self.ground_height,
            Number::KeepCubesFor => &mut self.keep_cubes_for,
            Number::NdtMapCell => &mut self.ndt_map_cell,
        };
        *slot = value;
    }

    /// [`DebugConfig::stale_after`] as a Duration.
    pub fn stale_after(&self) -> Duration {
        Duration::from_secs_f64(self.stale_after)
    }

    /// [`DebugConfig::pose_window`] as a Duration.
    pub fn pose_window(&self) -> Duration {
        Duration::from_secs_f64(self.pose_window)
    }

    /// [`DebugConfig::keep_cubes_for`] as a Duration.
    pub fn keep_cubes_for(&self) -> Duration {
        Duration::from_secs_f64(self.keep_cubes_for)
    }

    /// What the cube grid needs.
    pub fn cube_settings(&self) -> CubeSettings {
        CubeSettings {
            size: Metres(self.cube_size),
            max_cubes: usize::try_from(self.max_cubes).unwrap_or(usize::MAX),
            ground_height: Metres(self.ground_height),
        }
    }
}

impl Default for DebugConfig {
    /// `config/debug_hud.yaml` as it was built in.
    fn default() -> Self {
        Self::from_yaml(BUILT_IN_YAML).expect("config/debug_hud.yaml is checked by a unit test")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_built_in_file_reads() {
        let c = DebugConfig::from_yaml(BUILT_IN_YAML).expect("config/debug_hud.yaml must parse");
        assert!(c.stale_after() > Duration::ZERO);
        assert!(c.pose_window() >= Duration::from_millis(500));
        for t in Topic::ALL {
            assert!(c.topic(t).starts_with('/'), "{t:?}: {}", c.topic(t));
        }
        assert_eq!(c.frame(Frame::Base), "base_link");
        assert_eq!(c.cube_settings().max_cubes, 20_000);
        assert_eq!(c.keep_cubes_for(), Duration::from_secs(2));
        assert!(c.ndt_map.ends_with(".pcd"));
    }

    #[test]
    fn a_topic_and_a_number_can_be_changed() {
        let mut c = DebugConfig::default();
        c.set_topic(Topic::Lidar, "/other/points".into());
        assert_eq!(c.topic(Topic::Lidar), "/other/points");
        c.set_number(Number::CubeSize, 0.5);
        assert_eq!(c.cube_settings().size, Metres(0.5));
        c.set_frame(Frame::Map, "earth".into());
        assert_eq!(c.map_frame, "earth");
    }

    #[test]
    fn bad_files_are_refused() {
        assert!(matches!(
            DebugConfig::from_yaml("hud:\n  x: 1\n"),
            Err(ConfigError::Yaml(_))
        ));
        let bad = BUILT_IN_YAML.replace("cube_size: 0.2", "cube_size: 0.0");
        assert!(matches!(
            DebugConfig::from_yaml(&bad),
            Err(ConfigError::NotPositive {
                name: "cube_size",
                ..
            })
        ));
        let bad = BUILT_IN_YAML.replace("max_cubes: 20000", "max_cubes: 0");
        assert!(matches!(
            DebugConfig::from_yaml(&bad),
            Err(ConfigError::NoCubes)
        ));
        let bad = BUILT_IN_YAML.replace("odom_frame: odom", "odom_frame: ''");
        assert!(matches!(
            DebugConfig::from_yaml(&bad),
            Err(ConfigError::Empty { name: "odom_frame" })
        ));
    }
}

//! What the two mux nodes share: parameters as rclpy declares them, the steady clock, and
//! the ROS graph questions (who publishes /system/source; who publishes a sensor topic).

use std::any::Any;
use std::collections::HashSet;
use std::sync::{Arc, OnceLock};
use std::time::Instant;

use anyhow::{bail, Context as _, Result};
use io_mux_core::provenance::Entry;
use io_mux_core::provenance::Kind;
use rclrs::{Node, ParameterValue};
use ros_env::aav_interfaces::msg::DataSource;

/// /system/source, as the announcers publish it: reliable, transient local, deep enough
/// to keep every announcer's one message (the Python's `ANNOUNCEMENTS`).
pub const SOURCE_TOPIC: &str = "/system/source";
pub const SOURCE_DEPTH: u32 = 50;

/// s on a steady clock (the Python's `time.monotonic()`): never simulated time.
#[must_use]
pub fn steady_now() -> f64 {
    static START: OnceLock<Instant> = OnceLock::new();
    // Not zero at start: the Python's monotonic clock never is either, and no rule
    // gives zero a meaning.
    1000.0 + START.get_or_init(Instant::now).elapsed().as_secs_f64()
}

/// A DataSource message's entries.
#[must_use]
pub fn entries_of(msg: &DataSource) -> Vec<Entry> {
    msg.sensors
        .iter()
        .map(|s| Entry {
            announcer: msg.announcer.clone(),
            name: s.name.clone(),
            topic: s.topic.clone(),
            kind: Kind::from_byte(s.source),
        })
        .collect()
}

/// The names of the nodes publishing a topic now.
///
/// # Errors
/// The ROS graph can't be read.
pub fn publishers_of(node: &Node, topic: &str) -> Result<Vec<String>> {
    Ok(node
        .get_publishers_info_by_topic(topic)
        .with_context(|| format!("the ROS graph: publishers of {topic}"))?
        .into_iter()
        .map(|i| i.node_name)
        .collect())
}

/// The names of the nodes publishing /system/source now.
///
/// # Errors
/// The ROS graph can't be read.
pub fn announcers_alive(node: &Node) -> Result<HashSet<String>> {
    Ok(publishers_of(node, SOURCE_TOPIC)?.into_iter().collect())
}

/// Declares parameters as rclpy does: numbers take whole numbers too; all read only.
/// (aav_watchdog_rs's `params.rs`, the parts these nodes use.) Holds the handles: rclrs
/// undeclares a parameter when its handle drops, so keep it as long as the node.
pub struct Declarer<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn Any>>,
}

impl<'a> Declarer<'a> {
    #[must_use]
    pub fn new(node: &'a Node) -> Self {
        Declarer {
            node,
            handles: Vec::new(),
        }
    }

    /// Keep the handles for as long as the node runs.
    #[must_use]
    pub fn finish(self) -> Vec<Box<dyn Any>> {
        self.handles
    }

    /// A read-only number (float or whole).
    ///
    /// # Errors
    /// Not a number, or not finite.
    pub fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::Double(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a number)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        match value {
            ParameterValue::Double(v) if v.is_finite() => Ok(v),
            ParameterValue::Integer(v) => Ok(v as f64),
            other => bail!("{name}:={other:?}: not a number"),
        }
    }

    /// A read-only text.
    ///
    /// # Errors
    /// Not text.
    pub fn text(&mut self, name: &str, default: &str) -> Result<String> {
        let p = self
            .node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (text)"))?;
        let value = p.get().to_string();
        self.handles.push(Box::new(p));
        Ok(value)
    }
}

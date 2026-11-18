//! Declaring parameters as rclpy does: numbers take whole numbers too; all read only.
//! (aav_ego_rs's ego-node `params.rs`, the parts this node uses.)

use std::any::Any;
use std::sync::Arc;

use anyhow::{bail, Context as _, Result};
use rclrs::{Node, ParameterValue};

/// Declares parameters; holds the handles (rclrs undeclares a parameter when its handle
/// drops), so keep it as long as the node.
pub struct Declarer<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn Any>>,
}

/// A number from a parameter: a float, or a whole number (`ros2 param set /ego payload 80`).
pub fn as_number(value: &ParameterValue) -> Option<f64> {
    match value {
        ParameterValue::Double(v) => Some(*v),
        ParameterValue::Integer(v) => Some(*v as f64),
        _ => None,
    }
}

impl<'a> Declarer<'a> {
    pub fn new(node: &'a Node) -> Self {
        Declarer {
            node,
            handles: Vec::new(),
        }
    }

    /// Keep the handles for as long as the node runs.
    pub fn finish(self) -> Vec<Box<dyn Any>> {
        self.handles
    }

    /// A read-only number (float or whole).
    pub fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::Double(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a number)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        match as_number(&value) {
            Some(v) if v.is_finite() => Ok(v),
            _ => bail!("{name}:={value:?}: not a number"),
        }
    }

    pub fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (true or false)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        Ok(value)
    }

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

    pub fn names(&mut self, name: &str, default: &[&str]) -> Result<Vec<String>> {
        let default: Arc<[Arc<str>]> = default.iter().map(|s| Arc::from(*s)).collect();
        // An empty list in YAML ([]) has no type: take it as no names.
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::StringArray(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a list of names)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        match value {
            ParameterValue::StringArray(names) => Ok(names.iter().map(|s| s.to_string()).collect()),
            ParameterValue::ByteArray(b) if b.is_empty() => Ok(Vec::new()),
            ParameterValue::IntegerArray(b) if b.is_empty() => Ok(Vec::new()),
            ParameterValue::DoubleArray(b) if b.is_empty() => Ok(Vec::new()),
            ParameterValue::BoolArray(b) if b.is_empty() => Ok(Vec::new()),
            other => bail!("{name}:={other:?}: not a list of names"),
        }
    }
}

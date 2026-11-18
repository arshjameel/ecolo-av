//! Declaring parameters as rclpy does: numbers take whole numbers too; all read only.
//! (aav_watchdog_rs's `params.rs`, with whole numbers added.)

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

/// A number from a parameter: a float, or a whole number.
fn as_number(value: &ParameterValue) -> Option<f64> {
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

    /// A read-only whole number.
    pub fn integer(&mut self, name: &str, default: i64) -> Result<i64> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (a whole number)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    /// A read-only true or false.
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

    /// A read-only text.
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

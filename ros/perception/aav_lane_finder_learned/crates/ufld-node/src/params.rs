//! Declaring parameters as rclpy does: numbers take whole numbers too; all read only.
//! (aav_lane_finder_rs's `params.rs`, with text for the model's file and provider.)

use std::any::Any;

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

    fn declare(
        &mut self,
        name: &str,
        default: ParameterValue,
        kind: &str,
    ) -> Result<ParameterValue> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} ({kind})"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    /// A read-only number (float or whole).
    pub fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let value = self.declare(name, ParameterValue::Double(default), "a number")?;
        match as_number(&value) {
            Some(v) if v.is_finite() => Ok(v),
            _ => bail!("{name}:={value:?}: not a number"),
        }
    }

    /// A read-only whole number, at least 0 (a float with no fraction too).
    pub fn count(&mut self, name: &str, default: usize) -> Result<usize> {
        let value = self.declare(
            name,
            ParameterValue::Integer(default as i64),
            "a whole number",
        )?;
        match as_number(&value) {
            Some(v) if v >= 0.0 && v.fract() == 0.0 && v <= f64::from(u32::MAX) => Ok(v as usize),
            _ => bail!("{name}:={value:?}: not a whole number, 0 or more"),
        }
    }

    /// Read-only text.
    pub fn text(&mut self, name: &str, default: &str) -> Result<String> {
        match self.declare(name, ParameterValue::String(default.into()), "text")? {
            ParameterValue::String(v) => Ok(v.to_string()),
            other => bail!("{name}:={other:?}: not text"),
        }
    }

    pub fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        match self.declare(name, ParameterValue::Bool(default), "true or false")? {
            ParameterValue::Bool(v) => Ok(v),
            other => bail!("{name}:={other:?}: not true or false"),
        }
    }

    /// Three read-only numbers (whole numbers too).
    pub fn three(&mut self, name: &str, default: [f64; 3]) -> Result<[f64; 3]> {
        let value = self.declare(
            name,
            ParameterValue::DoubleArray(default.into()),
            "three numbers",
        )?;
        let numbers: Vec<f64> = match &value {
            ParameterValue::DoubleArray(v) => v.to_vec(),
            ParameterValue::IntegerArray(v) => v.iter().map(|&i| i as f64).collect(),
            _ => Vec::new(),
        };
        match <[f64; 3]>::try_from(numbers) {
            Ok(v) if v.iter().all(|x| x.is_finite()) => Ok(v),
            _ => bail!("{name}:={value:?}: not three numbers"),
        }
    }
}

//! Declaring parameters as rclpy's ego does: numbers take whole numbers too, the live ones
//! are checked when set, the rest are read only.

use std::any::Any;
use std::sync::Arc;

use anyhow::{bail, Context as _, Result};
use rclrs::{MandatoryParameter, Node, ParameterValue};

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

    /// A list of numbers, or `None` when not given (no default).
    pub fn numbers(&mut self, name: &str) -> Result<Option<Vec<f64>>> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .optional()
            .with_context(|| format!("parameter {name} (a list of numbers)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        match value {
            None => Ok(None),
            Some(ParameterValue::DoubleArray(v)) => Ok(Some(v.to_vec())),
            Some(ParameterValue::IntegerArray(v)) => {
                Ok(Some(v.iter().map(|&x| x as f64).collect()))
            }
            Some(other) => bail!("{name}:={other:?}: not a list of numbers"),
        }
    }

    /// A live number, checked within `range` each time it is set.
    pub fn live_number(
        &mut self,
        name: &'static str,
        default: f64,
        range: (f64, f64),
    ) -> Result<MandatoryParameter<ParameterValue>> {
        let (lo, hi) = range;
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::Double(default))
            .validate(move |value| match as_number(value) {
                None => Err(format!("{name} must be a number")),
                Some(v) if !(lo..=hi).contains(&v) => {
                    Err(format!("{name} must be between {lo} and {hi}"))
                }
                Some(_) => Ok(()),
            })
            .mandatory()
            .with_context(|| format!("parameter {name} (a number from {lo} to {hi})"))?;
        Ok(p)
    }

    /// A live text, checked by `check` each time it is set.
    pub fn live_text(
        &mut self,
        name: &str,
        default: &str,
        check: impl Fn(&str) -> Result<(), String> + Send + Sync + 'static,
    ) -> Result<MandatoryParameter<Arc<str>>> {
        let p = self
            .node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .validate(move |value: &Arc<str>| check(value))
            .mandatory()
            .with_context(|| format!("parameter {name} (text)"))?;
        Ok(p)
    }
}

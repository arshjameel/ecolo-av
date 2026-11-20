//! What the three safety nodes share: start-up and shut-down, and read-only parameters.

use std::any::Any;
use std::sync::{Mutex, MutexGuard};

use anyhow::{bail, Context as _, Result};
use rclrs::*;

/// What a node keeps alive while it spins: its publishers, subscriptions, timers, state.
pub type Parts = Vec<Box<dyn Any>>;

/// Start the node `name`, let `start` make its parts, and spin until a signal.
///
/// # Errors
/// rclrs failing, or `start` refusing its parameters.
pub fn run(name: &str, start: fn(&Node) -> Result<Parts>) -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(name)?;
    let _keep = start(&node)?;
    let commands = std::sync::Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The lock, even if a callback panicked while holding it.
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// The node's parameters, declared read-only; their handles kept alive.
pub struct Params<'a> {
    node: &'a Node,
    keep: Vec<Box<dyn Any>>,
}

impl<'a> Params<'a> {
    /// Parameters of `node`.
    #[must_use]
    pub fn new(node: &'a Node) -> Self {
        Self {
            node,
            keep: Vec::new(),
        }
    }

    /// The handles to keep, with the node's other parts.
    #[must_use]
    pub fn into_keep(self) -> Vec<Box<dyn Any>> {
        self.keep
    }

    /// A parameter of any type.
    ///
    /// # Errors
    /// rclrs refusing it.
    pub fn value(&mut self, name: &str, default: ParameterValue) -> Result<ParameterValue> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name}"))?;
        let value = p.get();
        self.keep.push(Box::new(p));
        Ok(value)
    }

    /// A number, float or whole.
    ///
    /// # Errors
    /// Not a number.
    pub fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        match self.value(name, ParameterValue::Double(default))? {
            ParameterValue::Double(v) => Ok(v),
            ParameterValue::Integer(v) => Ok(v as f64),
            other => bail!("{name}:={other:?}: not a number"),
        }
    }

    /// True or false.
    ///
    /// # Errors
    /// Not a bool.
    pub fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        match self.value(name, ParameterValue::Bool(default))? {
            ParameterValue::Bool(v) => Ok(v),
            other => bail!("{name}:={other:?}: not true or false"),
        }
    }

    /// Text.
    ///
    /// # Errors
    /// Not text.
    pub fn text(&mut self, name: &str, default: &str) -> Result<String> {
        match self.value(name, ParameterValue::String(default.into()))? {
            ParameterValue::String(v) => Ok(v.to_string()),
            other => bail!("{name}:={other:?}: not text"),
        }
    }

    /// A list of numbers.
    ///
    /// # Errors
    /// Not a list of numbers.
    pub fn numbers(&mut self, name: &str, default: &[f64]) -> Result<Vec<f64>> {
        match self.value(name, ParameterValue::DoubleArray(default.into()))? {
            ParameterValue::DoubleArray(v) => Ok(v.to_vec()),
            ParameterValue::IntegerArray(v) => Ok(v.iter().map(|&i| i as f64).collect()),
            other => bail!("{name}:={other:?}: not a list of numbers"),
        }
    }

    /// A list of text.
    ///
    /// # Errors
    /// Not a list of text.
    pub fn texts(&mut self, name: &str, default: &[&str]) -> Result<Vec<String>> {
        let d: Vec<std::sync::Arc<str>> = default.iter().map(|&s| s.into()).collect();
        match self.value(name, ParameterValue::StringArray(d.into()))? {
            ParameterValue::StringArray(v) => Ok(v.iter().map(|s| s.to_string()).collect()),
            other => bail!("{name}:={other:?}: not a list of text"),
        }
    }
}

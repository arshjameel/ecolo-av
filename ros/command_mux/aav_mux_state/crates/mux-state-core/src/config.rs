//! twist_mux's inputs and locks, read from the same parameter files it reads.
//!
//! A file is a ROS parameters file: `twist_mux: ros__parameters: topics: <name>: {topic,
//! timeout, priority}` and the same under `locks:`. Several files load in order and a
//! later one wins, field by field, as `--params-file` does (aav_command_mux loads
//! `no_sensor_lock.yaml` after `twist_mux.yaml` to set one timeout).
//!
//! As twist_mux 4.5.0 (`twist_mux.cpp`, `topic_handle.hpp`): priorities are clamped to
//! 0..255; a missing timeout or priority is 0.

use serde_yaml_ng::Value;
use thiserror::Error;

/// The node's name: the top key of its parameter files.
pub const NODE: &str = "twist_mux";

/// One input or one lock.
#[derive(Debug, Clone, PartialEq)]
pub struct Slot {
    /// Its name in the file: joy, keyboard, nav... or estop, sensors...
    pub name: String,
    /// The topic it listens to.
    pub topic: String,
    /// Seconds: older than this, an input stops counting and a lock locks. 0 = never.
    pub timeout: f64,
    /// 0..255: higher wins (inputs), or holds more (locks).
    pub priority: u8,
}

/// Everything twist_mux was told.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MuxConfig {
    /// The drivers, by name (twist_mux lists its parameters sorted, so a tie goes to
    /// the first name).
    pub inputs: Vec<Slot>,
    /// The locks, by name.
    pub locks: Vec<Slot>,
    /// Inputs are TwistStamped (true, the default) or Twist.
    pub use_stamped: bool,
}

/// Why the files can't be used.
#[derive(Debug, Error, PartialEq)]
pub enum ConfigError {
    /// Not YAML.
    #[error("not YAML: {0}")]
    Yaml(String),
    /// No file had `twist_mux: ros__parameters:`.
    #[error("no `{NODE}: ros__parameters:` in any file")]
    NoParameters,
    /// An input or lock without a topic.
    #[error("{kind}.{name} has no topic")]
    NoTopic {
        /// topics or locks.
        kind: &'static str,
        /// The slot's name.
        name: String,
    },
    /// A value of the wrong kind.
    #[error("{kind}.{name}.{field}: {why}")]
    Bad {
        /// topics or locks.
        kind: &'static str,
        /// The slot's name.
        name: String,
        /// topic, timeout or priority.
        field: &'static str,
        /// What was wrong.
        why: String,
    },
    /// No inputs at all: nothing could ever drive.
    #[error("no inputs under topics:")]
    NoInputs,
}

/// One slot while the files are read: each field as the last file said it.
#[derive(Debug, Default, Clone)]
struct Partial {
    topic: Option<String>,
    timeout: Option<f64>,
    priority: Option<i64>,
}

impl MuxConfig {
    /// Reads twist_mux's parameter files, in the order it loads them.
    ///
    /// # Errors
    /// [`ConfigError`]: bad YAML, no `twist_mux: ros__parameters:` anywhere, a slot with
    /// no topic, a value of the wrong type, or no inputs.
    pub fn from_yaml(files: &[&str]) -> Result<Self, ConfigError> {
        let mut inputs: Vec<(String, Partial)> = Vec::new();
        let mut locks: Vec<(String, Partial)> = Vec::new();
        let mut use_stamped = true;
        let mut found = false;
        for text in files {
            let doc: Value =
                serde_yaml_ng::from_str(text).map_err(|e| ConfigError::Yaml(e.to_string()))?;
            let Some(params) = parameters(&doc) else {
                continue;
            };
            found = true;
            if let Some(v) = params.get("use_stamped") {
                use_stamped = v.as_bool().ok_or_else(|| ConfigError::Bad {
                    kind: "twist_mux",
                    name: String::new(),
                    field: "use_stamped",
                    why: "not true or false".into(),
                })?;
            }
            merge(params.get("topics"), "topics", &mut inputs)?;
            merge(params.get("locks"), "locks", &mut locks)?;
        }
        if !found {
            return Err(ConfigError::NoParameters);
        }
        let inputs = finish(inputs, "topics")?;
        if inputs.is_empty() {
            return Err(ConfigError::NoInputs);
        }
        Ok(Self {
            inputs,
            locks: finish(locks, "locks")?,
            use_stamped,
        })
    }
}

/// `twist_mux: ros__parameters:` (or `/twist_mux:`, or `/**:`, which apply to it too).
fn parameters(doc: &Value) -> Option<&Value> {
    ["twist_mux", "/twist_mux", "/**"]
        .iter()
        .filter_map(|k| doc.get(k)?.get("ros__parameters"))
        .find(|p| p.get("topics").is_some() || p.get("locks").is_some())
}

fn merge(
    section: Option<&Value>,
    kind: &'static str,
    into: &mut Vec<(String, Partial)>,
) -> Result<(), ConfigError> {
    let Some(Value::Mapping(map)) = section else {
        return Ok(());
    };
    for (key, fields) in map {
        let name = match key {
            Value::String(s) => s.clone(),
            other => format!("{other:?}"),
        };
        let bad = |field, why: &str| ConfigError::Bad {
            kind,
            name: name.clone(),
            field,
            why: why.to_owned(),
        };
        let at = match into.iter().position(|(n, _)| *n == name) {
            Some(i) => i,
            None => {
                into.push((name.clone(), Partial::default()));
                into.len() - 1
            }
        };
        let slot = &mut into[at].1;
        if let Some(v) = fields.get("topic") {
            slot.topic = Some(v.as_str().ok_or_else(|| bad("topic", "not text"))?.into());
        }
        if let Some(v) = fields.get("timeout") {
            let t = v.as_f64().ok_or_else(|| bad("timeout", "not a number"))?;
            if !t.is_finite() || t < 0.0 {
                return Err(bad("timeout", "must be 0 or more seconds"));
            }
            slot.timeout = Some(t);
        }
        if let Some(v) = fields.get("priority") {
            slot.priority = Some(
                v.as_i64()
                    .ok_or_else(|| bad("priority", "not a whole number"))?,
            );
        }
    }
    Ok(())
}

fn finish(slots: Vec<(String, Partial)>, kind: &'static str) -> Result<Vec<Slot>, ConfigError> {
    let mut out = slots
        .into_iter()
        .map(|(name, p)| {
            let topic = p.topic.ok_or_else(|| ConfigError::NoTopic {
                kind,
                name: name.clone(),
            })?;
            Ok(Slot {
                topic,
                timeout: p.timeout.unwrap_or(0.0),
                // twist_mux clamps (and warns): 0..255.
                priority: u8::try_from(p.priority.unwrap_or(0).clamp(0, 255)).unwrap_or(0),
                name,
            })
        })
        .collect::<Result<Vec<_>, ConfigError>>()?;
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE: &str = "
twist_mux:
  ros__parameters:
    use_stamped: true
    topics:
      keyboard: {topic: /drivers/keyboard/cmd_vel, timeout: 0.5, priority: 90}
      nav: {topic: /drivers/nav/cmd_vel, timeout: 0.5, priority: 20}
    locks:
      sensors: {topic: /safety/sensor_watchdog/lock, timeout: 1.0, priority: 30}
";

    #[test]
    fn reads_inputs_and_locks_sorted_by_name() {
        let c = MuxConfig::from_yaml(&[BASE]).expect("valid");
        let names: Vec<&str> = c.inputs.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["keyboard", "nav"]);
        assert_eq!(c.inputs[0].priority, 90);
        assert_eq!(c.locks[0].timeout, 1.0);
        assert!(c.use_stamped);
    }

    #[test]
    fn a_later_file_wins_field_by_field() {
        let off =
            "twist_mux:\n  ros__parameters:\n    locks:\n      sensors:\n        timeout: 0.0\n";
        let c = MuxConfig::from_yaml(&[BASE, off]).expect("valid");
        assert_eq!(c.locks[0].timeout, 0.0);
        assert_eq!(c.locks[0].priority, 30, "untouched fields stay");
        assert_eq!(c.locks[0].topic, "/safety/sensor_watchdog/lock");
    }

    #[test]
    fn priorities_are_clamped_and_whole_timeouts_read() {
        let y = "twist_mux:\n  ros__parameters:\n    topics:\n      a: {topic: /a, timeout: 1, priority: 300}\n      b: {topic: /b, priority: -4}\n";
        let c = MuxConfig::from_yaml(&[y]).expect("valid");
        assert_eq!(c.inputs[0].priority, 255);
        assert_eq!(c.inputs[0].timeout, 1.0);
        assert_eq!(c.inputs[1].priority, 0);
        assert_eq!(c.inputs[1].timeout, 0.0);
    }

    #[test]
    fn refuses_what_twist_mux_could_not_use() {
        assert_eq!(
            MuxConfig::from_yaml(&["other: {}"]),
            Err(ConfigError::NoParameters)
        );
        let no_topic = "twist_mux:\n  ros__parameters:\n    topics:\n      a: {priority: 3}\n";
        assert!(matches!(
            MuxConfig::from_yaml(&[no_topic]),
            Err(ConfigError::NoTopic { .. })
        ));
        let bad =
            "twist_mux:\n  ros__parameters:\n    topics:\n      a: {topic: /a, timeout: soon}\n";
        assert!(matches!(
            MuxConfig::from_yaml(&[bad]),
            Err(ConfigError::Bad { .. })
        ));
        let none = "twist_mux:\n  ros__parameters:\n    locks:\n      a: {topic: /a}\n";
        assert_eq!(MuxConfig::from_yaml(&[none]), Err(ConfigError::NoInputs));
        assert!(matches!(
            MuxConfig::from_yaml(&[": : :"]),
            Err(ConfigError::Yaml(_))
        ));
    }

    /// The car's own files, as aav_command_mux installs them.
    #[test]
    fn reads_the_cars_files() {
        let base = include_str!("../../../../aav_command_mux/config/twist_mux.yaml");
        let sensors = include_str!("../../../../aav_command_mux/config/no_sensor_lock.yaml");
        let c = MuxConfig::from_yaml(&[base]).expect("twist_mux.yaml");
        let joy = c.inputs.iter().find(|s| s.name == "joy").expect("joy");
        assert_eq!((joy.priority, joy.timeout), (100, 0.5));
        assert_eq!(c.inputs.len(), 10);
        let estop = c.locks.iter().find(|s| s.name == "estop").expect("estop");
        assert_eq!(
            (estop.priority, estop.topic.as_str()),
            (255, "/safety/estop")
        );
        let c = MuxConfig::from_yaml(&[base, sensors]).expect("both");
        let s = c
            .locks
            .iter()
            .find(|s| s.name == "sensors")
            .expect("sensors");
        assert_eq!(s.timeout, 0.0);
        // Every input is a driver's.
        assert!(c.inputs.iter().all(|s| s.topic.starts_with("/drivers/")));
    }
}

//! What the node does with each message, and each tick, without ROS
//! (`sensor_watchdog.py`'s `on_message`, `tick` and `publish_diagnostics`), so the tests
//! check it against the Python too.
//!
//! ```text
//! a message ──▶ Feed::odom / imu / gps / ... ──▶ Watch::arrived
//! the timer ──▶ Feed::tick ──▶ the lock, the state line, who changed (for the log)
//!          └──▶ Feed::diagnostics, once a second ──▶ one status per input
//! ```

use std::collections::{HashMap, HashSet};

use ego_core::fmt_g;

use crate::watch::{Arrival, Digest, Kind, State, Verdict, Watchdog};

/// The wheels' topic: its speed is the camera's frozen check's and the speed input's.
pub const VEHICLE_ODOM: &str = "/vehicle/odom";
/// What the odom input's cross check reads.
pub const IMU_TOPIC: &str = "/sensors/imu/data";

/// A diagnostic status's level (`diagnostic_msgs/DiagnosticStatus`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// OK.
    Ok,
    /// Wrong, but only warns.
    Warn,
    /// Wrong, and stops the drivers.
    Error,
}

impl Level {
    /// The message's byte.
    #[must_use]
    pub fn byte(self) -> u8 {
        match self {
            Level::Ok => 0,
            Level::Warn => 1,
            Level::Error => 2,
        }
    }
}

/// One input's `/diagnostics` status.
#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    /// `sensor_watchdog: <name>`.
    pub name: String,
    /// OK, WARN or ERROR.
    pub level: Level,
    /// One line, with the rate and the gap.
    pub message: String,
    /// The topic.
    pub hardware_id: String,
    /// The numbers, as text.
    pub values: Vec<(String, String)>,
}

/// What to log for one input whose state changed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Said {
    /// Back to OK (info).
    Back(String),
    /// Wrong, and it stops the drivers (error).
    Stops(String),
    /// Wrong, a warning.
    Warns(String),
}

/// One tick's answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// Every input's verdict.
    pub states: Vec<Verdict>,
    /// The lock: an input that stops isn't OK.
    pub locked: bool,
    /// The state line.
    pub summary: String,
    /// What to log.
    pub said: Vec<Said>,
}

/// The watchdog, and what the node keeps beside it.
#[derive(Debug, Clone)]
pub struct Feed {
    dog: Watchdog,
    /// The wheels' newest speed (m/s), and when it arrived (s).
    speed: Option<(f64, f64)>,
    said: HashMap<String, State>,
}

impl Feed {
    /// The node's watchdog on these rules: `stall` five ticks (at least 0.5 s), `settle` 3 s.
    #[must_use]
    pub fn new(dog: Watchdog) -> Self {
        Feed {
            dog,
            speed: None,
            said: HashMap::new(),
        }
    }

    /// The watchdog.
    #[must_use]
    pub fn dog(&self) -> &Watchdog {
        &self.dog
    }

    fn arrive(&mut self, now: f64, name: &str, a: Arrival) {
        if let Some(w) = self.dog.watch_mut(name) {
            w.arrived(now, a);
        }
    }

    fn topic_of(&self, name: &str) -> Option<&str> {
        self.dog
            .watches()
            .iter()
            .find(|w| w.rule().name == name)
            .map(|w| w.rule().topic.as_str())
    }

    /// An odom input's message: (vx, vy, wz, x, y).
    pub fn odom(&mut self, now: f64, name: &str, v: [f64; 5]) {
        if self.topic_of(name) == Some(VEHICLE_ODOM) {
            self.speed = Some((v[0], now));
        }
        let values = v.to_vec();
        self.arrive(
            now,
            name,
            Arrival {
                digest: Some(Digest::Values(values.clone())),
                values,
                ..Arrival::default()
            },
        );
    }

    /// An IMU input's message: (gyro x, y, z, accel x, y, z).
    pub fn imu(&mut self, now: f64, name: &str, v: [f64; 6]) {
        let values = v.to_vec();
        self.arrive(
            now,
            name,
            Arrival {
                digest: Some(Digest::Values(values.clone())),
                values,
                ..Arrival::default()
            },
        );
        if self.topic_of(name) == Some(IMU_TOPIC) {
            self.dog.imu(now, v[3], v[2]);
        }
    }

    /// A GPS input's fix: its stamp (s), where (degrees), and whether it says it has one.
    pub fn gps(&mut self, now: f64, name: &str, stamp: f64, fix: (f64, f64), has_fix: bool) {
        self.arrive(
            now,
            name,
            Arrival {
                fix: Some(fix),
                no_fix: !has_fix,
                stamp: Some(stamp),
                ..Arrival::default()
            },
        );
        if has_fix {
            self.dog.gps(stamp, fix);
        }
    }

    /// A camera's picture: its digest (`picture_digest`).
    pub fn image(&mut self, now: f64, name: &str, digest: u32) {
        let speed = self.speed.map(|(v, _)| v);
        self.arrive(
            now,
            name,
            Arrival {
                digest: Some(Digest::Crc(digest)),
                speed,
                ..Arrival::default()
            },
        );
    }

    /// A detections message from this camera frame: every number in its boxes.
    pub fn detections(&mut self, now: f64, name: &str, frame_id: &str, values: Vec<f64>) {
        let counts = self
            .dog
            .watches()
            .iter()
            .any(|w| w.rule().name == name && w.rule().counts(frame_id));
        if counts {
            self.arrive(
                now,
                name,
                Arrival {
                    values,
                    ..Arrival::default()
                },
            );
        }
    }

    /// An actuator's state: what it was told, what it did.
    pub fn actuator(&mut self, now: f64, name: &str, command: f64, measured: f64) {
        self.arrive(
            now,
            name,
            Arrival {
                values: vec![command, measured],
                ..Arrival::default()
            },
        );
    }

    /// Ego's commanded speed, against the wheels' newest (NaN before any) and its age:
    /// a stale wheel speed is no check (`Watch`'s overspeed check).
    pub fn speed(&mut self, now: f64, name: &str, command: f64) {
        let (measured, age) = self
            .speed
            .map_or((f64::NAN, f64::INFINITY), |(v, at)| (v, now - at));
        self.arrive(
            now,
            name,
            Arrival {
                values: vec![command, measured, age],
                ..Arrival::default()
            },
        );
    }

    /// A heartbeat: the frames drawn so far.
    pub fn heartbeat(&mut self, now: f64, name: &str, count: u64) {
        self.arrive(
            now,
            name,
            Arrival {
                digest: Some(Digest::Count(count)),
                ..Arrival::default()
            },
        );
    }

    /// A status message from one of our nodes: its name, its level byte, its words. Only
    /// the input's own name counts (`Rule::hears`).
    pub fn status(&mut self, now: f64, name: &str, status_name: &str, level: u8, words: &str) {
        let hears = self
            .dog
            .watches()
            .iter()
            .any(|w| w.rule().name == name && w.rule().hears(status_name));
        if hears {
            self.arrive(
                now,
                name,
                Arrival {
                    status: Some((level, words.to_string())),
                    ..Arrival::default()
                },
            );
        }
    }

    /// A cloud (a lidar's, a radar's): its digest (`picture_digest` of its data), for the
    /// frozen check where its rule has one (only while the car moves).
    pub fn cloud(&mut self, now: f64, name: &str, digest: u32) {
        let speed = self.speed.map(|(v, _)| v);
        self.arrive(
            now,
            name,
            Arrival {
                digest: Some(Digest::Crc(digest)),
                speed,
                ..Arrival::default()
            },
        );
    }

    /// `/vehicle/odom`, where no input watches it: only the speed.
    pub fn wheels(&mut self, now: f64, speed: f64) {
        self.speed = Some((speed, now));
    }

    /// The IMU for the cross check, where no input watches it.
    pub fn imu_only(&mut self, now: f64, forward: f64, yaw_rate: f64) {
        self.dog.imu(now, forward, yaw_rate);
    }

    /// A person cleared the latches.
    pub fn clear(&mut self) {
        self.dog.clear();
    }

    /// The node inputs' names to look for, and their full names.
    #[must_use]
    pub fn node_inputs(&self) -> Vec<(String, String)> {
        self.dog
            .watches()
            .iter()
            .filter(|w| w.rule().kind == Kind::Node)
            .map(|w| (w.rule().name.clone(), w.rule().topic.clone()))
            .collect()
    }

    /// One tick at `now` (the node's clock) and `steady` (a steady clock's s): the node
    /// inputs found in `running` (full names, `/ns/name`) arrive, then the verdict.
    pub fn tick(&mut self, now: f64, steady: f64, running: &HashSet<String>) -> Tick {
        for (name, topic) in self.node_inputs() {
            if running.contains(&topic) {
                self.arrive(now, &name, Arrival::default());
            }
        }
        self.dog.update(now, steady);
        let states = self.dog.states(now);
        let locked = states.iter().any(|v| v.stops);
        let summary = self.dog.summary(now);
        let mut said = Vec::new();
        for v in &states {
            let before = self.said.get(&v.name).copied();
            if before == Some(v.state) {
                continue;
            }
            if v.state == State::Ok {
                if !matches!(before, None | Some(State::Never)) {
                    said.push(Said::Back(format!("{}: OK again", v.name)));
                }
            } else if before.is_some() || v.state != State::Never {
                if v.stops {
                    said.push(Said::Stops(format!(
                        "{} {}: {}. Nav2 and the AI slot stop (twist_mux lock)",
                        v.name, v.state, v.detail
                    )));
                } else {
                    said.push(Said::Warns(format!("{} {}: {}", v.name, v.state, v.detail)));
                }
            }
            self.said.insert(v.name.clone(), v.state);
        }
        Tick {
            states,
            locked,
            summary,
            said,
        }
    }

    /// `/diagnostics`' statuses for this tick's `states`, at `now`; starts each input's
    /// traffic window afresh.
    pub fn diagnostics(&mut self, now: f64, states: &[Verdict]) -> Vec<Status> {
        let mut out = Vec::new();
        for v in states {
            let Some(w) = self.dog.watch_mut(&v.name) else {
                continue;
            };
            let level = if v.state == State::Ok {
                Level::Ok
            } else if v.stops {
                Level::Error
            } else {
                Level::Warn
            };
            let (rate, gap, worst_gap) = w.traffic(now);
            let rule = w.rule();
            let mut values = vec![
                ("rate_hz".to_string(), format!("{rate:.1}")),
                ("longest_gap_s".to_string(), format!("{gap:.2}")),
                ("worst_gap_s".to_string(), format!("{worst_gap:.2}")),
                ("max_age".to_string(), fmt_g(rule.max_age)),
                ("stops".to_string(), rule.stop.to_string()),
            ];
            if let Some(cross) = w.cross() {
                // For tuning the tolerances on the car: how close normal driving comes.
                let gaps = cross.gaps.map(|g| [g.0, g.1, g.2]);
                let worst = [cross.worst.0, cross.worst.1, cross.worst.2];
                for (i, key) in ["speed_gap", "turn_gap", "gps_gap"].iter().enumerate() {
                    values.push((
                        (*key).to_string(),
                        gaps.map_or(String::new(), |g| format!("{:.3}", g[i])),
                    ));
                    values.push((format!("worst_{key}"), format!("{:.3}", worst[i])));
                }
            }
            out.push(Status {
                name: format!("sensor_watchdog: {}", v.name),
                level,
                message: format!(
                    "{}: {}, {rate:.1} Hz, longest gap {gap:.2} s",
                    v.state, v.detail
                ),
                hardware_id: rule.topic.clone(),
                values,
            });
        }
        out
    }
}

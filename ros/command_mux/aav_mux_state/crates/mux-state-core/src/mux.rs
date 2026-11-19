//! twist_mux's rule, beside it: who it passes on now.
//!
//! As twist_mux 4.5.0 (`topic_handle.hpp`, `TwistMux::hasPriority`):
//!
//! - an input or lock has *expired* when its timeout is above 0 and its last message is
//!   older than the timeout, measured from when the message arrived (stamps are ignored).
//!   Never heard counts as heard long ago, so it has expired too;
//! - a lock is *on* while its last value was true, or it has expired (silence locks);
//! - the lock priority is the highest priority among the locks that are on (0 if none);
//! - an input is *masked* when it has expired, or its priority is below the lock
//!   priority; among the inputs not masked, the highest priority wins, and only one
//!   above 0 can win. A tie goes to the first by name.
//!
//! Times are seconds on the node's clock (the simulator's with use_sim_time), passed in.

use crate::config::{MuxConfig, Slot};

/// Why a lock is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Why {
    /// Its last message said true.
    Set,
    /// It went quiet for longer than its timeout (or was never heard): fail-safe.
    Silent,
}

/// One lock that is on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LockOn {
    /// Its name: estop, sensors...
    pub name: String,
    /// Its priority: it holds every input below this.
    pub priority: u8,
    /// Set, or silent.
    pub why: Why,
}

/// The driver twist_mux passes on.
#[derive(Debug, Clone, PartialEq)]
pub struct Driver {
    /// Its name: joy, keyboard, nav...
    pub name: String,
    /// Its topic.
    pub topic: String,
    /// Its priority.
    pub priority: u8,
    /// Seconds since its last command; None if never heard (only an input with timeout
    /// 0 can win unheard, as in twist_mux).
    pub age: Option<f64>,
}

/// The answer at one moment.
#[derive(Debug, Clone, PartialEq)]
pub struct Choice {
    /// Who drives; None: nothing passes, and ego stops after its own timeout.
    pub driver: Option<Driver>,
    /// The locks that are on, highest priority first.
    pub locks: Vec<LockOn>,
    /// The highest of their priorities (0: none on).
    pub lock_priority: u8,
    /// Inputs that are fresh but held by a lock (they would drive otherwise, or rank
    /// below one that does), highest priority first.
    pub held: Vec<String>,
}

/// One input's or lock's last message.
#[derive(Debug, Clone, Copy, Default)]
struct Heard {
    at: Option<f64>,
    value: bool,
}

impl Heard {
    fn age(self, now: f64) -> Option<f64> {
        self.at.map(|t| now - t)
    }

    fn expired(self, slot: &Slot, now: f64) -> bool {
        slot.timeout > 0.0 && self.age(now).is_none_or(|a| a > slot.timeout)
    }
}

/// The mux, as heard.
#[derive(Debug, Clone)]
pub struct Mux {
    config: MuxConfig,
    inputs: Vec<Heard>,
    locks: Vec<Heard>,
}

impl Mux {
    /// Nothing heard yet.
    #[must_use]
    pub fn new(config: MuxConfig) -> Self {
        Self {
            inputs: vec![Heard::default(); config.inputs.len()],
            locks: vec![Heard::default(); config.locks.len()],
            config,
        }
    }

    /// The config it follows.
    #[must_use]
    pub fn config(&self) -> &MuxConfig {
        &self.config
    }

    /// Input number `i` (in [`MuxConfig::inputs`]' order) sent a command at `now`. An
    /// unknown number is ignored.
    pub fn input(&mut self, i: usize, now: f64) {
        if let Some(h) = self.inputs.get_mut(i) {
            h.at = Some(now);
        }
    }

    /// Lock number `i` said `value` at `now`.
    pub fn lock(&mut self, i: usize, value: bool, now: f64) {
        if let Some(h) = self.locks.get_mut(i) {
            *h = Heard {
                at: Some(now),
                value,
            };
        }
    }

    /// Who twist_mux passes on at `now`, and why the others don't drive.
    #[must_use]
    pub fn choice(&self, now: f64) -> Choice {
        let mut locks: Vec<LockOn> = self
            .config
            .locks
            .iter()
            .zip(&self.locks)
            .filter_map(|(slot, h)| {
                let why = if h.value {
                    Why::Set
                } else if h.expired(slot, now) {
                    Why::Silent
                } else {
                    return None;
                };
                Some(LockOn {
                    name: slot.name.clone(),
                    priority: slot.priority,
                    why,
                })
            })
            .collect();
        locks.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.name.cmp(&b.name)));
        let lock_priority = locks.first().map_or(0, |l| l.priority);

        let fresh = || {
            self.config
                .inputs
                .iter()
                .zip(&self.inputs)
                .filter(move |(slot, h)| !h.expired(slot, now))
        };
        // The first strictly higher wins: a tie keeps the earlier name, as twist_mux.
        let mut driver: Option<(&Slot, &Heard)> = None;
        for (slot, h) in fresh().filter(|(slot, _)| slot.priority >= lock_priority) {
            if slot.priority > driver.map_or(0, |(d, _)| d.priority) {
                driver = Some((slot, h));
            }
        }
        let mut held: Vec<&Slot> = fresh()
            .filter(|(slot, _)| slot.priority < lock_priority)
            .map(|(slot, _)| slot)
            .collect();
        held.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.name.cmp(&b.name)));
        Choice {
            driver: driver.map(|(slot, h)| Driver {
                name: slot.name.clone(),
                topic: slot.topic.clone(),
                priority: slot.priority,
                age: h.age(now),
            }),
            locks,
            lock_priority,
            held: held.into_iter().map(|s| s.name.clone()).collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slot(name: &str, priority: u8, timeout: f64) -> Slot {
        Slot {
            name: name.into(),
            topic: format!("/{name}"),
            timeout,
            priority,
        }
    }

    /// Like the car's: two people, a test, navigation; the e-stop, the sensors lock
    /// (fail-safe) and a stop sign (never on silence).
    fn mux() -> Mux {
        let mut inputs = vec![
            slot("joy", 100, 0.5),
            slot("keyboard", 90, 0.5),
            slot("tests", 60, 0.5),
            slot("nav", 20, 0.5),
        ];
        inputs.sort_by(|a, b| a.name.cmp(&b.name));
        let mut locks = vec![
            slot("estop", 255, 0.0),
            slot("sensors", 30, 1.0),
            slot("stop_sign", 30, 0.0),
        ];
        locks.sort_by(|a, b| a.name.cmp(&b.name));
        Mux::new(MuxConfig {
            inputs,
            locks,
            use_stamped: true,
        })
    }

    fn index(m: &Mux, name: &str) -> usize {
        m.config()
            .inputs
            .iter()
            .position(|s| s.name == name)
            .expect("an input")
    }

    fn lock_index(m: &Mux, name: &str) -> usize {
        m.config()
            .locks
            .iter()
            .position(|s| s.name == name)
            .expect("a lock")
    }

    fn who(c: &Choice) -> Option<&str> {
        c.driver.as_ref().map(|d| d.name.as_str())
    }

    #[test]
    fn nobody_at_first_and_silence_locks_the_sensors() {
        let m = mux();
        let c = m.choice(10.0);
        assert_eq!(who(&c), None);
        assert_eq!(c.lock_priority, 30);
        assert_eq!(c.locks[0].name, "sensors");
        assert_eq!(c.locks[0].why, Why::Silent);
        assert!(c.held.is_empty());
    }

    #[test]
    fn the_highest_fresh_input_wins() {
        let mut m = mux();
        let sensors = lock_index(&m, "sensors");
        m.lock(sensors, false, 10.0);
        let (nav, keyboard) = (index(&m, "nav"), index(&m, "keyboard"));
        m.input(nav, 10.0);
        assert_eq!(who(&m.choice(10.1)), Some("nav"));
        m.input(keyboard, 10.2);
        let c = m.choice(10.3);
        assert_eq!(who(&c), Some("keyboard"));
        let d = c.driver.expect("a driver");
        assert_eq!(d.priority, 90);
        assert!((d.age.expect("heard") - 0.1).abs() < 1e-9);
        // The keyboard goes quiet for more than 0.5 s; navigation keeps sending.
        m.input(nav, 10.75);
        assert_eq!(who(&m.choice(10.75)), Some("nav"));
    }

    #[test]
    fn a_lock_holds_inputs_below_it_not_above() {
        let mut m = mux();
        let (nav, tests) = (index(&m, "nav"), index(&m, "tests"));
        m.input(nav, 5.0);
        // Sensors silent: navigation is held.
        let c = m.choice(5.1);
        assert_eq!(who(&c), None);
        assert_eq!(c.held, ["nav"]);
        // A test (60) is above the lock (30): it drives.
        m.input(tests, 5.1);
        let c = m.choice(5.2);
        assert_eq!(who(&c), Some("tests"));
        assert_eq!(c.held, ["nav"]);
        // The stop sign set too: both locks listed, highest first, then by name.
        let sign = lock_index(&m, "stop_sign");
        m.lock(sign, true, 5.2);
        let c = m.choice(5.2);
        let names: Vec<&str> = c.locks.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names, ["sensors", "stop_sign"]);
        assert_eq!(c.locks[1].why, Why::Set);
    }

    #[test]
    fn the_estop_holds_everyone() {
        let mut m = mux();
        let estop = lock_index(&m, "estop");
        let joy = index(&m, "joy");
        m.lock(estop, true, 1.0);
        m.input(joy, 1.0);
        let c = m.choice(1.0);
        assert_eq!(who(&c), None);
        assert_eq!(c.lock_priority, 255);
        assert_eq!(c.held, ["joy"]);
        // Cleared: false, and timeout 0 never locks on silence.
        m.lock(estop, false, 1.1);
        m.input(joy, 100.0);
        assert_eq!(who(&m.choice(100.0)), Some("joy"));
    }

    #[test]
    fn equal_priority_passes_a_lock_and_a_tie_goes_to_the_first_name() {
        let mut m = Mux::new(MuxConfig {
            inputs: vec![slot("a", 30, 0.5), slot("b", 30, 0.5)],
            locks: vec![slot("l", 30, 0.0)],
            use_stamped: true,
        });
        m.lock(0, true, 0.0);
        m.input(1, 0.0);
        assert_eq!(who(&m.choice(0.0)), Some("b"), "not below the lock: passes");
        m.input(0, 0.0);
        assert_eq!(who(&m.choice(0.0)), Some("a"));
    }

    #[test]
    fn timeout_zero_never_expires_and_priority_zero_never_drives() {
        let mut m = Mux::new(MuxConfig {
            inputs: vec![slot("forever", 5, 0.0), slot("zero", 0, 0.5)],
            locks: vec![],
            use_stamped: true,
        });
        // As twist_mux: a timeout-0 input counts before it is heard.
        let c = m.choice(1000.0);
        assert_eq!(who(&c), Some("forever"));
        assert_eq!(c.driver.expect("a driver").age, None);
        m.input(1, 1000.0);
        assert_eq!(who(&m.choice(1000.0)), Some("forever"));
        let mut m = Mux::new(MuxConfig {
            inputs: vec![slot("zero", 0, 0.5)],
            locks: vec![],
            use_stamped: true,
        });
        m.input(0, 1.0);
        assert_eq!(who(&m.choice(1.0)), None);
        m.input(7, 1.0); // unknown: ignored
        m.lock(7, true, 1.0);
    }
}

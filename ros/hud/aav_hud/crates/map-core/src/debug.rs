//! The debug page (key D): who is driving, what it asks, what ego sends, the gate, what
//! the car did; and beside them the locks, the sensor watchdog and real or simulated.
//!
//! Our professor's idea (TODO 247): the map screen doubles as a debug screen, like the
//! hardware panel: "see the commands being sent and maybe by who".
//!
//! Each box is a [`Card`]: a label, one big word or number, one small line, and a colour
//! ([`Tone`], the panel's: green, amber, red, grey). [`cards`] makes them from a [`MapState`]; the screen only draws them.
//!
//! | Card | From |
//! |---|---|
//! | WHO DRIVES | `/control/command_mux/state` (aav_mux_state); the navigator's follower from `/system/algorithms` |
//! | IT ASKS | `/control/cmd_vel`: the chosen driver's command, as twist_mux passed it |
//! | EGO SENDS | `/ego/actuator_cmd` |
//! | GATE | `/safety/output_mux/state` |
//! | THE CAR DID | `/vehicle/status` |
//! | LOCKS | `/control/command_mux/state` |
//! | SENSORS | `/safety/sensor_watchdog/state` |
//! | DATA | `/system/source` |

use std::time::Instant;

use hud_core::{
    format,
    gate::{Gate, Sensors},
    health::Health,
    vehicle::VehicleStatus,
};

use crate::{config::Topic, state::MapState, trip::Tone};

/// What `command_mux_state` said (its DiagnosticStatus).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MuxReport {
    /// Its one line, for people.
    pub line: String,
    /// The driver twist_mux passes on (`keyboard`, `nav`...), if any.
    pub driver: Option<String>,
    /// Its priority (0 if none).
    pub priority: u8,
    /// Seconds since its last command.
    pub age: Option<f64>,
    /// The locks on, highest first, as sent (`sensors 30 silent`).
    pub locks: Vec<String>,
    /// Inputs sending but held by a lock.
    pub held: Vec<String>,
}

fn list(text: &str) -> Vec<String> {
    if text.is_empty() || text == "none" {
        return Vec::new();
    }
    text.split(", ").map(str::to_owned).collect()
}

impl MuxReport {
    /// Reads the message's line and its key-value rows (aav_mux_state report.rs).
    /// Missing or unreadable rows count as nothing.
    #[must_use]
    pub fn from_values<'a>(
        line: &str,
        values: impl IntoIterator<Item = (&'a str, &'a str)>,
    ) -> Self {
        let mut r = Self {
            line: line.to_owned(),
            ..Self::default()
        };
        for (key, value) in values {
            match key {
                "driver" if !value.is_empty() && value != "none" => {
                    r.driver = Some(value.to_owned());
                }
                "priority" => r.priority = value.parse().unwrap_or(0),
                "age_s" => r.age = value.parse().ok(),
                "locks" => r.locks = list(value),
                "held" => r.held = list(value),
                _ => {}
            }
        }
        r
    }
}

/// The command twist_mux passed on (`/control/cmd_vel`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Ask {
    /// m/s, forward +.
    pub speed: f64,
    /// rad/s, left +.
    pub turn: f64,
}

/// What ego sends to the car (`/ego/actuator_cmd`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sent {
    /// m/s.
    pub speed: f64,
    /// rad, left +.
    pub steering: f64,
    /// m/s², + speeding up, - braking.
    pub accel: f64,
}

/// Everything the page needs that the map itself doesn't.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct DebugState {
    /// `command_mux_state`'s last word.
    pub mux: Option<MuxReport>,
    /// The chosen command.
    pub ask: Option<Ask>,
    /// Ego's command.
    pub sent: Option<Sent>,
    /// The output mux's line.
    pub gate: Option<String>,
    /// The car.
    pub vehicle: Option<VehicleStatus>,
    /// The navigator's path follower, from `/system/algorithms` (`mppi_lag (aav_mppi_lag)`).
    pub follower: Option<String>,
}

/// One box on the page.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Card {
    /// Its name, small, at the top: WHO DRIVES.
    pub label: &'static str,
    /// The one big thing: KEYBOARD, 0.80 m/s, OPEN.
    pub big: String,
    /// One small line under it.
    pub small: String,
    /// Its colour.
    pub tone: Tone,
}

impl Card {
    fn new(
        label: &'static str,
        big: impl Into<String>,
        small: impl Into<String>,
        tone: Tone,
    ) -> Self {
        Self {
            label,
            big: big.into(),
            small: small.into(),
            tone,
        }
    }
}

/// Who a driver is, in big words, and whether a person drives.
#[must_use]
pub fn who(driver: &str) -> (String, bool) {
    let (words, person) = match driver {
        "joy" => ("JOYSTICK", true),
        "rc" => ("RC REMOTE", true),
        "keyboard" => ("KEYBOARD", true),
        "buttons" => ("PANEL BUTTONS", true),
        "tests" => ("ACTUATOR TEST", false),
        "scripted" => ("SCRIPT", false),
        "replay" => ("REPLAY", false),
        "point" => ("DRIVE TO POINT", false),
        "nav" => ("NAVIGATOR (AI)", false),
        "ai" => ("AI SLOT", false),
        other => return (other.to_uppercase(), false),
    };
    (words.to_owned(), person)
}

/// A lock's name in words.
fn lock_words(lock: &str) -> String {
    // "sensors 30 silent" -> "SENSORS (silent)"
    let mut parts = lock.split(' ');
    let name = parts.next().unwrap_or_default();
    let silent = lock.ends_with(" silent");
    let name = match name {
        "estop" => "E-STOP",
        "sensors" => "SENSORS",
        "stop_sign" => "STOP SIGN",
        "stop_zone" => "STOP ZONE",
        "geofence" => "GEOFENCE",
        "no_timeout" => "OLD FIRMWARE",
        other => return other.to_uppercase(),
    };
    if silent {
        format!("{name} (silent)")
    } else {
        name.to_owned()
    }
}

/// Which way a turn goes (+ left, REP-103); a tiny one is straight.
fn side(turn: f64) -> &'static str {
    if turn > 1e-3 {
        " left"
    } else if turn < -1e-3 {
        " right"
    } else {
        ""
    }
}

fn quiet(label: &'static str, health: Health, what: &str) -> Card {
    match health {
        Health::Never => Card::new(
            label,
            "NOTHING",
            format!("{what}: never heard"),
            Tone::Quiet,
        ),
        _ => Card::new(
            label,
            "NOTHING",
            format!("{what}: {}", format::health(health)),
            Tone::Quiet,
        ),
    }
}

/// The page's cards, in two columns: the command's path (who, asks, ego, gate, car), then
/// what may stop it (locks, sensors, data).
#[must_use]
pub fn cards(s: &MapState, now: Instant) -> (Vec<Card>, Vec<Card>) {
    let d = &s.debug;
    let h = |t| s.health(t, now);
    (
        vec![
            who_card(d, h(Topic::MuxState)),
            ask_card(d, h(Topic::Command)),
            sent_card(d, h(Topic::Actuator)),
            gate_card(d, h(Topic::Gate)),
            car_card(d, h(Topic::Vehicle)),
        ],
        vec![
            locks_card(d, h(Topic::MuxState)),
            sensors_card(s.watchdog.as_deref(), h(Topic::Watchdog)),
            data_card(s),
        ],
    )
}

fn who_card(d: &DebugState, health: Health) -> Card {
    const L: &str = "WHO DRIVES";
    let Some(mux) = d.mux.as_ref().filter(|_| health.is_ok()) else {
        return match health {
            Health::Never => Card::new(L, "?", "no command mux state heard", Tone::Quiet),
            _ => Card::new(
                L,
                "?",
                format!("mux state {}", format::health(health)),
                Tone::Bad,
            ),
        };
    };
    match &mux.driver {
        Some(name) => {
            let (words, person) = who(name);
            let age = mux
                .age
                .map(|a| format!(", {a:.1} s ago"))
                .unwrap_or_default();
            let mut small = format!(
                "{} (priority {}{age})",
                if person { "a person" } else { "a program" },
                mux.priority
            );
            if name == "nav" {
                if let Some(f) = &d.follower {
                    small = format!("follower {f}{age}");
                }
            }
            Card::new(L, words, small, Tone::Good)
        }
        None if mux.held.is_empty() => Card::new(L, "NOBODY", "no driver is sending", Tone::Quiet),
        None => {
            let held: Vec<String> = mux.held.iter().map(|h| who(h).0).collect();
            Card::new(
                L,
                "NOBODY",
                format!("held: {}", held.join(", ")),
                Tone::Warn,
            )
        }
    }
}

fn ask_card(d: &DebugState, health: Health) -> Card {
    const L: &str = "IT ASKS";
    match (d.ask, health) {
        (Some(a), Health::Ok(_)) => Card::new(
            L,
            format!("{:.2} m/s", a.speed),
            format!("turn {:.2} rad/s{}", a.turn.abs(), side(a.turn)),
            if a.speed.abs() < 1e-3 && a.turn.abs() < 1e-3 {
                Tone::Quiet
            } else {
                Tone::Good
            },
        ),
        _ => quiet(L, health, "no command"),
    }
}

fn sent_card(d: &DebugState, health: Health) -> Card {
    const L: &str = "EGO SENDS";
    match (d.sent, health) {
        (Some(c), Health::Ok(_)) => {
            let steer = format::steering(c.steering.to_degrees() as f32);
            let pedal = if c.accel < -1e-3 {
                format!("braking {:.1} m/s2", -c.accel)
            } else if c.accel > 1e-3 {
                format!("speeding up {:.1} m/s2", c.accel)
            } else {
                "holding".to_owned()
            };
            Card::new(
                L,
                format!("{:.2} m/s", c.speed),
                format!("steer {steer}, {pedal}"),
                Tone::Good,
            )
        }
        (_, Health::Stale(_)) => Card::new(
            L,
            "SILENT",
            format!("ego {}", format::health(health)),
            Tone::Bad,
        ),
        _ => quiet(L, health, "ego"),
    }
}

fn gate_card(d: &DebugState, health: Health) -> Card {
    const L: &str = "GATE";
    match (&d.gate, health) {
        (Some(line), Health::Ok(_)) => match Gate::parse(line) {
            Gate::Open(what) => Card::new(L, "OPEN", what, Tone::Good),
            Gate::Wait(why) => Card::new(L, "WAIT", why, Tone::Warn),
            Gate::Stop(why) => Card::new(L, "STOP", why, Tone::Bad),
            Gate::Other(text) => Card::new(L, "?", text, Tone::Warn),
        },
        (_, Health::Stale(_)) => Card::new(L, "SILENT", "nothing reaches the car", Tone::Bad),
        _ => Card::new(L, "?", "no output mux heard", Tone::Quiet),
    }
}

fn car_card(d: &DebugState, health: Health) -> Card {
    const L: &str = "THE CAR DID";
    match (d.vehicle, health) {
        (Some(v), Health::Ok(_)) => {
            let steer = format::steering(v.steering_angle.to_degrees());
            let small = format!(
                "steer {steer}, throttle {:.0}%, brake {:.0}%, {}",
                v.throttle.get() * 100.0,
                v.brake.get() * 100.0,
                v.mode
            );
            if v.estop_active {
                Card::new(L, "E-STOP", small, Tone::Bad)
            } else if !v.faults.is_empty() {
                Card::new(
                    L,
                    format!("{:.2} m/s", v.speed.0),
                    format!("FAULT {}", v.faults),
                    Tone::Bad,
                )
            } else {
                Card::new(L, format!("{:.2} m/s", v.speed.0), small, Tone::Good)
            }
        }
        (_, Health::Stale(_)) => Card::new(
            L,
            "SILENT",
            format!("/vehicle/status {}", format::health(health)),
            Tone::Bad,
        ),
        _ => quiet(L, health, "no car status"),
    }
}

fn locks_card(d: &DebugState, health: Health) -> Card {
    const L: &str = "LOCKS";
    let Some(mux) = d.mux.as_ref().filter(|_| health.is_ok()) else {
        return Card::new(L, "?", "no command mux state heard", Tone::Quiet);
    };
    if mux.locks.is_empty() {
        return Card::new(L, "NONE", "nothing held", Tone::Good);
    }
    let names: Vec<String> = mux.locks.iter().map(|l| lock_words(l)).collect();
    let small = if mux.held.is_empty() {
        "holding nobody who is sending".to_owned()
    } else {
        let held: Vec<String> = mux.held.iter().map(|h| who(h).0).collect();
        format!("holding {}", held.join(", "))
    };
    let tone = if mux.locks.iter().any(|l| l.starts_with("estop ")) {
        Tone::Bad
    } else {
        Tone::Warn
    };
    Card::new(L, names.join(", "), small, tone)
}

fn sensors_card(line: Option<&str>, health: Health) -> Card {
    const L: &str = "SENSORS";
    match (line, health) {
        (_, Health::Never) => Card::new(L, "?", "no sensor watchdog heard", Tone::Quiet),
        (_, Health::Stale(_)) => Card::new(
            L,
            "SILENT",
            "the watchdog stopped: navigation held",
            Tone::Bad,
        ),
        (Some(text), _) => match Sensors::parse(text) {
            Sensors::Held { input, state } => {
                Card::new(L, "STOP", format!("{input} {state}"), Tone::Bad)
            }
            Sensors::Drive if text.starts_with("WARN") => Card::new(
                L,
                "WARN",
                text.trim_start_matches("WARN")
                    .trim_start_matches([':', ' ', '-']),
                Tone::Warn,
            ),
            Sensors::Drive => Card::new(L, "OK", "every sensor fresh", Tone::Good),
        },
        (None, _) => Card::new(L, "?", "", Tone::Quiet),
    }
}

fn data_card(s: &MapState) -> Card {
    const L: &str = "DATA";
    if s.origins.entries().next().is_none() {
        return Card::new(L, "?", "nobody said where the data comes from", Tone::Quiet);
    }
    match s.origins.banner() {
        None => Card::new(L, "REAL", "every sensor is the car's own", Tone::Good),
        Some(kind) => Card::new(L, kind, "not the real car", Tone::Warn),
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use hud_core::{
        origin::{Announcement, Entry, Origin},
        units::{Fraction, MetresPerSecond, Radians},
        vehicle::{Faults, Mode},
    };

    use super::*;
    use crate::{config::MapConfig, state::Update};

    fn report(driver: &str, locks: &str, held: &str) -> MuxReport {
        MuxReport::from_values(
            "line",
            [
                ("driver", driver),
                ("priority", "90"),
                ("age_s", "0.05"),
                ("locks", locks),
                ("held", held),
            ],
        )
    }

    #[test]
    fn the_mux_report_reads_its_rows() {
        let r = report("keyboard", "sensors 30 silent, stop_sign 30", "nav, ai");
        assert_eq!(r.driver.as_deref(), Some("keyboard"));
        assert_eq!(r.priority, 90);
        assert_eq!(r.age, Some(0.05));
        assert_eq!(r.locks, ["sensors 30 silent", "stop_sign 30"]);
        assert_eq!(r.held, ["nav", "ai"]);
        let none = report("none", "none", "none");
        assert_eq!(none.driver, None);
        assert!(none.locks.is_empty() && none.held.is_empty());
        let empty = MuxReport::from_values("", []);
        assert_eq!(empty.priority, 0);
    }

    #[test]
    fn drivers_have_plain_names() {
        assert_eq!(who("joy"), ("JOYSTICK".into(), true));
        assert_eq!(who("nav"), ("NAVIGATOR (AI)".into(), false));
        assert_eq!(who("new_one"), ("NEW_ONE".into(), false));
        assert_eq!(lock_words("sensors 30 silent"), "SENSORS (silent)");
        assert_eq!(lock_words("estop 255"), "E-STOP");
    }

    fn state() -> (MapState, Instant) {
        (MapState::new(MapConfig::default()), Instant::now())
    }

    #[test]
    fn nothing_heard_is_grey() {
        let (s, now) = state();
        let (path, safety) = cards(&s, now);
        assert_eq!(path.len(), 5);
        assert_eq!(safety.len(), 3);
        assert!(
            path.iter().chain(&safety).all(|c| c.tone == Tone::Quiet),
            "{path:?} {safety:?}"
        );
    }

    #[test]
    fn the_keyboard_drives_through_an_open_gate() {
        let (mut s, now) = state();
        s.apply(Update::MuxState(report("keyboard", "none", "none")), now);
        s.apply(
            Update::Command(Ask {
                speed: 0.8,
                turn: 0.1,
            }),
            now,
        );
        s.apply(
            Update::Actuator(Sent {
                speed: 0.8,
                steering: 0.1,
                accel: -0.5,
            }),
            now,
        );
        s.apply(Update::Gate("OPEN - sim: every source SIM".into()), now);
        let v = VehicleStatus {
            mode: Mode::Computer,
            armed: true,
            speed: MetresPerSecond(0.75),
            steering_angle: Radians(0.09),
            throttle: Fraction::new(0.3),
            ..VehicleStatus::default()
        };
        s.apply(Update::Vehicle(v), now);
        s.apply(Update::Watchdog("OK".into()), now);
        let (path, safety) = cards(&s, now);
        assert_eq!(path[0].big, "KEYBOARD");
        assert!(path[0].small.contains("a person"), "{}", path[0].small);
        assert_eq!(path[1].big, "0.80 m/s");
        assert!(path[1].small.ends_with("left"));
        assert!(path[2].small.contains("braking 0.5"), "{}", path[2].small);
        assert_eq!(path[3].big, "OPEN");
        assert_eq!(path[4].big, "0.75 m/s");
        assert!(path[4].small.contains("throttle 30%"), "{}", path[4].small);
        assert!(path.iter().all(|c| c.tone == Tone::Good), "{path:?}");
        assert_eq!(safety[0].big, "NONE");
        assert_eq!(safety[1].big, "OK");
    }

    #[test]
    fn the_navigator_shows_its_follower_and_a_lock_holds_it() {
        let (mut s, now) = state();
        s.apply(Update::Follower("mppi_lag (aav_mppi_lag)".into()), now);
        s.apply(Update::MuxState(report("nav", "none", "none")), now);
        let (path, _) = cards(&s, now);
        assert_eq!(path[0].big, "NAVIGATOR (AI)");
        assert!(
            path[0].small.starts_with("follower mppi_lag"),
            "{}",
            path[0].small
        );
        s.apply(
            Update::MuxState(report("none", "sensors 30 silent", "nav")),
            now,
        );
        s.apply(
            Update::Watchdog("STOP: gps STALE (nothing for 2.3 s)".into()),
            now,
        );
        let (path, safety) = cards(&s, now);
        assert_eq!((path[0].big.as_str(), path[0].tone), ("NOBODY", Tone::Warn));
        assert_eq!(path[0].small, "held: NAVIGATOR (AI)");
        assert_eq!(safety[0].big, "SENSORS (silent)");
        assert_eq!(safety[0].tone, Tone::Warn);
        assert_eq!(
            (safety[1].big.as_str(), safety[1].tone),
            ("STOP", Tone::Bad)
        );
        assert_eq!(safety[1].small, "gps STALE");
        // The e-stop is red.
        s.apply(Update::MuxState(report("none", "estop 255", "joy")), now);
        assert_eq!(cards(&s, now).1[0].tone, Tone::Bad);
    }

    #[test]
    fn silence_and_trouble_are_red() {
        let (mut s, now) = state();
        s.apply(
            Update::Gate("STOP - car - target car needs REAL - gps is SIM - latched".into()),
            now,
        );
        s.apply(Update::MuxState(report("keyboard", "none", "none")), now);
        s.apply(
            Update::Vehicle(VehicleStatus {
                faults: Faults(8),
                ..VehicleStatus::default()
            }),
            now,
        );
        let (path, _) = cards(&s, now);
        assert_eq!((path[3].big.as_str(), path[3].tone), ("STOP", Tone::Bad));
        assert!(path[4].small.starts_with("FAULT"), "{}", path[4].small);
        let later = now + Duration::from_secs(5);
        let (path, safety) = cards(&s, later);
        assert_eq!((path[0].big.as_str(), path[0].tone), ("?", Tone::Bad));
        assert_eq!(path[3].big, "SILENT");
        assert_eq!(path[4].big, "SILENT");
        assert_eq!(safety[0].tone, Tone::Quiet);
    }

    #[test]
    fn the_data_card_says_real_or_not() {
        let (mut s, now) = state();
        let ann = |origin| Announcement {
            announcer: "x".into(),
            entries: vec![Entry {
                name: "gps".into(),
                topic: "/sensors/gps/fix".into(),
                origin,
            }],
        };
        s.apply(Update::Source(ann(Origin::Real)), now);
        assert_eq!(cards(&s, now).1[2].big, "REAL");
        s.apply(Update::Source(ann(Origin::Sim)), now);
        let c = &cards(&s, now).1[2];
        assert_eq!((c.big.as_str(), c.tone), ("SIMULATED", Tone::Warn));
    }
}

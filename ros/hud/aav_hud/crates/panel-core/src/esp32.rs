//! The ESP32's status lines (`firmware/status/README.md`): one JSON object a line, read
//! over its WebSocket by `hw_panel --esp32` ([`crate::ws`]). Read only: nothing here ever
//! reaches the buttons, the gate or the calibration page's rules.
//!
//! | `t` | Becomes |
//! |---|---|
//! | `hb` | [`StatusLine::Heartbeat`]: uptime, boot ID, build |
//! | `status` | [`StatusLine::Status`]: the Pi's last frame as the board got it (`rx`), what went to each device, the link's counts |
//! | `event` | [`StatusLine::Event`]: what changed, from and to |
//! | another `t`, or another `schema` | [`StatusLine::Unknown`]: said, and its values treated as unknown |
//!
//! Fields this panel doesn't know are ignored (the contract's rule).

use std::fmt;

use serde::Deserialize;
use serde_json::Value;
use thiserror::Error;

/// The schema this panel reads (`schema` in every line).
pub const SCHEMA: u64 = 1;

/// Why a line can't be read at all.
#[derive(Debug, Error)]
pub enum LineError {
    /// The line isn't JSON.
    #[error("not JSON: {0}")]
    Json(#[from] serde_json::Error),
    /// A key every line has is missing.
    #[error("no `{0}`: every line has t, schema, seq and up_ms")]
    Missing(&'static str),
}

/// One line.
#[derive(Debug, Clone, PartialEq)]
pub enum StatusLine {
    /// `hb`: the heartbeat.
    Heartbeat(Heartbeat),
    /// `status`: what the board got and did.
    Status(Box<Status>),
    /// `event`: what changed.
    Event(Event),
    /// A kind or a schema this panel doesn't know: its values are unknown.
    Unknown(Unknown),
}

impl StatusLine {
    /// The line's sequence number: a gap means lines were lost.
    #[must_use]
    pub fn seq(&self) -> u64 {
        match self {
            Self::Heartbeat(h) => h.seq,
            Self::Status(s) => s.seq,
            Self::Event(e) => e.seq,
            Self::Unknown(u) => u.seq,
        }
    }
}

/// A line this panel can't read the values of.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unknown {
    /// Its `t`.
    pub kind: String,
    /// Its `schema`.
    pub schema: u64,
    /// Its sequence number.
    pub seq: u64,
}

impl fmt::Display for Unknown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.schema == SCHEMA {
            write!(f, "a line of kind {:?} this panel doesn't know", self.kind)
        } else {
            write!(
                f,
                "schema {} (this panel reads {SCHEMA}): its values are unknown",
                self.schema
            )
        }
    }
}

/// `hb`, once a second.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Heartbeat {
    /// The line's sequence number.
    pub seq: u64,
    /// ms since the board started.
    pub up_ms: u64,
    /// A new one at each start.
    #[serde(default)]
    pub boot_id: String,
    /// The firmware's build.
    #[serde(default)]
    pub build: String,
    /// The features built in.
    #[serde(default)]
    pub features: String,
    /// Why the board last started.
    #[serde(default)]
    pub last_reset: String,
}

/// The last good frame from the Pi, as the board got it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Rx {
    /// The frame's version.
    pub v: u8,
    /// The frame's counter.
    pub counter: u8,
    /// The frame's last byte: with the counter, it tells frames apart.
    pub check: u8,
    /// -1..1, the frame's own values.
    pub steer: f64,
    /// The speed, -1..1.
    pub speed: f64,
    /// The acceleration, -1..1.
    pub accel: f64,
    /// ms since the frame came.
    pub age_ms: u32,
}

/// What the firmware asked of each device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub struct Out {
    /// The steering actuator's target (counts).
    pub steer_counts: u16,
    /// The brake actuator's target (counts).
    pub brake_counts: u16,
    /// The throttle's DAC value.
    pub dac: u16,
    /// The reverse relay is on.
    #[serde(default)]
    pub reverse: bool,
}

/// The lowest or highest of each since the last line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub struct Extremes {
    /// The steering actuator's target (counts).
    pub steer_counts: u16,
    /// The brake actuator's target (counts).
    pub brake_counts: u16,
    /// The throttle's DAC value.
    pub dac: u16,
}

/// Each Jrk's position in counts, when known.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
pub struct Positions {
    /// The steering actuator's.
    pub steer: Option<u16>,
    /// The brake actuator's.
    pub brake: Option<u16>,
}

/// The link's counts since boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
pub struct LinkCounts {
    /// Frames that came.
    pub frames: u32,
    /// Frames with a bad check.
    pub bad: u32,
    /// Frames missed, by the counter.
    pub skipped: u32,
    /// Frames that came twice.
    pub repeats: u32,
}

/// Who drives the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoardMode {
    /// The computer drives.
    Computer,
    /// The RC remote drives.
    Rc,
    /// Stopped.
    Stopped,
    /// A test drives.
    Test,
    /// A mode this panel doesn't know.
    #[serde(other)]
    Other,
}

impl fmt::Display for BoardMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Computer => "COMPUTER",
            Self::Rc => "RC",
            Self::Stopped => "STOPPED",
            Self::Test => "TEST",
            Self::Other => "a mode this panel doesn't know",
        })
    }
}

/// The board's state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub struct BoardState {
    /// Who drives.
    pub mode: BoardMode,
    /// The board is armed.
    pub armed: bool,
    /// An e-stop is on.
    pub estop: bool,
    /// The fault bits.
    pub faults: u16,
}

/// The tick's timing since boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
pub struct Tick {
    /// The longest gap between ticks (us).
    pub worst_gap_us: u32,
    /// Ticks that ran late.
    pub overruns: u32,
}

/// `status`, 5 to 20 times a second.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Status {
    /// The line's sequence number.
    pub seq: u64,
    /// ms since the board started.
    pub up_ms: u64,
    /// None before the first good frame.
    pub rx: Option<Rx>,
    /// What it asked of each device.
    pub out: Out,
    /// The lowest of each since the last line.
    pub low: Option<Extremes>,
    /// The highest of each since the last line.
    pub high: Option<Extremes>,
    /// Each Jrk's position.
    #[serde(default)]
    pub pos: Positions,
    /// The link's counts.
    #[serde(default)]
    pub link: LinkCounts,
    /// The board's state.
    pub state: BoardState,
    /// The tick's timing.
    #[serde(default)]
    pub tick: Tick,
}

/// `event`: what changed.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Event {
    /// The line's sequence number.
    pub seq: u64,
    /// ms since the board started.
    pub up_ms: u64,
    /// What changed.
    pub what: String,
    /// What it was.
    #[serde(default)]
    pub from: String,
    /// What it is now.
    #[serde(default)]
    pub to: String,
}

/// Reads one line. Blank lines are an error too: the caller skips them.
///
/// # Errors
///
/// [`LineError`] if the line isn't JSON, or a key every line has is missing.
pub fn parse_line(line: &str) -> Result<StatusLine, LineError> {
    let value: Value = serde_json::from_str(line)?;
    let kind = value
        .get("t")
        .and_then(Value::as_str)
        .ok_or(LineError::Missing("t"))?
        .to_owned();
    let schema = value
        .get("schema")
        .and_then(Value::as_u64)
        .ok_or(LineError::Missing("schema"))?;
    let seq = value
        .get("seq")
        .and_then(Value::as_u64)
        .ok_or(LineError::Missing("seq"))?;
    value
        .get("up_ms")
        .and_then(Value::as_u64)
        .ok_or(LineError::Missing("up_ms"))?;
    let unknown = || {
        StatusLine::Unknown(Unknown {
            kind: kind.clone(),
            schema,
            seq,
        })
    };
    if schema != SCHEMA {
        return Ok(unknown());
    }
    Ok(match kind.as_str() {
        "hb" => StatusLine::Heartbeat(serde_json::from_value(value)?),
        "status" => StatusLine::Status(Box::new(serde_json::from_value(value)?)),
        "event" => StatusLine::Event(serde_json::from_value(value)?),
        _ => unknown(),
    })
}

/// Lines lost between two sequence numbers (0: none).
#[must_use]
pub fn lost_between(last: u64, next: u64) -> u64 {
    next.saturating_sub(last).saturating_sub(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The contract's golden lines: every reader tests against them.
    const GOLDEN: &str = include_str!("../../../../../../firmware/status/golden.jsonl");

    fn golden() -> Vec<StatusLine> {
        GOLDEN
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| parse_line(l).unwrap_or_else(|e| panic!("{e}: {l}")))
            .collect()
    }

    #[test]
    fn every_golden_line_reads() {
        let lines = golden();
        assert_eq!(lines.len(), 7);
        let seqs: Vec<u64> = lines.iter().map(StatusLine::seq).collect();
        assert_eq!(seqs, vec![812, 813, 814, 815, 816, 817, 818]);
        // config, devices and health (7 Nov): kinds this panel doesn't read yet.
        let kinds: Vec<&str> = lines[4..]
            .iter()
            .map(|l| match l {
                StatusLine::Unknown(u) => u.kind.as_str(),
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(kinds, ["config", "devices", "health"]);
        match &lines[0] {
            StatusLine::Heartbeat(h) => {
                assert_eq!((h.up_ms, h.boot_id.as_str()), (40210, "a1b2c3d4"));
                assert_eq!(h.last_reset, "PowerOn");
            }
            other => panic!("{other:?}"),
        }
        match &lines[1] {
            StatusLine::Status(s) => {
                let rx = s.rx.expect("a frame");
                assert_eq!((rx.v, rx.counter, rx.check, rx.age_ms), (2, 41, 17, 18));
                assert_eq!((rx.steer, rx.speed, rx.accel), (-0.1, 0.3, 0.0));
                assert_eq!(s.out.steer_counts, 1755);
                assert_eq!(s.low.map(|l| l.dac), Some(204));
                assert_eq!(s.pos.steer, Some(1762));
                assert_eq!(s.pos.brake, None);
                assert_eq!(s.link.frames, 12040);
                assert_eq!(s.state.mode, BoardMode::Computer);
                assert_eq!(s.tick.worst_gap_us, 10840);
            }
            other => panic!("{other:?}"),
        }
        match &lines[2] {
            StatusLine::Status(s) => {
                assert_eq!(s.rx, None, "no frame yet");
                assert_eq!(s.state.mode, BoardMode::Stopped);
                assert_eq!(s.state.faults, 256);
            }
            other => panic!("{other:?}"),
        }
        match &lines[3] {
            StatusLine::Event(e) => {
                assert_eq!(
                    (e.what.as_str(), e.from.as_str(), e.to.as_str()),
                    ("mode", "rc", "computer")
                );
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn unknown_fields_are_ignored_and_unknown_schemas_said() {
        let golden_status = GOLDEN.lines().nth(1).expect("a status line");
        let more = golden_status.replacen("{\"t\"", "{\"new_field\":[1,2],\"t\"", 1);
        assert!(matches!(parse_line(&more), Ok(StatusLine::Status(_))));
        let next = golden_status.replace("\"schema\":1", "\"schema\":2");
        match parse_line(&next) {
            Ok(StatusLine::Unknown(u)) => {
                assert_eq!(u.schema, 2);
                assert!(u.to_string().contains("unknown"));
            }
            other => panic!("{other:?}"),
        }
        let kind = r#"{"t":"scan","schema":1,"seq":5,"up_ms":1}"#;
        assert!(matches!(parse_line(kind), Ok(StatusLine::Unknown(_))));
        let mode = golden_status.replace("\"mode\":\"computer\"", "\"mode\":\"warp\"");
        match parse_line(&mode) {
            Ok(StatusLine::Status(s)) => assert_eq!(s.state.mode, BoardMode::Other),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn bad_lines_are_errors() {
        assert!(matches!(parse_line("not json"), Err(LineError::Json(_))));
        assert!(matches!(
            parse_line(r#"{"schema":1,"seq":1,"up_ms":1}"#),
            Err(LineError::Missing("t"))
        ));
        assert!(matches!(
            parse_line(r#"{"t":"hb","schema":1,"up_ms":1}"#),
            Err(LineError::Missing("seq"))
        ));
        // A status with a field of the wrong type.
        let golden_status = GOLDEN.lines().nth(1).expect("a status line");
        let broken = golden_status.replace("\"dac\":210,", "\"dac\":\"x\",");
        assert!(parse_line(&broken).is_err());
    }

    #[test]
    fn gaps_in_seq() {
        assert_eq!(lost_between(812, 813), 0);
        assert_eq!(lost_between(812, 816), 3);
        assert_eq!(lost_between(900, 5), 0, "a restart is no loss");
    }
}

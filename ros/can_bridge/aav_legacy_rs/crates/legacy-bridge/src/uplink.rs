//! What the bridge reads back from the ESP32: its uplink frames (`uplink_reader.py`, and
//! `legacy_protocol.py`'s `UplinkParser`).
//!
//! ```text
//!   ESP32 TX1 ──35 or 40-byte frames, 50 Hz──> serial port ──> UplinkReader::feed(bytes, now)
//!                                                                │
//!        status / steering_state / brake_state / throttle_state / rc / raw <── each frame
//!        calibration_line, health <── the reader
//! ```
//!
//! Each frame is decoded by the firmware's own code (legacy-link's `Uplink::decode`).

use legacy_link::{CalState, Refusal, Uplink, SYNC, UPLINK_LEN, UPLINK_V2_LEN, VERSION, VERSION_3};

use crate::relay::CalRefusal;

/// An uplink frame's length for its version byte, if the version is known.
#[must_use]
pub fn uplink_length(version: u8) -> Option<usize> {
    match version {
        VERSION => Some(UPLINK_V2_LEN),
        VERSION_3 => Some(UPLINK_LEN),
        _ => None,
    }
}

/// Bytes in, uplink frames out (version 2 or 3), as the Python's `UplinkParser`.
///
/// Wait for 0xA5; the version byte after it gives the length; take that many. A bad
/// frame or an unknown version is dropped (counted in `bad`), and the search starts again
/// at the next 0xA5 inside it. Bytes outside a frame count in `skipped`.
#[derive(Debug, Clone, Default)]
pub struct UplinkParser {
    buf: Vec<u8>,
    /// Frames turned down.
    pub bad: u64,
    /// Bytes outside a frame.
    pub skipped: u64,
}

impl UplinkParser {
    /// A parser with nothing buffered.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The good frames in `data` (with what came before), oldest first.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Uplink> {
        let mut out = Vec::new();
        for &b in data {
            if self.buf.is_empty() && b != SYNC {
                self.skipped += 1;
                continue;
            }
            self.buf.push(b);
            let Some(&version) = self.buf.get(1) else {
                continue;
            };
            let length = uplink_length(version);
            if length.is_some_and(|n| self.buf.len() < n) {
                continue;
            }
            match length.map(|_| Uplink::decode(&self.buf)) {
                Some(Ok(u)) => {
                    out.push(u);
                    self.buf.clear();
                }
                _ => {
                    self.bad += 1;
                    let rest = &self.buf[1..];
                    self.buf = match rest.iter().position(|&x| x == SYNC) {
                        Some(at) => rest[at..].to_vec(),
                        None => Vec::new(),
                    };
                }
            }
        }
        out
    }
}

/// The latest uplink frame, when it came, and the counts since the start.
#[derive(Debug, Clone)]
pub struct UplinkReader {
    /// s: an older frame is stale.
    pub timeout: f64,
    parser: UplinkParser,
    latest: Option<Uplink>,
    at: Option<f64>,
    frames: u64,
    lost: u64,
}

impl UplinkReader {
    /// A reader that has heard nothing.
    #[must_use]
    pub fn new(timeout: f64) -> Self {
        UplinkReader {
            timeout,
            parser: UplinkParser::new(),
            latest: None,
            at: None,
            frames: 0,
            lost: 0,
        }
    }

    /// Takes the bytes read at `now` (s); the frames they completed.
    pub fn feed(&mut self, data: &[u8], now: f64) -> Vec<Uplink> {
        let found = self.parser.feed(data);
        for u in &found {
            if let Some(last) = &self.latest {
                let gap = u.sequence.wrapping_sub(last.sequence).wrapping_sub(1);
                self.lost += u64::from(gap);
            }
            self.latest = Some(*u);
            self.at = Some(now);
            self.frames += 1;
        }
        found
    }

    /// True if a frame came within the timeout.
    #[must_use]
    pub fn fresh(&self, now: f64) -> bool {
        self.at.is_some_and(|at| now - at <= self.timeout)
    }

    /// The latest frame, if any.
    #[must_use]
    pub fn latest(&self) -> Option<&Uplink> {
        self.latest.as_ref()
    }

    /// `UplinkHealth`'s fields: how the frames are arriving.
    #[must_use]
    pub fn health(&self, now: f64) -> Health {
        Health {
            fresh: self.fresh(now),
            frames: low32(self.frames),
            lost: low32(self.lost),
            bad: low32(self.parser.bad),
        }
    }
}

/// The low 32 bits, as the Python's `& 0xFFFFFFFF`.
fn low32(n: u64) -> u32 {
    (n & u64::from(u32::MAX)) as u32
}

/// `aav_interfaces/UplinkHealth`'s fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Health {
    /// A frame came within the timeout.
    pub fresh: bool,
    /// Good frames.
    pub frames: u32,
    /// Gaps in the sequence numbers.
    pub lost: u32,
    /// Frames turned down.
    pub bad: u32,
}

/// `aav_interfaces/VehicleStatus`'s fields for a frame. That ESP32 has no wheel speed: NaN.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Status {
    /// `VehicleStatus.mode`'s numbers: 0 stopped, 1 RC, 2 computer.
    pub mode: u8,
    /// The ESP32 takes the computer's frames.
    pub armed: bool,
    /// Any stop.
    pub estop_active: bool,
    /// m/s: NaN, not measured here.
    pub speed: f32,
    /// rad, + left; NaN unknown.
    pub steering_angle: f32,
    /// 0..1.
    pub throttle: f32,
    /// 0..1.
    pub brake: f32,
    /// The fault bits.
    pub faults: u16,
}

/// A frame's status.
#[must_use]
pub fn status(u: &Uplink) -> Status {
    Status {
        mode: u.mode.to_byte(),
        armed: u.armed,
        estop_active: u.estop,
        speed: f32::NAN,
        steering_angle: u.steering.unwrap_or(f32::NAN),
        throttle: u.throttle_out,
        brake: u.brake_out,
        faults: u.faults,
    }
}

/// `/vehicle/steering`'s (command, measured): ego's angle (None: no fresh one), the Jrk's.
#[must_use]
pub fn steering_state(u: &Uplink, command: Option<f64>) -> (f32, f32) {
    (
        command.map_or(f32::NAN, |c| c as f32),
        u.steering.unwrap_or(f32::NAN),
    )
}

/// `/vehicle/brake`'s (command, measured): what the ESP32 drives, the Jrk's position.
#[must_use]
pub fn brake_state(u: &Uplink) -> (f32, f32) {
    (u.brake_out, u.brake_position.unwrap_or(f32::NAN))
}

/// `/vehicle/throttle`'s (command, measured): the uplink's `throttle_out` (before the
/// firmware's ramp), and NaN: no sensor reads the throttle.
#[must_use]
pub fn throttle_state(u: &Uplink) -> (f32, f32) {
    (u.throttle_out, f32::NAN)
}

/// `aav_interfaces/RcSticks`'s fields.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rc {
    /// `RcSticks.SWITCH_*`.
    pub mode_switch: u8,
    /// -1..1, + forward.
    pub throttle: f32,
    /// -1..1, + left.
    pub steering: f32,
    /// The left stick in its brake band.
    pub brake: bool,
}

/// A frame's RC sticks: the same numbers.
#[must_use]
pub fn rc(u: &Uplink) -> Rc {
    Rc {
        mode_switch: u.switch.to_byte(),
        throttle: u.sticks.throttle,
        steering: u.sticks.steering,
        brake: u.sticks.brake,
    }
}

/// `aav_interfaces/ActuatorRaw`'s fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Raw {
    /// A position is known.
    pub valid: bool,
    /// Counts, 0 when not valid.
    pub position: u16,
    /// `ActuatorRaw.CALIBRATION_*`.
    pub calibration: u8,
}

/// An actuator's raw position (None: not known), and the calibration's state.
#[must_use]
pub fn raw(counts: Option<u16>, calibration: CalState) -> Raw {
    Raw {
        valid: counts.is_some(),
        position: counts.unwrap_or(0),
        calibration: calibration.to_byte(),
    }
}

/// The words for a calibration state (`legacy_protocol.CAL_STATES`).
#[must_use]
pub fn cal_words(state: CalState) -> &'static str {
    match state {
        CalState::Off => "OFF",
        CalState::Limp => "LIMP",
        CalState::Jog => "JOG",
        CalState::Hold => "HOLD",
        CalState::Return => "RETURN",
        CalState::Refused(Refusal::Moving) => "REFUSED - the speed command is not zero",
        CalState::Refused(Refusal::NotBraked) => "REFUSED - not braked, or the throttle is on",
        CalState::Refused(Refusal::NoFeedback) => {
            "REFUSED - no position from the steering actuator \
             (the feedback of its Jrk must be analog)"
        }
        CalState::Refused(Refusal::Estop) => "REFUSED - e-stop",
        CalState::Refused(Refusal::OtherDriver) => "REFUSED - the RC (direct) or a test drives",
    }
}

/// `/vehicle/steering_actuator/calibration`'s line for people.
///
/// `wanted`: the panel asks for limp or jog; `refusal`: why the bridge doesn't pass it on
/// (None: it does); `latest`: the latest uplink, for the firmware's own state.
#[must_use]
pub fn calibration_line(
    wanted: bool,
    refusal: Option<CalRefusal>,
    latest: Option<&Uplink>,
) -> String {
    if let (true, Some(r)) = (wanted, refusal) {
        return format!("REFUSED - {r}");
    }
    let state = latest.map_or(CalState::Off, |u| u.calibration);
    let mut words = cal_words(state).to_string();
    if state == CalState::Limp {
        words += " - the steering actuator's motor is off: turn the wheels by hand";
    }
    if let Some(counts) = latest.and_then(|u| u.counts.steering) {
        words += &format!(" (at {counts} counts)");
    }
    words
}

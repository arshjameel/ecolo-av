//! What the bridge says it wrote to the ESP32 (`/control/car/link_frame`; `link_report.py`).
//!
//! ```text
//!   Relay::frame(now) ─> port write ─> LinkReport::wrote(frame, sent) ─> counts every frame
//!                                                   │
//!                  a sample, link_frame_rate_hz <───┘  frame_fields: decoded from the bytes
//! ```
//!
//! The values come from the bytes that went out, decoded by the firmware's own code
//! (legacy-link), not from what the relay meant to send, so a packing fault shows up here.
//! `written` means the kernel took the bytes, no more.

use core::fmt;

use legacy_link::{
    CalRequest, Frame, FrameError, FrameV2, FrameV3, FRAME_LEN, FRAME_V2_LEN, FRAME_V3_LEN,
};

/// `aav_interfaces/LinkFrame`'s frame fields for one written frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkFields {
    /// 1, 2 or 3.
    pub version: u8,
    /// The Orin's counter; 0 in version 1.
    pub counter: u8,
    /// False in version 1.
    pub armed: bool,
    /// False in version 1.
    pub estop: bool,
    /// -1..1.
    pub steering: f32,
    /// -1..1.
    pub speed: f32,
    /// -1..1.
    pub accel: f32,
    /// Version 3: 0 off, 1 limp, 2 jog.
    pub cal_mode: u8,
    /// Version 3: counts; 0 otherwise.
    pub jog_target: u16,
    /// The frame's last byte.
    pub check: u8,
}

/// Why a written frame can't be reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkError {
    /// No version has this many bytes.
    Length(usize),
    /// The bytes aren't a good frame of their length.
    Frame(FrameError),
}

impl fmt::Display for LinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LinkError::Length(n) => {
                write!(f, "a frame of {n} bytes: no version has that length")
            }
            LinkError::Frame(e) => write!(f, "a bad frame: {e:?}"),
        }
    }
}

impl std::error::Error for LinkError {}

impl From<FrameError> for LinkError {
    fn from(e: FrameError) -> Self {
        LinkError::Frame(e)
    }
}

/// A written frame's fields. The length says the version (13, 17, 20 bytes).
///
/// # Errors
///
/// [`LinkError`] if the bytes aren't a good frame.
pub fn frame_fields(frame: &[u8]) -> Result<LinkFields, LinkError> {
    let check = frame.last().copied().unwrap_or(0);
    let fields = |version, values: Frame, counter, armed, estop, cal: CalRequest| {
        let (cal_mode, jog_target) = match cal {
            CalRequest::Off => (0, 0),
            CalRequest::Limp => (1, 0),
            CalRequest::Jog(t) => (2, t),
        };
        LinkFields {
            version,
            counter,
            armed,
            estop,
            steering: values.steering,
            speed: values.speed,
            accel: values.accel,
            cal_mode,
            jog_target,
            check,
        }
    };
    match frame.len() {
        FRAME_LEN => {
            let f = Frame::decode(
                frame
                    .try_into()
                    .map_err(|_| LinkError::Length(frame.len()))?,
            )?;
            Ok(fields(1, f, 0, false, false, CalRequest::Off))
        }
        FRAME_V2_LEN => {
            let f = FrameV2::decode(
                frame
                    .try_into()
                    .map_err(|_| LinkError::Length(frame.len()))?,
            )?;
            Ok(fields(
                2,
                f.values,
                f.counter,
                f.flags.armed,
                f.flags.estop,
                CalRequest::Off,
            ))
        }
        FRAME_V3_LEN => {
            let f = FrameV3::decode(
                frame
                    .try_into()
                    .map_err(|_| LinkError::Length(frame.len()))?,
            )?;
            Ok(fields(
                3,
                f.values,
                f.counter,
                f.flags.armed,
                f.flags.estop,
                f.calibration,
            ))
        }
        n => Err(LinkError::Length(n)),
    }
}

/// Counts every write, and says when the next sample is due (`LinkReport`).
#[derive(Debug, Clone)]
pub struct LinkReport {
    period: Option<f64>,
    next_due: Option<f64>,
    written: u64,
    write_errors: u64,
}

/// A negative `link_frame_rate_hz`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BadRate(pub f64);

impl fmt::Display for BadRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "link_frame_rate_hz {}: use 0 (off) or more", self.0)
    }
}

impl std::error::Error for BadRate {}

impl LinkReport {
    /// A report sampling at `rate_hz` (0: no samples; the counts still run).
    ///
    /// # Errors
    ///
    /// [`BadRate`] for a negative rate.
    pub fn new(rate_hz: f64) -> Result<Self, BadRate> {
        if rate_hz < 0.0 {
            return Err(BadRate(rate_hz));
        }
        Ok(LinkReport {
            period: (rate_hz > 0.0).then(|| 1.0 / rate_hz),
            next_due: None,
            written: 0,
            write_errors: 0,
        })
    }

    /// Notes a write that took `sent` bytes; true if the whole frame went.
    ///
    /// Part of a frame isn't a frame: the ESP32 can't count it, so neither do we.
    pub fn wrote(&mut self, frame: &[u8], sent: usize) -> bool {
        if sent == frame.len() {
            self.written += 1;
            true
        } else {
            self.write_errors += 1;
            false
        }
    }

    /// Notes a write that failed: nothing counts as written.
    pub fn failed(&mut self) {
        self.write_errors += 1;
    }

    /// True if a sample is due at `now` (s); starts the next period.
    pub fn due(&mut self, now: f64) -> bool {
        let Some(period) = self.period else {
            return false;
        };
        match self.next_due {
            Some(next) if now - next < period => {
                if now >= next {
                    // Keeps the rate through a tick's jitter.
                    self.next_due = Some(next + period);
                    true
                } else {
                    false
                }
            }
            _ => {
                // The first sample, or a gap (the port was closed): start again from now.
                self.next_due = Some(now + period);
                true
            }
        }
    }

    /// Frames written since the start (the low 32 bits).
    #[must_use]
    pub fn written(&self) -> u32 {
        (self.written & u64::from(u32::MAX)) as u32
    }

    /// Writes that failed since the start (the low 32 bits).
    #[must_use]
    pub fn write_errors(&self) -> u32 {
        (self.write_errors & u64::from(u32::MAX)) as u32
    }
}

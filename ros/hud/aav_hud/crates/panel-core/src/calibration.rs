//! The steering calibration page: find the steering actuator's ends and centre, work out
//! equal limits, and check them lock to lock (docs/actuator-tests.md, "Find the steering
//! actuator's ends and centre", way B).
//!
//! ```text
//! /vehicle/steering_actuator/raw (counts, firmware state) ─┐
//! the e-stop, the speed seen ──► readiness (allowed, or why not)
//! presses: LIMP, HOLD, STEP, MARK, CHECK, END ──► CalPage ──► a Request, 10 a second
//!                                                  marks ──► steering_limits ──► the lines to copy
//! ```
//!
//! | Mode | What goes to `/drivers/buttons/steering_calibration` |
//! |---|---|
//! | [`Mode::Off`] | nothing |
//! | [`Mode::Limp`] | `MODE_LIMP`: the motor off, the wheels turned by hand |
//! | [`Mode::Jog`] | `MODE_JOG` to a target: HOLD (target = where the wheels are), STEP (± `jog_step`) |
//! | [`Mode::Sweep`] | `MODE_JOG`, the lock-to-lock check's target ([`crate::sweep`]) |
//! | [`Mode::Ending`] | `MODE_OFF` for 0.5 s, then nothing: the firmware moves back slowly |
//!
//! Its rules:
//!
//! - nothing is asked while the page isn't allowed: an e-stop, the car moving, the speed
//!   or the position not seen. Going not-allowed while asking ends the mode (the check
//!   stops and holds first); a press is needed to start again;
//! - a jog target is never outside `jog_min`..`jog_max` (the firmware's own software
//!   limits today; it clamps too); the check's targets never leave the computed limits;
//! - the firmware and the bridge decide again (standing, braked, analog feedback,
//!   `steering_calibration: stands`): their refusal is shown, and stops the check.

use std::{
    fmt,
    time::{Duration, Instant},
};

use crate::{
    steering_limits::{self, Limits, Mark, Marks, MeasurementError},
    sweep::{Outcome, Sweep, SweepSettings},
};

/// Why the firmware refused a mode (legacy-link's `Refusal`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    Moving,
    NotBraked,
    NoFeedback,
    Estop,
    OtherDriver,
}

/// The firmware's steering calibration state: ActuatorRaw's `calibration` (legacy-link's
/// `CalState`, the uplink's byte 38).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FirmwareState {
    #[default]
    Off,
    Limp,
    Jog,
    Hold,
    Return,
    Refused(Refusal),
    /// A number this panel doesn't know: a newer firmware.
    Unknown(u8),
}

impl FirmwareState {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::Off,
            1 => Self::Limp,
            2 => Self::Jog,
            3 => Self::Hold,
            4 => Self::Return,
            5 => Self::Refused(Refusal::Moving),
            6 => Self::Refused(Refusal::NotBraked),
            7 => Self::Refused(Refusal::NoFeedback),
            8 => Self::Refused(Refusal::Estop),
            9 => Self::Refused(Refusal::OtherDriver),
            other => Self::Unknown(other),
        }
    }
}

impl fmt::Display for FirmwareState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Off => f.write_str("OFF (normal steering)"),
            Self::Limp => f.write_str("LIMP (the motor off)"),
            Self::Jog => f.write_str("JOG"),
            Self::Hold => f.write_str("HOLD (no fresh frames: holding)"),
            Self::Return => f.write_str("RETURN (moving back slowly)"),
            Self::Refused(r) => f.write_str(match r {
                Refusal::Moving => "REFUSED: a speed is asked for",
                Refusal::NotBraked => "REFUSED: not braked, or the throttle on",
                Refusal::NoFeedback => "REFUSED: no position (the Jrk's feedback isn't analog)",
                Refusal::Estop => "REFUSED: an e-stop",
                Refusal::OtherDriver => "REFUSED: the RC or a test drives",
            }),
            Self::Unknown(n) => write!(f, "state {n} (unknown to this panel)"),
        }
    }
}

/// One `/vehicle/steering_actuator/raw` message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Raw {
    /// Counts, if the firmware has a position.
    pub position: Option<u16>,
    pub state: FirmwareState,
}

/// Why the page may not ask for anything now.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NotReady {
    Estop,
    Moving(f64),
    SpeedUnseen,
    /// No fresh `/vehicle/steering_actuator/raw`.
    NoRaw,
    /// It comes, but without a position (the feedback isn't analog, or a version 2 uplink).
    NoPosition,
}

impl fmt::Display for NotReady {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Estop => f.write_str("the e-stop is on"),
            Self::Moving(v) => write!(f, "the car moves ({v:+.2} m/s)"),
            Self::SpeedUnseen => f.write_str("the speed isn't seen (/vehicle/status)"),
            Self::NoRaw => f.write_str(
                "no steering actuator position (the bridge needs uplink true, frame_version 3)",
            ),
            Self::NoPosition => {
                f.write_str("the steering actuator gives no position (feedback not analog?)")
            }
        }
    }
}

/// Below this speed (m/s) the car stands still: the bridge's own rule.
pub const STANDSTILL: f64 = 0.05;

/// What the page needs to know to be allowed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Situation {
    pub estop: bool,
    /// The speed seen (m/s), while fresh.
    pub speed: Option<f64>,
    /// The latest raw message, while fresh.
    pub raw: Option<Raw>,
}

/// Every reason the page may not ask now; empty: allowed.
pub fn readiness(s: &Situation) -> Vec<NotReady> {
    let mut why = Vec::new();
    if s.estop {
        why.push(NotReady::Estop);
    }
    match s.speed {
        None => why.push(NotReady::SpeedUnseen),
        Some(v) if v.abs() >= STANDSTILL => why.push(NotReady::Moving(v)),
        Some(_) => {}
    }
    match s.raw {
        None => why.push(NotReady::NoRaw),
        Some(Raw { position: None, .. }) => why.push(NotReady::NoPosition),
        Some(_) => {}
    }
    why
}

/// What goes to `/drivers/buttons/steering_calibration` (SteeringCalibration's modes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Off,
    Limp,
    /// Counts.
    Jog(u16),
}

/// A press on the page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalPress {
    Limp,
    Hold,
    StepLeft,
    StepRight,
    Mark(Mark),
    ClearMarks,
    Check,
    End,
}

/// The page's numbers (config/hw_panel.yaml).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalSettings {
    /// Counts a STEP.
    pub jog_step: u16,
    /// Counts: no jog target outside these (the firmware's software limits).
    pub jog_min: u16,
    pub jog_max: u16,
    pub sweep: SweepSettings,
}

/// What the page is doing.
#[derive(Debug, Clone, PartialEq)]
pub enum Mode {
    Off,
    Limp,
    Jog {
        target: u16,
    },
    Sweep(Box<Sweep>),
    /// Ended: `MODE_OFF` until then.
    Ending {
        until: Instant,
    },
}

/// How long END sends `MODE_OFF`.
const ENDING: Duration = Duration::from_millis(500);

/// The page.
#[derive(Debug, Clone)]
pub struct CalPage {
    settings: CalSettings,
    mode: Mode,
    left: Option<u16>,
    centre: Option<u16>,
    right: Option<u16>,
    /// The last thing to tell the person (a refusal, a mark, the check's result).
    note: Option<String>,
    /// The lock-to-lock check's last result.
    checked: Option<Outcome>,
}

impl CalPage {
    pub fn new(settings: CalSettings) -> Self {
        Self {
            settings,
            mode: Mode::Off,
            left: None,
            centre: None,
            right: None,
            note: None,
            checked: None,
        }
    }

    pub fn mode(&self) -> &Mode {
        &self.mode
    }

    pub fn settings(&self) -> &CalSettings {
        &self.settings
    }

    pub fn note(&self) -> Option<&str> {
        self.note.as_deref()
    }

    pub fn checked(&self) -> Option<&Outcome> {
        self.checked.as_ref()
    }

    /// Whether anything is being asked of the firmware (limp, jog, the check).
    pub fn is_asking(&self) -> bool {
        matches!(self.mode, Mode::Limp | Mode::Jog { .. } | Mode::Sweep(_))
    }

    pub fn mark(&self, m: Mark) -> Option<u16> {
        match m {
            Mark::Left => self.left,
            Mark::Centre => self.centre,
            Mark::Right => self.right,
        }
    }

    /// The three marks (left, centre, right), once all are there.
    pub fn marks(&self) -> Option<(u16, u16, u16)> {
        Some((self.left?, self.centre?, self.right?))
    }

    /// The limits from the three marks, once all are there.
    pub fn limits(&self) -> Option<Result<Limits, MeasurementError>> {
        let marks = Marks {
            left: self.left?,
            centre: self.centre?,
            right: self.right?,
            left_deg: None,
            right_deg: None,
        };
        Some(steering_limits::compute(&marks))
    }

    /// +1 if steering left raises the count: from the marks once they say, otherwise the
    /// firmware's guess today (calibration.rs: 3100 counts a radian, positive left).
    pub fn left_sign(&self) -> i32 {
        let known = |a: u16, b: u16| if a > b { 1 } else { -1 };
        match (self.left, self.centre, self.right) {
            (Some(l), Some(c), _) if l != c => known(l, c),
            (_, Some(c), Some(r)) if r != c => known(c, r),
            (Some(l), _, Some(r)) if l != r => known(l, r),
            _ => 1,
        }
    }

    /// A target inside the jog range.
    fn clamp(&self, target: i32) -> u16 {
        let (lo, hi) = (
            i32::from(self.settings.jog_min),
            i32::from(self.settings.jog_max),
        );
        // In 0..=4095 after the clamp (jog_max is a u16), so it fits.
        target.clamp(lo, hi) as u16
    }

    /// A person's press. `position`: the wheels now (counts), if seen; `ready`: from
    /// [`readiness`].
    pub fn press(&mut self, p: CalPress, now: Instant, position: Option<u16>, ready: &[NotReady]) {
        let blocked = |page: &mut Self| -> bool {
            if let Some(why) = ready.first() {
                page.note = Some(format!("not now: {why}"));
                true
            } else {
                false
            }
        };
        match p {
            CalPress::Limp => {
                if !blocked(self) {
                    self.stop_check("LIMP pressed");
                    self.mode = Mode::Limp;
                    self.note = Some("limp: turn the wheels by hand, gently".into());
                }
            }
            CalPress::Hold => {
                self.stop_check("HOLD pressed");
                if blocked(self) {
                    return;
                }
                if let Some(at) = position {
                    let target = self.clamp(i32::from(at));
                    self.mode = Mode::Jog { target };
                    self.note = Some(format!("holding at {target} counts"));
                }
            }
            CalPress::StepLeft | CalPress::StepRight => {
                if blocked(self) {
                    return;
                }
                let Some(at) = position else { return };
                let base = match self.mode {
                    Mode::Jog { target } => target,
                    _ => at,
                };
                let dir = if p == CalPress::StepLeft { 1 } else { -1 } * self.left_sign();
                let step = i32::from(self.settings.jog_step) * dir;
                let target = self.clamp(i32::from(base) + step);
                self.stop_check("STEP pressed");
                self.mode = Mode::Jog { target };
                self.note = if target == self.settings.jog_min || target == self.settings.jog_max {
                    Some(format!(
                        "{target}: the end of the jog range ({} to {}); use LIMP to go further \
                         by hand",
                        self.settings.jog_min, self.settings.jog_max
                    ))
                } else {
                    None
                };
            }
            CalPress::Mark(m) => match position {
                Some(at) => {
                    match m {
                        Mark::Left => self.left = Some(at),
                        Mark::Centre => self.centre = Some(at),
                        Mark::Right => self.right = Some(at),
                    }
                    self.checked = None;
                    self.note = Some(format!("marked {m} at {at} counts"));
                }
                None => self.note = Some("can't mark: no position".into()),
            },
            CalPress::ClearMarks => {
                (self.left, self.centre, self.right) = (None, None, None);
                self.checked = None;
                self.note = Some("marks cleared".into());
            }
            CalPress::Check => {
                if blocked(self) {
                    return;
                }
                match self.limits() {
                    Some(Ok(limits)) => {
                        match Sweep::start(now, position, &limits, self.settings.sweep) {
                            Ok(sweep) => {
                                self.mode = Mode::Sweep(Box::new(sweep));
                                self.checked = None;
                                self.note = Some("lock-to-lock check running".into());
                            }
                            Err(why) => self.note = Some(format!("can't check: {why}")),
                        }
                    }
                    Some(Err(e)) => self.note = Some(format!("can't check: {e}")),
                    None => self.note = Some("can't check: mark left, centre and right".into()),
                }
            }
            CalPress::End => {
                self.stop_check("END pressed");
                if self.mode != Mode::Off {
                    self.mode = Mode::Ending {
                        until: now + ENDING,
                    };
                    self.note = Some("ended: the wheels go back slowly".into());
                }
            }
        }
    }

    /// Ends a running check, keeping its result; the mode becomes a hold where it was.
    fn stop_check(&mut self, why: &str) {
        if let Mode::Sweep(sweep) = &mut self.mode {
            sweep.abort(why);
            self.finish_check();
        }
    }

    /// A check that has ended: keep its result, hold where the wheels were.
    fn finish_check(&mut self) {
        let Mode::Sweep(sweep) = &self.mode else {
            return;
        };
        let Some(outcome) = sweep.outcome().cloned() else {
            return;
        };
        let hold = match &outcome {
            Outcome::Stopped { position, .. } => *position,
            Outcome::Passed { .. } => sweep.target(),
        };
        self.note = Some(outcome.to_string());
        self.checked = Some(outcome);
        self.mode = Mode::Jog {
            target: self.clamp(i32::from(hold)),
        };
    }

    /// One frame: what to send now (None: nothing). `raw` and `ready` as for a press.
    pub fn tick(&mut self, now: Instant, raw: Option<Raw>, ready: &[NotReady]) -> Option<Request> {
        if let Some(why) = ready.first().filter(|_| self.is_asking()) {
            let why = why.to_string();
            self.stop_check(&why);
            self.mode = Mode::Ending {
                until: now + ENDING,
            };
            self.note = Some(format!("stopped: {why}"));
        }
        if let Mode::Sweep(sweep) = &mut self.mode {
            let position = raw.and_then(|r| r.position);
            let state = raw.map_or(FirmwareState::Off, |r| r.state);
            if let Some(target) = sweep.step(now, position, state) {
                return Some(Request::Jog(target));
            }
            self.finish_check();
        }
        match self.mode {
            Mode::Off | Mode::Sweep(_) => None,
            Mode::Limp => Some(Request::Limp),
            Mode::Jog { target } => Some(Request::Jog(target)),
            Mode::Ending { until } if now < until => Some(Request::Off),
            Mode::Ending { .. } => {
                self.mode = Mode::Off;
                None
            }
        }
    }
}

/// The lines a person copies, and where: written to a file by the app, never into
/// vehicle.yaml or the firmware (docs/actuator-tests.md: a person, in a pull request,
/// with the hardware lead).
pub fn result_file(limits: &Limits, marks: (u16, u16, u16), checked: Option<&Outcome>) -> String {
    let (l, c, r) = marks;
    let check = checked.map_or_else(|| "not run".to_owned(), ToString::to_string);
    format!(
        "# The steering actuator's ends and centre, from the hardware panel (hw_panel).\n\
         # Marked: left {l}, centre {c}, right {r} (counts).\n\
         # Lock-to-lock check: {check}\n\
         # Copy by hand, in a pull request, with this file and the sheet\n\
         # (docs/actuator-tests.md); the hardware lead reviews it.\n         # With the wheel angles (a protractor, each end), for the gain and\n         # max_steering_angle:\n         #   ros2 run aav_legacy steering_limits --left {l} --centre {c} --right {r} \\\n         #       --left-deg <mean> --right-deg <mean>\n\n{}\n",
        steering_limits::report(limits)
    )
}

/// A file name's time, UTC, from seconds since 1970: `2026-11-01T21-05-33Z`.
pub fn utc_stamp(unix_secs: u64) -> String {
    let (days, secs) = (unix_secs / 86_400, unix_secs % 86_400);
    // Howard Hinnant's civil_from_days, for days since 1970-01-01.
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}-{:02}-{:02}Z",
        secs / 3600,
        secs % 3600 / 60,
        secs % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings() -> CalSettings {
        CalSettings {
            jog_step: 25,
            jog_min: 400,
            jog_max: 3500,
            sweep: SweepSettings::default(),
        }
    }

    fn raw(at: u16, state: FirmwareState) -> Option<Raw> {
        Some(Raw {
            position: Some(at),
            state,
        })
    }

    #[test]
    fn readiness_says_every_reason() {
        let ok = Situation {
            estop: false,
            speed: Some(0.0),
            raw: raw(2000, FirmwareState::Off),
        };
        assert!(readiness(&ok).is_empty());
        let bad = Situation {
            estop: true,
            speed: Some(0.3),
            raw: Some(Raw {
                position: None,
                state: FirmwareState::Off,
            }),
        };
        assert_eq!(
            readiness(&bad),
            vec![NotReady::Estop, NotReady::Moving(0.3), NotReady::NoPosition]
        );
        let unseen = Situation {
            estop: false,
            speed: None,
            raw: None,
        };
        assert_eq!(
            readiness(&unseen),
            vec![NotReady::SpeedUnseen, NotReady::NoRaw]
        );
    }

    #[test]
    fn limp_hold_and_steps_ask_for_what_they_say() {
        let t = Instant::now();
        let mut page = CalPage::new(settings());
        assert_eq!(page.tick(t, raw(2000, FirmwareState::Off), &[]), None);
        page.press(CalPress::Limp, t, Some(2000), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Limp));
        page.press(CalPress::Hold, t, Some(2210), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Jog(2210)));
        // Left raises the count (the firmware's guess) until the marks say.
        page.press(CalPress::StepLeft, t, Some(2210), &[]);
        page.press(CalPress::StepLeft, t, Some(2215), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Jog(2260)));
        page.press(CalPress::Mark(Mark::Left), t, Some(1000), &[]);
        page.press(CalPress::Mark(Mark::Centre), t, Some(2000), &[]);
        assert_eq!(page.left_sign(), -1);
        page.press(CalPress::StepLeft, t, Some(2260), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Jog(2235)));
    }

    #[test]
    fn a_jog_never_leaves_the_jog_range() {
        let t = Instant::now();
        let mut page = CalPage::new(settings());
        page.press(CalPress::Hold, t, Some(3490), &[]);
        page.press(CalPress::StepLeft, t, Some(3490), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Jog(3500)));
        assert!(page.note().is_some_and(|n| n.contains("LIMP")));
        page.press(CalPress::Hold, t, Some(100), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Jog(400)));
    }

    #[test]
    fn not_ready_refuses_presses_and_ends_what_runs() {
        let t = Instant::now();
        let mut page = CalPage::new(settings());
        page.press(CalPress::Limp, t, Some(2000), &[NotReady::Estop]);
        assert_eq!(page.tick(t, None, &[NotReady::Estop]), None);
        assert!(page.note().is_some_and(|n| n.contains("e-stop")));
        page.press(CalPress::Limp, t, Some(2000), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Limp));
        let moving = [NotReady::Moving(0.2)];
        assert_eq!(page.tick(t, None, &moving), Some(Request::Off));
        assert_eq!(page.tick(t + ENDING, None, &moving), None);
        assert_eq!(*page.mode(), Mode::Off);
    }

    #[test]
    fn end_sends_off_for_half_a_second_then_nothing() {
        let t = Instant::now();
        let mut page = CalPage::new(settings());
        page.press(CalPress::End, t, Some(2000), &[]);
        assert_eq!(*page.mode(), Mode::Off);
        page.press(CalPress::Limp, t, Some(2000), &[]);
        page.press(CalPress::End, t, Some(2000), &[]);
        assert_eq!(page.tick(t, None, &[]), Some(Request::Off));
        assert_eq!(page.tick(t + Duration::from_millis(600), None, &[]), None);
    }

    #[test]
    fn marks_give_limits_and_the_file_to_copy() {
        let t = Instant::now();
        let mut page = CalPage::new(settings());
        assert!(page.limits().is_none());
        for (m, at) in [(Mark::Left, 3420), (Mark::Centre, 1985), (Mark::Right, 610)] {
            page.press(CalPress::Mark(m), t, Some(at), &[]);
        }
        let l = page.limits().expect("three marks").expect("limits");
        assert_eq!((l.lo, l.hi), (1985 - 1306, 1985 + 1306));
        let text = result_file(&l, (3420, 1985, 610), None);
        assert!(text.contains("min: 679") && text.contains("Lock-to-lock check: not run"));
        assert!(text.contains("hardware lead"));
        assert!(text.contains("steering_limits --left 3420 --centre 1985 --right 610"));
    }

    #[test]
    fn utc_stamps() {
        assert_eq!(utc_stamp(0), "1970-01-01T00-00-00Z");
        // 1 Nov 2026, 21:05:33 UTC.
        assert_eq!(utc_stamp(1_793_567_133), "2026-11-01T21-05-33Z");
        assert_eq!(utc_stamp(951_782_400), "2000-02-29T00-00-00Z");
    }

    #[test]
    fn the_check_runs_only_with_limits_and_holds_when_it_stops() {
        let t = Instant::now();
        let mut page = CalPage::new(settings());
        page.press(CalPress::Check, t, Some(2000), &[]);
        assert!(page.note().is_some_and(|n| n.contains("mark left")));
        for (m, at) in [(Mark::Left, 3420), (Mark::Centre, 1985), (Mark::Right, 610)] {
            page.press(CalPress::Mark(m), t, Some(at), &[]);
        }
        page.press(CalPress::Check, t, Some(1990), &[]);
        assert!(matches!(page.mode(), Mode::Sweep(_)));
        let jog = FirmwareState::Jog;
        assert!(matches!(
            page.tick(t, raw(1990, jog), &[]),
            Some(Request::Jog(_))
        ));
        // The firmware refuses: the check stops, and the page holds where the wheels are.
        let refused = FirmwareState::Refused(Refusal::NotBraked);
        let next = page.tick(t + Duration::from_millis(20), raw(1992, refused), &[]);
        assert_eq!(next, Some(Request::Jog(1992)));
        assert!(matches!(page.checked(), Some(Outcome::Stopped { .. })));
        assert!(page.note().is_some_and(|n| n.starts_with("STOPPED")));
        // HOLD during a check stops it too.
        page.press(CalPress::Check, t, Some(1992), &[]);
        page.press(CalPress::Hold, t, Some(1995), &[]);
        assert_eq!(page.tick(t, raw(1995, jog), &[]), Some(Request::Jog(1995)));
    }

    #[test]
    fn firmware_states_from_their_numbers() {
        assert_eq!(FirmwareState::from_byte(1), FirmwareState::Limp);
        assert_eq!(
            FirmwareState::from_byte(7),
            FirmwareState::Refused(Refusal::NoFeedback)
        );
        assert_eq!(FirmwareState::from_byte(42), FirmwareState::Unknown(42));
        assert!(FirmwareState::from_byte(6).to_string().contains("braked"));
    }
}

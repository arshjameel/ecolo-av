//! `--cal-self-test` (with `--demo` only): the calibration page driven through its own
//! buttons, for checks and screenshots. It goes limp, marks each end and the centre as
//! the demo's pretend hand rests there, holds at the centre, then runs the lock-to-lock
//! check, and prints what it did. With `--demo-bind`, the check must find the stiff spot.
//!
//! Never on a car: it waits for the demo's own counts.

use std::time::{Duration, Instant};

use panel_core::{
    calibration::{CalPage, CalPress, Mode},
    steering_limits::Mark,
    sweep::Outcome,
};

/// Where the demo's hand rests (demo.rs), and what it marks there.
const STOPS: [(u16, Mark); 3] = [(3420, Mark::Left), (1985, Mark::Centre), (610, Mark::Right)];
/// Close enough to a resting place to mark it.
const WITHIN: u16 = 2;
/// The longest it may take.
const TIMEOUT: Duration = Duration::from_secs(150);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Limp,
    /// Waiting for the hand at STOPS[i].
    Marking(usize),
    /// Waiting for the hand back at the centre, then HOLD.
    Centring,
    /// Holding: then the check.
    Holding,
    Checking,
    Done,
}

/// The script.
#[derive(Debug, Clone)]
pub struct CalScript {
    step: Step,
    started: Option<Instant>,
    done_at: Option<Instant>,
    printed: Option<Instant>,
    /// `--demo-bind`: the check must stop on resistance.
    expect_resistance: bool,
}

impl CalScript {
    pub fn new(expect_resistance: bool) -> Self {
        Self {
            step: Step::Limp,
            started: None,
            done_at: None,
            printed: None,
            expect_resistance,
        }
    }

    /// The press for this frame, if any.
    pub fn frame(
        &mut self,
        now: Instant,
        page: &CalPage,
        position: Option<u16>,
    ) -> Option<CalPress> {
        let started = *self.started.get_or_insert(now);
        if self.done_at.is_none() && now.saturating_duration_since(started) > TIMEOUT {
            println!("hw_panel cal-self-test: timed out at {:?}", self.step);
            self.step = Step::Done;
            self.done_at = Some(now);
            return Some(CalPress::End);
        }
        let at = position?;
        match self.step {
            Step::Limp => {
                self.step = Step::Marking(0);
                println!("hw_panel cal-self-test: limp");
                Some(CalPress::Limp)
            }
            Step::Marking(i) => {
                let (goal, mark) = STOPS[i];
                if at.abs_diff(goal) > WITHIN {
                    return None;
                }
                self.step = if i + 1 < STOPS.len() {
                    Step::Marking(i + 1)
                } else {
                    Step::Centring
                };
                println!("hw_panel cal-self-test: mark {mark} at {at}");
                Some(CalPress::Mark(mark))
            }
            Step::Centring => {
                if at.abs_diff(1985) > WITHIN {
                    return None;
                }
                self.step = Step::Holding;
                println!("hw_panel cal-self-test: hold at {at}");
                Some(CalPress::Hold)
            }
            Step::Holding => {
                if !matches!(page.mode(), Mode::Jog { .. }) {
                    return None;
                }
                self.step = Step::Checking;
                println!("hw_panel cal-self-test: lock-to-lock check");
                Some(CalPress::Check)
            }
            Step::Checking => {
                if !matches!(page.mode(), Mode::Sweep(_)) {
                    self.step = Step::Done;
                    self.done_at = Some(now);
                    let result = page
                        .checked()
                        .map_or_else(|| "no result".to_owned(), ToString::to_string);
                    println!("hw_panel cal-self-test: {result}");
                }
                None
            }
            Step::Done => None,
        }
    }

    /// One line every second while the check runs.
    pub fn print(&mut self, now: Instant, page: &CalPage, position: Option<u16>) {
        if self
            .printed
            .is_some_and(|t| now.saturating_duration_since(t) < Duration::from_secs(1))
        {
            return;
        }
        self.printed = Some(now);
        if let Mode::Sweep(s) = page.mode() {
            let p = position.map_or_else(|| "-".to_owned(), |p| p.to_string());
            println!(
                "hw_panel cal-self-test: {} target {} position {p}",
                s.leg(),
                s.target()
            );
        }
    }

    /// Ended, and the window has shown the result for 3 s.
    pub fn finished(&self, now: Instant) -> bool {
        self.done_at
            .is_some_and(|t| now.saturating_duration_since(t) >= Duration::from_secs(3))
    }

    /// Whether the check ended as it should: passed, or (`--demo-bind`) stopped on
    /// resistance.
    pub fn as_expected(&self, page: &CalPage) -> bool {
        match page.checked() {
            Some(Outcome::Passed { .. }) => !self.expect_resistance,
            Some(Outcome::Stopped { why, .. }) => {
                self.expect_resistance
                    && matches!(
                        why,
                        panel_core::sweep::Stop::Resistance { .. }
                            | panel_core::sweep::Stop::NoMovement { .. }
                    )
            }
            None => false,
        }
    }
}

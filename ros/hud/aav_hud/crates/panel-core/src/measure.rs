//! The MEASURE page (TODO 71): speed and distance, a tape measure typed in, and what it
//! says about `wheel_diameter` (docs/know-the-car.md, test 3: "10 m straight, by tape",
//! within 1 %).
//!
//! The panel never writes vehicle.yaml: SAVE writes a file for a person to copy, as the
//! calibration page does.
//!
//! ```text
//! RESET -> drive a straight run past a tape -> type the tape's metres -> the check
//!          (the odometer counts |speed| x time from /vehicle/status while it is fresh)
//! ```

use std::time::{Duration, Instant};

/// A gap in the speed longer than this isn't counted: the distance would be a guess.
pub const MAX_GAP: Duration = Duration::from_millis(500);
/// The check's limit (know-the-car: 1 %, 10 cm in 10 m).
pub const WITHIN: f64 = 0.01;
/// The shortest tape run worth a check (m): shorter, a wheel's turn is too much of it.
pub const SHORTEST_RUN: f64 = 2.0;

/// Distance and speed since RESET, from the measured speed.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Odometer {
    /// Metres driven, either way (|speed| x time).
    pub distance: f64,
    /// The fastest speed seen (m/s, either way).
    pub top: f64,
    /// Time with a known speed.
    pub timed: Duration,
    /// Gaps over `MAX_GAP`, or a speed unknown, after the first: not counted.
    pub gaps: u32,
    last: Option<(Instant, f64)>,
}

impl Odometer {
    /// Starts again from zero.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Counts one more look at the speed (`None`: unknown or stale now). The step uses
    /// the mean of the last speed and this one (the trapezoid rule).
    pub fn update(&mut self, speed: Option<f64>, now: Instant) {
        let Some(v) = speed.filter(|v| v.is_finite()) else {
            if self.last.take().is_some() {
                self.gaps += 1;
            }
            return;
        };
        if let Some((then, before)) = self.last {
            let dt = now.saturating_duration_since(then);
            if dt <= MAX_GAP {
                self.distance += 0.5 * (v.abs() + before.abs()) * dt.as_secs_f64();
                self.timed += dt;
            } else {
                self.gaps += 1;
            }
        }
        self.top = self.top.max(v.abs());
        self.last = Some((now, v));
    }

    /// The mean speed over the counted time (m/s), once there is some.
    pub fn mean(&self) -> Option<f64> {
        let t = self.timed.as_secs_f64();
        (t > 0.0).then(|| self.distance / t)
    }
}

/// A number typed in: digits and one point, at most 7 characters.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Entry {
    text: String,
}

impl Entry {
    /// What has been typed.
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Takes one typed character; anything but a digit or a first point is ignored.
    pub fn push(&mut self, c: char) {
        let point_ok = c == '.' && !self.text.contains('.');
        if (c.is_ascii_digit() || point_ok) && self.text.len() < 7 {
            self.text.push(c);
        }
    }

    /// Backspace.
    pub fn pop(&mut self) {
        self.text.pop();
    }

    pub fn clear(&mut self) {
        self.text.clear();
    }

    /// The number, if it is one above zero.
    pub fn value(&self) -> Option<f64> {
        self.text.parse().ok().filter(|v: &f64| *v > 0.0)
    }
}

/// What a tape run says about the wheel diameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WheelCheck {
    /// Not yet: the reason, for the screen.
    NotYet(&'static str),
    /// The wheels' distance against the tape's.
    Done {
        /// wheels / tape - 1: + means the wheels read long (diameter too big).
        error: f64,
        /// The diameter that would have read the tape exactly (m).
        suggested: f64,
        /// Within `WITHIN`.
        pass: bool,
    },
}

/// The check: the odometer's run against the tape, with vehicle.yaml's `diameter`.
pub fn wheel_check(odometer: &Odometer, tape: Option<f64>, diameter: Option<f64>) -> WheelCheck {
    let Some(diameter) = diameter.filter(|d| *d > 0.0) else {
        return WheelCheck::NotYet("vehicle.yaml has no wheel_diameter");
    };
    let Some(tape) = tape else {
        return WheelCheck::NotYet("type the tape's metres");
    };
    if tape < SHORTEST_RUN {
        return WheelCheck::NotYet("the tape run is under 2 m: too short to tell 1 %");
    }
    if odometer.gaps > 0 {
        return WheelCheck::NotYet("the speed had gaps: RESET and drive it again");
    }
    if odometer.distance < 0.5 * tape {
        return WheelCheck::NotYet("drive the run first (the wheels read under half the tape)");
    }
    let error = odometer.distance / tape - 1.0;
    WheelCheck::Done {
        error,
        suggested: diameter / (1.0 + error),
        pass: error.abs() <= WITHIN,
    }
}

/// SAVE's file: what was measured, and what to copy into vehicle.yaml if anything.
pub fn report(odometer: &Odometer, tape: Option<f64>, diameter: Option<f64>, when: &str) -> String {
    let mut out = format!(
        "# The hardware panel's MEASURE page, {when}\n\
         # A person copies what is needed into vehicle.yaml; the panel never writes it.\n\
         wheels_distance_m: {:.3}\n\
         top_speed_mps: {:.3}\n\
         timed_s: {:.2}\n\
         speed_gaps: {}\n",
        odometer.distance,
        odometer.top,
        odometer.timed.as_secs_f64(),
        odometer.gaps
    );
    if let Some(t) = tape {
        out += &format!("tape_m: {t:.3}\n");
    }
    if let Some(d) = diameter {
        out += &format!("wheel_diameter_now: {d:.4}\n");
    }
    match wheel_check(odometer, tape, diameter) {
        WheelCheck::NotYet(why) => out += &format!("check: not yet ({why})\n"),
        WheelCheck::Done {
            error,
            suggested,
            pass,
        } => {
            let verdict = if pass { "PASS" } else { "CHANGE" };
            out += &format!(
                "error_percent: {:+.2}\ncheck: {verdict} (limit 1 %)\nwheel_diameter_suggested: {suggested:.4}\n",
                error * 100.0
            );
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drive(o: &mut Odometer, start: Instant, speed: f64, seconds: f64, hz: f64) -> Instant {
        let steps = (seconds * hz).round() as u32;
        let mut t = start;
        for i in 0..=steps {
            t = start + Duration::from_secs_f64(f64::from(i) / hz);
            o.update(Some(speed), t);
        }
        t
    }

    #[test]
    fn a_steady_speed_counts_its_distance() {
        let mut o = Odometer::default();
        drive(&mut o, Instant::now(), 1.0, 10.0, 20.0);
        assert!((o.distance - 10.0).abs() < 1e-9, "{}", o.distance);
        assert!((o.top - 1.0).abs() < 1e-12);
        assert!((o.mean().expect("timed") - 1.0).abs() < 1e-9);
        assert_eq!(o.gaps, 0);
    }

    #[test]
    fn backwards_counts_as_distance_too() {
        let mut o = Odometer::default();
        drive(&mut o, Instant::now(), -0.5, 4.0, 10.0);
        assert!((o.distance - 2.0).abs() < 1e-9);
    }

    #[test]
    fn a_long_gap_or_an_unknown_speed_is_not_counted_and_is_told() {
        let mut o = Odometer::default();
        let t = drive(&mut o, Instant::now(), 1.0, 2.0, 10.0);
        o.update(Some(1.0), t + Duration::from_secs(2));
        assert!((o.distance - 2.0).abs() < 1e-9);
        assert_eq!(o.gaps, 1);
        o.update(None, t + Duration::from_secs(3));
        assert_eq!(o.gaps, 2);
        o.update(None, t + Duration::from_secs(4)); // still unknown: the same gap
        assert_eq!(o.gaps, 2);
    }

    #[test]
    fn reset_starts_from_zero() {
        let mut o = Odometer::default();
        drive(&mut o, Instant::now(), 1.0, 1.0, 10.0);
        o.reset();
        assert_eq!(o, Odometer::default());
    }

    #[test]
    fn the_entry_takes_one_number() {
        let mut e = Entry::default();
        for c in "1x0.2.5".chars() {
            e.push(c);
        }
        assert_eq!(e.text(), "10.25");
        assert_eq!(e.value(), Some(10.25));
        e.pop();
        assert_eq!(e.value(), Some(10.2));
        e.clear();
        assert_eq!(e.value(), None);
        e.push('.');
        assert_eq!(e.value(), None);
        for c in "123456789".chars() {
            e.push(c);
        }
        assert_eq!(e.text().len(), 7);
    }

    #[test]
    fn wheels_reading_long_ask_for_a_smaller_diameter() {
        let mut o = Odometer::default();
        drive(&mut o, Instant::now(), 1.0, 10.3, 50.0); // 10.3 m by the wheels
        match wheel_check(&o, Some(10.0), Some(0.36)) {
            WheelCheck::Done {
                error,
                suggested,
                pass,
            } => {
                assert!((error - 0.03).abs() < 1e-6);
                assert!((suggested - 0.36 / 1.03).abs() < 1e-6);
                assert!(!pass);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn within_one_percent_passes() {
        let mut o = Odometer::default();
        drive(&mut o, Instant::now(), 1.0, 10.05, 50.0);
        assert!(matches!(
            wheel_check(&o, Some(10.0), Some(0.36)),
            WheelCheck::Done { pass: true, .. }
        ));
    }

    #[test]
    fn the_check_waits_for_what_it_needs() {
        let mut o = Odometer::default();
        let not_yet =
            |o: &Odometer, tape, d| matches!(wheel_check(o, tape, d), WheelCheck::NotYet(_));
        assert!(not_yet(&o, Some(10.0), None));
        assert!(not_yet(&o, None, Some(0.36)));
        assert!(not_yet(&o, Some(1.0), Some(0.36)));
        assert!(not_yet(&o, Some(10.0), Some(0.36))); // not driven yet
        let t = drive(&mut o, Instant::now(), 1.0, 10.0, 20.0);
        o.update(None, t);
        assert!(not_yet(&o, Some(10.0), Some(0.36))); // a gap
    }

    #[test]
    fn the_report_says_what_to_copy() {
        let mut o = Odometer::default();
        drive(&mut o, Instant::now(), 1.0, 10.3, 50.0);
        let r = report(&o, Some(10.0), Some(0.36), "2026-11-15 17:00");
        assert!(r.contains("check: CHANGE"), "{r}");
        assert!(r.contains("wheel_diameter_suggested: 0.3495"), "{r}");
        assert!(r.contains("never writes"));
        let r = report(&Odometer::default(), None, Some(0.36), "now");
        assert!(r.contains("check: not yet (type the tape's metres)"), "{r}");
    }
}

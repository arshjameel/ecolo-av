//! The steering actuator's limits, from its marked ends and centre: `aav_legacy`'s
//! `steering_limits.py`, ported rule for rule.
//!
//! Counts are the steering actuator's position: its (Jrk's) scaled feedback, 0..4095,
//! in the units of its targets.
//!
//! | Step | Rule |
//! |---|---|
//! | 1 | travel left = \|left − centre\|, travel right = \|right − centre\| |
//! | 2 | errors: the ends under [`MIN_SPAN`] apart; the centre not between them; a count outside 0..4095; an angle not above 0 |
//! | 3 | half = floor(smaller travel × (1 − margin)), margin [`MARGIN`] |
//! | 4 | lo = centre − half, hi = centre + half |
//! | 5 | sign = +1 if left > centre (steering left raises the target), else −1 |
//! | 6 | with both wheel angles: counts a radian = sign × the mean of the two sides' gains; `max_steering_angle` = half ÷ that |
//! | 7 | warnings: the centre off the middle, a lopsided travel, an end at 0 or 4095, gains that differ |
//!
//! Both sides test against one file of vectors,
//! `ros/can_bridge/aav_legacy/test/steering_limits_vectors.json`: change the two together.
//! Writing the result into the firmware or vehicle.yaml is a person's job, in a pull
//! request: the steering actuator's own (Jrk) limits are the hardware lead's.

use std::fmt;

use thiserror::Error;

/// The highest count.
pub const COUNTS_MAX: u16 = 4095;
/// Counts: ends closer than this, and the feedback doesn't change.
pub const MIN_SPAN: u16 = 100;
/// Off the smaller side's travel.
pub const MARGIN: f64 = 0.05;
/// Of the span: a centre further than this from the middle is flagged.
pub const CENTRE_OFF: f64 = 0.10;
/// The smaller travel over the larger, below which it is flagged.
pub const LOPSIDED: f64 = 0.8;
/// Of their mean: two sides' gains further apart are flagged.
pub const GAINS_DIFFER: f64 = 0.15;

/// The three marks, in counts, and the wheel angle at each end (degrees, if measured).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Marks {
    pub left: u16,
    pub centre: u16,
    pub right: u16,
    pub left_deg: Option<f64>,
    pub right_deg: Option<f64>,
}

/// Which mark.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mark {
    Left,
    Centre,
    Right,
}

impl fmt::Display for Mark {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Mark::Left => "left",
            Mark::Centre => "centre",
            Mark::Right => "right",
        })
    }
}

/// Why the marks can't give limits.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum MeasurementError {
    #[error("{0} {1}: counts are 0 to {COUNTS_MAX}")]
    OutOfRange(Mark, u16),
    #[error(
        "the ends are {0} counts apart (less than {MIN_SPAN}): the feedback doesn't change. \
         Is the Jrk's feedback mode analog, and its sensor wired?"
    )]
    NoSpan(u16),
    #[error("the centre ({centre}) is not between the ends ({left}, {right}): mark it again")]
    CentreNotBetween { left: u16, centre: u16, right: u16 },
    #[error("the wheel angles are positive degrees, each side")]
    Angles,
}

/// A result that still gives limits, but a person should look at.
#[derive(Debug, Clone, PartialEq)]
pub enum Warning {
    /// The centre is this far (counts) from the middle of the travel.
    CentreOff { centre: u16, off: f64, middle: f64 },
    /// One side travels much less than the other.
    Lopsided { left: u16, right: u16 },
    /// An end reads 0 or 4095.
    Clipped(Mark, u16),
    /// The two sides' counts a radian.
    GainsDiffer { left: f64, right: f64 },
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Warning::CentreOff {
                centre,
                off,
                middle,
            } => write!(
                f,
                "the centre ({centre}) is {off:.0} counts from the middle of the travel \
                 ({middle:.0}): check the linkage, and mark the centre again"
            ),
            Warning::Lopsided { left, right } => write!(
                f,
                "the travel is lopsided: {left} counts left, {right} right; the limits use \
                 the smaller side"
            ),
            Warning::Clipped(mark, value) => write!(
                f,
                "the {mark} end reads {value}: the feedback may be clipped (by the Jrk \
                 feedback scaling); the true end may be further"
            ),
            Warning::GainsDiffer { left, right } => write!(
                f,
                "the gains of the two sides differ: {left:.0} and {right:.0} counts a radian; \
                 measure the angles again, or the linkage is not linear"
            ),
        }
    }
}

/// What the marks give.
#[derive(Debug, Clone, PartialEq)]
pub struct Limits {
    pub centre: u16,
    /// Counts: the limits, the same distance each side of the centre.
    pub lo: u16,
    pub hi: u16,
    /// Counts each side of the centre.
    pub half: u16,
    /// +1: steering left raises the target.
    pub sign: i8,
    /// Signed, as calibration.rs's `counts_per_rad`; with both wheel angles only.
    pub counts_per_rad: Option<f64>,
    /// rad at lo or hi; with both wheel angles only.
    pub max_steering_angle: Option<f64>,
    pub warnings: Vec<Warning>,
}

impl Limits {
    /// The limit on the left (full left lock's side), counts.
    pub fn left_limit(&self) -> u16 {
        if self.sign > 0 {
            self.hi
        } else {
            self.lo
        }
    }

    /// The limit on the right, counts.
    pub fn right_limit(&self) -> u16 {
        if self.sign > 0 {
            self.lo
        } else {
            self.hi
        }
    }
}

/// The limits for the marks, with [`MARGIN`].
pub fn compute(m: &Marks) -> Result<Limits, MeasurementError> {
    compute_with_margin(m, MARGIN)
}

/// The limits for the marks: the module's rules.
pub fn compute_with_margin(m: &Marks, margin: f64) -> Result<Limits, MeasurementError> {
    for (mark, value) in [
        (Mark::Left, m.left),
        (Mark::Centre, m.centre),
        (Mark::Right, m.right),
    ] {
        if value > COUNTS_MAX {
            return Err(MeasurementError::OutOfRange(mark, value));
        }
    }
    let span = m.left.abs_diff(m.right);
    if span < MIN_SPAN {
        return Err(MeasurementError::NoSpan(span));
    }
    let (left, centre, right) = (i32::from(m.left), i32::from(m.centre), i32::from(m.right));
    if (left - centre) * (right - centre) >= 0 {
        return Err(MeasurementError::CentreNotBetween {
            left: m.left,
            centre: m.centre,
            right: m.right,
        });
    }
    let travel_left = m.left.abs_diff(m.centre);
    let travel_right = m.right.abs_diff(m.centre);
    let smaller = travel_left.min(travel_right);
    let larger = travel_left.max(travel_right);
    // floor(), as Python's: the travel is under 4096, so it fits.
    let half = (f64::from(smaller) * (1.0 - margin)).floor() as u16;
    let sign: i8 = if m.left > m.centre { 1 } else { -1 };
    let mut warnings = Vec::new();

    let middle = (f64::from(m.left) + f64::from(m.right)) / 2.0;
    let off = (f64::from(m.centre) - middle).abs();
    if off > CENTRE_OFF * f64::from(span) {
        warnings.push(Warning::CentreOff {
            centre: m.centre,
            off,
            middle,
        });
    }
    if f64::from(smaller) / f64::from(larger) < LOPSIDED {
        warnings.push(Warning::Lopsided {
            left: travel_left,
            right: travel_right,
        });
    }
    for (mark, value) in [(Mark::Left, m.left), (Mark::Right, m.right)] {
        if value == 0 || value == COUNTS_MAX {
            warnings.push(Warning::Clipped(mark, value));
        }
    }

    let (mut counts_per_rad, mut max_steering_angle) = (None, None);
    if let (Some(left_deg), Some(right_deg)) = (m.left_deg, m.right_deg) {
        if !(left_deg > 0.0 && right_deg > 0.0) {
            return Err(MeasurementError::Angles);
        }
        let gain_left = f64::from(travel_left) / left_deg.to_radians();
        let gain_right = f64::from(travel_right) / right_deg.to_radians();
        let mean = (gain_left + gain_right) / 2.0;
        counts_per_rad = Some(f64::from(sign) * mean);
        max_steering_angle = Some(f64::from(half) / mean);
        if (gain_left - gain_right).abs() > GAINS_DIFFER * mean {
            warnings.push(Warning::GainsDiffer {
                left: gain_left,
                right: gain_right,
            });
        }
    }
    Ok(Limits {
        centre: m.centre,
        lo: m.centre - half,
        hi: m.centre + half,
        half,
        sign,
        counts_per_rad,
        max_steering_angle,
        warnings,
    })
}

/// spine-board calibration.rs's `Steering` for the result: a person pastes it.
pub fn firmware_steering(r: &Limits) -> String {
    let cpr = r.counts_per_rad.map_or_else(
        || "TODO: measure the wheel angles".to_owned(),
        |c| format!("{c:.1}"),
    );
    format!(
        "steering: Steering {{\n    centre: {},\n    counts_per_rad: {cpr},\n    min: {},\n    \
         max: {},\n}},",
        r.centre, r.lo, r.hi
    )
}

/// vehicle.yaml's `max_steering_angle` line for the result.
pub fn vehicle_yaml(r: &Limits) -> String {
    match r.max_steering_angle {
        None => "# max_steering_angle: measure the wheel angles at each end first".to_owned(),
        Some(a) => format!("max_steering_angle: {a:.3}  # rad, measured (steering_limits)"),
    }
}

/// The result for people, in short lines: `steering_limits.py`'s `report`.
pub fn report(r: &Limits) -> String {
    let mut lines = vec![
        format!(
            "centre {}, limits {} to {} ({} counts each side)",
            r.centre, r.lo, r.hi, r.half
        ),
        format!(
            "steering left {} the target",
            if r.sign > 0 { "raises" } else { "lowers" }
        ),
    ];
    if let (Some(a), Some(c)) = (r.max_steering_angle, r.counts_per_rad) {
        lines.push(format!(
            "{:.0} counts a radian; at the limits {a:.3} rad ({:.1} deg)",
            c.abs(),
            a.to_degrees()
        ));
    }
    lines.extend(r.warnings.iter().map(|w| format!("WARNING: {w}")));
    lines.extend([
        String::new(),
        "For firmware/spine/crates/spine-board/src/calibration.rs:".to_owned(),
        firmware_steering(r),
        String::new(),
        "For vehicle.yaml:".to_owned(),
        vehicle_yaml(r),
        String::new(),
        "The steering actuator's own (Jrk) limits are the hardware lead's to set.".to_owned(),
    ]);
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    fn marks(left: u16, centre: u16, right: u16) -> Marks {
        Marks {
            left,
            centre,
            right,
            left_deg: None,
            right_deg: None,
        }
    }

    #[test]
    fn symmetric_limits_from_the_smaller_side_less_5_percent() {
        let r = compute(&marks(3500, 1950, 500)).expect("limits");
        assert_eq!((r.half, r.lo, r.hi, r.sign), (1377, 573, 3327, 1));
        assert_eq!((r.left_limit(), r.right_limit()), (3327, 573));
        assert!(r.warnings.is_empty());
        let text = report(&r);
        assert!(text.contains("limits 573 to 3327") && text.contains("min: 573"));
        assert!(text.contains("TODO: measure the wheel angles"));
    }

    #[test]
    fn steering_left_lowering_the_target() {
        let m = Marks {
            left_deg: Some(28.0),
            right_deg: Some(28.0),
            ..marks(600, 2000, 3400)
        };
        let r = compute(&m).expect("limits");
        assert!(r.sign < 0 && r.counts_per_rad.is_some_and(|c| c < 0.0));
        assert_eq!((r.left_limit(), r.right_limit()), (670, 3330));
        assert!(vehicle_yaml(&r).starts_with("max_steering_angle: 0.4"));
    }

    // --- the vectors shared with steering_limits.py ---

    #[derive(Deserialize)]
    struct Vectors {
        margin: f64,
        cases: Vec<Case>,
    }

    #[derive(Deserialize)]
    struct Case {
        name: String,
        left: u16,
        centre: u16,
        right: u16,
        left_deg: Option<f64>,
        right_deg: Option<f64>,
        expect: Expect,
    }

    #[derive(Deserialize)]
    struct Expect {
        error: Option<String>,
        lo: Option<u16>,
        hi: Option<u16>,
        half: Option<u16>,
        sign: Option<i8>,
        counts_per_rad: Option<f64>,
        max_steering_angle: Option<f64>,
        #[serde(default)]
        warnings: Vec<String>,
    }

    fn error_kind(e: &MeasurementError) -> &'static str {
        match e {
            MeasurementError::OutOfRange(..) => "out_of_range",
            MeasurementError::NoSpan(_) => "no_span",
            MeasurementError::CentreNotBetween { .. } => "centre_not_between",
            MeasurementError::Angles => "angles",
        }
    }

    fn warning_kind(w: &Warning) -> &'static str {
        match w {
            Warning::CentreOff { .. } => "centre_off",
            Warning::Lopsided { .. } => "lopsided",
            Warning::Clipped(Mark::Left, _) => "clipped_left",
            Warning::Clipped(_, _) => "clipped_right",
            Warning::GainsDiffer { .. } => "gains_differ",
        }
    }

    /// The same vectors as aav_legacy's test_steering_limits.py: the two give the same.
    #[test]
    fn the_shared_vectors() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../can_bridge/aav_legacy/test/steering_limits_vectors.json"
        );
        let text = std::fs::read_to_string(path).expect("the shared vectors file");
        let v: Vectors = serde_yaml_ng::from_str(&text).expect("JSON is YAML");
        assert_eq!(v.margin, MARGIN);
        assert!(v.cases.len() >= 8);
        for c in &v.cases {
            let m = Marks {
                left: c.left,
                centre: c.centre,
                right: c.right,
                left_deg: c.left_deg,
                right_deg: c.right_deg,
            };
            let got = compute(&m);
            let e = &c.expect;
            match (&e.error, got) {
                (Some(kind), Err(err)) => assert_eq!(error_kind(&err), kind, "{}", c.name),
                (Some(kind), Ok(_)) => panic!("{}: expected {kind}", c.name),
                (None, Err(err)) => panic!("{}: {err}", c.name),
                (None, Ok(r)) => {
                    assert_eq!(Some(r.lo), e.lo, "{}", c.name);
                    assert_eq!(Some(r.hi), e.hi, "{}", c.name);
                    assert_eq!(Some(r.half), e.half, "{}", c.name);
                    assert_eq!(Some(r.sign), e.sign, "{}", c.name);
                    let close = |a: Option<f64>, b: Option<f64>| match (a, b) {
                        (Some(a), Some(b)) => (a - b).abs() < 1e-6 * b.abs().max(1.0),
                        (None, None) => true,
                        _ => false,
                    };
                    assert!(close(r.counts_per_rad, e.counts_per_rad), "{}", c.name);
                    assert!(
                        close(r.max_steering_angle, e.max_steering_angle),
                        "{}",
                        c.name
                    );
                    let kinds: Vec<&str> = r.warnings.iter().map(warning_kind).collect();
                    assert_eq!(kinds, e.warnings, "{}", c.name);
                }
            }
        }
    }
}

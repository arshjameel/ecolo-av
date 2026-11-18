//! The `drive_to_point` command's words and sums, without ROS (`point_cli.py`): its
//! command line, where B is, the feedback's phase words and the real car's checklist.

use std::fmt;

/// The checklist before the real car moves (aav_actuator_tests safety.py's LOT_CHECKLIST).
pub const LOT_CHECKLIST: [&str; 5] = [
    "Stage 3 passed on stands, and the ESP32 timeout fix (test 3.9) is in",
    "A safety driver in the seat, holding the joystick: its deadman outranks this test",
    "Someone else holds the e-stop",
    "An empty lot: nobody in the way, and the space this test needs (shown above) is clear",
    "Walking pace first: start with the default speed cap",
];

/// One line of help, as argparse writes it.
pub const USAGE: &str = "usage: drive-to-point [-h] [--lat LAT] [--lon LON] [--x X] [--y Y] \
                         [--ahead AHEAD] [--left LEFT] [--speed SPEED] [--sim]";

/// Where B is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Where {
    /// A GPS point: degrees north, east.
    LatLon(f64, f64),
    /// A map point: m east, m north.
    Map(f64, f64),
    /// m ahead of the car's nose, m to its left.
    Ahead(f64, f64),
}

/// What the command line asks for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Args {
    /// Where B is.
    pub target: Where,
    /// Cruise speed, m/s (0: the driver's default).
    pub speed: f64,
    /// Ravensim: this computer, the simulator's clock.
    pub sim: bool,
}

/// A command line argparse would refuse (exit 2).
#[derive(Debug, Clone, PartialEq)]
pub enum ArgsError {
    /// A flag it doesn't know.
    Unknown(String),
    /// A flag with no value after it.
    Missing(String),
    /// A value that isn't a number (the flag, the text).
    NotANumber(String, String),
    /// None or several of the three kinds of point.
    OneKind,
    /// --lat without --lon, or the other way.
    LatLon,
    /// --x without --y, or the other way.
    Xy,
}

impl fmt::Display for ArgsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ArgsError::Unknown(a) => write!(f, "unrecognized arguments: {a}"),
            ArgsError::Missing(a) => write!(f, "argument {a}: expected one argument"),
            ArgsError::NotANumber(a, t) => write!(f, "argument {a}: invalid float value: '{t}'"),
            ArgsError::OneKind => {
                f.write_str("give one of: --lat and --lon, --x and --y, or --ahead [--left]")
            }
            ArgsError::LatLon => f.write_str("--lat needs --lon, and --lon needs --lat"),
            ArgsError::Xy => f.write_str("--x needs --y, and --y needs --x"),
        }
    }
}

impl std::error::Error for ArgsError {}

/// The arguments (without the program's name).
///
/// # Errors
///
/// [`ArgsError`] for a command line argparse would refuse.
pub fn parse(argv: &[String]) -> Result<Args, ArgsError> {
    let mut v: [Option<f64>; 7] = [None; 7]; // lat lon x y ahead left speed
    let mut sim = false;
    let mut it = argv.iter();
    while let Some(arg) = it.next() {
        if arg == "--sim" {
            sim = true;
            continue;
        }
        let (flag, inline) = match arg.split_once('=') {
            Some((f, t)) => (f, Some(t.to_string())),
            None => (arg.as_str(), None),
        };
        let slot = [
            "--lat", "--lon", "--x", "--y", "--ahead", "--left", "--speed",
        ]
        .iter()
        .position(|f| *f == flag)
        .ok_or_else(|| ArgsError::Unknown(arg.clone()))?;
        let text = inline
            .or_else(|| it.next().cloned())
            .ok_or_else(|| ArgsError::Missing(flag.to_string()))?;
        let number = text
            .parse()
            .map_err(|_| ArgsError::NotANumber(flag.to_string(), text.clone()))?;
        v[slot] = Some(number);
    }
    let [lat, lon, x, y, ahead, left, speed] = v;
    let kinds = [
        lat.is_some() || lon.is_some(),
        x.is_some() || y.is_some(),
        ahead.is_some(),
    ];
    if kinds.iter().filter(|k| **k).count() != 1 {
        return Err(ArgsError::OneKind);
    }
    let target = match (lat, lon, x, y, ahead) {
        (Some(la), Some(lo), ..) => Where::LatLon(la, lo),
        (Some(_), None, ..) | (None, Some(_), ..) => return Err(ArgsError::LatLon),
        (_, _, Some(x), Some(y), _) => Where::Map(x, y),
        (_, _, Some(_), None, _) | (_, _, None, Some(_), _) => return Err(ArgsError::Xy),
        (.., Some(a)) => Where::Ahead(a, left.unwrap_or(0.0)),
        _ => return Err(ArgsError::OneKind),
    };
    Ok(Args {
        target,
        speed: speed.unwrap_or(0.0),
        sim,
    })
}

/// B on the map for `--ahead m --left m` from the car at (x, y, yaw).
#[must_use]
pub fn ahead_of(pose: (f64, f64, f64), ahead: f64, left: f64) -> (f64, f64) {
    let (x, y, yaw) = pose;
    (
        x + ahead * yaw.cos() - left * yaw.sin(),
        y + ahead * yaw.sin() + left * yaw.cos(),
    )
}

/// The feedback's phase, in words (its number if unknown).
#[must_use]
pub fn phase_word(phase: u8) -> String {
    match phase {
        0 => "turning".into(),
        1 => "speeding up".into(),
        2 => "cruising".into(),
        3 => "slowing".into(),
        4 => "stopping".into(),
        n => n.to_string(),
    }
}

/// One feedback line, as point_cli.py prints it.
#[must_use]
pub fn feedback_line(phase: u8, left: f64, cross: f64, measured: f64, asked: f64) -> String {
    format!(
        "  {:<11} {left:6.1} m left, {cross:+.2} m off the line, {measured:.2} m/s \
         (asked {asked:.2})",
        phase_word(phase)
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(text: &str) -> Result<Args, ArgsError> {
        parse(
            &text
                .split_whitespace()
                .map(String::from)
                .collect::<Vec<_>>(),
        )
    }

    #[test]
    fn one_kind_of_point() {
        let a = args("--ahead 20 --sim").unwrap();
        assert!(a.sim && a.target == Where::Ahead(20.0, 0.0));
        assert_eq!(args("--x 1 --y 2 --speed=0.5").unwrap().speed, 0.5);
        assert_eq!(args("--x 1"), Err(ArgsError::Xy));
        assert_eq!(args("--lat 45"), Err(ArgsError::LatLon));
        assert_eq!(args("--lat 45 --ahead 3"), Err(ArgsError::OneKind));
        assert_eq!(args(""), Err(ArgsError::OneKind));
        assert!(matches!(args("--ahead x"), Err(ArgsError::NotANumber(..))));
        assert!(matches!(args("--fast"), Err(ArgsError::Unknown(_))));
        assert!(matches!(args("--ahead"), Err(ArgsError::Missing(_))));
    }

    #[test]
    fn ahead_is_in_the_cars_frame() {
        let (x, y) = ahead_of((1.0, 1.0, std::f64::consts::FRAC_PI_2), 10.0, 2.0);
        assert!((x - -1.0).abs() < 1e-9 && (y - 11.0).abs() < 1e-9);
        assert_eq!(
            feedback_line(3, 12.345, -0.5, 1.0, 0.8),
            "  slowing       12.3 m left, -0.50 m off the line, 1.00 m/s (asked 0.80)"
        );
    }
}

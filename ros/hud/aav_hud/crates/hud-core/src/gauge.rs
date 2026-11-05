//! The arcade speed gauge (the `initial_d` theme): its scale in km/h, where each tick
//! and number goes round the dial, the redline's red blocks, the needle's smooth swing,
//! and what the two small boxes say.
//!
//! The car is slow (ego's cap is 1 m/s today, maybe 3-5 m/s later), so the scale comes
//! from the configured top speed, not from a car's 0-180 km/h: the redline starts at
//! `max_speed` and fills about the top fifth of the dial.
//!
//! ```text
//!            90 deg sweep left open at the bottom
//!        4      5         numbers inside the ticks
//!     3 /  ''''  \ ###    ### = the redline, red blocks
//!    2 |    \     |
//!    1  \    o   /        o = the needle's hub
//!      0          [ 3.6] [D]
//! ```
//!
//! Angles are the screen's: 0 points right, and they grow clockwise (y points down),
//! as raylib draws rings. 0 km/h sits at 135 degrees (lower left), the full scale at
//! 405 degrees (lower right).

use std::time::Duration;

use thiserror::Error;

use crate::{
    units::MetresPerSecond,
    vehicle::{Gear, VehicleStatus},
};

/// Where 0 km/h sits: lower left, in degrees on the screen (clockwise from right).
pub const START_DEG: f32 = 135.0;
/// How far round the dial goes, clockwise: three quarters, the bottom left open.
pub const SWEEP_DEG: f32 = 270.0;
/// The redline starts at `max_speed`, this fraction of the way up the scale or less.
pub const REDLINE_AT: f32 = 0.8;
/// How much of each redline block is gap, at its top end: the blocks look cut.
pub const BLOCK_GAP: f32 = 0.22;

/// Full scales the dial may have, in km/h: the first at or above `max_speed / 0.8`.
const NICE_FULL_SCALES: [u32; 14] = [2, 5, 10, 20, 25, 30, 40, 50, 60, 80, 100, 120, 160, 200];
/// Steps between numbered ticks, in km/h: the smallest that gives 10 numbers or fewer.
const MAJOR_STEPS: [u32; 7] = [1, 2, 5, 10, 20, 50, 100];

/// A top speed the gauge can't scale to.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
#[error("max_speed must be a positive number of m/s, not {0}")]
pub struct BadMaxSpeed(pub f32);

/// The dial's numbers: its full scale, where the redline starts, and the ticks' steps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scale {
    full: u32,
    redline: f32,
    major: u32,
    minors: u32,
}

/// One tick round the dial.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tick {
    /// Where it is, in km/h.
    pub kmh: f32,
    /// A long tick with a number beside it.
    pub major: bool,
}

impl Tick {
    /// The number beside a major tick: "0", "1", "20".
    pub fn label(&self) -> Option<String> {
        self.major.then(|| format!("{:.0}", self.kmh))
    }
}

impl Scale {
    /// The scale for a car whose top speed is `max_speed`.
    pub fn for_max_speed(max_speed: MetresPerSecond) -> Result<Self, BadMaxSpeed> {
        let max = max_speed.0;
        if !(max.is_finite() && max > 0.0) {
            return Err(BadMaxSpeed(max));
        }
        let redline = max_speed.to_km_per_hour();
        let wanted = redline / REDLINE_AT;
        let full = NICE_FULL_SCALES
            .into_iter()
            .find(|&f| f as f32 >= wanted)
            // Faster than any car this team builds: round up to 50s.
            .unwrap_or_else(|| (wanted / 50.0).ceil() as u32 * 50);
        let major = MAJOR_STEPS
            .into_iter()
            .find(|&s| full % s == 0 && full / s <= 10)
            .unwrap_or(full);
        // Quarters for a 2 or 20 step (0.5, 5 km/h), fifths otherwise.
        let minors = if major.to_string().starts_with('2') {
            4
        } else {
            5
        };
        Ok(Self {
            full,
            redline,
            major,
            minors,
        })
    }

    /// The top of the dial, in km/h.
    pub fn full_scale(&self) -> f32 {
        self.full as f32
    }

    /// Where the red blocks start, in km/h: the configured top speed.
    pub fn redline(&self) -> f32 {
        self.redline
    }

    /// The step between numbered ticks, in km/h.
    pub fn major_step(&self) -> f32 {
        self.major as f32
    }

    /// The step between any two ticks, in km/h.
    pub fn minor_step(&self) -> f32 {
        self.major as f32 / self.minors as f32
    }

    /// Where `kmh` points, in radians on the screen (clockwise from right). Below 0 or
    /// above the full scale, the needle stops at the end.
    pub fn angle(&self, kmh: f32) -> f32 {
        let k = if kmh.is_nan() {
            0.0
        } else {
            (kmh / self.full_scale()).clamp(0.0, 1.0)
        };
        (START_DEG + SWEEP_DEG * k).to_radians()
    }

    /// Every tick from 0 to the full scale, in order.
    pub fn ticks(&self) -> impl Iterator<Item = Tick> + '_ {
        let count = self.full / self.major * self.minors;
        (0..=count).map(move |i| Tick {
            kmh: i as f32 * self.minor_step(),
            major: i % self.minors == 0,
        })
    }

    /// The redline's blocks, as (from, to) in km/h: one per tick step from the redline
    /// to the full scale, each with its gap cut off its top end. A redline at or above
    /// the full scale has none.
    pub fn redline_blocks(&self) -> impl Iterator<Item = (f32, f32)> + '_ {
        let step = self.minor_step();
        let first = (self.redline / step).floor() as u32;
        let last = self.full / self.major * self.minors;
        (first..last).filter_map(move |i| {
            let from = (i as f32 * step).max(self.redline);
            let to = (i + 1) as f32 * step;
            let end = to - step * BLOCK_GAP;
            (end > from).then_some((from, end))
        })
    }
}

/// A point `radius` from `centre` at `angle` (radians, clockwise from right on a screen).
pub fn point(centre: (f32, f32), radius: f32, angle: f32) -> (f32, f32) {
    (
        centre.0 + radius * angle.cos(),
        centre.1 + radius * angle.sin(),
    )
}

/// How quickly the needle catches up: about 63 % of the way in this long, and no
/// overshoot. Quick enough to follow the car, slow enough not to jitter.
pub const NEEDLE_LAG: Duration = Duration::from_millis(120);

/// The needle, swinging smoothly to the speed instead of jumping.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Needle {
    kmh: f32,
}

impl Needle {
    /// Where the needle points now, in km/h.
    pub fn kmh(&self) -> f32 {
        self.kmh
    }

    /// Moves the needle `dt` towards `target` km/h and returns where it points. It never
    /// passes the target. A target that isn't a number counts as 0 (no speed known).
    pub fn follow(&mut self, target: f32, dt: Duration) -> f32 {
        let target = if target.is_finite() { target } else { 0.0 };
        let k = 1.0 - (-dt.as_secs_f32() / NEEDLE_LAG.as_secs_f32()).exp();
        self.kmh += (target - self.kmh) * k;
        if !self.kmh.is_finite() {
            self.kmh = target;
        }
        self.kmh
    }
}

/// The digital box's text: km/h with one decimal, "--.-" with no speed. A speed in
/// reverse shows as its size (the gear box says R). Over 99.9 shows 99.9: the box
/// has three digits.
pub fn readout(speed: Option<MetresPerSecond>) -> String {
    match speed.map(MetresPerSecond::to_km_per_hour) {
        Some(kmh) if kmh.is_finite() => format!("{:.1}", kmh.abs().min(99.9)),
        _ => "--.-".to_owned(),
    }
}

/// What the small box beside the speed says (where the arcade gauge says "MT 1").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GearBox {
    /// The e-stop is on: that comes first.
    Estop,
    /// The way the car goes: D, R or P.
    Gear(Gear),
    /// No status yet.
    Unknown,
}

impl GearBox {
    /// The box for the newest status, if there is one.
    pub fn of(status: Option<&VehicleStatus>) -> Self {
        match status {
            None => Self::Unknown,
            Some(s) if s.estop_active => Self::Estop,
            Some(s) => Self::Gear(s.gear()),
        }
    }

    /// Its words: "E-STOP", "D", "R", "P" or "--".
    pub fn label(self) -> &'static str {
        match self {
            Self::Estop => "E-STOP",
            Self::Gear(g) => g.letter(),
            Self::Unknown => "--",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        units::{Fraction, Radians},
        vehicle::{Faults, Mode},
    };

    fn scale(max: f32) -> Scale {
        Scale::for_max_speed(MetresPerSecond(max)).expect("a speed")
    }

    #[test]
    fn the_scale_fits_the_car() {
        // Today's cap, 1 m/s = 3.6 km/h: a 0-5 dial, numbered every 1.
        let s = scale(1.0);
        assert_eq!(s.full_scale(), 5.0);
        assert_eq!(s.major_step(), 1.0);
        assert!((s.redline() - 3.6).abs() < 1e-4);
        // Later, 5 m/s = 18 km/h: 0-25 every 5.
        let s = scale(5.0);
        assert_eq!((s.full_scale(), s.major_step()), (25.0, 5.0));
        // 3 m/s = 10.8 km/h: 0-20 every 2, in quarters.
        let s = scale(3.0);
        assert_eq!((s.full_scale(), s.major_step()), (20.0, 2.0));
        assert_eq!(s.minor_step(), 0.5);
        // A fast car still gets a round scale.
        assert_eq!(scale(100.0).full_scale(), 450.0);
    }

    #[test]
    fn the_redline_is_the_top_of_the_dial() {
        for max in [0.3, 1.0, 2.0, 3.0, 5.0, 8.0, 15.0] {
            let s = scale(max);
            let k = s.redline() / s.full_scale();
            assert!(k <= REDLINE_AT + 1e-6, "{max} m/s: red from {k}");
            assert!(k >= 0.4, "{max} m/s: too much red, from {k}");
            let labels = s.ticks().filter(|t| t.major).count();
            assert!((3..=11).contains(&labels), "{max} m/s: {labels} numbers");
        }
    }

    #[test]
    fn a_bad_top_speed_is_refused() {
        for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(Scale::for_max_speed(MetresPerSecond(bad)).is_err(), "{bad}");
        }
    }

    #[test]
    fn ticks_run_from_zero_to_full_with_numbers() {
        let s = scale(1.0);
        let ticks: Vec<Tick> = s.ticks().collect();
        assert_eq!(ticks.len(), 26); // 0 to 5 in fifths
        assert_eq!(ticks[0].label().as_deref(), Some("0"));
        assert_eq!(ticks[1].label(), None);
        assert_eq!(ticks.last().and_then(Tick::label).as_deref(), Some("5"));
        assert!(ticks.windows(2).all(|w| w[1].kmh > w[0].kmh));
    }

    #[test]
    fn the_dial_goes_round_clockwise_from_lower_left() {
        let s = scale(1.0);
        let deg = |kmh: f32| s.angle(kmh).to_degrees();
        assert!((deg(0.0) - 135.0).abs() < 1e-3);
        assert!((deg(2.5) - 270.0).abs() < 1e-3); // half way: straight up
        assert!((deg(5.0) - 405.0).abs() < 1e-3);
        // The needle stops at the ends.
        assert_eq!(deg(-3.0), deg(0.0));
        assert_eq!(deg(99.0), deg(5.0));
        assert_eq!(deg(f32::NAN), deg(0.0));
        // Straight up is up the screen: y smaller.
        let (x, y) = point((100.0, 100.0), 10.0, s.angle(2.5));
        assert!((x - 100.0).abs() < 1e-3 && (y - 90.0).abs() < 1e-3);
        // 0 is lower left: x smaller, y bigger.
        let (x, y) = point((100.0, 100.0), 10.0, s.angle(0.0));
        assert!(x < 100.0 && y > 100.0);
    }

    #[test]
    fn redline_blocks_fill_from_the_cap_to_the_top_with_gaps() {
        let s = scale(1.0); // red from 3.6 to 5, in 0.2 steps
        let blocks: Vec<(f32, f32)> = s.redline_blocks().collect();
        assert_eq!(blocks.len(), 7);
        assert!((blocks[0].0 - 3.6).abs() < 1e-4);
        let (_, last) = blocks[blocks.len() - 1];
        assert!(last < 5.0 && last > 4.8);
        for (from, to) in &blocks {
            assert!(to > from);
        }
        // A gap between each block and the next.
        assert!(blocks.windows(2).all(|w| w[1].0 > w[0].1));
    }

    #[test]
    fn the_needle_swings_smoothly_and_never_overshoots() {
        let mut n = Needle::default();
        let frame = Duration::from_millis(33);
        let mut last = 0.0;
        for _ in 0..30 {
            let at = n.follow(4.0, frame);
            assert!(at > last && at <= 4.0, "{at}");
            last = at;
        }
        assert!((n.kmh() - 4.0).abs() < 0.01, "a second later: {}", n.kmh());
        // One frame moves it only part of the way.
        let mut n = Needle::default();
        let first = n.follow(4.0, frame);
        assert!(first > 0.5 && first < 3.5, "{first}");
        // No speed: back home to 0.
        assert!(n.follow(f32::NAN, Duration::from_secs(5)) < 0.01);
    }

    #[test]
    fn the_readout_has_one_decimal() {
        assert_eq!(readout(Some(MetresPerSecond(1.0))), "3.6");
        assert_eq!(readout(Some(MetresPerSecond(-0.5))), "1.8");
        assert_eq!(readout(Some(MetresPerSecond(0.0))), "0.0");
        assert_eq!(readout(Some(MetresPerSecond(50.0))), "99.9");
        assert_eq!(readout(None), "--.-");
        assert_eq!(readout(Some(MetresPerSecond(f32::NAN))), "--.-");
    }

    #[test]
    fn the_gear_box_puts_the_estop_first() {
        let mut v = VehicleStatus {
            mode: Mode::Computer,
            armed: true,
            estop_active: false,
            speed: MetresPerSecond(1.0),
            steering_angle: Radians(0.0),
            throttle: Fraction::new(0.0),
            brake: Fraction::new(0.0),
            faults: Faults(0),
        };
        assert_eq!(GearBox::of(Some(&v)).label(), "D");
        v.speed = MetresPerSecond(-0.5);
        assert_eq!(GearBox::of(Some(&v)).label(), "R");
        v.estop_active = true;
        assert_eq!(GearBox::of(Some(&v)), GearBox::Estop);
        assert_eq!(GearBox::of(None).label(), "--");
    }
}

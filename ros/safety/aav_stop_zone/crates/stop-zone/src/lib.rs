//! The stop zone, a safety lock: Nav2's `collision_monitor` stop decision, ported to Rust
//! with no ROS (TODO 182: "ours in Rust with its own tests first"). Nav2's runs on the car
//! until this one's node has scored the same.
//!
//! ```text
//!  the command (x, turn) ─▶ the box for its direction (the first speed band that holds it)
//!  each sensor's newest reading ─▶ too old? ─▶ STOP (a silent sensor)
//!                                ─▶ its points, filtered (range, height), in base_link
//!  points in the box, all sensors together ≥ min_points ─▶ STOP: the command becomes zero
//!  after a stop, zeros keep going out stop_pub_timeout s, then nothing (twist_mux's slot
//!  times out)
//! ```
//!
//! Ported from Nav2's nav2_collision_monitor (Apache-2.0, Samsung R&D Institute Russia):
//! `CollisionMonitor::process` and `publishVelocity`, `VelocityPolygon::isInRange` (not
//! holonomic), `Polygon::isPointInside` (Shimrat 1962, ray crossing), `PointCloud::getData`
//! and `Source::sourceValid`. Only the `stop` action: our config uses no other.
//!
//! Units: m, rad, s; `base_link` (x forward, y left). A command's `x` is m/s, `turn` rad/s.
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use std::fmt;
use std::ops::RangeInclusive;

/// A point on the ground, `base_link` (m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// m, + forward.
    pub x: f64,
    /// m, + left.
    pub y: f64,
}

impl Point {
    /// The point (`x`, `y`).
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A drive command: forward speed (m/s, negative reversing) and turn rate (rad/s).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Command {
    /// m/s, negative reversing.
    pub x: f64,
    /// rad/s, + left.
    pub turn: f64,
}

impl Command {
    /// Standing still.
    pub const STOP: Command = Command { x: 0.0, turn: 0.0 };

    /// Both zero.
    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.x == 0.0 && self.turn == 0.0
    }
}

/// A closed outline (its last corner joins its first).
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon(pub Vec<Point>);

impl Polygon {
    /// Is `p` inside? Ray crossing (Shimrat 1962), as Nav2's `isPointInside`.
    #[must_use]
    pub fn contains(&self, p: Point) -> bool {
        let corners = &self.0;
        let mut inside = false;
        let mut i = corners.len().wrapping_sub(1);
        for (j, b) in corners.iter().enumerate() {
            let a = corners[i];
            if (p.y <= a.y) == (p.y > b.y) {
                let x_cross = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if x_cross > p.x {
                    inside = !inside;
                }
            }
            i = j;
        }
        inside
    }
}

/// One box and the commands it is for (Nav2's `velocity_polygons` entry, not holonomic).
#[derive(Debug, Clone, PartialEq)]
pub struct SpeedBand {
    /// Its name in nav2.yaml.
    pub name: String,
    /// The box, in `base_link`.
    pub polygon: Polygon,
    /// The forward speeds it is for (m/s).
    pub speed: RangeInclusive<f64>,
    /// The turn rates it is for (rad/s).
    pub turn: RangeInclusive<f64>,
}

impl SpeedBand {
    fn holds(&self, cmd: Command) -> bool {
        self.turn.contains(&cmd.turn) && self.speed.contains(&cmd.x)
    }
}

/// What a sensor sends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceKind {
    /// A laser scan: points on its plane, no height filter.
    Scan,
    /// A point cloud: points kept between these heights in `base_link` (m).
    Cloud {
        /// m.
        min_height: f64,
        /// m.
        max_height: f64,
    },
}

/// Where a sensor sits in `base_link`: (x, y, z) m and its yaw (rad). The sensors we use
/// are level; roll and pitch are left out.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Mount {
    /// m, + forward.
    pub x: f64,
    /// m, + left.
    pub y: f64,
    /// m, + up.
    pub z: f64,
    /// rad, + left.
    pub yaw: f64,
}

/// One sensor the zone watches.
#[derive(Debug, Clone, PartialEq)]
pub struct Source {
    /// Its name in nav2.yaml.
    pub name: String,
    /// A scan or a cloud.
    pub kind: SourceKind,
    /// Where it sits on the car.
    pub mount: Mount,
    /// m from the sensor: closer points are its own clutter (clouds only).
    pub min_range: f64,
}

/// A sensor's newest reading: its stamp (s) and its points in its own frame (m).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Reading {
    /// s.
    pub stamp: f64,
    /// x, y, z (m), in the sensor's frame.
    pub points: Vec<[f64; 3]>,
}

/// The zone's settings (nav2.yaml's `collision_monitor`).
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// The boxes, each for its commands.
    pub bands: Vec<SpeedBand>,
    /// Points in the box, all sensors together, that stop the car.
    pub min_points: usize,
    /// s: a sensor's newest reading older than this stops the car (0: never).
    pub source_timeout: f64,
    /// s: after a stop, zeros keep going out this long, then nothing.
    pub stop_pub_timeout: f64,
}

/// Why the zone stopped the command.
#[derive(Debug, Clone, PartialEq)]
pub enum Why {
    /// This many points in the box `band`.
    Points {
        /// The box.
        band: String,
        /// The points in it.
        count: usize,
    },
    /// A sensor has sent nothing, or nothing new enough.
    Silent {
        /// The sensor.
        source: String,
    },
}

/// The polygon name collision_monitor's state gives a silent sensor's stop.
pub const INVALID_SOURCE: &str = "invalid source";

impl Why {
    /// The polygon name the state says for this stop, as collision_monitor's: the box's
    /// `polygon` name for points in it, [`INVALID_SOURCE`] for a silent sensor.
    #[must_use]
    pub fn polygon_name(&self, polygon: &str) -> String {
        match self {
            Why::Points { .. } => polygon.to_owned(),
            Why::Silent { .. } => INVALID_SOURCE.to_owned(),
        }
    }
}

impl fmt::Display for Why {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Why::Points { band, count } => write!(f, "{count} points in the {band} box"),
            Why::Silent { source } => write!(f, "{source} is silent"),
        }
    }
}

/// One step's answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Decision {
    /// The command to send on; `None`: send nothing (stopped longer than stop_pub_timeout).
    pub out: Option<Command>,
    /// Why it stopped; `None`: the command went through.
    pub stop: Option<Why>,
    /// The box that was checked.
    pub band: Option<String>,
}

/// Why the settings can't make a zone.
#[derive(Debug, Clone, PartialEq)]
pub enum SettingsError {
    /// No box at all.
    NoBands,
    /// A box with fewer than three corners.
    BadBox(String),
    /// `min_points` is 0: the car would always stop.
    ZeroMinPoints,
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SettingsError::NoBands => write!(f, "the stop zone needs at least one box"),
            SettingsError::BadBox(name) => write!(f, "the {name} box needs three corners or more"),
            SettingsError::ZeroMinPoints => write!(f, "min_points must be 1 or more"),
        }
    }
}

impl std::error::Error for SettingsError {}

/// The stop zone: settings, sensors, and what it did last.
#[derive(Debug, Clone)]
pub struct StopZone {
    settings: Settings,
    sources: Vec<Source>,
    band: usize,
    last_zero: bool,
    stopped_at: f64,
}

impl StopZone {
    /// A zone for these settings, watching `sources`.
    ///
    /// # Errors
    ///
    /// [`SettingsError`] for no box, a box under three corners, or `min_points` 0.
    pub fn new(settings: Settings, sources: Vec<Source>) -> Result<Self, SettingsError> {
        if settings.bands.is_empty() {
            return Err(SettingsError::NoBands);
        }
        if let Some(bad) = settings.bands.iter().find(|b| b.polygon.0.len() < 3) {
            return Err(SettingsError::BadBox(bad.name.clone()));
        }
        if settings.min_points == 0 {
            return Err(SettingsError::ZeroMinPoints);
        }
        Ok(Self {
            settings,
            sources,
            band: 0,
            last_zero: false,
            stopped_at: f64::NEG_INFINITY,
        })
    }

    /// The sensors it watches.
    #[must_use]
    pub fn sources(&self) -> &[Source] {
        &self.sources
    }

    /// One command at `now` (s), with each source's newest reading, in `sources()` order
    /// (`None`: nothing yet).
    pub fn step(&mut self, now: f64, cmd: Command, readings: &[Option<&Reading>]) -> Decision {
        let mut stop = self.silent(now, readings);
        let mut band_name = None;
        if stop.is_none() {
            // The first band that holds the command; none: the last one stays (Nav2).
            if let Some(i) = self.settings.bands.iter().position(|b| b.holds(cmd)) {
                self.band = i;
            }
            let band = &self.settings.bands[self.band];
            band_name = Some(band.name.clone());
            let count: usize = self
                .sources
                .iter()
                .zip(readings)
                .filter_map(|(source, reading)| reading.map(|r| (source, r)))
                .map(|(source, r)| {
                    in_base(source, r)
                        .filter(|p| band.polygon.contains(*p))
                        .count()
                })
                .sum();
            if count >= self.settings.min_points {
                stop = Some(Why::Points {
                    band: band.name.clone(),
                    count,
                });
            }
        }
        let wanted = if stop.is_some() { Command::STOP } else { cmd };
        Decision {
            out: self.publish(now, wanted),
            stop,
            band: band_name,
        }
    }

    /// The first source with no reading, or one older than source_timeout.
    fn silent(&self, now: f64, readings: &[Option<&Reading>]) -> Option<Why> {
        let timeout = self.settings.source_timeout;
        self.sources.iter().enumerate().find_map(|(i, source)| {
            let fresh = match readings.get(i).copied().flatten() {
                None => false,
                Some(r) => timeout == 0.0 || now - r.stamp <= timeout,
            };
            (!fresh).then(|| Why::Silent {
                source: source.name.clone(),
            })
        })
    }

    /// Nav2's `publishVelocity`: a zero right after a moving command marks the stop; zeros
    /// then go out for stop_pub_timeout s, then nothing until a moving command comes.
    fn publish(&mut self, now: f64, wanted: Command) -> Option<Command> {
        let out = if wanted.is_zero() {
            if !self.last_zero {
                self.stopped_at = now;
                Some(wanted)
            } else if now - self.stopped_at > self.settings.stop_pub_timeout {
                None
            } else {
                Some(wanted)
            }
        } else {
            Some(wanted)
        };
        self.last_zero = wanted.is_zero();
        out
    }
}

/// A reading's points that count, on the ground in `base_link`: clouds without the
/// sensor's own clutter (closer than min_range) and only between their heights.
fn in_base<'a>(source: &'a Source, reading: &'a Reading) -> impl Iterator<Item = Point> + 'a {
    let m = source.mount;
    let (sin, cos) = m.yaw.sin_cos();
    reading.points.iter().filter_map(move |&[x, y, z]| {
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            return None;
        }
        let base = Point::new(m.x + cos * x - sin * y, m.y + sin * x + cos * y);
        match source.kind {
            SourceKind::Scan => Some(base),
            SourceKind::Cloud {
                min_height,
                max_height,
            } => {
                let range = (x * x + y * y + z * z).sqrt();
                let height = m.z + z;
                (range >= source.min_range && (min_height..=max_height).contains(&height))
                    .then_some(base)
            }
        }
    })
}

#[cfg(test)]
mod tests;

//! The GRAPHS page: does the Pi pass ego's commands on to the ESP32? (docs/designs/
//! 2026-11-07_esp32-debug-link_brief.md, "The plan now"; review 2, M1-M4.)
//!
//! ```text
//! /ego/actuator_cmd ──► /control/car/actuator_cmd ──► /control/car/link_frame ──► the ESP32's status
//!        ego         ──►          mux             ──►        Pi wrote          ──►    ESP32 got
//! ```
//!
//! Each hop gets a [`Verdict`]: OK, DIFFERENT, STALE, NO DATA or NOT TESTED. A value that
//! never changed proves nothing, so a parked car shows NOT TESTED, never OK. How each hop
//! is judged:
//!
//! | Hop | Compared with the hop before | How |
//! |---|---|---|
//! | ego | nothing | fresh, and changed |
//! | mux | ego | paired by the header's stamp (the mux copies it): equal |
//! | Pi wrote | mux | no key to pair by (the bridge writes on its own timer, in other units): it must change when the mux's does, the same way (correlation 0.5 or more) |
//! | ESP32 got | Pi wrote | paired by counter and check byte (review 2, M1): equal within the JSON's rounding |
//!
//! The proof of the wire is counts (review 2, M2): the status strip's frames written,
//! frames counted, lost and bad. The graphs only draw a sample: "sampled N frames", never
//! "every frame".
//!
//! Three graphs, over the last 60 s, each with now / avg / low / high: steering, brake and
//! throttle, in the frame's own units (-1..1). Two lines each: what the Pi wrote, and what
//! the ESP32 says it got (its status lines' `rx`; without them, the uplink's brake and
//! throttle outputs, labelled so). FREEZE keeps a copy to read while the car carries on.

use std::{
    collections::{HashMap, VecDeque},
    fmt,
    time::{Duration, Instant},
};

use crate::esp32::{LinkCounts, Rx, StatusLine};

/// How far back the graphs, the numbers and the verdicts look.
pub const WINDOW: Duration = Duration::from_secs(60);
/// Samples further apart than this are drawn with a gap.
pub const GAP: Duration = Duration::from_millis(500);
/// A change smaller than this is no change: in rad, m/s or the frame's -1..1.
pub const CHANGED: f64 = 0.02;
/// The JSON's rounding: a received value this close to the written one is the same.
pub const RX_TOLERANCE: f64 = 0.01;
/// A frame's `accel` below this is a brake demand (the firmware's own threshold).
pub const BRAKE_DEMAND: f64 = -0.05;
/// What "follows" means between the mux and the frames: a correlation at least this.
pub const FOLLOWS: f64 = 0.5;

/// One of the three graphs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    /// The steering actuator.
    Steering,
    /// The brake actuator.
    Brake,
    /// The throttle.
    Throttle,
}

impl Channel {
    /// Every channel.
    pub const ALL: [Channel; 3] = [Channel::Steering, Channel::Brake, Channel::Throttle];

    /// Its name on the screen.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Steering => "STEERING",
            Self::Brake => "BRAKE",
            Self::Throttle => "THROTTLE",
        }
    }

    /// The channel's number in a frame's (steering, speed, accel), -1..1.
    #[must_use]
    pub fn of_frame(self, steering: f64, speed: f64, accel: f64) -> f64 {
        match self {
            Self::Steering => steering,
            Self::Throttle => speed,
            Self::Brake if accel < BRAKE_DEMAND => -accel,
            Self::Brake => 0.0,
        }
    }

    /// The channel in ego's command: rad, m/s; ego has no brake value (review 2, M3).
    #[must_use]
    pub fn of_drive(self, d: &Drive) -> Option<f64> {
        match self {
            Self::Steering => Some(d.steering),
            Self::Throttle => Some(d.speed),
            Self::Brake => None,
        }
    }
}

/// A command from ego or the output mux (ackermann_msgs/AckermannDriveStamped).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drive {
    /// The header's stamp, ns: the mux copies ego's, so the two pair by it. 0: none.
    pub stamp_ns: u64,
    /// m/s.
    pub speed: f64,
    /// rad, + left.
    pub steering: f64,
}

/// One frame the Pi wrote (aav_interfaces/LinkFrame).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PiFrame {
    /// The frame's version.
    pub version: u8,
    /// The frame's counter.
    pub counter: u8,
    /// The frame's last byte.
    pub check: u8,
    /// Armed.
    pub armed: bool,
    /// An e-stop.
    pub estop: bool,
    /// -1..1.
    pub steering: f64,
    /// -1..1.
    pub speed: f64,
    /// -1..1.
    pub accel: f64,
    /// Frames written since the bridge started.
    pub written: u32,
    /// Writes that failed since the bridge started.
    pub write_errors: u32,
}

impl PiFrame {
    /// One channel's value.
    #[must_use]
    pub fn value(&self, c: Channel) -> f64 {
        c.of_frame(self.steering, self.speed, self.accel)
    }
}

/// How the ESP32's uplink frames arrive (aav_interfaces/UplinkHealth).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UplinkHealth {
    /// Frames are arriving.
    pub fresh: bool,
    /// Frames that came.
    pub frames: u32,
    /// Frames missed.
    pub lost: u32,
    /// Frames with a bad check.
    pub bad: u32,
}

/// A hop on the way to the ESP32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hop {
    /// ego's command.
    Ego,
    /// The output mux's command.
    Mux,
    /// What the Pi wrote.
    PiWrote,
    /// What the ESP32 got.
    Esp32Got,
}

impl Hop {
    /// Every hop, in order.
    pub const ALL: [Hop; 4] = [Hop::Ego, Hop::Mux, Hop::PiWrote, Hop::Esp32Got];

    /// Its name on the screen.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Ego => "ego",
            Self::Mux => "mux",
            Self::PiWrote => "Pi wrote",
            Self::Esp32Got => "ESP32 got",
        }
    }
}

/// What a hop, or one channel of it, shows.
#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    /// Fresh, changed, and the same as the hop before.
    Ok(String),
    /// Fresh, but not the same as the hop before: how.
    Different(String),
    /// Heard, but not for a while.
    Stale,
    /// Nothing heard in the window.
    NoData,
    /// Nothing to judge by: the value never changed, or nothing to compare with.
    NotTested(String),
}

impl Verdict {
    /// Its word on the screen.
    #[must_use]
    pub fn word(&self) -> &'static str {
        match self {
            Self::Ok(_) => "OK",
            Self::Different(_) => "DIFFERENT",
            Self::Stale => "STALE",
            Self::NoData => "NO DATA",
            Self::NotTested(_) => "NOT TESTED",
        }
    }

    /// Why, in a few words (empty for STALE and NO DATA).
    #[must_use]
    pub fn why(&self) -> &str {
        match self {
            Self::Ok(s) | Self::Different(s) | Self::NotTested(s) => s,
            Self::Stale | Self::NoData => "",
        }
    }
}

impl fmt::Display for Verdict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.why() {
            "" => f.write_str(self.word()),
            why => write!(f, "{}: {why}", self.word()),
        }
    }
}

/// One hop's verdict, and each channel's.
#[derive(Debug, Clone, PartialEq)]
pub struct HopReport {
    /// Which hop.
    pub hop: Hop,
    /// The hop's verdict.
    pub verdict: Verdict,
    /// Each channel's.
    pub channels: Vec<(Channel, Verdict)>,
}

/// A graph's two lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Line {
    /// What the Pi wrote.
    PiWrote,
    /// What the ESP32 got or drives.
    Esp32,
}

/// Where the ESP32's line comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Esp32Line {
    /// Its status lines' `rx`: what it got, in the frame's units.
    Received,
    /// No status lines: the uplink's output (`/vehicle/brake`, `/vehicle/throttle`'s
    /// command), what it drives, not what it got.
    UplinkOutput,
}

/// One point of a line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// When.
    pub at: Instant,
    /// The value.
    pub value: f64,
    /// Lines lost, a restart or a long silence before it: don't join it to the last.
    pub gap_before: bool,
}

/// now / avg / low / high.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    /// The latest.
    pub now: f64,
    /// The mean.
    pub avg: f64,
    /// The lowest.
    pub low: f64,
    /// The highest.
    pub high: f64,
    /// How many points.
    pub count: usize,
}

impl Stats {
    /// The numbers of these points; None for none.
    #[must_use]
    pub fn of(points: &[Point]) -> Option<Self> {
        let now = points.last()?.value;
        let (sum, low, high) = points
            .iter()
            .fold((0.0, f64::INFINITY, f64::NEG_INFINITY), |(s, lo, hi), p| {
                (s + p.value, lo.min(p.value), hi.max(p.value))
            });
        Some(Self {
            now,
            avg: sum / points.len() as f64,
            low,
            high,
            count: points.len(),
        })
    }
}

/// The status strip: the counts that prove the wire.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Counts {
    /// The Pi's frames written and failed writes since its bridge started, and the
    /// frames written in the window.
    pub pi: Option<(u32, u32, u32)>,
    /// The ESP32's uplink, last heard.
    pub uplink: Option<UplinkHealth>,
    /// The ESP32's own counts since boot, and their rise in the window.
    pub esp32: Option<(LinkCounts, LinkCounts)>,
    /// Status lines that never came (gaps in `seq`).
    pub lines_lost: u64,
    /// The board restarted in the window.
    pub restarted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Timed<T> {
    at: Instant,
    value: T,
}

#[derive(Debug, Clone)]
struct Log<T>(VecDeque<Timed<T>>);

impl<T> Default for Log<T> {
    fn default() -> Self {
        Self(VecDeque::new())
    }
}

impl<T: Copy> Log<T> {
    fn push(&mut self, at: Instant, value: T) {
        self.0.push_back(Timed { at, value });
        while self
            .0
            .front()
            .is_some_and(|t| at.saturating_duration_since(t.at) > WINDOW)
        {
            self.0.pop_front();
        }
    }

    fn since(&self, from: Instant) -> impl Iterator<Item = &Timed<T>> {
        self.0.iter().filter(move |t| t.at >= from)
    }

    fn last(&self) -> Option<&Timed<T>> {
        self.0.back()
    }

    /// NO DATA or STALE, or None while fresh.
    fn freshness(&self, now: Instant, stale_after: Duration) -> Option<Verdict> {
        match self.last() {
            None => Some(Verdict::NoData),
            Some(t) if now.saturating_duration_since(t.at) > stale_after => Some(Verdict::Stale),
            Some(_) => None,
        }
    }
}

/// The graphs' data: everything heard in the last 60 s.
#[derive(Debug, Clone, Default)]
pub struct Graphs {
    ego: Log<Drive>,
    mux: Log<Drive>,
    pi: Log<PiFrame>,
    rx: Log<Rx>,
    /// Every status line's link counts: also when the status lines were heard.
    esp32_link: Log<LinkCounts>,
    uplink: Log<UplinkHealth>,
    uplink_brake: Log<f64>,
    uplink_throttle: Log<f64>,
    /// The IMU's turn rate, deg/s.
    imu_yaw: Log<f64>,
    /// Where lines were lost, or the board restarted: the lines break there.
    breaks: Log<()>,
    restarts: Log<()>,
    last_seq: Option<u64>,
    lines_lost: u64,
    boot: Option<(String, u64)>,
}

impl Graphs {
    /// Keeps ego's command.
    pub fn record_ego(&mut self, at: Instant, d: Drive) {
        self.ego.push(at, d);
    }

    /// Keeps the output mux's command.
    pub fn record_mux(&mut self, at: Instant, d: Drive) {
        self.mux.push(at, d);
    }

    /// Keeps a frame the Pi wrote.
    pub fn record_pi(&mut self, at: Instant, f: PiFrame) {
        self.pi.push(at, f);
    }

    /// Keeps the uplink's health.
    pub fn record_uplink(&mut self, at: Instant, h: UplinkHealth) {
        self.uplink.push(at, h);
    }

    /// `/vehicle/brake` or `/vehicle/throttle`'s command: what the ESP32 drives (legacy
    /// bridge, uplink true). NaN (unknown) is left out.
    pub fn record_uplink_output(&mut self, at: Instant, c: Channel, value: f64) {
        if !value.is_finite() {
            return;
        }
        match c {
            Channel::Brake => self.uplink_brake.push(at, value),
            Channel::Throttle => self.uplink_throttle.push(at, value),
            Channel::Steering => {}
        }
    }

    /// Keeps the IMU's turn rate (degrees a second); NaN is left out.
    pub fn record_imu_yaw(&mut self, at: Instant, deg_per_s: f64) {
        if deg_per_s.is_finite() {
            self.imu_yaw.push(at, deg_per_s);
        }
    }

    /// One of the ESP32's status lines.
    pub fn record_esp32(&mut self, at: Instant, line: &StatusLine) {
        let seq = line.seq();
        if let Some(last) = self.last_seq {
            let lost = crate::esp32::lost_between(last, seq);
            if lost > 0 {
                self.lines_lost += lost;
                self.breaks.push(at, ());
            }
        }
        self.last_seq = Some(seq);
        match line {
            StatusLine::Heartbeat(h) => {
                let restarted = self
                    .boot
                    .as_ref()
                    .is_some_and(|(id, up)| *id != h.boot_id || h.up_ms < *up);
                if restarted {
                    self.restarts.push(at, ());
                    self.breaks.push(at, ());
                    self.last_seq = Some(seq);
                }
                self.boot = Some((h.boot_id.clone(), h.up_ms));
            }
            StatusLine::Status(s) => {
                self.esp32_link.push(at, s.link);
                if let Some(rx) = s.rx {
                    self.rx.push(at, rx);
                }
            }
            StatusLine::Event(_) | StatusLine::Unknown(_) => {}
        }
    }

    /// The window's start.
    fn from(now: Instant) -> Instant {
        now.checked_sub(WINDOW).unwrap_or(now)
    }

    /// Where the ESP32's line comes from now, if anywhere.
    #[must_use]
    pub fn esp32_line(&self, now: Instant) -> Option<Esp32Line> {
        if self.rx.since(Self::from(now)).next().is_some() {
            Some(Esp32Line::Received)
        } else if self.uplink_brake.last().is_some() || self.uplink_throttle.last().is_some() {
            Some(Esp32Line::UplinkOutput)
        } else {
            None
        }
    }

    /// One graph's line over the window.
    #[must_use]
    pub fn line(&self, c: Channel, line: Line, now: Instant) -> Vec<Point> {
        let from = Self::from(now);
        let raw: Vec<(Instant, f64)> = match line {
            Line::PiWrote => self
                .pi
                .since(from)
                .map(|t| (t.at, t.value.value(c)))
                .collect(),
            Line::Esp32 => match self.esp32_line(now) {
                Some(Esp32Line::Received) => self
                    .rx
                    .since(from)
                    .map(|t| {
                        (
                            t.at,
                            c.of_frame(t.value.steer, t.value.speed, t.value.accel),
                        )
                    })
                    .collect(),
                Some(Esp32Line::UplinkOutput) => {
                    let log = match c {
                        Channel::Brake => &self.uplink_brake,
                        Channel::Throttle => &self.uplink_throttle,
                        Channel::Steering => return Vec::new(),
                    };
                    log.since(from).map(|t| (t.at, t.value)).collect()
                }
                None => Vec::new(),
            },
        };
        self.with_gaps(raw)
    }

    /// The IMU's turn rate over the window, deg/s.
    #[must_use]
    pub fn imu_line(&self, now: Instant) -> Vec<Point> {
        let raw = self
            .imu_yaw
            .since(Self::from(now))
            .map(|t| (t.at, t.value))
            .collect();
        self.with_gaps(raw)
    }

    fn with_gaps(&self, raw: Vec<(Instant, f64)>) -> Vec<Point> {
        let mut last: Option<Instant> = None;
        raw.into_iter()
            .map(|(at, value)| {
                let gap_before = last.is_some_and(|prev| {
                    at.saturating_duration_since(prev) > GAP
                        || self.breaks.0.iter().any(|b| b.at > prev && b.at <= at)
                });
                last = Some(at);
                Point {
                    at,
                    value,
                    gap_before,
                }
            })
            .collect()
    }

    /// When the board restarted, in the window (the graphs mark it).
    #[must_use]
    pub fn restarts(&self, now: Instant) -> Vec<Instant> {
        self.restarts.since(Self::from(now)).map(|t| t.at).collect()
    }

    /// The status strip.
    #[must_use]
    pub fn counts(&self, now: Instant) -> Counts {
        let from = Self::from(now);
        let rise = |first: u32, last: u32| last.saturating_sub(first);
        let pi = self.pi.last().map(|last| {
            let first = self
                .pi
                .since(from)
                .next()
                .map_or(last.value.written, |t| t.value.written);
            (
                last.value.written,
                last.value.write_errors,
                rise(first, last.value.written),
            )
        });
        let esp32 = self.esp32_link.last().map(|last| {
            let l = last.value;
            let f = self.esp32_link.since(from).next().map_or(l, |t| t.value);
            let delta = LinkCounts {
                frames: rise(f.frames, l.frames),
                bad: rise(f.bad, l.bad),
                skipped: rise(f.skipped, l.skipped),
                repeats: rise(f.repeats, l.repeats),
            };
            (l, delta)
        });
        Counts {
            pi,
            uplink: self.uplink.last().map(|t| t.value),
            esp32,
            lines_lost: self.lines_lost,
            restarted: self.restarts.since(from).next().is_some(),
        }
    }

    /// Each hop's verdict, ego first.
    #[must_use]
    pub fn hops(&self, now: Instant, stale_after: Duration) -> Vec<HopReport> {
        let from = Self::from(now);
        vec![
            self.ego_hop(now, from, stale_after),
            self.mux_hop(now, from, stale_after),
            self.pi_hop(now, from, stale_after),
            self.esp32_hop(now, from, stale_after),
        ]
    }

    fn ego_hop(&self, now: Instant, from: Instant, stale: Duration) -> HopReport {
        let channels = Channel::ALL.map(|c| {
            let values: Vec<f64> = self
                .ego
                .since(from)
                .filter_map(|t| c.of_drive(&t.value))
                .collect();
            let v = if c == Channel::Brake {
                Verdict::NotTested("ego has no brake value".into())
            } else if range(&values) < CHANGED {
                not_changed()
            } else {
                Verdict::Ok(format!("changed by {:.2}", range(&values)))
            };
            (c, v)
        });
        report(Hop::Ego, self.ego.freshness(now, stale), None, channels)
    }

    fn mux_hop(&self, now: Instant, from: Instant, stale: Duration) -> HopReport {
        let ego: HashMap<u64, Drive> = self
            .ego
            .since(from)
            .filter(|t| t.value.stamp_ns != 0)
            .map(|t| (t.value.stamp_ns, t.value))
            .collect();
        let pairs: Vec<(Drive, Drive)> = self
            .mux
            .since(from)
            .filter_map(|t| ego.get(&t.value.stamp_ns).map(|e| (*e, t.value)))
            .collect();
        let channels = Channel::ALL.map(|c| {
            let v = if c == Channel::Brake {
                Verdict::NotTested("ego has no brake value".into())
            } else if pairs.is_empty() {
                Verdict::NotTested(if self.ego.last().is_none() {
                    "ego not heard: nothing to compare".into()
                } else {
                    "no command paired with ego's by stamp".into()
                })
            } else {
                let differ = pairs
                    .iter()
                    .filter(|(e, m)| match (c.of_drive(e), c.of_drive(m)) {
                        (Some(a), Some(b)) => (a - b).abs() > 1e-4,
                        _ => false,
                    })
                    .count();
                let values: Vec<f64> = pairs.iter().filter_map(|(_, m)| c.of_drive(m)).collect();
                if differ > 0 {
                    Verdict::Different(format!(
                        "{differ} of {} commands paired by stamp differ",
                        pairs.len()
                    ))
                } else if range(&values) < CHANGED {
                    not_changed()
                } else {
                    Verdict::Ok(format!(
                        "{} commands paired by stamp, all equal",
                        pairs.len()
                    ))
                }
            };
            (c, v)
        });
        report(Hop::Mux, self.mux.freshness(now, stale), None, channels)
    }

    fn pi_hop(&self, now: Instant, from: Instant, stale: Duration) -> HopReport {
        let frames: Vec<&Timed<PiFrame>> = self.pi.since(from).collect();
        let errors = match (frames.first(), frames.last()) {
            (Some(a), Some(b)) if b.value.write_errors > a.value.write_errors => {
                Some(Verdict::Different(format!(
                    "{} writes failed",
                    b.value.write_errors - a.value.write_errors
                )))
            }
            _ => None,
        };
        let mux: Vec<&Timed<Drive>> = self.mux.since(from).collect();
        let channels = Channel::ALL.map(|c| {
            let v = if c == Channel::Brake {
                Verdict::NotTested("the mux has no brake value to follow".into())
            } else if mux.is_empty() {
                Verdict::NotTested("the mux not heard: nothing to compare".into())
            } else {
                // Each frame against the mux's latest command when it was written.
                let pairs: Vec<(f64, f64)> = frames
                    .iter()
                    .filter_map(|f| {
                        let up = mux.iter().rev().find(|m| m.at <= f.at)?;
                        Some((c.of_drive(&up.value)?, f.value.value(c)))
                    })
                    .collect();
                let (up, down): (Vec<f64>, Vec<f64>) = pairs.iter().copied().unzip();
                if range(&up) < CHANGED {
                    Verdict::NotTested("the mux's value hasn't changed".into())
                } else if range(&down) < CHANGED {
                    Verdict::Different(format!(
                        "the mux's changed by {:.2}, what the Pi wrote didn't",
                        range(&up)
                    ))
                } else {
                    let r = correlation(&pairs);
                    if r < FOLLOWS {
                        Verdict::Different(format!("doesn't follow the mux (r = {r:.2})"))
                    } else {
                        Verdict::Ok(format!("follows the mux (r = {r:.2})"))
                    }
                }
            };
            (c, v)
        });
        report(
            Hop::PiWrote,
            self.pi.freshness(now, stale),
            errors,
            channels,
        )
    }

    fn esp32_hop(&self, now: Instant, from: Instant, stale: Duration) -> HopReport {
        if self.esp32_link.last().is_none() {
            // No status lines: the uplink says frames arrive back, not what was got.
            let fresh = self.uplink.freshness(now, stale).or_else(|| {
                self.uplink
                    .last()
                    .filter(|t| !t.value.fresh)
                    .map(|_| Verdict::Stale)
            });
            let why = "the uplink carries no received values: start with --esp32 ws://...";
            let channels = Channel::ALL.map(|c| (c, Verdict::NotTested(why.into())));
            return report(Hop::Esp32Got, fresh, None, channels);
        }
        let links: Vec<&Timed<LinkCounts>> = self.esp32_link.since(from).collect();
        let bad = match (links.first(), links.last()) {
            (Some(a), Some(b)) => {
                let bad = b.value.bad.saturating_sub(a.value.bad);
                let skipped = b.value.skipped.saturating_sub(a.value.skipped);
                (bad + skipped > 0)
                    .then(|| Verdict::Different(format!("{bad} bad and {skipped} skipped frames")))
            }
            _ => None,
        };
        // The counter wraps every 256 frames, so in 60 s one counter and check byte can
        // stand for two different frames. Such a key can't say which frame arrived: it is
        // left out (`None`), as `tools/link_check.py` does. A repeat of one frame is fine.
        let mut written: HashMap<(u8, u8), Option<PiFrame>> = HashMap::new();
        for t in self.pi.since(from) {
            let f = t.value;
            written
                .entry((f.counter, f.check))
                .and_modify(|seen| {
                    if seen.is_some_and(|s| Channel::ALL.iter().any(|&c| s.value(c) != f.value(c)))
                    {
                        *seen = None;
                    }
                })
                .or_insert(Some(f));
        }
        let pairs: Vec<(PiFrame, Rx)> = self
            .rx
            .since(from)
            .filter_map(|t| {
                written
                    .get(&(t.value.counter, t.value.check))
                    .copied()
                    .flatten()
                    .map(|f| (f, t.value))
            })
            .collect();
        let channels = Channel::ALL.map(|c| {
            let v = if pairs.is_empty() {
                Verdict::NotTested(if self.pi.last().is_none() {
                    "the Pi's frames not heard: nothing to compare".into()
                } else {
                    "no frame paired by counter and check".into()
                })
            } else {
                let differ = pairs
                    .iter()
                    .filter(|(f, rx)| {
                        (f.value(c) - c.of_frame(rx.steer, rx.speed, rx.accel)).abs() > RX_TOLERANCE
                    })
                    .count();
                let values: Vec<f64> = pairs.iter().map(|(f, _)| f.value(c)).collect();
                if differ > 0 {
                    Verdict::Different(format!("{differ} of {} sampled frames differ", pairs.len()))
                } else if range(&values) < CHANGED {
                    not_changed()
                } else {
                    Verdict::Ok(format!("sampled {} frames, all equal", pairs.len()))
                }
            };
            (c, v)
        });
        report(
            Hop::Esp32Got,
            self.esp32_link.freshness(now, stale),
            bad,
            channels,
        )
    }
}

fn not_changed() -> Verdict {
    Verdict::NotTested("hasn't changed: a parked car tests nothing".into())
}

/// A hop's verdict: no data or stale first, then a fault of the whole hop, then its
/// channels: any DIFFERENT, else any OK, else NOT TESTED.
fn report(
    hop: Hop,
    freshness: Option<Verdict>,
    fault: Option<Verdict>,
    channels: [(Channel, Verdict); 3],
) -> HopReport {
    let verdict = freshness.or(fault).unwrap_or_else(|| {
        let of = |word: &str| -> Vec<String> {
            channels
                .iter()
                .filter(|(_, v)| v.word() == word)
                .map(|(c, v)| format!("{} {}", c.label().to_lowercase(), v.why()))
                .collect()
        };
        let named = |word: &str| -> Vec<String> {
            channels
                .iter()
                .filter(|(_, v)| v.word() == word)
                .map(|(c, _)| c.label().to_lowercase())
                .collect()
        };
        let different = of("DIFFERENT");
        let tested = named("OK");
        if !different.is_empty() {
            Verdict::Different(different.join("; "))
        } else if !tested.is_empty() {
            let untested = named("NOT TESTED");
            let mut s = format!("{} tested", tested.join(", "));
            if !untested.is_empty() {
                s.push_str(&format!("; {} not", untested.join(", ")));
            }
            Verdict::Ok(s)
        } else {
            let first = channels
                .iter()
                .find(|(c, _)| *c != Channel::Brake)
                .map_or("", |(_, v)| v.why());
            Verdict::NotTested(first.to_owned())
        }
    });
    HopReport {
        hop,
        verdict,
        channels: channels.to_vec(),
    }
}

/// The highest minus the lowest; 0 for none.
fn range(values: &[f64]) -> f64 {
    let (lo, hi) = values
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
            (lo.min(v), hi.max(v))
        });
    if values.is_empty() {
        0.0
    } else {
        hi - lo
    }
}

/// Pearson's correlation of the pairs; 0 when either side is flat.
fn correlation(pairs: &[(f64, f64)]) -> f64 {
    let n = pairs.len() as f64;
    if pairs.len() < 2 {
        return 0.0;
    }
    let (mx, my) = pairs
        .iter()
        .fold((0.0, 0.0), |(a, b), (x, y)| (a + x / n, b + y / n));
    let (sxy, sxx, syy) = pairs.iter().fold((0.0, 0.0, 0.0), |(xy, xx, yy), (x, y)| {
        let (dx, dy) = (x - mx, y - my);
        (xy + dx * dy, xx + dx * dx, yy + dy * dy)
    });
    if sxx <= 0.0 || syy <= 0.0 {
        0.0
    } else {
        sxy / (sxx * syy).sqrt()
    }
}

/// Why the graphs are frozen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreezeWhy {
    /// FREEZE pressed.
    Pressed,
    /// An e-stop came on: frozen at once, to read what led to it.
    Estop,
}

/// FREEZE: a copy of the graphs to read, while the live ones carry on.
#[derive(Debug, Clone, Default)]
pub struct Freezer {
    frozen: Option<(Instant, FreezeWhy, Box<Graphs>)>,
    estop_was: bool,
}

impl Freezer {
    /// FREEZE pressed: freeze now, or go live again.
    pub fn toggle(&mut self, now: Instant, live: &Graphs) {
        self.frozen = match self.frozen {
            Some(_) => None,
            None => Some((now, FreezeWhy::Pressed, Box::new(live.clone()))),
        };
    }

    /// Each frame: an e-stop coming on freezes (unless already frozen).
    pub fn watch_estop(&mut self, now: Instant, estop: bool, live: &Graphs) {
        if estop && !self.estop_was && self.frozen.is_none() {
            self.frozen = Some((now, FreezeWhy::Estop, Box::new(live.clone())));
        }
        self.estop_was = estop;
    }

    /// When the graphs froze, and why; None while live.
    #[must_use]
    pub fn frozen(&self) -> Option<(Instant, FreezeWhy)> {
        self.frozen.as_ref().map(|(at, why, _)| (*at, *why))
    }

    /// What to draw, and as of when.
    #[must_use]
    pub fn view<'a>(&'a self, live: &'a Graphs, now: Instant) -> (&'a Graphs, Instant) {
        match &self.frozen {
            Some((at, _, copy)) => (copy, *at),
            None => (live, now),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::esp32::{parse_line, Heartbeat};

    const STALE: Duration = Duration::from_secs(1);

    fn ms(t: Instant, n: u64) -> Instant {
        t + Duration::from_millis(n)
    }

    fn drive(stamp: u64, speed: f64, steering: f64) -> Drive {
        Drive {
            stamp_ns: stamp,
            speed,
            steering,
        }
    }

    fn frame(counter: u8, steering: f64, speed: f64, accel: f64, written: u32) -> PiFrame {
        PiFrame {
            version: 2,
            counter,
            // Stands in for the CRC: a different value, a different byte.
            check: (steering * 100.0) as i8 as u8 ^ (accel * 10.0) as i8 as u8,
            armed: true,
            estop: false,
            steering,
            speed,
            accel,
            written,
            write_errors: 0,
        }
    }

    fn status(seq: u64, rx: Option<Rx>, frames: u32, bad: u32) -> StatusLine {
        let mut line = parse_line(
            include_str!("../../../../../../firmware/status/golden.jsonl")
                .lines()
                .nth(1)
                .expect("a status line"),
        )
        .expect("golden");
        if let StatusLine::Status(s) = &mut line {
            s.seq = seq;
            s.rx = rx;
            s.link.frames = frames;
            s.link.bad = bad;
        }
        line
    }

    fn rx_of(f: &PiFrame) -> Rx {
        Rx {
            v: 2,
            counter: f.counter,
            check: f.check,
            // As the JSON rounds it.
            steer: (f.steering * 1000.0).round() / 1000.0,
            speed: (f.speed * 1000.0).round() / 1000.0,
            accel: (f.accel * 1000.0).round() / 1000.0,
            age_ms: 10,
        }
    }

    /// 10 s of a steering sweep through the whole path, every hop as it should be.
    fn sweep(g: &mut Graphs, t: Instant, steer: impl Fn(f64) -> f64) -> Instant {
        let mut now = t;
        for i in 0..100u32 {
            now = ms(t, u64::from(i) * 100);
            let s = steer(f64::from(i) / 10.0);
            let d = drive(u64::from(i) + 1, 0.3, s);
            g.record_ego(now, d);
            g.record_mux(now, d);
            let f = frame(i as u8, s / 0.5, 0.1, 0.0, i * 5);
            g.record_pi(now, f);
            g.record_esp32(now, &status(u64::from(i), Some(rx_of(&f)), i * 5, 0));
        }
        now
    }

    #[test]
    fn a_working_path_is_ok_hop_by_hop() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let now = sweep(&mut g, t, |s| 0.3 * (s * 1.3).sin());
        let hops = g.hops(now, STALE);
        let words: Vec<&str> = hops.iter().map(|h| h.verdict.word()).collect();
        assert_eq!(words, vec!["OK", "OK", "OK", "OK"], "{hops:#?}");
        // Throttle never changed, brake has no ego value: each says so.
        let esp = &hops[3];
        assert!(
            esp.verdict.why().contains("steering tested"),
            "{:?}",
            esp.verdict
        );
        let steer = &esp.channels[0].1;
        assert!(steer.why().starts_with("sampled 100 frames"), "{steer:?}");
        assert_eq!(esp.channels[2].1.word(), "NOT TESTED");
        let c = g.counts(now);
        assert_eq!(c.pi, Some((495, 0, 495)));
        assert_eq!(c.esp32.map(|(_, d)| d.frames), Some(495));
    }

    #[test]
    fn a_parked_car_is_never_ok() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let now = sweep(&mut g, t, |_| 0.0);
        for h in g.hops(now, STALE) {
            assert_eq!(h.verdict.word(), "NOT TESTED", "{h:?}");
        }
        assert!(g.hops(now, STALE)[0].verdict.why().contains("parked"));
    }

    #[test]
    fn nothing_heard_is_no_data_and_quiet_is_stale() {
        let t = Instant::now();
        let g = Graphs::default();
        for h in g.hops(t, STALE) {
            assert_eq!(h.verdict, Verdict::NoData, "{h:?}");
        }
        let mut g = Graphs::default();
        let end = sweep(&mut g, t, |s| s / 20.0);
        let later = end + Duration::from_secs(3);
        for h in g.hops(later, STALE) {
            assert_eq!(h.verdict, Verdict::Stale, "{h:?}");
        }
    }

    #[test]
    fn a_frame_received_wrong_is_different() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let mut now = t;
        for i in 0..50u32 {
            now = ms(t, u64::from(i) * 100);
            let s = f64::from(i) / 100.0;
            let d = drive(u64::from(i) + 1, 0.3, s);
            g.record_ego(now, d);
            g.record_mux(now, d);
            let f = frame(i as u8, s, 0.1, 0.0, i);
            g.record_pi(now, f);
            let mut rx = rx_of(&f);
            if i == 30 {
                rx.steer += 0.2;
            }
            g.record_esp32(now, &status(u64::from(i), Some(rx), i, 0));
        }
        let esp = &g.hops(now, STALE)[3];
        assert_eq!(esp.verdict.word(), "DIFFERENT");
        assert!(
            esp.verdict.why().contains("1 of 50 sampled frames differ"),
            "{esp:?}"
        );
        // Pairs by counter and check: rx frames whose check matches nothing are left out.
    }

    #[test]
    fn a_key_two_frames_share_is_left_out_not_different() {
        // The counter wraps: frames 3 and 259 share counter 3 and the check byte (the test's
        // check ignores speed), with different speeds. The ESP32 got frame 259 right: the
        // key is ambiguous, so it is left out, not called a difference.
        let t = Instant::now();
        let mut g = Graphs::default();
        let mut now = t;
        for i in 0..300u32 {
            now = ms(t, u64::from(i) * 50);
            let s = f64::from(i % 256 % 50) / 100.0;
            let d = drive(u64::from(i) + 1, 0.3, s);
            g.record_ego(now, d);
            g.record_mux(now, d);
            let speed = if i == 259 { 0.4 } else { 0.1 };
            let f = frame(i as u8, s, speed, 0.0, i);
            g.record_pi(now, f);
            g.record_esp32(now, &status(u64::from(i), Some(rx_of(&f)), i, 0));
        }
        assert_eq!(
            frame(3, 0.03, 0.1, 0.0, 3).check,
            frame(3, 0.03, 0.4, 0.0, 259).check
        );
        let esp = &g.hops(now, STALE)[3];
        assert_ne!(esp.verdict.word(), "DIFFERENT", "{esp:?}");
    }

    #[test]
    fn bad_frames_on_the_board_are_different() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let now = sweep(&mut g, t, |s| s / 20.0);
        g.record_esp32(now, &status(100, None, 500, 3));
        let esp = &g.hops(now, STALE)[3];
        assert!(
            matches!(&esp.verdict, Verdict::Different(w) if w.contains("3 bad")),
            "{esp:?}"
        );
    }

    #[test]
    fn the_mux_changing_a_command_is_different() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let mut now = t;
        for i in 0..20u64 {
            now = ms(t, i * 100);
            let s = i as f64 / 50.0;
            g.record_ego(now, drive(i + 1, 0.3, s));
            g.record_mux(now, drive(i + 1, 0.3, if i == 7 { 0.0 } else { s }));
        }
        let mux = &g.hops(now, STALE)[1];
        assert!(mux.verdict.why().contains("1 of 20 commands"), "{mux:?}");
    }

    #[test]
    fn a_pi_that_doesnt_follow_or_goes_the_other_way_is_different() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let mut now = t;
        for i in 0..40u32 {
            now = ms(t, u64::from(i) * 100);
            let s = (f64::from(i) / 5.0).sin() * 0.3;
            g.record_mux(now, drive(u64::from(i) + 1, 0.3, s));
            // The wrong sign: steering_sign not set for this car.
            g.record_pi(now, frame(i as u8, -s / 0.5, 0.1, 0.0, i));
        }
        let pi = &g.hops(now, STALE)[2];
        assert!(pi.verdict.why().contains("doesn't follow"), "{pi:?}");
        // Flat while the mux moves.
        let mut g = Graphs::default();
        for i in 0..40u32 {
            now = ms(t, u64::from(i) * 100);
            g.record_mux(now, drive(u64::from(i) + 1, 0.3, f64::from(i) / 100.0));
            g.record_pi(now, frame(i as u8, 0.0, 0.1, 0.0, i));
        }
        let pi = &g.hops(now, STALE)[2];
        assert!(pi.verdict.why().contains("didn't"), "{pi:?}");
    }

    #[test]
    fn without_status_lines_the_esp32_hop_says_what_it_needs() {
        let t = Instant::now();
        let mut g = Graphs::default();
        g.record_uplink(
            t,
            UplinkHealth {
                fresh: true,
                frames: 10,
                lost: 0,
                bad: 0,
            },
        );
        let esp = &g.hops(t, STALE)[3];
        assert_eq!(esp.verdict.word(), "NOT TESTED");
        assert!(esp.verdict.why().contains("--esp32"));
        g.record_uplink(
            t,
            UplinkHealth {
                fresh: false,
                frames: 10,
                lost: 2,
                bad: 0,
            },
        );
        assert_eq!(g.hops(t, STALE)[3].verdict, Verdict::Stale);
        // The graph's ESP32 line falls back to the uplink's outputs, labelled.
        g.record_uplink_output(t, Channel::Brake, 0.3);
        g.record_uplink_output(t, Channel::Brake, f64::NAN);
        assert_eq!(g.esp32_line(t), Some(Esp32Line::UplinkOutput));
        assert_eq!(g.line(Channel::Brake, Line::Esp32, t).len(), 1);
        assert!(g.line(Channel::Steering, Line::Esp32, t).is_empty());
    }

    #[test]
    fn lines_break_at_lost_lines_restarts_and_silences() {
        let t = Instant::now();
        let mut g = Graphs::default();
        let f = frame(1, 0.1, 0.0, -0.5, 1);
        let hb = |boot: &str, up| {
            StatusLine::Heartbeat(Heartbeat {
                seq: 0,
                up_ms: up,
                boot_id: boot.into(),
                build: String::new(),
                features: String::new(),
                last_reset: String::new(),
            })
        };
        g.record_esp32(t, &hb("a", 1000));
        g.record_esp32(ms(t, 100), &status(1, Some(rx_of(&f)), 1, 0));
        g.record_esp32(ms(t, 200), &status(2, Some(rx_of(&f)), 2, 0));
        g.record_esp32(ms(t, 300), &status(6, Some(rx_of(&f)), 3, 0)); // 3 lost
        g.record_esp32(ms(t, 1300), &status(7, Some(rx_of(&f)), 4, 0)); // 1 s quiet
        let line = g.line(Channel::Brake, Line::Esp32, ms(t, 1300));
        let gaps: Vec<bool> = line.iter().map(|p| p.gap_before).collect();
        assert_eq!(gaps, vec![false, false, true, true]);
        assert_eq!(line[0].value, 0.5, "a brake demand");
        let c = g.counts(ms(t, 1300));
        assert_eq!(c.lines_lost, 3);
        assert!(!c.restarted);
        g.record_esp32(ms(t, 1400), &hb("b", 50));
        assert!(g.counts(ms(t, 1400)).restarted);
        assert_eq!(g.restarts(ms(t, 1400)).len(), 1);
    }

    #[test]
    fn the_window_is_sixty_seconds_and_the_numbers_cover_it() {
        let t = Instant::now();
        let mut g = Graphs::default();
        for i in 0..=90u32 {
            g.record_pi(
                ms(t, u64::from(i) * 1000),
                frame(i as u8, f64::from(i) / 100.0, 0.0, 0.0, i),
            );
        }
        let now = ms(t, 90_000);
        let line = g.line(Channel::Steering, Line::PiWrote, now);
        assert_eq!(line.len(), 61);
        let s = Stats::of(&line).expect("points");
        assert!((s.now - 0.9).abs() < 1e-9 && (s.low - 0.3).abs() < 1e-9);
        assert!((s.high - 0.9).abs() < 1e-9 && (s.avg - 0.6).abs() < 1e-9);
        assert_eq!(Stats::of(&[]), None);
    }

    #[test]
    fn the_brake_is_a_demand_only_past_the_threshold() {
        assert_eq!(Channel::Brake.of_frame(0.0, 0.0, -0.3), 0.3);
        assert_eq!(Channel::Brake.of_frame(0.0, 0.0, -0.04), 0.0);
        assert_eq!(Channel::Brake.of_frame(0.0, 0.0, 0.5), 0.0);
        assert_eq!(Channel::Throttle.of_frame(0.0, 0.2, 0.0), 0.2);
    }

    #[test]
    fn freeze_keeps_a_copy_and_an_estop_freezes() {
        let t = Instant::now();
        let mut g = Graphs::default();
        g.record_imu_yaw(t, 5.0);
        let mut f = Freezer::default();
        f.toggle(t, &g);
        g.record_imu_yaw(ms(t, 100), 6.0);
        let (view, at) = f.view(&g, ms(t, 200));
        assert_eq!((view.imu_line(at).len(), at), (1, t));
        f.toggle(ms(t, 200), &g);
        assert_eq!(f.frozen(), None);
        let (view, at) = f.view(&g, ms(t, 200));
        assert_eq!(view.imu_line(at).len(), 2);
        // An e-stop's edge freezes; holding it doesn't freeze again once let go.
        f.watch_estop(ms(t, 300), true, &g);
        assert_eq!(f.frozen().map(|(_, w)| w), Some(FreezeWhy::Estop));
        f.toggle(ms(t, 400), &g);
        f.watch_estop(ms(t, 500), true, &g);
        assert_eq!(f.frozen(), None);
    }

    #[test]
    fn correlation_and_range() {
        assert!((correlation(&[(0.0, 0.0), (1.0, 2.0), (2.0, 4.0)]) - 1.0).abs() < 1e-12);
        assert!((correlation(&[(0.0, 0.0), (1.0, -2.0), (2.0, -4.0)]) + 1.0).abs() < 1e-12);
        assert_eq!(correlation(&[(0.0, 1.0), (1.0, 1.0)]), 0.0);
        assert_eq!(range(&[]), 0.0);
        assert_eq!(range(&[1.0, -1.0, 0.5]), 2.0);
    }
}

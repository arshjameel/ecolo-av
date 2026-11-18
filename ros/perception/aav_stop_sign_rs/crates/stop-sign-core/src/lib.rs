//! When to stop for a stop sign, from the detector's boxes. No ROS here. The Rust twin of
//! `aav_detector`'s `stop_sign.py` (and its `normal_label`, `is_optical`).
//!
//! A sign counts once it is close enough and its score good enough. How close comes from
//! its box's height and the camera's lens, a pinhole camera:
//!
//! ```text
//! distance = focal_px x sign_height_m / box_height_px
//!
//!     CLEAR --a close sign--> STOPPING --hold_s--> PASSING --ignore_s--> CLEAR
//!               (lock on)                  (lock off, signs ignored)
//! ```
#![warn(missing_docs, clippy::must_use_candidate)]

/// A stop sign's height, flat to flat: Ontario's and the US's standard sign (750 mm).
pub const SIGN_HEIGHT_M: f64 = 0.75;
/// The simulator's and the bench's front camera: 640x360, focal length 343 px. For a
/// camera with no CameraInfo yet.
pub const GAZEBO_FOCAL_PX: f64 = 343.0;

/// `'Stop_Sign'` (the 2024-25 model) and `'stop sign'` (the stock model) are one label.
#[must_use]
pub fn normal_label(label: &str) -> String {
    label
        .replace(['_', '-'], " ")
        .to_lowercase()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

/// An optical frame's name (REP-103: z forward, x right, y down).
#[must_use]
pub fn is_optical(frame_id: &str) -> bool {
    frame_id.ends_with("_optical") || frame_id.ends_with("_optical_frame")
}

/// How far a sign is (m, along the camera's axis) from its box's height (px).
#[must_use]
pub fn sign_distance_m(height_px: f64, focal_px: f64, sign_height_m: f64) -> f64 {
    if height_px > 0.0 {
        focal_px * sign_height_m / height_px
    } else {
        f64::INFINITY
    }
}

/// The watcher's thresholds (its parameters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopRule {
    /// m: a sign this close or closer stops the car.
    pub stop_distance_m: f64,
    /// px: a camera's lens until its CameraInfo comes.
    pub focal_px: f64,
    /// m: the sign's height.
    pub sign_height_m: f64,
    /// The least score that counts.
    pub min_score: f64,
    /// s held stopped.
    pub hold_s: f64,
    /// s of signs ignored after, to drive past it.
    pub ignore_s: f64,
}

impl Default for StopRule {
    fn default() -> Self {
        StopRule {
            stop_distance_m: 4.7,
            focal_px: GAZEBO_FOCAL_PX,
            sign_height_m: SIGN_HEIGHT_M,
            min_score: 0.5,
            hold_s: 4.0,
            ignore_s: 10.0,
        }
    }
}

impl StopRule {
    /// The box height (px) of a sign at `stop_distance_m`, with this lens (`None` or 0:
    /// the rule's `focal_px`, as the Python's `focal_px or self.focal_px`).
    #[must_use]
    pub fn min_height_px(&self, focal_px: Option<f64>) -> f64 {
        focal_px.filter(|f| *f != 0.0).unwrap_or(self.focal_px) * self.sign_height_m
            / self.stop_distance_m
    }
}

/// Where the watcher is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No sign: the lock is off.
    Clear,
    /// Stopped for a sign: the lock is on.
    Stopping,
    /// Driving past it: the lock is off, signs ignored.
    Passing,
}

impl State {
    /// The word for the log (the Python's).
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            State::Clear => "clear",
            State::Stopping => "stopping",
            State::Passing => "passing",
        }
    }
}

/// One detection: its label, score and box height (px).
#[derive(Debug, Clone, PartialEq)]
pub struct Sighting<'a> {
    /// The class's name, as the detector wrote it.
    pub label: &'a str,
    /// 0..1.
    pub score: f64,
    /// px: the box's height.
    pub height: f64,
    /// px: the box's centre column.
    pub centre: f64,
}

/// The watcher: CLEAR, STOPPING, PASSING, and since when.
#[derive(Debug, Clone, PartialEq)]
pub struct StopSignWatcher {
    /// Its thresholds.
    pub rule: StopRule,
    state: State,
    since: Option<f64>,
}

impl StopSignWatcher {
    /// CLEAR, with this rule.
    #[must_use]
    pub fn new(rule: StopRule) -> Self {
        StopSignWatcher {
            rule,
            state: State::Clear,
            since: None,
        }
    }

    /// Where it is now.
    #[must_use]
    pub fn state(&self) -> State {
        self.state
    }

    /// Whether any box is a stop sign close enough. `focal_px`: the lens of the camera
    /// that saw them (its CameraInfo's fy); `None`: the rule's.
    #[must_use]
    pub fn close_sign(&self, boxes: &[Sighting<'_>], focal_px: Option<f64>) -> bool {
        let least = self.rule.min_height_px(focal_px);
        boxes.iter().any(|b| {
            normal_label(b.label) == "stop sign"
                && b.score >= self.rule.min_score
                && b.height >= least
        })
    }

    /// Take the newest boxes (and their camera's `focal_px`) at `now` (s); `true` while
    /// stopped.
    pub fn update(&mut self, boxes: &[Sighting<'_>], now: f64, focal_px: Option<f64>) -> bool {
        let since = self.since.unwrap_or(now);
        match self.state {
            State::Clear if self.close_sign(boxes, focal_px) => {
                self.state = State::Stopping;
                self.since = Some(now);
            }
            State::Stopping if now - since >= self.rule.hold_s => {
                self.state = State::Passing;
                self.since = Some(now);
            }
            State::Passing if now - since >= self.rule.ignore_s => {
                self.state = State::Clear;
                self.since = None;
            }
            _ => {}
        }
        self.state == State::Stopping
    }

    /// Advance the timers without new boxes (the detector can go quiet).
    pub fn tick(&mut self, now: f64) -> bool {
        self.update(&[], now, None)
    }
}

/// Where a sign's box puts the sign, (x, y, z) in its camera's frame (m).
///
/// The range along the axis is [`sign_distance_m`]; the bearing comes from the box's
/// centre column and the lens (`lens`: fx, cx, the CameraInfo's K[0] and K[2]); with no
/// lens yet, straight ahead. `optical`: z forward, x right; else x forward, y left. The
/// height isn't known: 0.
#[must_use]
pub fn sign_point(
    centre_px: f64,
    height_px: f64,
    focal_px: f64,
    lens: Option<(f64, f64)>,
    optical: bool,
    sign_height_m: f64,
) -> [f64; 3] {
    let ahead = sign_distance_m(height_px, focal_px, sign_height_m);
    let right = match lens {
        Some((fx, cx)) if fx != 0.0 => ahead * (centre_px - cx) / fx,
        _ => 0.0,
    };
    if optical {
        [right, 0.0, ahead]
    } else {
        [ahead, -right, 0.0]
    }
}

/// (centre px, height px) of the tallest stop-sign box with at least `min_score`.
#[must_use]
pub fn biggest_sign(boxes: &[Sighting<'_>], min_score: f64) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for b in boxes {
        if normal_label(b.label) == "stop sign"
            && b.score >= min_score
            && b.height > 0.0
            && best.is_none_or(|(_, h)| b.height > h)
        {
            best = Some((b.centre, b.height));
        }
    }
    best
}

/// The lidar's place for the sign the camera sees (TODO 192 step 2), or `None`.
///
/// `camera_xy`: where the box's height puts it; `things`: (kind, x, y) from the things
/// node with names on, in the same frame. The thing named stop sign nearest the camera's
/// guess, within `gate_m`, wins (the first of equals, as the Python).
#[must_use]
pub fn lidar_sign(
    camera_xy: (f64, f64),
    things: &[(String, f64, f64)],
    gate_m: f64,
) -> Option<(f64, f64)> {
    let mut best = None;
    let mut best_d = gate_m;
    for (kind, x, y) in things {
        if normal_label(kind) != "stop sign" {
            continue;
        }
        let d = ((x - camera_xy.0).powi(2) + (y - camera_xy.1).powi(2)).sqrt();
        if d <= best_d {
            best = Some((*x, *y));
            best_d = d;
        }
    }
    best
}

/// Whether a camera frame faces forward: it names a forward word and no backward one.
/// The rear camera's signs face the other way, and must never slow the car.
#[must_use]
pub fn facing_forward(frame: &str, forward: &[String], backward: &[String]) -> bool {
    !backward.iter().any(|w| frame.contains(w.as_str()))
        && forward.iter().any(|w| frame.contains(w.as_str()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sign(height: f64) -> Sighting<'static> {
        Sighting {
            label: "stop sign",
            score: 0.9,
            height,
            centre: 320.0,
        }
    }

    #[test]
    fn holds_then_lets_go_and_ignores_the_same_sign() {
        let rule = StopRule {
            stop_distance_m: 3.5,
            ..StopRule::default()
        };
        let mut w = StopSignWatcher::new(rule);
        assert!(!w.update(&[sign(30.0)], 0.0, None));
        assert!(w.update(&[sign(100.0)], 0.0, None));
        assert!(w.update(&[sign(100.0)], 3.9, None));
        assert!(!w.update(&[sign(100.0)], 4.0, None));
        assert_eq!(w.state(), State::Passing);
        assert!(!w.update(&[sign(100.0)], 13.9, None));
        assert!(!w.tick(14.0));
        assert_eq!(w.state(), State::Clear);
    }

    #[test]
    fn labels_and_frames() {
        assert_eq!(normal_label("Stop_Sign"), "stop sign");
        assert_eq!(normal_label("  stop--  SIGN "), "stop sign");
        assert!(is_optical("camera_front_left_optical_frame"));
        let words = |w: &[&str]| w.iter().map(|s| (*s).to_string()).collect::<Vec<_>>();
        let (fwd, back) = (words(&["front", "left", "right"]), words(&["rear"]));
        assert!(facing_forward("camera_front_left_link", &fwd, &back));
        assert!(!facing_forward("camera_rear_left_link", &fwd, &back));
    }

    #[test]
    fn the_default_is_55_px_with_the_sims_lens() {
        assert!((StopRule::default().min_height_px(None) - 54.73).abs() < 0.01);
        assert_eq!(sign_distance_m(0.0, 343.0, 0.75), f64::INFINITY);
    }
}

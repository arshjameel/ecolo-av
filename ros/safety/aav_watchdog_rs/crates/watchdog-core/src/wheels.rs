//! The wheels against the IMU and the GPS: do they tell the same story?
//! (`aav_safety/wheel_check.py`.)
//!
//! A wheel-speed driver stuck in a loop sends its last speed again with fresh stamps: at a
//! steady speed that looks like a car at a steady speed. The IMU sees it as soon as the
//! speed or turn changes, the GPS once the speed is wrong for a while.
//!
//! | Gap | Over |
//! |---|---|
//! | speed | `window` s: the wheels' change in speed against the IMU's forward acceleration, added up |
//! | turn | the same window: the wheels' mean turn rate against the gyro's |
//! | gps | `gps_window` s: the speed of a line fitted to the GPS fixes against one fitted to where the wheels (and the gyro) say the car went |
//!
//! Any gap over its tolerance for `persist` s: DISAGREE, latched until [`WheelCheck::clear`].
//! Units: m/s, m/s², rad/s, seconds on the node's clock (a GPS fix: its own stamp).

use std::collections::VecDeque;
use std::f64::consts::PI;
use std::fmt;

use ego_core::fmt_g;

use crate::{hypot, py_max, py_min, py_sum};

/// m: the Earth's radius.
pub const EARTH_RADIUS: f64 = 6_371_000.0;
/// m/s between two fixes, + JUMP_MARGIN m: a jump, not driving.
pub const JUMP_SPEED: f64 = 15.0;
/// m (as the watchdog's own GPS jump check).
pub const JUMP_MARGIN: f64 = 5.0;
/// s without wheels or IMU: a hole, not to be added up across.
pub const GAP: f64 = 0.3;

/// Python's `math.radians`.
pub(crate) fn radians(degrees: f64) -> f64 {
    degrees * (PI / 180.0)
}

/// How hard to look. Tuned in Gazebo only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrossCheck {
    /// s of samples compared, wheels against the IMU.
    pub window: f64,
    /// m/s: the wheels' change against the IMU's, over the window.
    pub speed_tolerance: f64,
    /// rad/s: mean turn rates, over the window.
    pub yaw_tolerance: f64,
    /// s a gap must last before it counts.
    pub persist: f64,
    /// s: how fast the IMU's forward offset is learnt.
    pub bias_time: f64,
    /// s of GPS fixes fitted.
    pub gps_window: f64,
    /// m/s: the wheels' fitted speed against the GPS's.
    pub gps_tolerance: f64,
}

impl Default for CrossCheck {
    fn default() -> Self {
        CrossCheck {
            window: 1.5,
            speed_tolerance: 0.3,
            yaw_tolerance: 0.15,
            persist: 1.0,
            bias_time: 10.0,
            gps_window: 10.0,
            gps_tolerance: 0.15,
        }
    }
}

/// A cross check's number out of range.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CrossCheckError(pub String);

impl fmt::Display for CrossCheckError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for CrossCheckError {}

impl CrossCheck {
    /// These numbers, checked.
    ///
    /// # Errors
    ///
    /// [`CrossCheckError`] for a window, tolerance or bias time not above 0, or a negative
    /// persist.
    pub fn checked(self) -> Result<Self, CrossCheckError> {
        for (name, v) in [
            ("window", self.window),
            ("speed_tolerance", self.speed_tolerance),
            ("yaw_tolerance", self.yaw_tolerance),
            ("bias_time", self.bias_time),
            ("gps_window", self.gps_window),
            ("gps_tolerance", self.gps_tolerance),
        ] {
            if v.is_nan() || v <= 0.0 {
                return Err(CrossCheckError(format!(
                    "cross check: {name} must be positive"
                )));
            }
        }
        if self.persist < 0.0 {
            return Err(CrossCheckError(
                "cross check: persist must not be negative".into(),
            ));
        }
        Ok(self)
    }
}

/// The integral over [start, end] of (t, value) samples, sorted by t.
#[must_use]
pub fn trapezoid(samples: &VecDeque<(f64, f64)>, start: f64, end: f64) -> f64 {
    let mut total = 0.0;
    for ((t0, a0), (t1, a1)) in samples.iter().zip(samples.iter().skip(1)) {
        let (lo, hi) = (py_max(*t0, start), py_min(*t1, end));
        if hi <= lo {
            continue;
        }
        let at = |t: f64| a0 + (a1 - a0) * (t - t0) / (t1 - t0);
        total += 0.5 * (at(lo) + at(hi)) * (hi - lo);
    }
    total
}

/// The speed (m/s) of the straight line fitted to (t, x, y) points.
#[must_use]
pub fn fitted_speed(points: &[(f64, f64, f64)]) -> f64 {
    let n = points.len() as f64;
    let tm = py_sum(points.iter().map(|p| p.0)) / n;
    let xm = py_sum(points.iter().map(|p| p.1)) / n;
    let ym = py_sum(points.iter().map(|p| p.2)) / n;
    let tt = py_sum(points.iter().map(|p| (p.0 - tm) * (p.0 - tm)));
    if tt <= 0.0 {
        return 0.0;
    }
    let vx = py_sum(points.iter().map(|p| (p.0 - tm) * (p.1 - xm))) / tt;
    let vy = py_sum(points.iter().map(|p| (p.0 - tm) * (p.2 - ym))) / tt;
    hypot(vx, vy)
}

/// The wheels' speed and turn rate against the IMU's and the GPS's, over sliding windows.
#[derive(Debug, Clone)]
pub struct WheelCheck {
    check: CrossCheck,
    /// (t, speed, yaw rate).
    wheel: VecDeque<(f64, f64, f64)>,
    /// (t, forward acceleration).
    accel: VecDeque<(f64, f64)>,
    /// (t, yaw rate).
    gyro: VecDeque<(f64, f64)>,
    /// (stamp, east m, north m), from the first fix.
    fixes: VecDeque<(f64, f64, f64)>,
    origin: Option<(f64, f64)>,
    last_fix: Option<(f64, f64)>,
    /// (t, x, y): where the wheels and the gyro say it went.
    track: VecDeque<(f64, f64, f64)>,
    pose: (f64, f64, f64),
    /// m/s²: what the IMU's forward reading adds (slopes).
    bias: Option<f64>,
    since: Option<f64>,
    latched: Option<String>,
    last: Option<f64>,
    /// The biggest gaps seen before any latch, for tuning: speed, turn, GPS.
    pub worst: (f64, f64, f64),
    /// The newest (speed, turn, GPS) gaps; `None`: not enough.
    pub gaps: Option<(f64, f64, f64)>,
}

impl WheelCheck {
    /// Nothing seen yet.
    #[must_use]
    pub fn new(check: CrossCheck) -> Self {
        WheelCheck {
            check,
            wheel: VecDeque::new(),
            accel: VecDeque::new(),
            gyro: VecDeque::new(),
            fixes: VecDeque::new(),
            origin: None,
            last_fix: None,
            track: VecDeque::new(),
            pose: (0.0, 0.0, 0.0),
            bias: None,
            since: None,
            latched: None,
            last: None,
            worst: (0.0, 0.0, 0.0),
            gaps: None,
        }
    }

    /// Its settings.
    #[must_use]
    pub fn check(&self) -> &CrossCheck {
        &self.check
    }

    /// A wheel sample: forward speed (m/s) and turn rate (rad/s).
    pub fn wheels(&mut self, now: f64, speed: f64, yaw_rate: f64) {
        if !(speed.is_finite() && yaw_rate.is_finite()) {
            return;
        }
        if let Some(last) = self.wheel.back() {
            if now - last.0 > GAP {
                // The wheels were quiet (STALE): start afresh, not across the hole.
                self.wheel.clear();
                self.track.clear();
            }
        }
        if let Some(last) = self.wheel.back() {
            let dt = now - last.0;
            if dt > 0.0 {
                let turn = self.gyro.back().map_or(yaw_rate, |g| g.1);
                let (x, y, mut h) = self.pose;
                h += turn * dt;
                self.pose = (x + speed * h.cos() * dt, y + speed * h.sin() * dt, h);
            }
        }
        self.wheel.push_back((now, speed, yaw_rate));
        self.track.push_back((now, self.pose.0, self.pose.1));
        while self
            .track
            .front()
            .is_some_and(|p| p.0 < now - self.check.gps_window - 2.0)
        {
            self.track.pop_front();
        }
    }

    /// An IMU sample: forward acceleration (m/s²) and yaw rate (rad/s). One the same as the
    /// last, to the bit, is a frozen IMU: left out.
    pub fn imu(&mut self, now: f64, forward: f64, yaw_rate: f64) {
        if !(forward.is_finite() && yaw_rate.is_finite()) {
            return;
        }
        if let (Some(a), Some(g)) = (self.accel.back(), self.gyro.back()) {
            if (forward, yaw_rate) == (a.1, g.1) {
                return;
            }
        }
        if self.accel.back().is_some_and(|a| now - a.0 > GAP) {
            self.accel.clear(); // quiet or frozen a while: not across the hole
            self.gyro.clear();
        }
        self.accel.push_back((now, forward));
        self.gyro.push_back((now, yaw_rate));
    }

    /// A good GPS fix: its stamp (s), and where (degrees).
    pub fn gps(&mut self, stamp: f64, latitude: f64, longitude: f64) {
        if !(latitude.is_finite() && longitude.is_finite()) {
            return;
        }
        if self.fixes.back().is_some_and(|f| stamp <= f.0) {
            return; // an old fix, late
        }
        if self.last_fix == Some((latitude, longitude)) {
            return; // frozen: the GPS's own business
        }
        self.last_fix = Some((latitude, longitude));
        let (lat0, lon0) = *self.origin.get_or_insert((latitude, longitude));
        let east = radians(longitude - lon0) * EARTH_RADIUS * radians(lat0).cos();
        let north = radians(latitude - lat0) * EARTH_RADIUS;
        self.fixes.push_back((stamp, east, north));
    }

    /// (speed gap m/s, turn gap rad/s, the IMU's own mean forward reading m/s²); `None`
    /// while the window doesn't hold enough of both.
    fn measure(&mut self, now: f64) -> Option<(f64, f64, f64)> {
        let start = now - self.check.window;
        while self.wheel.len() > 2 && self.wheel[1].0 <= start {
            self.wheel.pop_front();
        }
        while self.accel.len() > 2 && self.accel[1].0 <= start {
            self.accel.pop_front();
        }
        while self.gyro.len() > 2 && self.gyro[1].0 <= start {
            self.gyro.pop_front();
        }
        let wheel: Vec<(f64, f64, f64)> = self
            .wheel
            .iter()
            .filter(|w| w.0 >= start)
            .copied()
            .collect();
        if wheel.len() < 2 || self.accel.len() < 2 || self.gyro.is_empty() {
            return None;
        }
        let (first, last) = (wheel[0], wheel[wheel.len() - 1]);
        let (t0, t1) = (first.0, last.0);
        let span = t1 - t0;
        if span < 0.6 * self.check.window
            || self.accel[0].0 > t0 + 0.1
            || self.accel[self.accel.len() - 1].0 < t1 - 0.1
            || self.gyro[self.gyro.len() - 1].0 < t1 - 0.1
        {
            return None;
        }
        let felt = trapezoid(&self.accel, t0, t1);
        let change = last.1 - first.1;
        let bias = *self.bias.get_or_insert_with(|| {
            // The first look: a car standing still shows the IMU's own offset.
            if wheel.iter().all(|w| w.1.abs() < 0.01) {
                (felt - change) / span
            } else {
                0.0
            }
        });
        let speed_gap = (change - (felt - bias * span)).abs();
        let mut gyro: Vec<f64> = self
            .gyro
            .iter()
            .filter(|g| t0 <= g.0 && g.0 <= t1)
            .map(|g| g.1)
            .collect();
        if gyro.is_empty() {
            gyro.push(self.gyro[self.gyro.len() - 1].1);
        }
        let turn_gap = (py_sum(wheel.iter().map(|w| w.2)) / wheel.len() as f64
            - py_sum(gyro.iter().copied()) / gyro.len() as f64)
            .abs();
        Some((speed_gap, turn_gap, (felt - change) / span))
    }

    /// (gap, the wheels' fitted speed, the GPS's) over the GPS window, all m/s; `None`
    /// while there aren't enough fixes, or a jump lies between two of them.
    fn gps_gap(&mut self) -> Option<(f64, f64, f64)> {
        let start = self.fixes.back()?.0 - self.check.gps_window;
        while self.fixes.front().is_some_and(|f| f.0 < start) {
            self.fixes.pop_front();
        }
        while self.track.len() > 2 && self.track[1].0 <= start {
            self.track.pop_front();
        }
        let (track_first, track_last) = (*self.track.front()?, *self.track.back()?);
        // Only the time both cover: the wheels may have started afresh after a hole.
        let (fix_first, fix_last) = (*self.fixes.front()?, *self.fixes.back()?);
        let lo = py_max(fix_first.0, track_first.0);
        let hi = py_min(fix_last.0, track_last.0);
        let fixes: Vec<_> = self
            .fixes
            .iter()
            .filter(|f| lo <= f.0 && f.0 <= hi)
            .copied()
            .collect();
        let track: Vec<_> = self
            .track
            .iter()
            .filter(|p| lo <= p.0 && p.0 <= hi)
            .copied()
            .collect();
        if fixes.len() < 8 || track.len() < 8 || hi - lo < 0.8 * self.check.gps_window {
            return None;
        }
        let jumped = fixes.windows(2).any(|w| {
            let ((t0, x0, y0), (t1, x1, y1)) = (w[0], w[1]);
            hypot(x1 - x0, y1 - y0) > JUMP_SPEED * (t1 - t0) + JUMP_MARGIN
        });
        if jumped {
            return None;
        }
        let (wheels, gps) = (fitted_speed(&track), fitted_speed(&fixes));
        Some(((wheels - gps).abs(), wheels, gps))
    }

    /// Look again (each tick): why the wheels are wrong, or `None` if they're not.
    pub fn update(&mut self, now: f64) -> Option<String> {
        let dt = self.last.map_or(0.0, |last| py_max(0.0, now - last));
        self.last = Some(now);
        if self.latched.is_some() {
            return self.latched.clone();
        }
        let got = self.measure(now);
        let by_gps = self.gps_gap();
        if got.is_none() && by_gps.is_none() {
            self.gaps = None;
            self.since = None;
            return None;
        }
        let (speed_gap, turn_gap, offset) = match got {
            Some((s, t, o)) => (s, t, Some(o)),
            None => (0.0, 0.0, None),
        };
        let gps_gap = by_gps.map_or(0.0, |g| g.0);
        self.gaps = Some((speed_gap, turn_gap, gps_gap));
        self.worst = (
            py_max(self.worst.0, speed_gap),
            py_max(self.worst.1, turn_gap),
            py_max(self.worst.2, gps_gap),
        );
        let c = self.check;
        let imu_wrong = speed_gap > c.speed_tolerance || turn_gap > c.yaw_tolerance;
        let gps_wrong = gps_gap > c.gps_tolerance;
        if imu_wrong || gps_wrong {
            let since = *self.since.get_or_insert(now);
            if now - since >= c.persist {
                self.latched = Some(if imu_wrong {
                    format!(
                        "the wheels and the IMU disagree: speed {speed_gap:.2} m/s apart over \
                         {} s, turn {turn_gap:.2} rad/s",
                        fmt_g(c.window)
                    )
                } else {
                    let (_, w, g) = by_gps.unwrap_or((0.0, 0.0, 0.0));
                    format!(
                        "the wheels and the GPS disagree: the wheels {w:.2} m/s, the GPS \
                         {g:.2} m/s over {} s",
                        fmt_g(c.gps_window)
                    )
                });
            }
            return self.latched.clone();
        }
        self.since = None;
        if let (Some(offset), Some(bias)) = (offset, self.bias.as_mut()) {
            *bias += (offset - *bias) * py_min(1.0, dt / c.bias_time);
        }
        None
    }

    /// Forget a latched disagreement, and the samples from before it: start afresh.
    pub fn clear(&mut self) {
        self.latched = None;
        self.since = None;
        self.gaps = None;
        self.wheel.clear();
        self.accel.clear();
        self.gyro.clear();
        self.fixes.clear();
        self.track.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive a car at 0.5 m/s² from 1 s, wheels and IMU at 50 Hz (`test_wheel_check.py`'s
    /// `drive`), the wheels frozen from `frozen_at`; when it first latched, or `None`.
    fn drive(w: &mut WheelCheck, end: f64, frozen_at: Option<f64>) -> Option<f64> {
        let ramp = |t: f64| (0.5 * (t - 1.0)).clamp(0.0, 1.0);
        let (mut t, mut held) = (0.0, None);
        while t <= end {
            let accel = (ramp(t + 1e-3) - ramp(t - 1e-3)) / 2e-3;
            if frozen_at.is_some_and(|f| t >= f) && held.is_none() {
                held = Some(ramp(t));
            }
            w.wheels(t, held.unwrap_or(ramp(t)), 0.0);
            // A real IMU's noise never repeats: a noiseless one would count as frozen.
            w.imu(
                t,
                accel + 0.01 * (t * 977.0).sin(),
                0.001 * (t * 613.0).cos(),
            );
            if w.update(t).is_some() {
                return Some(t);
            }
            t += 0.02;
        }
        None
    }

    #[test]
    fn normal_driving_never_latches() {
        let mut w = WheelCheck::new(CrossCheck::default());
        assert_eq!(drive(&mut w, 30.0, None), None);
        assert!(w.worst.0 < 0.15 && w.worst.1 < 0.05);
    }

    #[test]
    fn wheels_frozen_while_speeding_up_latch_within_two_seconds() {
        let mut w = WheelCheck::new(CrossCheck::default());
        let t = drive(&mut w, 10.0, Some(1.5)).expect("latched");
        assert!(t - 1.5 < 2.5);
        let why = w.update(t + 5.0).expect("still latched");
        assert!(why.contains("disagree"), "{why}");
        w.clear();
        assert_eq!(w.update(t + 5.0), None);
    }

    #[test]
    fn a_bad_setting_is_refused() {
        let bad = CrossCheck {
            window: 0.0,
            ..CrossCheck::default()
        };
        assert!(bad.checked().is_err());
        assert!(CrossCheck::default().checked().is_ok());
    }

    #[test]
    fn a_fitted_line_gives_its_speed() {
        let pts: Vec<_> = (0..10)
            .map(|i| (f64::from(i), 3.0 * f64::from(i), 4.0 * f64::from(i)))
            .collect();
        assert!((fitted_speed(&pts) - 5.0).abs() < 1e-12);
        assert_eq!(fitted_speed(&[(1.0, 0.0, 0.0)]), 0.0);
    }
}

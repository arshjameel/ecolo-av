//! The stop zone: its box in front of (or behind) the car, and what it is doing.

use std::time::{Duration, Instant};

use map_core::geom::{Point, Pose};

use crate::marker::Frame;

/// The box and the state count as silent after this long.
pub const SILENT_AFTER: Duration = Duration::from_secs(2);

/// What the stop zone does (`nav2_msgs/CollisionMonitorState.action_type`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ZoneAction {
    /// Nothing in the box.
    #[default]
    Clear,
    Stop,
    Slow,
    Approach,
    Limit,
}

impl ZoneAction {
    /// From `action_type`: 0 nothing, 1 stop, 2 slow down, 3 approach, 4 limit.
    #[must_use]
    pub fn from_code(code: u8) -> Self {
        match code {
            1 => Self::Stop,
            2 => Self::Slow,
            3 => Self::Approach,
            4 => Self::Limit,
            _ => Self::Clear,
        }
    }

    /// The word on the screen.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Clear => "CLEAR",
            Self::Stop => "STOP",
            Self::Slow => "SLOW",
            Self::Approach => "APPROACH",
            Self::Limit => "LIMIT",
        }
    }

    /// Whether the car is being held or slowed by it.
    #[must_use]
    pub fn acts(self) -> bool {
        self != Self::Clear
    }
}

/// The stop zone as the screen knows it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StopZone {
    frame: Option<Frame>,
    corners: Vec<Point>,
    box_heard: Option<Instant>,
    action: ZoneAction,
    action_heard: Option<Instant>,
}

impl StopZone {
    /// The box in use (`/safety/stop_zone/box`): its frame and corners.
    pub fn set_box(&mut self, frame_id: &str, corners: &[(f64, f64)], now: Instant) {
        self.frame = Frame::from_id(frame_id);
        self.corners = corners.iter().map(|&(x, y)| Point::new(x, y)).collect();
        self.box_heard = Some(now);
    }

    /// What it does now (`/safety/stop_zone/state`, sent on each change).
    pub fn set_action(&mut self, action: ZoneAction, now: Instant) {
        self.action = action;
        self.action_heard = Some(now);
    }

    /// What it does: the last state heard; `None` before any.
    #[must_use]
    pub fn action(&self) -> Option<ZoneAction> {
        self.action_heard.map(|_| self.action)
    }

    /// The box on the map, closed (the first corner again at the end); empty when the
    /// box is silent, too small, or in a frame the view can't place.
    #[must_use]
    pub fn outline(&self, car_odom: Pose, car_map: Pose, now: Instant) -> Vec<Point> {
        let fresh = self
            .box_heard
            .is_some_and(|heard| now.saturating_duration_since(heard) <= SILENT_AFTER);
        let (true, Some(frame), true) = (fresh, self.frame, self.corners.len() >= 3) else {
            return Vec::new();
        };
        self.corners
            .iter()
            .chain(self.corners.first())
            .map(|p| frame.place(*p, car_odom, car_map))
            .collect()
    }

    /// The words beside the box: "STOP ZONE: STOP".
    #[must_use]
    pub fn words(&self) -> String {
        match self.action() {
            Some(action) => format!("STOP ZONE: {}", action.word()),
            None => "STOP ZONE".to_owned(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOX: [(f64, f64); 4] = [(1.8, -0.7), (4.8, -0.7), (4.8, 0.7), (1.8, 0.7)];

    #[test]
    fn the_box_rides_with_the_car_and_closes() {
        let now = Instant::now();
        let mut zone = StopZone::default();
        zone.set_box("base_link", &BOX, now);
        let car = Pose::new(10.0, 5.0, std::f64::consts::PI);
        let line = zone.outline(Pose::default(), car, now);
        assert_eq!(line.len(), 5);
        assert_eq!(line[0], line[4]);
        // The car faces west: the box's far edge is 4.8 m to its west.
        assert!((line[1].x - 5.2).abs() < 1e-9 && (line[1].y - 5.7).abs() < 1e-9);
    }

    #[test]
    fn a_silent_box_is_not_drawn() {
        let now = Instant::now();
        let mut zone = StopZone::default();
        assert!(zone
            .outline(Pose::default(), Pose::default(), now)
            .is_empty());
        zone.set_box("base_link", &BOX, now);
        let later = now + SILENT_AFTER + Duration::from_millis(1);
        assert!(zone
            .outline(Pose::default(), Pose::default(), later)
            .is_empty());
    }

    #[test]
    fn a_box_in_a_frame_we_cannot_place_is_not_drawn() {
        let now = Instant::now();
        let mut zone = StopZone::default();
        zone.set_box("radar_link", &BOX, now);
        assert!(zone
            .outline(Pose::default(), Pose::default(), now)
            .is_empty());
    }

    #[test]
    fn the_state_is_the_last_one_heard_and_stays() {
        let now = Instant::now();
        let mut zone = StopZone::default();
        assert_eq!(zone.words(), "STOP ZONE");
        zone.set_action(ZoneAction::from_code(1), now);
        assert_eq!(zone.words(), "STOP ZONE: STOP");
        assert!(zone.action().is_some_and(ZoneAction::acts));
        zone.set_action(ZoneAction::from_code(0), now);
        assert_eq!(zone.words(), "STOP ZONE: CLEAR");
        assert_eq!(ZoneAction::from_code(2).word(), "SLOW");
    }
}

//! The trip's status, in one word for the panel, and the time left.
//!
//! Worked out from what the map window sees: the goal (the plan's last point, or RViz's
//! `/goal_pose`), the car, and whether Nav2 still sends plans. Nav2's own action
//! feedback isn't needed for this.

use std::time::Duration;

use hud_core::health::Health;

use crate::geom::{Metres, Point};

/// Where the trip is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trip {
    /// No plan and no goal: nothing to drive to.
    NoGoal,
    /// A goal, but no position for the car.
    NoPosition,
    /// A goal from RViz, and no plan for it yet.
    WaitingForPlan,
    /// A fresh plan, and the goal still ahead.
    Driving,
    /// Within `arrive_within` of the goal.
    Arrived,
    /// The goal still ahead, but Nav2 stopped planning: cancelled, failed, or gone.
    PlanStopped,
}

/// How the panel colours a status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    /// Nothing to flag.
    Quiet,
    /// All is well.
    Good,
    /// Worth a look.
    Warn,
    /// Something is wrong.
    Bad,
}

impl Trip {
    /// Works the status out.
    #[must_use]
    pub fn of(
        goal: Option<Point>,
        car: Option<Point>,
        plan: Health,
        arrive_within: Metres,
    ) -> Self {
        let Some(goal) = goal else {
            return Self::NoGoal;
        };
        let Some(car) = car else {
            return Self::NoPosition;
        };
        if car.distance(goal) <= arrive_within {
            return Self::Arrived;
        }
        match plan {
            Health::Never => Self::WaitingForPlan,
            Health::Stale(_) => Self::PlanStopped,
            Health::Ok(_) => Self::Driving,
        }
    }

    /// The words on the screen.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::NoGoal => "NO GOAL",
            Self::NoPosition => "NO POSITION",
            Self::WaitingForPlan => "WAITING FOR A PLAN",
            Self::Driving => "DRIVING",
            Self::Arrived => "ARRIVED",
            Self::PlanStopped => "NO NEW PLAN",
        }
    }

    /// One line under the word, saying what it means.
    #[must_use]
    pub fn hint(self) -> &'static str {
        match self {
            Self::NoGoal => "Send a goal from RViz or Foxglove",
            Self::NoPosition => "Nothing on the car's position topic",
            Self::WaitingForPlan => "Nav2 hasn't planned yet",
            Self::Driving => "Nav2 is driving to the goal",
            Self::Arrived => "At the goal",
            Self::PlanStopped => "Nav2 stopped: cancelled or failed?",
        }
    }

    /// Its colour on the panel.
    #[must_use]
    pub fn tone(self) -> Tone {
        match self {
            Self::NoGoal => Tone::Quiet,
            Self::Driving | Self::Arrived => Tone::Good,
            Self::WaitingForPlan | Self::PlanStopped => Tone::Warn,
            Self::NoPosition => Tone::Bad,
        }
    }
}

/// Below this speed, no time left is shown: the car is (nearly) stopped.
const ETA_MIN_SPEED: f64 = 0.2;

/// The time left at the current speed; None when stopped or going backwards.
#[must_use]
pub fn time_left(remaining: Metres, speed_m_per_s: f64) -> Option<Duration> {
    (speed_m_per_s.is_finite() && speed_m_per_s >= ETA_MIN_SPEED && remaining.0.is_finite())
        .then(|| Duration::from_secs_f64(remaining.0.max(0.0) / speed_m_per_s))
}

#[cfg(test)]
mod tests {
    use super::*;

    const NEAR: Metres = Metres(1.0);
    const FRESH: Health = Health::Ok(Duration::ZERO);

    #[test]
    fn each_status() {
        let goal = Some(Point::new(10.0, 0.0));
        let car = Some(Point::new(0.0, 0.0));
        let old = Health::Stale(Duration::from_secs(9));
        assert_eq!(Trip::of(None, car, FRESH, NEAR), Trip::NoGoal);
        assert_eq!(Trip::of(goal, None, FRESH, NEAR), Trip::NoPosition);
        assert_eq!(Trip::of(goal, car, FRESH, NEAR), Trip::Driving);
        assert_eq!(Trip::of(goal, car, old, NEAR), Trip::PlanStopped);
        assert_eq!(
            Trip::of(goal, car, Health::Never, NEAR),
            Trip::WaitingForPlan
        );
        let there = Some(Point::new(9.5, 0.5));
        assert_eq!(Trip::of(goal, there, old, NEAR), Trip::Arrived);
        assert_eq!(Trip::Arrived.tone(), Tone::Good);
        assert_eq!(Trip::PlanStopped.label(), "NO NEW PLAN");
    }

    #[test]
    fn time_left_at_this_speed() {
        assert_eq!(time_left(Metres(30.0), 3.0), Some(Duration::from_secs(10)));
        assert_eq!(time_left(Metres(30.0), 0.1), None);
        assert_eq!(time_left(Metres(30.0), -2.0), None);
        assert_eq!(time_left(Metres(30.0), f64::NAN), None);
    }
}

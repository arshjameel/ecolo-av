//! One drive to B, tick by tick: the speed ([`crate::approach`]) and the steering
//! ([`crate::line`]) together (`drive.py`). It returns a speed and a curvature; the node
//! publishes them as a TwistStamped (angular.z = v k).

use crate::approach::{Approach, Phase};
use crate::hypot;
use crate::line::{bearing_error, pursuit_curvature, turn_speed_limit, Line, Point, Pose};
use crate::py_max;

/// How to steer. Distances m, angles rad, times s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Steering {
    /// 1/m: full lock, tan(max_steering) / wheelbase.
    pub max_curvature: f64,
    /// About the wheelbase or more, or a car weaves.
    pub lookahead_min: f64,
    /// s of driving: the lookahead grows with speed.
    pub lookahead_time: f64,
    /// B further off the nose than this: turn towards it first.
    pub turn_above: f64,
    /// ... until it is this close, then the line.
    pub aligned: f64,
    /// m/s²: slow down in tight turns.
    pub lateral_accel: f64,
}

impl Steering {
    /// The Python's defaults, with this full lock.
    #[must_use]
    pub fn new(max_curvature: f64) -> Self {
        Steering {
            max_curvature,
            lookahead_min: 2.0,
            lookahead_time: 1.0,
            turn_above: 20.0_f64.to_radians(),
            aligned: 10.0_f64.to_radians(),
            lateral_accel: 1.5,
        }
    }
}

/// One tick's command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// m/s.
    pub speed: f64,
    /// 1/m, + left.
    pub curvature: f64,
    /// Where in the drive it is.
    pub phase: Phase,
    /// m to B: along the line, or straight while turning.
    pub left: f64,
    /// m left (+) of the line; 0 while turning.
    pub cross: f64,
}

/// Drive from where the car is to `target`, and stop there.
#[derive(Debug, Clone, PartialEq)]
pub struct Drive {
    target: Point,
    approach: Approach,
    steering: Steering,
    line: Option<Line>,
    speed: f64,
}

impl Drive {
    /// A drive from `start`: on the line at once if B is within `turn_above` of the nose.
    #[must_use]
    pub fn new(target: Point, approach: Approach, steering: Steering, start: &Pose) -> Self {
        let line = (bearing_error(start, target).abs() <= steering.turn_above).then_some(Line {
            a: (start.x, start.y),
            b: target,
        });
        Drive {
            target,
            approach,
            steering,
            line,
            speed: 0.0,
        }
    }

    /// The line it follows, once lined up.
    #[must_use]
    pub fn line(&self) -> Option<&Line> {
        self.line.as_ref()
    }

    /// The plan.
    #[must_use]
    pub fn approach(&self) -> &Approach {
        &self.approach
    }

    /// One tick: the car at `pose`, going `measured` m/s, `dt` s since the last.
    pub fn tick(&mut self, pose: &Pose, measured: f64, dt: f64) -> Command {
        let st = self.steering;
        if self.line.is_none() && bearing_error(pose, self.target).abs() <= st.aligned {
            self.line = Some(Line {
                a: (pose.x, pose.y),
                b: self.target,
            });
        }
        let lookahead = py_max(st.lookahead_min, st.lookahead_time * measured.abs());
        let (left, cross, k) = match self.line.as_ref().map(|line| {
            (
                line.progress((pose.x, pose.y)),
                line.carrot((pose.x, pose.y), lookahead),
            )
        }) {
            Some((Ok((left, cross)), Ok(carrot))) => (
                left,
                cross,
                pursuit_curvature(pose, carrot, st.max_curvature),
            ),
            // No line yet (or A is B, which the rules refuse): straight at B.
            _ => (
                hypot(self.target.0 - pose.x, self.target.1 - pose.y),
                0.0,
                pursuit_curvature(pose, self.target, st.max_curvature),
            ),
        };
        let (speed, mut phase) = self.approach.step(
            self.speed,
            dt,
            left,
            measured,
            turn_speed_limit(k, st.lateral_accel),
        );
        if self.line.is_none() && phase != Phase::Stopping {
            phase = Phase::Turn;
        }
        self.speed = speed;
        Command {
            speed,
            curvature: if speed > 0.0 { k } else { 0.0 },
            phase,
            left,
            cross,
        }
    }
}

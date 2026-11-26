//! Where the car has been: the line behind it on the map.

use std::collections::VecDeque;

use map_core::geom::Point;

/// A new point is kept once the car has moved this far, metres.
pub const STEP: f64 = 0.2;
/// The most points kept: 800 m of driving at [`STEP`].
pub const MOST: usize = 4_000;
/// A move this long between two poses is not driving: the car was placed somewhere
/// else (a new run, a reset), so the trail starts again. Metres.
pub const JUMP: f64 = 10.0;

/// The car's trail, oldest point first.
#[derive(Debug, Clone, PartialEq)]
pub struct Trail {
    points: VecDeque<Point>,
    step: f64,
    most: usize,
}

impl Default for Trail {
    fn default() -> Self {
        Self::new(STEP, MOST)
    }
}

impl Trail {
    /// A trail with a point every `step` metres, the newest `most` kept.
    #[must_use]
    pub fn new(step: f64, most: usize) -> Self {
        Self {
            points: VecDeque::new(),
            step,
            most: most.max(1),
        }
    }

    /// The car is here now. Returns whether the trail changed.
    pub fn push(&mut self, at: Point) -> bool {
        if !(at.x.is_finite() && at.y.is_finite()) {
            return false;
        }
        if let Some(last) = self.points.back() {
            let moved = (at.x - last.x).hypot(at.y - last.y);
            if moved >= JUMP {
                self.points.clear();
            } else if moved < self.step {
                return false;
            }
        }
        self.points.push_back(at);
        while self.points.len() > self.most {
            self.points.pop_front();
        }
        true
    }

    /// Forgets the trail.
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// The points, oldest first.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Point> + '_ {
        self.points.iter().copied()
    }

    /// How many points it holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether it holds nothing yet.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Its length along the ground, metres.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.points
            .iter()
            .zip(self.points.iter().skip(1))
            .map(|(a, b)| (b.x - a.x).hypot(b.y - a.y))
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_parked_car_leaves_one_point() {
        let mut trail = Trail::default();
        assert!(trail.push(Point::new(1.0, 1.0)));
        for _ in 0..50 {
            assert!(!trail.push(Point::new(1.05, 1.0)));
        }
        assert_eq!(trail.len(), 1);
    }

    #[test]
    fn a_drive_leaves_a_point_each_step() {
        let mut trail = Trail::default();
        for i in 0..=1000 {
            trail.push(Point::new(f64::from(i) * 0.01, 0.0));
        }
        assert!((49..=51).contains(&trail.len()), "{}", trail.len());
        assert!((trail.length() - 10.0).abs() < 2.0 * STEP);
    }

    #[test]
    fn only_the_newest_are_kept() {
        let mut trail = Trail::new(1.0, 5);
        for i in 0..20 {
            trail.push(Point::new(f64::from(i), 0.0));
        }
        let xs: Vec<f64> = trail.points().map(|p| p.x).collect();
        assert_eq!(xs, [15.0, 16.0, 17.0, 18.0, 19.0]);
    }

    #[test]
    fn a_jump_starts_the_trail_again() {
        let mut trail = Trail::default();
        trail.push(Point::new(0.0, 0.0));
        trail.push(Point::new(1.0, 0.0));
        assert!(trail.push(Point::new(-150.0, 0.0)));
        assert_eq!(trail.len(), 1);
    }

    #[test]
    fn a_pose_that_is_not_a_number_is_dropped() {
        let mut trail = Trail::default();
        assert!(!trail.push(Point::new(f64::NAN, 0.0)));
        assert!(trail.is_empty());
    }
}

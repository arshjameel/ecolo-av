//! Nav2's planned path (`/plan`, nav_msgs/Path): where the car is along it, and how far
//! is left to the goal.
//!
//! Nav2 plans again from the car every second while it drives, so the car is normally
//! near the path's start; the distance left is measured along the path from the point
//! nearest the car.

use crate::geom::{Metres, Point};

/// A planned path on the map: its points, start to goal.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Route {
    points: Vec<Point>,
}

/// Where the car is along a route.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Progress {
    /// The point of the path nearest the car.
    pub nearest: Point,
    /// How far the car is from the path.
    pub off_path: Metres,
    /// The length of path from `nearest` to the goal.
    pub remaining: Metres,
}

impl Route {
    /// A route through these points. Points that aren't finite numbers are dropped.
    pub fn new(points: impl IntoIterator<Item = Point>) -> Self {
        Self {
            points: points.into_iter().filter(|p| p.is_finite()).collect(),
        }
    }

    /// The route's points.
    #[must_use]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// True if it has no points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// The last point: where Nav2 is going.
    #[must_use]
    pub fn goal(&self) -> Option<Point> {
        self.points.last().copied()
    }

    /// The whole length, start to goal.
    #[must_use]
    pub fn length(&self) -> Metres {
        Metres(self.segments().map(|(a, b)| (b - a).length()).sum())
    }

    /// Each pair of neighbouring points.
    fn segments(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        self.points.windows(2).map(|w| (w[0], w[1]))
    }

    /// Where `car` is along the route; None for an empty route.
    #[must_use]
    pub fn progress(&self, car: Point) -> Option<Progress> {
        let goal = self.goal()?;
        // The nearest point on each segment, and that segment's number; keep the nearest
        // of all (the first, on a tie: the car is near the start of a fresh plan).
        let nearest = self
            .segments()
            .enumerate()
            .map(|(i, (a, b))| (i, nearest_on_segment(car, a, b)))
            .min_by(|(_, p), (_, q)| car.distance(*p).0.total_cmp(&car.distance(*q).0));
        let Some((i, nearest)) = nearest else {
            // One point: the goal itself.
            return Some(Progress {
                nearest: goal,
                off_path: car.distance(goal),
                remaining: Metres(0.0),
            });
        };
        let rest_of_segment = nearest.distance(self.points[i + 1]).0;
        let later_segments: f64 = self
            .segments()
            .skip(i + 1)
            .map(|(a, b)| (b - a).length())
            .sum();
        Some(Progress {
            nearest,
            off_path: car.distance(nearest),
            remaining: Metres(rest_of_segment + later_segments),
        })
    }
}

/// The point of segment a-b nearest p.
fn nearest_on_segment(p: Point, a: Point, b: Point) -> Point {
    let ab = b - a;
    let len2 = ab.dot(ab);
    if len2 == 0.0 {
        return a;
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    a + ab * t
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Metres, b: f64) -> bool {
        (a.0 - b).abs() < 1e-9
    }

    /// An L: 10 m east, then 5 m north.
    fn ell() -> Route {
        Route::new([
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
        ])
    }

    #[test]
    fn length_and_goal() {
        let r = ell();
        assert!(close(r.length(), 15.0));
        assert_eq!(r.goal(), Some(Point::new(10.0, 5.0)));
        assert_eq!(Route::default().goal(), None);
        assert_eq!(Route::default().progress(Point::default()), None);
    }

    #[test]
    fn at_the_start_the_whole_path_is_left() {
        let p = ell().progress(Point::new(0.0, 0.0)).expect("a route");
        assert!(close(p.remaining, 15.0));
        assert!(close(p.off_path, 0.0));
    }

    #[test]
    fn beside_the_path_counts_from_the_nearest_point() {
        // 4 m along the first leg, 1 m to the side of it.
        let p = ell().progress(Point::new(4.0, -1.0)).expect("a route");
        assert_eq!(p.nearest, Point::new(4.0, 0.0));
        assert!(close(p.off_path, 1.0));
        assert!(close(p.remaining, 6.0 + 5.0));
    }

    #[test]
    fn on_the_second_leg() {
        let p = ell().progress(Point::new(11.0, 3.0)).expect("a route");
        assert_eq!(p.nearest, Point::new(10.0, 3.0));
        assert!(close(p.remaining, 2.0));
    }

    #[test]
    fn past_the_goal_nothing_is_left() {
        let p = ell().progress(Point::new(10.0, 9.0)).expect("a route");
        assert_eq!(p.nearest, Point::new(10.0, 5.0));
        assert!(close(p.remaining, 0.0));
        assert!(close(p.off_path, 4.0));
    }

    #[test]
    fn a_one_point_route_is_the_goal() {
        let r = Route::new([Point::new(3.0, 4.0)]);
        let p = r.progress(Point::new(0.0, 0.0)).expect("a route");
        assert!(close(p.off_path, 5.0));
        assert!(close(p.remaining, 0.0));
    }

    #[test]
    fn repeated_points_and_nans_do_no_harm() {
        let r = Route::new([
            Point::new(0.0, 0.0),
            Point::new(0.0, 0.0),
            Point::new(f64::NAN, 1.0),
            Point::new(2.0, 0.0),
        ]);
        assert_eq!(r.points().len(), 3);
        let p = r.progress(Point::new(1.0, 0.0)).expect("a route");
        assert!(close(p.remaining, 1.0));
    }
}

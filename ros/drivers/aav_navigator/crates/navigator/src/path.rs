//! A path's geometry: the nearest point, the distance left, passed poses.

use crate::Pose;

/// The index of the path's point nearest to `p`, and how far it is (m). None: no path.
pub fn nearest(path: &[Pose], p: &Pose) -> Option<(usize, f64)> {
    path.iter()
        .enumerate()
        .map(|(i, q)| (i, q.distance(p)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// Metres along the path from its point nearest the car to its end; the straight line
/// with no path.
pub fn remaining(path: &[Pose], car: &Pose) -> f64 {
    match nearest(path, car) {
        None => 0.0,
        Some((i, gap)) => gap + length(&path[i..]),
    }
}

/// A path's length, m.
pub fn length(path: &[Pose]) -> f64 {
    path.windows(2).map(|w| w[0].distance(&w[1])).sum()
}

/// Has the car passed `pose` along `path`? True when the pose lies on the path (within
/// `on_path` m) and the car's nearest point is at or beyond the pose's.
pub fn passed_along(path: &[Pose], pose: &Pose, car: &Pose, on_path: f64) -> bool {
    match (nearest(path, pose), nearest(path, car)) {
        (Some((at, gap)), Some((here, _))) => gap <= on_path && here >= at,
        _ => false,
    }
}

/// A straight line from `car` to `distance` m behind it, a pose every `spacing` m, each
/// facing the way the car faces: a path a reversing follower backs along (BackUp, done by
/// the follower where no behaviour server runs). The car's own pose first.
pub fn straight_back(car: &Pose, distance: f64, spacing: f64) -> Vec<Pose> {
    let distance = distance.abs();
    let spacing = spacing.abs().max(0.01);
    let steps = (distance / spacing).ceil().max(1.0) as usize;
    let (sin, cos) = car.yaw.sin_cos();
    (0..=steps)
        .map(|i| {
            let back = distance * i as f64 / steps as f64;
            Pose::new(car.x - back * cos, car.y - back * sin, car.yaw)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(n: usize) -> Vec<Pose> {
        (0..n).map(|i| Pose::new(i as f64, 0.0, 0.0)).collect()
    }

    #[test]
    fn straight_back_runs_behind_the_car_facing_forward() {
        let car = Pose::new(2.0, 1.0, std::f64::consts::FRAC_PI_2); // facing +y
        let path = straight_back(&car, 1.0, 0.25);
        assert_eq!(path.len(), 5);
        assert_eq!(path[0], car);
        let end = path.last().unwrap();
        assert!((end.x - 2.0).abs() < 1e-9 && (end.y - 0.0).abs() < 1e-9);
        assert!(path.iter().all(|p| p.yaw == car.yaw));
        assert!((length(&path) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn nearest_and_remaining() {
        let path = line(11);
        assert_eq!(nearest(&path, &Pose::new(3.2, 0.5, 0.0)).unwrap().0, 3);
        let left = remaining(&path, &Pose::new(3.0, 0.0, 0.0));
        assert!((left - 7.0).abs() < 1e-9);
        assert_eq!(remaining(&[], &Pose::default()), 0.0);
        assert!((length(&path) - 10.0).abs() < 1e-9);
    }

    #[test]
    fn passed_along_the_path_not_only_near_it() {
        let path = line(21);
        let pose = Pose::new(10.0, 1.5, 0.0); // the bend, cut by 1.5 m
        assert!(!passed_along(&path, &pose, &Pose::new(8.0, 0.0, 0.0), 2.0));
        assert!(passed_along(&path, &pose, &Pose::new(11.0, 0.0, 0.0), 2.0));
        // Off the path: never passed this way.
        let far = Pose::new(10.0, 5.0, 0.0);
        assert!(!passed_along(&path, &far, &Pose::new(15.0, 0.0, 0.0), 2.0));
    }
}

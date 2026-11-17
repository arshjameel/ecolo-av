//! The radar's clusters as speeds along their rays, in odom.
//!
//! `aav_radar_clusters` sends every radar point with its cluster's number. Each cluster's
//! middle (the mean of its points) and mean radial speed, the car's own speed taken off,
//! give the nearest thing its speed along the ray ([`crate::Tracker::radar`]).

use ego_core::hypot;

use crate::{
    mean,
    tracker::{ground_radial_speed, RadarCluster},
    Pose2,
};

/// One radar point, in the radar's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadarPoint {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// m/s along the ray, relative to the radar (negative while closing).
    pub velocity: f64,
    /// Its cluster's number.
    pub cluster: f64,
}

/// The clusters among `points`, in odom, by their numbers in order. `radar`: the radar's
/// frame in odom; `car_velocity`: the car's, in odom (m/s). A point with a NaN is left
/// out; a cluster within 0.1 m of the radar has no ray, and is left out too.
#[must_use]
pub fn clusters(points: &[RadarPoint], radar: Pose2, car_velocity: [f64; 2]) -> Vec<RadarCluster> {
    let points: Vec<&RadarPoint> = points
        .iter()
        .filter(|p| !(p.x.is_nan() || p.y.is_nan() || p.velocity.is_nan() || p.cluster.is_nan()))
        .collect();
    let mut numbers: Vec<f64> = points.iter().map(|p| p.cluster).collect();
    numbers.sort_by(f64::total_cmp);
    numbers.dedup();
    let (c, s) = (radar.yaw.cos(), radar.yaw.sin());
    numbers
        .into_iter()
        .filter_map(|number| {
            let members: Vec<&&RadarPoint> =
                points.iter().filter(|p| p.cluster == number).collect();
            let column =
                |f: fn(&RadarPoint) -> f64| -> Vec<f64> { members.iter().map(|p| f(p)).collect() };
            let (rx, ry) = (mean(&column(|p| p.x)), mean(&column(|p| p.y)));
            let distance = hypot(rx, ry);
            if distance < 0.1 {
                return None;
            }
            let ray = [(c * rx - s * ry) / distance, (s * rx + c * ry) / distance];
            Some(RadarCluster {
                x: radar.x + c * rx - s * ry,
                y: radar.y + s * rx + c * ry,
                ray_x: ray[0],
                ray_y: ray[1],
                speed: ground_radial_speed(mean(&column(|p| p.velocity)), ray, car_velocity),
            })
        })
        .collect()
}

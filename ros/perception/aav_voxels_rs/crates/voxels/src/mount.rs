//! Where the lidar sits on the car: points from its frame into `base_link`.

use crate::Point;

/// `base_link → lidar_link`: `vehicle.yaml`'s `lidar_link`, which the launch file passes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mount {
    /// The rotation (roll, pitch, yaw as URDF and REP-103 mean them), row by row.
    turn: [[f64; 3]; 3],
    /// Where the lidar is in `base_link` (m).
    at: [f64; 3],
}

impl Mount {
    /// The lidar at `xyz` (m), turned by `rpy` (rad), in `base_link`.
    pub fn new(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        let [(sr, cr), (sp, cp), (sy, cy)] = rpy.map(f64::sin_cos);
        // aav_lidar scan.rotation's matrix.
        let turn = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ];
        Self { turn, at: xyz }
    }

    /// `p`, from the lidar's frame, in `base_link`.
    pub fn to_base(&self, p: &Point) -> Point {
        let [x, y, z] = [0, 1, 2].map(|row| {
            let r = self.turn[row];
            p.x * r[0] + p.y * r[1] + p.z * r[2] + self.at[row]
        });
        Point::new(x, y, z)
    }
}

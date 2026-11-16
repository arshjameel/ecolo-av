//! Nav2's local costmap, as `/local_costmap/costmap_raw` sends it, and the car's outline
//! on it (Nav2's FootprintCollisionChecker: the cost along the outline's edges).

use crate::model::Pose;

/// A cell holding an obstacle.
pub const LETHAL: u8 = 254;
/// A cell where the car's inscribed circle would touch an obstacle.
pub const INSCRIBED: u8 = 253;
/// A cell nobody has seen.
pub const NO_INFORMATION: u8 = 255;

/// A grid of Nav2 costs (0 free .. 254 lethal, 255 unknown), row by row from the origin.
#[derive(Debug, Clone, PartialEq)]
pub struct Costmap {
    /// m: the corner of cell (0, 0), odom frame.
    pub origin_x: f64,
    /// m.
    pub origin_y: f64,
    /// m per cell.
    pub resolution: f64,
    /// Cells along x.
    pub width: usize,
    /// Cells along y.
    pub height: usize,
    /// width x height costs.
    pub data: Vec<u8>,
}

impl Costmap {
    /// A grid, or `None` if its size and data disagree or the resolution isn't positive.
    #[must_use]
    pub fn new(
        origin: (f64, f64),
        resolution: f64,
        width: usize,
        height: usize,
        data: Vec<u8>,
    ) -> Option<Self> {
        (resolution > 0.0 && data.len() == width * height).then_some(Self {
            origin_x: origin.0,
            origin_y: origin.1,
            resolution,
            width,
            height,
            data,
        })
    }

    /// The cell holding (x, y), or `None` outside the grid.
    #[must_use]
    pub fn cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let cx = ((x - self.origin_x) / self.resolution).floor();
        let cy = ((y - self.origin_y) / self.resolution).floor();
        let inside = cx >= 0.0 && cy >= 0.0 && cx < self.width as f64 && cy < self.height as f64;
        inside.then_some((cx as usize, cy as usize))
    }

    /// The cost at cell (cx, cy).
    #[must_use]
    pub fn at(&self, cx: usize, cy: usize) -> u8 {
        self.data[cy * self.width + cx]
    }

    /// The cost at (x, y); unknown outside the grid.
    #[must_use]
    pub fn cost(&self, x: f64, y: f64) -> u8 {
        self.cell(x, y)
            .map_or(NO_INFORMATION, |(cx, cy)| self.at(cx, cy))
    }

    /// The highest cost along the line between two cells (Bresenham); lethal at once.
    fn line_cost(&self, a: (i64, i64), b: (i64, i64)) -> u8 {
        let (mut x, mut y) = a;
        let (dx, dy) = ((b.0 - a.0).abs(), -(b.1 - a.1).abs());
        let (sx, sy) = (
            if a.0 < b.0 { 1 } else { -1 },
            if a.1 < b.1 { 1 } else { -1 },
        );
        let mut err = dx + dy;
        let mut worst = 0u8;
        loop {
            let cost = if x < 0 || y < 0 || x >= self.width as i64 || y >= self.height as i64 {
                NO_INFORMATION
            } else {
                self.at(x as usize, y as usize)
            };
            if cost == LETHAL {
                return LETHAL;
            }
            worst = worst.max(cost);
            if (x, y) == b {
                return worst;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    /// The highest cost under the outline `footprint` (base_link points) at `pose`.
    #[must_use]
    pub fn footprint_cost(&self, pose: &Pose, footprint: &[(f64, f64)]) -> u8 {
        let (s, c) = pose.yaw.sin_cos();
        let cells: Vec<(i64, i64)> = footprint
            .iter()
            .map(|&(fx, fy)| {
                let x = pose.x + fx * c - fy * s;
                let y = pose.y + fx * s + fy * c;
                (
                    ((x - self.origin_x) / self.resolution).floor() as i64,
                    ((y - self.origin_y) / self.resolution).floor() as i64,
                )
            })
            .collect();
        let mut worst = 0u8;
        for (i, &a) in cells.iter().enumerate() {
            let b = cells[(i + 1) % cells.len()];
            let cost = self.line_cost(a, b);
            if cost == LETHAL {
                return LETHAL;
            }
            worst = worst.max(cost);
        }
        worst
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map_with_wall() -> Costmap {
        // 10 x 10 m at 0.1 m, a lethal wall along x = 5 m.
        let (w, h) = (100, 100);
        let mut data = vec![0u8; w * h];
        for cy in 0..h {
            data[cy * w + 50] = LETHAL;
        }
        Costmap::new((0.0, 0.0), 0.1, w, h, data).expect("a map")
    }

    #[test]
    fn cells_and_outside() {
        let m = map_with_wall();
        assert_eq!(m.cell(0.05, 0.05), Some((0, 0)));
        assert_eq!(m.cell(-0.01, 0.0), None);
        assert_eq!(m.cost(5.05, 1.0), LETHAL);
        assert_eq!(m.cost(20.0, 1.0), NO_INFORMATION);
        assert!(Costmap::new((0.0, 0.0), 0.1, 2, 2, vec![0; 3]).is_none());
    }

    #[test]
    fn footprint_touches_the_wall_only_when_it_reaches_it() {
        let m = map_with_wall();
        let car = [(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)];
        assert_eq!(m.footprint_cost(&Pose::new(2.0, 5.0, 0.0), &car), 0);
        assert_eq!(m.footprint_cost(&Pose::new(3.5, 5.0, 0.0), &car), LETHAL);
        // Turned to face along the wall, beside it: clear.
        let along = Pose::new(4.0, 3.0, std::f64::consts::FRAC_PI_2);
        assert_eq!(m.footprint_cost(&along, &car), 0);
    }
}

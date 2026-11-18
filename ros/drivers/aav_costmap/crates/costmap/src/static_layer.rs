//! Nav2's `StaticLayer`: the map (`/map`), read into costs.

use crate::cost::{FREE, LETHAL, NO_INFORMATION};
use crate::grid::Grid;
use crate::transform::Pose2;

/// The map as costs, in the map frame. Like Nav2, it ignores the map origin's yaw.
#[derive(Debug, Clone, PartialEq)]
pub struct StaticMap {
    /// The costs, on the map's own cells.
    pub grid: Grid,
}

/// At or above this occupancy (0-100), a map cell is lethal (Nav2's `lethal_cost_threshold`).
pub const LETHAL_THRESHOLD: i8 = 100;

impl StaticMap {
    /// The map from an `OccupancyGrid`'s values (-1 unknown, 0-100), read as Nav2 reads them
    /// with `trinary_costmap` (its default): lethal at the threshold, else free; unknown
    /// stays unknown only with `track_unknown`. `None` if the size and data disagree.
    #[must_use]
    pub fn from_occupancy(
        origin: (f64, f64),
        resolution: f64,
        width: usize,
        height: usize,
        values: &[i8],
        track_unknown: bool,
    ) -> Option<Self> {
        if values.len() != width * height || resolution <= 0.0 {
            return None;
        }
        let mut grid = Grid::new(origin, resolution, width, height, FREE);
        for (cell, &v) in grid.data.iter_mut().zip(values) {
            *cell = match v {
                -1 if track_unknown => NO_INFORMATION,
                -1 => FREE,
                v if v >= LETHAL_THRESHOLD => LETHAL,
                _ => FREE,
            };
        }
        Some(Self { grid })
    }

    /// Write the map into a grid in another frame, cell by cell (each cell's centre looked
    /// up in the map; cells off the map keep their cost): the rolling window's way.
    /// `map_from_grid` is the grid frame's pose in the map frame.
    pub fn stamp(&self, into: &mut Grid, map_from_grid: &Pose2) {
        for cy in 0..into.height {
            for cx in 0..into.width {
                let (x, y) = map_from_grid.apply(into.centre(cx, cy));
                if let Some((mx, my)) = self.grid.cell(x, y) {
                    into.set(cx, cy, self.grid.get(mx, my));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_values_as_nav2_does() {
        let m = StaticMap::from_occupancy((0.0, 0.0), 1.0, 4, 1, &[0, 65, 100, -1], true).unwrap();
        assert_eq!(m.grid.data, vec![FREE, FREE, LETHAL, NO_INFORMATION]);
        let m = StaticMap::from_occupancy((0.0, 0.0), 1.0, 4, 1, &[0, 65, 100, -1], false).unwrap();
        assert_eq!(m.grid.data[3], FREE);
        assert!(StaticMap::from_occupancy((0.0, 0.0), 1.0, 3, 1, &[0], false).is_none());
    }

    #[test]
    fn stamps_into_a_turned_window() {
        // A wall 1 m east of the map's origin.
        let mut values = vec![0i8; 100];
        values[5 * 10 + 6] = 100;
        let m = StaticMap::from_occupancy((-5.0, -5.0), 1.0, 10, 10, &values, false).unwrap();
        // The window's frame is turned a quarter left: its -y is the map's +x.
        let mut w = Grid::new((-2.0, -2.0), 1.0, 4, 4, FREE);
        m.stamp(&mut w, &Pose2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2));
        let (cx, cy) = w.cell(0.5, -1.5).unwrap();
        assert_eq!(w.get(cx, cy), LETHAL);
        assert_eq!(w.data.iter().filter(|&&c| c == LETHAL).count(), 1);
    }
}

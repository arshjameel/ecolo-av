//! A grid of costs on the ground: Nav2's `Costmap2D`.

use crate::cost::{INSCRIBED, LETHAL};

/// A grid of costs, row by row from the origin corner (x fastest), as Nav2 and
/// `nav_msgs/OccupancyGrid` store them.
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    /// m: the corner of cell (0, 0), in the grid's frame.
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

/// A rectangle of cells, `x0..x1` by `y0..y1` (ends excluded): the area an update changed
/// (Nav2's `updateBounds`, in cells).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bounds {
    /// The first column.
    pub x0: usize,
    /// The first row.
    pub y0: usize,
    /// One past the last column.
    pub x1: usize,
    /// One past the last row.
    pub y1: usize,
}

impl Bounds {
    /// One cell.
    pub fn cell(cx: usize, cy: usize) -> Self {
        Self {
            x0: cx,
            y0: cy,
            x1: cx + 1,
            y1: cy + 1,
        }
    }

    /// The smallest rectangle holding both.
    pub fn union(self, other: Self) -> Self {
        Self {
            x0: self.x0.min(other.x0),
            y0: self.y0.min(other.y0),
            x1: self.x1.max(other.x1),
            y1: self.y1.max(other.y1),
        }
    }

    /// Grown by `cells` on every side, cut to a `width` x `height` grid.
    pub fn grown(self, cells: usize, width: usize, height: usize) -> Self {
        Self {
            x0: self.x0.saturating_sub(cells),
            y0: self.y0.saturating_sub(cells),
            x1: (self.x1 + cells).min(width),
            y1: (self.y1 + cells).min(height),
        }
    }

    /// Columns.
    pub fn width(&self) -> usize {
        self.x1.saturating_sub(self.x0)
    }

    /// Rows.
    pub fn height(&self) -> usize {
        self.y1.saturating_sub(self.y0)
    }

    /// The bounds of the cells at these indices of a grid `width` cells wide.
    pub fn of_indices(indices: impl IntoIterator<Item = usize>, width: usize) -> Option<Self> {
        indices
            .into_iter()
            .map(|i| Self::cell(i % width, i / width))
            .reduce(Self::union)
    }
}

/// The union of two optional areas.
pub fn union(a: Option<Bounds>, b: Option<Bounds>) -> Option<Bounds> {
    match (a, b) {
        (Some(a), Some(b)) => Some(a.union(b)),
        (a, b) => a.or(b),
    }
}

impl Grid {
    /// A grid with every cell `fill`.
    pub fn new(origin: (f64, f64), resolution: f64, width: usize, height: usize, fill: u8) -> Self {
        Self {
            origin_x: origin.0,
            origin_y: origin.1,
            resolution,
            width,
            height,
            data: vec![fill; width * height],
        }
    }

    /// The same size and place, every cell `fill`.
    pub fn like(&self, fill: u8) -> Self {
        Self::new(
            (self.origin_x, self.origin_y),
            self.resolution,
            self.width,
            self.height,
            fill,
        )
    }

    /// The index of cell (cx, cy) in `data`.
    pub fn index(&self, cx: usize, cy: usize) -> usize {
        cy * self.width + cx
    }

    /// The cost of cell (cx, cy).
    pub fn get(&self, cx: usize, cy: usize) -> u8 {
        self.data[self.index(cx, cy)]
    }

    /// Set cell (cx, cy).
    pub fn set(&mut self, cx: usize, cy: usize, cost: u8) {
        let i = self.index(cx, cy);
        self.data[i] = cost;
    }

    /// The cell holding (x, y), or `None` outside (Nav2's `worldToMap`).
    pub fn cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        if x < self.origin_x || y < self.origin_y {
            return None;
        }
        let cx = ((x - self.origin_x) / self.resolution) as usize;
        let cy = ((y - self.origin_y) / self.resolution) as usize;
        (cx < self.width && cy < self.height).then_some((cx, cy))
    }

    /// The centre of cell (cx, cy) (Nav2's `mapToWorld`).
    pub fn centre(&self, cx: usize, cy: usize) -> (f64, f64) {
        (
            self.origin_x + (cx as f64 + 0.5) * self.resolution,
            self.origin_y + (cy as f64 + 0.5) * self.resolution,
        )
    }

    /// The cells in `b`, as a grid of their own (its corner on the same ground).
    pub fn crop(&self, b: Bounds) -> Self {
        let mut data = Vec::with_capacity(b.width() * b.height());
        for row in self
            .data
            .chunks_exact(self.width.max(1))
            .take(b.y1)
            .skip(b.y0)
        {
            data.extend_from_slice(&row[b.x0..b.x1]);
        }
        Self {
            origin_x: self.origin_x + b.x0 as f64 * self.resolution,
            origin_y: self.origin_y + b.y0 as f64 * self.resolution,
            resolution: self.resolution,
            width: b.width(),
            height: b.height(),
            data,
        }
    }

    /// Write `part`'s cells in `b` back, `part` being a crop at `at` (its first column
    /// and row in this grid).
    pub fn paste(&mut self, part: &Grid, at: (usize, usize), b: Bounds) {
        for y in b.y0..b.y1 {
            let from = (y - at.1) * part.width + (b.x0 - at.0);
            let to = y * self.width + b.x0;
            self.data[to..to + b.width()].copy_from_slice(&part.data[from..from + b.width()]);
        }
    }

    /// The cost at (x, y); `outside` off the grid.
    pub fn cost_at(&self, x: f64, y: f64, outside: u8) -> u8 {
        self.cell(x, y).map_or(outside, |(cx, cy)| self.get(cx, cy))
    }

    /// Nav2's IsPathValid (planner_server, `consider_footprint` false): the indices, from
    /// `from` on, of the points whose cell is lethal or inside the car's inscribed circle.
    /// A point off the grid can't be judged and is skipped, as Nav2's.
    pub fn blocked_along(&self, points: &[(f64, f64)], from: usize) -> Vec<usize> {
        points
            .iter()
            .enumerate()
            .skip(from)
            .filter(|(_, &(x, y))| {
                self.cell(x, y)
                    .is_some_and(|(cx, cy)| matches!(self.get(cx, cy), INSCRIBED | LETHAL))
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// The far corner, m.
    pub fn end(&self) -> (f64, f64) {
        (
            self.origin_x + self.width as f64 * self.resolution,
            self.origin_y + self.height as f64 * self.resolution,
        )
    }

    /// A distance in whole cells, rounded up (Nav2's `cellDistance`).
    pub fn cell_distance(&self, metres: f64) -> usize {
        (metres / self.resolution).ceil().max(0.0) as usize
    }

    /// Move the grid's corner near (x, y), keeping what the old and new grids share, and
    /// `fill` everywhere else (Nav2's `updateOrigin`, for the rolling window). The corner
    /// moves in whole cells, so a cell's cost stays on the same ground.
    pub fn move_origin(&mut self, x: f64, y: f64, fill: u8) {
        // Truncated towards zero, as Nav2's int().
        let ox = ((x - self.origin_x) / self.resolution) as i64;
        let oy = ((y - self.origin_y) / self.resolution) as i64;
        if ox == 0 && oy == 0 {
            return;
        }
        let (w, h) = (self.width as i64, self.height as i64);
        let old = std::mem::replace(&mut self.data, vec![fill; self.width * self.height]);
        // The old cells (ox.., oy..) become the new (0.., 0..).
        let (x0, x1) = (ox.clamp(0, w), (ox + w).clamp(0, w));
        let (y0, y1) = (oy.clamp(0, h), (oy + h).clamp(0, h));
        for oy_cell in y0..y1 {
            let ny = oy_cell - oy;
            let from = (oy_cell * w) as usize;
            let to = (ny * w) as usize;
            for ox_cell in x0..x1 {
                let nx = ox_cell - ox;
                self.data[to + nx as usize] = old[from + ox_cell as usize];
            }
        }
        self.origin_x += ox as f64 * self.resolution;
        self.origin_y += oy as f64 * self.resolution;
    }

    /// Visit the cells on the line from cell a to cell b, as Nav2's `raytraceLine`: from
    /// `min_length` cells along it, for at most `max_length` cells, then the cell it stops
    /// on. Both ends must be on the grid.
    pub fn raytrace(
        &self,
        a: (usize, usize),
        b: (usize, usize),
        max_length: usize,
        min_length: usize,
        mut visit: impl FnMut(usize),
    ) {
        let (x0, y0) = (a.0 as i64, a.1 as i64);
        let (x1, y1) = (b.0 as i64, b.1 as i64);
        let (dx_full, dy_full) = (x1 - x0, y1 - y0);
        let dist = (dx_full as f64).hypot(dy_full as f64);
        if dist < min_length as f64 {
            return;
        }
        let (sx, sy) = if dist > 0.0 {
            (
                (x0 as f64 + dx_full as f64 / dist * min_length as f64) as i64,
                (y0 as f64 + dy_full as f64 / dist * min_length as f64) as i64,
            )
        } else {
            (x0, y0)
        };
        let w = self.width as i64;
        let offset = sy * w + sx;
        let (dx, dy) = (x1 - sx, y1 - sy);
        let (abs_dx, abs_dy) = (dx.abs(), dy.abs());
        let (step_x, step_y) = (dx.signum(), dy.signum() * w);
        let scale = if dist == 0.0 {
            1.0
        } else {
            (max_length as f64 / dist).min(1.0)
        };
        if abs_dx >= abs_dy {
            let limit = (scale * abs_dx as f64) as i64;
            bresenham(
                abs_dx,
                abs_dy,
                abs_dx / 2,
                step_x,
                step_y,
                offset,
                limit,
                &mut visit,
            );
        } else {
            let limit = (scale * abs_dy as f64) as i64;
            bresenham(
                abs_dy,
                abs_dx,
                abs_dy / 2,
                step_y,
                step_x,
                offset,
                limit,
                &mut visit,
            );
        }
    }
}

/// Nav2's `bresenham2D`: `end` steps along the long axis a, then the cell it stops on.
#[allow(clippy::too_many_arguments)]
fn bresenham(
    abs_da: i64,
    abs_db: i64,
    mut error_b: i64,
    step_a: i64,
    step_b: i64,
    mut offset: i64,
    max_length: i64,
    visit: &mut impl FnMut(usize),
) {
    let end = max_length.min(abs_da);
    for _ in 0..end {
        visit(offset as usize);
        offset += step_a;
        error_b += abs_db;
        if error_b >= abs_da {
            offset += step_b;
            error_b -= abs_da;
        }
    }
    visit(offset as usize);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blocked_along_finds_lethal_and_inscribed_ahead_only() {
        let mut g = Grid::new((0.0, 0.0), 1.0, 10, 1, 0);
        g.set(2, 0, LETHAL);
        g.set(5, 0, INSCRIBED);
        g.set(7, 0, 252); // inflated, outside the inscribed circle: still free to drive
        g.set(8, 0, crate::cost::NO_INFORMATION);
        let line: Vec<(f64, f64)> = (0..12).map(|i| (i as f64 + 0.5, 0.5)).collect();
        assert_eq!(g.blocked_along(&line, 0), vec![2, 5]);
        assert_eq!(g.blocked_along(&line, 3), vec![5]);
        assert!(g.blocked_along(&line, 6).is_empty());
    }

    #[test]
    fn cells_and_centres() {
        let g = Grid::new((-1.0, -1.0), 0.5, 4, 4, 0);
        assert_eq!(g.cell(-1.0, -1.0), Some((0, 0)));
        assert_eq!(g.cell(0.99, 0.2), Some((3, 2)));
        assert_eq!(g.cell(1.0, 0.0), None);
        assert_eq!(g.cell(-1.01, 0.0), None);
        assert_eq!(g.centre(1, 2), (-0.25, 0.25));
        assert_eq!(g.cell_distance(0.51), 2);
    }

    #[test]
    fn moving_the_window_keeps_cells_on_the_same_ground() {
        let mut g = Grid::new((0.0, 0.0), 1.0, 5, 5, 0);
        g.set(3, 2, 254);
        g.move_origin(2.4, -1.0, 7);
        assert_eq!((g.origin_x, g.origin_y), (2.0, -1.0));
        let (cx, cy) = g.cell(3.5, 2.5).unwrap();
        assert_eq!(g.get(cx, cy), 254);
        // What came into the window is the fill.
        assert_eq!(g.cost_at(6.5, 0.5, 0), 7);
        assert_eq!(g.data.iter().filter(|&&c| c == 254).count(), 1);
    }

    #[test]
    fn a_crop_sits_on_the_same_ground_and_pastes_back() {
        let mut g = Grid::new((-1.0, -2.0), 0.5, 8, 6, 0);
        g.set(3, 2, 9);
        let read = Bounds::cell(3, 2).grown(2, g.width, g.height);
        assert_eq!(
            read,
            Bounds {
                x0: 1,
                y0: 0,
                x1: 6,
                y1: 5
            }
        );
        let mut part = g.crop(read);
        assert_eq!((part.width, part.height), (5, 5));
        assert_eq!(part.cost_at(0.8, -0.8, 0), 9);
        part.data.fill(4);
        g.paste(
            &part,
            (read.x0, read.y0),
            Bounds::cell(3, 2).grown(1, g.width, g.height),
        );
        assert_eq!(g.data.iter().filter(|&&c| c == 4).count(), 9);
        assert_eq!(g.get(3, 2), 4);
        assert_eq!(g.get(5, 2), 0);
        assert_eq!(
            Bounds::of_indices([g.index(7, 1), g.index(2, 4)], g.width),
            Some(Bounds {
                x0: 2,
                y0: 1,
                x1: 8,
                y1: 5
            })
        );
    }

    #[test]
    fn a_window_moved_right_off_keeps_nothing() {
        let mut g = Grid::new((0.0, 0.0), 1.0, 3, 3, 254);
        g.move_origin(10.0, 10.0, 0);
        assert!(g.data.iter().all(|&c| c == 0));
    }

    #[test]
    fn a_ray_visits_every_cell_to_its_end() {
        let g = Grid::new((0.0, 0.0), 1.0, 10, 10, 0);
        let mut seen = Vec::new();
        g.raytrace((1, 1), (6, 3), usize::MAX, 0, |i| seen.push(i));
        assert_eq!(seen.first(), Some(&g.index(1, 1)));
        assert_eq!(seen.last(), Some(&g.index(6, 3)));
        assert_eq!(seen.len(), 6);
    }

    #[test]
    fn a_ray_stops_at_its_longest() {
        let g = Grid::new((0.0, 0.0), 1.0, 20, 20, 0);
        let mut seen = Vec::new();
        g.raytrace((0, 0), (10, 0), 4, 0, |i| seen.push(i));
        // Four steps, then the cell it stopped on.
        assert_eq!(seen, vec![0, 1, 2, 3, 4]);
        seen.clear();
        g.raytrace((0, 0), (10, 0), usize::MAX, 3, |i| seen.push(i));
        assert_eq!(seen.first(), Some(&3));
    }
}

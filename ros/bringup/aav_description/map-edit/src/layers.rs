//! What the window draws, as plain pictures and outlines: no window code here.
//!
//! | Layer | From |
//! |---|---|
//! | NDT's points from above | [`HeightImage`]: each cell the highest point in it, coloured by height |
//! | the 2D map | [`grid_rgba`]: occupied white, free dim, unknown clear |
//! | the 64 m chunks | [`chunk_outlines`]: the chunks a save would write |

use map_chunks::{ChunkKey, Grid};

use crate::folder::{Cell, Point3};
use crate::geom::Point;

/// The biggest picture side (pixels): a campus-sized map gets bigger cells.
pub const MAX_SIDE: usize = 4096;

/// NDT's points from above: each cell the highest point in it.
#[derive(Debug, Clone, PartialEq)]
pub struct HeightImage {
    /// The lower-left corner (m).
    pub origin: Point,
    /// One cell's side (m).
    pub cell: f64,
    /// Cells across.
    pub width: usize,
    /// Cells up.
    pub height: usize,
    /// The highest z in each cell, bottom row first; NaN where there is none.
    pub top: Vec<f32>,
    /// The lowest and highest z of all (m): the colours' ends.
    pub z_range: (f32, f32),
}

impl HeightImage {
    /// The picture of these points, with cells of `cell` m (bigger if it would pass
    /// [`MAX_SIDE`]). `None` with no points.
    pub fn new<'a>(
        points: impl IntoIterator<Item = &'a Point3> + Clone,
        cell: f64,
    ) -> Option<Self> {
        let finite = |p: &&Point3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
        let (lo, hi) = crate::geom::bounds_of(
            points
                .clone()
                .into_iter()
                .filter(finite)
                .map(|p| Point::new(p.x, p.y)),
        )?;
        let span = (hi.x - lo.x).max(hi.y - lo.y);
        let cell = cell.max(span / (MAX_SIDE - 1) as f64).max(1e-3);
        let width = ((hi.x - lo.x) / cell).floor() as usize + 1;
        let height = ((hi.y - lo.y) / cell).floor() as usize + 1;
        let mut top = vec![f32::NAN; width * height];
        let mut z_range = (f32::INFINITY, f32::NEG_INFINITY);
        for p in points.into_iter().filter(finite) {
            let col = (((p.x - lo.x) / cell) as usize).min(width - 1);
            let row = (((p.y - lo.y) / cell) as usize).min(height - 1);
            let z = p.z as f32;
            let slot = &mut top[row * width + col];
            if slot.is_nan() || z > *slot {
                *slot = z;
            }
            z_range = (z_range.0.min(z), z_range.1.max(z));
        }
        Some(Self {
            origin: lo,
            cell,
            width,
            height,
            top,
            z_range,
        })
    }

    /// The picture's box (lower left, upper right; m).
    #[must_use]
    pub fn bounds(&self) -> (Point, Point) {
        (
            self.origin,
            Point::new(
                self.origin.x + self.width as f64 * self.cell,
                self.origin.y + self.height as f64 * self.cell,
            ),
        )
    }

    /// RGBA pixels, top row first (as pictures are): coloured by height, clear where
    /// there is no point.
    #[must_use]
    pub fn rgba(&self) -> Vec<u8> {
        let (low, high) = self.z_range;
        let spread = (high - low).max(1e-3);
        self.top
            .chunks(self.width.max(1))
            .rev()
            .flatten()
            .flat_map(|&z| {
                if z.is_nan() {
                    [0, 0, 0, 0]
                } else {
                    let [r, g, b] = height_rgb((z - low) / spread);
                    [r, g, b, 255]
                }
            })
            .collect()
    }
}

/// A height's colour, low to high (0 to 1): blue, green, yellow, red.
#[must_use]
pub fn height_rgb(t: f32) -> [u8; 3] {
    const STOPS: [[f32; 3]; 4] = [
        [40.0, 90.0, 255.0],
        [40.0, 210.0, 110.0],
        [250.0, 220.0, 50.0],
        [240.0, 60.0, 50.0],
    ];
    let t = if t.is_finite() {
        t.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let at = t * (STOPS.len() - 1) as f32;
    let i = (at.floor() as usize).min(STOPS.len() - 2);
    let f = at - i as f32;
    let mix = |k: usize| (STOPS[i][k] + (STOPS[i + 1][k] - STOPS[i][k]) * f).round() as u8;
    [mix(0), mix(1), mix(2)]
}

/// The 2D map's cells as RGBA pixels, top row first: occupied white, free a dim grey,
/// unknown clear.
#[must_use]
pub fn grid_rgba(cells: &[Cell], width: usize) -> Vec<u8> {
    cells
        .chunks(width.max(1))
        .rev()
        .flatten()
        .flat_map(|c| match c {
            Cell::Occupied => [245, 245, 245, 255],
            Cell::Free => [90, 90, 105, 120],
            Cell::Unknown => [0, 0, 0, 0],
        })
        .collect()
}

/// The chunks of `size` m from `origin` that the box `lo`..`hi` touches, each with its
/// lower-left and upper-right corners.
#[must_use]
pub fn chunk_outlines(
    origin: Point,
    lo: Point,
    hi: Point,
    size: f64,
) -> Vec<(ChunkKey, (Point, Point))> {
    let Ok(grid) = Grid::new(origin, size) else {
        return Vec::new();
    };
    let (a, b) = (grid.key_at(lo), grid.key_at(hi));
    (a.j..=b.j)
        .flat_map(|j| (a.i..=b.i).map(move |i| ChunkKey::new(i, j)))
        .map(|key| {
            let c = grid.corner(key);
            (key, (c, Point::new(c.x + size, c.y + size)))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_highest_point_wins_its_cell() {
        let points = [
            Point3::new(0.0, 0.0, 0.5),
            Point3::new(0.05, 0.05, 3.0),
            Point3::new(1.0, 0.0, 1.0),
        ];
        let image = HeightImage::new(points.iter(), 0.1).expect("an image");
        assert_eq!((image.width, image.height), (11, 1));
        assert_eq!(image.top[0], 3.0);
        assert!(image.top[5].is_nan());
        assert_eq!(image.z_range, (0.5, 3.0));
        let rgba = image.rgba();
        assert_eq!(rgba.len(), 11 * 4);
        assert_eq!(&rgba[..4], &[240, 60, 50, 255]);
        assert_eq!(rgba[5 * 4 + 3], 0);
        assert!(HeightImage::new([].iter(), 0.1).is_none());
    }

    #[test]
    fn a_huge_map_gets_bigger_cells() {
        let points = [Point3::new(0.0, 0.0, 0.0), Point3::new(10_000.0, 10.0, 1.0)];
        let image = HeightImage::new(points.iter(), 0.1).expect("an image");
        assert!(image.width <= MAX_SIDE);
    }

    #[test]
    fn colours_run_low_to_high() {
        assert_eq!(height_rgb(0.0), [40, 90, 255]);
        assert_eq!(height_rgb(1.0), [240, 60, 50]);
        assert_eq!(height_rgb(f32::NAN), [40, 90, 255]);
    }

    #[test]
    fn grid_pictures_are_top_row_first() {
        let cells = [Cell::Occupied, Cell::Free, Cell::Unknown, Cell::Unknown];
        let rgba = grid_rgba(&cells, 2);
        assert_eq!(rgba[3], 0); // top left: the upper row's first cell, unknown
        assert_eq!(&rgba[8..12], &[245, 245, 245, 255]);
    }

    #[test]
    fn chunks_cover_the_box() {
        let outlines = chunk_outlines(
            Point::new(-35.0, -12.0),
            Point::new(-10.0, -9.0),
            Point::new(69.0, 49.0),
            64.0,
        );
        let keys: Vec<String> = outlines.iter().map(|(k, _)| k.to_string()).collect();
        assert_eq!(keys, ["0_0", "1_0"]);
        assert_eq!(outlines[1].1 .0, Point::new(29.0, -12.0));
    }
}

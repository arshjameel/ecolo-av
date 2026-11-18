//! The car's outline: Nav2's `footprint.cpp` (padding, the inscribed radius) and
//! `setConvexPolygonCost` (the cells under it).

use crate::grid::Grid;
use crate::transform::Pose2;

/// An outline from Nav2's string form, `[[x, y], [x, y], ...]`; `None` if it has fewer
/// than three corners or an odd count of numbers.
pub fn parse(text: &str) -> Option<Vec<(f64, f64)>> {
    let numbers: Vec<f64> = text
        .split(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')))
        .filter(|s| !s.is_empty())
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    if !numbers.len().is_multiple_of(2) || numbers.len() < 6 {
        return None;
    }
    Some(numbers.chunks(2).map(|p| (p[0], p[1])).collect())
}

/// Each corner moved `padding` m further out along x and y (Nav2's `padFootprint`).
#[must_use]
pub fn padded(points: &[(f64, f64)], padding: f64) -> Vec<(f64, f64)> {
    let out = |v: f64| {
        if v > 0.0 {
            v + padding
        } else if v < 0.0 {
            v - padding
        } else {
            v
        }
    };
    points.iter().map(|&(x, y)| (out(x), out(y))).collect()
}

/// The inscribed and circumscribed radii from base_link, m (Nav2's
/// `calculateMinAndMaxDistances`: the nearest and furthest corner or edge).
#[must_use]
pub fn radii(points: &[(f64, f64)]) -> (f64, f64) {
    let mut near = f64::INFINITY;
    let mut far: f64 = 0.0;
    for (i, &a) in points.iter().enumerate() {
        let b = points[(i + 1) % points.len()];
        let corner = a.0.hypot(a.1);
        let edge = distance_to_segment((0.0, 0.0), a, b);
        near = near.min(corner).min(edge);
        far = far.max(corner).max(edge);
    }
    (near, far)
}

fn distance_to_segment(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}

/// The outline at `pose`, in the pose's parent frame.
#[must_use]
pub fn placed(points: &[(f64, f64)], pose: &Pose2) -> Vec<(f64, f64)> {
    points.iter().map(|&p| pose.apply(p)).collect()
}

/// The indices of the cells under a convex outline: its edges' cells and every cell whose
/// centre is inside. Empty if a corner is off the grid (Nav2 then clears nothing).
#[must_use]
pub fn cells_under(grid: &Grid, outline: &[(f64, f64)]) -> Vec<usize> {
    let Some(corners) = outline
        .iter()
        .map(|&(x, y)| grid.cell(x, y))
        .collect::<Option<Vec<_>>>()
    else {
        return Vec::new();
    };
    let mut cells = Vec::new();
    for (i, &a) in corners.iter().enumerate() {
        let b = corners[(i + 1) % corners.len()];
        grid.raytrace(a, b, usize::MAX, 0, |c| cells.push(c));
    }
    let (lo_x, hi_x) = min_max(corners.iter().map(|c| c.0));
    let (lo_y, hi_y) = min_max(corners.iter().map(|c| c.1));
    for cy in lo_y..=hi_y {
        for cx in lo_x..=hi_x {
            if inside(grid.centre(cx, cy), outline) {
                cells.push(grid.index(cx, cy));
            }
        }
    }
    cells
}

fn min_max(values: impl Iterator<Item = usize>) -> (usize, usize) {
    values.fold((usize::MAX, 0), |(lo, hi), v| (lo.min(v), hi.max(v)))
}

/// Is the point inside the polygon (even-odd rule)?
fn inside((x, y): (f64, f64), polygon: &[(f64, f64)]) -> bool {
    let mut odd = false;
    for (i, &(ax, ay)) in polygon.iter().enumerate() {
        let (bx, by) = polygon[(i + 1) % polygon.len()];
        if (ay > y) != (by > y) && x < ax + (y - ay) / (by - ay) * (bx - ax) {
            odd = !odd;
        }
    }
    odd
}

#[cfg(test)]
mod tests {
    use super::*;

    /// nav2.yaml's outline before the launch file writes the car's in.
    const CAR: &str = "[[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]]";

    #[test]
    fn parses_nav2s_string() {
        let p = parse(CAR).unwrap();
        assert_eq!(p, vec![(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)]);
        assert!(parse("[[1, 2], [3]]").is_none());
        assert!(parse("[]").is_none());
    }

    #[test]
    fn the_cars_radii_padded_as_nav2_does() {
        let p = padded(&parse(CAR).unwrap(), 0.05);
        assert_eq!(p[2], (-0.25, -0.55));
        let (inscribed, circumscribed) = radii(&p);
        // The rear edge, 0.25 m behind base_link; the front corners, 1.93 m away
        // (docs/costmaps.md: the circumscribed radius).
        assert!((inscribed - 0.25).abs() < 1e-12);
        assert!((circumscribed - 1.85f64.hypot(0.55)).abs() < 1e-12);
    }

    #[test]
    fn the_cells_under_the_car() {
        let g = Grid::new((-5.0, -5.0), 0.1, 100, 100, 0);
        let outline = placed(&parse(CAR).unwrap(), &Pose2::IDENTITY);
        let mut cells = cells_under(&g, &outline);
        cells.sort_unstable();
        cells.dedup();
        // 2 m x 1 m at 0.1 m: 200 cells, and the edges' cells round it up a little.
        assert!((200..=260).contains(&cells.len()), "{}", cells.len());
        assert!(cells.contains(&g.index(55, 50)));
        assert!(!cells.contains(&g.index(75, 50)));
    }
}

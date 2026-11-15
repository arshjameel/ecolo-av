//! Nav2's `ObstacleLayer`: each sensor's points mark lethal cells, and each ray from the
//! sensor to a point clears the cells it passes. The layer keeps its own grid, so a mark
//! stays until a later ray clears it.

use crate::cost::{FREE, LETHAL, NO_INFORMATION};
use crate::footprint;
use crate::grid::{self, Bounds, Grid};

/// One observation source's settings: Nav2's names, from nav2.yaml.
#[derive(Debug, Clone, PartialEq)]
pub struct Source {
    /// Its name in nav2.yaml (`lidar`, `radar`...).
    pub name: String,
    /// Its points mark lethal cells.
    pub marking: bool,
    /// The rays to its points clear cells.
    pub clearing: bool,
    /// m, in the costmap's frame: points lower than this are dropped.
    pub min_obstacle_height: f64,
    /// m: points higher than this are dropped.
    pub max_obstacle_height: f64,
    /// m from the sensor: nearer points don't mark.
    pub obstacle_min_range: f64,
    /// m: points this far or further don't mark.
    pub obstacle_max_range: f64,
    /// m: a ray clears from this far out.
    pub raytrace_min_range: f64,
    /// m: a ray clears no further than this.
    pub raytrace_max_range: f64,
    /// A scan's inf means "nothing out to its range": clear the whole ray.
    pub inf_is_valid: bool,
}

impl Source {
    /// Nav2's defaults (marking and clearing, 0-2 m high, 0-2.5 m marks, 0-3 m clears).
    pub fn named(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            marking: true,
            clearing: false,
            min_obstacle_height: 0.0,
            max_obstacle_height: 2.0,
            obstacle_min_range: 0.0,
            obstacle_max_range: 2.5,
            raytrace_min_range: 0.0,
            raytrace_max_range: 3.0,
            inf_is_valid: false,
        }
    }
}

/// What one sensor saw, in the costmap's frame (odom or map).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Observation {
    /// m: where the sensor was.
    pub origin: [f64; 3],
    /// m: its points.
    pub points: Vec<[f64; 3]>,
}

/// A laser scan's points in its own frame, as Nav2 gets them: with `inf_is_valid` an inf
/// range becomes a point just short of `range_max` (clears, never marks past the marking
/// range); NaN and ranges outside [range_min, range_max) are dropped.
pub fn scan_points(
    angle_min: f64,
    angle_increment: f64,
    range_min: f64,
    range_max: f64,
    ranges: &[f32],
    inf_is_valid: bool,
) -> Vec<[f64; 3]> {
    ranges
        .iter()
        .enumerate()
        .filter_map(|(i, &r)| {
            let mut r = f64::from(r);
            if inf_is_valid && r.is_infinite() && r > 0.0 {
                r = range_max - 1e-4;
            }
            (r.is_finite() && r >= range_min && r < range_max).then(|| {
                let (s, c) = (angle_min + i as f64 * angle_increment).sin_cos();
                [r * c, r * s, 0.0]
            })
        })
        .collect()
}

/// One obstacle layer: its sources' settings, and the grid it keeps.
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleLayer {
    /// Its name in nav2.yaml (`lidar_layer`...).
    pub name: String,
    /// Its sources.
    pub sources: Vec<Source>,
    /// m: the layer's own bounds on a marking point's height (nav2.yaml's layer level).
    pub min_obstacle_height: f64,
    /// m.
    pub max_obstacle_height: f64,
    /// Clear the cells under the car (Nav2's `footprint_clearing_enabled`, on by default).
    pub footprint_clearing: bool,
    /// What it holds: marks, cleared cells, and `default` where nothing has been seen.
    pub grid: Grid,
    /// FREE, or NO_INFORMATION with `track_unknown_space`.
    pub default: u8,
}

impl ObstacleLayer {
    /// A layer with an empty grid of the master's size.
    pub fn new(name: &str, sources: Vec<Source>, like: &Grid, track_unknown: bool) -> Self {
        let default = if track_unknown { NO_INFORMATION } else { FREE };
        Self {
            name: name.to_owned(),
            sources,
            min_obstacle_height: -f64::INFINITY,
            max_obstacle_height: 2.0,
            footprint_clearing: true,
            grid: like.like(default),
            default,
        }
    }

    /// Take what the sensors saw: `observations[i]` is source i's (its newest, or those
    /// within its persistence; none if it is silent). Clears along every clearing source's
    /// rays first, then marks, then clears under `outline` (the car, in the costmap's
    /// frame), as Nav2 does. Returns the cells it touched (Nav2's `updateBounds`): every
    /// cell a ray cleared, a point marked or the car stood on, whether its cost changed or
    /// not; `None` if none.
    pub fn update(
        &mut self,
        observations: &[Vec<&Observation>],
        outline: &[(f64, f64)],
    ) -> Option<Bounds> {
        let mut touched = None;
        let sources = &self.sources;
        let seen = || {
            sources
                .iter()
                .zip(observations)
                .flat_map(|(s, list)| list.iter().map(move |o| (s, *o)))
        };
        let clears: Vec<_> = seen().filter(|(s, _)| s.clearing).collect();
        for (source, obs) in clears {
            touched = grid::union(touched, raytrace_free(&mut self.grid, source, obs));
        }
        let marks: Vec<_> = seen().filter(|(s, _)| s.marking).collect();
        for (source, obs) in marks {
            for p in obs.points.iter().filter(|p| keeps(source, p)) {
                if p[2] > self.max_obstacle_height || p[2] < self.min_obstacle_height {
                    continue;
                }
                let d2 = (p[0] - obs.origin[0]).powi(2)
                    + (p[1] - obs.origin[1]).powi(2)
                    + (p[2] - obs.origin[2]).powi(2);
                if d2 >= source.obstacle_max_range.powi(2) || d2 < source.obstacle_min_range.powi(2)
                {
                    continue;
                }
                if let Some((cx, cy)) = self.grid.cell(p[0], p[1]) {
                    self.grid.set(cx, cy, LETHAL);
                    touched = grid::union(touched, Some(Bounds::cell(cx, cy)));
                }
            }
        }
        if self.footprint_clearing {
            let under = footprint::cells_under(&self.grid, outline);
            for &i in &under {
                self.grid.data[i] = FREE;
            }
            touched = grid::union(touched, Bounds::of_indices(under, self.grid.width));
        }
        touched
    }

    /// Into the master grid, keeping the higher cost; this layer's unknown cells add
    /// nothing (Nav2's `updateWithMax`).
    pub fn combine_into(&self, master: &mut Grid) {
        combine(&mut master.data, &self.grid.data);
    }

    /// The same, for the cells in `b` only, into `part`: a crop of the master at `b`'s
    /// corner (Grid::crop).
    pub fn combine_area(&self, part: &mut Grid, b: Bounds) {
        let w = self.grid.width;
        for (y, row) in (b.y0..b.y1).zip(part.data.chunks_exact_mut(b.width().max(1))) {
            combine(row, &self.grid.data[y * w + b.x0..y * w + b.x1]);
        }
    }
}

/// Nav2's `updateWithMax`, cell by cell.
fn combine(master: &mut [u8], layer: &[u8]) {
    for (m, &c) in master.iter_mut().zip(layer) {
        if c != NO_INFORMATION && (*m == NO_INFORMATION || *m < c) {
            *m = c;
        }
    }
}

/// The source's own height band (Nav2's observation buffer).
fn keeps(source: &Source, p: &[f64; 3]) -> bool {
    p[2] >= source.min_obstacle_height && p[2] <= source.max_obstacle_height
}

/// Nav2's `raytraceFreespace`: each ray from the sensor, cut at the grid's edge, clears.
/// Returns the cells it cleared.
fn raytrace_free(grid: &mut Grid, source: &Source, obs: &Observation) -> Option<Bounds> {
    let (ox, oy) = (obs.origin[0], obs.origin[1]);
    let start = grid.cell(ox, oy)?;
    let (gx, gy) = (grid.origin_x, grid.origin_y);
    let (ex, ey) = grid.end();
    let max_cells = grid.cell_distance(source.raytrace_max_range);
    let min_cells = grid.cell_distance(source.raytrace_min_range);
    let mut cleared = Vec::new();
    for p in obs.points.iter().filter(|p| keeps(source, p)) {
        let (mut wx, mut wy) = (p[0], p[1]);
        let (a, b) = (wx - ox, wy - oy);
        if wx < gx {
            let t = (gx - ox) / a;
            wx = gx;
            wy = oy + b * t;
        }
        if wy < gy {
            let t = (gy - oy) / b;
            wx = ox + a * t;
            wy = gy;
        }
        if wx > ex {
            let t = (ex - ox) / a;
            wx = ex - 0.001;
            wy = oy + b * t;
        }
        if wy > ey {
            let t = (ey - oy) / b;
            wx = ox + a * t;
            wy = ey - 0.001;
        }
        if let Some(end) = grid.cell(wx, wy) {
            grid.raytrace(start, end, max_cells, min_cells, |i| cleared.push(i));
        }
    }
    for &i in &cleared {
        grid.data[i] = FREE;
    }
    Bounds::of_indices(cleared, grid.width)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lidar() -> Source {
        Source {
            clearing: true,
            min_obstacle_height: -0.5,
            max_obstacle_height: 0.5,
            obstacle_max_range: 12.0,
            raytrace_max_range: 15.0,
            inf_is_valid: true,
            ..Source::named("lidar")
        }
    }

    fn layer() -> ObstacleLayer {
        let g = Grid::new((-10.0, -10.0), 0.1, 200, 200, FREE);
        let mut l = ObstacleLayer::new("lidar_layer", vec![lidar()], &g, false);
        l.min_obstacle_height = -0.5;
        l.max_obstacle_height = 0.5;
        l
    }

    fn seen(points: Vec<[f64; 3]>) -> Observation {
        Observation {
            origin: [0.0; 3],
            points,
        }
    }

    #[test]
    fn a_point_marks_and_its_ray_clears() {
        let mut l = layer();
        l.update(&[vec![&seen(vec![[5.0, 0.0, 0.0]])]], &[]);
        assert_eq!(l.grid.cost_at(5.05, 0.05, 0), LETHAL);
        // Something moved away: the next scan sees further along the same ray.
        l.update(&[vec![&seen(vec![[8.0, 0.0, 0.0]])]], &[]);
        assert_eq!(l.grid.cost_at(5.05, 0.05, 0), FREE);
        assert_eq!(l.grid.cost_at(8.05, 0.05, 0), LETHAL);
    }

    #[test]
    fn marks_only_within_range_and_height() {
        let mut l = layer();
        let far = [12.5, 0.0, 0.0];
        let high = [3.0, 3.0, 0.8];
        l.update(&[vec![&seen(vec![far, high])]], &[]);
        assert!(l.grid.data.iter().all(|&c| c == FREE));
    }

    #[test]
    fn inf_clears_out_to_the_raytrace_range() {
        let mut l = layer();
        l.grid.set(150, 100, LETHAL); // 5 m ahead
        let points = scan_points(0.0, 0.1, 0.2, 30.0, &[f32::INFINITY], true);
        assert_eq!(points.len(), 1);
        l.update(&[vec![&seen(points)]], &[]);
        assert_eq!(l.grid.get(150, 100), FREE);
        // Without inf_is_valid an inf is dropped, and clears nothing.
        assert!(scan_points(0.0, 0.1, 0.2, 30.0, &[f32::INFINITY, f32::NAN], false).is_empty());
    }

    #[test]
    fn the_cars_own_cells_are_cleared() {
        let mut l = layer();
        l.update(&[vec![&seen(vec![[1.0, 0.0, 0.0]])]], &[]);
        assert_eq!(l.grid.cost_at(1.05, 0.05, 0), LETHAL);
        let car = [(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)];
        l.update(&[vec![]], &car);
        assert_eq!(l.grid.cost_at(1.05, 0.05, 0), FREE);
    }

    #[test]
    fn combining_keeps_the_higher_and_ignores_unknown() {
        let mut l = layer();
        l.grid.data[0] = NO_INFORMATION;
        l.grid.data[1] = LETHAL;
        let mut master = l.grid.like(FREE);
        master.data[0] = 100;
        l.combine_into(&mut master);
        assert_eq!(master.data[..3], [100, LETHAL, FREE]);
    }
}

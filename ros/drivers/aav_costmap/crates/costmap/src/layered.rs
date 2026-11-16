//! Nav2's `LayeredCostmap`: the static map, then each obstacle layer, then the inflation,
//! into one grid. Rolling (the local costmap: a window round the car, in odom) or fixed
//! to the map (the global costmap, in map).
//!
//! A fixed costmap recomputes every cell only when its map arrives or it is cleared;
//! otherwise only the cells its obstacle layers touched, grown by the inflation's reach
//! (Nav2's `updateBounds` and `updateCosts`). A rolling one moves every update, so it is
//! recomputed whole (a 200 x 200 window: well under a millisecond).

use crate::cost::{FREE, NO_INFORMATION};
use crate::footprint;
use crate::grid::{self, Bounds, Grid};
use crate::inflation::Inflation;
use crate::lanes::{self, Lanes};
use crate::obstacle::{Observation, ObstacleLayer};
use crate::static_layer::StaticMap;
use crate::transform::Pose2;

/// One costmap's settings: Nav2's, from nav2.yaml.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// A window that moves with the car (`rolling_window`).
    pub rolling: bool,
    /// m: the window's size (rolling only; a fixed costmap takes the map's).
    pub width: f64,
    /// m.
    pub height: f64,
    /// m per cell (rolling only; a fixed costmap takes the map's).
    pub resolution: f64,
    /// Unknown cells stay unknown (`track_unknown_space`); else they are free.
    pub track_unknown: bool,
    /// The car's outline in base_link, padded (footprint::padded).
    pub footprint: Vec<(f64, f64)>,
    /// m, and 1/m: the inflation (`inflation_layer`); None without one.
    pub inflation: Option<(f64, f64)>,
    /// A static layer reads the map.
    pub static_layer: bool,
}

/// A costmap: its layers and the grid they make.
#[derive(Debug, Clone)]
pub struct Costmap {
    settings: Settings,
    master: Grid,
    map: Option<StaticMap>,
    layers: Vec<ObstacleLayer>,
    inflation: Option<Inflation>,
    /// The next update recomputes every cell (a new map, a clear, the first update).
    whole: bool,
    /// The lanes layer (a fixed costmap only; TODO 148), and its costs on the master's
    /// cells, drawn when the map comes and again where a new route changes them.
    lanes: Option<(Lanes, Grid)>,
    /// The cells a new route redrew, for the next update.
    lanes_touched: Option<Bounds>,
}

/// What an update changed in the grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Changed {
    /// Nothing: the grid is as it was.
    Nothing,
    /// Only the cells in this rectangle.
    Area(Bounds),
    /// Any cell.
    All,
}

impl Costmap {
    /// A costmap with these obstacle layers (their grids are resized to fit). A fixed one
    /// is empty until it has a map.
    #[must_use]
    pub fn new(settings: Settings, layers: Vec<ObstacleLayer>) -> Self {
        let (inscribed, _) = footprint::radii(&settings.footprint);
        let inflation = settings.inflation.map(|(radius, scaling)| Inflation {
            inflation_radius: radius,
            cost_scaling_factor: scaling,
            inscribed_radius: inscribed,
        });
        let master = if settings.rolling {
            let cells = |m: f64| (m / settings.resolution).round().max(1.0) as usize;
            Grid::new(
                (0.0, 0.0),
                settings.resolution,
                cells(settings.width),
                cells(settings.height),
                default(&settings),
            )
        } else {
            Grid::new((0.0, 0.0), settings.resolution, 0, 0, default(&settings))
        };
        let mut c = Self {
            settings,
            master,
            map: None,
            layers,
            inflation,
            whole: true,
            lanes: None,
            lanes_touched: None,
        };
        c.resize_layers();
        c
    }

    /// The lanes layer on: the roads' costs go over the inflation (TODO 148). A rolling
    /// costmap ignores it: the lanes are the planner's, on the global costmap.
    pub fn set_lanes(&mut self, lanes: Lanes) {
        if self.settings.rolling {
            return;
        }
        let mut grid = self.master.like(0);
        let all = everything(&grid);
        lanes.draw(&mut grid, all);
        self.lanes = Some((lanes, grid));
        self.whole = true;
    }

    /// The lanes layer, if on.
    #[must_use]
    pub fn lanes(&self) -> Option<&Lanes> {
        self.lanes.as_ref().map(|(l, _)| l)
    }

    /// A new route (map frame, in driving order): the lanes it drives are drawn its way,
    /// in the next update. Returns true if any cost changed.
    pub fn set_route(&mut self, route: &[(f64, f64)]) -> bool {
        let Some((lanes, grid)) = self.lanes.as_mut() else {
            return false;
        };
        let Some((x0, y0, x1, y1)) = lanes.set_route(route) else {
            return false;
        };
        let cell = |v: f64, origin: f64, n: usize| {
            (((v - origin) / grid.resolution).floor().max(0.0) as usize).min(n)
        };
        let area = Bounds {
            x0: cell(x0, grid.origin_x, grid.width),
            y0: cell(y0, grid.origin_y, grid.height),
            x1: cell(x1, grid.origin_x, grid.width)
                .saturating_add(1)
                .min(grid.width),
            y1: cell(y1, grid.origin_y, grid.height)
                .saturating_add(1)
                .min(grid.height),
        };
        if area.width() == 0 || area.height() == 0 {
            return false;
        }
        lanes.draw(grid, area);
        self.lanes_touched = grid::union(self.lanes_touched, Some(area));
        true
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The obstacle layers.
    #[must_use]
    pub fn layers(&self) -> &[ObstacleLayer] {
        &self.layers
    }

    /// The newest grid.
    #[must_use]
    pub fn grid(&self) -> &Grid {
        &self.master
    }

    /// The inscribed radius (m) and the inflation, if any.
    #[must_use]
    pub fn inflation(&self) -> Option<&Inflation> {
        self.inflation.as_ref()
    }

    /// A new map. A fixed costmap takes its size, cells and corner (Nav2's static layer
    /// resizes the master), and its obstacle layers start again.
    pub fn set_map(&mut self, map: StaticMap) {
        if !self.settings.rolling {
            let g = &map.grid;
            self.master = Grid::new(
                (g.origin_x, g.origin_y),
                g.resolution,
                g.width,
                g.height,
                default(&self.settings),
            );
            self.resize_layers();
            if let Some((lanes, grid)) = self.lanes.as_mut() {
                *grid = self.master.like(0);
                let all = everything(grid);
                lanes.draw(grid, all);
            }
        }
        self.map = Some(map);
        self.whole = true;
    }

    /// Has it a map (or needs none)?
    #[must_use]
    pub fn ready(&self) -> bool {
        self.settings.rolling || self.map.is_some()
    }

    /// Forget every mark (Nav2's `ClearEntireCostmap`: each layer reset). The map comes
    /// back with the next update; the sensors' marks only as they see them again.
    pub fn clear(&mut self) {
        for layer in &mut self.layers {
            layer.grid.data.fill(layer.default);
        }
        self.master.data.fill(default(&self.settings));
        self.whole = true;
    }

    fn resize_layers(&mut self) {
        for layer in &mut self.layers {
            layer.grid = self.master.like(layer.default);
        }
    }

    /// One update, as Nav2's `updateMap`: the window follows `robot` (the car's pose in
    /// this costmap's frame), then the master starts from nothing, the map is written in
    /// (`map_from_frame`: this frame's pose in the map frame; without it the map is left
    /// out), each obstacle layer takes `observations[layer][source]` (a list) and adds its grid,
    /// and the inflation goes last. Returns what changed: a fixed costmap redoes only the
    /// area its layers touched, grown by the inflation's reach.
    pub fn update(
        &mut self,
        robot: &Pose2,
        map_from_frame: Option<&Pose2>,
        observations: &[Vec<Vec<&Observation>>],
    ) -> Changed {
        if !self.ready() {
            return Changed::Nothing;
        }
        if self.settings.rolling {
            // Nav2's getSizeInMetersX: (cells - 1 + 0.5) x resolution, half a cell short.
            let (w, h) = (
                (self.master.width as f64 - 0.5) * self.master.resolution,
                (self.master.height as f64 - 0.5) * self.master.resolution,
            );
            let (x, y) = (robot.x - w / 2.0, robot.y - h / 2.0);
            self.master.move_origin(x, y, default(&self.settings));
            for layer in &mut self.layers {
                layer.grid.move_origin(x, y, layer.default);
            }
        }
        let outline = footprint::placed(&self.settings.footprint, robot);
        let mut touched = None;
        for (i, layer) in self.layers.iter_mut().enumerate() {
            let mut seen = observations.get(i).cloned().unwrap_or_default();
            seen.resize(layer.sources.len(), Vec::new());
            touched = grid::union(touched, layer.update(&seen, &outline));
        }
        touched = grid::union(touched, self.lanes_touched.take());
        if self.settings.rolling || self.whole {
            self.whole = false;
            self.compose_all(map_from_frame);
            return Changed::All;
        }
        match touched {
            Some(touched) => Changed::Area(self.compose_area(touched)),
            None => Changed::Nothing,
        }
    }

    /// Every cell: the map, each layer, then the inflation.
    fn compose_all(&mut self, map_from_frame: Option<&Pose2>) {
        self.master.data.fill(default(&self.settings));
        // Not static_map(): it would borrow all of self, the master too.
        if let Some(map) = self.map.as_ref().filter(|_| self.settings.static_layer) {
            if self.settings.rolling {
                if let Some(t) = map_from_frame {
                    map.stamp(&mut self.master, t);
                }
            } else {
                self.master.data.copy_from_slice(&map.grid.data);
            }
        }
        for layer in &self.layers {
            layer.combine_into(&mut self.master);
        }
        if let Some(inflation) = &self.inflation {
            inflation.apply(&mut self.master);
        }
        if let Some((_, grid)) = &self.lanes {
            lanes::combine(&mut self.master.data, &grid.data);
        }
    }

    /// A fixed costmap's cells near `touched` only, as compose_all gives them: the touched
    /// cells grown by the inflation's reach can change, and they are worked out from the
    /// cells a reach further out (an obstacle further away inflates none of them).
    /// Returns the cells written.
    fn compose_area(&mut self, touched: Bounds) -> Bounds {
        let (w, h) = (self.master.width, self.master.height);
        // In cells, and one more: Inflation::apply's cut-off is cell_distance(radius).
        let reach = self
            .inflation
            .as_ref()
            .map_or(0, |i| self.master.cell_distance(i.inflation_radius) + 1);
        let changed = touched.grown(reach, w, h);
        let read = changed.grown(reach, w, h);
        let mut part = self.master.crop(read);
        match self.static_map() {
            Some(map) => part.data.copy_from_slice(&map.grid.crop(read).data),
            None => part.data.fill(default(&self.settings)),
        }
        for layer in &self.layers {
            layer.combine_area(&mut part, read);
        }
        if let Some(inflation) = &self.inflation {
            inflation.apply(&mut part);
        }
        if let Some((_, grid)) = &self.lanes {
            lanes::combine(&mut part.data, &grid.crop(read).data);
        }
        self.master.paste(&part, (read.x0, read.y0), changed);
        changed
    }

    /// The map, if the static layer is on and it has come.
    fn static_map(&self) -> Option<&StaticMap> {
        self.map.as_ref().filter(|_| self.settings.static_layer)
    }
}

/// Every cell of a grid.
fn everything(g: &Grid) -> Bounds {
    Bounds {
        x0: 0,
        y0: 0,
        x1: g.width,
        y1: g.height,
    }
}

fn default(s: &Settings) -> u8 {
    if s.track_unknown {
        NO_INFORMATION
    } else {
        FREE
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obstacle::{scan_points, Source};

    /// A fixed costmap on a 60 m x 40 m map at 0.25 m (walls, unknown corners), the global
    /// costmap's inflation, and a lidar layer.
    fn global(track_unknown: bool) -> Costmap {
        let (w, h) = (240usize, 160usize);
        let values: Vec<i8> = (0..w * h)
            .map(|i| {
                let (x, y) = (i % w, i / w);
                if x < 20 && y < 20 {
                    -1
                } else if y == 80 && !(100..140).contains(&x) || x == 60 && y > 100 {
                    100
                } else {
                    0
                }
            })
            .collect();
        let settings = Settings {
            rolling: false,
            width: 0.0,
            height: 0.0,
            resolution: 0.25,
            track_unknown,
            footprint: footprint::padded(
                &[(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)],
                0.05,
            ),
            inflation: Some((3.0, 1.0)),
            static_layer: true,
        };
        let empty = Grid::new((0.0, 0.0), 0.25, 0, 0, FREE);
        let source = Source {
            clearing: true,
            min_obstacle_height: -0.5,
            max_obstacle_height: 0.5,
            obstacle_max_range: 12.0,
            raytrace_max_range: 15.0,
            inf_is_valid: true,
            ..Source::named("lidar")
        };
        let layer = ObstacleLayer::new("lidar_layer", vec![source], &empty, track_unknown);
        let mut c = Costmap::new(settings, vec![layer]);
        let map = StaticMap::from_occupancy((-30.0, -20.0), 0.25, w, h, &values, track_unknown);
        c.set_map(map.expect("the map's size and data agree"));
        c
    }

    /// A post 5 m ahead of `pose` (or none), the scan's other rays inf: in the map frame.
    fn seen(pose: &Pose2, post: bool) -> Observation {
        let n = 360;
        let step = std::f64::consts::TAU / n as f64;
        let ranges: Vec<f32> = (0..n)
            .map(|i| {
                if post && (178..182).contains(&i) {
                    5.0
                } else {
                    f32::INFINITY
                }
            })
            .collect();
        let (s, c) = pose.yaw.sin_cos();
        let points = scan_points(-std::f64::consts::PI, step, 0.2, 30.0, &ranges, true)
            .into_iter()
            .map(|[x, y, z]| [pose.x + c * x - s * y, pose.y + s * x + c * y, z])
            .collect();
        Observation {
            origin: [pose.x, pose.y, 0.0],
            points,
        }
    }

    #[test]
    fn an_area_update_gives_the_same_cells_as_a_whole_one() {
        for track_unknown in [false, true] {
            let mut c = global(track_unknown);
            assert_eq!(c.update(&Pose2::IDENTITY, None, &[]), Changed::All);
            // The car drives along the map, a post ahead every other scan, its marks
            // staying where no ray clears them.
            for k in 0..12 {
                let pose = Pose2::new(
                    -20.0 + 3.5 * k as f64,
                    -5.0 + 0.4 * k as f64,
                    0.1 * k as f64,
                );
                let obs = seen(&pose, k % 2 == 0);
                let changed = c.update(&pose, None, &[vec![vec![&obs]]]);
                assert!(matches!(changed, Changed::Area(_)), "{changed:?}");
                let mut whole = c.clone();
                whole.compose_all(None);
                assert_eq!(
                    c.grid(),
                    whole.grid(),
                    "update {k}, track_unknown {track_unknown}"
                );
            }
        }
    }

    #[test]
    fn a_fixed_costmap_without_sensors_is_worked_out_once() {
        let mut c = global(true);
        c.layers.clear();
        assert_eq!(c.update(&Pose2::IDENTITY, None, &[]), Changed::All);
        let first = c.grid().clone();
        assert_eq!(c.update(&Pose2::IDENTITY, None, &[]), Changed::Nothing);
        c.clear();
        assert_eq!(c.update(&Pose2::IDENTITY, None, &[]), Changed::All);
        assert_eq!(c.grid(), &first);
    }

    #[test]
    fn the_area_reaches_the_inflation_round_what_was_touched() {
        let mut c = global(false);
        c.update(&Pose2::IDENTITY, None, &[]);
        let pose = Pose2::new(0.0, -10.0, 0.0);
        let Changed::Area(b) = c.update(&pose, None, &[vec![vec![&seen(&pose, true)]]]) else {
            panic!("an area was expected");
        };
        // The rays clear 15 m round the car, plus the 3 m inflation: well short of the map.
        let cells = |m: f64| (m / 0.25) as usize;
        assert!(b.width() >= cells(2.0 * 18.0) && b.width() < 240, "{b:?}");
        assert!(b.height() < 160, "{b:?}");
    }

    #[test]
    fn a_new_route_redraws_its_lanes_as_a_whole_update_would() {
        use crate::lanes::{Ramp, Road};
        let mut c = global(false);
        let road = Road {
            name: "east".to_owned(),
            points: vec![(-25.0, -10.0), (25.0, -10.0)],
            width: 7.0,
            lanes: 0,
            oneway: false,
            edge_inset: 0.5,
        };
        c.set_lanes(Lanes::new(vec![road], Ramp::default()).expect("a good road"));
        assert_eq!(c.update(&Pose2::IDENTITY, None, &[]), Changed::All);
        // No route yet: both lanes' centres free.
        let cost = |c: &Costmap, y: f64| c.grid().cost_at(0.0, y, 0);
        assert!(cost(&c, -11.5) <= 7 && cost(&c, -8.5) <= 7);
        // Driving east: the north lane is the oncoming one.
        assert!(c.set_route(&[(-20.0, -11.5), (20.0, -11.5)]));
        let changed = c.update(&Pose2::IDENTITY, None, &[]);
        assert!(matches!(changed, Changed::Area(_)), "{changed:?}");
        assert!(cost(&c, -11.5) <= 7);
        assert_eq!(cost(&c, -8.5), Ramp::default().wrong_lane);
        let mut whole = c.clone();
        whole.compose_all(None);
        assert_eq!(c.grid(), whole.grid());
        // The map's walls stay walls.
        assert_eq!(c.grid().cost_at(-20.0, 0.1, 0), crate::cost::LETHAL);
        assert!(
            !c.set_route(&[(-20.0, -11.5), (20.0, -11.5)]),
            "the same route again"
        );
    }
}

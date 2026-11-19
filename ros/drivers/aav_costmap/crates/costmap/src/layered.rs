//! Nav2's `LayeredCostmap`: the static map, then each obstacle layer, then the inflation,
//! into one grid. Rolling (the local costmap: a window round the car, in odom) or fixed
//! to the map (the global costmap, in map).
//!
//! A fixed costmap recomputes every cell only when its map arrives or it is cleared;
//! otherwise only the cells its obstacle layers touched, grown by the inflation's reach
//! (Nav2's `updateBounds` and `updateCosts`). A rolling one moves every update, so it is
//! recomputed whole (a 200 x 200 window: well under a millisecond).
//!
//! A fixed costmap is worked out in tiles, each on its own core (rayon, TODO 195): a tile
//! reads its cells and the inflation's reach round them, so the tiles agree with one pass
//! over the whole grid. Made of map chunks ([`Costmap::set_chunks`], TODO 175), its tiles
//! are the chunks, and a move of the window redoes only the chunks it changed.

use std::collections::BTreeMap;

use map_chunks::{Changes, ChunkKey, Detail, Store};
use rayon::prelude::*;

use crate::camera_lanes::{self, CameraLanes, Seen};
use crate::chunked::{copy_shared, stitch, ChunkError, ChunkSource, Window};
use crate::cost::{FREE, NO_INFORMATION};
use crate::footprint;
use crate::grid::{self, Bounds, Grid};
use crate::inflation::Inflation;
use crate::lanes::{self, Lanes, NO_ROAD};
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
    /// Its cells are [`NO_ROAD`] where no road reaches, so the camera's may fill them.
    lanes: Option<(Lanes, Grid)>,
    /// The lanes from the camera's lines (TODO 264), where the lane map has no road.
    camera: Option<CameraLayer>,
    /// The cells a new route redrew, for the next update.
    lanes_touched: Option<Bounds>,
    /// Made of map chunks (a fixed costmap only): the chunks held and where.
    chunks: Option<Chunked>,
    /// The cells a moved window must work out again, in the next update.
    redo: Option<Vec<Bounds>>,
}

/// The camera's lanes, their grid (the master's cells, [`NO_ROAD`] where nothing is
/// drawn) and the area the last picture drew.
#[derive(Debug, Clone)]
struct CameraLayer {
    lanes: CameraLanes,
    grid: Grid,
    drawn: Option<Bounds>,
}

/// A fixed costmap's chunks: where they come from, which are held, the route ahead.
#[derive(Debug, Clone)]
struct Chunked {
    source: ChunkSource,
    store: Store<Vec<u8>>,
    window: Option<Window>,
    route: Vec<(f64, f64)>,
}

/// What [`Costmap::follow_chunks`] did when the window moved.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Moved {
    /// The chunks read and dropped.
    pub changes: Changes,
    /// The window now.
    pub window: Window,
    /// Chunks whose cells the next update works out again (the rest kept theirs).
    pub redo: usize,
}

/// Cells along a tile's side when a whole fixed costmap is worked out: a 64 m chunk at
/// 0.25 m.
const TILE: usize = 256;
/// Tiles worked out at once: their parts are held at the same time (memory).
const TILES_AT_ONCE: usize = 64;

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
            camera: None,
            lanes_touched: None,
            chunks: None,
            redo: None,
        };
        c.resize_layers();
        c
    }

    /// Build the static map from map chunks, not a whole map (TODO 175): only the chunks
    /// round the car and along the route ahead are held, as the car goes
    /// ([`Costmap::follow_chunks`]). A map given later ([`Costmap::set_map`]) is ignored.
    /// A rolling costmap (the local one) holds the chunks round the car as its static map
    /// and stamps its window from them, at its own cells, as it would from a whole map.
    ///
    /// # Errors
    ///
    /// [`ChunkError::Resolution`] if a fixed costmap's cells aren't the fine chunks'.
    pub fn set_chunks(&mut self, source: ChunkSource) -> Result<(), ChunkError> {
        let fine = Detail::Fine.cell();
        if !self.settings.rolling && (self.settings.resolution - fine).abs() > 1e-9 {
            return Err(ChunkError::Resolution {
                costmap: self.settings.resolution,
                chunks: fine,
            });
        }
        self.chunks = Some(Chunked {
            source,
            store: Store::new(),
            window: None,
            route: Vec::new(),
        });
        self.map = None;
        self.whole = true;
        Ok(())
    }

    /// The route ahead (map frame, in driving order): its chunks for the next `ahead` m are
    /// held too. Only with chunks.
    pub fn set_ahead(&mut self, route: &[(f64, f64)]) {
        if let Some(c) = self.chunks.as_mut() {
            c.route = route.to_vec();
        }
    }

    /// Made of chunks: the window held now (None until the first
    /// [`Costmap::follow_chunks`]), and how many chunks.
    #[must_use]
    pub fn chunk_window(&self) -> Option<(Window, usize)> {
        let c = self.chunks.as_ref()?;
        c.window.map(|w| (w, c.store.len()))
    }

    /// Is it made of chunks?
    #[must_use]
    pub fn chunked(&self) -> bool {
        self.chunks.is_some()
    }

    /// Made of chunks: the window for the car at `car` (map frame) and the route ahead.
    /// If it moved, the new chunks are read (each on its own core), the static map is
    /// stitched from them, the cells shared with the old window are kept, and the next
    /// [`Costmap::update`] works out only the chunks the move changed. `None`: no chunks,
    /// or the window is where it was.
    ///
    /// # Errors
    ///
    /// [`ChunkError::File`]: a chunk couldn't be read; nothing changes.
    pub fn follow_chunks(&mut self, car: (f64, f64)) -> Result<Option<Moved>, ChunkError> {
        let Some(c) = self.chunks.as_mut() else {
            return Ok(None);
        };
        let window = c.source.window(car, &c.route);
        if c.window == Some(window) {
            return Ok(None);
        }
        let want: BTreeMap<ChunkKey, Detail> = window.keys().map(|k| (k, Detail::Fine)).collect();
        let track = self.settings.track_unknown;
        let source = &c.source;
        let changes = c.store.update(&want, |key, _| source.load(key, track))?;
        let side = source.side();
        let mut map = source.grid_for(&window, default(&self.settings));
        for key in window.keys() {
            if let Some((_, cells)) = c.store.get(key) {
                stitch(&mut map, &window, key, side, cells);
            }
        }
        if self.settings.rolling {
            // The window rolls on its own: the chunks are only the map it stamps from.
            self.map = Some(StaticMap { grid: map });
            c.window = Some(window);
            return Ok(Some(Moved {
                changes,
                window,
                redo: 0,
            }));
        }
        // A lanes layer's costs follow the newest route: redo every chunk then.
        let fresh = self.whole || self.lanes.is_some() || self.camera.is_some();
        let redo = window.to_redo(if fresh { None } else { c.window.as_ref() });
        let mut master = map.like(default(&self.settings));
        let all = everything(&master);
        copy_shared(&mut master, &self.master, all);
        for layer in &mut self.layers {
            let mut g = master.like(layer.default);
            copy_shared(&mut g, &layer.grid, all);
            layer.grid = g;
        }
        self.master = master;
        c.window = Some(window);
        self.redraw_lanes();
        self.lanes_touched = None;
        self.map = Some(StaticMap { grid: map });
        self.redo = Some(redo.iter().map(|&k| window.cells_of(k, side)).collect());
        Ok(Some(Moved {
            changes,
            window,
            redo: redo.len(),
        }))
    }

    /// The lanes layer on: the roads' costs go over the inflation (TODO 148). A rolling
    /// costmap ignores it: the lanes are the planner's, on the global costmap.
    pub fn set_lanes(&mut self, lanes: Lanes) {
        if self.settings.rolling {
            return;
        }
        let mut grid = self.master.like(NO_ROAD);
        let all = everything(&grid);
        lanes.draw_marked(&mut grid, all, NO_ROAD);
        self.lanes = Some((lanes, grid));
        self.whole = true;
    }

    /// The lanes from the camera on (TODO 264): what [`Costmap::see_lanes`] is given is
    /// drawn with the lane map's costs where the map has no road. A rolling costmap
    /// ignores it, as it does the lane map.
    pub fn set_camera_lanes(&mut self, lanes: CameraLanes) {
        if self.settings.rolling {
            return;
        }
        let grid = self.master.like(NO_ROAD);
        self.camera = Some(CameraLayer {
            lanes,
            grid,
            drawn: None,
        });
        self.whole = true;
    }

    /// The camera's lanes, if on.
    #[must_use]
    pub fn camera_lanes(&self) -> Option<&CameraLanes> {
        self.camera.as_ref().map(|c| &c.lanes)
    }

    /// One camera picture's lanes (`car`: base_link in this costmap's frame at its stamp),
    /// in place of the last; None forgets them (too old). Drawn in the next update.
    /// Returns true if any cell changed.
    pub fn see_lanes(&mut self, seen: Option<&Seen>, car: &Pose2) -> bool {
        let Some(c) = self.camera.as_mut() else {
            return false;
        };
        let old = c.drawn.take();
        if let Some(area) = old {
            camera_lanes::clear(&mut c.grid, area);
        }
        let new = seen.and_then(|s| c.lanes.draw(s, car, &mut c.grid));
        c.drawn = new;
        let changed = grid::union(old, new);
        self.lanes_touched = grid::union(self.lanes_touched, changed);
        changed.is_some()
    }

    /// Both lanes grids again at the master's size: the map's roads drawn whole, the
    /// camera's picture forgotten (the next one draws again).
    fn redraw_lanes(&mut self) {
        if let Some((lanes, grid)) = self.lanes.as_mut() {
            *grid = self.master.like(NO_ROAD);
            let all = everything(grid);
            lanes.draw_marked(grid, all, NO_ROAD);
        }
        if let Some(c) = self.camera.as_mut() {
            c.grid = self.master.like(NO_ROAD);
            c.drawn = None;
        }
    }

    /// The lanes' costs on the cells `read` (all of them: None), map first, then the
    /// camera's; None without a lanes layer.
    fn lane_costs(&self, read: Option<Bounds>) -> Option<Vec<u8>> {
        let cells = |g: &Grid| match read {
            Some(b) => g.crop(b).data,
            None => g.data.clone(),
        };
        match (&self.lanes, &self.camera) {
            (None, None) => None,
            (Some((_, map)), None) => Some(
                cells(map)
                    .into_iter()
                    .map(|m| lanes::merged(m, NO_ROAD))
                    .collect(),
            ),
            (None, Some(c)) => Some(
                cells(&c.grid)
                    .into_iter()
                    .map(|v| lanes::merged(NO_ROAD, v))
                    .collect(),
            ),
            (Some((_, map)), Some(c)) => Some(
                cells(map)
                    .into_iter()
                    .zip(cells(&c.grid))
                    .map(|(m, v)| lanes::merged(m, v))
                    .collect(),
            ),
        }
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
        lanes.draw_marked(grid, area, NO_ROAD);
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
        if self.chunks.is_some() {
            return;
        }
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
            self.redraw_lanes();
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
            self.redo = None;
            self.compose_all(map_from_frame);
            return Changed::All;
        }
        if let Some(mut tiles) = self.redo.take() {
            // A moved window: its changed chunks, and what the sensors touched.
            if let Some(t) = touched {
                tiles.push(t.grown(self.reach(), self.master.width, self.master.height));
            }
            self.compose_tiles(&tiles);
            return Changed::All;
        }
        match touched {
            Some(touched) => Changed::Area(self.compose_area(touched)),
            None => Changed::Nothing,
        }
    }

    /// Every cell: the map, each layer, then the inflation. A fixed costmap in tiles, each
    /// on its own core; a rolling one in one pass.
    fn compose_all(&mut self, map_from_frame: Option<&Pose2>) {
        if self.settings.rolling {
            self.compose_whole(map_from_frame);
            return;
        }
        let (w, h) = (self.master.width, self.master.height);
        let tiles: Vec<Bounds> = (0..h)
            .step_by(TILE)
            .flat_map(|y0| {
                (0..w).step_by(TILE).map(move |x0| Bounds {
                    x0,
                    y0,
                    x1: (x0 + TILE).min(w),
                    y1: (y0 + TILE).min(h),
                })
            })
            .collect();
        self.compose_tiles(&tiles);
    }

    /// Every cell in one pass over the whole grid: the rolling window's way, and what the
    /// tiles must agree with.
    fn compose_whole(&mut self, map_from_frame: Option<&Pose2>) {
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
        if let Some(costs) = self.lane_costs(None) {
            lanes::combine(&mut self.master.data, &costs);
        }
    }

    /// A fixed costmap's cells near `touched` only, as compose_all gives them: the touched
    /// cells grown by the inflation's reach can change, and they are worked out from the
    /// cells a reach further out (an obstacle further away inflates none of them).
    /// Returns the cells written.
    fn compose_area(&mut self, touched: Bounds) -> Bounds {
        let (w, h) = (self.master.width, self.master.height);
        let changed = touched.grown(self.reach(), w, h);
        let (part, read) = self.compose_part(changed);
        self.master.paste(&part, (read.x0, read.y0), changed);
        changed
    }

    /// The inflation's reach in cells, and one more: Inflation::apply's cut-off is
    /// cell_distance(radius).
    fn reach(&self) -> usize {
        self.inflation
            .as_ref()
            .map_or(0, |i| self.master.cell_distance(i.inflation_radius) + 1)
    }

    /// Work out each of `tiles` on its own core (rayon), then write them into the grid.
    /// Tiles may overlap: where they do, they agree.
    fn compose_tiles(&mut self, tiles: &[Bounds]) {
        for batch in tiles.chunks(TILES_AT_ONCE) {
            let parts: Vec<(Grid, Bounds, Bounds)> = batch
                .par_iter()
                .filter(|t| t.width() > 0 && t.height() > 0)
                .map(|&t| {
                    let (part, read) = self.compose_part(t);
                    (part, read, t)
                })
                .collect();
            for (part, read, t) in parts {
                self.master.paste(&part, (read.x0, read.y0), t);
            }
        }
    }

    /// The cells of `changed` as compose_all gives them, worked out from the cells a reach
    /// further out (an obstacle further away inflates none of them): the grid round
    /// `changed`, and where it starts in the master.
    fn compose_part(&self, changed: Bounds) -> (Grid, Bounds) {
        let (w, h) = (self.master.width, self.master.height);
        let read = changed.grown(self.reach(), w, h);
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
        if let Some(costs) = self.lane_costs(Some(read)) {
            lanes::combine(&mut part.data, &costs);
        }
        (part, read)
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
                whole.compose_whole(None);
                assert_eq!(
                    c.grid(),
                    whole.grid(),
                    "update {k}, track_unknown {track_unknown}"
                );
            }
        }
    }

    #[test]
    fn chunks_with_a_lidar_layer_agree_with_one_pass_as_the_window_moves() {
        // A 320 m map in 5 x 5 chunks of 64 m: walls every 30 m. The car drives east with
        // a post ahead every other scan; after each update the grid must be what one pass
        // over the window's static map and layers gives.
        let (n, res) = (1280usize, 0.25);
        let values: Vec<i8> = (0..n * n)
            .map(|i| {
                if (i % n) % 120 == 0 && (i / n) % 7 != 0 {
                    100
                } else {
                    0
                }
            })
            .collect();
        let dir = std::env::temp_dir().join(format!("costmap-layers-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let grid = map_chunks::Grid::new(map_chunks::Point::new(-160.0, -160.0), 64.0).unwrap();
        let whole = map_chunks::WholeMap {
            origin: map_chunks::Point::new(-160.0, -160.0),
            resolution: res,
            width: n,
            height: n,
            data: &values,
        };
        map_chunks::files::write_chunks(&grid, &whole, &dir).unwrap();
        let mut c = global(true);
        c.set_chunks(ChunkSource::open(&dir, 100.0, 0).unwrap())
            .unwrap();
        for k in 0..40 {
            let pose = Pose2::new(-150.0 + 7.0 * f64::from(k), -3.0, 0.0);
            c.follow_chunks((pose.x, pose.y)).unwrap();
            let obs = seen(&pose, k % 2 == 0);
            c.update(&pose, None, &[vec![vec![&obs]]]);
            let mut one = c.clone();
            one.compose_whole(None);
            assert_eq!(c.grid(), one.grid(), "step {k}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_rolling_window_from_the_chunks_round_the_car_is_the_one_from_the_whole_map() {
        // The same 320 m map, whole and in 64 m chunks; a 20 m local window at 0.1 m cells,
        // in odom, odom turned and moved off map. The car drives across chunk edges.
        let (n, res) = (1280usize, 0.25);
        let values: Vec<i8> = (0..n * n)
            .map(|i| {
                let (x, y) = (i % n, i / n);
                if x % 90 == 0 && y % 5 != 0 || (x * 7 + y * 3) % 401 == 0 {
                    100
                } else if (x / 40 + y / 40) % 9 == 0 {
                    -1
                } else {
                    0
                }
            })
            .collect();
        let dir = std::env::temp_dir().join(format!("costmap-local-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let grid = map_chunks::Grid::new(map_chunks::Point::new(-160.0, -160.0), 64.0).unwrap();
        let whole = map_chunks::WholeMap {
            origin: map_chunks::Point::new(-160.0, -160.0),
            resolution: res,
            width: n,
            height: n,
            data: &values,
        };
        map_chunks::files::write_chunks(&grid, &whole, &dir).unwrap();
        let settings = Settings {
            rolling: true,
            width: 20.0,
            height: 20.0,
            resolution: 0.1,
            track_unknown: true,
            footprint: footprint::padded(
                &[(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)],
                0.05,
            ),
            inflation: Some((1.0, 3.0)),
            static_layer: true,
        };
        let mut from_map = Costmap::new(settings.clone(), Vec::new());
        from_map.set_map(
            StaticMap::from_occupancy((-160.0, -160.0), res, n, n, &values, true).unwrap(),
        );
        let mut from_chunks = Costmap::new(settings, Vec::new());
        from_chunks
            .set_chunks(ChunkSource::open(&dir, 0.0, 0).unwrap())
            .unwrap();
        let map_from_odom = Pose2::new(3.0, -2.0, 0.3);
        let mut moves = 0;
        for k in 0..40 {
            let robot = Pose2::new(-140.0 + 7.0 * f64::from(k), 5.0, 0.2);
            let car = map_from_odom.apply((robot.x, robot.y));
            if from_chunks.follow_chunks(car).unwrap().is_some() {
                moves += 1;
            }
            from_map.update(&robot, Some(&map_from_odom), &[]);
            from_chunks.update(&robot, Some(&map_from_odom), &[]);
            assert_eq!(from_chunks.grid(), from_map.grid(), "step {k}");
        }
        assert!(moves >= 4, "{moves}: the chunks held follow the car");
        // 3 x 3 chunks held, never the whole map.
        let (window, held) = from_chunks.chunk_window().unwrap();
        assert_eq!((window.across(), window.up(), held), (3, 3, 9));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_tiles_on_their_cores_agree_with_one_pass() {
        // 700 x 600 cells: 3 x 3 tiles, the last ones cut short; walls across tile edges.
        let (w, h) = (700usize, 600usize);
        let values: Vec<i8> = (0..w * h)
            .map(|i| {
                let (x, y) = (i % w, i / w);
                if x < 30 && y < 30 {
                    -1
                } else if y == 255 || x == 258 && y > 100 || (x + y) % 97 == 0 {
                    100
                } else {
                    0
                }
            })
            .collect();
        let mut c = global(true);
        c.layers.clear();
        let map = StaticMap::from_occupancy((-30.0, -20.0), 0.25, w, h, &values, true);
        c.set_map(map.expect("the map's size and data agree"));
        assert_eq!(c.update(&Pose2::IDENTITY, None, &[]), Changed::All);
        let mut whole = c.clone();
        whole.compose_whole(None);
        assert_eq!(c.grid(), whole.grid());
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
        whole.compose_whole(None);
        assert_eq!(c.grid(), whole.grid());
        // The map's walls stay walls.
        assert_eq!(c.grid().cost_at(-20.0, 0.1, 0), crate::cost::LETHAL);
        assert!(
            !c.set_route(&[(-20.0, -11.5), (20.0, -11.5)]),
            "the same route again"
        );
    }
    #[test]
    fn the_camera_draws_lanes_where_the_map_has_none() {
        use crate::camera_lanes::{CameraSettings, Paint, SeenLine};
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
        c.set_camera_lanes(
            CameraLanes::new(Ramp::default(), CameraSettings::default()).expect("good"),
        );
        c.set_route(&[(-20.0, -11.5), (20.0, -11.5)]);
        c.update(&Pose2::IDENTITY, None, &[]);
        let line = |y: f64| SeenLine {
            a: 0.0,
            b: 0.0,
            c: y,
            paint: Paint::Unknown,
        };
        let lane = Seen {
            left: Some(line(1.75)),
            right: Some(line(-1.75)),
            width: 3.5,
        };
        let cost = |c: &Costmap, x: f64, y: f64| c.grid().cost_at(x, y, 0);
        let r = Ramp::default();
        // Off the map's roads: the camera's lanes, the car keeping right.
        let car = Pose2::new(10.0, 10.0, 0.0);
        assert!(c.see_lanes(Some(&lane), &car));
        assert!(matches!(
            c.update(&Pose2::IDENTITY, None, &[]),
            Changed::Area(_)
        ));
        assert!(cost(&c, 15.0, 10.0) <= 7);
        assert_eq!(cost(&c, 15.0, 13.5), r.wrong_lane);
        let mut whole = c.clone();
        whole.compose_whole(None);
        assert_eq!(c.grid(), whole.grid());
        // On a map road the map wins: the car in the oncoming lane sees its own lines,
        // but the map knows which way the route drives.
        let wrong = Pose2::new(0.0, -8.5, 0.0);
        c.see_lanes(Some(&lane), &wrong);
        c.update(&Pose2::IDENTITY, None, &[]);
        assert_eq!(cost(&c, 5.0, -8.5), r.wrong_lane);
        assert!(cost(&c, 5.0, -11.5) <= 7);
        // The first picture's lanes are gone with the second; none forgets them all.
        assert_eq!(cost(&c, 15.0, 13.5), 0);
        assert!(c.see_lanes(None, &wrong));
        c.update(&Pose2::IDENTITY, None, &[]);
        let mut whole = c.clone();
        whole.compose_whole(None);
        assert_eq!(c.grid(), whole.grid());
    }
}

//! Ben's layout, in egui. Left: the scene, over the console. Right: the debug view of what the
//! lidar sees from the car with NDT's state, the one map you click on, keys and algorithms.

use std::time::Instant;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiTextureHandle};
use debug_core::{
    cubes::{Band, CubeScan, Metres},
    flat::{ground_the_flat, FlatRule},
    ndt::{MapCells, NdtStatus},
    orbit::Orbit,
    swap::Tracker,
};
use hud_core::{
    ahead::{self, Ahead, PlanSource, Summary, Tone},
    gate::Light,
    origin::Origins,
    things::Thing,
    vehicle::VehicleStatus,
};
use map_core::{
    geom::{Point, Pose},
    grid::{Grid, Palette},
    view::{Mode as MapMode, ScreenPoint, Viewport as MapViewport},
};

use crate::{
    chunks,
    feed::{self, Heard, Level, Update},
    map_nav::{Gesture, MapNav},
    set_viewport, CubesCamera, PictureImage, Source,
};

/// Height of the status strip, in points.
const STRIP: f32 = 48.0;
/// The share of the window's width the left column takes.
const LEFT_SHARE: f32 = 0.58;
/// The share of the left column's height the scene takes.
const SCENE_SHARE: f32 = 0.68;
/// The right column, top to bottom: the cubes, then the map; the rest is keys.
const CUBES_SHARE: f32 = 0.34;
const MAP_SHARE: f32 = 0.44;
/// The driver's view at rest: a little down, as the lidar's eyes are level with a wall's top.
const LOOK_DOWN: [f32; 2] = [0.0, -0.18];
/// The view from the car, top to bottom, radians: wide, as Ben preferred. The wheel zooms.
pub const DRIVER_FOV: f32 = 1.15;
/// Cubes lower than this are the ground's waves, kerbs and grass: not compared with NDT's map.
pub const WAVES_TOP: f32 = 0.45;
/// NDT's picture on the flat map: metres per pixel, and the most pixels a side.
const FOOTPRINT_CELL: f32 = 0.5;
const MAX_FOOTPRINT: usize = 4096;
/// Log lines kept for the console.
const CONSOLE_LINES: usize = 500;
/// The most cubes drawn on the map. A real scan has about 2,000; beyond that it's a blur.
const MAX_MAP_CUBES: usize = 4_000;
/// A wall's colour, on the map and the costmap alike: when they differed, a wall flashed each
/// time Nav2 cleared or sent its costmap again (Ben, 5 Nov).
const WALL: [u8; 3] = [172, 150, 180];
/// The map's colours: a dark floor and walls, with unknown nearly black.
const MAP_COLOURS: Palette = Palette {
    free: [32, 32, 36, 255],
    occupied: [WALL[0], WALL[1], WALL[2], 255],
    unknown: [12, 12, 14, 255],
};
/// The costmap's: the wall's colour, fading out as the danger falls.
const COST_COLOURS: Palette = Palette {
    free: [WALL[0], WALL[1], WALL[2], 0],
    occupied: [WALL[0], WALL[1], WALL[2], 255],
    unknown: [0, 0, 0, 0],
};

/// The cubes view's colours (Ben, 5 Nov): one set of cells, going dark to light purple.
pub const NDT_DARK_RGB: [u8; 3] = [54, 36, 92];
pub const NDT_LIT_RGB: [u8; 3] = [216, 160, 255];
pub const NEW_CUBE_RGB: [u8; 3] = [255, 158, 40];

/// The cubes' height bands, as the debug HUD's legend says them.
pub const BAND_RGB: [[u8; 3]; 6] = [
    [58, 66, 78],
    [34, 96, 128],
    [42, 136, 158],
    [62, 178, 188],
    [116, 212, 212],
    [204, 244, 240],
];
const BAND_LEGEND: [([u8; 3], &str); 6] = [
    (BAND_RGB[0], "ground"),
    (BAND_RGB[1], "< 0.75 m"),
    (BAND_RGB[2], "< 1.25 m"),
    (BAND_RGB[3], "< 1.75 m"),
    (BAND_RGB[4], "< 2.5 m"),
    (BAND_RGB[5], "higher"),
];

/// One painted line of the lane map, on the map.
#[derive(Clone, Debug, PartialEq)]
pub struct LaneLine {
    pub points: Vec<Point>,
    pub look: LaneLook,
}

/// How a lane map's line is drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaneLook {
    Solid,
    Dashed,
    Stop,
    Curb,
}

/// What's drawn. The defaults are the quiet view Ben asked for.
#[derive(Clone, Copy, Debug)]
pub struct Layers {
    /// The ground's cubes: the road and the lidar's rings on flat raised ground (key G;
    /// off: Ben, 17 Nov, "lets get rid of the waves the lidar made").
    pub ground: bool,
    /// The lane map's lines on the map (`--lanes`; key L).
    pub lanes: bool,
    pub nav2_cubes: bool,
    pub ndt_cubes: bool,
    pub costmap: bool,
    pub ndt_map: bool,
    pub scan: bool,
    pub plan: bool,
    /// The squiggly line: the trace, the plan ahead, the stop bar and their words (key T).
    pub ahead: bool,
    /// The things near the car, each box in its kind's colour, with its name (key K).
    pub things: bool,
    /// The chunks loaded and their saved maps (`--chunks`; key H), and the route.
    pub chunks: bool,
}

impl Default for Layers {
    fn default() -> Self {
        Self {
            ground: false,
            lanes: true,
            nav2_cubes: false,
            ndt_cubes: false,
            costmap: true,
            ndt_map: true,
            scan: true,
            plan: true,
            ahead: true,
            things: true,
            chunks: true,
        }
    }
}

/// Where a click on the map has got to.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Goal {
    #[default]
    None,
    /// Clicked, waiting for DRIVE HERE.
    Picked(Point),
    /// DRIVE HERE pressed, waiting for CONFIRM.
    Armed(Point),
    /// Confirmed: pretend the navigator is driving there.
    Driving(Point),
}

impl Goal {
    fn point(self) -> Option<Point> {
        match self {
            Self::None => None,
            Self::Picked(p) | Self::Armed(p) | Self::Driving(p) => Some(p),
        }
    }
}

/// The corner's two tabs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Keys,
    Algorithms,
}

/// One grid, and its picture once it's been made into a texture.
#[derive(Default)]
struct GridLayer {
    grid: Option<Grid>,
    texture: Option<egui::TextureHandle>,
    fresh: bool,
}

impl GridLayer {
    /// Nav2 sends the whole grid again and again. An unchanged one costs nothing: making a
    /// picture of a campus takes long enough to freeze the window, and makes it flash.
    fn set(&mut self, grid: Grid) {
        if self.grid.as_ref() != Some(&grid) {
            self.grid = Some(grid);
            self.fresh = true;
        }
    }

    /// Paints the grid at its place on the map, making a new texture if it changed.
    fn paint(
        &mut self,
        ui: &egui::Ui,
        name: &str,
        palette: &Palette,
        painter: &egui::Painter,
        to: impl Fn(Point) -> egui::Pos2,
    ) {
        let Some(grid) = &self.grid else { return };
        if self.fresh || self.texture.is_none() {
            let image = egui::ColorImage::from_rgba_unmultiplied(
                [grid.info.width as usize, grid.info.height as usize],
                &grid.to_rgba(palette),
            );
            // The same texture, new contents: a new one would be blank for a frame.
            match &mut self.texture {
                Some(texture) => texture.set(image, egui::TextureOptions::NEAREST),
                None => {
                    self.texture = Some(ui.ctx().load_texture(
                        name,
                        image,
                        egui::TextureOptions::NEAREST,
                    ));
                }
            }
            self.fresh = false;
        }
        if let Some(texture) = &self.texture {
            let bounds = grid.info.bounds();
            let top_left = to(Point::new(bounds.min.x, bounds.max.y));
            let bottom_right = to(Point::new(bounds.max.x, bounds.min.y));
            painter.image(
                texture.id(),
                egui::Rect::from_min_max(top_left, bottom_right),
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }
    }
}

/// Everything the feed has said.
#[derive(Default)]
pub struct WorldNews {
    pub cubes: CubeScan,
    pub road: CubeScan,
    pub car_odom: Pose,
    pub car_map: Pose,
    ndt_index: Option<feed::MapIndex>,
    ndt_picture: Option<egui::TextureHandle>,
    pub ndt: Option<MapCells>,
    /// How the newest cubes sit on NDT's map.
    pub map_match: feed::MapMatch,
    ndt_status: Option<NdtStatus>,
    ndt_pose: Option<Pose>,
    pub picture_size: Option<[u32; 2]>,
    pub plan: Vec<Point>,
    /// The road ahead, on the map (drawn with the plan, key P).
    pub road_ahead: feed::RoadAhead,
    /// The lane map's painted lines (`--lanes`, a Lanelet2 .osm), drawn with the road.
    pub lanes: Vec<LaneLine>,
    /// The map's chunks (`--chunks`, a folder with chunks.yaml): those loaded for the car
    /// and its route (`chunks::loaded`, as the planner and NDT load them, TODO 175), each
    /// with its saved map (key H).
    pub chunks: Option<map_chunks::Grid>,
    /// The folder `--chunks` named: the saved maps are read from it as chunks load.
    pub chunk_folder: Option<std::path::PathBuf>,
    saved_maps: chunks::SavedMaps,
    saved_pictures:
        std::collections::BTreeMap<(map_chunks::ChunkKey, map_chunks::Detail), egui::TextureHandle>,
    /// The street route (`/drivers/nav/route/path`, map): the zoomed-out level's line.
    pub route: Vec<Point>,
    /// The squiggly line, in base_link, and when its trace and numbers were last heard.
    pub ahead: Ahead,
    trace_heard: Option<Instant>,
    /// The things near the car (odom), named by the camera, and when they were last heard.
    things: Vec<Thing>,
    things_heard: Option<Instant>,
    /// When the newest plan came: a navigator replans while it drives a goal.
    plan_heard: Option<Instant>,
    summary_heard: Option<Instant>,
    map: GridLayer,
    costmap: GridLayer,
    origins: Origins,
    vehicle: Option<VehicleStatus>,
    gate: Heard,
    watchdog: Heard,
    /// Who is driving (the command mux's state).
    driving: Heard,
    console: Vec<(Level, String)>,
}

impl WorldNews {
    /// The line the coarse chunks follow: the street route if there is one, else the plan.
    pub fn chunk_route(&self) -> &[Point] {
        if self.route.len() >= 2 {
            &self.route
        } else {
            &self.plan
        }
    }

    /// A line for the console, from the window itself.
    pub fn note(&mut self, line: String) {
        self.push_line(Level::Info, line);
    }

    fn push_line(&mut self, level: Level, line: String) {
        self.console.push((level, line));
        if self.console.len() > CONSOLE_LINES {
            let extra = self.console.len() - CONSOLE_LINES;
            self.console.drain(..extra);
        }
    }

    /// Moves the newest cubes onto the map and sees which NDT cells they light.
    fn match_cubes(&mut self) {
        let Some(cells) = &self.ndt else { return };
        // The cubes are in odom; the car's two newest poses say where odom is on the map.
        let (car_odom, car_map) = (self.car_odom, self.car_map);
        let cubes: Vec<[f32; 3]> = self
            .cubes
            .cubes
            .iter()
            .map(|c| {
                let p = feed::odom_to_map([c.centre[0], c.centre[1]], car_odom, car_map);
                [p.x as f32, p.y as f32, c.centre[2]]
            })
            .collect();
        let index = self
            .ndt_index
            .get_or_insert_with(|| feed::MapIndex::new(&cells.centres, cells.size));
        self.map_match = index.match_cubes(&cubes, WAVES_TOP);
    }

    /// The voxels node's cubes on flat raised ground (the lidar's rings on the campus's
    /// grass, a kerb up from the road) become ground, against the newest road
    /// (`debug_core::flat`). Each cube's band starts again from its height, so a newer
    /// road can give one back.
    fn ground_the_waves(&mut self) {
        for c in &mut self.cubes.cubes {
            c.band = Band::of(Metres(f64::from(c.centre[2])), Metres(0.0));
        }
        ground_the_flat(&mut self.cubes.cubes, &self.road.cubes, FlatRule::default());
    }

    /// One piece of news. The picture isn't here: it goes straight into its image.
    pub fn apply(&mut self, update: Update, now: Instant) {
        match update {
            Update::Cubes(scan) => {
                self.cubes = scan;
                self.ground_the_waves();
                self.match_cubes();
            }
            Update::Road(mut scan) => {
                // The voxels node's road is ground, however high it sits.
                for c in &mut scan.cubes {
                    c.band = Band::Ground;
                }
                self.road = scan;
                self.ground_the_waves();
            }
            Update::CarInOdom(pose) => self.car_odom = pose,
            Update::CarInMap(pose) => self.car_map = pose,
            Update::Picture(_) => {}
            Update::Log(level, line) => self.push_line(level, line),
            Update::Ndt(status) => self.ndt_status = Some(status),
            Update::NdtPose(pose) => self.ndt_pose = Some(pose),
            Update::Plan(points) => {
                self.plan = points;
                self.plan_heard = Some(now);
            }
            Update::RoadAhead(road) => self.road_ahead = road,
            Update::Map(grid) => self.map.set(grid),
            Update::Costmap(grid) => self.costmap.set(grid),
            Update::Origin(announcement) => self.origins.apply(announcement),
            Update::Vehicle(status) => self.vehicle = Some(status),
            Update::Gate(text) => self.gate.set(text, now),
            Update::Watchdog(text) => self.watchdog.set(text, now),
            Update::Driving(text) => self.driving.set(text, now),
            Update::Trace(points) => {
                self.ahead.trace = points;
                self.trace_heard = Some(now);
            }
            Update::PlanAhead(points) => self.ahead.plan = points,
            Update::StopPoint(stop) => self.ahead.stop = Some(stop),
            Update::Ahead(summary) => {
                self.ahead.summary = Some(summary);
                self.summary_heard = Some(now);
            }
            Update::Things(things) => {
                self.things = things;
                self.things_heard = Some(now);
            }
            Update::Route(points) => self.route = points,
        }
    }

    /// The goal someone else sent (sim-check, the tools, RViz, the map window): the end of a
    /// plan heard within [`GOAL_PLAN_FRESH`]. Every navigator, Nav2's or ours, replans while
    /// it drives (every 1 s to a pose, 3 s through poses) and stops when the goal ends.
    pub fn goal_elsewhere(&self, now: Instant) -> Option<Point> {
        let heard = self.plan_heard?;
        (now.saturating_duration_since(heard) <= GOAL_PLAN_FRESH)
            .then(|| self.plan.last().copied())
            .flatten()
    }

    /// The things while they are fresh; none once `things` has been quiet 2 s.
    pub fn things_now(&self, now: Instant) -> &[Thing] {
        match self.things_heard {
            Some(at) if now.saturating_duration_since(at) <= feed::STALE_AFTER => &self.things,
            _ => &[],
        }
    }

    /// The squiggly line while ego's trace is fresh, with its numbers while they are.
    pub fn ahead_now(&self, now: Instant) -> Option<Ahead> {
        let fresh = |at: Option<Instant>| {
            at.is_some_and(|at| now.saturating_duration_since(at) <= feed::STALE_AFTER)
        };
        if !fresh(self.trace_heard) {
            return None;
        }
        let mut ahead = self.ahead.clone();
        if !fresh(self.summary_heard) {
            ahead.summary = None;
        }
        Some(ahead)
    }
}

/// How long a plan says a goal is still being driven: more than the slowest replan (3 s).
pub const GOAL_PLAN_FRESH: std::time::Duration = std::time::Duration::from_secs(5);

/// The squiggly line's colours: the plan faint, the trace by how well it keeps to the plan.
pub const AHEAD_PLAN_RGB: [u8; 3] = [150, 156, 164];
pub const AHEAD_NO_PLAN_RGB: [u8; 3] = [245, 245, 245];
pub const AHEAD_STOP_RGB: [u8; 3] = [255, 255, 255];

/// The trace's colour: green, amber or red by the gap; white with no plan or no numbers.
pub fn trace_rgb(summary: Option<&Summary>) -> [u8; 3] {
    match summary {
        Some(s) if s.plan != PlanSource::None && s.gap.is_some() => tone_rgb(s.grade.into()),
        _ => AHEAD_NO_PLAN_RGB,
    }
}

/// A word's colour by its meaning (`hud_core::ahead::Tone`).
pub fn tone_rgb(tone: Tone) -> [u8; 3] {
    match tone {
        Tone::Plain => [235, 235, 235],
        Tone::Good => [45, 210, 110],
        Tone::Warn => [240, 190, 70],
        Tone::Bad => [255, 95, 95],
        Tone::Dim => [150, 150, 150],
    }
}

/// Half the car's width, for the stop bar, metres: the scene's car box is 1.2 m wide.
pub const STOP_BAR_HALF: f64 = 0.6;

/// Whose eyes the main view has.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Eyes {
    /// From the car, looking where it drives: what it sees.
    Driver,
    /// Behind the car and above it, through the debug HUD's `Orbit`: the view that's easiest to
    /// read (Ben, 5 Nov), so the one it opens with.
    #[default]
    Orbit,
}

impl Eyes {
    fn label(self) -> &'static str {
        match self {
            Self::Driver => "FROM THE CAR",
            Self::Orbit => "FROM BEHIND",
        }
    }
}

/// All the window's own state.
#[derive(Resource)]
pub struct UiState {
    pub layers: Layers,
    pub orbit: Orbit,
    /// Whose eyes the main view has.
    pub camera: Eyes,
    /// Driver's view: how far the head is turned and tilted, radians.
    pub look: [f32; 2],
    pub driver_fov: f32,
    /// Where the lidar sits on the car, in base_link: the driver's eyes.
    pub eyes: [f32; 3],
    pub world: WorldNews,
    /// The map panel's view: following the car, the whole map, or free (`map_nav`).
    pub map: MapNav,
    /// F was pressed: the map panel follows the car, or shows the whole map, next frame.
    map_follow_key: bool,
    goal: Goal,
    tab: Tab,
    console_floor: Level,
    styled: bool,
    picks: [usize; ALGORITHMS.len()],
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            layers: Layers::default(),
            orbit: Orbit::default(),
            camera: Eyes::default(),
            look: LOOK_DOWN,
            driver_fov: DRIVER_FOV,
            eyes: [0.0, 0.0, 1.5],
            world: WorldNews::default(),
            map: MapNav::default(),
            map_follow_key: false,
            goal: Goal::default(),
            tab: Tab::default(),
            console_floor: Level::Info,
            styled: false,
            picks: [0; ALGORITHMS.len()],
        }
    }
}

/// The swap list's rows: today's real choices (`docs/designs/2026-11-04_one-car.md`).
const ALGORITHMS: [(&str, &[&str]); 5] = [
    (
        "Planner",
        &[
            "lattice",
            "hybrid",
            "smac2d",
            "navfn",
            "theta*",
            "diffusion",
        ],
    ),
    (
        "Follower",
        &[
            "mppi",
            "rpp",
            "pure_pursuit",
            "stanley",
            "lqr",
            "mpc",
            "mpc_full",
        ],
    ),
    (
        "Where am I",
        &["ekf", "ndt", "amcl", "both", "ndt_ekf", "kiss_ekf", "kiss"],
    ),
    ("Obstacles", &["flat scan", "cubes"]),
    ("Speed loop", &["open", "pi", "smooth", "lqi"]),
];

/// The whole window, once a frame.
pub fn draw(
    mut contexts: EguiContexts,
    mut source: ResMut<Source>,
    picture: Res<PictureImage>,
    mut state: ResMut<UiState>,
    window: Single<&Window>,
    mut camera: Single<(&mut Camera, &GlobalTransform), With<CubesCamera>>,
) -> Result {
    let picture_id = contexts.add_image(EguiTextureHandle::Weak(picture.0.id()));
    let ctx = contexts.ctx_mut()?.clone();
    let state = &mut *state;

    if !state.styled {
        apply_style(&ctx);
        state.styled = true;
    }
    read_keys(&ctx, state);

    let whole = ctx.viewport_rect();
    let body = egui::Rect::from_min_max(whole.min + egui::vec2(0.0, STRIP), whole.max);
    let left_width = body.width() * LEFT_SHARE;
    let left = egui::Rect::from_min_size(body.min, egui::vec2(left_width, body.height()));
    let right = egui::Rect::from_min_max(egui::pos2(left.right(), body.top()), body.max);

    let picture_rect = egui::Rect::from_min_size(
        left.min,
        egui::vec2(left.width(), left.height() * SCENE_SHARE),
    );
    let console_rect = egui::Rect::from_min_max(picture_rect.left_bottom(), left.max);
    let cubes_rect = egui::Rect::from_min_size(
        right.min,
        egui::vec2(right.width(), right.height() * CUBES_SHARE),
    );
    let map_rect = egui::Rect::from_min_size(
        cubes_rect.left_bottom(),
        egui::vec2(right.width(), right.height() * MAP_SHARE),
    );
    let corner_rect = egui::Rect::from_min_max(map_rect.left_bottom(), right.max);

    let mut root = egui::Ui::new(
        ctx.clone(),
        "window".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(whole),
    );

    let strip_rect = egui::Rect::from_min_size(whole.min, egui::vec2(whole.width(), STRIP));
    let feed = source.0.as_mut();
    panel(&mut root, strip_rect, |ui| status_strip(ui, state, feed));
    panel(&mut root, picture_rect, |ui| {
        picture_panel(ui, picture_id, state.world.picture_size);
    });
    panel(&mut root, console_rect, |ui| console_panel(ui, state));
    cubes_panel(&mut root, cubes_rect, state);
    ndt_line(&root, cubes_rect, &state.world, state.layers.ndt_cubes);
    if state.layers.things {
        let (eyes, at) = (&*camera.0, camera.1);
        thing_names(&root, cubes_rect, &state.world, |p| {
            eyes.world_to_viewport(at, p).ok()
        });
    }
    if state.layers.ahead {
        let now = Instant::now();
        ahead_words(&root, cubes_rect, &state.world, now);
        ahead_inset(&root, cubes_rect, &state.world, now);
    }
    panel(&mut root, map_rect, |ui| map_panel(ui, state, feed));
    panel(&mut root, corner_rect, |ui| corner_panel(ui, state, feed));

    let scale = window.scale_factor();
    let inner = cubes_rect.shrink(1.0);
    set_viewport(
        &mut camera.0,
        UVec2::new((inner.left() * scale) as u32, (inner.top() * scale) as u32),
        UVec2::new(
            (inner.width() * scale) as u32,
            (inner.height() * scale) as u32,
        ),
    );
    Ok(())
}

fn apply_style(ctx: &egui::Context) {
    ctx.set_visuals(egui::Visuals::dark());
    ctx.all_styles_mut(|style| {
        // Big, plain words: the docs' rule for a dyslexic reader holds on screens too.
        for (text, size) in [
            (egui::TextStyle::Body, 16.0),
            (egui::TextStyle::Button, 16.0),
            (egui::TextStyle::Monospace, 14.0),
            (egui::TextStyle::Heading, 19.0),
            (egui::TextStyle::Small, 13.0),
        ] {
            if let Some(font) = style.text_styles.get_mut(&text) {
                font.size = size;
            }
        }
    });
}

fn read_keys(ctx: &egui::Context, state: &mut UiState) {
    if ctx.egui_wants_keyboard_input() {
        return;
    }
    ctx.input(|input| {
        let layers = &mut state.layers;
        for (key, layer) in [
            (egui::Key::G, &mut layers.ground),
            (egui::Key::L, &mut layers.lanes),
            (egui::Key::V, &mut layers.nav2_cubes),
            (egui::Key::N, &mut layers.ndt_cubes),
            (egui::Key::O, &mut layers.costmap),
            (egui::Key::M, &mut layers.ndt_map),
            (egui::Key::S, &mut layers.scan),
            (egui::Key::P, &mut layers.plan),
            (egui::Key::T, &mut layers.ahead),
            (egui::Key::K, &mut layers.things),
            (egui::Key::H, &mut layers.chunks),
        ] {
            if input.key_pressed(key) {
                *layer = !*layer;
            }
        }
        if input.key_pressed(egui::Key::C) {
            state.camera = match state.camera {
                Eyes::Driver => Eyes::Orbit,
                Eyes::Orbit => Eyes::Driver,
            };
        }
        if input.key_pressed(egui::Key::F) {
            state.map_follow_key = true;
        }
        if input.key_pressed(egui::Key::R) {
            state.orbit.reset();
            state.look = LOOK_DOWN;
            state.driver_fov = DRIVER_FOV;
        }
    });
}

/// A panel: its own rectangle, a dark background and a thin edge.
fn panel(root: &mut egui::Ui, rect: egui::Rect, add: impl FnOnce(&mut egui::Ui)) {
    root.painter()
        .rect_filled(rect, 0.0, egui::Color32::from_gray(14));
    root.painter().rect_stroke(
        rect,
        0.0,
        egui::Stroke::new(1.0, egui::Color32::from_gray(50)),
        egui::StrokeKind::Inside,
    );
    let mut ui = root.new_child(egui::UiBuilder::new().max_rect(rect.shrink(8.0)));
    ui.set_clip_rect(rect);
    add(&mut ui);
}

fn status_strip(ui: &mut egui::Ui, state: &mut UiState, feed: &mut dyn feed::Feed) {
    let world = &mut state.world;
    let estop_on = world.vehicle.is_some_and(|v| v.estop_active);
    let verdict = feed::verdict(estop_on, &world.gate, &world.watchdog, Instant::now());
    let allowed = feed::command_allowed(feed.may_command(), &world.origins);
    let mut press = false;
    ui.horizontal_centered(|ui| {
        if let Some(banner) = world.origins.banner() {
            pill(
                ui,
                &banner,
                egui::Color32::from_rgb(255, 140, 30),
                egui::Color32::BLACK,
            );
        }
        let light = match verdict.light {
            Light::Go => egui::Color32::from_rgb(45, 210, 110),
            Light::Wait => egui::Color32::from_rgb(240, 190, 70),
            Light::Stop => egui::Color32::from_rgb(235, 70, 70),
            Light::Unknown => egui::Color32::from_gray(150),
        };
        pill(ui, verdict.headline, light, egui::Color32::BLACK);
        ui.label(&verdict.reason);
        ui.label(egui::RichText::new(verdict.hint).color(egui::Color32::from_gray(150)));
        ui.separator();
        // Who is driving, as the map window's debug page says (TODO 247).
        match world.driving.fresh(Instant::now()) {
            Some(who) => ui.label(egui::RichText::new(format!("DRIVING: {who}")).strong()),
            None => {
                ui.label(egui::RichText::new("DRIVING: ?").color(egui::Color32::from_gray(150)))
            }
        };
        ui.separator();
        ui.label(if estop_on { "e-stop ON" } else { "e-stop off" });
        ui.separator();
        ui.label(feed.label());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let stop = egui::Button::new(
                egui::RichText::new("  E-STOP  ")
                    .strong()
                    .color(egui::Color32::WHITE),
            )
            .fill(egui::Color32::from_rgb(210, 30, 40));
            // It only ever sets the e-stop: clearing is pixi run clear estop.
            let response = ui.add_enabled(allowed.is_ok(), stop);
            press = response.clicked();
            if let Err(why) = &allowed {
                response.on_disabled_hover_text(why);
            }
        });
    });
    if press {
        if let Err(why) = feed.press_estop() {
            world.push_line(Level::Error, format!("e-stop not sent: {why}"));
        }
    }
}

fn pill(ui: &mut egui::Ui, words: &str, fill: egui::Color32, text: egui::Color32) {
    egui::Frame::new()
        .fill(fill)
        .corner_radius(4.0)
        .inner_margin(egui::vec2(10.0, 4.0))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(words).strong().color(text));
        });
}

fn title(ui: &mut egui::Ui, words: &str) {
    ui.label(
        egui::RichText::new(words)
            .strong()
            .color(egui::Color32::from_gray(190)),
    );
}

fn picture_panel(ui: &mut egui::Ui, picture_id: egui::TextureId, size: Option<[u32; 2]>) {
    title(ui, "THE SCENE");
    let Some([width, height]) = size else {
        ui.centered_and_justified(|ui| {
            ui.label("no picture yet: is the chase camera on? (chase_camera:=true)");
        });
        return;
    };
    let room = ui.available_size();
    let aspect = width as f32 / height.max(1) as f32;
    let size = if room.x / room.y > aspect {
        egui::vec2(room.y * aspect, room.y)
    } else {
        egui::vec2(room.x, room.x / aspect)
    };
    ui.centered_and_justified(|ui| {
        ui.image(egui::load::SizedTexture::new(picture_id, size));
    });
}

fn console_panel(ui: &mut egui::Ui, state: &mut UiState) {
    ui.horizontal(|ui| {
        title(ui, "CONSOLE");
        ui.selectable_value(&mut state.console_floor, Level::Info, "All");
        ui.selectable_value(&mut state.console_floor, Level::Warn, "Warnings");
        ui.selectable_value(&mut state.console_floor, Level::Error, "Errors");
    });
    egui::ScrollArea::vertical()
        .auto_shrink(false)
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for (level, line) in state
                .world
                .console
                .iter()
                .filter(|(l, _)| *l >= state.console_floor)
            {
                let colour = match level {
                    Level::Info => egui::Color32::from_gray(190),
                    Level::Warn => egui::Color32::from_rgb(240, 190, 70),
                    Level::Error => egui::Color32::from_rgb(255, 95, 95),
                };
                ui.label(egui::RichText::new(line).monospace().color(colour));
            }
        });
}

/// The cubes' panel draws no background: Bevy's 3D camera shows through.
fn cubes_panel(root: &mut egui::Ui, rect: egui::Rect, state: &mut UiState) {
    let response = root.allocate_rect(rect, egui::Sense::click_and_drag());
    let drag = response.drag_delta();
    if response.dragged_by(egui::PointerButton::Primary) {
        match state.camera {
            Eyes::Driver => {
                state.look[0] -= drag.x * 0.005;
                state.look[1] = (state.look[1] - drag.y * 0.005).clamp(-1.2, 1.2);
            }
            Eyes::Orbit => state.orbit.rotate(drag.x * 0.01, drag.y * 0.01),
        }
    }
    if state.camera == Eyes::Orbit && response.dragged_by(egui::PointerButton::Secondary) {
        state.orbit.pan(drag.x * 0.05, drag.y * 0.05);
    }
    if response.hovered() {
        let scroll = root.input(|input| input.smooth_scroll_delta.y);
        if scroll != 0.0 {
            let by = (1.0 - scroll * 0.002).clamp(0.5, 1.5);
            match state.camera {
                Eyes::Driver => state.driver_fov = (state.driver_fov * by).clamp(0.15, 1.4),
                Eyes::Orbit => state.orbit.zoom(by),
            }
        }
    }
    root.painter().rect_stroke(
        rect,
        0.0,
        egui::Stroke::new(1.0, egui::Color32::from_gray(50)),
        egui::StrokeKind::Inside,
    );
    root.painter().text(
        rect.left_top() + egui::vec2(8.0, 6.0),
        egui::Align2::LEFT_TOP,
        format!("WHAT THE LIDAR SEES   {}", state.camera.label()),
        egui::FontId::proportional(16.0),
        egui::Color32::from_gray(210),
    );
}

/// NDT's footprint: (west, south, east, north), metres.
fn footprint_bounds(points: &[[f32; 2]]) -> [f32; 4] {
    points.iter().fold(
        [f32::MAX, f32::MAX, f32::MIN, f32::MIN],
        |[w, s, e, n], [x, y]| [w.min(*x), s.min(*y), e.max(*x), n.max(*y)],
    )
}

/// NDT's footprint as a picture, made once: drawing a campus's dots every frame cost more than
/// the rest of the map together.
fn footprint_image(points: &[[f32; 2]]) -> egui::ColorImage {
    let [west, south, east, north] = footprint_bounds(points);
    let size = |span: f32| ((span / FOOTPRINT_CELL).ceil() as usize + 1).clamp(1, MAX_FOOTPRINT);
    let (width, height) = (size(east - west), size(north - south));
    let mut image = egui::ColorImage::new(
        [width, height],
        vec![egui::Color32::TRANSPARENT; width * height],
    );
    let dot = egui::Color32::from_rgb(205, 165, 245);
    for [x, y] in points {
        let col = (((x - west) / (east - west).max(1.0)) * (width - 1) as f32) as usize;
        let row = (((north - y) / (north - south).max(1.0)) * (height - 1) as f32) as usize;
        image.pixels[row * width + col] = dot;
    }
    image
}

/// What the colours mean, so nobody has to guess.
fn legend(root: &egui::Ui, rect: egui::Rect, world: &WorldNews, ndt_cubes: bool) {
    let rows: &[([u8; 3], &str)] = if ndt_cubes && world.ndt.is_some() {
        &[
            (NDT_DARK_RGB, "NDT's map"),
            (NDT_LIT_RGB, "lidar sees it too"),
            (NEW_CUBE_RGB, "lidar only: not on the map"),
        ]
    } else {
        &BAND_LEGEND
    };
    let mut x = rect.left() + 8.0;
    let y = rect.top() + 50.0;
    for ([r, g, b], words) in rows {
        let square = egui::Rect::from_min_size(egui::pos2(x, y + 2.0), egui::vec2(12.0, 12.0));
        root.painter()
            .rect_filled(square, 2.0, egui::Color32::from_rgb(*r, *g, *b));
        let end = root.painter().text(
            egui::pos2(x + 17.0, y),
            egui::Align2::LEFT_TOP,
            *words,
            egui::FontId::proportional(14.0),
            egui::Color32::from_gray(200),
        );
        x = end.right() + 16.0;
    }
}

/// NDT's state over the cubes: is localization holding, and how far from the EKF?
fn ndt_line(root: &egui::Ui, rect: egui::Rect, world: &WorldNews, ndt_cubes: bool) {
    let mut line = feed::ndt_line(world.ndt_status.as_ref(), world.ndt_pose, world.car_map);
    if let Some(share) = world.map_match.share_on_map(world.cubes.cubes.len()) {
        line.words
            .push_str(&format!("   {:.0}% of cubes on the map", share * 100.0));
    }
    let colour = match line.state {
        feed::NdtLight::Good => egui::Color32::from_rgb(45, 210, 110),
        feed::NdtLight::Poor => egui::Color32::from_rgb(240, 190, 70),
        feed::NdtLight::Lost => egui::Color32::from_rgb(255, 95, 95),
        feed::NdtLight::Off => egui::Color32::from_gray(170),
    };
    root.painter().text(
        rect.left_top() + egui::vec2(8.0, 28.0),
        egui::Align2::LEFT_TOP,
        line.words,
        egui::FontId::proportional(16.0),
        colour,
    );
    legend(root, rect, world, ndt_cubes);
    if world.cubes.cubes.is_empty() {
        root.painter().text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "no cubes yet: run with screen_cubes:=true or voxels:=true",
            egui::FontId::proportional(16.0),
            egui::Color32::from_gray(170),
        );
    }
}

fn colour([r, g, b]: [u8; 3]) -> egui::Color32 {
    egui::Color32::from_rgb(r, g, b)
}

/// The squiggly line on a flat view: the plan faint and thin, the trace bold in its grade's
/// colour, the stop bar across the way. `place` puts a base_link point on the screen.
fn paint_ahead(
    painter: &egui::Painter,
    squiggle: &Ahead,
    place: impl Fn((f64, f64)) -> egui::Pos2,
) {
    if squiggle.plan.len() >= 2 {
        let line = squiggle.plan.iter().map(|p| place(*p)).collect();
        painter.add(egui::Shape::line(
            line,
            egui::Stroke::new(2.0, colour(AHEAD_PLAN_RGB)),
        ));
    }
    if squiggle.trace.len() >= 2 {
        let line = squiggle.trace.iter().map(|p| place(*p)).collect();
        let rgb = trace_rgb(squiggle.summary.as_ref());
        painter.add(egui::Shape::line(line, egui::Stroke::new(5.0, colour(rgb))));
    }
    if let Some(stop) = squiggle.stop {
        let (sin, cos) = stop.yaw.sin_cos();
        let end = |side: f64| place((stop.x - side * sin, stop.y + side * cos));
        painter.line_segment(
            [end(STOP_BAR_HALF), end(-STOP_BAR_HALF)],
            egui::Stroke::new(4.0, colour(AHEAD_STOP_RGB)),
        );
    }
}

/// The squiggly line's three lines of words, at the scene's bottom left, on a dark band.
fn ahead_words(root: &egui::Ui, rect: egui::Rect, world: &WorldNews, now: Instant) {
    let squiggle = world.ahead_now(now);
    let summary = squiggle.as_ref().and_then(|s| s.summary.as_ref());
    let lines = ahead::lines(summary);
    let source = summary.map_or("", |s| s.plan.label());
    let size = 18.0;
    let gap = 24.0;
    let height = gap * 4.0 + 8.0;
    let band = egui::Rect::from_min_size(
        egui::pos2(rect.left() + 8.0, rect.bottom() - height - 8.0),
        egui::vec2(250.0, height),
    );
    root.painter()
        .rect_filled(band, 4.0, egui::Color32::from_gray(10));
    let mut y = band.top() + 6.0;
    root.painter().text(
        egui::pos2(band.left() + 8.0, y),
        egui::Align2::LEFT_TOP,
        format!("AHEAD (T)   {source}"),
        egui::FontId::proportional(14.0),
        egui::Color32::from_gray(190),
    );
    y += gap;
    for line in lines.iter().filter(|l| !l.text.is_empty()) {
        root.painter().text(
            egui::pos2(band.left() + 8.0, y),
            egui::Align2::LEFT_TOP,
            &line.text,
            egui::FontId::proportional(size),
            colour(tone_rgb(line.tone)),
        );
        y += gap;
    }
}

/// Metres the inset shows ahead of the rear axle, and behind it.
const INSET_AHEAD: f64 = 8.0;
const INSET_BEHIND: f64 = 1.5;
/// The car's outline in the inset, base_link metres: (rear, front, half width). Only a
/// picture: the scene's car box is the same size.
const INSET_CAR: (f64, f64, f64) = (-0.5, 2.1, 0.6);

/// The squiggly line from above, close up, at the scene's right: at the scene's own scale
/// two seconds of driving is a few pixels. Forward is up; a faint line every 2 m.
fn ahead_inset(root: &egui::Ui, rect: egui::Rect, world: &WorldNews, now: Instant) {
    let Some(squiggle) = world.ahead_now(now) else {
        return;
    };
    let width = 170.0;
    let inset = egui::Rect::from_min_max(
        egui::pos2(rect.right() - width - 8.0, rect.top() + 60.0),
        egui::pos2(rect.right() - 8.0, rect.bottom() - 8.0),
    );
    let painter = root.painter_at(inset);
    painter.rect_filled(inset, 4.0, egui::Color32::from_gray(10));
    let view = ahead::View {
        rect: (inset.left(), inset.top(), inset.width(), inset.height()),
        ahead: INSET_AHEAD,
        behind: INSET_BEHIND,
    };
    let at = |p: (f64, f64)| {
        let (x, y) = view.to_panel(p);
        egui::pos2(x, y)
    };
    for metres in (2..=INSET_AHEAD as i32).step_by(2) {
        let m = f64::from(metres);
        let y = at((m, 0.0)).y;
        painter.line_segment(
            [egui::pos2(inset.left(), y), egui::pos2(inset.right(), y)],
            egui::Stroke::new(1.0, egui::Color32::from_gray(45)),
        );
        painter.text(
            egui::pos2(inset.left() + 4.0, y - 2.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{metres} m"),
            egui::FontId::proportional(12.0),
            egui::Color32::from_gray(140),
        );
    }
    let (rear, front, half) = INSET_CAR;
    painter.rect_stroke(
        egui::Rect::from_two_pos(at((front, half)), at((rear, -half))),
        2.0,
        egui::Stroke::new(1.5, egui::Color32::from_rgb(140, 190, 255)),
        egui::StrokeKind::Middle,
    );
    // painter_at clips to the inset: a line that runs off the top just stops there.
    paint_ahead(&painter, &squiggle, at);
}

fn screen(p: ScreenPoint) -> egui::Pos2 {
    egui::pos2(p.x as f32, p.y as f32)
}

fn map_panel(ui: &mut egui::Ui, state: &mut UiState, feed: &mut dyn feed::Feed) {
    let (mut follow_clicked, mut whole_clicked) = (false, false);
    ui.horizontal(|ui| {
        title(ui, "THE MAP: click to pick a goal");
        let mode = state.map.view.mode;
        follow_clicked = ui
            .selectable_label(mode == MapMode::FollowCar, "Follow car (F)")
            .clicked();
        whole_clicked = ui
            .selectable_label(mode == MapMode::WholeMap, "Whole map")
            .clicked();
        let layers = &mut state.layers;
        ui.toggle_value(&mut layers.costmap, "Costmap");
        ui.toggle_value(&mut layers.ndt_map, "NDT");
        ui.toggle_value(&mut layers.scan, "Cubes");
        ui.toggle_value(&mut layers.plan, "Plan");
        ui.toggle_value(&mut layers.ahead, "Ahead");
        ui.toggle_value(&mut layers.things, "Things");
    });

    let rect = ui.available_rect_before_wrap();
    let viewport = MapViewport {
        x: f64::from(rect.left()),
        y: f64::from(rect.top()),
        width: f64::from(rect.width()),
        height: f64::from(rect.height()),
    };
    let car = state.world.car_map.position;
    // With chunks, the view starts sized to those loaded (the car's 3 x 3 and the
    // route's): the three levels. The whole map is the static map's box.
    let near = state
        .world
        .chunks
        .as_ref()
        .filter(|_| state.layers.chunks)
        .and_then(|grid| chunks::bounds(&chunks::loaded(grid, car, state.world.chunk_route())));
    let whole = state
        .world
        .map
        .grid
        .as_ref()
        .map(|grid| grid.info.bounds())
        .or(near);

    let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());
    if response.clicked() {
        if let Some(at) = response.interact_pointer_pos() {
            let on_map = state.map.view.to_map(
                ScreenPoint::new(f64::from(at.x), f64::from(at.y)),
                &viewport,
            );
            match feed::refusal(on_map, state.world.map.grid.as_ref()) {
                None => state.goal = Goal::Picked(on_map),
                Some(why) => state
                    .world
                    .push_line(Level::Warn, format!("no goal: {why}")),
            }
        }
    }
    let mut gestures = Vec::new();
    // Any button's drag pans: a Mac's trackpad has no easy right-drag.
    if response.dragged() {
        let drag = response.drag_delta();
        gestures.push(Gesture::Drag {
            dx: f64::from(drag.x),
            dy: f64::from(drag.y),
        });
    }
    if response.hovered() {
        let at = response
            .hover_pos()
            .map(|p| ScreenPoint::new(f64::from(p.x), f64::from(p.y)));
        let (scroll, pinch) = ui.input(|input| (input.smooth_scroll_delta.y, input.zoom_delta()));
        gestures.push(Gesture::Wheel {
            by: f64::from(scroll),
            at,
        });
        gestures.push(Gesture::Pinch {
            factor: f64::from(pinch),
            at,
        });
    }
    if std::mem::take(&mut state.map_follow_key) {
        gestures.push(Gesture::ToggleFollow);
    }
    if follow_clicked {
        gestures.push(Gesture::FollowCar);
    }
    if whole_clicked {
        gestures.push(Gesture::WholeMap);
    }
    for gesture in gestures {
        state.map.apply(gesture, &viewport);
    }
    state.map.frame(car, whole, near, &viewport);

    let painter = ui.painter_at(rect);
    let view = state.map.view;
    let to = |p: Point| screen(view.to_screen(p, &viewport));
    let k = view.scale.0 as f32;
    let layers = state.layers;
    let world = &mut state.world;

    if world.map.grid.is_none() {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "waiting for the map",
            egui::FontId::proportional(16.0),
            egui::Color32::from_gray(170),
        );
    }
    world.map.paint(ui, "map", &MAP_COLOURS, &painter, to);
    if layers.costmap {
        world
            .costmap
            .paint(ui, "costmap", &COST_COLOURS, &painter, to);
    }
    if layers.ndt_map {
        if let Some(cells) = &world.ndt {
            let texture = world.ndt_picture.get_or_insert_with(|| {
                ui.ctx().load_texture(
                    "ndt_map",
                    footprint_image(&cells.footprint.points),
                    egui::TextureOptions::NEAREST,
                )
            });
            let bounds = footprint_bounds(&cells.footprint.points);
            painter.image(
                texture.id(),
                egui::Rect::from_min_max(
                    to(Point::new(f64::from(bounds[0]), f64::from(bounds[3]))),
                    to(Point::new(f64::from(bounds[2]), f64::from(bounds[1]))),
                ),
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }
    }
    if let (true, Some(grid)) = (layers.chunks, world.chunks) {
        paint_chunks(ui, world, &grid, &painter, to);
    }
    if layers.lanes {
        for lane in &world.lanes {
            let (width, colour) = match lane.look {
                LaneLook::Solid => (1.5, egui::Color32::from_rgb(230, 230, 230)),
                LaneLook::Dashed => (1.0, egui::Color32::from_rgb(150, 150, 150)),
                LaneLook::Stop => (3.0, egui::Color32::from_rgb(235, 40, 40)),
                LaneLook::Curb => (2.0, egui::Color32::from_rgb(200, 170, 60)),
            };
            let line: Vec<_> = lane.points.iter().map(|p| to(*p)).collect();
            painter.add(egui::Shape::line(line, egui::Stroke::new(width, colour)));
        }
    }
    let car = world.car_map;
    if layers.scan {
        for cube in world.cubes.cubes.iter().take(MAX_MAP_CUBES) {
            let p = feed::odom_to_map([cube.centre[0], cube.centre[1]], world.car_odom, car);
            painter.circle_filled(to(p), 1.5, egui::Color32::from_rgb(40, 220, 110));
        }
    }
    if layers.things {
        // Each thing: its box (odom, onto the map) in its kind's colour, and its name.
        for thing in world.things_now(Instant::now()) {
            let (hl, hw) = (thing.length / 2.0, thing.width / 2.0);
            let corner = |dx: f64, dy: f64| {
                let odom = [(thing.x + dx) as f32, (thing.y + dy) as f32];
                to(feed::odom_to_map(odom, world.car_odom, car))
            };
            let outline = vec![
                corner(-hl, -hw),
                corner(hl, -hw),
                corner(hl, hw),
                corner(-hl, hw),
                corner(-hl, -hw),
            ];
            let colour = rgba_colour(thing.colour());
            painter.add(egui::Shape::line(outline, egui::Stroke::new(2.0, colour)));
            if thing.is_named() {
                painter.text(
                    corner(0.0, hw),
                    egui::Align2::CENTER_BOTTOM,
                    thing.label(),
                    egui::FontId::proportional(14.0),
                    egui::Color32::WHITE,
                );
            }
        }
    }
    if layers.chunks && world.route.len() >= 2 {
        let line: Vec<_> = world.route.iter().map(|p| to(*p)).collect();
        painter.add(egui::Shape::line(
            line,
            egui::Stroke::new(4.0, egui::Color32::from_rgb(170, 90, 230)),
        ));
    }
    if layers.plan && world.plan.len() >= 2 {
        let line: Vec<_> = world.plan.iter().map(|p| to(*p)).collect();
        painter.add(egui::Shape::line(
            line,
            egui::Stroke::new(2.5, egui::Color32::from_rgb(90, 200, 255)),
        ));
    }
    if layers.plan && world.road_ahead.line.len() >= 2 {
        let top = world.road_ahead.top_speed();
        for pair in world.road_ahead.line.windows(2) {
            let (a, v) = pair[0];
            let (b, _) = pair[1];
            painter.line_segment([to(a), to(b)], egui::Stroke::new(5.0, road_colour(v, top)));
        }
        if let Some((end, heading)) = world.road_ahead.end {
            let (sin, cos) = heading.sin_cos();
            let side = |s: f64| to(Point::new(end.x - 1.5 * s * sin, end.y + 1.5 * s * cos));
            painter.line_segment(
                [side(1.0), side(-1.0)],
                egui::Stroke::new(4.0, egui::Color32::from_rgb(235, 40, 40)),
            );
        }
        for (stop, why) in &world.road_ahead.stops {
            painter.circle_stroke(to(*stop), 6.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
            painter.text(
                to(*stop),
                egui::Align2::LEFT_BOTTOM,
                format!("  {why}"),
                egui::FontId::proportional(13.0),
                egui::Color32::WHITE,
            );
        }
    }
    if layers.ahead {
        if let Some(squiggle) = world.ahead_now(Instant::now()) {
            let place = |p: (f64, f64)| to(feed::from_base(car, p));
            paint_ahead(&painter, &squiggle, place);
        }
    }
    if let Some(goal) = state.goal.point() {
        painter.circle_stroke(to(goal), 7.0, egui::Stroke::new(2.5, egui::Color32::YELLOW));
    } else if let Some(goal) = world.goal_elsewhere(Instant::now()) {
        painter.circle_stroke(to(goal), 7.0, egui::Stroke::new(2.5, egui::Color32::GOLD));
    }
    // The car as an arrow, big enough to find whatever the zoom.
    let (sin, cos) = car.yaw.sin_cos();
    let reach = f64::from(10.0 / k).max(1.5);
    let nose = Point::new(car.position.x + reach * cos, car.position.y + reach * sin);
    let side = |d: f64| {
        Point::new(
            car.position.x - reach * 0.6 * cos - d * sin,
            car.position.y - reach * 0.6 * sin + d * cos,
        )
    };
    painter.add(egui::Shape::convex_polygon(
        vec![to(nose), to(side(reach * 0.6)), to(side(-reach * 0.6))],
        egui::Color32::from_rgb(140, 190, 255),
        egui::Stroke::new(1.5, egui::Color32::WHITE),
    ));

    let world_goal = state.world.goal_elsewhere(Instant::now());
    goal_buttons(ui, rect, state, feed, world_goal);
}

/// The two presses a goal needs, over the map's bottom edge. The gates are checked at the
/// last press, against what's true then.
fn goal_buttons(
    ui: &mut egui::Ui,
    map: egui::Rect,
    state: &mut UiState,
    feed: &mut dyn feed::Feed,
    world_goal: Option<Point>,
) {
    let row = egui::Rect::from_min_max(
        egui::pos2(map.left() + 6.0, map.bottom() - 34.0),
        egui::pos2(map.right() - 6.0, map.bottom() - 4.0),
    );
    ui.scope_builder(egui::UiBuilder::new().max_rect(row), |ui| {
        ui.horizontal(|ui| match state.goal {
            Goal::None => match world_goal {
                Some(goal) => {
                    ui.label(format!(
                        "driving to a goal sent elsewhere: x {:.1}, y {:.1}",
                        goal.x, goal.y
                    ));
                }
                None => {
                    ui.label("no goal");
                }
            },
            Goal::Picked(goal) => {
                if ui.button("DRIVE HERE").clicked() {
                    state.goal = Goal::Armed(goal);
                }
                if ui.button("Cancel").clicked() {
                    state.goal = Goal::None;
                }
            }
            Goal::Armed(goal) => {
                if ui.button("CONFIRM").clicked() {
                    state.goal = confirm(state, feed, goal);
                }
                if ui.button("Cancel").clicked() {
                    state.goal = Goal::None;
                }
            }
            Goal::Driving(_) => {
                if ui.button("CANCEL THE GOAL").clicked() {
                    cancel(state, feed);
                }
                if ui.button("Clear the marker").clicked() {
                    state.goal = Goal::None;
                }
            }
        });
    });
}

/// CANCEL THE GOAL: the navigator's goals cancelled, through the same gates as a goal.
fn cancel(state: &mut UiState, feed: &mut dyn feed::Feed) {
    let world = &mut state.world;
    match feed::command_allowed(feed.may_command(), &world.origins)
        .and_then(|()| feed.cancel_goal())
    {
        Ok(()) => {
            world.note("cancel sent: the navigator answers in the console".to_owned());
            state.goal = Goal::None;
        }
        Err(why) => world.push_line(Level::Error, format!("goal not cancelled: {why}")),
    }
}

/// A click and the two presses at once, for `--click-goal`.
pub fn click_and_confirm(state: &mut UiState, feed: &mut dyn feed::Feed, at: Point) {
    match feed::refusal(at, state.world.map.grid.as_ref()) {
        None => state.goal = confirm(state, feed, at),
        Some(why) => state
            .world
            .push_line(Level::Warn, format!("no goal: {why}")),
    }
}

/// The last press: every gate, then the goal.
fn confirm(state: &mut UiState, feed: &mut dyn feed::Feed, at: Point) -> Goal {
    let world = &mut state.world;
    let sent = feed::command_allowed(feed.may_command(), &world.origins)
        .and_then(|()| feed.send_goal(feed::goal_pose(world.car_map, at)));
    match sent {
        Ok(()) => {
            world.note(format!("goal sent: x {:.1}, y {:.1}", at.x, at.y));
            Goal::Driving(at)
        }
        Err(why) => {
            world.push_line(Level::Error, format!("goal not sent: {why}"));
            Goal::None
        }
    }
}

fn corner_panel(ui: &mut egui::Ui, state: &mut UiState, feed: &mut dyn feed::Feed) {
    ui.horizontal(|ui| {
        ui.selectable_value(&mut state.tab, Tab::Keys, "KEYS");
        ui.selectable_value(&mut state.tab, Tab::Algorithms, "ALGORITHMS");
    });
    ui.separator();
    egui::ScrollArea::vertical()
        .auto_shrink(false)
        .show(ui, |ui| match state.tab {
            Tab::Keys => keys_tab(ui, state),
            Tab::Algorithms => algorithms_tab(ui, state, feed),
        });
}

fn keys_tab(ui: &mut egui::Ui, state: &UiState) {
    let on = |value: bool| if value { "on" } else { "off" };
    let layers = state.layers;
    let rows = [
        (
            "G",
            "the ground: road, and the lidar's rings on raised grass",
            on(layers.ground),
        ),
        ("V", "outline every live cube", on(layers.nav2_cubes)),
        (
            "N",
            "NDT's cells: dark, light when the lidar agrees (off: raw cubes)",
            on(layers.ndt_cubes),
        ),
        ("O", "costmap", on(layers.costmap)),
        ("L", "lane map's lines on the map", on(layers.lanes)),
        ("M", "NDT's map", on(layers.ndt_map)),
        ("S", "cubes on the map", on(layers.scan)),
        ("P", "plan", on(layers.plan)),
        (
            "H",
            "chunks loaded, their saved map; route",
            on(layers.chunks),
        ),
        (
            "T",
            "squiggly line: trace, plan ahead, stop",
            on(layers.ahead),
        ),
        (
            "F",
            "map: follow the car, or the whole map",
            state.map.view.mode.label(),
        ),
        (
            "drag",
            "map: move it (either button); click picks a goal",
            "",
        ),
        ("wheel", "map: zoom at the pointer (or pinch)", ""),
        ("C", "camera", state.camera.label()),
        ("R", "reset the camera", ""),
    ];
    egui::Grid::new("keys")
        .num_columns(3)
        .striped(true)
        .show(ui, |ui| {
            for (key, what, value) in rows {
                ui.label(egui::RichText::new(key).strong());
                ui.label(what);
                ui.label(value);
                ui.end_row();
            }
        });
}

/// The follower row's place in `ALGORITHMS`: the one row that swaps live.
const FOLLOWER_ROW: usize = 1;

/// Whether `pick` is one of aav_track's followers, which swap live (`tracker` parameter).
/// The chunks loaded for the car (`chunks::loaded`): each one's saved map under its
/// outline, bright round the car (0.25 m) and amber along the route (1 m), labelled (i, j).
fn paint_chunks(
    ui: &egui::Ui,
    world: &mut WorldNews,
    grid: &map_chunks::Grid,
    painter: &egui::Painter,
    to: impl Fn(Point) -> egui::Pos2,
) {
    let boxes = chunks::loaded(grid, world.car_map.position, world.chunk_route());
    if let Some(folder) = world.chunk_folder.clone() {
        let maps = world.saved_maps.of(&folder, grid, &boxes);
        for (key, detail, saved) in maps {
            let texture = world
                .saved_pictures
                .entry((key, detail))
                .or_insert_with(|| {
                    let image = egui::ColorImage::from_rgba_unmultiplied(
                        [saved.info.width as usize, saved.info.height as usize],
                        &saved.to_rgba(&MAP_COLOURS),
                    );
                    ui.ctx().load_texture(
                        format!("chunk {key} {}", chunks::detail_word(detail)),
                        image,
                        egui::TextureOptions::NEAREST,
                    )
                });
            let bounds = saved.info.bounds();
            painter.image(
                texture.id(),
                egui::Rect::from_min_max(
                    to(Point::new(bounds.min.x, bounds.max.y)),
                    to(Point::new(bounds.max.x, bounds.min.y)),
                ),
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }
        let errors = std::mem::take(&mut world.saved_maps.errors);
        for why in errors {
            world.push_line(Level::Warn, format!("a chunk's saved map: {why}"));
        }
    }
    for b in &boxes {
        let colour = match (b.here, b.detail) {
            (true, _) => egui::Color32::from_rgb(120, 200, 255),
            (false, map_chunks::Detail::Fine) => egui::Color32::from_rgb(70, 120, 220),
            (false, map_chunks::Detail::Coarse) => egui::Color32::from_rgb(220, 150, 60),
        };
        let line: Vec<_> = b.outline.iter().map(|p| to(*p)).collect();
        painter.add(egui::Shape::line(line, egui::Stroke::new(1.5, colour)));
        let top_left = b.outline[3];
        painter.text(
            to(Point::new(top_left.x + 1.0, top_left.y - 1.0)),
            egui::Align2::LEFT_TOP,
            &b.label,
            egui::FontId::proportional(12.0),
            colour,
        );
    }
}

fn live_tracker(pick: &str) -> Option<&'static str> {
    Tracker::ALL.iter().map(|t| t.name()).find(|n| *n == pick)
}

fn algorithms_tab(ui: &mut egui::Ui, state: &mut UiState, feed: &mut dyn feed::Feed) {
    egui::Grid::new("algorithms").num_columns(2).show(ui, |ui| {
        for (row, (name, choices)) in ALGORITHMS.iter().enumerate() {
            ui.label(*name);
            egui::ComboBox::from_id_salt(name).show_index(
                ui,
                &mut state.picks[row],
                choices.len(),
                |i| choices.get(i).copied().unwrap_or(""),
            );
            ui.end_row();
        }
    });
    let pick = ALGORITHMS[FOLLOWER_ROW]
        .1
        .get(state.picks[FOLLOWER_ROW])
        .copied()
        .unwrap_or("");
    match live_tracker(pick) {
        Some(name) => {
            if ui.button(format!("SWAP THE FOLLOWER TO {name}")).clicked() {
                let world = &mut state.world;
                let asked = feed::command_allowed(feed.may_command(), &world.origins)
                    .and_then(|()| feed.swap_tracker(name));
                match asked {
                    Ok(()) => world.note(format!(
                        "swap to {name} asked: the answer comes in the console"
                    )),
                    Err(why) => world.push_line(Level::Error, format!("not swapped: {why}")),
                }
            }
        }
        None => {
            ui.add_enabled(false, egui::Button::new("SWAP"));
        }
    }
    ui.label(
        "Live, in the simulator: aav_track's followers (pure_pursuit, stanley, lqr, mpc), when the \
         sim runs one (tracker:=pure_pursuit). The rest are chosen at launch: planner:=, \
         planner_search:=, tracker:=, ndt:=, ndt_ekf:=, voxels:=, speed_loop:= (docs/trials.md).",
    );
}

/// The things' names in the 3D scene: each named thing's words over its top, in a dark
/// box, as the camera sees them (`place`: a point in odom to the window, None off screen).
fn thing_names(
    root: &egui::Ui,
    rect: egui::Rect,
    world: &WorldNews,
    place: impl Fn(Vec3) -> Option<Vec2>,
) {
    let painter = root.painter_at(rect);
    // Below the panel's title, NDT line and legend; and never on a name already drawn
    // (seen 15 Nov: three names in a pile on the header). The farthest first, from the
    // top of the picture down, so a pushed name stays in depth order; a name moved off its
    // thing gets a thin line to it.
    let clear_top = rect.top() + NAMES_BELOW;
    let mut labels: Vec<(egui::Pos2, &Thing)> = world
        .things_now(Instant::now())
        .iter()
        .filter(|thing| thing.is_named())
        .filter_map(|thing| {
            let top = Vec3::new(thing.x as f32, thing.y as f32, thing.height as f32 + 0.4);
            let at = place(top).map(|p| egui::pos2(p.x, p.y))?;
            rect.contains(at).then_some((at, thing))
        })
        .collect();
    labels.sort_by(|a, b| a.0.y.total_cmp(&b.0.y));
    let mut placed: Vec<egui::Rect> = Vec::new();
    for (at, thing) in labels {
        let galley = painter.layout_no_wrap(
            thing.label(),
            egui::FontId::proportional(16.0),
            egui::Color32::WHITE,
        );
        let size = galley.size() + egui::vec2(12.0, 6.0);
        let mut back = egui::Rect::from_center_size(at - egui::vec2(0.0, size.y / 2.0), size);
        if back.top() < clear_top {
            back = back.translate(egui::vec2(0.0, clear_top - back.top()));
        }
        while let Some(hit) = placed.iter().find(|r| r.intersects(back)) {
            back = back.translate(egui::vec2(0.0, hit.bottom() + 2.0 - back.top()));
        }
        if !rect.contains_rect(back) {
            continue;
        }
        let colour = rgba_colour(thing.colour());
        if !back.expand(2.0).contains(at) {
            let foot = egui::pos2(back.center().x, back.top());
            painter.line_segment([foot, at], egui::Stroke::new(1.0, colour));
        }
        painter.rect_filled(back, 3.0, egui::Color32::from_black_alpha(190));
        painter.rect_stroke(
            back,
            3.0,
            egui::Stroke::new(2.0, colour),
            egui::StrokeKind::Inside,
        );
        painter.galley(
            back.min + egui::vec2(6.0, 3.0),
            galley,
            egui::Color32::WHITE,
        );
        placed.push(back);
    }
}

/// The 3D view's names start this far below its top: under its title, NDT line and
/// legend (`ndt_line`, `cubes_panel`).
const NAMES_BELOW: f32 = 74.0;

/// A hud_core colour (the things' kinds) for egui.
fn rgba_colour(c: hud_core::theme::Rgba) -> egui::Color32 {
    egui::Color32::from_rgba_unmultiplied(c.r, c.g, c.b, c.a)
}

/// The road ahead's colour for a speed limit: green at the strip's top speed, red at a stop.
fn road_colour(limit: f64, top: f64) -> egui::Color32 {
    let share = feed::speed_share(limit, top);
    egui::Color32::from_rgb(
        (235.0 * (1.0 - share) + 40.0 * share) as u8,
        (40.0 * (1.0 - share) + 210.0 * share) as u8,
        60,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_road_and_the_rings_on_raised_grass_are_ground_the_kerb_stands() {
        use debug_core::cubes::Cube;
        let now = Instant::now();
        let cube = |x: f32, y: f32, z: f32| Cube {
            centre: [x, y, z],
            band: Band::of(Metres(f64::from(z)), Metres(0.0)),
        };
        let scan = |cubes| CubeScan {
            cubes,
            ..CubeScan::default()
        };
        let mut world = WorldNews::default();
        // The voxels node's road, a little above base_link: still ground.
        world.apply(Update::Road(scan(vec![cube(4.8, 1.0, 0.01)])), now);
        assert_eq!(world.road.cubes[0].band, Band::Ground);
        // The kerb beside the road, grass behind it, a wall.
        let cubes = vec![
            cube(5.2, 1.0, 0.31),
            cube(9.0, 1.0, 0.31),
            cube(9.0, 4.0, 1.0),
        ];
        world.apply(Update::Cubes(scan(cubes)), now);
        let bands: Vec<Band> = world.cubes.cubes.iter().map(|c| c.band).collect();
        assert_eq!(bands, [Band::Knee, Band::Ground, Band::Waist]);
        // A newer road away from the kerb: the kerb's cube is flat ground too.
        world.apply(Update::Road(scan(vec![cube(-4.0, 1.0, 0.0)])), now);
        assert_eq!(world.cubes.cubes[0].band, Band::Ground);
        assert!(!Layers::default().ground, "the ground starts hidden");
    }

    #[test]
    fn a_fresh_plan_shows_a_goal_sent_elsewhere() {
        let start = Instant::now();
        let mut world = WorldNews::default();
        assert_eq!(world.goal_elsewhere(start), None);
        world.apply(
            Update::Plan(vec![Point::new(0.0, 0.0), Point::new(14.0, 2.0)]),
            start,
        );
        assert_eq!(world.goal_elsewhere(start), Some(Point::new(14.0, 2.0)));
        let later = start + GOAL_PLAN_FRESH + std::time::Duration::from_millis(1);
        assert_eq!(
            world.goal_elsewhere(later),
            None,
            "a quiet plan: the goal has ended"
        );
        world.apply(Update::Plan(Vec::new()), later);
        assert_eq!(
            world.goal_elsewhere(later),
            None,
            "an empty plan has no end"
        );
    }

    #[test]
    fn every_live_tracker_is_on_the_follower_row_and_only_they_swap() {
        let (name, choices) = ALGORITHMS[FOLLOWER_ROW];
        assert_eq!(name, "Follower");
        for t in Tracker::ALL {
            assert!(choices.contains(&t.name()), "{} missing", t.name());
            assert_eq!(live_tracker(t.name()), Some(t.name()));
        }
        // Chosen at launch, never swapped live.
        for launch_only in ["mppi", "rpp", "mpc_full"] {
            assert_eq!(live_tracker(launch_only), None);
        }
    }
}

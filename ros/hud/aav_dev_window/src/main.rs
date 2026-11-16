//! The developer window (`docs/designs/2026-11-05_own-simulator.md`): Ben's layout, in Bevy
//! and egui, for laptops.
//!
//! ```text
//! dev_window [--live] [--developer] [--ndt-map FILE] [--click-goal X,Y] [--cubes N] [--all-layers]
//!            [--seconds S] [--screenshot FILE]
//! ```
//!
//! Without `--live`: pretend data, with no ROS. `--live` (the `ros` feature) is the node
//! `dev_window`; its topics are in `ros.rs`. `--developer` lets it send goals and set the
//! e-stop, in the simulator only: `tools/dev_window.sh` is what gives it.
//!
//! `--seconds` is the speed test: no vsync, the frame rate printed, then it exits.
//! `--all-layers` starts with every layer on. `--screenshot` saves one picture halfway
//! through `--seconds`. `--click-goal X,Y` is what a click and the two presses do, after
//! `CLICK_AFTER` seconds, through the same gates: macOS blocks scripted clicks.

mod chunks;
mod feed;
mod lanes;
mod panels;
mod pretend;
#[cfg(feature = "ros")]
mod ros;

use std::{f32::consts::PI, path::PathBuf, time::Instant};

use bevy::{
    app::AppExit,
    asset::RenderAssetUsages,
    camera::{visibility::RenderLayers, CameraOutputMode, Viewport},
    prelude::*,
    render::{
        render_resource::{Extent3d, TextureDimension, TextureFormat},
        view::screenshot::{save_to_disk, Screenshot},
    },
    window::PresentMode,
};
use bevy_egui::{EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass, PrimaryEguiContext};
use debug_core::{cubes::Band, ndt::MapFile};
use map_core::geom::Point;
use wgpu_types::BlendState;

use crate::feed::{Feed, Update};

const USAGE: &str = "usage: dev_window [--live] [--developer] [--ndt-map FILE] \
                     [--click-goal X,Y] [--cubes N] \
                     [--all-layers] [--seconds S] [--screenshot FILE]";
/// One NDT map cube's edge on screen, metres: at 1 m a kerb looked as big as a building.
const NDT_CELL: f64 = 0.5;
/// The most outlines drawn in red. Drawing lines costs more than drawing cubes.
const MAX_OUTLINES: usize = 2_000;
/// The view's width from behind, top to bottom, radians. From the car: `state.driver_fov`.
const ORBIT_FOV: f32 = PI / 4.0;
/// The ground's grid: a line every so many metres, out to this many lines each way.
const GRID_STEP: f32 = 5.0;
const GRID_LINES: i32 = 12;
/// How far from the car NDT's cells are drawn, metres: about the lidar's reach.
const NDT_REACH: f64 = 40.0;
/// How far the car moves before the cells near it are found again, metres.
const NEAR_MOVE: f64 = 5.0;
/// A pool cube is this big already; a map cell scales it up to its own size.
const POOL_CUBE: f32 = pretend::CUBE_SIDE * 0.9;
/// A cell is drawn a little smaller than itself, so its neighbours show as separate blocks.
const CELL_FILL: f32 = 0.92;

/// What this run was asked for.
#[derive(Resource, Clone, Debug)]
struct Options {
    live: bool,
    developer: bool,
    ndt_map: Option<String>,
    lanes: Option<String>,
    chunks: Option<String>,
    click_goal: Option<[f64; 2]>,
    cubes: u32,
    all_layers: bool,
    seconds: Option<f32>,
    screenshot: Option<PathBuf>,
}

impl Options {
    fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self {
            live: false,
            developer: false,
            ndt_map: None,
            lanes: None,
            chunks: None,
            click_goal: None,
            cubes: 20_000,
            all_layers: false,
            seconds: None,
            screenshot: None,
        };
        let mut args = args;
        while let Some(arg) = args.next() {
            let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
            match arg.as_str() {
                "--live" => options.live = true,
                "--developer" => options.developer = true,
                "--ndt-map" => options.ndt_map = Some(value()?),
                "--lanes" => options.lanes = Some(value()?),
                "--chunks" => options.chunks = Some(value()?),
                "--click-goal" => {
                    let text = value()?;
                    let parsed = text
                        .split_once(',')
                        .and_then(|(x, y)| Some([x.trim().parse().ok()?, y.trim().parse().ok()?]));
                    options.click_goal =
                        Some(parsed.ok_or(format!("--click-goal {text}: use X,Y in metres"))?);
                }
                "--cubes" => {
                    options.cubes = value()?.parse().map_err(|e| format!("--cubes: {e}"))?;
                }
                "--seconds" => {
                    options.seconds =
                        Some(value()?.parse().map_err(|e| format!("--seconds: {e}"))?);
                }
                "--all-layers" => options.all_layers = true,
                "--screenshot" => options.screenshot = Some(value()?.into()),
                other => return Err(format!("unknown option {other}")),
            }
        }
        Ok(options)
    }
}

/// Where the news comes from: pretend data or ROS. Nothing else knows which.
#[derive(Resource)]
struct Source(Box<dyn Feed>);

/// One of the pool of cubes, and the band it's drawn as.
#[derive(Component)]
struct PoolCube {
    number: usize,
    look: Look,
}

/// What one cube in the pool is showing.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Look {
    Band(Band),
    MapDark,
    MapLit,
    NotOnMap,
}

/// The camera that draws the cubes.
#[derive(Component)]
struct CubesCamera;

/// The squiggly line's faint plan: its own gizmo group, for its own line width.
#[derive(Default, Reflect, GizmoConfigGroup)]
struct PlanAheadGizmos;

/// The squiggly line's bold trace and stop bar.
#[derive(Default, Reflect, GizmoConfigGroup)]
struct TraceGizmos;

/// The lines' widths, pixels.
const PLAN_AHEAD_WIDTH: f32 = 3.0;
const TRACE_WIDTH: f32 = 8.0;
/// How high over the road the squiggly line floats, metres, so the road cubes don't hide it.
const AHEAD_LIFT: f32 = 0.12;

/// One material per height band.
#[derive(Resource)]
struct LookMaterials(Vec<(Look, Handle<StandardMaterial>)>);

impl LookMaterials {
    fn of(&self, look: Look) -> Handle<StandardMaterial> {
        self.0
            .iter()
            .find(|(l, _)| *l == look)
            .map(|(_, m)| m.clone())
            .unwrap_or_default()
    }
}

fn rgb([r, g, b]: [u8; 3]) -> Color {
    Color::srgb_u8(r, g, b)
}

/// The car's picture, as a Bevy image.
#[derive(Resource)]
struct PictureImage(Handle<Image>);

/// The frame times of the speed test, after it has warmed up.
#[derive(Resource, Default)]
struct SpeedTest {
    frames: Vec<f32>,
    shot: bool,
}

/// How long `--click-goal` waits, seconds: Nav2 needs time to start.
const CLICK_AFTER: f32 = 20.0;
/// How long the speed test lets things settle before it counts, in seconds.
const WARM_UP: f32 = 3.0;

fn make_source(options: &Options) -> Result<Box<dyn Feed>, String> {
    if !options.live {
        return Ok(Box::new(pretend::PretendFeed::new(options.cubes)));
    }
    #[cfg(feature = "ros")]
    {
        ros::RosFeed::start(options.developer).map(|feed| Box::new(feed) as Box<dyn Feed>)
    }
    #[cfg(not(feature = "ros"))]
    {
        Err("--live needs the ros feature: use pixi run dev-window --live".to_owned())
    }
}

fn main() -> AppExit {
    let options = match Options::parse(std::env::args().skip(1)) {
        Ok(options) => options,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return AppExit::from_code(2);
        }
    };
    let source = match make_source(&options) {
        Ok(source) => source,
        Err(message) => {
            eprintln!("{message}");
            return AppExit::from_code(1);
        }
    };
    let present_mode = if options.seconds.is_some() {
        PresentMode::AutoNoVsync
    } else {
        PresentMode::AutoVsync
    };
    let mut ui_state = panels::UiState::default();
    match feed::lidar_mount() {
        Ok(eyes) => ui_state.eyes = eyes,
        Err(why) => ui_state.world.note(format!("the car's eyes: {why}")),
    }
    ui_state.layers.nav2_cubes = options.all_layers;
    ui_state.world.ndt = if options.live {
        options.ndt_map.as_deref().map(|path| {
            let file = MapFile::read(path, NDT_CELL);
            ui_state.world.note(file.to_string());
            file.cells().cloned()
        })
    } else {
        Some(Some(pretend::ndt_cells()))
    }
    .flatten();
    if let Some(path) = options.lanes.as_deref() {
        match lanes::read(path) {
            Ok(lines) => {
                ui_state
                    .world
                    .note(format!("{path}: {} lane lines", lines.len()));
                ui_state.world.lanes = lines;
            }
            Err(why) => ui_state.world.note(format!("{path}: {why}")),
        }
    }
    if let Some(path) = options.chunks.as_deref() {
        match map_chunks::files::read_grid(std::path::Path::new(path)) {
            Ok(grid) => {
                ui_state
                    .world
                    .note(format!("{path}: {} m chunks", grid.size()));
                ui_state.world.chunks = Some(grid);
                ui_state.world.chunk_folder = Some(path.into());
            }
            Err(why) => ui_state.world.note(format!("{path}: {why}")),
        }
    }

    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(options)
        .insert_resource(ui_state)
        .insert_resource(Source(source))
        .init_resource::<SpeedTest>()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "AAV developer window".into(),
                resolution: (1600, 1000).into(),
                present_mode,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .init_gizmo_group::<PlanAheadGizmos>()
        .init_gizmo_group::<TraceGizmos>()
        .add_systems(Startup, (setup, set_line_widths))
        .add_systems(
            Update,
            (
                take_news,
                (draw_cubes, draw_wireframes, draw_ahead, aim_camera),
                click_goal,
                speed_test,
            )
                .chain(),
        )
        .add_systems(EguiPrimaryContextPass, panels::draw)
        .run()
}

fn band_colour(band: Band) -> Color {
    rgb(match band {
        Band::Ground => panels::BAND_RGB[0],
        Band::Knee => panels::BAND_RGB[1],
        Band::Waist => panels::BAND_RGB[2],
        Band::Head => panels::BAND_RGB[3],
        Band::High => panels::BAND_RGB[4],
        Band::Above => panels::BAND_RGB[5],
    })
}

fn setup(
    mut commands: Commands,
    options: Res<Options>,
    mut global: ResMut<EguiGlobalSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    global.auto_create_primary_context = false;

    let colours = Band::ALL
        .iter()
        .map(|&band| (Look::Band(band), band_colour(band)))
        .chain([
            (Look::MapDark, rgb(panels::NDT_DARK_RGB)),
            (Look::MapLit, rgb(panels::NDT_LIT_RGB)),
            (Look::NotOnMap, rgb(panels::NEW_CUBE_RGB)),
        ]);
    let look_materials = LookMaterials(
        colours
            .map(|(look, colour)| {
                let material = materials.add(StandardMaterial {
                    base_color: colour,
                    unlit: true,
                    ..default()
                });
                (look, material)
            })
            .collect(),
    );

    // One mesh for all, so Bevy can batch them. Hidden until a scan gives them a place.
    let cube = meshes.add(Cuboid::from_length(POOL_CUBE));
    let start = look_materials.of(Look::Band(Band::Knee));
    let pool: Vec<_> = (0..options.cubes as usize)
        .map(|number| {
            (
                PoolCube {
                    number,
                    look: Look::Band(Band::Knee),
                },
                Mesh3d(cube.clone()),
                MeshMaterial3d(start.clone()),
                Transform::default(),
                Visibility::Hidden,
            )
        })
        .collect();
    commands.spawn_batch(pool);
    commands.insert_resource(look_materials);

    let picture = blank_image(16, 9);
    commands.insert_resource(PictureImage(images.add(picture)));

    // The world stays in ROS's axes: z up, so nothing needs converting.
    commands.spawn((
        CubesCamera,
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: PI / 4.0,
            ..default()
        }),
        Transform::from_xyz(-20.0, 0.0, 15.0).looking_at(Vec3::ZERO, Vec3::Z),
    ));
    // egui's own camera, drawn over the top and adding nothing else.
    commands.spawn((
        PrimaryEguiContext,
        Camera2d,
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: CameraOutputMode::Write {
                blend_state: Some(BlendState::ALPHA_BLENDING),
                clear_color: ClearColorConfig::None,
            },
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
    ));
}

fn blank_image(width: u32, height: u32) -> Image {
    Image::new_fill(
        Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[20, 20, 20, 255],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    )
}

/// Everything that arrived, into the window's state. The picture goes into its image.
fn take_news(
    mut source: ResMut<Source>,
    mut state: ResMut<panels::UiState>,
    handle: Res<PictureImage>,
    mut images: ResMut<Assets<Image>>,
) {
    source.0.frame_drawn();
    let now = Instant::now();
    for update in source.0.drain(now) {
        if let Update::Picture(picture) = update {
            state.world.picture_size = Some([picture.width, picture.height]);
            if let Some(mut image) = images.get_mut(&handle.0) {
                if image.width() != picture.width || image.height() != picture.height {
                    *image = blank_image(picture.width, picture.height);
                }
                image.data = Some(picture.rgba);
            }
        } else {
            state.world.apply(update, now);
        }
    }
}

/// Gives the pool of cubes the newest scan's places.
fn draw_cubes(
    state: Res<panels::UiState>,
    materials: Res<LookMaterials>,
    mut near: Local<NearCells>,
    mut pool: Query<(
        &mut PoolCube,
        &mut Transform,
        &mut Visibility,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
) {
    if !state.is_changed() {
        return;
    }
    near.refresh(&state.world);
    let shown = cubes_to_show(&state, &near.cells);
    pool.par_iter_mut()
        .for_each(|(mut cube, mut transform, mut visibility, mut material)| {
            match shown.get(cube.number) {
                Some(&(at, scale, look)) => {
                    transform.translation = at;
                    transform.scale = Vec3::splat(scale);
                    if *visibility != Visibility::Inherited {
                        *visibility = Visibility::Inherited;
                    }
                    if cube.look != look {
                        cube.look = look;
                        material.0 = materials.of(look);
                    }
                }
                None => {
                    if *visibility != Visibility::Hidden {
                        *visibility = Visibility::Hidden;
                    }
                }
            }
        });
}

/// NDT's cells around the car. A campus has too many to search every frame, so they're found
/// again only when the car has moved a little of the distance they reach.
#[derive(Default)]
struct NearCells {
    around: Option<Point>,
    cells: Vec<usize>,
}

impl NearCells {
    fn refresh(&mut self, world: &panels::WorldNews) {
        let Some(map) = &world.ndt else { return };
        let car = world.car_map.position;
        if self.around.is_some_and(|at| at.distance(car).0 < NEAR_MOVE) {
            return;
        }
        self.around = Some(car);
        self.cells = map
            .centres
            .iter()
            .enumerate()
            .filter(|(_, c)| {
                Point::new(f64::from(c[0]), f64::from(c[1])).distance(car).0 < NDT_REACH + NEAR_MOVE
            })
            .map(|(number, _)| number)
            .collect();
    }
}

/// What the cubes view shows, in the order the pool fills. With NDT's map and N on: one set of
/// cells, dark or lit, and only the cubes the map doesn't explain. Otherwise the raw cubes.
fn cubes_to_show(state: &panels::UiState, near: &[usize]) -> Vec<(Vec3, f32, Look)> {
    let world = &state.world;
    let road = state.layers.road.then_some(world.road.cubes.as_slice());
    let raw = |c: &debug_core::cubes::Cube| (Vec3::from_array(c.centre), 1.0, Look::Band(c.band));
    let mut shown: Vec<(Vec3, f32, Look)> = Vec::new();
    match (&world.ndt, state.layers.ndt_cubes) {
        (Some(cells), true) => {
            let scale = cells.size * CELL_FILL / POOL_CUBE;
            let is_lit = |number: usize| world.map_match.lit.get(number).copied().unwrap_or(false);
            let cell = |number: &usize, look| {
                cells
                    .centres
                    .get(*number)
                    .map(|centre| (cell_in_odom(world, centre), scale, look))
            };
            // Lit first: they matter most if the pool runs out.
            shown.extend(
                near.iter()
                    .filter(|&&number| is_lit(number))
                    .filter_map(|number| cell(number, Look::MapLit)),
            );
            shown.extend(
                world
                    .map_match
                    .off_map
                    .iter()
                    .filter_map(|&i| world.cubes.cubes.get(i))
                    .map(|c| (Vec3::from_array(c.centre), 1.0, Look::NotOnMap)),
            );
            // Kerbs and raised grass: the lidar's waves, in their own colours.
            shown.extend(
                world
                    .cubes
                    .cubes
                    .iter()
                    .filter(|c| c.centre[2] < panels::WAVES_TOP)
                    .map(raw),
            );
            shown.extend(
                near.iter()
                    .filter(|&&number| !is_lit(number))
                    .filter_map(|number| cell(number, Look::MapDark)),
            );
        }
        _ => shown.extend(world.cubes.cubes.iter().map(raw)),
    }
    if let Some(road) = road {
        shown.extend(road.iter().map(raw));
    }
    shown
}

/// A map cell's place in odom, where the cubes view lives: the opposite of odom to map.
fn cell_in_odom(world: &panels::WorldNews, centre: &[f32; 3]) -> Vec3 {
    let at = feed::odom_to_map([centre[0], centre[1]], world.car_map, world.car_odom);
    Vec3::new(at.x as f32, at.y as f32, centre[2])
}

/// The overlays: the ground's grid, the car from behind, and outlines of the live cubes.
fn draw_wireframes(state: Res<panels::UiState>, mut gizmos: Gizmos) {
    let world = &state.world;
    let car = world.car_odom;
    // A faint grid on the ground, so moving past nothing still looks like moving.
    let reach = GRID_STEP * GRID_LINES as f32;
    let snap = |v: f64| (v as f32 / GRID_STEP).round() * GRID_STEP;
    let (cx, cy) = (snap(car.position.x), snap(car.position.y));
    let grid = Color::srgb(0.13, 0.15, 0.16);
    for i in -GRID_LINES..=GRID_LINES {
        let d = i as f32 * GRID_STEP;
        gizmos.line(
            Vec3::new(cx + d, cy - reach, 0.0),
            Vec3::new(cx + d, cy + reach, 0.0),
            grid,
        );
        gizmos.line(
            Vec3::new(cx - reach, cy + d, 0.0),
            Vec3::new(cx + reach, cy + d, 0.0),
            grid,
        );
    }
    // From inside the car, its own box would only be in the way.
    if state.camera == panels::Eyes::Orbit {
        gizmos.cube(
            Transform::from_xyz(car.position.x as f32, car.position.y as f32, 0.7)
                .with_rotation(Quat::from_rotation_z(car.yaw as f32))
                .with_scale(Vec3::new(2.6, 1.2, 1.4)),
            Color::srgb(0.55, 0.75, 1.0),
        );
    }
    // Rings round the car, for reading distance; where it's going; and which way it faces.
    let at = Vec3::new(car.position.x as f32, car.position.y as f32, 0.05);
    if state.camera == panels::Eyes::Orbit {
        for radius in [10.0, 20.0, 40.0] {
            gizmos.circle(
                Isometry3d::from_translation(at),
                radius,
                Color::srgb(0.25, 0.28, 0.30),
            );
        }
        let heading = Vec3::new(car.yaw.cos() as f32, car.yaw.sin() as f32, 0.0);
        gizmos.arrow(at, at + heading * 3.0, Color::srgb(0.5, 1.0, 0.3));
    }
    if state.layers.plan {
        let line = world.plan.iter().map(|p| {
            let odom = feed::odom_to_map([p.x as f32, p.y as f32], world.car_map, car);
            Vec3::new(odom.x as f32, odom.y as f32, 0.1)
        });
        gizmos.linestrip(line, Color::srgb(1.0, 0.85, 0.2));
    }
    if state.layers.plan && world.road_ahead.line.len() >= 2 {
        // The road ahead, in the scene (odom): green where it allows the top speed, red at
        // a stop; a red bar across its end.
        let top = world.road_ahead.top_speed();
        let lift = |p: Point, z: f32| {
            let odom = feed::odom_to_map([p.x as f32, p.y as f32], world.car_map, car);
            Vec3::new(odom.x as f32, odom.y as f32, z)
        };
        gizmos.linestrip_gradient(world.road_ahead.line.iter().map(|&(p, v)| {
            let share = feed::speed_share(v, top);
            (
                lift(p, 0.15),
                Color::srgb(0.92 - 0.76 * share, 0.16 + 0.66 * share, 0.24),
            )
        }));
        if let Some((end, heading)) = world.road_ahead.end {
            let (sin, cos) = heading.sin_cos();
            let side = |s: f64| {
                lift(
                    Point::new(end.x - 1.5 * s * sin, end.y + 1.5 * s * cos),
                    0.15,
                )
            };
            gizmos.line(side(1.0), side(-1.0), Color::srgb(0.92, 0.16, 0.16));
        }
    }
    if state.layers.things {
        // The things near the car (odom): each an upright box in its kind's colour
        // (hud_core::things); their names are words over the scene (panels::thing_names).
        for thing in world.things_now(std::time::Instant::now()) {
            let c = thing.colour();
            gizmos.cube(
                Transform::from_xyz(thing.x as f32, thing.y as f32, thing.height as f32 / 2.0)
                    .with_scale(Vec3::new(
                        thing.length.max(0.1) as f32,
                        thing.width.max(0.1) as f32,
                        thing.height.max(0.1) as f32,
                    )),
                Color::srgb_u8(c.r, c.g, c.b),
            );
        }
    }
    if state.layers.nav2_cubes {
        for cube in world.cubes.cubes.iter().take(MAX_OUTLINES) {
            gizmos.cube(
                Transform::from_translation(Vec3::from_array(cube.centre))
                    .with_scale(Vec3::splat(pretend::CUBE_SIDE)),
                Color::srgb(0.9, 0.15, 0.15),
            );
        }
    }
}

/// The squiggly line's widths, and drawn over the cubes rather than behind them.
fn set_line_widths(mut store: ResMut<GizmoConfigStore>) {
    let plan = store.config_mut::<PlanAheadGizmos>().0;
    plan.line.width = PLAN_AHEAD_WIDTH;
    plan.depth_bias = -0.5;
    let trace = store.config_mut::<TraceGizmos>().0;
    trace.line.width = TRACE_WIDTH;
    trace.depth_bias = -0.5;
}

/// The squiggly line in the scene: ego's trace (bold, coloured by how well it keeps to the
/// plan), the plan ahead (faint), the stop bar. Sent in base_link: placed with the car in odom.
fn draw_ahead(
    state: Res<panels::UiState>,
    mut plan_gizmos: Gizmos<PlanAheadGizmos>,
    mut trace_gizmos: Gizmos<TraceGizmos>,
) {
    if !state.layers.ahead {
        return;
    }
    let world = &state.world;
    let Some(squiggle) = world.ahead_now(Instant::now()) else {
        return;
    };
    let car = world.car_odom;
    let place = |p: (f64, f64), lift: f32| {
        let at = feed::from_base(car, p);
        Vec3::new(at.x as f32, at.y as f32, lift)
    };
    if squiggle.plan.len() >= 2 {
        plan_gizmos.linestrip(
            squiggle.plan.iter().map(|p| place(*p, AHEAD_LIFT)),
            rgb(panels::AHEAD_PLAN_RGB),
        );
    }
    if squiggle.trace.len() >= 2 {
        trace_gizmos.linestrip(
            squiggle.trace.iter().map(|p| place(*p, AHEAD_LIFT + 0.02)),
            rgb(panels::trace_rgb(squiggle.summary.as_ref())),
        );
    }
    if let Some(stop) = squiggle.stop {
        let (sin, cos) = stop.yaw.sin_cos();
        let half = panels::STOP_BAR_HALF;
        let end = |side: f64| {
            place(
                (stop.x - side * sin, stop.y + side * cos),
                AHEAD_LIFT + 0.04,
            )
        };
        let colour = rgb(panels::AHEAD_STOP_RGB);
        trace_gizmos.line(end(half), end(-half), colour);
        // Two short posts, so the bar reads as a wall to stop at from behind the car.
        for side in [half, -half] {
            let foot = end(side);
            trace_gizmos.line(foot, foot + Vec3::Z * 0.5, colour);
        }
    }
}

/// Puts the camera where the view wants it: the car's own eyes, or behind it.
fn aim_camera(
    mut state: ResMut<panels::UiState>,
    camera: Single<(&mut Transform, &mut Projection), With<CubesCamera>>,
) {
    let (mut transform, mut projection) = camera.into_inner();
    let car = state.world.car_odom;
    state.orbit.update(Some(car));
    let (from, target, fov) = match state.camera {
        panels::Eyes::Driver => {
            let (from, target) = feed::driver_eye(car, state.eyes, state.look);
            (from, target, state.driver_fov)
        }
        panels::Eyes::Orbit => (state.orbit.position(), state.orbit.target, ORBIT_FOV),
    };
    *transform = Transform::from_translation(Vec3::from_array(from))
        .looking_at(Vec3::from_array(target), Vec3::Z);
    if let Projection::Perspective(perspective) = projection.as_mut() {
        perspective.fov = fov;
    }
}

/// `--click-goal`: the same refusal and gates as a person's click and two presses.
fn click_goal(
    time: Res<Time<Real>>,
    options: Res<Options>,
    mut source: ResMut<Source>,
    mut state: ResMut<panels::UiState>,
    mut done: Local<bool>,
) {
    let Some([x, y]) = options.click_goal else {
        return;
    };
    if *done || time.elapsed_secs() < CLICK_AFTER {
        return;
    }
    *done = true;
    panels::click_and_confirm(&mut state, source.0.as_mut(), Point::new(x, y));
}

/// Fits the 3D view into its panel. Called by the panels, which know where it is.
pub fn set_viewport(camera: &mut Camera, position: UVec2, size: UVec2) {
    camera.viewport = Some(Viewport {
        physical_position: position,
        physical_size: size.max(UVec2::ONE),
        ..default()
    });
}

/// `--seconds`: counts frames after the warm-up, prints the rate and exits.
fn speed_test(
    mut commands: Commands,
    time: Res<Time<Real>>,
    options: Res<Options>,
    mut test: ResMut<SpeedTest>,
    mut exit: MessageWriter<AppExit>,
) {
    let elapsed = time.elapsed_secs();
    let shot_at = WARM_UP + options.seconds.unwrap_or(0.0) / 2.0;
    if !test.shot && elapsed > shot_at {
        if let Some(path) = &options.screenshot {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            test.shot = true;
        }
    }
    let Some(seconds) = options.seconds else {
        return;
    };
    if elapsed > WARM_UP {
        test.frames.push(time.delta_secs());
    }
    if elapsed > WARM_UP + seconds {
        let mut frames = test.frames.clone();
        frames.sort_by(f32::total_cmp);
        let total: f32 = frames.iter().sum();
        let worst = frames
            .get(frames.len().saturating_mul(99) / 100)
            .copied()
            .unwrap_or(0.0);
        println!(
            "cubes {}: {:.1} fps on average, slowest 1% of frames {:.1} ms, over {} frames",
            options.cubes,
            frames.len() as f32 / total.max(f32::EPSILON),
            worst * 1000.0,
            frames.len(),
        );
        exit.write(AppExit::Success);
    }
}

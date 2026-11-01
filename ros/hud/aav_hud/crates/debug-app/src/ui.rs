//! The debug screen, drawn with raylib: the 3D view on the left; on the right, the
//! seeing screen's two top-downs (the default) or, with key L, the panel.
//!
//! The seeing screen (docs/designs/2026-10-29_two-screens.md):
//!
//! ```text
//! +-------------------------------------+------------------------+
//! | LEGEND                          fps | WHAT THE CAR SEES  SIM |
//! |                                     | the lidar map from     |
//! |   cubes (kept, fading), the car,    | above, every car       |
//! |   the localizers, the plan          +------------------------+
//! |                                     | NDT's match: ellipses, |
//! | KEYS                                | the scan, the score    |
//! +-------------------------------------+------------------------+
//! ```
//!
//! The panel (key L):
//!
//! ```text
//! +-------------------------------------------------+-----------------+
//! | LEGEND (localizers, offsets; cube heights)  fps | DEBUG HUD   SIM |
//! |                                                 | SENSORS         |
//! |        cubes (kept, fading), the car, the       |  IMU    OK 0.0s |
//! |        localizers, AMCL's particles, NDT's      |  ...            |
//! |        map, Nav2's plan, the trail              | VEHICLE         |
//! | KEYS (H hides them)                             | TRACKER         |
//! |  K cubes      kept, fading                      | NDT             |
//! |  M NDT's map  on ...                            | LIDAR           |
//! +-------------------------------------------------+-----------------+
//! ```
//!
//! The world is odom, z up. The cubes are drawn instanced: one cube mesh, one draw per
//! height band and fade step, a small shader that shades each face by its normal. A
//! fading cube is drawn darker (towards the background), not see-through, so nothing
//! needs sorting. NDT's map is drawn through map -> odom with rlgl's matrix stack.

use std::time::Instant;

use debug_core::{
    cubes::Band,
    keys::{CubeView, Layout},
    ndt::{Fit, MapCells, MapFile, NdtState},
    orbit::Orbit,
    persist::{Fade, FadedCube},
    state::{DebugState, Localizer, Source},
    topdown::{Rect as TopRect, View},
};
use hud_core::{health::Health, vehicle::VehicleStatus};
use map_core::{
    car::CarShape,
    geom::{Point, Pose},
};
use raylib::prelude::*;

/// The panel's width, in window points.
pub const PANEL_W: f32 = 460.0;

const BACKGROUND: Color = Color::new(12, 14, 20, 255);
const PANEL: Color = Color::new(22, 25, 33, 255);
const SECTION: Color = Color::new(31, 35, 46, 255);
const CAPTION: Color = Color::new(140, 150, 170, 255);
const TEXT: Color = Color::new(240, 242, 246, 255);
const DIM: Color = Color::new(105, 110, 125, 255);
const GOOD: Color = Color::new(60, 200, 90, 255);
const WARN: Color = Color::new(245, 180, 40, 255);
const BAD: Color = Color::new(230, 60, 60, 255);
const NOT_REAL: Color = Color::new(255, 140, 0, 255);
const GRID: Color = Color::new(34, 39, 50, 255);
const GRID_MAJOR: Color = Color::new(50, 57, 72, 255);
const PLAN: Color = Color::new(255, 220, 60, 255);
const TRAIL: Color = Color::new(120, 125, 140, 255);
/// NDT's map: faint purple wireframes.
const NDT_MAP: Color = Color::new(150, 110, 220, 70);
/// AMCL's particles: small orange arrows, a little fainter than AMCL's own car.
const PARTICLE: Color = Color::new(255, 150, 40, 170);
/// The seeing screen: NDT's map's points from above.
const MAP_POINT: Color = Color::new(170, 140, 230, 200);
/// NDT's cells, as ellipses round the car.
const NDT_CELL: Color = Color::new(170, 120, 240, 150);
/// The latest scan's cubes from above: on a map cell, or off the map.
const SCAN_ON: Color = Color::new(80, 230, 120, 255);
const SCAN_OFF: Color = Color::new(240, 80, 80, 255);
/// NDT's match is shown this many metres across, round the car.
const MATCH_WIDTH: f32 = 24.0;
/// The most NDT map cubes drawn, nearest the car (the test lot has about 2,500).
const MAX_MAP_CUBES: usize = 6000;
/// A particle's arrow, metres.
const PARTICLE_LEN: f32 = 0.45;

/// Each localizer's colour (the design's: local blue, global green, truth white, AMCL
/// orange; NDT purple).
pub fn localizer_colour(who: Localizer) -> Color {
    match who {
        Localizer::Local => Color::new(80, 130, 255, 255),
        Localizer::Global => Color::new(70, 225, 100, 255),
        Localizer::Truth => Color::new(245, 245, 245, 255),
        Localizer::Amcl => Color::new(255, 150, 40, 255),
        Localizer::Ndt => Color::new(190, 110, 255, 255),
    }
}

/// A band's colour at a fade step: darker, towards the background, as it fades.
pub fn faded_colour(band: Band, fade: Fade) -> Color {
    let (c, k) = (band_colour(band), fade.brightness());
    let mix = |a: u8, b: u8| (f32::from(a) * k + f32::from(b) * (1.0 - k)).round() as u8;
    Color::new(
        mix(c.r, BACKGROUND.r),
        mix(c.g, BACKGROUND.g),
        mix(c.b, BACKGROUND.b),
        255,
    )
}

/// Each height band's colour: a cool ramp, dark ground to pale tops, so the
/// localizers' bright colours stand out on it.
pub fn band_colour(band: Band) -> Color {
    match band {
        Band::Ground => Color::new(48, 56, 70, 255),
        Band::Knee => Color::new(35, 98, 128, 255),
        Band::Waist => Color::new(38, 140, 160, 255),
        Band::Head => Color::new(70, 180, 190, 255),
        Band::High => Color::new(130, 210, 215, 255),
        Band::Above => Color::new(200, 238, 240, 255),
    }
}

/// Fonts tried in order; raylib's own small pixel font if none is there.
const FONT_FILES: [&str; 4] = [
    "/System/Library/Fonts/Supplemental/Verdana Bold.ttf", // macOS
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",   // macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", // Ubuntu, Raspberry Pi OS
    "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf", // Fedora, AlmaLinux
];
const FONT_LOAD_SIZE: i32 = 96;

/// The cube shader: each instance's transform, a face shade from its normal.
const CUBE_VS: &str = r#"#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in mat4 instanceTransform;
uniform mat4 mvp;
out float shade;
void main() {
    vec3 n = normalize(vertexNormal);
    // Lit from above and a little to one side: tops brightest, sides apart.
    shade = 0.45 + 0.4 * max(n.z, 0.0) + 0.15 * abs(n.x) + 0.05 * abs(n.y);
    gl_Position = mvp * instanceTransform * vec4(vertexPosition, 1.0);
}
"#;
const CUBE_FS: &str = r#"#version 330
in float shade;
uniform vec4 colDiffuse;
out vec4 finalColor;
void main() {
    finalColor = vec4(colDiffuse.rgb * shade, colDiffuse.a);
}
"#;

/// raylib's font handle: one loaded from a file, or its built-in one.
enum UiFont {
    File(Font),
    BuiltIn(WeakFont),
}

impl AsRef<raylib::ffi::Font> for UiFont {
    fn as_ref(&self) -> &raylib::ffi::Font {
        match self {
            Self::File(f) => f.as_ref(),
            Self::BuiltIn(f) => f.as_ref(),
        }
    }
}

impl UiFont {
    fn measure(&self, text: &str, size: f32) -> f32 {
        let spacing = size / 24.0;
        match self {
            Self::File(f) => f.measure_text(text, size, spacing).x,
            Self::BuiltIn(f) => f.measure_text(text, size, spacing).x,
        }
    }
}

/// One instanced draw per band and fade step: its material (the shader, the colour),
/// and the transforms gathered this frame.
struct BandBatch {
    band: Band,
    fade: Fade,
    material: WeakMaterial,
    transforms: Vec<Matrix>,
}

/// What a frame needs besides the state.
pub struct Frame<'a> {
    pub now: Instant,
    pub label: &'a str,
    pub orbit: &'a Orbit,
    pub fps: u32,
    /// The drawing test: say so on screen.
    pub bench: Option<usize>,
    /// NDT's map, or why there is none.
    pub map: &'a MapFile,
}

/// The screen: its font, the cube mesh and shader.
pub struct Ui {
    font: UiFont,
    cube: Mesh,
    /// Kept alive while the materials point at it.
    _shader: Shader,
    batches: Vec<BandBatch>,
    /// The size the cube mesh was made for.
    cube_size: f32,
    car: CarShape,
}

impl Ui {
    /// Loads the font, the shader and the cube. Needs the window open.
    pub fn new(
        rl: &mut RaylibHandle,
        thread: &RaylibThread,
        car: CarShape,
        cube_size: f32,
    ) -> Self {
        let loaded = FONT_FILES
            .iter()
            .filter(|path| std::path::Path::new(path).exists())
            .find_map(|path| rl.load_font_ex(thread, path, FONT_LOAD_SIZE, None).ok());
        let font = match loaded {
            Some(f) => {
                f.texture()
                    .set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_BILINEAR);
                UiFont::File(f)
            }
            None => UiFont::BuiltIn(rl.get_font_default()),
        };
        let shader = rl.load_shader_from_memory(thread, Some(CUBE_VS), Some(CUBE_FS));
        let batches = Band::ALL
            .iter()
            .flat_map(|&band| Fade::ALL.iter().map(move |&fade| (band, fade)))
            .map(|(band, fade)| {
                let mut material = rl.load_material_default(thread);
                // SAFETY: a copy of the shader's handle; `_shader` keeps it loaded for as
                // long as the materials (and so the window) live.
                *material.shader_mut() = unsafe { WeakShader::from_raw(*shader.as_ref()) };
                material.maps_mut()[MaterialMapIndex::MATERIAL_MAP_ALBEDO as usize].color =
                    faded_colour(band, fade);
                BandBatch {
                    band,
                    fade,
                    material,
                    transforms: Vec::new(),
                }
            })
            .collect();
        // A hair smaller than the grid, so neighbours show an edge between them.
        let edge = cube_size * 0.92;
        let cube = Mesh::gen_mesh_cube(thread, edge, edge, edge);
        Self {
            font,
            cube,
            _shader: shader,
            batches,
            cube_size,
            car,
        }
    }

    /// Draws one frame.
    pub fn draw(&mut self, d: &mut RaylibDrawHandle, state: &DebugState, f: &Frame<'_>) {
        d.clear_background(BACKGROUND);
        let (w, h) = (d.get_screen_width() as f32, d.get_screen_height() as f32);
        let view_w = view_width(w, state.layers.layout);
        let size = state.scan.as_ref().map_or(self.cube_size, |s| s.size);
        self.gather(state.drawn_cubes(), size);
        self.world(d, state, f, view_w, h);
        self.legend(d, state, f, view_w);
        if state.layers.keys {
            self.key_legend(d, state, f, h);
        } else {
            self.text(d, "H: keys", 16.0, h - 34.0, 18.0, DIM);
        }
        match state.layers.layout {
            Layout::Panel => self.panel(d, state, f, view_w, w, h),
            Layout::Seeing => self.seeing(d, state, f, view_w, w, h),
        }
    }

    /// The seeing screen's right side: NDT's map from above with every localizer's car
    /// (top); NDT's match round the car: its cells as ellipses, the latest scan over
    /// them, the score (bottom).
    fn seeing(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        x0: f32,
        w: f32,
        h: f32,
    ) {
        d.draw_rectangle_rec(Rectangle::new(x0, 0.0, w - x0, h), PANEL);
        let (x, right) = (x0 + 18.0, w - 18.0);
        let mut y = 14.0;
        self.text(d, "WHAT THE CAR SEES", x, y, 28.0, TEXT);
        let label_w = self.font.measure(f.label, 20.0);
        let tag = Rectangle::new(right - label_w - 20.0, y + 2.0, label_w + 20.0, 30.0);
        d.draw_rectangle_rounded(tag, 0.3, 6, SECTION);
        self.text(d, f.label, tag.x + 10.0, y + 6.0, 20.0, CAPTION);
        y += 44.0;
        if let Some(banner) = state.origins.banner() {
            let r = Rectangle::new(x0 + 10.0, y, w - x0 - 20.0, 32.0);
            d.draw_rectangle_rounded(r, 0.25, 6, NOT_REAL);
            self.text(d, &banner, x + 4.0, y + 6.0, 19.0, Color::BLACK);
            y += 40.0;
        }
        // Two boxes of about the same height; the score's lines under the second.
        let score_h = 96.0;
        let boxes = (h - y - score_h - 2.0 * 34.0 - 16.0).max(120.0);
        let top_h = boxes / 2.0;
        y = self.section(d, "THE LIDAR MAP FROM ABOVE: EVERY LOCALIZER", x0, y, w) - 6.0;
        let top = TopRect::new(x0 + 10.0, y, w - x0 - 20.0, top_h);
        self.map_from_above(d, state, f, top);
        y += top_h + 8.0;
        y = self.section(d, "NDT'S MATCH: THE SCAN ON THE MAP'S CELLS", x0, y, w) - 6.0;
        let bottom = TopRect::new(x0 + 10.0, y, w - x0 - 20.0, boxes - top_h);
        let on_map = self.ndt_match(d, state, f, bottom);
        y += boxes - top_h + 8.0;
        self.match_score(d, state, f, on_map, x, right, y);
    }

    /// NDT's map's points from above, fitted to the box, and each localizer's car.
    fn map_from_above(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        r: TopRect,
    ) {
        d.draw_rectangle_rec(Rectangle::new(r.x, r.y, r.w, r.h), BACKGROUND);
        let Some(cells) = f.map.cells() else {
            self.no_map(d, f, r);
            return;
        };
        let fp = &cells.footprint;
        let Some(bounds) = fp.bounds else {
            self.text(
                d,
                "the map has no points",
                r.x + 12.0,
                r.y + 12.0,
                18.0,
                WARN,
            );
            return;
        };
        let view = View::fit(bounds, r);
        let mut s = d.begin_scissor_mode(r.x as i32, r.y as i32, r.w as i32, r.h as i32);
        let dot = (view.scale * 0.15).clamp(1.5, 3.0);
        for &[px, py] in &fp.points {
            let [sx, sy] = view.to_screen(Point::new(f64::from(px), f64::from(py)));
            s.draw_rectangle_v(
                Vector2::new(sx - dot / 2.0, sy - dot / 2.0),
                Vector2::new(dot, dot),
                MAP_POINT,
            );
        }
        self.cars_from_above(&mut s, state, &view);
    }

    /// Every localizer's car from above, the local EKF's on top.
    fn cars_from_above(&self, d: &mut RaylibDrawHandle, state: &DebugState, view: &View) {
        for who in [
            Localizer::Truth,
            Localizer::Amcl,
            Localizer::Ndt,
            Localizer::Global,
            Localizer::Local,
        ] {
            if let Some(pose) = state.pose_in_map(who) {
                car_from_above(d, &self.car, view, pose, localizer_colour(who));
            }
        }
    }

    /// NDT's cells round the car as ellipses, the latest scan's cubes over them (green
    /// on a cell, red off the map), the car. Returns how many of the scan's cubes were
    /// on the map, and of how many.
    fn ndt_match(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        r: TopRect,
    ) -> Option<(usize, usize)> {
        d.draw_rectangle_rec(Rectangle::new(r.x, r.y, r.w, r.h), BACKGROUND);
        let Some(cells) = f.map.cells() else {
            self.no_map(d, f, r);
            return None;
        };
        // Round NDT's own car when it has one, else the global EKF's, else the local.
        let Some(car) = [Localizer::Ndt, Localizer::Global, Localizer::Local]
            .into_iter()
            .find_map(|who| state.pose_in_map(who))
        else {
            let s = "no car on the map yet (map -> odom, or a map-frame localizer)";
            let s = fit(&self.font, s, r.w - 24.0, 17.0);
            self.text(d, &s, r.x + 12.0, r.y + 12.0, 17.0, DIM);
            return None;
        };
        let fp = &cells.footprint;
        let view = View::around(car.position, MATCH_WIDTH, r);
        let mut s = d.begin_scissor_mode(r.x as i32, r.y as i32, r.w as i32, r.h as i32);
        for e in fp.ellipses_near(car.position, view.half_span() + 1.0) {
            let ring: Vec<Vector2> = view
                .ellipse(e.centre, e.half_axes, e.angle, 18)
                .into_iter()
                .map(|[x, y]| Vector2::new(x, y))
                .collect();
            s.draw_line_strip(&ring, NDT_CELL);
        }
        let scan = state.scan_in_map();
        let on = scan.iter().filter(|&&p| fp.covers(p)).count();
        let dot = (view.scale * 0.2).clamp(2.0, 5.0);
        for &p in scan.iter().filter(|&&p| view.shows(p)) {
            let [sx, sy] = view.to_screen(p);
            let colour = if fp.covers(p) { SCAN_ON } else { SCAN_OFF };
            s.draw_rectangle_v(
                Vector2::new(sx - dot / 2.0, sy - dot / 2.0),
                Vector2::new(dot, dot),
                colour,
            );
        }
        self.cars_from_above(&mut s, state, &view);
        drop(s);
        let note = format!("{MATCH_WIDTH:.0} m across, round the car");
        self.text(d, &note, r.x + 10.0, r.y + r.h - 24.0, 15.0, DIM);
        (!scan.is_empty()).then_some((on, scan.len()))
    }

    /// Under NDT's match: its state, score per point and time, and how much of the scan
    /// landed on the map.
    #[allow(clippy::too_many_arguments)]
    fn match_score(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        on_map: Option<(usize, usize)>,
        x: f32,
        right: f32,
        y: f32,
    ) {
        let fresh = state.health(Source::NdtStatus, f.now).is_ok();
        let (word, colour, numbers) = match &state.ndt {
            Some(s) if fresh => {
                let colour = match (s.headline(), s.fit) {
                    ("LOST", _) => BAD,
                    ("TRACKING", Fit::Good) => GOOD,
                    _ => WARN,
                };
                (s.headline(), colour, s.numbers())
            }
            Some(_) => ("STALE", DIM, String::new()),
            None => ("NOT RUNNING", SECTION, String::new()),
        };
        let ww = self.font.measure(word, 18.0);
        d.draw_rectangle_rounded(Rectangle::new(x, y, ww + 20.0, 28.0), 0.4, 6, colour);
        let text_colour = if colour == SECTION { DIM } else { Color::BLACK };
        self.text(d, word, x + 10.0, y + 4.0, 18.0, text_colour);
        let nx = x + ww + 34.0;
        if !numbers.is_empty() {
            let numbers = fit(&self.font, &numbers, right - nx, 22.0);
            self.text(d, &numbers, nx, y + 2.0, 22.0, TEXT);
        } else if state.ndt.is_none() {
            let hint = fit(&self.font, "start the sim with ndt:=true", right - nx, 17.0);
            self.text(d, &hint, nx, y + 5.0, 17.0, DIM);
        }
        let mut line = Vec::new();
        if let Some((on, of)) = on_map {
            line.push(format!(
                "{on} of {of} scan cubes on the map ({:.0}%)",
                100.0 * on as f64 / of.max(1) as f64
            ));
        }
        if let Some(s) = state.ndt.as_ref().filter(|_| fresh) {
            let details = s.details();
            if !details.is_empty() {
                line.push(details);
            }
        }
        let line = fit(&self.font, &line.join("   "), right - x, 17.0);
        self.text(d, &line, x, y + 38.0, 17.0, CAPTION);
        let key = "green: on a map cell   red: off the map   rings: NDT's cells (2 sigma)";
        let key = fit(&self.font, key, right - x, 15.0);
        self.text(d, &key, x, y + 64.0, 15.0, DIM);
    }

    /// In a top-down's box: there is no NDT map, and how to make one.
    fn no_map(&self, d: &mut RaylibDrawHandle, f: &Frame<'_>, r: TopRect) {
        let line = f.map.to_string();
        let mut y = r.y + 12.0;
        for l in wrap(&self.font, &line, r.w - 24.0, 17.0)
            .into_iter()
            .take(4)
        {
            self.text(d, &l, r.x + 12.0, y, 17.0, WARN);
            y += 23.0;
        }
    }

    /// This frame's cube transforms, by band and fade step. `size`: the cubes' edge.
    fn gather(&mut self, cubes: impl Iterator<Item = FadedCube>, size: f32) {
        for b in &mut self.batches {
            b.transforms.clear();
        }
        let k = if self.cube_size > 0.0 {
            size / self.cube_size
        } else {
            1.0
        };
        let fades = Fade::ALL.len();
        for c in cubes {
            let [x, y, z] = c.centre;
            let m = if (k - 1.0).abs() < 1e-3 {
                Matrix::translate(x, y, z)
            } else {
                Matrix::scale(k, k, k) * Matrix::translate(x, y, z)
            };
            // The batches go band by band, each with every fade step in order.
            let band = Band::ALL.iter().position(|&b| b == c.band).unwrap_or(0);
            let fade = Fade::ALL.iter().position(|&f| f == c.fade).unwrap_or(0);
            if let Some(b) = self.batches.get_mut(band * fades + fade) {
                debug_assert!(b.band == c.band && b.fade == c.fade);
                b.transforms.push(m);
            }
        }
    }

    /// The 3D view, in the left part of the window.
    fn world(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        view_w: f32,
        h: f32,
    ) {
        let orbit = f.orbit;
        let [px, py, pz] = orbit.position();
        let [tx, ty, tz] = orbit.target;
        let camera = Camera3D::perspective(
            Vector3::new(px, py, pz),
            Vector3::new(tx, ty, tz),
            Vector3::new(0.0, 0.0, 1.0),
            50.0,
        );
        // The framebuffer's pixels per window point (2 on a Retina screen). raylib keeps
        // the render size up to date on a resize; rlgl's own framebuffer size it doesn't.
        let (fb_w, fb_h) = (d.get_render_width(), d.get_render_height());
        let k = fb_w as f32 / d.get_screen_width().max(1) as f32;
        let mut d3 = d.begin_mode3D(camera);
        viewport(view_w * k, h * k, 50.0);

        ground_grid(&mut d3, orbit.target, orbit.distance);
        if state.layers.ndt_map {
            if let Some(cells) = f.map.cells() {
                ndt_map(&mut d3, state, cells);
            }
        }
        for b in &self.batches {
            if !b.transforms.is_empty() {
                d3.draw_mesh_instanced(&self.cube, b.material.clone(), &b.transforms);
            }
        }
        // The trail, then the plan, on the ground.
        let trail: Vec<Point> = state.trail().collect();
        for pair in trail.windows(2) {
            d3.draw_line3D(v3(pair[0], 0.03), v3(pair[1], 0.03), TRAIL);
        }
        // Nav2's points are ~5 cm apart: one tube per 0.5 m is plenty.
        let plan = thin(state.plan_in_odom(), 0.5);
        for pair in plan.windows(2) {
            d3.draw_cylinder_ex(v3(pair[0], 0.06), v3(pair[1], 0.06), 0.07, 0.07, 6, PLAN);
        }
        if state.layers.particles {
            for p in state.particles_in_odom() {
                particle(&mut d3, p);
            }
        }
        // The localizers: map-frame ones first, so the local EKF's car sits on top.
        for (i, who) in [
            Localizer::Truth,
            Localizer::Amcl,
            Localizer::Ndt,
            Localizer::Global,
        ]
        .into_iter()
        .enumerate()
        {
            if let Some(pose) = state.pose_in_odom(who) {
                marker(
                    &mut d3,
                    &self.car,
                    pose,
                    localizer_colour(who),
                    0.04 + 0.02 * i as f32,
                );
            }
        }
        if let Some(pose) = state.car() {
            car_body(&mut d3, &self.car, pose, localizer_colour(Localizer::Local));
        }
        drop(d3);
        // Back to the whole window for the 2D parts.
        // SAFETY: an rlgl state call on the drawing thread, after the 3D part.
        unsafe { raylib::ffi::rlViewport(0, 0, fb_w, fb_h) };
    }

    /// Over the 3D view: what each colour means, the camera, the keys.
    fn legend(&self, d: &mut RaylibDrawHandle, state: &DebugState, f: &Frame<'_>, view_w: f32) {
        let x = 16.0;
        let mut y = 16.0;
        let box_h = 34.0 + 30.0 * Localizer::ALL.len() as f32 + 12.0 + 26.0 * 3.0;
        d.draw_rectangle_rounded(
            Rectangle::new(x - 6.0, y - 6.0, 330.0, box_h),
            0.06,
            8,
            Color::new(16, 18, 26, 215),
        );
        self.text(d, "WHERE THE CAR IS", x + 6.0, y, 18.0, CAPTION);
        y += 30.0;
        for who in Localizer::ALL {
            let colour = localizer_colour(who);
            let seen = state.pose_in_odom(who).is_some();
            d.draw_rectangle_rounded(Rectangle::new(x + 6.0, y + 3.0, 20.0, 20.0), 0.3, 4, colour);
            let words = if !seen {
                format!("{}  -", who.label())
            } else if who == Localizer::Local {
                who.label().to_owned()
            } else {
                match state.offset_from_local(who) {
                    Some(m) => format!("{}  {m:.2} m off", who.label()),
                    None => who.label().to_owned(),
                }
            };
            self.text(d, &words, x + 36.0, y, 22.0, if seen { TEXT } else { DIM });
            y += 30.0;
        }
        y += 8.0;
        let caption = match state.layers.cubes {
            CubeView::Kept => "CUBES: HEIGHT (DARKER = OLDER)",
            CubeView::Scan => "CUBES: HEIGHT, THIS SCAN",
        };
        self.text(d, caption, x + 6.0, y, 16.0, CAPTION);
        y += 24.0;
        for (i, band) in Band::ALL.into_iter().enumerate() {
            let col = i % 3;
            let row = i / 3;
            let bx = x + 6.0 + col as f32 * 104.0;
            let by = y + row as f32 * 26.0;
            d.draw_rectangle_rounded(
                Rectangle::new(bx, by + 3.0, 16.0, 16.0),
                0.2,
                4,
                band_colour(band),
            );
            self.text(d, band.label(), bx + 22.0, by, 16.0, TEXT);
        }

        // Top right of the view: the frame rate, and the camera's mode.
        let fps = format!("{} fps   {}", f.fps, f.orbit.mode.label());
        let fw = self.font.measure(&fps, 20.0);
        self.text(d, &fps, view_w - fw - 16.0, 16.0, 20.0, CAPTION);
        if let Some(n) = f.bench {
            let s = format!("DRAWING TEST: {n} cubes");
            let sw = self.font.measure(&s, 26.0);
            self.text(d, &s, (view_w - sw) / 2.0, 16.0, 26.0, WARN);
        }
    }

    /// Bottom left of the view: every key, what it does, and what it is now.
    fn key_legend(&self, d: &mut RaylibDrawHandle, state: &DebugState, f: &Frame<'_>, h: f32) {
        let lines = state.layers.legend(f.orbit.mode, f.map.cells().is_some());
        let row = 24.0;
        let box_h = 30.0 + row * lines.len() as f32 + 6.0;
        let (x, top) = (16.0, h - box_h - 10.0);
        d.draw_rectangle_rounded(
            Rectangle::new(x - 6.0, top, 500.0, box_h),
            0.06,
            8,
            Color::new(16, 18, 26, 215),
        );
        self.text(d, "KEYS (H HIDES THEM)", x + 6.0, top + 6.0, 16.0, CAPTION);
        let mut y = top + 30.0;
        for line in &lines {
            self.text(d, line.key, x + 6.0, y, 17.0, TEXT);
            self.text(d, line.what, x + 80.0, y, 17.0, CAPTION);
            if !line.now.is_empty() {
                let colour = if line.now == "no map" { WARN } else { TEXT };
                self.text(d, &line.now, x + 266.0, y, 17.0, colour);
            }
            y += row;
        }
    }

    /// The panel: every sensor from its own topic, the vehicle, the tracker, the lidar.
    fn panel(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        x0: f32,
        w: f32,
        h: f32,
    ) {
        d.draw_rectangle_rec(Rectangle::new(x0, 0.0, w - x0, h), PANEL);
        let x = x0 + 18.0;
        let right = w - 18.0;
        let mut y = 14.0;
        self.text(d, "DEBUG HUD", x, y, 32.0, TEXT);
        let label_w = self.font.measure(f.label, 22.0);
        let tag = Rectangle::new(right - label_w - 20.0, y + 4.0, label_w + 20.0, 32.0);
        d.draw_rectangle_rounded(tag, 0.3, 6, SECTION);
        self.text(d, f.label, tag.x + 10.0, y + 8.0, 22.0, CAPTION);
        y += 48.0;
        if let Some(banner) = state.origins.banner() {
            let r = Rectangle::new(x0 + 10.0, y, w - x0 - 20.0, 34.0);
            d.draw_rectangle_rounded(r, 0.25, 6, NOT_REAL);
            self.text(d, &banner, x + 4.0, y + 6.0, 20.0, Color::BLACK);
            y += 44.0;
        }

        y = self.section(d, "SENSORS AND ESTIMATES", x0, y, w);
        for source in Source::PANEL {
            let health = state.health(source, f.now);
            self.text(d, source.label(), x, y, 19.0, TEXT);
            let (word, colour) = health_word(health);
            let pill = Rectangle::new(x + 148.0, y + 1.0, 78.0, 22.0);
            d.draw_rectangle_rounded(pill, 0.4, 6, colour);
            let ww = self.font.measure(word, 15.0);
            self.text(
                d,
                word,
                pill.x + (pill.width - ww) / 2.0,
                y + 4.0,
                15.0,
                Color::BLACK,
            );
            let detail = match (health.age(), state.detail(source)) {
                (_, Some(text)) if health.is_ok() => text.to_owned(),
                (Some(age), _) => format!("{:.1} s ago", age.as_secs_f32()),
                (None, _) => String::new(),
            };
            let detail = fit(&self.font, &detail, right - (x + 240.0), 17.0);
            self.text(
                d,
                &detail,
                x + 240.0,
                y + 2.0,
                17.0,
                if health.is_ok() { CAPTION } else { DIM },
            );
            y += 26.0;
        }

        y = self.section(d, "VEHICLE", x0, y + 4.0, w);
        let vehicle_ok = state.health(Source::Vehicle, f.now).is_ok();
        match state.vehicle {
            Some(v) => y = self.vehicle(d, &v, x, right, y, vehicle_ok),
            None => {
                self.text(d, "no /vehicle/status yet", x, y, 20.0, DIM);
                y += 32.0;
            }
        }

        y = self.section(d, "TRACKER", x0, y + 4.0, w);
        match &state.track {
            Some(t) => {
                let colour = if state.health(Source::Track, f.now).is_ok() {
                    TEXT
                } else {
                    DIM
                };
                self.text(d, &t.tracker, x, y, 24.0, colour);
                let mut parts = Vec::new();
                if let Some(e) = t.cross_track {
                    parts.push(format!("{e:+.2} m off the path"));
                }
                if let Some(ms) = t.loop_ms {
                    parts.push(format!("{ms:.1} ms"));
                }
                let s = parts.join("   ");
                self.text(d, &s, x + 160.0, y + 4.0, 18.0, CAPTION);
                y += 34.0;
            }
            None => {
                self.text(d, "none (Nav2's own controller)", x, y, 20.0, DIM);
                y += 32.0;
            }
        }
        if let Some(reply) = &state.swap_reply {
            let colour = if reply.outcome.is_ok() { GOOD } else { WARN };
            let s = fit(&self.font, &reply.line(), right - x, 17.0);
            self.text(d, &s, x, y, 17.0, colour);
            y += 26.0;
        }

        y = self.ndt(d, state, f, x0, y + 4.0, w);

        y = self.section(d, "LIDAR CUBES", x0, y + 4.0, w);
        match &state.scan {
            Some(scan) => {
                let rate = state
                    .scan_rate()
                    .map_or_else(String::new, |hz| format!("   {hz:.0} Hz"));
                let (drawn, cut) = state.drawn_count();
                let s = format!(
                    "{drawn} cubes ({}) from {} points{rate}",
                    state.layers.cubes.label(),
                    scan.points
                );
                let s = fit(&self.font, &s, right - x, 19.0);
                self.text(d, &s, x, y, 19.0, TEXT);
                y += 26.0;
                if cut > 0 {
                    let s = format!("{cut} far cubes cut (max_cubes)");
                    self.text(d, &s, x, y, 17.0, WARN);
                    y += 24.0;
                }
            }
            None => {
                self.text(d, "no scan yet", x, y, 20.0, DIM);
                y += 28.0;
            }
        }
        for problem in [&state.scan_problem, &state.frame_problem]
            .into_iter()
            .flatten()
        {
            let s = fit(&self.font, problem, right - x, 17.0);
            self.text(d, &s, x, y, 17.0, WARN);
            y += 24.0;
        }
        let note = "heights above base_link (a 2D EKF)";
        if y < h - 34.0 {
            self.text(d, note, x, h - 30.0, 15.0, DIM);
        }
    }

    /// The NDT section: its state, score per point, ms per match; or why it isn't
    /// running and how to start it. Returns where the next section starts.
    fn ndt(
        &self,
        d: &mut RaylibDrawHandle,
        state: &DebugState,
        f: &Frame<'_>,
        x0: f32,
        y: f32,
        w: f32,
    ) -> f32 {
        let mut y = self.section(d, "NDT (A TRIAL: NEVER DRIVES)", x0, y, w);
        let (x, right) = (x0 + 18.0, w - 18.0);
        let fresh = state.health(Source::NdtStatus, f.now).is_ok();
        match &state.ndt {
            Some(s) => {
                let (word, colour) = if !fresh {
                    ("STALE", DIM)
                } else {
                    let colour = match (s.headline(), s.fit) {
                        ("LOST", _) => BAD,
                        ("TRACKING", Fit::Good) => GOOD,
                        _ => WARN,
                    };
                    (s.headline(), colour)
                };
                let ww = self.font.measure(word, 17.0);
                let pill = Rectangle::new(x, y, ww + 20.0, 26.0);
                d.draw_rectangle_rounded(pill, 0.4, 6, colour);
                self.text(d, word, x + 10.0, y + 4.0, 17.0, Color::BLACK);
                let nx = x + ww + 34.0;
                let numbers = fit(&self.font, &s.numbers(), right - nx, 19.0);
                self.text(d, &numbers, nx, y + 2.0, 19.0, TEXT);
                y += 32.0;
                let details = s.details();
                if !details.is_empty() {
                    let details = fit(&self.font, &details, right - x, 16.0);
                    self.text(d, &details, x, y, 16.0, CAPTION);
                    y += 22.0;
                }
                if s.fit != Fit::Good && !s.why.is_empty() {
                    let why = fit(&self.font, &s.why, right - x, 16.0);
                    self.text(d, &why, x, y, 16.0, WARN);
                    y += 22.0;
                }
                if s.state == Some(NdtState::Waiting) {
                    let hint = fit(&self.font, "no start pose yet", right - x, 16.0);
                    self.text(d, &hint, x, y, 16.0, DIM);
                    y += 22.0;
                }
            }
            None => {
                let s = match f.map {
                    MapFile::Loaded { .. } => "not running: start the sim with ndt:=true",
                    _ => "not running",
                };
                self.text(d, s, x, y, 19.0, DIM);
                y += 26.0;
            }
        }
        // The map: where it is, or how to make it (key M draws it).
        let colour = if f.map.cells().is_some() { DIM } else { WARN };
        let line = f.map.to_string();
        for l in wrap(&self.font, &line, right - x, 16.0).into_iter().take(3) {
            self.text(d, &l, x, y, 16.0, colour);
            y += 21.0;
        }
        y
    }

    fn vehicle(
        &self,
        d: &mut RaylibDrawHandle,
        v: &VehicleStatus,
        x: f32,
        right: f32,
        y: f32,
        fresh: bool,
    ) -> f32 {
        let mode = v.mode.to_string();
        self.text(d, &mode, x, y, 24.0, if fresh { TEXT } else { DIM });
        let mw = self.font.measure(&mode, 24.0);
        let (armed, armed_colour) = if v.armed {
            ("ARMED", GOOD)
        } else {
            ("DISARMED", DIM)
        };
        let mut cx = x + mw + 16.0;
        for (word, colour) in [
            (armed, armed_colour),
            ("E-STOP", if v.estop_active { BAD } else { SECTION }),
        ] {
            let ww = self.font.measure(word, 17.0);
            let r = Rectangle::new(cx, y + 1.0, ww + 18.0, 28.0);
            d.draw_rectangle_rounded(r, 0.4, 6, colour);
            let text_colour = if colour == SECTION { DIM } else { Color::BLACK };
            self.text(d, word, cx + 9.0, y + 5.0, 17.0, text_colour);
            cx += ww + 26.0;
        }
        let y = y + 36.0;
        let s = format!(
            "steer {:+.1} deg    {:.1} m/s    gear {}",
            v.steering_angle.to_degrees(),
            v.speed.0,
            v.gear().letter()
        );
        let s = fit(&self.font, &s, right - x, 19.0);
        self.text(d, &s, x, y, 19.0, if fresh { CAPTION } else { DIM });
        let y = y + 28.0;
        if !v.faults.is_empty() {
            let s = fit(
                &self.font,
                &format!("faults: {}", v.faults),
                right - x,
                18.0,
            );
            self.text(d, &s, x, y, 18.0, BAD);
            return y + 26.0;
        }
        y
    }

    /// A section's band and caption; returns where its rows start.
    fn section(&self, d: &mut RaylibDrawHandle, title: &str, x0: f32, y: f32, w: f32) -> f32 {
        d.draw_rectangle_rec(Rectangle::new(x0, y, w - x0, 30.0), SECTION);
        self.text(d, title, x0 + 18.0, y + 6.0, 17.0, CAPTION);
        y + 40.0
    }

    fn text(&self, d: &mut RaylibDrawHandle, s: &str, x: f32, y: f32, size: f32, colour: Color) {
        d.draw_text_ex(&self.font, s, Vector2::new(x, y), size, size / 24.0, colour);
    }
}

/// The 3D view's viewport and projection: the left `w` x `h` pixels, 50 degrees high.
/// raylib's own begin_mode3D sizes them for the whole window.
fn viewport(w: f32, h: f32, fovy_deg: f32) {
    let (near, far) = (0.1_f64, 2000.0_f64);
    let top = near * (f64::from(fovy_deg).to_radians() / 2.0).tan();
    let right = top * f64::from(w) / f64::from(h.max(1.0));
    // SAFETY: rlgl state calls on the drawing thread, inside begin_mode3D (which has
    // pushed the projection; end_mode3D pops it).
    unsafe {
        raylib::ffi::rlViewport(0, 0, w as i32, h as i32);
        raylib::ffi::rlMatrixMode(raylib::ffi::RL_PROJECTION as i32);
        raylib::ffi::rlLoadIdentity();
        raylib::ffi::rlFrustum(-right, right, -top, top, near, far);
        raylib::ffi::rlMatrixMode(raylib::ffi::RL_MODELVIEW as i32);
    }
}

/// How wide the 3D view is in a window `w` wide: beside the panel (key L), or beside
/// the seeing screen's two top-downs (about a third of the window).
pub fn view_width(w: f32, layout: Layout) -> f32 {
    let right = match layout {
        Layout::Panel => PANEL_W,
        Layout::Seeing => (w * 0.36).clamp(380.0, 720.0),
    };
    (w - right).max(200.0)
}

/// One car from above: its outline from vehicle.yaml and its arrow.
fn car_from_above(
    d: &mut RaylibDrawHandle,
    car: &CarShape,
    view: &View,
    pose: Pose,
    colour: Color,
) {
    let at = |p: Point| {
        let [x, y] = view.to_screen(p);
        Vector2::new(x, y)
    };
    let outline = car.outline(pose).map(at);
    for i in 0..4 {
        d.draw_line_ex(outline[i], outline[(i + 1) % 4], 2.0, colour);
    }
    let [a, b, c] = car.arrow(pose).map(at);
    // Seen from either side: the screen's y is upside down.
    d.draw_triangle(a, b, c, colour);
    d.draw_triangle(a, c, b, colour);
}

/// The points at least `step` metres apart, the last one kept.
fn thin(points: Vec<Point>, step: f64) -> Vec<Point> {
    let last = points.last().copied();
    let mut out: Vec<Point> = Vec::with_capacity(points.len());
    for p in points {
        if out.last().is_none_or(|q| q.distance(p).0 >= step) {
            out.push(p);
        }
    }
    if let (Some(l), Some(&end)) = (last, out.last()) {
        if end != l {
            out.push(l);
        }
    }
    out
}

fn v3(p: Point, z: f32) -> Vector3 {
    Vector3::new(p.x as f32, p.y as f32, z)
}

/// Lines every metre near the target, every 5 m brighter; reaching farther as the camera
/// backs off.
fn ground_grid(d: &mut impl RaylibDraw3D, target: [f32; 3], distance: f32) {
    let step = if distance > 80.0 { 5.0 } else { 1.0 };
    let reach = (distance * 2.5).clamp(30.0, 400.0);
    let n = (reach / step) as i32;
    let (cx, cy) = (
        (target[0] / step).round() * step,
        (target[1] / step).round() * step,
    );
    for i in -n..=n {
        let o = i as f32 * step;
        let major = ((cx + o) / 5.0).round() * 5.0 == (cx + o);
        let colour = if major { GRID_MAJOR } else { GRID };
        d.draw_line3D(
            Vector3::new(cx + o, cy - reach, 0.0),
            Vector3::new(cx + o, cy + reach, 0.0),
            colour,
        );
        let major = ((cy + o) / 5.0).round() * 5.0 == (cy + o);
        let colour = if major { GRID_MAJOR } else { GRID };
        d.draw_line3D(
            Vector3::new(cx - reach, cy + o, 0.0),
            Vector3::new(cx + reach, cy + o, 0.0),
            colour,
        );
    }
}

/// A filled triangle seen from both sides.
fn triangle(d: &mut impl RaylibDraw3D, a: Vector3, b: Vector3, c: Vector3, colour: Color) {
    d.draw_triangle3D(a, b, c, colour);
    d.draw_triangle3D(a, c, b, colour);
}

/// The local EKF's car: its outline from vehicle.yaml as a box (1.2 m high), filled
/// arrow on the roof.
fn car_body(d: &mut impl RaylibDraw3D, car: &CarShape, pose: Pose, colour: Color) {
    let outline = car.outline(pose);
    let (low, high) = (0.05, 1.2);
    let roof = Color::new(colour.r, colour.g, colour.b, 90);
    let r = outline.map(|p| v3(p, high));
    triangle(d, r[0], r[1], r[2], roof);
    triangle(d, r[0], r[2], r[3], roof);
    for i in 0..4 {
        let (a, b) = (outline[i], outline[(i + 1) % 4]);
        d.draw_line3D(v3(a, low), v3(b, low), colour);
        d.draw_line3D(v3(a, high), v3(b, high), colour);
        d.draw_line3D(v3(a, low), v3(a, high), colour);
    }
    let arrow = car.arrow(pose).map(|p| v3(p, high + 0.02));
    triangle(d, arrow[0], arrow[1], arrow[2], colour);
    pole(d, pose, colour);
}

/// Another localizer's guess: the car's outline on the ground, a filled arrow, a pole.
fn marker(d: &mut impl RaylibDraw3D, car: &CarShape, pose: Pose, colour: Color, z: f32) {
    let outline = car.outline(pose);
    for i in 0..4 {
        let (a, b) = (outline[i], outline[(i + 1) % 4]);
        d.draw_line3D(v3(a, z), v3(b, z), colour);
        d.draw_line3D(v3(a, z + 0.3), v3(b, z + 0.3), colour);
    }
    let arrow = car.arrow(pose).map(|p| v3(p, z));
    triangle(d, arrow[0], arrow[1], arrow[2], colour);
    pole(d, pose, colour);
}

/// NDT's map: each occupied cell as a faint wireframe cube, from the map frame into
/// odom through map -> odom (none without it); at most [`MAX_MAP_CUBES`], the nearest
/// the car.
fn ndt_map(d: &mut impl RaylibDraw3D, state: &DebugState, cells: &MapCells) {
    let Some(odom_in_map) = state.map_to_odom.odom_in_map else {
        return;
    };
    let car_in_map = state
        .car()
        .map_or(odom_in_map.position, |c| odom_in_map.transform(c.position));
    let near = if cells.len() > MAX_MAP_CUBES {
        cells.nearest(car_in_map, MAX_MAP_CUBES)
    } else {
        cells.centres.clone()
    };
    let edge = cells.size * 0.96;
    // SAFETY: rlgl's matrix stack, on the drawing thread inside begin_mode3D; pushed
    // here and popped below. A vertex v on the map goes to R(-yaw) (v - odom's
    // position): odom.
    unsafe {
        raylib::ffi::rlPushMatrix();
        raylib::ffi::rlRotatef((-odom_in_map.yaw).to_degrees() as f32, 0.0, 0.0, 1.0);
        raylib::ffi::rlTranslatef(
            -odom_in_map.position.x as f32,
            -odom_in_map.position.y as f32,
            0.0,
        );
    }
    for [x, y, z] in near {
        d.draw_cube_wires(Vector3::new(x, y, z), edge, edge, edge, NDT_MAP);
    }
    // SAFETY: pops the matrix pushed above, on the same thread.
    unsafe { raylib::ffi::rlPopMatrix() };
}

/// One of AMCL's particles: a short arrow on the ground, pointing its way.
fn particle(d: &mut impl RaylibDraw3D, p: Pose) {
    let (s, c) = (p.yaw.sin() as f32, p.yaw.cos() as f32);
    let (x, y, z) = (p.position.x as f32, p.position.y as f32, 0.08);
    let tip = Vector3::new(x + c * PARTICLE_LEN, y + s * PARTICLE_LEN, z);
    d.draw_line3D(Vector3::new(x, y, z), tip, PARTICLE);
    // The head: two short strokes back from the tip, 25 degrees each side.
    let back = PARTICLE_LEN * 0.35;
    for side in [0.44_f32, -0.44] {
        let (hs, hc) = side.sin_cos();
        let (dx, dy) = (c * hc - s * hs, s * hc + c * hs);
        d.draw_line3D(
            tip,
            Vector3::new(tip.x - dx * back, tip.y - dy * back, z),
            PARTICLE,
        );
    }
}

/// A thin pole with a ball on top over base_link: seen from far off.
fn pole(d: &mut impl RaylibDraw3D, pose: Pose, colour: Color) {
    let p = pose.position;
    d.draw_cylinder_ex(v3(p, 0.0), v3(p, 2.4), 0.04, 0.04, 6, colour);
    d.draw_sphere(v3(p, 2.5), 0.18, colour);
}

fn health_word(h: Health) -> (&'static str, Color) {
    match h {
        Health::Ok(_) => ("OK", GOOD),
        Health::Stale(_) => ("STALE", WARN),
        Health::Never => ("NEVER", DIM),
    }
}

/// `s` in lines that fit `width` at `size`, broken between words.
fn wrap(font: &UiFont, s: &str, width: f32, size: f32) -> Vec<String> {
    let mut lines: Vec<String> = Vec::new();
    let mut line = String::new();
    for word in s.split_whitespace() {
        let longer = if line.is_empty() {
            word.to_owned()
        } else {
            format!("{line} {word}")
        };
        if line.is_empty() || font.measure(&longer, size) <= width {
            line = longer;
        } else {
            lines.push(std::mem::replace(&mut line, word.to_owned()));
        }
    }
    if !line.is_empty() {
        lines.push(line);
    }
    lines
}

/// `s`, cut with "..." to fit `width` at `size`.
fn fit(font: &UiFont, s: &str, width: f32, size: f32) -> String {
    if font.measure(s, size) <= width {
        return s.to_owned();
    }
    let mut out: String = s.to_owned();
    while !out.is_empty() && font.measure(&format!("{out}..."), size) > width {
        out.pop();
    }
    format!("{out}...")
}

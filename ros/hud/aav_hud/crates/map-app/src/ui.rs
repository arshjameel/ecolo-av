//! The map window, drawn with raylib. The same look as the HUD (hud-app's ui.rs): its
//! theme (hud_core::theme, `theme:` in config/map.yaml) and its font.
//!
//! ```text
//! +--------------------------------------------------+-----------------+
//! | [ SIMULATED band, when /system/source says so ]  | TRIP            |
//! | FOLLOWING THE CAR  [ / Search for a place ]  N   |   DRIVING       |
//! |                    [ results, or the place:  ]   |                 |
//! |                    [ DRIVE HERE / CONFIRM    ]   |                 |
//! |                                                  | DISTANCE LEFT   |
//! |         the streets (OpenStreetMap), and over    |   42 m          |
//! |         them Nav2's map, north up                |                 |
//! |         the plan (blue), the goal (green)        | TIME LEFT 15 s  |
//! |         a street route (violet, aav_routes)      |                 |
//! |         the car (its outline and an arrow)       | SPEED   11 km/h |
//! | [ the sensor watchdog's verdict, unless OK ]     | TOPICS          |
//! | |----| 10 m          © OpenStreetMap contributors | keys            |
//! +--------------------------------------------------+-----------------+
//! ```
//!
//! The map part uses the window's pixels; the panel's sizes grow with the window, so a
//! 15" monitor and a laptop look alike.
//!
//! A tap (a click) on the search box also opens the on-screen keyboard along the bottom
//! (map-core's `keyboard`): the car's screen has no real keyboard.
//!
//! The boxes over the map are frosted glass: see-through, with the map blurred behind
//! them when `blur` is on (frost.rs). The panel on the right is glass over the window's
//! soft background; in the arcade style (`initial_d`) the HUD's faint car stands at the
//! foot of that background, behind the panel's cards (hud_app::paint::faint_car).

use std::{sync::Arc, time::Instant};

use hud_app::paint::{self, ToColor};
use hud_core::{
    format,
    health::Health,
    robot::{Form, Odometer, Xy, TILT},
    silhouette::Placement,
    theme::{GridColours, Rgba, Theme},
    watchdog::Alarm,
};
use map_core::{
    car::CarShape,
    config::Topic,
    destination::{Finder, Step},
    geom::{Metres, Point},
    grid::{Grid, Palette},
    keyboard::{KeyCap, Keyboard},
    places::{self, MIN_QUERY_CHARS},
    robot::{self as robot_marker, CarMarker},
    state::{MapState, Remaining},
    streets::TileShapes,
    trip::{time_left, Tone},
    view::{self, ScreenPoint, View, Viewport},
};
use raylib::prelude::*;

use crate::{
    frost::Frost,
    search::Search,
    streets::{self, Status as StreetStatus, StreetMap},
};

/// What the street map's data needs said wherever it's shown (ODbL), and where the tiles
/// were made.
const STREET_CREDIT: &str = "© OpenStreetMap contributors · Protomaps";

/// Fonts tried in order, as the HUD; raylib's own small pixel font if none is there.
const FONT_FILES: [&str; 4] = [
    "/System/Library/Fonts/Supplemental/Verdana Bold.ttf", // macOS
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",   // macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", // Ubuntu, Raspberry Pi OS
    "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf", // Fedora, AlmaLinux
];
const FONT_LOAD_SIZE: i32 = 128;
/// The letters loaded from the font: ASCII, the street map's credit (© ·), and the
/// accents of Ottawa's French street names (rue Élisabeth-Bruyère, chemin de la Côte...).
const FONT_GLYPHS: &str = concat!(
    " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`",
    "abcdefghijklmnopqrstuvwxyz{|}~",
    "©·–—’éèêëàâäçôöîïùûüÉÈÊÀÂÇÔÎœŒ°"
);

/// The panel's width at the reference size (1600 x 900); it grows with the window.
const PANEL_W: f32 = 400.0;
/// Below this many pixels long, the car is drawn as a plain arrow, so it never vanishes.
const CAR_MIN_PX: f32 = 22.0;
/// The little robot is never drawn shorter than this many pixels (at the reference size).
const ROBOT_MIN_PX: f32 = 40.0;

/// raylib's font handle: one loaded from a file, or its built-in one.
enum MapFont {
    File(Font),
    BuiltIn(WeakFont),
}

impl MapFont {
    fn measure(&self, text: &str, size: f32) -> f32 {
        let spacing = size / 20.0;
        match self {
            Self::File(f) => f.measure_text(text, size, spacing).x,
            Self::BuiltIn(f) => f.measure_text(text, size, spacing).x,
        }
    }
}

impl AsRef<raylib::ffi::Font> for MapFont {
    fn as_ref(&self) -> &raylib::ffi::Font {
        match self {
            Self::File(f) => f.as_ref(),
            Self::BuiltIn(f) => f.as_ref(),
        }
    }
}

/// The map's picture on the graphics card.
struct MapTexture {
    texture: Texture2D,
    width: u32,
    height: u32,
}

/// The window's font, its colours, the map's texture, and the frosted glass.
pub struct Ui {
    font: MapFont,
    theme: &'static Theme,
    map: Option<MapTexture>,
    /// The blur behind the glass: `None` when `blur` is off, or the card refused it.
    frost: Option<Frost>,
    /// How the car is drawn (`car_marker`).
    marker: CarMarker,
    /// Metres the car has rolled: the little robot's treads.
    odometer: Odometer,
}

/// A grid's colours as the picture's pixels.
fn palette(g: &GridColours) -> Palette {
    let px = |c: Rgba| [c.r, c.g, c.b, c.a];
    Palette {
        free: px(g.free),
        occupied: px(g.occupied),
        unknown: px(g.unknown),
    }
}

/// Where things are in the window this frame.
#[derive(Clone, Copy)]
pub struct Layout {
    /// The panel's scale: 1 at 1600 x 900.
    u: f32,
    pub map: Viewport,
    panel: Rectangle,
    /// The SIMULATED band above the map, when there is one.
    band: Option<Rectangle>,
}

impl Layout {
    /// `banner`: a SIMULATED / BENCH / REPLAY band goes above the map, not over it, so
    /// the whole map still fits below it.
    pub fn new(window_w: i32, window_h: i32, banner: bool) -> Self {
        let (w, h) = (window_w as f32, window_h as f32);
        let u = (w / 1600.0).min(h / 900.0).clamp(0.6, 3.0);
        let pw = PANEL_W * u;
        let band = banner.then(|| Rectangle::new(0.0, 0.0, (w - pw).max(1.0), 56.0 * u));
        let top = band.map_or(0.0, |b| b.height);
        Self {
            u,
            map: Viewport {
                x: 0.0,
                y: f64::from(top),
                width: f64::from((w - pw).max(1.0)),
                height: f64::from((h - top).max(1.0)),
            },
            panel: Rectangle::new(w - pw, 0.0, pw, h),
            band,
        }
    }

    /// The on-screen keyboard, at the bottom of the window, centred under the map.
    pub fn keyboard(&self) -> Keyboard {
        let window = Viewport {
            x: 0.0,
            y: 0.0,
            width: f64::from(self.panel.x + self.panel.width),
            height: f64::from(self.panel.height),
        };
        Keyboard::new(window, self.map.centre().x, f64::from(self.u))
    }
}

impl Ui {
    /// Loads the font, and the blur's shaders when `blur`. Needs the window open.
    pub fn new(
        rl: &mut RaylibHandle,
        thread: &RaylibThread,
        theme: &'static Theme,
        blur: bool,
        marker: CarMarker,
    ) -> Self {
        let loaded = FONT_FILES
            .iter()
            .filter(|path| std::path::Path::new(path).exists())
            .find_map(|path| {
                rl.load_font_ex(thread, path, FONT_LOAD_SIZE, Some(FONT_GLYPHS))
                    .ok()
            });
        let font = match loaded {
            Some(f) => {
                f.texture()
                    .set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_BILINEAR);
                MapFont::File(f)
            }
            None => MapFont::BuiltIn(rl.get_font_default()),
        };
        let frost = blur
            .then(|| Frost::new(rl, thread))
            .and_then(|f| f.inspect_err(|e| eprintln!("map: no blur: {e}")).ok());
        Self {
            font,
            theme,
            map: None,
            frost,
            marker,
            odometer: Odometer::default(),
        }
    }

    /// The car is at `pose` this frame: turns the little robot's treads.
    pub fn roll(&mut self, pose: map_core::geom::Pose) {
        let at = Xy::new(pose.position.x, pose.position.y);
        self.odometer.roll(at, pose.yaw);
    }

    /// Whether the glass blurs the map (`blur`, and the card took the shaders).
    pub fn blurs(&self) -> bool {
        self.frost.is_some()
    }

    /// Before each frame: the blur's textures, the window's size.
    pub fn prepare(&mut self, rl: &mut RaylibHandle, thread: &RaylibThread) {
        if let Some(f) = self.frost.as_mut() {
            if let Err(e) = f.fit(rl, thread) {
                eprintln!("map: no blur: {e}");
                self.frost = None;
            }
        }
    }

    /// Puts a new map on the graphics card.
    pub fn upload(
        &mut self,
        rl: &mut RaylibHandle,
        thread: &RaylibThread,
        grid: &Grid,
        over_streets: bool,
    ) -> anyhow::Result<()> {
        let (gw, gh) = (grid.info.width, grid.info.height);
        let same_size = self
            .map
            .as_ref()
            .is_some_and(|m| m.width == gw && m.height == gh);
        if !same_size {
            let (w, h) = (i32::try_from(gw)?, i32::try_from(gh)?);
            // gen_image_color makes RGBA8 pixels, the layout to_rgba gives.
            let image = Image::gen_image_color(w, h, Color::BLACK);
            let texture = rl.load_texture_from_image(thread, &image)?;
            // Sharp cells when zoomed in: each is a square, not a blur.
            texture.set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_POINT);
            self.map = Some(MapTexture {
                texture,
                width: gw,
                height: gh,
            });
        }
        if let Some(m) = self.map.as_mut() {
            let colours = if over_streets {
                &self.theme.grid_over_streets
            } else {
                &self.theme.grid
            };
            m.texture.update_texture(&grid.to_rgba(&palette(colours)))?;
        }
        Ok(())
    }

    /// Draws one frame. `tiles`: the street map's, from [`StreetMap::frame`].
    /// `keyboard`: the on-screen keyboard when it's open, and the key held down.
    #[allow(clippy::too_many_arguments)]
    pub fn draw(
        &mut self,
        d: &mut RaylibDrawHandle,
        thread: &RaylibThread,
        state: &MapState,
        view: &View,
        car: &CarShape,
        now: Instant,
        source: &str,
        streets: &StreetMap,
        tiles: &[Arc<TileShapes>],
        search: &Search,
        keyboard: Option<(&Keyboard, Option<KeyCap>)>,
    ) {
        let banner = state.origins.banner();
        let layout = Layout::new(
            d.get_screen_width(),
            d.get_screen_height(),
            banner.is_some(),
        );
        let t = self.theme;
        let vp = layout.map;
        let (w, h) = (d.get_screen_width(), d.get_screen_height());
        paint::backdrop(d, w, h, t);
        faint_car(d, &layout, t);
        // Everything on the map stays on the map.
        let (sx, sy, sw, sh) = (vp.x as i32, vp.y as i32, vp.width as i32, vp.height as i32);
        {
            let mut s = d.begin_scissor_mode(sx, sy, sw, sh);
            let mut p = Painter {
                d: &mut s,
                font: &self.font,
                u: layout.u,
                t,
                frost: None,
            };
            self.map_layers(&mut p, state, view, &vp, car, now, streets, tiles, search);
        }
        // The frosted glass's map: what's on the screen now, copied and blurred.
        let mut frost = self.frost.take();
        if let Some(f) = frost.as_mut() {
            f.capture(d);
            f.blur(d, thread);
        }
        {
            let mut s = d.begin_scissor_mode(sx, sy, sw, sh);
            let mut p = Painter {
                d: &mut s,
                font: &self.font,
                u: layout.u,
                t,
                frost: frost.as_mut(),
            };
            map_overlays(&mut p, state, view, &vp);
            street_notes(&mut p, &vp, streets);
            watchdog_banner(&mut p, &vp, state, now);
        }
        let mut p = Painter {
            d,
            font: &self.font,
            u: layout.u,
            t,
            frost: frost.as_mut(),
        };
        if let (Some(band), Some(words)) = (layout.band, &banner) {
            p.d.draw_rectangle_rec(band, t.not_real.rl());
            let cx = band.x + band.width / 2.0;
            p.centred(cx, band.y + 8.0 * p.u, 40.0, words, t.not_real_ink);
        }
        panel(&mut p, &layout, state, now, source);
        let boxes = SearchLayout::new(&layout, &search.finder, keyboard.map(|(kb, _)| kb));
        draw_search(&mut p, &boxes, search, state, now);
        if let Some((kb, held)) = keyboard {
            draw_keyboard(&mut p, kb, held);
        }
        self.frost = frost;
    }

    /// The map itself: the streets, Nav2's map, the routes, the goal, the car, the
    /// picked place. What the glass over it blurs.
    #[allow(clippy::too_many_arguments)]
    fn map_layers(
        &self,
        p: &mut Painter<impl RaylibDraw>,
        state: &MapState,
        view: &View,
        vp: &Viewport,
        car: &CarShape,
        now: Instant,
        streets: &StreetMap,
        tiles: &[Arc<TileShapes>],
        search: &Search,
    ) {
        let on_streets = streets.visible();
        if on_streets {
            let font = &self.font;
            streets::draw(
                p.d,
                &p.t.streets,
                font,
                |text, size| font.measure(text, size),
                tiles,
                view,
                vp,
                p.u,
            );
        }
        self.draw_map(p, state, view, vp, car, now, on_streets);
        if let Some(place) = search.finder.picked() {
            picked_marker(p, screen(view, search.on_map(place), vp));
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_map(
        &self,
        p: &mut Painter<impl RaylibDraw>,
        state: &MapState,
        view: &View,
        vp: &Viewport,
        car: &CarShape,
        now: Instant,
        on_streets: bool,
    ) {
        let t = p.t;
        let at = |pt: Point| screen(view, pt, vp);
        // The map's picture, turned as the grid is.
        if let (Some(grid), Some(tex)) = (&state.grid, &self.map) {
            let info = grid.info;
            let (w_m, h_m) = info.size();
            let top_left = at(info.origin.transform(Point::new(0.0, h_m)));
            let k = view.scale.0 as f32;
            let dest = Rectangle::new(top_left.x, top_left.y, w_m as f32 * k, h_m as f32 * k);
            let src = Rectangle::new(0.0, 0.0, tex.width as f32, tex.height as f32);
            let turn = -(info.origin.yaw.to_degrees() as f32);
            p.d.draw_texture_pro(&tex.texture, src, dest, Vector2::zero(), turn, Color::WHITE);
            let line = if on_streets {
                t.grid_line_over_streets
            } else {
                t.grid_line
            };
            grid_lines(p, view, vp, grid, line);
        }

        // A street route (aav_routes), under the plan: what Nav2 may be asked to follow.
        let street: Vec<Vector2> = state.street_route.points().iter().map(|&q| at(q)).collect();
        if street.len() >= 2 {
            p.d.draw_spline_linear(&street, 7.0 * p.u, t.plan_casing.rl());
            p.d.draw_spline_linear(&street, 4.0 * p.u, t.street_route.rl());
        }

        // The plan: a casing under the accent's line, so it shows on any colour.
        let route: Vec<Vector2> = state.route.points().iter().map(|&q| at(q)).collect();
        if route.len() >= 2 {
            p.d.draw_spline_linear(&route, 9.0 * p.u, t.plan_casing.rl());
            p.d.draw_spline_linear(&route, 5.0 * p.u, t.accent.rl());
        }
        if let Some(goal) = state.goal() {
            goal_marker(p, at(goal));
        }
        if let Some(c) = &state.car {
            let fresh = state.health(Topic::Odom, now).is_ok();
            match self.marker {
                CarMarker::Outline => car_marker(p, view, vp, car, c.pose, fresh),
                CarMarker::Robot => robot(p, view, vp, car, c.pose, fresh, self.odometer.travelled),
            }
        }
    }
}

/// Over the map: the view's mode, north, the scale bar, and any problem. Glass.
fn map_overlays(p: &mut Painter<impl RaylibDraw>, state: &MapState, view: &View, vp: &Viewport) {
    let t = p.t;
    {
        let top = vp.y as f32 + 12.0 * p.u;
        p.chip(vp.x as f32 + 14.0 * p.u, top, view.mode.label(), t.text);
        north(
            p,
            vp.x as f32 + vp.width as f32 - 34.0 * p.u,
            top + 4.0 * p.u,
        );
        scale_bar(p, view, vp);
        let mut problems = state.frame_problems();
        problems.extend(state.map_problem.clone());
        if state.grid.is_none() {
            let topic = state.config.topic(Topic::Map);
            let c = vp.centre();
            let msg = format!("NO MAP YET: waiting for {topic}");
            p.chip_centred(c.x as f32, c.y as f32 - 80.0 * p.u, &msg, t.warn);
        }
        for (i, why) in problems.iter().enumerate() {
            let y = vp.y as f32 + vp.height as f32 - (96.0 + 36.0 * i as f32) * p.u;
            p.chip(vp.x as f32 + 14.0 * p.u, y, why, t.warn);
        }
    }
}

fn screen(view: &View, p: Point, vp: &Viewport) -> Vector2 {
    let s = view.to_screen(p, vp);
    Vector2::new(s.x as f32, s.y as f32)
}

/// Thin lines every round number of metres, over the map only.
fn grid_lines(
    p: &mut Painter<impl RaylibDraw>,
    view: &View,
    vp: &Viewport,
    grid: &Grid,
    colour: Rgba,
) {
    let colour = colour.rl();
    let step = view::grid_step(view.scale, 70.0 * f64::from(p.u));
    let seen = view.visible(vp);
    let map = grid.info.bounds();
    let (x0, x1) = (seen.min.x.max(map.min.x), seen.max.x.min(map.max.x));
    let (y0, y1) = (seen.min.y.max(map.min.y), seen.max.y.min(map.max.y));
    if x0 >= x1 || y0 >= y1 {
        return;
    }
    for x in view::grid_lines(x0, x1, step) {
        let (a, b) = (
            screen(view, Point::new(x, y0), vp),
            screen(view, Point::new(x, y1), vp),
        );
        p.d.draw_line_ex(a, b, 1.0, colour);
    }
    for y in view::grid_lines(y0, y1, step) {
        let (a, b) = (
            screen(view, Point::new(x0, y), vp),
            screen(view, Point::new(x1, y), vp),
        );
        p.d.draw_line_ex(a, b, 1.0, colour);
    }
}

/// Bottom right of the map: the street map's credit when it's shown, or why it isn't.
/// The sensor watchdog's verdict, as on the HUD: red while it holds Nav2 (STOP, or the
/// watchdog silent), amber for a warning. Across the bottom of the map, over the scale.
fn watchdog_banner(
    p: &mut Painter<impl RaylibDraw>,
    vp: &Viewport,
    state: &MapState,
    now: Instant,
) {
    let Some(alarm) = state.sensor_alarm(now) else {
        return;
    };
    let t = p.t;
    let (text, colour, ink) = match &alarm {
        Alarm::Stop(why) => (why, t.stop_fill, t.stop_ink),
        Alarm::Warn(why) => (why, t.wait_fill, t.wait_ink),
    };
    let margin = 14.0 * p.u;
    let h = 44.0 * p.u;
    let r = Rectangle::new(
        vp.x as f32 + margin,
        (vp.y + vp.height) as f32 - 96.0 * p.u - h,
        (vp.width as f32 - 2.0 * margin).max(1.0),
        h,
    );
    paint::rounded(p.d, r, p.t.style.corner(12.0 * p.u), colour);
    p.fitted(
        r.x + 12.0 * p.u,
        r.y + 9.0 * p.u,
        24.0,
        r.width - 24.0 * p.u,
        text,
        ink,
    );
}

fn street_notes(p: &mut Painter<impl RaylibDraw>, vp: &Viewport, streets: &StreetMap) {
    let right = (vp.x + vp.width) as f32 - 14.0 * p.u;
    let bottom = (vp.y + vp.height) as f32 - 12.0 * p.u;
    if streets.visible() {
        let size = 15.0;
        let w = p.width(STREET_CREDIT, size);
        let r = Rectangle::new(
            right - w - 16.0 * p.u,
            bottom - 26.0 * p.u,
            w + 16.0 * p.u,
            26.0 * p.u,
        );
        p.frosted(r, r.height / 2.0, p.t.overlay);
        p.text(
            r.x + 8.0 * p.u,
            r.y + 5.0 * p.u,
            size,
            STREET_CREDIT,
            p.t.caption,
        );
        return;
    }
    let note = match &streets.status {
        _ if !streets.shown => "STREETS OFF (S)".to_owned(),
        StreetStatus::Missing(_) => "No street map: pixi run get street-map".to_owned(),
        StreetStatus::Failed(why) => format!("No street map: {why}"),
        StreetStatus::Loading | StreetStatus::Ready => return,
    };
    let w = p.width(&note, 17.0) + 24.0 * p.u;
    p.chip(right - w, bottom - 32.0 * p.u, &note, p.t.caption);
}

/// The picked place: a pin (the theme's pink), its point at the place.
fn picked_marker(p: &mut Painter<impl RaylibDraw>, at: Vector2) {
    let (edge, pin) = (p.t.marker_edge.rl(), p.t.picked.rl());
    let r = 12.0 * p.u;
    let head = at - Vector2::new(0.0, 30.0 * p.u);
    p.d.draw_line_ex(at, head, 4.0 * p.u, edge);
    p.d.draw_circle_v(at, 4.0 * p.u, edge);
    p.d.draw_circle_v(head, r + 3.0 * p.u, edge);
    p.d.draw_circle_v(head, r, pin);
}

/// What a click on the search box hit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Click {
    /// The box itself: open it.
    Field,
    /// One of the results.
    Result(usize),
    /// DRIVE HERE, or CONFIRM.
    Button,
    /// Cancel, next to CONFIRM.
    Cancel,
}

/// Where the search box's parts are this frame: for drawing, and for clicks.
pub struct SearchLayout {
    u: f32,
    field: Rectangle,
    /// Under the box: the results (typing) or the picked place's card.
    under: Option<Rectangle>,
    rows: Vec<Rectangle>,
    button: Option<Rectangle>,
    cancel: Option<Rectangle>,
    /// The on-screen keyboard is open: the hints say tap.
    keyboard: bool,
}

const ROW_H: f32 = 62.0;
const FOOTER_H: f32 = 34.0;
const CARD_H: f32 = 180.0;

impl SearchLayout {
    /// `keyboard`: the on-screen keyboard, when it's open; the results stop above it.
    pub fn new(layout: &Layout, finder: &Finder, keyboard: Option<&Keyboard>) -> Self {
        let u = layout.u;
        let vp = layout.map;
        let w = (620.0 * u).min(vp.width as f32 - 40.0 * u).max(100.0);
        let x = vp.x as f32 + (vp.width as f32 - w) / 2.0;
        let field = Rectangle::new(x, vp.y as f32 + 12.0 * u, w, 46.0 * u);
        let top = field.y + field.height + 6.0 * u;
        let mut boxes = Self {
            u,
            field,
            under: None,
            rows: Vec::new(),
            button: None,
            cancel: None,
            keyboard: keyboard.is_some(),
        };
        match finder.step() {
            Step::Closed => {}
            Step::Typing => {
                // Only the rows that fit above the keyboard (at least one).
                let room = keyboard.map_or(f32::INFINITY, |kb| {
                    kb.area().y as f32 - 8.0 * u - top - FOOTER_H * u
                });
                let fit = (room / (ROW_H * u)).floor().max(1.0) as usize;
                let n = finder.results().len().min(fit);
                boxes.rows = (0..n)
                    .map(|i| Rectangle::new(x, top + i as f32 * ROW_H * u, w, ROW_H * u))
                    .collect();
                let body = if n == 0 {
                    46.0 * u
                } else {
                    n as f32 * ROW_H * u
                };
                boxes.under = Some(Rectangle::new(x, top, w, body + FOOTER_H * u));
            }
            Step::Picked | Step::Confirming => {
                let card = Rectangle::new(x, top, w, CARD_H * u);
                let by = card.y + 112.0 * u;
                let bh = 52.0 * u;
                if finder.step() == Step::Confirming {
                    let cw = 150.0 * u;
                    boxes.button = Some(Rectangle::new(x + 16.0 * u, by, w - cw - 44.0 * u, bh));
                    boxes.cancel = Some(Rectangle::new(x + w - cw - 16.0 * u, by, cw, bh));
                } else {
                    boxes.button = Some(Rectangle::new(x + 16.0 * u, by, w - 32.0 * u, bh));
                }
                boxes.under = Some(card);
            }
        }
        boxes
    }

    /// Whether the mouse is over the box or what's under it (not the map).
    pub fn contains(&self, at: ScreenPoint) -> bool {
        let v = Vector2::new(at.x as f32, at.y as f32);
        std::iter::once(self.field)
            .chain(self.under)
            .any(|r| r.check_collision_point_rec(v))
    }

    /// What a click at `at` hits.
    pub fn click(&self, at: ScreenPoint) -> Option<Click> {
        let v = Vector2::new(at.x as f32, at.y as f32);
        let hit = |r: &Rectangle| r.check_collision_point_rec(v);
        if hit(&self.field) {
            return Some(Click::Field);
        }
        if let Some(i) = self.rows.iter().position(hit) {
            return Some(Click::Result(i));
        }
        if self.cancel.as_ref().is_some_and(hit) {
            return Some(Click::Cancel);
        }
        self.button
            .as_ref()
            .filter(|r| hit(r))
            .map(|_| Click::Button)
    }
}

/// The search box, and under it the results or the picked place.
fn draw_search(
    p: &mut Painter<impl RaylibDraw>,
    boxes: &SearchLayout,
    search: &Search,
    state: &MapState,
    now: Instant,
) {
    let u = boxes.u;
    let finder = &search.finder;
    let f = boxes.field;
    let typing = finder.is_typing();
    let t = p.t;
    p.frosted(f, 14.0 * u, t.overlay_strong);
    if typing {
        paint::rounded_edge(p.d, f, p.t.style.corner(14.0 * u), 3.0 * u, t.accent);
    }
    let tx = f.x + 18.0 * u;
    let ty = f.y + 11.0 * u;
    let tw = f.width - 36.0 * u;
    if finder.query().is_empty() {
        let hint = if typing {
            "Type a place or an address"
        } else {
            "Search for a place: tap here, or /"
        };
        p.fitted(tx, ty, 21.0, tw, hint, t.caption);
    } else {
        let shown = if typing {
            format!("{}|", finder.query())
        } else {
            finder.query().to_owned()
        };
        p.fitted(tx, ty, 21.0, tw, &shown, t.text);
    }

    let Some(under) = boxes.under else {
        if let Some(n) = search.notice(now) {
            let colour = if n.good { t.good } else { t.warn };
            p.chip(f.x, f.y + f.height + 8.0 * u, &n.text, colour);
        }
        return;
    };
    p.frosted(under, 14.0 * u, t.overlay_strong);
    match finder.step() {
        Step::Typing => draw_results(p, boxes, search, state, under),
        Step::Picked | Step::Confirming => draw_picked(p, boxes, search, state, under),
        Step::Closed => {}
    }
}

fn draw_results(
    p: &mut Painter<impl RaylibDraw>,
    boxes: &SearchLayout,
    search: &Search,
    state: &MapState,
    under: Rectangle,
) {
    let u = boxes.u;
    let t = p.t;
    let finder = &search.finder;
    let inner = under.width - 36.0 * u;
    for (i, (place, row)) in finder.results().iter().zip(&boxes.rows).enumerate() {
        if i == finder.selected() {
            paint::rounded(p.d, *row, p.t.style.corner(10.0 * u), p.t.selected_row);
        }
        let x = row.x + 18.0 * u;
        let far = search
            .distance_from_car(place, state)
            .map(distance)
            .unwrap_or_default();
        let far_w = p.width(&far, 18.0);
        p.right(
            row.x + row.width - 18.0 * u,
            row.y + 10.0 * u,
            18.0,
            &far,
            t.caption,
        );
        let title_w = inner - far_w - 16.0 * u;
        p.fitted(x, row.y + 8.0 * u, 21.0, title_w, &place.title, t.text);
        p.fitted(x, row.y + 36.0 * u, 15.0, inner, &place.detail(), t.caption);
    }
    let x = under.x + 18.0 * u;
    if finder.results().is_empty() {
        let (msg, colour) = match finder.problem() {
            Some(why) => (why.to_owned(), t.warn),
            None if finder.query().chars().count() < MIN_QUERY_CHARS => {
                (format!("Type {MIN_QUERY_CHARS} letters or more"), t.caption)
            }
            None => ("No places found".to_owned(), t.caption),
        };
        p.fitted(x, under.y + 12.0 * u, 18.0, inner, &msg, colour);
    }
    let fy = under.y + under.height - FOOTER_H * u + 8.0 * u;
    p.text(x, fy, 14.0, places::CREDIT, t.dim);
    let hint = if boxes.keyboard {
        "Tap a place, or Enter for the top one"
    } else {
        "Up Down  Enter pick  Esc close"
    };
    p.right(under.x + under.width - 18.0 * u, fy, 14.0, hint, t.dim);
}

fn rectangle(r: Viewport) -> Rectangle {
    Rectangle::new(r.x as f32, r.y as f32, r.width as f32, r.height as f32)
}

/// The on-screen keyboard: big keys, the letters light, the editing keys darker, Enter
/// in the accent; the key under a finger lights up.
fn draw_keyboard(p: &mut Painter<impl RaylibDraw>, kb: &Keyboard, held: Option<KeyCap>) {
    let u = p.u;
    let t = p.t;
    let area = rectangle(kb.area());
    p.frosted(area, 18.0 * u, t.overlay_strong);
    // Letters about 45% of a key's height; the words on the wide keys a little smaller.
    let letter = kb.key_height() as f32 * 0.45 / u;
    for (cap, r) in kb.keys() {
        let r = rectangle(r);
        let (back, ink) = match cap {
            _ if held == Some(cap) => (t.key_down, t.text),
            KeyCap::Enter => (t.accent, t.accent_ink),
            c if c.is_command() => (t.key_command, t.text),
            _ => (t.key, t.text),
        };
        paint::rounded(p.d, r, p.t.style.corner(10.0 * u), back);
        let label = cap.label();
        let size = if label.chars().count() > 1 {
            letter * 0.7
        } else {
            letter
        };
        let y = r.y + (r.height - size * u) / 2.0;
        p.centred(r.x + r.width / 2.0, y, size, &label, ink);
    }
}

fn draw_picked(
    p: &mut Painter<impl RaylibDraw>,
    boxes: &SearchLayout,
    search: &Search,
    state: &MapState,
    card: Rectangle,
) {
    let u = boxes.u;
    let t = p.t;
    let Some(place) = search.finder.picked() else {
        return;
    };
    let x = card.x + 18.0 * u;
    let inner = card.width - 36.0 * u;
    p.fitted(x, card.y + 14.0 * u, 26.0, inner, &place.title, t.text);
    let far = search
        .distance_from_car(place, state)
        .map(|d| format!("  ·  {} from the car", distance(d)))
        .unwrap_or_default();
    p.fitted(
        x,
        card.y + 52.0 * u,
        17.0,
        inner,
        &format!("{}{far}", place.detail()),
        t.caption,
    );
    p.text(x, card.y + 80.0 * u, 14.0, places::CREDIT, t.dim);
    let confirming = search.finder.step() == Step::Confirming;
    let goal = search.picked_goal(state);
    match (goal, boxes.button) {
        (Some(Ok(_)), Some(b)) => {
            let (colour, ink, words) = if confirming {
                (t.confirm, t.confirm_ink, "CONFIRM: send goal  (Enter)")
            } else {
                (t.go_fill, t.go_ink, "DRIVE HERE  (Enter)")
            };
            paint::rounded(p.d, b, p.t.style.corner(12.0 * u), colour);
            p.centred(b.x + b.width / 2.0, b.y + 13.0 * u, 23.0, words, ink);
        }
        (Some(Err(why)), Some(b)) => {
            paint::rounded_edge(p.d, b, p.t.style.corner(12.0 * u), 2.0 * u, t.warn);
            p.fitted(
                b.x + 16.0 * u,
                b.y + 14.0 * u,
                21.0,
                b.width - 32.0 * u,
                &why.to_string(),
                t.warn,
            );
        }
        _ => {}
    }
    if let Some(c) = boxes.cancel {
        paint::rounded_edge(p.d, c, p.t.style.corner(12.0 * u), 2.0 * u, t.caption);
        p.centred(
            c.x + c.width / 2.0,
            c.y + 14.0 * u,
            20.0,
            "Esc cancel",
            t.text,
        );
    }
    let hint = if confirming {
        "Enter sends Nav2 this goal"
    } else {
        "Esc back to the list"
    };
    p.right(
        card.x + card.width - 18.0 * u,
        card.y + 80.0 * u,
        14.0,
        hint,
        t.dim,
    );
}

/// The goal: a green ring with a light edge and a light middle.
fn goal_marker(p: &mut Painter<impl RaylibDraw>, at: Vector2) {
    let (edge, goal) = (p.t.marker_edge.rl(), p.t.goal.rl());
    let r = 13.0 * p.u;
    p.d.draw_circle_v(at, r + 3.0 * p.u, edge);
    p.d.draw_circle_v(at, r, goal);
    p.d.draw_circle_v(at, r * 0.38, edge);
}

/// The car: its outline to scale, with an arrow; or, when that would be tiny, an arrow
/// of a fixed size. Grey when its position is old.
fn car_marker(
    p: &mut Painter<impl RaylibDraw>,
    view: &View,
    vp: &Viewport,
    shape: &CarShape,
    pose: map_core::geom::Pose,
    fresh: bool,
) {
    let edge = p.t.marker_edge;
    let body = if fresh { p.t.car_body } else { p.t.dim };
    let at = |q: Point| screen(view, q, vp);
    let length_px = (shape.length * view.scale.0) as f32;
    if length_px >= CAR_MIN_PX {
        let o = shape.outline(pose).map(at);
        fill_quad(p, o, body);
        for i in 0..4 {
            p.d.draw_line_ex(o[i], o[(i + 1) % 4], 2.0 * p.u, edge.rl());
        }
        let a = shape.arrow(pose).map(at);
        fill_triangle(p, a, edge);
    } else {
        // A fixed arrow, CAR_MIN_PX long, at the car's middle.
        let centre = at(shape.centre(pose));
        let (s, c) = (pose.yaw as f32).sin_cos();
        let (fwd, left) = (Vector2::new(c, -s), Vector2::new(-s, -c));
        let len = CAR_MIN_PX * p.u;
        let tip = centre + fwd * (len * 0.6);
        let back_l = centre - fwd * (len * 0.4) + left * (len * 0.4);
        let back_r = centre - fwd * (len * 0.4) - left * (len * 0.4);
        let edge_px = 3.0 * p.u;
        fill_triangle(
            p,
            [
                tip + fwd * edge_px,
                back_l + (left - fwd) * edge_px,
                back_r - (left + fwd) * edge_px,
            ],
            edge,
        );
        fill_triangle(p, [tip, back_l, back_r], body);
    }
}

/// The car as the little robot (`car_marker: robot`): on the car's true outline, facing
/// its heading, a little tilted (heights drawn up the screen). Faded when its position
/// is old.
fn robot(
    p: &mut Painter<impl RaylibDraw>,
    view: &View,
    vp: &Viewport,
    shape: &CarShape,
    pose: map_core::geom::Pose,
    fresh: bool,
    travelled: f64,
) {
    let colours = p.t.robot();
    let k = view.scale.0;
    let min_length = f64::from(ROBOT_MIN_PX * p.u) / k;
    let lift = (k * TILT) as f32;
    let at = |q: hud_core::robot::Xyz| {
        screen(view, Point::new(q.x, q.y), vp) - Vector2::new(0.0, q.z as f32 * lift)
    };
    let edge = colours.edge;
    for piece in robot_marker::pieces(shape, pose, min_length, travelled) {
        let colour = colours.of(piece.part);
        let colour = if fresh {
            colour
        } else {
            colour.mix(p.t.dim, 65)
        };
        match &piece.form {
            Form::Polygon(points) => {
                let pts: Vec<Vector2> = points.iter().map(|&q| at(q)).collect();
                for i in 1..pts.len().saturating_sub(1) {
                    fill_triangle(p, [pts[0], pts[i], pts[i + 1]], colour);
                }
                for (i, &a) in pts.iter().enumerate() {
                    let b = pts[(i + 1) % pts.len()];
                    p.d.draw_line_ex(a, b, 1.0 * p.u, edge.rl());
                }
            }
            Form::Circle { centre, radius } => {
                let r = (*radius * k) as f32;
                p.d.draw_circle_v(at(*centre), r + 1.0 * p.u, edge.rl());
                p.d.draw_circle_v(at(*centre), r, colour.rl());
            }
        }
    }
}

/// raylib fills a triangle only when its corners go one way round; draw both ways.
fn fill_triangle(p: &mut Painter<impl RaylibDraw>, [a, b, c]: [Vector2; 3], colour: Rgba) {
    let colour = colour.rl();
    p.d.draw_triangle(a, b, c, colour);
    p.d.draw_triangle(a, c, b, colour);
}

fn fill_quad(p: &mut Painter<impl RaylibDraw>, [a, b, c, d]: [Vector2; 4], colour: Rgba) {
    fill_triangle(p, [a, b, c], colour);
    fill_triangle(p, [a, c, d], colour);
}

/// North is always up: a small arrow and an N, top right.
fn north(p: &mut Painter<impl RaylibDraw>, x: f32, y: f32) {
    let r = 20.0 * p.u;
    let t = p.t;
    p.frosted(Rectangle::new(x - r, y, 2.0 * r, 2.0 * r), r, t.overlay);
    let tip = Vector2::new(x, y + 5.0 * p.u);
    fill_triangle(
        p,
        [
            tip,
            Vector2::new(x - 6.0 * p.u, y + 16.0 * p.u),
            Vector2::new(x + 6.0 * p.u, y + 16.0 * p.u),
        ],
        t.text,
    );
    p.centred(x, y + 17.0 * p.u, 18.0, "N", t.text);
}

/// A bar of a round number of metres, bottom left.
fn scale_bar(p: &mut Painter<impl RaylibDraw>, view: &View, vp: &Viewport) {
    let Some((metres, px)) = view::scale_bar(view.scale, f64::from(180.0 * p.u)) else {
        return;
    };
    let px = px as f32;
    let label = if metres >= 1.0 {
        format!("{metres:.0} m")
    } else {
        format::metres(metres)
    };
    let x = vp.x as f32 + 18.0 * p.u;
    let y = vp.y as f32 + vp.height as f32 - 46.0 * p.u;
    let text_w = p.width(&label, 20.0);
    let back = Rectangle::new(
        x - 10.0 * p.u,
        y - 10.0 * p.u,
        px + text_w + 34.0 * p.u,
        38.0 * p.u,
    );
    p.frosted(back, back.height / 2.0, p.t.overlay);
    let bar_y = y + 9.0 * p.u;
    let thick = 3.0 * p.u;
    let ink = p.t.text;
    p.d.draw_line_ex(
        Vector2::new(x, bar_y),
        Vector2::new(x + px, bar_y),
        thick,
        ink.rl(),
    );
    for end in [x, x + px] {
        p.d.draw_line_ex(
            Vector2::new(end, bar_y - 7.0 * p.u),
            Vector2::new(end, bar_y + 7.0 * p.u),
            thick,
            ink.rl(),
        );
    }
    p.text(x + px + 12.0 * p.u, y - 2.0 * p.u, 20.0, &label, ink);
}

/// The panel: the trip, the distance and time left, the speed, the topics, the keys.
/// The arcade style's faint car, across the foot of the panel's column, behind its
/// cards: TOPICS and the keys, where the words are smallest and the cards hide it most.
fn faint_car(d: &mut impl RaylibDraw, layout: &Layout, t: &Theme) {
    let r = layout.panel;
    let m = 12.0 * layout.u;
    let place = Placement::bottom_right(r.x + m, r.y, r.width - 2.0 * m, r.height - m);
    paint::faint_car(d, t, place, 2.0 * layout.u, paint::Under::Panels);
}

fn panel(
    p: &mut Painter<impl RaylibDraw>,
    layout: &Layout,
    state: &MapState,
    now: Instant,
    source: &str,
) {
    let u = p.u;
    let t = p.t;
    let r = layout.panel;
    let m = 12.0 * u;
    let (x, w) = (r.x + m, r.width - 2.0 * m);
    let inner = x + 16.0 * u;
    let inner_w = w - 32.0 * u;
    let mut y = r.y + m;

    // TRIP: the word, big, and what it means.
    let trip = state.trip(now);
    let colour = match trip.tone() {
        Tone::Quiet => t.caption,
        Tone::Good => t.good,
        Tone::Warn => t.warn,
        Tone::Bad => t.bad,
    };
    let h = 132.0 * u;
    p.card(x, y, w, h, "TRIP");
    p.fitted(inner, y + 42.0 * u, 46.0, inner_w, trip.label(), colour);
    p.fitted(inner, y + 98.0 * u, 18.0, inner_w, trip.hint(), t.caption);
    y += h + m;

    // DISTANCE LEFT: along the plan from the car.
    let h = 150.0 * u;
    p.card(x, y, w, h, "DISTANCE LEFT");
    let remaining = state.remaining();
    let (big, note) = match remaining {
        Some(Remaining::AlongPlan(d)) => (distance(d), "along Nav2's plan"),
        Some(Remaining::StraightLine(d)) => (distance(d), "in a straight line: no plan yet"),
        None if state.goal().is_some() => ("--".to_owned(), "the car's position is unknown"),
        None => ("--".to_owned(), "no goal"),
    };
    let fresh_car = state.health(Topic::Odom, now).is_ok();
    let big_colour = if remaining.is_some() && fresh_car {
        t.text
    } else {
        t.dim
    };
    p.fitted(inner, y + 38.0 * u, 64.0, inner_w, &big, big_colour);
    p.fitted(inner, y + 116.0 * u, 18.0, inner_w, note, t.caption);
    y += h + m;

    // TIME LEFT and SPEED, side by side.
    let h = 104.0 * u;
    let half = (w - m) / 2.0;
    let speed = state.car.as_ref().map(|c| c.speed);
    p.card(x, y, half, h, "TIME LEFT");
    let eta = match (remaining, speed) {
        (Some(rem), Some(v)) if !matches!(trip, map_core::trip::Trip::Arrived) => {
            time_left(rem.metres(), f64::from(v.0)).map(format::age)
        }
        (Some(_), _) if matches!(trip, map_core::trip::Trip::Arrived) => Some("0 s".to_owned()),
        _ => None,
    };
    let (eta_text, eta_colour) = eta.map_or(("--".to_owned(), t.dim), |e| (e, t.text));
    p.fitted(
        inner,
        y + 40.0 * u,
        40.0,
        half - 32.0 * u,
        &eta_text,
        eta_colour,
    );
    let sx = x + half + m;
    p.card(sx, y, half, h, "SPEED");
    let (speed_text, speed_colour) = match speed {
        Some(v) => (
            format!("{} km/h", format::speed_km_per_hour(v)),
            if fresh_car { t.text } else { t.dim },
        ),
        None => ("--".to_owned(), t.dim),
    };
    p.fitted(
        sx + 16.0 * u,
        y + 40.0 * u,
        40.0,
        half - 32.0 * u,
        &speed_text,
        speed_colour,
    );
    y += h + m;

    // TOPICS: what's been heard.
    let keys_h = 156.0 * u;
    let row_h = 32.0 * u;
    let wanted = 44.0 * u + Topic::ALL.len() as f32 * row_h + 8.0 * u;
    let h = (r.y + r.height - m - keys_h - m - y).clamp(60.0 * u, wanted);
    p.card(x, y, w, h, "TOPICS");
    let rows = ((h - 44.0 * u) / row_h).floor().max(0.0) as usize;
    for (i, topic) in Topic::ALL.into_iter().take(rows).enumerate() {
        let ty = y + 42.0 * u + i as f32 * row_h;
        let health = state.health(topic, now);
        let status = match (topic, health) {
            // Latched or sent once: when it came means little; that it came, a lot.
            (Topic::Map | Topic::Goal | Topic::Source | Topic::StreetRoute, Health::Ok(_)) => {
                "HEARD".to_owned()
            }
            _ => format::health(health),
        };
        let colour = match health {
            Health::Ok(_) => t.good,
            Health::Stale(_) => t.warn,
            // No goal and no plan is normal; nothing announcing the source is too, and no
            // sensor watchdog (the turtle runs none).
            Health::Never
                if matches!(
                    topic,
                    Topic::Plan
                        | Topic::Goal
                        | Topic::Source
                        | Topic::Watchdog
                        | Topic::StreetRoute
                ) =>
            {
                t.dim
            }
            Health::Never => t.bad,
        };
        let sw = p.width(&status, 18.0);
        p.text(inner + inner_w - sw, ty, 18.0, &status, colour);
        p.fitted(
            inner,
            ty,
            18.0,
            inner_w - sw - 12.0 * u,
            state.config.topic(topic),
            t.text,
        );
    }

    // The keys, and where the data comes from: at the bottom.
    let y = (r.y + r.height - m - keys_h).max(y + h + m);
    p.card(x, y, w, keys_h, "KEYS");
    p.fitted(
        inner,
        y + 38.0 * u,
        17.0,
        inner_w,
        "F  follow the car / whole map",
        t.caption,
    );
    p.fitted(
        inner,
        y + 64.0 * u,
        17.0,
        inner_w,
        "wheel, + -  zoom    drag  move",
        t.caption,
    );
    p.fitted(
        inner,
        y + 90.0 * u,
        17.0,
        inner_w,
        "S  streets on / off",
        t.caption,
    );
    p.fitted(
        inner,
        y + 116.0 * u,
        17.0,
        inner_w,
        "/ or tap the box  search",
        t.caption,
    );
    p.right(x + w - 16.0 * u, y + 10.0 * u, 18.0, source, t.caption);
}

/// "42 m", "1.2 km".
fn distance(d: Metres) -> String {
    if d.0 >= 1000.0 {
        format!("{:.1} km", d.0 / 1000.0)
    } else {
        format::metres(d.0)
    }
}

/// Drawing with the font, sizes in points at the reference size.
struct Painter<'a, D: RaylibDraw> {
    d: &'a mut D,
    font: &'a MapFont,
    u: f32,
    t: &'static Theme,
    /// The blur behind the glass over the map, when it's on.
    frost: Option<&'a mut Frost>,
}

impl<D: RaylibDraw> Painter<'_, D> {
    fn width(&self, text: &str, size: f32) -> f32 {
        self.font.measure(text, size * self.u)
    }

    fn text(&mut self, x: f32, y: f32, size: f32, text: &str, colour: Rgba) {
        let px = size * self.u;
        self.d.draw_text_ex(
            self.font,
            text,
            Vector2::new(x, y),
            px,
            px / 20.0,
            colour.rl(),
        );
    }

    fn centred(&mut self, cx: f32, y: f32, size: f32, text: &str, colour: Rgba) {
        let w = self.width(text, size);
        self.text(cx - w / 2.0, y, size, text, colour);
    }

    fn right(&mut self, x_right: f32, y: f32, size: f32, text: &str, colour: Rgba) {
        let w = self.width(text, size);
        self.text(x_right - w, y, size, text, colour);
    }

    /// Text that fits in `max_w`: shrinks it, down to 10 points.
    fn fitted(&mut self, x: f32, y: f32, size: f32, max_w: f32, text: &str, colour: Rgba) {
        let mut s = size;
        while s > 10.0 && self.width(text, s) > max_w {
            s -= 1.0;
        }
        self.text(x, y, s, text, colour);
    }

    /// Glass over the map: the map blurred behind it (when the blur is on), then the
    /// see-through `tint`, a soft edge and a shine.
    fn frosted(&mut self, r: Rectangle, radius: f32, tint: Rgba) {
        let radius = self.t.style.corner(radius);
        if let Some(f) = self.frost.as_deref_mut() {
            f.behind(self.d, r, radius);
        }
        paint::glass(self.d, r, radius, tint, self.t, self.u.max(1.0));
    }

    /// A panel card: glass over the window's background, and its caption.
    fn card(&mut self, x: f32, y: f32, w: f32, h: f32, caption: &str) {
        let r = Rectangle::new(x, y, w, h);
        let u = self.u;
        let radius = self.t.style.corner(14.0 * u);
        paint::glass(self.d, r, radius, self.t.panel, self.t, u.max(1.0));
        let caption_colour = self.t.caption;
        self.text(x + 16.0 * u, y + 10.0 * u, 16.0, caption, caption_colour);
    }

    /// A label on a glass pill, over the map.
    fn chip(&mut self, x: f32, y: f32, text: &str, colour: Rgba) {
        let size = 17.0;
        let w = self.width(text, size);
        let r = Rectangle::new(x, y, w + 24.0 * self.u, 32.0 * self.u);
        self.frosted(r, r.height / 2.0, self.t.overlay);
        self.text(x + 12.0 * self.u, y + 7.0 * self.u, size, text, colour);
    }

    /// A chip centred on `cx`.
    fn chip_centred(&mut self, cx: f32, y: f32, text: &str, colour: Rgba) {
        let w = self.width(text, 17.0) + 24.0 * self.u;
        self.chip(cx - w / 2.0, y, text, colour);
    }
}

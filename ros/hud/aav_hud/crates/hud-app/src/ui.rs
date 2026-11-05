//! The driving screen, drawn with raylib.
//!
//! Everything is laid out on a 1280 x 720 canvas, then scaled to the window and centred,
//! so the screen looks the same at any size. The layout:
//!
//! ```text
//! +--------------------------------------------------------------------+
//! | MODE | ARMED | GEAR | E-STOP | BATTERY | FAULTS               DEMO |
//! +----------------+--------------------------+------------------------+
//! | SPEED          | CAMERA                   |  [  E-STOP button  ]   |
//! |   12 km/h      | (STOP SIGN banner)       |  sent / failed         |
//! |                | (SIMULATED banner)       |                        |
//! +----------------+--------------------------+------------------------+
//! | (icon) GO      | TOPICS                   | GPS                    |
//! | all REAL       |  /vehicle/status  SIM OK |  RTK FIXED             |
//! +----------------+  /sensors/gps/fix  STALE |  lat, lon, accuracy    |
//! | STEERING  ---  |                          |                        |
//! | THROTTLE  ===  |                          |                        |
//! | BRAKE     ===  |                          |                        |
//! +--------------------------------------------------------------------+
//! ```
//!
//! The GO / STOP panel answers "may the car move?" for the safety drivers: a colour, an
//! icon, a word and the one main reason (hud_core::gate). The orange SIMULATED / BENCH /
//! REPLAY banner shows whenever `/system/source` says any data isn't from the real car;
//! each topic it names gets a REAL / SIM tag under TOPICS.
//!
//! Every colour comes from the theme (`hud_core::theme`, `theme:` in config/hud.yaml):
//! the panels are frosted glass over a soft background (hud_app::paint); GO, WAIT,
//! STOP and the e-stop button stay solid.
//!
//! The arcade style (`theme: initial_d`, the default) is sharp instead: black panels
//! with thin white edges and small corners, a faint hatchback drawn in lines behind them
//! (hud_core::silhouette), the logo (tools/make_logo.py), and the speed as an arcade
//! gauge: white ticks, italic numbers, a thin red needle swinging smoothly, red redline
//! blocks from `max_speed` up, and a digital box of 7-segment digits beside a gear box
//! (hud_core::gauge, hud_core::segments).

use std::time::{Duration, Instant};

use hud_app::paint::{self, ToColor};
use hud_core::{
    config::Topic,
    format,
    gate::{Light, Verdict},
    gauge::{self, GearBox, Needle, Scale as Dial},
    gps::FixKind,
    health::Health,
    origin::Origin,
    picture::Picture,
    segments::{self, Digit, Segment},
    silhouette::Placement,
    state::{Alarm, EstopReport, HudState},
    theme::{Arcade, Rgba, Style, Theme},
    vehicle::VehicleStatus,
};
use raylib::prelude::*;

/// The canvas everything is drawn on, before scaling.
const CANVAS_W: f32 = 1280.0;
const CANVAS_H: f32 = 720.0;

/// The steering bar's ends, in degrees each way.
const STEERING_RANGE_DEG: f32 = 30.0;

/// Fonts tried in order; raylib's own small pixel font if none is there.
const FONT_FILES: [&str; 4] = [
    "/System/Library/Fonts/Supplemental/Verdana Bold.ttf", // macOS
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",   // macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", // Ubuntu, Raspberry Pi OS
    "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf", // Fedora, AlmaLinux
];
/// Italic fonts for the arcade gauge's numbers, tried in order; the upright ones if none.
const ITALIC_FONT_FILES: [&str; 4] = [
    "/System/Library/Fonts/Supplemental/Verdana Bold Italic.ttf", // macOS
    "/System/Library/Fonts/Supplemental/Arial Bold Italic.ttf",   // macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-BoldOblique.ttf", // Ubuntu, Raspberry Pi OS
    "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-BoldOblique.ttf", // Fedora, AlmaLinux
];
/// The size glyphs are rendered at; text is scaled down from it, so keep it large.
const FONT_LOAD_SIZE: i32 = 128;

/// The arcade style's logo, built in: made by tools/make_logo.py (docs/hud.md, Themes).
const LOGO_PNG: &[u8] = include_bytes!("../../../assets/hud-logo.png");

/// Panels' corners in the glass style, in canvas units; the arcade style's are sharper
/// (`Style::corner`).
const GLASS_RADIUS: f32 = 14.0;

/// Where each part of the screen is, on the canvas.
mod area {
    /// (x, y, width, height)
    pub type Area = (f32, f32, f32, f32);
    pub const STRIP: Area = (8.0, 8.0, 1264.0, 76.0);
    pub const SPEED: Area = (8.0, 94.0, 430.0, 300.0);
    pub const GATE: Area = (8.0, 404.0, 430.0, 176.0);
    pub const CONTROLS: Area = (8.0, 590.0, 430.0, 122.0);
    pub const CAMERA: Area = (448.0, 94.0, 504.0, 300.0);
    pub const TOPICS: Area = (448.0, 404.0, 504.0, 308.0);
    pub const ESTOP: Area = (962.0, 94.0, 310.0, 184.0);
    pub const ESTOP_NOTE: Area = (962.0, 284.0, 310.0, 110.0);
    pub const GPS: Area = (962.0, 404.0, 310.0, 308.0);
}

/// raylib's font handle: one loaded from a file, or its built-in one.
enum HudFont {
    File(Font),
    BuiltIn(WeakFont),
}

impl HudFont {
    fn measure(&self, text: &str, size: f32) -> f32 {
        let spacing = size / 20.0;
        match self {
            Self::File(f) => f.measure_text(text, size, spacing).x,
            Self::BuiltIn(f) => f.measure_text(text, size, spacing).x,
        }
    }
}

impl AsRef<raylib::ffi::Font> for HudFont {
    fn as_ref(&self) -> &raylib::ffi::Font {
        match self {
            Self::File(f) => f.as_ref(),
            Self::BuiltIn(f) => f.as_ref(),
        }
    }
}

/// The camera picture on the graphics card.
struct CameraTexture {
    texture: Texture2D,
    width: u32,
    height: u32,
}

/// The screen: its fonts, its colours, the camera's texture, and the arcade gauge's
/// dial, needle and logo.
pub struct Ui {
    font: HudFont,
    /// The gauge's numbers: an italic font, or the upright one again.
    italic: HudFont,
    theme: &'static Theme,
    camera: Option<CameraTexture>,
    dial: Dial,
    needle: Needle,
    /// When the needle last moved.
    last_frame: Option<Instant>,
    /// Only in the arcade style.
    logo: Option<Texture2D>,
}

/// The first font of `paths` that loads, smoothed; raylib's own if none does.
fn load_font<'a>(
    rl: &mut RaylibHandle,
    thread: &RaylibThread,
    paths: impl IntoIterator<Item = &'a &'a str>,
) -> HudFont {
    let loaded = paths
        .into_iter()
        .filter(|path| std::path::Path::new(path).exists())
        .find_map(|path| rl.load_font_ex(thread, path, FONT_LOAD_SIZE, None).ok());
    match loaded {
        Some(f) => {
            f.texture()
                .set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_BILINEAR);
            HudFont::File(f)
        }
        None => HudFont::BuiltIn(rl.get_font_default()),
    }
}

/// The built-in logo on the graphics card, mipmapped so it shrinks cleanly. `None`
/// (and the HUD goes on without it) if it won't load.
fn load_logo(rl: &mut RaylibHandle, thread: &RaylibThread) -> Option<Texture2D> {
    let image = Image::load_image_from_mem(".png", LOGO_PNG).ok()?;
    let mut texture = rl.load_texture_from_image(thread, &image).ok()?;
    texture.gen_texture_mipmaps();
    texture.set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_TRILINEAR);
    Some(texture)
}

impl Ui {
    /// Loads the fonts (and the logo, in the arcade style). Needs the window open.
    pub fn new(
        rl: &mut RaylibHandle,
        thread: &RaylibThread,
        theme: &'static Theme,
        dial: Dial,
    ) -> Self {
        let font = load_font(rl, thread, &FONT_FILES);
        let italic = load_font(rl, thread, ITALIC_FONT_FILES.iter().chain(&FONT_FILES));
        let logo = match theme.style {
            Style::Arcade(_) => load_logo(rl, thread),
            Style::Glass => None,
        };
        Self {
            font,
            italic,
            theme,
            camera: None,
            dial,
            needle: Needle::default(),
            last_frame: None,
            logo,
        }
    }

    /// Puts a new camera picture on the graphics card.
    pub fn upload(
        &mut self,
        rl: &mut RaylibHandle,
        thread: &RaylibThread,
        picture: &Picture,
    ) -> anyhow::Result<()> {
        let same_size = self
            .camera
            .as_ref()
            .is_some_and(|c| c.width == picture.width && c.height == picture.height);
        if !same_size {
            let (w, h) = (to_i32(picture.width)?, to_i32(picture.height)?);
            // gen_image_color makes RGBA8 pixels, the layout Picture has.
            let image = Image::gen_image_color(w, h, Color::BLACK);
            let texture = rl.load_texture_from_image(thread, &image)?;
            texture.set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_BILINEAR);
            self.camera = Some(CameraTexture {
                texture,
                width: picture.width,
                height: picture.height,
            });
        }
        if let Some(c) = self.camera.as_mut() {
            c.texture.update_texture(&picture.rgba)?;
        }
        Ok(())
    }

    /// The e-stop button's place in the window, in window pixels.
    pub fn estop_button(&self, window_w: i32, window_h: i32) -> Rectangle {
        Scale::new(window_w, window_h).rect(area::ESTOP)
    }

    /// Draws one frame. `hover`: the pointer is over the e-stop button.
    pub fn draw(
        &mut self,
        d: &mut RaylibDrawHandle,
        state: &HudState,
        now: Instant,
        source: &str,
        hover: bool,
    ) {
        let scale = Scale::new(d.get_screen_width(), d.get_screen_height());
        let blink = (d.get_time() * 2.0).fract() < 0.5;
        let t = self.theme;
        paint::backdrop(d, d.get_screen_width(), d.get_screen_height(), t);
        let vehicle = Fresh::of(state, Topic::VehicleStatus, now, state.vehicle, t.dim);
        // The needle swings towards the speed; with no fresh speed it falls home.
        let dt = self
            .last_frame
            .map_or(Duration::ZERO, |last| now.saturating_duration_since(last));
        self.last_frame = Some(now);
        let target = match (vehicle.value, vehicle.health.is_ok()) {
            (Some(v), true) => v.speed.to_km_per_hour().abs(),
            _ => 0.0,
        };
        let needle = self.needle.follow(target, dt);
        let mut c = Canvas {
            d,
            font: &self.font,
            italic: &self.italic,
            scale,
            t,
            radius: t.style.corner(GLASS_RADIUS),
        };
        car_behind(&mut c);
        strip(&mut c, state, &vehicle, now, source);
        match t.style {
            Style::Glass => speed(&mut c, &vehicle),
            Style::Arcade(look) => {
                let gauge = Gauge {
                    dial: &self.dial,
                    needle,
                    look,
                    logo: self.logo.as_ref(),
                };
                arcade_speed(&mut c, &vehicle, &gauge);
            }
        }
        gate(&mut c, &state.verdict(now));
        controls(&mut c, &vehicle);
        camera(&mut c, state, now, self.camera.as_ref(), blink);
        topics(&mut c, state, now);
        estop(&mut c, state, &vehicle, hover);
        gps(&mut c, state, now);
    }
}

fn to_i32(n: u32) -> anyhow::Result<i32> {
    Ok(i32::try_from(n)?)
}

/// The newest message of one topic, and whether it's fresh.
struct Fresh<T> {
    value: Option<T>,
    health: Health,
    /// The colour of old news.
    dim: Rgba,
}

impl<T: Copy> Fresh<T> {
    fn of(state: &HudState, topic: Topic, now: Instant, value: Option<T>, dim: Rgba) -> Self {
        Self {
            value,
            health: state.health(topic, now),
            dim,
        }
    }

    /// The text colour: normal when fresh, dim when old.
    fn colour(&self, fresh: Rgba) -> Rgba {
        if self.health.is_ok() {
            fresh
        } else {
            self.dim
        }
    }
}

/// From the 1280 x 720 canvas to the window: one scale, and the offset that centres it.
#[derive(Clone, Copy)]
struct Scale {
    k: f32,
    x0: f32,
    y0: f32,
}

impl Scale {
    fn new(window_w: i32, window_h: i32) -> Self {
        let (w, h) = (window_w as f32, window_h as f32);
        let k = (w / CANVAS_W).min(h / CANVAS_H).max(0.01);
        Self {
            k,
            x0: (w - CANVAS_W * k) / 2.0,
            y0: (h - CANVAS_H * k) / 2.0,
        }
    }

    fn rect(self, (x, y, w, h): area::Area) -> Rectangle {
        Rectangle::new(
            self.x0 + x * self.k,
            self.y0 + y * self.k,
            w * self.k,
            h * self.k,
        )
    }
}

/// Drawing in canvas units.
struct Canvas<'a, 'b> {
    d: &'a mut RaylibDrawHandle<'b>,
    font: &'a HudFont,
    italic: &'a HudFont,
    scale: Scale,
    t: &'static Theme,
    /// A panel's corners, in canvas units.
    radius: f32,
}

impl Canvas<'_, '_> {
    /// A frosted-glass panel.
    fn glass(&mut self, a: area::Area) {
        let r = self.scale.rect(a);
        let k = self.scale.k;
        paint::glass(self.d, r, self.radius * k, self.t.panel, self.t, k.max(1.0));
    }

    /// A solid panel: GO / STOP, the e-stop button.
    fn fill(&mut self, a: area::Area, colour: Rgba) {
        let r = self.scale.rect(a);
        paint::rounded(self.d, r, self.radius * self.scale.k, colour);
    }

    fn outline(&mut self, a: area::Area, thick: f32, colour: Rgba) {
        let r = self.scale.rect(a);
        let k = self.scale.k;
        paint::rounded_edge(self.d, r, self.radius * k, thick * k, colour);
    }

    /// A bar, a little rounded: the controls' and the banners.
    fn bar(&mut self, a: area::Area, colour: Rgba) {
        let r = self.scale.rect(a);
        paint::rounded(self.d, r, 6.0 * self.scale.k, colour);
    }

    fn width(&self, text: &str, size: f32) -> f32 {
        self.font.measure(text, size * self.scale.k) / self.scale.k
    }

    fn text(&mut self, x: f32, y: f32, size: f32, text: &str, colour: Rgba) {
        let s = self.scale;
        let pos = Vector2::new(s.x0 + x * s.k, s.y0 + y * s.k);
        let px = size * s.k;
        self.d
            .draw_text_ex(self.font, text, pos, px, px / 20.0, colour.rl());
    }

    /// Italic words, centred on (cx, cy): the gauge's numbers.
    fn italic_centred(&mut self, cx: f32, cy: f32, size: f32, text: &str, colour: Rgba) {
        let s = self.scale;
        let px = size * s.k;
        let w = self.italic.measure(text, px) / s.k;
        let pos = Vector2::new(s.x0 + (cx - w / 2.0) * s.k, s.y0 + (cy - size / 2.0) * s.k);
        self.d
            .draw_text_ex(self.italic, text, pos, px, px / 20.0, colour.rl());
    }

    /// A canvas point on the window.
    fn at(&self, (x, y): (f32, f32)) -> Vector2 {
        let s = self.scale;
        Vector2::new(s.x0 + x * s.k, s.y0 + y * s.k)
    }

    /// A straight line, `thick` canvas units wide.
    fn line(&mut self, a: (f32, f32), b: (f32, f32), thick: f32, colour: Rgba) {
        let (a, b) = (self.at(a), self.at(b));
        self.d
            .draw_line_ex(a, b, (thick * self.scale.k).max(1.0), colour.rl());
    }

    /// A filled convex shape, its corners in order round it (either way round).
    fn convex(&mut self, corners: &[(f32, f32)], colour: Rgba) {
        let Some((&first, rest)) = corners.split_first() else {
            return;
        };
        for pair in rest.windows(2) {
            let (a, mut b, mut c) = (self.at(first), self.at(pair[0]), self.at(pair[1]));
            // raylib fills a triangle only when its corners run anticlockwise on the
            // screen: with y down, that's a negative cross product.
            if (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0.0 {
                std::mem::swap(&mut b, &mut c);
            }
            self.d.draw_triangle(a, b, c, colour.rl());
        }
    }

    /// A ring's piece, `from` to `to` radians (clockwise from right), between two radii.
    fn ring(&mut self, centre: (f32, f32), radii: (f32, f32), angles: (f32, f32), fill: Rgba) {
        let k = self.scale.k;
        let c = self.at(centre);
        self.d.draw_ring(
            c,
            radii.0 * k,
            radii.1 * k,
            angles.0.to_degrees(),
            angles.1.to_degrees(),
            48,
            fill.rl(),
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

    /// A panel: its glass and its caption.
    fn panel(&mut self, a: area::Area, caption: &str) {
        self.glass(a);
        self.text(a.0 + 14.0, a.1 + 10.0, 18.0, caption, self.t.caption);
    }

    /// The light's icon, centred at `centre` (canvas units), radius `r`: a tick in a
    /// circle (GO), a pause sign (WAIT), a stop sign (STOP), a question mark (no gate).
    fn icon(&mut self, light: Light, centre: (f32, f32), r: f32, fill: Rgba, ink: Rgba) {
        let s = self.scale;
        let (fill_c, ink_c) = (fill.rl(), ink.rl());
        let at = |x: f32, y: f32| Vector2::new(s.x0 + x * s.k, s.y0 + y * s.k);
        let (cx, cy) = centre;
        let (k, c) = (s.k, at(cx, cy));
        if light == Light::Stop {
            // An octagon, flat on top, as a road's stop sign: a rim, and a bar across.
            self.d.draw_poly(c, 8, r * k, 22.5, ink_c);
            self.d.draw_poly(c, 8, (r - 6.0) * k, 22.5, fill_c);
        } else {
            self.d.draw_circle_v(c, r * k, ink_c);
            self.d.draw_circle_v(c, (r - 6.0) * k, fill_c);
        }
        let rect = |x: f32, y: f32, w: f32, h: f32| {
            Rectangle::new(
                s.x0 + (cx + x * r) * k,
                s.y0 + (cy + y * r) * k,
                w * r * k,
                h * r * k,
            )
        };
        match light {
            Light::Stop => self
                .d
                .draw_rectangle_rec(rect(-0.6, -0.16, 1.2, 0.32), ink_c),
            Light::Go => {
                let thick = r * 0.2 * k;
                let (a, b, e) = (
                    at(cx - r * 0.45, cy + r * 0.02),
                    at(cx - r * 0.12, cy + r * 0.36),
                    at(cx + r * 0.48, cy - r * 0.34),
                );
                self.d.draw_line_ex(a, b, thick, ink_c);
                self.d.draw_line_ex(b, e, thick, ink_c);
                self.d.draw_circle_v(b, thick / 2.0, ink_c);
            }
            Light::Wait => {
                self.d
                    .draw_rectangle_rec(rect(-0.32, -0.42, 0.2, 0.84), ink_c);
                self.d
                    .draw_rectangle_rec(rect(0.12, -0.42, 0.2, 0.84), ink_c);
            }
            Light::Unknown => self.centred(cx, cy - r * 0.55, r * 1.1, "?", ink),
        }
    }

    /// Text that fits in `max_w`: shrinks it down to `min_size`, no smaller.
    fn fitted(&mut self, x: f32, y: f32, size: f32, max_w: f32, text: &str, colour: Rgba) {
        let mut s = size;
        while s > 12.0 && self.width(text, s) > max_w {
            s -= 2.0;
        }
        self.text(x, y, s, text, colour);
    }
}

fn health_colour(t: &Theme, h: Health) -> Rgba {
    match h {
        Health::Ok(_) => t.good,
        Health::Stale(_) => t.warn,
        Health::Never => t.bad,
    }
}

/// The top strip: mode, armed, gear, e-stop, battery, faults, and the source.
fn strip(
    c: &mut Canvas,
    state: &HudState,
    vehicle: &Fresh<VehicleStatus>,
    now: Instant,
    source: &str,
) {
    let (x, y, _, h) = area::STRIP;
    c.glass(area::STRIP);
    let v = vehicle.value;
    let dash = || ("--".to_owned(), c.t.dim);
    let mode = v.map_or_else(dash, |v| (v.mode.to_string(), c.t.text));
    let armed = v.map_or_else(dash, |v| {
        if v.armed {
            ("ARMED".into(), c.t.good)
        } else {
            ("DISARMED".into(), c.t.warn)
        }
    });
    let gear = v.map_or_else(dash, |v| (v.gear().letter().into(), c.t.text));
    let estop = v.map_or_else(dash, |v| {
        if v.estop_active {
            ("ON".into(), c.t.bad)
        } else {
            ("off".into(), c.t.good)
        }
    });
    let battery = Fresh::of(state, Topic::Battery, now, state.battery, c.t.dim);
    let charge = battery.value.map_or_else(dash, |b| {
        let text = match (b.percent, b.voltage) {
            (Some(p), _) => format!("{p:.0} %"),
            (None, Some(v)) => format!("{v:.1} V"),
            (None, None) => "?".into(),
        };
        let low = b.percent.is_some_and(|p| p < 20.0);
        (text, battery.colour(if low { c.t.bad } else { c.t.text }))
    });
    let faults = v.map_or_else(dash, |v| {
        if v.faults.is_empty() {
            ("none".into(), c.t.good)
        } else {
            (v.faults.to_string(), c.t.bad)
        }
    });
    let cells: [(&str, (String, Rgba), f32); 6] = [
        ("MODE", mode, 190.0),
        ("COMPUTER", armed, 180.0),
        ("GEAR", gear, 90.0),
        ("E-STOP", estop, 120.0),
        ("BATTERY", charge, 140.0),
        ("FAULTS", faults, 420.0),
    ];
    let mut cx = x + 16.0;
    for (caption, (value, colour), w) in cells {
        // Old news is grey, whatever it says.
        let colour = if caption == "BATTERY" {
            colour
        } else {
            vehicle.colour(colour)
        };
        c.text(cx, y + 8.0, 16.0, caption, c.t.caption);
        c.fitted(cx, y + 30.0, 34.0, w - 14.0, &value, colour);
        cx += w;
    }
    let tag = match vehicle.health {
        Health::Ok(_) => source.to_owned(),
        Health::Stale(age) => format!("{source}: status {}", format::age(age)),
        Health::Never => format!("{source}: no status"),
    };
    let tag_colour = if vehicle.health.is_ok() {
        c.t.caption
    } else {
        c.t.warn
    };
    c.right(
        area::STRIP.0 + area::STRIP.2 - 14.0,
        y + h / 2.0 - 10.0,
        20.0,
        &tag,
        tag_colour,
    );
}

/// The big speed.
fn speed(c: &mut Canvas, vehicle: &Fresh<VehicleStatus>) {
    let a = area::SPEED;
    c.panel(a, "SPEED");
    let cx = a.0 + a.2 / 2.0;
    match vehicle.value {
        Some(v) => {
            let colour = vehicle.colour(c.t.text);
            c.centred(
                cx,
                a.1 + 50.0,
                170.0,
                &format::speed_km_per_hour(v.speed),
                colour,
            );
            c.centred(cx, a.1 + 226.0, 30.0, "km/h", c.t.caption);
        }
        None => c.centred(cx, a.1 + 50.0, 170.0, "--", c.t.dim),
    }
    if let Health::Stale(age) = vehicle.health {
        c.centred(
            cx,
            a.1 + 262.0,
            24.0,
            &format!("OLD: {}", format::age(age)),
            c.t.warn,
        );
    } else if vehicle.health == Health::Never {
        c.centred(cx, a.1 + 262.0, 24.0, "no vehicle status", c.t.bad);
    }
}

/// What the arcade gauge needs beyond the canvas.
struct Gauge<'a> {
    dial: &'a Dial,
    /// Where the needle points now, km/h (it lags the speed: hud_core::gauge::Needle).
    needle: f32,
    look: Arcade,
    logo: Option<&'a Texture2D>,
}

/// The arcade gauge's parts, in canvas units from the speed panel's corner.
mod dial {
    /// The dial's centre, and the radius its ticks reach out to.
    pub const CENTRE: (f32, f32) = (152.0, 162.0);
    pub const RADIUS: f32 = 118.0;
    /// The redline's blocks: a band just outside the ticks.
    pub const RED_BAND: (f32, f32) = (5.0, 17.0);
    /// Ticks: (length, thickness), major and minor.
    pub const MAJOR: (f32, f32) = (20.0, 3.0);
    pub const MINOR: (f32, f32) = (9.0, 1.5);
    /// The numbers: how far in from the ticks' ends, and their size.
    pub const NUMBER_IN: f32 = 42.0;
    pub const NUMBER_SIZE: f32 = 28.0;
    /// The right column: the logo, the speed's digits, the gear.
    pub const COLUMN_X: f32 = 292.0;
    pub const COLUMN_W: f32 = 126.0;
    /// The logo: wider than the column, over the dial's top right.
    pub const LOGO_X: f32 = 256.0;
    pub const LOGO_W: f32 = 162.0;
    pub const LOGO_Y: f32 = 16.0;
    pub const READOUT_Y: f32 = 136.0;
    pub const READOUT_H: f32 = 70.0;
    pub const GEAR_Y: f32 = 218.0;
    pub const GEAR_H: f32 = 44.0;
}

/// The speed as an arcade gauge (the arcade style): the dial and its needle, the logo,
/// the digital box and the gear box (hud_core::gauge, hud_core::segments).
fn arcade_speed(c: &mut Canvas, vehicle: &Fresh<VehicleStatus>, g: &Gauge) {
    let a = area::SPEED;
    c.glass(a);
    let look = g.look;
    let dial = g.dial;
    let centre = (a.0 + dial::CENTRE.0, a.1 + dial::CENTRE.1);
    let r = dial::RADIUS;
    let ends = (dial.angle(0.0), dial.angle(dial.full_scale()));
    c.ring(centre, (r - 1.0, r + 0.5), ends, look.tick);
    for (from, to) in dial.redline_blocks() {
        let (inner, outer) = dial::RED_BAND;
        let angles = (dial.angle(from), dial.angle(to));
        c.ring(centre, (r + inner, r + outer), angles, look.redline);
    }
    for tick in dial.ticks() {
        let angle = dial.angle(tick.kmh);
        let (length, thick) = if tick.major { dial::MAJOR } else { dial::MINOR };
        let inner = gauge::point(centre, r - length, angle);
        c.line(inner, gauge::point(centre, r, angle), thick, look.tick);
        if let Some(label) = tick.label() {
            let (x, y) = gauge::point(centre, r - dial::NUMBER_IN, angle);
            c.italic_centred(x, y, dial::NUMBER_SIZE, &label, look.number);
        }
    }
    c.italic_centred(centre.0, centre.1 + 50.0, 20.0, "km/h", c.t.caption);
    // The needle: thin, from a little behind the hub to just short of the ticks.
    let angle = dial.angle(g.needle);
    let tail = gauge::point(centre, -16.0, angle);
    c.line(tail, gauge::point(centre, r - 6.0, angle), 3.0, look.needle);
    let k = c.scale.k;
    let hub = c.at(centre);
    c.d.draw_circle_v(hub, 9.0 * k, look.needle.rl());
    c.d.draw_circle_v(hub, 4.0 * k, c.t.background.rl());

    let x = a.0 + dial::COLUMN_X;
    if let Some(logo) = g.logo {
        let (w, h) = (logo.width() as f32, logo.height() as f32);
        let height = dial::LOGO_W * h / w;
        let place = (a.0 + dial::LOGO_X, a.1 + dial::LOGO_Y, dial::LOGO_W, height);
        let dest = c.scale.rect(place);
        let src = Rectangle::new(0.0, 0.0, w, h);
        c.d.draw_texture_pro(logo, src, dest, Vector2::zero(), 0.0, Color::WHITE);
    }
    let speed = vehicle.value.map(|v| v.speed);
    let readout = (x, a.1 + dial::READOUT_Y, dial::COLUMN_W, dial::READOUT_H);
    digital_box(
        c,
        readout,
        &gauge::readout(speed),
        &look,
        vehicle.colour(look.segment_on),
    );

    // The gear box, as the arcade's "MT 1": GEAR and the letter; E-STOP fills it.
    let gear = (x, a.1 + dial::GEAR_Y, dial::COLUMN_W, dial::GEAR_H);
    c.fill(gear, look.readout);
    c.outline(gear, 1.5, look.readout_edge);
    let gearbox = GearBox::of(vehicle.value.as_ref());
    let ink = match gearbox {
        GearBox::Estop => c.t.bad,
        GearBox::Gear(_) => vehicle.colour(c.t.text),
        GearBox::Unknown => c.t.dim,
    };
    let word = gearbox.label();
    if gearbox == GearBox::Estop {
        let size = 30.0_f32.min(30.0 * (gear.2 - 16.0) / c.width(word, 30.0).max(1.0));
        let cx = gear.0 + gear.2 / 2.0;
        c.centred(cx, gear.1 + (gear.3 - size) / 2.0, size, word, ink);
    } else {
        c.text(gear.0 + 12.0, gear.1 + 13.0, 18.0, "GEAR", c.t.caption);
        c.right(gear.0 + gear.2 - 14.0, gear.1 + 7.0, 30.0, word, ink);
    }

    let notice = match vehicle.health {
        Health::Stale(age) => Some((format!("OLD: {}", format::age(age)), c.t.warn)),
        Health::Never => Some(("no vehicle status".to_owned(), c.t.bad)),
        Health::Ok(_) => None,
    };
    if let Some((words, colour)) = notice {
        let w = c.width(&words, 22.0).min(260.0);
        c.fitted(centre.0 - w / 2.0, a.1 + 268.0, 22.0, 260.0, &words, colour);
    }
}

/// A digital box: `text` in three 7-segment digits, every segment there dim, the lit
/// ones in `lit`.
fn digital_box(c: &mut Canvas, a: area::Area, text: &str, look: &Arcade, lit: Rgba) {
    c.fill(a, look.readout);
    c.outline(a, 1.5, look.readout_edge);
    const CELLS: usize = 3;
    let (w, h) = (26.0, a.3 - 22.0);
    let pitch = (a.2 - 24.0) / CELLS as f32;
    for (i, cell) in segments::cells(text, CELLS).into_iter().enumerate() {
        let digit = Digit {
            x: a.0 + 10.0 + i as f32 * pitch,
            y: a.1 + 11.0,
            width: w,
            height: h,
            thick: 6.0,
            slant: 0.12,
        };
        for segment in Segment::ALL {
            let colour = if cell.lit.has(segment) {
                lit
            } else {
                look.segment_off
            };
            c.convex(&digit.polygon(segment), colour);
        }
        let point = if cell.point { lit } else { look.segment_off };
        c.convex(&digit.point(), point);
    }
}

/// The arcade style's faint car, in lines across the window behind the panels.
fn car_behind(c: &mut Canvas) {
    // Low on the right, behind TOPICS and GPS, where the panels hold fewest words.
    const LEFT: f32 = 600.0;
    const LENGTH: f32 = 660.0;
    const GROUND: f32 = 704.0;
    let s = c.scale;
    let place = Placement {
        left: s.x0 + LEFT * s.k,
        ground: s.y0 + GROUND * s.k,
        length: LENGTH * s.k,
    };
    paint::faint_car(c.d, c.t, place, 2.0 * s.k, paint::Under::Panels);
}

/// May the car move? The whole panel in the light's colour, its icon, the word, the one
/// main reason, and what a person can do (hud_core::gate).
fn gate(c: &mut Canvas, verdict: &Verdict) {
    let a = area::GATE;
    let t = c.t;
    let (fill, ink) = match verdict.light {
        Light::Go => (t.go_fill, t.go_ink),
        Light::Wait => (t.wait_fill, t.wait_ink),
        Light::Stop => (t.stop_fill, t.stop_ink),
        Light::Unknown => (t.panel, t.caption),
    };
    c.fill(a, fill);
    let r = 52.0;
    let centre = (a.0 + 22.0 + r, a.1 + a.3 / 2.0);
    c.icon(verdict.light, centre, r, fill, ink);
    let x = a.0 + 2.0 * r + 44.0;
    let w = a.0 + a.2 - 14.0 - x;
    c.fitted(x, a.1 + 14.0, 64.0, w, verdict.headline, ink);
    c.fitted(x, a.1 + 88.0, 30.0, w, &verdict.reason, ink);
    c.fitted(x, a.1 + 132.0, 20.0, w, verdict.hint, ink);
}

/// Steering, throttle and brake, as bars: one row each.
fn controls(c: &mut Canvas, vehicle: &Fresh<VehicleStatus>) {
    let a = area::CONTROLS;
    c.glass(a);
    let (x, right) = (a.0 + 16.0, a.0 + a.2 - 16.0);
    let (bar_x, bar_w) = (x + 130.0, right - 96.0 - (x + 130.0));
    let v = vehicle.value;
    let steer_deg = v.map(|v| v.steering_angle.to_degrees());
    let rows = ["STEERING", "THROTTLE", "BRAKE"];
    for (i, label) in rows.into_iter().enumerate() {
        let y = a.1 + 12.0 + i as f32 * 36.0;
        c.text(x, y + 4.0, 18.0, label, c.t.caption);
        c.bar((bar_x, y + 2.0, bar_w, 26.0), c.t.track);
        let value = match (i, v) {
            (_, None) => "--".to_owned(),
            (0, Some(_)) => format::steering(steer_deg.unwrap_or(0.0)),
            (1, Some(v)) => format!("{:.0} %", v.throttle.get() * 100.0),
            (_, Some(v)) => format!("{:.0} %", v.brake.get() * 100.0),
        };
        c.right(right, y + 3.0, 20.0, &value, vehicle.colour(c.t.text));
        let Some(v) = v else { continue };
        let fill = vehicle.colour(match i {
            0 => c.t.text,
            1 => c.t.good,
            _ => c.t.bad,
        });
        if i == 0 {
            // Centre = straight; left of centre = turning left (positive, REP-103).
            let k = (steer_deg.unwrap_or(0.0) / STEERING_RANGE_DEG).clamp(-1.0, 1.0);
            let mid = bar_x + bar_w / 2.0;
            c.bar((mid - 1.0, y, 2.0, 30.0), c.t.caption);
            let marker = mid - k * (bar_w / 2.0 - 5.0);
            c.bar((marker - 5.0, y + 2.0, 10.0, 26.0), fill);
        } else {
            let level = if i == 1 { v.throttle } else { v.brake };
            c.bar((bar_x, y + 2.0, bar_w * level.get(), 26.0), fill);
        }
    }
}

/// The camera picture, and the stop-sign banner over it.
fn camera(
    c: &mut Canvas,
    state: &HudState,
    now: Instant,
    texture: Option<&CameraTexture>,
    blink: bool,
) {
    let a = area::CAMERA;
    let topic = state.config.topic(Topic::Camera);
    c.panel(a, &format!("CAMERA  {topic}"));
    let inner = (a.0 + 10.0, a.1 + 36.0, a.2 - 20.0, a.3 - 46.0);
    c.bar(inner, c.t.camera_back);
    let health = state.health(Topic::Camera, now);
    if let Some(t) = texture {
        // Fit the picture in the panel, keeping its shape.
        let (tw, th) = (t.width as f32, t.height as f32);
        let k = (inner.2 / tw).min(inner.3 / th);
        let (w, h) = (tw * k, th * k);
        let dest = (
            inner.0 + (inner.2 - w) / 2.0,
            inner.1 + (inner.3 - h) / 2.0,
            w,
            h,
        );
        let src = Rectangle::new(0.0, 0.0, tw, th);
        let tint = if health.is_ok() { Rgba::WHITE } else { c.t.dim };
        let dest = c.scale.rect(dest);
        c.d.draw_texture_pro(&t.texture, src, dest, Vector2::zero(), 0.0, tint.rl());
    }
    let cx = inner.0 + inner.2 / 2.0;
    let cy = inner.1 + inner.3 / 2.0;
    match health {
        Health::Never => c.centred(cx, cy - 14.0, 28.0, "NO PICTURE YET", c.t.bad),
        Health::Stale(age) => {
            let msg = format!("CAMERA STOPPED {}", format::age(age));
            c.centred(cx, cy - 14.0, 28.0, &msg, c.t.warn);
        }
        Health::Ok(_) => {}
    }
    // Not the real car: a big orange band along the bottom, always on.
    let banner_h = 70.0;
    let not_real = state.origins.banner();
    if let Some(words) = &not_real {
        let band = (inner.0, inner.1 + inner.3 - banner_h, inner.2, banner_h);
        c.bar(band, c.t.not_real);
        c.centred(cx, band.1 + 8.0, 52.0, words, c.t.not_real_ink);
    }
    let lift = if not_real.is_some() { banner_h } else { 0.0 };
    if let Some(why) = &state.picture_problem {
        c.fitted(
            inner.0 + 8.0,
            inner.1 + inner.3 - 30.0 - lift,
            18.0,
            inner.2 - 16.0,
            why,
            c.t.warn,
        );
    }
    let mut top = inner.1;
    if state.stop_sign && state.health(Topic::StopSign, now).is_ok() {
        let banner = (inner.0, inner.1, inner.2, 84.0);
        let fill = if blink {
            c.t.stop_fill_alt
        } else {
            c.t.stop_fill
        };
        c.bar(banner, fill);
        c.centred(cx, inner.1 + 14.0, 56.0, "STOP SIGN", c.t.stop_ink);
        top += 84.0;
    }
    // The sensor watchdog: what failed, and whether Nav2 and the AI slot stopped for it.
    if let Some(alarm) = state.sensor_alarm(now) {
        let th = c.t;
        let (text, colour, ink) = match &alarm {
            Alarm::Stop(why) => {
                let fill = if blink {
                    th.stop_fill_alt
                } else {
                    th.stop_fill
                };
                (why, fill, th.stop_ink)
            }
            Alarm::Warn(why) => (why, th.wait_fill, th.wait_ink),
        };
        c.bar((inner.0, top, inner.2, 40.0), colour);
        c.fitted(inner.0 + 8.0, top + 9.0, 22.0, inner.2 - 16.0, text, ink);
    }
}

/// Every topic the HUD listens to: OK / STALE / NEVER, how long ago, and where its data
/// comes from (REAL, SIM...) when `/system/source` says.
fn topics(c: &mut Canvas, state: &HudState, now: Instant) {
    let a = area::TOPICS;
    c.panel(a, "TOPICS");
    let (x, right) = (a.0 + 16.0, a.0 + a.2 - 16.0);
    for (i, (name, health)) in state.topic_health(now).enumerate() {
        let y = a.1 + 42.0 + i as f32 * 43.0;
        let status = format::health(health);
        let status_w = c.width(&status, 24.0);
        c.right(right, y, 24.0, &status, health_colour(c.t, health));
        let mut name_right = right - status_w - 16.0;
        if let Some(origin) = state.origins.of_topic(name) {
            let tag = origin.tag();
            let colour = if origin == Origin::Real {
                c.t.caption
            } else {
                c.t.not_real_text
            };
            c.right(name_right, y + 2.0, 20.0, tag, colour);
            name_right -= c.width(tag, 20.0) + 12.0;
        }
        c.fitted(x, y, 24.0, name_right - x, name, c.t.text);
    }
}

/// The e-stop button, and what came of the last press.
fn estop(c: &mut Canvas, state: &HudState, vehicle: &Fresh<VehicleStatus>, hover: bool) {
    let a = area::ESTOP;
    let t = c.t;
    c.fill(a, if hover { t.stop_fill_alt } else { t.stop_fill });
    c.outline(a, 4.0, t.stop_ink);
    let cx = a.0 + a.2 / 2.0;
    c.centred(cx, a.1 + 40.0, 72.0, "E-STOP", t.stop_ink);
    c.centred(cx, a.1 + 128.0, 20.0, "tap to stop the car", t.stop_ink);

    let n = area::ESTOP_NOTE;
    let car_stopped = vehicle.health.is_ok() && vehicle.value.is_some_and(|v| v.estop_active);
    let estop_topic = state.config.topic(Topic::Estop);
    let lines: Vec<(String, Rgba)> = match state.estop {
        None if car_stopped => vec![("The car says: E-STOP ON".into(), c.t.bad)],
        None => vec![("Not pressed".into(), c.t.dim)],
        Some(EstopReport::Sent { subscribers: 0 }) => vec![
            ("E-STOP FAILED".into(), c.t.bad),
            (format!("nothing listens on {estop_topic}"), c.t.bad),
            ("Press the car's red button!".into(), c.t.bad),
        ],
        Some(EstopReport::Sent { subscribers }) => vec![
            (format!("SENT to {subscribers} node(s)"), c.t.good),
            if car_stopped {
                ("The car says: E-STOP ON".into(), c.t.bad)
            } else {
                ("waiting for the car...".into(), c.t.warn)
            },
            ("Clear it from a laptop".into(), c.t.caption),
        ],
        Some(EstopReport::Failed) => vec![
            ("E-STOP FAILED: not sent".into(), c.t.bad),
            ("Press the car's red button!".into(), c.t.bad),
        ],
    };
    for (i, (line, colour)) in lines.iter().enumerate() {
        let y = n.1 + 8.0 + i as f32 * 32.0;
        c.fitted(n.0 + 8.0, y, 24.0, n.2 - 16.0, line, *colour);
    }
}

/// Is there GPS data? The answer first, big; then the numbers.
fn gps(c: &mut Canvas, state: &HudState, now: Instant) {
    let a = area::GPS;
    let topic = state.config.topic(Topic::Gps);
    c.panel(a, &format!("GPS  {topic}"));
    let (x, w) = (a.0 + 16.0, a.2 - 32.0);
    let health = state.health(Topic::Gps, now);
    let fix = state.gps;
    let (headline, colour) = match (health, fix) {
        (Health::Never, _) | (_, None) => ("NO GPS DATA".to_owned(), c.t.bad),
        (Health::Stale(_), _) => ("GPS SILENT".to_owned(), c.t.warn),
        (Health::Ok(_), Some(f)) => (f.kind.to_string(), kind_colour(c.t, f.kind)),
    };
    c.fitted(x, a.1 + 40.0, 44.0, w, &headline, colour);

    let dim_if_old = |col: Rgba| if health.is_ok() { col } else { c.t.dim };
    let mut rows: Vec<(&str, String)> = Vec::new();
    if let Some(f) = fix {
        if !health.is_ok() {
            rows.push(("LAST", f.kind.to_string()));
        }
        match f.position {
            Some((lat, lon)) => {
                rows.push(("LAT", format::latitude(lat)));
                rows.push(("LON", format::longitude(lon)));
            }
            None => rows.push(("POS", "none".into())),
        }
        let acc = f
            .horizontal_sigma
            .map_or("not given".into(), |s| format!("+/- {}", format::metres(s)));
        rows.push(("ACC", acc));
    }
    let age = health.age().map_or("never".into(), format::age);
    rows.push(("AGE", age));
    for (i, (label, value)) in rows.iter().enumerate() {
        let y = a.1 + 104.0 + i as f32 * 38.0;
        c.text(x, y, 22.0, label, c.t.caption);
        let colour = if *label == "AGE" {
            health_colour(c.t, health)
        } else {
            dim_if_old(c.t.text)
        };
        c.fitted(x + 70.0, y, 24.0, w - 70.0, value, colour);
    }
}

fn kind_colour(t: &Theme, kind: FixKind) -> Rgba {
    match kind {
        FixKind::NoFix => t.bad,
        FixKind::Unknown | FixKind::Fix2d | FixKind::RtkFloat => t.warn,
        FixKind::Fix3d | FixKind::Dgps | FixKind::RtkFixed => t.good,
    }
}

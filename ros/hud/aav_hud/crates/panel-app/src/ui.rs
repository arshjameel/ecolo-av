//! The hardware panel, drawn with raylib: big buttons on top, every sensor below.
//!
//! ```text
//! +--------------------------------------------------------------+
//! | HARDWARE PANEL   SIM   ROS                       [ E-STOP ]  |
//! | SPEED BUTTONS ON: cap 1.0 m/s  (or OFF, and why)             |
//! |  SPEED            BRAKE              STEERING                |
//! | [Speed +][Speed -] [    STOP    ]    [< LEFT][RIGHT >]       |
//! |  asked 0.30 m/s    brake 18 %        [    CENTRE    ]        |
//! |  seen  0.28 m/s    throttle 10 %     asked +0.10  seen +0.09 |
//! | [            HOLD SPACE (OR HERE) TO DRIVE              ]    |
//! | driving / let GO go first (zeroed: the e-stop) / held: ...   |
//! | SENSORS  IMU OK acc ... gyro ...                             |
//! |          wheels, GPS, lidar Hz, radar Hz, camera Hz, battery |
//! +--------------------------------------------------------------+
//! ```
//!
//! Two pages, by the tabs or Tab: DRIVE (above) and STEERING CALIBRATION
//! ([`calibration`]: find the steering actuator's ends and centre, the limits, the
//! lock-to-lock check).
//!
//! Every colour comes from the theme (`hud_core::theme`, `theme:` in
//! config/hw_panel.yaml), as the HUD's: the pills, the buttons, the E-STOP's red. In
//! the arcade style (`initial_d`) the HUD's faint car stands bottom right, right of the
//! sensors' words, as faint as under a panel since words sit straight on the window
//! (`hud_app::paint::faint_car`).

mod calibration;

use std::time::Instant;

use hud_app::paint::{self, ToColor};
use hud_core::{
    health::Health,
    silhouette::Placement,
    theme::{Rgba, Theme, Tone},
};
use panel_core::{
    buttons::{Buttons, Press},
    calibration::{CalPage, CalPress, NotReady},
    gate::Gate,
    state::{PanelState, Row},
    steering_limits::Mark,
};
use raylib::prelude::*;

/// Fonts tried in order; raylib's own small pixel font if none is there.
const FONT_FILES: [&str; 4] = [
    "/System/Library/Fonts/Supplemental/Verdana Bold.ttf", // macOS
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",   // macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", // Ubuntu, Raspberry Pi OS
    "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf", // Fedora, AlmaLinux
];
const FONT_LOAD_SIZE: i32 = 96;
const MARGIN: f32 = 24.0;
/// Where the faint car's box starts, as a part of the window's width: right of the
/// sensors' longest line.
const CAR_FROM: f32 = 0.6;

/// A page of the panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    Drive,
    Calibration,
}

impl Page {
    fn label(self) -> &'static str {
        match self {
            Self::Drive => "DRIVE",
            Self::Calibration => "STEERING CAL",
        }
    }

    /// The other page (Tab).
    pub fn other(self) -> Self {
        match self {
            Self::Drive => Self::Calibration,
            Self::Calibration => Self::Drive,
        }
    }
}

/// An on-screen button.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    /// One of the buttons' presses.
    Press(Press),
    /// Held to drive.
    Go,
    Estop,
    Tab(Page),
    /// The calibration page's presses.
    Cal(CalPress),
    /// The calibration page: write the result to a file.
    Save,
}

impl Button {
    fn label(self) -> &'static str {
        match self {
            Self::Tab(p) => p.label(),
            Self::Cal(CalPress::Limp) => "GO LIMP",
            Self::Cal(CalPress::Hold) => "HOLD",
            Self::Cal(CalPress::StepLeft) => "< STEP LEFT",
            Self::Cal(CalPress::StepRight) => "STEP RIGHT >",
            Self::Cal(CalPress::End) => "END",
            Self::Cal(CalPress::Mark(Mark::Left)) => "MARK LEFT",
            Self::Cal(CalPress::Mark(Mark::Centre)) => "MARK CENTRE",
            Self::Cal(CalPress::Mark(Mark::Right)) => "MARK RIGHT",
            Self::Cal(CalPress::ClearMarks) => "CLEAR MARKS",
            Self::Cal(CalPress::Check) => "LOCK-TO-LOCK CHECK",
            Self::Save => "SAVE",
            Self::Press(Press::SpeedUp) => "SPEED +",
            Self::Press(Press::SpeedDown) => "SPEED -",
            Self::Press(Press::Left) => "< LEFT",
            Self::Press(Press::Right) => "RIGHT >",
            Self::Press(Press::Centre) => "CENTRE",
            Self::Press(Press::Stop) => "STOP",
            Self::Go => "HOLD SPACE (OR HERE) TO DRIVE",
            Self::Estop => "E-STOP",
        }
    }
}

/// Where each button is, in a window `w` wide: the same for drawing and for the mouse.
pub fn layout(w: f32, page: Page) -> Vec<(Button, Rectangle)> {
    let mut buttons = vec![(
        Button::Estop,
        Rectangle::new(w - MARGIN - 240.0, 14.0, 240.0, 84.0),
    )];
    let tab_w = 170.0;
    let tab_x = w - MARGIN - 240.0 - 16.0 - 2.0 * tab_w - 8.0;
    for (i, p) in [Page::Drive, Page::Calibration].into_iter().enumerate() {
        let x = tab_x + i as f32 * (tab_w + 8.0);
        buttons.push((Button::Tab(p), Rectangle::new(x, 58.0, tab_w, 36.0)));
    }
    match page {
        Page::Drive => buttons.extend(drive_layout(w)),
        Page::Calibration => buttons.extend(calibration::layout(w)),
    }
    buttons
}

/// The drive page's buttons.
fn drive_layout(w: f32) -> Vec<(Button, Rectangle)> {
    let col_w = (w - 2.0 * MARGIN - 2.0 * MARGIN) / 3.0;
    let x = |col: f32| MARGIN + col * (col_w + MARGIN);
    let half = (col_w - 12.0) / 2.0;
    let (row1, row2, bh) = (172.0, 252.0, 70.0);
    vec![
        (
            Button::Press(Press::SpeedUp),
            Rectangle::new(x(0.0), row1, half, bh),
        ),
        (
            Button::Press(Press::SpeedDown),
            Rectangle::new(x(0.0) + half + 12.0, row1, half, bh),
        ),
        (
            Button::Press(Press::Stop),
            Rectangle::new(x(1.0), row1, col_w, bh),
        ),
        (
            Button::Press(Press::Left),
            Rectangle::new(x(2.0), row1, half, bh),
        ),
        (
            Button::Press(Press::Right),
            Rectangle::new(x(2.0) + half + 12.0, row1, half, bh),
        ),
        (
            Button::Press(Press::Centre),
            Rectangle::new(x(2.0), row2, col_w, bh),
        ),
        (
            Button::Go,
            Rectangle::new(MARGIN, 400.0, w - 2.0 * MARGIN, 76.0),
        ),
    ]
}

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

/// What a frame needs besides the state.
pub struct Frame<'a> {
    pub now: Instant,
    pub label: &'a str,
    pub gate: &'a Gate,
    pub buttons: &'a Buttons,
    /// The on-screen button the mouse holds down, if any.
    pub held: Option<Button>,
    /// "front, not driven" on stands (TODO 49b).
    pub wheels_note: Option<&'a str>,
    pub stands: bool,
    pub page: Page,
    pub cal: &'a CalPage,
    /// Why the calibration page may not ask now (empty: allowed).
    pub cal_ready: &'a [NotReady],
    /// The calibration result's file, once saved.
    pub saved: Option<&'a str>,
    /// The little robot drives across the bottom lane: the keys line steps aside.
    pub hide_keys: bool,
}

/// The little robot's room (the easter egg): where nothing else is drawn this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Room {
    /// A band across the window under everything but the keys line: where it drives.
    pub lane: Option<Rectangle>,
    /// A box on the bottom edge, right of every word: where its eyes peek.
    pub corner: Option<Rectangle>,
}

/// The corner's height at most, and at least.
const CORNER_H: f32 = 130.0;
const CORNER_MIN_H: f32 = 48.0;
/// The lane's least height.
const LANE_MIN_H: f32 = 28.0;
/// Space kept round the robot.
const ROOM_GAP: f32 = 16.0;

impl Room {
    /// `content_end`: the lowest y anything but the keys line reaches. `keys_end`: the
    /// keys line's right end. `words`: other things near the bottom, as (top, bottom,
    /// right end).
    pub fn new(w: f32, h: f32, content_end: f32, keys_end: f32, words: &[(f32, f32, f32)]) -> Self {
        let lane_top = content_end + 4.0;
        let lane =
            (h - lane_top >= LANE_MIN_H).then(|| Rectangle::new(0.0, lane_top, w, h - lane_top));
        let keys = (h - 30.0, h, keys_end);
        let mut corner = None;
        let mut top = h - CORNER_H;
        while h - top >= CORNER_MIN_H {
            let x0 = words
                .iter()
                .chain([&keys])
                .filter(|(t, b, _)| *b > top && *t < h)
                .map(|&(_, _, right)| right)
                .fold(0.0, f32::max)
                + ROOM_GAP;
            let width = w - MARGIN - x0;
            let need = hud_core::robot::peek_width(f64::from(h - top)) as f32;
            if width >= need {
                corner = Some(Rectangle::new(x0, top, width, h - top));
                break;
            }
            top += 8.0;
        }
        Self { lane, corner }
    }
}

pub struct Ui {
    font: UiFont,
    theme: &'static Theme,
}

impl Ui {
    pub fn new(rl: &mut RaylibHandle, thread: &RaylibThread, theme: &'static Theme) -> Self {
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
        Self { font, theme }
    }

    /// Draws one frame; returns the little robot's room.
    pub fn draw(&self, d: &mut RaylibDrawHandle, state: &PanelState, f: &Frame<'_>) -> Room {
        let t = self.theme;
        let (w, h) = (d.get_screen_width() as f32, d.get_screen_height() as f32);
        // The window's soft gradient, as the HUD's (no glow: words sit straight on it).
        d.draw_rectangle_gradient_v(
            0,
            0,
            w.ceil() as i32,
            h.ceil() as i32,
            t.background.rl(),
            t.background_low.rl(),
        );
        // The arcade style's faint car: bottom right, right of the sensors' words,
        // above the keys line.
        let left = w * CAR_FROM;
        let place = Placement::bottom_right(left, 0.0, w - MARGIN - left, h - 40.0);
        paint::faint_car(d, t, place, 2.0, paint::Under::Words);
        let buttons = layout(w, f.page);
        let speed_on = matches!(f.gate, Gate::On { .. });

        // Title, the source, the label.
        self.text(d, "HARDWARE PANEL", MARGIN, 18.0, 32.0, t.text);
        let kind = state.kind();
        let mut tx = MARGIN + self.font.measure("HARDWARE PANEL", 32.0) + 20.0;
        let kind_word = if f.stands {
            format!("{kind} · STANDS")
        } else {
            kind.to_string()
        };
        let kind_tone = if kind.is_real() {
            Tone::Stop
        } else {
            Tone::NotReal
        };
        for (word, tone) in [(kind_word.as_str(), kind_tone), (f.label, Tone::Off)] {
            let p = t.pill(tone);
            let ww = self.font.measure(word, 20.0);
            d.draw_rectangle_rounded(
                Rectangle::new(tx, 22.0, ww + 20.0, 30.0),
                t.style.corner(0.3),
                6,
                p.fill.rl(),
            );
            self.text(d, word, tx + 10.0, 26.0, 20.0, p.ink);
            tx += ww + 32.0;
        }
        if let Some(banner) = state.origins.banner() {
            self.text(d, &banner, MARGIN, 62.0, 18.0, t.not_real_text);
        }

        for &(b, r) in buttons.iter().take(3) {
            self.button(d, b, r, f, true);
        }
        if f.page == Page::Calibration {
            return self.draw_calibration(d, state, f, &buttons[3..]);
        }

        // Where the speed buttons may drive.
        let gate_y = 104.0;
        match f.gate {
            Gate::On { cap, watch_unseen } => {
                let s = format!(
                    "SPEED BUTTONS ON: up to {cap:.1} m/s{}",
                    if *watch_unseen {
                        ", stopped if the speed isn't seen for 4 s"
                    } else {
                        ""
                    }
                );
                self.text(d, &s, MARGIN, gate_y, 22.0, t.good);
            }
            Gate::Off(why) => {
                self.text(d, "SPEED BUTTONS OFF:", MARGIN, gate_y, 22.0, t.warn);
                let x = MARGIN + self.font.measure("SPEED BUTTONS OFF:", 22.0) + 12.0;
                let lines = wrap(&self.font, &why.to_string(), w - x - MARGIN - 260.0, 16.0);
                for (i, l) in lines.iter().take(3).enumerate() {
                    self.text(d, l, x, gate_y - 14.0 + 19.0 * i as f32, 16.0, t.warn);
                }
            }
        }

        // Column captions.
        let col_w = (w - 4.0 * MARGIN) / 3.0;
        for (i, caption) in ["SPEED", "BRAKE", "STEERING"].into_iter().enumerate() {
            let x = MARGIN + i as f32 * (col_w + MARGIN);
            self.text(d, caption, x, 146.0, 17.0, t.caption);
        }

        for &(b, r) in &buttons[3..] {
            let enabled = match b {
                Button::Press(Press::Stop | Press::Centre) => true,
                Button::Go | Button::Press(_) => speed_on,
                _ => true,
            };
            self.button(d, b, r, f, enabled);
        }

        // Asked and seen, under each column.
        let b = f.buttons;
        let seen = |v: Option<f64>, unit: &str, k: f64| {
            v.map_or_else(|| "-".to_owned(), |v| format!("{:+.2} {unit}", v * k))
        };
        let pct = |row: Row| {
            let a = match row {
                Row::Brake => state.brake,
                _ => state.throttle,
            };
            a.and_then(|a| a.measured())
                .filter(|_| state.health(row, f.now).is_ok())
                .map_or_else(|| "-".to_owned(), |m| format!("{:.0} %", m * 100.0))
        };
        let x0 = MARGIN;
        let x1 = MARGIN + col_w + MARGIN;
        let x2 = MARGIN + 2.0 * (col_w + MARGIN);
        let ya = 256.0;
        self.text(
            d,
            &format!("asked {:.2} m/s", b.asked_speed()),
            x0,
            ya,
            22.0,
            t.text,
        );
        let s = format!("seen  {}", seen(state.seen_speed(f.now), "m/s", 1.0));
        self.text(d, &s, x0, ya + 30.0, 22.0, t.caption);
        if f.wheels_note.is_some() {
            self.text(
                d,
                "the wheel speed: front, not driven",
                x0,
                ya + 60.0,
                16.0,
                t.warn,
            );
        }
        self.text(
            d,
            &format!("brake seen {}", pct(Row::Brake)),
            x1,
            ya,
            22.0,
            t.caption,
        );
        let s = format!("throttle seen {}", pct(Row::Throttle));
        self.text(d, &s, x1, ya + 30.0, 22.0, t.caption);
        let note = "STOP: speed 0 at once. Brake %: later (option B)";
        let note = fit(&self.font, note, col_w, 15.0);
        self.text(d, &note, x1, ya + 62.0, 15.0, t.dim);
        let s = format!(
            "asked {:+.2} rad   seen {}",
            b.asked_steer(),
            seen(state.seen_steer(f.now), "rad", 1.0)
        );
        let s = fit(&self.font, &s, col_w, 19.0);
        self.text(d, &s, x2, ya + 72.0, 19.0, t.text);
        let note = "the wheels turn only while moving (ego)";
        let note = fit(&self.font, note, col_w, 15.0);
        self.text(d, &note, x2, ya + 100.0, 15.0, t.dim);

        // What the buttons are doing.
        let (line, colour) = self.status(state, f);
        let line = fit(&self.font, &line, w - 2.0 * MARGIN, 20.0);
        self.text(d, &line, MARGIN, 490.0, 20.0, colour);
        let mux = match state.mux {
            Some((at, c)) if f.now.saturating_duration_since(at).as_secs_f64() < 0.5 => format!(
                "twist_mux passes {:+.2} m/s, {:+.2} rad/s",
                c.speed, c.yaw_rate
            ),
            _ => "twist_mux passes nothing now".to_owned(),
        };
        self.text(d, &mux, MARGIN, 518.0, 16.0, t.dim);

        // The sensors.
        let mut y = 548.0;
        d.draw_rectangle_rec(Rectangle::new(0.0, y, w, 28.0), t.section.rl());
        self.text(
            d,
            "SENSORS, EACH FROM ITS OWN NODE",
            MARGIN,
            y + 5.0,
            17.0,
            t.caption,
        );
        y += 36.0;
        let row_h = ((h - y - 34.0) / Row::SENSORS.len() as f32).clamp(20.0, 30.0);
        let mut words = Vec::new();
        for row in Row::SENSORS {
            let health = state.health(row, f.now);
            self.text(d, row.label(), MARGIN, y, 19.0, t.text);
            let (word, tone) = health_word(health);
            let p = t.pill(tone);
            let pill = Rectangle::new(MARGIN + 110.0, y + 1.0, 80.0, 22.0);
            d.draw_rectangle_rounded(pill, t.style.corner(0.4), 6, p.fill.rl());
            let ww = self.font.measure(word, 15.0);
            let tx = pill.x + (pill.width - ww) / 2.0;
            self.text(d, word, tx, y + 4.0, 15.0, p.ink);
            let detail = match health {
                Health::Never => String::new(),
                _ => state.detail(row, f.now, f.wheels_note),
            };
            let detail = fit(&self.font, &detail, w - MARGIN - (MARGIN + 210.0), 18.0);
            let ink = if health.is_ok() { t.caption } else { t.dim };
            self.text(d, &detail, MARGIN + 210.0, y + 2.0, 18.0, ink);
            let right = MARGIN + 210.0 + self.font.measure(&detail, 18.0);
            words.push((y, y + row_h, right));
            y += row_h;
        }
        let keys = "keys: Space held = GO   Up/+ Down/- speed   Left Right steer   C centre   \
                    S stop   E e-stop   Tab steering calibration   F11 full screen";
        let keys = fit(&self.font, keys, w - 2.0 * MARGIN, 15.0);
        if !f.hide_keys {
            self.text(d, &keys, MARGIN, h - 26.0, 15.0, t.dim);
        }
        let keys_end = MARGIN + self.font.measure(&keys, 15.0);
        // The last row's words end 24 px under its top.
        let words_end = y - row_h + 24.0;
        Room::new(w, h, words_end, keys_end, &words)
    }

    /// One line: what the buttons are doing, or why not.
    fn status(&self, state: &PanelState, f: &Frame<'_>) -> (String, Rgba) {
        let (b, t) = (f.buttons, self.theme);
        if state.estop_on() {
            return (
                "E-STOP ON: nothing is sent. A person clears it (pixi run clear estop); then \
                 let GO go, and press Speed + again"
                    .to_owned(),
                t.bad,
            );
        }
        if b.needs_release() {
            let why = b
                .last_zero()
                .map_or_else(String::new, |z| format!(" (zeroed: {z})"));
            return (format!("let GO go, then press it again{why}"), t.warn);
        }
        if state.geofence_holds(f.now) && b.is_driving() {
            return (
                "held: the geofence's lock (51) is above the buttons (45)".to_owned(),
                t.warn,
            );
        }
        if b.is_centring() {
            return (
                "CENTRE: straightening the wheels at a creep".to_owned(),
                t.warn,
            );
        }
        if b.is_driving() {
            return (format!("DRIVING at {:.2} m/s", b.asked_speed()), t.good);
        }
        match b.last_zero() {
            Some(z) => (format!("zeroed: {z}"), t.warn),
            None => ("hold Space (or GO) to drive".to_owned(), t.caption),
        }
    }

    fn button(
        &self,
        d: &mut RaylibDrawHandle,
        b: Button,
        r: Rectangle,
        f: &Frame<'_>,
        enabled: bool,
    ) {
        let t = self.theme;
        let down = f.held == Some(b)
            || (b == Button::Go && f.buttons.is_driving())
            || b == Button::Tab(f.page);
        let (fill, ink) = match (b, enabled, down) {
            (Button::Estop, _, true) => (t.stop_fill_alt, t.stop_ink),
            (Button::Estop, _, false) => (t.stop_fill, t.stop_ink),
            (_, false, _) => (t.button_off, t.dim),
            (Button::Go, true, true) => (t.go_fill, t.go_ink),
            (_, true, true) => (t.button_down, t.text),
            (_, true, false) => (t.button, t.text),
        };
        d.draw_rectangle_rounded(r, t.style.corner(0.18), 8, fill.rl());
        if enabled && !down && b != Button::Estop {
            d.draw_rectangle_rounded_lines_ex(r, t.style.corner(0.18), 8, 1.0, t.glass_edge.rl());
        }
        let size = match b {
            Button::Estop => 34.0,
            Button::Tab(_) => 18.0,
            Button::Cal(_) | Button::Save => 21.0,
            _ => 26.0,
        };
        let label = b.label();
        let tw = self.font.measure(label, size);
        self.text(
            d,
            label,
            r.x + (r.width - tw) / 2.0,
            r.y + (r.height - size) / 2.0,
            size,
            ink,
        );
    }

    fn text(&self, d: &mut RaylibDrawHandle, s: &str, x: f32, y: f32, size: f32, colour: Rgba) {
        let at = Vector2::new(x, y);
        d.draw_text_ex(&self.font, s, at, size, size / 24.0, colour.rl());
    }
}

fn health_word(h: Health) -> (&'static str, Tone) {
    match h {
        Health::Ok(_) => ("OK", Tone::Go),
        Health::Stale(_) => ("STALE", Tone::Wait),
        Health::Never => ("NEVER", Tone::Idle),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_buttons_never_overlap() {
        for page in [Page::Drive, Page::Calibration] {
            for w in [1100.0, 1280.0, 1920.0] {
                let buttons = layout(w, page);
                for (i, (a, ra)) in buttons.iter().enumerate() {
                    assert!(ra.x >= 0.0 && ra.x + ra.width <= w, "{a:?} at {w}");
                    for (b, rb) in &buttons[i + 1..] {
                        assert!(
                            ra.get_collision_rec(*rb).is_none(),
                            "{a:?} and {b:?} overlap at {w}"
                        );
                    }
                }
            }
        }
        assert_eq!(layout(1280.0, Page::Drive).len(), 10);
    }

    #[test]
    fn the_robots_room_keeps_clear_of_the_words() {
        // Rows of words reaching x = 700 down to y = 824; the keys line to 800.
        let rows: Vec<(f32, f32, f32)> = (0..8)
            .map(|i| {
                let y = 584.0 + 30.0 * i as f32;
                (y, y + 30.0, 700.0)
            })
            .collect();
        let room = Room::new(1280.0, 860.0, 824.0, 800.0, &rows);
        let corner = room.corner.expect("room right of the words");
        assert!(corner.x >= 800.0 + ROOM_GAP);
        assert!(corner.x + corner.width <= 1280.0 - MARGIN);
        assert!(
            (corner.y + corner.height - 860.0).abs() < 1e-3,
            "on the bottom edge"
        );
        assert!(f64::from(corner.width) >= hud_core::robot::peek_width(f64::from(corner.height)));
        let lane = room.lane.expect("a lane under the rows");
        assert!(lane.y >= 824.0);
        // Words right across: no corner; no space under them: no lane.
        let wide = [(700.0, 860.0, 1280.0)];
        let room = Room::new(1280.0, 860.0, 850.0, 800.0, &wide);
        assert_eq!(room.corner, None);
        assert_eq!(room.lane, None);
        assert_eq!(layout(1280.0, Page::Calibration).len(), 3 + 11);
    }
}

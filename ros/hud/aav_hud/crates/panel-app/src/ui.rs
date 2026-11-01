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

use std::time::Instant;

use hud_core::health::Health;
use panel_core::{
    buttons::{Buttons, Press},
    gate::Gate,
    state::{PanelState, Row},
};
use raylib::prelude::*;

const BACKGROUND: Color = Color::new(16, 18, 24, 255);
const SECTION: Color = Color::new(31, 35, 46, 255);
const BUTTON: Color = Color::new(52, 60, 78, 255);
const BUTTON_DOWN: Color = Color::new(90, 104, 134, 255);
const BUTTON_OFF: Color = Color::new(34, 38, 48, 255);
const CAPTION: Color = Color::new(140, 150, 170, 255);
const TEXT: Color = Color::new(240, 242, 246, 255);
const DIM: Color = Color::new(105, 110, 125, 255);
const GOOD: Color = Color::new(60, 200, 90, 255);
const WARN: Color = Color::new(245, 180, 40, 255);
const BAD: Color = Color::new(230, 60, 60, 255);
const NOT_REAL: Color = Color::new(255, 140, 0, 255);

/// Fonts tried in order; raylib's own small pixel font if none is there.
const FONT_FILES: [&str; 4] = [
    "/System/Library/Fonts/Supplemental/Verdana Bold.ttf", // macOS
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",   // macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", // Ubuntu, Raspberry Pi OS
    "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf", // Fedora, AlmaLinux
];
const FONT_LOAD_SIZE: i32 = 96;
const MARGIN: f32 = 24.0;

/// An on-screen button.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    /// One of the buttons' presses.
    Press(Press),
    /// Held to drive.
    Go,
    Estop,
}

impl Button {
    fn label(self) -> &'static str {
        match self {
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

/// Where each button is, in a window `w` x `h`: the same for drawing and for the mouse.
pub fn layout(w: f32) -> Vec<(Button, Rectangle)> {
    let col_w = (w - 2.0 * MARGIN - 2.0 * MARGIN) / 3.0;
    let x = |col: f32| MARGIN + col * (col_w + MARGIN);
    let half = (col_w - 12.0) / 2.0;
    let (row1, row2, bh) = (172.0, 252.0, 70.0);
    vec![
        (
            Button::Estop,
            Rectangle::new(w - MARGIN - 240.0, 14.0, 240.0, 84.0),
        ),
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
}

pub struct Ui {
    font: UiFont,
}

impl Ui {
    pub fn new(rl: &mut RaylibHandle, thread: &RaylibThread) -> Self {
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
        Self { font }
    }

    pub fn draw(&self, d: &mut RaylibDrawHandle, state: &PanelState, f: &Frame<'_>) {
        d.clear_background(BACKGROUND);
        let (w, h) = (d.get_screen_width() as f32, d.get_screen_height() as f32);
        let buttons = layout(w);
        let speed_on = matches!(f.gate, Gate::On { .. });

        // Title, the source, the label.
        self.text(d, "HARDWARE PANEL", MARGIN, 18.0, 32.0, TEXT);
        let kind = state.kind();
        let mut tx = MARGIN + self.font.measure("HARDWARE PANEL", 32.0) + 20.0;
        let kind_word = if f.stands {
            format!("{kind} · STANDS")
        } else {
            kind.to_string()
        };
        for (word, colour) in [
            (
                kind_word.as_str(),
                if kind.is_real() { BAD } else { NOT_REAL },
            ),
            (f.label, SECTION),
        ] {
            let ww = self.font.measure(word, 20.0);
            d.draw_rectangle_rounded(Rectangle::new(tx, 22.0, ww + 20.0, 30.0), 0.3, 6, colour);
            let ink = if colour == SECTION {
                CAPTION
            } else {
                Color::BLACK
            };
            self.text(d, word, tx + 10.0, 26.0, 20.0, ink);
            tx += ww + 32.0;
        }
        if let Some(banner) = state.origins.banner() {
            self.text(d, &banner, MARGIN, 62.0, 18.0, NOT_REAL);
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
                self.text(d, &s, MARGIN, gate_y, 22.0, GOOD);
            }
            Gate::Off(why) => {
                self.text(d, "SPEED BUTTONS OFF:", MARGIN, gate_y, 22.0, WARN);
                let x = MARGIN + self.font.measure("SPEED BUTTONS OFF:", 22.0) + 12.0;
                let lines = wrap(&self.font, &why.to_string(), w - x - MARGIN - 260.0, 16.0);
                for (i, l) in lines.iter().take(3).enumerate() {
                    self.text(d, l, x, gate_y - 14.0 + 19.0 * i as f32, 16.0, WARN);
                }
            }
        }

        // Column captions.
        let col_w = (w - 4.0 * MARGIN) / 3.0;
        for (i, caption) in ["SPEED", "BRAKE", "STEERING"].into_iter().enumerate() {
            let x = MARGIN + i as f32 * (col_w + MARGIN);
            self.text(d, caption, x, 146.0, 17.0, CAPTION);
        }

        for &(b, r) in &buttons {
            let enabled = match b {
                Button::Estop | Button::Press(Press::Stop | Press::Centre) => true,
                Button::Go | Button::Press(_) => speed_on,
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
            TEXT,
        );
        let s = format!("seen  {}", seen(state.seen_speed(f.now), "m/s", 1.0));
        self.text(d, &s, x0, ya + 30.0, 22.0, CAPTION);
        if f.wheels_note.is_some() {
            self.text(
                d,
                "the wheel speed: front, not driven",
                x0,
                ya + 60.0,
                16.0,
                WARN,
            );
        }
        self.text(
            d,
            &format!("brake seen {}", pct(Row::Brake)),
            x1,
            ya,
            22.0,
            CAPTION,
        );
        let s = format!("throttle seen {}", pct(Row::Throttle));
        self.text(d, &s, x1, ya + 30.0, 22.0, CAPTION);
        let note = "STOP: speed 0 at once. Brake %: later (option B)";
        let note = fit(&self.font, note, col_w, 15.0);
        self.text(d, &note, x1, ya + 62.0, 15.0, DIM);
        let s = format!(
            "asked {:+.2} rad   seen {}",
            b.asked_steer(),
            seen(state.seen_steer(f.now), "rad", 1.0)
        );
        let s = fit(&self.font, &s, col_w, 19.0);
        self.text(d, &s, x2, ya + 72.0, 19.0, TEXT);
        let note = "the wheels turn only while moving (ego)";
        let note = fit(&self.font, note, col_w, 15.0);
        self.text(d, &note, x2, ya + 100.0, 15.0, DIM);

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
        self.text(d, &mux, MARGIN, 518.0, 16.0, DIM);

        // The sensors.
        let mut y = 548.0;
        d.draw_rectangle_rec(Rectangle::new(0.0, y, w, 28.0), SECTION);
        self.text(
            d,
            "SENSORS, EACH FROM ITS OWN NODE",
            MARGIN,
            y + 5.0,
            17.0,
            CAPTION,
        );
        y += 36.0;
        let row_h = ((h - y - 34.0) / Row::SENSORS.len() as f32).clamp(20.0, 30.0);
        for row in Row::SENSORS {
            let health = state.health(row, f.now);
            self.text(d, row.label(), MARGIN, y, 19.0, TEXT);
            let (word, colour) = health_word(health);
            let pill = Rectangle::new(MARGIN + 110.0, y + 1.0, 80.0, 22.0);
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
            let detail = match health {
                Health::Never => String::new(),
                _ => state.detail(row, f.now, f.wheels_note),
            };
            let detail = fit(&self.font, &detail, w - MARGIN - (MARGIN + 210.0), 18.0);
            let ink = if health.is_ok() { CAPTION } else { DIM };
            self.text(d, &detail, MARGIN + 210.0, y + 2.0, 18.0, ink);
            y += row_h;
        }
        let keys = "keys: Space held = GO   Up/+ Down/- speed   Left Right steer   C centre   \
                    S stop   E e-stop   F11 full screen";
        let keys = fit(&self.font, keys, w - 2.0 * MARGIN, 15.0);
        self.text(d, &keys, MARGIN, h - 26.0, 15.0, DIM);
    }

    /// One line: what the buttons are doing, or why not.
    fn status(&self, state: &PanelState, f: &Frame<'_>) -> (String, Color) {
        let b = f.buttons;
        if state.estop_on() {
            return (
                "E-STOP ON: nothing is sent. A person clears it (pixi run estop-clear); then \
                 let GO go, and press Speed + again"
                    .to_owned(),
                BAD,
            );
        }
        if b.needs_release() {
            let why = b
                .last_zero()
                .map_or_else(String::new, |z| format!(" (zeroed: {z})"));
            return (format!("let GO go, then press it again{why}"), WARN);
        }
        if state.geofence_holds(f.now) && b.is_driving() {
            return (
                "held: the geofence's lock (51) is above the buttons (45)".to_owned(),
                WARN,
            );
        }
        if b.is_centring() {
            return (
                "CENTRE: straightening the wheels at a creep".to_owned(),
                WARN,
            );
        }
        if b.is_driving() {
            return (format!("DRIVING at {:.2} m/s", b.asked_speed()), GOOD);
        }
        match b.last_zero() {
            Some(z) => (format!("zeroed: {z}"), WARN),
            None => ("hold Space (or GO) to drive".to_owned(), CAPTION),
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
        let down = f.held == Some(b) || (b == Button::Go && f.buttons.is_driving());
        let colour = match (b, enabled, down) {
            (Button::Estop, _, _) => BAD,
            (_, false, _) => BUTTON_OFF,
            (Button::Go, true, true) => GOOD,
            (_, true, true) => BUTTON_DOWN,
            (_, true, false) => BUTTON,
        };
        d.draw_rectangle_rounded(r, 0.18, 8, colour);
        let size = if b == Button::Estop { 34.0 } else { 26.0 };
        let label = b.label();
        let tw = self.font.measure(label, size);
        let ink = if b == Button::Estop || (b == Button::Go && enabled && down) {
            Color::BLACK
        } else if enabled {
            TEXT
        } else {
            DIM
        };
        self.text(
            d,
            label,
            r.x + (r.width - tw) / 2.0,
            r.y + (r.height - size) / 2.0,
            size,
            ink,
        );
    }

    fn text(&self, d: &mut RaylibDrawHandle, s: &str, x: f32, y: f32, size: f32, colour: Color) {
        d.draw_text_ex(&self.font, s, Vector2::new(x, y), size, size / 24.0, colour);
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_buttons_never_overlap() {
        let buttons = layout(1280.0);
        for (i, (a, ra)) in buttons.iter().enumerate() {
            for (b, rb) in &buttons[i + 1..] {
                assert!(
                    ra.get_collision_rec(*rb).is_none(),
                    "{a:?} and {b:?} overlap"
                );
            }
        }
        assert_eq!(buttons.len(), 8);
    }
}

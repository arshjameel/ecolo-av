//! The MEASURE page (TODO 71): speed and distance since RESET, a tape run typed in, and
//! the wheel diameter's check (docs/know-the-car.md, test 3). The logic is panel-core's
//! `measure`; this only draws it.
//!
//! ```text
//! | SPEED 0.98 m/s (OK)      DISTANCE 10.31 m     TOP 1.02 m/s   MEAN 0.97 m/s  [RESET] |
//! | TAPE: 10.0_ m  (type the tape's metres; Backspace)                         [SAVE]  |
//! | THE CHECK: wheels read +3.1 % long: CHANGE wheel_diameter 0.360 -> 0.349           |
//! | how: RESET at the tape's start, drive straight past its end, stop, type its metres |
//! ```

use hud_app::paint::ToColor;
use hud_core::theme::Tone;
use panel_core::{
    measure::{wheel_check, WheelCheck, WITHIN},
    state::{PanelState, Speed},
};
use raylib::prelude::*;

use super::{fit, Button, Frame, Room, Ui, MARGIN};

const TOP: f32 = 104.0;
const BUTTON_W: f32 = 200.0;

/// The page's two buttons: RESET and SAVE.
pub fn layout(w: f32) -> Vec<(Button, Rectangle)> {
    vec![
        (
            Button::Reset,
            Rectangle::new(w - MARGIN - BUTTON_W, TOP + 40.0, BUTTON_W, 44.0),
        ),
        (
            Button::Save,
            Rectangle::new(w - MARGIN - BUTTON_W, TOP + 150.0, BUTTON_W, 44.0),
        ),
    ]
}

impl Ui {
    pub(super) fn draw_measure(
        &self,
        d: &mut RaylibDrawHandle,
        state: &PanelState,
        f: &Frame<'_>,
        buttons: &[(Button, Rectangle)],
    ) -> Room {
        let t = self.theme;
        let (w, h) = (d.get_screen_width() as f32, d.get_screen_height() as f32);
        let text_w = w - 2.0 * MARGIN - BUTTON_W - 24.0;
        let o = f.odometer;

        // Speed and distance.
        self.text(
            d,
            "SPEED AND DISTANCE, SINCE RESET",
            MARGIN,
            TOP + 8.0,
            20.0,
            t.caption,
        );
        let (speed, ink) = match state.speed(f.now) {
            Speed::Measured(v) => (format!("{v:.2} m/s"), t.text),
            Speed::Unknown(why) => (format!("unknown: {why}"), t.warn),
        };
        let mean = o.mean().map_or("-".to_owned(), |m| format!("{m:.2} m/s"));
        let row = [
            ("SPEED", speed, ink),
            ("DISTANCE", format!("{:.2} m", o.distance), t.text),
            ("TOP", format!("{:.2} m/s", o.top), t.text),
            ("MEAN", mean, t.text),
        ];
        let col = text_w / 4.0;
        for (i, (label, value, ink)) in row.iter().enumerate() {
            let x = MARGIN + i as f32 * col;
            self.text(d, label, x, TOP + 40.0, 16.0, t.dim);
            let value = fit(&self.font, value, col - 12.0, 30.0);
            self.text(d, &value, x, TOP + 60.0, 30.0, *ink);
        }
        let gaps = if o.gaps == 0 {
            ("no gaps in the speed".to_owned(), t.dim)
        } else {
            (
                format!(
                    "{} gaps in the speed (over 0.5 s, or unknown): not counted",
                    o.gaps
                ),
                t.warn,
            )
        };
        self.text(d, &gaps.0, MARGIN, TOP + 100.0, 16.0, gaps.1);

        // The tape, typed in.
        self.text(
            d,
            "THE TAPE, TYPED IN",
            MARGIN,
            TOP + 140.0,
            20.0,
            t.caption,
        );
        let typed = format!("{}_ m", f.tape.text());
        self.text(d, &typed, MARGIN, TOP + 168.0, 34.0, t.text);
        let hint = "digits and a point; Backspace";
        self.text(d, hint, MARGIN + 260.0, TOP + 180.0, 16.0, t.dim);

        // The check.
        self.text(
            d,
            "THE WHEEL DIAMETER'S CHECK",
            MARGIN,
            TOP + 230.0,
            20.0,
            t.caption,
        );
        let now_d = f
            .wheel_diameter
            .map_or("unset".to_owned(), |d| format!("{d:.3} m"));
        let (verdict, tone) = match wheel_check(o, f.tape.value(), f.wheel_diameter) {
            WheelCheck::NotYet(why) => (format!("not yet: {why}"), Tone::Idle),
            WheelCheck::Done {
                error,
                suggested,
                pass: true,
            } => (
                format!(
                    "PASS: the wheels read {:+.2} % of the tape (within {:.0} %); \
                     wheel_diameter {now_d} stays (exact: {suggested:.3})",
                    error * 100.0,
                    WITHIN * 100.0
                ),
                Tone::Go,
            ),
            WheelCheck::Done {
                error,
                suggested,
                pass: false,
            } => (
                format!(
                    "CHANGE: the wheels read {:+.2} % of the tape; wheel_diameter \
                     {now_d} -> {suggested:.3} m (a person copies it into vehicle.yaml)",
                    error * 100.0
                ),
                Tone::Stop,
            ),
        };
        let p = t.pill(tone);
        let lines = super::wrap(&self.font, &verdict, text_w - 20.0, 22.0);
        let box_h = 16.0 + lines.len() as f32 * 28.0;
        d.draw_rectangle_rounded(
            Rectangle::new(MARGIN, TOP + 260.0, text_w, box_h),
            t.style.corner(0.15),
            6,
            p.fill.rl(),
        );
        for (i, line) in lines.iter().enumerate() {
            self.text(
                d,
                line,
                MARGIN + 10.0,
                TOP + 268.0 + i as f32 * 28.0,
                22.0,
                p.ink,
            );
        }

        // How, and the saved file.
        let how = [
            "HOW (know-the-car, test 3):",
            "1. Lay a tape straight, 10 m if there is room. Stop the car at its start.",
            "2. RESET. Drive straight past the tape's end at a steady speed (the joystick, or DRIVE).",
            "3. Stop exactly at the end. Type the tape's metres. Read the check.",
            "4. SAVE writes it all to a file; nothing here writes vehicle.yaml.",
        ];
        let how_y = TOP + 280.0 + box_h;
        for (i, line) in how.iter().enumerate() {
            let ink = if i == 0 { t.caption } else { t.dim };
            let line = fit(&self.font, line, text_w, 17.0);
            self.text(d, &line, MARGIN, how_y + i as f32 * 24.0, 17.0, ink);
        }
        if let Some(saved) = f.measure_saved {
            let line = fit(&self.font, &format!("saved: {saved}"), text_w, 17.0);
            self.text(d, &line, MARGIN, how_y + 130.0, 17.0, t.good);
        }

        for &(b, r) in buttons {
            self.button(d, b, r, f, true);
        }
        let keys = "keys: digits . Backspace the tape   R reset   W save   Tab next page   \
                    E e-stop   F11 full screen";
        let keys = fit(&self.font, keys, w - 2.0 * MARGIN, 15.0);
        if !f.hide_keys {
            self.text(d, &keys, MARGIN, h - 26.0, 15.0, t.dim);
        }
        Room {
            lane: None,
            corner: None,
        }
    }
}

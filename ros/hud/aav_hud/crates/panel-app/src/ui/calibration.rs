//! The steering calibration page (docs/actuator-tests.md, way B).
//!
//! ```text
//! | ALLOWED (or NOT ALLOWED: why)                                    |
//! | bridge: LIMP - the steering actuator's motor is off ...          |
//! | 1985 counts      firmware JOG    page: holding at 1990           |
//! | [====|=====L=====C=====R=====|====]  the counts, left on the left|
//! | [GO LIMP][HOLD][< STEP LEFT][STEP RIGHT >][END]                  |
//! | [MARK LEFT][MARK CENTRE][MARK RIGHT][CLEAR MARKS][SAVE]          |
//! | [               LOCK-TO-LOCK CHECK                ]              |
//! | the last thing that happened                                     |
//! | MARKS AND LIMITS, the lines to copy | LOCK-TO-LOCK CHECK         |
//! ```

use hud_app::paint::ToColor;
use hud_core::theme::Rgba;
use panel_core::{
    calibration::{CalPage, CalPress, Mode},
    state::PanelState,
    steering_limits::{self, Mark, COUNTS_MAX},
    sweep::Outcome,
};
use raylib::prelude::*;

use super::{fit, wrap, Button, Frame, Room, Ui, MARGIN};

const ROW1: f32 = 262.0;
const ROW_H: f32 = 62.0;
const GAP: f32 = 12.0;
const BAR_Y: f32 = 214.0;
const BAR_H: f32 = 24.0;
const NOTE_Y: f32 = 476.0;
const SECTION_Y: f32 = 508.0;

/// The page's buttons, under the header's.
pub fn layout(w: f32) -> Vec<(Button, Rectangle)> {
    let cw = w - 2.0 * MARGIN;
    let col = (cw - 4.0 * GAP) / 5.0;
    let at = |i: usize, row: f32| Rectangle::new(MARGIN + i as f32 * (col + GAP), row, col, ROW_H);
    let row2 = ROW1 + ROW_H + 10.0;
    let row3 = row2 + ROW_H + 10.0;
    let first = [
        CalPress::Limp,
        CalPress::Hold,
        CalPress::StepLeft,
        CalPress::StepRight,
        CalPress::End,
    ];
    let second = [
        Button::Cal(CalPress::Mark(Mark::Left)),
        Button::Cal(CalPress::Mark(Mark::Centre)),
        Button::Cal(CalPress::Mark(Mark::Right)),
        Button::Cal(CalPress::ClearMarks),
        Button::Save,
    ];
    let mut out: Vec<(Button, Rectangle)> = first
        .into_iter()
        .enumerate()
        .map(|(i, p)| (Button::Cal(p), at(i, ROW1)))
        .collect();
    out.extend(
        second
            .into_iter()
            .enumerate()
            .map(|(i, b)| (b, at(i, row2))),
    );
    out.push((
        Button::Cal(CalPress::Check),
        Rectangle::new(MARGIN, row3, cw, 56.0),
    ));
    out
}

/// What the page is doing, in a few words.
fn mode_words(cal: &CalPage) -> String {
    match cal.mode() {
        Mode::Off => "not asking: normal steering".to_owned(),
        Mode::Limp => "asking LIMP: turn the wheels by hand".to_owned(),
        Mode::Jog { target } => format!("asking JOG to {target} counts"),
        Mode::Sweep(s) => format!("CHECK: {} (target {})", s.leg(), s.target()),
        Mode::Ending { .. } => "ending: the wheels go back slowly".to_owned(),
    }
}

impl Ui {
    pub(super) fn draw_calibration(
        &self,
        d: &mut RaylibDrawHandle,
        state: &PanelState,
        f: &Frame<'_>,
        buttons: &[(Button, Rectangle)],
    ) -> Room {
        let t = self.theme;
        let w = d.get_screen_width() as f32;
        let h = d.get_screen_height() as f32;
        let cal = f.cal;
        let ready = f.cal_ready.is_empty();
        let raw = state.fresh_raw(f.now);
        let position = raw.and_then(|r| r.position);

        // Allowed, or why not.
        if ready {
            let s = "CALIBRATION ALLOWED: standing still, no e-stop, the position seen";
            self.text(d, s, MARGIN, 104.0, 21.0, t.good);
        } else {
            let why: Vec<String> = f.cal_ready.iter().map(ToString::to_string).collect();
            let s = format!("NOT ALLOWED: {}", why.join("; "));
            let s = fit(&self.font, &s, w - 2.0 * MARGIN, 21.0);
            self.text(d, &s, MARGIN, 104.0, 21.0, t.warn);
        }
        let bridge = match &state.calibration_line {
            Some((at, line)) if f.now.saturating_duration_since(*at).as_secs_f64() < 3.0 => {
                format!("bridge: {line}")
            }
            _ => "bridge: nothing heard (it speaks while asked, with uplink true)".to_owned(),
        };
        let bridge = fit(&self.font, &bridge, w - 2.0 * MARGIN, 16.0);
        let ink = if bridge.contains("REFUSED") {
            t.warn
        } else {
            t.caption
        };
        self.text(d, &bridge, MARGIN, 132.0, 16.0, ink);

        // The position, big; the firmware's state and what the page asks.
        let big = position.map_or_else(|| "- counts".to_owned(), |p| format!("{p} counts"));
        self.text(d, &big, MARGIN, 156.0, 38.0, t.text);
        let x = MARGIN + self.font.measure("0000 counts", 38.0) + 30.0;
        let firmware = raw.map_or_else(
            || "firmware: -".to_owned(),
            |r| format!("firmware: {}", r.state),
        );
        let firmware = fit(&self.font, &firmware, w - x - MARGIN, 19.0);
        self.text(d, &firmware, x, 156.0, 19.0, t.caption);
        let asks = fit(&self.font, &mode_words(cal), w - x - MARGIN, 19.0);
        let ink = if cal.is_asking() { t.warn } else { t.caption };
        self.text(d, &asks, x, 180.0, 19.0, ink);

        self.bar(d, cal, position, w);

        for &(b, r) in buttons {
            let enabled = match b {
                Button::Cal(CalPress::Mark(_)) => position.is_some(),
                Button::Cal(CalPress::ClearMarks) => any_mark(cal),
                Button::Cal(CalPress::End) => !matches!(cal.mode(), Mode::Off),
                Button::Cal(CalPress::Check) => {
                    ready
                        && matches!(cal.limits(), Some(Ok(_)))
                        && !matches!(cal.mode(), Mode::Sweep(_))
                }
                Button::Save => matches!(cal.limits(), Some(Ok(_))),
                Button::Cal(_) => ready,
                _ => true,
            };
            self.button(d, b, r, f, enabled);
        }

        // The check's result has its own column below: don't say it twice.
        let checked = cal.checked().map(|o| o.to_string());
        if let Some(note) = cal.note().filter(|n| checked.as_deref() != Some(*n)) {
            let ink = if note.starts_with("PASSED") || note.starts_with("marked") {
                t.good
            } else if note.starts_with("STOPPED")
                || note.starts_with("not now")
                || note.starts_with("can't")
                || note.starts_with("stopped")
            {
                t.warn
            } else {
                t.caption
            };
            let note = fit(&self.font, note, w - 2.0 * MARGIN, 20.0);
            self.text(d, &note, MARGIN, NOTE_Y, 20.0, ink);
        }

        // Two columns: the marks and limits; the check.
        let col_w = (w - 3.0 * MARGIN) / 2.0;
        let right_x = 2.0 * MARGIN + col_w;
        d.draw_rectangle_rec(Rectangle::new(0.0, SECTION_Y, w, 28.0), t.section.rl());
        self.text(
            d,
            "MARKS AND LIMITS",
            MARGIN,
            SECTION_Y + 5.0,
            17.0,
            t.caption,
        );
        self.text(
            d,
            "LOCK-TO-LOCK CHECK",
            right_x,
            SECTION_Y + 5.0,
            17.0,
            t.caption,
        );
        let left_end =
            self.limits_column(d, cal, MARGIN, SECTION_Y + 38.0, col_w, h - 34.0, f.saved);
        let right_end = self.check_column(d, cal, position, right_x, SECTION_Y + 38.0, col_w);

        let keys = "keys: L limp   H hold   Left Right step   1 2 3 mark left centre right   \
                    K check   W save   O end   Tab drive page   E e-stop";
        let keys = fit(&self.font, keys, w - 2.0 * MARGIN, 15.0);
        if !f.hide_keys {
            self.text(d, &keys, MARGIN, h - 26.0, 15.0, t.dim);
        }
        let keys_end = MARGIN + self.font.measure(&keys, 15.0);
        let words = [
            (SECTION_Y, left_end, MARGIN + col_w),
            (SECTION_Y, right_end, w - MARGIN),
        ];
        Room::new(w, h, left_end.max(right_end), keys_end, &words)
    }

    /// The counts as a bar, with the steering's left on the screen's left: the jog
    /// range, the limits, the marks, the target and the position.
    fn bar(&self, d: &mut RaylibDrawHandle, cal: &CalPage, position: Option<u16>, w: f32) {
        let t = self.theme;
        let (x0, width) = (MARGIN, w - 2.0 * MARGIN);
        let left_up = cal.left_sign() > 0;
        let at = |c: u16| {
            let k = f32::from(c) / f32::from(COUNTS_MAX);
            x0 + width * if left_up { 1.0 - k } else { k }
        };
        let span = |a: u16, b: u16| {
            let (p, q) = (at(a), at(b));
            Rectangle::new(p.min(q), BAR_Y, (p - q).abs(), BAR_H)
        };
        d.draw_rectangle_rounded(
            Rectangle::new(x0, BAR_Y, width, BAR_H),
            t.style.corner(0.4),
            6,
            t.track.rl(),
        );
        let s = cal.settings();
        d.draw_rectangle_rec(span(s.jog_min, s.jog_max), t.button.rl());
        if let Some(Ok(l)) = cal.limits() {
            d.draw_rectangle_rec(span(l.lo, l.hi), t.go_fill.rl());
        }
        self.text(d, "LEFT", x0, BAR_Y - 20.0, 15.0, t.dim);
        let rw = self.font.measure("RIGHT", 15.0);
        self.text(d, "RIGHT", x0 + width - rw, BAR_Y - 20.0, 15.0, t.dim);
        for (m, word) in [(Mark::Left, "L"), (Mark::Centre, "C"), (Mark::Right, "R")] {
            if let Some(c) = cal.mark(m) {
                let x = at(c);
                d.draw_line_ex(
                    Vector2::new(x, BAR_Y - 4.0),
                    Vector2::new(x, BAR_Y + BAR_H + 4.0),
                    3.0,
                    t.accent.rl(),
                );
                self.text(d, word, x - 5.0, BAR_Y + BAR_H + 4.0, 16.0, t.accent);
            }
        }
        let target = match cal.mode() {
            Mode::Jog { target } => Some(*target),
            Mode::Sweep(s) => Some(s.target()),
            _ => None,
        };
        if let Some(c) = target {
            let x = at(c);
            d.draw_rectangle_lines_ex(
                Rectangle::new(x - 6.0, BAR_Y - 6.0, 12.0, BAR_H + 12.0),
                2.0,
                t.warn.rl(),
            );
        }
        if let Some(c) = position {
            let x = at(c);
            d.draw_triangle(
                Vector2::new(x, BAR_Y + 4.0),
                Vector2::new(x - 9.0, BAR_Y - 12.0),
                Vector2::new(x + 9.0, BAR_Y - 12.0),
                t.text.rl(),
            );
            d.draw_line_ex(
                Vector2::new(x, BAR_Y),
                Vector2::new(x, BAR_Y + BAR_H),
                3.0,
                t.text.rl(),
            );
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn limits_column(
        &self,
        d: &mut RaylibDrawHandle,
        cal: &CalPage,
        x: f32,
        mut y: f32,
        width: f32,
        bottom: f32,
        saved: Option<&str>,
    ) -> f32 {
        let t = self.theme;
        let mark = |m: Mark| {
            cal.mark(m)
                .map_or_else(|| "-".to_owned(), |c| c.to_string())
        };
        let s = format!(
            "left {}   centre {}   right {}",
            mark(Mark::Left),
            mark(Mark::Centre),
            mark(Mark::Right)
        );
        self.text(d, &s, x, y, 20.0, t.text);
        y += 28.0;
        let line = |d: &mut RaylibDrawHandle, s: &str, size: f32, ink: Rgba, y: &mut f32| {
            for l in wrap(&self.font, s, width, size) {
                if *y + size > bottom {
                    return;
                }
                self.text(d, &l, x, *y, size, ink);
                *y += size + 4.0;
            }
        };
        match cal.limits() {
            None => line(
                d,
                "mark full left, straight and full right: limp and turn them by hand, or step",
                17.0,
                t.dim,
                &mut y,
            ),
            Some(Err(e)) => line(d, &format!("no limits: {e}"), 17.0, t.bad, &mut y),
            Some(Ok(l)) => {
                let s = format!(
                    "limits {} to {} ({} each side), steering left {} the count",
                    l.lo,
                    l.hi,
                    l.half,
                    if l.sign > 0 { "raises" } else { "lowers" }
                );
                line(d, &s, 18.0, t.good, &mut y);
                for w in &l.warnings {
                    line(d, &format!("WARNING: {w}"), 15.0, t.warn, &mut y);
                }
                y += 4.0;
                line(
                    d,
                    "To copy by hand, in a pull request (the hardware lead reviews):",
                    15.0,
                    t.caption,
                    &mut y,
                );
                let lines = steering_limits::firmware_steering(&l)
                    .replace("\n    ", " ")
                    .replace('\n', " ");
                line(d, &format!("calibration.rs: {lines}"), 15.0, t.text, &mut y);
                let yaml = steering_limits::vehicle_yaml(&l);
                line(d, &format!("vehicle.yaml: {yaml}"), 15.0, t.text, &mut y);
                let file = saved.map_or_else(
                    || "SAVE writes it all to ~/.local/share/aav/steering-calibration/".to_owned(),
                    |p| format!("saved: {p}"),
                );
                line(
                    d,
                    &file,
                    15.0,
                    if saved.is_some() { t.good } else { t.dim },
                    &mut y,
                );
            }
        }
        y
    }

    fn check_column(
        &self,
        d: &mut RaylibDrawHandle,
        cal: &CalPage,
        position: Option<u16>,
        x: f32,
        mut y: f32,
        width: f32,
    ) -> f32 {
        let t = self.theme;
        let mut line = |d: &mut RaylibDrawHandle, s: &str, size: f32, ink: Rgba| {
            for l in wrap(&self.font, s, width, size) {
                self.text(d, &l, x, y, size, ink);
                y += size + 4.0;
            }
        };
        if let Mode::Sweep(s) = cal.mode() {
            line(d, &format!("RUNNING: {}", s.leg()), 20.0, t.warn);
            let error =
                position.map_or_else(|| "-".to_owned(), |p| p.abs_diff(s.target()).to_string());
            let p = position.map_or_else(|| "-".to_owned(), |p| p.to_string());
            line(
                d,
                &format!(
                    "target {}   position {p}   behind {error} counts",
                    s.target()
                ),
                18.0,
                t.text,
            );
            let st = cal.settings().sweep;
            line(
                d,
                &format!(
                    "stops and holds if more than {} behind, or still for {:.1} s",
                    st.max_error,
                    st.stall_after.as_secs_f64()
                ),
                15.0,
                t.dim,
            );
            return y;
        }
        match cal.checked() {
            None => line(
                d,
                "once the limits are set: centre to each limit and back, slowly; it stops at \
                 the first sign of resistance",
                17.0,
                t.dim,
            ),
            Some(o @ Outcome::Passed { max_error }) => {
                line(d, &o.to_string(), 19.0, t.good);
                for (leg, e) in max_error {
                    line(d, &format!("{leg}: within {e} counts"), 15.0, t.caption);
                }
            }
            Some(o @ Outcome::Stopped { .. }) => line(d, &o.to_string(), 19.0, t.bad),
        }
        y
    }
}

fn any_mark(cal: &CalPage) -> bool {
    [Mark::Left, Mark::Centre, Mark::Right]
        .into_iter()
        .any(|m| cal.mark(m).is_some())
}

//! The LADDER page (wrap-up step 3, the test bench): the test ladder beside the hardware.
//! What `ladder_window` shows, in raylib, from the same crate (`ladder-core`); this only
//! draws it.
//!
//! ```text
//! | PRETEND DATA / results kept in ...          | Rung 1: On stands: accelerate, brake, turn |
//! | SIMULATED CAR: on the ground  [PUT ON STANDS]| THE QUESTION  Do the actuators follow ...  |
//! | RUNG                   SIMULATOR   THE CAR   | PASSES WHEN ...                            |
//! | 9  Rules of the road   not built   not built | [RUN IN THE SIMULATOR] [STOP THE RUN]      |
//! | ...                                          | Off: why / where it runs                   |
//! | 1  On stands           PASS 23 Nov not run   | ON THE CAR: pixi run car-test --stands     |
//! | S  Sensors             not built   not built | +-- PASS: the wheels follow... (verdict) --+|
//! | THE RUN  stands: running... Step 3b          | [WHEELS] [STEERING] [BRAKE] [FAULTS]       |
//! |   the run's last lines                       | the graph: asked dashed, measured solid    |
//! |                                              | its numbers, as the tool wrote them        |
//! ```
//!
//! Run is on only when ladder-core says so (`Session::run_button`), with its reason when
//! off; the car's side is words only (`CAR_RULE`): nothing here starts a car test.

use hud_app::paint::ToColor;
use hud_core::theme::{Rgba, Tone};
use ladder_core::{
    command::TOOL,
    report::tick_label,
    session::Notice,
    show::{Cell, RunButton, CAR_RULE},
    Answer, AnswerGraph, CarSide, Faults, Jacks, Place, Session, Source, State, Status,
    VerdictState,
};
use raylib::prelude::*;

use super::{fit, wrap, Button, Frame, Room, Ui, MARGIN};
use crate::ladder::{views, View};

const TOP: f32 = 104.0;
/// The rungs' table: this many rows (the ladder has ten).
pub const ROWS: usize = 10;
/// The answer's views: up to four graphs and the faults.
pub const VIEWS: usize = 5;
const ROW_H: f32 = 38.0;
const TABLE_Y: f32 = TOP + 96.0;
const CELL_W: f32 = 128.0;
const RUN_Y: f32 = TOP + 150.0;
const VERDICT_Y: f32 = TOP + 292.0;
const VIEWS_Y: f32 = TOP + 420.0;
const GRAPH_Y: f32 = TOP + 462.0;
/// The car's slot beside each graph.
const SLOT_W: f32 = 170.0;

/// The left column's width (the rungs), and where the right one (the chosen rung) starts.
fn columns(w: f32) -> (f32, f32, f32) {
    let left = ((w - 3.0 * MARGIN) * 0.42).clamp(420.0, 640.0);
    let right_x = MARGIN + left + MARGIN;
    (left, right_x, w - MARGIN - right_x)
}

/// The page's buttons: the jacks, a row per rung, RUN and STOP, the answer's views.
pub fn layout(w: f32) -> Vec<(Button, Rectangle)> {
    let (left, rx, rw) = columns(w);
    let mut buttons = vec![(
        Button::Jacks,
        Rectangle::new(MARGIN + left - 220.0, TOP + 30.0, 220.0, 34.0),
    )];
    buttons.extend((0..ROWS).map(|i| {
        let y = TABLE_Y + i as f32 * ROW_H;
        (
            Button::Rung(i),
            Rectangle::new(MARGIN, y, left, ROW_H - 4.0),
        )
    }));
    let run_w = (rw * 0.6).min(380.0);
    buttons.push((Button::RunRung, Rectangle::new(rx, RUN_Y, run_w, 46.0)));
    buttons.push((
        Button::StopRun,
        Rectangle::new(rx + run_w + 12.0, RUN_Y, 200.0, 46.0),
    ));
    let view_w = (rw - (VIEWS as f32 - 1.0) * 8.0) / VIEWS as f32;
    buttons.extend((0..VIEWS).map(|i| {
        let x = rx + i as f32 * (view_w + 8.0);
        (Button::View(i), Rectangle::new(x, VIEWS_Y, view_w, 32.0))
    }));
    buttons
}

fn find(buttons: &[(Button, Rectangle)], b: Button) -> Option<Rectangle> {
    buttons.iter().find(|(x, _)| *x == b).map(|&(_, r)| r)
}

/// A cell's tone: colour is never the only signal, the words say it too.
fn cell_tone(cell: &Cell<'_>) -> Tone {
    match cell.state {
        State::Pass => Tone::Go,
        State::Fail => Tone::Stop,
        State::NotRun if cell.shut => Tone::Off,
        State::NotRun => Tone::Idle,
        State::NotBuilt => Tone::Off,
    }
}

fn verdict_tone(state: VerdictState) -> Tone {
    match state {
        VerdictState::Pass => Tone::Go,
        VerdictState::Fail => Tone::Stop,
        VerdictState::NotFinished | VerdictState::OneTestOnly | VerdictState::None => Tone::Wait,
    }
}

/// A view's word on its tab: `WHEELS`, `FAULTS`.
fn view_word(answer: &Answer, view: View) -> String {
    match view {
        View::Graph(i) => answer
            .graphs
            .get(i)
            .map_or_else(String::new, |g| g.id.to_uppercase()),
        View::Faults => "FAULTS".to_owned(),
    }
}

impl Ui {
    pub(super) fn draw_ladder(
        &self,
        d: &mut RaylibDrawHandle,
        f: &Frame<'_>,
        buttons: &[(Button, Rectangle)],
    ) -> Room {
        let t = self.theme;
        let (w, h) = (d.get_screen_width() as f32, d.get_screen_height() as f32);
        let none = Room {
            lane: None,
            corner: None,
        };
        let Some(session) = f.ladder.session() else {
            // No tool here: say so, plainly; every other tab still works.
            let why = f.ladder.missing().unwrap_or("no ladder here");
            self.text(d, "THE TEST LADDER", MARGIN, TOP + 8.0, 22.0, t.caption);
            let mut y = TOP + 44.0;
            for line in wrap(&self.font, why, w - 2.0 * MARGIN, 20.0) {
                self.text(d, &line, MARGIN, y, 20.0, t.warn);
                y += 26.0;
            }
            let more = "The other tabs work without it. The ladder's rules live in the tool \
                        (pixi run ladder); this tab only shows it and asks it to run.";
            for line in wrap(&self.font, more, w - 2.0 * MARGIN, 17.0) {
                self.text(d, &line, MARGIN, y + 10.0, 17.0, t.dim);
                y += 22.0;
            }
            return none;
        };
        let (left, rx, rw) = columns(w);
        self.ladder_left(d, f, session, buttons, left, h);
        self.ladder_right(d, f, session, buttons, rx, rw, h);
        let keys = "keys: Up/Down a rung   RUN runs it in the simulator   Tab next page   \
                    E e-stop   F11 full screen";
        if !f.hide_keys {
            let keys = fit(&self.font, keys, w - 2.0 * MARGIN, 15.0);
            self.text(d, &keys, MARGIN, h - 26.0, 15.0, t.dim);
        }
        none
    }

    /// The source, the jacks, the rungs' table, the run's state and its last lines.
    fn ladder_left(
        &self,
        d: &mut RaylibDrawHandle,
        f: &Frame<'_>,
        session: &Session,
        buttons: &[(Button, Rectangle)],
        left: f32,
        h: f32,
    ) {
        let t = self.theme;
        // Where the ladder comes from, and a problem reading it.
        let ladder = session.ladder();
        match session.source() {
            Source::Pretend => {
                let words = "PRETEND DATA: nothing runs, nothing is recorded";
                self.pill_words(d, words, MARGIN, TOP + 2.0, 16.0, Tone::Wait);
            }
            Source::Tool { .. } => {
                let words = ladder.map_or_else(
                    || format!("Reading the ladder ({TOOL})..."),
                    |l| format!("Results kept in {}", l.file),
                );
                let words = fit(&self.font, &words, left, 15.0);
                self.text(d, &words, MARGIN, TOP + 6.0, 15.0, t.dim);
            }
        }
        if let Some(problem) = session.problem() {
            let words = fit(&self.font, &format!("PROBLEM: {problem}"), left, 14.0);
            self.text(d, &words, MARGIN, TOP + 68.0, 14.0, t.bad);
        }

        // The simulated car's jacks: its state, and the one button ladder-core allows.
        let jacks = session.jacks();
        self.text(d, "SIMULATED CAR", MARGIN, TOP + 32.0, 14.0, t.caption);
        let ink = match jacks {
            Jacks::OnStands => t.warn,
            Jacks::OnGround => t.good,
            _ => t.dim,
        };
        let state = fit(&self.font, &jacks.to_string(), left - 236.0, 15.0);
        self.text(d, &state, MARGIN, TOP + 49.0, 15.0, ink);
        if let Some(r) = find(buttons, Button::Jacks) {
            let on = session.jacks_button().is_some();
            self.button_with(d, Button::Jacks, r, f, on, jacks.button_words());
        }
        if let Some(said) = session.jacks_said() {
            let word = if said.good { "Done" } else { "REFUSED" };
            let line = fit(&self.font, &format!("{word}: {}", said.text), left, 14.0);
            let ink = if said.good { t.good } else { t.bad };
            self.text(d, &line, MARGIN, TOP + 68.0, 14.0, ink);
        }

        // The table: the top rung first, a column a place.
        let name_w = left - 2.0 * CELL_W - 24.0;
        let cell_x = |i: f32| MARGIN + name_w + 12.0 + i * (CELL_W + 6.0);
        self.text(d, "RUNG", MARGIN + 8.0, TABLE_Y - 20.0, 14.0, t.caption);
        for (i, place) in Place::ALL.into_iter().enumerate() {
            let x = cell_x(i as f32);
            self.text(d, place.heading(), x, TABLE_Y - 20.0, 14.0, t.caption);
        }
        let rows = f.ladder.rows();
        let chosen = f.ladder.selected_row();
        for (i, rung) in rows.iter().enumerate().take(ROWS) {
            let Some(r) = find(buttons, Button::Rung(i)) else {
                continue;
            };
            if chosen == Some(i) {
                d.draw_rectangle_rounded(r, t.style.corner(0.2), 6, t.button_down.rl());
                d.draw_rectangle_rounded_lines_ex(
                    r,
                    t.style.corner(0.2),
                    6,
                    1.5,
                    t.glass_edge.rl(),
                );
            } else {
                d.draw_line_ex(
                    Vector2::new(r.x, r.y + r.height + 2.0),
                    Vector2::new(r.x + r.width, r.y + r.height + 2.0),
                    1.0,
                    t.section.rl(),
                );
            }
            let name = format!("{}  {}", rung.number, rung.title);
            let name = fit(&self.font, &name, name_w - 8.0, 17.0);
            let ink = if rung.built { t.text } else { t.dim };
            self.text(d, &name, r.x + 8.0, r.y + 8.0, 17.0, ink);
            for (c, place) in Place::ALL.into_iter().enumerate() {
                let cell = Cell::of(rung.side(place));
                let words = fit(&self.font, &cell.words(), CELL_W - 16.0, 14.0);
                let p = t.pill(cell_tone(&cell));
                let pill = Rectangle::new(cell_x(c as f32), r.y + 5.0, CELL_W, r.height - 10.0);
                d.draw_rectangle_rounded(pill, t.style.corner(0.3), 6, p.fill.rl());
                self.text(d, &words, pill.x + 8.0, pill.y + 6.0, 14.0, p.ink);
            }
        }
        let mut y = TABLE_Y + ROWS as f32 * ROW_H + 4.0;
        let note = if rows.len() > ROWS {
            format!(
                "and {} more: pixi run ladder lists them all",
                rows.len() - ROWS
            )
        } else {
            "A rung opens when the one under it has passed; \"shut\": not open yet.".to_owned()
        };
        let note = fit(&self.font, &note, left, 13.0);
        self.text(d, &note, MARGIN, y, 13.0, t.dim);
        y += 26.0;

        // The run: what it is doing, or how the last one ended; then its last lines.
        self.text(d, "THE RUN", MARGIN, y, 14.0, t.caption);
        y += 18.0;
        let (status, ink) = match (session.running(), session.last_run()) {
            (Some(run), _) => {
                let what = match run.test {
                    Some(test) => format!("{}, {test} alone", run.rung),
                    None => run.rung.to_owned(),
                };
                let step = run.step.map_or_else(String::new, |s| format!(" {s}"));
                (format!("{what}: running...{step}"), t.warn)
            }
            (None, Some(Notice { good, text })) => {
                (text.clone(), if *good { t.good } else { t.bad })
            }
            (None, None) => ("nothing is running".to_owned(), t.dim),
        };
        for line in wrap(&self.font, &status, left, 15.0).into_iter().take(2) {
            self.text(d, &line, MARGIN, y, 15.0, ink);
            y += 19.0;
        }
        let room = ((h - 34.0 - y) / 16.0).max(0.0) as usize;
        let console: Vec<&str> = session.console().collect();
        for line in &console[console.len().saturating_sub(room)..] {
            let line = fit(&self.font, line, left, 13.0);
            self.text(d, &line, MARGIN, y, 13.0, t.caption);
            y += 16.0;
        }
    }

    /// The chosen rung: its question, RUN and STOP, the car's side, the answer.
    #[allow(clippy::too_many_arguments)]
    fn ladder_right(
        &self,
        d: &mut RaylibDrawHandle,
        f: &Frame<'_>,
        session: &Session,
        buttons: &[(Button, Rectangle)],
        rx: f32,
        rw: f32,
        h: f32,
    ) {
        let t = self.theme;
        let Some(rung) = session.selected() else {
            self.text(d, "Reading the ladder...", rx, TOP + 8.0, 20.0, t.dim);
            return;
        };
        let heading = fit(&self.font, &rung.heading(), rw, 24.0);
        self.text(d, &heading, rx, TOP + 4.0, 24.0, t.text);
        self.text(d, "THE QUESTION", rx, TOP + 36.0, 14.0, t.caption);
        self.lines(d, &rung.question, rx, TOP + 54.0, rw, 20.0, 2, t.text);
        let passes = format!("It passes when: {}", rung.passes);
        self.lines(d, &passes, rx, TOP + 106.0, rw, 15.0, 2, t.caption);

        // RUN and STOP: ladder-core decides whether RUN is on, and why not.
        let button = session.run_button();
        let running = session.running();
        if let Some(r) = find(buttons, Button::RunRung) {
            let on = matches!(button, Some(RunButton::Ready(_)));
            let words = format!("RUN RUNG {} IN THE SIMULATOR", rung.number);
            self.button_with(d, Button::RunRung, r, f, on, &words);
        }
        if let Some(r) = find(buttons, Button::StopRun) {
            let words = if running.is_some_and(|r| r.stop_asked) {
                "STOP HARDER (KILL)"
            } else {
                "STOP THE RUN"
            };
            self.button_with(d, Button::StopRun, r, f, running.is_some(), words);
        }
        let (why, ink) = match (&button, session.runs_in()) {
            (Some(RunButton::Off(why)), _) => (format!("Off: {why}"), t.warn),
            (_, Some(runs_in)) => (runs_in.to_string(), t.dim),
            _ => (String::new(), t.dim),
        };
        self.lines(d, &why, rx, RUN_Y + 54.0, rw, 15.0, 2, ink);

        // The car's side: words to read and copy, never a button.
        let car = match CarSide::of(rung) {
            CarSide::Command(command) => format!("ON THE CAR: {command}"),
            CarSide::ByHand { how } => format!("ON THE CAR: no one command; by hand ({how})"),
        };
        let car = fit(&self.font, &car, rw, 15.0);
        self.text(d, &car, rx, TOP + 244.0, 15.0, t.caption);
        let rule = fit(&self.font, CAR_RULE, rw, 15.0);
        self.text(d, &rule, rx, TOP + 264.0, 15.0, t.text);

        // The answer: the verdict in words, then the views under their tabs.
        if !rung.built {
            let words = format!(
                "Not built yet: nothing to answer with. Today: {}",
                rung.today
            );
            self.lines(d, &words, rx, VERDICT_Y, rw, 17.0, 4, t.dim);
            return;
        }
        if let Some(run) = running.filter(|r| r.rung == rung.id) {
            let words = format!(
                "RUNNING NOW: {}. The answer below is the run before; the new one comes when \
                 it ends.",
                run.step.unwrap_or("starting...")
            );
            self.lines(d, &words, rx, VERDICT_Y - 22.0, rw, 14.0, 1, t.warn);
        }
        let Some(report) = session.report() else {
            self.text(d, "Reading the last run...", rx, VERDICT_Y, 17.0, t.dim);
            return;
        };
        let Some(answer) = report.answer.as_ref().filter(|a| a.found) else {
            let why = report
                .answer
                .as_ref()
                .map_or(report.why.as_str(), |a| a.why.as_str());
            let why = if why.is_empty() {
                "This run has no answer to show; pixi run ladder-window shows every step."
            } else {
                why
            };
            self.lines(d, why, rx, VERDICT_Y, rw, 17.0, 4, t.dim);
            return;
        };
        let p = t.pill(verdict_tone(answer.verdict.state));
        let words = wrap(&self.font, &answer.verdict.words, rw - 24.0, 17.0);
        let shown = words.len().min(4);
        let box_r = Rectangle::new(rx, VERDICT_Y, rw, 12.0 + shown as f32 * 21.0);
        d.draw_rectangle_rounded_lines_ex(box_r, t.style.corner(0.1), 6, 3.0, p.fill.rl());
        self.lines(
            d,
            &answer.verdict.words,
            rx + 12.0,
            VERDICT_Y + 6.0,
            rw - 24.0,
            17.0,
            4,
            t.text,
        );
        let when = format!(
            "Run {} in {}; the code's commit {}",
            answer.started, answer.place, answer.commit
        );
        let when = fit(&self.font, &when, rw, 13.0);
        self.text(d, &when, rx, VERDICT_Y + 104.0, 13.0, t.dim);

        let all = views(answer);
        for (i, view) in all.iter().enumerate().take(VIEWS) {
            if let Some(r) = find(buttons, Button::View(i)) {
                self.button_with(d, Button::View(i), r, f, true, &view_word(answer, *view));
            }
        }
        let [mine, other] = answer.places();
        match f.ladder.view() {
            Some(View::Graph(i)) => {
                if let Some(graph) = answer.graphs.get(i) {
                    self.answer_graph(d, graph, (mine, other), rx, rw, h);
                }
            }
            Some(View::Faults) => {
                if let Some(faults) = &answer.faults {
                    self.faults(d, faults, (mine, other), rx, rw, h);
                }
            }
            None => {}
        }
    }

    /// One graph of the answer: its two lines, then its numbers; the car's slot beside.
    fn answer_graph(
        &self,
        d: &mut RaylibDrawHandle,
        g: &AnswerGraph,
        (mine, other): (&str, &str),
        rx: f32,
        rw: f32,
        h: f32,
    ) {
        let t = self.theme;
        let wide = rw - SLOT_W - 16.0;
        let title = fit(&self.font, &format!("{mine}: {}", g.title), wide, 16.0);
        self.text(d, &title, rx, GRAPH_Y, 16.0, t.text);
        self.car_slot(d, other, &g.car, rx + wide + 16.0, GRAPH_Y, h);
        let table_h = 17.0 * (1 + g.rows.len() + g.facts.len()) as f32 + 18.0;
        // The value axis's name over the plot, the legend under it.
        let unit = fit(&self.font, &g.y_label, wide, 13.0);
        self.text(d, &unit, rx, GRAPH_Y + 22.0, 13.0, t.caption);
        let top = GRAPH_Y + 42.0;
        let plot_h = (h - 34.0 - top - table_h - 34.0).clamp(60.0, 220.0);
        let plot = Rectangle::new(rx + 52.0, top, wide - 52.0, plot_h);
        let mut y = match g.lines() {
            Some(lines) => {
                self.plot(d, lines, plot, g);
                plot.y + plot.height + 34.0
            }
            None => {
                self.lines(d, &g.missing, rx, top, wide, 16.0, 2, t.warn);
                top + 44.0
            }
        };
        y = self.words_table(
            d,
            &g.columns,
            g.rows.iter().map(Vec::as_slice),
            rx,
            wide,
            y,
            h,
        );
        for fact in &g.facts {
            if let [what, value] = fact.as_slice() {
                let line = fit(&self.font, &format!("{what}: {value}"), wide, 14.0);
                if y < h - 46.0 {
                    self.text(d, &line, rx, y, 14.0, t.text);
                }
                y += 17.0;
            }
        }
        if y < h - 46.0 {
            let judged = fit(&self.font, &g.judged, wide, 13.0);
            self.text(d, &judged, rx, y + 2.0, 13.0, t.dim);
        }
    }

    /// Commanded (dashed) against measured (solid), with the axes' round numbers.
    fn plot(
        &self,
        d: &mut RaylibDrawHandle,
        lines: ladder_core::Lines<'_>,
        plot: Rectangle,
        g: &AnswerGraph,
    ) {
        let t = self.theme;
        d.draw_rectangle_rec(plot, t.track.rl());
        let (Some(time), Some(values)) = (lines.time(), lines.values()) else {
            return;
        };
        let x_of = |s: f64| plot.x + plot.width * time.share(s) as f32;
        let y_of = |v: f64| plot.y + plot.height * (1.0 - values.share(v) as f32);
        for v in values.ticks(4) {
            let y = y_of(v);
            d.draw_line_ex(
                Vector2::new(plot.x, y),
                Vector2::new(plot.x + plot.width, y),
                1.0,
                if v == 0.0 { t.dim } else { t.section }.rl(),
            );
            let label = tick_label(v);
            let lw = self.font.measure(&label, 12.0);
            self.text(d, &label, plot.x - lw - 6.0, y - 7.0, 12.0, t.dim);
        }
        for s in time.ticks(6) {
            let label = tick_label(s);
            let lw = self.font.measure(&label, 12.0);
            let x = (x_of(s) - lw / 2.0).clamp(plot.x, plot.x + plot.width - lw);
            self.text(d, &label, x, plot.y + plot.height + 2.0, 12.0, t.dim);
        }
        // Asked: dashed, in the words' colour; measured: solid, in the local EKF's blue
        // (as the GRAPHS page's second line), so the two read apart in every theme.
        let asked: Vec<Vector2> = lines
            .points(lines.asked)
            .filter(|(s, v)| s.is_finite() && v.is_finite())
            .map(|(s, v)| Vector2::new(x_of(s), y_of(v)))
            .collect();
        for (i, pair) in asked.windows(2).enumerate() {
            if i % 4 < 2 {
                d.draw_line_ex(pair[0], pair[1], 2.0, t.text.rl());
            }
        }
        let measured_ink = t.scene.localizers.local;
        let measured: Vec<Vector2> = lines
            .points(lines.measured)
            .filter(|(s, v)| s.is_finite() && v.is_finite())
            .map(|(s, v)| Vector2::new(x_of(s), y_of(v)))
            .collect();
        for pair in measured.windows(2) {
            d.draw_line_ex(pair[0], pair[1], 2.5, measured_ink.rl());
        }
        // The legend and the axes' names, under the time's numbers.
        let y = plot.y + plot.height + 16.0;
        let asked = fit(&self.font, &g.asked_words, plot.width * 0.38, 13.0);
        self.text(d, &asked, plot.x, y, 13.0, t.text);
        let solid = fit(&self.font, &g.measured_words, plot.width * 0.38, 13.0);
        let x = plot.x + plot.width * 0.4;
        self.text(d, &solid, x, y, 13.0, measured_ink);
        let time = fit(&self.font, &g.x_label, plot.width * 0.2, 13.0);
        let tw = self.font.measure(&time, 13.0);
        self.text(d, &time, plot.x + plot.width - tw, y, 13.0, t.caption);
    }

    /// The faults tried on purpose: did each stop the car, in how long.
    fn faults(
        &self,
        d: &mut RaylibDrawHandle,
        faults: &Faults,
        (mine, other): (&str, &str),
        rx: f32,
        rw: f32,
        h: f32,
    ) {
        let t = self.theme;
        let wide = rw - SLOT_W - 16.0;
        let title = format!("{mine}: faults on purpose: did each stop the car?");
        let title = fit(&self.font, &title, wide, 16.0);
        self.text(d, &title, rx, GRAPH_Y, 16.0, t.text);
        self.car_slot(d, other, &faults.car, rx + wide + 16.0, GRAPH_Y, h);
        if !faults.missing.is_empty() {
            self.lines(
                d,
                &faults.missing,
                rx,
                GRAPH_Y + 24.0,
                wide,
                15.0,
                2,
                t.warn,
            );
            return;
        }
        // The fault and its answer get the room; "counted from" is the longest, last.
        let shares = [0.38, 0.2, 0.12, 0.1, 0.2];
        let xs: Vec<f32> = shares
            .iter()
            .scan(rx, |x, share| {
                let at = *x;
                *x += wide * share;
                Some(at)
            })
            .collect();
        let mut y = GRAPH_Y + 26.0;
        for (i, column) in faults.columns.iter().enumerate().take(xs.len()) {
            let words = fit(&self.font, column, wide * shares[i] - 8.0, 13.0);
            self.text(d, &words, xs[i], y, 13.0, t.caption);
        }
        y += 18.0;
        for row in &faults.rows {
            if y > h - 50.0 {
                break;
            }
            let ink = match row.status {
                Status::Pass => t.text,
                Status::Fail => t.bad,
                Status::Skipped | Status::NotRun => t.dim,
            };
            let stopped_ink = match row.stopped.as_str() {
                "NO" => t.bad,
                s if s.starts_with("yes") || s.starts_with("caught") => t.good,
                _ => t.warn,
            };
            for (i, cell) in row.cells().iter().enumerate().take(xs.len()) {
                let words = fit(&self.font, cell, wide * shares[i] - 8.0, 14.0);
                let ink = if i == 1 { stopped_ink } else { ink };
                self.text(d, &words, xs[i], y, 14.0, ink);
            }
            y += 19.0;
        }
    }

    /// The other place's slot: `no car run yet`, until a run there fills it.
    fn car_slot(&self, d: &mut RaylibDrawHandle, other: &str, slot: &str, x: f32, y: f32, h: f32) {
        let t = self.theme;
        let r = Rectangle::new(x, y, SLOT_W, (h - 40.0 - y).min(150.0));
        d.draw_rectangle_rounded_lines_ex(r, t.style.corner(0.1), 6, 1.0, t.glass_edge.rl());
        self.text(d, other, x + 10.0, y + 8.0, 14.0, t.caption);
        let mut words = slot.to_owned();
        if let Some(first) = words.get_mut(..1) {
            first.make_ascii_uppercase();
        }
        let words = format!("{words}. Its graph and numbers go here once it has run there.");
        self.lines(d, &words, x + 10.0, y + 30.0, SLOT_W - 20.0, 13.0, 6, t.dim);
    }

    /// A table of the tool's own words; returns the y under it.
    #[allow(clippy::too_many_arguments)]
    fn words_table<'a>(
        &self,
        d: &mut RaylibDrawHandle,
        columns: &[String],
        rows: impl Iterator<Item = &'a [String]>,
        x: f32,
        wide: f32,
        mut y: f32,
        h: f32,
    ) -> f32 {
        let t = self.theme;
        if columns.is_empty() {
            return y;
        }
        let col_w = wide / columns.len() as f32;
        for (i, column) in columns.iter().enumerate() {
            let words = fit(&self.font, column, col_w - 8.0, 13.0);
            self.text(d, &words, x + i as f32 * col_w, y, 13.0, t.caption);
        }
        y += 17.0;
        for row in rows {
            if y > h - 50.0 {
                break;
            }
            for (i, cell) in row.iter().enumerate().take(columns.len()) {
                let words = fit(&self.font, cell, col_w - 8.0, 14.0);
                self.text(d, &words, x + i as f32 * col_w, y, 14.0, t.text);
            }
            y += 17.0;
        }
        y + 4.0
    }

    /// Words wrapped to `width`, at most `most` lines; the last cut with "..." if more.
    #[allow(clippy::too_many_arguments)]
    fn lines(
        &self,
        d: &mut RaylibDrawHandle,
        s: &str,
        x: f32,
        y: f32,
        width: f32,
        size: f32,
        most: usize,
        ink: Rgba,
    ) {
        let mut lines = wrap(&self.font, s, width, size);
        if lines.len() > most {
            lines.truncate(most);
            if let Some(last) = lines.last_mut() {
                *last = fit(&self.font, &format!("{last} ..."), width, size);
                if !last.ends_with("...") {
                    last.push_str("...");
                }
            }
        }
        for (i, line) in lines.iter().enumerate() {
            self.text(d, line, x, y + i as f32 * (size + 4.0), size, ink);
        }
    }

    /// Words on a pill of their tone.
    fn pill_words(&self, d: &mut RaylibDrawHandle, s: &str, x: f32, y: f32, size: f32, tone: Tone) {
        let t = self.theme;
        let p = t.pill(tone);
        let ww = self.font.measure(s, size);
        let r = Rectangle::new(x, y, ww + 20.0, size + 10.0);
        d.draw_rectangle_rounded(r, t.style.corner(0.3), 6, p.fill.rl());
        self.text(d, s, x + 10.0, y + 5.0, size, p.ink);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_page_fits_the_smallest_window() {
        for w in [1100.0, 1280.0, 1600.0, 1920.0] {
            let (left, rx, rw) = columns(w);
            assert!(left >= 420.0 && rw > 400.0, "{w}: {left} {rw}");
            assert!(rx + rw <= w - MARGIN + 0.01);
            for (b, r) in layout(w) {
                assert!(r.x >= 0.0 && r.x + r.width <= w, "{b:?} at {w}");
            }
        }
    }

    #[test]
    fn a_cells_tone_matches_its_words() {
        let cell = |state, shut| Cell {
            state,
            shut,
            day: None,
            why_shut: &[],
        };
        assert_eq!(cell_tone(&cell(State::Pass, false)), Tone::Go);
        assert_eq!(cell_tone(&cell(State::Fail, true)), Tone::Stop);
        assert_eq!(cell_tone(&cell(State::NotRun, true)), Tone::Off);
        assert_eq!(cell_tone(&cell(State::NotRun, false)), Tone::Idle);
        assert_eq!(verdict_tone(VerdictState::Pass), Tone::Go);
        assert_eq!(verdict_tone(VerdictState::NotFinished), Tone::Wait);
    }
}

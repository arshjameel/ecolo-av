//! The test ladder's window, drawn in egui: the ladder on the left, the chosen rung on the
//! right, the run's console along the bottom. What each part says is `ladder-core`'s.
//!
//! The car's side has words and a Copy button, and nothing that runs. The one switch that
//! moves anything is "The simulated car": Ravensim's jacks, on only while a simulator runs
//! on this computer (`ladder-core`'s `jacks`, through `pixi run jacks`).

use bevy_egui::egui;
use ladder_core::{
    report::tick_label, session::Notice, show::CAR_RULE, CarSide, Cell, Fit, Graph, Jacks, Ladder,
    Outcome, Place, Report, RunButton, Rung, Session, Side, Source, State, Status, StepRow, Watch,
    WatchButton, Which,
};

const GREEN: egui::Color32 = egui::Color32::from_rgb(45, 210, 110);
const RED: egui::Color32 = egui::Color32::from_rgb(240, 85, 85);
const AMBER: egui::Color32 = egui::Color32::from_rgb(240, 190, 70);
const GREY: egui::Color32 = egui::Color32::from_gray(165);
/// A shut cell: darker than "not run", and it says "shut".
const SHUT: egui::Color32 = egui::Color32::from_gray(95);
const DARK: egui::Color32 = egui::Color32::from_gray(12);
const BRIGHT: egui::Color32 = egui::Color32::from_gray(235);
/// The graph's two lines: the first two hues of the charts' dark palette (blue, orange).
/// They differ by their dashes too, and the legend says which is which in words.
const COMMANDED: egui::Color32 = egui::Color32::from_rgb(0xd9, 0x59, 0x26);
const MEASURED: egui::Color32 = egui::Color32::from_rgb(0x39, 0x87, 0xe5);
/// The graph's grid and axes: there, but behind the lines.
const GRID: egui::Color32 = egui::Color32::from_gray(52);
/// The longest "what happened" a table row shows; the rest is in the hover.
const DETAIL_CHARS: usize = 60;

/// What a press asks for; done after the frame's drawing, when nothing is borrowed.
enum Action {
    Select(String),
    Run,
    Stop,
    Record,
    Jacks,
    RunTest(String),
    Watch(Watch),
    Step(String),
    Show(Which),
}

/// Big, plain words: the docs' rule for a dyslexic reader holds on screens too.
pub fn style(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.override_text_color = Some(BRIGHT);
    ctx.set_visuals(visuals);
    ctx.all_styles_mut(|style| {
        for (text, size) in [
            (egui::TextStyle::Body, 18.0),
            (egui::TextStyle::Button, 18.0),
            (egui::TextStyle::Monospace, 15.0),
            (egui::TextStyle::Heading, 26.0),
            (egui::TextStyle::Small, 14.0),
        ] {
            if let Some(font) = style.text_styles.get_mut(&text) {
                font.size = size;
            }
        }
        style.spacing.item_spacing = egui::vec2(10.0, 8.0);
        style.spacing.button_padding = egui::vec2(12.0, 6.0);
    });
}

/// One frame of the whole window.
pub fn window(ctx: &egui::Context, session: &mut Session) {
    let mut actions = Vec::new();
    let ladder = session.ladder().cloned();
    let chosen = session.selected().cloned();
    let mut root = egui::Ui::new(
        ctx.clone(),
        "ladder_window".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );
    egui::Panel::top("where").show(&mut root, |ui| {
        top(ui, session, ladder.as_ref());
        jacks(ui, session, &mut actions);
    });
    egui::Panel::bottom("console")
        .default_size(130.0)
        .resizable(true)
        .show(&mut root, |ui| console(ui, session, &mut actions));
    egui::Panel::left("ladder")
        .default_size(450.0)
        .resizable(true)
        .show(&mut root, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| match &ladder {
                Some(ladder) => rungs(ui, ladder, chosen.as_ref(), &mut actions),
                None => {
                    ui.label("Reading the ladder...");
                }
            });
        });
    // The results get their own column, so the table and the graph are seen without
    // scrolling past the buttons.
    egui::Panel::right("results")
        .default_size(720.0)
        .resizable(true)
        .show(&mut root, |ui| {
            egui::ScrollArea::vertical()
                .id_salt("results")
                .show(ui, |ui| match &chosen {
                    Some(rung) => last_session(ui, session, rung, &mut actions),
                    None => {
                        ui.label("Choose a rung on the left.");
                    }
                });
        });
    egui::CentralPanel::default().show(&mut root, |ui| {
        egui::ScrollArea::vertical().show(ui, |ui| match &chosen {
            Some(rung) => detail(ui, session, rung, &mut actions),
            None => {
                ui.label("Choose a rung on the left.");
            }
        });
    });
    for action in actions {
        match action {
            Action::Select(id) => session.select(&id),
            Action::Run => session.press_run(),
            Action::Stop => session.press_stop(),
            Action::Record => session.press_record(),
            Action::Jacks => session.press_jacks(),
            Action::RunTest(name) => session.press_run_test(&name),
            Action::Watch(watch) => session.press_watch(watch),
            Action::Step(name) => session.select_step(&name),
            Action::Show(which) => session.show(which),
        }
    }
}

fn top(ui: &mut egui::Ui, session: &Session, ladder: Option<&Ladder>) {
    ui.horizontal_wrapped(|ui| {
        ui.heading("The test ladder");
        match session.source() {
            Source::Pretend => {
                pill(
                    ui,
                    "PRETEND DATA: nothing runs, nothing is recorded",
                    AMBER,
                    DARK,
                );
            }
            Source::Tool { .. } => {
                if let Some(ladder) = ladder {
                    ui.label(egui::RichText::new(format!("Progress: {}", ladder.file)).color(GREY));
                }
            }
        }
    });
    if let Some(problem) = session.problem() {
        ui.label(
            egui::RichText::new(format!("PROBLEM: {problem}"))
                .strong()
                .color(RED),
        );
        ui.label("The ladder shown is the last one read. Start it with: pixi run ladder-window");
    }
}

/// "The simulated car: on stands / on the ground", and the one button that changes it.
/// Off, with the reason, unless a simulator runs on this computer.
fn jacks(ui: &mut egui::Ui, session: &Session, actions: &mut Vec<Action>) {
    ui.horizontal_wrapped(|ui| {
        ui.strong("The simulated car:");
        let state = session.jacks();
        let colour = match state {
            Jacks::OnStands => AMBER,
            Jacks::OnGround => GREEN,
            _ => GREY,
        };
        ui.label(
            egui::RichText::new(state.to_string())
                .strong()
                .color(colour),
        );
        let button = egui::Button::new(state.button_words());
        if ui
            .add_enabled(session.jacks_button().is_some(), button)
            .on_hover_text("Ravensim only, on this computer: pixi run jacks up|down --sim")
            .clicked()
        {
            actions.push(Action::Jacks);
        }
        if let Some(notice) = session.jacks_said() {
            said(ui, notice, "Done", "REFUSED");
        }
    });
}

/// The ladder: the top rung at the top, a row a rung, a column a place.
fn rungs(ui: &mut egui::Ui, ladder: &Ladder, chosen: Option<&Rung>, actions: &mut Vec<Action>) {
    egui::Grid::new("rungs")
        .num_columns(3)
        .spacing(egui::vec2(18.0, 14.0))
        .show(ui, |ui| {
            ui.strong("Rung");
            for place in Place::ALL {
                ui.strong(place.heading());
            }
            ui.end_row();
            for rung in ladder.top_first() {
                let is_chosen = chosen.is_some_and(|c| c.id == rung.id);
                let name = egui::RichText::new(&rung.id).strong().size(20.0);
                if ui
                    .selectable_label(is_chosen, name)
                    .on_hover_text(&rung.title)
                    .clicked()
                {
                    actions.push(Action::Select(rung.id.clone()));
                }
                for place in Place::ALL {
                    cell(ui, &Cell::of(rung.side(place)));
                }
                ui.end_row();
            }
        });
    ui.add_space(8.0);
    ui.label(
        egui::RichText::new("shut: the rungs under it have not passed yet. Hover for why.")
            .small()
            .color(GREY),
    );
}

/// One cell: the state in words, on its colour; a shut one says so and why.
fn cell(ui: &mut egui::Ui, cell: &Cell<'_>) {
    let (fill, text) = match cell.state {
        State::Pass => (GREEN, DARK),
        State::Fail => (RED, DARK),
        State::NotRun if cell.shut => (SHUT, BRIGHT),
        State::NotRun => (GREY, DARK),
    };
    let response = pill(ui, &cell.words(), fill, text);
    if cell.shut {
        response.on_hover_text(format!("Shut:\n{}", cell.why_shut.join("\n")));
    }
}

fn pill(
    ui: &mut egui::Ui,
    words: &str,
    fill: egui::Color32,
    text: egui::Color32,
) -> egui::Response {
    egui::Frame::new()
        .fill(fill)
        .corner_radius(4.0)
        .inner_margin(egui::vec2(10.0, 4.0))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(words).strong().color(text));
        })
        .response
}

/// The chosen rung: what it is, each side, and the form.
fn detail(ui: &mut egui::Ui, session: &mut Session, rung: &Rung, actions: &mut Vec<Action>) {
    ui.heading(format!("{}: {}", rung.id, rung.title));
    // A line each, wrapped: this column is narrow beside the results.
    for (name, words) in [
        ("Where", &rung.location),
        ("It measures", &rung.measures),
        ("It passes when", &rung.passes),
        ("How, by hand", &rung.how),
    ] {
        ui.horizontal_wrapped(|ui| {
            ui.strong(format!("{name}:"));
            ui.label(words);
        });
    }

    ui.separator();
    ui.heading(Place::Sim.heading());
    side(ui, &rung.sim);
    match rung.sim.command.as_str() {
        "" => ui.label("No one command: it is done by hand."),
        command => ui.label(egui::RichText::new(command).monospace()),
    };
    let button = session.run_button();
    let ready = matches!(button, Some(RunButton::Ready(_)));
    ui.horizontal_wrapped(|ui| {
        let run = egui::Button::new(egui::RichText::new("Run in the simulator").strong());
        if ui.add_enabled(ready, run).clicked() {
            actions.push(Action::Run);
        }
        if let Some(RunButton::Off(why)) = &button {
            ui.label(egui::RichText::new(format!("Off: {why}")).color(AMBER));
        }
    });
    if let Some(runs_in) = session.runs_in() {
        ui.label(runs_in.to_string());
    }
    if !rung.after.is_empty() {
        ui.add(egui::Label::new(egui::RichText::new(&rung.after).color(GREY)).wrap());
    }
    single_tests(ui, session, rung, actions);

    ui.separator();
    ui.heading(Place::Car.heading());
    side(ui, &rung.car);
    match CarSide::of(rung) {
        CarSide::Command(command) => {
            ui.horizontal_wrapped(|ui| {
                ui.label(egui::RichText::new(command).monospace());
                if ui.button("Copy").clicked() {
                    ui.ctx().copy_text(command.to_owned());
                }
            });
        }
        CarSide::ByHand { how } => {
            ui.label(format!("No one command: it is done by hand ({how})."));
        }
    }
    ui.label(egui::RichText::new(CAR_RULE).strong());

    ui.separator();
    record_form(ui, session, rung, actions);
    research(ui, session, actions);
}

/// The research watch, set clearly apart from the ladder: its own heading, its own box,
/// under everything the capstone's rung shows.
fn research(ui: &mut egui::Ui, session: &Session, actions: &mut Vec<Action>) {
    ui.add_space(18.0);
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(
            egui::RichText::new(ladder_core::research::HEADING)
                .strong()
                .color(AMBER),
        );
        ui.add(
            egui::Label::new(egui::RichText::new(ladder_core::research::RULE).color(GREY)).wrap(),
        );
        egui::Grid::new("research")
            .num_columns(3)
            .spacing(egui::vec2(14.0, 6.0))
            .show(ui, |ui| {
                for watch in Watch::ALL {
                    let button = session.watch_button(watch);
                    let ready = matches!(button, WatchButton::Ready(_));
                    let hover = match &button {
                        WatchButton::Ready(command) => command.line(),
                        WatchButton::Off(why) => format!("Off: {why}"),
                    };
                    if ui
                        .add_enabled(ready, egui::Button::new("Run"))
                        .on_hover_text(&hover)
                        .on_disabled_hover_text(&hover)
                        .clicked()
                    {
                        actions.push(Action::Watch(watch));
                    }
                    ui.strong(format!("barrier-watch {}", watch.word()));
                    ui.label(watch.says());
                    ui.end_row();
                }
            });
    });
}

/// The rung's tests, one at a time: each with its own Run. Simulator only: the tool's
/// `run --only` has no car side.
fn single_tests(ui: &mut egui::Ui, session: &Session, rung: &Rung, actions: &mut Vec<Action>) {
    if rung.tests.is_empty() {
        return;
    }
    ui.add_space(6.0);
    ui.strong("Its tests, one at a time (simulator only)");
    ui.label(
        egui::RichText::new(
            "A single test is not recorded: a rung passes whole. The car goes on stands or \
             to the ground by itself, as the test needs.",
        )
        .color(GREY),
    );
    egui::Grid::new("tests")
        .num_columns(4)
        .spacing(egui::vec2(14.0, 6.0))
        .show(ui, |ui| {
            for test in &rung.tests {
                let button = session.test_button(&test.name);
                let ready = matches!(button, Some(RunButton::Ready(_)));
                if ui
                    .add_enabled(ready, egui::Button::new("Run"))
                    .on_hover_text(&test.command)
                    .on_disabled_hover_text(match &button {
                        Some(RunButton::Off(why)) => format!("Off: {why}"),
                        _ => "Off".into(),
                    })
                    .clicked()
                {
                    actions.push(Action::RunTest(test.name.clone()));
                }
                ui.label(&test.number);
                ui.strong(&test.name);
                ui.label(&test.title);
                ui.end_row();
            }
        });
}

/// The rung's newest simulator session: the step running now, then each step's result as a
/// table, the chosen step's graph and numbers, and a hallway session's fits.
fn last_session(ui: &mut egui::Ui, session: &Session, rung: &Rung, actions: &mut Vec<Action>) {
    ui.heading("The last session in the simulator");
    // Close rows: fourteen steps and a graph fit one screen.
    ui.spacing_mut().item_spacing.y = 4.0;
    if rung.tests.is_empty() {
        ui.label(
            egui::RichText::new(
                "This rung keeps no steps to show here: its run's lines are in the console, \
                 and its result is on the ladder.",
            )
            .color(GREY),
        );
        return;
    }
    if let Some(running) = session.running().filter(|r| r.rung == rung.id) {
        let step = running.step.unwrap_or("starting...");
        ui.label(
            egui::RichText::new(format!("RUNNING NOW: {step}"))
                .strong()
                .color(AMBER),
        );
        ui.label(
            egui::RichText::new("The table below is the session before this run.").color(GREY),
        );
    }
    ui.horizontal_wrapped(|ui| {
        ui.strong("Show");
        for which in Which::ALL {
            if ui
                .selectable_label(session.which() == which, which.words())
                .clicked()
            {
                actions.push(Action::Show(which));
            }
        }
    });
    let Some(report) = session.report() else {
        ui.label(egui::RichText::new("Reading the last session...").color(GREY));
        return;
    };
    if !report.found {
        ui.label(egui::RichText::new(&report.why).color(GREY));
        return;
    }
    ui.add(
        egui::Label::new(format!(
            "{}: {}. {}",
            report.started, report.counts, report.location
        ))
        .wrap(),
    );
    ui.label(egui::RichText::new(format!("{} {}", report.standing(), report.folder)).color(GREY));
    let chosen = session.step();
    steps_table(ui, report, chosen, actions);
    if let Some(step) = chosen {
        step_detail(ui, step);
    }
    if !report.fits.is_empty() {
        fits_table(ui, &report.fits);
    }
}

/// A step's result on its colour, in its word.
fn status_pill(ui: &mut egui::Ui, status: Status) {
    let (fill, text) = match status {
        Status::Pass => (GREEN, DARK),
        Status::Fail => (RED, DARK),
        Status::Skipped => (GREY, DARK),
        Status::NotRun => (SHUT, BRIGHT),
    };
    egui::Frame::new()
        .fill(fill)
        .corner_radius(3.0)
        .inner_margin(egui::vec2(8.0, 0.0))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(status.word()).strong().color(text));
        });
}

fn steps_table(
    ui: &mut egui::Ui,
    report: &Report,
    chosen: Option<&StepRow>,
    actions: &mut Vec<Action>,
) {
    // Its own scroll, so the chosen step's graph stays in view under it.
    egui::ScrollArea::vertical()
        .id_salt("steps")
        .max_height(300.0)
        .show(ui, |ui| {
            egui::Grid::new("steps")
                .num_columns(3)
                .striped(true)
                .spacing(egui::vec2(12.0, 2.0))
                .show(ui, |ui| {
                    for heading in ["Step", "Result", "What happened; its vehicle.yaml lines"] {
                        ui.strong(heading);
                    }
                    ui.end_row();
                    for step in &report.steps {
                        let is_chosen = chosen.is_some_and(|c| c.name == step.name);
                        let name = format!("{} {}", step.number, step.name);
                        if ui
                            .selectable_label(is_chosen, name)
                            .on_hover_text("Its numbers and its graph, below")
                            .clicked()
                        {
                            actions.push(Action::Step(step.name.clone()));
                        }
                        // In a row of its own height: a cell would stretch the pill.
                        ui.horizontal(|ui| status_pill(ui, step.status));
                        let short: String = step.detail.chars().take(DETAIL_CHARS).collect();
                        let cut = short.len() < step.detail.len();
                        let shown = if cut { format!("{short}...") } else { short };
                        ui.vertical(|ui| {
                            ui.label(shown).on_hover_text(&step.detail);
                            if !step.yaml.is_empty() {
                                let lines = egui::RichText::new(step.yaml.join("; "));
                                ui.label(lines.monospace().size(13.0));
                            }
                        });
                        ui.end_row();
                    }
                });
        });
}

/// The chosen step: what happened in full, its graph, and its numbers beside it.
fn step_detail(ui: &mut egui::Ui, step: &StepRow) {
    ui.add_space(6.0);
    ui.horizontal_wrapped(|ui| {
        ui.strong(format!("{} {}", step.number, step.name));
        status_pill(ui, step.status);
    });
    ui.add(egui::Label::new(&step.detail).wrap());
    match &step.graph {
        Some(graph) => self::graph(ui, graph),
        None => {
            ui.label(
                egui::RichText::new("No graph: this step keeps no log of speed or steering.")
                    .color(GREY),
            );
        }
    }
    if !step.numbers.is_empty() {
        ui.strong("Its numbers");
        egui::Grid::new("numbers")
            .num_columns(4)
            .striped(true)
            .spacing(egui::vec2(14.0, 4.0))
            .show(ui, |ui| {
                for heading in ["What", "Measured", "vehicle.yaml now", "Note"] {
                    ui.strong(heading);
                }
                ui.end_row();
                for number in &step.numbers {
                    ui.label(&number.what);
                    ui.strong(number.measured());
                    ui.label(&number.now);
                    ui.label(egui::RichText::new(&number.note).color(GREY));
                    ui.end_row();
                }
            });
    }
}

/// A hallway session's fitted numbers, each with its error bar and whether to trust it.
fn fits_table(ui: &mut egui::Ui, fits: &[Fit]) {
    ui.add_space(6.0);
    ui.strong("The session's fits (mean ± the passes' spread)");
    egui::Grid::new("fits")
        .num_columns(4)
        .striped(true)
        .spacing(egui::vec2(14.0, 4.0))
        .show(ui, |ui| {
            for heading in ["Number", "Measured", "vehicle.yaml now", "Trust"] {
                ui.strong(heading);
            }
            ui.end_row();
            for fit in fits {
                ui.label(&fit.name);
                ui.strong(fit.measured());
                ui.label(fit.now.map_or_else(String::new, |now| format!("{now}")));
                let colour = if fit.trusted { BRIGHT } else { AMBER };
                ui.label(egui::RichText::new(fit.trust()).color(colour));
                ui.end_row();
            }
        });
}

/// One graph: commanded (dashed) against measured (solid), over the run's time. One value
/// axis; the axes and the legend say what they are in words; hover reads both lines.
fn graph(ui: &mut egui::Ui, graph: &Graph) {
    let (Some(time), Some(values)) = (graph.time(), graph.values()) else {
        ui.label(egui::RichText::new("No graph: the log has no points.").color(GREY));
        return;
    };
    // The legend: a sample of each line, then its name in the text's own colour.
    ui.horizontal(|ui| {
        for (words, colour, dashed) in [
            (Graph::COMMANDED, COMMANDED, true),
            (Graph::MEASURED, MEASURED, false),
        ] {
            let (sample, _) = ui.allocate_exact_size(egui::vec2(44.0, 14.0), egui::Sense::hover());
            let ends = [sample.left_center(), sample.right_center()];
            let stroke = egui::Stroke::new(2.5, colour);
            if dashed {
                ui.painter()
                    .extend(egui::Shape::dashed_line(&ends, stroke, 8.0, 5.0));
            } else {
                ui.painter().line_segment(ends, stroke);
            }
            ui.label(words);
            ui.add_space(14.0);
        }
        if let Some((when, gap)) = graph.widest_gap() {
            ui.label(
                egui::RichText::new(format!("widest gap {gap:.2} {} at {when:.1} s", graph.unit))
                    .color(GREY),
            );
        }
    });
    let size = egui::vec2(ui.available_width().clamp(360.0, 820.0), 230.0);
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::hover());
    let plot = egui::Rect::from_min_max(
        rect.min + egui::vec2(58.0, 26.0),
        rect.max - egui::vec2(18.0, 44.0),
    );
    let painter = ui.painter_at(rect);
    let font = egui::FontId::proportional(14.0);
    #[allow(clippy::cast_possible_truncation)]
    let place = |t: f64, v: f64| {
        egui::pos2(
            plot.left() + plot.width() * time.share(t) as f32,
            plot.bottom() - plot.height() * values.share(v) as f32,
        )
    };
    // Recessive grid, round numbers on both axes.
    for v in values.ticks(4) {
        let y = place(time.low, v).y;
        painter.line_segment(
            [egui::pos2(plot.left(), y), egui::pos2(plot.right(), y)],
            egui::Stroke::new(1.0, GRID),
        );
        painter.text(
            egui::pos2(plot.left() - 8.0, y),
            egui::Align2::RIGHT_CENTER,
            tick_label(v),
            font.clone(),
            GREY,
        );
    }
    for t in time.ticks(6) {
        let x = place(t, values.low).x;
        painter.line_segment(
            [
                egui::pos2(x, plot.bottom()),
                egui::pos2(x, plot.bottom() + 5.0),
            ],
            egui::Stroke::new(1.0, GREY),
        );
        painter.text(
            egui::pos2(x, plot.bottom() + 8.0),
            egui::Align2::CENTER_TOP,
            tick_label(t),
            font.clone(),
            GREY,
        );
    }
    painter.line_segment(
        [plot.left_bottom(), plot.right_bottom()],
        egui::Stroke::new(1.0, GREY),
    );
    painter.text(
        egui::pos2(rect.left(), rect.top() + 2.0),
        egui::Align2::LEFT_TOP,
        graph.value_label(),
        font.clone(),
        BRIGHT,
    );
    painter.text(
        egui::pos2(plot.center().x, rect.bottom() - 2.0),
        egui::Align2::CENTER_BOTTOM,
        Graph::TIME_LABEL,
        font,
        BRIGHT,
    );
    let line = |values: &[f64]| -> Vec<egui::Pos2> {
        graph.points(values).map(|(t, v)| place(t, v)).collect()
    };
    painter.extend(egui::Shape::dashed_line(
        &line(&graph.commanded),
        egui::Stroke::new(2.0, COMMANDED),
        8.0,
        5.0,
    ));
    painter.add(egui::Shape::line(
        line(&graph.measured),
        egui::Stroke::new(2.0, MEASURED),
    ));
    // Hover: a crosshair at the nearest sample, and both lines' values in words.
    let Some(pointer) = response.hover_pos().filter(|p| plot.contains(*p)) else {
        return;
    };
    let nearest = graph
        .t
        .iter()
        .enumerate()
        .map(|(i, &t)| (i, (place(t, values.low).x - pointer.x).abs()))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i);
    let read = nearest.and_then(|i| {
        Some((
            *graph.t.get(i)?,
            *graph.commanded.get(i)?,
            *graph.measured.get(i)?,
        ))
    });
    if let Some((t, asked, got)) = read {
        let x = place(t, values.low).x;
        painter.line_segment(
            [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
            egui::Stroke::new(1.0, GREY),
        );
        response.on_hover_text_at_pointer(format!(
            "{t:.2} s\ncommanded {asked:.3} {unit}\nmeasured {got:.3} {unit}",
            unit = graph.unit
        ));
    }
}

/// One side's newest result, and what keeps it shut.
fn side(ui: &mut egui::Ui, side: &Side) {
    cell(ui, &Cell::of(side));
    if side.state != State::NotRun {
        for (name, words) in [
            ("Last result", &side.when),
            ("Commit", &side.commit),
            ("Report", &side.report),
            ("Override", &side.override_reason),
        ] {
            if !words.is_empty() {
                ui.horizontal_wrapped(|ui| {
                    ui.strong(format!("{name}:"));
                    ui.label(words);
                });
            }
        }
    }
    for why in &side.blocked_by {
        ui.label(egui::RichText::new(format!("Shut: {why}")).color(AMBER));
    }
}

fn record_form(ui: &mut egui::Ui, session: &mut Session, rung: &Rung, actions: &mut Vec<Action>) {
    ui.heading("Record by hand");
    ui.label(format!(
        "For a test a person ran: it goes to the tool, which keeps it for {}.",
        rung.id
    ));
    let form = &mut session.form;
    ui.horizontal(|ui| {
        ui.strong("Result");
        ui.radio_value(&mut form.outcome, Outcome::Pass, "pass");
        ui.radio_value(&mut form.outcome, Outcome::Fail, "fail");
        ui.add_space(24.0);
        ui.strong("Where");
        for place in Place::ALL {
            ui.radio_value(&mut form.place, place, place.heading());
        }
    });
    egui::Grid::new("form").num_columns(2).show(ui, |ui| {
        ui.strong("Note");
        ui.add(
            egui::TextEdit::singleline(&mut form.note)
                .hint_text("what was seen: 2.5 m/s, dry")
                .desired_width(520.0),
        );
        ui.end_row();
        ui.strong("Override");
        ui.add(
            egui::TextEdit::singleline(&mut form.override_reason)
                .hint_text("why a shut rung is passed anyway")
                .desired_width(520.0),
        );
        ui.end_row();
    });
    if form.override_expected(rung) && form.override_reason.trim().is_empty() {
        ui.label(
            egui::RichText::new("This side is shut: the tool wants an override reason for a pass.")
                .color(AMBER),
        );
    }
    let busy = session.is_recording();
    ui.horizontal_wrapped(|ui| {
        if ui.add_enabled(!busy, egui::Button::new("Record")).clicked() {
            actions.push(Action::Record);
        }
        if busy {
            ui.label("recording...");
        }
        if let Some(notice) = session.recorded() {
            said(ui, notice, "Recorded", "REFUSED");
        }
    });
}

/// A notice with its kind in words, not only in colour.
fn said(ui: &mut egui::Ui, notice: &Notice, good: &str, bad: &str) {
    let (word, colour) = if notice.good {
        (good, GREEN)
    } else {
        (bad, RED)
    };
    let text = if notice.text.starts_with(word) {
        notice.text.clone()
    } else {
        format!("{word}: {}", notice.text)
    };
    ui.label(egui::RichText::new(text).strong().color(colour));
}

/// The run's last lines, its state, and Stop.
fn console(ui: &mut egui::Ui, session: &Session, actions: &mut Vec<Action>) {
    ui.horizontal_wrapped(|ui| {
        ui.strong("CONSOLE");
        match session.running() {
            Some(running) => {
                ui.spinner();
                let what = match running.test {
                    Some(test) => format!("{}, {test} alone", running.rung),
                    None => running.rung.to_owned(),
                };
                let step = running.step.map_or_else(String::new, |s| format!(" {s}"));
                ui.label(
                    egui::RichText::new(format!("{what}: running...{step}"))
                        .strong()
                        .color(AMBER),
                );
                let words = if running.stop_asked {
                    "Stop harder (kill)"
                } else {
                    "Stop"
                };
                if ui.button(words).clicked() {
                    actions.push(Action::Stop);
                }
            }
            None => match session.last_run() {
                Some(notice) => said(ui, notice, "DONE", "NOT PASSED"),
                None => {
                    ui.label(egui::RichText::new("nothing is running").color(GREY));
                }
            },
        }
    });
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for line in session.console() {
                ui.label(egui::RichText::new(line).monospace());
            }
        });
}

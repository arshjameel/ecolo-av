//! The test ladder's window, drawn in egui: the ladder on the left, the chosen rung on the
//! right, the run's console along the bottom. What each part says is `ladder-core`'s.
//!
//! The car's side has words and a Copy button, and nothing that runs. The one switch that
//! moves anything is "The simulated car": Ravensim's jacks, on only while a simulator runs
//! on this computer (`ladder-core`'s `jacks`, through `pixi run jacks`).

use bevy_egui::egui;
use ladder_core::{
    session::Notice, show::CAR_RULE, CarSide, Cell, Jacks, Ladder, Outcome, Place, RunButton, Rung,
    Session, Side, Source, State,
};

const GREEN: egui::Color32 = egui::Color32::from_rgb(45, 210, 110);
const RED: egui::Color32 = egui::Color32::from_rgb(240, 85, 85);
const AMBER: egui::Color32 = egui::Color32::from_rgb(240, 190, 70);
const GREY: egui::Color32 = egui::Color32::from_gray(165);
/// A shut cell: darker than "not run", and it says "shut".
const SHUT: egui::Color32 = egui::Color32::from_gray(95);
const DARK: egui::Color32 = egui::Color32::from_gray(12);
const BRIGHT: egui::Color32 = egui::Color32::from_gray(235);

/// What a press asks for; done after the frame's drawing, when nothing is borrowed.
enum Action {
    Select(String),
    Run,
    Stop,
    Record,
    Jacks,
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
        .default_size(170.0)
        .resizable(true)
        .show(&mut root, |ui| console(ui, session, &mut actions));
    egui::Panel::left("ladder")
        .default_size(640.0)
        .resizable(true)
        .show(&mut root, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| match &ladder {
                Some(ladder) => rungs(ui, ladder, chosen.as_ref(), &mut actions),
                None => {
                    ui.label("Reading the ladder...");
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
    egui::Grid::new("about")
        .num_columns(2)
        .spacing(egui::vec2(18.0, 8.0))
        .show(ui, |ui| {
            for (name, words) in [
                ("Where", &rung.location),
                ("It measures", &rung.measures),
                ("It passes when", &rung.passes),
                ("How, by hand", &rung.how),
            ] {
                ui.strong(name);
                ui.add(egui::Label::new(words).wrap());
                ui.end_row();
            }
        });

    ui.separator();
    ui.heading(Place::Sim.heading());
    side(ui, "sim", &rung.sim);
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

    ui.separator();
    ui.heading(Place::Car.heading());
    side(ui, "car", &rung.car);
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
}

/// One side's newest result, and what keeps it shut.
fn side(ui: &mut egui::Ui, id: &str, side: &Side) {
    cell(ui, &Cell::of(side));
    if side.state != State::NotRun {
        egui::Grid::new(("last", id))
            .num_columns(2)
            .spacing(egui::vec2(18.0, 6.0))
            .show(ui, |ui| {
                for (name, words) in [
                    ("Last result", &side.when),
                    ("Commit", &side.commit),
                    ("Report", &side.report),
                    ("Override", &side.override_reason),
                ] {
                    if !words.is_empty() {
                        ui.strong(name);
                        ui.add(egui::Label::new(words).wrap());
                        ui.end_row();
                    }
                }
            });
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
                ui.label(
                    egui::RichText::new(format!("{}: running...", running.rung))
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

//! The hardware panel as a page of a window: `hw_panel` fills its own window with it;
//! the map window shows it as its HARDWARE page (F2, or the button).
//!
//! Each frame, in this order:
//!
//! ```text
//! news(now)            what arrived; a source change zeroes the buttons; the gate
//! read_input(rl, now)  the keys and the mouse (only while the page shows)
//! step(now, input)     the buttons and the calibration page: what to send now
//! draw(d, &step)       one frame (only while the page shows)
//! frame_drawn(&step)   to the node: the command, under hud-core's heartbeat
//! ```
//!
//! While the page is hidden its host still calls `news`, `step` with [`Input::default`]
//! and `frame_drawn`: no key, no click and no GO reach it, so the buttons send at most
//! their 0.5 s of zeros, and hiding the page ends the calibration page's mode as leaving
//! that page does ([`HardwarePage::hide`]). The e-stop button and its key work only
//! while the page shows.

use std::{path::PathBuf, time::Instant};

use anyhow::{Context as _, Result};
use hud_core::robot::{EasterEgg, EggSettings, Show};
use panel_core::{
    buttons::{Buttons, Command, Press, Steps, World, Zeroed},
    calibration::{self, CalPage, CalPress, NotReady, Request},
    config::PanelConfig,
    controls::{Action, Controls},
    easter_egg::{self, PassWatch},
    gate::{self, Caps, Gate, Situation},
    graphs::Freezer,
    measure::{self, Entry, Odometer},
    rules::Kind,
    state::{self as panel_state, PanelState, Update},
    vehicle::Vehicle,
};
use raylib::prelude::*;

use crate::{
    ladder::LadderTab,
    robot,
    source::DataSource,
    ui::{self, Button, Frame, Page, Ui},
};

/// raylib's keys for each press.
pub const PRESS_KEYS: [(KeyboardKey, Press); 10] = [
    (KeyboardKey::KEY_UP, Press::SpeedUp),
    (KeyboardKey::KEY_EQUAL, Press::SpeedUp),
    (KeyboardKey::KEY_KP_ADD, Press::SpeedUp),
    (KeyboardKey::KEY_DOWN, Press::SpeedDown),
    (KeyboardKey::KEY_MINUS, Press::SpeedDown),
    (KeyboardKey::KEY_KP_SUBTRACT, Press::SpeedDown),
    (KeyboardKey::KEY_LEFT, Press::Left),
    (KeyboardKey::KEY_RIGHT, Press::Right),
    (KeyboardKey::KEY_C, Press::Centre),
    (KeyboardKey::KEY_S, Press::Stop),
];

/// raylib's keys for the calibration page's presses.
pub const CAL_KEYS: [(KeyboardKey, CalPress); 9] = [
    (KeyboardKey::KEY_L, CalPress::Limp),
    (KeyboardKey::KEY_H, CalPress::Hold),
    (KeyboardKey::KEY_LEFT, CalPress::StepLeft),
    (KeyboardKey::KEY_RIGHT, CalPress::StepRight),
    (
        KeyboardKey::KEY_ONE,
        CalPress::Mark(panel_core::steering_limits::Mark::Left),
    ),
    (
        KeyboardKey::KEY_TWO,
        CalPress::Mark(panel_core::steering_limits::Mark::Centre),
    ),
    (
        KeyboardKey::KEY_THREE,
        CalPress::Mark(panel_core::steering_limits::Mark::Right),
    ),
    (KeyboardKey::KEY_K, CalPress::Check),
    (KeyboardKey::KEY_O, CalPress::End),
];

/// The key that flips the map window between its MAP and HARDWARE pages.
pub const FLIP_KEY: KeyboardKey = KeyboardKey::KEY_F2;

/// Which window the page is in: panel-core's, as it decides what each host may do
/// ([`Controls::for_host`]).
pub use panel_core::controls::Host;

/// The panel between the source and the window: news in, where it may drive, the
/// buttons, what to send. The window adds the keys, the mouse and the drawing; the
/// Ravensim check drives it without a window.
pub struct Panel {
    state: PanelState,
    buttons: Buttons,
    last_kind: Option<Kind>,
    caps: Caps,
    vehicle: Vehicle,
    stands: bool,
    page: Page,
    cal: CalPage,
    /// The calibration result's file, once saved (or why it wasn't).
    saved: Option<String>,
    /// The GRAPHS page's FREEZE.
    freezer: Freezer,
    /// The MEASURE page: distance since RESET, the tape typed in, the file once saved.
    odometer: Odometer,
    tape: Entry,
    measure_saved: Option<String>,
    /// `hw_panel`'s own window, or the map's HARDWARE page: with the source, what a
    /// person may do here ([`Self::controls`]).
    host: Host,
}

impl Panel {
    pub fn new(config: &PanelConfig, vehicle: &Vehicle, stands: bool, host: Host) -> Self {
        Self {
            state: PanelState::new(config.stale_after()),
            buttons: Buttons::new(Steps {
                speed: config.speed_step,
                steer: config.steer_step,
                creep: config.creep,
                max_steer: vehicle.max_steering_angle,
                wheelbase: vehicle.wheelbase,
            }),
            last_kind: None,
            caps: Caps {
                sim: config.sim_cap,
                stands: config.stands_cap,
            },
            vehicle: vehicle.clone(),
            stands,
            page: Page::Drive,
            cal: CalPage::new(config.calibration()),
            saved: None,
            freezer: Freezer::default(),
            odometer: Odometer::default(),
            tape: Entry::default(),
            measure_saved: None,
            host,
        }
    }

    /// What a person may do here now: the map's page on the real car (or nothing
    /// announced) only watches (Ben, 26 Nov, option A).
    pub fn controls(&self) -> Controls {
        Controls::for_host(self.host, self.state.kind())
    }

    /// What the panel has heard.
    pub fn state(&self) -> &PanelState {
        &self.state
    }

    pub fn buttons(&self) -> &Buttons {
        &self.buttons
    }

    pub fn cal(&self) -> &CalPage {
        &self.cal
    }

    pub fn page(&self) -> Page {
        self.page
    }

    /// Opens a page. Leaving the calibration page ends what it asks.
    pub fn show(&mut self, page: Page, now: Instant) {
        if self.page == Page::Calibration && page != Page::Calibration {
            self.end_calibration(now);
        }
        self.page = page;
    }

    /// What leaving the calibration page does: its mode ends, and a person's ON STANDS,
    /// STILL is forgotten (it holds only while they stay on the page).
    fn end_calibration(&mut self, now: Instant) {
        let position = self.position(now);
        self.cal.press(CalPress::End, now, position, &[]);
        self.cal.forget_still();
    }

    /// The steering actuator's position, while fresh.
    pub fn position(&self, now: Instant) -> Option<u16> {
        self.state.fresh_raw(now).and_then(|r| r.position)
    }

    /// What the calibration page's checks rest on now.
    fn cal_situation(&self, now: Instant) -> calibration::Situation {
        self.state
            .calibration_situation(now, self.cal.still_confirmed())
    }

    /// Why the calibration page may not ask now.
    pub fn cal_ready(&self, now: Instant) -> Vec<NotReady> {
        calibration::readiness(&self.cal_situation(now))
    }

    /// This frame's calibration presses; returns what to send to the bridge now.
    pub fn calibrate(
        &mut self,
        now: Instant,
        presses: impl IntoIterator<Item = CalPress>,
    ) -> Option<Request> {
        let ready = self.cal_ready(now);
        let position = self.position(now);
        // Watch-only: no press reaches the page (news has already ended its mode).
        let allowed = self.controls().allows(Action::Calibrate);
        for p in presses.into_iter().filter(|_| allowed) {
            self.cal.press(p, now, position, &ready);
        }
        self.cal.tick(now, self.state.fresh_raw(now), &ready)
    }

    /// SAVE: the result for a person to copy, in a file of its own; never vehicle.yaml.
    fn save(&mut self) {
        let (Some(Ok(limits)), Some(marks)) = (self.cal.limits(), self.cal.marks()) else {
            return;
        };
        let text = calibration::result_file(&limits, marks, self.cal.checked());
        self.saved = Some(match write_result(CALIBRATION_DIR, "steering", &text) {
            Ok(path) => {
                println!("hw_panel: the steering calibration, saved to {path}:\n{text}");
                path
            }
            Err(e) => {
                eprintln!("hw_panel: couldn't save the steering calibration: {e:#}\n{text}");
                format!("NOT saved ({e}); the lines are in the log")
            }
        });
    }

    /// The MEASURE page's RESET: the odometer and the tape back to zero.
    fn reset_measure(&mut self) {
        self.odometer.reset();
        self.tape.clear();
        self.measure_saved = None;
    }

    /// The MEASURE page's SAVE: what was measured, in a file of its own; never vehicle.yaml.
    fn save_measure(&mut self) {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        let when = calibration::utc_stamp(secs);
        let diameter = self.vehicle.wheel_diameter;
        let text = measure::report(&self.odometer, self.tape.value(), diameter, &when);
        self.measure_saved = Some(match write_result(MEASURE_DIR, "measure", &text) {
            Ok(path) => {
                println!("hw_panel: the measurements, saved to {path}:\n{text}");
                path
            }
            Err(e) => {
                eprintln!("hw_panel: couldn't save the measurements: {e:#}\n{text}");
                format!("NOT saved ({e}); the lines are in the log")
            }
        });
    }

    /// Takes what arrived; a source change zeroes the buttons (not the first
    /// announcement). Returns where the speed buttons may drive now.
    pub fn news(&mut self, source: &mut dyn DataSource, now: Instant) -> Gate {
        for update in source.drain(now) {
            if let Update::Mux(c) = &update {
                self.buttons.observe_mux(*c, now);
            }
            self.state.apply(update, now);
        }
        // An e-stop freezes the graphs, to read what led to it.
        self.freezer
            .watch_estop(now, self.state.estop_on(), &self.state.graphs);
        // Unknown to anything is the first announcement, not a change.
        let kind = self.state.kind();
        if self
            .last_kind
            .is_some_and(|k| k != kind && k != Kind::Unknown)
        {
            self.buttons.zero(Zeroed::SourceChanged);
        }
        self.last_kind = Some(kind);
        let controls = self.controls();
        if controls.is_watch_only() && self.cal.is_asking() {
            // Watch-only now (the source went REAL): end what the calibration page asks,
            // as leaving it does.
            self.end_calibration(now);
        }
        if !controls.allows(Action::Drive) {
            return Gate::Off(gate::Off::WatchOnly);
        }
        gate::gate(
            &Situation {
                kind,
                stands: self.stands,
                speed_wheels: self.vehicle.speed_wheels.as_ref(),
                no_timeout_lock: self.state.no_timeout_lock,
                // Not read yet (a parameter of the legacy bridge): unknown.
                speed_loop_mode: None,
                speed_seen: self.state.seen_speed(now).is_some(),
            },
            self.caps,
        )
    }

    /// This frame's presses and GO; returns what to send now.
    pub fn act(
        &mut self,
        now: Instant,
        gate: &Gate,
        presses: impl IntoIterator<Item = Press>,
        go: bool,
    ) -> Option<Command> {
        // The calibration page has no drive buttons: GO is never held there. Watch-only:
        // none anywhere.
        let drive = self.page == Page::Drive && self.controls().allows(Action::Drive);
        for p in presses.into_iter().filter(|_| drive) {
            self.buttons.press(p, now, gate);
        }
        self.buttons.set_go(go && drive, now);
        self.buttons.command(
            now,
            &World {
                gate,
                estop: self.state.estop_on(),
                seen_steer: self.state.seen_steer(now),
                seen_speed: self.state.seen_speed(now),
            },
        )
    }

    /// "front, not driven" on the real car's stands.
    fn wheels_note(&self) -> Option<String> {
        panel_state::wheels_note(
            self.stands,
            self.state.kind(),
            self.vehicle.speed_wheels.as_ref(),
        )
    }
}

/// One frame's keys and mouse, from [`HardwarePage::read_input`]; or none at all
/// ([`Input::default`]) while the page is hidden. A host's scripts (`--self-test`,
/// `--cal-self-test`) add their presses here, through the same rules.
#[derive(Debug, Clone, Default)]
pub struct Input {
    pub presses: Vec<Press>,
    pub cal_presses: Vec<CalPress>,
    /// GO held: Space or the button, in a window with focus.
    pub go: bool,
    /// The on-screen button the mouse holds down.
    held: Option<Button>,
    /// The calibration page's SAVE.
    save: bool,
    /// Any key, click or move: hides the little robot.
    touched: bool,
    /// The MAP button, or its key ([`Host::Map`] only): back to the map.
    pub leave: bool,
    /// The page shows this frame (from [`HardwarePage::read_input`]): the LADDER tab asks
    /// its tool only then, or while a run it started goes on.
    shown: bool,
}

/// What one frame decided: what to send, and what to draw.
#[derive(Debug, Clone)]
pub struct Step {
    pub command: Option<Command>,
    pub calibration: Option<Request>,
    held: Option<Button>,
    show: Show,
}

/// The whole hardware panel, ready for a window: [`Panel`], its source, its drawing and
/// the little robot.
pub struct HardwarePage {
    panel: Panel,
    ui: Ui,
    source: Box<dyn DataSource>,
    host: Host,
    config: PanelConfig,
    egg: EasterEgg,
    passes: PassWatch,
    /// Where the speed buttons may drive, from this frame's news.
    gate: Gate,
    /// The LADDER tab: the test ladder's session (pretend with the demo car), or why
    /// there is none here.
    ladder: LadderTab,
}

impl HardwarePage {
    /// Needs the window open (it loads the font). `egg`: the little robot's settings
    /// (`config.easter_egg_settings()`, or changed by `--robot-now`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        rl: &mut RaylibHandle,
        thread: &RaylibThread,
        source: Box<dyn DataSource>,
        config: PanelConfig,
        vehicle: &Vehicle,
        stands: bool,
        host: Host,
        egg: EggSettings,
        now: Instant,
    ) -> Self {
        let ui = Ui::new(rl, thread, config.theme.theme());
        let mut egg = EasterEgg::new(egg, now);
        // It drives across once at start-up, then peeks when all is quiet.
        egg.drive_across(now);
        let ladder = LadderTab::open(source.is_pretend());
        Self {
            panel: Panel::new(&config, vehicle, stands, host),
            ui,
            source,
            host,
            config,
            egg,
            passes: PassWatch::default(),
            gate: Gate::Off(gate::Off::Unannounced),
            ladder,
        }
    }

    /// The LADDER tab's state.
    pub fn ladder(&self) -> &LadderTab {
        &self.ladder
    }

    pub fn panel(&self) -> &Panel {
        &self.panel
    }

    /// Where the speed buttons may drive, as of the last [`Self::news`].
    pub fn gate(&self) -> &Gate {
        &self.gate
    }

    /// Opens one of the panel's own pages (DRIVE, STEERING CAL, GRAPHS, MEASURE, LADDER).
    pub fn show(&mut self, page: Page, now: Instant) {
        self.panel.show(page, now);
    }

    /// The host hides the page: the calibration page's mode ends, as on leaving it.
    /// The page stays where it was, for when it shows again.
    pub fn hide(&mut self, now: Instant) {
        if self.panel.page == Page::Calibration {
            self.panel.end_calibration(now);
        }
    }

    /// "ROS" or "DEMO".
    pub fn label(&self) -> &'static str {
        self.source.label()
    }

    /// Whether the source has stopped (Ctrl-C, or the ROS node failed).
    pub fn has_stopped(&self) -> bool {
        self.source.has_stopped()
    }

    /// Takes what arrived. Every frame, shown or not.
    pub fn news(&mut self, now: Instant) {
        self.gate = self.panel.news(self.source.as_mut(), now);
    }

    /// The keys and the mouse, while the page shows. Tabs, FREEZE, the tape, RESET,
    /// the measure page's SAVE and the E-STOP act at once; the rest is returned.
    pub fn read_input(&mut self, rl: &mut RaylibHandle, now: Instant) -> Input {
        let panel = &mut self.panel;
        // Any touch at all hides the little robot.
        let touched = rl.get_key_pressed().is_some()
            || [
                MouseButton::MOUSE_BUTTON_LEFT,
                MouseButton::MOUSE_BUTTON_RIGHT,
            ]
            .into_iter()
            .any(|b| rl.is_mouse_button_down(b))
            || rl.get_mouse_delta().length() > 0.0
            || rl.get_mouse_wheel_move() != 0.0;

        // A window without focus never holds GO.
        let focused = rl.is_window_focused();
        let w = rl.get_screen_width() as f32;
        let mouse = rl.get_mouse_position();
        let under = ui::layout(w, panel.page, self.host)
            .into_iter()
            .find(|(_, r)| r.check_collision_point_rec(mouse))
            .map(|(b, _)| b);
        let mouse_down = rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT);
        let held = under.filter(|_| mouse_down);
        let on_cal = panel.page == Page::Calibration;
        let on_measure = panel.page == Page::Measure;
        // The measure page types numbers: the drive keys are off there.
        let mut presses: Vec<Press> = PRESS_KEYS
            .iter()
            .filter(|(k, _)| !on_cal && !on_measure && rl.is_key_pressed(*k))
            .map(|&(_, p)| p)
            .collect();
        let mut cal_presses: Vec<CalPress> = CAL_KEYS
            .iter()
            .filter(|(k, _)| on_cal && rl.is_key_pressed(*k))
            .map(|&(_, p)| p)
            .collect();
        let clicked = under.filter(|_| rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT));
        match clicked {
            Some(Button::Press(p)) => presses.push(p),
            Some(Button::Cal(p)) => cal_presses.push(p),
            Some(Button::Tab(page)) => panel.show(page, now),
            _ => {}
        }
        if rl.is_key_pressed(KeyboardKey::KEY_TAB) {
            panel.show(panel.page.next(), now);
        }
        let on_graphs = panel.page == Page::Graphs;
        if on_graphs && (clicked == Some(Button::Freeze) || rl.is_key_pressed(KeyboardKey::KEY_P)) {
            panel.freezer.toggle(now, &panel.state.graphs);
        }
        let save_key = clicked == Some(Button::Save) || rl.is_key_pressed(KeyboardKey::KEY_W);
        // The measure page: the tape typed in, RESET, SAVE.
        while let Some(c) = rl.get_char_pressed() {
            if on_measure {
                panel.tape.push(c);
            }
        }
        if on_measure {
            if rl.is_key_pressed(KeyboardKey::KEY_BACKSPACE) {
                panel.tape.pop();
            }
            if clicked == Some(Button::Reset) || rl.is_key_pressed(KeyboardKey::KEY_R) {
                panel.reset_measure();
            }
            if save_key {
                panel.save_measure();
            }
        }
        // The ladder page: a rung, RUN, STOP, the jacks, the answer's views. ladder-core
        // decides whether each may act; Up and Down choose a rung.
        // Watch-only: no RUN and no jacks; STOP still stops a run this tab started.
        if panel.page == Page::Ladder {
            let controls = panel.controls();
            let ladder = &mut self.ladder;
            match clicked {
                Some(Button::Rung(row)) => ladder.select_row(row),
                Some(Button::RunRung) if controls.allows(Action::RunLadder) => {
                    ladder.press_run();
                }
                Some(Button::StopRun) if controls.allows(Action::StopOwnRun) => {
                    ladder.press_stop();
                }
                Some(Button::Jacks) if controls.allows(Action::Jacks) => ladder.press_jacks(),
                Some(Button::View(i)) => ladder.show_view(i),
                _ => {}
            }
            if rl.is_key_pressed(KeyboardKey::KEY_UP) {
                ladder.select_next(-1);
            }
            if rl.is_key_pressed(KeyboardKey::KEY_DOWN) {
                ladder.select_next(1);
            }
        }
        if clicked == Some(Button::Estop) || rl.is_key_pressed(KeyboardKey::KEY_E) {
            self.source.press_estop();
        }
        let go = focused && (rl.is_key_down(KeyboardKey::KEY_SPACE) || held == Some(Button::Go));
        let leave = self.host == Host::Map
            && (clicked == Some(Button::Leave) || rl.is_key_pressed(FLIP_KEY));
        Input {
            presses,
            cal_presses,
            go,
            held,
            save: on_cal && save_key,
            touched,
            leave,
            shown: true,
        }
    }

    /// The buttons and the calibration page under the rules: what to send now. Every
    /// frame, shown or not (hidden: with [`Input::default`]).
    pub fn step(&mut self, now: Instant, input: Input) -> Step {
        let panel = &mut self.panel;
        // The odometer counts on every page.
        let seen = panel.state.seen_speed(now);
        panel.odometer.update(seen, now);
        let command = panel.act(now, &self.gate, input.presses, input.go);
        let calibration = panel.calibrate(now, input.cal_presses);
        if input.save && panel.controls().allows(Action::Calibrate) {
            panel.save();
        }
        self.ladder
            .tick(now, input.shown && panel.page == Page::Ladder);
        if self.passes.just_passed(&panel.cal) {
            self.egg.drive_across(now);
        }
        let page = match panel.page {
            Page::Drive => 0,
            Page::Calibration => 1,
            Page::Graphs => 2,
            Page::Measure => 3,
            Page::Ladder => 4,
        };
        let watch = easter_egg::watch(
            &panel.state,
            &panel.buttons,
            &self.gate,
            &panel.cal,
            page,
            input.touched,
            now,
        );
        let show = self.egg.update(now, watch);
        Step {
            command,
            calibration,
            held: input.held,
            show,
        }
    }

    /// Draws the page over the whole window.
    pub fn draw(&self, d: &mut RaylibDrawHandle, now: Instant, step: &Step) {
        let panel = &self.panel;
        let cal_allowance = calibration::allowance(&panel.cal_situation(now));
        let note = panel.wheels_note();
        let frame = Frame {
            now,
            label: self.source.label(),
            gate: &self.gate,
            buttons: &panel.buttons,
            held: step.held,
            wheels_note: note.as_deref(),
            stands: panel.stands,
            page: panel.page,
            host: self.host,
            cal: &panel.cal,
            cal_allowance: &cal_allowance,
            saved: panel.saved.as_deref(),
            hide_keys: matches!(step.show, Show::Drive { .. }),
            freezer: &panel.freezer,
            stale_after: self.config.stale_after(),
            odometer: &panel.odometer,
            tape: &panel.tape,
            wheel_diameter: panel.vehicle.wheel_diameter,
            measure_saved: panel.measure_saved.as_deref(),
            ladder: &self.ladder,
            controls: panel.controls(),
        };
        let room = self.ui.draw(d, &panel.state, &frame);
        robot::draw(d, self.config.theme.theme(), step.show, &room);
    }

    /// Hands the frame's command and calibration request to the node: they go out only
    /// while frames keep coming (hud-core's heartbeat).
    pub fn frame_drawn(&mut self, step: &Step) {
        self.source.frame_drawn(step.command, step.calibration);
    }
}

/// The calibration page's SAVE folder, under `$XDG_DATA_HOME/aav/`.
const CALIBRATION_DIR: &str = "steering-calibration";
/// The measure page's.
const MEASURE_DIR: &str = "measurements";

/// Where SAVE writes: `$XDG_DATA_HOME/aav/<folder>/`, or `~/.local/share/aav/<folder>/`.
fn results_dir(folder: &str) -> Result<PathBuf> {
    let base = match std::env::var_os("XDG_DATA_HOME").filter(|v| !v.is_empty()) {
        Some(dir) => PathBuf::from(dir),
        None => {
            PathBuf::from(std::env::var_os("HOME").context("HOME isn't set")?).join(".local/share")
        }
    };
    Ok(base.join("aav").join(folder))
}

/// Writes `text` to a new file of its own, `<prefix>-<time>.txt`; returns its path.
fn write_result(folder: &str, prefix: &str, text: &str) -> Result<String> {
    let dir = results_dir(folder)?;
    std::fs::create_dir_all(&dir).with_context(|| format!("making {}", dir.display()))?;
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let path = dir.join(format!("{prefix}-{}.txt", calibration::utc_stamp(secs)));
    std::fs::write(&path, text).with_context(|| format!("writing {}", path.display()))?;
    Ok(path.display().to_string())
}

/// How long [`Panel::calibrate`]'s END keeps sending MODE_OFF: the tests' bound.
#[cfg(test)]
const END_FOR: std::time::Duration = std::time::Duration::from_secs(1);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{cal_script::CalScript, demo::Demo};
    use panel_core::gate::Off;
    use std::time::Duration;

    /// A source that hands out the updates it is given, once.
    pub(crate) struct Script(pub Vec<Update>);

    impl DataSource for Script {
        fn label(&self) -> &'static str {
            "TEST"
        }
        fn drain(&mut self, _now: Instant) -> Vec<Update> {
            std::mem::take(&mut self.0)
        }
        fn press_estop(&mut self) {}
        fn frame_drawn(&mut self, _command: Option<Command>, _calibration: Option<Request>) {}
    }

    fn announce(origin: hud_core::origin::Origin) -> Update {
        Update::Source(hud_core::origin::Announcement {
            announcer: "sim_adapter".into(),
            entries: vec![hud_core::origin::Entry {
                name: "imu".into(),
                topic: "/sensors/imu/data".into(),
                origin,
            }],
        })
    }

    #[test]
    fn the_first_announcement_is_not_a_source_change() {
        use hud_core::origin::Origin;
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Standalone,
        );
        let mut quiet = Script(vec![]);
        assert!(matches!(panel.news(&mut quiet, t), Gate::Off(_)));
        // Ravensim announces late, while GO is already held: no zero.
        let mut sim = Script(vec![announce(Origin::Sim)]);
        let gate = panel.news(&mut sim, t);
        assert!(matches!(gate, Gate::On { .. }));
        assert_eq!(panel.buttons.last_zero(), None);
        let c = panel.act(t, &gate, [Press::SpeedUp], true);
        assert!(c.is_some_and(|c| (c.speed - 0.1).abs() < 1e-12));
        // A real announcement beside it: a change, zeroed, and the speed buttons off.
        let mut real = Script(vec![Update::Source(hud_core::origin::Announcement {
            announcer: "legacy_uart_bridge".into(),
            entries: vec![hud_core::origin::Entry {
                name: "car".into(),
                topic: String::new(),
                origin: Origin::Real,
            }],
        })]);
        let gate = panel.news(&mut real, t);
        assert_eq!(panel.buttons.last_zero(), Some(&Zeroed::SourceChanged));
        assert!(matches!(gate, Gate::Off(Off::OnTheGround)));
        assert_eq!(panel.act(t, &gate, [Press::SpeedUp], true), None);
    }

    fn calibration_news() -> Script {
        use panel_core::calibration::{FirmwareState, Raw};
        Script(vec![
            announce(hud_core::origin::Origin::Sim),
            Update::Vehicle(hud_core::vehicle::VehicleStatus {
                mode: hud_core::vehicle::Mode::from(1),
                armed: false,
                estop_active: false,
                speed: hud_core::units::MetresPerSecond(0.0),
                steering_angle: hud_core::units::Radians(0.0),
                throttle: hud_core::units::Fraction::new(0.0),
                brake: hud_core::units::Fraction::new(1.0),
                faults: hud_core::vehicle::Faults(0),
            }),
            Update::SteeringRaw(Raw {
                position: Some(2000),
                state: FirmwareState::Off,
            }),
        ])
    }

    #[test]
    fn the_calibration_page_never_drives_and_leaving_it_ends_the_mode() {
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Standalone,
        );
        let gate = panel.news(&mut calibration_news(), t);
        panel.show(Page::Calibration, t);
        assert_eq!(panel.act(t, &gate, [Press::SpeedUp], true), None);
        assert!(panel.cal_ready(t).is_empty());
        assert_eq!(panel.calibrate(t, [CalPress::Limp]), Some(Request::Limp));
        panel.show(Page::Drive, t);
        assert_eq!(panel.calibrate(t, []), Some(Request::Off));
        assert_eq!(panel.calibrate(t + END_FOR, []), None);
    }

    #[test]
    fn hiding_the_page_ends_the_calibration_as_leaving_it_does() {
        // HardwarePage::hide without a window: the same steps on the Panel.
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Standalone,
        );
        panel.news(&mut calibration_news(), t);
        panel.show(Page::Calibration, t);
        assert_eq!(panel.calibrate(t, [CalPress::Limp]), Some(Request::Limp));
        panel.end_calibration(t);
        assert_eq!(panel.page(), Page::Calibration, "the page stays for later");
        assert_eq!(panel.calibrate(t, []), Some(Request::Off));
        assert_eq!(panel.calibrate(t + END_FOR, []), None);
    }

    #[test]
    fn a_hidden_page_sends_only_the_zeros_after_go() {
        // Hidden: no presses and no GO (Input::default). Driving stops at once: zeros,
        // then nothing.
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Standalone,
        );
        let mut sim = Script(vec![announce(hud_core::origin::Origin::Sim)]);
        let gate = panel.news(&mut sim, t);
        assert!(panel.act(t, &gate, [Press::SpeedUp], true).is_some());
        let hidden = Input::default();
        assert!(!hidden.go && hidden.presses.is_empty());
        let c = panel.act(t, &gate, hidden.presses, hidden.go);
        assert!(c.is_some_and(|c| c.speed == 0.0 && c.yaw_rate == 0.0));
        assert_eq!(
            panel.act(t + Duration::from_secs(1), &gate, [], false),
            None
        );
    }

    /// The calibration self-test, without a window: the demo, the page and the script,
    /// a frame every 20 ms of the demo's time.
    fn run_cal_self_test(bind: Option<u16>) -> (bool, Option<panel_core::sweep::Outcome>) {
        let t0 = Instant::now();
        let mut demo = Demo::new(t0, bind, true);
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Standalone,
        );
        panel.show(Page::Calibration, t0);
        let mut script = CalScript::new(bind.is_some());
        let mut now = t0;
        for _ in 0..10_000 {
            now += Duration::from_millis(20);
            panel.news(&mut demo, now);
            let press = script.frame(now, &panel.cal, panel.position(now));
            let request = panel.calibrate(now, press);
            demo.frame_drawn(None, request);
            if script.finished(now) {
                break;
            }
        }
        (script.as_expected(&panel.cal), panel.cal.checked().cloned())
    }

    #[test]
    fn the_cal_self_test_passes_on_the_demo_and_finds_a_stiff_spot() {
        let (ok, outcome) = run_cal_self_test(None);
        assert!(ok, "{outcome:?}");
        let (ok, outcome) = run_cal_self_test(Some(2800));
        assert!(ok, "{outcome:?}");
        match outcome {
            Some(panel_core::sweep::Outcome::Stopped { position, .. }) => {
                assert!(position.abs_diff(2800) <= 2, "{position}")
            }
            other => panic!("{other:?}"),
        }
    }

    /// The calibration news, announced as `origin`.
    fn calibration_news_from(origin: hud_core::origin::Origin) -> Script {
        let mut news = calibration_news();
        news.0[0] = announce(origin);
        news
    }

    #[test]
    fn the_maps_page_on_the_real_car_only_watches() {
        use hud_core::origin::Origin;
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Map,
        );
        // Nothing heard yet: as the real car.
        assert_eq!(
            panel.news(&mut Script(vec![]), t),
            Gate::Off(Off::WatchOnly)
        );
        assert_eq!(panel.controls(), Controls::WatchOnly);
        let gate = panel.news(&mut calibration_news_from(Origin::Real), t);
        assert_eq!(gate, Gate::Off(Off::WatchOnly));
        assert_eq!(
            panel.act(t, &gate, [Press::SpeedUp, Press::Left], true),
            None
        );
        assert_eq!(
            panel.buttons().asked_steer(),
            0.0,
            "no steering press reaches it"
        );
        panel.show(Page::Calibration, t);
        for press in [
            CalPress::Limp,
            CalPress::StepLeft,
            CalPress::Hold,
            CalPress::Check,
        ] {
            assert_eq!(panel.calibrate(t, [press]), None, "{press:?}");
        }
        assert!(!panel.cal().is_asking());
    }

    #[test]
    fn the_maps_page_on_sim_or_bench_keeps_every_control() {
        use hud_core::origin::Origin;
        for origin in [Origin::Sim, Origin::Bench] {
            let t = Instant::now();
            let mut panel = Panel::new(
                &PanelConfig::default(),
                &Vehicle::default(),
                false,
                Host::Map,
            );
            let gate = panel.news(&mut calibration_news_from(origin), t);
            assert!(matches!(gate, Gate::On { .. }), "{origin:?}");
            assert_eq!(panel.controls(), Controls::Full);
            panel.show(Page::Calibration, t);
            assert_eq!(panel.calibrate(t, [CalPress::Limp]), Some(Request::Limp));
        }
    }

    #[test]
    fn hw_panel_on_the_real_car_keeps_its_own_rules() {
        use hud_core::origin::Origin;
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Standalone,
        );
        let gate = panel.news(&mut calibration_news_from(Origin::Real), t);
        assert_eq!(gate, Gate::Off(Off::OnTheGround), "the gate's own reason");
        assert_eq!(panel.controls(), Controls::Full);
        panel.show(Page::Calibration, t);
        assert_eq!(panel.calibrate(t, [CalPress::Limp]), Some(Request::Limp));
    }

    #[test]
    fn the_source_going_real_ends_the_maps_calibration() {
        use hud_core::origin::Origin;
        let t = Instant::now();
        let mut panel = Panel::new(
            &PanelConfig::default(),
            &Vehicle::default(),
            false,
            Host::Map,
        );
        panel.news(&mut calibration_news(), t);
        panel.show(Page::Calibration, t);
        assert_eq!(panel.calibrate(t, [CalPress::Limp]), Some(Request::Limp));
        let mut real = Script(vec![announce(Origin::Real)]);
        assert_eq!(panel.news(&mut real, t), Gate::Off(Off::WatchOnly));
        assert!(!panel.cal().is_asking());
        assert_eq!(panel.calibrate(t, [CalPress::Limp]), Some(Request::Off));
        assert_eq!(panel.calibrate(t + END_FOR, []), None);
    }

    #[test]
    fn centre_is_c() {
        let centre = PRESS_KEYS.iter().find(|(_, p)| *p == Press::Centre);
        assert_eq!(centre.map(|(k, _)| *k), Some(KeyboardKey::KEY_C));
        // The flip key is none of the panel's own.
        assert!(PRESS_KEYS.iter().all(|(k, _)| *k != FLIP_KEY));
        assert!(CAL_KEYS.iter().all(|(k, _)| *k != FLIP_KEY));
    }
}

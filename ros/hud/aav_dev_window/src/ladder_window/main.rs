//! The test ladder's window (TODO 271, `docs/proving-it.md`): every rung, the simulator's
//! side and the car's, and the progress kept by `tools/ladder.py`. Bevy and egui, like the
//! developer window; no ROS.
//!
//! ```text
//! ladder_window [--pretend] [--root DIR] [--seconds S] [--screenshot FILE]
//! ```
//!
//! `--pretend`: built-in sample data, with no tool; nothing runs. `--root`: the repository
//! (default: the folder above this one that holds `tools/ladder.py`). `--seconds` exits
//! after S seconds; `--screenshot` saves one picture halfway through.
//!
//! Every rule is the Python tool's; the window's logic is `ladder-core`
//! (`crates/ladder-core`), and this file and `draw.rs` only show it. The window starts
//! simulator runs only: it has no control that starts a car test.

mod draw;

use std::{
    path::{Path, PathBuf},
    sync::{Mutex, PoisonError},
    time::Instant,
};

use bevy::{
    app::AppExit,
    prelude::*,
    render::view::screenshot::{save_to_disk, Screenshot},
};
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use ladder_core::{command::TOOL, Session, Source};

const USAGE: &str = "usage: ladder_window [--pretend] [--root DIR] [--seconds S] \
                     [--screenshot FILE]";
/// How long the window settles before `--screenshot` and `--seconds` count, seconds.
const SETTLE: f32 = 1.0;

/// What this run was asked for.
#[derive(Debug, Clone, PartialEq, Default)]
struct Options {
    pretend: bool,
    root: Option<PathBuf>,
    seconds: Option<f32>,
    screenshot: Option<PathBuf>,
}

impl Options {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self::default();
        while let Some(arg) = args.next() {
            let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
            match arg.as_str() {
                "--pretend" => options.pretend = true,
                "--root" => options.root = Some(value()?.into()),
                "--seconds" => {
                    options.seconds =
                        Some(value()?.parse().map_err(|e| format!("--seconds: {e}"))?);
                }
                "--screenshot" => options.screenshot = Some(value()?.into()),
                other => return Err(format!("unknown option {other}")),
            }
        }
        Ok(options)
    }

    /// Where the ladder comes from: sample data, or the tool in the repository's root.
    fn source(&self, from: &Path) -> Result<Source, String> {
        if self.pretend {
            return Ok(Source::Pretend);
        }
        let root = match &self.root {
            Some(root) => root.clone(),
            None => from
                .ancestors()
                .find(|dir| dir.join(TOOL).is_file())
                .ok_or_else(|| {
                    format!(
                        "no {TOOL} in {} or above it: start it in the repository \
                         (pixi run ladder-window), or give --root DIR, or --pretend",
                        from.display()
                    )
                })?
                .to_owned(),
        };
        if root.join(TOOL).is_file() {
            Ok(Source::Tool { root })
        } else {
            Err(format!("no {TOOL} in {}", root.display()))
        }
    }
}

/// The session (behind a mutex only to be a Bevy resource: one system uses it at a time).
#[derive(Resource)]
struct Ui {
    session: Mutex<Session>,
    styled: bool,
}

#[derive(Resource)]
struct Run {
    seconds: Option<f32>,
    screenshot: Option<PathBuf>,
    shot: bool,
}

fn main() -> AppExit {
    let source = Options::parse(std::env::args().skip(1)).and_then(|options| {
        let here = std::env::current_dir().map_err(|why| format!("no current folder: {why}"))?;
        Ok((options.source(&here)?, options))
    });
    let (source, options) = match source {
        Ok(found) => found,
        Err(message) => {
            eprintln!("ladder_window: {message}\n{USAGE}");
            return AppExit::from_code(2);
        }
    };

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.06, 0.06, 0.07)))
        .insert_resource(Ui {
            session: Mutex::new(Session::new(source)),
            styled: false,
        })
        .insert_resource(Run {
            seconds: options.seconds,
            screenshot: options.screenshot,
            shot: false,
        })
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "AAV test ladder".into(),
                resolution: (1500, 950).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
        .add_systems(EguiPrimaryContextPass, draw)
        .add_systems(Update, timer)
        .add_systems(Last, stop_on_exit)
        .run()
}

fn draw(mut contexts: EguiContexts, mut ui: ResMut<Ui>) -> Result {
    let ctx = contexts.ctx_mut()?.clone();
    let ui = &mut *ui;
    if !ui.styled {
        draw::style(&ctx);
        ui.styled = true;
    }
    let session = ui.session.get_mut().unwrap_or_else(PoisonError::into_inner);
    session.tick(Instant::now());
    draw::window(&ctx, session);
    Ok(())
}

/// `--seconds` and `--screenshot`.
fn timer(
    mut commands: Commands,
    time: Res<Time<Real>>,
    mut run: ResMut<Run>,
    mut exit: MessageWriter<AppExit>,
) {
    let elapsed = time.elapsed_secs();
    let shot_at = SETTLE + run.seconds.unwrap_or(0.0) / 2.0;
    if !run.shot && elapsed > shot_at {
        if let Some(path) = run.screenshot.clone() {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            run.shot = true;
        }
    }
    if run.seconds.is_some_and(|s| elapsed > SETTLE + s) {
        exit.write(AppExit::Success);
    }
}

/// The window closing takes its run with it: no simulator is left behind.
fn stop_on_exit(mut exits: MessageReader<AppExit>, mut ui: ResMut<Ui>) {
    if exits.read().next().is_some() {
        ui.session
            .get_mut()
            .unwrap_or_else(PoisonError::into_inner)
            .press_stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Options, String> {
        Options::parse(args.iter().map(|&a| a.to_owned()))
    }

    #[test]
    fn the_options_read() {
        let options = parse(&["--pretend", "--seconds", "5", "--screenshot", "/tmp/l.png"])
            .expect("they read");
        assert!(options.pretend);
        assert_eq!(options.seconds, Some(5.0));
        assert_eq!(options.screenshot, Some("/tmp/l.png".into()));
        assert_eq!(parse(&[]).expect("none is fine"), Options::default());
    }

    #[test]
    fn a_wrong_option_is_refused() {
        assert!(parse(&["--seconds"]).is_err());
        assert!(parse(&["--seconds", "soon"]).is_err());
        assert!(parse(&["--car"]).is_err());
        assert!(parse(&["stands"]).is_err());
    }

    #[test]
    fn pretend_needs_no_repository() {
        let options = parse(&["--pretend"]).expect("it reads");
        assert_eq!(options.source(Path::new("/")), Ok(Source::Pretend));
    }

    #[test]
    fn the_tool_is_found_from_any_folder_of_the_repository() {
        // This package is ros/hud/aav_dev_window: the tool is three folders up.
        let here = Path::new(env!("CARGO_MANIFEST_DIR"));
        let Ok(Source::Tool { root }) = Options::default().source(&here.join("src")) else {
            panic!("the repository has tools/ladder.py");
        };
        assert_eq!(Some(root.as_path()), here.ancestors().nth(3));
        assert!(Options::default().source(Path::new("/")).is_err());
        let wrong = Options {
            root: Some(here.to_owned()),
            ..Options::default()
        };
        assert!(wrong.source(here).is_err());
    }
}

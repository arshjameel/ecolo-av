//! hud: the in-car screen. One program: a raylib window, and the ROS node `hud`.
//!
//! Subscribes  (names from config/hud.yaml, or ROS parameters)
//!             vehicle_status_topic  aav_interfaces/VehicleStatus  speed, mode, e-stop, faults
//!             gps_topic             sensor_msgs/NavSatFix         the GPS panel
//!             gps_filtered_topic    sensor_msgs/NavSatFix         listed under TOPICS
//!             stop_sign_topic       std_msgs/Bool                 the STOP SIGN banner
//!             battery_topic         sensor_msgs/BatteryState      the strip's battery
//!             camera_topic          sensor_msgs/Image             the camera panel
//!             source_topic          aav_interfaces/DataSource     latched: the SIMULATED banner
//!             watchdog_topic        std_msgs/String               the sensor watchdog: a banner,
//!                                                                 and the GO / STOP panel
//!             output_mux_topic      std_msgs/String               the output mux's gate: the
//!                                                                 GO / STOP panel (hud_core::gate)
//! Publishes   estop_topic           std_msgs/Bool                 only true: the e-stop button
//!             heartbeat_topic       std_msgs/UInt64               10 Hz while the screen draws:
//!                                                                 frames drawn (hud_core::heartbeat)
//! Parameters  the topics above, stale_after (s), max_speed (m/s: the gauge's redline),
//!             and theme (initial_d, mocha, latte, calm)
//!
//! raylib owns the main thread (macOS needs that); rclrs spins on its own thread and
//! sends news over a channel (ros.rs). `--demo` swaps the ROS node for pretend data.
//!
//! usage: hud [--demo] [--fullscreen] [--monitor NAME|N] [--list-monitors] [--config FILE]
//!            [--screenshot FILE] [--ros-args ...]

mod demo;
#[cfg(feature = "ros")]
mod ros;
mod source;
mod ui;

use std::{
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use hud_app::window;
use hud_core::{config::HudConfig, screen::Monitor, state::HudState};
use raylib::prelude::*;

use crate::{demo::Demo, source::DataSource, ui::Ui};

const USAGE: &str = "\
usage: hud [--demo] [--fullscreen] [--monitor NAME|N] [--list-monitors] [--config FILE]
           [--screenshot FILE [--screenshot-after SECONDS]] [--ros-args ...]

  --demo             pretend data: no ROS, no car
  --fullscreen       fill the screen (F11 switches too)
  --monitor NAME|N   open on this monitor: its name exactly (HDMI-1, as xrandr says) or
                     its number (0 = the first). Not there: an error that lists them
  --list-monitors    print the monitors' numbers, names and sizes, then quit
  --config FILE      topic names from FILE instead of the built-in config/hud.yaml
  --screenshot FILE  save the window as a PNG after 4 s, then quit (for the docs)
  --screenshot-after SECONDS
                     when to save it instead: the demo's STOP is at 50 s, AUTO STOP at 37 s
  --ros-args ...     passed to ROS, e.g. --ros-args -p gps_topic:=/localization/gps/fix";

/// When --screenshot saves its picture, unless --screenshot-after says otherwise: once the
/// data has had time to arrive.
const SCREENSHOT_AFTER: Duration = Duration::from_secs(4);

/// The command line, before `--ros-args`.
#[derive(Debug, Default)]
struct Options {
    demo: bool,
    fullscreen: bool,
    monitor: Option<Monitor>,
    list_monitors: bool,
    config: Option<PathBuf>,
    screenshot: Option<PathBuf>,
    screenshot_after: Option<Duration>,
}

impl Options {
    /// Reads our own options; stops at `--ros-args`, which belongs to ROS.
    fn parse(args: &[String]) -> Result<Self> {
        let mut opts = Self::default();
        let mut it = args.iter().skip(1);
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--ros-args" => break,
                "--demo" => opts.demo = true,
                "--fullscreen" => opts.fullscreen = true,
                "--list-monitors" => opts.list_monitors = true,
                "--monitor" => {
                    let m = it.next().context("--monitor needs a name or a number")?;
                    opts.monitor = Some(m.parse()?);
                }
                "--config" => {
                    opts.config = Some(it.next().context("--config needs a file")?.into())
                }
                "--screenshot" => {
                    opts.screenshot = Some(it.next().context("--screenshot needs a file")?.into())
                }
                "--screenshot-after" => {
                    let s = it.next().context("--screenshot-after needs seconds")?;
                    let secs: f64 = s.parse().with_context(|| format!("not seconds: {s:?}"))?;
                    opts.screenshot_after = Some(
                        Duration::try_from_secs_f64(secs)
                            .with_context(|| format!("not a time: {s:?}"))?,
                    );
                }
                "-h" | "--help" => {
                    println!("{USAGE}");
                    std::process::exit(0);
                }
                other => bail!("unknown option {other:?}\n{USAGE}"),
            }
        }
        Ok(opts)
    }
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let opts = Options::parse(&args)?;
    if opts.list_monitors {
        window::print_monitors();
        return Ok(());
    }
    let config = match &opts.config {
        Some(path) => {
            let text = std::fs::read_to_string(path)
                .with_context(|| format!("reading {}", path.display()))?;
            HudConfig::from_yaml(&text).with_context(|| format!("in {}", path.display()))?
        }
        None => HudConfig::default(),
    };
    let (source, config) = open_source(&opts, args, config)?;
    run(&opts, source, config)
}

/// The demo, or the ROS node (which may rename topics from its parameters).
fn open_source(
    opts: &Options,
    args: Vec<String>,
    config: HudConfig,
) -> Result<(Box<dyn DataSource>, HudConfig)> {
    if opts.demo {
        return Ok((Box::new(Demo::new(Instant::now())), config));
    }
    #[cfg(feature = "ros")]
    {
        let (source, config) = ros::RosSource::start(args, config)?;
        Ok((Box::new(source), config))
    }
    #[cfg(not(feature = "ros"))]
    {
        let _ = (args, config);
        bail!("this hud was built without ROS (--no-default-features): run it with --demo")
    }
}

/// The window's loop: news in, a click on the e-stop out, one frame drawn.
fn run(opts: &Options, mut source: Box<dyn DataSource>, config: HudConfig) -> Result<()> {
    let (mut rl, thread) = raylib::init()
        .size(1280, 720)
        .title("AAV HUD")
        .resizable()
        .msaa_4x()
        .highdpi()
        .log_level(TraceLogLevel::LOG_WARNING)
        .build();
    // Escape would close the HUD by accident; the window's close button still does.
    rl.set_exit_key(None);
    rl.set_target_fps(30);
    rl.set_window_min_size(640, 360);
    window::place(&mut rl, opts.monitor.as_ref(), opts.fullscreen)?;
    // The speed gauge's scale, from max_speed (validate() has checked it).
    let dial = config.gauge()?;
    let mut ui = Ui::new(&mut rl, &thread, config.theme.theme(), dial);
    let mut state = HudState::new(config);
    let started = Instant::now();

    while !rl.window_should_close() && !source.has_stopped() {
        let now = Instant::now();
        for update in source.drain(now) {
            state.apply(update, now);
        }
        if let Some(picture) = state.take_new_picture() {
            if let Err(e) = ui.upload(&mut rl, &thread, &picture) {
                state.picture_problem = Some(format!("can't show the picture: {e}"));
            }
        }
        if rl.is_key_pressed(KeyboardKey::KEY_F11) {
            window::toggle_fullscreen(&mut rl);
        }
        let button = ui.estop_button(rl.get_screen_width(), rl.get_screen_height());
        let hover = button.check_collision_point_rec(rl.get_mouse_position());
        // A touchscreen's tap arrives as a left click.
        if hover && rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            source.press_estop();
            state.estop_pressed();
        }
        let label = source.label();
        let mut d = rl.begin_drawing(&thread);
        ui.draw(&mut d, &state, now, label, hover);
        drop(d);
        // On the screen now: the heartbeat may say so.
        source.frame_drawn();

        if let Some(path) = &opts.screenshot {
            if now.duration_since(started) > opts.screenshot_after.unwrap_or(SCREENSHOT_AFTER) {
                save_screenshot(&mut rl, &thread, path)?;
                break;
            }
        }
    }
    Ok(())
}

/// Saves what the window shows as a PNG. (raylib's own take_screenshot only writes into
/// the folder the program started in.)
fn save_screenshot(rl: &mut RaylibHandle, thread: &RaylibThread, path: &Path) -> Result<()> {
    let path = std::path::absolute(path)?;
    let name = path
        .to_str()
        .context("--screenshot: the path must be UTF-8")?;
    rl.load_image_from_screen(thread).export_image(name);
    if !path.exists() {
        bail!("raylib couldn't write {name}");
    }
    println!("saved {name}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        std::iter::once("hud")
            .chain(list.iter().copied())
            .map(String::from)
            .collect()
    }

    #[test]
    fn options_stop_at_ros_args() {
        let o = Options::parse(&args(&["--demo", "--ros-args", "--whatever"])).expect("valid");
        assert!(o.demo);
        assert!(!o.fullscreen);
    }

    #[test]
    fn options_take_files() {
        let o = Options::parse(&args(&["--config", "a.yaml", "--fullscreen"])).expect("valid");
        assert_eq!(o.config, Some(PathBuf::from("a.yaml")));
        assert!(o.fullscreen);
        assert!(Options::parse(&args(&["--config"])).is_err());
        assert!(Options::parse(&args(&["--bogus"])).is_err());
    }

    #[test]
    fn options_take_a_screenshot_time() {
        let o = Options::parse(&args(&[
            "--screenshot",
            "a.png",
            "--screenshot-after",
            "50",
        ]))
        .expect("valid");
        assert_eq!(o.screenshot, Some(PathBuf::from("a.png")));
        assert_eq!(o.screenshot_after, Some(Duration::from_secs(50)));
        assert!(Options::parse(&args(&["--screenshot-after", "soon"])).is_err());
        assert!(Options::parse(&args(&["--screenshot-after", "-1"])).is_err());
        assert!(Options::parse(&args(&["--screenshot-after"])).is_err());
    }

    #[test]
    fn options_take_a_monitor() {
        let o = Options::parse(&args(&["--monitor", "HDMI-1", "--fullscreen"])).expect("valid");
        assert_eq!(o.monitor, Some(Monitor::Name("HDMI-1".into())));
        let o = Options::parse(&args(&["--monitor", "1"])).expect("valid");
        assert_eq!(o.monitor, Some(Monitor::Number(1)));
        assert!(
            Options::parse(&args(&["--list-monitors"]))
                .expect("valid")
                .list_monitors
        );
        assert!(Options::parse(&args(&["--monitor"])).is_err());
        assert!(Options::parse(&args(&["--monitor", ""])).is_err());
    }
}

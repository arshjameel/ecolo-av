//! map: the big screen's map window. One program: a raylib window, and the ROS node `map`.
//!
//! Subscribes  (names from config/map.yaml, or ROS parameters)
//!             map_topic     nav_msgs/OccupancyGrid       latched: Nav2's map, drawn north up
//!             odom_topic    nav_msgs/Odometry            the car, in the map frame, and its speed
//!             plan_topic    nav_msgs/Path                Nav2's plan: the line, the distance left
//!             goal_topic    geometry_msgs/PoseStamped    RViz's goal (or ours), until Nav2 plans for it
//!             source_topic  aav_interfaces/DataSource    latched: the SIMULATED banner
//!             street_route_topic  nav_msgs/Path          latched: street_router's street route
//!                                                        (aav_routes), violet, under the plan
//!             watchdog_topic  std_msgs/String            the sensor watchdog's verdict
//!                                                        (/safety/sensor_watchdog/state):
//!                                                        a banner unless OK
//! Publishes   goal_topic    geometry_msgs/PoseStamped    the place search's goal, map frame,
//!                                                        only after DRIVE HERE then CONFIRM
//! Parameters  the topics above, stale_after, plan_stale_after (s), final_approach (m),
//!             plan_kept_for (s), arrive_within (m),
//!             street_map (the offline street map's file), places (the place database)
//!
//! Under Nav2's map: OpenStreetMap's streets, from an offline file (streets.rs), placed
//! with navsat_transform's datum (navsat.yaml), so the car sits on real streets.
//!
//! The place search (search.rs, `/` to type): OpenStreetMap's names and addresses from
//! ottawa.db, then DRIVE HERE and CONFIRM send Nav2 a goal. Refused off Nav2's map or
//! with no car position. A tap (or a click) on the box opens an on-screen keyboard, for
//! the car's touchscreen; a real keyboard works as well.
//!
//! raylib owns the main thread (macOS needs that); rclrs spins on its own thread and
//! sends news over a channel (ros.rs). `--demo` swaps the ROS node for pretend data.
//! The maths (map to screen, the distance left) is in map-core.
//!
//! usage: map [--demo] [--fullscreen] [--monitor NAME|N] [--list-monitors]
//!            [--whole-map] [--config FILE] [--vehicle FILE]
//!            [--navsat FILE] [--streets FILE] [--places FILE] [--zoom PX_PER_M]
//!            [--search TEXT [--pick N]] [--keyboard] [--send-goal TEXT] [--screenshot FILE]
//!            [--ros-args ...]

mod demo;
#[cfg(feature = "ros")]
mod ros;
mod search;
mod source;
mod streets;
mod ui;

use std::{
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use hud_app::window;
use hud_core::screen::Monitor;
use map_core::{
    car::CarShape,
    config::MapConfig,
    destination::Key,
    earth::{Datum, BUILT_IN_NAVSAT_YAML},
    keyboard::Press,
    places::Places,
    state::MapState,
    view::{PixelsPerMetre, ScreenPoint, View},
};
use raylib::prelude::*;

use crate::{
    demo::Demo,
    search::Search,
    source::DataSource,
    streets::StreetMap,
    ui::{Layout, SearchLayout, Ui},
};

const USAGE: &str = "\
usage: map [--demo] [--fullscreen] [--monitor NAME|N] [--list-monitors]
           [--whole-map] [--config FILE] [--vehicle FILE]
           [--navsat FILE] [--streets FILE] [--places FILE] [--zoom PX_PER_M]
           [--search TEXT [--pick N]] [--keyboard] [--send-goal TEXT] [--screenshot FILE]
           [--ros-args ...]

  --demo             pretend data: no ROS, no car
  --fullscreen       fill the screen (F11 switches too)
  --monitor NAME|N   open on this monitor: its name exactly (HDMI-2, as xrandr says) or
                     its number (0 = the first). Not there: an error that lists them
  --list-monitors    print the monitors' numbers, names and sizes, then quit
  --whole-map        start showing the whole map, not following the car (F switches)
  --config FILE      topic names from FILE instead of the built-in config/map.yaml
  --vehicle FILE     the car's size from FILE instead of the built-in vehicle.yaml
  --navsat FILE      the datum (map 0,0 on the Earth) from navsat_transform's FILE
                     instead of the built-in navsat.yaml
  --streets FILE     the street map (PMTiles) instead of street_map in config/map.yaml
  --places FILE      the place database (SQLite) instead of places in config/map.yaml
  --zoom PX_PER_M    start at this scale, in pixels per metre (0.1 shows ~10 km)
  --search TEXT      start with TEXT typed in the search box (for screenshots)
  --pick N           with --search: pick result N (1 = the first)
  --keyboard         start with the search box open and the on-screen keyboard showing
                     (as after a tap on the box; for screenshots)
  --send-goal TEXT   once the map and the car are known: search TEXT, send the first
                     result inside Nav2's map as the goal, say so on the terminal (tests)
  --screenshot FILE  save the window as a PNG after 4 s, then quit (for the docs)
  --ros-args ...     passed to ROS, e.g. --ros-args -p plan_topic:=/plan

keys: / search for a place (Esc closes; Up Down and Enter pick; Enter: DRIVE HERE, then
      CONFIRM), F follow the car / whole map, S streets on / off, + - or the wheel zoom,
      drag to move, F11 full screen
touch: tap the search box for the on-screen keyboard; tap a result, DRIVE HERE, CONFIRM";

/// When --screenshot saves its picture: once the data has had time to arrive.
const SCREENSHOT_AFTER: Duration = Duration::from_secs(4);
/// How long --send-goal waits for the map and the car before it gives up.
const SEND_GOAL_WAIT: Duration = Duration::from_secs(120);
/// One click of the mouse wheel, or one press of + or -.
const WHEEL_ZOOM: f64 = 1.15;
const KEY_ZOOM: f64 = 1.25;

/// The command line, before `--ros-args`.
#[derive(Debug, Default)]
struct Options {
    demo: bool,
    fullscreen: bool,
    monitor: Option<Monitor>,
    list_monitors: bool,
    whole_map: bool,
    config: Option<PathBuf>,
    vehicle: Option<PathBuf>,
    navsat: Option<PathBuf>,
    streets: Option<PathBuf>,
    places: Option<PathBuf>,
    zoom: Option<f64>,
    search: Option<String>,
    pick: Option<usize>,
    keyboard: bool,
    send_goal: Option<String>,
    screenshot: Option<PathBuf>,
}

impl Options {
    /// Reads our own options; stops at `--ros-args`, which belongs to ROS.
    fn parse(args: &[String]) -> Result<Self> {
        let mut opts = Self::default();
        let mut it = args.iter().skip(1);
        while let Some(arg) = it.next() {
            // The option's value: a file, or words.
            let mut value = |what: &str| -> Result<String> {
                Ok(it
                    .next()
                    .with_context(|| format!("{what} needs a value"))?
                    .clone())
            };
            match arg.as_str() {
                "--ros-args" => break,
                "--demo" => opts.demo = true,
                "--fullscreen" => opts.fullscreen = true,
                "--list-monitors" => opts.list_monitors = true,
                "--monitor" => opts.monitor = Some(value("--monitor")?.parse()?),
                "--whole-map" => opts.whole_map = true,
                "--config" => opts.config = Some(value("--config")?.into()),
                "--vehicle" => opts.vehicle = Some(value("--vehicle")?.into()),
                "--navsat" => opts.navsat = Some(value("--navsat")?.into()),
                "--streets" => opts.streets = Some(value("--streets")?.into()),
                "--places" => opts.places = Some(value("--places")?.into()),
                "--search" => opts.search = Some(value("--search")?),
                "--send-goal" => opts.send_goal = Some(value("--send-goal")?),
                "--keyboard" => opts.keyboard = true,
                "--pick" => {
                    let n: usize = it
                        .next()
                        .context("--pick needs a number")?
                        .parse()
                        .context("--pick: a result's number, 1 for the first")?;
                    if n == 0 {
                        bail!("--pick counts from 1");
                    }
                    opts.pick = Some(n);
                }
                "--zoom" => {
                    let k: f64 = it
                        .next()
                        .context("--zoom needs a number")?
                        .parse()
                        .context("--zoom: pixels per metre, such as 0.2")?;
                    if !(k.is_finite() && k > 0.0) {
                        bail!("--zoom must be a positive number");
                    }
                    opts.zoom = Some(k);
                }
                "--screenshot" => opts.screenshot = Some(value("--screenshot")?.into()),
                "-h" | "--help" => {
                    println!("{USAGE}");
                    std::process::exit(0);
                }
                other => bail!("unknown option {other:?}\n{USAGE}"),
            }
        }
        if opts.pick.is_some() && opts.search.is_none() {
            bail!("--pick needs --search");
        }
        Ok(opts)
    }
}

fn read(path: &Path) -> Result<String> {
    std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))
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
            MapConfig::from_yaml(&read(path)?).with_context(|| format!("in {}", path.display()))?
        }
        None => MapConfig::default(),
    };
    let car = match &opts.vehicle {
        Some(path) => CarShape::from_vehicle_yaml(&read(path)?)
            .with_context(|| format!("in {}", path.display()))?,
        None => CarShape::default(),
    };
    let datum = match &opts.navsat {
        Some(path) => Datum::from_navsat_yaml(&read(path)?),
        None => Datum::from_navsat_yaml(BUILT_IN_NAVSAT_YAML),
    }
    .with_context(|| {
        let file = opts
            .navsat
            .as_deref()
            .unwrap_or(Path::new("the built-in navsat.yaml"));
        format!("the datum, in {}", file.display())
    })?;
    let (source, config) = open_source(&opts, args, config)?;
    let street_file = data_file(opts.streets.as_deref(), &config.street_map);
    let streets = StreetMap::open(&street_file, datum);
    let places_file = data_file(opts.places.as_deref(), &config.places);
    let places = Places::open(&places_file).map_err(|e| e.to_string());
    if let Err(why) = &places {
        eprintln!("map: no place search: {why}");
    }
    let search = Search::new(places, datum);
    run(&opts, source, config, car, streets, search)
}

/// A data file (the street map, the places): the option, or config/map.yaml's. A
/// relative path is looked for in the folder the window started in, then in the pixi
/// project's.
fn data_file(option: Option<&Path>, configured: &str) -> PathBuf {
    let file = option.map_or_else(|| PathBuf::from(configured), Path::to_path_buf);
    if file.is_absolute() || file.exists() {
        return file;
    }
    std::env::var_os("PIXI_PROJECT_ROOT")
        .map(|root| PathBuf::from(root).join(&file))
        .filter(|p| p.exists())
        .unwrap_or(file)
}

/// The demo, or the ROS node (which may rename topics from its parameters).
fn open_source(
    opts: &Options,
    args: Vec<String>,
    config: MapConfig,
) -> Result<(Box<dyn DataSource>, MapConfig)> {
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
        bail!("this map was built without ROS (--no-default-features): run it with --demo")
    }
}

/// The window's loop: news in, keys and mouse, one frame drawn.
fn run(
    opts: &Options,
    mut source: Box<dyn DataSource>,
    config: MapConfig,
    car: CarShape,
    mut streets: StreetMap,
    mut search: Search,
) -> Result<()> {
    let (mut rl, thread) = raylib::init()
        .size(1600, 900)
        .title("AAV map")
        .resizable()
        .msaa_4x()
        .highdpi()
        .log_level(TraceLogLevel::LOG_WARNING)
        .build();
    // Escape would close the map by accident; the window's close button still does.
    rl.set_exit_key(None);
    rl.set_target_fps(30);
    rl.set_window_min_size(800, 450);
    window::place(&mut rl, opts.monitor.as_ref(), opts.fullscreen)?;
    let mut ui = Ui::new(&mut rl, &thread);
    let mut state = MapState::new(config);
    let mut view = View::default();
    if opts.whole_map {
        view.toggle_follow();
    }
    if let Some(k) = opts.zoom {
        view.set_scale(PixelsPerMetre(k));
    }
    // Nav2's map is drawn see-through over the streets: redrawn when they come or go.
    let mut over_streets = streets.visible();
    let mut dragging = false;
    let started = Instant::now();
    // --search waits for the first news (the car), so the distances show.
    let mut prefill = opts.search.clone();
    let mut send_goal = opts.send_goal.clone();
    // The on-screen keyboard: opened by a tap on the box (or --keyboard), shown while
    // the box is typing. `/` on a real keyboard leaves it shut.
    let mut keyboard_on = false;
    let mut open_keyboard = opts.keyboard;

    while !rl.window_should_close() && !source.has_stopped() {
        let now = Instant::now();
        for update in source.drain(now) {
            state.apply(update, now);
        }
        let streets_changed = over_streets != streets.visible();
        if state.take_grid_change() || streets_changed {
            over_streets = streets.visible();
            if let Some(grid) = &state.grid {
                if let Err(e) = ui.upload(&mut rl, &thread, grid, over_streets) {
                    state.map_problem = Some(format!("can't show the map: {e}"));
                }
            }
        }

        let banner = state.origins.banner().is_some();
        let layout = Layout::new(rl.get_screen_width(), rl.get_screen_height(), banner);
        let vp = layout.map;
        let mouse = rl.get_mouse_position();
        let mouse = ScreenPoint::new(f64::from(mouse.x), f64::from(mouse.y));
        let over_map = vp.contains(mouse);

        // The place search first: while typing, every key is a letter for it.
        if let Some(query) = prefill.take() {
            search.prefill(&query, &state, &mut view, source.as_mut(), now);
            if let Some(n) = opts.pick {
                search.choose(n - 1, &state, &mut view, source.as_mut(), now);
            }
        }
        // --keyboard: after --search, so the words are there.
        if prefill.is_none() && std::mem::take(&mut open_keyboard) {
            if !search.finder.is_typing() {
                search.key(Key::Open, &state, &mut view, source.as_mut(), now);
            }
            keyboard_on = true;
        }
        if let Some(query) = send_goal.clone() {
            if state.grid.is_some() && state.car.is_some() {
                send_goal = None;
                match search.send_first(&query, &state, source.as_mut(), now) {
                    Ok(said) => println!("map: --send-goal: {said}"),
                    Err(why) => eprintln!("map: --send-goal {query:?}: {why}"),
                }
            } else if now.duration_since(started) > SEND_GOAL_WAIT {
                send_goal = None;
                eprintln!("map: --send-goal {query:?}: no map or no car position after {SEND_GOAL_WAIT:?}");
            }
        }
        let typed: Vec<char> = std::iter::from_fn(|| rl.get_char_pressed()).collect();
        for key in search_keys(
            &rl,
            &typed,
            search.finder.is_typing(),
            search.finder.is_open(),
        ) {
            search.key(key, &state, &mut view, source.as_mut(), now);
        }
        // The keyboard goes away once a place is picked, or the box shuts.
        keyboard_on &= search.finder.is_typing();
        let keyboard = keyboard_on.then(|| layout.keyboard());
        let on_keyboard = keyboard.as_ref().filter(|kb| kb.contains(mouse));
        let boxes = SearchLayout::new(&layout, &search.finder, keyboard.as_ref());
        let over_search = boxes.contains(mouse) || on_keyboard.is_some();
        if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            if let Some(kb) = on_keyboard {
                // A tap on a key (between keys: nothing, and not the map either).
                match kb.key_at(mouse).map(|cap| cap.press()) {
                    Some(Press::Search(key)) => {
                        search.key(key, &state, &mut view, source.as_mut(), now)
                    }
                    Some(Press::Close) => keyboard_on = false,
                    None => {}
                }
            } else if let Some(click) = boxes.click(mouse) {
                match click {
                    ui::Click::Field => {
                        search.key(Key::Open, &state, &mut view, source.as_mut(), now);
                        keyboard_on = true;
                    }
                    ui::Click::Result(i) => {
                        search.choose(i, &state, &mut view, source.as_mut(), now)
                    }
                    ui::Click::Button => {
                        search.key(Key::Enter, &state, &mut view, source.as_mut(), now)
                    }
                    ui::Click::Cancel => {
                        search.key(Key::Escape, &state, &mut view, source.as_mut(), now)
                    }
                }
            }
        }

        // The map's own keys, unless they're letters for the search box.
        let map_keys = !search.finder.is_typing();
        if rl.is_key_pressed(KeyboardKey::KEY_F11) {
            window::toggle_fullscreen(&mut rl);
        }
        if map_keys && rl.is_key_pressed(KeyboardKey::KEY_F) {
            view.toggle_follow();
        }
        if map_keys && rl.is_key_pressed(KeyboardKey::KEY_S) {
            streets.shown = !streets.shown;
        }
        let wheel = rl.get_mouse_wheel_move();
        if wheel != 0.0 && over_map && !over_search {
            view.zoom(WHEEL_ZOOM.powf(f64::from(wheel)), Some(mouse), &vp);
        }
        let zoom_in = [KeyboardKey::KEY_EQUAL, KeyboardKey::KEY_KP_ADD];
        let zoom_out = [KeyboardKey::KEY_MINUS, KeyboardKey::KEY_KP_SUBTRACT];
        if map_keys && zoom_in.into_iter().any(|k| rl.is_key_pressed(k)) {
            view.zoom(KEY_ZOOM, None, &vp);
        }
        if map_keys && zoom_out.into_iter().any(|k| rl.is_key_pressed(k)) {
            view.zoom(1.0 / KEY_ZOOM, None, &vp);
        }
        // Drag to move: only a drag that starts on the map, not on the search box.
        if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            dragging = over_map && !over_search;
        }
        if !rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT) {
            dragging = false;
        }
        if dragging {
            let d = rl.get_mouse_delta();
            view.pan(f64::from(d.x), f64::from(d.y));
        }
        let car_centre = state.car.as_ref().map(|c| car.centre(c.pose));
        let map_bounds = state.grid.as_ref().map(|g| g.info.bounds());
        view.update(car_centre, map_bounds, &vp);

        let label = source.label();
        let tiles = streets.frame(&view, &vp);
        // After this frame's taps: the keyboard as it is now, and the key held down.
        keyboard_on &= search.finder.is_typing();
        let keyboard = keyboard_on.then(|| layout.keyboard());
        let held = keyboard
            .as_ref()
            .filter(|_| rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT))
            .and_then(|kb| kb.key_at(mouse));
        let mut d = rl.begin_drawing(&thread);
        ui.draw(
            &mut d,
            &state,
            &view,
            &car,
            now,
            label,
            &streets,
            &tiles,
            &search,
            keyboard.as_ref().map(|kb| (kb, held)),
        );
        drop(d);

        if let Some(path) = &opts.screenshot {
            if now.duration_since(started) > SCREENSHOT_AFTER {
                save_screenshot(&mut rl, &thread, path)?;
                break;
            }
        }
    }
    Ok(())
}

/// This frame's keys for the search box. Typing: the letters, and the editing keys
/// (held Backspace repeats). Open: Enter and Esc. Shut: `/` opens it.
fn search_keys(rl: &RaylibHandle, typed: &[char], typing: bool, open: bool) -> Vec<Key> {
    let pressed = |k| rl.is_key_pressed(k);
    let mut keys = Vec::new();
    if typing {
        keys.extend(typed.iter().copied().map(Key::Char));
        let backspace = KeyboardKey::KEY_BACKSPACE;
        if pressed(backspace) || rl.is_key_pressed_repeat(backspace) {
            keys.push(Key::Backspace);
        }
        if pressed(KeyboardKey::KEY_UP) {
            keys.push(Key::Up);
        }
        if pressed(KeyboardKey::KEY_DOWN) {
            keys.push(Key::Down);
        }
    } else if typed.contains(&'/') {
        keys.push(Key::Open);
        return keys;
    }
    if open {
        if pressed(KeyboardKey::KEY_ENTER) || pressed(KeyboardKey::KEY_KP_ENTER) {
            keys.push(Key::Enter);
        }
        if pressed(KeyboardKey::KEY_ESCAPE) {
            keys.push(Key::Escape);
        }
    }
    keys
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
        std::iter::once("map")
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
        let o = Options::parse(&args(&["--vehicle", "v.yaml", "--screenshot", "a.png"]))
            .expect("valid");
        assert_eq!(o.vehicle, Some(PathBuf::from("v.yaml")));
        assert_eq!(o.screenshot, Some(PathBuf::from("a.png")));
        assert!(Options::parse(&args(&["--config"])).is_err());
        assert!(Options::parse(&args(&["--bogus"])).is_err());
    }

    #[test]
    fn options_take_the_street_map_and_a_zoom() {
        let o = Options::parse(&args(&[
            "--streets",
            "s.pmtiles",
            "--navsat",
            "n.yaml",
            "--zoom",
            "0.2",
        ]))
        .expect("valid");
        assert_eq!(o.streets, Some(PathBuf::from("s.pmtiles")));
        assert_eq!(o.navsat, Some(PathBuf::from("n.yaml")));
        assert_eq!(o.zoom, Some(0.2));
        assert!(Options::parse(&args(&["--zoom", "-1"])).is_err());
        assert!(Options::parse(&args(&["--zoom", "near"])).is_err());
    }

    #[test]
    fn options_take_a_search() {
        let o = Options::parse(&args(&[
            "--places",
            "p.db",
            "--search",
            "carleton u",
            "--pick",
            "2",
            "--send-goal",
            "university drive",
        ]))
        .expect("valid");
        assert_eq!(o.places, Some(PathBuf::from("p.db")));
        assert_eq!(o.search.as_deref(), Some("carleton u"));
        assert_eq!(o.pick, Some(2));
        assert_eq!(o.send_goal.as_deref(), Some("university drive"));
        assert!(!o.keyboard);
        assert!(
            Options::parse(&args(&["--keyboard"]))
                .expect("valid")
                .keyboard
        );
        assert!(
            Options::parse(&args(&["--pick", "1"])).is_err(),
            "--pick needs --search"
        );
        assert!(Options::parse(&args(&["--search", "x", "--pick", "0"])).is_err());
        assert!(Options::parse(&args(&["--search"])).is_err());
    }

    #[test]
    fn options_take_a_monitor() {
        let o = Options::parse(&args(&["--monitor", "HDMI-2", "--fullscreen"])).expect("valid");
        assert_eq!(o.monitor, Some(Monitor::Name("HDMI-2".into())));
        assert!(o.fullscreen);
        let o = Options::parse(&args(&["--monitor", "0"])).expect("valid");
        assert_eq!(o.monitor, Some(Monitor::Number(0)));
        assert!(
            Options::parse(&args(&["--list-monitors"]))
                .expect("valid")
                .list_monitors
        );
        assert!(Options::parse(&args(&["--monitor"])).is_err());
        assert!(Options::parse(&args(&["--monitor", " "])).is_err());
    }

    #[test]
    fn the_built_in_datum_reads() {
        assert!(Datum::from_navsat_yaml(BUILT_IN_NAVSAT_YAML).is_ok());
    }
}

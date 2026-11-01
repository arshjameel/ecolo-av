//! hw_panel: the hardware screen. One program: a raylib window, and the ROS node `hw_panel`.
//!
//! Every sensor's number straight from its own node, with OK / STALE / NEVER, and very
//! simple buttons for the actuators (docs/designs/2026-10-29_two-screens.md): Speed +/−,
//! the steering arrows, CENTRE, STOP, GO held to drive, and the E-STOP. The buttons go
//! to twist_mux's `buttons` slot (45), then ego and the spine, never around them; the
//! speed buttons work only on SIM and BENCH (and, once TODO 49b is answered, the car on
//! stands). debug_hud, the seeing screen, stays read-only: only this program publishes.
//!
//! Subscribes  (names from config/hw_panel.yaml, or ROS parameters)
//!             vehicle_status_topic  aav_interfaces/VehicleStatus   mode, armed, e-stop, speed
//!             steering_topic, throttle_topic, brake_topic  aav_interfaces/ActuatorState  seen
//!             wheels_topic          aav_interfaces/WheelSpeeds     each wheel
//!             imu_topic             sensor_msgs/Imu                acceleration, turn rates
//!             gps_topic             sensor_msgs/NavSatFix          the fix
//!             lidar_topic, radar_topic  sensor_msgs/PointCloud2    counted: Hz (best effort, keep last 1)
//!             camera_topic          sensor_msgs/Image              counted: Hz (best effort, keep last 1)
//!             battery_topic         sensor_msgs/BatteryState       volts
//!             source_topic          aav_interfaces/DataSource      latched: SIM, BENCH, REAL
//!             estop_topic           std_msgs/Bool                  an e-stop zeroes the buttons
//!             control_topic         geometry_msgs/TwistStamped     twist_mux's output: a takeover
//!             geofence_lock_topic   std_msgs/Bool                  shown: it holds the buttons
//!             no_timeout_lock_topic std_msgs/Bool                  latched: the stands rule
//! Publishes   buttons_topic         geometry_msgs/TwistStamped     only while GO is held (and 0.5 s
//!                                                                  of zeros after), at 10 Hz, and only
//!                                                                  while the window draws
//!             estop_topic           std_msgs/Bool                  E-STOP: true 5 times, never false
//! Parameters  the topics above; stale_after (s), speed_step (m/s), steer_step (rad),
//!             creep (m/s), sim_cap (m/s), stands_cap (m/s)
//!
//! raylib owns the main thread (macOS needs that); rclrs spins on its own thread. `--demo`
//! swaps the ROS node for a pretend car. The logic is in panel-core.
//!
//! usage: hw_panel [--demo] [--stands] [--fullscreen] [--monitor NAME|N] [--list-monitors]
//!                 [--config FILE] [--vehicle FILE] [--screenshot FILE [--after S]]
//!                 [--self-test SPEED:SECONDS] [--ros-args ...]

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
use hud_core::screen::Monitor;
use panel_core::{
    buttons::{Buttons, Press, Steps, World, Zeroed},
    config::PanelConfig,
    gate::{self, Caps, Gate, Situation},
    rules::Kind,
    state::{self as panel_state, PanelState, Update},
    vehicle::Vehicle,
};
use raylib::prelude::*;

use crate::{
    demo::Demo,
    source::DataSource,
    ui::{Button, Frame, Ui},
};

const USAGE: &str = "\
usage: hw_panel [--demo] [--stands] [--fullscreen] [--monitor NAME|N] [--list-monitors]
                [--config FILE] [--vehicle FILE] [--screenshot FILE [--after S]]
                [--self-test SPEED:SECONDS] [--ros-args ...]

  --demo             a pretend car: no ROS, no car
  --stands           the car's wheels are off the ground (the speed buttons stay off on
                     the real car until TODO 49b: a driven-wheel speed)
  --fullscreen       fill the screen (F11 switches too)
  --monitor NAME|N   open on this monitor: its name exactly, or its number (0 = the first)
  --list-monitors    print the monitors' numbers, names and sizes, then quit
  --config FILE      topic names from FILE instead of the built-in config/hw_panel.yaml
  --vehicle FILE     the car from FILE instead of the built-in vehicle.yaml
  --screenshot FILE  save the window as a PNG after --after seconds (4), then quit
  --self-test V:S    for checks: press Speed + up to V m/s, hold GO for S seconds, let go;
                     through the same buttons and rules; prints asked and seen each 0.5 s
  --ros-args ...     passed to ROS, e.g. --ros-args -p gps_topic:=/x

keys: Space held = GO (the car moves only while it is held), Up or + / Down or -
      speed, Left / Right steer, C centre, S stop, E e-stop, F11 full screen";

/// The command line, before `--ros-args`.
#[derive(Debug, Default)]
struct Options {
    demo: bool,
    stands: bool,
    fullscreen: bool,
    monitor: Option<Monitor>,
    list_monitors: bool,
    config: Option<PathBuf>,
    vehicle: Option<PathBuf>,
    screenshot: Option<PathBuf>,
    after: Option<f64>,
    self_test: Option<SelfTest>,
}

impl Options {
    /// Reads our own options; stops at `--ros-args`, which belongs to ROS.
    fn parse(args: &[String]) -> Result<Self> {
        let mut opts = Self::default();
        let mut it = args.iter().skip(1);
        while let Some(arg) = it.next() {
            let mut value = |what: &str| -> Result<String> {
                Ok(it
                    .next()
                    .with_context(|| format!("{what} needs a value"))?
                    .clone())
            };
            match arg.as_str() {
                "--ros-args" => break,
                "--demo" => opts.demo = true,
                "--stands" => opts.stands = true,
                "--fullscreen" => opts.fullscreen = true,
                "--list-monitors" => opts.list_monitors = true,
                "--monitor" => opts.monitor = Some(value("--monitor")?.parse()?),
                "--config" => opts.config = Some(value("--config")?.into()),
                "--vehicle" => opts.vehicle = Some(value("--vehicle")?.into()),
                "--screenshot" => opts.screenshot = Some(value("--screenshot")?.into()),
                "--after" => {
                    let s: f64 = value("--after")?
                        .parse()
                        .context("--after: seconds, such as 20")?;
                    if !(s.is_finite() && s >= 0.0) {
                        bail!("--after must be 0 or more seconds");
                    }
                    opts.after = Some(s);
                }
                "--self-test" => opts.self_test = Some(SelfTest::parse(&value("--self-test")?)?),
                "-h" | "--help" => {
                    println!("{USAGE}");
                    std::process::exit(0);
                }
                other => bail!("unknown option {other:?}\n{USAGE}"),
            }
        }
        if opts.after.is_some() && opts.screenshot.is_none() {
            bail!("--after needs --screenshot");
        }
        Ok(opts)
    }
}

/// `--self-test V:S`: Speed + up to V, GO held for S seconds, then let go; for checks
/// and screenshots. It goes through the same buttons and the same gate as a person.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SelfTest {
    speed: f64,
    hold: Duration,
    /// When GO was first held.
    held_at: Option<Instant>,
    /// When it let go.
    done_at: Option<Instant>,
    printed: Option<Instant>,
}

/// After the self-test lets go, the window stays this long (to see the car stop).
const SELF_TEST_TAIL: Duration = Duration::from_secs(3);

impl SelfTest {
    fn parse(s: &str) -> Result<Self> {
        let (v, t) = s
            .split_once(':')
            .context("--self-test: SPEED:SECONDS, such as 0.3:8")?;
        let speed: f64 = v.parse().context("--self-test: the speed, m/s")?;
        let secs: f64 = t.parse().context("--self-test: the seconds")?;
        if !(speed.is_finite() && speed > 0.0 && secs.is_finite() && secs > 0.0) {
            bail!("--self-test: the speed and the seconds must be above 0");
        }
        Ok(Self {
            speed,
            hold: Duration::from_secs_f64(secs),
            held_at: None,
            done_at: None,
            printed: None,
        })
    }

    /// This frame's presses and whether GO is held. Waits for the speed buttons to be on,
    /// presses Speed + up to the speed, then holds GO like a person holding Space: it
    /// never lets go or presses again before the time is up, whatever zeroes the buttons.
    fn frame(&mut self, now: Instant, buttons: &Buttons, gate: &Gate) -> (Option<Press>, bool) {
        if self.done_at.is_some() {
            return (None, false);
        }
        if let Some(held_at) = self.held_at {
            if now.saturating_duration_since(held_at) >= self.hold {
                self.done_at = Some(now);
                return (None, false);
            }
            return (None, true);
        }
        if !matches!(gate, Gate::On { .. }) {
            return (None, false);
        }
        if buttons.asked_speed() + 1e-9 < self.speed {
            return (Some(Press::SpeedUp), false);
        }
        self.held_at = Some(now);
        (None, true)
    }

    fn finished(&self, now: Instant) -> bool {
        self.done_at
            .is_some_and(|t| now.saturating_duration_since(t) >= SELF_TEST_TAIL)
    }

    /// One line every half second: what was asked, what was seen, what twist_mux passed.
    fn print(&mut self, now: Instant, state: &PanelState, buttons: &Buttons) {
        if self
            .printed
            .is_some_and(|t| now.saturating_duration_since(t) < Duration::from_millis(500))
        {
            return;
        }
        self.printed = Some(now);
        let seen = state
            .seen_speed(now)
            .map_or_else(|| "-".to_owned(), |v| format!("{v:.3}"));
        let mux = state
            .mux
            .filter(|(at, _)| now.saturating_duration_since(*at) < Duration::from_millis(500))
            .map_or_else(|| "-".to_owned(), |(_, c)| format!("{:.3}", c.speed));
        let zero = buttons
            .last_zero()
            .map_or_else(String::new, |z| format!(" zeroed: {z}"));
        println!(
            "hw_panel self-test: go {} asked {:.2} seen {seen} mux {mux}{zero}",
            buttons.is_driving(),
            buttons.asked_speed()
        );
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
        Some(path) => PanelConfig::from_yaml(&read(path)?)
            .with_context(|| format!("in {}", path.display()))?,
        None => PanelConfig::default(),
    };
    let vehicle = match &opts.vehicle {
        Some(path) => Vehicle::from_vehicle_yaml(&read(path)?)
            .with_context(|| format!("in {}", path.display()))?,
        None => Vehicle::default(),
    };
    let (source, config) = open_source(&opts, args, config)?;
    run(opts, source, &config, &vehicle)
}

/// The demo, or the ROS node (which may rename topics from its parameters).
fn open_source(
    opts: &Options,
    args: Vec<String>,
    config: PanelConfig,
) -> Result<(Box<dyn DataSource>, PanelConfig)> {
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
        bail!("this hw_panel was built without ROS (--no-default-features): run it with --demo")
    }
}

/// raylib's keys for each press.
const PRESS_KEYS: [(KeyboardKey, Press); 10] = [
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

/// The panel between the source and the window: news in, where it may drive, the
/// buttons, what to send. The window adds the keys, the mouse and the drawing; the
/// Gazebo check drives it without a window.
struct Panel {
    state: PanelState,
    buttons: Buttons,
    last_kind: Option<Kind>,
    caps: Caps,
    vehicle: Vehicle,
    stands: bool,
}

impl Panel {
    fn new(config: &PanelConfig, vehicle: &Vehicle, stands: bool) -> Self {
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
        }
    }

    /// Takes what arrived; a source change zeroes the buttons (not the first
    /// announcement). Returns where the speed buttons may drive now.
    fn news(&mut self, source: &mut dyn DataSource, now: Instant) -> Gate {
        for update in source.drain(now) {
            if let Update::Mux(c) = &update {
                self.buttons.observe_mux(*c, now);
            }
            self.state.apply(update, now);
        }
        // Unknown to anything is the first announcement, not a change.
        let kind = self.state.kind();
        if self
            .last_kind
            .is_some_and(|k| k != kind && k != Kind::Unknown)
        {
            self.buttons.zero(Zeroed::SourceChanged);
        }
        self.last_kind = Some(kind);
        gate::gate(
            &Situation {
                kind,
                stands: self.stands,
                speed_wheels: self.vehicle.speed_wheels.as_ref(),
                no_timeout_lock: self.state.no_timeout_lock,
                // Not read yet (a parameter of the legacy bridge): unknown.
                speed_loop_mode: None,
            },
            self.caps,
        )
    }

    /// This frame's presses and GO; returns what to send now.
    fn act(
        &mut self,
        now: Instant,
        gate: &Gate,
        presses: impl IntoIterator<Item = Press>,
        go: bool,
    ) -> Option<panel_core::buttons::Command> {
        for p in presses {
            self.buttons.press(p, now, gate);
        }
        self.buttons.set_go(go, now);
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

/// The window's loop: news in, the buttons, one frame drawn, the command out.
fn run(
    mut opts: Options,
    mut source: Box<dyn DataSource>,
    config: &PanelConfig,
    vehicle: &Vehicle,
) -> Result<()> {
    let (mut rl, thread) = raylib::init()
        .size(1280, 860)
        .title("AAV hardware panel")
        .resizable()
        .msaa_4x()
        .highdpi()
        .log_level(TraceLogLevel::LOG_WARNING)
        .build();
    rl.set_exit_key(None);
    rl.set_target_fps(60);
    rl.set_window_min_size(1100, 760);
    window::place(&mut rl, opts.monitor.as_ref(), opts.fullscreen)?;
    let ui = Ui::new(&mut rl, &thread);
    let mut panel = Panel::new(config, vehicle, opts.stands);
    let started = Instant::now();
    let screenshot_after = Duration::from_secs_f64(opts.after.unwrap_or(4.0));

    while !rl.window_should_close() && !source.has_stopped() {
        let now = Instant::now();
        let gate = panel.news(source.as_mut(), now);

        // The mouse and the keys. A window without focus never holds GO.
        let focused = rl.is_window_focused();
        let w = rl.get_screen_width() as f32;
        let mouse = rl.get_mouse_position();
        let under = ui::layout(w)
            .into_iter()
            .find(|(_, r)| r.check_collision_point_rec(mouse))
            .map(|(b, _)| b);
        let mouse_down = rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT);
        let held = under.filter(|_| mouse_down);
        let mut presses: Vec<Press> = PRESS_KEYS
            .iter()
            .filter(|(k, _)| rl.is_key_pressed(*k))
            .map(|&(_, p)| p)
            .collect();
        let clicked = under.filter(|_| rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT));
        if let Some(Button::Press(p)) = clicked {
            presses.push(p);
        }
        if clicked == Some(Button::Estop) || rl.is_key_pressed(KeyboardKey::KEY_E) {
            source.press_estop();
        }
        if rl.is_key_pressed(KeyboardKey::KEY_F11) {
            window::toggle_fullscreen(&mut rl);
        }
        let mut go =
            focused && (rl.is_key_down(KeyboardKey::KEY_SPACE) || held == Some(Button::Go));
        if let Some(test) = &mut opts.self_test {
            let (press, hold) = test.frame(now, &panel.buttons, &gate);
            presses.extend(press);
            go |= hold;
        }
        let command = panel.act(now, &gate, presses, go);

        let note = panel.wheels_note();
        let frame = Frame {
            now,
            label: source.label(),
            gate: &gate,
            buttons: &panel.buttons,
            held,
            wheels_note: note.as_deref(),
            stands: opts.stands,
        };
        let mut d = rl.begin_drawing(&thread);
        ui.draw(&mut d, &panel.state, &frame);
        drop(d);
        source.frame_drawn(command);

        if let Some(test) = &mut opts.self_test {
            test.print(now, &panel.state, &panel.buttons);
            if test.finished(now) && opts.screenshot.is_none() {
                break;
            }
        }
        if let Some(path) = &opts.screenshot {
            if now.duration_since(started) > screenshot_after {
                save_screenshot(&mut rl, &thread, path)?;
                break;
            }
        }
    }
    Ok(())
}

/// Saves what the window shows as a PNG.
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
    use panel_core::gate::Off;

    fn args(list: &[&str]) -> Vec<String> {
        std::iter::once("hw_panel")
            .chain(list.iter().copied())
            .map(String::from)
            .collect()
    }

    #[test]
    fn options_stop_at_ros_args_and_check_their_values() {
        let o = Options::parse(&args(&["--demo", "--stands", "--ros-args", "--x"])).expect("ok");
        assert!(o.demo && o.stands);
        let o = Options::parse(&args(&["--self-test", "0.3:8"])).expect("valid");
        let t = o.self_test.expect("parsed");
        assert_eq!((t.speed, t.hold), (0.3, Duration::from_secs(8)));
        assert!(Options::parse(&args(&["--self-test", "0.3"])).is_err());
        assert!(Options::parse(&args(&["--self-test", "-1:8"])).is_err());
        assert!(Options::parse(&args(&["--after", "2"])).is_err());
        assert!(Options::parse(&args(&["--bogus"])).is_err());
    }

    #[test]
    fn the_self_test_waits_for_the_gate_then_presses_and_holds() {
        let steps = Steps {
            speed: 0.1,
            steer: 0.05,
            creep: 0.1,
            max_steer: 0.5,
            wheelbase: 1.57,
        };
        let mut b = Buttons::new(steps);
        let mut test = SelfTest::parse("0.3:1").expect("valid");
        let t = Instant::now();
        let off = Gate::Off(Off::Unannounced);
        assert_eq!(test.frame(t, &b, &off), (None, false));
        let on = Gate::On {
            cap: 1.0,
            watch_unseen: false,
        };
        for _ in 0..3 {
            let (p, go) = test.frame(t, &b, &on);
            assert_eq!((p, go), (Some(Press::SpeedUp), false));
            b.press(Press::SpeedUp, t, &on);
        }
        assert_eq!(test.frame(t, &b, &on), (None, true));
        // Zeroed while held (a takeover): it keeps holding, as a person on Space would,
        // and never presses Speed + again.
        b.zero(Zeroed::TakenOver);
        assert_eq!(test.frame(t, &b, &off), (None, true));
        assert_eq!(
            test.frame(t + Duration::from_secs(2), &b, &on),
            (None, false)
        );
        assert!(test.finished(t + Duration::from_secs(6)));
    }

    /// The Gazebo check without a window (a locked screen can't open one): the real node
    /// and the same buttons, a frame every 33 ms, the self-test's presses and GO.
    /// `HW_PANEL_SELF_TEST=0.3:8 cargo test --release -p panel-app -- --ignored
    /// --nocapture gazebo_self_test`, beside a running sim on the same domain.
    #[cfg(feature = "ros")]
    #[test]
    #[ignore = "needs a running Gazebo on this ROS domain"]
    fn gazebo_self_test() -> Result<()> {
        let spec = std::env::var("HW_PANEL_SELF_TEST").unwrap_or_else(|_| "0.3:8".into());
        let mut test = SelfTest::parse(&spec)?;
        let config = PanelConfig::default();
        let (source, config) = ros::RosSource::start(vec!["hw_panel".into()], config)?;
        let mut source: Box<dyn DataSource> = Box::new(source);
        let mut panel = Panel::new(&config, &Vehicle::default(), false);
        let started = Instant::now();
        loop {
            let now = Instant::now();
            let gate = panel.news(source.as_mut(), now);
            let (press, go) = test.frame(now, &panel.buttons, &gate);
            let command = panel.act(now, &gate, press, go);
            source.frame_drawn(command);
            test.print(now, &panel.state, &panel.buttons);
            if test.finished(now) || now.duration_since(started) > Duration::from_secs(120) {
                break;
            }
            std::thread::sleep(Duration::from_millis(33));
        }
        assert!(
            test.finished(Instant::now()),
            "the speed buttons never came on"
        );
        Ok(())
    }

    /// A source that hands out the updates it is given, once.
    struct Script(Vec<Update>);

    impl DataSource for Script {
        fn label(&self) -> &'static str {
            "TEST"
        }
        fn drain(&mut self, _now: Instant) -> Vec<Update> {
            std::mem::take(&mut self.0)
        }
        fn press_estop(&mut self) {}
        fn frame_drawn(&mut self, _command: Option<panel_core::buttons::Command>) {}
    }

    fn announce(origin: hud_core::origin::Origin) -> Update {
        Update::Source(hud_core::origin::Announcement {
            announcer: "gazebo_adapter".into(),
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
        let mut panel = Panel::new(&PanelConfig::default(), &Vehicle::default(), false);
        let mut quiet = Script(vec![]);
        assert!(matches!(panel.news(&mut quiet, t), Gate::Off(_)));
        // Gazebo announces late, while GO is already held: no zero.
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

    #[test]
    fn the_keys_say_what_the_help_says() {
        assert!(USAGE.contains("Space held = GO") && USAGE.contains("C centre"));
        let centre = PRESS_KEYS.iter().find(|(_, p)| *p == Press::Centre);
        assert_eq!(centre.map(|(k, _)| *k), Some(KeyboardKey::KEY_C));
    }
}

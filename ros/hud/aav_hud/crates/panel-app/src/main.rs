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
//!             steering_raw_topic    aav_interfaces/ActuatorRaw     the calibration page: counts, the
//!                                                                  firmware's calibration state
//!             calibration_line_topic std_msgs/String               the bridge's line: what, why not
//!             ego_command_topic, car_command_topic  ackermann_msgs/AckermannDriveStamped  the GRAPHS
//!                                                                  page: ego's and the mux's command
//!             link_frame_topic      aav_interfaces/LinkFrame       the GRAPHS page: what the Pi wrote
//!             uplink_health_topic   aav_interfaces/UplinkHealth    the GRAPHS page: the uplink's counts
//! Publishes   buttons_topic         geometry_msgs/TwistStamped     only while GO is held (and 0.5 s
//!                                                                  of zeros after), at 10 Hz, and only
//!                                                                  while the window draws
//!             estop_topic           std_msgs/Bool                  E-STOP: true 5 times, never false
//!             steering_calibration_topic aav_interfaces/SteeringCalibration  the calibration page's
//!                                                                  limp or jog, 10 Hz while it asks and
//!                                                                  the window draws; then MODE_OFF 0.5 s
//! Parameters  the topics above; stale_after (s), speed_step (m/s), steer_step (rad),
//!             creep (m/s), sim_cap (m/s), stands_cap (m/s), theme (initial_d, mocha, latte, calm);
//!             the calibration page's jog_step, jog_min, jog_max, sweep_max_error (counts),
//!             sweep_rate (counts/s), sweep_stall (s); easter_egg (the little robot, true),
//!             easter_egg_after (s); esp32_url (the ESP32's WebSocket, read only; empty: off)
//! Reads       --esp32 ws://...: the ESP32's JSON status lines (firmware/status), on a
//!             thread of its own; it only ever sends the WebSocket's opening, a pong or a
//!             close. They feed the GRAPHS page only, never the buttons or the gate
//! Writes      SAVE on the calibration page: the result, for a person to copy, in
//!             $XDG_DATA_HOME/aav/steering-calibration/ (~/.local/share/...); never
//!             vehicle.yaml or the firmware
//!
//! raylib owns the main thread (macOS needs that); rclrs spins on its own thread. `--demo`
//! swaps the ROS node for a pretend car. The logic is in panel-core.
//!
//! usage: hw_panel [--demo [--demo-bind COUNTS]] [--stands] [--fullscreen] [--monitor NAME|N]
//!                 [--list-monitors] [--config FILE] [--vehicle FILE]
//!                 [--page drive|calibration|graphs|measure] [--esp32 ws://HOST:PORT/]
//!                 [--screenshot FILE [--after S]] [--self-test SPEED:SECONDS]
//!                 [--cal-self-test] [--robot-now] [--ros-args ...]

use std::{
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use hud_app::window;
use hud_core::screen::Monitor;
use panel_app::{
    cal_script::CalScript, open_source, source::DataSource, ui::Page, HardwarePage, Host,
    SourceOptions,
};
use panel_core::{
    buttons::{Buttons, Press},
    config::PanelConfig,
    gate::Gate,
    state::PanelState,
    vehicle::Vehicle,
};
use raylib::prelude::*;

const USAGE: &str = "\
usage: hw_panel [--demo [--demo-bind COUNTS]] [--stands] [--fullscreen] [--monitor NAME|N]
                [--list-monitors] [--config FILE] [--vehicle FILE]
                [--page drive|calibration|graphs|measure] [--esp32 ws://HOST:PORT/]
                [--screenshot FILE [--after S]] [--self-test SPEED:SECONDS]
                [--cal-self-test] [--robot-now] [--ros-args ...]

  --demo             a pretend car: no ROS, no car
  --demo-bind COUNTS the demo's steering has a stiff spot there (the check finds it)
  --page PAGE        open on this page: drive, calibration (steering), graphs or measure
  --esp32 URL        also read the ESP32's status lines from its WebSocket, such as
                     ws://192.168.4.1:81/ (read only: the GRAPHS page; esp32_url too)
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
  --cal-self-test    with --demo only: the calibration page goes limp, marks each end and
                     the centre, holds, runs the lock-to-lock check, prints the result
                     (with --demo-bind it must stop on the stiff spot)
  --robot-now        the little robot (the easter egg) at once: it drives across, then
                     its eyes peek after 1 s of quiet (for screenshots)
  --ros-args ...     passed to ROS, e.g. --ros-args -p gps_topic:=/x

keys: Space held = GO (the car moves only while it is held), Up or + / Down or -
      speed, Left / Right steer, C centre, S stop, E e-stop, F11 full screen,
      Tab the next page.
      Steering calibration page: L limp, H hold, Left / Right step, 1 2 3 mark left,
      centre, right, K lock-to-lock check, W save, O end.
      Graphs page: P freeze (an e-stop freezes too)";

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
    page: Option<Page>,
    demo_bind: Option<u16>,
    cal_self_test: bool,
    robot_now: bool,
    esp32: Option<String>,
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
                "--cal-self-test" => opts.cal_self_test = true,
                "--robot-now" => opts.robot_now = true,
                "--page" => {
                    opts.page = Some(match value("--page")?.as_str() {
                        "calibration" => Page::Calibration,
                        "drive" => Page::Drive,
                        "graphs" => Page::Graphs,
                        "measure" => Page::Measure,
                        other => {
                            bail!("--page: drive, calibration, graphs or measure, not {other:?}")
                        }
                    })
                }
                "--esp32" => {
                    let url = value("--esp32")?;
                    panel_core::ws::Url::parse(&url).context("--esp32")?;
                    opts.esp32 = Some(url);
                }
                "--demo-bind" => {
                    let c: u16 = value("--demo-bind")?
                        .parse()
                        .context("--demo-bind: counts, such as 2800")?;
                    opts.demo_bind = Some(c);
                }
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
        if (opts.cal_self_test || opts.demo_bind.is_some()) && !opts.demo {
            bail!("--cal-self-test and --demo-bind need --demo: they are for the pretend car only");
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
/// `--robot-now`: the quiet before the eyes peek.
const ROBOT_NOW_AFTER: Duration = Duration::from_secs(1);

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
    let source_options = SourceOptions {
        demo: opts.demo,
        demo_bind: opts.demo_bind,
        esp32: opts.esp32.clone(),
    };
    // The ESP32's status lines beside it: --esp32, or esp32_url (a ROS parameter too).
    let (source, config) = open_source(args, config, &source_options, "hw_panel")?;
    run(opts, source, config, &vehicle)
}

/// The window's loop: news in, the buttons, one frame drawn, the command out. The page
/// itself is panel-app's [`HardwarePage`], the same as the map window's HARDWARE page.
fn run(
    mut opts: Options,
    source: Box<dyn DataSource>,
    config: PanelConfig,
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
    let started = Instant::now();
    // The little robot: it drives across once at start-up, then peeks when all is quiet.
    let mut egg = config.easter_egg_settings();
    if opts.robot_now {
        egg.on = true;
        egg.idle_after = ROBOT_NOW_AFTER;
    }
    let mut page = HardwarePage::new(
        &mut rl,
        &thread,
        source,
        config,
        vehicle,
        opts.stands,
        Host::Standalone,
        egg,
        started,
    );
    if let Some(p) = opts
        .page
        .or(opts.cal_self_test.then_some(Page::Calibration))
    {
        page.show(p, started);
    }
    let mut cal_script = opts
        .cal_self_test
        .then(|| CalScript::new(opts.demo_bind.is_some()));
    let screenshot_after = Duration::from_secs_f64(opts.after.unwrap_or(4.0));

    while !rl.window_should_close() && !page.has_stopped() {
        let now = Instant::now();
        page.news(now);
        let mut input = page.read_input(&mut rl, now);
        if let Some(script) = &mut cal_script {
            let panel = page.panel();
            let position = panel.position(now);
            input
                .cal_presses
                .extend(script.frame(now, panel.cal(), position));
        }
        if rl.is_key_pressed(KeyboardKey::KEY_F11) {
            window::toggle_fullscreen(&mut rl);
        }
        if let Some(test) = &mut opts.self_test {
            let (press, hold) = test.frame(now, page.panel().buttons(), page.gate());
            input.presses.extend(press);
            input.go |= hold;
        }
        let step = page.step(now, input);
        let mut d = rl.begin_drawing(&thread);
        page.draw(&mut d, now, &step);
        drop(d);
        page.frame_drawn(&step);

        if let Some(script) = &mut cal_script {
            let panel = page.panel();
            script.print(now, panel.cal(), panel.position(now));
            if script.finished(now) && opts.screenshot.is_none() {
                if !script.as_expected(panel.cal()) {
                    bail!("the lock-to-lock check didn't end as it should");
                }
                break;
            }
        }

        if let Some(test) = &mut opts.self_test {
            let panel = page.panel();
            test.print(now, panel.state(), panel.buttons());
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
    use panel_core::{
        buttons::{Steps, Zeroed},
        gate::Off,
    };

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

    /// The Ravensim check without a window (a locked screen can't open one): the real
    /// node and the same buttons, a frame every 33 ms, the self-test's presses and GO.
    /// `HW_PANEL_SELF_TEST=0.3:8 cargo test --release -p panel-app -- --ignored
    /// --nocapture gazebo_self_test`, beside a running sim on the same domain.
    #[cfg(feature = "ros")]
    #[test]
    #[ignore = "needs a running Ravensim on this ROS domain"]
    fn gazebo_self_test() -> Result<()> {
        use panel_app::Panel;
        let spec = std::env::var("HW_PANEL_SELF_TEST").unwrap_or_else(|_| "0.3:8".into());
        let mut test = SelfTest::parse(&spec)?;
        let config = PanelConfig::default();
        let (source, config) = panel_app::ros::RosSource::start(vec!["hw_panel".into()], config)?;
        let mut source: Box<dyn DataSource> = Box::new(source);
        let mut panel = Panel::new(&config, &Vehicle::default(), false);
        let started = Instant::now();
        loop {
            let now = Instant::now();
            let gate = panel.news(source.as_mut(), now);
            let (press, go) = test.frame(now, panel.buttons(), &gate);
            let command = panel.act(now, &gate, press, go);
            source.frame_drawn(command, None);
            test.print(now, panel.state(), panel.buttons());
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

    #[test]
    fn the_cal_options_need_the_demo() {
        assert!(Options::parse(&args(&["--cal-self-test"])).is_err());
        assert!(Options::parse(&args(&["--demo", "--cal-self-test"])).is_ok());
        let o = Options::parse(&args(&["--page", "calibration"])).expect("ok");
        assert_eq!(o.page, Some(Page::Calibration));
        assert!(Options::parse(&args(&["--page", "x"])).is_err());
        let o = Options::parse(&args(&["--page", "graphs"])).expect("ok");
        assert_eq!(o.page, Some(Page::Graphs));
        let o = Options::parse(&args(&["--esp32", "ws://192.168.4.1:81/"])).expect("ok");
        assert_eq!(o.esp32.as_deref(), Some("ws://192.168.4.1:81/"));
        assert!(Options::parse(&args(&["--esp32", "http://x/"])).is_err());
        assert!(Options::parse(&args(&["--esp32", "wss://x/"])).is_err());
    }

    #[test]
    fn the_keys_say_what_the_help_says() {
        assert!(USAGE.contains("K lock-to-lock check") && USAGE.contains("W save"));
        assert!(USAGE.contains("Space held = GO") && USAGE.contains("C centre"));
        assert!(USAGE.contains("P freeze"));
    }
}

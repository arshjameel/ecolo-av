//! drive_to_point, in Rust: drive the car to a point and stop there (point_driver's
//! /drive_to_point). The same command as aav_drive_to_point's point_cli.py.
//!
//! ```text
//! pixi run drive-to-point --sim --ahead 20                 Ravensim: 20 m ahead of the car
//! pixi run drive-to-point --sim --ahead 30 --left 10       ... and 10 m to its left
//! pixi run drive-to-point --sim --x 30 --y 5               a map point (x east, y north, m)
//! pixi run drive-to-point --lat 45.38630 --lon -75.69580   the real car: a GPS point
//! pixi run drive-to-point --lat ... --lon ... --speed 0.5  slower (ego caps it anyway)
//! ```
//!
//! Ctrl-C stops the car (it cancels the goal: zero commands). On the real car
//! (/system/source REAL, or nobody announcing) it shows what it will do and a checklist, and
//! drives only after you type "drive". --sim keeps ROS on this computer
//! (tools/drive_to_point.sh) and uses the simulator's clock.
//!
//! Subscribes  system/source             aav_interfaces/DataSource, latched
//!             localization/odom/global  nav_msgs/Odometry: for --ahead/--left, and the distance
//! Calls       /drive_to_point           aav_interfaces/action/DriveToPoint
//!
//! Exit codes: 0 at B, 2 not there, 1 refused, 130 Ctrl-C.

use std::collections::HashMap;
use std::io::{BufRead, IsTerminal, Write};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use anyhow::Result;
use futures::future::{self, Either};
use point_core::cli::{self, feedback_line, Args, Where, LOT_CHECKLIST, USAGE};
use point_core::rules::{self, Kind};
use rclrs::*;
use ros_env::aav_interfaces::action::{DriveToPoint, DriveToPoint_Goal, DriveToPoint_Result};
use ros_env::aav_interfaces::msg::DataSource;
use ros_env::nav_msgs::msg::Odometry;

fn say(text: &str) {
    println!("{text}");
    let _ = std::io::stdout().flush();
}

/// What the client has heard, and how its goal goes.
#[derive(Default)]
struct Heard {
    sources: HashMap<String, Vec<u8>>,
    odom: Option<(f64, f64, f64)>,
    accepted: Option<bool>,
    cancel: Option<CancellationClient<DriveToPoint>>,
    result: Option<DriveToPoint_Result>,
}

fn lock(h: &Mutex<Heard>) -> MutexGuard<'_, Heard> {
    h.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// Spin up to `seconds`; true once `until` holds (or, with no condition, at the end).
fn spin_for(
    executor: &mut Executor,
    seconds: f64,
    until: Option<&dyn Fn() -> bool>,
    stop: &AtomicBool,
) -> bool {
    let end = Instant::now() + Duration::from_secs_f64(seconds);
    while Instant::now() < end && !stop.load(Ordering::SeqCst) {
        let _ = executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
        if until.is_some_and(|u| u()) {
            return true;
        }
    }
    until.is_none()
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.iter().any(|a| a == "-h" || a == "--help") {
        say(USAGE);
        say("\nDrive the car to a point and stop there (point_driver's /drive_to_point).");
        return ExitCode::SUCCESS;
    }
    let a = match cli::parse(&argv) {
        Ok(a) => a,
        Err(why) => {
            eprintln!("{USAGE}\ndrive-to-point: error: {why}");
            return ExitCode::from(2);
        }
    };
    match run(&a) {
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("drive-to-point: {e:#}");
            ExitCode::from(1)
        }
    }
}

fn run(a: &Args) -> Result<u8> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let sim = if a.sim { "true" } else { "false" };
    let node = executor.create_node("point_cli".arguments([
        "--ros-args",
        "-p",
        &format!("use_sim_time:={sim}"),
    ]))?;
    let heard = Arc::new(Mutex::new(Heard::default()));
    let h = Arc::clone(&heard);
    let _source = node.create_subscription(
        "system/source".keep_last(1).reliable().transient_local(),
        move |m: DataSource| {
            lock(&h).sources.insert(
                m.announcer.clone(),
                m.sensors.iter().map(|s| s.source).collect(),
            );
        },
    )?;
    let h = Arc::clone(&heard);
    let _odom = node.create_subscription("localization/odom/global", move |m: Odometry| {
        let p = &m.pose.pose;
        let q = &p.orientation;
        let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        lock(&h).odom = Some((p.position.x, p.position.y, yaw));
    })?;
    let action = node.create_action_client::<DriveToPoint>("drive_to_point")?;
    let never = AtomicBool::new(false);

    let h = Arc::clone(&heard);
    spin_for(
        &mut executor,
        3.0,
        Some(&|| {
            let g = lock(&h);
            !g.sources.is_empty() && g.odom.is_some()
        }),
        &never,
    );
    let (kind, pose) = {
        let g = lock(&heard);
        (
            rules::source_kind(g.sources.values().flatten().copied()),
            g.odom,
        )
    };
    if a.sim && kind != Kind::Sim {
        say(&format!(
            "REFUSED: --sim, but /system/source says {}. Start Gazebo first (pixi run sim ...)",
            kind.word()
        ));
        return Ok(1);
    }
    if !a.sim && kind == Kind::Sim {
        say("REFUSED: Gazebo announces SIM on /system/source: add --sim");
        return Ok(1);
    }
    let mut goal = DriveToPoint_Goal {
        cruise_speed: a.speed as f32,
        ..DriveToPoint_Goal::default()
    };
    let (x, y) = match (a.target, pose) {
        (Where::LatLon(lat, lon), _) => {
            goal.use_lat_lon = true;
            goal.latitude = lat;
            goal.longitude = lon;
            (f64::NAN, f64::NAN)
        }
        (Where::Map(x, y), _) => (x, y),
        (Where::Ahead(ahead, left), Some(p)) => cli::ahead_of(p, ahead, left),
        (Where::Ahead(..), None) => {
            say("REFUSED: no /localization/odom/global: --ahead needs the car's position");
            return Ok(1);
        }
    };
    let where_ = if goal.use_lat_lon {
        format!("lat {:.7}, lon {:.7}", goal.latitude, goal.longitude)
    } else {
        goal.x = x;
        goal.y = y;
        format!("map x {x:.2} m, y {y:.2} m")
    };
    let far = match pose {
        Some((x, y, _)) if !goal.use_lat_lon => format!(
            ", {:.1} m from the car",
            point_core::hypot(goal.x - x, goal.y - y)
        ),
        _ => String::new(),
    };
    say(&format!("B: {where_}{far} ({})", kind.word()));
    if matches!(kind, Kind::Real | Kind::Unknown) {
        say("");
        say("THE REAL CAR. Check, out loud:");
        for item in LOT_CHECKLIST {
            say(&format!("  [ ] {item}"));
        }
        say("  [ ] The straight line to B, and the turn to face it, are clear");
        say("To stop at any time: Ctrl-C here, the e-stop, or the joystick deadman.");
        if !std::io::stdin().is_terminal() {
            say("REFUSED: the real car needs a person at the keyboard to confirm");
            return Ok(1);
        }
        print!(
            "Type \"{}\" to start (anything else cancels): ",
            rules::CONFIRM_WORD
        );
        let _ = std::io::stdout().flush();
        let mut typed = String::new();
        let read = std::io::stdin().lock().read_line(&mut typed);
        let typed = match read {
            Ok(n) if n > 0 => Some(typed),
            _ => None,
        };
        if !rules::confirmed(typed.as_deref()) {
            say("Cancelled: nothing moved.");
            return Ok(1);
        }
        goal.confirmed = true;
    }
    // From here Ctrl-C cancels the goal (a second one can't cut that short).
    let stop = Arc::new(AtomicBool::new(false));
    let s = Arc::clone(&stop);
    ctrlc::set_handler(move || s.store(true, Ordering::SeqCst))?;
    let ready = spin_for(
        &mut executor,
        5.0,
        Some(&|| action.server_is_available().unwrap_or(false)),
        &stop,
    );
    if stop.load(Ordering::SeqCst) {
        say("STOPPING: Ctrl-C");
        return Ok(130);
    }
    if !ready {
        say(
            "REFUSED: no /drive_to_point action: is point_driver running? (sim, bench and car \
             start it)",
        );
        return Ok(1);
    }

    let h = Arc::clone(&heard);
    let client = Arc::clone(&action);
    let _flow = executor.commands().run(async move {
        let Some(gc) = client.request_goal(goal).await else {
            lock(&h).accepted = Some(false);
            return;
        };
        {
            let mut g = lock(&h);
            g.accepted = Some(true);
            g.cancel = Some(gc.cancellation.clone());
        }
        let GoalClient {
            mut feedback,
            result,
            ..
        } = gc;
        let mut result = Box::pin(result);
        let mut last: Option<Instant> = None;
        loop {
            match future::select(Box::pin(feedback.recv()), result.as_mut()).await {
                Either::Left((Some(f), _)) => {
                    if last.is_none_or(|t| t.elapsed() >= Duration::from_millis(500)) {
                        last = Some(Instant::now());
                        say(&feedback_line(
                            f.phase,
                            f64::from(f.distance_left),
                            f64::from(f.cross_track),
                            f64::from(f.measured_speed),
                            f64::from(f.commanded_speed),
                        ));
                    }
                }
                Either::Left((None, _)) => {
                    let (_, r) = result.await;
                    lock(&h).result = Some(r);
                    break;
                }
                Either::Right(((_, r), _)) => {
                    lock(&h).result = Some(r);
                    break;
                }
            }
        }
    });

    let h = Arc::clone(&heard);
    spin_for(
        &mut executor,
        10.0,
        Some(&|| lock(&h).accepted.is_some()),
        &stop,
    );
    if !stop.load(Ordering::SeqCst) && lock(&heard).accepted != Some(true) {
        say("REFUSED: point_driver would not take it (another drive is running?)");
        return Ok(1);
    }
    let h = Arc::clone(&heard);
    while !stop.load(Ordering::SeqCst) && lock(&h).result.is_none() {
        let _ = executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
    }
    if stop.load(Ordering::SeqCst) {
        say("STOPPING: Ctrl-C");
        let cancel = lock(&heard).cancel.clone();
        if let Some(cancel) = cancel {
            let _asked = executor.commands().run(cancel.cancel());
            let calm = AtomicBool::new(false);
            spin_for(
                &mut executor,
                10.0,
                Some(&|| lock(&h).result.is_some()),
                &calm,
            );
            if let Some(r) = &lock(&heard).result {
                say(&r.message);
            }
        }
        return Ok(130);
    }
    let r = lock(&heard).result.clone().unwrap_or_default();
    say(&format!(
        "{}{}; {:.1} m driven in {:.1} s",
        if r.success { "DONE: " } else { "NOT THERE: " },
        r.message,
        r.distance_travelled,
        r.duration
    ));
    Ok(if r.success { 0 } else { 2 })
}

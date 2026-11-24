//! rc_pad: a gamepad as the FS-i6X transmitter and its receiver, for spine_sim (23 Nov).
//!
//! It turns a gamepad's sticks and buttons into the channels the car's RC receiver would
//! send (bench-car's `Pad`, in µs), 50 times a second. spine_sim with rc:=true decodes them
//! as the board decodes the receiver's iBUS, so the firmware's RC rules (spine-core) run
//! unchanged: the left stick is the pedal (CH3), the right stick stop and give (CH2) and
//! steering (CH4), R1 the mode switch (CH6), and L1 held the radio lost (the failsafe).
//!
//! No Joy message for half a second (the gamepad unplugged, its node gone) and it stops
//! sending: spine_sim then sees a silent receiver, a lost RC, as on the car.
//!
//! Subscribes  /drivers/rc_pad/joy           sensor_msgs/Joy          joy's game_controller_node
//! Publishes   /vehicle/bench/rc_channels    std_msgs/UInt16MultiArray the 14 iBUS channels, µs
//! Parameters  mode_start (high): where the mode switch starts, high (manual, RC direct)
//!             or low (normal: the right stick hands the car over)

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

use anyhow::{bail, Result};
use bench_car::rc::{Pad, PadLayout, RcMap};
use rclrs::*;
use ros_env::sensor_msgs::msg::Joy;
use ros_env::std_msgs::msg::UInt16MultiArray;

/// How often the pretend receiver sends (ms): a real one sends about every 7 ms.
const PERIOD_MS: u64 = 20;
/// No Joy message for this long, and the receiver goes silent.
const JOY_STALE: Duration = Duration::from_millis(500);

/// The latest Joy message's sticks and buttons, and when it came.
#[derive(Default)]
struct Latest {
    axes: Vec<f32>,
    buttons: Vec<i32>,
    at: Option<Instant>,
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("rc_pad")?;

    let mode_start = node
        .declare_parameter::<Arc<str>>("mode_start")
        .default("high".into())
        .read_only()?
        .get();
    let mode_high = match &*mode_start {
        "high" => true,
        "low" => false,
        other => bail!("mode_start:={other}: use high (manual) or low (normal)"),
    };
    log_info!(
        node.logger(),
        "A gamepad as the FS-i6X: left stick the pedal, right stick stop (down), give (up) \
         and steering, R1 the mode switch (starting {mode_start}), L1 held the radio lost."
    );

    let latest = Arc::new(Mutex::new(Latest::default()));
    let _joy = {
        let latest = Arc::clone(&latest);
        node.create_subscription::<Joy, _>("/drivers/rc_pad/joy", move |j: Joy| {
            let mut l = latest.lock().expect("the pad's lock");
            l.axes = j.axes;
            l.buttons = j.buttons;
            l.at = Some(Instant::now());
        })?
    };
    let channels = node.create_publisher::<UInt16MultiArray>("/vehicle/bench/rc_channels")?;

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        let commands = Arc::clone(executor.commands());
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
            commands.halt_spinning();
        })?;
    }
    let sender = {
        let running = Arc::clone(&running);
        let logger = node.logger().clone();
        thread::spawn(move || -> Result<()> {
            let mut pad = Pad::new(PadLayout::PS4, RcMap::TODAY, mode_high);
            let mut was_sending = false;
            let mut was_high = mode_high;
            while running.load(Ordering::SeqCst) {
                let frame = {
                    let l = latest.lock().expect("the pad's lock");
                    l.at.filter(|at| at.elapsed() < JOY_STALE)
                        .map(|_| pad.channels(&l.axes, &l.buttons))
                };
                if frame.is_some() != was_sending {
                    was_sending = frame.is_some();
                    if was_sending {
                        log_info!(&logger, "The gamepad is here: sending the RC's channels.");
                    } else {
                        log_warn!(&logger, "No gamepad for 0.5 s: the receiver goes silent.");
                    }
                }
                if pad.mode_high() != was_high {
                    was_high = pad.mode_high();
                    let mode = if was_high {
                        "high (manual)"
                    } else {
                        "low (normal)"
                    };
                    log_info!(&logger, "The mode switch: {mode}");
                }
                if let Some(ch) = frame {
                    channels.publish(UInt16MultiArray {
                        data: ch.0.to_vec(),
                        ..UInt16MultiArray::default()
                    })?;
                }
                thread::sleep(Duration::from_millis(PERIOD_MS));
            }
            Ok(())
        })
    };

    executor.spin(SpinOptions::default()).first_error()?;
    running.store(false, Ordering::SeqCst);
    match sender.join() {
        Ok(result) => result,
        Err(_) => bail!("the pad's sender panicked"),
    }
}

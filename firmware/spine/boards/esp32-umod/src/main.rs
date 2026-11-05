//! The spine on today's control board: the SparkFun ESP32 MicroMod (the original ESP32,
//! Xtensa) on the AAV carrier, beside the Motor-Pi. The Rust replacement for
//! `firmware/legacy/aav_umod_esp32`.
//!
//! It puts `spine-board` and `spine-core` on the ESP32's peripherals, with the same pins
//! and devices as the C++ ([`pins`]). The computer's commands come from one of two links,
//! chosen when it is built:
//!
//! | Feature | Link | Use |
//! |---|---|---|
//! | `legacy-uart` (default) | the Motor-Pi's version 2 frames on UART1, with the Orin's heartbeat | `aav_legacy`'s `legacy_uart_bridge` with `frame_version: 2` |
//! | `legacy-uart` + `link-v1` | the Motor-Pi's 2025-26 13-byte frames | a drop-in for the C++: `frame_version: 1`, no heartbeat |
//! | `can` | `firmware/can/aav.dbc` on the TWAI controller | the KEFC plan: the Orin's `spine_bridge` |
//!
//! With `uplink` (off until the hardware lead confirms the wire from TX1, GPIO17, to the
//! Motor-Pi's RX: TODO 50, 48), the link also sends a status frame back every 20 ms
//! (`legacy_link::Uplink`): the mode, the RC mode switch, the faults, the last Orin
//! counter taken, the Jrks' positions and the RC sticks. The same build reads the RC's
//! mode switch with three positions (`ibus::RcMap::THREE_POSITION`): computer, RC via
//! ROS (the sticks go up, a ROS driver turns them into commands) and RC direct.
//!
//! The heartbeat (TODO 50): each version 2 frame carries a counter the Orin's output mux
//! makes and the Motor-Pi only copies. A frame whose counter hasn't changed isn't a
//! command, so 100 ms after the counter stops, the spine's watchdog brakes, cuts the
//! throttle and latches, however many frames still arrive (`spine_board::LegacyLink`).
//!
//! | Part | How |
//! |---|---|
//! | the spine's loop | `SpineBoard::tick` every 10 ms, on Embassy's thread executor |
//! | steering, brake, throttle, direction | the two Jrk G2s, the DAC and the PCA9555, on one blocking I2C bus at 100 kHz |
//! | the RC receiver | iBUS on UART2, read from its 128-byte FIFO each tick (about 28 ms of iBUS) |
//! | the Motor-Pi | UART1, read the same way (`legacy-uart`); with `uplink`, its TX sends the status frame every 20 ms |
//! | CAN | TWAI at 500 kbit/s, polled each tick (`can`) |
//! | the wheel speed | none on this board (the encoder talks to the Motor-Pi): the speed loop runs open loop |
//! | the e-stop's spare contact | an input, with `estop-input`, once it is wired (Q6) |
//! | the hardware watchdog | TIMG1's: fed after every tick; a hang or a panic resets the chip within 100 ms |
//! | the log | UART0, the USB cable: a status line every 200 ms, as the C++ printed, an event line at each change, and (`legacy-uart`) a link line with the frame counts |
//!
//! With `--features qemu`, it runs in Espressif's QEMU instead (`tools/esp32.sh qemu-check`;
//! `docs/esp32-bench.md`, "Before the board: the emulator"). Two things differ, both only
//! in that build: stand-in devices on the I2C bus (`spine_board::StandInBus`: QEMU has
//! none of ours), and a busy-wait for the 10 ms tick instead of Embassy's timer (QEMU's
//! interrupt matrix has no status registers, so esp-hal can't tell which peripheral
//! interrupted). The spine, the UART link, its decoder and the drivers are the same code.
//!
//! After any reset the spine is safe by itself: `SpineBoard::init` puts the throttle at 0
//! and the brake on, and `spine-core` boots latched: it drives only after a stop (a stop
//! frame, or `armed = 0` on CAN), then a drive command.
//!
//! The e-stop rule (Ben): the red button always stops the car, whatever this program does.
//! It stays wired as it is; this program only reads a spare contact. No output of this
//! program is in its path.
#![no_std]
#![no_main]
// For one instruction: turning the floating-point unit on (`enable_fpu`).
#![feature(asm_experimental_arch)]

#[cfg(all(feature = "legacy-uart", feature = "can"))]
compile_error!(
    "pick one link: `legacy-uart` (the default) or `can` (--no-default-features --features can)"
);
#[cfg(not(any(feature = "legacy-uart", feature = "can")))]
compile_error!("pick a link: `legacy-uart` or `can`");
#[cfg(all(feature = "qemu", feature = "can"))]
compile_error!("`qemu` runs the Motor-Pi UART link only: QEMU has no CAN (TWAI)");
#[cfg(all(feature = "link-v1", not(feature = "legacy-uart")))]
compile_error!("`link-v1` is a frame of the Motor-Pi UART link: build it with `legacy-uart`");
#[cfg(all(feature = "uplink", not(feature = "legacy-uart")))]
compile_error!("`uplink` sends on the Motor-Pi UART link's TX: build it with `legacy-uart`");

#[cfg(feature = "can")]
mod can;
mod pins;

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time::Duration;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::gpio::{AnyPin, Level, Output, OutputConfig};
#[cfg(not(feature = "qemu"))]
use esp_hal::i2c::master::{self as i2c, I2c};
use esp_hal::time::Instant;
#[cfg(not(feature = "qemu"))]
use esp_hal::time::Rate;
use esp_hal::timer::timg::{MwdtStage, TimerGroup};
use esp_hal::uart::{self, UartRx};
use log::{info, warn};
use spine_board::{Board, Calibration, CanFrame, Devices, SpineBoard, WheelCount};
#[cfg(feature = "qemu")]
use spine_board::{DeviceTargets, StandInBus};
use spine_core::protocol::Mode;
use spine_core::{Outputs, SpeedFeedback, SpineConfig};
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

// --- Settings -------------------------------------------------------------------------

/// The spine's loop: about 100 times a second (`spine-board`).
const TICK: Duration = Duration::from_millis(10);

/// TIMG1's watchdog: when a tick doesn't end within this, the chip resets. Ten ticks; a
/// tick takes a few milliseconds of I2C.
const WATCHDOG_MS: u64 = 100;

/// I2C to the Jrk G2s, the DAC and the expander: 100 kHz, as the C++ (`Wire`'s default).
#[cfg(not(feature = "qemu"))]
const I2C_KHZ: u32 = 100;

/// The car's top speed at full throttle, for the open speed loop: this board has no wheel
/// speed. The same guess as `legacy.yaml`'s `full_throttle_speed`. TODO(Q4): measure.
const FULL_THROTTLE_SPEED: f32 = 3.0;

/// The brake while nobody drives, from boot on. The C++ kept it full on (3500), and this
/// board has no e-stop input yet to hold it, so it does the same until the team settles
/// spine-core's `IDLE_BRAKE` (proposed 0.0: released). TODO(Ben): decide.
const IDLE_BRAKE: f32 = 1.0;

/// The status line on the USB cable, as often as the C++'s `publishStatus()`.
const STATUS_EVERY_TICKS: u32 = 20;

/// The status LED blinks slowly while commands come in, fast while they don't.
const BLINK_OK_TICKS: u32 = 50;
const BLINK_NO_LINK_TICKS: u32 = 10;

// --- The board ---------------------------------------------------------------------------

/// What this board gives `spine-board`: the clock, the e-stop, the wheel counter, CAN.
struct Esp32Umod {
    #[cfg(feature = "can")]
    can: can::CanPort,
    #[cfg(feature = "estop-input")]
    estop: esp_hal::gpio::Input<'static>,
    leds: Leds,
}

impl Board for Esp32Umod {
    fn now_ms(&mut self) -> u32 {
        now_ms()
    }

    fn estop_open(&mut self) -> bool {
        #[cfg(feature = "estop-input")]
        {
            self.estop.level() == pins::ESTOP_OPEN
        }
        // Not wired yet (Q6): the red button still stops the car on its own.
        #[cfg(not(feature = "estop-input"))]
        {
            false
        }
    }

    /// No wheel encoder on this board: it talks to the Motor-Pi over Ethernet.
    fn wheel(&mut self) -> WheelCount {
        WheelCount::default()
    }

    fn can_receive(&mut self) -> Option<CanFrame> {
        #[cfg(feature = "can")]
        {
            self.can.receive()
        }
        #[cfg(not(feature = "can"))]
        {
            None
        }
    }

    fn can_send(&mut self, _frame: &CanFrame) -> bool {
        #[cfg(feature = "can")]
        {
            self.can.send(_frame)
        }
        #[cfg(not(feature = "can"))]
        {
            false
        }
    }
}

/// The four LEDs the C++ drove.
struct Leds {
    /// GPIO2, on the processor board: the heartbeat.
    status: Output<'static>,
    /// A fault is reported.
    red: Output<'static>,
    /// The computer drives.
    green: Output<'static>,
    /// The RC transmitter drives.
    blue: Output<'static>,
}

impl Leds {
    fn show(&mut self, out: &Outputs, ticks: u32) {
        let level = |on: bool| if on { Level::High } else { Level::Low };
        self.red.set_level(level(out.faults != 0));
        self.green.set_level(level(out.mode == Mode::Computer));
        self.blue.set_level(level(out.mode == Mode::Rc));
        let half_period = if out.mode == Mode::Computer {
            BLINK_OK_TICKS
        } else {
            BLINK_NO_LINK_TICKS
        };
        if ticks.is_multiple_of(half_period) {
            self.status.toggle();
        }
    }
}

/// Turns on the floating-point unit (coprocessor 0), before any interrupt or `f32`.
///
/// esp-hal saves the FPU's registers on every interrupt (`float-save-restore`), and
/// nothing in esp-hal or the runtime turns the FPU on: it relies on CPENABLE's state at
/// reset, which the Xtensa ISA leaves undefined. Espressif's QEMU starts with it off, so
/// the first interrupt was a double exception there (`docs/esp32-bench.md`, "Before the
/// board: the emulator"). Setting it is harmless on a chip that already has it on.
fn enable_fpu() {
    // SAFETY: writes only CPENABLE (bit 0: the FPU), then waits for it to take effect.
    unsafe {
        core::arch::asm!("movi {0}, 1", "wsr.cpenable {0}", "rsync", out(reg) _);
    }
}

/// The 10 ms tick: Embassy's timer on the chip; a busy-wait in QEMU, where interrupts
/// can't be dispatched (the module docs).
struct Pace {
    #[cfg(not(feature = "qemu"))]
    ticker: embassy_time::Ticker,
    #[cfg(feature = "qemu")]
    next: Instant,
}

impl Pace {
    fn new() -> Self {
        Pace {
            #[cfg(not(feature = "qemu"))]
            ticker: embassy_time::Ticker::every(TICK),
            #[cfg(feature = "qemu")]
            next: Instant::now(),
        }
    }

    async fn next(&mut self) {
        #[cfg(not(feature = "qemu"))]
        self.ticker.next().await;
        #[cfg(feature = "qemu")]
        {
            self.next += esp_hal::time::Duration::from_millis(TICK.as_millis());
            while Instant::now() < self.next {}
        }
    }
}

/// What an event line reports: it is printed whenever one of these changes.
#[derive(Clone, Copy, PartialEq)]
struct Summary {
    mode: Mode,
    armed: bool,
    estop: bool,
    throttle_on: bool,
    brake: f32,
    faults: u16,
}

impl From<&Outputs> for Summary {
    fn from(out: &Outputs) -> Self {
        Summary {
            mode: out.mode,
            armed: out.armed,
            estop: out.estop_active,
            throttle_on: out.throttle > 0.0,
            brake: out.brake,
            faults: out.faults,
        }
    }
}

/// Milliseconds since boot. It wraps after 49 days, as `spine-core` expects.
fn now_ms() -> u32 {
    Instant::now().duration_since_epoch().as_millis() as u32
}

// --- The program -------------------------------------------------------------------------

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    enable_fpu();
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let p = esp_hal::init(esp_hal::Config::default());

    // Embassy's clock, on TIMG0. Not in QEMU: its first interrupt would never clear.
    #[cfg(not(feature = "qemu"))]
    {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0, p.FROM_CPU_INTR0);
    }

    // The watchdog first: from here on, a hang anywhere resets the chip.
    let mut watchdog = TimerGroup::new(p.TIMG1).wdt;
    watchdog.set_timeout(
        MwdtStage::Stage0,
        esp_hal::time::Duration::from_millis(WATCHDOG_MS),
    );
    watchdog.enable();
    info!(
        "spine-esp32-umod {}, link: {}{}{}; the last reset: {:?}",
        env!("CARGO_PKG_VERSION"),
        if cfg!(feature = "can") {
            "CAN"
        } else if cfg!(feature = "link-v1") {
            "Motor-Pi UART, version 1 frames (no heartbeat)"
        } else {
            "Motor-Pi UART, version 2 frames (the Orin's heartbeat)"
        },
        if cfg!(feature = "uplink") {
            ", with the uplink on TX1 and the three-position mode switch"
        } else {
            ""
        },
        if cfg!(feature = "qemu") {
            ", in QEMU with stand-in devices"
        } else {
            ""
        },
        esp_hal::system::reset_reason()
    );

    // The pin map (src/pins.rs, docs/rust-firmware.md).
    let pins = pins::take_pins!(p);

    #[cfg(not(feature = "qemu"))]
    let i2c_bus = {
        let i2c_config = i2c::Config::default().with_frequency(Rate::from_khz(I2C_KHZ));
        match I2c::new(p.I2C0, i2c_config) {
            Ok(i2c) => RefCell::new(i2c.with_sda(pins.sda).with_scl(pins.scl)),
            Err(e) => panic!("I2C at {} kHz: {:?}", I2C_KHZ, e), // a constant: never on the car
        }
    };
    // QEMU has none of the car's devices on its I2C bus: stand-ins answer instead.
    #[cfg(feature = "qemu")]
    let i2c_bus = {
        let _ = (p.I2C0, pins.sda, pins.scl);
        RefCell::new(StandInBus::new(&Calibration::TODAY))
    };

    let ibus_config = uart::Config::default().with_baudrate(ibus::BAUD);
    let ibus_rx = UartRx::new(p.UART2, ibus_config)
        .map(|rx| rx.with_rx(pins.ibus_rx))
        .unwrap_or_else(|e| panic!("iBUS UART: {:?}", e));

    let output = |pin: AnyPin<'static>, level| Output::new(pin, level, OutputConfig::default());
    let leds = Leds {
        status: output(pins.status_led.into(), Level::High),
        red: output(pins.red_led.into(), Level::Low),
        green: output(pins.green_led.into(), Level::Low),
        blue: output(pins.blue_led.into(), Level::Low),
    };

    let board = Esp32Umod {
        #[cfg(feature = "can")]
        can: {
            use esp_hal::twai::{BaudRate, TwaiConfiguration, TwaiMode};
            let config = TwaiConfiguration::new(
                p.TWAI0,
                pins.can_rx,
                pins.can_tx,
                BaudRate::B500K,
                TwaiMode::Normal,
            );
            can::CanPort::new(config.start())
        },
        #[cfg(feature = "estop-input")]
        estop: esp_hal::gpio::Input::new(
            pins.estop,
            esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
        ),
        leds,
    };

    // The spine: the devices first, safe at once (throttle 0, brake on).
    // With the uplink, the RC's mode switch has a middle position: RC via ROS.
    #[cfg(feature = "uplink")]
    let cal = Calibration {
        rc: ibus::RcMap::THREE_POSITION,
        ..Calibration::TODAY
    };
    #[cfg(not(feature = "uplink"))]
    let cal = Calibration::TODAY;
    let devices = Devices::new(
        RefCellDevice::new(&i2c_bus),
        RefCellDevice::new(&i2c_bus),
        RefCellDevice::new(&i2c_bus),
        RefCellDevice::new(&i2c_bus),
        &cal,
    );
    let config = SpineConfig {
        speed_feedback: SpeedFeedback::None {
            full_throttle_speed: FULL_THROTTLE_SPEED,
        },
        idle_brake: IDLE_BRAKE,
        ..SpineConfig::default()
    };

    #[cfg(feature = "legacy-uart")]
    let mut spine = {
        let motor_pi_config = uart::Config::default().with_baudrate(legacy_link::BAUD);
        let version = if cfg!(feature = "link-v1") {
            spine_board::FrameVersion::V1
        } else {
            spine_board::FrameVersion::V2
        };
        #[cfg(not(feature = "uplink"))]
        let link = {
            let motor_pi_rx = UartRx::new(p.UART1, motor_pi_config)
                .map(|rx| rx.with_rx(pins.motor_pi_rx))
                .unwrap_or_else(|e| panic!("Motor-Pi UART: {:?}", e));
            spine_board::LegacyLink::new(motor_pi_rx, cal, version)
        };
        // The same UART, both ways: RX on GPIO16, the uplink's TX on GPIO17.
        #[cfg(feature = "uplink")]
        let link = {
            let (motor_pi_rx, motor_pi_tx) = esp_hal::uart::Uart::new(p.UART1, motor_pi_config)
                .map(|u| {
                    u.with_rx(pins.motor_pi_rx)
                        .with_tx(pins.motor_pi_tx)
                        .split()
                })
                .unwrap_or_else(|e| panic!("Motor-Pi UART: {:?}", e));
            spine_board::LegacyLink::with_uplink(motor_pi_rx, motor_pi_tx, cal, version)
        };
        SpineBoard::with_link(board, devices, ibus_rx, link, cal, config)
    };
    #[cfg(feature = "can")]
    let mut spine = {
        let _ = pins.motor_pi_rx; // the Motor-Pi's UART stays unused
        SpineBoard::new(board, devices, ibus_rx, cal, config)
    };

    let faults = spine.init();
    if faults != 0 {
        warn!(
            "power-up: device faults {:#x}; the car stays stopped until a stop, then a command",
            faults
        );
    }

    let mut pace = Pace::new();
    let mut ticks: u32 = 0;
    let mut last: Option<Summary> = None;
    #[cfg(feature = "qemu")]
    let mut last_devices = DeviceTargets::default();
    loop {
        let out = spine.tick();
        let now = now_ms();
        let board = spine.board();
        board.leds.show(&out, ticks);
        #[cfg(feature = "can")]
        board.can.service(now);
        // The last Orin counter taken (-1: none, or not this link): it ties each line to a
        // frame, whatever the clocks do (tools/esp32_qemu.py).
        #[cfg(feature = "legacy-uart")]
        let counter = spine.link().stats().counter.map_or(-1, i32::from);
        #[cfg(not(feature = "legacy-uart"))]
        let counter = -1;
        let summary = Summary::from(&out);
        if last != Some(summary) {
            log_line("EVENT", now, &out, counter);
            last = Some(summary);
        }
        if ticks.is_multiple_of(STATUS_EVERY_TICKS) {
            log_line("STATUS", now, &out, counter);
            fsm_line(now, &spine.state());
            #[cfg(feature = "legacy-uart")]
            link_line(now, &spine.link().stats());
        }
        #[cfg(feature = "qemu")]
        {
            let devices = i2c_bus.borrow().devices();
            if devices != last_devices {
                devices_line(now, &devices);
                last_devices = devices;
            }
        }
        watchdog.feed();
        ticks = ticks.wrapping_add(1);
        pace.next().await;
    }
}

/// One line on the USB cable: what the spine decided, and its faults. `STATUS` every
/// 200 ms, `EVENT` when the mode, arming, e-stop, throttle on or off, brake or faults
/// change. `t` is milliseconds since boot; `counter` the last Orin counter taken (-1: none).
fn log_line(kind: &str, now: u32, out: &Outputs, counter: i32) {
    info!(
        "{} t={} mode={:?} armed={} estop={} throttle={:.2} brake={:.2} reverse={} steer={:.3} faults={:#06x} counter={}",
        kind,
        now,
        out.mode,
        out.armed,
        out.estop_active,
        out.throttle,
        out.brake,
        out.reverse,
        out.steering_target,
        out.faults,
        counter
    );
}

/// The five state regions (`spine_core::fsm`), with each status line: which one the
/// spine is in, in each region.
fn fsm_line(now: u32, s: &spine_core::State) {
    info!(
        "FSM t={} lifecycle={:?} safety={:?} authority={:?} motion={:?} confidence={:?}",
        now, s.lifecycle, s.safety, s.authority, s.motion, s.confidence
    );
}

/// The Motor-Pi link's counts since boot, with each status line: frames taken, turned
/// down (`bad`), bytes outside a frame (`skipped`), frames with the same counter as the
/// one before (`repeats`), the last counter (-1: none yet), and the uplink frames sent
/// and failed (0 without `uplink`).
#[cfg(feature = "legacy-uart")]
fn link_line(now: u32, s: &spine_board::LinkStats) {
    info!(
        "LINK t={} frames={} bad={} skipped={} repeats={} counter={} uplinks={} uplink_errors={}",
        now,
        s.frames,
        s.bad,
        s.skipped,
        s.repeats,
        s.counter.map_or(-1, i32::from),
        s.uplinks,
        s.uplink_errors
    );
}

/// In QEMU: what the stand-in devices were last told, whenever it changes. These are the
/// bytes the car's Jrks, DAC and expander would have got.
#[cfg(feature = "qemu")]
fn devices_line(now: u32, d: &DeviceTargets) {
    let show = |v: Option<u16>| v.map_or(-1, i32::from);
    info!(
        "DEVICES t={} steering={} brake={} dac={} expander={:#06x}",
        now,
        show(d.steering),
        show(d.brake),
        show(d.dac),
        d.expander.unwrap_or(0)
    );
}

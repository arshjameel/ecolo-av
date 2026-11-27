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
//! With `uplink` (on by default; the wire from TX1, GPIO17, to the Motor-Pi's RX is assumed:
//! TODO 50, 48), the link also sends a status frame back every 20 ms
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
//! | the log | UART0, the USB cable: a status line every 200 ms, as the C++ printed, an event line at each change, and (`legacy-uart`) a link line with the frame counts; a tick-timing line and a health line (the Pi link's verdict, the devices, the faults) once a second; a config line and a devices line at boot and every 10 s |
//! | the first look | after `init`, before the first tick: which of the four I2C devices answer, and each Jrk's feedback mode (`spine_board::DeviceCheck`, read only), for `pixi run esp first-check` |
//!
//! Two features report more, and neither changes what the spine does:
//!
//! | Feature | Reports |
//! |---|---|
//! | `json-log` | the JSON status lines of `firmware/status` on the USB cable instead of the text lines, a chunk a tick (`report`) |
//! | `wifi-status` | the same lines over WiFi, read only: a page, `/status.json`, `/scan`, a WebSocket (`wifi`). Off by default; its credentials come from an untracked file |
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
#[cfg(all(feature = "cpp-throttle-ramp", not(feature = "legacy-uart")))]
compile_error!(
    "`cpp-throttle-ramp` is the C++'s ramp on the Motor-Pi UART link: build it with `legacy-uart`"
);
#[cfg(all(feature = "wifi-status", feature = "qemu"))]
compile_error!("`wifi-status` needs the radio and Embassy's timer: QEMU has neither");
#[cfg(all(feature = "wifi-status", not(wifi_secrets)))]
compile_error!(
    "`wifi-status` needs the WiFi's name and password: copy src/wifi_secrets.rs.example to \
     src/wifi_secrets.rs and fill it in (git ignores that file; never commit, log or copy \
     the password). README.md, \"WiFi status\", says more"
);

#[cfg(feature = "can")]
mod can;
mod pins;
#[cfg(any(feature = "json-log", feature = "wifi-status"))]
mod report;
#[cfg(feature = "wifi-update")]
mod update;
#[cfg(feature = "wifi-status")]
mod wifi;

use core::cell::RefCell;
use core::fmt::{self, Write as _};
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
use spine_board::ErrorFlags;
use spine_board::{
    feedback_name, Board, Calibration, CanFrame, DeviceCheck, Devices, Found, LinkReport,
    SpineBoard, TickTiming, WheelCount,
};
#[cfg(feature = "qemu")]
use spine_board::{DeviceTargets, StandInBus};
use spine_core::protocol::Mode;
use spine_core::{EstopSteering, Outputs, SpeedFeedback, SpineConfig, RC_BRAKE_MS};
use static_cell::StaticCell;
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

/// The least DAC code (of 1023) that turns the wheels forward: the open loop's throttle
/// starts here and rises to the cap (`Calibration::TODAY`'s 700) at `FULL_THROTTLE_SPEED`.
/// Arsh, 27 Nov, on stands with the C++, the stick pushed slowly from the middle: the wheels
/// started at 470 to 480, and stop at about the same. (265, from 26 Nov, was read with the
/// wheels already turning: the Rust's 410 from it gave no torque, #51.)
const THROTTLE_START_CODE: u16 = 475;

/// The same in reverse: the wheels started at 521 to 552 (Arsh, 27 Nov).
const THROTTLE_START_REVERSE_CODE: u16 = 525;

/// The steering's rate limit (rad/s), for the RC and the Motor-Pi's commands alike: the
/// steering motor's own speed, so the RC steers as the C++ did, while a target that jumps
/// still moves no faster. Arsh, 27 Nov, the C++ on stands: centre to full lock (1550
/// counts) in about 0.22 s, filmed at 30 frames a second, so 1.9 to 2.8 rad/s at 3100
/// counts a radian (`Calibration::TODAY`, itself TODO(Q1)). spine-core's default, 0.6,
/// copies `ego.yaml`'s `max_steering_rate`, which still limits the computer upstream.
const STEER_RATE: f32 = 2.2;

/// The brake while nobody drives, from boot on. The C++ kept it full on (3500), and this
/// board has no e-stop input yet to hold it, so it does the same until the team settles
/// spine-core's `IDLE_BRAKE` (proposed 0.0: released). TODO(Ben): decide.
const IDLE_BRAKE: f32 = 1.0;

/// The status line on the USB cable, as often as the C++'s `publishStatus()`.
const STATUS_EVERY_TICKS: u32 = 20;

/// The tick's timing on the USB cable: once a second, between two status lines, so it
/// doesn't lengthen their burst.
const TIMING_EVERY_TICKS: u32 = 100;
const TIMING_AT_TICK: u32 = 50;

/// The health line on the USB cable: once a second, on its own tick.
const HEALTH_EVERY_TICKS: u32 = 100;
const HEALTH_AT_TICK: u32 = 75;

/// The config and devices lines again every 10 s, so a reader that starts late (the first
/// check) sees them: each on a tick of its own, away from the status lines.
const CONFIG_EVERY_TICKS: u32 = 1000;
const CONFIG_AT_TICK: u32 = 250;
const DEVICES_AT_TICK: u32 = 750;

/// With `json-log`: the most bytes of status lines put on the USB cable each tick. At
/// 115200 baud the UART sends about 115 bytes in 10 ms, so its 128-byte FIFO is empty
/// again by the next tick and the print never waits (it waits with interrupts off).
#[cfg(feature = "json-log")]
const LOG_BYTES_PER_TICK: usize = 96;

/// With `wifi-status`: I2C addresses probed each tick during a `/scan`, after the tick's
/// own transfers: about 0.1 ms each at 100 kHz.
#[cfg(feature = "wifi-status")]
const SCAN_PER_TICK: usize = 4;

/// The RC channels a status line carries: the six the car's receiver uses.
#[cfg(any(feature = "json-log", feature = "wifi-status"))]
const RC_CHANNELS_SHOWN: usize = 6;

/// The text lines on the USB cable: off with `json-log`, whose lines take their place.
const TEXT_LOG: bool = !cfg!(feature = "json-log");

/// The Motor-Pi's UART link, and its sync-byte frames (the steering calibration rides on
/// version 3): which settings the config line has.
const UART_LINK: bool = cfg!(feature = "legacy-uart");
const STEER_CAL: bool = UART_LINK && !cfg!(feature = "link-v1");

/// The status lines' period, in ticks, as the config line says it: the JSON lines' with
/// `json-log` or `wifi-status`, otherwise the text `STATUS` line's.
#[cfg(any(feature = "json-log", feature = "wifi-status"))]
const STATUS_LINE_EVERY_TICKS: u32 = report::STATUS_EVERY_TICKS;
#[cfg(not(any(feature = "json-log", feature = "wifi-status")))]
const STATUS_LINE_EVERY_TICKS: u32 = STATUS_EVERY_TICKS;

/// Text lines waiting for the cable, while a long one (the config line, about 1 KB: 90 ms
/// of the cable) goes out a chunk a tick. Each slot holds the longest config line.
const TEXT_SLOTS: usize = 6;
const TEXT_SLOT_BYTES: usize = status_lines::MAX_LINE + 1;

/// The most bytes of queued text put on the cable each tick: as `LOG_BYTES_PER_TICK`, the
/// UART's 128-byte FIFO empties within the tick, so the print never waits.
const TEXT_BYTES_PER_TICK: usize = 96;

/// The C++'s `STATUS:` line (`publishStatus()`), every 200 ms as it was, on a tick of its
/// own: half way between two `STATUS` lines.
const CPP_STATUS_AT_TICK: u32 = 10;

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

/// The text lines on the USB cable. A line is printed at once, as before, unless lines are
/// waiting behind a long one: then it waits its turn, so two lines never mix. A long line
/// (the config line) always waits, and goes out [`TEXT_BYTES_PER_TICK`] a tick, so the
/// print never holds the tick: esp-println waits, with interrupts off, while the UART's
/// FIFO is full, and 1 KB is 90 ms at 115200 baud, near the 100 ms watchdog.
struct TextLog {
    queue: status_lines::Trickle<TEXT_SLOTS, TEXT_SLOT_BYTES>,
}

impl TextLog {
    /// One line, as `info!` writes it.
    fn line(&mut self, args: fmt::Arguments<'_>) {
        #[cfg(feature = "wifi-status")]
        wifi::publish_text(&args);
        if self.queue.is_empty() {
            info!("{args}");
        } else {
            // A full queue drops the line whole: it is only the log.
            let _ = self.queue.push_text(&Logged(args));
        }
    }

    /// A long line: queued, a chunk a tick.
    fn long(&mut self, text: &impl fmt::Display) {
        #[cfg(feature = "wifi-status")]
        wifi::publish_text(text);
        let _ = self.queue.push_text(&Logged(text));
    }

    /// This tick's chunk of the queued lines, if any.
    fn next_chunk(&mut self) -> Option<&[u8]> {
        self.queue.next_chunk(TEXT_BYTES_PER_TICK)
    }
}

/// A queued line, with the prefix esp-println's logger gives an `info!` line.
struct Logged<T>(T);

impl<T: fmt::Display> fmt::Display for Logged<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "INFO - {}", self.0)
    }
}

/// Milliseconds since boot. It wraps after 49 days, as `spine-core` expects.
fn now_ms() -> u32 {
    Instant::now().duration_since_epoch().as_millis() as u32
}

// --- The program -------------------------------------------------------------------------

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
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
    let last_reset = reset_name(esp_hal::system::reset_reason());
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
    #[cfg(feature = "json-log")]
    let mut trickle: status_lines::Trickle = status_lines::Trickle::new();

    // The pin map (src/pins.rs, docs/rust-firmware.md).
    let pins = pins::take_pins!(p);
    boot_pin_lines(&pins);

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
            use esp_hal::twai::{TwaiConfiguration, TwaiMode};
            let config = TwaiConfiguration::new(
                p.TWAI0,
                pins.can_rx,
                pins.can_tx,
                can::BAUD,
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
    // `cpp-rc-steering`: the C++'s RC steering, harder to the right.
    #[cfg(feature = "cpp-rc-steering")]
    let cal = Calibration {
        rc: ibus::RcMap {
            steer_right_gain: legacy_link::STEER_RIGHT_PER_LEFT,
            ..cal.rc
        },
        ..cal
    };
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
            start_throttle: cal.throttle.fraction(THROTTLE_START_CODE),
            start_throttle_reverse: cal.throttle.fraction(THROTTLE_START_REVERSE_CODE),
        },
        idle_brake: IDLE_BRAKE,
        max_steer_rate: STEER_RATE,
        // The RC receiver is on GPIO 32: in normal mode too, losing the RC is a hard stop
        // (Arsh, 21 Nov).
        rc_fitted: true,
        // `cpp-stop-centre`: on a stop, the wheels back to straight, as `jrkSafeStop()`.
        estop_steering: if cfg!(feature = "cpp-stop-centre") {
            EstopSteering::Centre
        } else {
            spine_core::ESTOP_STEERING
        },
        ..SpineConfig::default()
    };
    let numbers = board_config(&cal, &config, last_reset);

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
        // `cpp-throttle-ramp`: the C++'s DAC ramp, a step a frame.
        #[cfg(all(not(feature = "uplink"), feature = "cpp-throttle-ramp"))]
        let link = link.with_cpp_ramp();
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
        #[cfg(all(feature = "uplink", feature = "cpp-throttle-ramp"))]
        let link = link.with_cpp_ramp();
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
    // The first look, safe already (throttle 0, brake on, latched): which devices answer.
    // Read only: probes with no data, and the Jrks' feedback mode, never their errors.
    let found = DeviceCheck::run(&mut *i2c_bus.borrow_mut(), &cal.addresses);
    watchdog.feed();
    // In place: it is 8 KB, too big to build on the stack and move.
    static TEXT: static_cell::ConstStaticCell<TextLog> =
        static_cell::ConstStaticCell::new(TextLog {
            queue: status_lines::Trickle::new(),
        });
    let text = TEXT.take();
    // The C++'s boot lines (spine_board::cpp_log), at once: about 30 ms of the cable.
    for line in spine_board::cpp_log::boot_devices(&found, &cal) {
        info!("{line}");
    }
    info!("{}", spine_board::cpp_log::RcReady(&cal.rc));
    watchdog.feed();
    config_line(text, now_ms(), &numbers);
    devices_line(text, now_ms(), &found);
    #[cfg(any(feature = "json-log", feature = "wifi-status"))]
    let mut reporter = report::Reporter::new(esp_hal::rng::Rng::new().random(), numbers, &found);
    #[cfg(feature = "legacy-uart")]
    let mut link_health = spine_board::LinkHealth::new(config.cmd_timeout_ms);

    // The WiFi tasks, on this executor: each await gives the tick its turn (wifi.rs).
    #[cfg(feature = "wifi-status")]
    wifi::start(
        spawner,
        p.WIFI,
        cal.addresses,
        #[cfg(feature = "wifi-update")]
        p.FLASH,
    );
    #[cfg(not(feature = "wifi-status"))]
    let _ = spawner;
    #[cfg(feature = "wifi-status")]
    let mut scan = spine_board::BusScan::new();
    // From here on the JSON lines own the cable: only warnings may come between them.
    if cfg!(feature = "json-log") {
        log::set_max_level(log::LevelFilter::Warn);
    }

    let mut pace = Pace::new();
    let mut ticks: u32 = 0;
    let mut timing = TickTiming::new(TICK.as_micros() as u32);
    let mut last: Option<Summary> = None;
    // For the C++-style lines: the faults and the Jrks' errors last logged.
    let mut last_faults: u16 = 0;
    let mut last_jrk_errors = (Some(ErrorFlags(0)), Some(ErrorFlags(0)));
    #[cfg(feature = "qemu")]
    let mut last_devices = DeviceTargets::default();
    loop {
        timing.tick(Instant::now().duration_since_epoch().as_micros());
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
        // The Pi link's verdict (none on CAN: the Orin's spine_bridge judges that one).
        #[cfg(feature = "legacy-uart")]
        let health = Some(link_health.update(now, &spine.link().stats()));
        #[cfg(not(feature = "legacy-uart"))]
        let health: Option<LinkReport> = None;
        let summary = Summary::from(&out);
        if last != Some(summary) {
            if TEXT_LOG {
                log_line(text, "EVENT", now, &out, counter);
            }
            last = Some(summary);
        }
        if TEXT_LOG && ticks.is_multiple_of(STATUS_EVERY_TICKS) {
            log_line(text, "STATUS", now, &out, counter);
            fsm_line(text, now, &spine.state());
            #[cfg(feature = "legacy-uart")]
            link_line(text, now, &spine.link().stats());
        }
        if TEXT_LOG {
            cpp_lines(
                text,
                now,
                ticks,
                &out,
                &found,
                config.max_steering_angle,
                spine.actuators().jrk_errors(),
                &mut last_faults,
                &mut last_jrk_errors,
            );
        }
        if TEXT_LOG && ticks % TIMING_EVERY_TICKS == TIMING_AT_TICK {
            timing_line(text, now, &timing);
        }
        if TEXT_LOG && ticks % HEALTH_EVERY_TICKS == HEALTH_AT_TICK {
            health_line(text, now, health.as_ref(), &found, &out);
        }
        if TEXT_LOG && ticks % CONFIG_EVERY_TICKS == CONFIG_AT_TICK {
            config_line(text, now, &numbers);
        }
        if TEXT_LOG && ticks % CONFIG_EVERY_TICKS == DEVICES_AT_TICK {
            devices_line(text, now, &found);
        }
        #[cfg(any(feature = "json-log", feature = "wifi-status"))]
        {
            #[cfg(feature = "legacy-uart")]
            let link = {
                let l = spine.link();
                Some((l.stats(), l.last_frame()))
            };
            #[cfg(not(feature = "legacy-uart"))]
            let link = None;
            let actuators = spine.actuators();
            let (steer_counts, brake_counts) = (
                actuators.steering_counts().unwrap_or(0),
                actuators.brake_counts().unwrap_or(0),
            );
            let (dac, reverse) = (actuators.throttle_code(), actuators.reverse());
            let channels = spine
                .rc_channels()
                .map_or(&[][..], |c| &c.0[..RC_CHANNELS_SHOWN]);
            let snapshot = report::Snapshot {
                now_ms: now,
                out: &out,
                report: spine.last_report(),
                channels,
                steer_counts,
                brake_counts,
                dac,
                reverse,
                link,
                health,
                timing: &timing,
            };
            reporter.tick(ticks, &snapshot, |line| {
                // A full queue drops the line whole: the reader sees the gap in `seq`.
                #[cfg(feature = "json-log")]
                let _ = trickle.push(line);
                #[cfg(feature = "wifi-status")]
                wifi::publish(line);
            });
        }
        // One writer a tick: the queued text first (the boot's config line, with
        // `json-log` too), then the JSON lines.
        if let Some(chunk) = text.next_chunk() {
            esp_println::Printer::write_bytes(chunk);
        } else {
            #[cfg(feature = "json-log")]
            if let Some(chunk) = trickle.next_chunk(LOG_BYTES_PER_TICK) {
                esp_println::Printer::write_bytes(chunk);
            }
        }
        // A bus scan for /scan: after the tick's own transfers, and only while stopped.
        #[cfg(feature = "wifi-status")]
        {
            // wifi-update: the car's state for the update's checks; its restart, here in
            // the tick, only after a checked write, and only while parked.
            #[cfg(feature = "wifi-update")]
            {
                update::set_car(app_update::CarState {
                    stopped: out.mode == Mode::Manual,
                    armed: out.armed,
                    throttle: out.throttle,
                    brake: out.brake,
                });
                if update::restart_due(now) {
                    warn!("restarting into the new firmware");
                    esp_hal::system::software_reset();
                }
            }
            if wifi::take_scan_request() {
                scan.start();
            }
            let busy = out.armed || out.mode != Mode::Manual;
            let was_done = scan.state() == spine_board::ScanState::Done;
            scan.step(&mut *i2c_bus.borrow_mut(), busy, SCAN_PER_TICK);
            wifi::set_scan(&scan);
            // Done: the C++'s scan lines (`handleScan()`), on the cable and on /log.
            if TEXT_LOG && !was_done && scan.state() == spine_board::ScanState::Done {
                scan_lines(text, &scan, &cal.addresses);
            }
        }
        #[cfg(feature = "qemu")]
        {
            let devices = i2c_bus.borrow().devices();
            if devices != last_devices {
                stand_in_line(text, now, &devices);
                last_devices = devices;
            }
        }
        watchdog.feed();
        ticks = ticks.wrapping_add(1);
        pace.next().await;
    }
}

/// The C++'s first boot lines: which pins and speeds the links use. Their numbers come
/// from the pins themselves (src/pins.rs).
fn boot_pin_lines(pins: &pins::Pins) {
    use esp_hal::gpio::Pin as _;
    info!(
        "=== ESP32: AAV Controller (Rust, spine-esp32-umod {}) ===",
        env!("CARGO_PKG_VERSION")
    );
    #[cfg(all(feature = "legacy-uart", feature = "uplink"))]
    info!(
        "Serial1 (ROS UART) @ GPIO{}/{}, {} 8N1",
        pins.motor_pi_rx.number(),
        pins.motor_pi_tx.number(),
        legacy_link::BAUD
    );
    #[cfg(all(feature = "legacy-uart", not(feature = "uplink")))]
    info!(
        "Serial1 (ROS UART) @ GPIO{} (RX only), {} 8N1",
        pins.motor_pi_rx.number(),
        legacy_link::BAUD
    );
    #[cfg(feature = "can")]
    info!(
        "TWAI (CAN) @ GPIO{} TX, GPIO{} RX, {} kbit/s",
        pins.can_tx.number(),
        pins.can_rx.number(),
        can::KBIT
    );
    info!(
        "iBUS Serial2 @ GPIO{}, {}",
        pins.ibus_rx.number(),
        ibus::BAUD
    );
    #[cfg(not(feature = "qemu"))]
    info!(
        "I2C -> SDA:{} SCL:{}, {} kHz",
        pins.sda.number(),
        pins.scl.number(),
        I2C_KHZ
    );
    #[cfg(feature = "qemu")]
    info!(
        "I2C -> SDA:{} SCL:{}, stand-in devices (QEMU)",
        pins.sda.number(),
        pins.scl.number()
    );
}

/// The C++'s running lines (spine_board::cpp_log): its `STATUS:` line every 200 ms; a
/// `FAULT` line when the faults change; a `JRK ... ERR` line when a Jrk's errors change.
#[allow(clippy::too_many_arguments)]
fn cpp_lines(
    text: &mut TextLog,
    now: u32,
    ticks: u32,
    out: &Outputs,
    found: &DeviceCheck,
    max_steering_angle: f32,
    jrk_errors: (Option<ErrorFlags>, Option<ErrorFlags>),
    last_faults: &mut u16,
    last_jrk_errors: &mut (Option<ErrorFlags>, Option<ErrorFlags>),
) {
    use spine_board::cpp_log::{CppStatus, FaultChange, Jrk, JrkErrorLine};
    if ticks % STATUS_EVERY_TICKS == CPP_STATUS_AT_TICK {
        let line = CppStatus::new(out, max_steering_angle, found);
        text.line(format_args!("{line}"));
    }
    if out.faults != *last_faults {
        let line = FaultChange {
            now_ms: now,
            was: *last_faults,
            now: out.faults,
        };
        text.line(format_args!("{line}"));
        *last_faults = out.faults;
    }
    for (jrk, flags, last) in [
        (Jrk::Steering, jrk_errors.0, &mut last_jrk_errors.0),
        (Jrk::Brake, jrk_errors.1, &mut last_jrk_errors.1),
    ] {
        if flags != *last {
            let line = JrkErrorLine {
                jrk,
                now_ms: now,
                flags,
            };
            text.line(format_args!("{line}"));
            *last = flags;
        }
    }
}

/// The C++'s `/scan` lines, once a scan is done: `=== I2C Scan ===`, a `Found:` line for
/// each address, with its name when it is one of the car's four, `=== Scan done ===`.
#[cfg(feature = "wifi-status")]
fn scan_lines(
    text: &mut TextLog,
    scan: &spine_board::BusScan,
    a: &spine_board::calibration::Addresses,
) {
    use spine_board::cpp_log::{scan_label, ScanFound};
    text.line(format_args!("=== I2C Scan ==="));
    let mut any = false;
    for address in scan.found() {
        let line = ScanFound {
            address,
            label: scan_label(address, a),
        };
        text.line(format_args!("  {line}"));
        any = true;
    }
    if !any {
        text.line(format_args!("  No I2C devices found!"));
    }
    text.line(format_args!("=== Scan done ==="));
}

/// One line on the USB cable: what the spine decided, and its faults. `STATUS` every
/// 200 ms, `EVENT` when the mode, arming, e-stop, throttle on or off, brake or faults
/// change. `t` is milliseconds since boot; `counter` the last Orin counter taken (-1: none).
fn log_line(text: &mut TextLog, kind: &str, now: u32, out: &Outputs, counter: i32) {
    text.line(format_args!(
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
    ));
}

/// The five state regions (`spine_core::fsm`), with each status line: which one the
/// spine is in, in each region.
fn fsm_line(text: &mut TextLog, now: u32, s: &spine_core::State) {
    text.line(format_args!(
        "FSM t={} lifecycle={:?} safety={:?} authority={:?} motion={:?} confidence={:?}",
        now, s.lifecycle, s.safety, s.authority, s.motion, s.confidence
    ));
}

/// The Motor-Pi link's counts since boot, with each status line: frames taken, turned
/// down (`bad`), bytes outside a frame (`skipped`), frames with the same counter as the
/// one before (`repeats`), the last counter (-1: none yet), and the uplink frames sent
/// and failed (0 without `uplink`).
#[cfg(feature = "legacy-uart")]
fn link_line(text: &mut TextLog, now: u32, s: &spine_board::LinkStats) {
    text.line(format_args!(
        "LINK t={} frames={} bad={} skipped={} repeats={} counter={} uplinks={} uplink_errors={}",
        now,
        s.frames,
        s.bad,
        s.skipped,
        s.repeats,
        s.counter.map_or(-1, i32::from),
        s.uplinks,
        s.uplink_errors
    ));
}

/// The tick's timing since boot: the longest gap between ticks, and the ticks that came
/// more than 10 ms late. The baseline for the timing test (README.md).
fn timing_line(text: &mut TextLog, now: u32, t: &TickTiming) {
    text.line(format_args!(
        "TICK t={} worst_gap_us={} overruns={}",
        now,
        t.worst_gap_us(),
        t.overruns()
    ));
}

/// The build and the numbers it runs with, for the config lines (text and JSON).
fn board_config(
    cal: &Calibration,
    config: &SpineConfig,
    last_reset: &'static str,
) -> status_lines::Config<'static> {
    let (a, s, b) = (&cal.addresses, &cal.steering, &cal.brake);
    status_lines::Config {
        build: env!("AAV_BUILD"),
        features: env!("AAV_FEATURES"),
        version: env!("CARGO_PKG_VERSION"),
        link: if cfg!(feature = "can") { "can" } else { "uart" },
        frame_version: if cfg!(feature = "can") {
            None
        } else if cfg!(feature = "link-v1") {
            Some(1)
        } else {
            Some(2)
        },
        tick_ms: TICK.as_millis() as u32,
        watchdog_ms: WATCHDOG_MS as u32,
        cmd_timeout_ms: config.cmd_timeout_ms,
        idle_brake: config.idle_brake,
        full_throttle_speed: FULL_THROTTLE_SPEED,
        #[cfg(not(feature = "qemu"))]
        i2c_khz: Some(I2C_KHZ),
        // Stand-ins: no bus speed.
        #[cfg(feature = "qemu")]
        i2c_khz: None,
        addresses: status_lines::Addresses {
            steering: a.steering,
            brake: a.brake,
            dac: a.dac,
            expander: a.expander,
        },
        steering: status_lines::SteeringLimits {
            min: s.min,
            centre: s.centre,
            max: s.max,
        },
        brake: status_lines::BrakeLimits {
            released: b.released,
            full: b.full,
        },
        throttle_full_code: cal.throttle.full_code,
        last_reset,
        steering_more: status_lines::SteeringSettings {
            counts_per_rad: s.counts_per_rad,
            max_angle_rad: config.max_steering_angle,
            rate_rad_s: config.max_steer_rate,
            // The steering calibration rides on the sync-byte frames (version 3) only.
            cal_rate: STEER_CAL.then_some(legacy_link::CAL_RATE),
            cal_timeout_ms: STEER_CAL.then_some(legacy_link::CAL_TIMEOUT_MS),
            on_stop: match config.estop_steering {
                EstopSteering::Hold => "hold",
                EstopSteering::Centre => "centre",
            },
        },
        brake_more: status_lines::BrakeSettings {
            stop: config.stop_brake,
            overspeed: config.overspeed_brake,
            rc_band: cal.rc.brake_band,
            rc_brake_ms: RC_BRAKE_MS,
            rc_lost_below: cal.rc.lost_below,
            rc_fitted: config.rc_fitted,
        },
        speed: status_lines::SpeedSettings {
            max_forward: config.max_speed,
            max_reverse: config.max_reverse_speed,
            cap_code: config
                .open_loop_throttle_cap(false)
                .map(|cap| cal.throttle.code(cap)),
            reverse_cap_code: config
                .open_loop_throttle_cap(true)
                .map(|cap| cal.throttle.code(cap)),
            frame_dac_cap: UART_LINK.then_some(legacy_link::DAC_CAP),
            rise_per_s: cal.throttle.rise_per_s,
            max_accel: config.max_accel,
            max_decel: config.max_decel,
            overspeed_margin: config.overspeed_margin,
        },
        timeouts: status_lines::Timeouts {
            rc_lost_ms: cal.rc_timeout_ms,
            speed_lost_ms: config.speed_timeout_ms,
            overspeed_ms: spine_core::OVERSPEED_WINDOW_MS,
            estop_release_ms: cal.estop_release_ms,
            bad_frames_ms: UART_LINK.then_some(spine_board::BAD_WINDOW_MS),
        },
        rates: status_lines::Rates {
            uart_baud: UART_LINK.then_some(legacy_link::BAUD),
            frame_hz: UART_LINK.then_some(legacy_link::FRAME_HZ),
            uplink_ms: cfg!(feature = "uplink").then_some(spine_board::UPLINK_PERIOD_MS),
            #[cfg(feature = "can")]
            can_kbit: Some(can::KBIT),
            #[cfg(not(feature = "can"))]
            can_kbit: None,
            ibus_baud: ibus::BAUD,
            // SpineBoard::tick gives every device its target, every tick (actuators.rs).
            jrk_write_ms: TICK.as_millis() as u32,
            dac_write_ms: TICK.as_millis() as u32,
            status_ms: STATUS_LINE_EVERY_TICKS * TICK.as_millis() as u32,
        },
    }
}

/// One device as the lines say it: a Jrk whose feedback mode couldn't be read is
/// `unknown`.
fn device_seen(f: &Found) -> status_lines::DeviceSeen<'static> {
    status_lines::DeviceSeen {
        name: f.device.name(),
        address: f.address,
        answered: f.answered,
        feedback: match (f.device.is_jrk() && f.answered, f.feedback) {
            (false, _) => None,
            (true, Some(mode)) => Some(feedback_name(mode)),
            (true, None) => Some("unknown"),
        },
    }
}

/// The build and its numbers, at boot and every 10 s (`pixi run esp first-check` reads it):
/// about 1 KB, so it goes out a chunk a tick (`TextLog::long`).
fn config_line(text: &mut TextLog, now: u32, c: &status_lines::Config<'_>) {
    text.long(&c.text(now));
}

/// Which devices answered at boot: `DEVICES 4/4 answered t=… steering=0x05,answered,analog …`.
/// (Not `DEVICES t=`: that is QEMU's line of what the stand-ins were told.)
fn devices_line(text: &mut TextLog, now: u32, found: &DeviceCheck) {
    let mut list = LineText::<160>::new();
    for f in found.iter() {
        let d = device_seen(f);
        let _ = write!(
            list,
            " {}={:#04x},{}",
            d.name,
            d.address,
            if d.answered { "answered" } else { "missing" }
        );
        if let Some(feedback) = d.feedback {
            let _ = write!(list, ",{feedback}");
        }
    }
    text.line(format_args!(
        "DEVICES {}/{} answered t={}{}",
        found.answered(),
        found.expected(),
        now,
        list.as_str()
    ));
}

/// Once a second: is it working? The Pi link's verdict, its counts, the devices at boot,
/// the faults and the mode.
fn health_line(
    text: &mut TextLog,
    now: u32,
    health: Option<&LinkReport>,
    found: &DeviceCheck,
    out: &Outputs,
) {
    let (link, age, moving, s) = match health {
        Some(h) => (
            h.verdict.name(),
            h.age_ms,
            match h.counter_moving {
                Some(true) => "yes",
                Some(false) => "no",
                None => "unknown",
            },
            h.stats,
        ),
        None => ("can", None, "unknown", spine_board::LinkStats::default()),
    };
    let mut age_text = LineText::<12>::new();
    let _ = match age {
        Some(ms) => write!(age_text, "{ms}"),
        None => age_text.write_str("none"),
    };
    text.line(format_args!(
        "HEALTH t={} link={} age_ms={} counter_moving={} frames={} bad={} skipped={} repeats={} devices={}/{} faults={:#06x} mode={:?} armed={}",
        now,
        link,
        age_text.as_str(),
        moving,
        s.frames,
        s.bad,
        s.skipped,
        s.repeats,
        found.answered(),
        found.expected(),
        out.faults,
        out.mode,
        out.armed
    ));
}

/// esp-hal's name for the last reset (`ChipPowerOn`, `CoreMwdt1`, ...), kept for the
/// program's life. A name too long is cut: it is only for people.
fn reset_name(reason: Option<impl fmt::Debug>) -> &'static str {
    static NAME: StaticCell<LineText<32>> = StaticCell::new();
    let name = NAME.init(LineText::new());
    let _ = match reason {
        Some(r) => write!(name, "{r:?}"),
        None => name.write_str("Unknown"),
    };
    let name: &'static LineText<32> = name;
    name.as_str()
}

/// A short text, built in place.
struct LineText<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> LineText<N> {
    const fn new() -> Self {
        LineText {
            buf: [0; N],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl<const N: usize> fmt::Write for LineText<N> {
    /// Keeps what fits, whole characters only; says so when it cuts.
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            let mut utf8 = [0; 4];
            let bytes = c.encode_utf8(&mut utf8).as_bytes();
            let end = self.len + bytes.len();
            let Some(dest) = self.buf.get_mut(self.len..end) else {
                return Err(fmt::Error);
            };
            dest.copy_from_slice(bytes);
            self.len = end;
        }
        Ok(())
    }
}

/// In QEMU: what the stand-in devices were last told, whenever it changes. These are the
/// bytes the car's Jrks, DAC and expander would have got.
#[cfg(feature = "qemu")]
fn stand_in_line(text: &mut TextLog, now: u32, d: &DeviceTargets) {
    let show = |v: Option<u16>| v.map_or(-1, i32::from);
    text.line(format_args!(
        "DEVICES t={} steering={} brake={} dac={} expander={:#06x}",
        now,
        show(d.steering),
        show(d.brake),
        show(d.dac),
        d.expander.unwrap_or(0)
    ));
}

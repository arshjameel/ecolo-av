//! The ESP32's status lines: one JSON object a line, as `firmware/status/README.md` says.
//!
//! | `t` | Type | When |
//! |---|---|---|
//! | `hb` | [`Heartbeat`] | once a second |
//! | `status` | [`Status`] | 5 to 20 times a second |
//! | `event` | [`Event`] | at each change |
//! | `health` | [`Health`] | once a second: the Pi link's verdict, the devices, the faults |
//! | `config` | [`Config`] | at boot and every 10 s: the build and the numbers it runs with |
//! | `devices` | [`DeviceList`] | at boot and every 10 s: which I2C devices answered at boot |
//!
//! A [`Line`] is one of them with its `seq` and `up_ms`. It is [`Display`],
//! so it can go straight to a log, or into a fixed buffer with [`Line::write_to`] or a
//! [`LineBuf`] for a socket. Nothing allocates. [`Spread`] keeps the lowest and highest
//! device values between two status lines; [`Trickle`] hands lines to a slow serial log a
//! few bytes a tick.
//!
//! The lines are built from values the tick already has: this crate reads nothing.
//! `golden.jsonl` beside the contract is the test: every line here is checked against it
//! byte for byte.
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

mod json;
mod spread;
mod trickle;

use core::fmt::{self, Display, Formatter, Write};

pub use spread::Spread;
pub use trickle::{Dropped, Trickle};

use json::{Float, Text};

/// The contract's schema number (`firmware/status/README.md`).
pub const SCHEMA: u32 = 1;

/// A buffer this long holds any line this crate writes, with room to spare: the longest
/// status line, every number at its widest, is 783 bytes, and the config line, every
/// number at its widest with 160 bytes of build, features and names, under 1400 (tests
/// check both). The strings in the heartbeat, the event, the health, the config and the
/// device lines are the caller's; keep them short.
pub const MAX_LINE: usize = 1408;

/// How many RC channels a status line carries at most.
pub const MAX_CHANNELS: usize = 14;

/// One line: its kind and values, its number and the board's uptime.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line<'a> {
    /// Counts up by one for every line of any kind: a gap means lines were lost.
    pub seq: u32,
    /// Milliseconds since boot.
    pub up_ms: u32,
    /// The line's kind and its values.
    pub body: Body<'a>,
}

/// What a line says.
// no_std, no heap: a line lives on the stack for one print, so a Box would cost more.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Body<'a> {
    /// `hb`.
    Heartbeat(Heartbeat<'a>),
    /// `status`.
    Status(Status<'a>),
    /// `event`.
    Event(Event<'a>),
    /// `health`.
    Health(Health<'a>),
    /// `config`.
    Config(Config<'a>),
    /// `devices`.
    Devices(DeviceList<'a>),
}

/// `hb`: the board is alive, and which program it runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heartbeat<'a> {
    /// Random at each boot: a new one means the board restarted.
    pub boot_id: u32,
    /// The program's git commit, short.
    pub build: &'a str,
    /// The features it was built with, comma-separated.
    pub features: &'a str,
    /// Why the chip last reset, as the HAL names it.
    pub last_reset: &'a str,
}

/// `status`: what the board received, what it asked of each device, and how it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Status<'a> {
    /// The last good frame from the Pi; `None` before the first.
    pub rx: Option<Rx>,
    /// What it asked of each device this tick.
    pub out: Out,
    /// The lowest and highest of `out` since the last status line ([`Spread`]).
    pub low: Levels,
    /// The highest of `out` since the last status line.
    pub high: Levels,
    /// Each Jrk's position.
    pub pos: Positions,
    /// The Pi link's counts and verdict.
    pub link: LinkCounts,
    /// The RC.
    pub rc: Rc<'a>,
    /// The spine's state.
    pub state: State,
    /// The tick's timing.
    pub tick: Tick,
}

/// The last good frame from the Pi.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rx {
    /// The frame's version: 1, 2 or 3.
    pub version: u8,
    /// The Orin's counter; `None` in version 1 (written `null`).
    pub counter: Option<u8>,
    /// The frame's last byte: its sum or CRC.
    pub check: u8,
    /// Steering, speed and accel, -1..1, as the frame said. A value that is not a
    /// finite number is written `null`.
    pub steer: f32,
    /// The frame's speed, -1..1.
    pub speed: f32,
    /// The frame's accel, -1..1.
    pub accel: f32,
    /// How long ago it came.
    pub age_ms: u32,
}

/// What the firmware asked of each device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Out {
    /// The steering Jrk's target (counts).
    pub steer_counts: u16,
    /// The brake Jrk's target (counts).
    pub brake_counts: u16,
    /// The throttle DAC's code.
    pub dac: u16,
    /// The direction output: true for reverse.
    pub reverse: bool,
}

/// The three device values that [`Spread`] follows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Levels {
    /// The steering Jrk's target (counts).
    pub steer_counts: u16,
    /// The brake Jrk's target (counts).
    pub brake_counts: u16,
    /// The throttle DAC's code.
    pub dac: u16,
}

impl From<&Out> for Levels {
    fn from(out: &Out) -> Self {
        Levels {
            steer_counts: out.steer_counts,
            brake_counts: out.brake_counts,
            dac: out.dac,
        }
    }
}

/// Each Jrk's position in counts; `None` when not known (written `null`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Positions {
    /// The steering Jrk's.
    pub steer: Option<u16>,
    /// The brake Jrk's.
    pub brake: Option<u16>,
}

/// The Pi link's counts since boot, and its verdict now.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LinkCounts {
    /// Frames that passed their checksum or CRC.
    pub frames: u32,
    /// Frames turned down: a checksum or CRC, a version, a flag.
    pub bad: u32,
    /// Stray bytes skipped between frames.
    pub skipped: u32,
    /// Good frames whose counter hadn't moved.
    pub repeats: u32,
    /// The board's verdict (`spine-board`'s `LinkHealth`); `None` (written `null`) on a
    /// link it doesn't judge (CAN).
    pub verdict: Option<Verdict>,
}

/// The Pi link's state, as the board judges it, by the spine's own 100 ms rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// No good frame since boot.
    NoFrames,
    /// A new command (a new counter) within 100 ms.
    Ok,
    /// Good frames came, then stopped.
    Stale,
    /// Frames come, but the Orin's counter doesn't move: its output mux isn't running.
    CounterFrozen,
    /// Bad frames or stray bytes in the last few seconds.
    BadFrames,
}

impl Verdict {
    /// Its name in a line.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Verdict::NoFrames => "no_frames",
            Verdict::Ok => "ok",
            Verdict::Stale => "stale",
            Verdict::CounterFrozen => "counter_frozen",
            Verdict::BadFrames => "bad_frames",
        }
    }
}

/// `health`: is the board working, in one small object. Also what `/health` serves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Health<'a> {
    /// `None` on a link the board doesn't judge (CAN).
    pub link: Option<Verdict>,
    /// How old the last good frame is; `None` before the first.
    pub age_ms: Option<u32>,
    /// `None`: no frames to tell from, or no counter (version 1 frames).
    pub counter_moving: Option<bool>,
    /// The link's counts since boot.
    pub counts: LinkCounts,
    /// The I2C devices that answered at boot, of how many, and the names of the rest.
    pub devices_answered: u8,
    /// The I2C devices the board looks for.
    pub devices_expected: u8,
    /// The names of those that didn't answer.
    pub missing: &'a [&'a str],
    /// `spine-core`'s fault bits.
    pub faults: u16,
    /// Who drives.
    pub mode: Mode,
    /// Why the chip last reset, as the HAL names it.
    pub last_reset: &'a str,
    /// The program's git commit, short.
    pub build: &'a str,
    /// Random at each boot.
    pub boot_id: u32,
}

/// `config`: the build, and the numbers the firmware runs with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config<'a> {
    /// The program's git commit, short.
    pub build: &'a str,
    /// The features it was built with, comma-separated.
    pub features: &'a str,
    /// The program's version (Cargo's).
    pub version: &'a str,
    /// `uart` or `can`.
    pub link: &'a str,
    /// The UART link's frame version (1 or 2); `None` on CAN.
    pub frame_version: Option<u8>,
    /// The spine's tick.
    pub tick_ms: u32,
    /// The spine's command watchdog.
    pub watchdog_ms: u32,
    /// A command older than this stops the car.
    pub cmd_timeout_ms: u32,
    /// The brake while nobody drives, 0..1.
    pub idle_brake: f32,
    /// The open speed loop's scale, m/s.
    pub full_throttle_speed: f32,
    /// The I2C bus speed; `None` with stand-in devices (QEMU).
    pub i2c_khz: Option<u32>,
    /// The I2C devices' addresses.
    pub addresses: Addresses,
    /// The steering Jrk's targets.
    pub steering: SteeringLimits,
    /// The brake Jrk's targets.
    pub brake: BrakeLimits,
    /// The DAC code for full throttle.
    pub throttle_full_code: u16,
    /// Why the chip last reset, as the HAL names it.
    pub last_reset: &'a str,
    /// The rest of the steering's settings, written inside `steering` after `max`.
    pub steering_more: SteeringSettings<'a>,
    /// The rest of the brake's settings, written inside `brake` after `full`.
    pub brake_more: BrakeSettings,
    /// The speed limits and the throttle caps (`speed`).
    pub speed: SpeedSettings,
    /// How long before each lost input stops the car (`timeouts`).
    pub timeouts: Timeouts,
    /// How fast the board talks to the Pi, the RC receiver and the devices (`rates`).
    pub rates: Rates,
}

/// The steering's settings beyond its three targets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteeringSettings<'a> {
    /// Jrk target counts a radian: radians to counts.
    pub counts_per_rad: f32,
    /// The spine's steering limit, either way (rad).
    pub max_angle_rad: f32,
    /// The fastest the spine moves the steering (rad/s).
    pub rate_rad_s: f32,
    /// The steering calibration's jog and return rate (counts/s); `None` without it (CAN).
    pub cal_rate: Option<f32>,
    /// The calibration ends after this long with no fresh frame asking (ms).
    pub cal_timeout_ms: Option<u32>,
    /// What the steering does when the spine stops the car: `hold` or `centre`.
    pub on_stop: &'a str,
}

/// The brake's settings beyond released and full, each 0..1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakeSettings {
    /// On every stop the spine makes.
    pub stop: f32,
    /// While the car comes back under the speed limit.
    pub overspeed: f32,
    /// The RC's brake band: the left stick this near its middle, a share of its travel
    /// each way, is the full brake (0..1).
    pub rc_band: f32,
    /// The RC's brake before forward and reverse change, and normal mode's window (ms).
    pub rc_brake_ms: u32,
    /// The mode switch under this is the transmitter's failsafe: RC lost (µs).
    pub rc_lost_below: u16,
    /// The board has an RC receiver: the RC must stay in normal mode too.
    pub rc_fitted: bool,
}

/// The speed limits, and the throttle they allow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedSettings {
    /// The fastest forward and backward, whoever drives (m/s).
    pub max_forward: f32,
    /// The fastest backward (m/s).
    pub max_reverse: f32,
    /// The highest DAC code an open-loop command can get, forward and in reverse: the
    /// limit over `full_throttle_speed`. `None` with a wheel speed (the throttle is cut at
    /// the limit instead).
    pub cap_code: Option<u16>,
    /// The same, in reverse.
    pub reverse_cap_code: Option<u16>,
    /// The DAC code the UART frame's speed 1.0 is scaled to (the C++'s `DAC_MAX_CAP`);
    /// `None` on CAN.
    pub frame_dac_cap: Option<u16>,
    /// The fastest the DAC code may rise (codes/s); it falls at once.
    pub rise_per_s: u16,
    /// The speed loop's speeding-up and slowing-down caps (m/s²).
    pub max_accel: f32,
    /// The slowing-down cap (m/s²).
    pub max_decel: f32,
    /// Measured this far over the limit trips the overspeed rule (m/s).
    pub overspeed_margin: f32,
}

/// How long each input may go quiet before the spine stops the car (ms). The command
/// timeout and the watchdog are `cmd_timeout_ms` and `watchdog_ms`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timeouts {
    /// No good iBUS frame for this long: the RC is lost.
    pub rc_lost_ms: u32,
    /// No wheel speed for this long, once one came.
    pub speed_lost_ms: u32,
    /// Still over the limit this long after tripping: a hard stop.
    pub overspeed_ms: u32,
    /// The e-stop input must read closed this long to release.
    pub estop_release_ms: u32,
    /// Bad frames this recent make the Pi link's verdict `bad_frames`; `None` on CAN.
    pub bad_frames_ms: Option<u32>,
}

/// How fast the board talks to each thing. The tick and the I2C bus are `tick_ms` and
/// `i2c_khz`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rates {
    /// The Motor-Pi's UART; `None` on CAN.
    pub uart_baud: Option<u32>,
    /// The frames a second the Motor-Pi sends; `None` on CAN.
    pub frame_hz: Option<u32>,
    /// The uplink's period; `None` without one.
    pub uplink_ms: Option<u32>,
    /// The CAN bus; `None` on the UART link.
    pub can_kbit: Option<u32>,
    /// The RC receiver's iBUS.
    pub ibus_baud: u32,
    /// How often each Jrk and the DAC get their target.
    pub jrk_write_ms: u32,
    /// How often the DAC gets its code.
    pub dac_write_ms: u32,
    /// How often a status line is written.
    pub status_ms: u32,
}

/// The four I2C devices' addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Addresses {
    /// The steering Jrk.
    pub steering: u8,
    /// The brake Jrk.
    pub brake: u8,
    /// The throttle DAC.
    pub dac: u8,
    /// The I/O expander (the direction pin).
    pub expander: u8,
}

/// The steering Jrk's targets: never beyond `min` and `max`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SteeringLimits {
    /// The lowest target.
    pub min: u16,
    /// Straight ahead.
    pub centre: u16,
    /// The highest target.
    pub max: u16,
}

/// The brake Jrk's targets, released and full.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BrakeLimits {
    /// The brake let go.
    pub released: u16,
    /// The brake full on.
    pub full: u16,
}

/// `devices`: which I2C devices answered at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceList<'a> {
    /// Each device looked for.
    pub devices: &'a [DeviceSeen<'a>],
}

/// One I2C device at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceSeen<'a> {
    /// What it is: `steering`, `dac`, ...
    pub name: &'a str,
    /// Its I2C address.
    pub address: u8,
    /// It answered.
    pub answered: bool,
    /// A Jrk's feedback mode (`none`, `analog`, `frequency`, `other`, or `unknown` when
    /// the read failed); `None` (written `null`) for the others.
    pub feedback: Option<&'a str>,
}

/// The RC: its mode switch, and the raw channels (none while it is silent).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rc<'a> {
    /// Where the mode switch is.
    pub switch: Switch,
    /// At most [`MAX_CHANNELS`] are written.
    pub channels: &'a [u16],
}

/// The RC's mode switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Switch {
    /// The computer drives.
    Computer,
    /// RC through ROS (the three-position switch's middle).
    ViaRos,
    /// RC straight to this board.
    Direct,
    /// The receiver is silent, or the switch is out of range.
    NoSignal,
}

impl Switch {
    /// Its name in a line.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Switch::Computer => "computer",
            Switch::ViaRos => "via_ros",
            Switch::Direct => "direct",
            Switch::NoSignal => "no_signal",
        }
    }
}

/// Who drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Nobody: the actuators are idle (`spine-core`'s `Manual`).
    Stopped,
    /// A person, on the RC.
    Rc,
    /// The computer.
    Computer,
    /// The hardware test mode.
    Test,
}

impl Mode {
    /// Its name in a line.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Mode::Stopped => "stopped",
            Mode::Rc => "rc",
            Mode::Computer => "computer",
            Mode::Test => "test",
        }
    }
}

/// The spine's state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    /// Who drives.
    pub mode: Mode,
    /// The car is armed.
    pub armed: bool,
    /// An e-stop is on.
    pub estop: bool,
    /// `spine-core`'s fault bits.
    pub faults: u16,
}

/// The tick's timing since boot (`spine-board`'s `TickTiming`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Tick {
    /// The longest gap between two ticks (µs).
    pub worst_gap_us: u32,
    /// Ticks that ran late.
    pub overruns: u32,
}

/// `event`: something changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Event<'a> {
    /// What changed: `mode`, `armed`, ...
    pub what: &'a str,
    /// Its value before.
    pub from: Value<'a>,
    /// Its value now.
    pub to: Value<'a>,
}

/// One value in an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Value<'a> {
    /// A name, written as a string.
    Text(&'a str),
    /// Written `true` or `false`.
    Flag(bool),
    /// Written as a number.
    Number(u32),
}

/// The line did not fit in the buffer. Nothing in it is to be used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TooLong;

impl Display for TooLong {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_str("the status line did not fit in its buffer")
    }
}

impl Line<'_> {
    /// The line into `buf`, without a newline. Returns the part of `buf` it filled.
    ///
    /// # Errors
    ///
    /// [`TooLong`] if it doesn't fit; `buf` then holds nothing to use.
    pub fn write_to<'b>(&self, buf: &'b mut [u8]) -> Result<&'b str, TooLong> {
        let mut w = SliceWriter { buf, len: 0 };
        write!(w, "{self}").map_err(|_| TooLong)?;
        let SliceWriter { buf, len } = w;
        // Only whole `str`s went in, so the bytes are UTF-8.
        core::str::from_utf8(&buf[..len]).map_err(|_| TooLong)
    }
}

impl Display for Line<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let t = match self.body {
            Body::Heartbeat(_) => "hb",
            Body::Status(_) => "status",
            Body::Event(_) => "event",
            Body::Health(_) => "health",
            Body::Config(_) => "config",
            Body::Devices(_) => "devices",
        };
        write!(
            f,
            r#"{{"t":"{t}","schema":{SCHEMA},"seq":{},"up_ms":{}"#,
            self.seq, self.up_ms
        )?;
        match &self.body {
            Body::Heartbeat(hb) => write!(
                f,
                r#","boot_id":"{:08x}","build":{},"features":{},"last_reset":{}"#,
                hb.boot_id,
                Text(hb.build),
                Text(hb.features),
                Text(hb.last_reset)
            )?,
            Body::Status(s) => s.fmt(f)?,
            Body::Event(e) => write!(
                f,
                r#","what":{},"from":{},"to":{}"#,
                Text(e.what),
                e.from,
                e.to
            )?,
            Body::Health(h) => h.fmt(f)?,
            Body::Config(c) => c.fmt(f)?,
            Body::Devices(d) => d.fmt(f)?,
        }
        f.write_char('}')
    }
}

/// A status line's fields after `up_ms`, each with its leading comma.
impl Display for Status<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_str(r#","rx":"#)?;
        match &self.rx {
            None => f.write_str("null")?,
            Some(rx) => {
                write!(f, r#"{{"v":{},"counter":"#, rx.version)?;
                json::option(f, rx.counter)?;
                write!(
                    f,
                    r#","check":{},"steer":{},"speed":{},"accel":{},"age_ms":{}}}"#,
                    rx.check,
                    Float(rx.steer),
                    Float(rx.speed),
                    Float(rx.accel),
                    rx.age_ms
                )?;
            }
        }
        let o = &self.out;
        write!(
            f,
            r#","out":{{"steer_counts":{},"brake_counts":{},"dac":{},"reverse":{}}}"#,
            o.steer_counts, o.brake_counts, o.dac, o.reverse
        )?;
        write!(f, r#","low":{},"high":{}"#, self.low, self.high)?;
        f.write_str(r#","pos":{"steer":"#)?;
        json::option(f, self.pos.steer)?;
        f.write_str(r#","brake":"#)?;
        json::option(f, self.pos.brake)?;
        let l = &self.link;
        write!(
            f,
            r#"}},"link":{{"frames":{},"bad":{},"skipped":{},"repeats":{},"verdict":"#,
            l.frames, l.bad, l.skipped, l.repeats
        )?;
        verdict(f, l.verdict)?;
        f.write_char('}')?;
        write!(f, r#","rc":{{"switch":"{}","ch":["#, self.rc.switch.name())?;
        for (i, ch) in self.rc.channels.iter().take(MAX_CHANNELS).enumerate() {
            if i > 0 {
                f.write_char(',')?;
            }
            write!(f, "{ch}")?;
        }
        let s = &self.state;
        write!(
            f,
            r#"]}},"state":{{"mode":"{}","armed":{},"estop":{},"faults":{}}}"#,
            s.mode.name(),
            s.armed,
            s.estop,
            s.faults
        )?;
        write!(
            f,
            r#","tick":{{"worst_gap_us":{},"overruns":{}}}"#,
            self.tick.worst_gap_us, self.tick.overruns
        )
    }
}

/// A verdict, quoted, or `null`.
fn verdict(f: &mut Formatter<'_>, v: Option<Verdict>) -> fmt::Result {
    match v {
        Some(v) => write!(f, r#""{}""#, v.name()),
        None => f.write_str("null"),
    }
}

/// A health line's fields after `up_ms`.
impl Display for Health<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_str(r#","link":"#)?;
        verdict(f, self.link)?;
        f.write_str(r#","age_ms":"#)?;
        json::option(f, self.age_ms)?;
        f.write_str(r#","counter_moving":"#)?;
        json::option(f, self.counter_moving)?;
        let c = &self.counts;
        write!(
            f,
            r#","frames":{},"bad":{},"skipped":{},"repeats":{},"devices":{{"answered":{},"expected":{},"missing":["#,
            c.frames, c.bad, c.skipped, c.repeats, self.devices_answered, self.devices_expected
        )?;
        for (i, name) in self.missing.iter().enumerate() {
            if i > 0 {
                f.write_char(',')?;
            }
            Text(name).fmt(f)?;
        }
        write!(
            f,
            r#"]}},"faults":{},"mode":"{}","last_reset":{},"build":{},"boot_id":"{:08x}""#,
            self.faults,
            self.mode.name(),
            Text(self.last_reset),
            Text(self.build),
            self.boot_id
        )
    }
}

/// A config line's fields after `up_ms`.
impl Display for Config<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            r#","build":{},"features":{},"version":{},"link":{},"frame_version":"#,
            Text(self.build),
            Text(self.features),
            Text(self.version),
            Text(self.link)
        )?;
        json::option(f, self.frame_version)?;
        write!(
            f,
            r#","tick_ms":{},"watchdog_ms":{},"cmd_timeout_ms":{},"idle_brake":{},"full_throttle_speed":{},"i2c_khz":"#,
            self.tick_ms,
            self.watchdog_ms,
            self.cmd_timeout_ms,
            Float(self.idle_brake),
            Float(self.full_throttle_speed)
        )?;
        json::option(f, self.i2c_khz)?;
        let (a, s, b) = (&self.addresses, &self.steering, &self.brake);
        write!(
            f,
            r#","addresses":{{"steering":{},"brake":{},"dac":{},"expander":{}}}"#,
            a.steering, a.brake, a.dac, a.expander
        )?;
        let (sm, bm) = (&self.steering_more, &self.brake_more);
        write!(
            f,
            r#","steering":{{"min":{},"centre":{},"max":{},"counts_per_rad":{},"max_angle_rad":{},"rate_rad_s":{},"cal_rate":"#,
            s.min,
            s.centre,
            s.max,
            Float(sm.counts_per_rad),
            Float(sm.max_angle_rad),
            Float(sm.rate_rad_s)
        )?;
        json::option(f, sm.cal_rate.map(Float))?;
        f.write_str(r#","cal_timeout_ms":"#)?;
        json::option(f, sm.cal_timeout_ms)?;
        write!(
            f,
            r#","on_stop":{}}},"brake":{{"released":{},"full":{},"stop":{},"overspeed":{},"rc_band":{},"rc_brake_ms":{},"rc_lost_below":{},"rc_fitted":{}}}"#,
            Text(sm.on_stop),
            b.released,
            b.full,
            Float(bm.stop),
            Float(bm.overspeed),
            Float(bm.rc_band),
            bm.rc_brake_ms,
            bm.rc_lost_below,
            bm.rc_fitted
        )?;
        write!(
            f,
            r#","throttle_full_code":{},"last_reset":{}"#,
            self.throttle_full_code,
            Text(self.last_reset)
        )?;
        let sp = &self.speed;
        write!(
            f,
            r#","speed":{{"max_forward":{},"max_reverse":{},"cap_code":"#,
            Float(sp.max_forward),
            Float(sp.max_reverse)
        )?;
        json::option(f, sp.cap_code)?;
        f.write_str(r#","reverse_cap_code":"#)?;
        json::option(f, sp.reverse_cap_code)?;
        f.write_str(r#","frame_dac_cap":"#)?;
        json::option(f, sp.frame_dac_cap)?;
        write!(
            f,
            r#","rise_per_s":{},"max_accel":{},"max_decel":{},"overspeed_margin":{}}}"#,
            sp.rise_per_s,
            Float(sp.max_accel),
            Float(sp.max_decel),
            Float(sp.overspeed_margin)
        )?;
        let t = &self.timeouts;
        write!(
            f,
            r#","timeouts":{{"rc_lost_ms":{},"speed_lost_ms":{},"overspeed_ms":{},"estop_release_ms":{},"bad_frames_ms":"#,
            t.rc_lost_ms, t.speed_lost_ms, t.overspeed_ms, t.estop_release_ms
        )?;
        json::option(f, t.bad_frames_ms)?;
        let r = &self.rates;
        f.write_str(r#"},"rates":{"uart_baud":"#)?;
        json::option(f, r.uart_baud)?;
        f.write_str(r#","frame_hz":"#)?;
        json::option(f, r.frame_hz)?;
        f.write_str(r#","uplink_ms":"#)?;
        json::option(f, r.uplink_ms)?;
        f.write_str(r#","can_kbit":"#)?;
        json::option(f, r.can_kbit)?;
        write!(
            f,
            r#","ibus_baud":{},"jrk_write_ms":{},"dac_write_ms":{},"status_ms":{}}}"#,
            r.ibus_baud, r.jrk_write_ms, r.dac_write_ms, r.status_ms
        )
    }
}

impl<'a> Config<'a> {
    /// The text form of this line, for the USB cable's text log (`CONFIG t=… …`), `now_ms`
    /// after boot. Its first keys, up to `last_reset`, are the ones the first check has
    /// always read; the rest follow in the JSON line's order. An unknown value is `none`
    /// (JSON's `null`), never 0: `frame=none` on CAN, `i2c_khz=none` with QEMU's stand-ins.
    #[must_use]
    pub fn text(&self, now_ms: u32) -> ConfigText<'_, 'a> {
        ConfigText {
            config: self,
            now_ms,
        }
    }
}

/// [`Config::text`]: one line, `CONFIG t=… key=value …`, without a newline.
#[derive(Debug, Clone, Copy)]
pub struct ConfigText<'c, 'a> {
    config: &'c Config<'a>,
    now_ms: u32,
}

/// A number in a text line, or `none`.
struct Maybe<T>(Option<T>);

impl<T: Display> Display for Maybe<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(v) => v.fmt(f),
            None => f.write_str("none"),
        }
    }
}

impl Display for ConfigText<'_, '_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let c = self.config;
        let a = &c.addresses;
        write!(
            f,
            "CONFIG t={} build={} features={} version={} link={} frame={} tick_ms={} watchdog_ms={} cmd_timeout_ms={} idle_brake={:.2} full_throttle_speed={:.2} i2c_khz={} steering_jrk={:#04x} brake_jrk={:#04x} dac={:#04x} expander={:#04x} steer_min={} steer_centre={} steer_max={} brake_released={} brake_full={} throttle_full_code={} last_reset={}",
            self.now_ms,
            c.build,
            c.features,
            c.version,
            c.link,
            Maybe(c.frame_version),
            c.tick_ms,
            c.watchdog_ms,
            c.cmd_timeout_ms,
            c.idle_brake,
            c.full_throttle_speed,
            Maybe(c.i2c_khz),
            a.steering,
            a.brake,
            a.dac,
            a.expander,
            c.steering.min,
            c.steering.centre,
            c.steering.max,
            c.brake.released,
            c.brake.full,
            c.throttle_full_code,
            c.last_reset
        )?;
        let (s, b, sp, t, r) = (
            &c.steering_more,
            &c.brake_more,
            &c.speed,
            &c.timeouts,
            &c.rates,
        );
        write!(
            f,
            " steer_counts_per_rad={:.1} steer_max_rad={:.3} steer_rate_rad_s={:.2} steer_cal_rate={} steer_cal_timeout_ms={} steer_on_stop={}",
            s.counts_per_rad,
            s.max_angle_rad,
            s.rate_rad_s,
            Maybe(s.cal_rate),
            Maybe(s.cal_timeout_ms),
            s.on_stop
        )?;
        write!(
            f,
            " brake_stop={:.2} brake_overspeed={:.2} rc_band={:.2} rc_brake_ms={} rc_lost_below={} rc_fitted={}",
            b.stop, b.overspeed, b.rc_band, b.rc_brake_ms, b.rc_lost_below, b.rc_fitted
        )?;
        write!(
            f,
            " max_speed={:.2} max_reverse_speed={:.2} throttle_cap_code={} reverse_cap_code={} frame_dac_cap={} dac_rise_per_s={} max_accel={:.2} max_decel={:.2} overspeed_margin={:.2}",
            sp.max_forward,
            sp.max_reverse,
            Maybe(sp.cap_code),
            Maybe(sp.reverse_cap_code),
            Maybe(sp.frame_dac_cap),
            sp.rise_per_s,
            sp.max_accel,
            sp.max_decel,
            sp.overspeed_margin
        )?;
        write!(
            f,
            " rc_lost_ms={} speed_lost_ms={} overspeed_ms={} estop_release_ms={} bad_frames_ms={}",
            t.rc_lost_ms,
            t.speed_lost_ms,
            t.overspeed_ms,
            t.estop_release_ms,
            Maybe(t.bad_frames_ms)
        )?;
        write!(
            f,
            " uart_baud={} frame_hz={} uplink_ms={} can_kbit={} ibus_baud={} jrk_write_ms={} dac_write_ms={} status_ms={}",
            Maybe(r.uart_baud),
            Maybe(r.frame_hz),
            Maybe(r.uplink_ms),
            Maybe(r.can_kbit),
            r.ibus_baud,
            r.jrk_write_ms,
            r.dac_write_ms,
            r.status_ms
        )
    }
}

/// A devices line's fields after `up_ms`.
impl Display for DeviceList<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let answered = self.devices.iter().filter(|d| d.answered).count();
        write!(
            f,
            r#","answered":{},"expected":{},"list":["#,
            answered,
            self.devices.len()
        )?;
        for (i, d) in self.devices.iter().enumerate() {
            if i > 0 {
                f.write_char(',')?;
            }
            write!(
                f,
                r#"{{"name":{},"address":{},"answered":{},"feedback":"#,
                Text(d.name),
                d.address,
                d.answered
            )?;
            match d.feedback {
                Some(name) => Text(name).fmt(f)?,
                None => f.write_str("null")?,
            }
            f.write_char('}')?;
        }
        f.write_char(']')
    }
}

impl Display for Levels {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            r#"{{"steer_counts":{},"brake_counts":{},"dac":{}}}"#,
            self.steer_counts, self.brake_counts, self.dac
        )
    }
}

impl Display for Value<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match *self {
            Value::Text(s) => Text(s).fmt(f),
            Value::Flag(b) => write!(f, "{b}"),
            Value::Number(n) => write!(f, "{n}"),
        }
    }
}

/// A line in a buffer of its own, `N` bytes: for a queue or a socket.
#[derive(Clone)]
pub struct LineBuf<const N: usize = MAX_LINE> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> LineBuf<N> {
    /// An empty buffer.
    #[must_use]
    pub const fn new() -> Self {
        LineBuf {
            buf: [0; N],
            len: 0,
        }
    }

    /// Replace what it holds with `line`. On [`TooLong`] it holds nothing.
    ///
    /// # Errors
    ///
    /// [`TooLong`] if the line doesn't fit in `N` bytes.
    pub fn set(&mut self, line: &Line<'_>) -> Result<(), TooLong> {
        self.len = 0;
        let len = line.write_to(&mut self.buf)?.len();
        self.len = len;
        Ok(())
    }

    /// Replace what it holds with any one line of text (a text log line). On [`TooLong`]
    /// it holds nothing.
    ///
    /// # Errors
    ///
    /// [`TooLong`] if the text doesn't fit in `N` bytes.
    pub fn set_text(&mut self, text: &impl Display) -> Result<(), TooLong> {
        self.len = 0;
        let mut w = SliceWriter {
            buf: &mut self.buf,
            len: 0,
        };
        write!(w, "{text}").map_err(|_| TooLong)?;
        self.len = w.len;
        Ok(())
    }

    /// The line, without a newline; empty if none.
    #[must_use]
    pub fn as_str(&self) -> &str {
        // `set` only keeps what `write_to` checked.
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }

    /// The line's bytes, without a newline; empty if none.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// It holds no line.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<const N: usize> Default for LineBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Debug for LineBuf<N> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_tuple("LineBuf").field(&self.as_str()).finish()
    }
}

/// `fmt::Write` into a byte slice; an error once it is full.
struct SliceWriter<'b> {
    buf: &'b mut [u8],
    len: usize,
}

impl Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let end = self.len.checked_add(s.len()).ok_or(fmt::Error)?;
        let dest = self.buf.get_mut(self.len..end).ok_or(fmt::Error)?;
        dest.copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests;

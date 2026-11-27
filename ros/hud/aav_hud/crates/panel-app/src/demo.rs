//! Pretend data for `--demo`: a simulated car that follows the buttons, with every
//! sensor talking. No ROS, no car; it announces itself as SIM, so the speed buttons work.
//!
//! Its steering actuator (the calibration page): ends at 3420 (left) and 610 (right)
//! counts, straight at 1985, so the car steers a little right, as ours. Limp: a pretend
//! hand turns the wheels full left, straight, full right and back, resting 3 s at each.
//! Jog: the firmware's rules (300 counts a second, 400 to 3500). `--demo-bind COUNTS`
//! puts a stiff spot there (the motor can't push past it, away from straight), so the
//! lock-to-lock check finds resistance.
//!
//! The GRAPHS page's pretend path (`PretendLink`): ego drives a slow steering sweep and
//! a speed that stops for 3 s every 20 s; the mux passes it on, the Pi writes frames
//! (10 a second sampled), and the ESP32's status lines say what it got: every hop OK.
//! With `--esp32`, the real board's lines come instead of the pretend ones.

use std::time::{Duration, Instant};

use hud_core::{
    gps::{Fix, RawFix},
    origin::{Announcement, Entry, Origin},
    state::Battery,
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use panel_core::{
    buttons::Command,
    calibration::{FirmwareState, Raw, Refusal, Request},
    esp32::{
        BoardMode, BoardState, Extremes, Heartbeat, LinkCounts, Out, Positions, Rx, Status,
        StatusLine, Tick,
    },
    graphs::{Drive, PiFrame, UplinkHealth},
    state::{Actuator, Esp32News, ImuReading, Row, Update, Wheels},
};

use crate::source::DataSource;

/// ego's own limits, near enough: it slows at 2 m/s², speeds up at 1.
const DECEL: f64 = 2.0;
const ACCEL: f64 = 1.0;
/// The pretend car's wheelbase (vehicle.yaml's).
const WHEELBASE: f64 = 1.57;
/// A command older than this is no command (ego's cmd_timeout).
const STALE: Duration = Duration::from_millis(300);

/// The pretend steering actuator's ends and straight ahead, counts.
const LEFT_END: f64 = 3420.0;
const RIGHT_END: f64 = 610.0;
const STRAIGHT: f64 = 1985.0;
/// The firmware's jog: counts a second, and its software limits.
const CAL_RATE: f64 = 300.0;
const SOFT_MIN: f64 = 400.0;
const SOFT_MAX: f64 = 3500.0;
/// The bridge drops a request this old.
const CAL_STALE: Duration = Duration::from_millis(500);
/// The pretend hand: where it takes the wheels, resting 3 s at each, moving 1.5 s between.
const HAND: [f64; 4] = [LEFT_END, STRAIGHT, RIGHT_END, STRAIGHT];
const HAND_REST: f64 = 3.0;
const HAND_MOVE: f64 = 1.5;

/// The pretend steering actuator and its firmware.
struct Steering {
    position: f64,
    /// The firmware's own target (jog, return), counts.
    target: f64,
    state: FirmwareState,
    request: Option<(Instant, Request)>,
    /// When limp began, for the hand.
    limp_since: Option<Instant>,
    /// A stiff spot (counts): the wheels can't pass it.
    bind: Option<f64>,
}

impl Steering {
    fn new(bind: Option<u16>) -> Self {
        Self {
            position: STRAIGHT,
            target: STRAIGHT,
            state: FirmwareState::Off,
            request: None,
            limp_since: None,
            bind: bind.map(f64::from),
        }
    }

    /// Where the hand has the wheels, `t` seconds into limp (from `from`).
    fn hand(t: f64, from: f64) -> f64 {
        let leg = HAND_REST + HAND_MOVE;
        let i = (t / leg).floor() as usize;
        let into = t - i as f64 * leg;
        let start = if i == 0 {
            from
        } else {
            HAND[(i - 1) % HAND.len()]
        };
        let end = HAND[i % HAND.len()];
        let k = (into / HAND_MOVE).min(1.0);
        start + (end - start) * k
    }

    fn step(&mut self, now: Instant, dt: f64, estop: bool) {
        let fresh = self
            .request
            .filter(|(at, _)| now.saturating_duration_since(*at) < CAL_STALE)
            .map(|(_, r)| r);
        let asking = matches!(fresh, Some(Request::Limp | Request::Jog(_)));
        if asking && estop {
            self.state = FirmwareState::Refused(Refusal::Estop);
            self.limp_since = None;
            self.target = self.position;
        } else {
            match fresh {
                Some(Request::Limp) => {
                    let since = *self.limp_since.get_or_insert(now);
                    if self.state != FirmwareState::Limp {
                        self.target = self.position;
                    }
                    self.state = FirmwareState::Limp;
                    let t = now.saturating_duration_since(since).as_secs_f64();
                    self.position = Self::hand(t, self.target);
                }
                Some(Request::Jog(goal)) => {
                    if self.state == FirmwareState::Limp {
                        self.target = self.position;
                    }
                    self.limp_since = None;
                    self.state = FirmwareState::Jog;
                    let goal = f64::from(goal).clamp(SOFT_MIN, SOFT_MAX);
                    self.target += (goal - self.target).clamp(-CAL_RATE * dt, CAL_RATE * dt);
                }
                Some(Request::Off) | None => {
                    self.limp_since = None;
                    if matches!(
                        self.state,
                        FirmwareState::Limp | FirmwareState::Jog | FirmwareState::Return
                    ) {
                        if self.state != FirmwareState::Return {
                            self.target = self.position;
                        }
                        self.state = FirmwareState::Return;
                        let d = STRAIGHT - self.target;
                        self.target += d.clamp(-CAL_RATE * dt, CAL_RATE * dt);
                        if (STRAIGHT - self.target).abs() < 1.0 {
                            self.state = FirmwareState::Off;
                        }
                    } else {
                        self.state = FirmwareState::Off;
                    }
                }
            }
        }
        if self.state != FirmwareState::Limp {
            // The motor follows its target in about 0.1 s.
            let k = (dt / 0.1).min(1.0);
            let mut next = self.position + (self.target - self.position) * k;
            if let Some(b) = self.bind {
                // The motor can't push past the stiff spot, away from straight; it comes
                // back freely.
                if b > STRAIGHT && self.position <= b {
                    next = next.min(b);
                } else if b < STRAIGHT && self.position >= b {
                    next = next.max(b);
                }
            }
            self.position = next;
        }
        self.position = self.position.clamp(RIGHT_END, LEFT_END);
    }

    fn raw(&self) -> Raw {
        Raw {
            position: Some(self.position.round() as u16),
            state: self.state,
        }
    }

    /// The bridge's line.
    fn line(&self) -> String {
        format!("{} (at {:.0} counts)", self.state, self.position)
    }
}

/// The legacy bridge's mapping, near enough: full lock and full throttle.
const MAX_STEER: f64 = 0.5;
const FULL_SPEED: f64 = 3.0;
const HOLD_BRAKE: f64 = 0.3;

/// The GRAPHS page's pretend path: ego, the mux, the Pi's frames, the ESP32's lines.
struct PretendLink {
    counter: u8,
    written: u32,
    seq: u64,
    /// The last frame written: what the board got.
    last: Option<PiFrame>,
    /// Pretend status lines (not with `--esp32`: the real board's come).
    lines: bool,
}

/// A stand-in for the frame's CRC-8: different values, a different byte.
fn check_byte(counter: u8, steering: f64, speed: f64, accel: f64) -> u8 {
    [steering, speed, accel]
        .iter()
        .flat_map(|v| ((v * 1000.0).round() as i32).to_le_bytes())
        .fold(counter, |c, b| c.rotate_left(3) ^ b)
}

impl PretendLink {
    /// What ego asks `t` s in: rad, m/s.
    fn ego(t: f64) -> (f64, f64) {
        let steering = 0.3 * (t * 0.45).sin() + 0.08 * (t * 1.7).sin();
        let stopped = (t % 20.0) > 9.0 && (t % 20.0) < 12.0;
        let speed = if stopped {
            0.0
        } else {
            0.4 + 0.25 * (t * 0.3).sin()
        };
        (steering, speed)
    }

    fn news(&mut self, t: f64, tick: u64, news: &mut Vec<Update>) {
        let (steering, speed) = Self::ego(t);
        let stamp_ns = (t * 1e9) as u64 + 1;
        let drive = Drive {
            stamp_ns,
            speed,
            steering,
        };
        news.push(Update::EgoCommand(drive));
        news.push(Update::CarCommand(drive));
        self.counter = self.counter.wrapping_add(1);
        self.written += 1;
        // The bridge's mapping (legacy_protocol.to_legacy), roughly.
        let s = (steering / MAX_STEER).clamp(-1.0, 1.0);
        let (v, accel) = if speed < 0.05 {
            (0.0, -HOLD_BRAKE)
        } else {
            ((speed / FULL_SPEED).clamp(0.031, 1.0), 0.0)
        };
        let frame = PiFrame {
            version: 2,
            counter: self.counter,
            check: check_byte(self.counter, s, v, accel),
            armed: true,
            estop: false,
            steering: s,
            speed: v,
            accel,
            written: self.written,
            write_errors: 0,
        };
        // A sample, 10 a second (link_frame_rate_hz).
        if tick.is_multiple_of(6) {
            news.push(Update::LinkFrame(frame));
            self.last = Some(frame);
        }
        if tick.is_multiple_of(60) {
            news.push(Update::UplinkHealth(UplinkHealth {
                fresh: true,
                frames: self.written,
                lost: 0,
                bad: 0,
            }));
        }
        if !self.lines {
            return;
        }
        if tick.is_multiple_of(60) {
            self.seq += 1;
            news.push(Update::Esp32(Esp32News::Line(StatusLine::Heartbeat(
                Heartbeat {
                    seq: self.seq,
                    up_ms: (t * 1000.0) as u64,
                    boot_id: "demo0001".into(),
                    build: "demo".into(),
                    features: "legacy-uart,wifi-status".into(),
                    last_reset: "PowerOn".into(),
                },
            ))));
        }
        // Its status lines, 10 a second, a little after the frames.
        if tick % 6 == 3 {
            let Some(f) = self.last else { return };
            let round = |x: f64| (x * 1000.0).round() / 1000.0;
            let steer_counts = (1985.0 + f.steering * 1300.0) as u16;
            let brake_counts = if f.accel < -0.05 { 2200 } else { 400 };
            let dac = (f.speed * 700.0) as u16;
            let extremes = Extremes {
                steer_counts,
                brake_counts,
                dac,
            };
            self.seq += 1;
            news.push(Update::Esp32(Esp32News::Line(StatusLine::Status(
                Box::new(Status {
                    seq: self.seq,
                    up_ms: (t * 1000.0) as u64,
                    rx: Some(Rx {
                        v: f.version,
                        counter: f.counter,
                        check: f.check,
                        steer: round(f.steering),
                        speed: round(f.speed),
                        accel: round(f.accel),
                        age_ms: 12,
                    }),
                    out: Out {
                        steer_counts,
                        brake_counts,
                        dac,
                        reverse: false,
                    },
                    low: Some(extremes),
                    high: Some(extremes),
                    pos: Positions {
                        steer: Some(steer_counts),
                        brake: None,
                    },
                    link: LinkCounts {
                        frames: self.written,
                        bad: 0,
                        skipped: 0,
                        repeats: 2,
                    },
                    state: BoardState {
                        mode: BoardMode::Computer,
                        armed: true,
                        estop: false,
                        faults: 0,
                    },
                    tick: Tick {
                        worst_gap_us: 10_840,
                        overruns: 0,
                    },
                }),
            ))));
        }
    }
}

/// The pretend car.
pub struct Demo {
    start: Instant,
    last: Instant,
    speed: f64,
    steer: f64,
    command: Option<(Instant, Command)>,
    estop: bool,
    announced: bool,
    ticks: u64,
    steering_actuator: Steering,
    link: PretendLink,
    /// What it announces on `/system/source`: SIM, or REAL to see the real car's rules.
    origin: Origin,
}

impl Demo {
    /// `bind`: a stiff spot in the steering, counts (`--demo-bind`). `esp32_lines`:
    /// pretend status lines (false when the real board's come, `--esp32`).
    pub fn new(now: Instant, bind: Option<u16>, esp32_lines: bool) -> Self {
        Self {
            start: now,
            last: now,
            speed: 0.0,
            steer: 0.0,
            command: None,
            estop: false,
            announced: false,
            ticks: 0,
            steering_actuator: Steering::new(bind),
            link: PretendLink {
                counter: 0,
                written: 0,
                seq: 0,
                last: None,
                lines: esp32_lines,
            },
            origin: Origin::Sim,
        }
    }

    /// Announces `origin` instead of SIM (`map --demo --demo-real`: how the dash screen
    /// looks on the real car). Still pretend: nothing leaves the program.
    #[must_use]
    pub fn announcing(mut self, origin: Origin) -> Self {
        self.origin = origin;
        self
    }

    /// The car follows the last fresh command, as ego would.
    fn step(&mut self, now: Instant) {
        let dt = now.saturating_duration_since(self.last).as_secs_f64();
        self.last = now;
        self.steering_actuator.step(now, dt.min(0.1), self.estop);
        let wanted = match self.command {
            Some((at, c)) if !self.estop && now.saturating_duration_since(at) < STALE => c,
            _ => Command::ZERO,
        };
        let dv = wanted.speed - self.speed;
        let rate = if dv > 0.0 { ACCEL } else { DECEL };
        self.speed += dv.clamp(-rate * dt, rate * dt);
        // ego holds the wheels below 0.05 m/s.
        if self.speed > 0.05 && wanted.speed > 0.05 {
            let target = (wanted.yaw_rate * WHEELBASE / wanted.speed).atan();
            self.steer += (target - self.steer).clamp(-0.5 * dt, 0.5 * dt);
        }
    }
}

impl DataSource for Demo {
    fn label(&self) -> &'static str {
        "DEMO"
    }

    fn is_pretend(&self) -> bool {
        true
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        self.step(now);
        self.ticks += 1;
        let t = now.saturating_duration_since(self.start).as_secs_f64();
        let wiggle = |k: f64| (t * k).sin() * 0.02;
        let mut news = Vec::new();
        if !self.announced {
            self.announced = true;
            news.push(Update::Source(Announcement {
                announcer: "demo".into(),
                entries: vec![Entry {
                    name: "car".into(),
                    topic: "/vehicle/status".into(),
                    origin: self.origin,
                }],
            }));
            news.push(Update::GeofenceLock(false));
        }
        let speed = self.speed as f32;
        news.push(Update::Vehicle(VehicleStatus {
            mode: Mode::from(1),
            armed: true,
            estop_active: self.estop,
            speed: MetresPerSecond(speed),
            steering_angle: Radians(self.steer as f32),
            throttle: Fraction::new((speed / 3.0).min(1.0)),
            brake: Fraction::new(0.0),
            faults: Faults(0),
        }));
        news.push(Update::Steering(Actuator {
            command: self.steer as f32,
            measured: self.steer as f32,
        }));
        news.push(Update::Throttle(Actuator {
            command: f32::NAN,
            measured: (speed / 3.0).min(1.0),
        }));
        news.push(Update::Brake(Actuator {
            command: f32::NAN,
            measured: if self.command.is_none() { 0.2 } else { 0.0 },
        }));
        news.push(Update::Wheels(Wheels {
            front_left: speed,
            front_right: speed,
            rear_left: speed,
            rear_right: speed,
        }));
        news.push(Update::Imu(ImuReading {
            accel: [wiggle(7.0), wiggle(5.0), 9.81 + wiggle(3.0)],
            gyro: [
                wiggle(11.0),
                wiggle(13.0),
                self.speed * self.steer.tan() / WHEELBASE,
            ],
        }));
        news.push(Update::Counted(Row::Lidar));
        if self.ticks.is_multiple_of(2) {
            news.push(Update::Counted(Row::Camera));
        }
        if self.ticks.is_multiple_of(4) {
            news.push(Update::Counted(Row::Radar));
            news.push(Update::Gps(Fix::from(RawFix {
                status: 2,
                latitude: 45.3876,
                longitude: -75.6960,
                altitude: 80.0,
                position_covariance: [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.001],
                position_covariance_type: 2,
            })));
            news.push(Update::Battery(Battery::from_message(51.2, 0.82, -12.0)));
            news.push(Update::GeofenceLock(false));
        }
        // twist_mux passes the buttons' command: nobody else drives here.
        if let Some((at, c)) = self.command {
            if now.saturating_duration_since(at) < STALE {
                news.push(Update::Mux(c));
            }
        }
        news.push(Update::Estop(self.estop));
        self.link.news(t, self.ticks, &mut news);
        news.push(Update::SteeringRaw(self.steering_actuator.raw()));
        if self.ticks.is_multiple_of(30) {
            news.push(Update::CalibrationLine(self.steering_actuator.line()));
        }
        news
    }

    fn press_estop(&mut self) {
        self.estop = true;
    }

    fn frame_drawn(&mut self, command: Option<Command>, calibration: Option<Request>) {
        // The demo's own clock: the last drain's time (so a test can run it fast).
        let now = self.last;
        if let Some(c) = command {
            self.command = Some((now, c));
        }
        if let Some(r) = calibration {
            self.steering_actuator.request = Some((now, r));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_pretend_car_follows_the_buttons_and_stops_without_them() {
        let t = Instant::now();
        let mut demo = Demo::new(t, None, true);
        let ahead = Command::from_steering(0.3, 0.0, WHEELBASE);
        for i in 1..=20 {
            let at = t + Duration::from_millis(100 * i);
            demo.command = Some((at, ahead));
            demo.step(at);
        }
        assert!((demo.speed - 0.3).abs() < 1e-9);
        // No command for a while: it stops.
        demo.step(t + Duration::from_secs(5));
        assert_eq!(demo.speed, 0.0);
    }

    #[test]
    fn the_pretend_steering_jogs_inside_its_limits_and_goes_limp() {
        let t = Instant::now();
        let mut s = Steering::new(None);
        let tick = Duration::from_millis(20);
        let mut now = t;
        for _ in 0..500 {
            now += tick;
            s.request = Some((now, Request::Jog(4000)));
            s.step(now, 0.02, false);
        }
        assert_eq!(s.state, FirmwareState::Jog);
        // The soft limit is 3500, but the wheels stop at their end, 3420.
        assert_eq!(s.raw().position, Some(3420));
        // Limp: the hand takes them to full left first.
        for _ in 0..100 {
            now += tick;
            s.request = Some((now, Request::Limp));
            s.step(now, 0.02, false);
        }
        assert_eq!(s.state, FirmwareState::Limp);
        // The request stops: the firmware returns to straight, slowly, then is off.
        for _ in 0..600 {
            now += tick;
            s.step(now, 0.02, false);
        }
        assert_eq!(s.state, FirmwareState::Off);
        assert!((s.position - STRAIGHT).abs() < 2.0);
        // An e-stop refuses.
        s.request = Some((now, Request::Limp));
        s.step(now, 0.02, true);
        assert_eq!(s.state, FirmwareState::Refused(Refusal::Estop));
    }

    #[test]
    fn a_stiff_spot_stops_the_wheels() {
        let t = Instant::now();
        let mut s = Steering::new(Some(2600));
        let mut now = t;
        for _ in 0..500 {
            now += Duration::from_millis(20);
            s.request = Some((now, Request::Jog(3300)));
            s.step(now, 0.02, false);
        }
        assert_eq!(s.raw().position, Some(2600));
    }
}

#[cfg(test)]
mod link_tests {
    use super::*;
    use panel_core::state::PanelState;

    /// The pretend path, 30 s at 60 frames a second: every hop OK.
    #[test]
    fn the_pretend_path_passes_every_hop() {
        let t0 = Instant::now();
        let mut demo = Demo::new(t0, None, true);
        let mut state = PanelState::new(Duration::from_secs(1));
        let mut now = t0;
        for i in 1..=1800u64 {
            now = t0 + Duration::from_micros(i * 16_667);
            for u in demo.drain(now) {
                state.apply(u, now);
            }
        }
        let hops = state.graphs.hops(now, Duration::from_secs(1));
        for h in &hops {
            assert_eq!(h.verdict.word(), "OK", "{h:#?}");
        }
        let c = state.graphs.counts(now);
        assert_eq!(c.lines_lost, 0);
        assert!(c.esp32.is_some_and(|(_, d)| d.frames > 1000));
    }
}

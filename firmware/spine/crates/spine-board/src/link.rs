//! Where the computer's commands come from: one trait, two links.
//!
//! | Link | Wire | Commands | Reports back |
//! |---|---|---|---|
//! | [`CanLink`] | CAN, through [`Board::can_receive`] (`firmware/can/aav.dbc`) | DRIVE_CMD (a speed), ESTOP_CMD | VEHICLE_FEEDBACK, VEHICLE_STATUS |
//! | [`LegacyLink`] | the Motor-Pi's UART (`legacy-link`): version 2 frames with the Orin's heartbeat, or the 2025-26 13-byte frames | throttle, brake and steering as they are: `DirectCmd`; version 2's e-stop flag | with an uplink ([`LegacyLink::with_uplink`]): the [`Uplink`] status frame at 50 Hz on TX1; otherwise nothing |
//!
//! [`SpineBoard`](crate::SpineBoard) takes either; the rest of the loop is the same. After
//! each tick it hands the link a [`Report`]: what the spine decided, the RC, the Jrks'
//! positions.

use crate::actuators::Positions;
use crate::board::{Board, CanFrame};
use crate::calibration::Calibration;
use crate::spine_board::Every;
use core::convert::Infallible;
use embedded_io::{Read, ReadReady, Write};
use legacy_link::{
    Ask, CalRequest, Counts, DriveMode, Frame, FrameError, FrameV3, Heartbeat, ModeSwitch, Parser,
    ParserV3, Situation, SteerCal, SteeringDrive, Sticks, Targets, Throttle, Uplink,
};
use spine_core::protocol::{decode_incoming, EstopCmd, FrameError as CanError, Incoming, Mode};
use spine_core::{DirectCmd, Inputs, Outputs, RcInput, RcSwitch};

/// The uplink's period: 50 frames a second.
pub const UPLINK_PERIOD_MS: u32 = 20;

/// Limits on one tick's work, so a flood of frames or bytes can't stall the loop.
const MAX_CAN_FRAMES: usize = 32;
const MAX_UART_READS: usize = 16;

/// A source of the computer's commands, and the way back for the spine's reports.
pub trait CommandLink {
    /// Everything that arrived since the last tick, into `inputs`: `drive_cmd`,
    /// `direct_cmd`, `estop_cmd`, `bad_frame`.
    fn receive<B: Board>(&mut self, board: &mut B, inputs: &mut Inputs);

    /// Send one of the spine's reports (a CAN ID and its bytes). True if it went, or if
    /// this link has no way back; false if it should have gone and didn't.
    fn send<B: Board>(&mut self, board: &mut B, id: u16, data: &[u8]) -> bool;

    /// After each tick: what the spine decided, for a link with a status of its own
    /// to send (the UART's uplink). Nothing by default.
    fn report<B: Board>(&mut self, _board: &mut B, _report: &Report) {}

    /// After the spine's step, before the devices are driven: what the steering
    /// actuator gets. The spine's target by default; the UART link's version 3 frames
    /// can ask for the steering calibration (`legacy_link::SteerCal`).
    fn steering(
        &mut self,
        _inputs: &Inputs,
        _out: &Outputs,
        _positions: &Positions,
    ) -> SteeringDrive {
        SteeringDrive::Spine
    }
}

/// One tick, as the spine saw it: for a link's status frame.
#[derive(Debug, Clone, Copy)]
pub struct Report {
    pub now_ms: u32,
    pub outputs: Outputs,
    /// The RC this tick, or none (silent, or its mode switch out of range).
    pub rc: Option<RcInput>,
    /// Where the steering and brake motors are, if known.
    pub positions: Positions,
}

/// CAN, through the board's CAN controller: the frames of `firmware/can/aav.dbc`.
#[derive(Debug, Clone, Copy, Default)]
pub struct CanLink;

impl CommandLink for CanLink {
    fn receive<B: Board>(&mut self, board: &mut B, inputs: &mut Inputs) {
        for _ in 0..MAX_CAN_FRAMES {
            let Some(frame) = board.can_receive() else {
                break;
            };
            match decode_incoming(frame.id(), frame.data()) {
                // A disarm in any frame this tick wins, so an arming edge isn't lost.
                Ok(Incoming::Drive(cmd)) => {
                    if inputs.drive_cmd.is_none_or(|old| old.armed) {
                        inputs.drive_cmd = Some(cmd);
                    }
                }
                // An e-stop request in any frame this tick wins over a release.
                Ok(Incoming::Estop(e)) => {
                    if inputs.estop_cmd.is_none_or(|old| !old.estop_request) {
                        inputs.estop_cmd = Some(e);
                    }
                }
                Err(CanError::UnknownId) => {} // for someone else on the bus
                Err(_) => inputs.bad_frame = true,
            }
        }
    }

    fn send<B: Board>(&mut self, board: &mut B, id: u16, data: &[u8]) -> bool {
        CanFrame::new(id, data).is_some_and(|frame| board.can_send(&frame))
    }
}

/// Which frame the Motor-Pi sends (`legacy_uart_bridge`'s `frame_version`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameVersion {
    /// The 2025-26 13-byte frame: no counter, no flags, a sum checksum.
    V1,
    /// The sync-byte frames: version 2 (17 bytes with the Orin's heartbeat counter,
    /// armed and e-stop flags and a CRC-8, TODO 50) and version 3 (20 bytes: version 2
    /// and the steering calibration, 1 Nov), told apart by their version byte.
    V2,
}

/// What the link has seen since boot, for the log.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkStats {
    /// Frames that passed their checksum or CRC.
    pub frames: u32,
    /// Frames turned down: a checksum or CRC, a version, a flag. Each one is ignored.
    pub bad: u32,
    /// Bytes outside a frame (version 2): noise, or the rest of a bad frame.
    pub skipped: u32,
    /// Version 2 frames whose counter hadn't changed: they don't count as a command.
    pub repeats: u32,
    /// The last counter seen (version 2).
    pub counter: Option<u8>,
    /// Uplink frames sent, and writes that failed (with an uplink).
    pub uplinks: u32,
    pub uplink_errors: u32,
}

/// No uplink: a link that sends nothing back ([`LegacyLink::new`]).
#[derive(Debug, Clone, Copy, Default)]
pub struct NoUplink;

impl embedded_io::ErrorType for NoUplink {
    type Error = Infallible;
}

impl Write for NoUplink {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

enum Decoder {
    V1(Parser),
    V2 {
        parser: ParserV3,
        heartbeat: Heartbeat,
    },
}

/// The UART link from the Motor-Pi (`legacy_uart_bridge`).
///
/// Each frame becomes the Jrk and DAC targets the C++ set for it (`legacy-link`), then a
/// [`DirectCmd`] in spine units, through the calibration, so the devices get the same
/// targets. What the C++ did, and what this does:
///
/// | Frame | C++ | Here |
/// |---|---|---|
/// | accel below -0.05 | brake `\|accel\| * 3500`, DAC cut | the same brake, throttle 0; a stop (the arming edge's first half) |
/// | \|speed\| under 0.03 | DAC ramps to 0, brake released | throttle 0, brake released |
/// | otherwise | DAC ramps to `\|speed\| * 700`, direction from the sign | the same code and direction |
/// | a value outside -1..1 | turned down; DAC ramps to 0, the rest stays | the same: the last command with no throttle |
/// | a bad checksum | one byte dropped | the same; a bad-frame fault bit |
///
/// The direction only changes with throttle: a stop or an idle frame keeps the last one.
///
/// Version 2 adds the heartbeat (TODO 50). A frame is a command only when its counter
/// changed since the frame before ([`Heartbeat`]); one with the same counter is ignored,
/// so the spine's 100 ms watchdog times out when the Orin stops, however many frames the
/// Motor-Pi still sends: brake full, throttle 0, latched, fault bit 1 (`CMD_TIMEOUT`).
///
/// | Version 2 frame | Here |
/// |---|---|
/// | the counter changed | a command, as above |
/// | the counter didn't change | ignored: no command, the watchdog isn't fed |
/// | `armed` clear | a stop: throttle 0, the frame's brake |
/// | `estop` set | an e-stop at once, whatever the counter; released only by a frame with a new counter and `estop` clear |
/// | a bad CRC, version or flag | ignored; a bad-frame fault bit |
///
/// With an uplink (TODO 50, 48: [`LegacyLink::with_uplink`], TX1 on today's board), it
/// sends an [`Uplink`] status frame every 20 ms: the mode, the mode switch, the faults,
/// the last Orin counter taken (its echo), the Jrks' positions (in radians and 0..1, and
/// raw in counts), the RC sticks and the steering calibration's state.
///
/// Version 3 frames (1 Nov) can ask for the steering calibration: the steering actuator
/// limp, or a jog target in counts. Its rules are `legacy_link::SteerCal`'s; this link
/// gives it each fresh frame's request, and holds the throttle at 0 in any frame that
/// asks for a mode.
pub struct LegacyLink<U, T = NoUplink> {
    uart: U,
    tx: Option<T>,
    decoder: Decoder,
    cal: Calibration,
    last: Option<DirectCmd>,
    stats: LinkStats,
    uplink_due: Every,
    sequence: u8,
    steer_cal: SteerCal,
    /// The latest frame's e-stop flag.
    frame_estop: bool,
}

impl<U: Read + ReadReady> LegacyLink<U, NoUplink> {
    /// `uart`: the Motor-Pi's serial port, at [`legacy_link::BAUD`], 8N1. No uplink.
    pub fn new(uart: U, cal: Calibration, version: FrameVersion) -> Self {
        LegacyLink::build(uart, None, cal, version)
    }
}

impl<U: Read + ReadReady, T: Write> LegacyLink<U, T> {
    /// The same, and the uplink's frames on `tx`: the same port's TX, 115200 8N1.
    pub fn with_uplink(uart: U, tx: T, cal: Calibration, version: FrameVersion) -> Self {
        LegacyLink::build(uart, Some(tx), cal, version)
    }

    fn build(uart: U, tx: Option<T>, cal: Calibration, version: FrameVersion) -> Self {
        let decoder = match version {
            FrameVersion::V1 => Decoder::V1(Parser::new()),
            FrameVersion::V2 => Decoder::V2 {
                parser: ParserV3::new(),
                heartbeat: Heartbeat::new(),
            },
        };
        let limits = cal.steering;
        LegacyLink {
            uart,
            tx,
            decoder,
            cal,
            last: None,
            stats: LinkStats::default(),
            uplink_due: Every::new(UPLINK_PERIOD_MS),
            sequence: 0,
            steer_cal: SteerCal::new(limits.min.min(limits.max), limits.max.max(limits.min)),
            frame_estop: false,
        }
    }

    /// What the link has seen since boot.
    pub fn stats(&self) -> LinkStats {
        self.stats
    }

    /// The uplink frame for a tick: `sequence`, then what the report says.
    pub fn uplink_frame(&self, report: &Report, sequence: u8) -> Uplink {
        let out = &report.outputs;
        let mode = match out.mode {
            Mode::Manual => DriveMode::Stopped,
            Mode::Rc => DriveMode::RcDirect,
            Mode::Computer => DriveMode::Computer,
            // The 2025-26 uplink has no test mode: to the Motor-Pi the computer isn't driving.
            Mode::Test => DriveMode::Stopped,
        };
        let (switch, sticks) = match report.rc {
            None => (ModeSwitch::NoSignal, Sticks::default()),
            Some(rc) => (
                match rc.switch {
                    RcSwitch::Computer => ModeSwitch::Computer,
                    RcSwitch::ViaRos => ModeSwitch::RcViaRos,
                    RcSwitch::Direct => ModeSwitch::RcDirect,
                },
                Sticks {
                    throttle: rc.throttle,
                    steering: rc.steering,
                    brake: rc.brake,
                },
            ),
        };
        Uplink {
            sequence,
            echo: self.stats.counter,
            mode,
            switch,
            armed: out.armed,
            estop: out.estop_active,
            faults: out.faults,
            steering: report.positions.steering,
            brake_position: report.positions.brake,
            throttle_out: out.throttle,
            brake_out: out.brake,
            sticks,
            counts: Counts {
                steering: report.positions.steering_counts,
                brake: report.positions.brake_counts,
            },
            calibration: self.steer_cal.state(),
        }
    }

    /// The command for one frame's targets.
    fn command(&self, t: &Targets) -> DirectCmd {
        let last_reverse = self.last.is_some_and(|c| c.reverse);
        let (throttle, reverse, stop) = match t.throttle {
            Throttle::Cut => (0.0, last_reverse, true),
            Throttle::Idle => (0.0, last_reverse, false),
            Throttle::Drive { code, reverse } => (self.cal.throttle.fraction(code), reverse, false),
        };
        DirectCmd {
            throttle,
            reverse,
            brake: self.cal.brake.fraction(t.brake),
            steering_angle: self.cal.steering.angle(t.steering),
            stop,
        }
    }

    /// The command for a frame's values, or none. A value outside -1..1 is a bad frame:
    /// the C++'s "packet rejected", the throttle ramps to 0, the rest stays.
    fn command_for(&self, values: Frame, inputs: &mut Inputs) -> Option<DirectCmd> {
        match values.check() {
            Ok(values) => Some(self.command(&values.targets())),
            Err(_) => {
                inputs.bad_frame = true;
                self.last.map(|last| DirectCmd {
                    throttle: 0.0,
                    ..last
                })
            }
        }
    }

    /// The tick's command: a stop in any frame this tick wins, so an arming edge isn't lost.
    fn give(&mut self, cmd: DirectCmd, inputs: &mut Inputs) {
        self.last = Some(cmd);
        if inputs.direct_cmd.is_none_or(|old| !old.stop) {
            inputs.direct_cmd = Some(cmd);
        }
    }

    fn take_v1(&mut self, result: Result<Frame, FrameError>, inputs: &mut Inputs) {
        let values = match result {
            Ok(frame) => frame,
            // The parser checks the range itself: the values are gone.
            Err(FrameError::OutOfRange) => {
                self.stats.frames += 1;
                inputs.bad_frame = true;
                if let Some(last) = self.last {
                    self.give(
                        DirectCmd {
                            throttle: 0.0,
                            ..last
                        },
                        inputs,
                    );
                }
                return;
            }
            Err(_) => {
                self.stats.bad += 1;
                inputs.bad_frame = true;
                return;
            }
        };
        self.stats.frames += 1;
        if let Some(cmd) = self.command_for(values, inputs) {
            self.give(cmd, inputs);
        }
    }

    fn take_v2(&mut self, result: Result<FrameV3, FrameError>, inputs: &mut Inputs) {
        let frame = match result {
            Ok(frame) => frame,
            Err(FrameError::Sync) => {
                self.stats.skipped += 1;
                inputs.bad_frame = true;
                return;
            }
            Err(_) => {
                self.stats.bad += 1;
                inputs.bad_frame = true;
                return;
            }
        };
        self.stats.frames += 1;
        self.stats.counter = Some(frame.counter);
        self.frame_estop = frame.flags.estop;
        // An e-stop is always safe to obey: whatever the counter, and it wins in a tick.
        if frame.flags.estop {
            inputs.estop_cmd = Some(EstopCmd {
                estop_request: true,
                counter: frame.counter,
            });
        }
        let Decoder::V2 { heartbeat, .. } = &mut self.decoder else {
            return;
        };
        if !heartbeat.fresh(frame.counter) {
            self.stats.repeats += 1;
            return;
        }
        if !frame.flags.estop && inputs.estop_cmd.is_none() {
            inputs.estop_cmd = Some(EstopCmd {
                estop_request: false,
                counter: frame.counter,
            });
        }
        let values = frame.values;
        self.steer_cal.take(
            Ask {
                request: frame.calibration,
                speed_zero: values.speed.abs() < legacy_link::SPEED_DEADBAND,
                braked: values.accel < legacy_link::BRAKE_BELOW,
            },
            inputs.now_ms,
        );
        let Some(mut cmd) = self.command_for(values, inputs) else {
            return;
        };
        // Asking for the steering calibration, refused or not: no throttle.
        if !frame.flags.armed || frame.calibration != CalRequest::Off {
            cmd.throttle = 0.0;
            cmd.stop = true;
        }
        self.give(cmd, inputs);
    }
}

impl<U: Read + ReadReady, T: Write> CommandLink for LegacyLink<U, T> {
    fn receive<B: Board>(&mut self, _board: &mut B, inputs: &mut Inputs) {
        let mut buf = [0; 64];
        for _ in 0..MAX_UART_READS {
            if !matches!(self.uart.read_ready(), Ok(true)) {
                break;
            }
            match self.uart.read(&mut buf) {
                Ok(n) if n > 0 => {
                    for &byte in &buf[..n] {
                        match &mut self.decoder {
                            Decoder::V1(parser) => {
                                if let Some(result) = parser.push(byte) {
                                    self.take_v1(result, inputs);
                                }
                            }
                            Decoder::V2 { parser, .. } => {
                                if let Some(result) = parser.push(byte) {
                                    self.take_v2(result, inputs);
                                }
                            }
                        }
                    }
                }
                _ => break,
            }
        }
    }

    /// The CAN frames have no place on this link: nothing to send, and nothing lost.
    fn send<B: Board>(&mut self, _board: &mut B, _id: u16, _data: &[u8]) -> bool {
        true
    }

    /// The steering calibration's decision for this tick (`legacy_link::SteerCal`).
    fn steering(&mut self, inputs: &Inputs, out: &Outputs, positions: &Positions) -> SteeringDrive {
        self.steer_cal.step(&Situation {
            now_ms: inputs.now_ms,
            feedback: positions.steering_counts,
            spine_target: self.cal.steering.target(out.steering_target),
            stopped: out.throttle == 0.0 && out.brake > 0.0,
            estop: inputs.hw_estop || self.frame_estop,
            other_driver: matches!(out.mode, Mode::Rc | Mode::Test),
        })
    }

    /// The uplink frame, every 20 ms, if this link has an uplink.
    fn report<B: Board>(&mut self, _board: &mut B, report: &Report) {
        if self.tx.is_none() || !self.uplink_due.due(report.now_ms) {
            return;
        }
        let bytes = self.uplink_frame(report, self.sequence).encode();
        let Some(tx) = self.tx.as_mut() else {
            return;
        };
        if tx.write_all(&bytes).is_ok() {
            self.stats.uplinks = self.stats.uplinks.wrapping_add(1);
            self.sequence = self.sequence.wrapping_add(1);
        } else {
            self.stats.uplink_errors = self.stats.uplink_errors.wrapping_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::rig::{FakeBoard, FakeTx, FakeUart};
    use legacy_link::Frame;

    fn frame(steering: f32, speed: f32, accel: f32) -> [u8; 13] {
        Frame {
            steering,
            speed,
            accel,
        }
        .encode()
    }

    fn receive<T: embedded_io::Write>(link: &mut LegacyLink<FakeUart, T>, bytes: &[u8]) -> Inputs {
        link.uart.0.borrow_mut().extend(bytes.iter().copied());
        let mut inputs = Inputs::default();
        link.receive(&mut FakeBoard::new(), &mut inputs);
        inputs
    }

    fn link() -> LegacyLink<FakeUart> {
        LegacyLink::new(FakeUart::default(), Calibration::TODAY, FrameVersion::V1)
    }

    /// The devices' targets for a command, through today's calibration.
    fn targets(cmd: &DirectCmd) -> (u16, u16, u16) {
        let cal = Calibration::TODAY;
        (
            cal.steering.target(cmd.steering_angle),
            cal.brake.target(cmd.brake),
            cal.throttle.code(cmd.throttle),
        )
    }

    #[test]
    fn a_frame_gives_the_cpp_targets_back() {
        let mut l = link();
        let cases = [
            ((0.5, 0.5, 0.0), (2725, 400, 350)),
            ((-0.5, -1.0, 0.0), (975, 400, 700)),
            ((-1.0, 0.0, -1.0), (400, 3500, 0)),
            ((0.123, 0.0, -0.3), (2140, 1050, 0)),
        ];
        for ((s, v, a), want) in cases {
            let cmd = receive(&mut l, &frame(s, v, a)).direct_cmd.unwrap();
            assert_eq!(targets(&cmd), want, "{s} {v} {a}");
        }
    }

    #[test]
    fn a_brake_frame_is_a_stop_and_keeps_the_direction() {
        let mut l = link();
        let cmd = receive(&mut l, &frame(0.0, -0.5, 0.0)).direct_cmd.unwrap();
        assert!(cmd.reverse && !cmd.stop);
        let cmd = receive(&mut l, &frame(0.0, 0.5, -1.0)).direct_cmd.unwrap();
        assert!(cmd.stop && cmd.reverse && cmd.throttle == 0.0);
        let cmd = receive(&mut l, &frame(0.0, 0.0, 0.0)).direct_cmd.unwrap();
        assert!(!cmd.stop && cmd.reverse && cmd.throttle == 0.0); // idle
    }

    #[test]
    fn a_stop_wins_within_a_tick() {
        let mut l = link();
        let both = [frame(0.0, 0.0, -1.0), frame(0.0, 0.5, 0.0)].concat();
        assert!(receive(&mut l, &both).direct_cmd.unwrap().stop);
        let both = [frame(0.0, 0.5, 0.0), frame(0.0, 0.4, 0.0)].concat();
        let cmd = receive(&mut l, &both).direct_cmd.unwrap();
        assert_eq!(targets(&cmd).2, 280); // the latest
    }

    #[test]
    fn a_frame_out_of_range_cuts_the_throttle_and_keeps_the_rest() {
        let mut l = link();
        receive(&mut l, &frame(0.5, 0.5, 0.0));
        let mut bad = [0u8; 13];
        bad[..4].copy_from_slice(&2.0f32.to_le_bytes());
        bad[12] = bad[..12].iter().fold(0u8, |s, &b| s.wrapping_add(b));
        let inputs = receive(&mut l, &bad);
        assert!(inputs.bad_frame);
        assert_eq!(targets(&inputs.direct_cmd.unwrap()), (2725, 400, 0));
        // With nothing before it, nothing.
        assert_eq!(receive(&mut link(), &bad).direct_cmd, None);
    }

    #[test]
    fn noise_is_a_bad_frame_and_the_next_frame_still_comes() {
        let mut l = link();
        let noisy = [&[0x01, 0x02][..], &frame(0.0, 0.25, 0.0)].concat();
        let inputs = receive(&mut l, &noisy);
        assert!(inputs.bad_frame);
        assert_eq!(targets(&inputs.direct_cmd.unwrap()).2, 175);
    }

    #[test]
    fn nothing_goes_back() {
        assert!(link().send(&mut FakeBoard::new(), 0x181, &[0; 8]));
        // Without an uplink a report sends nothing either.
        let mut l = link();
        l.report(&mut FakeBoard::new(), &report(0, None));
        assert_eq!(l.stats().uplinks, 0);
    }

    // --- The uplink (TODO 50, 48) ---

    use crate::actuators::Positions;
    use legacy_link::{DriveMode, FrameV2, ModeSwitch, Uplink, UPLINK_LEN};
    use spine_core::protocol::Mode;
    use spine_core::{Outputs, RcInput, RcSwitch};
    use std::vec::Vec;

    fn report(now_ms: u32, rc: Option<RcInput>) -> Report {
        Report {
            now_ms,
            outputs: Outputs {
                mode: Mode::Computer,
                throttle: 0.25,
                brake: 0.0,
                reverse: false,
                steering_target: 0.1,
                armed: true,
                estop_active: false,
                faults: 0x0100,
            },
            rc,
            positions: Positions {
                steering: Some(0.1),
                brake: None,
                steering_counts: Some(2260),
                brake_counts: None,
            },
        }
    }

    fn frames(tx: &FakeTx) -> Vec<Uplink> {
        tx.0.borrow()
            .chunks(UPLINK_LEN)
            .map(|c| Uplink::decode(c).unwrap())
            .collect()
    }

    #[test]
    fn the_uplink_sends_a_status_every_20_ms() {
        let tx = FakeTx::default();
        let mut l = LegacyLink::with_uplink(
            FakeUart::default(),
            tx.clone(),
            Calibration::TODAY,
            FrameVersion::V2,
        );
        let mut board = FakeBoard::new();
        // A tick every 10 ms for 100 ms: 5 frames, numbered.
        for t in (0..100).step_by(10) {
            l.report(&mut board, &report(t, None));
        }
        let sent = frames(&tx);
        assert_eq!(sent.len(), 5);
        assert_eq!(l.stats().uplinks, 5);
        let seq: Vec<u8> = sent.iter().map(|u| u.sequence).collect();
        assert_eq!(seq, [0, 1, 2, 3, 4]);
        let u = sent[0];
        assert_eq!(
            (u.mode, u.switch, u.armed, u.faults),
            (DriveMode::Computer, ModeSwitch::NoSignal, true, 0x0100)
        );
        assert_eq!(
            (u.steering, u.brake_position, u.echo),
            (Some(0.1), None, None)
        );
        assert_eq!((u.throttle_out, u.brake_out), (0.25, 0.0));
    }

    #[test]
    fn the_uplink_echoes_the_last_orin_counter_and_the_sticks() {
        let tx = FakeTx::default();
        let mut l = LegacyLink::with_uplink(
            FakeUart::default(),
            tx.clone(),
            Calibration::TODAY,
            FrameVersion::V2,
        );
        receive(&mut l, &v2(0.0, 0.0, 0.0, 41, true));
        receive(&mut l, &v2(0.0, 0.0, 0.0, 42, true));
        let rc = RcInput {
            switch: RcSwitch::ViaRos,
            throttle: 0.5,
            steering: -0.25,
            brake: false,
        };
        l.report(&mut FakeBoard::new(), &report(0, Some(rc)));
        let u = frames(&tx)[0];
        assert_eq!(u.echo, Some(42));
        assert_eq!(u.switch, ModeSwitch::RcViaRos);
        assert_eq!((u.sticks.throttle, u.sticks.steering), (0.5, -0.25));
        // Version 1 frames carry no counter: nothing to echo.
        let tx = FakeTx::default();
        let mut l = LegacyLink::with_uplink(
            FakeUart::default(),
            tx.clone(),
            Calibration::TODAY,
            FrameVersion::V1,
        );
        receive(&mut l, &frame(0.0, 0.0, -1.0));
        l.report(&mut FakeBoard::new(), &report(0, None));
        assert_eq!(frames(&tx)[0].echo, None);
    }

    // --- Version 2: the heartbeat (TODO 50) ---

    fn v2(steering: f32, speed: f32, accel: f32, counter: u8, armed: bool) -> [u8; 17] {
        v2_flags(steering, speed, accel, counter, armed, false)
    }

    fn v2_flags(
        steering: f32,
        speed: f32,
        accel: f32,
        counter: u8,
        armed: bool,
        estop: bool,
    ) -> [u8; 17] {
        FrameV2 {
            values: Frame {
                steering,
                speed,
                accel,
            },
            counter,
            flags: legacy_link::Flags { armed, estop },
        }
        .encode()
    }

    fn link_v2() -> LegacyLink<FakeUart> {
        LegacyLink::new(FakeUart::default(), Calibration::TODAY, FrameVersion::V2)
    }

    #[test]
    fn v2_a_frame_counts_only_when_its_counter_changed() {
        let mut l = link_v2();
        // The first frame proves nothing: it only sets the counter.
        assert_eq!(
            receive(&mut l, &v2(0.5, 0.5, 0.0, 10, true)).direct_cmd,
            None
        );
        let cmd = receive(&mut l, &v2(0.5, 0.5, 0.0, 11, true))
            .direct_cmd
            .unwrap();
        assert_eq!(targets(&cmd), (2725, 400, 350));
        // The same counter again: a stuck Pi, or an Orin gone quiet. Ignored.
        for _ in 0..5 {
            assert_eq!(
                receive(&mut l, &v2(0.5, 0.5, 0.0, 11, true)).direct_cmd,
                None
            );
        }
        assert!(receive(&mut l, &v2(0.5, 0.5, 0.0, 12, true))
            .direct_cmd
            .is_some());
        let s = l.stats();
        assert_eq!((s.frames, s.repeats, s.bad, s.counter), (8, 6, 0, Some(12)));
    }

    #[test]
    fn v2_not_armed_is_a_stop() {
        let mut l = link_v2();
        receive(&mut l, &v2(0.0, 0.0, 0.0, 1, true));
        let cmd = receive(&mut l, &v2(0.4, 0.8, -1.0, 2, false))
            .direct_cmd
            .unwrap();
        assert!(cmd.stop && cmd.throttle == 0.0);
        assert_eq!(targets(&cmd).1, 3500);
        // Even with throttle in the values.
        let cmd = receive(&mut l, &v2(0.0, 0.8, 0.0, 3, false))
            .direct_cmd
            .unwrap();
        assert!(cmd.stop && cmd.throttle == 0.0);
    }

    #[test]
    fn v2_the_estop_flag_works_whatever_the_counter() {
        let mut l = link_v2();
        receive(&mut l, &v2(0.0, 0.0, 0.0, 1, true));
        receive(&mut l, &v2(0.0, 0.5, 0.0, 2, true));
        // The same counter: no command, but the e-stop still comes through.
        let inputs = receive(&mut l, &v2_flags(0.0, 0.0, -1.0, 2, false, true));
        assert_eq!(inputs.direct_cmd, None);
        assert!(inputs.estop_cmd.unwrap().estop_request);
        // A release needs a new counter.
        assert_eq!(
            receive(&mut l, &v2(0.0, 0.0, -1.0, 2, false)).estop_cmd,
            None
        );
        let inputs = receive(&mut l, &v2(0.0, 0.0, -1.0, 3, false));
        assert!(!inputs.estop_cmd.unwrap().estop_request);
        // A request and a release in one tick: the request wins.
        let both = [
            v2_flags(0.0, 0.0, -1.0, 4, false, true),
            v2(0.0, 0.0, -1.0, 5, false),
        ];
        assert!(
            receive(&mut l, &both.concat())
                .estop_cmd
                .unwrap()
                .estop_request
        );
    }

    #[test]
    fn v2_a_bad_crc_is_ignored_and_counted() {
        let mut l = link_v2();
        receive(&mut l, &v2(0.0, 0.0, 0.0, 1, true));
        let mut bad = v2(0.0, 0.9, 0.0, 2, true);
        bad[16] ^= 0xFF;
        let inputs = receive(&mut l, &bad);
        assert!(inputs.bad_frame);
        assert_eq!(inputs.direct_cmd, None);
        assert_eq!(l.stats().bad, 1);
        // The next good frame, counter 3, still comes.
        assert!(receive(&mut l, &v2(0.0, 0.25, 0.0, 3, true))
            .direct_cmd
            .is_some());
    }

    #[test]
    fn v2_ignores_version_1_frames() {
        let mut l = link_v2();
        for i in 0..20 {
            let inputs = receive(&mut l, &frame(0.0, 0.5 + i as f32 / 100.0, 0.0));
            assert_eq!(inputs.direct_cmd, None);
        }
        assert_eq!(l.stats().frames, 0);
    }

    #[test]
    fn v2_out_of_range_cuts_the_throttle() {
        let mut l = link_v2();
        receive(&mut l, &v2(0.0, 0.0, 0.0, 1, true));
        receive(&mut l, &v2(0.5, 0.5, 0.0, 2, true));
        let inputs = receive(&mut l, &v2(2.0, 0.5, 0.0, 3, true));
        assert!(inputs.bad_frame);
        assert_eq!(targets(&inputs.direct_cmd.unwrap()), (2725, 400, 0));
    }
}

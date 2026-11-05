//! The loop: read everything, step the spine, drive the devices, send CAN frames.
//!
//! One tick, about 100 times a second:
//!
//! | Step | Reads | Fills, in `Inputs` |
//! |---|---|---|
//! | 1 | the computer's commands, from the link: CAN, or the UART ([`CommandLink`]) | `drive_cmd` or `direct_cmd`, `estop_cmd`, `bad_frame` |
//! | 2 | the RC receiver's serial port | `rc` |
//! | 3 | the hardware e-stop, debounced | `hw_estop` |
//! | 4 | the wheel encoder's counter | `measured_speed` |
//! | 5 | the steering and brake Jrks | `measured_steering`; `driver_faults`, with last tick's write errors |
//!
//! Then `Spine::step`, then the link's say on the steering (`CommandLink::steering`: the
//! UART link's steering calibration), then `Actuators::apply_with`, then VEHICLE_FEEDBACK every 20 ms
//! and VEHICLE_STATUS every 100 ms, through the link (the UART link drops them), then
//! the tick's [`Report`] to the link (the UART link's uplink frame, every 20 ms).

use crate::actuators::{Actuators, Devices};
use crate::board::Board;
use crate::calibration::Calibration;
use crate::link::{CanLink, CommandLink, Report};
use crate::wheel::WheelSpeed;
use aav_inputs::Debouncer;
use embedded_hal::i2c::I2c;
use embedded_io::{Read, ReadReady};
use ibus::{RcMap, Receiver};
use spine_core::protocol::{VEHICLE_FEEDBACK_ID, VEHICLE_STATUS_ID};
use spine_core::{faults, Inputs, Outputs, Spine, SpineConfig};

/// VEHICLE_FEEDBACK at 50 Hz, and VEHICLE_STATUS at 10 Hz (`docs/firmware.md`).
pub const FEEDBACK_PERIOD_MS: u32 = 20;
pub const STATUS_PERIOD_MS: u32 = 100;

/// A limit on one tick's work, so a flood of bytes can't stall the loop.
const MAX_UART_READS: usize = 16;

/// The whole spine on a board: give it the board, the four devices and the RC
/// receiver's serial port, then call [`SpineBoard::tick`] about 100 times a second.
///
/// The computer's commands come through `L`: CAN by default ([`SpineBoard::new`]), or
/// another [`CommandLink`] ([`SpineBoard::with_link`]).
pub struct SpineBoard<B, I2C, U, L = CanLink> {
    board: B,
    uart: U,
    link: L,
    actuators: Actuators<I2C>,
    spine: Spine,
    rc: Receiver,
    rc_map: RcMap,
    estop: Debouncer,
    wheel: WheelSpeed,
    /// Errors from last tick's writes, and from sending CAN.
    write_faults: u16,
    can_fault: u16,
    feedback: Every,
    status: Every,
}

impl<B: Board, I2C: I2c, U: Read + ReadReady> SpineBoard<B, I2C, U, CanLink> {
    /// The spine, with the computer on CAN, through the board.
    ///
    /// `uart`: the RC receiver's serial port, set to 115200 baud, 8N1.
    pub fn new(
        board: B,
        devices: Devices<I2C>,
        uart: U,
        cal: Calibration,
        config: SpineConfig,
    ) -> Self {
        Self::with_link(board, devices, uart, CanLink, cal, config)
    }
}

impl<B: Board, I2C: I2c, U: Read + ReadReady, L: CommandLink> SpineBoard<B, I2C, U, L> {
    /// The spine, with the computer's commands through `link`.
    ///
    /// `uart`: the RC receiver's serial port, set to 115200 baud, 8N1.
    pub fn with_link(
        board: B,
        devices: Devices<I2C>,
        uart: U,
        link: L,
        cal: Calibration,
        config: SpineConfig,
    ) -> Self {
        SpineBoard {
            board,
            uart,
            link,
            actuators: Actuators::new(devices, cal),
            spine: Spine::new(config),
            rc: Receiver::new(cal.rc_timeout_ms),
            rc_map: cal.rc,
            // Starts tripped, so a reboot is safe. Trips at once; releases only
            // after reading closed for `estop_release_ms`.
            estop: Debouncer::with_times(true, 0, cal.estop_release_ms),
            wheel: WheelSpeed::new(cal.wheel.metres_per_count, cal.wheel.timeout_ms),
            write_faults: 0,
            can_fault: 0,
            feedback: Every::new(FEEDBACK_PERIOD_MS),
            status: Every::new(STATUS_PERIOD_MS),
        }
    }

    /// Power-up: throttle 0, brake on, expander set up, steering centred. Call once
    /// before the first tick. Returns the fault bits; they also go into the first
    /// tick, so a device that failed here keeps the car stopped until it is armed.
    pub fn init(&mut self) -> u16 {
        self.write_faults = self.actuators.init();
        self.write_faults
    }

    /// One pass of the loop. Returns what the spine decided.
    pub fn tick(&mut self) -> Outputs {
        let now = self.board.now_ms();
        let mut inputs = Inputs {
            now_ms: now,
            ..Inputs::default()
        };
        self.link.receive(&mut self.board, &mut inputs);
        self.read_rc(now);
        inputs.rc = self.rc.channels(now).and_then(|c| self.rc_map.rc_input(&c));
        let open = self.board.estop_open();
        self.estop.update(open, now);
        inputs.hw_estop = self.estop.level();
        let wheel = self.board.wheel();
        inputs.measured_speed = self.wheel.update(wheel, now);
        let readings = self.actuators.read();
        inputs.measured_steering = readings.steering;
        inputs.driver_faults = readings.faults | self.write_faults | self.can_fault;

        let out = self.spine.step(&inputs);
        let steering = self.link.steering(&inputs, &out, &readings.positions);
        self.write_faults = self.actuators.apply_with(&out, now, steering);
        self.send_frames(now, &inputs, wheel.count, &out);
        let report = Report {
            now_ms: now,
            outputs: out,
            rc: inputs.rc,
            positions: readings.positions,
        };
        self.link.report(&mut self.board, &report);
        out
    }

    /// The spine's five state regions after the latest tick (`spine_core::fsm`).
    pub fn state(&self) -> spine_core::State {
        self.spine.state()
    }

    /// The board, for whatever else it does (LEDs, say).
    pub fn board(&mut self) -> &mut B {
        &mut self.board
    }

    /// The command link, to read what it has seen (its stats, say).
    pub fn link(&mut self) -> &mut L {
        &mut self.link
    }

    /// The devices and their state, to read something else from them.
    pub fn actuators(&mut self) -> &mut Actuators<I2C> {
        &mut self.actuators
    }

    /// Feed the iBUS parser whatever the serial port has. A UART error loses those
    /// bytes; the parser resyncs.
    fn read_rc(&mut self, now: u32) {
        let mut buf = [0; 32];
        for _ in 0..MAX_UART_READS {
            if !matches!(self.uart.read_ready(), Ok(true)) {
                break;
            }
            match self.uart.read(&mut buf) {
                Ok(n) if n > 0 => {
                    for &byte in &buf[..n] {
                        self.rc.push(byte, now);
                    }
                }
                _ => break,
            }
        }
    }

    fn send_frames(&mut self, now: u32, inputs: &Inputs, wheel_count: i32, out: &Outputs) {
        let mut sent: Option<bool> = None;
        if self.feedback.due(now) {
            let data = self.spine.feedback_frame(
                inputs.measured_speed,
                inputs.measured_steering,
                wheel_count as u16, // the frame carries the low 16 bits
            );
            sent = Some(self.send(VEHICLE_FEEDBACK_ID, &data));
        }
        if self.status.due(now) {
            let data = self.spine.status_frame(out);
            let ok = self.send(VEHICLE_STATUS_ID, &data);
            sent = Some(sent.unwrap_or(true) && ok);
        }
        if let Some(ok) = sent {
            self.can_fault = if ok { 0 } else { faults::CAN_BUS };
        }
    }

    fn send(&mut self, id: u16, data: &[u8]) -> bool {
        self.link.send(&mut self.board, id, data)
    }
}

/// A job due every `period_ms`, on average.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Every {
    period_ms: u32,
    next_ms: Option<u32>,
}

impl Every {
    pub(crate) const fn new(period_ms: u32) -> Self {
        Every {
            period_ms,
            next_ms: None,
        }
    }

    /// True if the job is due at `now_ms`; then the next one is booked.
    pub(crate) fn due(&mut self, now_ms: u32) -> bool {
        let next = *self.next_ms.get_or_insert(now_ms);
        if (now_ms.wrapping_sub(next) as i32) < 0 {
            return false;
        }
        let after = next.wrapping_add(self.period_ms);
        // More than a period late: start again from now, rather than catch up.
        let late = now_ms.wrapping_sub(after) as i32 >= 0;
        self.next_ms = Some(if late {
            now_ms.wrapping_add(self.period_ms)
        } else {
            after
        });
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_keeps_its_average() {
        let mut e = Every::new(20);
        let due: usize = (0..100).filter(|i| e.due(i * 10)).count();
        assert_eq!(due, 50); // every other 10 ms tick
        let mut e = Every::new(20);
        // Ticks 11 ms apart still give one every 20 ms on average.
        let due: usize = (0..200).filter(|i| e.due(i * 11)).count();
        assert!((109..=111).contains(&due), "{due}");
    }

    #[test]
    fn every_restarts_after_a_long_gap() {
        let mut e = Every::new(20);
        assert!(e.due(0));
        assert!(e.due(1000)); // late: once, not 50 times
        assert!(!e.due(1010));
        assert!(e.due(1020));
    }

    #[test]
    fn every_survives_clock_wrap() {
        let mut e = Every::new(20);
        assert!(e.due(u32::MAX - 5));
        assert!(!e.due(u32::MAX));
        assert!(e.due(14)); // 20 ms after the first
    }
}

/// The whole loop, end to end, on the test rig.
#[cfg(test)]
mod loop_tests {
    extern crate std;
    use crate::rig::{Rig, BRAKE, DAC, EXPANDER, STEER};
    use jrk_g2::command::set_target;
    use jrk_g2::ErrorFlags;
    use spine_core::faults;
    use spine_core::protocol::{
        Mode, VehicleFeedback, VehicleStatus, DRIVE_CMD_ID, VEHICLE_FEEDBACK_ID, VEHICLE_STATUS_ID,
    };
    use std::vec;

    #[test]
    fn init_cuts_the_throttle_then_brakes_then_sets_up() {
        let w = Rig::new().writes();
        assert_eq!(w[0], (DAC, vec![0x00, 0x00]));
        assert_eq!(w[1], (BRAKE, set_target(3500).to_vec()));
        assert_eq!(w[2], (EXPANDER, vec![0x04, 0x00, 0x00]));
        assert_eq!(w[3], (EXPANDER, vec![0x02, 0xFF, 0xFF]));
        assert_eq!(w[4], (EXPANDER, vec![0x06, 0x00, 0x00]));
        assert_eq!(w[5], (STEER, set_target(1950).to_vec()));
    }

    #[test]
    fn armed_frames_after_a_reboot_do_not_drive() {
        let mut rig = Rig::new();
        let out = rig.drive_for(1.0, true, 30);
        assert_eq!(out.mode, Mode::Manual);
        assert!(rig.dac_codes().iter().all(|&c| c == 0));
    }

    #[test]
    fn once_armed_the_computer_drives_and_the_throttle_ramps() {
        let mut rig = Rig::new();
        rig.arm();
        rig.clear();
        let out = rig.drive_for(1.0, true, 30);
        assert_eq!(out.mode, Mode::Computer);
        let codes = rig.dac_codes();
        assert!(*codes.last().unwrap() > 0);
        assert!(codes.windows(2).all(|w| w[1] <= w[0] + 6), "{codes:?}");
    }

    /// Arm, drive until the throttle is on, and forget the writes so far.
    fn driving() -> Rig {
        let mut rig = Rig::new();
        rig.arm();
        rig.drive_for(1.0, true, 30);
        assert!(rig.sb.actuators().throttle_code() > 0);
        rig.clear();
        rig
    }

    #[test]
    fn a_hardware_estop_cuts_the_throttle_before_the_brake() {
        let mut rig = driving();
        rig.sb.board().estop_open = true;
        rig.drive(1.0, true);
        let out = rig.tick();
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        let throttle_off = rig.position(DAC, 0).unwrap();
        let brake_on = rig.position(BRAKE, 3500).unwrap();
        assert!(throttle_off < brake_on);
    }

    #[test]
    fn an_estop_frame_does_the_same() {
        let mut rig = driving();
        rig.drive(1.0, true);
        rig.estop_frame(true);
        let out = rig.tick();
        assert!(out.estop_active);
        assert!(rig.position(DAC, 0).unwrap() < rig.position(BRAKE, 3500).unwrap());
    }

    #[test]
    fn an_estop_request_wins_over_a_release_in_the_same_tick() {
        let mut rig = driving();
        rig.drive(1.0, true);
        rig.estop_frame(true);
        rig.estop_frame(false);
        assert_eq!(rig.tick().brake, 1.0);
    }

    #[test]
    fn a_disarm_and_an_arm_in_one_tick_still_arm() {
        let mut rig = Rig::new();
        rig.ticks(6); // the e-stop input releases
        rig.drive(0.0, false);
        rig.drive(1.0, true);
        rig.tick();
        assert_eq!(rig.drive_for(1.0, true, 3).mode, Mode::Computer);
    }

    #[test]
    fn rc_drives_without_the_computer() {
        let mut rig = Rig::new();
        rig.ticks(6);
        let mut out = rig.tick();
        for _ in 0..30 {
            rig.rc(true, 1000, 1500, 1000); // override on, full throttle, forward
            out = rig.tick();
        }
        assert_eq!(out.mode, Mode::Rc);
        assert!(out.throttle > 0.0);
        assert!(rig.sb.actuators().throttle_code() > 0);
    }

    #[test]
    fn a_silent_receiver_stops_rc() {
        let mut rig = Rig::new();
        rig.ticks(6);
        for _ in 0..30 {
            rig.rc(true, 1000, 1500, 1000);
            rig.tick();
        }
        let out = rig.ticks(15); // 150 ms without a frame
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::RC_SIGNAL_LOST, 0);
        assert_eq!(rig.sb.actuators().throttle_code(), 0);
    }

    #[test]
    fn feedback_every_20_ms_and_status_every_100_ms() {
        let mut rig = Rig::new();
        rig.sb.board().wheel.count = 70_000;
        rig.bus.borrow_mut().steering_feedback = 2260; // 0.1 rad left
        rig.ticks(100); // 1 s
        let sent = &rig.sb.board().sent;
        let feedback: std::vec::Vec<_> = sent
            .iter()
            .filter(|f| f.id() == VEHICLE_FEEDBACK_ID)
            .collect();
        let status = sent.iter().filter(|f| f.id() == VEHICLE_STATUS_ID).count();
        assert_eq!((feedback.len(), status), (50, 10));
        let last = VehicleFeedback::decode(feedback[49].data()).unwrap();
        assert_eq!(last.wheel_ticks, 70_000_u32 as u16);
        assert!((last.measured_steering_angle - 0.1).abs() < 0.01);
        let status = sent.iter().rfind(|f| f.id() == VEHICLE_STATUS_ID).unwrap();
        assert!(VehicleStatus::decode(status.data()).is_ok());
    }

    #[test]
    fn a_failed_send_is_a_can_bus_fault() {
        let mut rig = Rig::new();
        rig.sb.board().can_ok = false;
        rig.tick();
        assert_ne!(rig.tick().faults & faults::CAN_BUS, 0);
        rig.sb.board().can_ok = true;
        rig.ticks(3); // the next sends work
        assert_eq!(rig.tick().faults & faults::CAN_BUS, 0);
    }

    #[test]
    fn a_dead_dac_stops_the_car() {
        let mut rig = driving();
        rig.bus.borrow_mut().failing.push(DAC);
        rig.drive(1.0, true);
        rig.tick(); // the write fails
        rig.drive(1.0, true);
        let out = rig.tick();
        assert_ne!(out.faults & faults::THROTTLE_DAC, 0);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
    }

    #[test]
    fn a_jrk_error_stops_the_car() {
        let mut rig = driving();
        rig.bus
            .borrow_mut()
            .jrk_errors
            .push((BRAKE, ErrorFlags::NO_POWER));
        rig.drive(1.0, true);
        let out = rig.tick();
        assert_ne!(out.faults & faults::BRAKE_DRIVER, 0);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
    }

    #[test]
    fn bad_frames_are_reported_and_others_frames_ignored() {
        let mut rig = Rig::new();
        rig.can(0x300, &[1, 2, 3]); // someone else's frame
        assert_eq!(rig.tick().faults & faults::BAD_CHECKSUM, 0);
        rig.can(DRIVE_CMD_ID, &[0; 8]); // the CRC is wrong
        assert_ne!(rig.tick().faults & faults::BAD_CHECKSUM, 0);
    }
}

/// The loop with the 2025-26 UART link instead of CAN, and no wheel speed: today's ESP32
/// control board, as `boards/esp32-umod` builds it.
#[cfg(test)]
mod legacy_loop_tests {
    extern crate std;
    use super::SpineBoard;
    use crate::actuators::Devices;
    use crate::calibration::Calibration;
    use crate::link::{FrameVersion, LegacyLink};
    use crate::rig::{BusState, FakeBoard, FakeBus, FakeTx, FakeUart, BRAKE, DAC, STEER};
    use core::cell::RefCell;
    use jrk_g2::command::set_target;
    use legacy_link::{Flags, Frame, FrameV2};
    use spine_core::protocol::Mode;
    use spine_core::{faults, Outputs, SpeedFeedback, SpineConfig};
    use std::rc::Rc;
    use std::vec::Vec;

    struct LegacyRig {
        sb: SpineBoard<FakeBoard, FakeBus, FakeUart, LegacyLink<FakeUart, FakeTx>>,
        bus: Rc<RefCell<BusState>>,
        motor_pi: FakeUart,
        receiver: FakeUart,
    }

    impl LegacyRig {
        fn new() -> Self {
            Self::with_version(FrameVersion::V1)
        }

        fn with_version(version: FrameVersion) -> Self {
            let bus = FakeBus::default();
            let (motor_pi, receiver) = (FakeUart::default(), FakeUart::default());
            let cal = Calibration::TODAY;
            let devices = Devices::new(bus.clone(), bus.clone(), bus.clone(), bus.clone(), &cal);
            let config = SpineConfig {
                speed_feedback: SpeedFeedback::None {
                    full_throttle_speed: 3.0,
                },
                ..SpineConfig::default()
            };
            // With an uplink, as the board builds it with `uplink`: the frames down are the same.
            let link = LegacyLink::with_uplink(motor_pi.clone(), FakeTx::default(), cal, version);
            let board = FakeBoard::new();
            let mut sb = SpineBoard::with_link(board, devices, receiver.clone(), link, cal, config);
            sb.init();
            LegacyRig {
                sb,
                bus: bus.0,
                motor_pi,
                receiver,
            }
        }

        /// One frame from the Motor-Pi, then a 20 ms tick: the bridge's 50 Hz.
        fn frame(&mut self, steering: f32, speed: f32, accel: f32) -> Outputs {
            let bytes = Frame {
                steering,
                speed,
                accel,
            }
            .encode();
            self.motor_pi.0.borrow_mut().extend(bytes);
            self.sb.board().now += 20;
            self.sb.tick()
        }

        /// One version 2 frame, then a 20 ms tick.
        fn frame_v2(&mut self, speed: f32, accel: f32, counter: u8, armed: bool) -> Outputs {
            let bytes = FrameV2 {
                values: Frame {
                    steering: 0.0,
                    speed,
                    accel,
                },
                counter,
                flags: Flags {
                    armed,
                    estop: false,
                },
            }
            .encode();
            self.motor_pi.0.borrow_mut().extend(bytes);
            self.sb.board().now += 20;
            self.sb.tick()
        }

        /// `n` version 2 frames, the counter moving by one each (from `*counter`).
        fn moving(&mut self, n: usize, speed: f32, accel: f32, counter: &mut u8) -> Outputs {
            (0..n)
                .map(|_| {
                    *counter = counter.wrapping_add(1);
                    self.frame_v2(speed, accel, *counter, true)
                })
                .last()
                .unwrap()
        }

        fn frames(&mut self, n: usize, steering: f32, speed: f32, accel: f32) -> Outputs {
            (0..n)
                .map(|_| self.frame(steering, speed, accel))
                .last()
                .unwrap()
        }

        /// Stop frames past the e-stop input's release (50 ms after boot): the arming
        /// edge's first half. The bridge sends them whenever ego is stale.
        fn arm(&mut self) {
            self.frames(5, 0.0, 0.0, -1.0);
        }

        /// The latest bytes written to a device.
        fn last(&self, address: u8) -> Option<Vec<u8>> {
            let state = self.bus.borrow();
            let latest = state.writes.iter().rev().find(|w| w.0 == address);
            latest.map(|w| w.1.clone())
        }

        fn dac(&self) -> u16 {
            self.last(DAC)
                .map_or(0, |w| ad5311::decode([w[0], w[1]]).code)
        }
    }

    #[test]
    fn drive_frames_after_a_reboot_do_not_drive_until_a_stop_frame() {
        let mut rig = LegacyRig::new();
        let out = rig.frames(10, 0.0, 0.5, 0.0);
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(rig.dac(), 0);
        // The bridge sends stop frames whenever ego is stale, so this comes first anyway.
        rig.arm();
        assert_eq!(rig.last(BRAKE), Some(set_target(3500).to_vec()));
        let out = rig.frames(100, 0.0, 0.5, 0.0);
        assert_eq!(out.mode, Mode::Computer);
        // The C++'s code for speed 0.5, reached through the ramp; the brake let go.
        assert_eq!(rig.dac(), 350);
        assert_eq!(rig.last(BRAKE), Some(set_target(400).to_vec()));
    }

    #[test]
    fn the_steering_reaches_the_cpp_target() {
        let mut rig = LegacyRig::new();
        rig.arm();
        rig.frames(200, 0.5, 0.0, 0.0); // 2725: the C++'s target for +0.5
        assert_eq!(rig.last(STEER), Some(set_target(2725).to_vec()));
        rig.frames(300, -1.0, 0.0, 0.0); // 0, clipped to 400
        assert_eq!(rig.last(STEER), Some(set_target(400).to_vec()));
    }

    #[test]
    fn a_brake_frame_cuts_the_throttle_at_once() {
        let mut rig = LegacyRig::new();
        rig.arm();
        rig.frames(100, 0.0, 0.8, 0.0);
        assert!(rig.dac() > 0);
        rig.frame(0.0, 0.8, -0.3);
        assert_eq!(rig.dac(), 0);
        assert_eq!(rig.last(BRAKE), Some(set_target(1050).to_vec()));
    }

    #[test]
    fn silence_brakes_fully_within_the_timeout() {
        let mut rig = LegacyRig::new();
        rig.arm();
        rig.frames(100, 0.0, 0.5, 0.0);
        let mut out = rig.sb.tick();
        for _ in 0..25 {
            rig.sb.board().now += 10;
            out = rig.sb.tick();
        }
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        assert_eq!(rig.dac(), 0);
        assert_eq!(rig.last(BRAKE), Some(set_target(3500).to_vec()));
    }

    #[test]
    fn the_rc_override_beats_the_motor_pi() {
        let mut rig = LegacyRig::new();
        rig.arm();
        rig.frames(100, 0.0, 0.5, 0.0);
        let mut ch = [1500; ibus::CHANNELS];
        ch[5] = 2000; // the override switch
        ch[4] = 1500; // the gear switch on brake
        let rc = ibus::encode(&ibus::Channels(ch));
        rig.receiver.0.borrow_mut().extend(rc);
        let out = rig.frame(0.0, 0.5, 0.0);
        assert_eq!(out.mode, Mode::Rc);
    }

    // --- Version 2: the Orin's heartbeat (TODO 50) ---

    /// Armed (a stand-still frame: the hold brake is a stop), then driving at speed 0.5.
    fn driving_v2() -> (LegacyRig, u8) {
        let mut rig = LegacyRig::with_version(FrameVersion::V2);
        let mut counter = 0;
        rig.moving(5, 0.0, -0.3, &mut counter);
        let out = rig.moving(100, 0.5, 0.0, &mut counter);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!(rig.dac(), 350);
        (rig, counter)
    }

    #[test]
    fn v2_a_moving_counter_drives() {
        driving_v2();
    }

    #[test]
    fn v2_a_frozen_counter_brakes_within_the_timeout_though_frames_still_come() {
        let (mut rig, counter) = driving_v2();
        // The Pi keeps sending the same drive frame at 50 Hz: the Orin stopped.
        let mut braked_after = None;
        for i in 1..=15 {
            let out = rig.frame_v2(0.5, 0.0, counter, true);
            if braked_after.is_none() && out.faults & faults::CMD_TIMEOUT != 0 {
                braked_after = Some(i * 20);
                assert_eq!((out.throttle, out.brake), (0.0, 1.0));
            }
        }
        // 100 ms after the last new counter, at the next 20 ms tick.
        assert_eq!(braked_after, Some(120));
        assert_eq!(rig.dac(), 0);
        assert_eq!(rig.last(BRAKE), Some(set_target(3500).to_vec()));
        // The 15, and the very first frame after boot, which only set the counter.
        assert_eq!(rig.sb.link().stats().repeats, 16);
    }

    #[test]
    fn v2_after_a_heartbeat_loss_it_stays_braked_until_a_stop() {
        let (mut rig, mut counter) = driving_v2();
        for _ in 0..10 {
            rig.frame_v2(0.5, 0.0, counter, true);
        }
        // The counter moves again, still driving: latched, no throttle.
        let out = rig.moving(20, 0.5, 0.0, &mut counter);
        assert_eq!(out.mode, Mode::Manual);
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        assert_eq!(rig.dac(), 0);
        // A stop with a new counter (ego standing still), then driving again: re-armed.
        rig.moving(3, 0.0, -0.3, &mut counter);
        let out = rig.moving(100, 0.5, 0.0, &mut counter);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!(rig.dac(), 350);
    }

    #[test]
    fn v2_stop_frames_with_a_frozen_counter_do_not_re_arm() {
        // The bridge's stale stops while the Orin is gone carry the last counter: they
        // must not count as the arming edge's stop.
        let (mut rig, mut counter) = driving_v2();
        for _ in 0..20 {
            rig.frame_v2(0.0, -1.0, counter, false);
        }
        let out = rig.moving(30, 0.5, 0.0, &mut counter);
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(rig.dac(), 0);
    }

    #[test]
    fn v2_bad_crc_frames_are_ignored_so_the_heartbeat_stops() {
        let (mut rig, mut counter) = driving_v2();
        let mut out = rig.sb.tick();
        for _ in 0..10 {
            counter = counter.wrapping_add(1);
            let mut bytes = FrameV2 {
                values: Frame {
                    steering: 0.0,
                    speed: 0.5,
                    accel: 0.0,
                },
                counter,
                flags: Flags {
                    armed: true,
                    estop: false,
                },
            }
            .encode();
            bytes[16] ^= 0x5A;
            rig.motor_pi.0.borrow_mut().extend(bytes);
            rig.sb.board().now += 20;
            out = rig.sb.tick();
        }
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        assert_eq!(rig.dac(), 0);
        assert_eq!(rig.sb.link().stats().bad, 10);
    }

    #[test]
    fn v2_ignores_version_1_frames_so_never_drives() {
        let mut rig = LegacyRig::with_version(FrameVersion::V2);
        rig.arm(); // version 1 stop frames
        let out = rig.frames(100, 0.0, 0.5, 0.0);
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(rig.dac(), 0);
    }
}

/// The uplink and the RC via ROS, through the whole loop (TODO 50, 48).
#[cfg(test)]
mod uplink_loop_tests {
    extern crate std;
    use super::SpineBoard;
    use crate::actuators::Devices;
    use crate::calibration::Calibration;
    use crate::link::{FrameVersion, LegacyLink};
    use crate::rig::{FakeBoard, FakeBus, FakeTx, FakeUart};
    use ibus::RcMap;
    use legacy_link::{DriveMode, Flags, Frame, FrameV2, ModeSwitch, Uplink, UPLINK_LEN};
    use spine_core::protocol::Mode;
    use spine_core::{faults, Outputs, SpeedFeedback, SpineConfig};
    use std::vec::Vec;

    struct Rig {
        sb: SpineBoard<FakeBoard, FakeBus, FakeUart, LegacyLink<FakeUart, FakeTx>>,
        bus: FakeBus,
        motor_pi: FakeUart,
        receiver: FakeUart,
        uplink: FakeTx,
        counter: u8,
    }

    impl Rig {
        /// The board as `esp32-umod` builds it with `uplink`: version 2 frames, the
        /// three-position mode switch.
        fn new() -> Self {
            let bus = FakeBus::default();
            let (motor_pi, receiver, uplink) =
                (FakeUart::default(), FakeUart::default(), FakeTx::default());
            let cal = Calibration {
                rc: RcMap::THREE_POSITION,
                ..Calibration::TODAY
            };
            let devices = Devices::new(bus.clone(), bus.clone(), bus.clone(), bus.clone(), &cal);
            let config = SpineConfig {
                speed_feedback: SpeedFeedback::None {
                    full_throttle_speed: 3.0,
                },
                ..SpineConfig::default()
            };
            let link =
                LegacyLink::with_uplink(motor_pi.clone(), uplink.clone(), cal, FrameVersion::V2);
            let mut sb = SpineBoard::with_link(
                FakeBoard::new(),
                devices,
                receiver.clone(),
                link,
                cal,
                config,
            );
            sb.init();
            Rig {
                sb,
                bus,
                motor_pi,
                receiver,
                uplink,
                counter: 0,
            }
        }

        /// One iBUS frame: the mode switch, the throttle stick, the gear forward.
        fn rc(&mut self, switch: u16, throttle: u16) {
            let mut ch = [1500; ibus::CHANNELS];
            ch[5] = switch;
            ch[2] = throttle;
            ch[4] = 1000; // forward
            self.receiver
                .0
                .borrow_mut()
                .extend(ibus::encode(&ibus::Channels(ch)));
        }

        /// One version 2 frame with the next counter (speed, accel), then two 10 ms
        /// ticks, an iBUS frame before each when `rc` says so.
        fn step(&mut self, speed: f32, accel: f32, rc: Option<(u16, u16)>) -> Outputs {
            self.counter = self.counter.wrapping_add(1);
            let frame = FrameV2 {
                values: Frame {
                    steering: 0.0,
                    speed,
                    accel,
                },
                counter: self.counter,
                flags: Flags {
                    armed: true,
                    estop: false,
                },
            };
            self.motor_pi.0.borrow_mut().extend(frame.encode());
            let mut out = None;
            for _ in 0..2 {
                if let Some((switch, throttle)) = rc {
                    self.rc(switch, throttle);
                }
                self.sb.board().now += 10;
                out = Some(self.sb.tick());
            }
            out.unwrap()
        }

        fn steps(&mut self, n: usize, speed: f32, accel: f32, rc: Option<(u16, u16)>) -> Outputs {
            (0..n).map(|_| self.step(speed, accel, rc)).last().unwrap()
        }

        fn uplinks(&self) -> Vec<Uplink> {
            self.uplink
                .0
                .borrow()
                .chunks(UPLINK_LEN)
                .map(|c| Uplink::decode(c).unwrap())
                .collect()
        }

        fn last_uplink(&self) -> Uplink {
            *self.uplinks().last().unwrap()
        }

        fn dac(&self) -> u16 {
            let state = self.bus.0.borrow();
            let w = state.writes.iter().rev().find(|w| w.0 == crate::rig::DAC);
            w.map_or(0, |w| ad5311::decode([w.1[0], w.1[1]]).code)
        }
    }

    const VIA_ROS: u16 = 1500;
    const DIRECT: u16 = 2000;
    const FULL: u16 = 1000; // the throttle stick, all the way

    #[test]
    fn the_uplink_echoes_the_counter_at_50_hz() {
        let mut rig = Rig::new();
        rig.steps(5, 0.0, -0.3, None); // the arming stop
        let out = rig.steps(50, 0.5, 0.0, None);
        assert_eq!(out.mode, Mode::Computer);
        let ups = rig.uplinks();
        assert_eq!(ups.len(), 55); // one every 20 ms: 110 ticks of 10 ms
        let last = rig.last_uplink();
        assert_eq!(last.echo, Some(rig.counter));
        assert_eq!((last.mode, last.armed), (DriveMode::Computer, true));
        assert_eq!(last.switch, ModeSwitch::NoSignal);
        // The stand-in fake bus answers analog feedback: both positions known.
        assert!(last.steering.is_some() && last.brake_position.is_some());
        assert!(ups
            .windows(2)
            .all(|w| w[1].sequence == w[0].sequence.wrapping_add(1)));
    }

    #[test]
    fn rc_via_ros_sends_the_sticks_up_and_drives_only_from_the_frames() {
        let mut rig = Rig::new();
        let rc = Some((VIA_ROS, FULL));
        rig.steps(5, 0.0, -0.3, rc);
        // The sticks say full throttle; the frames say stand still: it stands still.
        let out = rig.steps(50, 0.0, -0.3, rc);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!(rig.dac(), 0);
        let up = rig.last_uplink();
        assert_eq!(up.switch, ModeSwitch::RcViaRos);
        assert_eq!(up.mode, DriveMode::Computer);
        assert_eq!(up.sticks.throttle, 1.0);
        // The frames drive (as the ROS driver's commands would): the DAC follows them.
        rig.steps(100, 0.5, 0.0, rc);
        assert_eq!(rig.dac(), 350);
    }

    #[test]
    fn rc_direct_drives_from_the_sticks_whatever_the_frames_say() {
        let mut rig = Rig::new();
        rig.steps(5, 0.0, -0.3, Some((VIA_ROS, FULL)));
        let out = rig.steps(100, 0.0, -0.3, Some((DIRECT, FULL)));
        assert_eq!(out.mode, Mode::Rc);
        assert!(rig.dac() > 0);
        let up = rig.last_uplink();
        assert_eq!(
            (up.mode, up.switch),
            (DriveMode::RcDirect, ModeSwitch::RcDirect)
        );
    }

    #[test]
    fn losing_the_rc_via_ros_brakes_with_fault_256() {
        let mut rig = Rig::new();
        let rc = Some((VIA_ROS, 1500));
        rig.steps(5, 0.0, -0.3, rc);
        rig.steps(100, 0.5, 0.0, rc);
        assert_eq!(rig.dac(), 350);
        // The receiver goes silent; the frames still drive. 100 ms later: braked.
        let out = rig.steps(8, 0.5, 0.0, None);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::RC_SIGNAL_LOST, 0);
        assert_eq!(rig.dac(), 0);
        let up = rig.last_uplink();
        assert_eq!(up.switch, ModeSwitch::NoSignal);
        assert_eq!(up.mode, DriveMode::Stopped);
        assert_ne!(up.faults & faults::RC_SIGNAL_LOST, 0);
    }
}

/// Finding the steering actuator's ends and centre (version 3 frames, 1 Nov), through the
/// whole loop: limp, jog, and the ways out.
#[cfg(test)]
mod steer_cal_loop_tests {
    extern crate std;
    use super::SpineBoard;
    use crate::actuators::Devices;
    use crate::calibration::Calibration;
    use crate::link::{FrameVersion, LegacyLink};
    use crate::rig::{BusState, FakeBoard, FakeBus, FakeTx, FakeUart, DAC, STEER};
    use core::cell::RefCell;
    use jrk_g2::command::{set_target, STOP_MOTOR};
    use legacy_link::{CalRequest, CalState, Flags, Frame, FrameV3, Refusal, Uplink, UPLINK_LEN};
    use spine_core::{Outputs, SpeedFeedback, SpineConfig};
    use std::rc::Rc;
    use std::vec::Vec;

    struct Rig {
        sb: SpineBoard<FakeBoard, FakeBus, FakeUart, LegacyLink<FakeUart, FakeTx>>,
        bus: Rc<RefCell<BusState>>,
        motor_pi: FakeUart,
        uplink: FakeTx,
        counter: u8,
    }

    impl Rig {
        fn new() -> Self {
            let bus = FakeBus::default();
            let (motor_pi, uplink) = (FakeUart::default(), FakeTx::default());
            let cal = Calibration::TODAY;
            let devices = Devices::new(bus.clone(), bus.clone(), bus.clone(), bus.clone(), &cal);
            let config = SpineConfig {
                speed_feedback: SpeedFeedback::None {
                    full_throttle_speed: 3.0,
                },
                ..SpineConfig::default()
            };
            let link =
                LegacyLink::with_uplink(motor_pi.clone(), uplink.clone(), cal, FrameVersion::V2);
            let mut sb = SpineBoard::with_link(
                FakeBoard::new(),
                devices,
                FakeUart::default(),
                link,
                cal,
                config,
            );
            sb.init();
            bus.0.borrow_mut().steering_feedback = 1950;
            Rig {
                sb,
                bus: bus.0,
                motor_pi,
                uplink,
                counter: 0,
            }
        }

        /// One version 3 frame with the next counter, then a 20 ms tick.
        fn frame(&mut self, speed: f32, accel: f32, armed: bool, cal: CalRequest) -> Outputs {
            self.counter = self.counter.wrapping_add(1);
            let frame = FrameV3 {
                values: Frame {
                    steering: 0.0,
                    speed,
                    accel,
                },
                counter: self.counter,
                flags: Flags {
                    armed,
                    estop: false,
                },
                calibration: cal,
            };
            self.motor_pi.0.borrow_mut().extend(frame.encode());
            self.sb.board().now += 20;
            self.sb.tick()
        }

        /// `n` calibration frames, as the bridge sends them: a stop, brake full.
        fn calibrate(&mut self, n: usize, cal: CalRequest) {
            for _ in 0..n {
                self.frame(0.0, -1.0, false, cal);
            }
        }

        /// The steering Jrk's writes since the last clear.
        fn steering_writes(&self) -> Vec<Vec<u8>> {
            let s = self.bus.borrow();
            s.writes
                .iter()
                .filter(|w| w.0 == STEER)
                .map(|w| w.1.clone())
                .collect()
        }

        fn targets(&self) -> Vec<u16> {
            self.steering_writes()
                .iter()
                .filter(|w| w.len() == 2)
                .map(|w| u16::from(w[0] & 0x1F) | (u16::from(w[1]) << 5))
                .collect()
        }

        fn clear(&mut self) {
            self.bus.borrow_mut().writes.clear();
        }

        fn last_uplink(&self) -> Uplink {
            let bytes = self.uplink.0.borrow();
            let last = bytes.chunks(UPLINK_LEN).last().unwrap();
            Uplink::decode(last).unwrap()
        }

        fn dac_codes(&self) -> Vec<u16> {
            let s = self.bus.borrow();
            s.writes
                .iter()
                .filter(|w| w.0 == DAC)
                .map(|w| ad5311::decode([w.1[0], w.1[1]]).code)
                .collect()
        }
    }

    #[test]
    fn the_target_encoding_matches_the_jrk_command() {
        // The helper above reads set_target's two bytes back.
        let mut rig = Rig::new();
        rig.clear();
        rig.frame(0.0, -1.0, false, CalRequest::Off);
        assert_eq!(
            rig.steering_writes().last(),
            Some(&set_target(1950).to_vec())
        );
        assert_eq!(rig.targets().last(), Some(&1950));
    }

    #[test]
    fn limp_stops_the_motor_and_the_uplink_reports_the_counts() {
        let mut rig = Rig::new();
        rig.calibrate(5, CalRequest::Off);
        rig.clear();
        rig.calibrate(10, CalRequest::Limp);
        let writes = rig.steering_writes();
        // The first frame's tick still steers (the spine's target); then every tick limp.
        assert!(writes[1..].iter().all(|w| w == &[STOP_MOTOR]), "{writes:?}");
        assert!(rig.dac_codes().iter().all(|&c| c == 0));
        // A person turns the wheels; the uplink sees it, in counts.
        rig.bus.borrow_mut().steering_feedback = 3010;
        rig.calibrate(3, CalRequest::Limp);
        let u = rig.last_uplink();
        assert_eq!(u.calibration, CalState::Limp);
        assert_eq!(u.counts.steering, Some(3010));
    }

    #[test]
    fn leaving_limp_holds_the_wheels_then_brings_them_back_slowly() {
        let mut rig = Rig::new();
        rig.calibrate(5, CalRequest::Limp);
        rig.bus.borrow_mut().steering_feedback = 3010;
        rig.calibrate(2, CalRequest::Limp);
        rig.clear();
        rig.calibrate(1, CalRequest::Off);
        // 20 ms at 300 counts a second: 6 counts from where the wheels are.
        assert_eq!(rig.targets(), [3004]);
        rig.calibrate(400, CalRequest::Off);
        let t = rig.targets();
        assert!(
            t.windows(2).all(|w| w[0] >= w[1] && w[0] - w[1] <= 6),
            "{t:?}"
        );
        assert_eq!(t.last(), Some(&1950));
        assert_eq!(rig.last_uplink().calibration, CalState::Off);
    }

    #[test]
    fn jog_moves_slowly_and_never_past_the_software_limits() {
        let mut rig = Rig::new();
        rig.calibrate(5, CalRequest::Off);
        rig.clear();
        rig.calibrate(700, CalRequest::Jog(4000));
        let t = rig.targets();
        assert!(
            t.windows(2).all(|w| w[1] >= w[0] && w[1] - w[0] <= 6),
            "{t:?}"
        );
        assert_eq!(t.last(), Some(&3500));
        assert_eq!(rig.last_uplink().calibration, CalState::Jog);
    }

    #[test]
    fn a_frame_asking_for_speed_is_refused_and_has_no_throttle() {
        let mut rig = Rig::new();
        rig.calibrate(5, CalRequest::Off);
        for _ in 0..50 {
            rig.frame(0.5, 0.0, true, CalRequest::Limp);
        }
        assert!(rig.dac_codes().iter().all(|&c| c == 0));
        assert!(!rig.steering_writes().iter().any(|w| w == &[STOP_MOTOR]));
        assert_eq!(
            rig.last_uplink().calibration,
            CalState::Refused(Refusal::Moving)
        );
    }

    #[test]
    fn without_analog_feedback_limp_is_refused() {
        let mut rig = Rig::new();
        rig.bus.borrow_mut().feedback_modes.push((STEER, 0)); // feedback: none
        rig.calibrate(10, CalRequest::Limp);
        assert!(!rig.steering_writes().iter().any(|w| w == &[STOP_MOTOR]));
        let u = rig.last_uplink();
        assert_eq!(u.calibration, CalState::Refused(Refusal::NoFeedback));
        assert_eq!(u.counts.steering, None);
    }

    #[test]
    fn losing_the_heartbeat_mid_limp_holds_where_the_wheels_are() {
        let mut rig = Rig::new();
        rig.calibrate(5, CalRequest::Limp);
        rig.bus.borrow_mut().steering_feedback = 900;
        rig.calibrate(2, CalRequest::Limp);
        rig.clear();
        // The Orin goes quiet: no frames at all for 300 ms.
        for _ in 0..30 {
            rig.sb.board().now += 10;
            rig.sb.tick();
        }
        let writes = rig.steering_writes();
        // Limp for 100 ms, then held at 900: the motor on again, not moving.
        assert!(writes[..9].iter().all(|w| w == &[STOP_MOTOR]), "{writes:?}");
        assert!(rig.targets().iter().all(|&t| t == 900));
        assert!(!rig.targets().is_empty());
        assert_eq!(rig.last_uplink().calibration, CalState::Hold);
        assert!(rig.dac_codes().iter().all(|&c| c == 0));
    }
}

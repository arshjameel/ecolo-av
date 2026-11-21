//! A test rig: a fake board, an I2C bus that records every write and answers every
//! read, and a fake serial port. The loop makes about ten transfers a tick, so the
//! end-to-end tests look at what was written rather than list each transfer; the
//! drivers and `actuators` are tested transfer by transfer with `embedded-hal-mock`.

extern crate std;

use crate::actuators::Devices;
use crate::board::{Board, CanFrame, WheelCount};
use crate::calibration::Calibration;
use crate::spine_board::SpineBoard;
use core::cell::RefCell;
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use ibus::{encode, Channels, CHANNELS};
use spine_core::protocol::{DriveCmd, EstopCmd, DRIVE_CMD_ID, ESTOP_CMD_ID};
use spine_core::{Outputs, SpineConfig};
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec;

pub const STEER: u8 = 0x05;
pub const BRAKE: u8 = 0x04;
pub const DAC: u8 = 0x0C;
pub const EXPANDER: u8 = 0x20;

/// What the fake bus has seen, and what its devices answer.
#[derive(Default)]
pub struct BusState {
    /// Every transfer that only wrote, in order: (address, bytes).
    pub writes: Vec<(u8, Vec<u8>)>,
    /// Addresses that don't answer.
    pub failing: Vec<u8>,
    /// The steering Jrk's scaled feedback.
    pub steering_feedback: u16,
    /// The brake Jrk's scaled feedback.
    pub brake_feedback: u16,
    /// What a Jrk says its feedback mode is, if not 1 (analog): (address, mode).
    pub feedback_modes: Vec<(u8, u8)>,
    /// The error flags each Jrk reports: (address, flags).
    pub jrk_errors: Vec<(u8, u16)>,
}

#[derive(Clone, Default)]
pub struct FakeBus(pub Rc<RefCell<BusState>>);

impl ErrorType for FakeBus {
    type Error = ErrorKind;
}

impl I2c for FakeBus {
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        let mut s = self.0.borrow_mut();
        if s.failing.contains(&address) {
            return Err(ErrorKind::Other);
        }
        let mut command = None;
        let mut written = Vec::new();
        let mut read = false;
        for op in ops {
            match op {
                Operation::Write(bytes) => {
                    command = bytes.first().copied();
                    written.extend_from_slice(bytes);
                }
                Operation::Read(buf) => {
                    read = true;
                    let value = match (address, command) {
                        (STEER, Some(0xA7)) => s.steering_feedback,
                        (BRAKE, Some(0xA7)) => s.brake_feedback,
                        (a, Some(0xEA)) => s
                            .feedback_modes
                            .iter()
                            .find(|m| m.0 == a)
                            .map_or(1, |m| u16::from(m.1)),
                        (a, Some(0xB3)) => {
                            s.jrk_errors.iter().find(|e| e.0 == a).map_or(0, |e| e.1)
                        }
                        _ => 0,
                    };
                    let bytes = value.to_le_bytes();
                    for (i, b) in buf.iter_mut().enumerate() {
                        *b = bytes.get(i).copied().unwrap_or(0);
                    }
                }
            }
        }
        if !read {
            s.writes.push((address, written));
        }
        Ok(())
    }
}

pub struct FakeBoard {
    pub now: u32,
    pub estop_open: bool,
    pub wheel: WheelCount,
    pub rx: VecDeque<CanFrame>,
    pub sent: Vec<CanFrame>,
    pub can_ok: bool,
}

impl FakeBoard {
    /// At 1000 ms, the e-stop closed, CAN working, nothing received.
    pub fn new() -> Self {
        FakeBoard {
            now: 1000,
            estop_open: false,
            wheel: WheelCount::default(),
            rx: VecDeque::new(),
            sent: Vec::new(),
            can_ok: true,
        }
    }
}

impl Board for FakeBoard {
    fn now_ms(&mut self) -> u32 {
        self.now
    }
    fn estop_open(&mut self) -> bool {
        self.estop_open
    }
    fn wheel(&mut self) -> WheelCount {
        self.wheel
    }
    fn can_receive(&mut self) -> Option<CanFrame> {
        self.rx.pop_front()
    }
    fn can_send(&mut self, frame: &CanFrame) -> bool {
        if self.can_ok {
            self.sent.push(*frame);
        }
        self.can_ok
    }
}

#[derive(Clone, Default)]
pub struct FakeUart(pub Rc<RefCell<VecDeque<u8>>>);

impl embedded_io::ErrorType for FakeUart {
    type Error = embedded_io::ErrorKind;
}

impl embedded_io::Read for FakeUart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut q = self.0.borrow_mut();
        let n = buf.len().min(q.len());
        for b in &mut buf[..n] {
            *b = q.pop_front().unwrap_or(0);
        }
        Ok(n)
    }
}

impl embedded_io::ReadReady for FakeUart {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.0.borrow().is_empty())
    }
}

/// A serial port's TX: every byte written, in order.
#[derive(Clone, Default)]
pub struct FakeTx(pub Rc<RefCell<Vec<u8>>>);

impl embedded_io::ErrorType for FakeTx {
    type Error = embedded_io::ErrorKind;
}

impl embedded_io::Write for FakeTx {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.borrow_mut().extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// A whole spine on fakes, after `init()`, with the clock at 1000 ms.
pub struct Rig {
    pub sb: SpineBoard<FakeBoard, FakeBus, FakeUart>,
    pub bus: Rc<RefCell<BusState>>,
    pub uart: Rc<RefCell<VecDeque<u8>>>,
    counter: u8,
}

impl Rig {
    pub fn new() -> Self {
        let bus = FakeBus::default();
        let uart = FakeUart::default();
        let cal = Calibration::TODAY;
        let devices = Devices::new(bus.clone(), bus.clone(), bus.clone(), bus.clone(), &cal);
        let mut sb = SpineBoard::new(
            FakeBoard::new(),
            devices,
            uart.clone(),
            cal,
            SpineConfig::default(),
        );
        sb.init();
        Rig {
            sb,
            bus: bus.0,
            uart: uart.0,
            counter: 0,
        }
    }

    /// 10 ms on, then one tick.
    pub fn tick(&mut self) -> Outputs {
        self.sb.board().now += 10;
        self.sb.tick()
    }

    pub fn ticks(&mut self, n: usize) -> Outputs {
        (0..n).map(|_| self.tick()).last().unwrap()
    }

    /// Queue a CAN frame for the next tick.
    pub fn can(&mut self, id: u16, data: &[u8]) {
        let frame = CanFrame::new(id, data).unwrap();
        self.sb.board().rx.push_back(frame);
    }

    /// Queue a DRIVE_CMD, with the next counter.
    pub fn drive(&mut self, speed: f32, armed: bool) {
        self.counter = (self.counter + 1) & 0x0F;
        let cmd = DriveCmd {
            target_speed: speed,
            steering_angle: 0.0,
            accel_limit: 0.0,
            steer_rate_limit: 0.0,
            armed,
            counter: self.counter,
        };
        self.can(DRIVE_CMD_ID, &cmd.encode());
    }

    pub fn estop_frame(&mut self, request: bool) {
        let cmd = EstopCmd {
            estop_request: request,
            counter: 0,
        };
        self.can(ESTOP_CMD_ID, &cmd.encode());
    }

    /// One DRIVE_CMD a tick, for `n` ticks.
    pub fn drive_for(&mut self, speed: f32, armed: bool, n: usize) -> Outputs {
        (0..n)
            .map(|_| {
                self.drive(speed, armed);
                self.tick()
            })
            .last()
            .unwrap()
    }

    /// Wait for the e-stop input to release (50 ms), sending disarm frames, so the
    /// next armed frame arms the spine.
    pub fn arm(&mut self) {
        self.drive_for(0.0, false, 8);
    }

    /// Queue one iBUS frame on the serial port: the mode switch on manual (`override_on`)
    /// or normal, the left stick (1500 is the brake band, 1000 full forward), steering.
    pub fn rc(&mut self, override_on: bool, throttle: u16, steering: u16) {
        let mut ch = [1500; CHANNELS];
        ch[5] = if override_on { 2000 } else { 1000 };
        ch[2] = throttle;
        ch[3] = steering;
        self.uart.borrow_mut().extend(encode(&Channels(ch)));
    }

    /// Every write since the last `clear`.
    pub fn writes(&self) -> Vec<(u8, Vec<u8>)> {
        self.bus.borrow().writes.clone()
    }

    pub fn clear(&mut self) {
        self.bus.borrow_mut().writes.clear();
    }

    /// The codes written to the DAC, oldest first.
    pub fn dac_codes(&self) -> Vec<u16> {
        self.writes()
            .iter()
            .filter(|w| w.0 == DAC)
            .map(|w| ad5311::decode([w.1[0], w.1[1]]).code)
            .collect()
    }

    /// Where in `writes()` a Jrk target or DAC code was first written.
    pub fn position(&self, address: u8, value: u16) -> Option<usize> {
        let bytes = if address == DAC {
            ad5311::encode(value, ad5311::Mode::Normal)
        } else {
            jrk_g2::command::set_target(value)
        };
        self.writes()
            .iter()
            .position(|w| w.0 == address && w.1 == bytes)
    }
}

//! The Microchip MCP2515 CAN controller, over SPI.
//!
//! The Pico 2's chip, the RP2350, has no CAN controller, so the spine reaches the car's bus
//! through an MCP2515 and a transceiver. This driver sets the chip up for one bus (the bit
//! timing from the module's crystal, acceptance filters for our IDs), sends and receives
//! classic CAN frames, and reports the bus's error state.
//!
//! Plain `no_std` Rust against `embedded-hal` 1.0's `SpiDevice`: it runs on any board, and in
//! `cargo test` with a mock bus. The instructions and registers come from Microchip's *MCP2515
//! Stand-Alone CAN Controller with SPI Interface* data sheet (DS20001801K). The bit timing
//! follows Linux's `mcp251x` driver, so the spine samples each bit where SocketCAN on the
//! Orin does.
//!
//! | Step | Call |
//! |---|---|
//! | set up | [`Mcp2515::init`] with a [`Config`]: [`BitTiming`], [`Filters`], [`Mode`] |
//! | send | [`Mcp2515::send`]: into a free transmit buffer, of three |
//! | receive | [`Mcp2515::receive`]: from two receive buffers, the older frame first |
//! | watch | [`Mcp2515::bus_status`]: the error counters and state, lost frames, the mode |
//!
//! ```
//! use embedded_hal_mock::eh1::spi::{Mock, Transaction as T};
//! use mcp2515::{Frame, Mcp2515};
//!
//! // What the chip sees when the spine sends two bytes with the ID 0x181:
//! let spi = Mock::new(&[
//!     // READ EFLG: not off the bus
//!     T::transaction_start(),
//!     T::write_vec(vec![0x03, 0x2D]),
//!     T::read_vec(vec![0x00]),
//!     T::transaction_end(),
//!     // READ STATUS: every transmit buffer is free
//!     T::transaction_start(),
//!     T::write_vec(vec![0xA0]),
//!     T::read_vec(vec![0x00]),
//!     T::transaction_end(),
//!     // LOAD TX BUFFER 0: SIDH, SIDL, EID8, EID0, DLC, then the data
//!     T::transaction_start(),
//!     T::write_vec(vec![0x40]),
//!     T::write_vec(vec![0x30, 0x20, 0, 0, 2, 7, 9]),
//!     T::transaction_end(),
//!     // RTS: send TXB0
//!     T::transaction_start(),
//!     T::write_vec(vec![0x81]),
//!     T::transaction_end(),
//! ]);
//! let mut can = Mcp2515::new(spi);
//! can.send(&Frame::new_standard(0x181, &[7, 9]).unwrap()).unwrap();
//! can.release().done();
//! ```
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

mod filter;
mod frame;
mod regs;
mod timing;

pub use embedded_can::{ExtendedId, Id, StandardId};
pub use filter::{FilterError, Filters, MAX_IDS};
pub use frame::Frame;
pub use timing::{BitTiming, Bitrate, Crystal, TimingError};

use core::fmt;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};
use regs::{eflg, Instruction, Register, RxBuffer, TxBuffer};

/// How long [`Mcp2515::init`] waits for each change of mode: 100 polls, 100 µs apart, so
/// 10 ms. After a reset the chip needs 128 cycles of its crystal; it enters normal mode once
/// the bus is idle, within one frame.
const MODE_POLLS: u32 = 100;
const POLL_US: u32 = 100;

/// How the chip takes part in the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Send, receive, and acknowledge the others' frames: the car.
    #[default]
    Normal,
    /// Receive only: it never sends anything, not even an acknowledgement or an error flag.
    /// To watch a bus without disturbing it.
    ListenOnly,
    /// Frames go from the transmit buffers straight to the receive buffers, never to the
    /// bus: to check the SPI wiring with no bus at all.
    Loopback,
}

impl Mode {
    /// CANCTRL's REQOP bits for this mode (Register 10-1).
    const fn bits(self) -> u8 {
        match self {
            Mode::Normal => 0b000,
            Mode::Loopback => 0b010,
            Mode::ListenOnly => 0b011,
        }
    }

    /// The mode in CANSTAT's OPMOD bits; None in sleep or configuration mode.
    fn from_canstat(canstat: u8) -> Option<Mode> {
        match canstat >> regs::MODE_SHIFT {
            0b000 => Some(Mode::Normal),
            0b010 => Some(Mode::Loopback),
            0b011 => Some(Mode::ListenOnly),
            _ => None,
        }
    }
}

/// Everything [`Mcp2515::init`] sets up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// From the module's crystal and the bus's bitrate: [`BitTiming::new`].
    pub timing: BitTiming,
    /// Which frames the chip keeps.
    pub filters: Filters,
    /// The mode to leave the chip in.
    pub mode: Mode,
}

/// What can go wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// An SPI transfer failed.
    Spi(E),
    /// No MCP2515 answers: after a reset it must report configuration mode, and then keep
    /// the settings written to it. Check its power, and the CS, SCK, SI and SO wires.
    NoChip,
    /// The chip didn't change mode within 10 ms.
    ModeTimeout,
    /// All three transmit buffers are still waiting for the bus. Nobody acknowledges our
    /// frames (no other node on the bus, or no terminators), or the bus is very busy.
    TxBusy,
    /// The chip has left the bus after too many errors. It comes back by itself once the
    /// bus has been idle for 128 times 11 bits (data sheet, section 6.6).
    BusOff,
}

impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Spi(e) => write!(f, "an SPI transfer to the MCP2515 failed: {e:?}"),
            Error::NoChip => f.write_str("no MCP2515 answers on this SPI bus"),
            Error::ModeTimeout => f.write_str("the MCP2515 didn't change mode within 10 ms"),
            Error::TxBusy => f.write_str("all three transmit buffers are waiting for the bus"),
            Error::BusOff => f.write_str("the MCP2515 is off the bus after too many errors"),
        }
    }
}

impl<E: fmt::Debug> core::error::Error for Error<E> {}

/// CAN's error states (data sheet, section 6.6), from the two error counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorState {
    /// Fewer than 96 errors on both counters: all well.
    Active,
    /// 96 or more on a counter: still working, but something on the bus is wrong.
    Warning,
    /// 128 or more: still working, but it signals errors without disturbing the bus.
    Passive,
    /// More than 255 transmit errors: off the bus, see [`Error::BusOff`].
    BusOff,
}

impl ErrorState {
    fn from_eflg(flags: u8) -> ErrorState {
        if flags & eflg::TXBO != 0 {
            ErrorState::BusOff
        } else if flags & (eflg::TXEP | eflg::RXEP) != 0 {
            ErrorState::Passive
        } else if flags & eflg::EWARN != 0 {
            ErrorState::Warning
        } else {
            ErrorState::Active
        }
    }
}

/// What [`Mcp2515::bus_status`] found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusStatus {
    /// Error active, passive or bus off.
    pub state: ErrorState,
    /// The transmit and receive error counters (TEC, REC).
    pub tx_errors: u8,
    /// The receive error counter (REC).
    pub rx_errors: u8,
    /// A frame arrived with both receive buffers full, and was lost, since the last call.
    pub rx_overflow: bool,
    /// The chip's mode now. None in configuration mode: a chip that reset itself, after a
    /// dip in its supply, needs [`Mcp2515::init`] again.
    pub mode: Option<Mode>,
}

/// One MCP2515 on an SPI bus.
#[derive(Debug)]
pub struct Mcp2515<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> Mcp2515<SPI> {
    /// The chip on `spi`, with its own CS pin: SPI mode 0 (or 3), at most 10 MHz.
    pub fn new(spi: SPI) -> Self {
        Mcp2515 { spi }
    }

    /// Give the bus back.
    pub fn release(self) -> SPI {
        self.spi
    }

    /// Reset the chip, then set it up: the bit timing, the filters, the INT pin for frames
    /// received, then `config.mode`. `delay` spaces the polls while the chip changes mode.
    ///
    /// Until this succeeds, the chip sends nothing and acknowledges nothing.
    ///
    /// # Errors
    ///
    /// [`Error::NoChip`] if no MCP2515 answers, [`Error::ModeTimeout`] if it won't
    /// change mode, [`Error::Spi`] if a transfer fails.
    pub fn init(
        &mut self,
        config: &Config,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<SPI::Error>> {
        self.command(&[Instruction::Reset.byte()])?;
        // An MCP2515 is there if it reports configuration mode, with CANCTRL as a reset
        // leaves it. A missing chip reads as all zeros or all ones.
        let reset = self.poll(delay, |chip| {
            let mut r = [0; 2];
            chip.read(Register::CanStat, &mut r)?;
            let [canstat, canctrl] = r;
            Ok(canstat >> regs::MODE_SHIFT == regs::MODE_CONFIGURATION
                && canctrl == regs::CANCTRL_AFTER_RESET)
        })?;
        if !reset {
            return Err(Error::NoChip);
        }

        let [cnf3, cnf2, cnf1] = config.timing.registers();
        let timing = [cnf3, cnf2, cnf1, regs::RX_INTERRUPTS];
        self.write(Register::Cnf3, &timing)?;
        let filters = config.filters.registers();
        self.write(Register::Rxf0, &filters.rxf0_2)?;
        self.write(Register::Rxf3, &filters.rxf3_5)?;
        self.write(Register::Rxm0, &filters.masks)?;
        self.write(Register::Rxb0Ctrl, &[filters.rxb0ctrl])?;
        self.write(Register::Rxb1Ctrl, &[filters.rxb1ctrl])?;
        // Read the timing back: a chip that can't hear us (SI not wired) fails here, rather
        // than on the bus.
        let mut back = [0; 4];
        self.read(Register::Cnf3, &mut back)?;
        if back != timing {
            return Err(Error::NoChip);
        }

        // REQOP; one-shot mode, abort and the CLKOUT pin all off.
        self.write(Register::CanCtrl, &[config.mode.bits() << regs::MODE_SHIFT])?;
        let running = self.poll(delay, |chip| Ok(chip.mode()? == Some(config.mode)))?;
        if running {
            Ok(())
        } else {
            Err(Error::ModeTimeout)
        }
    }

    /// Put `frame` in a free transmit buffer, and ask the chip to send it. The chip keeps
    /// trying until another node acknowledges it.
    ///
    /// # Errors
    ///
    /// [`Error::BusOff`] off the bus, [`Error::TxBusy`] with every buffer still
    /// waiting, [`Error::Spi`] if a transfer fails.
    pub fn send(&mut self, frame: &Frame) -> Result<(), Error<SPI::Error>> {
        if self.read_byte(Register::Eflg)? & eflg::TXBO != 0 {
            return Err(Error::BusOff);
        }
        let status = self.read_status()?;
        let buffer = TxBuffer::ALL
            .into_iter()
            .find(|&b| status & regs::status_tx_pending(b) == 0)
            .ok_or(Error::TxBusy)?;
        let (bytes, len) = frame.to_registers();
        self.transaction(&mut [
            Operation::Write(&[Instruction::LoadTxBuffer(buffer).byte()]),
            Operation::Write(&bytes[..len]),
        ])?;
        self.command(&[Instruction::RequestToSend(buffer).byte()])
    }

    /// The next frame received, if there is one. RXB0 goes first: when both buffers are
    /// full, it holds the older frame.
    ///
    /// # Errors
    ///
    /// [`Error::Spi`] if a transfer fails.
    pub fn receive(&mut self) -> Result<Option<Frame>, Error<SPI::Error>> {
        let status = self.read_status()?;
        let Some(buffer) = RxBuffer::ALL
            .into_iter()
            .find(|&b| status & regs::status_rx_full(b) != 0)
        else {
            return Ok(None);
        };
        // Raising CS at the end of READ RX BUFFER frees the buffer for the next frame.
        let mut r = [0; 13];
        self.transaction(&mut [
            Operation::Write(&[Instruction::ReadRxBuffer(buffer).byte()]),
            Operation::Read(&mut r),
        ])?;
        Ok(Some(Frame::from_registers(&r)))
    }

    /// The error counters and state, the mode, and whether a frame was lost since the last
    /// call.
    ///
    /// # Errors
    ///
    /// [`Error::Spi`] if a transfer fails.
    pub fn bus_status(&mut self) -> Result<BusStatus, Error<SPI::Error>> {
        let mut r = [0; 3];
        self.read(Register::Tec, &mut r)?; // TEC, REC, and CANSTAT's copy at 0x1E
        let [tx_errors, rx_errors, canstat] = r;
        let flags = self.read_byte(Register::Eflg)?;
        let rx_overflow = flags & eflg::OVERFLOW != 0;
        if rx_overflow {
            self.command(&[
                Instruction::BitModify.byte(),
                Register::Eflg.address(),
                eflg::OVERFLOW,
                0,
            ])?;
        }
        Ok(BusStatus {
            state: ErrorState::from_eflg(flags),
            tx_errors,
            rx_errors,
            rx_overflow,
            mode: Mode::from_canstat(canstat),
        })
    }

    /// Ask `ready` until it says yes: at most [`MODE_POLLS`] times, [`POLL_US`] apart.
    fn poll(
        &mut self,
        delay: &mut impl DelayNs,
        mut ready: impl FnMut(&mut Self) -> Result<bool, Error<SPI::Error>>,
    ) -> Result<bool, Error<SPI::Error>> {
        for _ in 0..MODE_POLLS {
            if ready(self)? {
                return Ok(true);
            }
            delay.delay_us(POLL_US);
        }
        Ok(false)
    }

    fn mode(&mut self) -> Result<Option<Mode>, Error<SPI::Error>> {
        self.read_byte(Register::CanStat).map(Mode::from_canstat)
    }

    fn read_status(&mut self) -> Result<u8, Error<SPI::Error>> {
        let mut status = [0];
        self.transaction(&mut [
            Operation::Write(&[Instruction::ReadStatus.byte()]),
            Operation::Read(&mut status),
        ])?;
        Ok(status[0])
    }

    fn read(&mut self, start: Register, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        self.transaction(&mut [
            Operation::Write(&[Instruction::Read.byte(), start.address()]),
            Operation::Read(buf),
        ])
    }

    fn read_byte(&mut self, register: Register) -> Result<u8, Error<SPI::Error>> {
        let mut byte = [0];
        self.read(register, &mut byte)?;
        Ok(byte[0])
    }

    fn write(&mut self, start: Register, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.transaction(&mut [
            Operation::Write(&[Instruction::Write.byte(), start.address()]),
            Operation::Write(data),
        ])
    }

    fn command(&mut self, bytes: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.transaction(&mut [Operation::Write(bytes)])
    }

    /// One instruction: CS low, the operations, CS high.
    fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Error<SPI::Error>> {
        self.spi.transaction(ops).map_err(Error::Spi)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::spi::{ErrorKind, ErrorType};
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction as T};
    use std::vec;
    use std::vec::Vec;

    type Script = Vec<T<u8>>;

    /// One instruction that only writes.
    fn command(bytes: &[u8]) -> Script {
        vec![
            T::transaction_start(),
            T::write_vec(bytes.to_vec()),
            T::transaction_end(),
        ]
    }

    /// READ from `address`, and what the chip answers.
    fn read(address: u8, reply: &[u8]) -> Script {
        vec![
            T::transaction_start(),
            T::write_vec(vec![0x03, address]),
            T::read_vec(reply.to_vec()),
            T::transaction_end(),
        ]
    }

    /// WRITE `data` from `address`.
    fn write(address: u8, data: &[u8]) -> Script {
        vec![
            T::transaction_start(),
            T::write_vec(vec![0x02, address]),
            T::write_vec(data.to_vec()),
            T::transaction_end(),
        ]
    }

    /// An instruction byte, then `reply` from the chip: READ STATUS, READ RX BUFFER.
    fn ask(instruction: u8, reply: &[u8]) -> Script {
        vec![
            T::transaction_start(),
            T::write_vec(vec![instruction]),
            T::read_vec(reply.to_vec()),
            T::transaction_end(),
        ]
    }

    /// An instruction byte, then `data`: LOAD TX BUFFER.
    fn load(instruction: u8, data: &[u8]) -> Script {
        vec![
            T::transaction_start(),
            T::write_vec(vec![instruction]),
            T::write_vec(data.to_vec()),
            T::transaction_end(),
        ]
    }

    fn chip(steps: &[Script]) -> Mcp2515<Mock<u8>> {
        Mcp2515::new(Mock::new(&steps.concat()))
    }

    fn id(raw: u16) -> StandardId {
        StandardId::new(raw).unwrap()
    }

    /// The spine's set-up: the car's bus, and ESTOP_CMD and DRIVE_CMD only.
    fn spines(crystal: Crystal, mode: Mode) -> Config {
        Config {
            timing: BitTiming::new(crystal, Bitrate::KBPS_500).unwrap(),
            filters: Filters::standard_ids(&[id(0x010), id(0x100)]).unwrap(),
            mode,
        }
    }

    /// Everything `init` writes after the reset, with these CNF3, CNF2, CNF1 bytes, up to
    /// the read-back of the timing.
    fn settings(cnf: [u8; 3]) -> Vec<Script> {
        let estop = [0x02, 0x00, 0, 0];
        let drive = [0x20, 0x00, 0, 0];
        let timing = [cnf[0], cnf[1], cnf[2], 0x03];
        vec![
            write(0x28, &timing),
            write(0x00, &[estop, drive, estop].concat()),
            write(0x10, &[drive, estop, drive].concat()),
            write(0x20, &[0xFF, 0xE0, 0, 0, 0xFF, 0xE0, 0, 0]),
            write(0x60, &[0x04]),
            write(0x70, &[0x00]),
            read(0x28, &timing),
        ]
    }

    /// The reset, then the chip in configuration mode at once.
    fn reset() -> Vec<Script> {
        vec![command(&[0xC0]), read(0x0E, &[0x80, 0x87])]
    }

    #[test]
    fn init_sets_up_the_cars_bus_on_a_16_mhz_module() {
        let mut steps = reset();
        steps.extend(settings([0x01, 0xB5, 0x00]));
        steps.push(write(0x0F, &[0x00])); // normal mode
        steps.push(read(0x0E, &[0x00]));
        let mut can = chip(&steps);
        let config = spines(Crystal::MHZ_16, Mode::Normal);
        assert_eq!(can.init(&config, &mut NoopDelay::new()), Ok(()));
        can.release().done();
    }

    #[test]
    fn an_8_mhz_module_gets_its_own_timing() {
        let mut steps = reset();
        steps.extend(settings([0x01, 0x91, 0x00]));
        steps.push(write(0x0F, &[0x00]));
        steps.push(read(0x0E, &[0x00]));
        let mut can = chip(&steps);
        let config = spines(Crystal::MHZ_8, Mode::Normal);
        assert_eq!(can.init(&config, &mut NoopDelay::new()), Ok(()));
        can.release().done();
    }

    #[test]
    fn init_waits_out_the_reset() {
        // The chip answers nonsense while its oscillator starts, then reports for duty.
        let mut steps = vec![
            command(&[0xC0]),
            read(0x0E, &[0xFF, 0xFF]),
            read(0x0E, &[0x00, 0x00]),
            read(0x0E, &[0x80, 0x87]),
        ];
        steps.extend(settings([0x01, 0xB5, 0x00]));
        steps.push(write(0x0F, &[0x00]));
        steps.push(read(0x0E, &[0x80])); // still in configuration mode
        steps.push(read(0x0E, &[0x00]));
        let mut can = chip(&steps);
        let config = spines(Crystal::MHZ_16, Mode::Normal);
        assert_eq!(can.init(&config, &mut NoopDelay::new()), Ok(()));
        can.release().done();
    }

    #[test]
    fn the_other_modes() {
        for (mode, canctrl) in [(Mode::ListenOnly, 0x60), (Mode::Loopback, 0x40)] {
            let mut steps = reset();
            steps.extend(settings([0x01, 0xB5, 0x00]));
            steps.push(write(0x0F, &[canctrl]));
            steps.push(read(0x0E, &[canctrl])); // OPMOD sits where REQOP does
            let mut can = chip(&steps);
            let config = spines(Crystal::MHZ_16, mode);
            assert_eq!(can.init(&config, &mut NoopDelay::new()), Ok(()), "{mode:?}");
            can.release().done();
        }
    }

    #[test]
    fn no_chip_answers() {
        // SO floating low, then high: neither ever reads as a fresh MCP2515.
        for floating in [0x00, 0xFF] {
            let mut steps = vec![command(&[0xC0])];
            steps.extend((0..MODE_POLLS).map(|_| read(0x0E, &[floating, floating])));
            let mut can = chip(&steps);
            let config = spines(Crystal::MHZ_16, Mode::Normal);
            assert_eq!(can.init(&config, &mut NoopDelay::new()), Err(Error::NoChip));
            can.release().done();
        }
    }

    #[test]
    fn a_chip_that_cant_hear_us_is_no_chip() {
        // SI not wired: nothing we write arrives. A chip fresh from power-up still reads as
        // reset, but its timing reads back as zeros.
        let mut steps = reset();
        let mut writes = settings([0x01, 0xB5, 0x00]);
        writes.pop();
        steps.extend(writes);
        steps.push(read(0x28, &[0, 0, 0, 0]));
        let mut can = chip(&steps);
        let config = spines(Crystal::MHZ_16, Mode::Normal);
        assert_eq!(can.init(&config, &mut NoopDelay::new()), Err(Error::NoChip));
        can.release().done();
    }

    #[test]
    fn a_mode_that_never_comes() {
        let mut steps = reset();
        steps.extend(settings([0x01, 0xB5, 0x00]));
        steps.push(write(0x0F, &[0x00]));
        steps.extend((0..MODE_POLLS).map(|_| read(0x0E, &[0x80])));
        let mut can = chip(&steps);
        let config = spines(Crystal::MHZ_16, Mode::Normal);
        assert_eq!(
            can.init(&config, &mut NoopDelay::new()),
            Err(Error::ModeTimeout)
        );
        can.release().done();
    }

    /// The frame `send_*` tests send: VEHICLE_STATUS's ID, two bytes.
    fn status_frame() -> (Frame, [u8; 7]) {
        let frame = Frame::new_standard(0x181, &[7, 9]).unwrap();
        (frame, [0x30, 0x20, 0, 0, 2, 7, 9])
    }

    #[test]
    fn send_takes_the_first_free_buffer() {
        let (frame, bytes) = status_frame();
        // READ STATUS bits: TXB0's TXREQ is 0x04, TXB1's 0x10.
        for (status, load_tx, rts) in [(0x00, 0x40, 0x81), (0x04, 0x42, 0x82), (0x14, 0x44, 0x84)] {
            let mut can = chip(&[
                read(0x2D, &[0x00]),
                ask(0xA0, &[status]),
                load(load_tx, &bytes),
                command(&[rts]),
            ]);
            assert_eq!(can.send(&frame), Ok(()), "status {status:#04x}");
            can.release().done();
        }
    }

    #[test]
    fn send_with_every_buffer_busy() {
        let (frame, _) = status_frame();
        let mut can = chip(&[read(0x2D, &[0x00]), ask(0xA0, &[0x54])]);
        assert_eq!(can.send(&frame), Err(Error::TxBusy));
        can.release().done();
    }

    #[test]
    fn send_while_off_the_bus() {
        let (frame, _) = status_frame();
        let mut can = chip(&[read(0x2D, &[0x20])]); // TXBO
        assert_eq!(can.send(&frame), Err(Error::BusOff));
        can.release().done();
    }

    #[test]
    fn receive_the_older_frame_first() {
        // Two DRIVE_CMDs (0x100, 8 bytes), one in each buffer.
        let older = [0x20, 0x00, 0, 0, 8, 1, 2, 3, 4, 5, 6, 7, 8];
        let newer = [0x20, 0x00, 0, 0, 8, 1, 2, 3, 4, 5, 6, 7, 9];
        let (first, second) = (Frame::from_registers(&older), Frame::from_registers(&newer));
        let mut can = chip(&[
            ask(0xA0, &[0x03]), // both full: RXB0 first
            ask(0x90, &older),
            ask(0xA0, &[0x02]),
            ask(0x94, &newer),
            ask(0xA0, &[0x00]),
        ]);
        assert_eq!(can.receive(), Ok(Some(first)));
        assert_eq!(can.receive(), Ok(Some(second)));
        assert_eq!(can.receive(), Ok(None));
        assert_eq!(
            first.standard_data(),
            Some((0x100, &[1, 2, 3, 4, 5, 6, 7, 8][..]))
        );
        can.release().done();
    }

    #[test]
    fn bus_status_reads_the_counters_and_the_state() {
        // (TEC, REC, CANSTAT), EFLG, and what that means.
        let cases = [
            ([0, 0, 0x00], 0x00, ErrorState::Active, Some(Mode::Normal)),
            (
                [100, 5, 0x00],
                0x05,
                ErrorState::Warning,
                Some(Mode::Normal),
            ), // EWARN, TXWAR
            (
                [130, 0, 0x00],
                0x15,
                ErrorState::Passive,
                Some(Mode::Normal),
            ), // and TXEP
            (
                [0, 135, 0x60],
                0x0B,
                ErrorState::Passive,
                Some(Mode::ListenOnly),
            ), // RXEP
            ([255, 0, 0x00], 0x35, ErrorState::BusOff, Some(Mode::Normal)), // and TXBO
            ([0, 0, 0x80], 0x00, ErrorState::Active, None),                 // it reset itself
        ];
        for (counters, flags, state, mode) in cases {
            let mut can = chip(&[read(0x1C, &counters), read(0x2D, &[flags])]);
            let status = can.bus_status().unwrap();
            assert_eq!(
                status,
                BusStatus {
                    state,
                    tx_errors: counters[0],
                    rx_errors: counters[1],
                    rx_overflow: false,
                    mode,
                }
            );
            can.release().done();
        }
    }

    #[test]
    fn a_lost_frame_is_reported_once() {
        let mut can = chip(&[
            read(0x1C, &[0, 0, 0x00]),
            read(0x2D, &[0x80]),                // RX1OVR
            command(&[0x05, 0x2D, 0xC0, 0x00]), // BIT MODIFY: clear both overflow flags
            read(0x1C, &[0, 0, 0x00]),
            read(0x2D, &[0x00]),
        ]);
        assert!(can.bus_status().unwrap().rx_overflow);
        assert!(!can.bus_status().unwrap().rx_overflow);
        can.release().done();
    }

    /// An SPI bus whose every transfer fails.
    struct Broken;

    impl ErrorType for Broken {
        type Error = ErrorKind;
    }

    impl SpiDevice for Broken {
        fn transaction(&mut self, _: &mut [Operation<'_, u8>]) -> Result<(), ErrorKind> {
            Err(ErrorKind::Other)
        }
    }

    #[test]
    fn spi_errors_come_back() {
        let mut can = Mcp2515::new(Broken);
        let spi = Err(Error::Spi(ErrorKind::Other));
        let config = spines(Crystal::MHZ_16, Mode::Normal);
        assert_eq!(can.init(&config, &mut NoopDelay::new()), spi);
        assert_eq!(can.send(&status_frame().0), spi);
        assert_eq!(can.receive(), Err(Error::Spi(ErrorKind::Other)));
        assert_eq!(can.bus_status(), Err(Error::Spi(ErrorKind::Other)));
    }

    #[test]
    fn errors_say_what_to_check() {
        let e: Error<ErrorKind> = Error::NoChip;
        assert_eq!(std::format!("{e}"), "no MCP2515 answers on this SPI bus");
    }
}

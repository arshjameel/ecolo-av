//! Which addresses answer on the I2C bus: a few each tick, and only while not armed.
//!
//! For the bench (the ESP32's `/scan` page): it tells a missing or misaddressed device
//! from a dead one. Each probe is an address with no data (a zero-length write): the
//! device acknowledges or not, and nothing in it changes. The board runs it after the
//! tick's own transfers, so the tick's writes always come first.

use embedded_hal::i2c::I2c;

/// The first and last addresses probed: 0x00 (the general call) is never probed, and
/// 0x78..0x7F are reserved for 10-bit addressing. The Jrks sit at 0x04 and 0x05, inside
/// the range the I2C spec reserves, so 0x01..0x07 are probed too.
pub const FIRST: u8 = 0x01;
/// The last address probed.
pub const LAST: u8 = 0x77;

/// What a scan is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanState {
    /// Never asked for since boot.
    Idle,
    /// Probing; `next` is the next address.
    Running {
        /// The next address to probe.
        next: u8,
    },
    /// Waiting: the spine is armed. It goes on from `next` once disarmed.
    Paused {
        /// The address it goes on from.
        next: u8,
    },
    /// Every address probed.
    Done,
}

/// One scan of the bus, a few addresses at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusScan {
    state: ScanState,
    /// Bit `a` set: address `a` acknowledged in this scan.
    found: u128,
}

impl BusScan {
    /// Not scanning; nothing found.
    #[must_use]
    pub const fn new() -> Self {
        BusScan {
            state: ScanState::Idle,
            found: 0,
        }
    }

    /// Start again from the first address, unless a scan is already going.
    pub fn start(&mut self) {
        if !matches!(
            self.state,
            ScanState::Running { .. } | ScanState::Paused { .. }
        ) {
            self.state = ScanState::Running { next: FIRST };
            self.found = 0;
        }
    }

    /// Probe up to `per_step` addresses on `i2c`, if `armed` is false. While armed it
    /// probes nothing and waits.
    pub fn step<I: I2c>(&mut self, i2c: &mut I, armed: bool, per_step: usize) {
        let next = match self.state {
            ScanState::Running { next } | ScanState::Paused { next } => next,
            ScanState::Idle | ScanState::Done => return,
        };
        if armed {
            self.state = ScanState::Paused { next };
            return;
        }
        let end = next.saturating_add(u8::try_from(per_step).unwrap_or(u8::MAX));
        for address in (next..end).take_while(|&a| a <= LAST) {
            if i2c.write(address, &[]).is_ok() {
                self.found |= 1 << address;
            }
        }
        self.state = if end > LAST {
            ScanState::Done
        } else {
            ScanState::Running { next: end }
        };
    }

    /// Where the scan is.
    #[must_use]
    pub fn state(&self) -> ScanState {
        self.state
    }

    /// The addresses that acknowledged so far, lowest first.
    pub fn found(&self) -> impl Iterator<Item = u8> + '_ {
        (FIRST..=LAST).filter(|&a| self.found & (1 << a) != 0)
    }
}

impl Default for BusScan {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};
    use std::vec::Vec;

    /// A bus where only some addresses answer; it records every probe.
    struct Bus {
        present: Vec<u8>,
        probes: Vec<(u8, usize)>,
    }

    #[derive(Debug)]
    struct Nack;

    impl embedded_hal::i2c::Error for Nack {
        fn kind(&self) -> ErrorKind {
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
        }
    }

    impl ErrorType for Bus {
        type Error = Nack;
    }

    impl I2c for Bus {
        fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), Nack> {
            let bytes = ops
                .iter()
                .map(|op| match op {
                    Operation::Write(b) => b.len(),
                    Operation::Read(b) => b.len(),
                })
                .sum();
            self.probes.push((address, bytes));
            if self.present.contains(&address) {
                Ok(())
            } else {
                Err(Nack)
            }
        }
    }

    fn bus() -> Bus {
        Bus {
            present: std::vec![0x04, 0x05, 0x0C, 0x20],
            probes: Vec::new(),
        }
    }

    #[test]
    fn it_finds_todays_four_devices_four_addresses_a_tick() {
        let mut b = bus();
        let mut s = BusScan::new();
        assert_eq!(s.state(), ScanState::Idle);
        s.step(&mut b, false, 4);
        assert!(b.probes.is_empty(), "nothing until asked");
        s.start();
        let mut ticks = 0;
        while s.state() != ScanState::Done {
            s.step(&mut b, false, 4);
            ticks += 1;
            assert!(ticks < 100);
        }
        assert_eq!(ticks, 30); // 0x01..=0x77 is 119 addresses
        assert_eq!(s.found().collect::<Vec<_>>(), [0x04, 0x05, 0x0C, 0x20]);
        let addresses: Vec<u8> = b.probes.iter().map(|p| p.0).collect();
        assert_eq!(addresses, (FIRST..=LAST).collect::<Vec<_>>());
        assert!(b.probes.iter().all(|p| p.1 == 0), "no data, ever");
    }

    #[test]
    fn armed_it_waits_and_goes_on_where_it_was() {
        let mut b = bus();
        let mut s = BusScan::new();
        s.start();
        s.step(&mut b, false, 4);
        s.step(&mut b, true, 4);
        assert_eq!(s.state(), ScanState::Paused { next: 5 });
        assert_eq!(b.probes.len(), 4);
        s.start(); // already going: no restart
        s.step(&mut b, false, 4);
        assert_eq!(s.state(), ScanState::Running { next: 9 });
        assert_eq!(s.found().collect::<Vec<_>>(), [0x04, 0x05]);
    }

    #[test]
    fn start_after_done_starts_again() {
        let mut b = bus();
        let mut s = BusScan::new();
        s.start();
        s.step(&mut b, false, 200);
        assert_eq!(s.state(), ScanState::Done);
        s.start();
        assert_eq!(s.state(), ScanState::Running { next: FIRST });
        assert_eq!(s.found().count(), 0);
    }
}

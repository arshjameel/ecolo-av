//! Which of the car's four I2C devices answer, once at boot: for the bench's first look.
//!
//! Read only. Each device gets a probe with no data (a zero-length write: it acknowledges
//! or not, and nothing in it changes), as [`BusScan`](crate::BusScan) does. Each Jrk that
//! answers is then asked its feedback mode, a RAM setting (`JrkG2::feedback_mode`): that
//! read clears nothing. The error registers are never read here: reading them clears
//! them (0xB3, 0xB5).
//!
//! Four probes and two one-byte reads: about 1 ms at 100 kHz, well inside the watchdog.

use crate::calibration::Addresses;
use embedded_hal::i2c::I2c;
use jrk_g2::{FeedbackMode, JrkG2};

/// One of the car's I2C devices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Device {
    /// The steering Jrk.
    SteeringJrk,
    /// The brake Jrk.
    BrakeJrk,
    /// The throttle DAC.
    Dac,
    /// The I/O expander.
    Expander,
}

impl Device {
    /// In the order a line lists them.
    pub const ALL: [Device; 4] = [
        Device::SteeringJrk,
        Device::BrakeJrk,
        Device::Dac,
        Device::Expander,
    ];

    /// Its name in a line: `steering`, `brake`, `dac`, `expander`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Device::SteeringJrk => "steering",
            Device::BrakeJrk => "brake",
            Device::Dac => "dac",
            Device::Expander => "expander",
        }
    }

    /// Its address in `a`.
    #[must_use]
    pub const fn address(self, a: &Addresses) -> u8 {
        match self {
            Device::SteeringJrk => a.steering,
            Device::BrakeJrk => a.brake,
            Device::Dac => a.dac,
            Device::Expander => a.expander,
        }
    }

    /// One of the two Jrks.
    #[must_use]
    pub const fn is_jrk(self) -> bool {
        matches!(self, Device::SteeringJrk | Device::BrakeJrk)
    }
}

/// What one device did at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Found {
    /// Which device.
    pub device: Device,
    /// The address it was looked for at.
    pub address: u8,
    /// It answered.
    pub answered: bool,
    /// A Jrk that answered: its feedback mode, or `None` if that read failed. Always
    /// `None` for the DAC and the expander.
    pub feedback: Option<FeedbackMode>,
}

/// The four devices, as they answered at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceCheck {
    found: [Found; 4],
}

impl DeviceCheck {
    /// Probe the four devices at `addresses` on `i2c`. Read only (the module docs).
    pub fn run<I: I2c>(i2c: &mut I, addresses: &Addresses) -> Self {
        let found = Device::ALL.map(|device| {
            let address = device.address(addresses);
            let answered = i2c.write(address, &[]).is_ok();
            let feedback = if answered && device.is_jrk() {
                JrkG2::new(&mut *i2c, address).feedback_mode().ok()
            } else {
                None
            };
            Found {
                device,
                address,
                answered,
                feedback,
            }
        });
        DeviceCheck { found }
    }

    /// Each device, in [`Device::ALL`]'s order.
    pub fn iter(&self) -> impl Iterator<Item = &Found> {
        self.found.iter()
    }

    /// How many answered, of [`Self::expected`].
    #[must_use]
    pub fn answered(&self) -> usize {
        self.found.iter().filter(|f| f.answered).count()
    }

    /// How many devices were looked for.
    #[must_use]
    pub const fn expected(&self) -> usize {
        self.found.len()
    }

    /// The devices that didn't answer.
    pub fn missing(&self) -> impl Iterator<Item = Device> + '_ {
        self.found.iter().filter(|f| !f.answered).map(|f| f.device)
    }
}

/// A feedback mode's name in a line: `none`, `analog`, `frequency`, `other`.
#[must_use]
pub const fn feedback_name(mode: FeedbackMode) -> &'static str {
    match mode {
        FeedbackMode::None => "none",
        FeedbackMode::Analog => "analog",
        FeedbackMode::Frequency => "frequency",
        FeedbackMode::Other(_) => "other",
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::calibration::Calibration;
    use crate::stand_in::StandInBus;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;
    use std::vec::Vec;

    const A: Addresses = Calibration::TODAY.addresses;

    #[test]
    fn all_four_answer_and_the_jrks_say_their_feedback() {
        let expected = [
            T::write(0x05, vec![]),
            T::write_read(0x05, vec![0xEA, 0x18], vec![0x01]),
            T::write(0x04, vec![]),
            T::write_read(0x04, vec![0xEA, 0x18], vec![0x00]),
            T::write(0x0C, vec![]),
            T::write(0x20, vec![]),
        ];
        let mut bus = Mock::new(&expected);
        let check = DeviceCheck::run(&mut bus, &A);
        bus.done(); // exactly these transfers: no error register is read
        assert_eq!((check.answered(), check.expected()), (4, 4));
        let modes: Vec<_> = check.iter().map(|f| f.feedback).collect();
        assert_eq!(
            modes,
            [
                Some(FeedbackMode::Analog),
                Some(FeedbackMode::None),
                None,
                None
            ]
        );
        assert_eq!(check.missing().count(), 0);
    }

    #[test]
    fn a_missing_device_is_missing_and_a_missing_jrk_is_not_asked_more() {
        let nack = ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address);
        let expected = [
            T::write(0x05, vec![]).with_error(nack),
            T::write(0x04, vec![]),
            T::write_read(0x04, vec![0xEA, 0x18], vec![0x01]).with_error(ErrorKind::Other),
            T::write(0x0C, vec![]).with_error(nack),
            T::write(0x20, vec![]),
        ];
        let mut bus = Mock::new(&expected);
        let check = DeviceCheck::run(&mut bus, &A);
        bus.done();
        assert_eq!(check.answered(), 2);
        assert_eq!(
            check.missing().collect::<Vec<_>>(),
            [Device::SteeringJrk, Device::Dac]
        );
        let brake = check.iter().nth(1).unwrap();
        assert!(brake.answered);
        assert_eq!(brake.feedback, None, "answered, but its mode is unknown");
    }

    #[test]
    fn the_stand_in_bus_answers_for_all_four() {
        let mut bus = StandInBus::new(&Calibration::TODAY);
        let check = DeviceCheck::run(&mut bus, &A);
        assert_eq!(check.answered(), 4);
        assert_eq!(
            check.iter().next().unwrap().feedback,
            Some(FeedbackMode::Analog)
        );
        assert_eq!(bus.devices(), Default::default(), "nothing was written");
    }

    #[test]
    fn names() {
        let names: Vec<_> = Device::ALL.iter().map(|d| d.name()).collect();
        assert_eq!(names, ["steering", "brake", "dac", "expander"]);
        assert_eq!(feedback_name(FeedbackMode::Analog), "analog");
        assert_eq!(feedback_name(FeedbackMode::Other(9)), "other");
        let addresses: Vec<_> = Device::ALL.iter().map(|d| d.address(&A)).collect();
        assert_eq!(addresses, [0x05, 0x04, 0x0C, 0x20]);
    }
}

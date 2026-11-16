//! `/vehicle/status` (aav_interfaces/msg/VehicleStatus) in plain words.
//!
//! The numbers match the message's constants; a test in the ROS part of hud-app checks
//! them against the generated message code.

use std::fmt;

use crate::units::{Fraction, MetresPerSecond, Radians};

/// Slower than this counts as standing still, for the gear letter (as aav_hud's).
pub const MOVING: MetresPerSecond = MetresPerSecond(0.05);

/// Who drives the actuators.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// A person drives; the actuators are idle.
    #[default]
    Manual,
    /// The RC transmitter overrides the computer.
    Rc,
    /// The spine follows the computer's commands.
    Computer,
    /// The hardware test mode: one actuator at a time, from a laptop.
    Test,
    /// A number this HUD doesn't know: a newer message, or a bug.
    Unknown(u8),
}

impl From<u8> for Mode {
    fn from(code: u8) -> Self {
        match code {
            0 => Self::Manual,
            1 => Self::Rc,
            2 => Self::Computer,
            3 => Self::Test,
            other => Self::Unknown(other),
        }
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Manual => f.write_str("MANUAL"),
            Self::Rc => f.write_str("RC"),
            Self::Computer => f.write_str("COMPUTER"),
            Self::Test => f.write_str("TEST"),
            Self::Unknown(code) => write!(f, "MODE {code}?"),
        }
    }
}

/// The letter on the screen. The car has no gearbox: this says which way it goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gear {
    /// Moving forward, or the computer may move it.
    Drive,
    /// Moving backward.
    Reverse,
    /// Standing still, and nothing may move it.
    Park,
}

impl Gear {
    /// Moving: the direction. Standing still: Drive only when the computer drives and
    /// may move (armed, no e-stop), otherwise Park. The same rule as aav_hud's.
    #[must_use]
    pub fn pick(mode: Mode, armed: bool, estop: bool, speed: MetresPerSecond) -> Self {
        if speed.0 < -MOVING.0 {
            Self::Reverse
        } else if speed.0 > MOVING.0 || (mode == Mode::Computer && armed && !estop) {
            Self::Drive
        } else {
            Self::Park
        }
    }

    /// The one letter: D, R or P.
    #[must_use]
    pub fn letter(self) -> &'static str {
        match self {
            Self::Drive => "D",
            Self::Reverse => "R",
            Self::Park => "P",
        }
    }
}

/// One fault the spine reports, a bit of VehicleStatus's `faults`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    CmdTimeout,
    BadChecksum,
    BadCounter,
    SteeringDriver,
    BrakeDriver,
    ThrottleDac,
    WheelEncoder,
    CanBus,
    RcSignalLost,
    Overspeed,
}

impl Fault {
    /// Every fault, in bit order.
    pub const ALL: [Fault; 10] = [
        Fault::CmdTimeout,
        Fault::BadChecksum,
        Fault::BadCounter,
        Fault::SteeringDriver,
        Fault::BrakeDriver,
        Fault::ThrottleDac,
        Fault::WheelEncoder,
        Fault::CanBus,
        Fault::RcSignalLost,
        Fault::Overspeed,
    ];

    /// Its bit in `faults` (VehicleStatus's FAULT_* constants).
    #[must_use]
    pub fn bit(self) -> u16 {
        1 << (self as u16)
    }

    /// A short name for the screen.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::CmdTimeout => "no commands",
            Self::BadChecksum => "bad checksum",
            Self::BadCounter => "bad counter",
            Self::SteeringDriver => "steering driver",
            Self::BrakeDriver => "brake driver",
            Self::ThrottleDac => "throttle DAC",
            Self::WheelEncoder => "wheel encoder",
            Self::CanBus => "CAN bus",
            Self::RcSignalLost => "RC signal lost",
            Self::Overspeed => "overspeed",
        }
    }
}

/// The fault bits, as a set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Faults(pub u16);

impl Faults {
    /// No fault at all.
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// The faults that are set, in bit order.
    pub fn iter(self) -> impl Iterator<Item = Fault> {
        Fault::ALL
            .into_iter()
            .filter(move |f| self.0 & f.bit() != 0)
    }

    /// Bits that no [`Fault`] names (a newer firmware), or 0.
    #[must_use]
    pub fn unknown_bits(self) -> u16 {
        let known = Fault::ALL.iter().fold(0, |bits, f| bits | f.bit());
        self.0 & !known
    }
}

impl fmt::Display for Faults {
    /// "none", or the names joined by commas, with any unknown bits in hex at the end.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_empty() {
            return f.write_str("none");
        }
        let mut names: Vec<String> = self.iter().map(|x| x.name().to_owned()).collect();
        if self.unknown_bits() != 0 {
            names.push(format!("0x{:04x}", self.unknown_bits()));
        }
        f.write_str(&names.join(", "))
    }
}

/// One `/vehicle/status` message, in our types.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VehicleStatus {
    pub mode: Mode,
    pub armed: bool,
    /// Any stop: the hardware button, `/safety/estop`, or the spine's watchdog.
    pub estop_active: bool,
    pub speed: MetresPerSecond,
    pub steering_angle: Radians,
    pub throttle: Fraction,
    pub brake: Fraction,
    pub faults: Faults,
}

impl VehicleStatus {
    /// The gear letter for this status.
    #[must_use]
    pub fn gear(&self) -> Gear {
        Gear::pick(self.mode, self.armed, self.estop_active, self.speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_from_code() {
        assert_eq!(Mode::from(0), Mode::Manual);
        assert_eq!(Mode::from(1), Mode::Rc);
        assert_eq!(Mode::from(2), Mode::Computer);
        assert_eq!(Mode::from(3), Mode::Test);
        assert_eq!(Mode::from(9), Mode::Unknown(9));
        assert_eq!(Mode::Unknown(9).to_string(), "MODE 9?");
    }

    #[test]
    fn gear_follows_motion_then_permission() {
        let still = MetresPerSecond(0.0);
        assert_eq!(
            Gear::pick(Mode::Manual, false, false, MetresPerSecond(-1.0)),
            Gear::Reverse
        );
        assert_eq!(
            Gear::pick(Mode::Manual, false, false, MetresPerSecond(1.0)),
            Gear::Drive
        );
        assert_eq!(Gear::pick(Mode::Computer, true, false, still), Gear::Drive);
        assert_eq!(Gear::pick(Mode::Computer, true, true, still), Gear::Park);
        assert_eq!(Gear::pick(Mode::Computer, false, false, still), Gear::Park);
        assert_eq!(Gear::pick(Mode::Rc, true, false, still), Gear::Park);
        // Creeping slower than MOVING counts as standing still.
        assert_eq!(
            Gear::pick(Mode::Manual, false, false, MetresPerSecond(0.04)),
            Gear::Park
        );
    }

    #[test]
    fn fault_bits_match_the_message() {
        // aav_interfaces/msg/VehicleStatus: FAULT_CMD_TIMEOUT=1 ... FAULT_OVERSPEED=512.
        let bits: Vec<u16> = Fault::ALL.iter().map(|f| f.bit()).collect();
        assert_eq!(bits, [1, 2, 4, 8, 16, 32, 64, 128, 256, 512]);
    }

    #[test]
    fn faults_are_named() {
        assert_eq!(Faults(0).to_string(), "none");
        assert_eq!(Faults(8 | 16).to_string(), "steering driver, brake driver");
        assert_eq!(Faults(1 | 0x8000).to_string(), "no commands, 0x8000");
        assert_eq!(Faults(0x8000).unknown_bits(), 0x8000);
    }
}

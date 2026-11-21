//! The 2025-26 C++'s log lines, in its words, beside our own: so a person who knows the
//! C++'s log can read this one (`docs/esp32-cpp-parity.md`).
//!
//! | Line | The C++ |
//! |---|---|
//! | [`boot_devices`]: `PCA9555 (0x20) ready`, `DAC (0x0c) OK`, `JRK STEER (0x05) OK - centred (1950)`, `JRK BRAKE (0x04) OK - engaged (3500)`, or a `WARNING: ... not found` | `setup()` in `main.cpp` |
//! | [`RcReady`]: `Ready - CH6 mode: RC direct above 1500 \| CH3 throttle: ...` | `setup()`'s last line |
//! | [`CppStatus`]: `STATUS:0.00,0.00,-1.00 \| STR:OK BRK:OK` | `publishStatus()`, every 200 ms |
//! | [`FaultChange`]: `FAULT t=... set=THROTTLE_DAC cleared=none faults=0x0020` | `ERROR: DAC did not acknowledge!`, `RC: No valid signal - SAFE STOP`, `JRK SAFE STOP` |
//! | [`JrkErrorLine`]: `JRK STEER ERR t=... flags=0x0004` | `JRK STEER ERR: 0x%04X` in `setSteer()`, `setBrake()` |
//! | [`ScanFound`]: `Found: 0x05 <- JRK steer` | `handleScan()` |
//!
//! Log only: everything here is made from what the tick already has, and drives nothing.
//! No line starts with `STATUS t=`, `EVENT t=`, `LINK t=` or `DEVICES t=`: those are the
//! lines `tools/esp32_qemu.py` and the first check read.

use crate::calibration::{Addresses, Calibration};
use crate::probe::{Device, DeviceCheck, Found};
use core::fmt::{self, Display, Formatter};
use ibus::RcMap;
use jrk_g2::ErrorFlags;
use spine_core::{faults, Outputs};

/// The order `setup()` brought the devices up in, and printed them.
const CPP_ORDER: [Device; 4] = [
    Device::Expander,
    Device::Dac,
    Device::SteeringJrk,
    Device::BrakeJrk,
];

/// One device's boot line, as `setup()` printed it.
#[derive(Debug, Clone, Copy)]
pub struct BootDevice<'a> {
    found: &'a Found,
    cal: &'a Calibration,
}

/// The four devices' boot lines, in the C++'s order. `init` has already put the steering
/// at its centre and the brake full on: the values the lines say.
pub fn boot_devices<'a>(
    check: &'a DeviceCheck,
    cal: &'a Calibration,
) -> impl Iterator<Item = BootDevice<'a>> {
    CPP_ORDER.into_iter().filter_map(move |device| {
        check
            .iter()
            .find(|f| f.device == device)
            .map(|found| BootDevice { found, cal })
    })
}

impl Display for BootDevice<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let (a, ok) = (self.found.address, self.found.answered);
        match (self.found.device, ok) {
            (Device::SteeringJrk, true) => write!(
                f,
                "JRK STEER ({a:#04x}) OK - centred ({})",
                self.cal.steering.centre
            ),
            (Device::SteeringJrk, false) => write!(f, "WARNING: JRK STEER ({a:#04x}) not found!"),
            (Device::BrakeJrk, true) => write!(
                f,
                "JRK BRAKE ({a:#04x}) OK - engaged ({})",
                self.cal.brake.full
            ),
            (Device::BrakeJrk, false) => write!(f, "WARNING: JRK BRAKE ({a:#04x}) not found!"),
            (Device::Dac, true) => write!(f, "DAC ({a:#04x}) OK"),
            (Device::Dac, false) => write!(f, "WARNING: DAC ({a:#04x}) not found - no throttle"),
            (Device::Expander, true) => write!(f, "PCA9555 ({a:#04x}) ready"),
            (Device::Expander, false) => write!(
                f,
                "WARNING: PCA9555 ({a:#04x}) not found - no throttle (the direction pin is on it)"
            ),
        }
    }
}

/// `setup()`'s last line, for the RC switches as this build reads them. The C++ printed
/// `Ready — SwA: RC_EN | SwC: TOP=Brake, MID=Fwd, LOW=Rev`; this says the channels and the
/// thresholds the code uses (`ibus::RcMap`), counted from 1 as the transmitter does.
#[derive(Debug, Clone, Copy)]
pub struct RcReady<'a>(pub &'a RcMap);

impl Display for RcReady<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let m = self.0;
        write!(
            f,
            "Ready - CH{} mode: RC direct above {}",
            m.mode_switch + 1,
            m.direct_above
        )?;
        if let Some(via) = m.via_ros_above {
            write!(f, ", RC via ROS above {via}")?;
        }
        write!(
            f,
            " | CH{} throttle: brake band {}% each way | CH{} steering",
            m.throttle + 1,
            (m.brake_band * 100.0) as u32,
            m.steering + 1
        )
    }
}

/// `publishStatus()`'s line: `STATUS:<steering>,<speed>,<accel> | STR:<OK|FAIL> BRK:<OK|FAIL>`.
///
/// The C++ printed the values it was given (the Pi's frame, or the sticks). This prints
/// what the spine decided, on the same scales: steering -1..1 of full lock (positive
/// left), speed the throttle -1..1 (negative in reverse), accel the brake as -1..0. `OK`:
/// the Jrk answered at boot and has no fault now (the C++: it answered at boot).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CppStatus {
    /// -1..1 of full lock, positive left.
    pub steering: f32,
    /// The throttle, -1..1, negative in reverse.
    pub speed: f32,
    /// The brake, -1..0.
    pub accel: f32,
    /// The steering Jrk answered at boot and has no fault now.
    pub steer_ok: bool,
    /// The brake Jrk answered at boot and has no fault now.
    pub brake_ok: bool,
}

impl CppStatus {
    /// The line for this tick's outputs.
    #[must_use]
    pub fn new(out: &Outputs, max_steering_angle: f32, check: &DeviceCheck) -> Self {
        let answered = |device| check.iter().any(|f| f.device == device && f.answered);
        let steering = if max_steering_angle > 0.0 {
            (out.steering_target / max_steering_angle).clamp(-1.0, 1.0)
        } else {
            0.0
        };
        // `+ 0.0` and `0.0 -` turn -0.0 into 0.0, so the line never says -0.00.
        CppStatus {
            steering: steering + 0.0,
            speed: if out.reverse {
                0.0 - out.throttle
            } else {
                out.throttle + 0.0
            },
            accel: 0.0 - out.brake,
            steer_ok: answered(Device::SteeringJrk) && out.faults & faults::STEERING_DRIVER == 0,
            brake_ok: answered(Device::BrakeJrk) && out.faults & faults::BRAKE_DRIVER == 0,
        }
    }
}

impl Display for CppStatus {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let ok = |b: bool| if b { "OK" } else { "FAIL" };
        write!(
            f,
            "STATUS:{:.2},{:.2},{:.2} | STR:{} BRK:{}",
            self.steering,
            self.speed,
            self.accel,
            ok(self.steer_ok),
            ok(self.brake_ok)
        )
    }
}

/// A change in the spine's fault bits, by name: `FAULT t=... set=A,B cleared=C faults=0x....`.
/// The C++ printed one line for each kind (`ERROR: DAC did not acknowledge!`, `RC: No
/// valid signal - SAFE STOP`, `JRK SAFE STOP`); this one line covers them all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultChange {
    /// When (ms since boot).
    pub now_ms: u32,
    /// The fault bits before.
    pub was: u16,
    /// The fault bits now.
    pub now: u16,
}

impl Display for FaultChange {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "FAULT t={} set=", self.now_ms)?;
        names(f, self.now & !self.was)?;
        f.write_str(" cleared=")?;
        names(f, self.was & !self.now)?;
        write!(f, " faults={:#06x}", self.now)
    }
}

/// `A,B`, or `none`.
fn names(f: &mut Formatter<'_>, bits: u16) -> fmt::Result {
    let mut any = false;
    for name in faults::names(bits) {
        if any {
            f.write_str(",")?;
        }
        f.write_str(name)?;
        any = true;
    }
    if !any {
        f.write_str("none")?;
    }
    Ok(())
}

/// Which Jrk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Jrk {
    /// The steering Jrk.
    Steering,
    /// The brake Jrk.
    Brake,
}

/// A Jrk's error flags when they change (`Actuators::jrk_errors`): `JRK STEER ERR t=...
/// flags=0x0004`, `cleared` when they go back to none, `unread` when the read failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JrkErrorLine {
    /// Which Jrk.
    pub jrk: Jrk,
    /// When (ms since boot).
    pub now_ms: u32,
    /// Its flags now; `None` if the read failed.
    pub flags: Option<ErrorFlags>,
}

impl Display for JrkErrorLine {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let name = match self.jrk {
            Jrk::Steering => "STEER",
            Jrk::Brake => "BRAKE",
        };
        write!(f, "JRK {name} ERR t={} ", self.now_ms)?;
        match self.flags {
            Some(ErrorFlags(0)) => f.write_str("cleared"),
            Some(ErrorFlags(bits)) => write!(f, "flags={bits:#06x}"),
            None => f.write_str("unread (the read failed)"),
        }
    }
}

/// The name `/scan` gives an address, as `handleScan()` labelled it; `None` for any other.
#[must_use]
pub const fn scan_label(address: u8, a: &Addresses) -> Option<&'static str> {
    if address == a.steering {
        Some("JRK steer")
    } else if address == a.brake {
        Some("JRK brake")
    } else if address == a.expander {
        Some("PCA9555")
    } else if address == a.dac {
        Some("DAC (A0=GND)")
    } else {
        None
    }
}

/// One address a scan found, as `handleScan()` printed it: `Found: 0x05 <- JRK steer`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanFound {
    /// The address that answered.
    pub address: u8,
    /// Its name ([`scan_label`]), if it is one of ours.
    pub label: Option<&'static str>,
}

impl Display for ScanFound {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "Found: {:#04x}", self.address)?;
        match self.label {
            Some(label) => write!(f, " <- {label}"),
            None => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::stand_in::StandInBus;
    use spine_core::protocol::Mode;
    use std::string::{String, ToString};
    use std::vec::Vec;

    const CAL: Calibration = Calibration::TODAY;

    fn all_answer() -> DeviceCheck {
        DeviceCheck::run(&mut StandInBus::new(&CAL), &CAL.addresses)
    }

    fn none_answer() -> DeviceCheck {
        let other = Addresses {
            steering: 0x60,
            brake: 0x61,
            dac: 0x62,
            expander: 0x63,
        };
        DeviceCheck::run(&mut StandInBus::new(&CAL), &other)
    }

    fn out(throttle: f32, brake: f32, reverse: bool, steering: f32, faults: u16) -> Outputs {
        Outputs {
            mode: Mode::Manual,
            throttle,
            brake,
            reverse,
            steering_target: steering,
            armed: false,
            estop_active: false,
            faults,
        }
    }

    #[test]
    fn boot_lines_in_the_cpps_order_and_words() {
        let lines: Vec<String> = boot_devices(&all_answer(), &CAL)
            .map(|l| l.to_string())
            .collect();
        assert_eq!(
            lines,
            [
                "PCA9555 (0x20) ready",
                "DAC (0x0c) OK",
                "JRK STEER (0x05) OK - centred (1950)",
                "JRK BRAKE (0x04) OK - engaged (3500)",
            ]
        );
        let check = none_answer();
        let lines: Vec<String> = boot_devices(&check, &CAL).map(|l| l.to_string()).collect();
        assert!(
            lines.iter().all(|l| l.starts_with("WARNING: ")),
            "{lines:?}"
        );
        assert!(lines[1].ends_with("not found - no throttle"));
        assert_eq!(lines[2], "WARNING: JRK STEER (0x60) not found!");
    }

    #[test]
    fn the_ready_line_says_the_channels_from_one() {
        assert_eq!(
            RcReady(&RcMap::TODAY).to_string(),
            "Ready - CH6 mode: RC direct above 1500 | CH3 throttle: brake band 30% each way \
             | CH4 steering"
        );
        assert!(RcReady(&RcMap::THREE_POSITION)
            .to_string()
            .starts_with("Ready - CH6 mode: RC direct above 1700, RC via ROS above 1300 |"));
    }

    #[test]
    fn the_status_line_reads_as_the_cpps() {
        let check = all_answer();
        let parked = CppStatus::new(&out(0.0, 1.0, false, 0.0, 0), 0.5, &check);
        assert_eq!(parked.to_string(), "STATUS:0.00,0.00,-1.00 | STR:OK BRK:OK");
        let driving = CppStatus::new(&out(0.3, 0.0, true, -0.25, 0), 0.5, &check);
        assert_eq!(
            driving.to_string(),
            "STATUS:-0.50,-0.30,0.00 | STR:OK BRK:OK"
        );
        let faulted = CppStatus::new(
            &out(0.0, 1.0, false, 0.0, faults::BRAKE_DRIVER),
            0.5,
            &check,
        );
        assert!(faulted.to_string().ends_with("STR:OK BRK:FAIL"));
        let missing = CppStatus::new(&out(0.0, 1.0, false, -0.0, 0), 0.5, &none_answer());
        assert_eq!(
            missing.to_string(),
            "STATUS:0.00,0.00,-1.00 | STR:FAIL BRK:FAIL"
        );
        // Never a line the QEMU check or the first check would take for its STATUS.
        assert!(!parked.to_string().contains("STATUS "));
    }

    #[test]
    fn a_fault_change_names_what_came_and_went() {
        let line = FaultChange {
            now_ms: 12,
            was: faults::CMD_TIMEOUT,
            now: faults::THROTTLE_DAC | faults::RC_SIGNAL_LOST,
        };
        assert_eq!(
            line.to_string(),
            "FAULT t=12 set=THROTTLE_DAC,RC_SIGNAL_LOST cleared=CMD_TIMEOUT faults=0x0120"
        );
        let gone = FaultChange {
            now_ms: 20,
            was: faults::THROTTLE_DAC,
            now: 0,
        };
        assert_eq!(
            gone.to_string(),
            "FAULT t=20 set=none cleared=THROTTLE_DAC faults=0x0000"
        );
    }

    #[test]
    fn jrk_error_lines() {
        let line = |flags| {
            JrkErrorLine {
                jrk: Jrk::Steering,
                now_ms: 5,
                flags,
            }
            .to_string()
        };
        assert_eq!(
            line(Some(ErrorFlags(ErrorFlags::MOTOR_DRIVER))),
            "JRK STEER ERR t=5 flags=0x0004"
        );
        assert_eq!(line(Some(ErrorFlags(0))), "JRK STEER ERR t=5 cleared");
        assert_eq!(line(None), "JRK STEER ERR t=5 unread (the read failed)");
    }

    #[test]
    fn scan_labels_are_the_cpps() {
        let a = &CAL.addresses;
        let found = |address| {
            ScanFound {
                address,
                label: scan_label(address, a),
            }
            .to_string()
        };
        assert_eq!(found(0x05), "Found: 0x05 <- JRK steer");
        assert_eq!(found(0x04), "Found: 0x04 <- JRK brake");
        assert_eq!(found(0x0C), "Found: 0x0c <- DAC (A0=GND)");
        assert_eq!(found(0x20), "Found: 0x20 <- PCA9555");
        assert_eq!(found(0x42), "Found: 0x42");
    }
}

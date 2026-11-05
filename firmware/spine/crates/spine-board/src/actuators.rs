//! The car's I2C devices: drive them from `spine_core::Outputs`, and read them for
//! `spine_core::Inputs`.
//!
//! Every tick, every device gets its command, so one that reset gets it back. The
//! order keeps throttle and brake from overlapping, even for the moment between two
//! writes:
//!
//! | Step | Device | Rule |
//! |---|---|---|
//! | 1 | DAC | if the throttle is falling: first, so an e-stop's throttle is 0 before the brake comes on |
//! | 2 | brake Jrk | always |
//! | 3 | expander | the direction changes only while the DAC is at 0 |
//! | 4 | DAC | if the throttle is rising: last, and only if the brake write worked |
//! | 5 | steering Jrk | always |
//!
//! The throttle rises at most `rise_per_s`, and falls at once. It stays at 0 while
//! the brake is on, while the direction changes, and while the expander isn't set up.
//!
//! Each tick it also reads both Jrks: their errors, and where each motor is (their
//! scaled feedback, as the 2024-25 `FinalRev` firmware read the steering with
//! `getScaledFeedback()`). It asks each Jrk once for its feedback mode: a Jrk with no
//! analog feedback set up has no position to give, and its position stays unknown.
//!
//! A failed transfer sets the device's fault bit; `spine-core` then stops the car and
//! waits for the computer to arm again. The expander has no bit of its own: the
//! direction belongs to the throttle, so it uses THROTTLE_DAC.

use crate::calibration::Calibration;
use ad5311::Ad5311;
use embedded_hal::i2c::I2c;
use jrk_g2::{ErrorFlags, JrkG2};
use legacy_link::SteeringDrive;
use pca9555::Pca9555;
use spine_core::{faults, Outputs};

/// The four devices, each with its own handle on the bus. To share one bus, give
/// each a handle from `embedded-hal-bus` or `embassy-embedded-hal`.
#[derive(Debug)]
pub struct Devices<I2C> {
    pub steering: JrkG2<I2C>,
    pub brake: JrkG2<I2C>,
    pub dac: Ad5311<I2C>,
    pub expander: Pca9555<I2C>,
}

impl<I2C: I2c> Devices<I2C> {
    /// The devices at the calibration's addresses. The DAC never writes above the
    /// full-throttle code.
    pub fn new(steering: I2C, brake: I2C, dac: I2C, expander: I2C, cal: &Calibration) -> Self {
        let a = cal.addresses;
        Devices {
            steering: JrkG2::new(steering, a.steering),
            brake: JrkG2::new(brake, a.brake),
            dac: Ad5311::with_limit(dac, a.dac, cal.throttle.full_code),
            expander: Pca9555::new(expander, a.expander),
        }
    }
}

/// Where the motors are, this tick: `None` when unknown (the Jrk has no analog
/// feedback set up, it hasn't said yet, or the read failed).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Positions {
    /// The steering (rad, positive left).
    pub steering: Option<f32>,
    /// The brake (0 released .. 1 full).
    pub brake: Option<f32>,
    /// The same, raw: each Jrk's scaled feedback, 0 to 4095 counts in target units, not
    /// converted with the calibration (for measuring it).
    pub steering_counts: Option<u16>,
    pub brake_counts: Option<u16>,
}

/// What the devices said this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Readings {
    /// The steering angle from the Jrk's scaled feedback (rad, positive left).
    /// If the read failed, the last good value; 0 if there never was one.
    pub steering: f32,
    /// This tick's positions, or unknown.
    pub positions: Positions,
    /// STEERING_DRIVER or BRAKE_DRIVER: a failed read, or a Jrk error.
    pub faults: u16,
}

#[derive(Debug)]
pub struct Actuators<I2C> {
    dev: Devices<I2C>,
    cal: Calibration,
    /// The code last written to the DAC. After a failed write, the higher of the
    /// old and new codes: the DAC holds one of them.
    throttle_code: u16,
    /// The direction last written.
    reverse: bool,
    /// False until the expander's pins are outputs, and after any expander error.
    expander_ready: bool,
    last_ms: Option<u32>,
    steering: f32,
    /// Each Jrk's feedback: a position, or not; `None` until it has said.
    steering_feedback: Option<bool>,
    brake_feedback: Option<bool>,
}

impl<I2C: I2c> Actuators<I2C> {
    pub fn new(devices: Devices<I2C>, cal: Calibration) -> Self {
        Actuators {
            dev: devices,
            cal,
            // Unknown until the first write: assume the worst.
            throttle_code: cal.throttle.full_code,
            reverse: false,
            expander_ready: false,
            last_ms: None,
            steering: 0.0,
            steering_feedback: None,
            brake_feedback: None,
        }
    }

    /// Power-up, as the C++ `setup()` did, but in the same order as every tick: the
    /// throttle to 0, the brake full on, the expander's pins to outputs (forward),
    /// the steering to the centre. Returns the fault bits.
    pub fn init(&mut self) -> u16 {
        self.write_throttle(0)
            | self.write_brake(1.0)
            | self.write_direction(false)
            | self.write_steering(0.0)
    }

    /// Read both Jrks: where each motor is, and their errors. The error read clears
    /// the Jrk's latched errors, as the C++ did; `spine-core` keeps the car stopped
    /// until the computer arms again. A failed position read is the Jrk's fault bit.
    pub fn read(&mut self) -> Readings {
        let mut bits = 0;
        let steering_counts = match position(&mut self.dev.steering, &mut self.steering_feedback) {
            Ok(target) => target,
            Err(()) => {
                bits |= faults::STEERING_DRIVER;
                None
            }
        };
        let steering = steering_counts.map(|t| self.cal.steering.angle(t));
        if let Some(angle) = steering {
            self.steering = angle;
        }
        bits |= jrk_fault(
            self.dev.steering.take_error_flags(),
            faults::STEERING_DRIVER,
        );
        let brake_counts = match position(&mut self.dev.brake, &mut self.brake_feedback) {
            Ok(target) => target,
            Err(()) => {
                bits |= faults::BRAKE_DRIVER;
                None
            }
        };
        let brake = brake_counts.map(|t| self.cal.brake.fraction(t));
        bits |= jrk_fault(self.dev.brake.take_error_flags(), faults::BRAKE_DRIVER);
        Readings {
            steering: self.steering,
            positions: Positions {
                steering,
                brake,
                steering_counts,
                brake_counts,
            },
            faults: bits,
        }
    }

    /// Drive every device from the spine's outputs. Returns the fault bits.
    pub fn apply(&mut self, out: &Outputs, now_ms: u32) -> u16 {
        self.apply_with(out, now_ms, SteeringDrive::Spine)
    }

    /// The same, with the steering actuator driven as `steering` says (the link's
    /// steering calibration: `legacy_link::SteerCal`). Anything but
    /// [`SteeringDrive::Spine`] also holds the DAC at 0. Returns the fault bits.
    pub fn apply_with(&mut self, out: &Outputs, now_ms: u32, steering: SteeringDrive) -> u16 {
        // spine-core never asks for both. If it ever did, the brake would win.
        debug_assert!(
            !(out.throttle > 0.0 && out.brake > 0.0),
            "throttle and brake both on: {out:?}"
        );
        let dt_ms = self.last_ms.map_or(0, |t| now_ms.wrapping_sub(t).min(100));
        self.last_ms = Some(now_ms);

        let turning = out.reverse != self.reverse;
        let mut want = self.cal.throttle.code(out.throttle);
        let calibrating = steering != SteeringDrive::Spine;
        if out.brake > 0.0 || turning || !self.expander_ready || calibrating {
            want = 0;
        }
        let rise = self.cal.throttle.max_rise(dt_ms);
        let want = want.min(self.throttle_code.saturating_add(rise));
        let rising = want > self.throttle_code;

        let mut bits = 0;
        if !rising {
            bits |= self.write_throttle(want);
        }
        let brake_fault = self.write_brake(out.brake);
        bits |= brake_fault;
        bits |= self.write_direction(out.reverse);
        if rising {
            let ok = brake_fault == 0 && self.expander_ready;
            bits |= self.write_throttle(if ok { want } else { self.throttle_code });
        }
        bits | match steering {
            SteeringDrive::Spine => self.write_steering(out.steering_target),
            SteeringDrive::MotorOff => self.stop_steering(),
            SteeringDrive::Target(target) => self.write_steering_target(target),
        }
    }

    /// The code last written to the DAC (or, after a failed write, the higher one).
    pub fn throttle_code(&self) -> u16 {
        self.throttle_code
    }

    /// The direction last written.
    pub fn reverse(&self) -> bool {
        self.reverse
    }

    /// The devices, to read something else from them.
    pub fn devices(&mut self) -> &mut Devices<I2C> {
        &mut self.dev
    }

    pub fn release(self) -> Devices<I2C> {
        self.dev
    }

    /// Levels first, then make the pins outputs, so they never glitch; forward.
    fn setup_expander(&mut self) -> u16 {
        let x = &mut self.dev.expander;
        let ok = x.set_polarity(0).is_ok()
            && x.set_outputs(self.cal.direction.levels(false)).is_ok()
            && x.set_inputs(0).is_ok();
        self.expander_ready = ok;
        if ok {
            self.reverse = false;
            0
        } else {
            faults::THROTTLE_DAC
        }
    }

    /// Set the direction. It changes, and the expander is set up, only while the DAC
    /// is at 0.
    fn write_direction(&mut self, reverse: bool) -> u16 {
        if !self.expander_ready {
            return if self.throttle_code == 0 {
                self.setup_expander()
            } else {
                faults::THROTTLE_DAC
            };
        }
        let reverse = if self.throttle_code == 0 {
            reverse
        } else {
            self.reverse
        };
        let levels = self.cal.direction.levels(reverse);
        if self.dev.expander.set_outputs(levels).is_ok() {
            self.reverse = reverse;
            0
        } else {
            // The pin may be either way now: no throttle until it is set up again.
            self.expander_ready = false;
            faults::THROTTLE_DAC
        }
    }

    fn write_throttle(&mut self, code: u16) -> u16 {
        match self.dev.dac.set_code(code) {
            Ok(written) => {
                self.throttle_code = written;
                0
            }
            Err(_) => {
                self.throttle_code = self.throttle_code.max(code);
                faults::THROTTLE_DAC
            }
        }
    }

    fn write_brake(&mut self, brake: f32) -> u16 {
        let target = self.cal.brake.target(brake);
        match self.dev.brake.set_target(target) {
            Ok(()) => 0,
            Err(_) => faults::BRAKE_DRIVER,
        }
    }

    fn write_steering(&mut self, angle: f32) -> u16 {
        self.write_steering_target(self.cal.steering.target(angle))
    }

    /// A target in counts, kept inside the calibration's limits.
    fn write_steering_target(&mut self, target: u16) -> u16 {
        let s = &self.cal.steering;
        let target = target.clamp(s.min.min(s.max), s.max.max(s.min));
        match self.dev.steering.set_target(target) {
            Ok(()) => 0,
            Err(_) => faults::STEERING_DRIVER,
        }
    }

    /// The steering Jrk's motor off (its "stop motor" command), every tick, so a Jrk that
    /// reset gets it again. A target, next, turns it back on.
    fn stop_steering(&mut self) -> u16 {
        match self.dev.steering.stop_motor() {
            Ok(()) => 0,
            Err(_) => faults::STEERING_DRIVER,
        }
    }
}

/// A Jrk's position, in target units: `Ok(None)` if it has no analog feedback, or
/// hasn't said yet (its feedback mode is asked once, and again after a failed ask);
/// `Err` if the position read failed.
fn position<I2C: I2c>(
    jrk: &mut JrkG2<I2C>,
    is_position: &mut Option<bool>,
) -> Result<Option<u16>, ()> {
    if is_position.is_none() {
        *is_position = jrk.feedback_mode().ok().map(|mode| mode.is_position());
    }
    match is_position {
        Some(true) => jrk.scaled_feedback().map(Some).map_err(|_| ()),
        _ => Ok(None),
    }
}

/// A Jrk's fault bit, if reading its errors failed or it reports one.
fn jrk_fault<E>(flags: Result<ErrorFlags, E>, bit: u16) -> u16 {
    match flags {
        Ok(f) if !f.is_fault() => 0,
        _ => bit,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use spine_core::protocol::Mode;
    use std::vec;
    use std::vec::Vec;

    const STEER: u8 = 0x05;
    const BRAKE: u8 = 0x04;
    const DAC: u8 = 0x0C;
    const EXP: u8 = 0x20;

    fn jrk(addr: u8, target: u16) -> T {
        T::write(addr, jrk_g2::command::set_target(target).to_vec())
    }
    fn dac(code: u16) -> T {
        T::write(DAC, ad5311::encode(code, ad5311::Mode::Normal).to_vec())
    }
    fn expander(levels: u16) -> T {
        T::write(EXP, vec![0x02, levels as u8, (levels >> 8) as u8])
    }
    fn expander_setup() -> Vec<T> {
        vec![
            T::write(EXP, vec![0x04, 0x00, 0x00]),
            expander(0xFFFF),
            T::write(EXP, vec![0x06, 0x00, 0x00]),
        ]
    }

    /// The init sequence, which every test starts with.
    fn init() -> Vec<T> {
        let mut t = vec![dac(0), jrk(BRAKE, 3500)];
        t.extend(expander_setup());
        t.push(jrk(STEER, 1950));
        t
    }

    /// Actuators on one mock bus: all four devices share its list of transfers, so
    /// the order between devices is checked too.
    fn actuators(expected: &[T]) -> (Actuators<Mock>, Mock) {
        let bus = Mock::new(expected);
        let cal = Calibration::TODAY;
        let dev = Devices::new(bus.clone(), bus.clone(), bus.clone(), bus.clone(), &cal);
        (Actuators::new(dev, cal), bus)
    }

    fn out(throttle: f32, brake: f32, reverse: bool, steering: f32) -> Outputs {
        Outputs {
            mode: Mode::Computer,
            throttle,
            brake,
            reverse,
            steering_target: steering,
            armed: true,
            estop_active: false,
            faults: 0,
        }
    }

    #[test]
    fn init_is_the_cpp_setup() {
        let (mut a, mut bus) = actuators(&init());
        assert_eq!(a.init(), 0);
        assert_eq!(a.throttle_code(), 0);
        bus.done();
    }

    #[test]
    fn without_init_the_first_tick_is_still_safe() {
        // The DAC's code is unknown, so it gets 0 before the expander is set up.
        let mut t = vec![dac(0), jrk(BRAKE, 400)];
        t.extend(expander_setup());
        t.push(jrk(STEER, 1950));
        let (mut a, mut bus) = actuators(&t);
        assert_eq!(a.apply(&out(1.0, 0.0, false, 0.0), 0), 0);
        bus.done();
    }

    #[test]
    fn with_the_dac_silent_the_expander_is_left_alone() {
        let mut t = vec![
            dac(0).with_error(ErrorKind::Other),
            jrk(BRAKE, 3500),
            jrk(STEER, 1950),
        ];
        // The DAC answers next tick: then the expander is set up.
        t.extend([dac(0), jrk(BRAKE, 400)]);
        t.extend(expander_setup());
        t.push(jrk(STEER, 1950));
        let (mut a, mut bus) = actuators(&t);
        assert_eq!(a.init(), faults::THROTTLE_DAC);
        assert_eq!(a.apply(&out(1.0, 0.0, false, 0.0), 0), 0);
        bus.done();
    }

    #[test]
    fn the_throttle_rises_at_most_6_codes_a_tick() {
        let mut t = init();
        // The first tick has no time to rise. After that: the brake off first, then
        // forward, then the throttle, then the steering.
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFF), jrk(STEER, 1950)]);
        for code in [6, 12] {
            t.extend([
                jrk(BRAKE, 400),
                expander(0xFFFF),
                dac(code),
                jrk(STEER, 1950),
            ]);
        }
        let (mut a, mut bus) = actuators(&t);
        a.init();
        for (i, now) in [0, 10, 20].into_iter().enumerate() {
            assert_eq!(a.apply(&out(1.0, 0.0, false, 0.0), now), 0, "tick {i}");
        }
        assert_eq!(a.throttle_code(), 12);
        bus.done();
    }

    #[test]
    fn an_estop_cuts_the_throttle_before_the_brake_comes_on() {
        let mut t = init();
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFF), jrk(STEER, 2260)]);
        for code in [6, 12] {
            t.extend([
                jrk(BRAKE, 400),
                expander(0xFFFF),
                dac(code),
                jrk(STEER, 2260),
            ]);
        }
        // The e-stop tick: DAC to 0, then the brake, then the rest.
        t.extend([dac(0), jrk(BRAKE, 3500), expander(0xFFFF), jrk(STEER, 2260)]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        for now in [0, 10, 20] {
            a.apply(&out(1.0, 0.0, false, 0.1), now);
        }
        let estop = Outputs {
            estop_active: true,
            ..out(0.0, 1.0, false, 0.1)
        };
        assert_eq!(a.apply(&estop, 30), 0);
        assert_eq!(a.throttle_code(), 0);
        bus.done();
    }

    #[test]
    fn reversing_waits_for_the_throttle_to_reach_0() {
        let mut t = init();
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFF), jrk(STEER, 1950)]);
        t.extend([jrk(BRAKE, 400), expander(0xFFFF), dac(6), jrk(STEER, 1950)]);
        // Reverse with throttle: the DAC goes to 0 first, then P00 goes low, and the
        // throttle stays at 0 this tick.
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFE), jrk(STEER, 1950)]);
        // Next tick it rises again, in reverse.
        t.extend([jrk(BRAKE, 400), expander(0xFFFE), dac(6), jrk(STEER, 1950)]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        a.apply(&out(1.0, 0.0, false, 0.0), 0);
        a.apply(&out(1.0, 0.0, false, 0.0), 10);
        a.apply(&out(1.0, 0.0, true, 0.0), 20);
        assert!(a.reverse());
        a.apply(&out(1.0, 0.0, true, 0.0), 30);
        bus.done();
    }

    #[test]
    fn a_failed_dac_write_holds_the_direction() {
        let mut t = init();
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFF), jrk(STEER, 1950)]);
        t.extend([jrk(BRAKE, 400), expander(0xFFFF), dac(6), jrk(STEER, 1950)]);
        // Reverse, but the DAC doesn't take the 0: P00 stays high (forward).
        t.extend([
            dac(0).with_error(ErrorKind::Other),
            jrk(BRAKE, 400),
            expander(0xFFFF),
            jrk(STEER, 1950),
        ]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        a.apply(&out(1.0, 0.0, false, 0.0), 0);
        a.apply(&out(1.0, 0.0, false, 0.0), 10);
        assert_eq!(a.apply(&out(1.0, 0.0, true, 0.0), 20), faults::THROTTLE_DAC);
        assert!(!a.reverse());
        assert_eq!(a.throttle_code(), 6); // it may still hold 6
        bus.done();
    }

    #[test]
    fn a_failed_brake_release_keeps_the_throttle_down() {
        let mut t = init();
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFF), jrk(STEER, 1950)]);
        t.extend([
            jrk(BRAKE, 400).with_error(ErrorKind::Other),
            expander(0xFFFF),
            dac(0),
            jrk(STEER, 1950),
        ]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        a.apply(&out(1.0, 0.0, false, 0.0), 0);
        assert_eq!(
            a.apply(&out(1.0, 0.0, false, 0.0), 10),
            faults::BRAKE_DRIVER
        );
        assert_eq!(a.throttle_code(), 0);
        bus.done();
    }

    #[test]
    fn an_expander_error_stops_the_throttle_until_it_is_set_up_again() {
        let mut t = init();
        t.extend([dac(0), jrk(BRAKE, 400), expander(0xFFFF), jrk(STEER, 1950)]);
        t.extend([
            jrk(BRAKE, 400),
            expander(0xFFFF).with_error(ErrorKind::Other),
            dac(0),
            jrk(STEER, 1950),
        ]);
        // Next tick: no throttle, and the expander is set up again.
        t.extend([dac(0), jrk(BRAKE, 400)]);
        t.extend(expander_setup());
        t.push(jrk(STEER, 1950));
        let (mut a, mut bus) = actuators(&t);
        a.init();
        a.apply(&out(1.0, 0.0, false, 0.0), 0);
        assert_eq!(
            a.apply(&out(1.0, 0.0, false, 0.0), 10),
            faults::THROTTLE_DAC
        );
        assert_eq!(a.apply(&out(1.0, 0.0, false, 0.0), 20), 0);
        bus.done();
    }

    #[test]
    fn bus_errors_set_each_devices_fault() {
        let mut t = init();
        t.extend([
            dac(0).with_error(ErrorKind::Other),
            jrk(BRAKE, 400).with_error(ErrorKind::Other),
            expander(0xFFFF),
            jrk(STEER, 1950).with_error(ErrorKind::Other),
        ]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        let bits = a.apply(&out(0.0, 0.0, false, 0.0), 0);
        assert_eq!(
            bits,
            faults::THROTTLE_DAC | faults::BRAKE_DRIVER | faults::STEERING_DRIVER
        );
        bus.done();
    }

    fn mode(addr: u8, mode: u8) -> T {
        T::write_read(addr, vec![0xEA, 0x18], vec![mode])
    }
    fn scaled(addr: u8, value: u16) -> T {
        T::write_read(addr, vec![0xA7], value.to_le_bytes().to_vec())
    }
    fn errors(addr: u8, flags: u16) -> T {
        T::write_read(addr, vec![0xB3], flags.to_le_bytes().to_vec())
    }

    #[test]
    fn read_gives_the_positions_and_the_jrk_faults() {
        let mut t = init();
        // First read: each Jrk says its feedback is analog (1), once. 2260 is 0.1 rad
        // left; 1950 is half the brake. Steering: only "awaiting command"; brake: no
        // power.
        t.extend([
            mode(STEER, 1),
            scaled(STEER, 2260),
            errors(STEER, 0x0001),
            mode(BRAKE, 1),
            scaled(BRAKE, 1950),
            errors(BRAKE, 0x0002),
        ]);
        // Then the steering's read fails: the last good angle stays, the position is
        // unknown this tick, and it is the steering's fault.
        t.extend([
            scaled(STEER, 0).with_error(ErrorKind::Other),
            errors(STEER, 0),
            scaled(BRAKE, 3500),
            errors(BRAKE, 0),
        ]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        let r = a.read();
        assert!((r.steering - 0.1).abs() < 1e-6);
        assert!((r.positions.steering.unwrap() - 0.1).abs() < 1e-6);
        assert!((r.positions.brake.unwrap() - 0.5).abs() < 1e-6);
        assert_eq!(r.faults, faults::BRAKE_DRIVER);
        let r = a.read();
        assert!((r.steering - 0.1).abs() < 1e-6);
        assert_eq!(r.positions.steering, None);
        assert_eq!(r.positions.brake, Some(1.0));
        assert_eq!(r.faults, faults::STEERING_DRIVER);
        bus.done();
    }

    #[test]
    fn a_jrk_without_feedback_has_no_position_and_no_fault() {
        let mut t = init();
        // The steering has none (0); the brake doesn't answer the question at first.
        t.extend([
            mode(STEER, 0),
            errors(STEER, 0),
            mode(BRAKE, 0).with_error(ErrorKind::Other),
            errors(BRAKE, 0),
        ]);
        // Asked once: the steering isn't asked again; the brake is, and says analog.
        t.extend([
            errors(STEER, 0),
            mode(BRAKE, 1),
            scaled(BRAKE, 400),
            errors(BRAKE, 0),
        ]);
        let (mut a, mut bus) = actuators(&t);
        a.init();
        let r = a.read();
        assert_eq!(r.positions, Positions::default());
        assert_eq!((r.steering, r.faults), (0.0, 0));
        let r = a.read();
        assert_eq!(r.positions.steering, None);
        assert_eq!(r.positions.brake, Some(0.0));
        assert_eq!(r.faults, 0);
        bus.done();
    }

    #[test]
    #[should_panic(expected = "throttle and brake both on")]
    fn throttle_and_brake_together_is_a_bug() {
        let (mut a, _bus) = actuators(&init());
        a.init();
        a.apply(&out(0.5, 0.5, false, 0.0), 0);
    }
}

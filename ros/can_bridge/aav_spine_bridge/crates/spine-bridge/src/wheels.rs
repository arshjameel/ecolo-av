//! The front wheels' speeds, from the wheel nodes' WHEEL_SPEED frames (0x200, 0x201).
//!
//! Two jobs:
//! - keep the latest frame of each wheel, give it a sign when the sensor can't (one Hall
//!   sensor sees only a magnitude), and say when a pair is ready to publish;
//! - turn the front wheels' mean speed into the speed of base_link, under the rear axle.

use std::time::Duration;

use spine_core::protocol::{Wheel, WheelSpeed};

/// m/s: slower than this the wheel sensors can't tell motion from noise; the car stands
/// (TODO 191: stops reset the drift; aav_description's `STANDSTILL_SPEED`).
pub const STANDSTILL_SPEED: f64 = 0.01;
/// (m/s)², (rad/s)²: a standing car's zero speed and turn rate, known rather than guessed.
pub const STANDSTILL_VARIANCE: f64 = 1e-6;

/// Whether base_link's speed says the car stands still (NaN: no).
pub fn is_standstill(speed: f64) -> bool {
    speed.is_finite() && speed.abs() < STANDSTILL_SPEED
}

/// Which way the car is going.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
}

impl Direction {
    /// Which way the car is going, for a wheel sensor that sees only a magnitude: the
    /// spine's measured speed first, then the commanded speed, then forward.
    pub fn of_drive(measured_speed: Option<f32>, target_speed: Option<f32>) -> Direction {
        match [measured_speed, target_speed]
            .into_iter()
            .flatten()
            .find(|v| *v != 0.0)
        {
            Some(v) if v.is_sign_negative() => Direction::Reverse,
            _ => Direction::Forward,
        }
    }

    /// +1 forward, -1 in reverse.
    pub fn sign(self) -> f64 {
        match self {
            Direction::Forward => 1.0,
            Direction::Reverse => -1.0,
        }
    }
}

/// base_link's speed (under the rear axle), in m/s, from the front wheels' mean speed.
///
/// In a turn the front axle travels on a wider circle than the rear one: with the bicycle
/// model, v_rear = v_front * cos(steering). The two front wheels' mean is close to the
/// front axle's centre (within 0.2 % at full lock on this car).
pub fn rear_axle_speed(front_mean: f64, steering_angle: f64) -> f64 {
    front_mean * steering_angle.cos()
}

/// One wheel's latest frame, and when it came.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Sample {
    frame: WheelSpeed,
    time: Duration,
}

/// One front wheel in a [`Reading`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Side {
    /// m/s, signed: the sensor's sign if it has one, else the drive's. 0 if never heard.
    pub speed: f64,
    /// Heard from within the timeout.
    pub fresh: bool,
    /// The sensor's pulse count, as sent (0 if never heard).
    pub pulses: u16,
}

impl Side {
    /// The speed, while it's fresh.
    pub fn fresh_speed(&self) -> Option<f64> {
        self.fresh.then_some(self.speed)
    }
}

/// Both front wheels at one moment, for /vehicle/wheels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    pub left: Side,
    pub right: Side,
    /// Every wheel heard from measures its own sign.
    pub direction_measured: bool,
}

impl Reading {
    /// The two wheels' mean speed, while both are fresh.
    pub fn mean(&self) -> Option<f64> {
        Some((self.left.fresh_speed()? + self.right.fresh_speed()?) / 2.0)
    }
}

/// The latest frame from each front wheel node.
#[derive(Debug, Clone)]
pub struct FrontWheels {
    timeout: Duration,
    left: Option<Sample>,
    right: Option<Sample>,
    /// Which wheels have sent since the last publish: [left, right].
    unpublished: [bool; 2],
}

fn index(wheel: Wheel) -> usize {
    match wheel {
        Wheel::Left => 0,
        Wheel::Right => 1,
    }
}

fn other(wheel: Wheel) -> Wheel {
    match wheel {
        Wheel::Left => Wheel::Right,
        Wheel::Right => Wheel::Left,
    }
}

impl FrontWheels {
    /// A wheel older than `timeout` is stale.
    pub fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            left: None,
            right: None,
            unpublished: [false; 2],
        }
    }

    fn sample(&self, wheel: Wheel) -> Option<&Sample> {
        match wheel {
            Wheel::Left => self.left.as_ref(),
            Wheel::Right => self.right.as_ref(),
        }
    }

    /// Stores one wheel's frame. Returns true when it's time to publish: once both wheels
    /// have sent since the last publish, or at once if the other wheel has gone quiet.
    /// About 100 Hz either way.
    pub fn update(&mut self, frame: WheelSpeed, now: Duration) -> bool {
        let sample = Some(Sample { frame, time: now });
        match frame.wheel {
            Wheel::Left => self.left = sample,
            Wheel::Right => self.right = sample,
        }
        self.unpublished[index(frame.wheel)] = true;
        let partner = other(frame.wheel);
        if self.unpublished[index(partner)] || !self.fresh(partner, now) {
            self.unpublished = [false; 2];
            return true;
        }
        false
    }

    /// The wheel has sent within the timeout.
    pub fn fresh(&self, wheel: Wheel, now: Duration) -> bool {
        self.sample(wheel)
            .is_some_and(|s| now.saturating_sub(s.time) <= self.timeout)
    }

    /// Both wheels at `now`; `direction` signs a wheel that sent only a magnitude.
    pub fn reading(&self, now: Duration, direction: Direction) -> Reading {
        let side = |wheel| {
            let sample = self.sample(wheel);
            Side {
                speed: sample.map_or(0.0, |s| {
                    let speed = f64::from(s.frame.wheel_speed);
                    if s.frame.direction_known {
                        speed
                    } else {
                        direction.sign() * speed.abs()
                    }
                }),
                fresh: self.fresh(wheel, now),
                pulses: sample.map_or(0, |s| s.frame.pulse_count),
            }
        };
        let mut heard = [self.left, self.right].into_iter().flatten().peekable();
        let direction_measured = heard.peek().is_some() && heard.all(|s| s.frame.direction_known);
        Reading {
            left: side(Wheel::Left),
            right: side(Wheel::Right),
            direction_measured,
        }
    }
}

#[cfg(test)]
mod tests {
    //! The cases of aav_spine's test_wheel_speed.py.
    use super::*;

    #[test]
    fn wheels_that_read_nothing_mean_standing_still() {
        assert!(is_standstill(0.0));
        assert!(is_standstill(-0.005));
        assert!(!is_standstill(0.02));
        assert!(!is_standstill(f64::NAN));
    }

    const TIMEOUT: Duration = Duration::from_millis(100);

    fn ms(t: u64) -> Duration {
        Duration::from_millis(t)
    }

    fn frame(wheel: Wheel, speed: f32, pulses: u16, known: bool) -> WheelSpeed {
        WheelSpeed {
            wheel,
            wheel_speed: speed,
            pulse_count: pulses,
            direction_known: known,
            counter: 0,
        }
    }

    fn left(speed: f32) -> WheelSpeed {
        frame(Wheel::Left, speed, 0, false)
    }

    fn right(speed: f32) -> WheelSpeed {
        frame(Wheel::Right, speed, 0, false)
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn publishes_once_per_pair() {
        let mut w = FrontWheels::new(TIMEOUT);
        assert!(w.update(left(1.0), ms(0))); // right never heard: publish alone
        assert!(!w.update(right(1.0), ms(1))); // left was just published
        assert!(w.update(left(1.0), ms(10))); // the pair is complete
        assert!(!w.update(right(1.0), ms(11)));
        assert!(w.update(left(1.0), ms(20)));
    }

    #[test]
    fn a_silent_wheel_does_not_stop_the_other() {
        let mut w = FrontWheels::new(TIMEOUT);
        w.update(left(1.0), ms(0));
        w.update(right(1.0), ms(0));
        assert!(w.update(left(1.0), ms(500))); // right is stale: publish now
        let reading = w.reading(ms(500), Direction::Forward);
        assert!(reading.left.fresh && !reading.right.fresh);
        assert_eq!(reading.mean(), None);
    }

    #[test]
    fn mean_of_two_fresh_wheels() {
        let mut w = FrontWheels::new(TIMEOUT);
        w.update(frame(Wheel::Left, 1.0, 10, false), ms(0));
        w.update(frame(Wheel::Right, 1.2, 12, false), ms(0));
        let reading = w.reading(ms(50), Direction::Forward);
        assert!(reading.mean().is_some_and(|m| close(m, 1.1)));
        assert_eq!((reading.left.pulses, reading.right.pulses), (10, 12));
        assert!(!reading.direction_measured);
    }

    #[test]
    fn magnitude_takes_the_drive_sign_but_a_measured_sign_stays() {
        let mut w = FrontWheels::new(TIMEOUT);
        w.update(left(0.8), ms(0));
        w.update(frame(Wheel::Right, -0.7, 0, true), ms(0));
        let reading = w.reading(ms(0), Direction::Reverse);
        assert!(close(reading.left.speed, -0.8));
        assert!(close(reading.right.speed, -0.7));
    }

    #[test]
    fn direction_is_measured_only_when_every_wheel_heard_measures_it() {
        let mut w = FrontWheels::new(TIMEOUT);
        assert!(!w.reading(ms(0), Direction::Forward).direction_measured);
        w.update(frame(Wheel::Left, 0.5, 0, true), ms(0));
        assert!(w.reading(ms(0), Direction::Forward).direction_measured);
        w.update(right(0.5), ms(0));
        assert!(!w.reading(ms(0), Direction::Forward).direction_measured);
    }

    #[test]
    fn drive_direction() {
        // The spine's measurement wins; stopped, the command says which way; else forward.
        assert_eq!(
            Direction::of_drive(Some(-0.5), Some(1.0)),
            Direction::Reverse
        );
        assert_eq!(
            Direction::of_drive(Some(0.0), Some(-1.0)),
            Direction::Reverse
        );
        assert_eq!(Direction::of_drive(None, None), Direction::Forward);
    }

    #[test]
    fn rear_axle() {
        assert!(close(rear_axle_speed(2.0, 0.0), 2.0));
        assert!(close(rear_axle_speed(2.0, 0.5), 2.0 * 0.5_f64.cos()));
        assert!(rear_axle_speed(-1.0, -0.3) < 0.0);
    }
}

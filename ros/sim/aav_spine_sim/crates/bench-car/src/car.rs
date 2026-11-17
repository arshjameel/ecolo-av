//! The car the spine drives: the bench's model, or one measured somewhere else.
//!
//! The spine only needs a car's speed and steering each tick, and to tell it what to do.
//! The wheel nodes need how far each front tyre has rolled, and the spine's feedback frame
//! the motor encoder's count. [`Plant`] works all of that out itself; [`MeasuredCar`] is
//! told it, by a simulator that moves the car (Ravensim, through sim_adapter).

use spine_core::Outputs;

use crate::plant::Plant;

/// What the spine and the wheel nodes need from a car.
pub trait Car {
    /// m/s along the car, negative in reverse.
    fn speed(&self) -> f32;
    /// rad, the front wheels' angle, positive left.
    fn steering(&self) -> f32;
    /// The motor encoder's 16-bit counter, as VEHICLE_FEEDBACK sends it.
    fn wheel_ticks(&self) -> u16;
    /// How far each front tyre has rolled, either way (m): [left, right].
    fn front_rolled(&self) -> [f64; 2];
    /// `dt` seconds pass with the spine's `out` on the actuators.
    fn step(&mut self, out: &Outputs, dt: f32);
}

impl Car for Plant {
    fn speed(&self) -> f32 {
        self.pose().speed
    }
    fn steering(&self) -> f32 {
        self.pose().steering
    }
    fn wheel_ticks(&self) -> u16 {
        Plant::wheel_ticks(self)
    }
    fn front_rolled(&self) -> [f64; 2] {
        Plant::front_rolled(self)
    }
    fn step(&mut self, out: &Outputs, dt: f32) {
        Plant::step(self, out, dt)
    }
}

/// A car that something else moves, such as Ravensim's: it only keeps the last measurements
/// it was given, and counts the encoder and the tyres' distance from them.
///
/// The spine's outputs go back to whatever moves the car (`Bench::outputs`): here they
/// change nothing directly.
#[derive(Debug, Clone)]
pub struct MeasuredCar {
    ticks_per_metre: f32,
    speed: f32,
    steering: f32,
    front: [f32; 2],
    ticks: f64,
    front_rolled: [f64; 2],
}

impl MeasuredCar {
    /// A car standing still, its encoder at `ticks_per_metre` counts per metre.
    #[must_use]
    pub fn new(ticks_per_metre: f32) -> Self {
        MeasuredCar {
            ticks_per_metre,
            speed: 0.0,
            steering: 0.0,
            front: [0.0; 2],
            ticks: 0.0,
            front_rolled: [0.0; 2],
        }
    }

    /// The car's measured speed (m/s, signed). A NaN or infinite one is ignored: a sensor
    /// fault keeps the last good value.
    pub fn measure_speed(&mut self, speed: f32) {
        if speed.is_finite() {
            self.speed = speed;
        }
    }

    /// The front wheels' measured angle (rad, + left). Non-finite values are ignored.
    pub fn measure_steering(&mut self, steering: f32) {
        if steering.is_finite() {
            self.steering = steering;
        }
    }

    /// Each front tyre's speed over the ground (m/s): [left, right]. Non-finite ones are
    /// ignored, each on its own.
    pub fn measure_front_wheels(&mut self, front: [f32; 2]) {
        for (kept, new) in self.front.iter_mut().zip(front) {
            if new.is_finite() {
                *kept = new;
            }
        }
    }
}

impl Car for MeasuredCar {
    fn speed(&self) -> f32 {
        self.speed
    }
    fn steering(&self) -> f32 {
        self.steering
    }
    fn wheel_ticks(&self) -> u16 {
        (self.ticks as u64 & 0xFFFF) as u16
    }
    fn front_rolled(&self) -> [f64; 2] {
        self.front_rolled
    }
    fn step(&mut self, _out: &Outputs, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let dt = dt as f64;
        self.ticks += (self.speed.abs() * self.ticks_per_metre) as f64 * dt;
        for (rolled, speed) in self.front_rolled.iter_mut().zip(self.front) {
            *rolled += speed.abs() as f64 * dt;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use spine_core::protocol::Mode;

    fn idle() -> Outputs {
        Outputs {
            mode: Mode::Manual,
            throttle: 0.0,
            brake: 0.0,
            reverse: false,
            steering_target: 0.0,
            armed: false,
            estop_active: false,
            faults: 0,
        }
    }

    #[test]
    fn it_counts_the_encoder_and_the_tyres_from_what_it_is_told() {
        let mut car = MeasuredCar::new(100.0);
        car.measure_speed(-2.0);
        car.measure_front_wheels([1.0, 1.5]);
        for _ in 0..100 {
            car.step(&idle(), 0.01);
        }
        assert_eq!(car.speed(), -2.0);
        assert!(
            (car.wheel_ticks() as i32 - 200).abs() <= 1,
            "2 m either way"
        );
        let [l, r] = car.front_rolled();
        assert!((l - 1.0).abs() < 1e-6 && (r - 1.5).abs() < 1e-6, "{l} {r}");
    }

    #[test]
    fn a_broken_measurement_keeps_the_last_good_one() {
        let mut car = MeasuredCar::new(100.0);
        car.measure_speed(1.0);
        car.measure_steering(0.2);
        car.measure_front_wheels([1.0, 1.0]);
        car.measure_speed(f32::NAN);
        car.measure_steering(f32::INFINITY);
        car.measure_front_wheels([f32::NAN, 0.5]);
        assert_eq!((car.speed(), car.steering()), (1.0, 0.2));
        car.step(&idle(), 1.0);
        assert_eq!(car.front_rolled(), [1.0, 0.5]);
    }
}

//! Pretend data for `--demo`: a simulated car that follows the buttons, with every
//! sensor talking. No ROS, no car; it announces itself as SIM, so the speed buttons work.

use std::time::{Duration, Instant};

use hud_core::{
    gps::{Fix, RawFix},
    origin::{Announcement, Entry, Origin},
    state::Battery,
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use panel_core::{
    buttons::Command,
    state::{Actuator, ImuReading, Row, Update, Wheels},
};

use crate::source::DataSource;

/// ego's own limits, near enough: it slows at 2 m/s², speeds up at 1.
const DECEL: f64 = 2.0;
const ACCEL: f64 = 1.0;
/// The pretend car's wheelbase (vehicle.yaml's).
const WHEELBASE: f64 = 1.57;
/// A command older than this is no command (ego's cmd_timeout).
const STALE: Duration = Duration::from_millis(300);

/// The pretend car.
pub struct Demo {
    start: Instant,
    last: Instant,
    speed: f64,
    steer: f64,
    command: Option<(Instant, Command)>,
    estop: bool,
    announced: bool,
    ticks: u64,
}

impl Demo {
    pub fn new(now: Instant) -> Self {
        Self {
            start: now,
            last: now,
            speed: 0.0,
            steer: 0.0,
            command: None,
            estop: false,
            announced: false,
            ticks: 0,
        }
    }

    /// The car follows the last fresh command, as ego would.
    fn step(&mut self, now: Instant) {
        let dt = now.saturating_duration_since(self.last).as_secs_f64();
        self.last = now;
        let wanted = match self.command {
            Some((at, c)) if !self.estop && now.saturating_duration_since(at) < STALE => c,
            _ => Command::ZERO,
        };
        let dv = wanted.speed - self.speed;
        let rate = if dv > 0.0 { ACCEL } else { DECEL };
        self.speed += dv.clamp(-rate * dt, rate * dt);
        // ego holds the wheels below 0.05 m/s.
        if self.speed > 0.05 && wanted.speed > 0.05 {
            let target = (wanted.yaw_rate * WHEELBASE / wanted.speed).atan();
            self.steer += (target - self.steer).clamp(-0.5 * dt, 0.5 * dt);
        }
    }
}

impl DataSource for Demo {
    fn label(&self) -> &'static str {
        "DEMO"
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        self.step(now);
        self.ticks += 1;
        let t = now.saturating_duration_since(self.start).as_secs_f64();
        let wiggle = |k: f64| (t * k).sin() * 0.02;
        let mut news = Vec::new();
        if !self.announced {
            self.announced = true;
            news.push(Update::Source(Announcement {
                announcer: "demo".into(),
                entries: vec![Entry {
                    name: "car".into(),
                    topic: "/vehicle/status".into(),
                    origin: Origin::Sim,
                }],
            }));
            news.push(Update::GeofenceLock(false));
        }
        let speed = self.speed as f32;
        news.push(Update::Vehicle(VehicleStatus {
            mode: Mode::from(1),
            armed: true,
            estop_active: self.estop,
            speed: MetresPerSecond(speed),
            steering_angle: Radians(self.steer as f32),
            throttle: Fraction::new((speed / 3.0).min(1.0)),
            brake: Fraction::new(0.0),
            faults: Faults(0),
        }));
        news.push(Update::Steering(Actuator {
            command: self.steer as f32,
            measured: self.steer as f32,
        }));
        news.push(Update::Throttle(Actuator {
            command: f32::NAN,
            measured: (speed / 3.0).min(1.0),
        }));
        news.push(Update::Brake(Actuator {
            command: f32::NAN,
            measured: if self.command.is_none() { 0.2 } else { 0.0 },
        }));
        news.push(Update::Wheels(Wheels {
            front_left: speed,
            front_right: speed,
            rear_left: speed,
            rear_right: speed,
        }));
        news.push(Update::Imu(ImuReading {
            accel: [wiggle(7.0), wiggle(5.0), 9.81 + wiggle(3.0)],
            gyro: [
                wiggle(11.0),
                wiggle(13.0),
                self.speed * self.steer.tan() / WHEELBASE,
            ],
        }));
        news.push(Update::Counted(Row::Lidar));
        if self.ticks.is_multiple_of(2) {
            news.push(Update::Counted(Row::Camera));
        }
        if self.ticks.is_multiple_of(4) {
            news.push(Update::Counted(Row::Radar));
            news.push(Update::Gps(Fix::from(RawFix {
                status: 2,
                latitude: 45.3876,
                longitude: -75.6960,
                altitude: 80.0,
                position_covariance: [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.001],
                position_covariance_type: 2,
            })));
            news.push(Update::Battery(Battery::from_message(51.2, 0.82, -12.0)));
            news.push(Update::GeofenceLock(false));
        }
        // twist_mux passes the buttons' command: nobody else drives here.
        if let Some((at, c)) = self.command {
            if now.saturating_duration_since(at) < STALE {
                news.push(Update::Mux(c));
            }
        }
        news.push(Update::Estop(self.estop));
        news
    }

    fn press_estop(&mut self) {
        self.estop = true;
    }

    fn frame_drawn(&mut self, command: Option<Command>) {
        if let Some(c) = command {
            self.command = Some((Instant::now(), c));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_pretend_car_follows_the_buttons_and_stops_without_them() {
        let t = Instant::now();
        let mut demo = Demo::new(t);
        let ahead = Command::from_steering(0.3, 0.0, WHEELBASE);
        for i in 1..=20 {
            let at = t + Duration::from_millis(100 * i);
            demo.command = Some((at, ahead));
            demo.step(at);
        }
        assert!((demo.speed - 0.3).abs() < 1e-9);
        // No command for a while: it stops.
        demo.step(t + Duration::from_secs(5));
        assert_eq!(demo.speed, 0.0);
    }
}

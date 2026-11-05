//! `--demo`: pretend data, so the HUD runs on any computer with no ROS and no car.
//!
//! It walks through what the screen must show: the speed going up and down, a fault now
//! and then, a stop sign every 25 s, and the GPS cycling through 3D, RTK float, RTK
//! fixed, no fix and silence (40 s). The GO / STOP panel follows (hud_core::gate): the
//! pretend watchdog holds Nav2 while the GPS is silent, and every 60 s the pretend output
//! mux waits, then latches a stop for a simulated camera. The e-stop button stops the
//! pretend car; a pretend laptop clears it 10 s later, so the demo keeps going.

use std::{
    f32::consts::TAU,
    time::{Duration, Instant},
};

use hud_core::{
    estop::EstopButton,
    gps::{Fix, RawFix},
    picture::Picture,
    state::{Battery, EstopReport, Update},
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};

use crate::source::DataSource;

/// The pretend car's e-stop lasts this long, then the "laptop" clears it.
const ESTOP_HOLD: Duration = Duration::from_secs(10);
/// Carleton University, near the AAV's lab: where the pretend car drives.
const HOME: (f64, f64) = (45.3876, -75.6960);
/// The pretend camera's picture.
const PICTURE_W: u32 = 320;
const PICTURE_H: u32 = 180;

/// Each pretend topic, and how often it sends.
#[derive(Debug, Clone, Copy)]
enum Stream {
    Vehicle,
    Gps,
    GpsFiltered,
    StopSign,
    Battery,
    Camera,
    Watchdog,
    OutputMux,
}

impl Stream {
    const ALL: [Stream; 8] = [
        Stream::Vehicle,
        Stream::Gps,
        Stream::GpsFiltered,
        Stream::StopSign,
        Stream::Battery,
        Stream::Camera,
        Stream::Watchdog,
        Stream::OutputMux,
    ];

    fn period(self) -> Duration {
        Duration::from_millis(match self {
            Self::Vehicle => 100,
            Self::Gps | Self::GpsFiltered => 200,
            Self::StopSign => 200,
            Self::Battery => 1000,
            Self::Camera => 100,
            Self::Watchdog => 100,
            Self::OutputMux => 500,
        })
    }
}

/// The demo source.
pub struct Demo {
    start: Instant,
    due: [Instant; 8],
    button: EstopButton,
    estop_since: Option<Instant>,
    /// The pretend car's speed when the e-stop came, to brake from.
    speed_at_estop: f32,
}

impl Demo {
    /// A demo that starts now.
    pub fn new(now: Instant) -> Self {
        Self {
            start: now,
            due: [now; 8],
            button: EstopButton::default(),
            estop_since: None,
            speed_at_estop: 0.0,
        }
    }

    /// Seconds since the demo started.
    fn t(&self, now: Instant) -> f32 {
        now.saturating_duration_since(self.start).as_secs_f32()
    }

    fn update(&mut self, stream: Stream, now: Instant) -> Option<Update> {
        let t = self.t(now);
        match stream {
            Stream::Vehicle => Some(Update::Vehicle(self.vehicle(now))),
            Stream::Gps => gps(t).map(Update::Gps),
            Stream::GpsFiltered => gps(t).map(|_| Update::GpsFiltered(filtered(t))),
            Stream::StopSign => Some(Update::StopSign((8.0..11.0).contains(&(t % 25.0)))),
            Stream::Battery => Some(Update::Battery(Battery::from_message(
                51.2 - t / 600.0,
                (0.87 - t / 36_000.0).max(0.0),
                -12.0,
            ))),
            Stream::Camera => Some(
                Picture::from_image("rgb8", PICTURE_W, PICTURE_H, PICTURE_W * 3, &road(t))
                    .map_or_else(|e| Update::BadPicture(e.to_string()), Update::Picture),
            ),
            Stream::Watchdog => Some(Update::Watchdog(watchdog(t).into())),
            Stream::OutputMux => Some(Update::OutputMux(output_mux(t).into())),
        }
    }

    fn vehicle(&mut self, now: Instant) -> VehicleStatus {
        let t = self.t(now);
        // m/s: up and down under ego's 1 m/s cap (hud.yaml's max_speed), as the car drives.
        let cruising = 0.6 + 0.35 * (t / 4.0).sin();
        let stopped_for = self
            .estop_since
            .map(|since| now.saturating_duration_since(since));
        if stopped_for.is_some_and(|d| d > ESTOP_HOLD) {
            self.estop_since = None;
        }
        let faults = if (20.0..25.0).contains(&(t % 60.0)) {
            Faults(64) // the wheel encoder, now and then
        } else {
            Faults(0)
        };
        match stopped_for.filter(|d| *d <= ESTOP_HOLD) {
            Some(d) => VehicleStatus {
                mode: Mode::Computer,
                armed: false,
                estop_active: true,
                // Brakes hard: down to nothing in about a second.
                speed: MetresPerSecond(self.speed_at_estop * (-3.0 * d.as_secs_f32()).exp()),
                steering_angle: Radians(0.0),
                throttle: Fraction::new(0.0),
                brake: Fraction::new(1.0),
                faults,
            },
            None => {
                self.speed_at_estop = cruising;
                let slowing = (t / 4.0).cos() < 0.0;
                VehicleStatus {
                    mode: Mode::Computer,
                    armed: true,
                    estop_active: false,
                    speed: MetresPerSecond(cruising),
                    steering_angle: Radians(0.3 * (t / 3.0).sin()),
                    throttle: Fraction::new(if slowing { 0.05 } else { 0.35 }),
                    brake: Fraction::new(if slowing { 0.3 } else { 0.0 }),
                    faults,
                }
            }
        }
    }
}

impl DataSource for Demo {
    fn label(&self) -> &'static str {
        "DEMO"
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        let mut out = Vec::new();
        while self.button.poll(now).is_some() {
            // The pretend car hears it at once; the first one starts its stop.
            if self.estop_since.is_none() {
                self.estop_since = Some(now);
            }
            out.push(Update::Estop(EstopReport::Sent { subscribers: 1 }));
        }
        for (i, stream) in Stream::ALL.into_iter().enumerate() {
            if now >= self.due[i] {
                self.due[i] = now + stream.period();
                out.extend(self.update(stream, now));
            }
        }
        out
    }

    fn press_estop(&mut self) {
        self.button.press(Instant::now());
    }
}

/// The pretend receiver at `t`: None while it's silent.
fn gps(t: f32) -> Option<Fix> {
    let phase = t % 40.0;
    let (status, sigma) = match phase {
        p if p < 10.0 => (0, 1.8),   // 3D
        p if p < 20.0 => (2, 0.4),   // RTK float
        p if p < 30.0 => (2, 0.015), // RTK fixed
        p if p < 34.0 => (-1, 0.0),  // no fix
        _ => return None,            // silent: STALE
    };
    let (lat, lon) = circle(t);
    let mut raw = RawFix {
        status,
        latitude: lat,
        longitude: lon,
        altitude: 80.0,
        position_covariance_type: 2,
        ..RawFix::default()
    };
    raw.position_covariance[0] = sigma * sigma;
    raw.position_covariance[4] = sigma * sigma;
    Some(Fix::from(raw))
}

/// The pretend sensor watchdog at `t`: it holds Nav2 once the GPS has been silent 2 s.
fn watchdog(t: f32) -> &'static str {
    if (36.0..40.0).contains(&(t % 40.0)) {
        "STOP: gps STALE (nothing for 2.0 s (more than 2 s))"
    } else {
        "OK"
    }
}

/// The pretend output mux at `t`: a minute's cycle of WAIT, OPEN, and a latched STOP that
/// a pretend laptop clears.
fn output_mux(t: f32) -> &'static str {
    match t % 60.0 {
        p if p < 2.0 => "WAIT - car: all REAL, settling for 2 s",
        p if (46.0..54.0).contains(&p) => {
            "STOP - target car needs REAL - camera_front_left is SIM (gazebo_adapter) - \
             latched: /safety/output_mux/clear"
        }
        p if (54.0..56.0).contains(&p) => "WAIT - car: all REAL, settling for 2 s",
        _ => "OPEN - car: every source REAL",
    }
}

/// The pretend EKF's position: the same circle, always a fix.
fn filtered(t: f32) -> Fix {
    let (latitude, longitude) = circle(t);
    Fix::from(RawFix {
        status: 0,
        latitude,
        longitude,
        altitude: 80.0,
        ..RawFix::default()
    })
}

/// A 100 m circle round HOME, once a minute.
fn circle(t: f32) -> (f64, f64) {
    let a = f64::from(t / 60.0 * TAU);
    let metres_per_degree = 111_320.0;
    let lat = HOME.0 + 100.0 * a.sin() / metres_per_degree;
    let lon = HOME.1 + 100.0 * a.cos() / (metres_per_degree * HOME.0.to_radians().cos());
    (lat, lon)
}

/// The pretend camera: sky, grass, a road, and its dashes moving towards the car.
fn road(t: f32) -> Vec<u8> {
    let (w, h) = (PICTURE_W as f32, PICTURE_H as f32);
    let horizon = h * 0.45;
    let mut px = Vec::with_capacity((PICTURE_W * PICTURE_H * 3) as usize);
    for y in 0..PICTURE_H {
        for x in 0..PICTURE_W {
            let (x, y) = (x as f32, y as f32);
            let rgb: [u8; 3] = if y < horizon {
                let k = y / horizon;
                [(90.0 + 60.0 * k) as u8, (140.0 + 50.0 * k) as u8, 220]
            } else {
                // 0 at the horizon, 1 at the bottom: the road widens towards the car.
                let depth = (y - horizon) / (h - horizon);
                let half_road = w * (0.05 + 0.45 * depth);
                let from_centre = (x - w / 2.0).abs();
                let dash = ((1.0 / (depth + 0.05)) * 2.0 + t * 4.0).fract() < 0.5;
                if from_centre < w * 0.006 + 2.0 * depth && dash {
                    [240, 220, 60]
                } else if from_centre < half_road {
                    [70, 70, 75]
                } else {
                    [60, (130.0 + 40.0 * depth) as u8, 60]
                }
            };
            px.extend_from_slice(&rgb);
        }
    }
    px
}

#[cfg(test)]
mod tests {
    use super::*;
    use hud_core::gps::FixKind;

    #[test]
    fn the_gps_walks_through_every_kind() {
        let kind = |t| gps(t).map(|f| f.kind);
        assert_eq!(kind(1.0), Some(FixKind::Fix3d));
        assert_eq!(kind(11.0), Some(FixKind::RtkFloat));
        assert_eq!(kind(21.0), Some(FixKind::RtkFixed));
        assert_eq!(kind(31.0), Some(FixKind::NoFix));
        assert_eq!(kind(35.0), None);
    }

    #[test]
    fn the_estop_stops_the_pretend_car() {
        let start = Instant::now();
        let mut demo = Demo::new(start);
        demo.press_estop();
        let later = Instant::now() + Duration::from_millis(500);
        let updates = demo.drain(later);
        assert!(updates.iter().any(|u| matches!(u, Update::Estop(_))));
        let car = demo.vehicle(later + Duration::from_secs(3));
        assert!(car.estop_active);
        assert!(car.speed.0 < 0.1);
    }

    #[test]
    fn the_go_stop_panel_walks_through_its_states() {
        use hud_core::gate::{verdict, Light};
        use hud_core::health::Health;
        let ok = Health::Ok(Duration::ZERO);
        let light = |t| verdict(false, (ok, Some(output_mux(t))), (ok, Some(watchdog(t)))).light;
        assert_eq!(light(1.0), Light::Wait);
        assert_eq!(light(10.0), Light::Go);
        assert_eq!(light(37.0), Light::Stop); // the GPS silent: AUTO STOP
        assert_eq!(light(50.0), Light::Stop); // the gate latched
    }

    #[test]
    fn the_camera_picture_converts() {
        let p = Picture::from_image("rgb8", PICTURE_W, PICTURE_H, PICTURE_W * 3, &road(1.0));
        assert!(p.is_ok_and(|p| p.rgba.len() == (PICTURE_W * PICTURE_H * 4) as usize));
    }
}

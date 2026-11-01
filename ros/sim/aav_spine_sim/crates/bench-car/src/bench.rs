//! The bench: the spine's real logic driving the model car, one tick at a time.
//!
//! Frames go in as they arrive from the bus. Each tick builds the spine's `Inputs` (the
//! frames since the last tick, the model's speed and steering, the bench's switches), runs
//! `Spine::step`, moves the model under the `Outputs`, and returns the frames the spine
//! sends: VEHICLE_FEEDBACK every 20 ms, VEHICLE_STATUS every 100 ms, as on the car.
//!
//! The two front wheel nodes run here too, on their own logic (wheel-core): the model's
//! tyres pass rotor holes, each hole is a Hall sensor pulse at the moment it passes, and
//! WHEEL_SPEED_L and WHEEL_SPEED_R go out every 10 ms. Like the real sensors, they send a
//! magnitude only (direction_known = 0).

use spine_core::protocol::Wheel;
use spine_core::protocol::{decode_incoming, FrameError, Incoming};
use spine_core::protocol::{VEHICLE_FEEDBACK_ID, VEHICLE_STATUS_ID};
use spine_core::{Inputs, Outputs, Spine, SpineConfig};
use wheel_core::{ConfigError, WheelConfig, WheelNode};

use crate::car::Car;
use crate::plant::{Plant, PlantConfig, Pose};

/// How often the spine sends each frame (ms), from firmware/can/aav.dbc.
pub const FEEDBACK_PERIOD_MS: u32 = 20;
pub const STATUS_PERIOD_MS: u32 = 100;
pub use wheel_core::FRAME_PERIOD_MS as WHEEL_PERIOD_MS;

/// The bench's wheel nodes: 8 rotor holes (a guess until the rotors are counted) on
/// vehicle.yaml's 0.30 m wheel.
pub const BENCH_WHEELS: WheelConfig = WheelConfig {
    holes_per_rev: 8,
    wheel_radius: 0.15,
    zero_speed_timeout_ms: 500,
    min_pulse_gap_us: 0,
};

/// One CAN frame: an 11-bit ID and 8 data bytes.
pub type Frame = (u16, [u8; 8]);

/// The spine and the car it drives: the bench's model car ([`Plant`], the default) or
/// one measured elsewhere, such as Gazebo's ([`crate::MeasuredCar`]).
pub struct Bench<C: Car = Plant> {
    spine: Spine,
    plant: C,
    now_ms: u32,
    since_feedback_ms: u32,
    since_status_ms: u32,
    pending: Inputs,
    hw_estop: bool,
    driver_faults: u16,
    last: Option<Outputs>,
    /// The front wheel nodes, [left, right], and the holes each has passed so far.
    wheels: [WheelNode; 2],
    holes_passed: [u64; 2],
    since_wheels_ms: u32,
}

impl Bench<Plant> {
    /// The spine driving the bench's model car, from `start`.
    pub fn new(
        spine: SpineConfig,
        plant: PlantConfig,
        start: Pose,
        wheels: WheelConfig,
    ) -> Result<Self, ConfigError> {
        Bench::with_car(spine, Plant::new(plant, start), wheels)
    }

    /// Where the model car is: the ground truth.
    pub fn pose(&self) -> Pose {
        self.plant.pose()
    }
}

impl<C: Car> Bench<C> {
    /// The spine driving any car: the model, or one measured elsewhere.
    pub fn with_car(spine: SpineConfig, car: C, wheels: WheelConfig) -> Result<Self, ConfigError> {
        Ok(Bench {
            spine: Spine::new(spine),
            plant: car,
            now_ms: 0,
            since_feedback_ms: 0,
            since_status_ms: 0,
            pending: Inputs::default(),
            hw_estop: false,
            driver_faults: 0,
            last: None,
            wheels: [
                WheelNode::new(Wheel::Left, wheels)?,
                WheelNode::new(Wheel::Right, wheels)?,
            ],
            holes_passed: [0; 2],
            since_wheels_ms: 0,
        })
    }

    /// A frame from the bus. Frames for other IDs are ignored, as the spine's filter does.
    pub fn receive(&mut self, id: u16, data: &[u8]) {
        match decode_incoming(id, data) {
            Ok(Incoming::Drive(cmd)) => self.pending.drive_cmd = Some(cmd),
            Ok(Incoming::Estop(cmd)) => self.pending.estop_cmd = Some(cmd),
            Err(FrameError::UnknownId) => {}
            Err(_) => self.pending.bad_frame = true,
        }
    }

    /// The hardware e-stop loop: true = open (pressed).
    pub fn set_hw_estop(&mut self, open: bool) {
        self.hw_estop = open;
    }

    /// Pretend a driver failed: spine_core::faults bits, 0 = all well.
    pub fn set_driver_faults(&mut self, bits: u16) {
        self.driver_faults = bits;
    }

    /// Run the spine and the model for `dt_ms`, and return the frames due.
    pub fn tick(&mut self, dt_ms: u32) -> Vec<Frame> {
        self.now_ms = self.now_ms.wrapping_add(dt_ms);
        let inputs = Inputs {
            now_ms: self.now_ms,
            hw_estop: self.hw_estop,
            rc: None,
            measured_speed: self.plant.speed(),
            measured_steering: self.plant.steering(),
            driver_faults: self.driver_faults,
            ..core::mem::take(&mut self.pending)
        };
        let out = self.spine.step(&inputs);
        let rolled_before = self.plant.front_rolled();
        self.plant.step(&out, dt_ms as f32 / 1000.0);
        self.last = Some(out);
        self.pass_holes(rolled_before, dt_ms);

        let mut frames = Vec::new();
        self.since_wheels_ms += dt_ms;
        if self.since_wheels_ms >= WHEEL_PERIOD_MS {
            self.since_wheels_ms -= WHEEL_PERIOD_MS;
            let now_us = self.now_us();
            frames.extend(
                self.wheels
                    .iter_mut()
                    .map(|w| (w.wheel().id(), w.frame(now_us, None))),
            );
        }
        self.since_feedback_ms += dt_ms;
        self.since_status_ms += dt_ms;
        if self.since_feedback_ms >= FEEDBACK_PERIOD_MS {
            self.since_feedback_ms -= FEEDBACK_PERIOD_MS;
            let data = self.spine.feedback_frame(
                self.plant.speed(),
                self.plant.steering(),
                self.plant.wheel_ticks(),
            );
            frames.push((VEHICLE_FEEDBACK_ID, data));
        }
        if self.since_status_ms >= STATUS_PERIOD_MS {
            self.since_status_ms -= STATUS_PERIOD_MS;
            frames.push((VEHICLE_STATUS_ID, self.spine.status_frame(&out)));
        }
        frames
    }

    /// The wheel nodes' microsecond clock. It wraps, as theirs does.
    fn now_us(&self) -> u32 {
        self.now_ms.wrapping_mul(1000)
    }

    /// A pulse to each wheel node for every hole that passed its sensor during the last
    /// `dt_ms`, at the moment it passed (the tyre's distance, interpolated in the tick).
    fn pass_holes(&mut self, rolled_before: [f64; 2], dt_ms: u32) {
        let start_us = self.now_us().wrapping_sub(dt_ms * 1000);
        let rolled_after = self.plant.front_rolled();
        for (i, node) in self.wheels.iter_mut().enumerate() {
            let per_hole = node.config().metres_per_pulse() as f64;
            let (d0, d1) = (rolled_before[i], rolled_after[i]);
            let holes = (d1 / per_hole) as u64;
            while self.holes_passed[i] < holes {
                self.holes_passed[i] += 1;
                let at = self.holes_passed[i] as f64 * per_hole;
                let fraction = ((at - d0) / (d1 - d0)).clamp(0.0, 1.0);
                node.on_pulse(start_us.wrapping_add((fraction * dt_ms as f64 * 1000.0) as u32));
            }
        }
    }

    /// The car the spine drives.
    pub fn car(&self) -> &C {
        &self.plant
    }

    /// The car the spine drives, to give it new measurements.
    pub fn car_mut(&mut self) -> &mut C {
        &mut self.plant
    }

    /// What the spine wrote to the actuators on the last tick.
    pub fn outputs(&self) -> Option<Outputs> {
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use spine_core::faults;
    use spine_core::protocol::{
        DriveCmd, EstopCmd, Mode, VehicleStatus, WheelSpeed, WHEEL_SPEED_L_ID, WHEEL_SPEED_R_ID,
    };

    /// The bench, and what spine_bridge would send it.
    struct Rig {
        bench: Bench,
        counter: u8,
    }

    impl Rig {
        fn new() -> Self {
            Rig::with_spine(SpineConfig::default())
        }

        fn with_spine(spine: SpineConfig) -> Self {
            Rig {
                bench: Bench::new(spine, PlantConfig::default(), Pose::default(), BENCH_WHEELS)
                    .expect("the bench's wheel settings are valid"),
                counter: 0,
            }
        }

        fn drive(&mut self, speed: f32, steering: f32, armed: bool) {
            self.counter = (self.counter + 1) & 0x0F;
            let cmd = DriveCmd {
                target_speed: speed,
                steering_angle: steering,
                accel_limit: 0.0,
                steer_rate_limit: 0.0,
                armed,
                counter: self.counter,
            };
            self.bench
                .receive(spine_core::protocol::DRIVE_CMD_ID, &cmd.encode());
        }

        /// `seconds` of 10 ms ticks, with a drive command every 20 ms (50 Hz).
        fn run(&mut self, seconds: f32, cmd: Option<(f32, f32, bool)>) -> Vec<Frame> {
            let mut frames = Vec::new();
            for i in 0..(seconds * 100.0) as usize {
                if let (Some((v, d, armed)), true) = (cmd, i % 2 == 0) {
                    self.drive(v, d, armed);
                }
                frames.extend(self.bench.tick(10));
            }
            frames
        }

        /// What spine_bridge's /spine/arm does: armed=0 frames, then armed=1.
        fn arm(&mut self) {
            self.run(0.1, Some((0.0, 0.0, false)));
            self.run(0.1, Some((0.0, 0.0, true)));
        }

        fn last_status(frames: &[Frame]) -> VehicleStatus {
            let (_, data) = frames
                .iter()
                .rev()
                .find(|(id, _)| *id == VEHICLE_STATUS_ID)
                .unwrap();
            VehicleStatus::decode(data).unwrap()
        }
    }

    #[test]
    fn boots_latched_and_does_not_move_without_the_arming_edge() {
        let mut rig = Rig::new();
        let frames = rig.run(2.0, Some((1.0, 0.0, true)));
        assert_eq!(rig.bench.pose().speed, 0.0);
        assert!(!Rig::last_status(&frames).armed);
    }

    #[test]
    fn armed_it_drives_to_the_target_speed_through_the_speed_loop() {
        let mut rig = Rig::new();
        rig.arm();
        let frames = rig.run(8.0, Some((1.0, 0.0, true)));
        let v = rig.bench.pose().speed;
        assert!((v - 1.0).abs() < 0.15, "settles near 1 m/s: {v}");
        let status = Rig::last_status(&frames);
        assert_eq!(status.mode, Mode::Computer);
        assert!(status.armed);
        assert!(
            rig.bench.pose().x > 4.0,
            "it went somewhere: {}",
            rig.bench.pose().x
        );
    }

    /// The highest speed on the way to 1 m/s, and the speed after 25 s.
    fn peak_and_end(spine: SpineConfig) -> (f32, f32) {
        let mut rig = Rig::with_spine(spine);
        rig.arm();
        let mut peak = 0.0f32;
        for _ in 0..250 {
            rig.run(0.1, Some((1.0, 0.0, true)));
            peak = peak.max(rig.bench.pose().speed);
        }
        (peak, rig.bench.pose().speed)
    }

    #[test]
    fn the_smooth_speed_loop_does_not_overshoot_on_the_bench() {
        // TODO 27: the spine's own loop, through the encoder, on the bench's model car.
        let (peak, end) = peak_and_end(SpineConfig::default());
        assert!(peak > 1.05, "today's gains overshoot: {peak}");
        assert!((end - 1.0).abs() < 0.05, "{end}");
        let smooth = SpineConfig {
            speed_loop: spine_core::speed_loop::SpeedLoopProfile::Smooth.config(),
            ..SpineConfig::default()
        };
        let (peak, end) = peak_and_end(smooth);
        assert!(peak < 1.03, "smooth: {peak}");
        assert!((end - 1.0).abs() < 0.05, "smooth: {end}");
    }

    fn with_profile(profile: spine_core::speed_loop::SpeedLoopProfile) -> SpineConfig {
        SpineConfig {
            speed_loop: profile.config(),
            ..SpineConfig::default()
        }
    }

    /// Drive at 1 m/s for 8 s, then ask for 0: (metres rolled, seconds to stand still).
    fn stop_from_1(spine: SpineConfig) -> (f32, f32) {
        let mut rig = Rig::with_spine(spine);
        rig.arm();
        rig.run(8.0, Some((1.0, 0.0, true)));
        let x = rig.bench.pose().x;
        let mut t = 0.0;
        while rig.bench.pose().speed > 0.0 && t < 5.0 {
            rig.run(0.01, Some((0.0, 0.0, true)));
            t += 0.01;
        }
        ((rig.bench.pose().x - x) as f32, t)
    }

    #[test]
    fn the_lqi_reaches_1_m_s_without_overshoot_and_stops_on_the_bench() {
        // TODO 3: spine-core's LQI, through the encoder, on the bench's model car.
        use spine_core::speed_loop::SpeedLoopProfile;
        let (peak, end) = peak_and_end(with_profile(SpeedLoopProfile::Lqi));
        assert!(peak < 1.03, "lqi: {peak}");
        assert!((end - 1.0).abs() < 0.05, "lqi: {end}");
        let (rolled, took) = stop_from_1(with_profile(SpeedLoopProfile::Lqi));
        let (rolled_pi, _) = stop_from_1(SpineConfig::default());
        assert!(took < 2.0, "stopped in {took} s");
        assert!(
            rolled <= rolled_pi + 0.05,
            "lqi {rolled} m, pi {rolled_pi} m"
        );
    }

    /// The bench's scores for each profile: `cargo test -p bench-car scores -- --nocapture`.
    #[test]
    fn scores() {
        for p in spine_core::speed_loop::SpeedLoopProfile::ALL {
            let (peak, end) = peak_and_end(with_profile(p));
            let (rolled, took) = stop_from_1(with_profile(p));
            std::println!(
                "{:8} peak {peak:.3} m/s, at 25 s {end:.3}; stop from 1 m/s: {rolled:.3} m in {took:.2} s",
                p.name()
            );
        }
    }

    #[test]
    fn asked_to_stop_it_stops_instead_of_creeping() {
        let mut rig = Rig::new();
        rig.arm();
        rig.run(8.0, Some((1.0, 0.0, true)));
        rig.run(2.0, Some((0.0, 0.0, true)));
        assert_eq!(rig.bench.pose().speed, 0.0, "stopped within 2 s");
        let x = rig.bench.pose().x;
        rig.run(5.0, Some((0.0, 0.0, true)));
        assert_eq!(rig.bench.pose().x, x, "and held there");
    }

    #[test]
    fn steering_follows_the_command() {
        let mut rig = Rig::new();
        rig.arm();
        rig.run(3.0, Some((1.0, 0.3, true)));
        assert!((rig.bench.pose().steering - 0.3).abs() < 0.02);
        assert!(rig.bench.pose().yaw > 0.0, "turning left");
    }

    #[test]
    fn lost_commands_brake_the_car_and_latch_it() {
        let mut rig = Rig::new();
        rig.arm();
        rig.run(5.0, Some((1.0, 0.0, true)));
        assert!(rig.bench.pose().speed > 0.5);
        let frames = rig.run(3.0, None);
        assert_eq!(rig.bench.pose().speed, 0.0, "the watchdog braked");
        let status = Rig::last_status(&frames);
        assert!(!status.armed);
        assert!(status.faults & faults::CMD_TIMEOUT != 0);
    }

    #[test]
    fn the_software_estop_brakes() {
        let mut rig = Rig::new();
        rig.arm();
        rig.run(5.0, Some((1.0, 0.0, true)));
        let estop = EstopCmd {
            estop_request: true,
            counter: 1,
        };
        rig.bench
            .receive(spine_core::protocol::ESTOP_CMD_ID, &estop.encode());
        let frames = rig.run(2.0, Some((1.0, 0.0, true)));
        assert_eq!(rig.bench.pose().speed, 0.0);
        assert!(Rig::last_status(&frames).estop_active);
    }

    #[test]
    fn the_hardware_estop_and_a_driver_fault_brake_too() {
        for which in 0..2 {
            let mut rig = Rig::new();
            rig.arm();
            rig.run(5.0, Some((1.0, 0.0, true)));
            if which == 0 {
                rig.bench.set_hw_estop(true);
            } else {
                rig.bench.set_driver_faults(faults::STEERING_DRIVER);
            }
            rig.run(2.0, Some((1.0, 0.0, true)));
            assert_eq!(rig.bench.pose().speed, 0.0, "case {which}");
        }
    }

    #[test]
    fn a_bad_checksum_is_reported_and_ignored() {
        let mut rig = Rig::new();
        let mut data = DriveCmd {
            target_speed: 1.0,
            steering_angle: 0.0,
            accel_limit: 0.0,
            steer_rate_limit: 0.0,
            armed: false,
            counter: 1,
        }
        .encode();
        data[7] ^= 0xFF;
        rig.bench.receive(spine_core::protocol::DRIVE_CMD_ID, &data);
        let frames = rig.run(0.1, None);
        assert!(Rig::last_status(&frames).faults & faults::BAD_CHECKSUM != 0);
    }

    #[test]
    fn frames_go_out_at_50_10_and_100_hz() {
        let mut rig = Rig::new();
        let frames = rig.run(1.0, None);
        let count = |id| frames.iter().filter(|(i, _)| *i == id).count();
        assert_eq!(count(VEHICLE_FEEDBACK_ID), 50);
        assert_eq!(count(VEHICLE_STATUS_ID), 10);
        assert_eq!(count(WHEEL_SPEED_L_ID), 100);
        assert_eq!(count(WHEEL_SPEED_R_ID), 100);
    }

    #[test]
    fn the_wheel_nodes_measure_the_model_car() {
        let mut rig = Rig::new();
        rig.arm();
        let frames = rig.run(8.0, Some((1.0, 0.0, true)));
        let truth = rig.bench.pose().speed;
        for wheel in [Wheel::Left, Wheel::Right] {
            let (_, data) = frames
                .iter()
                .rev()
                .find(|(id, _)| *id == wheel.id())
                .expect("a wheel frame");
            let w = WheelSpeed::decode(wheel, data).expect("a valid frame");
            assert!(!w.direction_known);
            assert!(
                (w.wheel_speed - truth).abs() < 0.1,
                "{wheel:?}: {} m/s, the car {truth}",
                w.wheel_speed
            );
            // 8 m or so at 0.118 m a hole.
            assert!(w.pulse_count > 40, "{}", w.pulse_count);
        }
    }

    /// The spine driving a car that something else moves (Gazebo): the same logic.
    #[test]
    fn a_measured_car_gets_the_same_spine_and_its_watchdog() {
        use crate::MeasuredCar;
        let mut bench = Bench::with_car(
            SpineConfig::default(),
            MeasuredCar::new(100.0),
            BENCH_WHEELS,
        )
        .expect("the bench's wheel settings are valid");
        let mut counter = 0u8;
        let mut drive = |bench: &mut Bench<MeasuredCar>, armed: bool| {
            counter = (counter + 1) & 0x0F;
            let cmd = DriveCmd {
                target_speed: 1.0,
                steering_angle: 0.2,
                accel_limit: 0.0,
                steer_rate_limit: 0.0,
                armed,
                counter,
            };
            bench.receive(spine_core::protocol::DRIVE_CMD_ID, &cmd.encode());
        };
        // Arm on the edge, then drive for a second: the car is measured still, so the
        // speed loop pushes.
        for i in 0..120 {
            drive(&mut bench, i >= 10);
            bench.tick(10);
        }
        let out = bench.outputs().expect("it ran");
        assert!(
            out.armed && out.throttle > 0.0 && out.brake == 0.0,
            "{out:?}"
        );
        assert!(out.steering_target > 0.0, "steering toward 0.2: {out:?}");

        // The car moves (as Gazebo says): the wheel nodes count it.
        bench.car_mut().measure_speed(1.0);
        bench.car_mut().measure_front_wheels([1.0, 1.0]);
        let mut frames = Vec::new();
        for _ in 0..100 {
            drive(&mut bench, true);
            frames.extend(bench.tick(10));
        }
        let (_, data) = frames
            .iter()
            .rev()
            .find(|(id, _)| *id == WHEEL_SPEED_L_ID)
            .expect("a wheel frame");
        let w = WheelSpeed::decode(Wheel::Left, data).expect("a valid frame");
        assert!((w.wheel_speed - 1.0).abs() < 0.15, "{}", w.wheel_speed);

        // No commands for 100 ms: the watchdog brakes, and latches.
        for _ in 0..12 {
            bench.tick(10);
        }
        let out = bench.outputs().expect("it ran");
        assert!(
            !out.armed && out.throttle == 0.0 && out.brake > 0.5,
            "{out:?}"
        );
        assert!(out.faults & faults::CMD_TIMEOUT != 0);
    }
}

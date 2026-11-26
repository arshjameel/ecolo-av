//! The car on stands ("on jacks"): what the lifted car does, and when it may come down.
//!
//! The real stands test (docs/testing-plan.md, phase A) runs with all four wheels off the
//! ground: the motor spins the rear wheels, the steering turns, and the car stays where
//! it is. The simulator does the same when its [`Stance`] is [`Stance::OnStands`]:
//!
//! | Sensor | On stands it says | Why |
//! |---|---|---|
//! | the driven (rear) wheels | spinning, at the speed the car model gives | the motor turns them |
//! | the free (front) wheels | still | nothing turns them: the car is rear-wheel drive |
//! | the steering joints | the commanded angle, through the same lags | the linkage doesn't care |
//! | the wheels' odometry | moving, if the speed sensors are on the driven wheels ([`SpeedWheels::Rear`]); 0 if on the front | it only counts wheel turns |
//! | the truth, the IMU, the GPS, the lidar | still | the body doesn't move |
//!
//! The wheels and the rest disagree on purpose, as on the real car: nothing here hides it.
//!
//! The rule for the switch ([`switch`]): the car goes up or comes down only standing
//! still with no speed asked for, so it can never leap forward off the stands.
//! Units: m/s. No ROS here.

use std::fmt;
use std::str::FromStr;

/// Where the car's wheels are.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Stance {
    /// On the ground: the wheels move the car.
    #[default]
    OnGround,
    /// On stands: the wheels spin free, the body stays put.
    OnStands,
}

impl Stance {
    /// The stance for the node's `on_stands` flag.
    #[must_use]
    pub fn from_on_stands(on_stands: bool) -> Self {
        if on_stands {
            Stance::OnStands
        } else {
            Stance::OnGround
        }
    }

    /// Whether the wheels are off the ground.
    #[must_use]
    pub fn is_on_stands(self) -> bool {
        self == Stance::OnStands
    }
}

impl fmt::Display for Stance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Stance::OnGround => "on the ground",
            Stance::OnStands => "on stands",
        })
    }
}

/// Which wheels carry the speed sensors (vehicle.yaml `speed_wheels`). The car is
/// rear-wheel drive, so on stands only [`SpeedWheels::Rear`] sees the motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpeedWheels {
    /// The free wheels: today's car (the Arduino's two sensors). On stands they read 0.
    Front,
    /// The driven wheels: the car the stands test needs (TODO 49b).
    #[default]
    Rear,
}

/// A `speed_wheels` that is neither `front` nor `rear`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownSpeedWheels(pub String);

impl fmt::Display for UnknownSpeedWheels {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "speed_wheels:={}: front or rear", self.0)
    }
}

impl std::error::Error for UnknownSpeedWheels {}

impl FromStr for SpeedWheels {
    type Err = UnknownSpeedWheels;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "front" => Ok(SpeedWheels::Front),
            "rear" => Ok(SpeedWheels::Rear),
            other => Err(UnknownSpeedWheels(other.to_string())),
        }
    }
}

/// m/s: slower than this is standing still, for the wheels, the car and a command.
pub const STILL: f64 = 0.02;

/// Why the car can't go on or off the stands now.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Refusal {
    /// Coming down: the wheels still spin at this speed (m/s at the tyre).
    WheelsSpinning(f64),
    /// Going up: the car still moves at this speed (m/s).
    CarMoving(f64),
    /// Either way: this speed (m/s) is still asked for.
    SpeedCommanded(f64),
    /// A speed that isn't a number: nothing is known, so nothing moves.
    NotFinite,
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refusal::WheelsSpinning(v) => write!(
                f,
                "the wheels still spin at {v:+.2} m/s: stop them first, or the car would \
                 leap off the stands"
            ),
            Refusal::CarMoving(v) => {
                write!(
                    f,
                    "the car moves at {v:+.2} m/s: stop it before it goes on stands"
                )
            }
            Refusal::SpeedCommanded(v) => write!(
                f,
                "a speed of {v:+.2} m/s is still commanded: send zero first"
            ),
            Refusal::NotFinite => f.write_str("the speed is not a number: nothing is changed"),
        }
    }
}

impl std::error::Error for Refusal {}

/// What a request to switch did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Switched {
    /// The car was like that already: nothing changed.
    Already(Stance),
    /// The car is now like this.
    Now(Stance),
}

/// May the car go from `now` to `wanted`? `speed`: the wheels' speed at the tyre (on the
/// ground, the car's); `commanded`: the speed the simulator is told to follow (m/s).
///
/// # Errors
///
/// [`Refusal`]: the wheels spin, the car moves, or a speed is still commanded (each over
/// [`STILL`]), or a speed is not a number. Asking for the stance it has is never refused.
pub fn switch(
    now: Stance,
    wanted: Stance,
    speed: f64,
    commanded: f64,
) -> Result<Switched, Refusal> {
    if now == wanted {
        return Ok(Switched::Already(now));
    }
    if !speed.is_finite() || !commanded.is_finite() {
        return Err(Refusal::NotFinite);
    }
    if speed.abs() > STILL {
        return Err(match wanted {
            Stance::OnGround => Refusal::WheelsSpinning(speed),
            Stance::OnStands => Refusal::CarMoving(speed),
        });
    }
    if commanded.abs() > STILL {
        return Err(Refusal::SpeedCommanded(commanded));
    }
    Ok(Switched::Now(wanted))
}

/// What the lifted car's wheels do at the car model's speed (m/s at the tyre).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FreeWheels {
    /// m/s at the tyre: rear left, rear right, front left, front right
    /// ([`crate::sim::Joints::WHEELS`]' order).
    pub tyre_speeds: [f64; 4],
    /// m/s: what the wheels' odometry reads.
    pub sensed_speed: f64,
}

impl FreeWheels {
    /// The driven rear wheels turn together at `speed` (straight or steered: nothing
    /// pulls one round faster than the other); the free front wheels stand.
    #[must_use]
    pub fn at(speed: f64, sensors: SpeedWheels) -> Self {
        Self {
            tyre_speeds: [speed, speed, 0.0, 0.0],
            sensed_speed: match sensors {
                SpeedWheels::Rear => speed,
                SpeedWheels::Front => 0.0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_car_comes_down_only_with_still_wheels_and_no_speed_asked() {
        let down = |speed, commanded| switch(Stance::OnStands, Stance::OnGround, speed, commanded);
        assert_eq!(down(0.0, 0.0), Ok(Switched::Now(Stance::OnGround)));
        assert_eq!(down(0.01, -0.01), Ok(Switched::Now(Stance::OnGround)));
        assert_eq!(down(0.3, 0.0), Err(Refusal::WheelsSpinning(0.3)));
        assert_eq!(down(-0.3, 0.0), Err(Refusal::WheelsSpinning(-0.3)));
        // Still for now, but told to go: it would drive off the moment it touched down.
        assert_eq!(down(0.0, 0.2), Err(Refusal::SpeedCommanded(0.2)));
        assert_eq!(down(f64::NAN, 0.0), Err(Refusal::NotFinite));
        assert_eq!(down(0.0, f64::INFINITY), Err(Refusal::NotFinite));
    }

    #[test]
    fn the_car_goes_up_only_standing_still() {
        let up = |speed, commanded| switch(Stance::OnGround, Stance::OnStands, speed, commanded);
        assert_eq!(up(0.0, 0.0), Ok(Switched::Now(Stance::OnStands)));
        assert_eq!(up(1.0, 1.0), Err(Refusal::CarMoving(1.0)));
        assert_eq!(up(0.0, 1.0), Err(Refusal::SpeedCommanded(1.0)));
    }

    #[test]
    fn asking_for_what_it_is_changes_nothing_and_is_never_refused() {
        assert_eq!(
            switch(Stance::OnStands, Stance::OnStands, 3.0, 3.0),
            Ok(Switched::Already(Stance::OnStands))
        );
        assert_eq!(
            switch(Stance::OnGround, Stance::OnGround, 3.0, f64::NAN),
            Ok(Switched::Already(Stance::OnGround))
        );
    }

    #[test]
    fn a_refusal_says_why_in_words() {
        let why = Refusal::WheelsSpinning(0.3).to_string();
        assert!(why.contains("+0.30 m/s") && why.contains("leap"), "{why}");
        assert!(Refusal::SpeedCommanded(-0.2).to_string().contains("-0.20"));
    }

    #[test]
    fn on_stands_the_rear_wheels_spin_and_the_front_stand() {
        let rear = FreeWheels::at(0.3, SpeedWheels::Rear);
        assert_eq!(rear.tyre_speeds, [0.3, 0.3, 0.0, 0.0]);
        assert_eq!(rear.sensed_speed, 0.3);
        // Today's car: its sensors are on the front, which nothing turns.
        assert_eq!(FreeWheels::at(0.3, SpeedWheels::Front).sensed_speed, 0.0);
    }

    #[test]
    fn speed_wheels_reads_vehicle_yamls_words() {
        assert_eq!("front".parse(), Ok(SpeedWheels::Front));
        assert_eq!("rear".parse(), Ok(SpeedWheels::Rear));
        let bad = "both".parse::<SpeedWheels>().unwrap_err();
        assert_eq!(bad.to_string(), "speed_wheels:=both: front or rear");
        assert_eq!(Stance::from_on_stands(true), Stance::OnStands);
        assert!(!Stance::from_on_stands(false).is_on_stands());
        assert_eq!(Stance::OnStands.to_string(), "on stands");
    }
}

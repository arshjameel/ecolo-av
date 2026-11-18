"""
Ego's Ackermann command as a CARLA car's controls, and back. No ROS, no `carla`.

Ego sends a speed (m/s) and the virtual centre wheel's angle (rad, + left), sized for OUR
car (vehicle.yaml: wheelbase, max_steering_angle). A CARLA car is bigger, so the same
angle would turn it on a wider circle. match_curvature() keeps the circle instead: the
CARLA car turns on the radius ours would.

Two ways to drive the CARLA car (carla_adapter's control parameter):

  ackermann  VehicleAckermannControl (CARLA 0.9.14 and later): speed, acceleration and a
             steering angle; CARLA's own speed loop works the pedals. The default.
  pedals     VehicleControl: throttle, brake and steer in -1..1, from our SpeedLoop here.
             For a CARLA version without the Ackermann controller, or to tune our own.

CARLA steers + right (Unreal's y points right); ours is + left. Both flip it here.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

# The cars to try when none is named, best first: the smallest (closest to our ~2 m car)
# first, then cars every CARLA has (0.9.x and the Unreal Engine 5 0.10.x).
CARS = ('vehicle.micro.microlino', 'vehicle.mini.cooper_s_2021', 'vehicle.mini.cooper',
        'vehicle.lincoln.mkz_2020', 'vehicle.lincoln.mkz')


def pick_car(available, wanted: str = '') -> str:
    """
    Return the car blueprint to spawn.

    The car blueprint to spawn: wanted if CARLA has it, else the first of CARS it has.

    available: the blueprint ids CARLA offers (world.get_blueprint_library()).
    """
    ids = set(available)
    if wanted:
        if wanted not in ids:
            raise ValueError(f'CARLA has no {wanted}; it has e.g. '
                             f'{sorted(i for i in ids if i.startswith("vehicle."))[:8]}')
        return wanted
    for car in CARS:
        if car in ids:
            return car
    raise ValueError(f'CARLA has none of {CARS}: name one with vehicle:=')


def match_curvature(steer: float, wheelbase: float, carla_wheelbase: float) -> float:
    """Return our angle (rad) as the one that turns the CARLA car on our circle."""
    return math.atan(math.tan(steer) * carla_wheelbase / wheelbase)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Geometry:
    """Both cars' steering: ours from vehicle.yaml, CARLA's from its physics control."""

    wheelbase: float           # m, ours
    max_steer: float           # rad, ours
    carla_wheelbase: float     # m
    carla_max_steer: float     # rad: the CARLA car's front wheels' limit

    def carla_angle(self, steer: float) -> float:
        """Return ego's angle (rad, + left) as the CARLA car's (rad, + RIGHT)."""
        ours = clamp(steer, -self.max_steer, self.max_steer)
        theirs = match_curvature(ours, self.wheelbase, self.carla_wheelbase)
        return -clamp(theirs, -self.carla_max_steer, self.carla_max_steer)

    def carla_steer(self, steer: float) -> float:
        """Return ego's angle as VehicleControl.steer: -1 (full left) .. 1 (right)."""
        return clamp(self.carla_angle(steer) / self.carla_max_steer, -1.0, 1.0)

    def our_angle(self, left_deg: float, right_deg: float) -> float:
        """
        Return the CARLA car's measured steering as our angle.

        The CARLA car's front wheels (Vehicle.get_wheel_steer_angle, degrees, + right) as
        our virtual centre wheel (rad, + left): what /vehicle/steering measures.
        """
        theirs = -math.radians((left_deg + right_deg) / 2)
        return match_curvature(theirs, self.carla_wheelbase, self.wheelbase)


@dataclass(frozen=True)
class Ackermann:
    """The fields of carla.VehicleAckermannControl (steer + right, rad; m/s; m/s^2)."""

    steer: float
    steer_speed: float
    speed: float
    acceleration: float
    jerk: float


def ackermann(geometry: Geometry, speed: float, steer: float, acceleration: float = 0.0,
              steer_speed: float = 0.0, max_accel: float = 1.5) -> Ackermann:
    """
    Return ego's command as VehicleAckermannControl's fields.

    Ego's command as VehicleAckermannControl. 0 means "as fast as you can" to CARLA, for
    the acceleration and the steering speed, so ego's 0 becomes our car's own limit
    (vehicle.yaml actuators: max_accel; the steering speed is left to CARLA).
    """
    accel = abs(acceleration) if acceleration else max_accel
    return Ackermann(steer=geometry.carla_angle(steer), steer_speed=abs(steer_speed),
                     speed=speed, acceleration=accel, jerk=0.0)


@dataclass(frozen=True)
class Pedals:
    """The fields of carla.VehicleControl that we set."""

    throttle: float
    brake: float
    steer: float
    reverse: bool
    hand_brake: bool = False


@dataclass
class SpeedLoop:
    """
    Hold a speed with throttle and brake: a PI loop.

    A PI speed loop for the pedals mode: the stand-in for the spine's, as in Ravensim.
    Throttle (0..1) to speed up, brake (0..1) to slow; in reverse below zero.
    """

    kp: float = 0.5            # throttle per m/s of error
    ki: float = 0.1            # throttle per m of error (per m/s per second)
    brake_gain: float = 0.5    # brake per m/s of overspeed
    integral: float = 0.0

    def step(self, target: float, speed: float, dt: float) -> tuple[float, float, bool]:
        """Return throttle, brake and reverse for this target and measured speed (m/s)."""
        reverse = target < 0.0 or (target == 0.0 and speed < -0.05)
        # Work in the direction of travel: forwards, or backwards in reverse.
        want = abs(target)
        have = -speed if reverse else speed
        error = want - have
        if want < 0.05 and abs(have) < 0.05:
            self.integral = 0.0
            return (0.0, 1.0, reverse)          # stopped: hold it
        if error >= 0.0:
            self.integral = clamp(self.integral + error * dt, 0.0, 1.0 / self.ki)
            return (clamp(self.kp * error + self.ki * self.integral, 0.0, 1.0), 0.0, reverse)
        self.integral = clamp(self.integral + error * dt, 0.0, 1.0 / self.ki)
        return (0.0, clamp(-self.brake_gain * error, 0.0, 1.0), reverse)


def pedals(geometry: Geometry, loop: SpeedLoop, target: float, steer: float,
           speed: float, dt: float) -> Pedals:
    """Return ego's command as VehicleControl, through the speed loop."""
    throttle, brake, reverse = loop.step(target, speed, dt)
    return Pedals(throttle=throttle, brake=brake, steer=geometry.carla_steer(steer),
                  reverse=reverse)


def stopped() -> Pedals:
    """Return the e-stop: full brake and the hand brake, wheels straight."""
    return Pedals(throttle=0.0, brake=1.0, steer=0.0, reverse=False, hand_brake=True)


@dataclass
class Watchdog:
    """No command for timeout s: stop (target speed 0), as the spine's watchdog does."""

    timeout: float = 0.2
    last: float | None = None

    def fed(self, now: float) -> None:
        self.last = now

    def fresh(self, now: float) -> bool:
        return self.last is not None and now - self.last <= self.timeout

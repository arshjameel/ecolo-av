"""
The car on stands in Ravensim: free wheels, without the car's weight on them. No ROS.

Ravensim's simulator lifts the car (aav_simulator, sim-core jacks.rs: the body stays put,
the rear wheels spin) and says so on /vehicle/sim/jacks. sim_adapter's actuator model then
has to answer as free wheels do: the same throttle spins them up far faster, and the
brake stops them almost at once, because the motor and the brake no longer move the
car's mass, only the wheels and the drive train.

The car model already has the rule for a different mass (car_model.py: share =
mass / (mass + payload), which scales the throttle's push and the brake). On stands the
mass the motor moves is the spinning parts' own, as if it rode on the tyres:
`free_wheel_mass`, kg. So on stands payload = free_wheel_mass - mass (negative: weight
taken off), and share = mass / free_wheel_mass. With 181 kg and 20 kg: 9 times the push.

`FREE_WHEEL_MASS`, 20 kg, is A GUESS: two rear wheels, the axle, and the motor through
its gearing. Nobody has measured it. To measure: on the real stands, time the wheels from
0 to a known speed at a known throttle.

What stays as on the ground: the lags, the dead times, the steering, and `drag` (1/s).
The real free wheels' drag is the drive train's own, not the tyres' on the road: not
measured either, so not guessed twice.
"""

from dataclasses import replace
import math

FREE_WHEEL_MASS = 20.0   # kg. GUESS: see the table above


def check_free_wheel_mass(mass):
    """Return why this free-wheel mass (kg) can't be, or None."""
    if not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0.0:
        return f'free_wheel_mass must be above 0 kg, not {mass}'
    return None


def free_wheels(config, free_wheel_mass=FREE_WHEEL_MASS):
    """
    Return the car model's numbers with the car on stands.

    config: the car on the ground (car_model.ActuatorConfig, any payload: what is aboard
    doesn't load wheels that are off the ground). Only its payload changes, so its share
    becomes mass / free_wheel_mass.
    """
    why = check_free_wheel_mass(free_wheel_mass)
    if why:
        raise ValueError(why)
    return replace(config, payload=float(free_wheel_mass) - config.mass)

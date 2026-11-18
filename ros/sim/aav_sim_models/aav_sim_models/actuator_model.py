"""
The actuator model's old import path.

It moved to aav_description.car_model on 7 Nov (one car model for ego and the simulators).
Every name here is the same object as there.
"""

from aav_description.car_model import (  # noqa: F401
    ActuatorConfig, Actuators, CarState, clamp, Command, delay, delay_steps, Demand, Ideal,
    IGNORED, lag, LQI_MODEL, LqiConfig, predict, speed_loop_profile, SPEED_LOOP_PROFILES,
    SpeedLoop, SpeedLoopConfig, State, step,
)

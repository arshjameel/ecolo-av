"""
The RC transmitter as a ROS driver (TODO 48): its sticks to a command. No ROS here.

  the ESP32 reads the receiver ─uplink─> legacy_uart_bridge ─/vehicle/rc─> RcDrive
       ─> (v, w) ─/drivers/rc/cmd_vel─> twist_mux (priority 95) ─> ego ─> the output mux
       ─> the Motor Pi's frames, with the Orin's counter ─> the ESP32 drives

The transmitter's mode switch (CH6; the three positions are a proposal: docs/decisions.md):

  computer     the computer drives (Nav2, ...)               this driver: nothing
  RC via ROS   a person, with the sticks, through ego         a command every tick, zeros
                                                              too: nothing below it drives
  RC direct    a person, straight from the ESP32 (fallback)   nothing: the ESP32 ignores
                                                              the computer then
  no signal    nobody: the ESP32 brakes (fault 256)           nothing

Leaving RC via ROS: one zero, then nothing (as teleop_twist_joy does on its deadman).
/vehicle/rc gone quiet while on RC via ROS: zeros, a stop, until it comes back; the
person can still take over with RC direct, which needs no computer.

The sticks mean what they mean in RC direct (ibus RcMap): the left stick forward or
back, its brake band (the middle 20 % each way) a stop, the steering stick a fraction of
full lock. Through ego the band is a stop at ego's rates, not the ESP32's full brake.
The steering becomes a turn rate by the bicycle model, as ego expects; so the car can't
steer while it stands still (ego holds the wheels below 0.05 m/s), unlike RC direct.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

# aav_interfaces/RcSticks's mode_switch.
SWITCH_NONE, SWITCH_COMPUTER, SWITCH_RC_VIA_ROS, SWITCH_RC_DIRECT = 0, 1, 2, 3


@dataclass
class Limits:
    max_speed: float = 1.0           # m/s at full stick forward: the joystick's
    max_reverse_speed: float = 0.5   # m/s at full stick in reverse (vehicle.yaml)
    max_steering_angle: float = 0.5  # rad at full stick (vehicle.yaml)
    wheelbase: float = 1.57          # m (vehicle.yaml)
    stale: float = 0.2               # s without /vehicle/rc: the sticks are stale


@dataclass(frozen=True)
class Sticks:
    """One /vehicle/rc message."""

    switch: int        # SWITCH_*
    throttle: float    # -1..1, + forward
    steering: float    # -1..1, + left
    brake: bool        # the left stick in its brake band


def clamp1(x: float) -> float:
    if math.isnan(x):
        return 0.0
    return max(-1.0, min(1.0, x))


def twist(s: Sticks, lim: Limits) -> Tuple[float, float]:
    """Return (v m/s, w rad/s) for the sticks: the bicycle model's turn rate."""
    throttle = 0.0 if s.brake else clamp1(s.throttle)
    v = throttle * (lim.max_speed if throttle >= 0 else lim.max_reverse_speed)
    angle = clamp1(s.steering) * lim.max_steering_angle
    return v, v * math.tan(angle) / lim.wheelbase


class RcDrive:
    """The driver's state: the latest sticks, and whether it was driving."""

    def __init__(self, limits: Limits):
        self.limits = limits
        self.sticks: Optional[Sticks] = None
        self.at: Optional[float] = None
        self.driving = False    # the last sticks heard were on RC via ROS

    def heard(self, sticks: Sticks, now: float) -> None:
        """Note one /vehicle/rc message, at `now` (s)."""
        self.sticks = sticks
        self.at = now

    def command(self, now: float) -> Optional[Tuple[float, float]]:
        """Return this tick's (v, w), or None: publish nothing."""
        fresh = self.at is not None and now - self.at <= self.limits.stale
        if not fresh:
            # Gone quiet on RC via ROS: hold a stop. Never heard, or not driving: nothing.
            return (0.0, 0.0) if self.driving else None
        if self.sticks.switch == SWITCH_RC_VIA_ROS:
            self.driving = True
            return twist(self.sticks, self.limits)
        if self.driving:
            self.driving = False
            return 0.0, 0.0     # one zero as the switch leaves RC via ROS
        return None

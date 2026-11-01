"""
The stop hold: once collision_monitor stops the car, keep it stopped until the way stays clear.

No ROS here, so the tests run anywhere.

Nav2's collision_monitor (Jazzy, 1.3.13) decides afresh on every command: three points in
the stop zone, stop; two, go. An obstacle at the zone's edge, with a sensor's noise on it,
flickers between the two, and each go lets the car creep on (TODO, 11 Oct). It has no
setting to hold a stop: none of its parameters looks at what it decided before.

StopHold adds that memory. It is locked from the moment the monitor says STOP, and lets go
only once the monitor has said something else for `clear_time` seconds without a break.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class StopHold:
    """Locked from a stop until `clear_time` s of no stop; times in s, any clock."""

    clear_time: float = 1.0
    stopped: bool = False
    clear_since: Optional[float] = None    # when the last stop ended; None while stopped

    def __post_init__(self):
        if not self.clear_time >= 0.0 or math.isinf(self.clear_time):
            raise ValueError('clear_time must be a number of seconds >= 0')

    def update(self, now: float, stop: bool) -> None:
        """Take the monitor's state at `now`: `stop` is true while it stops the car."""
        if stop:
            self.stopped, self.clear_since = True, None
        elif self.stopped and self.clear_since is None:
            self.clear_since = now

    def locked(self, now: float) -> bool:
        """Return whether the hold is on at `now`."""
        if not self.stopped:
            return False
        if self.clear_since is not None and now - self.clear_since >= self.clear_time:
            self.stopped, self.clear_since = False, None
            return False
        return True

"""
The goal leash (TODO 116): a goal's owner that goes quiet loses its goals. No ROS here.

The language driver (aav_language) runs on a laptop and sends goals to drivers on the car
(/scripted_drive, /follow_gps_waypoints). A ROS action carries on when its client
vanishes, so a laptop dropping off the WiFi mid-drive left the car driving a goal
nobody could see or stop but the safety driver. The owner beats; the leash, on the car,
trips once when the beats stop:

    beats (1, 2, 3...) ─► armed ─► silent over `timeout` s ─► trip: cancel the goals
    beat 0 (goodbye: the owner closed cleanly, cancelling its own) ─► disarmed

A trip is a cancel, so the car slows to a stop the driver's own way: not an e-stop (a
laptop dropping off is no reason to brake hard on the road). It re-arms on the next beat.
"""

from dataclasses import dataclass, field
from typing import Optional

GOODBYE = 0


@dataclass
class Leash:
    """Watches one owner's beats. `timeout` s of silence while armed trips it once."""

    timeout: float = 1.0
    last: Optional[float] = field(default=None)

    @property
    def armed(self) -> bool:
        return self.last is not None

    def beat(self, value: int, now: float) -> None:
        """Take a beat from the owner: a count arms it, GOODBYE disarms it (nothing cancelled)."""
        self.last = None if value == GOODBYE else now

    def check(self, now: float) -> bool:
        """Return True once, the moment an armed owner has been silent over `timeout` s."""
        if self.last is None or now - self.last <= self.timeout:
            return False
        self.last = None
        return True

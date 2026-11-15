"""
Clouds waiting for their transform: tf2's MessageFilter, in short, with no ROS.

A cloud may come a little before /tf has the car's pose at its stamp ("Lookup would require
extrapolation into the future"). Asking again later is the usual ROS answer: keep the
cloud, oldest first, and use it once the transform is there. Never block a callback to
wait: in a one-thread node, /tf can't arrive while it waits.

    stamp  <=  newest transform        found: AT_STAMP          not found: DROP (too old)
    newest <   stamp <= newest + tol   NEWEST: the newest pose stands in
    stamp  >   newest + tolerance      WAIT; DROP once older than max_wait
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Optional


class Step(Enum):
    """What to do with one waiting cloud."""

    AT_STAMP = 'at_stamp'   # the transform at its stamp is there
    NEWEST = 'newest'       # a hair newer than /tf: the newest transform stands in
    WAIT = 'wait'           # too new: ask again later
    DROP = 'drop'           # never coming: too old for the buffer, or waited too long


@dataclass(frozen=True)
class WaitShape:
    tolerance: float = 0.1  # s: a stamp this much newer than the newest transform uses it
    max_wait: float = 1.5   # s on the node's clock: a cloud this old is dropped
    size: int = 30          # clouds kept waiting (3 s of the 10 Hz cubes); more: oldest out


def decide(stamp: float, newest: Optional[float], found: bool, now: float,
           shape: WaitShape) -> Step:
    """
    Return the step for one cloud.

    stamp: the cloud's (s); newest: the newest transform's (s), None if none yet; found:
    the transform at the stamp is there; now: the node's clock (s).
    """
    if found:
        return Step.AT_STAMP
    if newest is not None and stamp <= newest:
        return Step.DROP                      # older than the buffer, or no path between
    if newest is not None and stamp - newest <= shape.tolerance:
        return Step.NEWEST
    return Step.DROP if now - stamp > shape.max_wait else Step.WAIT


class Waiting:
    """One stream's clouds, oldest first, each used once its transform is there."""

    def __init__(self, shape: WaitShape = WaitShape()):
        self.shape = shape
        self.queue: deque = deque()
        self.dropped = 0                      # since the last time the node said so

    def add(self, stamp: float, item: Any) -> None:
        """Keep a cloud; past `size`, the oldest goes."""
        self.queue.append((stamp, item))
        while len(self.queue) > self.shape.size:
            self.queue.popleft()
            self.dropped += 1

    def take(self, look: Callable[[float], tuple], now: float) -> list:
        """
        Return the clouds ready now, oldest first, as (item, Step.AT_STAMP or Step.NEWEST).

        look(stamp, item) -> (found, newest), as decide() takes them. Stops at the first
        cloud still waiting: those after it are newer.
        """
        ready = []
        while self.queue:
            stamp, item = self.queue[0]
            found, newest = look(stamp, item)
            step = decide(stamp, newest, found, now, self.shape)
            if step is Step.WAIT:
                break
            self.queue.popleft()
            if step is Step.DROP:
                self.dropped += 1
            else:
                ready.append((item, step))
        return ready

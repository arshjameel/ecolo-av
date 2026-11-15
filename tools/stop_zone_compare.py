"""
Compare our stop zone in shadow with Nav2's collision_monitor, stop by stop.

Do they stop at the same times? TODO 182.
Both are step functions of time: Nav2's state comes on each change, ours on every
command. No ROS here; sim_check.py --stop-zone-shadow records both and calls compare().

    agree       the share of the time both said the same (stop, or clear)
    delay       s from Nav2's first stop to ours (negative: ours first); None if either
                never stopped
    missed      s Nav2 stopped while ours was clear: the one that matters for safety
    extra       s ours stopped while Nav2 was clear
"""

from typing import NamedTuple, Optional, Sequence, Tuple

Event = Tuple[float, bool]          # (s, stop)


class Comparison(NamedTuple):
    agree: float
    delay: Optional[float]
    missed: float
    extra: float
    nav2_stops: int
    our_stops: int


def at(events: Sequence[Event], t: float) -> bool:
    """Return the state at time t: the newest event's at or before t (clear before any)."""
    state = False
    for when, stop in events:
        if when > t:
            break
        state = stop
    return state


def stops(events: Sequence[Event]) -> int:
    """Return how many times it went from clear to stop."""
    count, state = 0, False
    for _, stop in events:
        if stop and not state:
            count += 1
        state = stop
    return count


def first_stop(events: Sequence[Event]) -> Optional[float]:
    return next((t for t, stop in events if stop), None)


def compare(nav2: Sequence[Event], ours: Sequence[Event], start: float,
            end: float) -> Comparison:
    """Compare the two, in time order, over [start, end]."""
    nav2, ours = sorted(nav2), sorted(ours)
    times = sorted({start, end, *(t for t, _ in nav2), *(t for t, _ in ours)})
    times = [t for t in times if start <= t <= end]
    same = missed = extra = 0.0
    for a, b in zip(times, times[1:]):
        theirs, mine = at(nav2, a), at(ours, a)
        span = b - a
        if theirs == mine:
            same += span
        elif theirs:
            missed += span
        else:
            extra += span
    total = end - start
    f_nav2, f_ours = first_stop(nav2), first_stop(ours)
    delay = f_ours - f_nav2 if f_nav2 is not None and f_ours is not None else None
    return Comparison(same / total if total > 0 else 1.0, delay, missed, extra,
                      stops(nav2), stops(ours))

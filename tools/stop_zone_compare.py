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

Per stop (lags(), travelled()): for each time Nav2 went from clear to stop, how much later
ours stopped (negative or 0: ours already stopped), and the metres the car drove in that
time by a track of (t, x, y, ...): the swap's rule, "ours stops every time Nav2's does, no
later" (TODO 182).
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


def onsets(events: Sequence[Event]) -> list:
    """Return the times it went from clear to stop."""
    out, state = [], False
    for when, stop in sorted(events):
        if stop and not state:
            out.append(when)
        state = stop
    return out


def lags(nav2: Sequence[Event], ours: Sequence[Event]) -> list:
    """
    For each of Nav2's stops, return (its time, s until ours stopped, or None if never).

    Ours already stopping then counts from its own onset (0 or less): it was there first.
    """
    ours = sorted(ours)
    mine = onsets(ours)
    out = []
    for t in onsets(nav2):
        if at(ours, t):
            out.append((t, max((o for o in mine if o <= t), default=t) - t))
        else:
            later = next((o for o in mine if o > t), None)
            out.append((t, None if later is None else later - t))
    return out


def travelled(track: Sequence[tuple], start: float, end: float) -> float:
    """Return the metres along a track of (t, x, y, ...) between two times, either order."""
    sign = 1.0
    if end < start:
        start, end, sign = end, start, -1.0
    points = [p for p in track if start <= p[0] <= end]
    total = sum(((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
                for a, b in zip(points, points[1:]))
    return sign * total

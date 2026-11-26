#!/usr/bin/env python3
"""
How a measured value follows the value asked for: the steps, and three numbers a step.

Given one run's samples (time, asked, measured), find each step in what was asked, and for
each say:

    delay      s from the step until the measured value has moved 10 % of the way
    rise       s from 10 % to 90 % of the way
    off        the measured value at the end of the step (the mean of its last quarter)
               less the value asked for

"The way" is from the measured value just before the step to the value asked for. A
number that the samples do not hold is None, and `why` says what is missing: nothing is
guessed, fitted or smoothed. A time is never finer than the gap between two samples
(`period`).

No ROS, no files: lists in, numbers out. Tested in tools/tests/test_step_response.py.
tools/rung_answer.py uses it for a rung's graphs.
"""
from dataclasses import dataclass
import statistics
from typing import List, Optional, Sequence

START, END = 0.1, 0.9      # the shares of the way that time the delay and the rise
TAIL = 0.25                # the share of a step's samples that say where it ended


@dataclass(frozen=True)
class Step:
    """One step in what was asked: samples [first, last) hold the new value."""

    first: int
    last: int
    asked_from: float
    asked_to: float


@dataclass(frozen=True)
class Response:
    """How the measured value answered one step."""

    step: Step
    at: float                    # s, when the step was asked
    held: float                  # s the new value was held
    before: float                # the measured value just before the step
    delay: Optional[float]
    rise: Optional[float]
    end: float                   # the measured value at the end (mean of the last quarter)
    off: float                   # end - asked_to
    why: str = ''                # why delay or rise is None, in words


def steps(asked: Sequence[float], least: float) -> List[Step]:
    """
    Return the steps in what was asked: a jump of at least `least` from one sample to the next.

    Jumps on consecutive samples are one step (a value written in two parts). A wobble
    smaller than `least` is not a step, and does not move the level the next is judged from.
    """
    found: List[Step] = []
    if not asked:
        return found
    level = asked[0]
    for i in range(1, len(asked)):
        if abs(asked[i] - level) < least:
            continue
        if found and found[-1].first == i - 1:      # still the same step
            found[-1] = Step(found[-1].first, len(asked), found[-1].asked_from, asked[i])
        else:
            if found:
                found[-1] = Step(found[-1].first, i, found[-1].asked_from, found[-1].asked_to)
            found.append(Step(i, len(asked), level, asked[i]))
        level = asked[i]
    return found


def respond(t: Sequence[float], measured: Sequence[float], step: Step,
            least: float) -> Response:
    """Return how `measured` answered one step. `least`: a way shorter than this is not timed."""
    at = t[step.first]
    mine = range(step.first, step.last)
    before = measured[step.first - 1] if step.first else measured[0]
    way = step.asked_to - before
    tail = max(1, round(len(mine) * TAIL))
    end = statistics.fmean(measured[i] for i in mine[-tail:])
    held = t[step.last - 1] - at
    common = {'step': step, 'at': at, 'held': held, 'before': before, 'end': end,
              'off': end - step.asked_to}
    if abs(way) < least:
        return Response(delay=None, rise=None, why='the measured value was already there',
                        **common)
    sign = 1.0 if way > 0 else -1.0

    def reached(share, start):
        return next((i for i in range(start, step.last)
                     if (measured[i] - before) * sign >= share * abs(way)), None)

    started = reached(START, step.first)
    if started is None:
        return Response(delay=None, rise=None, **common,
                        why=f'it did not move 10 % of the way in the {held:.1f} s it had')
    there = reached(END, started)
    if there is None:
        return Response(delay=t[started] - at, rise=None, **common,
                        why=f'it did not get 90 % of the way in the {held:.1f} s it had')
    return Response(delay=t[started] - at, rise=t[there] - t[started], **common)


def responses(t: Sequence[float], asked: Sequence[float], measured: Sequence[float],
              least: float) -> List[Response]:
    """Return every step's response; the three lists are one sample an index."""
    if not (len(t) == len(asked) == len(measured)):
        raise ValueError('time, asked and measured must be the same length')
    return [respond(t, measured, s, least) for s in steps(asked, least)]


def period(t: Sequence[float]) -> Optional[float]:
    """Return the usual gap between two samples (the median), or None with under two."""
    gaps = [b - a for a, b in zip(t, t[1:])]
    return statistics.median(gaps) if gaps else None


@dataclass(frozen=True)
class Spread:
    """Some numbers in three: the middle one, the smallest, the largest."""

    median: float
    low: float
    high: float
    n: int


def spread(values: Sequence[Optional[float]]) -> Optional[Spread]:
    """Return the spread of the values that exist, or None when none does."""
    have = [v for v in values if v is not None]
    if not have:
        return None
    return Spread(statistics.median(have), min(have), max(have), len(have))


def first_below(t: Sequence[float], values: Sequence[float], start: int,
                level: float) -> Optional[float]:
    """Return s from sample `start` until |value| is under `level` and stays so, or None."""
    last_above = next((i for i in range(len(values) - 1, start - 1, -1)
                       if abs(values[i]) >= level), None)
    if last_above is None:
        return 0.0
    if last_above == len(values) - 1:
        return None
    return t[last_above + 1] - t[start]

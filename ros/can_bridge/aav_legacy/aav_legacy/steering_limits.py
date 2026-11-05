"""
The steering actuator's limits, from its measured ends and centre. No ROS imports.

For the hardware panel's calibration page and this module's command line
(`ros2 run aav_legacy steering_limits`, or `python3 -m aav_legacy.steering_limits`), and
the sheet in docs/actuator-tests.md ("Find the steering actuator's ends and centre").

Inputs, in counts (the steering actuator's position: its Jrk's scaled feedback, 0..4095,
the units of its targets): `left` (the wheels at full left lock), `centre` (the wheels
straight) and `right` (full right lock). Optional: the wheel angle at each end, in
degrees, measured with a protractor (positive numbers, each side).

The rules, kept simple so the panel's Rust twin (panel-core) can port them line by line:

  1. travel_left = |left - centre|, travel_right = |right - centre|.
  2. Errors (no limits):
     - |left - right| < MIN_SPAN (100 counts): the feedback doesn't change;
     - left and right on the same side of centre (or equal to it): the centre isn't
       between the ends;
     - a count outside 0..4095.
  3. half = floor(min(travel_left, travel_right) x (1 - margin)), margin 0.05: the same
     travel each side of the centre, the smaller side's, less 5 %.
  4. limits: lo = centre - half, hi = centre + half (counts).
  5. sign = +1 if left > centre (steering left raises the target), else -1.
  6. With both angles: gain_left = travel_left / rad(left_deg), gain_right likewise;
     counts_per_rad = sign x (gain_left + gain_right) / 2;
     max_steering_angle = half / |counts_per_rad| (rad). With one or none: not known.
  7. Warnings (limits still given):
     - |centre - (left + right) / 2| > CENTRE_OFF (10 %) of |left - right|: the centre
       is far from the middle of the travel;
     - min travel / max travel < LOPSIDED (0.8): the travel is lopsided;
     - an end at 0 or 4095: the feedback may be clipped (the Jrk's feedback scaling);
     - the two sides' gains differ by more than GAINS_DIFFER (15 %) of their mean.

The firmware's limits (spine-board's calibration.rs, `Steering`) and vehicle.yaml's
max_steering_angle come from the result (`firmware_steering`, `vehicle_yaml`). Writing them
there is a person's job, with the hardware lead: the steering actuator's own (Jrk) limits
are the last line of defence, and theirs to set.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence

COUNTS_MAX = 4095
MIN_SPAN = 100        # counts: less between the ends, and the feedback doesn't change
MARGIN = 0.05         # off the smaller side's travel
CENTRE_OFF = 0.10     # of the span: a centre further from the middle is flagged
LOPSIDED = 0.8        # smaller over larger travel, below which it is flagged
GAINS_DIFFER = 0.15   # of their mean


@dataclass
class Limits:
    """What the measurements give."""

    centre: int
    lo: int                                   # counts
    hi: int
    half: int                                 # counts each side of the centre
    sign: int                                 # +1: steering left raises the target
    counts_per_rad: Optional[float] = None    # signed, as calibration.rs's counts_per_rad
    max_steering_angle: Optional[float] = None  # rad at lo or hi
    warnings: List[str] = field(default_factory=list)


class MeasurementError(ValueError):
    """The measurements can't give limits."""


def compute(left: int, centre: int, right: int, left_deg: Optional[float] = None,
            right_deg: Optional[float] = None, margin: float = MARGIN) -> Limits:
    """Return the limits for the marked ends and centre (counts); raise MeasurementError."""
    for name, value in (('left', left), ('centre', centre), ('right', right)):
        if not 0 <= value <= COUNTS_MAX:
            raise MeasurementError(f'{name} {value}: counts are 0 to {COUNTS_MAX}')
    if abs(left - right) < MIN_SPAN:
        raise MeasurementError(
            f'the ends are {abs(left - right)} counts apart (less than {MIN_SPAN}): the '
            "feedback doesn't change. Is the Jrk's feedback mode analog, and its sensor wired?")
    if (left - centre) * (right - centre) >= 0:
        raise MeasurementError(
            f'the centre ({centre}) is not between the ends ({left}, {right}): mark it again')
    travel_left, travel_right = abs(left - centre), abs(right - centre)
    half = math.floor(min(travel_left, travel_right) * (1.0 - margin))
    sign = 1 if left > centre else -1
    result = Limits(centre=centre, lo=centre - half, hi=centre + half, half=half, sign=sign)

    span = abs(left - right)
    middle = (left + right) / 2
    if abs(centre - middle) > CENTRE_OFF * span:
        result.warnings.append(
            f'the centre ({centre}) is {abs(centre - middle):.0f} counts from the middle of '
            f'the travel ({middle:.0f}): check the linkage, and mark the centre again')
    if min(travel_left, travel_right) / max(travel_left, travel_right) < LOPSIDED:
        result.warnings.append(
            f'the travel is lopsided: {travel_left} counts left, {travel_right} right; the '
            f'limits use the smaller side')
    for name, value in (('left', left), ('right', right)):
        if value in (0, COUNTS_MAX):
            result.warnings.append(
                f'the {name} end reads {value}: the feedback may be clipped (by the Jrk '
                'feedback scaling); the true end may be further')

    if left_deg is not None and right_deg is not None:
        if left_deg <= 0 or right_deg <= 0:
            raise MeasurementError('the wheel angles are positive degrees, each side')
        gain_left = travel_left / math.radians(left_deg)
        gain_right = travel_right / math.radians(right_deg)
        mean = (gain_left + gain_right) / 2
        result.counts_per_rad = sign * mean
        result.max_steering_angle = half / mean
        if abs(gain_left - gain_right) > GAINS_DIFFER * mean:
            result.warnings.append(
                f'the gains of the two sides differ: {gain_left:.0f} and {gain_right:.0f} '
                'counts a radian; measure the angles again, or the linkage is not linear')
    return result


def firmware_steering(r: Limits) -> str:
    """Return spine-board calibration.rs's Steering for the result (a person pastes it)."""
    cpr = ('TODO: measure the wheel angles' if r.counts_per_rad is None
           else f'{r.counts_per_rad:.1f}')
    return (f'steering: Steering {{\n    centre: {r.centre},\n    counts_per_rad: {cpr},\n'
            f'    min: {r.lo},\n    max: {r.hi},\n}},')


def vehicle_yaml(r: Limits) -> str:
    """Return vehicle.yaml's max_steering_angle line for the result."""
    if r.max_steering_angle is None:
        return '# max_steering_angle: measure the wheel angles at each end first'
    return f'max_steering_angle: {r.max_steering_angle:.3f}  # rad, measured (steering_limits)'


def report(r: Limits) -> str:
    """Return the result for people: short lines."""
    lines = [f'centre {r.centre}, limits {r.lo} to {r.hi} ({r.half} counts each side)',
             f'steering left {"raises" if r.sign > 0 else "lowers"} the target']
    if r.max_steering_angle is not None:
        degrees = math.degrees(r.max_steering_angle)
        lines.append(f'{abs(r.counts_per_rad):.0f} counts a radian; at the limits '
                     f'{r.max_steering_angle:.3f} rad ({degrees:.1f} deg)')
    lines += [f'WARNING: {w}' for w in r.warnings]
    lines += ['', 'For firmware/spine/crates/spine-board/src/calibration.rs:',
              firmware_steering(r), '', 'For vehicle.yaml:', vehicle_yaml(r), '',
              "The steering actuator's own (Jrk) limits are the hardware lead's to set."]
    return '\n'.join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(
        prog='steering_limits',
        description="The steering actuator's limits from its marked ends and centre (counts).")
    p.add_argument('--left', type=int, required=True, help='counts at full left lock')
    p.add_argument('--centre', type=int, required=True, help='counts with the wheels straight')
    p.add_argument('--right', type=int, required=True, help='counts at full right lock')
    p.add_argument('--left-deg', type=float, help='wheel angle at full left, degrees')
    p.add_argument('--right-deg', type=float, help='wheel angle at full right, degrees')
    p.add_argument('--margin', type=float, default=MARGIN,
                   help='off the smaller side (default 0.05)')
    a = p.parse_args(argv)
    try:
        r = compute(a.left, a.centre, a.right, a.left_deg, a.right_deg, a.margin)
    except MeasurementError as e:
        print(f'ERROR: {e}')
        return 1
    print(report(r))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

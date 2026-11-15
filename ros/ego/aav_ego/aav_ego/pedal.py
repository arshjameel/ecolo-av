"""
The pedal: a person's stick read as an accelerator and a brake, not a speed (Ben, 15 Nov).

On the test drive the driver had to switch between speeding up, braking and reversing.
With the pedal, one stick does all three:

    | Stick, moving forward      | ego                                                  |
    |----------------------------|------------------------------------------------------|
    | ahead of the speed now     | speeds up to it, as before (max_acceleration, jerk)  |
    | let off part way           | slows gently: the more let off, the harder           |
    | let go (0)                 | slows at let_off (1.0 m/s^2), like engine braking    |
    | pulled back                | brakes: a little softly, all the way at full brake   |
    | still back at a standstill | reverses, speeding up as above                       |

Backwards, the same with forward and back swapped. The stick's request stays a speed
(TwistStamped): how far it is below the speed now, or against it, sets how hard ego slows.

    let off to a fraction f of the speed now:  let_off x (1 - f)
    pulled back by b (0..1 of a full stick):   let_off + b x (max_deceleration - let_off)

Never weaker than asked by anything else: a stale command, the e-stop, or a turn the body
can't take at this speed brake at max_deceleration, as without the pedal (ego_node and
ackermann.EgoModel decide when the pedal applies). Only people use it by default (joy, rc,
keyboard): a computer driver plans its own slowing (the stop law, speed profiles) and needs
ego to follow its speeds at once. No ROS here.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Pedal:
    """The pedal's two numbers."""

    let_off: float = 1.0    # m/s^2: slowing with the stick let go (Ben, 15 Nov)
    full: float = 1.0       # m/s: the speed a full stick asks for (teleop_twist_joy's scale)


def slow_rate(pedal: Pedal, current: float, target: float, max_deceleration: float) -> float:
    """
    Return how hard (m/s^2) to slow from `current` toward `target`, both signed m/s.

    Called only while slowing: |target| below |current|, or target the other way. Against
    the motion, a target's size over `full` is how far the stick is pulled back.
    """
    let_off = min(pedal.let_off, max_deceleration)
    if current == 0.0:
        return max_deceleration
    if target != 0.0 and (target > 0.0) != (current > 0.0):
        back = min(1.0, abs(target) / pedal.full) if pedal.full > 0.0 else 1.0
        return let_off + back * (max_deceleration - let_off)
    kept = min(1.0, abs(target) / abs(current))
    return let_off * (1.0 - kept)

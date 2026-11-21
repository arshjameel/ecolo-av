"""tools/sim_moves.py: the 3 m drive's hold works out (no ROS needed)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import sim_moves  # noqa: E402


def test_the_whole_drive_is_the_metres_asked():
    speed, accel, decel = 0.5, 0.3, 0.3
    hold = sim_moves.hold_for(3.0, speed, accel, decel)
    assert abs(hold + sim_moves.ramps(speed, accel, decel) - 3.0) < 1e-9
    assert sim_moves.hold_for(0.1, speed, accel, decel) == 0.0   # too short: no hold


def test_the_pedal_script_ends_let_go_and_reverses_on_a_held_stick():
    sticks = [stick for _, stick, _ in sim_moves.PEDAL]
    assert sticks[0] == 0.0 and sticks[-1] == 0.0
    assert any(s > 0 for s in sticks) and sticks.count(-1.0) >= 2

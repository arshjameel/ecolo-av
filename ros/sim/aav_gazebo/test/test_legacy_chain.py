"""
Today's car's whole speed path, without ROS: what sim.launch.py esp32:=true runs.

  ego's speed ─> SpeedControl ─> from_pedals ─> 13-byte frame ─> Esp32Firmware ─> Actuators

The bridge's loop (aav_legacy) through the C++ firmware's ramp and brake (esp32_firmware)
into Gazebo's actuator model, at 50 Hz. Guarded, not skipped at module level: aav_legacy
is another package (CLAUDE.md, Testing).
"""

import importlib.util

from aav_gazebo.actuator_model import ActuatorConfig, Actuators, Demand
from aav_gazebo.esp32_firmware import Esp32Firmware
import pytest

HAS_LEGACY = importlib.util.find_spec('aav_legacy') is not None
pytestmark = pytest.mark.skipif(not HAS_LEGACY, reason='needs aav_legacy (colcon test)')

DT = 0.02
PLAN = 1.5      # m/s^2: drive_to_point's planned stop (half max_brake)


def profile(top, accel=1.0, decel=PLAN, cruise=4.0):
    """Return ego's speed each tick: up, hold, down at the plan, then standing."""
    v, out = 0.0, []
    while v < top:
        v = min(top, v + accel * DT)
        out.append(v)
    out += [top] * int(cruise / DT)
    while v > 0.0:
        v = max(0.0, v - decel * DT)
        out.append(v)
    return out + [0.0] * int(3.0 / DT)


def drive(top, mode):
    """Return (the car's speeds, its distance, ego's planned distance)."""
    from aav_legacy.legacy_protocol import from_pedals, LegacyMapping, pack_frame, to_legacy
    from aav_legacy.speed_control import SpeedControl
    loop, firmware, car, m = SpeedControl(), Esp32Firmware(), Actuators(ActuatorConfig()), \
        LegacyMapping()
    speeds, distance, planned = [], 0.0, 0.0
    for target in profile(top):
        if mode == 'pi':
            p = loop.update(target, car.speed, DT)
            values = from_pedals(0.0, p.throttle, p.brake, m)
        else:
            values = to_legacy(target, 0.0, m)
        firmware.feed(pack_frame(*values))
        o = firmware.outputs()
        state = car.drive(Demand(o.throttle, o.brake, o.reverse), 0.0, DT)
        speeds.append(state.speed)
        distance += state.speed * DT
        planned += target * DT
    return speeds, distance, planned


def hardest(speeds, window=0.5):
    """Return the hardest slowing over any `window` s (m/s^2), as sim-check measures it."""
    n = int(window / DT)
    return max((speeds[i] - speeds[i + n]) / window for i in range(len(speeds) - n))


@pytest.mark.parametrize('top', [1.0, 3.0])
def test_the_loop_stops_at_its_planned_deceleration(top):
    speeds, _, _ = drive(top, 'pi')
    assert speeds[-1] == 0.0
    assert hardest(speeds) <= 1.1 * PLAN


@pytest.mark.parametrize('top', [1.0, 3.0])
def test_the_loop_ends_near_egos_plan_and_open_loop_does_not(top):
    _, pi, planned = drive(top, 'pi')
    _, open_loop, _ = drive(top, 'open')
    assert abs(pi - planned) < abs(open_loop - planned)

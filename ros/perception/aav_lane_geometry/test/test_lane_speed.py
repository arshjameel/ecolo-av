"""The lane follower's slow-down rule (scripts/lane_speed.py): no ROS needed."""

from pathlib import Path
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / 'scripts'))

from lane_speed import AI_SLOT_TIMEOUT_S, slow_down, stopping_speed  # noqa: E402, I100


def test_slows_at_decel_rate():
    assert slow_down(2.0, 0.5, 0.1) == pytest.approx(1.95)


def test_never_below_zero():
    assert slow_down(0.02, 0.5, 0.1) == 0.0
    assert slow_down(0.0, 0.5, 0.1) == 0.0


def test_no_decel_rate_stops_at_once():
    assert slow_down(2.0, 0.0, 0.1) == 0.0
    assert slow_down(2.0, -1.0, 0.1) == 0.0


def test_clock_going_back_changes_nothing():
    assert slow_down(1.5, 0.5, -0.3) == 1.5


def test_a_whole_stop_takes_speed_over_decel_rate():
    """From 2 m/s at 0.5 m/s^2, ticking every 0.1 s: 4 s, getting slower every tick."""
    speeds = [2.0]
    while speeds[-1] > 0.0:
        speeds.append(stopping_speed(speeds[-1], 0.5, 0.1, quiet_s=0.1))
    assert len(speeds) - 1 == pytest.approx(40, abs=1)
    assert all(b < a for a, b in zip(speeds, speeds[1:]))


def test_no_slow_down_once_twist_mux_has_dropped_the_slot():
    """Ego has stopped the car by then: starting from the old speed would pull away."""
    assert stopping_speed(2.0, 0.5, 0.1, quiet_s=AI_SLOT_TIMEOUT_S) == 0.0
    assert stopping_speed(2.0, 0.5, 0.1, quiet_s=3.0) == 0.0
    assert stopping_speed(2.0, 0.5, 0.1, quiet_s=0.4) == pytest.approx(1.95)


def test_timeout_matches_twist_mux():
    config = REPO / 'ros/command_mux/aav_command_mux/config/twist_mux.yaml'
    params = yaml.safe_load(config.read_text())['twist_mux']['ros__parameters']
    assert params['topics']['ai']['topic'] == '/drivers/ai/cmd_vel'
    assert params['topics']['ai']['timeout'] == AI_SLOT_TIMEOUT_S


def test_stop_wait_default_starts_the_slow_down_in_time():
    """The watchdog ticks every 0.1 s, so it must notice before twist_mux's timeout."""
    from lane_follow import Settings
    assert Settings().stop_wait_s + 0.1 < AI_SLOT_TIMEOUT_S

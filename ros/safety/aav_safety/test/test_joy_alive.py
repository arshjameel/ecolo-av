"""JoyAlive: a joystick that goes silent or freezes while driving stops, and stays stopped."""

from aav_safety.joy_alive import FROZEN, JoyAlive, OK, SILENT, WAITING
import pytest

RELEASED = [0, 0, 0]
HELD = [1, 0, 0]
DT = 0.05                       # joy_node's 20 Hz


def started(**settings):
    """Return a check that has seen the deadman released once, so it may drive."""
    check = JoyAlive(**settings)
    assert check.on_joy(0.0, [0.0, 0.0], RELEASED)
    return check


def hand(i):
    """Return a real hand's throttle: a little jitter round 0.5."""
    return [0.0, 0.5 + 0.004 * ((i * 7) % 5 - 2)]


def drive(check, t0, seconds, axes=hand):
    """Feed held messages from t0 for `seconds`; return the time after and the last answer."""
    ok, n = True, int(round(seconds / DT))
    for i in range(n):
        ok = check.on_joy(t0 + i * DT, axes(i), HELD)
    return t0 + n * DT, ok


def test_nothing_drives_before_the_deadman_is_seen_released():
    check = JoyAlive()
    assert check.state(0.0).startswith(WAITING)
    # A trigger already held at start-up (a pad left on the seat) doesn't drive.
    assert not check.on_joy(0.0, [0.0, 0.5], HELD)
    assert check.state(0.0).startswith(WAITING)
    assert check.on_joy(0.1, [0.0, 0.0], RELEASED)
    assert check.on_joy(0.2, [0.0, 0.5], HELD)
    assert check.state(0.2) == f'{OK} - driving'


def test_a_real_hand_drives_on():
    check = started()
    t, ok = drive(check, 1.0, 20.0)
    assert ok and check.tripped is None


def test_frozen_values_while_driving_stop_after_frozen_after():
    check = started(frozen_after=2.0)
    t, ok = drive(check, 1.0, 3.0)
    assert ok
    frozen = [0.0, 0.5]
    assert check.on_joy(t, frozen, HELD)
    assert check.on_joy(t + 1.95, frozen, HELD)
    assert not check.on_joy(t + 2.0, frozen, HELD)
    assert check.state(t + 2.0).startswith(FROZEN)


def test_frozen_latches_until_the_deadman_is_released():
    check = started()
    t, _ = drive(check, 1.0, 1.0, axes=lambda i: [0.0, 1.0])     # pinned, or a dead pad
    t, ok = drive(check, t, 2.0, axes=lambda i: [0.0, 1.0])
    assert not ok
    # The pad comes back with the trigger still held, the stick moving: still stopped.
    t, ok = drive(check, t, 3.0)
    assert not ok and check.tripped == FROZEN
    assert check.on_joy(t, [0.0, 0.0], RELEASED)
    assert check.state(t) == f'{OK} - idle'
    assert check.on_joy(t + DT, [0.0, 0.5], HELD)


def test_still_values_without_motion_never_freeze():
    check = started()
    # The deadman held, the throttle centred: the car is asked to stand, so no FROZEN.
    t, ok = drive(check, 1.0, 10.0, axes=lambda i: [0.3, 0.0])
    assert ok
    # Nothing held, the stick left forward: teleop_twist_joy sends nothing anyway.
    for i in range(200):
        assert check.on_joy(t + i * DT, [0.0, 0.5], RELEASED)


def test_only_the_motion_axes_count_as_motion():
    check = started(motion_axes=(1,))
    # Steering held still at full lock, no throttle: the car can't move, so no FROZEN.
    t, ok = drive(check, 1.0, 5.0, axes=lambda i: [1.0, 0.0])
    assert ok
    # But a change in any axis counts as a live hand.
    t, ok = drive(check, t, 5.0, axes=lambda i: [0.01 * (i % 2), 0.5])
    assert ok


def test_frozen_after_zero_turns_the_rule_off():
    check = started(frozen_after=0.0)
    t, ok = drive(check, 1.0, 30.0, axes=lambda i: [0.0, 1.0])
    assert ok


def test_silence_trips_and_latches():
    check = started(silent_after=0.5)
    t, _ = drive(check, 1.0, 1.0)
    assert not check.tick(t + 0.4)
    assert check.tick(t + 0.5)
    assert not check.tick(t + 1.0)          # it says so once
    assert check.state(t + 1.0).startswith(SILENT)
    # Plugged in again with the trigger held: still stopped.
    assert not check.on_joy(t + 2.0, [0.0, 0.5], HELD)
    assert 'back with the deadman held' in check.state(t + 2.0)
    assert check.on_joy(t + 2.1, [0.0, 0.0], RELEASED)
    assert check.on_joy(t + 2.2, [0.0, 0.5], HELD)


def test_silence_while_idle_also_trips():
    check = started()
    assert check.tick(0.5)
    assert check.tripped == SILENT
    assert check.on_joy(1.0, [0.0, 0.0], RELEASED)


def test_no_joystick_at_all_is_waiting_not_silent():
    check = JoyAlive()
    assert not check.tick(100.0)
    assert check.state(100.0) == f'{WAITING} - no joystick yet'


def test_a_short_message_is_not_held():
    check = started(enable_button=5)
    assert not check.held([1, 1])
    assert not check.driving([0.0, 1.0], [1, 1])


@pytest.mark.parametrize('bad', [{'silent_after': 0.0}, {'silent_after': -1.0},
                                 {'frozen_after': float('nan')},
                                 {'frozen_after': float('inf')}, {'enable_button': -1}])
def test_bad_settings_are_refused(bad):
    with pytest.raises(ValueError):
        JoyAlive(**bad)

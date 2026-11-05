"""legacy_uart_bridge's logic (uart_relay): it relays the Orin's counter, never makes one."""

from aav_legacy.legacy_protocol import (CAL_MODE_JOG, CAL_MODE_LIMP, CAL_MODE_OFF, LegacyMapping,
                                        unpack_frame, unpack_frame_v2, unpack_frame_v3)
from aav_legacy.uart_relay import programs_locked, Relay
import pytest

M = LegacyMapping(max_steering_angle=0.5, full_throttle_speed=3.0)


def relay(version=2):
    return Relay(M, cmd_timeout=0.3, version=version)


def test_a_fresh_command_is_armed_and_carries_the_orins_counter():
    r = relay()
    r.heartbeat(41)
    r.command(1.5, 0.25, now=10.0)
    f = unpack_frame_v2(r.frame(now=10.02))
    assert (f.counter, f.armed, f.estop) == (41, True, False)
    assert (f.steering, f.speed, f.accel) == pytest.approx((0.5, 0.5, 0.0))


def test_the_counter_moves_only_when_the_orin_sends_one():
    r = relay()
    r.command(1.0, 0.0, now=0.0)
    counters = []
    for i in range(10):            # 10 ticks, 4 heartbeats: the relay adds none
        if i in (2, 3, 7, 8):
            r.heartbeat(i)
        counters.append(unpack_frame_v2(r.frame(now=0.01 * i)).counter)
    assert counters == [0, 0, 2, 3, 3, 3, 3, 7, 8, 8]


def test_when_ego_goes_quiet_it_sends_stops_with_the_last_counter():
    r = relay()
    r.heartbeat(200)
    r.command(1.0, 0.2, now=0.0)
    r.frame(now=0.1)
    for t in (0.31, 0.5, 5.0):    # stale: older than 0.3 s
        f = unpack_frame_v2(r.frame(now=t))
        assert (f.counter, f.armed) == (200, False)
        assert (f.speed, f.accel) == (0.0, -1.0)        # the stale stop: brake full
        assert f.steering == pytest.approx(0.4)          # the steering held


def test_the_estop_flag_and_brake_win_over_a_fresh_command():
    r = relay()
    r.heartbeat(5)
    r.command(1.0, 0.0, now=0.0)
    assert r.set_estop(True) and not r.set_estop(True)
    f = unpack_frame_v2(r.frame(now=0.01))
    assert (f.armed, f.estop, f.speed, f.accel) == (False, True, 0.0, -1.0)
    r.set_estop(False)
    assert unpack_frame_v2(r.frame(now=0.02)).armed


def test_before_any_heartbeat_the_counter_is_zero_and_never_moves():
    r = relay()
    r.command(1.0, 0.0, now=0.0)
    assert {unpack_frame_v2(r.frame(now=0.01 * i)).counter for i in range(20)} == {0}
    assert not r.heard_counter


def test_version_1_is_the_cpp_frame():
    r = relay(version=1)
    r.heartbeat(9)
    r.command(1.5, 0.25, now=0.0)
    frame = r.frame(now=0.0)
    assert len(frame) == 13
    assert unpack_frame(frame) == pytest.approx((0.5, 0.5, 0.0))
    assert unpack_frame(r.stop_frame()) == pytest.approx((0.5, 0.0, -1.0))


def test_an_unknown_version_is_refused():
    with pytest.raises(ValueError):
        relay(version=4)


def test_programs_are_locked_out_of_the_firmware_with_no_timeout():
    # Version 1: the C++ firmware, no command timeout. Only people drive.
    assert programs_locked(1) is True
    # Version 2: the Rust firmware brakes 100 ms after the Orin's counter stops.
    assert programs_locked(2) is False
    # The deliberate override, for a supervised stand.
    assert programs_locked(1, allow_programs_without_timeout=True) is False
    assert programs_locked(2, allow_programs_without_timeout=True) is False
    assert programs_locked(3) is False    # version 3: the same firmware, the same watchdog
    with pytest.raises(ValueError):
        programs_locked(4)


def looped():
    from aav_legacy.speed_control import SpeedControl
    return Relay(M, cmd_timeout=0.3, version=2, speed_loop=SpeedControl(), odom_timeout=0.3)


def test_with_a_speed_loop_too_fast_brakes_while_moving():
    r = looped()
    r.command(1.0, 0.0, now=0.0)
    r.wheel_speed(2.0, now=0.0)                # 1 m/s too fast
    f = unpack_frame_v2(r.frame(now=0.02))
    assert f.speed == 0.0 and f.accel < -400 / 3500
    assert r.pedals.brake > 0.0


def test_with_a_speed_loop_too_slow_opens_the_throttle_more_than_open_loop():
    r = looped()
    for i in range(50):                        # 1 s asking 1 m/s of a car stuck at 0.5
        r.command(1.0, 0.0, now=0.02 * i)
        r.wheel_speed(0.5, now=0.02 * i)
        f = unpack_frame_v2(r.frame(now=0.02 * i))
    assert f.accel == 0.0
    assert f.speed > 1.0 / M.full_throttle_speed   # open loop would send 1 / 3


def test_with_a_stale_wheel_speed_the_loop_runs_on_feed_forward_alone():
    r = looped()
    r.wheel_speed(0.0, now=0.0)
    for i in range(50):
        r.command(1.0, 0.0, now=1.0 + 0.02 * i)
        r.frame(now=1.0 + 0.02 * i)            # the wheel speed is 1 s old
    assert r.pedals.integral == 0.0
    assert r.pedals.throttle == pytest.approx(0.1 * 1.0 / 1.5)   # drag x speed / max_accel


def test_disarming_resets_the_speed_loop():
    r = looped()
    r.command(1.0, 0.0, now=0.0)
    r.wheel_speed(0.9, now=0.0)
    r.frame(now=0.02)
    unpack_frame_v2(r.frame(now=1.0))          # the command went stale: brake, reset
    assert (r.loop.integral, r.loop.last_target, r.pedals) == (0.0, None, None)


# --- Version 3: the steering actuator's calibration (1 Nov) ---

def calibrating(where='stands', now=10.0):
    """Return a version 3 relay, the uplink fresh, the panel asking for limp at `now`."""
    r = Relay(M, cmd_timeout=0.3, version=3, calibration=where)
    r.heartbeat(9)
    r.uplink_seen(now)
    r.calibrate(CAL_MODE_LIMP, 0, now)
    return r


def test_calibration_frames_stand_the_car_still():
    r = calibrating()
    r.command(0.0, 0.3, now=10.0)        # ego stands still, wheels turned
    r.frame(now=10.0)
    f = unpack_frame_v3(r.frame(now=10.02))
    assert (f.cal_mode, f.speed, f.accel, f.armed, f.counter) == (CAL_MODE_LIMP, 0.0, -1.0,
                                                                  False, 9)
    r.calibrate(CAL_MODE_JOG, 2300, now=10.05)
    f = unpack_frame_v3(r.frame(now=10.06))
    assert (f.cal_mode, f.jog_target, f.speed) == (CAL_MODE_JOG, 2300, 0.0)


def test_calibration_is_off_unless_the_hardware_lead_set_where():
    r = calibrating(where='none')
    assert unpack_frame_v3(r.frame(now=10.0)).cal_mode == CAL_MODE_OFF
    assert 'hardware lead' in r.cal_refusal(10.0)
    with pytest.raises(ValueError):
        Relay(M, cmd_timeout=0.3, version=3, calibration='ground')


@pytest.mark.parametrize('version', [1, 2])
def test_calibration_needs_version_3(version):
    r = Relay(M, cmd_timeout=0.3, version=version, calibration='stands')
    r.uplink_seen(0.0)
    r.calibrate(CAL_MODE_LIMP, 0, 0.0)
    assert not r.cal_active(0.0)
    assert 'needs 3' in r.cal_refusal(0.0)


def test_calibration_refused_while_driving_moving_estop_or_without_uplink():
    r = calibrating()
    r.command(1.0, 0.0, now=10.0)
    assert 'driving' in r.cal_refusal(10.0)
    f = unpack_frame_v3(r.frame(now=10.0))
    assert f.cal_mode == CAL_MODE_OFF and f.speed > 0.0   # ego drives on, untouched
    r = calibrating()
    r.wheel_speed(0.2, now=10.0)
    assert 'moves' in r.cal_refusal(10.0)
    r = calibrating()
    r.set_estop(True)
    assert 'e-stop' in r.cal_refusal(10.0)
    r = calibrating()
    assert 'uplink' in r.cal_refusal(10.5)               # the last uplink 0.5 s ago


def test_calibration_ends_when_the_panel_stops_asking():
    r = calibrating()
    r.uplink_seen(10.4)
    assert unpack_frame_v3(r.frame(now=10.4)).cal_mode == CAL_MODE_LIMP
    r.uplink_seen(10.6)
    f = unpack_frame_v3(r.frame(now=10.6))                # 0.6 s since the panel asked
    assert f.cal_mode == CAL_MODE_OFF and (f.speed, f.accel) == (0.0, -1.0)
    r.calibrate(CAL_MODE_OFF, 0, now=10.7)
    assert not r.cal_wanted(10.7)

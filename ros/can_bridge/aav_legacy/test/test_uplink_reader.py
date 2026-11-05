"""The bridge's side of the uplink (TODO 50, 48): frames in, ROS fields out. No ROS."""

import math

from aav_legacy.legacy_protocol import (MODE_COMPUTER, MODE_STOPPED, pack_uplink,
                                        SWITCH_RC_VIA_ROS, Uplink)
from aav_legacy.uplink_reader import (brake_state, rc_fields, status_fields, steering_state,
                                      UplinkReader)


def up(**changes):
    fields = {'sequence': 0, 'echo': 5, 'mode': MODE_COMPUTER, 'switch': SWITCH_RC_VIA_ROS,
              'armed': True, 'estop': False, 'faults': 0x0100, 'steering': 0.125,
              'brake_position': None, 'throttle_out': 0.25, 'brake_out': 0.0,
              'rc_throttle': 0.5, 'rc_steering': -0.75, 'rc_brake': False}
    fields.update(changes)
    return Uplink(**fields)


def test_frames_in_pieces_and_freshness():
    r = UplinkReader(timeout=0.2)
    assert not r.fresh(0.0)
    data = pack_uplink(up(sequence=1)) + pack_uplink(up(sequence=2))
    assert r.feed(data[:20], 1.0) == []
    found = r.feed(data[20:], 1.01)
    assert [u.sequence for u in found] == [1, 2]
    assert r.latest.sequence == 2 and r.fresh(1.2) and not r.fresh(1.25)
    assert (r.frames, r.lost) == (2, 0)


def test_lost_frames_and_bad_frames_are_counted():
    r = UplinkReader()
    bad = bytearray(pack_uplink(up(sequence=6)))
    bad[12] ^= 0x01
    r.feed(pack_uplink(up(sequence=254)) + pack_uplink(up(sequence=255))
           + pack_uplink(up(sequence=2)) + bytes(bad) + pack_uplink(up(sequence=7)), 0.0)
    assert r.lost == 2 + 4          # 0 and 1 missing; 3..6 missing (6 was bad)
    assert r.bad == 1


def test_the_status_is_vehicle_status():
    s = status_fields(up(mode=MODE_STOPPED, armed=False, estop=True, faults=1, steering=None))
    assert (s['mode'], s['armed'], s['estop_active'], s['faults']) == (0, False, True, 1)
    assert math.isnan(s['speed']) and math.isnan(s['steering_angle'])
    assert (s['throttle'], s['brake']) == (0.25, 0.0)


def test_steering_brake_and_sticks():
    u = up()
    assert steering_state(u, 0.1) == (0.1, 0.125)
    command, measured = steering_state(u, None)
    assert math.isnan(command) and measured == 0.125
    command, measured = brake_state(u)
    assert command == 0.0 and math.isnan(measured)
    assert brake_state(up(brake_position=0.5, brake_out=0.5)) == (0.5, 0.5)
    assert rc_fields(u) == {'mode_switch': SWITCH_RC_VIA_ROS, 'throttle': 0.5,
                            'steering': -0.75, 'brake': False}


def test_raw_fields_and_the_calibration_line():
    from aav_legacy.legacy_protocol import CAL_LIMP, CAL_REFUSED_NO_FEEDBACK
    from aav_legacy.uplink_reader import calibration_line, raw_fields
    assert raw_fields(2048, CAL_LIMP) == {'valid': True, 'position': 2048, 'calibration': 1}
    assert raw_fields(None) == {'valid': False, 'position': 0, 'calibration': 0}
    limp = up(steering_counts=3010, calibration=CAL_LIMP)
    assert calibration_line(True, None, limp).startswith('LIMP - ')
    assert '(at 3010 counts)' in calibration_line(True, None, limp)
    assert calibration_line(True, 'the e-stop is on', limp) == 'REFUSED - the e-stop is on'
    # A refusal the panel didn't ask about says nothing; the firmware's own state shows.
    assert calibration_line(False, 'the e-stop is on', None) == 'OFF'
    refused = up(calibration=CAL_REFUSED_NO_FEEDBACK)
    assert 'analog' in calibration_line(True, None, refused)


def test_a_version_3_uplink_carries_the_counts():
    r = UplinkReader()
    (u,) = r.feed(pack_uplink(up(steering_counts=1234, brake_counts=400)), 0.0)
    assert (u.steering_counts, u.brake_counts) == (1234, 400)
    (u,) = r.feed(pack_uplink(up(sequence=1), version=2), 0.0)
    assert (u.steering_counts, u.brake_counts, u.calibration) == (None, None, 0)

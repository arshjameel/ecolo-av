import math
from pathlib import Path

from aav_language import safety
from aav_language.safety import allowed_here, Caps, check_drive, check_turn, Where
import pytest
import yaml

EGO = {'max_speed': 1.0, 'max_reverse_speed': 0.5, 'max_acceleration': 1.0,
       'max_deceleration': 2.0, 'wheelbase': 1.57, 'max_steering_angle': 0.5}
MUX_YAML = (Path(__file__).resolve().parents[3] / 'command_mux' / 'aav_command_mux' / 'config'
            / 'twist_mux.yaml')


def test_default_caps_are_the_agreed_ones():
    c = Caps()
    assert (c.max_speed, c.max_distance, c.max_reverse_speed, c.max_reverse_distance) == (
        1.0, 200.0, 0.5, 10.0)


def test_ego_tightens_the_caps_never_loosens():
    c = safety.effective_caps(Caps(), {**EGO, 'max_speed': 0.3, 'max_reverse_speed': 2.0})
    assert c.max_speed == 0.3
    assert c.max_reverse_speed == 0.5            # the server's own cap stays
    assert c.min_radius == pytest.approx(1.57 / math.tan(0.5))
    assert safety.effective_caps(Caps(), {**EGO, 'max_speed': 5.0}).max_speed == 1.0
    assert safety.effective_caps(Caps(), None) == Caps()


@pytest.mark.parametrize('distance,speed,reverse,ok', [
    (100, 0.5, False, True), (200, 1.0, False, True), (201, 0.5, False, False),
    (5000, 0.5, False, False), (10, 1.01, False, False), (10, 5.0, False, False),
    (3, 0.5, True, True), (10, 0.5, True, True), (10.5, 0.5, True, False),
    (50, 0.5, True, False), (3, 0.6, True, False), (0, 0.5, False, False),
    (-5, 0.5, False, False), (5, 0, False, False), (math.nan, 0.5, False, False),
    (math.inf, 0.5, False, False)])
def test_drive_limits(distance, speed, reverse, ok):
    assert (check_drive(Caps(), distance, speed, reverse) is None) == ok


def test_turn_limits():
    c = safety.effective_caps(Caps(), EGO)
    assert check_turn(c, 90, 5, 0.5) is None
    assert check_turn(c, -45, 8, 0.5) is None
    assert check_turn(c, 360, 4, 0.5) is None
    assert 'tighter' in check_turn(c, 90, 1, 0.5)
    assert check_turn(c, 0, 5, 0.5) is not None
    assert check_turn(c, 720, 5, 0.5) is not None
    assert check_turn(c, 90, 5, 3.0) is not None
    assert 'arc' in check_turn(c, 360, 40, 0.5)          # 251 m of arc


def test_where_stand_ins_are_allowed():
    assert allowed_here(Where('LOCALHOST', '', False, ('ego',)), car=False) is None
    assert allowed_here(Where('localhost', '', False, ()), car=False) is None
    assert allowed_here(Where('', '', True, ('gazebo_adapter',)), car=False) is None
    bench = ('spine_bridge', 'spine_sim', 'ego')
    assert allowed_here(Where('LOCALHOST', '', False, bench), car=False) is None


def test_where_the_car_needs_the_flag():
    for w in (Where('', '', False, ()), Where('SUBNET', '', False, ()),
              Where('LOCALHOST', '10.0.0.5', False, ()),
              Where('LOCALHOST', '', False, ('legacy_uart_bridge',)),
              Where('LOCALHOST', '', True, ('spine_bridge',)),
              Where('LOCALHOST', '', False, ('encoder_udp', 'ego'))):
        assert allowed_here(w, car=False) is not None, w
        assert allowed_here(w, car=True) is None


def test_mux_config_reads_our_twist_mux_yaml():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    prio = {i.name: i.priority for i in inputs}
    # A person always outranks the scripted driver (and so this server).
    assert prio['joy'] > prio['scripted'] and prio['keyboard'] > prio['scripted']
    assert inputs[0].name == 'joy'
    assert {lk.name for lk in locks} == {'estop', 'no_timeout', 'stop_sign', 'sensors',
                                         'stop_zone', 'geofence'}


def test_the_stop_hold_holds_nav2_not_the_point_driver():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    held = {'sensors': (False, 0.1), 'geofence': (False, 0.1), 'stop_zone': (True, 0.1)}
    assert safety.mux_choice(inputs, {'nav': 0.1}, locks, held)['blocked_by'] == 'stop_zone'
    assert safety.mux_choice(inputs, {'point': 0.1}, locks, held)['driver'] == 'point'
    # Timeout 0: never heard (no collision_monitor in this profile) locks nothing.
    quiet = {'sensors': (False, 0.1), 'geofence': (False, 0.1)}
    assert safety.mux_choice(inputs, {'nav': 0.1}, locks, quiet)['driver'] == 'nav'


def test_mux_choice_follows_twist_mux_rules():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    # the sensor watchdog's and the geofence's heartbeats: all well
    no_locks = {'sensors': (False, 0.1), 'geofence': (False, 0.1)}
    assert safety.mux_choice(inputs, {}, locks, no_locks) == {'driver': None,
                                                              'blocked_by': None}
    ages = {'scripted': 0.1, 'nav': 0.1}
    assert safety.mux_choice(inputs, ages, locks, no_locks)['driver'] == 'scripted'
    ages['keyboard'] = 0.2
    assert safety.mux_choice(inputs, ages, locks, no_locks)['driver'] == 'keyboard'
    ages['keyboard'] = 0.6                           # timed out (0.5 s)
    assert safety.mux_choice(inputs, ages, locks, no_locks)['driver'] == 'scripted'
    stopped = safety.mux_choice(inputs, ages, locks, {'estop': (True, 30.0)})
    assert stopped == {'driver': None, 'blocked_by': 'estop'}
    # the stop-sign lock (30) holds Nav2 (20), not the scripted driver (50)
    sign = {**no_locks, 'stop_sign': (True, 1.0)}
    assert safety.mux_choice(inputs, {'nav': 0.1}, locks, sign)['blocked_by'] == 'stop_sign'
    assert safety.mux_choice(inputs, {'scripted': 0.1}, locks, sign)['driver'] == 'scripted'


def test_a_lock_with_a_timeout_locks_on_silence_as_twist_mux_does():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    sensors = next(lk for lk in locks if lk.name == 'sensors')
    # Fail-safe: the sensor watchdog's heartbeat must keep arriving, or Nav2 is held.
    assert 0.0 < sensors.timeout <= 2.0 and sensors.priority == 30
    heard = {'sensors': (False, 0.1), 'geofence': (False, 0.1)}
    assert safety.mux_choice(inputs, {'nav': 0.1}, locks, heard)['driver'] == 'nav'
    quiet = {**heard, 'sensors': (False, sensors.timeout + 0.5)}
    assert safety.mux_choice(inputs, {'nav': 0.1}, locks, quiet)['blocked_by'] == 'sensors'
    never = {'geofence': (False, 0.1)}
    assert safety.mux_choice(inputs, {'nav': 0.1}, locks, never)['blocked_by'] == 'sensors'
    # A person, a test, still drive.
    assert safety.mux_choice(inputs, {'joy': 0.1}, locks, quiet)['driver'] == 'joy'
    assert safety.mux_choice(inputs, {'scripted': 0.1}, locks, never)['driver'] == 'scripted'


def test_the_geofence_holds_every_program_but_not_people_or_the_tests():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    lock = next(lk for lk in locks if lk.name == 'geofence')
    # Fail-safe, as the watchdog's: a dead geofence holds them too.
    assert 0.0 < lock.timeout <= 2.0 and lock.priority == 51
    heard = {'sensors': (False, 0.1), 'geofence': (False, 0.1)}
    for on in ({**heard, 'geofence': (True, 0.1)}, {'sensors': (False, 0.1)}):
        for driver in ('scripted', 'replay', 'point', 'nav', 'ai'):
            assert safety.mux_choice(inputs, {driver: 0.1}, locks, on)['blocked_by'] == 'geofence'
        for driver in ('joy', 'rc', 'keyboard', 'tests'):
            assert safety.mux_choice(inputs, {driver: 0.1}, locks, on)['driver'] == driver


def test_the_no_timeout_lock_holds_every_program_but_not_people_or_the_tests():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    lock = next(lk for lk in locks if lk.name == 'no_timeout')
    # Timeout 0: where no legacy bridge runs, its silence never locks.
    assert lock.timeout == 0.0 and lock.priority == 60
    heard = {'sensors': (False, 0.1), 'geofence': (False, 0.1)}
    for driver in ('scripted', 'replay', 'point', 'nav', 'ai'):
        assert safety.mux_choice(inputs, {driver: 0.1}, locks, heard)['driver'] == driver
    on = {**heard, 'no_timeout': (True, 0.1)}
    for driver in ('scripted', 'replay', 'point', 'nav', 'ai'):
        assert safety.mux_choice(inputs, {driver: 0.1}, locks, on)['blocked_by'] == 'no_timeout'
    for driver in ('joy', 'rc', 'keyboard', 'tests'):
        assert safety.mux_choice(inputs, {driver: 0.1, 'scripted': 0.1}, locks,
                                 on)['driver'] == driver


def test_holding_locks_by_the_driver_that_carries_the_move():
    inputs, locks = safety.mux_config(yaml.safe_load(MUX_YAML.read_text()))
    free = {'estop': False, 'stop_sign': False, 'sensors': False}
    assert safety.holding_locks('nav', inputs, locks, free) == ()
    sign = {**free, 'stop_sign': True}
    # The stop-sign lock (30) holds Nav2 (20), so go_to_gps; not the scripted driver (50).
    assert safety.holding_locks('nav', inputs, locks, sign) == ('stop_sign',)
    assert safety.holding_locks('scripted', inputs, locks, sign) == ()
    # The e-stop (255) holds everyone.
    assert safety.holding_locks('scripted', inputs, locks, {**free, 'estop': True}) == ('estop',)
    # An unknown driver counts as the lowest.
    assert set(safety.holding_locks('nobody', inputs, locks,
                                    {**sign, 'sensors': True})) == {'stop_sign', 'sensors'}


def test_where_a_move_ends_and_the_fence():
    square = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]
    assert safety.drive_end(None, 5.0) is None
    assert safety.drive_end((0.0, 0.0, 0.0), 5.0) == pytest.approx((5.0, 0.0))
    assert safety.drive_end((0.0, 0.0, 0.0), 5.0, reverse=True) == pytest.approx((-5.0, 0.0))
    assert safety.fence_refusal(square, (5.0, 0.0)) is None
    assert 'outside the geofence' in safety.fence_refusal(square, (15.0, 0.0))
    assert safety.fence_refusal(None, (15.0, 0.0)) is None
    assert safety.fence_refusal(square, None) is None
    # 0.001 degree of latitude north is about 111 m.
    x, y = safety.gps_end((1.0, 2.0, 0.0), (45.0, -75.0), 45.001, -75.0)
    assert (x, y) == pytest.approx((1.0, 2.0 + 111.2), abs=0.2)


def test_car_unreachable_when_discovery_stays_on_this_computer():
    # tools/ros.sh --local sets LOCALHOST: --car could then never reach the car.
    assert safety.car_unreachable(True, 'LOCALHOST', '') is not None
    assert 'AAV_DISCOVERY=SUBNET' in safety.car_unreachable(True, 'LOCALHOST', '')
    assert safety.car_unreachable(True, 'SUBNET', '') is None
    assert safety.car_unreachable(True, 'LOCALHOST', '192.168.1.140') is None
    assert safety.car_unreachable(False, 'LOCALHOST', '') is None

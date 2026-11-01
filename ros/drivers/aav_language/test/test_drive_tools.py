import json
import math

from aav_language import safety
from aav_language.drive_tools import DriveTools
from aav_language.fake_backend import FakeBackend
import pytest


def make(**kw):
    backend = FakeBackend(**kw)
    lines = []
    return DriveTools(backend, log=lines.append), backend, lines


def test_drive_sends_one_goal_covering_the_whole_distance():
    tools, car, _ = make()
    r = tools.drive(10.0)
    assert r['ok'], r
    goal = car.sent[0]
    assert goal['target_speed'] == 0.5
    # 0.25 m speeding up + 0.25 m slowing down at 0.5 m/s^2
    assert goal['hold_distance'] == pytest.approx(9.5)
    assert goal['curvature'] == 0.0
    assert r['distance_travelled_m'] == pytest.approx(10.0)
    assert r['odom_displacement_m'] == pytest.approx(10.0)


def test_reverse_and_its_limits():
    tools, car, _ = make()
    assert tools.drive(3.0, direction='reverse')['ok']
    assert car.sent[-1]['target_speed'] == -0.5
    assert car.pose[0] == pytest.approx(-3.0)
    r = tools.drive(50.0, direction='reverse')
    assert not r['ok'] and 'reverse is limited to 10 m' in r['message']
    assert len(car.sent) == 1
    assert not tools.drive(2.0, speed_mps=1.0, direction='reverse')['ok']
    assert not tools.drive(2.0, direction='sideways')['ok']


@pytest.mark.parametrize('distance,speed', [(5000, 0.5), (201, 0.5), (30, 5.0), (30, 1.5)])
def test_caps_refuse_and_nothing_is_sent(distance, speed):
    tools, car, _ = make()
    r = tools.drive(distance, speed)
    assert not r['ok'] and r['message'].startswith('refused')
    assert car.sent == []


def test_ego_max_speed_caps_below_the_server():
    tools, car, _ = make(ego={'max_speed': 0.3})
    assert not tools.drive(10, 0.5)['ok']
    assert tools.drive(10, 0.3)['ok']


def test_turn_is_an_arc_of_the_right_length():
    tools, car, _ = make()
    r = tools.turn(90, 5)
    assert r['ok'], r
    assert car.sent[0]['curvature'] == pytest.approx(0.2)
    assert car.sent[0]['distance'] == pytest.approx(5 * math.pi / 2)
    assert r['heading_change_deg'] == pytest.approx(90.0, abs=0.1)
    assert car.pose[0] == pytest.approx(5.0) and car.pose[1] == pytest.approx(5.0)
    r = tools.turn(-45, 8)
    assert car.sent[1]['curvature'] == pytest.approx(-0.125)
    assert r['heading_change_deg'] == pytest.approx(-45.0, abs=0.1)


def test_turn_direction_words_give_the_side():
    tools, car, _ = make()
    for angle, side, turned in ((45, 'right', -45), (-45, 'Right', -45), (-30, 'left', 30)):
        r = tools.turn(angle, 8, direction=side)
        assert r['heading_change_deg'] == pytest.approx(turned, abs=0.1)
    r = tools.turn(30, 8, direction='up')
    assert not r['ok'] and 'left' in r['message']
    assert len(car.sent) == 3


def test_turn_tighter_than_the_car_is_refused():
    tools, car, _ = make()
    r = tools.turn(90, 1.0)
    assert not r['ok'] and 'tighter' in r['message']
    assert car.sent == []


def test_estop_on_refuses_every_move_and_stop_still_works():
    tools, car, _ = make()
    assert tools.estop()['ok']
    assert car.estops == 1
    for r in (tools.drive(5), tools.turn(90, 5), tools.go_to_gps(45.3861, -75.6959)):
        assert not r['ok'] and 'e-stop is on' in r['message']
    assert tools.stop()['ok']
    assert car.sent == []


def test_an_estop_pressed_before_the_server_started_still_refuses():
    # Gazebo, 15 Oct: /safety/estop was sent before this server started, and nobody sends
    # it again. Only twist_mux's report (/diagnostics) still says the lock is on.
    tools, car, _ = make()
    plain = car.state

    def late():
        s = plain()
        s.estop = s.vehicle_estop = None
        s.mux_locks = {'estop': True, 'stop_sign': False}
        return s
    car.state = late
    r = tools.drive(5)
    assert not r['ok'] and 'e-stop is on' in r['message']
    assert tools.status()['estop'] is True
    assert car.sent == []


def test_no_word_from_twist_mux_refuses_to_move():
    tools, car, _ = make()
    car.mux_silent = True
    r = tools.drive(5)
    assert not r['ok'] and 'twist_mux' in r['message']
    assert tools.stop()['ok']


def test_mux_locks_from_twist_mux_diagnostics():
    # As twist_mux 4.5 sent it on the turtle, 15 Oct, after pixi run estop.
    values = {'velocity topics.joy': ' masked (listening to /drivers/joy/cmd_vel @ 0.5s)',
              'lock locks.estop': ' locked (listening to /safety/estop @ 0.000000s with '
                                  'priority #255)',
              'lock locks.stop_sign': ' free (listening to /perception/stop_sign @ 0.0s)',
              'current priority': '255'}
    got = safety.mux_locks_from_diagnostics([('ekf_local: odometry', {'x': '1'}),
                                             ('twist_mux: Twist mux status', values)])
    assert got == {'estop': True, 'stop_sign': False}
    assert safety.mux_locks_from_diagnostics([('ekf_local: odometry', {})]) is None


def test_tools_never_clear_the_estop():
    tools, car, _ = make()
    tools.estop()
    for name in ('status', 'stop', 'wait'):
        getattr(tools, name)(*([0] if name == 'wait' else []))
    assert car.estop_on


def test_not_on_the_car_without_the_flag():
    where = safety.Where('SUBNET', '', False, ('ego',))
    tools, car, _ = make(where=where)
    r = tools.drive(5)
    assert not r['ok'] and 'real car' in r['message']
    assert tools.status()['may_drive_here'] is False
    car_tools = DriveTools(car, car=True)
    assert car_tools.drive(5)['ok']


def test_go_to_gps():
    tools, car, _ = make()
    r = tools.go_to_gps(45.3861, -75.6959)
    assert r['ok'], r
    assert 70 < r['straight_line_m'] < 90
    assert car.sent[-1] == {'kind': 'gps', 'lat': 45.3861, 'lon': -75.6959}
    far = tools.go_to_gps(45.5, -75.6959)
    assert not far['ok'] and 'm away' in far['message']
    tools2, _, _ = make(nav=False)
    assert 'Nav2 is not running' in tools2.go_to_gps(45.3861, -75.6959)['message']


def test_wait_is_capped():
    tools, car, _ = make()
    assert tools.wait(3)['ok'] and car.slept == 3
    assert not tools.wait(600)['ok']
    assert not tools.wait(-1)['ok']


def test_status_has_what_a_person_asks():
    tools, _, _ = make()
    s = tools.status()
    for key in ('speed_mps', 'odom_pose', 'map_pose', 'gps', 'estop', 'armed', 'mux_driver',
                'drive_running', 'limits', 'may_drive_here'):
        assert key in s
    assert s['gps']['fix'] == 'GPS fix'
    assert s['limits']['min_turn_radius_m'] == pytest.approx(2.88, abs=0.01)


def test_check_moves_nothing():
    tools, car, _ = make()
    assert tools.check('drive', {'distance_m': 10}) is None
    assert 'limited' in tools.check('drive', {'distance_m': 5000})
    assert tools.check('turn', {'angle_deg': 90, 'radius_m': 5}) is None
    assert 'bad arguments' in tools.check('drive', {'distance': 10})
    assert car.sent == []


def test_every_call_is_logged_with_its_time(tmp_path):
    car = FakeBackend()
    lines = []
    path = tmp_path / 'calls.jsonl'
    tools = DriveTools(car, log=lines.append, log_path=path)
    tools.drive(5)
    tools.drive(5000)
    tools.stop()
    logged = [json.loads(x) for x in lines]
    assert [x['tool'] for x in logged] == ['drive', 'drive', 'stop']
    assert all(x['time'] for x in logged)
    assert [x['ok'] for x in logged] == [True, False, True]
    assert len(path.read_text().splitlines()) == 3


def test_a_stop_sign_lock_refuses_go_to_gps_but_not_drive():
    # twist_mux's stop-sign lock (30) holds Nav2 (20), which carries go_to_gps; the
    # scripted driver (50), which carries drive and turn, is above it.
    from pathlib import Path
    import yaml
    mux_yaml = (Path(__file__).resolve().parents[3]
                / 'command_mux/aav_command_mux/config/twist_mux.yaml')
    car = FakeBackend()
    tools = DriveTools(car, mux=safety.mux_config(yaml.safe_load(mux_yaml.read_text())))
    plain = car.state

    def sign_locked():
        s = plain()
        s.mux_locks = {**s.mux_locks, 'stop_sign': True}
        return s
    car.state = sign_locked
    r = tools.go_to_gps(45.3860, -75.6960)
    assert not r['ok'] and 'stop_sign' in r['message'], r
    assert tools.drive(5)['ok']


def test_a_move_that_would_end_outside_the_geofence_is_refused():
    car = FakeBackend()
    car.fence = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]
    tools = DriveTools(car)
    r = tools.drive(15)
    assert not r['ok'] and 'geofence' in r['message'], r
    assert tools.drive(5)['ok']
    # The GPS goal is about 80 m away: outside.
    assert 'geofence' in tools.check_go_to_gps(45.3860, -75.6960)
    car.fence = None              # no fence heard: the lock still holds near the edge
    assert tools.check_drive(15) is None


def test_the_drive_tools_never_clear_a_safety_latch():
    # Only people clear the geofence and the watchdog (Ben's rule: nothing in software
    # clears a stop on its own). The MCP drive tools are this package's drive_server.
    from pathlib import Path
    package = Path(__file__).resolve().parents[1] / 'aav_language'
    for path in package.glob('*.py'):
        text = path.read_text()
        for service in ('/safety/geofence/clear', '/safety/sensor_watchdog/clear',
                        'geofence/clear', 'geofence-clear'):
            if service in text:
                # Only in a docstring that says it never calls it.
                assert all('Never calls' in line or 'never' in line.lower()
                           for line in text.splitlines() if service in line), (path, service)

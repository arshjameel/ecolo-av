import math
import os

from aav_safety import watchdog
from aav_safety.watchdog import FROZEN, JUMP, LOST, NAN, NEVER, OK, Rule, STALE, Watchdog
import pytest
import yaml

ODOM = Rule('odom', '/vehicle/odom', 'odom', max_age=0.5)
IMU = Rule('imu', '/sensors/imu/data', 'imu', max_age=0.5, frozen_after=1.0)
GPS = Rule('gps', '/sensors/gps/fix', 'gps', max_age=2.0, max_jump_speed=15.0)
CAMERA = Rule('camera', '/sensors/camera/front_left/image_raw', 'image', max_age=1.0,
              frozen_after=1.0)
LIDAR = Rule('lidar', '/sensors/lidar/points', 'cloud', max_age=1.0, frozen_after=1.0)
RADAR = Rule('radar', '/sensors/radar/points', 'cloud', max_age=1.0, stop=False)
HUD = Rule('hud', '/hud/heartbeat', 'heartbeat', max_age=1.0, frozen_after=1.0)
LATCHED_HUD = Rule('hud', '/hud/heartbeat', 'heartbeat', max_age=1.0, frozen_after=1.0,
                   latch=True, latch_grace=5.0)
HERE = (45.3860, -75.6960)


def state(dog, name, now):
    return {n: s for n, s, _, _ in dog.states(now)}[name]


def test_nothing_yet_stops_the_drivers():
    dog = Watchdog([ODOM])
    assert state(dog, 'odom', 0.0) == NEVER
    assert dog.locked(0.0)


def test_fresh_is_ok_and_silence_goes_stale():
    dog = Watchdog([ODOM])
    dog.watches['odom'].arrived(10.0, (1.0, 0.0))
    assert state(dog, 'odom', 10.4) == OK and not dog.locked(10.4)
    assert state(dog, 'odom', 10.6) == STALE and dog.locked(10.6)
    dog.watches['odom'].arrived(10.7, (1.0, 0.0))     # back: drives again on its own
    assert not dog.locked(10.7)


def test_nan_stops_at_once():
    dog = Watchdog([ODOM])
    dog.watches['odom'].arrived(1.0, (math.nan, 0.0))
    assert state(dog, 'odom', 1.0) == NAN and dog.locked(1.0)


def test_a_gps_without_a_fix_counts_as_lost_after_max_age():
    dog = Watchdog([GPS])
    gps = dog.watches['gps']
    gps.arrived(0.0, fix=HERE)
    gps.arrived(1.0, fix=(math.nan, math.nan))
    assert state(dog, 'gps', 1.5) == OK              # a short gap is normal for a GPS
    gps.arrived(2.1, fix=HERE, has_fix=False)
    s, detail = gps.state(2.1)
    assert s == STALE and 'no fix' in detail


def test_a_gps_jump_latches_until_cleared():
    dog = Watchdog([GPS])
    gps = dog.watches['gps']
    gps.arrived(0.0, fix=HERE)
    gps.arrived(0.2, fix=(HERE[0] + 0.5e-5, HERE[1]))   # 0.56 m in 0.2 s: driving
    assert state(dog, 'gps', 0.2) == OK
    gps.arrived(0.4, fix=(HERE[0] + 20 / 111195.0, HERE[1]))   # 20 m in 0.2 s
    assert state(dog, 'gps', 0.4) == JUMP
    gps.arrived(0.6, fix=(HERE[0] + 20 / 111195.0, HERE[1]))
    assert state(dog, 'gps', 10.0) == JUMP            # the next fixes agree: still latched
    dog.clear()
    gps.arrived(10.0, fix=(HERE[0] + 20 / 111195.0, HERE[1]))
    assert state(dog, 'gps', 10.0) == OK


def test_gps_noise_and_fixes_that_arrive_together_are_not_jumps():
    dog = Watchdog([GPS])
    gps = dog.watches['gps']
    north = 1 / 111195.0           # a metre of latitude
    gps.arrived(0.0, fix=HERE, stamp=100.0)
    gps.arrived(0.2, fix=(HERE[0] + 2.5 * north, HERE[1]), stamp=100.2)   # 2.5 m of noise
    # Two queued fixes delivered at once: 0.2 s apart by their stamps, not 0 s.
    gps.arrived(0.5, fix=(HERE[0] + 1.4 * north, HERE[1]), stamp=100.4)
    gps.arrived(0.5, fix=(HERE[0] + 2.6 * north, HERE[1]), stamp=100.6)
    gps.arrived(0.5, fix=(HERE[0] + 2.6 * north, HERE[1]), stamp=100.6)   # the same again
    assert state(dog, 'gps', 0.5) == OK
    gps.arrived(0.7, fix=(HERE[0] + 23.0 * north, HERE[1]), stamp=100.8)  # 20 m in 0.2 s
    assert state(dog, 'gps', 0.7) == JUMP


def test_frozen_values_stop_after_frozen_after():
    dog = Watchdog([IMU])
    imu = dog.watches['imu']
    for k in range(5):
        imu.arrived(k * 0.1, (0.01 * k, 0.0), digest=k)
    for k in range(5, 20):
        imu.arrived(k * 0.1, (0.04, 0.0), digest='same')
    assert state(dog, 'imu', 1.5) == OK
    assert state(dog, 'imu', 1.95) == FROZEN and dog.locked(1.95)


def test_a_still_camera_is_frozen_only_while_the_car_moves():
    dog = Watchdog([CAMERA])
    cam = dog.watches['camera']
    for k in range(30):                               # parked: the same picture is fine
        cam.arrived(k * 0.1, digest='street', speed=0.0)
    assert state(dog, 'camera', 2.9) == OK
    for k in range(30, 45):                           # driving: the picture must change
        cam.arrived(k * 0.1, digest='street', speed=1.0)
    assert state(dog, 'camera', 4.4) == FROZEN


def test_a_silent_lidar_goes_stale_and_stops_the_drivers():
    dog = Watchdog([LIDAR])
    lidar = dog.watches['lidar']
    for k in range(10):                               # 10 Hz, a new cloud each time
        lidar.arrived(k * 0.1, digest=k, speed=1.0)
    assert state(dog, 'lidar', 1.8) == OK and not dog.locked(1.8)
    assert state(dog, 'lidar', 2.0) == STALE and dog.locked(2.0)    # 1.1 s of nothing
    lidar.arrived(2.1, digest=99, speed=1.0)           # back: drives again on its own
    assert state(dog, 'lidar', 2.1) == OK and not dog.locked(2.1)


def test_a_repeated_lidar_cloud_is_frozen_only_while_the_car_moves():
    dog = Watchdog([LIDAR])
    lidar = dog.watches['lidar']
    for k in range(30):                               # parked: the same cloud is fine
        lidar.arrived(k * 0.1, digest='kerb', speed=0.0)
    assert state(dog, 'lidar', 2.9) == OK
    for k in range(30, 45):                           # driving: the cloud must change
        lidar.arrived(k * 0.1, digest='kerb', speed=1.0)
    assert state(dog, 'lidar', 4.4) == FROZEN and dog.locked(4.4)
    lidar.arrived(4.5, digest='road', speed=1.0)      # a new cloud: OK again
    assert state(dog, 'lidar', 4.5) == OK


def test_a_cloud_without_frozen_after_never_freezes():
    dog = Watchdog([RADAR])
    radar = dog.watches['radar']
    for k in range(30):
        radar.arrived(k * 0.1, digest='same', speed=2.0)
    assert state(dog, 'radar', 2.9) == OK


def test_the_lidar_input_stops_as_the_camera_does():
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'sensor_watchdog.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    lidar, camera = cfg['lidar'], cfg['camera']
    assert lidar['topic'] == '/sensors/lidar/points' and lidar['kind'] == 'cloud'
    assert lidar['stop'] is True and lidar['stop'] == camera['stop']
    assert lidar['max_age'] == camera['max_age'] == 1.0     # both 10 Hz
    assert lidar['frozen_after'] == camera['frozen_after']


def test_the_hud_must_beat_before_and_while_nav2_drives():
    dog = Watchdog([HUD])
    hud = dog.watches['hud']
    assert state(dog, 'hud', 0.0) == NEVER and dog.locked(0.0)     # no screen yet
    for k in range(1, 11):                            # 10 Hz, the frames it has drawn
        hud.arrived(k * 0.1, digest=3 * k)
    assert state(dog, 'hud', 1.5) == OK and not dog.locked(1.5)
    assert state(dog, 'hud', 2.1) == STALE and dog.locked(2.1)     # frozen or dead
    hud.arrived(2.2, digest=0)                        # restarted: its count from 0 again
    assert not dog.locked(2.2)


def test_a_hud_that_beats_without_drawing_is_frozen():
    dog = Watchdog([HUD])
    hud = dog.watches['hud']
    for k in range(1, 30):                            # still sent, the count stuck
        hud.arrived(k * 0.1, digest=42)
    assert state(dog, 'hud', 2.9) == FROZEN and dog.locked(2.9)


def beat(dog, start, end, rate=10.0, count=None):
    """Beat for the HUD from start to end (s), the watchdog ticking at 10 Hz."""
    hud = dog.watches['hud']
    k = 0
    while start + k / rate <= end + 1e-9:
        t = start + k / rate
        hud.arrived(t, digest=(count or 0) + k + 1)
        dog.update(t, t)
        k += 1


def tick(dog, start, end):
    """Tick the watchdog at 10 Hz from start to end (s), with nothing arriving."""
    k = 0
    while start + k * 0.1 <= end + 1e-9:
        dog.update(start + k * 0.1, start + k * 0.1)
        k += 1


def test_one_dropout_while_the_hud_starts_is_forgiven():
    dog = Watchdog([LATCHED_HUD])
    beat(dog, 0.0, 2.0)                               # the window opens...
    tick(dog, 2.1, 3.5)                               # ...a slow first frame
    assert state(dog, 'hud', 3.5) == STALE and dog.locked(3.5)
    beat(dog, 3.6, 10.0, count=100)
    assert state(dog, 'hud', 10.0) == OK and not dog.locked(10.0)   # back on its own


def test_a_hud_that_drops_out_after_its_start_up_latches_until_cleared():
    dog = Watchdog([LATCHED_HUD])
    beat(dog, 0.0, 20.0)
    tick(dog, 20.1, 22.0)                             # it died
    assert state(dog, 'hud', 22.0) == LOST and dog.locked(22.0)
    beat(dog, 27.0, 40.0)                             # systemd restarted it: still held
    s, detail = dog.watches['hud'].state(40.0)
    assert s == LOST and 'STALE' in detail and 'OK now' in detail
    assert dog.locked(40.0)
    assert dog.summary(40.0).startswith('STOP: hud LOST')
    dog.clear()                                       # a person looked
    assert state(dog, 'hud', 40.0) == OK and not dog.locked(40.0)


def test_a_hud_that_keeps_crashing_while_it_starts_latches_the_second_time():
    dog = Watchdog([LATCHED_HUD])
    beat(dog, 0.0, 2.0)
    tick(dog, 2.1, 3.5)                               # one: forgiven
    beat(dog, 3.6, 5.0)
    tick(dog, 5.1, 6.5)                               # two, still within the grace
    assert state(dog, 'hud', 6.5) == LOST


def test_a_cleared_hud_gets_its_grace_again():
    dog = Watchdog([LATCHED_HUD])
    beat(dog, 0.0, 20.0)
    tick(dog, 20.1, 22.0)
    dog.clear()                                       # cleared while it is still dead
    assert state(dog, 'hud', 22.0) == STALE           # still holds, not latched
    tick(dog, 22.1, 29.9)
    beat(dog, 30.0, 31.0)                             # restarted; a slow frame
    tick(dog, 31.1, 32.5)
    assert state(dog, 'hud', 32.5) == STALE
    beat(dog, 32.6, 40.0, count=50)
    assert not dog.locked(40.0)


def test_a_stalled_watchdog_latches_nothing():
    # The watchdog itself stopped for 3 s (SIGSTOP): its first ticks back see the HUD
    # silent, as its beats still wait in the queue. They arrive, and nothing latched.
    dog = Watchdog([LATCHED_HUD], stall=0.5, settle=3.0)
    beat(dog, 0.0, 20.0)
    tick(dog, 23.0, 24.0)                             # the first ticks after the stall
    assert state(dog, 'hud', 24.0) == STALE
    beat(dog, 24.1, 30.0, count=1000)                 # the queued beats, then new ones
    assert state(dog, 'hud', 30.0) == OK and not dog.locked(30.0)


def test_a_hud_that_dies_in_a_stall_still_latches_after_it():
    dog = Watchdog([LATCHED_HUD], stall=0.5, settle=3.0)
    beat(dog, 0.0, 20.0)
    tick(dog, 23.0, 27.0)                             # it never comes back
    assert state(dog, 'hud', 27.0) == LOST


def test_a_rule_without_latch_never_latches():
    dog = Watchdog([HUD])
    beat(dog, 0.0, 20.0)
    tick(dog, 20.1, 22.0)
    beat(dog, 27.0, 28.0)
    assert state(dog, 'hud', 28.0) == OK


def test_the_summary_puts_what_stops_first():
    dog = Watchdog([RADAR, ODOM])
    dog.watches['odom'].arrived(0.0, (0.0,))
    assert dog.summary(0.6).startswith('STOP: odom STALE')
    assert 'radar NEVER' in dog.summary(0.6)


def test_a_warn_input_never_stops_the_drivers():
    dog = Watchdog([ODOM, RADAR])
    dog.watches['odom'].arrived(0.0, (0.0,))
    assert not dog.locked(0.3)                        # the radar has never spoken
    assert dog.summary(0.3).startswith('WARN: radar NEVER')
    assert dog.summary(0.6).startswith('STOP: odom STALE')


def test_ground_distance_is_metres():
    north = (HERE[0] + 100 / 111195.0, HERE[1])
    assert watchdog.ground_distance(HERE, north) == pytest.approx(100.0, rel=1e-3)


def test_rules_are_checked():
    with pytest.raises(ValueError):
        Rule('x', '/x', 'lidar', max_age=1.0)
    with pytest.raises(ValueError):
        Rule('x', '/x', 'odom', max_age=0.0)


LEFT = Rule('front_left_stream', '/perception/detections', 'detections', max_age=1.0,
            frame_id='camera_left_link')


def test_a_streamed_camera_is_watched_through_its_detections():
    dog = Watchdog([LEFT])
    left = dog.watches['front_left_stream']
    assert state(dog, 'front_left_stream', 0.0) == NEVER and dog.locked(0.0)
    for k in range(1, 16):                            # 15 a second, boxes or none
        left.arrived(k / 15.0, ())
    assert state(dog, 'front_left_stream', 1.5) == OK and not dog.locked(1.5)
    # The same empty list for seconds is a clear road, not a frozen camera.
    for k in range(16, 60):
        left.arrived(k / 15.0, ())
    assert state(dog, 'front_left_stream', 4.5) == OK
    # The stream, its decoder or its detector stops: no message at all.
    assert state(dog, 'front_left_stream', 5.0) == STALE and dog.locked(5.0)
    left.arrived(5.1, (math.nan, 10.0, 20.0, 30.0, 0.9))     # a broken detector
    assert state(dog, 'front_left_stream', 5.1) == NAN


def test_each_detections_input_counts_only_its_own_camera():
    assert LEFT.counts('camera_left_link')
    assert not LEFT.counts('camera_right_link')       # the other stream, same topic
    assert CAMERA.counts('anything')                  # other kinds have no filter


def test_detections_rules_are_checked():
    with pytest.raises(ValueError, match='frame_id'):
        Rule('x', '/perception/detections', 'detections', max_age=1.0)
    with pytest.raises(ValueError, match='frame_id'):
        Rule('x', '/x', 'image', max_age=1.0, frame_id='camera_left_link')
    with pytest.raises(ValueError, match='frozen'):
        Rule('x', '/perception/detections', 'detections', max_age=1.0,
             frame_id='camera_left_link', frozen_after=1.0)


def test_traffic_is_the_rate_and_the_gaps_for_people():
    watch = Watchdog([IMU]).watches['imu']
    assert watch.traffic(5.0) == (0.0, 0.0, 0.0)      # nothing yet
    for i in range(10):                               # 10 Hz for a second
        watch.arrived(10.0 + i * 0.1, (0.0, 0.0, float(i)), digest=i)
    rate, gap, worst = watch.traffic(10.95)
    assert rate == pytest.approx(10 / 0.95)
    assert gap == pytest.approx(0.1) and worst == pytest.approx(0.1)
    watch.arrived(11.5, (0.0, 0.0, 1.0), digest=10)   # 0.6 s of silence, then one
    rate, gap, worst = watch.traffic(12.0)
    assert rate == pytest.approx(1 / 1.05)
    assert gap == pytest.approx(0.6) and worst == pytest.approx(0.6)


def test_traffic_counts_a_silence_that_has_not_ended():
    watch = Watchdog([ODOM]).watches['odom']
    watch.arrived(1.0, (0.0, 0.0))
    watch.traffic(1.5)
    rate, gap, worst = watch.traffic(4.0)             # silent since 1.0
    assert rate == 0.0 and gap == pytest.approx(3.0) and worst == pytest.approx(3.0)
    watch.arrived(4.1, (0.0, 0.0))
    _, gap, worst = watch.traffic(4.2)
    assert gap == pytest.approx(3.1) and worst == pytest.approx(3.1)


PLANNER = Rule('planner', '/planner', 'node', max_age=3.0, stop=False)


def test_a_node_that_leaves_the_graph_warns_and_never_stops():
    dog = Watchdog([PLANNER])
    assert state(dog, 'planner', 0.0) == NEVER and not dog.locked(0.0)
    dog.watches['planner'].arrived(1.0)
    assert state(dog, 'planner', 3.9) == OK
    assert state(dog, 'planner', 4.1) == STALE and not dog.locked(4.1)
    dog.watches['planner'].arrived(4.2)                # respawned
    assert state(dog, 'planner', 4.2) == OK


def test_a_node_input_may_not_stop_the_car():
    with pytest.raises(ValueError, match='only warns'):
        Rule('planner', '/planner', 'node', max_age=3.0)
    with pytest.raises(ValueError, match='full name'):
        Rule('planner', 'planner', 'node', max_age=3.0, stop=False)


# TODO 171: our nodes in their own words, warn only by default.
COSTMAP = Rule('costmap', '/drivers/nav/costmap/status', 'status', max_age=3.0, stop=False,
               status_name='costmap local', min_level=1)
PLANS = Rule('plans', '/drivers/nav/planner/status', 'status', max_age=3.0, stop=False,
             quiet_ok=True)


def test_a_node_that_says_it_waits_warns_in_its_own_words():
    dog = Watchdog([COSTMAP])
    w = dog.watches['costmap']
    assert state(dog, 'costmap', 0.0) == NEVER
    w.arrived(1.0, (0,), words='local costmap: 0.4 ms an update')
    assert state(dog, 'costmap', 1.0) == OK
    w.arrived(2.0, (1,), words='local costmap: waiting for odometry')
    s, detail = w.state(2.0)
    assert s == watchdog.FAULT and detail == 'says WARN: local costmap: waiting for odometry'
    assert not dog.locked(2.0), 'warn only'
    assert 'WARN: costmap FAULT' in dog.summary(2.0)
    w.arrived(2.5, (0,), words='fine')
    assert state(dog, 'costmap', 2.5) == OK
    assert state(dog, 'costmap', 5.6) == STALE, 'a hung node says nothing'


def test_only_its_own_name_counts_on_a_shared_topic():
    assert COSTMAP.hears('costmap local') and not COSTMAP.hears('costmap global')
    assert PLANS.hears('anything') and Rule('a', '/a', 'odom', 1.0).hears('x')


def test_a_quiet_node_is_ok_and_its_old_complaint_fades():
    dog = Watchdog([PLANS])
    w = dog.watches['plans']
    assert w.state(0.0) == (OK, 'quiet: nothing yet')
    w.arrived(1.0, (2,), words='no path: the goal is in a wall')
    assert state(dog, 'plans', 1.0) == watchdog.FAULT
    assert state(dog, 'plans', 2.0) == watchdog.FAULT
    assert w.state(4.5) == (OK, 'quiet for 3.5 s')
    w.arrived(5.0, (1,), words='slow')
    assert state(dog, 'plans', 5.0) == OK, 'WARN is under its min_level, ERROR'


def test_a_status_input_can_be_switched_to_stop():
    rule = Rule('nav', '/drivers/nav/navigator/status', 'status', max_age=2.0)
    dog = Watchdog([rule])
    dog.watches['nav'].arrived(0.0, (2,), words='broken')
    assert dog.locked(0.0)


def test_the_configs_status_inputs_are_good_rules_that_only_warn():
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'sensor_watchdog.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    status = {k: v for k, v in params.items() if isinstance(v, dict) and v['kind'] == 'status'}
    assert set(status) == {'status_costmap_local', 'status_costmap_global', 'status_planner',
                           'status_navigator', 'status_mppi_lag', 'pose_check'}
    for name, p in status.items():
        rule = Rule(name, p['topic'], 'status', p['max_age'], stop=p['stop'],
                    status_name=p.get('status_name', ''), min_level=p.get('min_level', 2),
                    quiet_ok=p.get('quiet_ok', False))
        if name == 'pose_check':
            # Not navigation's: where the car is (TODO 210). HOLD (ERROR) and silence stop.
            assert rule.stop and rule.min_level == 2 and not rule.quiet_ok
        else:
            assert rule.stop is False, f'{name}: warn only (Ben, 18 Nov)'


def test_with_ndt_leading_the_gps_checks_only_warn():
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'sensor_watchdog.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    gps, check = params['gps'], params['gps_check']
    assert gps['stop'] is True and check['stop'] is False
    assert {k: v for k, v in gps.items() if k != 'stop'} == {
        k: v for k, v in check.items() if k != 'stop'}


def test_status_settings_belong_to_status_inputs():
    with pytest.raises(ValueError, match='status input'):
        Rule('odom', '/vehicle/odom', 'odom', 0.5, quiet_ok=True)
    with pytest.raises(ValueError, match='status input'):
        Rule('odom', '/vehicle/odom', 'odom', 0.5, status_name='x')
    with pytest.raises(ValueError, match='min_level'):
        Rule('s', '/s', 'status', 1.0, min_level=0)


# TODO 207: an actuator that doesn't follow its command (steering stuck in a bend).
STEERING = Rule('steering', '/vehicle/steering', 'actuator', max_age=1.0,
                follow_tolerance=0.15, follow_persist=1.0)


def steer(dog, now, command, measured):
    dog.watches['steering'].arrived(now, (command, measured))


def test_steering_that_follows_its_command_is_ok_even_while_it_catches_up():
    dog = Watchdog([STEERING])
    steer(dog, 0.0, 0.0, 0.0)
    # A step to 0.4 rad, caught up in 0.5 s (under follow_persist): never stuck.
    for i in range(1, 11):
        steer(dog, i * 0.1, 0.4, min(0.4, i * 0.08))
    assert state(dog, 'steering', 1.0) == OK
    assert not dog.locked(1.0)


def test_steering_stuck_off_its_command_latches_stuck_and_stops_the_drivers():
    dog = Watchdog([STEERING])
    for i in range(0, 16):
        steer(dog, i * 0.1, 0.4, 0.05)       # asked 0.4, stays at 0.05
    s, words = dog.watches['steering'].state(1.5)
    assert s == watchdog.STUCK and 'asked +0.40, did +0.05' in words
    assert dog.locked(1.5)
    # It latches: following again doesn't undo it, a person's clear does.
    steer(dog, 1.6, 0.05, 0.05)
    assert state(dog, 'steering', 1.6) == watchdog.STUCK
    dog.clear()
    steer(dog, 1.7, 0.05, 0.05)
    assert state(dog, 'steering', 1.7) == OK


def test_a_short_miss_resets_and_nan_is_no_fault():
    dog = Watchdog([STEERING])
    steer(dog, 0.0, 0.4, 0.0)
    steer(dog, 0.9, 0.4, 0.0)                # off 0.9 s
    steer(dog, 1.0, 0.4, 0.38)               # back within: the clock restarts
    steer(dog, 1.5, 0.4, 0.0)
    steer(dog, 2.2, 0.4, 0.0)                # off 0.7 s only
    assert state(dog, 'steering', 2.2) == OK
    # The car today: the command side is known, the measured side NaN. No check, no fault.
    for i in range(30):
        steer(dog, 3.0 + i * 0.1, 0.4, math.nan)
    assert state(dog, 'steering', 5.9) == OK
    # Silence is still STALE.
    assert state(dog, 'steering', 8.0) == STALE


def test_only_an_actuator_input_has_a_follow_check():
    with pytest.raises(ValueError, match='actuator'):
        Rule('odom', '/vehicle/odom', 'odom', max_age=0.5, follow_tolerance=0.1)
    with pytest.raises(ValueError, match='follow_persist'):
        Rule('steering', '/vehicle/steering', 'actuator', max_age=1.0, follow_persist=0.0)


# TODO 207, the throttle half: the car faster than ego asked (a stuck throttle, a loop that
# runs away). The margin, warning and persist are config/sensor_watchdog.yaml's.
SPEED = Rule('speed', '/ego/actuator_cmd', 'speed', max_age=1.0, over_margin=0.2,
             over_warn=1.0, over_persist=2.0, over_slowing=0.3)


def drive(dog, start, seconds, command, measured, dt=0.02):
    """Feed (command, measured) at 50 Hz; each may be a number or a function of t."""
    t = start
    for i in range(int(round(seconds / dt))):
        t = start + i * dt
        c = command(t) if callable(command) else command
        m = measured(t) if callable(measured) else measured
        dog.watches['speed'].arrived(t, (c, m))
    return t


def test_the_loops_overshoot_is_under_the_margin():
    dog = Watchdog([SPEED])
    # The car model's default loop: 1.144 m/s for 1.0 asked, held 10 s.
    t = drive(dog, 0.0, 10.0, 1.0, 1.144)
    assert state(dog, 'speed', t) == OK
    assert not dog.locked(t)


def test_a_healthy_stop_lagging_its_command_never_counts():
    dog = Watchdog([SPEED])
    t = drive(dog, 0.0, 2.0, 1.0, 1.0)
    # Ego ramps 1 -> 0 at 1 m/s^2; the car follows 0.3 s late, then brakes at 1 m/s^2,
    # 0.7 m/s over its command at worst: slowing all the while, so no warning.
    t = drive(dog, t, 3.0, lambda s: max(0.0, 1.0 - (s - 2.0)),
              lambda s: 1.0 if s < 2.7 else max(0.0, 1.0 - (s - 2.7)))
    assert state(dog, 'speed', t) == OK
    # An e-stop: the command 0 at once, the car 2.5 -> 0 at 1.5 m/s^2 (1.7 s over 0.2).
    t = drive(dog, t, 2.0, 2.5, 2.5)
    t = drive(dog, t, 2.5, 0.0, lambda s, t0=t: max(0.0, 2.5 - 1.5 * (s - t0)))
    assert state(dog, 'speed', t) == OK


def test_a_stuck_throttle_warns_then_latches_overspeed_and_stops_the_drivers():
    dog = Watchdog([SPEED])
    t = drive(dog, 0.0, 1.2, 1.0, 1.3)          # 0.3 m/s over, steady: not slowing
    s, words = dog.watches['speed'].state(t)
    assert s == watchdog.OVER and 'asked 1.00 m/s, did 1.30' in words
    assert not dog.locked(t)                    # a warning only
    assert dog.summary(t).startswith('WARN')
    t = drive(dog, t, 1.0, 1.0, 1.3)
    s, words = dog.watches['speed'].state(t)
    assert s == watchdog.OVERSPEED
    assert dog.locked(t) and dog.summary(t).startswith('STOP')
    # Latched: back under the command doesn't undo it; a person's clear does.
    t = drive(dog, t, 1.0, 1.0, 1.0)
    assert state(dog, 'speed', t) == watchdog.OVERSPEED
    dog.clear()
    t = drive(dog, t, 0.1, 1.0, 1.0)
    assert state(dog, 'speed', t) == OK


def test_reversing_counts_by_size_and_nan_is_no_check():
    dog = Watchdog([SPEED])
    t = drive(dog, 0.0, 3.0, -0.5, -0.9)        # backing 0.4 m/s faster than asked
    assert state(dog, 'speed', t) == watchdog.OVERSPEED
    dog = Watchdog([SPEED])
    t = drive(dog, 0.0, 3.0, 1.0, math.nan)     # no wheel speed yet: no check
    assert state(dog, 'speed', t) == OK


def test_a_stale_wheel_speed_or_command_never_counts():
    """sim-check --faults' "odom stop": the wheels' last 0.5 m/s, kept, is no overspeed."""
    dog = Watchdog([SPEED])
    speed = dog.watches['speed']
    t = drive(dog, 0.0, 2.0, 0.5, 0.5)
    # The wheels go quiet at 2.0 s; ego, stopped by the lock, asks 0 for 10 s more.
    for i in range(500):
        now = 2.0 + i * 0.02
        speed.arrived(now, (0.0, 0.5, now - 2.0))
        if now - 2.0 > SPEED.max_age:
            assert speed.over_since is None, now
    assert state(dog, 'speed', now) == OK and not dog.locked(now)
    # A fresh wheel speed that runs over still counts, as before.
    for i in range(150):
        now = 12.0 + i * 0.02
        speed.arrived(now, (0.0, 0.5, 0.01))
    assert state(dog, 'speed', now) == watchdog.OVERSPEED
    # A command after a gap longer than max_age starts afresh: no OVER carried across it.
    dog = Watchdog([SPEED])
    t = drive(dog, 0.0, 0.9, 1.0, 1.3)          # over since 0.0 s, under over_warn
    t = drive(dog, t + 1.5, 0.2, 1.0, 1.3)      # ego silent 1.5 s, then over again
    assert state(dog, 'speed', t) == OK         # 0.2 s over, not 2.6 s


def test_the_car_models_healthy_drives_never_warn():
    """Every speed loop profile, ramps up and down and a step to 0: never OVER."""
    car_model = pytest.importorskip('aav_description.car_model')
    for profile in car_model.SPEED_LOOP_PROFILES:
        for top in (1.0, 2.5):
            for step_stop in (False, True):
                dog = Watchdog([SPEED])
                car = car_model.Actuators(loop=car_model.speed_loop_profile(profile))
                command, dt = 0.0, 0.02
                for i in range(int(30 / dt)):
                    t = i * dt
                    target = top if t < 15.0 else 0.0
                    if step_stop and t >= 15.0:
                        command = 0.0
                    elif target > command:
                        command = min(target, command + 0.5 * dt)
                    else:
                        command = max(target, command - 1.0 * dt)
                    s = car.step(command, 0.0, dt)
                    dog.watches['speed'].arrived(t, (command, s.speed))
                    assert state(dog, 'speed', t) == OK, (profile, top, step_stop, t)


def test_only_a_speed_input_has_an_overspeed_check():
    with pytest.raises(ValueError, match='speed input'):
        Rule('odom', '/vehicle/odom', 'odom', max_age=0.5, over_margin=0.2)
    with pytest.raises(ValueError, match='over_warn'):
        Rule('speed', '/ego/actuator_cmd', 'speed', max_age=1.0, over_margin=0.2,
             over_warn=3.0, over_persist=2.0)


def test_the_car_model_with_its_throttle_stuck_while_speeding_up_latches_overspeed():
    """The throttle stuck 1 s into speeding up to 1 m/s: the car holds 1.25 m/s or more."""
    car_model = pytest.importorskip('aav_description.car_model')
    dog = Watchdog([SPEED])
    car = car_model.Actuators(loop=car_model.speed_loop_profile('default'))
    command, dt, held, seen = 0.0, 0.02, None, None
    for i in range(int(15 / dt)):
        t = i * dt
        command = min(1.0, command + 0.5 * dt)
        fault = None
        if t >= 1.0:
            held = car.throttle if held is None else held
            fault = (lambda h: lambda d, steer: (car_model.Demand(h, d.brake, d.reverse),
                                                 steer))(held)
        s = car.step(command, 0.0, dt, fault)
        dog.watches['speed'].arrived(t, (command, s.speed))
        if seen is None and state(dog, 'speed', t) == watchdog.OVERSPEED:
            seen = t
    assert seen is not None and seen < 15.0

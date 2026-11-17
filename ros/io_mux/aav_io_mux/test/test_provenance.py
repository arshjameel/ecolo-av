"""The sensor mux's and the output mux's rules (provenance.py): no ROS needed."""

from aav_io_mux import provenance as p
from aav_io_mux.provenance import BENCH, Entry, Gate, REAL, REPLAY, Report, SIM
import pytest

IMU = '/sensors/imu/data_raw'
GPS = '/sensors/gps/fix'
CAMERA = '/sensors/camera/front_left/image_raw'


def gazebo(*topics, kind=SIM):
    return [Entry('sim_adapter', t.split('/')[-2], t, kind) for t in topics]


# --- sources and targets ---

def test_each_target_needs_its_own_source_and_topic():
    assert p.target_of('car') == REAL
    assert p.target_of('sim') == p.target_of('turtle') == p.target_of('carla') == SIM
    assert p.target_of('bench') == BENCH
    assert p.command_topic('car') == '/control/car/actuator_cmd'
    assert p.command_topic('sim') == '/control/sim/actuator_cmd'


@pytest.mark.parametrize('bad', ['', 'real', 'Car', 'gazebo'])
def test_an_unknown_target_is_refused(bad):
    with pytest.raises(ValueError, match='target:='):
        p.command_topic(bad)


def test_a_launch_takes_the_targets_own_source_and_no_other():
    assert p.choose('car', '') == ('car', 'real')
    assert p.choose('sim', 'sim') == ('sim', 'sim')
    assert p.choose('bench', '') == ('bench', 'bench')
    assert p.choose('', 'replay') == ('', 'replay')
    with pytest.raises(ValueError, match='target:=car needs source:=real, not sim'):
        p.choose('car', 'sim')
    with pytest.raises(ValueError, match='target:=sim needs source:=sim, not real'):
        p.choose('sim', 'real')
    with pytest.raises(ValueError, match='give target'):
        p.choose('', '')


def test_sources_by_name():
    assert [p.source_of(s) for s in ('real', 'sim', 'bench', 'replay')] == [
        REAL, SIM, BENCH, REPLAY]
    with pytest.raises(ValueError, match='source:='):
        p.source_of('gazebo')


def test_an_unknown_number_is_never_real():
    assert p.kind_name(7) == 'UNKNOWN(7)'
    assert p.foreign([Entry('new_node', 'imu', IMU, 7)], REAL) == ['imu is UNKNOWN(7) (new_node)']


# --- announcements ---

def heard(*announcements, now=0.0):
    a = p.Announcers()
    for entries in announcements:
        a.hear(entries[0].announcer, entries, now)
    return a


LEGACY = [Entry('legacy_uart_bridge', 'actuators', '', REAL)]


def test_an_announcement_is_forgotten_once_its_announcer_has_gone():
    a = heard(gazebo(IMU), LEGACY)
    both = {'sim_adapter', 'legacy_uart_bridge'}
    assert {e.announcer for e in a.entries(both, 1.0)} == both
    # Gazebo stops: its SIM entries stop counting, so a stop can be cleared.
    assert [e.announcer for e in a.entries({'legacy_uart_bridge'}, 2.0)] == [
        'legacy_uart_bridge']
    assert a.entries(set(), 3.0) == []


def test_an_announcement_counts_before_the_graph_shows_its_announcer():
    # The message can come first (24 Oct): ignoring it would let a SIM source through.
    a = heard(gazebo(IMU), now=10.0)
    assert [e.announcer for e in a.entries(set(), 10.2)] == ['sim_adapter']
    assert p.foreign(a.entries(set(), 10.2), REAL) == ['imu is SIM (sim_adapter)']
    # Never in the graph at all: forgotten after the grace (a short-lived publisher).
    assert a.entries(set(), 10.0 + a.grace) == []


def test_an_announcer_heard_again_is_fresh():
    a = heard(gazebo(IMU), now=0.0)
    a.entries({'sim_adapter'}, 1.0)          # seen in the graph
    a.hear('sim_adapter', gazebo(IMU), 50.0)   # Gazebo started again
    assert len(a.entries(set(), 50.1)) == 1      # counts before the graph shows it again


def test_an_announcer_in_the_graph_not_heard_yet_is_a_doubt():
    # The graph can come first, by up to 30 ms across processes (24 Oct): wait for its
    # announcement, which may say SIM.
    a = heard(LEGACY)
    alive = {'legacy_uart_bridge', 'sim_adapter'}
    assert a.unheard(alive) == [
        'sim_adapter publishes /system/source, its announcement not heard yet']
    check = p.check_sensors(REAL, a.entries(alive, 0.0), {}, unheard=a.unheard(alive))
    assert not check.ok and not check.problems and len(check.doubts) == 1
    a.hear('sim_adapter', gazebo(IMU), 0.1)
    assert a.unheard(alive) == []


def test_foreign_lists_what_isnt_the_source_asked_for():
    entries = gazebo(IMU) + [Entry('spine_bridge', 'odometry', '/vehicle/odom', REAL)]
    assert p.foreign(entries, REAL) == ['imu is SIM (sim_adapter)']
    assert p.foreign(entries, SIM) == ['odometry is REAL (spine_bridge)']


# --- the sensor mux ---

def test_the_car_with_its_stock_drivers_is_all_real():
    # ublox_gps and the IMU driver announce nothing; legacy_uart_bridge says REAL.
    check = p.check_sensors(REAL, [Entry('legacy_uart_bridge', 'actuators', '', REAL)],
                            {IMU: ['imu_yahboom'], GPS: ['ublox_gps_node']})
    assert check.ok and check.problems == ()
    assert [(s.name, s.kind) for s in check.sensors] == [('imu', REAL), ('gps', REAL)]


def test_gazebo_on_the_cars_network_is_caught():
    check = p.check_sensors(REAL, gazebo(IMU), {IMU: ['sim_imu']})
    assert not check.ok
    assert check.problems == ('imu is SIM (sim_adapter)',)
    assert check.sensors[0].kind == SIM


def test_a_stand_in_that_announces_anything_is_caught_even_without_a_sensor():
    # mock_battery says BENCH, and no watched topic has a publisher yet.
    check = p.check_sensors(REAL, [Entry('mock_battery', 'battery', '/vehicle/battery', BENCH)],
                            {})
    assert not check.ok and 'battery is BENCH (mock_battery)' in check.problems


TWO = ('imu: 2 publishers on /sensors/imu/data_raw (imu_yahboom, rosbag2_player): two '
       'sources at once')


def test_two_publishers_on_one_sensor_are_two_sources():
    # A bag playing beside the real driver: the bag announces nothing.
    check = p.check_sensors(REAL, [], {IMU: ['imu_yahboom', 'rosbag2_player']})
    assert not check.ok
    assert check.problems == () and check.doubts == (TWO,)


def test_a_doubt_becomes_a_problem_once_it_lasts():
    hold = p.Hold(hold=5.0)
    two = p.check_sensors(REAL, [], {IMU: ['imu_yahboom', 'rosbag2_player']})
    assert hold.judge(0.0, two).doubts == (TWO,)
    assert hold.judge(4.9, two).problems == ()
    late = hold.judge(5.0, two)
    assert late.problems == (TWO,) and late.doubts == () and not late.ok


def test_a_doubt_that_goes_away_starts_again():
    # Discovery: a publisher seen before its announcement, for a moment.
    hold = p.Hold(hold=5.0)
    unknown = p.check_sensors(SIM, [], {IMU: ['_NODE_NAME_UNKNOWN_']})
    known = p.check_sensors(SIM, gazebo(IMU), {IMU: ['sim_imu']})
    assert hold.judge(0.0, unknown).doubts
    assert hold.judge(3.0, known).ok
    assert hold.judge(4.0, unknown).doubts and hold.judge(8.9, unknown).problems == ()
    # An announcement of another source never waits.
    assert hold.judge(9.0, p.check_sensors(REAL, gazebo(IMU), {IMU: ['sim_imu']})).problems


def test_gazebo_in_gazebo_is_fine_and_absent_sensors_are_not_faults():
    check = p.check_sensors(SIM, gazebo(IMU, GPS, CAMERA),
                            {IMU: ['sim_imu'], GPS: ['sim_gps']})
    assert check.ok
    assert [s.name for s in check.sensors] == ['imu', 'gps']


def test_an_unannounced_sensor_in_a_simulator_is_a_real_one():
    check = p.check_sensors(SIM, gazebo(IMU), {IMU: ['sim_imu'], GPS: ['ublox_gps_node']})
    assert not check.ok
    assert check.doubts == ('gps: /sensors/gps/fix from ublox_gps_node, which nobody '
                            'announces on /system/source: a real driver?',)


def test_a_real_actuator_in_a_simulator_is_caught():
    check = p.check_sensors(SIM, gazebo(IMU) + [Entry('legacy_uart_bridge', 'actuators', '',
                                                      REAL)], {IMU: ['sim_imu']})
    assert check.problems == ('actuators is REAL (legacy_uart_bridge)',)


def test_the_bench_and_a_replay():
    bench = [Entry('mock_imu', 'imu', IMU, BENCH)]
    assert p.check_sensors(BENCH, bench, {IMU: ['mock_imu']}).ok
    replay = [Entry('replay_source', 'imu', IMU, REPLAY)]
    assert p.check_sensors(REPLAY, replay, {IMU: ['rosbag2_player']}).ok
    assert not p.check_sensors(BENCH, replay, {IMU: ['rosbag2_player']}).ok


# --- the output mux ---

def ok(t, source=REAL):
    return Report(time=t, source=source, ok=True)


def test_the_car_opens_only_after_settling_with_all_real():
    gate = Gate('car')
    assert not gate.is_open(0.0)
    assert gate.state(0.0) == ('WAIT - car: waiting for the sensor mux '
                               '(/system/sensor_mux/state)')
    gate.report = ok(0.5)
    assert not gate.is_open(0.5)
    assert gate.state(1.0).startswith('WAIT - car: all REAL, settling')
    gate.report = ok(2.4)
    assert gate.is_open(2.5)
    assert gate.state(2.5) == 'OPEN - car: every source REAL'


def test_waiting_for_the_sensor_mux_never_latches():
    gate = Gate('car')
    for t in range(10):
        assert not gate.is_open(float(t))
    assert gate.latched == ()


def test_a_simulated_source_cant_drive_the_real_car_and_the_stop_latches():
    gate = Gate('car')
    gate.report = ok(0.0)
    assert not gate.is_open(0.0)
    gate.report = ok(2.0)
    assert gate.is_open(2.0)
    # Gazebo starts somewhere on the car's network.
    gate.entries = gazebo(CAMERA)
    assert not gate.is_open(2.1)
    assert gate.state(2.1) == ('STOP - target car needs REAL - front_left is SIM '
                               '(sim_adapter) - latched: /safety/output_mux/clear')
    # Gazebo goes away: still stopped, until a person clears it.
    gate.entries = []
    gate.report = ok(3.0)
    assert not gate.is_open(3.0)
    assert gate.clear(3.0)[0]
    assert not gate.is_open(4.0)          # settling again
    gate.report = ok(5.0)
    assert gate.is_open(5.0)


def test_clearing_is_refused_while_anything_is_wrong():
    gate = Gate('car')
    gate.report = ok(0.0)
    gate.update(0.0)
    gate.entries = gazebo(IMU)
    gate.update(0.1)
    done, why = gate.clear(0.2)
    assert not done and why == 'still wrong: imu is SIM (sim_adapter)'
    assert gate.latched


def test_the_sensor_mux_saying_no_latches_with_its_reasons():
    gate = Gate('car')
    gate.report = Report(0.0, REAL, False, problems=('imu: 2 publishers',))
    assert not gate.is_open(0.0)
    assert gate.latched == ('imu: 2 publishers',)


def test_a_sensor_mux_checking_for_the_wrong_source_latches():
    gate = Gate('car')
    gate.report = ok(0.0, source=SIM)
    assert not gate.is_open(0.0)
    assert gate.latched == ('the sensor mux checks for SIM, target car needs REAL',)


def test_a_silent_sensor_mux_stops_the_car_until_it_speaks_again():
    gate = Gate('car', settle=1.0)
    gate.report = ok(0.0)
    gate.update(0.0)
    assert gate.is_open(2.5)
    assert not gate.is_open(3.1)
    assert gate.state(3.1) == 'WAIT - car: the sensor mux is quiet (3.1 s)'
    assert gate.latched == ()           # no evidence of a wrong source: only waits
    gate.report = ok(4.0)
    gate.update(4.0)                    # as the node does when a report arrives
    assert not gate.is_open(4.5)        # settling again
    assert gate.is_open(5.0)


def test_the_sensor_mux_unsure_holds_the_gate_without_latching():
    gate = Gate('car', settle=0.0)
    gate.report = Report(0.0, REAL, False, doubts=(TWO,))
    assert not gate.is_open(0.0)
    assert gate.latched == ()
    assert gate.state(0.0) == 'WAIT - car: unsure: ' + TWO
    gate.report = ok(1.0)
    assert gate.is_open(1.0)
    # The doubt lasted: the sensor mux says problem, and the stop latches.
    gate.report = Report(6.0, REAL, False, problems=(TWO,))
    assert not gate.is_open(6.0)
    assert gate.latched == (TWO,)


def test_an_announcer_not_heard_yet_holds_the_gate_and_its_sim_latches():
    unheard = 'sim_adapter publishes /system/source, its announcement not heard yet'
    gate = Gate('car', settle=0.0)
    gate.report = ok(0.0)
    assert gate.is_open(0.0)
    gate.unheard = [unheard]
    assert not gate.is_open(0.1) and gate.latched == ()
    assert gate.state(0.1) == f'WAIT - car: {unheard}'
    # Its announcement arrives: SIM, a stop.
    gate.unheard, gate.entries = [], gazebo(CAMERA)
    assert not gate.is_open(0.2)
    assert gate.latched == ('front_left is SIM (sim_adapter)',)


def test_gazebo_refuses_a_real_backend_in_its_graph():
    gate = Gate('sim', settle=0.0)
    gate.report = ok(0.0, source=SIM)
    gate.entries = gazebo(IMU)
    assert gate.is_open(0.0)
    gate.entries.append(Entry('legacy_uart_bridge', 'actuators', '', REAL))
    assert not gate.is_open(0.1)
    assert gate.latched == ('actuators is REAL (legacy_uart_bridge)',)


def test_the_bench_needs_bench_sources():
    gate = Gate('bench', settle=0.0)
    gate.report = ok(0.0, source=BENCH)
    gate.entries = [Entry('mock_imu', 'imu', IMU, BENCH)]
    assert gate.is_open(0.0)
    gate.entries.append(Entry('sim_adapter', 'gps', GPS, SIM))
    assert not gate.is_open(0.1)


def open_gate(**args):
    gate = Gate('sim', settle=0.0, **args)
    gate.report = Report(0.0, SIM, True, (), ())
    return gate


def test_commands_wait_for_the_screens_they_need():
    gate = open_gate(screens=('dev_window',))
    assert not gate.is_open(0.5), 'not heard yet'
    gate.beat('dev_window', 0.5)
    gate.update(0.5)
    assert gate.is_open(0.6)
    assert not gate.is_open(1.7), 'quiet for over a second'
    assert not gate.latched, 'a quiet screen only waits'
    gate.report = Report(1.8, SIM, True, (), ())
    gate.beat('dev_window', 1.8)
    gate.update(1.8)
    assert gate.is_open(1.9), 'and comes back'


def test_every_screen_named_must_beat():
    gate = open_gate(screens=('dev_window', 'hw_panel'))
    gate.beat('dev_window', 0.5)
    gate.update(0.5)
    assert not gate.is_open(0.6)
    gate.beat('hw_panel', 0.6)
    gate.update(0.6)
    assert gate.is_open(0.7)


def test_no_screens_named_changes_nothing():
    assert open_gate().is_open(0.1)


def test_the_real_car_refuses_the_screens_rule():
    with pytest.raises(ValueError, match='takes no screens'):
        Gate('car', screens=('dev_window',))

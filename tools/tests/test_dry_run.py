"""The dry run's plan and its stop (tools/dry_run.py): no ROS, no simulator."""
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import dry_run  # noqa: E402
import pytest  # noqa: E402


def test_the_simulator_comes_first_with_what_every_rung_needs():
    parts = dry_run.plan(dry_run.parse([]))
    assert [p.name for p in parts] == ['the simulator', "the simulator's picture",
                                       'the ladder window']
    sim = parts[0].cmd
    assert sim[:2] == ('tools/ros.sh', '--local')
    for switch in ('world:=road', 'nav:=false', 'faults:=true', 'sim_speed:=3', 'rviz:=false'):
        assert switch in sim
    assert parts[0].needed and parts[0].log and not parts[0].after_sim


def test_your_switches_come_last_so_they_win():
    parts = dry_run.plan(dry_run.parse(['sim_speed:=2', 'world:=empty']))
    assert parts[0].cmd[-2:] == ('sim_speed:=2', 'world:=empty')
    with pytest.raises(SystemExit):
        dry_run.parse(['stands'])


def test_the_extra_screens_are_asked_for_and_everything_with_ros_stays_local():
    args = dry_run.parse(['--panel', '--debug', '--no-picture'])
    parts = dry_run.plan(args)
    assert [p.name for p in parts] == ['the simulator', 'the ladder window',
                                       'the hardware panel', 'the debug screen']
    for part in parts:
        if 'ros2' in part.cmd:
            assert part.cmd[:2] == ('tools/ros.sh', '--local'), part.name
    # The picture's script keeps itself local (tools/dev_window.sh --live).
    picture = dry_run.plan(dry_run.parse([]))[1].cmd
    assert picture == ('tools/dev_window.sh', '--live')
    text = open(os.path.join(dry_run.ROOT, 'tools/dev_window.sh')).read()
    assert 'exec tools/ros.sh --local cargo run' in text


def test_it_refuses_beside_a_simulator_or_anything_that_is_not_one():
    quiet = {'simulator': False, 'message': 'no simulator heard on this computer (nothing '
             'on /system/source): start one with pixi run sim'}
    assert dry_run.why_not_start(quiet) is None
    assert 'already running' in dry_run.why_not_start({'simulator': True, 'message': ''})
    real = {'simulator': False, 'message': '/system/source says REAL (legacy_bridge)'}
    assert 'REAL' in dry_run.why_not_start(real)
    assert dry_run.why_not_start(None)


def test_stop_ends_each_group_and_what_it_started():
    part = dry_run.Part('a sleeper', ('sh', '-c', 'sleep 60 & echo $!; wait'))
    proc = subprocess.Popen(part.cmd, stdout=subprocess.PIPE, text=True,
                            start_new_session=True)
    grandchild = int(proc.stdout.readline())
    assert dry_run.alive(proc.pid)
    left = dry_run.stop([('a sleeper', proc)], wait=5.0)
    assert left == []
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        try:
            os.kill(grandchild, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        pytest.fail('the grandchild is still running')

"""pixi run screens (tools/screens.py): which windows, which car, the stop. No ROS."""
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402
import screens  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SIM_HEARD = {'simulator': True, 'message': ''}
NOTHING = {'simulator': False, 'message': 'no simulator heard on this computer'}
REAL = {'simulator': False, 'message': '/system/source says REAL (legacy_uart_bridge): ...'}


def test_the_virtual_car_gets_all_three_windows_on_this_computer_only():
    windows, notes = screens.plan(screens.SIM)
    assert [w.name for w in windows] == ['world', 'hud', 'map']
    assert notes == []
    assert windows[0].cmd == ('tools/dev_window.sh', '--live')   # --live is local inside
    for w in windows[1:]:
        assert w.cmd[:2] == ('tools/ros.sh', '--local'), w.name
    assert windows[1].cmd[-1] == 'hud.launch.py'
    assert windows[2].cmd[-1] == 'map.launch.py'     # with its HARDWARE page (F2)


def test_the_real_car_is_only_watched():
    windows, notes = screens.plan(screens.CAR)
    assert [w.name for w in windows] == ['world', 'map']
    assert windows[0].cmd == ('tools/dev_window.sh', '--watch')
    assert windows[1].cmd[0] == 'tools/ros.sh' and '--local' not in windows[1].cmd
    assert windows[1].cmd[-1] == 'hardware:=false'    # the car's own map runs hw_panel
    assert notes == [screens.HUD_ON_CAR]


def test_only_opens_the_windows_asked_for():
    windows, _ = screens.plan(screens.SIM, ['map'])
    assert [w.name for w in windows] == ['map']
    windows, _ = screens.plan(screens.SIM, ['hud', 'world'])
    assert [w.name for w in windows] == ['world', 'hud']
    windows, notes = screens.plan(screens.CAR, ['hud'])
    assert windows == [] and notes == [screens.HUD_ON_CAR]
    with pytest.raises(SystemExit):
        screens.parse(['--only', 'rviz'])
    with pytest.raises(SystemExit):
        screens.parse(['--sim', '--car'])


def test_which_car_from_system_source():
    assert screens.which_car(SIM_HEARD, None)[0] == screens.SIM
    assert screens.which_car(NOTHING, REAL)[0] == screens.CAR
    assert screens.which_car(REAL, None)[0] == screens.CAR
    car, why = screens.which_car(NOTHING, NOTHING)
    assert car is None and 'pixi run sim' in why
    car, why = screens.which_car(NOTHING, SIM_HEARD)
    assert car is None and 'another computer' in why
    assert screens.which_car(None, None)[0] is None


def test_sim_is_refused_beside_a_real_car():
    assert screens.check_sim(REAL).startswith('REFUSED')
    assert screens.check_sim(SIM_HEARD) is None
    assert screens.check_sim(NOTHING) is None


def test_print_starts_nothing_and_says_each_command():
    done = subprocess.run([sys.executable, 'tools/screens.py', '--print', '--car'], cwd=ROOT,
                          capture_output=True, text=True, timeout=30)
    assert done.returncode == 0, done.stderr
    assert 'dev_window.sh --watch' in done.stdout
    assert 'hardware:=false' in done.stdout
    assert 'HUD is not opened' in done.stdout


def test_one_window_closing_stops_every_window_group(tmp_path, monkeypatch):
    """Two stand-in windows: one closing stops the other, and nothing is left."""
    marker = tmp_path / 'started'
    fake = [screens.Window('a', 'window a', ('sleep', '30')),
            screens.Window('b', 'window b', ('sh', '-c', f'touch {marker}; sleep 1'))]
    monkeypatch.setattr(screens, 'plan', lambda car, only=(): (fake, []))
    began = time.monotonic()
    monkeypatch.setattr(screens, 'heard', lambda where: SIM_HEARD)
    assert screens.main(['--sim']) == 0
    assert marker.exists()
    assert time.monotonic() - began < 25
    assert signal.getsignal(signal.SIGINT) is signal.default_int_handler   # given back

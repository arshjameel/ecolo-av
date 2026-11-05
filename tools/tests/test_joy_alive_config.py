"""
TODO 74a: joy_alive watches the same deadman and throttle axis teleop_twist_joy drives by.

If someone changes the joystick's buttons or axes in teleop_twist_joy.yaml (car day, TODO
74d) and not in joy_alive.yaml, the frozen rule would watch the wrong stick: this fails.
And teleop_twist_joy must read joy_alive's output, never joy_node's.
"""

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'ros' / 'drivers' / 'aav_teleop' / 'config'
sys.path.insert(0, str(ROOT / 'ros' / 'safety' / 'aav_safety'))
from aav_safety.joy_alive import JoyAlive  # noqa: E402
import yaml  # noqa: E402, I100


def params(name, node):
    return yaml.safe_load((CONFIG / name).read_text())[f'/**/{node}']['ros__parameters']


def test_the_same_deadman_and_throttle():
    teleop = params('teleop_twist_joy.yaml', 'teleop_twist_joy_node')
    alive = params('joy_alive.yaml', 'joy_alive')
    assert teleop['require_enable_button']
    assert alive['enable_button'] == teleop['enable_button']
    assert teleop['axis_linear']['x'] in alive['motion_axes']


def test_its_settings_are_accepted():
    alive = params('joy_alive.yaml', 'joy_alive')
    JoyAlive(**{k: v for k, v in alive.items() if k != 'rate_hz'})
    # Silence must trip before twist_mux's own joystick timeout would matter much, and
    # well after joy_node's 20 Hz repeat.
    joy = params('joy.yaml', 'joy_node')
    assert joy['autorepeat_rate'] > 0, 'without the repeat, a still hand reads as silence'
    assert 3.0 / joy['autorepeat_rate'] <= alive['silent_after'] <= 1.0


def test_teleop_reads_only_what_joy_alive_passes():
    launch = (ROOT / 'ros' / 'drivers' / 'aav_teleop' / 'launch' / 'teleop.launch.py').read_text()
    assert re.search(r"remappings=\[\('joy', 'joy_checked'\)\]", launch)
    assert "executable='joy_alive'" in launch

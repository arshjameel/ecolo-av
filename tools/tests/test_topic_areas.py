"""Every topic in docs/interfaces.yaml starts with its area (docs/naming.md, Decided 15 Oct)."""

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]

# /<area>/<device or node>/<thing>: the layers of the picture, as in docs/naming.md.
AREAS = ('sensors', 'vehicle', 'can', 'localization', 'perception', 'drivers', 'control',
         'ego', 'safety', 'hud', 'system')
# ROS's own global names, which every tool expects, and Nav2's, which stay for now.
GLOBALS = {'/tf', '/tf_static', '/clock', '/robot_description', '/joint_states',
           '/diagnostics', '/rosout', '/parameter_events', '/map', '/goal_pose', '/plan',
           '/cmd_vel_nav'}


def topics():
    spec = yaml.safe_load((ROOT / 'docs' / 'interfaces.yaml').read_text())
    return list(spec['topics'])


def area_of(name):
    parts = name.split('/')
    return parts[1] if name.startswith('/') and len(parts) > 2 else None


def test_every_topic_has_an_area():
    bad = [n for n in topics() if n not in GLOBALS and area_of(n) not in AREAS]
    assert not bad, f'topics outside an area (docs/naming.md): {bad}'


def test_driver_commands_are_drivers_name_cmd_vel():
    # Each driver's command is /drivers/<name>/cmd_vel, TwistStamped, into twist_mux.
    spec = yaml.safe_load((ROOT / 'docs' / 'interfaces.yaml').read_text())['topics']
    for name, t in spec.items():
        if name.endswith('/cmd_vel') and area_of(name) == 'drivers':
            assert len(name.split('/')) == 4, name
            assert t['type'] == 'geometry_msgs/msg/TwistStamped', name
            assert 'twist_mux' in t['sub'], name


def test_twist_mux_reads_only_driver_topics():
    path = ROOT / 'ros' / 'command_mux' / 'aav_command_mux' / 'config' / 'twist_mux.yaml'
    mux = yaml.safe_load(path.read_text())['twist_mux']['ros__parameters']
    for name, entry in mux['topics'].items():
        assert entry['topic'] == f'/drivers/{name}/cmd_vel', entry['topic']
    contract = set(topics())
    for entry in list(mux['topics'].values()) + list(mux['locks'].values()):
        assert entry['topic'] in contract, entry['topic']


def test_area_of():
    assert area_of('/sensors/gps/fix') == 'sensors'
    assert area_of('/tf') is None
    assert area_of('cmd_vel') is None

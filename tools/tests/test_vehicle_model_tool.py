"""vehicle_model: the tables print, and Gazebo's steering gain is read from the xacro."""

from pathlib import Path
import re
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import vehicle_model as tool  # noqa: E402
    HAVE_SCIPY = True
except ImportError:          # no numpy or scipy here: pixi run test has them
    HAVE_SCIPY = False

pytestmark = pytest.mark.skipif(not HAVE_SCIPY, reason='needs numpy and scipy')


def test_the_tables_print(capsys):
    assert tool.main(['--no-figures', '--set', 'throttle_lag=0.25']) == 0
    out = capsys.readouterr().out
    assert 'with throttle_lag=0.25' in out
    for title in ('Speed loop margins', 'Steering: second order', 'Lateral margins',
                  'Discrete time', "Gazebo's car"):
        assert title in out


def test_gazebo_s_steering_gain():
    # The xacro's steer_p_gain: Gazebo turns each steering joint at gain x error (1/s).
    assert tool.gazebo_steer_gain() == 100.0
    gain, rows = tool.gazebo_rows(tool.vm.VehicleModel())
    assert len(rows) == 1 + len(tool.vm.lateral_speeds())


def test_gazebo_steers_as_vehicle_yaml_says():
    # TODO 52c: Gazebo's own steering lag (1/steer_p_gain, and the adapter's 50 Hz hold)
    # comes after the actuator model's steer_lag. It was 0.11 s (gain 10), more than
    # steer_lag itself. Keep it a small part of steer_lag, so Gazebo's car is the model's.
    actuators = yaml.safe_load(tool.VEHICLE_YAML.read_text())['actuators']
    assert tool.gazebo_extra_lag() <= 0.25 * actuators['steer_lag']


def test_gazebo_s_joints_can_turn_as_fast_as_the_model():
    # Gazebo holds each steering joint to the URDF's velocity limit: at 1.0 rad/s its wheels
    # turned only 0.63, slower than the model's steer_rate (TODO 52c). The inner wheel turns
    # faster than the centre one, and the gain's catch-up on each 20 ms hold faster still.
    urdf = (tool.ROOT / 'ros/bringup/aav_description/urdf/aav.urdf.xacro').read_text()
    limit = re.search(r'<limit [^>]*upper="\$\{1.3 \* max_steer\}"[^>]*velocity="([\d.]+)"',
                      urdf)
    actuators = yaml.safe_load(tool.VEHICLE_YAML.read_text())['actuators']
    assert limit and float(limit.group(1)) >= 4.0 * actuators['steer_rate']


def test_a_wrong_key_is_refused():
    with pytest.raises(KeyError):
        tool.main(['--no-figures', '--set', 'throtle_lag=0.2'])


def test_the_speed_loop_study_prints(capsys):
    assert tool.main(['--speed-loop', '--no-figures']) == 0
    out = capsys.readouterr().out
    assert 'The speed loop against the uncertainty' in out
    rows = [line for line in out.splitlines() if 'K (vehicle.yaml)' in line]
    today = next(r for r in rows if r.startswith('today (kp 0.3'))
    smooth = next(r for r in rows if r.startswith("'smooth'"))
    assert ' 17 % ' in today and ' 0 % ' in smooth


def test_the_speed_loop_study_follows_the_measured_numbers():
    model = tool.vm.VehicleModel().with_measured({'drag': 0.5})
    assert tool.plant_numbers(model, 0.2, 1.0) == {'max_accel': 1.5 * 0.2, 'drag': 0.5}


# --- one key list: every actuators: key is used or ignored by each reader ----------------

# What ego leaves out of vehicle.yaml's actuators: (aav_description vehicle.ego_params takes
# only the two strengths, for its braking and grip limits). Here until vehicle.py keeps its
# own list, as vehicle_model.NOT_LINEAR and actuator_model.IGNORED do.
EGO_PARAMS_IGNORES = {'throttle_deadband', 'throttle_lag', 'throttle_dead_time', 'brake_lag',
                      'brake_dead_time', 'drag', 'steer_rate', 'steer_lag', 'steer_dead_time',
                      'steer_offset', 'steer_deadband', 'open_loop_decel'}


def actuator_keys():
    """Return vehicle.yaml's actuators: keys, the commented-out examples too."""
    block = tool.VEHICLE_YAML.read_text().split('\nactuators:\n', 1)[1]
    keys = set()
    for line in block.splitlines():
        if line and not line.startswith(' '):
            break
        found = re.match(r'^  (?:#\s*)?([a-z_]+):\s', line)
        if found:
            keys.add(found.group(1))
    return keys


def ego_params_uses(keys):
    """Return the actuators: keys vehicle.ego_params reads: each gets its own number."""
    sys.path.insert(0, str(tool.ROOT / 'ros/bringup/aav_description'))
    from aav_description import vehicle
    cfg = yaml.safe_load(tool.VEHICLE_YAML.read_text())
    marks = {k: 1000.0 + i for i, k in enumerate(sorted(keys))}
    cfg['actuators'] = dict(marks)
    seen = set(vehicle.ego_params(cfg).values())
    return {k for k, v in marks.items() if v in seen}


def test_every_actuator_key_is_used_or_ignored_by_each_reader():
    keys = actuator_keys()
    assert {'max_accel', 'steer_lag', 'throttle_dead_time'} <= keys
    readers = {
        'vehicle_model.py (YAML_KEYS, NOT_LINEAR)': (
            {k.split('.')[1] for k in tool.vm.YAML_KEYS if k.startswith('actuators.')},
            {k.split('.')[1] for k in tool.vm.NOT_LINEAR}),
        'actuator_model.py (ActuatorConfig, IGNORED)': (
            {f for f in vars(tool.am.ActuatorConfig()) if f != 'max_steering'},
            set(tool.am.IGNORED)),
        'vehicle.ego_params (EGO_PARAMS_IGNORES)': (ego_params_uses(keys), EGO_PARAMS_IGNORES),
    }
    for name, (used, ignored) in readers.items():
        assert not used & ignored, f'{name}: both used and ignored: {used & ignored}'
        assert keys <= used | ignored, f'{name}: neither uses nor ignores {keys - used - ignored}'
        assert used | ignored <= keys, f'{name}: lists keys vehicle.yaml lacks: ' \
                                       f'{(used | ignored) - keys}'


def test_the_dead_times_reach_the_model_tables(capsys):
    assert tool.main(['--no-figures', '--set', 'steer_dead_time=0.05']) == 0
    assert '(134 ms)' in capsys.readouterr().out      # typical lateral: 84 ms + 50 ms


def test_the_lqi_constants_are_the_ones_in_spine_core(capsys):
    assert tool.main(['--lqi']) == 0
    out = capsys.readouterr().out
    printed = out[out.index('pub const MODEL'):out.index('};') + 2]
    src = (tool.ROOT / 'firmware/spine/crates/spine-core/src/speed_loop.rs').read_text()
    assert printed in src, 'paste `pixi run vehicle-model --lqi` into speed_loop.rs'
    assert 'LQI_MODEL = LqiConfig(' in out


def test_the_speed_loop_study_has_the_lqi(capsys):
    assert tool.main(['--speed-loop', '--no-figures']) == 0
    out = capsys.readouterr().out
    lqi = next(r for r in out.splitlines() if r.startswith("'lqi'"))
    assert ' 0 % ' in lqi and 'reached' in lqi

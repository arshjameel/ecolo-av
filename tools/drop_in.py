"""
Judge whether a stream is a drop-in.

Ben, 5 Nov: "just swap out and test". No ROS here; tools/drop_in_check.py measures and
tools/tests/test_drop_in.py tests this.

A topic's `drop_in:` in docs/interfaces.yaml holds what its stream must be, beyond the name
and type: `min_hz`, `max_hz`, `frame` and `reliable`. Ravensim and the real sensor must
both pass it. Passing is necessary, not enough: it says nothing of a stream's values.
"""

# The nodes that may read a simulator's own topics. Anything else on those topics would stop
# the simulator being a drop-in, or let a test's truth reach the code that drives.
SIM_ONLY_PREFIXES = ('/vehicle/sim/', '/control/sim/')
# Ravensim's own parts: the simulator, its cameras (sim_cameras: drawn where the truth puts
# the car) and the nodes that turn its raw streams into the car's (sim_*).
SIM_READERS = {'sim_adapter', 'sim_gps', 'spine_sim', 'chase_jpeg', 'debug_hud',
               'dev_window', 'hw_panel', 'spine_bridge', 'legacy_uart_bridge', 'foxglove_bridge',
               'sim_cameras', 'simulator', 'sim_imu', 'sim_radar', 'sim_camera_relay',
               'sim_lidar_relay'}


def rules(spec):
    """Return every topic with a `drop_in:` rule: {topic: rule}."""
    return {name: t['drop_in'] for name, t in spec['topics'].items() if t.get('drop_in')}


def judge(name, rule, heard, seconds, frames, reliable_publishers=None):
    """
    Return the problems with one topic: `heard` messages in `seconds`, and the frames seen.

    `seconds` is on the run's own clock, so a run faster than real time doesn't look fast.
    `reliable_publishers`: whether every publisher is reliable; None if it wasn't checked.
    """
    if heard == 0:
        return [f'{name}: nothing heard in {seconds:.0f} s']
    problems = []
    hz = heard / seconds
    if 'min_hz' in rule and hz < rule['min_hz']:
        problems.append(f'{name}: {hz:.1f} Hz, the contract needs at least {rule["min_hz"]}')
    if 'max_hz' in rule and hz > rule['max_hz']:
        problems.append(f'{name}: {hz:.1f} Hz, the contract allows at most {rule["max_hz"]}')
    if rule.get('reliable') and reliable_publishers is False:
        problems.append(f'{name}: a publisher is best effort: reliable readers would hear nothing')
    wanted = rule.get('frame')
    if wanted is not None and set(frames) != {wanted}:
        problems.append(f'{name}: frame {sorted(frames)}, the contract says {wanted}')
    return problems


def sim_leaks(spec):
    """Return the nodes reading a simulator's own topics that shouldn't: must stay empty."""
    leaks = []
    for name, t in spec['topics'].items():
        if name.startswith(SIM_ONLY_PREFIXES):
            leaks += [f'{node} reads {name}' for node in t.get('sub', [])
                      if node not in SIM_READERS]
    return leaks


def named_after_a_simulator(spec):
    """Return the topics the car's stack could come to depend on that name one simulator."""
    return [name for name in spec['topics']
            if name.startswith(('/vehicle/gazebo/', '/control/gazebo/', '/sensors/gazebo/'))]

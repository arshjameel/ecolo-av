#!/usr/bin/env python3
"""
Write io-mux-core's agreement vectors from the Python muxes (aav_io_mux).

The Python: provenance.py, heartbeat.py and muxes.py, the two nodes' work without ROS.
The Rust (crates/io-mux-core, tests/vectors.rs) must give the same answers at every step:
each SourceCheck the sensor mux publishes and what it logs; each state line the output
mux publishes and what it logs; each command passed on or held, with its heartbeat
counter; each clear's answer; the launch-time refusals (an unknown target or source,
screens for the car). A safety lock: compared as text, no tolerance.

Runs (each a fresh node):
  - hand-written stories: the car on its stock drivers, a simulator's announcement on
    the car (latched STOP, clear refused, then cleared once it goes), the 30 s start-up
    grace (WAIT, then a latch), two publishers on a sensor, an announcer not heard yet,
    a quiet sensor mux, a sensor mux checking for the wrong source, NOT SIM / WAIT on the
    sim target, screens, the heartbeat wrapping past 255, every target;
  - random runs: announcements, graph changes, reports, commands, clears and ticks
    in any order, so every branch meets every other.

Writes ros/io_mux/aav_io_mux_rs/crates/io-mux-core/tests/vectors.json. Run it after any
change to those modules (aav_io_mux's test_vectors.py fails until you do):

  pixi run python ros/io_mux/aav_io_mux_rs/gen_vectors.py
"""

import json
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_io_mux')]

from aav_io_mux import muxes, provenance  # noqa: E402
from aav_io_mux.provenance import BENCH, Entry, REAL, REPLAY, SIM  # noqa: E402

OUT = HERE / 'crates' / 'io-mux-core' / 'tests' / 'vectors.json'
SENSOR_TOPICS = [topic for _, topic in provenance.SENSORS]
IMU, GPS, ODOM = '/sensors/imu/data_raw', '/sensors/gps/fix', '/vehicle/odom'


def entries_json(entries):
    return [[e.name, e.topic, e.kind] for e in entries]


def check_json(check):
    return {'source': check.source, 'ok': check.ok, 'problems': list(check.problems),
            'doubts': list(check.doubts),
            'sensors': [[s.name, s.topic, s.kind] for s in check.sensors]}


def published_json(published):
    if published is None:
        return None
    line, log = published
    return {'line': line, 'log': None if log is None else list(log)}


class SensorRun:
    """One sensor mux, its steps and answers."""

    def __init__(self, source, start=100.0, startup=2.0, hold=5.0, grace=30.0):
        self.params = {'source': source, 'start': start, 'startup': startup, 'hold': hold,
                       'grace': grace}
        self.steps = []
        try:
            self.mux = muxes.SensorMuxLogic(source, start, startup=startup, hold=hold,
                                            grace=grace)
            self.error = None
        except ValueError as e:
            self.mux, self.error = None, str(e)

    def hear(self, now, announcer, entries):
        self.mux.hear(announcer, entries, now)
        self.steps.append({'t': now, 'do': 'hear', 'announcer': announcer,
                           'entries': entries_json(entries)})

    def tick(self, now, alive, publishers):
        check, log = self.mux.tick(now, set(alive), publishers)
        self.steps.append({'t': now, 'do': 'tick', 'alive': sorted(alive),
                           'publishers': {t: list(p) for t, p in publishers.items()},
                           'check': None if check is None else check_json(check),
                           'log': None if log is None else list(log)})

    def json(self):
        return {'node': 'sensor_mux', 'params': self.params, 'error': self.error,
                'steps': self.steps}


class OutputRun:
    """One output mux, its steps and answers."""

    def __init__(self, target, settle=2.0, stale_after=3.0, screens=''):
        self.params = {'target': target, 'settle': settle, 'stale_after': stale_after,
                       'screens': screens}
        self.steps = []
        try:
            self.mux = muxes.OutputMuxLogic(target, settle=settle, stale_after=stale_after,
                                            screens=muxes.screen_names(screens))
            self.error = None
        except ValueError as e:
            self.mux, self.error = None, str(e)

    def check(self, now, alive, source, ok, problems=(), doubts=()):
        out = self.mux.check(now, set(alive), source, ok, list(problems), list(doubts))
        self.steps.append({'t': now, 'do': 'check', 'alive': sorted(alive), 'source': source,
                           'ok': ok, 'problems': list(problems), 'doubts': list(doubts),
                           'published': published_json(out)})

    def source(self, now, alive, announcer, entries):
        out = self.mux.source(now, set(alive), announcer, entries)
        self.steps.append({'t': now, 'do': 'source', 'alive': sorted(alive),
                           'announcer': announcer, 'entries': entries_json(entries),
                           'published': published_json(out)})

    def timer(self, now, alive):
        out = self.mux.timer(now, set(alive))
        self.steps.append({'t': now, 'do': 'timer', 'alive': sorted(alive),
                           'published': published_json(out)})

    def command(self, now):
        self.steps.append({'t': now, 'do': 'command', 'counter': self.mux.command(now)})

    def clear(self, now, alive):
        success, message, out = self.mux.clear(now, set(alive))
        self.steps.append({'t': now, 'do': 'clear', 'alive': sorted(alive),
                           'success': success, 'message': message,
                           'published': published_json(out)})

    def beat(self, now, screen):
        self.mux.beat(screen, now)
        self.steps.append({'t': now, 'do': 'beat', 'screen': screen})

    def json(self):
        return {'node': 'output_mux', 'params': self.params, 'error': self.error,
                'steps': self.steps}


def sim_entries(*topics, kind=SIM, announcer='sim_adapter'):
    return [Entry(announcer, t.split('/')[-2], t, kind) for t in topics]


LEGACY = [Entry('legacy_uart_bridge', 'actuators', '', REAL)]


def stories():
    runs = []

    # --- the sensor mux ---
    # The car on its stock drivers: unannounced publishers are REAL; quiet at start-up.
    r = SensorRun('real')
    car = {IMU: ['imu_yahboom'], GPS: ['ublox_gps_node']}
    r.tick(100.5, [], car)
    r.hear(100.6, 'legacy_uart_bridge', LEGACY)
    r.tick(102.0, ['legacy_uart_bridge'], car)
    r.tick(102.5, ['legacy_uart_bridge'], car)
    # A simulator joins on the car: a problem at once.
    r.hear(103.0, 'sim_adapter', sim_entries(IMU, GPS))
    r.tick(103.0, ['legacy_uart_bridge'], {IMU: ['imu_yahboom', 'sim_imu'], GPS: ['sim_gps']})
    r.tick(103.5, ['legacy_uart_bridge', 'sim_adapter'],
           {IMU: ['imu_yahboom', 'sim_imu'], GPS: ['sim_gps']})
    # Two publishers: a doubt that lasts `hold` s becomes a problem.
    r.tick(110.0, ['legacy_uart_bridge'], {IMU: ['imu_yahboom', 'sim_imu']})
    r.tick(113.0, ['legacy_uart_bridge'], {IMU: ['imu_yahboom', 'sim_imu']})
    r.tick(115.0, ['legacy_uart_bridge'], {IMU: ['imu_yahboom', 'sim_imu']})   # hold, exactly
    r.tick(116.0, ['legacy_uart_bridge'], {IMU: ['imu_yahboom', 'sim_imu']})
    r.tick(116.5, ['legacy_uart_bridge'], car)
    r.tick(117.0, ['legacy_uart_bridge'], {})
    # An unknown source number is never REAL.
    r.hear(118.0, 'new_node', [Entry('new_node', 'imu', IMU, 7)])
    r.tick(118.0, ['legacy_uart_bridge', 'new_node'], car)
    # A fresh doubt, judged at exactly `hold` s.
    r.tick(120.0, ['legacy_uart_bridge', 'new_node'], {GPS: ['ublox_gps_node', 'sim_gps']})
    r.tick(125.0, ['legacy_uart_bridge', 'new_node'], {GPS: ['ublox_gps_node', 'sim_gps']})
    runs.append(r)

    # The simulator's start-up on a busy computer: sensors 10 s before their announcer.
    r = SensorRun('sim')
    sim = {IMU: ['sim_imu'], GPS: ['sim_gps'], ODOM: ['sim_adapter']}
    for i in range(0, 80):
        now = 102.0 + 0.5 * i
        if i == 20:
            r.hear(now, 'sim_adapter', sim_entries(IMU, GPS, ODOM))
        alive = ['sim_adapter'] if i >= 10 else []
        r.tick(now, alive, sim if i < 70 else {**sim, '/sensors/radar/points': ['radar']})
    runs.append(r)

    # Never announced: after the 30 s grace and the hold, it latches (on the output mux).
    r = SensorRun('sim', grace=30.0)
    for i in range(0, 90):
        r.tick(102.0 + 0.5 * i, [], {GPS: ['sim_gps']})
    runs.append(r)

    # An announcer in the graph, not heard yet: a doubt, pending, given the grace.
    r = SensorRun('bench', grace=3.0, hold=1.0, startup=0.0)
    for i in range(12):
        r.tick(100.0 + 0.5 * i, ['spine_sim'], {ODOM: ['spine_sim']})
    r.hear(106.0, 'spine_sim', [Entry('spine_sim', 'odometry', ODOM, BENCH)])
    r.tick(106.5, ['spine_sim'], {ODOM: ['spine_sim']})
    # Heard, then seen, then gone: forgotten; heard but never seen: kept for 10 s.
    r.tick(107.0, [], {})
    r.hear(108.0, 'ghost', [Entry('ghost', 'imu', IMU, SIM)])
    r.tick(108.5, [], {})
    r.tick(118.0, [], {})   # the announcers' grace, exactly
    r.tick(118.5, [], {})
    runs.append(r)

    r = SensorRun('replay', startup=1.0)
    r.hear(100.0, 'bag', [Entry('bag', 'imu', IMU, REPLAY), Entry('bag', 'gps', GPS, REAL)])
    r.tick(101.0, ['bag'], {IMU: ['bag'], GPS: ['bag']})
    r.tick(101.5, ['bag'], {IMU: ['bag']})
    runs.append(r)

    runs.append(SensorRun('gazebo'))
    runs.append(SensorRun(''))

    # --- the output mux ---
    # The car: wait for the sensor mux, settle, open; the heartbeat counts and wraps.
    r = OutputRun('car')
    r.timer(0.5, [])
    r.command(0.6)
    r.check(1.0, ['legacy_uart_bridge'], REAL, True)
    r.source(1.1, ['legacy_uart_bridge'], 'legacy_uart_bridge', LEGACY)
    r.command(2.0)
    r.timer(2.5, ['legacy_uart_bridge'])
    r.command(3.0)
    for i in range(300):
        r.command(3.0 + 0.02 * (i + 1))
        if i % 25 == 0:
            r.check(3.0 + 0.02 * (i + 1) + 0.001, ['legacy_uart_bridge'], REAL, True)
    # A quiet sensor mux: WAIT, never a latch; back, and open after settle.
    r.timer(12.0, ['legacy_uart_bridge'])
    r.command(12.1)
    r.timer(12.5, ['legacy_uart_bridge'])
    r.check(13.0, ['legacy_uart_bridge'], REAL, True)
    r.command(14.0)
    r.command(15.0)
    # Unsure: waits, with its doubts.
    r.check(15.5, ['legacy_uart_bridge'], REAL, False, doubts=['imu: 2 publishers'])
    r.command(15.6)
    r.check(16.0, ['legacy_uart_bridge'], REAL, True)
    r.command(18.25)
    # A simulator announces on the car: STOP, latched; clear refused while it lives.
    r.source(20.0, ['legacy_uart_bridge', 'sim_adapter'], 'sim_adapter',
             sim_entries(IMU, GPS))
    r.command(20.1)
    r.clear(21.0, ['legacy_uart_bridge', 'sim_adapter'])
    r.timer(21.5, ['legacy_uart_bridge', 'sim_adapter'])
    # It goes: still latched until a person clears; then settle.
    r.timer(22.0, ['legacy_uart_bridge'])
    r.command(22.1)
    r.clear(22.5, ['legacy_uart_bridge'])
    r.command(23.0)
    r.check(24.0, ['legacy_uart_bridge'], REAL, True)
    r.command(24.5)
    r.clear(25.0, ['legacy_uart_bridge'])
    # The sensor mux's problems latch, with its reasons; "says no" with none.
    r.check(26.0, ['legacy_uart_bridge'], REAL, False, problems=['gps is SIM (sim_gps)'])
    r.check(26.5, ['legacy_uart_bridge'], REAL, True)
    r.clear(27.0, ['legacy_uart_bridge'])
    r.check(27.5, ['legacy_uart_bridge'], REAL, False)
    r.clear(28.0, ['legacy_uart_bridge'])
    # A sensor mux checking for another source.
    r.check(28.5, ['legacy_uart_bridge'], SIM, True)
    r.clear(29.0, ['legacy_uart_bridge'])
    r.check(29.5, ['legacy_uart_bridge'], REAL, True)
    r.clear(30.0, ['legacy_uart_bridge'])
    # An announcer not heard yet: waits; a clear while waiting says so.
    r.timer(30.5, ['legacy_uart_bridge', 'imu_driver'])
    r.clear(31.0, ['legacy_uart_bridge', 'imu_driver'])
    r.source(31.2, ['legacy_uart_bridge', 'imu_driver'], 'imu_driver',
             [Entry('imu_driver', 'imu', IMU, REAL)])
    r.check(33.25, ['legacy_uart_bridge', 'imu_driver'], REAL, True)
    r.command(33.5)
    # The age, printed to 0.1 s, on its exact halves.
    for age in (3.05, 3.25, 3.35, 3.45, 3.75, 4.25, 10.65):
        r.timer(33.25 + age, ['legacy_uart_bridge', 'imu_driver'])
    runs.append(r)

    # The simulator: NOT SIM latches; an unknown number too.
    r = OutputRun('sim', settle=0.5, stale_after=1.5)
    r.check(0.0, ['sim_adapter'], SIM, True)
    r.source(0.1, ['sim_adapter'], 'sim_adapter', sim_entries(IMU))
    r.command(0.7)
    r.source(1.0, ['sim_adapter', 'spine_bridge'], 'spine_bridge',
             [Entry('spine_bridge', 'odometry', ODOM, REAL)])
    r.command(1.1)
    r.clear(2.0, ['sim_adapter'])
    r.check(2.1, ['sim_adapter'], SIM, True)
    r.command(2.7)
    r.source(3.0, ['sim_adapter', 'x'], 'x', [Entry('x', 'imu', IMU, 9)])
    r.clear(3.5, ['sim_adapter'])
    runs.append(r)

    # Screens, a test-only rule: wait for each, a quiet one waits again.
    r = OutputRun('sim', settle=1.0, screens=' dev_window, ,hw_panel')
    r.check(0.0, ['sim_adapter'], SIM, True)
    r.timer(0.5, ['sim_adapter'])
    r.beat(0.6, 'dev_window')
    r.timer(1.0, ['sim_adapter'])
    r.beat(1.1, 'hw_panel')
    r.check(1.2, ['sim_adapter'], SIM, True)
    r.command(2.0)
    r.beat(2.15, 'dev_window')
    r.beat(2.15, 'hw_panel')
    r.check(2.2, ['sim_adapter'], SIM, True)
    r.command(2.3)
    r.timer(3.3, ['sim_adapter'])
    r.command(3.4)
    runs.append(r)

    # Each limit exactly (times a computer holds exactly): settle, stale_after, the
    # screens' timeout.
    r = OutputRun('sim', settle=1.0, stale_after=1.5, screens='hud')
    r.beat(0.0, 'hud')
    r.check(0.0, [], SIM, True)
    r.command(0.75)
    r.command(1.0)
    r.timer(1.5, [])
    r.command(1.5)
    r.timer(1.75, [])
    r.beat(1.75, 'hud')
    r.check(2.0, [], SIM, True)
    r.command(2.75)
    r.timer(3.0, [])
    r.command(3.0)
    r.timer(3.5, [])
    runs.append(r)

    runs.append(OutputRun('car', screens='dev_window'))
    runs.append(OutputRun('gazebo'))
    runs.append(OutputRun(''))
    for target, kind in (('turtle', SIM), ('carla', SIM), ('bench', BENCH)):
        r = OutputRun(target, settle=0.0)
        r.timer(0.0, [])
        r.check(0.5, [], kind, True)
        r.command(0.5)
        r.check(1.0, [], REAL, True)
        r.command(1.0)
        runs.append(r)
    return runs


ANNOUNCERS = ['sim_adapter', 'legacy_uart_bridge', 'spine_bridge', 'imu_driver', 'bag']
NAMES = {IMU: 'imu', GPS: 'gps', ODOM: 'odometry', '': 'actuators',
         '/sensors/lidar/points': 'lidar'}


def random_entries(rng, announcer, expected):
    out = []
    for topic in rng.sample(list(NAMES), rng.randint(0, 3)):
        kind = expected if rng.random() < 0.8 else rng.choice([REAL, SIM, BENCH, REPLAY, 5])
        out.append(Entry(announcer, NAMES[topic], topic, kind))
    return out


def random_sensor_run(seed):
    rng = random.Random(seed)
    source = rng.choice(['real', 'sim', 'bench', 'replay'])
    expected = provenance.SOURCES[source]
    r = SensorRun(source, start=50.0, startup=rng.choice([0.0, 2.0]),
                  hold=rng.choice([1.0, 5.0]), grace=rng.choice([0.0, 10.0, 30.0]))
    now, alive, publishers = 50.0, set(), {}
    for _ in range(80):
        now = round(now + rng.choice([0.1, 0.5, 0.5, 1.0, 3.0]) + rng.random() * 0.01, 6)
        roll = rng.random()
        if roll < 0.15:
            alive ^= {rng.choice(ANNOUNCERS)}
        elif roll < 0.3:
            a = rng.choice(ANNOUNCERS)
            r.hear(now, a, random_entries(rng, a, expected))
        elif roll < 0.45:
            # One sensor topic's publishers change: none, one, or two.
            topic = rng.choice(SENSOR_TOPICS[:4] + [ODOM])
            publishers[topic] = rng.sample(ANNOUNCERS + ['driver_a', 'driver_b'],
                                           rng.choice([0, 1, 1, 2]))
        else:
            r.tick(now, alive, {t: p for t, p in publishers.items() if p})
    return r


def random_output_run(seed):
    rng = random.Random(seed)
    target = rng.choice(list(provenance.TARGETS))
    expected = provenance.TARGETS[target]
    screens = '' if expected == REAL or rng.random() < 0.6 else 'hud,panel'
    r = OutputRun(target, settle=rng.choice([0.0, 0.5, 2.0]),
                  stale_after=rng.choice([1.0, 3.0]), screens=screens)
    now, alive = 0.0, set()
    for _ in range(120):
        now = round(now + rng.choice([0.02, 0.1, 0.5, 0.5, 1.0, 4.0]) + rng.random() * 0.01, 6)
        roll = rng.random()
        if roll < 0.1:
            alive ^= {rng.choice(ANNOUNCERS)}
        elif roll < 0.2:
            a = rng.choice(ANNOUNCERS)
            r.source(now, alive, a, random_entries(rng, a, expected))
        elif roll < 0.4:
            source = expected if rng.random() < 0.85 else rng.choice([REAL, SIM, BENCH, 6])
            ok = rng.random() < 0.7
            problems = [] if ok or rng.random() < 0.5 else [f'p{rng.randint(0, 3)}']
            doubts = [] if ok or rng.random() < 0.5 else [f'd{rng.randint(0, 3)}']
            r.check(now, alive, source, ok, problems, doubts)
        elif roll < 0.6:
            r.command(now)
        elif roll < 0.67:
            r.clear(now, alive)
        elif roll < 0.75 and screens:
            r.beat(now, rng.choice(['hud', 'panel']))
        else:
            r.timer(now, alive)
    return r


def shorten(run):
    """
    Write a check or a published state equal to the last one in its run as 'same'.

    The file stays small enough for git; the Rust test keeps the last one and compares.
    """
    last = {}
    for step in run['steps']:
        for key in ('check', 'published'):
            value = step.get(key)
            if value is None:
                continue
            if value == last.get(key):
                step[key] = 'same'
            last[key] = value
    return run


def build():
    """Return every run, as written to vectors.json."""
    runs = stories()
    runs += [random_sensor_run(seed) for seed in range(30)]
    runs += [random_output_run(seed) for seed in range(40)]
    return {'sensors': [list(s) for s in provenance.SENSORS],
            'runs': [shorten(r.json()) for r in runs]}


def main():
    OUT.write_text(json.dumps(build(), indent=None, separators=(',', ':')) + '\n')
    runs = json.loads(OUT.read_text())['runs']
    print(f'{OUT}: {len(runs)} runs, {sum(len(r["steps"]) for r in runs)} steps')


if __name__ == '__main__':
    main()

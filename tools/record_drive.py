#!/usr/bin/env python3
"""
Record a mapping drive: every raw sensor at full rate, checked before, summarised after.

The testing plan's phase C (TODO 249): one hand-driven drive records everything the later
phases need: the NDT map and SLAM, AMCL and a learned localizer, labelling, the transfer
functions.

    pixi run record-drive                      check, then record until Ctrl-C
    pixi run record-drive --name lot --note "the lot, clockwise, dry"
    pixi run record-drive --duration 600       stop by itself after 10 min
    pixi run record-drive --check              only the checks: record nothing
    pixi run record-drive --summary bags/<session>   the summary again
    pixi run record-drive --sim                Ravensim's test lot on this computer, a short
                                               drive, recorded, summarised, then the NDT map
                                               built from it (about 5 min)
    pixi run record-drive --sim --loop         ... the whole mapping route round the lot and
                                               back to the start: a loop for SLAM to close
                                               (TODO 250; about 8 min)
    pixi run record-drive --sim --loop --sim-box 22 -6.5
                                               ... with a parked car (a 4.5 x 1.8 x 1.5 m
                                               box) at x 22, y -6.5 for this drive only:
                                               test data for map-area's passes (TODO 259)
    tools/record.sh --mapping ...              the same as pixi run record-drive

One folder a drive, bags/<date>_<name>/:

| File | What |
|---|---|
| `bag/` | MCAP, zstd (rosbag2's `zstd_fast` preset), a new file every 2 GB or 5 min |
| `drive.yaml` | when, the git commit, the computer, the person's note, the checks, the clocks |
| `vehicle.yaml`, `machines/` | the car's measurements and machine files, as they were |
| `summary.md` | per topic: count, rate, gaps, size; good for which job |
| `record.log` | the recorder's own output |

Before (refuses only on a full disk or no topics at all, else warns and asks): the free
disk against the expected GB an hour, each topic there at its rate (the preflight's
SENSORS rates), each sensor's stamps against this computer's clock, the clocks
(`chronyc tracking` here and on each car computer reachable by ssh), and /tf_static's
sensor frames. A camera with no JPEG topic gets image_transport's republisher for the
drive (raw in, JPEG out), so the bag stays small.

Subscribes: the topics of recording.STREAMS (raw bytes, counted; /tf_static decoded for
its frames). Publishes nothing. Runs: `ros2 bag record`; `image_transport republish` per
camera without JPEG; `chronyc tracking` (here, and over ssh with BatchMode: keys only, never
a password). With --sim: Ravensim (sim.launch.py world:=test_lot) in its own process group
on a private ROS domain, Nav2-style goals to /navigate_to_pose, and tools/ndt_map.py build;
with --sim-box, calls /vehicle/sim/spawn_entity (simulation_interfaces/SpawnEntity).

The maths, no ROS: tools/recording.py (tools/tests/test_recording.py).
"""

import argparse
import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros' / 'drivers' / 'aav_actuator_tests'))
import recording as rec  # noqa: E402, I100

VEHICLE = ROOT / 'ros' / 'bringup' / 'aav_description' / 'config' / 'vehicle.yaml'
MACHINES = ROOT / 'ros' / 'bringup' / 'aav_bringup' / 'config' / 'machines'
SPLIT_BYTES = 2_000_000_000
SPLIT_SECONDS = 300
# The car's computers (docs/car-day.md, "The computers"): their clocks over ssh.
CAR_COMPUTERS = (('orin', '192.168.1.140'), ('motor_pi', '192.168.1.120'),
                 ('localization', '192.168.1.113'), ('dash', '192.168.1.196'))
# Ravensim's short drive: two legs of tools/ndt_map.py's mapping route.
SIM_ROUTE = (('the south side', 44.0, -4.0, 0.0), ('the east lane', 45.0, 42.0, 90.0))
# --loop: the whole route, back home (ndt_map.ROUTE), so a SLAM run has a loop to close.
SIM_LOOP = (SIM_ROUTE + (('the north side', 5.0, 44.0, 180.0),
                         ('the west side', -4.0, 12.0, -90.0), ('home', 0.0, 0.0, 0.0)))
SIM_POSES = '/localization/odom/global'
ME = 'record_drive_check'


def with_preflight_rates(items):
    """Return the streams with the preflight's documented rates (car and stand-in)."""
    try:
        from aav_actuator_tests import preflight
    except ImportError:
        return items
    known = {s.topic: s for s in preflight.SENSORS}
    out = []
    for s in items:
        p = known.get(s.topic)
        if p is None and s.topic.startswith('/sensors/camera/front_left/image_raw'):
            p = known.get('/sensors/camera/front_left/image_raw')
        out.append(rec.Stream(s.topic, s.group, p.car_rate, s.size, p.standin_rate, s.must,
                              s.sim_only, s.car_only) if p else s)
    return out


# --- the computer: disk, clocks, git ---------------------------------------------------

def chronyc(host=None):
    """Return (parsed chronyc tracking, why not) here, or on a host over ssh."""
    cmd = ['chronyc', 'tracking']
    if host:
        cmd = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3', host] + cmd
    elif shutil.which('chronyc') is None:
        return None, 'no chronyc on this computer (a Mac? its clock comes from Apple)'
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired) as error:
        return None, f'{error.__class__.__name__}'
    parsed = rec.parse_chronyc(done.stdout)
    if parsed is None:
        why = (done.stderr.strip().splitlines() or ['no answer'])[-1]
        return None, why[:100]
    return parsed, ''


def clock_checks(sim, hosts):
    """Return the clocks' lines and a record for drive.yaml."""
    if sim:
        return [rec.Check('OK', 'clocks', 'Ravensim: one computer, one clock')], {}
    lines, record = [], {}
    here = socket.gethostname()
    tracking, why = chronyc()
    lines.append(rec.clock_line(f'{here} (here)', tracking, why))
    record[here] = tracking or why
    for name, host in hosts:
        tracking, why = chronyc(host)
        lines.append(rec.clock_line(f'{name} ({host})', tracking, why))
        record[name] = tracking or why
    return lines, record


def git(*args):
    try:
        return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True,
                              timeout=10).stdout.strip()
    except OSError:
        return ''


def describe_computer():
    commit = git('rev-parse', 'HEAD') or 'unknown (no git here)'
    dirty = bool(git('status', '--porcelain', '--untracked-files=no'))
    return {'git_commit': commit, 'git_dirty': dirty, 'host': socket.gethostname(),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'rmw': os.environ.get('RMW_IMPLEMENTATION', ''),
            'discovery': os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '')}


# --- the live check (ROS) --------------------------------------------------------------

class Sampler:
    """Count each wanted topic's messages and bytes; stamp ages; /tf_static's frames."""

    def __init__(self, node, wanted):
        self.node = node
        self.wanted = set(wanted)
        self.subs = {}
        self.count = {}
        self.bytes = {}
        self.lags = {}
        self.frames = set()

    def discover(self):
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from rosidl_runtime_py.utilities import get_message
        for topic, types in self.node.get_topic_names_and_types():
            if topic not in self.wanted or topic in self.subs:
                continue
            try:
                kind = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            infos = self.node.get_publishers_info_by_topic(topic)
            latched = any(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                          for i in infos)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100 if latched else 50,
                             reliability=(ReliabilityPolicy.RELIABLE if latched
                                          else ReliabilityPolicy.BEST_EFFORT),
                             durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                                         else DurabilityPolicy.VOLATILE))
            fields = list(kind.get_fields_and_field_types().items())
            stamped = bool(fields) and fields[0] == ('header', 'std_msgs/Header')
            self.count.setdefault(topic, 0)
            self.bytes.setdefault(topic, 0)
            self.subs[topic] = self.node.create_subscription(
                kind, topic, self.callback(topic, stamped), qos, raw=True)

    def callback(self, topic, stamped):
        def heard(data):
            self.count[topic] += 1
            self.bytes[topic] += len(data)
            if stamped and len(data) >= 12 and data[1] == 1:   # CDR, little-endian
                sec, nsec = struct.unpack_from('<iI', data, 4)
                now = self.node.get_clock().now().nanoseconds * 1e-9
                if sec > 0:
                    self.lags.setdefault(topic, []).append(now - (sec + nsec * 1e-9))
            if topic == '/tf_static':
                from rclpy.serialization import deserialize_message
                from tf2_msgs.msg import TFMessage
                for t in deserialize_message(data, TFMessage).transforms:
                    self.frames.add(t.child_frame_id)
        return heard

    def restart(self):
        for topic in self.count:
            if topic != '/tf_static':
                self.count[topic] = 0
                self.bytes[topic] = 0
        self.lags = {}


def spin(node, seconds, sampler=None):
    import rclpy
    end = time.monotonic() + seconds
    last = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if sampler and time.monotonic() - last > 0.5:
            sampler.discover()
            last = time.monotonic()


def republishers(node, items, raw_cameras):
    """Start image_transport's republisher for each camera with pictures but no JPEG."""
    if raw_cameras:
        return []
    names = {t for t, _ in node.get_topic_names_and_types()}
    started = []
    for s in items:
        if not s.topic.endswith('/image_raw/compressed') or s.topic in names:
            continue
        raw = s.topic[:-len('/compressed')]
        if raw not in names:
            continue
        cam = raw.split('/')[3]
        started.append(subprocess.Popen(
            ['ros2', 'run', 'image_transport', 'republish', '--ros-args',
             '-r', f'__node:=record_jpeg_{cam}', '-p', 'in_transport:=raw',
             '-p', 'out_transport:=compressed', '-r', f'in:={raw}',
             '-r', f'out/compressed:={s.topic}'],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            start_new_session=True))
        print(f'   {raw}: no JPEG topic; republishing it as {s.topic} for the drive')
    return started


def live_checks(items, sim, seconds, raw_cameras, out_parent):
    """Sample the graph; return (checks, measured bytes/s, frames, republishers)."""
    import rclpy
    from rclpy.parameter import Parameter
    node = rclpy.create_node(ME, parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, sim)])
    try:
        spin(node, 2.0)
        helpers = republishers(node, items, raw_cameras)
        sampler = Sampler(node, [s.topic for s in items])
        spin(node, 4.0, sampler)            # find the topics, connect
        sampler.restart()
        start = time.monotonic()
        spin(node, seconds, sampler)
        took = time.monotonic() - start     # rates per real second
        checks = []
        if not sampler.subs:
            checks.append(rec.Check('STOP', 'topics', "none of the drive's topics is there: "
                                    'is the stack running, on this ROS_DOMAIN_ID?'))
        for s in items:
            checks.append(rec.rate_line(s, sampler.count.get(s.topic, 0), took, sim))
        for topic in sorted(sampler.lags):
            line = rec.stamp_lag_line(topic, sampler.lags[topic])
            if line and topic.startswith(('/sensors/', '/vehicle/')):
                checks.append(line)
        lost = rec.missing_frames(sorted(sampler.frames))
        checks.append(rec.Check('WARN' if lost else 'OK', '/tf_static frames',
                                'missing ' + ', '.join(lost) if lost else
                                f"{len(sampler.frames)} frames, every sensor's"))
        measured = sum(sampler.bytes.values()) / took if took > 0 else 0.0
        return checks, measured, sorted(sampler.frames), helpers
    finally:
        node.destroy_node()


# --- record ----------------------------------------------------------------------------

def record_command(bag, items, sim, split_bytes, split_seconds):
    cmd = ['ros2', 'bag', 'record', '-o', str(bag), '-s', 'mcap',
           '--storage-preset-profile', 'zstd_fast',
           '--max-bag-size', str(split_bytes), '--max-bag-duration', str(split_seconds),
           '--disable-keyboard-controls', '--topics', *[s.topic for s in items],
           # The u-blox driver's own raw messages, under its node's name, if it runs.
           '--regex', '^/ublox_gps/.*|^/sensors/gps/.*']
    if sim:
        cmd.append('--use-sim-time')
    return cmd


def start_recording(session, items, sim, args):
    bag = session / 'bag'
    log = open(session / 'record.log', 'w')
    proc = subprocess.Popen(record_command(bag, items, sim, int(args.split_gb * 1e9),
                                           args.split_minutes * 60),
                            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, log


def stop_group(proc, grace=30.0):
    """SIGINT a process group, then SIGTERM, SIGKILL; return True if it stopped."""
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return True
        try:
            proc.wait(wait)
            return True
        except subprocess.TimeoutExpired:
            continue
    return False


# --- after: the summary ----------------------------------------------------------------

def read_bag(bag):
    """Return ({topic: (times, sizes)}, frames in /tf_static, seconds) from a bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    seen, frames = {}, set()
    first = last = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        times, sizes = seen.setdefault(topic, ([], []))
        times.append(t)
        sizes.append(len(data))
        first = t if first is None else min(first, t)
        last = t if last is None else max(last, t)
        if topic == '/tf_static':
            frames.update(x.child_frame_id
                          for x in deserialize_message(data, TFMessage).transforms)
    return seen, sorted(frames), (last - first) if first is not None else 0.0


def summarise(session, items, sim):
    bag = session / 'bag'
    seen, frames, duration = read_bag(bag)
    expected = {s.topic: (s.standin_rate if sim else s.rate) or None for s in items
                if s.must or s.topic in seen}
    stats = rec.summarise(seen, expected, duration)
    verdict = rec.verdicts(stats, frames)
    on_disk = sum(f.stat().st_size for f in bag.glob('*') if f.is_file())
    log = (session / 'record.log').read_text() if (session / 'record.log').exists() else ''
    lost = rec.lost_messages(log)
    notes = []
    missing = rec.missing_frames(frames)
    if missing:
        notes.append('/tf_static has no ' + ', '.join(missing))
    page = rec.markdown(stats, verdict, duration, on_disk, lost, notes)
    (session / 'summary.md').write_text(page)
    meta_path = session / 'drive.yaml'
    meta = yaml.safe_load(meta_path.read_text()) if meta_path.exists() else {}
    meta['after'] = {'seconds': round(duration, 1), 'on_disk_bytes': on_disk,
                     'lost_by_recorder': lost, 'good_for': verdict,
                     'flagged': {s.topic: s.flags for s in stats if s.flags}}
    meta_path.write_text(yaml.safe_dump(meta, sort_keys=False))
    return page, verdict


# --- the drive on the car, or in Ravensim ----------------------------------------------

def new_session(name):
    session = ROOT / 'bags' / f'{datetime.datetime.now():%Y-%m-%d_%H%M%S}_{name}'
    session.mkdir(parents=True)
    shutil.copy(VEHICLE, session / 'vehicle.yaml')
    shutil.copytree(MACHINES, session / 'machines')
    return session


def print_checks(checks):
    for c in checks:
        print(f'  {c.status:<5} {c.what:<46} {c.detail}')


def before(items, sim, args, out_parent):
    """Run every check; return (checks, clocks, frames, helpers, refuse)."""
    on_car = with_preflight_rates(rec.streams(False, args.raw_cameras))
    car = rec.bytes_per_second([(s.rate, s.size) for s in on_car])
    print(f'== Expected size on the car: {car * 3600 / rec.GB:.0f} GB an hour '
          '(before zstd), by group:')
    for group, gb in sorted(rec.by_group(on_car).items(), key=lambda kv: -kv[1]):
        print(f'   {group:<14} {gb:6.1f} GB/h')
    print(f'== Listening for {args.check_seconds:.0f} s ...', flush=True)
    checks, measured, frames, helpers = live_checks(items, sim, args.check_seconds,
                                                    args.raw_cameras, out_parent)
    print(f'   measured now: {measured * 3600 / rec.GB:.1f} GB an hour')
    free = shutil.disk_usage(out_parent).free
    checks.insert(0, rec.disk_check(free, measured if sim else max(car, measured)))
    clock_lines, clocks = clock_checks(sim, [] if args.no_ssh else CAR_COMPUTERS)
    checks[1:1] = clock_lines
    print_checks(checks)
    refuse = [c for c in checks if c.status == 'STOP']
    return checks, clocks, frames, helpers, refuse, measured


def write_meta(session, args, checks, clocks, frames, measured, items):
    meta = {'when': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
            'note': args.note, **describe_computer(), 'sim': args.sim,
            'cameras': 'raw' if args.raw_cameras else 'JPEG',
            'split': {'gb': args.split_gb, 'minutes': args.split_minutes},
            'clocks': clocks, 'tf_static_frames': frames,
            'measured_gb_per_hour': round(measured * 3600 / rec.GB, 2),
            'checks': [{'status': c.status, 'what': c.what, 'detail': c.detail}
                       for c in checks],
            'topics': [s.topic for s in items]}
    (session / 'drive.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))


def car_drive(args):
    items = with_preflight_rates(rec.streams(False, args.raw_cameras))
    import rclpy
    rclpy.init()
    helpers = []
    try:
        (ROOT / 'bags').mkdir(exist_ok=True)
        checks, clocks, frames, helpers, refuse, measured = before(
            items, False, args, ROOT / 'bags')
        if args.check:
            return 1 if refuse else 0
        if refuse and not args.force:
            print('REFUSED: ' + '; '.join(c.detail for c in refuse) + ' (--force to record '
                  'anyway)')
            return 1
        warns = [c for c in checks if c.status == 'WARN']
        if warns and not args.yes:
            answer = input(f'{len(warns)} warnings above. Record anyway? [y/N] ')
            if answer.strip().lower() not in ('y', 'yes'):
                return 1
        if args.note is None:
            args.note = input('A note for this drive (where, which way, the weather): ')
        session = new_session(args.name)
        write_meta(session, args, checks, clocks, frames, measured, items)
        proc, log = start_recording(session, items, False, args)
        print(f'== Recording into {session}/bag. Ctrl-C to stop.', flush=True)
        try:
            end = time.monotonic() + args.duration if args.duration else math.inf
            while proc.poll() is None and time.monotonic() < end:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        stop_group(proc)
        log.close()
    finally:
        for h in helpers:
            stop_group(h, 5.0)
        rclpy.try_shutdown()
    page, _ = summarise(session, items, False)
    print(page)
    print(f'Kept in {session}')
    return 0


BOX_SDF = """<?xml version="1.0"?>
<!-- record_drive.py, sim box: a parked car for one drive only (TODO 259's test data). -->
<sdf version="1.9">
  <model name="parked_box">
    <static>true</static>
    <link name="link">
      <pose>0 0 {h2} 0 0 0</pose>
      <visual name="visual"><geometry><box><size>{l} {w} {h}</size></box></geometry></visual>
      <collision name="collision"><geometry><box><size>{l} {w} {h}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
BOX_SIZE = (4.5, 1.8, 1.5)      # m: a car's size, as a box


def spawn_box(session, x, y, yaw_deg=0.0):
    """Put a car-sized box in Ravensim at (x, y) for this drive (simulation_interfaces)."""
    l, w, h = BOX_SIZE
    sdf = session / 'parked_box.sdf'
    sdf.write_text(BOX_SDF.format(l=l, w=w, h=h, h2=h / 2.0))
    yaw = math.radians(yaw_deg)
    request = ('{name: parked_box, uri: "file://%s", initial_pose: {pose: {position: '
               '{x: %f, y: %f, z: 0.0}, orientation: {z: %f, w: %f}}}}'
               % (sdf, x, y, math.sin(yaw / 2), math.cos(yaw / 2)))
    done = subprocess.run(['ros2', 'service', 'call', '/vehicle/sim/spawn_entity',
                           'simulation_interfaces/srv/SpawnEntity', request],
                          capture_output=True, text=True, timeout=60)
    if done.returncode != 0 or 'result=1,' not in done.stdout.replace(' ', ''):
        raise SystemExit(f'the box was not placed: {done.stdout[-400:]} {done.stderr[-400:]}')
    print(f'   a parked car (box {l} x {w} x {h} m) at x {x}, y {y}', flush=True)


def sim_drive(args):
    """Ravensim's test lot on a private domain: check, record a short drive, summarise, map."""
    import controller_compare as cc
    import rclpy
    items = with_preflight_rates(rec.streams(True, args.raw_cameras))
    domain = cc.private(set())
    session = new_session(args.name)
    sim_log = session / 'sim.log'
    print(f'== Ravensim, the test lot, headless, on ROS domain {domain} (this computer only)',
          flush=True)
    with open(sim_log, 'w') as log:
        sim = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py',
                                'world:=test_lot', 'rviz:=false',
                                'lidar:=true', 'reversing:=false', 'stack:=best'],
                               cwd=ROOT, env=dict(os.environ), stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
        proc = rlog = None
        helpers = []
        rclpy.init()
        car = cc.Recorder()
        try:
            up = car.spin_for(cc.UP_TIME, lambda: car.last_truth is not None and car.glob)
            if not (up and car.spin_for(cc.UP_TIME, car.nav.server_is_ready)):
                raise SystemExit(f'Ravensim never came up: see {sim_log}')
            if args.sim_box:
                spawn_box(session, *args.sim_box)
            car.spin_for(cc.SETTLE)
            checks, clocks, frames, helpers, refuse, measured = before(
                items, True, args, session)
            if refuse:
                raise SystemExit('REFUSED: ' + '; '.join(c.detail for c in refuse))
            args.note = args.note or "Ravensim test lot: the recording kit's own test"
            if args.sim_box:
                args.note += f' (a parked box at {args.sim_box[0]}, {args.sim_box[1]})'
            write_meta(session, args, checks, clocks, frames, measured, items)
            proc, rlog = start_recording(session, items, True, args)
            car.spin_for(3.0)
            print(f'== Recording into {session}/bag; driving:', flush=True)
            for name, x, y, heading in (SIM_LOOP if args.loop else SIM_ROUTE):
                leg, notes = cc.drive_leg(car, sim.pid, (x, y, math.radians(heading)), 1.0,
                                          None, None)
                print(f'   {name}: ' + ('reached' if leg['reached'] else 'NOT reached') +
                      ''.join(f'; {n}' for n in notes), flush=True)
        finally:
            car.destroy_node()
            rclpy.try_shutdown()
            if proc is not None:
                stop_group(proc)
                rlog.close()
            for h in helpers:
                stop_group(h, 5.0)
            if not cc.stop(sim):
                print(f'   some of process group {sim.pid} would not stop', flush=True)
            sim.wait()
    page, verdict = summarise(session, items, True)
    print(page)
    print(f'== The NDT map from it, placed by {SIM_POSES} (as a real drive would be):',
          flush=True)
    out = session / 'ndt_map'
    built = subprocess.run([sys.executable, str(ROOT / 'tools' / 'ndt_map.py'), '--out',
                            str(out), 'build', str(session / 'bag'), '--poses', SIM_POSES],
                           cwd=ROOT)
    print(f'Kept in {session}')
    return 0 if built.returncode == 0 and verdict.get('NDT map') == 'good' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--name', default='mapping', help='the folder name after the date')
    parser.add_argument('--note', help='what this drive is (asked if not given)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='stop after this many seconds (default: Ctrl-C)')
    parser.add_argument('--sim', action='store_true',
                        help="Ravensim on this computer: the kit's own test")
    parser.add_argument('--loop', action='store_true',
                        help='with --sim: the whole route round the lot, back to the start')
    parser.add_argument('--sim-box', type=float, nargs='+', metavar='X Y [YAW]',
                        help='with --sim: a car-sized box at x, y (m; yaw deg) for this '
                             'drive only (test data for map-area, TODO 259)')
    parser.add_argument('--check', action='store_true', help='only the checks')
    parser.add_argument('--summary', type=Path, metavar='SESSION',
                        help="a recorded drive's summary again")
    parser.add_argument('--raw-cameras', action='store_true',
                        help='raw pictures, not JPEG: about 22 GB an hour a camera more')
    parser.add_argument('--check-seconds', type=float, default=5.0,
                        help='how long the check listens (5)')
    parser.add_argument('--split-gb', type=float, default=SPLIT_BYTES / 1e9,
                        help='a new file every this many GB (2)')
    parser.add_argument('--split-minutes', type=int, default=SPLIT_SECONDS // 60,
                        help='... or every this many minutes (5)')
    parser.add_argument('--no-ssh', action='store_true',
                        help="don't ask the car's other computers for their clocks")
    parser.add_argument('--yes', action='store_true', help='record despite warnings')
    parser.add_argument('--force', action='store_true', help='record despite a refusal')
    args = parser.parse_args()
    if args.summary:
        meta = args.summary / 'drive.yaml'
        sim = bool(yaml.safe_load(meta.read_text()).get('sim')) if meta.exists() else False
        page, _ = summarise(args.summary, with_preflight_rates(
            rec.streams(sim, args.raw_cameras)), sim)
        print(page)
        return 0
    return sim_drive(args) if args.sim else car_drive(args)


if __name__ == '__main__':
    sys.exit(main())

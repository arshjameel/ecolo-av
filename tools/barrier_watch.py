#!/usr/bin/env python3
"""
Watch the barriers in Ravensim: one drive past a ghost, the follower three ways.

Outside the capstone's scope (Ben's own research, toward his MASc thesis): docs/barriers.md,
"Watch it in Ravensim". This computer only (pixi runs it through tools/ros.sh --local, on a
ROS domain of its own), so it never reaches a real car.

  pixi run barrier-watch plain                 the follower (mpc_lag_rs) alone
  pixi run barrier-watch step                  a hard distance limit: the rule with gamma 1
  pixi run barrier-watch barrier               the barrier (gamma 0.05)
  pixi run barrier-watch barrier --speed 3 --offset 0.4
  pixi run barrier-watch --side-by-side        plain and barrier at once, two simulators
  Ctrl-C                                       stops everything it started

It starts Ravensim's road world with navigation and the ghost, and (without --headless)
our own navigation view (pixi run nav-view, the developer window's map panel alone) on that
simulator, the mode in its title: the map, the local costmap, the ghost (purple) in the
barrier's keep-out circle, the plan, the car's trail and the stop zone's box. It keeps the
view's newest picture beside the CSV (<mode>-<speed>-<offset>.png, every 2 s). RViz opens
only with --rviz (rviz/barrier.rviz, the reference). It
waits for navigation, places the ghost, sends a goal straight ahead, and prints one line.
Every safety lock stays on (the stop zone too): nothing here takes one out.

  barrier, 3 m/s, ghost 0 m left: WENT ROUND: least gap 1.31 m; first to act: the
  tracker's stop rule, at 2.68 m/s, 7.83 m from it; stops asked by: the stop rule; ...

Options:
  --speed V        m/s the car may drive (1, navigation's own; up to 5: sim_speed:=)
  --offset M       the ghost's centre, m left of the car's line (negative: right; 0: on it)
  --ahead M        the ghost's near edge, m past the front bumper at the start (15)
  --goal M         the goal, m straight ahead of the car (40)
  --radius M       the ghost's radius (0.25: a person 0.5 m wide)
  --gamma G        with barrier: another share of the room a tick (0.05)
  --headless       no windows, and stop when the drive ends (tests)
  --dev-window     the whole developer window, live, in place of the navigation view
  --rviz           also RViz, on rviz/barrier.rviz (the stock viewer: the reference only)
  --side-by-side   two runs at once, each its own simulator and domain: plain and barrier
                   (or --modes plain,step)
  --keep S         with windows: stay open this long after the result (default: until Ctrl-C)
  name:=value      any other sim.launch.py switch, last so it wins (cameras:=true ...)

Each run's samples go to /tmp/aav-barrier-watch/<mode>-<speed>-<offset>.csv, its commit,
command and settings first; the launch's log beside it. The maths is tools/barrier_run.py.
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import barrier_run  # noqa: E402, I100 - beside this file

OUT = '/tmp/aav-barrier-watch'
RVIZ = os.path.join(ROOT, 'ros', 'bringup', 'aav_bringup', 'rviz', 'barrier.rviz')
GROUPS = []         # the process groups this run started, to stop


def options(argv):
    parser = argparse.ArgumentParser(prog='pixi run barrier-watch', add_help=False)
    parser.add_argument('mode', nargs='?', choices=barrier_run.MODES)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--offset', type=float, default=0.0)
    parser.add_argument('--ahead', type=float, default=15.0)
    parser.add_argument('--goal', type=float, default=40.0)
    parser.add_argument('--radius', type=float, default=0.25)
    parser.add_argument('--gamma', type=float)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--dev-window', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--side-by-side', action='store_true')
    parser.add_argument('--modes', default='plain,barrier')
    parser.add_argument('--keep', type=float)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('-h', '--help', action='store_true')
    args, rest = parser.parse_known_args(argv)
    extra = [a for a in rest if ':=' in a and not a.startswith('-')]
    wrong = [a for a in rest if a not in extra]
    if args.help:
        print(__doc__)
        sys.exit(0)
    if wrong or (args.mode is None and not args.side_by_side):
        print(f'usage: pixi run barrier-watch <{"|".join(barrier_run.MODES)}> [--speed V] '
              '[--offset M] [--headless] [name:=value ...]   (--help: all of it)'
              + (f'\nnot understood: {" ".join(wrong)}' if wrong else ''), file=sys.stderr)
        sys.exit(2)
    return args, extra


def stop_groups(wait=30.0):
    """Stop every process group this run started: SIGINT, then SIGKILL after `wait` s."""
    for group in GROUPS:
        try:
            os.killpg(group.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    end = time.monotonic() + wait
    for group in GROUPS:
        try:
            group.wait(timeout=max(0.1, end - time.monotonic()))
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(group.pid, signal.SIGKILL)    # what its launch left behind, if any
        except ProcessLookupError:
            pass
    GROUPS.clear()


def start(command, log=None, env=None):
    """Start a program in a process group of its own, and remember it."""
    out = open(log, 'w') if log else None
    group = subprocess.Popen(command, cwd=ROOT, env=env, stdout=out,
                             stderr=subprocess.STDOUT if out else None,
                             start_new_session=True)
    GROUPS.append(group)
    return group


def side_by_side(args, argv):
    """Run two modes at once, each a copy of this program on a ROS domain of its own."""
    modes = args.modes.split(',')
    if len(modes) != 2 or any(m not in barrier_run.MODES for m in modes):
        print(f'--modes takes two of {", ".join(barrier_run.MODES)}: plain,barrier',
              file=sys.stderr)
        return 2
    from aav_io_mux.domains import free_domain
    passed = [a for a in argv if a != '--side-by-side' and a not in barrier_run.MODES]
    first = free_domain()
    # The second: another free one. free_domain looks at what runs now, and the first
    # simulator isn't up yet: step past it.
    domains = [first, first + 1 if first < 232 else first - 1]
    for slot, (mode, domain) in enumerate(zip(modes, domains)):
        # The slot: where this run's window opens, so the two sit beside each other.
        env = dict(os.environ, AAV_SIM_DOMAIN=str(domain), AAV_BARRIER_WATCH_SLOT=str(slot))
        start([sys.executable, os.path.abspath(__file__), mode, *passed], env=env)
        print(f'{mode}: its own simulator, on ROS domain {domain}', flush=True)
    codes = [group.wait() for group in list(GROUPS)]
    return max(codes)


def run(args, extra):
    switches = barrier_run.launch_switches(args.mode, args.speed, args.gamma)
    # No cameras unless asked: nothing here looks through them, and the drive is the
    # same on a computer with no GPU.
    switches += ['cameras:=false', *extra]
    # sim_check sets this run's own ROS domain as it loads (AAV_SIM_DOMAIN, or a free
    # one), on this computer only; its Probe, ghost and goal calls are the ones used here.
    import sim_check
    from aav_description import vehicle
    from diagnostic_msgs.msg import DiagnosticStatus
    from nav2_msgs.msg import CollisionMonitorState
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient

    os.makedirs(OUT, exist_ok=True)
    stem = os.path.join(OUT, f'{args.mode}-{args.speed:g}-{args.offset:g}')
    env = dict(os.environ)
    print(f'{args.mode}: Ravensim on ROS domain {env["ROS_DOMAIN_ID"]} (this computer only): '
          f'sim.launch.py {" ".join(switches)}', flush=True)
    start(['ros2', 'launch', 'aav_bringup', 'sim.launch.py', *switches], f'{stem}.log', env)
    if not args.headless:
        title = (f'barrier-watch: {args.mode.upper()}, {args.speed:g} m/s, the ghost '
                 f'{args.offset:g} m left')
        start(view_command(args, title, f'{stem}.png',
                           os.environ.get('AAV_BARRIER_WATCH_SLOT')),
              f'{stem}-window.log', env)
        if args.rviz:       # only when asked: the stock viewer, as the reference
            start(['rviz2', '-d', rviz_layout(f'{stem}.rviz', args), '-t', title],
                  f'{stem}-rviz.log', env)
    rclpy.init()
    probe = sim_check.Probe()
    status = {}
    probe.create_subscription(
        DiagnosticStatus, '/drivers/nav/track/status',
        lambda m: status.update({v.key: v.value for v in m.values}), 10)
    try:
        def missing():
            return [n for n, m in (('truth', probe.truth), ('the global EKF', probe.glob),
                                   ('the sensor watchdog', probe.lock)) if m is None]

        print('waiting for the simulator and navigation...', flush=True)
        if not probe.spin_for(120.0, lambda: not missing()):
            print(f'FAILED TO START: nothing from {", ".join(missing())} in 120 s '
                  f'(see {stem}.log)')
            return 1
        if not probe.spin_for(120.0, probe.nav.server_is_ready):
            print(f'FAILED TO START: the navigator never came up (see {stem}.log)')
            return 1
        client = AsyncParameterClient(probe, '/ghost_obstacle')
        if not client.wait_for_services(timeout_sec=20.0):
            print('FAILED TO START: no ghost_obstacle')
            return 1
        body = barrier_run.Body.from_vehicle(vehicle.load())
        # The simulator's truth: the ghost stands where its map coordinates are in the
        # world (the map's origin is the world's), as a real thing would.
        origin = (*probe.xy(probe.truth), sim_check.yaw_of(probe.truth))
        centre = barrier_run.object_centre(body, args.ahead, args.offset, args.radius)
        gx, gy = barrier_run.from_run_frame(*centre, origin)
        # ahead below 0: at x, y in the map.
        if not sim_check.set_ghost(probe, client, ahead=-1.0, x=gx, y=gy,
                                   width=2.0 * args.radius, enabled=True):
            print("FAILED TO START: ghost_obstacle didn't take the ghost")
            return 1
        probe.keep_out = keep_out(probe, gx, gy, args.radius)    # held: a view may come late
        if probe.glob.header.frame_id != 'map':
            print(f'FAILED TO START: the global EKF is in {probe.glob.header.frame_id}, not map')
            return 1
        print(f'the ghost: {args.ahead:g} m past the bumper, {args.offset:g} m left of the '
              f"car's line (map x {gx:.2f}, y {gy:.2f})", flush=True)
        probe.spin_for(1.0)
        done = probe.go_to(*barrier_run.from_run_frame(args.goal, 0.0, origin), origin[2])
        if done is None:
            print('FAILED TO START: no navigator, or it refused the goal')
            return 1
        print(f'the goal: {args.goal:g} m straight ahead. Driving...', flush=True)
        samples, seen, still, moved = [], len(probe.monitor), None, False
        end = time.monotonic() + args.timeout
        while time.monotonic() < end and not done.done():
            probe.spin_for(0.05)
            t, believed = probe.truth, probe.glob
            x, y, yaw = barrier_run.to_run_frame(*probe.xy(t), sim_check.yaw_of(t), origin)
            zone = (probe.monitor[-1][0] == CollisionMonitorState.STOP
                    if len(probe.monitor) > seen else False)
            samples.append(barrier_run.Sample(
                t=sim_check.seconds(t.header.stamp), x=x, y=y, yaw=yaw,
                v=t.twist.twist.linear.x, barrier_stop=status.get('barrier_stop') == '1',
                zone_stop=zone, off=math.dist(probe.xy(t), probe.xy(believed))))
            moved = moved or samples[-1].v > barrier_run.MOVING
            if moved and abs(samples[-1].v) < barrier_run.STILL:
                still = still if still is not None else probe.now()
                if probe.now() - still > 8.0:       # stood 8 s: it is not going on
                    break
            else:
                still = None
        result = barrier_run.verdict(samples, body, centre, args.radius)
        reached = done.done() and done.result().status == 4
        save(f'{stem}.csv', args, switches, samples, body, centre, result, reached)
        print(f'{args.mode}, {args.speed:g} m/s, ghost {args.offset:g} m left: {result.line()}; '
              f'goal {"reached" if reached else "not reached"}   ({stem}.csv)', flush=True)
        if not done.done():
            probe.goal.cancel_goal_async()
            probe.spin_for(1.0)
        if not args.headless:
            print('Ctrl-C to close the windows and the simulator.' if args.keep is None
                  else f'closing in {args.keep:g} s.', flush=True)
            probe.spin_for(args.keep if args.keep is not None else 86400.0)
        return 0
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


VIEW_WIDTH = 900    # the navigation view's window, points across (dev_window --map-only)


def view_command(args, title, picture, slot=None):
    """
    Return the command of this run's own picture: our navigation view, live on its simulator.

    The developer window's map panel alone (tools/dev_window.sh --map-only), or the whole
    window with --dev-window; the mode in the title; the view as wide as the drive and a
    margin, following the car; its newest screenshot kept in `picture`. `slot` (0 or 1,
    a side-by-side pair): the second window opens to the right of the first.
    """
    command = ['tools/dev_window.sh']
    if not args.dev_window:
        command.append('--map-only')
    command += ['--live', '--title', title, '--map-span', f'{args.goal + 20.0:g}',
                '--screenshot', picture, '--screenshot-every', '2']
    if slot is not None:
        command += ['--position', f'{40 + int(slot) * (VIEW_WIDTH + 20)},60']
    return command


def rviz_layout(path, args):
    """
    Write this run's copy of rviz/barrier.rviz, its view on this drive; return the path.

    The view is centred between the start and the goal, and the scale fits the drive
    into about half a laptop's screen beside RViz's panels (RViz counts real pixels: a
    Mac's screen has two a point). Scroll to zoom. Where the window opens is RViz's own
    choice: it keeps its last place.
    """
    import yaml
    with open(RVIZ) as f:
        layout = yaml.safe_load(f)
    view = layout['Visualization Manager']['Views']['Current']
    view['X'] = args.goal / 2.0
    view['Scale'] = round((1700 if sys.platform == 'darwin' else 850) / (args.goal + 6.0), 1)
    with open(path, 'w') as f:
        yaml.safe_dump(layout, f)
    return path


# The barrier's own numbers (aav_mpc_lag's barriers.body_radius and barriers.margin, its
# README): the body's point may come this near the object's edge, and no nearer.
KEEP_OUT = 1.12 + 0.2


def keep_out(probe, x, y, radius):
    """
    Draw the barrier's keep-out circle round the object, for the view; return what holds it.

    The same circle in every mode, so the plain run shows what it drives through. A line
    on the ground in the map frame, sent once and kept for a view that starts late
    (transient local). The topic is in docs/interfaces.yaml; only screens read it.
    """
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = probe.create_publisher(Marker, '/drivers/nav/track/barrier_watch/keep_out', latched)
    m = Marker()
    m.header.frame_id = 'map'
    m.ns, m.id, m.type, m.action = 'keep_out', 0, Marker.LINE_STRIP, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.05
    m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.3, 1.0, 0.9
    r = radius + KEEP_OUT
    m.points = [Point(x=x + r * math.cos(a), y=y + r * math.sin(a), z=0.05)
                for a in (math.radians(d) for d in range(0, 361, 5))]
    pub.publish(m)
    return pub


def save(path, args, switches, samples, body, centre, result, reached):
    """Write the run: its commit, command and settings first, then every sample."""
    commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT,
                            capture_output=True, text=True).stdout.strip()
    with open(path, 'w', newline='') as f:
        f.write(f'# commit {commit}\n# command pixi run barrier-watch {" ".join(sys.argv[1:])}\n'
                f'# sim.launch.py {" ".join(switches)}\n'
                f"# ghost: centre x {centre[0]:.3f}, y {centre[1]:.3f} (the run's frame), "
                f'radius {args.radius:g}; goal {args.goal:g} m\n'
                f'# result: {result.line()}; goal {"reached" if reached else "not reached"}\n')
        rows = csv.writer(f)
        rows.writerow(['t', 'x', 'y', 'yaw', 'v', 'gap', 'barrier_stop', 'zone_stop', 'off'])
        for s in samples:
            rows.writerow([f'{s.t:.3f}', f'{s.x:.3f}', f'{s.y:.3f}', f'{s.yaw:.4f}',
                           f'{s.v:.3f}', f'{barrier_run.gap(body, s, centre, args.radius):.3f}',
                           int(s.barrier_stop), int(s.zone_stop), f'{s.off:.3f}'])


def main():
    argv = sys.argv[1:]
    args, extra = options(argv)
    # Ctrl-C, or a stop from the parent of a side-by-side pair: the same clean end. Set by
    # hand: a program started in the background of a script has Ctrl-C's signal ignored.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        if args.side_by_side:
            return side_by_side(args, argv)
        return run(args, extra)
    except KeyboardInterrupt:
        print('\nstopping everything this run started...', flush=True)
        return 130
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2
    finally:
        # A pair's two runs each stop their own simulator first: give them the time.
        stop_groups(75.0 if args.side_by_side else 30.0)


if __name__ == '__main__':
    sys.exit(main())

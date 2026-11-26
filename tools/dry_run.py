#!/usr/bin/env python3
"""
The dry run: the virtual test platform, three screens in one go (Ben, 23 Nov).

    pixi run dry-run                    Ravensim's screen, the hardware screen, the tests screen
    pixi run dry-run --nav              and the navigation view in a window of its own
    pixi run dry-run --debug            and the raylib debug screen (the in-car one)
    pixi run dry-run --no-panel         without the hardware panel
    pixi run dry-run --no-picture       without the developer window (a slow laptop)

  Screen            Window                         Shows
  1 Ravensim        the developer window           the 3D scene; the debug view (what the
                                                   lidar sees: what we can hit); the map
                                                   from above (the nav view): one window
                    --nav: the navigation view     that map alone, for another monitor
  2 the hardware    the hardware panel             the car's switches and lights
  3 the tests       the ladder window              the rungs, each test's Run, the session
    pixi run dry-run sim_speed:=2       any sim.launch.py switch, name:=value (yours win)
    Ctrl-C, or close the ladder window  stops everything it started

Everything we will do with the real car, done first with the simulated one you can see
(Ben, 22 Nov; TODO 273; docs/proving-it.md). The ladder window's runs attach to this
simulator (car-test --sim --attach), so the car moves in the picture while a test runs.

What it starts, each in a process group of its own, all on this computer only (the task is
tools/ros.sh --local, and every part goes through it again):

  1. the simulator   ros2 launch aav_bringup sim.launch.py + SIM_ARGS (below, with why);
                     its output goes to /tmp/aav-dry-run/sim-launch.log
  2. the picture     tools/dev_window.sh --live (the developer window on the simulator)
  3. the ladder      the ladder window (ros/hud/aav_dev_window, ladder_window; no ROS)
  4. the hardware    the hardware panel (ros2 launch aav_hud hw_panel.launch.py); not with
                     --no-panel
  5. --nav           the navigation view (tools/dev_window.sh --map-only --live)
  6. --debug         the debug screen (ros2 launch aav_hud debug_hud.launch.py)

No RViz anywhere: the simulator gets rviz:=false, and the pictures are ours.

It refuses to start when a simulator, or anything else, already announces on
/system/source here (tools/jacks.py's check): two simulators on one ROS domain, or one
beside a real car, would be wrong. It never moves a real car: it has no car side at all.

Ctrl-C, the simulator ending, or the ladder window closing stops every group it started:
SIGINT, then SIGTERM after 20 s, then SIGKILL; it says so if anything is left.

The ladder window's tests are told which process group is the simulator's
(AAV_SIM_LAUNCH_LOG, AAV_SIM_LAUNCH_PGID; and /tmp/aav-dry-run/sim-launch.pgid for a test
typed in another terminal), so the stands test's heartbeat step can pause this simulator's
output mux, and nobody else's.

No ROS in this file: it starts programs. Its plan is tested in tools/tests/test_dry_run.py.
"""
import argparse
from dataclasses import dataclass
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCAL = ('tools/ros.sh', '--local')
LOG_DIR = '/tmp/aav-dry-run'
SIM_LOG = os.path.join(LOG_DIR, 'sim-launch.log')
SIM_GROUP = os.path.join(LOG_DIR, 'sim-launch.pgid')   # car_test.attached_launch reads it
# The one world every rung's test can use (the tests' own headless simulators differ:
# car-test --stands takes the road world, --hallway the empty one with no cameras):
#   world:=road             the stands test waits for a camera's picture and needs
#                           something for the lidar to see; the picture shows a road
#   nav:=false              nothing else drives, as in every actuator test's simulator
#   faults:=true            the stands test's step 5 breaks the camera and the lidar
#   sim_speed:=3            ego's cap, so brake and speed-up reach 2 and 3 m/s (the lot)
#   start_x:=-150           the hallway's and the lot's passes all drive forward (nobody
#                           turns the simulated car round): 340 m of flat ground ahead
#   geofence_radius:=340    the fence ends 10 m before the ground does (x = 190 of 200)
SIM_ARGS = ('world:=road', 'rviz:=false', 'nav:=false', 'faults:=true', 'sim_speed:=3',
            'start_x:=-150', 'geofence_radius:=340')
SIM_UP = 180.0      # s for the simulator to announce itself
STOP_WAIT = 20.0    # s for a group to end after SIGINT, and again after SIGTERM


@dataclass(frozen=True)
class Part:
    name: str
    cmd: tuple
    needed: bool = False    # its ending ends the dry run
    log: str = ''           # its output goes here, not to the terminal
    after_sim: bool = True  # started once the simulator has announced itself


def parse(argv):
    parser = argparse.ArgumentParser(
        prog='pixi run dry-run',
        description='The virtual test platform: Ravensim, the hardware panel and the tests.')
    parser.add_argument('--panel', action='store_true',
                        help='the hardware panel (it opens anyway since 23 Nov: kept so '
                             'an old command still works)')
    parser.add_argument('--no-panel', action='store_true', help='no hardware panel')
    parser.add_argument('--nav', action='store_true',
                        help='the navigation view too, in a window of its own')
    parser.add_argument('--debug', action='store_true', help='the debug screen too')
    parser.add_argument('--no-picture', action='store_true',
                        help='no developer window on the simulator')
    parser.add_argument('switches', nargs='*', metavar='name:=value',
                        help="sim.launch.py's switches; yours come last, so they win")
    args = parser.parse_args(argv)
    bad = [s for s in args.switches if ':=' not in s]
    if bad:
        parser.error(f'not a switch (name:=value): {" ".join(bad)}')
    return args


def plan(args):
    """Return the parts to start, in order: the simulator first."""
    parts = [Part('the simulator', LOCAL + ('ros2', 'launch', 'aav_bringup', 'sim.launch.py')
                  + SIM_ARGS + tuple(args.switches), needed=True, log=SIM_LOG,
                  after_sim=False)]
    if not args.no_picture:
        parts.append(Part("the simulator's picture", ('tools/dev_window.sh', '--live')))
    parts.append(Part('the ladder window',
                      ('cargo', 'run', '--release', '-q', '--manifest-path',
                       'ros/hud/aav_dev_window/Cargo.toml', '--bin', 'ladder_window'),
                      needed=True))
    if not args.no_panel:
        parts.append(Part('the hardware panel',
                          LOCAL + ('ros2', 'launch', 'aav_hud', 'hw_panel.launch.py')))
    if args.nav:
        parts.append(Part('the navigation view',
                          ('tools/dev_window.sh', '--map-only', '--live')))
    if args.debug:
        parts.append(Part('the debug screen',
                          LOCAL + ('ros2', 'launch', 'aav_hud', 'debug_hud.launch.py')))
    return parts


def heard():
    """Ask tools/jacks.py what announces here: its answer (a dict), or None."""
    done = subprocess.run([*LOCAL, 'python3', 'tools/jacks.py', 'status', '--sim', '--json',
                           '--wait', '2'], cwd=ROOT, capture_output=True, text=True)
    for line in reversed(done.stdout.splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


def why_not_start(answer):
    """Return why the dry run must not start here, or None; `answer` is heard()'s."""
    if answer is None:
        return 'tools/jacks.py gave no answer: is the workspace built (pixi run build)?'
    if answer.get('simulator'):
        return ('a simulator is already running on this computer. Stop it first; or keep '
                'it, and open only the window: pixi run ladder-window')
    if 'no simulator heard' in answer.get('message', ''):
        return None
    return f'this is not a place for a simulator: {answer.get("message", "")}'


def alive(group):
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:      # macOS, for a group whose leader has gone
        return False


def start(part, env):
    """Start a part as the leader of its own process group; return its Popen."""
    out = None
    if part.log:
        os.makedirs(os.path.dirname(part.log), exist_ok=True)
        out = open(part.log, 'w')
    return subprocess.Popen(part.cmd, cwd=ROOT, env=env, stdout=out,
                            stderr=subprocess.STDOUT if out else None,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def stop(procs, wait=STOP_WAIT, say=print):
    """Stop every group, the last started first; return the names of any still alive."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        left = [(name, p) for name, p in procs if alive(p.pid)]
        if not left:
            break
        for name, p in reversed(left):
            try:
                os.killpg(p.pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        end = time.monotonic() + (wait if sig is not signal.SIGKILL else 2.0)
        while time.monotonic() < end and any(alive(p.pid) for _, p in left):
            for _, p in left:
                p.poll()         # reap, so a finished leader's group reads as gone
            time.sleep(0.2)
    for _, p in procs:
        p.poll()
    return [name for name, p in procs if alive(p.pid)]


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    why = why_not_start(heard())
    if why:
        print(f'REFUSED: {why}', file=sys.stderr)
        return 2
    parts = plan(args)
    procs = []
    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append('Ctrl-C'))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append('asked to stop'))
    env = dict(os.environ)
    try:
        sim = start(parts[0], env)
        procs.append((parts[0].name, sim))
        env.update(AAV_SIM_LAUNCH_LOG=SIM_LOG, AAV_SIM_LAUNCH_PGID=str(sim.pid))
        with open(SIM_GROUP, 'w') as f:      # for a test typed in another terminal
            f.write(f'{sim.pid}\n')
        print(f'1/{len(parts)}  the simulator: sim.launch.py {" ".join(parts[0].cmd[6:])}')
        print(f'      its output: {SIM_LOG}; waiting for it to announce itself ...',
              flush=True)
        end = time.monotonic() + SIM_UP
        up = False
        while not stopping and time.monotonic() < end and sim.poll() is None:
            answer = heard()
            if answer and answer.get('simulator'):
                up = True
                break
        if not up and not stopping:
            print(f'The simulator did not come up (see {SIM_LOG}).', file=sys.stderr)
            return 1
        for number, part in enumerate(parts[1:], 2):
            if stopping:
                break
            print(f'{number}/{len(parts)}  {part.name}: {" ".join(part.cmd)}', flush=True)
            procs.append((part.name, start(part, env)))
        if not stopping:
            print()
            print('The dry run is up, on this computer only (a real car is never reached).')
            print('In the ladder window: choose a rung, Run in the simulator; the car in '
                  'the picture is the one tested.')
            print('Ctrl-C here, or close the ladder window, to stop everything.', flush=True)
        while not stopping:
            ended = [name for (name, p), part in zip(procs, parts)
                     if part.needed and p.poll() is not None]
            if ended:
                stopping.append(f'{ended[0]} ended')
            time.sleep(0.5)
        print(f'\n{stopping[0]}: stopping everything the dry run started ...', flush=True)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if os.path.exists(SIM_GROUP):
            os.remove(SIM_GROUP)
        left = stop(procs)
        if left:
            print(f'STILL RUNNING: {", ".join(left)} (look with: pgrep -fl sim.launch.py)',
                  file=sys.stderr)
        elif procs:
            print('Stopped: nothing is left running.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""
car_test: phases A and B of docs/testing-plan.md, one command, one report. The ROS part.

    pixi run car-test --stands                      the real car, wheels off the ground
    pixi run car-test --stands --sim --no-manual    its own Ravensim (this computer only)
    pixi run car-test --stands --only faults        some steps (car_test.STEPS, or a group)
    pixi run car-test --hallway                     phase B in a measured hallway (TODO 248)
    pixi run car-test --hallway --sim --no-manual   the same in its own Ravensim
    pixi run car-test --analyse <session>           a hallway session's report again

--hallway (HallwayTest): asks the hallway's measured length (or --length) and works out
its top speed (safety.hallway_limits: speed up + stop + margin, within 1.0 m/s); then each
pass (car_test.hall_passes: coast, speed-up, brake, steer, --repeats times, the car turned
round between passes unless --one-way) is an actuator_test hall-* child, which stands
still 10 s (the IMU's offset), drives, and brakes in time to stop inside the room. On the
car a person puts the car on the start line before each pass (Enter). Any pass that is
refused or stopped by a safety rule ends the session. Then hallway.py fits everything,
with error bars, and lays the model over each pass: ~/aav-tests/<time>-hallway[-sim]/
report.md, fits.csv, compare.csv, overlays/, vehicle-yaml.txt, session.yaml. In --sim,
Ravensim's empty world (no cameras), with --sim-actuators as the simulated car's own
numbers (sim.launch.py actuators:=) and --wheel-noise, --wheel-slip.

It writes one folder, ~/aav-tests/<time>-stands[-sim]/: report.md (the table), run.log
(everything printed), and a folder per step with that step's own files.

- Steps 1-4 run actuator_test as a child process each (the preflight, the sensors alone,
  steer-sweep, steer-rate, steer-centre, wheel-steps), with --stands (and --sim). On the
  car each asks the person to type its name, as it always does.
- Step 5's faults run here: a slow roll (car_test.ROLL), the fault, then who said why and
  how soon the car stood still. In --sim the runner makes each fault itself; on the car a
  person does (asked, then Enter), or --no-manual skips it (SKIPPED).
- Any FAIL ends the run: the steps after it are NOT RUN. It stops the car first: zeros
  until still; not still within 5 s, it publishes /safety/estop true (never false), as the
  preflight does.

Subscribes  everything the preflight does (PreflightNode: every sensor, counted), and
            safety/sensor_watchdog/state  std_msgs/String: who says why
            safety/sensor_watchdog/lock   std_msgs/Bool
            vehicle/status                aav_interfaces/VehicleStatus: armed, e-stop, faults
            safety/output_mux/state       std_msgs/String: the gate (OPEN before a step)
            (never a simulator's own topic, such as /control/sim/heartbeat: tools/drop_in.py)
Publishes   drivers/tests/cmd_vel  geometry_msgs/TwistStamped: the faults' slow roll, 50 Hz
            safety/estop           std_msgs/Bool, true only: when the car won't stop; and
                                   in --sim the e-stop step's press, standing in for a person
Calls       /<node>/set_parameters  --sim only: fault.<sensor> stop, then ok, on Ravensim's
                                    sensor node (aav_sim_models faults.NODES)
Signals     --sim only: its own Ravensim's output_mux, SIGSTOP then SIGCONT (the heartbeat
            step), found in its own launch's log and checked in its own process group:
            never anyone else's process
Runs        python -m aav_actuator_tests.actuator_test per tool step; with --sim,
            sim.launch.py (world:=road faults:=true nav:=false rviz:=false),
            in its own process group, stopped by that group at the end

The logic is in car_test.py (no ROS); this file only connects it to ROS.
"""

import csv
import datetime
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

from aav_actuator_tests import car_test as ct
from aav_actuator_tests import hallway
from aav_actuator_tests import reports
from aav_actuator_tests import safety
from aav_actuator_tests.actuator_test import Aborted
from aav_actuator_tests.analysis import STOPPED
from aav_actuator_tests.preflight_node import PreflightNode
from aav_description import vehicle
from aav_interfaces.msg import VehicleStatus
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
import yaml

ODOM_STALE = 0.5       # s
TICK = 0.02            # s: 50 Hz
ROLL_UP = 10.0         # s for the roll to start
STOP_TIME = 5.0        # s for the car to be still after the zeros, else the e-stop
ESTOP_TIME = 10.0      # s of zeros after that e-stop
SIM_UP = 600.0         # s for our Ravensim to come up (its cameras may build first)
MEND_TIME = 30.0       # s for the watchdog to say OK again after a fault is mended
# What the watchdog's inputs are called in the preflight's sensor counts (preflight.SENSORS).
COUNTED = {'gps': 'gps', 'imu': 'imu_filtered', 'camera': 'camera', 'lidar': 'lidar'}
SIM_ARGS = ('world:=road', 'rviz:=false', 'faults:=true', 'nav:=false')


class Tee:
    """Print to the terminal and to run.log."""

    def __init__(self, path):
        self.file = open(path, 'a')

    def say(self, text=''):
        print(text, flush=True)
        self.file.write(text + '\n')
        self.file.flush()

    def raw(self, data):
        sys.stdout.write(data)
        sys.stdout.flush()
        self.file.write(data)
        self.file.flush()


class SimLaunch:
    """Our own Ravensim: its own process group and log, so we only ever touch our own."""

    def __init__(self, folder, say, args=SIM_ARGS):
        self.log_path = os.path.join(folder, 'sim-launch.log')
        self.proc = None
        self.say = say
        self.args = tuple(args)

    def start(self):
        self.say(f'Starting Ravensim on ROS domain {os.environ["ROS_DOMAIN_ID"]} '
                 f'(this computer only): {" ".join(self.args)}')
        log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py',
                                      *self.args], stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def pid_of(self, name):
        """Return a node's pid from our launch's log, only if it is in our process group."""
        with open(self.log_path) as f:
            found = re.findall(rf'\[{name}-\d+\]: process started with pid \[(\d+)\]', f.read())
        if not found:
            return None
        pid = int(found[-1])
        try:
            return pid if os.getpgid(pid) == self.proc.pid else None
        except ProcessLookupError:
            return None

    def stop(self):
        if self.proc is None:
            return
        self.say('Stopping our Ravensim (its own process group)...')
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            # Gone already (macOS says EPERM for a group whose leader has exited, 20 Nov).
            pass


class CarTestNode(PreflightNode):
    """The preflight's node (every sensor counted), and who says why."""

    def __init__(self, sim):
        super().__init__(sim, name='car_test')
        self.watchdog = None       # (s, line)
        self.wd_lock = None        # not `lock`: PreflightNode's is the legacy bridge's
        self.status = None         # (s, VehicleStatus)
        self.gate = None           # (s, line)
        self.create_subscription(String, 'safety/sensor_watchdog/state', self.on_watchdog, 10)
        self.create_subscription(Bool, 'safety/sensor_watchdog/lock', self.on_wd_lock, 10)
        self.create_subscription(VehicleStatus, 'vehicle/status', self.on_status, 10)
        self.create_subscription(String, 'safety/output_mux/state', self.on_gate, 10)

    def on_watchdog(self, msg):
        self.watchdog = (self.seconds(), msg.data)

    def on_wd_lock(self, msg):
        self.wd_lock = bool(msg.data)

    def on_status(self, msg):
        self.status = (self.seconds(), msg)

    def on_gate(self, msg):
        self.gate = (self.seconds(), msg.data)

    def state(self):
        return self.watchdog[1] if self.watchdog else None

    def fresh_status(self, age=1.0):
        if self.status and self.seconds() - self.status[0] <= age:
            return self.status[1]
        return None


class CarTest:
    """The whole run: the checks before, each step, the stops, the report."""

    def __init__(self, node, args, folder, sim, out):
        self.node, self.args, self.folder, self.sim, self.out = node, args, folder, sim, out
        self.say = out.say
        self.results = []
        self.yaml = {}
        self.notes = []
        self.rolling = False
        self.cfg = vehicle.load()
        self.wheelbase = reports.context(self.cfg).wheelbase
        self.cap = safety.STANDS_CAP
        self.real = True           # until /system/source says otherwise (prepare)
        self.interrupted = False
        self.inputs = watchdog_inputs()
        self.still_since = None

    # --- before -----------------------------------------------------------------------

    def prepare(self):
        """Every check before anything moves; returns why not, or None."""
        node, a = self.node, self.args
        if self.sim:
            self.say('Waiting for our Ravensim: /system/source SIM, the speed, the camera, '
                     'the sensor watchdog, and the output gate OPEN (the first run on a '
                     'computer also builds its cameras: minutes) ...')
            up = node.spin_for(SIM_UP, lambda: (
                safety.source_kind(node.sources) is safety.Kind.SIM and self.fresh_odom()
                and bool(node.samples['camera'].times)
                and node.watchdog is not None and node.gate is not None
                and node.gate[1].startswith('OPEN')))
            if not up:
                return (f'Ravensim did not come up in {SIM_UP:.0f} s (see '
                        f'{self.sim.log_path})')
            node.spin_for(5.0)   # the rest settles
            self.notes.append(f'Ravensim has no {", ".join(ct.SIM_SKIP)}: the preflight '
                              'steps skip it (--skip)')
        else:
            node.spin_for(3.0, lambda: bool(node.sources))
            node.spin_for(1.0)
        kind = safety.source_kind(node.sources)
        d = safety.decide(kind, a.sim, True, 3)
        if not d.allowed:
            return d.reason
        self.cap, self.real = d.cap, d.real
        self.say(f'Driving: {d.reason} ({kind.value}); every step on stands, capped at '
                 f'{d.cap:g} m/s; the faults roll at {min(ct.ROLL, d.cap):g} m/s')
        if d.real:
            self.say()
            self.say('THE REAL CAR, ON STANDS. Check, out loud:')
            for item in safety.PREFLIGHT_CHECKLIST:
                self.say(f'  [ ] {item}')
            self.say('  [ ] The e-stop person stays at the button for EVERY step, the faults too')
            self.say('Each test asks again before it moves. To stop at any time: Ctrl-C '
                     'here, the e-stop, or the joystick deadman.')
            if not sys.stdin.isatty():
                return 'the real car needs a person at the keyboard to confirm'
            typed = ask('Type "car-test" to start (anything else cancels): ')
            if not safety.confirmed(typed, 'car-test'):
                return 'cancelled: nothing moved'
        return None

    def fresh_odom(self):
        t = self.node.odom_time
        return t is not None and self.node.seconds() - t < ODOM_STALE

    # --- the run ----------------------------------------------------------------------

    def run(self):
        steps = ct.plan(self.args.only, bool(self.args.sim), self.args.no_manual)
        why = self.prepare()
        if why:
            self.say(f'REFUSED: {why}')
            self.notes.append(f'Refused before any step: {why}')
            self.results = [ct.Result(s, ct.Status.NOT_RUN, 'refused before any step')
                            for s, _ in steps]
            return
        stopped = None
        for step, skip in steps:
            if stopped:
                self.results.append(ct.Result(step, ct.Status.NOT_RUN, stopped))
                continue
            if skip:
                self.results.append(ct.Result(step, ct.Status.SKIPPED, skip))
                continue
            self.say()
            self.say(f'=== Step {step.number}: {step.title} ===')
            try:
                result = self.tool(step) if step.how is ct.How.TOOL else self.fault(step)
            except Aborted as e:
                result = ct.Result(step, ct.Status.FAIL, f'ABORTED: {e}')
            except KeyboardInterrupt:
                # A second Ctrl-C must not cut the stop short: the run ends here anyway.
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                self.interrupted = True
                result = ct.Result(step, ct.Status.FAIL, 'Ctrl-C')
            finally:
                if not self.stop_car():
                    self.notes.append(f'Step {step.number}: NOT STOPPED within '
                                      f'{STOP_TIME:g} s: the runner set the e-stop')
                    stopped = 'the car would not stop: the e-stop is set'
            if stopped and result.status is not ct.Status.FAIL:
                result = ct.Result(step, ct.Status.FAIL, f'{result.detail}; {stopped}',
                                   result.files)
            self.results.append(result)
            self.say(f'Step {step.number}: {result.status.value}: {result.detail}')
            if result.status is ct.Status.FAIL and not stopped:
                stopped = (f'step {step.number} ({step.name}) failed: the run stops there'
                           if not self.interrupted else 'Ctrl-C')

    # --- steps 1-4: actuator_test -------------------------------------------------------

    def tool(self, step):
        sub = os.path.join(self.folder, step.name)
        os.makedirs(sub, exist_ok=True)
        cmd = [sys.executable, '-m', 'aav_actuator_tests.actuator_test',
               *ct.tool_args(step, bool(self.args.sim), sub)]
        self.say(f'$ actuator-test {" ".join(cmd[3:])}')
        code = self.child(cmd, os.path.join(sub, 'output.log'))
        run = ct.only_folder(sub)
        files = os.path.relpath(run, self.folder) if run else step.name
        summary = ''
        if run and os.path.exists(os.path.join(run, 'summary.txt')):
            with open(os.path.join(run, 'summary.txt')) as f:
                summary = f.read()
        wheels = None
        if step.name == 'wheel-steps':
            rows = []
            if run and os.path.exists(os.path.join(run, 'log.csv')):
                with open(os.path.join(run, 'log.csv'), newline='') as f:
                    rows = reports.parse_rows(list(csv.DictReader(f)))
            v = reports.judge_wheel_steps(reports.wheel_step_results(rows))
            wheels = (v.follows, v.detail)
        steering = step.name in ct.GROUPS['steering']
        lines = ct.yaml_lines_of(summary) if steering else ''
        if lines:
            self.yaml[step.name] = lines
        status, detail = ct.tool_result(code, lines, wheels, needs_yaml=steering)
        return ct.Result(step, status, detail, files)

    def child(self, cmd, log_path):
        """
        Run an actuator_test; copy what it prints; return its exit code.

        Its stdin stays the terminal, so it can ask the person.
        """
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        got_int = []
        old = signal.signal(signal.SIGINT, lambda *_: got_int.append(True))
        try:
            with open(log_path, 'w') as log:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        env=env)
                # The child is in our process group: a Ctrl-C reaches it too, and it stops
                # the car itself. We keep copying until it has.
                while True:
                    data = os.read(proc.stdout.fileno(), 4096)
                    if not data:
                        break
                    text = data.decode(errors='replace')
                    self.out.raw(text)
                    log.write(text)
                code = proc.wait()
        finally:
            signal.signal(signal.SIGINT, old)
        if got_int:
            self.interrupted = True
            raise KeyboardInterrupt
        return code

    # --- step 5: the faults -------------------------------------------------------------

    def fault(self, step):
        node = self.node
        node.spin_for(0.5)
        if node.estop:
            raise Aborted('/safety/estop is already true: clear it first (pixi run clear estop)')
        if self.real:
            why = safety.preflight_refusal(
                safety.Kind.REAL, True, self.cfg.get('vehicle', {}).get('speed_wheels'),
                node.lock, None)
            if why:
                raise Aborted(why)
        sim = self.args.sim
        if step.sensor:
            return self.sensor_fault(step, sim)
        if step.name == 'heartbeat':
            return self.heartbeat_sim(step) if sim else self.heartbeat_car(step)
        if step.name == 'estop':
            return self.estop(step, sim)
        return self.manual_fault(step)

    # The roll and the checks while it runs.

    def tick(self, speed, estop_ok=False, stale_ok=False, press=False):
        """One 50 Hz tick: send, listen, check. Raises Aborted on any rule."""
        node = self.node
        node.send(speed, 0.0, self.wheelbase)
        if press:
            node.press_estop()
        end = time.monotonic() + TICK
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)
        v = node.v['speed']
        fresh = self.fresh_odom()
        if fresh and math.isfinite(v) and abs(v) > safety.speed_limit(self.cap):
            raise Aborted(f'speed {abs(v):.2f} m/s is over the limit '
                          f'{safety.speed_limit(self.cap):.2f} m/s')
        if node.estop and not estop_ok:
            raise Aborted('/safety/estop is true')
        if not fresh and not stale_ok:
            raise Aborted(f'no /vehicle/odom for {ODOM_STALE} s: the speed is unknown')
        recent = [m for m in node.mux if time.monotonic() - m[0] < 0.2]
        if speed > 0 and recent and safety.taken_over(node.sent, recent[-1][1:]):
            raise Aborted('someone else is driving (the joystick or the keyboard)')
        if fresh and math.isfinite(v) and abs(v) < STOPPED:
            self.still_since = self.still_since or node.seconds()
        else:
            self.still_since = None

    def still(self):
        return (self.still_since is not None
                and self.node.seconds() - self.still_since >= ct.STILL_FOR)

    def roll_up(self, roll):
        """Roll at `roll` m/s: True once the wheels turn at half of it for 0.5 s."""
        self.rolling = True
        since = None
        end = time.monotonic() + ROLL_UP
        while time.monotonic() < end:
            self.tick(roll)
            v = self.node.v['speed']
            if math.isfinite(v) and abs(v) >= 0.5 * roll:
                since = since or time.monotonic()
                if time.monotonic() - since >= 0.5:
                    return
            else:
                since = None
        raise Aborted(f'the wheels would not turn at {roll:g} m/s within {ROLL_UP:g} s')

    def stop_car(self):
        """Zeros until still; not still within 5 s: the e-stop (true only). True if still."""
        if not self.rolling:
            return True
        node = self.node
        end = time.monotonic() + STOP_TIME
        while time.monotonic() < end:
            try:
                self.tick(0.0, estop_ok=True, stale_ok=True)
            except Aborted:
                pass
            if self.still():
                break
        for _ in range(5):
            node.send(0.0, 0.0, self.wheelbase)
            time.sleep(TICK)
        self.rolling = False
        if self.still():
            return True
        if not self.fresh_odom() and not self.real:
            return True    # the sim's speed paused with it: nothing to see, nothing to press
        self.say('NOT STOPPED: E-STOP NOW. The car is not still 5 s after the stop; '
                 'publishing /safety/estop true and sending zeros')
        end = time.monotonic() + ESTOP_TIME
        while time.monotonic() < end:
            node.press_estop()
            node.send(0.0, 0.0, self.wheelbase)
            rclpy.spin_once(node, timeout_sec=0.05)
        self.say('The e-stop stays set: clear it by hand once the car is safe '
                 '(pixi run clear estop)')
        return False

    def roll_until(self, roll, seconds, word, **kw):
        """Roll until word() returns a who-said-it, or `seconds`; return (s, who) or None."""
        t0 = self.node.seconds()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick(roll, **kw)
            who = word()
            if who:
                return self.node.seconds() - t0, who
        return None

    def zeros_until_still(self, seconds, **kw):
        """Zeros until still; return s from now to still, or None."""
        t0 = self.node.seconds()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick(0.0, estop_ok=True, **kw)
            if self.still():
                return self.still_since - t0
        return None

    def ask_go(self, step):
        """On the car: say what the person will do, and wait for Enter. False: skipped."""
        self.say(f'A person: when the wheels turn, {step.ask}.')
        answer = ask('Press Enter to start this step (or type skip): ')
        return answer is not None and answer.strip().lower() != 'skip'

    def wait_watchdog_ok(self, name, seconds):
        return self.node.spin_for(seconds, lambda: self.node.state() is not None
                                  and not ct.named(self.node.state(), name))

    # The steps.

    def sensor_fault(self, step, sim):
        node, sensor = self.node, step.sensor
        limit = ct.limit_for(sensor, self.inputs)
        if limit is None:
            return ct.Result(step, ct.Status.FAIL, f'sensor_watchdog.yaml has no {sensor} '
                             'input: nothing would say it is lost (a gap to fill)')
        if not self.wait_watchdog_ok(sensor, MEND_TIME):
            return ct.Result(step, ct.Status.FAIL, 'the sensor watchdog is silent, or already '
                             f'names {sensor} before the fault: {node.state()}')
        if not sim and not self.ask_go(step):
            return ct.Result(step, ct.Status.SKIPPED, 'the person typed skip')
        roll = min(ct.ROLL, self.cap)
        self.roll_up(roll)
        if sim:
            if not self.set_fault(sensor, 'stop'):
                return ct.Result(step, ct.Status.FAIL, f'Ravensim refused fault.{sensor} stop')
            self.say(f'fault.{sensor} stop: Ravensim stopped sending it')
        else:
            self.say(f'NOW: {step.ask}. Waiting up to {ct.MANUAL_WAIT:g} s for the watchdog.')
        t_fault = node.seconds()
        wait = limit + 5.0 if sim else ct.MANUAL_WAIT + limit
        said = self.roll_until(roll, wait, lambda: (f'the sensor watchdog ("{node.state()}")'
                                                    if ct.named(node.state(), sensor) else None))
        # The tests' slot outranks the watchdog's lock (30) by design: navigation and the AI
        # slot are what it holds. So the runner stops on its word, as they would be held.
        t_word = node.seconds()
        still = self.zeros_until_still(STOP_TIME + ct.STILL_AFTER)
        counted = node.samples.get(COUNTED.get(sensor, sensor))
        last = [t for t in (counted.times if counted else []) if t <= t_word]
        t_last = last[-1] if last and last[-1] >= t_fault - limit else t_fault
        since = 'its last message' if last else 'the fault'
        note = ("the tests outrank the watchdog's lock (it holds navigation and the AI "
                'slot), so the runner stopped on its word')
        if said is None:
            seen = ct.Seen(since, None, '', None, note)
        else:
            base = t_word - t_last
            seen = ct.Seen(since, base, said[1], None if still is None else base + still, note)
        status, detail = ct.judge_fault(seen, limit, ct.STILL_AFTER)
        mended = self.mend(step, sim)
        if mended:
            detail += f'; {mended}'
            status = ct.Status.FAIL
        return ct.Result(step, status, detail)

    def mend(self, step, sim):
        """Put the sensor back; return what went wrong, or ''."""
        if sim:
            if not self.set_fault(step.sensor, 'ok'):
                return f'Ravensim refused fault.{step.sensor} ok'
        else:
            ask(f'Put it back ({step.sensor}: plug it in, start its driver). Press Enter: ')
        if not self.wait_watchdog_ok(step.sensor, MEND_TIME):
            return f'the watchdog still names {step.sensor} {MEND_TIME:g} s after: ' \
                   f'{self.node.state()}'
        return ''

    def set_fault(self, sensor, mode):
        from aav_sim_models.faults import NODES
        client = AsyncParameterClient(self.node, NODES[sensor])
        if not client.wait_for_services(timeout_sec=10.0):
            return False
        sent = client.set_parameters([Parameter(f'fault.{sensor}', value=mode)])
        self.node.spin_for(10.0, sent.done)
        return sent.done() and all(r.successful for r in sent.result().results)

    def heartbeat_sim(self, step):
        """Pause our Ravensim's output mux: its commands and heartbeat stop together."""
        node = self.node
        pid = self.sim.pid_of('output_mux')
        if pid is None:
            return ct.Result(step, ct.Status.FAIL, "no output_mux in our launch's log")
        roll = min(ct.ROLL, self.cap)
        self.roll_up(roll)
        t0 = node.seconds()
        os.kill(pid, signal.SIGSTOP)
        try:
            self.roll_until(roll, ct.HEARTBEAT_LIMIT + 3.0,
                            lambda: 'still' if self.still() else None)
            still = self.still_since - t0 if self.still() else None
            status = node.fresh_status()
        finally:
            os.kill(pid, signal.SIGCONT)
        reopened = node.spin_for(15.0, lambda: node.gate is not None
                                 and node.gate[1].startswith('OPEN'))
        timeout = bool(status and status.faults & VehicleStatus.FAULT_CMD_TIMEOUT)
        cmd_timeout = '; CMD_TIMEOUT' if timeout else ''
        note = ("stopped by the backend's own "
                f'command timeout (sim_adapter, 0.2 s{cmd_timeout}): '
                "on the car that is the ESP32's 100 ms heartbeat check, which Ravensim "
                "doesn't run (ESP32 in QEMU: pixi run esp qemu-check)")
        seen = ct.Seen('the output mux paused (no commands, no heartbeat)', None, '', still,
                       note)
        status_, detail = ct.judge_fault(seen, None, ct.HEARTBEAT_LIMIT, need_word=False)
        if not reopened:
            status_, detail = ct.Status.FAIL, detail + '; the output gate did not reopen'
        return ct.Result(step, status_, detail)

    def heartbeat_car(self, step):
        """Ask a person to pull the Orin's link: the ESP32 must brake, and hold once back."""
        if not self.ask_go(step):
            return ct.Result(step, ct.Status.SKIPPED, 'the person typed skip')
        node = self.node
        roll = min(ct.ROLL, self.cap)
        self.roll_up(roll)
        self.say(f'NOW: {step.ask}. Leave it out until asked.')
        t0 = node.seconds()
        stopped = None
        end = time.monotonic() + ct.MANUAL_WAIT
        while time.monotonic() < end and stopped is None:
            self.tick(roll, stale_ok=True)
            if self.still():
                stopped = self.still_since - t0
        self.say('Plug the cable back in now. Keep away from the wheels: the runner still '
                 'asks them to roll, to show the ESP32 holds its brake.')
        answer = Waiting('Press Enter once it is back in: ')
        end = time.monotonic() + ct.MANUAL_WAIT
        while not answer.done:
            if time.monotonic() > end:
                raise Aborted(f'no Enter in {ct.MANUAL_WAIT:g} s: is the cable back in?')
            self.tick(roll, stale_ok=True)
        back = time.monotonic() + 10.0
        while time.monotonic() < back and not self.fresh_odom():
            self.tick(roll, stale_ok=True)
        top = None
        end = time.monotonic() + ct.LATCH_HOLD
        while time.monotonic() < end:
            self.tick(roll, stale_ok=True)
            v = node.v['speed']
            if self.fresh_odom() and math.isfinite(v):
                top = max(top or 0.0, abs(v))
        status = node.fresh_status()
        self.zeros_until_still(STOP_TIME, stale_ok=True)
        status_, detail = ct.judge_heartbeat_car(
            stopped, top, None if status is None else status.armed,
            None if status is None else status.faults, VehicleStatus.FAULT_CMD_TIMEOUT)
        return ct.Result(step, status_, detail)

    def words(self, before):
        """Who says why now, against `before` (named inputs, status), or None."""
        node = self.node
        state = node.state()
        new = [n for n in ct.named_inputs(state) if n not in before[0]] \
            if state and state.startswith('STOP') else []
        if new:
            return f'the sensor watchdog ("{state}")'
        if node.estop:
            return '/safety/estop'
        status = node.fresh_status()
        if status is not None:
            faults = status.faults & ~before[1]
            if status.estop_active:
                return '/vehicle/status (e-stop active)'
            if faults:
                return f'/vehicle/status (faults {faults})'
            if before[2] and not status.armed:
                return '/vehicle/status (disarmed)'
        if node.gate and not node.gate[1].startswith('OPEN'):
            return f'the output gate ("{node.gate[1]}")'
        if not self.fresh_odom():
            return 'the wheel speed went silent'
        return None

    def baseline(self):
        node = self.node
        status = node.fresh_status()
        return (ct.named_inputs(node.state()), status.faults if status else 0,
                bool(status and status.armed))

    def manual_fault(self, step):
        """Ask a person to pull a cable (a Pi's link, any cable): what stops it, how soon."""
        if not self.ask_go(step):
            return ct.Result(step, ct.Status.SKIPPED, 'the person typed skip')
        roll = min(ct.ROLL, self.cap)
        before = self.baseline()
        self.roll_up(roll)
        self.say(f'NOW: {step.ask}.')
        said = self.roll_until(roll, ct.MANUAL_WAIT, lambda: self.words(before),
                               stale_ok=True, estop_ok=True)
        still = self.zeros_until_still(STOP_TIME + ct.STILL_AFTER, stale_ok=True)
        seen = ct.Seen('the person pulled it (as the runner saw: the first word)',
                       0.0 if said else None, said[1] if said else '',
                       None if still is None else still,
                       'the time from the pull itself is unknown: a person did it')
        status, detail = ct.judge_fault(seen, None, ct.STILL_AFTER)
        ask('Put it back. Press Enter: ')
        return ct.Result(step, status, detail)

    def estop(self, step, sim):
        """Press the e-stop: in --sim the runner (true only); on the car a person."""
        node = self.node
        if not sim and not self.ask_go(step):
            return ct.Result(step, ct.Status.SKIPPED, 'the person typed skip')
        roll = min(ct.ROLL, self.cap)
        self.roll_up(roll)
        if sim:
            self.say('The runner presses /safety/estop (true), standing in for the person')
        else:
            self.say(f'NOW: {step.ask}.')
        t0 = node.seconds()
        pressed = None
        end = time.monotonic() + (5.0 if sim else ct.MANUAL_WAIT)
        while time.monotonic() < end and not self.still():
            self.tick(roll, estop_ok=True, stale_ok=not sim, press=sim)
            status = node.fresh_status()
            if pressed is None and (node.estop or (status and status.estop_active)):
                pressed = node.seconds() - t0
        if not sim and pressed is None and not self.still():
            raise Aborted('nothing changed: was the e-stop pressed?')
        if not self.still():
            self.zeros_until_still(STOP_TIME, stale_ok=True)
        t_still = self.still_since - t0 if self.still() else None
        status = node.fresh_status()
        heard = [w for w, on in (('/safety/estop', node.estop),
                                 ('/vehicle/status (e-stop active)',
                                  bool(status and status.estop_active))) if on]
        said = ' and '.join(heard) or 'no topic (the Kar-Tech cuts the power on its own?)'
        note = (f'heard: {said}; the e-stop stays set: clear it by hand '
                '(pixi run clear estop)')
        start = pressed if pressed is not None else 0.0
        seen = ct.Seen('the e-stop', None, '',
                       None if t_still is None else max(0.0, t_still - start), note)
        return ct.Result(step, *ct.judge_fault(seen, None, ct.ESTOP_LIMIT, need_word=False))

    # --- after ------------------------------------------------------------------------

    def report(self, started, where):
        text = ct.table(self.results)
        self.say()
        self.say(text)
        md = ct.markdown(self.results, where, started, self.yaml, self.notes)
        with open(os.path.join(self.folder, 'report.md'), 'w') as f:
            f.write(md)
        with open(os.path.join(self.folder, 'report.yaml'), 'w') as f:
            steps = [{'step': r.step.name, 'number': r.step.number, 'status': r.status.value,
                      'detail': r.detail, 'files': r.files} for r in self.results]
            yaml.safe_dump({'started': started, 'where': where,
                            'counts': dict(ct.counts(self.results)), 'steps': steps},
                           f, sort_keys=False)
        self.say(f'Saved in {self.folder} (report.md)')


class HallwayTest(CarTest):
    """Phase B: the hallway's passes, each an actuator_test hall-* child; then the fits."""

    def __init__(self, node, args, folder, sim, out):
        super().__init__(node, args, folder, sim, out)
        self.hall = None
        self.passes = []          # what each pass did, for session.yaml and report.md
        self.session = None

    def prepare(self):
        """Every check before anything moves; returns why not, or None."""
        node, a = self.node, self.args
        if self.sim:
            self.say('Waiting for our Ravensim: /system/source SIM, the speed, the output '
                     'gate OPEN ...')
            up = node.spin_for(SIM_UP, lambda: (
                safety.source_kind(node.sources) is safety.Kind.SIM and self.fresh_odom()
                and node.gate is not None and node.gate[1].startswith('OPEN')))
            if not up:
                return (f'Ravensim did not come up in {SIM_UP:.0f} s (see '
                        f'{self.sim.log_path})')
            node.spin_for(3.0)
        else:
            node.spin_for(3.0, lambda: bool(node.sources))
            node.spin_for(1.0)
        kind = safety.source_kind(node.sources)
        d = safety.decide(kind, a.sim, False, 4)
        if not d.allowed:
            return d.reason
        self.real = d.real
        length = a.length
        if length is None:
            if not sys.stdin.isatty():
                return 'the hallway length is needed: --length, or a person to type it'
            typed = ask('The hallway, measured with a tape: metres from the start line to '
                        'the far wall: ')
            try:
                length = float(typed or '')
            except ValueError:
                return f'not a length: {typed!r}'
        hall, why = safety.hallway_limits(length, a.margin, min(d.cap, safety.REAL_CAP))
        if hall is None:
            return why
        self.hall = hall
        self.say(f'Hallway {hall.length:g} m, margin {hall.margin:g} m: the car stands still '
                 f'before {hall.room:.1f} m; top speed {hall.top:g} m/s (speed up '
                 f'{safety.HALL_ACCEL:g} m/s^2, stop {safety.HALL_DECEL:g} m/s^2, '
                 f'{safety.HALL_REACTION:g} s to react: {hall.needed(hall.top):.1f} m)')
        if d.real:
            self.say()
            self.say('THE REAL CAR, ON THE GROUND, IN THE HALLWAY. Check, out loud:')
            for item in safety.HALLWAY_CHECKLIST:
                self.say(f'  [ ] {item}')
            self.say('Each pass asks again before it moves. To stop at any time: Ctrl-C '
                     'here, the e-stop, or the joystick deadman.')
            if not sys.stdin.isatty():
                return 'the real car needs a person at the keyboard to confirm'
            typed = ask('Type "hallway" to start (anything else cancels): ')
            if not safety.confirmed(typed, 'hallway'):
                return 'cancelled: nothing moved'
        return None

    def run(self):
        a = self.args
        why = self.prepare()
        if why:
            self.say(f'REFUSED: {why}')
            self.notes.append(f'Refused before any pass: {why}')
            return
        both = not (a.one_way or a.sim)
        if a.sim and not a.one_way:
            self.notes.append('Ravensim: nobody turns the car round, and the floor is flat: '
                              'every pass faces "there"')
        stopped = None
        for p in ct.hall_passes(a.kinds, a.repeats, both):
            entry = {'pass': p.folder, 'test': p.test, 'direction': p.direction}
            self.passes.append(entry)
            if stopped:
                entry.update(status='NOT RUN', detail=stopped)
                continue
            if self.real and not self.place(p):
                entry.update(status='SKIPPED', detail='skipped by the person')
                continue
            self.say()
            self.say(f'=== Pass {p.number}: {p.test}, facing {p.direction} ===')
            sub = os.path.join(self.folder, p.folder)
            os.makedirs(sub, exist_ok=True)
            cmd = [sys.executable, '-m', 'aav_actuator_tests.actuator_test',
                   *ct.hall_args(p, self.hall.length, self.hall.margin, bool(a.sim), sub)]
            self.say(f'$ actuator-test {" ".join(cmd[3:])}')
            try:
                code = self.child(cmd, os.path.join(sub, 'output.log'))
            except KeyboardInterrupt:
                entry.update(status='FAIL', detail='Ctrl-C')
                stopped = 'Ctrl-C'
                continue
            if code == 0:
                entry.update(status='PASS', detail='ran to the end')
            else:
                entry.update(status='FAIL', detail=ct.TOOL_CODES.get(code, f'exit {code}'))
                stopped = f'pass {p.number} ({p.test}): {entry["detail"]}: the session stops'

    def place(self, p):
        """On the car: the person puts it on the start line, facing the way. False: skip."""
        if p.number == 1:
            return True
        turn = ('turn it round' if p.direction != self.passes[-2]['direction']
                else 'facing the same way')
        typed = ask(f'Put the car back on the start line, {turn} (now facing {p.direction}): '
                    'push it, or drive it with the joystick. Then Enter (s skips this pass, '
                    'q stops): ')
        if (typed or '').strip().lower() == 'q':
            raise KeyboardInterrupt
        return (typed or '').strip().lower() != 's'

    def report(self, started, where):
        a = self.args
        info = {'started': started, 'where': where, 'sim': bool(a.sim),
                'length': self.hall.length if self.hall else a.length,
                'margin': self.hall.margin if self.hall else a.margin,
                'room': self.hall.room if self.hall else None,
                'top': self.hall.top if self.hall else None, 'repeats': a.repeats,
                'mass': float(self.cfg.get('vehicle', {}).get('mass', float('nan'))),
                'passes': self.passes, 'notes': self.notes}
        if a.sim:
            overrides = ct.sim_overrides(a.sim_actuators)
            info.update(sim_actuators=dict(overrides), wheel_noise=a.wheel_noise,
                        wheel_slip=a.wheel_slip,
                        truth=hallway.with_truth(reports.current_values(self.cfg), overrides))
        with open(os.path.join(self.folder, 'session.yaml'), 'w') as f:
            yaml.safe_dump(info, f, sort_keys=False)
        self.say()
        self.say('Working out the transfer functions (fits, error bars, the model replayed) ...')
        self.session = hallway.analyse(self.folder, info)
        hallway.write(self.session)
        self.say(hallway.text_table(self.session))
        self.say(f'Saved in {self.folder} (report.md, vehicle-yaml.txt, overlays/)')

    def failed(self):
        return any(e.get('status') == 'FAIL' for e in self.passes) or not self.passes


class Waiting:
    """A prompt answered on another thread, so the runner can keep its 50 Hz meanwhile."""

    def __init__(self, prompt):
        self.done = False
        self.text = None
        threading.Thread(target=self.read, args=(prompt,), daemon=True).start()

    def read(self, prompt):
        self.text = ask(prompt)
        self.done = True


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        return None


def watchdog_inputs():
    """Return sensor_watchdog.yaml's inputs (name -> its settings), {} if not found."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('aav_safety'), 'config',
                            'sensor_watchdog.yaml')
        with open(path) as f:
            params = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    except (LookupError, OSError, KeyError, TypeError):
        return {}
    return {k: v for k, v in params.items() if isinstance(v, dict)}


def pick_domain():
    """--sim: our own ROS domain, nobody else's (AAV_SIM_DOMAIN, or a free one)."""
    if 'AAV_SIM_DOMAIN' in os.environ:
        return os.environ['AAV_SIM_DOMAIN']
    from aav_io_mux.domains import free_domain
    return str(free_domain())


def main(argv=None):
    args = ct.parse(sys.argv[1:] if argv is None else argv)
    if args.analyse:
        return hallway.main([args.analyse])
    now = datetime.datetime.now()
    phase = 'hallway' if args.hallway else 'stands'
    folder = ct.report_folder(args.out, args.sim, now, phase)
    os.makedirs(folder, exist_ok=True)
    out = Tee(os.path.join(folder, 'run.log'))
    started = now.strftime('%Y-%m-%d %H:%M:%S')
    sim = None
    if args.sim:
        os.environ['ROS_DOMAIN_ID'] = pick_domain()
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        launch = (ct.sim_launch_args(ct.sim_overrides(args.sim_actuators), args.wheel_noise,
                                     args.wheel_slip)
                  if args.hallway else SIM_ARGS)
        sim = SimLaunch(folder, out.say, launch)
        sim.start()
    if args.hallway:
        where = (f"Ravensim's empty world, a pretend {args.length:g} m hallway (ROS domain "
                 f"{os.environ.get('ROS_DOMAIN_ID')}, this computer only)" if args.sim
                 else 'the real car, in the hallway')
    else:
        where = (f"Ravensim's road world, started by the runner (ROS domain "
                 f"{os.environ.get('ROS_DOMAIN_ID')}, this computer only)" if args.sim
                 else 'the real car, on stands')
    out.say(f'car-test --{phase}: {where}. Report folder: {folder}')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CarTestNode(bool(args.sim))
    run = (HallwayTest if args.hallway else CarTest)(node, args, folder, sim, out)
    try:
        run.run()
    except KeyboardInterrupt:
        out.say('Ctrl-C: stopping')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            run.stop_car()
            run.report(started, where)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
            if sim is not None:
                sim.stop()
    if args.hallway:
        return 5 if run.failed() else 0
    return 5 if ct.failed(run.results) else 0


if __name__ == '__main__':
    sys.exit(main())

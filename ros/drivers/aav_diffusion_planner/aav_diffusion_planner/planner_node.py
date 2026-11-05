"""
diffusion_planner: a model draws the path; the guard checks it; the lattice is the fallback.

A trial planner (planner:=diffusion, docs/designs/2026-11-02_diffusion-planner.md), off by
default and refused on the car. It stands in for planner_server's ComputePathToPose, so
bt_navigator's trees don't change; planner_server keeps running with the lattice, its
own action renamed, and answers the fallback at the same time.

For each plan: ask the lattice (its renamed action) for the same path; crop the global
costmap round the start and goal; the model worker (worker.py, in the learn
environment's Python, ONNX Runtime) draws `samples` paths; the guard (guard.py, G1-G8)
checks each; the cheapest that passes, if within `budget_ms`, is the plan. Else the
lattice's. Neither: Nav2's NO_VALID_PATH. No model file, no learn environment, a model
error: always the lattice's, and the status says why.

Serves      compute_path_to_pose (nav2_msgs/action/ComputePathToPose), as planner_server
Calls       compute_path_to_pose_lattice (planner_server, remapped by navigation.launch.py)
Subscribes  /global_costmap/costmap (nav_msgs/OccupancyGrid), TF map -> base_link
Publishes   /plan (nav_msgs/Path: the path it returned, as planner_server publishes);
            /drivers/nav/diffusion/status (diagnostic_msgs/DiagnosticStatus: each plan's
            source, why, timings, the guard's refusals by check, and the running share
            from the model)
Parameters  model ('' = <repo>/models/diffusion/planner.onnx), learn_python ('' =
            <repo>/.pixi/envs/learn/bin/python), provider (ONNX Runtime's:
            CPUExecutionProvider, CoreMLExecutionProvider...), samples (16), budget_ms
            (150), reversing, footprint (Nav2's string), min_radius (m), goal_xy,
            goal_yaw, max_cusps, max_reverse, length_ratio, lattice_wait (s)
"""

from collections import Counter
import math
import os
import select
import subprocess
import threading
import time

from aav_diffusion_planner import guard, window, wire
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

WORKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'worker.py')
MODEL = os.path.join('models', 'diffusion', 'planner.onnx')
LEARN_PYTHON = os.path.join('.pixi', 'envs', 'learn', 'bin', 'python')
KEEP_ENV = ('HOME', 'PATH', 'LANG', 'TMPDIR')
NO_VALID_PATH = ComputePathToPose.Result.NO_VALID_PATH
TF_ERROR = ComputePathToPose.Result.TF_ERROR


def repo_path(given, default):
    """Return `given` if set, else <PIXI_PROJECT_ROOT or the start folder>/default."""
    if given:
        return os.path.abspath(os.path.expanduser(given))
    return os.path.join(os.environ.get('PIXI_PROJECT_ROOT') or os.getcwd(), default)


class Worker:
    """The model worker: a child program, one request at a time, with a deadline."""

    def __init__(self, python, model, provider):
        self.proc = subprocess.Popen(
            [python, '-I', WORKER], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0,
            env={k: os.environ[k] for k in KEEP_ENV if k in os.environ})
        self.proc.stdin.write(wire.encode_settings({'model': model, 'provider': provider}))
        self.seq = 0
        self.pending = 0      # replies still to come for requests given up on

    def alive(self):
        return self.proc.poll() is None

    def _readable(self, seconds):
        ready, _, _ = select.select([self.proc.stdout], [], [], max(0.0, seconds))
        return bool(ready)

    def sample(self, crop, cond, noise, seconds):
        """Return (paths or None, model ms or None, why). None paths: no answer in time."""
        # Replies to requests given up on come first: read past them (briefly).
        while self.pending:
            if not self._readable(1.0):
                return None, None, 'the worker is still busy with an earlier plan'
            wire.read_reply(self.proc.stdout)
            self.pending -= 1
        self.seq += 1
        self.proc.stdin.write(wire.encode_request(self.seq, crop, cond, noise))
        if not self._readable(seconds):
            self.pending += 1
            return None, None, 'no answer in the budget'
        seq, paths, ms = wire.read_reply(self.proc.stdout)
        if paths is None:
            return None, ms, 'the model failed (the worker logs why)'
        return paths, ms, ''

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5.0)
        except (OSError, subprocess.TimeoutExpired):
            self.proc.kill()


def yaw_of(pose):
    q = pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


class DiffusionPlanner(Node):

    def __init__(self):
        super().__init__('diffusion_planner')
        p = self.declare_parameter
        self.model = repo_path(p('model', '').value, MODEL)
        self.python = repo_path(p('learn_python', '').value, LEARN_PYTHON)
        self.provider = p('provider', 'CPUExecutionProvider').value
        self.samples = int(p('samples', 16).value)
        self.budget_ms = float(p('budget_ms', 150.0).value)
        self.lattice_wait = float(p('lattice_wait', 8.0).value)
        polygon = yaml.safe_load(p('footprint', '[[1.85, 0.55], [1.85, -0.55], '
                                   '[-0.25, -0.55], [-0.25, 0.55]]').value)
        self.footprint = guard.footprint_points(polygon)
        self.limits = guard.Limits(
            min_radius=float(p('min_radius', 2.73).value),
            reversing=bool(p('reversing', False).value),
            goal_xy=float(p('goal_xy', 1.0).value), goal_yaw=float(p('goal_yaw', 0.75).value),
            max_cusps=int(p('max_cusps', 4).value),
            max_reverse=float(p('max_reverse', 15.0).value),
            length_ratio=float(p('length_ratio', 1.5).value))
        self.rng = np.random.default_rng(int(p('seed', 0).value))

        self.grid = None
        self.plans = Counter()      # source -> plans, since the start
        self.refused = Counter()    # check -> samples refused, since the start
        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap,
                                 latched, callback_group=MutuallyExclusiveCallbackGroup())
        self.plan_pub = self.create_publisher(Path, '/plan', latched)
        self.status_pub = self.create_publisher(DiagnosticStatus, '/drivers/nav/diffusion/status',
                                                10)
        group = ReentrantCallbackGroup()
        self.lattice = ActionClient(self, ComputePathToPose, 'compute_path_to_pose_lattice',
                                    callback_group=group)
        self.worker = None
        self.worker_lock = threading.Lock()
        self.start_worker()
        ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.execute,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.get_logger().info(
            f'diffusion_planner: {self.samples} samples, budget {self.budget_ms:.0f} ms, '
            f'reversing {self.limits.reversing}, model {self.model}, '
            f'worker {"up" if self.worker else "none: every plan is the lattice"}')

    def start_worker(self):
        why = ('no learn environment at ' + self.python if not os.path.isfile(self.python)
               else 'no model at ' + self.model if not os.path.isfile(self.model) else '')
        if why:
            self.get_logger().error(f'{why}: the lattice plans alone (pixi install --locked '
                                    '-e learn; pixi run diffusion train)')
            return
        self.worker = Worker(self.python, self.model, self.provider)
        # The first call loads the model: not in a real plan's budget.
        noise = np.zeros((self.samples, window.POINTS, window.CHANNELS), np.float32)
        paths, _, why = self.worker.sample(np.zeros((window.SIZE, window.SIZE)),
                                           np.zeros(window.COND), noise, 60.0)
        if paths is None:
            self.get_logger().error(f'the model worker: {why}; the lattice plans alone')
            self.worker.close()
            self.worker = None

    def on_costmap(self, msg):
        if abs(yaw_of(msg.info.origin)) > 1e-6:
            self.get_logger().error('a rotated costmap: not handled; the lattice plans alone',
                                    throttle_duration_sec=10.0)
            self.grid = None
            return
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid = window.Grid(data, msg.info.origin.position.x, msg.info.origin.position.y,
                                msg.info.resolution)

    def start_of(self, request):
        """Return the start (x, y, yaw) in map: the request's, or the car's from TF."""
        if request.use_start:
            return (request.start.pose.position.x, request.start.pose.position.y,
                    yaw_of(request.start.pose))
        t = self.tf.lookup_transform('map', 'base_link', Time(),
                                     timeout=Duration(seconds=0.5)).transform
        return (t.translation.x, t.translation.y,
                2.0 * math.atan2(t.rotation.z, t.rotation.w))

    def ask_lattice(self, request, start):
        """Send the lattice the same plan, from the same start; return (event, holder)."""
        goal = ComputePathToPose.Goal()
        goal.goal, goal.planner_id, goal.use_start = request.goal, request.planner_id, True
        goal.start.header.frame_id = 'map'
        goal.start.pose.position.x, goal.start.pose.position.y = start[0], start[1]
        goal.start.pose.orientation.z = math.sin(start[2] / 2.0)
        goal.start.pose.orientation.w = math.cos(start[2] / 2.0)
        done, out = threading.Event(), {}

        def on_result(future):
            out['result'] = future.result().result
            done.set()

        def on_handle(future):
            handle = future.result()
            if not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        if not self.lattice.wait_for_server(timeout_sec=1.0):
            done.set()
        else:
            self.lattice.send_goal_async(goal).add_done_callback(on_handle)
        return done, out

    def run_model(self, start, goal, t0):
        """Return (the samples in the map (K, POINTS, 3) or None, the grid, model ms, why)."""
        grid = self.grid
        if self.worker is None or grid is None:
            return None, grid, None, ('no model' if self.worker is None else 'no costmap yet')
        win = window.window_for(start, goal)
        crop = window.crop(grid, win)
        cond = window.cond(win, start, goal, self.limits.reversing)
        noise = self.rng.standard_normal((self.samples, window.POINTS, window.CHANNELS))
        left = self.budget_ms / 1000.0 - (time.monotonic() - t0)
        with self.worker_lock:
            out, ms, why = self.worker.sample(crop, cond, noise.astype(np.float32), left)
            if out is None and not self.worker.alive():
                self.get_logger().error('the model worker died: the lattice plans alone')
                self.worker = None
        if out is None:
            return None, grid, ms, why
        paths, _ = window.decode_paths(win, out)
        return [window.smooth(p) for p in paths], grid, ms, ''

    def execute(self, handle):
        request = handle.request
        t0 = time.monotonic()
        result = ComputePathToPose.Result()
        try:
            start = self.start_of(request)
        except TransformException as e:
            handle.abort()
            result.error_code, result.error_msg = TF_ERROR, f'no map -> base_link: {e}'
            return result
        goal = (request.goal.pose.position.x, request.goal.pose.position.y,
                yaw_of(request.goal.pose))
        lattice_done, lattice = self.ask_lattice(request, start)
        paths, grid, model_ms, why = self.run_model(start, goal, t0)
        lattice_done.wait(self.lattice_wait)
        lat = lattice.get('result')
        lattice_ok = lat is not None and lat.error_code == 0 and len(lat.path.poses) > 1
        lattice_xy = (np.array([(s.pose.position.x, s.pose.position.y, 0.0)
                                for s in lat.path.poses]) if lattice_ok else None)
        picked, facts, refusals, passed = None, None, Counter(), 0
        if paths is not None:
            best, facts, refusals, passed = guard.choose(
                paths, grid, start, goal, self.limits, self.footprint,
                guard.length(lattice_xy) if lattice_ok else None)
            picked = best
        guard_ms = 1000.0 * (time.monotonic() - t0)
        source, reason = guard.decide(paths is not None, picked is not None, guard_ms,
                                      self.budget_ms, lattice_ok)
        if paths is None:
            reason = f'{reason}: {why}'
        self.plans[source] += 1
        self.refused.update(refusals)
        took = time.monotonic() - t0
        if source == 'none':
            handle.abort()
            result.error_code = NO_VALID_PATH
            result.error_msg = (lat.error_msg if lat is not None and lat.error_msg
                                else 'diffusion_planner: neither the model nor the lattice')
        else:
            if source == 'model':
                # The lattice's stamp (the costmap's time, as planner_server stamps it):
                # a stamp ahead of TF made controller_server abort, "transform data too
                # old" (2 Nov, Gazebo). With no lattice path, zero: TF's latest.
                result.path = self.to_path(facts['dense'])
                if lattice_ok:
                    result.path.header.stamp = lat.path.header.stamp
            else:
                result.path = lat.path        # as planner_server gave it, stamp and all
            handle.succeed()
            self.plan_pub.publish(result.path)
        result.planning_time = Duration(seconds=took).to_msg()
        self.publish_status(source, reason, model_ms, guard_ms, took, refusals, passed,
                            paths, facts if source == 'model' else None, lattice_xy, lat)
        return result

    def to_path(self, dense):
        path = Path()
        path.header.frame_id = 'map'
        for x, y, yaw in dense:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def publish_status(self, source, reason, model_ms, guard_ms, took, refusals, passed,
                       paths, facts, lattice_xy, lat):
        total = sum(self.plans.values())
        msg = DiagnosticStatus()
        msg.name = 'diffusion_planner'
        msg.level = (DiagnosticStatus.OK if source == 'model' else
                     DiagnosticStatus.WARN if source == 'lattice' else DiagnosticStatus.ERROR)
        msg.message = f'{source}: {reason}'
        values = {
            'source': source,
            'why': reason,
            'model_ms': f'{model_ms:.1f}' if model_ms is not None else '',
            'model_and_guard_ms': f'{guard_ms:.1f}',
            'total_ms': f'{1000.0 * took:.1f}',
            'samples': str(len(paths) if paths is not None else 0),
            'samples_passed': str(passed),
            'lattice_ms': (f'{lat.planning_time.sec * 1e3 + lat.planning_time.nanosec * 1e-6:.1f}'
                           if lat is not None else ''),
            'plans': str(total),
            'from_model': str(self.plans['model']),
            'share_from_model': f'{self.plans["model"] / total:.3f}' if total else '',
        }
        for check in guard.CHECKS:
            values[f'refused_{check}'] = str(refusals.get(check, 0))
            values[f'refused_{check}_total'] = str(self.refused.get(check, 0))
        if facts is not None:
            values['length_m'] = f'{facts["length"]:.1f}'
            values['cusps'] = str(facts['cusps'])
            if lattice_xy is not None:
                values['lattice_length_m'] = f'{guard.length(lattice_xy):.1f}'
                values['gap_to_lattice_m'] = f'{guard.gap(facts["dense"], lattice_xy):.2f}'
        msg.values = [KeyValue(key=k, value=v) for k, v in values.items() if v != '']
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.worker is not None:
            self.worker.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DiffusionPlanner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

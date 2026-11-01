"""
language_drive: the drive tools' ROS side, one rclpy node spinning on its own thread.

Subscribes  /localization/odom/local  nav_msgs/Odometry (speed, odom pose)
            /sensors/gps/fix          sensor_msgs/NavSatFix (best effort; lat/lon, fix kind)
            /safety/estop             std_msgs/Bool (refuse to move while true)
            /vehicle/status           aav_interfaces/VehicleStatus (armed, e-stop)
            every twist_mux input and lock in aav_command_mux's twist_mux.yaml (/drivers/*/cmd_vel,
            /perception/stop_sign): only their arrival times, to tell who drives
            /diagnostics              diagnostic_msgs/DiagnosticArray: twist_mux's own word,
                                      once a second, on each lock (is the e-stop on?). The
                                      only place a late starter can learn it: /safety/estop
                                      is sent a few times and its senders go away
            /tf, /tf_static           the map pose (map -> base_link), when there is one
            /safety/geofence/fence    geometry_msgs/PolygonStamped, transient local: a move
                                      that would end outside it is refused. Only people
                                      clear it: this never calls /safety/geofence/clear
Publishes   /safety/estop             std_msgs/Bool: ONLY true, five times in 0.5 s (estop()).
                                      Never false: a person clears the e-stop
Calls       /scripted_drive           aav_interfaces/action/ScriptedDrive (drive, turn, stop)
            /follow_gps_waypoints     nav2_msgs/action/FollowGPSWaypoints (go_to_gps)
            /ego/get_parameters       ego's limits: max_speed, wheelbase, steering...

The rules live in drive_tools.py and safety.py; this file only moves messages.
"""

from __future__ import annotations

import itertools
import math
import os
import threading
import time
from typing import Dict, List, Optional

from aav_interfaces.action import ScriptedDrive
from aav_interfaces.msg import VehicleStatus
from aav_language import safety
from aav_language.drive_maths import DriveGoal
from aav_language.drive_tools import BackendError, CarState
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PolygonStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import tf2_ros
import yaml

DIAG_WAIT = 3.0     # s to wait at start for twist_mux's first report (it sends one a second)
DIAG_FRESH = 3.5    # s: an older report means twist_mux has gone quiet
EGO_PARAMS = ('max_speed', 'max_reverse_speed', 'max_acceleration', 'max_deceleration',
              'wheelbase', 'max_steering_angle')
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                GoalStatus.STATUS_ABORTED: 'aborted'}


def load_mux_config():
    """twist_mux's inputs and locks, from the installed aav_command_mux (empty if it isn't)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('aav_command_mux'), 'config',
                            'twist_mux.yaml')
        with open(path, encoding='utf-8') as f:
            return safety.mux_config(yaml.safe_load(f))
    except (LookupError, OSError, KeyError, TypeError):
        return (), ()


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosBackend:
    """The car side of DriveTools, over ROS. Start it once; it spins its own thread."""

    def __init__(self, node_name: str = 'language_drive'):
        if not rclpy.ok():
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(node_name)
        n = self.node
        self.lock = threading.Lock()
        self.odom: Optional[Odometry] = None
        self.odom_at = 0.0
        self.fix: Optional[NavSatFix] = None
        self.estop_value: Optional[bool] = None
        self.status: Optional[VehicleStatus] = None
        self.arrivals: Dict[str, float] = {}
        self.lock_values: Dict[str, tuple] = {}
        self.goals: Dict[str, dict] = {}
        self.ids = itertools.count(1)
        self.ego_cache: Optional[Dict[str, float]] = None
        self.ego_at = -1e9

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(Odometry, '/localization/odom/local', self._on_odom, 10)
        n.create_subscription(NavSatFix, '/sensors/gps/fix', self._on_fix,
                              qos_profile_sensor_data)
        n.create_subscription(Bool, '/safety/estop', self._on_estop, reliable)
        n.create_subscription(VehicleStatus, '/vehicle/status', self._on_status, 10)
        n.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self.fence = None
        n.create_subscription(PolygonStamped, '/safety/geofence/fence', self._on_fence,
                              QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                         durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.diag_locks: Optional[Dict[str, bool]] = None
        self.diag_at = -1e9
        self.mux_inputs, self.mux_locks = load_mux_config()
        for i in self.mux_inputs:
            n.create_subscription(TwistStamped, i.topic,
                                  lambda _m, name=i.name: self._arrived(name), 10)
        for lk in self.mux_locks:
            if lk.topic != '/safety/estop':
                n.create_subscription(Bool, lk.topic,
                                      lambda m, name=lk.name: self._locked(name, m.data),
                                      reliable)
        self.estop_pub = n.create_publisher(Bool, '/safety/estop', reliable)
        self.scripted = ActionClient(n, ScriptedDrive, '/scripted_drive')
        self.gps_action = None
        try:
            from nav2_msgs.action import FollowGPSWaypoints
            self.gps_type = FollowGPSWaypoints
            self.gps_action = ActionClient(n, FollowGPSWaypoints, '/follow_gps_waypoints')
        except ImportError:
            self.gps_type = None
        self.ego_params = n.create_client(GetParameters, '/ego/get_parameters')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, n)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, name='ros', daemon=True)
        self.thread.start()
        # twist_mux reports its locks once a second: wait for one report, so the very first
        # request already knows whether the e-stop is on.
        deadline = time.monotonic() + DIAG_WAIT
        while self.diag_locks is None and time.monotonic() < deadline:
            time.sleep(0.05)

    # --- incoming ---

    def _on_fence(self, msg):
        self.fence = [(p.x, p.y) for p in msg.polygon.points] or None

    def _on_odom(self, msg):
        with self.lock:
            self.odom, self.odom_at = msg, time.monotonic()

    def _on_fix(self, msg):
        with self.lock:
            self.fix = msg

    def _on_estop(self, msg):
        with self.lock:
            self.estop_value = bool(msg.data)
            self.lock_values['estop'] = (bool(msg.data), time.monotonic())

    def _on_status(self, msg):
        with self.lock:
            self.status = msg

    def _on_diagnostics(self, msg):
        locks = safety.mux_locks_from_diagnostics(
            [(s.name, {kv.key: kv.value for kv in s.values}) for s in msg.status])
        if locks is not None:
            with self.lock:
                self.diag_locks, self.diag_at = locks, time.monotonic()

    def _arrived(self, name):
        with self.lock:
            self.arrivals[name] = time.monotonic()

    def _locked(self, name, value):
        with self.lock:
            self.lock_values[name] = (bool(value), time.monotonic())

    # --- Backend ---

    def state(self) -> CarState:
        now = time.monotonic()
        with self.lock:
            odom, odom_at, fix = self.odom, self.odom_at, self.fix
            status, estop = self.status, self.estop_value
            ages = {k: now - t for k, t in self.arrivals.items()}
            locks = {k: (v, now - t) for k, (v, t) in self.lock_values.items()}
            diag = dict(self.diag_locks) if (
                self.diag_locks is not None and now - self.diag_at < DIAG_FRESH) else None
        s = CarState(estop=estop, mux_locks=diag, fence=self.fence)
        for name, value in (diag or {}).items():
            locks[name] = (value, 0.0)          # twist_mux's own word beats ours
        if odom is not None:
            p = odom.pose.pose
            s.speed = odom.twist.twist.linear.x
            s.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))
            s.odom_age = now - odom_at
        if fix is not None and math.isfinite(fix.latitude) and fix.status.status >= 0:
            s.gps, s.gps_status = (fix.latitude, fix.longitude), fix.status.status
        elif fix is not None:
            s.gps_status = fix.status.status
        if status is not None:
            s.vehicle_estop, s.armed = bool(status.estop_active), bool(status.armed)
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            tr = t.transform
            s.map_pose = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass
        s.mux = safety.mux_choice(self.mux_inputs, ages, self.mux_locks, locks)
        return s

    def where(self) -> safety.Where:
        clock = bool(self.node.get_publishers_info_by_topic('/clock'))
        nodes = tuple(name for name, _ns in self.node.get_node_names_and_namespaces())
        return safety.Where(os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', ''),
                            os.environ.get('ROS_STATIC_PEERS', ''), clock, nodes)

    def ego_limits(self) -> Optional[Dict[str, float]]:
        """Ask ego for its limits, once a minute (None when it doesn't answer in 2 s)."""
        if time.monotonic() - self.ego_at < 60.0:
            return self.ego_cache
        self.ego_at = time.monotonic()
        if not self.ego_params.wait_for_service(timeout_sec=2.0):
            self.ego_cache = None
            return None
        req = GetParameters.Request(names=list(EGO_PARAMS))
        res = self._wait(self.ego_params.call_async(req), 2.0)
        if res is None:
            self.ego_cache = None
            return None
        out = {}
        for name, value in zip(EGO_PARAMS, res.values):
            v = value.double_value if value.type == 3 else (
                float(value.integer_value) if value.type == 2 else None)
            if v is not None:
                out[name] = v
        self.ego_cache = out or None
        return self.ego_cache

    def _wait(self, future, timeout: Optional[float]):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def _send(self, client, goal_msg, kind: str) -> str:
        if not client.wait_for_server(timeout_sec=3.0):
            raise BackendError(f'no {kind} action server is running')
        handle = self._wait(client.send_goal_async(goal_msg), 5.0)
        if handle is None:
            raise BackendError(f'the {kind} server did not answer')
        if not handle.accepted:
            raise BackendError(f'the {kind} server rejected the goal (see its log)')
        goal_id = f'{kind}-{next(self.ids)}'
        entry = {'handle': handle, 'done': threading.Event(), 'result': None}
        fut = handle.get_result_async()

        def finished(f, entry=entry):
            r = f.result()
            entry['result'] = r
            entry['done'].set()
        fut.add_done_callback(finished)
        with self.lock:
            self.goals[goal_id] = entry
        return goal_id

    def send_scripted(self, goal: DriveGoal) -> str:
        msg = ScriptedDrive.Goal(
            target_speed=float(goal.target_speed), acceleration=float(goal.acceleration),
            deceleration=float(goal.deceleration), hold_time=float(goal.hold_time),
            hold_distance=float(goal.hold_distance), curvature=float(goal.curvature))
        return self._send(self.scripted, msg, 'scripted_drive')

    def nav_ready(self) -> bool:
        return self.gps_action is not None and self.gps_action.wait_for_server(timeout_sec=2.0)

    def send_gps(self, lat: float, lon: float) -> str:
        if self.gps_action is None:
            raise BackendError('nav2_msgs is not installed')
        from geographic_msgs.msg import GeoPose
        pose = GeoPose()
        pose.position.latitude, pose.position.longitude = lat, lon
        pose.orientation.w = 1.0
        return self._send(self.gps_action, self.gps_type.Goal(gps_poses=[pose]),
                          'follow_gps_waypoints')

    def result(self, goal_id: str, timeout: Optional[float]) -> Optional[dict]:
        with self.lock:
            entry = self.goals.get(goal_id)
        if entry is None or not entry['done'].wait(timeout):
            return None
        r = entry['result']
        out = {'status': STATUS_NAMES.get(r.status, f'status {r.status}')}
        res = r.result
        if hasattr(res, 'distance_travelled'):
            out.update(distance_travelled=res.distance_travelled, duration=res.duration,
                       message=res.message)
        elif hasattr(res, 'missed_waypoints'):
            missed = len(res.missed_waypoints)
            out['message'] = (res.error_msg or
                              ('reached the waypoint' if not missed else 'missed the waypoint'))
            if missed:
                out['status'] = 'aborted'
        return out

    def running(self) -> List[str]:
        with self.lock:
            return [k for k, e in self.goals.items() if not e['done'].is_set()]

    def cancel_all(self) -> List[str]:
        with self.lock:
            live = [(k, e) for k, e in self.goals.items() if not e['done'].is_set()]
        for _k, e in live:
            self._wait(e['handle'].cancel_goal_async(), 3.0)
        return [k for k, _e in live]

    def estop(self) -> None:
        # Like the HUD's button and `pixi run estop`: true, five times in 0.5 s, so one
        # lost message can't keep the car going.
        for _ in range(5):
            self.estop_pub.publish(Bool(data=True))
            time.sleep(0.1)

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            rclpy.try_shutdown()
        except Exception:  # noqa: B902 - the process is ending anyway
            pass

"""
track: a trial path follower in controller_server's place (docs/trials.md).

Action      follow_path                 nav2_msgs/action/FollowPath: bt_navigator's, a new
                                        goal each time the planner replans (the newest
                                        replaces the last without stopping)
Subscribes  localization/odom/local     nav_msgs/Odometry (odom_topic): where the car is in
                                        odom, its speed and yaw rate
            /tf, /tf_static             map -> odom, to move each new path into odom
Publishes   cmd_vel_nav                 geometry_msgs/TwistStamped (Twist with stamped:=
                                        false), at rate_hz while a path runs: (v, v tan(steer)
                                        / L), to Nav2's velocity smoother, collision_monitor,
                                        then /drivers/nav/cmd_vel. v from the shared
                                        speed policy (speed.py), or mpc_full's own plan
            drivers/nav/track/status    diagnostic_msgs/DiagnosticStatus, every tick: the
                                        tracker, loop time, missed ticks, cross track, steering
Serves      nothing else; calls nothing.
Parameters  tracker can be changed while it runs, only with live_swap:=true (sim.launch.py
            sets it; the car never): `ros2 param set /track tracker lqr`. The new tracker
            is built between ticks (one thread), takes the path and its place on it,
            starts its steering from the last command, and the loop statistics reset.

navigation.launch.py tracker:=pure_pursuit|stanley|lqr|mpc|mpc_full starts it, and renames
controller_server's own action follow_path_unused, so bt_navigator's FollowPath comes here.
Everything after cmd_vel_nav (the smoother, collision_monitor, twist_mux's slot 20, the
locks) is unchanged. The checks are Nav2's own (checks.py, with nav2.yaml's numbers):

| Result | When |
|---|---|
| succeeded (zero sent) | within xy_goal_tolerance of the end, then yaw_goal_tolerance |
| FAILED_TO_MAKE_PROGRESS | neither moved progress_radius nor turned progress_angle in time |
| TF_ERROR | the path's frame can't be moved into odom (transform_tolerance) |
| INVALID_PATH | an empty path, or one that isn't numbers |
| UNKNOWN | no odometry for odom_timeout s |
| canceled | bt_navigator cancelled it (zero command sent) |
"""

import math
import signal
import time

from aav_description import vehicle as vehicle_file
from aav_ego.vehicle_model import from_config
from aav_track import trackers
from aav_track.checks import GoalCheck, ProgressCheck
from aav_track.path import moved, Path, wrap
from aav_track.speed import SpeedPolicy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
import tf2_ros

# Each tracker's own parameters (config/track.yaml), as <tracker>.<name>.
TRACKER_PARAMS = {
    'pure_pursuit': {},           # its lookahead comes from nav2.yaml's RPP block
    'stanley': {'gain': 1.0, 'soft_speed': 1.0, 'yaw_rate_gain': 0.0},
    'lqr': {'q': [1.0, 2.0, 0.05], 'r': 0.2, 'r_rate': 100.0, 'preview': 2.0,
            'min_speed': 0.5},
    'mpc': {'horizon': 30, 'q': [1.0, 2.0, 0.05], 'r': 2.0, 'r_rate': 1.0, 'min_speed': 0.5},
    # Its speed limits come from the node's own (cruise_speed, acceleration, max_brake,
    # lateral_accel) and vehicle.yaml's max_accel: the same as the shared policy's.
    'mpc_full': {'ds': 0.3, 'horizon': 30, 'q': [1.0, 2.0, 0.05], 'r': 0.2, 'r_rate': 1.0,
                 'q_speed': 0.2},
}
Code = FollowPath.Result


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Job:
    """One FollowPath goal: its handle, its path in odom, and how it ended."""

    def __init__(self, handle, path, started):
        self.handle = handle
        self.path = path
        self.started = started        # s: no odometry yet is only an error odom_timeout after
        self.done = Future()          # (outcome, error code, message)
        self.hint = 0

    def finish(self, outcome, code=Code.NONE, message=''):
        if not self.done.done():
            self.done.set_result((outcome, code, message))


class Track(Node):

    def __init__(self, **kwargs):
        super().__init__('track', **kwargs)
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('tracker', 'pure_pursuit'), ('rate_hz', 20.0),
            ('odom_topic', '/localization/odom/local'), ('odom_frame', 'odom'),
            ('odom_timeout', 1.0), ('transform_tolerance', 0.5), ('stamped', True),
            ('cruise_speed', 1.0), ('acceleration', 1.0), ('max_brake', 3.0),
            ('lateral_accel', 1.0),
            ('lookahead_min', 1.5), ('lookahead_max', 4.0), ('lookahead_time', 1.5),
            ('xy_goal_tolerance', 1.0), ('yaw_goal_tolerance', 0.75),
            ('progress_radius', 0.5), ('progress_angle', 0.5), ('progress_time', 10.0),
            ('wheelbase', 1.57), ('max_steering_angle', 0.5), ('max_steering_rate', 0.6))}
        self.p = p
        name = p['tracker']
        if name not in TRACKER_PARAMS:
            raise ValueError(f'tracker:={name}: pick one of {", ".join(TRACKER_PARAMS)}')
        # Every tracker's own parameters, so a live swap finds them (config/track.yaml).
        self.own = {n: {k: self.declare_parameter(f'{n}.{k}', v).value
                        for k, v in params.items()}
                    for n, params in TRACKER_PARAMS.items()}
        self.live_swap = bool(self.declare_parameter('live_swap', False).value)
        self.limits = trackers.Limits(p['wheelbase'], p['max_steering_angle'],
                                      p['max_steering_rate'])
        self.tracker = self.build_tracker(name)
        self.speed = SpeedPolicy(cruise=p['cruise_speed'], accel=p['acceleration'],
                                 max_brake=p['max_brake'], lateral_accel=p['lateral_accel'])
        self.goal = GoalCheck(p['xy_goal_tolerance'], p['yaw_goal_tolerance'])
        self.progress = ProgressCheck(p['progress_radius'], p['progress_angle'],
                                      p['progress_time'])
        self.period = 1.0 / p['rate_hz']

        self.job = None
        self.odom = None              # (x, y, yaw, v, yaw rate), odom frame
        self.odom_time = None
        self.asked = 0.0              # m/s: the last speed asked
        self.steer = 0.0              # rad: the last steering asked
        self.last_tick = None
        self.missed = 0
        self.loop_max = 0.0

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.stamped = bool(p['stamped'])
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                         'cmd_vel_nav', 10)
        self.status = self.create_publisher(DiagnosticStatus, 'drivers/nav/track/status', 10)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, 10)
        self.server = ActionServer(
            self, FollowPath, 'follow_path', execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(self.period, self.tick)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info(
            f'{name} follows Nav2\'s paths at {p["rate_hz"]:g} Hz, up to '
            f'{p["cruise_speed"]:g} m/s; done within {p["xy_goal_tolerance"]:g} m and '
            f'{p["yaw_goal_tolerance"]:g} rad')

    # --- the tracker, and swapping it live (sim only) -----------------------------------

    def build_tracker(self, name):
        """Return a new tracker called `name`, with its parameters as declared at start."""
        own = dict(self.own[name])
        p = self.p
        if name == 'pure_pursuit':
            own = {'lookahead_min': p['lookahead_min'], 'lookahead_max': p['lookahead_max'],
                   'lookahead_time': p['lookahead_time']}
        if name in trackers.MODEL_TRACKERS:
            own['period'] = 1.0 / p['rate_hz']
        if name == 'mpc_full':
            own.update(cruise=p['cruise_speed'], accel=p['acceleration'],
                       max_brake=p['max_brake'], lateral_accel=p['lateral_accel'])
        model = from_config(vehicle_file.load()) if name in trackers.MODEL_TRACKERS else None
        return trackers.make(name, self.limits, model, **own)

    def on_parameters(self, params):
        """Refuse a new tracker unless live_swap is on; otherwise swap to it at once."""
        for param in params:
            if param.name != 'tracker':
                continue
            name = param.value
            if not self.live_swap:
                return SetParametersResult(
                    successful=False,
                    reason='live_swap is off: trackers change only at launch on the car')
            if name not in TRACKER_PARAMS:
                return SetParametersResult(
                    successful=False, reason=f'pick one of {", ".join(TRACKER_PARAMS)}')
            try:
                new = self.build_tracker(name)
            except (ValueError, RuntimeError) as e:
                return SetParametersResult(successful=False, reason=str(e))
            self.swap(new)
        return SetParametersResult(successful=True)

    def swap(self, new):
        """Put `new` in the old tracker's place: the path, its place on it, the steering."""
        old = self.tracker
        job = self.job
        if job is not None:
            new.reset(job.path)
            new.hint = job.hint
        # lqr and mpc estimate the wheels' angle (SteerLag): start it at the last command.
        lag = getattr(new, 'lag', None)
        if lag is not None:
            lag.angle = self.steer
        self.tracker = new
        self.missed, self.loop_max, self.last_tick = 0, 0.0, None
        self.get_logger().info(f'tracker swapped live: {old.name} -> {new.name}'
                               f'{" mid-path" if job is not None else ""}')

    # --- listening ---------------------------------------------------------------------

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose
        values = (q.position.x, q.position.y, yaw_of(q.orientation),
                  msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if all(math.isfinite(v) for v in values):
            self.odom = values
            self.odom_time = self.seconds()

    def path_in_odom(self, msg):
        """Return the goal's path moved into odom, or raise (ValueError: bad, else TF)."""
        poses = [(s.pose.position.x, s.pose.position.y, yaw_of(s.pose.orientation))
                 for s in msg.poses]
        if not poses:
            raise ValueError('the path is empty')
        frame = msg.header.frame_id or self.p['odom_frame']
        if frame.lstrip('/') != self.p['odom_frame']:
            # The newest map -> odom, without waiting: a wait here would hold the one
            # thread that receives /tf. ekf_global sends it at 30 Hz.
            t = self.tf.lookup_transform(self.p['odom_frame'], frame, Time())
            age = self.seconds() - Time.from_msg(t.header.stamp).nanoseconds * 1e-9
            if age > self.p['transform_tolerance'] + 1.0:
                raise tf2_ros.TransformException(f'map -> odom is {age:.1f} s old')
            r = t.transform
            poses = moved(poses, (r.translation.x, r.translation.y, yaw_of(r.rotation)))
        return Path(poses)

    # --- the action ----------------------------------------------------------------------

    async def execute(self, handle):
        try:
            path = self.path_in_odom(handle.request.path)
        except ValueError as e:
            return self.end(handle, 'abort', Code.INVALID_PATH, str(e))
        except tf2_ros.TransformException as e:
            return self.end(handle, 'abort', Code.TF_ERROR, f'no map -> odom: {e}')
        job = Job(handle, path, self.seconds())
        old, self.job = self.job, job
        if old is not None:
            # The planner replanned: the newest path carries on, with no stop between.
            old.finish('abort', Code.NONE, 'replaced by a newer path')
        else:
            self.goal.reset()
            self.progress.reset()
            self.missed, self.loop_max, self.last_tick = 0, 0.0, None
        self.tracker.reset(path)
        outcome, code, message = await job.done
        return self.end(handle, outcome, code, message)

    def end(self, handle, outcome, code=Code.NONE, message=''):
        result = FollowPath.Result()
        result.error_code = code
        result.error_msg = message
        if outcome == 'succeed':
            handle.succeed()
        elif outcome == 'cancel':
            handle.canceled()
        else:
            handle.abort()
            if message and message != 'replaced by a newer path':
                self.get_logger().warning(f'FollowPath failed: {message}')
        return result

    def stop(self, job, outcome, code=Code.NONE, message=''):
        self.publish(0.0, 0.0)
        self.asked = 0.0
        if self.job is job:
            self.job = None
        job.finish(outcome, code, message)

    # --- every tick ---------------------------------------------------------------------

    def tick(self):
        job = self.job
        if job is None:
            return
        start = time.perf_counter()
        now = self.seconds()
        if self.last_tick is not None:
            late = round((now - self.last_tick) / self.period) - 1
            self.missed += max(0, late)
        self.last_tick = now
        if job.handle.is_cancel_requested:
            return self.stop(job, 'cancel')
        if self.odom is None:
            if now - job.started <= self.p['odom_timeout']:
                return                # a goal can arrive before the first odometry
            return self.stop(job, 'abort', Code.UNKNOWN,
                             f'no odometry on {self.p["odom_topic"]}')
        if now - self.odom_time > self.p['odom_timeout']:
            return self.stop(job, 'abort', Code.UNKNOWN,
                             f'no odometry on {self.p["odom_topic"]}')
        x, y, yaw, v, yaw_rate = self.odom
        path = job.path
        foot = path.project((x, y), job.hint)
        job.hint = foot.index
        if self.goal.done(x, y, yaw, (*path.end, path.goal_yaw)):
            return self.stop(job, 'succeed')
        if self.progress.stuck(x, y, yaw, now):
            return self.stop(job, 'abort', Code.FAILED_TO_MAKE_PROGRESS,
                             f'moved under {self.p["progress_radius"]:g} m in '
                             f'{self.p["progress_time"]:g} s')
        cmd = self.tracker.step(trackers.State(x, y, yaw, v, yaw_rate, self.steer),
                                self.period)
        self.steer = cmd.steer
        if cmd.speed is None:
            self.asked = self.speed.step(path, foot.s, self.asked, v, self.period,
                                         cmd.speed_cap)
        else:
            self.asked = cmd.speed        # mpc_full plans its own, within the same limits
        self.publish(self.asked, self.asked * math.tan(self.steer) / self.limits.wheelbase)
        feedback = FollowPath.Feedback()
        feedback.distance_to_goal = float(max(path.left(foot.s), 0.0))
        feedback.speed = float(v)
        job.handle.publish_feedback(feedback)
        loop_ms = 1000.0 * (time.perf_counter() - start)
        self.loop_max = max(self.loop_max, loop_ms)
        self.report(loop_ms, foot, wrap(yaw - foot.heading), path.left(foot.s), cmd)

    def publish(self, v, w):
        twist = Twist()
        twist.linear.x, twist.angular.z = float(v), float(w)
        if not self.stamped:
            self.pub.publish(twist)
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist = twist
        self.pub.publish(msg)

    def report(self, loop_ms, foot, heading, left, cmd):
        msg = DiagnosticStatus()
        msg.name = 'track'
        msg.hardware_id = 'aav_track'
        msg.level = DiagnosticStatus.WARN if loop_ms > 1000.0 * self.period else \
            DiagnosticStatus.OK
        msg.message = self.tracker.name
        values = {'tracker': self.tracker.name, 'loop_ms': f'{loop_ms:.3f}',
                  'loop_ms_max': f'{self.loop_max:.3f}', 'missed_ticks': str(self.missed),
                  'cross_track_m': f'{foot.cross:.3f}', 'heading_error_rad': f'{heading:.4f}',
                  'steer_rad': f'{self.steer:.4f}', 'speed_mps': f'{self.asked:.3f}',
                  'left_m': f'{left:.2f}'}
        values.update({k: f'{v:.4g}' for k, v in cmd.debug.items()})
        msg.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        self.status.publish(msg)


def main(args=None):
    # Ctrl-C (the launch's SIGINT) ends the spin quietly.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(args=args)
    node = Track()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

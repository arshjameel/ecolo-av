"""
ego: the virtual car between the command mux and the backends.

Subscribes  control/cmd_vel   geometry_msgs/TwistStamped (or Twist, use_stamped:=false)
            vehicle/odom      nav_msgs/Odometry: the measured speed, for the trace's twin
            drivers/<d>/lag_aware  std_msgs/Bool, latched (transient local), and
            drivers/<d>/cmd_vel    geometry_msgs/TwistStamped, for each d in
                                   lag_aware_drivers: is a lag-aware driver driving?
            world/road_ahead  aav_interfaces/RoadAhead (map), and
            localization/odom/global  nav_msgs/Odometry (map): only with road_ahead true.
                              An automatic driver's forward speed (road_ahead_drivers) is
                              never above what the road ahead allows where the car is
                              (road_limit.py); no road, or a pose older than
                              road_pose_timeout s, allows 0. People are never trimmed
Publishes   ego/actuator_cmd  ackermann_msgs/AckermannDriveStamped, at rate_hz
            ego/trace         nav_msgs/Path, frame base_link, at trace_rate_hz: where the
                              car goes over trace_horizon s (trace_ahead.py)
            ego/stop_point    geometry_msgs/PoseStamped, frame base_link, with each trace:
                              where a full brake now stops the car (stop_ahead.py);
                              stamped as the trace (base_link now), so TF at the stamp
                              puts it on the map
Parameters  the limits (ackermann.Limits) and the body (dynamics.Body: mass, payload,
            track, centre of mass height, tyre grip, the throttle's and brake's
            accelerations, the sideways limit), read at start-up. Only max_steering_rate,
            payload and lag_lead may change while running (ackermann.LIVE_LIMITS)
            lag_lead (auto|on|off), lag_lead_filter, lag_lead_max_speed, lag_lead_max_steer,
            lag_aware_drivers: the lead (lag_lead.py); road_ahead, road_ahead_drivers
            (each one of lag_aware_drivers), road_pose_timeout: the road ahead's limit
            (road_limit.py; docs/designs/2026-11-14_the-whole-car.md); trace_horizon,
            trace_rate_hz, trace_speed_loop: the trace (trace_ahead.py; its twin takes each
            measured speed);
            vehicle_yaml: the car model's numbers ('' = aav_description's vehicle.yaml)

Every backend (the CAN spine, the legacy serial bridge, turtlesim, Gazebo,
CARLA) gets the same /ego/actuator_cmd, through the output mux (aav_io_mux), so
the drivers never know which car they are driving.

A command is stale cmd_timeout seconds after it ARRIVED (like twist_mux), not
after its header stamp: the joystick may run on a Pi whose clock is off, and a
replayed bag carries old stamps. A stamp far from now only logs a warning, so
fix the clock (chrony) rather than the car stopping for no visible reason.
"""

import math
import signal

from aav_description import vehicle
from aav_description.car_model import ActuatorConfig, speed_loop_profile
from aav_description.road_ahead_ros import from_msg
from aav_description.stop_law import from_car_model
from aav_ego import dynamics
from aav_ego.ackermann import EgoModel, Limits, live_change, LIVE_LIMITS
from aav_ego.dynamics import Body
from aav_ego.lag_lead import (check_mode, command_key, LagAwareWatch, LagLead, lead_wanted,
                              LeadLimits, LeadTimes, START_ONLY)
from aav_ego.road_limit import allowed, cap
from aav_ego.stop_ahead import stop_point
from aav_ego.trace_ahead import trace, Twin
from aav_interfaces.msg import RoadAhead
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

# The program slots of twist_mux.yaml that might lead for the lag themselves. People
# (joy, rc, keyboard, buttons), the actuator tests and replays never do.
LAG_AWARE_DRIVERS = ['scripted', 'point', 'nav', 'ai']


class Ego(Node):

    def __init__(self, **kwargs):
        # kwargs: Node's own (context, parameter_overrides), for test_ego_node.py.
        super().__init__('ego', **kwargs)
        self.limits = Limits(**self.declare_all(Limits()))
        self.body = Body(**self.declare_all(Body()))
        self.timeout = self.declare_parameter('cmd_timeout', 0.3).value
        rate = self.declare_parameter('rate_hz', 50.0).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        use_stamped = self.declare_parameter('use_stamped', True).value

        self.model = EgoModel(self.limits, self.body)
        self.cmd = None          # (v, w)
        self.cmd_key = None      # lag_lead.command_key of it (stamped only)
        self.cmd_time = None     # rclpy Time the command arrived
        self.last_tick = None
        self.declare_lead_and_trace(rate)

        if use_stamped:
            self.create_subscription(TwistStamped, 'control/cmd_vel', self.on_stamped, 10)
        else:
            self.create_subscription(Twist, 'control/cmd_vel', self.on_twist, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, 'ego/actuator_cmd', 10)
        self.trace_pub = self.create_publisher(Path, 'ego/trace', 10)
        self.stop_pub = self.create_publisher(PoseStamped, 'ego/stop_point', 10)
        self.create_subscription(Odometry, 'vehicle/odom', self.on_odom, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        for driver in self.watch.aware:
            self.create_subscription(
                Bool, f'drivers/{driver}/lag_aware',
                lambda msg, d=driver: self.watch.set_aware(d, msg.data), latched)
            self.create_subscription(
                TwistStamped, f'drivers/{driver}/cmd_vel',
                lambda msg, d=driver: self.watch.sent(d, self.key(msg)), 10)
        self.declare_road_ahead()
        self.create_timer(1.0 / rate, self.tick)
        if self.trace_rate > 0.0:
            self.create_timer(1.0 / self.trace_rate, self.publish_trace)
        # After the declarations: only changes made while running reach it.
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.get_logger().info(
            f'wheelbase {self.limits.wheelbase} m, max steering '
            f'{self.limits.max_steering_angle} rad, max speed {self.limits.max_speed} m/s')
        self.log_body()

    def declare_lead_and_trace(self, rate):
        """Declare the lead's and the trace's parameters; build them from the car model."""
        p = self.declare_parameter
        self.lag_lead = p('lag_lead', 'auto').value
        reason = check_mode(self.lag_lead)
        if reason:
            raise ValueError(reason)
        limits = LeadLimits(filter_time=p('lag_lead_filter', 0.1).value,
                            max_speed_lead=p('lag_lead_max_speed', 0.3).value,
                            max_steer_lead=p('lag_lead_max_steer', 0.1).value)
        drivers = p('lag_aware_drivers', LAG_AWARE_DRIVERS).value
        self.trace_horizon = p('trace_horizon', 2.0).value
        self.trace_rate = p('trace_rate_hz', 10.0).value
        profile = p('trace_speed_loop', 'default').value
        path = p('vehicle_yaml', '').value or vehicle.config_path()
        cfg = ActuatorConfig.from_vehicle(vehicle.load(path))
        times = LeadTimes.from_config(cfg)
        self.lead = LagLead(times, limits)
        self.watch = LagAwareWatch(drivers)
        self.twin_loop = speed_loop_profile(profile)
        self.twin = Twin(cfg, self.twin_loop)
        self.trace_dt = 1.0 / rate
        self.trace_every = max(1, round(rate / self.trace_rate)) if self.trace_rate > 0 else 1
        self.lead_on = False         # did the lead run on the last tick
        self.get_logger().info(
            f'lag_lead {self.lag_lead} (on {"a simulator" if self.sim() else "the wall clock"}):'
            f' leads {times.speed_up:.2f} s speeding up, {times.slow_down:.2f} s slowing, '
            f'{times.steer:.2f} s steering, all guesses until tests 3.10-4.5; lag-aware '
            f'drivers watched: {", ".join(drivers) or "none"}')

    def declare_road_ahead(self):
        """Declare the road ahead's limit on the automatic drivers (road_limit.py)."""
        p = self.declare_parameter
        self.road_on = bool(p('road_ahead', False).value)
        self.road_drivers = list(p('road_ahead_drivers', ['nav']).value)
        self.road_pose_timeout = float(p('road_pose_timeout', 1.0).value)
        self.road, self.road_pose, self.road_pose_time = None, None, None
        self.road_trimming = False
        if not self.road_on:
            return
        unwatched = [d for d in self.road_drivers if d not in self.watch.aware]
        if unwatched:
            raise ValueError(f'road_ahead_drivers {unwatched} must also be in '
                             f'lag_aware_drivers: ego only knows whose command it is for those')
        jerk = self.limits.max_jerk if self.limits.max_jerk > 0.0 else 1e6
        self.road_curve = from_car_model(self.twin.cfg, self.limits.max_deceleration, jerk,
                                         loop=self.twin_loop)
        self.create_subscription(RoadAhead, 'world/road_ahead',
                                 lambda m: setattr(self, 'road', from_msg(m)), 10)
        self.create_subscription(Odometry, 'localization/odom/global', self.on_global, 10)
        self.get_logger().info(
            f'the road ahead limits {", ".join(self.road_drivers)}: '
            f'{self.road_curve.describe()}')

    def on_global(self, msg: Odometry):
        self.road_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.road_pose_time = self.get_clock().now()

    def road_limit(self, cmd, now):
        """Return cmd, its forward speed trimmed to the road ahead if an automatic driver's."""
        if not self.road_on or cmd is None or self.watch.whose(self.cmd_key) not in \
                self.road_drivers:
            return cmd
        age = (math.inf if self.road_pose_time is None
               else (now - self.road_pose_time).nanoseconds * 1e-9)
        limit = allowed(self.road, self.road_pose, age, self.road_pose_timeout,
                        self.road_curve, self.limits.max_speed)
        out = cap(cmd, limit)
        trimming = out != cmd
        if trimming != self.road_trimming:
            self.get_logger().info(
                f'the road ahead {"now trims" if trimming else "no longer trims"} the '
                f'automatic driver' + (f' to {limit:.2f} m/s' if trimming else ''))
        self.road_trimming = trimming
        return out

    def sim(self):
        return bool(self.get_parameter('use_sim_time').value)

    @staticmethod
    def key(msg):
        return command_key(msg.header.stamp.sec, msg.header.stamp.nanosec,
                           msg.twist.linear.x, msg.twist.angular.z)

    def on_odom(self, msg: Odometry):
        speed = msg.twist.twist.linear.x
        if math.isfinite(speed):
            self.twin.measured(speed)

    def publish_trace(self):
        now = self.get_clock().now()
        poses = trace(self.twin, self.model, self.fresh_cmd(now),
                      self.lead if self.lead_on else None, self.trace_horizon,
                      self.trace_dt, self.trace_every)
        out = Path()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        for pose in poses:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = (now + Duration(seconds=pose.t)).to_msg()
            ps.pose.position.x = float(pose.x)
            ps.pose.position.y = float(pose.y)
            ps.pose.orientation.z = math.sin(pose.yaw / 2.0)
            ps.pose.orientation.w = math.cos(pose.yaw / 2.0)
            out.poses.append(ps)
        self.trace_pub.publish(out)
        self.publish_stop_point(now)

    def publish_stop_point(self, now):
        stop = stop_point(self.twin.cfg, self.twin.state)
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = now.to_msg()
        ps.pose.position.x = float(stop.x)
        ps.pose.position.y = float(stop.y)
        ps.pose.orientation.z = math.sin(stop.yaw / 2.0)
        ps.pose.orientation.w = math.cos(stop.yaw / 2.0)
        self.stop_pub.publish(ps)

    def declare_all(self, defaults):
        """Declare a parameter for each field of a dataclass; return {name: value}."""
        # The live ones take whole numbers too: `ros2 param set /ego payload 80`.
        return {name: self.declare_parameter(
                    name, float(value),
                    ParameterDescriptor(dynamic_typing=name in LIVE_LIMITS)).value
                for name, value in vars(defaults).items()}

    def log_body(self):
        lim = self.limits
        for line in dynamics.describe(self.body, lim.wheelbase, lim.max_steering_angle,
                                      lim.max_speed, lim.max_acceleration,
                                      lim.max_deceleration, lim.max_jerk):
            if line.startswith('WARNING'):
                self.get_logger().warning(line[len('WARNING: '):])
            else:
                self.get_logger().info(line)

    def on_set_parameters(self, params):
        for p in params:
            if p.name == 'lag_lead':
                reason = check_mode(p.value)
            elif p.name in START_ONLY:
                reason = f'{p.name} is read only at start-up: restart ego to change it'
            else:
                reason = live_change(p.name, p.value)
            if reason:
                return SetParametersResult(successful=False, reason=reason)
        for p in params:
            if p.name == 'lag_lead':
                self.lag_lead = p.value
                self.get_logger().warning(f'lag_lead is now {p.value}')
            elif p.name == 'max_steering_rate':
                self.limits.max_steering_rate = float(p.value)
                self.get_logger().warning(f'max_steering_rate is now {p.value} rad/s')
            elif p.name == 'payload':
                self.body.payload = float(p.value)
                self.log_body()
        return SetParametersResult(successful=True)

    def on_stamped(self, msg: TwistStamped):
        now = self.get_clock().now()
        self.cmd_key = self.key(msg)
        stamp = Time.from_msg(msg.header.stamp, clock_type=now.clock_type)
        if stamp.nanoseconds and abs((now - stamp).nanoseconds) * 1e-9 > 0.5:
            self.get_logger().warning(
                'cmd_vel stamp is more than 0.5 s from this clock: check chrony on '
                'every machine (or this is a replayed bag)', throttle_duration_sec=10.0)
        self._store(msg.twist, now)

    def on_twist(self, msg: Twist):
        self.cmd_key = None    # no stamp: never matched to a driver, so never lag-aware
        self._store(msg, self.get_clock().now())

    def _store(self, twist, arrived):
        self.cmd = (twist.linear.x, twist.angular.z)
        self.cmd_time = arrived

    def fresh_cmd(self, now):
        """Return the command ego obeys now, or None: silence means stop."""
        if self.cmd is not None and (now - self.cmd_time).nanoseconds * 1e-9 <= self.timeout:
            return self.cmd
        return None

    def tick(self):
        now = self.get_clock().now()
        dt = 0.0 if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        cmd = self.road_limit(self.fresh_cmd(now), now)
        aware = self.watch.driving(self.cmd_key) if cmd is not None else None
        lead_on = cmd is not None and lead_wanted(self.lag_lead, self.sim(), aware is not None)
        if lead_on != self.lead_on and cmd is not None:   # a stop needs no log line
            self.get_logger().info(f'the lag lead is now {"on" if lead_on else "off"}'
                                   + (f' ({aware} is lag-aware)' if aware else ''))
        self.lead_on = lead_on
        if not lead_on:
            self.lead.reset()
        step = min(dt, 0.1)
        speed, steering, accel = self.model.step(cmd, step, self.lead.step if lead_on else None)
        # The trace's twin: each measured speed (on_odom), then what ego sent.
        self.twin.sent(speed, steering, step)

        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        out.drive.speed = float(speed)
        out.drive.steering_angle = float(steering)
        out.drive.acceleration = float(accel)
        out.drive.steering_angle_velocity = float(self.limits.max_steering_rate)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Ego()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C reaches a launched node twice (terminal and launch), and Jazzy can then
        # raise RCLError ("context is not valid"). Only a live context means a real error.
        if rclpy.ok():
            raise
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

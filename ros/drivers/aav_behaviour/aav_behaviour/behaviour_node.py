"""
behaviour: why the car should slow, and a speed cap on Nav2 that slows it (a trial).

Subscribes  /perception/stop_sign/distance  geometry_msgs/PointStamped: a sign, in its
                                    camera's frame at the picture's stamp (kept in odom)
            /perception/stop_sign, /safety/stop_hold/lock, /safety/geofence/lock,
            /safety/sensor_watchdog/lock, /safety/estop   std_msgs/Bool: the locks
            /safety/geofence/state   std_msgs/String: NEAR and why
            /safety/geofence/fence   geometry_msgs/PolygonStamped (map, transient local)
            /plan                    nav_msgs/Path (map): the goal's and the fence's distance
            /drivers/point/plan      nav_msgs/Path (map, latched): a drive to a point's line
                                     A -> B; while it runs it is the plan (active_plan)
            /localization/odom/local   nav_msgs/Odometry: the speed, and its age
            /localization/odom/global  nav_msgs/Odometry: where the car is on /plan
            /sensors/lidar/scan      sensor_msgs/LaserScan (base_link, best effort): the lane
            /world/things            aav_interfaces/Things (odom), with things true: then an
                                     obstacle stop is only for a thing that moves (a parked
                                     one is the planner's to go round); the scan's rule
                                     again, where the scan runs, if the list is over
                                     things_timeout s old (the stop zone still holds)
            /navigate_to_pose/_action/status  action_msgs/GoalStatusArray: a goal running,
                                     or aborted (goal_status_topics: each navigator
                                     action's; Nav2's and ours serve the same names)
            /tf, /tf_static          the sign's camera in odom
Publishes   /drivers/nav/speed_limit   nav2_msgs/SpeedLimit, 20 Hz: absolute m/s, 0.0 = no
                                     limit (controller_server's speed_limit_topic with
                                     behaviour:=true)
            /drivers/nav/behaviour/state   diagnostic_msgs/DiagnosticStatus, 5 Hz: state,
                                     reason, cap_mps, left_m, warnings
            /world/road_ahead        aav_interfaces/RoadAhead (map), 10 Hz: the road ahead,
                                     /plan's next 40 m from the car, its stops, its end,
                                     the speed limit by the stopping law (road_ahead.py;
                                     docs/designs/2026-11-14_the-whole-car.md). With
                                     lane_map (a lanes.yaml): the line is the car's lane's
                                     centre along the route, with its edges (lane_map.py)

The maths is behaviour.py (no ROS); the stopping law is aav_description stop_law.py, from
the car model's numbers. It isn't a driver: no cmd_vel, no twist_mux slot. It only ever
asks Nav2 to go slower; the locks make every final stop, and it never clears one.
navigation.launch.py behaviour:=true starts it (default false: the stock path).
"""

import math
import signal

from aav_behaviour.behaviour import (active_plan, along_until, Behaviour, diagnostic, lane_ahead,
                                     moving_ahead, path_left, Seen, Settings, Stop, World)
from aav_description.car_model import ActuatorConfig, speed_loop_profile
from aav_description.lane_map import lane_line, load as load_lanes, with_edges
from aav_description.road_ahead import build
from aav_description.road_ahead_ros import to_msg
from aav_description.stop_law import from_car_model
from aav_interfaces.msg import RoadAhead, Things
from aav_safety.geofence import Polygon
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, PolygonStamped
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header, String
from tf2_geometry_msgs import do_transform_point
import tf2_ros

LOCK_TOPICS = {'stop_sign': '/perception/stop_sign', 'stop_zone': '/safety/stop_hold/lock',
               'geofence': '/safety/geofence/lock', 'sensors': '/safety/sensor_watchdog/lock',
               'estop': '/safety/estop'}
RUNNING = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


class BehaviourNode(Node):

    def __init__(self):
        super().__init__('behaviour')
        # plan_timeout 25 s: within 8 m of the goal the to-pose tree keeps its path for up
        # to 20 s without replanning (navigate_to_pose_car.xml), so /plan goes quiet there.
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('base_speed', 1.0), ('floor', 0.3), ('accel', 0.5), ('fall', 2.0),
            ('ask_after', 30.0), ('release_hold', 1.0),
            ('max_deceleration', 2.0), ('max_jerk', 2.0),
            ('vehicle_yaml', ''), ('speed_loop', 'default'),
            ('stop_distance_m', 4.7), ('front', 2.0), ('half_width', 0.6),
            ('stop_gap', 1.0), ('near_depth', 2.5), ('plan_timeout', 25.0),
            ('rate_hz', 20.0), ('state_rate_hz', 5.0), ('obstacles', True),
            ('road_rate_hz', 10.0), ('road_length', 40.0), ('road_spacing', 0.5),
            # A lanes.yaml (lane_map.py): the road ahead's line on the lane's centre. Empty:
            # the line is the plan's.
            ('lane_map', ''),
            # /world/things (aav_things): wait only for what moves, leave a parked thing to
            # the planner. False: anything in the lane slows the car (the scan's rule).
            ('things', False), ('things_timeout', 1.0),
            # Each navigator action's status (navigation.launch.py; Nav2's and ours serve
            # the same names): a goal running on any of them.
            ('goal_status_topics', ['/navigate_to_pose/_action/status']))}
        self.p = p
        if p['vehicle_yaml']:
            car = ActuatorConfig.from_vehicle_yaml(p['vehicle_yaml'])
        else:
            from aav_description import vehicle
            car = ActuatorConfig.from_vehicle_yaml(vehicle.config_path())
        curve = from_car_model(car, p['max_deceleration'], p['max_jerk'],
                               loop=speed_loop_profile(p['speed_loop']))
        self.curve = curve
        self.lanes = load_lanes(p['lane_map']) if p['lane_map'] else None
        if self.lanes is not None:
            self.get_logger().info(f'lane map {p["lane_map"]}: {len(self.lanes.roads)} roads; '
                                   'the road ahead follows the lane centre')
        self.machine = Behaviour(curve, Settings(
            base=p['base_speed'], floor=p['floor'], accel=p['accel'], fall=p['fall'],
            ask_after=p['ask_after'], release_hold=p['release_hold']))
        self.get_logger().info(
            f'{curve.describe()}; cap {p["floor"]:g}-{p["base_speed"]:g} m/s on '
            '/drivers/nav/speed_limit')

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.locks = {}
        for name, topic in LOCK_TOPICS.items():
            self.create_subscription(Bool, topic,
                                     lambda m, n=name: self.locks.__setitem__(n, m.data), 10)
        self.near = False
        self.create_subscription(String, '/safety/geofence/state',
                                 lambda m: setattr(self, 'near', m.data.startswith('NEAR')), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.fence = None
        self.create_subscription(PolygonStamped, '/safety/geofence/fence', self.on_fence, latched)
        self.plan, self.plan_time = [], -math.inf
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.point_plan = []
        self.create_subscription(
            Path, '/drivers/point/plan',
            lambda m: setattr(self, 'point_plan',
                              [(s.pose.position.x, s.pose.position.y) for s in m.poses]),
            latched)
        self.speed, self.odom_time = None, -math.inf
        self.create_subscription(Odometry, '/localization/odom/local', self.on_local, 10)
        self.pose = None
        self.create_subscription(Odometry, '/localization/odom/global', self.on_global, 10)
        self.ahead = math.inf          # the nearest thing in the lane, x in base_link (m)
        self.odom_pose = None          # (x, y, yaw) in odom, from the local odometry
        self.moving, self.things_time = math.inf, -math.inf
        if p['things']:
            self.create_subscription(Things, '/world/things', self.on_things, 10)
        if p['obstacles']:
            self.create_subscription(LaserScan, '/sensors/lidar/scan', self.on_scan,
                                     qos_profile_sensor_data)
        self.sign, self.sign_time = None, -math.inf   # (x, y) in odom; the camera's frame
        self.create_subscription(PointStamped, '/perception/stop_sign/distance', self.on_sign,
                                 10)
        self.goal_active, self.aborted, self.seen_aborted = False, False, set()
        self.running = {}       # topic -> a goal running there
        for topic in p['goal_status_topics']:
            self.create_subscription(GoalStatusArray, topic,
                                     lambda msg, t=topic: self.on_status(msg, t), 10)

        self.limit = self.create_publisher(SpeedLimit, '/drivers/nav/speed_limit', 10)
        self.state = self.create_publisher(DiagnosticStatus, '/drivers/nav/behaviour/state', 10)
        self.road = self.create_publisher(RoadAhead, '/world/road_ahead', 10)
        self.decision = None
        self.said = set()
        self.create_timer(1.0 / p['rate_hz'], self.tick)
        self.create_timer(1.0 / p['state_rate_hz'], self.publish_state)
        self.create_timer(1.0 / p['road_rate_hz'], self.publish_road)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ------------------------------------------------------------------------

    def on_fence(self, msg: PolygonStamped):
        corners = [(p.x, p.y) for p in msg.polygon.points]
        self.fence = Polygon(corners) if len(corners) >= 3 else None

    def on_plan(self, msg: Path):
        self.plan = [(s.pose.position.x, s.pose.position.y) for s in msg.poses]
        self.plan_time = self.now()

    def on_local(self, msg: Odometry):
        self.speed, self.odom_time = msg.twist.twist.linear.x, self.now()
        q, at = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (at.x, at.y, yaw)

    def on_things(self, msg: Things):
        if self.odom_pose is None:
            return
        seen = [Seen(t.x, t.y, t.vx, t.vy, t.length, t.width) for t in msg.things]
        # The car's speed: a person walking in counts if they are in the lane by the time
        # the car would reach them (TODO 194).
        self.moving = moving_ahead(seen, self.odom_pose, self.p['front'], self.p['half_width'],
                                   speed=max(0.0, self.speed or 0.0))
        self.things_time = self.now()

    def obstacle_ahead(self, now):
        """Return x (base_link) of what the car slows for: a moving thing, or the scan's."""
        if self.p['things'] and now - self.things_time <= self.p['things_timeout']:
            return self.moving
        return self.ahead

    def on_global(self, msg: Odometry):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_scan(self, msg: LaserScan):
        self.ahead = lane_ahead(msg.ranges, msg.angle_min, msg.angle_increment,
                                self.p['front'], self.p['half_width'])

    def on_sign(self, msg: PointStamped):
        """Keep the sign in odom: a GPS jump moves map, and mustn't move the sign."""
        try:
            t = self.tf.lookup_transform('odom', msg.header.frame_id,
                                         Time.from_msg(msg.header.stamp), Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            # No TF at the picture's stamp: drop the picture, and say why.
            self.get_logger().warning(f'stop sign dropped: {e}', throttle_duration_sec=5.0)
            return
        point = do_transform_point(msg, t).point
        self.sign = (point.x, point.y, msg.header.frame_id)
        self.sign_time = self.now()

    def on_status(self, msg: GoalStatusArray, topic='/navigate_to_pose/_action/status'):
        self.running[topic] = any(s.status in RUNNING for s in msg.status_list)
        self.goal_active = any(self.running.values())
        for s in msg.status_list:
            key = bytes(s.goal_info.goal_id.uuid)
            if s.status == GoalStatus.STATUS_ABORTED and key not in self.seen_aborted:
                self.seen_aborted.add(key)
                self.aborted = True

    # --- each tick ---------------------------------------------------------------------

    def sign_left(self):
        """Return the metres from the sign's camera now to the sign, less the stop distance."""
        if self.sign is None:
            return None
        sx, sy, frame = self.sign
        try:
            t = self.tf.lookup_transform('odom', frame, Time())
        except tf2_ros.TransformException:
            return None
        rng = math.hypot(sx - t.transform.translation.x, sy - t.transform.translation.y)
        return rng - self.p['stop_distance_m']

    def driving(self, now):
        """Return (the plan the car drives, whose): the point driver's line, or /plan."""
        return active_plan(self.point_plan, self.plan,
                           bool(self.plan) and now - self.plan_time < self.p['plan_timeout'])

    def stops(self, now):
        found = []
        left = self.sign_left()
        if left is not None and now - self.sign_time < 5.0:
            found.append(Stop('sign', left))
        ahead = self.obstacle_ahead(now)
        if math.isfinite(ahead):
            found.append(Stop('obstacle', ahead - self.p['front'] - self.p['stop_gap']))
        plan, whose = self.driving(now)
        if plan and self.pose is not None:
            gx, gy = self.pose
            if self.goal_active or whose == 'point':
                found.append(Stop('goal', path_left(plan, gx, gy)[0]))
            if self.fence is not None:
                depth = self.p['near_depth']
                fence = along_until(plan, gx, gy,
                                    lambda x, y: self.fence.depth(x, y) < depth)
                if math.isfinite(fence):
                    found.append(Stop('fence', fence))
        return found

    def tick(self):
        now = self.now()
        world = World(now=now, speed=self.speed, odom_age=now - self.odom_time,
                      locks=dict(self.locks), near=self.near, stops=self.stops(now),
                      sign_age=now - self.sign_time,
                      # A drive to a point is a goal too: its line is there while it runs.
                      goal_active=self.goal_active or len(self.point_plan) >= 2,
                      goal_aborted=self.aborted)
        self.aborted = False
        d = self.machine.step(world)
        if self.decision is None or (d.state, d.reason) != (self.decision.state,
                                                            self.decision.reason):
            self.get_logger().info(f'{d.state}{": " + d.reason if d.reason else ""} '
                                   f'(cap {d.cap:.2f} m/s, left {d.left:.1f} m)')
        for warning in d.warnings:
            if warning not in self.said:
                self.get_logger().warning(warning)
        self.said = set(d.warnings)
        self.decision = d
        self.limit.publish(SpeedLimit(header=self.header(), percentage=False,
                                      speed_limit=float(d.cap)))

    def header(self):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')

    def publish_road(self):
        """Send the road ahead: /plan (or its lane's centre) from the car, cut at the end."""
        now = self.now()
        plan, _ = self.driving(now)
        if self.lanes is not None and plan and self.pose is not None:
            plan = lane_line(self.lanes, plan, self.pose, spacing=self.p['road_spacing'],
                             length=self.p['road_length'])
        d = self.decision
        strip = build(plan, self.pose, self.stops(now), self.curve,
                      self.p['base_speed'], state=d.state if d else '',
                      reason=d.reason if d else '', spacing=self.p['road_spacing'],
                      length=self.p['road_length'])
        if self.lanes is not None:
            strip = with_edges(strip, self.lanes)
        self.road.publish(to_msg(strip, Header(stamp=self.get_clock().now().to_msg(),
                                               frame_id='map')))

    def publish_state(self):
        if self.decision is None:
            return
        d = self.decision
        level = (DiagnosticStatus.WARN if d.warnings or d.state == 'ASK'
                 else DiagnosticStatus.OK)
        self.state.publish(DiagnosticStatus(
            level=level, name='behaviour',
            message=f'{d.state}{": " + d.reason if d.reason else ""}',
            values=[KeyValue(key=k, value=v) for k, v in diagnostic(d).items()]))


def main():
    rclpy.init()
    node = None
    try:
        node = BehaviourNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # A clean exit sends nothing: 0.0 mid-approach would jump to cruise.
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

"""
street_router: a street route from the car to a destination, for Nav2 (docs/routes.md).

For real streets later; the test lot first. A route is only a suggestion: Nav2 still
plans and drives each step on its own costmaps, and twist_mux, ego's limits and
timeout, the sensor watchdog and the spine stay in charge of the car. Nothing goes to
Nav2 unless someone asks (the follow service), or follow_at_once is set.

It asks the router (Valhalla on this computer, `pixi run -e routes router`; or an OSRM
server; or, with `router: campus`, the campus's own road graph, campus_graph.py, in this
process) for the way along the streets from the car's GPS fix to the destination, cuts it
into GPS waypoints spaced for Nav2 (osrm_route.py), and publishes them, with the route
as a map-frame path for the map window and RViz.

Subscribes  /localization/gps/fix            sensor_msgs/NavSatFix: where the car is (fix_topic)
            /drivers/nav/route/destination   geographic_msgs/GeoPointStamped: where to go
            /drivers/nav/route/goal          geometry_msgs/PoseStamped, map frame: where to
                                             go, as a map point (turned into lat/lon by /toLL)
            /drivers/nav/route/place         std_msgs/String: where to go, by name ("Mackenzie
                                             Building", "P4"); router: campus only
Publishes   /drivers/nav/route/waypoints     geographic_msgs/GeoPath, latched: the waypoints
            /drivers/nav/route/path          nav_msgs/Path, map frame, latched: the route
            /drivers/nav/route/status        std_msgs/String, latched: the length and time,
                                             or why there's no route
Serves      /drivers/nav/route/follow        std_srvs/Trigger: send the waypoints to Nav2
Calls       the router over HTTP (router_url; not for campus), /fromLL, /toLL (navsat_transform),
            /follow_gps_waypoints (only on follow): Nav2's waypoint_follower, or our
                                             navigator (aav_navigator) in its place, under
                                             the same name

One thread: rclpy's MultiThreadedExecutor spins a whole core whenever a subscription gets
messages (rclpy 7.1.12; docs/mac.md), and this node hears the GPS fix all the time. The
service calls inside callbacks are awaited (coroutine callbacks, in a reentrant group),
so the one thread keeps taking the answers. The router's HTTP request still blocks it,
for up to `timeout`: fixes that arrive meanwhile wait in the queue.
"""

import json
import math
import os
import urllib.error
import urllib.request

from aav_routes import campus_graph, osrm_route
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPath, GeoPointStamped, GeoPose, GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowGPSWaypoints
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

CAMPUS = 'campus'
# Nav2's GPS waypoints action: waypoint_follower's, or our navigator's in its place.
FOLLOW_GPS = '/follow_gps_waypoints'
ROUTERS = osrm_route.ROUTERS + (CAMPUS,)
# Latched: the map window, RViz or the HUD may start after the route was made.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SERVICE_WAIT = 1.0      # s to wait for /fromLL, /toLL, Nav2
PATH_TOLERANCE = 0.5    # m: the drawn route cuts corners by at most this


class StreetRouter(Node):

    def __init__(self, **kwargs):
        super().__init__('street_router', **kwargs)
        param = self.declare_parameter
        self.router = param('router', 'valhalla').value
        self.router_url = param('router_url', 'http://127.0.0.1:8002').value
        self.costing = param('costing', 'auto').value
        self.timeout = param('timeout', 5.0).value
        self.tolerance = param('tolerance', 1.0).value
        self.min_spacing = param('min_spacing', 10.0).value
        self.max_spacing = param('max_spacing', 30.0).value
        self.max_length = param('max_length', 3000.0).value
        self.fix_max_age = param('fix_max_age', 2.0).value
        self.map_frame = param('map_frame', 'map').value
        self.follow_at_once = param('follow_at_once', False).value
        fix_topic = param('fix_topic', '/localization/gps/fix').value
        # router: campus. The site file's .osm path is from `root` (the repo, where
        # `pixi run` starts); speed is the car's, for the time the status line gives.
        site_file = param('site_file', '').value
        root = param('root', '.').value
        self.speed = param('speed', 1.0).value
        if self.router not in ROUTERS:
            raise ValueError(f'router must be one of {ROUTERS}')
        self.campus = None
        if self.router == CAMPUS:
            try:
                self.campus = campus_graph.Campus(site_file or os.path.join(
                    get_package_share_directory('aav_routes'), 'config', 'campus_site.yaml'),
                    root)
            except (OSError, ValueError) as e:      # RouteError is a ValueError
                self.get_logger().error(f'no campus graph: {e}')

        self.busy = False               # one route at a time
        self.fix = None                 # (lat, lon, when)
        self.route = None               # the newest waypoints, for follow
        # Reentrant, so a callback awaiting a service lets the answer in on the one thread.
        group = ReentrantCallbackGroup()

        self.create_subscription(NavSatFix, fix_topic, self.on_fix, qos_profile_sensor_data,
                                 callback_group=group)
        self.create_subscription(GeoPointStamped, '/drivers/nav/route/destination',
                                 self.on_destination, 10, callback_group=group)
        self.create_subscription(PoseStamped, '/drivers/nav/route/goal', self.on_goal, 10,
                                 callback_group=group)
        self.create_subscription(String, '/drivers/nav/route/place', self.on_place, 10,
                                 callback_group=group)
        self.waypoints_pub = self.create_publisher(GeoPath, '/drivers/nav/route/waypoints',
                                                   LATCHED)
        self.path_pub = self.create_publisher(Path, '/drivers/nav/route/path', LATCHED)
        self.status_pub = self.create_publisher(String, '/drivers/nav/route/status', LATCHED)
        self.create_service(Trigger, '/drivers/nav/route/follow', self.on_follow,
                            callback_group=group)
        self.from_ll = self.create_client(FromLL, '/fromLL', callback_group=group)
        self.to_ll = self.create_client(ToLL, '/toLL', callback_group=group)
        self.nav = ActionClient(self, FollowGPSWaypoints, FOLLOW_GPS,
                                callback_group=group)
        if self.router != CAMPUS:
            self.say(f'no route yet: waiting for a destination ({self.router} at '
                     f'{self.router_url})')
        elif self.campus is None:
            self.say('no campus graph: is maps/carla/carleton.osm there (pixi run campus)?',
                     error=True)
        else:
            self.say(f'no route yet: waiting for a destination (the campus: '
                     f'{len(self.campus.graph)} road corners, '
                     f'{len(self.campus.places)} places)')

    def say(self, text, error=False):
        # Two calls, not one: rclpy refuses a change of severity at the same line.
        if error:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    # --- inputs ---

    def on_fix(self, msg):
        # NO_FIX only: STATUS_UNKNOWN (-2) is the message's default, which some drivers
        # leave; navsat_transform marks its fixes as GBAS.
        if msg.status.status == NavSatStatus.STATUS_NO_FIX or not (
                math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        self.fix = (msg.latitude, msg.longitude, self.get_clock().now())

    async def on_destination(self, msg):
        await self.plan((msg.position.latitude, msg.position.longitude))

    async def on_place(self, msg):
        if self.campus is None:
            self.say('no route: places need router: campus', error=True)
            return
        try:
            place = self.campus.find(msg.data)
        except ValueError as e:
            self.say(f'no route: {e}', error=True)
            return
        self.get_logger().info(f'to {place.name}')
        await self.plan(campus_graph.target(place), place.name)

    async def on_goal(self, msg):
        if msg.header.frame_id not in ('', self.map_frame):
            self.say(f'no route: the goal is in {msg.header.frame_id!r}, not '
                     f'{self.map_frame!r}', error=True)
            return
        if not self.to_ll.wait_for_service(timeout_sec=SERVICE_WAIT):
            self.say('no route: nobody serves /toLL (is navsat_transform running?)',
                     error=True)
            return
        req = ToLL.Request()
        req.map_point = msg.pose.position
        answer = await self.call(self.to_ll, req)
        if answer is None:
            self.say('no route: /toLL did not answer', error=True)
            return
        await self.plan((answer.ll_point.latitude, answer.ll_point.longitude))

    async def call(self, client, request):
        """Ask a service without blocking the thread; None if no answer in SERVICE_WAIT."""
        future = client.call_async(request)
        timer = self.create_timer(SERVICE_WAIT, future.cancel)   # a cancelled call gives None
        try:
            return await future
        finally:
            self.destroy_timer(timer)

    # --- the route ---

    async def plan(self, destination, name=None):
        if self.busy:
            self.say('still working on the last destination: this one is ignored',
                     error=True)
            return
        self.busy = True
        try:
            await self._plan(destination, name)
        finally:
            self.busy = False

    async def _plan(self, destination, name=None):
        start = self.car()
        if start is None:
            self.say('no route: no recent GPS fix for the car', error=True)
            return
        try:
            if self.router == CAMPUS:
                if self.campus is None:
                    raise osrm_route.RouteError('no campus graph (pixi run campus)')
                route = self.campus.route(start, destination, self.speed)
            else:
                req = osrm_route.request(self.router, self.router_url, start, destination,
                                         self.costing)
                route = osrm_route.parse_response(self.ask(req))
            if route.length > self.max_length:
                raise osrm_route.RouteError(
                    f'{route.length:.0f} m is longer than max_length ({self.max_length:.0f} m)')
            waypoints = osrm_route.waypoints(route.points, self.tolerance, self.min_spacing,
                                             self.max_spacing)
        except (OSError, ValueError) as e:     # URLError and RouteError are these
            self.say(f'no route: {e}', error=True)
            return
        self.route = waypoints
        self.waypoints_pub.publish(self.geo_path(waypoints))
        drawn = await self.map_path(route.points)
        if self.router == CAMPUS:
            text = osrm_route.summary(route, len(waypoints), f'this car at {self.speed:g} m/s')
        else:
            text = osrm_route.summary(route, len(waypoints))
        if name:
            text = f'to {name}: {text}'

        if drawn is None:
            text += '; no map-frame path (nobody serves /fromLL)'
        else:
            self.path_pub.publish(drawn)
        self.say(text)
        if self.follow_at_once:
            self.send_to_nav2()

    def car(self):
        if self.fix is None:
            return None
        lat, lon, when = self.fix
        age = (self.get_clock().now() - when).nanoseconds * 1e-9
        return (lat, lon) if age <= self.fix_max_age else None

    def ask(self, req):
        """Ask the router; its answer as parsed JSON. Errors are OSError or ValueError."""
        data = None if req.body is None else json.dumps(req.body).encode()
        http = urllib.request.Request(req.url, data=data,
                                      headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(http, timeout=self.timeout) as answer:
                return json.loads(answer.read())
        except urllib.error.HTTPError as e:
            # Valhalla and OSRM say why in a JSON body, with a 400.
            try:
                return json.loads(e.read())
            except ValueError:
                raise OSError(f'the router answered HTTP {e.code}') from None
        except urllib.error.URLError as e:
            raise OSError(f'no router at {self.router_url} ({e.reason}): start it with '
                          'pixi run -e routes router') from None

    def geo_path(self, waypoints):
        msg = GeoPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = [GeoPoseStamped(header=msg.header, pose=geo_pose(w)) for w in waypoints]
        return msg

    async def map_path(self, points):
        """Turn the route into the map frame, through /fromLL; None if nobody serves it."""
        if not self.from_ll.wait_for_service(timeout_sec=SERVICE_WAIT):
            return None
        points = osrm_route.geo.simplify(points, PATH_TOLERANCE)
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for (lat, lon), yaw in zip(points, osrm_route.geo.headings(points)):
            req = FromLL.Request()
            req.ll_point.latitude, req.ll_point.longitude = lat, lon
            answer = await self.call(self.from_ll, req)
            if answer is None:
                return None
            pose = PoseStamped(header=msg.header)
            pose.pose.position.x, pose.pose.position.y = (answer.map_point.x,
                                                          answer.map_point.y)
            o = pose.pose.orientation
            o.x, o.y, o.z, o.w = osrm_route.geo.quaternion(yaw)
            msg.poses.append(pose)
        return msg

    # --- to Nav2, only when asked ---

    def on_follow(self, request, response):
        response.success, response.message = self.send_to_nav2()
        return response

    def send_to_nav2(self):
        if not self.route:
            return False, 'no route yet'
        if not self.nav.wait_for_server(timeout_sec=SERVICE_WAIT):
            self.say(f'Nav2 is not running: nobody serves {FOLLOW_GPS}', error=True)
            return False, 'Nav2 is not running'
        goal = FollowGPSWaypoints.Goal()
        goal.gps_poses = [geo_pose(w) for w in self.route]
        self.nav.send_goal_async(goal).add_done_callback(self.on_answer)
        self.get_logger().info(f'route sent to Nav2: {len(self.route)} GPS waypoints')
        return True, f'{len(self.route)} GPS waypoints sent to Nav2'

    def on_answer(self, future):
        if not future.result().accepted:
            self.say('Nav2 refused the route', error=True)


def geo_pose(waypoint):
    """Make a GeoPose for Nav2: altitude 0, not NaN, which would spoil /fromLL's answer."""
    pose = GeoPose()
    pose.position.latitude, pose.position.longitude = waypoint.lat, waypoint.lon
    pose.position.altitude = 0.0
    o = pose.orientation
    o.x, o.y, o.z, o.w = osrm_route.geo.quaternion(waypoint.yaw)
    return pose


def main(args=None):
    rclpy.init(args=args)
    node = StreetRouter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

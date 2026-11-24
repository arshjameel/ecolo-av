"""
Test street_router in one process, with a fake router, navsat_transform and Nav2.

A destination in, the route out, then sent to Nav2 only when asked. The router is a
fake over HTTP.

Needs ROS sourced: `colcon test` runs it. It stays on this computer, on a domain of its
own (it uses absolute names). Before, on GitHub's Macs it reached the network and never
ended (TODO 42): stopping Cyclone DDS, the last node joins the thread that reads
multicast, which waits for a packet the computer sends itself, and that one never came.
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import random
import threading
import time

import pytest

# Not pytest.importorskip: ROS's launch_testing plugin then skips the whole folder.
try:
    from aav_routes.street_router import StreetRouter
    from geographic_msgs.msg import GeoPath, GeoPointStamped
    from nav2_msgs.action import FollowGPSWaypoints
    from nav_msgs.msg import Path as PathMsg
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from robot_localization.srv import FromLL
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
except ImportError:
    StreetRouter = None
from test_campus_graph import ll, write_osm  # noqa: I100 (this folder's own test)

pytestmark = pytest.mark.skipif(StreetRouter is None, reason='needs ROS sourced: colcon test')

ANSWER = (Path(__file__).resolve().parent / 'fixtures'
          / 'valhalla_carleton_lansdowne.json').read_bytes()
CARLETON = (45.3876, -75.6960)
LANSDOWNE = (45.3983, -75.6838)


class FakeRouter(BaseHTTPRequestHandler):
    """Valhalla's /route, answering with the recorded route whatever is asked."""

    asked = []

    def do_POST(self):     # noqa: N802 (http.server's name)
        FakeRouter.asked.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(ANSWER)

    def log_message(self, *args):
        pass


@pytest.fixture
def router():
    server = HTTPServer(('127.0.0.1', 0), FakeRouter)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    FakeRouter.asked = []
    yield f'http://127.0.0.1:{server.server_port}'
    server.shutdown()


class Ros:
    """
    street_router, and a fake car (fix, /fromLL, Nav2), each spun in a thread of its own.

    street_router gets one thread, as in main(): a service call inside its callbacks
    must not wait on a second thread to take the answer.
    """

    def __init__(self, **params):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        rclpy.init(domain_id=int(os.environ.get('AAV_TEST_DOMAIN') or random.randint(150, 230)))
        self.node = StreetRouter(parameter_overrides=[Parameter(k, value=v)
                                                      for k, v in params.items()])
        self.car = rclpy.create_node('fake_car')
        group = ReentrantCallbackGroup()
        self.car.create_service(FromLL, '/fromLL', self.from_ll, callback_group=group)
        self.goals = []
        ActionServer(self.car, FollowGPSWaypoints, '/follow_gps_waypoints', self.execute,
                     callback_group=group)
        self.fix = self.car.create_publisher(NavSatFix, '/localization/gps/fix', 10)
        self.fixing = False     # send_fix(): a fix at 10 Hz from then on, as a GPS does
        self.at = CARLETON      # where the fix says the car is
        self.car.create_timer(0.1, self.tick, callback_group=group)
        self.dest = self.car.create_publisher(GeoPointStamped,
                                              '/drivers/nav/route/destination', 10)
        self.place = self.car.create_publisher(String, '/drivers/nav/route/place', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.got = {'status': [], 'waypoints': [], 'path': []}
        for name, kind in (('status', String), ('waypoints', GeoPath), ('path', PathMsg)):
            self.car.create_subscription(kind, f'/drivers/nav/route/{name}',
                                         self.got[name].append, latched)
        self.follow = self.car.create_client(Trigger, '/drivers/nav/route/follow')
        self.executors = []
        for node in (self.node, self.car):
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            threading.Thread(target=executor.spin, daemon=True).start()
            self.executors.append(executor)

    def from_ll(self, request, response):
        # A pretend map: 1 m per micro-degree, enough to see the order kept.
        response.map_point.x = (request.ll_point.longitude - CARLETON[1]) * 1e5
        response.map_point.y = (request.ll_point.latitude - CARLETON[0]) * 1e5
        return response

    def execute(self, handle):
        self.goals.append(handle.request)
        handle.succeed()
        return FollowGPSWaypoints.Result()

    def send_fix(self):
        self.fixing = True

    def tick(self):
        if self.fixing:
            msg = NavSatFix()
            msg.latitude, msg.longitude = self.at
            self.fix.publish(msg)

    def go_to(self, lat_lon):
        msg = GeoPointStamped()
        msg.position.latitude, msg.position.longitude = lat_lon
        self.dest.publish(msg)

    def wait(self, check, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if check():
                return True
            time.sleep(0.05)
        return False

    def close(self):
        for executor in self.executors:
            executor.shutdown()
        self.node.destroy_node()
        self.car.destroy_node()
        rclpy.try_shutdown()


def test_a_destination_becomes_a_route_and_goes_to_nav2_only_when_asked(router):
    ros = Ros(router_url=router)
    try:
        assert ros.wait(lambda: ros.fix.get_subscription_count() > 0
                        and ros.dest.get_subscription_count() > 0)
        ros.send_fix()
        time.sleep(0.3)
        ros.go_to(LANSDOWNE)
        # The status comes last.
        assert ros.wait(lambda: any(m.data.startswith('route: 1.99 km')
                                    for m in ros.got['status']))
        # The router was asked from the car to the destination, in OSRM's JSON.
        [asked] = FakeRouter.asked
        assert asked['format'] == 'osrm'
        assert asked['locations'] == [{'lat': CARLETON[0], 'lon': CARLETON[1]},
                                      {'lat': LANSDOWNE[0], 'lon': LANSDOWNE[1]}]
        waypoints = ros.got['waypoints'][-1].poses
        assert 60 <= len(waypoints) <= 200
        end = waypoints[-1].pose.position
        assert (end.latitude, end.longitude) == pytest.approx((45.397983, -75.684669))
        path = ros.got['path'][-1]
        assert path.header.frame_id == 'map' and len(path.poses) > 10
        # Nothing went to Nav2 by itself.
        time.sleep(0.3)
        assert ros.goals == []
        assert ros.follow.wait_for_service(timeout_sec=5.0)
        future = ros.follow.call_async(Trigger.Request())
        assert ros.wait(future.done)
        assert future.result().success
        assert ros.wait(lambda: ros.goals)
        assert len(ros.goals[0].gps_poses) == len(waypoints)
    finally:
        ros.close()


def test_no_fix_no_route(router):
    ros = Ros(router_url=router)
    try:
        assert ros.wait(lambda: ros.dest.get_subscription_count() > 0)
        ros.go_to(LANSDOWNE)
        assert ros.wait(lambda: any('no recent GPS fix' in m.data for m in ros.got['status']))
        assert FakeRouter.asked == [] and ros.got['waypoints'] == []
    finally:
        ros.close()


def test_no_router_says_how_to_start_it():
    ros = Ros(router_url='http://127.0.0.1:9', timeout=1.0)    # port 9: nobody
    try:
        assert ros.wait(lambda: ros.fix.get_subscription_count() > 0
                        and ros.dest.get_subscription_count() > 0)
        ros.send_fix()
        time.sleep(0.3)
        ros.go_to(LANSDOWNE)
        assert ros.wait(lambda: any('pixi run -e routes router' in m.data
                                    for m in ros.got['status']))
    finally:
        ros.close()


def test_too_long_is_refused(router):
    ros = Ros(router_url=router, max_length=1000.0)
    try:
        assert ros.wait(lambda: ros.fix.get_subscription_count() > 0
                        and ros.dest.get_subscription_count() > 0)
        ros.send_fix()
        time.sleep(0.3)
        ros.go_to(LANSDOWNE)
        assert ros.wait(lambda: any('longer than max_length' in m.data
                                    for m in ros.got['status']))
        assert ros.got['waypoints'] == []
    finally:
        ros.close()


def test_campus_a_place_by_name(tmp_path):
    write_osm(tmp_path / 'tiny.osm')
    (tmp_path / 'site.yaml').write_text('datum: [45.386, -75.696]\nosm: tiny.osm\n')
    ros = Ros(router='campus', site_file=str(tmp_path / 'site.yaml'), root=str(tmp_path),
              speed=2.0)
    try:
        assert ros.wait(lambda: ros.fix.get_subscription_count() > 0
                        and ros.place.get_subscription_count() > 0)
        assert ros.wait(lambda: any('the campus: 6 road corners' in m.data
                                    for m in ros.got['status']))
        ros.at = ll(0, 95)      # beside D
        ros.send_fix()
        time.sleep(0.3)
        ros.place.publish(String(data='nobody'))
        assert ros.wait(lambda: any('no place called' in m.data for m in ros.got['status']))
        ros.place.publish(String(data='minto'))
        assert ros.wait(lambda: any(m.data.startswith('to Minto Hall: route: ')
                                    and 'this car at 2 m/s' in m.data
                                    for m in ros.got['status']))
        # It ends on the road nearest Minto Hall's middle.
        minto = ros.node.campus.find('Minto Hall')
        snap = ros.node.campus.graph.snap((minto.lat, minto.lon))
        end = ros.got['waypoints'][-1].poses[-1].pose.position
        assert (end.latitude, end.longitude) == pytest.approx((snap.lat, snap.lon), abs=1e-6)
        assert ros.goals == []
    finally:
        ros.close()

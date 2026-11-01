#!/usr/bin/env python3
"""
Sim check: start Gazebo's road world headless, then test the car's whole graph in it.

    pixi run sim-check                     (about 2 minutes, on this computer only)
    pixi run -e ml sim-check --perception  (adds the detector and the stop sign: 2 more minutes)
    pixi run sim-check --obstacle          (adds a box for the radar stand-in: 2 more minutes;
                                            the planner blind to it, plan_around:=false, so
                                            the stop zone must stop the car)
    pixi run sim-check --plan-around       (a box in the lane the planner sees: the car must
                                            drive round it without stopping, in a second
                                            run: 3 more minutes)
    pixi run sim-check --lane              (adds lane keeping, in a second run: 2 more minutes)
    pixi run sim-check --lidar-obstacle    (the lidar alone, radar off, stops for a post: a
                                            second run, 3 more minutes)
    pixi run sim-check --rear-obstacle     (backing up, the lidar stops the car for a post
                                            behind it: a second run, 3 more minutes)
    pixi run sim-check --ghost             (a pretend person only Nav2 sees, aav_ghost: a
                                            second run, 3 more minutes)
    pixi run sim-check --faults            (sensor failures on purpose, each while Nav2 drives:
                                            the car must stop, and say why: 5 more minutes)
    pixi run sim-check --faults --no-watchdog   (the same without the sensor watchdog: a report
                                            of what the stack does on its own, no pass or fail)
    pixi run sim-check --no-lidar          (without the simulated roof lidar: lidar:=false)
    pixi run sim-check --spine             (adds the spine in the loop, spine:=true: its
                                            stops, e-stops and arming, in a second run: 3
                                            more minutes; --python-bridge runs aav_spine's
                                            Python spine_bridge there, not the Rust one)
    pixi run sim-check --drive-to-point    (drives to points and stops there, at 1 and 3 m/s,
                                            in the empty world: two more runs, 8 more minutes;
                                            --drive-to-point-only skips the road world's run;
                                            --goto and --goto-only, the old names, still work)
    pixi run sim-check --esp32 pi|open     (every run with today's car in the loop instead,
                                            esp32:=true: the car's legacy_uart_bridge, its
                                            speed_loop.mode pi or open, and gazebo_esp32,
                                            the 2025-26 C++ firmware standing in)
    pixi run sim-check --speed-steps       (the speed loop's steps, TODO 27: reverse 0.5,
                                            then 1.0 and 2.0 m/s, in the empty world: one
                                            more run, 3 more minutes; --speed-steps-only
                                            skips the road world's run)
    pixi run sim-check --speed-loop smooth (every run with that speed loop profile:
                                            speed_loop:=smooth, docs/vehicle-model.md;
                                            lqi: TODO 3's LQI, docs/controllers.md)
    pixi run sim-check --spine-steps       (the speed steps again with the spine in the
                                            loop, spine:=true: spine-core's own loop, in
                                            the empty world; one more run, 3 more minutes)
    pixi run sim-check --spine-points      (the drives to points again with the spine in
                                            the loop, at 1 and 3 m/s, drives only: two
                                            more runs; --spine-points-only skips the
                                            road world's run)

bench_check.py tests the car's software against pretend sensors that draw a flat world;
this tests it against Gazebo's camera and radar stand-in, which see the road world
(aav_gazebo worlds/road.sdf: the bench road and its stop sign, in 3D).

1. Starts sim.launch.py world:=road gui:=false rviz:=false on a private ROS domain and
   Gazebo partition, on this computer only.
2. Start-up: Gazebo's truth, the global EKF, the camera and the radar stand-in all run,
   and Nav2 is up (unless nav:=false).
3. Checks the running graph against docs/interfaces.yaml (check_graph.py --profile road).
4. The sensors: the camera's picture and lens are the bench camera's, in camera_left_link;
   the radar stand-in's cloud has the TI driver's fields, in radar_link, and nothing in it
   while nothing is within 8.4 m. Unless --no-lidar, the simulated roof lidar: 1800 x 16 rays
   in lidar_link, 10 Hz, some of them hitting the ground and the sign; and lidar_scan's 2D
   scan of it for Nav2 (/sensors/lidar/scan: 720 ranges in base_link, 10 Hz, the sign in it).
5. Drives a scripted profile, and compares where the car really is (/vehicle/gazebo/truth)
   with where the car's own software thinks it is (/localization/odom/global). After all
   the drives of this run, `wheels vs IMU and GPS`: the watchdog's wheel cross check never
   said DISAGREE, and the worst gaps it saw (its /diagnostics).
6. With --perception: Nav2 drives to 40 m east, past the stop sign at 25 m. The car must
   stop with its front bumper before the sign, wait, and go on to the goal. The camera's
   picture with the detector's boxes, as the lock went on, is saved to /tmp/aav-sim-stop-sign.png.
7. With --obstacle: the bench's 1 m box appears in the lane 15 m ahead (Gazebo's create
   service), and Nav2 drives 30 m. collision_monitor's stop zone must stop the car before
   the box; once the box is taken away, the car must go on to the goal.
8. With --lidar-obstacle: Gazebo again, with the radar off (radar:=false), so only the roof
   lidar guards Nav2. A person-sized post (0.5 x 0.5 x 1.8 m) in the lane 15 m ahead: the
   stop zone must stop the car before it from the lidar's scan, and the post must show in
   the local costmap; once the post is gone, the car must go on to the goal.
   With --rear-obstacle, the same backing up: the goal 15 m behind the car, the post 6 m
   behind the rear bumper, and the stop zone behind the car must stop it.
9. With --ghost: Gazebo again, with ghost:=true (aav_ghost): nothing in Gazebo, only a
   pretend person 15 m past the bumper in Nav2's view (its own costmap layer and
   collision_monitor source), placed through ghost_obstacle's parameters as
   `pixi run ghost ahead 15` does. Nav2 drives 30 m: the stop zone must stop the car
   0-2 m before the ghost (as the global EKF has the car), and the local costmap must mark
   it; once the ghost is switched off, the car must go on to the goal, through where it
   stood.
10. With --faults: Gazebo again (faults:=true), and for each fault in FAULTS: Nav2 drives,
   the fault goes on (fault.<sensor>, on its node: faults.NODES), and the car must stop within
   REACT seconds, stopped by the sensor watchdog's lock or collision_monitor; then the
   fault goes off, and the lock must open again. Two faults are of the watchdog itself:
   `watchdog hang` stops its process (SIGSTOP, then SIGCONT to mend it) and `watchdog
   kill` kills it (the last fault: nothing mends it). Its silence must hold Nav2 through
   twist_mux's lock timeout. `odom freeze` goes on at FROZEN_AT m/s, while the car still
   speeds up, and counts from when the frozen wheels go wrong by more than the cross
   check's tolerances (the car's speed or turn changes: Nav2 speeds up, slows or steers),
   as they're right till then, and has FROZEN_REACT s: a speed that stays wrong needs the
   GPS's 10 s window.
   `hud stop`: the watchdog also wants the HUD's heartbeat here (watch_hud:=true); with no
   screen, the check beats for it (/hud/heartbeat, 10 Hz, a rising count) and stops, as a
   frozen HUD does. Latched: beating again, it must still hold Nav2 (hud LOST) 5 s later,
   until the check clears it (/safety/sensor_watchdog/clear), as a person would.
   With --no-watchdog it only reports, without the watchdog's own two.
11. With --lane: Gazebo again, with lane keeping (lane:=true) and nothing else driving, the
   car 0.3 m left of the road's centre. lane_finder and control_interface must bring it
   back within 5 cm, and keep it there.
11. With --spine: Gazebo again with the spine in the loop (spine:=true: ego -> spine_bridge
   -> CAN frames -> spine_sim, the spine's own firmware logic -> the actuator model). The
   normal checks first (the graph against the spine profile, the scripted drive, the EKF
   against the truth, a Nav2 goal), then the spine's own stops, each while driving:
   `command stream lost` (ego's process stopped, SIGSTOP: the spine's watchdog must brake
   fully within SPINE_BRAKE s and latch, until armed again), `software e-stop`
   (/safety/estop), `hardware e-stop` (the bench's /vehicle/bench/hw_estop: still latched
   after it is let go, until armed again) and `not armed` (disarmed, a drive must not
   move the car). The first run also measures `command stream lost` and `software
   e-stop` on the direct path, and a last table compares the two.
12. With --drive-to-point: Gazebo's empty world, twice: sim_speed 1 and 3. The point driver
   (aav_drive_to_point)
   drives to POINT_TARGETS (10, 30 and 60 m, ahead, to the side and behind), each from where
   the last stopped. Each must stop within 2 m of B by Gazebo's truth, no more than
   POINT_PAST past it by its own EKF, and slow down at about half max_brake (the hardest
   0.5 s of its final stop, by the truth; from 1 m/s, only never harder: POINT_DECEL_FROM).
   At 1 m/s also: a lat/lon B (through /toLL and /fromLL), a 250 m B refused, and a drive
   stopped by a cancel (Ctrl-C), the e-stop and a GPS fault (the driver's own timeout and
   the sensor watchdog's lock). Each drive's truth goes to /tmp/aav-sim-points/ as an
   atob-score CSV (`pixi run atob-score /tmp/aav-sim-points/1-*.csv --speed 1`).

13. With --speed-steps: Gazebo's empty world, nothing else driving. A scripted drive
   (ramped at ego's 1 m/s^2) holds each of SPEED_STEPS for 20 s: reverse at 0.5, then 1.0
   and 2.0 m/s. Each must end within 5 % of its target; its overshoot and the time to stay
   within 5 % are reported (Gazebo's truth). With --speed-loop smooth, the overshoot must
   also stay under 3 % (the profile's promise, TODO 27). On a busy computer ego's commands
   can reach gazebo_adapter late (over its 0.2 s cmd_timeout), and it brakes: a step with
   such a brake below its target is driven again, up to 3 times, and only reported (no
   pass or fail) if every try was spoiled. The samples go to /tmp/aav-sim-speed-steps.csv.

Times are Gazebo's, simulated; only the limits on waiting are wall-clock. Prints a table and
exits 1 if anything failed. The launch logs go to /tmp/aav-sim-check.log and, with --lane,
/tmp/aav-sim-check-lane.log: AAV_SIM_LOG=/tmp/mine.log moves them. AAV_SIM_DOMAIN and
AAV_SIM_PARTITION pick the ROS domain and the Gazebo partition (otherwise a domain nobody
on this computer uses now).
"""

import bisect
import collections
import importlib.util
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# A private ROS domain and Gazebo partition, on this computer only, set before rclpy
# starts: the check can't disturb a car, another laptop or someone else's simulator. The
# domain is one nobody on this computer uses now (aav_io_mux domains: on 24 Oct a guard
# test's spine_bridge, on the same random domain, latched a check's output mux).
if 'AAV_SIM_DOMAIN' in os.environ:
    DOMAIN = os.environ['AAV_SIM_DOMAIN']
else:
    from aav_io_mux.domains import free_domain
    DOMAIN = str(free_domain())
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['GZ_IP'] = '127.0.0.1'
os.environ['GZ_PARTITION'] = os.environ.get('AAV_SIM_PARTITION', f'sim_check_{DOMAIN}')

from aav_description import vehicle  # noqa: E402
from aav_gazebo.faults import NODES as FAULT_NODES  # noqa: E402
from aav_interfaces.action import DriveToPoint, ScriptedDrive  # noqa: E402
from aav_interfaces.msg import ActuatorState, SpineOutputs, VehicleStatus  # noqa: E402
from aav_lane_geometry.msg import LaneGeometry  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
import gz_crash  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
import numpy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.parameter_client import AsyncParameterClient  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from robot_localization.srv import ToLL  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Bool, String, UInt64  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from vision_msgs.msg import Detection2DArray  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402
import yaml  # noqa: E402

RESULTS = []

# The road world is the bench world (test_worlds.py checks it): its road, sign and camera.
with open(os.path.join(ROOT, 'ros', 'sim', 'aav_bench', 'config', 'bench.yaml')) as f:
    BENCH = yaml.safe_load(f)['mock_camera']['ros__parameters']
ROAD_Y = BENCH['road_y']                # m: the lane's centre line
SIGN_X = BENCH['stop_signs'][0]         # m east: the stop sign
MODELS = os.path.join(ROOT, 'ros', 'sim', 'aav_gazebo', 'models')
OBSTACLE = os.path.join(MODELS, 'obstacle', 'model.sdf')   # the bench's 1 m box
POST = os.path.join(MODELS, 'post', 'model.sdf')           # 0.5 x 0.5 x 1.8 m: a person
# What the radar adds to the road world's graph: absent with radar:=false.
RADAR_TOPICS = ('/sensors/radar/points', '/perception/radar/clusters',
                '/perception/radar/cluster_boxes')
RADAR_FIELDS = ['x', 'y', 'z', 'intensity', 'velocity']   # the TI driver's
LIDAR_RAYS = (1800, 16)     # the simulated roof lidar: steps around, beams (sensors xacro)

# Lane keeping (--lane), as bench_check.py: where the car starts, what counts as back on the
# centre line, and how long it has to get there.
LANE_START = 0.3        # m left of the road's centre
LANE_BAND = 0.05        # m either side of the centre
LANE_TIME = 30.0        # s of driving, from when the car starts moving
LOOKAHEAD = 3.0         # m: where lane_finder measures the offset (its config, lookahead)

# Sensor failures on purpose (--faults): each while Nav2 drives, in this order (the NaNs
# last: a NaN that reaches an EKF could spoil the runs after it). The watchdog's limits
# (aav_safety config/sensor_watchdog.yaml) and collision_monitor's source_timeout set how
# soon each must stop the car: REACT gives each that, plus ego's stop.
# The watchdog's own two: hang (mended by SIGCONT) early; kill last, as nothing mends it.
FAULTS = [('gps', 'stop'), ('watchdog', 'hang'), ('hud', 'stop'), ('imu', 'stop'),
          ('odom', 'stop'),
          ('odom', 'freeze'), ('camera', 'freeze'), ('radar', 'stop'), ('imu', 'freeze'),
          ('gps', 'jump'), ('gps', 'nan'), ('odom', 'nan'), ('imu', 'nan'), ('watchdog', 'kill')]
REACT = 6.0             # s of Gazebo's time from the fault to standing still
STILL = 0.05            # m/s: standing still
DRIVING = 0.5           # m/s: the car drives, so a fault can go on
# `odom freeze` goes on sooner, while the car still speeds up. Nav2's MPPI plans from the
# speed the wheels say, so frozen wheels keep the car only a little faster than them:
# frozen at 0.5 m/s, it drove on at 0.6, under any tolerance the GPS's noise allows; at
# 0.2, it drives on at about 0.4, which the GPS part sees (24 Oct, docs/simulation.md).
FROZEN_AT = 0.2         # m/s
FROZEN_WAIT = 30.0      # s of Gazebo's time for frozen wheels to go wrong (Nav2 slows, turns)
HELD = 10.0             # s of Gazebo's time the car must stay still with no watchdog at all
with open(os.path.join(ROOT, 'ros', 'command_mux', 'aav_command_mux', 'config',
                       'twist_mux.yaml')) as f:
    # s of silence from the watchdog before twist_mux's sensors lock holds Nav2 itself
    LOCK_TIMEOUT = yaml.safe_load(f)['twist_mux']['ros__parameters']['locks']['sensors'][
        'timeout']
with open(os.path.join(ROOT, 'ros', 'safety', 'aav_safety', 'config',
                       'sensor_watchdog.yaml')) as f:
    # the wheels' cross check against the IMU: its tolerances
    CROSS = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']['odom']
# s from frozen wheels going wrong to standing still: the IMU sees a quick change at once,
# but a speed that stays wrong needs the GPS's window, then the persist time and the stop.
FROZEN_REACT = CROSS['cross_gps_window'] + CROSS['cross_persist'] + 3.0
LAUNCH = {}             # the running launch: its process group and log (session())

# The spine in the loop (--spine). spine_bridge stops sending DRIVE_CMD cmd_timeout s after
# ego's last command (spine.yaml), and the spine's watchdog brakes fully 0.1 s after its
# last DRIVE_CMD (spine-core cmd_timeout_ms). On the direct path gazebo_adapter's own
# cmd_timeout (0.2 s) does both. TICKS allows for each one's tick and the frames' trip.
with open(os.path.join(ROOT, 'ros', 'can_bridge', 'aav_spine', 'config', 'spine.yaml')) as f:
    BRIDGE_TIMEOUT = yaml.safe_load(f)['spine_bridge']['ros__parameters']['cmd_timeout']
SPINE_WATCHDOG = 0.1    # s: spine-core's cmd_timeout_ms
DIRECT_TIMEOUT = 0.2    # s: gazebo_adapter's cmd_timeout
TICKS = 0.08            # s
SPINE_BRAKE = BRIDGE_TIMEOUT + SPINE_WATCHDOG + TICKS   # ego quiet -> full brake, spine
DIRECT_BRAKE = DIRECT_TIMEOUT + TICKS                   # ego quiet -> brake, direct path
ESTOP_BRAKE = 0.15      # s from /safety/estop or the hardware e-stop to the spine's brake
STOP_TIME = 3.0         # s from the stop to standing still, from about 1 m/s
LATCH_HOLD = 3.0        # s the car must stay still, its drive still asking, while latched
STOPS = {}              # (stop, path) -> (s to the brake, s to standing still, m/s before)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def result(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print(f'{"PASS" if ok else "FAIL"}  {name}: {detail}', flush=True)


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rate(stamps):
    """Return messages per second of simulated time, from their stamps (s)."""
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    return (len(stamps) - 1) / span if span > 0 else 0.0


def gz_service(service, reqtype, request):
    """Call one of the world's Gazebo services; True if it says it worked."""
    try:
        done = subprocess.run(
            ['gz', 'service', '-s', f'/world/aav/{service}', '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request],
            capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return False
    return done.returncode == 0 and 'data: true' in done.stdout


class Probe(Node):

    def __init__(self):
        # On Gazebo's clock, like every node in the sim.
        super().__init__('sim_check', parameter_overrides=[Parameter('use_sim_time',
                                                                     value=True)])
        self.truth = self.glob = self.image = self.info = self.cloud = None
        # Every truth sample, lane message and sensor stamp, from the start: lane keeping
        # starts as soon as the camera sees the lane, before its check begins.
        self.track, self.lanes, self.image_stamps, self.cloud_stamps = [], [], [], []
        self.create_subscription(Odometry, '/vehicle/gazebo/truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/localization/odom/global',
                                 lambda m: setattr(self, 'glob', m), 10)
        self.create_subscription(LaneGeometry, '/perception/lane/geometry', self.on_lane, 10)
        self.create_subscription(Image, '/sensors/camera/front_left/image_raw', self.on_image, 2)
        self.create_subscription(CameraInfo, '/sensors/camera/front_left/camera_info',
                                 lambda m: setattr(self, 'info', m), 2)
        self.create_subscription(PointCloud2, '/sensors/radar/points', self.on_cloud,
                                 qos_profile_sensor_data)
        self.drive = ActionClient(self, ScriptedDrive, '/scripted_drive')
        self.drive_to_point = ActionClient(self, DriveToPoint, '/drive_to_point')
        self.estop = self.create_publisher(Bool, '/safety/estop', 10)
        self.to_ll = self.create_client(ToLL, '/toLL')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.stop, self.stop_picture, self.picture, self.stop_sign = False, None, None, None
        self.create_subscription(Bool, '/perception/stop_sign', self.on_stop, 10)
        self.create_subscription(Image, '/perception/detections/image',
                                 lambda m: setattr(self, 'picture', m), 2)
        # The stop signs the detector saw: (where the car was, x m; score; box height, px).
        self.signs = []
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_boxes, 10)
        self.lock, self.state = None, ''     # the sensor watchdog's
        self.lock_heard = None               # when its lock last arrived (Gazebo's time)
        self.fail_safe = True                # twist_mux's sensors lock holds on silence
        self.create_subscription(Bool, '/safety/sensor_watchdog/lock', self.on_lock, 10)
        self.create_subscription(String, '/safety/sensor_watchdog/state', self.on_state, 10)
        # The HUD's heartbeat, standing in for its screen (--faults, watch_hud:=true):
        # 10 Hz with a rising count while hud_beating, the way aav_hud sends it. On
        # its own thread, as the HUD's: a check that waits without spinning (a service, an
        # action server) mustn't look like a dropout, which latches the watchdog.
        self.hud_beating, self.hud_frames = False, 0
        self.hud_beat = self.create_publisher(UInt64, '/hud/heartbeat', 10)
        threading.Thread(target=self.beat_for_the_hud, daemon=True).start()
        self.disagreed = False      # the wheels' cross check ever said DISAGREE
        self.gaps = {}              # its worst gaps, from /diagnostics
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        # The car's status (the spine's, with spine:=true), and the actuators' demands as
        # (stamp, throttle, brake): spine_sim's outputs with the spine in the loop, or
        # gazebo_adapter's own speed loop's commands on the direct path. For the stops.
        self.status, self.demands, self.direct = None, [], {}
        self.latches = []    # Gazebo's time each time the spine stopped being armed
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_status, 10)
        self.create_subscription(SpineOutputs, '/vehicle/gazebo/spine_outputs',
                                 lambda m: self.on_demand(seconds(m.header.stamp), m.throttle,
                                                          m.brake), 10)
        self.create_subscription(ActuatorState, '/vehicle/throttle',
                                 lambda m: self.on_direct('throttle', m), 10)
        self.create_subscription(ActuatorState, '/vehicle/brake',
                                 lambda m: self.on_direct('brake', m), 10)
        self.estop = self.create_publisher(Bool, '/safety/estop', 10)
        self.hw_estop = self.create_publisher(Bool, '/vehicle/bench/hw_estop', 10)
        self.arm_client = self.create_client(SetBool, '/spine/arm')
        self.monitor = []    # (action, polygon) from collision_monitor, as it changes
        self.create_subscription(CollisionMonitorState, '/safety/collision_monitor/state',
                                 lambda m: self.monitor.append((m.action_type, m.polygon_name)),
                                 10)
        self.output_mux = ''   # the output mux's state: OPEN, WAIT or STOP (aav_io_mux)
        self.create_subscription(String, '/safety/output_mux/state',
                                 lambda m: setattr(self, 'output_mux', m.data), 10)

    def on_truth(self, msg):
        self.truth = msg
        self.track.append((seconds(msg.header.stamp), msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw_of(msg), msg.twist.twist.linear.x))

    def on_status(self, msg):
        if self.armed() and not msg.armed:
            self.latches.append(seconds(msg.header.stamp))
        self.status = msg

    def on_demand(self, stamp, throttle, brake):
        self.demands = self.demands[-2999:] + [(stamp, throttle, brake)]

    def on_direct(self, kind, msg):
        """gazebo_adapter's throttle and brake commands (NaN from spine_bridge: skipped)."""
        if math.isnan(msg.command):
            return
        self.direct[kind] = msg.command
        if kind == 'brake' and 'throttle' in self.direct:
            self.on_demand(seconds(msg.header.stamp), self.direct['throttle'], msg.command)

    def arm(self, on=True):
        """Ask spine_bridge to arm the spine (on) or disarm it; True if it agreed."""
        if not self.arm_client.wait_for_service(timeout_sec=20.0):
            return False
        future = self.arm_client.call_async(SetBool.Request(data=on))
        self.spin_for(10.0, future.done)
        return future.done() and future.result().success

    def armed(self):
        return self.status is not None and self.status.armed

    def arm_and_hold(self, tries=3):
        """Arm, and wait until the spine has stayed armed for 2 s (bench_check.py's)."""
        # Ego's commands reach spine_bridge only once the output mux is open; armed before
        # that, the spine's watchdog latches it again at once.
        self.spin_for(60.0, lambda: self.output_mux.startswith('OPEN'))
        for _ in range(tries):
            self.arm()
            if self.spin_for(3.0, self.armed) and not self.spin_for(2.0,
                                                                    lambda: not self.armed()):
                return True
        return False

    def send_drive(self, speed=1.0, hold=15.0):
        """Start a scripted drive; return its goal handle, or None if refused."""
        if not self.drive.wait_for_server(timeout_sec=20.0):
            return None
        sent = self.drive.send_goal_async(ScriptedDrive.Goal(
            target_speed=speed, acceleration=0.5, deceleration=1.0, hold_time=hold))
        self.spin_for(10.0, sent.done)
        return sent.result() if sent.done() and sent.result().accepted else None

    def speed(self):
        return self.truth.twist.twist.linear.x

    def beat_for_the_hud(self):
        while rclpy.ok():
            time.sleep(0.1)
            if not self.hud_beating:
                continue
            self.hud_frames += 3            # 30 frames a second, as the HUD draws
            try:
                self.hud_beat.publish(UInt64(data=self.hud_frames))
            except Exception:               # the check is over: the node has gone
                return

    def on_state(self, msg):
        self.state = msg.data
        self.disagreed = self.disagreed or 'DISAGREE' in msg.data

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'sensor_watchdog: odom':
                self.gaps.update({kv.key: kv.value for kv in status.values
                                  if kv.key.startswith('worst_')})

    def on_lock(self, msg):
        self.lock, self.lock_heard = msg.data, self.now()

    def silent(self):
        """Return True if the watchdog has been quiet long enough for twist_mux to lock."""
        return self.fail_safe and (self.lock_heard is None
                                   or self.now() - self.lock_heard > LOCK_TIMEOUT)

    def locked(self):
        """Return True while twist_mux's sensors lock holds Nav2: true, or silent."""
        return self.lock is True or self.silent()

    def on_lane(self, msg):
        self.lanes.append((seconds(msg.header.stamp), msg.vehicle_offset_m,
                           msg.both_lanes_detected))

    def on_image(self, msg):
        self.image = msg
        self.image_stamps = self.image_stamps[-49:] + [seconds(msg.header.stamp)]

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_stamps = self.cloud_stamps[-49:] + [seconds(msg.header.stamp)]

    def on_boxes(self, msg):
        x = self.truth.pose.pose.position.x if self.truth is not None else math.nan
        self.signs += [(x, d.results[0].hypothesis.score, d.bbox.size_y)
                       for d in msg.detections
                       if d.results and d.results[0].hypothesis.class_id == 'stop sign']

    def on_stop(self, msg):
        if msg.data and not self.stop:
            self.stop_picture = self.picture   # what the camera saw as the lock went on
            self.stop_sign = self.signs[-1] if self.signs else None
        self.stop = msg.data

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None

    def xy(self, msg):
        return msg.pose.pose.position.x, msg.pose.pose.position.y

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def go_to(self, x, y, yaw):
        """Send Nav2 a goal in the map frame; return the result's future, or None if refused."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return None
        sent = self.nav.send_goal_async(goal)
        self.spin_for(10.0, lambda: sent.done())
        if not sent.done() or not sent.result().accepted:
            return None
        self.goal = sent.result()     # its handle, to cancel it
        return self.goal.get_result_async()


def check_contract(probe, env, tag, ignore=(), profile='road'):
    """
    Check the running graph against docs/interfaces.yaml (check_graph.py --profile road).

    profile: spine instead, with the spine in the loop.

    ignore: topics this run leaves out on purpose, such as Nav2's /map with nav:=false.
    """
    # Up to three tries, 10 s apart, as in bench_check.py: a node's first start after a
    # build can be slow on a Mac. The probe spins meanwhile, so its track has no gaps.
    for attempt in range(3):
        with tempfile.TemporaryFile('w+') as out:
            graph = subprocess.Popen(
                [sys.executable, os.path.join(ROOT, 'tools', 'check_graph.py'), '--profile',
                 profile], cwd=ROOT, env=env, stdout=out, stderr=subprocess.DEVNULL, text=True)
            probe.spin_for(60.0, lambda: graph.poll() is not None)
            code = graph.wait()
            out.seek(0)
            every = [line for line in out.read().splitlines() if line.startswith('FAIL')]
            fails = [line for line in every
                     if not any(line.startswith(f'FAIL  {t}:') for t in ignore)]
        # check_graph exits 1 on any FAIL, the ignored ones too: then only fails count.
        ok = not fails and (code == 0 or bool(every))
        if ok:
            break
        probe.spin_for(10.0)
    result(f'graph vs contract{tag}', ok,
           f'{len(fails)} failures' + (': ' + '; '.join(fails[:3]) if fails else ''))


def check_sensors(probe):
    """Check the camera is the bench camera, in camera_left_link, and the radar the TI's."""
    probe.spin_for(5.0)     # a few seconds of each, for their rates
    image, info, cloud = probe.image, probe.info, probe.cloud
    camera_ok = (image.width, image.height, image.header.frame_id) == (
        BENCH['width'], BENCH['height'], 'camera_left_link')
    lens_ok = (info.header.frame_id == 'camera_left_link'
               and abs(info.k[0] - BENCH['focal_px']) < 1.0
               and abs(info.k[4] - BENCH['focal_px']) < 1.0
               and abs(info.k[2] - BENCH['width'] / 2.0) < 1.0
               and abs(info.k[5] - BENCH['height'] / 2.0) < 1.0)
    fields = [f.name for f in cloud.fields]
    radar_ok = cloud.header.frame_id == 'radar_link' and fields == RADAR_FIELDS
    empty = cloud.width * cloud.height == 0     # the sign is 25 m away, the radar sees 8.4 m
    camera_hz, radar_hz = rate(probe.image_stamps), rate(probe.cloud_stamps)
    ok = (camera_ok and lens_ok and radar_ok and empty and abs(camera_hz - 10.0) < 1.5
          and abs(radar_hz - 10.0) < 1.5)
    result('sensors', ok,
           f'camera {image.width} x {image.height} {image.encoding} in {image.header.frame_id} '
           f"at {camera_hz:.1f} Hz, focal {info.k[0]:.1f} px (the bench's "
           f'{BENCH["focal_px"]:.0f}); radar stand-in in {cloud.header.frame_id} at '
           f'{radar_hz:.1f} Hz, fields {",".join(fields)}, '
           f'{cloud.width * cloud.height} points with nothing within 8.4 m')


def check_cameras(probe):
    """
    Check the other two cameras: front right and rear, each in its own frame.

    At 5 Hz or more: they send 10, but on a busy Mac two more reliable picture streams lose
    some. The front left camera's own check (check_sensors) holds 10 Hz.
    """
    # Only for this check, as the lidar's: two more picture streams would load the run.
    got = {'front_right': [], 'rear': []}
    subs = [probe.create_subscription(Image, f'/sensors/camera/{name}/image_raw',
                                      lambda m, n=name: got[n].append(m), 2)
            for name in got]
    probe.spin_for(5.0)
    for sub in subs:
        probe.destroy_subscription(sub)
    frames = {'front_right': 'camera_right_link', 'rear': 'camera_rear_link'}
    parts, ok = [], True
    for name, msgs in got.items():
        hz = rate([seconds(m.header.stamp) for m in msgs])
        frame = msgs[-1].header.frame_id if msgs else 'nothing'
        ok &= bool(msgs) and frame == frames[name] and hz >= 5.0
        parts.append(f'{name} in {frame} at {hz:.1f} Hz')
    result('cameras', ok, '; '.join(parts))


def check_lidar(probe):
    """Check the simulated roof lidar: 1800 x 16 rays in lidar_link at 10 Hz, some hitting."""
    # Only for this check: 9 MB/s of reliable clouds would load the rest of the run.
    clouds = []
    sub = probe.create_subscription(PointCloud2, '/sensors/lidar/points', clouds.append, 2)
    probe.spin_for(5.0)
    probe.destroy_subscription(sub)
    if not clouds:
        result('lidar', False, 'nothing on /sensors/lidar/points in 5 s')
        return
    cloud = clouds[-1]
    x = point_cloud2.read_points(cloud, field_names=['x'], skip_nans=False)['x']
    hits = int(numpy.isfinite(x).sum())
    lidar_hz = rate([seconds(c.header.stamp) for c in clouds])
    ok = (cloud.header.frame_id == 'lidar_link' and (cloud.width, cloud.height) == LIDAR_RAYS
          and abs(lidar_hz - 10.0) < 1.5 and hits > 1000)
    result('lidar', ok, f'{cloud.width} x {cloud.height} rays in {cloud.header.frame_id} at '
                        f'{lidar_hz:.1f} Hz, {hits} hits (the ground and the sign)')


def check_lidar_scan(probe):
    """Check lidar_scan's 2D scan for Nav2: 720 ranges in base_link, 10 Hz, the sign in it."""
    scans = []
    sub = probe.create_subscription(LaserScan, '/sensors/lidar/scan', scans.append,
                                    qos_profile_sensor_data)
    probe.spin_for(5.0)
    probe.destroy_subscription(sub)
    if not scans:
        result('lidar scan', False, 'nothing on /sensors/lidar/scan in 5 s')
        return
    scan = scans[-1]
    ranges = numpy.asarray(scan.ranges)
    finite = ranges[numpy.isfinite(ranges)]
    hz = rate([seconds(m.header.stamp) for m in scans])
    # The road world's only thing over 0.3 m near the road: the stop sign, 25 m ahead.
    ok = (scan.header.frame_id == 'base_link' and len(ranges) in (720, 721)
          and abs(hz - 10.0) < 2.0 and finite.size > 0)
    nearest = f'the nearest {finite.min():.1f} m away' if finite.size else 'nothing seen'
    result('lidar scan', ok, f'{len(ranges)} ranges in {scan.header.frame_id} at {hz:.1f} Hz, '
                             f'{finite.size} of them see something, {nearest} (the sign)')


def check_drive(probe, spine=False):
    """
    Drive the scripted profile; compare the distance with the plan, and the EKF with truth.

    spine: the spine is in the loop. If it latched on the way (a gap in ego's commands on a
    busy computer, as on the bench), which stops the car, arm it and drive the plan again,
    once, as bench_check.py does.
    """
    if not probe.drive.wait_for_server(timeout_sec=20.0):
        result('scripted drive', False, 'no scripted driver')
        return
    for attempt in range(2 if spine else 1):
        if spine and not probe.armed():
            probe.arm_and_hold()
        latches = len(probe.latches)
        x0, y0 = probe.xy(probe.truth)
        sent = probe.drive.send_goal_async(ScriptedDrive.Goal(
            target_speed=1.0, acceleration=0.5, deceleration=1.0, hold_time=5.0))
        probe.spin_for(10.0, lambda: sent.done())
        if not sent.done() or not sent.result().accepted:
            result('scripted drive', False, 'refused')
            return
        done = sent.result().get_result_async()
        probe.spin_for(60.0, lambda: done.done())
        start = probe.now()
        probe.spin_for(20.0, lambda: probe.now() - start > 3.0)    # 3 s to come to rest
        x1, y1 = probe.xy(probe.truth)
        travelled = math.hypot(x1 - x0, y1 - y0)
        # The plan: 2 s up to 1 m/s (1 m), 5 s at 1 m/s (5 m), 1 s down (0.5 m). The range
        # is bench_check.py's: it catches a stretched plan, or no motion at all.
        good = 4.5 < travelled < 8.0
        if good or len(probe.latches) == latches:
            break
    again = ', the second time: the spine latched during the first' if attempt else ''
    result('scripted drive', good,
           f'{travelled:.2f} m (plan: 6.5 m), {y1 - y0:+.2f} m to the side{again}')
    gx, gy = probe.xy(probe.glob)
    error = math.hypot(gx - x1, gy - y1)
    result('global EKF vs truth', error < 1.5, f'{error:.2f} m apart (GPS noise is 0.5 m)')


def check_wheels_vs_imu(probe):
    """After the drives: the watchdog's wheel cross check must not have cried wolf."""
    probe.spin_for(3.0, lambda: bool(probe.gaps))
    worst = {k: probe.gaps.get(f'worst_{k}_gap', '?') for k in ('speed', 'turn', 'gps')}
    result('wheels vs IMU and GPS', bool(probe.gaps) and not probe.disagreed,
           f"worst gaps in normal driving: speed {worst['speed']} m/s, turn {worst['turn']} "
           f"rad/s, GPS speed {worst['gps']} m/s (the watchdog stops at "
           f"{CROSS['cross_speed_tolerance']:g}, {CROSS['cross_yaw_tolerance']:g} and "
           f"{CROSS['cross_gps_tolerance']:g} for {CROSS['cross_persist']:g} s)"
           + ('; it said DISAGREE' if probe.disagreed else ''))


def check_stop_sign(probe):
    """Nav2 drives to 40 m east, past the stop sign at 25 m: it must stop before it."""
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']   # base_link to the front bumper
    done = probe.go_to(40.0, ROAD_Y, 0.0)
    if done is None:
        result('stop sign', False, 'no Nav2, or it refused the goal')
        return
    seen = len(probe.signs)
    probe.stop_sign = None
    locked_at, still_at, end = None, None, time.monotonic() + 240.0
    while time.monotonic() < end and not done.done():
        probe.spin_for(0.1)
        if probe.stop and locked_at is None:
            locked_at = probe.truth.pose.pose.position.x
        if probe.stop and still_at is None and abs(probe.truth.twist.twist.linear.x) < 0.05:
            still_at = probe.truth.pose.pose.position.x
    x = probe.truth.pose.pose.position.x
    reached = done.done() and done.result().status == 4
    ok = (locked_at is not None and locked_at < SIGN_X - 1.0 and still_at is not None
          and still_at + front < SIGN_X and reached and x > 35.0)
    signs = probe.signs[seen:]
    first = (f'the detector first saw it from x = {signs[0][0]:.1f} m' if signs
             else 'the detector never saw it')
    lock = 'never locked'
    if locked_at is not None:
        lock = f'locked at x = {locked_at:.1f} m'
        if probe.stop_sign is not None:
            lock += f' (score {probe.stop_sign[1]:.2f}, {probe.stop_sign[2]:.0f} px tall)'
    still = (f'stood still at x = {still_at:.1f} m, its bumper '
             f'{SIGN_X - still_at - front:.1f} m before the sign'
             if still_at is not None else 'never stood still')
    result('stop sign', ok, f'{first}; {lock}; {still} (the sign is at {SIGN_X:.0f} m); '
                            f'Nav2 goal {"reached" if reached else "not reached"} at {x:.1f} m')
    if probe.stop_picture is not None:
        try:
            from cv_bridge import CvBridge
            import cv2
            cv2.imwrite('/tmp/aav-sim-stop-sign.png',
                        CvBridge().imgmsg_to_cv2(probe.stop_picture, 'bgr8'))
            print('The camera as the lock went on: /tmp/aav-sim-stop-sign.png', flush=True)
        except Exception as e:  # noqa: B902 - the picture is a bonus
            print(f'could not save the picture: {e}', flush=True)


def costmap_peak(grid, x, y, radius):
    """Return the highest cost (0-100) in an OccupancyGrid within `radius` m of (x, y)."""
    info = grid.info
    cells = numpy.asarray(grid.data, dtype=numpy.int16).reshape(info.height, info.width)
    ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
    cx = (numpy.arange(info.width) + 0.5) * res + ox
    cy = (numpy.arange(info.height) + 0.5) * res + oy
    near = (cx[None, :] - x) ** 2 + (cy[:, None] - y) ** 2 <= radius ** 2
    return int(cells[near].max()) if near.any() else -1


def check_obstacle(probe, ahead=15.0, goal=30.0, model=OBSTACLE, depth=1.0, name='obstacle',
                   costmap=False, backwards=False):
    """
    Put a model `ahead` m in the lane: the stop zone must stop the car before it.

    model is the bench's box (1 m deep) unless given; the lidar's check uses the post
    (0.5 m). With costmap, the local costmap must also mark it while the car stands there.
    backwards: the model and the goal are behind the car, which must back up to them, and
    the stop zone behind the rear bumper must stop it.
    """
    body = vehicle.load()['vehicle']['body']
    # base_link to the bumper the car drives towards: the front one, or the rear one.
    front = body['rear_overhang'] if backwards else body['length'] - body['rear_overhang']
    way = -1.0 if backwards else 1.0
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = way * math.cos(h), way * math.sin(h)     # the way the car goes

    def along():
        x, y = probe.xy(probe.truth)
        return (x - x0) * c + (y - y0) * s

    # Its near face `ahead` m past the bumper, on the car's line.
    bx, by = x0 + (front + ahead + depth / 2.0) * c, y0 + (front + ahead + depth / 2.0) * s
    if not gz_service('create', 'gz.msgs.EntityFactory',
                      f'sdf_filename: "{model}", name: "obstacle", '
                      f'pose: {{position: {{x: {bx}, y: {by}, z: 0}}, '
                      f'orientation: {{z: {math.sin(h / 2.0)}, w: {math.cos(h / 2.0)}}}}}'):
        result(name, False, "could not place the box (Gazebo's create service)")
        return
    newest = {}     # the local costmap, in odom: on the world's origin, as the car started
    if costmap:
        sub = probe.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                        lambda m: newest.update(grid=m), 2)
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h)    # facing as it started
    if done is None:
        result(name, False, 'no Nav2, or it refused the goal')
        gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')
        return
    seen, top, still, gap, peak = len(probe.monitor), 0.0, None, None, None
    moving = 0.3 if backwards else 0.5      # m/s: it has got going (reversing: 0.5 at most)
    furthest, fired = 0.0, None             # m towards the model; when the stop zone fired

    def take_away():
        nonlocal gap, peak
        gap = ahead - furthest              # the closest the bumper came
        if 'grid' in newest:
            peak = costmap_peak(newest['grid'], bx, by, depth / 2.0 + 0.5)
        gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')

    end = time.monotonic() + 240.0
    while time.monotonic() < end and not done.done():
        probe.spin_for(0.1)
        v = way * probe.truth.twist.twist.linear.x
        top = max(top, v)
        if gap is not None:
            continue
        furthest = max(furthest, along())
        if fired is None and any(a == CollisionMonitorState.STOP and p == 'StopZone'
                                 for a, p in probe.monitor[seen:]):
            fired = probe.now()
        if top > moving and abs(v) < 0.02:
            still = still if still is not None else probe.now()
            if probe.now() - still > 3.0:      # stood still for 3 s: take the box away
                take_away()
        else:
            still = None
            # Stopped, then rocking: the controller may pull away (forward, while backing
            # up) and try again, the stop zone stopping it each time. Take it away anyway.
            if fired is not None and probe.now() - fired > 8.0:
                take_away()
    if costmap:
        probe.destroy_subscription(sub)
    stop_zone = any(a == CollisionMonitorState.STOP and p == 'StopZone'
                    for a, p in probe.monitor[seen:])
    reached = done.done() and done.result().status == 4
    passed = along() > front + ahead + depth
    marked = not costmap or (peak is not None and peak >= 99)
    ok = gap is not None and 0.0 < gap < 2.0 and stop_zone and reached and passed and marked
    where = f'came within {gap:.2f} m of it' if gap is not None else 'never stopped'
    mark = ''
    if costmap:
        mark = (f'; the local costmap {peak} of 100 there' if peak is not None
                else '; no local costmap seen')
    result(name, ok, f'{where} (top speed {top:.2f} m/s), stop zone fired: {stop_zone}{mark}; '
                     f'taken away, Nav2 goal {"reached" if reached else "not reached"} '
                     f'{along():.1f} m on')
    if gap is None:
        gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')


def check_lane_keeping(probe):
    """
    Lane keeping alone: from LANE_START m left of the road's centre, the car must come back.

    control_interface drives from the moment it sees the lane, so this reads the track the
    probe has kept since start-up: LANE_TIME seconds of Gazebo's time, from when the car
    started moving.
    """
    def moved():
        return next((s for s in probe.track if s[4] > 0.1), None)

    if not probe.spin_for(60.0, lambda: moved() is not None):
        result('lane keeping', False, f'the car never moved ({len(probe.lanes)} lane messages)')
        return
    t0, x0, y0 = moved()[:3]
    probe.spin_for(4.0 * LANE_TIME, lambda: probe.track[-1][0] - t0 >= LANE_TIME)
    run = [s for s in probe.track if t0 <= s[0] <= t0 + LANE_TIME]
    stamps = [s[0] for s in run]

    def at(t):
        """Return the truth sample nearest to time t (s, Gazebo's)."""
        i = min(bisect.bisect_left(stamps, t), len(stamps) - 1)
        return run[i - 1 if i and t - stamps[i - 1] < stamps[i] - t else i]

    start = y0 - ROAD_Y
    outside = [s[0] - t0 for s in run if abs(s[2] - ROAD_Y) > LANE_BAND]
    settled = outside[-1] if outside else 0.0   # s of driving: within the band from then on
    over = max(0.0, max(-math.copysign(1.0, start) * (s[2] - ROAD_Y) for s in run))
    last = [s for s in run if s[0] >= stamps[-1] - 5.0]
    end = sum(s[2] for s in last) / len(last) - ROAD_Y
    heading = sum(s[3] for s in last) / len(last)
    # lane_finder against the truth: the car's offset LOOKAHEAD m ahead, square to the road.
    errors, both = [], 0
    for t, offset, two in (m for m in probe.lanes if t0 <= m[0] <= stamps[-1]):
        _, _, y, yaw, _ = at(t)
        errors.append(abs(offset - (y - ROAD_Y + LOOKAHEAD * math.sin(yaw)) / math.cos(yaw)))
        both += two
    every = '  '.join(f'{at(t0 + k)[2] - ROAD_Y:+.2f}' for k in range(0, int(LANE_TIME) + 1, 3))
    print(f'      the offset (m), every 3 s of driving: {every}', flush=True)
    driven = stamps[-1] - t0
    ok = (driven >= LANE_TIME - 1.0 and settled <= LANE_TIME - 5.0 and over < 0.10
          and abs(end) < LANE_BAND and bool(errors) and both >= 0.9 * len(errors))
    finder = (f'lane_finder {100 * sum(errors) / len(errors):.1f} cm from the truth on average, '
              f'both lines in {both} of {len(errors)} pictures' if errors else 'no lane messages')
    result('lane keeping', ok,
           f'{start:+.2f} m to {end:+.2f} m, within {100 * LANE_BAND:.0f} cm after '
           f'{settled:.1f} s and {at(t0 + settled)[1] - x0:.1f} m, overshoot {100 * over:.0f} '
           f'cm, heading {heading:+.3f} rad, over {driven:.0f} s; {finder}')


def check_plan_around(probe, ahead=15.0, goal=30.0, depth=1.0):
    """
    Put the box `ahead` m in the lane: the car must drive round it, not stop for it.

    The planner sees the sensors (plan_around, the default), so it plans round the box; the
    car must reach the goal past it without the stop zone firing.
    """
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)
    centre = front + ahead + depth / 2.0
    bx, by = x0 + centre * c, y0 + centre * s
    if not gz_service('create', 'gz.msgs.EntityFactory',
                      f'sdf_filename: "{OBSTACLE}", name: "obstacle", '
                      f'pose: {{position: {{x: {bx}, y: {by}, z: 0}}, '
                      f'orientation: {{z: {math.sin(h / 2.0)}, w: {math.cos(h / 2.0)}}}}}'):
        result('plan around', False, "could not place the box (Gazebo's create service)")
        return
    start = len(probe.track)
    seen = len(probe.monitor)
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h)
    if done is None:
        result('plan around', False, 'no Nav2, or it refused the goal')
    else:
        probe.spin_for(240.0, done.done)
    gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')
    if done is None:
        return
    drive = probe.track[start:]
    # The car's centre line, base_link to the bumper, against the box's centre: less half
    # the car's width and half the box.
    gaps = []
    for _, x, y, yaw, _ in drive:
        ux, uy = math.cos(yaw), math.sin(yaw)
        k = max(0.0, min(front, (bx - x) * ux + (by - y) * uy))
        gaps.append(math.hypot(bx - x - k * ux, by - y - k * uy) - body['width'] / 2.0
                    - depth / 2.0)
    stood = stood_still(drive)
    stop_zone = any(a == CollisionMonitorState.STOP for a, _ in probe.monitor[seen:])
    reached = done.done() and done.result().status == 4
    x, y = probe.xy(probe.truth)
    passed = (x - x0) * c + (y - y0) * s > centre + depth
    clear = min(gaps) if gaps else float('nan')
    ok = reached and passed and not stop_zone and stood < 3.0 and clear > 0.3
    result('plan around', ok,
           f'{"reached" if reached else "did not reach"} the goal {goal:.0f} m on, '
           f'{clear:.2f} m clear of the box at the closest, stood still {stood:.1f} s at the '
           f'longest, stop zone fired: {stop_zone}')


def stood_still(track, still=0.02):
    """Return the longest stand-still (s) in a track of (t, x, y, yaw, v), once moving."""
    longest, since, moving = 0.0, None, False
    for t, _, _, _, v in track:
        moving = moving or v > 0.5
        if moving and abs(v) < still:
            since = t if since is None else since
            longest = max(longest, t - since)
        else:
            since = None
    return longest


def check_lidar_obstacle(probe):
    """Stop for a post only the lidar sees (the radar is off), and mark it in the costmap."""
    check_obstacle(probe, model=POST, depth=0.5, name='lidar obstacle', costmap=True)


def check_rear_obstacle(probe):
    """
    Back up to a goal 15 m behind, with a post 6 m behind the rear bumper: stop before it.

    The stop zone behind the car (it turns round with the command) and the lidar, which
    sees all round, must stop it; once the post is gone the car must back on to the goal.
    15 m: a reverse metre costs the planner 2, so a longer goal could make a forward loop
    cheaper than backing straight there.
    """
    check_obstacle(probe, ahead=6.0, goal=15.0, model=POST, depth=0.5, name='rear obstacle',
                   costmap=True, backwards=True)


def set_ghost(probe, client, **values):
    """Set ghost_obstacle's parameters, all at once; True if it took them."""
    sent = client.set_parameters_atomically([Parameter(k, value=v) for k, v in values.items()])
    probe.spin_for(10.0, sent.done)
    return sent.done() and sent.result().result.successful


def check_ghost(probe, ahead=15.0, goal=30.0):
    """
    Put a ghost (aav_ghost) `ahead` m past the bumper: not in Gazebo, only in Nav2's view.

    Nav2 must stop the car 0-2 m before it, and the local costmap must mark it; once the
    ghost is switched off, the car must go on to the goal, through where it stood. Either
    may stop it: collision_monitor's stop zone, or the controller's own collision check on
    the costmap (Regulated Pure Pursuit), which can stop it first. The gap is measured as
    Nav2 sees it: from the ghost's own scan, the bumper to its nearest point ahead.
    """
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']   # base_link to the front bumper
    client = AsyncParameterClient(probe, '/ghost_obstacle')
    if not client.wait_for_services(timeout_sec=20.0):
        result('ghost', False, 'no ghost_obstacle (navigation.launch.py ghost:=true)')
        return
    marker = {}
    marker_sub = probe.create_subscription(
        Marker, '/safety/ghost/marker',
        lambda m: marker.update(ghost=m) if m.action == Marker.ADD else marker.clear(), 10)
    scan = {}
    scan_sub = probe.create_subscription(LaserScan, '/safety/ghost/scan',
                                         lambda m: scan.update(newest=m),
                                         qos_profile_sensor_data)
    newest = {}     # the local costmap, in odom: on the world's origin, as the car started
    grid_sub = probe.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                         lambda m: newest.update(grid=m), 2)
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)

    def along():
        x, y = probe.xy(probe.truth)
        return (x - x0) * c + (y - y0) * s

    def gap_now():
        """Return the m from the bumper to the ghost, in its scan: in front, the car's width."""
        m = scan.get('newest')
        if m is None:
            return None
        r = numpy.asarray(m.ranges, dtype=float)
        a = m.angle_min + m.angle_increment * numpy.arange(len(r))
        x, y = r * numpy.cos(a), r * numpy.sin(a)
        ahead_ = numpy.isfinite(r) & (x > 0.0) & (numpy.abs(y) < body['width'] / 2.0)
        return float(x[ahead_].min()) - front if ahead_.any() else None

    placed = (set_ghost(probe, client, ahead=ahead, enabled=True)
              and probe.spin_for(10.0, lambda: 'ghost' in marker))
    if not placed:
        result('ghost', False, "ghost_obstacle didn't place the ghost")
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h) if placed else None
    if placed and done is None:
        result('ghost', False, 'no Nav2, or it refused the goal')
    seen, top, still, gap, peak, gone = len(probe.monitor), 0.0, None, None, None, False
    where = None        # the ghost's centre, in map
    end = time.monotonic() + 240.0
    while done is not None and time.monotonic() < end and not done.done():
        probe.spin_for(0.1)
        v = probe.truth.twist.twist.linear.x
        top = max(top, v)
        if gap is None and top > 0.5 and abs(v) < 0.02:
            still = still if still is not None else probe.now()
            if probe.now() - still > 3.0:      # stood still for 3 s: take the ghost away
                gap = gap_now()
                gap = gap if gap is not None else math.nan
                g = marker['ghost']
                where = (g.pose.position.x, g.pose.position.y)
                if 'grid' in newest:
                    peak = costmap_peak(newest['grid'], *where, g.scale.x / 2.0 + 0.5)
                gone = set_ghost(probe, client, enabled=False)
        else:
            still = None
    probe.destroy_subscription(marker_sub)
    probe.destroy_subscription(scan_sub)
    probe.destroy_subscription(grid_sub)
    if done is None:
        set_ghost(probe, client, enabled=False)
        return
    stop_zone = any(a == CollisionMonitorState.STOP and p == 'StopZone'
                    for a, p in probe.monitor[seen:])
    reached = done.done() and done.result().status == 4
    passed = along() > front + ahead + 0.5
    marked = peak is not None and peak >= 99
    ok = gap is not None and 0.0 < gap < 2.0 and marked and gone and reached and passed
    stood = f'stood still {gap:.2f} m before it' if gap is not None else 'never stopped'
    mark = f'the local costmap {peak} of 100 there' if peak is not None else 'no local costmap'
    result('ghost', ok, f'{stood} (top speed {top:.2f} m/s), stop zone fired: {stop_zone} '
                        f'(else the controller did); {mark}; switched off: {gone}, Nav2 goal '
                        f'{"reached" if reached else "not reached"} {along():.1f} m on')
    if gap is None:
        set_ghost(probe, client, enabled=False)


def node_pid(name='sensor_watchdog'):
    """
    Return a node's process id (the sensor watchdog's, or ego's), from the launch log, or None.

    Only a process in this run's own process group: never someone else's.
    """
    with open(LAUNCH['log']) as f:
        found = re.findall(rf'\[{name}-\d+\]: process started with pid \[(\d+)\]', f.read())
    if not found:
        return None
    pid = int(found[-1])
    try:
        return pid if os.getpgid(pid) == LAUNCH['pgid'] else None
    except ProcessLookupError:
        return None


def signal_watchdog(mode):
    """Send the watchdog its fault: hang (SIGSTOP), ok (SIGCONT), kill (SIGKILL); True if sent."""
    pid = node_pid()
    if pid is None:
        return False
    os.kill(pid, {'hang': signal.SIGSTOP, 'ok': signal.SIGCONT, 'kill': signal.SIGKILL}[mode])
    return True


def fault_client(probe, sensor):
    """Return a parameter client for the node that takes this sensor's fault."""
    clients = probe.__dict__.setdefault('fault_clients', {})
    node = FAULT_NODES[sensor]
    if node not in clients:
        clients[node] = AsyncParameterClient(probe, node)
    return clients[node]


def set_fault(probe, sensor, mode):
    """Set the sensor node's fault.<sensor>, or signal the watchdog; True if it took it."""
    if sensor == 'watchdog':
        return signal_watchdog(mode)
    if sensor == 'hud':
        probe.hud_beating = mode == 'ok'    # the screen froze, or draws again
        return True
    client = fault_client(probe, sensor)
    if not client.wait_for_services(timeout_sec=10.0):
        return False
    sent = client.set_parameters([Parameter(f'fault.{sensor}', value=mode)])
    probe.spin_for(10.0, sent.done)
    return sent.done() and all(r.successful for r in sent.result().results)


def drive_on(probe, speed=DRIVING):
    """
    Send Nav2 15 m on, along the road; wait until the car drives at `speed` m/s.

    East until x = 60 m, then west back to x = 0 (the map ends at 100 m): Nav2 turns the
    car round between the two, so the wait allows for a three-point turn.
    """
    x, y = probe.xy(probe.truth)
    facing_east = math.cos(yaw_of(probe.truth)) > 0.0
    east = x < 0.0 or (facing_east and x < 60.0)
    done = probe.go_to(x + (15.0 if east else -15.0), y, 0.0 if east else math.pi)
    if done is None:
        return None
    probe.spin_for(120.0, lambda: probe.truth.twist.twist.linear.x > speed or done.done())
    return done


def check_faults(probe, watchdog=True):
    """Each fault in FAULTS while Nav2 drives: the car must stop, and something say why."""
    if not fault_client(probe, 'odom').wait_for_services(timeout_sec=20.0):
        result('faults', False, "no gazebo_adapter parameters: can't set a fault")
        return
    clear = probe.create_client(Trigger, '/safety/sensor_watchdog/clear')
    probe.fail_safe = watchdog
    probe.hud_beating = True    # the screen draws, until its fault
    for sensor, mode in FAULTS:
        if sensor in ('watchdog', 'hud') and not watchdog:
            continue            # no watchdog to hang or kill, or to want the HUD
        name = f'fault {sensor} {mode}'
        if not probe.spin_for(30.0, lambda: not probe.locked()) and watchdog:
            result(name, False, f'the watchdog was still locked before it: {probe.state}')
            continue
        frozen = (sensor, mode) == ('odom', 'freeze')
        speed = FROZEN_AT if frozen else DRIVING
        done = drive_on(probe, speed)
        if done is not None and done.done():
            done = drive_on(probe, speed)   # it turned round and arrived: once more, ahead
        if done is None or probe.truth.twist.twist.linear.x <= speed:
            result(name, False, 'Nav2 would not drive before the fault')
            if done is not None:
                probe.goal.cancel_goal_async()
            continue
        seen = len(probe.monitor)
        x0, y0 = probe.xy(probe.truth)
        if not set_fault(probe, sensor, mode):
            result(name, False, f'{FAULT_NODES.get(sensor, sensor)} refused the fault')
            continue
        t0, stopped_at, locked, silent, why = probe.now(), None, False, False, ''
        # Frozen wheels at a speed the car keeps are still right: they go wrong only once
        # the car's speed or turn changes (wheel_check.py), so the clock starts then: once
        # they're wrong by more than the cross check's tolerances, as it measures them (the
        # speed against the GPS's; the turn rate as a mean over its window, so Nav2's small
        # weaving doesn't count).
        held_v, held_w = probe.truth.twist.twist.linear.x, probe.truth.twist.twist.angular.z
        turns = collections.deque()         # (t, the truth's turn rate) over the window
        wrong = None if frozen else 0.0     # s after the fault: the wheels' values are wrong
        react = FROZEN_REACT if frozen else REACT

        def waiting():
            if wrong is None:
                return probe.now() - t0 < FROZEN_WAIT
            return probe.now() - t0 < wrong + react + 4.0

        while waiting():
            probe.spin_for(0.1)
            locked = locked or probe.lock is True
            # Its words, once they say STOP: the lock can arrive a moment before them.
            if locked and not why and probe.state.startswith('STOP'):
                why = probe.state
            silent = silent or probe.silent()
            twist = probe.truth.twist.twist
            if wrong is None and frozen:
                turns.append((probe.now(), twist.angular.z))
                while turns[0][0] < probe.now() - CROSS['cross_window']:
                    turns.popleft()
                turn = sum(w for _, w in turns) / len(turns)
                if (abs(twist.linear.x - held_v) > CROSS['cross_gps_tolerance']
                        or abs(turn - held_w) > CROSS['cross_yaw_tolerance']):
                    wrong = probe.now() - t0
            if stopped_at is None and abs(twist.linear.x) < STILL:
                stopped_at = probe.now() - t0
        x1, y1 = probe.xy(probe.truth)
        on = math.hypot(x1 - x0, y1 - y0)
        gx, gy = probe.xy(probe.glob)
        ekf = math.hypot(gx - x1, gy - y1)
        invalid = any(a == CollisionMonitorState.STOP and p == 'invalid source'
                      for a, p in probe.monitor[seen:])
        by = ', '.join(w for w, on_ in (('the watchdog', locked),
                                        ("the watchdog's silence (twist_mux's "
                                         f'{LOCK_TIMEOUT:g} s lock timeout)', silent),
                                        ('collision_monitor (invalid source)', invalid)) if on_)
        stood = (f'stood still {stopped_at:.1f} s after it' if stopped_at is not None
                 else f'still driving after {probe.now() - t0:.0f} s')
        if frozen:
            stood += (f" (the wheels went wrong {wrong:.1f} s after it, as the car's speed or "
                      'turn changed)' if wrong is not None else ' (the wheels never went wrong)')
        detail = (f'{stood}, {on:.1f} m on; stopped by: {by or "nothing"}; the global EKF '
                  f'{ekf:.1f} m from the truth' + (f'; the watchdog said "{why}"' if why else ''))
        ok = (stopped_at is not None and wrong is not None and stopped_at - wrong <= react
              and bool(by))
        if mode == 'kill':
            # Nothing mends it: Nav2's goal still stands, and the car must stay still.
            top = 0.0
            end = probe.now() + HELD
            while probe.now() < end:
                probe.spin_for(0.1)
                top = max(top, abs(probe.truth.twist.twist.linear.x))
            held = top < STILL and probe.silent()
            ok = ok and held
            detail += (f'; held {HELD:.0f} s more with no watchdog, Nav2 still asking '
                       f'(top speed {top:.2f} m/s)')
            if not done.done():
                probe.goal.cancel_goal_async()
            result(name, ok, detail)
            continue
        # Mend it, and let the car's software settle before the next.
        set_fault(probe, sensor, 'ok')
        if sensor == 'hud':
            # Latched (Ben, 24 Oct): a HUD that beats again still holds Nav2, as hud LOST,
            # until a person clears it (below).
            probe.spin_for(5.0)
            held = probe.locked() and 'hud LOST' in probe.state
            ok = ok and held
            detail += ('; beating again, still held 5 s later (hud LOST) until cleared'
                       if held else f'; beating again, NOT held 5 s later: {probe.state}')
        if mode == 'jump' or (sensor, mode) == ('odom', 'freeze') or sensor == 'hud':
            # Latched: a person checks the position (or the wheels), then clears it.
            probe.spin_for(30.0, lambda: math.hypot(*(a - b for a, b in zip(
                probe.xy(probe.glob), probe.xy(probe.truth)))) < 2.0)
            probe.spin_for(3.0)
            if clear.wait_for_service(timeout_sec=5.0):
                cleared = clear.call_async(Trigger.Request())
                probe.spin_for(10.0, cleared.done)
        released = probe.spin_for(30.0, lambda: not probe.locked())
        if watchdog and not released:
            ok = False
            detail += f'; the lock never opened again: {probe.state}'
        if not done.done():
            probe.goal.cancel_goal_async()
        probe.spin_for(30.0, lambda: abs(probe.truth.twist.twist.linear.x) < STILL)
        if watchdog:
            result(name, ok, detail)
        else:
            print(f'INFO  {name}: {detail}', flush=True)
            RESULTS.append((name, None, detail))


def check_nav_goal(probe):
    """Nav2 drives 15 m on, along the car's heading: it must get there."""
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    gx, gy = x0 + 15.0 * math.cos(h), y0 + 15.0 * math.sin(h)
    done = probe.go_to(gx, gy, h)
    if done is None:
        result('Nav2 goal', False, 'no Nav2, or it refused the goal')
        return
    probe.spin_for(180.0, done.done)
    reached = done.done() and done.result().status == 4
    if not done.done():
        probe.goal.cancel_goal_async()
    x1, y1 = probe.xy(probe.truth)
    result('Nav2 goal', reached, f'{"reached" if reached else "not reached"}: '
                                 f'{math.hypot(x1 - x0, y1 - y0):.1f} m on, '
                                 f'{math.hypot(gx - x1, gy - y1):.2f} m from the goal')


def finish(probe, handle):
    """Cancel a scripted drive and wait until it has stopped: one drive at a time."""
    result = handle.get_result_async()
    handle.cancel_goal_async()
    probe.spin_for(15.0, result.done)


def get_going(probe, spine):
    """Arm the spine if need be, start a scripted drive, wait for 0.8 m/s; its handle or None."""
    for _ in range(2):
        if spine and not probe.armed():
            probe.arm_and_hold()
        handle = probe.send_drive()
        if handle is None:
            return None
        if probe.spin_for(30.0, lambda: probe.speed() > 0.8):
            return handle
        # The spine latched before the car got going (a gap in commands on a busy
        # computer), which isn't what these check: once more.
        finish(probe, handle)
    return None


def measure_stop(probe, t0, press=None):
    """
    Return (s to the brake, s to standing still) after t0 (Gazebo's time), None if never.

    The brake: the first demand after t0 with no throttle and some brake. press: an e-stop
    publisher to keep pressing meanwhile.
    """
    def still():
        if press is not None:
            press.publish(Bool(data=True))
        return seconds(probe.truth.header.stamp) > t0 and abs(probe.speed()) < STILL

    probe.spin_for(30.0, still)
    stood = seconds(probe.truth.header.stamp) - t0 if still() else None
    brake = next((t - t0 for t, throttle, b in probe.demands
                  if t >= t0 and throttle == 0.0 and b > 0.0), None)
    return brake, stood


def held(probe, seconds_, press=None):
    """Spin seconds_ of Gazebo's time; return the top speed meanwhile (m/s)."""
    top, end = 0.0, probe.now() + seconds_
    while probe.now() < end:
        if press is not None:
            press.publish(Bool(data=False))
        probe.spin_for(0.1)
        top = max(top, abs(probe.speed()))
    return top


def stop_detail(brake, stood, v0, limit):
    b = f'braked {1000 * brake:.0f} ms after it' if brake is not None else 'never braked'
    s = f'stood still {stood:.2f} s after it' if stood is not None else 'never stood still'
    return f'from {v0:.2f} m/s: {b} (limit {1000 * limit:.0f} ms), {s}'


def stop_ok(brake, stood, limit):
    return brake is not None and brake <= limit and stood is not None and stood <= STOP_TIME


def rearm(probe, top):
    """After a latched stop: the car stayed still (top m/s); arm again. Return the words."""
    status = probe.status
    latched = status is not None and not status.armed
    cleared = probe.arm_and_hold()
    return (latched and top < STILL and cleared,
            f'latched: {latched} (top speed {top:.2f} m/s for {LATCH_HOLD:g} s, its drive '
            f'still asking), armed again: {cleared}')


def check_command_lost(probe, spine):
    """Stop ego's process while driving: the car must brake and stop; the spine latches."""
    name = 'command stream lost' + ('' if spine else ' (direct path)')
    limit = SPINE_BRAKE if spine else DIRECT_BRAKE
    pid = node_pid('ego')
    handle = pid and get_going(probe, spine)
    if not handle:
        result(name, False, "no ego process in this run's log" if not pid
               else 'the car would not get going')
        return
    v0, t0 = probe.speed(), probe.now()
    os.kill(pid, signal.SIGSTOP)        # ego's own process, in this run's process group
    try:
        brake, stood = measure_stop(probe, t0)
        if not spine:
            finish(probe, handle)
    finally:
        os.kill(pid, signal.SIGCONT)
    detail = stop_detail(brake, stood, v0, limit)
    ok = stop_ok(brake, stood, limit)
    if spine:
        # ego is back and the drive still asks: the spine must hold until armed again.
        timeout = probe.status is not None and bool(
            probe.status.faults & VehicleStatus.FAULT_CMD_TIMEOUT)
        top = held(probe, LATCH_HOLD)
        finish(probe, handle)
        good, words = rearm(probe, top)
        ok = ok and good and timeout
        detail += f'; CMD_TIMEOUT: {timeout}; {words}'
    STOPS[('command stream lost', 'spine' if spine else 'direct')] = (brake, stood, v0)
    result(name, ok, detail)


def check_estop(probe, spine, hardware=False):
    """Press an e-stop while driving: the car must brake and stop; the spine latches."""
    stop = 'hardware e-stop' if hardware else 'software e-stop'
    name = stop + ('' if spine else ' (direct path)')
    publisher = probe.hw_estop if hardware else probe.estop
    handle = get_going(probe, spine)
    if handle is None:
        result(name, False, 'the car would not get going')
        return
    v0, t0 = probe.speed(), probe.now()
    publisher.publish(Bool(data=True))
    brake, stood = measure_stop(probe, t0, press=publisher)
    limit = ESTOP_BRAKE if spine else DIRECT_BRAKE + 0.5
    detail = stop_detail(brake, stood, v0, limit)
    ok = stop_ok(brake, stood, limit)
    if spine:
        reported = probe.status is not None and probe.status.estop_active
        # Let go, the drive still asking: still latched until armed again.
        top = held(probe, LATCH_HOLD, press=publisher)
        finish(probe, handle)
        good, words = rearm(probe, top)
        ok = ok and reported and good
        detail += f'; the spine reported it: {reported}; let go, {words}'
    else:
        finish(probe, handle)
        held(probe, 1.0, press=publisher)
    STOPS[(stop, 'spine' if spine else 'direct')] = (brake, stood, v0)
    result(name, ok, detail)


def check_not_armed(probe):
    """Disarm the spine: a scripted drive must not move the car, nor the throttle open."""
    disarmed = probe.arm(False) and probe.spin_for(5.0, lambda: not probe.armed())
    t0 = probe.now()
    handle = probe.send_drive(hold=3.0)
    top = held(probe, 8.0)
    if handle is not None:
        finish(probe, handle)
    throttle = max((t for stamp, t, _ in probe.demands if stamp >= t0), default=None)
    status = probe.status
    ok = (disarmed and handle is not None and top < STILL and throttle == 0.0
          and not status.armed)
    rearmed = probe.arm_and_hold()
    result('not armed', ok and rearmed,
           f'disarmed: {disarmed}, a drive asked for 1 m/s for 8 s: top speed {top:.2f} m/s, '
           f"the spine's top throttle {throttle}, mode {status.mode} (0 = manual); "
           f'armed again: {rearmed}')


def armed_first(check):
    """With the spine in the loop: arm it again first if a gap in commands latched it."""
    def run(probe):
        if not probe.armed():
            probe.arm_and_hold()
        check(probe)
    return run


def print_stops():
    """Print the stop times: the direct path beside the spine in the loop."""
    if not STOPS:
        return

    def cell(key):
        if key not in STOPS:
            return 'not run'
        brake, stood, v0 = STOPS[key]
        b = f'{1000 * brake:.0f} ms' if brake is not None else 'never'
        s = f'{stood:.2f} s' if stood is not None else 'never'
        return f'brake {b}, still {s} (from {v0:.2f} m/s)'

    print('\n| Stop | Direct path | Spine in the loop |\n|---|---|---|')
    for stop in ('command stream lost', 'software e-stop', 'hardware e-stop'):
        print(f'| {stop} | {cell((stop, "direct"))} | {cell((stop, "spine"))} |')


# The flags' names before 26 Oct, still accepted (aav_goto became aav_drive_to_point).
OLD_FLAGS = {'--goto': '--drive-to-point', '--goto-only': '--drive-to-point-only'}
# --drive-to-point: drive to points and stop there (aav_drive_to_point). The same targets
# as test_drive.py: metres away, and degrees to the left of the car's nose, each from
# where the last stopped.
POINT_TARGETS = [(10.0, 0.0), (30.0, 45.0), (60.0, -30.0), (30.0, 150.0)]
POINT_SPEEDS = (1.0, 3.0)       # m/s: the car's ego cap, and Gazebo's sim_speed 3
SPEED_STEPS = (-0.5, 1.0, 2.0)  # m/s: TODO 27's steps (reversing is capped at 0.5)
SPEED_HOLD = 20.0               # s at each step
SPEED_BAND = 0.05               # the band a step must end, and settle, in (of its target)
SPEED_TRIES = 3                 # drives of a step, while late commands spoil them
# Each step's samples (profile, target, try, s, m/s), for a plot: AAV_SIM_SPEED_LOG moves it.
SPEED_LOG = os.environ.get('AAV_SIM_SPEED_LOG', '/tmp/aav-sim-speed-steps.csv')
# Each drive to point's truth, as an atob-score CSV: AAV_SIM_POINT_LOG moves the folder.
POINT_LOG = os.environ.get('AAV_SIM_POINT_LOG', '/tmp/aav-sim-points')
POINT_TOLERANCE = 2.0           # m from B, by Gazebo's truth: docs/tests.md 5.2 without RTK
POINT_PAST = 0.5                # m past B, along the car's heading, by its own EKF: GPS noise
POINT_DECEL = (0.6, 1.25)       # x the planned deceleration (half max_brake): the band
# m/s: the band's floor counts from this top speed. Slower, the stop lasts under a second
# (0.7 s from 1 m/s), and the brake's and speed loop's lags smooth it below the plan: only
# "never harder" counts there.
POINT_DECEL_FROM = 2.0
# m/s^2, as drive_to_point.yaml
POINT_PLAN = 0.5 * float(vehicle.load()['actuators']['max_brake'])
# A drive the watchdog's lock or a stale position stopped (a busy computer stalls every
# sensor at once) is sent again, from where it stopped, this many times; the table says so.
POINT_RESENDS = 2
STALL_WORDS = ("watchdog's lock", 'is stale')


def ahead_of(msg, distance, degrees):
    """Return the map point `distance` m away, `degrees` left of the car's nose."""
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
    a = yaw_of(msg) + math.radians(degrees)
    return x + distance * math.cos(a), y + distance * math.sin(a)


def slowing(track, t0, window=0.5):
    """
    Return (top speed, the hardest deceleration over `window` s, m/s^2) after t0.

    From Gazebo's truth, over the final stop only: from the last moment at 90 % of the top
    speed. As test_drive.py's: a window smooths the truth's speed, which is a difference.
    """
    run = [s for s in track if s[0] >= t0]
    if len(run) < 2:
        return 0.0, math.nan
    top = max(s[4] for s in run)
    i = max(k for k, s in enumerate(run) if s[4] >= 0.9 * top)
    hardest, j = math.nan, i
    for a in range(i, len(run)):
        j = max(j, a)
        while j < len(run) and run[j][0] - run[a][0] < window:
            j += 1
        if j == len(run):
            break
        d = (run[a][4] - run[j][4]) / (run[j][0] - run[a][0])
        hardest = d if math.isnan(hardest) else max(hardest, d)
    return top, hardest


def save_point_run(track, t0, speed, b):
    """
    Write one drive's truth as an atob-score CSV: POINT_LOG/<speed>-<n>.csv.

    `pixi run atob-score /tmp/aav-sim-points/*.csv --speed <speed>` scores them (stop error,
    overshoot, hold, jerk). The folder is emptied at the start of each sim-check.
    """
    os.makedirs(POINT_LOG, exist_ok=True)
    n = len([f for f in os.listdir(POINT_LOG) if f.startswith(f'{speed:g}-')]) + 1
    with open(os.path.join(POINT_LOG, f'{speed:g}-{n}.csv'), 'w') as out:
        out.write('stream,t,x,y,v,phase,left,cross\n')
        out.write(f'b,,{b[0]:.3f},{b[1]:.3f},,,,\n')
        for t, x, y, _, v in (s for s in track if s[0] >= t0):
            out.write(f'truth,{t:.3f},{x:.3f},{y:.3f},{v:.4f},,,\n')


def send_point_goal(probe, goal, wait=300.0):
    """Send /drive_to_point a goal; return its result (None if refused or none) and handle."""
    if not probe.drive_to_point.wait_for_server(timeout_sec=20.0):
        return None, None
    sent = probe.drive_to_point.send_goal_async(goal)
    probe.spin_for(10.0, sent.done)
    if not sent.done() or not sent.result().accepted:
        return None, None
    handle = sent.result()
    done = handle.get_result_async()
    probe.spin_for(wait, done.done)
    return (done.result() if done.done() else None), handle


def point_drive(probe, speed, distance, degrees, lat_lon=False):
    """Drive to one point; check the stop against Gazebo's truth, the braking and overshoot."""
    name = f'drive to point {distance:g} m at {degrees:+g} deg, {speed:g} m/s' + (
        ' (lat/lon)' if lat_lon else '')
    bx, by = ahead_of(probe.glob, distance, degrees)
    goal = DriveToPoint.Goal(x=bx, y=by, cruise_speed=float(speed))
    where = f'B ({bx:.1f}, {by:.1f})'
    if lat_lon:
        if not probe.to_ll.wait_for_service(timeout_sec=10.0):
            result(name, False, 'no /toLL from navsat_transform')
            return
        req = ToLL.Request()
        req.map_point.x, req.map_point.y = bx, by
        ll = probe.to_ll.call_async(req)
        probe.spin_for(10.0, ll.done)
        if not ll.done():
            result(name, False, '/toLL did not answer')
            return
        p = ll.result().ll_point
        goal = DriveToPoint.Goal(use_lat_lon=True, latitude=p.latitude, longitude=p.longitude,
                                 cruise_speed=float(speed))
        where = f'B lat {p.latitude:.7f} lon {p.longitude:.7f}'
    stalls = []
    for _ in range(1 + POINT_RESENDS):
        t0 = probe.now()
        res, _ = send_point_goal(probe, goal, wait=60.0 + 6.0 * distance / speed)
        probe.spin_for(10.0, lambda: abs(probe.truth.twist.twist.linear.x) < STILL)
        if res is None:
            result(name, False, f'{where}: refused, or no answer')
            return
        r = res.result
        if r.success or not any(w in r.message for w in STALL_WORDS):
            break
        # A busy computer stalls every sensor at once: the watchdog locks, the driver stops,
        # as they must. Once the lock opens, the same B again, from where it stopped.
        stalls.append(r.message.split(': ', 1)[-1])
        probe.spin_for(30.0, lambda: not probe.locked())
    tx, ty = probe.xy(probe.truth)
    error = math.hypot(tx - r.target_x, ty - r.target_y)
    gx, gy = probe.xy(probe.glob)
    yaw = yaw_of(probe.glob)
    past = (gx - r.target_x) * math.cos(yaw) + (gy - r.target_y) * math.sin(yaw)
    yaw = yaw_of(probe.truth)
    truly = (tx - r.target_x) * math.cos(yaw) + (ty - r.target_y) * math.sin(yaw)
    top, decel = slowing(probe.track, t0)
    save_point_run(probe.track, t0, speed, (r.target_x, r.target_y))
    lo, hi = POINT_DECEL
    floor = lo * POINT_PLAN if top >= POINT_DECEL_FROM else 0.0
    ok = (r.success and error <= POINT_TOLERANCE and past <= POINT_PAST
          and floor <= decel <= hi * POINT_PLAN)
    again = (f'; sent again after {len(stalls)} stall(s) that stopped it: '
             f'{"; ".join(stalls)}' if stalls else '')
    result(name, ok,
           f'{where}: stopped {error:.2f} m from B by the truth ({r.distance_left:.2f} m by '
           f'its EKF), {past:+.2f} m past it by its EKF ({truly:+.2f} by the truth); top '
           f'{top:.2f} m/s, slowed at {decel:.2f} '
           f'm/s^2 at most (plan {POINT_PLAN:g}); {r.distance_travelled:.1f} m in '
           f'{r.duration:.0f} s; "{r.message}"{again}')


def point_stops(probe, name, stop, mend=None, react=4.0):
    """Start a 40 m drive, then `stop(probe)` once it moves: it must stand still within react s."""
    bx, by = ahead_of(probe.glob, 40.0, 0.0)
    if not probe.drive_to_point.wait_for_server(timeout_sec=20.0):
        result(name, False, 'no /drive_to_point')
        return
    sent = probe.drive_to_point.send_goal_async(DriveToPoint.Goal(x=bx, y=by, cruise_speed=1.0))
    probe.spin_for(10.0, sent.done)
    if not sent.done() or not sent.result().accepted:
        result(name, False, 'refused')
        return
    handle = sent.result()
    done = handle.get_result_async()
    moving = probe.spin_for(60.0, lambda: probe.truth.twist.twist.linear.x > 0.8)
    if not moving:
        handle.cancel_goal_async()
        result(name, False, 'it never got going')
        return
    x0, y0 = probe.xy(probe.truth)
    stop(probe, handle)
    t0 = probe.now()
    still = probe.spin_for(60.0, lambda: abs(probe.truth.twist.twist.linear.x) < STILL)
    took = probe.now() - t0
    probe.spin_for(15.0, done.done)
    message = done.result().result.message if done.done() else 'no result'
    # It must stay stopped: a few seconds more, nothing moves.
    end, top = probe.now() + 3.0, 0.0
    while probe.now() < end:
        probe.spin_for(0.1)
        top = max(top, abs(probe.truth.twist.twist.linear.x))
    on = math.hypot(*(a - b for a, b in zip(probe.xy(probe.truth), (x0, y0))))
    if mend is not None:
        mend(probe)
    if not done.done():
        handle.cancel_goal_async()
    ok = still and took <= react and top < STILL and done.done()
    result(name, ok, f'stood still {took:.1f} s after it, {on:.1f} m on, then held (top '
                     f'{top:.2f} m/s); the driver said "{message}"')


def gps_fault(mode):
    def set_it(probe, *_):
        set_fault(probe, 'gps', mode)
    return set_it


def estop(on):
    def press(probe, *_):
        for _ in range(5):
            probe.estop.publish(Bool(data=on))
            probe.spin_for(0.1)
    return press


def check_point_refusals(probe):
    """Check it refuses a B over max_distance away (drive_to_point.yaml: 200 m)."""
    bx, by = ahead_of(probe.glob, 250.0, 0.0)
    far, _ = send_point_goal(probe, DriveToPoint.Goal(x=bx, y=by), wait=20.0)
    refused = far is not None and not far.result.success and 'over the limit' in \
        far.result.message
    result('drive to point refuses 250 m', refused,
           f'"{far.result.message}"' if far is not None else 'no answer')


def check_drive_to_point(probe, speed, stops=True):
    """Drive POINT_TARGETS at this speed (Gazebo's sim_speed); at 1 m/s, its stops too."""
    for distance, degrees in POINT_TARGETS:
        point_drive(probe, speed, distance, degrees)
    if stops and speed == POINT_SPEEDS[0]:
        point_drive(probe, speed, 20.0, 0.0, lat_lon=True)
        check_point_refusals(probe)
        point_stops(probe, 'drive to point: cancel (Ctrl-C) stops it',
                    lambda p, handle: handle.cancel_goal_async())
        point_stops(probe, 'drive to point: the e-stop stops it', estop(True), mend=estop(False))
        # The GPS stops: the driver's own gps_timeout (2 s) and the watchdog's lock (30).
        point_stops(probe, "drive to point: a GPS fault stops it (watchdog's lock)",
                    gps_fault('stop'), mend=gps_fault('ok'), react=6.0)
        probe.spin_for(30.0, lambda: not probe.locked())


def step_numbers(samples, target):
    """
    Return (overshoot, time to settle, end) of one step: samples (t, speed), in time order.

    Overshoot: past |target|, as a fraction. Settle: from the moment the car moves until the
    speed stays within SPEED_BAND of the target (None if it never does). End: the mean
    speed of the last 3 s. Times from the first move: before it, only the system's delay.
    """
    a = abs(target)
    moving = [(t, abs(v)) for t, v in samples if abs(v) > 0.02]
    if not moving:
        return None, None, None
    t0, end_t = moving[0][0], moving[-1][0]
    overshoot = max(0.0, max(v for _, v in moving) / a - 1.0)
    settle = t0
    for t, v in moving:
        if abs(v - a) > SPEED_BAND * a:
            settle = t
    last = [v for t, v in moving if t >= end_t - 3.0]
    settled = settle - t0 if settle < end_t - 1.0 else None
    return overshoot, settled, sum(last) / len(last)


def dropouts(demands, samples, target, start, end):
    """
    Return how many brake demands came while the car was below its target.

    Below the target (and its coast band) the speed loop never brakes: only a command
    that arrived too late does (gazebo_adapter's cmd_timeout, 0.2 s: the target drops
    to 0). That is the computer's load, not the loop: such a step is driven again.
    """
    count = 0
    for stamp, _, brake in demands:
        if start <= stamp <= end and brake > 0.0:
            i = bisect.bisect_left(samples, (stamp,))
            near = samples[min(i, len(samples) - 1)][1] if samples else 0.0
            count += abs(near) < abs(target) + 0.05     # the loop brakes past + 0.2
    return count


def speed_step(probe, target):
    """Drive one held step; return (its samples, from its start; brake dropouts)."""
    probe.spin_for(30.0, lambda: abs(probe.speed()) < 0.02)      # from a standstill
    # Times on Gazebo's truth's stamps, the samples' own clock: the scripted driver's
    # phases as the latest truth sample when each was heard.
    start = probe.track[-1][0]
    slowing = []

    def on_feedback(msg):
        if msg.feedback.phase == ScriptedDrive.Feedback.PHASE_DECELERATE and not slowing:
            slowing.append(probe.track[-1][0])

    sent = probe.drive.send_goal_async(ScriptedDrive.Goal(
        target_speed=target, acceleration=1.0, deceleration=1.0, hold_time=SPEED_HOLD),
        feedback_callback=on_feedback)
    probe.spin_for(10.0, lambda: sent.done())
    if not sent.done() or not sent.result().accepted:
        return None, 0
    done = sent.result().get_result_async()
    probe.spin_for(5.0 * SPEED_HOLD, lambda: done.done())
    end = slowing[0] if slowing else probe.track[-1][0]
    samples = [(t, v) for t, _, _, _, v in probe.track if start <= t <= end]
    # Until the car moves, the hold brake is still on: not a late command.
    moved = next((t for t, v in samples if abs(v) > 0.02), end)
    return [(t - start, v) for t, v in samples], dropouts(probe.demands, samples, target,
                                                          moved + 0.5, end)


def check_speed_steps(probe, profile):
    """Hold each of SPEED_STEPS with a scripted drive; check its end, report its overshoot."""
    if not probe.drive.wait_for_server(timeout_sec=20.0):
        result('speed steps', False, 'no scripted driver')
        return
    for target in SPEED_STEPS:
        name = f'speed step {target:+g} m/s ({profile})'
        for attempt in range(1, SPEED_TRIES + 1):
            samples, late = speed_step(probe, target)
            if samples is None or not late:
                break
        if samples is None:
            result(name, False, 'refused')
            continue
        with open(SPEED_LOG, 'a') as log:
            log.writelines(f'{profile},{target:g},{attempt},{t:.3f},{v:.4f}\n'
                           for t, v in samples)
        overshoot, settle, end = step_numbers(samples, target)
        if end is None:
            result(name, False, "the car never moved (Gazebo's truth)")
            continue
        tries = f' ({attempt} tries: late commands braked the others)' if attempt > 1 else ''
        peak = max(abs(v) for _, v in samples)
        within = (f'within 5 % {settle:.1f} s after it moved' if settle is not None
                  else 'never within 5 %')
        detail = (f'peak {peak:.3f} m/s: overshoot {100 * overshoot:.1f} %, {within}, '
                  f'{end:.3f} m/s at the end')
        if late:
            # Every try was spoiled by the computer's load: not the loop's fault.
            result(name, None, f'{detail}; {late} late-command brakes in each of '
                               f'{SPEED_TRIES} tries: the computer is too busy to judge it')
            continue
        ends_right = abs(end - abs(target)) <= SPEED_BAND * abs(target)
        smooth_ok = profile != 'smooth' or overshoot < 0.03
        result(name, ends_right and smooth_ok, detail + tries)


LIDAR_TOPICS = ('/sensors/lidar/points', '/sensors/lidar/scan')
WATCHDOG_TOPICS = ('/safety/sensor_watchdog/lock', '/safety/sensor_watchdog/state')


EVERY_RUN = []   # launch arguments every session adds: --speed-loop's


def session(launch_args, checks, log_path, label='', ignore=(), radar=True, spine=False,
            world='road', retry=True):
    """
    Start sim.launch.py's world with these arguments, run the checks, and stop it.

    retry: if Gazebo itself crashes while starting (gz_crash), start once more.

    spine: the spine is in the loop (spine:=true): its status must come, and it must arm.
    world: road (the camera and radar must run, the road profile) or empty (only the truth
    and the global EKF, the sim profile).
    """
    road = world == 'road'
    radar = radar and road
    watchdog = 'watchdog:=false' not in launch_args
    tag = f' ({label})' if label else ''
    # What a run leaves out on purpose isn't missing from the graph.
    ignore = (tuple(ignore) + (LIDAR_TOPICS if 'lidar:=false' in launch_args else ())
              + (WATCHDOG_TOPICS if 'watchdog:=false' in launch_args else ()))
    env = dict(os.environ)
    print(f"Starting Gazebo's {world} world{tag} on ROS domain {env['ROS_DOMAIN_ID']}, Gazebo "
          f"partition {env['GZ_PARTITION']} (this computer only)...", flush=True)
    log = open(log_path, 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py', f'world:={world}',
                               'gui:=false', 'rviz:=false', *launch_args, *EVERY_RUN],
                              cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    LAUNCH.update(pgid=launch.pid, log=log_path)
    rclpy.init()
    probe = Probe()
    again = False
    try:
        def missing():
            return [n for n, m in (('truth', probe.truth), ('global EKF', probe.glob),
                                   ('camera', probe.image if road else True),
                                   ('lens', probe.info if road else True),
                                   ('radar', probe.cloud if radar else True),
                                   # Its heartbeat: without it twist_mux holds the drivers.
                                   ('the sensor watchdog', probe.lock if watchdog else True))
                    if m is None]

        up = probe.spin_for(120.0, lambda: not missing())
        # Nav2 comes up last: its lifecycle manager starts its servers one at a time. On a
        # busy computer it can stall on a lost reply, and then Nav2 never comes up at all.
        nav = 'nav:=false' not in launch_args
        nav_up = up and (not nav or probe.spin_for(120.0, probe.nav.server_is_ready))
        if not up:
            log.flush()
            with open(log_path) as text:
                crash = gz_crash.gazebo_crashed(text.read())
            if crash and retry:
                result(f'start-up{tag}', None, f'Gazebo crashed while starting ({crash}): '
                       f'once more (log: {log_path})')
                again = True
                return
            detail = f'nothing from: {", ".join(missing())}, in 120 s' + (
                f' (Gazebo: {crash}, twice)' if crash else '')
        elif not nav_up:
            detail = ('the sensors run, but Nav2 never came up in 120 s (its lifecycle '
                      f'manager stalled? See {log_path})')
        else:
            detail = ("Gazebo's truth, the global EKF" + (', the camera' if road else '') +
                      (' and the radar stand-in are' if radar else ' are (no radar)') +
                      ' running' + (', and Nav2 is up' if nav else ''))
        if nav_up and spine:
            # The launch arms it after 12 s (arm:=true); a gap in commands may latch it.
            nav_up = probe.spin_for(30.0, lambda: probe.status is not None) and (
                probe.armed() or probe.arm_and_hold())
            detail += ('; the spine is armed' if nav_up
                       else '; but the spine would not stay armed')
        result(f'start-up{tag}', nav_up, detail)
        up = nav_up
        if up:
            # The spine profile is the road world's and more: in the empty world (the
            # --spine-steps run) only the sim profile's topics can be live.
            check_contract(probe, env, tag, ignore,
                           'spine' if spine and road else 'road' if road else 'sim')
            for check in checks:
                check(probe)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        log.close()
        if again:
            # Its own log: the crashed one stays for a look.
            stem, ext = os.path.splitext(log_path)
            session(launch_args, checks, f'{stem}-retry{ext}', label, ignore, radar, spine,
                    world, retry=False)


def main():
    # --goto and --goto-only: the old names (before 26 Oct), still accepted.
    args = [OLD_FLAGS.get(a, a) for a in sys.argv[1:]]
    if '-h' in args or '--help' in args:
        print(__doc__)
        return 0
    perception = '--perception' in args
    if perception and importlib.util.find_spec('rfdetr') is None:   # the detector's default
        print('--perception needs the detector: pixi run -e ml sim-check --perception',
              file=sys.stderr)
        return 2
    lidar = '--no-lidar' not in args
    checks = ([check_sensors, check_cameras] + ([check_lidar, check_lidar_scan] if lidar else [])
              + [check_drive])
    if perception:
        checks.append(check_stop_sign)
    if '--obstacle' in args:
        checks.append(check_obstacle)
    checks.append(check_wheels_vs_imu)
    spine = '--spine' in args
    if spine:
        # The direct path's stop times, to compare with the spine's below.
        checks += [lambda probe: check_command_lost(probe, False),
                   lambda probe: check_estop(probe, False)]
    profile = 'default'
    if '--speed-loop' in args:
        at = args.index('--speed-loop')
        if at + 1 >= len(args) or args[at + 1] not in ('default', 'smooth', 'lqi'):
            print('--speed-loop takes default, smooth or lqi', file=sys.stderr)
            return 2
        profile = args[at + 1]
        EVERY_RUN.append(f'speed_loop:={profile}')
    if '--esp32' in args:
        at = args.index('--esp32')
        if at + 1 >= len(args) or args[at + 1] not in ('pi', 'open'):
            print('--esp32 takes pi or open', file=sys.stderr)
            return 2
        if spine:
            print('--esp32 and --spine are two backends: pick one', file=sys.stderr)
            return 2
        EVERY_RUN.extend(['esp32:=true', f'esp32_loop:={args[at + 1]}'])
    log = os.environ.get('AAV_SIM_LOG', '/tmp/aav-sim-check.log')
    logs = [log]
    if not {'--drive-to-point-only', '--speed-steps-only', '--spine-points-only'} & set(args):
        # With --obstacle the planner is blind to the box, so the stop zone must stop the car.
        session([f'perception:={str(perception).lower()}', f'lidar:={str(lidar).lower()}']
                + (['plan_around:=false'] if '--obstacle' in args else []),
                checks, logs[0])
    else:
        logs = []
    if '--lidar-obstacle' in args:
        # Its own run, with the radar off: only the lidar can stop the car for the post.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-lidar{ext}')
        session(['radar:=false', 'lidar:=true', 'plan_around:=false'], [check_lidar_obstacle],
                logs[-1], 'lidar', ignore=RADAR_TOPICS, radar=False)
    if '--rear-obstacle' in args:
        # Its own run, as --lidar-obstacle: the radar off, and the post behind the car.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-rear{ext}')
        session(['radar:=false', 'lidar:=true', 'plan_around:=false', 'reversing:=true'],
                [check_rear_obstacle], logs[-1], 'rear', ignore=RADAR_TOPICS, radar=False)
    if '--plan-around' in args:
        # Its own run: the planner sees the radar and lidar (the default), so it plans round.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-around{ext}')
        session([f'lidar:={str(lidar).lower()}'], [check_plan_around], logs[-1], 'around')
    if '--ghost' in args:
        # Its own run: the ghost is one more collision_monitor source, beside the radar and
        # the lidar, so the other runs keep the graph the car has.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-ghost{ext}')
        session(['ghost:=true', f'lidar:={str(lidar).lower()}'], [check_ghost], logs[-1],
                'ghost')
    if '--faults' in args:
        # Its own run: a fault spoils the car's software for a while, and the camera's
        # picture goes through gazebo_adapter (faults:=true). No lidar: a lighter run.
        watchdog = '--no-watchdog' not in args
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-faults{ext}')
        session(['faults:=true', 'lidar:=false', f'watchdog:={str(watchdog).lower()}',
                 f'watch_hud:={str(watchdog).lower()}'],
                [lambda probe: check_faults(probe, watchdog)], logs[-1], 'faults')
    if {'--drive-to-point', '--drive-to-point-only', '--spine-points',
            '--spine-points-only'} & set(args):
        shutil.rmtree(POINT_LOG, ignore_errors=True)
    if '--drive-to-point' in args or '--drive-to-point-only' in args:
        # Its own runs, one per speed (sim_speed is ego's cap, set at launch): the empty
        # world, so any angle is open ground. No Nav2, lidar or radar: lighter runs.
        stem, ext = os.path.splitext(log)
        for speed in POINT_SPEEDS:
            logs.append(f'{stem}-drive-to-point-{speed:g}{ext}')
            session(['nav:=false', 'lidar:=false', 'radar:=false', f'sim_speed:={speed:g}'],
                    [lambda probe, v=speed: check_drive_to_point(probe, v)], logs[-1],
                    f'drive to point {speed:g} m/s', ignore=('/map',), world='empty')
    if '--spine-points' in args or '--spine-points-only' in args:
        # The same drives with the spine in the loop (spine:=true): spine-core's own speed
        # loop stops the car. Only the drives: an e-stop would latch the spine.
        stem, ext = os.path.splitext(log)
        for speed in POINT_SPEEDS:
            logs.append(f'{stem}-spine-drive-to-point-{speed:g}{ext}')
            session(['spine:=true', 'nav:=false', 'lidar:=false', 'radar:=false',
                     f'sim_speed:={speed:g}'],
                    [lambda probe, v=speed: check_drive_to_point(probe, v, stops=False)],
                    logs[-1], f'drive to point, spine, {speed:g} m/s', ignore=('/map',),
                    spine=True, world='empty')
    if '--speed-steps' in args or '--speed-steps-only' in args:
        # Its own run: the empty world, nothing else driving (no Nav2), ego's cap at 2 m/s.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-speed{ext}')
        session(['nav:=false', 'lidar:=false', 'radar:=false', 'sim_speed:=2'],
                [lambda probe: check_speed_steps(probe, profile)], logs[-1],
                f'speed steps, {profile}', ignore=('/map',), world='empty')
    if '--spine-steps' in args:
        # The same steps with the spine in the loop (spine:=true): spine_sim runs
        # spine-core's own speed loop (Rust), not the actuator model's Python port.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-spine-speed{ext}')
        session(['spine:=true', 'nav:=false', 'lidar:=false', 'radar:=false', 'sim_speed:=2'],
                [lambda probe: check_speed_steps(probe, profile)], logs[-1],
                f'speed steps, spine, {profile}', ignore=('/map',), spine=True, world='empty')
    if '--lane' in args:
        # Its own run: control_interface drives whenever it sees the lane, so it would take
        # the wheel between the other checks. Nothing else drives here: no Nav2.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-lane{ext}')
        session(['lane:=true', 'nav:=false', 'lidar:=false', f'start_y:={ROAD_Y + LANE_START}'],
                [check_lane_keeping], logs[-1], 'lane', ignore=('/map',))

    if spine:
        # Its own run: the spine in the loop (spine:=true). No lidar: a lighter run.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-spine{ext}')
        # --python-bridge: aav_spine's Python spine_bridge instead of the Rust one.
        bridge = ['bridge:=python'] if '--python-bridge' in args else []
        session(['spine:=true', 'lidar:=false'] + bridge,
                [lambda probe: check_drive(probe, spine=True), armed_first(check_nav_goal),
                 check_wheels_vs_imu,
                 lambda probe: check_command_lost(probe, True),
                 lambda probe: check_estop(probe, True),
                 lambda probe: check_estop(probe, True, hardware=True),
                 check_not_armed],
                logs[-1], 'spine', spine=True)

    print('\n| Check | Result | Detail |\n|---|---|---|')
    for name, ok, detail in RESULTS:
        print(f'| {name} | {"ℹ️" if ok is None else "✅" if ok else "❌"} | {detail} |')
    print_stops()
    print(f'\nThe launch log: {", ".join(logs)}')
    return 0 if all(ok is not False for _, ok, _ in RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())

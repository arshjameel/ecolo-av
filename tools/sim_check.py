#!/usr/bin/env python3
"""
Sim check: start the road world headless, then test the car's whole graph in it.

Ravensim, our own simulator, and our own stack. Gazebo went on 16 Nov (its last scores:
docs/history/gazebo-as-it-was.md); Nav2 on 17 Nov (TODO 184; its last scores:
docs/league-results-2026-11-16.md).

    pixi run sim-check                     (about 2 minutes, on this computer only)
    pixi run -e ml sim-check --perception  (adds the detector and the stop sign: 2 more minutes)
    pixi run sim-check --obstacle          (adds a box for the radar stand-in: 2 more minutes;
                                            the planner blind to it, plan_around:=false, so
                                            the stop zone must stop the car)
    pixi run sim-check --plan-around       (a box in the lane the planner sees: the car must
                                            drive round it without stopping, in a second
                                            run: 3 more minutes)
    pixi run sim-check --lidar-obstacle    (the lidar alone, radar off, stops for a post: a
                                            second run, 3 more minutes)
    pixi run sim-check --rear-obstacle     (backing up, the lidar stops the car for a post
                                            behind it: a second run, 3 more minutes)
    pixi run sim-check --ghost             (a pretend person only navigation sees, aav_ghost: a
                                            second run, 3 more minutes)
    pixi run sim-check --faults            (sensor failures on purpose, each while the car drives:
                                            the car must stop, and say why: 5 more minutes)
    pixi run sim-check --faults --no-watchdog   (the same without the sensor watchdog: a report
                                            of what the stack does on its own, no pass or fail)
    Every run checks the launch's default stack, ours (stack:=ours: our planner, mppi_lag,
    our costmaps and navigator, the behaviour planner), unless a flag below says otherwise.
    pixi run sim-check --stack best        (every run with a named stack from stacks.yaml,
                                            stack:=, docs/league.md: ours, best, car; a
                                            flag below still wins over it)
    pixi run sim-check --tracker lqr       (every run with that path follower, tracker:=;
                                            also --planner, --costmap and --navigator, each
                                            with only ours left since Nav2 went)
    pixi run sim-check --legs-only --lidar-obstacle --plan-around
                                           (only the extra runs named, not the road
                                            world's first run: the costmap's scoring)
    pixi run sim-check --no-lidar         (without the simulated roof lidar: lidar:=false)
    pixi run sim-check --no-cameras       (no cameras drawn, cameras:=false: Ravensim
                                           on a computer with no GPU; the radar checked alone)
    pixi run sim-check --voxels            (every run with the lidar takes voxels:=true: we
                                            reads the lidar as cubes, aav_voxels, instead of
                                            the flat scan; with --lidar-obstacle, the trial's
                                            stop test. docs/designs/2026-10-30_voxels.md)
    pixi run sim-check --kiss-ekf          (every run with the lidar takes kiss_ekf:=true:
                                            ekf_local takes KISS-ICP's twist, a trial; and
                                            checks KISS-ICP's odometry came and ekf_local
                                            kept up. Needs pixi install --locked -e kiss)
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
                                            speed_loop.mode pi or open, and sim_esp32,
                                            the 2025-26 C++ firmware standing in;
                                            the Rust bridge, --python-legacy the Python)
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
    pixi run sim-check --stop-point        (where the car really stops against where ego
                                            said, /ego/stop_point: scripted drives at 1,
                                            1.5 and 2 m/s, then the e-stop (a full brake,
                                            the spine in the loop); truth and localization.
                                            The empty world, one more run;
                                            --stop-point-only skips the road world's run.
                                            docs/stopping.md)
    pixi run sim-check --loaded-stop       (a stop from 2 m/s, empty and with 150 kg
                                            aboard, the adaptive brake off and on: each
                                            against the empty car model's stop, within
                                            0.25 m with the adaptive brake. The empty
                                            world, one more run; --loaded-stop-only skips
                                            the road world's run. docs/stopping.md)
    pixi run sim-check --things           (the things, things:=true, aav_things_rs: a box
                                            placed 12 m ahead must be listed on
                                            /world/things within 1 m, standing; taken away,
                                            still listed 1.5 s on, gone after 5 s. One more
                                            run; --things-only skips the road world's run)
    pixi run sim-check --behaviour        (the behaviour trial, behaviour:=true: a wall
                                            across the road (nothing to go round) and a
                                            goal, each
                                            slowed by the speed cap, and with -e ml
                                            --perception the stop sign too; one more run.
                                            --behaviour-only skips the road world's run)
    pixi run sim-check --behaviour-things (the behaviour trial reading the things,
                                            behaviour:=true things:=true: a parked box
                                            the planner sees; the car must go round it,
                                            never APPROACH(obstacle). One more run;
                                            --behaviour-things-only skips the road world's)
    pixi run sim-check --moving           (TODO 170, Ravensim: a person crosses the
                                            lane ahead at 1 m/s while the car drives on;
                                            the behaviour must slow for it, the car keep
                                            clear and reach its goal. behaviour:=true
                                            things:=true; --moving-only skips the road
                                            world's run)
    pixi run sim-check --movers           (17 Nov, Ravensim: an oncoming car, a person
                                            walking along the roadside, cross traffic; the
                                            behaviour must hold the car for each
                                            (WAIT(moving)), the car keep clear and reach
                                            its goal after. --movers-only skips the road
                                            world's run)
    pixi run sim-check --with name:=value  (any sim.launch.py argument on every session,
                                            repeatable: an A/B of one setting)
    pixi run -e ml sim-check --thing-names (TODO 192: the detector names the lidar's
                                            things; the thing at the stop sign must be
                                            named stop sign, nothing else so;
                                            --thing-names-only skips the road world's run)
    pixi run sim-check --low-box          (TODO 188: a 0.5 m box the roof lidar loses near
                                            the bumper, the radar off; the car must go round
                                            it clear. Two runs: lidar_memory:=0, then 3 s;
                                            --low-box-only skips the road world's run)
    pixi run sim-check --actuator-faults  (TODO 159, the empty world: the e-stop from 1 m/s
                                            healthy, with a weak brake, a dead one and a
                                            stuck throttle: metres to stand; the throttle
                                            stuck while speeding up: the watchdog's
                                            OVERSPEED; the steering stuck in a turn: does
                                            anything stop the car.
                                            Evidence for docs/safety-case.md; one more run,
                                            --actuator-faults-only skips the road world's)
    pixi run sim-check --kerb             (TODO 119: a 0.3 m kerb across the road, the
                                            planner blind to it; the car must stop before
                                            it and the local costmap mark it (the stop
                                            zone needn't: it can't see that low and near);
                                            says from how far the kerbs' step finder saw
                                            it. Two runs: the lidar alone, then with the
                                            radar; --kerb-only skips the road world's run;
                                            the step finder is ours' (--with kerbs:=false
                                            for the flat scan alone))
    pixi run sim-check --three-point      (reverse, TODO 174: a 6.5 m street of walls, a
                                            goal behind the car facing back; the car must
                                            turn in it, backing at least once, without
                                            touching a wall. One more run, with the lidar;
                                            --three-point-only skips the road world's;
                                            AAV_THREE_POINT_SAVE=<file> keeps the run's
                                            track, plans and commands as JSON)
    pixi run sim-check --blocked-street   (reverse, TODO 174: a 3.5 m street of walls, too
                                            narrow to turn in, closed 8 m ahead; the goal
                                            25 m behind, facing back. The car must back out
                                            of it, changing direction at least once,
                                            without touching a wall. One more run, with the
                                            lidar; --blocked-street-only skips the road
                                            world's)

bench_check.py tests the car's software against pretend sensors that draw a flat world;
this tests it against Ravensim's cameras and radar stand-in, which see the road world
(aav_sim_models worlds/road.sdf: the bench road and its stop sign, in 3D).

1. Starts sim.launch.py world:=road rviz:=false on a private ROS domain,
   on this computer only.
2. Start-up: the truth, the global EKF, the camera and the radar stand-in all run,
   and navigation is up (unless nav:=false).
3. Checks the running graph against docs/interfaces.yaml (check_graph.py --profile road).
4. The sensors: the camera's picture and lens are the bench camera's, in camera_left_link;
   the radar stand-in's cloud has the TI driver's fields, in radar_link, and nothing in it
   while nothing is within 8.4 m. Unless --no-lidar, the simulated roof lidar: 1800 x 16 rays
   in lidar_link, 10 Hz, some of them hitting the ground and the sign; and lidar_scan's 2D
   scan of it for the costmap (/sensors/lidar/scan: 720 ranges in base_link, 10 Hz, the sign
   in it).
5. Drives a scripted profile, and compares where the car really is (/vehicle/sim/truth)
   with where the car's own software thinks it is (/localization/odom/global). After all
   the drives of this run, `wheels vs IMU and GPS`: the watchdog's wheel cross check never
   said DISAGREE, and the worst gaps it saw (its /diagnostics).
6. With --perception: the car drives to 40 m east, past the stop sign at 25 m. The car must
   stop with its front bumper before the sign, wait, and go on to the goal. The camera's
   picture with the detector's boxes, as the lock went on, is saved to /tmp/aav-sim-stop-sign.png.
7. With --obstacle: the bench's 1 m box appears in the lane 15 m ahead (Ravensim's
   spawn_entity), and the car drives 30 m. the stop zone must stop the car before
   the box; once the box is taken away, the car must go on to the goal.
8. With --lidar-obstacle: the road world again, with the radar off (radar:=false), so only the roof
   lidar guards the car. A person-sized post (0.5 x 0.5 x 1.8 m) in the lane 15 m ahead: the
   stop zone must stop the car before it from the lidar's scan, and the post must show in
   the local costmap; once the post is gone, the car must go on to the goal.
   With --rear-obstacle, the same backing up: the goal 15 m behind the car, the post 6 m
   behind the rear bumper, and the stop zone behind the car must stop it.
9. With --ghost: the road world again, with ghost:=true (aav_ghost): nothing in the world, only a
   pretend person 15 m past the bumper in navigation's view (its own costmap layer and
   stop zone source), placed through ghost_obstacle's parameters as
   `pixi run ghost ahead 15` does. the car drives 30 m: the stop zone must stop the car
   0-2 m before the ghost (as the global EKF has the car), and the local costmap must mark
   it; once the ghost is switched off, the car must go on to the goal, through where it
   stood.
10. With --faults: the road world again (faults:=true), and for each fault in FAULTS: the
   car drives, the fault goes on (fault.<sensor>, on its node: faults.NODES), and the car
   must stop within REACT seconds, stopped by the sensor watchdog's lock or the stop zone; then the
   fault goes off, and the lock must open again. Two faults are of the watchdog itself:
   `watchdog hang` stops its process (SIGSTOP, then SIGCONT to mend it) and `watchdog
   kill` kills it (the last fault: nothing mends it). Its silence must hold the navigator through
   twist_mux's lock timeout. `odom freeze` goes on at FROZEN_AT m/s, while the car still
   speeds up, and counts from when the frozen wheels go wrong by more than the cross
   check's tolerances (the car's speed or turn changes: the follower speeds up, slows or steers),
   as they're right till then, and has FROZEN_REACT s: a speed that stays wrong needs the
   GPS's 10 s window.
   `hud stop`: the watchdog also wants the HUD's heartbeat here (watch_hud:=true); with no
   screen, the check beats for it (/hud/heartbeat, 10 Hz, a rising count) and stops, as a
   frozen HUD does. Latched: beating again, it must still hold the navigator (hud LOST) 5 s later,
   until the check clears it (/safety/sensor_watchdog/clear), as a person would.
   `lidar stop` (TODO 246): sim_lidar_relay stops the roof lidar; the watchdog's lidar input
   goes STALE after 1 s and holds the navigator, as on the car (car-test's step 5e).
   With --no-watchdog it only reports, without the watchdog's own two.
11. With --spine: the road world again with the spine in the loop (spine:=true: ego -> spine_bridge
   -> CAN frames -> spine_sim, the spine's own firmware logic -> the actuator model). The
   normal checks first (the graph against the spine profile, the scripted drive, the EKF
   against the truth, a goal), then the spine's own stops, each while driving:
   `command stream lost` (ego's process stopped, SIGSTOP: the spine's watchdog must brake
   fully within SPINE_BRAKE s and latch, until armed again), `software e-stop`
   (/safety/estop), `hardware e-stop` (the bench's /vehicle/bench/hw_estop: still latched
   after it is let go, until armed again) and `not armed` (disarmed, a drive must not
   move the car). The first run also measures `command stream lost` and `software
   e-stop` on the direct path, and a last table compares the two.
12. With --drive-to-point: the empty world, twice: sim_speed 1 and 3. The point driver
   (aav_drive_to_point)
   drives to POINT_TARGETS (10, 30 and 60 m, ahead, to the side and behind), each from where
   the last stopped. Each must stop within 2 m of B by the truth, no more than
   POINT_PAST past it by its own EKF, and slow down at about half max_brake (the hardest
   0.5 s of its final stop, by the truth; from 1 m/s, only never harder: POINT_DECEL_FROM).
   At 1 m/s also: a lat/lon B (through /toLL and /fromLL), a 250 m B refused, and a drive
   stopped by a cancel (Ctrl-C), the e-stop and a GPS fault (the driver's own timeout and
   the sensor watchdog's lock). Each drive's truth goes to /tmp/aav-sim-points/ as an
   measure atob CSV (`pixi run measure atob /tmp/aav-sim-points/1-*.csv --speed 1`).

14. With --things: the road world again with things:=true (aav_things_rs), the car standing.
   The bench's box 12 m past the bumper must be on /world/things within 1 m of where it
   is (relative to the car: the truth for the box, the local EKF for the thing), at under
   0.3 m/s. Then the box goes: the same id must still be listed 1.5 s later (the memory,
   coasting) and gone 5 s later (keep, 3 s, outside the blind zone).

13. With --speed-steps: the empty world, nothing else driving. A scripted drive
   (ramped at ego's 1 m/s^2) holds each of SPEED_STEPS for 20 s: reverse at 0.5, then 1.0
   and 2.0 m/s. Each must end within 5 % of its target; its overshoot and the time to stay
   within 5 % are reported (the truth). With --speed-loop smooth, the overshoot must
   also stay under 3 % (the profile's promise, TODO 27). On a busy computer ego's commands
   can reach sim_adapter late (over its 0.2 s cmd_timeout), and it brakes: a step with
   such a brake below its target is driven again, up to 3 times, and only reported (no
   pass or fail) if every try was spoiled. The samples go to /tmp/aav-sim-speed-steps.csv.

Every run is on Ravensim (aav_simulator, docs/own-simulator.md), with its cameras
(aav_sim_cameras; --no-cameras: none). The obstacles go in through its
/vehicle/sim/spawn_entity and delete_entity services. Under tools/software_gl.sh (no GPU)
its clock waits for each picture (camera_lockstep:=auto): slower than real time, every
camera at its full rate. --simulator ravensim is the only choice (own: its old name).

Times are Ravensim's, simulated; only the limits on waiting are wall-clock. Prints a table and
exits 1 if anything failed. The launch logs go to /tmp/aav-sim-check.log (each extra run
beside it): AAV_SIM_LOG=/tmp/mine.log moves them. --lane (lane keeping) went with the lane
follower on 19 Nov (TODO 264; git tag save/lane-followers). AAV_SIM_DOMAIN picks the
ROS domain (otherwise one nobody on this computer uses now).
"""

import bisect
import collections
import importlib.util
import json
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
# A private ROS domain, on this computer only, set before rclpy
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

from aav_description import vehicle  # noqa: E402
from aav_interfaces.action import DriveToPoint, ScriptedDrive  # noqa: E402
from aav_interfaces.msg import ActuatorState, SpineOutputs, VehicleStatus  # noqa: E402
from aav_lane_geometry.msg import LaneGeometry  # noqa: E402
from aav_sim_models.faults import ACTUATOR_NODE, NODES as FAULT_NODES  # noqa: E402
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # noqa: E402
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
import stop_points  # noqa: E402
import street_check  # noqa: E402
from street_check import WALL_THICK  # noqa: E402
import tf2_ros  # noqa: E402
from vision_msgs.msg import Detection2DArray  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402
import yaml  # noqa: E402

RESULTS = []

# The road world is the bench world (test_worlds.py checks it): its road, sign and camera.
with open(os.path.join(ROOT, 'ros', 'sim', 'aav_bench', 'config', 'bench.yaml')) as f:
    BENCH = yaml.safe_load(f)['mock_camera']['ros__parameters']
ROAD_Y = BENCH['road_y']                # m: the lane's centre line
SIGN_X = BENCH['stop_signs'][0]         # m east: the stop sign
SIGN_Y = BENCH['stop_signs'][1]         # m north: right of the road
MODELS = os.path.join(ROOT, 'ros', 'sim', 'aav_sim_models', 'models')
OBSTACLE = os.path.join(MODELS, 'obstacle', 'model.sdf')   # the bench's 1 m box
POST = os.path.join(MODELS, 'post', 'model.sdf')           # 0.5 x 0.5 x 1.8 m: a person
LOW_BOX = os.path.join(MODELS, 'low_box', 'model.sdf')     # 0.5 m: the lidar loses it close
WALL = os.path.join(MODELS, 'wall', 'model.sdf')           # 10 x 0.3 x 2 m: a street's side
KERB = os.path.join(MODELS, 'kerb', 'model.sdf')           # 10 x 0.3 x 0.3 m: a foot-tall kerb
# What the radar adds to the road world's graph: absent with radar:=false.
RADAR_TOPICS = ('/sensors/radar/points', '/vehicle/sim/radar/points',
                '/perception/radar/clusters', '/perception/radar/cluster_boxes')
# What only runs with navigation (navigation.launch.py): the map, the costmaps, and
# stop_hold beside the stop zone. A nav:=false run checks the contract without them.
NAV_TOPICS = ('/map', '/safety/stop_hold/lock', '/drivers/nav/costmap/global',
              '/drivers/nav/costmap/local')
RADAR_FIELDS = ['x', 'y', 'z', 'intensity', 'velocity']   # the TI driver's
LIDAR_RAYS = (1800, 16)     # the simulated roof lidar: steps around, beams (sensors xacro)

# Sensor failures on purpose (--faults): each while the car drives, in this order (the NaNs
# last: a NaN that reaches an EKF could spoil the runs after it). The watchdog's limits
# (aav_safety config/sensor_watchdog.yaml) and the stop zone's source_timeout set how
# soon each must stop the car: REACT gives each that, plus ego's stop.
# The watchdog's own two: hang (mended by SIGCONT) early; kill last, as nothing mends it.
FAULTS = [('gps', 'stop'), ('watchdog', 'hang'), ('hud', 'stop'), ('imu', 'stop'),
          ('odom', 'stop'),
          ('odom', 'freeze'), ('camera', 'freeze'), ('radar', 'stop'), ('lidar', 'stop'),
          ('imu', 'freeze'),
          ('gps', 'jump'), ('gps', 'nan'), ('odom', 'nan'), ('imu', 'nan'), ('watchdog', 'kill')]
REACT = 6.0             # s of sim time from the fault to standing still
STILL = 0.05            # m/s: standing still
DRIVING = 0.5           # m/s: the car drives, so a fault can go on
# `odom freeze` goes on sooner, while the car still speeds up. mppi_lag plans from the
# speed the wheels say, so frozen wheels keep the car only a little faster than them:
# frozen at 0.5 m/s, it drove on at 0.6, under any tolerance the GPS's noise allows; at
# 0.2, it drives on at about 0.4, which the GPS part sees (24 Oct, docs/simulation.md).
FROZEN_AT = 0.2         # m/s
FROZEN_WAIT = 30.0      # s of sim time for frozen wheels to go wrong (the follower slows, turns)
HELD = 10.0             # s of sim time the car must stay still with no watchdog at all
with open(os.path.join(ROOT, 'ros', 'command_mux', 'aav_command_mux', 'config',
                       'twist_mux.yaml')) as f:
    # s of silence from the watchdog before twist_mux's sensors lock holds the navigator itself
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
# last DRIVE_CMD (spine-core cmd_timeout_ms). On the direct path sim_adapter's own
# cmd_timeout (0.2 s) does both. TICKS allows for each one's tick and the frames' trip.
with open(os.path.join(ROOT, 'ros', 'can_bridge', 'aav_spine', 'config', 'spine.yaml')) as f:
    BRIDGE_TIMEOUT = yaml.safe_load(f)['spine_bridge']['ros__parameters']['cmd_timeout']
SPINE_WATCHDOG = 0.1    # s: spine-core's cmd_timeout_ms
DIRECT_TIMEOUT = 0.2    # s: sim_adapter's cmd_timeout
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
    print(f'{"INFO" if ok is None else "PASS" if ok else "FAIL"}  {name}: {detail}', flush=True)


def process_cpu():
    """Return {program: CPU seconds} for the running launch's costmap processes."""
    if 'pgid' not in LAUNCH:
        return {}
    out = subprocess.run(['ps', '-A', '-o', 'pgid=,time=,command='], capture_output=True,
                         text=True).stdout
    cpu = {}
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) < 3 or parts[0] != str(LAUNCH['pgid']):
            continue
        name = next((n for n in COSTMAP_PROGRAMS if n in parts[2]), None)
        if name:
            # [[h:]m:]s.ss
            secs = 0.0
            for field in parts[1].split(':'):
                secs = secs * 60.0 + float(field)
            cpu[name] = cpu.get(name, 0.0) + secs
    return cpu


# Which processes hold a costmap: ours, both in aav_costmap's costmap.
COSTMAP_PROGRAMS = ('aav_costmap/costmap',)


class CostmapWatch:
    """Watch the costmaps while a leg runs: our update times and the costmap's CPU."""

    def __init__(self, probe):
        self.probe, self.times = probe, []
        self.subs = [
            probe.create_subscription(DiagnosticStatus, '/drivers/nav/costmap/status',
                                      self.on_status, 10)]
        self.cpu, self.start = process_cpu(), time.monotonic()

    def on_status(self, msg):
        values = {kv.key: kv.value for kv in msg.values}
        if values.get('costmap') == 'local':
            self.times.append(float(values['update_ms']))

    def report(self):
        """One line: the costmap's CPU and its update time."""
        wall = time.monotonic() - self.start
        now = process_cpu()
        used = {n: 100.0 * (now[n] - self.cpu.get(n, 0.0)) / wall for n in now}
        cpu = ', '.join(f'{n.split("/")[-1]} {used[n]:.0f} %'
                        for n in COSTMAP_PROGRAMS if n in used)
        for sub in self.subs:
            self.probe.destroy_subscription(sub)
        line = f'CPU over {wall:.0f} s: {cpu or "none seen"}'
        if self.times:
            t = sorted(self.times)
            line += (f'; our local update {sum(t) / len(t):.2f} ms mean, '
                     f'{t[int(0.95 * (len(t) - 1))]:.2f} ms 95th')
        return line


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rate(stamps):
    """Return messages per second of simulated time, from their stamps (s)."""
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    return (len(stamps) - 1) / span if span > 0 else 0.0


def place_model(model, x, y, yaw, name='obstacle'):
    """Put a model in the world: Ravensim's spawn_entity."""
    return ros_service('/vehicle/sim/spawn_entity', 'simulation_interfaces/srv/SpawnEntity',
                       f'{{name: "{name}", uri: "{model}", initial_pose: {{pose: {{position: '
                       f'{{x: {x}, y: {y}, z: 0.0}}, orientation: {{z: {math.sin(yaw / 2.0)}, '
                       f'w: {math.cos(yaw / 2.0)}}}}}}}}}')


def remove_model(name='obstacle'):
    """Take a placed model away again."""
    return ros_service('/vehicle/sim/delete_entity', 'simulation_interfaces/srv/DeleteEntity',
                       f'{{entity: "{name}"}}')


SERVICE_SAID = ['']     # the last service call's answer, for a failure's message


def ros_service(service, kind, request):
    """Call a ROS service by `ros2 service call`; True if its Result says RESULT_OK (1)."""
    try:
        done = subprocess.run(['ros2', 'service', 'call', service, kind, request],
                              capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        SERVICE_SAID[0] = f'{service}: no answer in 60 s'
        return False
    SERVICE_SAID[0] = ' '.join((done.stdout + done.stderr).split())[-300:]
    return done.returncode == 0 and re.search(r'result=1\b', done.stdout) is not None


class Probe(Node):

    def __init__(self):
        # On the simulated clock, like every node in the sim.
        super().__init__('sim_check', parameter_overrides=[Parameter('use_sim_time',
                                                                     value=True)])
        self.truth = self.glob = self.image = self.info = self.cloud = None
        # Every truth sample, lane message and sensor stamp, from the start: lane keeping
        # starts as soon as the camera sees the lane, before its check begins.
        self.track, self.lanes, self.image_stamps, self.cloud_stamps = [], [], [], []
        self.create_subscription(Odometry, '/vehicle/sim/truth', self.on_truth, 10)
        self.glob_track = []     # (t, x, y, yaw, v) of the global EKF, for --stop-point
        self.create_subscription(Odometry, '/localization/odom/global', self.on_glob, 10)
        # (stamp, v) of the one command ego obeys, for --stop-point
        self.commands = []
        self.create_subscription(TwistStamped, '/control/cmd_vel',
                                 lambda m: self.commands.append(
                                     (seconds(m.header.stamp), m.twist.linear.x)), 10)
        # (stamp, speed) of what the backend got from ego, for --loaded-stop
        self.actuator_cmds = []
        self.create_subscription(AckermannDriveStamped, '/control/sim/actuator_cmd',
                                 lambda m: self.actuator_cmds.append(
                                     (seconds(m.header.stamp), m.drive.speed)), 10)
        # (stamp, x, y) of ego's stop point, base_link at its stamp, and /tf for it
        self.stop_points = []
        self.create_subscription(PoseStamped, '/ego/stop_point',
                                 lambda m: self.stop_points.append(
                                     (seconds(m.header.stamp), m.pose.position.x,
                                      m.pose.position.y)), 10)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        # (t, state, reason, cap, left) from the behaviour trial (--behaviour)
        self.behaviour = []
        self.create_subscription(DiagnosticStatus, '/drivers/nav/behaviour/state',
                                 self.on_behaviour, 10)
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
        # Our navigator serves Nav2's action names (Nav2's own went, 17 Nov).
        self.nav = ActionClient(self, NavigateToPose,
                                vehicle.navigator_action('navigate_to_pose'))
        self.stop, self.stop_picture, self.picture, self.stop_sign = False, None, None, None
        self.create_subscription(Bool, '/perception/stop_sign', self.on_stop, 10)
        self.create_subscription(Image, '/perception/detections/image',
                                 lambda m: setattr(self, 'picture', m), 2)
        # The stop signs the detector saw: (where the car was, x m; score; box height, px).
        self.signs = []
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_boxes, 10)
        self.lock, self.state = None, ''     # the sensor watchdog's
        self.lock_heard = None               # when its lock last arrived (sim time)
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
        self.steer_stuck_at = None  # when the watchdog first said steering STUCK (TODO 207)
        self.over_at = None         # ... first said speed OVER (a warning), then OVERSPEED
        self.overspeed_at = None
        self.gaps = {}              # its worst gaps, from /diagnostics
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        # The car's status (the spine's, with spine:=true), and the actuators' demands as
        # (stamp, throttle, brake): spine_sim's outputs with the spine in the loop, or
        # sim_adapter's own speed loop's commands on the direct path. For the stops.
        self.status, self.demands, self.direct = None, [], {}
        self.latches = []    # sim time each time the spine stopped being armed
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_status, 10)
        self.create_subscription(SpineOutputs, '/vehicle/sim/spine_outputs',
                                 lambda m: self.on_demand(seconds(m.header.stamp), m.throttle,
                                                          m.brake), 10)
        self.create_subscription(ActuatorState, '/vehicle/throttle',
                                 lambda m: self.on_direct('throttle', m), 10)
        self.create_subscription(ActuatorState, '/vehicle/brake',
                                 lambda m: self.on_direct('brake', m), 10)
        self.estop = self.create_publisher(Bool, '/safety/estop', 10)
        self.hw_estop = self.create_publisher(Bool, '/vehicle/bench/hw_estop', 10)
        self.arm_client = self.create_client(SetBool, '/spine/arm')
        self.monitor = []    # (action, polygon) from the stop zone, as it changes
        self.create_subscription(CollisionMonitorState, '/safety/stop_zone/state',
                                 lambda m: self.monitor.append((m.action_type, m.polygon_name)),
                                 10)
        self.output_mux = ''   # the output mux's state: OPEN, WAIT or STOP (aav_io_mux)
        self.create_subscription(String, '/safety/output_mux/state',
                                 lambda m: setattr(self, 'output_mux', m.data), 10)
        # (stamp, vx): KISS-ICP's odometry and ekf_local's, for --kiss-ekf.
        self.kiss, self.local = [], []
        self.create_subscription(Odometry, '/localization/kiss_icp/odom',
                                 lambda m: self.kiss.append((seconds(m.header.stamp),
                                                             m.twist.twist.linear.x)), 10)
        self.create_subscription(Odometry, '/localization/odom/local',
                                 lambda m: self.local.append((seconds(m.header.stamp),
                                                              m.twist.twist.linear.x)), 10)
        self.voxels = []     # the voxels trial's status values, every cloud (--voxels)
        self.create_subscription(DiagnosticStatus, '/perception/voxels/status',
                                 lambda m: self.voxels.append({v.key: v.value for v in m.values}),
                                 10)

    def on_glob(self, msg):
        self.glob = msg
        self.glob_track.append((seconds(msg.header.stamp), msg.pose.pose.position.x,
                                msg.pose.pose.position.y, yaw_of(msg), msg.twist.twist.linear.x))

    def on_behaviour(self, msg):
        v = {kv.key: kv.value for kv in msg.values}
        self.behaviour.append((self.now(), v.get('state', ''), v.get('reason', ''),
                               float(v.get('cap_mps', 'nan')), float(v.get('left_m', 'nan'))))

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
        """sim_adapter's throttle and brake commands (NaN from spine_bridge: skipped)."""
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
        if self.steer_stuck_at is None and 'steering STUCK' in msg.data:
            self.steer_stuck_at = self.now()
        if self.over_at is None and 'speed OVER' in msg.data:
            self.over_at = self.now()
        if self.overspeed_at is None and 'speed OVERSPEED' in msg.data:
            self.overspeed_at = self.now()

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
        """Return True while twist_mux's sensors lock holds the navigator: true, or silent."""
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
        """Send the navigator a goal in the map frame; return its result's future, or None."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return None
        # The action server can exist before the navigator takes goals (its planner and
        # follower not yet there): on a busy computer that can be seconds after.
        end = time.monotonic() + 30.0
        while True:
            sent = self.nav.send_goal_async(goal)
            self.spin_for(10.0, lambda: sent.done())
            if sent.done() and sent.result().accepted:
                break
            if time.monotonic() > end:
                return None
            self.spin_for(2.0)
        self.goal = sent.result()     # its handle, to cancel it
        return self.goal.get_result_async()


def check_contract(probe, env, tag, ignore=(), profile='road'):
    """
    Check the running graph against docs/interfaces.yaml (check_graph.py --profile road).

    profile: spine instead, with the spine in the loop.

    ignore: topics this run leaves out on purpose, such as our /map with nav:=false.
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


def check_radar(probe):
    """Check the radar stand-in alone (--no-cameras): the TI's fields, 10 Hz, nothing near."""
    probe.spin_for(5.0)
    cloud = probe.cloud
    fields = [f.name for f in cloud.fields]
    radar_hz = rate(probe.cloud_stamps)
    ok = (cloud.header.frame_id == 'radar_link' and fields == RADAR_FIELDS
          and cloud.width * cloud.height == 0 and abs(radar_hz - 10.0) < 1.5)
    result('sensors (no cameras)', ok,
           f'radar stand-in in {cloud.header.frame_id} at {radar_hz:.1f} Hz, fields '
           f'{",".join(fields)}, {cloud.width * cloud.height} points within 8.4 m')


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
    """Check lidar_scan's 2D scan for the costmap: 720 ranges in base_link, 10 Hz, the sign."""
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
    """Drive to 40 m east, past the stop sign at 25 m: it must stop before it."""
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']   # base_link to the front bumper
    done = probe.go_to(40.0, ROAD_Y, 0.0)
    if done is None:
        result('stop sign', False, 'no navigator, or it refused the goal')
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
                            f'goal {"reached" if reached else "not reached"} at {x:.1f} m')
    if probe.stop_picture is not None:
        try:
            from cv_bridge import CvBridge
            import cv2
            cv2.imwrite('/tmp/aav-sim-stop-sign.png',
                        CvBridge().imgmsg_to_cv2(probe.stop_picture, 'bgr8'))
            print('The camera as the lock went on: /tmp/aav-sim-stop-sign.png', flush=True)
        except Exception as e:  # noqa: B902 - the picture is a bonus
            print(f'could not save the picture: {e}', flush=True)


def check_kiss_ekf(probe):
    """
    Check the KISS-ICP trial (--kiss-ekf): its odometry, and ekf_local reading it.

    KISS-ICP's odometry came at the lidar's 10 Hz, and ekf_local, which reads its twist,
    kept its 30 Hz and the truth's speed.
    """
    kiss, local = list(probe.kiss), list(probe.local)
    if len(kiss) < 10:
        result('kiss_ekf', False, f'{len(kiss)} KISS-ICP odometries: is the kiss env installed?')
        return
    kiss_rate = rate([t for t, _ in kiss])
    local_rate = rate([t for t, _ in local])
    result('kiss_ekf', kiss_rate > 5.0 and local_rate > 20.0,
           f'KISS-ICP {kiss_rate:.1f} Hz, ekf_local {local_rate:.1f} Hz (simulated time)')
    # ekf_local's speed against the truth's, at each of its stamps.
    truth = [(t, v) for t, _, _, _, v in probe.track]
    times = [t for t, _ in truth]
    gaps = []
    for t, v in local:
        i = bisect.bisect_left(times, t)
        if 0 < i < len(times):
            gaps.append(abs(v - truth[i][1]))
    if gaps:
        gaps.sort()
        result('kiss_ekf speed', None,
               f"ekf_local's speed vs the truth: median {gaps[len(gaps) // 2]:.3f} m/s, "
               f'worst {gaps[-1]:.3f} m/s over {len(gaps)} samples')


def check_voxels(probe):
    """
    Check the voxels trial (--voxels): its status every cloud, and its cost against the scan.

    Over the run so far: the clouds cut, ms a cloud (wall clock, read to publish), cubes a
    cloud, the road fitted. Then one raw cloud, cut both ways here, 20 times each: the flat
    scan (aav_lidar scan.flatten) and the cubes (aav_voxels cubes.cubes), on the same points.
    """
    from aav_lidar.scan import body_box, flatten, ScanShape, to_base
    from aav_voxels.cubes import cubes, CubeShape
    probe.spin_for(3.0)
    seen = list(probe.voxels)
    if not seen:
        result('voxels', False, 'nothing on /perception/voxels/status')
        return
    ms = numpy.array([float(v['ms']) for v in seen])
    count = numpy.array([int(v['cubes']) for v in seen])
    fitted = sum(v['fitted'] == 'true' for v in seen)
    result('voxels', True,
           f'{len(seen)} clouds: {ms.mean():.1f} ms mean, {numpy.percentile(ms, 95):.1f} ms '
           f'95th, {ms.max():.1f} ms worst; {count.mean():.0f} cubes a cloud (most '
           f'{count.max()}); road fitted in {fitted} of {len(seen)}; '
           f'{seen[-1]["clouds_dropped"]} clouds dropped')
    clouds = []
    sub = probe.create_subscription(PointCloud2, '/sensors/lidar/points', clouds.append, 2)
    probe.spin_for(3.0)
    probe.destroy_subscription(sub)
    if not clouds:
        return
    car = vehicle.load()
    mount = car['sensors']['lidar_link']
    body = body_box(vehicle.footprint_points(car['vehicle']))
    low, high = vehicle.obstacle_heights(car['vehicle'])

    def timed(work):
        start = time.perf_counter()
        for _ in range(20):
            work()
        return (time.perf_counter() - start) / 20 * 1000.0

    xyz = point_cloud2.read_points_numpy(clouds[-1], field_names=('x', 'y', 'z'),
                                         skip_nans=False)
    points = to_base(numpy.asarray(xyz, dtype=float).reshape(-1, 3), mount['xyz'], mount['rpy'])
    scan_ms = timed(lambda: flatten(points, ScanShape(min_height=low, max_height=high,
                                                      body=body)))
    shape = CubeShape(min_height=low, max_height=high, body=body)
    cube_ms = timed(lambda: cubes(points, shape))
    read_ms = timed(lambda: point_cloud2.read_points_numpy(
        clouds[-1], field_names=('x', 'y', 'z'), skip_nans=False))
    result('voxels vs scan', None,
           f'one cloud ({len(points)} rays), here: reading it {read_ms:.2f} ms, then the flat '
           f'scan {scan_ms:.2f} ms, the cubes {cube_ms:.2f} ms')


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
                   costmap=False, backwards=False, across=False, stop_zone_needed=True,
                   steps=False):
    """
    Put a model `ahead` m in the lane: the stop zone must stop the car before it.

    model is the bench's box (1 m deep) unless given; the lidar's check uses the post
    (0.5 m). With costmap, the local costmap must also mark it while the car stands there.
    backwards: the model and the goal are behind the car, which must back up to them, and
    the stop zone behind the rear bumper must stop it. across: the model turned a quarter
    turn (a wall across the road: nothing to go round). stop_zone_needed: False when only
    the costmap can see it (a kerb: the roof lidar loses it about 4 m out), so a stop by the
    controller passes too. steps: also say from how far the kerbs' step finder
    (/perception/voxels/steps, kerbs:=true) first saw it, from the bumper.
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
    if not place_model(model, bx, by, h + (math.pi / 2.0 if across else 0.0)):
        result(name, False, "could not place the box (the simulator's create service)")
        return
    newest = {}     # the local costmap, in odom: on the world's origin, as the car started
    if costmap:
        sub = probe.create_subscription(OccupancyGrid, LOCAL_COSTMAP[COSTMAP],
                                        lambda m: newest.update(grid=m), 2)
    seen_steps = {}

    def on_steps(msg):
        # In base_link: the model's face, ahead of the bumper, across the car's way.
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        xyz = numpy.asarray(xyz, dtype=float).reshape(-1, 3)
        near = xyz[(numpy.abs(xyz[:, 1]) < 3.0) & (way * xyz[:, 0] > front)]
        if len(near) >= 3 and 'first' not in seen_steps:
            seen_steps['first'] = float(numpy.min(way * near[:, 0])) - front
    step_sub = (probe.create_subscription(PointCloud2, '/perception/voxels/steps', on_steps,
                                          qos_profile_sensor_data) if steps else None)
    watch = CostmapWatch(probe)
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h)    # facing as it started
    if done is None:
        result(name, False, 'no navigator, or it refused the goal')
        remove_model()
        watch.report()
        return
    seen, top, still, gap, peak = len(probe.monitor), 0.0, None, None, None
    stopped_by = None                       # who stopped the car first
    moving = 0.3 if backwards else 0.5      # m/s: it has got going (reversing: 0.5 at most)
    furthest, fired = 0.0, None             # m towards the model; when the stop zone fired

    def take_away():
        nonlocal gap, peak
        gap = ahead - furthest              # the closest the bumper came
        if 'grid' in newest:
            peak = costmap_peak(newest['grid'], bx, by, depth / 2.0 + 0.5)
        remove_model()

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
            if stopped_by is None:
                stopped_by = 'the stop zone' if fired is not None else 'the controller'
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
    if step_sub is not None:
        probe.destroy_subscription(step_sub)
    stop_zone = any(a == CollisionMonitorState.STOP and p == 'StopZone'
                    for a, p in probe.monitor[seen:])
    reached = done.done() and done.result().status == 4
    passed = along() > front + ahead + depth
    marked = not costmap or (peak is not None and peak >= 99)
    ok = (gap is not None and 0.0 < gap < 2.0 and (stop_zone or not stop_zone_needed)
          and reached and passed and marked)
    where = f'came within {gap:.2f} m of it' if gap is not None else 'never stopped'
    mark = ''
    if costmap:
        mark = (f'; the local costmap {peak} of 100 there' if peak is not None
                else '; no local costmap seen')
    if steps:
        mark += ('; the step finder first saw it '
                 f"{seen_steps['first']:.1f} m past the bumper" if 'first' in seen_steps
                 else '; the step finder never saw it')
    result(name, ok, f'{where} (top speed {top:.2f} m/s), stop zone fired: {stop_zone}{mark}; '
                     f'first stopped by {stopped_by or "nothing"}; '
                     f'taken away, goal {"reached" if reached else "not reached"} '
                     f'{along():.1f} m on')
    result(f'{name}: costmaps ({COSTMAP})', None, watch.report())
    if gap is None:
        remove_model()


def check_plan_around(probe, ahead=15.0, goal=30.0, depth=1.0, model=OBSTACLE,
                      name='plan around'):
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
    if not place_model(model, bx, by, h):
        result(name, False, "could not place the box (the simulator's create service)")
        return
    start = len(probe.track)
    seen = len(probe.monitor)
    watch = CostmapWatch(probe)
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h)
    if done is None:
        result(name, False, 'no navigator, or it refused the goal')
    else:
        probe.spin_for(240.0, done.done)
    remove_model()
    costs = watch.report()
    if done is None:
        return
    result(f'{name}: costmaps ({COSTMAP})', None, costs)
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
    result(name, ok,
           f'{"reached" if reached else "did not reach"} the goal {goal:.0f} m on, '
           f'{clear:.2f} m clear of the box at the closest, stood still {stood:.1f} s at the '
           f'longest, stop zone fired: {stop_zone}')


def place_street(x0, y0, h, half, alongs, name):
    """
    Put a street's walls in the world (street_check.walls): their names, or None.

    If one can't be placed, the ones placed go again and the check `name` fails.
    """
    names = []
    for wall, wx, wy in street_check.walls(x0, y0, h, half, alongs):
        if not place_model(WALL, wx, wy, h, name=wall):
            result(name, False, f'could not place {wall}: {SERVICE_SAID[0]}')
            for placed in names:
                remove_model(placed)
            return None
        names.append(wall)
    return names


def check_three_point_turn(probe, width=6.5, goal=5.0):
    """
    Turn round in a narrow street: walls `width` m apart, a goal `goal` m on, facing back.

    The car's tightest circle needs more than 6.5 m between the walls, so it must back at
    least once (a three-point turn: TODO 174). It must reach the goal, change direction at
    least twice, and keep every corner inside the walls.
    """
    body = vehicle.load()['vehicle']['body']
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)
    half = width / 2.0
    # Four 10 m walls a side, from 15 m behind to 25 m ahead: too long to drive round.
    names = place_street(x0, y0, h, half, (-10.0, 0.0, 10.0, 20.0), 'three-point turn')
    if names is None:
        return
    probe.spin_for(3.0)          # the lidar sees the walls before the planner is asked
    start = len(probe.track)
    # Every plan the car was given, to set its changes of direction against the plan's.
    plans = []
    sub = probe.create_subscription(
        Path, '/plan', lambda m: plans.append(
            (seconds(m.header.stamp), [(p.pose.position.x, p.pose.position.y)
                                       for p in m.poses])), 10)
    # The follower's own commands, before the mux and ego: (stamp, v, turn rate).
    asked = []
    asked_sub = probe.create_subscription(
        TwistStamped, '/drivers/nav/cmd_vel', lambda m: asked.append(
            (seconds(m.header.stamp), m.twist.linear.x, m.twist.angular.z)), 10)
    gx, gy, gyaw = x0 + goal * c, y0 + goal * s, math.atan2(-s, -c)
    done = probe.go_to(gx, gy, gyaw)
    if done is not None:
        probe.spin_for(300.0, done.done)
        if not done.done():
            probe.goal.cancel_goal_async()
    probe.destroy_subscription(sub)
    probe.destroy_subscription(asked_sub)
    for name in names:
        remove_model(name)
    if done is None:
        result('three-point turn', False, 'no navigator, or it refused the goal')
        return
    drive = probe.track[start:]
    reached = done.done() and done.result().status == 4
    turns = street_check.turns(drive)
    room = street_check.clearance(drive, x0, y0, h, half, body)
    x1, y1 = probe.xy(probe.truth)
    turned = abs(math.remainder(yaw_of(probe.truth) - gyaw, math.tau))
    ok = reached and turns >= 2 and room > 0.0
    planned = street_check.plan_cusps([path for _, path in plans])
    result('three-point turn', ok,
           f'{"reached" if reached else "did not reach"} the goal '
           f'({math.hypot(gx - x1, gy - y1):.2f} m, {math.degrees(turned):.0f} deg off), '
           f'{turns} changes of direction ({len(plans)} plans, their changes '
           f'{planned or "-"}), {room:.2f} m from a wall at the closest (street {width:g} m)')
    save = os.environ.get('AAV_THREE_POINT_SAVE')
    if save:
        # The run's raw data, to see where each change came from (TODO 174).
        with open(save, 'w') as f:
            json.dump({'track': drive, 'plans': plans, 'goal': (gx, gy, gyaw),
                       'street': (x0, y0, h, half), 'commands': probe.commands,
                       'actuator': probe.actuator_cmds, 'asked': asked,
                       'behaviour': probe.behaviour, 'monitor': probe.monitor}, f)


def check_blocked_street(probe, width=3.5, block=8.0, goal=25.0):
    """
    Leave a narrow street blocked ahead (TODO 174): walls `width` m apart, closed `block` m on.

    Too narrow to turn in, even in three points: the car must back out. The goal is
    `goal` m behind it, facing back, past the street's end (15 m behind). It must reach
    it, change direction at least once, and keep every corner inside the walls.
    """
    body = vehicle.load()['vehicle']['body']
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)
    half = width / 2.0
    names = place_street(x0, y0, h, half, (-10.0, 0.0, 10.0), 'blocked street')
    if names is None:
        return
    wall, bx, by, byaw = street_check.block(x0, y0, h, block)
    if not place_model(WALL, bx, by, byaw, name=wall):
        result('blocked street', False, f'could not place {wall}: {SERVICE_SAID[0]}')
        for name in names:
            remove_model(name)
        return
    names.append(wall)
    probe.spin_for(3.0)          # the lidar sees the walls before the planner is asked
    start = len(probe.track)
    plans = []
    sub = probe.create_subscription(
        Path, '/plan', lambda m: plans.append(
            (seconds(m.header.stamp), [(p.pose.position.x, p.pose.position.y)
                                       for p in m.poses])), 10)
    gx, gy, gyaw = x0 - goal * c, y0 - goal * s, math.atan2(-s, -c)
    done = probe.go_to(gx, gy, gyaw)
    if done is not None:
        probe.spin_for(300.0, done.done)
        if not done.done():
            probe.goal.cancel_goal_async()
    probe.destroy_subscription(sub)
    for name in names:
        remove_model(name)
    if done is None:
        result('blocked street', False, 'no navigator, or it refused the goal')
        return
    drive = probe.track[start:]
    reached = done.done() and done.result().status == 4
    turns = street_check.turns(drive)
    # Only the street's own stretch: past its end the car is free to turn round.
    inside = [p for p in drive
              if -15.0 <= (p[1] - x0) * c + (p[2] - y0) * s <= block]
    room = street_check.clearance(inside, x0, y0, h, half, body) if inside else math.inf
    x1, y1 = probe.xy(probe.truth)
    ok = reached and turns >= 1 and room > 0.0
    planned = street_check.plan_cusps([path for _, path in plans])
    result('blocked street', ok,
           f'{"reached" if reached else "did not reach"} the goal '
           f'({math.hypot(gx - x1, gy - y1):.2f} m off), {turns} changes of direction '
           f'({len(plans)} plans, their changes {planned or "-"}), {room:.2f} m from a wall '
           f'at the closest (street {width:g} m)')


def check_kerb(probe, sensors):
    """
    Lay a foot-tall kerb across the road (TODO 119), the planner blind to it: is it seen.

    The flat scan keeps only points 0.3 m or more above the road (GROUND_CLEARANCE), the
    kerb's own height; with kerbs:=true the voxels node's step finder sees it (TODO 119).
    The car must stop before the kerb as check_obstacle's box, and the local costmap must
    mark it; Ravensim stops a car that drives into a step over 0.1 m as at a wall.
    The stop zone needn't fire: it reaches 1 m past the bumper, and the roof lidar loses
    the kerb about 4 m out, so the costmap's mark (kept) is what stops the car. sensors:
    the words for which ran (the lidar, the radar).
    """
    check_obstacle(probe, model=KERB, depth=0.3, across=True, costmap=True,
                   name=f'kerb, 0.3 m ({sensors})', stop_zone_needed=False, steps=True)


def check_low_box(probe):
    """
    Go round a 0.5 m box the roof lidar loses within about 2.5 m of the bumper (TODO 188).

    The radar is off (as on the car today), so only the costmap's memory of the lidar
    (lidar_memory:=) keeps the box marked as the car passes it. The same rules as
    --plan-around: reach the goal past it, at least 0.3 m clear, the stop zone quiet.
    """
    check_plan_around(probe, ahead=10.0, goal=25.0, depth=0.5, model=LOW_BOX, name='low box')


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


def check_things(probe, ahead=12.0, depth=1.0):
    """
    Check the things (things:=true): a box ahead, listed, remembered, forgotten.

    The box must be listed where it is, standing; taken away, it is remembered a little
    (coasting), then forgotten.
    """
    from aav_interfaces.msg import Things
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']
    newest, odom = {}, {}
    subs = [probe.create_subscription(Things, '/world/things',
                                      lambda m: newest.update(things=m), 10),
            probe.create_subscription(Odometry, '/localization/odom/local',
                                      lambda m: odom.update(pose=m), 10)]
    try:
        x0, y0 = probe.xy(probe.truth)
        h = yaw_of(probe.truth)
        c, s = math.cos(h), math.sin(h)
        bx = x0 + (front + ahead + depth / 2.0) * c
        by = y0 + (front + ahead + depth / 2.0) * s
        if not place_model(OBSTACLE, bx, by, h):
            result('things', False, "could not place the box (the simulator's service)")
            return
        # Where the box is from the car: ahead and left, by the truth.
        want = ((bx - x0) * c + (by - y0) * s, -(bx - x0) * s + (by - y0) * c)

        def ahead_left(th):
            p = odom['pose'].pose.pose
            ox, oy, oh = p.position.x, p.position.y, yaw_of(odom['pose'])
            dx, dy = th.x - ox, th.y - oy
            return (dx * math.cos(oh) + dy * math.sin(oh),
                    -dx * math.sin(oh) + dy * math.cos(oh))

        def nearest():
            if 'things' not in newest or 'pose' not in odom:
                return None, math.inf
            best = min(newest['things'].things, default=None,
                       key=lambda th: math.dist(ahead_left(th), want))
            return best, (math.dist(ahead_left(best), want) if best else math.inf)

        found, miss, end = None, math.inf, probe.now() + 15.0
        while probe.now() < end:
            probe.spin_for(0.2)
            found, miss = nearest()
            if found is not None and miss < 1.0 and found.seen >= 10:
                break
        if found is None or miss >= 1.0:
            listed = len(newest['things'].things) if 'things' in newest else 'no'
            result('things', False, f'the box not listed within 1 m ({listed} things; '
                                    f'nearest {miss:.2f} m off)')
            remove_model()
            return
        speed = math.hypot(found.vx, found.vy)
        result('things: the box', speed < 0.3,
               f'id {found.id}, {miss:.2f} m from where it is, {speed:.2f} m/s, '
               f'{found.length:.2f} x {found.width:.2f} x {found.height:.2f} m, seen '
               f'{found.seen} times, {len(newest["things"].things)} things listed')
        ident = found.id
        remove_model()
        gone_at = probe.now()

        def listed():
            return [th for th in newest['things'].things if th.id == ident]

        while probe.now() < gone_at + 1.5:     # the simulator's seconds, as the tracker's
            probe.spin_for(0.1)
        kept = listed()
        coasting = bool(kept) and kept[0].coasting
        while probe.now() < gone_at + 5.0:
            probe.spin_for(0.2)
        forgotten = not listed()
        result('things: the memory', bool(kept) and coasting and forgotten,
               f'taken away: 1.5 s later {"still listed" if kept else "gone"}'
               f'{" (coasting)" if coasting else ""}; 5 s later '
               f'{"gone" if forgotten else "still listed"}')
    finally:
        for sub in subs:
            probe.destroy_subscription(sub)


def check_ghost(probe, ahead=15.0, goal=30.0):
    """
    Put a ghost (aav_ghost) `ahead` m past the bumper: not in the world, only in navigation's view.

    the stop zone must stop the car 0-2 m before it, and the local costmap must mark it; once the
    ghost is switched off, the car must go on to the goal, through where it stood. Either
    may stop it: the stop zone, or the controller's own collision check on
    the costmap (Regulated Pure Pursuit), which can stop it first. The gap is measured as
    navigation sees it: from the ghost's own scan, the bumper to its nearest point ahead.
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
        result('ghost', False, 'no navigator, or it refused the goal')
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
                        f'(else the controller did); {mark}; switched off: {gone}, goal '
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
    node = FAULT_NODES.get(sensor, ACTUATOR_NODE)     # an actuator's: sim_adapter
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
    Send the navigator 15 m on, along the road; wait until the car drives at `speed` m/s.

    East until x = 60 m, then west back to x = 0 (the map ends at 100 m): the navigator turns the
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
    """Each fault in FAULTS while the car drives: the car must stop, and something say why."""
    if not fault_client(probe, 'odom').wait_for_services(timeout_sec=20.0):
        result('faults', False, "no sim_adapter parameters: can't set a fault")
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
            result(name, False, 'the car would not drive before the fault')
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
        # speed against the GPS's; the turn rate as a mean over its window, so the follower's small
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
                                        ('the stop zone (invalid source)', invalid)) if on_)
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
            # Nothing mends it: the goal still stands, and the car must stay still.
            top = 0.0
            end = probe.now() + HELD
            while probe.now() < end:
                probe.spin_for(0.1)
                top = max(top, abs(probe.truth.twist.twist.linear.x))
            held = top < STILL and probe.silent()
            ok = ok and held
            detail += (f'; held {HELD:.0f} s more with no watchdog, the navigator still asking '
                       f'(top speed {top:.2f} m/s)')
            if not done.done():
                probe.goal.cancel_goal_async()
            result(name, ok, detail)
            continue
        # Mend it, and let the car's software settle before the next.
        set_fault(probe, sensor, 'ok')
        if sensor == 'hud':
            # Latched (Ben, 24 Oct): a HUD that beats again still holds the navigator, as hud LOST,
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
    """Drive 15 m on, along the car's heading: it must get there."""
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    gx, gy = x0 + 15.0 * math.cos(h), y0 + 15.0 * math.sin(h)
    done = probe.go_to(gx, gy, h)
    if done is None:
        result('navigation goal', False, 'no navigator, or it refused the goal')
        return
    probe.spin_for(180.0, done.done)
    reached = done.done() and done.result().status == 4
    if not done.done():
        probe.goal.cancel_goal_async()
    x1, y1 = probe.xy(probe.truth)
    result('navigation goal', reached, f'{"reached" if reached else "not reached"}: '
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
    Return (s to the brake, s to standing still) after t0 (sim time), None if never.

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
    """Spin seconds_ of sim time; return the top speed meanwhile (m/s)."""
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


def roll_out(probe, x0, y0, press, limit=30.0):
    """
    Press the e-stop until the car stands, at most `limit` s of the sim's time.

    Return (metres from (x0, y0) to where it stood or was at the end, s, standing).
    """
    t0 = probe.now()

    def still():
        press.publish(Bool(data=True))
        return probe.now() > t0 + 0.2 and abs(probe.speed()) < STILL

    standing = probe.spin_for(limit, still)
    x, y = probe.xy(probe.truth)
    return math.hypot(x - x0, y - y0), probe.now() - t0, standing


def actuator_stop(probe, fault):
    """
    One e-stop from about 1 m/s with an actuator fault (None: healthy): its result line.

    The fault goes on 0.5 s before the press, so a stuck throttle holds the cruise's.
    Mended afterwards, the e-stop let go and the drive ended.
    """
    name = f'actuator {" ".join(fault)}: the e-stop' if fault else 'actuator healthy: the e-stop'
    handle = get_going(probe, False)
    if handle is None:
        result(name, False, 'the car would not get going')
        return None
    if fault and not set_fault(probe, *fault):
        result(name, False, f'sim_adapter refused {" ".join(fault)}')
        finish(probe, handle)
        return None
    probe.spin_for(0.5)
    v0 = abs(probe.speed())
    x0, y0 = probe.xy(probe.truth)
    metres, took, standing = roll_out(probe, x0, y0, probe.estop)
    v_end = abs(probe.speed())
    if fault:
        set_fault(probe, fault[0], 'ok')
    probe.spin_for(10.0, lambda: abs(probe.speed()) < STILL)
    held(probe, 1.0, press=probe.estop)
    finish(probe, handle)
    return name, v0, metres, took, standing, v_end


TURNING = 0.06     # rad/s: the car is in the bend (check_actuator_faults)


def clear_watchdog(probe):
    """Clear the watchdog's latches, as a person would, and forget what it said."""
    clear = probe.create_client(Trigger, '/safety/sensor_watchdog/clear')
    if clear.wait_for_service(timeout_sec=5.0):
        cleared = clear.call_async(Trigger.Request())
        probe.spin_for(10.0, cleared.done)
    probe.spin_for(1.0)
    probe.over_at = probe.overspeed_at = None


def check_overspeed(probe, watch=20.0):
    """
    Stick the throttle while speeding up to 1 m/s: the watchdog must say OVERSPEED.

    TODO 207.

    Stuck at 0.4 m/s, the throttle holds more than cruising needs and the speed loop's
    brake only partly beats it. Passes if the watchdog latches OVERSPEED within `watch` s;
    then the fault is mended, the drive ended and the latch cleared.
    """
    name = 'actuator throttle stuck: speeding up'
    probe.spin_for(30.0, lambda: abs(probe.speed()) < STILL)
    clear_watchdog(probe)
    handle = probe.send_drive()
    if handle is None or not probe.spin_for(30.0, lambda: probe.speed() > 0.4):
        result(name, False, 'the car would not get going')
        if handle is not None:
            finish(probe, handle)
        return
    if not set_fault(probe, 'throttle', 'stuck'):
        result(name, False, 'refused the fault')
        finish(probe, handle)
        return
    t0 = probe.now()
    top = [0.0]
    # Ego's speed and the wheels', as the watchdog sees them, for the record.
    rows, wheels = [], [math.nan]
    subs = [probe.create_subscription(AckermannDriveStamped, '/ego/actuator_cmd',
                                      lambda m: rows.append((probe.now(), m.drive.speed,
                                                             wheels[0], probe.speed())), 10),
            probe.create_subscription(Odometry, '/vehicle/odom',
                                      lambda m: wheels.__setitem__(0, m.twist.twist.linear.x),
                                      10)]
    probe.spin_for(watch, lambda: top.__setitem__(0, max(top[0], probe.speed()))
                   or probe.overspeed_at is not None)
    for sub in subs:
        probe.destroy_subscription(sub)
    stem = os.path.splitext(LAUNCH.get('log', '/tmp/aav-sim-check.log'))[0]
    with open(f'{stem}-overspeed.csv', 'w') as out:
        out.write('t,command,wheels,truth\n')
        out.writelines(f'{t - t0:.3f},{c:.3f},{w:.3f},{v:.3f}\n' for t, c, w, v in rows)
    set_fault(probe, 'throttle', 'ok')
    over_at, overspeed_at = probe.over_at, probe.overspeed_at
    finish(probe, handle)
    probe.spin_for(30.0, lambda: abs(probe.speed()) < STILL)
    clear_watchdog(probe)
    warned = (f'OVER (a warning) {over_at - t0:.1f} s after, '
              if over_at is not None else 'no OVER warning, ')
    said = (f'OVERSPEED {overspeed_at - t0:.1f} s after the fault'
            if overspeed_at is not None
            else f'the watchdog never said OVERSPEED in {watch:.0f} s')
    result(name, overspeed_at is not None,
           f'asked 1.00 m/s, top {top[0]:.2f} m/s; {warned}{said}')


def check_actuator_faults(probe, react=40.0):
    """
    Actuator faults on purpose (TODO 159, faults.ACTUATORS): what each does, as evidence.

    The e-stop from about 1 m/s, healthy, then with a weak brake, a dead one and a stuck
    throttle: how far the car rolls and whether it stands within 30 s. Then the steering
    stuck in a turn, the navigator driving to a goal round the corner: does anything stop the car
    in `react` s, and how far off the straight to the goal it gets. Each line is evidence
    for docs/safety-case.md; none fails because a layer is missing (the safety case says
    which are), only when the fault could not be set or the car would not drive.
    """
    if not fault_client(probe, 'steer').wait_for_services(timeout_sec=20.0):
        result('actuator faults', False, "no sim_adapter parameters: can't set a fault")
        return
    base = None
    for fault in (None, ('brake', 'weak'), ('brake', 'dead'), ('throttle', 'stuck')):
        clear_watchdog(probe)
        out = actuator_stop(probe, fault)
        over = ('; the watchdog said OVERSPEED' if probe.overspeed_at is not None
                else '; no OVERSPEED')
        if out is None:
            continue
        name, v0, metres, took, standing, v_end = out
        base = metres if fault is None else base
        than = f' ({metres / base:.1f} x the healthy {base:.2f} m)' if base and fault else ''
        ended = (f'stood still after {took:.1f} s' if standing
                 else f'still at {v_end:.2f} m/s after {took:.0f} s')
        result(name, True, f'from {v0:.2f} m/s: {metres:.2f} m{than}, {ended}{over}')
    check_overspeed(probe)
    clear_watchdog(probe)
    # The steering stuck in a turn: a goal 10 m on and 10 m left, facing left (a bend of
    # about 10 m radius: 0.1 rad/s at 1 m/s), stuck once the car turns at TURNING.
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)
    gx, gy = x0 + 10.0 * c - 10.0 * s, y0 + 10.0 * s + 10.0 * c
    done = probe.go_to(gx, gy, h + math.pi / 2.0)
    name = 'actuator steer stuck: in a turn'
    if done is None:
        result(name, False, 'no navigator, or it refused the goal')
        return
    turning = probe.spin_for(60.0, lambda: abs(probe.truth.twist.twist.angular.z) > TURNING
                             and probe.speed() > 0.3)
    probe.steer_stuck_at = None
    if not turning or not set_fault(probe, 'steer', 'stuck'):
        result(name, False, 'the car never turned' if not turning else 'refused the fault')
        probe.goal.cancel_goal_async()
        return
    t0, start = probe.now(), len(probe.track)
    rate = probe.truth.twist.twist.angular.z
    probe.spin_for(react)
    run = probe.track[start:]
    # Standing still for 3 s or more (a change of direction passes through 0 too).
    stood, since = None, None
    for t, *_, v in run:
        since = (since if since is not None else t) if abs(v) < STILL else None
        if since is not None and t - since >= 3.0:
            stood = since - t0
            break
    # How far it went from the straight line to the goal, and how far it drove.
    dx, dy = gx - x0, gy - y0
    length = math.hypot(dx, dy)
    off = max((abs((x - x0) * dy - (y - y0) * dx) / length for _, x, y, *_ in run), default=0.0)
    driven = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(run, run[1:]))
    status = done.result().status if done.done() else None
    set_fault(probe, 'steer', 'ok')
    if not done.done():
        probe.goal.cancel_goal_async()
    probe.spin_for(10.0, lambda: abs(probe.speed()) < STILL)
    why = ('the goal ended (status {})'.format(status) if status is not None
           else 'nothing ended the goal')
    still = f'stood still {stood:.0f} s after it' if stood is not None else 'never stood 3 s'
    # TODO 207: the watchdog must notice (steering STUCK, latched), then a person clears it.
    noticed = probe.steer_stuck_at
    seen = (f'the watchdog said steering STUCK {noticed - t0:.1f} s after it'
            if noticed is not None else 'the watchdog never said steering STUCK')
    clear = probe.create_client(Trigger, '/safety/sensor_watchdog/clear')
    if noticed is not None and clear.wait_for_service(timeout_sec=5.0):
        cleared = clear.call_async(Trigger.Request())
        probe.spin_for(10.0, cleared.done)
    result(name, noticed is not None,
           f'stuck turning at {rate:+.2f} rad/s: {seen}; {still} (watched {react:.0f} s); '
           f'drove {driven:.1f} m, {street_check.turns(run)} changes of direction, up to '
           f'{off:.1f} m off the straight to the goal; {why}')


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
POINT_SPEEDS = (1.0, 3.0)       # m/s: the car's ego cap, and sim_speed 3
SPEED_STEPS = (-0.5, 1.0, 2.0)  # m/s: TODO 27's steps (reversing is capped at 0.5)
SPEED_HOLD = 20.0               # s at each step
SPEED_BAND = 0.05               # the band a step must end, and settle, in (of its target)
SPEED_TRIES = 3                 # drives of a step, while late commands spoil them
# Each step's samples (profile, target, try, s, m/s), for a plot: AAV_SIM_SPEED_LOG moves it.
SPEED_LOG = os.environ.get('AAV_SIM_SPEED_LOG', '/tmp/aav-sim-speed-steps.csv')
# Each drive to point's truth, as a CSV for measure atob: AAV_SIM_POINT_LOG moves the folder.
POINT_LOG = os.environ.get('AAV_SIM_POINT_LOG', '/tmp/aav-sim-points')
POINT_TOLERANCE = 2.0           # m from B, by the truth: docs/tests.md 5.2 without RTK
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

    From the truth, over the final stop only: from the last moment at 90 % of the top
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
    Write one drive's truth as a CSV for measure atob: POINT_LOG/<speed>-<n>.csv.

    `pixi run measure atob /tmp/aav-sim-points/*.csv --speed <speed>` scores them (stop error,
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
    """Drive to one point; check the stop against the truth, the braking and overshoot."""
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
    """Drive POINT_TARGETS at this speed (sim_speed); at 1 m/s, its stops too."""
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
    that arrived too late does (sim_adapter's cmd_timeout, 0.2 s: the target drops
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
    # Times on the truth's stamps, the samples' own clock: the scripted driver's
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
            result(name, False, 'the car never moved (the truth)')
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
# --no-cameras (cameras:=false): what only a drawn picture makes.
CAMERA_TOPICS = tuple(f'/sensors/camera/{c}/{t}' for c in ('front_left', 'front_right', 'rear')
                      for t in ('image_raw', 'image_raw/compressed', 'camera_info')) + (
    '/perception/camera/front_left/preview/compressed', '/perception/camera/front_left/preview',
    '/perception/camera/front_right/preview/compressed',
    '/vehicle/sim/chase_camera/image_raw', '/vehicle/sim/chase_camera/compressed')
CAMERAS = True        # --no-cameras: none drawn (Ravensim on a computer with no GPU)


EVERY_RUN = []   # launch arguments every session adds: --speed-loop's, --simulator's
SIMULATOR = None      # --simulator ravensim (the only one); sim.launch.py's default (main)
# What runs: sim.launch.py's default stack (ours, stacks.DEFAULT), or --stack's, with any
# --navigator, --costmap, --planner or --tracker on top (main works it out again).
COSTMAP = 'own'       # the only one since Nav2's went (17 Nov)
LOCAL_COSTMAP = {'own': '/drivers/nav/costmap/local'}
VOXELS = False   # --voxels: every session with the lidar takes voxels:=true
KISS_EKF = False  # --kiss-ekf: every session with the lidar takes kiss_ekf:=true


# m/s: --stop-point's drives. The spine caps the car at its MAX_SPEED, 1.0 m/s (spine-core
# spine.rs, Arsh, 6 Nov): a safety lock, so the drives stay under it.
STOP_SPEEDS = (0.5, 0.75, 1.0)


def programs_held(probe, wait=120.0):
    """
    Wait until programs may drive; return '' then, or why they are still held.

    The geofence holds them until the global pose settles (STARTING), the watchdog while a
    sensor is late: a drive sent before then runs while twist_mux holds it.
    """
    fence = []
    sub = probe.create_subscription(String, '/safety/geofence/state',
                                    lambda m: fence.append(m.data), 10)
    ready = probe.spin_for(wait, lambda: bool(fence) and fence[-1].startswith('OK')
                           and probe.lock is False)
    probe.destroy_subscription(sub)
    return '' if ready else (f'programs still held after {wait:g} s: geofence '
                             f'{fence[-1] if fence else "silent"}, watchdog {probe.state}')


def check_stop_point(probe):
    """
    Drive at each of STOP_SPEEDS, then the e-stop: check the car stops where ego said.

    With the spine in the loop an e-stop is a full brake, what /ego/stop_point predicts
    (aav_ego stop_ahead.py: the shared car model, full brake now). The prediction is the
    first stop point ego stamped at or after the press; it is moved into the map frame by
    /tf at its stamp. Truth: Ravensim's, along the heading at that stamp. Localization: the
    global EKF's stop against the predicted point. Then the e-stop is let go and the spine
    armed again for the next drive.
    """
    from aav_description.car_model import ActuatorConfig
    from aav_description.stop_law import from_car_model
    if not probe.drive.wait_for_server(timeout_sec=20.0):
        result('stop point', False, 'no scripted driver')
        return
    with open(os.path.join(ROOT, 'ros', 'ego', 'aav_ego', 'config', 'ego.yaml')) as f:
        ego = yaml.safe_load(f)['ego']['ros__parameters']
    law = from_car_model(ActuatorConfig.from_vehicle(vehicle.load()),
                         ego['max_deceleration'], ego['max_jerk'])
    waiting = programs_held(probe)
    if waiting:
        result('stop point', False, waiting)
        return
    stops = []
    stem = os.path.splitext(LAUNCH.get('log', '/tmp/aav-sim-check.log'))[0]
    for speed in STOP_SPEEDS:
        name = f'stop point {speed:g} m/s'
        probe.spin_for(30.0, lambda: abs(probe.speed()) < STILL)
        if not probe.armed():
            probe.arm_and_hold()
        begin = probe.now()
        sent = probe.drive.send_goal_async(ScriptedDrive.Goal(
            target_speed=speed, acceleration=0.5, deceleration=1.0, hold_time=30.0))
        probe.spin_for(10.0, sent.done)
        if not sent.done() or not sent.result().accepted:
            result(name, False, 'the scripted drive was refused')
            continue
        handle = sent.result()
        # Steady at the speed for 1.5 s (sim time), then the e-stop.
        steady = []
        probe.spin_for(60.0, lambda: (steady.append(probe.now())
                                      if abs(probe.speed() - speed) < 0.05 else steady.clear())
                       or (steady and steady[-1] - steady[0] > 1.5))
        t0 = probe.now()
        probe.estop.publish(Bool(data=True))
        probe.spin_for(30.0, lambda: probe.now() - t0 > 2.0 and abs(probe.speed()) < STILL)
        top = held(probe, 2.0, press=probe.estop)
        finish(probe, handle)
        track = [stop_points.Sample(*row) for row in probe.track]
        glob = [stop_points.Sample(*row) for row in probe.glob_track]
        with open(f'{stem}-{speed:g}.csv', 'w') as out:
            out.write('kind,t,x,y,yaw,v\n')
            out.writelines(f'stop_point,{t:.3f},{x:.4f},{y:.4f},,\n'
                           for t, x, y in probe.stop_points if t >= begin)
            out.writelines(f'truth,{s.t:.3f},{s.x:.4f},{s.y:.4f},{s.yaw:.4f},{s.v:.4f}\n'
                           for s in track if s.t >= begin)
            out.writelines(f'ekf,{s.t:.3f},{s.x:.4f},{s.y:.4f},{s.yaw:.4f},{s.v:.4f}\n'
                           for s in glob if s.t >= begin)
        said = next(((t, x, y) for t, x, y in probe.stop_points if t >= t0), None)
        end = stop_points.stopped(track, t0, STILL)
        origin = stop_points.at(track, said[0]) if said else None
        rearmed, words = rearm(probe, top)
        if said is None or origin is None or end is None:
            result(name, False, 'no /ego/stop_point after the press' if said is None
                   else 'never stood still')
            continue
        if origin.v < 0.5 * speed:
            result(name, False, f'the car only reached {origin.v:.2f} m/s: no stop to check')
            continue
        st, sx, sy = said
        predicted = math.hypot(sx, sy)
        localized = None
        try:
            from rclpy.time import Time
            tf = probe.tf.lookup_transform('map', 'base_link', Time(seconds=st))
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            o = stop_points.Sample(st, tf.transform.translation.x, tf.transform.translation.y,
                                   yaw, 0.0)
            g1 = stop_points.at(glob, end.t + 1.0)
            if g1 is not None:
                localized = stop_points.along(o, g1)
        except Exception as e:  # noqa: B902 - no TF: no localization's number
            print(f'no TF at the stamp of the stop point: {e}', flush=True)
        stop = stop_points.Stop(speed=origin.v, predicted=predicted,
                                truth=stop_points.along(origin, end), localized=localized,
                                law=law.distance(origin.v))
        stops.append(stop)
        lag = f'; the stop point {1000 * (st - t0):.0f} ms after the press'
        result(name, stop.ok, stop.words() + lag + ('' if rearmed else f'; {words}'))
    located = [s.localized_ok for s in stops if s.localized_ok is not None]
    result('stop point, localization', all(located) if located else None,
           f'{sum(located)} of {len(located)} within the tolerance + '
           f'{stop_points.TOLERANCE:g} m (the global EKF, by /tf at the stamp of the stop point)')


# --loaded-stop: a stop with people aboard, the adaptive brake off and on (docs/stopping.md).
LOADED_SPEED = 2.0       # m/s: no spine here, so ego's sim cap (sim_speed:=2), not 1 m/s
LOADED_PAYLOAD = 150.0   # kg aboard: two adults. The brakes give 181 / 331 = 55 % of empty
# (kg aboard, adaptive brake on): the last is the next stop, with what the first one learnt.
LOADED_CASES = ((0.0, False), (LOADED_PAYLOAD, False), (0.0, True), (LOADED_PAYLOAD, True),
                (LOADED_PAYLOAD, True))
LOADED_HOLD = 8.0        # s at the speed first: the loaded car is slower to get there
LOADED_LOG = '/tmp/aav-sim-loaded-stop.csv'


def check_loaded_stop(probe):
    """
    Stop from LOADED_SPEED, empty and loaded, the adaptive brake off and on.

    Each stop is scored against the empty car model. sim_adapter's payload and
    adaptive_brake are set live for each case. The stop is the scripted drive's own
    slowing (1 m/s^2), through the adapter's speed loop, the adaptive brake (when on) and
    the modelled actuators. The prediction is
    stop_law.stop_following: the empty car model fed the commands the adapter got from
    t0, the moment they start to fall. Pass: within stop_points.TOLERANCE (0.25 m),
    except loaded with the adaptive brake off, which must overshoot by more (else the
    load didn't show, and the other cases prove nothing).
    """
    from aav_description.car_model import ActuatorConfig
    from aav_description.stop_law import stop_following
    client = AsyncParameterClient(probe, '/sim_adapter')
    if not client.wait_for_services(timeout_sec=20.0):
        result('loaded stop', False, 'no sim_adapter parameters')
        return
    if not probe.drive.wait_for_server(timeout_sec=20.0):
        result('loaded stop', False, 'no scripted driver')
        return
    empty = ActuatorConfig.from_vehicle(vehicle.load())
    waiting = programs_held(probe)
    if waiting:
        result('loaded stop', False, waiting)
        return
    with open(LOADED_LOG, 'w') as out:
        out.write('case,payload,adaptive,kind,t,v\n')
    for i, (payload, adaptive) in enumerate(LOADED_CASES):
        again = (payload, adaptive) in LOADED_CASES[:i]
        name = (f'loaded stop {payload:g} kg, adaptive brake {"on" if adaptive else "off"}'
                + (', the next stop' if again else ''))
        if not set_ghost(probe, client, payload=payload, adaptive_brake=adaptive):
            result(name, False, 'sim_adapter refused payload or adaptive_brake')
            continue
        probe.spin_for(30.0, lambda: abs(probe.speed()) < STILL)
        begin = probe.now()
        handle = probe.send_drive(LOADED_SPEED, hold=LOADED_HOLD)
        if handle is None:
            result(name, False, 'the scripted drive was refused')
            continue
        done = handle.get_result_async()
        probe.spin_for(90.0, done.done)
        probe.spin_for(10.0, lambda: probe.now() - begin > 1.0 and abs(probe.speed()) < STILL)
        probe.spin_for(1.0)                 # stood still long enough to count
        commands = [(t, v) for t, v in probe.actuator_cmds if t >= begin]
        track = [stop_points.Sample(*row) for row in probe.track if row[0] >= begin]
        with open(LOADED_LOG, 'a') as out:
            out.writelines(f'{i},{payload:g},{adaptive},command,{t:.3f},{v:.4f}\n'
                           for t, v in commands)
            out.writelines(f'{i},{payload:g},{adaptive},truth,{s.t:.3f},{s.v:.4f}\n'
                           for s in track)
        t0 = stop_points.slowing_start(commands, LOADED_SPEED)
        origin = stop_points.at(track, t0) if t0 is not None else None
        end = stop_points.stopped(track, t0, STILL) if t0 is not None else None
        if origin is None or end is None:
            result(name, False, 'the commands never slowed' if t0 is None
                   else 'never stood still')
            continue
        predicted, _ = stop_following(empty, origin.v, [c for c in commands if c[0] >= t0])
        truth = stop_points.along(origin, end)
        error = truth - predicted
        words = (f'from {origin.v:.2f} m/s: the empty car model {predicted:.2f} m, truth '
                 f'{truth:.2f} m ({error:+.2f}, tolerance {stop_points.TOLERANCE:g})')
        if payload > 0.0 and not adaptive:
            result(name, error > stop_points.TOLERANCE,
                   words + ': the load must show here, past the tolerance')
        else:
            result(name, abs(error) <= stop_points.TOLERANCE, words)
    set_ghost(probe, client, payload=0.0, adaptive_brake=False)


def behaviour_seen(probe, since):
    """Return the behaviour states seen since `since`, in order, each once: 'STATE(reason)'."""
    seen = []
    for t, state, reason, *_ in probe.behaviour:
        word = f'{state}({reason})' if reason not in ('', '-') else state
        if t >= since and (not seen or seen[-1] != word):
            seen.append(word)
    return seen


def check_behaviour_things(probe):
    """
    Check the behaviour with the things on: a parked box is the planner's to go round.

    The car must pass the box as in --plan-around, and the behaviour must never wait for it:
    no APPROACH(obstacle) or STOP(obstacle). The stop zone stays the safety net.
    """
    since = probe.now()
    check_plan_around(probe)
    seen = behaviour_seen(probe, since)
    # Nor held for it as a moving thing (17 Nov): a parked box is passed, not waited for.
    waited = [w for w in seen if w.endswith('(obstacle)') or w.endswith('(moving)')]
    result('behaviour things: no wait for a parked box', bool(seen) and not waited,
           f'states: {" > ".join(seen) or "none heard"}')


# Things moving near the car (Ben, 16 Nov: "if we see a car people or stuff moving around us
# we should likely just stop"): each one must hold the car (WAIT(moving)), which must then
# keep clear of it and reach its goal once it has gone. Our simulator's road world (one
# 4.5 m road on y = 0, -20 to 100 m). Each: name, model, where it starts against the car
# (m ahead of the bumper, m to the left), its speed (m/s along the car's way, to the left).
MOVERS = (
    # An oncoming car in the other lane (3 m to the left), passing us at 2 m/s.
    ('oncoming car', OBSTACLE, 30.0, 3.0, -2.0, 0.0),
    # A person walking along the roadside 5 m to the right, towards the car.
    ('person along the roadside', POST, 18.0, -5.0, -1.0, 0.0),
    # Cross traffic: a car crossing the road 22 m ahead at 3 m/s, from 12 m to the right.
    ('cross traffic', OBSTACLE, 22.0, -12.0, 0.0, 3.0),
)


def check_movers(probe, movers=MOVERS, goal=26.0):
    """
    Check the car holds for each of `movers`, and goes on once it has gone.

    WAIT(moving) seen, the car keeps clear of it while it moves (0.3 m ahead of the
    bumper), and reaches a goal `goal` m on once it has gone. Each starts moving when the
    goal is sent. One a session: the road
    world's fence is near past 60 m.
    """
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']
    for i, (name, model, ahead, side, along_v, side_v) in enumerate(movers):
        probe.spin_for(30.0, lambda: abs(probe.speed()) < STILL)
        x0, y0 = probe.xy(probe.truth)
        h = yaw_of(probe.truth)
        c, s = math.cos(h), math.sin(h)
        px, py = x0 + (front + ahead) * c - side * s, y0 + (front + ahead) * s + side * c
        vx, vy = along_v * c - side_v * s, along_v * s + side_v * c
        entity = f'mover{i}'
        if not place_model(model, px, py, h, name=entity):
            result(f'mover: {name}', False, f'could not place it: {SERVICE_SAID[0]}')
            continue
        since = probe.now()
        start = len(probe.track)
        done = probe.go_to(x0 + goal * c, y0 + goal * s, h)
        if done is None:
            remove_model(entity)
            result(f'mover: {name}', False, 'no navigator, or it refused the goal')
            continue
        t0 = seconds(probe.truth.header.stamp)
        walking = walk_model(entity, px, py, h, vx, vy)
        if walking:
            probe.spin_for(240.0, done.done)
        if not done.done():
            probe.goal.cancel_goal_async()
        remove_model(entity)
        if not walking:
            result(f'mover: {name}', False, f'the simulator would not move it: '
                   f'{SERVICE_SAID[0]}')
            continue
        size = 0.5 if model == POST else 1.0
        gaps = []
        for t, x, y, yaw, v in probe.track[start:]:
            k_t = max(0.0, t - t0)
            wx, wy = px + vx * k_t, py + vy * k_t
            ux, uy = math.cos(yaw), math.sin(yaw)
            along_car = (wx - x) * ux + (wy - y) * uy
            if abs(v) > 0.05 and along_car >= front:
                k = min(front, along_car)
                gaps.append(math.hypot(wx - x - k * ux, wy - y - k * uy)
                            - body['width'] / 2.0 - size / 2.0)
        seen = behaviour_seen(probe, since)
        reached = done.done() and done.result().status == 4
        held = 'WAIT(moving)' in seen
        clear = min(gaps) if gaps else float('nan')
        result(f'mover: {name}', reached and held and not clear <= 0.3,
               f'{"reached" if reached else "did not reach"} the goal; '
               f'{"held" if held else "never held"} for it; {clear:.2f} m clear ahead of '
               f'the moving car; states {" > ".join(seen) or "none heard"}')


def walk_model(name, x, y, yaw, vx, vy):
    """Put a placed model at (x, y, yaw), then walking at (vx, vy) m/s (Ravensim)."""
    return ros_service(
        '/vehicle/sim/set_entity_state', 'simulation_interfaces/srv/SetEntityState',
        f'{{entity: "{name}", state: {{header: {{frame_id: "map"}}, pose: {{position: '
        f'{{x: {x}, y: {y}, z: 0.0}}, orientation: {{z: {math.sin(yaw / 2.0)}, '
        f'w: {math.cos(yaw / 2.0)}}}}}, twist: {{linear: {{x: {vx}, y: {vy}}}}}}}}}')


def check_moving_thing(probe, ahead=20.0, side=4.0, walk=0.8, goal=35.0, near=15.0):
    """
    Check the car lets a person cross: `ahead` m before the bumper, from `side` m right.

    They start walking left at `walk` m/s when the car, at its speed then, would reach
    their path as they reach the lane (3 to `near` m out), so they cross in front of it
    whatever its speed. The car drives to a goal `goal` m on. The behaviour must slow for
    them (APPROACH(obstacle) or STOP(obstacle): a moving thing in the lane, TODO 170), the
    car keep at least 0.3 m from them ahead of its bumper while it moves (the walker walks
    on through a car standing for it), and reach the goal once they are by.
    """
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)
    along = front + ahead
    px, py = x0 + along * c + side * s, y0 + along * s - side * c     # to the right
    vx, vy = -walk * s, walk * c                                         # walking left
    if not place_model(POST, px, py, h, name='walker'):
        result('moving thing', False, f'could not place the walker: {SERVICE_SAID[0]}')
        return
    since = probe.now()
    start = len(probe.track)
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h)
    if done is None:
        remove_model('walker')
        result('moving thing', False, 'no navigator, or it refused the goal')
        return

    def bumper_near():
        # Start them so they reach the lane as the car does: `side / walk` s before the
        # bumper arrives, at the car's speed now (at least 3 m, at most `near` m out).
        x, y = probe.xy(probe.truth)
        recent = probe.track[-6:]
        speed = 0.0
        if len(recent) >= 2 and recent[-1][0] > recent[0][0]:
            speed = (math.hypot(recent[-1][1] - recent[0][1], recent[-1][2] - recent[0][2])
                     / (recent[-1][0] - recent[0][0]))
        lead = min(near, max(3.0, speed * side / walk))
        return (x - x0) * c + (y - y0) * s + front >= along - lead

    probe.spin_for(120.0, lambda: bumper_near() or done.done())
    t0 = seconds(probe.truth.header.stamp)
    walking = walk_model('walker', px, py, h, vx, vy)
    if walking:
        probe.spin_for(240.0, done.done)
    if not done.done():
        probe.goal.cancel_goal_async()
    remove_model('walker')
    if not walking:
        result('moving thing', False, f'the simulator would not walk it: {SERVICE_SAID[0]}')
        return
    gaps, speeds, ahead_of = [], [], []
    for t, x, y, yaw, v in probe.track[start:]:
        speeds.append(abs(v))      # to tell a hit from being walked into while standing
        k_t = max(0.0, t - t0)                   # standing at its start until it walks
        wx, wy = px + vx * k_t, py + vy * k_t
        ux, uy = math.cos(yaw), math.sin(yaw)
        along_car = (wx - x) * ux + (wy - y) * uy
        ahead_of.append(along_car >= front)
        k = max(-body['rear_overhang'], min(front, along_car))
        # The car's centre line against the post's centre: less half the car, half the post.
        gaps.append(math.hypot(wx - x - k * ux, wy - y - k * uy) - body['width'] / 2.0 - 0.25)
    seen = behaviour_seen(probe, since)
    reached = done.done() and done.result().status == 4
    # The gap ahead of the bumper while the car moves: the simulator's walker never stops
    # and passes through the car like a ghost, so beside or inside the car (walked into it
    # while it stood) is not the car's doing. Only driving at someone in front is judged.
    moving = [g for g, v, a in zip(gaps, speeds, ahead_of) if v > 0.05 and a]
    clear = min(moving) if moving else float('nan')
    at = speeds[gaps.index(min(gaps))] if gaps else float('nan')
    touched = min(gaps) if gaps else float('nan')
    # Slowed for them, or held for something moving near (17 Nov).
    slowed = ('APPROACH(obstacle)' in seen or 'STOP(obstacle)' in seen
              or 'WAIT(moving)' in seen)
    result('moving thing', reached and slowed and clear > 0.3,
           f'{"reached" if reached else "did not reach"} the goal, {clear:.2f} m from the '
           f'walker ahead of the moving car ({touched:.2f} m in all, the car at '
           f'{at:.2f} m/s then); states '
           f'{" > ".join(seen) or "none heard"}')


def check_thing_names(probe, near=1.5):
    """
    Check the things' names (things_names:=true, TODO 192).

    The lidar's thing at the sign must be named stop sign by the camera's detector, and
    nothing far from it.

    The car drives past the sign as check_stop_sign does. A thing counts as the sign's when
    it is within `near` m of it (truth, through the car's pose); a "stop sign" further
    than 3 m from it is a wrong name.
    """
    from aav_interfaces.msg import Things
    newest, odom = {}, {}
    subs = [probe.create_subscription(Things, '/world/things',
                                      lambda m: newest.update(things=m), 10),
            probe.create_subscription(Odometry, '/localization/odom/local',
                                      lambda m: odom.update(pose=m), 10)]
    try:
        done = probe.go_to(40.0, ROAD_Y, 0.0)
        if done is None:
            result('thing names', False, 'no navigator, or it refused the goal')
            return
        named_at, wrong, kinds, first = None, 0, set(), None
        end = time.monotonic() + 120.0
        while time.monotonic() < end and not done.done():
            probe.spin_for(0.2)
            if 'things' not in newest or 'pose' not in odom:
                continue
            p = odom['pose'].pose.pose
            ox, oy, oh = p.position.x, p.position.y, yaw_of(odom['pose'])
            tx, ty = probe.xy(probe.truth)
            th = yaw_of(probe.truth)
            for thing in newest['things'].things:
                # The thing from the car (odom), then onto the truth's frame.
                dx, dy = thing.x - ox, thing.y - oy
                ax = dx * math.cos(oh) + dy * math.sin(oh)
                ay = -dx * math.sin(oh) + dy * math.cos(oh)
                wx = tx + ax * math.cos(th) - ay * math.sin(th)
                wy = ty + ax * math.sin(th) + ay * math.cos(th)
                off = math.hypot(wx - SIGN_X, wy - SIGN_Y)
                is_sign = thing.kind.replace('_', ' ').strip().lower() == 'stop sign'
                if off <= near:
                    kinds.add(thing.kind)
                    if first is None:
                        first = tx
                    if is_sign and named_at is None:
                        named_at = (tx, off)
                elif is_sign and off > 3.0:
                    wrong += 1
        ok = named_at is not None and wrong == 0
        seen = (f'the lidar had a thing at the sign from x = {first:.1f} m'
                if first is not None else 'the lidar never had a thing at the sign')
        named = (f'named stop sign at x = {named_at[0]:.1f} m, {named_at[1]:.2f} m off it'
                 if named_at else f'never named stop sign (its names: {sorted(kinds) or "none"})')
        result('thing names', ok, f'{seen}; {named}; {wrong} wrong "stop sign" names elsewhere')
    finally:
        for sub in subs:
            probe.destroy_subscription(sub)


def lowest_cap(probe, since):
    caps = [cap for t, _, _, cap, _ in probe.behaviour if t >= since and cap > 0.0]
    return min(caps) if caps else None


def check_behaviour(probe, sign=False):
    """
    Check the behaviour trial (behaviour:=true): it slows for each stop; the locks still stop.

    With `sign`, the stop sign first (check_stop_sign's rules, plus the speed as the lock
    fired); then an obstacle in the lane (check_obstacle's rules, plus the speed as the
    stop zone fired); then a goal (reached, and how far off it stopped).
    """
    if not probe.spin_for(30.0, lambda: bool(probe.behaviour)):
        result('behaviour', False, 'nothing on /drivers/nav/behaviour/state')
        return
    waiting = programs_held(probe)
    if waiting:
        result('behaviour', False, waiting)
        return
    if sign:
        since = probe.now()
        fired = []
        sub = probe.create_subscription(
            Bool, '/perception/stop_sign',
            lambda m: fired.append(abs(probe.speed())) if m.data and not fired else None, 10)
        check_stop_sign(probe)
        probe.destroy_subscription(sub)
        seen = behaviour_seen(probe, since)
        at = f'{fired[0]:.2f} m/s' if fired else 'never'
        result('behaviour: sign', 'APPROACH(sign)' in seen and bool(fired) and fired[0] <= 0.5,
               f'speed as the lock fired: {at}; lowest cap {lowest_cap(probe, since)}; '
               f'states {" > ".join(seen)}')
    since, monitor = probe.now(), len(probe.monitor)
    speed_at = []

    def watch():
        if not speed_at and any(a == CollisionMonitorState.STOP and p == 'StopZone'
                                for a, p in probe.monitor[monitor:]):
            speed_at.append(abs(probe.speed()))
        return False

    # A wall across the road (TODO 162): with room beside a box, the follower's own costmap
    # steers round it before the cap binds (at 1 m/s only in the last 2 m), and there is
    # no stop to score. A parked box to pass is --behaviour-things' leg.
    timer = probe.create_timer(0.05, watch)
    check_obstacle(probe, model=WALL, depth=WALL_THICK, across=True,
                   name='behaviour: obstacle (the stop zone)')
    probe.destroy_timer(timer)
    seen = behaviour_seen(probe, since)
    at = f'{speed_at[0]:.2f} m/s' if speed_at else 'never'
    result('behaviour: obstacle', 'APPROACH(obstacle)' in seen,
           f'speed as the stop zone fired: {at}; lowest cap {lowest_cap(probe, since)}; '
           f'states {" > ".join(seen)}')
    since = probe.now()
    check_nav_goal(probe)
    seen = behaviour_seen(probe, since)
    result('behaviour: goal', 'APPROACH(goal)' in seen,
           f'lowest cap {lowest_cap(probe, since)}; states {" > ".join(seen)}')


def session(launch_args, checks, log_path, label='', ignore=(), radar=True, spine=False,
            world='road'):
    """
    Start sim.launch.py's world with these arguments, run the checks, and stop it.

    spine: the spine is in the loop (spine:=true): its status must come, and it must arm.
    world: road (the camera and radar must run, the road profile) or empty (only the truth
    and the global EKF, the sim profile).
    """
    road = world == 'road'
    radar = radar and road
    cameras = road and CAMERAS
    watchdog = 'watchdog:=false' not in launch_args
    tag = f' ({label})' if label else ''
    # What a run leaves out on purpose isn't missing from the graph.
    if VOXELS and 'lidar:=false' not in launch_args:
        # The trial: the cubes instead of the scan, which then has no publisher.
        launch_args = list(launch_args) + ['voxels:=true']
        ignore = tuple(ignore) + ('/sensors/lidar/scan',)
    if KISS_EKF and 'lidar:=false' not in launch_args:
        launch_args = list(launch_args) + ['kiss_ekf:=true']
    ignore = (tuple(ignore) + (LIDAR_TOPICS if 'lidar:=false' in launch_args else ())
              + (WATCHDOG_TOPICS if 'watchdog:=false' in launch_args else ())
              + (() if CAMERAS else CAMERA_TOPICS))
    env = dict(os.environ)
    print(f"Starting Ravensim's {world} world{tag} on ROS domain "
          f"{env['ROS_DOMAIN_ID']} (this computer only)...", flush=True)
    log = open(log_path, 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py', f'world:={world}',
                               'rviz:=false', *launch_args, *EVERY_RUN],
                              cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    LAUNCH.update(pgid=launch.pid, log=log_path)
    rclpy.init()
    probe = Probe()
    try:
        def missing():
            return [n for n, m in (('truth', probe.truth), ('global EKF', probe.glob),
                                   ('camera', probe.image if cameras else True),
                                   ('lens', probe.info if cameras else True),
                                   ('radar', probe.cloud if radar else True),
                                   # Its heartbeat: without it twist_mux holds the drivers.
                                   ('the sensor watchdog', probe.lock if watchdog else True))
                    if m is None]

        up = probe.spin_for(120.0, lambda: not missing())
        # Navigation comes up last: the navigator serves its goals once the planner,
        # the follower and the costmaps answer.
        nav = 'nav:=false' not in launch_args
        nav_up = up and (not nav or probe.spin_for(120.0, probe.nav.server_is_ready))
        if not up:
            detail = f'nothing from: {", ".join(missing())}, in 120 s'
        elif not nav_up:
            detail = ('the sensors run, but the navigator never came up in 120 s '
                      f'(see {log_path})')
        else:
            detail = ("Ravensim's truth, the global EKF" +
                      (', the camera' if cameras else '') +
                      (' and the radar stand-in are' if radar else ' are (no radar)') +
                      ' running' + (', and navigation is up' if nav else ''))
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


def main():
    # --goto and --goto-only: the old names (before 26 Oct), still accepted.
    args = [OLD_FLAGS.get(a, a) for a in sys.argv[1:]]
    if '-h' in args or '--help' in args:
        print(__doc__)
        return 0
    perception = '--perception' in args
    if {'--thing-names', '--thing-names-only'} & set(args):
        perception = perception or importlib.util.find_spec('rfdetr') is not None
        if importlib.util.find_spec('rfdetr') is None:
            print('--thing-names needs the detector: pixi run -e ml sim-check --thing-names',
                  file=sys.stderr)
            return 2
    if perception and importlib.util.find_spec('rfdetr') is None:   # the detector's default
        print('--perception needs the detector: pixi run -e ml sim-check --perception',
              file=sys.stderr)
        return 2
    lidar = '--no-lidar' not in args
    global VOXELS, KISS_EKF
    VOXELS = lidar and '--voxels' in args
    KISS_EKF = lidar and '--kiss-ekf' in args
    scan_check = check_voxels if VOXELS else check_lidar_scan
    global CAMERAS
    CAMERAS = '--no-cameras' not in args
    if not CAMERAS:
        if perception:
            print('--no-cameras leaves nothing for --perception to see',
                  file=sys.stderr)
            return 2
        EVERY_RUN.append('cameras:=false')
    checks = ([check_sensors, check_cameras] if CAMERAS else [check_radar])
    checks += ([check_lidar, scan_check] if lidar else []) + [check_drive]
    checks += [check_kiss_ekf] if KISS_EKF else []
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
        if at + 1 >= len(args) or args[at + 1] not in ('default', 'smooth', 'lqi', 'firm'):
            print('--speed-loop takes default, smooth, lqi or firm', file=sys.stderr)
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
        # --python-legacy: aav_legacy's Python legacy_uart_bridge, not the Rust one.
        if '--python-legacy' in args:
            EVERY_RUN.append('legacy_impl:=python')
    global SIMULATOR
    from aav_sim_models import worlds
    SIMULATOR = worlds.DEFAULT_SIMULATOR
    if '--simulator' in args:
        at = args.index('--simulator')
        given = args[at + 1] if at + 1 < len(args) else ''
        try:
            SIMULATOR = worlds.check_simulator(given)
        except ValueError as e:
            print(f'--simulator: {e}', file=sys.stderr)
            return 2
    # Always said: the checks place models and read logs the way this simulator wants.
    EVERY_RUN.append(f'simulator:={SIMULATOR}')
    global COSTMAP
    chosen = {}
    # --with name:=value (repeatable): any sim.launch.py argument, on every session.
    for at, a in enumerate(args):
        if a == '--with':
            if at + 1 >= len(args) or ':=' not in args[at + 1]:
                print('--with takes name:=value', file=sys.stderr)
                return 2
            EVERY_RUN.append(args[at + 1])
    for flag_name, choices in (('--costmap', vehicle.COSTMAPS), ('--tracker', vehicle.TRACKERS),
                               ('--navigator', vehicle.NAVIGATORS),
                               ('--planner', vehicle.PLANNERS)):
        if flag_name in args:
            at = args.index(flag_name)
            if at + 1 >= len(args) or args[at + 1] not in choices:
                print(f'{flag_name} takes {", ".join(choices)}', file=sys.stderr)
                return 2
            EVERY_RUN.append(f'{flag_name[2:]}:={args[at + 1]}')
            if flag_name == '--tracker' and args[at + 1] in vehicle.FORWARD_ONLY_TRACKERS:
                EVERY_RUN.append('reversing:=false')     # aav_track's: forward only
            if flag_name == '--costmap':
                COSTMAP = args[at + 1]
            chosen[flag_name[2:]] = args[at + 1]
    # The stack (stacks.yaml, docs/league.md): --stack's, else the launch's default (ours).
    # The launch sets its switches, and a flag above, set by hand, still wins over it.
    from aav_description import stacks
    league = stacks.load()
    stack = stacks.DEFAULT
    if '--stack' in args:
        at = args.index('--stack')
        if at + 1 >= len(args) or args[at + 1] not in stacks.names(league):
            print(f'--stack takes {", ".join(stacks.names(league))}', file=sys.stderr)
            return 2
        stack = args[at + 1]
        EVERY_RUN.append(f'stack:={stack}')
    picked = stacks.switches(league, stack)
    for level in ('costmap', 'tracker', 'navigator', 'planner'):
        chosen.setdefault(level, picked.get(level))
    COSTMAP = chosen['costmap'] or COSTMAP
    if CAMERAS:
        # The launch starts the cameras with cargo run, which builds them first if they
        # changed: minutes for Bevy, longer than start-up's 120 s. Build them here instead.
        from aav_sim_models.worlds import SIM_CAMERAS, sim_cameras_features
        print("Building Ravensim's cameras (aav_sim_cameras), if they changed...",
              flush=True)
        built = subprocess.run(['cargo', 'build', '--release', '-q', '--features',
                                sim_cameras_features(os.environ),
                                '--manifest-path', os.path.join(ROOT, SIM_CAMERAS)])
        if built.returncode != 0:
            print("Ravensim's cameras did not build: see above, or try --no-cameras",
                  file=sys.stderr)
            return 1
    log = os.environ.get('AAV_SIM_LOG', '/tmp/aav-sim-check.log')
    logs = [log]
    if not {'--drive-to-point-only', '--speed-steps-only', '--spine-points-only',
            '--stop-point-only', '--loaded-stop-only', '--behaviour-only',
            '--things-only', '--behaviour-things-only', '--legs-only', '--movers-only',
            '--three-point-only', '--blocked-street-only', '--moving-only', '--low-box-only',
            '--thing-names-only',
            '--actuator-faults-only', '--kerb-only'} & set(args):
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
        session(['radar:=false', 'lidar:=true', 'plan_around:=false'],
                [check_lidar_obstacle] + ([check_voxels] if VOXELS else []),
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
        # Its own run: the ghost is one more stop zone source, beside the radar and
        # the lidar, so the other runs keep the graph the car has.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-ghost{ext}')
        session(['ghost:=true', f'lidar:={str(lidar).lower()}'], [check_ghost], logs[-1],
                'ghost')
    if {'--kerb', '--kerb-only'} & set(args):
        # Two runs (TODO 119): the lidar alone, then with the radar, as on the car. The
        # planner is blind to the kerb (plan_around:=false): only seeing it stops the car.
        stem, ext = os.path.splitext(log)
        for radar_on, words in ((False, 'the lidar'), (True, 'the lidar and the radar')):
            logs.append(f'{stem}-kerb-{"radar" if radar_on else "lidar"}{ext}')
            session(['lidar:=true', f'radar:={str(radar_on).lower()}', 'plan_around:=false'],
                    [lambda probe, w=words: check_kerb(probe, w)], logs[-1], f'kerb, {words}',
                    ignore=() if radar_on else RADAR_TOPICS, radar=radar_on)
    if {'--actuator-faults', '--actuator-faults-only'} & set(args):
        # Its own run, the empty world (room to roll and to circle): the actuators' faults
        # on purpose (TODO 159), sim_adapter's. No lidar or radar: a lighter run.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-actuator-faults{ext}')
        session(['lidar:=false', 'radar:=false'], [check_actuator_faults], logs[-1],
                'actuator faults', world='empty')
    if '--faults' in args:
        # Its own run: a fault spoils the car's software for a while, and the camera's
        # picture and the lidar's cloud go through their relays (faults:=true). With the
        # lidar, as the car: its own fault, `lidar stop` (TODO 246).
        watchdog = '--no-watchdog' not in args
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-faults{ext}')
        session(['faults:=true', f'watchdog:={str(watchdog).lower()}',
                 f'watch_hud:={str(watchdog).lower()}'],
                [lambda probe: check_faults(probe, watchdog)], logs[-1], 'faults')
    if {'--drive-to-point', '--drive-to-point-only', '--spine-points',
            '--spine-points-only'} & set(args):
        shutil.rmtree(POINT_LOG, ignore_errors=True)
    if '--drive-to-point' in args or '--drive-to-point-only' in args:
        # Its own runs, one per speed (sim_speed is ego's cap, set at launch): the empty
        # world, so any angle is open ground. No navigation, lidar or radar: lighter runs.
        stem, ext = os.path.splitext(log)
        for speed in POINT_SPEEDS:
            logs.append(f'{stem}-drive-to-point-{speed:g}{ext}')
            session(['nav:=false', 'lidar:=false', 'radar:=false', f'sim_speed:={speed:g}'],
                    [lambda probe, v=speed: check_drive_to_point(probe, v)], logs[-1],
                    f'drive to point {speed:g} m/s', ignore=NAV_TOPICS, world='empty')
    if '--spine-points' in args or '--spine-points-only' in args:
        # The same drives with the spine in the loop (spine:=true): spine-core's own speed
        # loop stops the car. Only the drives: an e-stop would latch the spine.
        stem, ext = os.path.splitext(log)
        for speed in POINT_SPEEDS:
            logs.append(f'{stem}-spine-drive-to-point-{speed:g}{ext}')
            session(['spine:=true', 'nav:=false', 'lidar:=false', 'radar:=false',
                     f'sim_speed:={speed:g}'],
                    [lambda probe, v=speed: check_drive_to_point(probe, v, stops=False)],
                    logs[-1], f'drive to point, spine, {speed:g} m/s', ignore=NAV_TOPICS,
                    spine=True, world='empty')
    if '--speed-steps' in args or '--speed-steps-only' in args:
        # Its own run: the empty world, nothing else driving (no navigation), ego's cap at 2 m/s.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-speed{ext}')
        session(['nav:=false', 'lidar:=false', 'radar:=false', 'sim_speed:=2'],
                [lambda probe: check_speed_steps(probe, profile)], logs[-1],
                f'speed steps, {profile}', ignore=NAV_TOPICS, world='empty')
    if '--spine-steps' in args:
        # The same steps with the spine in the loop (spine:=true): spine_sim runs
        # spine-core's own speed loop (Rust), not the actuator model's Python port.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-spine-speed{ext}')
        session(['spine:=true', 'nav:=false', 'lidar:=false', 'radar:=false', 'sim_speed:=2'],
                [lambda probe: check_speed_steps(probe, profile)], logs[-1],
                f'speed steps, spine, {profile}', ignore=NAV_TOPICS, spine=True, world='empty')
    if '--stop-point' in args or '--stop-point-only' in args:
        # Its own run: the empty world, nothing else driving, ego's cap at the top speed.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-stop-point{ext}')
        session(['spine:=true', 'nav:=false', 'lidar:=false', 'radar:=false',
                 f'sim_speed:={max(STOP_SPEEDS):g}'],
                [check_stop_point], logs[-1], 'stop point', ignore=NAV_TOPICS, spine=True,
                world='empty')
    if '--loaded-stop' in args or '--loaded-stop-only' in args:
        # Its own run: the empty world, nothing else driving, no spine (the adaptive brake
        # sits after sim_adapter's own speed loop), ego's cap at the drive's speed.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-loaded-stop{ext}')
        session(['nav:=false', 'lidar:=false', 'radar:=false',
                 f'sim_speed:={LOADED_SPEED:g}'],
                [check_loaded_stop], logs[-1], 'loaded stop', ignore=NAV_TOPICS,
                world='empty')
    if '--behaviour' in args or '--behaviour-only' in args:
        # Its own run: the trial on, the planner blind to the box (the cap must slow, the
        # stop zone must stop), the lidar on for the lane ahead. The things off: with them
        # (stack:=ours' default since 16 Nov) the obstacle stop is only for what moves, and
        # a wall the planner can't see is the stop zone's alone. The scan's obstacle stop
        # is what this run checks; --behaviour-things checks the things' way.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-behaviour{ext}')
        session(['behaviour:=true', 'lidar:=true', 'plan_around:=false', 'things:=false',
                 f'perception:={str(perception).lower()}'],
                [lambda probe: check_behaviour(probe, sign=perception)], logs[-1],
                'behaviour')
    if {'--behaviour-things', '--behaviour-things-only'} & set(args):
        # Its own run: the behaviour reads the things, the planner sees the box (gate 170).
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-behaviour-things{ext}')
        session(['behaviour:=true', 'things:=true', 'lidar:=true'],
                [check_behaviour_things], logs[-1], 'behaviour things')
    if {'--moving', '--moving-only'} & set(args):
        # Its own run: the behaviour reads the things, which see the walker (TODO 170).
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-moving{ext}')
        session(['behaviour:=true', 'things:=true', 'lidar:=true'],
                [check_moving_thing], logs[-1], 'moving thing')
    if {'--movers', '--movers-only'} & set(args):
        # Its own run: an oncoming car, a person along the roadside, cross traffic; the
        # behaviour must hold for each (Ben, 16 Nov).
        stem, ext = os.path.splitext(log)
        for i, mover in enumerate(MOVERS):
            logs.append(f'{stem}-movers-{i}{ext}')
            session(['behaviour:=true', 'things:=true', 'lidar:=true'],
                    [lambda probe, m=mover: check_movers(probe, [m])], logs[-1],
                    f'mover: {mover[0]}')
    if {'--thing-names', '--thing-names-only'} & set(args):
        # Its own run: the detector names the lidar's things (TODO 192; needs -e ml).
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-thing-names{ext}')
        session(['perception:=true', 'lidar:=true', 'things:=true', 'things_names:=true'],
                [check_thing_names], logs[-1], 'thing names')
    if {'--low-box', '--low-box-only'} & set(args):
        # Two runs, the radar off: without the lidar's memory, then with 3 s of it.
        stem, ext = os.path.splitext(log)
        for memory in ('0.0', '3.0'):
            logs.append(f'{stem}-low-box-{memory}{ext}')
            session(['radar:=false', 'lidar:=true', f'lidar_memory:={memory}'], [check_low_box],
                    logs[-1], f'low box, lidar_memory {memory} s', ignore=RADAR_TOPICS,
                    radar=False)
    if {'--three-point', '--three-point-only'} & set(args):
        # Its own run: the lidar sees the walls, the default stack reverses (TODO 174).
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-three-point{ext}')
        session(['lidar:=true'], [check_three_point_turn], logs[-1], 'three-point turn')
    if {'--blocked-street', '--blocked-street-only'} & set(args):
        # Its own run, as the three-point turn's: the lidar sees the walls (TODO 174).
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-blocked-street{ext}')
        session(['lidar:=true'], [check_blocked_street], logs[-1], 'blocked street')
    if {'--things', '--things-only'} & set(args):
        # Its own run: the cubes and the things on, the car standing, a box placed.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-things{ext}')
        session(['things:=true', 'lidar:=true'], [check_things], logs[-1], 'things')

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

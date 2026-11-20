#!/usr/bin/env python3
"""
Controller comparison: our followers and the trial trackers over the same routes, simulated.

Nav2 went on 17 Nov (TODO 184): its last scores are in docs/league-results-2026-11-16.md,
and every run since is on our own stack (stack:=best, the champions).

Each drives the test lot, headless, and is measured against where the car really was.

    pixi run measure controllers                   (mppi_lag, both tours: ~8 min)
    pixi run measure controllers --tours obstacle   (only the box in the lane)
    pixi run measure controllers --speed 3          (the follower and ego up to 3 m/s)
    pixi run measure controllers --tours lot --legs 1   (only the first leg: a quick check)
    pixi run measure controllers --reversing        (the car may reverse: reversing:=true)
    pixi run measure controllers --plans            (keep every plan, not just each leg's first)
    pixi run measure controllers --tours campus --speed 2.5 --trackers pure_pursuit stanley
                    Carleton's roads; every report opens with which rides best: reached every
                    goal, then the least sideways swing (drive_metrics.sway)

The trials (docs/trials.md):
    pixi run measure controllers --trackers pure_pursuit stanley lqr mpc --runs 3
                    aav_track's trackers (tracker:=), forward only; add --trackers
                    mppi_lag for the champion beside them
    pixi run measure controllers --runs 3           (each follower 3 times: the mean and spread)
    pixi run measure controllers --trackers mpc_lag mppi_lag --lag-scale 2
                    the car's lags twice what vehicle.yaml says (sim's lag_scale:=2), the
                    followers none the wiser: how each copes with a slower car
    pixi run measure controllers --amcl             (AMCL beside the EKF, amcl:=true: scored
                    against the truth, as the EKF is; ours, aav_amcl_rs)
    pixi run measure controllers --ndt              (NDT beside the EKF, ndt:=true, on the map
                    pixi run ndt-map drive made: its errors, the share of clouds that gave
                    a pose, time over 1 m off, each match's wall-clock time; and its
                    covariance against its errors: NEES, the k that would make it honest,
                    and how far off each band of score per point was)
    pixi run measure controllers --ndt --ndt-map bags/<drive>/map/ndt.pcd
                    (NDT on a map map-drive made from a recording, TODO 250)
    pixi run measure controllers --ndt-ekf          (the same, with NDT's pose into the global
                    EKF, ndt_ekf:=true: compare its EKF numbers with --ndt's)
    pixi run measure controllers --ndt --gps-off    (the GPS stops once the EKF has its first
                    fix; with watchdog:=false geofence:=false, or they would hold the car)
    pixi run measure controllers --ndt-leads        (NDT leads ekf_global, the GPS checks:
                    pose_source:=ndt, TODO 210; each leg's note says pose_check's sources,
                    hand-overs and seeds)
    pixi run measure controllers --ndt --gps-fault drift --runs 3
    pixi run measure controllers --ndt-leads --gps-fault drift --runs 3
                    a GPS fault on purpose once the EKF has its first fix (sim-fault gps
                    stop, jump or drift), the watchdog and the geofence ON: does the car stop,
                    or drive on with a good pose? Latches are not cleared (a person would),
                    and a leg the locks hold still for 20 s is cancelled
    pixi run measure controllers --voxels --tours lot
                    the costmap and the stop zone read the lidar as cubes (voxels:=true,
                    aav_voxels) instead of the flat scan: the same legs, and each leg's
                    note says the voxels node's ms a cloud and cubes a cloud
    pixi run measure controllers --kiss-icp         (KISS-ICP lidar odometry, kiss_icp:=true,
                    beside ekf_local: each lined up with the truth at the start, its drift
                    per 100 m, its heading at the end, each match's wall-clock time; per
                    leg and for the whole tour. Needs pixi install --locked -e kiss)
    pixi run measure controllers --kiss-ekf         (the same, with KISS-ICP's twist into
                    ekf_local, kiss_ekf:=true: compare its ekf_local numbers with
                    --kiss-icp's)
    pixi run measure controllers --planners own own_hybrid own32 --reversing --tours lot
                    our planner's searches and files (planner:=), each with mppi_lag unless
                    --trackers says otherwise. Before each leg it also asks the planner
                    alone for a path: its planning time, the path's length and its tightest
                    turn against the car's full lock. Then a table of the whole tour: legs
                    finished, time, path length, planning time, kept to the turning radius
                    (docs/controllers.md, Planners); own32: the lattice's 32-heading file
    pixi run measure controllers --planners own own_hybrid --reversing --tours lot
                    our planner's two searches: the State Lattice (own) against Hybrid-A*
                    (own_hybrid: planner_search:=hybrid; docs/own-planner.md)
    pixi run measure controllers --stacks own own_calm --tours fieldhouse --speed 2.5
                    the campus route, P4 to the Fieldhouse (453 m, TODO 140): the campus
                    graph's GPS waypoints, as street_router makes them, sent to the
                    navigator's follow_gps_waypoints, which passes them as one
                    through-poses goal. own is ours as it was until 15 Nov; own_calm the
                    sim's ours stack now (plan_smoothing:=strong, map_correction:=slow,
                    mppi_lag_steer_gamma:=0.05); own_slow and own_strong(_slow) each step.
                    --series keeps each leg's raw series
    pixi run measure controllers --stacks own_calm own_calm_chunks --tours lot campus
            university --runs 2 --reversing --launch behaviour:=true lane_route:=true
                    TODO 175: our global costmap from the map's chunks (costmap_chunks:=auto,
                    ours' since 16 Nov) against the whole map; each leg's notes give the
                    costmap's and planner's CPU and memory, the plan times and the plans
                    cut at the costmap's edge
    pixi run measure controllers --stacks own_calm --tours university --runs 2 --reversing
            --launch behaviour:=true lane_route:=true
                    TODO 148, path:=lane: University Drive, about 290 m of mapped road
                    in 3 legs (the campus tour has ~30 s a leg); again with nothing more,
                    and with lane_costs:=true; tools/lane_score.py on each results.json
    pixi run measure controllers --planners own diffusion --reversing --tours lot
                    the diffusion trial beside our lattice: each leg's note says how many
                    plans came from the model, how many from the lattice (the guard's
                    fallback) and why, and the guard's refusals by check.
                    diffusion_nomodel: the same with no model file (every plan the lattice's).
                    diffusion_rs: its model from Rust (ONNX Runtime, Core ML on a Mac);
                    diffusion_coreml: the Python worker on Core ML.
                    --diffusion-model <.onnx> another model; --diffusion-record <folder>
                    each plan's inputs, for pixi run diffusion replay

For each tour and controller it:
1. starts sim.launch.py world:=test_lot rviz:=false stack:=best (the
   champions, then the piece under test on top) tracker:=<name> [planner:=<name>]
   reversing:=false (forward only, unless --reversing), on a ROS domain of its
   own, on this computer only;
2. waits for the simulator's truth, the global EKF and the navigator;
3. sends each leg's goal in turn, each from where the last one ended, and records where the
   car really was (/vehicle/sim/truth), the planner's paths (/plan), the controller's
   speed (/cmd_vel_nav), ego's steering, the sensor watchdog and the stop zone; with a
   tracker its tick (/drivers/nav/track/status); with --amcl, /localization/amcl/pose;
   with --ndt, /localization/ndt/pose and /localization/ndt/status; with --kiss-icp,
   /localization/kiss_icp/odom and status, and ekf_local's /localization/odom/local;
4. stops the simulator by its process group, and clears Fast DDS's shared memory
   (docs/path-followers.md).

Tours (map frame: the car starts at 0, 0 facing east; the walls, building and parked cars
are aav_sim_models worlds/test_lot.sdf):
  lot       a left turn; round the building to the lane north of it; a turn back in that
            14 m wide lane; home
  obstacle  30 m east with the bench's 1 m box in the lane at 15.5 m (obstacle:=true).
            The global costmap holds only the map, so the planner's path goes through the
            box: only the follower and the stop zone can deal with it

What each number means: drive_metrics.py (COLUMNS). The results go to
log/controller-compare/<time>/: results.md, a picture of each tour (<tour>.svg: where each
controller took the car), results.json (the numbers; each leg's track, the EKF's belief,
the controller's speed command and its first plan) and each run's launch log.
"""

import argparse
import datetime
import json
import math
import os
import random
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
# On this computer only: the comparison can't disturb a car, another laptop or someone
# else's simulator. Each run also gets its own ROS domain (private()),
# so one still shutting down can't meet the next (21 Oct: the second run never came up).
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from aav_description import vehicle  # noqa: E402
from aav_sim_models import worlds  # noqa: E402
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
import drive_metrics  # noqa: E402
from geographic_msgs.msg import GeoPose  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose, FollowGPSWaypoints, NavigateToPose  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from robot_localization.srv import FromLL  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
import yaml  # noqa: E402

# The campus tours' geofence: a 600 m circle round lot P4, as docs/routes.md runs it. The
# sim's 200 m holds every program before the campus's further roads.
CAMPUS_FENCE = 'geofence_radius:=600'
# (name, x m, y m, heading deg): each leg's goal, map frame.
TOURS = {
    'lot': {'obstacle': False, 'legs': [
        ('a left turn', 15.0, 15.0, 90.0),
        ('round the building', 35.0, 42.0, 0.0),
        ('turn back', 20.0, 42.0, 180.0),
        ('home', 0.0, 0.0, 0.0),
    ]},
    'obstacle': {'obstacle': True, 'legs': [
        ('past the box', 30.0, 0.0, 0.0),
    ]},
    # Carleton (pixi run campus): only when asked for, at the campus's speed. Long roads and real
    # bends, where a wobble shows. Round the bend's route goes 216 m from the start: the sim's
    # 200 m fence held the car at a junction 196.9 m out (14 Nov night), so the campus's fence.
    'campus': {'obstacle': False, 'world': worlds.CAMPUS, 'launch': [CAMPUS_FENCE], 'legs': [
        ('out of the lot', 73.0, -38.0, 45.0),
        ('along the road', 165.0, -111.0, -40.0),
        # Facing the way the route arrives (109 deg, up the road; the road runs at -71 deg,
        # measured on campus.pgm, 14 Nov). The roads don't join from the north, so the route
        # loops round and comes up from the south: facing -71 meant a three-point turn in a
        # 6.5 m street at the goal (TODO 174 tests that on its own).
        ('round the bend', 139.0, -190.0, 109.0),
    ]},
    # TODO 148: more mapped road than the campus tour (whose middle goals are off the lane
    # map): out of P4 onto University Drive heading south-east, down it 135 m, then round
    # the bend south 150 m; each goal on the right lane's centre (maps/campus/lanes.yaml),
    # facing the way the road runs there. lattice-planner's examples/lane.rs plans it too.
    'university': {'obstacle': False, 'world': worlds.CAMPUS, 'launch': [CAMPUS_FENCE],
                   'legs': [
                       ('onto University Drive', 87.7, -72.6, -67.4),
                       ('down University Drive', 139.9, -196.4, -67.0),
                       ('round the bend south', 169.0, -337.4, -129.0),
                   ]},
    # Ben, 16 Nov: keep right, leave the lane only to pass. The car starts on University
    # Drive's right lane (its straight 115 m, heading -66.9 deg), a 1 m box parked on that
    # lane's centre 35 m ahead; the goal is the university tour's second, 90 m on. The car
    # must pass the box (into the other lane only as needed), then come back to its lane.
    # lane_score.py's pass_score scores it. Our simulator only (spawn_entity).
    'university_box': {'obstacle': False, 'world': worlds.CAMPUS,
                       'launch': [CAMPUS_FENCE, 'start_x:=104.68', 'start_y:=-113.49',
                                  'start_yaw:=-1.1676'],
                       'boxes': [(118.39, -145.69)],
                       'legs': [('past the parked box', 139.9, -196.4, -67.0)]},
    # TODO 149: the wiggle's road, in one goal from lot P4: out of the lot, then about 120 m
    # of gently bending road (the campus tour's second leg, without its stop in between,
    # which left the car on a cell the costmap called occupied: "Start occupied").
    'wiggle': {'obstacle': False, 'world': worlds.CAMPUS, 'legs': [
        ('P4 along the road', 165.0, -111.0, -40.0),
    ]},
    # TODO 140: directions on the campus, P4 to the Fieldhouse, as GPS waypoints (route:
    # from, to, by the campus graph). The leg's goal is the last waypoint, put on the map
    # by /fromLL when the run starts (None until then).
    # The fence: as docs/routes.md runs it (200 m stops it half way).
    'fieldhouse': {'obstacle': False, 'world': worlds.CAMPUS, 'route': ('P4', 'Fieldhouse'),
                   'launch': [CAMPUS_FENCE],
                   'legs': [('P4 to the Fieldhouse', None, None, None)]},
}
# --stacks: every navigation piece ours (mppi_lag follows), as it was or with a trial.
STACKS = {
    'own': ('tracker', 'mppi_lag',
            ['planner:=own', 'costmap:=own', 'navigator:=own']),
    # All ours on the lattice's 32-heading file: a road at an angle is less of a
    # staircase (the sway's third source, 15 Nov). A trial against own.
    'own32': ('tracker', 'mppi_lag',
              ['planner:=own', 'costmap:=own', 'navigator:=own', 'lattice_headings:=32']),
    # All ours, the follower taking the GPS's map corrections slowly (TODO 186): a
    # challenger against own, scored in the same runs.
    'own_slow': ('tracker', 'mppi_lag',
                 ['planner:=own', 'costmap:=own', 'navigator:=own', 'map_correction:=slow']),
    # The planner's smoother pulling harder (plan_smoothing:=strong, TODO 168): no
    # staircase on a road at an angle. Alone, and with the slow map correction.
    'own_strong': ('tracker', 'mppi_lag',
                   ['planner:=own', 'costmap:=own', 'navigator:=own',
                    'plan_smoothing:=strong']),
    'own_strong_slow': ('tracker', 'mppi_lag',
                        ['planner:=own', 'costmap:=own', 'navigator:=own',
                         'plan_smoothing:=strong', 'map_correction:=slow']),
    # And mppi_lag's control cost on its steering 3 times nav2.yaml's gamma
    # (mppi_lag_steer_gamma:=0.05): less sampling jitter (TODO 168).
    'own_calm': ('tracker', 'mppi_lag',
                 ['planner:=own', 'costmap:=own', 'navigator:=own',
                  'plan_smoothing:=strong', 'map_correction:=slow',
                  'mppi_lag_steer_gamma:=0.05']),
    # own_calm with our global costmap from the map's chunks round the car and along the
    # route (costmap_chunks:=auto, TODO 175): the campus's; elsewhere the whole map.
    'own_calm_chunks': ('tracker', 'mppi_lag',
                        ['planner:=own', 'costmap:=own', 'navigator:=own',
                         'plan_smoothing:=strong', 'map_correction:=slow',
                         'mppi_lag_steer_gamma:=0.05', 'costmap_chunks:=auto']),
    # own_calm_chunks with the kerbs (kerbs:=true, TODO 119): the step finder's kerbs in
    # the local costmap. Against own_calm_chunks: no phantom stops?
    'own_calm_chunks_kerbs': ('tracker', 'mppi_lag',
                              ['planner:=own', 'costmap:=own', 'navigator:=own',
                               'plan_smoothing:=strong', 'map_correction:=slow',
                               'mppi_lag_steer_gamma:=0.05', 'costmap_chunks:=auto',
                               'kerbs:=true']),
    # own_calm_chunks routed on the campus's Lanelet2 map (lane_graph:=lanelet2); and that
    # with the local costmap from the chunks too (local_chunks:=true): TODO 175.
    'own_calm_chunks_ll2': ('tracker', 'mppi_lag',
                            ['planner:=own', 'costmap:=own', 'navigator:=own',
                             'plan_smoothing:=strong', 'map_correction:=slow',
                             'mppi_lag_steer_gamma:=0.05', 'costmap_chunks:=auto',
                             'lane_graph:=lanelet2']),
    'own_calm_chunks_maps': ('tracker', 'mppi_lag',
                             ['planner:=own', 'costmap:=own', 'navigator:=own',
                              'plan_smoothing:=strong', 'map_correction:=slow',
                              'mppi_lag_steer_gamma:=0.05', 'costmap_chunks:=auto',
                              'lane_graph:=lanelet2', 'local_chunks:=true']),
    'own_calm_chunks_local': ('tracker', 'mppi_lag',
                              ['planner:=own', 'costmap:=own', 'navigator:=own',
                               'plan_smoothing:=strong', 'map_correction:=slow',
                               'mppi_lag_steer_gamma:=0.05', 'costmap_chunks:=auto',
                               'local_chunks:=true']),
}
STACK_LABELS = {'own': 'all-ours', 'own32': 'all-ours-32',
                'own_slow': 'all-ours-slow', 'own_strong': 'all-ours-strong',
                'own_strong_slow': 'all-ours-strong-slow', 'own_calm': 'all-ours-calm',
                'own_calm_chunks': 'all-ours-calm-chunks',
                'own_calm_chunks_kerbs': 'all-ours-calm-kerbs',
                'own_calm_chunks_ll2': 'all-ours-calm-chunks-ll2',
                'own_calm_chunks_maps': 'all-ours-calm-chunks-maps',
                'own_calm_chunks_local': 'all-ours-calm-chunks-local'}
BOX = 1.0               # m: the bench's obstacle, a cube (aav_sim_models models/obstacle)
UP_TIME = 150.0         # s for the simulator, the EKF, then navigation, to come up
SETTLE = 5.0            # s after that, before the first goal
READY_TIME = 30.0       # s for the sensor watchdog to let the car drive, before each leg
HELD_TIME = 20.0        # s (sim) the locks hold the car still before --gps-fault gives up a leg
LEG_TIME = 60.0         # s for each leg, plus 4 s for every metre at the top speed
TRACK_EVERY = 0.5       # s between the saved track's points
SERIES_EVERY = 0.05     # s between --series' saved samples
REFUSED_TIME = 60.0     # s (wall) to keep asking a navigator that refuses the first goal
# The test lot's drawing (tools/make_map.py made the map from it): the picture's background.
MAP_SPEC = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'maps',
                        'example_lot.spec.yaml')
COLOURS = {'rpp': '#1f6fb4', 'mppi': '#e8590c', 'pure_pursuit': '#2b8a3e', 'stanley': '#862e9c',
           'lqr': '#c92a2a', 'mpc': '#e67700', 'mpc_full': '#5f3dc4',
           'mpc_lag': '#087f5b', 'mppi_lag': '#0b7285', 'mpc_lag_rs': '#2f9e44',
           # --planners: a column is a planner's name when one follower drives them all.
           'hybrid': '#e8590c', 'lattice': '#1f6fb4', 'smac2d': '#2b8a3e',
           'navfn': '#862e9c', 'theta_star': '#c92a2a', 'lattice32': '#5f3dc4',
           'own': '#0b7285', 'all-ours': '#0b7285', 'own32': '#5f3dc4',
           'all-ours-32': '#5f3dc4', 'all-ours-slow': '#e67700',
           'all-ours-strong': '#c92a2a', 'all-ours-strong-slow': '#e8590c',
           'all-ours-calm': '#2f9e44', 'all-ours-calm-chunks': '#e67700',
           'all-ours-calm-chunks-ll2': '#1f6fb4', 'all-ours-calm-chunks-maps': '#862e9c',
           'all-ours-calm-chunks-local': '#c92a2a'}
# --smoothers: ours is red.
COLOURS.update({f'{k}+{w}-smoother': '#c92a2a'
                for k, c in list(COLOURS.items()) for w in vehicle.SMOOTHERS})
PLAN_TIME = 15.0        # s (wall) for the planner alone to answer, before each leg
# NDT's map of the test lot (tools/ndt_map.py; maps/ is not in git).
NDT_MAP = os.path.join(ROOT, 'maps', 'ndt', 'test_lot.pcd')


# --ndt-map: another map for NDT, such as one map-drive made from a recording (TODO 250).
NDT_MAP_GIVEN = []


def ndt_map(world):
    """Return NDT's map for a tour's world: --ndt-map's, the campus's own, else the lot's."""
    if NDT_MAP_GIVEN:
        return NDT_MAP_GIVEN[0]
    return os.path.join(ROOT, worlds.CAMPUS_NDT) if world == worlds.CAMPUS else NDT_MAP


NDT_CONFIG = os.path.join(ROOT, 'ros', 'localization', 'aav_ndt', 'config', 'ndt.yaml')
# The process to time for each kind of follower (its command line, in the run's group).
# controller:=mppi is mppi_lag since Nav2's controller_server went (17 Nov).
PROCESS = {'controller': 'lib/aav_mppi_lag/mppi_lag', 'tracker': 'lib/aav_track/track',
           # tracker:=mppi_lag runs aav_mppi_lag's Rust node, not aav_track's.
           'mppi_lag': 'lib/aav_mppi_lag/mppi_lag',
           # tracker:=mpc_lag_rs runs aav_mpc_lag's Rust node.
           'mpc_lag_rs': 'lib/aav_mpc_lag/mpc_lag_rs'}


def follower_process(kind, controller):
    """Return the command-line piece of the process to time for this follower."""
    return PROCESS[controller] if controller in ('mppi_lag', 'mpc_lag_rs') else PROCESS[kind]


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cpu_seconds(pid):
    """Return a process's CPU time so far (s), from ps: [D-]H:M:S or M:S.cc; None if gone."""
    if pid is None:
        return None
    out = subprocess.run(['ps', '-o', 'time=', '-p', str(pid)], capture_output=True,
                         text=True).stdout.strip()
    if not out:
        return None
    days, _, clock = out.rpartition('-')
    total = 0.0
    for part in clock.split(':'):
        total = total * 60.0 + float(part)
    return total + (float(days) * 86400.0 if days else 0.0)


# Our own nodes whose CPU and memory each leg's notes give, when they run.
OWN_PROCESSES = ('costmap', 'planner')


def rss_mb(pid):
    """Return a process's resident memory (MB), from ps; 0 if gone."""
    out = subprocess.run(['ps', '-o', 'rss=', '-p', str(pid)], capture_output=True,
                         text=True).stdout.strip()
    return float(out) / 1024.0 if out else 0.0


def follower_pid(pgid, name='lib/aav_mppi_lag/mppi_lag'):
    """Return the pid of the run's follower (mppi_lag, or aav_track's track)."""
    out = subprocess.run(['ps', '-ax', '-o', 'pid=,pgid=,command='], capture_output=True,
                         text=True).stdout
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3 and parts[1] == str(pgid) and name in parts[2]:
            return int(parts[0])
    return None


class Recorder(Node):
    """Where the car really is, what navigation planned and asked for, the safety layers."""

    def __init__(self):
        # On the simulator's clock, like every node in the sim.
        super().__init__('controller_compare', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.recording = False
        self.clear_latches = True       # False (--gps-fault): a person would clear them
        self.watchdog = True            # False: watchdog:=false, nothing publishes its lock
        self.truth, self.plans, self.steering, self.asked = [], [], [], []
        self.wanted = []     # the steering angle ego was asked for (drive_metrics.clipped)
        # The wiggle: the follower's turn rate before the smoother, and after it.
        self.asked_turn, self.smoothed_turn = [], []
        self.wheelbase = float(vehicle.load()['vehicle']['wheelbase'])
        self.belief, self.amcl, self.loops = [], [], []
        self.ndt, self.ndt_status, self.ndt_fits = [], [], []
        self.voxels = []     # (ms a cloud, cubes) from the voxels trial's status
        self.kiss, self.kiss_status, self.local = [], [], []
        self.diffusion = []  # each plan's status, with planner:=diffusion
        self.own_plans = []  # (total ms, cut) from our planner's status (planner:=own)
        self.last_truth = self.glob = None
        self.lock, self.locked, self.states = None, False, set()
        self.monitor, self.monitor_stops = None, 0
        self.fence = set()   # the geofence's holds while driving (NEAR, OUTSIDE, ...)
        self.fence_now = None   # the geofence's state, now
        # pose_source:=ndt (TODO 210): pose_check's sources seen while driving, its last
        # message, its hand-overs and seeds.
        self.pose_check = {}
        self.create_subscription(DiagnosticStatus, '/localization/pose_check/status',
                                 self.on_pose_check, 10)
        self.create_subscription(Odometry, '/vehicle/sim/truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/localization/odom/global', self.on_global, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/amcl/pose',
                                 self.on_amcl, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/ndt/pose',
                                 self.on_ndt, 10)
        self.create_subscription(DiagnosticStatus, '/localization/ndt/status',
                                 self.on_ndt_status, 10)
        self.create_subscription(DiagnosticStatus, '/perception/voxels/status',
                                 self.on_voxels, 10)
        self.create_subscription(Odometry, '/localization/kiss_icp/odom',
                                 lambda m: self.on_odometry(self.kiss, m), 10)
        self.create_subscription(Odometry, '/localization/odom/local',
                                 lambda m: self.on_odometry(self.local, m), 10)
        self.create_subscription(DiagnosticStatus, '/localization/kiss_icp/status',
                                 self.on_kiss_status, 10)
        self.create_subscription(DiagnosticStatus, '/drivers/nav/track/status',
                                 self.on_loop, 10)
        self.create_subscription(DiagnosticStatus, '/drivers/nav/diffusion/status',
                                 self.on_diffusion, 10)
        self.create_subscription(DiagnosticStatus, '/drivers/nav/planner/status',
                                 self.on_own_plan, 10)
        self.create_subscription(AckermannDriveStamped, '/ego/actuator_cmd', self.on_steering,
                                 10)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self.on_asked, 10)
        self.create_subscription(TwistStamped, '/control/cmd_vel', self.on_wanted, 10)
        self.create_subscription(TwistStamped, '/drivers/nav/cmd_vel', self.on_smoothed, 10)
        self.create_subscription(Bool, '/safety/sensor_watchdog/lock', self.on_lock, 10)
        self.create_subscription(String, '/safety/sensor_watchdog/state', self.on_state, 10)
        self.create_subscription(CollisionMonitorState, '/safety/stop_zone/state',
                                 self.on_monitor, 10)
        self.create_subscription(String, '/safety/geofence/state', self.on_fence, 10)
        # Our navigator serves Nav2's action names (Nav2's own went, 17 Nov).
        self.nav = ActionClient(self, NavigateToPose,
                                vehicle.navigator_action('navigate_to_pose'))
        self.gps = ActionClient(self, FollowGPSWaypoints,
                                vehicle.navigator_action('follow_gps_waypoints'))
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        # planner:=own: ours (aav_planner).
        self.own_planner = ActionClient(self, ComputePathToPose, vehicle.OWN_PLANNER_ACTION)
        self.clear = self.create_client(Trigger, '/safety/sensor_watchdog/clear')

    def start(self):
        self.truth, self.plans, self.steering, self.asked = [], [], [], []
        self.wanted, self.asked_turn, self.smoothed_turn = [], [], []
        self.belief, self.amcl, self.loops = [], [], []
        self.ndt, self.ndt_status, self.ndt_fits = [], [], []
        self.voxels = []
        self.kiss, self.kiss_status, self.local = [], [], []
        self.diffusion = []
        self.own_plans = []
        self.locked, self.states, self.monitor_stops = False, set(), 0
        self.fence = set()
        self.pose_check = {}
        self.recording = True

    def on_truth(self, msg):
        self.last_truth = msg
        if self.recording:
            p = msg.pose.pose.position
            self.truth.append((seconds(msg.header.stamp), p.x, p.y,
                               yaw_of(msg.pose.pose.orientation)))

    def on_global(self, msg):
        self.glob = msg
        if self.recording:
            p = msg.pose.pose.position
            self.belief.append((seconds(msg.header.stamp), p.x, p.y,
                                yaw_of(msg.pose.pose.orientation)))

    def on_amcl(self, msg):
        if self.recording:
            p = msg.pose.pose.position
            self.amcl.append((seconds(msg.header.stamp), p.x, p.y,
                              yaw_of(msg.pose.pose.orientation)))

    def on_ndt(self, msg):
        if self.recording:
            p = msg.pose.pose.position
            self.ndt.append((seconds(msg.header.stamp), p.x, p.y,
                             yaw_of(msg.pose.pose.orientation)))

    def on_ndt_status(self, msg):
        if self.recording:
            v = {kv.key: kv.value for kv in msg.values}
            try:
                self.ndt_status.append((self.now(), msg.level == DiagnosticStatus.OK,
                                        float(v['match_ms']), int(v['restarts'])))
            except (KeyError, ValueError):
                pass
            # Every fit, good or poor, at its cloud's stamp: for NEES and the score floor.
            try:
                self.ndt_fits.append((
                    float(v['stamp']), msg.level == DiagnosticStatus.OK,
                    float(v['score_per_point']), float(v['x']), float(v['y']), float(v['yaw']),
                    tuple(float(v[f'cov_{n}'])
                          for n in ('xx', 'xy', 'xyaw', 'yy', 'yyaw', 'yawyaw'))))
            except (KeyError, ValueError):
                pass

    def on_voxels(self, msg):
        if self.recording:
            v = {kv.key: kv.value for kv in msg.values}
            try:
                self.voxels.append((float(v['ms']), int(v['cubes'])))
            except (KeyError, ValueError):
                pass

    def on_own_plan(self, msg):
        if self.recording:
            v = {kv.key: kv.value for kv in msg.values}
            try:
                self.own_plans.append((float(v['total_ms']), v.get('cut') == 'true'))
            except (KeyError, ValueError):
                pass

    def on_odometry(self, samples, msg):
        if self.recording:
            p = msg.pose.pose.position
            samples.append((seconds(msg.header.stamp), p.x, p.y,
                            yaw_of(msg.pose.pose.orientation)))

    def on_kiss_status(self, msg):
        if self.recording:
            v = {kv.key: kv.value for kv in msg.values}
            try:
                self.kiss_status.append((self.now(), float(v['match_ms']),
                                         int(v['clouds_dropped'])))
            except (KeyError, ValueError):
                pass

    def on_diffusion(self, msg):
        if self.recording:
            self.diffusion.append({kv.key: kv.value for kv in msg.values})

    def on_loop(self, msg):
        if self.recording:
            v = {kv.key: kv.value for kv in msg.values}
            try:
                self.loops.append((self.now(), float(v['loop_ms']), float(v['loop_ms_max']),
                                   int(v['missed_ticks'])))
            except (KeyError, ValueError):
                pass

    def on_plan(self, msg):
        if self.recording:
            self.plans.append((self.now(), [(s.pose.position.x, s.pose.position.y)
                                            for s in msg.poses]))

    def on_steering(self, msg):
        if self.recording:
            self.steering.append((seconds(msg.header.stamp), msg.drive.steering_angle))

    def on_asked(self, msg):
        if self.recording:
            self.asked.append((self.now(), msg.twist.linear.x))
            self.asked_turn.append((self.now(), msg.twist.angular.z))

    def on_smoothed(self, msg):
        if self.recording:
            self.smoothed_turn.append((self.now(), msg.twist.angular.z))

    def on_wanted(self, msg):
        v, w = msg.twist.linear.x, msg.twist.angular.z
        if self.recording and abs(v) >= drive_metrics.STILL:
            self.wanted.append((self.now(), math.atan(self.wheelbase * w / v)))

    def on_lock(self, msg):
        self.lock = msg.data
        if self.recording and msg.data:
            self.locked = True

    def on_state(self, msg):
        if self.recording and msg.data not in ('', 'OK'):
            self.states.add(msg.data[:90])

    def on_pose_check(self, msg):
        if not self.recording:
            return
        values = {kv.key: kv.value for kv in msg.values}
        self.pose_check.setdefault('sources', set()).add(values.get('source', '?'))
        self.pose_check['last'] = msg.message
        if msg.level >= DiagnosticStatus.ERROR:
            self.pose_check['hold'] = msg.message
        for key in ('handovers', 'seeds'):
            self.pose_check.setdefault(f'{key}0', values.get(key, '0'))
            self.pose_check[key] = values.get(key, '0')

    def on_fence(self, msg):
        # 'STATE: why'. STARTING before the first leg isn't a hold while driving.
        state = msg.data.split(':', 1)[0]
        self.fence_now = state
        if self.recording and state in ('NEAR', 'OUTSIDE', 'LOST', 'JUMP', 'COASTING'):
            self.fence.add(state)

    def on_monitor(self, msg):
        if self.recording and msg.action_type == CollisionMonitorState.STOP and (
                self.monitor != CollisionMonitorState.STOP):
            self.monitor_stops += 1
        self.monitor = msg.action_type

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, wall_seconds, until=None):
        end = time.monotonic() + wall_seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None

    def wait(self, future, wall_seconds):
        self.spin_for(wall_seconds, future.done)
        return future.result() if future.done() else None

    def send_route(self, waypoints):
        """Send the navigator GPS waypoints; return its handle, or None if refused."""
        goal = FollowGPSWaypoints.Goal()
        for w in waypoints:
            pose = GeoPose()
            pose.position.latitude, pose.position.longitude = w.lat, w.lon
            pose.position.altitude = 0.0
            pose.orientation.z = math.sin(w.yaw / 2.0)
            pose.orientation.w = math.cos(w.yaw / 2.0)
            goal.gps_poses.append(pose)
        if not self.gps.wait_for_server(timeout_sec=10.0):
            return None
        handle = self.wait(self.gps.send_goal_async(goal), 10.0)
        return handle if handle is not None and handle.accepted else None

    def on_map(self, lat, lon):
        """Return (x, y) of a lat/lon on the map, from navsat_transform's /fromLL; None."""
        if not self.from_ll.wait_for_service(timeout_sec=10.0):
            return None
        request = FromLL.Request()
        request.ll_point.latitude, request.ll_point.longitude = lat, lon
        answer = self.wait(self.from_ll.call_async(request), 10.0)
        return None if answer is None else (answer.map_point.x, answer.map_point.y)

    def send(self, x, y, yaw):
        """Send the navigator a goal in the map frame; return its handle, or None."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # The action server can answer before the navigator takes goals (its planner and
        # follower not there yet). Ask again for up to REFUSED_TIME.
        end = time.monotonic() + REFUSED_TIME
        while True:
            handle = self.wait(self.nav.send_goal_async(goal), 10.0)
            if handle is not None and handle.accepted:
                return handle
            if time.monotonic() > end:
                return None
            self.spin_for(2.0)


def plan_alone(rec, goal, lock_radius, client=None, name='/compute_path_to_pose'):
    """
    Ask the planner alone for a path from where the car stands; return (planned, notes).

    planned is drive_metrics' leg['planned']: the path, the planner's own time, the call's
    wall time, and the car's full-lock radius. Nothing moves. client, name: whose action
    (None: /compute_path_to_pose, the diffusion planner's).
    """
    client = client or rec.planner
    x, y, yaw = goal
    request = ComputePathToPose.Goal()
    request.goal.header.frame_id = 'map'
    request.goal.pose.position.x, request.goal.pose.position.y = x, y
    request.goal.pose.orientation.z = math.sin(yaw / 2.0)
    request.goal.pose.orientation.w = math.cos(yaw / 2.0)
    request.planner_id = 'GridBased'      # whichever planner:= runs behind it
    request.use_start = False             # from the car, through TF, as the navigator asks
    out = {'path': [], 'ms': None, 'wall_ms': None, 'lock_radius': lock_radius}
    if not client.wait_for_server(timeout_sec=5.0):
        return out, [f'the planner alone: no {name}']
    wall0 = time.monotonic()
    handle = rec.wait(client.send_goal_async(request), 5.0)
    if handle is None or not handle.accepted:
        return out, ['the planner alone: refused']
    done = rec.wait(handle.get_result_async(), PLAN_TIME)
    out['wall_ms'] = 1000.0 * (time.monotonic() - wall0)
    if done is None:
        rec.wait(handle.cancel_goal_async(), 5.0)
        return out, [f'the planner alone: no answer in {PLAN_TIME:.0f} s']
    result = done.result
    took = result.planning_time
    out['ms'] = took.sec * 1000.0 + took.nanosec * 1e-6
    out['path'] = [(s.pose.position.x, s.pose.position.y) for s in result.path.poses]
    if done.status != GoalStatus.STATUS_SUCCEEDED or not out['path']:
        why = getattr(result, 'error_msg', '') or f'error {getattr(result, "error_code", "?")}'
        return out, [f'the planner alone found no path: {why}']
    return out, []


def ready(rec):
    """Wait until the sensor watchdog lets the car drive; clear a latch once. Return notes."""
    if not rec.watchdog:
        return []                       # watchdog:=false (--gps-off): no lock to wait for
    if rec.spin_for(READY_TIME, lambda: rec.lock is False):
        return []
    if not rec.clear_latches:
        # --gps-fault: a person would clear it, so nobody does.
        return ['the sensor watchdog held the car before the goal (not cleared)']
    # Still locked: a latch (a GPS jump, the wheels against the IMU) waits for a person.
    rec.wait(rec.clear.call_async(Trigger.Request()), 5.0)
    if rec.spin_for(READY_TIME, lambda: rec.lock is False):
        return ['the sensor watchdog was latched before the goal: cleared it']
    return ['the sensor watchdog still held the car when the goal went']


def drive_leg(rec, pgid, goal, speed, body, obstacle, process='lib/aav_mppi_lag/mppi_lag',
              amcl=False, ndt=False, lock_radius=None, kiss=False, own_planner=False,
              route=None):
    """
    Send one goal and record it; return (drive_metrics leg, notes).

    lock_radius (m, --planners): first ask the planner alone for the leg's path, and score
    its turns against the car's full lock. own_planner (planner:=own): ask ours (else the
    diffusion planner's /compute_path_to_pose). route: GPS
    waypoints (a tour's 'route'), sent to follow_gps_waypoints instead of one goal.
    """
    notes = ready(rec)
    planned = None
    if lock_radius is not None and own_planner:
        planned, more = plan_alone(rec, goal, lock_radius, rec.own_planner,
                                   vehicle.OWN_PLANNER_ACTION)
        notes += more
    elif lock_radius is not None:
        planned, more = plan_alone(rec, goal, lock_radius)
        notes += more
    x, y, yaw = goal
    here = rec.last_truth.pose.pose.position
    timeout = LEG_TIME + 4.0 * math.hypot(x - here.x, y - here.y) / speed
    pid = follower_pid(pgid, process)
    cpu0, wall0 = cpu_seconds(pid), time.monotonic()
    # Our costmap's and planner's own CPU and memory (TODO 175: the chunks against /map).
    ours = {name: follower_pid(pgid, f'lib/aav_{name}/{name}') for name in OWN_PROCESSES}
    ours_cpu0 = {name: cpu_seconds(p) for name, p in ours.items()}
    rec.start()
    start = rec.now()
    reached = False
    handle = rec.send_route(route) if route else rec.send(x, y, yaw)
    if handle is None:
        notes.append('the navigator refused the goal')
    else:
        result = handle.get_result_async()
        # The simulator's time for the limit, and twice that in wall time in case it stalls.
        end_wall = time.monotonic() + 2.0 * timeout + 30.0
        still_since = None
        held = False
        while not result.done() and rec.now() < start + timeout and (
                time.monotonic() < end_wall):
            rclpy.spin_once(rec, timeout_sec=0.05)
            if not rec.clear_latches:
                # --gps-fault: a car the locks hold still for HELD_TIME has stopped.
                v = rec.last_truth.twist.twist.linear
                locks = rec.lock or rec.fence_now not in (None, 'OK')
                if math.hypot(v.x, v.y) < 0.05 and locks:
                    still_since = rec.now() if still_since is None else still_since
                    if rec.now() - still_since > HELD_TIME:
                        held = True
                        break
                else:
                    still_since = None
        if held:
            rec.wait(handle.cancel_goal_async(), 5.0)
            notes.append(f'held still by the locks for {HELD_TIME:.0f} s: cancelled')
        elif result.done():
            done = result.result()
            reached = done.status == GoalStatus.STATUS_SUCCEEDED
            missed = list(getattr(done.result, 'missed_waypoints', []))
            if missed:
                # waypoint_follower succeeds with waypoints missed: not there.
                notes.append(f'{len(missed)} of {len(route)} waypoints missed '
                             f'({", ".join(str(m.index) for m in missed[:8])})')
                reached = reached and done.result.missed_waypoints[-1].index != len(route) - 1
            if not reached:
                why = getattr(done.result, 'error_msg', '') or f'status {done.status}'
                notes.append(f'the navigator gave up: {why}')
        else:
            rec.wait(handle.cancel_goal_async(), 5.0)
            notes.append(f'not there in {timeout:.0f} s: cancelled')
    end = rec.now()
    rec.recording = False
    cpu1, wall1 = cpu_seconds(pid), time.monotonic()
    for name, p in ours.items():
        c0, c1 = ours_cpu0[name], cpu_seconds(p)
        if None not in (c0, c1) and wall1 > wall0:
            notes.append(f'{name}: CPU {100.0 * (c1 - c0) / (wall1 - wall0):.0f} %, '
                         f'{rss_mb(p):.0f} MB')
    if rec.own_plans:
        ms = sorted(m for m, _ in rec.own_plans)
        cut = sum(1 for _, c in rec.own_plans if c)
        notes.append(f'planner {ms[len(ms) // 2]:.0f} ms a plan median, {ms[-1]:.0f} worst, '
                     f'{len(ms)} plans, {cut} cut at the costmap edge')
    if rec.locked:
        notes.append('the sensor watchdog stopped the car' +
                     (': ' + '; '.join(sorted(rec.states)) if rec.states else ''))
    if rec.monitor_stops:
        notes.append(f'the stop zone stopped the car {rec.monitor_stops}x')
    if rec.fence:
        notes.append('the geofence held the car: ' + ', '.join(sorted(rec.fence)))
    if rec.pose_check:
        pc = rec.pose_check
        handovers = int(pc.get('handovers', 0)) - int(pc.get('handovers0', 0))
        seeds = int(pc.get('seeds', 0)) - int(pc.get('seeds0', 0))
        notes.append(f'pose_check: {"/".join(sorted(pc.get("sources", [])))}, '
                     f'{handovers} hand-overs, {seeds} seeds; last "{pc.get("last", "")}"' +
                     (f'; HOLD "{pc["hold"]}"' if 'hold' in pc else ''))
    if rec.diffusion:
        notes.append(diffusion_note(rec.diffusion))
    if rec.voxels:
        ms = sorted(m for m, _ in rec.voxels)
        cubes = [c for _, c in rec.voxels]
        notes.append(f'voxels {sum(ms) / len(ms):.1f} ms a cloud mean, '
                     f'{ms[int(0.95 * (len(ms) - 1))]:.1f} 95th, {ms[-1]:.1f} worst; '
                     f'{sum(cubes) / len(cubes):.0f} cubes a cloud, {len(ms)} clouds')
    leg = {'truth': rec.truth, 'belief': rec.belief, 'plans': rec.plans,
           'steering': rec.steering, 'wanted': rec.wanted,
           'asked': rec.asked, 'asked_turn': rec.asked_turn,
           'smoothed_turn': rec.smoothed_turn, 'start': start, 'end': end, 'goal': goal,
           'reached': reached,
           'cpu': (100.0 * (cpu1 - cpu0) / (wall1 - wall0)
                   if None not in (cpu0, cpu1) and wall1 > wall0 else None),
           'obstacle': obstacle, 'body': body}
    if amcl:
        leg['amcl'] = rec.amcl
    if ndt:
        leg['ndt'], leg['ndt_status'] = rec.ndt, rec.ndt_status
        leg['ndt_fits'], leg['ndt_cov'] = rec.ndt_fits, ndt_cov()
    if kiss:
        leg['kiss'], leg['kiss_status'], leg['local'] = rec.kiss, rec.kiss_status, rec.local
    if rec.loops:
        leg['loops'] = rec.loops
    if rec.diffusion:
        leg['diffusion'] = rec.diffusion
    if planned is not None:
        leg['planned'] = planned
    return leg, notes


def diffusion_note(plans):
    """Return a leg's note from diffusion_planner's statuses: whose plans, why, how fast."""
    model = sum(p.get('source') == 'model' for p in plans)
    late = sum(p.get('source') != 'model' and 'late' in p.get('why', '') for p in plans)
    refused = {c: sum(int(p.get(f'refused_{c}', 0)) for p in plans)
               for c in ('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7', 'G8')}
    samples = sum(int(p.get('samples', 0)) for p in plans)
    passed = sum(int(p.get('samples_passed', 0)) for p in plans)
    ms = [float(p['model_and_guard_ms']) for p in plans if 'model_and_guard_ms' in p]
    return (f'diffusion: {model} of {len(plans)} plans from the model ({model / len(plans):.0%})'
            f", {len(plans) - model} the lattice's ({late} late); samples passed {passed} of "
            f'{samples}; refused ' + ', '.join(f'{c} {n}' for c, n in refused.items() if n) +
            (f'; model and guard {sum(ms) / len(ms):.0f} ms mean, {max(ms):.0f} worst'
             if ms else ''))


def ndt_cov():
    """Return NDT's (k, floor_xy, floor_yaw) from aav_ndt's config/ndt.yaml."""
    with open(NDT_CONFIG) as f:
        p = yaml.safe_load(f)['ndt']['ros__parameters']
    return (p['covariance_scale'], p['variance_floor_xy'], p['variance_floor_yaw'])


def ndt_calibration(results):
    """
    Return results.md's NDT section: its spread against its errors over every leg.

    The k that would make the good fits' mean NEES 3 (with the floors kept), the NEES
    with the k in use, and how far off each band of score per point was (good and poor).
    """
    errors = [e for *_, drawn in results for e in drawn.get('ndt_errors', [])]
    if not errors:
        return []
    k, floor_xy, floor_yaw = ndt_cov()
    fitted = drive_metrics.fit_scale(errors, floor_xy, floor_yaw)
    lines = ['## NDT: its spread against its errors', '',
             f'{sum(1 for ok, *_ in errors if ok)} good fits, '
             f'{sum(1 for ok, *_ in errors if not ok)} poor, over every leg and run. '
             'NEES: about 3 when the covariance is honest.', '',
             '| | |', '|---|---|',
             f'| k in use (config/ndt.yaml) | {k:g} |',
             f'| NEES, mean, with it | '
             f'{drive_metrics.mean_nees(errors, k, floor_xy, floor_yaw):.1f} |',
             f'| k for NEES 3 (floors kept) | {fitted:.3g} |', '',
             '| Score per point | Fits | Off, mean (m) | Off, worst (m) | Heading, worst (deg) |',
             '|---|---|---|---|---|']
    lines += [f'| {lo:g} to {hi:g} | {n} | {mean:.3f} | {worst:.2f} | {deg:.1f} |'
              for lo, hi, n, mean, worst, deg in drive_metrics.score_bands(errors)]
    return lines + ['']


def gps_off(rec, mode='stop'):
    """Set the GPS's fault (aav_sim_models faults: sim_gps fault.gps); return a note."""
    out = subprocess.run(['ros2', 'param', 'set', '/sim_gps', 'fault.gps', mode],
                         capture_output=True, text=True, timeout=60)
    rec.spin_for(2.0)
    return (f'the GPS fault on purpose: {mode}'
            if out.returncode == 0 and 'successful' in out.stdout.lower()
            else f'could not set the GPS to {mode}: {(out.stdout + out.stderr).strip()[:120]}')


def private(used):
    """
    Pick an unused ROS domain, and set it, before rclpy.init.

    The domain: AAV_SIM_DOMAIN's for the first run, the next ones up from it for the
    later runs (never above 232: Fast DDS breaks there), else one from 150-230.
    """
    first = os.environ.get('AAV_SIM_DOMAIN')
    above = [str(d) for d in range(int(first), 233) if str(d) not in used] if first else []
    if above:
        domain = above[0]
    else:
        domain = str(random.choice([d for d in range(150, 231) if str(d) not in used]))
    used.add(domain)
    os.environ['ROS_DOMAIN_ID'] = domain
    return domain


def alive(pgid):
    """Return True while a process group has a process that isn't a zombie (ps, any OS)."""
    out = subprocess.run(['ps', '-ax', '-o', 'pgid=,stat='], capture_output=True,
                         text=True).stdout
    return any(f[0] == str(pgid) and not f[1].startswith('Z')
               for f in (line.split() for line in out.splitlines()) if len(f) == 2)


def stop(launch, grace=30.0):
    """Stop a run's whole process group: Ctrl-C, then kill what's left; wait until it's gone."""
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGKILL, 10.0)):
        try:
            os.killpg(launch.pid, sig)
        except (ProcessLookupError, PermissionError):   # gone, or only zombies (macOS)
            pass
        end = time.monotonic() + wait
        while time.monotonic() < end:
            launch.poll()               # reap the launcher, or it stays a zombie
            if not alive(launch.pid):
                return True
            time.sleep(0.5)
    return False


def campus_route(start, end):
    """
    Return (waypoints, the last one): the campus graph's route between two places.

    Cut into GPS waypoints as street_router does (aav_routes; maps/carla/carleton.osm).
    """
    from aav_routes import campus_graph, osrm_route
    site = os.path.join(ROOT, 'ros', 'drivers', 'aav_routes', 'config', 'campus_site.yaml')
    campus = campus_graph.Campus(site, ROOT)
    route = campus.route(campus_graph.target(campus.find(start)),
                         campus_graph.target(campus.find(end)))
    waypoints = osrm_route.waypoints(route.points)
    return waypoints, waypoints[-1]


def place_box(name, x, y):
    """Put the bench's 1 m box at (x, y) by our simulator's spawn_entity: what it said."""
    model = os.path.join(ROOT, 'ros', 'sim', 'aav_sim_models', 'models', 'obstacle', 'model.sdf')
    try:
        done = subprocess.run(
            ['ros2', 'service', 'call', '/vehicle/sim/spawn_entity',
             'simulation_interfaces/srv/SpawnEntity',
             f'{{name: "{name}", uri: "{model}", initial_pose: {{pose: {{position: '
             f'{{x: {x}, y: {y}, z: 0.0}}}}}}}}'],
            capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return 'no answer in 60 s'
    return 'placed' if 'result=1' in done.stdout else ' '.join(done.stdout.split())[-200:]


def tour_legs(tour, args):
    """Return the tour's legs to drive: all, or only the first --legs."""
    return TOURS[tour]['legs'][:args.legs]


def followers(args):
    """
    Return [(kind, name, planner, label)]: each follower with each planner, as asked.

    --stacks: [('stack', 'own'..., None, 'all-ours'...)] instead.

    --controllers (mppi: mppi_lag since Nav2 went), then the trackers; none of them:
    mppi_lag alone. Without --planners, planner is None (the launch's own default) and the
    label is the follower's name, as before.
    """
    if args.smoothers:
        # Each follower behind each smoother: a fifth item, and the label says which.
        plain = followers(argparse.Namespace(**{**vars(args), 'smoothers': None}))
        return [(*f, word, f'{f[3]}+{word}-smoother') for f in plain
                for word in args.smoothers]
    if args.stacks:
        return [('stack', word, None, STACK_LABELS[word]) for word in args.stacks]
    controllers = args.controllers
    if controllers is None:
        controllers = []
    chosen = ([('controller', c) for c in controllers] +
              [('tracker', t) for t in args.trackers or []]) or [('tracker', 'mppi_lag')]
    if not args.planners:
        return [(kind, name, None, name) for kind, name in chosen]
    return [(kind, name, planner, planner if len(chosen) == 1 else f'{name}+{planner}')
            for kind, name in chosen for planner in args.planners]


# --planners words beyond vehicle.PLANNERS: the same planner on another of its files.
# own32: our State Lattice on its 32-heading trial file (lattice_headings:=32).
PLANNER_VARIANTS = {f'own{h}': ('own', [f'lattice_headings:={h}'])
                    for h in vehicle.LATTICE_HEADINGS[1:]}
# own_smooth (and _smooth_fast): ours on the smooth moves (lattice_moves:=,
# tools/lattice_smooth.py).
PLANNER_VARIANTS.update({f'own_{m}': ('own', [f'lattice_moves:={m}'])
                         for m in vehicle.LATTICE_MOVES[1:]})
# own_hybrid: our planner running Hybrid-A* (planner_search:=hybrid; the league's
# hybrid_rs), against own, the lattice (lattice_rs).
PLANNER_VARIANTS['own_hybrid'] = ('own', ['planner_search:=hybrid'])
# diffusion_nomodel: the diffusion trial with no model file, so every plan must be the
# lattice's (its guard's fallback test: docs/tests.md).
PLANNER_VARIANTS['diffusion_nomodel'] = ('diffusion', ['diffusion_model:=/nonexistent.onnx'])
# diffusion_rs: the same trial, its model run from Rust (aav_diffusion_rs's worker, ONNX
# Runtime; Core ML on a Mac: auto). diffusion_coreml: the Python worker on Core ML.
# The node, its guard and the lattice's fallback are the same (TODO 243, docs/onnx.md).
PLANNER_VARIANTS['diffusion_rs'] = ('diffusion', ['diffusion_impl:=rust',
                                                  'diffusion_provider:=auto'])
PLANNER_VARIANTS['diffusion_coreml'] = ('diffusion',
                                        ['diffusion_provider:=CoreMLExecutionProvider'])
PLANNER_WORDS = vehicle.PLANNERS + tuple(PLANNER_VARIANTS)


def base_planner(word):
    """Return the planner:= behind a --planners word (own for own_hybrid, own_smooth)."""
    return PLANNER_VARIANTS.get(word, (word, []))[0]


def planner_launch(word):
    """Return sim.launch.py's arguments for a --planners word: planner:= and any others."""
    planner, extra = PLANNER_VARIANTS.get(word, (word, []))
    return [f'planner:={planner}'] + extra


def planner_names(text):
    """Return the planners in a --planners word: comma-separated, or one."""
    names = [n.strip().lower() for n in text.split(',') if n.strip()]
    for name in names:
        if name not in PLANNER_WORDS:
            raise argparse.ArgumentTypeError(f'{name}: pick from {", ".join(PLANNER_WORDS)}')
    return names


def tracker_names(text):
    """Return the trackers in a --trackers word: comma-separated, or one."""
    names = [n.strip().lower() for n in text.split(',') if n.strip()]
    for name in names:
        if name not in vehicle.TRACKERS:
            raise argparse.ArgumentTypeError(f'{name}: pick from {", ".join(vehicle.TRACKERS)}')
    return names


def session(tour, follower, args, out_dir, body, used, run=1):
    """Start the simulator for one tour and follower, drive its legs, stop it."""
    plan = TOURS[tour]
    kind, controller, planner = follower[:3]
    # --smoothers: (..., smoother, label); otherwise (..., label).
    label = follower[-1]
    smoother = follower[4] if len(follower) > 4 else None
    extra = []
    if kind == 'stack':
        kind, controller, extra = STACKS[follower[1]]
    tag = f'{label}-{run}' if args.runs > 1 else label
    log_path = os.path.join(out_dir, f'sim-{tour}-{tag}.log')
    command = ['ros2', 'launch', 'aav_bringup', 'sim.launch.py',
               f'world:={plan.get("world", "test_lot")}',
               'rviz:=false',
               # Every run on the champions' stack (best), then the piece under test on
               # top: each column differs from the next only in what it names. Until 17 Nov
               # this was Nav2's stock stack (docs/league-results-2026-11-16.md).
               'stack:=best',
               f'tracker:={controller}' if kind == 'tracker' else f'controller:={controller}',
               # Forward only (MPPI dithered between forward and back at the start,
               # 22 Oct), unless --reversing: the car may then back out of a dead end.
               f'reversing:={str(args.reversing).lower()}']
    if planner:
        command += planner_launch(planner)
        if planner.startswith('diffusion'):
            if args.diffusion_model and planner != 'diffusion_nomodel':
                command.append(f'diffusion_model:={os.path.abspath(args.diffusion_model)}')
            if args.diffusion_smoothing:
                command.append(f'diffusion_smoothing:={args.diffusion_smoothing}')
            if args.diffusion_record:
                # Each plan's inputs, a folder a run, to replay offline (training/replay.py).
                command.append('diffusion_record:=' + os.path.join(
                    os.path.abspath(args.diffusion_record), f'{tour}-{tag}'))
    command += extra + plan.get('launch', []) + (args.launch or [])
    if smoother:
        command.append(f'smoother:={smoother}')
    if args.amcl:
        command.append('amcl:=true')
    if args.ndt_leads:
        command += ['pose_source:=ndt', f'ndt_map:={ndt_map(plan.get("world"))}']
    elif args.ndt_ekf:
        command += ['ndt_ekf:=true', f'ndt_map:={ndt_map(plan.get("world"))}']
    elif args.ndt:
        command += ['ndt:=true', f'ndt_map:={ndt_map(plan.get("world"))}']
    if args.gps_off:
        # The sensor watchdog and the geofence (LOST after 2 s without a GPS position)
        # would hold the car with the GPS quiet: the drive is the test.
        command += ['watchdog:=false', 'geofence:=false']
    if args.voxels:
        command.append('voxels:=true')
    if args.kiss_ekf:
        command.append('kiss_ekf:=true')
    elif args.kiss_icp:
        command.append('kiss_icp:=true')
    if plan['obstacle']:
        command.append('obstacle:=true')
    if args.speed:
        command.append(f'sim_speed:={args.speed:g}')
    if args.lag_scale != 1.0:
        # Only the sim's adapter scales the car's lags (sim.launch.py refuses it otherwise).
        command += [f'lag_scale:={args.lag_scale:g}']
    domain = private(used)
    print(f'\n== {tour}, {label}' + (f' (run {run} of {args.runs})' if args.runs > 1
                                     else '') +
          f': starting the test lot on ROS domain {domain} '
          f'(this computer only)...', flush=True)
    rows = []
    obstacle = (*worlds.OBSTACLE, BOX) if plan['obstacle'] else None
    if plan.get('boxes'):
        obstacle = (*plan['boxes'][0], BOX)
    # The car's full lock: tighter than this, a planned turn can't be driven as drawn.
    lock = vehicle.min_turning_radius(vehicle.load()['vehicle'])
    with open(log_path, 'w') as log:
        launch = subprocess.Popen(command, cwd=ROOT, env=dict(os.environ), stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        rec = Recorder()
        rec.watchdog = not args.gps_off
        rec.clear_latches = not args.gps_fault
        try:
            up = rec.spin_for(UP_TIME, lambda: rec.last_truth is not None and rec.glob)
            server = rec.gps if plan.get('route') else rec.nav
            if not (up and rec.spin_for(UP_TIME, server.server_is_ready)):
                print(f'   it never came up: see {log_path}', flush=True)
                return [(tour, name, label, None, ['never came up'], {'run': run})
                        for name, *_ in tour_legs(tour, args)]
            rec.spin_for(SETTLE)
            for k, (bx, by) in enumerate(plan.get('boxes', [])):
                said = place_box(f'parked_box_{k}', bx, by)
                print(f'   a 1 m box at ({bx:.1f}, {by:.1f}): {said}', flush=True)
            if plan.get('boxes'):
                rec.spin_for(3.0)          # the lidar sees it before the first goal
            if args.gps_off:
                # After the EKF's first fix (the wait above): it knows where it started.
                print(f'   {gps_off(rec)}', flush=True)
            elif args.gps_fault:
                print(f'   {gps_off(rec, args.gps_fault)}', flush=True)
            route = None
            if plan.get('route'):
                route, end = campus_route(*plan['route'])
                goal_xy = rec.on_map(end.lat, end.lon)
                if goal_xy is None:
                    print('   no /fromLL: the route has no goal on the map', flush=True)
                    return [(tour, name, label, None, ['no /fromLL'], {'run': run})
                            for name, *_ in tour_legs(tour, args)]
            for name, x, y, heading in tour_legs(tour, args):
                if route:
                    (x, y), heading = goal_xy, math.degrees(route[-1].yaw)
                leg, notes = drive_leg(rec, launch.pid, (x, y, math.radians(heading)),
                                       args.speed or 1.0, body, obstacle,
                                       follower_process(kind, controller), args.amcl,
                                       args.ndt,
                                       lock if planner else None, args.kiss_icp,
                                       base_planner(planner) == 'own', route)
                row = drive_metrics.summary(leg)
                # t, x, y, yaw: where the car was, and where the global EKF put it.
                drawn = {'run': run, 'goal': [round(x, 2), round(y, 2)],
                         'track': [[round(v, 3) for v in s]
                                   for s in drive_metrics.thin(leg['truth'], TRACK_EVERY)],
                         'belief': [[round(v, 3) for v in s]
                                    for s in drive_metrics.thin(leg['belief'], TRACK_EVERY)],
                         # t, speed: what the controller asked for, before the smoother.
                         'asked': [[round(v, 3) for v in s]
                                   for s in drive_metrics.thin(leg['asked'], TRACK_EVERY)],
                         'plan': [[round(x, 2), round(y, 2)]
                                  for x, y in (leg['plans'][0][1][::4] if leg['plans'] else [])]}
                if args.kiss_icp:
                    # The odometries, for the whole tour's drift (report()).
                    for key in ('kiss', 'local'):
                        drawn[key] = [[round(v, 3) for v in s]
                                      for s in drive_metrics.thin(leg[key], TRACK_EVERY)]
                if args.plans:
                    # Every plan of the leg, each with its time: where the planner sent the
                    # car as it went (TODO 17: a goal circled after a replan).
                    drawn['plans'] = [[round(t, 2), [[round(x, 2), round(y, 2)]
                                                     for x, y in pts[::4] + pts[-1:]]]
                                      for t, pts in leg['plans']]
                if args.series:
                    # The wiggle's raw series, every 0.05 s: to look at it again offline.
                    drawn['series'] = {
                        key: [[round(v, 4) for v in s]
                              for s in drive_metrics.thin(leg[key], SERIES_EVERY)]
                        for key in ('truth', 'belief', 'steering', 'asked_turn',
                                    'smoothed_turn')}
                if leg.get('planned', {}).get('path'):
                    # The planner's own path, from where the car stood (--planners): every
                    # point, so its turns and cusps can be scored again from the file.
                    drawn['planned'] = [[round(x, 3), round(y, 3)]
                                        for x, y in leg['planned']['path']]
                if leg.get('diffusion'):
                    drawn['diffusion'] = leg['diffusion']   # each plan's status
                if leg.get('ndt_fits'):
                    # Each fit's error at its own stamp, for ndt_calibration().
                    drawn['ndt_errors'] = drive_metrics.fit_errors(leg['truth'],
                                                                   leg['ndt_fits'])
                rows.append((tour, name, label, row, notes, drawn))
                print(f'   {name}: ' + ('reached' if row['reached'] else 'NOT reached') +
                      f' in {row["time_s"]:.0f} s, {row["driven_m"]:.1f} m, off the path '
                      f'{row["off_path_mean_m"]:.2f} m on average' +
                      (f'; tick {row["loop_ms_mean"]:.2f} ms mean, {row["loop_ms_max"]:.1f} '
                       f'worst, {row["missed_ticks"]} missed' if 'loop_ms_mean' in row else '') +
                      (f'; AMCL off {row["amcl_mean_m"]:.2f} m mean over '
                       f'{row["amcl_samples"]} poses (EKF {row["belief_mean_m"]:.2f})'
                       if 'amcl_mean_m' in row else '') +
                      (f'; planned {row["plan_m"]:.1f} m in {row["plan_ms"]:.0f} ms, '
                       f'tightest turn {drive_metrics.radius_cell(row["plan_tightest_m"])} m'
                       if row.get('plan_found') else '') +
                      (f'; NDT off {row["ndt_mean_m"]:.3f} m mean, {row["ndt_max_m"]:.2f} '
                       f'worst, {row["ndt_available"]:.0%} of clouds, '
                       f'{row["ndt_match_ms_mean"]:.1f} ms a match'
                       if 'ndt_available' in row and row.get('ndt_samples') else '') +
                      (f'; KISS-ICP {row["kiss_drift_per_100m"]:.2f} m per 100 m, '
                       f'heading {row["kiss_heading_drift_deg"]:.2f} deg, '
                       f'{row.get("kiss_match_ms_mean", math.nan):.1f} ms a match '
                       f'(ekf_local {row["local_drift_per_100m"]:.2f} m per 100 m, '
                       f'{row["local_heading_drift_deg"]:.2f} deg)'
                       if row.get('kiss_odom_samples') else '') +
                      ''.join(f'; {n}' for n in notes), flush=True)
        finally:
            rec.destroy_node()
            rclpy.try_shutdown()
            if not stop(launch):
                print(f'   some of process group {launch.pid} would not stop', flush=True)
            launch.wait()
            # Programs that die leave Fast DDS's shared-memory files behind; too many of
            # those stop later runs talking (21 Oct).
            subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True)
    return rows


def report(results, args, started):
    """
    Return results.md: for each leg, a table with one column per follower.

    With --runs above 1, a column for each run (lqr #2), then each follower's mean over
    its runs (lqr, mean of 3), with the spread of the main numbers.
    """
    runs = ('One run each' if args.runs == 1 else
            f'{args.runs} runs each: every run, then the mean and its spread (SD)')
    lines = [f'# Controller comparison, {started:%Y-%m-%d %H:%M}', '',
             f"Ravensim ({', '.join(args.tours)}), headless, on this computer; "
             'the follower and ego at up to '
             f'{args.speed or 1.0:g} m/s, ' +
             ('reversing allowed' if args.reversing else 'forward only') +
             (', AMCL beside the EKF' if args.amcl else '') +
             (", NDT beside the EKF (its map: tools/ndt_map.py; Gazebo's lidar has no sweep, "
              "the car's does)" if args.ndt and not (args.ndt_ekf or args.ndt_leads)
              else '') +
             (', NDT leading ekf_global, the GPS checking (pose_source:=ndt, TODO 210)'
              if args.ndt_leads else '') +
             (f', a GPS fault on purpose after the first fix: {args.gps_fault} (the watchdog '
              'and geofence on, latches not cleared)' if args.gps_fault else '') +
             (", NDT's pose into the global EKF (ndt_ekf:=true)" if args.ndt_ekf else '') +
             (', the GPS stopped after the first fix (watchdog:=false geofence:=false)'
              if args.gps_off else '') +
             (', the lidar as cubes (voxels:=true, aav_voxels) instead of the flat scan'
              if args.voxels else '') +
             (', KISS-ICP lidar odometry beside ekf_local (each lined up with the truth at '
              "the start; Gazebo's lidar has no sweep, so no deskew)" if args.kiss_icp
              else '') +
             (", KISS-ICP's twist into ekf_local (kiss_ekf:=true)" if args.kiss_ekf else '') +
             (f', planners: {", ".join(args.planners)} (planner:=; the planner alone '
              "asked once before each leg, its turns against the car's full lock of "
              f'{vehicle.min_turning_radius(vehicle.load()["vehicle"]):.2f} m)'
              if args.planners else '') +
             f'. {runs}. The trial trackers (docs/trials.md) share one speed policy; '
             'mppi_lag sets its own. What each number means: '
             'tools/drive_metrics.py.', '']
    for tour in args.tours if args.planners else []:
        # The whole tour, one column a follower (and run): the planners' table.
        columns = list(dict.fromkeys((c, drawn.get('run', 1))
                                     for t, _, c, _, _, drawn in results if t == tour))
        rows = [(c if args.runs == 1 else f'{c} #{r}',
                 drive_metrics.overall([row for t, _, c2, row, _, d in results
                                        if t == tour and c2 == c and d.get('run', 1) == r]))
                for c, r in columns]
        lines += [f'## {tour}: the whole tour', '',
                  drive_metrics.table(rows, columns=drive_metrics.OVERALL), '']
    for tour in args.tours:
        # Least sideways swing first, of those that reached every goal: which rides best.
        columns = list(dict.fromkeys(c for t, _, c, *_ in results if t == tour))
        rows = [(c, drive_metrics.comfort([row for t, _, c2, row, *_ in results
                                           if t == tour and c2 == c])) for c in columns]
        rows.sort(key=lambda r: drive_metrics.comfort_order(r[1]))
        lines += [f'## {tour}: which rides best', '',
                  drive_metrics.table(rows, columns=drive_metrics.COMFORT), '']
    for tour in args.tours if args.kiss_icp else []:
        lines += [f'## {tour}: odometry over the whole tour', '',
                  drive_metrics.table(tour_odometry(results, tour, args.runs),
                                      columns=ODOMETRY), '']
    for tour in args.tours:
        for name, x, y, heading in tour_legs(tour, args):
            legs = [(c, row, notes, drawn.get('run', 1))
                    for t, n, c, row, notes, drawn in results if t == tour and n == name]
            if x is None:
                # A route's goal: where its last waypoint fell on the map, in a run.
                ends = [d['goal'] for t, n, _, _, _, d in results
                        if t == tour and n == name and d.get('goal')]
                x, y, heading = (*ends[0], 0.0) if ends else (math.nan, math.nan, math.nan)
            lines += [f'## {tour}: {name} (to {x:g}, {y:g}, facing {heading:g} deg)', '']
            if args.runs == 1:
                rows = [(c, row) for c, row, _, _ in legs if row is not None]
            else:
                rows = [(f'{c} #{r}', row) for c, row, _, r in legs if row is not None]
                for c in dict.fromkeys(c for c, *_ in legs):
                    mine = [row for f, row, _, _ in legs if f == c]
                    rows.append((f'{c}, mean of {len(mine)}', drive_metrics.combine(mine)))
            if rows:
                lines += [drive_metrics.table(rows), '']
            lines += [f'- {c}' + (f' #{r}' if args.runs > 1 else '') + f': {"; ".join(notes)}'
                      for c, _, notes, r in legs if notes]
            lines.append('')
    if args.ndt:
        lines += ndt_calibration(results)
    return '\n'.join(lines)


ODOMETRY = [c for c in drive_metrics.COLUMNS
            if c[0].startswith(('kiss_', 'local_')) or c[0] == 'driven_m']


def tour_odometry(results, tour, runs):
    """Return [(column, row)]: KISS-ICP's and ekf_local's drift over each run's whole tour."""
    out = []
    keys = dict.fromkeys((c, d.get('run', 1)) for t, _, c, _, _, d in results
                         if t == tour and d and 'kiss' in d)
    for c, r in keys:
        legs = [d for t, _, c2, _, _, d in results
                if t == tour and c2 == c and d and d.get('run', 1) == r and 'kiss' in d]
        truth = [tuple(s) for d in legs for s in d['track']]
        row = {'driven_m': drive_metrics.travelled(truth)}
        for key in ('kiss', 'local'):
            row.update(drive_metrics.odometry_row(key, truth,
                                                  [tuple(s) for d in legs for s in d[key]]))
        out.append((c if runs == 1 else f'{c} #{r}', row))
    return out


def open_picture(results, tour, args):
    """Return an SVG of a tour with no drawing of its world: the tracks, framed by themselves."""
    runs = [r for r in results if r[0] == tour and r[5] and r[5].get('track')]
    tracks = []
    for controller in dict.fromkeys(r[2] for r in runs):
        line = [(x, y) for r in runs if r[2] == controller for _, x, y, *_ in r[5]['track']]
        tracks.append((f'{controller}: where the car went', COLOURS.get(controller, 'black'),
                       [line], False))
    # A route's goal is only known once a run asked /fromLL: from its first leg.
    marks = [(x, y, name) for name, x, y, _ in tour_legs(tour, args) if x is not None]
    marks += [(r[5]['goal'][0], r[5]['goal'][1], r[1]) for r in runs[:1]
              if r[5].get('goal') and not marks]
    xs = [p[0] for _, _, lines, _ in tracks for p in lines[0]] + [m[0] for m in marks]
    ys = [p[1] for _, _, lines, _ in tracks for p in lines[0]] + [m[1] for m in marks]
    return drive_metrics.svg((min(xs) - 5, min(ys) - 5, max(xs) + 5, max(ys) + 5), tracks, (),
                             marks, scale=3.0)


def picture(results, tour, args):
    """Return an SVG of one tour on the test lot: each controller's track, the first plans."""
    if 'world' in TOURS[tour]:
        return open_picture(results, tour, args)
    with open(MAP_SPEC) as f:
        spec = yaml.safe_load(f)
    (ox, oy), (w, h), wall = spec['origin'], spec['size'], spec.get('border', 0.0)
    grey, white = '#9e9e9e', 'white'
    blocks = [(ox, oy, ox + w, oy + wall, grey), (ox, oy + h - wall, ox + w, oy + h, grey),
              (ox, oy, ox + wall, oy + h, grey), (ox + w - wall, oy, ox + w, oy + h, grey)]
    blocks += [(*b, grey) for b in spec.get('blocked', [])]
    blocks += [(*b, white) for b in spec.get('free', [])]
    if TOURS[tour]['obstacle']:
        bx, by = worlds.OBSTACLE
        blocks.append((bx - BOX / 2, by - BOX / 2, bx + BOX / 2, by + BOX / 2, '#c92a2a'))
    # A run that never came up has a leg without a track: nothing to draw.
    runs = [r for r in results if r[0] == tour and r[5] and 'track' in r[5]]
    tracks = []
    for controller in dict.fromkeys(c for _, _, c, *_ in runs):
        legs = [r for r in runs if r[2] == controller]
        # One line per run: a run's legs join up, two runs' don't.
        lines = [[(x, y) for r in legs if r[5].get('run', 1) == n for _, x, y, *_ in r[5]['track']]
                 for n in dict.fromkeys(r[5].get('run', 1) for r in legs)]
        tracks.append((f'{controller}: where the car went', COLOURS.get(controller, 'black'),
                       lines, False))
    plans = [r[5]['plan'] for r in runs
             if r[2] == runs[0][2] and r[5].get('run', 1) == 1 and 'plan' in r[5]]
    tracks.append(('the first plan of each leg', '#868e96', plans, True))
    marks = [(x, y, name) for name, x, y, _ in tour_legs(tour, args)]
    return drive_metrics.svg((ox, oy, ox + w, oy + h), tracks, blocks, marks)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--controllers', nargs='+', choices=vehicle.CONTROLLERS,
                        default=None,
                        help='mppi (mppi_lag, the champion, since Nav2 went); the '
                             'default when no --trackers is given')
    parser.add_argument('--trackers', nargs='+', type=tracker_names, default=None,
                        help='the trial trackers (aav_track, docs/trials.md): pure_pursuit, '
                             'stanley, lqr, mpc, mpc_full, mpc_lag, mpc_lag_rs; spaces or commas '
                             'between. '
                             'Forward only')
    parser.add_argument('--planners', nargs='+', type=planner_names, default=None,
                        help='the planners (planner:=): ' + ', '.join(PLANNER_WORDS) +
                             ' (own32: ours on its 32-heading file); each behind '
                             'mppi_lag unless --trackers says')
    parser.add_argument('--stacks', nargs='+', choices=list(STACKS), default=None,
                        help='every navigation piece ours (own: '
                             'navigator, planner, costmap, mppi_lag; own32: the same on the '
                             "lattice's 32 headings; own_slow: map_correction:=slow; "
                             'own_strong: plan_smoothing:=strong; '
                             'own_strong_slow: both; own_calm: both and '
                             'mppi_lag_steer_gamma:=0.05)')
    parser.add_argument('--smoothers', nargs='+', choices=list(vehicle.SMOOTHERS),
                        default=None,
                        help='each follower behind our velocity smoother (own: '
                             'aav_smoother, smoother:=), the only one since Nav2 went')
    parser.add_argument('--launch', nargs='+', default=None, metavar='NAME:=VALUE',
                        help='more sim.launch.py arguments for every run, such as '
                             'lidar:=false (to find what a failure needs)')
    parser.add_argument('--runs', type=int, default=1,
                        help='drive each follower this many times (default 1; 3 for a real '
                             'comparison: the mean and spread)')
    parser.add_argument('--amcl', action='store_true',
                        help='run AMCL beside the EKF (amcl:=true) and score its pose too')
    parser.add_argument('--ndt', action='store_true',
                        help='run NDT beside the EKF (ndt:=true) and score it too; needs '
                             'the map: pixi run ndt-map drive')
    parser.add_argument('--ndt-map', metavar='PCD',
                        help="with --ndt: this map, not the world's own (such as "
                             "map-drive's <drive>/map/ndt.pcd)")
    parser.add_argument('--ndt-ekf', action='store_true',
                        help="as --ndt, and the global EKF takes NDT's pose (ndt_ekf:=true)")
    parser.add_argument('--gps-off', action='store_true',
                        help='stop the GPS once the EKF has its first fix (watchdog:=false '
                             'geofence:=false)')
    parser.add_argument('--ndt-leads', action='store_true',
                        help='NDT leads ekf_global, the GPS checks (pose_source:=ndt, '
                             'TODO 210); NDT scored as with --ndt')
    parser.add_argument('--gps-fault', choices=('stop', 'jump', 'drift'), default=None,
                        help='a GPS fault on purpose once the EKF has its first fix, the '
                             'watchdog and geofence on; latches not cleared')
    parser.add_argument('--voxels', action='store_true',
                        help='the costmap and the stop zone read the lidar as cubes '
                             '(voxels:=true, aav_voxels), not the flat scan')
    parser.add_argument('--kiss-icp', action='store_true',
                        help='run KISS-ICP lidar odometry (kiss_icp:=true) and score its '
                             'drift beside ekf_local; needs pixi install --locked -e kiss')
    parser.add_argument('--kiss-ekf', action='store_true',
                        help="as --kiss-icp, and ekf_local takes KISS-ICP's twist "
                             '(kiss_ekf:=true)')
    parser.add_argument('--tours', nargs='+', choices=list(TOURS),
                        default=[t for t in TOURS if 'world' not in TOURS[t]])
    parser.add_argument('--speed', type=float, default=None,
                        help="m/s, up to 5: sim_speed:= (the car's own top speed: 1.0)")
    parser.add_argument('--lag-scale', type=float, default=1.0,
                        help="the car's throttle, brake and steering lags times this "
                             "(sim's lag_scale:=): the followers still read "
                             "vehicle.yaml's")
    parser.add_argument('--reversing', action='store_true',
                        help="let the car reverse (reversing:=true; the car's own default is "
                             'false until H5); forward only otherwise')
    parser.add_argument('--legs', type=int, default=None,
                        help="only each tour's first N legs (each starts where the last ended)")
    parser.add_argument('--plans', action='store_true',
                        help='save every plan of each leg in results.json, not just its first')
    parser.add_argument('--series', action='store_true',
                        help="save each leg's truth, EKF, steering and turn rates (before "
                             'and after the smoother) every 0.05 s in results.json: the '
                             "wiggle's raw data")
    parser.add_argument('--diffusion-model', default=None,
                        help='with --planners diffusion: this model (.onnx) instead of '
                             'models/diffusion/planner.onnx')
    parser.add_argument('--diffusion-smoothing', choices=('gear', 'average'), default=None,
                        help="with --planners diffusion: how the model's paths are smoothed "
                             '(gear, the default; average: 2 Nov)')
    parser.add_argument('--diffusion-record', default=None,
                        help="with --planners diffusion: save each plan's inputs in this "
                             'folder (a subfolder a run), to replay offline: '
                             'pixi run diffusion replay')
    parser.add_argument('--out', help='where the results go (default: log/controller-compare/)')
    args = parser.parse_args()
    if args.trackers:
        args.trackers = list(dict.fromkeys(t for names in args.trackers for t in names))
    if args.planners:
        args.planners = list(dict.fromkeys(p for names in args.planners for p in names))
    if args.stacks and (args.trackers or args.controllers or args.planners):
        parser.error('--stacks sets the follower and planner: leave out --trackers, '
                     '--controllers and --planners')
    forward_only = set(args.trackers or ()) & set(vehicle.FORWARD_ONLY_TRACKERS)
    if forward_only and args.reversing:
        parser.error(f'--trackers {" ".join(sorted(forward_only))} drive forwards only: '
                     'leave out --reversing')
    if args.runs < 1:
        parser.error('--runs must be 1 or more')
    args.ndt = args.ndt or args.ndt_ekf or args.ndt_leads
    if args.ndt_map:
        NDT_MAP_GIVEN.append(os.path.abspath(args.ndt_map))
    args.kiss_icp = args.kiss_icp or args.kiss_ekf
    for tour in args.tours:
        # The files a tour needs, before half an hour of driving finds one missing.
        plan = TOURS[tour]
        if args.ndt and not os.path.isfile(ndt_map(plan.get('world'))):
            parser.error(f'--ndt: no map at {ndt_map(plan.get("world"))}; make it with '
                         'pixi run ndt-map drive (the lot) or pixi run campus')
        if plan.get('world') == worlds.CAMPUS and not os.path.isfile(
                os.path.join(ROOT, worlds.CAMPUS_MAP)):
            parser.error(f'--tours {tour}: no {worlds.CAMPUS_MAP}; make it with pixi run campus')
        if plan.get('route'):
            try:
                campus_route(*plan['route'])
            except Exception as e:  # noqa: B902 - any reason the route can't be drawn
                parser.error(f'--tours {tour}: no route {plan["route"][0]} to '
                             f'{plan["route"][1]}: {e}')
    started = datetime.datetime.now()
    out_dir = args.out or os.path.join(ROOT, 'log', 'controller-compare',
                                       f'{started:%Y%m%d-%H%M%S}')
    os.makedirs(out_dir, exist_ok=True)
    car = vehicle.load()['vehicle']['body']
    body = (car['length'], car['width'], car['rear_overhang'])
    results, used = [], set()
    for tour in args.tours:
        for follower in followers(args):
            for run in range(1, args.runs + 1):
                try:
                    results += session(tour, follower, args, out_dir, body, used, run)
                except Exception as e:  # noqa: B902 - one tour's crash must not lose the rest
                    print(f'   {tour} crashed: {e!r}', flush=True)
                    results += [(tour, name, follower[-1], None,
                                 [f'crashed: {e}'], {'run': run})
                                for name, *_ in tour_legs(tour, args)]
    # The numbers first: a report that fails must not lose them.
    with open(os.path.join(out_dir, 'results.json'), 'w') as f:
        json.dump([{'tour': t, 'leg': n, 'controller': c, 'row': row, 'notes': notes, **drawn}
                   for t, n, c, row, notes, drawn in results], f, indent=1, default=str)
    text = report(results, args, started)
    with open(os.path.join(out_dir, 'results.md'), 'w') as f:
        f.write(text)
    for tour in args.tours:
        with open(os.path.join(out_dir, f'{tour}.svg'), 'w') as f:
            f.write(picture(results, tour, args))
    print('\n' + text + f'\nSaved in {out_dir}', flush=True)
    return 0 if all(row and row['reached'] for _, _, _, row, _, _ in results) else 1


if __name__ == '__main__':
    sys.exit(main())

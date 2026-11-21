"""
Read config/vehicle.yaml and derive what other nodes need from it.

The launch files use this so the car's measurements live in one file:

    from aav_description import vehicle
    v = vehicle.load()['vehicle']
    vehicle.min_turning_radius(v)    # Nav2 SmacPlannerHybrid, MPPI Ackermann
    vehicle.footprint(v)             # Nav2 costmaps
    vehicle.stop_zone_points(v)      # Nav2 collision_monitor: ahead, going forward
    vehicle.rear_stop_zone_points(v) # ... and behind, reversing
    vehicle.blind_behind(reversing, lidar=..., rear_camera=...)  # may Nav2 reverse?
    vehicle.obstacle_heights(v)      # the lidar's band of obstacle heights (lidar_scan)
    vehicle.nav2_rewrites(v, reversing, planner)  # Nav2's file: forward only, or reversing too
    vehicle.check_planner(name, real)  # planner:=, only the car-shaped ones on the car
    vehicle.nav2_tuned(nav2, v, reversing, max_speed, radar, lidar, ghost,  # what
                       controller)                              # RewrittenYaml can't write
    vehicle.ego_params(vehicle.load())          # ego: the whole file (the actuators too)
    vehicle.track_params(nav2, cfg, ego, max_speed)  # aav_track: a trial tracker (tracker:=)
    vehicle.mppi_lag_params(nav2, cfg, ego, max_speed, reversing, lane)  # aav_mppi_lag's
    vehicle.mpc_lag_rs_params(nav2, cfg, ego, max_speed)  # aav_mpc_lag's (tracker:=mpc_lag_rs)
    vehicle.check_speed_loop(name, tracker)  # speed_loop:=, the profile every car model assumes
    vehicle.check_speed_loop_model(name, car_loop, tracker)  # speed_loop_model:=, '' = car's
    vehicle.centre_of_mass(v, front, rear, front_lifted, lift)   # from the scales (M7, M7b)
    vehicle.wheel_angles(v, centre)  # each front wheel's angle for a centre (bicycle) angle
    vehicle.centre_angle(v, inner, outer)       # the reverse: from M5's two measured angles
"""

import copy
import math
import os

from aav_description.car_model import SPEED_LOOP_PROFILES
import yaml


def config_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('aav_description'), 'config', 'vehicle.yaml')


def load(path=None):
    with open(path or config_path()) as f:
        return yaml.safe_load(f)


def min_turning_radius(v):
    """Radius of the path of base_link (the rear axle centre) at full lock, in metres."""
    return v['wheelbase'] / math.tan(v['max_steering_angle'])


# max_steering_angle is the virtual wheel in the middle of the front axle: the bicycle
# model's angle, which ego and Nav2 use. With Ackermann steering both front wheels point
# at one turning centre on the rear axle's line, so the inner wheel turns further and the
# outer less (docs/measure-the-car.md M5 measures both):
#
#     cot(outer) - cot(inner) = track / wheelbase,   cot(centre) = (cot(inner) + cot(outer)) / 2


def wheel_angles(v, centre):
    """Return (inner, outer) front wheel angles (rad, >= 0) for a centre angle (rad)."""
    c = abs(centre)
    if c == 0.0:
        return 0.0, 0.0
    radius = v['wheelbase'] / math.tan(c)       # rear axle centre to the turning centre
    half = v['track'] / 2.0
    return (math.atan2(v['wheelbase'], radius - half),
            math.atan2(v['wheelbase'], radius + half))


def centre_angle(v, inner, outer):
    """Return the centre (bicycle) angle from both measured front wheel angles (rad)."""
    if inner <= 0.0 or outer <= 0.0:
        raise ValueError('measure both wheels turned the same way, each > 0')
    return math.atan(2.0 / (1.0 / math.tan(inner) + 1.0 / math.tan(outer)))


def footprint_points(v):
    """Return the body outline as [x, y] corners in base_link, front-left first."""
    body = v['body']
    rear = -body['rear_overhang']
    front = rear + body['length']
    half = body['width'] / 2.0
    return [[front, half], [front, -half], [rear, -half], [rear, half]]


# The car as its own lidar sees it: the body and every sensor mounted on it, widened by a
# margin. Gazebo's lidar (3 cm of range noise) put about 3 % of its hits on the car's own
# roof corners and front cameras just outside a box flush with the body; over a drive
# they drew a ghost double line 1.5 m up in NDT's map (found 30 Oct). 0.15 m is 5 sigma of
# that noise, and covers a sensor's own housing (the URDF draws each one 8 cm across).
SELF_MARGIN = 0.15      # m, each side
SENSOR_HALF_SIZE = 0.04  # m: half a sensor's box in the URDF (aav.urdf.xacro sensor_frame)


def self_box(cfg, margin=SELF_MARGIN):
    """
    Return the box, in base_link from above, that holds the car and its sensors.

    cfg is the whole vehicle.yaml (load()). Returns {'min': [x, y], 'max': [x, y]} (m):
    the body outline and each sensor's mount (plus SENSOR_HALF_SIZE), widened by margin.
    A lidar point inside is the car itself: NDT's scan filter and its map drop them.
    """
    (front, left), _, (rear, right), _ = footprint_points(cfg['vehicle'])
    xs, ys = [rear, front], [right, left]
    for sensor in cfg.get('sensors', {}).values():
        x, y = float(sensor['xyz'][0]), float(sensor['xyz'][1])
        xs += [x - SENSOR_HALF_SIZE, x + SENSOR_HALF_SIZE]
        ys += [y - SENSOR_HALF_SIZE, y + SENSOR_HALF_SIZE]
    return {'min': [min(xs) - margin, min(ys) - margin],
            'max': [max(xs) + margin, max(ys) + margin]}


def polygon(points):
    """Return [x, y] corners in the string form Nav2 takes for a polygon."""
    return '[' + ', '.join(f'[{x:.3f}, {y:.3f}]' for x, y in points) + ']'


def footprint(v):
    """Return the outline in the string form Nav2's costmaps take for `footprint`."""
    return polygon(footprint_points(v))


# collision_monitor's stop zone, ahead of the front bumper. At Nav2's 1 m/s, ego's 2 m/s^2
# stops the car in 0.25 m; add about 0.4 s for the radar's frame, the mux, ego and the
# brakes (0.4 m), and 0.35 m for the radar's range noise and the guessed measurements.
STOP_ZONE_AHEAD = 1.0   # m
STOP_ZONE_MARGIN = 0.2  # m each side of the body


def stop_zone_points(v, ahead=STOP_ZONE_AHEAD, margin=STOP_ZONE_MARGIN):
    """Return the stop zone's corners: from the front bumper to `ahead` past it, widened."""
    body = v['body']
    front = body['length'] - body['rear_overhang']
    half = body['width'] / 2.0 + margin
    return [[front + ahead, half], [front + ahead, -half], [front, -half], [front, half]]


# The stop zone while reversing, behind the rear bumper. Reversing is at most
# max_reverse_speed (0.5 m/s): ego's 2 m/s^2 stops it in 0.06 m, 0.4 s of delay adds 0.2 m;
# the rest is margin for the lidar's noise and a guessed rear overhang.
STOP_ZONE_BEHIND = 1.0  # m


def rear_stop_zone_points(v, behind=STOP_ZONE_BEHIND, margin=STOP_ZONE_MARGIN):
    """Return the reversing stop zone's corners: from the rear bumper to `behind` past it."""
    body = v['body']
    rear = -body['rear_overhang']
    half = body['width'] / 2.0 + margin
    return [[rear, half], [rear, -half], [rear - behind, -half], [rear - behind, half]]


# What can see behind the car, for collision_monitor's reversing zone: the roof lidar
# (all round), and the rear camera's detections (camera_obstacles, rear_camera=True: its
# detector reads the rear picture). Not the radar: it faces forward. Not camera_obstacles
# alone: by default it reads the front left camera only.
SEES_BEHIND = ('lidar', 'rear_camera')


def blind_behind(reversing, **sensors):
    """
    Why navigation must not reverse with these sensors, or None if it may.

    sensors: name=True for each one running (lidar=..., radar=..., rear_camera=...).
    """
    if not reversing or any(sensors.get(name) for name in SEES_BEHIND):
        return None
    return ('nothing sees behind the car (the radar faces forward): navigation stays forward '
            "only. Start the lidar (or the rear camera's obstacles) to let it reverse")


# The lidar's obstacles: what lies between these heights above the road (base_link's z).
# Below: the road itself, kerbs, and the ground the lidar sees as its beams tilt with the
# car (0.3 m is the ground 10 m away, with the car pitched 1.7 degrees). Above the car's
# roof by more than OVERHEAD_MARGIN: branches, signs, a garage door the car passes under.
GROUND_CLEARANCE = 0.3  # m
# m: how far the planner's copy of the lidar's layer marks (the follower's: 12 m). A box
# parked in the lane was first planned round 7-11 m ahead at 2.5 m/s (the global costmap
# and the plan each once a second), too late for a gentle pass (16 Nov). Not further: the
# ground at 20 m is 0.6 m up with the car pitched 1.7 degrees.
PLANNER_SEES = 18.0
OVERHEAD_MARGIN = 0.5   # m above the body's height


def obstacle_heights(v):
    """Return (lowest, highest) m above the road that counts as an obstacle for the lidar."""
    return GROUND_CLEARANCE, float(v['body']['height']) + OVERHEAD_MARGIN


# Both EKFs let the car's command slow their speed (robot_localization's control term,
# TODO 207). With the wheels silent (a fault, a lost frame) an EKF with no speed
# measurement kept its last speed and its last acceleration: ekf_local ran on 15 m while
# the car stood, and map -> odom jumped 10 m back when the wheels came back (the
# follower's path then lay outside its costmap, and goals failed). Now a command below the
# speed slows the prediction toward it (or to 0 once the command is silent
# EKF_CONTROL_TIMEOUT s: twist_mux's locks send nothing), at EKF_CONTROL_GAIN (1/s) and at
# most the car's max_brake. A command above the speed never speeds the prediction up
# (acceleration limit 0): the car answers late (its actuators' lag), and a prediction
# that ran ahead of wheels saying 0 dragged the position metres on (tried, 17 Nov). The
# wheels, when they speak, still decide.
EKF_CONTROL_TOPIC = '/control/cmd_vel'
EKF_CONTROL_TIMEOUT = 0.5
EKF_CONTROL_GAIN = 2.0


def ekf_control_params(cfg):
    """
    Return robot_localization's control term for both EKFs, from vehicle.yaml's actuators.

    Only the forward speed (vx) follows the command, and only down: acceleration limit 0,
    deceleration up to max_brake. The turn is the IMU's. Six values each (vx, vy, vz,
    vroll, vpitch, vyaw), as robot_localization wants.
    """
    brake = float(cfg['actuators']['max_brake'])
    return {'use_control': True, 'stamped_control': True,
            'control_timeout': EKF_CONTROL_TIMEOUT,
            'control_config': [True, False, False, False, False, False],
            'acceleration_limits': [0.0] + [1.0] * 5,
            'acceleration_gains': [EKF_CONTROL_GAIN] + [1.0] * 5,
            'deceleration_limits': [brake] + [1.0] * 5,
            'deceleration_gains': [EKF_CONTROL_GAIN] + [1.0] * 5}


def ego_params(cfg):
    """Return what ego takes from vehicle.yaml (the whole file): its geometry and its body."""
    v, act = cfg['vehicle'], cfg['actuators']
    return {
        'wheelbase': float(v['wheelbase']),
        'max_steering_angle': float(v['max_steering_angle']),
        'max_reverse_speed': float(v['max_reverse_speed']),
        'mass': float(v['mass']),
        'track': float(v['track']),
        'cg_height': float(v['centre_of_mass']['height']),
        'tyre_grip': float(v['tyre_grip']),
        'full_throttle_accel': float(act['max_accel']),
        'full_brake_decel': float(act['max_brake']),
    }


def centre_of_mass(v, front, rear, front_lifted=None, lift=None):
    """
    Return the centre of mass from the scales: (x ahead of the rear axle, height), m.

    front, rear: the weight on each axle, level (docs/measure-the-car.md, M7; any unit).
    front_lifted, lift: the front axle again with both rear wheels on blocks `lift` m high
    (M7b). Without them the height is None.

    Level, the front axle carries the share of the weight that x is of the wheelbase.
    Lifted, the car tips forward by asin(lift / L), and a centre of mass h - r above the
    axles moves (h - r) tan(tilt) forward, so the front gains W (h - r) tan(tilt) / L.
    """
    L, total = float(v['wheelbase']), float(front + rear)
    x = L * front / total
    if front_lifted is None or not lift:
        return x, None
    tilt = math.asin(lift / L)
    r = float(v['wheel_diameter']) / 2.0
    return x, r + L * (front_lifted - front) / (total * math.tan(tilt))


def spine_params(v):
    return {
        'wheelbase': float(v['wheelbase']),
        'track': float(v['track']),
    }


def reverse_limit(v, reversing):
    """Return the lowest speed Nav2 may command (m/s): -max_reverse_speed, or 0 forward only."""
    return -float(v['max_reverse_speed']) if reversing else 0.0


# The planner plans turns this much wider than the car's tightest, so the controller has
# steering left to stay on the path. At full lock in the plan, both controllers cut the last
# turn before a goal, arrived 0.5 m off and facing wrong, and drove round again (22 Oct).
PLAN_TURN_MARGIN = 1.25


def plan_turning_radius(v):
    """Return the turning radius the planner plans with (m): full lock, PLAN_TURN_MARGIN wider."""
    return min_turning_radius(v) * PLAN_TURN_MARGIN


def s_bend_length(radius, offset):
    """
    Return how far ahead (m) a car needs to step `offset` m sideways and face the same way.

    Two arcs of `radius`, one each way: the shortest forward move that does it. Beyond
    2 x radius sideways it can't be done this way: it's a loop (math.inf).
    """
    d = abs(offset)
    if d > 2.0 * radius:
        return math.inf
    return math.sqrt(4.0 * radius * d - d * d)


def final_approach(v):
    """
    Return how near the goal (m) Nav2 must keep its path rather than plan again (TODO 17).

    Planned from a few metres out, a little off the approach line, a car's path to the
    goal's heading is a loop: an S-bend back onto the line needs more room ahead than is
    left. The worst S-bend, 2 turning radii sideways, needs 2 radii ahead: within that,
    a new path may be a loop, so the behaviour trees keep the one they have.
    """
    r = plan_turning_radius(v)
    return s_bend_length(r, 2.0 * r)


# How often the navigator plans a to-pose goal again (Hz): Nav2's own tree's 1 Hz. The
# wiggle's second test (TODO 149) plans once per goal: REPLAN_ONCE_HZ, once in about three
# hours (navigator_params' replan_period).
REPLAN_HZ = 1.0
REPLAN_ONCE_HZ = 1e-4


# The follower's tuning block in nav2.yaml (controller_server's FollowPath): mppi, Nav2's
# MPPI block, which mppi_lag and the trackers read. Nav2's controller_server, and with it
# Regulated Pure Pursuit (rpp), went with Nav2 on 17 Nov (TODO 184): its scores are in
# docs/league-results-2026-11-16.md and docs/history/nav2-as-it-was.md.
CONTROLLERS = ('mppi',)
DEFAULT_CONTROLLER = 'mppi'
# nav2_tuned's layouts of FollowPath: mppi, the MPPI block copied there (what the launch
# always picks); rpp, the file's own FollowPath kept (Regulated Pure Pursuit's numbers,
# whose lookahead aav_track reads; and a part of nav2.yaml with no MPPI block, as tests give).
TUNED_CONTROLLERS = ('rpp', 'mppi')


# The path followers (tracker:=, docs/trials.md): each serves follow_path. mppi_lag, the
# default since Nav2 went (17 Nov), is aav_mppi_lag's node (Rust): Nav2's MPPI ported, on a
# car model with the actuators' lag (docs/mppi-lag.md). mpc_lag_rs is aav_mpc_lag's node
# (Rust): aav_track's mpc_lag ported, a challenger beside it (TODO 132, 199). The rest are
# aav_track's (Python). 'none' (Nav2's controller_server) went with Nav2.
TRACKERS = ('pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full', 'mpc_lag', 'mppi_lag',
            'mpc_lag_rs')
DEFAULT_TRACKER = 'mppi_lag'
# The trackers that drive forwards only: refused with reversing:=true. None since 20 Nov:
# mppi_lag reverses as Nav2's MPPI does (Ben, 14 Nov: "The car can reverse"); aav_track's
# since 19 Nov (TODO 174: each backwards stretch mirrored, aav_track gears.py); mpc_lag_rs
# since 20 Nov (TODO 213, the same, in aav_mpc_lag's gears.rs). Kept for a challenger.
FORWARD_ONLY_TRACKERS = ()


def check_tracker(tracker, reversing=False):
    """Return the tracker's name (lower case), or raise ValueError saying what to pick."""
    name = str(tracker).strip().lower()
    if name not in TRACKERS:
        raise ValueError(f'tracker:={tracker}: pick one of {", ".join(TRACKERS)}')
    if name in FORWARD_ONLY_TRACKERS and reversing:
        raise ValueError(f'tracker:={name} drives forwards only: add reversing:=false '
                         "(the planner would plan reversing it can't follow)")
    return name


# Whose costmaps (costmap:=, docs/own-costmap.md): own (aav_costmap), the only one since
# Nav2's went (17 Nov, TODO 184); the switch stays so a challenger can join it. Its raw
# local costmap, for aav_mppi_lag; its local costmap, for the screens' look ahead.
COSTMAPS = ('own',)
DEFAULT_COSTMAP = 'own'
COSTMAP_RAW_TOPICS = {'own': '/drivers/nav/costmap/local_raw'}
LOCAL_COSTMAP_TOPIC = '/drivers/nav/costmap/local'
GLOBAL_COSTMAP_GRID_TOPIC = '/drivers/nav/costmap/global'


def check_costmap(costmap):
    """Return the costmap's name (lower case), or raise ValueError saying what to pick."""
    name = str(costmap).strip().lower()
    if name not in COSTMAPS:
        raise ValueError(f'costmap:={costmap}: pick one of {", ".join(COSTMAPS)}')
    return name


# Whose navigator (navigator:=, docs/own-navigator.md): own (aav_navigator), the only one
# since bt_navigator and waypoint_follower went with Nav2 (17 Nov). It serves the four
# actions under Nav2's own names, so no goal sender changed.
NAVIGATORS = ('own',)
DEFAULT_NAVIGATOR = 'own'
NAVIGATOR_ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
                     'follow_gps_waypoints')


def check_navigator(navigator):
    """Return the navigator's name (lower case), or raise ValueError saying what to pick."""
    name = str(navigator).strip().lower()
    if name not in NAVIGATORS:
        raise ValueError(f'navigator:={navigator}: pick one of {", ".join(NAVIGATORS)}')
    return name


# Whose velocity smoother (smoother:=, docs/own-smoother.md): own (aav_smoother's
# car_smoother: the turn rate from where the wheels can be), the only one since Nav2's
# velocity_smoother went (17 Nov). It runs under that node name and reads its block.
SMOOTHERS = ('own',)
DEFAULT_SMOOTHER = 'own'


def check_smoother(smoother):
    """Return the smoother's name (lower case), or raise ValueError saying what to pick."""
    name = str(smoother).strip().lower()
    if name not in SMOOTHERS:
        raise ValueError(f'smoother:={smoother}: pick one of {", ".join(SMOOTHERS)}')
    return name


# Which voxels node cuts the lidar into cubes (voxels:= or screen_cubes:=, voxels_impl:=):
# python (aav_voxels, the default) or rust (aav_voxels_rs, its port: the same node name,
# topics and answers, about 8 times faster). Each one's package.
VOXELS_IMPLS = {'python': 'aav_voxels', 'rust': 'aav_voxels_rs'}
DEFAULT_VOXELS_IMPL = 'python'


def check_voxels_impl(impl):
    """Return the voxels node's language (lower case), or raise ValueError saying what to pick."""
    name = str(impl).strip().lower()
    if name not in VOXELS_IMPLS:
        raise ValueError(f'voxels_impl:={impl}: pick one of {", ".join(VOXELS_IMPLS)}')
    return name


# Which lidar_scan flattens the lidar (lidar:=true, lidar_impl:=): rust (aav_lidar_rs, the
# default since 17 Nov: the same node name, topics and ranges, 2 % of a core against 5 %)
# or python (aav_lidar, the fallback). Each one's package.
LIDAR_IMPLS = {'python': 'aav_lidar', 'rust': 'aav_lidar_rs'}
DEFAULT_LIDAR_IMPL = 'rust'


def check_lidar_impl(impl):
    """Return lidar_scan's language (lower case), or raise ValueError saying what to pick."""
    name = str(impl).strip().lower()
    if name not in LIDAR_IMPLS:
        raise ValueError(f'lidar_impl:={impl}: pick one of {", ".join(LIDAR_IMPLS)}')
    return name


# Which AMCL runs (amcl:=true, amcl_impl:=): rust (aav_amcl_rs, the default since 18 Nov,
# TODO 231: stock's topics, parameters and frames, a plain node). Stock nav2_amcl, the last
# of Nav2, went on 20 Nov (TODO 232, tag save/nav2-amcl); the switch stays, a hook.
AMCL_IMPLS = {'rust': 'aav_amcl_rs'}
DEFAULT_AMCL_IMPL = 'rust'


def check_amcl_impl(impl):
    """Return which AMCL runs (lower case), or raise ValueError saying what to pick."""
    name = str(impl).strip().lower()
    if name not in AMCL_IMPLS:
        raise ValueError(f'amcl_impl:={impl}: pick one of {", ".join(AMCL_IMPLS)}')
    return name


# Which things node tracks the cubes (things:=true, things_impl:=): rust (aav_things_rs,
# the default since 17 Nov: the same node name, topics, parameters and things, 1.5 % of a
# core against 7.5 %). The Python went on 20 Nov (tag save/python-things); the switch
# stays, a challenger's hook.
THINGS_IMPLS = {'rust': 'aav_things_rs'}
DEFAULT_THINGS_IMPL = 'rust'


def check_things_impl(impl):
    """Return the things node's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in THINGS_IMPLS:
        raise ValueError(f'things_impl:={impl}: pick one of {", ".join(THINGS_IMPLS)}')
    return name


# Which object detector runs (detector.launch.py, sim.launch.py detector_impl:=): python
# (aav_detector: PyTorch, Core ML or TensorRT; the default) or rust (aav_detector_rs:
# RF-DETR's ONNX through ONNX Runtime, a challenger, docs/onnx.md). The same node name and
# topics. Each one's package.
DETECTOR_IMPLS = {'python': 'aav_detector', 'rust': 'aav_detector_rs'}
DEFAULT_DETECTOR_IMPL = 'python'


# Which model worker the diffusion planner runs (navigation.launch.py, sim.launch.py
# diffusion_impl:=): python (worker.py in the learn environment; the default) or rust
# (aav_diffusion_rs's diffusion_worker: the same pipe, ONNX Runtime from Rust, a
# challenger, docs/onnx.md). The node, its guard and the lattice's fallback are the same.
DIFFUSION_IMPLS = ('python', 'rust')
DEFAULT_DIFFUSION_IMPL = 'python'


def check_diffusion_impl(impl):
    """Return the diffusion worker's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in DIFFUSION_IMPLS:
        raise ValueError(f'diffusion_impl:={impl}: pick one of {", ".join(DIFFUSION_IMPLS)}')
    return name


def check_detector_impl(impl):
    """Return the detector's language (lower case), or raise ValueError saying what to pick."""
    name = str(impl).strip().lower()
    if name not in DETECTOR_IMPLS:
        raise ValueError(f'detector_impl:={impl}: pick one of {", ".join(DETECTOR_IMPLS)}')
    return name


# Which ego runs (ego.launch.py, car.launch.py, sim.launch.py ego_impl:=): rust
# (aav_ego_rs: ego and look_ahead in one process; the default on the car and in the
# simulator, Ben, 16 Nov) or python (aav_ego's two nodes: the same node names, topics and
# answers; kept selectable until a car day proves the Rust). Each one's package.
EGO_IMPLS = {'python': 'aav_ego', 'rust': 'aav_ego_rs'}
DEFAULT_EGO_IMPL = 'rust'
# The simulator's default: the car's since 16 Nov (the name stays for sim.launch.py).
SIM_DEFAULT_EGO_IMPL = DEFAULT_EGO_IMPL


def check_ego_impl(impl):
    """Return ego's language (lower case), or raise ValueError saying what to pick."""
    name = str(impl).strip().lower()
    if name not in EGO_IMPLS:
        raise ValueError(f'ego_impl:={impl}: pick one of {", ".join(EGO_IMPLS)}')
    return name


# Which behaviour node runs, with behaviour:=true (navigation.launch.py, car.launch.py,
# sim.launch.py behaviour_impl:=): rust (aav_behaviour_rs, the default on the car and in
# the simulator, Ben, 16 Nov). The Python went on 20 Nov (tag save/python-behaviour); the
# switch stays, a challenger's hook.
BEHAVIOUR_IMPLS = {'rust': 'aav_behaviour_rs'}
DEFAULT_BEHAVIOUR_IMPL = 'rust'
# The simulator's default: the car's since 16 Nov (the name stays for sim.launch.py).
SIM_DEFAULT_BEHAVIOUR_IMPL = DEFAULT_BEHAVIOUR_IMPL


# Which sensor watchdog runs (watchdog.launch.py, car.launch.py, sim.launch.py
# watchdog_impl:=): rust (aav_watchdog_rs, the default since Ben decided, 16 Nov:
# "watchdog in rust sure"). The Python went on 20 Nov (tag save/python-safety-nodes); the
# switch stays, a challenger's hook.
WATCHDOG_IMPLS = {'rust': 'aav_watchdog_rs'}
DEFAULT_WATCHDOG_IMPL = 'rust'


def check_watchdog_impl(impl):
    """Return the sensor watchdog's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in WATCHDOG_IMPLS:
        raise ValueError(f'watchdog_impl:={impl}: pick one of {", ".join(WATCHDOG_IMPLS)}')
    return name


# Which sensor mux and output mux run (io_mux.launch.py, car.launch.py, sim.launch.py
# io_mux_impl:=): rust (aav_io_mux_rs, the default on the car and in the simulator since
# Ben decided, 17 Nov: "Rust tomorrow ... I don't think we want to do this again later")
# or python (aav_io_mux: the same node names, topics and answers; kept selectable until a
# car day proves the Rust). Both muxes switch together. Each one's package.
IO_MUX_IMPLS = {'python': 'aav_io_mux', 'rust': 'aav_io_mux_rs'}
DEFAULT_IO_MUX_IMPL = 'rust'


def check_io_mux_impl(impl):
    """Return the muxes' language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in IO_MUX_IMPLS:
        raise ValueError(f'io_mux_impl:={impl}: pick one of {", ".join(IO_MUX_IMPLS)}')
    return name


# Which legacy_uart_bridge runs (legacy.launch.py, car.launch.py, sim.launch.py esp32:=true;
# legacy_impl:=): rust (aav_legacy_rs, the default since Ben decided, 17 Nov: "Rust
# tomorrow ... I don't think we want to do this again later") or python (aav_legacy: the
# same node name, topics, parameters and frames; kept until a car day proves the Rust).
LEGACY_IMPLS = {'python': 'aav_legacy', 'rust': 'aav_legacy_rs'}
DEFAULT_LEGACY_IMPL = 'rust'


def check_legacy_impl(impl):
    """Return the UART bridge's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in LEGACY_IMPLS:
        raise ValueError(f'legacy_impl:={impl}: pick one of {", ".join(LEGACY_IMPLS)}')
    return name


# Which rc_driver runs (rc.launch.py, legacy.launch.py, car.launch.py; rc_impl:=): rust
# (aav_rc_rs, the default since 17 Nov, with the bridge) or python (aav_rc).
RC_IMPLS = {'python': 'aav_rc', 'rust': 'aav_rc_rs'}
DEFAULT_RC_IMPL = 'rust'


def check_rc_impl(impl):
    """Return rc_driver's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in RC_IMPLS:
        raise ValueError(f'rc_impl:={impl}: pick one of {", ".join(RC_IMPLS)}')
    return name


# Which point_driver drives to one point (drive_to_point.launch.py, and the sim, bench and car
# profiles that start it; drive_to_point_impl:=): rust (aav_drive_to_point_rs, the default
# since 18 Nov: the same node name, action, topics and answers) or python
# (aav_drive_to_point, the fall-back). Each one's package.
DRIVE_TO_POINT_IMPLS = {'python': 'aav_drive_to_point', 'rust': 'aav_drive_to_point_rs'}
DEFAULT_DRIVE_TO_POINT_IMPL = 'rust'


def check_drive_to_point_impl(impl):
    """Return point_driver's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in DRIVE_TO_POINT_IMPLS:
        raise ValueError(f'drive_to_point_impl:={impl}: pick one of '
                         f'{", ".join(DRIVE_TO_POINT_IMPLS)}')
    return name


# Which arm_when_open arms the stand-ins' spine (bench.launch.py arm:=true, ravensim.launch.py
# spine:=true; arm_when_open_impl:=): rust (aav_spine_bridge's, beside the Rust bridge, the
# default since 18 Nov) or python (aav_spine's). Each one's package.
ARM_WHEN_OPEN_IMPLS = {'python': 'aav_spine', 'rust': 'aav_spine_bridge'}
DEFAULT_ARM_WHEN_OPEN_IMPL = 'rust'


def check_arm_when_open_impl(impl):
    """Return arm_when_open's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in ARM_WHEN_OPEN_IMPLS:
        raise ValueError(f'arm_when_open_impl:={impl}: pick one of '
                         f'{", ".join(ARM_WHEN_OPEN_IMPLS)}')
    return name


# Which lane_finder finds the lane lines (lane_finder.launch.py, and every profile that starts
# it: lane:=true, lane_pose:=true, the car's lane part; lane_finder_impl:=): rust
# (aav_lane_finder_rs, the default since Ben, 18 Nov: "it helps the car stay on the correct
# side"; the same node name, topics, parameters and lanes, no OpenCV) or python
# (aav_lane_finder, the fallback); or learned (aav_lane_finder_learned, a challenger, TODO
# 262b: our UFLD-v2-style model through ONNX Runtime, the same message; needs
# models/lane-ufld.onnx). Each one's package.
LANE_FINDER_IMPLS = {'python': 'aav_lane_finder', 'rust': 'aav_lane_finder_rs',
                     'learned': 'aav_lane_finder_learned'}
DEFAULT_LANE_FINDER_IMPL = 'rust'


def check_lane_finder_impl(impl):
    """Return lane_finder's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in LANE_FINDER_IMPLS:
        raise ValueError(f'lane_finder_impl:={impl}: pick one of '
                         f'{", ".join(LANE_FINDER_IMPLS)}')
    return name


# Which camera_stream pulls each camera Pi stream (camera_stream.launch.py, the car's
# camera_stream part; camera_stream_impl:=): rust (aav_camera_stream_rs, the default since
# Ben, 18 Nov: "lets port them to rust"; the same node name, topics, parameters and
# diagnostics) or python (aav_camera_stream, the fallback). The Rust one runs no model: with
# a detector (rfdetr, yolo) or decoder:=opencv, the Python one runs (camera_stream_runs).
CAMERA_STREAM_IMPLS = {'python': 'aav_camera_stream', 'rust': 'aav_camera_stream_rs'}
DEFAULT_CAMERA_STREAM_IMPL = 'rust'


def check_camera_stream_impl(impl):
    """Return camera_stream's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in CAMERA_STREAM_IMPLS:
        raise ValueError(f'camera_stream_impl:={impl}: pick one of '
                         f'{", ".join(CAMERA_STREAM_IMPLS)}')
    return name


def camera_stream_runs(impl, detector, decoder):
    """
    Return (the language that runs, why it isn't the one asked for, or '').

    The Rust camera_stream has no model and no in-process OpenCV decoder: with a detector
    or decoder:=opencv, the Python one runs instead.
    """
    name = check_camera_stream_impl(impl)
    if name == 'python':
        return name, ''
    if str(detector).strip().lower() != 'none':
        return 'python', (f'detector:={detector} runs inside the Python camera_stream '
                          '(the Rust one has no model)')
    if str(decoder).strip() == 'opencv':
        return 'python', 'decoder:=opencv decodes inside the Python camera_stream'
    return name, ''


# Which joy_alive runs (teleop.launch.py, car.launch.py; joy_alive_impl:=): rust
# (aav_joy_alive_rs, the default since Ben, 17 Nov: "I don't think we want to do this again
# later"). The Python went on 20 Nov (tag save/python-safety-nodes).
JOY_ALIVE_IMPLS = {'rust': 'aav_joy_alive_rs'}
DEFAULT_JOY_ALIVE_IMPL = 'rust'


def check_joy_alive_impl(impl):
    """Return joy_alive's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in JOY_ALIVE_IMPLS:
        raise ValueError(f'joy_alive_impl:={impl}: pick one of {", ".join(JOY_ALIVE_IMPLS)}')
    return name


# Which geofence, stop_hold and goal_leash run (geofence_impl:=, stop_hold_impl:=,
# goal_leash_impl:=): rust (aav_safety_rs, the defaults since 19 Nov: Ben, 16 Nov, "lets
# remove python were possible"; the Python's answers event by event and bit by bit). The
# Python nodes went on 20 Nov (tag save/python-safety-nodes); the switches stay, hooks.
# geofence.launch.py, navigation.launch.py, car, sim and bench pass them on.
SAFETY_IMPLS = {'rust': 'aav_safety_rs'}
GEOFENCE_IMPLS = STOP_HOLD_IMPLS = GOAL_LEASH_IMPLS = SAFETY_IMPLS
DEFAULT_GEOFENCE_IMPL = 'rust'
DEFAULT_STOP_HOLD_IMPL = 'rust'
DEFAULT_GOAL_LEASH_IMPL = 'rust'


def _check_safety_impl(argument, impl):
    name = str(impl).strip().lower()
    if name not in SAFETY_IMPLS:
        raise ValueError(f'{argument}:={impl}: pick one of {", ".join(SAFETY_IMPLS)}')
    return name


def check_geofence_impl(impl):
    """Return the geofence's language (lower case), or raise ValueError: what to pick."""
    return _check_safety_impl('geofence_impl', impl)


def check_stop_hold_impl(impl):
    """Return stop_hold's language (lower case), or raise ValueError: what to pick."""
    return _check_safety_impl('stop_hold_impl', impl)


def check_goal_leash_impl(impl):
    """Return goal_leash's language (lower case), or raise ValueError: what to pick."""
    return _check_safety_impl('goal_leash_impl', impl)


# Which gps_heading runs (localization.launch.py, gps_heading_impl:=): rust
# (aav_gps_heading_rs, the default since 19 Nov: course.py's answers to the bit). The
# Python went on 20 Nov (tag save/python-gps-heading); the switch stays, a challenger's hook.
GPS_HEADING_IMPLS = {'rust': 'aav_gps_heading_rs'}
DEFAULT_GPS_HEADING_IMPL = 'rust'


def check_gps_heading_impl(impl):
    """Return gps_heading's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in GPS_HEADING_IMPLS:
        raise ValueError(f'gps_heading_impl:={impl}: pick one of {", ".join(GPS_HEADING_IMPLS)}')
    return name


# Which stop_sign_watcher runs (car.launch.py's stop_signs, sim.launch.py and
# bench.launch.py with perception; stop_sign_impl:=): rust (aav_stop_sign_rs, the default
# since Ben, 17 Nov) or python (aav_detector's: the same node name, topics, parameters and
# answers). Each one's package.
STOP_SIGN_IMPLS = {'python': 'aav_detector', 'rust': 'aav_stop_sign_rs'}
DEFAULT_STOP_SIGN_IMPL = 'rust'


def check_stop_sign_impl(impl):
    """Return stop_sign_watcher's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in STOP_SIGN_IMPLS:
        raise ValueError(f'stop_sign_impl:={impl}: pick one of {", ".join(STOP_SIGN_IMPLS)}')
    return name


# Which localization_health runs (localization.launch.py, car.launch.py, sim.launch.py
# localization_health_impl:=): rust (aav_localization_health_rs, the default on the car and
# in the simulator, Ben, 16 Nov: the same answers at 1.5 % of a core against 9.4 %). The
# Python went on 20 Nov (tag save/python-localization-health); the switch stays, a hook.
# Warn only, so no safety lock.
LOCALIZATION_HEALTH_IMPLS = {'rust': 'aav_localization_health_rs'}
DEFAULT_LOCALIZATION_HEALTH_IMPL = 'rust'
# The simulator's default: the car's since 16 Nov (the name stays for sim.launch.py).
SIM_DEFAULT_LOCALIZATION_HEALTH_IMPL = DEFAULT_LOCALIZATION_HEALTH_IMPL


def check_localization_health_impl(impl):
    """Return localization_health's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in LOCALIZATION_HEALTH_IMPLS:
        raise ValueError(f'localization_health_impl:={impl}: pick one of '
                         f'{", ".join(LOCALIZATION_HEALTH_IMPLS)}')
    return name


def check_behaviour_impl(impl):
    """Return the behaviour node's language (lower case), or raise ValueError: what to pick."""
    name = str(impl).strip().lower()
    if name not in BEHAVIOUR_IMPLS:
        raise ValueError(f'behaviour_impl:={impl}: pick one of {", ".join(BEHAVIOUR_IMPLS)}')
    return name


# The spine's speed loop profile the car runs (sim.launch.py and bench.launch.py
# speed_loop:=): spine-core's SpeedLoopProfile, car_model.SPEED_LOOP_PROFILES. Every node
# with a model of the car predicts with the one the simulated car runs (TODO 163): ego's
# trace (trace_speed_loop), the behaviour planner's stopping law, aav_track's mpc_lag and
# aav_mpc_lag (mpc_lag.speed_loop), mppi_lag (speed_loop). The car's spine runs default.
DEFAULT_SPEED_LOOP = 'default'
# Trackers whose model has only the PI's law: they can't predict the LQI.
PI_ONLY_TRACKERS = ('mpc_lag', 'mpc_lag_rs')


def check_speed_loop(name, tracker=None):
    """
    Return a speed loop profile's name (lower case), or raise ValueError saying what to pick.

    tracker: the path follower that will predict it; mpc_lag and mpc_lag_rs refuse the LQI.
    """
    profile = str(name).strip().lower()
    if profile not in SPEED_LOOP_PROFILES:
        raise ValueError(f'speed_loop:={name}: pick one of {", ".join(SPEED_LOOP_PROFILES)}')
    if tracker in PI_ONLY_TRACKERS and SPEED_LOOP_PROFILES[profile].lqi is not None:
        raise ValueError(f"speed_loop:={profile}: tracker:={tracker} models the PI's "
                         'profiles only (default, smooth, firm)')
    return profile


def check_speed_loop_model(name, car_loop, tracker=None):
    """
    Return the profile every model of the car assumes (speed_loop_model:=), or raise ValueError.

    Empty: the car's own loop (car_loop), so the models predict the car they drive. Given:
    the models plan with it while the car runs car_loop (TODO 163: car firm, models smooth).
    tracker: as check_speed_loop.
    """
    if not str(name).strip():
        return check_speed_loop(car_loop, tracker)
    try:
        return check_speed_loop(name, tracker)
    except ValueError as err:
        raise ValueError(str(err).replace('speed_loop:=', 'speed_loop_model:=', 1)) from None


def smoother_params(cfg, ego, vehicle_yaml):
    """
    Return car_smoother's car parameters (smoother:=own), beside nav2.yaml's own block.

    cfg: vehicle.yaml (load()); ego: ego.yaml's ros__parameters; vehicle_yaml: its path,
    which the node reads through the car model. The steering rate is the tighter of the
    steering actuator's (actuators.steer_rate) and ego's ramp, as track_params takes it.
    """
    rate = float(cfg['actuators']['steer_rate'])
    if ego and 'max_steering_rate' in ego:
        rate = min(rate, float(ego['max_steering_rate']))
    return {'vehicle_yaml': str(vehicle_yaml), 'steering_rate': rate,
            'min_speed_for_steering': float((ego or {}).get('min_speed_for_steering', 0.05))}


def navigator_action(action):
    """
    Return the action's full name, the same whichever navigator runs: '/navigate_to_pose'.

    action: one of NAVIGATOR_ACTIONS.
    """
    if action not in NAVIGATOR_ACTIONS:
        raise ValueError(f'{action}: one of {", ".join(NAVIGATOR_ACTIONS)}')
    return '/' + action


def navigator_params(v, reversing, waypoints='pass_through', replan_hz=None):
    """
    Return aav_navigator's settings (navigator:=own) as a dict, for its JSON parameter.

    The trees' numbers (behavior_trees/*_car*.xml): replanning at 1 Hz, but a to-pose goal
    keeps a valid path up to 10 s (keep_path_age: the trees since 14 Nov, TODO 149; here
    since 15 Nov, TODO 168) (through-poses 0.333 Hz), the final approach
    (final_approach(v), TODO 17), 6 recoveries, Wait 5 s,
    BackUp 1 m at half max_reverse_speed only with reversing, RemovePassedGoals' 1 m.
    waypoints: pass_through (ours: one through-poses goal, TODO 140) or stop_at_each
    (Nav2's waypoint_follower). The time limits, so a server that dies mid-goal fails it:
    plan_timeout, twice nav2.yaml's max_planning_time (5 s); follow_timeout, the
    follower's silence, Nav2's bond_timeout (4 s); recovery_timeout, a costmap clear.
    """
    hz = REPLAN_HZ if replan_hz is None else float(replan_hz)
    if hz < 0.0:
        raise ValueError(f'replan_hz:={hz:g}: 0 (plan once) or more')
    out = {'replan_period': 1.0 / (hz if hz > 0.0 else REPLAN_ONCE_HZ),
           'through_replan_period': 3.0,
           'final_approach': round(final_approach(v), 3), 'path_end_tolerance': 0.5,
           'path_max_age': 20.0, 'keep_path_age': 10.0, 'retries': 6, 'wait': 5.0,
           'pass_radius': 1.0, 'on_path': 2.0, 'waypoints': waypoints, 'stop_on_failure': False,
           'replan_failures': 3, 'backup': None,
           'plan_timeout': 10.0, 'follow_timeout': 4.0, 'recovery_timeout': 5.0}
    if reversing:
        out['backup'] = {'distance': 1.0, 'speed': 0.5 * float(v['max_reverse_speed'])}
    return out


# The planners (planner:=, docs/own-planner.md). own (aav_planner): Nav2's State Lattice
# (and its Hybrid-A*, planner_search:=hybrid) ported to Rust, on the same primitives file
# and penalties: the only one on the car. diffusion (aav_diffusion_planner, a trial:
# docs/designs/2026-11-02_diffusion-planner.md): a model draws the path and a guard checks
# it, our planner behind it as the fallback (TRIAL_PLANNERS: the block each plans from).
# Nav2's planner_server and its planners (SmacPlannerHybrid, SmacPlannerLattice, Smac2D,
# NavFn, Theta*) went with Nav2 on 17 Nov (TODO 184).
PLANNERS = ('own', 'diffusion')
CAR_PLANNERS = ('own',)
TRIAL_PLANNERS = {'diffusion': 'lattice', 'own': 'lattice'}
# Our planner's action; the navigator's (and the diffusion planner's fallback) calls it.
OWN_PLANNER_ACTION = '/drivers/nav/planner/compute_path_to_pose'
# ... and its through-poses action (the navigator's).
OWN_PLANNER_THROUGH_ACTION = '/drivers/nav/planner/compute_path_through_poses'
# Ours, forward or reversing (its Hybrid-A* is planner_search:=hybrid).
DEFAULT_PLANNER = 'own'
FORWARD_ONLY_PLANNER = 'own'
# Each planner's block in nav2.yaml's planner_server (GridBased is hybrid's own): the
# file keeps Nav2's layout, so our ports read the blocks they were tuned with.
PLANNER_BLOCKS = {'hybrid': 'GridBased', 'lattice': 'Lattice', 'diffusion': 'Lattice',
                  'own': 'Lattice'}


def server_planner(planner):
    """Return nav2.yaml's planner block planner:= plans with: the lattice's (ours ported)."""
    return TRIAL_PLANNERS.get(planner, planner)


# The lattice's turning radius against plan_turning_radius: the file keeps 3 decimals.
LATTICE_TOLERANCE = 0.005
# The lattice primitives files (lattice_headings:=): 16 headings, the default, and 32, a
# trial (finer headings, more moves; tools/lattice_primitives.py --headings 32).
LATTICE_HEADINGS = (16, 32)
# The lattice's moves (lattice_moves:=): nav2, Nav2's generator's (straights and arcs, the
# default); smooth and smooth_fast, trials: curvature that changes no faster than the
# steering can turn at 1 m/s (ego's top speed) and at 2.5 m/s (the campus tours'), each
# move straight at both ends (tools/lattice_smooth.py; 16 headings only).
LATTICE_MOVES = ('nav2', 'smooth', 'smooth_fast')


# Our planner's search (planner_search:=, with planner:=own; docs/own-planner.md): lattice,
# Nav2's State Lattice ported (the default, the league's lattice_rs); hybrid, Nav2's
# Hybrid-A* ported (SmacPlannerHybrid, a challenger: hybrid_rs), on nav2.yaml's GridBased
# block. Both in aav_planner, sharing the search, the checks, the estimates and the smoother.
PLANNER_SEARCHES = ('lattice', 'hybrid')


def check_planner_search(search, planner='own'):
    """Return planner_search:= checked (empty: lattice), or raise ValueError saying why."""
    name = str(search).strip().lower() or PLANNER_SEARCHES[0]
    if name not in PLANNER_SEARCHES:
        raise ValueError(f'planner_search:={search}: pick one of {", ".join(PLANNER_SEARCHES)}')
    if name != PLANNER_SEARCHES[0] and planner != 'own':
        raise ValueError(f"planner_search:={name} is our planner's (planner:=own), "
                         f"not planner:={planner}'s")
    return name


def check_lattice_moves(moves):
    """Return lattice_moves:= checked (empty: nav2), or raise ValueError saying what to pick."""
    name = str(moves).strip().lower() or LATTICE_MOVES[0]
    if name not in LATTICE_MOVES:
        raise ValueError(f'lattice_moves:={moves}: pick one of {", ".join(LATTICE_MOVES)}')
    return name


def smooth_moves(moves):
    """Return whether lattice_moves:= is one of the smooth trials (not Nav2's moves)."""
    return check_lattice_moves(moves) != LATTICE_MOVES[0]


def lattice_file_name(headings=LATTICE_HEADINGS[0], moves=LATTICE_MOVES[0]):
    """Return the primitives file's name in aav_navigation's config/: headings, moves."""
    headings = int(headings)
    if headings not in LATTICE_HEADINGS:
        raise ValueError(f'lattice_headings:={headings}: pick one of '
                         f'{", ".join(str(h) for h in LATTICE_HEADINGS)}')
    moves = check_lattice_moves(moves)
    if smooth_moves(moves):
        if headings != LATTICE_HEADINGS[0]:
            raise ValueError(f'lattice_moves:={moves} has {LATTICE_HEADINGS[0]} headings only, '
                             f'not lattice_headings:={headings}')
        return f'lattice_ackermann_{moves}.json'
    tail = '' if headings == LATTICE_HEADINGS[0] else f'_{headings}'
    return f'lattice_ackermann{tail}.json'


def default_planner(reversing):
    """Return the planner to use when none is asked for: the lattice only if Nav2 may reverse."""
    return DEFAULT_PLANNER if reversing else FORWARD_ONLY_PLANNER


# The diffusion planner's car trial (Ben, 7 Nov: "We want diffusion", in 0.3, low speed,
# behind the guard and the locks). Only with car.launch.py diffusion_trial:=true, and then
# ego and Nav2 are held to DIFFUSION_TRIAL_SPEED at most. 0.5 m/s: walking pace, the car's
# own reverse limit (max_reverse_speed), so forwards and backwards are alike; at ego's
# 2 m/s^2 the car stops in 0.06 m, plus 0.15 m for a 0.3 s stale command. Half of ego's
# 1.0 m/s; never above it. The planner only draws the path: the guard (G1-G8) checks each,
# the lattice is the fallback, and MPPI, collision_monitor, twist_mux's locks, ego, the
# geofence and the spine's watchdog are the same as for the lattice.
TRIAL_CAR_PLANNERS = ('diffusion',)
DIFFUSION_TRIAL_SPEED = 0.5


def diffusion_trial_speed(max_speed):
    """
    Return the diffusion car trial's top speed (m/s) from car.launch.py's max_speed:=.

    Empty: DIFFUSION_TRIAL_SPEED. Above it, at or under 0, or not a number: ValueError.
    """
    text = str(max_speed).strip()
    if not text:
        return DIFFUSION_TRIAL_SPEED
    try:
        speed = float(text)
    except ValueError:
        raise ValueError(f'max_speed:={text}: a number of m/s, at most '
                         f'{DIFFUSION_TRIAL_SPEED}') from None
    if not 0.0 < speed <= DIFFUSION_TRIAL_SPEED:
        raise ValueError(f'max_speed:={text}: the diffusion trial drives at most '
                         f'{DIFFUSION_TRIAL_SPEED} m/s on the car (above 0); leave it empty '
                         f'for {DIFFUSION_TRIAL_SPEED}')
    return speed


def check_planner(planner, real=False, trial=False):
    """
    Return the planner's name (lower case), or raise ValueError saying what to pick.

    real: the launch drives the real car (data source REAL, car.launch.py). There only
    CAR_PLANNERS, whose paths keep to the turning radius: a corner from a grid planner is
    a path the car can't drive, and the safest answer is not to offer it at all.
    trial: car.launch.py diffusion_trial:=true, which lets TRIAL_CAR_PLANNERS (the
    diffusion planner, with the guard and the lattice behind it) on the car too, and only
    those; car.launch.py then holds the speed to DIFFUSION_TRIAL_SPEED.
    """
    name = str(planner).strip().lower()
    if name not in PLANNERS:
        raise ValueError(f'planner:={planner}: pick one of {", ".join(PLANNERS)}')
    if trial and name not in TRIAL_CAR_PLANNERS:
        raise ValueError(f'diffusion_trial:=true is for planner:=diffusion only, not '
                         f'planner:={name}')
    if real and name in TRIAL_CAR_PLANNERS and not trial:
        raise ValueError(f'planner:={name} is a trial: on the car only with '
                         f'diffusion_trial:=true (and at most {DIFFUSION_TRIAL_SPEED} m/s). '
                         f'Otherwise: {", ".join(CAR_PLANNERS)}')
    if real and name not in CAR_PLANNERS + (TRIAL_CAR_PLANNERS if trial else ()):
        raise ValueError(f"planner:={name} ignores the car's turning radius: a trial for "
                         f'sim and bench only. On the car: {", ".join(CAR_PLANNERS)}')
    return name


def lattice_problems(meta, v, resolution):
    """
    Return what is wrong with a lattice primitives file for this car and map (empty: fine).

    meta: the file's lattice_metadata; v: vehicle.yaml's vehicle; resolution: the map's
    cells (m), which the global costmap takes. Nav2 measures the primitives in cells of
    the file's own size, so another map's cells would stretch or shrink every move.
    """
    out = []
    if meta.get('motion_model') != 'ackermann':
        out.append(f"motion_model {meta.get('motion_model')}, not ackermann")
    want = round(plan_turning_radius(v), 3)
    if abs(float(meta.get('turning_radius', 0.0)) - want) > LATTICE_TOLERANCE:
        out.append(f"turning radius {meta.get('turning_radius')} m, but the car plans with "
                   f'{want} m (plan_turning_radius)')
    if abs(float(meta.get('grid_resolution', 0.0)) - float(resolution)) > 1e-9:
        out.append(f"cells of {meta.get('grid_resolution')} m, but the map's are "
                   f'{resolution} m')
    return out


def track_params(nav2, cfg, ego=None, max_speed=None, speed_loop=DEFAULT_SPEED_LOOP,
                 reversing=False):
    """
    Return aav_track's parameters: the same speed and "done" as Nav2's controllers.

    nav2: nav2.yaml as loaded (not tuned: nav2_tuned puts MPPI's block under FollowPath);
    cfg: vehicle.yaml (load()); ego: ego.yaml's ros__parameters, for its steering rate.
    The cruise speed is RPP's desired_linear_vel, or `max_speed` (sim_speed:=); the goal
    and progress checks are controller_server's; speeding up and slowing down are the
    velocity smoother's. `speed_loop`: the profile mpc_lag predicts (check_speed_loop).
    `reversing`: it may back up, at vehicle.yaml's max_reverse_speed (0: forward only).
    """
    server = nav2['controller_server']['ros__parameters']
    goal = server[server['goal_checker_plugins'][0]]
    progress = server[server['progress_checker_plugins'][0]]
    rpp = server['FollowPath']
    smoother = nav2['velocity_smoother']['ros__parameters']
    v, act = cfg['vehicle'], cfg['actuators']
    rate = float(act['steer_rate'])
    if ego and 'max_steering_rate' in ego:
        # ego's own ramp is the tighter one today (0.6 against the motor's 0.8 rad/s).
        rate = min(rate, float(ego['max_steering_rate']))
    return {
        'rate_hz': float(server['controller_frequency']),
        'odom_topic': server['odom_topic'],
        'cruise_speed': float(max_speed) if max_speed else float(rpp['desired_linear_vel']),
        'acceleration': float(smoother['max_accel'][0]),
        'deceleration': abs(float(smoother['max_decel'][0])),
        'max_brake': float(act['max_brake']),
        'lookahead_min': float(rpp['min_lookahead_dist']),
        'lookahead_max': float(rpp['max_lookahead_dist']),
        'lookahead_time': float(rpp['lookahead_time']),
        'transform_tolerance': float(rpp['transform_tolerance']),
        'xy_goal_tolerance': float(goal['xy_goal_tolerance']),
        'yaw_goal_tolerance': float(goal['yaw_goal_tolerance']),
        'progress_radius': float(progress['required_movement_radius']),
        'progress_angle': float(progress.get('required_movement_angle', math.inf)),
        'progress_time': float(progress['movement_time_allowance']),
        'wheelbase': float(v['wheelbase']),
        'max_steering_angle': float(v['max_steering_angle']),
        'max_steering_rate': rate,
        'stamped': bool(server.get('enable_stamped_cmd_vel', False)),
        'mpc_lag.speed_loop': check_speed_loop(speed_loop),
        'max_reverse_speed': -reverse_limit(v, reversing),
    }


# vehicle.yaml's actuator keys aav_mppi_lag's lag model takes (as actuators.<key>). The dead
# times are optional there: 0 when unset.
MPPI_LAG_ACTUATORS = ('max_accel', 'throttle_deadband', 'throttle_lag', 'max_brake',
                      'brake_lag', 'drag', 'steer_rate', 'steer_lag', 'steer_offset',
                      'steer_deadband', 'throttle_dead_time', 'brake_dead_time',
                      'steer_dead_time')
# aav_track's parameters mpc_lag_rs (aav_mpc_lag) doesn't take: pure pursuit's lookahead.
MPC_LAG_RS_UNUSED = ('lookahead_min', 'lookahead_max', 'lookahead_time')


def mpc_lag_rs_params(nav2, cfg, ego=None, max_speed=None, speed_loop=DEFAULT_SPEED_LOOP,
                      reversing=False):
    """
    Return aav_mpc_lag's parameters: aav_track's (track_params), and the car's actuators.

    The same speed, checks and steering limits as the Python mpc_lag gets, so the two race
    on the same terms; vehicle.yaml's actuators as actuators.<key> (MPPI_LAG_ACTUATORS, the
    dead times 0 when unset), where aav_track reads the file itself. `reversing`: it may
    back up, at vehicle.yaml's max_reverse_speed (0: forward only; TODO 213).
    """
    params = {k: v for k, v in track_params(nav2, cfg, ego, max_speed, speed_loop,
                                            reversing=reversing).items()
              if k not in MPC_LAG_RS_UNUSED}
    act = cfg['actuators']
    params.update({f'actuators.{k}': float(act.get(k, 0.0)) for k in MPPI_LAG_ACTUATORS})
    return params


# nav2.yaml's MPPI settings it takes under the same name, and their types.
MPPI_LAG_SAME = {'time_steps': int, 'model_dt': float, 'batch_size': int,
                 'iteration_count': int, 'temperature': float, 'gamma': float,
                 'vx_std': float, 'transform_tolerance': float, 'retry_attempt_limit': int}
# The road ahead (aav_interfaces/RoadAhead, the behaviour planner's), which mppi_lag reads
# for its LaneCritic when the lane map gives it edges.
ROAD_AHEAD_TOPIC = '/world/road_ahead'
# LaneCritic's settings (ours; aav_mppi_lag critics.rs, docs/mppi-lag.md): per m² past a
# lane line, per pose past a kerb (plus per m), every 2nd pose checked.
LANE_CRITIC = {'soft_weight': 100.0, 'hard_weight': 1000.0, 'cost_power': 1,
               'trajectory_point_step': 2}
# ego.yaml's limits it models in front of the car (as ego.<key>).
MPPI_LAG_EGO = ('max_acceleration', 'max_deceleration', 'max_steering_rate',
                'min_speed_for_steering')


def critic_value(key, value):
    """Return a critic setting as its node declares it: cost_power whole, other numbers float."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return value
    return int(value) if key in ('cost_power', 'offset_from_furthest',
                                 'trajectory_point_step', 'mode') else float(value)


def costmap_timeout(server, costmap):
    """
    Return the s after which a costmap is too old to act on, for a node outside Nav2.

    server: planner_server's or controller_server's ros__parameters (their
    costmap_update_timeout, Nav2's defaults 1.0 and 0.3 s); costmap: its costmap's
    (publish_frequency): the reader sees a new costmap only once a publish period.
    """
    wait = float(server.get('costmap_update_timeout', 0.3))
    return round(wait + 1.0 / float(costmap.get('publish_frequency', 1.0)), 3)


def mppi_lag_params(nav2, cfg, ego=None, max_speed=None, reversing=False, lane=False,
                    speed_loop=DEFAULT_SPEED_LOOP):
    """
    Return aav_mppi_lag's parameters: Nav2's MPPI block, its checks, and the car's lag.

    nav2: nav2.yaml as loaded; cfg: vehicle.yaml (load()); ego: ego.yaml's
    ros__parameters. MPPI's settings and critics are nav2.yaml's own (the MPPI block, as
    controller:=mppi runs it), the goal and progress checks controller_server's, the
    outline and inflation the local costmap's, the actuators vehicle.yaml's, the ramps
    ego's. `max_speed` (sim_speed:=) sets vx_max and stretches prune_distance as
    nav2_tuned does for Nav2's MPPI. `reversing` sets it up as nav2_tuned sets Nav2's
    MPPI: vx_min -max_reverse_speed (else 0), the path followed stretch by stretch at its
    cusps (enforce_path_inversion, nav2.yaml's tolerances), PathAngleCritic in mode 2,
    and no PreferForwardCritic.
    `lane` (behaviour:=true with a lane map: the road ahead has edges) reads
    `ROAD_AHEAD_TOPIC` and adds our LaneCritic (`LANE_CRITIC`) to the critics; without it
    the topic is empty and the critics are Nav2's, as before.
    `speed_loop`: the spine's profile its model predicts (check_speed_loop).
    """
    server = nav2['controller_server']['ros__parameters']
    mppi = server['MPPI']
    goal = server[server['goal_checker_plugins'][0]]
    progress = server[server['progress_checker_plugins'][0]]
    local = nav2['local_costmap']['local_costmap']['ros__parameters']
    v, act = cfg['vehicle'], cfg['actuators']
    speed = float(max_speed) if max_speed else float(mppi['vx_max'])
    ahead = float(mppi['time_steps']) * float(mppi['model_dt']) * speed
    out = {name: kind(mppi[name]) for name, kind in MPPI_LAG_SAME.items() if name in mppi}
    critics = list(mppi['critics'])
    if reversing:
        # As nav2_tuned's: the planner decides when to reverse (reverse_penalty).
        critics = [c for c in critics if c != 'PreferForwardCritic']
    if lane:
        critics.append('LaneCritic')
    out.update({
        'rate_hz': float(server['controller_frequency']),
        'odom_topic': server['odom_topic'],
        'vx_max': speed,
        'vx_min': reverse_limit(v, reversing),
        'enforce_path_inversion': bool(reversing),
        'inversion_xy_tolerance': float(mppi.get('inversion_xy_tolerance', 0.2)),
        'inversion_yaw_tolerance': float(mppi.get('inversion_yaw_tolerance', 0.4)),
        'prune_distance': max(float(mppi.get('prune_distance', 1.5)), ahead * 1.2),
        'critics': critics,
        'road_ahead_topic': ROAD_AHEAD_TOPIC if lane else '',
        'xy_goal_tolerance': float(goal['xy_goal_tolerance']),
        'yaw_goal_tolerance': float(goal['yaw_goal_tolerance']),
        'progress_radius': float(progress['required_movement_radius']),
        'progress_angle': float(progress.get('required_movement_angle', math.inf)),
        'progress_time': float(progress['movement_time_allowance']),
        'wheelbase': float(v['wheelbase']),
        'max_steering_angle': float(v['max_steering_angle']),
        'footprint': [float(c) for xy in footprint_points(v) for c in xy],
        'footprint_padding': float(local.get('footprint_padding', 0.0)),
        'inflation_radius': float(local['inflation_layer']['inflation_radius']),
        'cost_scaling_factor': float(local['inflation_layer']['cost_scaling_factor']),
        'track_unknown_space': bool(local.get('track_unknown_space', False)),
        # s: a costmap older than this stops the car and fails the path. Nav2's
        # controller waits costmap_update_timeout for a current one; ours sees the
        # costmap only when it is published, so one publish period more.
        'costmap_timeout': costmap_timeout(server, local),
        'speed_loop': check_speed_loop(speed_loop),
    })
    for key in MPPI_LAG_ACTUATORS:
        out[f'actuators.{key}'] = float(act.get(key, 0.0))
    for key in MPPI_LAG_EGO:
        if ego and key in ego:
            out[f'ego.{key}'] = float(ego[key])
    # Each critic's own settings, under its name, as Nav2 reads them.
    for critic in critics:
        for key, value in (mppi.get(critic) or {}).items():
            out[f'{critic}.{key}'] = critic_value(key, value)
    if lane:
        out.update({f'LaneCritic.{k}': v for k, v in LANE_CRITIC.items()})
    if reversing and 'PathAngleCritic' in critics:
        # Mode 2, as nav2_tuned's: follow the path the way it faces, backwards too.
        out['PathAngleCritic.mode'] = 2
    return out


# Nav2's own defaults for what nav2.yaml leaves out (costmap_2d's ObstacleLayer and
# ObservationBuffer, Jazzy), so our costmap reads a block as Nav2 does.
COSTMAP_SOURCE_DEFAULTS = {
    'topic': '', 'data_type': 'LaserScan', 'marking': True, 'clearing': False,
    'min_obstacle_height': 0.0, 'max_obstacle_height': 2.0,
    'obstacle_min_range': 0.0, 'obstacle_max_range': 2.5,
    'raytrace_min_range': 0.0, 'raytrace_max_range': 3.0,
    'inf_is_valid': False, 'observation_persistence': 0.0, 'view_angle': 0.0,
}


# The Lattice block's keys our planner reads (aav_planner's lattice-planner Settings).
OWN_PLANNER_KEYS = (
    'lattice_filepath', 'allow_reverse_expansion', 'allow_unknown', 'tolerance',
    'max_iterations', 'max_on_approach_iterations', 'max_planning_time',
    'terminal_checking_interval', 'reverse_penalty', 'change_penalty', 'cusp_penalty',
    'non_straight_penalty', 'cost_penalty', 'retrospective_penalty', 'rotation_penalty',
    'analytic_expansion_ratio', 'analytic_expansion_max_length',
    'analytic_expansion_max_cost', 'analytic_expansion_max_cost_override',
    'lookup_table_size', 'smooth_path', 'smoother')


# The Hybrid-A* block's keys our planner reads (planner_search:=hybrid), besides the shared
# ones in OWN_PLANNER_KEYS; motion_model_for_search and minimum_turning_radius are written.
OWN_HYBRID_KEYS = ('angle_quantization_bins', 'allow_primitive_interpolation',
                   'downsample_obstacle_heuristic')


def planner_params(tuned, v, costmap='own', search='lattice', nav2=None):
    """
    Return aav_planner's settings (planner:=own) as a dict, from the tuned nav2.yaml.

    tuned: nav2.yaml after nav2_tuned with planner:=own (the Lattice block, the file and
    reversing filled in, is planner_server's GridBased); v: vehicle.yaml's `vehicle`;
    costmap: which global costmap it plans on (nav2 or own). The same block, the car's
    outline and padding, and the global costmap's inflation, so ours plans as Nav2's does.
    search (PLANNER_SEARCHES): hybrid plans with Hybrid-A* on nav2.yaml's own Hybrid-A*
    block (`nav2`, the file as loaded: nav2_tuned keeps only the lattice's), as
    nav2_rewrites writes it for planner:=hybrid: the planning radius
    (plan_turning_radius), and REEDS_SHEPP when the lattice may reverse, else DUBIN; and
    the global costmap's cells, which its moves are made for.
    """
    search = check_planner_search(search)
    block = tuned['planner_server']['ros__parameters']['GridBased']
    gc = tuned['global_costmap']['global_costmap']['ros__parameters']
    if search == 'hybrid':
        if nav2 is None:
            raise ValueError('planner_search:=hybrid needs nav2.yaml (nav2) for its block')
        reversing = bool(block.get('allow_reverse_expansion', False))
        hybrid = nav2['planner_server']['ros__parameters'][PLANNER_BLOCKS['hybrid']]
        out = {k: hybrid[k] for k in OWN_PLANNER_KEYS + OWN_HYBRID_KEYS if k in hybrid}
        out['search'] = 'hybrid'
        out['motion_model_for_search'] = 'REEDS_SHEPP' if reversing else 'DUBIN'
        out['minimum_turning_radius'] = round(plan_turning_radius(v), 3)
        out['resolution'] = float(gc.get('resolution', 0.05))
    else:
        out = {k: block[k] for k in OWN_PLANNER_KEYS if k in block}
    out['footprint'] = footprint_points(v)
    out['footprint_padding'] = float(gc.get('footprint_padding', 0.01))
    out['costmap_timeout'] = costmap_timeout(tuned['planner_server']['ros__parameters'], gc)
    out['inflation'] = None
    for plugin in gc['plugins']:
        layer = gc.get(plugin, {})
        if (layer.get('plugin', '').endswith('InflationLayer') and
                layer.get('enabled', True)):
            out['inflation'] = {'radius': float(layer.get('inflation_radius', 0.55)),
                                'scaling': float(layer.get('cost_scaling_factor', 10.0))}
    return out


# planner:=own: each global costmap's raw topic, its partial updates, and whether it is
# latched (Nav2 sends the whole map once, then only the changed parts).
GLOBAL_COSTMAP_TOPICS = {
    'own': ('/drivers/nav/costmap/global_raw', '', False),
}


def costmap_params(tuned, v, radar=False, camera=False):
    """
    Return aav_costmap's two blocks, {'local': {...}, 'global': {...}}, from nav2.yaml.

    tuned: nav2.yaml after nav2_tuned (its layers as Nav2 runs them); v: vehicle.yaml's
    `vehicle`. radar and camera turn on the local costmap's obstacle_layer and
    camera_layer, as the launch file's rewrites do for Nav2. Each block holds only what
    our costmap reads, in plain types, with Nav2's defaults filled in: the frame, the
    window, the rates, the car's outline (vehicle.yaml's) and padding, the static map,
    the inflation, and each enabled obstacle layer with its sources.
    """
    enabled = {'obstacle_layer': radar, 'camera_layer': camera}
    out = {}
    for name in ('local', 'global'):
        block = tuned[f'{name}_costmap'][f'{name}_costmap']['ros__parameters']
        spec = {
            'global_frame': block.get('global_frame', 'map'),
            'rolling': bool(block.get('rolling_window', False)),
            'width': float(block.get('width', 5)),
            'height': float(block.get('height', 5)),
            'resolution': float(block.get('resolution', 0.05)),
            'track_unknown': bool(block.get('track_unknown_space', False)),
            'update_frequency': float(block.get('update_frequency', 5.0)),
            'publish_frequency': float(block.get('publish_frequency', 1.0)),
            'transform_tolerance': float(block.get('transform_tolerance', 0.3)),
            'footprint': footprint_points(v),
            'footprint_padding': float(block.get('footprint_padding', 0.01)),
            'static_layer': False, 'inflation': None, 'layers': [],
        }
        for plugin in block['plugins']:
            layer = block.get(plugin, {})
            kind = layer.get('plugin', '').split('::')[-1]
            if not enabled.get(plugin, layer.get('enabled', True)):
                continue
            if kind == 'StaticLayer':
                spec['static_layer'] = True
            elif kind == 'InflationLayer':
                spec['inflation'] = {'radius': float(layer.get('inflation_radius', 0.55)),
                                     'scaling': float(layer.get('cost_scaling_factor', 10.0))}
            elif kind in ('ObstacleLayer', 'VoxelLayer'):
                sources = []
                for source in str(layer.get('observation_sources', '')).split():
                    s = dict(COSTMAP_SOURCE_DEFAULTS, **layer.get(source, {}))
                    sources.append({'name': source, 'topic': s['topic'],
                                    'data_type': s['data_type'],
                                    'marking': bool(s['marking']),
                                    'clearing': bool(s['clearing']),
                                    'inf_is_valid': bool(s['inf_is_valid']),
                                    **{k: float(s[k]) for k in (
                                        'min_obstacle_height', 'max_obstacle_height',
                                        'obstacle_min_range', 'obstacle_max_range',
                                        'raytrace_min_range', 'raytrace_max_range',
                                        'observation_persistence', 'view_angle')}})
                spec['layers'].append({
                    'name': plugin,
                    'min_obstacle_height': float(layer.get('min_obstacle_height', 0.0)),
                    'max_obstacle_height': float(layer.get('max_obstacle_height', 2.0)),
                    'footprint_clearing': bool(layer.get('footprint_clearing_enabled', True)),
                    'sources': sources})
        out[name] = spec
    return out


def behaviour_params(nav2, cfg, ego=None, max_speed=None, controller=DEFAULT_CONTROLLER,
                     speed_loop=DEFAULT_SPEED_LOOP):
    """
    Return aav_behaviour_rs's parameters: the cap's top, the lane ahead, ego's limits.

    base_speed is the controller's own top speed (MPPI's vx_max, RPP's desired_linear_vel,
    or `max_speed`, sim_speed:=): a higher cap would raise MPPI's envelope. front and
    half_width are the body's front bumper and half its width plus the stop zone's margin;
    stop_gap is the stop zone's reach (STOP_ZONE_AHEAD): the obstacle's final stop.
    speed_loop: the spine's profile the stopping law's lag comes from (check_speed_loop).
    """
    server = nav2['controller_server']['ros__parameters']
    if max_speed:
        base = float(max_speed)
    elif controller == 'mppi':
        base = float(server['MPPI']['vx_max'])
    else:
        base = float(server['FollowPath']['desired_linear_vel'])
    v = cfg['vehicle']
    body = v['body']
    out = {'base_speed': base,
           'front': float(body['length'] - body['rear_overhang']),
           'half_width': float(body['width'] / 2.0 + STOP_ZONE_MARGIN),
           'stop_gap': float(STOP_ZONE_AHEAD),
           'speed_loop': check_speed_loop(speed_loop)}
    for key in ('max_deceleration', 'max_jerk'):
        if ego and key in ego:
            out[key] = float(ego[key])
    return out


# Stops reset the drift (TODO 191; the whole-car design, research idea 12): wheels that
# read nothing mean the car stands still, so its speed, sideways speed and turn rate are
# exactly 0: free information. The backends then send them nearly certain, and the EKFs'
# yaw stops creeping on the gyro's bias. Under STANDSTILL_SPEED the encoders can't tell
# motion from noise (one pulse per window).
STANDSTILL_SPEED = 0.01        # m/s
STANDSTILL_VARIANCE = 1e-6     # (m/s)², (rad/s)²: the zero is known, not guessed
TWIST_STILL = (0, 7, 35)       # vx, vy, vyaw on a 6 x 6 twist covariance's diagonal


def standstill_covariance(speed, covariance):
    """
    Return the wheels' twist covariance, nearly certain while they read standing still.

    speed: the wheels' speed (m/s); covariance: the 36 numbers the backend would send.
    Moving (or NaN), the list comes back unchanged.
    """
    out = list(covariance)
    if math.isfinite(speed) and abs(speed) < STANDSTILL_SPEED:
        for i in TWIST_STILL:
            out[i] = STANDSTILL_VARIANCE
    return out


LIDAR_MEMORY_SOURCES = ('lidar', 'voxels')    # the local lidar layer's marking sources


def with_lidar_memory(cfg, seconds):
    """
    Return nav2.yaml's dict with the local lidar layer remembering marks for `seconds`.

    observation_persistence on the lidar's marking sources (the scan, or the cubes): a box
    the roof lidar saw on the way in stays marked that long after it drops below the
    lowest beam, about 2.5 m from the bumper for a 0.5 m box (TODO 188, the stock
    blind-zone memory). 0: the newest scan only, as the file has it. Nav2's costmap and
    ours both read it.
    """
    if not (math.isfinite(seconds) and seconds >= 0.0):
        raise ValueError(f'lidar_memory must be 0 or more seconds (got {seconds})')
    layer = cfg['local_costmap']['local_costmap']['ros__parameters']['lidar_layer']
    for source in LIDAR_MEMORY_SOURCES:
        if source in layer:
            layer[source]['observation_persistence'] = float(seconds)
    return cfg


def stop_zone_params(tuned, car):
    """
    Return our stop zone's settings (aav_stop_zone's JSON) from tuned nav2.yaml and the car.

    The same as Nav2's collision_monitor reads, so the shadow decides on the same things:
    the forward and backward boxes (stop_zone_points, rear_stop_zone_points) with their
    speed ranges, min_points, the timeouts, and each observation source's topic, type,
    heights and range. Each source's mount in base_link: the radar's from vehicle.yaml
    (its points come in its own frame); every other source sends base_link already.
    tuned: nav2_tuned's dict; car: vehicle.load().
    """
    v = car['vehicle']
    monitor = tuned['collision_monitor']['ros__parameters']
    zone = monitor['StopZone']
    corners = {'forward': stop_zone_points(v), 'backward': rear_stop_zone_points(v)}
    bands = [{'name': name, 'points': corners[name],
              'speed': [float(zone[name]['linear_min']), float(zone[name]['linear_max'])],
              'turn': [float(zone[name]['theta_min']), float(zone[name]['theta_max'])]}
             for name in zone['velocity_polygons']]
    radar = car['sensors']['radar_link']
    mounts = {'radar': [*map(float, radar['xyz']), float(radar['rpy'][2])]}
    sources = []
    for name in monitor['observation_sources']:
        block = monitor[name]
        source = {'name': name, 'topic': block['topic'], 'kind': block['type'],
                  'min_range': float(block.get('min_range', 0.0)),
                  'mount': mounts.get(name, [0.0, 0.0, 0.0, 0.0])}
        for bound in ('min_height', 'max_height'):
            if bound in block:
                source[bound] = float(block[bound])
        sources.append(source)
    return {'min_points': int(zone['min_points']),
            'source_timeout': float(monitor['source_timeout']),
            'stop_pub_timeout': float(monitor['stop_pub_timeout']),
            'bands': bands, 'sources': sources}


# Our global costmap from the map's 64 m chunks (costmap_chunks:=, TODO 175 step 3): the
# chunks the route crosses in the next CHUNKS_AHEAD m are held besides the 3 x 3 round the
# car (the design's C3), and CHUNKS_MARGIN more chunks round them all, room for the planner
# to go round a block the route's own chunks leave out.
CHUNKS_AHEAD = 100.0
CHUNKS_MARGIN = 1


def costmap_chunks_block(folder, ahead=CHUNKS_AHEAD, margin=CHUNKS_MARGIN):
    """
    Return the global costmap's `chunks` block for aav_costmap: {dir, ahead, margin}.

    folder: what map-chunks' chunk-map wrote (chunks.yaml, fine/i_j.pgm); made absolute.
    Raises ValueError without its chunks.yaml, or with a negative ahead or margin.
    """
    path = os.path.abspath(folder)
    if not os.path.isfile(os.path.join(path, 'chunks.yaml')):
        raise ValueError(f'costmap_chunks:={folder}: no chunks.yaml there. Make the chunks '
                         'with pixi run campus (or map-chunks chunk-map)')
    if not (math.isfinite(ahead) and ahead >= 0.0) or int(margin) < 0:
        raise ValueError(f'chunks: ahead {ahead} m and margin {margin} must be 0 or more')
    return {'dir': path, 'ahead': float(ahead), 'margin': int(margin)}


def with_global_window(cfg, metres):
    """
    Return nav2.yaml's dict with the global costmap a `metres` square rolling with the car.

    The three levels' middle level (TODO 175, step 3): 192 m is the 3 x 3 map chunks round
    the car. The planner then searches only that, and the static layer fills it from the
    map as it rolls. The street router's legs (10-30 m between waypoints) always fit; a
    goal further than half the window away can't be planned to. 0: the whole map, as the
    file has it. Nav2's costmap and ours both read it.
    """
    if not (math.isfinite(metres) and metres >= 0.0):
        raise ValueError(f'global_window must be 0 or more metres (got {metres})')
    if metres > 0.0:
        block = cfg['global_costmap']['global_costmap']['ros__parameters']
        block.update(rolling_window=True, width=float(metres), height=float(metres))
    return cfg


# How hard the planner's smoother pulls its path straight (plan_smoothing:=; TODO 168).
# nav2: Nav2's own weights (w_data 0.2, w_smooth 0.3), the default. strong, a challenger:
# w_data 0.02, so the smoother also takes out the lattice's staircase. A road at an angle
# between two of its 16 headings comes out as steps about 9 m long and 0.2 m high; Nav2's
# weights only take out wiggles shorter than about 2 m, so the follower drove the steps:
# the sway on the campus roads. Offline (aav_planner's example staircase, 2.2 m/s) the
# plan's own swing 0.08-0.16 m -> 0.02-0.07 m. Both planners read it (Nav2's lattice and
# ours, planner:=own), from the GridBased block's `smoother`.
PLAN_SMOOTHINGS = {'nav2': None, 'strong': {'w_data': 0.02, 'w_smooth': 0.3}}


THINGS_TRACKERS = ('constant_velocity', 'per_kind')


def check_things_tracker(value):
    """Return things_tracker:= checked: constant_velocity (empty too) or per_kind (217)."""
    value = str(value).strip().lower() or THINGS_TRACKERS[0]
    if value not in THINGS_TRACKERS:
        raise ValueError(f'things_tracker:={value}: one of {", ".join(THINGS_TRACKERS)}')
    return value


GROUNDS = ('plane', 'patchwork')


def check_ground(value):
    """Return ground:= checked: plane (empty too) or patchwork (TODO 215)."""
    value = str(value).strip().lower() or GROUNDS[0]
    if value not in GROUNDS:
        raise ValueError(f'ground:={value}: one of {", ".join(GROUNDS)}')
    return value


MPPI_LAG_SAMPLINGS = ('correlated', 'smooth')


# m each side of the path where mpc_lag_rs leaves the car alone (the tube, Ben 21 Nov).
# Tuned 21 Nov, 3 runs each: University Drive's sway 0.251 (none), 0.197 (0.1), 0.205
# (0.2), 0.160 (0.3), 0.185 (0.4); the lot's unchanged (0.054); every leg reached.
MPC_LAG_TUBE = 0.3


def check_mpc_lag_tube(value):
    """Return mpc_lag_tube:= checked: metres each side of the path, 0 to 1."""
    try:
        tube = float(value)
    except ValueError:
        raise ValueError(f'mpc_lag_tube:={value}: metres, such as 0.2') from None
    if not 0.0 <= tube <= 1.0:
        raise ValueError(f'mpc_lag_tube:={value}: 0 to 1 m')
    return tube


def check_mppi_lag_sampling(value):
    """Return mppi_lag_sampling:= checked: correlated or smooth (TODO 212)."""
    value = str(value).strip() or MPPI_LAG_SAMPLINGS[0]
    if value not in MPPI_LAG_SAMPLINGS:
        raise ValueError(f'mppi_lag_sampling:={value}: one of {", ".join(MPPI_LAG_SAMPLINGS)}')
    return value


def check_steer_gamma(gamma):
    """Return mppi_lag_steer_gamma:= as a float (0 or more), or raise ValueError saying why."""
    try:
        value = float(gamma)
    except (TypeError, ValueError):
        raise ValueError(f'mppi_lag_steer_gamma:={gamma}: a number, such as 0.05') from None
    if not (math.isfinite(value) and value >= 0.0):
        raise ValueError(f'mppi_lag_steer_gamma:={gamma}: 0 or more')
    return value


def check_mppi_batch(batch):
    """
    Return mppi_lag_batch:= as a whole number (1 or more), or raise ValueError saying why.

    mppi_lag's samples a tick (its `batch_size`; nav2.yaml's 2000 when not given): 1000,
    a challenger (TODO 152), half the work a tick on the Orin.
    """
    try:
        value = int(str(batch).strip())
    except (TypeError, ValueError):
        raise ValueError(f'mppi_lag_batch:={batch}: a whole number, such as 1000') from None
    if value < 1:
        raise ValueError(f'mppi_lag_batch:={batch}: 1 or more')
    return value


def check_plan_smoothing(smoothing):
    """Return plan_smoothing:= checked (empty: nav2), or raise ValueError saying what to pick."""
    name = str(smoothing).strip().lower() or 'nav2'
    if name not in PLAN_SMOOTHINGS:
        raise ValueError(f'plan_smoothing:={smoothing}: pick one of '
                         f'{", ".join(PLAN_SMOOTHINGS)}')
    return name


def with_plan_smoothing(cfg, smoothing):
    """
    Return nav2.yaml's dict (tuned) with the planner's smoother weights for plan_smoothing:=.

    nav2 leaves the file's (Nav2's defaults); strong writes PLAN_SMOOTHINGS' weights into
    the GridBased block's `smoother`, which Nav2's Smac planners and ours
    (planner_params) both read.
    """
    weights = PLAN_SMOOTHINGS[check_plan_smoothing(smoothing)]
    plans = cfg.get('planner_server', {}).get('ros__parameters', {})
    if weights and 'GridBased' in plans:
        plans['GridBased'].setdefault('smoother', {}).update(weights)
    return cfg


def _typed(value):
    """Return a rewrite's text as its number or bool (RewrittenYaml's convert_types)."""
    if not isinstance(value, str):
        return value
    try:
        parsed = yaml.safe_load(value)
    except yaml.YAMLError:
        return value
    return parsed if isinstance(parsed, (bool, int, float)) else value


def rewrite(cfg, rewrites):
    """
    Return a copy of a parameter file's dict with `rewrites` written in (no ROS).

    Nav2's RewrittenYaml's rules, so nav2.yaml's tuning reads the same without Nav2: a key
    with no dot (`footprint`) changes every key of that name, at any depth; a dotted path
    (`collision_monitor.ros__parameters.StopZone.forward.points`) changes that one key,
    and adds it where it is missing. Text that reads as a number or a bool becomes one.
    """
    out = copy.deepcopy(cfg)
    leaves = {k: _typed(v) for k, v in rewrites.items() if '.' not in k}

    def walk(node):
        if isinstance(node, dict):
            for key in node:
                if key in leaves:
                    node[key] = leaves[key]
                else:
                    walk(node[key])
        elif isinstance(node, list):
            for item in node:
                walk(item)

    walk(out)
    for path, value in rewrites.items():
        if '.' not in path:
            continue
        *parents, leaf = path.split('.')
        node = out
        for key in parents:
            node = node.setdefault(key, {})
        node[leaf] = _typed(value)
    return out


def nav2_rewrites(v, reversing=False, planner='hybrid'):
    """
    Values to substitute into aav_navigation/config/nav2.yaml.

    RewrittenYaml takes a leaf key (every key of that name) or a whole dotted path. With
    `reversing`, Nav2 may drive backwards, at up to max_reverse_speed: the planner plans
    Reeds-Shepp paths (forward and back), the controller follows them backwards, and MPPI
    may go negative. Without it, the planner plans forward only (Dubins paths). The velocity
    smoother's limit is a list, which RewrittenYaml can't write: see nav2_tuned.
    A whole path that isn't in the file is added (RewrittenYaml's add_params), so the
    Hybrid planner's search model is written only with planner hybrid; the other planners'
    settings are nav2_tuned's.
    """
    r = min_turning_radius(v)
    low, high = obstacle_heights(v)
    hybrid = ({'planner_server.ros__parameters.GridBased.motion_model_for_search':
               'REEDS_SHEPP' if reversing else 'DUBIN'} if planner == 'hybrid' else {})
    return {
        **hybrid,
        'footprint': footprint(v),
        'lidar_scan.ros__parameters.min_height': f'{low:.3f}',
        'lidar_scan.ros__parameters.max_height': f'{high:.3f}',
        'voxels.ros__parameters.min_height': f'{low:.3f}',
        'voxels.ros__parameters.max_height': f'{high:.3f}',
        'minimum_turning_radius': f'{plan_turning_radius(v):.3f}',  # SmacPlannerHybrid
        'min_turning_r': f'{r:.3f}',            # MPPI AckermannConstraints: the car's own
        'collision_monitor.ros__parameters.StopZone.forward.points':
            polygon(stop_zone_points(v)),
        'collision_monitor.ros__parameters.StopZone.backward.points':
            polygon(rear_stop_zone_points(v)),
        'controller_server.ros__parameters.FollowPath.allow_reversing': str(reversing).lower(),
        'controller_server.ros__parameters.MPPI.vx_min': f'{reverse_limit(v, reversing):.3f}',
    }


def nav2_tuned(nav2, v, reversing=False, max_speed=None, radar=True, lidar=False,
               ghost=False, controller=DEFAULT_CONTROLLER, plan_around=True, camera=False,
               planner='own', lattice_file=None, voxels=False, lattice_moves='nav2',
               kerbs=False):
    """
    Return a copy of nav2.yaml's settings (`nav2`, loaded) with what RewrittenYaml can't write.

    RewrittenYaml writes only strings and numbers: a list comes out as a string, and an
    index into a list (`min_velocity.0`) raises a TypeError. So the launch file writes
    these into a copy of the file instead:
    - the controller (CONTROLLERS, DEFAULT_CONTROLLER): rpp keeps FollowPath, Regulated
      Pure Pursuit; mppi
      puts a copy of the MPPI block under FollowPath, the name the behaviour trees ask
      for, with the reverse speed (vx_min) and, with `reversing`, enforce_path_inversion
      (it changes direction where the planner's path does);
    - the velocity smoother's lowest speed, the last stop for a negative speed inside Nav2:
      0, or -max_reverse_speed with `reversing`;
    - with `max_speed` (m/s, sim.launch.py sim_speed:=), the controller's speed and the
      smoother's top speed;
    - the sensors collision_monitor reads: the radar, the lidar, and with `camera` the
      camera obstacles (each camera's detections on the road: behind too, with the rear one);
    - the local costmap's lidar_layer, only with the lidar: a layer that is there but
      disabled still subscribes to its topic;
    - with `ghost` (aav_ghost), the ghost as one more source, and its own layer,
      ghost_layer: it only adds a pretend obstacle, never clears the real sensors' marks;
    - with `radar` or `lidar` and `plan_around`, the same sensor layers in the planner's
      costmap too, so it plans round what they see (the ghost stays out: the car must stop
      for it). Without plan_around the planner sees only the map, and the car stops for
      what's in its way: the stop zone's tests;
    - the planner (PLANNERS; hybrid when none is named, as it needs no `lattice_file`;
      the launch files always name one, as default_planner(reversing) picks): its
      block (PLANNER_BLOCKS) becomes GridBased, the name the behaviour trees ask for, and
      the spare blocks go. lattice also takes
      `lattice_file` (the primitives) and, with `reversing`, may expand backwards; with
      `lattice_moves` smooth (LATTICE_MOVES), its smoother off (smooth_path): Nav2's
      smoother joins each stretch's ends with Dubins curves, the curvature jumps the
      smooth moves leave out (tools/lattice_smooth.py);
    - with `voxels` and `lidar` (aav_voxels, a trial): the lidar_layer reads the cubes and
      the road they saw (sources voxels, voxels_ground) instead of the scan, and
      collision_monitor's voxels source takes the lidar's place;
    - with `kerbs` and `lidar` (TODO 119): the local costmap's kerb_layer (marks only) and
      collision_monitor's steps source, both on /perception/voxels/steps: the step
      finder's kerbs, those of the last 10 s kept by the node (the roof lidar loses a kerb
      about 4 m before the bumper, before it reaches the stop zone).
    """
    if controller not in TUNED_CONTROLLERS:
        raise ValueError(f'controller:={controller}: pick one of {", ".join(TUNED_CONTROLLERS)}')
    planner = server_planner(check_planner(planner))
    out = copy.deepcopy(nav2)
    # (A part of nav2.yaml with no planner_server, as some tests give, keeps none.)
    plans = out.get('planner_server', {}).get('ros__parameters')
    if plans is not None:
        chosen = copy.deepcopy(plans[PLANNER_BLOCKS[planner]])
        for block in PLANNER_BLOCKS.values():
            plans.pop(block, None)
        if planner == 'lattice' and lattice_file:
            # The launch always gives it (navigation.launch.py lattice_for, checked).
            chosen['lattice_filepath'] = lattice_file
            chosen['allow_reverse_expansion'] = bool(reversing)
            if smooth_moves(lattice_moves):
                chosen['smooth_path'] = False
        plans['GridBased'] = chosen
    servers = out['controller_server']['ros__parameters']
    if controller == 'mppi':
        # A copy, so the MPPI block stays where nav2_rewrites' MPPI.vx_min expects it.
        mppi = copy.deepcopy(servers['MPPI'])
        mppi['vx_min'] = reverse_limit(v, reversing)
        mppi['enforce_path_inversion'] = bool(reversing)
        if reversing:
            # Follow the path in the direction the planner asks for, backwards too (mode 2:
            # Smac's paths carry it). Forward only, mode 0: turn towards the path.
            mppi.setdefault('PathAngleCritic', {})['mode'] = 2
            # The planner already decides when to reverse (reverse_penalty); a critic
            # against all reversing kept MPPI from backing up at all (22 Oct).
            mppi['critics'] = [c for c in mppi['critics'] if c != 'PreferForwardCritic']
        servers['FollowPath'] = mppi
    monitor = out['collision_monitor']['ros__parameters']
    cubes = voxels and lidar
    lidar_source = 'voxels' if cubes else 'lidar'
    monitor['observation_sources'] = [name for name, on in (('radar', radar),
                                                            (lidar_source, lidar),
                                                            ('steps', kerbs and lidar),
                                                            ('camera', camera), ('ghost', ghost))
                                      if on]
    local = out['local_costmap']['local_costmap']['ros__parameters']
    gone = {layer for layer, on in (('lidar_layer', lidar), ('ghost_layer', ghost),
                                    ('kerb_layer', kerbs and lidar)) if not on}
    local['plugins'] = [p for p in local['plugins'] if p not in gone]
    if cubes:
        # The cubes stand up to the roof plus its margin: the layer, which bounds every
        # source's heights too, must let them through.
        layer = local['lidar_layer']
        layer['observation_sources'] = 'voxels voxels_ground'
        for bound in ('min_obstacle_height', 'max_obstacle_height'):
            layer[bound] = float(layer['voxels'][bound])
    # The planner sees what the sensors see too, so it plans round an obstacle that isn't on
    # the map (docs/plan.md M5): a copy of each running sensor's layer, before the inflation.
    # Not the ghost: it tests the stop, and the car must stop for it, not steer round it.
    planner = out['global_costmap']['global_costmap']['ros__parameters']
    seen = [layer for layer, on in (('obstacle_layer', radar), ('lidar_layer', lidar))
            if on and plan_around]
    for layer in seen:
        planner[layer] = copy.deepcopy(local[layer])
        planner[layer]['enabled'] = True
        if layer == 'lidar_layer':
            for source in planner[layer].get('observation_sources', '').split():
                block = planner[layer][source]
                block['obstacle_max_range'] = max(float(block.get('obstacle_max_range', 0.0)),
                                                  PLANNER_SEES)
                block['raytrace_max_range'] = max(float(block.get('raytrace_max_range', 0.0)),
                                                  PLANNER_SEES + 3.0)
    if seen:
        at = planner['plugins'].index('inflation_layer')
        planner['plugins'] = planner['plugins'][:at] + seen + planner['plugins'][at:]
    smoother = out['velocity_smoother']['ros__parameters']
    smoother['min_velocity'] = ([reverse_limit(v, reversing)] +
                                [float(x) for x in smoother['min_velocity'][1:]])
    if max_speed is not None:
        follow = servers['FollowPath']
        follow['vx_max' if controller == 'mppi' else 'desired_linear_vel'] = float(max_speed)
        if controller == 'mppi':
            # Its futures must fit on the path it keeps: with 3 m of path and 2.8 s ahead, MPPI
            # stayed near 1 m/s whatever vx_max said (found on the campus, 5 Nov).
            ahead = float(follow['time_steps']) * float(follow['model_dt']) * float(max_speed)
            follow['prune_distance'] = max(float(follow['prune_distance']), ahead * 1.2)
        smoother['max_velocity'][0] = float(max_speed)
    return out

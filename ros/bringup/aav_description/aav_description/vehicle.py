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
    vehicle.centre_of_mass(v, front, rear, front_lifted, lift)   # from the scales (M7, M7b)
    vehicle.wheel_angles(v, centre)  # each front wheel's angle for a centre (bicycle) angle
    vehicle.centre_angle(v, inner, outer)       # the reverse: from M5's two measured angles
    vehicle.gazebo_steering_limit(v) # Gazebo's AckermannSteering steering_limit
"""

import copy
import math
import os

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


def gazebo_steering_limit(v):
    """
    Return the steering_limit for Gazebo's AckermannSteering system (rad).

    It is not the centre angle, nor the inner wheel's (gz-sim 8, AckermannSteering.cc,
    UpdateVelocity): Gazebo turns it into a smallest radius, wheelbase / sin(limit), then
    sets the centre angle to atan(wheelbase / radius). Given max_steering_angle itself,
    the car stops at atan(sin(0.5)) = 0.447 rad, not 0.5. So pass the limit whose sine
    is tan(max_steering_angle): the smallest radius is then wheelbase / tan(max), the
    bicycle model's, and the centre wheel reaches max_steering_angle.
    """
    t = math.tan(v['max_steering_angle'])
    if not 0.0 < t < 1.0:
        raise ValueError("Gazebo's AckermannSteering can't steer 45 degrees or more")
    return math.asin(t)


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
    Why Nav2 must not reverse with these sensors, or None if it may.

    sensors: name=True for each one running (lidar=..., radar=..., rear_camera=...).
    """
    if not reversing or any(sensors.get(name) for name in SEES_BEHIND):
        return None
    return ('nothing sees behind the car (the radar faces forward): Nav2 stays forward '
            "only. Start the lidar (or the rear camera's obstacles) to let it reverse")


# The lidar's obstacles: what lies between these heights above the road (base_link's z).
# Below: the road itself, kerbs, and the ground the lidar sees as its beams tilt with the
# car (0.3 m is the ground 10 m away, with the car pitched 1.7 degrees). Above the car's
# roof by more than OVERHEAD_MARGIN: branches, signs, a garage door the car passes under.
GROUND_CLEARANCE = 0.3  # m
OVERHEAD_MARGIN = 0.5   # m above the body's height


def obstacle_heights(v):
    """Return (lowest, highest) m above the road that counts as an obstacle for the lidar."""
    return GROUND_CLEARANCE, float(v['body']['height']) + OVERHEAD_MARGIN


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


# How often the navigate-to-pose trees plan again (Hz): Nav2's own tree's 1 Hz. The wiggle's
# second test (TODO 149) plans once per goal: REPLAN_ONCE_HZ, once in about three hours.
REPLAN_HZ = 1.0
REPLAN_ONCE_HZ = 1e-4
REPLAN_TAG = 'RateController hz="1.0"'


def replan_tree(xml, hz):
    """
    Return a behaviour tree's text with its replanning rate set to hz (0: plan once).

    The car's trees hold one RateController, at REPLAN_HZ, round the planning (the final
    approach's Fallback inside it stays). A tree without exactly one is refused.
    """
    if xml.count(REPLAN_TAG) != 1:
        raise ValueError(f'the tree needs exactly one {REPLAN_TAG}')
    hz = float(hz)
    if hz < 0.0:
        raise ValueError(f'replan_hz:={hz:g}: 0 (plan once) or more')
    if hz == REPLAN_HZ:
        return xml
    return xml.replace(REPLAN_TAG, f'RateController hz="{hz if hz > 0.0 else REPLAN_ONCE_HZ:g}"')


# Nav2's path followers (docs/path-followers.md): rpp, Regulated Pure Pursuit, chases a point
# ahead on the path; mppi samples thousands of futures through its Ackermann model. MPPI is
# the default since 22 Oct: in Gazebo it did at least as well in every run, and it turns to
# a goal's heading, which Pure Pursuit can't (it aims only at a point).
CONTROLLERS = ('rpp', 'mppi')
DEFAULT_CONTROLLER = 'mppi'


# The trial path followers (docs/trials.md, aav_track): each stands in for controller_server's
# FollowPath with tracker:=<name>. 'none' keeps controller_server (CONTROLLERS) following.
# Apart from CONTROLLERS, not in it: nav2_tuned treats anything not mppi as RPP.
# mppi_lag is aav_mppi_lag's node (Rust), not aav_track's: Nav2's MPPI ported, on a car
# model with the actuators' lag (docs/mppi-lag.md).
TRACKERS = ('none', 'pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full', 'mpc_lag', 'mppi_lag')
DEFAULT_TRACKER = 'none'
# The trackers that drive forwards only (aav_track's): refused with reversing:=true.
# mppi_lag reverses as Nav2's MPPI does (Ben, 14 Nov: "The car can reverse").
FORWARD_ONLY_TRACKERS = ('pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full', 'mpc_lag')


def check_tracker(tracker, reversing=False):
    """Return the tracker's name (lower case), or raise ValueError saying what to pick."""
    name = str(tracker).strip().lower()
    if name not in TRACKERS:
        raise ValueError(f'tracker:={tracker}: pick one of {", ".join(TRACKERS)}')
    if name in FORWARD_ONLY_TRACKERS and reversing:
        raise ValueError(f'tracker:={name} drives forwards only: add reversing:=false '
                         "(the planner would plan reversing it can't follow)")
    return name


# Whose costmaps (costmap:=, docs/own-costmap.md): nav2 (the car's) or own (aav_costmap:
# the sim's, stack:=ours). Each one's raw local costmap, for aav_mppi_lag.
COSTMAPS = ('nav2', 'own')
DEFAULT_COSTMAP = 'nav2'
COSTMAP_RAW_TOPICS = {'nav2': '/local_costmap/costmap_raw',
                      'own': '/drivers/nav/costmap/local_raw'}


def check_costmap(costmap):
    """Return the costmap's name (lower case), or raise ValueError saying what to pick."""
    name = str(costmap).strip().lower()
    if name not in COSTMAPS:
        raise ValueError(f'costmap:={costmap}: pick one of {", ".join(COSTMAPS)}')
    return name


# Whose navigator (navigator:=, docs/own-navigator.md): nav2 (bt_navigator and
# waypoint_follower: the car's) or own (aav_navigator: the sim's, stack:=ours). Both serve
# the same four actions under Nav2's own names, so no goal sender knows which one runs.
NAVIGATORS = ('nav2', 'own')
DEFAULT_NAVIGATOR = 'nav2'
NAVIGATOR_ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
                     'follow_gps_waypoints')


def check_navigator(navigator):
    """Return the navigator's name (lower case), or raise ValueError saying what to pick."""
    name = str(navigator).strip().lower()
    if name not in NAVIGATORS:
        raise ValueError(f'navigator:={navigator}: pick one of {", ".join(NAVIGATORS)}')
    return name


# Whose velocity smoother (smoother:=, docs/own-smoother.md): nav2 (velocity_smoother: the
# default, and the car's) or own (aav_smoother's car_smoother, a trial: the turn rate from
# where the wheels can be). Ours runs under Nav2's node name, so it reads the same block.
SMOOTHERS = ('nav2', 'own')
DEFAULT_SMOOTHER = 'nav2'


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


def all_ours(navigator, planner, costmap, tracker):
    """
    Return True when every navigation piece is ours.

    Then no Nav2 server is needed for navigation (planner_server, controller_server,
    behavior_server, bt_navigator, waypoint_follower stay off). map_server, the velocity
    smoother and collision_monitor still run: the map, and the command line in front of
    the safety locks.
    """
    return (navigator, planner, costmap, tracker) == ('own', 'own', 'own', 'mppi_lag')


def navigator_params(v, reversing, waypoints='pass_through'):
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
    out = {'replan_period': 1.0, 'through_replan_period': 3.0,
           'final_approach': round(final_approach(v), 3), 'path_end_tolerance': 0.5,
           'path_max_age': 20.0, 'keep_path_age': 10.0, 'retries': 6, 'wait': 5.0,
           'pass_radius': 1.0, 'on_path': 2.0, 'waypoints': waypoints, 'stop_on_failure': False,
           'replan_failures': 3, 'backup': None,
           'plan_timeout': 10.0, 'follow_timeout': 4.0, 'recovery_timeout': 5.0}
    if reversing:
        out['backup'] = {'distance': 1.0, 'speed': 0.5 * float(v['max_reverse_speed'])}
    return out


# Nav2's planners (planner:=, docs/controllers.md "Planners"), each under the one plugin name
# the behaviour trees ask for, GridBased. hybrid (SmacPlannerHybrid, the default) and lattice
# (SmacPlannerLattice, on our own primitives: tools/lattice_primitives.py) keep to the car's
# turning radius. smac2d (SmacPlanner2D), navfn (NavfnPlanner) and theta_star
# (ThetaStarPlanner) plan on the grid as if the car could turn on the spot: their paths may
# have corners the car can't drive, and the follower must round them. Trials only: refused
# on the real car (check_planner real=True).
# diffusion (aav_diffusion_planner, a trial: docs/designs/2026-11-02_diffusion-planner.md):
# a model draws the path and a guard checks it, with the lattice behind it in planner_server
# as the fallback (TRIAL_PLANNERS: the planner each stands on). Sim and bench only.
# own (aav_planner, a trial: docs/own-planner.md): Nav2's State Lattice ported to Rust,
# on the same primitives file and penalties; bt_navigator's trees call it, and
# planner_server keeps running the lattice beside it (navigate_through_poses, and the
# head-to-head in controller_compare). Sim and bench only.
PLANNERS = ('hybrid', 'lattice', 'smac2d', 'navfn', 'theta_star', 'diffusion', 'own')
CAR_PLANNERS = ('hybrid', 'lattice')
TRIAL_PLANNERS = {'diffusion': 'lattice', 'own': 'lattice'}
# planner:=own: our planner's action; bt_navigator's compute_path_to_pose is pointed here.
OWN_PLANNER_ACTION = '/drivers/nav/planner/compute_path_to_pose'
# ... and its through-poses action (bt_navigator's, and our navigator's, are pointed here).
OWN_PLANNER_THROUGH_ACTION = '/drivers/nav/planner/compute_path_through_poses'
# ... and planner_server's own /plan goes here (ours publishes /plan).
OWN_PLANNER_NAV2_PLAN = '/drivers/nav/planner/nav2_plan'
# The lattice is the default (Ben, 1 Nov): with reversing on, every leg of the lot tour,
# 29 s quicker than Hybrid-A* (docs/trials.md, 30 Oct). Forward only it lost the U-turn
# (29 Oct), so a car that may not reverse takes Hybrid-A* (FORWARD_ONLY_PLANNER).
DEFAULT_PLANNER = 'lattice'
FORWARD_ONLY_PLANNER = 'hybrid'
# Each planner's block in nav2.yaml's planner_server (GridBased is hybrid's own).
PLANNER_BLOCKS = {'hybrid': 'GridBased', 'lattice': 'Lattice', 'smac2d': 'Smac2D',
                  'navfn': 'NavFn', 'theta_star': 'ThetaStar', 'diffusion': 'Lattice',
                  'own': 'Lattice'}


def server_planner(planner):
    """Return the planner planner_server runs for planner:= (a trial's, the one behind it)."""
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


def track_params(nav2, cfg, ego=None, max_speed=None):
    """
    Return aav_track's parameters: the same speed and "done" as Nav2's controllers.

    nav2: nav2.yaml as loaded (not tuned: nav2_tuned puts MPPI's block under FollowPath);
    cfg: vehicle.yaml (load()); ego: ego.yaml's ros__parameters, for its steering rate.
    The cruise speed is RPP's desired_linear_vel, or `max_speed` (sim_speed:=); the goal
    and progress checks are controller_server's; speeding up and slowing down are the
    velocity smoother's.
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
    }


# vehicle.yaml's actuator keys aav_mppi_lag's lag model takes (as actuators.<key>). The dead
# times are optional there: 0 when unset.
MPPI_LAG_ACTUATORS = ('max_accel', 'throttle_deadband', 'throttle_lag', 'max_brake',
                      'brake_lag', 'drag', 'steer_rate', 'steer_lag', 'steer_offset',
                      'steer_deadband', 'throttle_dead_time', 'brake_dead_time',
                      'steer_dead_time')
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


def mppi_lag_params(nav2, cfg, ego=None, max_speed=None, reversing=False, lane=False):
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
    'terminal_checking_interval', 'reverse_penalty', 'change_penalty',
    'non_straight_penalty', 'cost_penalty', 'retrospective_penalty', 'rotation_penalty',
    'analytic_expansion_ratio', 'analytic_expansion_max_length',
    'analytic_expansion_max_cost', 'analytic_expansion_max_cost_override',
    'lookup_table_size', 'smooth_path')


# The Hybrid-A* block's keys our planner reads (planner_search:=hybrid), besides the shared
# ones in OWN_PLANNER_KEYS; motion_model_for_search and minimum_turning_radius are written.
OWN_HYBRID_KEYS = ('angle_quantization_bins', 'allow_primitive_interpolation',
                   'downsample_obstacle_heuristic')


def planner_params(tuned, v, costmap='nav2', search='lattice', nav2=None):
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
    'nav2': ('/global_costmap/costmap_raw', '/global_costmap/costmap_raw_updates', True),
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


def behaviour_params(nav2, cfg, ego=None, max_speed=None, controller=DEFAULT_CONTROLLER):
    """
    Return aav_behaviour's parameters: the cap's top, the lane ahead, ego's limits.

    base_speed is the controller's own top speed (MPPI's vx_max, RPP's desired_linear_vel,
    or `max_speed`, sim_speed:=): a higher cap would raise MPPI's envelope. front and
    half_width are the body's front bumper and half its width plus the stop zone's margin;
    stop_gap is the stop zone's reach (STOP_ZONE_AHEAD): the obstacle's final stop.
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
           'stop_gap': float(STOP_ZONE_AHEAD)}
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
               planner='hybrid', lattice_file=None, voxels=False, lattice_moves='nav2'):
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
      collision_monitor's voxels source takes the lidar's place.
    """
    if controller not in CONTROLLERS:
        raise ValueError(f'controller:={controller}: pick one of {", ".join(CONTROLLERS)}')
    planner = server_planner(check_planner(planner))
    if planner == 'lattice' and not lattice_file:
        raise ValueError('planner:=lattice needs its primitives file (lattice_file)')
    out = copy.deepcopy(nav2)
    # (A part of nav2.yaml with no planner_server, as some tests give, keeps none.)
    plans = out.get('planner_server', {}).get('ros__parameters')
    if plans is not None:
        chosen = copy.deepcopy(plans[PLANNER_BLOCKS[planner]])
        for block in PLANNER_BLOCKS.values():
            plans.pop(block, None)
        if planner == 'lattice':
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
                                                            ('camera', camera), ('ghost', ghost))
                                      if on]
    local = out['local_costmap']['local_costmap']['ros__parameters']
    gone = {layer for layer, on in (('lidar_layer', lidar), ('ghost_layer', ghost)) if not on}
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

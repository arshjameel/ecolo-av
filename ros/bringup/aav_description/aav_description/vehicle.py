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


# Nav2's path followers (docs/path-followers.md): rpp, Regulated Pure Pursuit, chases a point
# ahead on the path; mppi samples thousands of futures through its Ackermann model. MPPI is
# the default since 22 Oct: in Gazebo it did at least as well in every run, and it turns to
# a goal's heading, which Pure Pursuit can't (it aims only at a point).
CONTROLLERS = ('rpp', 'mppi')
DEFAULT_CONTROLLER = 'mppi'


# The trial path followers (docs/trials.md, aav_track): each stands in for controller_server's
# FollowPath with tracker:=<name>. 'none' keeps controller_server (CONTROLLERS) following.
# Apart from CONTROLLERS, not in it: nav2_tuned treats anything not mppi as RPP.
TRACKERS = ('none', 'pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full')
DEFAULT_TRACKER = 'none'


def check_tracker(tracker, reversing=False):
    """Return the tracker's name (lower case), or raise ValueError saying what to pick."""
    name = str(tracker).strip().lower()
    if name not in TRACKERS:
        raise ValueError(f'tracker:={tracker}: pick one of {", ".join(TRACKERS)}')
    if name != 'none' and reversing:
        raise ValueError(f'tracker:={name} drives forwards only: add reversing:=false '
                         "(the planner would plan reversing it can't follow)")
    return name


# Nav2's planners (planner:=, docs/controllers.md "Planners"), each under the one plugin name
# the behaviour trees ask for, GridBased. hybrid (SmacPlannerHybrid, the default) and lattice
# (SmacPlannerLattice, on our own primitives: tools/lattice_primitives.py) keep to the car's
# turning radius. smac2d (SmacPlanner2D), navfn (NavfnPlanner) and theta_star
# (ThetaStarPlanner) plan on the grid as if the car could turn on the spot: their paths may
# have corners the car can't drive, and the follower must round them. Trials only: refused
# on the real car (check_planner real=True).
PLANNERS = ('hybrid', 'lattice', 'smac2d', 'navfn', 'theta_star')
CAR_PLANNERS = ('hybrid', 'lattice')
DEFAULT_PLANNER = 'hybrid'
# Each planner's block in nav2.yaml's planner_server (GridBased is hybrid's own).
PLANNER_BLOCKS = {'hybrid': 'GridBased', 'lattice': 'Lattice', 'smac2d': 'Smac2D',
                  'navfn': 'NavFn', 'theta_star': 'ThetaStar'}
# The lattice's turning radius against plan_turning_radius: the file keeps 3 decimals.
LATTICE_TOLERANCE = 0.005


def check_planner(planner, real=False):
    """
    Return the planner's name (lower case), or raise ValueError saying what to pick.

    real: the launch drives the real car (data source REAL, car.launch.py). There only
    CAR_PLANNERS, whose paths keep to the turning radius: a corner from a grid planner is
    a path the car can't drive, and the safest answer is not to offer it at all.
    """
    name = str(planner).strip().lower()
    if name not in PLANNERS:
        raise ValueError(f'planner:={planner}: pick one of {", ".join(PLANNERS)}')
    if real and name not in CAR_PLANNERS:
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
    and progress checks are controller_server's; speeding up is the velocity smoother's.
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


def nav2_rewrites(v, reversing=False, planner=DEFAULT_PLANNER):
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
               planner=DEFAULT_PLANNER, lattice_file=None):
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
    - the planner (PLANNERS, DEFAULT_PLANNER): its block (PLANNER_BLOCKS) becomes GridBased,
      the name the behaviour trees ask for, and the spare blocks go. lattice also takes
      `lattice_file` (the primitives) and, with `reversing`, may expand backwards.
    """
    if controller not in CONTROLLERS:
        raise ValueError(f'controller:={controller}: pick one of {", ".join(CONTROLLERS)}')
    planner = check_planner(planner)
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
    monitor['observation_sources'] = [name for name, on in (('radar', radar), ('lidar', lidar),
                                                            ('camera', camera), ('ghost', ghost))
                                      if on]
    local = out['local_costmap']['local_costmap']['ros__parameters']
    gone = {layer for layer, on in (('lidar_layer', lidar), ('ghost_layer', ghost)) if not on}
    local['plugins'] = [p for p in local['plugins'] if p not in gone]
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
        smoother['max_velocity'][0] = float(max_speed)
    return out

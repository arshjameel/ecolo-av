"""
The whole stack in a simulator: same nodes and configs as the car, simulated hardware.

  ros2 launch aav_bringup sim.launch.py [stack:=ours|best|car|]
      [pose:=] [path:=] [steer:=] [lanes:=]
      [world:=empty|test_lot|road|campus] [nav:=true]
      [teleop:=false] [rviz:=true]
      [perception:=false] [lane:=false|true] [camera_obstacles:=false] [hud:=false]
      [map:=false] [foxglove:=false] [chase_camera:=true] [cameras:=true]
      [obstacle:=false] [start_x:=0.0] [start_y:=0.0] [start_yaw:=0.0] [lidar:=true] [radar:=true]
      [actuator_model:=true] [speed_loop:=default|smooth|lqi|firm] [speed_loop_model:=]
      [sim_speed:=]
      [payload:=0] [adaptive_brake:=false]
      [reversing:=true] [controller:=] [planner:=] [tracker:=] [costmap:=] [navigator:=]
      [smoother:=] [behaviour:=]
      [amcl:=false] [amcl_impl:=rust] [voxels:=false] [voxels_impl:=python] [kerbs:=]
      [ego_impl:=rust]
      [lidar_impl:=rust] [things_impl:=rust] [lane_finder_impl:=rust]
      [behaviour_impl:=rust] [watchdog_impl:=rust] [localization_health_impl:=rust]
      [io_mux_impl:=rust] [stop_sign_impl:=rust]
      [geofence_impl:=rust] [stop_hold_impl:=rust] [goal_leash_impl:=rust]
      [ndt:=false] [ndt_ekf:=false] [pose_source:=gps] [ndt_map:=maps/ndt/test_lot.pcd]
      [kiss_icp:=false]
      [kiss_ekf:=false] [lane_pose:=false]
      [plan_around:=true] [things:=false]
      [watchdog:=true] [geofence:=true] [geofence_radius:=]
      [watch_hud:=false] [faults:=false] [spine:=false] [arm:=true]
      [esp32:=false] [esp32_loop:=pi] [legacy_impl:=rust]
      [bridge:=rust|python]
      [ghost:=false] [simulator:=ravensim] [camera_lockstep:=auto] [graphics:=auto]
  pixi run sim world:=road                         the road: the car's camera and radar
  pixi run sim world:=campus                       Carleton, every level ours
  pixi run sim stack:=best                         every level's champion (as the car)
  pixi run sim path:=hybrid_rs steer:=mpc_lag      one level's league option each (stacks.yaml)

Ours is the default (Ben, 14 Nov: "Ours default now, then delete"): stack:=ours sets the
planner, path follower, costmap, navigator, smoother, behaviour and pose at once
(aav_description's config/stacks.yaml, docs/league.md). Those switches are empty by
default, meaning "the stack's choice"; any value given by hand wins over the stack.
One switch per level (one shape, "From 36 switches to 8"): pose:=, path:=, steer:= and
lanes:= pick that level's option by its name in stacks.yaml (path:=lane), over
the stack's pick; a single switch given by hand (planner:=) still wins over both.
stack:= (empty): no stack, each switch as given, else the champions'. The real
car (car.launch.py) keeps its own choices until ours wins there too.
  pixi run -e ml sim world:=road perception:=true  adds the detector and the stop-sign watcher
  pixi run sim world:=test_lot sim_speed:=3 hud:=true   Ravensim, RViz, navigation, the HUD
  pixi run sim world:=test_lot map:=true           ... and the big screen's map window

world:=empty pairs with the empty map (phase 2); world:=test_lot with
maps/example_lot.yaml (phase 3); world:=road, the bench's road in 3D, with the empty
map: chosen automatically. The car starts at the world origin facing east, which is also
the map origin (the datum); start_x, start_y and start_yaw move it, as on the bench.
world:=campus is Carleton (pixi run campus makes it): its map, its NDT guess and a start in lot
P4 come with it, because the datum itself is on a station shelter there.

The road and test_lot worlds render the car's three cameras, its radar stand-in and the
roof lidar (aav_sim_models worlds.has_sensors), so the car's whole graph runs there, with the
bench's argument names:

  the radar           where the world renders it (radar:=false drops it, to test the
                      lidar alone): radar_clusters, and our costmap's obstacle layer and
                      the stop zone (navigation.launch.py radar:=true)
  perception:=true    the detector (RF-DETR; the ml environment) and stop_sign_watcher: twist_mux's
                      stop-sign lock
  lane:=true          the lane finder alone, on /perception/lane/geometry (the lane
                      follower went on 19 Nov, TODO 264: lanes:=camera draws its lines into
                      the costmap, and starts it too)
  camera_obstacles    the detectors' boxes into our local costmap
  hud:=true           the HUD window (aav_hud, as pixi run hud): needs a screen
  map:=true           the map window (aav_hud, as pixi run map): the map, the
                      car, the plan, the distance left. Needs a screen
  foxglove:=true      foxglove_bridge on 127.0.0.1:8765, for Lichtblick or Foxglove on this
                      computer: open ws://localhost:8765 and a layout from tools/foxglove/
                      (foxglove.launch.py: the car's topic list, and the front camera's
                      preview where the world renders it; docs/foxglove.md)
  chase_camera:=false no chase camera: by default a camera behind and above the car
                      (aav_sim_models), for Lichtblick's cockpit layout (tools/foxglove/
                      cockpit.json): /vehicle/sim/chase_camera/compressed
  cameras:=false      no cameras at all (no picture, no chase camera): Ravensim then
                      runs on a computer with no GPU, as CI's runners; perception, lane
                      and camera_obstacles are refused
  run_info            always: which algorithms the run uses, from these arguments, on
                      /system/algorithms (aav_run_info), for the cockpit's Algorithms table
  obstacle:=true      the bench's 1 m box in the lane, 15 m ahead (aav_sim_models)
  lidar:=false        no roof lidar. It's on by default (15 Oct): a simulated 16-beam
                      lidar on /sensors/lidar/points, which the costmap reads as a 2D scan
                      (navigation.launch.py lidar:=true: its local costmap and
                      collision_monitor). Its earlier flakiness on a Mac came from other
                      programs competing
  watch_screens:=dev_window,hw_panel  commands flow only while those screens beat (they
                      send it with --developer): close one and the output mux closes, and
                      with spine:=true the spine's own state machine latches
  sim_speed:=3        a higher top speed (m/s, up to 5), for ego and the follower, simulators only:
                      the car's 1.0 m/s cap (ego.yaml, nav2.yaml) is for its first drives.
                      ego refuses it outside a simulator
  planner:=diffusion  a model draws the path, a guard checks it, our planner's path when it
                      fails (aav_diffusion_planner, a trial: pixi install --locked -e
                      learn, pixi run diffusion train first); diffusion_model:=<.onnx>
                      another model; diffusion_record:=<folder> each plan's inputs, to
                      replay offline; diffusion_smoothing:=average the 2 Nov smoothing;
                      diffusion_impl:=rust the model from Rust (aav_diffusion_rs), with
                      diffusion_provider:=auto (Core ML on a Mac; docs/onnx.md).
                      own (the default): our planner (aav_planner: Nav2's lattice in
                      Rust; docs/own-planner.md). Nav2's planners went on 17 Nov.
                      lattice_headings:=32: the lattice's trial file, 32 headings
                      plan_smoothing:=strong: the planner's smoother pulls harder, so a
                      road at an angle is no staircase (a challenger, TODO 168)
                      mppi_lag_steer_gamma:=0.05: mppi_lag's control cost on its steering
                      (empty: nav2.yaml's gamma, 0.015), a challenger: less sampling
                      jitter (TODO 168)
                      mppi_lag_batch:=1000: mppi_lag's samples a tick (empty: 2000),
                      a challenger: half the work a tick (TODO 152; steer:=mppi_lag_1k)
                      mpc_lag_tube:=0: mpc_lag_rs's tube off (empty: 0.3 m, tuned
                      21 Nov: it leaves the car alone that near the path, less sway)
                      mppi_lag_sampling:=smooth: mppi_lag samples the steering's
                      change, so its commands come out smooth (TODO 212, a challenger)
                      lattice_moves:=smooth|smooth_fast: the lattice's trial moves,
                      curvature no faster than the steering turns (tools/lattice_smooth.py)
                      planner_search:=hybrid: ours runs Nav2's Hybrid-A* ported instead
                      of the lattice (a challenger, the league's hybrid_rs)
  tracker:=lqr        another path follower (mppi_lag, ours in Rust, is the default;
                      aav_track: pure_pursuit, stanley, lqr or mpc; mpc_lag_rs), on the
                      same planner, smoother and stop zone. aav_track's drive forwards
                      only: with reversing:=false (docs/trials.md); mppi_lag reverses too
  costmap:=own, navigator:=own, smoother:=own, controller:=mppi
                      each the only option since Nav2 went (17 Nov, TODO 184; its scores:
                      docs/league-results-2026-11-16.md)
  behaviour:=false    no behaviour planner (on by default with stack:=ours). On:
                      the behaviour states and a speed cap on the follower from the stopping law
                      (aav_behaviour_rs, a trial: docs/stopping.md); it also sends the road ahead
                      (/world/road_ahead), and ego holds navigation to it (ego road_ahead:=true)
  kerbs:=true         the kerbs (TODO 119), where the lidar runs: the Rust voxels node finds
                      steps in the ground (aav_voxels_rs steps:=true) and the local
                      costmap's kerb_layer and the stop zone read them. Empty: the
                      stack's choice (on in ours since 16 Nov; stacks.yaml's kerbs level)
  lane_map:=world    with behaviour:=true: the road ahead's line is the lane's centre, from
                      the world's lane map (road: aav_sim_models worlds/road.lanes.yaml; campus:
                      maps/campus/lanes.yaml), with its edges; or a lanes.yaml's path.
                      Empty (the default): the line is /plan's
  lane_costs:=true    the lanes layer, a challenger (TODO 148, D2): our global costmap
                      (costmap:=own) draws the lane map (lane_map:='s, else the world's
                      own) as soft costs, lowest at the right lane's centre, so our planner
                      keeps right. A world with no lane map (the test lot): nothing drawn.
  lane_source:=camera  with lane_costs:=true (stacks.yaml's lanes:=camera, both): the
                      lanes layer from the lane finder's lines (it starts), the same costs:
                      kerb high, the centre line and oncoming lane medium, the right lane's
                      centre low (TODO 264); both: the lane map wins where it has a road.
                      Empty: the stack's choice (off)
  costmap_chunks:=auto  our global costmap (costmap:=own) from the map's 64 m chunks: the
                      3 x 3 round the car and those along the route ahead, read and
                      inflated each on its own core (TODO 175). auto: the campus's chunks
                      when pixi run campus made them, else the whole map; off: the whole
                      map; a folder: those chunks. Empty: the stack's choice (auto in
                      ours since 16 Nov, else off)
  lane_route:=true    the lane-centre planner (path:=lane; ours' pick since 16 Nov, TODO
                      148): our planner (planner:=own) routes along the lane map
                      (lane_map:='s, else the world's own) and plans the right lane's
                      centre, the lattice joining the car and the goal to it; off the map,
                      or a world with none (the test lot), the lattice alone.
                      Empty: the stack's choice (on in stack:=ours, else off)
  lane_graph:=lanelet2  with lane_route: route on the Lanelet2 map beside the lane map
                      (pixi run campus: maps/campus/campus.osm; lanelet-map in Rust): its
                      lanes, their neighbours, the stop lines on the way. lanes_yaml: the
                      lane map's roads; a world with no Lanelet2 map uses them too.
                      Empty: the stack's choice (TODO 175, step 6)
  local_chunks:=true  with costmap_chunks: the local costmap's static layer from the 3 x 3
                      chunks round the car too, so no costmap reads /map (TODO 175).
                      Empty: the stack's choice (true in ours since 16 Nov)
  lane_pass:=false    with lane_route: a parked obstacle on the lane line gives the whole
                      path to the lattice (before 16 Nov); true (the default): only the
                      blocked stretch moves left, the least that clears it, then back
  lane_corners:=clothoid  with lane_route: the lane line's corners as clothoid, arc,
                      clothoid, the curvature continuous (TODO 211, a challenger: a tie
                      on University Drive); arc (the default): circle arcs
  things:=true        the lidar's cubes as things with an id, a speed and a memory, on
                      /world/things (aav_things_rs; voxels runs for it): only with nav:=true
                      and the lidar. The behaviour planner waits for what moves (a hold
                      for anything moving near, 17 Nov). Empty: the stack's choice (on in
                      ours, where nav and the lidar run)
  things_background:=true  with things (the default): low cubes on the saved map are not
                      things (TODO 175); false: every cube is;
                      things_background_chunks:=maps/campus/chunks/ndt: the NDT map's
                      chunks round the car instead of /map;
                      things_background_below:=0.7 (the default): only cubes under
                      it drop
  amcl:=true          AMCL beside the global EKF, scored only (docs/trials.md): only
                      in world:=test_lot with nav:=true and the lidar, from the start pose;
                      (aav_amcl_rs; stock nav2_amcl removed 20 Nov)
  voxels:=true        our local costmap and the stop zone read the lidar as 0.25 m
                      cubes (aav_voxels, a trial: docs/designs/2026-10-30_voxels.md)
                      instead of the flat scan; only with the lidar
  lidar_impl:=python  the Python lidar_scan (aav_lidar) in the Rust one's place (the
                      default since 17 Nov): the same scan
  lane_finder_impl:=python  the Python lane_finder (aav_lane_finder, OpenCV) in the Rust
                      one's place (the default since 18 Nov), with lane:=true or
                      lane_pose:=true: the same topics and lanes
  voxels_impl:=rust   the Rust voxels node (aav_voxels_rs) in the Python one's place, for
                      voxels:=true or screen_cubes:=true: the same topics and answers
  detector_impl:=rust with perception:=true, the Rust detector (aav_detector_rs: RF-DETR's
                      ONNX through ONNX Runtime, Core ML on a Mac) in the Python one's
                      place, a challenger: the same name and topics (docs/onnx.md)
  ego_impl:=rust      ego and look_ahead in Rust, one process (aav_ego_rs), in the Python
                      nodes' place: the same node names, topics and answers (TODO 185);
                      the default on the car and here since 16 Nov (python: aav_ego's two nodes)
  behaviour_impl:=rust  the behaviour node in Rust (aav_behaviour_rs), with behaviour on
                      (TODO 185); the only one since the Python went on 20 Nov
  watchdog_impl:=rust  the sensor watchdog in Rust (aav_watchdog_rs): the same topics and
                      answers as the Python's had; the only one since 20 Nov (Ben, 16 Nov:
                      "watchdog in rust sure")
  stop_sign_impl:=rust  with perception:=true, stop_sign_watcher in Rust (aav_stop_sign_rs):
                      the same node name, topics and answers; the default since 17 Nov
                      (python: aav_detector's)
  io_mux_impl:=rust  the sensor mux and the output mux in Rust (aav_io_mux_rs): the same
                      node names, topics and answers; the default on the car and here
                      since 17 Nov (Ben: "Rust tomorrow"; python: aav_io_mux's)
  localization_health_impl:=rust  localization_health in Rust
                      (aav_localization_health_rs): warn only (TODO 185); the only one
                      since the Python went on 20 Nov
  ndt:=true          NDT on the saved lidar map (aav_ndt, Rust), scored only, never TF
                      (docs/designs/2026-10-29_ndt.md): only in world:=test_lot with the
                      lidar, from the start pose, and only once ndt_map (pixi run ndt-map
                      drive) is there
  ndt_chunks:=        with ndt:=true, the map in 64 m chunks, only the 3 x 3 round the car
                      held (TODO 175). Empty: on the campus, its chunks when pixi run
                      campus made them (scored as good as the whole map, 15 Nov), else the
                      whole map; off: the whole map; a folder: those chunks
  ndt_ekf:=true       a trial: NDT as above (ndt:=true is implied), and ekf_global takes
                      its pose (x, y, yaw) beside the GPS (aav_localization
                      config/ekf_global_ndt.yaml; docs/trials.md). Default false: the
                      global EKF on GPS alone
  pose_source:=ndt    a challenger (TODO 210): NDT leads, the GPS checks. NDT runs (as
                      ndt:=true); pose_check (aav_pose_check) passes NDT's pose to
                      ekf_global while NDT is trusted, the GPS's only while it isn't, and
                      says HOLD when neither is. The watchdog watches gps_check (the GPS's
                      checks, warn only) and pose_check (stops on HOLD) in gps's place; the
                      geofence's fix is /localization/pose_check/fix. Default gps: today's
  kiss_icp:=true      KISS-ICP lidar odometry (aav_kiss_icp), scored only, never TF or an
                      EKF (docs/designs/2026-10-30_kiss-icp.md): any world with the lidar;
                      needs the kiss environment once (pixi install --locked -e kiss)
  kiss_ekf:=true      a trial: KISS-ICP as above (kiss_icp:=true is implied), and ekf_local
                      takes its twist (vx, vyaw) beside the wheels and the IMU
                      (aav_localization config/ekf_local_kiss.yaml; docs/trials.md).
                      Default false: the local EKF on the wheels and the IMU alone
  odom_lidar:=kinematic_icp  with kiss_icp or kiss_ekf: Kinematic-ICP's idea in Rust
                      (aav_kinematic_icp, TODO 216, a challenger): one arc a step, the
                      wheels its prior; the same topics as KISS-ICP
  lane_pose:=true     a trial (TODO 155): the lane finder as a pose sensor. lane_finder runs
                      (as lane:=true, without the lane follower) and lane_pose
                      (aav_lane_pose) turns its lines and the lane map (lane_map:=, else
                      the world's own) into the car's place across its lane; ekf_global
                      takes its x and y beside the GPS (aav_localization
                      config/ekf_global_lane.yaml). Needs the camera. Empty: the
                      stack's choice (on in ours since 17 Nov, where there is a camera
                      and a lane map; stacks.yaml's pose level, ekf_lane)
  truth_pose:=true    a test (the wiggle, TODO 149): the simulator's truth is the car's pose.
                      No ekf_global; aav_sim_models's truth_localizer publishes
                      /localization/odom/global and map -> odom so that map -> base_link
                      is the truth. ekf_local still gives odom -> base_link. Never the car
  replan_hz:=1.0      how often the navigator plans a to-pose goal again (navigation.launch.py;
                      1.0 as Nav2's own tree). replan_hz:=0: plan once per goal, the wiggle's
                      second test
  map_correction:=slow  with a trial tracker (mppi_lag, aav_track), a challenger: the
                      follower's held map -> odom, moved toward the newest slowly (TODO 186);
                      newest (the default): each new path by the newest map -> odom
  plan_around:=false  the planner sees only the map, not the radar and lidar: the car
                      stops for an obstacle instead of planning round it (the stop tests)
  watchdog:=true      the sensor watchdog (aav_safety): twist_mux's sensors lock stops navigation
                      and the AI slot when the wheels, the IMU, the GPS or an EKF go quiet,
                      NaN or frozen, or the GPS jumps; and the camera, where something
                      reads it (perception, lane, camera_obstacles, faults). With
                      hud:=true, the HUD too: its /hud/heartbeat stops when its screen
                      freezes or dies
  geofence:=true      the geofence (aav_safety): twist_mux's geofence lock holds every
                      program (scripted, point, navigation, AI; never a person) before its
                      stopping distance reaches a circle of geofence_radius m (200 here,
                      600 on the campus: past every sim check's drive; the car's is 30)
                      round where the car starts (0, start_y), or while the global pose
                      can't be trusted
  watch_hud:=true     the watchdog watches /hud/heartbeat without the HUD window: something
                      else beats (sim-check --faults stands in for the screen)
  faults:=true        the front left picture through sim_camera_relay and the lidar through
                      sim_lidar_relay, so fault.camera and fault.lidar work (pixi run
                      sim-fault camera freeze, lidar stop); the other faults always do
  spine:=true         the spine in the loop: ego -> spine_bridge -> CAN frames -> spine_sim
                      (the spine's own firmware logic: arming, its 100 ms command
                      watchdog, both e-stops, faults, its speed loop) -> the actuator
                      model -> Ravensim; its measurements back as CAN frames ->
                      spine_bridge -> /vehicle/*. arm:=true arms it after 12 s;
                      bridge:=python runs aav_spine's Python spine_bridge, not the Rust one
  esp32:=true         today's car in the loop: ego -> legacy_uart_bridge (the car's own)
                      -> 13-byte frames over local TCP -> sim_esp32 (the 2025-26 C++
                      firmware's logic: its throttle ramp, brake, no timeout) -> the
                      actuator model -> Ravensim. esp32_loop:= is the bridge's
                      speed_loop.mode: pi (the default here) or open (the 2025-26 mapping)
                      legacy_impl:=rust (the default, aav_legacy_rs) or python picks the
                      bridge's language
  ghost:=true        a pretend person only navigation sees (aav_ghost), for stopping tests
                      with nothing in the world: pixi run ghost ahead 10, pixi run ghost off.
                      Its own costmap layer and stop zone source: it only adds
                      (docs/ghost-obstacles.md)
  actuator_model:=false  the old instant car instead of the actuator model (sim_adapter;
                      vehicle.yaml's actuators:), to compare the two
  actuators:="throttle_lag=0.3,drag=0.2"  the simulated car's own actuator numbers, over
                      vehicle.yaml's, while every driver still reads vehicle.yaml: a known
                      car for the hallway fits to find (car-test --hallway --sim, TODO 248)
  wheel_noise:=0.05 wheel_slip:=0.02 noise_seed:=1  noisy, slipping wheels (Ravensim's;
                      0: off)
  lag_scale:=2        the actuator model's throttle, brake and steering lags twice
                      vehicle.yaml's, while every driver still reads vehicle.yaml: how a
                      follower copes with a car slower than it was told (1.0: as written)
  speed_loop:=firm    the spine's speed loop profile the modelled car runs (default,
                      smooth, lqi, firm), and the one every model of the car assumes: the
                      path follower's (mppi_lag, aav_track's mpc_lag, mpc_lag_rs; those
                      two refuse lqi), ego's trace and the behaviour planner's stopping
                      law (TODO 163). The real car's spine runs default
  speed_loop_model:=smooth  the loop every model of the car assumes, when it is not the
                      car's own: the car runs speed_loop:=, the models plan with this
                      (TODO 163: car firm, models smooth). Empty (the default): speed_loop's
  simulator:=ravensim  Ravensim, our own simulator in Rust (aav_simulator,
                      docs/own-simulator.md), the only one since Gazebo was removed (16 Nov):
                      the worlds, the lidar and radar by ray casting, the cameras by Bevy
                      (aav_sim_cameras), no window.
  camera_lockstep:=auto  Ravensim's clock waits for each camera's picture: auto when
                      tools/software_gl.sh draws them (no GPU), true or false
  graphics:=auto      how much the cameras draw: pretty (grain, a sky, haze), simple (plain
                      colours, as Gazebo drew); auto: pretty on a GPU, simple in software

The empty and test_lot worlds render nothing, so they run headless anywhere, and the
camera's parts need world:=road. Every node runs on simulated time.
"""

import os

from aav_description import stacks, vehicle
from aav_description.lane_map import (check_lane_graph, check_lane_source,
                                      DEFAULT_LANE_GRAPH, DEFAULT_LANE_SOURCE)
from aav_sim_models import worlds
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

MAPS = {'empty': 'empty.yaml', 'test_lot': 'example_lot.yaml', 'road': 'empty.yaml',
        worlds.CAMPUS: os.path.abspath(worlds.CAMPUS_MAP)}
DEFAULT_NDT_MAP = os.path.join('maps', 'ndt', 'test_lot.pcd')


def campus_ndt_chunks(given, world, ndt_map):
    """
    Return ndt_chunks:= for NDT: '' for the whole map, else the chunks' folder.

    Empty (the default): the campus's chunks when pixi run campus made them and NDT is on
    the campus's own map (TODO 175: as good as the whole map, 15 Nov), else the whole map.
    off: the whole map. Anything else: that folder.
    """
    if given == 'off':
        return ''
    if given:
        return given
    chunks = worlds.CAMPUS_NDT_CHUNKS
    if (world == worlds.CAMPUS
            and os.path.abspath(ndt_map) == os.path.abspath(worlds.CAMPUS_NDT)
            and os.path.isfile(os.path.join(os.path.dirname(chunks), 'chunks.yaml'))):
        return chunks
    return ''


def campus_costmap_chunks(given, world):
    """
    Return costmap_chunks:= for navigation.launch.py: '' for the whole map, else a folder.

    off (or false): the whole map. auto (or true): the campus's chunks when pixi run
    campus made them, else the whole map (the test lot and the road world have none).
    Anything else: that folder.
    """
    given = given.strip()
    if given.lower() in ('', 'off', 'false'):
        return ''
    if given.lower() in ('auto', 'true'):
        chunks = worlds.CAMPUS_CHUNKS
        if world == worlds.CAMPUS and os.path.isfile(os.path.join(chunks, 'chunks.yaml')):
            return os.path.abspath(chunks)
        return ''
    return os.path.abspath(given)


# What each world shows in RViz: the road world adds the camera, the radar and the truth.
LAYOUTS = {'road': 'road.rviz', 'test_lot': 'road.rviz', worlds.CAMPUS: 'road.rviz'}
# The parts that read the camera, so need a world that renders it.
CAMERA_PARTS = ('perception', 'camera_obstacles')


def check_pose_source(word):
    """Return pose_source:= checked: gps (today's) or ndt (NDT leads, TODO 210)."""
    word = word.strip().lower()
    if word not in ('gps', 'ndt'):
        raise ValueError(f'pose_source:={word}: use gps or ndt')
    return word


def include(context, package, launch_file, **args):
    share = FindPackageShare(package).perform(context)
    return IncludeLaunchDescription(
        os.path.join(share, 'launch', launch_file),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


# The switches a stack (stack:=, config/stacks.yaml) may set. Each is declared empty
# (stacks.CHOSEN: "the stack's choice"), so any value given by hand wins; with no stack,
# an empty one takes its fallback here (the champions': ours since Nav2 went, 17 Nov).
STACK_FALLBACK = {'controller': vehicle.DEFAULT_CONTROLLER, 'tracker': vehicle.DEFAULT_TRACKER,
                  'planner': vehicle.DEFAULT_PLANNER, 'costmap': vehicle.DEFAULT_COSTMAP,
                  'navigator': vehicle.DEFAULT_NAVIGATOR, 'smoother': vehicle.DEFAULT_SMOOTHER,
                  'behaviour': 'false', 'ndt_ekf': 'false', 'kiss_ekf': 'false',
                  'lattice_moves': vehicle.LATTICE_MOVES[0],
                  'planner_search': vehicle.PLANNER_SEARCHES[0],
                  'plan_smoothing': 'nav2', 'map_correction': 'newest',
                  'mppi_lag_steer_gamma': '', 'mppi_lag_batch': '', 'mppi_lag_sampling': '',
                  'lane_pass_shape': '', 'ground': '', 'odom_lidar': '', 'things_tracker': '',
                  'lane_costs': 'false', 'lane_source': DEFAULT_LANE_SOURCE, 'lane_route': 'false',
                  'costmap_chunks': 'off', 'kerbs': 'false', 'things': 'false',
                  'lane_graph': DEFAULT_LANE_GRAPH,
                  'local_chunks': 'false', 'lane_pose': 'false'}


ODOM_LIDARS = ('kiss_icp', 'kinematic_icp')


def check_odom_lidar(value):
    """Return odom_lidar:= checked: kiss_icp (empty too) or kinematic_icp (TODO 216)."""
    value = value.strip().lower() or ODOM_LIDARS[0]
    if value not in ODOM_LIDARS:
        raise ValueError(f'odom_lidar:={value}: one of {", ".join(ODOM_LIDARS)}')
    return value


# Each level switch's options, for its help text (config/stacks.yaml).
LEVEL_OPTIONS = {level: tuple(stacks.load()['levels'][level]['options'])
                 for level in stacks.LEVEL_SWITCHES}


def launch_setup(context):
    def raw(name):
        return LaunchConfiguration(name).perform(context)

    # stack:= sets the switches in STACK_FALLBACK at once; one given by hand still wins.
    stack = raw('stack').strip().lower()
    league = stacks.load()
    levels = {level: raw(level) for level in stacks.LEVEL_SWITCHES}
    picked = stacks.resolve(league, stack, {n: raw(n) for n in STACK_FALLBACK},
                            STACK_FALLBACK, levels)
    print(f'[sim.launch.py] {stacks.describe(league, stack, levels)}', flush=True)

    def arg(name):
        return picked[name] if name in picked else raw(name)

    world = arg('world')
    maps = dict(MAPS)
    start_x, start_y, start_yaw = arg('start_x'), arg('start_y'), arg('start_yaw')
    ndt_map = arg('ndt_map')
    if worlds.is_world_file(world):
        # A twin (tools/twin_world.py, TODO 253): its own 2D map, NDT's points and start.
        worlds.world_file('', world)
        twin = worlds.twin(world)
        maps[world] = twin['map']
        if float(start_x) == 0.0 and float(start_y) == 0.0:
            start_x, start_y, start_yaw = (str(v) for v in twin['start'])
        if ndt_map == DEFAULT_NDT_MAP and twin['ndt']:
            ndt_map = twin['ndt']
    if world not in maps:
        raise ValueError(f"world:={world}: pick one of {sorted(MAPS)}, or a twin's .sdf")
    if world == worlds.CAMPUS:
        # Not on the world's origin unless someone asks: it's on a station shelter.
        worlds.world_file('', world)
        with open(worlds.CAMPUS_PLACE) as place:
            x, y, yaw = worlds.campus_start(place.read())
        if float(start_x) == 0.0 and float(start_y) == 0.0:
            start_x, start_y, start_yaw = str(x), str(y), str(yaw)
        if ndt_map == DEFAULT_NDT_MAP:
            ndt_map = worlds.CAMPUS_NDT
    ndt_chunks = campus_ndt_chunks(arg('ndt_chunks'), world, ndt_map)
    sim_speed = arg('sim_speed') or (worlds.CAMPUS_SPEED if world == worlds.CAMPUS else '')
    flag = {n: arg(n).lower() == 'true'
            for n in ('nav', 'teleop', 'rviz', 'perception', 'camera_obstacles', 'hud',
                      'map', 'foxglove', 'reversing', 'watchdog', 'faults',
                      'ghost', 'plan_around', 'watch_hud', 'geofence', 'voxels',
                      'screen_cubes', 'behaviour', 'things', 'kerbs')}
    lane = arg('lane').lower()
    if lane not in ('false', 'true'):
        raise ValueError(f'lane:={lane}: use true (the lane finder) or false')
    controller = arg('controller').lower()
    if controller not in vehicle.CONTROLLERS:
        raise ValueError(f'controller:={controller}: pick one of {", ".join(vehicle.CONTROLLERS)}')
    tracker = vehicle.check_tracker(arg('tracker'), flag['reversing'])
    # The simulated car's speed loop, and the one the followers', ego's and behaviour's
    # models assume: the car's own unless speed_loop_model:= says otherwise (TODO 163).
    speed_loop = vehicle.check_speed_loop(arg('speed_loop'))
    model_loop = vehicle.check_speed_loop_model(arg('speed_loop_model'), speed_loop,
                                                tracker if flag['nav'] else None)
    planner = vehicle.check_planner(arg('planner'))
    amcl = arg('amcl').lower() == 'true'
    # ndt_ekf: ekf_global takes NDT's pose, so NDT runs too.
    ndt_ekf = arg('ndt_ekf').lower() == 'true'
    # pose_source:=ndt: NDT leads ekf_global's pose, the GPS checks (TODO 210).
    pose_ndt = check_pose_source(arg('pose_source')) == 'ndt'
    ndt = arg('ndt').lower() == 'true' or ndt_ekf or pose_ndt
    truth_pose = arg('truth_pose').lower() == 'true'
    if truth_pose and (ndt_ekf or pose_ndt):
        raise ValueError('truth_pose:=true has no ekf_global for ndt_ekf:=true or '
                         'pose_source:=ndt to feed')
    lane_pose = arg('lane_pose').lower() == 'true'
    # Picked by the stack (ours' pose level since 17 Nov), not by hand: only where it can run
    # (an ekf_global, a camera, a lane map; below).
    lane_pose_stack = raw('lane_pose').strip() == stacks.CHOSEN
    if truth_pose and lane_pose and lane_pose_stack:
        lane_pose = False
    if truth_pose and lane_pose:
        raise ValueError('truth_pose:=true has no ekf_global for lane_pose:=true to feed')
    # kiss_ekf: ekf_local takes KISS-ICP's twist, so KISS-ICP runs too.
    kiss_ekf = arg('kiss_ekf').lower() == 'true'
    kiss_icp = arg('kiss_icp').lower() == 'true' or kiss_ekf
    models_share = FindPackageShare('aav_sim_models').perform(context)
    simulator = worlds.check_simulator(arg('simulator'))
    world_path = worlds.world_file(models_share, world)
    lane_map = worlds.lane_map_file(models_share, world, arg('lane_map').strip())
    # lane_costs: the lane map lane_map:= names, else the world's own, if it has one;
    # lane_source:=camera|both: the lane finder's lines too (TODO 264).
    lane_costs = ''
    lanes_on = arg('lane_costs').lower() == 'true'
    lane_source = check_lane_source(arg('lane_source')) if lanes_on else DEFAULT_LANE_SOURCE
    lane_camera = lane_source != 'map'
    if lanes_on and lane_source != 'camera':
        try:
            lane_costs = lane_map or worlds.lane_map_file(models_share, world, 'world')
        except ValueError as e:
            print(f'[sim.launch.py] lane_costs:=true, but no lane map here: {e}', flush=True)
    # lane_route (path:=lane): the lane map lane_map:= names, else the world's own.
    lane_route = ''
    if arg('lane_route').lower() == 'true':
        try:
            lane_route = lane_map or worlds.lane_map_file(models_share, world, 'world')
        except ValueError as e:
            print(f'[sim.launch.py] lane_route:=true, but no lane map here ({e}): '
                  'the lattice alone', flush=True)
    # The lidar and radar where the world has them; the cameras only where they're drawn.
    sensors = worlds.has_sensors(world_path)
    cameras = worlds.has_cameras(world_path, simulator) and arg('cameras').lower() == 'true'
    # lane_pose: the lane map lane_map:= names, else the world's own.
    pose_map = ''
    if lane_pose:
        try:
            pose_map = lane_map or worlds.lane_map_file(models_share, world, 'world')
        except ValueError:
            if not lane_pose_stack:
                raise
        if lane_pose_stack and not (pose_map and cameras):
            print("[sim.launch.py] the stack's lane_pose: off here (no camera or no lane "
                  'map)', flush=True)
            lane_pose, pose_map = False, ''
    # Each only where the world renders it, as ravensim.launch.py decides.
    radar = sensors and arg('radar').lower() == 'true'
    lidar = sensors and arg('lidar').lower() == 'true'
    wanted = ([n for n in CAMERA_PARTS if flag[n]] + (['lane'] if lane == 'true' else [])
              + (['lane_pose'] if lane_pose else [])
              + ([f'lane_source:={lane_source}'] if lane_camera else []))
    if amcl and not (world == 'test_lot' and flag['nav'] and lidar):
        raise ValueError('amcl:=true needs world:=test_lot nav:=true lidar:=true: the only '
                         'world with a real map, and AMCL reads the lidar')
    if flag['things'] and raw('things').strip() == stacks.CHOSEN and not (lidar and flag['nav']):
        # Picked by the stack (ours' behaviour level), not by hand: only where it can run.
        flag['things'] = False
    if flag['things'] and not (lidar and flag['nav']):
        raise ValueError('things:=true needs the lidar and nav:=true (navigation.launch.py '
                         'starts it, with voxels)')
    if (flag['voxels'] or flag['screen_cubes']) and not lidar:
        raise ValueError('voxels:=true needs the lidar (lidar:=true, world:=road or test_lot): '
                         'the cubes come from it')
    if ndt and not (world in ('test_lot', worlds.CAMPUS) and lidar):
        raise ValueError('ndt:=true (and ndt_ekf:=true) needs world:=test_lot or campus and '
                         'lidar:=true: the only worlds with '
                         'a lidar map (pixi run ndt-map drive), and NDT reads the lidar')
    if ndt and not os.path.isfile(ndt_map):
        raise ValueError(f'ndt:=true: no map at ndt_map:={ndt_map}. Make it with '
                         '`pixi run ndt-map drive`')
    if kiss_icp and not lidar:
        raise ValueError('kiss_icp:=true (and kiss_ekf:=true) needs the lidar: '
                         f'world:={world} lidar:={arg("lidar")} has none '
                         '(world:=test_lot or road, lidar:=true)')
    if wanted and not cameras:
        raise ValueError(f'{", ".join(wanted)}: world:={world} has no camera with '
                         f'simulator:={simulator} cameras:={arg("cameras")}. Use '
                         'world:=road, test_lot or campus, and cameras:=true')
    nav_share = FindPackageShare('aav_navigation').perform(context)
    actions = [
        # Every node on simulated time, those in the included files too.
        SetParameter(name='use_sim_time', value=True),
        include(context, 'aav_sim_models', 'ravensim.launch.py', world=world,
                obstacle=arg('obstacle'), start_x=start_x, start_y=start_y, start_yaw=start_yaw,
                lidar=arg('lidar'), radar=arg('radar'), chase_camera=arg('chase_camera'),
                cameras=str(cameras).lower(),
                actuator_model=arg('actuator_model'), speed_loop=speed_loop,
                payload=arg('payload'), adaptive_brake=arg('adaptive_brake'),
                lag_scale=arg('lag_scale'), actuators=arg('actuators'),
                wheel_noise=arg('wheel_noise'), wheel_slip=arg('wheel_slip'),
                noise_seed=arg('noise_seed'),
                faults=flag['faults'],
                spine=arg('spine'), arm=arg('arm'), bridge=arg('bridge'),
                arm_when_open_impl=vehicle.check_arm_when_open_impl(
                    arg('arm_when_open_impl')),
                esp32=arg('esp32'), esp32_loop=arg('esp32_loop'),
                legacy_impl=vehicle.check_legacy_impl(arg('legacy_impl')),
                camera_lockstep=arg('camera_lockstep'), graphics=arg('graphics')),
        include(context, 'aav_command_mux', 'mux.launch.py', use_sim_time='true',
                sensor_lock=flag['watchdog'], geofence_lock=flag['geofence']),
        # behaviour:=true: the road ahead's writer runs, so ego holds navigation to it. Its writer
        # runs inside navigation: with nav:=false there is no road ahead, and ego held the
        # point driver at 0 for ever waiting for one (TODO 94, 15 Nov).
        include(context, 'aav_ego', 'ego.launch.py', use_sim_time='true',
                sim_max_speed=sim_speed,
                road_ahead=str(flag['behaviour'] and flag['nav']).lower(),
                ego_impl=vehicle.check_ego_impl(arg('ego_impl')),
                # The models' speed loop: the car's unless told (TODO 163).
                speed_loop=model_loop),
        # ego's commands go to Ravensim (sim_adapter, or spine_bridge with spine:=true)
        # only while every source is SIM (aav_io_mux).
        include(context, 'aav_io_mux', 'io_mux.launch.py', target='sim', source='sim',
                io_mux_impl=vehicle.check_io_mux_impl(arg('io_mux_impl')),
                **({'screens': arg('watch_screens')} if arg('watch_screens') else {})),
        include(context, 'aav_scripted', 'scripted.launch.py', use_sim_time='true'),
        include(context, 'aav_drive_to_point', 'drive_to_point.launch.py', use_sim_time='true',
                watchdog=arg('watchdog'),
                drive_to_point_impl=vehicle.check_drive_to_point_impl(
                    arg('drive_to_point_impl'))),
        # Only the IMU filter: Ravensim stands in for the drivers.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick',
             parameters=[os.path.join(FindPackageShare('aav_sensors').perform(context),
                                      'config', 'imu_filter.yaml'), {'use_sim_time': True}],
             remappings=[('imu/data_raw', '/sensors/imu/data_raw'),
                         ('imu/data', '/sensors/imu/data')]),
        include(context, 'aav_localization', 'localization.launch.py', use_sim_time='true',
                ndt_ekf=str(ndt_ekf).lower(), kiss_ekf=str(kiss_ekf).lower(),
                pose_source='ndt' if pose_ndt else 'gps',
                pose_trust_score=(worlds.CAMPUS_POSE_TRUST
                                  if pose_ndt and world == worlds.CAMPUS else ''),
                global_ekf=str(not truth_pose).lower(),
                lane_pose=str(lane_pose).lower(), lane_map=pose_map,
                localization_health_impl=vehicle.check_localization_health_impl(
                    arg('localization_health_impl'))),
    ]
    if truth_pose:
        # The wiggle's first test: the truth as the pose, in ekf_global's place.
        actions.append(Node(package='aav_sim_models', executable='truth_localizer',
                            name='truth_localizer', parameters=[{'use_sim_time': True}]))
    # What this run uses, in words, for the cockpit's Algorithms table (aav_run_info).
    profile = f'sim world:={world}'
    actions.append(Node(package='aav_run_info', executable='run_info', name='run_info',
                        parameters=[{'use_sim_time': True, 'profile': profile,
                                     'nav': flag['nav'], 'planner': planner,
                                     'controller': controller, 'tracker': tracker,
                                     'lidar': lidar, 'radar': radar,
                                     'voxels': flag['voxels'], 'amcl': amcl, 'ndt': ndt,
                                     'ndt_ekf': ndt_ekf, 'kiss_icp': kiss_icp,
                                     'kiss_ekf': kiss_ekf}]))
    if radar:
        # The radar stand-in, as the bench's: few points per object, so small clusters.
        actions.append(include(context, 'aav_radar_clusters', 'radar_clusters.launch.py',
                               minimum_points='5'))
    if flag['nav']:
        # Where the world renders the radar and the lidar, our local costmap reads them,
        # and the stop zone guards navigation's commands, as on a car with them.
        actions.append(include(context, 'aav_navigation', 'navigation.launch.py',
                               use_sim_time='true',
                               map=os.path.join(nav_share, 'maps', maps[world]),
                               radar=radar, lidar=lidar, ghost=flag['ghost'],
                               voxels=flag['voxels'], kerbs=flag['kerbs'],
                               ground=vehicle.check_ground(arg('ground')),
                               screen_cubes=flag['screen_cubes'],
                               voxels_impl=vehicle.check_voxels_impl(arg('voxels_impl')),
                               lidar_impl=vehicle.check_lidar_impl(arg('lidar_impl')),
                               things_impl=vehicle.check_things_impl(arg('things_impl')),
                               stop_hold_impl=vehicle.check_stop_hold_impl(
                                   arg('stop_hold_impl')),
                               cube_floor=(worlds.CAMPUS_CUBE_FLOOR
                                           if world == worlds.CAMPUS and not flag['voxels']
                                           else ''),
                               stop_zone=radar or lidar or flag['ghost'],
                               camera_obstacles=flag['camera_obstacles'],
                               max_speed=sim_speed,
                               reversing=str(flag['reversing']).lower(),
                               controller=controller, tracker=tracker, planner=planner,
                               costmap=vehicle.check_costmap(arg('costmap')),
                               navigator=vehicle.check_navigator(arg('navigator')),
                               smoother=vehicle.check_smoother(arg('smoother')),
                               lattice_headings=arg('lattice_headings'),
                               plan_smoothing=vehicle.check_plan_smoothing(
                                   arg('plan_smoothing')),
                               mppi_lag_steer_gamma=arg('mppi_lag_steer_gamma'),
                               mppi_lag_batch=arg('mppi_lag_batch'),
                               mppi_lag_sampling=arg('mppi_lag_sampling'),
                               mpc_lag_tube=arg('mpc_lag_tube'),
                               lattice_moves=arg('lattice_moves'),
                               lidar_memory=arg('lidar_memory'),
                               global_window=arg('global_window'),
                               costmap_chunks=campus_costmap_chunks(arg('costmap_chunks'),
                                                                    world),
                               things_background=arg('things_background'),
                               things_background_below=arg('things_background_below'),
                               things_background_chunks=os.path.abspath(
                                   arg('things_background_chunks'))
                               if arg('things_background_chunks') else '',
                               things_names=arg('things_names'),
                               planner_search=arg('planner_search'),
                               diffusion_model=arg('diffusion_model'),
                               diffusion_record=arg('diffusion_record'),
                               diffusion_smoothing=arg('diffusion_smoothing'),
                               diffusion_impl=vehicle.check_diffusion_impl(
                                   arg('diffusion_impl')),
                               diffusion_provider=arg('diffusion_provider'),
                               # A trial tracker may be swapped live here (the debug
                               # HUD's panel shows which): the sim only, never the car.
                               live_swap='true',
                               amcl=amcl, amcl_x=start_x, amcl_y=start_y,
                               amcl_impl=vehicle.check_amcl_impl(arg('amcl_impl')),
                               amcl_yaw=start_yaw,
                               # The test lot's map for AMCL: finer, outlines only
                               # (maps/README.md). AMCL runs only in the test lot.
                               amcl_map=(os.path.join(nav_share, 'maps',
                                                      'example_lot_amcl.yaml')
                                         if amcl else ''),
                               plan_around=str(flag['plan_around']).lower(),
                               replan_hz=arg('replan_hz'),
                               map_correction=arg('map_correction'),
                               behaviour=str(flag['behaviour']).lower(),
                               behaviour_impl=vehicle.check_behaviour_impl(
                                   arg('behaviour_impl')),
                               things=str(flag['things']).lower(),
                               things_tracker=vehicle.check_things_tracker(
                                   arg('things_tracker')),
                               lane_map=lane_map, lane_costs=lane_costs,
                               lane_source=lane_source,
                               # Every model of the car: the car's loop unless told (TODO 163).
                               speed_loop=model_loop,
                               lane_route=lane_route, lane_pass=arg('lane_pass'),
                               lane_corners=arg('lane_corners'),
                               lane_pass_shape=arg('lane_pass_shape'),
                               lane_graph=check_lane_graph(arg('lane_graph')),
                               local_chunks=arg('local_chunks')))
    if ndt:
        # Scored only (measure controllers --ndt), unless ndt_ekf:=true: then ekf_global
        # reads its pose (measure controllers --ndt-ekf scores the difference).
        here = ({'ndt_min_score': worlds.CAMPUS_NDT_MIN_SCORE,
                 'ndt_min_height': worlds.CAMPUS_NDT_FLOOR} if world == worlds.CAMPUS else {})
        actions.append(include(context, 'aav_ndt', 'ndt.launch.py', use_sim_time='true',
                               ndt_map=ndt_map, ndt_chunks=ndt_chunks,
                               ndt_x=start_x, ndt_y=start_y,
                               ndt_yaw=start_yaw, **here))
    if kiss_icp:
        # Scored only (measure controllers --kiss-icp), unless kiss_ekf:=true: then
        # ekf_local reads its twist (measure controllers --kiss-ekf scores the difference).
        # odom_lidar:=kinematic_icp: Kinematic-ICP's idea in Rust (TODO 216, a challenger),
        # the same topics, so ekf_local can't tell; else KISS-ICP.
        if check_odom_lidar(arg('odom_lidar')) == 'kinematic_icp':
            actions.append(include(context, 'aav_kinematic_icp', 'kinematic_icp.launch.py',
                                   use_sim_time='true'))
        else:
            actions.append(include(context, 'aav_kiss_icp', 'kiss_icp.launch.py',
                                   use_sim_time='true'))
    if flag['watchdog']:
        # What this world has, and the camera only where something reads it: watching it
        # costs one more stream of pictures.
        camera = cameras and (flag['perception'] or flag['camera_obstacles'] or lane != 'false'
                              or lane_pose or lane_camera or flag['faults'])
        # NDT leading: the GPS's checks warn only, pose_check's HOLD stops (TODO 210).
        gps = ['gps_check', 'pose_check'] if pose_ndt else ['gps']
        inputs = (['odom', 'imu'] + gps + ['local', 'global'] + (['camera'] if camera else [])
                  + (['radar'] if radar else [])
                  # The roof lidar, where this world has one (TODO 246's step 5e): the stop
                  # zone and the costmap see through it, so a dead one stops, as the camera.
                  + (['lidar'] if lidar else [])
                  # The steering asked against measured (TODO 207): the adapter or
                  # spine_bridge publishes it; not checked yet with the ESP32 in the loop.
                  + ([] if arg('esp32').lower() == 'true' else ['steering'])
                  # Ego's speed against the wheels' (TODO 207): a stuck throttle.
                  + ['speed']
                  + (['hud'] if flag['hud'] or flag['watch_hud'] else [])
                  + our_nodes(nav=flag['nav'], costmap=vehicle.check_costmap(arg('costmap')),
                              planner=planner, tracker=tracker,
                              navigator=vehicle.check_navigator(arg('navigator')),
                              smoother=vehicle.check_smoother(arg('smoother')),
                              behaviour=flag['behaviour'],
                              cubes=(flag['voxels'] or flag['screen_cubes']
                                     or flag['things'] or (flag['kerbs'] and lidar)),
                              things=flag['things']))
        actions.append(include(context, 'aav_safety', 'watchdog.launch.py',
                               inputs=','.join(inputs), use_sim_time='true',
                               watchdog_impl=vehicle.check_watchdog_impl(
                                   arg('watchdog_impl'))))
    # The goal leash (TODO 116): a laptop driver (aav_language) that goes quiet loses its
    # goals, as on the car. Does nothing until the language driver beats.
    actions.append(Node(package=vehicle.GOAL_LEASH_IMPLS[
                            vehicle.check_goal_leash_impl(arg('goal_leash_impl'))],
                        executable='goal_leash', name='goal_leash',
                        parameters=[{'use_sim_time': True}]))
    if flag['geofence']:
        actions.append(include(context, 'aav_safety', 'geofence.launch.py',
                               backend=('spine' if arg('spine').lower() == 'true' else
                                        'sim_esp32' if arg('esp32').lower() == 'true'
                                        else 'sim'),
                               radius=fence_radius(arg('geofence_radius'), world),
                               centre_x=start_x,
                               centre_y=start_y, max_speed=sim_speed,
                               fix_topic=('/localization/pose_check/fix' if pose_ndt
                                          else '/localization/gps/odom'),
                               geofence_impl=vehicle.check_geofence_impl(
                                   arg('geofence_impl')),
                               use_sim_time='true'))
    if flag['teleop']:
        actions.append(include(context, 'aav_teleop', 'teleop.launch.py', use_sim_time='true'))
    if flag['perception']:
        actions += [
            include(context, 'aav_detector', 'detector.launch.py', labels='stop sign',
                    use_sim_time='true',
                    detector_impl=vehicle.check_detector_impl(arg('detector_impl'))),
            # With things named (TODO 192), the lidar gives the stop sign's distance.
            Node(package=vehicle.STOP_SIGN_IMPLS[
                     vehicle.check_stop_sign_impl(arg('stop_sign_impl'))],
                 executable='stop_sign_watcher', name='stop_sign_watcher',
                 parameters=[{'use_sim_time': True,
                              'lidar_distance': flag['things']
                              and arg('things_names').lower() == 'true'}]),
        ]
    if flag['camera_obstacles']:
        actions.append(Node(package='aav_detector', executable='camera_obstacles',
                            name='camera_obstacles',
                            parameters=[os.path.join(FindPackageShare('aav_detector')
                                                     .perform(context), 'config',
                                                     'camera_obstacles.yaml'),
                                        {'use_sim_time': True}]))
    if lane == 'true' or lane_pose or lane_camera:
        actions.append(include(
            context, 'aav_lane_finder', 'lane_finder.launch.py',
            lane_finder_impl=vehicle.check_lane_finder_impl(arg('lane_finder_impl'))))
    if flag['hud']:
        actions.append(include(context, 'aav_hud', 'hud.launch.py'))
    if flag['map']:
        actions.append(include(context, 'aav_hud', 'map.launch.py'))
    if flag['foxglove']:
        # This computer only, like the rest of the simulator (tools/ros.sh --local), and
        # the front camera's small preview where the world renders cameras, as on the car.
        actions.append(include(context, 'aav_bringup', 'foxglove.launch.py',
                               foxglove_address='127.0.0.1', foxglove_port=8765,
                               use_sim_time=True, foxglove_control=True,
                               foxglove_topics='wifi',
                               foxglove_preview='front_left' if cameras else ''))
    if flag['rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(FindPackageShare('aav_bringup').perform(context),
                                          'rviz', LAYOUTS.get(world, 'drive.rviz'))],
            parameters=[{'use_sim_time': True}]))
    return actions


# m: the geofence's circle when geofence_radius is empty. On the campus 200 m held the car
# at a junction 197 m from the start, half way round a tour (15 Nov).
FENCE_RADIUS = 200.0
CAMPUS_FENCE_RADIUS = 600.0


def fence_radius(given, world):
    """Return the geofence's radius as text: the one given, else by the world."""
    if given.strip():
        return given.strip()
    return str(CAMPUS_FENCE_RADIUS if world == worlds.CAMPUS else FENCE_RADIUS)


def our_nodes(nav, costmap, planner, tracker, navigator, smoother, behaviour, cubes,
              things):
    """
    Return the watchdog's node inputs for our own nodes this launch starts (TODO 180).

    Ben, 14 Nov: the watchdog warns about our nodes, warn only. The names are
    config/sensor_watchdog.yaml's node_* inputs; ego and look_ahead always run. And
    (Ben, 18 Nov, TODO 171) the status_* inputs: the same nodes in their own words, on
    their status topics, warn only too.
    """
    names = ['node_ego', 'node_look_ahead']
    if nav:
        names += [name for name, on in (
            ('node_costmap', costmap == 'own'), ('node_planner', planner == 'own'),
            ('node_navigator', navigator == 'own'), ('node_mppi_lag', tracker == 'mppi_lag'),
            ('node_smoother', smoother == 'own'), ('node_behaviour', behaviour),
            ('node_voxels', cubes), ('node_things', things),
            ('status_costmap_local', costmap == 'own'),
            ('status_costmap_global', costmap == 'own'),
            ('status_planner', planner == 'own'),
            ('status_navigator', navigator == 'own'),
            ('status_mppi_lag', tracker == 'mppi_lag')) if on]
    return names


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stack', default_value=stacks.DEFAULT,
                              description='the league (config/stacks.yaml): ours (the '
                              'default), best or car sets the planner, path '
                              'follower, costmap, navigator, smoother, behaviour and pose '
                              'at once; a switch given by hand still wins. Empty: every '
                              "switch as given, else the champions'"),
        *[DeclareLaunchArgument(level, default_value=stacks.CHOSEN,
                                description=f'the {level} level by its league name '
                                f'(config/stacks.yaml: {", ".join(options)}), over the '
                                "stack's pick; a single switch given by hand still wins. "
                                "Empty: the stack's choice")
          for level, options in LEVEL_OPTIONS.items()],
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty, test_lot or road'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true if a joystick is plugged into this computer'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='the object detector and the stop-sign watcher (the ml '
                                          'environment; world:=road)'),
        DeclareLaunchArgument('lane', default_value='false',
                              description='true: the lane finder, on '
                                          '/perception/lane/geometry (world:=road, campus)'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="the detectors' boxes into the local costmap "
                                          '(world:=road)'),
        DeclareLaunchArgument('hud', default_value='false',
                              description='the HUD window (aav_hud); needs a screen'),
        DeclareLaunchArgument('map', default_value='false',
                              description="the big screen's map window (aav_hud); "
                                          'needs a screen'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='foxglove_bridge on 127.0.0.1:8765 (Lichtblick)'),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="a 1 m box in the car's lane, 15 m ahead"),
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='m: start the car this far ahead (east) of the origin'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description="m: start the car this far left of the road's centre"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of east '
                                          '(the EKFs still start facing east)'),
        DeclareLaunchArgument('chase_camera', default_value='true',
                              description='a camera behind and above the car, for '
                                          "Lichtblick's cockpit layout (world:=road or "
                                          'test_lot)'),
        DeclareLaunchArgument('cameras', default_value='true',
                              description='false: no cameras, so Ravensim runs on a '
                              'computer with no GPU (the parts that read them are refused)'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='a simulated 16-beam roof lidar, on '
                                          '/sensors/lidar/points, read by the costmap '
                                          '(world:=road or test_lot)'),
        DeclareLaunchArgument('radar', default_value='true',
                              description='the radar stand-in (world:=road or test_lot); '
                                          'false: the lidar alone guards the car'),
        DeclareLaunchArgument('actuator_model', default_value='true',
                              description="the actuators as vehicle.yaml's actuators: say; "
                                          'false: the old instant car'),
        DeclareLaunchArgument('actuators', default_value='',
                              description="the simulated car's own actuator numbers over "
                                          'vehicle.yaml\'s ("throttle_lag=0.3,drag=0.2"); the '
                                          'drivers keep vehicle.yaml (TODO 248)'),
        DeclareLaunchArgument('wheel_noise', default_value='0',
                              description='m/s of noise on each wheel speed (0: off)'),
        DeclareLaunchArgument('wheel_slip', default_value='0',
                              description='the wheels read this share too far (0: off)'),
        DeclareLaunchArgument('noise_seed', default_value='1',
                              description="the sensors' noise seed"),
        DeclareLaunchArgument('lag_scale', default_value='1.0',
                              description="the actuator model's lags times this (a "
                                          'robustness test; the drivers keep vehicle.yaml)'),
        DeclareLaunchArgument('speed_loop', default_value='default',
                              description="the speed loop's profile: default (today's gains)"
                                          ', smooth (TODO 27), lqi (TODO 3) or firm (TODO '
                                          '163); every model of the car assumes it, '
                                          'unless speed_loop_model says otherwise'),
        DeclareLaunchArgument('speed_loop_model', default_value='',
                              description='the loop every model of the car assumes '
                                          "(ego's trace, the follower, the behaviour "
                                          "planner); empty: speed_loop's (TODO 163)"),
        DeclareLaunchArgument('payload', default_value='0',
                              description='kg aboard the modelled car: it speeds up and '
                                          'brakes less (ego is not told)'),
        DeclareLaunchArgument('adaptive_brake', default_value='false',
                              description='true: brake harder when the car slows less than '
                                          'expected, within vehicle.yaml brake: '
                                          '(docs/stopping.md); not with spine or esp32'),
        DeclareLaunchArgument('spine', default_value='false',
                              description='the spine in the loop: spine_bridge, CAN frames '
                                          "and spine_sim (the spine's own logic) before the "
                                          'actuator model'),
        DeclareLaunchArgument('arm', default_value='true',
                              description='with spine:=true: arm the spine after 12 s'),
        DeclareLaunchArgument('esp32', default_value='false',
                              description="today's car in the loop: legacy_uart_bridge and "
                                          'sim_esp32, the C++ firmware standing in'),
        DeclareLaunchArgument('legacy_impl', default_value=vehicle.DEFAULT_LEGACY_IMPL,
                              description='with esp32:=true: legacy_uart_bridge in rust '
                                          '(aav_legacy_rs) or python (aav_legacy)'),
        DeclareLaunchArgument('esp32_loop', default_value='pi',
                              description="with esp32:=true: the bridge's speed_loop.mode, "
                                          'pi or open'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='with spine:=true: spine_bridge in rust '
                                          '(aav_spine_bridge) or python (aav_spine)'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description='the sensor watchdog: stops navigation and the AI slot '
                                          'when a sensor they need fails (aav_safety)'),
        DeclareLaunchArgument('geofence', default_value='true',
                              description='the geofence: holds every program near the edge of '
                                          'a circle round the start (aav_safety)'),
        DeclareLaunchArgument('geofence_radius', default_value='',
                              description="m: the geofence's circle; empty: 600 on the "
                                          'campus (its routes go 250 m and more from the '
                                          "start), else 200 (the car's: 30)"),
        DeclareLaunchArgument('watch_hud', default_value='false',
                              description="the watchdog also wants the HUD's heartbeat, "
                                          'with no HUD window (hud:=true watches it anyway)'),
        DeclareLaunchArgument('faults', default_value='false',
                              description='route the front left picture through '
                                          'sim_adapter, for fault.camera'),
        DeclareLaunchArgument('watch_screens', default_value='',
                              description='screens whose heartbeat commands need, such as '
                                          'dev_window,hw_panel: lose one and, with '
                                          'spine:=true, the spine latches'),
        DeclareLaunchArgument('screen_cubes', default_value='false',
                              description='true: the cubes for the developer window, while '
                                          'the costmap stays on the flat scan'),
        DeclareLaunchArgument('things', default_value=stacks.CHOSEN,
                              description='true: the cubes as tracked things on /world/things '
                                          '(aav_things_rs; with nav and the lidar). Empty: the '
                                          "stack's choice (on in ours since 17 Nov, where the "
                                          'lidar and nav run)'),
        DeclareLaunchArgument('voxels', default_value='false',
                              description='true: the costmap reads the lidar as 0.25 m cubes '
                                          '(aav_voxels, a trial) instead of the flat scan'),
        DeclareLaunchArgument('lidar_impl', default_value=vehicle.DEFAULT_LIDAR_IMPL,
                              description='python or rust: which lidar_scan flattens the '
                                          'lidar (aav_lidar, or aav_lidar_rs)'),
        DeclareLaunchArgument('drive_to_point_impl',
                              default_value=vehicle.DEFAULT_DRIVE_TO_POINT_IMPL,
                              description='rust or python: which point_driver drives to a '
                                          'point (aav_drive_to_point_rs, or aav_drive_to_point)'),
        DeclareLaunchArgument('arm_when_open_impl',
                              default_value=vehicle.DEFAULT_ARM_WHEN_OPEN_IMPL,
                              description='rust or python: which arm_when_open arms the spine '
                                          "(aav_spine_bridge's, or aav_spine's)"),
        DeclareLaunchArgument('lane_finder_impl',
                              default_value=vehicle.DEFAULT_LANE_FINDER_IMPL,
                              description='rust or python: which lane_finder finds the lane '
                                          'lines (aav_lane_finder_rs, or aav_lane_finder)'),
        DeclareLaunchArgument('things_impl', default_value=vehicle.DEFAULT_THINGS_IMPL,
                              description='rust (aav_things_rs): which things node tracks '
                                          'the cubes (the Python removed 20 Nov)'),
        DeclareLaunchArgument('voxels_impl', default_value=vehicle.DEFAULT_VOXELS_IMPL,
                              description='python or rust: which voxels node cuts the '
                                          'cubes (aav_voxels, or aav_voxels_rs)'),
        DeclareLaunchArgument('detector_impl', default_value=vehicle.DEFAULT_DETECTOR_IMPL,
                              description='python or rust: which object detector runs, '
                                          'with perception:=true (aav_detector, or '
                                          'aav_detector_rs: ONNX Runtime)'),
        DeclareLaunchArgument('geofence_impl', default_value=vehicle.DEFAULT_GEOFENCE_IMPL,
                              description='rust (aav_safety_rs; the Python removed 20 Nov): '
                                          'the geofence'),
        DeclareLaunchArgument('stop_hold_impl', default_value=vehicle.DEFAULT_STOP_HOLD_IMPL,
                              description='rust (aav_safety_rs; the Python removed 20 Nov): '
                                          'stop_hold, with navigation'),
        DeclareLaunchArgument('goal_leash_impl', default_value=vehicle.DEFAULT_GOAL_LEASH_IMPL,
                              description='rust (aav_safety_rs; the Python removed 20 Nov): '
                                          'the goal leash'),
        DeclareLaunchArgument('watchdog_impl', default_value=vehicle.DEFAULT_WATCHDOG_IMPL,
                              description='rust (aav_watchdog_rs): which sensor watchdog '
                                          'runs (the Python removed 20 Nov)'),
        DeclareLaunchArgument('stop_sign_impl', default_value=vehicle.DEFAULT_STOP_SIGN_IMPL,
                              description='python or rust: which stop_sign_watcher runs, '
                                          'with perception:=true (aav_detector, or '
                                          'aav_stop_sign_rs)'),
        DeclareLaunchArgument('io_mux_impl', default_value=vehicle.DEFAULT_IO_MUX_IMPL,
                              description='python or rust: which sensor mux and output mux '
                                          'run (aav_io_mux, or aav_io_mux_rs)'),
        DeclareLaunchArgument('localization_health_impl',
                              default_value=vehicle.SIM_DEFAULT_LOCALIZATION_HEALTH_IMPL,
                              description='rust (aav_localization_health_rs): which '
                                          'localization_health runs'),
        DeclareLaunchArgument('behaviour_impl',
                              default_value=vehicle.SIM_DEFAULT_BEHAVIOUR_IMPL,
                              description='rust (aav_behaviour_rs): which behaviour node '
                                          'runs, with behaviour on'),
        DeclareLaunchArgument('ego_impl', default_value=vehicle.SIM_DEFAULT_EGO_IMPL,
                              description='python or rust: which ego runs (aav_ego, or '
                                          'aav_ego_rs: ego and look_ahead in one process)'),
        DeclareLaunchArgument('ghost', default_value='false',
                              description='a pretend person only navigation sees (aav_ghost); '
                                          'place it with pixi run ghost ahead 10'),
        DeclareLaunchArgument('sim_speed', default_value='',
                              description='m/s, up to 5: a higher top speed for ego and the '
                                          "follower than the car's 1.0, simulators only"),
        DeclareLaunchArgument('reversing', default_value='true',
                              description='the car may drive backwards, at up to '
                                          "vehicle.yaml's max_reverse_speed "
                                          '(docs/learn-nav2.md, Reversing)'),
        DeclareLaunchArgument('controller', default_value=stacks.CHOSEN,
                              description="mppi: nav2.yaml's MPPI block, which mppi_lag "
                                          "reads (Nav2's controller_server went, 17 Nov). "
                                          "Empty: the stack's choice"),
        DeclareLaunchArgument('planner', default_value=stacks.CHOSEN,
                              description=', '.join(vehicle.PLANNERS) + ': the planner. '
                                          "own (ours) keeps to the car's turning radius; "
                                          'diffusion is a trial (docs/own-planner.md). '
                                          "Empty: the stack's choice"),
        DeclareLaunchArgument('diffusion_model', default_value='',
                              description='with planner:=diffusion: the model (.onnx); '
                                          'empty: models/diffusion/planner.onnx. A file '
                                          "that is not there: every plan the lattice's "
                                          '(the fallback test)'),
        DeclareLaunchArgument('diffusion_smoothing', default_value='gear',
                              description="with planner:=diffusion: gear (each gear's run "
                                          'smoothed on its own) or average (2 Nov)'),
        DeclareLaunchArgument('diffusion_record', default_value='',
                              description="with planner:=diffusion: a folder for each plan's "
                                          'inputs, to replay offline (training/replay.py)'),
        DeclareLaunchArgument('diffusion_impl', default_value=vehicle.DEFAULT_DIFFUSION_IMPL,
                              description="with planner:=diffusion: the model's worker: "
                                          'python (worker.py) or rust (aav_diffusion_rs, '
                                          'ONNX Runtime from Rust; docs/onnx.md)'),
        DeclareLaunchArgument('diffusion_provider', default_value='CPUExecutionProvider',
                              description="with planner:=diffusion: ONNX Runtime's engine "
                                          '(CPUExecutionProvider, CoreMLExecutionProvider...; '
                                          'the Rust worker also auto, coreml, tensorrt)'),
        DeclareLaunchArgument('mppi_lag_batch', default_value=stacks.CHOSEN,
                              description='with tracker:=mppi_lag: its samples a tick '
                                          '(TODO 152): 1000 (steer:=mppi_lag_1k); '
                                          "nav2.yaml's 2000 when nothing sets it"),
        DeclareLaunchArgument('mpc_lag_tube', default_value='',
                              description='with tracker:=mpc_lag_rs: m each side of the '
                                          'path where being off it costs nothing (the '
                                          'tube, Ben 21 Nov); empty: 0.3 (tuned); 0: off'),
        DeclareLaunchArgument('mppi_lag_sampling', default_value=stacks.CHOSEN,
                              description='with tracker:=mppi_lag: correlated (the champion) '
                                          "or smooth, noise on the steering's change (TODO "
                                          '212, a challenger: steer:=mppi_lag_smooth)'),
        DeclareLaunchArgument('mppi_lag_steer_gamma', default_value=stacks.CHOSEN,
                              description="with tracker:=mppi_lag: its steering's control cost "
                                          "weight (TODO 168): 0.05; nav2.yaml's gamma when "
                                          "nothing sets it. Empty: the stack's choice (ours: "
                                          "0.05; best: nav2.yaml's)"),
        DeclareLaunchArgument('plan_smoothing', default_value=stacks.CHOSEN,
                              description="the planner's smoother: nav2, Nav2's weights; "
                                          "strong, it also takes out the lattice's staircase "
                                          "(TODO 168). Empty: the stack's choice (ours: "
                                          'strong; best: nav2)'),
        DeclareLaunchArgument('lattice_headings',
                              default_value=str(vehicle.LATTICE_HEADINGS[0]),
                              description='with planner:=own: 16 headings (the default) '
                                          'or 32 (a trial file)'),
        DeclareLaunchArgument('lidar_memory', default_value='0.0',
                              description='s the local lidar layer keeps a mark after the '
                                          'lidar loses it (0: the newest scan only; TODO 188)'),
        DeclareLaunchArgument('global_window', default_value='0.0',
                              description='m: the global costmap a square rolling with the '
                                          'car (192: the 3 x 3 chunks; 0: the whole map)'),
        DeclareLaunchArgument('things_background', default_value='true',
                              description='with things:=true: low cubes on the saved map '
                                          'are not things (TODO 175 step 5; the default '
                                          'since 16 Nov); false: every cube'),
        DeclareLaunchArgument('things_background_chunks', default_value='',
                              description='with things_background: a folder of the NDT '
                                          "map's chunks (maps/campus/chunks/ndt) as the "
                                          'background instead of /map'),
        DeclareLaunchArgument('things_background_below', default_value='0.7',
                              description='with things_background: m, only cubes lower '
                                          'than this drop (kerbs, grass; not a body). '
                                          'Empty: any height'),
        DeclareLaunchArgument('things_names', default_value='false',
                              description='with things:=true: things take the camera '
                                          "detector's labels (TODO 192)"),
        DeclareLaunchArgument('lattice_moves', default_value=stacks.CHOSEN,
                              description="nav2, the moves Nav2's generator made; "
                                          'moves; smooth or smooth_fast, a trial '
                                          "(tools/lattice_smooth.py). Empty: the stack's "
                                          "choice (nav2's moves)"),
        DeclareLaunchArgument('planner_search', default_value=stacks.CHOSEN,
                              description='lattice, the State Lattice; '
                                          "hybrid, Nav2's Hybrid-A* in Rust (a challenger). "
                                          "Empty: the stack's choice (the lattice)"),
        DeclareLaunchArgument('behaviour', default_value=stacks.CHOSEN,
                              description='true: the behaviour states, the road ahead and '
                                          'a speed cap (aav_behaviour_rs); false: none. '
                                          "Empty: the stack's choice (on in ours)"),
        DeclareLaunchArgument('lane_map', default_value='',
                              description="with behaviour:=true: world (the world's lane "
                                          "map) or a lanes.yaml: the road ahead's line on "
                                          "the lane's centre; empty: /plan's line"),
        DeclareLaunchArgument('kerbs', default_value=stacks.CHOSEN,
                              description='true: the kerbs, where the lidar runs (the Rust '
                                          "voxels node's step finder, the local costmap's "
                                          'kerb_layer; TODO 119); false: none. Empty: the '
                                          "stack's choice (on in ours)"),
        DeclareLaunchArgument('lane_costs', default_value=stacks.CHOSEN,
                              description='true: the lanes layer in our global costmap, '

                                          "lowest at the right lane's centre (TODO 148); "
                                          "false: none. Empty: the stack's choice (off)"),
        DeclareLaunchArgument('lane_source', default_value=stacks.CHOSEN,
                              description="with lane_costs:=true: map (the lane map's roads), "
                                          "camera (the lane finder's lines, which it then "
                                          'starts; TODO 264) or both (the map wins where it '
                                          "has a road). Empty: the stack's choice (map)"),
        DeclareLaunchArgument('costmap_chunks', default_value=stacks.CHOSEN,
                              description="with costmap:=own: auto, the map's 64 m chunks "
                                          'round the car and along the route (the '
                                          "campus's, when pixi run campus made them); off, "
                                          'the whole map; or a folder (TODO 175). Empty: '
                                          "the stack's choice (auto in ours)"),
        DeclareLaunchArgument('lane_route', default_value=stacks.CHOSEN,
                              description='true: the lane-centre planner (path:=lane): '
                                          "the right lane's centre along the lane map, "
                                          'planner:=own (TODO 148); false: none. Empty: '
                                          "the stack's choice (on in ours)"),
        DeclareLaunchArgument('lane_graph', default_value=stacks.CHOSEN,
                              description="with lane_route: lanes_yaml (the lane map's "
                                          'roads) or lanelet2 (the Lanelet2 map beside it, '
                                          'maps/campus/campus.osm; TODO 175). Empty: the '
                                          "stack's choice"),
        DeclareLaunchArgument('local_chunks', default_value=stacks.CHOSEN,
                              description="true, with costmap_chunks: the local costmap's "
                                          'static layer from the chunks round the car too, '
                                          "not /map (TODO 175). Empty: the stack's choice"),
        DeclareLaunchArgument('lane_pass', default_value='true',
                              description='with lane_route: true, a parked obstacle is '
                                          'passed by moving only its stretch left; false: '
                                          "the whole path is the lattice's (before 16 Nov)"),
        DeclareLaunchArgument('ground', default_value=stacks.CHOSEN,
                              description='plane (the champion: one road plane a cloud) or '
                                          'patchwork: a plane a patch, ring by ring, so a lower '
                                          'kerb is a cube, the costmap on the cubes (TODO 215, '
                                          'a challenger: kerbs:=patchwork)'),
        DeclareLaunchArgument('lane_pass_shape', default_value=stacks.CHOSEN,
                              description='with lane_route: the pass round a parked obstacle, '
                                          'cosine (the default) or jerk, a QP holding the '
                                          'lateral jerk (TODO 214, a challenger: '
                                          'path:=lane_jerk)'),
        DeclareLaunchArgument('lane_corners', default_value='arc',
                              description="with lane_route: the lane line's corners, "
                                          'arc (circle arcs, the default) or clothoid '
                                          '(continuous curvature, TODO 211, a challenger)'),
        DeclareLaunchArgument('navigator', default_value=stacks.CHOSEN,
                              description="own (aav_navigator, in bt_navigator's place; "
                                          'docs/own-navigator.md), the only one since Nav2 '
                                          "went. Empty: the stack's choice"),
        DeclareLaunchArgument('smoother', default_value=stacks.CHOSEN,
                              description='own: our velocity smoother, for a car '
                                          '(aav_smoother; docs/own-smoother.md), the only '
                                          "one since Nav2 went. Empty: the stack's choice"),
        DeclareLaunchArgument('costmap', default_value=stacks.CHOSEN,
                              description='own (aav_costmap; docs/own-costmap.md), the only '
                                          "one since Nav2 went. Empty: the stack's choice"),
        DeclareLaunchArgument('tracker', default_value=stacks.CHOSEN,
                              description='the path follower: ' +
                                          ', '.join(vehicle.TRACKERS) +
                                          " (aav_track's with reversing:=false; "
                                          'docs/trials.md). Empty: '
                                          "the stack's choice"),
        DeclareLaunchArgument('amcl', default_value='false',
                              description='AMCL beside the EKF, scored only '
                                          '(world:=test_lot nav:=true lidar:=true)'),
        DeclareLaunchArgument('amcl_impl', default_value=vehicle.DEFAULT_AMCL_IMPL,
                              description='rust: which AMCL (aav_amcl_rs; stock nav2_amcl '
                                          'removed 20 Nov)'),
        DeclareLaunchArgument('ndt', default_value='false',
                              description='NDT on the saved lidar map, scored only '
                                          '(world:=test_lot lidar:=true, ndt_map present)'),
        DeclareLaunchArgument('ndt_ekf', default_value=stacks.CHOSEN,
                              description='a trial: NDT (ndt:=true implied) and its pose '
                                          "into ekf_global, beside the GPS. Empty: the stack's "
                                          'choice (false)'),
        DeclareLaunchArgument('pose_source', default_value='gps',
                              description="gps (today's) or ndt: NDT leads ekf_global's "
                                          'pose, the GPS checks it (a challenger, TODO 210)'),
        DeclareLaunchArgument('ndt_chunks', default_value='',
                              description='with ndt:=true: the map in 64 m chunks from this '
                                          "folder (TODO 175). Empty: the campus's chunks "
                                          'when pixi run campus made them, else the whole '
                                          'map; off: the whole map'),
        DeclareLaunchArgument('ndt_map', default_value=DEFAULT_NDT_MAP,
                              description="NDT's map (pixi run ndt-map drive); relative to "
                                          'where the launch starts'),
        DeclareLaunchArgument('kiss_icp', default_value='false',
                              description='KISS-ICP lidar odometry, scored only (a world '
                                          'with the lidar; pixi install --locked -e kiss)'),
        DeclareLaunchArgument('things_tracker', default_value=stacks.CHOSEN,
                              description='with things:=true: constant_velocity (the champion) '
                                          'or per_kind, a motion model per kind (TODO 217, a '
                                          'challenger: behaviour:=on_per_kind)'),
        DeclareLaunchArgument('odom_lidar', default_value=stacks.CHOSEN,
                              description='with kiss_icp:=true or kiss_ekf:=true: kiss_icp (the '
                                          'champion) or kinematic_icp, lidar odometry that '
                                          "knows the car can't slide sideways, the wheels its "
                                          'prior (TODO 216, a challenger: pose:=kinematic_ekf)'),
        DeclareLaunchArgument('kiss_ekf', default_value=stacks.CHOSEN,
                              description='a trial: KISS-ICP (kiss_icp:=true implied) and its '
                                          'twist into ekf_local, beside the wheels and IMU. '
                                          "Empty: the stack's choice (false)"),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description='true: the planner plans round what the radar '
                                          'and lidar see; false: the map only, so the car '
                                          'stops for it'),
        DeclareLaunchArgument('lane_pose', default_value=stacks.CHOSEN,
                              description='the lane finder as a pose sensor into ekf_global '
                                          "(aav_lane_pose, TODO 155). Empty: the stack's "
                                          'choice (on in ours since 17 Nov, where a camera '
                                          'and a lane map are)'),
        DeclareLaunchArgument('truth_pose', default_value='false',
                              description="a test: the simulator's truth as the pose, in "
                                          "ekf_global's place (aav_sim_models truth_localizer)"),
        DeclareLaunchArgument('map_correction', default_value=stacks.CHOSEN,
                              description='with a trial tracker: newest, or slow (the held '
                                          "map -> odom, TODO 186). Empty: the stack's "
                                          'choice (ours: slow; best: newest)'),
        DeclareLaunchArgument('replan_hz', default_value='1.0',
                              description='how often the navigator plans a to-pose goal '
                                          'again; 0: once per goal (a test)'),
        DeclareLaunchArgument('simulator', default_value=worlds.DEFAULT_SIMULATOR,
                              description='ravensim: our own simulator (aav_simulator, '
                                          'aav_sim_cameras), the only one; own: its old '
                                          'name, for one release'),
        DeclareLaunchArgument('camera_lockstep', default_value='auto',
                              description="Ravensim's clock waits for each picture: auto "
                                          '(when tools/software_gl.sh draws them), true or '
                                          'false'),
        DeclareLaunchArgument('graphics', default_value='auto',
                              description='how much the cameras draw: pretty, simple, or '
                                          'auto (pretty on a GPU, simple in software)'),
        OpaqueFunction(function=launch_setup),
    ])

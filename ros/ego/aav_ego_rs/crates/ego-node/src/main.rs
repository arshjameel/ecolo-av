//! ego and look_ahead in Rust, one process, two nodes: a drop-in for aav_ego's two Python
//! nodes (ego_node.py, look_ahead_node.py), with the same node names, topics, QoS,
//! parameters and answers. The maths is the `ego-core` crate, with no ROS.
//!
//! Node `ego`
//! Subscribes  control/cmd_vel   geometry_msgs/TwistStamped (or Twist, use_stamped false)
//!             vehicle/odom      nav_msgs/Odometry: the measured speed, for the trace's twin
//!             drivers/<d>/lag_aware  std_msgs/Bool, latched (reliable, transient local,
//!                               depth 1), and drivers/<d>/cmd_vel (TwistStamped), for each
//!                               d in lag_aware_drivers: is a lag-aware driver driving?
//!             drivers/<d>/cmd_vel for each d in pedal_drivers, and safety/estop
//!                               (std_msgs/Bool): the pedal, off during an e-stop
//!             world/road_ahead  aav_interfaces/RoadAhead (map), and
//!             localization/odom/global  nav_msgs/Odometry (map): only with road_ahead true.
//!                               An automatic driver's forward speed (road_ahead_drivers) is
//!                               never above what the road ahead allows where the car is;
//!                               no road, or a pose older than road_pose_timeout s, allows 0
//! Publishes   ego/actuator_cmd  ackermann_msgs/AckermannDriveStamped, at rate_hz
//!             ego/trace         nav_msgs/Path, base_link, at trace_rate_hz: where the car
//!                               goes over trace_horizon s
//!             ego/stop_point    geometry_msgs/PoseStamped, base_link, with each trace: where
//!                               a full brake now stops the car
//! Parameters  ego.yaml's and vehicle.yaml's, as the Python (README). Only
//!             max_steering_rate, payload and lag_lead change while running; the others are
//!             read only. vehicle_yaml: the car model's numbers ('' = aav_description's,
//!             found on AMENT_PREFIX_PATH)
//!
//! Node `look_ahead`
//! Subscribes  ego/trace, ego/stop_point, drivers/nav/local_plan (Path), plan (Path),
//!             local_costmap/costmap (nav_msgs/OccupancyGrid), world/road_ahead, /tf and
//!             /tf_static (each frame into base_link: the newest)
//! Publishes   ego/plan_ahead    nav_msgs/Path, base_link, at rate_hz: the faint line
//!             ego/ahead         diagnostic_msgs/DiagnosticStatus, at rate_hz: gap_m, plan,
//!                               stop_m, goal_m, obstacle_m, room_m, front_m, rear_m,
//!                               half_width_m, road_mps, road_m, road_why
//! Parameters  rate_hz (10), plan_ahead_m (10), local_plan_timeout (0.5 s), stale_after
//!             (1 s), gap_warn (0.3 m), gap_bad (0.6 m), look_m (15 m), lethal (100),
//!             base_frame (base_link); body ([front, rear, half width] m from base_link:
//!             vehicle.yaml's outline, which ego.launch.py passes; no default)
//!
//! Serves nothing; calls nothing. Stops cleanly on SIGINT, SIGTERM or SIGHUP.

mod ego;
mod look;
mod params;

use std::sync::Arc;

use anyhow::Result;
use rclrs::*;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let ego_node = executor.create_node("ego")?;
    let _ego = ego::start(&ego_node)?;
    // --ego-only: ego alone (ego.launch.py look_ahead:=false).
    let look_node = if std::env::args().any(|a| a == "--ego-only") {
        None
    } else {
        let node = executor.create_node("look_ahead")?;
        let running = look::start(&node)?;
        Some((node, running))
    };

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    drop(look_node);
    Ok(())
}

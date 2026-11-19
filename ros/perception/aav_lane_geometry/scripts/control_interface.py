#!/usr/bin/env python3
"""
control_interface: the 2025-26 lane follower, a driver on twist_mux's AI slot.

A PID on the lane offset, in five states. The logic is lane_follow.py, beside this
script, with no ROS; this file is the ROS around it (aav, 19 Nov). Its Rust twin,
aav_lane_follower_rs, is the default; lane_follower_impl:=python runs this one.

Subscribes  /perception/lane/geometry  aav_lane_geometry/LaneGeometry (depth 10)
Publishes   /drivers/ai/cmd_vel        geometry_msgs/TwistStamped, base_link (twist_mux's
                                       AI slot, priority 10)
            /drivers/ai/lane_status    std_msgs/String: the state and numbers, as text
            /drivers/ai/lane_state     std_msgs/String: the state, once per lane message
Parameters  base_speed_mps, max_angular_z, min_speed_mps, min_curvature_m,
            lane_change_rate_threshold, lane_change_offset_threshold, lane_change_speed,
            lane_change_timeout_s, stop_wait_s, decel_rate, accel_rate, pid_kp, pid_ki,
            pid_kd, invert_offset_sign (README.md)
No services, no calls. A 0.1 s timer on the node's clock is the watchdog.
"""
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from aav_lane_geometry.msg import LaneGeometry
# aav: the logic and the slow-down rule, in modules with no ROS (beside this script).
from lane_follow import LaneFollower, Settings
from lane_speed import AI_SLOT_TIMEOUT_S


class ControlInterface(Node):
    def __init__(self):
        super().__init__('control_interface')
        values = {}
        for name, default in vars(Settings()).items():
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            values[name] = bool(value) if isinstance(default, bool) else float(value)
        self.settings = Settings(**values)
        # aav: the watchdog's times are on the node's clock (the simulator's under
        # use_sim_time, as twist_mux's and ego's are); the PID's on the wall clock.
        self.follower = LaneFollower(self.settings, time.time(), self.now_s())

        self.sub = self.create_subscription(LaneGeometry, '/perception/lane/geometry', self.geometry_cb, 10)
        # aav: TwistStamped on /drivers/ai/cmd_vel, twist_mux's slot for a learned driver (priority 10),
        # instead of Twist on /control_cmd. Every other driver outranks it.
        self.pub_cmd = self.create_publisher(TwistStamped, '/drivers/ai/cmd_vel', 10)
        self.pub_stat = self.create_publisher(String, '/drivers/ai/lane_status', 10)
        self.pub_vstate = self.create_publisher(String, '/drivers/ai/lane_state', 10)
        self.create_timer(0.1, self.watchdog_cb)

        self.get_logger().info('Control interface ready. Subscribing to /perception/lane/geometry and publishing /drivers/ai/cmd_vel')
        self.get_logger().info(f'invert_offset_sign={self.settings.invert_offset_sign}')
        if self.settings.stop_wait_s >= AI_SLOT_TIMEOUT_S - 0.1:
            self.get_logger().warning(
                f'stop_wait_s={self.settings.stop_wait_s} is not below twist_mux\'s {AI_SLOT_TIMEOUT_S} s '
                'for the AI slot: when lane messages stop, ego stops the car first, not this '
                'node\'s slow-down')

    def now_s(self) -> float:
        """Return the node's clock in seconds (aav)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def geometry_cb(self, msg: LaneGeometry):
        self.publish(self.follower.on_geometry(msg, time.time(), self.now_s()))

    def watchdog_cb(self):
        self.publish(self.follower.tick(time.time(), self.now_s()))

    def publish(self, reply):
        """Publish and log what the follower decided, in the order it did."""
        if reply.quiet_for is not None:
            self.get_logger().warning(f'No geometry message for {reply.quiet_for:.1f}s -> slowing down', throttle_duration_sec=2.0)
        for old, new in reply.transitions:
            self.get_logger().info(f'State: {old} -> {new}')
        for linear_x, angular_z in reply.commands:
            # aav: a TwistStamped, as twist_mux expects.
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.twist.linear.x = linear_x
            out.twist.angular.z = angular_z
            self.pub_cmd.publish(out)
        if reply.status is not None:
            self.pub_stat.publish(String(data=reply.status))
        if reply.state is not None:
            self.pub_vstate.publish(String(data=reply.state))


def main(args=None):
    rclpy.init(args=args)
    node = ControlInterface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # aav: Ctrl-C arrives twice under launch, and Jazzy's handler may already have shut
        # rclpy down: ignore the second, and shut down only if it hasn't happened.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

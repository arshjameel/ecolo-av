"""
gazebo_adapter: Gazebo (Harmonic) as the car, behind the same topics as the real one.

The car itself: its actuators, wheels and odometry, as the spine is on the car. Each
sensor has a node of its own, as on the car (relay.py; one process each, so each runs on
its own core, Ben 22 Oct): gazebo_imu, gazebo_gps, gazebo_radar, gazebo_camera.

Subscribes  control/gazebo/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
                                  command, from the output mux (aav_io_mux; not with spine)
            vehicle/gazebo/spine_outputs  aav_interfaces/SpineOutputs, only with spine:
                                  spine_sim's throttle, brake, gear and steering target
            vehicle/gazebo/esp32_outputs  aav_interfaces/SpineOutputs, only with esp32: the
                                  same from gazebo_esp32, today's C++ ESP32 standing in
                                  behind the real legacy_uart_bridge
            gazebo/odom           nav_msgs/Odometry   (bridged from the Ackermann system)
            joint_states          sensor_msgs/JointState (bridged: the wheels' spin)
Publishes   gazebo/cmd_vel        geometry_msgs/Twist (bridged to /model/aav/cmd_vel), at
                                  rate_hz: the actuator model's speed and steering
            vehicle/odom          nav_msgs/Odometry, twist only, like aav_spine (not with spine)
            vehicle/status        aav_interfaces/VehicleStatus (mode COMPUTER, armed; not with
                                  spine)
            vehicle/steering      aav_interfaces/ActuatorState: ego's angle, the model's; with
                                  spine, on vehicle/gazebo/steering: the spine's target, the
                                  model's angle
            vehicle/throttle      aav_interfaces/ActuatorState: the speed loop's, the model's
            vehicle/brake         aav_interfaces/ActuatorState: the same (NaN with the model
                                  off). Neither with spine: spine_bridge has them
            vehicle/wheels        aav_interfaces/WheelSpeeds: each wheel's spin x its radius;
                                  with spine, on vehicle/gazebo/wheels, for spine_sim
            system/source         aav_interfaces/DataSource, latched: everything above, and
                                  every sensor Gazebo gives (the sensor nodes' too), is SIM
                                  (sources.py)

Sensor faults on purpose (faults.py, docs/full-simulation.md step 6): fault.odom (ok,
stop, freeze, nan), set while it runs (pixi run sim-fault odom stop), changes what
/vehicle/odom and /vehicle/wheels carry. The other sensors' faults are their nodes'.

The actuator model (actuator_model.py, docs/full-simulation.md step 3) sits between
ego and Gazebo: a stand-in for the spine's speed loop turns ego's speed into throttle and
brake, and the model turns those, and ego's steering angle, into what the car does, with
the numbers in vehicle.yaml's `actuators:` section. actuator_model:=false gives the old
behaviour instead, to compare: the speed within Gazebo's old limits, an instant steer.
Gazebo's own limits (aav.gazebo.xacro) are wide, so the model, not Gazebo, limits. No
command for cmd_timeout s: the target speed drops to 0, as the spine's watchdog.

The spine in the loop (spine:=true, docs/full-simulation.md step 4): ego's commands go
to spine_bridge instead, and over the CAN topics to spine_sim, which runs the spine's own
firmware logic (spine-core). Its outputs come here, to the actuator model's actuators
(Actuators.drive: its speed loop is skipped), and what Gazebo's car does goes back to
spine_sim on vehicle/gazebo/wheels and vehicle/gazebo/steering; spine_bridge publishes the
car's /vehicle/* from the spine's CAN frames. Exactly one backend still takes ego's
commands: spine_bridge. If spine_sim goes quiet for cmd_timeout s, the car brakes fully.

Parameters  actuator_model (true), speed_loop (default: the actuator model's speed loop
            profile, 'default', 'smooth' or 'lqi': TODO 27, 3), spine (false), rate_hz (50),
            cmd_timeout (0.2 s), wheelbase,
            wheel_radius, rendered (the world renders cameras), radar, lidar (what
            /system/source announces), fault.odom (ok), speed_variance

So ekf_local, ekf_global, navsat_transform and imu_filter_madgwick run in simulation
with the same configs as on the car, and radar_clusters, Nav2's costmap and
collision_monitor read the radar as they would on the car. radar is true only in a world
that renders the radar stand-in (gazebo.launch.py). Gazebo's ground-truth pose is not
passed on: gz_bridge carries it to /vehicle/gazebo/truth, for checks only.
"""

import math
import signal

from aav_description import vehicle
from aav_gazebo import faults, sources, wheels
from aav_gazebo.actuator_model import (ActuatorConfig, Actuators, Demand, Ideal,
                                       speed_loop_profile)
from aav_interfaces.msg import ActuatorState, DataSource, SensorSource, SpineOutputs
from aav_interfaces.msg import VehicleStatus
from aav_interfaces.msg import WheelSpeeds
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


NAN = float('nan')
# What a silent spine_sim leaves on the actuators, in Gazebo: no throttle, full brake.
SILENT_SPINE = Demand(throttle=0.0, brake=1.0, reverse=False)


def odom_nan(msg):
    t = msg.twist.twist
    t.linear.x = t.angular.z = NAN


def wheels_nan(msg):
    msg.front_left = msg.front_right = msg.rear_left = msg.rear_right = NAN


def turn_rate(speed, steering, wheelbase):
    """Bicycle model: the yaw rate that gives this steering angle at this speed."""
    return speed * math.tan(steering) / wheelbase


class GazeboAdapter(Node):

    def __init__(self):
        super().__init__('gazebo_adapter')
        self.wheelbase = self.declare_parameter('wheelbase', 1.57).value
        self.speed_var = self.declare_parameter('speed_variance', 0.01).value
        lidar = self.declare_parameter('lidar', False).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.15).value   # m
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.2).value     # s
        rate = self.declare_parameter('rate_hz', 50.0).value
        car = vehicle.load()
        config = ActuatorConfig.from_vehicle(car)
        self.spine = self.declare_parameter('spine', False).value
        self.esp32 = self.declare_parameter('esp32', False).value
        if self.spine and self.esp32:
            raise ValueError('spine and esp32 are two backends: pick one')
        # Driven by a firmware's outputs (spine_sim's or gazebo_esp32's), not ego's speed.
        self.outputs = self.spine or self.esp32
        modelled = self.declare_parameter('actuator_model', True).value
        if self.outputs and not modelled:
            raise ValueError('spine:=true and esp32:=true drive the actuator model: '
                             'actuator_model must be true')
        profile = self.declare_parameter('speed_loop', 'default').value
        if modelled:
            self.model = Actuators(config, speed_loop_profile(profile))
            self.get_logger().info(f'the actuator model, from vehicle.yaml: {config}; '
                                   f'speed loop: {profile}')
        else:
            self.model = Ideal(config)
            self.get_logger().info("no actuator model: Gazebo's old limits, an instant steer")
        self.cmd = None          # ego's last command, or spine_sim's last outputs
        self.cmd_time = None
        self.last_tick = None
        self.state = None        # the model's last step

        self.cmd_pub = self.create_publisher(Twist, 'gazebo/cmd_vel', 10)
        if self.spine:
            # The car's /vehicle/* come from spine_bridge; these only go to spine_sim.
            self.steering_pub = self.create_publisher(ActuatorState, 'vehicle/gazebo/steering',
                                                      10)
            self.wheels_pub = self.create_publisher(WheelSpeeds, 'vehicle/gazebo/wheels', 10)
            self.create_subscription(SpineOutputs, 'vehicle/gazebo/spine_outputs',
                                     self.on_cmd, 10)
            self.get_logger().info('the spine in the loop: spine_sim drives the actuator model')
        else:
            self.odom_pub = self.create_publisher(Odometry, 'vehicle/odom', 10)
            self.status_pub = self.create_publisher(VehicleStatus, 'vehicle/status', 10)
            self.steering_pub = self.create_publisher(ActuatorState, 'vehicle/steering', 10)
            self.throttle_pub = self.create_publisher(ActuatorState, 'vehicle/throttle', 10)
            self.brake_pub = self.create_publisher(ActuatorState, 'vehicle/brake', 10)
            self.wheels_pub = self.create_publisher(WheelSpeeds, 'vehicle/wheels', 10)
            if self.esp32:
                # The car's own /vehicle/* stay here: on today's car they come from the
                # wheel sensors, not the ESP32.
                self.create_subscription(SpineOutputs, 'vehicle/gazebo/esp32_outputs',
                                         self.on_cmd, 10)
                self.get_logger().info("today's ESP32 in the loop: gazebo_esp32 drives the "
                                       'actuator model')
            else:
                self.create_subscription(AckermannDriveStamped, 'control/gazebo/actuator_cmd',
                                         self.on_cmd, 10)
        self.create_subscription(Odometry, 'gazebo/odom', self.on_odom, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_timer(1.0 / rate, self.tick)
        rendered = self.declare_parameter('rendered', False).value
        self.announce(sources.simulated(rendered=rendered, lidar=lidar,
                                        radar=self.declare_parameter('radar', False).value,
                                        vehicle=not self.spine))
        # The wheels' fault on purpose: one per stream (/vehicle/odom, /vehicle/wheels),
        # set together. The other sensors' faults are their own nodes' (faults.NODES).
        self.faults = {'odom': [faults.Fault('odom'), faults.Fault('odom')]}
        for sensor in self.faults:
            self.declare_parameter(f'fault.{sensor}', 'ok')
        self.add_on_set_parameters_callback(self.on_fault)

    def on_fault(self, params):
        for p in params:
            if not p.name.startswith('fault.'):
                continue
            sensor = p.name[len('fault.'):]
            if sensor not in self.faults:
                return SetParametersResult(
                    successful=False, reason=f"{sensor}'s fault is /{faults.NODES.get(sensor)}'s")
            mode = p.value if p.type_ == Parameter.Type.STRING else repr(p.value)
            why = faults.check(sensor, mode)
            if why is not None:
                return SetParametersResult(successful=False, reason=why)
            for fault in self.faults[sensor]:
                fault.switch(mode)
            # One call per severity: rclpy refuses a call site whose severity changes.
            if mode == 'ok':
                self.get_logger().info(f'fault cleared: {sensor} ok')
            else:
                self.get_logger().warning(f'FAULT ON PURPOSE: {sensor} {mode}')
        return SetParametersResult(successful=True)

    def faulty(self, sensor, msg, nan=None, jump=None, stream=0):
        """Return msg through the sensor's fault (None: send nothing), stamped as msg."""
        out = self.faults[sensor][stream].apply(msg, nan=nan, jump=jump)
        if out is not None and out is not msg:
            out.header.stamp = msg.header.stamp     # a frozen sensor still looks alive
        return out

    def announce(self, entries):
        """Say once, latched, that all of this is simulated (/system/source)."""
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_pub = self.create_publisher(DataSource, 'system/source', latched)
        msg = DataSource(announcer=self.get_name())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sensors = [SensorSource(name=n, topic=t, source=SensorSource.SIM)
                       for n, t in entries]
        self.source_pub.publish(msg)

    def on_cmd(self, msg):
        """Ego's AckermannDriveStamped, or a firmware's SpineOutputs (spine, esp32)."""
        self.cmd = msg if self.outputs else msg.drive
        self.cmd_time = self.get_clock().now()

    def tick(self):
        """Step the actuator model on Gazebo's clock, and send Gazebo what it gives."""
        now = self.get_clock().now()
        before, self.last_tick = self.last_tick, now
        if before is None:
            return
        dt = (now - before).nanoseconds * 1e-9
        if dt <= 0.0:
            return                       # the clock stood still, or went back (a reset)
        dt = min(dt, 0.1)                # a stall: don't jump
        fresh = (self.cmd is not None
                 and (now - self.cmd_time).nanoseconds * 1e-9 <= self.cmd_timeout)
        if self.outputs:
            # The firmware decided these: straight to the actuators.
            demand = (Demand(throttle=self.cmd.throttle, brake=self.cmd.brake,
                             reverse=self.cmd.reverse) if fresh else SILENT_SPINE)
            steer = self.cmd.steering if self.cmd is not None else 0.0
            s = self.state = self.model.drive(demand, steer, dt)
        else:
            speed = self.cmd.speed if fresh else 0.0
            steer = self.cmd.steering_angle if self.cmd is not None else 0.0
            s = self.state = self.model.step(speed, steer, dt)

        out = Twist()
        out.linear.x = s.speed
        out.angular.z = turn_rate(s.speed, s.steering, self.wheelbase)
        self.cmd_pub.publish(out)

        stamp = now.to_msg()
        actuators = [(self.steering_pub, s.steer_command, s.steering)]
        if not self.spine:
            actuators += [(self.throttle_pub, s.throttle_command, s.throttle),
                          (self.brake_pub, s.brake_command, s.brake)]
        for pub, command, measured in actuators:
            a = ActuatorState(command=float(command), measured=float(measured))
            a.header.stamp = stamp
            a.header.frame_id = 'base_link'
            pub.publish(a)

    def on_joints(self, msg):
        w = WheelSpeeds(**wheels.ground_speeds(msg.name, msg.velocity, self.wheel_radius),
                        direction_measured=True)
        w.header.stamp = msg.header.stamp
        w.header.frame_id = 'base_link'
        w = self.faulty('odom', w, nan=wheels_nan, stream=1)
        if w is not None:
            self.wheels_pub.publish(w)

    def on_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        speed = msg.twist.twist.linear.x
        if self.spine:
            return               # spine_bridge has the car's odometry and status
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = msg.twist.twist.angular.z
        cov = [0.0] * 36
        cov[0] = self.speed_var
        cov[7] = 1e-4
        cov[14] = cov[21] = cov[28] = 1e3
        cov[35] = 0.02
        odom.twist.covariance = cov
        odom.pose.covariance = [1e3 if i % 7 == 0 else 0.0 for i in range(36)]
        odom = self.faulty('odom', odom, nan=odom_nan)
        if odom is not None:
            self.odom_pub.publish(odom)

        s = VehicleStatus()
        s.header.stamp = msg.header.stamp
        s.header.frame_id = 'base_link'
        s.mode = VehicleStatus.MODE_COMPUTER
        s.armed = True
        s.speed = speed
        if self.state is not None:
            m = self.state
            s.steering_angle = m.steering
            # The model's levels; with the model off there are none (NaN): say 0.
            s.throttle = m.throttle if math.isfinite(m.throttle) else 0.0
            s.brake = m.brake if math.isfinite(m.brake) else 0.0
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C reaches a launched node twice (terminal and launch), and Jazzy can then
        # raise RCLError ("context is not valid"). Only a live context means a real error.
        if rclpy.ok():
            raise
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

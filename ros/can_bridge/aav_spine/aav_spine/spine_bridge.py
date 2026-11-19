"""
spine_bridge: the ROS side of the CAN spine.

Subscribes  control/<target>/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
                              command, from the output mux (aav_io_mux) - control/car/ for
                              source real, control/bench/ for bench, control/sim/ for
                              sim. Never /ego/actuator_cmd itself: on the car, only an
                              output mux that found every source REAL reaches this node
            safety/estop      std_msgs/Bool
            safety/output_mux/state  std_msgs/String: the safety gate, for spine/arm
            safety/self_test/status  diagnostic_msgs/DiagnosticStatus: the boot
                              self-test, for spine/arm (only with require_ready, TODO 255)
            can/rx            can_msgs/Frame  (ros2_socketcan receiver)
Publishes   can/tx            can_msgs/Frame  (ros2_socketcan sender)
            vehicle/odom      nav_msgs/Odometry  (twist only, in base_link)
            vehicle/wheels    aav_interfaces/WheelSpeeds  (the front wheel nodes, ~100 Hz;
                              NaN for a stale wheel and for the rear two)
            vehicle/status    aav_interfaces/VehicleStatus
            vehicle/joint_states  sensor_msgs/JointState  (front steering joints, for RViz;
                                  joint_state_publisher merges it into /joint_states)
            vehicle/steering  aav_interfaces/ActuatorState  (command: the last sent; measured:
                              VEHICLE_FEEDBACK)
            vehicle/throttle, vehicle/brake  aav_interfaces/ActuatorState  (measured, from
                              VEHICLE_STATUS; command NaN: the spine's speed loop decides)
            system/source     aav_interfaces/DataSource, latched: REAL on the car, BENCH on
                              the bench, SIM in the simulator's spine profile (the `source`
                              parameter)
Service     spine/arm         std_srvs/SetBool: refused during an e-stop, and while the
                              safety gate is shut: the output mux not OPEN for this
                              bridge's target, or quiet for gate_timeout s (Ben, 24 Oct;
                              output_gate); with require_ready, also until the
                              boot self-test says READY. Disarming always works

Frames are defined in firmware/can/aav.dbc and packed by can_codec.
/vehicle/odom's speed is the front wheel nodes' mean (WHEEL_SPEED_L/R, turned into
the rear axle's speed by wheel_speed), while both are fresh; otherwise the spine's
own measured_speed from VEHICLE_FEEDBACK.
DRIVE_CMD goes out at 50 Hz only while ego's command is fresh. If ego (or the
output mux) stops, the bridge stops sending and the spine's own watchdog brakes the car.

Arming always sends a few armed=0 frames before armed=1: the spine only arms on
that edge. It also clears latches the bridge can't see (a hardware e-stop, a
command timeout, a driver fault, a spine reboot).

role:=spine (the default) is all of the above. role:=sensors is for a car driven another
way (today's: the ESP32, through the Motor Pi's UART; docs/can-plan.md). The node only
reads the wheel nodes' frames: no can/tx, no safety/estop or safety/output_mux/state, no
spine/arm, no timers, and
none of vehicle/status, joint_states, steering, throttle, brake. vehicle/odom comes from
the wheels alone (their mean while both are fresh; no yaw rate: the steering angle isn't
on the bus), and system/source names only odometry and wheels. It still reads
control/<target>/actuator_cmd, only to sign a wheel that sends a magnitude.
"""

import math
import signal
import time

from aav_description import vehicle
from aav_interfaces.msg import ActuatorState, DataSource, SensorSource, VehicleStatus
from aav_interfaces.msg import WheelSpeeds
from aav_spine import bridges
from aav_spine import can_codec as cc
from aav_spine import output_gate
from aav_spine import wheel_speed as ws
from ackermann_msgs.msg import AckermannDriveStamped
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

# /system/source: what the `source` parameter may say.
SOURCES = {'real': SensorSource.REAL, 'bench': SensorSource.BENCH, 'sim': SensorSource.SIM}
# Where ego's commands come from, for each source: the output mux's topic for the target
# this bridge plays (aav_io_mux provenance.TARGETS). The real car's only from target car.
COMMANDS = {'real': 'control/car/actuator_cmd', 'bench': 'control/bench/actuator_cmd',
            'sim': 'control/sim/actuator_cmd'}
# What this node publishes, for /system/source.
ANNOUNCED = (('vehicle', '/vehicle/status'), ('odometry', '/vehicle/odom'),
             ('steering', '/vehicle/steering'), ('throttle', '/vehicle/throttle'),
             ('brake', '/vehicle/brake'), ('wheels', '/vehicle/wheels'))
# In role sensors: the wheels only, never the actuators.
ANNOUNCED_SENSORS = (('odometry', '/vehicle/odom'), ('wheels', '/vehicle/wheels'))
NAN = float('nan')
UNKNOWN = 1e3   # the covariance of what isn't measured


class SpineBridge(Node):

    FROM_BUS = (cc.VEHICLE_FEEDBACK_ID, cc.VEHICLE_STATUS_ID) + cc.WHEEL_SPEED_IDS

    def __init__(self):
        super().__init__('spine_bridge')
        self.wheelbase = self.declare_parameter('wheelbase', 1.57).value
        self.track = self.declare_parameter('track', 0.77).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.1).value
        wheel_timeout = self.declare_parameter('wheel_timeout', 0.1).value
        gate_timeout = self.declare_parameter('gate_timeout', output_gate.TIMEOUT).value
        # The boot self-test (TODO 255): on the car, no arming until it says READY.
        require_ready = self.declare_parameter('require_ready', False).value
        ready_timeout = self.declare_parameter('ready_timeout', output_gate.READY_TIMEOUT).value
        drive_hz = self.declare_parameter('drive_rate_hz', 50.0).value
        estop_hz = self.declare_parameter('estop_rate_hz', 10.0).value
        self.speed_var = self.declare_parameter('speed_variance', 0.01).value
        self.yaw_rate_var = self.declare_parameter('yaw_rate_variance', 0.02).value
        self.joints = self.declare_parameter(
            'steering_joints', ['front_left_steering_joint', 'front_right_steering_joint']).value
        source = self.declare_parameter('source', 'real').value
        if source not in SOURCES:
            raise ValueError(f'source:={source}: use one of {sorted(SOURCES)}')
        self.role = self.declare_parameter('role', bridges.ROLES[0]).value
        bridges.check_role(self.role)
        self.sends = self.role == 'spine'

        self.cmd = None
        self.cmd_time = None
        self.armed = False
        self.disarm_frames = 0   # armed=0 frames still to send before arming
        self.estop = False
        self.gate = output_gate.OutputGate(output_gate.TARGETS[source], gate_timeout)
        self.ready = output_gate.ReadyGate(ready_timeout) if require_ready else None
        self.counters = {}
        self.feedback = None
        self.bad_frames = 0
        self.wheels = ws.FrontWheels(wheel_timeout)

        self.odom_pub = self.create_publisher(Odometry, 'vehicle/odom', 10)
        self.wheel_pub = self.create_publisher(WheelSpeeds, 'vehicle/wheels', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_pub = self.create_publisher(DataSource, 'system/source', latched)
        # In role sensors, only to sign a wheel that sends a magnitude.
        self.create_subscription(AckermannDriveStamped, COMMANDS[source], self.on_cmd, 10)
        self.create_subscription(Frame, 'can/rx', self.on_frame, 100)
        if self.sends:
            self.to_can = self.create_publisher(Frame, 'can/tx', 50)
            self.status_pub = self.create_publisher(VehicleStatus, 'vehicle/status', 10)
            self.joint_pub = self.create_publisher(JointState, 'vehicle/joint_states', 10)
            self.steering_pub = self.create_publisher(ActuatorState, 'vehicle/steering', 10)
            self.throttle_pub = self.create_publisher(ActuatorState, 'vehicle/throttle', 10)
            self.brake_pub = self.create_publisher(ActuatorState, 'vehicle/brake', 10)
            self.create_subscription(Bool, 'safety/estop', self.on_estop, 10)
            self.create_subscription(String, 'safety/output_mux/state', self.on_gate, 10)
            self.create_subscription(DiagnosticStatus, 'safety/self_test/status',
                                     self.on_self_test, 10)
            self.create_service(SetBool, 'spine/arm', self.on_arm)
            self.create_timer(1.0 / drive_hz, self.send_drive)
            self.create_timer(1.0 / estop_hz, self.send_estop)
        else:
            # Only the wheel nodes count: a spine's frames are not this role's.
            self.FROM_BUS = cc.WHEEL_SPEED_IDS
            self.get_logger().info(
                'role sensors: reading the wheel nodes only; nothing is sent on CAN')
        self.announce(SOURCES[source], ANNOUNCED if self.sends else ANNOUNCED_SENSORS)

    def announce(self, kind, announced):
        """Say once, latched, where this node's data comes from (/system/source)."""
        msg = DataSource(announcer=self.get_name())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sensors = [SensorSource(name=n, topic=t, source=kind) for n, t in announced]
        self.source_pub.publish(msg)

    def actuator(self, stamp, command, measured):
        a = ActuatorState(command=float(command), measured=float(measured))
        a.header.stamp = stamp
        a.header.frame_id = 'base_link'
        return a

    # Orin -> spine

    def on_cmd(self, msg):
        self.cmd = msg.drive
        self.cmd_time = self.get_clock().now()

    def on_estop(self, msg):
        changed = msg.data != self.estop
        self.estop = msg.data
        if self.estop:
            self.armed = False
        if changed:
            self.send_estop()

    def on_gate(self, msg):
        # A steady clock: in the simulator the node's is simulated, and may pause.
        self.gate.heard(msg.data, time.monotonic())

    def on_self_test(self, msg):
        # The same steady clock as the gate's. Counts only with require_ready.
        if self.ready is not None:
            self.ready.heard(msg.level == DiagnosticStatus.OK, msg.message, time.monotonic())

    def on_arm(self, request, response):
        refusal = None
        if request.data and self.estop:
            refusal = 'e-stop is active; clear /safety/estop first'
        elif request.data:
            refusal = self.gate.refusal(time.monotonic())
            if refusal is None and self.ready is not None:
                refusal = self.ready.refusal(time.monotonic())
        if refusal:
            response.success = False
            response.message = refusal
            self.get_logger().warning(f'arm refused: {refusal}')
            return response
        if request.data:
            self.disarm_frames = 3   # the spine arms on armed 0 -> 1, every time
        self.armed = request.data
        response.success = True
        response.message = 'armed' if self.armed else 'disarmed'
        self.get_logger().info(response.message)
        return response

    def _next(self, can_id):
        self.counters[can_id] = (self.counters.get(can_id, 0) + 1) & 0x0F
        return self.counters[can_id]

    def _send(self, can_id, data):
        f = Frame()
        f.header.stamp = self.get_clock().now().to_msg()
        f.id = can_id
        f.dlc = len(data)
        f.data = list(data) + [0] * (8 - len(data))
        self.to_can.publish(f)

    def send_drive(self):
        if self.cmd is None:
            return
        age = (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.get_logger().warning('no fresh command from ego and the output mux; the '
                                      'spine will brake',
                                      throttle_duration_sec=2.0)
            return
        armed = self.armed and self.disarm_frames == 0
        if self.disarm_frames:
            self.disarm_frames -= 1
        self._send(cc.DRIVE_CMD_ID, cc.pack(cc.DRIVE_CMD_ID, {
            'target_speed': self.cmd.speed,
            'steering_angle': self.cmd.steering_angle,
            'accel_limit': abs(self.cmd.acceleration),
            'steer_rate_limit': abs(self.cmd.steering_angle_velocity),
            'armed': armed,
            'counter': self._next(cc.DRIVE_CMD_ID),
        }))

    def send_estop(self):
        self._send(cc.ESTOP_CMD_ID, cc.pack(cc.ESTOP_CMD_ID, {
            'estop_request': self.estop,
            'counter': self._next(cc.ESTOP_CMD_ID),
        }))

    # spine -> Orin

    def on_frame(self, f):
        if f.is_error or f.is_rtr or f.id not in self.FROM_BUS:
            return
        try:
            values = cc.unpack(f.id, bytes(f.data[:f.dlc]))
        except cc.FrameError as e:
            self.bad_frames += 1
            self.get_logger().warning(f'{e} ({self.bad_frames} so far)', throttle_duration_sec=2.0)
            return
        if f.id == cc.VEHICLE_FEEDBACK_ID:
            self.publish_feedback(values)
        elif f.id == cc.VEHICLE_STATUS_ID:
            self.publish_status(values)
        elif self.wheels.update(f.id, values, self.now_s()):
            self.publish_wheels()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive_sign(self):
        return ws.drive_sign(self.feedback and self.feedback['measured_speed'],
                             self.cmd.speed if self.cmd else None)

    def publish_wheels(self):
        r = self.wheels.reading(self.now_s(), self.drive_sign())
        msg = WheelSpeeds(front_left=r.left if r.left_fresh else NAN,
                          front_right=r.right if r.right_fresh else NAN,
                          rear_left=NAN, rear_right=NAN,
                          direction_measured=r.direction_measured)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.wheel_pub.publish(msg)
        # Role sensors: no spine sends VEHICLE_FEEDBACK, so the wheels alone give
        # /vehicle/odom, while both are fresh (else nothing: the watchdog sees it stop).
        # No steering angle on this bus: the front mean as it is, and no yaw rate (the EKF
        # takes it from the IMU).
        front = r.mean()
        if not self.sends and front is not None:
            self.publish_odom(msg.header.stamp, front, None)

    def publish_odom(self, stamp, speed, yaw_rate):
        """Publish /vehicle/odom: base_link's speed, and its yaw rate if known (else None)."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = 0.0 if yaw_rate is None else yaw_rate
        cov = [0.0] * 36
        cov[0] = self.speed_var          # vx
        cov[7] = 1e-4                    # vy: a car doesn't slide sideways
        cov[14] = cov[21] = cov[28] = UNKNOWN  # vz, roll and pitch rates: not measured
        cov[35] = UNKNOWN if yaw_rate is None else self.yaw_rate_var
        # Wheels that read nothing: the car stands, exactly (TODO 191).
        odom.twist.covariance = vehicle.standstill_covariance(speed, cov)
        odom.pose.covariance = [UNKNOWN if i % 7 == 0 else 0.0 for i in range(36)]
        self.odom_pub.publish(odom)

    def publish_feedback(self, v):
        self.feedback = v
        now = self.get_clock().now().to_msg()
        steer = v['measured_steering_angle']
        front = self.wheels.reading(self.now_s(), self.drive_sign()).mean()
        speed = v['measured_speed'] if front is None else ws.rear_axle_speed(front, steer)
        self.publish_odom(now, speed, speed * math.tan(steer) / self.wheelbase)

        js = JointState()
        js.header.stamp = now
        js.name = list(self.joints)
        js.position = list(self.wheel_angles(steer))
        self.joint_pub.publish(js)

        command = self.cmd.steering_angle if self.cmd is not None else NAN
        self.steering_pub.publish(self.actuator(now, command, steer))

    def wheel_angles(self, steer):
        """Return the left and right wheel angles for a virtual centre angle (Ackermann)."""
        if abs(steer) < 1e-6:
            return 0.0, 0.0
        radius = self.wheelbase / math.tan(steer)   # signed, positive turning left
        left = math.atan(self.wheelbase / (radius - self.track / 2))
        right = math.atan(self.wheelbase / (radius + self.track / 2))
        return left, right

    def publish_status(self, v):
        s = VehicleStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'base_link'
        s.mode = int(v['mode'])
        s.armed = bool(v['armed'])
        s.estop_active = bool(v['estop_active'])
        s.throttle = v['throttle'] / 100.0
        s.brake = v['brake'] / 100.0
        s.faults = int(v['faults'])
        if self.feedback:
            s.speed = self.feedback['measured_speed']
            s.steering_angle = self.feedback['measured_steering_angle']
        self.status_pub.publish(s)
        self.throttle_pub.publish(self.actuator(s.header.stamp, NAN, s.throttle))
        self.brake_pub.publish(self.actuator(s.header.stamp, NAN, s.brake))


def main(args=None):
    rclpy.init(args=args)
    node = SpineBridge()
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

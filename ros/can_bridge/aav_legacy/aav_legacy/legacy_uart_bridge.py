"""
legacy_uart_bridge: drive the ESP32 from ego's commands, and relay the Orin's heartbeat.

Interim backend until the CAN spine exists. Runs on the computer wired to
the ESP32's UART (today the Motor Pi, /dev/ttyAMA0).

Subscribes  control/car/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's command,
                              from the output mux (aav_io_mux, target car), which sends it
                              only while every source is REAL. Never /ego/actuator_cmd
            control/car/heartbeat     aav_interfaces/Heartbeat: the output mux's counter, one
                              per command passed on (TODO 50)
            safety/estop      std_msgs/Bool (true: full brake now, until false)
            vehicle/odom      nav_msgs/Odometry, only with speed_loop.mode pi: the wheel speed
                              the loop closes on (twist.linear.x). Older than
                              speed_loop.odom_timeout: feed-forward alone
Publishes   system/source     aav_interfaces/DataSource, latched: the actuators are REAL
            safety/legacy_uart_bridge/lock  std_msgs/Bool, latched and again at 5 Hz:
                              true with frame_version 1 (no firmware timeout), so
                              twist_mux's no_timeout lock (priority 60) holds every program
                              and only people and the actuator tests drive; false with 2
            and, only with uplink true (the ESP32's uplink frames on its TX1; TODO 50, 48):
            vehicle/status    aav_interfaces/VehicleStatus, 10 Hz: mode, armed, e-stop, faults
            vehicle/steering  aav_interfaces/ActuatorState: ego's angle, the Jrk's position
            vehicle/brake     aav_interfaces/ActuatorState: the ESP32's brake, the Jrk's
            vehicle/rc        aav_interfaces/RcSticks: the RC's mode switch and sticks
                              (rc_driver turns them into /drivers/rc/cmd_vel)
Reads       uplink frames on the same serial port (uplink_reader, legacy_protocol), with
            uplink true. Off by default: GPIO17 to the Pi's RX is the hardware lead's to
            confirm, and the firmware needs --features uplink
Writes      frames at rate_hz on the serial port (uart_relay, legacy_protocol):
            frame_version 2   17 bytes, with the Orin's counter, armed and e-stop flags and
                              a CRC-8: for the Rust ESP32 firmware, which brakes when the
                              counter hasn't changed for 100 ms
            frame_version 1   the 2025-26 13-byte frame, for the C++ firmware on the car
                              today: no counter, and that firmware has no timeout

port is a device (/dev/ttyAMA0) or any pyserial URL: in Gazebo, socket://127.0.0.1:<n> to
gazebo_esp32, the C++ firmware's stand-in (sim.launch.py esp32:=true). If it won't open,
the bridge says so and tries again every second, rather than dying: no frames go out
until it opens, and the ESP32 keeps (or, from boot, holds) its brake. source (real, the
default; sim in Gazebo) is what it announces on system/source.

speed_loop.mode picks how ego's speed becomes throttle and brake: open (the default, as the
2025-26 team drove: speed / full_throttle_speed, the brake only standing still) or pi
(speed_control: feed-forward from ego's speed profile plus PI on the wheel speed, out as
throttle or brake; docs/designs/2026-10-27_control-plan.md, step 1). Its gains are
speed_loop.kp and the rest of SpeedControlConfig; the car model's numbers come from
vehicle.yaml's actuators: through legacy.launch.py.

The bridge only relays the counter, never makes one: when ego or the output mux go quiet
it keeps sending its stale-stop frames (brake, no throttle), with the last counter
unchanged. If the command goes stale (cmd_timeout) or /safety/estop is true, it sends
brake frames. With frame_version 1 it can't protect against its own crash, because the
C++ firmware has no timeout: so it locks the programs out (uart_relay.programs_locked).
allow_programs_without_timeout (default false) lifts that lock, for a supervised stand
only (wheels off the ground, a person at the e-stop).
"""

import signal

from aav_interfaces.msg import (ActuatorState, DataSource, Heartbeat, RcSticks, SensorSource,
                                VehicleStatus)
from aav_legacy import uplink_reader as ur
from aav_legacy.legacy_protocol import LegacyMapping
from aav_legacy.speed_control import SpeedControl, SpeedControlConfig
from aav_legacy.uart_relay import programs_locked, Relay
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import serial
from std_msgs.msg import Bool


class LegacyUartBridge(Node):

    def __init__(self):
        super().__init__('legacy_uart_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyAMA0').value
        self.baud = self.declare_parameter('baud', 115200).value
        source = self.declare_parameter('source', 'real').value
        if source not in ('real', 'sim'):
            raise ValueError(f"source {source!r}: use real, or sim (Gazebo's stand-in)")
        kind = SensorSource.REAL if source == 'real' else SensorSource.SIM
        rate = self.declare_parameter('rate_hz', 50.0).value
        timeout = self.declare_parameter('cmd_timeout', 0.3).value
        version = self.declare_parameter('frame_version', 2).value
        self.uplink = self.declare_parameter('uplink', False).value
        allow = self.declare_parameter('allow_programs_without_timeout', False).value
        uplink_timeout = self.declare_parameter('uplink_timeout', 0.2).value
        defaults = LegacyMapping()
        mapping = LegacyMapping(**{
            k: self.declare_parameter(k, v).value for k, v in vars(defaults).items()})
        mode = self.declare_parameter('speed_loop.mode', 'open').value
        if mode not in ('open', 'pi'):
            raise ValueError(f'speed_loop.mode {mode!r}: use open (the 2025-26 mapping) or pi')
        odom_timeout = self.declare_parameter('speed_loop.odom_timeout', 0.3).value
        loop = None
        if mode == 'pi':
            loop = SpeedControl(SpeedControlConfig(**{
                k: float(self.declare_parameter(f'speed_loop.{k}', float(v)).value)
                for k, v in vars(SpeedControlConfig()).items()}))
        self.relay = Relay(mapping, timeout, version, loop, odom_timeout)
        if loop:
            self.create_subscription(Odometry, 'vehicle/odom', self.on_odom, 10)
            self.get_logger().info(f'speed_loop.mode pi: closed on /vehicle/odom, {loop.cfg}')
        self.serial = None
        self.last_open = None
        self.open_port()
        self.create_subscription(AckermannDriveStamped, 'control/car/actuator_cmd',
                                 self.on_cmd, 10)
        self.create_subscription(Heartbeat, 'control/car/heartbeat', self.on_heartbeat, 10)
        self.create_subscription(Bool, 'safety/estop', self.on_estop, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.reader = ur.UplinkReader(uplink_timeout)
        self.last_status = None
        self.uplink_was_fresh = False
        if self.uplink:
            self.status_pub = self.create_publisher(VehicleStatus, 'vehicle/status', 10)
            self.steering_pub = self.create_publisher(ActuatorState, 'vehicle/steering', 10)
            self.brake_pub = self.create_publisher(ActuatorState, 'vehicle/brake', 10)
            self.rc_pub = self.create_publisher(RcSticks, 'vehicle/rc', 10)
            self.create_timer(0.01, self.read_uplink)
        # Say once, latched, that this drives the real car (/system/source).
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_pub = self.create_publisher(DataSource, 'system/source', latched)
        announcement = DataSource(announcer=self.get_name())
        announcement.header.stamp = self.get_clock().now().to_msg()
        announcement.sensors = [SensorSource(name='actuators', topic='',
                                             source=kind)]
        if self.uplink:
            announcement.sensors += [
                SensorSource(name=name, topic=topic, source=kind)
                for name, topic in (('vehicle', '/vehicle/status'),
                                    ('steering', '/vehicle/steering'),
                                    ('brake', '/vehicle/brake'), ('rc', '/vehicle/rc'))]
        self.source_pub.publish(announcement)
        # The no-timeout lock: latched for late joiners that ask for it, and sent again at
        # 5 Hz, as twist_mux subscribes volatile and would miss a message sent before it
        # started (on the car it runs on the Orin, this on the Motor Pi).
        self.lock = Bool(data=programs_locked(version, allow))
        self.lock_pub = self.create_publisher(Bool, 'safety/legacy_uart_bridge/lock', latched)
        self.lock_pub.publish(self.lock)
        self.create_timer(0.2, lambda: self.lock_pub.publish(self.lock))
        if version == 1:
            self.get_logger().warning(
                'frame_version 1: the C++ firmware has no command timeout and no heartbeat; '
                'keep the e-stop in reach')
            if self.lock.data:
                self.get_logger().warning(
                    'no-timeout lock ON (/safety/legacy_uart_bridge/lock): twist_mux holds '
                    'every program (scripted, replay, point, Nav2, AI); people and the '
                    'actuator tests still drive')
            else:
                self.get_logger().error(
                    'allow_programs_without_timeout: programs may drive through firmware '
                    'with NO command timeout. Supervised stand only: wheels off the ground, '
                    'a person at the e-stop')
        else:
            self.get_logger().info(
                'frame_version 2: relaying the counter of the Orin from control/car/heartbeat; '
                'the ESP32 brakes 100 ms after it stops changing')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.relay.command(msg.drive.speed, msg.drive.steering_angle, self.now())

    def on_odom(self, msg):
        self.relay.wheel_speed(msg.twist.twist.linear.x, self.now())

    def on_heartbeat(self, msg):
        self.relay.heartbeat(msg.counter)

    def on_estop(self, msg):
        if self.relay.set_estop(msg.data):
            self.get_logger().warning('e-stop ON: braking' if msg.data else 'e-stop released')

    def open_port(self):
        """Open the port (a device or a pyserial URL); on failure, say so and try later."""
        self.last_open = self.now()
        try:
            self.serial = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0)
            self.get_logger().info(f'serial port {self.port} open')
        except (serial.SerialException, OSError, ValueError) as e:
            self.serial = None
            self.get_logger().error(f'serial port {self.port}: {e}; trying again every second',
                                    throttle_duration_sec=10.0)

    def tick(self):
        if self.serial is None:
            if self.now() - self.last_open >= 1.0:
                self.open_port()
            return
        try:
            self.serial.write(self.relay.frame(self.now()))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'serial write failed: {e}; reopening',
                                    throttle_duration_sec=2.0)
            self.close_port()

    def read_uplink(self):
        """Read the ESP32's uplink frames (100 Hz), and publish each one."""
        now = self.now()
        if self.serial is None:
            return
        try:
            data = self.serial.read(self.serial.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'serial read failed: {e}', throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        for u in self.reader.feed(data, now):
            self.publish_uplink(u, stamp, now)
        fresh = self.reader.fresh(now)
        if fresh != self.uplink_was_fresh:
            if fresh:
                self.get_logger().info('uplink: frames from the ESP32')
            else:
                self.get_logger().warning(
                    'uplink: no frame from the ESP32 for '
                    f'{self.reader.timeout:.1f} s (TX1 wired? firmware built with uplink?)')
            self.uplink_was_fresh = fresh

    def publish_uplink(self, u, stamp, now):
        rc = RcSticks(**ur.rc_fields(u))
        rc.header.stamp = stamp
        self.rc_pub.publish(rc)
        command = self.relay.cmd[1] if self.relay.fresh(now) else None
        for pub, (cmd, measured) in ((self.steering_pub, ur.steering_state(u, command)),
                                     (self.brake_pub, ur.brake_state(u))):
            msg = ActuatorState(command=float(cmd), measured=float(measured))
            msg.header.stamp = stamp
            pub.publish(msg)
        if self.last_status is None or now - self.last_status >= 0.1:
            self.last_status = now
            status = VehicleStatus(**ur.status_fields(u))
            status.header.stamp = stamp
            status.header.frame_id = 'base_link'
            self.status_pub.publish(status)

    def close_port(self):
        try:
            self.serial.close()
        except (serial.SerialException, OSError):
            pass
        self.serial = None

    def destroy_node(self):
        if self.serial is not None:
            try:
                for _ in range(5):   # leave the car braked
                    self.serial.write(self.relay.stop_frame())
            except (serial.SerialException, OSError):
                pass
            self.close_port()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegacyUartBridge()
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

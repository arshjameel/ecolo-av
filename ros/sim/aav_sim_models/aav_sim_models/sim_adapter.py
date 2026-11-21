"""
sim_adapter: Ravensim as the car, behind the same topics as the real one.

The car itself: its actuators, wheels and odometry, as the spine is on the car. Each
sensor has a node of its own, as on the car (relay.py; one process each, so each runs on
its own core, Ben 22 Oct): sim_imu, sim_gps, sim_radar, sim_camera_relay.

Subscribes  control/sim/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
                                  command, from the output mux (aav_io_mux; not with spine)
            vehicle/sim/spine_outputs  aav_interfaces/SpineOutputs, only with spine:
                                  spine_sim's throttle, brake, gear and steering target
            vehicle/sim/esp32_outputs  aav_interfaces/SpineOutputs, only with esp32: the
                                  same from sim_esp32, today's C++ ESP32 standing in
                                  behind the real legacy_uart_bridge
            vehicle/sim/odom           nav_msgs/Odometry   (bridged from the Ackermann system)
            joint_states          sensor_msgs/JointState (bridged: the wheels' spin)
Publishes   vehicle/sim/cmd_vel        geometry_msgs/Twist (bridged to /model/aav/cmd_vel), at
                                  rate_hz: the actuator model's speed and steering
            vehicle/odom          nav_msgs/Odometry, twist only, like aav_spine (not with spine)
            vehicle/status        aav_interfaces/VehicleStatus (mode COMPUTER, armed; not with
                                  spine)
            vehicle/steering      aav_interfaces/ActuatorState: ego's angle, the model's; with
                                  spine, on vehicle/sim/steering: the spine's target, the
                                  model's angle
            vehicle/throttle      aav_interfaces/ActuatorState: the speed loop's, the model's
            vehicle/brake         aav_interfaces/ActuatorState: the same (NaN with the model
                                  off). Neither with spine: spine_bridge has them
            vehicle/wheels        aav_interfaces/WheelSpeeds: each wheel's spin x its radius;
                                  with spine, on vehicle/sim/wheels, for spine_sim
            system/source         aav_interfaces/DataSource, latched: everything above, and
                                  every sensor Ravensim gives (the sensor nodes' too), is SIM
                                  (sources.py)

Sensor faults on purpose (faults.py, docs/full-simulation.md step 6): fault.odom (ok,
stop, freeze, nan), set while it runs (pixi run sim-fault odom stop), changes what
/vehicle/odom and /vehicle/wheels carry. The other sensors' faults are their nodes'.
Actuator faults on purpose (TODO 159, the actuator model only): fault.steer (ok, stuck),
fault.throttle (ok, stuck), fault.brake (ok, weak, dead), between the speed loop (or the
firmware's outputs) and the car; /vehicle/steering, throttle and brake still say what was
asked as their command, and what the car did as measured.

The actuator model (actuator_model.py, docs/full-simulation.md step 3) sits between
ego and Ravensim: a stand-in for the spine's speed loop turns ego's speed into throttle and
brake, and the model turns those, and ego's steering angle, into what the car does, with
the numbers in vehicle.yaml's `actuators:` section. actuator_model:=false gives the old
behaviour instead, to compare: the speed within Ravensim's old limits, an instant steer.
The simulator follows the model's speed and turn rate, so the model limits. No
command for cmd_timeout s: the target speed drops to 0, as the spine's watchdog.

The spine in the loop (spine:=true, docs/full-simulation.md step 4): ego's commands go
to spine_bridge instead, and over the CAN topics to spine_sim, which runs the spine's own
firmware logic (spine-core). Its outputs come here, to the actuator model's actuators
(Actuators.drive: its speed loop is skipped), and what Ravensim's car does goes back to
spine_sim on vehicle/sim/wheels and vehicle/sim/steering; spine_bridge publishes the
car's /vehicle/* from the spine's CAN frames. Exactly one backend still takes ego's
commands: spine_bridge. If spine_sim goes quiet for cmd_timeout s, the car brakes fully.

A heavier car (docs/stopping.md, "A heavier car"): payload (kg aboard, live) makes the
modelled car push and brake less (car_model's share), whoever drives it. adaptive_brake
(live; not with spine or esp32, whose firmware decides the brake) puts the adaptive brake
(aav_description adaptive_brake.py, vehicle.yaml's `brake:` limits) between the speed
loop and the brake, as it would sit on the car: below ego and every safety lock.

Parameters  actuator_model (true), speed_loop (default: the actuator model's speed loop
            profile, 'default', 'smooth', 'lqi' or 'firm': TODO 27, 3, 163), payload (0 kg, live),
            adaptive_brake (false, live), actuators ('': the car model's own numbers
            over vehicle.yaml's, "throttle_lag=0.3,drag=0.2": a known car to measure,
            TODO 248; only the car, the drivers keep vehicle.yaml), lag_scale (1.0: the
            throttle's, brake's and steering's lags times this, to test a follower against
            a car slower than vehicle.yaml says; only the model's), spine (false), rate_hz (50),
            cmd_timeout (0.2 s), wheelbase,
            wheel_radius, rendered (the world renders sensors), cameras (it draws the
            cameras: rendered by aav_sim_cameras), radar, lidar (what
            /system/source announces), fault.odom, fault.steer, fault.throttle,
            fault.brake (ok), speed_variance

So ekf_local, ekf_global, navsat_transform and imu_filter_madgwick run in simulation
with the same configs as on the car, and radar_clusters, the costmap and
the stop zone read the radar as they would on the car. radar is true only in a world
that renders the radar stand-in (ravensim.launch.py). Ravensim's ground-truth pose is not
passed on: gz_bridge carries it to /vehicle/sim/truth, for checks only.
"""

from dataclasses import replace
import math
import signal

from aav_description import vehicle
from aav_description.adaptive_brake import AdaptiveBrake, BrakeLimits
from aav_interfaces.msg import ActuatorState, DataSource, SensorSource, SpineOutputs
from aav_interfaces.msg import VehicleStatus
from aav_interfaces.msg import WheelSpeeds
from aav_sim_models import faults, sources, wheels
from aav_sim_models.actuator_model import (ActuatorConfig, Actuators, Demand, Ideal,
                                           speed_loop_profile)
from aav_sim_models.actuator_overrides import overridden
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


NAN = float('nan')
# What a silent spine_sim leaves on the actuators, in Ravensim: no throttle, full brake.
SILENT_SPINE = Demand(throttle=0.0, brake=1.0, reverse=False)


def odom_nan(msg):
    t = msg.twist.twist
    t.linear.x = t.angular.z = NAN


def wheels_nan(msg):
    msg.front_left = msg.front_right = msg.rear_left = msg.rear_right = NAN


def turn_rate(speed, steering, wheelbase):
    """Bicycle model: the yaw rate that gives this steering angle at this speed."""
    return speed * math.tan(steering) / wheelbase


class SimAdapter(Node):

    def __init__(self):
        super().__init__('sim_adapter')
        self.wheelbase = self.declare_parameter('wheelbase', 1.57).value
        self.speed_var = self.declare_parameter('speed_variance', 0.01).value
        lidar = self.declare_parameter('lidar', False).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.15).value   # m
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.2).value     # s
        rate = self.declare_parameter('rate_hz', 50.0).value
        car = vehicle.load()
        config = ActuatorConfig.from_vehicle(car)
        own = str(self.declare_parameter('actuators', '').value)
        if own:
            config = overridden(config, own)
            self.get_logger().warning(f"the car's own actuator numbers (actuators): {own}; "
                                      'the drivers still read vehicle.yaml')
        lag_scale = float(self.declare_parameter('lag_scale', 1.0).value)
        if not 0.0 < lag_scale <= 10.0:
            raise ValueError(f'lag_scale must be in (0, 10], not {lag_scale}')
        if lag_scale != 1.0:
            config = replace(config, throttle_lag=lag_scale * config.throttle_lag,
                             brake_lag=lag_scale * config.brake_lag,
                             steer_lag=lag_scale * config.steer_lag)
            self.get_logger().warning(f"the actuators' lags x{lag_scale:g} (lag_scale): "
                                      'a car slower than vehicle.yaml says')
        self.spine = self.declare_parameter('spine', False).value
        self.esp32 = self.declare_parameter('esp32', False).value
        if self.spine and self.esp32:
            raise ValueError('spine and esp32 are two backends: pick one')
        # Driven by a firmware's outputs (spine_sim's or sim_esp32's), not ego's speed.
        self.outputs = self.spine or self.esp32
        modelled = self.declare_parameter('actuator_model', True).value
        if self.outputs and not modelled:
            raise ValueError('spine:=true and esp32:=true drive the actuator model: '
                             'actuator_model must be true')
        profile = self.declare_parameter('speed_loop', 'default').value
        self.config = config
        self.brake_limits = BrakeLimits.from_vehicle(car)
        if modelled:
            self.model = Actuators(config, speed_loop_profile(profile))
            self.get_logger().info(f'the actuator model, from vehicle.yaml: {config}; '
                                   f'speed loop: {profile}')
        else:
            self.model = Ideal(config)
            self.get_logger().info("no actuator model: Ravensim's old limits, an instant steer")
        self.cmd = None          # ego's last command, or spine_sim's last outputs
        self.cmd_time = None
        self.last_tick = None
        self.state = None        # the model's last step

        self.cmd_pub = self.create_publisher(Twist, 'vehicle/sim/cmd_vel', 10)
        if self.spine:
            # The car's /vehicle/* come from spine_bridge; these only go to spine_sim.
            self.steering_pub = self.create_publisher(ActuatorState, 'vehicle/sim/steering',
                                                      10)
            self.wheels_pub = self.create_publisher(WheelSpeeds, 'vehicle/sim/wheels', 10)
            self.create_subscription(SpineOutputs, 'vehicle/sim/spine_outputs',
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
                self.create_subscription(SpineOutputs, 'vehicle/sim/esp32_outputs',
                                         self.on_cmd, 10)
                self.get_logger().info("today's ESP32 in the loop: sim_esp32 drives the "
                                       'actuator model')
            else:
                self.create_subscription(AckermannDriveStamped, 'control/sim/actuator_cmd',
                                         self.on_cmd, 10)
        self.create_subscription(Odometry, 'vehicle/sim/odom', self.on_odom, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_timer(1.0 / rate, self.tick)
        rendered = self.declare_parameter('rendered', False).value
        # Our own simulator renders the lidar and radar; aav_sim_cameras draws the cameras.
        cameras = self.declare_parameter('cameras', rendered).value
        self.announce(sources.simulated(rendered=rendered, lidar=lidar,
                                        radar=self.declare_parameter('radar', False).value,
                                        vehicle=not self.spine, cameras=cameras))
        # The wheels' fault on purpose: one per stream (/vehicle/odom, /vehicle/wheels),
        # set together. The other sensors' faults are their own nodes' (faults.NODES).
        self.faults = {'odom': [faults.Fault('odom'), faults.Fault('odom')]}
        for sensor in self.faults:
            self.declare_parameter(f'fault.{sensor}', 'ok')
        # The actuators' faults on purpose (TODO 159): between the speed loop and the car.
        self.actuator_faults = faults.ActuatorFaults()
        for actuator in faults.ACTUATORS:
            self.declare_parameter(f'fault.{actuator}', 'ok')
        # Whole numbers too: `ros2 param set /sim_adapter payload 90`.
        whole = ParameterDescriptor(dynamic_typing=True)
        self.set_load(float(self.declare_parameter('payload', 0.0, whole).value),
                      bool(self.declare_parameter('adaptive_brake', False).value))
        self.add_on_set_parameters_callback(self.on_fault)

    def load_problem(self, payload, adaptive):
        """Return why this payload or adaptive brake can't be had, or None."""
        if (payload, adaptive) != (0.0, False) and not isinstance(self.model, Actuators):
            return 'payload and adaptive_brake need the actuator model (actuator_model:=true)'
        if not 0.0 <= payload <= 1000.0:
            return f'payload must be 0 to 1000 kg, not {payload}'
        if adaptive and self.outputs:
            return ("adaptive_brake is the speed loop's: with spine or esp32 the firmware "
                    'decides the brake')
        return None

    def set_load(self, payload, adaptive):
        """Put payload kg aboard the modelled car; the adaptive brake on or off."""
        why = self.load_problem(payload, adaptive)
        if why:
            raise ValueError(why)
        if not isinstance(self.model, Actuators):
            return
        self.model.cfg = replace(self.config, payload=payload)
        if adaptive and self.model.brake_loop is None:
            # It expects the empty car: the load is what it finds out.
            self.model.brake_loop = AdaptiveBrake(self.config, self.brake_limits)
        elif not adaptive:
            self.model.brake_loop = None
        self.get_logger().info(
            f'{payload:g} kg aboard (its brakes and motor at {100 * self.model.cfg.share:.0f} %'
            f' of the empty car\'s); the adaptive brake {"on" if adaptive else "off"}')

    def on_fault(self, params):
        load = {p.name: p.value for p in params if p.name in ('payload', 'adaptive_brake')}
        if load:
            payload = float(load.get('payload', self.get_parameter('payload').value))
            adaptive = bool(load.get('adaptive_brake',
                                     self.get_parameter('adaptive_brake').value))
            why = self.load_problem(payload, adaptive)
            if why:
                return SetParametersResult(successful=False, reason=why)
            self.set_load(payload, adaptive)
        for p in params:
            if not p.name.startswith('fault.'):
                continue
            sensor = p.name[len('fault.'):]
            if sensor in faults.ACTUATORS:
                why = self.set_actuator_fault(sensor, p)
                if why is not None:
                    return SetParametersResult(successful=False, reason=why)
                continue
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

    def set_actuator_fault(self, actuator, p):
        """Set an actuator's fault from its parameter: None, or why it can't."""
        mode = p.value if p.type_ == Parameter.Type.STRING else repr(p.value)
        why = faults.check_actuator(actuator, mode)
        if why is None and mode != 'ok' and not isinstance(self.model, Actuators):
            why = 'actuator faults need the actuator model (actuator_model:=true)'
        if why is not None:
            return why
        self.actuator_faults.switch(actuator, mode)
        if mode == 'ok':
            self.get_logger().info(f'fault cleared: {actuator} ok')
        else:
            self.get_logger().warning(f'FAULT ON PURPOSE: {actuator} {mode}')
        return None

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
        """Step the actuator model on Ravensim's clock, and send Ravensim what it gives."""
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
            asked = steer
            demand, steer = self.actuator_faults.apply(demand, steer)
            s = self.state = replace(self.model.drive(demand, steer, dt), steer_command=asked)
        else:
            speed = self.cmd.speed if fresh else 0.0
            steer = self.cmd.steering_angle if self.cmd is not None else 0.0
            if isinstance(self.model, Actuators):
                s = self.state = self.model.step(speed, steer, dt,
                                                 fault=self.actuator_faults.apply)
            else:
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
        # Wheels that read nothing: the car stands, exactly (TODO 191).
        odom.twist.covariance = vehicle.standstill_covariance(speed, cov)
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
    node = SimAdapter()
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

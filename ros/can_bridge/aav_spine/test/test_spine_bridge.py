"""
Check the bridge's DRIVE_CMD stream (the arming edge, going quiet with ego), and its odometry.

Needs ROS sourced (rclpy and aav_interfaces): `colcon test` runs it, tools/test.sh skips it.
"""

import math
import time

import pytest

# Not pytest.importorskip: ROS's launch_testing pytest plugin turns a module-level skip
# into a skip of every test in the folder, and it also overrides collect_ignore.
try:
    from aav_spine import can_codec as cc
    from aav_spine.spine_bridge import SpineBridge
    from ackermann_msgs.msg import AckermannDrive
    from can_msgs.msg import Frame
    import rclpy
    from rclpy.duration import Duration
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool
except ImportError:
    SpineBridge = None

pytestmark = pytest.mark.skipif(SpineBridge is None, reason='needs ROS sourced: colcon test')


@pytest.fixture
def bridge():
    rclpy.init()
    try:
        node = SpineBridge()
        node.sent = []
        node._send = lambda can_id, data: node.sent.append(cc.unpack(can_id, data))
        yield node
        node.destroy_node()
    finally:
        # Even when the node fails to start: otherwise every later test fails on init.
        rclpy.shutdown()


def fresh_command(node, speed=1.0):
    node.cmd = AckermannDrive(speed=speed)
    node.cmd_time = node.get_clock().now()


def arm(node, on=True, gate='OPEN - car: every source REAL'):
    """/spine/arm, the output mux having just said `gate` (None: not heard)."""
    if gate is not None:
        node.on_gate(String(data=gate))
    return node.on_arm(SetBool.Request(data=on), SetBool.Response())


def armed_flags(node, frames):
    node.sent.clear()
    for _ in range(frames):
        node.send_drive()
    return [bool(f['armed']) for f in node.sent]


def test_arming_sends_the_edge(bridge):
    fresh_command(bridge)
    assert armed_flags(bridge, 2) == [False, False]
    assert arm(bridge).success
    # Fresh again: on a loaded machine the first command went stale before this (24 Oct),
    # and the bridge rightly sent nothing.
    fresh_command(bridge)
    assert armed_flags(bridge, 5) == [False, False, False, True, True]


def test_arming_again_sends_the_edge_again(bridge):
    # The spine may have latched without the bridge knowing: a hardware e-stop, a
    # timeout, a driver fault or a reboot. Arming again has to clear that.
    fresh_command(bridge)
    arm(bridge)
    armed_flags(bridge, 5)
    arm(bridge)
    fresh_command(bridge)
    assert armed_flags(bridge, 5) == [False, False, False, True, True]


def test_the_safety_gate_shut_refuses_arming(bridge):
    # Ben, 24 Oct: never armed while the output mux withholds the commands.
    stop = 'STOP - target car needs REAL - imu is SIM (sim_adapter) - latched: ...'
    fresh_command(bridge)
    for gate, says in ((None, '/safety/output_mux/state'),
                       ('WAIT - car: all REAL, settling for 2 s', 'settling'),
                       (stop, stop), ('OPEN - sim: every source SIM', 'another target')):
        answer = arm(bridge, gate=gate)
        assert not answer.success, gate
        assert answer.message.startswith('the safety gate is shut'), answer.message
        assert says in answer.message, answer.message
    fresh_command(bridge)
    assert armed_flags(bridge, 5) == [False] * 5
    assert arm(bridge, on=False, gate=None).success   # disarming always works
    assert arm(bridge).success                        # OPEN for the car


def test_estop_disarms_and_refuses_arming(bridge):
    fresh_command(bridge)
    arm(bridge)
    bridge.on_estop(Bool(data=True))
    assert not arm(bridge).success
    assert armed_flags(bridge, 5) == [False] * 5


def test_stale_command_stops_the_stream(bridge):
    fresh_command(bridge)
    arm(bridge)
    bridge.cmd_time = bridge.get_clock().now() - Duration(seconds=0.15)
    assert armed_flags(bridge, 3) == []


def receive(node, can_id, values):
    data = cc.pack(can_id, values)
    node.on_frame(Frame(id=can_id, dlc=len(data), data=list(data) + [0] * (8 - len(data))))


def capture(publisher):
    out = []
    publisher.publish = out.append
    return out


def test_odometry_uses_fresh_wheel_speeds(bridge):
    odom = capture(bridge.odom_pub)
    wheels = capture(bridge.wheel_pub)
    feedback = {'measured_speed': -0.5, 'measured_steering_angle': 0.0}
    receive(bridge, cc.VEHICLE_FEEDBACK_ID, feedback)
    assert odom[-1].twist.twist.linear.x == pytest.approx(-0.5)   # no wheel frames yet

    for _ in range(2):   # the first left frame goes out alone; then they pair up
        for can_id, v in ((cc.WHEEL_SPEED_L_ID, 1.0), (cc.WHEEL_SPEED_R_ID, 1.2)):
            receive(bridge, can_id, {'wheel_speed': v, 'pulse_count': 3})
    assert wheels[-1].front_left == pytest.approx(-1.0)    # signed by the spine's reverse
    assert wheels[-1].front_right == pytest.approx(-1.2)
    assert math.isnan(wheels[-1].rear_left) and math.isnan(wheels[-1].rear_right)
    assert not wheels[-1].direction_measured
    receive(bridge, cc.VEHICLE_FEEDBACK_ID, feedback)
    assert odom[-1].twist.twist.linear.x == pytest.approx(-1.1)


def test_odometry_falls_back_when_the_wheels_go_quiet(bridge):
    odom = capture(bridge.odom_pub)
    for can_id in cc.WHEEL_SPEED_IDS:
        receive(bridge, can_id, {'wheel_speed': 2.0})
    for w in bridge.wheels.wheels.values():
        w.time -= 1.0
    receive(bridge, cc.VEHICLE_FEEDBACK_ID, {'measured_speed': 0.7})
    assert odom[-1].twist.twist.linear.x == pytest.approx(0.7)


def test_a_quiet_wheel_reads_nan(bridge):
    wheels = capture(bridge.wheel_pub)
    receive(bridge, cc.WHEEL_SPEED_L_ID, {'wheel_speed': 1.0})
    assert wheels[-1].front_left == pytest.approx(1.0)
    assert math.isnan(wheels[-1].front_right)   # never heard from


# Role sensors: the car is driven through the Motor Pi's UART, and the Orin only reads the
# wheel nodes on its CAN bus (docs/can-plan.md, "Today's wiring").

@pytest.fixture
def sensors():
    rclpy.init(args=['--ros-args', '-p', 'role:=sensors'])
    try:
        node = SpineBridge()
        yield node
        node.destroy_node()
    finally:
        rclpy.shutdown()


def names(found):
    return {name for name, _ in found}


def test_sensors_have_no_way_to_drive(sensors):
    me = (sensors.get_name(), sensors.get_namespace())
    # The graph fills in the background: on a busy computer, right after the node starts it
    # may hold only /rosout and /parameter_events. Wait for the publishers it must have, so
    # the checks for the ones it must not have look at the whole node.
    end = time.monotonic() + 10.0
    publishers = set()
    while time.monotonic() < end:
        publishers = names(sensors.get_publisher_names_and_types_by_node(*me))
        if {'/vehicle/odom', '/vehicle/wheels'} <= publishers:
            break
        time.sleep(0.05)
    assert '/can/tx' not in publishers
    assert '/vehicle/status' not in publishers and '/vehicle/steering' not in publishers
    assert {'/vehicle/odom', '/vehicle/wheels'} <= publishers
    assert '/spine/arm' not in names(sensors.get_service_names_and_types_by_node(*me))
    assert '/safety/estop' not in names(sensors.get_subscriber_names_and_types_by_node(*me))
    assert not hasattr(sensors, 'to_can')


def test_sensors_give_odometry_from_the_wheels_alone(sensors):
    odom = capture(sensors.odom_pub)
    wheels = capture(sensors.wheel_pub)
    # A spine's frame is not this role's: no odometry from it.
    receive(sensors, cc.VEHICLE_FEEDBACK_ID, {'measured_speed': 5.0})
    assert odom == []
    receive(sensors, cc.WHEEL_SPEED_L_ID, {'wheel_speed': 1.0})
    assert odom == []                               # the right wheel never heard: no mean
    receive(sensors, cc.WHEEL_SPEED_R_ID, {'wheel_speed': 1.2})
    receive(sensors, cc.WHEEL_SPEED_L_ID, {'wheel_speed': 1.0})
    assert wheels[-1].front_right == pytest.approx(1.2)
    assert odom[-1].twist.twist.linear.x == pytest.approx(1.1)
    assert odom[-1].twist.covariance[35] >= 1e3    # no yaw rate: the IMU's job


def test_sensors_sign_the_wheels_by_the_command(sensors):
    wheels = capture(sensors.wheel_pub)
    fresh_command(sensors, speed=-0.5)
    receive(sensors, cc.WHEEL_SPEED_L_ID, {'wheel_speed': 0.5})
    assert wheels[-1].front_left == pytest.approx(-0.5)


def test_an_unknown_role_is_refused():
    rclpy.init(args=['--ros-args', '-p', 'role:=drive'])
    try:
        with pytest.raises(ValueError, match='role:=drive'):
            SpineBridge()
    finally:
        rclpy.shutdown()

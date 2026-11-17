"""
arm_when_open: arm the spine once the safety gate opens; the bench's and Gazebo's arm:=true.

Calls       spine/arm   std_srvs/SetBool (data: true), every `retry` s

spine_bridge refuses to arm while the output mux isn't OPEN (Ben, 24 Oct), so this asks
again each `retry` s while spine_bridge isn't up yet or the gate alone refuses, then
exits: 0 once armed, 1 on any other refusal (an e-stop: a person pressed it) or after
`give_up` s. It never arms a second time. Only the stand-ins start it (bench.launch.py,
ravensim.launch.py spine:=true); on the car a person arms.

Parameters  retry (1.0 s), give_up (120.0 s)
"""

import time

from aav_spine.output_gate import GATE_SHUT, try_again
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool


def ask(node, client, timeout):
    """One /spine/arm call: (success, message). Not up or no answer counts as shut."""
    if not client.wait_for_service(timeout_sec=timeout):
        return False, f'{GATE_SHUT}: spine_bridge has no /spine/arm yet'
    future = client.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        return False, f'{GATE_SHUT}: no answer from /spine/arm'
    return future.result().success, future.result().message


def main(args=None):
    rclpy.init(args=args)
    node = Node('arm_when_open')
    retry = node.declare_parameter('retry', 1.0).value
    give_up = node.declare_parameter('give_up', 120.0).value
    client = node.create_client(SetBool, 'spine/arm')
    log = node.get_logger()
    start = time.monotonic()
    code = 1
    try:
        while rclpy.ok():
            asked = time.monotonic()
            success, message = ask(node, client, retry)
            waited = time.monotonic() - start
            if success:
                log.info(f'{message} after {waited:.0f} s')
                code = 0
                break
            if not try_again(success, message, waited, give_up):
                gave_up = f' (gave up after {waited:.0f} s)' if waited >= give_up else ''
                log.error(f'not armed: {message}{gave_up}')
                break
            log.info(f'waiting to arm: {message}', throttle_duration_sec=10.0)
            time.sleep(max(0.0, retry - (time.monotonic() - asked)))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Put Ravensim's car on stands ("on jacks"), or back on the ground, while it runs.

    pixi run jacks status --sim      on stands, or on the ground?
    pixi run jacks up --sim          lift it: the rear wheels spin and steer, the car stays put
    pixi run jacks down --sim        lower it: refused while the wheels spin or a speed is asked
    ... --json                       one line of JSON instead (the ladder window reads it)

Run it next to `pixi run sim`, on the same ROS domain. It is for the simulator only, and
the task runs it through tools/ros.sh --local, so it can never reach a real car. It still
checks: it does nothing unless --sim is given and every announcement on /system/source
says SIM. The real car goes on real stands, by hand.

The rule for the switch is the simulator's own (aav_simulator, sim-core jacks.rs): this
tool only asks, and prints the simulator's answer.

Subscribes  /system/source      aav_interfaces/DataSource, latched: SIM, or it refuses
            /vehicle/sim/jacks  std_msgs/Bool, latched: true = on stands
Calls       /vehicle/sim/set_jacks  std_srvs/SetBool (up: true, down: false)

Exit codes: 0 done (or the status printed), 2 not asked of a simulator (no --sim, a real
car, no simulator heard), 3 the simulator refused.
"""

import argparse
import json
import sys
import time

REAL, SIM, BENCH, REPLAY = 0, 1, 2, 3     # aav_interfaces/SensorSource
VERBS = ('status', 'up', 'down')
OK, NOT_A_SIM, REFUSED = 0, 2, 3


def why_not(sim_flag, sources):
    """
    Return why the jacks can't be used here, or None.

    sim_flag: --sim was given. sources: announcer -> its /system/source values.
    Anything REAL refuses, even beside a simulator: one real actuator is enough.
    """
    if not sim_flag:
        return ('--sim is needed: this switch is for Ravensim only. The real car goes on '
                'real stands, by hand')
    values = [v for entries in sources.values() for v in entries]
    if not values:
        return ('no simulator heard on this computer (nothing on /system/source): '
                'start one with pixi run sim')
    if REAL in values:
        who = sorted(a for a, entries in sources.items() if REAL in entries)
        return (f'/system/source says REAL ({", ".join(who)}): a real car is on this '
                'network. Nothing was sent')
    if BENCH in values or REPLAY in values:
        return ('/system/source says BENCH or REPLAY, not SIM: only Ravensim has jacks')
    return None


def words(on_stands):
    """Say where the car is; on_stands None means the simulator has not said."""
    if on_stands is None:
        return 'unknown: the simulator has not said (an older Ravensim?)'
    return 'on stands' if on_stands else 'on the ground'


def report(verb, simulator, on_stands=None, ok=True, message=''):
    """Return the answer as a dict: what --json prints."""
    return {'verb': verb, 'simulator': bool(simulator), 'on_stands': on_stands,
            'ok': bool(ok), 'message': message}


def text(answer):
    """Return the answer as a person reads it."""
    if not answer['simulator']:
        return f'REFUSED: {answer["message"]}'
    state = f'The simulated car: {words(answer["on_stands"])}'
    if answer['verb'] == 'status':
        return state
    head = 'Done' if answer['ok'] else 'REFUSED'
    return f'{head}: {answer["message"]}\n{state}'


def exit_code(answer):
    if not answer['simulator']:
        return NOT_A_SIM
    return OK if answer['ok'] else REFUSED


def parse(argv):
    parser = argparse.ArgumentParser(
        prog='pixi run jacks', description="Ravensim's car on stands, or on the ground.")
    parser.add_argument('verb', choices=VERBS)
    parser.add_argument('--sim', action='store_true',
                        help='required: this is for the simulator only')
    parser.add_argument('--json', action='store_true', help='one line of JSON')
    parser.add_argument('--wait', type=float, default=5.0,
                        help='s to wait for the simulator to be heard (5)')
    return parser.parse_args(argv)


def ask(args):
    """Talk to the simulator; return the answer (see report)."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from aav_interfaces.msg import DataSource
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    rclpy.init()
    node = rclpy.create_node('jacks_cli')
    try:
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sources, state = {}, {}

        def on_source(msg):
            sources[msg.announcer] = [s.source for s in msg.sensors]

        node.create_subscription(DataSource, '/system/source', on_source, latched)
        node.create_subscription(Bool, '/vehicle/sim/jacks',
                                 lambda msg: state.update(on_stands=bool(msg.data)), latched)

        def spin(seconds, done=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not done():
                rclpy.spin_once(node, timeout_sec=0.05)

        spin(args.wait, lambda: bool(sources) and 'on_stands' in state)
        spin(0.5)    # the other announcers: one REAL among them is enough to refuse
        why = why_not(args.sim, sources)
        if why:
            return report(args.verb, False, message=why)
        if args.verb == 'status':
            return report('status', True, state.get('on_stands'))
        client = node.create_client(SetBool, '/vehicle/sim/set_jacks')
        if not client.wait_for_service(timeout_sec=args.wait):
            return report(args.verb, True, state.get('on_stands'), ok=False,
                          message='no /vehicle/sim/set_jacks: this simulator has no jacks')
        future = client.call_async(SetBool.Request(data=args.verb == 'up'))
        spin(args.wait, future.done)
        if not future.done():
            return report(args.verb, True, state.get('on_stands'), ok=False,
                          message=f'the simulator did not answer in {args.wait:g} s')
        result = future.result()
        want = args.verb == 'up'
        if result.success:
            spin(1.0, lambda: state.get('on_stands') == want)   # the latched state follows
        return report(args.verb, True, state.get('on_stands'), ok=result.success,
                      message=result.message)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    # No --sim: refuse before any ROS, so nothing is even listened to.
    answer = (report(args.verb, False, message=why_not(False, {})) if not args.sim
              else ask(args))
    print(json.dumps(answer) if args.json else text(answer))
    return exit_code(answer)


if __name__ == '__main__':
    sys.exit(main())

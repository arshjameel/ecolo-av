"""
Sensor faults on purpose, in Ravensim: what a broken sensor sends instead. No ROS here.

Each of Ravensim's sensor nodes passes its sensor on through a Fault (NODES;
docs/full-simulation.md step 6; docs/simulation.md, "Sensor failures on purpose"). Set one
while the sim runs:

    pixi run sim-fault gps stop        (ros2 param set /sim_gps fault.gps stop)
    pixi run sim-fault gps ok          (back to normal)

The modes, and what they stand for on the car:

  ok      passed on as it is
  stop    nothing sent: a cable pulled, a driver crashed, a GPS under a bridge
  freeze  the last good message again, with fresh stamps: a driver stuck in a loop.
          It looks alive: only the values give it away
  nan     NaN in every number: a driver that parsed garbage
  jump    the GPS only: the position jumps JUMP_NORTH m north and stays there (a bad
          multipath fix, an RTK fix on the wrong integer)
  drift   the GPS only: the position slides north, DRIFT_RATE m/s, until DRIFT_MAX m off,
          and stays (TODO 210: multipath near buildings, a bad correction stream). The
          hard case: no jump, never silent, the fix still says it is good

The sensors: gps (/sensors/gps/fix), imu (/sensors/imu/data_raw), odom (the wheels:
/vehicle/odom and /vehicle/wheels), radar (/sensors/radar/points), camera (the front
left camera's picture) and lidar (/sensors/lidar/points). The camera and the lidar only
with faults:=true, which routes them through sim_camera_relay and sim_lidar_relay.
"""

import copy
from dataclasses import replace
from typing import Optional

SENSORS = {
    'gps': ('ok', 'stop', 'freeze', 'nan', 'jump', 'drift'),
    'imu': ('ok', 'stop', 'freeze', 'nan'),
    'odom': ('ok', 'stop', 'freeze', 'nan'),
    'radar': ('ok', 'stop', 'freeze'),
    'camera': ('ok', 'stop', 'freeze'),
    'lidar': ('ok', 'stop', 'freeze'),
}
# Which node passes each sensor on, and so takes its fault.<sensor> parameter: one node
# per sensor (relay.py); the wheels go with the car, in sim_adapter.
NODES = {
    'gps': 'sim_gps',
    'imu': 'sim_imu',
    'odom': 'sim_adapter',
    'radar': 'sim_radar',
    'camera': 'sim_camera_relay',
    'lidar': 'sim_lidar_relay',
}
JUMP_NORTH = 20.0               # m
# The drift: a slow bias of a few metres over a minute, 0.083 m/s: under the watchdog's
# 0.15 m/s wheels-against-GPS speed check.
DRIFT_MAX = 5.0                 # m
DRIFT_RATE = DRIFT_MAX / 60.0   # m/s
METRES_PER_DEGREE = 111195.0    # of latitude, on a 6371 km Earth


def check(sensor: str, mode: str) -> Optional[str]:
    """Return why this sensor can't have this fault, or None if it can."""
    if sensor not in SENSORS:
        return f'no sensor {sensor!r}: pick one of {", ".join(SENSORS)}'
    if mode not in SENSORS[sensor]:
        return f'{sensor} has no {mode!r} fault: pick one of {", ".join(SENSORS[sensor])}'
    return None


def jumped(latitude: float, metres: float = JUMP_NORTH) -> float:
    """Return the latitude `metres` further north."""
    return latitude + metres / METRES_PER_DEGREE


def drifted(seconds: float) -> float:
    """Return how far north (m) the drift has slid `seconds` after it began."""
    return min(DRIFT_MAX, DRIFT_RATE * max(0.0, seconds))


class Fault:
    """One sensor's fault: decides what goes out in place of each message."""

    def __init__(self, sensor: str):
        self.sensor = sensor
        self.mode = 'ok'
        self.held = None        # the last message passed on while ok: what freeze repeats
        self.since = None       # drift: the time it began (the first message after)

    def switch(self, mode: str):
        why = check(self.sensor, mode)
        if why is not None:
            raise ValueError(why)
        self.mode = mode
        self.since = None

    def apply(self, msg, nan=None, jump=None, drift=None, now=None):
        """
        Return what to send for msg, or None to send nothing.

        nan(msg), jump(msg) and drift(msg, seconds) change a message in place: the caller
        knows its type. They get a copy, so the held message stays good. now (s, the
        message's own time) times the drift from its first message.
        """
        if self.mode == 'ok':
            self.held = msg
            return msg
        if self.mode == 'stop':
            return None
        if self.mode == 'freeze':
            return copy.deepcopy(self.held) if self.held is not None else None
        out = copy.deepcopy(msg)
        if self.mode == 'drift':
            if self.since is None:
                self.since = now
            drift(out, now - self.since)
        else:
            (nan if self.mode == 'nan' else jump)(out)
        return out


# Actuator faults on purpose (TODO 159): the car's own actuators, in sim_adapter, between
# the speed loop and the car model. What each stands for on the car:
#   steer     stuck: the steering actuator holds the angle it was asked for when it failed
#             (a jammed linkage, a Jrk that stopped following: hazard 7)
#   throttle  stuck: the throttle holds its level from the moment it failed, whatever the
#             speed loop asks; the brake still works against it (a stuck DAC: hazard 3)
#   brake     weak: WEAK_BRAKE of what the loop asks; dead: none (a slipping or unpowered
#             brake actuator: hazard 8)
ACTUATORS = {
    'steer': ('ok', 'stuck'),
    'throttle': ('ok', 'stuck'),
    'brake': ('ok', 'weak', 'dead'),
}
WEAK_BRAKE = 0.3    # the share of the asked brake a weak one gives
ACTUATOR_NODE = 'sim_adapter'    # the car's node takes every actuator's fault


def check_actuator(actuator: str, mode: str) -> Optional[str]:
    """Return why this actuator can't have this fault, or None if it can."""
    if actuator not in ACTUATORS:
        return f'no actuator {actuator!r}: pick one of {", ".join(ACTUATORS)}'
    if mode not in ACTUATORS[actuator]:
        return (f'{actuator} has no {mode!r} fault: pick one of '
                f'{", ".join(ACTUATORS[actuator])}')
    return None


class ActuatorFaults:
    """
    The actuators' faults on purpose: what reaches the car in place of the loop's demand.

    apply(demand, steer) takes the speed loop's Demand (car_model's) and the steering
    target, and returns what the faulty actuators really do. A stuck one holds what it
    was asked for at the first apply after the fault was set.
    """

    def __init__(self):
        self.modes = dict.fromkeys(ACTUATORS, 'ok')
        self.held = {}          # actuator -> the value it is stuck at

    def switch(self, actuator: str, mode: str):
        why = check_actuator(actuator, mode)
        if why is not None:
            raise ValueError(why)
        self.modes[actuator] = mode
        self.held.pop(actuator, None)

    def apply(self, demand, steer: float):
        """Return (demand, steer) as the faulty actuators give them."""
        if self.modes['steer'] == 'stuck':
            steer = self.held.setdefault('steer', steer)
        if self.modes['throttle'] == 'stuck':
            demand = replace(demand, throttle=self.held.setdefault('throttle', demand.throttle))
        brake = self.modes['brake']
        if brake != 'ok':
            share = WEAK_BRAKE if brake == 'weak' else 0.0
            demand = replace(demand, brake=demand.brake * share)
        return demand, steer


CAMERA = '/sensors/camera/front_left/image_raw'
# faults:=true: the cameras put the front left picture here, and sim_camera_relay passes it
# on to CAMERA through its fault.
CAMERA_RELAYED = '/vehicle/sim/camera/front_left/image_raw'
LIDAR = '/sensors/lidar/points'
# faults:=true: the simulator's lidar is remapped here, and sim_lidar_relay passes it on to
# LIDAR through its fault (TODO 246).
LIDAR_RELAYED = '/vehicle/sim/lidar/points'

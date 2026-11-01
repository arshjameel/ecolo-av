"""
TODO 52e: body roll must not tilt the roof lidar enough to see the road as an obstacle.

Puts together what decides it, each from where it lives:
- the lidar's mount (vehicle.yaml) and the lowest obstacle height (vehicle.py, 0.3 m);
- how far out the local costmap marks the lidar's points (nav2.yaml, 12 m);
- ego's limits: the sideways acceleration it allows, its steering rate and braking
  (ego.yaml, aav_ego dynamics.py), driven through ego's own model (ackermann.EgoModel);
- the roll model with its GUESSED springs (tools/state_space_demo.py, docs section 10).

The worst case: driving straight, then told full lock at once. Each speed must keep the
roll's peak (the springs overshoot) under aav_lidar ground.max_safe_roll. This fails if
someone raises max_lateral_accel or the costmap's range, lowers the obstacle height, or
measured springs turn out softer than about 7400 N/m a corner: then fix it first.
"""

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
for path in ('tools', 'ros/ego/aav_ego', 'ros/bringup/aav_description',
             'ros/perception/aav_lidar'):
    sys.path.insert(0, str(ROOT / path))
try:
    import numpy as np
    from scipy import signal
    import state_space_demo as ss  # noqa: E402
    HAVE_SCIPY = True
except ImportError:          # no numpy or scipy here: pixi run test has them
    HAVE_SCIPY = False

from aav_description import vehicle  # noqa: E402
from aav_ego.ackermann import EgoModel, Limits  # noqa: E402
from aav_ego.dynamics import Body  # noqa: E402
from aav_lidar.ground import max_safe_roll, worst_ground_height  # noqa: E402
import pytest  # noqa: E402
import yaml  # noqa: E402

pytestmark = pytest.mark.skipif(not HAVE_SCIPY, reason='needs numpy and scipy')

DT = 0.02           # s: ego's 50 Hz


def load(relative):
    with open(ROOT / relative) as f:
        return yaml.safe_load(f)


CAR = load('ros/bringup/aav_description/config/vehicle.yaml')
EGO = load('ros/ego/aav_ego/config/ego.yaml')['ego']['ros__parameters']
NAV2 = load('ros/drivers/aav_navigation/config/nav2.yaml')
LIDAR = CAR['sensors']['lidar_link']['xyz']
MIN_HEIGHT = vehicle.obstacle_heights(CAR['vehicle'])[0]
RANGE = (NAV2['local_costmap']['local_costmap']['ros__parameters']['lidar_layer']['lidar']
         ['obstacle_max_range'])


def ego(speed):
    """Return ego's model, as ego.yaml and vehicle.yaml set it up, cruising at speed."""
    v = CAR['vehicle']
    params = vehicle.ego_params(CAR)
    lim = Limits(wheelbase=v['wheelbase'], max_steering_angle=v['max_steering_angle'],
                 max_speed=speed, max_reverse_speed=v['max_reverse_speed'],
                 max_acceleration=EGO['max_acceleration'],
                 max_deceleration=EGO['max_deceleration'],
                 max_steering_rate=EGO['max_steering_rate'], max_jerk=EGO['max_jerk'])
    body = Body(mass=params['mass'], track=params['track'], cg_height=params['cg_height'],
                tyre_grip=params['tyre_grip'], full_throttle_accel=params['full_throttle_accel'],
                full_brake_decel=params['full_brake_decel'],
                max_lateral_accel=EGO['max_lateral_accel'])
    model = EgoModel(lim, body)
    model.speed = speed
    return model


def peak_roll(speed):
    """Return the roll's peak (rad): straight at speed, then full lock asked at once."""
    model = ego(speed)
    wheelbase, lock = CAR['vehicle']['wheelbase'], CAR['vehicle']['max_steering_angle']
    sideways = []
    for i in range(int(8.0 / DT)):
        turn = speed * math.tan(lock) / wheelbase if i * DT >= 1.0 else 0.0
        v, steering, _ = model.step((speed, turn), DT)
        sideways.append(v * v * math.tan(steering) / wheelbase)
    s = ss.car_corners(ss.load_car())
    roll = ss.roll_model(s['m_s'], s['arm'], s['k_roll'], s['inertia'], ss.SUSPENSION['zeta'])
    t = np.arange(len(sideways)) * DT
    _, phi, _ = signal.lsim(roll, np.array(sideways), t)
    return float(np.max(np.abs(phi)))


def test_the_settings_are_the_ones_these_sums_assume():
    assert MIN_HEIGHT == pytest.approx(0.3)
    assert RANGE == pytest.approx(12.0)
    # Measured 31 Oct (was a 1.8 m guess). Lower is safer: the road shows from 1.73 deg
    # of roll, not 1.53; ego's worst stays 1.26 deg.
    assert LIDAR[2] == pytest.approx(1.52)


def test_at_today_s_1_m_s_the_road_stays_well_below_the_obstacle_height():
    z, *_ = worst_ground_height(peak_roll(1.0), LIDAR, RANGE)
    assert z < MIN_HEIGHT - 0.2         # 0.04 m at most: 0.26 m to spare


@pytest.mark.parametrize('speed', [1.0, 2.0, 3.0, 4.0])
def test_up_to_4_m_s_ego_keeps_the_roll_under_what_the_lidar_allows(speed):
    # ego holds the sideways push to 2 m/s^2: about 1.3 degrees of roll at most, with the
    # springs' overshoot, against 1.46 degrees before the road shows within 12 m.
    assert peak_roll(speed) < max_safe_roll(LIDAR, MIN_HEIGHT, RANGE)


def test_at_5_m_s_a_sudden_full_lock_would_tilt_it_too_far():
    # ego can't shed speed as fast as the wheels turn (2 m/s^2 against 0.6 rad/s), so the
    # push overshoots its cap: what to fix before driving that fast.
    assert peak_roll(5.0) > max_safe_roll(LIDAR, MIN_HEIGHT, RANGE)

import math
from types import SimpleNamespace

from aav_description.car_model import Actuators, Demand
from aav_sim_models import faults
from aav_sim_models.faults import Fault
import pytest


def reading(value):
    return SimpleNamespace(value=value, stamp=0)


def test_ok_passes_on_and_stop_sends_nothing():
    f = Fault('imu')
    msg = reading(1.0)
    assert f.apply(msg) is msg
    f.switch('stop')
    assert f.apply(reading(2.0)) is None


def test_freeze_repeats_the_last_good_message_as_a_copy():
    f = Fault('odom')
    f.apply(reading(1.0))
    f.switch('freeze')
    out = f.apply(reading(2.0))
    assert out.value == 1.0
    out.stamp = 99                       # the adapter restamps it: the held one stays
    assert f.apply(reading(3.0)).stamp == 0


def test_freeze_before_anything_sends_nothing():
    f = Fault('camera')
    f.switch('freeze')
    assert f.apply(reading(1.0)) is None


def test_nan_and_jump_change_a_copy():
    def to_nan(m):
        m.value = float('nan')

    def north(m):
        m.value = faults.jumped(m.value)

    f = Fault('gps')
    msg = reading(45.0)
    f.switch('nan')
    assert math.isnan(f.apply(msg, nan=to_nan, jump=north).value)
    f.switch('jump')
    out = f.apply(msg, nan=to_nan, jump=north)
    assert (out.value - 45.0) * faults.METRES_PER_DEGREE == pytest.approx(faults.JUMP_NORTH)
    assert msg.value == 45.0


def test_drift_slides_north_slowly_then_stays_and_restarts_when_set_again():
    def drift(m, seconds):
        m.value = faults.jumped(m.value, faults.drifted(seconds))

    def north_m(out):
        return (out.value - 45.0) * faults.METRES_PER_DEGREE

    f = Fault('gps')
    f.switch('drift')
    assert north_m(f.apply(reading(45.0), drift=drift, now=100.0)) == pytest.approx(0.0)
    assert north_m(f.apply(reading(45.0), drift=drift, now=130.0)) == pytest.approx(2.5)
    assert north_m(f.apply(reading(45.0), drift=drift, now=400.0)) == pytest.approx(
        faults.DRIFT_MAX)
    # Never a jump for the watchdog (15 m/s + 5 m) or its speed check (0.15 m/s).
    assert faults.DRIFT_RATE < 0.15
    f.switch('drift')
    assert north_m(f.apply(reading(45.0), drift=drift, now=500.0)) == pytest.approx(0.0)
    assert faults.check('gps', 'drift') is None
    assert 'no' in faults.check('imu', 'drift')


def test_each_sensor_has_only_its_own_faults():
    assert faults.check('gps', 'jump') is None
    assert 'no' in faults.check('imu', 'jump')
    assert 'no sensor' in faults.check('sonar', 'stop')
    with pytest.raises(ValueError):
        Fault('radar').switch('nan')


def test_every_sensor_has_one_node_for_its_fault():
    assert set(faults.NODES) == set(faults.SENSORS)
    assert faults.NODES['odom'] == 'sim_adapter'          # the wheels go with the car
    assert len(set(faults.NODES.values())) == len(faults.NODES)


def test_a_stuck_steering_actuator_holds_the_angle_it_had():
    f = faults.ActuatorFaults()
    d = Demand(throttle=0.2, brake=0.0, reverse=False)
    assert f.apply(d, 0.1) == (d, 0.1)
    f.switch('steer', 'stuck')
    assert f.apply(d, 0.1)[1] == 0.1
    assert f.apply(d, -0.4)[1] == 0.1           # asked to turn the other way: it doesn't
    f.switch('steer', 'ok')
    assert f.apply(d, -0.4)[1] == -0.4


def test_a_stuck_throttle_keeps_pushing_while_the_brake_still_works():
    f = faults.ActuatorFaults()
    f.switch('throttle', 'stuck')
    f.apply(Demand(throttle=0.3, brake=0.0, reverse=False), 0.0)
    out, _ = f.apply(Demand(throttle=0.0, brake=0.8, reverse=False), 0.0)
    assert (out.throttle, out.brake) == (0.3, 0.8)


def test_a_weak_brake_gives_a_share_and_a_dead_one_none():
    f = faults.ActuatorFaults()
    d = Demand(throttle=0.0, brake=1.0, reverse=False)
    f.switch('brake', 'weak')
    assert f.apply(d, 0.0)[0].brake == pytest.approx(faults.WEAK_BRAKE)
    f.switch('brake', 'dead')
    assert f.apply(d, 0.0)[0].brake == 0.0
    assert 'no' in faults.check_actuator('brake', 'stuck')
    assert 'no actuator' in faults.check_actuator('gear', 'stuck')
    with pytest.raises(ValueError):
        f.switch('steer', 'weak')


def test_a_dead_brake_lets_the_car_model_roll_on():
    def roll(fault):
        car = Actuators()
        car.speed = 1.0
        for _ in range(300):                        # 3 s at 0.01 s, asked to stop
            s = car.step(0.0, 0.0, 0.01, fault=fault)
        return s

    healthy = roll(None)
    f = faults.ActuatorFaults()
    f.switch('brake', 'dead')
    dead = roll(f.apply)
    assert healthy.speed < 0.05 < dead.speed
    assert dead.brake_command > 0.0 and dead.brake == 0.0   # asked, never given


def test_the_lidar_can_stop_and_freeze_through_its_own_relay():
    assert faults.check('lidar', 'stop') is None
    assert faults.check('lidar', 'freeze') is None
    assert faults.check('lidar', 'nan') is not None     # a cloud's NaN is a ray that missed
    assert faults.NODES['lidar'] == 'sim_lidar_relay'
    assert faults.LIDAR_RELAYED.startswith('/vehicle/sim/')
    f = Fault('lidar')
    f.switch('stop')
    assert f.apply(reading(1.0)) is None

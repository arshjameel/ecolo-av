"""The pedal: a person's stick as accelerator and brake (pedal.py, Ben 15 Nov)."""

from aav_ego.ackermann import EgoModel, Limits
from aav_ego.pedal import may_shift, Pedal, slow_rate
import pytest

LIM = Limits(max_speed=1.0, max_reverse_speed=0.5, max_acceleration=1.0,
             max_deceleration=2.0, max_jerk=0.0)
PEDAL = Pedal(let_off=1.0, full=1.0)
DT = 0.02


def test_letting_off_more_slows_harder():
    rates = [slow_rate(PEDAL, 1.0, target, 2.0) for target in (0.9, 0.5, 0.0)]
    assert rates == sorted(rates)
    assert rates[0] == pytest.approx(0.1)
    assert rates[-1] == pytest.approx(1.0)          # let go: engine braking


def test_pulling_back_harder_brakes_harder():
    gentle = slow_rate(PEDAL, 1.0, -0.1, 2.0)
    full = slow_rate(PEDAL, 1.0, -1.0, 2.0)
    assert 1.0 < gentle < full
    assert full == pytest.approx(2.0)               # all the way back: full brake
    # Backwards, the same with the directions swapped.
    assert slow_rate(PEDAL, -0.5, 1.0, 2.0) == pytest.approx(2.0)
    assert slow_rate(PEDAL, -0.5, 0.0, 2.0) == pytest.approx(1.0)


def test_never_past_the_full_brake():
    assert slow_rate(Pedal(let_off=5.0), 1.0, 0.0, 2.0) == pytest.approx(2.0)


def run(model, v, seconds):
    speeds = []
    for _ in range(round(seconds / DT)):
        speed, _, _ = model.step((v, 0.0), DT)
        speeds.append(speed)
    return speeds


def moving(pedal):
    model = EgoModel(LIM)
    model.pedal = pedal
    run(model, 1.0, 2.0)
    assert model.speed == pytest.approx(1.0)
    return model


def test_full_forward_then_full_back_stops_holds_then_reverses():
    model = moving(PEDAL)
    speeds = run(model, -1.0, 4.0)
    stop = next(i for i, s in enumerate(speeds) if s <= 0.0)
    assert stop * DT == pytest.approx(0.5, abs=0.05)    # 1 m/s at 2 m/s^2
    back = next(i for i, s in enumerate(speeds) if s < 0.0)
    assert all(s == 0.0 for s in speeds[stop:back])     # held at 0, the brake on
    assert (back - stop) * DT >= PEDAL.shift_hold       # for the whole hold
    assert (back - stop) * DT < PEDAL.shift_hold + 3 * DT
    assert speeds[-1] == pytest.approx(-0.5)            # then reverses, to its limit


def test_backwards_then_forward_waits_the_same():
    model = EgoModel(LIM)
    model.pedal = PEDAL
    run(model, -0.5, 1.0)
    speeds = run(model, 1.0, 3.0)
    stop = next(i for i, s in enumerate(speeds) if s >= 0.0)
    ahead = next(i for i, s in enumerate(speeds) if s > 0.0)
    assert (ahead - stop) * DT >= PEDAL.shift_hold


def test_a_fresh_wheel_speed_must_also_be_stopped():
    model = moving(PEDAL)
    model.wheel_speed = 0.3                             # the wheels still turn
    speeds = run(model, -1.0, 3.0)
    assert min(speeds) == 0.0                           # never reverses
    model.wheel_speed = 0.01                            # now they have stopped
    assert run(model, -1.0, 0.1)[-1] < 0.0              # the hold is long over: reverse
    assert may_shift(PEDAL, 2.0, None)
    assert may_shift(PEDAL, 2.0, -0.04)
    assert not may_shift(PEDAL, 2.0, -0.06)
    assert not may_shift(PEDAL, 0.5, 0.0)


def test_from_rest_and_the_same_way_go_at_once():
    model = EgoModel(LIM)
    model.pedal = PEDAL
    assert run(model, -0.5, 0.04)[-1] < 0.0             # never moved: no hold
    model = moving(PEDAL)
    run(model, 0.0, 2.0)                                # let go: stops
    assert run(model, 1.0, 0.04)[-1] > 0.0              # the same way again: at once


def test_no_hold_set_reverses_at_once():
    model = moving(Pedal(shift_hold=0.0))
    speeds = run(model, -1.0, 1.0)
    stop = next(i for i, s in enumerate(speeds) if s <= 0.0)
    assert speeds[stop + 1] < 0.0


def test_the_e_stop_still_brakes_fully_at_once():
    model = moving(PEDAL)
    model.pedal = None                                  # ego_node: off during an e-stop
    speeds = [model.step((-1.0, 0.0), DT)[0] for _ in range(round(0.5 / DT))]
    assert speeds[0] == pytest.approx(1.0 - 2.0 * DT)  # max_deceleration from the first tick
    assert speeds[-1] == pytest.approx(0.0, abs=1e-9)


def test_gently_back_brakes_gently_then_reverses_slowly():
    model = moving(PEDAL)
    speeds = run(model, -0.2, 3.0)
    stop = next(i for i, s in enumerate(speeds) if s <= 0.0)
    assert 0.7 < stop * DT < 1.0                        # 1.2 m/s^2: about 0.83 s
    assert speeds[-1] == pytest.approx(-0.2)


def test_let_go_rolls_to_a_stop_like_engine_braking():
    model = moving(PEDAL)
    speeds = run(model, 0.0, 2.0)
    stop = next(i for i, s in enumerate(speeds) if s <= 0.0)
    assert stop * DT == pytest.approx(1.0, abs=0.05)    # 1 m/s at 1 m/s^2
    assert speeds[-1] == 0.0                            # and stays stopped


def test_without_the_pedal_every_slowdown_is_full():
    model = moving(None)
    speeds = run(model, 0.0, 1.0)
    stop = next(i for i, s in enumerate(speeds) if s <= 0.0)
    assert stop * DT == pytest.approx(0.5, abs=0.05)


def test_a_stale_command_still_brakes_fully():
    model = moving(PEDAL)
    for _ in range(round(0.5 / DT)):
        model.step(None, DT)
    assert model.speed == pytest.approx(0.0, abs=1e-9)

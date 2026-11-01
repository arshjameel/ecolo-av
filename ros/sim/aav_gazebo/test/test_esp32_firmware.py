"""The C++ ESP32 firmware's stand-in (esp32_firmware): frames, ramp, brake, as on the car."""

import struct

from aav_gazebo.esp32_firmware import (ACCEL_STEP, DAC_CAP, DECEL_STEP, Esp32Firmware,
                                       NEGATIVE_STEERING_GAIN)
import pytest


def frame(steering, speed, accel):
    body = struct.pack('<fff', steering, speed, accel)
    return body + bytes([sum(body) & 0xFF])


def test_at_boot_the_brake_is_full_and_the_throttle_off():
    out = Esp32Firmware().outputs()
    assert (out.throttle, out.brake, out.reverse) == (0.0, 1.0, False)


def test_the_throttle_ramps_up_six_counts_a_frame():
    fw = Esp32Firmware()
    for n in range(1, 11):
        fw.feed(frame(0.0, 1.0, 0.0))
        assert fw.dac == n * ACCEL_STEP
    assert fw.outputs().brake == 0.0          # a drive frame releases the brake


def test_full_throttle_takes_117_frames_about_2_3_s_at_50_hz():
    fw = Esp32Firmware()
    n = 0
    while fw.dac < DAC_CAP:
        fw.feed(frame(0.0, 1.0, 0.0))
        n += 1
    assert n == 117


def test_the_throttle_ramps_down_twelve_counts_a_frame():
    fw = Esp32Firmware()
    fw.dac = 300
    fw.feed(frame(0.0, 0.2, 0.0))              # 0.2 x 700 = 140
    assert fw.dac == 300 - DECEL_STEP


def test_below_three_percent_the_throttle_ramps_to_zero():
    fw = Esp32Firmware()
    fw.dac = 20
    fw.feed(frame(0.0, 0.02, 0.0))
    assert fw.dac == 8
    fw.feed(frame(0.0, 0.02, 0.0))
    assert fw.dac == 0


def test_a_brake_cuts_the_throttle_at_once():
    fw = Esp32Firmware()
    fw.dac = 500
    fw.feed(frame(0.0, 0.5, -0.5))
    out = fw.outputs()
    assert out.throttle == 0.0
    assert fw.brake == 1750
    assert out.brake == pytest.approx((1750 - 400) / 3100)


def test_a_small_brake_number_still_means_released():
    fw = Esp32Firmware()
    fw.feed(frame(0.0, 0.0, -0.1))             # 0.1 x 3500 = 350, clamped to 400
    assert fw.brake == 400 and fw.outputs().brake == 0.0


def test_above_minus_005_the_brake_is_released():
    fw = Esp32Firmware()
    fw.feed(frame(0.0, 0.0, -0.04))
    assert fw.outputs().brake == 0.0


def test_a_bad_checksum_drops_a_byte_and_finds_the_next_frame():
    fw = Esp32Firmware()
    good = frame(0.0, 1.0, 0.0)
    assert fw.feed(b'\x00\x01' + good) == 1
    assert fw.dac == ACCEL_STEP


def test_a_frame_out_of_range_ramps_down_and_changes_nothing_else():
    fw = Esp32Firmware()
    fw.feed(frame(0.3, 1.0, 0.0))
    fw.dac = 100
    fw.feed(frame(0.9, 1.5, 0.0))
    assert (fw.dac, fw.steering, fw.rejected) == (100 - DECEL_STEP, pytest.approx(0.3), 1)


def test_reverse_follows_the_speeds_sign():
    fw = Esp32Firmware()
    fw.feed(frame(0.0, -0.5, 0.0))
    assert fw.outputs().reverse


def test_steering_undoes_the_bridges_mapping():
    fw = Esp32Firmware(max_steering_angle=0.5, steering_sign=-1.0)
    # The bridge, 0.25 rad left with sign -1: -0.5, then x the negative side's gain.
    fw.feed(frame(-0.5 * NEGATIVE_STEERING_GAIN, 0.0, 0.0))
    assert fw.outputs().steering == pytest.approx(0.25)
    fw.feed(frame(0.5, 0.0, 0.0))              # positive numbers: no gain
    assert fw.outputs().steering == pytest.approx(-0.25)


def test_no_frames_keeps_the_last_outputs_as_the_c_plus_plus_does():
    fw = Esp32Firmware()
    for _ in range(50):
        fw.feed(frame(0.0, 0.5, 0.0))
    before = fw.outputs()
    fw.feed(b'')
    assert fw.outputs() == before and before.throttle > 0.0

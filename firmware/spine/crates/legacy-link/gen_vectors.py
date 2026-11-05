#!/usr/bin/env python3
"""
Regenerate the UART link's test vectors, from the ROS side's own code.

The bytes come from ros/can_bridge/aav_legacy/aav_legacy/legacy_protocol.py (pack_frame,
to_legacy, stop_command): what the Motor-Pi sends today. The expected Jrk and DAC targets
come from a line-by-line model of the 2025-26 C++ (ros2_handler.cpp, jrk_g2.cpp), in
single-precision floats as the ESP32 computes them.

Version 2 (TODO 50): the bytes come from aav_legacy's uart_relay (what legacy_uart_bridge
sends with frame_version 2) and pack_frame_v2; the expected result is the frame's
counter and flags, or why it is turned down, and the same targets.

Version 3 (1 Nov): the bytes come from uart_relay (frame_version 3, with and without
the panel's calibration request) and pack_frame_v3; the expected result is the frame's
counter, flags and calibration request, or why it is turned down. ParserV3 must find the
same frames as v3_parse, in a stream of version 2 and 3 frames.

The uplink (TODO 50, 48), the other way: the bytes come from pack_uplink, which is what
legacy_uart_bridge reads (unpack_uplink); the Rust Uplink must encode the same fields
to the same bytes, and turn down the same bad frames.

Writes:
  firmware/spine/crates/legacy-link/vectors.json     (read by aav_legacy's tests)
  firmware/spine/crates/legacy-link/tests/vectors.rs (read by `cargo test`)

Run it after any change to legacy_protocol.py or the legacy-link crate:
  python3 firmware/spine/crates/legacy-link/gen_vectors.py
"""

import importlib.util
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PROTOCOL = ROOT / 'ros/can_bridge/aav_legacy/aav_legacy/legacy_protocol.py'
JSON_OUT = HERE / 'vectors.json'
RUST_OUT = HERE / 'tests/vectors.rs'

spec = importlib.util.spec_from_file_location('legacy_protocol', PROTOCOL)
lp = importlib.util.module_from_spec(spec)
sys.modules['legacy_protocol'] = lp  # dataclasses look the module up while loading
spec.loader.exec_module(lp)

# uart_relay imports aav_legacy.legacy_protocol: give it the module loaded above.
# It also imports aav_legacy.speed_control: the stand-in package finds it in that folder.
package = sys.modules.setdefault('aav_legacy', type(sys)('aav_legacy'))
package.__path__ = [str(PROTOCOL.parent)]
sys.modules['aav_legacy.legacy_protocol'] = lp
spec = importlib.util.spec_from_file_location('aav_legacy.uart_relay',
                                              PROTOCOL.parent / 'uart_relay.py')
relay_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay_module)


def f32(x):
    """Round to the nearest single-precision float, as the ESP32's float maths does."""
    return struct.unpack('<f', struct.pack('<f', x))[0]


def u16(x):
    """C's (uint16_t) cast of a non-negative float: drop the fraction."""
    return int(x)


def constrain(x, lo, hi):
    return max(lo, min(hi, x))


# hardware_configs.h
JRK_MIN, JRK_CENTER, JRK_MAX, DAC_MAX_CAP = 400, 1950, 3500, 700


def cpp_targets(steering, speed, accel):
    """Return what handleROS2Messages() sent to the devices, or None if it refused."""
    if not (abs(steering) <= 1.0 and abs(speed) <= 1.0 and abs(accel) <= 1.0):
        return None  # "packet rejected - out of range": writeDACRamped(0) only
    if steering <= 0.0:
        steer = u16(f32(JRK_CENTER + f32(steering * JRK_CENTER)))
    else:
        steer = u16(f32(JRK_CENTER + f32(steering * (JRK_MAX - JRK_CENTER))))
    steer = constrain(steer, JRK_MIN, JRK_MAX)  # setSteer()
    if accel < f32(-0.05):  # -0.05f: a float literal
        brake = constrain(u16(f32(abs(accel) * JRK_MAX)), JRK_MIN, JRK_MAX)
        return {'steering': steer, 'brake': brake, 'throttle': 'cut'}
    if abs(speed) < f32(0.03):  # 0.03f
        return {'steering': steer, 'brake': JRK_MIN, 'throttle': 'idle'}
    code = constrain(u16(f32(abs(speed) * DAC_MAX_CAP)), 0, DAC_MAX_CAP)
    return {'steering': steer, 'brake': JRK_MIN,
            'throttle': {'code': code, 'reverse': speed < 0}}


def raw_frame(steering, speed, accel):
    """Pack a frame without pack_frame's clamp: for values the C++ must turn down."""
    body = struct.pack('<fff', steering, speed, accel)
    return body + bytes([sum(body) & 0xFF])


M = lp.LegacyMapping()   # the defaults: 0.5 rad full lock, 3.0 m/s full throttle
M_CAR = lp.LegacyMapping(steering_sign=-1.0)  # legacy.yaml's sign
NAN = float('nan')

# (name, how the frame was made, the frame's bytes)
CASES = [
    ('stand_still', 'to_legacy(0.0, 0.0)', lp.pack_frame(*lp.to_legacy(0.0, 0.0, M))),
    ('stale_stop', 'stop_command()', lp.pack_frame(*lp.stop_command(M))),
    ('stale_stop_holding_steer', 'stop_command(steering_hold=0.4)',
     lp.pack_frame(*lp.stop_command(M, 0.4))),
    ('creep', 'to_legacy(0.06, 0.0)', lp.pack_frame(*lp.to_legacy(0.06, 0.0, M))),
    ('cruise_left', 'to_legacy(1.5, 0.25)', lp.pack_frame(*lp.to_legacy(1.5, 0.25, M))),
    ('cruise_right', 'to_legacy(1.5, -0.25)', lp.pack_frame(*lp.to_legacy(1.5, -0.25, M))),
    ('car_sign_left', 'to_legacy(2.0, 0.3), steering_sign=-1',
     lp.pack_frame(*lp.to_legacy(2.0, 0.3, M_CAR))),
    ('full_forward', 'to_legacy(5.0, 1.0)', lp.pack_frame(*lp.to_legacy(5.0, 1.0, M))),
    ('reverse', 'to_legacy(-0.9, -0.1)', lp.pack_frame(*lp.to_legacy(-0.9, -0.1, M))),
    ('full_reverse', 'to_legacy(-9.0, 0.0)', lp.pack_frame(*lp.to_legacy(-9.0, 0.0, M))),
    ('full_left_raw', 'pack_frame(1, 0.5, 0)', lp.pack_frame(1.0, 0.5, 0.0)),
    ('full_right_raw', 'pack_frame(-1, 0.5, 0)', lp.pack_frame(-1.0, 0.5, 0.0)),
    ('odd_steer_raw', 'pack_frame(0.123, -0.456, 0.0)', lp.pack_frame(0.123, -0.456, 0.0)),
    ('deadband_edge', 'pack_frame(0, 0.0299, 0)', lp.pack_frame(0.0, 0.0299, 0.0)),
    ('just_past_deadband', 'pack_frame(0, -0.0301, 0)', lp.pack_frame(0.0, -0.0301, 0.0)),
    ('brake_threshold', 'pack_frame(0, 0.5, -0.05)', lp.pack_frame(0.0, 0.5, -0.05)),
    ('light_brake', 'pack_frame(0, 0.5, -0.06)', lp.pack_frame(0.0, 0.5, -0.06)),
    ('brake_and_throttle', 'pack_frame(-0.3, 0.8, -0.7)', lp.pack_frame(-0.3, 0.8, -0.7)),
    ('clamped_by_pack', 'pack_frame(2, -3, 9)', lp.pack_frame(2.0, -3.0, 9.0)),
    ('out_of_range', 'raw (1.5, 0, 0)', raw_frame(1.5, 0.0, 0.0)),
    ('not_a_number', 'raw (0, NaN, 0)', raw_frame(0.0, NAN, 0.0)),
]


def cpp_parse(stream):
    """Split a stream as the C++'s byte loop did: 13 bytes, then the sum; a mismatch drops one."""
    buf, out = [], []
    for b in stream:
        buf.append(b)
        if len(buf) == 13:
            if sum(buf[:12]) & 0xFF == buf[12]:
                out.append(bytes(buf))
                buf = []
            else:
                buf = buf[1:]
    return out


# Noise, a frame, a frame's tail, two frames back to back: what a Pi rebooting mid-frame,
# or a loose wire, gives.
STREAM = (bytes([0x00, 0xFF, 0x13]) + lp.pack_frame(0.1, 0.2, 0.0)
          + lp.pack_frame(-0.5, 0.0, -1.0)[5:]
          + lp.pack_frame(0.0, 0.4, 0.0) + lp.pack_frame(0.2, -0.3, 0.0))
STREAM_FRAMES = cpp_parse(STREAM)


def relay_frame(counter, cmd=None, estop=False, now=1.0):
    """Return what legacy_uart_bridge sends (frame_version 2): ego's command, or none."""
    relay = relay_module.Relay(M, cmd_timeout=0.3, version=2)
    relay.heartbeat(counter)
    relay.set_estop(estop)
    if cmd is not None:
        relay.command(*cmd, now)
    return relay.frame(now)


def with_byte(frame, at, value, fix_crc=True):
    """Return a frame with one byte changed, its CRC made right again (or not)."""
    b = bytearray(frame)
    b[at] = value
    if fix_crc:
        b[16] = lp.crc8(bytes(b[1:16]))
    return bytes(b)


def raw_frame_v2(steering, speed, accel, counter):
    """pack_frame_v2 without its clamp: for values the firmware must turn down."""
    body = bytes([2]) + struct.pack('<fff', steering, speed, accel) + bytes([counter, 1])
    return bytes([0xA5]) + body + bytes([lp.crc8(body)])


GOOD = lp.pack_frame_v2(0.2, 0.3, 0.0, 9, True, False)

# (name, how the frame was made, the frame's bytes, the error the firmware gives or None)
V2_CASES = [
    ('v2_drive', 'relay: heartbeat 7, to_legacy(1.5, 0.25)', relay_frame(7, (1.5, 0.25)), None),
    ('v2_stand_still', 'relay: heartbeat 8, to_legacy(0.0, 0.0)',
     relay_frame(8, (0.0, 0.0)), None),
    ('v2_stale_stop', 'relay: heartbeat 8, no command', relay_frame(8), None),
    ('v2_estop', 'relay: heartbeat 9, to_legacy(1.0, 0.1), e-stop on',
     relay_frame(9, (1.0, 0.1), estop=True), None),
    ('v2_counter_255', 'relay: heartbeat 255, to_legacy(0.6, 0.0)',
     relay_frame(255, (0.6, 0.0)), None),
    ('v2_counter_0', 'relay: heartbeat 0 (after 255), to_legacy(0.6, 0.0)',
     relay_frame(0, (0.6, 0.0)), None),
    ('v2_counter_is_the_sync_byte', 'relay: heartbeat 0xA5, to_legacy(-0.9, -0.1)',
     relay_frame(0xA5, (-0.9, -0.1)), None),
    ('v2_clamped', 'pack_frame_v2(2, -3, 9), counter 1, armed',
     lp.pack_frame_v2(2.0, -3.0, 9.0, 1, True, False), None),
    ('v2_out_of_range', 'raw (1.5, 0, 0), counter 3', raw_frame_v2(1.5, 0.0, 0.0, 3), None),
    ('v2_bad_crc', 'GOOD with its CRC flipped', with_byte(GOOD, 16, GOOD[16] ^ 0xFF, False),
     'Checksum'),
    ('v2_one_bit_flipped', 'GOOD with bit 0 of the speed flipped',
     with_byte(GOOD, 6, GOOD[6] ^ 0x01, False), 'Checksum'),
    ('v2_wrong_sync', 'GOOD starting 0x5A', with_byte(GOOD, 0, 0x5A, False), 'Sync'),
    ('v2_version_3', 'GOOD as version 3, its CRC right', with_byte(GOOD, 1, 3), 'Version'),
    ('v2_unknown_flag', 'GOOD with flag bit 2, its CRC right', with_byte(GOOD, 15, 0x05),
     'Flags'),
]


def v2_parse(stream):
    """
    Split a stream as legacy-link's ParserV2 does.

    Wait for 0xA5, take 17 bytes; a bad frame is dropped, and the search starts again at
    the next 0xA5 inside it.
    """
    buf, out = [], []
    for b in stream:
        if not buf and b != 0xA5:
            continue
        buf.append(b)
        if len(buf) == 17:
            try:
                lp.unpack_frame_v2(bytes(buf))
                out.append(bytes(buf))
                buf = []
            except ValueError:
                rest = buf[1:]
                buf = rest[rest.index(0xA5):] if 0xA5 in rest else []
    return out


# Noise, a frame, a frame's tail, a frame with a bad CRC, three good frames (one with the
# counter 0xA5): what a Pi rebooting mid-frame, or a loose wire, gives.
V2_STREAM = (bytes([0x00, 0xFF, 0x13]) + relay_frame(1, (1.0, 0.1))
             + relay_frame(2, (1.0, 0.1))[6:]
             + with_byte(relay_frame(3, (1.0, 0.1)), 16, 0x00, False)
             + relay_frame(4, (1.0, 0.1)) + relay_frame(0xA5, (1.0, 0.1))
             + relay_frame(0xA6, (0.0, 0.0)))
V2_STREAM_FRAMES = v2_parse(V2_STREAM)


def relay_v3(counter, cmd=None, cal=None, now=1.0):
    """Return what legacy_uart_bridge sends with frame_version 3, on stands, uplink fresh."""
    relay = relay_module.Relay(M, cmd_timeout=0.3, version=3, calibration='stands')
    relay.heartbeat(counter)
    relay.uplink_seen(now)
    if cmd is not None:
        relay.command(*cmd, now)
    if cal is not None:
        relay.calibrate(*cal, now)
    return relay.frame(now)


def with_byte_v3(frame, at, value):
    """Return a version 3 frame with one byte changed and its CRC made right again."""
    b = bytearray(frame)
    b[at] = value
    b[19] = lp.crc8(bytes(b[1:19]))
    return bytes(b)


GOOD_V3 = lp.pack_frame_v3(0.0, 0.0, -1.0, 9, False, False, lp.CAL_MODE_JOG, 2000)

# (name, how the frame was made, the frame's bytes, the error the firmware gives or None)
V3_CASES = [
    ('v3_drive', 'relay v3: heartbeat 7, to_legacy(1.5, 0.25)', relay_v3(7, (1.5, 0.25)), None),
    ('v3_limp', 'relay v3: heartbeat 8, the panel asks for limp',
     relay_v3(8, cal=(lp.CAL_MODE_LIMP, 0)), None),
    ('v3_jog', 'relay v3: heartbeat 9, the panel asks for a jog to 2300',
     relay_v3(9, cal=(lp.CAL_MODE_JOG, 2300)), None),
    ('v3_jog_clamped', 'pack_frame_v3 jog to 9999: clamped to 4095 by pack',
     lp.pack_frame_v3(0.0, 0.0, -1.0, 10, False, False, lp.CAL_MODE_JOG, 9999), None),
    ('v3_refused_while_driving', 'relay v3: ego drives (1.5 m/s), the panel asks for limp',
     relay_v3(11, (1.5, 0.0), cal=(lp.CAL_MODE_LIMP, 0)), None),
    ('v3_bad_crc', 'GOOD_V3 with its CRC flipped',
     bytes(GOOD_V3[:19]) + bytes([GOOD_V3[19] ^ 0xFF]), 'Checksum'),
    ('v3_mode_3', 'GOOD_V3 with mode 3, its CRC right', with_byte_v3(GOOD_V3, 16, 3), 'Field'),
    ('v3_limp_with_target', 'GOOD_V3 as limp, keeping its target, its CRC right',
     with_byte_v3(GOOD_V3, 16, 1), 'Field'),
    ('v3_target_4096', 'GOOD_V3 with target 4096, its CRC right',
     with_byte_v3(with_byte_v3(GOOD_V3, 17, 0x00), 18, 0x10), 'Field'),
    ('v3_unknown_flag', 'GOOD_V3 with flag bit 2, its CRC right',
     with_byte_v3(GOOD_V3, 15, 0x04), 'Flags'),
]


def v3_parse(stream):
    """
    Split a stream as legacy-link's ParserV3 does.

    Wait for 0xA5; the next byte gives the length (2: 17, 3: 20; anything else drops the
    two and starts again at the next 0xA5 in them); a bad frame is dropped, and the search
    starts again at the next 0xA5 inside it. A frame that only comes clear after a
    restart is found one byte later, as in the Rust.
    """
    buf, out = [], []
    for b in stream:
        if not buf and b != 0xA5:
            continue
        buf.append(b)
        if len(buf) < 2:
            continue
        want = {2: 17, 3: 20}.get(buf[1])
        if want is None:
            rest = buf[1:]
            buf = rest[rest.index(0xA5):] if 0xA5 in rest else []
            continue
        if len(buf) < want:
            continue
        try:
            frame = bytes(buf[:want])
            (lp.unpack_frame_v2 if want == 17 else lp.unpack_frame_v3)(frame)
            out.append(frame)
            buf = buf[want:]
        except ValueError:
            rest = buf[1:want]
            at = rest.index(0xA5) + 1 if 0xA5 in rest else want
            buf = buf[at:]
    return out


# Noise, a version 3 frame, a version 2 frame, a frame's tail, an unknown version, a v3
# frame with a bad CRC, then good frames of both versions.
V3_STREAM = (bytes([0x00, 0x13]) + relay_v3(1, cal=(lp.CAL_MODE_LIMP, 0))
             + relay_frame(2, (1.0, 0.1)) + relay_v3(3, cal=(lp.CAL_MODE_JOG, 1500))[7:]
             + bytes([0xA5, 0x09]) + with_byte(relay_v3(4), 19, 0x00, False)
             + relay_v3(5, cal=(lp.CAL_MODE_JOG, 1600)) + relay_frame(0xA5, (0.0, 0.0))
             + relay_v3(6))
V3_STREAM_FRAMES = v3_parse(V3_STREAM)


U = lp.Uplink
# Values exact in float32, so both sides read the same numbers.
UPLINK_CASES = [
    ('up_driving', 'computer drives, both positions known, no RC',
     U(sequence=7, echo=42, mode=lp.MODE_COMPUTER, switch=lp.SWITCH_NONE, armed=True,
       estop=False, faults=0, steering=0.125, brake_position=0.0, throttle_out=0.25,
       brake_out=0.0, rc_throttle=0.0, rc_steering=0.0, rc_brake=False,
       steering_counts=2338, brake_counts=400)),
    ('up_rc_via_ros', 'the switch on RC via ROS: the sticks go up, the computer drives',
     U(sequence=8, echo=43, mode=lp.MODE_COMPUTER, switch=lp.SWITCH_RC_VIA_ROS, armed=True,
       estop=False, faults=0, steering=-0.0625, brake_position=0.5, throttle_out=0.0,
       brake_out=0.5, rc_throttle=0.5, rc_steering=-0.75, rc_brake=False)),
    ('up_rc_direct', 'RC direct, reverse, steering unknown (no feedback on that Jrk)',
     U(sequence=255, echo=0xA5, mode=lp.MODE_RC_DIRECT, switch=lp.SWITCH_RC_DIRECT,
       armed=False, estop=False, faults=0, steering=None, brake_position=0.0,
       throttle_out=0.125, brake_out=0.0, rc_throttle=-0.25, rc_steering=1.0,
       rc_brake=False)),
    ('up_timeout', 'the heartbeat stopped: braked, latched, fault 1, the last echo',
     U(sequence=0, echo=99, mode=lp.MODE_STOPPED, switch=lp.SWITCH_COMPUTER, armed=False,
       estop=True, faults=0x0001, steering=0.0, brake_position=1.0, throttle_out=0.0,
       brake_out=1.0, rc_throttle=0.0, rc_steering=0.0, rc_brake=True,
       steering_counts=1950, brake_counts=3500, calibration=lp.CAL_HOLD)),
    ('up_limp', 'calibrating: the steering actuator limp, a person at the wheels',
     U(sequence=2, echo=17, mode=lp.MODE_STOPPED, switch=lp.SWITCH_COMPUTER, armed=False,
       estop=False, faults=0, steering=0.25, brake_position=1.0, throttle_out=0.0,
       brake_out=1.0, rc_throttle=0.0, rc_steering=0.0, rc_brake=False,
       steering_counts=2725, brake_counts=3500, calibration=lp.CAL_LIMP)),
    ('up_refused', 'calibration refused: no analog feedback on the steering actuator',
     U(sequence=3, echo=18, mode=lp.MODE_STOPPED, switch=lp.SWITCH_COMPUTER, armed=False,
       estop=False, faults=0, steering=None, brake_position=1.0, throttle_out=0.0,
       brake_out=1.0, rc_throttle=0.0, rc_steering=0.0, rc_brake=False,
       steering_counts=None, brake_counts=3500,
       calibration=lp.CAL_REFUSED_NO_FEEDBACK)),
    ('up_rc_lost', 'the RC lost via ROS: fault 256, no echo yet (version 1 frames)',
     U(sequence=1, echo=None, mode=lp.MODE_STOPPED, switch=lp.SWITCH_NONE, armed=False,
       estop=False, faults=0x0102, steering=None, brake_position=None, throttle_out=0.0,
       brake_out=1.0, rc_throttle=0.0, rc_steering=0.0, rc_brake=False)),
]
# Version 2 uplinks (before 1 Nov): still read, with no counts and calibration off.
UPLINK_V2_CASES = [(f'{n}_v2', f'{m}, as version 2', U(**{
    **vars(u), 'steering_counts': None, 'brake_counts': None, 'calibration': lp.CAL_OFF}))
    for n, m, u in UPLINK_CASES[:2]]
UP_GOOD = lp.pack_uplink(UPLINK_CASES[1][2])


def with_up_byte(frame, at, value, fix_crc=True):
    """Return an uplink frame with one byte changed, its CRC made right again (or not)."""
    b = bytearray(frame)
    b[at] = value
    if fix_crc:
        b[-1] = lp.crc8(bytes(b[1:-1]))
    return bytes(b)


# (name, how it was made, bytes, the error the Rust decoder gives)
UPLINK_BAD = [
    ('up_bad_crc', 'up_rc_via_ros with its CRC flipped',
     with_up_byte(UP_GOOD, 39, UP_GOOD[39] ^ 0xFF, False), 'Checksum'),
    ('up_one_bit_flipped', 'up_rc_via_ros with bit 0 of the faults flipped',
     with_up_byte(UP_GOOD, 8, UP_GOOD[8] ^ 0x01, False), 'Checksum'),
    ('up_wrong_sync', 'up_rc_via_ros starting 0x5A', with_up_byte(UP_GOOD, 0, 0x5A, False),
     'Sync'),
    ('up_kind_2', 'up_rc_via_ros as kind 2, its CRC right', with_up_byte(UP_GOOD, 2, 2),
     'Version'),
    ('up_mode_3', 'up_rc_via_ros with mode 3, its CRC right', with_up_byte(UP_GOOD, 5, 3),
     'Field'),
    ('up_switch_4', 'up_rc_via_ros with switch 4, its CRC right', with_up_byte(UP_GOOD, 6, 4),
     'Field'),
    ('up_unknown_flag_v2', 'up_rc_via_ros as version 2 with flag bit 6, its CRC right',
     with_up_byte(lp.pack_uplink(UPLINK_V2_CASES[1][2], version=2), 7, 0x40 | 0x1B), 'Flags'),
    ('up_cal_state_10', 'up_rc_via_ros with calibration state 10, its CRC right',
     with_up_byte(UP_GOOD, 38, 10), 'Field'),
    ('up_counts_4096', 'up_rc_via_ros with steering counts 4096, its CRC right',
     with_up_byte(with_up_byte(UP_GOOD, 34, 0x00), 35, 0x10), 'Field'),
    ('up_v3_cut_to_35', 'up_rc_via_ros, version 3 but 35 bytes long', UP_GOOD[:35], 'Version'),
]

RUST_MODE = {lp.MODE_STOPPED: 'Stopped', lp.MODE_RC_DIRECT: 'RcDirect',
             lp.MODE_COMPUTER: 'Computer'}
RUST_SWITCH = {lp.SWITCH_NONE: 'NoSignal', lp.SWITCH_COMPUTER: 'Computer',
               lp.SWITCH_RC_VIA_ROS: 'RcViaRos', lp.SWITCH_RC_DIRECT: 'RcDirect'}


def rust_f32(x):
    """Return a float32 as Rust, bit for bit."""
    return f"f32::from_bits(0x{struct.unpack('<I', struct.pack('<f', x))[0]:08x})"


def rust_opt(x, show):
    return 'None' if x is None else f'Some({show(x)})'


def rust_bool(b):
    return 'true' if b else 'false'


def rust_uplink(u):
    return (f'Uplink {{ sequence: {u.sequence}, echo: {rust_opt(u.echo, str)}, '
            f'mode: DriveMode::{RUST_MODE[u.mode]}, switch: ModeSwitch::{RUST_SWITCH[u.switch]}, '
            f'armed: {rust_bool(u.armed)}, estop: {rust_bool(u.estop)}, '
            f'faults: 0x{u.faults:04x}, steering: {rust_opt(u.steering, rust_f32)}, '
            f'brake_position: {rust_opt(u.brake_position, rust_f32)}, '
            f'throttle_out: {rust_f32(u.throttle_out)}, brake_out: {rust_f32(u.brake_out)}, '
            f'sticks: Sticks {{ throttle: {rust_f32(u.rc_throttle)}, '
            f'steering: {rust_f32(u.rc_steering)}, brake: {rust_bool(u.rc_brake)} }}, '
            f'counts: Counts {{ steering: {rust_opt(u.steering_counts, str)}, '
            f'brake: {rust_opt(u.brake_counts, str)} }}, '
            f'calibration: {RUST_CAL[u.calibration]} }}')


RUST_CAL = {
    lp.CAL_OFF: 'CalState::Off', lp.CAL_LIMP: 'CalState::Limp', lp.CAL_JOG: 'CalState::Jog',
    lp.CAL_HOLD: 'CalState::Hold', lp.CAL_RETURN: 'CalState::Return',
    lp.CAL_REFUSED_MOVING: 'CalState::Refused(Refusal::Moving)',
    lp.CAL_REFUSED_NOT_BRAKED: 'CalState::Refused(Refusal::NotBraked)',
    lp.CAL_REFUSED_NO_FEEDBACK: 'CalState::Refused(Refusal::NoFeedback)',
    lp.CAL_REFUSED_ESTOP: 'CalState::Refused(Refusal::Estop)',
    lp.CAL_REFUSED_OTHER_DRIVER: 'CalState::Refused(Refusal::OtherDriver)',
}
RUST_CAL_REQUEST = {lp.CAL_MODE_OFF: 'CalRequest::Off', lp.CAL_MODE_LIMP: 'CalRequest::Limp'}


def rust_cal_request(mode, target):
    return RUST_CAL_REQUEST.get(mode) or f'CalRequest::Jog({target})'


def rust_uplink_tests():
    out = []
    for name, made_by, u in UPLINK_CASES:
        out += ['#[test]', f'fn {name}() {{', f'    // {made_by}',
                f'    let bytes = [{hexes(lp.pack_uplink(u))}];',
                f'    let expected = {rust_uplink(u)};',
                '    assert_eq!(Uplink::decode(&bytes), Ok(expected));',
                '    assert_eq!(expected.encode(), bytes);', '}', '']
    for name, made_by, u in UPLINK_V2_CASES:
        out += ['#[test]', f'fn {name}() {{', f'    // {made_by}',
                f'    let bytes = [{hexes(lp.pack_uplink(u, version=2))}];',
                f'    let expected = {rust_uplink(u)};',
                '    assert_eq!(Uplink::decode(&bytes), Ok(expected));', '}', '']
    for name, made_by, frame, error in UPLINK_BAD:
        out += ['#[test]', f'fn {name}() {{', f'    // {made_by}',
                f'    let bytes = [{hexes(frame)}];',
                f'    assert_eq!(Uplink::decode(&bytes), Err(FrameError::{error}));', '}', '']
    return out


def rust_throttle(t):
    if t == 'cut':
        return 'Throttle::Cut'
    if t == 'idle':
        return 'Throttle::Idle'
    rev = 'true' if t['reverse'] else 'false'
    return f"Throttle::Drive {{ code: {t['code']}, reverse: {rev} }}"


def hexes(bs):
    return ', '.join(f'0x{b:02x}' for b in bs)


def rust_v3_tests(v3):
    out = []
    for c in v3:
        out += ['#[test]', f"fn {c['name']}() {{", f"    // {c['made_by']}",
                f"    let bytes = [{hexes(c['bytes'])}];"]
        if c['error']:
            out += [f"    assert_eq!(FrameV3::decode(&bytes), Err(FrameError::{c['error']}));",
                    '}', '']
            continue
        out += ['    let frame = FrameV3::decode(&bytes).expect("a valid frame");',
                f"    assert_eq!(frame.counter, {c['counter']});",
                f"    assert_eq!(frame.flags, Flags {{ armed: {rust_bool(c['armed'])}, "
                f"estop: {rust_bool(c['estop'])} }});",
                '    assert_eq!(frame.calibration, '
                f"{rust_cal_request(c['cal_mode'], c['jog_target'])});",
                '    assert_eq!(frame.encode(), bytes);',
                '    assert_eq!(',
                '        frame.values.check().expect("in range").targets(),',
                f"        Targets {{ steering: {c['cpp']['steering']}, "
                f"brake: {c['cpp']['brake']}, "
                f"throttle: {rust_throttle(c['cpp']['throttle'])} }}",
                '    );', '}', '']
    out += ['#[test]', 'fn v3_a_stream_of_both_versions_with_noise() {',
            f'    let stream = [{hexes(V3_STREAM)}];',
            '    let mut parser = ParserV3::new();',
            '    let found: Vec<(u8, FrameV3)> = stream',
            '        .iter()',
            '        .filter_map(|&b| parser.push(b))',
            '        .filter_map(Result::ok)',
            '        .map(|f| (f.counter, f))',
            '        .collect();',
            '    let expected: Vec<(u8, FrameV3)> = vec![']
    for f in V3_STREAM_FRAMES:
        if len(f) == 17:
            out.append(f'        ({f[14]}, FrameV2::decode(&[{hexes(f)}]).unwrap().into()),')
        else:
            out.append(f'        ({f[14]}, FrameV3::decode(&[{hexes(f)}]).unwrap()),')
    out += ['    ];', '    assert_eq!(found, expected);', '}', '']
    return out


def rust_tests(cases, v2, v3):
    out = ['// Generated by gen_vectors.py from legacy_protocol.py and a model of the',
           "// 2025-26 C++. Do not edit. Checks that this crate reads the Motor-Pi's",
           '// bytes, and sends the devices the targets, as the C++ did.',
           '',
           ('use legacy_link::{CalRequest, CalState, Counts, DriveMode, Flags, Frame, FrameError, '
            'FrameV2, FrameV3, ModeSwitch, Parser, ParserV2, ParserV3, Refusal, Sticks, Targets, '
            'Throttle, Uplink};'),
           '']
    for c in cases:
        out += ['#[test]', f"fn {c['name']}() {{",
                f"    // {c['made_by']}",
                f"    let bytes = [{hexes(c['bytes'])}];",
                '    let frame = Frame::decode(&bytes).expect("checksum");']
        t = c['cpp']
        if t is None:
            out.append('    assert_eq!(frame.check(), Err(FrameError::OutOfRange));')
        else:
            out += ['    let frame = frame.check().expect("in range");',
                    '    assert_eq!(',
                    '        frame.targets(),',
                    f"        Targets {{ steering: {t['steering']}, brake: {t['brake']}, "
                    f"throttle: {rust_throttle(t['throttle'])} }}",
                    '    );',
                    '    assert_eq!(frame.encode(), bytes);']
        out += ['}', '']
    out += ['#[test]', 'fn a_stream_with_noise() {',
            f'    let stream = [{hexes(STREAM)}];',
            '    let mut parser = Parser::new();',
            '    let found: Vec<[u8; 13]> = stream',
            '        .iter()',
            '        .filter_map(|&b| parser.push(b))',
            '        .filter_map(Result::ok)',
            '        .map(|f| f.encode())',
            '        .collect();',
            '    let expected: Vec<[u8; 13]> = vec![']
    out += [f'        [{hexes(f)}],' for f in STREAM_FRAMES]
    out += ['    ];', '    assert_eq!(found, expected);', '}', '']
    for c in v2:
        out += ['#[test]', f"fn {c['name']}() {{", f"    // {c['made_by']}",
                f"    let bytes = [{hexes(c['bytes'])}];"]
        if c['error']:
            out += [f"    assert_eq!(FrameV2::decode(&bytes), Err(FrameError::{c['error']}));",
                    '}', '']
            continue
        armed = 'true' if c['armed'] else 'false'
        estop = 'true' if c['estop'] else 'false'
        out += ['    let frame = FrameV2::decode(&bytes).expect("a valid frame");',
                f"    assert_eq!(frame.counter, {c['counter']});",
                f'    assert_eq!(frame.flags, Flags {{ armed: {armed}, estop: {estop} }});',
                '    assert_eq!(frame.encode(), bytes);']
        t = c['cpp']
        if t is None:
            out.append('    assert_eq!(frame.values.check(), Err(FrameError::OutOfRange));')
        else:
            out += ['    assert_eq!(',
                    '        frame.values.check().expect("in range").targets(),',
                    f"        Targets {{ steering: {t['steering']}, brake: {t['brake']}, "
                    f"throttle: {rust_throttle(t['throttle'])} }}",
                    '    );']
        out += ['}', '']
    out += ['#[test]', 'fn v2_a_stream_with_noise() {',
            f'    let stream = [{hexes(V2_STREAM)}];',
            '    let mut parser = ParserV2::new();',
            '    let found: Vec<[u8; 17]> = stream',
            '        .iter()',
            '        .filter_map(|&b| parser.push(b))',
            '        .filter_map(Result::ok)',
            '        .map(|f| f.encode())',
            '        .collect();',
            '    let expected: Vec<[u8; 17]> = vec![']
    out += [f'        [{hexes(f)}],' for f in V2_STREAM_FRAMES]
    out += ['    ];', '    assert_eq!(found, expected);', '}', '']
    out += rust_v3_tests(v3)
    out += rust_uplink_tests()
    return '\n'.join(out)


def main():
    cases = []
    for name, made_by, frame in CASES:
        steering, speed, accel = struct.unpack('<fff', frame[:12])
        cases.append({'name': name, 'made_by': made_by, 'bytes': list(frame),
                      'values': [steering, speed, accel],
                      'cpp': cpp_targets(steering, speed, accel)})
    v2 = []
    for name, made_by, frame, error in V2_CASES:
        case = {'name': name, 'made_by': made_by, 'bytes': list(frame), 'error': error}
        if error is None:
            steering, speed, accel = struct.unpack('<fff', frame[2:14])
            case.update(counter=frame[14], armed=bool(frame[15] & 1),
                        estop=bool(frame[15] & 2), cpp=cpp_targets(steering, speed, accel))
        v2.append(case)
    v3 = []
    for name, made_by, frame, error in V3_CASES:
        case = {'name': name, 'made_by': made_by, 'bytes': list(frame), 'error': error}
        if error is None:
            f = lp.unpack_frame_v3(frame)
            case.update(counter=f.counter, armed=f.armed, estop=f.estop, cal_mode=f.cal_mode,
                        jog_target=f.jog_target,
                        cpp=cpp_targets(f.steering, f.speed, f.accel))
        v3.append(case)
    doc = {'note': 'Generated by gen_vectors.py. bytes: what the Motor-Pi sends; '
                   'cpp: the Jrk and DAC targets the 2025-26 ESP32 firmware set for them '
                   '(null: it turned the frame down).',
           'cases': [{k: v for k, v in c.items() if k != 'values'} for c in cases],
           'stream': list(STREAM),
           'stream_frames': [list(f) for f in STREAM_FRAMES],
           'v2_note': 'Version 2 (TODO 50): bytes, what legacy_uart_bridge sends with '
                      'frame_version 2; error, why the firmware turns it down (null: it '
                      'takes it); counter and flags as it reads them; cpp, the targets.',
           'v2_cases': v2,
           'v2_stream': list(V2_STREAM),
           'v2_stream_frames': [list(f) for f in V2_STREAM_FRAMES],
           'v3_note': 'Version 3 (1 Nov): bytes, what legacy_uart_bridge sends with '
                      'frame_version 3; error, why the firmware turns it down (null: it '
                      'takes it); counter, flags, cal_mode and jog_target as it reads '
                      'them; cpp, the targets of its values. v3_stream mixes versions 2 '
                      'and 3; v3_stream_frames, the frames ParserV3 finds.',
           'v3_cases': v3,
           'v3_stream': list(V3_STREAM),
           'v3_stream_frames': [list(f) for f in V3_STREAM_FRAMES],
           'uplink_note': 'The uplink (TODO 50, 48), ESP32 to Motor-Pi: bytes, what the '
                          'ESP32 sends (pack_uplink); fields, what legacy_uart_bridge reads '
                          '(null: not known); error, why a bad frame is turned down.',
           'uplink_cases': [{'name': n, 'made_by': m, 'bytes': list(lp.pack_uplink(u)),
                             'fields': vars(u), 'error': None}
                            for n, m, u in UPLINK_CASES]
           + [{'name': n, 'made_by': m, 'bytes': list(lp.pack_uplink(u, version=2)),
               'fields': vars(u), 'error': None} for n, m, u in UPLINK_V2_CASES]
           + [{'name': n, 'made_by': m, 'bytes': list(f), 'fields': None, 'error': e}
              for n, m, f, e in UPLINK_BAD]}
    JSON_OUT.write_text(json.dumps(doc, indent=1) + '\n')
    RUST_OUT.parent.mkdir(exist_ok=True)
    RUST_OUT.write_text(rust_tests(cases, v2, v3))
    if shutil.which('rustfmt'):
        subprocess.run(['rustfmt', '--edition', '2021', str(RUST_OUT)], check=True)
    print(f'wrote {len(cases)} cases, {len(v2)} version 2 cases, {len(v3)} version 3 cases, '
          f'three streams and '
          f'{len(UPLINK_CASES) + len(UPLINK_V2_CASES) + len(UPLINK_BAD)} uplink cases '
          f'to {JSON_OUT.relative_to(ROOT)} and {RUST_OUT.relative_to(ROOT)}')


if __name__ == '__main__':
    main()

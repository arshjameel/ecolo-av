"""
The 2025-26 car's protocols, so the new stack can drive today's hardware.

No ROS imports. Sources: the 2025-26 team's repos, kept read-only in archive/
(docs/older-code.md says what came from each; on GitHub in AAV-2025-2026). The
branches and lines cited below are the repos' own:

UART to the ESP32 (AAV_cortex, branch 6-pico2-aav-firmware,
AAV_uMod_Firmware/src/modules/ros2_handler.cpp:17-80):
  13 bytes, no header: three little-endian float32 (steering, speed, accel),
  then checksum = sum(bytes 0..11) & 0xFF. Each value must be within -1..1.
  - steering: -1..1 of full lock each way (Jrk targets 400..1950..3500). The
    gain differs by side: 1950 counts per unit below 0 (the Jrk clips at 400,
    i.e. -0.795) and 1550 above. to_legacy() evens it out.
  - speed: |v| < 0.03 ramps the throttle to 0; otherwise the sign sets the
    direction and throttle DAC = |v| * 700, ramped per frame. Open loop.
  - accel: below -0.05 applies the brake, clamp(|a| * 3500, 400, 3500), and
    cuts the throttle at once. Otherwise the brake is released.
  The firmware sends nothing back and has NO command timeout: if frames stop,
  the last throttle stays on. See docs/firmware.md before driving.

Version 2 of the UART frame (TODO 50, for the Rust ESP32 firmware, legacy-link's
FrameV2): 17 bytes,
  0xA5 (sync), 2 (version), the same three float32, the Orin's counter (u8),
  flags (bit 0 armed, bit 1 e-stop), then CRC-8/SAE-J1850 of bytes 1..15:
  polynomial 0x1D, start 0xFF, final XOR 0xFF, as every frame in firmware/can/aav.dbc.
  The counter is the Orin's heartbeat: the output mux adds one for each command it
  passes on (/control/car/heartbeat); the Motor Pi only copies it. The ESP32 brakes
  when it hasn't changed for 100 ms, however many frames still arrive.
  Both sides check firmware/spine/crates/legacy-link/vectors.json.

The uplink (TODO 50, 48; legacy-link's Uplink): ESP32 -> Motor Pi on the ESP32's TX1
(GPIO17), 50 Hz, only with the firmware's `uplink` feature (the wire is the hardware
lead's to confirm). 35 bytes:
  0xA5, 2 (version), 1 (kind: status), sequence, echo (the last Orin counter the ESP32
  took), mode (0 stopped, 1 RC direct, 2 computer: VehicleStatus.mode's numbers),
  switch (the transmitter's mode switch: 0 no signal, 1 computer, 2 RC via ROS,
  3 RC direct), flags (bit 0 armed, 1 e-stop active, 2 echo valid, 3 steering known,
  4 brake position known, 5 gear on brake), faults (u16 LE), six float32 (steering rad,
  brake position 0..1, throttle out, brake out, RC throttle -1..1, RC steering -1..1),
  then CRC-8/SAE-J1850 of bytes 1..33. Unknown positions are NaN, their flag clear.

UDP from the wheel encoder (Wheel_Encoders, wiznet.ino:9-19,110-131):
  a 4-byte big-endian int32: signed spoke count since the previous packet,
  about every 100 ms, to port 10111. 8 spokes per wheel revolution. It sends
  only to the Motor Pi's address, 192.168.1.193 (wiznet.ino:19). Its Rust
  version (firmware/spine/boards/wheel-encoder-pico2) sends the same bytes:
  both sides check firmware/udp/wheel_encoder_vectors.txt.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import struct
from typing import Optional, Tuple

FRAME_LEN = 13
FRAME_V2_LEN = 17
SYNC = 0xA5
VERSION = 2
FLAG_ARMED = 0x01
FLAG_ESTOP = 0x02

# Scale negative steering by this, so both sides get the firmware's 1550 counts per unit.
NEGATIVE_STEERING_GAIN = 1550 / 1950
# The brake Jrk's targets: released and full (the firmware's JRK_MIN, JRK_MAX). The firmware
# sets clamp(|accel| x 3500, 400, 3500), so |accel| below 400/3500 still means released.
BRAKE_RELEASED = 400
BRAKE_FULL = 3500
# The firmware ignores a speed number below this (it ramps the throttle to 0).
THROTTLE_MIN = 0.03


@dataclass
class LegacyMapping:
    max_steering_angle: float = 0.5   # rad at full lock (TODO: measure)
    steering_sign: float = 1.0        # -1 if +1 steers right on this car (TODO: check)
    full_throttle_speed: float = 3.0  # m/s the car reaches at speed = 1.0 (TODO: measure)
    stop_speed: float = 0.05          # m/s below which ego's command means stand still
    hold_brake: float = 0.3           # brake (0..1) while standing still
    stop_brake: float = 1.0           # brake (0..1) when commands go stale


def clamp1(x: float) -> float:
    return max(-1.0, min(1.0, x))


def steering_value(steering: float, m: LegacyMapping) -> float:
    """Return the frame's steering number, -1..1, for an angle in rad (+ left)."""
    s = clamp1(m.steering_sign * steering / m.max_steering_angle)
    return s * NEGATIVE_STEERING_GAIN if s < 0 else s


def to_legacy(speed: float, steering: float, m: LegacyMapping) -> Tuple[float, float, float]:
    """Return (steering, speed, accel), each -1..1, for an ego command in m/s and rad."""
    s = steering_value(steering, m)
    if abs(speed) < m.stop_speed:
        return s, 0.0, -abs(m.hold_brake)
    v = clamp1(speed / m.full_throttle_speed)
    if abs(v) < 0.031:            # the firmware ignores |v| < 0.03; creep instead
        v = math.copysign(0.031, v)
    return s, v, 0.0


def from_pedals(steering: float, throttle: float, brake: float,
                m: LegacyMapping) -> Tuple[float, float, float]:
    """
    Return (steering, speed, accel) for a speed loop's pedals (speed_control), forward only.

    throttle (0..1) is the speed number itself: the firmware's DAC is that x its cap, open
    loop, so the loop on the Orin sets the throttle directly. Below THROTTLE_MIN the
    firmware would ignore it: 0. brake (0..1) spreads over the brake Jrk's travel, so any
    brake at all lands past the firmware's -0.05 threshold and its 400 floor.
    """
    s = steering_value(steering, m)
    if brake > 0.0:
        travel = BRAKE_RELEASED + min(brake, 1.0) * (BRAKE_FULL - BRAKE_RELEASED)
        return s, 0.0, -travel / BRAKE_FULL
    return s, (throttle if throttle >= THROTTLE_MIN else 0.0), 0.0


def stop_command(m: LegacyMapping, steering_hold: float = 0.0) -> Tuple[float, float, float]:
    """Return the frame values for a stale-command stop: no throttle, brake on."""
    return clamp1(steering_hold), 0.0, -abs(m.stop_brake)


def pack_frame(steering: float, speed: float, accel: float) -> bytes:
    body = struct.pack('<fff', clamp1(steering), clamp1(speed), clamp1(accel))
    return body + bytes([sum(body) & 0xFF])


def unpack_frame(frame: bytes) -> Tuple[float, float, float]:
    """Decode a frame (for tests and bench tools). Raises ValueError if invalid."""
    if len(frame) != FRAME_LEN or sum(frame[:12]) & 0xFF != frame[12]:
        raise ValueError('bad legacy frame')
    return struct.unpack('<fff', frame[:12])


@dataclass(frozen=True)
class FrameV2:
    """One version 2 frame's contents."""

    steering: float
    speed: float
    accel: float
    counter: int     # the Orin's heartbeat, 0..255
    armed: bool      # the Motor Pi has a fresh command and no e-stop
    estop: bool      # /safety/estop is on


def crc8(data: bytes) -> int:
    """Return the CRC-8/SAE-J1850 of `data` (0x4B for b'123456789')."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def pack_frame_v2(steering: float, speed: float, accel: float, counter: int,
                  armed: bool, estop: bool) -> bytes:
    """Return a version 2 frame: the values clamped to -1..1, the counter as it came."""
    flags = (FLAG_ARMED if armed else 0) | (FLAG_ESTOP if estop else 0)
    body = (bytes([VERSION])
            + struct.pack('<fff', clamp1(steering), clamp1(speed), clamp1(accel))
            + bytes([counter & 0xFF, flags]))
    return bytes([SYNC]) + body + bytes([crc8(body)])


def unpack_frame_v2(frame: bytes) -> FrameV2:
    """Decode a version 2 frame (for tests and bench tools). Raises ValueError if invalid."""
    if len(frame) != FRAME_V2_LEN or frame[0] != SYNC:
        raise ValueError('not a version 2 frame')
    if crc8(frame[1:16]) != frame[16]:
        raise ValueError('bad CRC')
    if frame[1] != VERSION or frame[15] & ~(FLAG_ARMED | FLAG_ESTOP):
        raise ValueError('unknown version or flags')
    steering, speed, accel = struct.unpack('<fff', frame[2:14])
    return FrameV2(steering, speed, accel, frame[14], bool(frame[15] & FLAG_ARMED),
                   bool(frame[15] & FLAG_ESTOP))


# --- The uplink: the ESP32's status ---

UPLINK_LEN = 35
KIND_STATUS = 1
# Byte 5: who drives, as VehicleStatus.mode.
MODE_STOPPED, MODE_RC_DIRECT, MODE_COMPUTER = 0, 1, 2
# Byte 6: the transmitter's mode switch.
SWITCH_NONE, SWITCH_COMPUTER, SWITCH_RC_VIA_ROS, SWITCH_RC_DIRECT = 0, 1, 2, 3
# Byte 7.
UP_ARMED, UP_ESTOP, UP_ECHO, UP_STEERING, UP_BRAKE, UP_STICK_BRAKE = (1 << i for i in range(6))
UP_FLAGS = 0x3F


@dataclass(frozen=True)
class Uplink:
    """One uplink status frame. None: not known (no echo yet, no position)."""

    sequence: int
    echo: Optional[int]
    mode: int                       # MODE_*
    switch: int                     # SWITCH_*
    armed: bool
    estop: bool
    faults: int
    steering: Optional[float]       # rad, + left, the steering Jrk's position
    brake_position: Optional[float]  # 0..1, the brake Jrk's position
    throttle_out: float             # 0..1, what the ESP32 drives the throttle with
    brake_out: float                # 0..1, and the brake
    rc_throttle: float              # -1..1, + forward (the gear switch gives the sign)
    rc_steering: float              # -1..1, + left
    rc_brake: bool                  # the gear switch on brake


def pack_uplink(u: Uplink) -> bytes:
    """Return the 35 bytes the ESP32 sends for `u` (for tests and the bench)."""
    flags = ((UP_ARMED if u.armed else 0) | (UP_ESTOP if u.estop else 0)
             | (UP_ECHO if u.echo is not None else 0)
             | (UP_STEERING if u.steering is not None else 0)
             | (UP_BRAKE if u.brake_position is not None else 0)
             | (UP_STICK_BRAKE if u.rc_brake else 0))
    nan = float('nan')
    body = (bytes([VERSION, KIND_STATUS, u.sequence & 0xFF,
                   (u.echo or 0) & 0xFF, u.mode, u.switch, flags])
            + struct.pack('<H6f', u.faults & 0xFFFF,
                          nan if u.steering is None else u.steering,
                          nan if u.brake_position is None else u.brake_position,
                          u.throttle_out, u.brake_out, u.rc_throttle, u.rc_steering))
    return bytes([SYNC]) + body + bytes([crc8(body)])


def unpack_uplink(frame: bytes) -> Uplink:
    """Decode an uplink frame. Raises ValueError if it isn't a good one."""
    if len(frame) != UPLINK_LEN or frame[0] != SYNC:
        raise ValueError('not an uplink frame')
    if crc8(frame[1:34]) != frame[34]:
        raise ValueError('bad CRC')
    if frame[1] != VERSION or frame[2] != KIND_STATUS:
        raise ValueError('unknown version or kind')
    sequence, echo, mode, switch, flags = frame[3:8]
    if flags & ~UP_FLAGS or mode > MODE_COMPUTER or switch > SWITCH_RC_DIRECT:
        raise ValueError('unknown flags, mode or switch')
    faults, steering, brake, throttle_out, brake_out, rc_t, rc_s = struct.unpack(
        '<H6f', frame[8:34])
    return Uplink(sequence, echo if flags & UP_ECHO else None, mode, switch,
                  bool(flags & UP_ARMED), bool(flags & UP_ESTOP), faults,
                  steering if flags & UP_STEERING else None,
                  brake if flags & UP_BRAKE else None,
                  throttle_out, brake_out, rc_t, rc_s, bool(flags & UP_STICK_BRAKE))


class UplinkParser:
    """
    Bytes in, uplink frames out: as legacy-link's ParserV2 finds version 2 frames.

    Wait for 0xA5, take 35 bytes; a bad frame is dropped (and counted in `bad`), and
    the search starts again at the next 0xA5 inside it. Bytes outside a frame count
    in `skipped`.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0
        self.skipped = 0

    def feed(self, data: bytes) -> list:
        """Return the good frames in `data` (with what came before), oldest first."""
        out = []
        for b in data:
            if not self.buf and b != SYNC:
                self.skipped += 1
                continue
            self.buf.append(b)
            if len(self.buf) < UPLINK_LEN:
                continue
            try:
                out.append(unpack_uplink(bytes(self.buf)))
                self.buf.clear()
            except ValueError:
                self.bad += 1
                rest = self.buf[1:]
                at = rest.find(SYNC)
                self.buf = rest[at:] if at >= 0 else bytearray()
        return out


def encoder_counts(packet: bytes) -> int:
    """Decode one wheel-encoder UDP packet: a big-endian int32 spoke count."""
    if len(packet) != 4:
        raise ValueError(f'expected 4 bytes, got {len(packet)}')
    return struct.unpack('>i', packet)[0]


def metres_per_count(wheel_diameter: float, counts_per_rev: int) -> float:
    return math.pi * wheel_diameter / counts_per_rev


class EncoderSpeed:
    """
    Wheel speed from encoder packets, averaged over `window` seconds.

    speed() returns None, not 0 m/s, when packets stop for longer than `timeout`,
    and until a full window has arrived since they came back. A parked car still
    sends packets, with a count of 0.
    """

    def __init__(self, metres_per_count: float, window: float = 0.5, timeout: float = 0.3):
        self.mpc = metres_per_count
        self.window = window
        self.timeout = timeout
        self.samples = deque()   # (time in s, counts)
        self.last = None         # when the latest packet arrived
        self.since = None        # when packets last started arriving

    def add(self, t: float, counts: int) -> None:
        if self.last is None or t - self.last > self.timeout:
            self.samples.clear()
            self.since = t
        self.last = t
        self.samples.append((t, counts))

    def speed(self, t: float) -> Optional[float]:
        if self.last is None or t - self.last > self.timeout or t - self.since < self.window:
            return None
        while self.samples and self.samples[0][0] < t - self.window:
            self.samples.popleft()
        return sum(c for _, c in self.samples) * self.mpc / self.window

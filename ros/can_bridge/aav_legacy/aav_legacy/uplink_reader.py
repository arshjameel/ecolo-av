"""
What legacy_uart_bridge reads back from the ESP32: its uplink frames (TODO 50, 48). No ROS.

  ESP32 TX1 ──35-byte frames, 50 Hz──> serial port ──> UplinkReader.feed(bytes, now)
                                                          │
           status_fields / steering_state / brake_state / rc_fields <── the latest frame
           raw_fields (version 3: the counts), calibration_line

Only with the bridge's `uplink: true` and the firmware built with `--features uplink`:
the wire from the ESP32's GPIO17 to the Pi's RX is the hardware lead's to confirm.
The frame's layout is in legacy_protocol.py (pack_uplink, unpack_uplink).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from aav_legacy.legacy_protocol import CAL_LIMP, CAL_OFF, CAL_STATES, Uplink, UplinkParser

NAN = float('nan')


class UplinkReader:
    """The latest uplink frame, when it came, and the counts since the start."""

    def __init__(self, timeout: float = 0.2):
        self.timeout = timeout
        self.parser = UplinkParser()
        self.latest: Optional[Uplink] = None
        self.at: Optional[float] = None
        self.frames = 0
        self.lost = 0           # gaps in the sequence numbers: frames that never came

    def feed(self, data: bytes, now: float) -> List[Uplink]:
        """Take the bytes read at `now` (s); return the frames they completed."""
        found = self.parser.feed(data)
        for u in found:
            if self.latest is not None:
                self.lost += (u.sequence - self.latest.sequence - 1) & 0xFF
            self.latest = u
            self.at = now
            self.frames += 1
        return found

    def fresh(self, now: float) -> bool:
        """Return True if a frame came within the timeout."""
        return self.at is not None and now - self.at <= self.timeout

    @property
    def bad(self) -> int:
        """Frames turned down: a CRC, a version, a field."""
        return self.parser.bad


def known(x: Optional[float]) -> float:
    """Return x, or NaN when it isn't known."""
    return NAN if x is None else x


def status_fields(u: Uplink) -> dict:
    """
    Return aav_interfaces/VehicleStatus's fields for a frame.

    The uplink's mode numbers are VehicleStatus's (0 manual/stopped, 1 RC, 2 computer).
    That ESP32 has no wheel speed: speed is NaN.
    """
    return {'mode': u.mode, 'armed': u.armed, 'estop_active': u.estop, 'speed': NAN,
            'steering_angle': known(u.steering), 'throttle': u.throttle_out,
            'brake': u.brake_out, 'faults': u.faults}


def steering_state(u: Uplink, command: Optional[float]) -> Tuple[float, float]:
    """Return /vehicle/steering's (command, measured): ego's angle, the Jrk's (rad)."""
    return known(command), known(u.steering)


def brake_state(u: Uplink) -> Tuple[float, float]:
    """Return /vehicle/brake's (command, measured): what the ESP32 drives, the Jrk's."""
    return u.brake_out, known(u.brake_position)


def rc_fields(u: Uplink) -> dict:
    """Return aav_interfaces/RcSticks's fields for a frame: the same numbers."""
    return {'mode_switch': u.switch, 'throttle': u.rc_throttle, 'steering': u.rc_steering,
            'brake': u.rc_brake}


def raw_fields(counts: Optional[int], calibration: int = CAL_OFF) -> dict:
    """Return aav_interfaces/ActuatorRaw's fields: counts (None: not known), the state."""
    return {'valid': counts is not None, 'position': counts or 0, 'calibration': calibration}


def calibration_line(wanted: bool, refusal: Optional[str], u: Optional[Uplink]) -> str:
    """
    Return /vehicle/steering_actuator/calibration's line for people.

    `wanted`: the panel asks for limp or jog; `refusal`: why the bridge doesn't pass it
    on (None: it does); `u`: the latest uplink, for the firmware's own state.
    """
    if wanted and refusal:
        return f'REFUSED - {refusal}'
    state = CAL_OFF if u is None else u.calibration
    words = CAL_STATES.get(state, f'state {state}')
    if state == CAL_LIMP:
        words += " - the steering actuator's motor is off: turn the wheels by hand"
    if u is not None and u.steering_counts is not None:
        words += f' (at {u.steering_counts} counts)'
    return words

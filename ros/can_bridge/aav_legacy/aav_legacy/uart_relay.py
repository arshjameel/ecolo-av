"""
What legacy_uart_bridge writes on the serial port each tick. No ROS here.

  ego's command (/control/car/actuator_cmd) ─┐
  the Orin's counter (/control/car/heartbeat) ┼─> Relay.frame(now) ─> one frame, 50 Hz
  /safety/estop ──────────────────────────────┘

A fresh command and no e-stop: ego's values, armed. Otherwise the stale-stop values
(throttle 0, brake full, steering held), not armed. With a speed loop (speed_control, the
bridge's speed_loop.mode: pi), ego's speed goes through it first, closed on the wheel speed
(/vehicle/odom), and its throttle or brake fills the frame (legacy_protocol.from_pedals);
without one, to_legacy's open-loop mapping, as the 2025-26 team drove. Version 2 frames
carry the Orin's counter as it last came, and the e-stop flag. The relay never makes a
counter up: when the Orin goes quiet the counter stays where it was, however many frames
still go out, and the ESP32 brakes 100 ms later (TODO 50). Version 1 is the 2025-26 13-byte frame,
for the C++ firmware: no counter, no flags.

Version 3 (1 Nov) is version 2 and the steering actuator's calibration: a person on the
hardware panel asks for limp (its motor off, turned by hand) or a jog target in counts
(/drivers/buttons/steering_calibration, again at 10 Hz while it holds). `cal_refusal`
decides whether the relay passes it on (the rules below); while it does, every frame is
a stop: no speed, the brake full on, the steering held, not armed, with the mode. The
firmware checks the same again, and more (legacy-link's SteerCal).

  | Refused when | Why |
  |---|---|
  | `steering_calibration` is none (the default) | stands or bench, with the hardware lead |
  | the frame version isn't 3 | only the Rust firmware knows the mode |
  | no fresh uplink frame | the counts come back on it: nothing to calibrate with |
  | the e-stop is on | it stops everything |
  | ego asks for speed (a fresh command at or above stop_speed) | someone is driving |
  | the wheel speed (/vehicle/odom, when fresh) is 0.05 m/s or more | the car is moving |
  | the request is older than 0.5 s | the panel stopped asking: its person let go, or it died |

`programs_locked` decides the no-timeout lock (/safety/legacy_uart_bridge/lock): with
version 1 the firmware has no command timeout, so twist_mux holds every program below the
actuator tests, and only people (and the tests they run) drive.
"""

from __future__ import annotations

from typing import Optional, Tuple

from aav_legacy.legacy_protocol import (CAL_MODE_JOG, CAL_MODE_LIMP, CAL_MODE_OFF,
                                        from_pedals, LegacyMapping, pack_frame, pack_frame_v2,
                                        pack_frame_v3, stop_command, to_legacy)
from aav_legacy.speed_control import Pedals, SpeedControl

VERSIONS = (1, 2, 3)
# Which frame versions go to a firmware that brakes by itself when the frames or the
# Orin's counter stop: 2 and 3 (the Rust firmware, 100 ms). Version 1 is the 2025-26 C++.
WITH_TIMEOUT = (2, 3)
# Where the steering calibration may run: never (the default), or with the hardware
# lead's say-so on the stands (wheels off the ground) or the bench (the ESP32 on a desk).
CAL_WHERE = ('none', 'stands', 'bench')   # 'none', not 'off': YAML reads off as false
CAL_REQUEST_TIMEOUT = 0.5   # s: the panel asks again at 10 Hz while its person holds
STANDSTILL = 0.05           # m/s: a faster wheel speed is a moving car


def programs_locked(version: int, allow_programs_without_timeout: bool = False) -> bool:
    """
    Return True if twist_mux must hold the programs (scripted, replay, point, Nav2, AI).

    True for a frame version whose firmware has no command timeout (1, the C++): if the
    bridge or the Orin dies mid-drive, the last throttle stays on, so only a person, with
    the e-stop in reach, drives. `allow_programs_without_timeout` overrides it, for a
    supervised stand (wheels off the ground) only.
    """
    if version not in VERSIONS:
        raise ValueError(f'frame_version {version}: use 1 (the C++ firmware), 2 or 3')
    return version not in WITH_TIMEOUT and not allow_programs_without_timeout


class Relay:
    """The bridge's state: the last command, the Orin's counter, the e-stop."""

    def __init__(self, mapping: LegacyMapping, cmd_timeout: float, version: int = 2,
                 speed_loop: Optional[SpeedControl] = None, odom_timeout: float = 0.3,
                 calibration: str = 'none'):
        if version not in VERSIONS:
            raise ValueError(f'frame_version {version}: use 1 (the C++ firmware), 2 or 3')
        if calibration not in CAL_WHERE:
            raise ValueError(f'steering_calibration {calibration!r}: use none, stands or bench')
        self.cal_where = calibration
        self.cal_request: Optional[Tuple[int, int, float]] = None   # (mode, target, time)
        self.uplink_at: Optional[float] = None
        self.uplink_timeout = 0.2
        self.mapping = mapping
        self.timeout = cmd_timeout
        self.version = version
        self.cmd: Optional[Tuple[float, float]] = None   # (speed m/s, steering rad)
        self.cmd_time: Optional[float] = None
        self.counter = 0            # the Orin's, as it last came; 0 until it does
        self.heard_counter = False
        self.estop = False
        self.last_steering = 0.0
        self.loop = speed_loop
        self.odom_timeout = odom_timeout
        self.measured: Optional[Tuple[float, float]] = None   # (speed m/s, time s)
        self.last_tick: Optional[float] = None
        self.pedals: Optional[Pedals] = None   # the speed loop's last output, to log

    def wheel_speed(self, speed: float, now: float) -> None:
        """Note the measured forward speed (m/s, /vehicle/odom) at `now` (s)."""
        self.measured = (speed, now)

    def command(self, speed: float, steering: float, now: float) -> None:
        """Note ego's command, passed on by the output mux, at `now` (s)."""
        self.cmd = (speed, steering)
        self.cmd_time = now

    def heartbeat(self, counter: int) -> None:
        """Note the Orin's counter: it goes into every frame until the next one."""
        self.counter = counter & 0xFF
        self.heard_counter = True

    def set_estop(self, on: bool) -> bool:
        """Note /safety/estop; return True if it changed."""
        changed = on != self.estop
        self.estop = on
        return changed

    def calibrate(self, mode: int, target: int, now: float) -> None:
        """Note the panel's request: CAL_MODE_OFF, _LIMP or _JOG (to `target` counts)."""
        if mode not in (CAL_MODE_OFF, CAL_MODE_LIMP, CAL_MODE_JOG):
            raise ValueError(f'calibration mode {mode}: use 0 off, 1 limp or 2 jog')
        self.cal_request = (mode, int(target), now)

    def uplink_seen(self, now: float) -> None:
        """Note that an uplink frame came at `now`: the counts are coming back."""
        self.uplink_at = now

    def cal_wanted(self, now: float) -> bool:
        """Return True if the panel asks for limp or jog, and asked within 0.5 s."""
        r = self.cal_request
        return r is not None and r[0] != CAL_MODE_OFF and now - r[2] <= CAL_REQUEST_TIMEOUT

    def cal_refusal(self, now: float) -> Optional[str]:
        """Return why the panel's calibration request isn't passed on now, or None."""
        if self.cal_where == 'none':
            return ('steering_calibration is none: stands or bench only, set by the '
                    'hardware lead')
        if self.version != 3:
            return f'frame_version {self.version}: the calibration needs 3 (the Rust firmware)'
        if self.uplink_at is None or now - self.uplink_at > self.uplink_timeout:
            return 'no uplink from the ESP32: nothing to read the position with'
        if self.estop:
            return 'the e-stop is on'
        if self.fresh(now) and abs(self.cmd[0]) >= self.mapping.stop_speed:
            return f'ego asks for {self.cmd[0]:.2f} m/s: someone is driving'
        if (self.measured is not None and now - self.measured[1] <= self.odom_timeout
                and abs(self.measured[0]) >= STANDSTILL):
            return f'the car moves ({self.measured[0]:.2f} m/s)'
        return None

    def cal_active(self, now: float) -> bool:
        """Return True if this tick's frame carries the panel's limp or jog."""
        return self.cal_wanted(now) and self.cal_refusal(now) is None

    def fresh(self, now: float) -> bool:
        """Return True if ego's last command is no older than the timeout."""
        return self.cmd_time is not None and now - self.cmd_time <= self.timeout

    def frame(self, now: float) -> bytes:
        """Return this tick's frame."""
        if self.cal_active(now):
            if self.loop:
                self.loop.reset()
                self.last_tick = None
                self.pedals = None
            mode, target, _ = self.cal_request
            values = stop_command(self.mapping, self.last_steering)
            return pack_frame_v3(*values, counter=self.counter, armed=False,
                                 estop=self.estop, cal_mode=mode, jog_target=target)
        armed = self.fresh(now) and not self.estop
        if armed:
            values = self.drive(now) if self.loop else to_legacy(*self.cmd, self.mapping)
            self.last_steering = values[0]
        else:
            if self.loop:
                self.loop.reset()
                self.last_tick = None
                self.pedals = None
            values = stop_command(self.mapping, self.last_steering)
        return self.pack(values, armed)

    def drive(self, now: float) -> Tuple[float, float, float]:
        """Run the speed loop one tick; return the frame's values."""
        dt = now - self.last_tick if self.last_tick is not None else 0.02
        self.last_tick = now
        fresh = self.measured is not None and now - self.measured[1] <= self.odom_timeout
        speed, steering = self.cmd
        self.pedals = self.loop.update(speed, self.measured[0] if fresh else None,
                                       max(dt, 1e-3))
        return from_pedals(steering, self.pedals.throttle, self.pedals.brake, self.mapping)

    def stop_frame(self) -> bytes:
        """Return a stale-stop frame, for leaving the car braked."""
        return self.pack(stop_command(self.mapping, self.last_steering), armed=False)

    def pack(self, values: Tuple[float, float, float], armed: bool) -> bytes:
        if self.version == 1:
            return pack_frame(*values)
        if self.version == 3:
            return pack_frame_v3(*values, counter=self.counter, armed=armed, estop=self.estop)
        return pack_frame_v2(*values, counter=self.counter, armed=armed, estop=self.estop)

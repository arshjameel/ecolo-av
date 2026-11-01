"""
The drive tools, as plain Python: status, drive, turn, stop, estop, go_to_gps, wait.

No ROS imports. The tools talk to the car through a Backend: ros_backend.RosBackend on a
ROS graph, fake_backend.FakeBackend in tests and the model test. Every safety rule is
checked HERE, whatever the caller (a language model, a person, a script) asks for:

- hard caps on speed, distance and reverse (safety.Caps, tightened to ego's limits);
- nothing moves while the e-stop is on, and these tools never clear it;
- no driving on what may be the real car, unless the server was started with --car;
- every call is logged, with the time.

Use it from Python:

    tools = DriveTools(RosBackend(node), car=False)
    tools.drive(10.0)            # 10 m forward in total, ramps included; waits until done
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
import os
from pathlib import Path
import time
from typing import Callable, Dict, List, Optional, Protocol, Tuple

from aav_language import drive_maths, safety

MOTION_TOOLS = ('drive', 'turn', 'go_to_gps')
FIX_KINDS = {-1: 'no fix', 0: 'GPS fix', 1: 'GPS fix with SBAS', 2: 'RTK fix (GBAS)'}


class BackendError(Exception):
    """The car side refused or failed a request; the message says why."""


@dataclass
class CarState:
    """What the server knows about the car now. None = not known (nothing heard)."""

    speed: Optional[float] = None                            # m/s, /localization/odom/local
    odom_pose: Optional[Tuple[float, float, float]] = None   # x, y (m), yaw (rad) in odom
    odom_age: Optional[float] = None                         # s since the last odometry
    map_pose: Optional[Tuple[float, float, float]] = None    # x, y, yaw in map (TF)
    gps: Optional[Tuple[float, float]] = None                # lat, lon of the last fix
    gps_status: Optional[int] = None                         # NavSatStatus.status
    estop: Optional[bool] = None                             # the last /safety/estop
    vehicle_estop: Optional[bool] = None                     # /vehicle/status estop_active
    armed: Optional[bool] = None                             # /vehicle/status armed
    mux: Dict[str, Optional[str]] = field(default_factory=dict)  # driver, blocked_by
    mux_locks: Optional[Dict[str, bool]] = None   # twist_mux's own report (/diagnostics)
    fence: Optional[List[Tuple[float, float]]] = None  # /safety/geofence/fence, map frame


class Backend(Protocol):
    """What the tools need from the car side."""

    def state(self) -> CarState: ...
    def where(self) -> safety.Where: ...
    def ego_limits(self) -> Optional[Dict[str, float]]: ...
    def send_scripted(self, goal: drive_maths.DriveGoal) -> str: ...
    def send_gps(self, lat: float, lon: float) -> str: ...
    def nav_ready(self) -> bool: ...
    def result(self, goal_id: str, timeout: Optional[float]) -> Optional[dict]: ...
    def running(self) -> List[str]: ...
    def cancel_all(self) -> List[str]: ...
    def estop(self) -> None: ...
    def sleep(self, seconds: float) -> None: ...


def default_log_path() -> Path:
    """Where the tool calls are logged: $ROS_LOG_DIR, else ~/.ros/log, in aav_language/."""
    base = os.environ.get('ROS_LOG_DIR') or os.path.join(
        os.environ.get('ROS_HOME') or os.path.expanduser('~/.ros'), 'log')
    return Path(base) / 'aav_language' / 'tool_calls.jsonl'


def estop_state(s: CarState) -> Optional[bool]:
    """Say whether any source says the e-stop is on; None when no source has spoken."""
    said = [v for v in (s.estop, s.vehicle_estop,
                        None if s.mux_locks is None else s.mux_locks.get('estop'))
            if v is not None]
    return any(said) if said else None


def _r(x: Optional[float], digits: int = 2) -> Optional[float]:
    return None if x is None else round(float(x), digits)


class DriveTools:
    """
    The tools, with every rule enforced. Each public method returns a dict, ok or not.

    backend: the car side. car: True only when started with --car (the real car).
    caps: the server's own limits (tightened further to ego's when ego answers).
    mux: twist_mux's (inputs, locks) from its configuration, so a move is refused while a
    lock holds the driver that would carry it out. Empty: only the e-stop lock counts.
    log: a function given each logged line; log_path: also a JSON-lines file (None = off).
    """

    def __init__(self, backend: Backend, car: bool = False,
                 caps: Optional[safety.Caps] = None,
                 log: Optional[Callable[[str], None]] = None,
                 log_path: Optional[Path] = None,
                 mux: Tuple[Tuple[safety.MuxInput, ...], Tuple[safety.MuxLock, ...]] = ((), ())):
        self.backend = backend
        self.mux_inputs, self.mux_locks = mux
        self.car = car
        self.server_caps = caps or safety.Caps()
        self.log = log or (lambda line: None)
        self.log_path = log_path

    # --- plumbing ---

    def caps(self) -> safety.Caps:
        return safety.effective_caps(self.server_caps, self.backend.ego_limits())

    def _record(self, tool: str, args: dict, result: dict, started: float) -> dict:
        entry = {'time': time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime(started)),
                 'tool': tool, 'args': args, 'ok': result.get('ok'),
                 'message': result.get('message'),
                 'seconds': round(time.monotonic() - self._t0, 3)}
        self.log(json.dumps(entry))
        if self.log_path is not None:
            try:
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                with self.log_path.open('a', encoding='utf-8') as f:
                    f.write(json.dumps({**entry, 'result': result}) + '\n')
            except OSError as e:
                self.log(f'could not write {self.log_path}: {e}')
        return result

    def _call(self, tool: str, args: dict, body: Callable[[], dict]) -> dict:
        started, self._t0 = time.time(), time.monotonic()
        try:
            result = body()
        except BackendError as e:
            result = {'ok': False, 'message': str(e)}
        return self._record(tool, args, result, started)

    def _may_move(self, driver: str) -> Optional[str]:
        """Why nothing may move now through this twist_mux input ('scripted', 'nav'), or None."""
        why = safety.allowed_here(self.backend.where(), self.car)
        if why:
            return why
        s = self.backend.state()
        if s.mux_locks is None:
            return ("twist_mux's report (/diagnostics) isn't arriving, so whether the e-stop "
                    'is on is unknown. Is the stack running?')
        if s.estop or s.vehicle_estop or s.mux_locks.get('estop'):
            return ('the e-stop is on. A person must clear it (pixi run estop-clear); '
                    'this server never does')
        held = safety.holding_locks(driver, self.mux_inputs, self.mux_locks, s.mux_locks)
        if held:
            return (f"twist_mux's {', '.join(held)} lock holds this drive "
                    f'({driver}) now; it would not move')
        busy = self.backend.running()
        if busy:
            return f'a drive is already running ({busy[0]}): stop() it or wait for it'
        return None

    def _finish(self, goal_id: str, wait: bool, timeout: float, before: CarState,
                planned: dict) -> dict:
        if not wait:
            return {'ok': True, 'message': 'started; status() shows it running',
                    'goal_id': goal_id, **planned}
        res = self.backend.result(goal_id, timeout)
        if res is None:
            self.backend.cancel_all()
            return {'ok': False, 'goal_id': goal_id, **planned,
                    'message': f'no result in {timeout:.0f} s, so it was cancelled (slowing '
                               'to a stop)'}
        after = self.backend.state()
        out = {'ok': res.get('status') == 'succeeded', 'goal_id': goal_id, **planned,
               'message': res.get('message', res.get('status')),
               'status': res.get('status'),
               'distance_travelled_m': _r(res.get('distance_travelled')),
               'duration_s': _r(res.get('duration'), 1)}
        if before.odom_pose and after.odom_pose:
            (x0, y0, a0), (x1, y1, a1) = before.odom_pose, after.odom_pose
            out['odom_displacement_m'] = _r(math.hypot(x1 - x0, y1 - y0))
            turned = math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
            out['heading_change_deg'] = _r(turned, 1)
        return out

    # --- the tools ---

    def status(self) -> dict:
        """Speed, pose, GPS, e-stop, armed, the mux's driver, and any running drive."""
        def body():
            s = self.backend.state()
            c = self.caps()
            out = {'ok': True,
                   'speed_mps': _r(s.speed),
                   'odom_pose': None if s.odom_pose is None else {
                       'x_m': _r(s.odom_pose[0]), 'y_m': _r(s.odom_pose[1]),
                       'heading_deg': _r(math.degrees(s.odom_pose[2]), 1)},
                   'map_pose': None if s.map_pose is None else {
                       'x_m': _r(s.map_pose[0]), 'y_m': _r(s.map_pose[1]),
                       'heading_deg': _r(math.degrees(s.map_pose[2]), 1)},
                   'gps': None if s.gps is None else {
                       'lat': round(s.gps[0], 7), 'lon': round(s.gps[1], 7),
                       'fix': FIX_KINDS.get(s.gps_status, f'status {s.gps_status}')},
                   'estop': estop_state(s),
                   'armed': s.armed,
                   'mux_driver': s.mux.get('driver'),
                   'mux_blocked_by': s.mux.get('blocked_by'),
                   'drive_running': self.backend.running() or None,
                   'limits': {'max_speed_mps': c.max_speed, 'max_distance_m': c.max_distance,
                              'max_reverse_speed_mps': c.max_reverse_speed,
                              'max_reverse_distance_m': c.max_reverse_distance,
                              'min_turn_radius_m': _r(c.min_radius)}}
            why = safety.allowed_here(self.backend.where(), self.car)
            out['may_drive_here'] = why is None
            if why:
                out['why_not'] = why
            out['message'] = 'ok'
            return out
        return self._call('status', {}, body)

    def check_drive(self, distance_m: float, speed_mps: float = 0.5,
                    direction: str = 'forward', curvature: float = 0.0) -> Optional[str]:
        """Why drive() would refuse these numbers now, or None. Moves nothing."""
        if direction not in ('forward', 'reverse'):
            return 'direction must be "forward" or "reverse"'
        c = self.caps()
        why = safety.check_drive(c, float(distance_m), float(speed_mps),
                                 direction == 'reverse')
        if why is None and curvature and c.min_radius and abs(1.0 / curvature) < c.min_radius:
            why = f'the car cannot turn tighter than a {c.min_radius:.1f} m radius'
        if why is None:
            s = self.backend.state()
            why = safety.fence_refusal(s.fence, safety.drive_end(
                s.map_pose, float(distance_m), float(curvature), direction == 'reverse'))
        return why or self._may_move('scripted')

    def drive(self, distance_m: float, speed_mps: float = 0.5, direction: str = 'forward',
              curvature: float = 0.0, wait: bool = True) -> dict:
        """
        Drive distance_m metres IN TOTAL (speeding up and slowing down included).

        speed_mps is the top speed; direction is "forward" or "reverse"; curvature (1/m,
        + = left) is 0 for straight. wait=False returns a goal id at once.
        """
        args = {'distance_m': distance_m, 'speed_mps': speed_mps, 'direction': direction,
                'curvature': curvature, 'wait': wait}

        def body():
            why = self.check_drive(distance_m, speed_mps, direction, curvature)
            if why:
                return {'ok': False, 'message': f'refused: {why}'}
            c = self.caps()
            goal = drive_maths.plan_distance(float(distance_m), float(speed_mps),
                                             c.acceleration, c.deceleration,
                                             reverse=direction == 'reverse',
                                             curvature=float(curvature))
            planned = {'goal': goal.as_dict()}
            before = self.backend.state()
            goal_id = self.backend.send_scripted(goal)
            return self._finish(goal_id, wait, 3.0 * goal.expected_duration + 30.0, before,
                                planned)
        return self._call('drive', args, body)

    def check_turn(self, angle_deg: float, radius_m: float, direction: Optional[str] = None,
                   speed_mps: float = 0.5) -> Optional[str]:
        """Why turn() would refuse these numbers now, or None. Moves nothing."""
        try:
            angle = drive_maths.signed_angle(float(angle_deg), direction)
        except ValueError as e:
            return str(e)
        why = safety.check_turn(self.caps(), angle, float(radius_m), float(speed_mps))
        if why is None:
            arc = drive_maths.plan_arc(angle, float(radius_m))
            s = self.backend.state()
            why = safety.fence_refusal(s.fence, safety.drive_end(s.map_pose, arc.length,
                                                                 arc.curvature))
        return why or self._may_move('scripted')

    def turn(self, angle_deg: float, radius_m: float, direction: Optional[str] = None,
             speed_mps: float = 0.5, wait: bool = True) -> dict:
        """
        Turn by angle_deg driving forward on a circle of radius_m.

        direction "left" or "right" gives the side, and angle_deg is then only the size;
        without it, + = left and - = right (REP-103). The arc's length is radius x angle;
        the drive covers exactly that, ramps included.
        """
        args = {'angle_deg': angle_deg, 'radius_m': radius_m, 'direction': direction,
                'speed_mps': speed_mps, 'wait': wait}

        def body():
            why = self.check_turn(angle_deg, radius_m, direction, speed_mps)
            if why:
                return {'ok': False, 'message': f'refused: {why}'}
            c = self.caps()
            angle = drive_maths.signed_angle(float(angle_deg), direction)
            arc = drive_maths.plan_arc(angle, float(radius_m))
            goal = drive_maths.plan_distance(arc.length, float(speed_mps), c.acceleration,
                                             c.deceleration, curvature=arc.curvature)
            planned = {'goal': goal.as_dict(), 'arc_length_m': _r(arc.length)}
            before = self.backend.state()
            goal_id = self.backend.send_scripted(goal)
            return self._finish(goal_id, wait, 3.0 * goal.expected_duration + 30.0, before,
                                planned)
        return self._call('turn', args, body)

    def check_go_to_gps(self, lat: float, lon: float) -> Optional[str]:
        """Why go_to_gps() would refuse now, or None. Moves nothing."""
        lat, lon = float(lat), float(lon)
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return 'latitude must be -90..90 and longitude -180..180'
        s = self.backend.state()
        if s.gps is None:
            return 'no GPS fix yet, so the distance to that point is unknown'
        d = drive_maths.haversine(s.gps[0], s.gps[1], lat, lon)
        if d > self.caps().max_distance:
            return (f'that point is {d:.0f} m away; commands are limited to '
                    f'{self.caps().max_distance:g} m')
        why = safety.fence_refusal(s.fence, safety.gps_end(s.map_pose, s.gps, lat, lon))
        if why:
            return why
        why = self._may_move('nav')
        if why:
            return why
        if not self.backend.nav_ready():
            return 'Nav2 is not running (no /follow_gps_waypoints server)'
        return None

    def go_to_gps(self, lat: float, lon: float, wait: bool = True) -> dict:
        """Drive to a latitude/longitude with Nav2 (/follow_gps_waypoints), if it's running."""
        args = {'lat': lat, 'lon': lon, 'wait': wait}

        def body():
            why = self.check_go_to_gps(lat, lon)
            if why:
                return {'ok': False, 'message': f'refused: {why}'}
            s = self.backend.state()
            planned = {'straight_line_m': _r(drive_maths.haversine(
                s.gps[0], s.gps[1], float(lat), float(lon)), 1)}
            before = self.backend.state()
            goal_id = self.backend.send_gps(float(lat), float(lon))
            # Nav2 drives at up to 1 m/s, round obstacles: allow plenty.
            return self._finish(goal_id, wait, 3.0 * planned['straight_line_m'] + 60.0,
                                before, planned)
        return self._call('go_to_gps', args, body)

    def stop(self) -> dict:
        """Cancel this server's drive: the car slows down and stops. Never refused."""
        def body():
            cancelled = self.backend.cancel_all()
            if not cancelled:
                return {'ok': True, 'message': 'nothing of mine was running; nothing to stop. '
                        'A person driving (joystick, keyboard) is not affected'}
            return {'ok': True, 'cancelled': cancelled,
                    'message': 'cancelled: slowing down to a stop'}
        return self._call('stop', {}, body)

    def estop(self) -> dict:
        """Press the e-stop (/safety/estop true): every driver stops until a person clears it."""
        def body():
            self.backend.cancel_all()
            self.backend.estop()
            return {'ok': True, 'message': 'e-stop sent (/safety/estop true). Only a person '
                    'can clear it: pixi run estop-clear'}
        return self._call('estop', {}, body)

    def wait(self, seconds: float) -> dict:
        """Wait, doing nothing, for up to the server's limit (120 s)."""
        def body():
            s = float(seconds)
            c = self.caps()
            if not (0 <= s <= c.max_wait):
                return {'ok': False, 'message': f'refused: wait 0 to {c.max_wait:g} s'}
            self.backend.sleep(s)
            return {'ok': True, 'message': f'waited {s:g} s'}
        return self._call('wait', {'seconds': seconds}, body)

    def check(self, tool: str, args: dict) -> Optional[str]:
        """Why `tool` with `args` would be refused now, or None. Moves nothing."""
        try:
            if tool == 'drive':
                return self.check_drive(**args)
            if tool == 'turn':
                return self.check_turn(**{k: v for k, v in args.items() if k != 'wait'})
            if tool == 'go_to_gps':
                return self.check_go_to_gps(args['lat'], args['lon'])
        except (TypeError, KeyError, ValueError) as e:
            return f'bad arguments: {e}'
        return None


def state_dict(s: CarState) -> dict:
    """Return a CarState as plain data, for logs."""
    return asdict(s)

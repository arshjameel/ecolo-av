"""
The joystick-alive check: is the joystick really there, and really moving? No ROS here.

joy_node (joystick_drivers 3.3.0) resends its last state 20 times a second while the USB
receiver is plugged in (autorepeat_rate), with a fresh stamp each time. A pad turned off,
out of range or out of battery, its receiver still in, may then look like a person holding
the trigger and the stick: the car would keep driving (TODO 74a; docs/car-day.md, A9b).

joy_alive (the node) sits between joy_node and teleop_twist_joy and asks this module, for
each Joy message, whether to pass it on or a released copy (every axis 0, every button 0:
teleop_twist_joy then sends one zero and stops). Two rules stop it:

  SILENT  no Joy message for silent_after s: the receiver unplugged (joy_node closes the
          device on SDL's DEVICEREMOVED and stops publishing), or joy_node dead
  FROZEN  the deadman held and a motion axis off-centre (|value| > motion_threshold), with
          every axis and button bit-identical for frozen_after s. Real hands jitter;
          a dead pad's receiver repeats the same numbers. 0 turns the rule off

Both latch until the deadman is seen released: a pad that comes back with the trigger
still held stays stopped. Once released, pressing it again drives. At start-up the same
holds (WAITING): nothing drives until the deadman has been seen released once.

The FROZEN rule can't tell a dead pad from a perfectly still hand. A stick held hard
against its edge reads exactly 1.0, and an 8-bit pad can read the same number for a
second or more: then it stops the car after frozen_after s, and the driver releases the
trigger and presses again. Car day measures it (docs/car-day.md, A9b step 7); a pad whose
receiver goes neutral or vanishes when the pad is off needs only SILENT (frozen_after 0).

Times are seconds on any steady clock.
"""

from dataclasses import dataclass, field
import math
from typing import Optional, Sequence, Tuple

OK = 'OK'
WAITING = 'WAITING'
SILENT = 'SILENT'
FROZEN = 'FROZEN'


@dataclass
class JoyAlive:
    """Watch one joystick's Joy messages; say whether each may pass, and why not."""

    enable_button: int = 0              # the deadman (teleop_twist_joy's enable_button)
    motion_axes: Tuple[int, ...] = (1,)  # the axes that move the car (axis_linear.x)
    motion_threshold: float = 0.05      # |value| above this: the stick asks for motion
    silent_after: float = 0.5           # s with no message: SILENT
    frozen_after: float = 2.0           # s of identical values while driving: FROZEN; 0 off
    tripped: Optional[str] = WAITING    # why it stops (WAITING, SILENT, FROZEN), or None
    last_time: Optional[float] = None   # when the newest message came
    same_since: Optional[float] = None  # when the newest values were first seen
    last_values: Optional[Tuple[Tuple[float, ...], Tuple[int, ...]]] = None
    sizes: Tuple[int, int] = field(default=(0, 0))   # the newest message's axes, buttons

    def __post_init__(self):
        for name in ('silent_after', 'frozen_after', 'motion_threshold'):
            value = getattr(self, name)
            if not value >= 0.0 or math.isinf(value):
                raise ValueError(f'{name} must be a number >= 0')
        if self.silent_after == 0.0:
            raise ValueError('silent_after must be more than 0: silence must stop the car')
        if self.enable_button < 0:
            raise ValueError('enable_button must be a button number (>= 0)')
        self.motion_axes = tuple(self.motion_axes)

    def held(self, buttons: Sequence[int]) -> bool:
        """Return whether the deadman is held in these buttons."""
        return len(buttons) > self.enable_button and bool(buttons[self.enable_button])

    def driving(self, axes: Sequence[float], buttons: Sequence[int]) -> bool:
        """Return whether these values ask the car to move: the deadman and a motion axis."""
        return self.held(buttons) and any(
            i < len(axes) and abs(axes[i]) > self.motion_threshold for i in self.motion_axes)

    def on_joy(self, now: float, axes: Sequence[float], buttons: Sequence[int]) -> bool:
        """Take a Joy message at `now`; return True to pass it on, False to send it released."""
        values = (tuple(axes), tuple(buttons))
        if values != self.last_values:
            self.last_values, self.same_since = values, now
        self.last_time, self.sizes = now, (len(axes), len(buttons))
        if self.tripped is not None and not self.held(buttons):
            self.tripped = None      # released: the next press drives
        if (self.tripped is None and self.frozen_after > 0.0 and self.driving(axes, buttons)
                and now - self.same_since >= self.frozen_after):
            self.tripped = FROZEN
        return self.tripped is None

    def tick(self, now: float) -> bool:
        """Check for silence at `now`; return True if it has just tripped."""
        if (self.last_time is not None and self.tripped != SILENT
                and now - self.last_time >= self.silent_after):
            self.tripped = SILENT
            return True
        return False

    def state(self, now: float) -> str:
        """Return one line for people: the state, a dash, and why."""
        if self.tripped == WAITING:
            if self.last_time is None:
                return f'{WAITING} - no joystick yet'
            return f'{WAITING} - release the deadman (button {self.enable_button}) once'
        if self.tripped == SILENT:
            if self.last_time is None:
                return f'{SILENT} - no joystick'
            if now - self.last_time < self.silent_after:
                return f'{SILENT} - the joystick is back with the deadman held: release it'
            return (f'{SILENT} - nothing from the joystick for {now - self.last_time:.1f} s '
                    f'(more than {self.silent_after:g} s): release the deadman to drive again')
        if self.tripped == FROZEN:
            return (f'{FROZEN} - the same values for over {self.frozen_after:g} s while '
                    'driving: a dead pad? Release the deadman to drive again')
        if self.last_values is not None and self.driving(*self.last_values):
            return f'{OK} - driving'
        return f'{OK} - idle'

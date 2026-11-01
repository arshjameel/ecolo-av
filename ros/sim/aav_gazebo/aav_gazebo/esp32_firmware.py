"""
Today's ESP32 firmware (the 2025-26 C++), as a stand-in for Gazebo. No ROS here.

gazebo_esp32 feeds it the bytes legacy_uart_bridge writes, and sends its outputs to
gazebo_adapter's actuator model (sim.launch.py esp32:=true), so the real bridge drives
Gazebo's car through the same logic as on the car:

    legacy_uart_bridge ── 13-byte frames ──> Esp32Firmware ── throttle, brake, steering ──>

It does what firmware/legacy/aav_umod_esp32/src/modules/ros2_handler/ros2_handler.cpp and
dac/dac.cpp do, for each valid frame:

  - a frame: three little-endian float32 (steering, speed, accel) and a sum checksum; on
    a bad checksum, drop one byte and try again. Any value outside -1..1: ramp the DAC
    toward 0, and change nothing else;
  - accel below -0.05: the brake Jrk to clamp(|accel| x 3500, 400, 3500), the DAC to 0 at
    once, nothing else. Otherwise the brake back to 400 (released), then the throttle:
  - |speed| below 0.03: ramp the DAC toward 0. Otherwise the direction from its sign, and
    ramp the DAC toward |speed| x 700: up 6, down 12 per frame (at 50 frames a second,
    0 to full takes 2.3 s);
  - steering: the Jrk's target; here, the angle the bridge meant (its mapping undone).
    How the real Jrk count turns into an angle is the actuator model's (vehicle.yaml).

At boot the brake is full (3500), the DAC 0. Like the C++, it has no command timeout:
frames stop, and the last outputs stay.
"""

from dataclasses import dataclass
import struct

FRAME_LEN = 13
BRAKE_RELEASED = 400       # JRK_MIN: the brake Jrk's target, released
BRAKE_FULL = 3500          # JRK_MAX
DAC_CAP = 700              # DAC_MAX_CAP: the DAC at speed 1.0
ACCEL_STEP = 6             # DAC counts up per frame
DECEL_STEP = 12            # DAC counts down per frame
BRAKE_BELOW = -0.05        # accel below this brakes
THROTTLE_MIN = 0.03        # |speed| below this ramps the throttle to 0
# The bridge scales negative steering by this (legacy_protocol.NEGATIVE_STEERING_GAIN).
NEGATIVE_STEERING_GAIN = 1550 / 1950


@dataclass(frozen=True)
class Outputs:
    """What the firmware drives, as the actuator model takes it."""

    throttle: float    # 0..1: the DAC over its cap
    brake: float       # 0..1 of the brake Jrk's travel past released
    reverse: bool      # the direction pin: true = backwards
    steering: float    # rad, + left: the angle the bridge asked for


class Esp32Firmware:
    """The C++ firmware's frame handling, ramp and brake, frame by frame."""

    def __init__(self, max_steering_angle: float = 0.5, steering_sign: float = -1.0):
        self.max_steering_angle = max_steering_angle
        self.steering_sign = steering_sign
        self.buf = bytearray()
        self.dac = 0
        self.brake = BRAKE_FULL          # full on from boot, as the C++
        self.reverse = False
        self.steering = 0.0              # the frame's steering number, -1..1
        self.frames = 0                  # valid frames handled
        self.rejected = 0                # frames with a value outside -1..1

    def feed(self, data: bytes) -> int:
        """Take bytes from the serial line; return how many valid frames they held."""
        self.buf += data
        handled = 0
        while len(self.buf) >= FRAME_LEN:
            frame = bytes(self.buf[:FRAME_LEN])
            if sum(frame[:12]) & 0xFF == frame[12]:
                self.handle(*struct.unpack('<fff', frame[:12]))
                del self.buf[:FRAME_LEN]
                handled += 1
            else:
                del self.buf[0]          # resync: memmove(buf, buf + 1, 12)
        return handled

    def handle(self, steering: float, speed: float, accel: float) -> None:
        """One valid frame, as handleROS2Messages() does it."""
        if not all(abs(x) <= 1.0 for x in (steering, speed, accel)):
            self.rejected += 1
            self.ramp(0)                 # "packet rejected — out of range"
            return
        self.frames += 1
        self.steering = steering
        if accel < BRAKE_BELOW:
            self.brake = min(max(int(abs(accel) * BRAKE_FULL), BRAKE_RELEASED), BRAKE_FULL)
            self.dac = 0                 # hard cut, no ramp
            return
        self.brake = BRAKE_RELEASED
        if abs(speed) < THROTTLE_MIN:
            self.ramp(0)
        else:
            self.reverse = speed < 0.0
            self.ramp(int(abs(speed) * DAC_CAP))

    def ramp(self, target: int) -> None:
        """writeDACRamped(): one step toward target, up ACCEL_STEP or down DECEL_STEP."""
        target = min(max(target, 0), DAC_CAP)
        if self.dac < target:
            self.dac = min(self.dac + ACCEL_STEP, target)
        elif self.dac > target:
            self.dac = max(self.dac - DECEL_STEP, target)

    def outputs(self) -> Outputs:
        """Return what the firmware drives now."""
        s = self.steering / NEGATIVE_STEERING_GAIN if self.steering < 0 else self.steering
        return Outputs(throttle=self.dac / DAC_CAP,
                       brake=(self.brake - BRAKE_RELEASED) / (BRAKE_FULL - BRAKE_RELEASED),
                       reverse=self.reverse,
                       steering=s * self.max_steering_angle / self.steering_sign)

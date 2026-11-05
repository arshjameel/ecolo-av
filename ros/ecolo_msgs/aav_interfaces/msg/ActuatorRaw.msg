# One actuator's position, raw: its motor controller's (Jrk G2's) scaled feedback in counts,
# 0..4095, in the same units as its targets. Not converted with the firmware's
# calibration, so it can be used to measure that calibration (docs/actuator-tests.md,
# "Find the steering actuator's ends and centre").
#
# Published by legacy_uart_bridge from the ESP32's version 3 uplink, 50 Hz, on
# /vehicle/steering_actuator/raw and /vehicle/brake_actuator/raw.

std_msgs/Header header
bool valid          # false: no position (the Jrk's feedback isn't analog, or the read failed)
uint16 position     # counts, 0..4095; 0 when not valid

# The steering calibration's state in the firmware (legacy-link's CalState, the uplink's
# byte 38). Always CALIBRATION_OFF on the brake actuator.
uint8 CALIBRATION_OFF = 0
uint8 CALIBRATION_LIMP = 1                  # its motor off: turned by hand
uint8 CALIBRATION_JOG = 2                   # moving slowly to, or at, a jog target
uint8 CALIBRATION_HOLD = 3                  # ended without fresh frames: holding there
uint8 CALIBRATION_RETURN = 4                # ended: moving back slowly
uint8 CALIBRATION_REFUSED_MOVING = 5        # refused: the speed command isn't zero
uint8 CALIBRATION_REFUSED_NOT_BRAKED = 6    # refused: no brake, or the throttle on
uint8 CALIBRATION_REFUSED_NO_FEEDBACK = 7   # refused: no position (feedback not analog)
uint8 CALIBRATION_REFUSED_ESTOP = 8         # refused: an e-stop
uint8 CALIBRATION_REFUSED_OTHER_DRIVER = 9  # refused: the RC (direct) or a test drives
uint8 calibration

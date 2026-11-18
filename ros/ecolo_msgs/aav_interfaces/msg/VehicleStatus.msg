# State of the car as reported by the backend that drives it.
# Published on /vehicle/status by every backend (docs/interfaces.yaml):
# spine_bridge, sim_adapter, turtlesim_adapter, carla_adapter, legacy_uart_bridge
# (that one only with uplink true).
# On the spine it comes from CAN 0x181 VEHICLE_STATUS, with speed and steering from
# 0x180 VEHICLE_FEEDBACK; the DBC in firmware/can/aav.dbc is the source of truth for
# that wire format.

std_msgs/Header header

# Who is driving the actuators.
uint8 MODE_MANUAL=0      # a person drives; the actuators are idle
uint8 MODE_RC=1          # the RC transmitter overrides the computer
uint8 MODE_COMPUTER=2    # the spine follows /ego/actuator_cmd
uint8 MODE_TEST=3        # the hardware test mode: one actuator at a time, from a laptop
uint8 mode

bool armed               # the spine accepts computer commands
bool estop_active        # any stop: hardware button, /safety/estop, or watchdog

float32 speed            # m/s, measured, negative in reverse
float32 steering_angle   # rad, measured, positive turns left (REP-103)
float32 throttle         # 0..1 of full throttle
float32 brake            # 0..1 of full brake

# Fault bits, same values as the `faults` signal of VEHICLE_STATUS.
uint16 FAULT_CMD_TIMEOUT=1        # no valid DRIVE_CMD for 100 ms
uint16 FAULT_BAD_CHECKSUM=2       # a frame failed its CRC
uint16 FAULT_BAD_COUNTER=4        # a frame repeated or skipped its counter
uint16 FAULT_STEERING_DRIVER=8    # steering Jrk G2 not responding or in error
uint16 FAULT_BRAKE_DRIVER=16      # brake Jrk G2 not responding or in error
uint16 FAULT_THROTTLE_DAC=32      # throttle DAC not responding
uint16 FAULT_WHEEL_ENCODER=64     # no ticks while the car should be moving
uint16 FAULT_CAN_BUS=128          # CAN error passive or bus off
uint16 FAULT_RC_SIGNAL_LOST=256   # RC receiver silent while in RC mode
uint16 FAULT_OVERSPEED=512        # measured speed above the firmware limit
uint16 faults

# A person on the hardware panel asks for the steering actuator's calibration mode, to
# find its ends and centre (docs/actuator-tests.md). Sent again at 10 Hz while the person
# holds the mode; legacy_uart_bridge drops it after 0.5 s of silence.
#
# On /drivers/buttons/steering_calibration, from hw_panel only: never Nav2 or a program.

std_msgs/Header header
uint8 MODE_OFF = 0    # drive as usual
uint8 MODE_LIMP = 1   # the steering actuator's motor off: turn the wheels by hand
uint8 MODE_JOG = 2    # go to target, slowly, inside the software limits
uint8 mode
uint16 target         # counts, 0..4095, for MODE_JOG; ignored otherwise

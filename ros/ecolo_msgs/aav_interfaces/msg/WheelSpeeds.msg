# Each wheel's speed along the ground, in m/s (its spin times its radius), negative in
# reverse. NaN means that wheel isn't measured, or its reading is stale.
# Published on /vehicle/wheels by the backend:
#   - sim_adapter: all four, from the wheel joints;
#   - spine_bridge: the front two, from the CAN wheel nodes (0x200 WHEEL_SPEED_L,
#     0x201 WHEEL_SPEED_R, firmware/can/aav.dbc), about 100 Hz; the rear two NaN.

std_msgs/Header header       # frame_id base_link
float32 front_left
float32 front_right
float32 rear_left
float32 rear_right

# false: the sign comes from the drive direction, because one Hall sensor can't tell
# which way its wheel turns. Gazebo always knows.
bool direction_measured

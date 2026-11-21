std_msgs/Header header
float32 left_curvature_m
float32 right_curvature_m
float32 mean_curvature_m
float32 curve_direction
float32 vehicle_offset_m
float32 lane_width_m
float32 left_poly_a
float32 left_poly_b
float32 left_poly_c
float32 right_poly_a
float32 right_poly_b
float32 right_poly_c
float32 left_confidence
float32 right_confidence
bool left_lane_detected
bool right_lane_detected
bool both_lanes_detected
float32 steering_hint
# Each line's paint, from its pixels' colour (TODO 264): the costmap's camera lanes price
# a yellow centre line apart from a white edge line. UNKNOWN when the line isn't detected.
uint8 PAINT_UNKNOWN=0
uint8 PAINT_WHITE=1
uint8 PAINT_YELLOW=2
uint8 left_paint
uint8 right_paint

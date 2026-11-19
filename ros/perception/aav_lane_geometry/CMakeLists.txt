cmake_minimum_required(VERSION 3.10)
project(aav_lane_geometry)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneGeometry.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/control_interface.py
  scripts/fake_lane_geometry_pub.py
  DESTINATION lib/${PROJECT_NAME}
)
# control_interface.py imports them from beside itself.
install(FILES scripts/lane_speed.py scripts/lane_follow.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch msg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # The slow-down rule and the follower (no ROS needed).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lane_speed test/test_lane_speed.py)
  # The follower's logic, and that aav_lane_follower_rs's vectors are still its answers.
  ament_add_pytest_test(test_lane_follow test/test_lane_follow.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

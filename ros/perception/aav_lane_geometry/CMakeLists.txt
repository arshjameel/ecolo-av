cmake_minimum_required(VERSION 3.10)
project(aav_lane_geometry)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# The lane finder's message. The 2025-26 lane follower that read it was removed on 19 Nov
# 2026 (TODO 264; git tag save/lane-followers): the costmap draws the lanes now.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneGeometry.msg"
  DEPENDENCIES std_msgs
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.10)
project(aav_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The launch file's own rules (no ROS needed).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_params test/test_launch_params.py)
  ament_add_pytest_test(test_respawn test/test_respawn.py)
endif()

ament_package()

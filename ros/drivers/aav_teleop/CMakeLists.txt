cmake_minimum_required(VERSION 3.10)
project(aav_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The joystick's chain, with each joy_alive (rust, python): a pretend pad drives ego
  # through teleop_twist_joy and twist_mux; frozen and silent stop it.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(joy_chain test/test_joy_chain.py TIMEOUT 240)
endif()

ament_package()

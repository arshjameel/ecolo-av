cmake_minimum_required(VERSION 3.10)
project(aav_command_mux)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_mux_runtime
    test/test_mux_runtime.py
    TIMEOUT 30
  )
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

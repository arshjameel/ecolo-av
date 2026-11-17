cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_voxels_rs is: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat lidar_scan like any other node.
project(aav_lidar_rs NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup (Ubuntu's apt cargo is too old for rclrs)")
endif()

# Build in colcon's build folder, not in src/. cargo itself decides what's out of date.
# make's jobserver flags are dropped: cargo can't reach the jobserver from here, and says so.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet --bin lidar_scan
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin lidar_scan"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/lidar_scan
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The maths without ROS, and the Python's answers on its clouds.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(lidar_scan_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p lidar-scan
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The node itself: one cloud in, the Python's scan out.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lidar_scan_node test/test_lidar_scan_node.py TIMEOUT 180)
endif()

ament_package()

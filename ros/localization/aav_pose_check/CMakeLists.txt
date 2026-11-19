cmake_minimum_required(VERSION 3.10)
# A Rust package, built the same way as aav_ndt: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat pose_check like any other node.
project(aav_pose_check NONE)

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
    ${CARGO} build --release --locked --quiet --bin pose_check
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin pose_check"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/pose_check
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The checker, without ROS: cargo test -p pose-check-core.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(pose_check_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p pose-check-core
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The node itself: a stream of NDT fits and GPS fixes in, the passed poses and the
  # status out (test/test_pose_check_node.py).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pose_check_node test/test_pose_check_node.py TIMEOUT 120)
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
# A Rust package, built the same way as aav_joy_alive_rs: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat control_interface like any other node.
project(aav_lane_follower_rs NONE)

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
    ${CARGO} build --release --locked --quiet --bin control_interface
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin control_interface"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/control_interface
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The states and the laws, without ROS: lane_follow.py's answers (tests/vectors.json,
  # from gen_vectors.py). cargo test -p lane-follow-core.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(lane_follow_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p lane-follow-core
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The node itself, and the Python one: lane messages in, the same commands out.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_interface_node test/test_control_interface_node.py
    TIMEOUT 180)
endif()

ament_package()

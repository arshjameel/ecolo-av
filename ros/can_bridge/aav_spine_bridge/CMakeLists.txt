cmake_minimum_required(VERSION 3.10)
# A Rust package, built the same way as aav_imu_yahboom: CMake only runs cargo and installs
# the result, so colcon, `ros2 run` and launch files treat spine_bridge like any other node.
project(aav_spine_bridge NONE)

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
    ${CARGO} build --release --locked --quiet --bin spine_bridge --bin arm_when_open
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin spine_bridge --bin arm_when_open"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/spine_bridge
  ${CARGO_TARGET_DIR}/release/arm_when_open
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The bridge's logic, without ROS: the Python node's test cases and the shared CAN test
  # vectors (firmware/can/test_vectors.json). cargo test -p spine-bridge.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(spine_bridge_logic
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p spine-bridge
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()

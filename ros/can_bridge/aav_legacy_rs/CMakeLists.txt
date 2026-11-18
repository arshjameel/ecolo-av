cmake_minimum_required(VERSION 3.10)
# A Rust package, built the same way as aav_imu_yahboom: CMake only runs cargo and installs
# the result, so colcon, `ros2 run` and launch files treat legacy_uart_bridge like any other node.
project(aav_legacy_rs NONE)

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
    ${CARGO} build --release --locked --quiet --bin legacy_uart_bridge
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin legacy_uart_bridge"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/legacy_uart_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The bridge's logic, without ROS: the Python modules' answers (tests/vectors.json, from
  # gen_vectors.py). cargo test -p legacy-bridge.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(legacy_bridge_logic
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p legacy-bridge
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The process itself runs in aav_legacy's chain tests, beside the Python one
  # (test_heartbeat_chain.py, test_uplink_chain.py, test_no_timeout_lock.py: impl rust).
endif()

ament_package()

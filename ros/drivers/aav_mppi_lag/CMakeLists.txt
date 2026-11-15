cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_spine_bridge is: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat mppi_lag like any other node.
project(aav_mppi_lag NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup (Ubuntu's apt cargo is too old for rclrs)")
endif()

# Build in colcon's build folder, not in src/. cargo itself decides what's out of date.
# make's jobserver flags are dropped: cargo can't reach the jobserver from here, and says so.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
# TODO 196: -DMPPI_LAG_FEATURES=trace (or tracy) builds in the stage spans; empty: none.
set(MPPI_LAG_FEATURES "" CACHE STRING "mppi-node's cargo features: trace, tracy or none")
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet --bin mppi_lag
    "--features=${MPPI_LAG_FEATURES}"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin mppi_lag"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/mppi_lag
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # MPPI without ROS: each critic, the lag model, the smoothing, and the closed-loop runs
  # (a circle, an S-curve and a corner, lag-aware against lag-free). cargo test -p mppi-lag.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(mppi_lag_logic
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p mppi-lag
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()

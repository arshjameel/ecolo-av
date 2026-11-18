cmake_minimum_required(VERSION 3.10)
# A Rust package, built the same way as aav_rc_rs: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat point_driver like any other node.
project(aav_drive_to_point_rs NONE)

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
    ${CARGO} build --release --locked --quiet --bin point_driver --bin drive_to_point
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin point_driver --bin drive_to_point"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/point_driver
  ${CARGO_TARGET_DIR}/release/drive_to_point
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The maths and rules, without ROS: the Python modules' answers (tests/vectors.json, from
  # gen_vectors.py). cargo test -p point-core.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(point_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p point-core
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The process itself: pixi run sim-check --drive-to-point (point_driver in Ravensim).
endif()

ament_package()

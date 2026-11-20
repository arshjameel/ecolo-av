cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_joy_alive_rs is: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat gps_heading like any other node.
project(aav_gps_heading_rs NONE)

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
    ${CARGO} build --release --locked --quiet --bin gps_heading
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin gps_heading"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/gps_heading
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The maths, without ROS: course.py's answers to the bit (tests/vectors.json, from
  # gen_vectors.py). cargo test -p gps-heading-core.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(gps_heading_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p gps-heading-core
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The process itself: wheels and a GPS velocity in, a heading out; slow, nothing.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gps_heading_rs_node test/test_gps_heading_rs_node.py TIMEOUT 120)
endif()

ament_package()

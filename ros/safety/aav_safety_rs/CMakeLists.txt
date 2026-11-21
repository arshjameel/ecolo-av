cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_joy_alive_rs is: CMake only runs cargo and installs the
# results, so colcon, `ros2 run` and launch files treat the three nodes like any other.
project(aav_safety_rs NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup (Ubuntu's apt cargo is too old for rclrs)")
endif()

# Build in colcon's build folder, not in src/. cargo itself decides what's out of date.
# make's jobserver flags are dropped: cargo can't reach the jobserver from here, and says so.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
set(NODES geofence stop_hold goal_leash)
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet -p safety-node
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release -p safety-node (geofence, stop_hold, goal_leash)"
  VERBATIM)

foreach(node ${NODES})
  install(PROGRAMS ${CARGO_TARGET_DIR}/release/${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  # The rules, without ROS: geofence.py's, hold.py's and leash.py's answers
  # (tests/vectors.json, from gen_vectors.py). cargo test -p safety-core.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(safety_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p safety-core
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The processes themselves: the fence, LOST, OUTSIDE latched and cleared; a held stop;
  # a quiet laptop's goals cancelled.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(safety_rs_nodes test/test_safety_rs_nodes.py TIMEOUT 240)
endif()

ament_package()

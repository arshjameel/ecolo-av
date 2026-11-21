cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_ego_rs is: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat the self-test like any other node.
project(aav_self_test_rs NONE)

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
    ${CARGO} build --release --locked --quiet --bin boot_self_test
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin boot_self_test"
  VERBATIM)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS ${CARGO_TARGET_DIR}/release/boot_self_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The logic without ROS: every check, the settling, the verdict.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(self_test_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p self-test-core
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  # The process itself: NOT READY, then READY as every input speaks, NOT READY again
  # when one goes quiet.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(self_test_node test/test_self_test_node.py TIMEOUT 180)
endif()

ament_package()

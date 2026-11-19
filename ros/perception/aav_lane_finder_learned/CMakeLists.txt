cmake_minimum_required(VERSION 3.10)
# A Rust package, built as aav_detector_rs is: CMake only runs cargo and installs the result,
# so colcon, `ros2 run` and launch files treat the learned lane_finder like any other node.
project(aav_lane_finder_learned NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup (Ubuntu's apt cargo is too old for rclrs)")
endif()

# The Orin: -DLANE_LEARNED=ON -DLANE_LEARNED_FEATURES=tensorrt (untested: docs/onnx.md).
set(LANE_LEARNED_FEATURES "" CACHE STRING "cargo features for the node: tensorrt, cuda")
set(FEATURE_ARGS "")
if(LANE_LEARNED_FEATURES)
  set(FEATURE_ARGS --features ${LANE_LEARNED_FEATURES})
endif()

# Built on macOS by default (ONNX Runtime + Core ML). On Linux (CI, the car kit, the Orin)
# only with -DLANE_LEARNED=ON, as the Rust detector: ort's downloaded ONNX Runtime didn't
# link with conda's gcc on CI's Linux (17 Nov), and the car kit installs offline.
if(APPLE)
  set(LANE_LEARNED_DEFAULT ON)
else()
  set(LANE_LEARNED_DEFAULT OFF)
endif()
option(LANE_LEARNED "build the learned lane_finder (needs ONNX Runtime)" ${LANE_LEARNED_DEFAULT})

# Build in colcon's build folder, not in src/. make's jobserver flags are dropped.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
if(LANE_LEARNED)
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet --bin lane_finder ${FEATURE_ARGS}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin lane_finder (learned)"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/lane_finder
  DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "aav_lane_finder_learned: the node is not built here (-DLANE_LEARNED=ON builds it)")
endif()

if(BUILD_TESTING)
  # The maths without ROS (the crop and scale, decoding, the ground, the lane); with ONNX
  # Runtime and models/lane-ufld.onnx, the model on Ravensim's pictures too.
  find_package(ament_cmake_test REQUIRED)
  if(LANE_LEARNED)
    set(TEST_FEATURES --features ort)
  else()
    set(TEST_FEATURES "")
  endif()
  ament_add_test(ufld_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p ufld-core ${TEST_FEATURES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 1800
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  if(LANE_LEARNED)
    # The node itself: skips without models/lane-ufld.onnx (outside git).
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(lane_finder_learned_node test/test_lane_finder_learned_node.py
      TIMEOUT 180)
  endif()
endif()

ament_package()

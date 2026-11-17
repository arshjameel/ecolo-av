cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_voxels_rs is: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat the detector like any other node.
project(aav_detector_rs NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup (Ubuntu's apt cargo is too old for rclrs)")
endif()

# The Orin: colcon build --cmake-args -DDETECTOR_FEATURES=tensorrt (untested: docs/onnx.md).
set(DETECTOR_FEATURES "" CACHE STRING "cargo features for the detector: tensorrt, cuda")
set(FEATURE_ARGS "")
if(DETECTOR_FEATURES)
  set(FEATURE_ARGS --features ${DETECTOR_FEATURES})
endif()

# Build in colcon's build folder, not in src/. cargo itself decides what's out of date.
# make's jobserver flags are dropped: cargo can't reach the jobserver from here, and says so.
# The first build downloads ONNX Runtime (pyke's build, for this computer) into its cache.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet --bin detector ${FEATURE_ARGS}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin detector"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/detector
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # The maths without ROS (resize, normalise, decode, labels, outlines); and, with the
  # model in models/, the same boxes as the Python on the saved pictures.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(detector_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p detector-core --features ort
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()

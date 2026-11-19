cmake_minimum_required(VERSION 3.10)
# A Rust package, built as aav_detector_rs is: CMake only runs cargo and installs the
# result. No ROS in it: diffusion_worker talks to aav_diffusion_planner's node over a pipe.
project(aav_diffusion_rs NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup")
endif()

# A Mac: Core ML, built by default. Linux (CI, the car kit, the Orin): only with
# -DDIFFUSION_RS=ON, as the Rust detector: ort's downloaded ONNX Runtime didn't link with
# conda's gcc on CI's Linux (17 Nov), and the car kit installs offline. The Orin:
# -DDIFFUSION_RS=ON -DDIFFUSION_FEATURES=tensorrt (untested: docs/onnx.md).
if(APPLE)
  set(DIFFUSION_FEATURES_DEFAULT coreml)
  set(DIFFUSION_RS_DEFAULT ON)
else()
  set(DIFFUSION_FEATURES_DEFAULT "")
  set(DIFFUSION_RS_DEFAULT OFF)
endif()
set(DIFFUSION_FEATURES "${DIFFUSION_FEATURES_DEFAULT}" CACHE STRING
  "cargo features for diffusion_worker: coreml, tensorrt, cuda")
option(DIFFUSION_RS "build the Rust diffusion worker (needs ONNX Runtime)" ${DIFFUSION_RS_DEFAULT})
set(FEATURE_ARGS "")
if(DIFFUSION_FEATURES)
  set(FEATURE_ARGS --features ${DIFFUSION_FEATURES})
endif()

# Build in colcon's build folder, not in src/. make's jobserver flags are dropped.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
if(DIFFUSION_RS)
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet --bin diffusion_worker ${FEATURE_ARGS}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin diffusion_worker"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/diffusion_worker
  DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "aav_diffusion_rs: the Rust worker is not built here (-DDIFFUSION_RS=ON builds it)")
endif()

if(BUILD_TESTING AND DIFFUSION_RS)
  # The pipe and the batches; with the model in models/, the CPU's and Core ML's paths.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(diffusion_core
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO} test --release --locked --quiet -p diffusion-core ${FEATURE_ARGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 1800
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()

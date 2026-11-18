cmake_minimum_required(VERSION 3.10)
project(aav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # car.launch.py finds its machine files in this package's own install folder.
  ament_add_pytest_test(test_car_launch test/test_car_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  ament_add_pytest_test(test_sim_launch test/test_sim_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  # CARLA as the car: the same stack as Gazebo's, and carla_adapter's parameters.
  ament_add_pytest_test(test_carla_launch test/test_carla_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  # target:= and source:=: one launch for every target, and each profile's two muxes.
  ament_add_pytest_test(test_stack_launch test/test_stack_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  # foxglove_bridge for Lichtblick: where each profile binds it, its topic list, previews.
  ament_add_pytest_test(test_foxglove_launch test/test_foxglove_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  # Nav2's settings with and without reversing, as navigation.launch.py writes them.
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  # A launch value reaches the Rust node (namespace '/'): ego's max_speed, the legacy
  # bridge's port and frame, the output mux's target, read back from the running processes.
  ament_add_pytest_test(test_rust_overrides test/test_rust_overrides.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    TIMEOUT 180)
endif()

ament_package()

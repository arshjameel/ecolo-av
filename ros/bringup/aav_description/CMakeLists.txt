cmake_minimum_required(VERSION 3.10)
project(aav_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME})

# aav_description.vehicle: lets launch files read config/vehicle.yaml.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_vehicle test/test_vehicle.py)
  ament_add_pytest_test(test_car_model test/test_car_model.py)
  ament_add_pytest_test(test_road_ahead test/test_road_ahead.py)
  ament_add_pytest_test(test_stacks test/test_stacks.py)
  ament_add_pytest_test(test_setups test/test_setups.py)
  ament_add_pytest_test(test_adaptive_brake test/test_adaptive_brake.py)
  ament_add_pytest_test(test_lane_map test/test_lane_map.py)
  ament_add_pytest_test(test_stop_law test/test_stop_law.py)
endif()

ament_package()

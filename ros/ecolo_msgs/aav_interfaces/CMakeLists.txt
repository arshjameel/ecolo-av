cmake_minimum_required(VERSION 3.10)
project(aav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleStatus.msg"
  "msg/SensorSource.msg"
  "msg/DataSource.msg"
  "msg/SourceCheck.msg"
  "msg/ActuatorState.msg"
  "msg/WheelSpeeds.msg"
  "msg/SpineOutputs.msg"
  "msg/Heartbeat.msg"
  "msg/RcSticks.msg"
  "msg/ActuatorRaw.msg"
  "msg/SteeringCalibration.msg"
  "msg/LinkFrame.msg"
  "msg/UplinkHealth.msg"
  "msg/StopPoint.msg"
  "msg/RoadAhead.msg"
  "msg/Thing.msg"
  "msg/Things.msg"
  "action/ScriptedDrive.action"
  "action/DriveToPoint.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

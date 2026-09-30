cmake_minimum_required(VERSION 3.16)
project(drone_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionState.msg"
  "msg/ActiveConfig.msg"
  "msg/PayloadState.msg"
  "msg/NodeHealth.msg"
  "msg/HealthStatus.msg"
  "msg/EnergyEstimate.msg"
  "srv/MissionCommand.srv"
  "srv/ConfirmDrop.srv"
  "action/DropPayload.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

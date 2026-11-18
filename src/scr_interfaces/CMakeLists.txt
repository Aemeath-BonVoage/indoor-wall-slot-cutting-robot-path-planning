cmake_minimum_required(VERSION 3.8)
project(scr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPoint.msg"
  "msg/StationRecord.msg"
  "msg/TaskStatus.msg"
  "action/FineAlign.action"
  "action/MoveToStation.action"
  "action/CutStation.action"
  "action/HoldBase.action"
  "action/ScanMarker.action"
  "action/RetractArm.action"
  "action/ExecuteCuttingTask.action"
  "action/ExecuteSlotPlan.action"
  DEPENDENCIES builtin_interfaces geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

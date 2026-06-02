cmake_minimum_required(VERSION 3.8)
project(pick_place_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickObject.action"
  "action/PlaceAt.action"
  "srv/AbortTask.srv"
  "srv/TriggerScan.srv"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

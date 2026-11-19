cmake_minimum_required(VERSION 3.16)
project(cbf_safety_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneEstimate.msg"
  "msg/SafetyStatus.msg"
  "srv/SetSafetyMode.srv"
  "srv/StartIntrusion.srv"
  "srv/InferAction.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

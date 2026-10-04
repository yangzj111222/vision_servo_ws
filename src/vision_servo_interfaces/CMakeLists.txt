cmake_minimum_required(VERSION 3.8)
project(vision_servo_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetFeatures.msg"
  "msg/AlignStatus.msg"
  "msg/GrabCommand.msg"
  "msg/GrabStatus.msg"
  "msg/GrabEvent.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
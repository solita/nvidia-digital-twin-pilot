cmake_minimum_required(VERSION 3.8)
project(warehouse_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskAssignment.msg"
  "msg/TaskStatus.msg"
  "msg/ForkliftStatus.msg"
  "srv/ResetSimulation.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()

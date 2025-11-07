cmake_minimum_required(VERSION 3.8)
project(uav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Khai báo file giao diện
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetGlobal.msg"
  "msg/UserCmd.msg"
  "msg/EventUav.msg"

)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

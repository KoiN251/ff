cmake_minimum_required(VERSION 3.8)
project(control_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uav_msgs REQUIRED)


add_executable(uav_gps_setpoint src/uav_gps_setpoint.cpp)
ament_target_dependencies(uav_gps_setpoint px4_msgs rclcpp uav_msgs)


install(TARGETS
  uav_gps_setpoint
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

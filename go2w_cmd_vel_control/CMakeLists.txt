cmake_minimum_required(VERSION 3.8)
project(go2w_cmd_vel_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(go2w_cmd_vel_control_node src/go2w_cmd_vel_control_node.cpp)
ament_target_dependencies(
  go2w_cmd_vel_control_node
  rclcpp
  geometry_msgs
  unitree_api
)

install(TARGETS
  go2w_cmd_vel_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

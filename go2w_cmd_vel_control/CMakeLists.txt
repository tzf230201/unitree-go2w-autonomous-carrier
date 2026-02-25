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

# Allow building even when the Unitree overlay is not sourced in the shell.
set(_unitree_overlay_prefix "$ENV{HOME}/unitree_ros2/cyclonedds_ws/install")
if (EXISTS "${_unitree_overlay_prefix}/unitree_api/share/unitree_api/cmake/unitree_apiConfig.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${_unitree_overlay_prefix}")
endif()

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

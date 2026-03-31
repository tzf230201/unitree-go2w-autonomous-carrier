cmake_minimum_required(VERSION 3.8)
project(go2w_lio_sam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(hesai_to_liosam_converter
  src/hesai_to_liosam_converter.cpp
)
ament_target_dependencies(hesai_to_liosam_converter
  rclcpp
  sensor_msgs
)

install(TARGETS hesai_to_liosam_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

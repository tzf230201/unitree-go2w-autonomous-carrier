cmake_minimum_required(VERSION 3.8)
project(go2w_lio_sam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(hesai_to_liosam_converter
  src/hesai_to_liosam_converter.cpp
)
ament_target_dependencies(hesai_to_liosam_converter
  rclcpp
  sensor_msgs
)

add_executable(lowstate_to_lio_imu
  src/lowstate_to_lio_imu.cpp
)
ament_target_dependencies(lowstate_to_lio_imu
  rclcpp
  sensor_msgs
  tf2
  unitree_go
)

install(TARGETS
  hesai_to_liosam_converter
  lowstate_to_lio_imu
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

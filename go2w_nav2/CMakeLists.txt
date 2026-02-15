cmake_minimum_required(VERSION 3.8)
project(go2w_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)

project(go2w_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config dae launch stl urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

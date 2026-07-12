cmake_minimum_required(VERSION 3.8)
project(om6dof_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
ament_package()

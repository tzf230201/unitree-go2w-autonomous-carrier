cmake_minimum_required(VERSION 3.8)
project(om_chain_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

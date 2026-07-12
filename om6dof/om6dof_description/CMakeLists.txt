cmake_minimum_required(VERSION 3.8)
project(om6dof_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()

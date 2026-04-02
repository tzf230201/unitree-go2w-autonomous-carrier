cmake_minimum_required(VERSION 3.8)

project(go2w_description_modified)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch stl xacro
  DESTINATION share/${PROJECT_NAME}
)

# Install original URDF and meshes from unitree_ros (avoids modifying that package)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_ros/robots/go2w_description/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_ros/robots/go2w_description/dae/
  DESTINATION share/${PROJECT_NAME}/dae
)

install(
  PROGRAMS scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

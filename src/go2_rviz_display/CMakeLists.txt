cmake_minimum_required(VERSION 3.5)
project(go2_rviz_display)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(sensor_msgs REQUIRED)

# unitree_goConfig.cmake in this SDK is empty, so resolve headers manually.
set(_unitree_go_include_dir "")

# Prefer build-generated headers (non-empty).
if (EXISTS "$ENV{HOME}/go2w_ws/build/unitree_go/rosidl_generator_cpp/unitree_go/msg/low_cmd.hpp")
  set(_unitree_go_include_dir "$ENV{HOME}/go2w_ws/build/unitree_go/rosidl_generator_cpp")
elseif (EXISTS "$ENV{HOME}/go2w_ws/unitree_ros2/build/unitree_go/rosidl_generator_cpp/unitree_go/msg/low_cmd.hpp")
  set(_unitree_go_include_dir "$ENV{HOME}/go2w_ws/unitree_ros2/build/unitree_go/rosidl_generator_cpp")
endif()

# Fall back to CMAKE_PREFIX_PATH.
if (NOT _unitree_go_include_dir)
  set(_prefix_list "${CMAKE_PREFIX_PATH}")
  if (NOT _prefix_list AND DEFINED ENV{CMAKE_PREFIX_PATH})
    string(REPLACE ":" ";" _prefix_list "$ENV{CMAKE_PREFIX_PATH}")
  endif()
  foreach(_prefix ${_prefix_list})
    if (EXISTS "${_prefix}/include/unitree_go/msg/low_cmd.hpp")
      set(_unitree_go_include_dir "${_prefix}/include")
      break()
    endif()
  endforeach()
endif()

# Final fallback to installed path.
if (NOT _unitree_go_include_dir)
  if (EXISTS "$ENV{HOME}/go2w_ws/unitree_ros2/install/unitree_go/include/unitree_go/msg/low_cmd.hpp")
    set(_unitree_go_include_dir "$ENV{HOME}/go2w_ws/unitree_ros2/install/unitree_go/include")
  endif()
endif()

if (NOT _unitree_go_include_dir)
  message(FATAL_ERROR "unitree_go headers not found. Source the workspace that installs unitree_go before building.")
endif()

# Resolve unitree_go C++ typesupport library for linking.
set(_unitree_go_typesupport_lib "")
if (EXISTS "$ENV{HOME}/go2w_ws/build/unitree_go/libunitree_go__rosidl_typesupport_cpp.so")
  set(_unitree_go_typesupport_lib "$ENV{HOME}/go2w_ws/build/unitree_go/libunitree_go__rosidl_typesupport_cpp.so")
elseif (EXISTS "$ENV{HOME}/go2w_ws/unitree_ros2/build/unitree_go/libunitree_go__rosidl_typesupport_cpp.so")
  set(_unitree_go_typesupport_lib "$ENV{HOME}/go2w_ws/unitree_ros2/build/unitree_go/libunitree_go__rosidl_typesupport_cpp.so")
else()
  # Fall back to CMAKE_PREFIX_PATH for installed library.
  set(_prefix_list "${CMAKE_PREFIX_PATH}")
  if (NOT _prefix_list AND DEFINED ENV{CMAKE_PREFIX_PATH})
    string(REPLACE ":" ";" _prefix_list "$ENV{CMAKE_PREFIX_PATH}")
  endif()
  foreach(_prefix ${_prefix_list})
    if (EXISTS "${_prefix}/lib/libunitree_go__rosidl_typesupport_cpp.so")
      set(_unitree_go_typesupport_lib "${_prefix}/lib/libunitree_go__rosidl_typesupport_cpp.so")
      break()
    endif()
  endforeach()
endif()

if (NOT _unitree_go_typesupport_lib)
  message(FATAL_ERROR "unitree_go typesupport library not found. Build unitree_go or check install path.")
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${_unitree_go_include_dir}
  ${unitree_go_INCLUDE_DIRS}
  ${unitree_api_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

link_directories(src)


add_library(${PROJECT_NAME}
  src/Go2JointStatePublisher.cpp
  src/motor_crc.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${_unitree_go_typesupport_lib}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  unitree_go
  unitree_api
  sensor_msgs
)

# Executable
add_executable(${PROJECT_NAME}_node src/go2_joints_state_publisher_node.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${_unitree_go_typesupport_lib}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  unitree_go
  unitree_api
  sensor_msgs
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch urdf dae rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

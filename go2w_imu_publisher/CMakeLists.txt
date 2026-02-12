cmake_minimum_required(VERSION 3.5)
project(go2w_imu_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

# unitree_goConfig.cmake in this SDK can be empty, so resolve headers manually.
set(_unitree_go_include_dir "")

if (EXISTS "$ENV{HOME}/go2w_ws/build/unitree_go/rosidl_generator_cpp/unitree_go/msg/low_state.hpp")
  set(_unitree_go_include_dir "$ENV{HOME}/go2w_ws/build/unitree_go/rosidl_generator_cpp")
elseif (EXISTS "$ENV{HOME}/go2w_ws/unitree_ros2/build/unitree_go/rosidl_generator_cpp/unitree_go/msg/low_state.hpp")
  set(_unitree_go_include_dir "$ENV{HOME}/go2w_ws/unitree_ros2/build/unitree_go/rosidl_generator_cpp")
endif()

if (NOT _unitree_go_include_dir)
  set(_prefix_list "${CMAKE_PREFIX_PATH}")
  if (NOT _prefix_list AND DEFINED ENV{CMAKE_PREFIX_PATH})
    string(REPLACE ":" ";" _prefix_list "$ENV{CMAKE_PREFIX_PATH}")
  endif()
  foreach(_prefix ${_prefix_list})
    if (EXISTS "${_prefix}/include/unitree_go/msg/low_state.hpp")
      set(_unitree_go_include_dir "${_prefix}/include")
      break()
    endif()
  endforeach()
endif()

if (NOT _unitree_go_include_dir)
  if (EXISTS "$ENV{HOME}/go2w_ws/unitree_ros2/install/unitree_go/include/unitree_go/msg/low_state.hpp")
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
  ${_unitree_go_include_dir}
)

add_executable(go2w_imu_publisher_node src/go2w_imu_publisher_node.cpp)
target_link_libraries(go2w_imu_publisher_node ${_unitree_go_typesupport_lib})

ament_target_dependencies(go2w_imu_publisher_node
  rclcpp
  sensor_msgs
  unitree_go
  unitree_api
)

install(TARGETS
  go2w_imu_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

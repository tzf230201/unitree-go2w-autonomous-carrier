cmake_minimum_required(VERSION 3.16)
project(ddgng_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# The GNG core has a handful of immediate-mode GL calls inside helper functions
# (e.g. the arrow drawing in calc_node_normal_vector, guarded by showArrow).
# We never trigger them at runtime, but the symbols must resolve at link time.
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_library(ddgng SHARED
  core/gng.cpp
  core/projection.cpp
  core/rnd.cpp
  core/malloc.cpp
  core/ddgng_wrapper.cpp)

# dDOUBLE keeps the floating-point typedefs consistent with the original build.
target_compile_definitions(ddgng PRIVATE dDOUBLE)
target_include_directories(ddgng PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_link_libraries(ddgng PRIVATE OpenGL::GL OpenGL::GLU GLUT::GLUT)

# Silence the wall of warnings from this legacy code so real errors stand out.
target_compile_options(ddgng PRIVATE -w)

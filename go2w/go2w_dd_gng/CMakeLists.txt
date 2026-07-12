cmake_minimum_required(VERSION 3.16)
project(DepthSensor_Buggy LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(ODE_PC REQUIRED ode)
# MSYS2's ode.pc lies about the library name (says -lode, ships libode_double),
# so resolve the real .dll.a ourselves and only borrow include dirs from pkg-config.
find_library(ODE_LIBRARY NAMES ode_double ode REQUIRED HINTS ${ODE_PC_LIBRARY_DIRS})
add_library(ODE::ODE INTERFACE IMPORTED)
target_include_directories(ODE::ODE INTERFACE ${ODE_PC_INCLUDE_DIRS})
target_link_libraries(ODE::ODE INTERFACE ${ODE_LIBRARY})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Vendored drawstuff (ODE 0.16.6) built as a static lib.
set(DS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/drawstuff)
set(DS_SOURCES ${DS_DIR}/src/drawstuff.cpp)
if(WIN32)
  list(APPEND DS_SOURCES ${DS_DIR}/src/windows.cpp)
elseif(APPLE)
  list(APPEND DS_SOURCES ${DS_DIR}/src/osx.cpp)
else()
  list(APPEND DS_SOURCES ${DS_DIR}/src/x11.cpp)
endif()

add_library(drawstuff STATIC ${DS_SOURCES})
target_include_directories(drawstuff
  PUBLIC  ${DS_DIR}/include
  PRIVATE ${DS_DIR}/src)
target_compile_definitions(drawstuff PUBLIC dDOUBLE)
target_link_libraries(drawstuff
  PUBLIC ODE::ODE OpenGL::GL OpenGL::GLU)
if(WIN32)
  target_link_libraries(drawstuff PUBLIC gdi32 winmm user32 comdlg32)
elseif(NOT APPLE)
  find_package(X11 REQUIRED)
  target_link_libraries(drawstuff PUBLIC X11::X11)
endif()

# Main application
set(APP_SOURCES
  DepthSensor_Buggy/main.cpp
  DepthSensor_Buggy/environment.cpp
  DepthSensor_Buggy/extDisplay.cpp
  DepthSensor_Buggy/gng.cpp
  DepthSensor_Buggy/malloc.cpp
  DepthSensor_Buggy/mesh.cpp
  DepthSensor_Buggy/projection.cpp
  DepthSensor_Buggy/rnd.cpp
  DepthSensor_Buggy/surfMatching.cpp)

# Drawstuff's Win32 resources (menu/accelerators/about-dialog) must live in the
# final PE — windres-produced resource objects don't survive being archived
# inside a static library, so the LoadAccelerators call at startup would fail.
if(WIN32)
  list(APPEND APP_SOURCES ${DS_DIR}/src/resources.rc)
endif()

add_executable(DepthSensor_Buggy ${APP_SOURCES})
if(WIN32)
  target_include_directories(DepthSensor_Buggy PRIVATE ${DS_DIR}/src)
endif()
target_compile_definitions(DepthSensor_Buggy PRIVATE dDOUBLE _USE_MATH_DEFINES)
target_link_libraries(DepthSensor_Buggy PRIVATE
  drawstuff ODE::ODE GLUT::GLUT OpenGL::GL OpenGL::GLU)

# Copy textures next to the executable so fn.path_to_textures = "textures" works.
add_custom_command(TARGET DepthSensor_Buggy POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${DS_DIR}/textures $<TARGET_FILE_DIR:DepthSensor_Buggy>/textures)

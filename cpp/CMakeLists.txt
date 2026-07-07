cmake_minimum_required(VERSION 3.16)
project(spacetug_mujoco_cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo: point CMAKE_PREFIX_PATH at an extracted MuJoCo release, e.g.
#   cmake -B build -DCMAKE_PREFIX_PATH=/path/to/mujoco-3.x.y
#
# Some MuJoCo binary releases include headers/libraries but no
# mujocoConfig.cmake. Prefer the package config when present, and otherwise
# create the imported target from the extracted release layout.
find_package(mujoco CONFIG QUIET)

if(NOT mujoco_FOUND)
  find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES include
  )
  find_library(MUJOCO_LIBRARY
    NAMES mujoco
    HINTS ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib bin
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(mujoco
    REQUIRED_VARS MUJOCO_LIBRARY MUJOCO_INCLUDE_DIR
  )

  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
  )
endif()

# GLFW is optional: without it the build is headless-only (parity CSV mode).
find_package(glfw3 QUIET)

add_executable(spacetug src/main.cpp)
target_link_libraries(spacetug PRIVATE mujoco::mujoco)

if(glfw3_FOUND)
  target_link_libraries(spacetug PRIVATE glfw)
  target_compile_definitions(spacetug PRIVATE HAVE_GLFW)
  message(STATUS "GLFW found: building interactive viewer + headless mode")
else()
  message(STATUS "GLFW not found: building headless mode only")
endif()

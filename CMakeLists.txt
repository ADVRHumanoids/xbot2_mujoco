cmake_minimum_required(VERSION 3.15)
project(xbot2_mujoco)

find_package(mujoco_cmake REQUIRED)
find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(DEFINED ENV{CONDA_PREFIX})
  set(CMAKE_INSTALL_PREFIX $ENV{CONDA_PREFIX}/ CACHE PATH "bindings install prefix" FORCE)
#   set(CMAKE_SYSROOT /usr)
#   include_directories(${CMAKE_SYSROOT}/include)
#   include_directories(${CMAKE_SYSROOT}/include/x86_64-linux-gnu)
#   message(STATUS "Using sysroot: $ENV{CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot")
#   set(CMAKE_SYSROOT "$ENV{CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot")
endif()

add_library(xbot2_mujoco STATIC
    src/xbot2_mj_joint.cpp
    src/xbot2_mj_imu.cpp
    src/xbot2_mj_linkstate.cpp
    )

target_link_libraries(xbot2_mujoco
    PUBLIC
    mujoco_cmake::mujoco
    xbot2::xbot2
    matlogger2::matlogger2
    )

add_executable(mujoco_simulator
    src/simulator.cpp
    src/uitools.cpp
    src/control.cpp)

target_link_libraries(mujoco_simulator
    xbot2_mujoco)

install(TARGETS mujoco_simulator
    DESTINATION bin)

install(PROGRAMS src/mujoco_simulator_wrapper.py 
    DESTINATION bin)



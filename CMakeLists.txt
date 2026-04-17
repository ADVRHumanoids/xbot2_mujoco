cmake_minimum_required(VERSION 3.20)
project(xbot2_mujoco)

find_package(mujoco_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)

find_library(EGL_LIBRARY EGL)


set(CMAKE_CXX_STANDARD 17)

find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

add_library(xbot2_mujoco STATIC
    src/xbot2_mj_joint.cpp
    src/xbot2_mj_imu.cpp
    src/xbot2_mj_linkstate.cpp
    )

find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

target_link_libraries(xbot2_mujoco
    PUBLIC
    mujoco_cmake::mujoco
    xbot2::xbot2
    matlogger2::matlogger2
    ${EGL_LIBRARY}
    ${ZeroMQ_LIBRARY}
    )

add_executable(mujoco_simulator
    src/simulator.cpp
    src/simulate.cc
    src/platform_ui_adapter.cc
    src/uitools.cpp
    src/control.cpp
    src/lodepng.h
    src/lodepng.cpp
    src/glfw_adapter.cc
    src/glfw_dispatch.cc)

target_link_libraries(mujoco_simulator
    glfw
    xbot2_mujoco)

install(TARGETS mujoco_simulator
    DESTINATION bin)

install(PROGRAMS src/mujoco_simulator_wrapper.py 
    DESTINATION bin)



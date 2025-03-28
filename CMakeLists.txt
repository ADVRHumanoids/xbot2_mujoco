cmake_minimum_required(VERSION 3.16)
project(xbot2_mujoco)

find_package(mujoco_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)


set(CMAKE_CXX_STANDARD 17)

add_library(xbot2_mujoco STATIC
    src/xbot2_mj_joint.cpp
    src/xbot2_mj_imu.cpp
    src/xbot2_mj_linkstate.cpp
    )

target_link_libraries(xbot2_mujoco
    PUBLIC
    mujoco_cmake::mujoco
    xbot2::xbot2
    matlogger2::matlogger2)

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



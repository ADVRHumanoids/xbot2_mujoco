cmake_minimum_required(VERSION 3.0)
project(xbot2_mujoco)

find_package(mujoco_cmake REQUIRED)
find_package(xbot2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(xbot2_mujoco STATIC
    src/xbot2_mj_joint.cpp
    src/xbot2_mj_imu.cpp
    )

target_link_libraries(xbot2_mujoco
    PUBLIC
    mujoco_cmake::mujoco
    xbot2::xbot2)

add_executable(mujoco_simulator
    src/simulator.cpp
    src/uitools.cpp
    src/control.cpp)

target_link_libraries(mujoco_simulator
    xbot2_mujoco)

install(TARGETS mujoco_simulator
    DESTINATION bin)



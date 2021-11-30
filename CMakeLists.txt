cmake_minimum_required(VERSION 3.0)
project(xbot2_mujoco)

find_package(mujoco_hhcm REQUIRED)

add_executable(mujoco_simulator
    src/simulator.cpp
    src/uitools.cpp)

target_link_libraries(mujoco_simulator
    mujoco_hhcm::mujoco)

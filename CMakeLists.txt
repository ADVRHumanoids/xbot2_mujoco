cmake_minimum_required(VERSION 3.15)
project(xbot2_mujoco)

find_package(mujoco_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(pugixml REQUIRED)
find_package(catkin COMPONENTS roscpp) # necessary for publishing ros clock from sim

set(CMAKE_CXX_STANDARD 17)
# Ensure position-independent code is generated for all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

option(WITH_XMJ_SIM_ENV "Compile XBotMjSimEnv" TRUE)
if(${WITH_XMJ_SIM_ENV})
    message(STATUS "Will compile XBotMjSimEnv.")
endif()

option(WITH_TESTS "Compile tests" FALSE)
if(${WITH_TESTS})
    message(STATUS "Will compile tests.")
endif()

find_package(pybind11 QUIET)
if(pybind11_FOUND)
    option(WITH_PYTHON "Compile and install python bindings" TRUE)
    if(${WITH_PYTHON})
        message(STATUS "Python bindings for will be built and installed.")
        add_definitions(-DWITH_PYTHON) # can be used by preprocessor macros
    endif()
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
target_compile_options(xbot2_mujoco PRIVATE -fPIC)

add_library(mujoco_simulator STATIC
    src/simulator.cpp
    src/simulate.cc
    src/platform_ui_adapter.cc
    src/uitools.cpp
    src/control.cpp
    src/lodepng.h
    src/lodepng.cpp
    src/glfw_adapter.cc
    src/glfw_dispatch.cc)

target_link_libraries(mujoco_simulator PUBLIC
    xbot2_mujoco
    glfw
    ${catkin_LIBRARIES})

# add_library(xbot_mujoco_env STATIC
#     src/xmj_sim_env.cpp)

# target_link_libraries(xbot_mujoco_env PUBLIC
#     mujoco_simulator)

add_library(xmj_loading_utils STATIC
    src/loading_utils.cpp)

target_link_libraries(xmj_loading_utils PUBLIC
    pugixml)

add_executable(launch_simulator src/launch_simulator.cpp)

target_link_libraries(launch_simulator PRIVATE
    mujoco_simulator)

install(TARGETS launch_simulator
    DESTINATION bin)

install(PROGRAMS src/mujoco_simulator_wrapper.py 
    DESTINATION bin)

if(${WITH_TESTS})
    enable_testing()
    add_subdirectory(tests/)
endif()

# Compile python bindings
if(${WITH_PYTHON})
    add_subdirectory(bindings/python/)
endif()


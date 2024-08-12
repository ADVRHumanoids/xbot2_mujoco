cmake_minimum_required(VERSION 3.5)

add_compile_options(-std=c++17)

# Setting default visibility to hidden
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
# Helps in reducing the possibility of symbol clashes and also minimizes
# the chance of accidentally exporting symbols that aren't intended to be
# part of your library's public API

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include/ src/)

find_package(pybind11 REQUIRED)

# find_package(xbot2_mujoco REQUIRED)

message(STATUS "Compiling python bindings for xbot2_mujoco")

# to get PYTHON_SITE
execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
        from distutils import sysconfig as sc
        print(sc.get_python_lib(prefix='', plat_specific=True))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

pybind11_add_module("PyXbotMjSimEnv" PyXbotMjSimEnv.cpp)
target_link_libraries("PyXbotMjSimEnv" PUBLIC pybind11::module)

# install python bindings
install(TARGETS "PyXbotMjSimEnv"
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${PROJECT_NAME})
install(FILES __init__.py DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${PROJECT_NAME})

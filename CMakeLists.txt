cmake_minimum_required(VERSION 3.16)
project(xbot2_mujoco)

# install marker file to make sure we have an install target
set(install_marker "${CMAKE_CURRENT_BINARY_DIR}/xbot2_mujoco.marker")
file(WRITE "${install_marker}" "")
install(FILES "${install_marker}" DESTINATION "share/${PROJECT_NAME}")

add_custom_target(install_python_pkg ALL
    COMMAND python -m pip show xbot2_py_mujoco || python -m pip install -e ${CMAKE_CURRENT_SOURCE_DIR}/python
    COMMENT "Installing Python package in editable mode"
)

enable_testing()

add_test(
    NAME pytest
    COMMAND python -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/python/tests -v
)

set_tests_properties(pytest PROPERTIES
    DEPENDS install_python_pkg
)

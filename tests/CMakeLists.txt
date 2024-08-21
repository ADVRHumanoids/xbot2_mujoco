cmake_minimum_required(VERSION 3.15)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR})

if(${WITH_XMJ_SIM_ENV})
    add_executable(xmj_env_test "test_xmj_env.cpp")
    target_link_libraries(xmj_env_test PRIVATE xbot_mujoco_env xmj_loading_utils ${GTEST_BOTH_LIBRARIES} pthread)
    gtest_discover_tests(xmj_env_test)

endif()

add_executable(parsing_test "test_loading_utils.cpp")
target_link_libraries(parsing_test PRIVATE xmj_loading_utils ${GTEST_BOTH_LIBRARIES} pthread)
# gtest_discover_tests(parsing_test)

# copying python bindings tests to build folder
add_custom_target(copy_py_tests_all ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/PyXBotMjSimEnv"
            "${CMAKE_CURRENT_BINARY_DIR}/PyXBotMjSimEnv"
    COMMENT "Copying PyXBotMjSimEnv python unittests directory to test folder"
)
# ensure this target is always run by setting a phony output
# used to specify that the copied directory should be considered
# for cleaning purposes (i.e., when running make clean)
set_target_properties(copy_py_tests_all PROPERTIES
    ADDITIONAL_CLEAN_FILES "${CMAKE_CURRENT_BINARY_DIR}/PyXBotMjSimEnv"
)

# Define the files directory path
set(iit_centauro_ros_pkg_DIR CACHE STRING "Path to the directory")
if(NOT EXISTS "${iit_centauro_ros_pkg_DIR}")
    message(FATAL_ERROR "Provided iit_centauro_ros_pkg_DIR [${iit_centauro_ros_pkg_DIR}] does not exist!")
else()
    message(STATUS "iit_centauro_ros_pkg_DIR set to folder ${iit_centauro_ros_pkg_DIR}")
endif()

set(FILES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/files")

# define a custom target to copy the 'files' directory
# add_custom_target(copy_parsing_test_files ALL
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#             "${FILES_DIR}"
#             "${CMAKE_CURRENT_BINARY_DIR}/files"
#     COMMENT "Copying 'files' directory to build/test folder"
# )
# # Ensure the custom target is always run by adding it as a dependency of the parsing test
# add_dependencies(parsing_test copy_parsing_test_files)

# Configure the header file with the path
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)
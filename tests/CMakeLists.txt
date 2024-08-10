cmake_minimum_required(VERSION 3.15)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(xmj_env_test "test_xmj_env.cpp")

target_link_libraries(xmj_env_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})
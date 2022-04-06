project(registration_core_test VERSION 1.0.0)

include_directories(
    ${gtest_SOURCE_DIR}/include
    ${gtest_SOURCE_DIR})

add_executable(${PROJECT_NAME}
    registration_core_test.cpp)

target_link_libraries(${PROJECT_NAME}
    gtest
    gtest_main
    registration_core)

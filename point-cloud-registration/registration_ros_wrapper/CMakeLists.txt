cmake_minimum_required(VERSION 3.5)

project(registration)

add_compile_options(-std=c++17)

set(REGISTRATION_PATH ../registration)

find_package(PCL 1.10 REQUIRED)

find_package(CUDA 11.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    registration_msgs)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package()

cuda_add_library(RegistrationCuda STATIC ${REGISTRATION_PATH}/src/cuda.cu)

include_directories(
    ${REGISTRATION_PATH}/include SYSTEM
    include SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
        src/registration_node.cpp
        ${REGISTRATION_PATH}/src/math.cpp
        ${REGISTRATION_PATH}/src/registration.cpp
        ${REGISTRATION_PATH}/src/set.cpp
        ${REGISTRATION_PATH}/src/base.cpp
        ${REGISTRATION_PATH}/src/octo_map.cpp
        ${REGISTRATION_PATH}/include/registration/timer.h
        src/registration_api.cpp
        src/helper.cpp)

add_dependencies(${PROJECT_NAME}_node
    RegistrationCuda)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    RegistrationCuda)

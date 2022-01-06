cmake_minimum_required(VERSION 3.5)

project(registration)

add_compile_options(-std=c++17)

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

cuda_add_library(RegistrationCuda STATIC src/cuda.cu)

include_directories(
    include SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/registration_node.cpp
    src/helper.cpp
    src/rc4pcs.cpp
    src/set.cpp
    src/base.cpp
    src/octo_map.cpp
    include/registration/timer.h)

add_dependencies(${PROJECT_NAME}_node
    RegistrationCuda)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    RegistrationCuda)

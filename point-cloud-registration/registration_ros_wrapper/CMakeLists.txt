cmake_minimum_required(VERSION 3.5)

project(registration_ros_wrapper)

add_compile_options(-std=c++17)

find_package(PCL 1.10 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    registration_core
    registration_msgs)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
    include SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/registration_node.cpp
    src/registration_api.cpp
    src/helper.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

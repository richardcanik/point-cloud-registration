cmake_minimum_required(VERSION 3.5)

project(localization)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    localization_msgs)

find_package(PCL 1.10 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
    include SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/localization_node.cpp
        src/helper.cpp
    src/rc4pcs.cpp
    src/set.cpp
    src/base.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

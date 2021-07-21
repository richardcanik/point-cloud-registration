cmake_minimum_required(VERSION 3.5)

project(localization_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)

add_service_files(
  FILES
  String.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE src/empty.cpp)
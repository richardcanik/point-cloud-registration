cmake_minimum_required(VERSION 3.5)

project(registration_core VERSION 1.0.0)

add_compile_options(-std=c++17)

find_package(Eigen3 3.3 REQUIRED)

find_package(CUDA 11.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

cuda_add_library(${PROJECT_NAME}_cuda
    src/cuda.cu)

add_library(${PROJECT_NAME}
    src/base.cpp
    src/math.cpp
    src/octo_map.cpp
    src/registration.cpp
    src/set.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PROJECT_NAME}_cuda)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()

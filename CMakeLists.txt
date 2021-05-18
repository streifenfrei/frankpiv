cmake_minimum_required(VERSION 3.17)
project(frankpiv)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

add_subdirectory(dependencies)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        message_runtime
        std_msgs
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
)

add_library(frankpiv SHARED
        src/general_backend.cpp
        )

set(frankx_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/frankx/include")
target_include_directories(frankpiv PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(frankpiv PUBLIC Eigen3::Eigen YAML_CPP roscpp frankr frankx)

add_executable(example examples/frankx_example.cpp)
target_link_libraries(example frankpiv frankx)
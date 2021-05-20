cmake_minimum_required(VERSION 3.16)
project(frankpiv)

set(CMAKE_CXX_STANDARD 17)
add_definitions("-fPIC")

# DEPENDENCIES
option(FRANKR "Build frankr backend" OFF)
option(FRANKX "Build frankx backend" OFF)
set(BACKENDS "")
if (FRANKR)
    list(APPEND BACKENDS frankr)
    add_compile_definitions(FRANKR)
endif ()
if (FRANKX)
    list(APPEND BACKENDS frankx)
    add_compile_definitions(FRANKX)
endif ()
add_subdirectory(dependencies)

find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)

# CATKIN
set(CATKIN_ENABLE_TESTING OFF)
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        message_runtime
        std_msgs
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
)

# FRANKPIV
set(BACKEND_SOURCES "")
foreach (BACKEND ${BACKENDS})
    list(APPEND BACKEND_SOURCES src/${BACKEND}_backend.cpp)
endforeach ()
add_library(
        ${PROJECT_NAME} SHARED
        src/controller.cpp
        src/general_backend.cpp
        src/frankx_backend.cpp
        src/frankr_backend.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp ${catkin_LIBRARIES} ${BACKENDS})

# EXAMPLE
add_executable(example examples/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
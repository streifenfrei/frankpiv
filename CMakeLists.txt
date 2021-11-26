cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 17)
project(frankpiv)

add_definitions("-fPIC")

# DEPENDENCIES
option(FRANKR "Build frankr backend" OFF)
option(FRANKX "Build frankx backend" OFF)
option(PYTHON_BINDING "Build python module. Requires pybind11" ON)

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
find_package(Boost REQUIRED COMPONENTS thread)
find_package(ompl REQUIRED) # required for the moveit_planner


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
find_package(rviz_visual_tools)
if (rviz_visual_tools_FOUND)
    add_compile_definitions(VISUALIZATION)
    list(APPEND catkin_INCLUDE_DIRS ${rviz_visual_tools_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${rviz_visual_tools_LIBRARIES})
endif ()

if (PYTHON_BINDING)
    find_package(pybind11 REQUIRED)
    catkin_python_setup()
endif ()

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        DEPENDS
        EIGEN3
        Boost
)


# FRANKPIV
set(BACKEND_SOURCES "")
foreach (BACKEND ${BACKENDS})
    list(APPEND BACKEND_SOURCES src/frankpiv/native/${BACKEND}_backend.cpp)
endforeach ()
add_library(
        ${PROJECT_NAME} SHARED
        src/frankpiv/native/controller.cpp
        src/frankpiv/native/utilities.cpp
        src/frankpiv/native/pivot_frame.cpp
        src/frankpiv/native/general_backend.cpp
        src/frankpiv/native/moveit_backend.cpp
        ${BACKEND_SOURCES}
        src/frankpiv/native/moveit_planner/state_sampler.cpp
        src/frankpiv/native/moveit_planner/validity_checker.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp ${catkin_LIBRARIES} ${BACKENDS} ${OMPL_LIBRARIES})

# EXAMPLE
add_executable(example examples/example.cpp)
target_link_libraries(example ${PROJECT_NAME})

# PYTHON
if (PYTHON_BINDING)
    pybind11_add_module(frankpyv src/frankpiv/native/python.cpp)
    target_link_libraries(frankpyv PUBLIC frankpiv)
    set_target_properties(frankpyv PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
endif ()

# post build script
if (PYTHON_BINDING)
    set(PYTHON_BINDING true)
else ()
    set(PYTHON_BINDING false)
endif ()
configure_file(post_build.sh ${CMAKE_CURRENT_BINARY_DIR}/post_build_ready.sh FILE_PERMISSIONS OWNER_READ OWNER_EXECUTE @ONLY)
add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/post_build_ready.sh)

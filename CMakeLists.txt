cmake_minimum_required(VERSION 3.16)
project(frankpiv)

set(CMAKE_CXX_STANDARD 17)
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
if (PYTHON_BINDING)
    find_package(pybind11 REQUIRED)
endif()

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

catkin_python_setup()

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
    list(APPEND BACKEND_SOURCES src/frankpiv/native/${BACKEND}_backend.cpp)
endforeach ()
add_library(
        ${PROJECT_NAME} SHARED
        src/frankpiv/native/controller.cpp
        src/frankpiv/native/general_backend.cpp
        src/frankpiv/native/utilities.cpp
        src/frankpiv/native/moveit_backend.cpp
        ${BACKEND_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp ${catkin_LIBRARIES} ${BACKENDS})

# EXAMPLE
add_executable(example examples/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
pybind11_add_module(frankpyv src/frankpiv/native/python.cpp)
target_link_libraries(frankpyv PUBLIC frankpiv)
set_target_properties(frankpyv PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frankpiv/default_config.yml
        ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
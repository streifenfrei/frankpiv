add_subdirectory(yaml-cpp)

set(BUILD_TESTS OFF)
set(BUILD_PYTHON_MODULE OFF)
set(BUILD_EXAMPLES OFF)
if (FRANKR)
    add_subdirectory(frankr/frankr)
    target_include_directories(frankr frankr/frankr/include)
endif()
if (FRANKX)
    add_subdirectory(frankx)
endif()
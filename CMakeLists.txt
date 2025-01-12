cmake_minimum_required(VERSION 3.16)

project(srukf VERSION 0.0.1)

add_subdirectory(lib)

set(BUILD_RUNNER true CACHE BOOL "Build dev runner")
set(BUILD_SKRUF_TESTING true CACHE BOOL "Include testing directory")

if (BUILD_RUNNER)
    add_subdirectory(runner)
endif ()

include(CTest)
if (BUILD_SKRUF_TESTING)
    add_subdirectory(tests)
endif ()

cmake_minimum_required(VERSION 3.16)

project(srukf VERSION 0.0.1)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0135 NEW)


set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

add_subdirectory(lib)

set(BUILD_RUNNER true)

if (BUILD_RUNNER)
    add_subdirectory(runner)
endif ()

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif ()



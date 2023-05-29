project(srukf_runner)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

add_executable(UKF_runner main.cpp)

include_directories(lib)
add_subdirectory(lib)

target_link_libraries(UKF_runner srukf)
target_compile_options(UKF_runner PRIVATE -Wall -Wextra -Wpedantic -Werror)


include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif ()

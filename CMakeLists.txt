project(srukf)

cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(srukf
        src/ukf.cpp
        src/operations.cpp
        src/noise.cpp
        src/transitions.cpp
        src/parameters.cpp include/macros.hpp)

target_link_libraries(srukf Eigen3::Eigen)

target_compile_options(srukf PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
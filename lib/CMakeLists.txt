
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED NO_MODULE)
include(FetchContent)

FetchContent_Declare(
        optional
        GIT_REPOSITORY https://github.com/TartanLlama/optional.git
        GIT_TAG v1.1.0
)

FetchContent_MakeAvailable(optional)

add_library(srukf INTERFACE
        include/core/field_base.h
        include/core/typedefs.h)

message(${optional_SOURCE_DIR})


target_include_directories(srukf INTERFACE include INTERFACE ${optional_SOURCE_DIR}/include)
target_link_libraries(srukf INTERFACE Eigen3::Eigen)
#target_compile_options(srukf INTERFACE -Wall -Wextra -Wpedantic -Werror)

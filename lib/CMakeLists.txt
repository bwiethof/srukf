
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED NO_MODULE)
include(FetchContent)


add_library(srukf INTERFACE
        include/core/field_base.h
        include/core/typedefs.h)


target_include_directories(srukf INTERFACE include INTERFACE ${optional_SOURCE_DIR}/include)
target_link_libraries(srukf INTERFACE Eigen3::Eigen)
#target_compile_options(srukf INTERFACE -Wall -Wextra -Wpedantic -Werror)

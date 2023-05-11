project(srukf)

cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(srukf
        src/ukf.cpp
        src/operations.cpp
        src/noise.cpp
        src/transitions.cpp
        src/parameters.cpp
        include/core/util/macros.hpp
        include/base/sensor_data.h
        include/slam/sensor_data.h
        include/base/sensor.hpp
        include/core/util/traits.hpp
        include/core/util/fwd_declarations.h
        include/slam/sensor.hpp
        include/core/sensor.hpp
        include/core/util/statics.h
        include/core/util/lookups.h
        include/core/util/functor.hpp)

target_include_directories(srukf PUBLIC include)
target_link_libraries(srukf Eigen3::Eigen)
target_compile_options(srukf PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(CTest)

if (BUILD_TESTING)
    add_subdirectory(tests)
endif ()

add_executable(UKF_runner src/main.cpp)
target_link_libraries(UKF_runner srukf)
target_compile_options(UKF_runner PRIVATE -Wall -Wextra -Wpedantic -Werror)

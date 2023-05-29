project(srukf)

cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED NO_MODULE)

set(HEADER_FILES
        include/core/util/macros.hpp
        include/base/sensor_data.h
        include/base/base_sensor.hpp
        include/slam/sensor_data.h
        include/slam/sensor.hpp
        include/core/util/traits.hpp
        include/core/util/fwd_declarations.h
        include/core/sensor.hpp
        include/core/util/statics.h
        include/core/util/lookups.h
        include/core/util/functor.hpp
        include/core/helper.hpp
        )

set(SOURCE_FILES
        src/ukf.cpp
        src/operations.cpp
        src/noise.cpp
        src/transitions.cpp
        src/parameters.cpp
        )

add_library(srukf
        ${SOURCE_FILES}
        ${HEADER_FILES}
        )

target_include_directories(srukf PUBLIC include)
target_link_libraries(srukf Eigen3::Eigen)
target_compile_options(srukf PRIVATE -Wall -Wextra -Wpedantic -Werror)

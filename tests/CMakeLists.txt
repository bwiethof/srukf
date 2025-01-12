project(ukf_tests)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


set(SOURCE_FILES core/parameters_test.cpp
        core/math/transformation_test.cpp
        core/detail/functor_test.cpp
        core/detail/transform_test.cpp
        core/detail/ukf_impl_test.cpp
        core/field_test.cpp
        core/sensor.test.cpp
        core/sensor_data_test.cpp
        core/state_test.cpp
        core/covariance_test.cpp
        core/ukf_test.cpp
)

add_executable(SRUKF_Test ${SOURCE_FILES})

target_compile_features(SRUKF_Test PRIVATE cxx_std_20)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} .)

target_link_libraries(SRUKF_Test srukf)

target_link_libraries(SRUKF_Test GTest::gtest GTest::gmock GTest::gtest_main)

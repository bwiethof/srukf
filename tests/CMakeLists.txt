project(ukf_tests)

set(CMAKE_CXX_STANDARD 14)


find_package(GTest)

set(SOURCE_FILES core/parameters_test.cpp
        core/math/transformation_test.cpp
        core/detail/functor_test.cpp
        core/detail/transform_test.cpp
        core/detail/ukf_impl_test.cpp
        core/field_test.cpp
)

add_executable(SRUKF_Test ${SOURCE_FILES})

target_compile_features(SRUKF_Test PRIVATE cxx_std_20)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} .)

target_link_libraries(SRUKF_Test srukf)

target_link_libraries(SRUKF_Test GTest::gtest GTest::gmock GTest::gtest_main)

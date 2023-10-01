project(ukf_tests)

set(CMAKE_CXX_STANDARD 14)


find_package(GTest)

set(SOURCE_FILES core/parameters_test.cpp
        math/transformation_test.cpp)

add_executable(SRUKF_Test ${SOURCE_FILES})

target_compile_features(SRUKF_Test PRIVATE cxx_std_20)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} .)

target_link_libraries(SRUKF_Test srukf)

target_link_libraries(SRUKF_Test GTest::gtest GTest::gtest_main)

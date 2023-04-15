project(ukf_tests)

set(CMAKE_CXX_STANDARD 14)


find_package(GTest)


add_executable(SRUKF_Test parametersTest.cpp transitionsTest.cpp sensorDataTest.cpp noiseTest.cpp testUtilities.hpp)

target_compile_features(SRUKF_Test PRIVATE cxx_std_20)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

target_link_libraries(SRUKF_Test GTest::gtest GTest::gtest_main srukf)

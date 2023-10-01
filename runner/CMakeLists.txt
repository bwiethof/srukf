
set(CMAKE_CXX_STANDARD 14)


include_directories(../lib)
add_executable(UKF_runner main.cpp)
target_link_libraries(UKF_runner srukf)

target_compile_options(UKF_runner PRIVATE -Wall -Wextra -Wpedantic -Werror)

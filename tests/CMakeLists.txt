cmake_minimum_required(VERSION 3.13)
project(ddt_can_bridge_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(unit_tests
    test_main.cpp
    ../firmware/src/bridge_controller.cpp
    ../firmware/src/motor_protocol.cpp
    ../firmware/src/pc_protocol.cpp
)
target_include_directories(unit_tests PRIVATE ../firmware/src)
target_compile_options(unit_tests PRIVATE -Wall -Wextra -fsanitize=address,undefined)
target_link_options(unit_tests PRIVATE -fsanitize=address,undefined)
add_test(NAME unit_tests COMMAND unit_tests)

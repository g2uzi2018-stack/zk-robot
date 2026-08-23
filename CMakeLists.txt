cmake_minimum_required(VERSION 3.16)

project(robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(robot
    main.cpp
    src/can/socket_can.cpp
    src/can/can_interface_manager.cpp
    src/ti5/hand/aoyi_protocol.cpp
    src/ti5/hand/hand_config.cpp
    src/ti5/hand/hand_discovery.cpp
)

add_subdirectory(src/tiago)
add_subdirectory(src/ti5)

target_include_directories(robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(robot PRIVATE
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
)

# Hardware-in-the-loop joint test menu. It is deliberately excluded from the
# default build. Building and running it must both be explicit.
add_executable(ti5_zero_home EXCLUDE_FROM_ALL
    tests/ti5_zero_home_test.cpp
    src/can/socket_can.cpp
    src/can/can_interface_manager.cpp
    src/ti5/can/can_protocol.cpp
    src/ti5/can/encoder_conversion.cpp
    src/ti5/can/can_discovery.cpp
    src/ti5/can/can_bus.cpp
    src/ti5/motor/can_motor.cpp
)

target_include_directories(ti5_zero_home PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ti5_zero_home PRIVATE
    ti5_config
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_zero_home PRIVATE
    TI5_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

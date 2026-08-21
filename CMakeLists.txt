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

include(CTest)

if(BUILD_TESTING)
    add_executable(ti5_can_tests
        tests/ti5_can_protocol_test.cpp
        src/ti5/can/can_protocol.cpp
        src/ti5/can/encoder_conversion.cpp
    )

    target_include_directories(ti5_can_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    add_test(NAME ti5_can_tests COMMAND ti5_can_tests)
endif()

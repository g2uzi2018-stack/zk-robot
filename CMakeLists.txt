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

target_include_directories(robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(robot PRIVATE
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
)
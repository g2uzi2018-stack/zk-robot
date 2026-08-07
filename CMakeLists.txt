cmake_minimum_required(VERSION 3.16)

project(robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(yaml-cpp REQUIRED)

add_executable(robot main.cpp src/tiago/can/can_config.cpp src/tiago/can/can_protocol.cpp)

target_include_directories(robot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(robot PRIVATE yaml-cpp)

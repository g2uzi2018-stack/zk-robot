cmake_minimum_required(VERSION 3.16)

project(robot LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

# 与具体机器人型号无关的 Linux SocketCAN 公共层。
add_library(robot_can STATIC
    src/can/socket_can.cpp
    src/can/can_interface_manager.cpp
)

target_include_directories(robot_can PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(robot
    main.cpp
)

add_subdirectory(src/tiago)
add_subdirectory(src/ti5)
add_subdirectory(tools)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

target_include_directories(robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(robot PRIVATE
    robot_can
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
)

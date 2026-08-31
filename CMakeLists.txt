cmake_minimum_required(VERSION 3.16)

project(robot LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 与具体机器人型号无关的 Linux SocketCAN 公共层。
add_library(robot_can STATIC
    src/can/socket_can.cpp
    src/can/can_interface_manager.cpp
)

target_include_directories(robot_can PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_subdirectory(src/motion)
add_subdirectory(src/input)
add_subdirectory(src/tiago)
add_subdirectory(src/ti5)
add_subdirectory(tools)

add_executable(robot
    main.cpp
)

target_include_directories(robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(robot PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(robot PRIVATE
    ti5_config
    ti5_hand_transport
)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

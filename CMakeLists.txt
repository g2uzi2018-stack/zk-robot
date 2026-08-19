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
    src/tiago/can/can_config.cpp
    src/tiago/can/can_protocol.cpp
    src/tiago/can/encoder_conversion.cpp
    src/tiago/motor/can_motor.cpp
    src/tiago/can/can_bus.cpp
    src/tiago/joint/joint.cpp
    src/tiago/arm/arm.cpp
    src/tiago/controller/arm_controller.cpp
    src/tiago/gripper/gripper.cpp
    src/tiago/controller/gripper_controller.cpp
    src/tiago/head/head.cpp
    src/tiago/controller/head_controller.cpp
    src/tiago/torso/torso.cpp
    src/tiago/controller/torso_controller.cpp
    src/tiago/base/base_config.cpp
    src/tiago/base/base.cpp
    src/tiago/controller/base_controller.cpp
    src/tiago/executor/robot_control_executor.cpp
    src/tiago/executor/robot_control_executor_arm.cpp
    src/tiago/executor/robot_control_executor_components.cpp
)

target_include_directories(robot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(robot PRIVATE yaml-cpp spdlog::spdlog fmt::fmt)

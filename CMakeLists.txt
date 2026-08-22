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

    add_executable(ti5_can_motor_tests
        tests/ti5_can_motor_test.cpp
        src/ti5/can/can_bus.cpp
        src/ti5/motor/can_motor.cpp
        src/ti5/can/can_protocol.cpp
        src/ti5/can/encoder_conversion.cpp
        src/can/socket_can.cpp
    )

    target_include_directories(ti5_can_motor_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    add_test(NAME ti5_can_motor_tests COMMAND ti5_can_motor_tests)

    add_executable(ti5_config_tests
        tests/ti5_config_loader_test.cpp
    )

    target_link_libraries(ti5_config_tests PRIVATE
        ti5_config
    )

    target_include_directories(ti5_config_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_compile_definitions(ti5_config_tests PRIVATE
        TI5_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    add_test(NAME ti5_config_tests COMMAND ti5_config_tests)


    add_executable(aoyi_protocol_tests
        tests/aoyi_protocol_test.cpp
        src/ti5/hand/aoyi_protocol.cpp
    )

    target_include_directories(aoyi_protocol_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    add_test(NAME aoyi_protocol_tests COMMAND aoyi_protocol_tests)

    add_executable(aoyi_hand_discovery_tests
        tests/aoyi_hand_discovery_test.cpp
        src/ti5/hand/aoyi_protocol.cpp
        src/ti5/hand/hand_discovery.cpp
        src/can/socket_can.cpp
    )

    target_include_directories(aoyi_hand_discovery_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(aoyi_hand_discovery_tests PRIVATE
        spdlog::spdlog
        fmt::fmt
    )

    add_test(
        NAME aoyi_hand_discovery_tests
        COMMAND aoyi_hand_discovery_tests
    )

    add_executable(aoyi_hand_config_tests
        tests/aoyi_hand_config_test.cpp
        src/ti5/hand/hand_config.cpp
    )

    target_include_directories(aoyi_hand_config_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(aoyi_hand_config_tests PRIVATE
        yaml-cpp
    )

    target_compile_definitions(aoyi_hand_config_tests PRIVATE
        TI5_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    add_test(NAME aoyi_hand_config_tests COMMAND aoyi_hand_config_tests)

    add_executable(can_interface_manager_tests
        tests/can_interface_manager_test.cpp
        src/can/can_interface_manager.cpp
    )

    target_include_directories(can_interface_manager_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    add_test(NAME can_interface_manager_tests COMMAND can_interface_manager_tests)
endif()

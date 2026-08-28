add_library(ti5_config STATIC
    config/config_loader.cpp
)

target_include_directories(ti5_config PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_config PUBLIC
    yaml-cpp
)

add_library(ti5_can STATIC
    can/can_protocol.cpp
    can/encoder_conversion.cpp
    can/can_discovery.cpp
    can/can_bus.cpp
)

target_include_directories(ti5_can PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_can PUBLIC
    robot_can
    ti5_config
    spdlog::spdlog
    fmt::fmt
)

add_library(ti5_motor STATIC
    motor/can_motor.cpp
)

target_include_directories(ti5_motor PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_motor PUBLIC
    ti5_can
)

add_library(ti5_joint STATIC
    joint/joint.cpp
    joint/joint_config_builder.cpp
)

target_include_directories(ti5_joint PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_joint PUBLIC
    ti5_motor
)

add_library(ti5_arm STATIC
    arm/arm.cpp
)

target_include_directories(ti5_arm PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_arm PUBLIC
    ti5_joint
)

add_library(ti5_head STATIC
    head/head.cpp
)

target_include_directories(ti5_head PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_head PUBLIC
    ti5_joint
)

add_library(ti5_hand_transport STATIC
    hand/aoyi_protocol.cpp
    hand/hand_channel.cpp
    hand/hand_config.cpp
    hand/hand_discovery.cpp
)

target_include_directories(ti5_hand_transport PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_hand_transport PUBLIC
    robot_can
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
)

add_library(ti5_hand STATIC
    hand/hand.cpp
)

target_include_directories(ti5_hand PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_hand PUBLIC
    ti5_hand_transport
)

add_library(ti5_controller STATIC
    controller/arm_controller.cpp
    controller/head_controller.cpp
    controller/hand_controller.cpp
)

target_include_directories(ti5_controller PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_controller PUBLIC
    ti5_arm
    ti5_head
    ti5_hand
)

target_compile_definitions(robot PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(robot PRIVATE
    ti5_controller
    ti5_config
)

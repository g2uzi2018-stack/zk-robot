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
)

target_include_directories(ti5_joint PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_joint PUBLIC
    ti5_motor
)

target_link_libraries(robot PRIVATE
    ti5_joint
    ti5_config
)

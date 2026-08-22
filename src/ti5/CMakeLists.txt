target_sources(robot PRIVATE
    can/can_protocol.cpp
    can/encoder_conversion.cpp
    can/can_discovery.cpp
    can/can_bus.cpp
    motor/can_motor.cpp
)

add_library(ti5_config STATIC
    config/config_loader.cpp
)

target_include_directories(ti5_config PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_config PUBLIC
    yaml-cpp
)

target_link_libraries(robot PRIVATE
    ti5_config
)
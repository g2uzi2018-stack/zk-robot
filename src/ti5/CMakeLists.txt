target_sources(robot PRIVATE
    can/can_protocol.cpp
    can/encoder_conversion.cpp
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

add_executable(ti5_can_discovery
    tools/can_discovery_main.cpp
    can/can_discovery.cpp
    can/can_protocol.cpp
    ${CMAKE_SOURCE_DIR}/src/can/socket_can.cpp
    ${CMAKE_SOURCE_DIR}/src/can/can_interface_manager.cpp
)

target_include_directories(ti5_can_discovery PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_can_discovery PRIVATE
    ti5_config
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
)

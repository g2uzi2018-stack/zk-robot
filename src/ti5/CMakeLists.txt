target_sources(robot PRIVATE
    can/can_protocol.cpp
    can/encoder_conversion.cpp
)

add_executable(ti5_can_discovery
    main.cpp
    config_loader.cpp
    discovery.cpp
    can/can_protocol.cpp
    ${CMAKE_SOURCE_DIR}/src/can/socket_can.cpp
)

target_include_directories(ti5_can_discovery PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ti5_can_discovery PRIVATE
    yaml-cpp
    spdlog::spdlog
    fmt::fmt
)

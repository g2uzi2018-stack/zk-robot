find_package(Threads REQUIRED)

add_library(exoskeleton_input STATIC
    serial_device_discovery.cpp
    serial_transport.cpp
    exoskeleton_protocol.cpp
    exoskeleton_stream_decoder.cpp
    exoskeleton.cpp
)

target_include_directories(exoskeleton_input PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_features(exoskeleton_input PUBLIC
    cxx_std_17
)

target_link_libraries(exoskeleton_input PUBLIC
    Threads::Threads
    yaml-cpp
)

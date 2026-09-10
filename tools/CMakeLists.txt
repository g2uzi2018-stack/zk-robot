# 会连接实体机器人的调试工具。它们不参与默认构建，必须显式指定目标。
add_executable(ti5_joint_cli EXCLUDE_FROM_ALL
    ti5/ti5_joint_cli.cpp
)

target_link_libraries(ti5_joint_cli PRIVATE
    ti5_controller
    ti5_config
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_joint_cli PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(ti5_zero_home EXCLUDE_FROM_ALL
    ti5/ti5_zero_home.cpp
)

target_link_libraries(ti5_zero_home PRIVATE
    ti5_motor
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_zero_home PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(exoskeleton_monitor
    exoskeleton/exoskeleton_monitor.cpp
)

target_link_libraries(exoskeleton_monitor PRIVATE
    exoskeleton_input
)

set_target_properties(exoskeleton_monitor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# 会使能 TIAGo 电机并持续下发遥操作目标；默认不参与普通构建，必须显式指定。
add_executable(exoskeleton_tiago_teleop EXCLUDE_FROM_ALL
    exoskeleton/exoskeleton_tiago_teleop.cpp
)

target_link_libraries(exoskeleton_tiago_teleop PRIVATE
    exoskeleton_input
    tiago_components
)

target_compile_definitions(exoskeleton_tiago_teleop PRIVATE
    ZK_ROBOT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

set_target_properties(exoskeleton_tiago_teleop PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

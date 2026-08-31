# 会连接实体机器人的调试工具。它们不参与默认构建，必须显式指定目标。
add_executable(ti5_zero_home EXCLUDE_FROM_ALL
    ti5_zero_home.cpp
)

target_link_libraries(ti5_zero_home PRIVATE
    ti5_motor
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_zero_home PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(ti5_direction_test EXCLUDE_FROM_ALL
    ti5_direction_test.cpp
)

target_link_libraries(ti5_direction_test PRIVATE
    ti5_joint
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_direction_test PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(ti5_arm_check EXCLUDE_FROM_ALL
    ti5_arm_check.cpp
)

target_link_libraries(ti5_arm_check PRIVATE
    ti5_arm
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_arm_check PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(ti5_hand_check EXCLUDE_FROM_ALL
    ti5_hand_check.cpp
)

target_link_libraries(ti5_hand_check PRIVATE
    ti5_controller
    ti5_config
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_hand_check PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(ti5_full_check EXCLUDE_FROM_ALL
    ti5_full_check.cpp
)

target_link_libraries(ti5_full_check PRIVATE
    ti5_controller
    ti5_config
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(ti5_full_check PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(exoskeleton_monitor
    exoskeleton_monitor.cpp
)

target_link_libraries(exoskeleton_monitor PRIVATE
    exoskeleton_input
)

set_target_properties(exoskeleton_monitor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# 会使能 TIAGo 电机并持续下发遥操作目标；默认不参与构建，必须显式指定。
add_executable(exoskeleton_tiago_teleop EXCLUDE_FROM_ALL
    exoskeleton_tiago_teleop.cpp
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

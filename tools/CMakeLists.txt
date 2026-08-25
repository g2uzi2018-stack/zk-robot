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

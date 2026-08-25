add_executable(ti5_can_protocol_test
    ti5_can_protocol_test.cpp
)

target_link_libraries(ti5_can_protocol_test PRIVATE
    ti5_can
)

add_test(NAME ti5_can_protocol COMMAND ti5_can_protocol_test)

add_executable(ti5_can_motor_test
    ti5_can_motor_test.cpp
)

target_link_libraries(ti5_can_motor_test PRIVATE
    ti5_motor
)

add_test(NAME ti5_can_motor COMMAND ti5_can_motor_test)

add_executable(ti5_joint_test
    ti5_joint_test.cpp
)

target_link_libraries(ti5_joint_test PRIVATE
    ti5_joint
)

add_test(NAME ti5_joint COMMAND ti5_joint_test)

add_executable(ti5_config_test
    ti5_config_test.cpp
)

target_link_libraries(ti5_config_test PRIVATE
    ti5_config
)

target_compile_definitions(ti5_config_test PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME ti5_config COMMAND ti5_config_test)

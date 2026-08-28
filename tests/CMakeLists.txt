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

add_executable(ti5_arm_test
    ti5_arm_test.cpp
)

target_link_libraries(ti5_arm_test PRIVATE
    ti5_controller
)

add_test(NAME ti5_arm COMMAND ti5_arm_test)

add_executable(tiago_executor_test
    tiago_executor_test.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/can/can_protocol.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/can/encoder_conversion.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/can/can_bus.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/motor/can_motor.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/joint/joint.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/arm/arm.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/gripper/gripper.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/head/head.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/torso/torso.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/base/base.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/controller/arm_controller.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/controller/gripper_controller.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/controller/head_controller.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/controller/torso_controller.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/controller/base_controller.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/executor/robot_control_executor.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/executor/robot_control_executor_arm.cpp
    ${CMAKE_SOURCE_DIR}/src/tiago/executor/robot_control_executor_components.cpp
)

target_include_directories(tiago_executor_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(tiago_executor_test PRIVATE
    robot_can
    spdlog::spdlog
    fmt::fmt
)

add_test(NAME tiago_executor COMMAND tiago_executor_test)

add_executable(ti5_head_test
    ti5_head_test.cpp
)

target_link_libraries(ti5_head_test PRIVATE
    ti5_controller
)

add_test(NAME ti5_head COMMAND ti5_head_test)

add_executable(ti5_hand_test
    ti5_hand_test.cpp
)

target_link_libraries(ti5_hand_test PRIVATE
    ti5_controller
)

add_test(NAME ti5_hand COMMAND ti5_hand_test)

add_executable(ti5_config_test
    ti5_config_test.cpp
)

target_link_libraries(ti5_config_test PRIVATE
    ti5_config
    ti5_joint
    ti5_hand_transport
)

target_compile_definitions(ti5_config_test PRIVATE
    TI5_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME ti5_config COMMAND ti5_config_test)

# MoveJ 是纯数学规划测试：不打开 CAN，也不依赖任何具体机器人实现。
add_executable(motion_move_j_test
    motion_move_j_test.cpp
)

target_include_directories(motion_move_j_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(motion_move_j_test PRIVATE
    robot_motion
)

add_test(NAME motion_move_j COMMAND motion_move_j_test)

# 笛卡尔直线路径同样是纯数学测试：不执行 IK，不连接 Executor 或硬件。
add_executable(motion_cartesian_line_path_test
    motion_cartesian_line_path_test.cpp
)

target_link_libraries(motion_cartesian_line_path_test PRIVATE
    robot_motion
)

add_test(NAME motion_cartesian_line_path COMMAND motion_cartesian_line_path_test)

add_executable(exoskeleton_protocol_test
    exoskeleton_protocol_test.cpp
)

target_link_libraries(exoskeleton_protocol_test PRIVATE
    exoskeleton_input
)

target_compile_definitions(exoskeleton_protocol_test PRIVATE
    EXOSKELETON_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME exoskeleton_protocol COMMAND exoskeleton_protocol_test)

add_executable(exoskeleton_stream_decoder_test
    exoskeleton_stream_decoder_test.cpp
)

target_link_libraries(exoskeleton_stream_decoder_test PRIVATE
    exoskeleton_input
)

add_test(NAME exoskeleton_stream_decoder COMMAND exoskeleton_stream_decoder_test)

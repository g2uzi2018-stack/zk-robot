target_sources(robot PRIVATE
    can/can_config.cpp
    can/can_protocol.cpp
    can/encoder_conversion.cpp
    can/can_bus.cpp

    motor/can_motor.cpp
    joint/joint.cpp

    arm/arm.cpp
    gripper/gripper.cpp
    head/head.cpp
    torso/torso.cpp

    base/base_config.cpp
    base/base.cpp

    controller/arm_controller.cpp
    controller/gripper_controller.cpp
    controller/head_controller.cpp
    controller/torso_controller.cpp
    controller/base_controller.cpp

    executor/robot_control_executor.cpp
    executor/robot_control_executor_arm.cpp
    executor/robot_control_executor_components.cpp
)
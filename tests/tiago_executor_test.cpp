#include "can/socket_can.hpp"
#include "tiago/arm/arm.hpp"
#include "tiago/base/base.hpp"
#include "tiago/can/encoder_conversion.hpp"
#include "tiago/controller/arm_controller.hpp"
#include "tiago/controller/base_controller.hpp"
#include "tiago/controller/gripper_controller.hpp"
#include "tiago/controller/head_controller.hpp"
#include "tiago/controller/torso_controller.hpp"
#include "tiago/executor/robot_control_executor.hpp"
#include "tiago/gripper/gripper.hpp"
#include "tiago/head/head.hpp"
#include "tiago/torso/torso.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
    using namespace std::chrono_literals;

    using Executor = robot::tiago::RobotControlExecutor;
    using Arm = robot::tiago::Arm;
    using JointValues = Arm::JointValues;

    constexpr std::int32_t kCountsPerMotorRevolution = 100000;
    constexpr double kFeedbackPosition = 0.30;

    void expect(const bool condition, const std::string &message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    template <typename Exception, typename Function>
    void expectThrow(Function &&function, const std::string &message)
    {
        try
        {
            function();
        }
        catch (const Exception &)
        {
            return;
        }
        throw std::runtime_error(message);
    }

    bool near(const double actual, const double expected, const double tolerance = 1e-9)
    {
        return std::abs(actual - expected) <= tolerance;
    }

    robot::tiago::CanMotorConfig rotaryMotor(const std::uint16_t node_id)
    {
        robot::tiago::CanMotorConfig motor;
        motor.node_id = node_id;
        motor.unit = robot::tiago::JointUnit::Radian;
        motor.encoder = robot::tiago::RotaryEncoderConfig{
            kCountsPerMotorRevolution,
            1.0,
            1,
            0.0};
        return motor;
    }

    robot::tiago::CanMotorConfig linearMotor(const std::uint16_t node_id)
    {
        robot::tiago::CanMotorConfig motor;
        motor.node_id = node_id;
        motor.unit = robot::tiago::JointUnit::Meter;
        motor.encoder = robot::tiago::LinearEncoderConfig{
            static_cast<double>(kCountsPerMotorRevolution),
            1,
            0.0};
        return motor;
    }

    robot::tiago::JointConfig rotaryJoint(const std::string &name, const std::uint16_t node_id)
    {
        robot::tiago::JointConfig joint;
        joint.name = name;
        joint.limits = {-2.0, 2.0, 5.0};
        joint.motor = rotaryMotor(node_id);
        return joint;
    }

    robot::tiago::JointConfig linearJoint(const std::string &name, const std::uint16_t node_id)
    {
        robot::tiago::JointConfig joint;
        joint.name = name;
        joint.limits = {-2.0, 2.0, 5.0};
        joint.motor = linearMotor(node_id);
        return joint;
    }

    robot::tiago::CanBusConfig rotaryBus(const std::string &interface_name,
                                         const std::uint16_t first_node,
                                         const std::size_t joint_count,
                                         const std::string &name_prefix)
    {
        robot::tiago::CanBusConfig bus;
        bus.interface_name = interface_name;
        for (std::size_t index = 0; index < joint_count; ++index)
        {
            bus.joints.push_back(rotaryJoint(
                name_prefix + std::to_string(index),
                static_cast<std::uint16_t>(first_node + index)));
        }
        return bus;
    }

    robot::tiago::CanBusConfig linearBus(const std::string &interface_name,
                                         const std::uint16_t first_node,
                                         const std::size_t joint_count,
                                         const std::string &name_prefix)
    {
        robot::tiago::CanBusConfig bus;
        bus.interface_name = interface_name;
        for (std::size_t index = 0; index < joint_count; ++index)
        {
            bus.joints.push_back(linearJoint(
                name_prefix + std::to_string(index),
                static_cast<std::uint16_t>(first_node + index)));
        }
        return bus;
    }

    robot::can::CanFrame feedbackFrame(const std::uint16_t node_id, const std::int32_t position_counts)
    {
        robot::can::CanFrame frame{};
        frame.id = static_cast<std::uint16_t>(0x180 + node_id);
        frame.data_length = 8;
        const auto raw = static_cast<std::uint32_t>(position_counts);
        frame.data[0] = static_cast<std::uint8_t>(raw & 0xFFU);
        frame.data[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
        frame.data[2] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
        frame.data[3] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
        frame.data[6] = 0x01;
        return frame;
    }

    void sendFeedback(robot::can::SocketCan &sender,
                       const std::uint16_t first_node,
                       const std::size_t joint_count,
                       const std::int32_t position_counts)
    {
        for (std::size_t index = 0; index < joint_count; ++index)
        {
            sender.send(feedbackFrame(
                static_cast<std::uint16_t>(first_node + index),
                position_counts));
        }
    }

    JointValues values(const double value)
    {
        JointValues result{};
        result.fill(value);
        return result;
    }

    void refreshServo(Executor &executor,
                      const bool refresh_left,
                      const bool refresh_right,
                      const JointValues &left_target,
                      const JointValues &right_target,
                      const std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (refresh_left)
            {
                executor.setLeftArmServoTarget(left_target, values(1.0));
            }
            if (refresh_right)
            {
                executor.setRightArmServoTarget(right_target, values(1.0));
            }
            std::this_thread::sleep_for(5ms);
        }
    }

    class FixedTrajectory final : public Executor::ArmTrajectory
    {
    public:
        FixedTrajectory(const JointValues &goal, const Duration duration)
            : goal_(goal), duration_(duration)
        {
        }

        Point sample(const Duration elapsed) const override
        {
            Point point;
            point.position = goal_;
            point.finished = elapsed >= duration_;
            return point;
        }

        Duration duration() const noexcept override
        {
            return duration_;
        }

    private:
        JointValues goal_;
        Duration duration_;
    };
}

int main()
{
    const std::array<std::string, 11> interfaces{
        "vcan0", "vcan1", "vcan2", "vcan3", "vcan4", "vcan5",
        "vcan6", "vcan7", "vcan8", "vcan9", "vcan10"};

    for (const auto &interface_name : interfaces)
    {
        if (if_nametoindex(interface_name.c_str()) == 0)
        {
            std::cout << "SKIP: TIAGo executor test requires " << interface_name << '\n';
            return 0;
        }
    }

    try
    {
        const auto left_shoulder_config = rotaryBus("vcan0", 1, 2, "left_shoulder_");
        const auto left_elbow_config = rotaryBus("vcan1", 3, 2, "left_elbow_");
        const auto left_wrist_config = rotaryBus("vcan2", 5, 3, "left_wrist_");
        const auto right_shoulder_config = rotaryBus("vcan3", 8, 2, "right_shoulder_");
        const auto right_elbow_config = rotaryBus("vcan4", 10, 2, "right_elbow_");
        const auto right_wrist_config = rotaryBus("vcan5", 12, 3, "right_wrist_");
        const auto left_gripper_config = linearBus("vcan6", 1, 2, "left_gripper_");
        const auto right_gripper_config = linearBus("vcan7", 3, 2, "right_gripper_");
        const auto head_config = rotaryBus("vcan8", 1, 2, "head_");
        const auto torso_config = linearBus("vcan9", 1, 1, "torso_");

        robot::tiago::BaseConfig base_config;
        base_config.interface_name = "vcan10";
        base_config.wheel_radius = 0.1;
        base_config.wheel_separation = 0.5;
        base_config.max_linear_velocity = 1.0;
        base_config.max_angular_velocity = 2.0;
        base_config.right_motor = rotaryMotor(1);
        base_config.left_motor = rotaryMotor(2);

        Arm left_arm(left_shoulder_config, left_elbow_config, left_wrist_config);
        Arm right_arm(right_shoulder_config, right_elbow_config, right_wrist_config);
        robot::tiago::Gripper left_gripper(left_gripper_config);
        robot::tiago::Gripper right_gripper(right_gripper_config);
        robot::tiago::Head head(head_config);
        robot::tiago::Torso torso(torso_config);
        robot::tiago::Base base(base_config);

        robot::tiago::ArmController left_arm_controller(left_arm);
        robot::tiago::ArmController right_arm_controller(right_arm);
        robot::tiago::GripperController left_gripper_controller(left_gripper);
        robot::tiago::GripperController right_gripper_controller(right_gripper);
        robot::tiago::HeadController head_controller(head);
        robot::tiago::TorsoController torso_controller(torso);
        robot::tiago::BaseController base_controller(base);

        robot::can::SocketCan left_shoulder_sender("vcan0");
        robot::can::SocketCan left_elbow_sender("vcan1");
        robot::can::SocketCan left_wrist_sender("vcan2");
        robot::can::SocketCan right_shoulder_sender("vcan3");
        robot::can::SocketCan right_elbow_sender("vcan4");
        robot::can::SocketCan right_wrist_sender("vcan5");

        const robot::tiago::RotaryEncoderConfig encoder{
            kCountsPerMotorRevolution,
            1.0,
            1,
            0.0};
        const auto feedback_counts = robot::tiago::radiansToCounts(kFeedbackPosition, encoder);
        sendFeedback(left_shoulder_sender, 1, 2, feedback_counts);
        sendFeedback(left_elbow_sender, 3, 2, feedback_counts);
        sendFeedback(left_wrist_sender, 5, 3, feedback_counts);
        sendFeedback(right_shoulder_sender, 8, 2, feedback_counts);
        sendFeedback(right_elbow_sender, 10, 2, feedback_counts);
        sendFeedback(right_wrist_sender, 12, 3, feedback_counts);

        const auto initial_positions = values(0.0);
        const auto velocity_limits = values(1.0);
        left_arm_controller.start(initial_positions, velocity_limits);
        right_arm_controller.start(initial_positions, velocity_limits);

        Executor::Config config;
        config.control_period = 1ms;
        config.command_timeout = 50ms;

        Executor::Config invalid_config = config;
        invalid_config.command_timeout = Executor::Duration::zero();
        expectThrow<std::invalid_argument>(
            [&]
            {
                Executor invalid_executor(
                    left_arm_controller,
                    right_arm_controller,
                    left_gripper_controller,
                    right_gripper_controller,
                    head_controller,
                    torso_controller,
                    base_controller,
                    invalid_config);
            },
            "Executor accepted a non-positive command timeout");

        Executor executor(
            left_arm_controller,
            right_arm_controller,
            left_gripper_controller,
            right_gripper_controller,
            head_controller,
            torso_controller,
            base_controller,
            config);

        executor.setLeftArmControlMode(Executor::ArmControlMode::Servo);
        executor.setRightArmControlMode(Executor::ArmControlMode::Servo);

        const auto left_servo_target = values(0.80);
        const auto right_servo_target = values(0.70);
        executor.setLeftArmServoTarget(left_servo_target, velocity_limits);
        executor.setRightArmServoTarget(right_servo_target, velocity_limits);
        executor.start();

        // 两侧持续刷新时，Servo target 应保持 Running，不得超时。
        refreshServo(executor, true, true, left_servo_target, right_servo_target, 150ms);
        std::this_thread::sleep_for(10ms);
        const auto continuous_executor_state = executor.state();
        executor.shutdown();
        expect(continuous_executor_state == Executor::State::Running,
               "Executor faulted while Servo commands were refreshed");
        expect(near(left_arm_controller.targetPositions()[0], left_servo_target[0]),
               "Left Servo target was not retained while refreshing");
        expect(near(right_arm_controller.targetPositions()[0], right_servo_target[0]),
               "Right Servo target was not retained while refreshing");

        // 停止刷新左臂，继续刷新右臂：左右 timeout 必须独立。
        executor.start();
        refreshServo(executor, false, true, left_servo_target, right_servo_target, 130ms);
        std::this_thread::sleep_for(10ms);
        const auto timeout_executor_state = executor.state();
        executor.shutdown();
        expect(timeout_executor_state == Executor::State::Running,
               "Servo timeout faulted the Executor");
        expect(executor.faultMessage().empty(),
               "Servo timeout recorded an Executor fault");

        const auto &left_feedback = left_arm_controller.currentPositions();
        expect(left_feedback[0].has_value(), "Left arm feedback was unavailable during hold");
        expect(near(left_arm_controller.targetPositions()[0], *left_feedback[0]),
               "Left arm did not enter current-position hold after Servo timeout");
        expect(!near(left_arm_controller.targetPositions()[0], left_servo_target[0]),
               "Left arm kept the stale Servo target after timeout");
        expect(left_arm_controller.state() == robot::tiago::ArmController::ControlState::Running,
               "Servo timeout stopped the left ArmController");
        expect(near(right_arm_controller.targetPositions()[0], right_servo_target[0]),
               "Left arm timeout affected the right arm");

        // Holding 后的新 Servo command 应重新成为运动 source。
        const auto resumed_left_target = values(0.90);
        executor.setLeftArmServoTarget(resumed_left_target, velocity_limits);
        executor.start();
        std::this_thread::sleep_for(10ms);
        const auto resumed_executor_state = executor.state();
        executor.shutdown();
        expect(resumed_executor_state == Executor::State::Running,
               "Resuming Servo left the Executor faulted");
        expect(near(left_arm_controller.targetPositions()[0], resumed_left_target[0]),
               "New Servo command did not resume the left arm from Holding");

        // latest target wins：未消费的 A/B 被 C 覆盖，C 仍应立即保持新鲜。
        executor.setLeftArmServoTarget(values(0.40), velocity_limits);
        executor.setLeftArmServoTarget(values(0.50), velocity_limits);
        const auto latest_left_target = values(0.60);
        executor.setLeftArmServoTarget(latest_left_target, velocity_limits);
        executor.start();
        std::this_thread::sleep_for(10ms);
        const auto latest_executor_state = executor.state();
        executor.shutdown();
        expect(latest_executor_state == Executor::State::Running,
               "Latest left Servo command timed out unexpectedly");
        expect(near(left_arm_controller.targetPositions()[0], latest_left_target[0]),
               "Latest left Servo target did not win");

        // timestamp 来自 Servo API 提交时刻，而不是 Executor 消费时刻。
        const auto stale_left_target = values(0.65);
        executor.setLeftArmServoTarget(stale_left_target, velocity_limits);
        std::this_thread::sleep_for(70ms);
        executor.start();
        std::this_thread::sleep_for(10ms);
        const auto stale_executor_state = executor.state();
        executor.shutdown();
        expect(stale_executor_state == Executor::State::Running,
               "A stale Servo command faulted the Executor");
        const auto &stale_feedback = left_arm_controller.currentPositions();
        expect(stale_feedback[0].has_value(), "Feedback was unavailable during stale Servo test");
        expect(near(left_arm_controller.targetPositions()[0], *stale_feedback[0]),
               "Servo timeout used consumption time instead of submission time");
        expect(!near(left_arm_controller.targetPositions()[0], stale_left_target[0]),
               "Stale Servo target was not replaced by Hold");

        // Trajectory mode 不应使用 command_timeout。
        executor.setRightArmControlMode(Executor::ArmControlMode::Trajectory);
        const auto trajectory_goal = values(0.20);
        executor.submitRightArmTrajectory(
            std::make_shared<FixedTrajectory>(trajectory_goal, 20ms),
            velocity_limits);
        executor.start();
        std::this_thread::sleep_for(100ms);
        const auto trajectory_executor_state = executor.state();
        executor.shutdown();
        expect(trajectory_executor_state == Executor::State::Running,
               "Trajectory execution faulted the Executor");
        expect(near(right_arm_controller.targetPositions()[0], trajectory_goal[0]),
               "Trajectory target was changed by command_timeout");

        // Hold / Stop 优先级：同一 mailbox 中的普通 Servo 不得覆盖安全动作。
        const auto ignored_servo_target = values(0.95);
        executor.setLeftArmServoTarget(ignored_servo_target, velocity_limits);
        executor.holdLeftArm();
        executor.start();
        std::this_thread::sleep_for(10ms);
        const auto hold_executor_state = executor.state();
        executor.shutdown();
        expect(hold_executor_state == Executor::State::Running,
               "Pending Hold faulted the Executor");
        const auto &held_feedback = left_arm_controller.currentPositions();
        expect(held_feedback[0].has_value(), "Feedback was unavailable during pending Hold");
        expect(near(left_arm_controller.targetPositions()[0], *held_feedback[0]),
               "Pending Hold did not take priority over Servo");
        expect(!near(left_arm_controller.targetPositions()[0], ignored_servo_target[0]),
               "Pending Servo overwrote Hold");

        executor.setLeftArmServoTarget(values(1.00), velocity_limits);
        executor.stopLeftArm();
        executor.start();
        std::this_thread::sleep_for(10ms);
        const auto stop_executor_state = executor.state();
        executor.shutdown();
        expect(stop_executor_state == Executor::State::Running,
               "Explicit Arm Stop faulted the Executor");
        expect(left_arm_controller.state() == robot::tiago::ArmController::ControlState::Idle,
               "Pending Stop did not take priority over Servo");

        std::cout << "TIAGo executor timeout test passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TIAGo executor timeout test failed: " << error.what() << '\n';
        return 1;
    }
}

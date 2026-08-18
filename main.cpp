#include "common/logger.hpp"

#include "tiago/arm/arm.hpp"
#include "tiago/base/base.hpp"
#include "tiago/base/base_config.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/gripper/gripper.hpp"
#include "tiago/head/head.hpp"
#include "tiago/torso/torso.hpp"

#include "tiago/controller/arm_controller.hpp"
#include "tiago/controller/base_controller.hpp"
#include "tiago/controller/gripper_controller.hpp"
#include "tiago/controller/head_controller.hpp"
#include "tiago/controller/torso_controller.hpp"

#include "tiago/executor/robot_control_executor.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
    using namespace robot::tiago;
    constexpr auto kFeedbackTimeout = std::chrono::seconds{2};
    constexpr auto kFeedbackPollPeriod = std::chrono::milliseconds{10};
    Arm::JointValues waitArmPositions(Arm &arm, const char *name)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto positions = arm.readPositions();
            Arm::JointValues values{};
            bool complete = true;
            for (std::size_t i = 0; i < Arm::kJointCount; ++i)
            {
                if (!positions[i])
                {
                    complete = false;
                    break;
                }

                values[i] = *positions[i];
            }

            if (complete)
            {
                return values;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error(std::string("Timed out waiting for ") + name + " feedback");
    }

    Gripper::FingerValues waitGripperPositions(Gripper &gripper, const char *name)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto positions = gripper.readPositions();
            Gripper::FingerValues values{};
            if (positions[0] && positions[1])
            {
                values[0] = *positions[0];
                values[1] = *positions[1];
                return values;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error(std::string("Timed out waiting for ") + name + " feedback");
    }

    Head::JointValues waitHeadPositions(Head &head)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto positions = head.readPositions();
            Head::JointValues values{};
            if (positions[0] && positions[1])
            {
                values[0] = *positions[0];
                values[1] = *positions[1];
                return values;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error("Timed out waiting for head feedback");
    }

    double waitTorsoPosition(Torso &torso)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto position = torso.readPosition();
            if (position)
            {
                return *position;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error("Timed out waiting for torso feedback");
    }

    Arm::JointValues armVelocityLimits(const CanBusConfig &shoulder, const CanBusConfig &elbow,
                                       const CanBusConfig &wrist)
    {
        if (shoulder.joints.size() != 2 || elbow.joints.size() != 2 || wrist.joints.size() != 3)
        {
            throw std::logic_error("Invalid arm config while building velocity limits");
        }

        return {shoulder.joints[0].limits.max_velocity, shoulder.joints[1].limits.max_velocity,
                elbow.joints[0].limits.max_velocity,    elbow.joints[1].limits.max_velocity,
                wrist.joints[0].limits.max_velocity,    wrist.joints[1].limits.max_velocity,
                wrist.joints[2].limits.max_velocity};
    }

    Gripper::FingerValues gripperVelocityLimits(const CanBusConfig &config)
    {
        if (config.joints.size() != Gripper::kFingerCount)
        {
            throw std::logic_error("Invalid gripper config while building velocity limits");
        }

        return {config.joints[0].limits.max_velocity, config.joints[1].limits.max_velocity};
    }

    Head::JointValues headVelocityLimits(const CanBusConfig &config)
    {
        if (config.joints.size() != Head::kJointCount)
        {
            throw std::logic_error("Invalid head config while building velocity limits");
        }

        return {config.joints[0].limits.max_velocity, config.joints[1].limits.max_velocity};
    }
} // namespace

int main()
{
    using namespace robot::tiago;
    try
    {
        robot::common::logger()->info("Robot startup");
        // ====================================================
        // 1. Load configuration
        // ====================================================

        const auto left_shoulder_config = loadCanBusConfig("config/tiago/can/left_shoulder.yaml");
        const auto left_elbow_config = loadCanBusConfig("config/tiago/can/left_elbow.yaml");
        const auto left_wrist_config = loadCanBusConfig("config/tiago/can/left_wrist.yaml");
        const auto right_shoulder_config = loadCanBusConfig("config/tiago/can/right_shoulder.yaml");
        const auto right_elbow_config = loadCanBusConfig("config/tiago/can/right_elbow.yaml");
        const auto right_wrist_config = loadCanBusConfig("config/tiago/can/right_wrist.yaml");
        const auto left_gripper_config = loadCanBusConfig("config/tiago/can/left_gripper.yaml");
        const auto right_gripper_config = loadCanBusConfig("config/tiago/can/right_gripper.yaml");
        const auto head_config = loadCanBusConfig("config/tiago/can/head.yaml");
        const auto torso_config = loadCanBusConfig("config/tiago/can/torso.yaml");
        const auto base_config = loadBaseConfig("config/tiago/base/base.yaml");
        // ====================================================
        // 2. Create hardware
        // ====================================================

        Arm left_arm(left_shoulder_config, left_elbow_config, left_wrist_config);
        Arm right_arm(right_shoulder_config, right_elbow_config, right_wrist_config);
        Gripper left_gripper(left_gripper_config);
        Gripper right_gripper(right_gripper_config);
        Head head(head_config);
        Torso torso(torso_config);
        Base base(base_config);
        // ====================================================
        // 3. Create controllers
        // ====================================================

        ArmController left_arm_controller(left_arm);
        ArmController right_arm_controller(right_arm);
        GripperController left_gripper_controller(left_gripper);
        GripperController right_gripper_controller(right_gripper);
        HeadController head_controller(head);
        TorsoController torso_controller(torso);
        BaseController base_controller(base);
        // ====================================================
        // 4. Hardware cleanup
        // ====================================================

            // 每个清理动作独立容错，确保单个设备失败时其余设备仍会停机。
            const auto shutdownHardware = [&]() noexcept
        {
            const auto safe = [](const char *name, auto &&action) noexcept
            {
                try
                {
                    action();
                }
                catch (const std::exception &error)
                {
                    robot::common::logger()->warn("Cleanup {} failed: {}", name, error.what());
                }
                catch (...)
                {
                    robot::common::logger()->warn("Cleanup {} failed with unknown error", name);
                }
            };
            safe("left arm stop", [&] { left_arm.stop(); });
            safe("right arm stop", [&] { right_arm.stop(); });
            safe("left gripper stop", [&] { left_gripper.stop(); });
            safe("right gripper stop", [&] { right_gripper.stop(); });
            safe("head stop", [&] { head.stop(); });
            safe("torso stop", [&] { torso.stop(); });
            safe("base stop", [&] { base.stop(); });
            safe("left arm disable", [&] { left_arm.disable(); });
            safe("right arm disable", [&] { right_arm.disable(); });
            safe("left gripper disable", [&] { left_gripper.disable(); });
            safe("right gripper disable", [&] { right_gripper.disable(); });
            safe("head disable", [&] { head.disable(); });
            safe("torso disable", [&] { torso.disable(); });
            safe("base disable", [&] { base.disable(); });
        };
        try
        {
            // =================================================
            // 5. Hardware startup
            // =================================================

            left_arm.clearFault();
            right_arm.clearFault();
            left_gripper.clearFault();
            right_gripper.clearFault();
            head.clearFault();
            torso.clearFault();
            base.clearFault();
            left_arm.enable();
            right_arm.enable();
            left_gripper.enable();
            right_gripper.enable();
            head.enable();
            torso.enable();
            base.enable();
            // =================================================
            // 6. Read real current positions
            // =================================================

            robot::common::logger()->info("Waiting for current robot feedback");
            const auto left_arm_position = waitArmPositions(left_arm, "left arm");
            const auto right_arm_position = waitArmPositions(right_arm, "right arm");
            const auto left_gripper_position = waitGripperPositions(left_gripper, "left gripper");
            const auto right_gripper_position = waitGripperPositions(right_gripper, "right gripper");
            const auto head_position = waitHeadPositions(head);
            const double torso_position = waitTorsoPosition(torso);
            robot::common::logger()->info("Current robot feedback acquired");
            // =================================================
            // 7. Velocity limits from YAML
            // =================================================

            const auto left_arm_velocity =
                armVelocityLimits(left_shoulder_config, left_elbow_config, left_wrist_config);
            const auto right_arm_velocity =
                armVelocityLimits(right_shoulder_config, right_elbow_config, right_wrist_config);
            const auto left_gripper_velocity = gripperVelocityLimits(left_gripper_config);
            const auto right_gripper_velocity = gripperVelocityLimits(right_gripper_config);
            const auto head_velocity = headVelocityLimits(head_config);
            const double torso_velocity = torso_config.joints.at(0).limits.max_velocity;
            // =================================================
            // 8. Start all controllers at real positions
            // =================================================

            left_arm_controller.start(left_arm_position, left_arm_velocity);
            right_arm_controller.start(right_arm_position, right_arm_velocity);
            left_gripper_controller.start(left_gripper_position, left_gripper_velocity);
            right_gripper_controller.start(right_gripper_position, right_gripper_velocity);
            head_controller.start(head_position, head_velocity);
            torso_controller.start(torso_position, torso_velocity);
            base_controller.start();
            // =================================================
            // 9. Helper: choose a safe target
            // =================================================

            // 优先向正方向移动 delta。
            //
            // 如果正方向接近限位，
            // 则自动向负方向移动。
            //
            // 这样测试不依赖机器人初始姿态。
            const auto offsetWithinLimits = [](double current, const auto &joint_config, double delta)
            {
                const double positive = current + delta;
                if (positive <= joint_config.limits.max_position)
                {
                    return positive;
                }

                const double negative = current - delta;
                if (negative >= joint_config.limits.min_position)
                {
                    return negative;
                }

                throw std::runtime_error("Cannot build safe test target");
            };
            // =================================================
            // 10. Helper: manual controller loop
            // =================================================
            //
            // IMPORTANT:
            //
            // 这里只能在 RobotControlExecutor 启动之前使用。
            //
            // 当前阶段 Gripper / Head / Torso / Base
            // 的 Executor mailbox 尚未实现，
            // 因此这里暂时由 main 单线程驱动 Controller。
            //
            // Executor 启动以后，
            // main 不再直接访问 Controller。
            // =================================================

            const auto runManualControl = [&](std::chrono::milliseconds duration)
            {
                auto next_cycle = std::chrono::steady_clock::now();
                const auto end_time = next_cycle + duration;
                while (std::chrono::steady_clock::now() < end_time)
                {
                    left_arm_controller.update();
                    right_arm_controller.update();
                    left_gripper_controller.update();
                    right_gripper_controller.update();
                    head_controller.update();
                    torso_controller.update();
                    base_controller.update();
                    next_cycle += std::chrono::milliseconds{100};
                    std::this_thread::sleep_until(next_cycle);
                }
            };
            // 先跑几个周期。
            runManualControl(std::chrono::milliseconds{300});
            // =================================================
            // TEST 1
            // Left gripper
            // =================================================

            robot::common::logger()->info("TEST 1: moving left gripper");
            Gripper::FingerValues left_gripper_test = left_gripper_position;
            // 两个 finger 保持同方向运动。
            bool left_gripper_can_open = true;
            for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
            {
                if (left_gripper_position[i] + 0.012 > left_gripper_config.joints[i].limits.max_position)
                {
                    left_gripper_can_open = false;
                }
            }

            for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
            {
                if (left_gripper_can_open)
                {
                    left_gripper_test[i] = left_gripper_position[i] + 0.012 * 2;
                }
                else
                {
                    left_gripper_test[i] = left_gripper_position[i] - 0.012 * 2;
                }
            }

            left_gripper_controller.setTarget(left_gripper_test, left_gripper_velocity);
            runManualControl(std::chrono::milliseconds{800});
            if (!left_gripper_controller.targetReached(0.004))
            {
                throw std::runtime_error("Left gripper did not reach test target");
            }

            // 回到启动位置。
            left_gripper_controller.setTarget(left_gripper_position, left_gripper_velocity);
            runManualControl(std::chrono::milliseconds{800});
            robot::common::logger()->info("TEST 1 PASS: left gripper");
            // =================================================
            // TEST 2
            // Right gripper
            // =================================================

            robot::common::logger()->info("TEST 2: moving right gripper");
            Gripper::FingerValues right_gripper_test = right_gripper_position;
            bool right_gripper_can_open = true;
            for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
            {
                if (right_gripper_position[i] + 0.012 > right_gripper_config.joints[i].limits.max_position)
                {
                    right_gripper_can_open = false;
                }
            }

            for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
            {
                if (right_gripper_can_open)
                {
                    right_gripper_test[i] = right_gripper_position[i] + 0.012 * 2;
                }
                else
                {
                    right_gripper_test[i] = right_gripper_position[i] - 0.012 * 2;
                }
            }

            right_gripper_controller.setTarget(right_gripper_test, right_gripper_velocity);
            runManualControl(std::chrono::milliseconds{800});
            if (!right_gripper_controller.targetReached(0.004))
            {
                throw std::runtime_error("Right gripper did not reach test target");
            }

            right_gripper_controller.setTarget(right_gripper_position, right_gripper_velocity);
            runManualControl(std::chrono::milliseconds{800});
            robot::common::logger()->info("TEST 2 PASS: right gripper");
            // =================================================
            // TEST 3
            // Head
            // =================================================

            robot::common::logger()->info("TEST 3: moving head");
            Head::JointValues head_test = head_position;
            // 只动 head joint 1。
            head_test[0] = offsetWithinLimits(head_position[0], head_config.joints[0], 0.20);
            head_controller.setTarget(head_test, head_velocity);
            runManualControl(std::chrono::milliseconds{1500});
            if (!head_controller.targetReached(0.03))
            {
                throw std::runtime_error("Head did not reach test target");
            }

            head_controller.setTarget(head_position, head_velocity);
            runManualControl(std::chrono::milliseconds{1500});
            robot::common::logger()->info("TEST 3 PASS: head");
            // =================================================
            // TEST 4
            // Torso
            // =================================================

            robot::common::logger()->info("TEST 4: moving torso");
            const double torso_test = offsetWithinLimits(torso_position, torso_config.joints.at(0), 0.05);
            torso_controller.setTarget(torso_test, torso_velocity);
            runManualControl(std::chrono::milliseconds{1400});
            if (!torso_controller.targetReached(0.01))
            {
                throw std::runtime_error("Torso did not reach test target");
            }

            torso_controller.setTarget(torso_position, torso_velocity);
            runManualControl(std::chrono::milliseconds{1400});
            robot::common::logger()->info("TEST 4 PASS: torso");
            // =================================================
            // TEST 5
            // Base
            // =================================================

            robot::common::logger()->info("TEST 5: moving base forward");
            // YAML 限制是 0.30 m/s。
            //
            // 使用较保守的速度，
            // 向前运行约 1 秒。
            const double base_test_velocity = base_config.max_linear_velocity * 0.4;
            base_controller.setTarget(base_test_velocity, 0.0);
            runManualControl(std::chrono::milliseconds{1000});
            // 底盘立即回到零速度目标。
            base_controller.setTarget(0.0, 0.0);
            runManualControl(std::chrono::milliseconds{400});
            if (base_controller.state() != BaseController::ControlState::Running)
            {
                throw std::runtime_error("BaseController is not Running after test");
            }

            robot::common::logger()->info("TEST 5 PASS: base");
            // =================================================
            // Manual phase complete
            // =================================================

            robot::common::logger()->info("Manual component motion tests complete");
            // =================================================
            // 11. Start RobotControlExecutor
            // =================================================

            RobotControlExecutor::Config executor_config;
            executor_config.control_period = std::chrono::milliseconds{100};
            executor_config.command_timeout = std::chrono::milliseconds{500};
            {
                RobotControlExecutor executor(
                    left_arm_controller,
                    right_arm_controller,
                    left_gripper_controller,
                    right_gripper_controller,
                    head_controller,
                    torso_controller,
                    base_controller,
                    executor_config);
                executor.start();
                robot::common::logger()->info("Arm Servo / Hold / Stop mailbox test started");
                // 让 Executor 先运行几个周期，
                // 保证 RobotState 和 Arm feedback 已刷新。
                std::this_thread::sleep_for(std::chrono::milliseconds{500});
                if (executor.state() == RobotControlExecutor::State::Faulted)
                {
                    throw std::runtime_error(std::string("Executor fault before Arm test: ") + executor.faultMessage());
                }

                // =================================================
                // TEST 6
                // Arm Servo movement
                // =================================================

                robot::common::logger()->info("TEST 6: moving both arms with Servo");
                Arm::JointValues left_servo_target = left_arm_position;
                Arm::JointValues right_servo_target = right_arm_position;
                // 两条机械臂都只移动 joint 1。
                //
                // 0.25 rad 约 14 度，
                // Webots 中应该比较容易看到。
                left_servo_target[0] = offsetWithinLimits(left_arm_position[0], left_shoulder_config.joints[0], 0.25);
                right_servo_target[0] =
                    offsetWithinLimits(right_arm_position[0], right_shoulder_config.joints[0], 0.25);
                // 目标来源切换为 Servo。
                executor.setLeftArmControlMode(RobotControlExecutor::ArmControlMode::Servo);
                executor.setRightArmControlMode(RobotControlExecutor::ArmControlMode::Servo);
                // 提交 Servo 目标。
                executor.setLeftArmServoTarget(left_servo_target, left_arm_velocity);
                executor.setRightArmServoTarget(right_servo_target, right_arm_velocity);
                // 故意不等待到最终目标。
                //
                // 让机械臂运动到一半，
                // 后面直接测试 Hold。
                std::this_thread::sleep_for(std::chrono::milliseconds{700});
                if (executor.state() == RobotControlExecutor::State::Faulted)
                {
                    throw std::runtime_error(std::string("Executor fault during Servo: ") + executor.faultMessage());
                }

                const auto moving_state = executor.latestState();
                // 确认真正发生过运动。
                double left_arm_motion = 0.0;
                double right_arm_motion = 0.0;
                for (std::size_t i = 0; i < Arm::kJointCount; ++i)
                {
                    if (!moving_state.left_arm_positions[i] || !moving_state.right_arm_positions[i])
                    {
                        throw std::runtime_error("Arm feedback unavailable during Servo");
                    }

                    double left_difference = *moving_state.left_arm_positions[i] - left_arm_position[i];
                    if (left_difference < 0.0)
                    {
                        left_difference = -left_difference;
                    }

                    double right_difference = *moving_state.right_arm_positions[i] - right_arm_position[i];
                    if (right_difference < 0.0)
                    {
                        right_difference = -right_difference;
                    }

                    if (left_difference > left_arm_motion)
                    {
                        left_arm_motion = left_difference;
                    }

                    if (right_difference > right_arm_motion)
                    {
                        right_arm_motion = right_difference;
                    }
                }

                if (left_arm_motion < 0.02)
                {
                    throw std::runtime_error("Left arm did not visibly move");
                }

                if (right_arm_motion < 0.02)
                {
                    throw std::runtime_error("Right arm did not visibly move");
                }

                robot::common::logger()->info("TEST 6 PASS: both arms moved");
                // =================================================
                // TEST 7
                // Hold while moving
                // =================================================

                robot::common::logger()->info("TEST 7: submitting Hold while arms are moving");
                executor.holdLeftArm();
                executor.holdRightArm();
                // 等待 Hold 被 mailbox 消费。
                std::this_thread::sleep_for(std::chrono::milliseconds{300});
                if (executor.state() == RobotControlExecutor::State::Faulted)
                {
                    throw std::runtime_error(std::string("Executor fault during Hold: ") + executor.faultMessage());
                }

                const auto hold_state_1 = executor.latestState();
                // Hold 后 Controller 必须继续 Running。
                if (hold_state_1.left_arm_state != ArmController::ControlState::Running ||
                    hold_state_1.right_arm_state != ArmController::ControlState::Running)
                {
                    throw std::runtime_error("ArmController is not Running during Hold");
                }

                // 再保持一段时间。
                //
                // 如果旧 Servo target 没有被清掉，
                // 机器人会继续运动。
                std::this_thread::sleep_for(std::chrono::milliseconds{700});
                const auto hold_state_2 = executor.latestState();
                double left_hold_drift = 0.0;
                double right_hold_drift = 0.0;
                for (std::size_t i = 0; i < Arm::kJointCount; ++i)
                {
                    if (!hold_state_1.left_arm_positions[i] || !hold_state_2.left_arm_positions[i] ||
                        !hold_state_1.right_arm_positions[i] || !hold_state_2.right_arm_positions[i])
                    {
                        throw std::runtime_error("Arm feedback unavailable during Hold");
                    }

                    double left_difference = *hold_state_2.left_arm_positions[i] - *hold_state_1.left_arm_positions[i];
                    if (left_difference < 0.0)
                    {
                        left_difference = -left_difference;
                    }

                    double right_difference =
                        *hold_state_2.right_arm_positions[i] - *hold_state_1.right_arm_positions[i];
                    if (right_difference < 0.0)
                    {
                        right_difference = -right_difference;
                    }

                    if (left_difference > left_hold_drift)
                    {
                        left_hold_drift = left_difference;
                    }

                    if (right_difference > right_hold_drift)
                    {
                        right_hold_drift = right_difference;
                    }
                }

                // 这里允许 0.03 rad 的反馈误差。
                //
                // 如果 Hold 后仍继续朝原 Servo target 运动，
                // 700 ms 内会明显超过这个量。
                if (left_hold_drift > 0.03)
                {
                    throw std::runtime_error("Left arm continued moving after Hold");
                }

                if (right_hold_drift > 0.03)
                {
                    throw std::runtime_error("Right arm continued moving after Hold");
                }

                robot::common::logger()->info("TEST 7 PASS: Hold stopped motion and maintained position");
                // =================================================
                // TEST 8
                // Stop
                // =================================================

                robot::common::logger()->info("TEST 8: submitting Arm Stop");
                executor.stopLeftArm();
                executor.stopRightArm();
                std::this_thread::sleep_for(std::chrono::milliseconds{500});
                if (executor.state() == RobotControlExecutor::State::Faulted)
                {
                    throw std::runtime_error(std::string("Executor fault during Stop: ") + executor.faultMessage());
                }

                const auto stop_state = executor.latestState();
                if (stop_state.left_arm_state != ArmController::ControlState::Idle ||
                    stop_state.right_arm_state != ArmController::ControlState::Idle)
                {
                    throw std::runtime_error("ArmControllers did not enter Idle after Stop");
                }

                robot::common::logger()->info("TEST 8 PASS: Arm Stop");
                // =================================================
                // 12. Final statistics
                // =================================================

                const auto statistics = executor.statistics();
                const auto last_execution_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(statistics.last_execution_time).count();
                std::cout << "\n"
                          << "====================================\n"
                          << "ROBOT COMPONENT INTEGRATION TEST\n"
                          << "====================================\n"
                          << "Left gripper:          PASS\n"
                          << "Right gripper:         PASS\n"
                          << "Head:                  PASS\n"
                          << "Torso:                 PASS\n"
                          << "Base:                  PASS\n"
                          << "Left / Right Arm Servo:PASS\n"
                          << "Arm Hold mailbox:      PASS\n"
                          << "Arm Stop mailbox:      PASS\n"
                          << "------------------------------------\n"
                          << "Cycle count: " << statistics.cycle_count << '\n'
                          << "Last execution: " << last_execution_us << " us\n"
                          << "Deadline miss: " << statistics.deadline_miss_count << '\n'
                          << "====================================\n"
                          << "RESULT: PASS\n"
                          << "====================================\n";
            }

            // =================================================
            // 13. Hardware shutdown
            // =================================================

            shutdownHardware();
            robot::common::logger()->info("Robot shutdown complete");
            return 0;
        }
        catch (...)
        {
            shutdownHardware();
            throw;
        }
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error("Robot test failed: {}", error.what());
        return 1;
    }
}

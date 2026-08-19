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

    Arm::JointValues armVelocityLimits(const CanBusConfig &shoulder, const CanBusConfig &elbow, const CanBusConfig &wrist)
    {
        if (shoulder.joints.size() != 2 || elbow.joints.size() != 2 || wrist.joints.size() != 3)
        {
            throw std::logic_error("Invalid arm config while building velocity limits");
        }
        return {shoulder.joints[0].limits.max_velocity, shoulder.joints[1].limits.max_velocity, elbow.joints[0].limits.max_velocity,
                elbow.joints[1].limits.max_velocity,    wrist.joints[0].limits.max_velocity,    wrist.joints[1].limits.max_velocity,
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
            // 6. Read current real position
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
            // 7. Velocity limits
            // =================================================

            const auto left_arm_velocity = armVelocityLimits(left_shoulder_config, left_elbow_config, left_wrist_config);
            const auto right_arm_velocity = armVelocityLimits(right_shoulder_config, right_elbow_config, right_wrist_config);
            const auto left_gripper_velocity = gripperVelocityLimits(left_gripper_config);
            const auto right_gripper_velocity = gripperVelocityLimits(right_gripper_config);
            const auto head_velocity = headVelocityLimits(head_config);
            const double torso_velocity = torso_config.joints.at(0).limits.max_velocity;

            // =================================================
            // 8. Start controllers at real current position
            // =================================================

            left_arm_controller.start(left_arm_position, left_arm_velocity);
            right_arm_controller.start(right_arm_position, right_arm_velocity);
            left_gripper_controller.start(left_gripper_position, left_gripper_velocity);
            right_gripper_controller.start(right_gripper_position, right_gripper_velocity);
            head_controller.start(head_position, head_velocity);
            torso_controller.start(torso_position, torso_velocity);
            base_controller.start();

            // =================================================
            // 9. Small helpers
            // =================================================

            const auto absolute = [](double value) { return value < 0.0 ? -value : value; };

            // 为普通旋转/直线关节选择一个安全偏移目标。
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

            // 夹爪使用更大的动作幅度。
            //
            // 不采用固定 +/- 0.024，
            // 因为机器人可能本来就在某个限位附近。
            //
            // 这里比较：
            //   接近最大开度
            //   接近最小开度
            //
            // 哪个距离当前位置更远，就选择哪个。
            const auto buildLargeGripperTarget = [&](const Gripper::FingerValues &current, const CanBusConfig &config)
            {
                constexpr double kLimitMargin = 0.003;
                Gripper::FingerValues open_target{};
                Gripper::FingerValues close_target{};
                double open_distance = 0.0;
                double close_distance = 0.0;
                for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
                {
                    open_target[i] = config.joints[i].limits.max_position - kLimitMargin;
                    close_target[i] = config.joints[i].limits.min_position + kLimitMargin;
                    open_distance += absolute(open_target[i] - current[i]);
                    close_distance += absolute(close_target[i] - current[i]);
                }
                if (open_distance >= close_distance)
                {
                    return open_target;
                }
                return close_target;
            };
            const auto checkExecutor = [](RobotControlExecutor &executor, const char *stage)
            {
                if (executor.state() == RobotControlExecutor::State::Faulted)
                {
                    throw std::runtime_error(std::string("Executor fault during ") + stage + ": " + executor.faultMessage());
                }
            };

            // =================================================
            // 10. Executor startup
            // =================================================

            RobotControlExecutor::Config executor_config;
            executor_config.control_period = std::chrono::milliseconds{100};
            executor_config.command_timeout = std::chrono::milliseconds{500};
            {
                RobotControlExecutor executor(left_arm_controller, right_arm_controller, left_gripper_controller, right_gripper_controller, head_controller,
                                              torso_controller, base_controller, executor_config);
                executor.start();
                robot::common::logger()->info("Full Executor mailbox integration test started");

                // 先让所有 Controller 在 Executor 中运行几个周期。
                std::this_thread::sleep_for(std::chrono::milliseconds{500});
                checkExecutor(executor, "startup");

                // =================================================
                // TEST 1
                // Left Gripper
                // =================================================

                robot::common::logger()->info("TEST 1: large left gripper motion via Executor");
                const auto left_gripper_test = buildLargeGripperTarget(left_gripper_position, left_gripper_config);
                executor.setLeftGripperTarget(left_gripper_test, left_gripper_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                checkExecutor(executor, "left gripper");
                {
                    const auto state = executor.latestState();
                    for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
                    {
                        if (!state.left_gripper_positions[i])
                        {
                            throw std::runtime_error("Left gripper feedback unavailable");
                        }
                        if (absolute(*state.left_gripper_positions[i] - left_gripper_test[i]) > 0.005)
                        {
                            throw std::runtime_error("Left gripper did not reach large test target");
                        }
                    }
                }

                // 回到启动位置。
                executor.setLeftGripperTarget(left_gripper_position, left_gripper_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                checkExecutor(executor, "left gripper return");
                robot::common::logger()->info("TEST 1 PASS: left gripper mailbox");

                // =================================================
                // TEST 2
                // Right Gripper
                // =================================================

                robot::common::logger()->info("TEST 2: large right gripper motion via Executor");
                const auto right_gripper_test = buildLargeGripperTarget(right_gripper_position, right_gripper_config);
                executor.setRightGripperTarget(right_gripper_test, right_gripper_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                checkExecutor(executor, "right gripper");
                {
                    const auto state = executor.latestState();
                    for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
                    {
                        if (!state.right_gripper_positions[i])
                        {
                            throw std::runtime_error("Right gripper feedback unavailable");
                        }
                        if (absolute(*state.right_gripper_positions[i] - right_gripper_test[i]) > 0.005)
                        {
                            throw std::runtime_error("Right gripper did not reach large test target");
                        }
                    }
                }
                executor.setRightGripperTarget(right_gripper_position, right_gripper_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                checkExecutor(executor, "right gripper return");
                robot::common::logger()->info("TEST 2 PASS: right gripper mailbox");

                // =================================================
                // TEST 3
                // Head
                // =================================================

                robot::common::logger()->info("TEST 3: moving head via Executor");
                Head::JointValues head_test = head_position;
                head_test[0] = offsetWithinLimits(head_position[0], head_config.joints[0], 0.25);
                executor.setHeadTarget(head_test, head_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1800});
                checkExecutor(executor, "head");
                {
                    const auto state = executor.latestState();
                    if (!state.head_positions[0])
                    {
                        throw std::runtime_error("Head feedback unavailable");
                    }
                    if (absolute(*state.head_positions[0] - head_test[0]) > 0.04)
                    {
                        throw std::runtime_error("Head did not reach test target");
                    }
                }
                executor.setHeadTarget(head_position, head_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1800});
                checkExecutor(executor, "head return");
                robot::common::logger()->info("TEST 3 PASS: head mailbox");

                // =================================================
                // TEST 4
                // Torso
                // =================================================

                robot::common::logger()->info("TEST 4: moving torso via Executor");
                const double torso_test = offsetWithinLimits(torso_position, torso_config.joints.at(0), 0.06);
                executor.setTorsoTarget(torso_test, torso_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1600});
                checkExecutor(executor, "torso");
                {
                    const auto state = executor.latestState();
                    if (!state.torso_position)
                    {
                        throw std::runtime_error("Torso feedback unavailable");
                    }
                    if (absolute(*state.torso_position - torso_test) > 0.012)
                    {
                        throw std::runtime_error("Torso did not reach test target");
                    }
                }
                executor.setTorsoTarget(torso_position, torso_velocity);
                std::this_thread::sleep_for(std::chrono::milliseconds{1600});
                checkExecutor(executor, "torso return");
                robot::common::logger()->info("TEST 4 PASS: torso mailbox");

                // =================================================
                // TEST 5
                // Base
                // =================================================

                robot::common::logger()->info("TEST 5: moving base via Executor");

                // 比之前稍明显一点。
                //
                // YAML 上限 0.30 m/s，
                // 这里使用 40%。
                const double base_velocity = base_config.max_linear_velocity * 0.4;
                executor.setBaseVelocity(base_velocity, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                checkExecutor(executor, "base movement");

                // 回零速度。
                executor.setBaseVelocity(0.0, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds{500});
                checkExecutor(executor, "base stop");
                {
                    const auto state = executor.latestState();
                    if (state.base_state != BaseController::ControlState::Running)
                    {
                        throw std::runtime_error("BaseController is not Running");
                    }
                }
                robot::common::logger()->info("TEST 5 PASS: base mailbox");

                // =================================================
                // TEST 6
                // Arm Servo
                // =================================================

                robot::common::logger()->info("TEST 6: moving both arms via Servo mailbox");
                Arm::JointValues left_servo_target = left_arm_position;
                Arm::JointValues right_servo_target = right_arm_position;

                // 两边只移动 joint 1。
                //
                // 0.25 rad 在 Webots 中应该比较明显。
                left_servo_target[0] = offsetWithinLimits(left_arm_position[0], left_shoulder_config.joints[0], 0.25);
                right_servo_target[0] = offsetWithinLimits(right_arm_position[0], right_shoulder_config.joints[0], 0.25);

                // mode 和 Servo target 可以在同一个 mailbox 周期提交。
                //
                // Executor processArmCommands() 会先处理 mode，再处理 Servo target。
                executor.setLeftArmControlMode(RobotControlExecutor::ArmControlMode::Servo);
                executor.setRightArmControlMode(RobotControlExecutor::ArmControlMode::Servo);
                executor.setLeftArmServoTarget(left_servo_target, left_arm_velocity);
                executor.setRightArmServoTarget(right_servo_target, right_arm_velocity);

                // 不等到目标完全到达。
                //
                // 后面测试运动中 Hold。
                std::this_thread::sleep_for(std::chrono::milliseconds{700});
                checkExecutor(executor, "Arm Servo");
                const auto moving_state = executor.latestState();
                if (!moving_state.left_arm_positions[0] || !moving_state.right_arm_positions[0])
                {
                    throw std::runtime_error("Arm feedback unavailable during Servo");
                }
                const double left_motion = absolute(*moving_state.left_arm_positions[0] - left_arm_position[0]);
                const double right_motion = absolute(*moving_state.right_arm_positions[0] - right_arm_position[0]);
                if (left_motion < 0.02)
                {
                    throw std::runtime_error("Left arm did not move");
                }
                if (right_motion < 0.02)
                {
                    throw std::runtime_error("Right arm did not move");
                }
                robot::common::logger()->info("TEST 6 PASS: Arm Servo mailbox");

                // =================================================
                // TEST 7
                // Hold
                // =================================================

                robot::common::logger()->info("TEST 7: Hold both arms while moving");
                executor.holdLeftArm();
                executor.holdRightArm();

                // 先等 Hold 生效并稳定一下。
                std::this_thread::sleep_for(std::chrono::milliseconds{300});
                checkExecutor(executor, "Arm Hold");
                const auto hold_state_1 = executor.latestState();
                if (hold_state_1.left_arm_state != ArmController::ControlState::Running || hold_state_1.right_arm_state != ArmController::ControlState::Running)
                {
                    throw std::runtime_error("ArmController is not Running during Hold");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds{700});
                checkExecutor(executor, "Arm Hold stability");
                const auto hold_state_2 = executor.latestState();
                if (!hold_state_1.left_arm_positions[0] || !hold_state_2.left_arm_positions[0] || !hold_state_1.right_arm_positions[0] ||
                    !hold_state_2.right_arm_positions[0])
                {
                    throw std::runtime_error("Arm feedback unavailable during Hold");
                }
                const double left_hold_drift = absolute(*hold_state_2.left_arm_positions[0] - *hold_state_1.left_arm_positions[0]);
                const double right_hold_drift = absolute(*hold_state_2.right_arm_positions[0] - *hold_state_1.right_arm_positions[0]);
                if (left_hold_drift > 0.03)
                {
                    throw std::runtime_error("Left arm continued moving after Hold");
                }
                if (right_hold_drift > 0.03)
                {
                    throw std::runtime_error("Right arm continued moving after Hold");
                }
                robot::common::logger()->info("TEST 7 PASS: Arm Hold mailbox");

                // =================================================
                // TEST 8
                // Arm Stop
                // =================================================

                robot::common::logger()->info("TEST 8: stopping both arms");
                executor.stopLeftArm();
                executor.stopRightArm();
                std::this_thread::sleep_for(std::chrono::milliseconds{500});
                checkExecutor(executor, "Arm Stop");
                {
                    const auto state = executor.latestState();
                    if (state.left_arm_state != ArmController::ControlState::Idle || state.right_arm_state != ArmController::ControlState::Idle)
                    {
                        throw std::runtime_error("ArmControllers did not enter Idle");
                    }
                }
                robot::common::logger()->info("TEST 8 PASS: Arm Stop mailbox");

                // =================================================
                // 11. Final result
                // =================================================

                const auto statistics = executor.statistics();
                const auto last_execution_us = std::chrono::duration_cast<std::chrono::microseconds>(statistics.last_execution_time).count();
                std::cout << "\n"
                          << "====================================\n"
                          << "FULL EXECUTOR MAILBOX TEST\n"
                          << "====================================\n"
                          << "Left gripper:     PASS\n"
                          << "Right gripper:    PASS\n"
                          << "Head:             PASS\n"
                          << "Torso:            PASS\n"
                          << "Base:             PASS\n"
                          << "Arm Servo:        PASS\n"
                          << "Arm Hold:         PASS\n"
                          << "Arm Stop:         PASS\n"
                          << "------------------------------------\n"
                          << "Cycle count: " << statistics.cycle_count << '\n'
                          << "Last execution: " << last_execution_us << " us\n"
                          << "Deadline miss: " << statistics.deadline_miss_count << '\n'
                          << "====================================\n"
                          << "RESULT: PASS\n"
                          << "====================================\n";
            }

            // =================================================
            // 12. Hardware shutdown
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
